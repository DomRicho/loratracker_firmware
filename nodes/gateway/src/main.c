#include "zephyr/pm/state.h"
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#include <lib/lora.h>

#include <stm32h7xx_ll_tim.h>
#include <stm32h7xx_ll_bus.h>
#include <stm32h7xx_ll_gpio.h>
#include <stm32h7xx_ll_rcc.h>

#define LOG_LEVEL CONFIG_LOG_LEVEL_DBG
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gateway);

#define SERIAL_NUMBER "GW0"

#define DAYS_BEFORE_AUG_2025 20301ULL
#define SECONDS_BEFORE_AUG_2025 (DAYS_BEFORE_AUG_2025 * 86400ULL)

enum cmd_type {
    CMD_LORA,
    CMD_WEATHER,
    CMD_NAV,
    CMD_TIME,
};

enum gnss_state {
    GNSS_INIT_STATE = 0,
    GNSS_NO_FIX_STATE,
    GNSS_SAMPLE_STATE, 
    GNSS_POSHOLD_STATE,
};

struct cmd {
    void *fifo_reserved;
    enum cmd_type type;
    void *data;
};

struct weather_data {
    uint16_t temp;
    uint16_t humi;
};

struct nav_data {
    struct navigation_data location;
    enum gnss_fix_status status;
    uint8_t poshold;
};

K_SEM_DEFINE(info_fill_sem, 0, 1);
K_FIFO_DEFINE(lora_tx_fifo);
K_FIFO_DEFINE(cmd_fifo);
K_MEM_SLAB_DEFINE(lora_slab, sizeof(struct lora_info), 3, 4);
K_MEM_SLAB_DEFINE(cmd_slab, sizeof(struct cmd), 15, 4);
K_MEM_SLAB_DEFINE(weather_slab, sizeof(struct weather_data), 3, 4);

static struct gpio_callback sw0_cb_data;
K_SEM_DEFINE(sw0_press_sem, 0, 1);
static struct gpio_callback sw1_cb_data;
K_SEM_DEFINE(sw1_press_sem, 0, 1);

int init_gpios(void);
int init_lora(const struct device** lora_dev);
int init_tim2(void);
int init_sen0546(const struct device** sen0546);
int print_command(char *talker_id, void *data, enum cmd_type data_type);

#define LORA_MODEM DT_ALIAS(lora0)
#define GNSS_MODEM DEVICE_DT_GET(DT_ALIAS(gnss))
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
static const struct gpio_dt_spec sw0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec sw1 = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);

int days_in_month[] = {31,28,31,30,31,30,31,31,30,31,30,31};
uint32_t utc_timestamp = SECONDS_BEFORE_AUG_2025;
struct navigation_data location;
struct cmd nav_data = { .type = CMD_NAV, .data = &location };
uint32_t lora_ticks = 0;
uint32_t prev_lora_ticks = 0;
uint32_t prev_pps_ticks = 0;
uint32_t pps_ticks = 0;
uint8_t pos_hold_flag = 0;
uint8_t sample_flag = 0;

enum gnss_state gnss_state = GNSS_INIT_STATE;

static void gnss_cb(const struct device *dev, const struct gnss_data *data)
{
    prev_pps_ticks = pps_ticks;
    pps_ticks = LL_TIM_IC_GetCaptureCH3(TIM2);
    switch(gnss_state) {
        case GNSS_INIT_STATE:
            if (data->info.fix_status == GNSS_FIX_STATUS_NO_FIX) {
                LOG_WRN("Invalid Fix...");
                gnss_state = GNSS_NO_FIX_STATE;
            } else {
                LOG_WRN("Sampling...");
                gnss_state = GNSS_SAMPLE_STATE;
            }
            break;
        case GNSS_NO_FIX_STATE:
            if (data->info.fix_status != GNSS_FIX_STATUS_NO_FIX) {
                LOG_WRN("Sampling...");
                gnss_state = GNSS_SAMPLE_STATE;
            }
            break;
        case GNSS_SAMPLE_STATE:
            if (data->info.pos_hold == 1) {
                memcpy(&location, &data->nav_hold, sizeof(struct navigation_data));
                k_fifo_put(&cmd_fifo, &nav_data);
                LOG_INF("Sampling Done.");
                gnss_state = GNSS_POSHOLD_STATE;
            }
            break;
        case GNSS_POSHOLD_STATE:
            gpio_pin_toggle_dt(&led2);
            struct gnss_time utc = data->utc;
            uint16_t days = 0;
            uint8_t month = utc.month;
            for (int m = 8; m < month; m++) {
                days += days_in_month[m-1];
            }
            days += (data->utc.month_day - 1);
            utc_timestamp = SECONDS_BEFORE_AUG_2025;
            utc_timestamp += days * 86400ULL;
            utc_timestamp += utc.hour * 3600ULL;
            utc_timestamp += utc.minute * 60ULL;
            utc_timestamp += utc.millisecond / 1000ULL;
            break;
        default:
            break;
    }
}
GNSS_DATA_CALLBACK_DEFINE(NULL, gnss_cb);

int main(void)
{
    init_gpios();
    struct lora_tx get_nav_an0 = {
        .data = "$GW0GETAN0NAV", 
        .size = strlen("$GW0GETAN0NAV"),
    };
    struct lora_tx get_nav_an1 = {
        .data = "$GW0GETAN1NAV", 
        .size = strlen("$GW0GETAN1NAV"),
    };
    while(1) {
        if (k_sem_take(&sw0_press_sem, K_MSEC(100)) == 0) {
            k_fifo_put(&lora_tx_fifo, &get_nav_an0);
            k_fifo_put(&cmd_fifo, &nav_data);
        }
        if (k_sem_take(&sw1_press_sem, K_MSEC(100)) == 0) {
            k_fifo_put(&lora_tx_fifo, &get_nav_an1);
        }
    }
    return(0);
}


void lora_info_recv_cb(const struct device *dev, uint8_t *data, uint16_t size,
		     int16_t rssi, int8_t snr, void *user_data)
{
	ARG_UNUSED(dev);
    struct lora_info *info = (struct lora_info*)user_data;
    lora_ticks = LL_TIM_IC_GetCaptureCH1(TIM2);
    info->utc = utc_timestamp;
    info->ticks = lora_ticks - pps_ticks;
    info->rssi = rssi;
    info->snr = snr;
    memcpy(info->data, data, size);
    info->size = size;
    gpio_pin_toggle_dt(&led1);
    lora_recv_async(dev, NULL, NULL);
    k_sem_give(&info_fill_sem);
}

void lora_transceiver(void)
{
    const struct device *lora_dev = NULL;
    init_tim2();
    init_lora(&lora_dev);
    struct lora_info *info = NULL;
    struct cmd *lora_cmd; 
    struct lora_tx *tx_data = NULL;
    uint8_t tmp[255];
    uint8_t size = 255;
    int16_t rssi = 0;
    int8_t snr = 0;
    while (1) {
        if (info == NULL) {
            k_mem_slab_alloc(&lora_slab, (void **)&info, K_FOREVER);
            lora_recv_async(lora_dev, lora_info_recv_cb, (void *)info);
        }

        if (k_sem_take(&info_fill_sem, K_MSEC(1)) == 0) {
            if (strncmp(info->data, "$EN0", 3) == 0) {
                if (lora_cmd == NULL) {
                    k_mem_slab_alloc(&cmd_slab, (void **)&lora_cmd, K_FOREVER);
                } else {
                    k_mem_slab_free(&lora_slab, lora_cmd->data);
                }
                lora_cmd->type = CMD_LORA;
                lora_cmd->data = (void*)info; 
                k_fifo_put(&cmd_fifo, lora_cmd);
                struct lora_modem_config lora_cfg = DEFAULT_LORA_CFG;
                lora_cfg.tx = true;
                lora_config(lora_dev, &lora_cfg);
                lora_send(lora_dev, "$GW0GETAN0LORA", strlen("$GW0GETAN0LORA"));
                lora_cfg.tx = false;
                lora_config(lora_dev, &lora_cfg);
                size = lora_recv(lora_dev, tmp, 255, K_FOREVER, &rssi, &snr);
                tmp[size] = '\0';
                printk("%s", tmp);
                k_msleep(1);
                lora_cfg.tx = true;
                lora_config(lora_dev, &lora_cfg);
                lora_send(lora_dev, "$GW0GETAN1LORA", strlen("$GW0GETAN1LORA"));
                lora_cfg.tx = false;
                lora_config(lora_dev, &lora_cfg);
                size = lora_recv(lora_dev, tmp, 255, K_FOREVER, &rssi, &snr);
                tmp[size] = '\0';
                printk("%s", tmp);
                info = NULL;
            } else if (strncmp(info->data, "$AN", 3) == 0) {
                printk("%s", info->data);
                k_mem_slab_free(&lora_slab, info);
                info = NULL;
            } else {
                k_mem_slab_free(&lora_slab, info);
                info = NULL;
            }
            if (info == NULL) {
                lora_recv_async(lora_dev, NULL, NULL);
            }
        } else {
            tx_data = (struct lora_tx *)k_fifo_get(&lora_tx_fifo, K_MSEC(10));
            if (tx_data != NULL) {
                lora_recv_async(lora_dev, NULL, NULL);
                struct lora_modem_config lora_cfg = DEFAULT_LORA_CFG;
                lora_cfg.tx = true;
                if (lora_config(lora_dev, &lora_cfg) < 0) {
                    LOG_ERR("LoRa config failed");
                    break;
                }
                lora_send(lora_dev, tx_data->data, tx_data->size);
                lora_cfg.tx = false;
                if (lora_config(lora_dev, &lora_cfg) < 0) {
                    LOG_ERR("LoRa config failed");
                    break;
                }
                if (info == NULL) {
                    k_mem_slab_alloc(&lora_slab, (void **)&info, K_FOREVER);
                }
                lora_recv_async(lora_dev, lora_info_recv_cb, (void *)info);
            }
        }
    }
}
#define LORA_THREAD_SIZE 1024
#define LORA_THREAD_PRIO 5
K_THREAD_DEFINE(lora_tid, LORA_THREAD_SIZE, lora_transceiver, NULL, NULL, NULL, 
        LORA_THREAD_PRIO, 0, 0);

void collect_weather_thread(void)
{
    const struct device *sen0546 = NULL;
    struct sensor_value weather; 
    struct cmd *weather_cmd; 
    struct weather_data *data; 
    init_sen0546(&sen0546);
    while(1) {
        k_mem_slab_alloc(&weather_slab, (void **)&data, K_FOREVER);
        k_mem_slab_alloc(&cmd_slab, (void **)&weather_cmd, K_FOREVER);
        weather_cmd->type = CMD_WEATHER;
        if(sensor_sample_fetch(sen0546) < 0) {
            LOG_ERR("Could not fetch sample");
        }
        if (sensor_channel_get(sen0546, SENSOR_CHAN_AMBIENT_TEMP, &weather) < 0) 
        {
            LOG_ERR("Could not get channel");
        }
        data->temp = (uint16_t)weather.val1;
        data->humi = (uint16_t)weather.val2; 
        weather_cmd->data = (void*)data; 
        k_fifo_put(&cmd_fifo, weather_cmd);
        k_msleep(1000);
    }
}
#define WEATHER_THREAD_SIZE 1024
#define WEATHER_THREAD_PRIO 5
K_THREAD_DEFINE(weather_tid, WEATHER_THREAD_SIZE, collect_weather_thread, NULL, 
        NULL, NULL, WEATHER_THREAD_PRIO, 0, 0);

/*
 * $GW0LORA,93,9,18000000000,1600000000*5c\r\n
 */
int send_command(void) 
{
    struct cmd *cmd;
    char cmd_str[255];
    uint8_t ret = 0;
    uint8_t checksum = 0;
    // struct lora_tx *lora_tx = k_malloc(sizeof(struct lora_tx));; 
    while(1) {
        cmd = k_fifo_get(&cmd_fifo, K_FOREVER);
        int len = 0;
        cmd_str[len++] = '$';
        strncpy(&cmd_str[len], SERIAL_NUMBER, 4);
        len += 3;
        switch(cmd->type) {
            case CMD_LORA: ;
                struct lora_info *info = (struct lora_info*)cmd->data;
                ret = snprintk(&cmd_str[len], 127 - len, 
                        "LORA,%.8s,%d,%d,%u,%u*", 
                        info->data, info->rssi, info->snr, info->utc, info->ticks
                    );
                len += ret;
                k_mem_slab_free(&lora_slab, info);
                break;
            case CMD_NAV: ;
                struct navigation_data *nav_data = (struct navigation_data*)cmd->data;
                ret = snprintk(&cmd_str[len], 127 - len, 
                        "POS,%lld,%lld,%d*", 
                        nav_data->longitude, nav_data->latitude, nav_data->altitude
                    );
                len += ret;
                break;
            case CMD_WEATHER: ;
                struct weather_data *wthr_data = (struct weather_data*)cmd->data;
                ret = snprintk(&cmd_str[len], 127 - len, 
                        "WTHR,%u,%u*", 
                        wthr_data->temp, wthr_data->humi
                    );
                len += ret;
                k_mem_slab_free(&weather_slab, wthr_data);
                break;
            case CMD_TIME:
                ret = snprintk(&cmd_str[len], 127 - len, "TIME,");
                len += ret;
                uint32_t *time = (uint32_t*)cmd->data;
                ret = snprintk(&cmd_str[len], 127 - len, "%u*", *time);
                len += ret;
                break;
            default:
                return -ENOTSUP;
        }
        checksum = 0;
        for (int i = 1; i < len - 1; i++) {
            checksum ^= (uint8_t)cmd_str[i]; 
        }
        ret = snprintk(&cmd_str[len], 127 - len, "%02x", checksum);
        len += ret;
        strncpy(&cmd_str[len], "\r\n", 3);
        len += 3;
        printk("%s", cmd_str);
        // strncpy(lora_tx->data, cmd_str, 255);
        // lora_tx->size = len;
        // k_fifo_put(&lora_tx_fifo, lora_tx);
        k_mem_slab_free(&cmd_slab, cmd);
    }
    return 0;
}
#define CMD_THREAD_SIZE 1024
#define CMD_THREAD_PRIO 5
K_THREAD_DEFINE(cmd_tid, CMD_THREAD_SIZE, send_command, NULL, NULL, NULL, 
        CMD_THREAD_PRIO, 0, 0);

void sw0_pressed(const struct device *dev, struct gpio_callback *cb,
            uint32_t pins)
{
    k_sem_give(&sw0_press_sem);
}

void sw1_pressed(const struct device *dev, struct gpio_callback *cb,
            uint32_t pins)
{
    k_sem_give(&sw1_press_sem);
}

int init_gpios(void)
{
    if (!device_is_ready(led0.port) ||
        !device_is_ready(led1.port) ||
        !device_is_ready(led2.port) ||
        !device_is_ready(sw0.port)  ||
        !device_is_ready(sw1.port)) {
        printk("Error: one or more devices not ready\n");
        return -1;
    }
    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&sw0, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure_dt(&sw0, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&sw0_cb_data, sw0_pressed, BIT(sw0.pin));
    gpio_add_callback(sw0.port, &sw0_cb_data);
    gpio_pin_interrupt_configure_dt(&sw1, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&sw1_cb_data, sw1_pressed, BIT(sw1.pin));
    gpio_add_callback(sw1.port, &sw1_cb_data);
    gpio_pin_configure_dt(&sw1, GPIO_INPUT | GPIO_PULL_UP);
    return 0;
}

int init_lora(const struct device **lora_dev)
{
	*lora_dev = DEVICE_DT_GET(LORA_MODEM);
    struct lora_modem_config lora_cfg = DEFAULT_LORA_CFG;
	if (!device_is_ready(*lora_dev)) {
		LOG_ERR("%s Device not ready", (*lora_dev)->name);
		return -1;
	}
	if (lora_config(*lora_dev, &lora_cfg) < 0) {
		LOG_ERR("LoRa config failed");
		return -1;
	}
    return 0;
}

int init_sen0546(const struct device** sen0546)
{
    *sen0546 = DEVICE_DT_GET(DT_NODELABEL(sen0546));
    if (!device_is_ready(*sen0546)) {
        LOG_ERR("Sensor not ready");
        return -1;
    }
    return 0;
}

int init_tim2(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOA);
    LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);

    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_15, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_15, LL_GPIO_AF_1);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_15, LL_GPIO_PULL_DOWN);

    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_10, LL_GPIO_AF_1);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_10, LL_GPIO_PULL_DOWN);

    /* Configure TIM2 CH1 for input capture on rising edge */
    LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
    LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);

    LL_TIM_IC_SetActiveInput(TIM2, LL_TIM_CHANNEL_CH3, LL_TIM_ACTIVEINPUT_DIRECTTI);
    LL_TIM_IC_SetPrescaler(TIM2, LL_TIM_CHANNEL_CH3, LL_TIM_ICPSC_DIV1);
    LL_TIM_IC_SetFilter(TIM2, LL_TIM_CHANNEL_CH3, LL_TIM_IC_FILTER_FDIV1);
    LL_TIM_IC_SetPolarity(TIM2, LL_TIM_CHANNEL_CH3, LL_TIM_IC_POLARITY_RISING);

    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM2, LL_TIM_CHANNEL_CH3);

    LL_TIM_EnableCounter(TIM2);
    return 0;
}

