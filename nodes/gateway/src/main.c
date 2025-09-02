#include <errno.h>
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

#define DAYS_BEFORE_AUG_2025 20301ULL
#define SECONDS_BEFORE_AUG_2025 (DAYS_BEFORE_AUG_2025 * 86400ULL)

/* 1 tick = 1 / 280,000,00 */
#define NS_PER_TICK 3.57142857143D

int init_gpios(void);
int init_lora(const struct device** lora_dev);
int init_tim2(void);
int init_sen0546(const struct device** sen0546);

struct lora_data {
    uint32_t source_id;
    uint32_t timestamp_utc;
    uint32_t ns_ticks;
    uint16_t rssi;
    uint8_t snr;
    uint8_t temp;
    uint8_t humi;
};

struct lora_data data_array[32] = {0};

struct navigation_data location;

#define LORA_MODEM DT_ALIAS(lora0)
#define GNSS_MODEM DEVICE_DT_GET(DT_ALIAS(gnss))
static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);
static const struct gpio_dt_spec sw0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec sw1 = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);

int days_in_month[] = {31,28,31,30,31,30,31,31,30,31,30,31};
uint32_t utc_timestamp = SECONDS_BEFORE_AUG_2025;
uint32_t lora_ticks = 0;
uint32_t prev_lora_ticks = 0;
uint32_t prev_pps_ticks = 0;
uint32_t pps_ticks = 0;;
uint8_t pos_hold_flag = 0;

static void gnss_cb(const struct device *dev, const struct gnss_data *data)
{
    if (data->info.fix_status == GNSS_FIX_STATUS_NO_FIX) {
        LOG_WRN("Invalid Fix...");
    } else {
        if (data->info.pos_hold == 1) {
            gpio_pin_toggle_dt(&led2);
            struct gnss_time utc = data->utc;
            uint16_t days = 0;
            uint8_t month = utc.month;
            for (int m = 8; m < month; m++) {
                days += days_in_month[m-1];
            }
            days += (data->utc.month_day - 1);
            // LOG_INF("%d %d %u %u %u", utc.month, days, utc.hour, utc.minute, utc.millisecond/1000);
            utc_timestamp = SECONDS_BEFORE_AUG_2025;
            utc_timestamp += days * 86400ULL;
            utc_timestamp += utc.hour * 3600ULL;
            utc_timestamp += utc.minute * 60ULL;
            utc_timestamp += utc.millisecond / 1000ULL;
            // LOG_INF("UTC Timestamp: %u", utc_timestamp);
            if (pos_hold_flag == 0) {
                memcpy(&location, &data->nav_hold, sizeof(struct navigation_data));
                pos_hold_flag = 1;
                LOG_INF("Hold Lat/Log: %lld / %lld", location.latitude, location.longitude);
            }
        } else {
            LOG_WRN("Taking Samples...");
        }
    }
}
GNSS_DATA_CALLBACK_DEFINE(NULL, gnss_cb);

void lora_receive_cb(const struct device *dev, uint8_t *data, uint16_t size,
		     int16_t rssi, int8_t snr, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(size);
	ARG_UNUSED(user_data);

    prev_lora_ticks = lora_ticks;
    lora_ticks = LL_TIM_IC_GetCaptureCH1(TIM2);
    LOG_INF("Difference LoRa Ticks: %u", lora_ticks - prev_lora_ticks);
    // uint32_t frac_time = floor((double)(lora_ticks - pps_ticks) * NS_PER_TICK); // nanoseconds
	LOG_INF("LoRa Timestamp: %u s, %u ticks", utc_timestamp, lora_ticks - pps_ticks);
	LOG_INF("LoRa RX RSSI: %d dBm, SNR: %d dB", rssi, snr);
	LOG_HEXDUMP_INF(data, size, "LoRa RX payload");
    // sensor_channel_get(sen0546, SENSOR_CHAN_AMBIENT_TEMP, &val);
    gpio_pin_toggle_dt(&led1);
}

int main(void)
{
    const struct device *lora_dev = NULL;
    const struct device *sen0546 = NULL;
    init_gpios();
    init_lora(&lora_dev);
    init_sen0546(&sen0546);
    init_tim2();

	lora_recv_async(lora_dev, lora_receive_cb, NULL);
    while(1) {
        int val0 = gpio_pin_get_dt(&sw0);
        int val1 = gpio_pin_get_dt(&sw1);
        gpio_pin_set_dt(&led0, (!val0 | !val1)); 
        prev_pps_ticks = pps_ticks;
        pps_ticks = LL_TIM_IC_GetCaptureCH3(TIM2);
        if (prev_pps_ticks != pps_ticks) {
            if(sensor_sample_fetch(sen0546) < 0) {
                LOG_ERR("Could not fetch sample");
            }
        }
        k_msleep(50);
    }
    return(0);
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
    gpio_pin_configure_dt(&led0, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led1, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led2, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&sw0, GPIO_INPUT | GPIO_PULL_UP);
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

