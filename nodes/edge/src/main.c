/*
 * Copyright (c) 2019 Manivannan Sadhasivam
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/lora.h>
#include <errno.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include <lib/lora.h>

#define DEFAULT_RADIO_NODE DT_ALIAS(lora0)

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(edge_node);

static const struct gpio_dt_spec sw0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

int main(void)
{
	const struct device *const lora_dev = DEVICE_DT_GET(DEFAULT_RADIO_NODE);
	struct lora_modem_config config = DEFAULT_LORA_CFG;
    config.tx = true;
    config.tx = 15;
	int ret;

	if (!device_is_ready(lora_dev)) {
		LOG_ERR("%s Device not ready", lora_dev->name);
		return 0;
	}

	ret = lora_config(lora_dev, &config);
    gpio_pin_configure_dt(&sw0, GPIO_INPUT | GPIO_PULL_UP);
	if (ret < 0) {
		LOG_ERR("LoRa config failed");
		return 0;
	}

    int count = 0;
    char num[16] = {0};
    char tx_data[9] = {0};
    int64_t last_timestamp = k_uptime_get();
	while (1) {
        snprintf(num, 16,"$EN0%04d", count);
        memcpy(tx_data, num, 8);
        tx_data[8] = '\0';
		ret = lora_send(lora_dev, tx_data, 8);
		if (ret < 0) {
			LOG_ERR("LoRa send failed");
			return 0;
		}
		LOG_INF("Data sent %s!", tx_data);
        while(1) {
            if (gpio_pin_get_dt(&sw0) > 0) {
                break;
            }
            if (k_uptime_get() - last_timestamp >= 60000) {
                break;
            }
            k_msleep(10);
        }
        last_timestamp = k_uptime_get();
        count++;
        if (count == 9999) count = 0;
        k_msleep(1000);
	}
	return 0;
}
