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

#include <lib/lora.h>

#define DEFAULT_RADIO_NODE DT_ALIAS(lora0)

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(edge_node);

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
	if (ret < 0) {
		LOG_ERR("LoRa config failed");
		return 0;
	}

    int count = 0;
    char num[16] = {0};
    char tx_data[9] = {0};
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


        k_msleep(5000);

        count++;
        if (count == 9999) count = 0;
	}
	return 0;
}
