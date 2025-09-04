/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sen0546

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/devicetree.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(SEN0546, CONFIG_SENSOR_LOG_LEVEL);

struct sen0546_data {
    uint16_t temp;
    uint16_t humi;
};

struct sen0546_config {
    struct i2c_dt_spec i2c;
};


static int sen0546_sample_fetch(const struct device *dev,
				      enum sensor_channel chan)
{
	const struct sen0546_config *config = dev->config;
	struct sen0546_data *data = dev->data;

    uint8_t buf[6] = {0};
    uint8_t int_addr[2] = {0x24, 0x00};
    int ret = i2c_write_dt(&config->i2c, int_addr, 2); 
    ret = i2c_read_dt(&config->i2c, buf, 6);
    data->temp = buf[0] << 8 | buf[1];
    data->humi = buf[3] << 8 | buf[4];

	return ret;
}

static int sen0546_channel_get(const struct device *dev,
				     enum sensor_channel chan,
				     struct sensor_value *val)
{
	struct sen0546_data *data = dev->data;

    int ret = 0;

    double value = 0;

    switch(chan) {
        case SENSOR_CHAN_AMBIENT_TEMP:
            value = -45.0 + 175.0 * (data->temp / 65535.0);
            break;
        case SENSOR_CHAN_HUMIDITY:
            value = 100.0 * (data->humi / 65535.0);
            break;
        default:
            return -ENOTSUP;
    }
    // val->val1 = (int32_t)value;
    // val->val2 = (value - val->val1) * 1000000;
    // LOG_INF("%u.%u Degrees", val->val1, val->val2);
    val->val1 = data->temp;
    val->val2 = data->humi;
	return ret;
}

static DEVICE_API(sensor, sen0546_api) = {
	.sample_fetch = &sen0546_sample_fetch,
	.channel_get = &sen0546_channel_get,
};

static int sen0546_init(const struct device *dev)
{
	const struct sen0546_config *cfg = dev->config;

	int ret = 0;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C dev not ready");
		return -ENODEV;
	}

    uint8_t reg[2] = {0x37, 0x81};
    uint8_t id[3] = {0x00, 0x00, 0x00}; 
    // read Manufacturer ID
    ret = i2c_write_dt(&cfg->i2c, reg, 2);
    ret = i2c_read_dt(&cfg->i2c, id, 3);

    if (ret < 0) {
        LOG_ERR("Failed to write to sensor");
        return -ENODEV;
    }

    if (id[0] != 0x59 && id[1] != 0x59 && id[2] != 0x53) {
        LOG_ERR("ID wrong");
        return -ENODEV;
    }
    LOG_INF("SEN0546 OK.");
	return ret;
}

#define SEN0546_INIT(i)          						                      \
	static struct sen0546_data sen0546_data_##i;            	              \
									                                          \
	static const struct sen0546_config sen0546_config_##i = {                 \
        .i2c = I2C_DT_SPEC_INST_GET(i)                                        \
	};								                                          \
									                                          \
	DEVICE_DT_INST_DEFINE(i, sen0546_init, NULL,		                      \
			      &sen0546_data_##i,			                              \
			      &sen0546_config_##i, POST_KERNEL,	                          \
			      CONFIG_SENSOR_INIT_PRIORITY, &sen0546_api);

DT_INST_FOREACH_STATUS_OKAY(SEN0546_INIT)
