/*
 * Copyright (c) 2025 Konrad Sikora
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT liteon_ltr390

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/sensor.h>
#include "ltr390.h"

LOG_MODULE_REGISTER(LTR390, CONFIG_SENSOR_LOG_LEVEL);

static int ltr390_check_device_id(const struct i2c_dt_spec *bus)
{
	uint8_t part_id;
	int ret;

	ret = i2c_reg_read_byte_dt(bus, LTR390_REG_PART_ID, &part_id);
	if (ret < 0) {
		LOG_ERR("Failed to read PART_ID");
		return ret;
	}

	if (part_id != LTR390_PART_ID_EXPECTED) {
		LOG_ERR("PART_ID mismatch: expected 0x%02X, got 0x%02X", LTR390_PART_ID_EXPECTED,
			part_id);
		return -ENODEV;
	}

	return 0;
}

static int ltr390_init_registers(const struct ltr390_config *cfg)
{
	const struct i2c_dt_spec *bus = &cfg->bus;
	int ret;
	uint8_t ctrl_val = 0;

	/* MAIN_CTRL: enable sensor and select UVS mode if configured */
	if (cfg->uvs_mode) {
		ctrl_val |= LTR390_REG_SET(LTR390_REG_MAIN_CTRL, UVS_MODE, 1U);
	}
	ctrl_val |= LTR390_REG_SET(LTR390_REG_MAIN_CTRL, ENABLE, 1U);
	ret = i2c_reg_write_byte_dt(bus, LTR390_REG_MAIN_CTRL, ctrl_val);
	if (ret < 0) {
		LOG_ERR("Failed to set MAIN_CTRL register");
		return ret;
	}

	/* Set MEAS_RATE: resolution and measurement rate */
	uint8_t meas_val = LTR390_REG_SET(LTR390_REG_MEAS_RATE, RES, cfg->resolution) |
			   LTR390_REG_SET(LTR390_REG_MEAS_RATE, RATE, cfg->meas_rate);
	ret = i2c_reg_write_byte_dt(bus, LTR390_REG_MEAS_RATE, meas_val);
	if (ret < 0) {
		LOG_ERR("Failed to set MEAS_RATE register");
		return ret;
	}

	/* Set ALS_UVS_GAIN register */
	uint8_t gain_val = LTR390_REG_SET(LTR390_REG_ALS_UVS_GAIN, RANGE, cfg->gain);
	ret = i2c_reg_write_byte_dt(bus, LTR390_REG_ALS_UVS_GAIN, gain_val);
	if (ret < 0) {
		LOG_ERR("Failed to set ALS_UVS_GAIN register");
		return ret;
	}

	return 0;
}

static int ltr390_check_data_ready(const struct i2c_dt_spec *bus)
{
	uint8_t status;
	int ret = i2c_reg_read_byte_dt(bus, LTR390_REG_MAIN_STATUS, &status);
	if (ret < 0) {
		LOG_ERR("Failed to read MAIN_STATUS register");
		return ret;
	}

	if (!(status & LTR390_REG_MAIN_STATUS_DATA_MASK)) {
		LOG_WRN("Data not ready");
		return -EBUSY;
	}

	return 0;
}

static int ltr390_read_sensor_data(const struct i2c_dt_spec *bus, bool uvs_mode, uint32_t *raw_data)
{
	uint8_t buf[3];
	int ret;

	if (uvs_mode) {
		ret = i2c_burst_read_dt(bus, LTR390_REG_UVS_DATA_0, buf, sizeof(buf));
	} else {
		ret = i2c_burst_read_dt(bus, LTR390_REG_ALS_DATA_0, buf, sizeof(buf));
	}
	if (ret < 0) {
		LOG_ERR("Failed to read %s data", uvs_mode ? "UVS" : "ALS");
		return ret;
	}

	*raw_data = (((uint32_t)(buf[2] & 0x0F)) << 16) | ((uint32_t)buf[1] << 8) | buf[0];
	return 0;
}

static int ltr390_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct ltr390_config *cfg = dev->config;
	struct ltr390_data *data = dev->data;
	int ret;

	/* Verify channel is supported */
	//TODO: UV
	if ((chan != SENSOR_CHAN_ALL) && ((cfg->uvs_mode && (chan != SENSOR_CHAN_LIGHT)) ||
					  (!cfg->uvs_mode && (chan != SENSOR_CHAN_LIGHT)))) {
		return -ENOTSUP;
	}

	ret = ltr390_check_data_ready(&cfg->bus);
	if (ret < 0) {
		return ret;
	}

	if (cfg->uvs_mode) {
		ret = ltr390_read_sensor_data(&cfg->bus, true, &data->raw_uvs);
	} else {
		ret = ltr390_read_sensor_data(&cfg->bus, false, &data->raw_als);
	}

	return ret;
}

static int ltr390_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	const struct ltr390_config *cfg = dev->config;
	struct ltr390_data *data = dev->data;

	if (cfg->uvs_mode) {
		// if ((chan != SENSOR_CHAN_UV) && (chan != SENSOR_CHAN_ALL)) 
		{
			return -ENOTSUP;
		}
		/* Return raw UVS value */
		val->val1 = data->raw_uvs;
		val->val2 = 0;
	} else {
		if ((chan != SENSOR_CHAN_LIGHT) && (chan != SENSOR_CHAN_ALL)) {
			return -ENOTSUP;
		}
		/* Return raw ALS value */
		val->val1 = data->raw_als;
		val->val2 = 0;
	}
	return 0;
}

static int ltr390_init(const struct device *dev)
{
	const struct ltr390_config *cfg = dev->config;
	struct ltr390_data *data = dev->data;
	int ret;

	if (!i2c_is_ready_dt(&cfg->bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	/* Wait for sensor startup */
	k_sleep(K_MSEC(100));

	ret = ltr390_check_device_id(&cfg->bus);
	if (ret < 0) {
		return ret;
	}

	ret = ltr390_init_registers(cfg);
	if (ret < 0) {
		return ret;
	}

	LOG_INF("LTR390 initialized (mode: %s)", cfg->uvs_mode ? "UVS" : "ALS");

	/* Initialize sensor raw data */
	data->raw_als = 0;
	data->raw_uvs = 0;

	return 0;
}

static const struct sensor_driver_api ltr390_driver_api = {
	.sample_fetch = ltr390_sample_fetch,
	.channel_get = ltr390_channel_get,
#if defined(CONFIG_LTR390_TRIGGER)
	// .trigger_set = ltr390_trigger_set,
#endif
};

#define DEFINE_LTR390(inst)                                                                        \
	static struct ltr390_data ltr390_data_##inst;                                              \
	static const struct ltr390_config ltr390_config_##inst = {                                 \
		.bus = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.uvs_mode = DT_INST_PROP(inst, uvs_mode),                                          \
		.resolution = DT_INST_PROP(inst, resolution),                                      \
		.meas_rate = DT_INST_PROP(inst, measurement_rate),                                        \
		.gain = DT_INST_PROP(inst, gain),                                                  \
		IF_ENABLED(CONFIG_LTR390_TRIGGER, (.int_gpio =                  \
			GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, { 0 }))) };            \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, ltr390_init, NULL, &ltr390_data_##inst,                 \
				     &ltr390_config_##inst, POST_KERNEL,                           \
				     CONFIG_SENSOR_INIT_PRIORITY, &ltr390_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_LTR390)
