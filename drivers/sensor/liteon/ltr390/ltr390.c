#define DT_DRV_COMPAT liteon_ltr390

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include "ltr390.h"

LOG_MODULE_REGISTER(LTR390, CONFIG_SENSOR_LOG_LEVEL);

/**
 * @brief Write a value to a sensor register over I2C.
 */
static int ltr390_write_reg(const struct i2c_dt_spec *bus, uint8_t reg, uint8_t value)
{
	uint8_t buf[2] = {reg, value};
	return i2c_write_dt(bus, buf, sizeof(buf));
}

/**
 * @brief Read a value from a sensor register over I2C.
 */
static int ltr390_read_reg(const struct i2c_dt_spec *bus, uint8_t reg, uint8_t *value)
{
	return i2c_write_read_dt(bus, &reg, sizeof(reg), value, 1);
}

/**
 * @brief Initialize sensor registers: MAIN_CTRL, MEAS_RATE, and ALS_UVS_GAIN.
 */
static int ltr390_init_regs(const struct ltr390_config *cfg)
{
	int ret;
	uint8_t ctrl_val = 0;

	/* Set MAIN_CTRL: enable sensor and select UVS mode if configured */
	if (cfg->uvs_mode) {
		ctrl_val |= LTR390_MAIN_CTRL_UVS_MODE_MASK;
	}
	ctrl_val |= LTR390_MAIN_CTRL_ENABLE_MASK;
	ret = ltr390_write_reg(&cfg->bus, LTR390_REG_MAIN_CTRL, ctrl_val);
	if (ret < 0) {
		LOG_ERR("Failed to write MAIN_CTRL");
		return ret;
	}

	uint8_t meas_val = LTR390_REG_SET(LTR390_REG_MEAS_RATE, RES, cfg->resolution) |
			   LTR390_REG_SET(LTR390_REG_MEAS_RATE, RATE, cfg->meas_rate);
	ret = ltr390_write_reg(&cfg->bus, LTR390_REG_MEAS_RATE, meas_val);
	if (ret < 0) {
		LOG_ERR("Failed to write MEAS_RATE");
		return ret;
	}

	uint8_t gain_val = LTR390_REG_SET(LTR390_REG_ALS_UVS_GAIN, ALS_UVS_GAIN, cfg->gain);
	ret = ltr390_write_reg(&cfg->bus, LTR390_REG_ALS_UVS_GAIN, gain_val);
	if (ret < 0) {
		LOG_ERR("Failed to write ALS_UVS_GAIN");
		return ret;
	}

	return 0;
}

static int ltr390_init(const struct device *dev)
{
	const struct ltr390_config *cfg = dev->config;
	struct ltr390_data *data = dev->data;
	int ret;
	uint8_t part_id;

	if (!i2c_is_ready_dt(&cfg->bus)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	/* Wait for sensor startup */
	k_sleep(K_MSEC(100));

	/* Perform a software reset */
	ret = ltr390_write_reg(&cfg->bus, LTR390_REG_MAIN_CTRL, LTR390_MAIN_CTRL_SW_RESET_MASK);
	if (ret < 0) {
		LOG_ERR("Software reset failed");
		return ret;
	}
	k_sleep(K_MSEC(10));

	/* Verify PART_ID */
	ret = ltr390_read_reg(&cfg->bus, LTR390_REG_PART_ID, &part_id);
	if (ret < 0) {
		LOG_ERR("Failed to read PART_ID");
		return ret;
	}
	if (part_id != LTR390_PART_ID_EXPECTED) {
		LOG_ERR("PART_ID mismatch: expected 0x%02X, got 0x%02X", LTR390_PART_ID_EXPECTED,
			part_id);
		return -ENODEV;
	}

	/* Configure sensor registers */
	ret = ltr390_init_regs(cfg);
	if (ret < 0) {
		return ret;
	}

	LOG_INF("LTR390 initialized (mode: %s)", cfg->uvs_mode ? "UVS" : "ALS");
	data->raw_als = 0;
	data->raw_uvs = 0;
	return 0;
}

static int ltr390_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct ltr390_config *cfg = dev->config;
	struct ltr390_data *data = dev->data;
	int ret;
	uint8_t status;

	ret = ltr390_read_reg(&cfg->bus, LTR390_REG_MAIN_STATUS, &status);
	if (ret < 0) {
		LOG_ERR("Failed to read MAIN_STATUS");
		return ret;
	}

	if (!(status & LTR390_MAIN_STATUS_DATA_MASK)) {
		LOG_WRN("Data not ready");
		return -EBUSY;
	}

	if (cfg->uvs_mode) {
		uint8_t buf[3];
		ret = i2c_burst_read_dt(&cfg->bus, LTR390_REG_UVS_DATA_0, buf, sizeof(buf));
		if (ret < 0) {
			LOG_ERR("Failed to read UVS data");
			return ret;
		}
		data->raw_uvs =
			(((uint32_t)buf[2] & 0x0F) << 16) | ((uint32_t)buf[1] << 8) | buf[0];
	} else {
		uint8_t buf[3];
		ret = i2c_burst_read_dt(&cfg->bus, LTR390_REG_ALS_DATA_0, buf, sizeof(buf));
		if (ret < 0) {
			LOG_ERR("Failed to read ALS data");
			return ret;
		}
		data->raw_als =
			(((uint32_t)buf[2] & 0x0F) << 16) | ((uint32_t)buf[1] << 8) | buf[0];
	}
	return 0;
}

static int ltr390_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	const struct ltr390_config *cfg = dev->config;
	struct ltr390_data *data = dev->data;

	if (cfg->uvs_mode) {
		if (chan != SENSOR_CHAN_UV && chan != SENSOR_CHAN_ALL) {
			return -ENOTSUP;
		}
		val->val1 = data->raw_uvs;
		val->val2 = 0;
	} else {
		if (chan != SENSOR_CHAN_LIGHT && chan != SENSOR_CHAN_ALL) {
			return -ENOTSUP;
		}
		val->val1 = data->raw_als;
		val->val2 = 0;
	}
	return 0;
}

static const struct sensor_driver_api ltr390_driver_api = {
	.sample_fetch = ltr390_sample_fetch,
	.channel_get = ltr390_channel_get,
#if defined(CONFIG_LTR390_TRIGGER)
	.trigger_set = ltr390_trigger_set,
#endif
};

#define DEFINE_LTR390(inst)                                                                        \
	static struct ltr390_data ltr390_data_##inst;                                              \
	static const struct ltr390_config ltr390_config_##inst = {                                 \
		.bus = I2C_DT_SPEC_INST_GET(inst),                                                 \
		.uvs_mode = DT_INST_PROP(inst, uvs_mode),                                          \
		.resolution = DT_INST_PROP(inst, resolution),                                      \
		.meas_rate = DT_INST_PROP(inst, meas_rate),                                        \
		.gain = DT_INST_PROP(inst, gain),                                                  \
		IF_ENABLED(CONFIG_LTR390_TRIGGER, (.int_gpio =                       \
             GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, { 0 }))) };            \
	DEVICE_DT_INST_DEFINE(inst, ltr390_init, NULL, &ltr390_data_##inst, &ltr390_config_##inst, \
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &ltr390_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DEFINE_LTR390)
