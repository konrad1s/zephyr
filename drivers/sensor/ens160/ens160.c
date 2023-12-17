/*
 * Copyright (c) 2024 Konrad Sikora
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ams_ens160

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>

#include "ens160.h"

#define ENS160_REG_PART_ID       0x00
#define ENS160_REG_OPMODE        0x10
#define ENS160_REG_CONFIG        0x11
#define ENS160_REG_COMMAND       0x12
#define ENS160_REG_TEMP_IN       0x13
#define ENS160_REG_RH_IN         0x15
#define ENS160_REG_DEVICE_STATUS 0x20
#define ENS160_REG_DATA_AQI      0x21
#define ENS160_REG_DATA_TVOC     0x22
#define ENS160_REG_DATA_ECO2     0x24
#define ENS160_REG_DATA_ETOH     0x22
#define ENS160_REG_DATA_T        0x30
#define ENS160_REG_DATA_RH       0x32
#define ENS160_REG_DATA_MISR     0x38
#define ENS160_REG_GPR_WRITE     0x40
#define ENS160_REG_GPR_READ      0x48

#define ENS160_PART_ID_LSB_MASK GENMASK(7, 0)
#define ENS160_PART_ID_LSB_SHIFT 0
#define ENS160_PART_ID_MSB_MASK GENMASK(15, 8)
#define ENS160_PART_ID_MSB_SHIFT 8

#define ENS160_OPMODE_MODE_MASK      GENMASK(7, 0)
#define ENS160_OPMODE_MODE_SHIFT     0

#define ENS160_CONFIG_INTEN_MASK      BIT(0)
#define ENS160_CONFIG_INTEN_SHIFT     0
#define ENS160_CONFIG_INTDAT_MASK     BIT(1)
#define ENS160_CONFIG_INTDAT_SHIFT    1
#define ENS160_CONFIG_INTGPR_MASK     BIT(3)
#define ENS160_CONFIG_INTGPR_SHIFT    3
#define ENS160_CONFIG_INT_CFG_MASK    BIT(5)
#define ENS160_CONFIG_INT_CFG_SHIFT   5
#define ENS160_CONFIG_INTPOL_MASK     BIT(6)
#define ENS160_CONFIG_INTPOL_SHIFT    6

#define ENS160_COMMAND_MASK     GENMASK(7, 0)
#define ENS160_COMMAND_SHIFT    0

#define ENS160_TEMP_IN_LSB_MASK GENMASK(7, 0)
#define ENS160_TEMP_IN_LSB_SHIFT 0
#define ENS160_TEMP_IN_MSB_MASK GENMASK(15, 8)
#define ENS160_TEMP_IN_MSB_SHIFT 8

#define ENS160_RH_IN_LSB_MASK GENMASK(7, 0)
#define ENS160_RH_IN_LSB_SHIFT 0
#define ENS160_RH_IN_MSB_MASK GENMASK(15, 8)
#define ENS160_RH_IN_MSB_SHIFT 8

#define ENS160_DEVICE_STATUS_NEWGPR_MASK BIT(0)
#define ENS160_DEVICE_STATUS_NEWGPR_SHIFT 0
#define ENS160_DEVICE_STATUS_NEWDAT_MASK BIT(1)
#define ENS160_DEVICE_STATUS_NEWDAT_SHIFT 1
#define ENS160_DEVICE_STATUS_VALIDITY_FLAG_MASK GENMASK(3, 2)
#define ENS160_DEVICE_STATUS_VALIDITY_FLAG_SHIFT 2
#define ENS160_DEVICE_STATUS_STATER_MASK BIT(6)
#define ENS160_DEVICE_STATUS_STATER_SHIFT 6
#define ENS160_DEVICE_STATUS_STATAS_MASK BIT(7)
#define ENS160_DEVICE_STATUS_STATAS_SHIFT 7

#define ENS160_DATA_AQI_UBA_MASK GENMASK(2, 0)
#define ENS160_DATA_AQI_UBA_SHIFT 0

#define ENS160_DATA_TVOC_LSB_MASK GENMASK(7, 0)
#define ENS160_DATA_TVOC_LSB_SHIFT 0
#define ENS160_DATA_TVOC_MSB_MASK GENMASK(15, 8)
#define ENS160_DATA_TVOC_MSB_SHIFT 8

#define ENS160_DATA_ECO2_LSB_MASK GENMASK(7, 0)
#define ENS160_DATA_ECO2_LSB_SHIFT 0
#define ENS160_DATA_ECO2_MSB_MASK GENMASK(15, 8)
#define ENS160_DATA_ECO2_MSB_SHIFT 8

#define ENS160_DATA_ETOH_LSB_MASK GENMASK(7, 0)
#define ENS160_DATA_ETOH_LSB_SHIFT 0
#define ENS160_DATA_ETOH_MSB_MASK GENMASK(15, 8)
#define ENS160_DATA_ETOH_MSB_SHIFT 8

#define ENS160_DATA_T_LSB_MASK GENMASK(7, 0)
#define ENS160_DATA_T_LSB_SHIFT 0
#define ENS160_DATA_T_MSB_MASK GENMASK(15, 8)
#define ENS160_DATA_T_MSB_SHIFT 8

#define ENS160_DATA_RH_LSB_MASK GENMASK(7, 0)
#define ENS160_DATA_RH_LSB_SHIFT 0
#define ENS160_DATA_RH_MSB_MASK GENMASK(15, 8)
#define ENS160_DATA_RH_MSB_SHIFT 8

#define ENS160_DATA_MISR_MASK GENMASK(7, 0)
#define ENS160_DATA_MISR_SHIFT 0

/* ---------- */
#define ENS160_PART_ID 0x1600

LOG_MODULE_REGISTER(ENS160, CONFIG_SENSOR_LOG_LEVEL);

static int ens160_init(const struct device *dev)
{
	struct ens160_data *data = dev->data;
	const struct ens160_config *config = dev->config;
	const struct ens160_bus *bus = &config->bus;

	if (config->bus_init(dev) < 0) {
		LOG_ERR("Failed to initialize bus");
		return -EIO;
	}

	uint8_t part_id[2];
	int rc = config->bus_io->read(bus, ENS160_REG_PART_ID, part_id, sizeof(part_id));
	if (rc < 0) {
		LOG_ERR("Failed to read part id");
		return rc;
	}

	uint16_t part_id_val = sys_get_be16(part_id);
	if (part_id_val != ENS160_PART_ID) {
		LOG_ERR("Invalid part id");
		return -ENODEV;
	}

	return 0;
}


static int ens160_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct ens160_data *data = dev->data;
	const struct ens160_config *config = dev->config;
	const struct ens160_bus *bus = &config->bus;

	uint8_t device_status;
	int rc = config->bus_io->read(bus, ENS160_REG_DEVICE_STATUS, &device_status, sizeof(device_status));
	if (rc < 0) {
		LOG_ERR("Failed to read device status");
		return rc;
	}

	if (!(device_status & BIT(1))) {
		LOG_ERR("No new data");
		return -EAGAIN;
	}

	uint16_t data_tvoc_val = sys_get_be16(data_tvoc);
	data->tvoc = (data_tvoc_val & ENS160_DATA_TVOC_LSB_MASK) >> ENS160_DATA_TVOC_LSB_SHIFT;

	uint8_t data_eco2[2];
	rc = config->bus_io->read(bus, ENS160_REG_DATA_ECO2, data_eco2, sizeof(data_eco2));
	if (rc < 0) {
		LOG_ERR("Failed to read eCO2");
		return rc;
	}

	uint16_t data_eco2_val = sys_get_be16(data_eco2);
	data->eco2 = (data_eco2_val & ENS160_DATA_ECO2_LSB_MASK) >> ENS160_DATA_ECO2_LSB_SHIFT;

	return 0;
}

static int ens160_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	struct ens160_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_CO2:
		val->val1 = data->eco2;
		val->val2 = 0;
		break;
	case SENSOR_CHAN_VOC:
		val->val1 = data->tvoc;
		val->val2 = 0;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api ens160_driver_api = {
	.sample_fetch = ens160_sample_fetch,
	.channel_get = ens160_channel_get,
};

#define ENS160_INIT(n) \
	static struct ens160_data ens160_data_##n; \
	static const struct ens160_config ens160_config_##n = { \
		.bus = ENS160_BUS_INIT(n), \
	}; \
	DEVICE_DT_INST_DEFINE(n, ens160_init, NULL, \
		&ens160_data_##n, &ens160_config_##n, \
		POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
		&ens160_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ENS160_INIT)
