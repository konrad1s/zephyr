/*
 * Copyright (c) 2024 Konrad Sikora
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sciosense_ens160

#include <zephyr/logging/log.h>

#include "ens160.h"

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)

LOG_MODULE_DECLARE(ENS160, CONFIG_SENSOR_LOG_LEVEL);

static int ens160_spi_read_data(const struct device *dev, uint8_t reg_addr, uint8_t *value,
				uint8_t len)
{
	const struct ens160_config *cfg = dev->config;

	return spi_burst_read_dt(&cfg->bus, reg_addr, value, len);
}

static int ens160_spi_write_data(const struct device *dev, uint8_t reg_addr, uint8_t *value,
				 uint8_t len)
{
	const struct ens160_config *cfg = dev->config;

	return spi_burst_write_dt(&cfg->bus, reg_addr, value, len);
}

static const struct ens160_bus_io ens160_spi_bus_io = {
	.read = ens160_spi_read_data,
	.write = ens160_spi_write_data,
};

int ens160_spi_init(const struct device *dev)
{
	const struct ens160_config *config = dev->config;

	config->bus_io = &ens160_spi_bus_io;

	if (!device_is_ready(config->bus)) {
		LOG_ERR("SPI bus device %s is not ready", config->bus->name);
		return -ENODEV;
	}
}

#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */
