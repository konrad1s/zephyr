/*
 * Copyright (c) 2024 Konrad Sikora
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_DRIVERS_SENSOR_ENS160_ENS160_H_
#define ZEPHYR_DRIVERS_SENSOR_ENS160_ENS160_H_

/* TODO: include */

union ens160_bus {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
	struct spi_dt_spec bus;
#endif
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
	struct i2c_dt_spec bus;
#endif
}

struct ens160_bus_io {
	int (*read)(union ens160_bus *bus, uint8_t reg_addr, uint8_t *data, uint16_t len);
	int (*write)(union ens160_bus *bus, uint8_t reg_addr, uint8_t *data, uint16_t len);
};

struct ens160_data {
	uint16_t co2;
	uint16_t voc;
};

struct ens160_config {
	union ens160_bus bus;
	struct ens160_bus_io *bus_io;
	int (*bus_init)(const struct device *dev);
	uint16_t temp_compensation;
	uint16_t rh_compensation;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_TMAG5170_TMAG5170_H_ */
