#ifndef ZEPHYR_DRIVERS_SENSOR_LTR390_LTR390_H_
#define ZEPHYR_DRIVERS_SENSOR_LTR390_LTR390_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>

/* Use Zephyr's existing macros */
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>

/* Register addresses */
#define LTR390_REG_MAIN_CTRL      0x00
#define LTR390_REG_MEAS_RATE      0x04
#define LTR390_REG_ALS_UVS_GAIN   0x05
#define LTR390_REG_PART_ID        0x06
#define LTR390_REG_MAIN_STATUS    0x07
#define LTR390_REG_ALS_DATA_0     0x0D
#define LTR390_REG_ALS_DATA_1     0x0E
#define LTR390_REG_ALS_DATA_2     0x0F
#define LTR390_REG_UVS_DATA_0     0x10
#define LTR390_REG_UVS_DATA_1     0x11
#define LTR390_REG_UVS_DATA_2     0x12
#define LTR390_REG_INT_CFG        0x19
#define LTR390_REG_INT_PST        0x1A
#define LTR390_REG_THRESH_UP_LSB  0x21
#define LTR390_REG_THRESH_LOW_LSB 0x24

/* Bit masks and shifts for MAIN_CTRL register */
#define LTR390_REG_MAIN_CTRL_SW_RESET_SHIFT 4
#define LTR390_REG_MAIN_CTRL_SW_RESET_MASK  BIT(4)
#define LTR390_REG_MAIN_CTRL_UVS_MODE_SHIFT 3
#define LTR390_REG_MAIN_CTRL_UVS_MODE_MASK  BIT(3)
#define LTR390_REG_MAIN_CTRL_ENABLE_SHIFT   1
#define LTR390_REG_MAIN_CTRL_ENABLE_MASK    BIT(1)

/* Bit masks and shifts for MEAS_RATE register */
#define LTR390_REG_MEAS_RATE_RES_SHIFT  4
#define LTR390_REG_MEAS_RATE_RES_MASK   GENMASK(6, 4)
#define LTR390_REG_MEAS_RATE_RATE_SHIFT 0
#define LTR390_REG_MEAS_RATE_RATE_MASK  GENMASK(2, 0)

/* Bit masks and shifts for ALS_UVS_GAIN register */
#define LTR390_REG_ALS_UVS_GAIN_RANGE_SHIFT 0
#define LTR390_REG_ALS_UVS_GAIN_RANGE_MASK  GENMASK(2, 0)

/* Bit masks and shifts for PART_ID register */
#define LTR390_REG_PART_ID_NUM_SHIFT 4
#define LTR390_REG_PART_ID_NUM_MASK  GENMASK(7, 4)
#define LTR390_REG_PART_ID_REV_SHIFT 0
#define LTR390_REG_PART_ID_REV_MASK  GENMASK(3, 0)

/* Bit masks and shifts for MAIN_STATUS register */
#define LTR390_REG_MAIN_STATUS_PWR_ON_SHIFT 5
#define LTR390_REG_MAIN_STATUS_PWR_ON_MASK  BIT(5)
#define LTR390_REG_MAIN_STATUS_INT_SHIFT    4
#define LTR390_REG_MAIN_STATUS_INT_MASK     BIT(4)
#define LTR390_REG_MAIN_STATUS_DATA_SHIFT   3
#define LTR390_REG_MAIN_STATUS_DATA_MASK    BIT(3)

/* Bit masks and shifts for INT_CFG register */
#define LTR390_REG_INT_CFG_INT_SEL_SHIFT    4
#define LTR390_REG_INT_CFG_INT_SEL_MASK     GENMASK(5, 4)
#define LTR390_REG_INT_CFG_INT_ENABLE_SHIFT 2
#define LTR390_REG_INT_CFG_INT_ENABLE_MASK  BIT(2)

/* Bit masks and shifts for INT_PST register */
#define LTR390_REG_INT_PST_PERSIST_SHIFT 4
#define LTR390_REG_INT_PST_PERSIST_MASK  GENMASK(7, 4)

/* Expected sensor IDs */
#define LTR390_PART_ID_EXPECTED  0xB2

/* Macros to set and get register fields.
 * For example:
 *    LTR390_REG_SET(LTR390_REG_MEAS_RATE, RES, cfg->resolution)
 *    LTR390_REG_GET(LTR390_REG_MEAS_RATE, RES, reg_val)
 */
#define LTR390_REG_SET(reg, field, value)                                                          \
	(((value) << reg##_##field##_SHIFT) & reg##_##field##_MASK)
#define LTR390_REG_GET(reg, field, value)                                                          \
	(((value) & reg##_##field##_MASK) >> reg##_##field##_SHIFT)

struct ltr390_config {
	struct i2c_dt_spec bus;
	bool uvs_mode;
	uint8_t resolution;
	uint8_t meas_rate;
	uint8_t gain;
#if defined(CONFIG_LTR390_TRIGGER)
	struct gpio_dt_spec int_gpio;
#endif
};

struct ltr390_data {
	uint32_t raw_als;
	uint32_t raw_uvs;
#if defined(CONFIG_LTR390_TRIGGER)
	struct gpio_callback gpio_cb;
	sensor_trigger_handler_t handler_drdy;
	const struct sensor_trigger *trigger_drdy;
	const struct device *dev;
#if defined(CONFIG_LTR390_TRIGGER_OWN_THREAD)
	struct k_sem trig_sem;
	struct k_thread trig_thread;
	K_KERNEL_STACK_MEMBER(trig_thread_stack, CONFIG_LTR390_TRIGGER_THREAD_STACK_SIZE);
#elif defined(CONFIG_LTR390_TRIGGER_GLOBAL_THREAD)
	struct k_work trig_work;
#endif
#endif
};

#if defined(CONFIG_LTR390_TRIGGER)
int ltr390_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler);

int ltr390_trigger_init(const struct device *dev);

int ltr390_threshold_set(const struct device *dev, const struct sensor_value *lower,
			 const struct sensor_value *upper);
#endif /* CONFIG_LTR390_TRIGGER */

#endif /* ZEPHYR_DRIVERS_SENSOR_LTR390_LTR390_H_ */
