#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/i2c.h>
#include "ltr390.h"

LOG_MODULE_REGISTER(LTR390_TRIGGER, CONFIG_SENSOR_LOG_LEVEL);

#if defined(CONFIG_LTR390_TRIGGER)

#if defined(CONFIG_LTR390_TRIGGER_OWN_THREAD)
static void ltr390_trigger_thread(void *arg1, void *unused1, void *unused2)
{
	const struct device *dev = (const struct device *)arg1;
	struct ltr390_data *data = dev->data;

	while (1) {
		k_sem_take(&data->trig_sem, K_FOREVER);
		if (data->handler_drdy) {
			data->handler_drdy(dev, data->trigger_drdy);
		}
	}
}
#endif /* CONFIG_LTR390_TRIGGER_OWN_THREAD */

#if defined(CONFIG_LTR390_TRIGGER_GLOBAL_THREAD)
static void ltr390_trigger_work_handler(struct k_work *work)
{
	struct ltr390_data *data = CONTAINER_OF(work, struct ltr390_data, trig_work);

	if (data->handler_drdy) {
		data->handler_drdy(data->dev, data->trigger_drdy);
	}
}
#endif /* CONFIG_LTR390_TRIGGER_GLOBAL_THREAD */

/* GPIO callback invoked on sensor interrupt */
static void ltr390_gpio_callback(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	struct ltr390_data *data = CONTAINER_OF(cb, struct ltr390_data, gpio_cb);

#if defined(CONFIG_LTR390_TRIGGER_OWN_THREAD)
	k_sem_give(&data->trig_sem);
#elif defined(CONFIG_LTR390_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&data->trig_work);
#elif defined(CONFIG_LTR390_TRIGGER_DIRECT)
	if (data->handler_drdy) {
		data->handler_drdy(data->dev, data->trigger_drdy);
	}
#endif
}

int ltr390_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler)
{
	struct ltr390_data *data = dev->data;
	const struct ltr390_config *cfg = dev->config;
	uint8_t int_cfg = 0;
	int ret;

	/* Support only DATA_READY and THRESHOLD triggers */
	if ((trig->type != SENSOR_TRIG_DATA_READY) && (trig->type != SENSOR_TRIG_THRESHOLD)) {
		return -ENOTSUP;
	}

	data->trigger_drdy = trig;
	data->handler_drdy = handler;

	/* Configure INT_CFG register using helper macro.
	 * For channel selection, use:
	 *   - 0x1 for ALS mode
	 *   - 0x3 for UVS mode
	 */
	if (cfg->uvs_mode) {
		int_cfg = LTR390_REG_SET(LTR390_REG_INT_CFG, INT_SEL, 3);
	} else {
		int_cfg = LTR390_REG_SET(LTR390_REG_INT_CFG, INT_SEL, 1);
	}
	if (handler != NULL) {
		int_cfg |= LTR390_INT_CFG_INT_ENABLE_MASK;
	}
	ret = ltr390_write_reg(&cfg->bus, LTR390_REG_INT_CFG, int_cfg);
	if (ret < 0) {
		LOG_ERR("Failed to configure INT_CFG: %d", ret);
		return ret;
	}

	return 0;
}

int ltr390_threshold_set(const struct device *dev, const struct sensor_value *lower,
			 const struct sensor_value *upper)
{
	const struct ltr390_config *cfg = dev->config;
	int ret;
	uint32_t lower_raw = lower->val1; /* Use integer part */
	uint32_t upper_raw = upper->val1;
	uint8_t buf[3];

	/* Write lower threshold registers starting at LTR390_REG_THRESH_LOW_LSB */
	buf[0] = lower_raw & 0xFF;
	buf[1] = (lower_raw >> 8) & 0xFF;
	buf[2] = (lower_raw >> 16) & 0x0F;
	ret = i2c_burst_write_dt(&cfg->bus, LTR390_REG_THRESH_LOW_LSB, buf, sizeof(buf));
	if (ret < 0) {
		LOG_ERR("Failed to write lower threshold: %d", ret);
		return ret;
	}

	/* Write upper threshold registers starting at LTR390_REG_THRESH_UP_LSB */
	buf[0] = upper_raw & 0xFF;
	buf[1] = (upper_raw >> 8) & 0xFF;
	buf[2] = (upper_raw >> 16) & 0x0F;
	ret = i2c_burst_write_dt(&cfg->bus, LTR390_REG_THRESH_UP_LSB, buf, sizeof(buf));
	if (ret < 0) {
		LOG_ERR("Failed to write upper threshold: %d", ret);
		return ret;
	}

	return 0;
}

int ltr390_trigger_init(const struct device *dev)
{
	struct ltr390_data *data = dev->data;
	const struct ltr390_config *cfg = dev->config;
	int ret;

	if (!device_is_ready(cfg->int_gpio.port)) {
		LOG_ERR("%s: GPIO device %s not ready", dev->name, cfg->int_gpio.port->name);
		return -ENODEV;
	}

	data->dev = dev;
#if defined(CONFIG_LTR390_TRIGGER_OWN_THREAD)
	k_sem_init(&data->trig_sem, 0, 1);
	k_thread_create(&data->trig_thread, data->trig_thread_stack,
			CONFIG_LTR390_TRIGGER_THREAD_STACK_SIZE, ltr390_trigger_thread, (void *)dev,
			NULL, NULL, K_PRIO_COOP(CONFIG_LTR390_TRIGGER_THREAD_PRIORITY), 0,
			K_NO_WAIT);
#elif defined(CONFIG_LTR390_TRIGGER_GLOBAL_THREAD)
	data->trig_work.handler = ltr390_trigger_work_handler;
#endif

	ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure int-gpio pin: %d", ret);
		return ret;
	}

	gpio_init_callback(&data->gpio_cb, ltr390_gpio_callback, BIT(cfg->int_gpio.pin));
	ret = gpio_add_callback(cfg->int_gpio.port, &data->gpio_cb);
	if (ret < 0) {
		LOG_ERR("Failed to add GPIO callback: %d", ret);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_EDGE_FALLING);
	if (ret < 0) {
		LOG_ERR("Failed to configure GPIO interrupt: %d", ret);
		return ret;
	}

	LOG_INF("LTR390 trigger initialized");
	return 0;
}

#endif /* CONFIG_LTR390_TRIGGER */
