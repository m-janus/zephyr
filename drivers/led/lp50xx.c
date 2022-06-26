/*
 * Copyright (c) 2020 Seagate Technology LLC
 * Copyright (c) 2022 Grinn
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief LP50xx LED controller
 */

#include "lp50xx.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>
#include <zephyr/zephyr.h>

#define LOG_LEVEL CONFIG_LED_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lp50xx);

#define LP50XX_DEVICE_CONFIG0		0
#define   CONFIG0_CHIP_EN		BIT(6)

#define LP50XX_DEVICE_CONFIG1		0x1
#define   CONFIG1_LED_GLOBAL_OFF	BIT(0)
#define   CONFIG1_MAX_CURRENT_OPT	BIT(1)
#define   CONFIG1_PWM_DITHERING_EN	BIT(2)
#define   CONFIG1_AUTO_INCR_EN		BIT(3)
#define   CONFIG1_POWER_SAVE_EN		BIT(4)
#define   CONFIG1_LOG_SCALE_EN		BIT(5)

#define LP50XX_DEVICE_RESET(chip)		((chip < LP5030) ? 0x17 : 0x38)
#define LP50XX_LED_BRIGHTNESS_BASE(chip)	((chip < LP5030) ? 0x7 : 0x8)
#define LP50XX_BANK_BRIGHTNESS(chip)		((chip < LP5030) ? 0x3 : 0x4)
#define LP503X_OUT_COLOR_BASE(chip)		((chip < LP5030) ? 0xB : 0x14)

/* Expose channels starting from the bank registers. */
#define LP50XX_CHANNEL_BASE(chip)		LP50XX_BANK_BRIGHTNESS(chip)

/* Supported chipset models supported by this driver. */
enum lp50xx_chips {
        LP5009,
        LP5012,
        LP5030,
        LP5036,
};

struct lp50xx_config {
	struct i2c_dt_spec bus;
	uint8_t num_leds;
	bool log_scale_en;
	bool max_curr_opt;
	const struct led_info *leds_info;
	const struct gpio_dt_spec en_pin;
	enum lp50xx_chips chip;
};

struct lp50xx_data {
	uint8_t *chan_buf;
};

static const struct led_info *
lp50xx_led_to_info(const struct lp50xx_config *config, uint32_t led)
{
	for (uint8_t i = 0; i < config->num_leds; i++) {
		if (config->leds_info[i].index == led) {
			return &config->leds_info[i];
		}
	}
	return NULL;
}

int lp50xx_get_info(const struct device *dev, uint32_t led, const struct led_info **info)
{
	const struct lp50xx_config *config = dev->config;
	const struct led_info *led_info = lp50xx_led_to_info(config, led);

	if (!led_info) {
		return -EINVAL;
	}

	*info = led_info;

	return 0;
}

static int lp50xx_set_brightness(const struct device *dev,
				 uint32_t led, uint8_t value)
{
	const struct lp50xx_config *config = dev->config;
	const struct led_info *led_info = lp50xx_led_to_info(config, led);
	uint8_t buf[2];

	if (!led_info || value > 100) {
		return -EINVAL;
	}

	buf[0] = LP50XX_LED_BRIGHTNESS_BASE(config->chip) + led_info->index;
	buf[1] = (value * 0xff) / 100;

	return i2c_write_dt(&config->bus, buf, sizeof(buf));
}

static int lp50xx_on(const struct device *dev, uint32_t led)
{
	return lp50xx_set_brightness(dev, led, 100);
}

static int lp50xx_off(const struct device *dev, uint32_t led)
{
	return lp50xx_set_brightness(dev, led, 0);
}

static int lp50xx_set_color(const struct device *dev, uint32_t led,
			    uint8_t num_colors, const uint8_t *color)
{
	const struct lp50xx_config *config = dev->config;
	const struct led_info *led_info = lp50xx_led_to_info(config, led);
	uint8_t buf[4];

	if (!led_info || num_colors != led_info->num_colors) {
		return -EINVAL;
	}

	buf[0] = LP50XX_OUT_COLOR_BASE + 3 * led_info->index;
	buf[1] = color[0];
	buf[2] = color[1];
	buf[3] = color[2];

	return i2c_write_dt(&config->bus, buf, sizeof(buf));
}

static int lp50xx_write_channels(const struct device *dev,
				 uint32_t start_channel,
				 uint32_t num_channels, const uint8_t *buf)
{
	const struct lp50xx_config *config = dev->config;
	struct lp50xx_data *data = dev->data;

	if (start_channel >= LP50XX_NUM_CHANNELS ||
	    start_channel + num_channels > LP50XX_NUM_CHANNELS) {
		return -EINVAL;
	}

	/*
	 * Unfortunately this controller don't support commands split into
	 * two I2C messages.
	 */
	data->chan_buf[0] = (config->chip) + start_channel;
	memcpy(data->chan_buf + 1, buf, num_channels);

	return i2c_write_dt(&config->bus, data->chan_buf, num_channels + 1);
}

static int lp50xx_init(const struct device *dev)
{
	const struct lp50xx_config *config = dev->config;
	uint8_t buf[3];
	int err;

	if (!device_is_ready(config->bus.bus)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}

	if (config->num_leds > LP50XX_MAX_LEDS) {
		LOG_ERR("%s: invalid number of LEDs %d (max %d)",
			dev->name, config->num_leds, LP50XX_MAX_LEDS);
		return -EINVAL;
	}

	/* Enable EN pin if given in devicetree. */
	if (device_is_ready(config->en_pin.port)) {
		gpio_pin_configure_dt(&config->en_pin, GPIO_OUTPUT_HIGH);
		/* Power save mode deglitch time */
		k_sleep(K_MSEC(40));
	}

	/*
	 * Since the status of the LP50xx controller is unknown when entering
	 * this function, at first there's a device reset applied and then the whole
	 * configuration must be applied.
	 */

	/* Reset device */
	buf[0] = LP50XX_DEVICE_RESET(config->chip);
	buf[1] = 0xff;
	err = i2c_write_dt(&config->bus, buf, 2);
	if (err < 0) {
		return err;
	}

	/* Enable LED controller. */
	buf[0] = LP50XX_DEVICE_CONFIG0;
	buf[1] = CONFIG0_CHIP_EN;
	err = i2c_write_dt(&config->bus, buf, 2);
	if (err < 0) {
		return err;
	}

	/* Apply configuration. */
	buf[0] = LP50XX_DEVICE_CONFIG1;
	buf[1] = CONFIG1_PWM_DITHERING_EN | CONFIG1_AUTO_INCR_EN
		| CONFIG1_POWER_SAVE_EN;
	if (config->max_curr_opt) {
		buf[1] |= CONFIG1_MAX_CURRENT_OPT;
	}

	if (config->log_scale_en) {
		buf[1] |= CONFIG1_LOG_SCALE_EN;
	}

	return i2c_write_dt(&config->bus, buf, 2);
}

static const struct led_driver_api lp50xx_led_api = {
	.on		= lp50xx_on,
	.off		= lp50xx_off,
	.get_info	= lp50xx_get_info,
	.set_brightness	= lp50xx_set_brightness,
	.set_color	= lp50xx_set_color,
	.write_channels	= lp50xx_write_channels,
};

#define COLOR_MAPPING(led_node_id)					\
const uint8_t color_mapping_##led_node_id[] =				\
		DT_PROP(led_node_id, color_mapping);

#define LED_INFO(led_node_id)						\
{									\
	.label		= DT_LABEL(led_node_id),			\
	.index		= DT_PROP(led_node_id, index),			\
	.num_colors	=						\
		DT_PROP_LEN(led_node_id, color_mapping),		\
	.color_mapping	= color_mapping_##led_node_id,			\
},

#define LP50xx_DEVICE(id)						\
									\
DT_INST_FOREACH_CHILD(id, COLOR_MAPPING)				\
									\
static const struct led_info lp50xx_leds_##id[] = {			\
	DT_INST_FOREACH_CHILD(id, LED_INFO)				\
};									\
									\
static uint8_t lp50xx_chan_buf_##id[LP50XX_NUM_CHANNELS + 1];		\
									\
static struct lp50xx_config lp50xx_config_##id = {			\
	.bus		= I2C_DT_SPEC_INST_GET(id),			\
	.num_leds	= ARRAY_SIZE(lp50xx_leds_##id),			\
	.max_curr_opt	= DT_INST_PROP(id, max_curr_opt),		\
	.log_scale_en	= DT_INST_PROP(id, log_scale_en),		\
	.leds_info	= lp50xx_leds_##id,				\
	.en_pin		= GPIO_DT_SPEC_INST_GET_OR(id, en_gpios, {0}),	\
	#if DT_HAS_COMPAT_STATUS_OKAY(ti_lp5036)			\
        .chip = LP5036							\
	#elif DT_HAS_COMPAT_STATUS_OKAY(ti_lp5030)			\
        .chip = LP5030							\
	#elif DT_HAS_COMPAT_STATUS_OKAY(ti_lp5012)			\
        .chip = LP5012							\
	#else   							\
        .chip = LP5009							\
	#endif  							\
};									\
									\
static struct lp50xx_data lp50xx_data_##id = {				\
	.chan_buf	= lp50xx_chan_buf_##id,				\
};									\
									\
DEVICE_DT_INST_DEFINE(id,						\
		    &lp50xx_init,					\
		    NULL,						\
		    &lp50xx_data_##id,					\
		    &lp50xx_config_##id,				\
		    POST_KERNEL, CONFIG_LED_INIT_PRIORITY,		\
		    &lp50xx_led_api);

DT_INST_FOREACH_STATUS_OKAY(LP50xx_DEVICE)
