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
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <device.h>
#include <zephyr.h>

const struct led_info *lp50xx_led_to_info(const struct lp50xx_config *config, uint32_t led)
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
