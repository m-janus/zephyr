/*
 * Copyright (c) 2020 Seagate Technology LLC
 * Copyright (c) 2022 Grinn
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_LED_LP50XX_H_
#define ZEPHYR_DRIVERS_LED_LP50XX_H_

#define LP50XX_DEVICE_CONFIG0		0
#define   CONFIG0_CHIP_EN		BIT(6)

#define LP50XX_DEVICE_CONFIG1		0x1
#define   CONFIG1_LED_GLOBAL_OFF	BIT(0)
#define   CONFIG1_MAX_CURRENT_OPT	BIT(1)
#define   CONFIG1_PWM_DITHERING_EN	BIT(2)
#define   CONFIG1_AUTO_INCR_EN		BIT(3)
#define   CONFIG1_POWER_SAVE_EN		BIT(4)
#define   CONFIG1_LOG_SCALE_EN		BIT(5)

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led.h>

struct lp50xx_config {
	struct i2c_dt_spec bus;
	uint8_t num_leds;
	bool log_scale_en;
	bool max_curr_opt;
	const struct led_info *leds_info;
	const struct gpio_dt_spec en_pin;
};

struct lp50xx_data {
	uint8_t *chan_buf;
};

const struct led_info *lp50xx_led_to_info(const struct lp50xx_config *config, uint32_t led);
int lp50xx_get_info(const struct device *dev, uint32_t led, const struct led_info **info);

#endif /* ZEPHYR_DRIVERS_LED_LP50XX_H_ */
