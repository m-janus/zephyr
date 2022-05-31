/*
 * Copyright (c) 2020 Seagate Technology LLC
 * Copyright (c) Grinn
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef ZEPHYR_INCLUDE_DRIVERS_LED_LP50XX_H_
#define ZEPHYR_INCLUDE_DRIVERS_LED_LP50XX_H_

#define LP50XX_COLORS_PER_LED		3

/* LED brightness channels. */
#define LP50XX_LED_BRIGHT_CHAN_BASE	4
#define LP50XX_LED_BRIGHT_CHAN(led)	(LP50XX_LED_BRIGHT_CHAN_BASE + led)

/* Bank channels. */
#define LP50XX_BANK_CHAN_BASE		0
#define LP50XX_BANK_BRIGHT_CHAN		LP50XX_BANK_CHAN_BASE
#define LP50XX_BANK_COL1_CHAN(led)	(LP50XX_BANK_CHAN_BASE + 1)
#define LP50XX_BANK_COL2_CHAN(led)	(LP50XX_BANK_CHAN_BASE + 2)
#define LP50XX_BANK_COL3_CHAN(led)	(LP50XX_BANK_CHAN_BASE + 3)

#define LP50XX_LED_COL1_CHAN(led)	(LP50XX_LED_COL_CHAN_BASE + \
					 led * LP50XX_COLORS_PER_LED)
#define LP50XX_LED_COL2_CHAN(led)	(LP50XX_LED_COL_CHAN_BASE + \
					 led * LP50XX_COLORS_PER_LED + 1)
#define LP50XX_LED_COL3_CHAN(led)	(LP50XX_LED_COL_CHAN_BASE + \
					 led * LP50XX_COLORS_PER_LED + 2)

#ifdef CONFIG_LP503X
#define LP50XX_MAX_LEDS			12

/*
 * LED channels mapping.
 */
#define LP50XX_NUM_CHANNELS		52

/* LED color channels. */
#define LP50XX_LED_COL_CHAN_BASE	16

#endif /* CONFIG_LP503X */
#ifdef CONFIG_LP5009_12
#define LP50XX_MAX_LEDS			4

/*
 * LED channels mapping.
 */
#define LP50XX_NUM_CHANNELS		20

/* LED color channels. */
#define LP50XX_LED_COL_CHAN_BASE	8

#endif /* CONFIG_LP5009_12 */

#endif /* ZEPHYR_INCLUDE_DRIVERS_LED_LP50XX_H_ */
