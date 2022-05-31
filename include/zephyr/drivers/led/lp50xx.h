/*
 * Copyright (c) 2020 Seagate Technology LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef ZEPHYR_INCLUDE_DRIVERS_LED_LP50XX_H_
#define ZEPHYR_INCLUDE_DRIVERS_LED_LP50XX_H_

/* Supported chipset models supported by this driver. */
enum lp50xx_chips {
	LP5009,
	LP5012,
	LP5030,
	LP5036,
};

#define LP50XX_MAX_LEDS(chip)			((chip == LP5009) ? 3 :	\
						((chip == LP5012) ? 4 :	\
						((chip == LP5030) ? 10 : 12)))
#define LP50XX_COLORS_PER_LED			3

/*
 * LED channels mapping.
 */

#define LP50XX_NUM_CHANNELS(chip)		((chip < LP5030) ? 20 : 52)

/* Bank channels. */
#define LP503X_BANK_CHAN_BASE			0
#define LP503X_BANK_BRIGHT_CHAN			LP503X_BANK_CHAN_BASE
#define LP503X_BANK_COL1_CHAN(led)		(LP503X_BANK_CHAN_BASE + 1)
#define LP503X_BANK_COL2_CHAN(led)		(LP503X_BANK_CHAN_BASE + 2)
#define LP503X_BANK_COL3_CHAN(led)		(LP503X_BANK_CHAN_BASE + 3)

/* LED brightness channels. */
#define LP50XX_LED_BRIGHT_CHAN_BASE		4
#define LP50XX_LED_BRIGHT_CHAN(led)
	(LP50XX_LED_BRIGHT_CHAN_BASE + led)

/* LED color channels. */
#define LP50XX_LED_COL_CHAN_BASE(chip)		((chip < LP5030) ? 8 : 16)
#define LP50XX_LED_COL1_CHAN(chip, led) \
	(LP50XX_LED_COL_CHAN_BASE(chip) + led * LP50XX_COLORS_PER_LED)
#define LP50XX_LED_COL2_CHAN(chip, led) \
	(LP50XX_LED_COL_CHAN_BASE(chip) + led * LP50XX_COLORS_PER_LED + 1)
#define LP50XX_LED_COL3_CHAN(chip, led) \
	(LP50XX_LED_COL_CHAN_BASE(chip) + led * LP50XX_COLORS_PER_LED + 2)

#endif /* ZEPHYR_INCLUDE_DRIVERS_LED_LP50XX_H_ */
