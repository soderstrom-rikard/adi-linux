/*
 * ad5398.h --  Voltage and current regulation for AD5398 and AD5821
 *
 * Copyright 2010 Analog Devices Inc.
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Licensed under the GPL-2 or later.
 */
#ifndef REGULATOR_AD5398
#define REGULATOR_AD5398

#include <linux/regulator/machine.h>

#define CURRENT_EN_MASK		0x8000
#define CURRENT_VAL_MASK	0x3FF0
#define CURRENT_VAL_OFFSET	4
#define CURRENT_VAL_MAX		0x8000

/**
 * ad5398_platform_data - platform data for ad5398
 * @num_current_level: number of current levels, depends on effective
 *			DA register bits on chip.
 * @ad5398_init_data: regulator init data
 */
struct ad5398_platform_data {
	unsigned short num_current_level;
	struct regulator_init_data *regulator_data;
};

#endif
