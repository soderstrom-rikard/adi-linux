/*
 * adp_switch.h --  Voltage regulation for switch only power devices.
 *		    AD122, AD123, AD124, AD125, AD150, AD5022, etc.
 *
 * Copyright 2010 Analog Devices Inc.
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Licensed under the GPL-2 or later.
 */
#ifndef REGULATOR_ADP_SWITCH
#define REGULATOR_ADP_SWITCH

#include <linux/regulator/machine.h>

/**
 * adp_switch_platform_data - platform data for power swtich
 * @regulator_num: number of regulators
 * @regulator_data: regulator init data
 */
struct adp_switch_platform_data {
	unsigned short regulator_num;
	struct regulator_init_data *regulator_data;
};

#endif
