/*
 * ADI GPIO Abstraction Layer
 *
 * Copyright 2007-2013 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later
 */


#ifndef GPIO_ADI2_H
#define GPIO_ADI2_H

#include <linux/io.h>
#include <linux/platform_device.h>

struct adi_gpio_platform_data {
	int port_pin_base;	/* optional, 0 - driver decides */
	int port_width;
	u8 pint_id;		/* which pint to map the gpio port */
	u8 pint_assign;		/* 0 - assgin to 1ow 16 bits
				 * 1 - assign to high 16 bits
				 */
	u8 pint_map;		/* port mapping mask in pint */
};

#endif
