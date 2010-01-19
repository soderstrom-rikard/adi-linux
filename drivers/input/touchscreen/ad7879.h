/*
 * AD7879/AD7889 touchscreen (bus interfaces)
 *
 * Copyright (C) 2008-2010 Michael Hennerich, Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _AD7879_H_
#define _AD7879_H_

#include <linux/types.h>

struct ad7879;

struct ad7879_bus_ops {
	int (*read) (void *client, u8 reg);
	int (*multi_read) (void *client, u8 first_reg, u8 count, u16 *buf);
	int (*write) (void *client, u8 reg, u16 val);
};
struct ad7879_bus_data {
	void *client;
	int irq;
	const struct ad7879_bus_ops *bops;
};

int ad7879_disable(struct device *dev);
int ad7879_enable(struct device *dev);
int ad7879_probe(struct device *dev, struct ad7879_bus_data *bdata, u8 devid, u16 bustype);
int ad7879_remove(struct device *dev);

#endif
