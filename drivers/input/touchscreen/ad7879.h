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

typedef int (ad7879_read_t) (void *bus_data, u8 reg);
typedef int (ad7879_multi_read_t) (void *bus_data, u8 first_reg, u8 count, u16 *buf);
typedef int (ad7879_write_t) (void *bus_data, u8 reg, u16 val);

struct ad7879_bus_ops {
	void *bus_data;
	int irq;
	ad7879_read_t *read;
	ad7879_multi_read_t *multi_read;
	ad7879_write_t *write;
};

int ad7879_disable(struct device *dev);
int ad7879_enable(struct device *dev);
int ad7879_probe(struct device *dev, struct ad7879_bus_ops *bops, u8 devid, u16 bustype);
int ad7879_remove(struct device *dev);

#endif
