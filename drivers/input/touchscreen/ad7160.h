/*
 * AD7160 touchscreen (bus interfaces)
 *
 * Copyright (C) 2010 Michael Hennerich, Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _AD7160_H_
#define _AD7160_H_

#include <linux/types.h>

#define REG_SIZE_BYTES		4
#define MAX_NUM_FINGERS		10
#define MAX_DATA_CNT		(MAX_NUM_FINGERS * 2)

struct ad7160;

struct ad7160_bus_ops {
	int (*read) (void *dev, u32 reg);
	int (*multi_read) (void *dev, u32 first_reg, u32 count, u32 *buf);
	int (*write) (void *dev, u32 reg, u32 val);
};
struct ad7160_bus_data {
	void *client;
	int irq;
	const struct ad7160_bus_ops *bops;
};

void ad7160_disable(struct device *dev);
void ad7160_enable(struct device *dev);
int ad7160_probe(struct device *dev, struct ad7160_bus_data *bdata, u32 devid, u16 bustype);
int ad7160_remove(struct device *dev);

#ifdef CONFIG_TOUCHSCREEN_AD7160_RAW
int ad7160_probe_raw(struct device *dev, struct ad7160_bus_data *bdata, u32 devid, u16 bustype);
int ad7160_remove_raw(struct device *dev);
void ad7160_feed_raw(void);
#else
static inline void ad7160_feed_raw(void)
{
}

static inline int ad7160_probe_raw(struct device *dev, struct ad7160_bus_data *bdata, u32 devid, u16 bustype)
{
	return 0;
}

static inline int ad7160_remove_raw(struct device *dev)
{
	return 0;
}

#endif /* CONFIG_AD7160_RAW_DATA_IFACE */
#endif /* _AD7160_H_ */
