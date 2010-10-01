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

#define AD7160_REG_AFE_DEVID			0x40050114
#define AD7160_REG_DEVICE_ID			0x40051E80
#define AD7160_REG_REV_ID			0x40051E84
#define AD7160_REG_FW_REV			0x40051E88
#define AD7160_REG_FP_BASELINE_CTRL		0x40051E8C
#define AD7160_REG_LP_BASELINE_CTRL		0x40051E90
#define AD7160_REG_DMA_RAM_ADDR_CTRL		0x40051E94
#define AD7160_REG_DMA_CTRL			0x40051E98
#define AD7160_REG_AFE_DAC_OFFS_CTRL		0x40051E9C
#define AD7160_REG_LPM_CTRL			0x40051EA0
#define AD7160_REG_BOOT_MODE_CTRL		0x40051EA4
#define AD7160_REG_FINGER_ACT_CTRL		0x40051EB8
#define AD7160_REG_XY_NB_SENSORS		0x40051EBC
#define AD7160_REG_XY_RES			0x40051EC0
#define AD7160_REG_POSITION_WINDOW_CTRL		0x40051EC4
#define AD7160_REG_FINGER_SEPARATION_CTRL	0x40051EC8
#define AD7160_REG_INT_GEST_EN_CTRL		0x40051EDC
#define AD7160_REG_HORZ_VERT_GEST_CTRL		0x40051EE0
#define AD7160_REG_PINCH_CTRL			0x40051EE4
#define AD7160_REG_ROT_CTRL			0x40051EE8
#define AD7160_REG_HAPTIC_CTRL			0x40051F00
#define AD7160_REG_HAPTIC_EFFECT1_CTRL		0x40051F04
#define AD7160_REG_HAPTIC_EFFECT2_CTRL		0x40051F08
#define AD7160_REG_HAPTIC_EFFECT3_CTRL1		0x40051F0C
#define AD7160_REG_HAPTIC_EFFECT3_CTRL2		0x40051F10
#define AD7160_REG_HAPTIC_EFFECT4_CTRL1		0x40051F14
#define AD7160_REG_HAPTIC_EFFECT4_CTRL2		0x40051F18
#define AD7160_REG_HAPTIC_EFFECT5_CTRL1		0x40051F1C
#define AD7160_REG_HAPTIC_EFFECT5_CTRL2		0x40051F20
#define AD7160_REG_HAPTIC_EFFECT5_CTRL3		0x40051F24
#define AD7160_REG_HAPTIC_EFFECT6_CTRL1		0x40051F28
#define AD7160_REG_HAPTIC_EFFECT6_CTRL2		0x40051F2C
#define AD7160_REG_HAPTIC_EFFECT6_CTRL3		0x40051F30
#define AD7160_REG_AFE_OFFS_ADJ_STAT		0x40051F5C
#define AD7160_REG_FINGER_ACT_STAT		0x40051F64
#define AD7160_REG_GEST_STAT			0x40051F68
#define AD7160_REG_NB_FINGERS			0x40051F6C
#define AD7160_REG_POS_DATA_STATUS1		0x40051F70
#define AD7160_REG_ABS_MT_TOUCH_STATUS1		0x40051F74
#define AD7160_REG_POS_DATA_STATUS2		0x40051F78
#define AD7160_REG_ABS_MT_TOUCH_STATUS2		0x40051F7C
#define AD7160_REG_POS_DATA_STATUS3		0x40051F80
#define AD7160_REG_ABS_MT_TOUCH_STATUS3		0x40051F84
#define AD7160_REG_POS_DATA_STATUS4		0x40051F88
#define AD7160_REG_ABS_MT_TOUCH_STATUS4		0x40051F8C
#define AD7160_REG_POS_DATA_STATUS5		0x40051F90
#define AD7160_REG_ABS_MT_TOUCH_STATUS5		0x40051F94
#define AD7160_REG_POS_DATA_STATUS6		0x40051F98
#define AD7160_REG_ABS_MT_TOUCH_STATUS6		0x40051F9C
#define AD7160_REG_POS_DATA_STATUS7		0x40051FA0
#define AD7160_REG_ABS_MT_TOUCH_STATUS7		0x40051FA4
#define AD7160_REG_POS_DATA_STATUS8		0x40051FA8
#define AD7160_REG_ABS_MT_TOUCH_STATUS8		0x40051FAC
#define AD7160_REG_POS_DATA_STATUS9		0x40051FB0
#define AD7160_REG_ABS_MT_TOUCH_STATUS9		0x40051FB4
#define AD7160_REG_POS_DATA_STATUS10		0x40051FB8
#define AD7160_REG_ABS_MT_TOUCH_STATUS10	0x40051FBC

#define REG_SIZE_BYTES		4
#define MAX_NUM_FINGERS		10
#define MAX_DATA_CNT		(MAX_NUM_FINGERS * 2)

struct ad7160;

struct ad7160_bus_ops {
	int (*read) (void *dev, u32 reg);
	int (*multi_read) (void *dev, u32 first_reg, u32 count, u32 *buf);
	int (*write) (void *dev, u32 reg, u32 val);
	int (*multi_write_bytes) (void *dev, u32 count, u8 *buf);
	void (*wakeup) (void *dev);
};

struct ad7160_bus_data {
	void *client;
	int irq;
	const struct ad7160_bus_ops *bops;
};

void ad7160_suspend(struct device *dev);
void ad7160_resume(struct device *dev);
int ad7160_probe(struct device *dev, struct ad7160_bus_data *bdata, u32 devid, u16 bustype);
int ad7160_remove(struct device *dev);

#if defined(CONFIG_TOUCHSCREEN_AD7160_RAW) || defined(CONFIG_TOUCHSCREEN_AD7160_RAW_MODULE)
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
