/*
 * File:         drivers/media/video/blackfin/adv7183b.c
 * Based on:     drivers/media/video/blackfin/mt9v022.c
 * Author:       Taha Iali
 *
 * Created:      may 2008
 * Description:  Command driver for Analog Devices ADV7183B sensor
 *
 *
 * Modified:
 *               Copyright 2008 Taha Iali
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include <linux/timex.h>
#include <linux/wait.h>
#include <media/v4l2-dev.h>
#include <asm/gpio.h>

#include "adv7183b.h"

static DEFINE_MUTEX(adv7183b_sysfs_lock);

static int adv7183b_write_byte(struct i2c_client *client, unsigned char offset, unsigned char data)
{
	u8 buf[2];

	BUG_ON(client == NULL);

	buf[0] = offset;
	buf[1] = data;
	i2c_master_send(client, buf, sizeof(buf));

	return 0;
}

static int adv7183b_probe(struct i2c_client *client)
{
	return 0;
}

static int adv7183b_set_pixfmt(struct i2c_client *client, u32 arg)
{
	return 0;
}

static int adv7183b_init(struct i2c_client *client, u32 arg)
{

	printk(KERN_INFO "driver for ADV7183B init\n");
	if (adv7183b_probe(client)) {
		return -ENODEV;
	}

	/* Configuration Example, taken from datasheet ADV7183B */

#ifdef ADV7183B_28MHZ
	/* EXAMPLES IN THIS SECTION USE A 28 MHz CLOCK.
	 * Mode 1 CVBS Input (Composite Video on AIN5)
	 * All standards are supported through autodetect, 8-bit, 4:2:2, ITU-R BT.656 output on P15 to P8.
	 */
	adv7183b_write_byte(client, 0x00, 0x04); /* CVBS input on AIN5. */
	adv7183b_write_byte(client, 0x15, 0x00); /* Slow down digital clamps. */
	adv7183b_write_byte(client, 0x17, 0x41); /* Set CSFM to SH1. */
	adv7183b_write_byte(client, 0x1D, 0x40); /* Enable 28 MHz crystal. */
	adv7183b_write_byte(client, 0x0F, 0x40); /* TRAQ. */
	adv7183b_write_byte(client, 0x3A, 0x16); /* Power down ADC 1 and ADC 2. */
	adv7183b_write_byte(client, 0x3D, 0xC3); /* MWE enable manual window. */
	adv7183b_write_byte(client, 0x3F, 0xE4); /* BGB to 36. */
	adv7183b_write_byte(client, 0x50, 0x04); /* Set DNR threshold to 4 for flat response. */
	adv7183b_write_byte(client, 0x0E, 0x80); /* ADI recommended programming sequence. */
	/* This sequence must be followed exactly when setting up the decoder. */
	adv7183b_write_byte(client, 0x50, 0x20); /* Recommended setting. */
	adv7183b_write_byte(client, 0x52, 0x18); /* Recommended setting. */
	adv7183b_write_byte(client, 0x58, 0xED); /* Recommended setting. */
	adv7183b_write_byte(client, 0x77, 0xC5); /* Recommended setting. */
	adv7183b_write_byte(client, 0x7C, 0x93); /* Recommended setting. */
	adv7183b_write_byte(client, 0x7D, 0x00); /* Recommended setting. */
	adv7183b_write_byte(client, 0x90, 0xC9); /* Recommended setting. */
	adv7183b_write_byte(client, 0x91, 0x40); /* Recommended setting. */
	adv7183b_write_byte(client, 0x92, 0x3C); /* Recommended setting. */
	adv7183b_write_byte(client, 0x93, 0xCA); /* Recommended setting. */
	adv7183b_write_byte(client, 0x94, 0xdD); /* Recommended setting. */
	adv7183b_write_byte(client, 0xCF, 0x50); /* Recommended setting. */
	adv7183b_write_byte(client, 0xD0, 0x4E); /* Recommended setting. */
	adv7183b_write_byte(client, 0xD6, 0xDD); /* Recommended setting. */
	adv7183b_write_byte(client, 0xE5, 0x51); /* Recommended setting. */
	adv7183b_write_byte(client, 0xD5, 0xA0); /* Recommended setting. */
	adv7183b_write_byte(client, 0xD7, 0xEA); /* Recommended setting. */
	adv7183b_write_byte(client, 0xE4, 0x3E); /* Recommended setting. */
	adv7183b_write_byte(client, 0xE9, 0x3E); /* Recommended setting. */
	adv7183b_write_byte(client, 0xEA, 0x0F); /* Recommended setting. */
	adv7183b_write_byte(client, 0x0E, 0x00); /* Recommended setting. */
#else
	/* EXAMPLES USING 27 MHz CLOCK
	 * Mode 1 CVBS Input (Composite Video on AIN5)
	 * All standards are supported through autodetect, 8-bit, 4:2:2, ITU-R BT.656 output on P15 to P8.
	 */
	adv7183b_write_byte(client, 0x00, 0x04); /* CVBS input on AIN5. */
	adv7183b_write_byte(client, 0x15, 0x00); /* Slow down digital clamps. */
	adv7183b_write_byte(client, 0x17, 0x41); /* Set CSFM to SH1. */
	adv7183b_write_byte(client, 0x3A, 0x16); /* Power down ADC 1 and ADC 2. */
	adv7183b_write_byte(client, 0x50, 0x04); /* Set DNR threshold to 4 for flat response. */
	adv7183b_write_byte(client, 0x0E, 0x80); /* ADI recommended programming sequence. */
	/* This sequence must be followed exactly when setting up the decoder. */
	adv7183b_write_byte(client, 0x50, 0x20); /* Recommended setting. */
	adv7183b_write_byte(client, 0x52, 0x18); /* Recommended setting. */
	adv7183b_write_byte(client, 0x58, 0xED); /* Recommended setting. */
	adv7183b_write_byte(client, 0x77, 0xC5); /* Recommended setting. */
	adv7183b_write_byte(client, 0x7C, 0x93); /* Recommended setting. */
	adv7183b_write_byte(client, 0x7D, 0x00); /* Recommended setting. */
	adv7183b_write_byte(client, 0xD0, 0x48); /* Recommended setting. */
	adv7183b_write_byte(client, 0xD5, 0xA0); /* Recommended setting. */
	adv7183b_write_byte(client, 0xD7, 0xEA); /* Recommended setting. */
	adv7183b_write_byte(client, 0xE4, 0x3E); /* Recommended setting. */
	adv7183b_write_byte(client, 0xE9, 0x3E); /* Recommended setting. */
	adv7183b_write_byte(client, 0xEA, 0x0F); /* Recommended setting. */
	adv7183b_write_byte(client, 0x0E, 0x00); /* Recommended setting. */
#endif

#ifdef ADV7183B_STRONG
	/* The chip allow to set the driving power of the output signals.
	 * By default, the signals are too weak for some design. The code below make
	 * the driving power at his highest level.
	 */
	adv7183b_write_byte(client, 0xF4, 0x3F);
#endif

#ifdef ADV7183B_GPIO_OE
	if (gpio_request(ADV7183B_GPIO_OE, DRV_NAME)) {
		printk(KERN_ERR "bcap_open: Failed to request GPIO %d\n", ADV7183B_GPIO_OE);
		return -EBUSY;
	}
	gpio_direction_output(ADV7183B_GPIO_OE, 0);
#endif

	return 0;

}

static int adv7183b_exit(struct i2c_client *client, u32 arg)
{
	return 0;
}

int adv7183b_cam_control(struct i2c_client *client, u32 cmd, u32 arg)
{
	switch (cmd) {
	case CAM_CMD_INIT:
		return adv7183b_init(client, arg);
	case CAM_CMD_SET_PIXFMT:
		return adv7183b_set_pixfmt(client, arg);
	case CAM_CMD_EXIT:
		return adv7183b_exit(client, arg);
	default:
		return -ENOIOCTLCMD;
	}
	return 0;
}

static int adv7183b_create_sysfs(struct video_device *v4ldev)
{
	return 0;
}

static int adv7183b_power(u32 arg)
{
#ifdef ADV7183B_GPIO_RESET
	if (gpio_request(ADV7183B_GPIO_RESET, DRV_NAME)) {
		printk(KERN_ERR "bcap_open: Failed to request GPIO %d\n", ADV7183B_GPIO_RESET);
		return -EBUSY;
	}
	gpio_direction_output(ADV7183B_GPIO_RESET, arg);
#endif

#ifdef CONFIG_BFIN533_EZKIT
#define FLASHA_PORTA_DIR 	0x20270006
#define FLASHA_PORTA_OUT 	0x20270004
#define RST_7183 		0x8		/* decoder reset bit #3 in flashA portA */
#define PPICLK_ADV7183_SELECT 	0x10		/* decoder clock to PPI bit #4 in flashA portA */

	bfin_write16(FLASHA_PORTA_DIR, 0xFFFF);
	bfin_write16(FLASHA_PORTA_OUT, bfin_read16(FLASHA_PORTA_OUT) | RST_7183 | PPICLK_ADV7183_SELECT);
#endif

	return 0;
}

static struct bcap_camera_ops adv7183b_ops = {
	.cam_control = adv7183b_cam_control,
	.create_sysfs = adv7183b_create_sysfs,
	.power = adv7183b_power,
};

struct bcap_camera_ops *get_camops(void)
{
	printk(KERN_INFO "driver for ADV7183B get_camops\n");
	return (&adv7183b_ops);

}
EXPORT_SYMBOL(get_camops);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Taha Iali <tahaiali@hotmail.com>");
