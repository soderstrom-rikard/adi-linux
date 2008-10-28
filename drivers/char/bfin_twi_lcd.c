/*
 * File:         drivers/char/bfin_twi_lcd.c
 * Based on:
 * Author:       Michael Hennerich
 *
 * Created:      Feb. 27th 2006
 * Description:  TWI LCD driver (HD44780) connected to a PCF8574 I2C IO expander
 *
 * Modified:
 *               Copyright 2006 Analog Devices Inc.
 *
 * Bugs:         Enter bugs at http://blackfin.uclinux.org/
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

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/ioport.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/delay.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/system.h>
#include <linux/delay.h>
#include <linux/i2c.h>

#include "bfin_twi_lcd.h"

static int __init pcf8574_lcd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	pcf8574_lcd_client = client;
	drv_HD_I2C_load();
	lcd_present = 1;

	return 0;
}

static int __exit pcf8574_lcd_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id pcf8574_lcd_id[] = {
	{ PCF8574_LCD_DRV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pcf8574_lcd_id);

static struct i2c_driver pcf8574_lcd_driver = {
	.driver = {
		.name = PCF8574_LCD_DRV_NAME,
	},
	.probe = pcf8574_lcd_probe,
	.remove = __exit_p(pcf8574_lcd_remove),
	.id_table = pcf8574_lcd_id,
};

static int lcd_ioctl(struct inode *inode, struct file *file,
		     unsigned int cmd, unsigned long arg)
{

	switch (cmd) {

	case LCD_Contr:
		break;

	case LCD_On:
		udelay(T_EXEC);
		BusyCheck();
		drv_HD_I2C_command(currController, 0x0F);
		break;

	case LCD_Off:
		udelay(T_EXEC);
		BusyCheck();
		drv_HD_I2C_command(currController, 0x08);
		break;

	case LCD_Reset:
		udelay(T_EXEC);
		drv_HD_I2C_command(currController, 0x3F);
		udelay(T_EXEC);
		drv_HD_I2C_command(currController, 0x3F);
		udelay(T_EXEC);
		drv_HD_I2C_command(currController, 0x3F);
		udelay(T_EXEC);
		drv_HD_I2C_command(currController, 0x3F);
		udelay(T_EXEC);
		drv_HD_I2C_command(currController, 0x01);
		udelay(T_EXEC);
		drv_HD_I2C_command(currController, 0x06);
		break;

	case LCD_Clear:
		udelay(T_EXEC);
		BusyCheck();
		drv_HD_I2C_command(currController, 0x01);
		msleep(T_CLEAR);
		break;

	case LCD_Cursor_Left:
		udelay(T_EXEC);
		BusyCheck();
		drv_HD_I2C_command(currController, 0x10);
		break;

	case LCD_Cursor_Right:
		udelay(T_EXEC);
		BusyCheck();
		drv_HD_I2C_command(currController, 0x14);
		break;

	case LCD_Cursor_Off:
		udelay(T_EXEC);
		BusyCheck();
		drv_HD_I2C_command(currController, 0x0C);
		break;

	case LCD_Cursor_On:
		udelay(T_EXEC);
		BusyCheck();
		drv_HD_I2C_command(currController, 0x0F);
		break;

	case LCD_Blink_Off:
		udelay(T_EXEC);
		BusyCheck();
		drv_HD_I2C_command(currController, 0x0E);
		break;

	case LCD_Curr_Controller:
		currController = arg;
		break;

	case LCD_Set_Cursor_Pos:
		{
			udelay(T_EXEC);
			BusyCheck();
			drv_HD_I2C_command(currController, arg | kLCD_Addr);
			break;
		}

	case LCD_Set_Cursor:
		{
			udelay(T_EXEC);
			BusyCheck();
			drv_HD_I2C_byte(currController, arg);
			udelay(T_EXEC);
			BusyCheck();
			drv_HD_I2C_command(currController, 0x10);

			break;
		}

	case LCD_Disp_Left:
		udelay(T_EXEC);
		BusyCheck();
		drv_HD_I2C_command(currController, 0x18);
		break;

	case LCD_Disp_Right:
		udelay(T_EXEC);
		BusyCheck();
		drv_HD_I2C_command(currController, 0x1C);
		break;

	case LCD_Home:
		udelay(T_EXEC);
		BusyCheck();
		drv_HD_I2C_command(currController, 0x02);
		break;

	default:
		return -EINVAL;

	}

	return 0;

}

static int lcd_open(struct inode *inode, struct file *file)
{
	if (!lcd_present)
		return -ENXIO;
	else
		return 0;
}

static ssize_t lcd_write(struct file *filp, const char *buf, size_t count,
			 loff_t * f_pos)
{

	drv_HD_I2C_data(currController, buf, count);

	return count;

}

/*
 *	The various file operations we support.
 */

static struct file_operations lcd_fops = {
      owner:THIS_MODULE,
//      read:lcd_read,
      write:lcd_write,
      ioctl:lcd_ioctl,
      open:lcd_open,
};

static struct miscdevice bfin_twi_lcd_dev = {
	LCD_MINOR,
	"lcd",
	&lcd_fops
};

static int __init lcd_init(void)
{
	int result;
	pr_info("%s\n", LCD_DRIVER);

	result = misc_register(&bfin_twi_lcd_dev);
	if (result < 0) {
		printk(KERN_WARNING "bfin_twi_lcd: can't get minor %d\n",
		       MISC_MAJOR);
		return result;
	}

	return i2c_add_driver(&pcf8574_lcd_driver);
}

static void drv_HD_I2C_nibble(unsigned char controller, unsigned char nibble)
{
	unsigned char enable;
	unsigned char command;	/* this is actually the first data byte on the PCF8574 */
	unsigned char data_block[2];
	/* enable signal: 'controller' is a bitmask */
	/* bit n .. send to controller #n */
	/* so we can send a byte to more controllers at the same time! */
	enable = 0;
	if (controller & 0x01)
		enable |= SIGNAL_ENABLE;
	if (controller & 0x02)
		enable |= SIGNAL_ENABLE2;

	command = nibble;
	data_block[0] = nibble | enable;
	data_block[1] = nibble;

	i2c_smbus_write_block_data(pcf8574_lcd_client, command, 2, data_block);
}

static void drv_HD_I2C_byte(const unsigned char controller,
			    const unsigned char data)
{
	/* send data with RS enabled */
	drv_HD_I2C_nibble(controller, ((data >> 4) & 0x0f) | SIGNAL_RS);
	drv_HD_I2C_nibble(controller, (data & 0x0f) | SIGNAL_RS);
	udelay(T_INIT2);
}

static void drv_HD_I2C_command(const unsigned char controller,
			       const unsigned char cmd)
{
	/* send data with RS disabled */
	drv_HD_I2C_nibble(controller, ((cmd >> 4) & 0x0f));
	drv_HD_I2C_nibble(controller, ((cmd) & 0x0f));
	udelay(T_INIT2);
}

static void drv_HD_I2C_data(const unsigned char controller, const char *string,
			    const int len)
{
	int l = len;

	/* sanity check */
	if (len <= 0)
		return;

	while (l--) {
		if (*string)
			drv_HD_I2C_byte(controller, *(string++));
	}
}

static int drv_HD_I2C_load(void)
{
	/* initialize display */
	drv_HD_I2C_nibble(CONTROLLER_BOTH, 0x03);
	msleep(T_INIT1);	/* 4 Bit mode, wait 4.1 ms */
	drv_HD_I2C_nibble(CONTROLLER_BOTH, 0x03);
	udelay(T_INIT2);	/* 4 Bit mode, wait 100 us */
	drv_HD_I2C_nibble(CONTROLLER_BOTH, 0x03);
	udelay(T_INIT2);	/* 4 Bit mode, wait 4.1 ms */
	drv_HD_I2C_nibble(CONTROLLER_BOTH, 0x02);
	udelay(T_INIT2);	/* 4 Bit mode, wait 100 us */
	drv_HD_I2C_command(CONTROLLER_BOTH, 0x28);	/* 4 Bit mode, 1/16 duty cycle, 5x8 font */

	/* Set defaults:
	   Low 0xC0 Display On
	   Low 0x06 Cursor increment, no shift
	   Low 0x80 Display Address 0
	   Low 0x02 Cursor Home */

	udelay(T_INIT2);
	drv_HD_I2C_command(CONTROLLER_BOTH, 0x0f);
	udelay(T_INIT2);
	drv_HD_I2C_command(CONTROLLER_BOTH, 0x06);
	udelay(T_INIT2);
	drv_HD_I2C_command(CONTROLLER_BOTH, 0x80);
	udelay(T_INIT2);
	drv_HD_I2C_command(CONTROLLER_BOTH, 0x02);
	udelay(T_INIT2);
	drv_HD_I2C_command(CONTROLLER_BOTH, 0x0C);
	udelay(T_INIT2);
	drv_HD_I2C_command(CONTROLLER_BOTH, 0x01);
	udelay(T_INIT2);

	return 0;
}

static void __exit lcd_exit(void)
{
	i2c_del_driver(&pcf8574_lcd_driver);
	misc_deregister(&bfin_twi_lcd_dev);
	printk(KERN_ALERT "Goodbye LCD\n");
}

module_init(lcd_init);
module_exit(lcd_exit);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("TWI LCD driver (HD44780)");
MODULE_LICENSE("GPL");
