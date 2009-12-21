/*
 * TWI LCD driver (HD44780) connected to a PCF8574 I2C IO expander
 *
 * Copyright 2006-2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#define pr_fmt(x) "bfin_twi_lcd: " x

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/sched.h>

/* HD44780 execution timings [microseconds]
 * as these values differ from spec to spec,
 * we use the worst-case values.
 */

#define T_INIT1    5		/* (ms) first init sequence:  4.1 msec */
#define T_INIT2  150		/* second init sequence: 100 usec */
#define T_EXEC    80		/* normal execution time */
#define T_WRCG   120		/* CG RAM Write */
#define T_CLEAR    3		/* (ms) Clear Display */

#define SIGNAL_RW		0x20
#define SIGNAL_RS		0x10
#define SIGNAL_ENABLE 	0x40
#define SIGNAL_ENABLE2	0x80

/* LCD Driver function headers and globals */

#define	PCF8574_LCD_DRV_NAME		"pcf8574_lcd"
static struct i2c_client *pcf8574_lcd_client;

static int currController = 0x2;
static unsigned int lcd_present = 0;

#define kLCD_Addr       0x80


#define BusyCheck()	do { } while (0)

/*
 * Function command codes for io_ctl.
 */
#define LCD_On			1
#define LCD_Off			2
#define LCD_Clear		3
#define LCD_Reset		4
#define LCD_Cursor_Left		5
#define LCD_Cursor_Right	6
#define LCD_Disp_Left		7
#define LCD_Disp_Right		8
#define LCD_Set_Cursor		10
#define LCD_Home		11
#define LCD_Curr_Controller	12
#define LCD_Cursor_Off		14
#define LCD_Cursor_On		15
#define LCD_Set_Cursor_Pos	17
#define LCD_Blink_Off           18
#define LCD_Contr           19

#define CONTROLLER_1	0x1
#define CONTROLLER_2	0x2
#define CONTROLLER_BOTH	0x3

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

static int __devinit pcf8574_lcd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	pcf8574_lcd_client = client;
	drv_HD_I2C_load();
	lcd_present = 1;

	return 0;
}

static int __devexit pcf8574_lcd_remove(struct i2c_client *client)
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
	.remove = __devexit_p(pcf8574_lcd_remove),
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

static const struct file_operations lcd_fops = {
	.owner = THIS_MODULE,
	.write = lcd_write,
	.ioctl = lcd_ioctl,
	.open  = lcd_open,
};

static struct miscdevice bfin_twi_lcd_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "lcd",
	.fops  = &lcd_fops
};

static int __init lcd_init(void)
{
	int result;
	pr_info("loaded\n");

	result = misc_register(&bfin_twi_lcd_dev);
	if (result < 0) {
		pr_err("unable to register misc device\n");
		return result;
	}

	return i2c_add_driver(&pcf8574_lcd_driver);
}
module_init(lcd_init);

static void __exit lcd_exit(void)
{
	i2c_del_driver(&pcf8574_lcd_driver);
	misc_deregister(&bfin_twi_lcd_dev);
	pr_info("goodbye\n");
}
module_exit(lcd_exit);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("TWI LCD driver (HD44780)");
MODULE_LICENSE("GPL");
