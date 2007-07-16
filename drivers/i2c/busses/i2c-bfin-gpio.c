/*
 * File:        drivers/i2c/busses/i2c-bfin-gpio.c
 * Based on:
 * Author:      Meihui Fan <mhfan@ustc.edu>
 * 		Michael Hennerich (hennerich@blackfin.uclinux.org)
 *
 * Created:
 * Description: I2C Adapter for algo_bit using the GPIO layer
 *
 * Modified:	CopyRight (c)  2004  HHTech (www.hhcn.com, www.hhcn.org)
 *		Copyright 2005-2007 Analog Devices Inc.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>

#include <asm/blackfin.h>
#include <asm/gpio.h>

#define DRV_NAME	"i2c-bfin-gpio"

static void hhbf_setsda(void *data, int state)
{
	if (state) {
		gpio_direction_input(CONFIG_I2C_BLACKFIN_GPIO_SDA);
	} else {
		gpio_direction_output(CONFIG_I2C_BLACKFIN_GPIO_SDA);
		gpio_set_value(CONFIG_I2C_BLACKFIN_GPIO_SDA, 0);
	}
}

static void hhbf_setscl(void *data, int state)
{
	if (state) {
		gpio_direction_input(CONFIG_I2C_BLACKFIN_GPIO_SCL);
	} else {
		gpio_direction_output(CONFIG_I2C_BLACKFIN_GPIO_SCL);
		gpio_set_value(CONFIG_I2C_BLACKFIN_GPIO_SCL, 0);
	}
}

static int hhbf_getsda(void *data)
{
	gpio_direction_input(CONFIG_I2C_BLACKFIN_GPIO_SDA);

	return gpio_get_value(CONFIG_I2C_BLACKFIN_GPIO_SDA);
}

static int hhbf_getscl(void *data)
{
	gpio_direction_input(CONFIG_I2C_BLACKFIN_GPIO_SCL);

	return gpio_get_value(CONFIG_I2C_BLACKFIN_GPIO_SCL);
}

static struct i2c_algo_bit_data bit_hhbf_data = {
	.setsda  = hhbf_setsda,
	.setscl  = hhbf_setscl,
	.getsda  = hhbf_getsda,
	.getscl  = hhbf_getscl,
	.udelay  = CONFIG_I2C_BLACKFIN_GPIO_CYCLE_DELAY,
	.timeout = HZ
};

static struct i2c_adapter hhbf_ops = {
	.owner 	= THIS_MODULE,
	.id 	= I2C_HW_B_BLACKFIN,
	.algo_data 	= &bit_hhbf_data,
	.name	= "Blackfin GPIO based I2C driver",
};

static int __init i2c_hhbf_init(void)
{

	if (gpio_request(CONFIG_I2C_BLACKFIN_GPIO_SCL, DRV_NAME)) {
		printk(KERN_ERR DRV_NAME": gpio_request GPIO %d failed \n"
		, CONFIG_I2C_BLACKFIN_GPIO_SCL);
		return -EBUSY;
	}

	if (gpio_request(CONFIG_I2C_BLACKFIN_GPIO_SDA, DRV_NAME)) {
		printk(KERN_ERR DRV_NAME": gpio_request GPIO %d failed \n"
		, CONFIG_I2C_BLACKFIN_GPIO_SDA);
		return -EBUSY;
	}

	gpio_direction_input(CONFIG_I2C_BLACKFIN_GPIO_SCL);
	gpio_direction_input(CONFIG_I2C_BLACKFIN_GPIO_SDA);

	return i2c_bit_add_bus(&hhbf_ops);
}

static void __exit i2c_hhbf_exit(void)
{
	gpio_free(CONFIG_I2C_BLACKFIN_GPIO_SCL);
	gpio_free(CONFIG_I2C_BLACKFIN_GPIO_SDA);
	i2c_bit_del_bus(&hhbf_ops);
}

MODULE_AUTHOR("Meihui Fan <mhfan@ustc.edu>");
MODULE_DESCRIPTION("I2C-Bus adapter routines for Blackfin and HHBF Boards");
MODULE_LICENSE("GPL");

module_init(i2c_hhbf_init);
module_exit(i2c_hhbf_exit);
