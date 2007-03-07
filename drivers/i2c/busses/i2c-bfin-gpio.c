/****************************************************************
 * Description:                                                 *
 *                                                              *
 * Maintainer: Meihui Fan <mhfan@ustc.edu>		        *
 *                                                              *
 * CopyRight (c)  2004  HHTech                                  *
 *   www.hhcn.com, www.hhcn.org                                 *
 *   All rights reserved.                                       *
 *                                                              *
 * This file is free software;                                  *
 *   you are free to modify and/or redistribute it   	        *
 *   under the terms of the GNU General Public Licence (GPL).   *
 *                                                              *
 ****************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>

#include <asm/blackfin.h>
#include <asm/gpio.h>

#define	I2C_HW_B_HHBF		    0x13

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
	gpio_set_value(CONFIG_I2C_BLACKFIN_GPIO_SCL, state);
}

static int hhbf_getsda(void *data)
{
	return (gpio_get_value(CONFIG_I2C_BLACKFIN_GPIO_SDA) != 0);
}


static struct i2c_algo_bit_data bit_hhbf_data = {
	.setsda  = hhbf_setsda,
	.setscl  = hhbf_setscl,
	.getsda  = hhbf_getsda,
	.udelay  = CONFIG_I2C_BLACKFIN_GPIO_CYCLE_DELAY,
	.timeout = HZ
};

static struct i2c_adapter hhbf_ops = {
	.owner 	= THIS_MODULE,
	.id 	= I2C_HW_B_HHBF,
	.algo_data 	= &bit_hhbf_data,
	.name	= "Blackfin GPIO based I2C driver",
};

static int __init i2c_hhbf_init(void)
{

	if (gpio_request(CONFIG_I2C_BLACKFIN_GPIO_SCL, NULL)) {
		printk(KERN_ERR "%s: gpio_request GPIO %d failed \n",__func__, CONFIG_I2C_BLACKFIN_GPIO_SCL);
		return -1;
	}

	if (gpio_request(CONFIG_I2C_BLACKFIN_GPIO_SDA, NULL)) {
		printk(KERN_ERR "%s: gpio_request GPIO %d failed \n",__func__, CONFIG_I2C_BLACKFIN_GPIO_SDA);
		return -1;
	}


	gpio_direction_output(CONFIG_I2C_BLACKFIN_GPIO_SCL);
	gpio_direction_input(CONFIG_I2C_BLACKFIN_GPIO_SDA);
	gpio_set_value(CONFIG_I2C_BLACKFIN_GPIO_SCL, 1);

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
