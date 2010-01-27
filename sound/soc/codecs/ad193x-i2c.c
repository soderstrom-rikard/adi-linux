/*
 * AD193X Audio Codec I2C bus driver
 *
 * Copyright 2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include "ad193x.h"

static int __devinit ad193x_i2c_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	return ad193x_bus_probe(&client->dev, client);
}

static int __devexit ad193x_i2c_remove(struct i2c_client *client)
{
	return ad193x_bus_remove(&client->dev);
}

static const struct i2c_device_id ad193x_id[] = {
	{ "ad1937", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ad193x_id);

static struct i2c_driver ad193x_i2c_driver = {
	.driver = {
		.name = "ad193x_captouch",
	},
	.probe    = ad193x_i2c_probe,
	.remove   = __devexit_p(ad193x_i2c_remove),
	.id_table = ad193x_id,
};

static __init int ad193x_i2c_init(void)
{
	return i2c_add_driver(&ad193x_i2c_driver);
}
module_init(ad193x_i2c_init);

static __exit void ad193x_i2c_exit(void)
{
	i2c_del_driver(&ad193x_i2c_driver);
}
module_exit(ad193x_i2c_exit);

MODULE_DESCRIPTION("ASoC ad193x i2c driver");
MODULE_AUTHOR("Barry Song <21cnbao@gmail.com>");
MODULE_LICENSE("GPL");
