/*
 * AD7879-1/AD7889-1 touchscreen (I2C bus)
 *
 * Copyright (C) 2008-2010 Michael Hennerich, Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/input.h>	/* BUS_I2C */
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/types.h>

#include "ad7879.h"

#define AD7879_DEVID		0x79	/* AD7879-1/AD7889-1 */

#ifdef CONFIG_PM
static int ad7879_i2c_suspend(struct i2c_client *client, pm_message_t message)
{
	return ad7879_disable(&client->dev);
}

static int ad7879_i2c_resume(struct i2c_client *client)
{
	return ad7879_enable(&client->dev);
}
#else
# define ad7879_i2c_suspend NULL
# define ad7879_i2c_resume  NULL
#endif

/* All registers are word-sized.
 * AD7879 uses a high-byte first convention.
 */
static int ad7879_i2c_read(void *client, u8 reg)
{
	return swab16(i2c_smbus_read_word_data(client, reg));
}

static int ad7879_i2c_multi_read(void *client, u8 first_reg, u8 count, u16 *buf)
{
	u8 idx;
	for (idx = 0; idx < count; ++idx)
		buf[idx] = ad7879_i2c_read(client, first_reg + idx);
	return 0;
}

static int ad7879_i2c_write(void *client, u8 reg, u16 val)
{
	return i2c_smbus_write_word_data(client, reg, swab16(val));
}

static int __devinit ad7879_i2c_probe(struct i2c_client *client,
				      const struct i2c_device_id *id)
{
	struct ad7879_bus_ops bops = {
		.bus_data = client,
		.irq = client->irq,
		.read = ad7879_i2c_read,
		.multi_read = ad7879_i2c_multi_read,
		.write = ad7879_i2c_write,
	};

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WORD_DATA)) {
		dev_err(&client->dev, "SMBUS Word Data not Supported\n");
		return -EIO;
	}

	return ad7879_probe(&client->dev, &bops, AD7879_DEVID, BUS_I2C);
}

static int __devexit ad7879_i2c_remove(struct i2c_client *client)
{
	return ad7879_remove(&client->dev);
}

static const struct i2c_device_id ad7879_id[] = {
	{ "ad7879", 0 },
	{ "ad7889", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ad7879_id);

static struct i2c_driver ad7879_i2c_driver = {
	.driver = {
		.name	= "ad7879",
		.owner	= THIS_MODULE,
	},
	.probe		= ad7879_i2c_probe,
	.remove		= __devexit_p(ad7879_i2c_remove),
	.suspend	= ad7879_i2c_suspend,
	.resume		= ad7879_i2c_resume,
	.id_table	= ad7879_id,
};

static int __init ad7879_i2c_init(void)
{
	return i2c_add_driver(&ad7879_i2c_driver);
}
module_init(ad7879_i2c_init);

static void __exit ad7879_i2c_exit(void)
{
	i2c_del_driver(&ad7879_i2c_driver);
}
module_exit(ad7879_i2c_exit);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("AD7879(-1) touchscreen I2C bus driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:ad7879");