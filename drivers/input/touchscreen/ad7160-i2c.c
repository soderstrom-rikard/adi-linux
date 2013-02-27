/*
 * AD7160  touchscreen (I2C bus)
 *
 * Copyright (C) 2010 Michael Hennerich, Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/input.h>	/* BUS_I2C */
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/types.h>

#include "ad7160.h"

#define AD7160_DEVID		0x7160	/* AD7160 */

#ifdef CONFIG_PM
static int ad7160_i2c_suspend(struct i2c_client *client, pm_message_t message)
{
	ad7160_suspend(&client->dev);
	return 0;
}

static int ad7160_i2c_resume(struct i2c_client *client)
{
	ad7160_resume(&client->dev);
	return 0;
}
#else
# define ad7160_i2c_suspend NULL
# define ad7160_i2c_resume  NULL
#endif

/* All registers are word-sized.
 * AD7160 uses be32 high byte first convention.
 */

static int ad7160_raw_i2c_read(void *dev, u32 reg, u32 len, u32 *data)
{
	struct i2c_client *client = dev;
	struct i2c_msg msg[2];
	u32 block_data;
	int ret, icnt;

	block_data = cpu_to_be32(reg);

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = REG_SIZE_BYTES;
	msg[0].buf = (char *)&block_data;

	msg[1].addr = client->addr;
	msg[1].flags = client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = len * REG_SIZE_BYTES;
	msg[1].buf = (char *)data;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != 2) {
		dev_err(&client->dev, "I2C read error: (%d) reg: 0x%X len: %d\n",
			ret, reg, len);
		return -EIO;
	}

	for (icnt = 0; icnt < len; icnt++)
		data[icnt] = be32_to_cpu(data[icnt]);

	return len;
}

static int ad7160_raw_i2c_write(void *dev, u32 reg, u32 len, u32 *data)
{
	struct i2c_client *client = dev;
	int ret, icnt;
	u32 block_data[MAX_DATA_CNT];

	block_data[0] = cpu_to_be32(reg);

	for (icnt = 0; icnt < len; icnt++)
		block_data[icnt + 1] = cpu_to_be32(data[icnt]);

	ret = i2c_master_send(client, (char *)block_data, (len + 1) * REG_SIZE_BYTES);
	if (ret < 0) {
		dev_err(&client->dev, "I2C write error\n");
		return ret;
	}
	return len;

}

static int ad7160_i2c_write_bytes(void *dev, u32 len, u8 *data)
{
	struct i2c_client *client = dev;

	int ret = i2c_master_send(client, data, len);
	if (ret < 0) {
		dev_err(&client->dev, "I2C write error\n");
		return ret;
	}
	return len;
}

/*
 * The AD7160 doesn't ACK on wake-up
 * So this is expected to generate a address NACK
 */

static void ad7160_i2c_wakeup(void *dev)
{
	struct i2c_client *client = dev;
	char dummy = 0;
	i2c_master_send(client, &dummy, 1);
}

static int ad7160_i2c_read(void *dev, u32 reg)
{
	struct i2c_client *client = dev;
	int ret;
	u32 retval;

	ret = ad7160_raw_i2c_read(client, reg, 1, &retval);

	return (ret < 0 ? ret : retval);
}

static int ad7160_i2c_write(void *dev, u32 reg, u32 val)
{
	struct i2c_client *client = dev;
	return ad7160_raw_i2c_write(client, reg, 1, &val);
}

static const struct ad7160_bus_ops bops = {
	.read = ad7160_i2c_read,
	.multi_read = ad7160_raw_i2c_read,
	.write = ad7160_i2c_write,
	.multi_write_bytes = ad7160_i2c_write_bytes,
	.wakeup = ad7160_i2c_wakeup,
};

static int ad7160_i2c_probe(struct i2c_client *client,
				      const struct i2c_device_id *id)
{
	int ret;
	struct ad7160_bus_data bdata = {
		.client = client,
		.irq = client->irq,
		.bops = &bops,
	};

	ret = ad7160_probe(&client->dev, &bdata, AD7160_DEVID, BUS_I2C);
	if (ret < 0)
		return ret;

	ret = ad7160_probe_raw(&client->dev, &bdata, AD7160_DEVID, BUS_I2C);

	if (ret < 0)
		dev_err(&client->dev, "failed to add raw data interface\n");

	return ret;
}

static int ad7160_i2c_remove(struct i2c_client *client)
{
	ad7160_remove_raw(&client->dev);
	return ad7160_remove(&client->dev);
}

static const struct i2c_device_id ad7160_id[] = {
	{ "ad7160", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ad7160_id);

static struct i2c_driver ad7160_i2c_driver = {
	.driver = {
		.name	= "ad7160",
		.owner	= THIS_MODULE,
	},
	.probe		= ad7160_i2c_probe,
	.remove		= ad7160_i2c_remove,
	.suspend	= ad7160_i2c_suspend,
	.resume		= ad7160_i2c_resume,
	.id_table	= ad7160_id,
};

static int __init ad7160_i2c_init(void)
{
	return i2c_add_driver(&ad7160_i2c_driver);
}
module_init(ad7160_i2c_init);

static void __exit ad7160_i2c_exit(void)
{
	i2c_del_driver(&ad7160_i2c_driver);
}
module_exit(ad7160_i2c_exit);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("AD7160 touchscreen I2C bus driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:ad7160");
