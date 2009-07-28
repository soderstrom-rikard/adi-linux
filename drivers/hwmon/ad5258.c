/*
 * Driver for the Analog Devices AD5258 single-channel digital
 * potentiometer.
 *
 * Copyright (c) 2009 Cyber Switching, Inc.
 * Author: Chris Verges <chrisv@cyberswitching.com>
 *
 * derived from ad5252.c
 * Copyright (c) 2006 Michael Hennerich <hennerich@blackfin.uclinux.org>
 *
 * derived from pcf8547.c
 * Copyright (c) 2000  Frodo Looijaard <frodol@dds.nl>,
 *                     Philip Edelbrock <phil@netroedge.com>,
 *                     Dan Eaton <dan.eaton@rocketlogix.com>
 * Ported to Linux 2.6 by Aurelien Jarno <aurel32@debian.org> with
 * the help of Jean Delvare <khali@linux-fr.org>
 *
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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#define DRIVER_NAME			"ad5258"
#define DRIVER_VERSION			"0.1"

#define AD5258_RDAC_MASK		(0x3F)		/* 6-bit */

/* RDAC-to-EEPROM Interface Commands */
#define AD5258_I2C_RDAC			(0x00 << 5)
#define AD5258_I2C_EEPROM		(0x01 << 5)
#define AD5258_I2C_WPREG		(0x02 << 5)
#define AD5258_NOP			(0x04 << 5)
#define AD5258_RESTORE_FROM_EEPROM	(0x05 << 5)
#define AD5258_STORE_TO_EEPROM		(0x06 << 5)

/* Registers */
#define AD5258_REG_RDAC			(0x00 << 0)	/* 1 byte  */
#define AD5258_REG_TOLERANCE		(0x1E << 0)	/* 2 bytes */

static s32 ad5258_read(struct i2c_client *client, u8 reg);
static s32 ad5258_write(struct i2c_client *client, u8 reg, u8 value);

/*
 * Client data (each client gets its own)
 */

struct dpot_data {
	struct mutex update_lock;
};


/* ------------------------------------------------------------------------- */

/* sysfs functions */

static ssize_t show_rdac(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct dpot_data *data = i2c_get_clientdata(client);
	s32 value;

	mutex_lock(&data->update_lock);
	value = ad5258_read(client, AD5258_I2C_RDAC | AD5258_REG_RDAC);
	mutex_unlock(&data->update_lock);

	if (value < 0)
		return -EINVAL;

	return sprintf(buf, "%u\n", value & AD5258_RDAC_MASK);
}

static ssize_t set_rdac(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct dpot_data *data = i2c_get_clientdata(client);
	unsigned long value;
	int err;

	err = strict_strtoul(buf, 10, &value);
	if (err)
		return err;

	if (value > AD5258_RDAC_MASK)
		value = AD5258_RDAC_MASK;

	mutex_lock(&data->update_lock);
	ad5258_write(client, AD5258_I2C_RDAC | AD5258_REG_RDAC, value);
	mutex_unlock(&data->update_lock);

	return count;
}

static DEVICE_ATTR(rdac, S_IWUSR | S_IRUGO, show_rdac, set_rdac);

static ssize_t show_eeprom(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct dpot_data *data = i2c_get_clientdata(client);
	s32 value;

	mutex_lock(&data->update_lock);
	value = ad5258_read(client, AD5258_I2C_EEPROM | AD5258_REG_RDAC);
	mutex_unlock(&data->update_lock);

	if (value < 0)
		return -EINVAL;

	return sprintf(buf, "%u\n", value & AD5258_RDAC_MASK);
}

static ssize_t set_eeprom(struct device *dev,
				struct device_attribute *attr,
				const char *buf,
				size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct dpot_data *data = i2c_get_clientdata(client);
	unsigned long value;
	int err;

	err = strict_strtoul(buf, 10, &value);
	if (err)
		return err;

	if (value > AD5258_RDAC_MASK)
		value = AD5258_RDAC_MASK;

	mutex_lock(&data->update_lock);
	ad5258_write(client, AD5258_I2C_EEPROM | AD5258_REG_RDAC, value);
	msleep(26); /* Sleep while the EEPROM updates */
	mutex_unlock(&data->update_lock);

	return count;
}

static DEVICE_ATTR(eeprom, S_IWUSR | S_IRUGO, show_eeprom, set_eeprom);

static ssize_t show_tolerance(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct dpot_data *data = i2c_get_clientdata(client);
	s32 value;

	mutex_lock(&data->update_lock);
	value = ad5258_read(client, AD5258_I2C_EEPROM | AD5258_REG_TOLERANCE);
	mutex_unlock(&data->update_lock);

	if (value < 0)
		return 0;

	/*
	 * Let someone else deal with converting this ...
	 * the tolerance is a two-byte value where the MSB
	 * is a sign + integer value, and the LSB is a
	 * decimal value.  See page 18 of the AD5258
	 * datasheet (Rev. A) for more details.
	 */
	return sprintf(buf, "0x%04x\n", value & 0x0000FFFF);
}

static DEVICE_ATTR(tolerance, S_IRUGO, show_tolerance, NULL);

/* ------------------------------------------------------------------------- */

/* i2c device functions */

/**
 * ad5258_read - return the value contained in the specified register
 * on the AD5258 device.
 * @client: value returned from i2c_new_device()
 * @reg: the register to read
 *
 * If the tolerance register is specified, 2 bytes are returned.
 * Otherwise, 1 byte is returned.  A negative value indicates an error
 * occurred while reading the register.
 */
static s32 ad5258_read(struct i2c_client *client, u8 reg)
{
	if ((reg & 0x1F) == AD5258_REG_TOLERANCE)
		return i2c_smbus_read_word_data(client, reg);
	else
		return i2c_smbus_read_byte_data(client, reg);
}

/**
 * ad5258_write - store the given value in the specified register on
 * the AD5258 device.
 * @client: value returned from i2c_new_device()
 * @reg: the register to write
 * @value: the byte to store in the register
 *
 * For certain instructions that do not require a data byte, "NULL"
 * should be specified for the "value" parameter.  These instructions
 * include NOP, RESTORE_FROM_EEPROM, and STORE_TO_EEPROM.
 *
 * A negative return value indicates an error occurred while reading
 * the register.
 */
static s32 ad5258_write(struct i2c_client *client, u8 reg, u8 value)
{
	/* Do not attempt to write the tolerance register */
	if ((reg & 0x1f) == AD5258_REG_TOLERANCE)
		return -EINVAL;

	/* Only write the instruction byte for certain commands */
	switch (reg & 0xE0) {
	case AD5258_NOP:
	case AD5258_RESTORE_FROM_EEPROM:
	case AD5258_STORE_TO_EEPROM:
		return i2c_smbus_write_byte(client, reg);
	}

	/* All other registers require instruction + data bytes */
	return i2c_smbus_write_byte_data(client, reg, value);
}

static int ad5258_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct dpot_data *data;
	int err = 0;

	dev_dbg(dev, "%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(dev, "missing I2C functionality for this driver\n");
		goto exit;
	}

	data = kzalloc(sizeof(struct dpot_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, data);
	mutex_init(&data->update_lock);

	/* Register sysfs hooks */
	err |= device_create_file(dev, &dev_attr_rdac);
	err |= device_create_file(dev, &dev_attr_eeprom);
	err |= device_create_file(dev, &dev_attr_tolerance);

	if (err)
		dev_err(dev, "ad5258 failed to register sysfs hooks\n");

	dev_info(dev, "ad5258 client created\n");
	return 0;

exit:
	dev_err(dev, "failed to create ad5258 client\n");
	return err;
}

static int __devexit ad5258_remove(struct i2c_client *client)
{
	struct dpot_data *data = i2c_get_clientdata(client);
	struct device *dev = &client->dev;

	device_remove_file(dev, &dev_attr_rdac);
	device_remove_file(dev, &dev_attr_eeprom);
	device_remove_file(dev, &dev_attr_tolerance);

	i2c_set_clientdata(client, NULL);
	kfree(data);

	return 0;
}

struct i2c_device_id ad5258_idtable[] = {
	{ DRIVER_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ad5258_idtable);

static struct i2c_driver ad5258_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= DRIVER_NAME,
	},
	.id_table	= ad5258_idtable,
	.probe		= ad5258_probe,
	.remove		= __devexit_p(ad5258_remove),
};

static int __init ad5258_init(void)
{
	return i2c_add_driver(&ad5258_driver);
}

module_init(ad5258_init);

static void __exit ad5258_exit(void)
{
	i2c_del_driver(&ad5258_driver);
}

module_exit(ad5258_exit);

MODULE_AUTHOR(
	"Chris Verges <chrisv@cyberswitching.com>, "
	"Michael Hennerich <hennerich@blackfin.uclinux.org>, "
	"Frodo Looijaard <frodol@dds.nl>, "
	"Philip Edelbrock <phil@netroedge.com>, "
	"Dan Eaton <dan.eaton@rocketlogix.com> "
	"and Aurelien Jarno <aurelien@aurel32.net>");
MODULE_DESCRIPTION("AD5258 digital potentiometer driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
