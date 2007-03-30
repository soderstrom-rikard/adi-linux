/*
    pca9543.c - Part of lm_sensors, Linux kernel modules for hardware
             monitoring
    Copyright (c) 2000  Frodo Looijaard <frodol@dds.nl>,
                        Philip Edelbrock <phil@netroedge.com>,
                        Dan Eaton <dan.eaton@rocketlogix.com>
    Ported to Linux 2.6 by Aurelien Jarno <aurel32@debian.org> with
    the help of Jean Delvare <khali@linux-fr.org>

    Copyright (C) 2007 Michael Hennerich, Analog Devices Inc.
    			<hennerich@blackfin.uclinux.org>
	based on the pcf8574.c

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
*/

/* A few notes about the PCA9543:

*   The PCA9543 is a bi-directional translating switch, controlled by the
    I2C bus. The SCL/SDA upstream pair fans out to two downstream
    pairs, or channels. Any individual SCx/SDx channels or combination
    of channels can be selected, determined by the contents of the
    programmable control register. Two interrupt inputs, INT0 to INT3,
    one for each of the downstream pairs, are provided. One interrupt
    output INT, which acts as an AND of the two interrupt inputs, is
    provided.

*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>

/* Addresses to scan */
static unsigned short normal_i2c[] = { 0x70,0x71, 0x72, 0x73, I2C_CLIENT_END };

/* Insmod parameters */
I2C_CLIENT_INSMOD_1(pca9543);


/* Each client has this additional data */
struct pca9543_data {
	struct i2c_client client;

	u8 write;			/* Remember last written value */
};

static int pca9543_attach_adapter(struct i2c_adapter *adapter);
static int pca9543_detect(struct i2c_adapter *adapter, int address, int kind);
static int pca9543_detach_client(struct i2c_client *client);


/* This is the driver that will be inserted */
static struct i2c_driver pca9543_driver = {
	.driver = {
		.name	= "pca9543",
	},
	.id		= I2C_DRIVERID_PCF8574,
	.attach_adapter	= pca9543_attach_adapter,
	.detach_client	= pca9543_detach_client,
};

/* following are the sysfs callback functions */
static ssize_t show_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	return sprintf(buf, "%u\n", i2c_smbus_read_byte(client));
}

static DEVICE_ATTR(read, S_IRUGO, show_read, NULL);

static ssize_t show_write(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pca9543_data *data = i2c_get_clientdata(to_i2c_client(dev));
	return sprintf(buf, "%u\n", data->write);
}

static ssize_t set_write(struct device *dev, struct device_attribute *attr, const char *buf,
			 size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pca9543_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);

	if (val > 0xff)
		return -EINVAL;

	data->write = val;
	i2c_smbus_write_byte(client, data->write);
	return count;
}

static DEVICE_ATTR(write, S_IWUSR | S_IRUGO, show_write, set_write);

static struct attribute *pca9543_attributes[] = {
	&dev_attr_read.attr,
	&dev_attr_write.attr,
	NULL
};

static const struct attribute_group pca9543_attr_group = {
	.attrs = pca9543_attributes,
};

/*
 * Real code
 */

static int pca9543_attach_adapter(struct i2c_adapter *adapter)
{
	return i2c_probe(adapter, &addr_data, pca9543_detect);
}

/* This function is called by i2c_probe */
static int pca9543_detect(struct i2c_adapter *adapter, int address, int kind)
{
	struct i2c_client *new_client;
	struct pca9543_data *data;
	int err = 0;
	const char *client_name = "pca9543";

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		goto exit;

	/* OK. For now, we presume we have a valid client. We now create the
	   client structure, even though we cannot fill it completely yet. */
	if (!(data = kzalloc(sizeof(struct pca9543_data), GFP_KERNEL))) {
		err = -ENOMEM;
		goto exit;
	}

	new_client = &data->client;
	i2c_set_clientdata(new_client, data);
	new_client->addr = address;
	new_client->adapter = adapter;
	new_client->driver = &pca9543_driver;
	new_client->flags = 0;

	/* Now, we would do the remaining detection. But the PCF8574 is plainly
	   impossible to detect! Stupid chip. */

	/* Fill in the remaining client fields and put it into the global list */
	strlcpy(new_client->name, client_name, I2C_NAME_SIZE);

	/* Tell the I2C layer a new client has arrived */
	if ((err = i2c_attach_client(new_client)))
		goto exit_free;


	/* Register sysfs hooks */
	err = sysfs_create_group(&new_client->dev.kobj, &pca9543_attr_group);
	if (err)
		goto exit_detach;
	return 0;

      exit_detach:
	i2c_detach_client(new_client);
      exit_free:
	kfree(data);
      exit:
	return err;
}

static int pca9543_detach_client(struct i2c_client *client)
{
	int err;

	sysfs_remove_group(&client->dev.kobj, &pca9543_attr_group);

	if ((err = i2c_detach_client(client)))
		return err;

	kfree(i2c_get_clientdata(client));
	return 0;
}


static int __init pca9543_init(void)
{
	return i2c_add_driver(&pca9543_driver);
}

static void __exit pca9543_exit(void)
{
	i2c_del_driver(&pca9543_driver);
}


MODULE_AUTHOR
    ("Frodo Looijaard <frodol@dds.nl>, "
     "Philip Edelbrock <phil@netroedge.com>, "
     "Dan Eaton <dan.eaton@rocketlogix.com> "
     "and Aurelien Jarno <aurelien@aurel32.net>"
     "Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("PCA9543 driver");
MODULE_LICENSE("GPL");

module_init(pca9543_init);
module_exit(pca9543_exit);
