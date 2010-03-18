/*
 * AD7414 digital temperature sensor driver supporting AD7414 and AD7415
 *
 * Copyright 2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/sysfs.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/rtc.h>

#include "../iio.h"
#include "../sysfs.h"

/*
 * AD7414 registers definition
 */

#define AD7414_TEMPERATURE		0
#define AD7414_CONFIG			1
#define AD7414_TEMPERATURE_HIGH		2
#define AD7414_TEMPERATURE_LOW		3

/*
 * AD7414 config bits
 */
#define AD7414_TEST_MODE	0x3
#define AD7414_ONE_SHOT		0x4
#define AD7414_ALERT_RESET	0x8
#define AD7414_ALERT_POLARITY	0x10
#define AD7414_ALERT_EN		0x20
#define AD7414_FLTR		0x40
#define AD7414_PD		0x80

/*
 * AD7414 masks
 */
#define AD7414_TEMP_SIGN	0x200
#define AD7414_TEMP_MASK	0xFFC0
#define AD7414_TEMP_OFFSET	6
#define AD7414_ALERT_FLAG	0x20
#define AD7414_T_SIGN		0x80
#define AD7414_T_HIGH_FLAG	0x10
#define AD7414_T_LOW_FLAG	0x8


/*
 * struct ad7414_chip_info - chip specifc information
 */

struct ad7414_chip_info {
	const char		*name;
	struct i2c_client	*client;
	struct iio_dev		*indio_dev;
	struct iio_work_cont	work_cont_thresh;
	s64			last_timestamp;
	u8			mode;
};

/*
 * ad7414 register access by I2C
 */

static int ad7414_i2c_read(struct ad7414_chip_info *chip, u8 reg, u8 *data)
{
	struct i2c_client *client = chip->client;
	int ret = 0, len;

	ret = i2c_smbus_write_byte(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "I2C read register address error\n");
		return ret;
	}

	if (reg == AD7414_TEMPERATURE)
		len = 2;
	else
		len = 1;

	ret = i2c_master_recv(client, data, len);
	if (ret < 0) {
		dev_err(&client->dev, "I2C read error\n");
		return ret;
	}

	return ret;
}

static int ad7414_i2c_write(struct ad7414_chip_info *chip, u8 reg, u8 data)
{
	struct i2c_client *client = chip->client;
	int ret = 0;

	ret = i2c_smbus_write_byte_data(client, reg, data);
	if (ret < 0)
		dev_err(&client->dev, "I2C write error\n");

	return ret;
}

static ssize_t ad7414_show_mode(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7414_chip_info *chip = dev_info->dev_data;

	if (chip->mode)
		return sprintf(buf, "power-save\n");
	else
		return sprintf(buf, "full\n");
}

static ssize_t ad7414_store_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7414_chip_info *chip = dev_info->dev_data;
	u8 config;
	int ret;

	ret = ad7414_i2c_read(chip, AD7414_CONFIG, &config);
	if (ret)
		return -EIO;

	if (strcmp(buf, "full")) {
		chip->mode = 0;
		config &= ~AD7414_PD;
	} else {
		chip->mode = 1;
		config |= AD7414_PD;
	}

	ret = ad7414_i2c_write(chip, AD7414_CONFIG, config);
	if (ret)
		return -EIO;

	return ret;
}

IIO_DEVICE_ATTR(mode, S_IRUGO | S_IWUSR,
		ad7414_show_mode,
		ad7414_store_mode,
		0);

static ssize_t ad7414_show_available_modes(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "full\npower-save\n");
}

IIO_DEVICE_ATTR(available_modes, S_IRUGO, ad7414_show_available_modes, NULL, 0);

static ssize_t ad7414_show_temperature(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7414_chip_info *chip = dev_info->dev_data;
	u8 config;
	u16 data;
	char sign = ' ';
	int ret;

	if (chip->mode) {
		ret = ad7414_i2c_read(chip, AD7414_CONFIG, &config);
		if (ret)
			return -EIO;
		ret = ad7414_i2c_write(chip, AD7414_CONFIG,
				config | AD7414_ONE_SHOT);
		if (ret)
			return -EIO;
	}

	ret = ad7414_i2c_read(chip, AD7414_TEMPERATURE, (u8 *)&data);
	if (ret)
		return -EIO;

	data = be16_to_cpu(data) >> AD7414_TEMP_OFFSET;
	if (data & AD7414_TEMP_SIGN) {
		data = (AD7414_TEMP_SIGN << 1) - data;
		sign = '-';
	}

	return sprintf(buf, "%c%d.%.2d\n", sign, (data >> 2), (data & 3) * 25);
}

IIO_DEVICE_ATTR(temperature, S_IRUGO, ad7414_show_temperature, NULL, 0);

static ssize_t ad7414_show_name(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7414_chip_info *chip = dev_info->dev_data;
	return sprintf(buf, "%s\n", chip->name);
}

IIO_DEVICE_ATTR(name, S_IRUGO, ad7414_show_name, NULL, 0);

static struct attribute *ad7414_attributes[] = {
	&iio_dev_attr_available_modes.dev_attr.attr,
	&iio_dev_attr_mode.dev_attr.attr,
	&iio_dev_attr_temperature.dev_attr.attr,
	&iio_dev_attr_name.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad7414_attribute_group = {
	.attrs = ad7414_attributes,
};

/*
 * temperature bound events
 */

#define IIO_EVENT_CODE_AD7414_T_HIGH    (IIO_EVENT_CODE_DEVICE_SPECIFIC + 1)
#define IIO_EVENT_CODE_AD7414_T_LOW     (IIO_EVENT_CODE_DEVICE_SPECIFIC + 2)

static void ad7414_interrupt_bh(struct work_struct *work_s)
{
	struct iio_work_cont *wc
		= container_of(work_s, struct iio_work_cont, ws_nocheck);
	struct ad7414_chip_info *chip = wc->st;
	u16 status;
	u8 config;
	int ret;

	/* retrive ALART status */
	ret = ad7414_i2c_read(chip, AD7414_TEMPERATURE, (u8 *)&status);
	if (ret)
		return;
	status = be16_to_cpu(status);

	/* clear ALART pin in chip configuration register */
	ret = ad7414_i2c_read(chip, AD7414_CONFIG, &config);
	if (ret)
		return;
	ret = ad7414_i2c_write(chip, AD7414_CONFIG,
			config | AD7414_ALERT_RESET);
	if (ret)
		return;

	enable_irq(chip->client->irq);

	if (status & AD7414_T_HIGH_FLAG)
		iio_push_event(chip->indio_dev, 0,
				IIO_EVENT_CODE_AD7414_T_HIGH,
				chip->last_timestamp);
	else if (status & AD7414_T_LOW_FLAG)
		iio_push_event(chip->indio_dev, 0,
				IIO_EVENT_CODE_AD7414_T_LOW,
				chip->last_timestamp);
}

static int ad7414_interrupt(struct iio_dev *dev_info,
		int index,
		s64 timestamp,
		int no_test)
{
	struct ad7414_chip_info *chip = dev_info->dev_data;

	chip->last_timestamp = timestamp;
	schedule_work(&chip->work_cont_thresh.ws);

	return 0;
}

IIO_EVENT_SH(ad7414, &ad7414_interrupt);

static ssize_t ad7414_show_enabled(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7414_chip_info *chip = dev_info->dev_data;
	u8 config;
	int ret;

	/* retrive ALART status */
	ret = ad7414_i2c_read(chip, AD7414_CONFIG, &config);
	if (ret)
		return -EIO;

	return sprintf(buf, "%d\n", !!(config & AD7414_ALERT_EN));
}

static ssize_t ad7414_set_enabled(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7414_chip_info *chip = dev_info->dev_data;
	unsigned long data;
	u8 config;
	int ret;

	ret = strict_strtoul(buf, 10, &data);
	if (ret)
		return -EINVAL;

	/* retrive ALART status */
	ret = ad7414_i2c_read(chip, AD7414_CONFIG, &config);
	if (ret)
		return -EIO;

	if (data)
		ret = ad7414_i2c_write(chip, AD7414_CONFIG,
			config & ~AD7414_ALERT_EN);
	else
		ret = ad7414_i2c_write(chip, AD7414_CONFIG,
			config | AD7414_ALERT_EN);
	if (ret)
		return -EIO;

	return ret;
}

static inline ssize_t ad7414_show_temperature_bound(struct device *dev,
		struct device_attribute *attr,
		u8 bound_reg,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7414_chip_info *chip = dev_info->dev_data;
	s8 data;
	int ret;

	ret = ad7414_i2c_read(chip, bound_reg, &data);
	if (ret)
		return -EIO;

	if (data & AD7414_T_SIGN)
		data = data&(~AD7414_T_SIGN) - AD7414_T_SIGN;

	return sprintf(buf, "%d\n", data);
}

static inline ssize_t ad7414_set_temperature_bound(struct device *dev,
		struct device_attribute *attr,
		u8 bound_reg,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7414_chip_info *chip = dev_info->dev_data;
	long data;
	s8 value;
	int ret;

	ret = strict_strtol(buf, 10, &data);

	if (ret || data > 127 || data < -127)
		return -EINVAL;

	value = (s8)data;
	if (value < 0)
		value = (AD7414_T_SIGN + value) | AD7414_T_SIGN;

	ret = ad7414_i2c_write(chip, bound_reg, value);
	if (ret)
		return -EIO;

	return ret;
}

static ssize_t ad7414_show_temperature_high(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return ad7414_show_temperature_bound(dev, attr,
			AD7414_TEMPERATURE_HIGH, buf);
}

static inline ssize_t ad7414_set_temperature_high(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	return ad7414_set_temperature_bound(dev, attr,
			AD7414_TEMPERATURE_HIGH, buf, len);
}

static ssize_t ad7414_show_temperature_low(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return ad7414_show_temperature_bound(dev, attr,
			AD7414_TEMPERATURE_LOW, buf);
}

static inline ssize_t ad7414_set_temperature_low(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	return ad7414_set_temperature_bound(dev, attr,
			AD7414_TEMPERATURE_LOW, buf, len);
}

IIO_EVENT_ATTR_SH(t_bound_enabled, iio_event_ad7414,
		ad7414_show_enabled, ad7414_set_enabled, 0);
IIO_EVENT_ATTR_SH(temperature_high, iio_event_ad7414,
		ad7414_show_temperature_high, ad7414_set_temperature_high, 0);
IIO_EVENT_ATTR_SH(temperature_low, iio_event_ad7414,
		ad7414_show_temperature_low, ad7414_set_temperature_low, 0);

static struct attribute *ad7414_event_attributes[] = {
	&iio_event_attr_t_bound_enabled.dev_attr.attr,
	&iio_event_attr_temperature_high.dev_attr.attr,
	&iio_event_attr_temperature_low.dev_attr.attr,
	NULL,
};

static struct attribute_group ad7414_event_attribute_group = {
	.attrs = ad7414_event_attributes,
};

/*
 * device probe and remove
 */

static int __devinit ad7414_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct ad7414_chip_info *chip;
	int ret = 0;
	unsigned int irq_flags = (unsigned int)client->dev.platform_data;
	u8 config;

	if (!(irq_flags == IRQF_TRIGGER_HIGH ||
		irq_flags == IRQF_TRIGGER_LOW)) {
		dev_err(&client->dev, "Invalid ALART polarity defined.\n");
		return -EINVAL;
	}

	chip = kzalloc(sizeof(struct ad7414_chip_info), GFP_KERNEL);

	if (chip == NULL)
		return -ENOMEM;

	/* this is only used for device removal purposes */
	i2c_set_clientdata(client, chip);

	chip->client = client;
	chip->name = id->name;

	chip->indio_dev = iio_allocate_device();
	if (chip->indio_dev == NULL) {
		ret = -ENOMEM;
		goto error_free_chip;
	}

	chip->indio_dev->dev.parent = &client->dev;
	chip->indio_dev->attrs = &ad7414_attribute_group;
	if (strcmp(id->name, "ad7414") == 0)
		chip->indio_dev->event_attrs = &ad7414_event_attribute_group;
	else
		client->irq = 0;
	chip->indio_dev->dev_data = (void *)chip;
	chip->indio_dev->driver_module = THIS_MODULE;
	chip->indio_dev->num_interrupt_lines = 1;
	chip->indio_dev->modes = INDIO_DIRECT_MODE;

	ret = iio_device_register(chip->indio_dev);
	if (ret)
		goto error_free_dev;

	if (client->irq && gpio_is_valid(irq_to_gpio(client->irq)) > 0) {
		iio_init_work_cont(&chip->work_cont_thresh,
				ad7414_interrupt_bh,
				ad7414_interrupt_bh,
				0,
				0,
				chip);

		ret = iio_register_interrupt_line(client->irq,
				chip->indio_dev,
				0,
				irq_flags,
				chip->name);
		if (ret)
			goto error_unreg_dev;

		/*
		 * The event handler list element refer to iio_event_ad7414.
		 * All event attributes bind to the same event handler.
		 * So, only register event handler once.
		 */
		iio_add_event_to_list(&iio_event_ad7414,
				&chip->indio_dev->interrupts[0]->ev_list);

		ret = ad7414_i2c_read(chip, AD7414_CONFIG, &config);
		if (ret) {
			ret = -EIO;
			goto error_unreg_dev;
		}

		if (irq_flags == IRQF_TRIGGER_HIGH)
			ret = ad7414_i2c_write(chip, AD7414_CONFIG,
				config | AD7414_ALERT_POLARITY);
		else
			ret = ad7414_i2c_write(chip, AD7414_CONFIG,
				config & ~AD7414_ALERT_POLARITY);
		if (ret) {
			ret = -EIO;
			goto error_unreg_dev;
		}
	}

	dev_info(&client->dev, "%s temperature sensor registered.\n",
			 id->name);

	return 0;

error_unreg_dev:
	iio_device_unregister(chip->indio_dev);
error_free_dev:
	iio_free_device(chip->indio_dev);
error_free_chip:
	kfree(chip);

	return ret;
}

static int __devexit ad7414_remove(struct i2c_client *client)
{
	struct ad7414_chip_info *chip = i2c_get_clientdata(client);
	struct iio_dev *indio_dev = chip->indio_dev;

	if (client->irq && gpio_is_valid(irq_to_gpio(client->irq)) > 0)
		iio_unregister_interrupt_line(indio_dev, 0);
	iio_device_unregister(indio_dev);
	kfree(chip);

	return 0;
}

static const struct i2c_device_id ad7414_id[] = {
	{ "ad7414", 0 },
	{ "ad7415", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, ad7414_id);

static struct i2c_driver ad7414_driver = {
	.driver = {
		.name = "ad7414",
	},
	.probe = ad7414_probe,
	.remove = __devexit_p(ad7414_remove),
	.id_table = ad7414_id,
};

static __init int ad7414_init(void)
{
	return i2c_add_driver(&ad7414_driver);
}

static __exit void ad7414_exit(void)
{
	i2c_del_driver(&ad7414_driver);
}

MODULE_AUTHOR("Sonic Zhang <sonic.zhang@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7414 and AD7415 digital"
			" temperature sensor driver");
MODULE_LICENSE("GPL v2");

module_init(ad7414_init);
module_exit(ad7414_exit);
