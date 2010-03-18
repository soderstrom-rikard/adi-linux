/*
 * AD7416 digital temperature sensor driver supporting AD7416/7/8
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
 * AD7416 registers definition
 */

#define AD7416_TEMPERATURE		0
#define AD7416_CONFIG			1
#define AD7416_T_OTI			2
#define AD7416_T_HYST			3
#define AD7416_ADC_VALUE		4
#define AD7416_CONFIG2			5

/*
 * AD7416 config
 */
#define AD7416_PD			0x1
#define AD7416_OTI_INT			0x2
#define AD7416_OTI_POLARITY		0x4
#define AD7416_FAULT_QUEUE_MASK		0x18
#define AD7416_FAULT_QUEUE_OFFSET	3
#define AD7416_CS_MASK			0xE0
#define AD7416_CS_OFFSET		5

/*
 * AD7416/7/8 channel maks
 */
#define AD7416_CHANNEL_MASK	0x1
#define AD7417_CHANNEL_MASK	0x1F
#define AD7418_CHANNEL_MASK	0x11

/*
 * AD7416 masks
 */
#define AD7416_VALUE_SIGN	0x200
#define AD7416_VALUE_OFFSET	6
#define AD7416_BOUND_VALUE_SIGN	0x100
#define AD7416_TEMP_OFFSET	7


/*
 * struct ad7416_chip_info - chip specifc information
 */

struct ad7416_chip_info {
	const char		*name;
	struct i2c_client	*client;
	struct iio_dev		*indio_dev;
	struct iio_work_cont	work_cont_thresh;
	s64			last_timestamp;
	u8			mode;
	u8			channel_id;	/* 0 always be temperature */
	u8			channel_mask;
};

/*
 * ad7416 register access by I2C
 */

static int ad7416_i2c_read(struct ad7416_chip_info *chip, u8 reg, u8 *data)
{
	struct i2c_client *client = chip->client;
	int ret = 0, len;

	ret = i2c_smbus_write_byte(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "I2C read register address error\n");
		return ret;
	}

	if (reg == AD7416_CONFIG || reg == AD7416_CONFIG2)
		len = 1;
	else
		len = 2;

	ret = i2c_master_recv(client, data, len);
	if (ret < 0) {
		dev_err(&client->dev, "I2C read error\n");
		return ret;
	}

	return ret;
}

static int ad7416_i2c_write(struct ad7416_chip_info *chip, u8 reg, u8 data)
{
	struct i2c_client *client = chip->client;
	int ret = 0;

	ret = i2c_smbus_write_byte_data(client, reg, data);
	if (ret < 0)
		dev_err(&client->dev, "I2C write error\n");

	return ret;
}

static ssize_t ad7416_show_mode(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7416_chip_info *chip = dev_info->dev_data;

	if (chip->mode)
		return sprintf(buf, "power-save\n");
	else
		return sprintf(buf, "full\n");
}

static ssize_t ad7416_store_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7416_chip_info *chip = dev_info->dev_data;
	u8 config;
	int ret;

	ret = ad7416_i2c_read(chip, AD7416_CONFIG, &config);
	if (ret)
		return -EIO;

	if (strcmp(buf, "full")) {
		chip->mode = 0;
		config &= ~AD7416_PD;
	} else {
		chip->mode = 1;
		config |= AD7416_PD;
	}

	ret = ad7416_i2c_write(chip, AD7416_CONFIG, config);
	if (ret)
		return -EIO;

	return ret;
}

IIO_DEVICE_ATTR(mode, S_IRUGO | S_IWUSR,
		ad7416_show_mode,
		ad7416_store_mode,
		0);

static ssize_t ad7416_show_available_modes(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "full\npower-save\n");
}

IIO_DEVICE_ATTR(available_modes, S_IRUGO, ad7416_show_available_modes, NULL, 0);

static ssize_t ad7416_show_channel(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7416_chip_info *chip = dev_info->dev_data;

	return sprintf(buf, "%d\n", chip->channel_id);
}

static ssize_t ad7416_store_channel(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7416_chip_info *chip = dev_info->dev_data;
	unsigned long data;
	u8 channel_mask;
	u8 config;
	int ret;

	ret = strict_strtoul(buf, 10, &data);
	if (ret)
		return -EINVAL;

	channel_mask = 1 << data;
	if (!(channel_mask & chip->channel_mask)) {
		dev_err(&chip->client->dev, "Invalid channel id %lu.\n", data);
		dev_err(&chip->client->dev, "Available channel mask 0x%x.\n",
				chip->channel_mask);
		return -EINVAL;
	} else if (data == chip->channel_id)
		return ret;

	ret = ad7416_i2c_read(chip, AD7416_CONFIG, &config);
	if (ret)
		return -EIO;

	config |= ((data << AD7416_CS_OFFSET) && AD7416_CS_MASK);
	ret = ad7416_i2c_write(chip, AD7416_CONFIG, config);
	if (ret)
		return -EIO;

	return ret;
}

IIO_DEVICE_ATTR(channel, S_IRUGO | S_IWUSR,
		ad7416_show_channel,
		ad7416_store_channel,
		0);


static ssize_t ad7416_show_value(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7416_chip_info *chip = dev_info->dev_data;
	u16 data;
	char sign = ' ';
	int ret;

	if (chip->mode) {
		/* write to active converter */
		ret = i2c_smbus_write_byte(chip->client, AD7416_TEMPERATURE);
		if (ret)
			return -EIO;
	}

	if (chip->channel_id == 0) {
		ret = ad7416_i2c_read(chip, AD7416_TEMPERATURE, (u8 *)&data);
		if (ret)
			return -EIO;

		data = be16_to_cpu(data);
		data >>= AD7416_VALUE_OFFSET;
		if (data & AD7416_VALUE_SIGN) {
			data = (AD7416_VALUE_SIGN << 1) - data;
			sign = '-';
		}

		return sprintf(buf, "%c%d.%.2d\n", sign, (data >> 2),
			 (data & 3) * 25);
	} else {
		ret = ad7416_i2c_read(chip, AD7416_ADC_VALUE, (u8 *)&data);
		if (ret)
			return -EIO;

		data = be16_to_cpu(data);
		data >>= AD7416_VALUE_OFFSET;

		return sprintf(buf, "%u\n", data);
	}
}

IIO_DEVICE_ATTR(value, S_IRUGO, ad7416_show_value, NULL, 0);

static ssize_t ad7416_show_name(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7416_chip_info *chip = dev_info->dev_data;
	return sprintf(buf, "%s\n", chip->name);
}

IIO_DEVICE_ATTR(name, S_IRUGO, ad7416_show_name, NULL, 0);

static struct attribute *ad7416_attributes[] = {
	&iio_dev_attr_available_modes.dev_attr.attr,
	&iio_dev_attr_mode.dev_attr.attr,
	&iio_dev_attr_channel.dev_attr.attr,
	&iio_dev_attr_value.dev_attr.attr,
	&iio_dev_attr_name.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad7416_attribute_group = {
	.attrs = ad7416_attributes,
};

/*
 * temperature bound events
 */

#define IIO_EVENT_CODE_AD7416_OTI    (IIO_EVENT_CODE_DEVICE_SPECIFIC + 1)

static void ad7416_interrupt_bh(struct work_struct *work_s)
{
	struct iio_work_cont *wc
		= container_of(work_s, struct iio_work_cont, ws_nocheck);
	struct ad7416_chip_info *chip = wc->st;

	enable_irq(chip->client->irq);

	iio_push_event(chip->indio_dev, 0,
			IIO_EVENT_CODE_AD7416_OTI,
			chip->last_timestamp);
}

static int ad7416_interrupt(struct iio_dev *dev_info,
		int index,
		s64 timestamp,
		int no_test)
{
	struct ad7416_chip_info *chip = dev_info->dev_data;

	chip->last_timestamp = timestamp;
	schedule_work(&chip->work_cont_thresh.ws);

	return 0;
}

IIO_EVENT_SH(ad7416, &ad7416_interrupt);

static ssize_t ad7416_show_oti_mode(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7416_chip_info *chip = dev_info->dev_data;
	u8 config;
	int ret;

	/* retrive ALART status */
	ret = ad7416_i2c_read(chip, AD7416_CONFIG, &config);
	if (ret)
		return -EIO;

	if (config & AD7416_OTI_INT)
		return sprintf(buf, "interrupt\n");
	else
		return sprintf(buf, "comparator\n");
}

static ssize_t ad7416_set_oti_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7416_chip_info *chip = dev_info->dev_data;
	u8 config;
	int ret;

	/* retrive ALART status */
	ret = ad7416_i2c_read(chip, AD7416_CONFIG, &config);
	if (ret)
		return -EIO;

	if (strcmp(buf, "comparator") == 0)
		ret = ad7416_i2c_write(chip, AD7416_CONFIG,
			config & ~AD7416_OTI_INT);
	else
		ret = ad7416_i2c_write(chip, AD7416_CONFIG,
			config | AD7416_OTI_INT);
	if (ret)
		return -EIO;

	return ret;
}

static ssize_t ad7416_show_available_oti_modes(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "comparator\ninterrupt\n");
}

static ssize_t ad7416_show_fault_queue(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7416_chip_info *chip = dev_info->dev_data;
	u8 config;
	int ret;

	/* retrive ALART status */
	ret = ad7416_i2c_read(chip, AD7416_CONFIG, &config);
	if (ret)
		return -EIO;

	return sprintf(buf, "%d\n", (config & AD7416_FAULT_QUEUE_MASK) >>
			AD7416_FAULT_QUEUE_OFFSET);
}

static ssize_t ad7416_set_fault_queue(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7416_chip_info *chip = dev_info->dev_data;
	unsigned long data;
	u8 config;
	int ret;

	ret = strict_strtoul(buf, 10, &data);
	if (ret || data > 3)
		return -EINVAL;

	/* retrive ALART status */
	ret = ad7416_i2c_read(chip, AD7416_CONFIG, &config);
	if (ret)
		return -EIO;

	config &= ~AD7416_FAULT_QUEUE_MASK;
	config |= (data << AD7416_FAULT_QUEUE_OFFSET);
	ret = ad7416_i2c_write(chip, AD7416_CONFIG, config);
	if (ret)
		return -EIO;

	return ret;
}
static inline ssize_t ad7416_show_t_bound(struct device *dev,
		struct device_attribute *attr,
		u8 bound_reg,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7416_chip_info *chip = dev_info->dev_data;
	u16 data;
	s16 value;
	int ret;

	ret = ad7416_i2c_read(chip, bound_reg, (u8 *)&data);
	if (ret)
		return -EIO;

	data = be16_to_cpu(data);
	data >>= (AD7416_VALUE_OFFSET + 1);

	if (chip->channel_id == 0) {
		if (data & AD7416_BOUND_VALUE_SIGN) {
			value = (s16)(data&(~AD7416_BOUND_VALUE_SIGN));
			value = value - AD7416_BOUND_VALUE_SIGN;
		}

		return sprintf(buf, "%d\n", value >> 1),
	} else
		return sprintf(buf, "%u\n", data);
}

static inline ssize_t ad7416_set_t_bound(struct device *dev,
		struct device_attribute *attr,
		u8 bound_reg,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7416_chip_info *chip = dev_info->dev_data;
	long value;
	u16 data;
	int ret;

	if (chip->channel_id == 0) {
		ret = strict_strtol(buf, 10, &value);

		if (ret || value < -128 || value > 127)
			return -EINVAL;
		if (value < 0)
			value = (AD7416_BOUND_VALUE_SIGN + value) |
				AD7416_BOUND_VALUE_SIGN;

		data = (u16)value << 1;
	} else {
		ret = strict_strtoul(buf, 10, &data);

		if (ret || data > 511)
			return -EINVAL;
	}

	data <<= (AD7416_BOUND_OFFSET + 1);
	data = cpu_to_be16(data);
	ret = ad7416_i2c_write(chip, bound_reg, (u8)data);
	if (ret)
		return -EIO;

	return ret;
}

static ssize_t ad7416_show_t_oti(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return ad7416_show_t_bound(dev, attr,
			AD7416_T_OTI, buf);
}

static inline ssize_t ad7416_set_t_oti(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	return ad7416_set_t_bound(dev, attr,
			AD7416_T_OTI, buf, len);
}

static ssize_t ad7416_show_t_hyst(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return ad7416_show_t_bound(dev, attr,
			AD7416_T_HYST, buf);
}

static inline ssize_t ad7416_set_t_hyst(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	return ad7416_set_t_bound(dev, attr,
			AD7416_T_HYST, buf, len);
}

IIO_EVENT_ATTR_SH(oti_mode, iio_event_ad7416,
		ad7416_show_oti_mode, ad7416_set_oti_mode, 0);
IIO_EVENT_ATTR_SH(available_oti_modes, iio_event_ad7416,
		ad7416_show_available_oti_modes, NULL, 0);
IIO_EVENT_ATTR_SH(fault_queue, iio_event_ad7416,
		ad7416_show_fault_queue, ad7416_set_fault_queue, 0);
IIO_EVENT_ATTR_SH(t_oti, iio_event_ad7416,
		ad7416_show_t_oti, ad7416_set_t_oti, 0);
IIO_EVENT_ATTR_SH(t_hyst, iio_event_ad7416,
		ad7416_show_t_hyst, ad7416_set_t_hyst, 0);

static struct attribute *ad7416_event_attributes[] = {
	&iio_event_attr_oti_mode.dev_attr.attr,
	&iio_event_attr_available_oti_modes.dev_attr.attr,
	&iio_event_attr_fault_queue.dev_attr.attr,
	&iio_event_attr_t_oti.dev_attr.attr,
	&iio_event_attr_t_hyst.dev_attr.attr,
	NULL,
};

static struct attribute_group ad7416_event_attribute_group = {
	.attrs = ad7416_event_attributes,
};

/*
 * device probe and remove
 */

static int __devinit ad7416_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct ad7416_chip_info *chip;
	int ret = 0;
	unsigned int irq_flags = (unsigned int)client->dev.platform_data;
	u8 config;

	if (!(irq_flags == IRQF_TRIGGER_HIGH ||
		irq_flags == IRQF_TRIGGER_LOW)) {
		dev_err(&client->dev, "Invalid ALART polarity defined.\n");
		return -EINVAL;
	}

	chip = kzalloc(sizeof(struct ad7416_chip_info), GFP_KERNEL);

	if (chip == NULL)
		return -ENOMEM;

	/* this is only used for device removal purposes */
	i2c_set_clientdata(client, chip);

	chip->client = client;
	chip->name = id->name;
	if (strcmp(chip->name, "ad7418") == 0)
		chip->channel_mask = AD7418_CHANNEL_MASK;
	else if (strcmp(chip->name, "ad7417") == 0)
		chip->channel_mask = AD7417_CHANNEL_MASK;
	else
		chip->channel_mask = AD7416_CHANNEL_MASK;

	chip->indio_dev = iio_allocate_device();
	if (chip->indio_dev == NULL) {
		ret = -ENOMEM;
		goto error_free_chip;
	}

	chip->indio_dev->dev.parent = &client->dev;
	chip->indio_dev->attrs = &ad7416_attribute_group;
	chip->indio_dev->event_attrs = &ad7416_event_attribute_group;
	chip->indio_dev->dev_data = (void *)chip;
	chip->indio_dev->driver_module = THIS_MODULE;
	chip->indio_dev->num_interrupt_lines = 1;
	chip->indio_dev->modes = INDIO_DIRECT_MODE;

	ret = iio_device_register(chip->indio_dev);
	if (ret)
		goto error_free_dev;

	if (client->irq && gpio_is_valid(irq_to_gpio(client->irq)) > 0) {
		iio_init_work_cont(&chip->work_cont_thresh,
				ad7416_interrupt_bh,
				ad7416_interrupt_bh,
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
		 * The event handler list element refer to iio_event_ad7416.
		 * All event attributes bind to the same event handler.
		 * So, only register event handler once.
		 */
		iio_add_event_to_list(&iio_event_ad7416,
				&chip->indio_dev->interrupts[0]->ev_list);

		ret = ad7416_i2c_read(chip, AD7416_CONFIG, &config);
		if (ret) {
			ret = -EIO;
			goto error_unreg_dev;
		}

		if (irq_flags == IRQF_TRIGGER_HIGH)
			ret = ad7416_i2c_write(chip, AD7416_CONFIG,
				config | AD7416_OTI_POLARITY);
		else
			ret = ad7416_i2c_write(chip, AD7416_CONFIG,
				config & ~AD7416_OTI_POLARITY);
		if (ret) {
			ret = -EIO;
			goto error_unreg_dev;
		}
	}

	dev_info(&client->dev, "%s temperature sensor and ADC registered.\n",
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

static int __devexit ad7416_remove(struct i2c_client *client)
{
	struct ad7416_chip_info *chip = i2c_get_clientdata(client);
	struct iio_dev *indio_dev = chip->indio_dev;

	if (client->irq && gpio_is_valid(irq_to_gpio(client->irq)) > 0)
		iio_unregister_interrupt_line(indio_dev, 0);
	iio_device_unregister(indio_dev);
	kfree(chip);

	return 0;
}

static const struct i2c_device_id ad7416_id[] = {
	{ "ad7416", 0 },
	{ "ad7417", 0 },
	{ "ad7418", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, ad7416_id);

static struct i2c_driver ad7416_driver = {
	.driver = {
		.name = "ad7416",
	},
	.probe = ad7416_probe,
	.remove = __devexit_p(ad7416_remove),
	.id_table = ad7416_id,
};

static __init int ad7416_init(void)
{
	return i2c_add_driver(&ad7416_driver);
}

static __exit void ad7416_exit(void)
{
	i2c_del_driver(&ad7416_driver);
}

MODULE_AUTHOR("Sonic Zhang <sonic.zhang@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7416/7/8 digital"
			" temperature sensor driver");
MODULE_LICENSE("GPL v2");

module_init(ad7416_init);
module_exit(ad7416_exit);
