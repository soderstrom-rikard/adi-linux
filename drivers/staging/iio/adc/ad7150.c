/*
 * AD7150 capacitive sensor driver supporting AD7150/1/6
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
 * AD7150 registers definition
 */

#define AD7150_STATUS              0
#define AD7150_STATUS_OUT1         (1 << 3)
#define AD7150_STATUS_OUT2         (1 << 5)
#define AD7150_CH1_DATA_HIGH       1
#define AD7150_CH1_DATA_LOW        2
#define AD7150_CH2_DATA_HIGH       3
#define AD7150_CH2_DATA_LOW        4
#define AD7150_CH1_AVG_HIGH        5
#define AD7150_CH1_AVG_LOW         6
#define AD7150_CH2_AVG_HIGH        7
#define AD7150_CH2_AVG_LOW         8
#define AD7150_CH1_SENSITIVITY     9
#define AD7150_CH1_THR_HOLD_H      9
#define AD7150_CH1_TIMEOUT         10
#define AD7150_CH1_THR_HOLD_L      10
#define AD7150_CH1_SETUP           11
#define AD7150_CH2_SENSITIVITY     12
#define AD7150_CH2_THR_HOLD_H      12
#define AD7150_CH2_TIMEOUT         13
#define AD7150_CH2_THR_HOLD_L      13
#define AD7150_CH2_SETUP           14
#define AD7150_CFG                 15
#define AD7150_CFG_FIX             (1 << 7)
#define AD7150_PD_TIMER            16
#define AD7150_CH1_CAPDAC          17
#define AD7150_CH2_CAPDAC          18
#define AD7150_SN3                 19
#define AD7150_SN2                 20
#define AD7150_SN1                 21
#define AD7150_SN0                 22
#define AD7150_ID                  23


/*
 * struct ad7150_chip_info - chip specifc information
 */

struct ad7150_chip_info {
	const char *name;
	struct i2c_client *client;
	struct iio_dev *indio_dev;
	struct iio_work_cont		work_cont_thresh;
	bool				inter;
	s64				last_timestamp;
	u16 ch1_threshold;     /* Ch1 Threshold (in fixed threshold mode) */
	u8  ch1_sensitivity;   /* Ch1 Sensitivity (in adaptive threshold mode) */
	u8  ch1_timeout;       /* Ch1 Timeout (in adaptive threshold mode) */
	u16 ch2_threshold;     /* Ch2 Threshold (in fixed threshold mode) */
	u8  ch2_sensitivity;   /* Ch1 Sensitivity (in adaptive threshold mode) */
	u8  ch2_timeout;       /* Ch1 Timeout (in adaptive threshold mode) */
	char current_mode[10]; /* adaptive/fixed threshold mode */
	int old_state;
};

/*
 * ad7150 register access by I2C
 */

static int ad7150_i2c_read(struct ad7150_chip_info *chip, u8 reg, u8 *data)
{
	struct i2c_client *client = chip->client;
	int ret = 0;
	u8 rx;

	ret = i2c_master_send(client, &reg, 1);
	if (ret < 0) {
		dev_err(&client->dev, "I2C write error\n");
		return ret;
	}

	ret = i2c_master_recv(client, &rx, 1);
	if (ret < 0) {
		dev_err(&client->dev, "I2C read error\n");
		return ret;
	}

	*data = rx;

	return ret;
}

static int ad7150_i2c_write(struct ad7150_chip_info *chip, u8 reg, u8 data)
{
	struct i2c_client *client = chip->client;
	int ret = 0;

	u8 tx[2] = {
		reg,
		data,
	};

	ret = i2c_master_send(client, tx, 2);
	if (ret < 0)
		dev_err(&client->dev, "I2C write error\n");

	return ret;
}

/*
 * sysfs nodes
 */

#define IIO_DEV_ATTR_AVAIL_THRESHOLD_MODES(_show)				\
	IIO_DEVICE_ATTR(available_threshold_modes, S_IRUGO, _show, NULL, 0)
#define IIO_DEV_ATTR_THRESHOLD_MODE(_mode, _show, _store)		\
	IIO_DEVICE_ATTR(threshold_mode, _mode, _show, _store, 0)
#define IIO_DEV_ATTR_CH1_THRESHOLD(_mode, _show, _store)              \
	IIO_DEVICE_ATTR(ch1_threshold, _mode, _show, _store, 0)
#define IIO_DEV_ATTR_CH2_THRESHOLD(_mode, _show, _store)              \
	IIO_DEVICE_ATTR(ch2_threshold, _mode, _show, _store, 0)
#define IIO_DEV_ATTR_CH1_SENSITIVITY(_mode, _show, _store)		\
	IIO_DEVICE_ATTR(ch1_sensitivity, _mode, _show, _store, 0)
#define IIO_DEV_ATTR_CH2_SENSITIVITY(_mode, _show, _store)		\
	IIO_DEVICE_ATTR(ch2_sensitivity, _mode, _show, _store, 0)
#define IIO_DEV_ATTR_CH1_TIMEOUT(_mode, _show, _store)		\
	IIO_DEVICE_ATTR(ch1_timeout, _mode, _show, _store, 0)
#define IIO_DEV_ATTR_CH2_TIMEOUT(_mode, _show, _store)		\
	IIO_DEVICE_ATTR(ch2_timeout, _mode, _show, _store, 0)
#define IIO_DEV_ATTR_CH1_VALUE(_show)		\
	IIO_DEVICE_ATTR(ch1_value, S_IRUGO, _show, NULL, 0)
#define IIO_DEV_ATTR_CH2_VALUE(_show)		\
	IIO_DEVICE_ATTR(ch2_value, S_IRUGO, _show, NULL, 0)

static ssize_t ad7150_show_threshold_modes(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	return sprintf(buf, "adaptive\nfixed\n");
}

IIO_DEV_ATTR_AVAIL_THRESHOLD_MODES(ad7150_show_threshold_modes);

static ssize_t ad7150_show_ch1_value(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7150_chip_info *chip = dev_info->dev_data;
	u8 data_l, data_h;

	ad7150_i2c_read(chip, AD7150_CH1_DATA_HIGH, &data_h);
	ad7150_i2c_read(chip, AD7150_CH1_DATA_LOW, &data_l);
	return sprintf(buf, "%d\n", (data_h << 8) | data_l);
}

IIO_DEV_ATTR_CH1_VALUE(ad7150_show_ch1_value);

static ssize_t ad7150_show_ch2_value(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7150_chip_info *chip = dev_info->dev_data;
	u8 data_l, data_h;

	ad7150_i2c_read(chip, AD7150_CH2_DATA_HIGH, &data_h);
	ad7150_i2c_read(chip, AD7150_CH2_DATA_LOW, &data_l);
	return sprintf(buf, "%d\n", (data_h << 8) | data_l);
}

IIO_DEV_ATTR_CH2_VALUE(ad7150_show_ch2_value);


static ssize_t ad7150_show_threshold_mode(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7150_chip_info *chip = dev_info->dev_data;

	return sprintf(buf, "%s\n", chip->current_mode);
}

static ssize_t ad7150_store_threshold_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7150_chip_info *chip = dev_info->dev_data;
	u8 cfg;

	ad7150_i2c_read(chip, AD7150_CFG, &cfg);

	if (strncmp(buf, "fixed", 5) == 0) {
		strcpy(chip->current_mode, "fixed");
		cfg |= AD7150_CFG_FIX;
		ad7150_i2c_write(chip, AD7150_CFG, cfg);

		return len;
	} else if (strncmp(buf, "adaptive", 8) == 0) {
		strcpy(chip->current_mode, "adaptive");
		cfg &= ~AD7150_CFG_FIX;
		ad7150_i2c_write(chip, AD7150_CFG, cfg);

		return len;
	}

	dev_err(dev, "not supported threshold mode\n");
	return -EINVAL;
}

IIO_DEV_ATTR_THRESHOLD_MODE(S_IRUGO | S_IWUSR,
		ad7150_show_threshold_mode,
		ad7150_store_threshold_mode);

static ssize_t ad7150_show_ch1_threshold(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7150_chip_info *chip = dev_info->dev_data;

	return sprintf(buf, "%d\n", chip->ch1_threshold);
}

static ssize_t ad7150_store_ch1_threshold(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7150_chip_info *chip = dev_info->dev_data;
	unsigned long data;
	int ret;

	ret = strict_strtoul(buf, 10, &data);

	if ((!ret) && (data < 0x10000)) {
		ad7150_i2c_write(chip, AD7150_CH1_THR_HOLD_H, data >> 8);
		ad7150_i2c_write(chip, AD7150_CH1_THR_HOLD_L, data);
		chip->ch1_threshold = data;
		return len;
	}

	return -EINVAL;
}

IIO_DEV_ATTR_CH1_THRESHOLD(S_IRUGO | S_IWUSR,
		ad7150_show_ch1_threshold,
		ad7150_store_ch1_threshold);

static ssize_t ad7150_show_ch2_threshold(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7150_chip_info *chip = dev_info->dev_data;

	return sprintf(buf, "%d\n", chip->ch2_threshold);
}

static ssize_t ad7150_store_ch2_threshold(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7150_chip_info *chip = dev_info->dev_data;
	unsigned long data;
	int ret;

	ret = strict_strtoul(buf, 10, &data);

	if ((!ret) && (data < 0x10000)) {
		ad7150_i2c_write(chip, AD7150_CH2_THR_HOLD_H, data >> 8);
		ad7150_i2c_write(chip, AD7150_CH2_THR_HOLD_L, data);
		chip->ch2_threshold = data;
		return len;
	}

	return -EINVAL;
}

IIO_DEV_ATTR_CH2_THRESHOLD(S_IRUGO | S_IWUSR,
		ad7150_show_ch2_threshold,
		ad7150_store_ch2_threshold);

static ssize_t ad7150_show_ch1_sensitivity(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7150_chip_info *chip = dev_info->dev_data;

	return sprintf(buf, "%d\n", chip->ch1_sensitivity);
}

static ssize_t ad7150_store_ch1_sensitivity(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7150_chip_info *chip = dev_info->dev_data;
	unsigned long data;
	int ret;

	ret = strict_strtoul(buf, 10, &data);

	if ((!ret) && (data < 0x100)) {
		ad7150_i2c_write(chip, AD7150_CH1_SENSITIVITY, data);
		chip->ch1_sensitivity = data;
		return len;
	}

	return -EINVAL;
}

IIO_DEV_ATTR_CH1_SENSITIVITY(S_IRUGO | S_IWUSR,
		ad7150_show_ch1_sensitivity,
		ad7150_store_ch1_sensitivity);

static ssize_t ad7150_show_ch2_sensitivity(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7150_chip_info *chip = dev_info->dev_data;

	return sprintf(buf, "%d\n", chip->ch2_sensitivity);
}

static ssize_t ad7150_store_ch2_sensitivity(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7150_chip_info *chip = dev_info->dev_data;
	unsigned long data;
	int ret;

	ret = strict_strtoul(buf, 10, &data);

	if ((!ret) && (data < 0x100)) {
		ad7150_i2c_write(chip, AD7150_CH2_SENSITIVITY, data);
		chip->ch2_sensitivity = data;
		return len;
	}

	return -EINVAL;
}

IIO_DEV_ATTR_CH2_SENSITIVITY(S_IRUGO | S_IWUSR,
		ad7150_show_ch2_sensitivity,
		ad7150_store_ch2_sensitivity);

static ssize_t ad7150_show_ch1_timeout(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7150_chip_info *chip = dev_info->dev_data;

	return sprintf(buf, "%d\n", chip->ch1_timeout);
}

static ssize_t ad7150_store_ch1_timeout(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7150_chip_info *chip = dev_info->dev_data;
	unsigned long data;
	int ret;

	ret = strict_strtoul(buf, 10, &data);

	if ((!ret) && (data < 0x100)) {
		ad7150_i2c_write(chip, AD7150_CH1_TIMEOUT, data);
		chip->ch1_timeout = data;
		return len;
	}

	return -EINVAL;
}

IIO_DEV_ATTR_CH1_TIMEOUT(S_IRUGO | S_IWUSR,
		ad7150_show_ch1_timeout,
		ad7150_store_ch1_timeout);

static ssize_t ad7150_show_ch2_timeout(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7150_chip_info *chip = dev_info->dev_data;

	return sprintf(buf, "%d\n", chip->ch2_timeout);
}

static ssize_t ad7150_store_ch2_timeout(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7150_chip_info *chip = dev_info->dev_data;
	unsigned long data;
	int ret;

	ret = strict_strtoul(buf, 10, &data);

	if ((!ret) && (data < 0x100)) {
		ad7150_i2c_write(chip, AD7150_CH2_TIMEOUT, data);
		chip->ch2_timeout = data;
		return len;
	}

	return -EINVAL;
}

IIO_DEV_ATTR_CH2_TIMEOUT(S_IRUGO | S_IWUSR,
		ad7150_show_ch2_timeout,
		ad7150_store_ch2_timeout);

static ssize_t ad7150_show_name(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7150_chip_info *chip = dev_info->dev_data;
	return sprintf(buf, "%s\n", chip->name);
}

IIO_DEVICE_ATTR(name, S_IRUGO, ad7150_show_name, NULL, 0);

static struct attribute *ad7150_attributes[] = {
	&iio_dev_attr_available_threshold_modes.dev_attr.attr,
	&iio_dev_attr_threshold_mode.dev_attr.attr,
	&iio_dev_attr_ch1_threshold.dev_attr.attr,
	&iio_dev_attr_ch2_threshold.dev_attr.attr,
	&iio_dev_attr_ch1_timeout.dev_attr.attr,
	&iio_dev_attr_ch2_timeout.dev_attr.attr,
	&iio_dev_attr_ch1_sensitivity.dev_attr.attr,
	&iio_dev_attr_ch2_sensitivity.dev_attr.attr,
	&iio_dev_attr_ch1_value.dev_attr.attr,
	&iio_dev_attr_ch2_value.dev_attr.attr,
	&iio_dev_attr_name.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad7150_attribute_group = {
	.attrs = ad7150_attributes,
};

/*
 * threshold events
 */

#define IIO_EVENT_CODE_CH1_HIGH    (IIO_EVENT_CODE_DEVICE_SPECIFIC + 1)
#define IIO_EVENT_CODE_CH1_LOW     (IIO_EVENT_CODE_DEVICE_SPECIFIC + 2)
#define IIO_EVENT_CODE_CH2_HIGH    (IIO_EVENT_CODE_DEVICE_SPECIFIC + 3)
#define IIO_EVENT_CODE_CH2_LOW     (IIO_EVENT_CODE_DEVICE_SPECIFIC + 4)

#define IIO_EVENT_ATTR_CH1_HIGH_SH(_evlist, _show, _store, _mask)	\
	IIO_EVENT_ATTR_SH(ch1_high, _evlist, _show, _store, _mask)

#define IIO_EVENT_ATTR_CH2_HIGH_SH(_evlist, _show, _store, _mask)	\
	IIO_EVENT_ATTR_SH(ch2_high, _evlist, _show, _store, _mask)

#define IIO_EVENT_ATTR_CH1_LOW_SH(_evlist, _show, _store, _mask)	\
	IIO_EVENT_ATTR_SH(ch1_low, _evlist, _show, _store, _mask)

#define IIO_EVENT_ATTR_CH2_LOW_SH(_evlist, _show, _store, _mask)	\
	IIO_EVENT_ATTR_SH(ch2_low, _evlist, _show, _store, _mask)

static void ad7150_interrupt_handler_bh(struct work_struct *work_s)
{
	struct iio_work_cont *wc
		= container_of(work_s, struct iio_work_cont, ws_nocheck);
	struct ad7150_chip_info *chip = wc->st;
	u8 int_status;

	enable_irq(chip->client->irq);

	ad7150_i2c_read(chip, AD7150_STATUS, &int_status);

	if ((int_status & AD7150_STATUS_OUT1) && !(chip->old_state & AD7150_STATUS_OUT1))
		iio_push_event(chip->indio_dev, 0,
				IIO_EVENT_CODE_CH1_HIGH,
				chip->last_timestamp);
	else if ((!(int_status & AD7150_STATUS_OUT1)) && (chip->old_state & AD7150_STATUS_OUT1))
		iio_push_event(chip->indio_dev, 0,
				IIO_EVENT_CODE_CH1_LOW,
				chip->last_timestamp);

	if ((int_status & AD7150_STATUS_OUT2) && !(chip->old_state & AD7150_STATUS_OUT2))
		iio_push_event(chip->indio_dev, 0,
				IIO_EVENT_CODE_CH2_HIGH,
				chip->last_timestamp);
	else if ((!(int_status & AD7150_STATUS_OUT2)) && (chip->old_state & AD7150_STATUS_OUT2))
		iio_push_event(chip->indio_dev, 0,
				IIO_EVENT_CODE_CH2_LOW,
				chip->last_timestamp);
}

static int ad7150_interrupt_handler_th(struct iio_dev *dev_info,
		int index,
		s64 timestamp,
		int no_test)
{
	struct ad7150_chip_info *chip = dev_info->dev_data;

	chip->last_timestamp = timestamp;
	schedule_work(&chip->work_cont_thresh.ws);

	return 0;
}

IIO_EVENT_SH(threshold, &ad7150_interrupt_handler_th);

static ssize_t ad7150_query_out_mode(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	/*
	 * AD7150 provides two logic output channels, which can be used as interrupt
	 * but the pins are not configurable
	 */
	return sprintf(buf, "1\n");
}

static ssize_t ad7150_set_out_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	return len;
}

IIO_EVENT_ATTR_CH1_HIGH_SH(iio_event_threshold, ad7150_query_out_mode, ad7150_set_out_mode, 0);
IIO_EVENT_ATTR_CH2_HIGH_SH(iio_event_threshold, ad7150_query_out_mode, ad7150_set_out_mode, 0);
IIO_EVENT_ATTR_CH1_LOW_SH(iio_event_threshold, ad7150_query_out_mode, ad7150_set_out_mode, 0);
IIO_EVENT_ATTR_CH2_LOW_SH(iio_event_threshold, ad7150_query_out_mode, ad7150_set_out_mode, 0);

static struct attribute *ad7150_event_attributes[] = {
	&iio_event_attr_ch1_high.dev_attr.attr,
	&iio_event_attr_ch2_high.dev_attr.attr,
	&iio_event_attr_ch1_low.dev_attr.attr,
	&iio_event_attr_ch2_low.dev_attr.attr,
	NULL,
};

static struct attribute_group ad7150_event_attribute_group = {
	.attrs = ad7150_event_attributes,
};

/*
 * device probe and remove
 */

static int __devinit ad7150_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0, regdone = 0;
	struct ad7150_chip_info *chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (chip == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}

	/* this is only used for device removal purposes */
	i2c_set_clientdata(client, chip);

	chip->client = client;
	chip->name = id->name;

	chip->indio_dev = iio_allocate_device();
	if (chip->indio_dev == NULL) {
		ret = -ENOMEM;
		goto error_free_chip;
	}

	/* Echipabilish that the iio_dev is a child of the i2c device */
	chip->indio_dev->dev.parent = &client->dev;
	chip->indio_dev->attrs = &ad7150_attribute_group;
	chip->indio_dev->event_attrs = &ad7150_event_attribute_group;
	chip->indio_dev->dev_data = (void *)(chip);
	chip->indio_dev->driver_module = THIS_MODULE;
	chip->indio_dev->num_interrupt_lines = 1;
	chip->indio_dev->modes = INDIO_DIRECT_MODE;

	ret = iio_device_register(chip->indio_dev);
	if (ret)
		goto error_free_dev;
	regdone = 1;

	if (client->irq && gpio_is_valid(irq_to_gpio(client->irq)) > 0) {
		iio_init_work_cont(&chip->work_cont_thresh,
				ad7150_interrupt_handler_bh,
				ad7150_interrupt_handler_bh,
				AD7150_STATUS,
				0,
				chip);
		ret = iio_register_interrupt_line(client->irq,
				chip->indio_dev,
				0,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				"ad7150");
		if (ret)
			goto error_free_dev;

		iio_add_event_to_list(iio_event_attr_ch2_low.listel,
				&chip->indio_dev->interrupts[0]->ev_list);

	}

	dev_err(&client->dev, "%s capacitive sensor registered, irq: %d\n", id->name, client->irq);

	return 0;

error_free_dev:
	if (regdone)
		iio_device_unregister(chip->indio_dev);
	else
		iio_free_device(chip->indio_dev);
error_free_chip:
	kfree(chip);
error_ret:
	return ret;
}

static int __devexit ad7150_remove(struct i2c_client *client)
{
	struct ad7150_chip_info *chip = i2c_get_clientdata(client);
	struct iio_dev *indio_dev = chip->indio_dev;

	if (client->irq && gpio_is_valid(irq_to_gpio(client->irq)) > 0)
		iio_unregister_interrupt_line(indio_dev, 0);
	iio_device_unregister(indio_dev);
	kfree(chip);

	return 0;
}

static const struct i2c_device_id ad7150_id[] = {
	{ "ad7150", 0 },
	{ "ad7151", 0 },
	{ "ad7156", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, ad7150_id);

static struct i2c_driver ad7150_driver = {
	.driver = {
		.name = "ad7150",
	},
	.probe = ad7150_probe,
	.remove = __devexit_p(ad7150_remove),
	.id_table = ad7150_id,
};

static __init int ad7150_init(void)
{
	return i2c_add_driver(&ad7150_driver);
}

static __exit void ad7150_exit(void)
{
	i2c_del_driver(&ad7150_driver);
}

MODULE_AUTHOR("Barry Song <21cnbao@gmail.com>");
MODULE_DESCRIPTION("Analog Devices ad7150/1/6 capacitive sensor driver");
MODULE_LICENSE("GPL v2");

module_init(ad7150_init);
module_exit(ad7150_exit);
