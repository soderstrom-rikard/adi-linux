/*
 * AD7476/5/7/8 (A) SPI ADC driver
 *
 * Copyright 2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/sysfs.h>
#include <linux/list.h>
#include <linux/spi/spi.h>

#include "../iio.h"
#include "../sysfs.h"

#include "ad7476.h"

static const struct ad7476_mode ad7476_mode_table[] = {
	{
		.name = "s0",
		.numvals = 1,
	},
};

static ssize_t ad7476_scan_direct(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7476_state *st = dev_info->dev_data;
	int ret;
	struct spi_device *spi = st->spi;

	ret = spi_sync(spi, &st->msg);
	if (ret)
		return ret;

	return sprintf(buf, "%d\n", (st->data[0] << 8) | st->data[1]);
}

static ssize_t ad7476_scan(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&dev_info->mlock);
	if (dev_info->currentmode == INDIO_RING_TRIGGERED)
		ret = ad7476_scan_from_ring(dev, attr, buf);
	else
		ret = ad7476_scan_direct(dev, attr, buf);
	mutex_unlock(&dev_info->mlock);

	return ret;
}

/* Cannot query the device, so use local copy of state */
static ssize_t ad7476_show_scan_mode(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct iio_dev *dev_info = dev_get_drvdata(dev);
	struct ad7476_state *st = dev_info->dev_data;

	return sprintf(buf, "%s\n", st->current_mode->name);
}


IIO_DEV_ATTR_AVAIL_SCAN_MODES(ad7476_show_scan_mode);
IIO_DEV_ATTR_SCAN_MODE(S_IRUGO | S_IWUSR,
		       ad7476_show_scan_mode, NULL);

IIO_DEV_ATTR_SCAN(ad7476_scan);

static ssize_t ad7476_show_name(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	return sprintf(buf, "%s\n", CHIP_NAME);
}

IIO_DEVICE_ATTR(name, S_IRUGO, ad7476_show_name, NULL, 0);

/*name export */

static struct attribute *ad7476_attributes[] = {
	&iio_dev_attr_available_scan_modes.dev_attr.attr,
	&iio_dev_attr_scan_mode.dev_attr.attr,
	&iio_dev_attr_scan.dev_attr.attr,
	&iio_dev_attr_name.dev_attr.attr,
	NULL,
};

static const struct attribute_group ad7476_attribute_group = {
	.attrs = ad7476_attributes,
};

static int __devinit ad7476_probe(struct spi_device *spi)
{
	int ret;
	struct ad7476_state *st = kzalloc(sizeof(*st), GFP_KERNEL);
	if (st == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}

	spi_set_drvdata(spi, st);

	atomic_set(&st->protect_ring, 0);
	st->spi = spi;

	st->indio_dev = iio_allocate_device();
	if (st->indio_dev == NULL) {
		ret = -ENOMEM;
		goto error_free_st;
	}

	/* Estabilish that the iio_dev is a child of the i2c device */
	st->indio_dev->dev.parent = &spi->dev;
	st->indio_dev->attrs = &ad7476_attribute_group;
	st->indio_dev->dev_data = (void *)(st);
	st->indio_dev->driver_module = THIS_MODULE;
	st->indio_dev->modes = INDIO_DIRECT_MODE;

	st->current_mode = &ad7476_mode_table[0];

	/* Setup default message */

	st->xfer.rx_buf = &st->data;
	st->xfer.len = st->current_mode->numvals * 2;

	spi_message_init(&st->msg);
	spi_message_add_tail(&st->xfer, &st->msg);

	ret = ad7476_register_ring_funcs_and_init(st->indio_dev);
	if (ret)
		goto error_free_device;

	ret = iio_device_register(st->indio_dev);
	if (ret)
		goto error_free_device;

	ret = ad7476_initialize_ring(st->indio_dev->ring);
	if (ret)
		goto error_cleanup_ring;
	return 0;

error_cleanup_ring:
	ad7476_ring_cleanup(st->indio_dev);
	iio_device_unregister(st->indio_dev);
error_free_device:
	iio_free_device(st->indio_dev);
error_free_st:
	kfree(st);
error_ret:
	return ret;
}

static int ad7476_remove(struct spi_device *spi)
{
	struct ad7476_state *st = spi_get_drvdata(spi);
	struct iio_dev *indio_dev = st->indio_dev;
	ad7476_uninitialize_ring(indio_dev->ring);
	ad7476_ring_cleanup(indio_dev);
	iio_device_unregister(indio_dev);
	kfree(st);

	return 0;
}


static struct spi_driver ad7476_driver = {
	.driver = {
		.name	= "ad7476",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= ad7476_probe,
	.remove		= __devexit_p(ad7476_remove),
};

static int __init ad7476_init(void)
{
	return spi_register_driver(&ad7476_driver);
}
module_init(ad7476_init);

static void __exit ad7476_exit(void)
{
	spi_unregister_driver(&ad7476_driver);
}
module_exit(ad7476_exit);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("Analog Devices AD7475/6/7/8(A) ADC");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("spi:ad7476");
