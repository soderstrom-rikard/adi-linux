/*
 * ADXRS450 Digital Output Gyroscope Driver
 *
 * Copyright 2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/list.h>

#include "../iio.h"
#include "../sysfs.h"
#include "gyro.h"
#include "../adc/adc.h"

#include "adxrs450.h"

#define DRIVER_NAME		"adxrs450"

/* ADXRS450 only support 32-bit spi transfer, all the registers are read only */


/**
 * adxrs450_spi_read_reg_16() - read 2 bytes from a register pair
 * @dev: device associated with child of actual device (iio_dev or iio_trig)
 * @reg_address: the address of the lower of the two registers,which should be an even address,
 * Second register's address is reg_address + 1.
 * @val: somewhere to pass back the value read
 **/
static int adxrs450_spi_read_reg_16(struct device *dev,
		u8 reg_address,
		u16 *val)
{
	struct spi_message msg;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct adxrs450_state *st = iio_dev_get_devdata(indio_dev);
	int ret;
	struct spi_transfer xfers[] = {
		{
			.tx_buf = st->tx,
			.bits_per_word = 32,
			.len = 4,
			.cs_change = 1,
		}, {
			.rx_buf = st->rx,
			.bits_per_word = 32,
			.len = 4,
			.cs_change = 1,
		},
	};

	mutex_lock(&st->buf_lock);
	st->tx[0] = ADXRS450_READ_REG(reg_address);
	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);
	ret = spi_sync(st->us, &msg);
	if (ret) {
		dev_err(&st->us->dev, "problem while reading 16 bit register 0x%02x\n",
				reg_address);
		goto error_ret;
	}
	*val = (st->rx[0] >> 5);

error_ret:
	mutex_unlock(&st->buf_lock);
	return ret;
}

/**
 * adxrs450_spi_sensor_data() - read 2 bytes sensor data
 * @dev: device associated with child of actual device (iio_dev or iio_trig)
 * @val: somewhere to pass back the value read
 **/
static int adxrs450_spi_sensor_data(struct device *dev,
		u32 *val, char chk)
{
	struct spi_message msg;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct adxrs450_state *st = iio_dev_get_devdata(indio_dev);
	int ret;
	struct spi_transfer xfers[] = {
		{
			.tx_buf = st->tx,
			.bits_per_word = 32,
			.len = 4,
			.cs_change = 1,
		}, {
			.rx_buf = st->rx,
			.bits_per_word = 32,
			.len = 4,
			.cs_change = 1,
		},
	};

	mutex_lock(&st->buf_lock);
	st->tx[0] = ADXRS450_SENSOR_DATA;
	if (chk)
		st->tx[0] |= 0x2;
	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);
	ret = spi_sync(st->us, &msg);
	if (ret) {
		dev_err(&st->us->dev, "problem while reading sensor data\n");
		goto error_ret;
	}

	*val = st->rx[0];

error_ret:
	mutex_unlock(&st->buf_lock);
	return ret;
}

static ssize_t adxrs450_read_rate(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int ret, len = 0;
	u16 t;
	ret = adxrs450_spi_read_reg_16(dev,
			ADXRS450_RATE1,
			&t);
	if (ret)
		return ret;
	len = sprintf(buf, "%d\n", t);
	return len;
}

static ssize_t adxrs450_read_temp(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int ret, len = 0;
	u16 t;
	ret = adxrs450_spi_read_reg_16(dev,
			ADXRS450_TEMP1,
			&t);
	if (ret)
		return ret;
	len = sprintf(buf, "%d\n", t);
	return len;
}

static ssize_t adxrs450_read_locst(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int ret, len = 0;
	u16 t;
	ret = adxrs450_spi_read_reg_16(dev,
			ADXRS450_LOCST1,
			&t);
	if (ret)
		return ret;
	len = sprintf(buf, "%d\n", t);
	return len;
}

static ssize_t adxrs450_read_hicst(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int ret, len = 0;
	u16 t;
	ret = adxrs450_spi_read_reg_16(dev,
			ADXRS450_HICST1,
			&t);
	if (ret)
		return ret;
	len = sprintf(buf, "%d\n", t);
	return len;
}

static ssize_t adxrs450_read_quad(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int ret, len = 0;
	u16 t;
	ret = adxrs450_spi_read_reg_16(dev,
			ADXRS450_QUAD1,
			&t);
	if (ret)
		return ret;
	len = sprintf(buf, "%d\n", t);
	return len;
}

static ssize_t adxrs450_read_fault(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int ret, len = 0;
	u16 t;
	ret = adxrs450_spi_read_reg_16(dev,
			ADXRS450_FAULT1,
			&t);
	if (ret)
		return ret;
	len = sprintf(buf, "%d\n", t);
	return len;
}

static ssize_t adxrs450_read_pid(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int ret, len = 0;
	u16 t;
	ret = adxrs450_spi_read_reg_16(dev,
			ADXRS450_PID1,
			&t);
	if (ret)
		return ret;
	len = sprintf(buf, "%d\n", t);
	return len;
}

static ssize_t adxrs450_read_sn(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int ret, len = 0;
	u16 t0, t1;
	u32 t;
	ret = adxrs450_spi_read_reg_16(dev,
			ADXRS450_SNH,
			&t0);
	if (ret)
		return ret;

	ret = adxrs450_spi_read_reg_16(dev,
			ADXRS450_SNL,
			&t1);
	if (ret)
		return ret;
	t = (t0 << 16) | t1;
	len = sprintf(buf, "%d\n", t);
	return len;
}

static ssize_t adxrs450_read_dnc(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int ret, len = 0;
	u16 t;
	ret = adxrs450_spi_read_reg_16(dev,
			ADXRS450_DNC1,
			&t);
	if (ret)
		return ret;
	len = sprintf(buf, "%d\n", t);
	return len;
}

static ssize_t adxrs450_read_sensor_data(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int ret, len = 0;
	u32 t;
	u16 data;
	ret = adxrs450_spi_sensor_data(dev, &t, 0);
	if (ret)
		return ret;
	data = (u16)(t >> 10);
	len = sprintf(buf, "%d\n", data);
	return len;
}

/* Recommended Startup Sequence by spec */
static int adxrs450_initial_setup(struct adxrs450_state *st)
{
	u32 t;
	int ret;
	struct device *dev = &st->indio_dev->dev;
	/* use low spi speed for init */
	st->us->max_speed_hz = ADXRS450_SPI_SLOW;
	st->us->mode = SPI_MODE_0;
	spi_setup(st->us);

	msleep(ADXRS450_STARTUP_DELAY*2);

	ret = adxrs450_spi_sensor_data(dev, &t, 1);
	if (ret)
		return ret;
	if (t != 0x00000001)
		return -ENODEV;

	msleep(ADXRS450_STARTUP_DELAY);
	ret = adxrs450_spi_sensor_data(dev, &t, 0);
	if (ret)
		return ret;
	msleep(ADXRS450_STARTUP_DELAY);
	ret = adxrs450_spi_sensor_data(dev, &t, 0);
	if (ret)
		return ret;
	if (((t & 0xff) != 0xff) && (ADXRS450_GET_ST(t) != 2))
		return -EIO;
	ret = adxrs450_spi_sensor_data(dev, &t, 0);
	if (ret)
		return ret;
	if (((t & 0xff) != 0xff) && (ADXRS450_GET_ST(t) != 2))
		return -EIO;

	printk(KERN_INFO DRIVER_NAME ": at CS%d\n",
			st->us->chip_select);

	return 0;
}

static IIO_DEVICE_ATTR(sensor_data, S_IRUGO,
		adxrs450_read_sensor_data, NULL, 0);
static IIO_DEVICE_ATTR(rate, S_IRUGO,
		adxrs450_read_rate, NULL, 0);
static IIO_DEVICE_ATTR(temperature, S_IRUGO,
		adxrs450_read_temp, NULL, 0);
static IIO_DEVICE_ATTR(low_cst, S_IRUGO,
		adxrs450_read_locst, NULL, 0);
static IIO_DEVICE_ATTR(high_cst, S_IRUGO,
		adxrs450_read_hicst, NULL, 0);
static IIO_DEVICE_ATTR(quad, S_IRUGO,
		adxrs450_read_quad, NULL, 0);
static IIO_DEVICE_ATTR(fault, S_IRUGO,
		adxrs450_read_fault, NULL, 0);
static IIO_DEVICE_ATTR(part_id, S_IRUGO,
		adxrs450_read_pid, NULL, 0);
static IIO_DEVICE_ATTR(serial_number, S_IRUGO,
		adxrs450_read_sn, NULL, 0);
static IIO_DEVICE_ATTR(dynamic_null_correction, S_IRUGO,
		adxrs450_read_dnc, NULL, 0);


static IIO_CONST_ATTR(name, "adxrs450");

static struct attribute *adxrs450_attributes[] = {

	&iio_dev_attr_sensor_data.dev_attr.attr,
	&iio_dev_attr_rate.dev_attr.attr,
	&iio_dev_attr_temperature.dev_attr.attr,
	&iio_dev_attr_low_cst.dev_attr.attr,
	&iio_dev_attr_high_cst.dev_attr.attr,
	&iio_dev_attr_quad.dev_attr.attr,
	&iio_dev_attr_fault.dev_attr.attr,
	&iio_dev_attr_part_id.dev_attr.attr,
	&iio_dev_attr_serial_number.dev_attr.attr,
	&iio_dev_attr_dynamic_null_correction.dev_attr.attr,
	&iio_const_attr_name.dev_attr.attr,
	NULL
};

static const struct attribute_group adxrs450_attribute_group = {
	.attrs = adxrs450_attributes,
};

static int __devinit adxrs450_probe(struct spi_device *spi)
{
	int ret, regdone = 0;
	struct adxrs450_state *st = kzalloc(sizeof *st, GFP_KERNEL);
	if (!st) {
		ret =  -ENOMEM;
		goto error_ret;
	}
	/* this is only used for removal purposes */
	spi_set_drvdata(spi, st);

	/* Allocate the comms buffers */
	st->rx = kzalloc(sizeof(*st->rx)*ADXRS450_MAX_RX, GFP_KERNEL);
	if (st->rx == NULL) {
		ret = -ENOMEM;
		goto error_free_st;
	}
	st->tx = kzalloc(sizeof(*st->tx)*ADXRS450_MAX_TX, GFP_KERNEL);
	if (st->tx == NULL) {
		ret = -ENOMEM;
		goto error_free_rx;
	}
	st->us = spi;
	mutex_init(&st->buf_lock);
	/* setup the industrialio driver allocated elements */
	st->indio_dev = iio_allocate_device();
	if (st->indio_dev == NULL) {
		ret = -ENOMEM;
		goto error_free_tx;
	}

	st->indio_dev->dev.parent = &spi->dev;
	st->indio_dev->attrs = &adxrs450_attribute_group;
	st->indio_dev->dev_data = (void *)(st);
	st->indio_dev->driver_module = THIS_MODULE;
	st->indio_dev->modes = INDIO_DIRECT_MODE;

	ret = iio_device_register(st->indio_dev);
	if (ret)
		goto error_free_dev;
	regdone = 1;

	/* Get the device into a sane initial state */
	ret = adxrs450_initial_setup(st);
	if (ret)
		goto error_initial;
	return 0;

error_initial:
error_free_dev:
	if (regdone)
		iio_device_unregister(st->indio_dev);
	else
		iio_free_device(st->indio_dev);
error_free_tx:
	kfree(st->tx);
error_free_rx:
	kfree(st->rx);
error_free_st:
	kfree(st);
error_ret:
	return ret;
}

/* fixme, confirm ordering in this function */
static int adxrs450_remove(struct spi_device *spi)
{
	struct adxrs450_state *st = spi_get_drvdata(spi);
	struct iio_dev *indio_dev = st->indio_dev;

	iio_device_unregister(indio_dev);
	kfree(st->tx);
	kfree(st->rx);
	kfree(st);

	return 0;
}

static struct spi_driver adxrs450_driver = {
	.driver = {
		.name = "adxrs450",
		.owner = THIS_MODULE,
	},
	.probe = adxrs450_probe,
	.remove = __devexit_p(adxrs450_remove),
};

static __init int adxrs450_init(void)
{
	return spi_register_driver(&adxrs450_driver);
}
module_init(adxrs450_init);

static __exit void adxrs450_exit(void)
{
	spi_unregister_driver(&adxrs450_driver);
}
module_exit(adxrs450_exit);

MODULE_AUTHOR("Cliff Cai <cliff.cai@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADXRS450 Gyroscope SPI driver");
MODULE_LICENSE("GPL v2");
