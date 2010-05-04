/*
 * ADIS16220 Programmable Digital Vibration Sensor driver
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

#include <linux/sysfs.h>
#include <linux/list.h>

#include "../iio.h"
#include "../sysfs.h"
#include "accel.h"
#include "../imu/volt.h"
#include "../adc/adc.h"

#include "adis16220.h"

#define DRIVER_NAME		"adis16220"

/**
 * adis16220_spi_write_reg_8() - write single byte to a register
 * @dev: device associated with child of actual device (iio_dev or iio_trig)
 * @reg_address: the address of the register to be written
 * @val: the value to write
 **/
int adis16220_spi_write_reg_8(struct device *dev,
		u8 reg_address,
		u8 val)
{
	int ret;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct adis16220_state *st = iio_dev_get_devdata(indio_dev);

	mutex_lock(&st->buf_lock);
	st->tx[0] = ADIS16220_WRITE_REG(reg_address);
	st->tx[1] = val;

	ret = spi_write(st->us, st->tx, 2);
	mutex_unlock(&st->buf_lock);

	return ret;
}

/**
 * adis16220_spi_write_reg_16() - write 2 bytes to a pair of registers
 * @dev: device associated with child of actual device (iio_dev or iio_trig)
 * @reg_address: the address of the lower of the two registers. Second register
 *               is assumed to have address one greater.
 * @val: value to be written
 **/
static int adis16220_spi_write_reg_16(struct device *dev,
		u8 lower_reg_address,
		u16 value)
{
	int ret;
	struct spi_message msg;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct adis16220_state *st = iio_dev_get_devdata(indio_dev);
	struct spi_transfer xfers[] = {
		{
			.tx_buf = st->tx,
			.bits_per_word = 8,
			.len = 2,
			.cs_change = 1,
		}, {
			.tx_buf = st->tx + 2,
			.bits_per_word = 8,
			.len = 2,
			.cs_change = 1,
		},
	};

	mutex_lock(&st->buf_lock);
	st->tx[0] = ADIS16220_WRITE_REG(lower_reg_address);
	st->tx[1] = value & 0xFF;
	st->tx[2] = ADIS16220_WRITE_REG(lower_reg_address + 1);
	st->tx[3] = (value >> 8) & 0xFF;

	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);
	ret = spi_sync(st->us, &msg);
	mutex_unlock(&st->buf_lock);

	return ret;
}

/**
 * adis16220_spi_read_reg_16() - read 2 bytes from a 16-bit register
 * @dev: device associated with child of actual device (iio_dev or iio_trig)
 * @reg_address: the address of the lower of the two registers. Second register
 *               is assumed to have address one greater.
 * @val: somewhere to pass back the value read
 **/
static int adis16220_spi_read_reg_16(struct device *dev,
		u8 lower_reg_address,
		u16 *val)
{
	struct spi_message msg;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct adis16220_state *st = iio_dev_get_devdata(indio_dev);
	int ret;
	struct spi_transfer xfers[] = {
		{
			.tx_buf = st->tx,
			.bits_per_word = 8,
			.len = 2,
			.cs_change = 1,
		}, {
			.rx_buf = st->rx,
			.bits_per_word = 8,
			.len = 2,
			.cs_change = 1,
		},
	};

	mutex_lock(&st->buf_lock);
	st->tx[0] = ADIS16220_READ_REG(lower_reg_address);
	st->tx[1] = 0;
	st->tx[2] = 0;
	st->tx[3] = 0;

	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);
	ret = spi_sync(st->us, &msg);
	if (ret) {
		dev_err(&st->us->dev, "problem when reading 16 bit register 0x%02X",
				lower_reg_address);
		goto error_ret;
	}
	*val = (st->rx[0] << 8) | st->rx[1];

error_ret:
	mutex_unlock(&st->buf_lock);
	return ret;
}

/**
 * adis16220_spi_read_sequence() - read a sequence of 16-bit registers
 * @dev: device associated with child of actual device (iio_dev or iio_trig)
 * @tx: register addresses in bytes 0,2,4,6... (min size is 2*num bytes)
 * @rx: somewhere to pass back the value read (min size is 2*num bytes)
 **/
int adis16220_spi_read_sequence(struct device *dev,
		u8 *tx, u8 *rx, int num)
{
	struct spi_message msg;
	struct spi_transfer *xfers;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct adis16220_state *st = iio_dev_get_devdata(indio_dev);
	int ret, i;

	xfers = kzalloc(num + 1, GFP_KERNEL);
	if (xfers == NULL) {
		dev_err(&st->us->dev, "memory alloc failed");
		ret = -ENOMEM;
		goto error_ret;
	}

	/* tx: |add1|addr2|addr3|...|addrN |zero|
	 * rx: |zero|res1 |res2 |...|resN-1|resN| */
	spi_message_init(&msg);
	for (i = 0; i < num + 1; i++) {
		if (i > 0)
			xfers[i].rx_buf = st->rx + 2*(i - 1);
		if (i < num)
			xfers[i].tx_buf = st->tx + 2*i;
		xfers[i].bits_per_word = 8;
		xfers[i].len = 2;
		xfers[i].cs_change = 1;
		spi_message_add_tail(&xfers[i], &msg);
	}

	mutex_lock(&st->buf_lock);

	ret = spi_sync(st->us, &msg);
	if (ret)
		dev_err(&st->us->dev, "problem when reading sequence");

	mutex_unlock(&st->buf_lock);
	kfree(xfers);

error_ret:
	return ret;
}

static ssize_t adis16220_spi_read_signed(struct device *dev,
		struct device_attribute *attr,
		char *buf,
		unsigned bits)
{
	int ret;
	s16 val = 0;
	unsigned shift = 16 - bits;
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);

	ret = adis16220_spi_read_reg_16(dev, this_attr->address, (u16 *)&val);
	if (ret)
		return ret;

	val = ((s16)(val << shift) >> shift);
	return sprintf(buf, "%d\n", val);
}

static ssize_t adis16220_read_12bit_unsigned(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	int ret;
	u16 val = 0;
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);

	ret = adis16220_spi_read_reg_16(dev, this_attr->address, &val);
	if (ret)
		return ret;

	return sprintf(buf, "%u\n", val & 0x0FFF);
}

static ssize_t adis16220_read_16bit(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	ssize_t ret;

	/* Take the iio_dev status lock */
	mutex_lock(&indio_dev->mlock);
	ret =  adis16220_spi_read_signed(dev, attr, buf, 16);
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static ssize_t adis16220_read_12bit_signed(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	ssize_t ret;

	/* Take the iio_dev status lock */
	mutex_lock(&indio_dev->mlock);
	ret =  adis16220_spi_read_signed(dev, attr, buf, 12);
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static ssize_t adis16220_write_16bit(struct device *dev,
		struct device_attribute *attr,
		const char *buf,
		size_t len)
{
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret;
	long val;

	ret = strict_strtol(buf, 10, &val);
	if (ret)
		goto error_ret;
	ret = adis16220_spi_write_reg_16(dev, this_attr->address, val);

error_ret:
	return ret ? ret : len;
}

static ssize_t adis16220_write_reset(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	if (len < 1)
		return -1;
	switch (buf[0]) {
	case '1':
	case 'y':
	case 'Y':
		return adis16220_reset(dev);
	}
	return -1;
}

int adis16220_set_irq(struct device *dev, bool enable)
{
	int ret;
	u16 msc;
	ret = adis16220_spi_read_reg_16(dev, ADIS16220_MSC_CTRL, &msc);
	if (ret)
		goto error_ret;

	msc |= ADIS16220_MSC_CTRL_DIO1_ACT_HIGH;
	if (enable)
		msc |= ADIS16220_MSC_CTRL_DIO1_BUSY_IND;
	else
		msc &= ~ADIS16220_MSC_CTRL_DIO1_BUSY_IND;

	ret = adis16220_spi_write_reg_16(dev, ADIS16220_MSC_CTRL, msc);
	if (ret)
		goto error_ret;

error_ret:
	return ret;
}

int adis16220_reset(struct device *dev)
{
	int ret;
	ret = adis16220_spi_write_reg_8(dev,
			ADIS16220_GLOB_CMD,
			ADIS16220_GLOB_CMD_SW_RESET);
	if (ret)
		dev_err(dev, "problem resetting device");

	return ret;
}

int adis16220_self_test(struct device *dev)
{
	int ret;
	ret = adis16220_spi_write_reg_16(dev,
			ADIS16220_MSC_CTRL,
			ADIS16220_MSC_CTRL_SELF_TEST_EN);
	if (ret) {
		dev_err(dev, "problem starting self test");
		goto err_ret;
	}

	adis16220_check_status(dev);

err_ret:
	return ret;
}

int adis16220_check_status(struct device *dev)
{
	u16 status;
	int ret;

	ret = adis16220_spi_read_reg_16(dev, ADIS16220_DIAG_STAT, &status);

	if (ret < 0) {
		dev_err(dev, "Reading status failed\n");
		goto error_ret;
	}
	ret = status;
	if (status & ADIS16220_DIAG_STAT_ALM_MAG2)
		dev_err(dev, "AIN2 sample > ALM_MAG2\n");
	if (status & ADIS16220_DIAG_STAT_ALM_MAG1)
		dev_err(dev, "AIN1 sample > ALM_MAG1\n");
	if (status & ADIS16220_DIAG_STAT_ALM_MAGA)
		dev_err(dev, "Acceleration sample > ALM_MAGA\n");
	if (status & ADIS16220_DIAG_STAT_ALM_MAGS)
		dev_err(dev, "Error condition programmed into ALM_MAGS[11:0]\n");
	if (status & ADIS16220_DIAG_STAT_PEAK_AIN2)
		dev_err(dev, "|Peak value in AIN2 data capture| > ALM_MAG2\n");
	if (status & ADIS16220_DIAG_STAT_PEAK_AIN1)
		dev_err(dev, "|Peak value in AIN1 data capture| > ALM_MAG1\n");
	if (status & ADIS16220_DIAG_STAT_PEAK_ACCEL)
		dev_err(dev, "|Peak value in acceleration data capture| > ALM_MAGA\n");
	if (status & ADIS16220_DIAG_STAT_VIOLATION)
		dev_err(dev, "Capture period violation/interruption\n");
	if (status & ADIS16220_DIAG_STAT_SPI_FAIL)
		dev_err(dev, "SPI failure\n");
	if (status & ADIS16220_DIAG_STAT_FLASH_UPT)
		dev_err(dev, "Flash update failed\n");
	if (status & ADIS16220_DIAG_STAT_POWER_HIGH)
		dev_err(dev, "Power supply above 5.25V\n");
	if (status & ADIS16220_DIAG_STAT_POWER_LOW)
		dev_err(dev, "Power supply below 4.75V\n");

error_ret:
	return ret;
}

static int adis16220_initial_setup(struct adis16220_state *st)
{
	int ret;
	struct device *dev = &st->indio_dev->dev;

	/* Disable IRQ */
	ret = adis16220_set_irq(dev, false);
	if (ret) {
		dev_err(dev, "disable irq failed");
		goto err_ret;
	}

	/* Do self test */

	/* Read status register to check the result */
	ret = adis16220_check_status(dev);
	if (ret) {
		adis16220_reset(dev);
		dev_err(dev, "device not playing ball -> reset");
		msleep(ADIS16220_STARTUP_DELAY);
		ret = adis16220_check_status(dev);
		if (ret) {
			dev_err(dev, "giving up");
			goto err_ret;
		}
	}

	printk(KERN_INFO DRIVER_NAME ": at CS%d (irq %d)\n",
			st->us->chip_select, st->us->irq);

err_ret:
	return ret;
}

static IIO_DEV_ATTR_VOLT(supply, adis16220_read_12bit_unsigned,
		ADIS16220_CAPT_SUPPLY);
static IIO_CONST_ATTR(volt_supply_scale, "0.0012207");
static IIO_DEV_ATTR_ACCEL(adis16220_read_16bit,
		ADIS16220_CAPT_BUFA);
static IIO_DEV_ATTR_ACCEL_PEAK(adis16220_read_16bit,
		ADIS16220_CAPT_PEAKA);
static IIO_DEV_ATTR_ACCEL_OFFSET(S_IWUSR | S_IRUGO,
		adis16220_read_16bit,
		adis16220_write_16bit,
		ADIS16220_ACCL_NULL);
static IIO_DEV_ATTR_TEMP(adis16220_read_12bit_signed);
static IIO_CONST_ATTR(temp_offset, "25 K");
static IIO_CONST_ATTR(temp_scale, "-0.47 K");

static IIO_DEV_ATTR_ADC(1, adis16220_read_16bit, ADIS16220_CAPT_BUF1);
static IIO_DEV_ATTR_ADC(2, adis16220_read_16bit, ADIS16220_CAPT_BUF2);

static IIO_DEV_ATTR_RESET(adis16220_write_reset);

static IIO_CONST_ATTR_AVAIL_SAMP_FREQ("100200");

static IIO_CONST_ATTR(name, "adis16220");

static struct attribute *adis16220_event_attributes[] = {
	NULL
};

static struct attribute_group adis16220_event_attribute_group = {
	.attrs = adis16220_event_attributes,
};

static struct attribute *adis16220_attributes[] = {
	&iio_dev_attr_volt_supply.dev_attr.attr,
	&iio_const_attr_volt_supply_scale.dev_attr.attr,
	&iio_dev_attr_accel.dev_attr.attr,
	&iio_dev_attr_accel_offset.dev_attr.attr,
	&iio_dev_attr_accel_peak.dev_attr.attr,
	&iio_dev_attr_temp.dev_attr.attr,
	&iio_dev_attr_adc_1.dev_attr.attr,
	&iio_dev_attr_adc_2.dev_attr.attr,
	&iio_const_attr_temp_offset.dev_attr.attr,
	&iio_const_attr_temp_scale.dev_attr.attr,
	&iio_const_attr_available_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_reset.dev_attr.attr,
	&iio_const_attr_name.dev_attr.attr,
	NULL
};

static const struct attribute_group adis16220_attribute_group = {
	.attrs = adis16220_attributes,
};

static int __devinit adis16220_probe(struct spi_device *spi)
{
	int ret, regdone = 0;
	struct adis16220_state *st = kzalloc(sizeof *st, GFP_KERNEL);
	if (!st) {
		ret =  -ENOMEM;
		goto error_ret;
	}
	/* this is only used for removal purposes */
	spi_set_drvdata(spi, st);

	/* Allocate the comms buffers */
	st->rx = kzalloc(sizeof(*st->rx)*ADIS16220_MAX_RX, GFP_KERNEL);
	if (st->rx == NULL) {
		ret = -ENOMEM;
		goto error_free_st;
	}
	st->tx = kzalloc(sizeof(*st->tx)*ADIS16220_MAX_TX, GFP_KERNEL);
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
	st->indio_dev->num_interrupt_lines = 1;
	st->indio_dev->event_attrs = &adis16220_event_attribute_group;
	st->indio_dev->attrs = &adis16220_attribute_group;
	st->indio_dev->dev_data = (void *)(st);
	st->indio_dev->driver_module = THIS_MODULE;
	st->indio_dev->modes = INDIO_DIRECT_MODE;

	ret = adis16220_configure_ring(st->indio_dev);
	if (ret)
		goto error_free_dev;

	ret = iio_device_register(st->indio_dev);
	if (ret)
		goto error_unreg_ring_funcs;
	regdone = 1;

	ret = adis16220_initialize_ring(st->indio_dev->ring);
	if (ret) {
		printk(KERN_ERR "failed to initialize the ring\n");
		goto error_unreg_ring_funcs;
	}

	if (spi->irq && gpio_is_valid(irq_to_gpio(spi->irq)) > 0) {
#if 0 /* fixme: here we should support */
		iio_init_work_cont(&st->work_cont_thresh,
				NULL,
				adis16220_thresh_handler_bh_no_check,
				0,
				0,
				st);
#endif
		ret = iio_register_interrupt_line(spi->irq,
				st->indio_dev,
				0,
				IRQF_TRIGGER_RISING,
				"adis16220");
		if (ret)
			goto error_uninitialize_ring;

		ret = adis16220_probe_trigger(st->indio_dev);
		if (ret)
			goto error_unregister_line;
	}

	/* Get the device into a sane initial state */
	ret = adis16220_initial_setup(st);
	if (ret)
		goto error_remove_trigger;
	return 0;

error_remove_trigger:
	if (st->indio_dev->modes & INDIO_RING_TRIGGERED)
		adis16220_remove_trigger(st->indio_dev);
error_unregister_line:
	if (st->indio_dev->modes & INDIO_RING_TRIGGERED)
		iio_unregister_interrupt_line(st->indio_dev, 0);
error_uninitialize_ring:
	adis16220_uninitialize_ring(st->indio_dev->ring);
error_unreg_ring_funcs:
	adis16220_unconfigure_ring(st->indio_dev);
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
static int adis16220_remove(struct spi_device *spi)
{
	struct adis16220_state *st = spi_get_drvdata(spi);
	struct iio_dev *indio_dev = st->indio_dev;

	flush_scheduled_work();

	adis16220_remove_trigger(indio_dev);
	if (spi->irq && gpio_is_valid(irq_to_gpio(spi->irq)) > 0)
		iio_unregister_interrupt_line(indio_dev, 0);

	adis16220_uninitialize_ring(indio_dev->ring);
	adis16220_unconfigure_ring(indio_dev);
	iio_device_unregister(indio_dev);
	kfree(st->tx);
	kfree(st->rx);
	kfree(st);

	return 0;
}

static struct spi_driver adis16220_driver = {
	.driver = {
		.name = "adis16220",
		.owner = THIS_MODULE,
	},
	.probe = adis16220_probe,
	.remove = __devexit_p(adis16220_remove),
};

static __init int adis16220_init(void)
{
	return spi_register_driver(&adis16220_driver);
}
module_init(adis16220_init);

static __exit void adis16220_exit(void)
{
	spi_unregister_driver(&adis16220_driver);
}
module_exit(adis16220_exit);

MODULE_AUTHOR("Barry Song <21cnbao@gmail.com>");
MODULE_DESCRIPTION("Analog Devices ADIS16220 Programmable Digital Vibration Sensor driver");
MODULE_LICENSE("GPL v2");
