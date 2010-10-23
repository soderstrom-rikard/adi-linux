#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/list.h>

#include "../iio.h"
#include "../sysfs.h"
#include "../ring_sw.h"
#include "../accel/accel.h"
#include "../trigger.h"
#include "ade7758.h"

/**
 * combine_8_to_32() utility function to munge to u8s into u32
 **/
static inline u32 combine_8_to_32(u8 lower, u8 mid, u8 upper)
{
	u32 _lower = lower;
	u32 _mid = mid;
	u32 _upper = upper;

	return _lower | (_mid << 8) | (_upper << 16);
}

static IIO_SCAN_EL_C(wform, ADE7758_SCAN_WFORM, IIO_SIGNED(24),
		     ADE7758_WFORM, NULL);

static IIO_SCAN_EL_TIMESTAMP(1);

static struct attribute *ade7758_scan_el_attrs[] = {
	&iio_scan_el_wform.dev_attr.attr,
	&iio_scan_el_timestamp.dev_attr.attr,
	NULL,
};

static struct attribute_group ade7758_scan_el_group = {
	.attrs = ade7758_scan_el_attrs,
	.name = "scan_elements",
};

/**
 * ade7758_poll_func_th() top half interrupt handler called by trigger
 * @private_data:	iio_dev
 **/
static void ade7758_poll_func_th(struct iio_dev *indio_dev, s64 time)
{
	struct ade7758_state *st = iio_dev_get_devdata(indio_dev);
	st->last_timestamp = time;
	schedule_work(&st->work_trigger_to_ring);
	/* Indicate that this interrupt is being handled */

	/* Technically this is trigger related, but without this
	 * handler running there is currently no way for the interrupt
	 * to clear.
	 */
}

/**
 * ade7758_spi_read_burst() - read all data registers
 * @dev: device associated with child of actual device (iio_dev or iio_trig)
 * @rx: somewhere to pass back the value read (min size is 24 bytes)
 **/
static int ade7758_spi_read_burst(struct device *dev, u8 *rx)
{
	struct spi_message msg;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ade7758_state *st = iio_dev_get_devdata(indio_dev);
	int ret;

	struct spi_transfer xfers[] = {
		{
			.tx_buf = st->tx,
			.rx_buf = rx,
			.bits_per_word = 8,
			.len = 4,
		}, {
			.tx_buf = st->tx + 4,
			.rx_buf = rx,
			.bits_per_word = 8,
			.len = 4,
		},
	};

	mutex_lock(&st->buf_lock);
	st->tx[0] = ADE7758_READ_REG(ADE7758_RSTATUS);
	st->tx[1] = 0;
	st->tx[2] = 0;
	st->tx[3] = 0;
	st->tx[4] = ADE7758_READ_REG(ADE7758_WFORM);
	st->tx[5] = 0;
	st->tx[6] = 0;
	st->tx[7] = 0;

	spi_message_init(&msg);
	spi_message_add_tail(&xfers[0], &msg);
	spi_message_add_tail(&xfers[1], &msg);
	ret = spi_sync(st->us, &msg);
	if (ret)
		dev_err(&st->us->dev, "problem when reading WFORM value\n");

	mutex_unlock(&st->buf_lock);

	return ret;
}

/* Whilst this makes a lot of calls to iio_sw_ring functions - it is to device
 * specific to be rolled into the core.
 */
static void ade7758_trigger_bh_to_ring(struct work_struct *work_s)
{
	struct ade7758_state *st
		= container_of(work_s, struct ade7758_state,
			       work_trigger_to_ring);

	int i = 0;
	s32 *data;
	size_t datasize = st->indio_dev
		->ring->access.get_bpd(st->indio_dev->ring);

	data = kmalloc(datasize , GFP_KERNEL);
	if (data == NULL) {
		dev_err(&st->us->dev, "memory alloc failed in ring bh");
		return;
	}

	if (st->indio_dev->scan_count)
		if (ade7758_spi_read_burst(&st->indio_dev->dev, st->rx) >= 0)
			for (; i < st->indio_dev->scan_count; i++) {
				data[i] = combine_8_to_32(st->rx[i*2+2],
						st->rx[i*2+1],
						st->rx[i*2]);
			}

	/* Guaranteed to be aligned with 8 byte boundary */
	if (st->indio_dev->scan_timestamp)
		*((s64 *)
		(((u32)data + 4 * st->indio_dev->scan_count + 4) & ~0x7)) =
			st->last_timestamp;

	st->indio_dev->ring->access.store_to(st->indio_dev->ring,
			(u8 *)data,
			st->last_timestamp);

	iio_trigger_notify_done(st->indio_dev->trig);
	kfree(data);

	return;
}

/* in these circumstances is it better to go with unaligned packing and
 * deal with the cost?*/
static int ade7758_data_rdy_ring_preenable(struct iio_dev *indio_dev)
{
	size_t size;
	dev_dbg(&indio_dev->dev, "%s\n", __func__);
	/* Check if there are any scan elements enabled, if not fail*/
	if (!(indio_dev->scan_count || indio_dev->scan_timestamp))
		return -EINVAL;

	if (indio_dev->ring->access.set_bpd) {
		if (indio_dev->scan_timestamp)
			if (indio_dev->scan_count) /* Timestamp and data */
				size = 2 * sizeof(s64);
			else /* Timestamp only  */
				size = sizeof(s64);
		else /* Data only */
			size = indio_dev->scan_count*sizeof(s32);
		indio_dev->ring->access.set_bpd(indio_dev->ring, size);
	}

	return 0;
}

static int ade7758_data_rdy_ring_postenable(struct iio_dev *indio_dev)
{
	return indio_dev->trig
		? iio_trigger_attach_poll_func(indio_dev->trig,
					       indio_dev->pollfunc)
		: 0;
}

static int ade7758_data_rdy_ring_predisable(struct iio_dev *indio_dev)
{
	return indio_dev->trig
		? iio_trigger_dettach_poll_func(indio_dev->trig,
						indio_dev->pollfunc)
		: 0;
}

void ade7758_unconfigure_ring(struct iio_dev *indio_dev)
{
	kfree(indio_dev->pollfunc);
	iio_sw_rb_free(indio_dev->ring);
}

int ade7758_configure_ring(struct iio_dev *indio_dev)
{
	int ret = 0;
	struct ade7758_state *st = indio_dev->dev_data;
	struct iio_ring_buffer *ring;
	INIT_WORK(&st->work_trigger_to_ring, ade7758_trigger_bh_to_ring);
	/* Set default scan mode */

	iio_scan_mask_set(indio_dev, iio_scan_el_wform.number);
	indio_dev->scan_timestamp = true;

	indio_dev->scan_el_attrs = &ade7758_scan_el_group;

	ring = iio_sw_rb_allocate(indio_dev);
	if (!ring) {
		ret = -ENOMEM;
		return ret;
	}
	indio_dev->ring = ring;
	/* Effectively select the ring buffer implementation */
	iio_ring_sw_register_funcs(&ring->access);
	ring->preenable = &ade7758_data_rdy_ring_preenable;
	ring->postenable = &ade7758_data_rdy_ring_postenable;
	ring->predisable = &ade7758_data_rdy_ring_predisable;
	ring->owner = THIS_MODULE;

	indio_dev->pollfunc = kzalloc(sizeof(*indio_dev->pollfunc), GFP_KERNEL);
	if (indio_dev->pollfunc == NULL) {
		ret = -ENOMEM;
		goto error_iio_sw_rb_free;;
	}
	indio_dev->pollfunc->poll_func_main = &ade7758_poll_func_th;
	indio_dev->pollfunc->private_data = indio_dev;
	indio_dev->modes |= INDIO_RING_TRIGGERED;
	return 0;

error_iio_sw_rb_free:
	iio_sw_rb_free(indio_dev->ring);
	return ret;
}

int ade7758_initialize_ring(struct iio_ring_buffer *ring)
{
	return iio_ring_buffer_register(ring, 0);
}

void ade7758_uninitialize_ring(struct iio_ring_buffer *ring)
{
	iio_ring_buffer_unregister(ring);
}
