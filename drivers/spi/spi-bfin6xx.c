/*
 * Analog Devices SPI3 controller driver
 *
 * Copyright (c) 2011 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>

#include <asm/bfin6xx_spi.h>
#include <asm/cacheflush.h>
#include <asm/portmux.h>

#define START_STATE	((void *)0)
#define RUNNING_STATE	((void *)1)
#define DONE_STATE	((void *)2)
#define ERROR_STATE	((void *)-1)

struct bfin_spi_master_data;

struct bfin_spi_transfer_ops {
	void (*write) (struct bfin_spi_master_data *);
	void (*read) (struct bfin_spi_master_data *);
	void (*duplex) (struct bfin_spi_master_data *);
};

struct bfin_spi_master_data {
	/* Driver model hookup */
	struct platform_device *pdev;

	/* SPI framework hookup */
	struct spi_master *master;

	/* Regs base of SPI controller */
	struct bfin_spi_regs __iomem *regs;

	/* Pin request list */
	u16 *pin_req;

	/* BFIN hookup */
	struct bfin6xx_spi_master *master_info;

	/* Driver message queue */
	struct workqueue_struct *workqueue;
	struct work_struct pump_messages;
	spinlock_t lock;
	struct list_head queue;
	int busy;
	bool running;

	/* Message Transfer pump */
	struct tasklet_struct pump_transfers;

	/* Current message transfer state info */
	struct spi_message *cur_msg;
	struct spi_transfer *cur_transfer;
	struct bfin_spi_slave_data *cur_chip;
	size_t len_in_bytes;
	void *tx;
	void *tx_end;
	void *rx;
	void *rx_end;

	u8 n_bytes;
	/* store register value for suspend/resume */
	u32 control;
	u32 ssel;

	int cs_change;
	const struct bfin_spi_transfer_ops *ops;
};

struct bfin_spi_slave_data {
	u32 control;
	u32 clock;
	u32 ssel;

	u8 chip_select_num;
	u16 cs_chg_udelay; /* Some devices require > 255usec delay */
	u32 cs_gpio;
	u32 tx_dummy_val; /* tx value for rx only transfer */
	const struct bfin_spi_transfer_ops *ops;
};

static void bfin_spi_enable(struct bfin_spi_master_data *drv_data)
{
	bfin_write_or(&drv_data->regs->control, SPI_CTL_EN);
}

static void bfin_spi_disable(struct bfin_spi_master_data *drv_data)
{
	bfin_write_and(&drv_data->regs->control, ~SPI_CTL_EN);
}

/* Caculate the SPI_CLOCK register value based on input HZ */
static u32 hz_to_spi_clock(u32 speed_hz)
{
	u_long sclk = get_sclk1();
	u32 spi_clock = sclk / speed_hz;

	if (spi_clock)
		spi_clock--;
	return spi_clock;
}

static int bfin_spi_flush(struct bfin_spi_master_data *drv_data)
{
	unsigned long limit = loops_per_jiffy << 1;

	/* wait for stop and clear stat */
	while (!(bfin_read(&drv_data->regs->status) & SPI_STAT_SPIF) && --limit)
		cpu_relax();

	bfin_write(&drv_data->regs->status, 0xFFFFFFFF);

	return limit;
}

/* Chip select operation functions for cs_change flag */
static void bfin_spi_cs_active(struct bfin_spi_master_data *drv_data, struct bfin_spi_slave_data *chip)
{
	if (likely(chip->chip_select_num < MAX_CTRL_CS))
		bfin_write_and(&drv_data->regs->ssel, ~chip->ssel);
	else
		gpio_set_value(chip->cs_gpio, 0);
}

static void bfin_spi_cs_deactive(struct bfin_spi_master_data *drv_data,
                                 struct bfin_spi_slave_data *chip)
{
	if (likely(chip->chip_select_num < MAX_CTRL_CS))
		bfin_write_or(&drv_data->regs->ssel, chip->ssel);
	else
		gpio_set_value(chip->cs_gpio, 1);

	/* Move delay here for consistency */
	if (chip->cs_chg_udelay)
		udelay(chip->cs_chg_udelay);
}

/* enable or disable the pin muxed by GPIO and SPI CS to work as SPI CS */
static inline void bfin_spi_cs_enable(struct bfin_spi_master_data *drv_data,
                                      struct bfin_spi_slave_data *chip)
{
	if (chip->chip_select_num < MAX_CTRL_CS)
		bfin_write_or(&drv_data->regs->ssel, chip->ssel >> 8);
}

static inline void bfin_spi_cs_disable(struct bfin_spi_master_data *drv_data,
                                       struct bfin_spi_slave_data *chip)
{
	if (chip->chip_select_num < MAX_CTRL_CS)
		bfin_write_and(&drv_data->regs->ssel, ~(chip->ssel >> 8));
}

/* stop controller and re-config current chip*/
static void bfin_spi_restore_state(struct bfin_spi_master_data *drv_data)
{
	struct bfin_spi_slave_data *chip = drv_data->cur_chip;

	/* Clear status and disable clock */
	bfin_write(&drv_data->regs->status, 0xFFFFFFFF);
	bfin_write(&drv_data->regs->rx_control, 0x0);
	bfin_write(&drv_data->regs->tx_control, 0x0);
	bfin_spi_disable(drv_data);
	dev_dbg(&drv_data->pdev->dev, "restoring spi control state\n");

	SSYNC();

	/* Load the registers */
	bfin_write(&drv_data->regs->control, chip->control);
	bfin_write(&drv_data->regs->clock, chip->clock);

	bfin_spi_enable(drv_data);
	/* we always choose tx transfer initiate */
	bfin_write(&drv_data->regs->rx_control, SPI_RXCTL_REN);
	bfin_write(&drv_data->regs->tx_control,
			SPI_TXCTL_TEN | SPI_TXCTL_TTI);
	bfin_spi_cs_active(drv_data, chip);
}

/* discard invalid rx data and empty rfifo */
static inline void dummy_read(struct bfin_spi_master_data *drv_data)
{
	while (!(bfin_read(&drv_data->regs->status) & SPI_STAT_RFE))
		bfin_read(&drv_data->regs->rfifo);
}

static void bfin_spi_u8_write(struct bfin_spi_master_data *drv_data)
{
	dummy_read(drv_data);
	while (drv_data->tx < drv_data->tx_end) {
		bfin_write(&drv_data->regs->tfifo, (*(u8 *)(drv_data->tx++)));
		while (bfin_read(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		bfin_read(&drv_data->regs->rfifo);
	}
}

static void bfin_spi_u8_read(struct bfin_spi_master_data *drv_data)
{
	u32 tx_val = drv_data->cur_chip->tx_dummy_val;

	dummy_read(drv_data);
	while (drv_data->rx < drv_data->rx_end) {
		bfin_write(&drv_data->regs->tfifo, tx_val);
		while (bfin_read(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u8 *)(drv_data->rx++) = bfin_read(&drv_data->regs->rfifo);
	}
}

static void bfin_spi_u8_duplex(struct bfin_spi_master_data *drv_data)
{
	dummy_read(drv_data);
	while (drv_data->rx < drv_data->rx_end) {
		bfin_write(&drv_data->regs->tfifo, (*(u8 *)(drv_data->tx++)));
		while (bfin_read(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u8 *)(drv_data->rx++) = bfin_read(&drv_data->regs->rfifo);
	}
}

static const struct bfin_spi_transfer_ops bfin_bfin_spi_transfer_ops_u8 = {
	.write  = bfin_spi_u8_write,
	.read   = bfin_spi_u8_read,
	.duplex = bfin_spi_u8_duplex,
};

static void bfin_spi_u16_write(struct bfin_spi_master_data *drv_data)
{
	dummy_read(drv_data);
	while (drv_data->tx < drv_data->tx_end) {
		bfin_write(&drv_data->regs->tfifo, (*(u16 *)drv_data->tx));
		drv_data->tx += 2;
		while (bfin_read(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		bfin_read(&drv_data->regs->rfifo);
	}
}

static void bfin_spi_u16_read(struct bfin_spi_master_data *drv_data)
{
	u32 tx_val = drv_data->cur_chip->tx_dummy_val;

	dummy_read(drv_data);
	while (drv_data->rx < drv_data->rx_end) {
		bfin_write(&drv_data->regs->tfifo, tx_val);
		while (bfin_read(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u16 *)drv_data->rx = bfin_read(&drv_data->regs->rfifo);
		drv_data->rx += 2;
	}
}

static void bfin_spi_u16_duplex(struct bfin_spi_master_data *drv_data)
{
	dummy_read(drv_data);
	while (drv_data->rx < drv_data->rx_end) {
		bfin_write(&drv_data->regs->tfifo, (*(u16 *)drv_data->tx));
		drv_data->tx += 2;
		while (bfin_read(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u16 *)drv_data->rx = bfin_read(&drv_data->regs->rfifo);
		drv_data->rx += 2;
	}
}

static const struct bfin_spi_transfer_ops bfin_bfin_spi_transfer_ops_u16 = {
	.write  = bfin_spi_u16_write,
	.read   = bfin_spi_u16_read,
	.duplex = bfin_spi_u16_duplex,
};

static void bfin_spi_u32_write(struct bfin_spi_master_data *drv_data)
{
	dummy_read(drv_data);
	while (drv_data->tx < drv_data->tx_end) {
		bfin_write(&drv_data->regs->tfifo, (*(u32 *)drv_data->tx));
		drv_data->tx += 4;
		while (bfin_read(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		bfin_read(&drv_data->regs->rfifo);
	}
}

static void bfin_spi_u32_read(struct bfin_spi_master_data *drv_data)
{
	u32 tx_val = drv_data->cur_chip->tx_dummy_val;

	dummy_read(drv_data);
	while (drv_data->rx < drv_data->rx_end) {
		bfin_write(&drv_data->regs->tfifo, tx_val);
		while (bfin_read(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u32 *)drv_data->rx = bfin_read(&drv_data->regs->rfifo);
		drv_data->rx += 4;
	}
}

static void bfin_spi_u32_duplex(struct bfin_spi_master_data *drv_data)
{
	dummy_read(drv_data);
	while (drv_data->rx < drv_data->rx_end) {
		bfin_write(&drv_data->regs->tfifo, (*(u32 *)drv_data->tx));
		drv_data->tx += 4;
		while (bfin_read(&drv_data->regs->status) & SPI_STAT_RFE)
			cpu_relax();
		*(u32 *)drv_data->rx = bfin_read(&drv_data->regs->rfifo);
		drv_data->rx += 4;
	}
}

static const struct bfin_spi_transfer_ops bfin_bfin_spi_transfer_ops_u32 = {
	.write  = bfin_spi_u32_write,
	.read   = bfin_spi_u32_read,
	.duplex = bfin_spi_u32_duplex,
};


/* test if there is more transfer to be done */
static void *bfin_spi_next_transfer(struct bfin_spi_master_data *drv_data)
{
	struct spi_message *msg = drv_data->cur_msg;
	struct spi_transfer *trans = drv_data->cur_transfer;

	/* Move to next transfer */
	if (trans->transfer_list.next != &msg->transfers) {
		drv_data->cur_transfer =
		    list_entry(trans->transfer_list.next,
			       struct spi_transfer, transfer_list);
		return RUNNING_STATE;
	} else
		return DONE_STATE;
}

static void bfin_spi_giveback(struct bfin_spi_master_data *drv_data)
{
	struct bfin_spi_slave_data *chip = drv_data->cur_chip;
	unsigned long flags;
	struct spi_message *msg;

	spin_lock_irqsave(&drv_data->lock, flags);
	msg = drv_data->cur_msg;
	drv_data->cur_msg = NULL;
	drv_data->cur_transfer = NULL;
	drv_data->cur_chip = NULL;
	queue_work(drv_data->workqueue, &drv_data->pump_messages);
	spin_unlock_irqrestore(&drv_data->lock, flags);

	msg->state = NULL;

	if (!drv_data->cs_change)
		bfin_spi_cs_deactive(drv_data, chip);

	if (msg->complete)
		msg->complete(msg->context);
}

static void bfin_spi_pump_transfers(unsigned long data)
{
	struct bfin_spi_master_data *drv_data = (struct bfin_spi_master_data *)data;
	struct spi_message *message = NULL;
	struct spi_transfer *transfer = NULL;
	struct spi_transfer *previous = NULL;
	struct bfin_spi_slave_data *chip = NULL;
	unsigned int bits_per_word;
	u32 cr, cr_width;
	u32 tranf_success = 1;
	u8 full_duplex = 0;

	/* Get current state information */
	message = drv_data->cur_msg;
	transfer = drv_data->cur_transfer;
	chip = drv_data->cur_chip;

	/*
	 * if msg is error or done, report it back using complete() callback
	 */

	 /* Handle for abort */
	if (message->state == ERROR_STATE) {
		dev_dbg(&drv_data->pdev->dev, "transfer: we've hit an error\n");
		message->status = -EIO;
		bfin_spi_giveback(drv_data);
		return;
	}

	/* Handle end of message */
	if (message->state == DONE_STATE) {
		dev_dbg(&drv_data->pdev->dev, "transfer: all done!\n");
		message->status = 0;
		bfin_spi_flush(drv_data);
		bfin_spi_giveback(drv_data);
		return;
	}

	/* Delay if requested at end of transfer */
	if (message->state == RUNNING_STATE) {
		dev_dbg(&drv_data->pdev->dev, "transfer: still running ...\n");
		previous = list_entry(transfer->transfer_list.prev,
				      struct spi_transfer, transfer_list);
		if (previous->delay_usecs)
			udelay(previous->delay_usecs);
	}

	/* Flush any existing transfers that may be sitting in the hardware */
	if (bfin_spi_flush(drv_data) == 0) {
		dev_err(&drv_data->pdev->dev, "pump_transfers: flush failed\n");
		message->status = -EIO;
		bfin_spi_giveback(drv_data);
		return;
	}

	if (transfer->len == 0) {
		/* Move to next transfer of this msg */
		message->state = bfin_spi_next_transfer(drv_data);
		/* Schedule next transfer tasklet */
		tasklet_schedule(&drv_data->pump_transfers);
		return;
	}

	if (transfer->tx_buf != NULL) {
		drv_data->tx = (void *)transfer->tx_buf;
		drv_data->tx_end = drv_data->tx + transfer->len;
		dev_dbg(&drv_data->pdev->dev, "tx_buf is %p, tx_end is %p\n",
			transfer->tx_buf, drv_data->tx_end);
	} else {
		drv_data->tx = NULL;
	}

	if (transfer->rx_buf != NULL) {
		full_duplex = transfer->tx_buf != NULL;
		drv_data->rx = transfer->rx_buf;
		drv_data->rx_end = drv_data->rx + transfer->len;
		dev_dbg(&drv_data->pdev->dev, "rx_buf is %p, rx_end is %p\n",
			transfer->rx_buf, drv_data->rx_end);
	} else {
		drv_data->rx = NULL;
	}

	drv_data->len_in_bytes = transfer->len;
	drv_data->cs_change = transfer->cs_change;

	/* Bits per word setup */
	bits_per_word = transfer->bits_per_word ? :
		message->spi->bits_per_word ? : 8;
	drv_data->n_bytes = bits_per_word/8;
	switch (bits_per_word) {
	case 8:
		cr_width = SPI_CTL_SIZE08;
		drv_data->ops = &bfin_bfin_spi_transfer_ops_u8;
		break;
	case 16:
		cr_width = SPI_CTL_SIZE16;
		drv_data->ops = &bfin_bfin_spi_transfer_ops_u16;
		break;
	case 32:
		cr_width = SPI_CTL_SIZE32;
		drv_data->ops = &bfin_bfin_spi_transfer_ops_u32;
		break;
	default:
		dev_err(&drv_data->pdev->dev, "transfer: unsupported bits_per_word\n");
		message->status = -EINVAL;
		bfin_spi_giveback(drv_data);
		return;
	}
	cr = bfin_read(&drv_data->regs->control) & ~SPI_CTL_SIZE;
	cr |= cr_width;
	bfin_write(&drv_data->regs->control, cr);

	message->state = RUNNING_STATE;

	/* Speed setup (surely valid because already checked) */
	if (transfer->speed_hz)
		bfin_write(&drv_data->regs->clock, hz_to_spi_clock(transfer->speed_hz));
	else
		bfin_write(&drv_data->regs->clock, chip->clock);

	bfin_write(&drv_data->regs->status, 0xFFFFFFFF);
	bfin_spi_cs_active(drv_data, chip);

	dev_dbg(&drv_data->pdev->dev,
		"now pumping a transfer: width is %d, len is %d\n",
		bits_per_word, transfer->len);

	if (full_duplex) {
		/* full duplex mode */
		BUG_ON((drv_data->tx_end - drv_data->tx) !=
		       (drv_data->rx_end - drv_data->rx));

		drv_data->ops->duplex(drv_data);

		if (drv_data->tx != drv_data->tx_end)
			tranf_success = 0;
	} else if (drv_data->tx != NULL) {
		/* write only half duplex */
		drv_data->ops->write(drv_data);

		if (drv_data->tx != drv_data->tx_end)
			tranf_success = 0;
	} else if (drv_data->rx != NULL) {
		/* read only half duplex */
		drv_data->ops->read(drv_data);
		if (drv_data->rx != drv_data->rx_end)
			tranf_success = 0;
	}

	if (!tranf_success) {
		dev_dbg(&drv_data->pdev->dev, "IO write error!\n");
		message->state = ERROR_STATE;
	} else {
		/* Update total byte transferred */
		message->actual_length += drv_data->len_in_bytes;
		/* Move to next transfer of this msg */
		message->state = bfin_spi_next_transfer(drv_data);
		if (drv_data->cs_change && message->state != DONE_STATE) {
			bfin_spi_flush(drv_data);
			bfin_spi_cs_deactive(drv_data, chip);
		}
	}

	/* Schedule next transfer tasklet */
	tasklet_schedule(&drv_data->pump_transfers);
}

/* pop a msg from queue and kick off real transfer */
static void bfin_spi_pump_messages(struct work_struct *work)
{
	struct bfin_spi_master_data *drv_data;
	unsigned long flags;

	drv_data = container_of(work, struct bfin_spi_master_data, pump_messages);

	/* Lock queue and check for queue work */
	spin_lock_irqsave(&drv_data->lock, flags);
	if (list_empty(&drv_data->queue) || !drv_data->running) {
		/* pumper kicked off but no work to do */
		drv_data->busy = 0;
		spin_unlock_irqrestore(&drv_data->lock, flags);
		return;
	}

	/* Make sure we are not already running a message */
	if (drv_data->cur_msg) {
		spin_unlock_irqrestore(&drv_data->lock, flags);
		return;
	}

	/* Extract head of queue */
	drv_data->cur_msg = list_entry(drv_data->queue.next,
				       struct spi_message, queue);

	/* Setup the SSP using the per chip configuration */
	drv_data->cur_chip = spi_get_ctldata(drv_data->cur_msg->spi);
	bfin_spi_restore_state(drv_data);

	list_del_init(&drv_data->cur_msg->queue);

	/* Initial message state */
	drv_data->cur_msg->state = START_STATE;
	drv_data->cur_transfer = list_entry(drv_data->cur_msg->transfers.next,
					    struct spi_transfer, transfer_list);

	dev_dbg(&drv_data->pdev->dev, "got a message to pump\n");
	dev_dbg(&drv_data->pdev->dev,
		"the first transfer len is %d\n",
		drv_data->cur_transfer->len);

	/* Mark as busy and launch transfers */
	tasklet_schedule(&drv_data->pump_transfers);

	drv_data->busy = 1;
	spin_unlock_irqrestore(&drv_data->lock, flags);
}

/*
 * got a msg to transfer, queue it in drv_data->queue.
 * And kick off message pumper
 */
static int bfin_spi_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct bfin_spi_master_data *drv_data = spi_master_get_devdata(spi->master);
	unsigned long flags;

	spin_lock_irqsave(&drv_data->lock, flags);

	if (!drv_data->running) {
		spin_unlock_irqrestore(&drv_data->lock, flags);
		return -ESHUTDOWN;
	}

	msg->actual_length = 0;
	msg->status = -EINPROGRESS;
	msg->state = START_STATE;

	dev_dbg(&spi->dev, "adding an msg in transfer() \n");
	list_add_tail(&msg->queue, &drv_data->queue);

	if (drv_data->running && !drv_data->busy)
		queue_work(drv_data->workqueue, &drv_data->pump_messages);

	spin_unlock_irqrestore(&drv_data->lock, flags);

	return 0;
}

#define MAX_SPI_SSEL	7

static const u16 ssel[][MAX_SPI_SSEL] = {
	{P_SPI0_SSEL1, P_SPI0_SSEL2, P_SPI0_SSEL3,
	P_SPI0_SSEL4, P_SPI0_SSEL5,
	P_SPI0_SSEL6, P_SPI0_SSEL7},

	{P_SPI1_SSEL1, P_SPI1_SSEL2, P_SPI1_SSEL3,
	P_SPI1_SSEL4, P_SPI1_SSEL5,
	P_SPI1_SSEL6, P_SPI1_SSEL7},

	{P_SPI2_SSEL1, P_SPI2_SSEL2, P_SPI2_SSEL3,
	P_SPI2_SSEL4, P_SPI2_SSEL5,
	P_SPI2_SSEL6, P_SPI2_SSEL7},
};

/* setup for devices (may be called multiple times -- not just first setup) */
static int bfin_spi_setup(struct spi_device *spi)
{
	struct bfin6xx_spi_chip *chip_info;
	struct bfin_spi_slave_data *chip = NULL;
	struct bfin_spi_master_data *drv_data = spi_master_get_devdata(spi->master);
	u32 bfin_ctl_reg;
	int ret = -EINVAL;

	/* Only alloc (or use chip_info) on first setup */
	chip_info = NULL;
	chip = spi_get_ctldata(spi);
	if (chip == NULL) {
		chip = kzalloc(sizeof(*chip), GFP_KERNEL);
		if (!chip) {
			dev_err(&spi->dev, "cannot allocate chip data\n");
			ret = -ENOMEM;
			goto error;
		}

		chip_info = spi->controller_data;
	}

	/* Let people set non-standard bits directly */
	bfin_ctl_reg = SPI_CTL_ODM | SPI_CTL_PSSE;

	/* chip_info isn't always needed */
	if (chip_info) {
		if (chip_info->control & ~bfin_ctl_reg) {
			dev_err(&spi->dev, "do not set bits in control reg "
				"that the SPI framework manages\n");
			goto error;
		}
		chip->control = chip_info->control;
		chip->cs_chg_udelay = chip_info->cs_chg_udelay;
		chip->tx_dummy_val = chip_info->tx_dummy_val;
	} else {
		/* force a default base state */
		chip->control &= bfin_ctl_reg;
	}

	if (spi->bits_per_word % 8) {
		dev_err(&spi->dev, "%d bits_per_word is not supported\n",
				spi->bits_per_word);
		goto error;
	}

	/* translate common spi framework into our register */
	if (spi->mode & ~(SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST)) {
		dev_err(&spi->dev, "unsupported spi modes detected\n");
		goto error;
	}
	if (spi->mode & SPI_CPOL)
		chip->control |= SPI_CTL_CPOL;
	if (spi->mode & SPI_CPHA)
		chip->control |= SPI_CTL_CPHA;
	if (spi->mode & SPI_LSB_FIRST)
		chip->control |= SPI_CTL_LSBF;
	chip->control |= SPI_CTL_MSTR;
	chip->control &= ~SPI_CTL_ASSEL; /* we choose software to controll cs */

	chip->clock = hz_to_spi_clock(spi->max_speed_hz);
	chip->chip_select_num = spi->chip_select;
	if (chip->chip_select_num < MAX_CTRL_CS)
		chip->ssel = (1 << spi->chip_select) << 8;
	else
		chip->cs_gpio = chip->chip_select_num - MAX_CTRL_CS;

	if (chip->chip_select_num >= MAX_CTRL_CS) {
		/* Only request on first setup */
		if (spi_get_ctldata(spi) == NULL) {
			ret = gpio_request(chip->cs_gpio, spi->modalias);
			if (ret) {
				dev_err(&spi->dev, "gpio_request() error\n");
				goto pin_error;
			}
			gpio_direction_output(chip->cs_gpio, 1);
		}
	}

	if (chip->chip_select_num < MAX_CTRL_CS) {
		ret = peripheral_request(ssel[spi->master->bus_num]
		                         [chip->chip_select_num-1], spi->modalias);
		if (ret) {
			dev_err(&spi->dev, "peripheral_request() error\n");
			goto pin_error;
		}
	}

	dev_dbg(&spi->dev, "setup spi chip %s, width is %d\n",
			spi->modalias, spi->bits_per_word);
	dev_dbg(&spi->dev, "control is 0x%x, ssel is 0x%x\n",
			chip->control, chip->ssel);
	dev_dbg(&spi->dev, "chip select number is %d\n", chip->chip_select_num);

	spi_set_ctldata(spi, chip);
		
	bfin_spi_cs_enable(drv_data, chip);
	bfin_spi_cs_deactive(drv_data, chip);

	return 0;

 pin_error:
	if (chip->chip_select_num >= MAX_CTRL_CS)
		gpio_free(chip->cs_gpio);
	else
		peripheral_free(ssel[spi->master->bus_num]
			[chip->chip_select_num - 1]);
 error:
	if (chip) {
		kfree(chip);
		/* prevent free 'chip' twice */
		spi_set_ctldata(spi, NULL);
	}

	return ret;
}

/*
 * callback for spi framework.
 * clean driver specific data
 */
static void bfin_spi_cleanup(struct spi_device *spi)
{
	struct bfin_spi_slave_data *chip = spi_get_ctldata(spi);
	struct bfin_spi_master_data *drv_data = spi_master_get_devdata(spi->master);

	if (!chip)
		return;

	if (chip->chip_select_num < MAX_CTRL_CS) {
		peripheral_free(ssel[spi->master->bus_num]
					[chip->chip_select_num-1]);
		bfin_spi_cs_disable(drv_data, chip);
	} else
		gpio_free(chip->cs_gpio);

	kfree(chip);
	/* prevent free 'chip' twice */
	spi_set_ctldata(spi, NULL);
}

static int bfin_spi_init_queue(struct bfin_spi_master_data *drv_data)
{
	INIT_LIST_HEAD(&drv_data->queue);
	spin_lock_init(&drv_data->lock);

	drv_data->running = false;
	drv_data->busy = 0;

	/* init transfer tasklet */
	tasklet_init(&drv_data->pump_transfers,
		     bfin_spi_pump_transfers, (unsigned long)drv_data);

	/* init messages workqueue */
	INIT_WORK(&drv_data->pump_messages, bfin_spi_pump_messages);
	drv_data->workqueue = create_singlethread_workqueue(
				dev_name(drv_data->master->dev.parent));
	if (drv_data->workqueue == NULL)
		return -EBUSY;

	return 0;
}

static int bfin_spi_start_queue(struct bfin_spi_master_data *drv_data)
{
	unsigned long flags;

	spin_lock_irqsave(&drv_data->lock, flags);

	if (drv_data->running || drv_data->busy) {
		spin_unlock_irqrestore(&drv_data->lock, flags);
		return -EBUSY;
	}

	drv_data->running = true;
	drv_data->cur_msg = NULL;
	drv_data->cur_transfer = NULL;
	drv_data->cur_chip = NULL;
	spin_unlock_irqrestore(&drv_data->lock, flags);

	queue_work(drv_data->workqueue, &drv_data->pump_messages);

	return 0;
}

static int bfin_spi_stop_queue(struct bfin_spi_master_data *drv_data)
{
	unsigned long flags;
	unsigned limit = 500;
	int status = 0;

	spin_lock_irqsave(&drv_data->lock, flags);

	/*
	 * This is a bit lame, but is optimized for the common execution path.
	 * A wait_queue on the drv_data->busy could be used, but then the common
	 * execution path (pump_messages) would be required to call wake_up or
	 * friends on every SPI message. Do this instead
	 */
	drv_data->running = false;
	while ((!list_empty(&drv_data->queue) || drv_data->busy) && limit--) {
		spin_unlock_irqrestore(&drv_data->lock, flags);
		msleep(10);
		spin_lock_irqsave(&drv_data->lock, flags);
	}

	if (!list_empty(&drv_data->queue) || drv_data->busy)
		status = -EBUSY;

	spin_unlock_irqrestore(&drv_data->lock, flags);

	return status;
}

static int bfin_spi_destroy_queue(struct bfin_spi_master_data *drv_data)
{
	int status;

	status = bfin_spi_stop_queue(drv_data);
	if (status != 0)
		return status;

	destroy_workqueue(drv_data->workqueue);

	return 0;
}

static int __devinit bfin_spi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bfin6xx_spi_master *platform_info;
	struct spi_master *master;
	struct bfin_spi_master_data *drv_data;
	struct resource *res;
	int status = 0;

	platform_info = dev->platform_data;

	/* Allocate master with space for drv_data */
	master = spi_alloc_master(dev, sizeof(*drv_data));
	if (!master) {
		dev_err(&pdev->dev, "can not alloc spi_master\n");
		return -ENOMEM;
	}

	drv_data = spi_master_get_devdata(master);
	drv_data->master = master;
	drv_data->master_info = platform_info;
	drv_data->pdev = pdev;
	drv_data->pin_req = platform_info->pin_req;

	/* the spi->mode bits supported by this driver: */
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST;

	master->bus_num = pdev->id;
	master->num_chipselect = platform_info->num_chipselect;
	master->cleanup = bfin_spi_cleanup;
	master->setup = bfin_spi_setup;
	master->transfer = bfin_spi_transfer;

	/* Find and map our resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(dev, "Cannot get IORESOURCE_MEM\n");
		status = -ENOENT;
		goto err_put_master;
	}

	drv_data->regs = ioremap(res->start, resource_size(res));
	if (drv_data->regs == NULL) {
		dev_err(dev, "Cannot map IO\n");
		status = -ENXIO;
		goto err_put_master;
	}

	/* Initial and start queue */
	status = bfin_spi_init_queue(drv_data);
	if (status != 0) {
		dev_err(dev, "problem initializing queue\n");
		goto err_destroy_queue;
	}

	status = bfin_spi_start_queue(drv_data);
	if (status != 0) {
		dev_err(dev, "problem starting queue\n");
		goto err_destroy_queue;
	}

	status = peripheral_request_list(drv_data->pin_req, "bfin-spi");
	if (status != 0) {
		dev_err(&pdev->dev, ": Requesting Peripherals failed\n");
		goto err_destroy_queue;
	}

	bfin_write(&drv_data->regs->control, SPI_CTL_MSTR | SPI_CTL_CPHA);
	bfin_write(&drv_data->regs->ssel, 0x0000FE00);
	bfin_write(&drv_data->regs->delay, 0x0);

	/* Register with the SPI framework */
	platform_set_drvdata(pdev, drv_data);
	status = spi_register_master(master);
	if (status != 0) {
		dev_err(dev, "problem registering spi master\n");
		goto err_free_peripheral;
	}

	dev_info(dev, "bfin-spi probe success\n");
	return status;

err_free_peripheral:
	peripheral_free_list(drv_data->pin_req);
err_destroy_queue:
	bfin_spi_destroy_queue(drv_data);
	iounmap(drv_data->regs);
err_put_master:
	spi_master_put(master);

	return status;
}

/* stop hardware and remove the driver */
static int __devexit bfin_spi_remove(struct platform_device *pdev)
{
	struct bfin_spi_master_data *drv_data = platform_get_drvdata(pdev);
	int status = 0;

	if (!drv_data)
		return 0;

	/* Remove the queue */
	status = bfin_spi_destroy_queue(drv_data);
	if (status != 0)
		return status;

	/* Disable the SSP at the peripheral and SOC level */
	bfin_spi_disable(drv_data);

	peripheral_free_list(drv_data->pin_req);
	iounmap(drv_data->regs);

	/* Disconnect from the SPI framework */
	spi_unregister_master(drv_data->master);
	
	/* Prevent double remove */
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int bfin_spi_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct bfin_spi_master_data *drv_data = platform_get_drvdata(pdev);
	int status = 0;

	status = bfin_spi_stop_queue(drv_data);
	if (status != 0)
		return status;

	drv_data->control = bfin_read(&drv_data->regs->control);
	drv_data->ssel = bfin_read(&drv_data->regs->ssel);

	bfin_write(&drv_data->regs->control, SPI_CTL_MSTR | SPI_CTL_CPHA);
	bfin_write(&drv_data->regs->ssel, 0x0000FE00);

	return 0;
}

static int bfin_spi_resume(struct platform_device *pdev)
{
	struct bfin_spi_master_data *drv_data = platform_get_drvdata(pdev);
	int status = 0;

	bfin_write(&drv_data->regs->control, drv_data->control);
	bfin_write(&drv_data->regs->ssel, drv_data->ssel);

	/* Start the queue running */
	status = bfin_spi_start_queue(drv_data);
	if (status != 0) {
		dev_err(&pdev->dev, "problem starting queue (%d)\n", status);
		return status;
	}

	return 0;
}
#else
#define bfin_spi_suspend NULL
#define bfin_spi_resume NULL
#endif /* CONFIG_PM */

MODULE_ALIAS("platform:bfin-spi");
static struct platform_driver bfin_spi_driver = {
	.driver	= {
		.name	= "bfin-spi",
		.owner	= THIS_MODULE,
	},
	.suspend	= bfin_spi_suspend,
	.resume		= bfin_spi_resume,
	.remove		= __devexit_p(bfin_spi_remove),
};

static int __init bfin_spi_init(void)
{
	return platform_driver_probe(&bfin_spi_driver, bfin_spi_probe);
}
subsys_initcall(bfin_spi_init);

static void __exit bfin_spi_exit(void)
{
	platform_driver_unregister(&bfin_spi_driver);
}
module_exit(bfin_spi_exit);

MODULE_DESCRIPTION("Analog Devices SPI3 controller driver");
MODULE_AUTHOR("Scott Jiang <Scott.Jiang.Linux@gmail.com>");
MODULE_LICENSE("GPL v2");
