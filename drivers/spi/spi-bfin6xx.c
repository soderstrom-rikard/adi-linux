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
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#include <asm/bfin6xx_spi.h>
#include <asm/cacheflush.h>
#include <asm/dma.h>
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
	bool busy; /* spi master is dealing with messages */
	bool running; /* spi master is ready */

	/* Message Transfer pump */
	struct tasklet_struct pump_transfers;

	/* Current message transfer state info */
	struct spi_message *cur_msg;
	struct spi_transfer *cur_transfer;
	struct bfin_spi_slave_data *cur_chip;
	unsigned transfer_len;
	unsigned cs_change;

	/* transfer buffer */
	void *tx;
	void *tx_end;
	void *rx;
	void *rx_end;

	/* dma info */
	unsigned int tx_dma;
	unsigned int rx_dma;
	dma_addr_t tx_dma_addr;
	dma_addr_t rx_dma_addr;
	unsigned long dummy_buffer; /* used in unidirectional transfer */
	unsigned long tx_dma_size;
	unsigned long rx_dma_size;
	int tx_num;
	int rx_num;

	/* store register value for suspend/resume */
	u32 control;
	u32 ssel;

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
	bool enable_dma;
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
	drv_data->tx_num = drv_data->rx_num = 0;
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
	bool tranf_success = true;
	bool full_duplex = false;

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

	if ((transfer->len == 0) || (transfer->tx_buf == NULL
				&& transfer->rx_buf == NULL)) {
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
		full_duplex = (transfer->tx_buf != NULL) ? true : false;
		drv_data->rx = transfer->rx_buf;
		drv_data->rx_end = drv_data->rx + transfer->len;
		dev_dbg(&drv_data->pdev->dev, "rx_buf is %p, rx_end is %p\n",
			transfer->rx_buf, drv_data->rx_end);
	} else {
		drv_data->rx = NULL;
	}

	drv_data->transfer_len = transfer->len;
	drv_data->cs_change = transfer->cs_change;

	/* Bits per word setup */
	bits_per_word = transfer->bits_per_word ? :
		message->spi->bits_per_word ? : 8;
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

	if (chip->enable_dma) {
		u32 dma_config;
		unsigned long word_count, word_size;
		void *tx_buf, *rx_buf;

		switch (bits_per_word) {
		case 8:
			dma_config = WDSIZE_8 | PSIZE_8;
			word_count = drv_data->transfer_len;
			word_size = 1;
			break;
		case 16:
			dma_config = WDSIZE_16 | PSIZE_16;
			word_count = drv_data->transfer_len / 2;
			word_size = 2;
			break;
		default:
			dma_config = WDSIZE_32 | PSIZE_32;
			word_count = drv_data->transfer_len / 4;
			word_size = 4;
			break;
		}

		if (full_duplex) {
			WARN_ON((drv_data->tx_end - drv_data->tx)
					!= (drv_data->rx_end - drv_data->rx));
			tx_buf = drv_data->tx;
			rx_buf = drv_data->rx;
			drv_data->tx_dma_size = drv_data->rx_dma_size
						= drv_data->transfer_len;
			set_dma_x_modify(drv_data->tx_dma, word_size);
			set_dma_x_modify(drv_data->rx_dma, word_size);
		} else if (drv_data->tx) {
			tx_buf = drv_data->tx;
			rx_buf = &drv_data->dummy_buffer;
			drv_data->tx_dma_size = drv_data->transfer_len;
			drv_data->rx_dma_size = sizeof(drv_data->dummy_buffer);
			set_dma_x_modify(drv_data->tx_dma, word_size);
			set_dma_x_modify(drv_data->rx_dma, 0);
		} else {
			drv_data->dummy_buffer = chip->tx_dummy_val;
			tx_buf = &drv_data->dummy_buffer;
			rx_buf = drv_data->rx;
			drv_data->tx_dma_size = sizeof(drv_data->dummy_buffer);
			drv_data->rx_dma_size = drv_data->transfer_len;
			set_dma_x_modify(drv_data->tx_dma, 0);
			set_dma_x_modify(drv_data->rx_dma, word_size);
		}

		drv_data->tx_dma_addr = dma_map_single(&message->spi->dev,
					(void *)tx_buf,
					drv_data->tx_dma_size,
					DMA_TO_DEVICE);
		if (dma_mapping_error(&message->spi->dev,
					drv_data->tx_dma_addr)) {
			dev_dbg(&drv_data->pdev->dev, "Unable to map TX DMA\n");
			message->state = ERROR_STATE;
			return;
		}

		drv_data->rx_dma_addr = dma_map_single(&message->spi->dev,
					(void *)rx_buf,
					drv_data->rx_dma_size,
					DMA_FROM_DEVICE);
		if (dma_mapping_error(&message->spi->dev,
					drv_data->rx_dma_addr)) {
			dev_dbg(&drv_data->pdev->dev, "Unable to map RX DMA\n");
			message->state = ERROR_STATE;
			dma_unmap_single(&message->spi->dev,
					drv_data->tx_dma_addr,
					drv_data->tx_dma_size,
					DMA_TO_DEVICE);
			return;
		}

		dummy_read(drv_data);
		set_dma_x_count(drv_data->tx_dma, word_count);
		set_dma_x_count(drv_data->rx_dma, word_count);
		set_dma_start_addr(drv_data->tx_dma, drv_data->tx_dma_addr);
		set_dma_start_addr(drv_data->rx_dma, drv_data->rx_dma_addr);
		dma_config |= DMAFLOW_STOP | RESTART | DI_EN;
		set_dma_config(drv_data->tx_dma, dma_config);
		set_dma_config(drv_data->rx_dma, dma_config | WNR);
		enable_dma(drv_data->tx_dma);
		enable_dma(drv_data->rx_dma);
		SSYNC();

		bfin_write(&drv_data->regs->rx_control, SPI_RXCTL_REN | SPI_RXCTL_RDR_NE);
		SSYNC();
		bfin_write(&drv_data->regs->tx_control,
				SPI_TXCTL_TEN | SPI_TXCTL_TTI | SPI_TXCTL_TDR_NF);

		return;
	}

	if (full_duplex) {
		/* full duplex mode */
		WARN_ON((drv_data->tx_end - drv_data->tx)
				!= (drv_data->rx_end - drv_data->rx));

		drv_data->ops->duplex(drv_data);

		if (drv_data->tx != drv_data->tx_end)
			tranf_success = false;
	} else if (drv_data->tx != NULL) {
		/* write only half duplex */
		drv_data->ops->write(drv_data);

		if (drv_data->tx != drv_data->tx_end)
			tranf_success = false;
	} else {
		/* read only half duplex */
		drv_data->ops->read(drv_data);
		if (drv_data->rx != drv_data->rx_end)
			tranf_success = false;
	}

	if (!tranf_success) {
		dev_dbg(&drv_data->pdev->dev, "IO write error!\n");
		message->state = ERROR_STATE;
	} else {
		/* Update total byte transferred */
		message->actual_length += drv_data->transfer_len;
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
		drv_data->busy = false;
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

	drv_data->busy = true;
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
		chip->enable_dma = chip_info->enable_dma;
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

	/* Only request on first setup */
	if (spi_get_ctldata(spi) == NULL) {
		if (chip->chip_select_num < MAX_CTRL_CS) {
			ret = peripheral_request(ssel[spi->master->bus_num]
					[chip->chip_select_num-1], spi->modalias);
			if (ret) {
				dev_err(&spi->dev, "peripheral_request() error\n");
				goto pin_error;
			}
		} else {
			ret = gpio_request(chip->cs_gpio, spi->modalias);
			if (ret) {
				dev_err(&spi->dev, "gpio_request() error\n");
				goto pin_error;
			}
			gpio_direction_output(chip->cs_gpio, 1);
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
	if (chip->chip_select_num < MAX_CTRL_CS)
		peripheral_free(ssel[spi->master->bus_num]
				[chip->chip_select_num - 1]);
	else
		gpio_free(chip->cs_gpio);
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
	drv_data->busy = false;

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

static irqreturn_t bfin_spi_tx_dma_isr(int irq, void *dev_id)
{
	struct bfin_spi_master_data *drv_data = dev_id;
	u32 dma_stat = get_dma_curr_irqstat(drv_data->tx_dma);

	clear_dma_irqstat(drv_data->tx_dma);
	if (dma_stat & DMA_DONE)
		drv_data->tx_num++;
	if (dma_stat & DMA_ERR)
		dev_err(&drv_data->pdev->dev,
				"spi tx dma error: %d\n", dma_stat);
	bfin_write_and(&drv_data->regs->tx_control, ~SPI_TXCTL_TDR_NF);
	return IRQ_HANDLED;
}

static irqreturn_t bfin_spi_rx_dma_isr(int irq, void *dev_id)
{
	struct bfin_spi_master_data *drv_data = dev_id;
	struct bfin_spi_slave_data *chip = drv_data->cur_chip;
	struct spi_message *msg = drv_data->cur_msg;
	u32 dma_stat = get_dma_curr_irqstat(drv_data->rx_dma);

	clear_dma_irqstat(drv_data->rx_dma);
	if (dma_stat & DMA_DONE)
		drv_data->rx_num++;
	if (dma_stat & DMA_ERR) {
		msg->state = ERROR_STATE;
		dev_err(&drv_data->pdev->dev,
				"spi rx dma error: %d\n", dma_stat);
	} else {
		msg->actual_length += drv_data->transfer_len;
		if (drv_data->cs_change)
			bfin_spi_cs_deactive(drv_data, chip);
		msg->state = bfin_spi_next_transfer(drv_data);
	}
	bfin_write(&drv_data->regs->tx_control, 0);
	bfin_write(&drv_data->regs->rx_control, 0);
	if (drv_data->rx_num != drv_data->tx_num)
		dev_dbg(&drv_data->pdev->dev,
				"dma interrupt missing: tx=%d,rx=%d\n",
				drv_data->tx_num, drv_data->rx_num);
	tasklet_schedule(&drv_data->pump_transfers);
	return IRQ_HANDLED;
}

static int bfin_spi_probe(struct platform_device *pdev)
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
		status = -ENODEV;
		goto err_put_master;
	}

	drv_data->regs = ioremap(res->start, resource_size(res));
	if (drv_data->regs == NULL) {
		dev_err(dev, "Cannot map IO\n");
		status = -ENXIO;
		goto err_put_master;
	}

	res = platform_get_resource(pdev, IORESOURCE_DMA, 0);
	if (res == NULL) {
		dev_err(dev, "Cannot get tx dma resource\n");
		status = -ENODEV;
		goto err_iounmap;
	}
	drv_data->tx_dma = res->start;

	res = platform_get_resource(pdev, IORESOURCE_DMA, 1);
	if (res == NULL) {
		dev_err(dev, "Cannot get rx dma resource\n");
		status = -ENODEV;
		goto err_iounmap;
	}
	drv_data->rx_dma = res->start;

	status = request_dma(drv_data->tx_dma, "SPI_TX_DMA");
	if (status) {
		dev_err(dev, "Unable to request SPI TX DMA channel\n");
		goto err_iounmap;
	}
	set_dma_callback(drv_data->tx_dma, bfin_spi_tx_dma_isr, drv_data);

	status = request_dma(drv_data->rx_dma, "SPI_RX_DMA");
	if (status) {
		dev_err(dev, "Unable to request SPI RX DMA channel\n");
		goto err_free_tx_dma;
	}
	set_dma_callback(drv_data->rx_dma, bfin_spi_rx_dma_isr, drv_data);

	/* request CLK, MOSI and MISO */
	status = peripheral_request_list(drv_data->pin_req, "bfin-spi");
	if (status != 0) {
		dev_err(&pdev->dev, ": Requesting Peripherals failed\n");
		goto err_free_rx_dma;
	}

	/* Initial and start queue */
	status = bfin_spi_init_queue(drv_data);
	if (status != 0) {
		dev_err(dev, "problem initializing queue\n");
		goto err_free_peripheral;
	}

	status = bfin_spi_start_queue(drv_data);
	if (status != 0) {
		dev_err(dev, "problem starting queue\n");
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

err_destroy_queue:
	bfin_spi_destroy_queue(drv_data);
err_free_peripheral:
	peripheral_free_list(drv_data->pin_req);
err_free_rx_dma:
	free_dma(drv_data->rx_dma);
err_free_tx_dma:
	free_dma(drv_data->tx_dma);
err_iounmap:
	iounmap(drv_data->regs);
err_put_master:
	spi_master_put(master);

	return status;
}

/* stop hardware and remove the driver */
static int bfin_spi_remove(struct platform_device *pdev)
{
	struct bfin_spi_master_data *drv_data = platform_get_drvdata(pdev);

	if (!drv_data)
		return 0;

	/* Remove the queue */
	bfin_spi_destroy_queue(drv_data);

	/* Disable the SSP at the peripheral and SOC level */
	bfin_spi_disable(drv_data);

	peripheral_free_list(drv_data->pin_req);
	free_dma(drv_data->rx_dma);
	free_dma(drv_data->tx_dma);
	iounmap(drv_data->regs);

	/* Disconnect from the SPI framework */
	spi_unregister_master(drv_data->master);
	spi_master_put(drv_data->master);
	/* Prevent double remove */
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int bfin_spi_suspend(struct device *dev)
{
	struct bfin_spi_master_data *drv_data = dev_get_drvdata(dev);
	int status = 0;

	status = bfin_spi_stop_queue(drv_data);
	if (status != 0)
		return status;

	drv_data->control = bfin_read(&drv_data->regs->control);
	drv_data->ssel = bfin_read(&drv_data->regs->ssel);

	bfin_write(&drv_data->regs->control, SPI_CTL_MSTR | SPI_CTL_CPHA);
	bfin_write(&drv_data->regs->ssel, 0x0000FE00);
	free_dma(drv_data->rx_dma);
	free_dma(drv_data->tx_dma);

	return 0;
}

static int bfin_spi_resume(struct device *dev)
{
	struct bfin_spi_master_data *drv_data = dev_get_drvdata(dev);
	int status = 0;

	status = request_dma(drv_data->tx_dma, "SPI_TX_DMA");
	if (status) {
		dev_err(dev, "Unable to request SPI TX DMA channel\n");
		return status;
	}
	set_dma_callback(drv_data->tx_dma, bfin_spi_tx_dma_isr, drv_data);

	status = request_dma(drv_data->rx_dma, "SPI_RX_DMA");
	if (status) {
		dev_err(dev, "Unable to request SPI RX DMA channel\n");
		free_dma(drv_data->tx_dma);
		return status;
	}
	/* rx dma is enabled when resume in spi boot mode */
	disable_dma(drv_data->rx_dma);
	set_dma_callback(drv_data->rx_dma, bfin_spi_rx_dma_isr, drv_data);

	bfin_write(&drv_data->regs->control, drv_data->control);
	bfin_write(&drv_data->regs->ssel, drv_data->ssel);

	/* Start the queue running */
	status = bfin_spi_start_queue(drv_data);
	if (status != 0) {
		dev_err(dev, "problem starting queue (%d)\n", status);
		free_dma(drv_data->rx_dma);
		free_dma(drv_data->tx_dma);
		return status;
	}

	return 0;
}
static const struct dev_pm_ops bfin_spi_pm_ops = {
	.suspend = bfin_spi_suspend,
	.resume  = bfin_spi_resume,
};
#endif /* CONFIG_PM */

MODULE_ALIAS("platform:bfin-spi");
static struct platform_driver bfin_spi_driver = {
	.driver	= {
		.name	= "bfin-spi",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm     = &bfin_spi_pm_ops,
#endif
	},
	.remove		= bfin_spi_remove,
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
