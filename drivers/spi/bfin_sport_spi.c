/*
 * Emulate a SPI bus using the Blackfin SPORT peripheral
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Copyright 2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>

#include <asm/gpio.h>
#include <asm/portmux.h>
#include <asm/bfin5xx_spi.h>
#include <asm/blackfin.h>
#include <asm/bfin_sport.h>
#include <asm/cacheflush.h>

#define DRV_NAME	"bfin-sport-spi"
#define DRV_DESC	"Emulate a SPI bus with via Blackfin SPORT"

MODULE_AUTHOR("Cliff Cai");
MODULE_DESCRIPTION(DRV_DESC);
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bfin-sport-spi");

#define START_STATE	((void *)0)
#define RUNNING_STATE	((void *)1)
#define DONE_STATE	((void *)2)
#define ERROR_STATE	((void *)-1)

struct master_data;

struct transfer_ops {
	void (*write) (struct master_data *);
	void (*read) (struct master_data *);
	void (*duplex) (struct master_data *);
};

struct master_data {
	/* Driver model hookup */
	struct platform_device *pdev;

	/* SPI framework hookup */
	struct spi_master *master;

	/* Regs base of SPI controller */
	volatile struct sport_register __iomem *regs;

	int err_irq;

	/* Pin request list */
	u16 *pin_req;

	/* BFIN hookup */
	struct bfin5xx_spi_master *master_info;

	/* Driver message queue */
	struct workqueue_struct *workqueue;
	struct work_struct pump_messages;
	spinlock_t lock;
	struct list_head queue;
	int busy;
	bool run;
#ifdef CONFIG_SPI_BFIN_LOCK
	/* SPI bus is lock by a slave for exclusive access */
	int locked;
#endif
	/* Message Transfer pump */
	struct tasklet_struct pump_transfers;

	/* Current message transfer state info */
	struct spi_message *cur_msg;
	struct spi_transfer *cur_transfer;
	struct slave_data *cur_chip;
	size_t len_in_bytes;
	size_t len;
	void *tx;
	void *tx_end;
	void *rx;
	void *rx_end;

	size_t rx_map_len;
	size_t tx_map_len;
	u8 n_bytes;
	int cs_change;
	struct transfer_ops *ops;
};

struct slave_data {
	u8 chip_select_num;
	u8 n_bytes;
	u8 width;		/* 0 or 1 */
	u8 enable_dma;
	u8 bits_per_word;	/* 8 or 16 */
	u16 cs_chg_udelay;	/* Some devices require > 255usec delay */
	u16 ctl_reg;
	u16 baud;
	u16 flag;
	u16 idle_tx_val;
	u32 cs_gpio;
	u32 speed;
	struct transfer_ops *ops;
};

static void bfin_sport_spi_enable(struct master_data *drv_data)
{
	drv_data->regs->tcr1 |= TSPEN;
	drv_data->regs->rcr1 |= TSPEN;
	SSYNC();
}

static void bfin_sport_spi_disable(struct master_data *drv_data)
{
	drv_data->regs->tcr1 &= ~TSPEN;
	drv_data->regs->rcr1 &= ~TSPEN;
	SSYNC();
}

/* Caculate the SPI_BAUD register value based on input HZ */
static u16 hz_to_spi_baud(u32 speed_hz)
{
	u_long clk, sclk = get_sclk();
	int div = (sclk / (2 * speed_hz)) - 1;

	if (div < 0)
		div = 0;

	clk = sclk / (2 * (div + 1));

	if (clk > speed_hz)
		div++;

	return div;
}

/* Chip select operation functions for cs_change flag */
static void bfin_sport_spi_cs_active(struct master_data *drv_data, struct slave_data *chip)
{
	gpio_direction_output(chip->cs_gpio, 0);
}

static void bfin_sport_spi_cs_deactive(struct master_data *drv_data, struct slave_data *chip)
{
	gpio_direction_output(chip->cs_gpio, 1);
	/* Move delay here for consistency */
	if (chip->cs_chg_udelay)
		udelay(chip->cs_chg_udelay);
}

/* stop controller and re-config current chip*/
static void bfin_sport_spi_restore_state(struct master_data *drv_data)
{
	struct slave_data *chip = drv_data->cur_chip;

	bfin_sport_spi_disable(drv_data);
	dev_dbg(&drv_data->pdev->dev, "restoring spi ctl state\n");

	drv_data->regs->tcr1 = chip->ctl_reg;
	drv_data->regs->tcr2 = chip->bits_per_word - 1;
	drv_data->regs->tclkdiv = chip->baud;
	drv_data->regs->tfsdiv = chip->bits_per_word - 1;
	SSYNC();

	drv_data->regs->rcr1 = chip->ctl_reg & ~(ITCLK | ITFS);
	drv_data->regs->rcr2 = chip->bits_per_word - 1;
	SSYNC();

	bfin_sport_spi_cs_active(drv_data, chip);
}

static void bfin_sport_spi_u8_writer(struct master_data *drv_data)
{
	u16 dummy;

	bfin_sport_spi_enable(drv_data);
	while (drv_data->tx < drv_data->tx_end) {
		drv_data->regs->tx16 = *(u8 *)(drv_data->tx++);
		while (!(drv_data->regs->stat & RXNE))
			cpu_relax();
		dummy = drv_data->regs->rx16;
	}
	bfin_sport_spi_disable(drv_data);
}

static void bfin_sport_spi_u8_reader(struct master_data *drv_data)
{
	u16 tx_val = drv_data->cur_chip->idle_tx_val;

	bfin_sport_spi_enable(drv_data);
	while (drv_data->rx < drv_data->rx_end) {
		drv_data->regs->tx16 = tx_val;
		while (!(drv_data->regs->stat & RXNE))
			cpu_relax();
		*(u8 *) (drv_data->rx++) = drv_data->regs->rx16;
	}
	bfin_sport_spi_disable(drv_data);
}

static void bfin_sport_spi_u8_duplex(struct master_data *drv_data)
{
	bfin_sport_spi_enable(drv_data);
	while (drv_data->rx < drv_data->rx_end) {
		drv_data->regs->tx16 = *(u8 *)(drv_data->tx++);
		while (!(drv_data->regs->stat & RXNE))
			cpu_relax();
		*(u8 *) (drv_data->rx++) = drv_data->regs->rx16;
	}
	bfin_sport_spi_disable(drv_data);
}

static struct transfer_ops bfin_transfer_ops_u8 = {
	.write  = bfin_sport_spi_u8_writer,
	.read   = bfin_sport_spi_u8_reader,
	.duplex = bfin_sport_spi_u8_duplex,
};

static void bfin_sport_spi_u16_writer(struct master_data *drv_data)
{
	u16 dummy;

	bfin_sport_spi_enable(drv_data);
	while (drv_data->tx < drv_data->tx_end) {
		drv_data->regs->tx16 = *(u16 *)drv_data->tx;
		drv_data->tx += 2;
		while (!(drv_data->regs->stat & RXNE))
			cpu_relax();
		dummy = drv_data->regs->rx16;
	}
	bfin_sport_spi_disable(drv_data);
}

static void bfin_sport_spi_u16_reader(struct master_data *drv_data)
{
	u16 tx_val = drv_data->cur_chip->idle_tx_val;

	bfin_sport_spi_enable(drv_data);
	while (drv_data->rx < drv_data->rx_end) {
		drv_data->regs->tx16 = tx_val;
		while (!(drv_data->regs->stat & RXNE))
			cpu_relax();
		*(u16 *) (drv_data->rx) = drv_data->regs->rx16;
		drv_data->rx += 2;
	}
	bfin_sport_spi_disable(drv_data);
}

static void bfin_sport_spi_u16_duplex(struct master_data *drv_data)
{
	bfin_sport_spi_enable(drv_data);
	while (drv_data->rx < drv_data->rx_end) {
		drv_data->regs->tx16 = *(u16 *)drv_data->tx;
		drv_data->tx += 2;
		while (!(drv_data->regs->stat & RXNE))
			cpu_relax();
		*(u16 *) (drv_data->rx) = drv_data->regs->rx16;
		drv_data->rx += 2;
	}
	bfin_sport_spi_disable(drv_data);
}

static struct transfer_ops bfin_transfer_ops_u16 = {
	.write  = bfin_sport_spi_u16_writer,
	.read   = bfin_sport_spi_u16_reader,
	.duplex = bfin_sport_spi_u16_duplex,
};

/* test if ther is more transfer to be done */
static void *bfin_sport_spi_next_transfer(struct master_data *drv_data)
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

/*
 * caller already set message->status;
 * dma and pio irqs are blocked give finished message back
 */
static void bfin_sport_spi_giveback(struct master_data *drv_data)
{
	struct slave_data *chip = drv_data->cur_chip;
	struct spi_transfer *last_transfer;
	unsigned long flags;
	struct spi_message *msg;

	spin_lock_irqsave(&drv_data->lock, flags);
	msg = drv_data->cur_msg;
	drv_data->cur_msg = NULL;
	drv_data->cur_transfer = NULL;
	drv_data->cur_chip = NULL;
	queue_work(drv_data->workqueue, &drv_data->pump_messages);
	spin_unlock_irqrestore(&drv_data->lock, flags);

	last_transfer = list_entry(msg->transfers.prev,
				   struct spi_transfer, transfer_list);

	msg->state = NULL;

	if (!drv_data->cs_change)
		bfin_sport_spi_cs_deactive(drv_data, chip);

	if (msg->complete)
		msg->complete(msg->context);
}

static irqreturn_t sport_err_handler(int irq, void *dev_id)
{
	struct master_data *drv_data = dev_id;
	u16 status;

	dev_dbg(&drv_data->pdev->dev, "%s enter\n", __func__);
	status = drv_data->regs->stat & (TOVF | TUVF | ROVF | RUVF);

	if (status) {
		drv_data->regs->stat = status;
		SSYNC();

		bfin_sport_spi_disable(drv_data);
		dev_err(&drv_data->pdev->dev, "status error:%s%s%s%s\n",
			status & TOVF ? " TOVF" : "",
			status & TUVF ? " TUVF" : "",
			status & ROVF ? " ROVF" : "",
			status & RUVF ? " RUVF" : "");
	}

	return IRQ_HANDLED;
}

static void bfin_sport_spi_pump_transfers(unsigned long data)
{
	struct master_data *drv_data = (struct master_data *)data;
	struct spi_message *message = NULL;
	struct spi_transfer *transfer = NULL;
	struct spi_transfer *previous = NULL;
	struct slave_data *chip = NULL;
	u8 width = 0;
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
		bfin_sport_spi_giveback(drv_data);
		return;
	}

	/* Handle end of message */
	if (message->state == DONE_STATE) {
		dev_dbg(&drv_data->pdev->dev, "transfer: all done!\n");
		message->status = 0;
		bfin_sport_spi_giveback(drv_data);
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

	if (transfer->len == 0) {
		/* Move to next transfer of this msg */
		message->state = bfin_sport_spi_next_transfer(drv_data);
		/* Schedule next transfer tasklet */
		tasklet_schedule(&drv_data->pump_transfers);
	}

	if (transfer->tx_buf != NULL) {
		drv_data->tx = (void *)transfer->tx_buf;
		drv_data->tx_end = drv_data->tx + transfer->len;
		dev_dbg(&drv_data->pdev->dev, "tx_buf is %p, tx_end is %p\n",
			transfer->tx_buf, drv_data->tx_end);
	} else
		drv_data->tx = NULL;

	if (transfer->rx_buf != NULL) {
		full_duplex = transfer->tx_buf != NULL;
		drv_data->rx = transfer->rx_buf;
		drv_data->rx_end = drv_data->rx + transfer->len;
		dev_dbg(&drv_data->pdev->dev, "rx_buf is %p, rx_end is %p\n",
			transfer->rx_buf, drv_data->rx_end);
	} else
		drv_data->rx = NULL;

	drv_data->len_in_bytes = transfer->len;
	drv_data->cs_change = transfer->cs_change;

	/* Bits per word setup */
	switch (transfer->bits_per_word) {
	case 8:
		drv_data->n_bytes = 1;
		width = CFG_SPI_WORDSIZE8;
		drv_data->ops = &bfin_transfer_ops_u8;
		break;

	case 16:
		drv_data->n_bytes = 2;
		width = CFG_SPI_WORDSIZE16;
		drv_data->ops = &bfin_transfer_ops_u16;
		break;

	default:
		drv_data->n_bytes = chip->n_bytes;
		width = chip->width;
		drv_data->ops = chip->ops;
		break;
	}

	if (width == CFG_SPI_WORDSIZE16)
		drv_data->len = transfer->len >> 1;
	else
		drv_data->len = transfer->len;

	message->state = RUNNING_STATE;

	if (drv_data->cs_change)
		bfin_sport_spi_cs_active(drv_data, chip);

	dev_dbg(&drv_data->pdev->dev,
		"now pumping a transfer: width is %d, len is %d\n",
		width, transfer->len);

	/* PIO mode write then read */
	dev_dbg(&drv_data->pdev->dev, "doing IO transfer\n");

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
		dev_dbg(&drv_data->pdev->dev,
			"IO write error!\n");
		message->state = ERROR_STATE;
	} else {
		/* Update total byte transfered */
		message->actual_length += drv_data->len_in_bytes;
		/* Move to next transfer of this msg */
		message->state = bfin_sport_spi_next_transfer(drv_data);
		if (drv_data->cs_change)
			bfin_sport_spi_cs_deactive(drv_data, chip);
	}

	/* Schedule next transfer tasklet */
	tasklet_schedule(&drv_data->pump_transfers);
}

/* pop a msg from queue and kick off real transfer */
static void bfin_sport_spi_pump_messages(struct work_struct *work)
{
	struct master_data *drv_data;
	unsigned long flags;
	struct spi_message *next_msg;
#ifdef CONFIG_SPI_BFIN_LOCK
	int locked_cs = -1;
	struct spi_message *msg = NULL;
#endif

	drv_data = container_of(work, struct master_data, pump_messages);

	/* Lock queue and check for queue work */
	spin_lock_irqsave(&drv_data->lock, flags);
	if (list_empty(&drv_data->queue) || !drv_data->run) {
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
	next_msg = list_entry(drv_data->queue.next,
		struct spi_message, queue);

#ifdef CONFIG_SPI_BFIN_LOCK
	if (drv_data->locked)
		locked_cs = drv_data->locked;

	/* Someone has locked the bus */
	if (drv_data->locked && next_msg->spi->chip_select != locked_cs) {
		list_for_each_entry(msg, &drv_data->queue, queue) {
			if (msg->spi->chip_select == locked_cs) {
				next_msg = msg;
				break;
			}
		}
		/* Do nothing even if there are messages for other devices */
		if (next_msg->spi->chip_select != locked_cs) {
			drv_data->busy = 0;
			spin_unlock_irqrestore(&drv_data->lock, flags);
			return;
		}
	}
#endif

	drv_data->cur_msg = next_msg;

	/* Setup the SSP using the per chip configuration */
	drv_data->cur_chip = spi_get_ctldata(drv_data->cur_msg->spi);

	list_del_init(&drv_data->cur_msg->queue);

	/* Initial message state */
	drv_data->cur_msg->state = START_STATE;
	drv_data->cur_transfer = list_entry(drv_data->cur_msg->transfers.next,
					    struct spi_transfer, transfer_list);
	bfin_sport_spi_restore_state(drv_data);
	dev_dbg(&drv_data->pdev->dev, "got a message to pump, "
		"state is set to: baud %d, flag 0x%x, ctl 0x%x\n",
		drv_data->cur_chip->baud, drv_data->cur_chip->flag,
		drv_data->cur_chip->ctl_reg);

	dev_dbg(&drv_data->pdev->dev,
		"the first transfer len is %d\n",
		drv_data->cur_transfer->len);

	/* Mark as busy and launch transfers */
	tasklet_schedule(&drv_data->pump_transfers);

	drv_data->busy = 1;
	spin_unlock_irqrestore(&drv_data->lock, flags);
}

/*
 * lock the spi bus for exclusive access
 */
#ifdef CONFIG_SPI_BFIN_LOCK
static int bfin_sport_spi_lock_bus(struct spi_device *spi)
{
	struct master_data *drv_data = spi_master_get_devdata(spi->master);
	unsigned long flags;

	spin_lock_irqsave(&drv_data->lock, flags);
	if (drv_data->locked) {
		spin_unlock_irqrestore(&drv_data->lock, flags);
		return -ENOLCK;
	}
	drv_data->locked = spi->chip_select;
	spin_unlock_irqrestore(&drv_data->lock, flags);

	return 0;
}

static int bfin_sport_spi_unlock_bus(struct spi_device *spi)
{
	struct master_data *drv_data = spi_master_get_devdata(spi->master);
	unsigned long flags;

	spin_lock_irqsave(&drv_data->lock, flags);
	drv_data->locked = 0;
	spin_unlock_irqrestore(&drv_data->lock, flags);

	return 0;
}
#else
# define bfin_sport_spi_lock_bus   NULL
# define bfin_sport_spi_unlock_bus NULL
#endif

/*
 * got a msg to transfer, queue it in drv_data->queue.
 * And kick off message pumper
 */
static int bfin_sport_spi_transfer(struct spi_device *spi, struct spi_message *msg)
{
	struct master_data *drv_data = spi_master_get_devdata(spi->master);
	unsigned long flags;

	spin_lock_irqsave(&drv_data->lock, flags);

	if (!drv_data->run) {
		spin_unlock_irqrestore(&drv_data->lock, flags);
		return -ESHUTDOWN;
	}

	msg->actual_length = 0;
	msg->status = -EINPROGRESS;
	msg->state = START_STATE;

	dev_dbg(&spi->dev, "adding an msg in transfer() \n");
	list_add_tail(&msg->queue, &drv_data->queue);

	if (drv_data->run && !drv_data->busy)
		queue_work(drv_data->workqueue, &drv_data->pump_messages);

	spin_unlock_irqrestore(&drv_data->lock, flags);

	return 0;
}

/* first setup for new devices */
static int bfin_sport_spi_setup(struct spi_device *spi)
{
	struct bfin5xx_spi_chip *chip_info = NULL;
	struct slave_data *chip;
	struct master_data *drv_data = spi_master_get_devdata(spi->master);
	int ret = 0;

	if (spi->bits_per_word != 8 && spi->bits_per_word != 16)
		return -EINVAL;

	/* Only alloc (or use chip_info) on first setup */
	chip = spi_get_ctldata(spi);
	if (chip == NULL) {
		chip = kzalloc(sizeof(*chip), GFP_KERNEL);
		if (!chip)
			return -ENOMEM;

		chip->enable_dma = 0;
		chip_info = spi->controller_data;
	}

	/* chip_info isn't always needed */
	if (chip_info) {
		chip->ctl_reg = chip_info->ctl_reg;
		chip->enable_dma = chip_info->enable_dma;
		chip->bits_per_word = chip_info->bits_per_word;
		chip->cs_chg_udelay = chip_info->cs_chg_udelay;
		chip->cs_gpio = chip_info->cs_gpio;
		chip->idle_tx_val = chip_info->idle_tx_val;
	}

	/* translate common spi framework into our register
	 * following configure contents are same for tx and rx.
	 */

	if (spi->mode & SPI_CPHA)
		chip->ctl_reg &= ~TCKFE;
	else
		chip->ctl_reg |= TCKFE;

	if (spi->mode & SPI_LSB_FIRST)
		chip->ctl_reg |= TLSBIT;

	/* Sport in master mode */
	chip->ctl_reg |= ITCLK | ITFS | TFSR | LATFS | LTFS;

	/*
	 * Notice: for blackfin, the speed_hz is the value of register
	 * SPI_BAUD, not the real baudrate
	 */
	chip->baud = hz_to_spi_baud(spi->max_speed_hz);
	chip->flag = 1 << spi->chip_select;
	chip->chip_select_num = spi->chip_select;

	switch (chip->bits_per_word) {
	case 8:
		chip->n_bytes = 1;
		chip->width = CFG_SPI_WORDSIZE8;
		chip->ops = &bfin_transfer_ops_u8;
		break;

	case 16:
		chip->n_bytes = 2;
		chip->width = CFG_SPI_WORDSIZE16;
		chip->ops = &bfin_transfer_ops_u16;
		break;

	default:
		dev_err(&spi->dev, "%d bits_per_word is not supported\n",
				chip->bits_per_word);
		if (chip_info)
			kfree(chip);
		return -ENODEV;
	}

	dev_dbg(&spi->dev, "setup spi chip %s, width is %d, dma is %d\n",
			spi->modalias, chip->width, chip->enable_dma);
	dev_dbg(&spi->dev, "ctl_reg is 0x%x, flag_reg is 0x%x\n",
			chip->ctl_reg, chip->flag);

	spi_set_ctldata(spi, chip);

	if (chip->chip_select_num == 0) {
		ret = gpio_request(chip->cs_gpio, spi->modalias);
		if (ret)
			dev_err(&spi->dev, "request GPIO CS failed\n");
		else
			gpio_direction_output(chip->cs_gpio, 1);
	}

	bfin_sport_spi_cs_deactive(drv_data, chip);

	return ret;
}

/*
 * callback for spi framework.
 * clean driver specific data
 */
static void bfin_sport_spi_cleanup(struct spi_device *spi)
{
	struct slave_data *chip = spi_get_ctldata(spi);

	if (!chip)
		return;

	if (chip->chip_select_num == 0)
		gpio_free(chip->cs_gpio);

	kfree(chip);
}

static inline int bfin_sport_spi_init_queue(struct master_data *drv_data)
{
	INIT_LIST_HEAD(&drv_data->queue);
	spin_lock_init(&drv_data->lock);

#ifdef CONFIG_SPI_BFIN_LOCK
	drv_data->locked = 0;
#endif
	drv_data->run = false;
	drv_data->busy = 0;

	/* init transfer tasklet */
	tasklet_init(&drv_data->pump_transfers,
		     bfin_sport_spi_pump_transfers, (unsigned long)drv_data);

	/* init messages workqueue */
	INIT_WORK(&drv_data->pump_messages, bfin_sport_spi_pump_messages);
	drv_data->workqueue =
	    create_singlethread_workqueue(dev_name(drv_data->master->dev.parent));
	if (drv_data->workqueue == NULL)
		return -EBUSY;

	return 0;
}

static inline int bfin_sport_spi_start_queue(struct master_data *drv_data)
{
	unsigned long flags;

	spin_lock_irqsave(&drv_data->lock, flags);

	if (drv_data->run || drv_data->busy) {
		spin_unlock_irqrestore(&drv_data->lock, flags);
		return -EBUSY;
	}

	drv_data->run = true;
	drv_data->cur_msg = NULL;
	drv_data->cur_transfer = NULL;
	drv_data->cur_chip = NULL;
	spin_unlock_irqrestore(&drv_data->lock, flags);

	queue_work(drv_data->workqueue, &drv_data->pump_messages);

	return 0;
}

static inline int bfin_sport_spi_stop_queue(struct master_data *drv_data)
{
	unsigned long flags;
	unsigned limit = 500;
	int status = 0;

	spin_lock_irqsave(&drv_data->lock, flags);

#ifdef CONFIG_SPI_BFIN_LOCK
	drv_data->locked = 0;
#endif
	/*
	 * This is a bit lame, but is optimized for the common execution path.
	 * A wait_queue on the drv_data->busy could be used, but then the common
	 * execution path (pump_messages) would be required to call wake_up or
	 * friends on every SPI message. Do this instead
	 */
	drv_data->run = false;
	while (!list_empty(&drv_data->queue) && drv_data->busy && limit--) {
		spin_unlock_irqrestore(&drv_data->lock, flags);
		msleep(10);
		spin_lock_irqsave(&drv_data->lock, flags);
	}

	if (!list_empty(&drv_data->queue) || drv_data->busy)
		status = -EBUSY;

	spin_unlock_irqrestore(&drv_data->lock, flags);

	return status;
}

static inline int bfin_sport_spi_destroy_queue(struct master_data *drv_data)
{
	int status;

	status = bfin_sport_spi_stop_queue(drv_data);
	if (status)
		return status;

	destroy_workqueue(drv_data->workqueue);

	return 0;
}

static int __devinit bfin_sport_spi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bfin5xx_spi_master *platform_info;
	struct spi_master *master;
	struct resource *res, *ires;
	struct master_data *drv_data;
	int status;

	platform_info = dev->platform_data;

	/* Allocate master with space for drv_data */
	master = spi_alloc_master(dev, sizeof(*master) + 16);
	if (!master) {
		dev_err(&pdev->dev, "cannot alloc spi_master\n");
		return -ENOMEM;
	}

	drv_data = spi_master_get_devdata(master);
	drv_data->master = master;
	drv_data->master_info = platform_info;
	drv_data->pdev = pdev;
	drv_data->pin_req = platform_info->pin_req;

	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST;
	master->bus_num = pdev->id;
	master->num_chipselect = platform_info->num_chipselect;
	master->cleanup = bfin_sport_spi_cleanup;
	master->setup = bfin_sport_spi_setup;
	master->transfer = bfin_sport_spi_transfer;
	master->lock_bus = bfin_sport_spi_lock_bus;
	master->unlock_bus = bfin_sport_spi_unlock_bus;

	/* Find and map our resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		status = -ENOENT;
		goto out_error_get_res;
	}

	drv_data->regs = ioremap(res->start, resource_size(res));
	if (drv_data->regs == NULL) {
		dev_err(&pdev->dev, "cannot map registers\n");
		status = -ENXIO;
		goto out_error_ioremap;
	}

	ires = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!ires) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_IRQ\n");
		status = -ENODEV;
		goto out_error_get_ires;
	}
	drv_data->err_irq = ires->start;

	/* Initial and start queue */
	status = bfin_sport_spi_init_queue(drv_data);
	if (status) {
		dev_err(&pdev->dev, "problem initializing queue\n");
		goto out_error_queue_alloc;
	}

	status = bfin_sport_spi_start_queue(drv_data);
	if (status) {
		dev_err(&pdev->dev, "problem starting queue\n");
		goto out_error_queue_alloc;
	}

	status = request_irq(drv_data->err_irq, sport_err_handler,
		0, "sport_spi_err", drv_data);
	if (status) {
		dev_err(&pdev->dev, "unable to request sport err irq\n");
		goto out_error_irq;
	}

	status = peripheral_request_list(drv_data->pin_req, DRV_NAME);
	if (status) {
		dev_err(&pdev->dev, "requesting peripherals failed\n");
		goto out_error_peripheral;
	}

	/* Register with the SPI framework */
	platform_set_drvdata(pdev, drv_data);
	status = spi_register_master(master);
	if (status) {
		dev_err(&pdev->dev, "problem registering spi master\n");
		goto out_error_master;
	}

	dev_info(dev, "%s, regs_base@%p\n", DRV_DESC, drv_data->regs);
	return 0;

 out_error_master:
	peripheral_free_list(drv_data->pin_req);
 out_error_peripheral:
	free_irq(drv_data->err_irq, drv_data);
 out_error_irq:
 out_error_queue_alloc:
	bfin_sport_spi_destroy_queue(drv_data);
 out_error_get_ires:
	iounmap((void *)drv_data->regs);
 out_error_ioremap:
 out_error_get_res:
	spi_master_put(master);

	return status;
}

/* stop hardware and remove the driver */
static int __devexit bfin_sport_spi_remove(struct platform_device *pdev)
{
	struct master_data *drv_data = platform_get_drvdata(pdev);
	int status = 0;

	if (!drv_data)
		return 0;

	/* Remove the queue */
	status = bfin_sport_spi_destroy_queue(drv_data);
	if (status)
		return status;

	/* Disable the SSP at the peripheral and SOC level */
	bfin_sport_spi_disable(drv_data);

	/* Disconnect from the SPI framework */
	spi_unregister_master(drv_data->master);

	peripheral_free_list(drv_data->pin_req);

	/* Prevent double remove */
	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM
static int bfin_sport_spi_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct master_data *drv_data = platform_get_drvdata(pdev);
	int status = 0;

	status = bfin_sport_spi_stop_queue(drv_data);
	if (status)
		return status;

	/* stop hardware */
	bfin_sport_spi_disable(drv_data);

	return 0;
}

static int bfin_sport_spi_resume(struct platform_device *pdev)
{
	struct master_data *drv_data = platform_get_drvdata(pdev);
	int status = 0;

	/* Enable the SPI interface */
	bfin_sport_spi_enable(drv_data);

	/* Start the queue running */
	status = bfin_sport_spi_start_queue(drv_data);
	if (status) {
		dev_err(&pdev->dev, "problem resuming queue\n");
		return status;
	}

	return 0;
}
#else
# define bfin_sport_spi_suspend NULL
# define bfin_sport_spi_resume  NULL
#endif

static struct platform_driver bfin_sport_spi_driver = {
	.driver	= {
		.name = DRV_NAME,
	},
	.probe   = bfin_sport_spi_probe,
	.remove  = __devexit_p(bfin_sport_spi_remove),
	.suspend = bfin_sport_spi_suspend,
	.resume  = bfin_sport_spi_resume,
};

static int __init bfin_sport_spi_init(void)
{
	return platform_driver_register(&bfin_sport_spi_driver);
}
module_init(bfin_sport_spi_init);

static void __exit bfin_sport_spi_exit(void)
{
	platform_driver_unregister(&bfin_sport_spi_driver);
}
module_exit(bfin_sport_spi_exit);
