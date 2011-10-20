/*
 * ADF702x Narrow-Band Short-Range Radio Transceiver
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Copyright (C) 2009-2010 Michael Hennerich, Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/rtnetlink.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/random.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <asm/dma.h>
#include <asm/portmux.h>
#include <asm/bfin_sport.h>

#include <linux/spi/adf702x.h>

/*
 * DEBUG LEVEL
 * 	0 	OFF
 *	1	ERRORS
 *	2	ERRORS + TRACE
 *	3	ERRORS + TRACE + PACKET DUMP
 */

#define ADF_DEBUG 0
#define DBG(n, args...) do { if (ADF_DEBUG >= (n)) pr_debug(args); } while (0)

struct adf702x_priv {
	struct spi_device *spi;
	struct net_device *ndev;
	struct sk_buff *tx_skb;
	struct delayed_work tx_work;
	struct work_struct tx_done_work;
	wait_queue_head_t waitq;
	dma_addr_t dma_handle;
	spinlock_t lock;
	unsigned rx_preamble:1;
	unsigned rx:1;
	unsigned rx_size;
	u32 *rx_buf;
	u32 *tx_buf;

	/* Base reg base of SPORT controller */
	volatile struct sport_register *sport;
	unsigned dma_ch_rx;
	unsigned dma_ch_tx;
	unsigned irq_sport_err;
	unsigned gpio_int_rfs;
};

static const u16 sym2chip[] = {
	0x744A,
	0x44AC,
	0x4AC3,
	0xAC39,
	0xC39B,
	0x39B7,
	0x9B74,
	0xB744,
	0xDEE2,
	0xEE26,
	0xE269,
	0x2693,
	0x6931,
	0x931D,
	0x31DE,
	0x1DEE,
};

#define PKT_MAGIC	(0xA54662DA)	/* Packet Valid Magic */
#define RX_HEADERSIZE	4	/* ?(PREAMBLE) + PKT_MAGIC + LEN_HI + LEN_LO */
#define TX_HEADERSIZE	5	/* PREAMBLE + PREAMBLE + PKT_MAGIC + LEN_HI + LEN_LO */
#define FIFO_WA		6	/* Transfer additional words to workaround DMA FIFO issues */
#define BITERR 		4	/* MAX Bit Errors Allowed */
#define REG0_DELAY	40	/* PLL settling delay in us */

#define MAX_PACKET_SIZE	(1550 * 4)

/*
 * Get 32-bit chip from 8-bit symbol
 */
static inline unsigned int adf702x_getchip(unsigned char sym)
{
	return (sym2chip[sym >> 4] << 16) | sym2chip[sym & 0xF];
}

/*
 * Test packet valid magic
 */
static inline int adf702x_testpkt_magic(struct adf702x_priv *lp)
{
	if (hweight32(lp->rx_buf[1] ^ PKT_MAGIC) < BITERR)
		return 1;

	if (hweight32(lp->rx_buf[0] ^ PKT_MAGIC) < BITERR)
		return 0;

	return -1;
}

/*
 * Get 8-bit symbol from 32-bit chip
 * Returns: symbol or -1 in case of an unrecoverable error
 */
static int adf702x_getsymbol(unsigned int chip)
{
	int symhi, symlo;
	unsigned chiphi, chiplo;

	chiphi = chip >> 16;
	chiplo = chip & 0xFFFF;

	for (symhi = 0; symhi < ARRAY_SIZE(sym2chip); symhi++)
		if (hweight32(chiphi ^ sym2chip[symhi]) < BITERR)
			break;

	if (symhi >= ARRAY_SIZE(sym2chip))
		return -1;

	for (symlo = 0; symlo < ARRAY_SIZE(sym2chip); symlo++)
		if (hweight32(chiplo ^ sym2chip[symlo]) < BITERR)
			break;

	if (symlo >= ARRAY_SIZE(sym2chip))
		return -1;

	return (symhi << 4) | symlo;
}

/*
 * Get Packet size from header
 * Returns: size or 64 in case of an unrecoverable error
 */
static inline unsigned short adf702x_getrxsize(struct adf702x_priv *lp, int offset)
{
	int size = adf702x_getsymbol(lp->rx_buf[offset + 1]) << 8 |
			adf702x_getsymbol(lp->rx_buf[offset + 2]);

	if (size > 0)
		return size;

	DBG(1, "%s :BITERR\n", __func__);
	lp->ndev->stats.rx_errors++;
	return 64;	/* Keep the Receiver busy for some time */
}

static int adf702x_spi_write(struct spi_device *spi, unsigned int data)
{
	u16 msg[2];

	msg[0] = data >> 16;
	msg[1] = data;

	return spi_write(spi, (u8 *)msg, 4);
}

static int adf702x_init(struct spi_device *spi)
{
	struct adf702x_platform_data *pdata = spi->dev.platform_data;

	switch (pdata->adf702x_model) {
	case MODEL_ADF7025:
		adf702x_spi_write(spi, pdata->adf702x_regs[2]);
		adf702x_spi_write(spi, pdata->adf702x_regs[0]);
		adf702x_spi_write(spi, pdata->adf702x_regs[1]);
		udelay(1000);
		adf702x_spi_write(spi, pdata->adf702x_regs[3]);
		adf702x_spi_write(spi, pdata->adf702x_regs[4]);
		adf702x_spi_write(spi, pdata->adf702x_regs[6]);
		udelay(300);
		adf702x_spi_write(spi, pdata->adf702x_regs[5]);
		adf702x_spi_write(spi, pdata->adf702x_regs[9]);
		break;
	case MODEL_ADF7021:
		adf702x_spi_write(spi, pdata->adf702x_regs[1]);
		udelay(800);
		adf702x_spi_write(spi, pdata->adf702x_regs[3]);
		adf702x_spi_write(spi, pdata->adf702x_regs[6]);
		adf702x_spi_write(spi, pdata->adf702x_regs[5]);
		udelay(300);
		adf702x_spi_write(spi, pdata->adf702x_regs[11]);
		adf702x_spi_write(spi, pdata->adf702x_regs[12]);
		adf702x_spi_write(spi, pdata->adf702x_regs[0]);
		adf702x_spi_write(spi, pdata->adf702x_regs[4]);
		adf702x_spi_write(spi, pdata->adf702x_regs[10]);
		adf702x_spi_write(spi, pdata->adf702x_regs[2]);
		break;
	default:
		dev_err(&spi->dev, "model not supported\n");
		return -ENODEV;
	}

	return 0;
}

static void adf702x_rx(struct spi_device *spi)
{
	struct adf702x_platform_data *pdata = spi->dev.platform_data;
	adf702x_spi_write(spi, pdata->adf702x_regs[0]);
	udelay(REG0_DELAY);
	DBG(2, "%s():\n", __func__);
}

static void adf702x_tx(struct spi_device *spi)
{
	struct adf702x_platform_data *pdata = spi->dev.platform_data;
	adf702x_spi_write(spi, pdata->tx_reg);
	udelay(REG0_DELAY);
	DBG(2, "%s():\n", __func__);
}

static void adf702x_sport_init(struct adf702x_priv *lp)
{
	struct adf702x_platform_data *pdata = lp->spi->dev.platform_data;

	lp->sport->tcr2 = DP_SLEN(32-1);
	lp->sport->tcr1 = TCKFE | LATFS | DITFS | ITFS;
	lp->sport->rcr2 = DP_SLEN(32-1);
	lp->sport->rcr1 = RCKFE | LARFS;

	/*
	 * The ADF7025 requires SPORT TCLK generated externally
	 * it should be within 2% of CDR_CLK/32.
	 */
	if (pdata->adf702x_model == MODEL_ADF7025) {
		lp->sport->tcr1 = TCKFE | LATFS | DITFS | ITFS | ITCLK;
		lp->sport->tclkdiv = pdata->adf7025_tclkdiv;
	}
}

static void adf702x_setup_rx(struct adf702x_priv *lp)
{
	unsigned long flags;

	spin_lock_irqsave(&lp->lock, flags);
	lp->rx_preamble = 1;
	lp->rx = 0;
	set_dma_x_count(lp->dma_ch_rx, RX_HEADERSIZE);
	set_dma_start_addr(lp->dma_ch_rx, (unsigned long)lp->rx_buf);
	enable_dma(lp->dma_ch_rx);
	lp->sport->rcr1 |= RSPEN;
	spin_unlock_irqrestore(&lp->lock, flags);
}

static void adf702x_tx_work(struct work_struct *work)
{
	struct adf702x_priv *lp = container_of(work,
			struct adf702x_priv, tx_work.work);

	DBG(2, "%s: %s(): txDataCount(%d)\n",
		 lp->ndev->name, __func__,  lp->tx_skb->len);

	DBG(2, "GPIO = %d rx = %d\n", gpio_get_value(lp->gpio_int_rfs), lp->rx);

	/*
	 * Do some media sense here
	 * Sleep while the media is busy
	 */

	wait_event(lp->waitq, !(lp->rx || gpio_get_value(lp->gpio_int_rfs)));

	lp->sport->rcr1 &= ~RSPEN;
	disable_dma(lp->dma_ch_rx);
	clear_dma_irqstat(lp->dma_ch_rx);

	adf702x_tx(lp->spi);

	set_dma_x_count(lp->dma_ch_tx, lp->tx_skb->len + TX_HEADERSIZE + FIFO_WA);
	set_dma_start_addr(lp->dma_ch_tx, (unsigned long)lp->tx_buf);
	enable_dma(lp->dma_ch_tx);

	lp->sport->tcr1 |= TSPEN;;

	lp->ndev->stats.tx_packets++;
	lp->ndev->stats.tx_bytes += lp->tx_skb->len;
}

static void adf702x_tx_done_work(struct work_struct *work)
{
	struct adf702x_priv *lp = container_of(work,
			struct adf702x_priv, tx_done_work);

	DBG(2, "%s: %s(): \n", lp->ndev->name, __func__);

	adf702x_setup_rx(lp);
	adf702x_rx(lp->spi);

	if (lp->tx_skb) {
		dev_kfree_skb(lp->tx_skb);
		lp->tx_skb = NULL;
	}

	netif_wake_queue(lp->ndev);
}

static int adf702x_net_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct adf702x_priv *lp = netdev_priv(dev);
	unsigned char *buf_ptr = skb->data;
	int i;
	unsigned char delay;

	DBG(2, "%s: %s(): transmitting %d bytes\n",
		 dev->name, __func__, skb->len);

	/* Only one packet at a time. Once TXDONE interrupt is serviced, the
	 * queue will be restarted.
	 */
	netif_stop_queue(dev);

	/* Remember the skb for deferred processing */
	lp->tx_skb = skb;

	BUG_ON(lp->tx_skb->len > MAX_PACKET_SIZE);

	lp->tx_buf[3] = adf702x_getchip(skb->len >> 8);
	lp->tx_buf[4] = adf702x_getchip(skb->len & 0xFF);

	DBG(3, "TX TX: ");
	for (i = 0; i < skb->len; i++) {
		lp->tx_buf[TX_HEADERSIZE + i] = adf702x_getchip(buf_ptr[i]);
		DBG(3, "%x ", buf_ptr[i]);
	}
	DBG(3, "\n");

	/* Avoid contentions
	 * Schedule transmission randomly (0..64ms)
	 * This allows other nodes to snip in
	 */

	delay = random32() >> 26;
	schedule_delayed_work(&lp->tx_work, msecs_to_jiffies(delay));

	return 0;
}

static int adf702x_receive(struct net_device *dev)
{
	struct sk_buff *skb;
	struct adf702x_priv *lp = netdev_priv(dev);
	int i, ret;
	u8 *data;

	DBG(2, "%s: %s(): \n", lp->ndev->name, __func__);

	skb = dev_alloc_skb(lp->rx_size + NET_IP_ALIGN);
	if (!skb) {
		dev->stats.rx_dropped++;
		return -ENOMEM;
	}

	skb_reserve(skb, NET_IP_ALIGN);

	data = skb_put(skb, lp->rx_size);

	DBG(3, "RX RX: ");
	for (i = 0; i < lp->rx_size; i++) {
		data[i] = ret = adf702x_getsymbol(lp->rx_buf[i]);
		if (ret < 0) {
			dev->stats.rx_errors++;
			dev_kfree_skb(skb);
			DBG(1, "\nRX BITERR chip = %x\n", lp->rx_buf[i]);
			return -EIO;
		}
		DBG(3, "%x ", data[i]);
	}
	DBG(3, "\n");

	skb->protocol = eth_type_trans(skb, dev);
	skb->ip_summed = CHECKSUM_NONE;

	/* No MAC filtering
	 * we're always in Promiscuous Mode
	 */

	netif_rx(skb);

	dev->stats.rx_packets++;
	dev->stats.rx_bytes += lp->rx_size;

	return 0;
}

static irqreturn_t adf702x_sport_err_irq(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct adf702x_priv *lp = netdev_priv(dev);
	unsigned int stat = lp->sport->stat;

	DBG(2, "%s: %s(): \n", lp->ndev->name, __func__);
	/* Overflow in RX FIFO */
	if (stat & ROVF)
		DBG(1, "%s: %s(): ROVF\n", lp->ndev->name, __func__);

	/* These should not happen */
	if (stat & (TOVF | TUVF | RUVF)) {
		DBG(1, "SPORT Error:%s %s %s\n",
		       (stat & TOVF) ? "TX overflow" : "",
		       (stat & TUVF) ? "TX underflow" : "",
		       (stat & RUVF) ? "RX underflow" : "");
	}

	disable_dma(lp->dma_ch_rx);
	clear_dma_irqstat(lp->dma_ch_rx);

	lp->sport->stat = ROVF | RUVF | TUVF | TOVF; /* Clear ROVF bit */
	lp->sport->rcr1 &= ~RSPEN;

	dev->stats.rx_over_errors++;
	dev->stats.rx_errors++;
	adf702x_setup_rx(lp);
	wake_up(&lp->waitq);

	return IRQ_HANDLED;
}

static irqreturn_t adf702x_tx_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct adf702x_priv *lp = netdev_priv(dev);

	DBG(2, "%s:%s(): got TXDone\n",
			 dev->name, __func__);
	lp->sport->tcr1 &= ~TSPEN;
	disable_dma(lp->dma_ch_tx);
	clear_dma_irqstat(lp->dma_ch_tx);
	schedule_work(&lp->tx_done_work);

	return IRQ_HANDLED;
}

static irqreturn_t adf702x_rx_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct adf702x_priv *lp = netdev_priv(dev);
	int offset;
	disable_dma(lp->dma_ch_rx);
	clear_dma_irqstat(lp->dma_ch_rx);

	if (lp->rx_preamble) {
		/*
		 * Keep the SPORT enabled
		 * The FIFO associated with the SPORT acts as buffer -
		 * while we setup the DMA for the remaining chips.
		 */

		offset = adf702x_testpkt_magic(lp);
		if (offset >= 0) {
			lp->rx_size = adf702x_getrxsize(lp, offset);
			if (offset == 1) {
				set_dma_x_count(lp->dma_ch_rx, lp->rx_size);
				set_dma_start_addr(lp->dma_ch_rx,
					(unsigned long)lp->rx_buf);
			} else {
				lp->rx_buf[0] = lp->rx_buf[3];
				set_dma_x_count(lp->dma_ch_rx, lp->rx_size - 1);
				set_dma_start_addr(lp->dma_ch_rx,
					(unsigned long)&lp->rx_buf[1]);
			}
			enable_dma(lp->dma_ch_rx);
			lp->rx = 1;
			lp->rx_preamble = 0;

			DBG(2, "%s:%s(): got RX data %d\n",
				 dev->name, __func__, lp->rx_size);

			return IRQ_HANDLED;

		} else {
			DBG(1, "%s:%s(): Failed PKT_MAGIC\n",
				 dev->name, __func__);
			lp->sport->rcr1 &= ~RSPEN;
			dev->stats.rx_dropped++;
			dev->stats.rx_errors++;
		}
	} else {
		lp->sport->rcr1 &= ~RSPEN;
		adf702x_receive(dev);
	}

	adf702x_setup_rx(lp);
	wake_up(&lp->waitq);

	return IRQ_HANDLED;
}

static int adf702x_net_open(struct net_device *dev)
{
	struct adf702x_priv *lp = netdev_priv(dev);
	struct adf702x_platform_data *pdata = lp->spi->dev.platform_data;
	unsigned int syncword;

	DBG(2, "%s: %s()\n", dev->name, __func__);

	adf702x_sport_init(lp);

	set_dma_config(lp->dma_ch_rx,
			set_bfin_dma_config(DIR_WRITE, DMA_FLOW_STOP,
					INTR_ON_BUF, DIMENSION_LINEAR,
					DATA_SIZE_32, DMA_SYNC_RESTART));

	set_dma_config(lp->dma_ch_tx,
			set_bfin_dma_config(DIR_READ, DMA_FLOW_STOP,
					INTR_ON_BUF, DIMENSION_LINEAR,
					DATA_SIZE_32, DMA_SYNC_RESTART));

	set_dma_x_modify(lp->dma_ch_rx, 4);
	set_dma_x_modify(lp->dma_ch_tx, 4);

	switch (pdata->adf702x_model) {
	case MODEL_ADF7025:
		syncword = 0xAA000000 | (pdata->adf702x_regs[5] >> 8);
		break;
	case MODEL_ADF7021:
		syncword = 0xAA000000 | (pdata->adf702x_regs[11] >> 8);
		break;
	default:
		return -ENODEV;
	}

	lp->tx_buf[0] = syncword; /* SFD Start-of-frame delimiter */
	lp->tx_buf[1] = syncword;
	lp->tx_buf[2] = PKT_MAGIC;

	adf702x_setup_rx(lp);

	dma_enable_irq(lp->dma_ch_rx);
	adf702x_rx(lp->spi);
	netif_start_queue(dev);

	return 0;
}

static int adf702x_net_close(struct net_device *dev)
{
	struct adf702x_priv *lp = netdev_priv(dev);
	DBG(2, "%s: %s()\n", dev->name, __func__);

	netif_stop_queue(dev);
	dma_disable_irq(lp->dma_ch_rx);

	return 0;
}

static const struct net_device_ops adf702x_netdev_ops = {
	.ndo_open		= adf702x_net_open,
	.ndo_stop		= adf702x_net_close,
	.ndo_start_xmit		= adf702x_net_xmit,
	.ndo_change_mtu		= eth_change_mtu,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_set_mac_address	= eth_mac_addr,
};

static int __devinit adf702x_probe(struct spi_device *spi)
{
	struct net_device *ndev;
	struct adf702x_priv *lp;
	struct adf702x_platform_data *pdata = spi->dev.platform_data;
	int err;

	if (!pdata) {
		dev_dbg(&spi->dev, "no platform data?\n");
		return -ENODEV;
	}

	ndev = alloc_etherdev(sizeof(*lp));
	if (!ndev) {
		err = -ENOMEM;
		goto out;
	}

	dev_set_drvdata(&spi->dev, ndev);
	SET_NETDEV_DEV(ndev, &spi->dev);

	ndev->netdev_ops = &adf702x_netdev_ops;

	/*
	 * MAC address? we use a
	 * random one ...
	 */

	if (pdata->mac_addr && is_valid_ether_addr(pdata->mac_addr)) {
		memcpy(ndev->dev_addr, pdata->mac_addr, ETH_ALEN);
	} else {
		dev_warn(&spi->dev, "using random MAC addr\n");
		random_ether_addr(ndev->dev_addr);
	}

	ndev->mtu = 576;
	ndev->tx_queue_len = 10;
	ndev->watchdog_timeo = 0;

	lp = netdev_priv(ndev);
	lp->ndev = ndev;
	lp->spi = spi;

	lp->sport = (struct sport_register *) pdata->regs_base;
	lp->dma_ch_rx = pdata->dma_ch_rx;
	lp->dma_ch_tx = pdata->dma_ch_tx;
	lp->irq_sport_err = pdata->irq_sport_err;
	lp->gpio_int_rfs = pdata->gpio_int_rfs;

	err = peripheral_request_list(pdata->pin_req, dev_name(&spi->dev));
	if (err) {
		dev_err(&spi->dev, "failed to request SPORT\n");
		goto out2;
	}

	err = request_dma(lp->dma_ch_rx, "SPORT RX Data");
	if (err) {
		dev_err(&spi->dev, "failed to request RX dma %d\n", lp->dma_ch_rx);
		goto out3;
	}

	err = request_dma(lp->dma_ch_tx, "SPORT TX Data");
	if (err) {
		dev_err(&spi->dev, "failed to request TX dma %d\n", lp->dma_ch_tx);
		goto out4;
	}

	err = set_dma_callback(lp->dma_ch_rx, adf702x_rx_interrupt, ndev);
	if (err) {
		dev_err(&spi->dev, "failed to request RX irq\n");
		goto out5;
	}

	err = set_dma_callback(lp->dma_ch_tx, adf702x_tx_interrupt, ndev);
	if (err) {
		dev_err(&spi->dev, "failed to request TX irq\n");
		goto out5;
	}

	dma_disable_irq(lp->dma_ch_rx);

	if (lp->irq_sport_err) {
		err = request_irq(lp->irq_sport_err, adf702x_sport_err_irq, 0,
			"ADF702x SPORT_ERROR", ndev);
		if (err) {
			dev_err(&spi->dev, "unable to request SPORT status interrupt\n");
			goto out5;
		}
	}

	lp->rx_buf = dma_alloc_coherent(NULL, MAX_PACKET_SIZE,
		&lp->dma_handle, GFP_KERNEL);

	if (lp->rx_buf == NULL) {
		err = -ENOMEM;
		goto out6;
	}

	lp->tx_buf = dma_alloc_coherent(NULL, MAX_PACKET_SIZE,
		&lp->dma_handle, GFP_KERNEL);

	if (lp->rx_buf == NULL) {
		err = -ENOMEM;
		goto out7;
	}

	/*
	 * This GPIO is connected to the ADF702X INT
	 * The ADF702X SWD feature starts the SPORT RX DMA (INT connected to SPORT RFS)
	 * While the initial transfer runs (PREAMBLE, PKT_MAGIC and Packet length), we sense
	 * this GPIO to see if there is an ongoing transfer.
	 */

	err = gpio_request(lp->gpio_int_rfs, "ADF702X-INT");
	if (err)
		goto out8;

	gpio_direction_input(lp->gpio_int_rfs);

	spi->bits_per_word = 16;
	spi_setup(spi);

	err = adf702x_init(lp->spi);
	if (err)
		goto out9;

	spin_lock_init(&lp->lock);
	INIT_DELAYED_WORK(&lp->tx_work, adf702x_tx_work);
	INIT_WORK(&lp->tx_done_work, adf702x_tx_done_work);
	init_waitqueue_head(&lp->waitq);

	err = register_netdev(ndev);
	if (err) {
		dev_err(&spi->dev, "failed to register netdev\n");
		goto out9;
	}

	dev_info(&spi->dev, "ADF702XNet Wireless Ethernet driver");

	return 0;

out9:
	gpio_free(lp->gpio_int_rfs);
out8:
	dma_free_coherent(NULL, MAX_PACKET_SIZE, lp->tx_buf, lp->dma_handle);
out7:
	dma_free_coherent(NULL, MAX_PACKET_SIZE, lp->rx_buf, lp->dma_handle);
out6:
	if (lp->irq_sport_err)
		free_irq(lp->irq_sport_err, ndev);
out5:
	free_dma(lp->dma_ch_tx);
out4:
	free_dma(lp->dma_ch_rx);
out3:
	peripheral_free_list(pdata->pin_req);
out2:
	unregister_netdev(ndev);
	free_netdev(ndev);
	dev_set_drvdata(&spi->dev, NULL);
out:
	return err;
}

static int __devexit adf702x_remove(struct spi_device *spi)
{
	struct adf702x_platform_data *pdata = spi->dev.platform_data;
	struct net_device *dev = dev_get_drvdata(&spi->dev);
	struct adf702x_priv *lp = netdev_priv(dev);

	rtnl_lock();
	dev_close(dev);
	unregister_netdevice(dev);
	rtnl_unlock();

	dma_free_coherent(NULL, MAX_PACKET_SIZE, lp->rx_buf, lp->dma_handle);
	dma_free_coherent(NULL, MAX_PACKET_SIZE, lp->tx_buf, lp->dma_handle);

	if (lp->irq_sport_err)
		free_irq(lp->irq_sport_err, dev);

	gpio_free(lp->gpio_int_rfs);
	free_dma(lp->dma_ch_rx);
	free_dma(lp->dma_ch_tx);
	peripheral_free_list(pdata->pin_req);

	free_netdev(dev);
	dev_set_drvdata(&spi->dev, NULL);
	return 0;
}

static struct spi_driver adf702x_spi_driver = {
	.driver = {
		.name	= "adf702x",
		.owner	= THIS_MODULE,
	},
	.probe		= adf702x_probe,
	.remove		= __devexit_p(adf702x_remove),
};

static int __init adf702x_init_module(void)
{
	return spi_register_driver(&adf702x_spi_driver);;
}

static void __exit adf702x_exit_module(void)
{
	spi_unregister_driver(&adf702x_spi_driver);
}

module_init(adf702x_init_module);
module_exit(adf702x_exit_module);
MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("ADF702XNet Wireless Ethernet driver");
MODULE_LICENSE("GPL");
