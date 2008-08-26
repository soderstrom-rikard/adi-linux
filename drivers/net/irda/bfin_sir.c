/*
 * Blackfin Infra-red Driver
 *
 * Copyright 2006-2008 Analog Devices Inc.
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Licensed under the GPL-2 or later.
 *
 */

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <net/irda/irda.h>
#include <net/irda/wrapper.h>
#include <net/irda/irda_device.h>

#include <asm/irq.h>
#include <asm/dma.h>
#include <asm/cacheflush.h>

#include <asm/mach/bfin_sir.h>

#ifdef CONFIG_SIR_BFIN_DMA
#define DMA_SIR_RX_XCNT        10
#define DMA_SIR_RX_YCNT        (PAGE_SIZE / DMA_SIR_RX_XCNT)
#define DMA_SIR_RX_FLUSH_JIFS  (HZ * 4 / 250)
#endif

static int max_rate = 115200;

static void turnaround_delay(unsigned long last_jif, int mtt)
{
	long ticks;

	mtt = mtt < 10000 ? 10000 : mtt;
	ticks = 1 + mtt / (USEC_PER_SEC / HZ);
	schedule_timeout_interruptible(ticks);
}
static void __init bfin_sir_init_ports(int i)
{
	sir_ports[i].membase   = (void __iomem *)bfin_sir_port_resource[i].base_addr;
	sir_ports[i].irq = bfin_sir_port_resource[i].irq;
	sir_ports[i].clk = get_sclk();
#ifdef CONFIG_SIR_BFIN_DMA
	sir_ports[i].tx_done        = 1;
	init_timer(&(sir_ports[i].rx_dma_timer));
#endif
	sir_ports[i].rx_dma_channel = bfin_sir_port_resource[i].rx_dma_channel;
	sir_ports[i].tx_dma_channel = bfin_sir_port_resource[i].tx_dma_channel;
}

static void bfin_sir_stop_tx(struct bfin_sir_port *port)
{
#ifndef CONFIG_BF54x
	unsigned short ier;
#endif
#ifdef CONFIG_SIR_BFIN_DMA
	disable_dma(port->tx_dma_channel);
#endif

	while (!(SIR_UART_GET_LSR(port) & THRE))
		continue;
#ifdef CONFIG_BF54x
	SIR_UART_PUT_LSR(port, TFI);
	SIR_UART_CLEAR_IER(port, ETBEI);
#else
	ier = SIR_UART_GET_IER(port);
	ier &= ~ETBEI;
	SIR_UART_PUT_IER(port, ier);
#endif
}

static void bfin_sir_enable_tx(struct bfin_sir_port *port)
{
#ifndef CONFIG_BF54x
	unsigned short ier;
#endif
#ifdef CONFIG_BF54x
	SIR_UART_SET_IER(port, ETBEI);
#else
	ier = SIR_UART_GET_IER(port);
	ier |= ETBEI;
	SIR_UART_PUT_IER(port, ier);
#endif
}

static void bfin_sir_stop_rx(struct bfin_sir_port *port)
{
#ifdef CONFIG_BF54x
	SIR_UART_CLEAR_IER(port, ERBFI);
#else
	unsigned short ier;

	ier = SIR_UART_GET_IER(port);
	ier &= ~ERBFI;
	SIR_UART_PUT_IER(port, ier);
#endif
}

static void bfin_sir_enable_rx(struct bfin_sir_port *port)
{
#ifdef CONFIG_BF54x
	SIR_UART_SET_IER(port, ERBFI);
#else
	SIR_UART_PUT_IER(port, SIR_UART_GET_IER(port) | ERBFI);
#endif
}

static int bfin_sir_set_speed(struct bfin_sir_port *port, int speed)
{
	int ret = -EINVAL;
	unsigned int quot;
	unsigned short val, lsr, lcr = 0;

	lcr = WLS(8);

	switch (speed) {
	case 9600:
	case 19200:
	case 38400:
	case 57600:
	case 115200:

		quot = (port->clk + (8 * speed)) / (16 * speed);

		do {
			lsr = SIR_UART_GET_LSR(port);
		} while (!(lsr & TEMT));

		/* Clear UCEN bit to reset the UART state machine
		 * and control registers
		 */
		val = SIR_UART_GET_GCTL(port);
		val &= ~UCEN;
		SIR_UART_PUT_GCTL(port, val);

#ifndef CONFIG_BF54x
		/* Set DLAB in LCR to Access DLL and DLH */
		val = SIR_UART_GET_LCR(port);
		val |= DLAB;
		SIR_UART_PUT_LCR(port, val);
		SSYNC();
#endif

		SIR_UART_PUT_DLL(port, quot & 0xFF);
		SSYNC();
		SIR_UART_PUT_DLH(port, (quot >> 8) & 0xFF);
		SSYNC();

#ifndef CONFIG_BF54x
		/* Clear DLAB in LCR to Access THR RBR IER */
		val = SIR_UART_GET_LCR(port);
		val &= ~DLAB;
		SIR_UART_PUT_LCR(port, val);
		SSYNC();
#endif

		SIR_UART_PUT_LCR(port, lcr);

		val = SIR_UART_GET_GCTL(port);
		val |= UCEN;
		SIR_UART_PUT_GCTL(port, val);

		ret = 0;
		/*printk(KERN_DEBUG "bfin_sir: Set new speed %d\n", speed);*/
		break;
	default:
		printk(KERN_WARNING "bfin_sir: Invalid speed %d\n", speed);
		break;
	}

	val = SIR_UART_GET_GCTL(port);
	/* If not add the 'RPOLC', we can't catch the receive interrupt.
	 * It's related with the HW layout and the IR transiver.
	 */
	val |= IREN | RPOLC;
	SIR_UART_PUT_GCTL(port, val);
	return ret;
}

static int bfin_sir_is_receiving(struct net_device *dev)
{
	struct bfin_sir_self *self = dev->priv;
	struct bfin_sir_port *port = self->sir_port;

	if (!(SIR_UART_GET_IER(port) & ERBFI))
		return 0;
	return self->rx_buff.state != OUTSIDE_FRAME;
}

#ifdef CONFIG_SIR_BFIN_PIO
static void bfin_sir_tx_chars(struct net_device *dev)
{
	unsigned int chr;
	struct bfin_sir_self *self = dev->priv;
	struct bfin_sir_port *port = self->sir_port;

	if (self->tx_buff.len != 0) {
		chr = *(self->tx_buff.data);
		SIR_UART_PUT_CHAR(port, chr);
		self->tx_buff.data++;
		self->tx_buff.len--;
	} else {
		self->stats.tx_packets++;
		self->stats.tx_bytes += self->tx_buff.data - self->tx_buff.head;
		if (self->newspeed) {
			bfin_sir_set_speed(port, self->newspeed);
			self->speed = self->newspeed;
			self->newspeed = 0;
		}
		bfin_sir_stop_tx(port);
		atomic_set(&port->enable_rx, 1);
		/* I'm hungry! */
		netif_wake_queue(dev);
	}
}

static void bfin_sir_rx_chars(struct net_device *dev)
{
	struct bfin_sir_self *self = dev->priv;
	struct bfin_sir_port *port = self->sir_port;
	unsigned char ch;

	SIR_UART_CLEAR_LSR(port);
	ch = SIR_UART_GET_CHAR(port);
	if (atomic_read(&port->enable_rx))
		async_unwrap_char(dev, &self->stats, &self->rx_buff, ch);
	else
		self->stats.collisions++;
	dev->last_rx = jiffies;
}

static irqreturn_t bfin_sir_rx_int(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct bfin_sir_self *self = dev->priv;
	struct bfin_sir_port *port = self->sir_port;
	while ((SIR_UART_GET_LSR(port) & DR))
		bfin_sir_rx_chars(dev);

	return IRQ_HANDLED;
}

static irqreturn_t bfin_sir_tx_int(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct bfin_sir_self *self = dev->priv;
	struct bfin_sir_port *port = self->sir_port;

	if (SIR_UART_GET_LSR(port) & THRE)
		bfin_sir_tx_chars(dev);
	return IRQ_HANDLED;
}
#endif /* CONFIG_SIR_BFIN_PIO */

#ifdef CONFIG_SIR_BFIN_DMA
static void bfin_sir_dma_tx_chars(struct net_device *dev)
{
	struct bfin_sir_self *self = dev->priv;
	struct bfin_sir_port *port = self->sir_port;

	if (!port->tx_done)
		return;
	port->tx_done = 0;

	if (self->tx_buff.len == 0) {
		self->stats.tx_packets++;
		if (self->newspeed) {
			bfin_sir_set_speed(port, self->newspeed);
			self->speed = self->newspeed;
			self->newspeed = 0;
		}
		atomic_set(&port->enable_rx, 1);
		port->tx_done = 1;
		netif_wake_queue(dev);
		return;
	}

	blackfin_dcache_flush_range((unsigned long)(self->tx_buff.data),
		(unsigned long)(self->tx_buff.data+self->tx_buff.len));
	set_dma_config(port->tx_dma_channel,
		set_bfin_dma_config(DIR_READ, DMA_FLOW_STOP,
			INTR_ON_BUF, DIMENSION_LINEAR,
			DATA_SIZE_8, DMA_SYNC_RESTART));
	set_dma_start_addr(port->tx_dma_channel, (unsigned long)(self->tx_buff.data));
	set_dma_x_count(port->tx_dma_channel, self->tx_buff.len);
	set_dma_x_modify(port->tx_dma_channel, 1);
	enable_dma(port->tx_dma_channel);
	bfin_sir_enable_tx(port);
}

static irqreturn_t bfin_sir_dma_tx_int(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct bfin_sir_self *self = dev->priv;
	struct bfin_sir_port *port = self->sir_port;
	spin_lock(&self->lock);
	if (!(get_dma_curr_irqstat(port->tx_dma_channel)&DMA_RUN)) {
		clear_dma_irqstat(port->tx_dma_channel);
		bfin_sir_stop_tx(port);

		self->stats.tx_packets++;
		self->stats.tx_bytes += self->tx_buff.len;
		self->tx_buff.len = 0;
		if (self->newspeed) {
			bfin_sir_set_speed(port, self->newspeed);
			self->speed = self->newspeed;
			self->newspeed = 0;
		}
		atomic_set(&port->enable_rx, 1);
		/* I'm hungry! */
		netif_wake_queue(dev);
		port->tx_done = 1;
	}
	spin_unlock(&self->lock);

	return IRQ_HANDLED;
}

static void bfin_sir_dma_rx_chars(struct net_device *dev)
{
	struct bfin_sir_self *self = dev->priv;
	struct bfin_sir_port *port = self->sir_port;
	int i, status;

	status = SIR_UART_GET_LSR(port);
	SIR_UART_CLEAR_LSR(port);

	if (status & BI) {
		self->stats.rx_errors++;
		status &= ~(PE | FE);
	}
	if (status & PE)
		self->stats.rx_errors++;
	if (status & OE)
		self->stats.rx_errors++;
	if (status & FE)
		self->stats.rx_errors++;

	for (i = port->rx_dma_buf.head; i < port->rx_dma_buf.tail; i++)
		if (atomic_read(&port->enable_rx))
			async_unwrap_char(dev, &self->stats, &self->rx_buff, port->rx_dma_buf.buf[i]);
		else
			self->stats.collisions++;
}

void bfin_sir_rx_dma_timeout(struct net_device *dev)
{
	struct bfin_sir_self *self = dev->priv;
	struct bfin_sir_port *port = self->sir_port;
	int x_pos, pos;
	int flags = 0;

	spin_lock_irqsave(&self->lock, flags);
	x_pos = DMA_SIR_RX_XCNT - get_dma_curr_xcount(port->rx_dma_channel);
	if (x_pos == DMA_SIR_RX_XCNT)
		x_pos = 0;

	pos = port->rx_dma_nrows * DMA_SIR_RX_XCNT + x_pos;

	if (pos > port->rx_dma_buf.tail) {
		port->rx_dma_buf.tail = pos;
		bfin_sir_dma_rx_chars(dev);
		port->rx_dma_buf.head = port->rx_dma_buf.tail;
	}
	spin_unlock_irqrestore(&self->lock, flags);
}

static irqreturn_t bfin_sir_dma_rx_int(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct bfin_sir_self *self = dev->priv;
	struct bfin_sir_port *port = self->sir_port;
	unsigned short irqstat;

	spin_lock(&self->lock);

	port->rx_dma_nrows++;
	port->rx_dma_buf.tail = DMA_SIR_RX_XCNT * port->rx_dma_nrows;
	bfin_sir_dma_rx_chars(dev);
	if (port->rx_dma_nrows >= DMA_SIR_RX_YCNT) {
		port->rx_dma_nrows = 0;
		port->rx_dma_buf.tail = 0;
	}
	port->rx_dma_buf.head = port->rx_dma_buf.tail;

	irqstat = get_dma_curr_irqstat(port->rx_dma_channel);
	clear_dma_irqstat(port->rx_dma_channel);
	spin_unlock(&self->lock);

	mod_timer(&(port->rx_dma_timer), jiffies + DMA_SIR_RX_FLUSH_JIFS);
	return IRQ_HANDLED;
}
#endif /* CONFIG_SIR_BFIN_DMA */

static int bfin_sir_startup(struct bfin_sir_port *port, struct net_device *dev)
{
#ifdef CONFIG_SIR_BFIN_DMA
	dma_addr_t dma_handle;
#endif /* CONFIG_SIR_BFIN_DMA */

	if (request_dma(port->rx_dma_channel, "BFIN_UART_RX") < 0) {
		printk(KERN_WARNING "bfin_sir: Unable to attach SIR RX DMA channel\n");
		return -EBUSY;
	}

	if (request_dma(port->tx_dma_channel, "BFIN_UART_TX") < 0) {
		printk(KERN_WARNING "bfin_sir: Unable to attach SIR TX DMA channel\n");
		free_dma(port->rx_dma_channel);
		return -EBUSY;
	}

#ifdef CONFIG_SIR_BFIN_DMA

	set_dma_callback(port->rx_dma_channel, bfin_sir_dma_rx_int, dev);
	set_dma_callback(port->tx_dma_channel, bfin_sir_dma_tx_int, dev);

	port->rx_dma_buf.buf = (unsigned char *)dma_alloc_coherent(NULL, PAGE_SIZE, &dma_handle, GFP_DMA);
	port->rx_dma_buf.head = 0;
	port->rx_dma_buf.tail = 0;
	port->rx_dma_nrows = 0;

	set_dma_config(port->rx_dma_channel,
				set_bfin_dma_config(DIR_WRITE, DMA_FLOW_AUTO,
									INTR_ON_ROW, DIMENSION_2D,
									DATA_SIZE_8, DMA_SYNC_RESTART));
	set_dma_x_count(port->rx_dma_channel, DMA_SIR_RX_XCNT);
	set_dma_x_modify(port->rx_dma_channel, 1);
	set_dma_y_count(port->rx_dma_channel, DMA_SIR_RX_YCNT);
	set_dma_y_modify(port->rx_dma_channel, 1);
	set_dma_start_addr(port->rx_dma_channel, (unsigned long)port->rx_dma_buf.buf);
	enable_dma(port->rx_dma_channel);

	port->rx_dma_timer.data = (unsigned long)(dev);
	port->rx_dma_timer.function = (void *)bfin_sir_rx_dma_timeout;

#else

	if (request_irq(port->irq, bfin_sir_rx_int, IRQF_DISABLED, "BFIN_SIR_RX", dev)) {
		printk(KERN_WARNING "bfin_sir: Unable to attach SIR RX interrupt\n");
		return -EBUSY;
	}

	if (request_irq(port->irq+1, bfin_sir_tx_int, IRQF_DISABLED, "BFIN_SIR_TX", dev)) {
		printk(KERN_WARNING "bfin_sir: Unable to attach SIR TX interrupt\n");
		free_irq(port->irq, dev);
		return -EBUSY;
	}
#endif

	return 0;
}

static void bfin_sir_shutdown(struct bfin_sir_port *port, struct net_device *dev)
{
	unsigned short val;

	bfin_sir_stop_rx(port);
#ifdef CONFIG_BF54x
	SIR_UART_CLEAR_IER(port, 0xF);
#else
	SIR_UART_PUT_IER(port, 0);
#endif
	val = SIR_UART_GET_GCTL(port);
	val &= ~(UCEN | IREN | RPOLC);
	SIR_UART_PUT_GCTL(port, val);

#ifdef CONFIG_SIR_BFIN_DMA
	disable_dma(port->tx_dma_channel);
	disable_dma(port->rx_dma_channel);
	del_timer(&(port->rx_dma_timer));
	dma_free_coherent(NULL, PAGE_SIZE, port->rx_dma_buf.buf, 0);
#else
	free_irq(port->irq+1, dev);
	free_irq(port->irq, dev);
#endif
	free_dma(port->tx_dma_channel);
	free_dma(port->rx_dma_channel);
}

#ifdef CONFIG_PM
static int bfin_sir_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct bfin_sir_port *psir_ports;
	struct net_device *dev;
	struct bfin_sir_self *self;
	int i;

	psir_ports = platform_get_drvdata(pdev);
	if (!psir_ports)
		return 0;
	for (i = nr_sirs-1; i >= 0; i--) {
		dev = (psir_ports+i)->dev;
		self = dev->priv;
		if (self->open) {
			flush_scheduled_work();
			bfin_sir_shutdown(self->sir_port, dev);
			netif_device_detach(dev);
		}
	}

	return 0;
}
static int bfin_sir_resume(struct platform_device *pdev)
{
	struct bfin_sir_port *psir_ports;
	struct net_device *dev;
	struct bfin_sir_self *self;
	struct bfin_sir_port *port;
	int i;

	psir_ports = platform_get_drvdata(pdev);
	if (!psir_ports) return 0;
	for (i = nr_sirs-1; i >= 0; i--) {
		dev = (psir_ports+i)->dev;
		self = dev->priv;
		port = self->sir_port;
		if (self->open) {
			if (self->newspeed) {
				self->speed = self->newspeed;
				self->newspeed = 0;
			}
			bfin_sir_startup(port, dev);
			bfin_sir_set_speed(port, 9600);
			atomic_set(&port->enable_rx, 1);
			bfin_sir_enable_rx(port);
			netif_device_attach(dev);
		}
	}
	return 0;
}
#else
#define bfin_sir_suspend   NULL
#define bfin_sir_resume    NULL
#endif

static void bfin_sir_send_work(struct work_struct *work)
{
	struct bfin_sir_self  *self = container_of(work, struct bfin_sir_self, work);
	struct net_device *dev = self->sir_port->dev;
	struct bfin_sir_port *port = self->sir_port;
	int tx_cnt = 10;

	while (bfin_sir_is_receiving(dev) && --tx_cnt)
		turnaround_delay(dev->last_rx, self->mtt);

	bfin_sir_set_speed(port, self->speed);

	atomic_set(&port->enable_rx, 0);

#ifdef CONFIG_SIR_BFIN_DMA
	bfin_sir_dma_tx_chars(dev);
#else
	bfin_sir_enable_tx(port);
#endif
	dev->trans_start = jiffies;
}

static int bfin_sir_hard_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct bfin_sir_self *self = dev->priv;
	int speed = irda_get_next_speed(skb);

	netif_stop_queue(dev);

	self->mtt = irda_get_mtt(skb);

	if (speed != self->speed && speed != -1)
		self->newspeed = speed;

	self->tx_buff.data = self->tx_buff.head;
	if (skb->len == 0)
		self->tx_buff.len = 0;
	else
		self->tx_buff.len = async_wrap_skb(skb, self->tx_buff.data, self->tx_buff.truesize);

	schedule_work(&self->work);
	dev_kfree_skb(skb);

	return 0;
}

static int bfin_sir_ioctl(struct net_device *dev, struct ifreq *ifreq, int cmd)
{
	struct if_irda_req *rq = (struct if_irda_req *)ifreq;
	struct bfin_sir_self *self = dev->priv;
	struct bfin_sir_port *port = self->sir_port;
	int ret = 0;

	switch (cmd) {
	case SIOCSBANDWIDTH:
		if (capable(CAP_NET_ADMIN)) {
			if (self->open) {
				ret = bfin_sir_set_speed(port, rq->ifr_baudrate);
				bfin_sir_enable_rx(port);
			} else {
				printk(KERN_WARNING "bfin_sir: SIOCSBANDWIDTH: !netif_running\n");
				ret = 0;
			}
		}
		break;

	case SIOCSMEDIABUSY:
		ret = -EPERM;
		if (capable(CAP_NET_ADMIN)) {
			irda_device_set_media_busy(dev, TRUE);
			ret = 0;
		}
		break;

	case SIOCGRECEIVING:
		rq->ifr_receiving = bfin_sir_is_receiving(dev);
		break;

	default:
		ret = -EOPNOTSUPP;
		break;
	}

	return ret;
}

static struct net_device_stats *bfin_sir_stats(struct net_device *dev)
{
	struct bfin_sir_self *self = dev->priv;
	return &self->stats;
}

static int bfin_sir_open(struct net_device *dev)
{
	struct bfin_sir_self *self = dev->priv;
	struct bfin_sir_port *port = self->sir_port;
	int err = -ENOMEM;

	self->newspeed = 0;
	self->speed = 9600;

	spin_lock_init(&self->lock);

	err = bfin_sir_startup(port, dev);
	if (err)
		goto err_startup;

	bfin_sir_set_speed(port, 9600);

	self->irlap = irlap_open(dev, &self->qos, DRIVER_NAME);
	if (!self->irlap)
		goto err_irlap;

	INIT_WORK(&self->work, bfin_sir_send_work);

	/*
	 * Now enable the interrupt then start the queue
	 */
	self->open = 1;
	atomic_set(&port->enable_rx, 1);
	bfin_sir_enable_rx(port);

	netif_start_queue(dev);

	return 0;

err_irlap:
	self->open = 0;
	bfin_sir_shutdown(port, dev);
err_startup:
	return err;
}

static int bfin_sir_stop(struct net_device *dev)
{
	struct bfin_sir_self *self = dev->priv;

	flush_scheduled_work();
	bfin_sir_shutdown(self->sir_port, dev);

	if (self->rxskb) {
		dev_kfree_skb(self->rxskb);
		self->rxskb = NULL;
	}

	/* Stop IrLAP */
	if (self->irlap) {
		irlap_close(self->irlap);
		self->irlap = NULL;
	}

	netif_stop_queue(dev);
	self->open = 0;

	return 0;
}

static int bfin_sir_init_iobuf(iobuff_t *io, int size)
{
	io->head = kmalloc(size, GFP_KERNEL | GFP_DMA);
	if (io->head != NULL) {
		io->truesize = size;
		io->in_frame = FALSE;
		io->state    = OUTSIDE_FRAME;
		io->data     = io->head;
	}
	return io->head ? 0 : -ENOMEM;
}

static int __devinit bfin_sir_probe(struct platform_device *pdev)
{
	struct net_device *dev;
	struct bfin_sir_self *self;
	unsigned int baudrate_mask;
	int i, err = 0;

	err = bfin_sir_hw_init();

	for (i = 0; i < nr_sirs; i++) {
		bfin_sir_init_ports(i);
		dev = alloc_irdadev(sizeof(struct bfin_sir_self));
		if (!dev)
			goto err_mem_1;

		self = dev->priv;
		self->dev = &pdev->dev;
		self->sir_port = &sir_ports[i];
		sir_ports[i].dev = dev;

		err = bfin_sir_init_iobuf(&self->rx_buff, IRDA_SKB_MAX_MTU);
		if (err)
			goto err_mem_2;
		err = bfin_sir_init_iobuf(&self->tx_buff, IRDA_SIR_MAX_FRAME);
		if (err)
			goto err_mem_3;

		dev->hard_start_xmit = bfin_sir_hard_xmit;
		dev->open            = bfin_sir_open;
		dev->stop            = bfin_sir_stop;
		dev->do_ioctl        = bfin_sir_ioctl;
		dev->get_stats       = bfin_sir_stats;
		dev->irq             = sir_ports[i].irq;

		irda_init_max_qos_capabilies(&self->qos);

		baudrate_mask = IR_9600;

		switch (max_rate) {
		case 115200:		baudrate_mask |= IR_115200;
		case 57600:		baudrate_mask |= IR_57600;
		case 38400:		baudrate_mask |= IR_38400;
		case 19200:		baudrate_mask |= IR_19200;
		}

		self->qos.baud_rate.bits &= baudrate_mask;

		self->qos.min_turn_time.bits = 1; /* 10 ms or more */

		irda_qos_bits_to_value(&self->qos);

		err = register_netdev(dev);

		if (err) {
err_mem_3:
			kfree(self->tx_buff.head);
err_mem_2:
			kfree(self->rx_buff.head);
err_mem_1:
			free_netdev(dev);
		}
	}
	if (err == 0)
		platform_set_drvdata(pdev, sir_ports);
	return err;
}

static int __devexit bfin_sir_remove(struct platform_device *pdev)
{
	int i;
	struct bfin_sir_port *psir_ports;
	struct net_device *dev = NULL;
	struct bfin_sir_self *self;


	psir_ports = platform_get_drvdata(pdev);
	if (!psir_ports)
		return 0;
	for (i = nr_sirs-1; i >= 0; i--) {
		dev = (psir_ports+i)->dev;
		self = dev->priv;
		unregister_netdev(dev);
		kfree(self->tx_buff.head);
		kfree(self->rx_buff.head);
		free_netdev(dev);
	}
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver bfin_ir_driver = {
	.probe   = bfin_sir_probe,
	.remove  = __devexit_p(bfin_sir_remove),
	.suspend = bfin_sir_suspend,
	.resume  = bfin_sir_resume,
	.driver  = {
		.name = DRIVER_NAME,
	},
};

static int __init bfin_sir_init(void)
{
	return platform_driver_register(&bfin_ir_driver);
}

static void __exit bfin_sir_exit(void)
{
	platform_driver_unregister(&bfin_ir_driver);
}

module_init(bfin_sir_init);
module_exit(bfin_sir_exit);

module_param(max_rate, int, 0);
MODULE_PARM_DESC(max_rate, "Maximum baud rate (115200, 57600, 38400, 19200, 9600)");

MODULE_AUTHOR("Graf.Yang <graf.yang@analog.com>");
MODULE_DESCRIPTION("Blackfin IrDA driver");
MODULE_LICENSE("GPL");
