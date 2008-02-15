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
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/slab.h>
#include <linux/rtnetlink.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

#include <net/irda/irda.h>
#include <net/irda/wrapper.h>
#include <net/irda/irda_device.h>

#include <asm/irq.h>
#include <asm/dma.h>

#include <asm/mach/bfin_sir.h>

static int max_rate = 115200;

static void __init bfin_sir_init_ports(int i)
{
	sir_ports[i].membase   = (void __iomem *)bfin_sir_port_resource[i].base_addr;
	sir_ports[i].irq = bfin_sir_port_resource[i].irq;
	sir_ports[i].clk = get_sclk();
}

static void bfin_sir_stop_tx(struct bfin_sir_port *port)
{
	unsigned short ier;

	while (!(SIR_UART_GET_LSR(port) & TEMT))
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

static void bfin_sir_enable_tx(struct bfin_sir_port *port)
{
	unsigned short ier;
#ifdef CONFIG_BF54x
	SIR_UART_SET_IER(port, ETBEI);
#else
	ier = SIR_UART_GET_IER(port);
	ier |= ETBEI;
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
	unsigned long flags;
	int ret = -EINVAL;
	unsigned int quot;
	unsigned short val, ier, lsr, lcr = 0;

	lcr = WLS(8);
	switch (speed) {
	case 9600:    case 19200:    case 38400:
	case 57600:    case 115200:

		quot = (port->clk + (8 * speed)) / (16 * speed);

		local_irq_save(flags);
		do {
			lsr = SIR_UART_GET_LSR(port);
		} while (!(lsr & TEMT));

		/* Disable UART */
		ier = SIR_UART_GET_IER(port);
#ifdef CONFIG_BF54x
		SIR_UART_CLEAR_IER(port, 0xF);
#else
		SIR_UART_PUT_IER(port, 0);
#endif

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

		/* Enable UART */
#ifdef CONFIG_BF54x
		SIR_UART_SET_IER(port, ier);
#else
		SIR_UART_PUT_IER(port, ier);
#endif

		val = SIR_UART_GET_GCTL(port);
		val |= UCEN;
		SIR_UART_PUT_GCTL(port, val);

		local_irq_restore(flags);
		ret = 0;
		break;
	default:
		break;
	}

	return ret;
}

static void bfin_sir_tx_chars(struct net_device *dev)
{
	unsigned int chr;
	struct bfin_sir_self *self = dev->priv;
	struct bfin_sir_port *port = self->sir_port;

	if ((SIR_UART_GET_LSR(port) & THRE) && self->tx_buff.len != 0) {
		chr = *(self->tx_buff.data);
		SIR_UART_PUT_CHAR(port, chr);
		self->tx_buff.data++;
		self->tx_buff.len--;
	}

	if (self->tx_buff.len == 0) {
		self->stats.tx_packets++;
		self->stats.tx_bytes += self->tx_buff.data - self->tx_buff.head;
		if (self->newspeed) {
			bfin_sir_set_speed(port, self->newspeed);
			self->newspeed = 0;
		}
		bfin_sir_stop_tx(port);
		bfin_sir_enable_rx(port);
		/* I'm hungry! */
		netif_wake_queue(dev);
	}
}

static void bfin_sir_rx_chars(struct net_device *dev)
{
	struct bfin_sir_self *self = dev->priv;
	struct bfin_sir_port *port = self->sir_port;
	unsigned int status;
	unsigned char ch;

	status = SIR_UART_GET_LSR(port);
	SIR_UART_CLEAR_LSR(port);
	ch = SIR_UART_GET_CHAR(port);
	async_unwrap_char(dev, &self->stats, &self->rx_buff, ch);
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

static int bfin_sir_startup(struct bfin_sir_port *port, struct net_device *dev)
{
	unsigned short val;

	if (request_irq(port->irq, bfin_sir_rx_int, IRQF_DISABLED, "BFIN_SIR_RX", dev)) {
		printk(KERN_NOTICE "Unable to attach BlackFin SIR RX interrupt\n");
		free_irq(port->irq+1, dev);
		return -EBUSY;
	}


	if (request_irq(port->irq+1, bfin_sir_tx_int, IRQF_DISABLED, "BFIN_SIR_TX", dev)) {
		printk(KERN_NOTICE "Unable to attach BlackFin SIR TX interrupt\n");
		free_irq(port->irq, dev);
		return -EBUSY;
	}

	val = SIR_UART_GET_GCTL(port);
	/* val |= IREN; */ /* Can't catch receive interrupt */
	val |= IREN | RPOLC;
	SIR_UART_PUT_GCTL(port, val);

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

	free_irq(port->irq+1, dev);
	free_irq(port->irq, dev);
	return;
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

static int bfin_sir_hard_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct bfin_sir_self *self = dev->priv;
	struct bfin_sir_port *port = self->sir_port;
	int speed = irda_get_next_speed(skb);

	if (speed != self->speed && speed != -1)
		self->newspeed = speed;

	if (skb->len == 0) {
		if (self->newspeed) {
			self->newspeed = 0;
			bfin_sir_set_speed(port, speed);
			bfin_sir_enable_rx(port);
			dev->trans_start = jiffies;
			printk(KERN_WARNING "Empty skb, new speed\n");
		}
		dev_kfree_skb(skb);
		return 0;
	}

	if (self->speed >= 4000000) printk(KERN_WARNING "FIR\n");

	netif_stop_queue(dev);

	self->tx_buff.data = self->tx_buff.head;
	self->tx_buff.len  = async_wrap_skb(skb, self->tx_buff.data, self->tx_buff.truesize);

	/* bfin_sir_stop_rx(port); */
	bfin_sir_enable_tx(port);
	dev->trans_start = jiffies;
	dev_kfree_skb(skb);

	dev->trans_start = jiffies;

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
				printk(KERN_WARNING "bfin_sir_ioctl: SIOCSBANDWIDTH: !netif_running\n");
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
		(rq->ifr_receiving) = \
		(self->rx_buff.state == INSIDE_FRAME) || (self->rx_buff.state == BEGIN_FRAME);
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

	self->speed = 9600;

	spin_lock_init(&self->lock);

	err = bfin_sir_startup(port, dev);
	if (err)
		goto err_startup;

	bfin_sir_set_speed(port, 9600);

	self->irlap = irlap_open(dev, &self->qos, "bfin_5xx");
	if (!self->irlap)
		goto err_irlap;

	/*
	 * Now enable the interrupt then start the queue
	 */
	self->open = 1;
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

static int bfin_sir_probe(struct platform_device *pdev)
{
	struct net_device *dev;
	struct bfin_sir_self *self;
	unsigned int baudrate_mask;
	int i, err = 0;

	for (i = 0; i < nr_sirs; i++) {
		bfin_sir_init_ports(i);
		dev = alloc_irdadev(sizeof(struct bfin_sir_self));
		if (!dev)
			goto err_mem_1;

		self = dev->priv;
		self->dev = &pdev->dev; /* graf, pdev->dev is a 'struct device' */
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
		case 115200:   baudrate_mask |= IR_115200;
		case 57600:    baudrate_mask |= IR_57600;
		case 38400:    baudrate_mask |= IR_38400;
		case 19200:    baudrate_mask |= IR_19200;
		}

		self->qos.baud_rate.bits &= baudrate_mask;
		self->qos.min_turn_time.bits = 7; /* 1 ms or more */

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

static int bfin_sir_remove(struct platform_device *pdev)
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
	.remove  = bfin_sir_remove,
	.suspend = bfin_sir_suspend,
	.resume  = bfin_sir_resume,
	.driver  = {
		.name = "bfin_sir",
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

MODULE_AUTHOR("Graf.Yang <graf.yang@analog.com>");
MODULE_DESCRIPTION("Blackfin IrDA driver");
MODULE_LICENSE("GPL");
MODULE_PARM_DESC(max_rate, "Maximum baud rate (115200, 57600, 38400, 19200, 9600)");
