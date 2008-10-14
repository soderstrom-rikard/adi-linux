/*
 * Copyright (C) 2005-2006 by Texas Instruments
 *
 * This file is part of the Inventra Controller Driver for Linux.
 *
 * The Inventra Controller Driver for Linux is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License version 2 as published by the Free Software
 * Foundation.
 *
 * The Inventra Controller Driver for Linux is distributed in
 * the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with The Inventra Controller Driver for Linux ; if not,
 * write to the Free Software Foundation, Inc., 59 Temple Place,
 * Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/clk.h>

#include <asm/io.h>
#include <asm/cacheflush.h>
#include <asm/gpio.h>

#include "musb_core.h"
#include "blackfin.h"

/* FIXME: Add this option to the platfrom device file */

#ifdef CONFIG_BFIN548_EZKIT
#define GPIO_USB_VRSEL	GPIO_PE7
#elif defined(CONFIG_BFIN527_EZKIT) || defined(CONFIG_BFIN526_EZBRD)
#define GPIO_USB_VRSEL	GPIO_PG13
#elif defined(CONFIG_BFIN548_BLUETECHNIX_CM)
#define GPIO_USB_VRSEL	GPIO_PH6
#elif defined(CONFIG_BFIN527_BLUETECHNIX_CM)
#define GPIO_USB_VRSEL	GPIO_PF11
#else
#error You need to specify a GPIO controlling VRSEL
#endif

static struct otg_transceiver *xceiv;

/**
 * otg_get_transceiver - find the (single) OTG transceiver driver
 *
 * Returns the transceiver driver, after getting a refcount to it; or
 * null if there is no such transceiver.  The caller is responsible for
 * releasing that count.
 */
struct otg_transceiver *otg_get_transceiver(void)
{
	if (xceiv)
		get_device(xceiv->dev);
	return xceiv;
}
EXPORT_SYMBOL(otg_get_transceiver);

int otg_set_transceiver(struct otg_transceiver *x)
{
	if (xceiv && x)
		return -EBUSY;
	xceiv = x;
	return 0;
}
EXPORT_SYMBOL(otg_set_transceiver);

#undef DUMP_FIFO_DATA
#ifdef DUMP_FIFO_DATA
static void dump_fifo_data(u8 *buf, u16 len)
{
	u8 *tmp = buf;
	int i;

	for (i = 0; i < len; i++) {
		if (!(i % 16) && i)
			printk("\n");
		printk("%02x ", *tmp++);
	}
	printk("\n");
}
#else
#define dump_fifo_data(buf, len)	do{} while (0)
#endif

#ifdef CONFIG_BF52x

#define USB_DMA_BASE		USB_DMA_INTERRUPT
#define USB_DMAx_CTRL		0x04
#define USB_DMAx_ADDR_LOW	0x08
#define USB_DMAx_ADDR_HIGH	0x0C
#define USB_DMAx_COUNT_LOW	0x10
#define USB_DMAx_COUNT_HIGH	0x14

#define USB_DMA_REG(ep, reg)	(USB_DMA_BASE + 0x20*ep + reg)
#endif

/*
 * Load an endpoint's FIFO
 */
void musb_write_fifo(struct musb_hw_ep *hw_ep, u16 len, const u8 *src)
{
	void __iomem *fifo = hw_ep->fifo;
	void __iomem *epio = hw_ep->regs;

	prefetch((u8 *)src);

	musb_writew(epio, MUSB_FIFOSIZE, len);

	DBG(4, "TX ep%d fifo %p count %d buf %p, epio %p\n",
			hw_ep->epnum, fifo, len, src, epio);

	dump_fifo_data(src, len);

	if (unlikely((unsigned long)src & 0x01))
		outsw_8((unsigned long)fifo, src,
			len & 0x01 ? (len >> 1) + 1 : len >> 1);
	else
		outsw((unsigned long)fifo, src,
			len & 0x01 ? (len >> 1) + 1 : len >> 1);
}

/*
 * Unload an endpoint's FIFO
 */
void musb_read_fifo(struct musb_hw_ep *hw_ep, u16 len, u8 *dst)
{
	void __iomem *fifo = hw_ep->fifo;
	u8 epnum = hw_ep->epnum;
	u16 dma_reg = 0;

	DBG(4, "%cX ep%d fifo %p count %d buf %p\n",
			'R', hw_ep->epnum, fifo, len, dst);

#if defined(CONFIG_BF52x)
	invalidate_dcache_range((unsigned int)dst,
		(unsigned int)(dst + len));

	/* Setup DMA address register */
	dma_reg = (u16) ((u32) dst & 0xFFFF);
	bfin_write16(USB_DMA_REG(epnum, USB_DMAx_ADDR_LOW), dma_reg);
	SSYNC();

	dma_reg = (u16) (((u32) dst >> 16) & 0xFFFF);
	bfin_write16(USB_DMA_REG(epnum, USB_DMAx_ADDR_HIGH), dma_reg);
	SSYNC();

	/* Setup DMA count register */
	bfin_write16(USB_DMA_REG(epnum, USB_DMAx_COUNT_LOW), len);
	bfin_write16(USB_DMA_REG(epnum, USB_DMAx_COUNT_HIGH), 0);
	SSYNC();

	/* Enable the DMA */
	dma_reg = (epnum << 4) | DMA_ENA | INT_ENA;
	bfin_write16(USB_DMA_REG(epnum, USB_DMAx_CTRL), dma_reg);
	SSYNC();

	/* Wait for compelete */
	while(!(bfin_read_USB_DMA_INTERRUPT() & (1 << epnum)))
		cpu_relax();

	/* acknowledge dma interrupt */
	bfin_write_USB_DMA_INTERRUPT(1 << epnum);
	SSYNC();

	/* Reset DMA */
	bfin_write16(USB_DMA_REG(epnum, USB_DMAx_CTRL), 0);
	SSYNC();
#else
	if (unlikely((unsigned long)dst & 0x01))
		insw_8((unsigned long)fifo, dst,
			len & 0x01 ? (len >> 1) + 1 : len >> 1);
	else
		insw((unsigned long)fifo, dst,
			len & 0x01 ? (len >> 1) + 1 : len >> 1);
#endif

	dump_fifo_data(dst, len);
}

static irqreturn_t blackfin_interrupt(int irq, void *__hci)
{
	unsigned long	flags;
	irqreturn_t	retval = IRQ_NONE;
	struct musb	*musb = __hci;

	spin_lock_irqsave(&musb->lock, flags);

	musb->int_usb = musb_readb(musb->mregs, MUSB_INTRUSB);
	musb->int_tx = musb_readw(musb->mregs, MUSB_INTRTX);
	musb->int_rx = musb_readw(musb->mregs, MUSB_INTRRX);

	if (musb->int_usb || musb->int_tx || musb->int_rx) {
		musb_writeb(musb->mregs, MUSB_INTRUSB, musb->int_usb);
		musb_writew(musb->mregs, MUSB_INTRTX, musb->int_tx);
		musb_writew(musb->mregs, MUSB_INTRRX, musb->int_rx);
		retval = musb_interrupt(musb);
	}

	spin_unlock_irqrestore(&musb->lock, flags);

	/* REVISIT we sometimes get spurious IRQs on g_ep0
	 * not clear why... fall in BF54x too.
	 */
	if (retval != IRQ_HANDLED)
		DBG(5, "spurious?\n");

	return IRQ_HANDLED;
}

/* Almost 1 second */
#define TIMER_DELAY	(1 * HZ)

static struct timer_list conn_timer;

static void conn_timer_handler(unsigned long _musb)
{
	struct musb *musb = (void *)_musb;
	unsigned long flags;
	u16 val;

	spin_lock_irqsave(&musb->lock, flags);
	switch (musb->xceiv.state) {
	case OTG_STATE_A_IDLE:
	case OTG_STATE_A_WAIT_BCON:
		/* Start a new session */
		val = musb_readw(musb->mregs, MUSB_DEVCTL);
		val |= MUSB_DEVCTL_SESSION;
		musb_writew(musb->mregs, MUSB_DEVCTL, val);

		val = musb_readw(musb->mregs, MUSB_DEVCTL);
		if (!(val & MUSB_DEVCTL_BDEVICE)) {
			gpio_set_value(GPIO_USB_VRSEL, 1);
			musb->xceiv.state = OTG_STATE_A_WAIT_BCON;
		} else {
			gpio_set_value(GPIO_USB_VRSEL, 0);

			/* Ignore VBUSERROR and SUSPEND IRQ */
			val = musb_readb(musb->mregs, MUSB_INTRUSBE);
			val &= ~MUSB_INTR_VBUSERROR;
			musb_writeb(musb->mregs, MUSB_INTRUSBE, val);

			val = MUSB_INTR_SUSPEND | MUSB_INTR_VBUSERROR;
			musb_writeb(musb->mregs, MUSB_INTRUSB, val);

			val = MUSB_POWER_HSENAB;
			musb_writeb(musb->mregs, MUSB_POWER, val);
		}
		mod_timer(&conn_timer, jiffies + TIMER_DELAY);
		break;

	default:
		DBG(1, "%s state not handled\n", otg_state_string(musb));
		break;
	}
	spin_unlock_irqrestore(&musb->lock, flags);

	DBG(4, "state is %s\n", otg_state_string(musb));

	return;
}

void musb_platform_enable(struct musb *musb)
{
	if (is_host_enabled(musb)) {
		mod_timer(&conn_timer, jiffies + TIMER_DELAY);
		musb->a_wait_bcon = TIMER_DELAY;
	}
}

void musb_platform_disable(struct musb *musb)
{
}

static void bfin_vbus_power(struct musb *musb, int is_on, int sleeping)
{
}

static void bfin_set_vbus(struct musb *musb, int is_on)
{
	if (is_on)
		gpio_set_value(GPIO_USB_VRSEL, 1);
	else
		gpio_set_value(GPIO_USB_VRSEL, 0);

	DBG(1, "VBUS %s, devctl %02x "
		/* otg %3x conf %08x prcm %08x */ "\n",
		otg_state_string(musb),
		musb_readb(musb->mregs, MUSB_DEVCTL));
}

static int bfin_set_power(struct otg_transceiver *x, unsigned mA)
{
	return 0;
}

void musb_platform_try_idle(struct musb *musb, unsigned long timeout)
{
	if (is_host_enabled(musb))
		mod_timer(&conn_timer, jiffies + TIMER_DELAY);
}

int musb_platform_get_vbus_status(struct musb *musb)
{
	return 0;
}

void musb_platform_set_mode(struct musb *musb, u8 musb_mode)
{
}

int musb_platform_resume(struct musb *musb);

int __init musb_platform_init(struct musb *musb)
{

	/*
	 * Rev 1.0 BF549 EZ-KITs require PE7 to be high for both DEVICE
	 * and OTG HOST modes, while rev 1.1 and greater require PE7 to
	 * be low for DEVICE mode and high for HOST mode. We set it high
	 * here because we are in host mode
	 */

	if (gpio_request(GPIO_USB_VRSEL, "USB_VRSEL")) {
		printk(KERN_ERR "Failed ro request USB_VRSEL GPIO_%d \n",
			GPIO_USB_VRSEL);
		return -ENODEV;
	}
	gpio_direction_output(GPIO_USB_VRSEL, 0);

	if (ANOMALY_05000346) {
		bfin_write_USB_APHY_CALIB(ANOMALY_05000346_value);
		SSYNC();
	}

	if (ANOMALY_05000347) {
		bfin_write_USB_APHY_CNTRL(0x0);
		SSYNC();
	}

	/* TODO
	 * Set SIC-IVG register
	 */

	/* Configure PLL oscillator register */
	bfin_write_USB_PLLOSC_CTRL(0x30a8);
	SSYNC();

	bfin_write_USB_SRP_CLKDIV((get_sclk()/1000) / 32 - 1);
	SSYNC();

	bfin_write_USB_EP_NI0_RXMAXP(64);
	SSYNC();

	bfin_write_USB_EP_NI0_TXMAXP(64);
	SSYNC();

	/* Route INTRUSB/INTR_RX/INTR_TX to USB_INT0*/
	bfin_write_USB_GLOBINTR(0x7);
	SSYNC();

	bfin_write_USB_GLOBAL_CTL(GLOBAL_ENA | EP1_TX_ENA | EP2_TX_ENA |
				EP3_TX_ENA | EP4_TX_ENA | EP5_TX_ENA |
				EP6_TX_ENA | EP7_TX_ENA | EP1_RX_ENA |
				EP2_RX_ENA | EP3_RX_ENA | EP4_RX_ENA |
				EP5_RX_ENA | EP6_RX_ENA | EP7_RX_ENA);
	SSYNC();

	if (is_host_enabled(musb)) {
		musb->board_set_vbus = bfin_set_vbus;
		setup_timer(&conn_timer,
			conn_timer_handler, (unsigned long) musb);
	}
	if (is_peripheral_enabled(musb))
		musb->xceiv.set_power = bfin_set_power;

	musb->isr = blackfin_interrupt;

	return 0;
}

int musb_platform_suspend(struct musb *musb)
{
	return 0;
}

int musb_platform_resume(struct musb *musb)
{
	return 0;
}


int musb_platform_exit(struct musb *musb)
{

	bfin_vbus_power(musb, 0 /*off*/, 1);
	gpio_free(GPIO_USB_VRSEL);
	musb_platform_suspend(musb);

	return 0;
}
