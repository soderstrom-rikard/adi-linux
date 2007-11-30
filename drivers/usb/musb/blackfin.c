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
#elif CONFIG_BFIN527_EZKIT
#define GPIO_USB_VRSEL	GPIO_PG13
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

#if defined(CONFIG_BF54x)

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
	int i;
	u16 *data;
	u8 epnum = hw_ep->epnum;
	u16 dma_reg = 0;

	prefetch((u8 *)src);

	bfin_write_USB_TXCOUNT(len);
	SSYNC();

	DBG(4, "TX ep%d fifo %p count %d buf %p\n",
			hw_ep->epnum, fifo, len, src);

	dump_fifo_data(src, len);

#if defined(CONFIG_BF54x)
	flush_dcache_range((unsigned int)src,
		(unsigned int)(src + len));

	/* Setup DMA address register */
	dma_reg = (u16) ((u32) src & 0xFFFF);
	bfin_write16(USB_DMA_REG(epnum, USB_DMAx_ADDR_LOW), dma_reg);
	SSYNC();

	dma_reg = (u16) (((u32) src >> 16) & 0xFFFF);
	bfin_write16(USB_DMA_REG(epnum, USB_DMAx_ADDR_HIGH), dma_reg);
	SSYNC();

	/* Setup DMA count register */
	bfin_write16(USB_DMA_REG(epnum, USB_DMAx_COUNT_LOW), len);
	bfin_write16(USB_DMA_REG(epnum, USB_DMAx_COUNT_HIGH), 0);
	SSYNC();

	/* Enable the DMA */
	dma_reg = (epnum << 4) | DMA_ENA | INT_ENA | DIRECTION;
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
#if (0)
	/* we can't assume unaligned reads work */
	if (likely((0x01 & (u16) src) == 0)) {
		data = (u16 *) src;

		if (len & 0x01) {
			u8 last = 0;
			for (i = 0; i < ((len - 1) >> 1); i++) {
				bfin_write16(fifo, *data);
				data++;
			}
			last = *(src + len - 2);
			bfin_write16(fifo, last);
		}
		else {
			for (i = 0; i < (len >> 1); i++) {
				bfin_write16(fifo, *data);
				data++;
			}
		}
	} else  {
		u16 first = *src;
		data = (u16 *) (src + 1);
		bfin_write16(fifo, first);

		for (i = 0; i < ((len - 1) >> 1); i++) {
			bfin_write16(fifo, *data);
			data++;
		}
	}
	SSYNC();
#else
	BUG_ON((unsigned long)src & 0x01);
	outsw(fifo, src, len & 0x01 ? (len >> 1) + 1 : len >> 1);
#endif

#endif
}

/*
 * Unload an endpoint's FIFO
 */
void musb_read_fifo(struct musb_hw_ep *hw_ep, u16 len, u8 *dst)
{
	void __iomem *fifo = hw_ep->fifo;
	u8 epnum = hw_ep->epnum;
	u16 dma_reg = 0;
	int i;
	u16 *data;

	DBG(4, "%cX ep%d fifo %p count %d buf %p\n",
			'R', hw_ep->epnum, fifo, len, dst);

#if defined(CONFIG_BF54x)
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
	/* we can't assume unaligned writes work */
#if (0)
	if (likely((0x01 & (unsigned long) dst) == 0)) {
		data = (u16 *) dst;

		if (len & 0x01) {
			for (i = 0; i < ((len - 1) >> 1); i++) {
				*data = bfin_read16(fifo);
				data++;
			}
			*((u8 *)data + 1) = (u8) bfin_read16(fifo);
		}
		else {
			for (i = 0; i < (len >> 1); i++)
				*data++ = bfin_read16(fifo);
		}
	} else  {
		/* byte aligned */
		BUG();
		readsb(fifo, dst, len);
	}
	SSYNC();
#else
	BUG_ON((unsigned long)dst & 0x01);
	insw(fifo, dst, len & 0x01 ? (len >> 1) + 1 : len >> 1);
#endif

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

void musb_platform_enable(struct musb *musb)
{
}

void musb_platform_disable(struct musb *musb)
{
}

static void bfin_vbus_power(struct musb *musb, int is_on, int sleeping)
{
}

static void bfin_set_vbus(struct musb *musb, int is_on)
{
	u8 devctl;
	/* HDRC controls CPEN, but beware current surges during device
	 * connect.  They can trigger transient overcurrent conditions
	 * that must be ignored.
	 */

	devctl = musb_readb(musb->mregs, MUSB_DEVCTL);

	if (is_on) {
		musb->is_active = 1;
		musb->xceiv.default_a = 1;
		musb->xceiv.state = OTG_STATE_A_WAIT_VRISE;
		devctl |= MUSB_DEVCTL_SESSION;

		MUSB_HST_MODE(musb);
	} else {
		musb->is_active = 0;

		/* NOTE:  we're skipping A_WAIT_VFALL -> A_IDLE and
		 * jumping right to B_IDLE...
		 */

		musb->xceiv.default_a = 0;
		musb->xceiv.state = OTG_STATE_B_IDLE;
		devctl &= ~MUSB_DEVCTL_SESSION;

		MUSB_DEV_MODE(musb);
	}
	musb_writeb(musb->mregs, MUSB_DEVCTL, devctl);

	DBG(1, "VBUS %s, devctl %02x "
		/* otg %3x conf %08x prcm %08x */ "\n",
		otg_state_string(musb),
		musb_readb(musb->mregs, MUSB_DEVCTL));
}

static int bfin_set_power(struct otg_transceiver *x, unsigned mA)
{
	return 0;
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
		return;
	}
	gpio_direction_output(GPIO_USB_VRSEL);


#ifdef CONFIG_USB_MUSB_PERIPHERAL
	gpio_set_value(GPIO_USB_VRSEL, 0);
#else
	gpio_set_value(GPIO_USB_VRSEL, 1);
#endif

	/* Anomaly #05000346 */
	bfin_write_USB_APHY_CALIB(0x5411);
	SSYNC();

	/* Anomaly #05000347 */
	bfin_write_USB_APHY_CNTRL(0x0);
	SSYNC();

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

	if (is_host_enabled(musb))
		musb->board_set_vbus = bfin_set_vbus;
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
