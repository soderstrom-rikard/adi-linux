/*
 * File:	drivers/usb/host/bf54x-hcd.c
 * Based on:
 * Author:
 *
 * Created:
 * Description:  BF54x HCD (Host Controller Driver) for USB.
 *
 * Modified:
 *               Copyright 2004-2006 Analog Devices Inc.
 *
 * Bugs:         Enter bugs at http://blackfin.uclinux.org/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/usb.h>
#include <linux/platform_device.h>
#include <asm/blackfin.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/system.h>
#include <asm/byteorder.h>
#include <asm/unaligned.h>

#include "../core/hcd.h"
#include "bf54x.h"


MODULE_DESCRIPTION("BF54x USB Host Controller Driver");
MODULE_LICENSE("GPL");

#define DRIVER_VERSION	"13 June, 2007"

static const char hcd_name[] = "bf54x-hcd";

static irqreturn_t bf54x_hcd_irq(struct usb_hcd *hcd)
{
	struct bf54x_hcd *bf54x_hcd = hcd_to_bf54x(hcd);
	u8		irqstat;
	irqreturn_t	ret = IRQ_NONE;

	return ret;
}

static int bf54x_hcd_start(struct usb_hcd *hcd)
{
	return 0;
}

static void bf54x_hcd_stop(struct usb_hcd *hcd)
{

}

static void bf54x_tx_zero_buffer(void)
{
}

static void bf54x_tx_data_buffer(void)
{
}

static int bf54x_urb_enqueue(
	struct usb_hcd		*hcd,
	struct usb_host_endpoint *hep,
	struct urb		*urb,
	gfp_t			mem_flags
) {
	struct bf54x_hcd	*bf54x_hcd = hcd_to_bf54x(hcd);
	struct usb_device	*udev = urb->dev;
	unsigned int		pipe = urb->pipe;
	int			is_out = !usb_pipein(pipe);
	int			type = usb_pipetype(pipe);
	int			epnum = usb_pipeendpoint(pipe);
	struct bf54x_ep		*ep = NULL;
	unsigned long		flags;
	int			i;
	int			retval = 0;

#ifdef	DISABLE_ISO
	if (type == PIPE_ISOCHRONOUS)
		return -ENOSPC;
#endif

	/* avoid all allocations within spinlocks */
	if (!hep->hcpriv)
		ep = kzalloc(sizeof *ep, mem_flags);

	spin_lock_irqsave(&bf54x_hcd->lock, flags);

	if (hep->hcpriv) {
		kfree(ep);
		ep = hep->hcpriv;
	} else if (!ep) {
		return -ENOMEM;

	} else {
		INIT_LIST_HEAD(&ep->schedule);
		ep->udev = usb_get_dev(udev);
		ep->epnum = epnum;
		ep->maxpacket = usb_maxpacket(udev, urb->pipe, is_out);
		usb_settoggle(udev, epnum, is_out, 0);

		if (type == PIPE_CONTROL)
			ep->nextpid = USB_PID_SETUP;
		else if (is_out)
			ep->nextpid = USB_PID_OUT;
		else
			ep->nextpid = USB_PID_IN;

		switch (type) {
		case PIPE_ISOCHRONOUS:
		case PIPE_INTERRUPT:
			if (urb->interval > PERIODIC_SIZE)
				urb->interval = PERIODIC_SIZE;
			ep->period = urb->interval;
			ep->branch = PERIODIC_SIZE;
			ep->load = usb_calc_bus_time(udev->speed, !is_out,
				(type == PIPE_ISOCHRONOUS),
				usb_maxpacket(udev, pipe, is_out))
					/ 1000;
			break;
		}

		ep->hep = hep;
		hep->hcpriv = ep;
	}

	/* maybe put endpoint into schedule */
	if (epnum == 0)
		bf54x_tx_zero_buffer();
	else
		bf54x_tx_data_buffer();

	return retval;
}

static int bf54x_urb_dequeue(struct usb_hcd *hcd, struct urb *urb)
{
	int			retval = 0;

	return retval;
}

static void
bf54x_endpoint_disable(struct usb_hcd *hcd, struct usb_host_endpoint *hep)
{
	struct bf54x_ep	*ep = hep->hcpriv;

	if (!ep)
		return;

	usb_put_dev(ep->udev);
	kfree(ep);
	hep->hcpriv = NULL;
}

/* the virtual root hub timer IRQ checks for hub status */
static int
bf54x_hub_status_data(struct usb_hcd *hcd, char *buf)
{
	struct bf54x_hcd *bf54x_hcd = hcd_to_bf54x(hcd);

	*buf = (1 << 1);
	return 1;
}

static void
bf54x_hub_descriptor (
	struct bf54x_hcd *bf54x_hcd,
	struct usb_hub_descriptor	*desc
) {
	desc->bDescriptorType = 0x29;
	desc->bHubContrCurrent = 0;

	desc->bNbrPorts = 1;
	desc->bDescLength = 9;

	desc->bPwrOn2PwrGood = 0;

	desc->wHubCharacteristics = 0;

	/* two bitmaps:  ports removable, and legacy PortPwrCtrlMask */
	desc->bitmap[0] = 0 << 1;
	desc->bitmap[1] = ~0;
}

static int
bf54x_hub_control(
	struct usb_hcd	*hcd,
	u16		typeReq,
	u16		wValue,
	u16		wIndex,
	char		*buf,
	u16		wLength
) {
	struct bf54x_hcd *bf54x_hcd = hcd_to_bf54x(hcd);
	int		retval = 0;
	unsigned long	flags;

	spin_lock_irqsave(&bf54x_hcd->lock, flags);

	switch (typeReq) {
	case ClearHubFeature:
	case SetHubFeature:
		switch (wValue) {
		case C_HUB_OVER_CURRENT:
		case C_HUB_LOCAL_POWER:
			break;
		default:
			goto error;
		}
		break;
	case ClearPortFeature:
		if (wIndex != 1 || wLength != 0)
			goto error;

		switch (wValue) {
		case USB_PORT_FEAT_ENABLE:
			break;
		case USB_PORT_FEAT_SUSPEND:
			break;
		case USB_PORT_FEAT_POWER:
			break;
		case USB_PORT_FEAT_C_ENABLE:
		case USB_PORT_FEAT_C_SUSPEND:
		case USB_PORT_FEAT_C_CONNECTION:
		case USB_PORT_FEAT_C_OVER_CURRENT:
		case USB_PORT_FEAT_C_RESET:
			break;
		default:
			goto error;
		}
		break;
	case GetHubDescriptor:
		bf54x_hub_descriptor(bf54x_hcd,
				(struct usb_hub_descriptor *) buf);
		break;
	case GetHubStatus:
		put_unaligned (cpu_to_le32(0), (__le32 *) buf);
		break;
	case GetPortStatus:
		if (wIndex != 1)
			goto error;
		break;
	case SetPortFeature:
		if (wIndex != 1 || wLength != 0)
			goto error;
		switch (wValue) {
		case USB_PORT_FEAT_SUSPEND:
			break;
		case USB_PORT_FEAT_POWER:
			break;
		case USB_PORT_FEAT_RESET:
			break;
		default:
			goto error;
		}
		break;

	default:
error:
		/* "protocol stall" on error */
		retval = -EPIPE;
	}

	spin_unlock_irqrestore(&bf54x_hcd->lock, flags);
	return retval;
}

#ifdef	CONFIG_PM

static int
bf54x_bus_suspend(struct usb_hcd *hcd)
{
	/*
	 * No further transaction are started and
	 * No SOF packets are generated.
	 */
	/*
	 * TODO: ADSP-BF54x PHR 13-33
	 * If the SYSPEND_MODE bit in the USB_POWER register is set,
	 * the USB controller completes the current transaction then
	 * stops the transaction scheduler and frame counter.
	 */
	return 0;
}

static int
bf54x_bus_resume(struct usb_hcd *hcd)
{
	/* the frame counter and transaction scheduler are started */
	/*
	 * TODO: ADSP-BF54x PHR 13-33
	 * To exit suspend mode, the processor core should set the RESUME_MODE
	 * bit and clear the SYSPEND_MODE bit in the USB_POWER register.
	 * While the RESUME_MODE bit is high, the USB controller generates
	 * resume signaling on the bus. After 20ms, the processor core should
	 * clear the RESUME_MODE bit
	 */
	return 0;
}

#else

#define	bf54x_bus_suspend	NULL
#define	bf54x_bus_resume	NULL

#endif

static struct hc_driver bf54x_hc_driver = {
	.description =		hcd_name,
	.hcd_priv_size =	sizeof(struct bf54x_hcd),

	/*
	 * generic hardware linkage
	 */
	.irq =			bf54x_hcd_irq,
	.flags =		HCD_USB11 | HCD_MEMORY,
	/* Basic lifecycle operations */
	.start =		bf54x_hcd_start,
	.stop =			bf54x_hcd_stop,

	/*
	 * managing i/o requests and associated device resources
	 */
	.urb_enqueue =		bf54x_urb_enqueue,
	.urb_dequeue =		bf54x_urb_dequeue,
	.endpoint_disable =	bf54x_endpoint_disable,

	/*
	 * root hub support
	 */
	.hub_status_data =	bf54x_hub_status_data,
	.hub_control =		bf54x_hub_control,
	.bus_suspend =		bf54x_bus_suspend,
	.bus_resume =		bf54x_bus_resume,
};

static void bf54x_hcd_reset(void)
{
	/* Acknowledge all USB interrupts and disable them */
	bfin_write_USB_INTRUSB(bfin_read_USB_INTRUSB() & 0xff);
	bfin_write_USB_INTRUSBE(0x0);

	/*
	 * Acknowledge all pending Tx interrupts and disable
	 * them, except EP0
	 */
	bfin_write_USB_INTRTX(bfin_read_USB_INTRTX() & 0xff);
	bfin_write_USB_INTRTXE(0x1);

	/* Acknowledge all pending Rx interrupts */
	bfin_write_USB_INTRRX(bfin_read_USB_INTRRX() & 0xff);
	bfin_write_USB_INTRRXE(0x0);

	/* Device address set to zero */
	bfin_write_USB_FADDR(0x0);

	/* Endpoint index register set to zero */
	bfin_write_USB_INDEX(0x0);
}

static void bf54x_hcd_wait_connect(void)
{
	volatile int i;
	while (bfin_read_USB_OTG_DEV_CTL() & (1 << 7)) {
		bfin_write_USB_INTRUSBE(RESET_OR_BABLE_B |
				CONN_B | DISCON_B);
		bfin_write_USB_GLOBAL_CTL(GLOBAL_ENA | EP1_TX_ENA | EP2_TX_ENA |
					EP3_TX_ENA | EP4_TX_ENA | EP5_TX_ENA |
					EP6_TX_ENA | EP7_TX_ENA | EP1_RX_ENA |
					EP2_RX_ENA | EP3_RX_ENA | EP4_RX_ENA |
					EP5_RX_ENA | EP6_RX_ENA | EP7_RX_ENA);
		bfin_write_USB_INTRTXE(0x1);
		bfin_write_USB_POWER(HS_ENABLE | SUSPEND_MODE);
		bfin_write_USB_OTG_DEV_CTL(bfin_read_USB_OTG_DEV_CTL() |
				SESSION);
		/* FIXME: we shouldn't use this kind of delay implement */
		for (i = 0; i < 0xffff; i++);

		if (!(bfin_read_USB_OTG_DEV_CTL() & (1 << 7)))
			break;

		bfin_write_USB_INTRUSB(bfin_read_USB_INTRUSB() | 0x0);
		bfin_write_USB_POWER(0x20);
		CSYNC();
		/* FIXME: we shouldn't use this kind of delay implement */
		for (i = 0; i < 0x4fffff; i++);
	}
}


static void bf54x_config_host(void)
{
	/* Issue Reset */
	bfin_write_USB_POWER((HS_ENABLE) |
				(SUSPEND_MODE));

	bfin_write_USB_OTG_DEV_CTL(bfin_read_USB_OTG_DEV_CTL() | SESSION);
	/*
	 * we come up as B-Device in case no cable plugged in,
	 * we wait for user to plug in the usb cable
	 */
#if 0
	if (bfin_read_USB_OTG_DEV_CTL() & (1 << 7))
		bf54x_hcd_wait_connect();
#endif

	/* clear connection interrupt */
	bfin_write_USB_INTRUSB(bfin_read_USB_INTRUSB() | CONN_B);

	bfin_write_USB_POWER(RESET | ENABLE_SUSPENDM | HS_ENABLE);
	udelay(2);
	bfin_write_USB_POWER(HS_ENABLE | ENABLE_SUSPENDM);

	/* enable SOF interrupt */
	bfin_write_USB_INTRUSBE(bfin_read_USB_INTRUSBE() | SOF_BE);
}

/*-------------------------------------------------------------------------*/

static int __devexit
bf54x_hcd_remove(struct platform_device *dev)
{
	struct usb_hcd		*hcd = platform_get_drvdata(dev);
	struct bf54x_hcd	*bf54x_hcd = hcd_to_bf54x(hcd);
	struct resource		*res;

	usb_remove_hcd(hcd);

	usb_put_hcd(hcd);
	return 0;
}

static int __devinit
bf54x_hcd_probe(struct platform_device *dev)
{
	struct usb_hcd		*hcd;
	struct bf54x_hcd	*bf54x_hcd;
	struct resource		*addr, *data;
	int			irq = 11;
	void __iomem		*addr_reg;
	void __iomem		*data_reg;
	int			retval;
	u8			tmp, ioaddr = 0;

	addr = platform_get_resource(dev, IORESOURCE_MEM, 0);
	/* Perform some primitive checking with the device */
	if ((bfin_read_USB_GLOBINTR() != 0x111)
			|| (bfin_read_USB_INTRTXE() != 0x00FF)
			|| (bfin_read_USB_INTRRXE() != 0x00FE))
		return -ENODEV;

	/*
	 * Rev 1.0 BF549 EZ-KITs require PE7 to be high for both DEVICE
	 * and OTG HOST modes, while rev 1.1 and greater require PE7 to
	 * be low for DEVICE mode and high for HOST mode. We set it high
	 * here because we are in host mode
	 */
	bfin_write_PORTE_DIR_SET(bfin_read_PORTE_DIR_SET() | 0x0080);
	bfin_write_PORTE_SET(bfin_read_PORTE_SET() | 0x0080);

	bf54x_hcd_reset();

	/* TODO
	 * Set SIC-IVG register
	 */

	/* allocate and initialize hcd */
	hcd = usb_create_hcd(&bf54x_hc_driver, &dev->dev, dev->dev.bus_id);
	if (!hcd)
		return -ENOMEM;

	hcd->rsrc_start = addr->start;

	/* Configure PLL oscillator register */
	bfin_write_USB_PLLOSC_CTRL(0x30a8);

	/* Maximim packet size for EP0 Rx Endpoint */
	bfin_write_USB_EP_NI0_RXMAXP(64);

	/* Maximim packet size for EP0 Tx Endpoint */
	bfin_write_USB_EP_NI0_TXMAXP(64);

	/* Enable reset and other connection interrupts */
	bfin_write_USB_INTRUSBE(RESET_OR_BABLE_B |
			CONN_B | DISCON_B);

	/*
	 * Enable USB and the data endpoint logic by setting appropriate bits
	 * in Global Control register with the execution of below statement
	 * we are ready to receive and transmit setup packets on EP0
	 */
	bfin_write_USB_GLOBAL_CTL(GLOBAL_ENA | EP1_TX_ENA | EP2_TX_ENA |
					EP3_TX_ENA | EP4_TX_ENA | EP5_TX_ENA |
					EP6_TX_ENA | EP7_TX_ENA | EP1_RX_ENA |
					EP2_RX_ENA | EP3_RX_ENA | EP4_RX_ENA |
					EP5_RX_ENA | EP6_RX_ENA | EP7_RX_ENA);
	bf54x_config_host();

	/* The chip's IRQ is level triggered, active high.  A requirement
	 * for platform device setup is to cope with things like signal
	 * inverters (e.g. CF is active low) or working only with edge
	 * triggers (e.g. most ARM CPUs).  Initial driver stress testing
	 * was on a system with single edge triggering, so most sorts of
	 * triggering arrangement should work.
	 */

	retval = usb_add_hcd(hcd, irq, IRQF_TRIGGER_HIGH | IRQF_DISABLED | IRQF_SHARED);
	if (retval != 0)
		usb_put_hcd(hcd);

	return retval;
}

#ifdef	CONFIG_PM

/* for this device there's no useful distinction between the controller
 * and its root hub, except that the root hub only gets direct PM calls
 * when CONFIG_USB_SUSPEND is enabled.
 */

static int
bf54x_hcd_suspend(struct platform_device *dev, pm_message_t state)
{
	struct usb_hcd	*hcd = platform_get_drvdata(dev);
	struct bf54x_hcd *bf54x_usb = hcd_to_bf54x(hcd);
	int	retval = 0;

	switch (state.event) {
	case PM_EVENT_FREEZE:
		retval = bf54x_bus_suspend(hcd);
		break;
	case PM_EVENT_SUSPEND:
	case PM_EVENT_PRETHAW:		/* explicitly discard hw state */
		port_power(bf54x_usb, 0);
		break;
	}
	if (retval == 0)
		dev->dev.power.power_state = state;
	return retval;
}

static int
bf54x_hcd_resume(struct platform_device *dev)
{
	struct usb_hcd	*hcd = platform_get_drvdata(dev);
	struct bf54x_hcd *bf54x_hcd = hcd_to_bf54x(hcd);

	/* with no "check to see if VBUS is still powered" board hook,
	 * let's assume it'd only be powered to enable remote wakeup.
	 */
	if (dev->dev.power.power_state.event == PM_EVENT_SUSPEND
			|| !device_can_wakeup(&hcd->self.root_hub->dev)) {
		port_power(bf54x_usb, 1);
		usb_root_hub_lost_power(hcd->self.root_hub);
		return 0;
	}

	dev->dev.power.power_state = PMSG_ON;
	return bf54x_bus_resume(hcd);
}

#else

#define	bf54x_hcd_suspend	NULL
#define	bf54x_hcd_resume	NULL

#endif

struct platform_driver bf54x_hcd_driver = {
	.probe =	bf54x_hcd_probe,
	.remove =	__devexit_p(bf54x_hcd_remove),

	.suspend =	bf54x_hcd_suspend,
	.resume =	bf54x_hcd_resume,
	.driver = {
		.name =	(char *) hcd_name,
		.owner = THIS_MODULE,
	},
};
EXPORT_SYMBOL(bf54x_hcd_driver);

/*-------------------------------------------------------------------------*/

static int __init bf54x_hcd_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	pr_info("driver %s, %s\n", hcd_name, DRIVER_VERSION);
	return platform_driver_register(&bf54x_hcd_driver);
}
module_init(bf54x_hcd_init);

static void __exit bf54x_hcd_cleanup(void)
{
	platform_driver_unregister(&bf54x_hcd_driver);
}
module_exit(bf54x_hcd_cleanup);
