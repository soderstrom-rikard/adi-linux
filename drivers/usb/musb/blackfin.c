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

#include "musb_core.h"
#include "blackfin.h"

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
	u8		devctl;
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
	return 0;

	/*
	 * Rev 1.0 BF549 EZ-KITs require PE7 to be high for both DEVICE
	 * and OTG HOST modes, while rev 1.1 and greater require PE7 to
	 * be low for DEVICE mode and high for HOST mode. We set it high
	 * here because we are in host mode
	 */
	bfin_write_PORTE_DIR_SET(bfin_read_PORTE_DIR_SET() | 0x0080);
	bfin_write_PORTE_SET(bfin_read_PORTE_SET() | 0x0080);

	/* TODO
	 * Set SIC-IVG register
	 */

	/* Configure PLL oscillator register */
	bfin_write_USB_PLLOSC_CTRL(0x30a8);

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

	if (is_host_enabled(musb))
		musb->board_set_vbus = bfin_set_vbus;
	if (is_peripheral_enabled(musb))
		musb->xceiv.set_power = bfin_set_power;

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

	musb_platform_suspend(musb);

	return 0;
}
