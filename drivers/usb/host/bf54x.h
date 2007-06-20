/*
 * File:	drivers/usb/host/bf54x.h
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

#define NUM_PHYSICAL_ENDPOINTS	8	/*including EP0 */
#define	LOG2_PERIODIC_SIZE	5
#define	PERIODIC_SIZE		(1 << LOG2_PERIODIC_SIZE)

struct bf54x_hcd {
	spinlock_t		lock;
	/* async schedule: control, bulk */
	struct list_head	async;
};

static inline struct bf54x_usb *hcd_to_bf54x(struct usb_hcd *hcd)
{
	return (struct bf54x_usb *) (hcd->hcd_priv);
}

static inline struct usb_hcd *bf54x_to_hcd(struct bf54x_usb *bf54x_usb)
{
	return container_of((void *) bf54x_usb, struct usb_hcd, hcd_priv);
}

struct bf54x_ep {
	struct usb_host_endpoint *hep;
	struct usb_device	*udev;

	u8			maxpacket;
	u8			epnum;
	u8			nextpid;
	/* periodic schedule */
	u16			period;
	u16			branch;
	u16			load;
	struct bf54x_ep		*next;
	/* async schedule */
	struct list_head	schedule;
};
