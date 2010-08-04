/*
 * net/dsa/tag_stpid.c - special tag identifier,
 * 0x810 + 4 bit "port mask" + 3 bit 8021p + 1 bit CFI + 12 bit VLAN ID
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/etherdevice.h>
#include <linux/list.h>
#include <linux/netdevice.h>
#include "dsa_priv.h"

#define ETH_P_8021QH      (ETH_P_8021Q >> 8)
#define ETH_P_8021QL      (ETH_P_8021Q & 0xFF)
#define STPID_HLEN        4

#define ZERO_VID          0
#define RESERVED_VID      0xFFF
#define STPID_VID         ZERO_VID

int stpid_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct dsa_slave_priv *p = netdev_priv(dev);
	u8 *dsa_header;

	dev->stats.tx_packets++;
	dev->stats.tx_bytes += skb->len;

	/*
	 * For 802.1Q frames, convert to STPID tagged frames,
	 * do nothing for common frames.
	 */
	if (skb->protocol == htons(ETH_P_8021Q)) {
		if (skb_cow_head(skb, 0) < 0)
			goto out_free;

		dsa_header = skb->data + 2 * ETH_ALEN;
		dsa_header[1] = p->port & 0x03;
	}

	skb->protocol = htons(ETH_P_STPID);

	skb->dev = p->parent->dst->master_netdev;
	dev_queue_xmit(skb);

	return NETDEV_TX_OK;

out_free:
	kfree_skb(skb);
	return NETDEV_TX_OK;
}

static int stpid_rcv(struct sk_buff *skb, struct net_device *dev,
		   struct packet_type *pt, struct net_device *orig_dev)
{
	struct dsa_switch_tree *dst = dev->dsa_ptr;
	struct dsa_switch *ds = dst->ds[0];
	u8 *dsa_header;
	int source_port;
	int vid;

	if (unlikely(ds == NULL))
		goto out_drop;

	skb = skb_unshare(skb, GFP_ATOMIC);
	if (skb == NULL)
		goto out;

	/* The ether_head has been pulled by master driver */
	dsa_header = skb->data - 2;

	vid = ((dsa_header[2] & 0x0f) << 8 | dsa_header[3]);

	source_port = dsa_header[1] & 0x03;
	if (source_port >= DSA_MAX_PORTS || ds->ports[source_port] == NULL)
		goto out_drop;

	if (((dsa_header[0] & ETH_P_8021QH) == ETH_P_8021QH) &&
			(vid != STPID_VID)) {
		u8 new_header[STPID_HLEN];

		/* Convert STPID tag to 802.1q tag */
		new_header[0] = ETH_P_8021QH;
		new_header[1] = ETH_P_8021QL;

		if (skb->ip_summed == CHECKSUM_COMPLETE) {
			__wsum c = skb->csum;
			c = csum_add(c, csum_partial(new_header, 2, 0));
			c = csum_sub(c, csum_partial(dsa_header, 2, 0));
			skb->csum = c;
		}
		memcpy(dsa_header, new_header, STPID_HLEN / 2);

	} else if ((dsa_header[0] & ETH_P_8021QH) &&
			(vid == STPID_VID)) {

		if (unlikely(!pskb_may_pull(skb, STPID_HLEN)))
			goto out_drop;

		/* Remove STPID tag and update checksum. */
		if (skb->ip_summed == CHECKSUM_COMPLETE) {
			__wsum c = skb->csum;
			c = csum_sub(c, csum_partial(dsa_header, STPID_HLEN, 0));
			skb->csum = c;
		}
		memmove(skb->data - ETH_HLEN + STPID_HLEN,
				skb->data - ETH_HLEN, 2 * ETH_ALEN);
		skb_pull(skb, STPID_HLEN);
	}

	skb->dev = ds->ports[source_port];
	skb_push(skb, ETH_HLEN);
	skb->pkt_type = PACKET_HOST;
	skb->protocol = eth_type_trans(skb, skb->dev);
	skb->dev->last_rx = jiffies;
	skb->dev->stats.rx_packets++;
	skb->dev->stats.rx_bytes += skb->len;
	netif_receive_skb(skb);

	return 0;

out_drop:
	kfree_skb(skb);
out:
	return 0;
}

static struct packet_type stpid_packet_type = {
	.type	= __constant_htons(ETH_P_STPID),
	.func	= stpid_rcv,
};

static int __init stpid_init_module(void)
{
	dev_add_pack(&stpid_packet_type);
	return 0;
}
module_init(stpid_init_module);

static void __exit stpid_cleanup_module(void)
{
	dev_remove_pack(&stpid_packet_type);
}
module_exit(stpid_cleanup_module);
