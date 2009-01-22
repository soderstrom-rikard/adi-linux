/*
 * Integrated 3-Port 10/100 Managed Switch with PHYs
 *
 * - KSZ8893M support
 *
 * Copyright 2008-2009 Analog Devices Inc.
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Licensed under the GPL-2 or later.
 *
 */

#include <linux/list.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/spi/spi.h>
#include "dsa_priv.h"
#include "ksz8893m.h"

#define  BUF_LEN 6

static struct _spi_switch {
	struct spi_transfer xfer;
	struct spi_device *dev;
} sw;

static int switch_read_spi(unsigned char *din, unsigned char reg, int len)
{
	struct spi_message message;
	unsigned char dout[BUF_LEN];
	struct spi_transfer *t = &sw.xfer;
	int i;

	t->len = len;
	t->tx_buf = dout;
	t->rx_buf = din;
	((unsigned char *)(t->tx_buf))[0] = SPI_READ;
	((unsigned char *)(t->tx_buf))[1] = reg;
	for (i = 2; i < len; i++)
		((unsigned char *)(t->tx_buf))[i] = 0;

	spi_message_init(&message);
	spi_message_add_tail(t, &message);
	return spi_sync(sw.dev, &message);
}

static int switch_write_spi(unsigned char *dout, unsigned char reg, int len)
{
	struct spi_message message;
	unsigned char din[BUF_LEN];
	struct spi_transfer *t = &sw.xfer;

	t->len = len;
	t->tx_buf = dout;
	t->rx_buf = din;
	((unsigned char *)(t->tx_buf))[0] = SPI_WRITE;
	((unsigned char *)(t->tx_buf))[1] = reg;

	spi_message_init(&message);
	spi_message_add_tail(t, &message);
	return spi_sync(sw.dev, &message);
}

static char *ksz8893m_probe(struct mii_bus *bus, int sw_addr)
{
	int phyid_low, phyid_high;
	unsigned char din[BUF_LEN];

	phyid_high = mdiobus_read(bus, KSZ8893M_CPU_PORT, MII_PHYSID1);
	phyid_low = mdiobus_read(bus, KSZ8893M_CPU_PORT, MII_PHYSID2);
	if (phyid_high != PHYID_HIGH || phyid_low != PHYID_LOW)
		return NULL;

	switch_read_spi(din, ChipID0, 3);

	if (FamilyID == din[2])
		return "KSZ8893M";

	return NULL;
}

static int ksz8893m_switch_reset(struct dsa_switch *ds)
{
	return 0;
}

static int ksz8893m_setup_global(struct dsa_switch *ds)
{
	unsigned char dout[BUF_LEN];
	unsigned char din[BUF_LEN];

	/* Set VLAN VID of port1 */
	switch_read_spi(din, Port1Control3, 3);
	din[2] &= 0xf0;
	dout[2] = (DEFAULT_PORT_VID & 0xfff) >> 8 | din[2];
	dout[3] = DEFAULT_PORT_VID & 0xff;
	switch_write_spi(dout, Port1Control3, 4);
			
	/* Set VLAN VID of port2 */
	switch_read_spi(din, Port2Control3, 3);
	din[2] &= 0xf0;
	dout[2] = (DEFAULT_PORT_VID & 0xfff) >> 8 | din[2];
	dout[3] = DEFAULT_PORT_VID & 0xff;
	switch_write_spi(dout, Port2Control3, 4);
			
	/* Set VLAN VID of port3 */
	switch_read_spi(din, Port3Control3, 3);
	din[2] &= 0xf0;
	dout[2] = (DEFAULT_PORT_VID & 0xfff) >> 8 | din[2];
	dout[3] = DEFAULT_PORT_VID & 0xff;
	switch_write_spi(dout, Port3Control3, 4);

	/* Insert VLAN tag that egress Port3 */
	switch_read_spi(din, Port3Control0, 3);
	dout[2] = 0x4 | din[2];
	switch_write_spi(dout, Port3Control0, 3);

	/* Enable STPID Mode */
	switch_read_spi(din, GlobalControl9, 3);
	dout[2] = 0x01 | din[2];
	switch_write_spi(dout, GlobalControl9, 3);

	/* Start switch */
	dout[2] = StartSwitch;
	switch_write_spi(dout, ChipID1_StartSwitch, 3);

	return 0;
}

static int ksz8893m_setup_port(struct dsa_switch *ds, int p)
{
	int val;
	val = mdiobus_read(ds->master_mii_bus, p, MII_BMCR);
	/* bit 12 auto-negotiation enabled */
	val |= 0x1000;
	/* bit 13 force 100, bit 8 force full duplex */
	val |= 0x2100;
	/* bit 11 power on, bit 3 enable auto MDI-X, bit 2 enble far-end fault detection */
	val &= ~0x80c;
	mdiobus_write(ds->master_mii_bus, p, MII_BMCR, val);

	val = mdiobus_read(ds->master_mii_bus, p, MII_ADVERTISE);
	/* bit 8/7/6/5 advertise 100full/100half/10full/10half ability */
	val |= 0x1e0;
	mdiobus_write(ds->master_mii_bus, p, MII_ADVERTISE, val);
	return 0;
}

static int ksz8893m_setup(struct dsa_switch *ds)
{
	int i;
	int ret;

	ret = ksz8893m_switch_reset(ds);
	if (ret < 0)
		return ret;

	ret = ksz8893m_setup_global(ds);
	if (ret < 0)
		return ret;

	for (i = 1; i < KSZ8893M_PORT_NUM; i++) {
		ret = ksz8893m_setup_port(ds, i);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ksz8893m_set_addr(struct dsa_switch *ds, u8 *addr)
{
	return 0;
}

static int ksz8893m_port_to_phy_addr(int port)
{
	if (port >= 1 && port <= KSZ8893M_PORT_NUM)
		return port;

	printk(KERN_INFO "ksz8893m: use default phy addr 3\n");
	return 3;
}

static int
ksz8893m_phy_read(struct dsa_switch *ds, int port, int regnum)
{
	int phy_addr = ksz8893m_port_to_phy_addr(port);
	return mdiobus_read(ds->master_mii_bus, phy_addr, regnum);
}

static int
ksz8893m_phy_write(struct dsa_switch *ds,
			      int port, int regnum, u16 val)
{
	int phy_addr = ksz8893m_port_to_phy_addr(port);
	return mdiobus_write(ds->master_mii_bus, phy_addr, regnum, val);
}

void ksz8893m_poll_link(struct dsa_switch *ds)
{
	int i;

	for (i = 1; i < KSZ8893M_PORT_NUM; i++) {
		struct net_device *dev;
		int val;
		int link;
		int speed;
		int duplex;
		int anc;
		int fefd;

		dev = ds->ports[i];
		if (dev == NULL)
			continue;

		link = 0;
		if (dev->flags & IFF_UP) {
			val = mdiobus_read(ds->master_mii_bus, i, MII_BMSR);
			if (val < 0)
				continue;

			/* bit 2 link is up */
			link = !!(val & 0x04);
			/* bit 4 far-end fault detected */
			fefd = !!(val & 0x10);
			/* bit 5 auto-negotiation complete */
			anc = !!(val & 0x20);
		}

		if (!link) {
			if (netif_carrier_ok(dev)) {
				printk(KERN_INFO "%s: link down\n", dev->name);
				netif_carrier_off(dev);
			}
			continue;
		}

		speed = 10;
		duplex = 0;
		val = mdiobus_read(ds->master_mii_bus, i, MII_BMSR);
		/* bit 14/13/12/11 capable of 100full/100half/10full/10half */
		val &= 0x7800;
		if (val & 0x4000) {
			speed = 100;
			duplex = 1;
		} else if (val & 0x2000) {
			speed = 100;
			duplex = 0;
		} else if (val & 0x1000) {
			speed = 10;
			duplex = 1;
		}

		if (!netif_carrier_ok(dev)) {
			printk(KERN_INFO "%s: link up, %d Mb/s, %s duplex\n",
					dev->name, speed, duplex ? "full" : "half");
			netif_carrier_on(dev);
		}
	}
}

static int __devinit spi_switch_probe(struct spi_device *spi)
{
	memset(&(sw.xfer), 0, sizeof(sw.xfer));
	sw.dev = spi;
	return 0;
}

static int __devexit spi_switch_remove(struct spi_device *spi)
{
	sw.dev = NULL;
	printk(KERN_INFO "spi switch exit\n");
	return 0;
}

static struct spi_driver spi_switch_driver = {
	.driver = {
		.name	= "spi_switch",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe	= spi_switch_probe,
	.remove	= __devexit_p(spi_switch_remove),
};

static struct dsa_switch_driver ksz8893m_switch_driver = {
	.tag_protocol		= __constant_htons(ETH_P_STPID),
	.probe			= ksz8893m_probe,
	.setup			= ksz8893m_setup,
	.set_addr		= ksz8893m_set_addr,
	.phy_read		= ksz8893m_phy_read,
	.phy_write		= ksz8893m_phy_write,
	.poll_link		= ksz8893m_poll_link,
};

int __init ksz8893m_init(void)
{
	int ret;
	ret = spi_register_driver(&spi_switch_driver);
	if (ret) {
		printk(KERN_ERR "Can't register spi_switch_driver!\n");
		return ret;
	}

	register_switch_driver(&ksz8893m_switch_driver);
	return 0;
}
module_init(ksz8893m_init);

void __exit ksz8893m_cleanup(void)
{
	spi_unregister_driver(&spi_switch_driver);
	unregister_switch_driver(&ksz8893m_switch_driver);
}
module_exit(ksz8893m_cleanup);

MODULE_AUTHOR("Graf.Yang <graf.yang@analog.com>");
MODULE_DESCRIPTION("KSZ8893M driver for DSA");
MODULE_LICENSE("GPL");

