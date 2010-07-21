/*
 * Integrated 3-Port 10/100 Managed Switch with PHYs
 *
 * - KSZ8893M support
 *
 * Copyright 2008-2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#define pr_fmt(fmt) "ksz8893m: " fmt

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

static int ksz8893m_read(unsigned char *din, unsigned char reg, int len)
{
	int i, ret;
	struct spi_message message;
	unsigned char dout[BUF_LEN];
	struct spi_transfer *t = &sw.xfer;

	t->len = len;
	t->tx_buf = dout;
	t->rx_buf = din;
	dout[0] = SPI_READ;
	dout[1] = reg;
	for (i = 2; i < len; i++)
		dout[i] = 0;

	spi_message_init(&message);
	spi_message_add_tail(t, &message);
	ret = spi_sync(sw.dev, &message);
	if (!ret)
		return message.status;

	pr_err("read reg%d failed, ret=%d\n", reg, ret);
	return ret;
}

static int ksz8893m_write(unsigned char *dout, unsigned char reg, int len)
{
	int ret;
	struct spi_message message;
	unsigned char din[BUF_LEN];
	struct spi_transfer *t = &sw.xfer;

	t->len = len;
	t->tx_buf = dout;
	t->rx_buf = din;
	dout[0] = SPI_WRITE;
	dout[1] = reg;

	spi_message_init(&message);
	spi_message_add_tail(t, &message);
	ret = spi_sync(sw.dev, &message);
	if (!ret)
		return message.status;

	pr_err("write reg%d failed, ret=%d\n", reg, ret);
	return ret;
}

static char *ksz8893m_probe(struct mii_bus *bus, int sw_addr)
{
	int ret, phyid_low, phyid_high;
	unsigned char din[BUF_LEN];

	phyid_high = mdiobus_read(bus, KSZ8893M_CPU_PORT, MII_PHYSID1);
	phyid_low = mdiobus_read(bus, KSZ8893M_CPU_PORT, MII_PHYSID2);
	if (phyid_high != PHYID_HIGH || phyid_low != PHYID_LOW)
		return NULL;

	ret = ksz8893m_read(din, ChipID0, 3);

	if (!ret && FAMILY_ID == din[2])
		return "KSZ8893M";

	return NULL;
}

static int ksz8893m_switch_reset(struct dsa_switch *ds)
{
	return 0;
}

static int ksz8893m_setup_global(struct dsa_switch *ds)
{
	int ret;
	unsigned char dout[BUF_LEN];
	unsigned char din[BUF_LEN];

	/* Set VLAN VID of port1 */
	ret = ksz8893m_read(din, Port1Control3, 3);
	if (ret)
		return ret;
	din[2] &= 0xf0;
	dout[2] = (DEFAULT_PORT_VID & 0xfff) >> 8 | din[2];
	dout[3] = DEFAULT_PORT_VID & 0xff;
	ret = ksz8893m_write(dout, Port1Control3, 4);
	if (ret)
		return ret;

	/* Set VLAN VID of port2 */
	ret = ksz8893m_read(din, Port2Control3, 3);
	if (ret)
		return ret;
	din[2] &= 0xf0;
	dout[2] = (DEFAULT_PORT_VID & 0xfff) >> 8 | din[2];
	dout[3] = DEFAULT_PORT_VID & 0xff;
	ret = ksz8893m_write(dout, Port2Control3, 4);
	if (ret)
		return ret;

	/* Set VLAN VID of port3 */
	ret = ksz8893m_read(din, Port3Control3, 3);
	if (ret)
		return ret;
	din[2] &= 0xf0;
	dout[2] = (DEFAULT_PORT_VID & 0xfff) >> 8 | din[2];
	dout[3] = DEFAULT_PORT_VID & 0xff;
	ret = ksz8893m_write(dout, Port3Control3, 4);
	if (ret)
		return ret;

	/* Insert VLAN tag that egress Port3 */
	ret = ksz8893m_read(din, Port3Control0, 3);
	if (ret)
		return ret;
	dout[2] = TAG_INSERTION | din[2];
	ret = ksz8893m_write(dout, Port3Control0, 3);
	if (ret)
		return ret;

	/* Enable STPID Mode */
	ret = ksz8893m_read(din, GlobalControl9, 3);
	if (ret)
		return ret;
	dout[2] = SPECIAL_TPID_MODE | din[2];
	ret = ksz8893m_write(dout, GlobalControl9, 3);
	if (ret)
		return ret;

	/* Start switch */
	dout[2] = START_SWITCH;
	ret = ksz8893m_write(dout, ChipID1_StartSwitch, 3);
	if (ret)
		return ret;

	return 0;
}

static int ksz8893m_setup_port(struct dsa_switch *ds, int p)
{
	int val, ret;
	val = mdiobus_read(ds->master_mii_bus, p, MII_BMCR);
	if (val < 0)
		return val;
	val |= AN_ENABLE | FORCE_100 | FORCE_FULL_DUPLEX;
	val &= ~(POWER_DOWN | DISABLE_MDIX | DIS_FAR_END_FAULT |\
			DISABLE_TRANSMIT | DISABLE_LED);
	ret = mdiobus_write(ds->master_mii_bus, p, MII_BMCR, val);
	if (ret < 0)
		return ret;

	val = mdiobus_read(ds->master_mii_bus, p, MII_ADVERTISE);
	if (val < 0)
		return val;
	val |= ADV_10_HALF | ADV_10_FULL | ADV_100_HALF | ADV_100_FULL;
	ret = mdiobus_write(ds->master_mii_bus, p, MII_ADVERTISE, val);
	if (ret < 0)
		return ret;
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

	pr_warning("use default phy addr 3\n");
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

static void ksz8893m_poll_link(struct dsa_switch *ds)
{
	int i;

	for (i = 1; i < KSZ8893M_PORT_NUM; i++) {
		struct net_device *dev;
		int val;
		int link;
		int speed;
		int duplex;
		int anc;

		dev = ds->ports[i];
		if (dev == NULL)
			continue;

		link = 0;
		if (dev->flags & IFF_UP) {
			val = mdiobus_read(ds->master_mii_bus, i, MII_BMSR);
			if (val < 0)
				continue;

			link = !!(val & LINK_STATUS);
			anc = !!(val & AN_COMPLETE);
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
		if (val < 0)
			continue;
		val &= HALF_10_CAPABLE | FULL_10_CAPABLE |\
		       HALF_100_CAPABLE | FULL_100_CAPABLE;
		if (val & FULL_100_CAPABLE) {
			speed = 100;
			duplex = 1;
		} else if (val & HALF_100_CAPABLE) {
			speed = 100;
			duplex = 0;
		} else if (val & FULL_10_CAPABLE) {
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
	if (sw.dev) {
		pr_err("only one instance supported at a time\n");
		return 1;
	}
	memset(&sw.xfer, 0, sizeof(sw.xfer));
	sw.dev = spi;
	return 0;
}

static int __devexit spi_switch_remove(struct spi_device *spi)
{
	sw.dev = NULL;
	return 0;
}

static struct spi_driver spi_switch_driver = {
	.driver = {
		.name	= "ksz8893m",
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

static int __init ksz8893m_init(void)
{
	int ret;

	ret = spi_register_driver(&spi_switch_driver);
	if (ret) {
		pr_err("can't register driver\n");
		return ret;
	}

	register_switch_driver(&ksz8893m_switch_driver);
	return 0;
}
module_init(ksz8893m_init);

static void __exit ksz8893m_cleanup(void)
{
	spi_unregister_driver(&spi_switch_driver);
	unregister_switch_driver(&ksz8893m_switch_driver);
}
module_exit(ksz8893m_cleanup);

MODULE_AUTHOR("Graf Yang <graf.yang@analog.com>");
MODULE_DESCRIPTION("KSZ8893M driver for DSA");
MODULE_LICENSE("GPL");
