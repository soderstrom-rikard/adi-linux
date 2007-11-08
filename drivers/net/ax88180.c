
/* ax88180: ASIX AX88180 Non-PCI Gigabit Ethernet Linux driver */

/*
 * Copyright (c) 2005 ASIX Electronics Corporation
 * Written by Allan Chou <allan@asix.com.tw>
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */

/*
 * ========================================================================
 * ASIX AX88180 Non-PCI 16/32-bit Gigabit Ethernet Linux Driver
 *
 * The AX88180 Ethernet controller is high performance and highly
 * integrated local CPU bus Ethernet controllers with embedded 40K bytes
 * SRAM and supports both 16-bit and 32-bit SRAM-Like interfaces
 * for any embedded systems.
 * The AX88180 is a single chip 10/100/1000Mbps Gigabit Ethernet controller
 * that supports both MII and RGMII interfaces and is compliant to
 * IEEE 802.3, IEEE 802.3u and IEEE 802.3z standards.
 *
 * Please visit ASIX's web site (http://www.asix.com.tw) for more details.
 *
 * Module Name : ax88180.c
 * Purpose     : This file is the main file.
 * Author      : Allan Chou <allan@asix.com.tw>
 * Date        : 2005-12-07
 * Notes       :
 * History     :
 * $Log:$
 * 1.0.0	2005-12-07
 * New release for AX88180 US1 chip.
 *
 * 1.0.1	2006-06-14
 * 1.Modify to support AX88180 US2 chip.
 * 2.Modify to support AX88180 US2 burst data access function.
 *
 * 3.Port to Blackfin (16-bit Mode) Phil Wilshire
 * 4.Port to Blackfin 16-bit Mode, restore 32-bit Mode, add to platfrom device
 *	port to linux-2.6.22, remove useless debug info,
 *	Various bug fixes, coding style cleanups, etc.
 *	8 Nov. 2007 Michael Hennerich <michael.hennerich@analog.com>
 *
 *
 *
 *
 *
 *
 *
 * ========================================================================
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/ioport.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/spinlock.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/crc32.h>
#include <linux/ethtool.h>
#include <linux/mii.h>
#include <linux/if_ether.h>
#include <linux/platform_device.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/uaccess.h>	       /* User space memory access functions */
#include <asm/blackfin.h>

#ifdef CONFIG_ARCH_S3C2410
#include <asm/arch/regs-mem.h>
#include <asm/arch/regs-irq.h>
#endif

#include "ax88180.h"

/*
 * ===========================================================================
 * <<<<<<                   Declare Global Variables                    >>>>>>
 * ===========================================================================
 */
#define	DRV_NAME	"ax88180"
#define	DRV_VERSION	"v1.1.0"

static const char version[] =
KERN_INFO "ax88180: ASIX AX88180 Non-PCI 32/16-bit Gigabit Ethernet Driver " DRV_VERSION "\n"
KERN_INFO "ax88180: Please visit http://www.asix.com.tw for the latest driver.\n";

/*
 * ===========================================================================
 * <<<<<<             Declare Macro/Structure Definition                >>>>>>
 * ===========================================================================
 */

/* Information that need to be kept for each board. */
struct ax88180_local {
	struct net_device_stats	stats;
	unsigned int Phy_MemBase;
	unsigned int PhyAddr;
	unsigned int PhyID0;
	unsigned int MediaMode;
	unsigned int RealMediaMode;
	unsigned int ForceMedia;
	unsigned int LineSpeed;
	unsigned int DuplexMode;
	unsigned int JumboFlag;
	unsigned int RxFilterMode;
	unsigned int FirstTxDesc;
	unsigned int NextTxDesc;
	unsigned int rxbuf_overflow_count;
	unsigned char *rx_buf;
	spinlock_t lock;
	struct mii_if_info mii_if;
};

static unsigned int Log_MemBase;
unsigned int jumbo = ENABLE_JUMBO;
unsigned int media;


#define	PRINTK(flag, args...) if (flag & DEBUG_FLAGS) printk(args)

#ifdef CONFIG_AX88180_16BIT
#define MACREG_OFFSET_16BIT	(0xDD00)
#define READ_RXBUF(data) data = inw((void __iomem *)(Log_MemBase + RXBUFFER_START))
#define WRITE_TXBUF(data) outw(data, Log_MemBase + TXBUFFER_START)
#define READ_MACREG(regaddr, regdata) regdata = inw((void __iomem *)(Log_MemBase + regaddr - MACREG_OFFSET_16BIT))
#define WRITE_MACREG(regaddr, regdata) outw(regdata, Log_MemBase + regaddr - MACREG_OFFSET_16BIT)
#else
#define READ_RXBUF(data) data = inl((void __iomem *)(Log_MemBase + RXBUFFER_START))
#define WRITE_TXBUF(data) outl(data, Log_MemBase + TXBUFFER_START)
#define READ_MACREG(regaddr, regdata) regdata = inw((void __iomem *)(Log_MemBase + regaddr))
#define WRITE_MACREG(regaddr, regdata) outw(regdata, Log_MemBase + regaddr)
#endif

#define READ_PHYREG(phyaddr, regaddr, regdata) { \
	unsigned int tmpval1, k1; \
	WRITE_MACREG(MDIOCTRL, READ_PHY | (regaddr << 8) | phyaddr); \
	for (k1 = 0; k1 < 10000; k1++) { \
		READ_MACREG(MDIOCTRL, tmpval1); \
		if ((tmpval1 & READ_PHY) == 0) { \
			break; \
		} \
		udelay(1); \
	} \
	READ_MACREG(MDIODP, regdata); \
}
#define WRITE_PHYREG(phyaddr, regaddr, regdata) { \
	unsigned int tmpval2, k2; \
	WRITE_MACREG(MDIODP, regdata); \
	WRITE_MACREG(MDIOCTRL, WRITE_PHY | (regaddr << 8) | phyaddr); \
	for (k2 = 0; k2 < 10000; k2++) { \
		READ_MACREG(MDIOCTRL, tmpval2); \
		if ((tmpval2 & WRITE_PHY) == 0) { \
			break; \
		} \
		udelay(1); \
	} \
}

#define RESET_MAC { \
	unsigned int tmpval3; \
	WRITE_MACREG(MISC, MISC_RESET_MAC); \
	READ_MACREG(MISC, tmpval3); \
	WRITE_MACREG(MISC, MISC_NORMAL); \
	WRITE_MACREG(RXINDICATOR, DEFAULT_RXINDICATOR); \
	WRITE_MACREG(TXCMD, DEFAULT_TXCMD); \
	WRITE_MACREG(TXBS, DEFAULT_TXBS); \
	WRITE_MACREG(TXDES0, DEFAULT_TXDES0); \
	WRITE_MACREG(TXDES1, DEFAULT_TXDES1); \
	WRITE_MACREG(TXDES2, DEFAULT_TXDES2); \
	WRITE_MACREG(TXDES3, DEFAULT_TXDES3); \
	WRITE_MACREG(TXCFG, DEFAULT_TXCFG); \
	WRITE_MACREG(MACCFG2, DEFAULT_MACCFG2); \
	WRITE_MACREG(MACCFG3, DEFAULT_MACCFG3); \
	WRITE_MACREG(TXLEN, DEFAULT_TXLEN); \
	WRITE_MACREG(TXPAUT, DEFAULT_TXPAUT); \
	WRITE_MACREG(RXBTHD0, DEFAULT_RXBTHD0); \
	WRITE_MACREG(RXBTHD1, DEFAULT_RXBTHD1); \
	WRITE_MACREG(RXFULTHD, DEFAULT_RXFULTHD); \
	WRITE_MACREG(DOGTHD0, DEFAULT_DOGTHD0); \
	WRITE_MACREG(DOGTHD1, DEFAULT_DOGTHD1); \
}

#define RESET_PHY { \
	unsigned int tmpval3a, k3a; \
	WRITE_PHYREG(pax88180_local->PhyAddr, BMCR, PHY_RESET); \
	for (k3a = 0; k3a < 500; k3a++) { \
		READ_PHYREG(pax88180_local->PhyAddr, BMCR, tmpval3a); \
		if (!(tmpval3a & PHY_RESET)) \
			break; \
		mdelay(1); \
	} \
}

#define	INIT_TXRX_VARIABLES { \
	pax88180_local->FirstTxDesc = TXDP0; \
	pax88180_local->NextTxDesc = TXDP0; \
	pax88180_local->rxbuf_overflow_count = 0; \
}

#define	ENABLE_INTERRUPT	WRITE_MACREG(IMR, DEFAULT_IMR)
#define	DISABLE_INTERRUPT	WRITE_MACREG(IMR, CLEAR_IMR)

#define	START_READ_RXBUFF 	WRITE_MACREG(RXINDICATOR, RX_START_READ)
#define	STOP_READ_RXBUFF 	WRITE_MACREG(RXINDICATOR, RX_STOP_READ)

/* Display all AX88180 MAC registers onto console screen */
#define	DISPLAY_ALLMACREG { \
	unsigned int tmpval4; \
	int k4; \
	PRINTK(DEBUG_MSG, "ax88180: AX88180 MAC Registers:\n"); \
	for (k4 = 0xFC00; k4 <= 0xFCFF; k4 += 4) { \
		READ_MACREG(k4, tmpval4); \
		PRINTK(DEBUG_MSG, "0x%04x=0x%08lx ", k4, tmpval4); \
		if ((k4 & 0xF) == 0xC) \
			PRINTK(DEBUG_MSG, "\n"); \
	} \
	PRINTK(DEBUG_MSG, "\n"); \
}

/* Display all AX88180 PHY registers onto console screen */
#define	DISPLAY_ALLPHYREG { \
	unsigned int tmpval5; \
	PRINTK(DEBUG_MSG, "ax88180: AX88180 PHY Registers: (media=%d)\n", media); \
	READ_PHYREG(pax88180_local->PhyAddr, BMCR, tmpval5); \
	PRINTK(DEBUG_MSG, "BMCR=0x%04x ", (unsigned int)tmpval5); \
	READ_PHYREG(pax88180_local->PhyAddr, BMSR, tmpval5); \
	PRINTK(DEBUG_MSG, "BMSR=0x%04x ", (unsigned int)tmpval5); \
	READ_PHYREG(pax88180_local->PhyAddr, PHYIDR0, tmpval5); \
	PRINTK(DEBUG_MSG, "PHYIDR0=0x%04x ", (unsigned int)tmpval5); \
	READ_PHYREG(pax88180_local->PhyAddr, PHYIDR1, tmpval5); \
	PRINTK(DEBUG_MSG, "PHYIDR1=0x%04x ", (unsigned int)tmpval5); \
	READ_PHYREG(pax88180_local->PhyAddr, ANAR, tmpval5); \
	PRINTK(DEBUG_MSG, "ANAR=0x%04x ", (unsigned int)tmpval5); \
	READ_PHYREG(pax88180_local->PhyAddr, ANLPAR, tmpval5); \
	PRINTK(DEBUG_MSG, "ANLPAR=0x%04x \n", (unsigned int)tmpval5); \
	READ_PHYREG(pax88180_local->PhyAddr, ANER, tmpval5); \
	PRINTK(DEBUG_MSG, "ANER=0x%04x ", (unsigned int)tmpval5); \
	READ_PHYREG(pax88180_local->PhyAddr, AUX_1000_CTRL, tmpval5); \
	PRINTK(DEBUG_MSG, "1G_CTRL=0x%04x ", (unsigned int)tmpval5); \
	READ_PHYREG(pax88180_local->PhyAddr, AUX_1000_STATUS, tmpval5); \
	PRINTK(DEBUG_MSG, "1G_STATUS=0x%04x \n", (unsigned int)tmpval5); \
	if (pax88180_local->PhyID0 == MARVELL_88E1111_PHYIDR0) { \
		READ_PHYREG(pax88180_local->PhyAddr, M88_SSR, tmpval5); \
		PRINTK(DEBUG_MSG, "M88_SSR=0x%04x ", (unsigned int)tmpval5); \
		READ_PHYREG(pax88180_local->PhyAddr, M88_IER, tmpval5); \
		PRINTK(DEBUG_MSG, "M88_IER=0x%04x ", (unsigned int)tmpval5); \
		READ_PHYREG(pax88180_local->PhyAddr, M88_ISR, tmpval5); \
		PRINTK(DEBUG_MSG, "M88_ISR=0x%04x ", (unsigned int)tmpval5); \
		READ_PHYREG(pax88180_local->PhyAddr, M88_EXT_SCR, tmpval5); \
		PRINTK(DEBUG_MSG, "M88_EXT_SCR=0x%04x ", (unsigned int)tmpval5); \
		READ_PHYREG(pax88180_local->PhyAddr, M88_EXT_SSR, tmpval5); \
		PRINTK(DEBUG_MSG, "M88_EXT_SSR=0x%04x \n", (unsigned int)tmpval5); \
	} else if (pax88180_local->PhyID0 == CICADA_CIS8201_PHYIDR0) { \
		READ_PHYREG(pax88180_local->PhyAddr, CIS_IMR, tmpval5); \
		PRINTK(DEBUG_MSG, "CIS_IMR=0x%04x ", (unsigned int)tmpval5); \
		READ_PHYREG(pax88180_local->PhyAddr, CIS_ISR, tmpval5); \
		PRINTK(DEBUG_MSG, "CIS_ISR=0x%04x ", (unsigned int)tmpval5); \
		READ_PHYREG(pax88180_local->PhyAddr, CIS_AUX_CTRL_STATUS, tmpval5); \
		PRINTK(DEBUG_MSG, "CIS_AUX=0x%04x \n", (unsigned int)tmpval5); \
	} \
	READ_MACREG(RXCFG, tmpval5); \
	PRINTK(DEBUG_MSG, "RXCFG=0x%08lx ", tmpval5); \
	READ_MACREG(MACCFG0, tmpval5); \
	PRINTK(DEBUG_MSG, "MACCFG0=0x%08lx ", tmpval5); \
	READ_MACREG(MACCFG1, tmpval5); \
	PRINTK(DEBUG_MSG, "MACCFG1=0x%08lx ", tmpval5); \
	READ_MACREG(MACCFG2, tmpval5); \
	PRINTK(DEBUG_MSG, "MACCFG2=0x%08lx \n\n", tmpval5); \
}


/* Index to functions, as function prototypes. */
static int ax88180_probe(struct net_device *ndev);

static int ax88180_open(struct net_device *ndev);
static int ax88180_stop(struct net_device *ndev);
static int ax88180_start_xmit(struct sk_buff *skb, struct net_device *ndev);
static void ax88180_tx_timeout(struct net_device *ndev);
static struct net_device_stats *ax88180_get_stats(struct net_device *ndev);
static void ax88180_set_multicast_list(struct net_device *ndev);

static irqreturn_t ax88180_interrupt(int irq, void *ndev_id);

static int ax88180_initialization(struct net_device *ndev);
static void ax88180_PHY_initial(struct net_device *ndev);
static void ax88180_meida_config(struct net_device *ndev);
static void get_MarvellPHY_meida_mode(struct net_device *ndev);
static void get_CicadaPHY_meida_mode(struct net_device *ndev);
static void ax88180_rx_handler(struct net_device *ndev);
static void ax88180_tx_handler(struct net_device *ndev);

static int ax88180_ioctl(struct net_device *ndev, struct ifreq *rq, int cmd);
static int ax88180_ethtool_ioctl(struct net_device *ndev, void *useraddr);
static int mdio_read(struct net_device *ndev, int phy_id, int regaddr);
static void mdio_write(struct net_device *ndev, int phy_id, int regaddr, int regval);


/*
 * ===========================================================================
 * <<<<<<                        MODULE-ROUTINES                        >>>>>>
 * ===========================================================================
 */

MODULE_AUTHOR("Allan Chou <allan@asix.com.tw>");
MODULE_DESCRIPTION("ASIX AX88180 Non-PCI 16-bit Gigabit Ethernet Driver");
MODULE_LICENSE("GPL");

/*
 *****************************************************************************
 * AX88180 module mode driver optional parameters:
 * Syntax: insmod ax88180.o media=<media_type> jumbo=x
 *   media	Set media mode (0:auto, 1:100full, 2:100half, 3:10full 4:10half)
 *   jumbo	Enable/disable Jumbo frame (1=enable, 0=disable)(default is 0)
 *

 * example: modprobe ax88180  media=auto jumbo=0
 *
 *****************************************************************************
 */

module_param(media, int, 0);
module_param(jumbo, int, 0);
MODULE_PARM_DESC(media, "Media Mode(auto, 100full, 100half, 10full or 10half)");
MODULE_PARM_DESC(jumbo, "Jumbo Frame(1=enable, 0=disable");


/*
 * ===========================================================================
 * <<<<<<                        MAIN-ROUTINES                          >>>>>>
 * ===========================================================================
 */

/*
 *****************************************************************************
 * ax88180_probe()
 *
 * This is the entry routine for kernel mode driver. This routine will probe
 * the AX88180 device and allocate a (64K bytes + private data structure size)
 * memory space for AX88180 operation.
 *
 * AX88180 32-bit memory mapping:
 * ==============================
 * 0x0000~0x3FFF	RX buffer area
 * 0x4000~0xFBFF	TX buffer area
 * 0xFC00~0xFCFF	MAC registers area
 * 0xFD00~0xFFFF	Unused
 *
 * Return 0 on success.
 *****************************************************************************
 */
int ax88180_probe(struct net_device *ndev)
{
	struct ax88180_local *pax88180_local = netdev_priv(ndev);
	unsigned int phy_membase = ndev->base_addr;

	PRINTK(DRIVER_MSG, "%s", version);

	PRINTK(INIT_MSG, "ax88180: ax88180_probe beginning ..........\n");

	pax88180_local->Phy_MemBase = phy_membase;
	Log_MemBase = (unsigned int)ioremap(pax88180_local->Phy_MemBase, AX88180_MEMORY_SIZE);


	PRINTK(DRIVER_MSG, "ax88180: ndev =0x%p, pax88180_local=0x%p, Log_MemBase=0x%p\n",
	       ndev, (void *)pax88180_local, (void *)Log_MemBase);


	PRINTK(DRIVER_MSG, "ax88180: Allocate AX88180 at Phy_MemBase=0x%04x. (name=%s, IRQ=%d)\n"
	       , (unsigned int)pax88180_local->Phy_MemBase
	       , ndev->name, (unsigned int)ndev->irq);


	/* Initialize the Ethernet Device structure */
	ether_setup(ndev);

	pax88180_local->MediaMode = media;
	pax88180_local->JumboFlag = jumbo;
	pax88180_local->PhyAddr = MARVELL_88E1111_PHYADDR;
	pax88180_local->rx_buf = NULL;

	/* Declare ax88180 routines here */
	ndev->open		= ax88180_open;
	ndev->stop		= ax88180_stop;
	ndev->hard_start_xmit 	= ax88180_start_xmit;
	ndev->tx_timeout		= ax88180_tx_timeout;
	ndev->watchdog_timeo	= 5*HZ;
	ndev->get_stats		= ax88180_get_stats;
	ndev->set_multicast_list	= ax88180_set_multicast_list;
	ndev->do_ioctl		= ax88180_ioctl;


	dev_alloc_name(ndev, "eth%d");


	PRINTK(DRIVER_MSG, "ax88180: ax88180_probe end ..........\n");
	return 0;
}


/*
 *****************************************************************************
 * ax88180_open()
 *
 * Open/initialize the board.  This is called (in the current kernel)
 * sometime after booting when the 'ifconfig' program is run.
 *
 * This routine should set everything up anew at each open, even
 * registers that "should" only need to be set once at boot, so that
 * there is non-reboot way to recover if something goes wrong.
 *
 * AKPM: do we need to do any locking here?
 *
 *****************************************************************************
 */
static int ax88180_open(struct net_device *ndev)
{
	struct ax88180_local *pax88180_local = netdev_priv(ndev);
	unsigned short tmp_regval;
	int rtn = -ENODEV;

	PRINTK(INIT_MSG, "ax88180: ax88180_open beginning ..........\n");

	if (pax88180_local->rx_buf == NULL) {
		/* Try to allocate memory space for RX buffer */
		pax88180_local->rx_buf = kzalloc(MAX_RX_SIZE, GFP_KERNEL);
		if (pax88180_local->rx_buf == NULL) {
			PRINTK(ERROR_MSG, "ax88180: Fail to allocate a RX buffer space!\n");
      			PRINTK(INIT_MSG, "ax88180: ax88180_open fail end ..........\n");
			return -ENOMEM;
		}
	}

	/* Initial AX88180 registers here */
	rtn = ax88180_initialization(ndev);
	if (rtn) {
		/* Release allocated resource here */
		PRINTK(ERROR_MSG, "ax88180: Fail to initialize AX88180 controller!!\n");
		PRINTK(INIT_MSG, "ax88180: ax88180_open fail end ..........\n");
		return rtn;
	}

	/* Initial variables here */
	INIT_TXRX_VARIABLES;

	rtn = request_irq(ndev->irq
			  , &ax88180_interrupt, IRQF_TRIGGER_LOW
			  , ndev->name, ndev);

	if (rtn) {
	        /* Release allocated resource here */
	        PRINTK(ERROR_MSG, "ax88180: Failed to request IRQ (0x%x)\n"
		       , ndev->irq);
		PRINTK(INIT_MSG, "ax88180: ax88180_open fail end ..........\n");
		return rtn;
	}

	/* Enable AX88180 interrupt */
	ENABLE_INTERRUPT;

	/* Start AX88180 TX/RX functions */
	WRITE_MACREG(CMD, RXEN | TXEN | WAKEMOD);

	/* Check if there is any invalid interrupt status. If yes, clear it. */
	READ_MACREG(ISR, tmp_regval);
	PRINTK(INIT_MSG, "ax88180: The interrupt status = 0x%04x\n", tmp_regval);
	if (tmp_regval)
		WRITE_MACREG(ISR, tmp_regval);

	/* Display all AX88180 MAC and PHY registers onto console screen */
	/*DISPLAY_ALLMACREG;
	  DISPLAY_ALLPHYREG;*/

	/* Inform upper protocol to start sending packets */
	netif_start_queue(ndev);

	/* Driver initialization successful */
	PRINTK(DRIVER_MSG, "ax88180: name=%s, Phy_MemBase=0x%04x, IRQ=0x%x, media=%d, jumbo=%u\n",
			ndev->name, pax88180_local->Phy_MemBase, ndev->irq, media, jumbo);
	PRINTK(DRIVER_MSG, "ax88180: The AX88180 driver is loaded successfully.\n");

	PRINTK(INIT_MSG, "ax88180: ax88180_open end ..........\n");
	return 0;
}

/*
 *****************************************************************************
 * ax88180_stop()
 *
 * The inverse routine to ax88180_open().
 *
 *****************************************************************************
 */
static int ax88180_stop(struct net_device *ndev)
{
	struct ax88180_local *pax88180_local = netdev_priv(ndev);
	unsigned int flags;

	PRINTK(INIT_MSG, "ax88180: ax88180_stop beginning ..........\n");

	spin_lock_irqsave(&pax88180_local->lock, flags);

	DISABLE_INTERRUPT;

	/* Stop AX88180 TX/RX functions */
	WRITE_MACREG(CMD, 0);

	/* Inform upper layer to stop sending packets to device driver */
	if (netif_device_present(ndev)) {
		netif_stop_queue(ndev);
		netif_carrier_off(ndev);
	}

	/* Release interrupt */
	free_irq(ndev->irq, ndev);

	spin_unlock_irqrestore(&pax88180_local->lock, flags);

	/* Driver initialization successful */
	PRINTK(DRIVER_MSG
        , "ax88180: The AX88180 driver is unloaded successfully.\n");

	PRINTK(INIT_MSG, "ax88180: ax88180_stop end ..........\n");
	return 0;
}

static int ax88180_start_xmit(struct sk_buff *skb
			      , struct net_device *ndev)
{
	struct ax88180_local *pax88180_local = netdev_priv(ndev);
	unsigned char *txdata;
	unsigned int TXDES_addr;
	unsigned int txcmd_txdp, txbs_txdp;
	unsigned int tmp_data;
	int i;

	txdata = skb->data;

	PRINTK(TX_MSG, "ax88180: ax88180_start_xmit beginning ..........\n");

	/* Inform upper layer to stop sending packets to device driver */
	netif_stop_queue(ndev);

	pax88180_local->FirstTxDesc = pax88180_local->NextTxDesc;
	txbs_txdp = 1 << pax88180_local->FirstTxDesc;

	/* allan9 add to make sure TX machine is OK */
	i = 0;
	READ_MACREG(TXBS, tmp_data);

	PRINTK(TX_MSG
        , "ax88180: Checking available TXDP (TXBS=0x%04x)......\n"
	       , tmp_data);
	while (tmp_data & txbs_txdp) {
		pax88180_local->NextTxDesc++;
		pax88180_local->NextTxDesc &= TXDP_MASK;
		pax88180_local->FirstTxDesc = pax88180_local->NextTxDesc;
		txbs_txdp = 1 << pax88180_local->FirstTxDesc;
		READ_MACREG(TXBS, tmp_data);
		i++;

		if (i > 1000) {
			RESET_MAC;
			pax88180_local->NextTxDesc = TXDP0;
			pax88180_local->FirstTxDesc = pax88180_local->NextTxDesc;
			txbs_txdp = 1 << pax88180_local->FirstTxDesc;
			READ_MACREG(TXBS, tmp_data);
			i = 0;
			PRINTK(ERROR_MSG, "ax88180: No available TXDP!!\n");
		}
	}

	PRINTK(TX_MSG, "ax88180: TXDP%d is available, i=%d\n"
	       , (int)pax88180_local->FirstTxDesc, i);


	txcmd_txdp = pax88180_local->FirstTxDesc << 13;

	TXDES_addr = TXDES0 + (pax88180_local->FirstTxDesc << 2);

	WRITE_MACREG(TXCMD, txcmd_txdp | skb->len | TX_START_WRITE);

#ifdef CONFIG_AX88180_16BIT
	if (((u32)txdata & 0x01) == 0) {
		i = skb->len / 2;
		if (skb->len & 0x1)
			i++;

		if (i > 23) {
			dma_outsw(Log_MemBase + TXBUFFER_START, txdata, i);
		} else {
			outsw(Log_MemBase + TXBUFFER_START, txdata, i);
		}
	} else {
		for (i = 0; i < skb->len; i += 2) {
		  tmp_data =
			(unsigned short)*(txdata + i) +
			(unsigned short)(*(txdata + i + 1) << 8);
		  WRITE_TXBUF(tmp_data);
		}
	}
#else
	if (((u32)txdata & 0x3) == 0) {
		i = skb->len / 4;
		if (skb->len & 0x3)
			i++;

		if (i > 18) {
			dma_outsl(Log_MemBase + TXBUFFER_START, txdata, i);
		} else {
			outsl(Log_MemBase + TXBUFFER_START, txdata, i);
		}
	} else {
		for (i = 0; i < skb->len; i += 4) {
			tmp_data =
			    (unsigned int)*(txdata + i) +
			    (unsigned int)(*(txdata + i + 1) << 8) +
			    (unsigned int)(*(txdata + i + 2) << 16) +
			    (unsigned int)(*(txdata + i + 3) << 24);
			WRITE_TXBUF(tmp_data);
		}
	}
#endif

	WRITE_MACREG(TXCMD, txcmd_txdp | skb->len);
	WRITE_MACREG(TXBS, txbs_txdp);
	WRITE_MACREG(TXDES_addr, TXDPx_ENABLE | skb->len);

	pax88180_local->stats.tx_packets++;
	pax88180_local->stats.tx_bytes += skb->len;
	dev_kfree_skb(skb);
	ndev->trans_start = jiffies;

	if (pax88180_local->JumboFlag == ENABLE_JUMBO) {
		pax88180_local->NextTxDesc += 2;
	} else {
		pax88180_local->NextTxDesc++;
	}
	pax88180_local->NextTxDesc &= TXDP_MASK;

	PRINTK(TX_MSG, "ax88180: ax88180_start_xmit end ..........\n\n");
	return 0;
}


/*
 *****************************************************************************
 * ax88180_tx_timeout()
 *****************************************************************************
 */
static void ax88180_tx_timeout(struct net_device *ndev)
{
	struct ax88180_local *pax88180_local = netdev_priv(ndev);

	PRINTK(TX_MSG, "ax88180: ax88180_tx_timeout beginning ..........\n");

	RESET_MAC;
	INIT_TXRX_VARIABLES;

	/* Inform upper layer to send next queued packets now */
	ndev->trans_start = jiffies;
	netif_wake_queue(ndev);

	PRINTK(TX_MSG, "ax88180: ax88180_tx_timeout end ..........\n");
	return;
}


/*
 *****************************************************************************
 * ax88180_get_stats()
 *
 * Get the current statistics. This may be called with the card open or closed.
 *
 *****************************************************************************
 */
static struct net_device_stats *ax88180_get_stats(struct net_device *ndev)
{
	struct ax88180_local *pax88180_local = netdev_priv(ndev);
	unsigned int tmp_regval;
	unsigned int flags;

	PRINTK(OTHERS_MSG, "ax88180: ax88180_get_stats beginning..........\n");

	spin_lock_irqsave(&pax88180_local->lock, flags);

	/* Update the statistics counter here..... */
	READ_MACREG(RXIPCRCCNT, tmp_regval);
	pax88180_local->stats.rx_errors += tmp_regval;
	WRITE_MACREG(RXIPCRCCNT, 0);

	READ_MACREG(RXCRCCNT, tmp_regval);
	pax88180_local->stats.rx_errors += tmp_regval;
	WRITE_MACREG(RXCRCCNT, 0);

	READ_MACREG(TXFAILCNT, tmp_regval);
	pax88180_local->stats.tx_errors += tmp_regval;
	WRITE_MACREG(TXFAILCNT, 0);

	spin_unlock_irqrestore(&pax88180_local->lock, flags);

	PRINTK(OTHERS_MSG, "ax88180: ax88180_get_stats end ..........\n");
	return &pax88180_local->stats;
}


/*
 *****************************************************************************
 * ax88180_set_multicast_list()
 *****************************************************************************
 */
static void ax88180_set_multicast_list(struct net_device *ndev)
{
	struct ax88180_local *pax88180_local = netdev_priv(ndev);
	struct dev_mc_list *mc_list;
	unsigned int mc_hash_table[2];
	int crc_val,i;

	PRINTK(OTHERS_MSG
          , "ax88180: ax88180_set_multicast_list beginning ..........\n");

      	pax88180_local->RxFilterMode = DEFAULT_RXFILTER;

	if (ndev->flags & IFF_PROMISC) {
		pax88180_local->RxFilterMode |= RX_RXANY;
	} else if (ndev->flags & IFF_ALLMULTI) {
		pax88180_local->RxFilterMode |= RX_MULTICAST;
	} else if (ndev->flags & IFF_MULTICAST) {
		pax88180_local->RxFilterMode |= RX_MULTI_HASH;

		/* Handle Rx multicast hash table here */
		mc_hash_table[0] = mc_hash_table[1] = 0;
		for (i = 0, mc_list = ndev->mc_list;
		     (mc_list != NULL) && (i < ndev->mc_count);
		     i++, mc_list = mc_list->next) {

			crc_val = ether_crc(ETH_ALEN, mc_list->dmi_addr);
			set_bit(crc_val >> 26, mc_hash_table);
		}

		WRITE_MACREG(HASHTAB0, (unsigned int)mc_hash_table[0]);
		WRITE_MACREG(HASHTAB1, (unsigned int)(mc_hash_table[0] >> 16));
		WRITE_MACREG(HASHTAB2, (unsigned int)mc_hash_table[1]);
		WRITE_MACREG(HASHTAB3, (unsigned int)(mc_hash_table[1] >> 16));
	}

	WRITE_MACREG(RXFILTER, pax88180_local->RxFilterMode);

	PRINTK(OTHERS_MSG
	       , "ax88180: ax88180_set_multicast_list end ..........\n");
	return;
}

/*
 *****************************************************************************
 * ax88180_interrupt()
 *
 * Handle the network interface interrupts.
 *
 *****************************************************************************
 */
static irqreturn_t ax88180_interrupt(int irq, void *ndev_id)
{

	struct net_device *ndev = ndev_id;
	struct ax88180_local *pax88180_local = netdev_priv(ndev);
	unsigned int ISR_Status;
	unsigned int rxcurt_ptr, rxbound_ptr;
	unsigned int bmsr_val;
	unsigned int tmp_regval;
	int i;

	/* Read and check interrupt status here...... */
	READ_MACREG(ISR, ISR_Status);
	if ( (ISR_Status == 0) || (ISR_Status & ~DEFAULT_IMR) ) {
		PRINTK(WARNING_MSG, "ax88180: Not our interrupt!!\n");
		return IRQ_RETVAL(0);
	}

	PRINTK(INT_MSG, "ax88180: ax88180_interrupt beginning ..........\n");

	/* Disable AX88180 interrupt */
	DISABLE_INTERRUPT;

	/* Clear the interrupt status */
	WRITE_MACREG(ISR, ISR_Status);
	PRINTK(INT_MSG
	       , "ax88180: The interrupt status = 0x%04x\n", ISR_Status);

	/* Handle AX88180 interrupt events */
	if (ISR_Status & ISR_WATCHDOG) {
		PRINTK(DRIVER_MSG
		       , "ax88180: Watchdog Timer interrupt (ISR = 0x%04x)\n"
		       , ISR_Status);
	}

	if (ISR_Status & ISR_RX) {
		ax88180_rx_handler(ndev);
	}

	if (ISR_Status & ISR_TX) {
		ax88180_tx_handler(ndev);
	}

	if (ISR_Status & ISR_RXBUFFOVR) {

		pax88180_local->rxbuf_overflow_count++;
		pax88180_local->stats.rx_fifo_errors++;

		READ_MACREG(RXCURT, rxcurt_ptr);
		READ_MACREG(RXBOUND, rxbound_ptr);

		PRINTK(ERROR_MSG, "ax88180: RX Buffer overflow!!"
			"(count=%d, RXBOUND=0x%04x, RXCURT=0x%04x)\n"
		       , (int)pax88180_local->rxbuf_overflow_count
		       , rxbound_ptr, rxcurt_ptr);

		PRINTK(ERROR_MSG, "ax88180: The interrupt status = 0x%04x\n"
		       , ISR_Status);

		if (pax88180_local->rxbuf_overflow_count > 10) {
		        RESET_MAC;
			INIT_TXRX_VARIABLES;
		}
	}

	if (ISR_Status & ISR_PHY) {

		/* Read ISR register once to clear Marvell PHY interrupt bit */
		READ_PHYREG(pax88180_local->PhyAddr, M88_ISR, tmp_regval);

		/* Waiting 200 msecs for PHY link stable */
		for (i = 0; i < 200; i++) {
			READ_PHYREG(pax88180_local->PhyAddr, BMSR, bmsr_val);
			if (bmsr_val & LINKOK) {
				break;
			}
			mdelay(1);
		}

		if (bmsr_val & LINKOK) {
			PRINTK(WARNING_MSG
			       , "ax88180: The cable is connected.\n");
			netif_carrier_on(ndev);

			if (pax88180_local->ForceMedia == AUTO_MEDIA)
				ax88180_meida_config(ndev);

			/* DISPLAY_ALLPHYREG; */
		} else {
			PRINTK(WARNING_MSG
			       , "ax88180: The cable is disconnected.\n");
			netif_carrier_off(ndev);
			/* DISPLAY_ALLPHYREG; */
		}
	}

	/* Enable AX88180 interrupt */
	ENABLE_INTERRUPT;

	PRINTK(INT_MSG, "ax88180: ax88180_interrupt end ..........\n\n");
	return IRQ_HANDLED;
}

/*
 *****************************************************************************
 * ax88180_initialization()
 *****************************************************************************
 */
static int ax88180_initialization(struct net_device *ndev)
{
	struct ax88180_local *pax88180_local = netdev_priv(ndev);
	unsigned int macid0_val, macid1_val, macid2_val;
	unsigned int tmp_regval;
	int i;

	PRINTK(INIT_MSG, "ax88180: ax88180_initialization beginning ..........\n");

#ifdef CONFIG_AX88180_16BIT
	/* Set up the system in 16 bit mode */
	WRITE_MACREG(0xDD00, 0);
	WRITE_MACREG(0xDD06, 0x10);
	WRITE_MACREG(0xDD00, 1);
#endif

	RESET_MAC;

	/* Reload MAC address from EEPROM */
	WRITE_MACREG(PROMCTRL, RELOAD_EEPROM);
	for (i = 0; i < 500; i++) {
		READ_MACREG(PROMCTRL, tmp_regval);
		if ((tmp_regval & RELOAD_EEPROM) == 0)
			break;
		mdelay(1);
	}

	/* Disable AX88180 interrupt */
	DISABLE_INTERRUPT;

	/* Disable AX88180 TX/RX functions */
	WRITE_MACREG(CMD, WAKEMOD);

	/* Get MAC addresses */
	READ_MACREG(MACID0, macid0_val);
	READ_MACREG(MACID1, macid1_val);
	READ_MACREG(MACID2, macid2_val);
	if ((macid0_val | macid1_val | macid2_val) != 0) {
		ndev->dev_addr[0] = (unsigned char)macid0_val;
		ndev->dev_addr[1] = (unsigned char)(macid0_val >> 8);
		ndev->dev_addr[2] = (unsigned char)macid1_val;
		ndev->dev_addr[3] = (unsigned char)(macid1_val >> 8);
		ndev->dev_addr[4] = (unsigned char)macid2_val;
		ndev->dev_addr[5] = (unsigned char)(macid2_val >> 8);
	} else {
		/* No EEPROM found!! Set a default MAC address. */
		/* The driver designer should assign a legal MAC address here. */
		ndev->dev_addr[0] = 0x00;
		ndev->dev_addr[1] = 0x12;
		ndev->dev_addr[2] = 0x34;
		ndev->dev_addr[3] = 0x56;
		ndev->dev_addr[4] = 0x78;
		ndev->dev_addr[5] = 0x9a;

		macid0_val = (ndev->dev_addr[1] << 8) + ndev->dev_addr[0];
		macid1_val = (ndev->dev_addr[3] << 8) + ndev->dev_addr[2];
		macid2_val = (ndev->dev_addr[5] << 8) + ndev->dev_addr[4];
		WRITE_MACREG(MACID0, macid0_val);
		WRITE_MACREG(MACID1, macid1_val);
		WRITE_MACREG(MACID2, macid2_val);
	}

	/* Print the MAC address */
	PRINTK(DRIVER_MSG, "ax88180: The MAC address is");
	for (i = 0; i < ETH_ALEN; i++)
		PRINTK(DRIVER_MSG, "%c%02x", i ? ':' : ' ', ndev->dev_addr[i]);
	PRINTK(DRIVER_MSG, "\n");

	/* Initial PHY registers */
	ax88180_PHY_initial(ndev);

	/* Configure MAC media mode registers */
	ax88180_meida_config(ndev);


	/* Initial MII interface information for ethtool ioctl */
	pax88180_local->mii_if.dev = ndev;
	pax88180_local->mii_if.phy_id = pax88180_local->PhyAddr;
	pax88180_local->mii_if.phy_id_mask = 0x1F;
	pax88180_local->mii_if.reg_num_mask = 0x1F;
	pax88180_local->mii_if.mdio_read = mdio_read;
	pax88180_local->mii_if.mdio_write = mdio_write;
	pax88180_local->mii_if.force_media = pax88180_local->ForceMedia;
	pax88180_local->mii_if.full_duplex = pax88180_local->LineSpeed;
	pax88180_local->mii_if.force_media = pax88180_local->DuplexMode;

	WRITE_MACREG(RXFILTER, DEFAULT_RXFILTER);

	PRINTK(INIT_MSG, "ax88180: ax88180_initialization end ..........\n");
	return 0;
}


/*
 *****************************************************************************
 * ax88180_PHY_initial()
 *
 * Initialize PHY registers.
 *
 *****************************************************************************
 */
static void ax88180_PHY_initial(struct net_device *ndev)
{
	struct ax88180_local *pax88180_local = netdev_priv(ndev);
	unsigned int bmcr_val, anar_val, bmsr_val;
	unsigned int aux_1000_ctrl;
	unsigned int tmp_regval;
	unsigned int i;

	PRINTK(INIT_MSG, "ax88180: ax88180_PHY_initial beginning ..........\n");

	/* Check avaliable PHY chipset  */
	pax88180_local->PhyAddr = MARVELL_88E1111_PHYADDR;
	READ_PHYREG(pax88180_local->PhyAddr, PHYIDR0, pax88180_local->PhyID0);
	if (pax88180_local->PhyID0 == MARVELL_88E1111_PHYIDR0) {
		PRINTK(DRIVER_MSG, "ax88180: Found Marvell 88E1111 PHY chipset. (PHY Addr=0x%x)\n",
			(unsigned int)pax88180_local->PhyAddr);
		READ_PHYREG(pax88180_local->PhyAddr, M88_EXT_SSR, tmp_regval);
		if ((tmp_regval & HWCFG_MODE_MASK) == RGMII_COPPER_MODE) {
			WRITE_PHYREG(pax88180_local->PhyAddr, M88_EXT_SCR, DEFAULT_EXT_SCR);
			RESET_PHY;
			WRITE_PHYREG(pax88180_local->PhyAddr, M88_IER, LINK_CHANGE_INT);
		}
	} else {
		pax88180_local->PhyAddr = CICADA_CIS8201_PHYADDR;
		READ_PHYREG(pax88180_local->PhyAddr, PHYIDR0, pax88180_local->PhyID0);
		if (pax88180_local->PhyID0 == CICADA_CIS8201_PHYIDR0) {
			PRINTK(DRIVER_MSG, "ax88180: Found CICADA CIS8201 PHY chipset. (PHY Addr=0x%x)\n",
				(unsigned int)pax88180_local->PhyAddr);
			WRITE_PHYREG(pax88180_local->PhyAddr, CIS_IMR, (CIS_INT_ENABLE | LINK_CHANGE_INT));

			/* Set CIS_SMI_PRIORITY bit before force the media mode  */
			READ_PHYREG(pax88180_local->PhyAddr, CIS_AUX_CTRL_STATUS, tmp_regval);
			tmp_regval &= ~CIS_SMI_PRIORITY;
			if (pax88180_local->MediaMode != MEDIA_AUTO)
				tmp_regval |= CIS_SMI_PRIORITY;
			WRITE_PHYREG(pax88180_local->PhyAddr, CIS_AUX_CTRL_STATUS, tmp_regval);
		} else {
			PRINTK(ERROR_MSG, "ax88180: Unknown PHY chipset!!\n");
		}
	}

	PRINTK(INIT_MSG, "ax88180: PHY_Addr=0x%08lx, PHY_ID=0x%04x, media=%d\n",
		pax88180_local->PhyAddr, (unsigned int)pax88180_local->PhyID0, media);

	switch (pax88180_local->MediaMode) {
	default:
	case MEDIA_AUTO:
		PRINTK(INIT_MSG, "ax88180: The meida mode is autosense.\n");
		pax88180_local->ForceMedia = AUTO_MEDIA;
		aux_1000_ctrl = DEFAULT_AUX_1000_CTRL;
		anar_val = (ANAR_PAUSE | ANAR_100FULL | ANAR_100HALF | ANAR_10FULL | ANAR_10HALF | ANAR_8023BIT);
		break;

	case MEDIA_100FULL:
		PRINTK(INIT_MSG, "ax88180: The meida mode is forced to 100full.\n");
		pax88180_local->ForceMedia = FORCE_MEDIA;
		aux_1000_ctrl = 0;
		anar_val = (ANAR_PAUSE | ANAR_100FULL | ANAR_8023BIT);
		break;

	case MEDIA_100HALF:
		PRINTK(INIT_MSG, "ax88180: The meida mode is forced to 100half.\n");
		pax88180_local->ForceMedia = FORCE_MEDIA;
		aux_1000_ctrl = 0;
		anar_val = (ANAR_100HALF | ANAR_8023BIT);
		break;

	case MEDIA_10FULL:
		PRINTK(INIT_MSG, "ax88180: The meida mode is forced to 10full.\n");
		pax88180_local->ForceMedia = FORCE_MEDIA;
		aux_1000_ctrl = 0;
		anar_val = (ANAR_PAUSE | ANAR_10FULL | ANAR_8023BIT);
		break;

	case MEDIA_10HALF:
		PRINTK(INIT_MSG, "ax88180: The meida mode is forced to 10half.\n");
		pax88180_local->ForceMedia = FORCE_MEDIA;
		aux_1000_ctrl = 0;
		anar_val = (ANAR_10HALF | ANAR_8023BIT);
		break;
	}
	WRITE_PHYREG(pax88180_local->PhyAddr, AUX_1000_CTRL, aux_1000_ctrl);
	WRITE_PHYREG(pax88180_local->PhyAddr, ANAR, anar_val);

	/* Enable and restart auto-negotiation operation */
	bmcr_val = (AUTONEG_EN | RESTART_AUTONEG);
	WRITE_PHYREG(pax88180_local->PhyAddr, BMCR, bmcr_val);

	/* Waiting 5 secs for PHY link stable */
	PRINTK(DRIVER_MSG, "ax88180: Waiting for auto-negotiation completion......\n");
	for (i = 0; i < 5000; i++) {
		READ_PHYREG(pax88180_local->PhyAddr, BMSR, bmsr_val);
		if (bmsr_val & LINKOK) {
			break;
		}
		mdelay(1);
	}

	PRINTK(INIT_MSG, "ax88180: ax88180_PHY_initial end ..........\n");
	return;
}


/*
 *****************************************************************************
 * ax88180_meida_config()
 *
 * Configure MAC registers (RXCFG, MACCFG0, MACCFG1) to match the real PHY media mode.
 *
 *****************************************************************************
 */
static void ax88180_meida_config(struct net_device *ndev)
{
	struct ax88180_local *pax88180_local = netdev_priv(ndev);
	unsigned int bmcr_val, bmsr_val;
	unsigned int rxcfg_val, maccfg0_val, maccfg1_val;
	int i;

	PRINTK(INIT_MSG, "ax88180: ax88180_meida_config beginning ..........\n");

	/* Waiting 200 msecs for PHY link stable */
	for (i = 0; i < 200; i++) {
		READ_PHYREG(pax88180_local->PhyAddr, BMSR, bmsr_val);
		if (bmsr_val & LINKOK) {
			break;
		}
		mdelay(1);
	}

	READ_PHYREG(pax88180_local->PhyAddr, BMSR, bmsr_val);
	if (bmsr_val & LINKOK) {
		READ_PHYREG(pax88180_local->PhyAddr, BMCR, bmcr_val);
		if (bmcr_val & AUTONEG_EN) {
			/* Waiting for Auto-negotiation completion */
			PRINTK(INIT_MSG, "ax88180: Auto-negotiation is enabled. Waiting for NWay completion.....\n");

			for (i = 0; i < 5000; i++) {
				if (bmsr_val & AUTONEG_COMPLETE) {
					break;
				}
				mdelay(1);
				READ_PHYREG(pax88180_local->PhyAddr, BMSR, bmsr_val);
			}
			if (i >= 5000)
				PRINTK(INIT_MSG, "ax88180: Auto-negotiation is NOT completed!!\n");
		} else
			PRINTK(INIT_MSG, "ax88180: Auto-negotiation is disabled.\n");

		PRINTK(DEBUG_MSG, "ax88180: BMCR=0x%04x, BMSR=0x%04x\n",
			(unsigned int)bmcr_val, (unsigned int)bmsr_val);

		/* Get real media mode here */
		if (pax88180_local->PhyID0 == MARVELL_88E1111_PHYIDR0) {
			get_MarvellPHY_meida_mode(ndev);
		} else if (pax88180_local->PhyID0 == CICADA_CIS8201_PHYIDR0) {
			get_CicadaPHY_meida_mode(ndev);
		} else {
			pax88180_local->RealMediaMode = MEDIA_1000FULL;
		}

		switch (pax88180_local->RealMediaMode) {
		default:
		case MEDIA_1000FULL:
			PRINTK(DRIVER_MSG, "ax88180: Set to 1000Mbps Full-duplex mode.\n");
			pax88180_local->LineSpeed = SPEED_1000;
			pax88180_local->DuplexMode = DUPLEX_FULL;

			rxcfg_val = RXFLOW_ENABLE | DEFAULT_RXCFG;
			maccfg0_val = TXFLOW_ENABLE | DEFAULT_MACCFG0;
			maccfg1_val = GIGA_MODE_EN | RXFLOW_EN | FULLDUPLEX | DEFAULT_MACCFG1;

			if (pax88180_local->JumboFlag == ENABLE_JUMBO) {
				PRINTK(DRIVER_MSG, "ax88180: Enable Jumbo Frame function.\n");
				maccfg1_val |= RXJUMBO_EN | JUMBO_LEN_15K;
				ndev->mtu = MAX_JUMBO_MTU;
			}
			break;

		case MEDIA_1000HALF:
			PRINTK(DRIVER_MSG, "ax88180: Set to 1000Mbps Half-duplex mode.\n");
			pax88180_local->LineSpeed = SPEED_1000;
			pax88180_local->DuplexMode = DUPLEX_HALF;

			rxcfg_val = DEFAULT_RXCFG;
			maccfg0_val = DEFAULT_MACCFG0;
			maccfg1_val = GIGA_MODE_EN | DEFAULT_MACCFG1;
			ndev->mtu = DEFAULT_ETH_MTU;
			break;

		case MEDIA_100FULL:
			PRINTK(DRIVER_MSG, "ax88180: Set to 100Mbps Full-duplex mode.\n");
			pax88180_local->LineSpeed = SPEED_100;
			pax88180_local->DuplexMode = DUPLEX_FULL;

			rxcfg_val = RXFLOW_ENABLE | DEFAULT_RXCFG;
			maccfg0_val = SPEED100 | TXFLOW_ENABLE | DEFAULT_MACCFG0;
			maccfg1_val = RXFLOW_EN | FULLDUPLEX | DEFAULT_MACCFG1;
			ndev->mtu = DEFAULT_ETH_MTU;
			break;

		case MEDIA_100HALF:
			PRINTK(DRIVER_MSG, "ax88180: Set to 100Mbps Half-duplex mode.\n");
			pax88180_local->LineSpeed = SPEED_100;
			pax88180_local->DuplexMode = DUPLEX_HALF;

			rxcfg_val = DEFAULT_RXCFG;
			maccfg0_val = SPEED100 | DEFAULT_MACCFG0;
			maccfg1_val = DEFAULT_MACCFG1;
			ndev->mtu = DEFAULT_ETH_MTU;
			break;

		case MEDIA_10FULL:
			PRINTK(DRIVER_MSG, "ax88180: Set to 10Mbps Full-duplex mode.\n");
			pax88180_local->LineSpeed = SPEED_10;
			pax88180_local->DuplexMode = DUPLEX_FULL;

			rxcfg_val = RXFLOW_ENABLE | DEFAULT_RXCFG;
			maccfg0_val = TXFLOW_ENABLE | DEFAULT_MACCFG0;
			maccfg1_val = RXFLOW_EN | FULLDUPLEX | DEFAULT_MACCFG1;
			ndev->mtu = DEFAULT_ETH_MTU;
			break;

		case MEDIA_10HALF:
			PRINTK(DRIVER_MSG, "ax88180: Set to 10Mbps Half-duplex mode.\n");
			pax88180_local->LineSpeed = SPEED_10;
			pax88180_local->DuplexMode = DUPLEX_HALF;

			rxcfg_val = DEFAULT_RXCFG;
			maccfg0_val = DEFAULT_MACCFG0;
			maccfg1_val = DEFAULT_MACCFG1;
			ndev->mtu = DEFAULT_ETH_MTU;
			break;
		}
	} else {
		PRINTK(INIT_MSG, "ax88180: The cable is disconnected!!\n");
		/* Set to default media mode (1000FULL) */
		pax88180_local->LineSpeed = SPEED_1000;
		pax88180_local->DuplexMode = DUPLEX_FULL;

		rxcfg_val = RXFLOW_ENABLE | DEFAULT_RXCFG;
		maccfg0_val = TXFLOW_ENABLE | DEFAULT_MACCFG0;
		maccfg1_val = GIGA_MODE_EN | RXFLOW_EN | FULLDUPLEX | DEFAULT_MACCFG1;

		if (pax88180_local->JumboFlag == ENABLE_JUMBO) {
			maccfg1_val |= RXJUMBO_EN | JUMBO_LEN_15K;
			ndev->mtu = MAX_JUMBO_MTU;
		}
	}

	WRITE_MACREG(RXCFG, rxcfg_val);
	WRITE_MACREG(MACCFG0, maccfg0_val);
	WRITE_MACREG(MACCFG1, maccfg1_val);

	PRINTK(INIT_MSG, "ax88180: ax88180_meida_config end ..........\n");
	return;
}


/*
 *****************************************************************************
 * get_MarvellPHY_meida_mode()
 *
 * Get real media mode of Marvell 88E1111 PHY.
 *
 *****************************************************************************
 */
static void get_MarvellPHY_meida_mode(struct net_device *ndev)
{
	struct ax88180_local *pax88180_local = netdev_priv(ndev);
	unsigned int m88_ssr;
	int i;

	PRINTK(INIT_MSG, "ax88180: get_MarvellPHY_meida_mode beginning ..........\n");

	/* Get the real media mode */
	for (i = 0; i < 200; i++) {
		READ_PHYREG(pax88180_local->PhyAddr, M88_SSR, m88_ssr);
		if (m88_ssr & SSR_MEDIA_RESOLVED_OK) {
			break;
		}
		mdelay(1);
	}

	READ_PHYREG(pax88180_local->PhyAddr, M88_SSR, m88_ssr);
	switch (m88_ssr & SSR_MEDIA_MASK) {
	default:
	case SSR_1000FULL:
		pax88180_local->RealMediaMode = MEDIA_1000FULL;
		break;

	case SSR_1000HALF:
		pax88180_local->RealMediaMode = MEDIA_1000HALF;
		break;

	case SSR_100FULL:
		pax88180_local->RealMediaMode = MEDIA_100FULL;
		break;

	case SSR_100HALF:
		pax88180_local->RealMediaMode = MEDIA_100HALF;
		break;

	case SSR_10FULL:
		pax88180_local->RealMediaMode = MEDIA_10FULL;
		break;

	case SSR_10HALF:
		pax88180_local->RealMediaMode = MEDIA_10HALF;
		break;
	}

	PRINTK(INIT_MSG, "ax88180: get_MarvellPHY_meida_mode end ..........\n");
	return;
}


/*
 *****************************************************************************
 * get_CicadaPHY_meida_mode()
 *
 * Get real media mode of CICADA CIS8201 PHY.
 *
 *****************************************************************************
 */
static void get_CicadaPHY_meida_mode(struct net_device *ndev)
{
	struct ax88180_local *pax88180_local = netdev_priv(ndev);
	unsigned int tmp_regval;

	PRINTK(INIT_MSG, "ax88180: get_CicadaPHY_meida_mode beginning ..........\n");

	READ_PHYREG(pax88180_local->PhyAddr, CIS_AUX_CTRL_STATUS, tmp_regval);
	switch (tmp_regval & CIS_MEDIA_MASK) {
	default:
	case CIS_1000FULL:
		pax88180_local->RealMediaMode = MEDIA_1000FULL;
		break;

	case CIS_1000HALF:
		pax88180_local->RealMediaMode = MEDIA_1000HALF;
		break;

	case CIS_100FULL:
		pax88180_local->RealMediaMode = MEDIA_100FULL;
		break;

	case CIS_100HALF:
		pax88180_local->RealMediaMode = MEDIA_100HALF;
		break;

	case CIS_10FULL:
		pax88180_local->RealMediaMode = MEDIA_10FULL;
		break;

	case CIS_10HALF:
		pax88180_local->RealMediaMode = MEDIA_10HALF;
		break;
	}

	PRINTK(INIT_MSG, "ax88180: get_CicadaPHY_meida_mode end ..........\n");
	return;
}


/*
 *****************************************************************************
 * ax88180_rx_handler()
 *
 * Handle packets received completion interrupt event.
 *
 *****************************************************************************
 */
static void ax88180_rx_handler(struct net_device *ndev)
{
	struct ax88180_local *pax88180_local = netdev_priv(ndev);
	struct sk_buff *skb;
	unsigned char *rxdata;
	unsigned int tmp_data;
	unsigned int rx_packet_len;
	unsigned int data_size;
	unsigned int sword_count, byte_count;
	unsigned int rxcurt_ptr, rxbound_ptr, next_ptr;
	int j;

	PRINTK(RX_MSG, "ax88180: ax88180_rx_handler beginning ..........\n");

	READ_MACREG(RXCURT, rxcurt_ptr);
	READ_MACREG(RXBOUND, rxbound_ptr);
	next_ptr = (rxbound_ptr + 1) & RX_PAGE_NUM_MASK;

	PRINTK(RX_MSG, "ax88180: RX original RXBOUND=0x%04x, RXCURT=0x%04x\n"
	       , rxbound_ptr, rxcurt_ptr);

	while (next_ptr != rxcurt_ptr) {
		START_READ_RXBUFF;

		READ_RXBUF(rx_packet_len);

		PRINTK(RX_MSG, "ax88180: Rx packet length (len=0x%04x)\n"
		       , rx_packet_len);
 	  	if ( (rx_packet_len == 0) || (rx_packet_len > MAX_RX_SIZE) ) {
			pax88180_local->stats.rx_errors++;
			STOP_READ_RXBUFF;

			RESET_MAC;
			PRINTK(ERROR_MSG
			, "ax88180: Invalid Rx packet length!! (len=0x%04x)\n"
			       , rx_packet_len);
			PRINTK(ERROR_MSG
			, "ax88180: RX RXBOUND=0x%04x, RXCURT=0x%04x\n"
			       , rxbound_ptr, rxcurt_ptr);
			PRINTK(RX_MSG
                        , "ax88180: ax88180_rx_handler fail end ..........\n");
			return;
		}

		data_size = (unsigned int)rx_packet_len;
		rxbound_ptr += (((data_size + 0xF) & 0xFFF0) >> 4) + 1;
		rxbound_ptr &= RX_PAGE_NUM_MASK;

		skb = dev_alloc_skb(data_size + NET_IP_ALIGN);
		if (skb == NULL) {
			pax88180_local->stats.rx_dropped++;
			STOP_READ_RXBUFF;

			PRINTK(ERROR_MSG, "ax88180: No available memory space. Dropping RX packets!!\n");
			PRINTK(RX_MSG, "ax88180: ax88180_rx_handler fail end ..........\n");
			return;
		}

		skb->data = skb->head;
		skb->tail = skb->head;
		skb_reserve(skb, NET_IP_ALIGN);
		skb->dev = ndev;
		rxdata = skb_put(skb, data_size);

#ifdef CONFIG_AX88180_16BIT
		sword_count = data_size >> 1;	/* Divide by 2 for 16-bit words */
		byte_count = data_size & 0x1;
		PRINTK(RX_MSG, "swords %d bytes %d [\n", sword_count, byte_count);

		if (sword_count > 24)
			dma_insw(Log_MemBase + RXBUFFER_START, rxdata, sword_count);
		else
			insw(Log_MemBase + RXBUFFER_START, rxdata, sword_count);

		if (byte_count != 0) {
			READ_RXBUF(tmp_data);
			for (j = 0; j < byte_count; j++) {
				*(rxdata + (sword_count * 2) + j) =
					(unsigned char)(tmp_data >> (j * 8));
			}
		}
#else
		sword_count = data_size >> 2;	/* Divide by 4 for 32-bit words */
		byte_count = data_size & 0x3;
		PRINTK(RX_MSG, "swords %d bytes %d [\n", sword_count, byte_count);

#ifdef CONFIG_BLACKFIN
		insl_16(Log_MemBase + RXBUFFER_START, rxdata, sword_count);
#else
		insl(Log_MemBase + RXBUFFER_START, rxdata, sword_count);
#endif
		if (byte_count != 0) {
			READ_RXBUF(tmp_data);
			for (j = 0; j < byte_count; j++) {
				*(rxdata + (sword_count * 4) + j) =
					(unsigned char)(tmp_data >> (j * 8));
			}
		}
#endif

		STOP_READ_RXBUFF;

		PRINTK(RX_MSG, " received data (bytes) byte_count %d [\n"
		       , byte_count);

		skb->protocol = eth_type_trans(skb, ndev);
		netif_rx(skb);
		ndev->last_rx = jiffies;
		pax88180_local->stats.rx_packets++;
		pax88180_local->stats.rx_bytes += data_size;

		WRITE_MACREG(RXBOUND, rxbound_ptr);

		PRINTK(DEBUG_MSG, "ax88180: Rx data size = 0x%x\n", data_size);

		READ_MACREG(RXCURT, rxcurt_ptr);
		READ_MACREG(RXBOUND, rxbound_ptr);
		next_ptr = (rxbound_ptr + 1) & RX_PAGE_NUM_MASK;

		PRINTK(RX_MSG, "ax88180: RX updated RXBOUND=0x%04x, RXCURT=0x%04x\n",
			rxbound_ptr, rxcurt_ptr);
	}

	if (pax88180_local->rxbuf_overflow_count > 0) {
		pax88180_local->rxbuf_overflow_count--;
	}


	PRINTK(RX_MSG, "ax88180: ax88180_rx_handler end ..........\n");
	return;
}


/*
 *****************************************************************************
 * ax88180_tx_handler()
 *
 * Handle packets transmitted completion interrupt event.
 *
 *****************************************************************************
 */
static void ax88180_tx_handler(struct net_device *ndev)
{
	PRINTK(TX_MSG, "ax88180: ax88180_tx_handler beginning ..........\n");

	/* Inform upper layer to send next queued packets now */
	netif_wake_queue(ndev);

	PRINTK(TX_MSG, "ax88180: ax88180_tx_handler end ..........\n");
	return;
}



/*
 *****************************************************************************
 * ax88180_ioctl()
 *
 * Handle the ioctl commands like ethtool.
 *
 *****************************************************************************
 */
static int ax88180_ioctl(struct net_device *ndev, struct ifreq *rq, int cmd)
{
	PRINTK(OTHERS_MSG, "ax88180: ax88180_ioctl beginning ..........\n");

	switch(cmd) {
	case SIOCETHTOOL:
		return ax88180_ethtool_ioctl(ndev, (void *)rq->ifr_data);

	default:
		return -EOPNOTSUPP;
	}

	PRINTK(OTHERS_MSG, "ax88180: ax88180_ioctl end ..........\n");
}


/*
 *****************************************************************************
 * ax88180_ethtool_ioctl()
 *
 * Handle the ethtool ioctl command.
 *
 *****************************************************************************
 */
static int ax88180_ethtool_ioctl(struct net_device *ndev, void *useraddr)
{
	struct ax88180_local *pax88180_local = netdev_priv(ndev);
	u32 ethcmd;

	PRINTK(OTHERS_MSG, "ax88180: ax88180_ethtool_ioctl beginning ..........\n");

	if (copy_from_user(&ethcmd, useraddr, sizeof(ethcmd)))
		return -EFAULT;

	switch (ethcmd) {
	case ETHTOOL_GDRVINFO: {
		struct ethtool_drvinfo info = { ETHTOOL_GDRVINFO };
		strcpy(info.driver, DRV_NAME);
		strcpy(info.version, DRV_VERSION);
		if (copy_to_user(useraddr, &info, sizeof(info)))
			return -EFAULT;
		return 0;
	}

	case ETHTOOL_GSET: {
		struct ethtool_cmd ecmd = { ETHTOOL_GSET };
		spin_lock_irq(&pax88180_local->lock);
		mii_ethtool_gset(&pax88180_local->mii_if, &ecmd);
		spin_unlock_irq(&pax88180_local->lock);
		if (copy_to_user(useraddr, &ecmd, sizeof(ecmd)))
			return -EFAULT;
		return 0;
	}

	case ETHTOOL_SSET: {
		int r;
		struct ethtool_cmd ecmd;
		if (copy_from_user(&ecmd, useraddr, sizeof(ecmd)))
			return -EFAULT;
		spin_lock_irq(&pax88180_local->lock);
		r = mii_ethtool_sset(&pax88180_local->mii_if, &ecmd);
		spin_unlock_irq(&pax88180_local->lock);
		return r;
	}

	case ETHTOOL_NWAY_RST: {
		return mii_nway_restart(&pax88180_local->mii_if);
	}

	case ETHTOOL_GLINK: {
		struct ethtool_value edata = {ETHTOOL_GLINK};
		edata.data = mii_link_ok(&pax88180_local->mii_if);
		if (copy_to_user(useraddr, &edata, sizeof(edata)))
			return -EFAULT;
		return 0;
	}

	default:
		break;
	}

	PRINTK(OTHERS_MSG, "ax88180: ax88180_ethtool_ioctl end ..........\n");
	return -EOPNOTSUPP;
}


/*
 *****************************************************************************
 * mdio_read()
 *
 *****************************************************************************
 */
static int mdio_read(struct net_device *ndev, int phy_id, int regaddr)
{
	unsigned int regval;

	READ_PHYREG(phy_id, regaddr, regval);
	PRINTK(DEBUG_MSG, "ax88180: mdio_read regval=0x%04x\n", regval);
	return regval;
}


/*
 *****************************************************************************
 * mdio_write()
 *
 *****************************************************************************
 */
static void mdio_write(struct net_device *ndev, int phy_id
		       , int regaddr, int regval)
{
	WRITE_PHYREG(phy_id, (unsigned int)regaddr, (unsigned int)regval);

	return;
}
/*
 *****************************************************************************
 * Platform Stuff
 *
 *****************************************************************************
 */


static int ax88180_drv_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct net_device *ndev;

	PRINTK(INIT_MSG, "ax88180: ax88180_init_module beginning ..........\n");

	ndev = alloc_etherdev(sizeof(struct ax88180_local));
	if (!ndev) {
		printk(KERN_ERR"%s: could not allocate device.\n", DRV_NAME);
		return -ENOMEM;
	}
	SET_MODULE_OWNER(ndev);
	SET_NETDEV_DEV(ndev, &pdev->dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ndev->base_addr = res->start;
	ndev->irq = platform_get_irq(pdev, 0);
	ndev->init = ax88180_probe;

	if (register_netdev(ndev) == 0) {
		PRINTK(INIT_MSG, "ax88180: ax88180_init_module end ..........\n");
		platform_set_drvdata(pdev, ndev);
		return 0;
	}

	free_netdev(ndev);

	return -ENXIO;
}


static int ax88180_drv_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct ax88180_local *pax88180_local = netdev_priv(ndev);


	PRINTK(INIT_MSG, "ax88180: ax88180_cleanup_module beginning ..........\n");

	if (ndev != NULL)
		unregister_netdev(ndev);


	kfree(pax88180_local->rx_buf);

	if (Log_MemBase != 0)
		iounmap((void *)Log_MemBase);

	free_netdev(ndev);

	platform_set_drvdata(pdev, NULL);

	PRINTK(INIT_MSG, "ax88180: ax88180_cleanup_module end ..........\n");

	return 0;
}

static struct platform_driver ax88180_driver = {
	.probe	 = ax88180_drv_probe,
	.remove	 = ax88180_drv_remove,
	.driver	 = {
		.name	 = DRV_NAME,
	},
};

static int __init ax88180_init(void)
{
	return platform_driver_register(&ax88180_driver);
}

static void __exit ax88180_cleanup(void)
{
	platform_driver_unregister(&ax88180_driver);
}

module_init(ax88180_init);
module_exit(ax88180_cleanup);
