
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
 *
 *
 *
 *
 *
 *
 *
 *
 * ========================================================================
 */

#include <linux/config.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/interrupt.h>
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
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
#include <linux/ethtool.h>
#endif
#include <linux/mii.h>
#include <linux/if_ether.h>

#include <asm/io.h>
#include <asm/uaccess.h>		/* User space memory access functions */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
#	ifdef CONFIG_BOARD_S3C2440_SMDK
#		include <asm/irq.h>
#		include <asm/arch/S3C2440.h>
#	endif
#else
#	ifdef CONFIG_ARCH_S3C2410
#		include <asm/arch/regs-mem.h>
#		include <asm/arch/regs-irq.h>
#	endif
#endif
#include "ax88180.h"


/*
 * ===========================================================================
 * <<<<<<                   Declare Global Variables                    >>>>>>
 * ===========================================================================
 */
#define	DRV_NAME	"ax88180"
#define	DRV_VERSION	"v1.1.0"

static char version[] __initdata = 
KERN_INFO "ax88180: ASIX AX88180 Non-PCI 32-bit Gigabit Ethernet Driver " DRV_VERSION "\n" 
KERN_INFO "ax88180: Please visit http://www.asix.com.tw for the latest driver.\n";



/*
 * ===========================================================================
 * <<<<<<             Declare Macro/Structure Definition                >>>>>>
 * ===========================================================================
 */

//allan1
/* Information that need to be kept for each board. */
struct _AX88180_PRIVATE {
	struct net_device_stats	stats;
	unsigned long Phy_MemBase;
	unsigned long PhyAddr;
	unsigned long PhyID0;
	unsigned int MediaMode;
	unsigned int RealMediaMode;
	unsigned int ForceMedia;
	unsigned int LineSpeed;
	unsigned int DuplexMode;
	unsigned int JumboFlag;
	unsigned long RxFilterMode;
	unsigned long FirstTxDesc;
	unsigned long NextTxDesc;
	unsigned long rxbuf_overflow_count;
	unsigned char *rx_buf;
	spinlock_t lock;
	struct mii_if_info mii_if;
} AX88180_PRIVATE, *PAX88180_PRIVATE;

static unsigned long Log_MemBase = 0;
unsigned long mem = PLATFORM_MEMBASE; 
unsigned long irq = PLATFORM_IRQ;
unsigned int jumbo = DISABLE_JUMBO; 
unsigned int media = 0;



#define	PRINTK(flag, args...) if (flag & DEBUG_FLAGS) printk(args)	

//Access RXBUFFER_START/TXBUFFER_START to read RX buffer/write TX buffer
#define READ_RXBUF(data) data = *(const volatile unsigned long * const)(Log_MemBase + RXBUFFER_START)
#define WRITE_TXBUF(data) *(volatile unsigned long *)(Log_MemBase + TXBUFFER_START) = data

#define READ_MACREG(regaddr, regdata) regdata = *(volatile unsigned long *)(Log_MemBase + regaddr)
#define WRITE_MACREG(regaddr, regdata) { \
	*(volatile unsigned long *)(Log_MemBase + regaddr) = regdata; \
	}

#define READ_PHYREG(phyaddr, regaddr, regdata) { \
	unsigned long tmpval1, k1; \
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
	unsigned long tmpval2, k2; \
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
	unsigned long tmpval3; \
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
	unsigned long tmpval3a, k3a; \
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
	unsigned long tmpval4; \
	int k4; \
	PRINTK(DEBUG_MSG, "ax88180: AX88180 MAC Registers:\n"); \
	for (k4 = 0xFC00; k4 <= 0xFCFF; k4+=4) { \
		READ_MACREG(k4, tmpval4); \
		PRINTK(DEBUG_MSG, "0x%04x=0x%08lx ", k4, tmpval4); \
		if ((k4 & 0xF) == 0xC) \
			PRINTK(DEBUG_MSG, "\n"); \
	} \
	PRINTK(DEBUG_MSG, "\n"); \
}

//allan3
/* Display all AX88180 PHY registers onto console screen */
#define	DISPLAY_ALLPHYREG { \
	unsigned long tmpval5; \
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
extern int __init ax88180_probe(struct net_device *global_dev);

static int ax88180_open(struct net_device *global_dev);
static int ax88180_stop(struct net_device *global_dev);
static int ax88180_start_xmit(struct sk_buff *skb, struct net_device *global_dev);
static void ax88180_tx_timeout(struct net_device *global_dev);
static struct net_device_stats * ax88180_get_stats(struct net_device *global_dev);
static void ax88180_set_multicast_list(struct net_device *global_dev);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
#ifdef CONFIG_BOARD_S3C2440_SMDK
int set_external_irq(int irq, int edge, int pullup);
#endif
static void ax88180_interrupt(int irq, void *global_dev_id, struct pt_regs * regs);
#else
static irqreturn_t ax88180_interrupt(int irq, void *global_dev_id, struct pt_regs * regs);
#endif

static int ax88180_initialization(struct net_device *global_dev);
static void ax88180_PHY_initial(struct net_device *global_dev);
static void ax88180_meida_config(struct net_device *global_dev);
static void get_MarvellPHY_meida_mode(struct net_device *global_dev);
static void get_CicadaPHY_meida_mode(struct net_device *global_dev);
static void ax88180_rx_handler(struct net_device *global_dev);
static void ax88180_tx_handler(struct net_device *global_dev);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
static int ax88180_ioctl(struct net_device *global_dev, struct ifreq *rq, int cmd);
static int ax88180_ethtool_ioctl(struct net_device *global_dev, void *useraddr);
static int mdio_read(struct net_device *global_dev, int phy_id, int regaddr);
static void mdio_write(struct net_device *global_dev, int phy_id, int regaddr, int regval);
#endif

int set_external_irq(int irq, int edge, int pullup);


/*
 * ===========================================================================
 * <<<<<<                        MODULE-ROUTINES                        >>>>>>
 * ===========================================================================
 */
#ifdef MODULE

MODULE_AUTHOR("Allan Chou <allan@asix.com.tw>");
MODULE_DESCRIPTION("ASIX AX88180 Non-PCI 32-bit Gigabit Ethernet Driver");
MODULE_LICENSE("GPL");

/*
 *****************************************************************************
 * AX88180 module mode driver optional parameters:
 * Syntax: insmod ax88180.o mem=0xXXXXXXXX irq=0xXX media=<media_type> jumbo=x 
 *   mem	Set memory base address (default is 0x08000000)
 *   irq	Set IRQ number (default is 0x27)
 *   media	Set media mode (0:auto, 1:100full, 2:100half, 3:10full 4:10half)
 *   jumbo	Enable/disable Jumbo frame (1=enable, 0=disable)(default is 0)
 * 
 * example: insmod ax88180.o mem=0x08000000 irq=0x27 media=auto jumbo=0 
 *
 *****************************************************************************
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
MODULE_PARM(media, "i");
MODULE_PARM(mem, "i");
MODULE_PARM(irq, "i");
MODULE_PARM(jumbo, "i");
#else
module_param(media, int, 0);
module_param(jumbo, int, 0);
module_param(mem, long, 0);
module_param(irq, long, 0);
#endif

MODULE_PARM_DESC(media, "Media Mode(auto, 100full, 100half, 10full or 10half)");
MODULE_PARM_DESC(mem, "Memory Base Address");
MODULE_PARM_DESC(irq, "Interrupt Number");
MODULE_PARM_DESC(jumbo, "Jumbo Frame(1=enable, 0=disable");

static struct net_device dev_ax;

static int __init ax88180_init_module(void);
static void __exit ax88180_cleanup_module(void);

module_init(ax88180_init_module);
module_exit(ax88180_cleanup_module);


/*
 *****************************************************************************
 * ax88180_init_module()
 *
 * Check for a network adaptor of this type, and return '0' if one exists.
 *
 * Return 0 on success.
 *****************************************************************************
 */
static int __init ax88180_init_module(void)
{
	static struct net_device *global_dev;
	PRINTK(INIT_MSG, "ax88180: ax88180_init_module beginning ..........\n");

	global_dev = &dev_ax;

	global_dev->irq = irq;
	global_dev->base_addr = mem;
	global_dev->init = ax88180_probe;
	if(register_netdev(global_dev) == 0) {
		PRINTK(INIT_MSG, "ax88180: ax88180_init_module end ..........\n");
		return 0;
	}

	if (mem != 0) {
		PRINTK(WARNING_MSG, "AX88180: No AX88180 card found at memory = %#lx\n", mem);
	}
	else {
		PRINTK(WARNING_MSG, "AX88180: You must supply \"mem=0xNNNNNNN\" value(s) for AX88180.\n");
	}
	return -ENXIO;
}


/*
 *****************************************************************************
 * ax88180_cleanup_module()
 *****************************************************************************
 */

static void __exit ax88180_cleanup_module(void)
{
	struct _AX88180_PRIVATE *pax88180_local;
	static struct net_device *global_dev = &dev_ax; 

	PRINTK(INIT_MSG, "ax88180: ax88180_cleanup_module beginning ..........\n");

	pax88180_local = (struct _AX88180_PRIVATE *) global_dev->priv;

	if (global_dev != NULL) {
		unregister_netdev(global_dev);
		free_irq(global_dev->irq, global_dev);
	}

	if (Log_MemBase != 0) {
		iounmap((void *)Log_MemBase);
	}

	if (pax88180_local != NULL) {
		if (pax88180_local->Phy_MemBase != 0) {
			release_mem_region(pax88180_local->Phy_MemBase, AX88180_MEMORY_SIZE);
		}
		kfree(global_dev->priv);
		global_dev->priv = NULL;			
	}

        PRINTK(INIT_MSG, "ax88180: ax88180_cleanup_module end ..........\n");
	return;
}
#endif



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
int __init ax88180_probe(struct net_device *global_dev)
{
        struct _AX88180_PRIVATE *pax88180_local;
	unsigned long phy_membase = mem;

/*
	//allan9 add for debugging
	unsigned long tmp_data;
	unsigned long rx_packet_len;
	unsigned int packet_len;
	unsigned long rxcurt_ptr, rxbound_ptr;
	int i;
*/

	PRINTK(DRIVER_MSG, "%s", version);

	PRINTK(INIT_MSG, "ax88180: ax88180_probe beginning ..........\n");

	SET_MODULE_OWNER(global_dev);

	/* Allocate memory space for AX88180_PRIVATE structure */
	global_dev->priv = kmalloc(sizeof(struct _AX88180_PRIVATE), GFP_KERNEL);
	if (global_dev->priv == NULL) {
		PRINTK(ERROR_MSG, "ax88180: Fail to allocate a private data structure!\n");
		PRINTK(INIT_MSG, "ax88180: ax88180_probe fail end ..........\n");
		unregister_netdev(global_dev);
		return -ENOMEM;
	}

	/* Initialize zero values in the AX88180_PRIVATE structure */
	memset(global_dev->priv, 0, sizeof(struct _AX88180_PRIVATE));

	pax88180_local = (struct _AX88180_PRIVATE *) global_dev->priv;

	if (check_mem_region(phy_membase , AX88180_MEMORY_SIZE) != 0) {
		PRINTK(ERROR_MSG, "ax88180: Memory Region specified (0x%08lX to 0x%08lX) is not available!\n", 
			phy_membase, (phy_membase + AX88180_MEMORY_SIZE - 1UL) );

		kfree(global_dev->priv);
		global_dev->priv = NULL;			
		unregister_netdev(global_dev);

		return -ENOMEM;;
	}
	request_mem_region(phy_membase, AX88180_MEMORY_SIZE, "ASIX_AX88180");

	pax88180_local->Phy_MemBase = phy_membase;
	Log_MemBase = (unsigned long)ioremap(pax88180_local->Phy_MemBase, AX88180_MEMORY_SIZE);

//	PRINTK(DEBUG_MSG, "ax88180: global_dev =0x%p, pax88180_local=0x%p, Log_MemBase=0x%08lx\n", 
//		global_dev, pax88180_local, Log_MemBase);

	PRINTK(DRIVER_MSG, "ax88180: Allocate AX88180 at Phy_MemBase=0x%08lx. (name=%s, IRQ=0x%x)\n", 
		pax88180_local->Phy_MemBase, global_dev->name, (unsigned int)irq);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
#ifdef CONFIG_BOARD_S3C2440_SMDK
	/* Configure SAMSUNG S3C2440A controller for AX88780 operation */
	BANKCON1 = BANKCON1_6CLKS_PAGE;

	PRINTK(DEBUG_MSG, "ax88780: CLKDIVN=0x%04x, CAMDIVN=0x%04x, BANKCON1=0x%04x\n",
	CLKDIVN, CAMDIVN, BANKCON1);
	PRINTK(DEBUG_MSG, "ax88780: UBRDIV0=0x%04x, UBRDIV1=0x%04x, UBRDIV2=0x%04x\n",
	UBRDIV0, UBRDIV1, UBRDIV2);
	PRINTK(DEBUG_MSG, "ax88780: BWSCON=0x%08x, MISCCR=0x%06x\n", BWSCON, MISCCR);

	set_external_irq(irq, EXT_LOWLEVEL, GPIO_PULLUP_DIS);
	EINTMASK &= ~EINT11_MASK; 
	EXTINT1 |= FLTEN11;
#endif
#else
#ifdef CONFIG_ARCH_S3C2410
	/* Configure SAMSUNG S3C2440A controller for AX88780 operation */
	__raw_writel(S3C2410_BANKCON_Tacc6, S3C2410_BANKCON1);
#endif
#endif

	/* Initialize the Ethernet Device structure */
	ether_setup(global_dev);

	global_dev->base_addr = mem;
	global_dev->irq = irq;

	pax88180_local->MediaMode = media;
	pax88180_local->JumboFlag = jumbo;
	pax88180_local->PhyAddr = MARVELL_88E1111_PHYADDR;
	pax88180_local->rx_buf = NULL;

	/* Declare ax88180 routines here */
	global_dev->open		= ax88180_open;
	global_dev->stop		= ax88180_stop;
	global_dev->hard_start_xmit 	= ax88180_start_xmit;
	global_dev->tx_timeout		= ax88180_tx_timeout;
	global_dev->watchdog_timeo	= 200*HZ;
	global_dev->get_stats		= ax88180_get_stats;
	global_dev->set_multicast_list	= ax88180_set_multicast_list;
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
	global_dev->do_ioctl		= ax88180_ioctl;
#endif

	//allan9 add for debugging
	DISPLAY_ALLMACREG;
	DISPLAY_ALLPHYREG;


/*
	//allan9 add for debugging
	READ_MACREG(RXCURT, rxcurt_ptr);
	READ_MACREG(RXBOUND, rxbound_ptr);

	START_READ_RXBUFF;
	READ_RXBUF(rx_packet_len);
    	if ( (rx_packet_len > 0) && (rx_packet_len <= MAX_RX_SIZE) ){
		packet_len = (unsigned int)rx_packet_len;
		PRINTK(DEBUG_MSG, "ax88180: Rx packet length = 0x%08lx\n", rx_packet_len);

		for (i = 0; i < (packet_len - 4); i += 4) {
			READ_RXBUF(tmp_data);
			PRINTK(DEBUG_MSG, "ax88180: Rx data #%d = 0x%08lx\n", i, tmp_data);
		}
	} else {
		PRINTK(DEBUG_MSG, "ax88180: Invalid Rx packet length!! (len=0x%08lx)\n", rx_packet_len);
	}
	STOP_READ_RXBUFF;
	//allan9 end
*/

	PRINTK(INIT_MSG, "ax88180: ax88180_probe end ..........\n");
	return 0;
}


#ifndef MODULE
/*
 * ----------------------------------------------------------------------------
 * Function Name: ax_kprobe
 * Purpose: 
 * Params:
 * Returns:
 * Note:
 * ----------------------------------------------------------------------------
 */
struct net_device * __init ax_kprobe(int unit)
{
	struct net_device *dev = alloc_ei_netdev();
	int err;

	if (!dev)
		return ERR_PTR(-ENOMEM);

	sprintf(dev->name, "eth%d", unit);
	netdev_boot_setup_check(dev);

	err = ax88180_probe(dev);
	if (err)
		goto out;
	return dev;
out:
	free_netdev(dev);
	return ERR_PTR(err);
}
#endif


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
static int ax88180_open(struct net_device *global_dev)
{
	struct _AX88180_PRIVATE *pax88180_local;
	unsigned long tmp_regval;
	int rtn = -ENODEV;

	PRINTK(INIT_MSG, "ax88180: ax88180_open beginning ..........\n");

	pax88180_local = (struct _AX88180_PRIVATE *) global_dev->priv;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
	/* Indicate this module is in use */
	MOD_INC_USE_COUNT;
#endif

	if (pax88180_local->rx_buf == NULL) {
		/* Try to allocate memory space for RX buffer */
		pax88180_local->rx_buf = kmalloc(MAX_RX_SIZE, GFP_KERNEL);
		if (pax88180_local->rx_buf == NULL) {
			PRINTK(ERROR_MSG, "ax88180: Fail to allocate a RX buffer space!\n");
      			PRINTK(INIT_MSG, "ax88180: ax88180_open fail end ..........\n");
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
			MOD_DEC_USE_COUNT;
#endif
			return -ENOMEM;
		}
	}

	/* Initialize zero values in the RX buffer */
	memset(pax88180_local->rx_buf, 0, MAX_RX_SIZE);

	/* Initial AX88180 registers here */
	rtn = ax88180_initialization(global_dev);
	if (rtn) {
		/* Release allocated resource here */
		PRINTK(ERROR_MSG, "ax88180: Fail to initialize AX88180 controller!!\n");
		PRINTK(INIT_MSG, "ax88180: ax88180_open fail end ..........\n");
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
		MOD_DEC_USE_COUNT;
#endif
		return rtn;
	}

	/* Initial variables here */
	INIT_TXRX_VARIABLES;

	/* Declare Interrupt routine to the allocated IRQ */
	rtn = request_irq(global_dev->irq, &ax88180_interrupt, 0, global_dev->name, global_dev);
	if (rtn) {
		/* Release allocated resource here */
		PRINTK(ERROR_MSG, "ax88180: Fail to request IRQ (0x%x)\n", global_dev->irq);
		PRINTK(INIT_MSG, "ax88180: ax88180_open fail end ..........\n");
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
		MOD_DEC_USE_COUNT;
#endif
		return rtn;
	}

	/* Enable AX88180 interrupt */
	ENABLE_INTERRUPT;

	/* Start AX88180 TX/RX functions */
	WRITE_MACREG(CMD, RXEN | TXEN | WAKEMOD);

	/* Check if there is any invalid interrupt status. If yes, clear it. */
	READ_MACREG(ISR, tmp_regval);
	PRINTK(INIT_MSG, "ax88180: The interrupt status = 0x%08lx\n", tmp_regval);
	if (tmp_regval)
		WRITE_MACREG(ISR, tmp_regval);

	/* Display all AX88180 MAC and PHY registers onto console screen */
	DISPLAY_ALLMACREG;
	DISPLAY_ALLPHYREG;

	/* Inform upper protocol to start sending packets */
	netif_start_queue(global_dev);

	/* Driver initialization successful */
	PRINTK(DRIVER_MSG, "ax88180: name=%s, Phy_MemBase=0x%08lx, IRQ=0x%x, media=%d, jumbo=%u\n", 
			global_dev->name, pax88180_local->Phy_MemBase, global_dev->irq, media, jumbo);
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
static int ax88180_stop(struct net_device *global_dev)
{
	struct _AX88180_PRIVATE *pax88180_local;
	unsigned long flags;

	PRINTK(INIT_MSG, "ax88180: ax88180_stop beginning ..........\n");

	pax88180_local = (struct _AX88180_PRIVATE *) global_dev->priv;

/*
	PRINTK(DEBUG_MSG, "ax88180: global_dev =0x%p, pax88180_local=0x%p, dev_name=%s\n", 
		global_dev, pax88180_local, global_dev->name);

	PRINTK(DEBUG_MSG, "ax88180: Phy_MemBase=0x%08lx, Log_MemBase=0x%08lx\n", 
		pax88180_local->Phy_MemBase, Log_MemBase);
*/

	spin_lock_irqsave(&pax88180_local->lock, flags);

	kfree(pax88180_local->rx_buf);

	/* Inform upper layer to stop sending packets to device driver */
	if (netif_device_present(global_dev)) {
		netif_stop_queue(global_dev);
	}

	/* Release interrupt */
	free_irq(global_dev->irq, global_dev);

	spin_unlock_irqrestore(&pax88180_local->lock, flags);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
	/* Indicate this module is NOT used now */
	MOD_DEC_USE_COUNT;
#endif

	/* Driver initialization successful */
	PRINTK(DRIVER_MSG, "ax88180: The AX88180 driver is unloaded successfully.\n");

	PRINTK(INIT_MSG, "ax88180: ax88180_stop end ..........\n");
	return 0;
}


static int ax88180_start_xmit(struct sk_buff *skb, struct net_device *global_dev)
{
	struct _AX88180_PRIVATE *pax88180_local;
	unsigned char *txdata;
	unsigned long TXDES_addr;
	unsigned long txcmd_txdp, txbs_txdp;
	unsigned long txdes0_val, txdes1_val, txdes2_val, txdes3_val;
	unsigned long tmp_data;
	int i;

	pax88180_local = (struct _AX88180_PRIVATE *) global_dev->priv;
	txdata = skb->data;

	PRINTK(TX_MSG, "ax88180: ax88180_start_xmit beginning ..........\n");

//	spin_lock_irq(&pax88180_local->lock);

	/* Inform upper layer to stop sending packets to device driver */
	netif_stop_queue(global_dev);

	pax88180_local->FirstTxDesc = pax88180_local->NextTxDesc;
	txbs_txdp = 1 << pax88180_local->FirstTxDesc;

	//allan9 add to make sure TX machine is OK 
	i = 0;
	READ_MACREG(TXBS, tmp_data);
	READ_MACREG(TXBS, tmp_data);
	PRINTK(TX_MSG, "ax88180: Checking available TXDP (TXBS=0x%08lx)......\n", tmp_data);
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

	PRINTK(TX_MSG, "ax88180: TXDP%d is available, i=%d\n", (int)pax88180_local->FirstTxDesc, i);
	//allan9 end

	txcmd_txdp = pax88180_local->FirstTxDesc << 13;
	TXDES_addr = TXDES0 + (pax88180_local->FirstTxDesc << 2);

	WRITE_MACREG(TXCMD, txcmd_txdp | skb->len | TX_START_WRITE);

	//allan9 add for debugging
	PRINTK(DEBUG_MSG, "ax88180: TX packets (len=0x%x, TXDP%d=0x%08lx)\n", 
		skb->len, (int)pax88180_local->FirstTxDesc, TXDES_addr);
	PRINTK(DEBUG_MSG, "[");
//	for (i = 0; i < skb->len; i++) {
	for (i = 0; i < 64; i++) {
		PRINTK(DEBUG_MSG, "0x%02x ", *(txdata + i));
		if ( (i & 0xF) == 0xF )
			PRINTK(DEBUG_MSG, "\n");
	}
	PRINTK(DEBUG_MSG, "]\n");

	//alln 2006.05.25 modify to support burst mode
/*
	for (i = 0; i < skb->len; i += 4) {
		tmp_data = (unsigned long)*(txdata + i) + (unsigned long)(*(txdata + i + 1) << 8) 
		    + (unsigned long)(*(txdata + i + 2) << 16) + (unsigned long)(*(txdata + i + 3) << 24);
		WRITE_TXBUF(tmp_data);
	}
*/
	memcpy ((void *)(Log_MemBase + TXBUFFER_START), txdata, (skb->len + (4 - skb->len%4)) );

	WRITE_MACREG(TXCMD, txcmd_txdp | skb->len);
	WRITE_MACREG(TXBS, txbs_txdp);
	WRITE_MACREG(TXDES_addr, TXDPx_ENABLE | skb->len);

	//allan9 add for debugging
	READ_MACREG(TXCMD, txcmd_txdp);
	READ_MACREG(TXBS, txbs_txdp);
	READ_MACREG(TXDES0, txdes0_val);
	READ_MACREG(TXDES1, txdes1_val);
	READ_MACREG(TXDES2, txdes2_val);
	READ_MACREG(TXDES3, txdes3_val);
	PRINTK(TX_MSG, "ax88180: TXCMD=0x%08lx, TXBS=0x%08lx\n", txcmd_txdp, txbs_txdp);
	PRINTK(TX_MSG, "ax88180: TXDES0=0x%08lx, TXDES1=0x%08lx, TXDES2=0x%08lx, TXDES3=0x%08lx\n", 
		txdes0_val, txdes1_val, txdes2_val, txdes3_val);

	pax88180_local->stats.tx_packets++;
	pax88180_local->stats.tx_bytes += skb->len;
	dev_kfree_skb(skb);
	global_dev->trans_start = jiffies;

	if (pax88180_local->JumboFlag == ENABLE_JUMBO)
		pax88180_local->NextTxDesc += 2;
	else 
		pax88180_local->NextTxDesc++;

	pax88180_local->NextTxDesc &= TXDP_MASK; 

	/* Inform upper layer to send next queued packets now */
//	netif_wake_queue(global_dev);			//allan9 move to ax88180_tx_handler routine 

//	spin_unlock_irq(&pax88180_local->lock);

	PRINTK(TX_MSG, "ax88180: ax88180_start_xmit end ..........\n\n");
	return 0;
}


/*
 *****************************************************************************
 * ax88180_tx_timeout()
 *****************************************************************************
 */
static void ax88180_tx_timeout(struct net_device *global_dev)
{
	struct _AX88180_PRIVATE *pax88180_local;

	pax88180_local = (struct _AX88180_PRIVATE *) global_dev->priv;

	PRINTK(TX_MSG, "ax88180: ax88180_tx_timeout beginning ..........\n");

	RESET_MAC;
	INIT_TXRX_VARIABLES;

	/* Inform upper layer to send next queued packets now */
	global_dev->trans_start = jiffies; 
	netif_wake_queue(global_dev);

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
static struct net_device_stats * ax88180_get_stats(struct net_device *global_dev)
{
	struct _AX88180_PRIVATE *pax88180_local;
	unsigned long tmp_regval;
	unsigned long flags;

	pax88180_local = (struct _AX88180_PRIVATE *) global_dev->priv;

	PRINTK(OTHERS_MSG, "ax88180: ax88180_get_stats beginning..........\n");

	spin_lock_irqsave(&pax88180_local->lock, flags);

	//Update the statistics counter here.....
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
static void ax88180_set_multicast_list(struct net_device *global_dev)
{
	struct _AX88180_PRIVATE *pax88180_local;
	struct dev_mc_list *mc_list;
	unsigned long mc_hash_table[2];
	int crc_val,i;

	pax88180_local = (struct _AX88180_PRIVATE *) global_dev->priv;

	PRINTK(OTHERS_MSG, "ax88180: ax88180_set_multicast_list beginning ..........\n");

      	pax88180_local->RxFilterMode = DEFAULT_RXFILTER;

	if (global_dev->flags & IFF_PROMISC) {
		pax88180_local->RxFilterMode |= RX_RXANY;
	} else if (global_dev->flags & IFF_ALLMULTI) {
		pax88180_local->RxFilterMode |= RX_MULTICAST;
	} else if (global_dev->flags & IFF_MULTICAST) {
		pax88180_local->RxFilterMode |= RX_MULTI_HASH;

		/* Handle Rx multicast hash table here */
		mc_hash_table[0] = mc_hash_table[1] = 0;
		for (i = 0, mc_list = global_dev->mc_list; (mc_list != NULL) && (i < global_dev->mc_count); 
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

	PRINTK(OTHERS_MSG, "ax88180: ax88180_set_multicast_list end ..........\n");
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
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
static void ax88180_interrupt(int irq, void *global_dev_id, struct pt_regs * regs)
#else
static irqreturn_t ax88180_interrupt(int irq, void *global_dev_id, struct pt_regs * regs)
#endif
{
	struct net_device *global_dev = global_dev_id;
	struct _AX88180_PRIVATE *pax88180_local;
	unsigned long ISR_Status;
	unsigned long rxcurt_ptr, rxbound_ptr;
	unsigned long bmsr_val;
	unsigned long tmp_regval;
	int i;

	pax88180_local = (struct _AX88180_PRIVATE *) global_dev->priv;

	/* Read and check interrupt status here...... */
	READ_MACREG(ISR, ISR_Status);
	if ( (ISR_Status == 0) || (ISR_Status & ~DEFAULT_IMR) ) {
//		PRINTK(WARNING_MSG, "ax88180: Not our interrupt!!\n");
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
		return;
#else
		return 0;
#endif
	}

	PRINTK(INT_MSG, "ax88180: ax88180_interrupt beginning ..........\n");

	//allan9 add for debugging
//	DISPLAY_ALLMACREG;		
//	DISPLAY_ALLPHYREG;

	/* Disable AX88180 interrupt */
	DISABLE_INTERRUPT;

	/* Clear the interrupt status */
	WRITE_MACREG(ISR, ISR_Status);
	PRINTK(INT_MSG, "ax88180: The interrupt status = 0x%08lx\n", ISR_Status);

	/* Handle AX88180 interrupt events */
	if (ISR_Status & ISR_WATCHDOG) {
		PRINTK(DRIVER_MSG, "ax88180: Watchdog Timer interrupt (ISR = 0x%08lx)\n", ISR_Status);
	} 

	if (ISR_Status & ISR_RX) {
		ax88180_rx_handler(global_dev);
	} 

	if (ISR_Status & ISR_TX) {
		ax88180_tx_handler(global_dev);
	} 

	if (ISR_Status & ISR_RXBUFFOVR) {
		pax88180_local->rxbuf_overflow_count++;
		pax88180_local->stats.rx_fifo_errors++;

		READ_MACREG(RXCURT, rxcurt_ptr);
		READ_MACREG(RXBOUND, rxbound_ptr);
		PRINTK(ERROR_MSG, "ax88180: RX Buffer overflow!! (count=%d, RXBOUND=0x%08lx, RXCURT=0x%08lx)\n", 
			(int)pax88180_local->rxbuf_overflow_count, rxbound_ptr, rxcurt_ptr);
		PRINTK(ERROR_MSG, "ax88180: The interrupt status = 0x%08lx\n", ISR_Status);

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
			PRINTK(WARNING_MSG, "ax88180: The cable is connected.\n");
			netif_carrier_on(global_dev);

			if (pax88180_local->ForceMedia == AUTO_MEDIA) 
				ax88180_meida_config(global_dev);

			DISPLAY_ALLPHYREG;
		} else {
			PRINTK(WARNING_MSG, "ax88180: The cable is disconnected.\n");
			netif_carrier_off(global_dev);
			DISPLAY_ALLPHYREG;
		}
	}

	/* Enable AX88180 interrupt */
	ENABLE_INTERRUPT;

	PRINTK(INT_MSG, "ax88180: ax88180_interrupt end ..........\n\n");

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,5,0)
	return;
#else
	return IRQ_RETVAL(0);
#endif
}



/*
 * ===========================================================================
 * <<<<<<             Declare INIT/OTHERS SUB-ROUTINES                  >>>>>>
 * ===========================================================================
 */

/*
 *****************************************************************************
 * ax88180_initialization()
 *****************************************************************************
 */
static int ax88180_initialization(struct net_device *global_dev)
{
	struct _AX88180_PRIVATE *pax88180_local;
//	unsigned long flags;
	unsigned long macid0_val, macid1_val, macid2_val;
	unsigned long tmp_regval;
	int i;

	pax88180_local = (struct _AX88180_PRIVATE *) global_dev->priv;

	PRINTK(INIT_MSG, "ax88180: ax88180_initialization beginning ..........\n");

//	spin_lock_irqsave(&pax88180_local->lock, flags);

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
		global_dev->dev_addr[0] = (unsigned char)macid0_val;
		global_dev->dev_addr[1] = (unsigned char)(macid0_val >> 8);
		global_dev->dev_addr[2] = (unsigned char)macid1_val;
		global_dev->dev_addr[3] = (unsigned char)(macid1_val >> 8);
		global_dev->dev_addr[4] = (unsigned char)macid2_val;
		global_dev->dev_addr[5] = (unsigned char)(macid2_val >> 8);
	} else {
		/* No EEPROM found!! Set a default MAC address. */
		/* The driver designer should assign a legal MAC address here. */
		global_dev->dev_addr[0] = 0x00;
		global_dev->dev_addr[1] = 0x12;
		global_dev->dev_addr[2] = 0x34;
		global_dev->dev_addr[3] = 0x56;
		global_dev->dev_addr[4] = 0x78;
		global_dev->dev_addr[5] = 0x9a;

		macid0_val = (global_dev->dev_addr[1] << 8) + global_dev->dev_addr[0];
		macid1_val = (global_dev->dev_addr[3] << 8) + global_dev->dev_addr[2];
		macid2_val = (global_dev->dev_addr[5] << 8) + global_dev->dev_addr[4];
		WRITE_MACREG(MACID0, macid0_val);
		WRITE_MACREG(MACID1, macid1_val);
		WRITE_MACREG(MACID2, macid2_val);
	}		

	/* Print the MAC address */
	PRINTK(DRIVER_MSG, "ax88180: The MAC address is");
	for (i = 0; i < ETH_ALEN; i++)
		PRINTK(DRIVER_MSG, "%c%02x", i ? ':' : ' ', global_dev->dev_addr[i]);
	PRINTK(DRIVER_MSG, "\n");

	/* Initial PHY registers */
	ax88180_PHY_initial(global_dev);

	/* Configure MAC media mode registers */
	ax88180_meida_config(global_dev);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
	/* Initial MII interface information for ethtool ioctl */
	pax88180_local->mii_if.dev = global_dev;
	pax88180_local->mii_if.phy_id = pax88180_local->PhyAddr;
	pax88180_local->mii_if.phy_id_mask = 0x1F;
	pax88180_local->mii_if.reg_num_mask = 0x1F;
	pax88180_local->mii_if.mdio_read = mdio_read;
	pax88180_local->mii_if.mdio_write = mdio_write;
	pax88180_local->mii_if.force_media = pax88180_local->ForceMedia; 
	pax88180_local->mii_if.full_duplex = pax88180_local->LineSpeed;
	pax88180_local->mii_if.force_media = pax88180_local->DuplexMode; 
#endif

	WRITE_MACREG(RXFILTER, DEFAULT_RXFILTER);

//	spin_unlock_irqrestore(&pax88180_local->lock, flags);

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
static void ax88180_PHY_initial(struct net_device *global_dev)
{
	struct _AX88180_PRIVATE *pax88180_local;
	unsigned long bmcr_val, anar_val, bmsr_val;
	unsigned long aux_1000_ctrl;
	unsigned long tmp_regval;
	unsigned int i;

	pax88180_local = (struct _AX88180_PRIVATE *) global_dev->priv;
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

			//allan9 add for debugging
			DISPLAY_ALLPHYREG;
		}
	}

	PRINTK(INIT_MSG, "ax88180: PHY_Addr=0x%lx, PHY_ID=0x%04x, media=%d\n", 
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

	//allan9 add for debugging
	DISPLAY_ALLPHYREG;

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
static void ax88180_meida_config(struct net_device *global_dev)
{
	struct _AX88180_PRIVATE *pax88180_local;
	unsigned long bmcr_val, bmsr_val;
	unsigned long rxcfg_val, maccfg0_val, maccfg1_val;
	int i;

	pax88180_local = (struct _AX88180_PRIVATE *) global_dev->priv;

	PRINTK(INIT_MSG, "ax88180: ax88180_meida_config beginning ..........\n");

	/* Waiting 200 msecs for PHY link stable */
	for (i = 0; i < 200; i++) {
		READ_PHYREG(pax88180_local->PhyAddr, BMSR, bmsr_val);
		if (bmsr_val & LINKOK) {
			break;
		}
		mdelay(1);
	}

	//allan9 add for debugging
//	DISPLAY_ALLPHYREG;

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
			get_MarvellPHY_meida_mode(global_dev);
		} else if (pax88180_local->PhyID0 == CICADA_CIS8201_PHYIDR0) {
			get_CicadaPHY_meida_mode(global_dev);
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
				global_dev->mtu = MAX_JUMBO_MTU;
			}
			break;

		case MEDIA_1000HALF:
			PRINTK(DRIVER_MSG, "ax88180: Set to 1000Mbps Half-duplex mode.\n");
			pax88180_local->LineSpeed = SPEED_1000;
			pax88180_local->DuplexMode = DUPLEX_HALF;

			rxcfg_val = DEFAULT_RXCFG;
			maccfg0_val = DEFAULT_MACCFG0;
			maccfg1_val = GIGA_MODE_EN | DEFAULT_MACCFG1;
			global_dev->mtu = DEFAULT_ETH_MTU;
			break;

		case MEDIA_100FULL:
			PRINTK(DRIVER_MSG, "ax88180: Set to 100Mbps Full-duplex mode.\n");
			pax88180_local->LineSpeed = SPEED_100;
			pax88180_local->DuplexMode = DUPLEX_FULL;

			rxcfg_val = RXFLOW_ENABLE | DEFAULT_RXCFG;
			maccfg0_val = SPEED100 | TXFLOW_ENABLE | DEFAULT_MACCFG0;
			maccfg1_val = RXFLOW_EN | FULLDUPLEX | DEFAULT_MACCFG1;
			global_dev->mtu = DEFAULT_ETH_MTU;
			break;

		case MEDIA_100HALF:
			PRINTK(DRIVER_MSG, "ax88180: Set to 100Mbps Half-duplex mode.\n");
			pax88180_local->LineSpeed = SPEED_100;
			pax88180_local->DuplexMode = DUPLEX_HALF;

			rxcfg_val = DEFAULT_RXCFG;
			maccfg0_val = SPEED100 | DEFAULT_MACCFG0;
			maccfg1_val = DEFAULT_MACCFG1;
			global_dev->mtu = DEFAULT_ETH_MTU;
			break;

		case MEDIA_10FULL:
			PRINTK(DRIVER_MSG, "ax88180: Set to 10Mbps Full-duplex mode.\n");
			pax88180_local->LineSpeed = SPEED_10;
			pax88180_local->DuplexMode = DUPLEX_FULL;

			rxcfg_val = RXFLOW_ENABLE | DEFAULT_RXCFG;
			maccfg0_val = TXFLOW_ENABLE | DEFAULT_MACCFG0;
			maccfg1_val = RXFLOW_EN | FULLDUPLEX | DEFAULT_MACCFG1;
			global_dev->mtu = DEFAULT_ETH_MTU;
			break;

		case MEDIA_10HALF:
			PRINTK(DRIVER_MSG, "ax88180: Set to 10Mbps Half-duplex mode.\n");
			pax88180_local->LineSpeed = SPEED_10;
			pax88180_local->DuplexMode = DUPLEX_HALF;

			rxcfg_val = DEFAULT_RXCFG;
			maccfg0_val = DEFAULT_MACCFG0;
			maccfg1_val = DEFAULT_MACCFG1;
			global_dev->mtu = DEFAULT_ETH_MTU;
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
			global_dev->mtu = MAX_JUMBO_MTU;
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
static void get_MarvellPHY_meida_mode(struct net_device *global_dev)
{
	struct _AX88180_PRIVATE *pax88180_local;
	unsigned long m88_ssr;
	int i;

	pax88180_local = (struct _AX88180_PRIVATE *) global_dev->priv;

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
static void get_CicadaPHY_meida_mode(struct net_device *global_dev)
{
	struct _AX88180_PRIVATE *pax88180_local;
	unsigned long tmp_regval;

	pax88180_local = (struct _AX88180_PRIVATE *) global_dev->priv;

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
static void ax88180_rx_handler(struct net_device *global_dev)
{
	struct _AX88180_PRIVATE *pax88180_local;
	struct sk_buff *skb;
	unsigned char *rxdata;
//	unsigned long tmp_data;
	unsigned long rx_packet_len;
	unsigned int data_size;
//	unsigned int dword_count, byte_count;
	unsigned long rxcurt_ptr, rxbound_ptr, next_ptr;
	int i;
//	int j;

	pax88180_local = (struct _AX88180_PRIVATE *) global_dev->priv;

	PRINTK(RX_MSG, "ax88180: ax88180_rx_handler beginning ..........\n");

//	spin_lock_irq(&pax88180_local->lock);

	READ_MACREG(RXCURT, rxcurt_ptr);
	READ_MACREG(RXBOUND, rxbound_ptr);
	next_ptr = (rxbound_ptr + 1) & RX_PAGE_NUM_MASK;

	PRINTK(RX_MSG, "ax88180: RX original RXBOUND=0x%08lx, RXCURT=0x%08lx\n", rxbound_ptr, rxcurt_ptr);

	while (next_ptr != rxcurt_ptr) {
		START_READ_RXBUFF;
		READ_RXBUF(rx_packet_len);
 	  	if ( (rx_packet_len == 0) || (rx_packet_len > MAX_RX_SIZE) ) {
			pax88180_local->stats.rx_errors++;
			STOP_READ_RXBUFF;
			RESET_MAC;
			PRINTK(ERROR_MSG, "ax88180: Invalid Rx packet length!! (len=0x%08lx)\n", rx_packet_len);
			PRINTK(ERROR_MSG, "ax88180: RX RXBOUND=0x%08lx, RXCURT=0x%08lx\n", rxbound_ptr, rxcurt_ptr);
			PRINTK(RX_MSG, "ax88180: ax88180_rx_handler fail end ..........\n");
			return;
		}
		data_size = (unsigned int)rx_packet_len;
		rxbound_ptr += (((data_size + 0xF) & 0xFFF0) >> 4) + 1;
		rxbound_ptr &= RX_PAGE_NUM_MASK;

		skb = dev_alloc_skb(data_size + 2);
		if (skb == NULL) {
			pax88180_local->stats.rx_dropped++;
			STOP_READ_RXBUFF;
			PRINTK(ERROR_MSG, "ax88180: No available memory space. Dropping RX packets!!\n");
			PRINTK(RX_MSG, "ax88180: ax88180_rx_handler fail end ..........\n");
			return;
		}

		skb->data = skb->head;
		skb->tail = skb->head;
		skb_reserve(skb, 2);
		skb->dev = global_dev;
		rxdata = skb_put(skb, data_size);

		memcpy (pax88180_local->rx_buf, (void *)(Log_MemBase + RXBUFFER_START), (data_size + 4 - data_size%4));
		memcpy (rxdata, pax88180_local->rx_buf, data_size);

/*
		dword_count = data_size >> 2;
		byte_count = data_size & 0x3;
		for (i = 0; i < dword_count; i++) {
			READ_RXBUF(tmp_data);
			*((unsigned long *)rxdata + i)= tmp_data;
		}
		if (byte_count != 0) {
			READ_RXBUF(tmp_data);
			for (j = 0; j < byte_count; j++) {
				*(rxdata + (dword_count * 4) + j) = (unsigned char)(tmp_data >> (j *8));
			}
		}
*/

		STOP_READ_RXBUFF;

		skb->protocol = eth_type_trans(skb, global_dev);
		netif_rx(skb);
		global_dev->last_rx = jiffies;
		pax88180_local->stats.rx_packets++;
		pax88180_local->stats.rx_bytes += data_size;

		WRITE_MACREG(RXBOUND, rxbound_ptr);

		PRINTK(DEBUG_MSG, "ax88180: Rx data size = 0x%x\n", data_size);
		PRINTK(DEBUG_MSG, "[");
		for (i = 0; i < data_size; i++) {
			PRINTK(DEBUG_MSG, "0x%02x ", *(rxdata + i));
			if ( (i & 0xF) == 0xF )
				PRINTK(DEBUG_MSG, "\n");
		}
		PRINTK(DEBUG_MSG, "]\n");

		READ_MACREG(RXCURT, rxcurt_ptr);
		READ_MACREG(RXBOUND, rxbound_ptr);
		next_ptr = (rxbound_ptr + 1) & RX_PAGE_NUM_MASK;

		PRINTK(RX_MSG, "ax88180: RX updated RXBOUND=0x%08lx, RXCURT=0x%08lx\n", 
			rxbound_ptr, rxcurt_ptr);
	}

	if (pax88180_local->rxbuf_overflow_count > 0)
		pax88180_local->rxbuf_overflow_count--;

//	spin_unlock_irq(&pax88180_local->lock);

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
static void ax88180_tx_handler(struct net_device *global_dev)
{
	struct _AX88180_PRIVATE *pax88180_local;

	pax88180_local = (struct _AX88180_PRIVATE *) global_dev->priv;

	PRINTK(TX_MSG, "ax88180: ax88180_tx_handler beginning ..........\n");

	/* Inform upper layer to send next queued packets now */
	netif_wake_queue(global_dev);

	PRINTK(TX_MSG, "ax88180: ax88180_tx_handler end ..........\n");
	return;
}



#if LINUX_VERSION_CODE > KERNEL_VERSION(2,5,0)
/*
 *****************************************************************************
 * ax88180_ioctl()
 *
 * Handle the ioctl commands like ethtool.
 *
 ***************************************************************************** 
 */
static int ax88180_ioctl(struct net_device *global_dev, struct ifreq *rq, int cmd)
{
	PRINTK(OTHERS_MSG, "ax88180: ax88180_ioctl beginning ..........\n");

	switch(cmd) {
	case SIOCETHTOOL:
		return ax88180_ethtool_ioctl(global_dev, (void*)rq->ifr_data); 

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
static int ax88180_ethtool_ioctl(struct net_device *global_dev, void *useraddr)
{
	struct _AX88180_PRIVATE *pax88180_local;
	u32 ethcmd;

	pax88180_local = (struct _AX88180_PRIVATE *) global_dev->priv;

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
 * 
 *
 ***************************************************************************** 
 */
static int mdio_read(struct net_device *global_dev, int phy_id, int regaddr)
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
 * 
 *
 ***************************************************************************** 
 */
static void mdio_write(struct net_device *global_dev, int phy_id, int regaddr, int regval)
{
	WRITE_PHYREG(phy_id, (unsigned long)regaddr, (unsigned long)regval);

	//allan9 add for debugging
	READ_PHYREG(phy_id, regaddr, regval);
	PRINTK(DEBUG_MSG, "ax88180: mdio_write regval=0x%04x\n", regval);
	return;
}
#endif
