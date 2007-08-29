/* can_sja1000funcs
*
* can4linux -- LINUX CAN device driver source
* 
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * 
* Copyright (c) 2002 port GmbH Halle/Saale
* (c) 2001 Heinz-Juergen Oertel (oe@port.de)
*------------------------------------------------------------------
* $Header: /z2/cvsroot/products/0530/software/can4linux/src/sja1000funcs.c,v 1.3 2007/02/07 15:25:38 oe Exp $
*
*--------------------------------------------------------------------------
*
*
* modification history
* --------------------
* $Log: sja1000funcs.c,v $
* Revision 1.3  2007/02/07 15:25:38  oe
* 64 bit
*
* Revision 1.2  2006/06/21 13:18:32  oe
* - Update support fpr EMS CPC-104
* - mixed configuration CPC-104 and PCM3680
*
* Revision 1.1  2006/04/21 16:26:53  oe
* - new version with new file structure for all architectures
*
* Revision 1.14  2006/01/09 10:07:28  oe
* - added int CAN_SetBTR
*
* Revision 1.13  2005/08/29 08:51:27  oe
* - added support for SJA1000 ListenOnlyMode, thanks to Steven Scholz
*   <steven.scholz@imc-berlin.de>
* - added support for frame self reception mode
*
* Revision 1.12  2005/07/22 12:01:47  oe
* applied patch by Steven Scholz
* - new ioctl( , CONF_LISTEN_ONLY_MODE, )
* - replaced verify_area() by !access_ok()
*
* Revision 1.11  2005/04/01 16:05:50  oe
* - replaced cli() ...
*
* Revision 1.10  2005/02/09 16:17:05  oe
* - corrected __iomem annotation
*
* Revision 1.9  2005/02/07 11:44:10  oe
* - casting io-mem data types __iomem
*
* Revision 1.8  2005/02/07 11:17:09  oe
* - IRQ Acknowledge for (CCPC104) added
*
* Revision 1.7  2004/12/14 09:36:36  oe
* - Release for kernel 2.6 with support for 82527 included
*
* Revision 1.6  2004/05/14 10:02:54  oe
* - started supporting CPC-Card
* - version number in can4linux.h available
* - only one structure type for Config_par_t Command_par_t
* - new ioctl command CMD_CLEARBUFFERS
*
* Revision 1.5  2004/02/13 14:19:50  oe
* *** empty log message ***
*
* Revision 1.4  2003/12/29 15:20:15  oe
* - support for indexed-i/o
*
* Revision 1.3  2003/08/27 17:49:27  oe
* - New CanStatusPar structure
*
* Revision 1.2  2003/08/27 13:06:27  oe
* - Version 3.0
*
* Revision 1.1  2003/07/05 14:28:55  oe
* - all changes for the new 3.0: try to eliminate hw depedencies at run-time.
*   configure for HW at compile time
*
* Revision 1.8  2002/10/25 10:39:00  oe
* - vendor specific handling for Advantech board added by "R.R.Robotica" <rrrobot@tin.it>
*
* Revision 1.7  2002/10/11 16:58:06  oe
* - IOModel, Outc, VendOpt are now set at compile time
* - deleted one misleading printk()
*
* Revision 1.6  2002/08/20 05:57:22  oe
* - new write() handling, now not ovrwriting buffer content if buffer fill
* - ioctl() get status returns buffer information
*
* Revision 1.5  2002/08/08 17:55:32  oe
* - added vendor specific code for stzp 's' IXXAT board iPC03
* - received message with id=0xffffffff == error
*
* Revision 1.4  2001/09/14 14:58:09  oe
* first free release
*
*
*/
#include "defs.h"
#include "linux/delay.h"
#include "sja1000.h"


/* int	CAN_Open = 0; */

/* timing values */
u8 CanTiming[10][2]={
	{CAN_TIM0_10K,  CAN_TIM1_10K},
	{CAN_TIM0_20K,  CAN_TIM1_20K},
	{CAN_TIM0_50K,  CAN_TIM1_50K},
	{CAN_TIM0_100K, CAN_TIM1_100K},
	{CAN_TIM0_125K, CAN_TIM1_125K},
	{CAN_TIM0_250K, CAN_TIM1_250K},
	{CAN_TIM0_500K, CAN_TIM1_500K},
	{CAN_TIM0_800K, CAN_TIM1_800K},
	{CAN_TIM0_1000K,CAN_TIM1_1000K}};



#ifdef CAN_INDEXED_PORT_IO
canregs_t* regbase = 0;
#endif

/* Board reset
   means the following procedure:
  set Reset Request
  wait for Rest request is changing - used to see if board is available
  initialize board (with valuse from /proc/sys/Can)
    set output control register
    set timings
    set acceptance mask
*/


#ifdef DEBUG
int CAN_ShowStat (int board)
{
    if (dbgMask && (dbgMask & DBG_DATA)) {
    printk(" MODE 0x%x,", CANin(board, canmode));
    printk(" STAT 0x%x,", CANin(board, canstat));
    printk(" IRQE 0x%x,", CANin(board, canirq_enable));
    printk(" INT 0x%x\n", CANin(board, canirq));
    printk("\n");
    }
    return 0;
}
#endif

/* can_GetStat - read back as many status information as possible 
*
* Because the CAN protocol itself describes different kind of information
* already, and the driver has some generic information
* (e.g about it's buffers)
* we can define a more or less hardware independent format.
*
*
* exception:
* ERROR WARNING LIMIT REGISTER (EWLR)
* The SJA1000 defines a EWLR, reaching this Error Warning Level
* an Error Warning interrupt can be generated.
* The default value (after hardware reset) is 96. In reset
* mode this register appears to the CPU as a read/write
* memory. In operating mode it is read only.
* Note, that a content change of the EWLR is only possible,
* if the reset mode was entered previously. An error status
* change (see status register; Table 14) and an error
* warning interrupt forced by the new register content will not
* occur until the reset mode is cancelled again.
*/

int can_GetStat(
	struct inode *inode,
	CanStatusPar_t *stat
	)
{
unsigned int board = iminor(inode);	
msg_fifo_t *Fifo;
unsigned long flags;

    stat->type = CAN_TYPE_SJA1000;

    stat->baud = Baud[board];
    /* printk(" STAT ST %d\n", CANin(board, canstat)); */
    stat->status = CANin(board, canstat);
    /* printk(" STAT EWL %d\n", CANin(board, errorwarninglimit)); */
    stat->error_warning_limit = CANin(board, errorwarninglimit);
    stat->rx_errors = CANin(board, rxerror);
    stat->tx_errors = CANin(board, txerror);
    stat->error_code= CANin(board, errorcode);

    /* Disable CAN (All !!) Interrupts */
    /* !!!!!!!!!!!!!!!!!!!!! */
    /* save_flags(flags); cli(); */
    local_irq_save(flags);

    Fifo = &Rx_Buf[board];
    stat->rx_buffer_size = MAX_BUFSIZE;	/**< size of rx buffer  */
    /* number of messages */
    stat->rx_buffer_used =
    	(Fifo->head < Fifo->tail)
    	? (MAX_BUFSIZE - Fifo->tail + Fifo->head) : (Fifo->head - Fifo->tail);
    Fifo = &Tx_Buf[board];
    stat->tx_buffer_size = MAX_BUFSIZE;	/* size of tx buffer  */
    /* number of messages */
    stat->tx_buffer_used = 
    	(Fifo->head < Fifo->tail)
    	? (MAX_BUFSIZE - Fifo->tail + Fifo->head) : (Fifo->head - Fifo->tail);
    /* Enable CAN Interrupts */
    /* !!!!!!!!!!!!!!!!!!!!! */
    /* restore_flags(flags); */
    local_irq_restore(flags);
    return 0;
}

int CAN_ChipReset (int minor)
{
uint8 status;
/* int i; */

    DBGin("CAN_ChipReset");
    DBGprint(DBG_DATA,(" INT 0x%x\n", CANin(minor, canirq)));

    CANout(minor, canmode, CAN_RESET_REQUEST);



    /* for(i = 0; i < 100; i++) SLOW_DOWN_IO; */
    udelay(10);

    status = CANin(minor, canstat);

    DBGprint(DBG_DATA,("status=0x%x mode=0x%x", status,
	    CANin(minor, canmode) ));
#if 0
    if( ! (CANin(minor, canmode) & CAN_RESET_REQUEST ) ){
	    printk("ERROR: no board present!\n");
	    /* MOD_DEC_USE_COUNT; */
#if defined(PCM3680) || defined(CPC104) || defined(CPC_PCM_104)
	    CAN_Release(minor);
#endif
	    DBGout();return -1;
    }
#endif

    DBGprint(DBG_DATA, ("[%d] CAN_mode 0x%x\n", minor, CANin(minor, canmode)));
    /* select mode: Basic or PeliCAN */
    CANout(minor, canclk, CAN_MODE_PELICAN + CAN_MODE_CLK);
    CANout(minor, canmode, CAN_RESET_REQUEST + CAN_MODE_DEF);
    DBGprint(DBG_DATA, ("[%d] CAN_mode 0x%x\n", minor, CANin(minor, canmode)));
    
    /* Board specific output control */
    if (Outc[minor] == 0) {
	Outc[minor] = CAN_OUTC_VAL; 
    }
    CANout(minor, canoutc, Outc[minor]);
    DBGprint(DBG_DATA, ("[%d] CAN_mode 0x%x\n", minor, CANin(minor, canmode)));

    CAN_SetTiming(minor, Baud[minor]    );
    DBGprint(DBG_DATA, ("[%d] CAN_mode 0x%x\n", minor, CANin(minor, canmode)));
    CAN_SetMask  (minor, AccCode[minor], AccMask[minor] );
    DBGprint(DBG_DATA, ("[%d] CAN_mode 0x%x\n", minor, CANin(minor, canmode)));

    /* can_dump(minor); */
    DBGout();
    return 0;
}


/*
 * Configures bit timing registers directly. Chip must be in bus off state.
 */
int CAN_SetBTR (int board, int btr0, int btr1)
{
    DBGin("CAN_SetBTR");
    DBGprint(DBG_DATA, ("[%d] btr0=%d, btr1=%d", board, btr0, btr1));

    /* select mode: Basic or PeliCAN */
    CANout(board, canclk, CAN_MODE_PELICAN + CAN_MODE_CLK);
    CANout(board, cantim0, (uint8) (btr0 & 0xff));
    CANout(board, cantim1, (uint8) (btr1 & 0xff));
    DBGprint(DBG_DATA,("tim0=0x%x tim1=0x%x", CANin(board, cantim0), CANin(board, cantim1)) );
    DBGout();
    return 0;
}


/*
 * Configures bit timing. Chip must be in bus off state.
 */
int CAN_SetTiming (int board, int baud)
{
int i = 5;
int custom=0;
    DBGin("CAN_SetTiming");
    DBGprint(DBG_DATA, ("baud[%d]=%d", board, baud));
    switch(baud)
    {
	case   10: i = 0; break;
	case   20: i = 1; break;
	case   50: i = 2; break;
	case  100: i = 3; break;
	case  125: i = 4; break;
	case  250: i = 5; break;
	case  500: i = 6; break;
	case  800: i = 7; break;
	case 1000: i = 8; break;
	default  : 
		custom=1;
    }
    /* select mode: Basic or PeliCAN */
    CANout(board, canclk, CAN_MODE_PELICAN + CAN_MODE_CLK);
    if( custom ) {
       CANout(board, cantim0, (uint8) (baud >> 8) & 0xff);
       CANout(board, cantim1, (uint8) (baud & 0xff ));
    } else {
       CANout(board,cantim0, (uint8) CanTiming[i][0]);
       CANout(board,cantim1, (uint8) CanTiming[i][1]);
    }
    DBGprint(DBG_DATA,("tim0=0x%x tim1=0x%x",
    		CANin(board, cantim0), CANin(board, cantim1)) );

    DBGout();
    return 0;
}


int CAN_StartChip (int board)
{
/* int i; */
    RxErr[board] = TxErr[board] = 0L;
    DBGin("CAN_StartChip");
    DBGprint(DBG_DATA, ("[%d] CAN_mode 0x%x\n", board, CANin(board, canmode)));
/*
    CANout( board,cancmd, (CAN_RELEASE_RECEIVE_BUFFER 
			  | CAN_CLEAR_OVERRUN_STATUS) ); 
*/
    /* for(i=0;i<100;i++) SLOW_DOWN_IO; */
    udelay(10);
    /* clear interrupts */
    CANin(board, canirq);

    /* Interrupts on Rx, TX, any Status change and data overrun */
    CANset(board, canirq_enable, (
		  CAN_OVERRUN_INT_ENABLE
		+ CAN_ERROR_INT_ENABLE
		+ CAN_TRANSMIT_INT_ENABLE
		+ CAN_RECEIVE_INT_ENABLE ));

    CANreset( board, canmode, CAN_RESET_REQUEST );
    DBGprint(DBG_DATA,("start mode=0x%x", CANin(board, canmode)));

    DBGout();
    return 0;
}


int CAN_StopChip (int board)
{
    DBGin("CAN_StopChip");
    CANset(board, canmode, CAN_RESET_REQUEST );
    DBGout();
    return 0;
}

/* set value of the output control register */
int CAN_SetOMode (int board, int arg)
{

    DBGin("CAN_SetOMode");
	DBGprint(DBG_DATA,("[%d] outc=0x%x", board, arg));
	CANout(board, canoutc, arg);

    DBGout();
    return 0;
}

/*
Listen-Only Mode
In listen-only mode, the CAN module is able to receive messages
without giving an acknowledgment.
Since the module does not influence the CAN bus in this mode
the host device is capable of functioning like a monitor
or for automatic bit-rate detection.

 must be done after CMD_START (CAN_StopChip)
 and before CMD_START (CAN_StartChip)
*/
int CAN_SetListenOnlyMode (int board, int arg)
{
    DBGin("CAN_SetListenOnlyMode");
    if (arg) {
	CANset(board, canmode, CAN_LISTEN_ONLY_MODE);
    } else {
	CANreset(board, canmode, CAN_LISTEN_ONLY_MODE );
    }

    DBGout();
    return 0;
}

int CAN_SetMask (int board, unsigned int code, unsigned int mask)
{
#ifdef CPC_PCI
# define R_OFF 4 /* offset 4 for the EMS CPC-PCI card */
#else
# define R_OFF 1
#endif

    DBGin("CAN_SetMask");
    DBGprint(DBG_DATA,("[%d] acc=0x%x mask=0x%x",  board, code, mask));
    CANout(board, frameinfo,
			(unsigned char)((unsigned int)(code >> 24) & 0xff));	
    CANout(board, frame.extframe.canid1,
			(unsigned char)((unsigned int)(code >> 16) & 0xff));	
    CANout(board, frame.extframe.canid2,
			(unsigned char)((unsigned int)(code >>  8) & 0xff));	
    CANout(board, frame.extframe.canid3,
    			(unsigned char)((unsigned int)(code >>  0 ) & 0xff));	

    CANout(board, frame.extframe.canid4,
			(unsigned char)((unsigned int)(mask >> 24) & 0xff));
    CANout(board, frame.extframe.canxdata[0 * R_OFF],
			(unsigned char)((unsigned int)(mask >> 16) & 0xff));
    CANout(board, frame.extframe.canxdata[1 * R_OFF],
			(unsigned char)((unsigned int)(mask >>  8) & 0xff));
    CANout(board, frame.extframe.canxdata[2 * R_OFF],
			(unsigned char)((unsigned int)(mask >>  0) & 0xff));

    DBGout();
    return 0;
}




/* 
Single CAN frames or the very first Message are copied into the CAN controller
using this function. After that an transmission request is set in the
CAN controllers command register.
After a succesful transmission, an interrupt will be generated,
which will be handled in the CAN ISR CAN_Interrupt()
*/
int CAN_SendMessage (int minor, canmsg_t *tx)
{
int i = 0;
int ext;			/* message format to send */
uint8 tx2reg, stat;

    DBGin("CAN_SendMessage");

    while ( ! (stat=CANin(minor, canstat))
  	& CAN_TRANSMIT_BUFFER_ACCESS ) {
	    #if LINUX_VERSION_CODE >= 131587
	    # if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0)
	    cond_resched();
	    # else
	    if( current->need_resched ) schedule();
	    # endif
	    #else
	    if( need_resched ) schedule();
	    #endif
    }

    DBGprint(DBG_DATA,(
    		"CAN[%d]: tx.flags=%d tx.id=0x%lx tx.length=%d stat=0x%x",
		minor, tx->flags, tx->id, tx->length, stat));

    tx->length %= 9;			/* limit CAN message length to 8 */
    ext = (tx->flags & MSG_EXT);	/* read message format */

    /* fill the frame info and identifier fields */
    tx2reg = tx->length;
    if( tx->flags & MSG_RTR) {
	    tx2reg |= CAN_RTR;
    }
    if(ext) {
    DBGprint(DBG_DATA, ("---> send ext message \n"));
	CANout(minor, frameinfo, CAN_EFF + tx2reg);
	CANout(minor, frame.extframe.canid1, (uint8)(tx->id >> 21));
	CANout(minor, frame.extframe.canid2, (uint8)(tx->id >> 13));
	CANout(minor, frame.extframe.canid3, (uint8)(tx->id >> 5));
	CANout(minor, frame.extframe.canid4, (uint8)(tx->id << 3) & 0xff);

    } else {
	DBGprint(DBG_DATA, ("---> send std message \n"));
	CANout(minor, frameinfo, CAN_SFF + tx2reg);
	CANout(minor, frame.stdframe.canid1, (uint8)((tx->id) >> 3) );
	CANout(minor, frame.stdframe.canid2, (uint8)(tx->id << 5 ) & 0xe0);
    }

    /* - fill data ---------------------------------------------------- */
    if(ext) {
	for( i=0; i < tx->length ; i++) {
	    CANout( minor, frame.extframe.canxdata[R_OFF * i], tx->data[i]);
	    	/* SLOW_DOWN_IO; SLOW_DOWN_IO; */
	}
    } else {
	for( i=0; i < tx->length ; i++) {
	    CANout( minor, frame.stdframe.candata[R_OFF * i], tx->data[i]);
	    	/* SLOW_DOWN_IO; SLOW_DOWN_IO; */
	}
    }
    /* - /end --------------------------------------------------------- */
    CANout(minor, cancmd, CAN_TRANSMISSION_REQUEST );

    if(selfreception[minor]) {
	/* prepare for next TX Interrupt and selfreception */
	memcpy(
	    (void *)&last_Tx_object[minor],
	    (void *)tx,
	    sizeof(canmsg_t));
    }

  DBGout();return i;
}

#if 1
/* CAN_GetMessage is used in Polling Mode with ioctl()
 * !!!! curently not working for the PELICAN mode 
 * and BASIC CAN mode code already removed !!!!
 */
int CAN_GetMessage (int minor, canmsg_t *rx )
{
uint8 dummy = 0, stat;
int i = 0;
  
    DBGin("CAN_GetMessage");
    stat = CANin(minor, canstat);

    rx->flags  = 0;
    rx->length = 0;

    if( stat & CAN_DATA_OVERRUN ) {
    DBGprint(DBG_DATA,("Rx: overrun!\n"));
    Overrun[minor]++;
    }

    if( stat & CAN_RECEIVE_BUFFER_STATUS ) {
      dummy  &= 0x0f; /* strip length code */ 
      rx->length = dummy;
      DBGprint(DBG_DATA,("rx.id=0x%lx rx.length=0x%x", rx->id, dummy));

      dummy %= 9;
      for( i = 0; i < dummy; i++) {
      /* missing code here */
	DBGprint(DBG_DATA,("rx[%d]: 0x%x ('%c')",i,rx->data[i],rx->data[i] ));
      }
      CANout(minor, cancmd, CAN_RELEASE_RECEIVE_BUFFER | CAN_CLEAR_OVERRUN_STATUS );
    } else {
      i=0;
    }
    DBGout();
    return i;
}
#endif




/*
 * Perform Vendor, that means sometimes CAN controller
 * or only board manufacturer specific initialization.
 *
 * Mainly it gets needed IO and IRQ ressources and initilaizes 
 * special hardware functions.
 *
 * This code should normally be in the CAN_VendorInit() function
 * in a TERGET specific file  target.c
 */
#if    !defined(KVASER_PCICAN)	\
    && !defined(VCMA9)		\
    && !defined(CPC104)		\
    && !defined(CPC_PCM_104)	\
    && !defined(PCM3680)

int CAN_VendorInit (int minor)
{
    DBGin("CAN_VendorInit");
/* 1. Vendor specific part ------------------------------------------------ */
#if defined(IXXAT_PCI03)
    can_range[minor] = 0x200; /*stpz board or Advantech Pcm-3680 */
#else
    can_range[minor] = CAN_RANGE;
#endif
    
/* End: 1. Vendor specific part ------------------------------------------- */


#if !defined(CAN4LINUX_PCI) || defined(CC_CANPCI)
    /* Request the controllers address space */
#if defined(CAN_PORT_IO) 
    /* It's port I/O */
    if(NULL == request_region(Base[minor], can_range[minor], "CAN-IO")) {
	DBGprint(DBG_DATA,("Request_region CAN-IO failed at %x\n",
		Base[minor]));
	return -EBUSY;
    }
#else
#if defined(CAN_INDEXED_PORT_IO)
    /* It's indexed port I/O */
    if(NULL == request_region(Base[minor], 2, "CAN-IO")) {
	DBGprint(DBG_DATA,("Request_region CAN-IO failed at %x\n",
		Base[minor]));
      return -EBUSY;
    } 
#else
    /* Some LINUX Systems scan the PC104 area already
       and claim the address range as "Adapter-ROM"
       (MPL PIP-10 did it this way)
     */
    /* It's Memory I/O */
    if(NULL == request_mem_region(Base[minor], can_range[minor], "CAN-IO")) {
	DBGprint(DBG_DATA,("Request_mem_region CAN-IO failed at %x\n",
		Base[minor]));
	return -EBUSY;
    }
#endif
#endif
#endif 	/* !defined(CAN4LINUX_PCI) */

/* printk("2\n"); */
/* printk("CAN[%d]: Base %x\n", minor, Base[minor]); */
/* printk("CAN[%d]: base %p\n", minor, can_base[minor]); */
/* printk("CAN[%d]: range %x\n", minor, can_range[minor]); */

#if (defined(CAN4LINUX_PCI) || defined (CAN4LINUX_PCCARD)) && !defined(CC_CANPCI)


	/* PCI scan for CPC-PCI (or others ) has already remapped the address */
	/* printk(" assign address direct\n"); */
	can_base[minor] = (void __iomem *)Base[minor];
#else
	/* printk(" --> use ioremap\n"); */
	can_base[minor] = ioremap(Base[minor], 0x200);
#endif




    /* now the virtual address can be used for the register address macros */
/* !!!!!!!!!!!!! can_base[minor] = (unsigned char *)Base[minor]; */
/* printk("---------------------- 3\n"); */
/* printk("CAN[%d]: Base %x\n", minor, Base[minor]); */
/* printk("CAN[%d]: base %p\n", minor, can_base[minor]); */
/* printk("CAN[%d]: range %x\n", minor, can_range[minor]); */
	/* release_region(Base[minor], can_range[minor] ); */
	/* return -EBUSY; */


/* 2. Vendor specific part ------------------------------------------------ */

/* ixxat/stpz board */
#if defined(IXXAT_PCI03)
	if( Base[minor] & 0x200 ) {
		/* printk("Resetting IXXAT PC I-03 [contr 1]\n"); */
		/* perform HW reset 2. contr*/
		writeb(0xff, can_base[minor] + 0x300);
	} else {
		/* printk("Resetting IXXAT PC I-03 [contr 0]\n"); */
		/* perform HW reset 1. contr*/
		writeb(0xff, can_base[minor] + 0x100);
	}
#endif


/* End: 2. Vendor specific part ------------------------------------------- */

    /* The Interrupt Line is alrady requestes by th PC CARD Services
     * (in case of CPC-Card: cpc-card_cs.c) 
    */ 
#if !defined (CAN4LINUX_PCCARD)
    if( IRQ[minor] > 0 || IRQ[minor] > MAX_IRQNUMBER ){
        if( Can_RequestIrq( minor, IRQ[minor] , CAN_Interrupt) ) {
	     printk("Can[%d]: Can't request IRQ %d \n", minor, IRQ[minor]);
	     DBGout(); return -EBUSY;
        }
    } else {
	/* Invalid IRQ number in /proc/.../IRQ */
	DBGout(); return -EBUSY;
    }
#endif

    DBGout(); return 0;
}
#endif   /*  !defined(KVASER_PCICAN) */

#ifdef KVASER_PCICAN
/* PCI Bridge AMCC 5920 registers */
#define S5920_OMB    0x0C
#define S5920_IMB    0x1C
#define S5920_MBEF   0x34
#define S5920_INTCSR 0x38
#define S5920_RCR    0x3C
#define S5920_PTCR   0x60

#define INTCSR_ADDON_INTENABLE_M        0x2000
#define INTCSR_INTERRUPT_ASSERTED_M     0x800000
#endif

#include <linux/pci.h>


int Can_RequestIrq(int minor, int irq,
    irqreturn_t (*handler)(int, void *, struct pt_regs *))
{
int err = 0;

    DBGin("Can_RequestIrq");
    /*

    int request_irq(unsigned int irq,			// interrupt number  
              void (*handler)(int, void *, struct pt_regs *), // pointer to ISR
		              irq, dev_id, registers on stack
              unsigned long irqflags, const char *devname,
              void *dev_id);

       dev_id - The device ID of this handler (see below).       
       This parameter is usually set to NULL,
       but should be non-null if you wish to do  IRQ  sharing.
       This  doesn't  matter when hooking the
       interrupt, but is required so  that,  when  free_irq()  is
       called,  the  correct driver is unhooked.  Since this is a
       void *, it can point to anything (such  as  a  device-spe-
       cific  structure,  or even empty space), but make sure you
       pass the same pointer to free_irq().

    */

    err = request_irq( irq, handler, SA_SHIRQ, "Can", &Can_minors[minor]);

    if( !err ){
/* printk("Requested IRQ[%d]: %d @ 0x%x", minor, irq, handler); */
      DBGprint(DBG_BRANCH,("Requested IRQ: %d @ 0x%lx",
      				irq, (unsigned long)handler));
      IRQ_requested[minor] = 1;
    }


#ifdef KVASER_PCICAN
    {
	unsigned long tmp;
	/* Enable interrupts from card */
	/* printk(" enable PCI interrupt\n"); */
	tmp = inl(pci_resource_start(Can_pcidev[minor], 0) + S5920_INTCSR);
	tmp |= INTCSR_ADDON_INTENABLE_M;
	outl(tmp, pci_resource_start(Can_pcidev[minor], 0) + S5920_INTCSR);
    }
#endif







    DBGout(); return err;
}

int Can_FreeIrq(int minor, int irq )
{
    DBGin("Can_FreeIrq");
    IRQ_requested[minor] = 0;
#ifdef KVASER_PCICAN

    /* Disable Interrupt on the PCI board only if all channels
     * are not in use */
    if(    IRQ_requested[0] == 0
        && IRQ_requested[1] == 0 
        && IRQ_requested[2] == 0 
        && IRQ_requested[3] == 0 )
    /* and what happens if we only have 2 channels on the board,
       or we have minor == 4, thats a second board ??) */
    {
	unsigned long tmp;
	/* Disable interrupts from card */
	/* printk(" disable PCI interrupt\n"); */
	tmp = inl(pci_resource_start(Can_pcidev[minor], 0) + S5920_INTCSR);
	tmp &= ~INTCSR_ADDON_INTENABLE_M;
	outl(tmp, pci_resource_start(Can_pcidev[minor], 0) + S5920_INTCSR);
    }
#endif

/* printk(" Free IRQ %d  minor %d\n", irq, minor);  */
    free_irq(irq, &Can_minors[minor]);
    DBGout();
    return 0;
}


/*
 * The plain SJA1000 interrupt
 *
 *				RX ISR           TX ISR
 *                              8/0 byte
 *                               _____            _   ___
 *                         _____|     |____   ___| |_|   |__
 *---------------------------------------------------------------------------
 * 1) CPUPentium 75 - 200
 *  75 MHz, 149.91 bogomips
 *  AT-CAN-MINI                 42/27탎            50 탎
 *  CPC-PCI		        35/26탎		   
 * ---------------------------------------------------------------------------
 * 2) AMD Athlon(tm) Processor 1M
 *    2011.95 bogomips
 *  AT-CAN-MINI  std            24/12탎            ?? 탎
 *               ext id           /14탎
 *
 * 
 * 1) 1Byte = (42-27)/8 = 1.87 탎
 * 2) 1Byte = (24-12)/8 = 1.5  탎
 *
 *
 *
 * RX Int with to Receive channels:
 * 1)                _____   ___
 *             _____|     |_|   |__
 *                   30    5  20  탎
 *   first ISR normal length,
 *   time between the two ISR -- short
 *   sec. ISR shorter than first, why? it's the same message
 */

irqreturn_t CAN_Interrupt ( int irq, void *dev_id, struct pt_regs *ptregs )
{
int minor;
int i;
unsigned long flags;
int ext;			/* flag for extended message format */
int irqsrc, dummy;
msg_fifo_t   *RxFifo; 
msg_fifo_t   *TxFifo;
#if CAN_USE_FILTER
msg_filter_t *RxPass;
unsigned int id;
#endif 
#if 0
int first = 0;
#endif 

#if CONFIG_TIME_MEASURE
    outb(0xff, 0x378);  
#endif


    minor = *(int *)dev_id;
    /* printk("CAN - ISR ; minor = %d\n", *(int *)dev_id); */


    RxFifo = &Rx_Buf[minor]; 
    TxFifo = &Tx_Buf[minor];
#if CAN_USE_FILTER
    RxPass = &Rx_Filter[minor];
#endif 

    /* Disable PITA Interrupt */
    /* writel(0x00000000, Can_pitapci_control[minor] + 0x0); */

    irqsrc = CANin(minor, canirq);
    if(irqsrc == 0) {
         /* first call to ISR, it's not for me ! */
#if LINUX_VERSION_CODE >= 0x020500 
	return IRQ_RETVAL(IRQ_NONE);
#else
	goto IRQdone_doneNothing;
#endif
    }
#if defined(CCPC104)
	pc104_irqack();
#endif

    do {
    /* loop as long as the CAN controller shows interrupts */
    DBGprint(DBG_DATA, (" => got IRQ[%d]: 0x%0x\n", minor, irqsrc));
    /* can_dump(minor); */

    do_gettimeofday(&(RxFifo->data[RxFifo->head]).timestamp);

#if 0
    /* how often do we lop through the ISR ? */
    if(first) printk("n = %d\n", first);
    first++;
#endif

    /* preset flags */
    (RxFifo->data[RxFifo->head]).flags =
        		(RxFifo->status & BUF_OVERRUN ? MSG_BOVR : 0);

    /*========== receive interrupt */
    if( irqsrc & CAN_RECEIVE_INT ) {
        /* fill timestamp as first action */
        /* printk(" CAN RX %d\n", minor); */

	dummy  = CANin(minor, frameinfo );
        if( dummy & CAN_RTR ) {
	    (RxFifo->data[RxFifo->head]).flags |= MSG_RTR;
	}

        if( dummy & CAN_EFF ) {
	    (RxFifo->data[RxFifo->head]).flags |= MSG_EXT;
	}
	ext = (dummy & CAN_EFF);
	if(ext) {
	    (RxFifo->data[RxFifo->head]).id =
	          ((unsigned int)(CANin(minor, frame.extframe.canid1)) << 21)
		+ ((unsigned int)(CANin(minor, frame.extframe.canid2)) << 13)
		+ ((unsigned int)(CANin(minor, frame.extframe.canid3)) << 5)
		+ ((unsigned int)(CANin(minor, frame.extframe.canid4)) >> 3);
	} else {
	    (RxFifo->data[RxFifo->head]).id =
        	  ((unsigned int)(CANin(minor, frame.stdframe.canid1 )) << 3) 
        	+ ((unsigned int)(CANin(minor, frame.stdframe.canid2 )) >> 5);
	}
	/* get message length */
        dummy  &= 0x0f;			/* strip length code */ 
        (RxFifo->data[RxFifo->head]).length = dummy;

        dummy %= 9;	/* limit count to 8 bytes */
        for( i = 0; i < dummy; i++) {
            /* SLOW_DOWN_IO;SLOW_DOWN_IO; */
	    if(ext) {
		(RxFifo->data[RxFifo->head]).data[i] =
			CANin(minor, frame.extframe.canxdata[R_OFF * i]);
	    } else {
		(RxFifo->data[RxFifo->head]).data[i] =
			CANin(minor, frame.stdframe.candata[R_OFF * i]);
	    }
	}
	/* mark just written entry as OK and full */
	RxFifo->status = BUF_OK;

        RxFifo->head = ++(RxFifo->head) % MAX_BUFSIZE;

	if(RxFifo->head == RxFifo->tail) {
		printk("CAN[%d] RX: FIFO overrun\n", minor);
		RxFifo->status = BUF_OVERRUN;
        } 
        /*---------- kick the select() call  -*/
        /* This function will wake up all processes
           that are waiting on this event queue,
           that are in interruptible sleep
        */
        wake_up_interruptible(&CanWait[minor]); 

        CANout(minor, cancmd, CAN_RELEASE_RECEIVE_BUFFER );
        if(CANin(minor, canstat) & CAN_DATA_OVERRUN ) {
		 printk("CAN[%d] Rx: Overrun Status \n", minor);
		 CANout(minor, cancmd, CAN_CLEAR_OVERRUN_STATUS );
	}

   }

    /*========== transmit interrupt */
    if( irqsrc & CAN_TRANSMIT_INT ) {
	/* CAN frame successfully sent */
	uint8 tx2reg;
	unsigned int id;

	if(selfreception[minor]) {
	    /* selfreception means, placing the last transmitted frame
	     * in the rx fifo too
	     */

	    /* use time stamp sampled with last INT */
	    last_Tx_object[minor].timestamp
	    		= RxFifo->data[RxFifo->head].timestamp;
	    memcpy(  
		(void *)&RxFifo->data[RxFifo->head],
	    	(void *)&last_Tx_object[minor],
		sizeof(canmsg_t));
	    
	    /* Mark message as 'self sent/received' */
	    RxFifo->data[RxFifo->head].flags |= MSG_SELF;

	    /* increment write index */
	    RxFifo->status = BUF_OK;
	    RxFifo->head = ++(RxFifo->head) % MAX_BUFSIZE;

	    if(RxFifo->head == RxFifo->tail) {
		printk("CAN[%d] RX: FIFO overrun\n", minor);
		RxFifo->status = BUF_OVERRUN;
	    } 
	    /*---------- kick the select() call  -*/
	    /* This function will wake up all processes
	       that are waiting on this event queue,
	       that are in interruptible sleep
	    */
	    wake_up_interruptible(&CanWait[minor]); 

	} /* selfreception */


	if( TxFifo->free[TxFifo->tail] == BUF_EMPTY ) {
	    /* TX FIFO empty, nothing more to sent */
	    /* printk("TXE\n"); */
	    TxFifo->status = BUF_EMPTY;
            TxFifo->active = 0;
	    /* This function will wake up all processes
	       that are waiting on this event queue,
	       that are in interruptible sleep
	    */
	    wake_up_interruptible(&CanOutWait[minor]); 
            goto Tx_done;
	} else {
	    /* printk("TX\n"); */
	}

	/* The TX message FIFO contains other CAN frames to be sent
	 * The next frame in the FIFO is copied into the last_Tx_object
	 * and directly into the hardware of the CAN controller
	 */

        /* enter critical section */
	local_irq_save(flags);

	if(selfreception[minor]) {
	    /* selfreception means, placing the last transmitted frame
	     * in the rx fifo too
	     */
	    /* prepare for next, store mesage that will be transmittet next */
	    memcpy(
	    	(void *)&last_Tx_object[minor],
		(void *)&TxFifo->data[TxFifo->tail],
		sizeof(canmsg_t));
	}

	tx2reg = (TxFifo->data[TxFifo->tail]).length;
	if( (TxFifo->data[TxFifo->tail]).flags & MSG_RTR) {
		tx2reg |= CAN_RTR;
	}
        ext = (TxFifo->data[TxFifo->tail]).flags & MSG_EXT;
        id = (TxFifo->data[TxFifo->tail]).id;
        if(ext) {
	    DBGprint(DBG_DATA, ("---> send ext message \n"));
	    CANout(minor, frameinfo, CAN_EFF + tx2reg);
	    CANout(minor, frame.extframe.canid1, (uint8)(id >> 21));
	    CANout(minor, frame.extframe.canid2, (uint8)(id >> 13));
	    CANout(minor, frame.extframe.canid3, (uint8)(id >> 5));
	    CANout(minor, frame.extframe.canid4, (uint8)(id << 3) & 0xff);
        } else {
	    DBGprint(DBG_DATA, ("---> send std message \n"));
	    CANout(minor, frameinfo, CAN_SFF + tx2reg);
	    CANout(minor, frame.stdframe.canid1, (uint8)((id) >> 3) );
	    CANout(minor, frame.stdframe.canid2, (uint8)(id << 5 ) & 0xe0);
        }


	tx2reg &= 0x0f;		/* restore length only */
	if(ext) {
	    for( i=0; i < tx2reg ; i++) {
		CANout(minor, frame.extframe.canxdata[R_OFF * i],
		    (TxFifo->data[TxFifo->tail]).data[i]);
	    }
        } else {
	    for( i=0; i < tx2reg ; i++) {
		CANout(minor, frame.stdframe.candata[R_OFF * i],
		    (TxFifo->data[TxFifo->tail]).data[i]);
	    }
        }

        CANout(minor, cancmd, CAN_TRANSMISSION_REQUEST );

	TxFifo->free[TxFifo->tail] = BUF_EMPTY; /* now this entry is EMPTY */
	TxFifo->tail = ++(TxFifo->tail) % MAX_BUFSIZE;

        /* leave critical section */
	/* restore_flags(flags); */
	local_irq_restore(flags);
   }
Tx_done:
   /*========== error status */
   if( irqsrc & CAN_ERROR_INT ) {
   int s;
	/* printk("CAN[%d]: error interrupt!\n", minor); */
        TxErr[minor]++;

        /* insert error */
        s = CANin(minor,canstat);
        if(s & CAN_BUS_STATUS ) {
	    (RxFifo->data[RxFifo->head]).flags += MSG_BUSOFF; 
    printk(" MSG_BUSOF\n");
        }
        if(s & CAN_ERROR_STATUS) {
	    (RxFifo->data[RxFifo->head]).flags += MSG_PASSIVE; 
    printk(" MSG_PASSIVE\n");
        }

	(RxFifo->data[RxFifo->head]).id = 0xFFFFFFFF;
        /* (RxFifo->data[RxFifo->head]).length = 0; */
	/* (RxFifo->data[RxFifo->head]).data[i] = 0; */
	RxFifo->status = BUF_OK;
        RxFifo->head = ++(RxFifo->head) % MAX_BUFSIZE;
	if(RxFifo->head == RxFifo->tail) {
		printk("CAN[%d] RX: FIFO overrun\n", minor);
		RxFifo->status = BUF_OVERRUN;
        } 
        /* tell someone that there is a new error message */
        wake_up_interruptible(&CanWait[minor]); 
	
   }
   if( irqsrc & CAN_OVERRUN_INT ) {
   int s;
	printk("CAN[%d]: overrun!\n", minor);
        Overrun[minor]++;

        /* insert error */
        s = CANin(minor,canstat);
        if(s & CAN_DATA_OVERRUN)
	    (RxFifo->data[RxFifo->head]).flags += MSG_OVR; 

	(RxFifo->data[RxFifo->head]).id = 0xFFFFFFFF;
        /* (RxFifo->data[RxFifo->head]).length = 0; */
	/* (RxFifo->data[RxFifo->head]).data[i] = 0; */
	RxFifo->status = BUF_OK;
        RxFifo->head = ++(RxFifo->head) % MAX_BUFSIZE;
	if(RxFifo->head == RxFifo->tail) {
		printk("CAN[%d] RX: FIFO overrun\n", minor);
		RxFifo->status = BUF_OVERRUN;
        } 
        /* tell someone that there is a new error message */
        wake_up_interruptible(  &CanWait[minor] ); 

        CANout(minor, cancmd, CAN_CLEAR_OVERRUN_STATUS );
   } 
   } while( (irqsrc = CANin(minor, canirq)) != 0);

/* IRQdone: */
    DBGprint(DBG_DATA, (" => leave IRQ[%d]\n", minor));

#if defined(CAN4LINUX_PCI) && defined(CPC_PCI)
    /* Interrupt_0_Enable (bit 17) + Int_0_Reset (bit 1) */
    /*  
     Uttenthaler:
      nur 
        writel(0x00020002, Can_pitapci_control[minor] + 0x0);
      als letzte Anweisung in der ISR
     Schoett:
      bei Eintritt
        writel(0x00000000, Can_pitapci_control[minor] + 0x0);
      am ende
        writel(0x00020002, Can_pitapci_control[minor] + 0x0);
    */
    writel(0x00020002, (void __iomem *)Can_pitapci_control[minor] + 0x0);
    writel(0x00020000, (void __iomem *)Can_pitapci_control[minor] + 0x0);
#endif

#if LINUX_VERSION_CODE < 0x020500 
IRQdone_doneNothing:
#endif


#if CONFIG_TIME_MEASURE
    outb(0x00, 0x378);  
#endif

    return IRQ_RETVAL(IRQ_HANDLED);
}


