/*
 * Blackfin On-Chip CAN Driver
 *
 * Copyright 2004-2009 Analog Devices Inc.
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __BLACKFIN_CAN_H
#define __BLACKFIN_CAN_H

#include <asm/io.h>

/* We use only one transmit object for all messages to be transmitted */
#define TRANSMIT_OBJ		24
#define RECEIVE_STD_OBJ 	0
#define RECEIVE_EXT_OBJ 	4
#define RECEIVE_RTR_OBJ 	8
#define RECEIVE_EXT_RTR_OBJ 	12

/* Mailbox acceptance mask registers */
struct can_mask {
	u16 aml;		/* mailbox acceptance mask low		*/
	u16 dummy1;
	u16 amh;		/* mailbox acceptance mask high		*/
	u16 dummy2;
};

#define CAN_MASK \
	((struct can_mask *)CAN_AM00L)

/*
 * Macros to handle BlackFin CAN message objects
 *
 * Structure for a single CAN object
 * A total of 32 such object structures exists (starting at CAN_BASE + 0x200)
 * ( 0xFFC02C00 )
 */

struct can_obj {
	u16 msg[8];     	/* Message Data 0 .. 7   		*/
	u16 dlc;		/* data length code			*/
	u16 dummy1;
	u16 tsv;		/* time stamp value			*/
	u16 dummy2;
	u16 id0;		/* MB ID0 register			*/
	u16 dummy3;
	u16 id1;		/* MB ID1 register			*/
	u16 dummy4;
};

/* CAN object definition */
#define CAN_OBJ  \
	((struct can_obj *)CAN_MB00_DATA0)

/*
 * macros to read/write can registers
 */
#define CAN_WRITE_REG(val, addr) \
	writew((val), (u16 *)(addr))

#define CAN_READ_REG(addr) \
	readw((u16 *)(addr))
/*
 * macros to read/write data length code
 */
#define CAN_WRITE_DLC(channel, length) \
	(CAN_WRITE_REG((length), &CAN_OBJ[channel].dlc))

#define CAN_READ_DLC(channel) \
	(CAN_READ_REG(&CAN_OBJ[channel].dlc))

/*
 * CAN_READ_OID(obj) is a macro to read the CAN-ID of the specified object.
 * It delivers the value as 16 bit from the standard ID registers.
 */
#define CAN_READ_OID(channel) \
	((CAN_READ_REG(&CAN_OBJ[channel].id1) & 0x1ffc) >> 2)

#define CAN_READ_XOID(channel) \
	(((CAN_READ_REG(&CAN_OBJ[channel].id1) & 0x1fff) << 16) \
	 + ((CAN_READ_REG(&CAN_OBJ[channel].id0))))

/*
 * CAN_WRITE_OID(obj, id) is a macro to write the CAN-ID
 * of the specified object with identifier id.
 * CAN_WRITE_XOID(obj, id) is a macro to write the extended CAN-ID
 */
#define CAN_WRITE_OID(channel, id) \
	CAN_WRITE_REG(((id) << 2) | AME, &CAN_OBJ[channel].id1)

#define CAN_WRITE_XOID(channel, id)  \
	do { \
		CAN_WRITE_REG((id), &CAN_OBJ[channel].id0); \
		CAN_WRITE_REG((((id) & 0x1FFF0000) >> 16) + IDE + AME, &CAN_OBJ[channel].id1); \
	} while (0)

/*
 * CAN_WRITE_OID_RTR(obj, id) is a macro to write the CAN-ID
 * of the specified object with identifier id and set the RTR Bit.
 */
#define CAN_WRITE_OID_RTR(channel, id) \
	CAN_WRITE_REG(((id) << 2) | RTR | AME, &CAN_OBJ[channel].id1)

#define CAN_WRITE_XOID_RTR(channel, id)  \
	do { \
		CAN_WRITE_REG((id), &CAN_OBJ[channel].id0); \
		CAN_WRITE_REG((((id) & 0x1FFF0000) >> 16) + IDE + RTR + AME, &CAN_OBJ[channel].id1); \
	} while (0)

/*
 * bfin can private data
 */

struct bfin_can_priv {
	struct can_priv can;	/* must be the first member */
	struct sk_buff *echo_skb;
	struct net_device *dev;
	int rx_irq;
	int tx_irq;
	int err_irq;
	unsigned short *pin_list;
};

#endif 		/* __BLACKFIN_CAN_H */
