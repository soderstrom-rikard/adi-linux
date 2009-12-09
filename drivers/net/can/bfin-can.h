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

/*
 * registers offset
 */
#define OFFSET_MB_MASK              0x100
#define OFFSET_MASK_AML             0x0
#define OFFSET_MASK_AMH             0x4
#define OFFSET_MB_OBJ               0x200
#define OFFSET_OBJ_DATA             0x0
#define OFFSET_OBJ_DLC              0x10
#define OFFSET_OBJ_ID0              0x18
#define OFFSET_OBJ_ID1              0x1C
#define OFFSET_CLOCK                0x80
#define OFFSET_TIMING               0x84
#define OFFSET_STATUS               0x8C
#define OFFSET_CEC                  0x90
#define OFFSET_GIS                  0x94
#define OFFSET_GIM                  0x98
#define OFFSET_CONTROL              0xA0
#define OFFSET_INTR                 0xA4
#define OFFSET_ESR                  0xB4
#define OFFSET_MBIM1                0x28
#define OFFSET_MBIM2                0x68
#define OFFSET_MC1                  0x0
#define OFFSET_MC2                  0x40
#define OFFSET_MD1                  0x4
#define OFFSET_MD2                  0x44
#define OFFSET_TRS2                 0x48
#define OFFSET_MBTIF1               0x20
#define OFFSET_MBTIF2               0x60
#define OFFSET_MBRIF1               0x24
#define OFFSET_MBRIF2               0x64

/*
 * transmit and receive channels
 */
#define TRANSMIT_CHL		24
#define RECEIVE_STD_CHL 	0
#define RECEIVE_EXT_CHL 	4
#define RECEIVE_RTR_CHL 	8
#define RECEIVE_EXT_RTR_CHL 	12

/*
 * bfin can private data
 */
struct bfin_can_priv {
	struct can_priv can;	/* must be the first member */
	struct net_device *dev;
	void __iomem *membase;
	int rx_irq;
	int tx_irq;
	int err_irq;
	unsigned short *pin_list;
};

/*
 * read/write CAN registers and messages
 */
#define can_membase(priv)  \
	((priv)->membase)
#define can_channel_membase(priv, channel) \
	((priv)->membase + OFFSET_MB_OBJ + ((channel) << 5))
#define can_mask_membase(priv, channel)  \
	((priv)->membase + OFFSET_MB_MASK + ((channel) << 3))

#define CAN_WRITE_REG(val, addr) \
	writew((val), (addr))

#define CAN_READ_REG(addr) \
	readw((addr))

#define CAN_WRITE_CTRL(priv, off, val) \
	CAN_WRITE_REG(val, can_membase((priv)) + (off))

#define CAN_READ_CTRL(priv, off) \
	CAN_READ_REG(can_membase((priv)) + (off))

#define CAN_WRITE_AML(priv, channel, aml) \
	(CAN_WRITE_REG((aml), can_mask_membase(priv, channel) + OFFSET_MASK_AML))

#define CAN_WRITE_AMH(priv, channel, amh) \
	(CAN_WRITE_REG((amh), can_mask_membase(priv, channel) + OFFSET_MASK_AMH))

#define CAN_WRITE_DLC(priv, channel, length) \
	(CAN_WRITE_REG((length), can_channel_membase(priv, channel) + OFFSET_OBJ_DLC))

#define CAN_READ_DLC(priv, channel) \
	(CAN_READ_REG(can_channel_membase((priv), (channel)) + OFFSET_OBJ_DLC))

#define CAN_READ_OID(priv, channel) \
	((CAN_READ_REG(can_channel_membase((priv), (channel)) + OFFSET_OBJ_ID1) & 0x1ffc) >> 2)

#define CAN_READ_XOID(priv, channel) \
	(((CAN_READ_REG(can_channel_membase((priv), (channel)) + OFFSET_OBJ_ID1) & 0x1fff) << 16) \
	 + ((CAN_READ_REG(can_channel_membase((priv), (channel)) + OFFSET_OBJ_ID0))))

#define CAN_READ_ID1(priv, channel) \
	(CAN_READ_REG(can_channel_membase((priv), (channel)) + OFFSET_OBJ_ID1))

#define CAN_WRITE_ID0(priv, channel, val) \
	CAN_WRITE_REG((val), can_channel_membase((priv), (channel)) + OFFSET_OBJ_ID0)

#define CAN_WRITE_OID(priv, channel, id) \
	CAN_WRITE_REG(((id) << 2) | AME, can_channel_membase((priv), (channel)) + OFFSET_OBJ_ID1)

#define CAN_WRITE_XOID(priv, channel, id)  \
	do { \
		CAN_WRITE_REG((id), can_channel_membase((priv), (channel)) + OFFSET_OBJ_ID0); \
		CAN_WRITE_REG((((id) & 0x1FFF0000) >> 16) + IDE + AME, \
				can_channel_membase((priv), (channel)) + OFFSET_OBJ_ID1); \
	} while (0)

#define CAN_WRITE_OID_RTR(priv, channel, id) \
	CAN_WRITE_REG(((id) << 2) | RTR | AME, can_channel_membase((priv), (channel)) + OFFSET_OBJ_ID1)

#define CAN_WRITE_XOID_RTR(priv, channel, id)  \
	do { \
		CAN_WRITE_REG((id), can_channel_membase((priv), (channel)) + OFFSET_OBJ_ID0); \
		CAN_WRITE_REG((((id) & 0x1FFF0000) >> 16) + IDE + RTR + AME, \
				can_channel_membase((priv), (channel)) + OFFSET_OBJ_ID1); \
	} while (0)

static inline void CAN_WRITE_DATA(struct bfin_can_priv *priv, int channel, u8 *data, int dlc)
{
	int i;
	u16 val;

	for (i = 0; i < 8; i += 2) {
		val = ((7 - i) < dlc ? (data[7 - i]) : 0) +
			((6 - i) < dlc ? (data[6 - i] << 8) : 0);
		CAN_WRITE_REG(val, can_channel_membase((priv), (channel)) + OFFSET_OBJ_DATA + (i << 1));
	}
}

static inline void CAN_READ_DATA(struct bfin_can_priv *priv, int channel, u8 *data, int dlc)
{
	int i;
	u16 val;

	for (i = 0; i < 8; i += 2) {
		val = CAN_READ_REG(can_channel_membase((priv), (channel)) + OFFSET_OBJ_DATA + (i << 1));
		data[7 - i] = (7 - i) < dlc ? val : 0;
		data[6 - i] = (6 - i) < dlc ? (val >> 8) : 0;
	}
}

#endif 		/* __BLACKFIN_CAN_H */
