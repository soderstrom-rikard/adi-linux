/*
 * bfin_crc.h - interface to Blackfin CRC controllers
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __BFIN_CRC_H__
#define __BFIN_CRC_H__

/* Function driver which use sport must initialize the structure */
struct crc_config {
	/* Input data address */
	unsigned long in_addr;
	/* Output data address */
	unsigned long out_addr;
	/* Input or output size */
	unsigned long crc_datacnt;
	/* Value to compare with input data or CRC of input data*/
	unsigned long crc_compare;
	/* CRC calculated from the input data */
	unsigned long crc_result;
	/* Data to fill */
	unsigned long fillval;
	/* CRC mirror flags */
	unsigned int bitmirr:1;
	unsigned int bytmirr:1;
	unsigned int w16swp:1;
	unsigned int fdsel:1;
	unsigned int rsltmirr:1;
	unsigned int polymirr:1;
	unsigned int cmpmirr:1;
};

/* Userspace interface */
#define CRC_IOC_MAGIC		'C'
#define CRC_IOC_CALC_CRC	_IOWR('C', 0x01, unsigned crc_config)
#define CRC_IOC_MEMCPY_CRC	_IOWR('C', 0x02, unsigned crc_config)
#define CRC_IOC_VERIFY_VAL	_IOWR('C', 0x03, unsigned crc_config)
#define CRC_IOC_FILL_VAL	_IOWR('C', 0x04, unsigned crc_config)


#ifdef __KERNEL__

#include <linux/types.h>

struct crc_register {
	u32 control;
	u32 datacnt;
	u32 datacntrld;
	u32 __pad_1[2];
	u32 compare;
	u32 fillval;
	u32 datafifo;
	u32 intren;
	u32 intrenset;
	u32 intrenclr;
	u32 poly;
	u32 __pad_2[4];
	u32 status;
	u32 datacntcap;
	u32 __pad_3;
	u32 result;
	u32 curresult;
	u32 __pad_4[3];
	u32 revid;
};

struct bfin_snd_platform_data {
	const unsigned short *pin_req;
};

/* CRC_STATUS Masks */
#define CMPERR			0x00000002	/* Compare error */
#define DCNTEXP			0x00000010	/* datacnt register expired */
#define IBR			0x00010000	/* Input buffer ready */
#define OBR			0x00020000	/* Output buffer ready */
#define IRR			0x00040000	/* Immediate result readt */
#define LUTDONE			0x00080000	/* Look-up table generation done */
#define FSTAT			0x00700000	/* FIFO status */
#define MAX_FIFO		4		/* Max fifo size */

/* CRC_CONTROL Masks */
#define BLKEN			0x00000001	/* Block enable */
#define OPMODE			0x000000F0	/* Operation mode */
#define MODE_DMACPY_CRC		1		/* MTM CRC compute and compare */
#define MODE_DATA_FILL		2		/* MTM data fill */
#define MODE_CALC_CRC		3		/* MSM CRC compute and compare */
#define MODE_DATA_VERIFY	4		/* MSM data verify */
#define AUTOCLRZ		0x00000100	/* Auto clear to zero */
#define AUTOCLRF		0x00000200	/* Auto clear to one */
#define OBRSTALL		0x00001000	/* Stall on output buffer ready */
#define IRRSTALL		0x00002000	/* Stall on immediate result ready */
#define BITMIRR			0x00010000	/* Mirror bits within each byte of 32-bit input data */
#define BYTMIRR			0x00020000	/* Mirror bytes of 32-bit input data */
#define W16SWP			0x00040000	/* Mirror uppper and lower 16-bit word of 32-bit input data */
#define FDSEL			0x00080000	/* FIFO is written after input data is mirrored */
#define RSLTMIRR		0x00100000	/* CRC result registers are mirrored. */
#define POLYMIRR		0x00200000	/* CRC poly register is mirrored. */
#define CMPMIRR			0x00400000	/* CRC compare register is mirrored. */

#endif
