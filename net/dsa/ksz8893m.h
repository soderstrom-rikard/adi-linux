/*
 * Integrated 3-Port 10/100 Managed Switch with PHYs
 *
 * - KSZ8893M support
 *
 * Copyright 2008-2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __KSZ8893M_H__
#define __KSZ8893M_H__

#define KSZ8893M_PORT_NUM 3
#define KSZ8893M_CPU_PORT 3

#define DEFAULT_PORT_VID  0

/* Simple SPI command set for poking registers */
#define SPI_READ          3
#define SPI_WRITE         2

/* Expected value for MII_PHYSID1 */
#define PHYID_HIGH        0x22
/* Expected value for MII_PHYSID2 */
#define PHYID_LOW         0x1430

/* ChipID0 defines */
#define FAMILY_ID         0x88

/* ChipID1_StartSwitch defines */
#define START_SWITCH      0x01

/* Port3Control0 defines */
#define TAG_INSERTION     0x04

/* GlobalControl9 defines */
#define SPECIAL_TPID_MODE 0x01

/* BMCR Reserved Bits */
#define HP_MDIX           0x0020
#define FORCE_MDI         0x0010
#define DISABLE_MDIX      0x0008
#define DIS_FAR_END_FAULT 0x0004
#define DISABLE_TRANSMIT  0x0002
#define DISABLE_LED       0x0001

/* BMSCR Reserved Bits */
#define PREAMBLE_SUPPRESS 0x0040

enum switch_phy_reg {
	/* Global Registers: 0-15 */
	ChipID0 = 0,
	ChipID1_StartSwitch,
	GlobalControl0,
	GlobalControl1,
	GlobalControl2, /* 4 */
	GlobalControl3,
	GlobalControl4,
	GlobalControl5,
	GlobalControl6, /* 8 */
	GlobalControl7,
	GlobalControl8,
	GlobalControl9,
	GlobalControl10, /* 12 */
	GlobalControl11,
	GlobalControl12,
	GlobalControl13,
	/* Port Registers: 16-95 */
	Port1Control0 = 16,
	Port1Control1,
	Port1Control2,
	Port1Control3,
	Port1Control4, /* 20 */
	Port1Control5,
	Port1Control6,
	Port1Control7,
	Port1Control8, /* 24 */
	Port1Control9,
	Port1PHYSpecialControl_Status,
	Port1LinkMDResult,
	Port1Control12, /* 28 */
	Port1Control13,
	Port1Status0,
	Port1Status1,
	Port2Control0, /* 32 */
	Port2Control1,
	Port2Control2,
	Port2Control3,
	Port2Control4, /* 36 */
	Port2Control5,
	Port2Control6,
	Port2Control7,
	Port2Control8, /* 40 */
	Port2Control9,
	Port2PHYSpecialControl_Status,
	Port2LinkMDResult,
	Port2Control12, /* 44 */
	Port2Control13,
	Port2Status0,
	Port2Status1,
	Port3Control0, /* 48 */
	Port3Control1,
	Port3Control2,
	Port3Control3,
	Port3Control4, /* 52 */
	Port3Control5,
	Port3Control6,
	Port3Control7,
	Port3Control8, /* 56 */
	Port3Control9,
	Reservednotappliedtoport3, /* 58-62 */
	Port3Status1 = 63,
	/* Advanced Control Registers: 96-141 */
	TOSPriorityControlRegister0 = 96,
	TOSPriorityControlRegister1,
	TOSPriorityControlRegister2,
	TOSPriorityControlRegister3,
	TOSPriorityControlRegister4, /* 100 */
	TOSPriorityControlRegister5,
	TOSPriorityControlRegister6,
	TOSPriorityControlRegister7,
	TOSPriorityControlRegister8, /* 104 */
	TOSPriorityControlRegister9,
	TOSPriorityControlRegister10,
	TOSPriorityControlRegister11,
	TOSPriorityControlRegister12, /* 108 */
	TOSPriorityControlRegister13,
	TOSPriorityControlRegister14,
	TOSPriorityControlRegister15,
	MACAddressRegister0 = 112,
	MACAddressRegister1,
	MACAddressRegister2,
	MACAddressRegister3,
	MACAddressRegister4,
	MACAddressRegister5,
	UserDefinedRegister1 = 118,
	UserDefinedRegister2,
	UserDefinedRegister3,
	IndirectAccessControl0 = 121,
	IndirectAccessControl1,
	IndirectDataRegister8 = 123,
	IndirectDataRegister7,
	IndirectDataRegister6,
	IndirectDataRegister5,
	IndirectDataRegister4,
	IndirectDataRegister3,
	IndirectDataRegister2,
	IndirectDataRegister1,
	IndirectDataRegister0,
	DigitalTestingStatus0 = 132,
	DigitalTestingControl0,
	AnalogTestingControl0,
	AnalogTestingControl1,
	AnalogTestingControl2,
	AnalogTestingControl3,
	AnalogTestingStatus,
	AnalogTestingControl4,
	QMDebug1,
	QMDebug2,
};

#endif
