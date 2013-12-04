/*
 * DO NOT EDIT THIS FILE
 * This file is under version control at
 *   svn://sources.blackfin.uclinux.org/toolchain/trunk/proc-defs/header-frags/
 * and can be replaced with that version at any time
 * DO NOT EDIT THIS FILE
 *
 * Copyright 2004-2012 Analog Devices Inc.
 * Licensed under the Clear BSD license.
 */

/* This file should be up to date with:
 *  - Revision A, 15/06/2012; ADSP-BF609 Blackfin Processor Anomaly List
 */

#if __SILICON_REVISION__ < 0
# error will not work on BF609 silicon version
#endif

#ifndef _MACH_ANOMALY_H_
#define _MACH_ANOMALY_H_

/* TRU_STAT.ADDRERR and TRU_ERRADDR.ADDR May Not Reflect the Correct Status */
#define ANOMALY_16000003 (1)
/* The EPPI Data Enable (DEN) Signal is Not Functional */
#define ANOMALY_16000004 (__SILICON_REVISION__ < 1)
/* Using L1 Instruction Cache with Parity Enabled is Unreliable */
#define ANOMALY_16000005 (__SILICON_REVISION__ < 1)
/* SEQSTAT.SYSNMI Clears Upon Entering the NMI ISR */
#define ANOMALY_16000006 (__SILICON_REVISION__ < 1)
/* DDR2 Memory Reads May Fail Intermittently */
#define ANOMALY_16000007 (1)
/* Instruction Memory Stalls Can Cause IFLUSH to Fail */
#define ANOMALY_16000008 (1)
/* TestSET Instruction Cannot Be Interrupted */
#define ANOMALY_16000009 (1)
/* IFLUSH Instruction at End of Hardware Loop Causes Infinite Stall */
#define ANOMALY_16000010 (1)
/* False Hardware Error when RETI Points to Invalid Memory */
#define ANOMALY_16000011 (1)
/* Speculative Fetches of Indirect-Pointer Instructions Can Cause False Hardware Errors */
#define ANOMALY_16000012 (1)
/* False Hardware Errors Caused by Fetches at the Boundary of Reserved Memory */
#define ANOMALY_16000013 (1)
/* False Hardware Error from an Access in the Shadow of a Conditional Branch */
#define ANOMALY_16000014 (1)
/* Multi-Issue Instruction with dsp32shiftimm in slot1 and P-reg Store in slot2 Not Supported */
#define ANOMALY_16000015 (1)
/* Speculative Fetches Can Cause Undesired External FIFO Operations */
#define ANOMALY_16000017 (1)
/* RSI Boot Cleanup Routine Does Not Clear Registers */
#define ANOMALY_16000018 (__SILICON_REVISION__ < 1)
/* SPI Master Boot Device Auto-detection Frequency is Set Incorrectly */
#define ANOMALY_16000019 (__SILICON_REVISION__ < 1)
/* rom_SysControl() Fails to Set DDR0_CTL.INIT for Wakeup From Hibernate */
#define ANOMALY_16000020 (__SILICON_REVISION__ < 1)
/* rom_SysControl() Fails to Save and Restore DDR0_PHYCTL3 for Hibernate/Wakeup Sequence */
#define ANOMALY_16000021 (__SILICON_REVISION__ < 1)
/* Boot Code Fails to Enable Parity Fault Detection */
#define ANOMALY_16000022 (__SILICON_REVISION__ < 1)
/* Rom_SysControl Does not Update CGU0_CLKOUTSEL */
#define ANOMALY_16000023 (__SILICON_REVISION__ < 1)
/* Spurious Fault Signaled After Clearing an Externally Generated Fault */
#define ANOMALY_16000024 (1)
/* SPORT May Drive Data Pins During Inactive Channels in Multichannel Mode */
#define ANOMALY_16000025 (1)
/* USB DMA interrupt status do not show the DMA channel interrupt in the DMA ISR */
#define ANOMALY_16000027 (__SILICON_REVISION__ < 1)
/* Default SPI Master Boot Mode Setting is Incorrect */
#define ANOMALY_16000028 (__SILICON_REVISION__ < 1)
/* PPI tDFSPI Timing Does Not Meet Data Sheet Specification */
#define ANOMALY_16000027 (__SILICON_REVISION__ < 1)
/* Interrupted Core Reads of MMRs May Cause Data Loss */
#define ANOMALY_16000030 (__SILICON_REVISION__ < 1)
/* Incorrect Default USB_PLL_OSC.PLLM Value */
#define ANOMALY_16000031 (__SILICON_REVISION__ < 1)
/* Core Reads of System MMRs May Cause the Core to Hang */
#define ANOMALY_16000032 (__SILICON_REVISION__ < 1)
/* PPI Data Underflow on First Word Not Reported in Certain Modes */
#define ANOMALY_16000033 (1)
/* CNV1 Red Pixel Substitution feature not functional in the PVP */
#define ANOMALY_16000034 (__SILICON_REVISION__ < 1)
/* IPF0 Output Port Color Separation feature not functional */
#define ANOMALY_16000035 (__SILICON_REVISION__ < 1)
/* Spurious USB Wake From Hibernate May Occur When USB_VBUS is Low */
#define ANOMALY_16000036 (__SILICON_REVISION__ < 1)
/* Core RAISE 2 Instruction Not Latched When Executed at Priority Level 0, 1, or 2 */
#define ANOMALY_16000037 (__SILICON_REVISION__ < 1)
/* Spurious Unhandled NMI or L1 Memory Parity Error Interrupt May Occur Upon Entering the NMI ISR */
#define ANOMALY_16000038 (__SILICON_REVISION__ < 1)
/* CGU_STAT.PLOCKERR Bit May be Unreliable */
#define ANOMALY_16000039 (1)
/* JTAG Emulator Reads of SDU_IDCODE Alter Register Contents */
#define ANOMALY_16000040 (1)
/* IFLUSH Instruction Causes Parity Error When Parity Is Enabled */
#define ANOMALY_16000041 (1)
/* Instruction Cache Failure When Parity Is Enabled */
#define ANOMALY_16000042 (__SILICON_REVISION__ == 1)

/* Anomalies that don't exist on this proc */
#define ANOMALY_05000158 (0)
#define ANOMALY_05000189 (0)
#define ANOMALY_05000198 (0)
#define ANOMALY_05000230 (0)
#define ANOMALY_05000231 (0)
#define ANOMALY_05000244 (0)
#define ANOMALY_05000263 (0)
#define ANOMALY_05000273 (0)
#define ANOMALY_05000274 (0)
#define ANOMALY_05000278 (0)
#define ANOMALY_05000281 (0)
#define ANOMALY_05000287 (0)
#define ANOMALY_05000311 (0)
#define ANOMALY_05000312 (0)
#define ANOMALY_05000323 (0)
#define ANOMALY_05000363 (0)
#define ANOMALY_05000380 (0)
#define ANOMALY_05000450 (0)
#define ANOMALY_05000480 (0)
#define ANOMALY_05000481 (1)

/* Reuse BF5xx anomalies IDs for the same anomaly in BF60x */
#define ANOMALY_05000491 ANOMALY_16000008
#define ANOMALY_05000477 ANOMALY_16000009
#define ANOMALY_05000443 ANOMALY_16000010
#define ANOMALY_05000461 ANOMALY_16000011
#define ANOMALY_05000426 ANOMALY_16000012
#define ANOMALY_05000310 ANOMALY_16000013
#define ANOMALY_05000245 ANOMALY_16000014
#define ANOMALY_05000074 ANOMALY_16000015
#define ANOMALY_05000416 ANOMALY_16000017


#endif
