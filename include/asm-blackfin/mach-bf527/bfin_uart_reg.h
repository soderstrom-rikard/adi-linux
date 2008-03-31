/*
 * file:        include/asm-blackfin/mach-bf527/bfin_uart_reg.h
 * based on:
 * author:
 *
 * created:
 * description:
 *	blackfin uart mmr register offset
 * rev:
 *
 * modified:
 *
 *
 * bugs:         enter bugs at http://blackfin.uclinux.org/
 *
 * this program is free software; you can redistribute it and/or modify
 * it under the terms of the gnu general public license as published by
 * the free software foundation; either version 2, or (at your option)
 * any later version.
 *
 * this program is distributed in the hope that it will be useful,
 * but without any warranty; without even the implied warranty of
 * merchantability or fitness for a particular purpose.  see the
 * gnu general public license for more details.
 *
 * you should have received a copy of the gnu general public license
 * along with this program; see the file copying.
 * if not, write to the free software foundation,
 * 59 temple place - suite 330, boston, ma 02111-1307, usa.
 */

#include <linux/serial.h>
#include <asm/dma.h>
#include <asm/portmux.h>

#define NR_PORTS		2

#define OFFSET_THR              0x00	/* Transmit Holding register            */
#define OFFSET_RBR              0x00	/* Receive Buffer register              */
#define OFFSET_DLL              0x00	/* Divisor Latch (Low-Byte)             */
#define OFFSET_IER              0x04	/* Interrupt Enable Register            */
#define OFFSET_DLH              0x04	/* Divisor Latch (High-Byte)            */
#define OFFSET_IIR              0x08	/* Interrupt Identification Register    */
#define OFFSET_LCR              0x0C	/* Line Control Register                */
#define OFFSET_MCR              0x10	/* Modem Control Register               */
#define OFFSET_LSR              0x14	/* Line Status Register                 */
#define OFFSET_MSR              0x18	/* Modem Status Register                */
#define OFFSET_SCR              0x1C	/* SCR Scratch Register                 */
#define OFFSET_GCTL             0x24	/* Global Control Register              */

