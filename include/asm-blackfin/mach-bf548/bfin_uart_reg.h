/*
 * file:        include/asm-blackfin/mach-bf548/bfin_uart_reg.h
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

#define NR_PORTS		4

#define OFFSET_DLL              0x00	/* Divisor Latch (Low-Byte)             */
#define OFFSET_DLH              0x04	/* Divisor Latch (High-Byte)            */
#define OFFSET_GCTL             0x08	/* Global Control Register              */
#define OFFSET_LCR              0x0C	/* Line Control Register                */
#define OFFSET_MCR              0x10	/* Modem Control Register               */
#define OFFSET_LSR              0x14	/* Line Status Register                 */
#define OFFSET_MSR              0x18	/* Modem Status Register                */
#define OFFSET_SCR              0x1C	/* SCR Scratch Register                 */
#define OFFSET_IER_SET          0x20	/* Set Interrupt Enable Register        */
#define OFFSET_IER_CLEAR        0x24	/* Clear Interrupt Enable Register      */
#define OFFSET_THR              0x28	/* Transmit Holding register            */
#define OFFSET_RBR              0x2C	/* Receive Buffer register              */

