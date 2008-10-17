/*
 * File:         arch/blackfin/mach-bf538/ints-priority.c
 * Based on:     arch/blackfin/mach-bf533/ints-priority.c
 * Author:       Michael Hennerich
 *
 * Created:
 * Description:  Set up the interrupt priorities
 *
 * Modified:
 *               Copyright 2008 Analog Devices Inc.
 *
 * Bugs:         Enter bugs at http://blackfin.uclinux.org/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/module.h>
#include <linux/irq.h>
#include <asm/blackfin.h>

void __init program_IAR(void)
{

	/* Program the IAR0 Register with the configured priority */
	bfin_write_SIC_IAR0(((CONFIG_IRQ_PLL_WAKEUP - 7) << IRQ_PLL_WAKEUP_POS) |
			((CONFIG_IRQ_DMA0_ERROR - 7) << IRQ_DMA0_ERROR_POS) |
			((CONFIG_IRQ_PPI_ERROR - 7) << IRQ_PPI_ERROR_POS) |
			((CONFIG_IRQ_SPORT0_ERROR - 7) << IRQ_SPORT0_ERROR_POS) |
			((CONFIG_IRQ_SPORT1_ERROR - 7) << IRQ_SPORT1_ERROR_POS) |
			((CONFIG_IRQ_SPI0_ERROR - 7) << IRQ_SPI0_ERROR_POS) |
			((CONFIG_IRQ_UART0_ERROR - 7) << IRQ_UART0_ERROR_POS) |
			((CONFIG_IRQ_RTC - 7) << IRQ_RTC_POS));

	bfin_write_SIC_IAR1(((CONFIG_IRQ_PPI - 7) << IRQ_PPI_POS) |
			((CONFIG_IRQ_SPORT0_RX - 7) << IRQ_SPORT0_RX_POS) |
			((CONFIG_IRQ_SPORT0_TX - 7) << IRQ_SPORT0_TX_POS) |
			((CONFIG_IRQ_SPORT1_RX - 7) << IRQ_SPORT1_RX_POS) |
			((CONFIG_IRQ_SPORT1_TX - 7) << IRQ_SPORT1_TX_POS) |
			((CONFIG_IRQ_SPI0 - 7) << IRQ_SPI0_POS) |
			((CONFIG_IRQ_UART0_RX - 7) << IRQ_UART0_RX_POS) |
			((CONFIG_IRQ_UART0_TX - 7) << IRQ_UART0_TX_POS));

	bfin_write_SIC_IAR2(((CONFIG_IRQ_TMR0 - 7) << IRQ_TMR0_POS) |
			((CONFIG_IRQ_TMR1 - 7) << IRQ_TMR1_POS) |
			((CONFIG_IRQ_TMR2 - 7) << IRQ_TMR2_POS) |
			((CONFIG_IRQ_PORTF_INTA - 7) << IRQ_PORTF_INTA_POS) |
			((CONFIG_IRQ_PORTF_INTB - 7) << IRQ_PORTF_INTB_POS) |
			((CONFIG_IRQ_MEM0_DMA0 - 7) << IRQ_MEM0_DMA0_POS) |
			((CONFIG_IRQ_MEM0_DMA1 - 7) << IRQ_MEM0_DMA1_POS) |
			((CONFIG_IRQ_WATCH - 7) << IRQ_WATCH_POS));

	bfin_write_SIC_IAR3(((CONFIG_IRQ_DMA1_ERROR - 7) << IRQ_DMA1_ERROR_POS) |
			((CONFIG_IRQ_SPORT2_ERROR - 7) << IRQ_SPORT2_ERROR_POS) |
			((CONFIG_IRQ_SPORT3_ERROR - 7) << IRQ_SPORT3_ERROR_POS) |
			((CONFIG_IRQ_SPI1_ERROR - 7) << IRQ_SPI1_ERROR_POS) |
			((CONFIG_IRQ_SPI2_ERROR - 7) << IRQ_SPI2_ERROR_POS) |
			((CONFIG_IRQ_UART1_ERROR - 7) << IRQ_UART1_ERROR_POS) |
			((CONFIG_IRQ_UART2_ERROR - 7) << IRQ_UART2_ERROR_POS));

	bfin_write_SIC_IAR4(((CONFIG_IRQ_CAN_ERROR - 7) << IRQ_CAN_ERROR_POS) |
			((CONFIG_IRQ_SPORT2_RX - 7) << IRQ_SPORT2_RX_POS) |
			((CONFIG_IRQ_SPORT2_TX - 7) << IRQ_SPORT2_TX_POS) |
			((CONFIG_IRQ_SPORT3_RX - 7) << IRQ_SPORT3_RX_POS) |
			((CONFIG_IRQ_SPORT3_TX - 7) << IRQ_SPORT3_TX_POS) |
			((CONFIG_IRQ_SPI1 - 7) << IRQ_SPI1_POS));

	bfin_write_SIC_IAR5(((CONFIG_IRQ_SPI2 - 7) << IRQ_SPI2_POS) |
			((CONFIG_IRQ_UART1_RX - 7) << IRQ_UART1_RX_POS) |
			((CONFIG_IRQ_UART1_TX - 7) << IRQ_UART1_TX_POS) |
			((CONFIG_IRQ_UART2_RX - 7) << IRQ_UART2_RX_POS) |
			((CONFIG_IRQ_UART2_TX - 7) << IRQ_UART2_TX_POS) |
			((CONFIG_IRQ_TWI0 - 7) << IRQ_TWI0_POS) |
			((CONFIG_IRQ_TWI1 - 7) << IRQ_TWI1_POS) |
			((CONFIG_IRQ_CAN_RX - 7) << IRQ_CAN_RX_POS));

	bfin_write_SIC_IAR6(((CONFIG_IRQ_CAN_TX - 7) << IRQ_CAN_TX_POS) |
			((CONFIG_IRQ_MEM1_DMA0 - 7) << IRQ_MEM1_DMA0_POS) |
			((CONFIG_IRQ_MEM1_DMA1 - 7) << IRQ_MEM1_DMA1_POS));

	SSYNC();
}
