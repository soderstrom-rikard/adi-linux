/*
 * bfin_serial.h - Blackfin UART/Serial definitions
 *
 * Copyright 2006-2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __BFIN_MACH_SERIAL_H__
#define __BFIN_MACH_SERIAL_H__

#define BFIN_UART_NR_PORTS	4

#if defined(CONFIG_BFIN_UART0_CTSRTS) || \
    defined(CONFIG_BFIN_UART1_CTSRTS) || \
    defined(CONFIG_BFIN_UART2_CTSRTS) || \
    defined(CONFIG_BFIN_UART3_CTSRTS)
# define CONFIG_SERIAL_BFIN_HARD_CTSRTS
#endif

#define BFIN_UART_BF54X_STYLE

#endif
