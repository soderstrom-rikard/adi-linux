/*
 * bfin_serial.h - Blackfin UART/Serial definitions
 *
 * Copyright 2006-2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __BFIN_MACH_SERIAL_H__
#define __BFIN_MACH_SERIAL_H__

#define BFIN_UART_NR_PORTS	1

#if defined(CONFIG_BFIN_UART0_CTSRTS)
# define CONFIG_SERIAL_BFIN_CTSRTS
#endif

#endif
