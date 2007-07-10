/*
 * Common header file for blackfin family of processors.
 *
 */

#ifndef _PORTMUX_H_
#define _PORTMUX_H_

#define P_IDENT(x)	((x) & 0x1FF)
#define P_FUNCT(x)	(((x) & 0x3) << 9)
#define P_FUNCT2MUX(x)	(((x) >> 9) & 0x3)
#define P_DEFINED	0x8000
#define P_UNDEF		0x4000
#define P_MAYSHARE	0x2000

#include <asm/gpio.h>
#include <asm/mach/portmux.h>


#endif				/* _PORTMUX_H_ */
