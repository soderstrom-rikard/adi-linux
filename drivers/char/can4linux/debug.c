/* Can_debug
 *
 * can4linux -- LINUX CAN device driver source
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 *
 * Copyright (c) 2001 port GmbH Halle/Saale
 * (c) 2001 Heinz-Jürgen Oertel (oe@port.de)
 *          Claus Schroeter (clausi@chemie.fu-berlin.de)
 *------------------------------------------------------------------
 * $Header: /cvsroot/uclinux533/uClinux-dist/linux-2.6.x/drivers/char/can4linux/debug.c,v 1.2 2006/03/30 15:21:45 hennerich Exp $
 *
 *--------------------------------------------------------------------------
 *
 *
 *
 *
 *
 */
#include "defs.h"


/* default debugging level */

#if DEBUG
unsigned int   dbgMask  = 0;
#else
unsigned int   dbgMask  = 0;
#endif


