/*
 * File:         arch/blackfin/kernel/early_printk.c
 * Author:       Robin Getz <rgetz@blackfin.uclinux.org
 *
 * Created:      03Jul2009
 * Description:  creates a shadow of the __log_buf, which is printed by the
 *		 bootloader if things panic during init
 *
 *               Copyright 2009 Analog Devices Inc.
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
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/string.h>
#include <asm/blackfin.h>
#include <asm/irq_handler.h>
#include <asm/early_printk.h>

#define SHADOW_CONSOLE_START		(0x500)
#define SHADOW_CONSOLE_END		(0x1000)
#define SHADOW_CONSOLE_MAGIC_LOC	(0x4F0)
#define SHADOW_CONSOLE_MAGIC		(0xDEADBEEF)

static __initdata char *shadow_console_buffer = (char *)SHADOW_CONSOLE_START;

static __init void early_shadow_write(struct console *con, const char *s,
				unsigned int n)
{
	/*
	 * save 2 bytes for the double null at the end
	 * once we fail on a long line, make sure we don't write a short line afterwards
	 */
	if ((shadow_console_buffer + n) <= (char *)(SHADOW_CONSOLE_END - 2)) {
		memcpy(shadow_console_buffer, s, n);
		shadow_console_buffer += n;
		shadow_console_buffer[0] = 0;
		shadow_console_buffer[1] = 0;
	} else
		shadow_console_buffer = (char *)SHADOW_CONSOLE_END;
}

static __initdata struct console early_shadow_console = {
	.name = "early_shadow",
	.write = early_shadow_write,
	.flags = CON_BOOT | CON_PRINTBUFFER,
	.index = -1,
	.device = 0,
};

__init void enable_shadow_console(void)
{
	int *loc = (int *)SHADOW_CONSOLE_MAGIC_LOC;

	if (!(early_shadow_console.flags & CON_ENABLED)) {
		register_console(&early_shadow_console);
		/* for now, assume things are going to fail */
		*loc = SHADOW_CONSOLE_MAGIC;
		loc++;
		*loc = SHADOW_CONSOLE_START;
	}
}

static __init int disable_shadow_console(void)
{
	/*
	 * by the time pure_initcall runs, the standard console is enabled,
	 * and the early_console is off, so unset the magic numbers
	 * unregistering the console is taken care of in common code (See
	 * ./kernel/printk:disable_boot_consoles() )
	 */
	int *loc = (int *)SHADOW_CONSOLE_MAGIC_LOC;

	*loc = 0;

	return 0;
}
pure_initcall(disable_shadow_console);
