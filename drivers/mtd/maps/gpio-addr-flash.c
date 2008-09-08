/*
 * drivers/mtd/maps/gpio-addr-flash.c
 *
 * Handle the case where a flash device is mostly addressed using physical
 * line and supplemented by GPIOs.  This way you can hook up say a 8meg flash
 * to a 2meg memory range and use the GPIOs to select a particular range.
 *
 * Copyright 2000 Nicolas Pitre <nico@cam.org>
 * Copyright 2005-2008 Analog Devices Inc.
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#include <asm/gpio.h>
#include <asm/io.h>

#define pr_devinit(fmt, args...) ({ static const __devinitconst char __fmt[] = fmt; printk(__fmt, ## args); })

#define DRIVER_NAME "gpio-addr-flash"

struct async_state {
	struct mtd_info *mtd;
	struct map_info map;
	size_t gpio_count;
	unsigned *gpio_addrs;
	unsigned long win_size;
};

static void gf_set_gpios(struct async_state *state, unsigned long ofs)
{
	size_t i;
	for (i = 0; i < state->gpio_count; ++i)
		gpio_set_value(state->gpio_addrs[i], !!((ofs / state->win_size) & (1 << i)));
}

static map_word gf_read(struct map_info *map, unsigned long ofs)
{
	struct async_state *state = (struct async_state *)map->map_priv_1;
	u16 word;
	map_word test;

	gf_set_gpios(state, ofs);

	word = readw(map->virt + (ofs % state->win_size));
	test.x[0] = word;
	return test;
}

static void gf_copy_from(struct map_info *map, void *to, unsigned long from, ssize_t len)
{
	struct async_state *state = (struct async_state *)map->map_priv_1;

	gf_set_gpios(state, from);

	/* BUG if operation crosss the win_size */
	BUG_ON(!((from + len) % state->win_size <= (from + len)));

	/* operation does not cross the win_size, so one shot it */
	memcpy_fromio(to, map->virt + (from % state->win_size), len);
}

static void gf_write(struct map_info *map, map_word d1, unsigned long ofs)
{
	struct async_state *state = (struct async_state *)map->map_priv_1;
	u16 d;

	gf_set_gpios(state, ofs);

	d = d1.x[0];
	writew(d, map->virt + (ofs % state->win_size));
}

static void gf_copy_to(struct map_info *map, unsigned long to, const void *from, ssize_t len)
{
	struct async_state *state = (struct async_state *)map->map_priv_1;

	gf_set_gpios(state, to);

	/* BUG if operation crosss the win_size */
	BUG_ON(!((to + len) % state->win_size <= (to + len)));

	/* operation does not cross the win_size, so one shot it */
	memcpy_toio(map->virt + (to % state->win_size), from, len);
}

#ifdef CONFIG_MTD_PARTITIONS
static const char *part_probe_types[] = { "cmdlinepart", "RedBoot", NULL };
#endif

static int __devinit gpio_flash_probe(struct platform_device *pdev)
{
	int ret;
	size_t i;
	struct physmap_flash_data *pdata = pdev->dev.platform_data;
	struct resource *memory = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct resource *gpios = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	struct async_state *state;

	state = kzalloc(sizeof(*state) + sizeof(unsigned) * gpios->end, GFP_KERNEL);
	if (!state)
		return -ENOMEM;

	state->gpio_count     = gpios->end;
	state->gpio_addrs     = (void *)(state + 1);
	state->win_size       = memory->end - memory->start + 1;
	memcpy(state->gpio_addrs, (void *)gpios->start, sizeof(unsigned) * state->gpio_count);

	state->map.name       = DRIVER_NAME;
	state->map.read       = gf_read;
	state->map.copy_from  = gf_copy_from;
	state->map.write      = gf_write;
	state->map.copy_to    = gf_copy_to;
	state->map.bankwidth  = pdata->width;
	state->map.size       = state->win_size * (1 << state->gpio_count);
	state->map.virt       = (void __iomem *)memory->start;
	state->map.phys       = memory->start;
	state->map.map_priv_1 = (unsigned long)state;

	platform_set_drvdata(pdev, state);

	for (i = 0; i < state->gpio_count; ++i) {
		if (gpio_request(state->gpio_addrs[i], DRIVER_NAME)) {
			pr_devinit(KERN_ERR DRIVER_NAME ": Failed to request gpio %d\n", state->gpio_addrs[i]);
			while (i--)
				gpio_free(state->gpio_addrs[i]);
			kfree(state);
			return -EBUSY;
		}
		gpio_direction_output(state->gpio_addrs[i], 0);
	}

	pr_devinit(KERN_NOTICE DRIVER_NAME ": probing %d-bit flash bus\n", state->map.bankwidth * 8);
	state->mtd = do_map_probe(memory->name, &state->map);
	if (!state->mtd) {
		for (i = 0; i < state->gpio_count; ++i)
			gpio_free(state->gpio_addrs[i]);
		kfree(state);
		return -ENXIO;
	}

#ifdef CONFIG_MTD_PARTITIONS
	ret = parse_mtd_partitions(state->mtd, part_probe_types, &pdata->parts, 0);
	if (ret > 0) {
		pr_devinit(KERN_NOTICE DRIVER_NAME ": Using commandline partition definition\n");
		add_mtd_partitions(state->mtd, pdata->parts, ret);

	} else if (pdata->nr_parts) {
		pr_devinit(KERN_NOTICE DRIVER_NAME ": Using board partition definition\n");
		add_mtd_partitions(state->mtd, pdata->parts, pdata->nr_parts);

	} else
#endif
	{
		pr_devinit(KERN_NOTICE DRIVER_NAME ": no partition info available, registering whole flash at once\n");
		add_mtd_device(state->mtd);
	}

	return 0;
}

static int __devexit gpio_flash_remove(struct platform_device *pdev)
{
	struct async_state *state = platform_get_drvdata(pdev);
	size_t i;
	for (i = 0; i < state->gpio_count; ++i)
		gpio_free(state->gpio_addrs[i]);
#ifdef CONFIG_MTD_PARTITIONS
	del_mtd_partitions(state->mtd);
#endif
	map_destroy(state->mtd);
	kfree(state);
	return 0;
}

static struct platform_driver gpio_flash_driver = {
	.probe		= gpio_flash_probe,
	.remove		= __devexit_p(gpio_flash_remove),
	.driver		= {
		.name	= DRIVER_NAME,
	},
};

static int __init gpio_flash_init(void)
{
	return platform_driver_register(&gpio_flash_driver);
}
module_init(gpio_flash_init);

static void __exit gpio_flash_exit(void)
{
	platform_driver_unregister(&gpio_flash_driver);
}
module_exit(gpio_flash_exit);

MODULE_AUTHOR("Mike Frysinger <vapier@gentoo.org>");
MODULE_DESCRIPTION("MTD map driver for flashes addressed physically and with gpios");
MODULE_LICENSE("GPL");
