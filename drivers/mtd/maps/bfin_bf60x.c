/*
 * Blackfin BF60x norflash map driver
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/concat.h>
#include <linux/io.h>

#define MAX_RESOURCES		4

struct bf60x_flash_info {
	struct mtd_info		*mtd[MAX_RESOURCES];
	struct mtd_info		*cmtd;
	struct map_info		map[MAX_RESOURCES];
};

static int bf60x_flash_remove(struct platform_device *dev)
{
	struct bf60x_flash_info *info;
	struct physmap_flash_data *bf60x_data;
	int i;

	info = platform_get_drvdata(dev);
	if (info == NULL)
		return 0;
	platform_set_drvdata(dev, NULL);

	bf60x_data = dev->dev.platform_data;

	if (info->cmtd) {
		mtd_device_unregister(info->cmtd);
		if (info->cmtd != info->mtd[0])
			mtd_concat_destroy(info->cmtd);
	}

	for (i = 0; i < MAX_RESOURCES; i++) {
		if (info->mtd[i] != NULL)
			map_destroy(info->mtd[i]);
	}

	if (bf60x_data->exit)
		bf60x_data->exit(dev);

	return 0;
}

static void bf60x_set_vpp(struct map_info *map, int state)
{
	struct platform_device *pdev;
	struct physmap_flash_data *bf60x_data;

	pdev = (struct platform_device *)map->map_priv_1;
	bf60x_data = pdev->dev.platform_data;

	if (bf60x_data->set_vpp)
		bf60x_data->set_vpp(pdev, state);
}

static const char *rom_probe_types[] = {
					"cfi_probe",
					"jedec_probe",
					"qinfo_probe",
					"map_rom",
					NULL };
static const char *part_probe_types[] = { "cmdlinepart", "RedBoot", "afs",
					  NULL };

static inline void bf60x_write16(struct map_info *map, const map_word datum,
			unsigned long ofs)
{
	int tmp;
	tmp = bfin_read32(SMC_B0CTL);
	bfin_write32(SMC_B0CTL, tmp | 0x2000);
	SSYNC();

	__raw_writew(datum.x[0], map->virt + ofs);

	tmp = bfin_read32(SMC_B0CTL);
	bfin_write32(SMC_B0CTL, tmp & ~0x2000);
	SSYNC();
}

static int bf60x_flash_probe(struct platform_device *dev)
{
	struct physmap_flash_data *bf60x_data;
	struct bf60x_flash_info *info;
	const char **probe_type;
	int err = 0;
	int i;
	int devices_found = 0;

	bf60x_data = dev->dev.platform_data;
	if (bf60x_data == NULL)
		return -ENODEV;

	info = devm_kzalloc(&dev->dev, sizeof(struct bf60x_flash_info),
			    GFP_KERNEL);
	if (info == NULL) {
		err = -ENOMEM;
		goto err_out;
	}

	if (bf60x_data->init) {
		err = bf60x_data->init(dev);
		if (err)
			goto err_out;
	}

	platform_set_drvdata(dev, info);

	for (i = 0; i < dev->num_resources; i++) {
		printk(KERN_NOTICE "bf60x platform flash device: %.8llx at %.8llx\n",
		       (unsigned long long)resource_size(&dev->resource[i]),
		       (unsigned long long)dev->resource[i].start);

		if (!devm_request_mem_region(&dev->dev,
			dev->resource[i].start,
			resource_size(&dev->resource[i]),
			dev_name(&dev->dev))) {
			dev_err(&dev->dev, "Could not reserve memory region\n");
			err = -ENOMEM;
			goto err_out;
		}

		info->map[i].name = dev_name(&dev->dev);
		info->map[i].phys = dev->resource[i].start;
		info->map[i].size = resource_size(&dev->resource[i]);
		info->map[i].bankwidth = bf60x_data->width;
		info->map[i].set_vpp = bf60x_set_vpp;
		info->map[i].pfow_base = bf60x_data->pfow_base;
		info->map[i].map_priv_1 = (unsigned long)dev;

		info->map[i].virt = devm_ioremap(&dev->dev, info->map[i].phys,
						 info->map[i].size);
		if (info->map[i].virt == NULL) {
			dev_err(&dev->dev, "Failed to ioremap flash region\n");
			err = -EIO;
			goto err_out;
		}

		simple_map_init(&info->map[i]);
		info->map[i].read = inline_map_read;
		info->map[i].write = bf60x_write16;
		info->map[i].copy_from = inline_map_copy_from;
		info->map[i].copy_to = inline_map_copy_to;

		probe_type = rom_probe_types;
		if (bf60x_data->probe_type == NULL) {
			for (; info->mtd[i] == NULL && *probe_type != NULL; probe_type++)
				info->mtd[i] = do_map_probe(*probe_type, &info->map[i]);
		} else
			info->mtd[i] = do_map_probe(bf60x_data->probe_type, &info->map[i]);

		if (info->mtd[i] == NULL) {
			dev_err(&dev->dev, "map_probe failed\n");
			err = -ENXIO;
			goto err_out;
		} else {
			devices_found++;
		}
		info->mtd[i]->owner = THIS_MODULE;
		info->mtd[i]->dev.parent = &dev->dev;
	}

	if (devices_found == 1) {
		info->cmtd = info->mtd[0];
	} else if (devices_found > 1) {
		/*
		 * We detected multiple devices. Concatenate them together.
		 */
		info->cmtd = mtd_concat_create(info->mtd, devices_found, dev_name(&dev->dev));
		if (info->cmtd == NULL)
			err = -ENXIO;
	}
	if (err)
		goto err_out;

	mtd_device_parse_register(info->cmtd, part_probe_types, 0,
				  bf60x_data->parts, bf60x_data->nr_parts);
	return 0;

err_out:
	bf60x_flash_remove(dev);
	return err;
}

#ifdef CONFIG_PM
static void bf60x_flash_shutdown(struct platform_device *dev)
{
	struct bf60x_flash_info *info = platform_get_drvdata(dev);
	int i;

	for (i = 0; i < MAX_RESOURCES && info->mtd[i]; i++)
		if (info->mtd[i]->suspend && info->mtd[i]->resume)
			if (info->mtd[i]->suspend(info->mtd[i]) == 0)
				info->mtd[i]->resume(info->mtd[i]);
}
#else
#define bf60x_flash_shutdown NULL
#endif

static struct platform_driver bf60x_flash_driver = {
	.probe		= bf60x_flash_probe,
	.remove		= bf60x_flash_remove,
	.shutdown	= bf60x_flash_shutdown,
	.driver		= {
		.name	= "bf60x-flash",
		.owner	= THIS_MODULE,
	},
};

static int __init bf60x_init(void)
{
	return platform_driver_register(&bf60x_flash_driver);
}

static void __exit bf60x_exit(void)
{
	platform_driver_unregister(&bf60x_flash_driver);
}

module_init(bf60x_init);
module_exit(bf60x_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bob Liu <lliubbo@gmail.com>");
MODULE_DESCRIPTION("BF60x MTD map driver");
