/*
 * Driver for ST PSD4256G flash
 *
 * Copyright 2004-2009 Analog Devices Inc.
 * Licensed under the GPL-2 or later.
 */

#define pr_fmt(fmt) "stm_flash: " fmt

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mtd/map.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/flashchip.h>
#include <linux/sched.h>
#include <linux/types.h>

/* Addresses */
#define ADDR_MANUFACTURER		0x0000
#define ADDR_DEVICE_ID			0x0002
#define ADDR_UNLOCK_1			0x0AAA
#define ADDR_UNLOCK_2			0x0554

/* Commands */
#define CMD_UNLOCK_DATA_1		0x00aa
#define CMD_UNLOCK_DATA_2		0x0055
#define CMD_MANUFACTURER_UNLOCK_DATA	0x0090
#define CMD_PROGRAM_UNLOCK_DATA		0x00A0
#define CMD_RESET_DATA			0x00F0
#define CMD_SECTOR_ERASE_UNLOCK_DATA	0x0080
#define CMD_SECTOR_ERASE_UNLOCK_DATA_2	0x0030

#define D6_MASK 0x40

struct stm_flash_private {
	unsigned long chipshift;
	struct flchip chip;
};

static void send_unlock(struct map_info *map, unsigned long base)
{
	map_word test;
	test.x[0] = 0x00aa;
	map_write(map, test, base + ADDR_UNLOCK_1);
	test.x[0] = 0x0055;
	map_write(map, test, base + ADDR_UNLOCK_2);
}

static void send_cmd(struct map_info *map, unsigned long base,
		     unsigned long cmd)
{
	map_word test;
	test.x[0] = cmd;
	send_unlock(map, base);
	map_write(map, test, base + ADDR_UNLOCK_1);
}

static void send_cmd_to_addr(struct map_info *map, unsigned long base,
			     unsigned long cmd, unsigned long addr)
{
	map_word test;
	test.x[0] = cmd;
	send_unlock(map, base);
	map_write(map, test, addr);
}

static void stm_flash_sync(struct mtd_info *mtd)
{
	struct map_info *map = mtd->priv;
	struct stm_flash_private *private = map->fldrv_priv;
	struct flchip *chip;
	DECLARE_WAITQUEUE(wait, current);

	chip = &private->chip;

 retry:
		spin_lock_bh(chip->mutex);

		switch (chip->state) {
		case FL_READY:
		case FL_STATUS:
		case FL_CFI_QUERY:
		case FL_JEDEC_QUERY:
			chip->oldstate = chip->state;
			chip->state = FL_SYNCING;
		case FL_SYNCING:
			spin_unlock_bh(chip->mutex);
			break;
		default:
			/* Not an idle state */
			add_wait_queue(&chip->wq, &wait);

			spin_unlock_bh(chip->mutex);
			schedule();
			remove_wait_queue(&chip->wq, &wait);

			goto retry;
		}

	chip = &private->chip;

	spin_lock_bh(chip->mutex);

	if (chip->state == FL_SYNCING) {
		chip->state = chip->oldstate;
		wake_up(&chip->wq);
	}
	spin_unlock_bh(chip->mutex);
}

static int read_one_chip(struct map_info *map, struct flchip *chip,
			loff_t addr, size_t len, unsigned char *buf)
{
	DECLARE_WAITQUEUE(wait, current);
	unsigned long timeo = jiffies + HZ;

 retry:
	spin_lock_bh(chip->mutex);

	if (chip->state != FL_READY) {
		pr_info("waiting for chip to read, state = %d\n",
			chip->state);
		set_current_state(TASK_UNINTERRUPTIBLE);
		add_wait_queue(&chip->wq, &wait);

		spin_unlock_bh(chip->mutex);

		schedule();
		remove_wait_queue(&chip->wq, &wait);

		if (signal_pending(current))
			return -EINTR;

		timeo = jiffies + HZ;

		goto retry;
	}

	addr += chip->start;

	chip->state = FL_READY;

	map_copy_from(map, buf, addr, len);

	wake_up(&chip->wq);
	spin_unlock_bh(chip->mutex);

	return 0;
}

static int stm_flash_read(struct mtd_info *mtd, loff_t from, size_t len,
			  size_t *retlen, unsigned char *buf)
{
	struct map_info *map = mtd->priv;
	struct stm_flash_private *private = map->fldrv_priv;
	unsigned long offset;
	int chipnum, ret = 0;

	if ((from + len) > mtd->size) {
		pr_warning("read request past end of device (0x%lx)\n",
			   (unsigned long)from + len);
		return -EINVAL;
	}

	/* Offset within the first chip that the first read should start. */
	chipnum = (from >> private->chipshift);
	offset = from - (chipnum << private->chipshift);

	*retlen = 0;

	while (len) {
		unsigned long this_len;

		if ((len + offset - 1) >> private->chipshift)
			this_len = (1 << private->chipshift) - offset;
		else
			this_len = len;

		ret = read_one_chip(map, &private->chip, offset,
				this_len, buf);

		if (ret)
			break;

		*retlen += this_len;
		len -= this_len;
		buf += this_len;

		offset = 0;
		chipnum++;
	}

	return ret;
}

static int flash_is_busy(struct map_info *map, unsigned long addr)
{
	unsigned short toggled;
	map_word read11, read21;

	read11 = map_read(map, addr);
	read21 = map_read(map, addr);

	toggled = (unsigned short)read11.x[0] ^ (unsigned short)read21.x[0];

	toggled &= (((unsigned short)1) << 6);
	return toggled;
}

static int write_one_word(struct map_info *map, struct flchip *chip,
		unsigned long addr, unsigned long datum)
{
	unsigned long timeo = jiffies + HZ;
	DECLARE_WAITQUEUE(wait, current);
	int ret = 0;
	int times_left;
	map_word test;

 retry:
	spin_lock_bh(chip->mutex);

	if (chip->state != FL_READY) {
		pr_info("%s: waiting for chip to write, state = %d\n",
			map->name, chip->state);
		set_current_state(TASK_UNINTERRUPTIBLE);
		add_wait_queue(&chip->wq, &wait);

		spin_unlock_bh(chip->mutex);

		schedule();
		remove_wait_queue(&chip->wq, &wait);
		pr_info("%s: woke up to write\n", map->name);
		if (signal_pending(current))
			return -EINTR;

		timeo = jiffies + HZ;

		goto retry;
	}

	chip->state = FL_WRITING;

	addr += chip->start;

	send_cmd(map, chip->start, CMD_PROGRAM_UNLOCK_DATA);

	test.x[0] = datum;
	map_write(map, test, addr);

	times_left = 50000;
	while (times_left-- && flash_is_busy(map, addr)) {
		if (need_resched()) {
			spin_unlock_bh(chip->mutex);
			schedule();
			spin_lock_bh(chip->mutex);
		}
	}

	if (!times_left) {
		pr_warning("write to 0x%lx timed out!\n", addr);
		ret = -EIO;
	} else {
		unsigned long verify;
		map_word test;

		test = map_read(map, addr);
		verify = test.x[0];
		if (verify != datum) {
			pr_warning("write to 0x%lx failed "
				   "datum = %lx, verify = %lx\n",
				   addr, datum, verify);
			ret = -EIO;
		}
	}
	chip->state = FL_READY;
	wake_up(&chip->wq);
	spin_unlock_bh(chip->mutex);

	return ret;
}

static int stm_flash_write(struct mtd_info *mtd, loff_t to, size_t len,
			   size_t *retlen, const unsigned char *buf)
{
	struct map_info *map = mtd->priv;
	struct stm_flash_private *private = map->fldrv_priv;
	int ret = 0;
	int chipnum;
	unsigned long offset;

	*retlen = 0;
	if (!len)
		return 0;

	chipnum = to >> private->chipshift;
	offset = to - (chipnum << private->chipshift);

	/* If it's not bus-aligned, do the first byte write. */
	if (offset & (map->bankwidth - 1)) {
		unsigned long bus_offset = offset & ~(map->bankwidth - 1);
		int i = offset - bus_offset;
		int n = 0;
		unsigned char tmp_buf[4];
		unsigned long datum;

		map_copy_from(map, tmp_buf,
			       bus_offset + private->chip.start,
			       map->bankwidth);
		while (len && i < map->bankwidth)
			tmp_buf[i++] = buf[n++], len--;

		if (map->bankwidth == 2)
			datum = *(__u16 *)tmp_buf;
		else if (map->bankwidth == 4)
			datum = *(__u32 *)tmp_buf;
		else
			return -EINVAL;

		ret = write_one_word(map, &private->chip, bus_offset,
				datum);

		if (ret)
			return ret;

		offset += n;
		buf += n;
		(*retlen) += n;

		if (offset >> private->chipshift)
			return 0;
	}

	/* We are now aligned, write as much as possible. */
	while (len >= map->bankwidth) {
		unsigned long datum;

		if (map->bankwidth == 1)
			datum = *(unsigned char *)buf;
		else if (map->bankwidth == 2)
			datum = *(unsigned short *)buf;
		else if (map->bankwidth == 4)
			datum = *(unsigned long *)buf;
		else
			return -EINVAL;

		ret = write_one_word(map, &private->chip, offset,
				datum);

		if (ret)
			return ret;

		offset += map->bankwidth;
		buf += map->bankwidth;
		(*retlen) += map->bankwidth;
		len -= map->bankwidth;

		if (offset >> private->chipshift)
			return 0;
	}

	if (len & (map->bankwidth - 1)) {
		int i = 0, n = 0;
		unsigned char tmp_buf[2];
		unsigned long datum;

		map_copy_from(map, tmp_buf,
				offset + private->chip.start,
				map->bankwidth);

		while (len--)
			tmp_buf[i++] = buf[n++];

		if (map->bankwidth == 2)
			datum = *(unsigned short *)tmp_buf;
		else if (map->bankwidth == 4)
			datum = *(unsigned long *)tmp_buf;
		else
			return -EINVAL;

		ret = write_one_word(map, &private->chip, offset,
				datum);

		if (ret)
			return ret;

		(*retlen) += n;
	}

	return 0;
}

static int erase_one_block(struct map_info *map, struct flchip *chip,
			   unsigned long addr, unsigned long size)
{
	unsigned long timeo = jiffies + HZ;
	DECLARE_WAITQUEUE(wait, current);

 retry:
	spin_lock_bh(chip->mutex);

	if (chip->state != FL_READY) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		add_wait_queue(&chip->wq, &wait);

		spin_unlock_bh(chip->mutex);

		schedule();
		remove_wait_queue(&chip->wq, &wait);

		if (signal_pending(current))
			return -EINTR;

		timeo = jiffies + HZ;

		goto retry;
	}

	chip->state = FL_ERASING;

	addr += chip->start;

	send_cmd(map, chip->start, CMD_SECTOR_ERASE_UNLOCK_DATA);
	send_cmd_to_addr(map, chip->start, CMD_SECTOR_ERASE_UNLOCK_DATA_2, addr);

	timeo = jiffies + (HZ * 20);

	spin_unlock_bh(chip->mutex);
	schedule_timeout(HZ);
	spin_lock_bh(chip->mutex);

	while (flash_is_busy(map, chip->start)) {
		if (chip->state != FL_ERASING) {
			/* Someone's suspended the erase. Sleep. */
			set_current_state(TASK_UNINTERRUPTIBLE);
			add_wait_queue(&chip->wq, &wait);

			spin_unlock_bh(chip->mutex);
			pr_info("erase suspended, sleeping\n");
			schedule();
			remove_wait_queue(&chip->wq, &wait);

			if (signal_pending(current))
				return -EINTR;

			timeo = jiffies + (HZ*2);
			spin_lock_bh(chip->mutex);
			continue;
		}

		/* OK Still waiting */
		if (time_after(jiffies, timeo)) {
			chip->state = FL_READY;
			spin_unlock_bh(chip->mutex);
			pr_warning("waiting for erase to complete timed out\n");
			return -EIO;
		}

		/* Latency issues. Drop the lock, wait a while, and retry. */
		spin_unlock_bh(chip->mutex);

		if (need_resched())
			schedule();
		else
			udelay(1);

		spin_lock_bh(chip->mutex);
	}

	{
		/* Verify every single word */
		int address;
		int error = 0;
		int verify;
		map_word test;

		for (address = addr; address < (addr + size); address += 2) {
			test = map_read(map, address);
			verify = test.x[0];
			if (verify != 0xFFFF) {
				error = 1;
				break;
			}
		}

		if (error) {
			chip->state = FL_READY;
			spin_unlock_bh(chip->mutex);
			pr_warning("verify error at 0x%x, size %ld\n",
				   address, size);
			return -EIO;
		}
	}

	chip->state = FL_READY;
	wake_up(&chip->wq);
	spin_unlock_bh(chip->mutex);

	return 0;
}

static int stm_flash_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct map_info *map = mtd->priv;
	struct stm_flash_private *private = map->fldrv_priv;
	unsigned long addr, len, shift;
	int chipnum;
	int ret = 0;
	int i, first;
	struct mtd_erase_region_info *regions = mtd->eraseregions;

	if (instr->addr > mtd->size)
		return -EINVAL;

	if ((instr->len + instr->addr) > mtd->size)
		return -EINVAL;

	/*
	 * Check that both start and end of the requested erase are aligned
	 * with the erasesize at the appropriate addresses.
	 */
	i = 0;

	/*
	 * Skip all erase regions which are ended before the start of the
	 * requested erase. Actually, to save on the calculations, we skip
	 * to the first erase region which starts after the start of the
	 * requested erase, and then go back one.
	 */
	while ((i < mtd->numeraseregions) &&
	       (instr->addr >= regions[i].offset))
		++i;
	--i;

	/*
	 * OK. Now i is pointing at the erase region in which this erase
	 * request starts. Check the start of the requested erase range
	 * is aligned with the erase size which is in effect here.
	 */
	if (instr->addr & (regions[i].erasesize - 1))
		return -EINVAL;

	/*
	 * Remember the erase region we start on.
	 */
	first = i;

	/*
	 * Next, theck that the end of the requested erase is aligned with
	 * the erase region at that address.
	 */
	while ((i < mtd->numeraseregions) &&
	       ((instr->addr + instr->len) >= regions[i].offset))
		++i;
	--i;

	if ((instr->addr + instr->len) & (regions[i].erasesize - 1))
		return -EINVAL;

	chipnum = instr->addr >> private->chipshift;
	addr = instr->addr - (chipnum << private->chipshift);
	len = instr->len;

	i = first;
	shift = (1 << private->chipshift) - 1;
	while (len) {
		ret = erase_one_block(map, &private->chip, addr,
				regions[i].erasesize);

		if (ret)
			return ret;

		addr += regions[i].erasesize;
		len -= regions[i].erasesize;

		if ((addr & shift) ==
		    ((regions[i].offset +
		      (regions[i].erasesize * regions[i].numblocks))
		     & shift))
			++i;

		if (addr & ~shift)
			break;
	}

	instr->state = MTD_ERASE_DONE;
	if (instr->callback)
		instr->callback(instr);

	return 0;
}

static void stm_flash_destroy(struct mtd_info *mtd)
{
	struct map_info *map = mtd->priv;
	struct stm_flash_private *private = map->fldrv_priv;
	kfree(private);
}

struct stm_flash_info {
	u8 mfr_id, dev_id;
	char *name;
	unsigned long size;
	int numeraseregions;
	struct mtd_erase_region_info regions[1];
};

const struct stm_flash_info stm_data[] = {
	{
		.mfr_id = 0x20,
		.dev_id = 0xe9,
		.name   = "STM PSD4256G6V",
		.size   = 0x100000,
		.numeraseregions = 1,
		.regions = {
			{
				.offset    = 0x000000,
				.erasesize = 0x10000,
				.numblocks = 16
			},
		},
	},
};

static int stm_probe_new_chip(struct mtd_info *mtd)
{
	const struct stm_flash_info *std;
	struct map_info *map = mtd->priv;
	int i;
	u8 mfr_id, dev_id;
	map_word mfr_word, dev_word;

	/* Enter autoselect mode */
	send_cmd(map, 0, CMD_RESET_DATA);
	send_cmd(map, 0, CMD_MANUFACTURER_UNLOCK_DATA);

	mfr_word = map_read(map, ADDR_MANUFACTURER);
	dev_word = map_read(map, ADDR_DEVICE_ID);
	mfr_id = mfr_word.x[0] & 0xFF;
	dev_id = dev_word.x[0] & 0xFF;

	/* Exit autoselect mode */
	send_cmd(map, 0, CMD_RESET_DATA);

	for (i = 0; i < ARRAY_SIZE(stm_data); ++i) {
		std = &stm_data[i];
		if (mfr_id == std->mfr_id && dev_id == std->dev_id)
			break;
	}
	if (i == ARRAY_SIZE(stm_data)) {
		pr_warning("unknown mfr/dev id: 0x%02x 0x%02x\n", mfr_id, dev_id);
		return 1;
	}

	mtd->size += std->size;
	mtd->numeraseregions += std->numeraseregions;

	mtd->eraseregions = kmalloc(sizeof(*mtd->eraseregions) *
				    mtd->numeraseregions, GFP_KERNEL);
	if (!mtd->eraseregions) {
		pr_warning("failed to allocate memory for MTD erase region\n");
		return 1;
	}

	for (i = 0; i < std->numeraseregions; ++i) {
		mtd->eraseregions[i].offset    = std->regions[i].offset;
		mtd->eraseregions[i].erasesize = std->regions[i].erasesize;
		mtd->eraseregions[i].numblocks = std->regions[i].numblocks;
		if (mtd->erasesize < mtd->eraseregions[i].erasesize)
			mtd->erasesize = mtd->eraseregions[i].erasesize;
	}

	pr_info("found %s device at %p in %d-bit bank\n",
	        std->name, map->virt, map->bankwidth * 8);

	return 0;
}

static struct mtd_info *stm_flash_probe(struct map_info *map);

static struct mtd_chip_driver stm_flash_chipdrv = {
	.probe   = stm_flash_probe,
	.destroy = stm_flash_destroy,
	.name    = "stm_flash",
	.module  = THIS_MODULE
};

static struct mtd_info *stm_flash_probe(struct map_info *map)
{
	struct mtd_info *mtd;
	struct flchip *chip;
	struct stm_flash_private *private;
	unsigned long size;

	mtd = kzalloc(sizeof(*mtd) + sizeof(*private), GFP_KERNEL);
	if (!mtd) {
		pr_warning("kmalloc failed for info structure\n");
		return NULL;
	}
	mtd->priv = map;
	map->fldrv_priv = private = (void *)&mtd[1];
	map->fldrv = &stm_flash_chipdrv;

	if (stm_probe_new_chip(mtd)) {
		kfree(mtd);
		return NULL;
	}

	chip = &private->chip;
	init_waitqueue_head(&chip->wq);
	spin_lock_init(&chip->_spinlock);
	chip->start = 0;
	chip->state = FL_READY;
	chip->mutex = &chip->_spinlock;
	for (size = mtd->size; size > 1; size >>= 1)
		++private->chipshift;

	mtd->type  = MTD_NORFLASH;
	mtd->flags = MTD_CAP_NORFLASH;
	mtd->name  = map->name;
	mtd->erase = stm_flash_erase;
	mtd->read  = stm_flash_read;
	mtd->write = stm_flash_write;
	mtd->sync  = stm_flash_sync;
	mtd->writesize = 1;

	return mtd;
}

static int __init stm_flash_init(void)
{
	register_mtd_chip_driver(&stm_flash_chipdrv);
	return 0;
}
module_init(stm_flash_init);

static void __exit stm_flash_exit(void)
{
	unregister_mtd_chip_driver(&stm_flash_chipdrv);
}
module_exit(stm_flash_exit);

MODULE_DESCRIPTION("MTD chip driver for ST PSD4256G flashes");
MODULE_LICENSE("GPL");
