/* linux/drivers/mtd/nand/bf54x_nand.c
 *
 * Copyright 2006-2007 Analog Devices Inc.
 *	http://blackfin.uclinux.org/
 *	Bryan Wu <bryan.wu@analog.com>
 *
 * Blackfin BF54x on-chip NAND flash controler driver
 *
 * Derived from drivers/mtd/nand/s3c2410.c
 * Copyright (c) 2007 Ben Dooks <ben@simtec.co.uk>
 *
 * Changelog:
 *	12-Jun-2007  Bryan Wu:  Initial version
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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/io.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>

#include <asm/blackfin.h>
#include <asm/dma.h>
#include <asm/mach/nand.h>

#ifdef CONFIG_MTD_NAND_BF54X_HWECC
int hardware_ecc = 1;
#else
int hardware_ecc = 0;
#endif

/*----------------------------------------------------------------------------
 * Data structures for bf54x nand flash controller driver
 */

/* 256 page date hardware ecc generate 8 bytes ecc code */
static struct nand_ecclayout bf54x_oob_256_8bytes = {
	.eccbytes = 4,
	.eccpos = {0, 1, 2, 3},
	.oobfree = {
		{.offset = 4,
		.length = 4}
	},
};

/* 512 page date hardware ecc generate 16 bytes ecc code */
static struct nand_ecclayout bf54x_oob_512_16bytes = {
	.eccbytes = 8,
	.eccpos = {0, 1, 2, 3, 8, 9, 10, 11},
	.oobfree = {
		{.offset = 4,
		.length = 4},
		{.offset = 12,
		.length = 4}
	},
};

/* bf54x nand info */
struct bf54x_nand_info {
	/* mtd info */
	struct nand_hw_control		controller;
	struct mtd_info			mtd;
	struct nand_chip		chip;

	/* platform info */
	struct bf54x_nand_platform	*platform;

	/* device info */
	struct device			*device;

	/* DMA stuff */
	struct completion		dma_completion;
};

/*----------------------------------------------------------------------------
 * Conversion functions
 */
static struct bf54x_nand_info *mtd_to_nand_info(struct mtd_info *mtd)
{
	return container_of(mtd, struct bf54x_nand_info, mtd);
}

static struct bf54x_nand_info *to_nand_info(struct platform_device *pdev)
{
	return platform_get_drvdata(pdev);
}

static struct bf54x_nand_platform *to_nand_plat(struct platform_device *pdev)
{
	return pdev->dev.platform_data;
}


/*----------------------------------------------------------------------------
 * struct nand_chip interface function pointers
 */

/*
 * bf54x_nand_hwcontrol
 *
 * Issue command and address cycles to the chip
 */
static void bf54x_nand_hwcontrol(struct mtd_info *mtd, int cmd,
				   unsigned int ctrl)
{
	if (cmd == NAND_CMD_NONE)
		return;

	while (bfin_read_NFC_STAT() & WB_FULL)
		continue;

	if (ctrl & NAND_CLE)
		bfin_write_NFC_CMD(cmd);
	else
		bfin_write_NFC_ADDR(cmd);
	SSYNC();
}

/*
 * bf54x_nand_devready()
 *
 * returns 0 if the nand is busy, 1 if it is ready
 */
static int bf54x_nand_devready(struct mtd_info *mtd)
{
	SSYNC();
	return (bfin_read_NFC_STAT() & NFC_STAT_NBUSY);
}

/*----------------------------------------------------------------------------
 * ECC functions
 *
 * These allow the bf54x to use the controller's ECC
 * generator block to ECC the data as it passes through
 */
static inline int count_bits(uint32_t byte)
{
	int res = 0;

	for (; byte; byte >>= 1)
		res += byte & 0x01;
	return res;
}

/*
 * ECC error correction function
 */
static int bf54x_nand_correct_data_256(struct mtd_info *mtd, u_char *dat,
					u_char *read_ecc, u_char *calc_ecc)
{
	struct bf54x_nand_info *info = mtd_to_nand_info(mtd);
	unsigned int syndrome[5];
	unsigned int full[2];
	unsigned short *stored = (unsigned short *) read_ecc;
	unsigned short *calced = (unsigned short *) calc_ecc;
	int i;

	dev_dbg(info->device, "(%p,%p,%p,%p)\n", mtd, dat, read_ecc, calc_ecc);

	full[0] = (calced[0] & 0x3FF) | ((calced[1] & 0x3FF) << 10);
	full[1] = (stored[0] & 0x3FF) | ((stored[1] & 0x3FF) << 10);

	dev_dbg(info->device, "full[0] 0x%08x, full[1] 0x%08x\n",
		full[0], full[1]);

	syndrome[0] = (full[0] ^ full[1]) & 0x3FFFFF;
	syndrome[1] = (stored[0] ^ calced[0]) & 0x3FF;
	syndrome[2] = (calced[0] ^ calced[1]) & 0x3FF;
	syndrome[3] = (stored[0] ^ stored[1]) & 0x3FF;
	syndrome[4] = (syndrome[2] ^ syndrome[3]) & 0x3FF;

	for (i = 0; i < 5; i++)
		dev_dbg(info->device, "syndrome[%d] 0x%08x\n", i, syndrome[i]);

	/*
	 * syndrome 0: all zero
	 * No error in data
	 * No action
	 */
	if (syndrome[0] == 0)
		return 0;

	/*
	 * sysdrome 0: exactly 11 bits are one, each parity
	 * and parity' pair is 1 & 0 or 0 & 1.
	 * 1-bit correctable error
	 * Correct the error
	 */
	if (count_bits(syndrome[0]) == 11 && syndrome[4] == 0x7FF) {
		dev_dbg(info->device, "1-bit correctable error, correct it.\n");
		dev_dbg(info->device, "syndrome[1] 0x%08x\n", syndrome[1]);
		/* TODO */

		return 0;
	}

	/*
	 * sysdrome 0: only one bit is one
	 * ECC data was incorrect
	 * No action
	 */
	if (count_bits(syndrome[0]) == 1) {
		dev_err(info->device, "ECC data was incorrect!\n");
		return 1;
	}

	/*
	 * sysdrome 0: random data
	 * More than 1-bit error, non-correctable error
	 * Discard data, mark bad block
	 */
	dev_err(info->device,
		"More than 1-bit error, non-correctable error.\n");
	dev_err(info->device,
		"Please discard data, mark bad block\n");

	return 1;
}

static int bf54x_nand_correct_data(struct mtd_info *mtd, u_char *dat,
					u_char *read_ecc, u_char *calc_ecc)
{
	int ret;

	ret = bf54x_nand_correct_data_256(mtd, dat, read_ecc, calc_ecc);

	/* If page size is 512, correct second 256 bytes */
	if (mtd->writesize == 512) {
		dat += 256;
		read_ecc += 8;
		calc_ecc += 8;
		ret = bf54x_nand_correct_data_256(mtd, dat, read_ecc, calc_ecc);
	}

	return ret;
}

static void bf54x_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	unsigned short rst;

	/*
	 * This register must be written before each page is
	 * transferred to generate the correct ECC register
	 * values.
	 */
	SSYNC();

	rst = bfin_read_NFC_RST();
	rst |= 0x1;
	bfin_write_NFC_RST(rst);

	SSYNC();
}

static int bf54x_nand_calculate_ecc(struct mtd_info *mtd,
		const u_char *dat, u_char *ecc_code)
{
	struct bf54x_nand_info *info = mtd_to_nand_info(mtd);
	unsigned short ecc;
	unsigned short *code = (unsigned short *)ecc_code;

	SSYNC();

	/* first 4 bytes ECC code for 256 page size */
	ecc = bfin_read_NFC_ECC0();
	code[0] = ecc && 0x3FF;

	ecc = bfin_read_NFC_ECC1();
	code[1] = ecc && 0x3FF;

	dev_dbg(info->device, "returning ecc %04x%04x\n", code[0], code[1]);

	/* second 4 bytes ECC code for 512 page size */
	if (mtd->writesize == 512) {
		ecc = bfin_read_NFC_ECC2();
		code[2] = ecc && 0x3FF;

		ecc = bfin_read_NFC_ECC3();
		code[3] = ecc && 0x3FF;

		dev_dbg(info->device, "returning ecc %04x%04x\n",
			code[0], code[1]);
	}

	return 0;
}

/*----------------------------------------------------------------------------
 * PIO mode for buffer writing and reading
 */
static void bf54x_nand_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	int i;
	unsigned short val;

	/*
	 * Data reads are requested by first writing to NFC_DATA_RD
	 * and then reading back from NFC_READ.
	 */
	for (i = 0; i < len; i++) {
		while (bfin_read_NFC_STAT() & WB_FULL)
			continue;

		/* Contents do not matter */
		bfin_write_NFC_DATA_RD(0x5555);
		SSYNC();

		while ((bfin_read_NFC_IRQSTAT() & RD_RDY) != RD_RDY)
			continue;

		buf[i] = bfin_read_NFC_READ();

		val = bfin_read_NFC_IRQSTAT();
		val |= RD_RDY;
		bfin_write_NFC_IRQSTAT(val);
		SSYNC();
	}
}

static uint8_t bf54x_nand_read_byte(struct mtd_info *mtd)
{
	uint8_t val;

	bf54x_nand_read_buf(mtd, &val, 1);

	return val;
}

static void bf54x_nand_write_buf(struct mtd_info *mtd,
				const uint8_t *buf, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		while (bfin_read_NFC_STAT() & WB_FULL)
			continue;

		bfin_write_NFC_DATA_WR(buf[i]);
		SSYNC();
	}
}

static void bf54x_nand_read_buf16(struct mtd_info *mtd, uint8_t *buf, int len)
{
	int i;
	u16 *p = (u16 *) buf;
	len >>= 1;

	/*
	 * Data reads are requested by first writing to NFC_DATA_RD
	 * and then reading back from NFC_READ.
	 */
	bfin_write_NFC_DATA_RD(0x5555);

	SSYNC();

	for (i = 0; i < len; i++)
		p[i] = bfin_read_NFC_READ();
}

static void bf54x_nand_write_buf16(struct mtd_info *mtd,
				const uint8_t *buf, int len)
{
	int i;
	u16 *p = (u16 *) buf;
	len >>= 1;

	for (i = 0; i < len; i++)
		bfin_write_NFC_DATA_WR(p[i]);

	SSYNC();
}

/*----------------------------------------------------------------------------
 * DMA functions for buffer writing and reading
 */
static irqreturn_t bf54x_nand_dma_irq (int irq, void *dev_id)
{
	struct bf54x_nand_info *info = (struct bf54x_nand_info *) dev_id;

	clear_dma_irqstat(CH_NFC);
	disable_dma(CH_NFC);
	complete(&info->dma_completion);

	return IRQ_HANDLED;
}

static int bf54x_nand_dma_rw_buf(struct mtd_info *mtd, uint8_t *buf,
				int len, int is_read)
{
	struct bf54x_nand_info *info = mtd_to_nand_info(mtd);
	struct bf54x_nand_platform *plat = info->platform;
	unsigned short page_size = (plat->page_size ? 512 : 256);
	int steps, i;
	uint8_t *p = buf;
	unsigned short val;

	dev_dbg(info->device, " mtd->%p, buf->%p, len %d, is_read %d\n",
			mtd, buf, len, is_read);

	bfin_write_NFC_RST(0x1);
	SSYNC();

	steps = len / page_size;
	for (i = 0; i < steps; i++) {
		/* setup DMA register with Blackfin DMA API */
		set_dma_config(CH_NFC, 0x0);
		set_dma_start_addr(CH_NFC, (unsigned long) p);
		set_dma_x_count(CH_NFC, (page_size >> 2));
		set_dma_x_modify(CH_NFC, 4);

		/* setup write or read operation */
		val = DI_EN | WDSIZE_32;
		if (is_read)
			val |= WNR;
		set_dma_config(CH_NFC, val);
		enable_dma(CH_NFC);

		/* Start PAGE read/write operation */
		if (is_read)
			bfin_write_NFC_PGCTL(0x1);
		else
			bfin_write_NFC_PGCTL(0x2);
		SSYNC();
		wait_for_completion(&info->dma_completion);
		p += page_size;
	}

	return 0;
}

static int bf54x_nand_dma_read_page(struct mtd_info *mtd,
					struct nand_chip *chip, uint8_t *buf)
{
	return bf54x_nand_dma_rw_buf(mtd, buf, mtd->writesize, 1);
}

static void bf54x_nand_dma_write_page(struct mtd_info *mtd,
				struct nand_chip *chip, const uint8_t *buf)
{
	bf54x_nand_dma_rw_buf(mtd, (uint8_t *)buf, mtd->writesize, 0);
}

static void bf54x_nand_dma_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct bf54x_nand_info *info = mtd_to_nand_info(mtd);
	struct bf54x_nand_platform *plat = info->platform;
	unsigned short page_size = (plat->page_size ? 512 : 256);

	if (len < page_size) {
		bf54x_nand_read_buf(mtd, buf, len);
		return;
	}

	bf54x_nand_dma_rw_buf(mtd, buf, len, 1);
}

static void bf54x_nand_dma_write_buf(struct mtd_info *mtd,
				const uint8_t *buf, int len)
{
	struct bf54x_nand_info *info = mtd_to_nand_info(mtd);
	struct bf54x_nand_platform *plat = info->platform;
	unsigned short page_size = (plat->page_size ? 512 : 256);

	if (len < page_size) {
		bf54x_nand_write_buf(mtd, buf, len);
		return;
	}

	bf54x_nand_dma_rw_buf(mtd, (uint8_t *)buf, len, 0);
}

/*----------------------------------------------------------------------------
 * System initialization functions
 */

static int bf54x_nand_dma_init(struct bf54x_nand_info *info)
{
	struct bf54x_nand_platform *plat = info->platform;
	int ret;
	unsigned short val;

	/* Do not use dma */
	if (!plat->enable_dma)
		return 0;

	init_completion(&info->dma_completion);

	/* Setup DMAC1 channel mux for NFC which shared with SDH */
	val = bfin_read_DMAC1_PERIMUX();
	val &= 0xFFFE;
	bfin_write_DMAC1_PERIMUX(val);
	SSYNC();

	/* Request NFC DMA channel */
	ret = request_dma(CH_NFC, "BF54X NFC driver");
	if (ret < 0) {
		dev_err(info->device, " unable to get DMA channel\n");
		return 1;
	}

	set_dma_callback(CH_NFC, (void *) bf54x_nand_dma_irq, (void *) info);

	/* Turn off the DMA channel first */
	disable_dma(CH_NFC);

	return 0;
}

/*
 * BF54X NFC hardware initialization
 *  - pin mux setup
 *  - clear interrupt status
 */
static int bf54x_nand_hw_init(struct bf54x_nand_info *info)
{
	int err = 0;
	unsigned short val;
	struct bf54x_nand_platform *plat = info->platform;

	if (!info)
		return -EINVAL;

	/* setup NFC_CTL register */
	dev_info(info->device,
		"page_size=%d, data_width=%d, wr_dly=%d, rd_dly=%d\n",
		(plat->page_size ? 512 : 256),
		(plat->data_width ? 16 : 8),
		plat->wr_dly, plat->rd_dly);

	val = (plat->page_size << NFC_PG_SIZE_OFFSET) |
		(plat->data_width << NFC_NWIDTH_OFFSET) |
		(plat->rd_dly << NFC_RDDLY_OFFSET) |
		(plat->rd_dly << NFC_WRDLY_OFFSET);
	dev_dbg(info->device, "NFC_CTL is 0x%04x\n", val);

	bfin_write_NFC_CTL(val);
	SSYNC();

	/* clear interrupt status */
	bfin_write_NFC_IRQMASK(0x0);
	SSYNC();
	val = bfin_read_NFC_IRQSTAT();
	bfin_write_NFC_IRQSTAT(val);
	SSYNC();

	/* enable GPIO function enable register */
	val = bfin_read_PORTJ_FER();
	val |= 6;
	bfin_write_PORTJ_FER(val);
	SSYNC();

	/* DMA initialization  */
	if (bf54x_nand_dma_init(info)) {
		err = -ENXIO;
	}

	return err;
}

/*----------------------------------------------------------------------------
 * Device management interface
 */
static int bf54x_nand_add_partition(struct bf54x_nand_info *info)
{
	struct mtd_info *mtd = &info->mtd;

#ifdef CONFIG_MTD_PARTITIONS
	struct mtd_partition *parts = info->platform->partitions;
	int nr = info->platform->nr_partitions;

	return add_mtd_partitions(mtd, parts, nr);
#else
	return add_mtd_device(mtd);
#endif
}

static int bf54x_nand_remove(struct platform_device *pdev)
{
	struct bf54x_nand_info *info = to_nand_info(pdev);
	struct mtd_info *mtd = NULL;

	platform_set_drvdata(pdev, NULL);

	if (!info)
		return 0;

	/* first thing we need to do is release all our mtds
	 * and their partitions, then go through freeing the
	 * resources used
	 */
	mtd = &info->mtd;
	if (mtd) {
		nand_release(mtd);
		kfree(mtd);
	}

	/* free the common resources */
	kfree(info);

	return 0;
}

/*
 * bf54x_nand_probe
 *
 * called by device layer when it finds a device matching
 * one our driver can handled. This code checks to see if
 * it can allocate all necessary resources then calls the
 * nand layer to look for devices
 */
static int bf54x_nand_probe(struct platform_device *pdev)
{
	struct bf54x_nand_platform *plat = to_nand_plat(pdev);
	struct bf54x_nand_info *info = NULL;
	struct nand_chip *chip = NULL;
	struct mtd_info *mtd = NULL;
	int err = 0;

	dev_dbg(&pdev->dev, "(%p)\n", pdev);

	if (!plat) {
		dev_err(&pdev->dev, "no platform specific information\n");
		goto exit_error;
	}

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (info == NULL) {
		dev_err(&pdev->dev, "no memory for flash info\n");
		err = -ENOMEM;
		goto exit_error;
	}

	platform_set_drvdata(pdev, info);

	spin_lock_init(&info->controller.lock);
	init_waitqueue_head(&info->controller.wq);

	info->device     = &pdev->dev;
	info->platform   = plat;

	/* initialise chip data struct */
	chip = &info->chip;

	if (plat->data_width)
		chip->options |= NAND_BUSWIDTH_16;

	chip->options |= NAND_SKIP_BBTSCAN;

	if (plat->enable_dma) {
		chip->read_buf = bf54x_nand_dma_read_buf;
		chip->write_buf = bf54x_nand_dma_write_buf;
	} else {
		chip->read_buf = (plat->data_width) ?
			bf54x_nand_read_buf16 : bf54x_nand_read_buf;
		chip->write_buf = (plat->data_width) ?
			bf54x_nand_write_buf16 : bf54x_nand_write_buf;
	}

	chip->read_byte    = bf54x_nand_read_byte;

	chip->cmd_ctrl     = bf54x_nand_hwcontrol;
	chip->dev_ready    = bf54x_nand_devready;

	chip->priv	   = &info->mtd;
	chip->controller   = &info->controller;

	chip->IO_ADDR_R    = (void __iomem *) NFC_READ;
	chip->IO_ADDR_W    = (void __iomem *) NFC_DATA_WR;

	chip->chip_delay   = 0;

	/* initialise mtd info data struct */
	mtd 		= &info->mtd;
	mtd->priv	= chip;
	mtd->owner	= THIS_MODULE;

	/* initialise the hardware */
	err = bf54x_nand_hw_init(info);
	if (err != 0)
		goto exit_error;

	/* setup hardware ECC data struct */
	if (hardware_ecc) {
		if (plat->page_size == NFC_PG_SIZE_256) {
			chip->ecc.bytes = 8;
			chip->ecc.layout = &bf54x_oob_256_8bytes;
			chip->ecc.size = 256;
		} else if (mtd->writesize == NFC_PG_SIZE_512) {
			chip->ecc.bytes = 16;
			chip->ecc.layout = &bf54x_oob_512_16bytes;
			chip->ecc.size = 512;
		}

		chip->ecc.calculate = bf54x_nand_calculate_ecc;
		chip->ecc.correct   = bf54x_nand_correct_data;
		chip->ecc.mode	    = NAND_ECC_HW;
		chip->ecc.hwctl	    = bf54x_nand_enable_hwecc;
		chip->ecc.read_page = bf54x_nand_dma_read_page;
		chip->ecc.write_page = bf54x_nand_dma_write_page;
	} else {
		chip->ecc.mode	    = NAND_ECC_SOFT;
	}

	/* scan hardware nand chip and setup mtd info data struct */
	if (nand_scan(mtd, 1)) {
		err = -ENXIO;
		goto exit_error;
	}

	/* add NAND partition */
	bf54x_nand_add_partition(info);

	dev_dbg(&pdev->dev, "initialised ok\n");
	return 0;

exit_error:
	bf54x_nand_remove(pdev);

	if (err == 0)
		err = -EINVAL;
	return err;
}

/* PM Support */
#ifdef CONFIG_PM

static int bf54x_nand_suspend(struct platform_device *dev, pm_message_t pm)
{
	struct bf54x_nand_info *info = platform_get_drvdata(dev);

	return 0;
}

static int bf54x_nand_resume(struct platform_device *dev)
{
	struct bf54x_nand_info *info = platform_get_drvdata(dev);

	if (info)
		bf54x_nand_hw_init(info);

	return 0;
}

#else
#define bf54x_nand_suspend NULL
#define bf54x_nand_resume NULL
#endif

/* driver device registration */
static struct platform_driver bf54x_nand_driver = {
	.probe		= bf54x_nand_probe,
	.remove		= bf54x_nand_remove,
	.suspend	= bf54x_nand_suspend,
	.resume		= bf54x_nand_resume,
	.driver		= {
		.name	= "bf54x-nand",
		.owner	= THIS_MODULE,
	},
};

static int __init bf54x_nand_init(void)
{
	printk(KERN_INFO "BF54X NAND Driver, (c) 2007 Analog Devices, Inc.\n");

	return platform_driver_register(&bf54x_nand_driver);
}

static void __exit bf54x_nand_exit(void)
{
	platform_driver_unregister(&bf54x_nand_driver);
}

module_init(bf54x_nand_init);
module_exit(bf54x_nand_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Bryan Wu <bryan.wu@analog.com");
MODULE_DESCRIPTION("BF54X MTD NAND driver");
