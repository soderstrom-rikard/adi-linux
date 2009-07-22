/*
 * File:         drivers/input/captouch/ad7147.c
 * Based on:
 * Author:       Barry Song
 *
 * Created:
 * Description:  AD7147 CapTouch Programmable Controller driver
 *
 * Modified:
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see the file COPYING, or write
 * to the Free Software Foundation, Inc.,
 * 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/device.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>

#define AD7147_SPI_ADDR        0x1C
#define AD7147_SPI_ADDR_SHFT   11
#define AD7147_SPI_READ        1
#define AD7147_SPI_READ_SHFT   10

#define AD7147_STG_CAL_EN_REG  0x1
#define AD7147_PARTID_REG      0x17
#define AD7147_PARTID          0x1470
#define AD7147_STAGECFG_REG    0x80
#define AD7147_SYSCFG_REG      0x0

#define STG_LOW_INT_EN_REG     0x5
#define STG_HIGH_INT_EN_REG    0x6
#define STG_COM_INT_EN_REG     0x7
#define STG_LOW_INT_STA_REG    0x8
#define STG_HIGH_INT_STA_REG   0x9
#define STG_COM_INT_STA_REG    0xA

#define CDC_RESULT_S0          0xB
#define CDC_RESULT_S1          0xC
#define CDC_RESULT_S2          0xD
#define CDC_RESULT_S3          0xE
#define CDC_RESULT_S4          0xF
#define CDC_RESULT_S5          0x10
#define CDC_RESULT_S6          0x11
#define CDC_RESULT_S7          0x12
#define CDC_RESULT_S8          0x13
#define CDC_RESULT_S9          0x14
#define CDC_RESULT_S10         0x15
#define CDC_RESULT_S11         0x16

#define STAGE_NUM              12
#define STAGE_CFGREG_NUM       8
#define SYS_CFGREG_NUM         8
#define BTN_NUM                5
#define SLD_NUM                8

struct ad7147_button_info {
	int old_status[BTN_NUM];
	int event[BTN_NUM];
	unsigned short l_mask[BTN_NUM];
	unsigned short h_mask[BTN_NUM];
};

struct ad7147_chip {
	struct spi_device *spi;
	struct input_dev *input;
	struct work_struct work;
} *chip;

static unsigned short stage_cfg_reg[STAGE_NUM][STAGE_CFGREG_NUM] = {
	/* stage 0~7 for slider */
	{0xFBFF, 0x1FFF, 0, 0x2626, 1600, 1600, 1600, 1600},
	{0xEFFF, 0x1FFF, 0, 0x2626, 1650, 1650, 1650, 1650},
	{0xFFFF, 0x1FFE, 0, 0x2626, 1650, 1650, 1650, 1650},
	{0xFFFF, 0x1FFB, 0, 0x2626, 1650, 1650, 1650, 1650},
	{0xFFFF, 0x1FEF, 0, 0x2626, 1650, 1650, 1650, 1650},
	{0xFFFF, 0x1FBF, 0, 0x2626, 1650, 1650, 1650, 1650},
	{0xFFFF, 0x1EFF, 0, 0x2626, 1650, 1650, 1650, 1650},
	{0xFFFF, 0x1BFF, 0, 0x2626, 1600, 1600, 1600, 1600},
	/* stage 8~11 for buttons */
	{0xFF7B, 0x3FFF, 0x506,  0x2626, 1100, 1100, 1150, 1150},
	{0xFDFE, 0x3FFF, 0x606,  0x2626, 1100, 1100, 1150, 1150},
	{0xFEBA, 0x1FFF, 0x1400, 0x2626, 1200, 1200, 1300, 1300},
	{0xFFEF, 0x1FFF, 0x0,    0x2626, 1100, 1100, 1150, 1150},
};

static unsigned short sys_cfg_reg[SYS_CFGREG_NUM] = {
	0x2B2, 0x0, 0x3233, 0x819, 0x832, 0xCFF, 0xCFF, 0x0
};

static struct ad7147_button_info ad7147_btn = {
	{0, 0, 0, 0, 0},
	{BTN_FORWARD, BTN_LEFT, BTN_MIDDLE, BTN_RIGHT, BTN_BACK},
	{0, 0, 0, 0x100, 0x200},
	{0x600, 0x500, 0x800, 0x400, 0x400},
};

static int ad7147_sld_older_status;

int ad7147_spi_read(unsigned short reg, unsigned short *data)
{
	int ret;
	unsigned short tx[2] = { 0xFFFF, 0xFFFF };
	unsigned short rx[2] = { 0xFFFF, 0xFFFF };
	struct spi_transfer t = {
		.tx_buf = tx,
		.rx_buf = rx,
		.len = 4,
	};
	struct spi_message m;

	tx[0] = (AD7147_SPI_ADDR << AD7147_SPI_ADDR_SHFT) |
		(AD7147_SPI_READ << AD7147_SPI_READ_SHFT) | reg;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	ret = spi_sync(chip->spi, &m);

	if (ret == 0)
		*data = rx[1];

	return ret;
}

int ad7147_spi_write(unsigned short reg, unsigned short data)
{
	unsigned short tx[2];
	struct spi_transfer t = {
		.tx_buf = tx,
		.len = 4,
	};
	struct spi_message m;

	tx[0] = (AD7147_SPI_ADDR << AD7147_SPI_ADDR_SHFT) | reg;
	tx[1] = data;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return spi_sync(chip->spi, &m);
}

static int ad7147_hw_detect(void)
{
	unsigned short data;

	ad7147_spi_read(AD7147_PARTID_REG, &data);
	if ((data & 0xFFF0) == AD7147_PARTID) {
		dev_info(&chip->spi->dev, "Found AD7147 captouch, rev:%d\n",
				data & 0xF);
		return 0;
	} else {
		dev_err(&chip->spi->dev, "Fail to detect AD7147 captouch,\
				read ID is %04x\n", data);
		return -ENODEV;
	}
}

static int ad7147_hw_init(void)
{
	int i, j;
	unsigned short reg_base;
	unsigned short data;

	/* configuration CDC and interrupts*/

	for (i = 0; i < STAGE_NUM; i++) {
		reg_base = AD7147_STAGECFG_REG + i * STAGE_CFGREG_NUM;
		for (j = 0; j < STAGE_CFGREG_NUM; j++) {
			ad7147_spi_write(reg_base + j, stage_cfg_reg[i][j]);
			ad7147_spi_read(AD7147_PARTID_REG, &data);
		}
	}

	for (i = 0; i < SYS_CFGREG_NUM; i++)
		ad7147_spi_write(AD7147_SYSCFG_REG + i, sys_cfg_reg[i]);

	ad7147_spi_write(AD7147_STG_CAL_EN_REG, 0xFFF);

	/* clear all interrupts */
	ad7147_spi_read(STG_LOW_INT_STA_REG, &data);
	ad7147_spi_read(STG_HIGH_INT_STA_REG, &data);
	ad7147_spi_read(STG_COM_INT_STA_REG, &data);

	return 0;
}

static void ad7147_work(struct work_struct *work)
{
	unsigned short l_data, h_data, stg_data;
	int i;
	struct input_dev *input = chip->input;

	ad7147_spi_read(STG_LOW_INT_STA_REG, &l_data);
	ad7147_spi_read(STG_HIGH_INT_STA_REG, &h_data);

	/* button event report */
	for (i = 0; i < BTN_NUM; i++) {
		if (((l_data & ad7147_btn.l_mask[i]) == ad7147_btn.l_mask[i]) &&
		((h_data & ad7147_btn.h_mask[i]) == ad7147_btn.h_mask[i])) {
			if (ad7147_btn.old_status[i] == 0) {
				input_report_key(input, ad7147_btn.event[i], 1);
				ad7147_btn.old_status[i] = 1;

				pr_debug("report BTN %x touched event\n",
					ad7147_btn.event[i]);
			}
		} else if (ad7147_btn.old_status[i] == 1) {
			input_report_key(input, ad7147_btn.event[i], 0);
			ad7147_btn.old_status[i] = 0;

			pr_debug("report BTN %x untouched event\n",
					ad7147_btn.event[i]);
		}
	}

	/* slider event, we need user space utility to help analysis the
	 * coordinate. In kernel, we only can report the CDC result to
	 * userspace.
	 *
	 * The software algorithm used to achieve the high resolution output,
	 * provided as C code, is available from Analog Devices by signing a
	 * software license agreement. This code requires approximately 11kB
	 * of program code and 700 bytes of RAM in the host processor to run
	 * successfully. Total code size depends on the functionality required.
	 */
	if ((l_data || h_data) & 0xff) {
		for (i = 0; i < SLD_NUM; i++) {
			if ((l_data & (1 << i)) || (h_data & (1 << i))) {
				ad7147_spi_read(CDC_RESULT_S0+i, &stg_data);

				/* Is EV_MSC suitable to report a meanless
				 * integer to userspace?
				 */
				input_event(input, EV_MSC, MSC_RAW,
					stg_data | ((int)i << 16));
				pr_debug("STAGE%d touched, CDC result:%d\n",
					i, stg_data);
			}
		}
		ad7147_sld_older_status = 1;
	} else if (ad7147_sld_older_status == 1) {
		input_event(input, EV_MSC, MSC_RAW, 0);
		ad7147_sld_older_status = 0;
	}

	input_sync(input);
}

static irqreturn_t ad7147_interrupt(int irq, void *_data)
{
	schedule_work(&chip->work);

	return IRQ_HANDLED;
}

static int __devinit ad7147_spi_probe(struct spi_device *spi)
{
	int ret = 0;

	chip = kmalloc(sizeof(struct ad7147_chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->spi = spi;

	ret = ad7147_hw_detect();
	if (ret)
		goto det_err;

	ad7147_hw_init();

	/* Start workqueue for defer message transfer */
	INIT_WORK(&chip->work, ad7147_work);

	if (spi->irq > 0) {
		ret = request_irq(spi->irq, ad7147_interrupt,
				IRQF_TRIGGER_FALLING, "ad7147_captouch", chip);
		if (ret) {
			dev_err(&spi->dev, "Can't allocate irq %d\n",
					spi->irq);
			goto fail_irq;
		}
	} else
		dev_warn(&spi->dev, "IRQ not configured!\n");

	/* Allocate and register AD7147 input device */
	chip->input = input_allocate_device();
	if (!chip->input) {
		dev_err(&spi->dev, "Can't allocate input device\n");
		ret = -ENOMEM;
		goto fail_allocate;
	}

	chip->input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_MSC);
	chip->input->keybit[BIT_WORD(BTN_LEFT)] = BIT_MASK(BTN_LEFT) |
		BIT_MASK(BTN_RIGHT) |
		BIT_MASK(BTN_MIDDLE) |
		BIT_MASK(BTN_FORWARD) |
		BIT_MASK(BTN_BACK);
	chip->input->mscbit[BIT_WORD(MSC_RAW)] = BIT_MASK(MSC_RAW);
	chip->input->name = "ad7147 captouch";
	chip->input->phys = "ad7147/input";
	chip->input->id.bustype = BUS_I2C;
	chip->input->id.vendor = 0x0001;
	chip->input->id.product = 0x0001;
	chip->input->id.version = 0x0100;
	ret = input_register_device(chip->input);
	if (ret) {
		dev_err(&spi->dev, "Failed to register AD7147 input device!\n");
		goto fail_register;
	}

	return 0;
fail_register:
	input_free_device(chip->input);
fail_allocate:
	free_irq(spi->irq, chip);
fail_irq:
det_err:
	kfree(chip);
	return ret;
}

static int __devexit ad7147_spi_remove(struct spi_device *spi)
{
	kfree(chip);
	return 0;
}

static struct spi_driver ad7147_spi_driver = {
	.driver = {
		.name	= "ad7147-spi",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= ad7147_spi_probe,
	.remove		= __devexit_p(ad7147_spi_remove),
};

static int __init ad7147_init(void)
{
	int ret;

	ret = spi_register_driver(&ad7147_spi_driver);
	if (ret != 0) {
		printk(KERN_ERR "Failed to register ad7147 SPI driver: %d\n",
				ret);
	}

	return ret;
}
module_init(ad7147_init);

static void __exit ad7147_exit(void)
{
	spi_unregister_driver(&ad7147_spi_driver);
}
module_exit(ad7147_exit);

MODULE_DESCRIPTION("ad7147 captouch driver");
MODULE_AUTHOR("Barry Song ");
MODULE_LICENSE("GPL");
