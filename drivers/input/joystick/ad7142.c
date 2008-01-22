/*
 * File:         drivers/input/joystick/ad7142.c
 * Original Author: Aubrey Li
 * Maintained by: Bryan Wu <bryan.wu@analog.com>
 *
 * Created:      Apr 7th, 2006
 * Description:
 *
 * Modified:
 *               Copyright 2005-2007 Analog Devices Inc.
 *
 * Bugs:         Enter bugs at http://blackfin.uclinux.org/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.
 * If not, write to the Free Software Foundation,
 * 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/irq.h>
#include <linux/workqueue.h>

#include <asm/blackfin.h>

MODULE_AUTHOR("Aubrey Li, Bryan Wu <bryan.wu@analog.com>");
MODULE_DESCRIPTION("Driver for AD7142 Joysticks");
MODULE_LICENSE("GPL");

#define AD7142_DRV_NAME		"ad7142_js"
#define AD7142_I2C_ID		0xE622
#define AD7142_I2C_ADDR		0x2C
/*
 * Ram map - these registers are defined as we go along
 */
/* RW   Power & conversion control */
#define PWRCONVCTL		0x00

/* RW   Ambient compensation control register 0 - 3 */
#define AMBCOMPCTL_REG0		0x01
#define AMBCOMPCTL_REG1		0x02
#define AMBCOMPCTL_REG2		0x03
#define AMBCOMPCTL_REG3		0x04

/* RW   Interrupt enable register 0 - 2 */
#define INTEN_REG0		0x05
#define INTEN_REG1		0x06
#define INTEN_REG2		0x07

/* R    Low limit interrupt status register 0 */
#define INTSTAT_REG0		0x08
/* R    High limit interrupt status register 1 */
#define INTSTAT_REG1		0x09
/* R    Interrupt status register 2 */
#define INTSTAT_REG2		0x0A

/* R    ADC stage 0 - 11 result (uncompensated) actually located in SRAM */
#define ADCRESULT_S0		0x0B
#define ADCRESULT_S1		0x0C
#define ADCRESULT_S2		0x0D
#define ADCRESULT_S3		0x0E
#define ADCRESULT_S4		0x0F
#define ADCRESULT_S5		0x10
#define ADCRESULT_S6		0x11
#define ADCRESULT_S7		0x12
#define ADCRESULT_S8		0x13
#define ADCRESULT_S9		0x14
#define ADCRESULT_S10		0x15
#define ADCRESULT_S11		0x16

/* R    I.D. Register */
#define DEVID			0x17

/* R    Current threshold status register 0, 1 */
#define THRES_STAT_REG0		0x40
#define THRES_STAT_REG1		0x41
/* R    Current proximity status register 2 */
#define PROX_STAT_REG		0x42

#define STAGE0_CONNECTION	0x80
#define STAGE1_CONNECTION	0x88
#define STAGE2_CONNECTION	0x90
#define STAGE3_CONNECTION	0x98
#define STAGE4_CONNECTION	0xA0
#define STAGE5_CONNECTION	0xA8
#define STAGE6_CONNECTION	0xB0
#define STAGE7_CONNECTION	0xB8
#define STAGE8_CONNECTION	0xC0
#define STAGE9_CONNECTION	0xC8
#define STAGE10_CONNECTION	0xD0
#define STAGE11_CONNECTION	0xD8

/*
 *	STAGE0: Button1   <----> CIN6(+)	Button2    <----> CIN5(-)
 *	STAGE1: Button3   <----> CIN4(-)	Button4    <----> CIN3(+)
 *	STAGE2: Axes.Left <----> CIN11(-)	Axes.Right <----> CIN13(+)
 *	STAGE3: Axes.Up   <----> CIN12(-)	Axes.Down  <----> CIN10(+)
 */
static unsigned short stage[5][8] = {
	{0xE7FF, 0x3FFF, 0x0005, 0x2626, 0x01F4, 0x01F4, 0x028A, 0x028A},
	{0xFDBF, 0x3FFF, 0x0001, 0x2626, 0x01F4, 0x01F4, 0x028A, 0x028A},
	{0xFFFF, 0x2DFF, 0x0001, 0x2626, 0x01F4, 0x01F4, 0x028A, 0x028A},
	{0xFFFF, 0x37BF, 0x0001, 0x2626, 0x01F4, 0x01F4, 0x028A, 0x028A},
	{0xFFFF, 0x3FFF, 0x0000, 0x0606, 0x01F4, 0x01F4, 0x0320, 0x0320},
};

struct ad7142_data {
	struct input_dev *input_dev;

	struct i2c_driver i2c_drv;
	struct i2c_client client;

	struct work_struct work;

	unsigned short old_status_low;
	unsigned short old_status_high;

	int irq;
	int is_open;
};

static unsigned short ignore[] = { I2C_CLIENT_END };
static unsigned short normal_addr[] = { AD7142_I2C_ADDR, I2C_CLIENT_END };

static struct i2c_client_address_data addr_data = {
	.normal_i2c = normal_addr,
	.probe = ignore,
	.ignore = ignore,
};

static int ad7142_attach(struct i2c_adapter *adap);
static int ad7142_detach(struct i2c_client *client);
static int ad7142_i2c_read(struct i2c_client *client, unsigned short offset,
		unsigned short *data, unsigned int len);

static struct i2c_driver ad7142_driver = {
	.driver = {
		.name = AD7142_DRV_NAME,
	},
	.id = AD7142_I2C_ID,
	.attach_adapter = ad7142_attach,
	.detach_client = ad7142_detach,
};

static irqreturn_t ad7142_interrupt(int irq, void *_data)
{
	struct ad7142_data *data = _data;

	disable_irq(data->irq);

	if (data->is_open)
		schedule_work(&data->work);

	return IRQ_HANDLED;
}

static int ad7142_i2c_write(struct i2c_client *client, unsigned short offset,
		unsigned short *data, unsigned int len)
{
	int ret = -1;
	int i;
	u8 block_data[34];

	if (len < 1 || len > 16) {
		printk(KERN_ERR "AD7142: Write data length error\n");
		return ret;
	}

	/* Do raw I2C, not smbus compatible */
	block_data[0] = (offset & 0xFF00) >> 8;
	block_data[1] = (offset & 0x00FF);

	for (i = 0; i < len; i++) {
		block_data[2 * i + 2] = (*data & 0xFF00) >> 8;
		block_data[2 * i + 3] = *data++ & 0x00FF;
	}

	ret = i2c_master_send(client, block_data, (len * 2 + 2));
	if (ret < 0) {
		printk(KERN_ERR "AD7142: I2C write error\n");
		return ret;
	}

	return ret;
}

static int ad7142_i2c_read(struct i2c_client *client, unsigned short offset,
		unsigned short *data, unsigned int len)
{
	int ret = -1;
	int i;
	u8 block_data[32];

	if (len < 1 || len > 16) {
		printk(KERN_ERR "AD7142: read data length error\n");
		return ret;
	}

	/* Do raw I2C, not smbus compatible */
	block_data[0] = (offset & 0xFF00) >> 8;
	block_data[1] = (offset & 0x00FF);

	ret = i2c_master_send(client, block_data, 2);
	if (ret < 0) {
		printk(KERN_ERR "AD7142: I2C read error\n");
		return ret;
	}

	ret = i2c_master_recv(client, block_data, len * 2);
	if (ret < 0) {
		printk(KERN_ERR "AD7142: I2C transfer error\n");
		return ret;
	}

	for (i = 0; i < len; i++) {
		unsigned short temp;
		temp = block_data[2 * i];
		temp = (temp << 8) & 0xFF00;
		*data++ = temp | block_data[2 * i + 1];
	}

	return ret;
}

static void ad7142_work(struct work_struct *work)
{
	struct ad7142_data *data = container_of(work, struct ad7142_data, work);
	struct i2c_client *client = &data->client;
	struct input_dev *input_dev = data->input_dev;
	unsigned short irqno_low, irqno_high;
	unsigned short temp;

	ad7142_i2c_read(client, INTSTAT_REG0, &irqno_low, 1);
	temp = irqno_low ^ data->old_status_low;
	switch (temp) {
	case 0x0001:
		input_report_key(input_dev, BTN_BASE, (irqno_low & 0x0001));
		break;
	case 0x0002:
		input_report_key(input_dev, BTN_BASE4,
					((irqno_low & 0x0002) >> 1));
		break;
	case 0x0004:
		input_report_key(input_dev, KEY_UP,
					((irqno_low & 0x0004) >> 2));
		break;
	case 0x0008:
		input_report_key(input_dev, KEY_RIGHT,
					((irqno_low & 0x0008) >> 3));
		break;
	}
	data->old_status_low = irqno_low;

	ad7142_i2c_read(client, INTSTAT_REG1, &irqno_high, 1);
	temp = irqno_high ^ data->old_status_high;
	switch (temp) {
	case 0x0001:
		input_report_key(input_dev, BTN_BASE2, irqno_high & 0x0001);
		break;
	case 0x0002:
		input_report_key(input_dev, BTN_BASE3,
					((irqno_high & 0x0002) >> 1));
		break;
	case 0x0004:
		input_report_key(input_dev, KEY_DOWN,
					((irqno_high & 0x0004) >> 2));
		break;
	case 0x0008:
		input_report_key(input_dev, KEY_LEFT,
					((irqno_high & 0x0008) >> 3));
		break;
	}
	data->old_status_high = irqno_high;

	input_sync(input_dev);

	enable_irq(data->irq);
}

static int ad7142_open(struct input_dev *dev)
{
	struct ad7142_data *data = input_get_drvdata(dev);
	struct i2c_client *client = &data->client;
	unsigned short id, value;

	ad7142_i2c_read(client, DEVID, &id, 1);
	if (id != AD7142_I2C_ID) {
		printk(KERN_ERR "Open AD7142 error\n");
		return -ENODEV;
	}

	ad7142_i2c_write(client, STAGE0_CONNECTION, stage[0], 8);
	ad7142_i2c_write(client, STAGE1_CONNECTION, stage[1], 8);
	ad7142_i2c_write(client, STAGE2_CONNECTION, stage[2], 8);
	ad7142_i2c_write(client, STAGE3_CONNECTION, stage[3], 8);
	ad7142_i2c_write(client, STAGE4_CONNECTION, stage[4], 8);
	ad7142_i2c_write(client, STAGE5_CONNECTION, stage[4], 8);
	ad7142_i2c_write(client, STAGE6_CONNECTION, stage[4], 8);
	ad7142_i2c_write(client, STAGE7_CONNECTION, stage[4], 8);
	ad7142_i2c_write(client, STAGE8_CONNECTION, stage[4], 8);
	ad7142_i2c_write(client, STAGE9_CONNECTION, stage[4], 8);
	ad7142_i2c_write(client, STAGE10_CONNECTION, stage[4], 8);
	ad7142_i2c_write(client, STAGE11_CONNECTION, stage[4], 8);

	/* In full power mode */
	value = 0x00B0;
	ad7142_i2c_write(client, PWRCONVCTL, &value, 1);

	value = 0x0690;
	ad7142_i2c_write(client, AMBCOMPCTL_REG1, &value, 1);

	value = 0x0664;
	ad7142_i2c_write(client, AMBCOMPCTL_REG2, &value, 1);

	value = 0x290F;
	ad7142_i2c_write(client, AMBCOMPCTL_REG3, &value, 1);

	value = 0x000F;
	ad7142_i2c_write(client, INTEN_REG0, &value, 1);
	ad7142_i2c_write(client, INTEN_REG1, &value, 1);

	value = 0x0000;
	ad7142_i2c_write(client, INTEN_REG2, &value, 1);

	ad7142_i2c_read(client, AMBCOMPCTL_REG1, &value, 1);

	value = 0x000F;
	ad7142_i2c_write(client, AMBCOMPCTL_REG0, &value, 1);

	data->is_open = 1;
	enable_irq(data->irq);
	return 0;
}

static void ad7142_close(struct input_dev *dev)
{
	struct ad7142_data *data = input_get_drvdata(dev);
	struct i2c_client *client = &data->client;
	unsigned short value;

	disable_irq(data->irq);
	data->is_open = 0;

	flush_scheduled_work();

	/*
	 * Turn AD7142 to full shutdown mode
	 * No CDC conversions
	 */
	value = 0x0001;
	ad7142_i2c_write(client, PWRCONVCTL, &value, 1);
}

static int ad7142_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct ad7142_data *data;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int rc;

	data = kzalloc(sizeof(struct ad7142_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	client = &data->client;
	i2c_set_clientdata(client, data);

	strlcpy(client->name, AD7142_DRV_NAME, I2C_NAME_SIZE);
	client->addr = addr;
	client->adapter = adap;
	client->driver = &ad7142_driver;

	rc = i2c_attach_client(client);
	if (rc) {
		printk(KERN_ERR "i2c_attach_client fail: %d\n", rc);
		goto fail_attach;
	}

	/*
	 * The ADV7142 has an autoincrement function,
	 * use it if the adapter understands raw I2C
	 */
	rc = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if (!rc) {
		printk(KERN_ERR
			"AD7142: i2c bus doesn't support raw I2C operation\n");
		rc = -ENOSYS;
		goto fail_check;
	}

	/* Start workqueue for defer message transfer */
	INIT_WORK(&data->work, ad7142_work);

	data->irq = CONFIG_BFIN_JOYSTICK_IRQ_PFX;
	rc = request_irq(data->irq, ad7142_interrupt,
		IRQF_TRIGGER_LOW, "ad7142_joy", data);
	if (rc) {
		printk(KERN_ERR "AD7142: Can't allocate irq %d\n", data->irq);
		goto fail_check;
	}

	printk(KERN_INFO "%s_attach: at 0x%02x\n",
			client->name, client->addr << 1);

	/* Allocate and register AD7142 input device */
	data->input_dev = input_allocate_device();
	if (!data->input_dev) {
		printk(KERN_ERR "AD7142: Can't allocate input device\n");
		rc = -ENOMEM;
		goto fail_allocate;
	}

	input_dev = data->input_dev;
	input_dev->open = ad7142_open;
	input_dev->close = ad7142_close;
	input_dev->evbit[0] = BIT(EV_KEY);
	input_dev->keybit[LONG(BTN_BASE)] = BIT(BTN_BASE) | BIT(BTN_BASE2) |
						BIT(BTN_BASE3) | BIT(BTN_BASE4);
	input_dev->keybit[LONG(KEY_UP)] |= BIT(KEY_UP) | BIT(KEY_DOWN) |
						BIT(KEY_LEFT) | BIT(KEY_RIGHT);

	input_dev->name = "ad7142 joystick";
	input_dev->phys = "ad7142/input0";
	input_dev->id.bustype = BUS_I2C;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0100;

	input_set_drvdata(input_dev, data);

	rc = input_register_device(input_dev);
	if (rc) {
		printk(KERN_ERR "Failed to register AD7142 input device!\n");
		goto fail_register;
	}

	return 0;

fail_register:
	input_free_device(input_dev);
fail_allocate:
	free_irq(data->irq, ad7142_interrupt);
fail_check:
	i2c_detach_client(client);
fail_attach:
	kfree(client);
	return rc;
}

static int ad7142_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, &ad7142_probe);
}

static int ad7142_detach(struct i2c_client *client)
{
	struct ad7142_data *data = i2c_get_clientdata(client);

	free_irq(data->irq, ad7142_interrupt);

	flush_scheduled_work();

	input_unregister_device(data->input_dev);

	kfree(data);

	return i2c_detach_client(client);
}


static int __init ad7142_init(void)
{
	return i2c_add_driver(&ad7142_driver);
}

static void __exit ad7142_exit(void)
{
	i2c_del_driver(&ad7142_driver);
}

module_init(ad7142_init);
module_exit(ad7142_exit);
