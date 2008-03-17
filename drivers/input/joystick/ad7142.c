/*
 * I2C Based AD7142 Joystick Input Device Driver
 *
 * Copyright 2005-2008 Analog Devices Inc.
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>

MODULE_AUTHOR("Bryan Wu <cooloney@kernel.org>");
MODULE_DESCRIPTION("Driver for AD7142 Joysticks");
MODULE_LICENSE("GPL");

#define AD7142_I2C_ID		0xE622

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
static const unsigned short stage[5][8] = {
	{0xE7FF, 0x3FFF, 0x0005, 0x2626, 0x01F4, 0x01F4, 0x028A, 0x028A},
	{0xFDBF, 0x3FFF, 0x0001, 0x2626, 0x01F4, 0x01F4, 0x028A, 0x028A},
	{0xFFFF, 0x2DFF, 0x0001, 0x2626, 0x01F4, 0x01F4, 0x028A, 0x028A},
	{0xFFFF, 0x37BF, 0x0001, 0x2626, 0x01F4, 0x01F4, 0x028A, 0x028A},
	{0xFFFF, 0x3FFF, 0x0000, 0x0606, 0x01F4, 0x01F4, 0x0320, 0x0320},
};

struct ad7142_data {
	struct input_dev *input;
	struct i2c_client *client;

	struct work_struct work;
	int is_open;

	unsigned short old_status_low;
	unsigned short old_status_high;
};

static irqreturn_t ad7142_interrupt(int irq, void *_data)
{
	struct ad7142_data *data = _data;

	disable_irq_nosync(irq);
	if (data->is_open)
		schedule_work(&data->work);

	return IRQ_HANDLED;
}

static int ad7142_i2c_write(struct i2c_client *client, unsigned short offset,
			const unsigned short *data, unsigned int len)
{
	int ret = -1;
	int i;
	u8 block_data[34];

	if (len < 1 || len > 16) {
		dev_err(&client->dev, "Write data length error\n");
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
		dev_err(&client->dev, "I2C write error\n");
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
		dev_err(&client->dev, "read data length error\n");
		return ret;
	}

	/* Do raw I2C, not smbus compatible */
	block_data[0] = (offset & 0xFF00) >> 8;
	block_data[1] = (offset & 0x00FF);

	ret = i2c_master_send(client, block_data, 2);
	if (ret < 0) {
		dev_err(&client->dev, "I2C read error\n");
		return ret;
	}

	ret = i2c_master_recv(client, block_data, len * 2);
	if (ret < 0) {
		dev_err(&client->dev, "I2C transfer error\n");
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
	struct ad7142_data *data = container_of(work,
						struct ad7142_data,
						work);
	struct i2c_client *client = data->client;
	struct input_dev *input = data->input;
	unsigned short irqno_low, irqno_high;
	unsigned short temp;

	ad7142_i2c_read(client, INTSTAT_REG0, &irqno_low, 1);
	temp = irqno_low ^ data->old_status_low;
	switch (temp) {
	case 0x0001:
		input_report_key(input, BTN_BASE, (irqno_low & 0x0001));
		break;
	case 0x0002:
		input_report_key(input, BTN_BASE4,
					((irqno_low & 0x0002) >> 1));
		break;
	case 0x0004:
		input_report_key(input, KEY_UP,
					((irqno_low & 0x0004) >> 2));
		break;
	case 0x0008:
		input_report_key(input, KEY_RIGHT,
					((irqno_low & 0x0008) >> 3));
		break;
	}
	data->old_status_low = irqno_low;

	ad7142_i2c_read(client, INTSTAT_REG1, &irqno_high, 1);
	temp = irqno_high ^ data->old_status_high;
	switch (temp) {
	case 0x0001:
		input_report_key(input, BTN_BASE2, irqno_high & 0x0001);
		break;
	case 0x0002:
		input_report_key(input, BTN_BASE3,
					((irqno_high & 0x0002) >> 1));
		break;
	case 0x0004:
		input_report_key(input, KEY_DOWN,
					((irqno_high & 0x0004) >> 2));
		break;
	case 0x0008:
		input_report_key(input, KEY_LEFT,
					((irqno_high & 0x0008) >> 3));
		break;
	}
	data->old_status_high = irqno_high;

	input_sync(input);

	enable_irq(client->irq);
}

static int ad7142_open(struct input_dev *dev)
{
	struct ad7142_data *data = input_get_drvdata(dev);
	struct i2c_client *client = data->client;
	unsigned short id, value;

	ad7142_i2c_read(client, DEVID, &id, 1);
	if (id != AD7142_I2C_ID) {
		dev_err(&client->dev, "Open AD7142 error\n");
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
	enable_irq(client->irq);
	return 0;
}

static void ad7142_close(struct input_dev *dev)
{
	struct ad7142_data *data = input_get_drvdata(dev);
	struct i2c_client *client = data->client;
	unsigned short value;

	disable_irq(client->irq);
	data->is_open = 0;

	flush_scheduled_work();

	/*
	 * Turn AD7142 to full shutdown mode
	 * No CDC conversions
	 */
	value = 0x0001;
	ad7142_i2c_write(client, PWRCONVCTL, &value, 1);
}

static int ad7142_probe(struct i2c_client *client)
{
	struct ad7142_data *data;
	struct input_dev *input;
	int rc;

	/*
	 * The ADV7142 has an autoincrement function,
	 * use it if the adapter understands raw I2C
	 */
	rc = i2c_check_functionality(client->adapter, I2C_FUNC_I2C);
	if (!rc) {
		dev_err(&client->dev,
			"This bus doesn't support raw I2C operation\n");
		return -EINVAL;
	}

	data = kzalloc(sizeof(struct ad7142_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->client = client;

	i2c_set_clientdata(client, data);

	/* Start workqueue for defer message transfer */
	INIT_WORK(&data->work, ad7142_work);

	if (client->irq > 0) {
		rc = request_irq(client->irq, ad7142_interrupt,
				IRQF_TRIGGER_LOW, "ad7142_joystick", data);
		if (rc) {
			dev_err(&client->dev, "Can't allocate irq %d\n",
				client->irq);
			goto fail_irq;
		}
	} else
		dev_warn(&client->dev, "IRQ not configured!\n");

	dev_info(&client->dev, "is attached at 0x%02x\n", client->addr);

	/* Allocate and register AD7142 input device */
	data->input = input_allocate_device();
	if (!data->input) {
		dev_err(&client->dev, "Can't allocate input device\n");
		rc = -ENOMEM;
		goto fail_allocate;
	}

	input = data->input;
	input->open = ad7142_open;
	input->close = ad7142_close;
	input->evbit[0] = BIT_MASK(EV_KEY);
	input->keybit[BIT_WORD(BTN_BASE)] = BIT_MASK(BTN_BASE) |
						BIT_MASK(BTN_BASE2) |
						BIT_MASK(BTN_BASE3) |
						BIT_MASK(BTN_BASE4);
	input->keybit[BIT_WORD(KEY_UP)] |=  BIT_MASK(KEY_UP) |
						BIT_MASK(KEY_DOWN) |
						BIT_MASK(KEY_LEFT) |
						BIT_MASK(KEY_RIGHT);

	input->name = "ad7142 joystick";
	input->phys = "ad7142/input0";
	input->id.bustype = BUS_I2C;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	input_set_drvdata(input, data);

	rc = input_register_device(input);
	if (rc) {
		dev_err(&client->dev,
			"Failed to register AD7142 input device!\n");
		goto fail_register;
	}

	return 0;

fail_register:
	input_free_device(input);
fail_allocate:
	free_irq(client->irq, data);
fail_irq:
	kfree(data);
	return rc;
}

static int __exit ad7142_remove(struct i2c_client *client)
{
	struct ad7142_data *data = i2c_get_clientdata(client);

	if (client->irq > 0)
		free_irq(client->irq, data);

	flush_scheduled_work();
	input_unregister_device(data->input);
	kfree(data);
	return 0;
}

static struct i2c_driver ad7142_driver = {
	.driver = {
		.name = "ad7142_joystick",
	},
	.probe = ad7142_probe,
	.remove = __exit_p(ad7142_remove),
};

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

