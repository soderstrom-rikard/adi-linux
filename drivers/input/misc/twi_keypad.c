/*
 * TWI Driver for an 4x4 Keybaord Matrix connected to a PCF8574 I2C IO expander
 *
 * Copyright 2005-2008 Analog Devices Inc.
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("TWI Keypad input driver");
MODULE_LICENSE("GPL");

static unsigned char twi_keypad_btncode[] = {
	[0] = KEY_RESERVED,
	[1] = KEY_ENTER,
	[2] = KEY_BACKSLASH,
	[3] = KEY_0,
	[4] = KEY_RIGHTBRACE,
	[5] = KEY_C,
	[6] = KEY_9,
	[7] = KEY_8,
	[8] = KEY_7,
	[9] = KEY_B,
	[10] = KEY_6,
	[11] = KEY_5,
	[12] = KEY_4,
	[13] = KEY_A,
	[14] = KEY_3,
	[15] = KEY_2,
	[16] = KEY_1
};

struct twikeypad {
	unsigned char *btncode;
	struct input_dev *idev;
	struct i2c_client *client;
	char name[64];
	char phys[32];
	unsigned char laststate;
	unsigned char statechanged;
	unsigned long irq_handled;
	unsigned long events_sended;
	unsigned long events_processed;
	struct work_struct twi_keypad_work;
};

#define	PCF8574_KP_DRV_NAME "pcf8574_keypad"

static short read_state(struct twikeypad *lp)
{
	unsigned char x, y, a, b;

	if (lp->client) {
		i2c_smbus_write_byte(lp->client, 240);
		x = 0xF & (~(i2c_smbus_read_byte(lp->client) >> 4));

		i2c_smbus_write_byte(lp->client, 15);
		y = 0xF & (~i2c_smbus_read_byte(lp->client));

		for (a = 0; x > 0; a++)
			x = x >> 1;
		for (b = 0; y > 0; b++)
			y = y >> 1;

		return (((a - 1) * 4) + b);
	}

	return -1;
}

static void check_and_notify(struct work_struct *work)
{
	struct twikeypad *lp =
	    container_of(work, struct twikeypad, twi_keypad_work);
	unsigned char nextstate = read_state(lp);

	lp->statechanged = lp->laststate ^ nextstate;

	if (lp->statechanged) {
		input_report_key(lp->idev,
				 nextstate > 17 ? lp->btncode[lp->laststate] :
				 lp->btncode[nextstate],
				 nextstate > 17 ? 0 : 1);

		lp->events_sended++;
	}
	lp->laststate = nextstate;
	input_sync(lp->idev);
	enable_irq(lp->client->irq);
}

static irqreturn_t twi_keypad_irq_handler(int irq, void *dev_id)
{
	struct twikeypad *lp = dev_id;

	disable_irq_nosync(lp->client->irq);
	schedule_work(&lp->twi_keypad_work);

	return IRQ_HANDLED;
}

static int pcf8574_kp_probe(struct i2c_client *client)
{
	int i, rc;
	struct input_dev *idev;
	struct twikeypad *lp;

	if (i2c_smbus_write_byte(client, 240) < 0) {
		dev_err(&client->dev, "in keypad probe: write fail\n");
		return -ENODEV;
	}

	lp = kzalloc(sizeof(struct twikeypad), GFP_KERNEL);
	if (!lp)
		return -ENOMEM;
	lp->client = client;

	i2c_set_clientdata(client, lp);

	if (client->irq > 0) {
		rc = request_irq(client->irq, twi_keypad_irq_handler,
			IRQF_TRIGGER_LOW, PCF8574_KP_DRV_NAME, lp);
		if (rc) {
			dev_err(&client->dev, "twikeypad: IRQ %d is not free.\n",
				client->irq);
			goto fail_irq;
		}
	} else
		dev_warn(&client->dev, "IRQ not configured!\n");


	idev = input_allocate_device();
	if (!idev) {
		dev_err(&client->dev, "Can't allocate input device\n");
		rc = -ENOMEM;
		goto fail_allocate;
	}

	lp->idev = idev;
	lp->btncode = twi_keypad_btncode;

	idev->evbit[0] = 0;

	idev->evbit[0] |= BIT_MASK(EV_KEY);
	idev->keycode = lp->btncode;
	idev->keycodesize = sizeof(twi_keypad_btncode);
	idev->keycodemax = ARRAY_SIZE(twi_keypad_btncode);

	for (i = 0; i <= ARRAY_SIZE(twi_keypad_btncode); i++)
		__set_bit(lp->btncode[i] & KEY_MAX, idev->keybit);

	__clear_bit(KEY_RESERVED, idev->keybit);

	sprintf(lp->name, "BF5xx twikeypad");
	sprintf(lp->phys, "twikeypad/input0");

	idev->name = lp->name;
	idev->phys = lp->phys;
	idev->id.bustype = BUS_I2C;
	idev->id.vendor = 0x0001;
	idev->id.product = 0x0001;
	idev->id.version = 0x0100;

	input_set_drvdata(idev, lp);

	rc = input_register_device(lp->idev);
	if (rc) {
		dev_err(&client->dev,
			"Failed to register TWI keypad input device!\n");
		goto fail_register;
	}

	lp->statechanged = 0x0;

	lp->laststate = read_state(lp);

	/* Set up our workqueue. */
	INIT_WORK(&lp->twi_keypad_work, check_and_notify);

	dev_info(&client->dev, "input: %s at %s IRQ %d\n", lp->name, lp->phys,
				 client->irq);

	return 0;


fail_register:
	input_set_drvdata(idev, NULL);
	input_free_device(idev);
fail_allocate:
	free_irq(client->irq, lp);
fail_irq:
	i2c_set_clientdata(client, NULL);
	kfree(lp);

	return rc;
}



static int __exit pcf8574_kp_remove(struct i2c_client *client)
{
	struct twikeypad *lp = i2c_get_clientdata(client);

	if (client->irq > 0)
		free_irq(client->irq, lp);

	input_set_drvdata(lp->idev, NULL);
	input_unregister_device(lp->idev);
	kfree(lp);

	i2c_set_clientdata(client, NULL);

	return 0;
}

static struct i2c_driver pcf8574_kp_driver = {
	.driver = {
		.name = PCF8574_KP_DRV_NAME,
	},
	.probe = pcf8574_kp_probe,
	.remove = __exit_p(pcf8574_kp_remove),
};

static int __init twi_keypad_init(void)
{
	return i2c_add_driver(&pcf8574_kp_driver);
}

void __exit twi_keypad_exit(void)
{
	i2c_del_driver(&pcf8574_kp_driver);
}

module_init(twi_keypad_init);
module_exit(twi_keypad_exit);
