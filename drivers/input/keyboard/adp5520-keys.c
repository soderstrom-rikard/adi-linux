/*
 * Keypad driver for Analog Devices ADP5520 MFD PMICs
 *
 * Copyright 2009 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/mfd/adp5520.h>

struct adp5520_keys {
	struct input_dev	*input;
	struct notifier_block	notifier;
	struct device *master;
	unsigned short keycode[ADP5520_KEYMAPSIZE];
};

static void adp5520_keys_report_event(struct adp5520_keys *dev,
					unsigned short keymask, int value)
{
	int i;

	for (i = 0; i < ADP5520_MAXKEYS; i++)
		if (keymask & (1 << i))
			input_report_key(dev->input, dev->keycode[i], value);

	input_sync(dev->input);
}

static int adp5520_keys_notifier(struct notifier_block *nb,
				 unsigned long event, void *data)
{
	struct adp5520_keys *dev;
	uint8_t reg_val_low, reg_val_high;
	unsigned short keymask;

	dev = container_of(nb, struct adp5520_keys, notifier);

	if (event & KP_INT) {
		adp5520_read(dev->master, KP_INT_STAT_1, &reg_val_low);
		adp5520_read(dev->master, KP_INT_STAT_2, &reg_val_high);

		keymask = (reg_val_high << 8) | reg_val_low;
		/* Read twice to clear */
		adp5520_read(dev->master, KP_INT_STAT_1, &reg_val_low);
		adp5520_read(dev->master, KP_INT_STAT_2, &reg_val_high);
		keymask |= (reg_val_high << 8) | reg_val_low;
		adp5520_keys_report_event(dev, keymask, 1);
	}

	if (event & KR_INT) {
		adp5520_read(dev->master, KR_INT_STAT_1, &reg_val_low);
		adp5520_read(dev->master, KR_INT_STAT_2, &reg_val_high);

		keymask = (reg_val_high << 8) | reg_val_low;
		/* Read twice to clear */
		adp5520_read(dev->master, KR_INT_STAT_1, &reg_val_low);
		adp5520_read(dev->master, KR_INT_STAT_2, &reg_val_high);
		keymask |= (reg_val_high << 8) | reg_val_low;
		adp5520_keys_report_event(dev, keymask, 0);
	}

	return 0;
}

static int __devinit adp5520_keys_probe(struct platform_device *pdev)
{
	struct adp5520_keys_platfrom_data *pdata = pdev->dev.platform_data;
	struct input_dev *input;
	struct adp5520_keys *dev;
	int ret, i;
	unsigned char en_mask, ctl_mask = 0;

	if (pdev->id != ID_ADP5520) {
		dev_err(&pdev->dev, "only ADP5520 supports Keypad\n");
		return -EFAULT;
	}

	if (pdata == NULL) {
		dev_err(&pdev->dev, "missing platform data\n");
		return -ENODEV;
	}

	if (!(pdata->rows_en_mask && pdata->cols_en_mask))
		return -ENODEV;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (dev == NULL) {
		dev_err(&pdev->dev, "failed to alloc memory\n");
		return -ENOMEM;
	}

	input = input_allocate_device();
	if (!input) {
		ret = -ENOMEM;
		goto err;
	}

	dev->master = pdev->dev.parent;
	dev->input = input;

	input->name = pdev->name;
	input->phys = "adp5520-keys/input0";
	input->dev.parent = &pdev->dev;

	input_set_drvdata(input, dev);

	input->id.bustype = BUS_I2C;
	input->id.vendor = 0x0001;
	input->id.product = 0x5520;
	input->id.version = 0x0001;

	input->keycodesize = sizeof(unsigned short);
	input->keycodemax = pdata->keymapsize;
	input->keycode = dev->keycode;

	memcpy(dev->keycode, pdata->keymap,
		pdata->keymapsize * input->keycodesize);

	/* setup input device */
	__set_bit(EV_KEY, input->evbit);

	if (pdata->repeat)
		__set_bit(EV_REP, input->evbit);

	for (i = 0; i < input->keycodemax; i++)
		__set_bit(dev->keycode[i] & KEY_MAX, input->keybit);
	__clear_bit(KEY_RESERVED, input->keybit);

	ret = input_register_device(input);
	if (ret) {
		dev_err(&pdev->dev,
			": Unable to register input device (%d)\n", ret);
		input_free_device(input);
		goto err;
	}

	en_mask = pdata->rows_en_mask | pdata->cols_en_mask;

	ret = adp5520_set_bits(dev->master, GPIO_CFG_1, en_mask);

	if (en_mask & COL_C3)
		ctl_mask |= C3_MODE;

	if (en_mask & ROW_R3)
		ctl_mask |= R3_MODE;

	if (ctl_mask)
		ret |= adp5520_set_bits(dev->master, LED_CONTROL,
			ctl_mask);

	ret |= adp5520_set_bits(dev->master, GPIO_PULLUP,
		pdata->rows_en_mask);

	if (ret) {
		dev_err(&pdev->dev, "failed to write\n");
		goto err1;
	}

	dev->notifier.notifier_call = adp5520_keys_notifier;
	ret = adp5520_register_notifier(dev->master, &dev->notifier,
			KP_IEN | KR_IEN);
	if (ret) {
		dev_err(&pdev->dev, "failed to register notifier\n");
		goto err1;
	}

	platform_set_drvdata(pdev, dev);
	return 0;

err1:
	input_unregister_device(input);
err:
	kfree(dev);
	return ret;
}

static int __devexit adp5520_keys_remove(struct platform_device *pdev)
{
	struct adp5520_keys *dev;
	dev = platform_get_drvdata(pdev);

	adp5520_unregister_notifier(dev->master, &dev->notifier,
				KP_IEN | KR_IEN);

	input_unregister_device(dev->input);
	kfree(dev);
	return 0;
}

static struct platform_driver adp5520_keys_driver = {
	.driver	= {
		.name	= "adp5520-keys",
		.owner	= THIS_MODULE,
	},
	.probe		= adp5520_keys_probe,
	.remove		= __devexit_p(adp5520_keys_remove),
};

static int __init adp5520_keys_init(void)
{
	return platform_driver_register(&adp5520_keys_driver);
}
module_init(adp5520_keys_init);

static void __exit adp5520_keys_exit(void)
{
	platform_driver_unregister(&adp5520_keys_driver);
}
module_exit(adp5520_keys_exit);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("Keys ADP5520 Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:adp5520-keys");
