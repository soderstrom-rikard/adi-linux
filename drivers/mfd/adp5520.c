/*
 * Base driver for Analog Devices ADP5520/ADP5501 MFD PMICs
 * LCD Backlight: drivers/video/backlight/adp5520_bl
 * LEDs		: drivers/led/leds-adp5520
 * GPIO		: drivers/gpio/adp5520-gpio (ADP5520 only)
 * Keys		: drivers/input/keyboard/adp5520-keys (ADP5520 only)
 *
 * Copyright 2009 Analog Devices Inc.
 *
 * Derived from da903x:
 * Copyright (C) 2008 Compulab, Ltd.
 * 	Mike Rapoport <mike@compulab.co.il>
 *
 * Copyright (C) 2006-2008 Marvell International Ltd.
 * 	Eric Miao <eric.miao@marvell.com>
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>

#include <linux/mfd/adp5520.h>

struct adp5520_chip {
	struct i2c_client *client;
	struct device *dev;
	struct mutex lock;
	struct work_struct irq_work;
	struct blocking_notifier_head notifier_list;
	int irq;
};

static int __adp5520_read(struct i2c_client *client,
				int reg, uint8_t *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "failed reading at 0x%02x\n", reg);
		return ret;
	}

	*val = (uint8_t)ret;
	return 0;
}

static int __adp5520_write(struct i2c_client *client,
				 int reg, uint8_t val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0) {
		dev_err(&client->dev, "failed writing 0x%02x to 0x%02x\n",
				val, reg);
		return ret;
	}
	return 0;
}

int __adp5520_ack_bits(struct i2c_client *client, int reg, uint8_t bit_mask)
{
	struct adp5520_chip *chip = i2c_get_clientdata(client);
	uint8_t reg_val;
	int ret;

	mutex_lock(&chip->lock);

	ret = __adp5520_read(client, reg, &reg_val);

	if (!ret) {
		reg_val |= bit_mask;
		ret = __adp5520_write(client, reg, reg_val);
	}

	mutex_unlock(&chip->lock);
	return ret;
}

int adp5520_write(struct device *dev, int reg, uint8_t val)
{
	return __adp5520_write(to_i2c_client(dev), reg, val);
}
EXPORT_SYMBOL_GPL(adp5520_write);

int adp5520_read(struct device *dev, int reg, uint8_t *val)
{
	return __adp5520_read(to_i2c_client(dev), reg, val);
}
EXPORT_SYMBOL_GPL(adp5520_read);

int adp5520_set_bits(struct device *dev, int reg, uint8_t bit_mask)
{
	struct adp5520_chip *chip = dev_get_drvdata(dev);
	uint8_t reg_val;
	int ret;

	mutex_lock(&chip->lock);

	ret = __adp5520_read(chip->client, reg, &reg_val);

	if (!ret && ((reg_val & bit_mask) == 0)) {
		reg_val |= bit_mask;
		ret = __adp5520_write(chip->client, reg, reg_val);
	}

	mutex_unlock(&chip->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(adp5520_set_bits);

int adp5520_clr_bits(struct device *dev, int reg, uint8_t bit_mask)
{
	struct adp5520_chip *chip = dev_get_drvdata(dev);
	uint8_t reg_val;
	int ret;

	mutex_lock(&chip->lock);

	ret = __adp5520_read(chip->client, reg, &reg_val);

	if (!ret && (reg_val & bit_mask)) {
		reg_val &= ~bit_mask;
		ret = __adp5520_write(chip->client, reg, reg_val);
	}

	mutex_unlock(&chip->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(adp5520_clr_bits);

int adp5520_register_notifier(struct device *dev, struct notifier_block *nb,
				unsigned int events)
{
	struct adp5520_chip *chip = dev_get_drvdata(dev);

	if (chip->irq) {
		adp5520_set_bits(chip->dev, INTERRUPT_ENABLE,
			events & (KP_IEN | KR_IEN | OVP_IEN | CMPR_IEN));

		return blocking_notifier_chain_register(&chip->notifier_list,
			 nb);
	}

	return -ENODEV;
}
EXPORT_SYMBOL_GPL(adp5520_register_notifier);

int adp5520_unregister_notifier(struct device *dev, struct notifier_block *nb,
				unsigned int events)
{
	struct adp5520_chip *chip = dev_get_drvdata(dev);

	adp5520_clr_bits(chip->dev, INTERRUPT_ENABLE,
		events & (KP_IEN | KR_IEN | OVP_IEN | CMPR_IEN));

	return blocking_notifier_chain_unregister(&chip->notifier_list, nb);
}
EXPORT_SYMBOL_GPL(adp5520_unregister_notifier);

static void adp5520_irq_work(struct work_struct *work)
{
	struct adp5520_chip *chip =
		container_of(work, struct adp5520_chip, irq_work);
	unsigned int events;
	uint8_t reg_val;
	int ret;

	ret = __adp5520_read(chip->client, MODE_STATUS, &reg_val);
	if (ret)
		goto out;

	events =  reg_val & (OVP_INT | CMPR_INT | GPI_INT | KR_INT | KP_INT);

	blocking_notifier_call_chain(&chip->notifier_list, events, NULL);
	/* ACK, Sticky bits are W1C */
	__adp5520_ack_bits(chip->client, MODE_STATUS, events);

out:
	enable_irq(chip->client->irq);
}

static int adp5520_irq_handler(int irq, void *data)
{
	struct adp5520_chip *chip = data;

	disable_irq_nosync(irq);
	schedule_work(&chip->irq_work);

	return IRQ_HANDLED;
}

static int __remove_subdev(struct device *dev, void *unused)
{
	platform_device_unregister(to_platform_device(dev));
	return 0;
}

static int adp5520_remove_subdevs(struct adp5520_chip *chip)
{
	return device_for_each_child(chip->dev, NULL, __remove_subdev);
}

static int __devinit adp5520_add_subdevs(struct adp5520_chip *chip,
					struct adp5520_platform_data *pdata)
{
	struct adp5520_subdev_info *subdev;
	struct platform_device *pdev;
	int i, ret = 0;

	for (i = 0; i < pdata->num_subdevs; i++) {
		subdev = &pdata->subdevs[i];

		pdev = platform_device_alloc(subdev->name, subdev->id);

		pdev->dev.parent = chip->dev;
		pdev->dev.platform_data = subdev->platform_data;

		ret = platform_device_add(pdev);
		if (ret)
			goto failed;
	}
	return 0;

failed:
	adp5520_remove_subdevs(chip);
	return ret;
}

static int __devinit adp5520_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct adp5520_platform_data *pdata = client->dev.platform_data;
	struct adp5520_chip *chip;
	int ret;

	if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "SMBUS Word Data not Supported\n");
		return -EIO;
	}

	if (pdata == NULL) {
		dev_err(&client->dev, "missing platform data\n");
		return -ENODEV;
	}

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	i2c_set_clientdata(client, chip);
	chip->client = client;

	chip->dev = &client->dev;
	chip->irq = client->irq;
	mutex_init(&chip->lock);

	if (chip->irq) {
		INIT_WORK(&chip->irq_work, adp5520_irq_work);
		BLOCKING_INIT_NOTIFIER_HEAD(&chip->notifier_list);

		ret = request_irq(chip->irq, adp5520_irq_handler,
				IRQF_SAMPLE_RANDOM | IRQF_DISABLED |
				IRQF_TRIGGER_LOW,
				"adp5520", chip);
		if (ret) {
			dev_err(&client->dev, "failed to request irq %d\n",
					chip->irq);
			goto out_free_chip;
		}
	}

	ret = adp5520_write(chip->dev, MODE_STATUS, nSTNBY);
	if (ret) {
		dev_err(&client->dev, "failed to write\n");
		goto out_free_irq;
	}

	ret = adp5520_add_subdevs(chip, pdata);

	if (!ret)
		return ret;

out_free_irq:
	if (chip->irq)
		free_irq(chip->irq, chip);

out_free_chip:
	i2c_set_clientdata(client, NULL);
	kfree(chip);

	return ret;
}

static int __devexit adp5520_remove(struct i2c_client *client)
{
	struct adp5520_chip *chip = dev_get_drvdata(&client->dev);

	if (chip->irq)
		free_irq(chip->irq, chip);

	adp5520_remove_subdevs(chip);
	adp5520_write(chip->dev, MODE_STATUS, 0);
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM
static int adp5520_suspend(struct i2c_client *client,
				 pm_message_t state)
{
	struct adp5520_chip *chip = dev_get_drvdata(&client->dev);

	adp5520_clr_bits(chip->dev, MODE_STATUS, nSTNBY);
	return 0;
}

static int adp5520_resume(struct i2c_client *client)
{
	struct adp5520_chip *chip = dev_get_drvdata(&client->dev);

	adp5520_set_bits(chip->dev, MODE_STATUS, nSTNBY);
	return 0;
}
#else
#define adp5520_suspend	NULL
#define adp5520_resume	NULL
#endif

static const struct i2c_device_id adp5520_id[] = {
	{ "pmic-adp5520", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, adp5520_id);

static struct i2c_driver adp5520_driver = {
	.driver = {
		.name	= "adp5520",
		.owner	= THIS_MODULE,
	},
	.probe		= adp5520_probe,
	.remove		= __devexit_p(adp5520_remove),
	.suspend	= adp5520_suspend,
	.resume		= adp5520_resume,
	.id_table 	= adp5520_id,
};

static int __init adp5520_init(void)
{
	return i2c_add_driver(&adp5520_driver);
}
module_init(adp5520_init);

static void __exit adp5520_exit(void)
{
	i2c_del_driver(&adp5520_driver);
}
module_exit(adp5520_exit);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("ADP5520(01) PMIC-MFD Driver");
MODULE_LICENSE("GPL");
