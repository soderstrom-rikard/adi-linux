/*
 * adp_switch.c  --  Voltage regulation for switch only power devices
 *		     AD122, AD123, AD124, AD125, AD150, AD5022, etc
 *
 * Copyright 2010 Analog Devices Inc.
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Licensed under the GPL-2 or later.
 */
#include <linux/module.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/adp_switch.h>

struct adp_switch_chip_info {
	unsigned short gpio_port;
	unsigned short enabled;
	struct regulator_dev *rdev;
	struct regulator_desc rdesc;
};

static int adp_switch_is_enabled(struct regulator_dev *rdev)
{
	struct adp_switch_chip_info *chip = rdev_get_drvdata(rdev);

	return chip->enabled;
}

static int adp_switch_enable(struct regulator_dev *rdev)
{
	struct adp_switch_chip_info *chip = rdev_get_drvdata(rdev);

	gpio_set_value(chip->gpio_port, 1);

	return 0;
}

static int adp_switch_disable(struct regulator_dev *rdev)
{
	struct adp_switch_chip_info *chip = rdev_get_drvdata(rdev);

	gpio_set_value(chip->gpio_port, 0);

	return 0;
}

static struct regulator_ops adp_switch_ops = {
	.enable = adp_switch_enable,
	.disable = adp_switch_disable,
	.is_enabled = adp_switch_is_enabled,
	.set_suspend_enable = adp_switch_enable,
	.set_suspend_disable = adp_switch_disable,
};

static int __devinit adp_switch_probe(struct platform_device *pdev)
{
	struct regulator_dev *rdev;
	struct adp_switch_platform_data *pdata = pdev->dev.platform_data;
	struct adp_switch_chip_info *chip;
	int i, ret = 0;

	dev_dbg(&pdev->dev, "%s enter\n", __func__);

	if (!pdata || !pdata->regulator_num)
		return -EINVAL;

	chip = kzalloc(sizeof(struct adp_switch_chip_info) *
			pdata->regulator_num, GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	for (i = 0; i < pdata->regulator_num; i++) {
		/*
		 * The GPIO port of the specific regulator is in driver_data field
		 * of struct regulator_init_data defined in its platform board file.
		 */
		chip[i].gpio_port =
		(unsigned short)(unsigned int)pdata->regulator_data[i].driver_data;
		chip[i].rdesc.name = pdata->regulator_data[i].consumer_supplies->supply;
		chip[i].rdesc.id = i,
		chip[i].rdesc.ops = &adp_switch_ops,
		chip[i].rdesc.type = REGULATOR_CURRENT,
		chip[i].rdesc.owner = THIS_MODULE,

		ret = gpio_request(chip[i].gpio_port, "adp_switch");
		if (ret) {
			dev_err(&pdev->dev,
				"Fail to request adp switch peripherals\n");
			goto err;
		}

		rdev = regulator_register(&chip[i].rdesc, &pdev->dev,
				 &pdata->regulator_data[i], &chip[i]);
		if (IS_ERR(rdev)) {
			ret = PTR_ERR(rdev);
			dev_err(&pdev->dev, "failed to register %s\n",
				chip[i].rdesc.name);
			gpio_free(chip[i].gpio_port);
			goto err;
		}

		gpio_direction_output(chip[i].gpio_port, 0);
		chip[i].rdev = rdev;
	}

	dev_set_drvdata(&pdev->dev, chip);
	dev_info(&pdev->dev, "regulator driver loaded\n");
	return 0;

err:
	while (--i >= 0) {
		regulator_unregister(chip[i].rdev);
		gpio_free(chip[i].gpio_port);
	}
	kfree(chip);
	return ret;
}

static int __devexit adp_switch_remove(struct platform_device *pdev)
{
	struct adp_switch_chip_info *chip = platform_get_drvdata(pdev);
	struct adp_switch_platform_data *pdata = pdev->dev.platform_data;
	int i;

	dev_dbg(&pdev->dev, "%s enter\n", __func__);
	dev_set_drvdata(&pdev->dev, NULL);


	if (chip) {
		for (i = 0; i < pdata->regulator_num; i++) {
			regulator_unregister(chip[i].rdev);
			gpio_free(chip[i].gpio_port);
		}
		kfree(chip);
	}

	return 0;
}

static struct platform_driver adp_switch_driver = {
	.probe          = adp_switch_probe,
	.remove         = __devexit_p(adp_switch_remove),
	.driver         = {
		.name   = "adp_switch",
	},
};

static int __init adp_switch_init(void)
{
	int ret;

	ret = platform_driver_register(&adp_switch_driver);
	if (ret)
		pr_err("failed to register adp switch driver:%d\n", ret);

	return ret;
}
module_init(adp_switch_init);

static void __exit adp_switch_exit(void)
{
	platform_driver_unregister(&adp_switch_driver);
}
module_exit(adp_switch_exit);

MODULE_DESCRIPTION("Switch only power regulator driver");
MODULE_AUTHOR("Sonic Zhang");
MODULE_LICENSE("GPL");
