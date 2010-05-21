/*
 * ad5398.c  --  Voltage and current regulation for AD5398 and AD5821
 *
 * Copyright 2010 Analog Devices Inc.
 *
 * Enter bugs at http://blackfin.uclinux.org/
 *
 * Licensed under the GPL-2 or later.
 */
#include <linux/module.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/ad5398.h>

struct ad5398_chip_info {
	struct i2c_client *client;
	unsigned int min_uA;
	unsigned int max_uA;
	unsigned int current_level;
	unsigned int current_mask;
	unsigned int current_offset;
	struct regulator_dev rdev;
};

static int ad5398_calc_current(struct ad5398_chip_info *chip,
	unsigned selector)
{
	unsigned range_uA = chip->max_uA - chip->min_uA;

	return chip->min_uA + (selector * range_uA / chip->current_level);
}

static int ad5398_get(struct regulator_dev *rdev)
{
	struct ad5398_chip_info *chip = rdev_get_drvdata(rdev);
	struct i2c_client *client = chip->client;
	unsigned short data;
	int ret;

	ret = i2c_master_recv(client, (char *)&data, 2);
	if (ret < 0) {
		dev_err(&client->dev, "I2C read error\n");
		return ret;
	}

	ret = (be16_to_cpu(data) & chip->current_mask) >> chip->current_offset;

	return ad5398_calc_current(chip, ret);
}

static int ad5398_set(struct regulator_dev *rdev, int min_uA, int max_uA)
{
	struct ad5398_chip_info *chip = rdev_get_drvdata(rdev);
	struct i2c_client *client = chip->client;
	unsigned range_uA = chip->max_uA - chip->min_uA;
	unsigned selector;
	unsigned short data;
	int ret;

	if (min_uA > chip->max_uA || max_uA < chip->min_uA)
		return -EINVAL;
	if (min_uA < chip->min_uA)
		min_uA = chip->min_uA;

	selector = ((min_uA - chip->min_uA) * chip->current_level +
			range_uA - 1) / range_uA;
	if (ad5398_calc_current(chip, selector) > max_uA)
		return -EINVAL;

	dev_dbg(&client->dev, "changing current %dmA\n",
		ad5398_calc_current(chip, selector) / 1000);

	/* read chip enable bit */
	ret = i2c_master_recv(client, (char *)&data, 2);
	if (ret < 0) {
		dev_err(&client->dev, "I2C read error\n");
		return ret;
	}
	data = be16_to_cpu(data);

	/* prepare register data */
	selector = (selector << chip->current_offset) & chip->current_mask;
	selector |= (data & CURRENT_EN_MASK);

	/* write the new current value back as well as enable bit */
	data = cpu_to_be16((unsigned short)selector);
	ret = i2c_master_send(client, (char *)&data, 2);
	if (ret < 0)
		dev_err(&client->dev, "I2C write error\n");

	return ret;
}

static int ad5398_is_enabled(struct regulator_dev *rdev)
{
	struct ad5398_chip_info *chip = rdev_get_drvdata(rdev);
	struct i2c_client *client = chip->client;
	unsigned short data;
	int ret;

	ret = i2c_master_recv(client, (char *)&data, 2);
	if (ret < 0) {
		dev_err(&client->dev, "I2C read error\n");
		return ret;
	}

	if (be16_to_cpu(data) & CURRENT_EN_MASK)
		return 1;
	else
		return 0;
}

static int ad5398_enable(struct regulator_dev *rdev)
{
	struct ad5398_chip_info *chip = rdev_get_drvdata(rdev);
	struct i2c_client *client = chip->client;
	unsigned short data;
	int ret;

	ret = i2c_master_recv(client, (char *)&data, 2);
	if (ret < 0) {
		dev_err(&client->dev, "I2C read error\n");
		return ret;
	}
	data = be16_to_cpu(data);
	if (data & CURRENT_EN_MASK)
		return 0;

	data = cpu_to_be16(data | CURRENT_EN_MASK);

	ret = i2c_master_send(client, (char *)&data, 2);
	if (ret < 0)
		dev_err(&client->dev, "I2C write error\n");

	return ret;
}

static int ad5398_disable(struct regulator_dev *rdev)
{
	struct ad5398_chip_info *chip = rdev_get_drvdata(rdev);
	struct i2c_client *client = chip->client;
	unsigned short data;
	int ret;

	ret = i2c_master_recv(client, (char *)&data, 2);
	if (ret < 0) {
		dev_err(&client->dev, "I2C read error\n");
		return ret;
	}
	if (!(data & CURRENT_EN_MASK))
		return 0;

	data = cpu_to_be16(data & ~CURRENT_EN_MASK);

	ret = i2c_master_send(client, (char *)&data, 2);
	if (ret < 0)
		dev_err(&client->dev, "I2C write error\n");

	return ret;
}

static struct regulator_ops ad5398_ops = {
	.get_current_limit = ad5398_get,
	.set_current_limit = ad5398_set,
	.enable = ad5398_enable,
	.disable = ad5398_disable,
	.is_enabled = ad5398_is_enabled,
	.set_suspend_enable = ad5398_enable,
	.set_suspend_disable = ad5398_disable,
};

static struct regulator_desc ad5398_reg = {
		.name = "isink",
		.id = 0,
		.ops = &ad5398_ops,
		.type = REGULATOR_CURRENT,
		.owner = THIS_MODULE,
};

static int ad5398_probe(struct i2c_client *client,
		      const struct i2c_device_id *id)
{
	struct regulator_dev *rdev;
	struct ad5398_platform_data *pdata = client->dev.platform_data;
	struct ad5398_chip_info *chip;
	int ret;

	if (!pdata || !(pdata->regulator_data))
		return -EINVAL;

	if (pdata->current_bits >= CURRENT_BITS_MAX)
		return -EINVAL;

	chip = kzalloc(sizeof(struct ad5398_chip_info), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;

	chip->min_uA = pdata->regulator_data->constraints.min_uA;
	chip->max_uA = pdata->regulator_data->constraints.max_uA;
	chip->current_level = 1 << pdata->current_bits;
	chip->current_offset = pdata->current_offset;
	chip->current_mask = (chip->current_level - 1) << chip->current_offset;

	rdev = regulator_register(&ad5398_reg, &client->dev,
			 pdata->regulator_data, chip);
	if (IS_ERR(rdev)) {
		ret = PTR_ERR(rdev);
		dev_err(&client->dev, "failed to register %s %s\n",
			id->name, ad5398_reg.name);
		goto err;
	}

	i2c_set_clientdata(client, chip);
	dev_info(&client->dev, "%s regulator driver loaded\n", id->name);
	return 0;

err:
	kfree(chip);
	return ret;
}

static int ad5398_remove(struct i2c_client *client)
{
	struct ad5398_chip_info *chip = i2c_get_clientdata(client);

	if (chip) {
		regulator_unregister(&chip->rdev);
		kfree(chip);
	}
	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id ad5398_id[] = {
	{ "ad5398", 0 },
	{ "ad5821", 1 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ad5398_id);

static struct i2c_driver ad5398_driver = {
	.probe = ad5398_probe,
	.remove = ad5398_remove,
	.driver		= {
		.name	= "ad5398",
	},
	.id_table	= ad5398_id,
};

static int __init ad5398_init(void)
{
	return i2c_add_driver(&ad5398_driver);
}
module_init(ad5398_init);

static void __exit ad5398_exit(void)
{
	i2c_del_driver(&ad5398_driver);
}
module_exit(ad5398_exit);

MODULE_DESCRIPTION("AD5398 and AD5821 current regulator driver");
MODULE_AUTHOR("Sonic Zhang");
MODULE_LICENSE("GPL");
