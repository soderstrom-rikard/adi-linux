/*
 * AD193X Audio Codec SPI bus driver
 *
 * Copyright 2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include "ad193x.h"

static int __devinit ad193x_spi_probe(struct spi_device *spi)
{
	return ad193x_bus_probe(&spi->dev, spi);
}

static int __devexit ad193x_spi_remove(struct spi_device *spi)
{
	return ad193x_bus_remove(&spi->dev);
}

static struct spi_driver ad193x_spi_driver = {
	.driver = {
		.name	= "ad193x",
		.owner	= THIS_MODULE,
	},
	.probe		= ad193x_spi_probe,
	.remove		= __devexit_p(ad193x_spi_remove),
};

static int __init ad193x_spi_init(void)
{
	return spi_register_driver(&ad193x_spi_driver);
}
module_init(ad193x_spi_init);

static void __exit ad193x_spi_exit(void)
{
	spi_unregister_driver(&ad193x_spi_driver);
}
module_exit(ad193x_spi_exit);

MODULE_DESCRIPTION("ASoC ad193x spi driver");
MODULE_AUTHOR("Barry Song <21cnbao@gmail.com>");
MODULE_LICENSE("GPL");
