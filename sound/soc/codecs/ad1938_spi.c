/*
 * File:         sound/blackfin/ad1938_spi.c
 * Based on:
 * Author:       Barry Song
 *
 * Created:
 * Description:  AD1938 SPI driver
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

#include "ad1938_spi.h"

#define SPI_ADDR 0x4
#define SPI_READ 0x1

struct spi_device *ad1938_spi_dev;

int ad1938_spi_read(uint8_t reg, uint8_t *val)
{
	uint8_t w_buf[3], r_buf[3];
	int ret;

	struct spi_transfer t = {
		.tx_buf = w_buf,
		.rx_buf = r_buf,
		.len = 3,
	};
	struct spi_message m;

	w_buf[0] = (SPI_ADDR << 1) | SPI_READ;
	w_buf[1] = reg;
	w_buf[2] = 0;
	r_buf[0] = r_buf[1] = r_buf[2] = 0;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	ret = spi_sync(ad1938_spi_dev, &m);

	if (ret == 0)
		*val = r_buf[2];

	return ret;
}


int ad1938_spi_write(uint8_t reg, uint8_t val)
{
	uint8_t buf[3];

	struct spi_transfer t = {
		.tx_buf = buf,
		.len = 3,
	};
	struct spi_message m;

	buf[0] = SPI_ADDR << 1;
	buf[1] = reg;
	buf[2] = val;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	return spi_sync(ad1938_spi_dev, &m);
}

int snd_ad1938_spi_probed(void)
{
	return 0;
}

static int __devinit ad1938_spi_probe(struct spi_device *spi)
{
	spi->dev.power.power_state = PMSG_ON;
	ad1938_spi_dev = spi;

	return snd_ad1938_spi_probed();
}

static int __devexit ad1938_spi_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver ad1938_spi_driver = {
	.driver = {
		.name	= "ad1938-spi",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= ad1938_spi_probe,
	.remove		= __devexit_p(ad1938_spi_remove),
};

int ad1938_spi_init(void)
{
	return spi_register_driver(&ad1938_spi_driver);
}

void ad1938_spi_done(void)
{
	spi_unregister_driver(&ad1938_spi_driver);
}
