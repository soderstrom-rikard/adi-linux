/*
 * File:         sound/blackfin/ad1938_spi.h
 * Based on:
 * Author:       Barry Song
 *
 * Created:      2009-05-22
 * Description:  ad1938 spi driver.
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

#ifndef __AD1938_SPI_H__
#define __AD1938_SPI_H__


int ad1938_spi_init(void);

void ad1938_spi_done(void);

int ad1938_spi_read(uint8_t reg, uint8_t *val);

int ad1938_spi_write(uint8_t reg, uint8_t val);

#endif
