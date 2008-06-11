/*
 * File:         sound/soc/codecs/ssm2602.h
 * Author:       Cliff Cai <Cliff.Cai@analog.com>
 *
 * Created:      Tue June 06 2008
 * Description:  Driver for SSM2602 sound chip built in ADSP-BF52xC
 *
 * Rev:          $Id: ssm2602.c 4104 2008-06-06 06:51:48Z cliff $
 *
 * Modified:
 *               Copyright 2008 Analog Devices Inc.
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

#ifndef _SSM2602_H
#define _SSM2602_H

/* SSM2602 register space */

#define SSM2602_LINVOL   0x00
#define SSM2602_RINVOL   0x01
#define SSM2602_LOUT1V   0x02
#define SSM2602_ROUT1V   0x03
#define SSM2602_APANA    0x04
#define SSM2602_APDIGI   0x05
#define SSM2602_PWR      0x06
#define SSM2602_IFACE    0x07
#define SSM2602_SRATE    0x08
#define SSM2602_ACTIVE   0x09
#define SSM2602_RESET	0x0f

#define SSM2602_CACHEREGNUM 	10

#define SSM2602_SYSCLK	0
#define SSM2602_DAI		0

struct ssm2602_setup_data {
	unsigned short i2c_address;
};

extern struct snd_soc_codec_dai ssm2602_dai;
extern struct snd_soc_codec_device soc_codec_dev_ssm2602;

#endif
