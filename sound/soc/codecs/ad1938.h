/*
 * File:         sound/soc/codecs/ad1836.h
 * Based on:
 * Author:       Barry Song <Barry.Song@analog.com>
 *
 * Created:      May 25, 2009
 * Description:  definitions for AD1938 registers
 *
 * Modified:
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

#ifndef __AD1938_H__
#define __AD1938_H__

#define AD1938_PLL_CLK_CTRL0    0
#define AD1938_PLL_CLK_CTRL1    1
#define AD1938_DAC_CTRL0        2
#define AD1938_DAC_CTRL1        3
#define AD1938_DAC_CTRL2        4
#define AD1938_DAC_CHNL_MUTE    5
#define AD1938_DAC_L1_VOL       6
#define AD1938_DAC_R1_VOL       7
#define AD1938_DAC_L2_VOL       8
#define AD1938_DAC_R2_VOL       9
#define AD1938_DAC_L3_VOL       10
#define AD1938_DAC_R3_VOL       11
#define AD1938_DAC_L4_VOL       12
#define AD1938_DAC_R4_VOL       13
#define AD1938_ADC_CTRL0        14
#define ADC0_MUTE 		2
#define ADC1_MUTE 		3
#define ADC2_MUTE 		4
#define ADC3_MUTE 		5
#define AD1938_ADC_CTRL1        15
#define AD1938_ADC_CTRL2        16

extern struct snd_soc_dai ad1938_dai;
extern struct snd_soc_codec_device soc_codec_dev_ad1938;
#endif
