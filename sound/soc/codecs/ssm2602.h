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

/* SSM2602 Codec Register definitions */

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
#define SSM2602_RESET	 0x0f

/*SSM2602 Codec Register Field definitions
 *(Mask value to extract the corresponding Register field)
 */

/*Left ADC Volume Control (SSM2602_REG_LEFT_ADC_VOL)*/
#define     LIN_VOL                0x01F   /* Left Channel PGA Volume control                      */
#define     LIN_ENABLE_MUTE        0x080   /* Left Channel Input Mute                              */
#define     LRIN_BOTH              0x100   /* Left Channel Line Input Volume update                */

/*Right ADC Volume Control (SSM2602_REG_RIGHT_ADC_VOL)*/
#define     RIN_VOL                0x01F   /* Right Channel PGA Volume control                     */
#define     RIN_ENABLE_MUTE        0x080   /* Right Channel Input Mute                             */
#define     RLIN_BOTH              0x100   /* Right Channel Line Input Volume update               */

/*Left DAC Volume Control (SSM2602_REG_LEFT_DAC_VOL)*/
#define     LHP_VOL                0x07F   /* Left Channel Headphone volume control                */
#define     ENABLE_LZC             0x080   /* Left Channel Zero cross detect enable                */
#define     LRHP_BOTH              0x100   /* Left Channel Headphone volume update                 */

/*Right DAC Volume Control (SSM2602_REG_RIGHT_DAC_VOL)*/
#define     RHP_VOL                0x07F   /* Right Channel Headphone volume control               */
#define     ENABLE_RZC             0x080   /* Right Channel Zero cross detect enable               */
#define     RLHP_BOTH              0x100   /* Right Channel Headphone volume update                */

/*Analogue Audio Path Control (SSM2602_REG_ANALOGUE_PATH)*/
#define     ENABLE_MIC_BOOST       0x001   /* Primary Microphone Amplifier gain booster control    */
#define     ENABLE_MIC_MUTE        0x002   /* Microphone Mute Control                              */
#define     ADC_IN_SELECT          0x004   /* Microphone/Line IN select to ADC (1=MIC, 0=Line In)  */
#define     ENABLE_BYPASS          0x008   /* Line input bypass to line output                     */
#define     SELECT_DAC             0x010   /* Select DAC (1=Select DAC, 0=Don't Select DAC)        */
#define     ENABLE_SIDETONE        0x020   /* Enable/Disable Side Tone                             */
#define     SIDETONE_ATTN          0x0C0   /* Side Tone Attenuation                                */
#define     ENABLE_MIC_BOOST2      0x100   /* Secondary Microphone Amplifier gain booster control  */

/*Digital Audio Path Control (SSM2602_REG_DIGITAL_PATH)*/
#define     ENABLE_ADC_HPF         0x001   /* Enable/Disable ADC Highpass Filter                   */
#define     DE_EMPHASIS            0x006   /* De-Emphasis Control                                  */
#define     ENABLE_DAC_MUTE        0x008   /* DAC Mute Control                                     */
#define     STORE_OFFSET           0x010   /* Store/Clear DC offset when HPF is disabled           */

/*Power Down Control (SSM2602_REG_POWER)
 *(1=Enable PowerDown, 0=Disable PowerDown)
 */
#define     LINE_IN_PDN            0x001   /* Line Input Power Down                                */
#define     MIC_PDN                0x002   /* Microphone Input & Bias Power Down                   */
#define     ADC_PDN                0x004   /* ADC Power Down                                       */
#define     DAC_PDN                0x008   /* DAC Power Down                                       */
#define     OUT_PDN                0x010   /* Outputs Power Down                                   */
#define     OSC_PDN                0x020   /* Oscillator Power Down                                */
#define     CLK_OUT_PDN            0x040   /* CLKOUT Power Down                                    */
#define     POWER_OFF              0x080   /* POWEROFF Mode                                        */

/*Digital Audio Interface Format (SSM2602_REG_DIGITAL_IFACE)*/
#define     IFACE_FORMAT           0x003   /* Digital Audio input format control                   */
#define     AUDIO_DATA_LEN         0x00C   /* Audio Data word length control                       */
#define     DAC_LR_POLARITY        0x010   /* Polarity Control for clocks in RJ,LJ and I2S modes   */
#define     DAC_LR_SWAP            0x020   /* Swap DAC data control                                */
#define     ENABLE_MASTER          0x040   /* Enable/Disable Master Mode                           */
#define     BCLK_INVERT            0x080   /* Bit Clock Inversion control                          */

/*Sampling Control (SSM2602_REG_SAMPLING_CTRL)*/
#define     ENABLE_USB_MODE        0x001   /* Enable/Disable USB Mode                              */
#define     BOS_RATE               0x002   /* Base Over-Sampling rate                              */
#define     SAMPLE_RATE            0x03C   /* Clock setting condition (Sampling rate control)      */
#define     CORECLK_DIV2           0x040   /* Core Clock divider select                            */
#define     CLKOUT_DIV2            0x080   /* Clock Out divider select                             */

/*Active Control (SSM2602_REG_ACTIVE_CTRL)*/
#define     ACTIVATE_CODEC         0x001   /* Activate Codec Digital Audio Interface               */

/*********************************************************************/

#define SSM2602_CACHEREGNUM 	10

#define SSM2602_SYSCLK	0
#define SSM2602_DAI		0

struct ssm2602_setup_data {
	unsigned short i2c_address;
};

extern struct snd_soc_codec_dai ssm2602_dai;
extern struct snd_soc_codec_device soc_codec_dev_ssm2602;

#endif
