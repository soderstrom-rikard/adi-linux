/*
 * Header for ssm2604 sound codec
 *
 * Copyright 2010 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef _SSM2604_H
#define _SSM2604_H

/* SSM2604 Codec Register definitions */

#define SSM2604_LINVOL   0x00
#define SSM2604_RINVOL   0x01
#define SSM2604_APANA    0x04
#define SSM2604_APDIGI   0x05
#define SSM2604_PWR      0x06
#define SSM2604_IFACE    0x07
#define SSM2604_SRATE    0x08
#define SSM2604_ACTIVE   0x09
#define SSM2604_RESET	 0x0f

/* SSM2604 Codec Register Field definitions
 * (Mask value to extract the corresponding Register field)
 */

/* Left ADC Volume Control (SSM2604_REG_LEFT_ADC_VOL) */
#define LINVOL_LIN_VOL          0x01F	/* Left Channel PGA Volume control                    */
#define LINVOL_LIN_ENABLE_MUTE  0x080	/* Left Channel Input Mute                            */
#define LINVOL_LRIN_BOTH        0x100	/* Left Channel Line Input Volume update              */

/* Right ADC Volume Control (SSM2604_REG_RIGHT_ADC_VOL) */
#define RINVOL_RIN_VOL          0x01F	/* Right Channel PGA Volume control                   */
#define RINVOL_RIN_ENABLE_MUTE  0x080	/* Right Channel Input Mute                           */
#define RINVOL_RLIN_BOTH        0x100	/* Right Channel Line Input Volume update             */


/* Analogue Audio Path Control (SSM2604_REG_ANALOGUE_PATH) */
#define APANA_ENABLE_BYPASS     0x008	/* Line input bypass to line output                   */
#define APANA_SELECT_DAC        0x010	/* Select DAC (1=Select DAC, 0=Don't Select DAC)      */

/* Digital Audio Path Control (SSM2604_REG_DIGITAL_PATH) */
#define APDIGI_ENABLE_ADC_HPF   0x001	/* Enable/Disable ADC Highpass Filter                 */
#define APDIGI_DE_EMPHASIS      0x006	/* De-Emphasis Control                                */
#define APDIGI_ENABLE_DAC_MUTE  0x008	/* DAC Mute Control                                   */
#define APDIGI_STORE_OFFSET     0x010	/* Store/Clear DC offset when HPF is disabled         */

/* Power Down Control (SSM2604_REG_POWER)
 * (1=Enable PowerDown, 0=Disable PowerDown)
 */
#define PWR_LINE_IN_PDN         0x001	/* Line Input Power Down                              */
#define PWR_ADC_PDN             0x004	/* ADC Power Down                                     */
#define PWR_DAC_PDN             0x008	/* DAC Power Down                                     */
#define PWR_OSC_PDN             0x020	/* Oscillator Power Down                              */
#define PWR_CLK_OUT_PDN         0x040	/* CLKOUT Power Down                                  */
#define PWR_POWER_OFF           0x080	/* POWEROFF Mode                                      */

/* Digital Audio Interface Format (SSM2604_REG_DIGITAL_IFACE) */
#define IFACE_IFACE_FORMAT      0x003	/* Digital Audio input format control                 */
#define IFACE_AUDIO_DATA_LEN    0x00C	/* Audio Data word length control                     */
#define IFACE_DAC_LR_POLARITY   0x010	/* Polarity Control for clocks in RJ,LJ and I2S modes */
#define IFACE_DAC_LR_SWAP       0x020	/* Swap DAC data control                              */
#define IFACE_ENABLE_MASTER     0x040	/* Enable/Disable Master Mode                         */
#define IFACE_BCLK_INVERT       0x080	/* Bit Clock Inversion control                        */

/* Sampling Control (SSM2604_REG_SAMPLING_CTRL) */
#define SRATE_ENABLE_USB_MODE   0x001	/* Enable/Disable USB Mode                            */
#define SRATE_BOS_RATE          0x002	/* Base Over-Sampling rate                            */
#define SRATE_SAMPLE_RATE       0x03C	/* Clock setting condition (Sampling rate control)    */
#define SRATE_CORECLK_DIV2      0x040	/* Core Clock divider select                          */
#define SRATE_CLKOUT_DIV2       0x080	/* Clock Out divider select                           */

/* Active Control (SSM2604_REG_ACTIVE_CTRL) */
#define ACTIVE_ACTIVATE_CODEC   0x001	/* Activate Codec Digital Audio Interface             */

/*********************************************************************/

#define SSM2604_CACHEREGNUM	9

#define SSM2604_SYSCLK	0
#define SSM2604_DAI		0

#endif
