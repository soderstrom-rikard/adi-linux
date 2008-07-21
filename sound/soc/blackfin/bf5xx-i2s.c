/*
 * bf5xx-i2s.c  --  ALSA Soc Audio Layer
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 * modified by:Cliff Cai
 *		cliff.cai@analog.com
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    27th Aug 2007   Initial version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <asm/irq.h>
#include <asm/gpio.h>
#include <asm/portmux.h>
#include <linux/mutex.h>

#include "bf5xx-sport.h"
#include "bf5xx-i2s.h"

#ifdef BF53X_I2S_DEBUG
#define i2s_printd(format, arg...) printk(KERN_INFO"sport-i2s: " format, ## arg)
#else
#define i2s_printd(format, arg...)
#endif

#ifndef SLEN_T
#define                    SLEN_T  0x1f       /* SPORT Word Length */
#endif

#ifndef SLEN_R
#define                    SLEN_R  0x1f       /* SPORT Word Length */
#endif

static int sport_num = CONFIG_SND_BF5XX_SPORT_NUM;

static struct sport_param sport_params[2] = {
	{
		.dma_rx_chan	= CH_SPORT0_RX,
		.dma_tx_chan	= CH_SPORT0_TX,
		.err_irq	= IRQ_SPORT0_ERROR,
		.regs		= (struct sport_register *)SPORT0_TCR1,
	},
	{
		.dma_rx_chan	= CH_SPORT1_RX,
		.dma_tx_chan	= CH_SPORT1_TX,
		.err_irq	= IRQ_SPORT1_ERROR,
		.regs		= (struct sport_register *)SPORT1_TCR1,
	}
};

struct sport_device *sport_handle;
EXPORT_SYMBOL(sport_handle);
/*
 * This is setup by the audio & dai ops and written to sport during prepare ()
 */
struct bf5xx_i2s_port {
	u16 tcr1;
	u16 rcr1;
	u16 tcr2;
	u16 rcr2;

	/* todo - sport master mode */
	unsigned int tclkdiv;
	unsigned int tfsdiv;
	unsigned int rclkdiv;
	unsigned int rfsdiv;
};

void bf5xx_pcm_to_frame(struct audio_frame *dst, const __u32 *src, \
		size_t count)
{
	__u32 *idst = (__u32 *)dst;
	count = count * 8;
	memcpy(idst, src, count);
}
EXPORT_SYMBOL(bf5xx_pcm_to_frame);

void bf5xx_frame_to_pcm(const struct audio_frame *src, __u32 *dst, \
		size_t count)
{
	__u32 *isrc = (__u32 *)src;
	count = count * 8;
	memcpy(dst, isrc, count);
}
EXPORT_SYMBOL(bf5xx_frame_to_pcm);

static int bf5xx_i2s_probe(struct platform_device *pdev)
{
	u16 sport_req[][7] = { {P_SPORT0_DTPRI, P_SPORT0_TSCLK, P_SPORT0_RFS,
		 P_SPORT0_DRPRI, P_SPORT0_RSCLK, 0}, {P_SPORT1_DTPRI,
		 P_SPORT1_TSCLK, P_SPORT1_RFS, P_SPORT1_DRPRI, P_SPORT1_RSCLK, 0} };
	if (peripheral_request_list(&sport_req[sport_num][0], "soc-audio")) {
		printk(KERN_ERR "Requesting Peripherals failed\n");
		return -EFAULT;
	}

	sport_handle = sport_init(&sport_params[sport_num], 4, \
			4 * sizeof(u32), NULL);
	if (!sport_handle) {
		peripheral_free_list(&sport_req[sport_num][0]);
		return -ENODEV;
	}
	/* We need to configure both RX and TX here,they are not totally independent.
	*  Currently,TX and RX are enabled at the same time,even if only one side is running.
	*  DAI format:I2S,word length:32 bit,slave mode
	*/
	sport_config_rx(sport_handle, RFSR | RCKFE, RSFSE|0x1f, 0, 0);
	sport_config_tx(sport_handle, TFSR | TCKFE, TSFSE|0x1f, 0, 0);

	return 0;
}

#ifdef CONFIG_PM
static int bf5xx_i2s_suspend(struct platform_device *dev,
	struct snd_soc_cpu_dai *dai)
{
	struct sport_device *sport =
		(struct sport_device *)dai->private_data;

	i2s_printd("%s : sport %d\n", __func__, dai->id);
	if (!dai->active)
		return 0;
	if (dai->capture.active)
		sport_rx_stop(sport);
	if (dai->playback.active)
		sport_tx_stop(sport);
	return 0;
}

static int bf5xx_i2s_resume(struct platform_device *pdev,
	struct snd_soc_cpu_dai *dai)
{
	struct sport_device *sport =
		(struct sport_device *)dai->private_data;

	i2s_printd("%s : sport %d\n", __func__, cpu_dai->id);
	if (!dai->active)
		return 0;
	sport_config_rx(sport_handle, RFSR | RCKFE, RSFSE|0x1f, 0, 0);
	sport_config_tx(sport_handle, TFSR | TCKFE, TSFSE|0x1f, 0, 0);
	if (dai->capture.active)
		sport_rx_start(sport);
	if (dai->playback.active)
		sport_tx_start(sport);
	return 0;
}

#else
#define bf5xx_i2s_suspend	NULL
#define bf5xx_i2s_resume	NULL
#endif

#define BF5XX_I2S_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
		SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 | \
		SNDRV_PCM_RATE_96000)

struct snd_soc_cpu_dai bf5xx_i2s_dai = {
	.name = "bf5xx-i2s-0",
	.id = 0,
	.type = SND_SOC_DAI_I2S,
	.probe = bf5xx_i2s_probe,
	.suspend = bf5xx_i2s_suspend,
	.resume = bf5xx_i2s_resume,
	.playback = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = BF5XX_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S32_LE,},
	.capture = {
		.channels_min = 2,
		.channels_max = 2,
		.rates = BF5XX_I2S_RATES,
		.formats = SNDRV_PCM_FMTBIT_S32_LE,},
};
EXPORT_SYMBOL_GPL(bf5xx_i2s_dai);

/* Module information */
MODULE_AUTHOR("Liam Girdwood, liam.girdwood@wolfsonmicro.com, www.wolfsonmicro.com");
MODULE_DESCRIPTION("Blackfin I2S SoC Interface");
MODULE_LICENSE("GPL");
