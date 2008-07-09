/*
 * File:         sound/soc/blackfin/bf5xx-i2s.c
 * Author:       Cliff Cai <Cliff.Cai@analog.com>
 *
 * Created:      Tue June 06 2008
 * Description:  Driver for SSM2602 sound chip built in ADSP-BF52xC
 *
 * Rev:          $Id: bf5xx-i2s.c 4104 2008-06-06 06:51:48Z cliff $
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

static int bf5xx_i2s_probe(struct platform_device *pdev)
{
	int ret;
	u16 sport_req[][7] = { {P_SPORT0_DTPRI, P_SPORT0_TSCLK, P_SPORT0_RFS,
		 P_SPORT0_DRPRI, P_SPORT0_RSCLK, 0}, {P_SPORT1_DTPRI,
		 P_SPORT1_TSCLK, P_SPORT1_RFS, P_SPORT1_DRPRI, P_SPORT1_RSCLK, 0} };
	if (peripheral_request_list(&sport_req[sport_num][0], "soc-audio")) {
		printk(KERN_ERR "Requesting Peripherals failed\n");
		return -EFAULT;
	}
	pr_debug("%s enter\n", __FUNCTION__);
	/*request DMA for SPORT*/
	sport_handle = sport_init(&sport_params[sport_num], 4, \
			10 * sizeof(u16), NULL);
	if (!sport_handle) {
		peripheral_free_list(&sport_req[sport_num][0]);
		return -ENODEV;
	}
	/*  TX and RX are not independent,they are enabled at the same time,
	 *  even if only one side is running.So,we need to configure both of them in advance.
	 *  CPU DAI format:I2S,word length:32 bit,slave mode.
	 */
	ret = sport_config_rx(sport_handle, RFSR | RCKFE, RSFSE|0x1f, 0, 0);
	if (ret) {
		printk(KERN_ERR "SPORT is busy!\n");
		return -EBUSY;
	}
	ret = sport_config_tx(sport_handle, TFSR | TCKFE, TSFSE|0x1f, 0, 0);
	if (ret) {
		printk(KERN_ERR "SPORT is busy!\n");
		return -EBUSY;
	}

	return 0;
}

#ifdef CONFIG_PM
static int bf5xx_i2s_suspend(struct platform_device *dev,
	struct snd_soc_cpu_dai *dai)
{
	struct sport_device *sport =
		(struct sport_device *)dai->private_data;

	pr_debug("%s : sport %d\n", __FUNCTION__, dai->id);
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
	int ret;
	struct sport_device *sport =
		(struct sport_device *)dai->private_data;

	pr_debug("%s : sport %d\n", __FUNCTION__, cpu_dai->id);
	if (!dai->active)
		return 0;
	ret = sport_config_rx(sport_handle, RFSR | RCKFE, RSFSE|0x1f, 0, 0);

	if (ret) {
		printk(KERN_ERR "SPORT is busy!\n");
		return -EBUSY;
	}
	ret = sport_config_tx(sport_handle, TFSR | TCKFE, TSFSE|0x1f, 0, 0);
	if (ret) {
		printk(KERN_ERR "SPORT is busy!\n");
		return -EBUSY;
	}

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
	.name = "bf5xx-i2s",
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
MODULE_AUTHOR("Cliff Cai");
MODULE_DESCRIPTION("I2S driver for ADI Blackfin");
MODULE_LICENSE("GPL");

