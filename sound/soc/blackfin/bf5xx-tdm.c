/*
 * File:         sound/soc/blackfin/bf5xx-tdm.c
 * Author:       Barry Song <Barry.Song@analog.com>
 *
 * Created:      Thurs June 04 2009
 * Description:  Blackfin TDM CPU DAI driver
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <asm/irq.h>
#include <asm/portmux.h>
#include <linux/mutex.h>
#include <linux/gpio.h>

#include "bf5xx-sport.h"
#include "bf5xx-tdm.h"

struct bf5xx_tdm_port {
	u16 tcr1;
	u16 rcr1;
	u16 tcr2;
	u16 rcr2;
	int configured;
};

static struct bf5xx_tdm_port bf5xx_tdm;
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

/*
 * Setting the TFS pin selector for SPORT 0 based on whether the selected
 * port id F or G. If the port is F then no conflict should exist for the
 * TFS. When Port G is selected and EMAC then there is a conflict between
 * the PHY interrupt line and TFS.  Current settings prevent the conflict
 * by ignoring the TFS pin when Port G is selected. This allows both
 * ssm2602 using Port G and EMAC concurrently.
 */
#ifdef CONFIG_BF527_SPORT0_PORTF
#define LOCAL_SPORT0_TFS (P_SPORT0_TFS)
#else
#define LOCAL_SPORT0_TFS (0)
#endif

static u16 sport_req[][7] = { {P_SPORT0_DTPRI, P_SPORT0_TSCLK, P_SPORT0_RFS,
		P_SPORT0_DRPRI, P_SPORT0_RSCLK, LOCAL_SPORT0_TFS, 0},
		{P_SPORT1_DTPRI, P_SPORT1_TSCLK, P_SPORT1_RFS, P_SPORT1_DRPRI,
		P_SPORT1_RSCLK, P_SPORT1_TFS, 0} };

static int bf5xx_tdm_set_dai_fmt(struct snd_soc_dai *cpu_dai,
		unsigned int fmt)
{
	int ret = 0;

	/* interface format:support TDM,slave mode */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_SPORT_TDM:
		break;
	default:
		printk(KERN_ERR "%s: Unknown DAI format type\n", __func__);
		ret = -EINVAL;
		break;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
	case SND_SOC_DAIFMT_CBM_CFS:
	case SND_SOC_DAIFMT_CBS_CFM:
		ret = -EINVAL;
		break;
	default:
		printk(KERN_ERR "%s: Unknown DAI master type\n", __func__);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int bf5xx_tdm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	int ret = 0;

	bf5xx_tdm.tcr2 &= ~0x1f;
	bf5xx_tdm.rcr2 &= ~0x1f;
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S32_LE:
		bf5xx_tdm.tcr2 |= 31;
		bf5xx_tdm.rcr2 |= 31;
		sport_handle->wdsize = 4;
		break;
	}

	if (!bf5xx_tdm.configured) {
		/*
		 * TX and RX are not independent,they are enabled at the
		 * same time, even if only one side is running. So, we
		 * need to configure both of them at the time when the first
		 * stream is opened.
		 *
		 * CPU DAI:slave mode.
		 */
		ret = sport_config_rx(sport_handle, bf5xx_tdm.rcr1,
				      bf5xx_tdm.rcr2, 0, 0);
		if (ret) {
			pr_err("SPORT is busy!\n");
			return -EBUSY;
		}

		ret = sport_config_tx(sport_handle, bf5xx_tdm.tcr1,
				      bf5xx_tdm.tcr2, 0, 0);
		if (ret) {
			pr_err("SPORT is busy!\n");
			return -EBUSY;
		}
		bf5xx_tdm.configured = 1;
	}

	return 0;
}

static int bf5xx_tdm_probe(struct platform_device *pdev,
		struct snd_soc_dai *dai)
{
	int ret = 0;

	if (peripheral_request_list(&sport_req[sport_num][0], "soc-audio")) {
		pr_err("Requesting Peripherals failed\n");
		return -EFAULT;
	}

	/* request DMA for SPORT */
	sport_handle = sport_init(&sport_params[sport_num], 4, \
			8 * sizeof(u32), NULL);
	if (!sport_handle) {
		peripheral_free_list(&sport_req[sport_num][0]);
		return -ENODEV;
	}

	/*SPORT works in TDM mode to simulate AC97 transfers*/
	ret = sport_set_multichannel(sport_handle, 8, 0xFF, 1);
	if (ret) {
		pr_err("SPORT is busy!\n");
		ret = -EBUSY;
		goto sport_config_err;
	}

	ret = sport_config_rx(sport_handle, IRFS, 0x1F, 0, 0);
	if (ret) {
		pr_err("SPORT is busy!\n");
		ret = -EBUSY;
		goto sport_config_err;
	}

	ret = sport_config_tx(sport_handle, ITFS, 0x1F, 0, 0);
	if (ret) {
		pr_err("SPORT is busy!\n");
		ret = -EBUSY;
		goto sport_config_err;
	}

sport_config_err:
	peripheral_free_list(&sport_req[sport_num][0]);
	return ret;
}

static void bf5xx_tdm_remove(struct platform_device *pdev,
		struct snd_soc_dai *dai)
{
	peripheral_free_list(&sport_req[sport_num][0]);
}

#ifdef CONFIG_PM
static int bf5xx_tdm_suspend(struct platform_device *dev,
		struct snd_soc_dai *dai)
{
	struct sport_device *sport =
		(struct sport_device *)dai->private_data;

	if (!dai->active)
		return 0;
	if (dai->capture.active)
		sport_rx_stop(sport);
	if (dai->playback.active)
		sport_tx_stop(sport);
	return 0;
}

static int bf5xx_tdm_resume(struct platform_device *pdev,
		struct snd_soc_dai *dai)
{
	struct sport_device *sport =
		(struct sport_device *)dai->private_data;

	if (!dai->active)
		return 0;

	if (dai->capture.active)
		sport_rx_start(sport);
	if (dai->playback.active)
		sport_tx_start(sport);
	return 0;
}

#else
#define bf5xx_tdm_suspend	NULL
#define bf5xx_tdm_resume	NULL
#endif

struct snd_soc_dai bf5xx_tdm_dai = {
	.name = "bf5xx-tdm",
	.id = 0,
	.type = SND_SOC_DAI_PCM,
	.probe = bf5xx_tdm_probe,
	.remove = bf5xx_tdm_remove,
	.suspend = bf5xx_tdm_suspend,
	.resume = bf5xx_tdm_resume,
	.playback = {
		.channels_min = 2,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S32_LE,},
	.capture = {
		.channels_min = 2,
		.channels_max = 8,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S32_LE,},
	.ops = {
		.hw_params = bf5xx_tdm_hw_params,},
	.dai_ops = {
		.set_fmt = bf5xx_tdm_set_dai_fmt,
	},
};
EXPORT_SYMBOL_GPL(bf5xx_tdm_dai);

/* Module information */
MODULE_AUTHOR("Barry Song");
MODULE_DESCRIPTION("TDM driver for ADI Blackfin");
MODULE_LICENSE("GPL");

