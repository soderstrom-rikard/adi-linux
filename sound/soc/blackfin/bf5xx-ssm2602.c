/*
 * File:         sound/soc/blackfin/bf5xx-ssm2602.c
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

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm_params.h>

#include <asm/dma.h>
#include <asm/gpio.h>
#include <asm/portmux.h>
#include "../codecs/ssm2602.h"
#include "bf5xx-sport.h"
#include "bf5xx-pcm.h"
#include "bf5xx-i2s.h"

#ifdef BF53X_SSM2602_DEBUG
#define printd(format, arg...) printk(KERN_INFO"bfin-SSM2602: " format, ## arg)
#else
#define printd(format, arg...)
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

static struct snd_soc_machine bf5xx_ssm2602;

static int bf5xx_ssm2602_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_cpu_dai *cpu_dai = rtd->dai->cpu_dai;

	printd("%s\n", __func__);
	cpu_dai->private_data = sport_handle;
	return 0;
}

static int bf5xx_ssm2602_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec_dai *codec_dai = rtd->dai->codec_dai;
	unsigned int clk = 0;
	int ret = 0;

	printd("%s rate %d format %x\n", __func__, params_rate(params),
		params_format(params));
	/*
	 * WARNING - TODO
	 *
	 * This code assumes there is a variable clocksource for the SSM2602.
	 * i.e. it supplies MCLK depending on rate.
	 *
	 * If you are using a crystal source then modify the below case
	 * statement with a static frequency.
	 *
	 * If you are using the SPORT to generate clocking then this is
	 * where to do it.
	 */

	switch (params_rate(params)) {
	case 8000:
	case 16000:
	case 48000:
	case 96000:
	case 11025:
	case 22050:
	case 44100:
		clk = 12000000;
		break;
	}

	/*
	 * CODEC is master for BCLK and LRC in this configuration.
	 */

	/* set codec DAI configuration */
	ret = codec_dai->dai_ops.set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	ret = codec_dai->dai_ops.set_sysclk(codec_dai, SSM2602_SYSCLK, clk,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops bf5xx_ssm2602_ops = {
	.startup = bf5xx_ssm2602_startup,
	.hw_params = bf5xx_ssm2602_hw_params,
};

static int bf5xx_ssm2602_init_dev(struct snd_soc_codec *codec)
{
	/*
	 * NC codec pins -
	 * Add any NC codec pins as follows :-
	 *
	 * snd_soc_dapm_set_endpoint(codec, "RINPUT1", 0);
	 */
	printd("%s\n", __func__);

	snd_soc_dapm_sync_endpoints(codec);
	return 0;
}

static struct snd_soc_dai_link bf5xx_ssm2602_dai = {
	.name = "ssm2602",
	.stream_name = "SSM2602",
	.cpu_dai = &bf5xx_i2s_dai,
	.codec_dai = &ssm2602_dai,
	.init = bf5xx_ssm2602_init_dev,
	.ops = &bf5xx_ssm2602_ops,
};

static int bf5xx_probe(struct platform_device *pdev)
{

	u16 sport_req[][7] = { {P_SPORT0_DTPRI, P_SPORT0_TSCLK, P_SPORT0_RFS,
		 P_SPORT0_DRPRI, P_SPORT0_RSCLK, 0}, {P_SPORT1_DTPRI,
		 P_SPORT1_TSCLK, P_SPORT1_RFS, P_SPORT1_DRPRI, P_SPORT1_RSCLK, 0} };
	if (peripheral_request_list(&sport_req[sport_num][0], "soc-audio")) {
		printk(KERN_ERR "Requesting Peripherals failed\n");
		return -EFAULT;
	}

	sport_handle = sport_init(&sport_params[sport_num], 4, \
			10 * sizeof(u16), NULL);
	if (!sport_handle) {
		peripheral_free_list(&sport_req[sport_num][0]);
		return -ENODEV;
	}
	return 0;
}

/*
 * SSM2602 2 wire address is determined by CSB
 * state during powerup.
 *    low  = 0x1a
 *    high = 0x1b
 */

static struct ssm2602_setup_data bf5xx_ssm2602_setup = {
	.i2c_address = 0x1b,
};

static struct snd_soc_machine bf5xx_ssm2602 = {
	.name = "bf5xx_ssm2602",
	.probe = bf5xx_probe,
	.dai_link = &bf5xx_ssm2602_dai,
	.num_links = 1,
};

static struct snd_soc_device bf5xx_ssm2602_snd_devdata = {
	.machine = &bf5xx_ssm2602,
	.platform = &bf5xx_soc_platform,
	.codec_dev = &soc_codec_dev_ssm2602,
	.codec_data = &bf5xx_ssm2602_setup,
};

static struct platform_device *bf52x_ssm2602_snd_device;

static int __init bf5xx_ssm2602_init(void)
{
	int ret;

	printd("%s\n", __func__);
	bf52x_ssm2602_snd_device = platform_device_alloc("soc-audio", -1);
	if (!bf52x_ssm2602_snd_device)
		return -ENOMEM;

	platform_set_drvdata(bf52x_ssm2602_snd_device, &bf5xx_ssm2602_snd_devdata);
	bf5xx_ssm2602_snd_devdata.dev = &bf52x_ssm2602_snd_device->dev;
	ret = platform_device_add(bf52x_ssm2602_snd_device);

	if (ret)
		platform_device_put(bf52x_ssm2602_snd_device);

	return ret;
}

static void __exit bf5xx_ssm2602_exit(void)
{
	printd("%s\n", __func__);
	platform_device_unregister(bf52x_ssm2602_snd_device);
}

module_init(bf5xx_ssm2602_init);
module_exit(bf5xx_ssm2602_exit);

/* Module information */
MODULE_AUTHOR("Cliff Cai");
MODULE_DESCRIPTION("ALSA SoC BF527-EZKIT");
MODULE_LICENSE("GPL");

