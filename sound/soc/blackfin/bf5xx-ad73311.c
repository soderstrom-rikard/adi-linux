/*
 * File:         sound/soc/blackfin/bf5xx-ad73311.c
 * Author:       Cliff Cai <Cliff.Cai@analog.com>
 *
 * Created:      Thur Sep 25 2008
 * Description:  Board driver for ad73311 sound chip
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
#include <linux/delay.h>
#include <linux/gpio.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#include <asm/blackfin.h>
#include <asm/cacheflush.h>
#include <asm/irq.h>
#include <asm/dma.h>
#include <asm/portmux.h>

#include "../codecs/ad73311.h"
#include "bf5xx-sport.h"
#include "bf5xx-i2s-pcm.h"

#if CONFIG_SND_BF5XX_SPORT_NUM == 0
#define bfin_write_SPORT_TCR1	bfin_write_SPORT0_TCR1
#define bfin_read_SPORT_TCR1	bfin_read_SPORT0_TCR1
#define bfin_write_SPORT_TCR2	bfin_write_SPORT0_TCR2
#define bfin_write_SPORT_TX16	bfin_write_SPORT0_TX16
#define bfin_read_SPORT_STAT	bfin_read_SPORT0_STAT
#else
#define bfin_write_SPORT_TCR1	bfin_write_SPORT1_TCR1
#define bfin_read_SPORT_TCR1	bfin_read_SPORT1_TCR1
#define bfin_write_SPORT_TCR2	bfin_write_SPORT1_TCR2
#define bfin_write_SPORT_TX16	bfin_write_SPORT1_TX16
#define bfin_read_SPORT_STAT	bfin_read_SPORT1_STAT
#endif

#define GPIO_SE CONFIG_SND_BFIN_AD73311_SE
#define GPIO_RESET CONFIG_SND_BFIN_AD73311_RESET

static struct snd_soc_card bf5xx_ad73311;

static void snd_ad73311_reset(void)
{
	gpio_set_value(GPIO_RESET, 0);
	udelay(100);
	gpio_set_value(GPIO_RESET, 1);
}

static void snd_ad73311_startup(void)
{
	pr_debug("%s enter\n", __func__);

	/* Pull up SE pin on AD73311L */
	gpio_set_value(GPIO_SE, 1);
	udelay(1);
}

static int snd_ad73311_configure(struct ad73311_snd_ctrls *ctrl)
{
	unsigned short ctrl_regs[7];
	unsigned short status = 0;
	int count = 0;

#if CONFIG_SND_AD7XXXX_SELECT == 0
	/* DMCLK = MCLK = 16.384 MHz
	 * SCLK = DMCLK/8 = 2.048 MHz
	 * Sample Rate = DMCLK/2048  = 8 KHz
	 */
	ctrl_regs[0] = AD_CONTROL | AD_WRITE | CTRL_REG_B | REGB_MCDIV(0) | \
			REGB_SCDIV(0) | REGB_DIRATE(ctrl->dirate);
	ctrl_regs[1] = AD_CONTROL | AD_WRITE | CTRL_REG_C | REGC_PUDEV | \
			REGC_PUADC | REGC_PUDAC | REGC_PUREF | REGC_REFUSE;
	ctrl_regs[2] = AD_CONTROL | AD_WRITE | CTRL_REG_D | REGD_OGS(ctrl->ogs) | \
			REGD_IGS(ctrl->igs);
	ctrl_regs[3] = AD_CONTROL | AD_WRITE | CTRL_REG_E | REGE_DA(0x1f);
	ctrl_regs[4] = AD_CONTROL | AD_WRITE | CTRL_REG_F | REGF_SEEN(ctrl->se_en);
	ctrl_regs[5] = AD_CONTROL | AD_WRITE | CTRL_REG_A | REGA_MODE_DATA;
#elif CONFIG_SND_AD7XXXX_SELECT == 1
	/* MCLK = MCLK = 12.288 MHz
	 * Sample Rate = 8 KHz
	 * IMCLK = MCLK/6 = 2.048 MHz = 8kHz * 256
	 */
	ctrl_regs[0] = AD_WRITE | CTRL_REG_A | REGA_REFAMP | REGA_REF |\
			REGA_DAC | REGA_ADC_INPAMP;
	ctrl_regs[1] = AD_WRITE | CTRL_REG_B | REGB_FCLKDIV(2) | \
			REGB_SCLKDIV(1) | REGB_TCLKDIV(0);
	ctrl_regs[2] = AD_WRITE | CTRL_REG_C | REGC_ADC_HP | \
			REGC_WORD_WIDTH(0);
	ctrl_regs[3] = AD_WRITE | CTRL_REG_D | REGD_MASTER | \
			REGD_FDCLK | REGD_DSP_MODE;
	ctrl_regs[4] = AD_WRITE | CTRL_REG_E;
	ctrl_regs[5] = AD_WRITE | CTRL_REG_F;
	ctrl_regs[6] = AD_WRITE | CTRL_REG_G;
#endif
	local_irq_disable();
	snd_ad73311_reset();
	snd_ad73311_startup();

	bfin_write_SPORT_TCR1(TFSR);
	bfin_write_SPORT_TCR2(0xF);
	SSYNC();

	/* SPORT Tx Register is a 8 x 16 FIFO, all the data can be put to
	 * FIFO before enable SPORT to transfer the data
	 */
	for (count = 0; count < 6; count++)
		bfin_write_SPORT_TX16(ctrl_regs[count]);
	SSYNC();
	bfin_write_SPORT_TCR1(bfin_read_SPORT_TCR1() | TSPEN);
	SSYNC();

	/* When TUVF is set, the data is already send out */
	while (!(status & TUVF) && ++count < 10000) {
		udelay(1);
		status = bfin_read_SPORT_STAT();
		SSYNC();
	}
	bfin_write_SPORT_TCR1(bfin_read_SPORT_TCR1() & ~TSPEN);
	SSYNC();
	local_irq_enable();

	if (count >= 10000) {
		printk(KERN_ERR "ad73311: failed to configure codec\n");
		return -1;
	}
	return 0;
}

static int snd_ad73311_write(void *control_data, const char *data, int len)
{
	return snd_ad73311_configure((struct ad73311_snd_ctrls *)data);
}

static int snd_ad73311_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	codec->hw_write = snd_ad73311_write;
	return 0;
}

static int bf5xx_probe(struct platform_device *pdev)
{
	int err;
	struct ad73311_snd_ctrls ctrl = {
		.dirate = 0,
		.igs = 2,
		.ogs = 2,
		.se_en = 1,
	};
	if (gpio_request(GPIO_SE, "AD73311_SE")) {
		printk(KERN_ERR "%s: Failed ro request GPIO_%d\n", __func__, GPIO_SE);
		return -EBUSY;
	}

	if (GPIO_SE != GPIO_RESET) {
		if (gpio_request(GPIO_RESET, "AD73311_RESET")) {
			printk(KERN_ERR "%s: Failed ro request GPIO_%d\n", __func__, GPIO_RESET);
			gpio_free(GPIO_SE);
			return -EBUSY;
		}
	}

	gpio_direction_output(GPIO_SE, 0);
	gpio_direction_output(GPIO_RESET, 0);

	err = snd_ad73311_configure(&ctrl);
	if (err < 0)
		return -EFAULT;

	return 0;
}

static int bf5xx_ad73311_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret = 0;

	pr_debug("%s rate %d format %x\n", __func__, params_rate(params),
		params_format(params));

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_DSP_A |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	return 0;
}


static struct snd_soc_ops bf5xx_ad73311_ops = {
	.hw_params = bf5xx_ad73311_hw_params,
};

static struct snd_soc_dai_link bf5xx_ad73311_dai[] = {
	{
		.name = "ad73311",
		.stream_name = "AD73311",
		.cpu_dai_name = "bfin-i2s.0",
		.codec_dai_name = "ad73311-hifi",
		.platform_name = "bfin-pcm-audio",
		.codec_name = "ad73311-codec",
		.init = snd_ad73311_init,
		.ops = &bf5xx_ad73311_ops,
	},
	{
		.name = "ad73311",
		.stream_name = "AD73311",
		.cpu_dai_name = "bfin-i2s.1",
		.codec_dai_name = "ad73311-hifi",
		.platform_name = "bfin-pcm-audio",
		.codec_name = "ad73311-codec",
		.init = snd_ad73311_init,
		.ops = &bf5xx_ad73311_ops,
	}
};

static struct snd_soc_card bf5xx_ad73311 = {
	.name = "bf5xx_ad73311",
	.probe = bf5xx_probe,
	.dai_link = &bf5xx_ad73311_dai[CONFIG_SND_BF5XX_SPORT_NUM],
	.num_links = 1,
};

static struct platform_device *bf5xx_ad73311_snd_device;

static int __init bf5xx_ad73311_init(void)
{
	int ret;

	pr_debug("%s enter\n", __func__);
	bf5xx_ad73311_snd_device = platform_device_alloc("soc-audio", -1);
	if (!bf5xx_ad73311_snd_device)
		return -ENOMEM;

	platform_set_drvdata(bf5xx_ad73311_snd_device, &bf5xx_ad73311);
	ret = platform_device_add(bf5xx_ad73311_snd_device);

	if (ret)
		platform_device_put(bf5xx_ad73311_snd_device);

	return ret;
}

static void __exit bf5xx_ad73311_exit(void)
{
	pr_debug("%s enter\n", __func__);
	platform_device_unregister(bf5xx_ad73311_snd_device);
}

module_init(bf5xx_ad73311_init);
module_exit(bf5xx_ad73311_exit);

/* Module information */
MODULE_AUTHOR("Cliff Cai");
MODULE_DESCRIPTION("ALSA SoC AD73311 Blackfin");
MODULE_LICENSE("GPL");

