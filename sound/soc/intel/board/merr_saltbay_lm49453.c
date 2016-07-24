/*
 *  merr_saltbay_lm49453.c - ASoc Machine driver for Intel Merrifield MID platform
 *  for the lm49453 codec
 *
 *  Copyright (C) 2012 Intel Corp
 *  Author: Vinod Koul <vinod.koul@linux.intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/async.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/rpmsg.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel_mid_rpmsg.h>
#include <linux/platform_data/intel_mid_remoteproc.h>
#include <asm/platform_mrfld_audio.h>
#include <asm/intel_sst_mrfld.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include "../../codecs/lm49453.h"

static int mrfld_set_clk_fmt(struct snd_soc_dai *codec_dai)
{
	unsigned int fmt;
	int ret;

	pr_debug("Enter in %s\n", __func__);

	/* LM49453  slave Mode */
	fmt =   SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF
		| SND_SOC_DAIFMT_CBS_CFS;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}
	return 0;
}

static int mrfld_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	return mrfld_set_clk_fmt(codec_dai);
}

static int mrfld_compr_set_params(struct snd_compr_stream *cstream)
{
	struct snd_soc_pcm_runtime *rtd = cstream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	return mrfld_set_clk_fmt(codec_dai);
}

struct mrfld_mc_private {
	struct snd_soc_jack jack;
	int jack_retry;
};

static int mrfld_jack_gpio_detect(void);

enum gpios {
	MRFLD_HSDET,
};

static struct snd_soc_jack_gpio hs_gpio[] = {
	[MRFLD_HSDET] = {
		.name			= "mrfld-hsdet-gpio",
		.report			= SND_JACK_HEADSET |
					  SND_JACK_HEADPHONE |
					  SND_JACK_BTN_0,
		.debounce_time		= 50,
		.jack_status_check	= mrfld_jack_gpio_detect,
		.irq_flags		= IRQF_TRIGGER_RISING,
	},
};

#define LM49453_PD_I_IRQ			BIT(4)
#define LM49453_PD_R_IRQ			BIT(5)
#define LM49453_DETECT_REPORT_VALID_IRQ		BIT(6)
#define LM49453_DETECT_REPORT_INVALID_IRQ	BIT(7)

static int mrfld_jack_gpio_detect(void)
{
	struct snd_soc_jack_gpio *gpio = &hs_gpio[MRFLD_HSDET];
	struct snd_soc_jack *jack = gpio->jack;
	struct snd_soc_codec *codec = jack->codec;
	struct mrfld_mc_private *ctx = snd_soc_card_get_drvdata(codec->card);
	unsigned int reg;
	int status = jack->status;

	/* reading register clears interrupt */
	reg = snd_soc_read(codec, LM49453_P0_HSD_IRQ1_REG);
	pr_debug("interrupt received, val = 0x%x, status = 0x%x\n",
		 (reg >> 4), status);
	if (reg & LM49453_DETECT_REPORT_INVALID_IRQ) {
		if (ctx->jack_retry == 0) {
			ctx->jack_retry++;
			pr_debug("invalid headset type - restart HSD\n");
			lm49453_restart_hsd(codec);
			return status;
		}
		ctx->jack_retry = 0;
		if (lm49453_jack_check_config5(codec)) {
			status = SND_JACK_HEADPHONE;
			goto report_hs;
		} else {
			/* invalid type detected */
			return status;
		}
	}
	/* if insertion bit also set,
	 * decide on removal based on type detected
	 */
	if (!(reg & LM49453_PD_I_IRQ) && (reg & LM49453_PD_R_IRQ)) {
		pr_debug("headset removed");
		status = 0;
	}
	if (reg & LM49453_DETECT_REPORT_VALID_IRQ)
		status = lm49453_get_jack_type(codec);

	if ((jack->status & SND_JACK_HEADSET) == SND_JACK_HEADSET) {
		status = lm49453_check_bp(codec, status);
		if (status & SND_JACK_BTN_0) {
			pr_debug("short press detected");
			/* report the press - release will be reported by
			 * jack_gpio framework when we return from this fn */
			snd_soc_jack_report(jack, status, gpio->report);
			status = SND_JACK_HEADSET;
		} else if (status & SND_JACK_BTN_1) {
			pr_debug("long press detected");
			/* android only needs button press/release events for
			 * long press so convert BTN_1 to BTN_0 */
			status &= ~SND_JACK_BTN_1;
			status |= SND_JACK_BTN_0;
		} else if (jack->status & SND_JACK_BTN_0) {
			pr_debug("long release detected");
		}
	}

	if (status == SND_JACK_HEADSET && jack->status == 0) {
		/* unmask button press interrupts */
		snd_soc_write(codec, LM49453_P0_HSD_IRQ_MASK3_REG, 0x70);
		lm49453_set_mic_bias(codec, "AMIC1Bias", true);
	} else if (status == 0) {
		snd_soc_write(codec, LM49453_P0_HSD_IRQ_MASK3_REG, 0);
		lm49453_set_mic_bias(codec, "AMIC1Bias", false);
	}

report_hs:
	return status;
}

static int mrfld_set_bias_level(struct snd_soc_card *card,
				struct snd_soc_dapm_context *dapm,
				enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		card->dapm.bias_level = level;
		break;
	case SND_SOC_BIAS_OFF:
		break;
	default:
		pr_err("%s: Invalid bias level=%d\n", __func__, level);
		return -EINVAL;
		break;
	}
	pr_debug("card(%s)->bias_level %u\n", card->name,
			card->dapm.bias_level);
	return 0;
}

static inline struct snd_soc_codec *mrfld_get_codec(struct snd_soc_card *card)
{
	bool found = false;
	struct snd_soc_codec *codec;

	list_for_each_entry(codec, &card->codec_dev_list, card_list) {
		if (!strstr(codec->name, "lm49453.1-001a")) {
			pr_debug("codec was %s", codec->name);
			continue;
		} else {
			found = true;
			break;
		}
	}
	if (found == false) {
		pr_err("%s: cant find codec", __func__);
		return NULL;
	}
	return codec;
}

static int mrfld_set_bias_level_post(struct snd_soc_card *card,
				     struct snd_soc_dapm_context *dapm,
				     enum snd_soc_bias_level level)
{
	struct snd_soc_codec *codec;
	struct mrfld_mc_private *ctx = snd_soc_card_get_drvdata(card);

	codec = mrfld_get_codec(card);
	if (!codec)
		return -EIO;

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		/* workaround for CONFIG5 headsets - need CHIP_EN to be 11 for
		 * playback to be heard */
		if (ctx->jack.status == SND_JACK_HEADPHONE)
			snd_soc_update_bits(codec, LM49453_P0_PMC_SETUP_REG,
					    LM49453_PMC_SETUP_CHIP_EN,
					    LM49453_CHIP_EN_INVALID_HSD);
		break;
	case SND_SOC_BIAS_OFF:
		if (codec->dapm.bias_level != SND_SOC_BIAS_OFF)
			break;
		card->dapm.bias_level = level;
		break;
	default:
		return -EINVAL;
	}
	pr_debug("%s:card(%s)->bias_level %u\n", __func__, card->name,
			card->dapm.bias_level);
	return 0;
}

static const struct snd_soc_dapm_widget widgets[] = {
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_MIC("Mic", NULL),
};

static const struct snd_soc_dapm_route map[] = {
	{"Headphones", NULL, "HPOUTR"},
	{"Headphones", NULL, "HPOUTL"},
	{"AMIC1", NULL, "Mic"},

	/* SWM map link the SWM outs to codec AIF */
	{ "PORT1_SDI", "NULL", "Codec OUT0"  },
	{ "PORT1_SDI", "NULL", "Codec OUT1"  },
	{ "Codec IN0", "NULL", "PORT1_SDO"  },
	{ "Codec IN1", "NULL", "PORT1_SDO"  },
};

static int mrfld_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret;
	struct snd_soc_codec *codec = runtime->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct snd_soc_card *card = runtime->card;
	struct mrfld_mc_private *ctx = snd_soc_card_get_drvdata(card);

	pr_debug("Entry %s\n", __func__);
	mrfld_set_bias_level(card, dapm, SND_SOC_BIAS_OFF);
	card->dapm.idle_bias_off = true;

	snd_soc_dapm_sync(dapm);
	/* FIXME
	 * set all the nc_pins, set all the init control
	 * and add any machine controls here
	 */

	ctx->jack_retry = 0;
	ret = snd_soc_jack_new(codec, "Intel MID Audio Jack",
			       SND_JACK_HEADSET | SND_JACK_HEADPHONE | SND_JACK_BTN_0,
			       &ctx->jack);
	if (ret) {
		pr_err("jack creation failed\n");
		return ret;
	}
	ret = snd_soc_jack_add_gpios(&ctx->jack, ARRAY_SIZE(hs_gpio), hs_gpio);
	if (ret) {
		pr_err("adding jack GPIO failed\n");
		return ret;
	}

	/* using the 32.7khz RTC for HSD */
	snd_soc_write(codec, LM49453_P0_HSDET_CLK_DIV_REG, 0x00);

	snd_soc_write(codec, LM49453_P0_AUDIO_PORT1_CLK_GEN2_REG, 0x60);
	/* set some magic reg in page 2 for jack detection to work well */
	lm49453_set_reg_on_page(codec, LM49453_PAGE2_SELECT, 0xFC, 0xE);
	/* set HSD mic thresholds to proper values */
	lm49453_set_reg_on_page(codec, LM49453_PAGE2_SELECT, 0x61, 0xDC);
	lm49453_set_reg_on_page(codec, LM49453_PAGE2_SELECT, 0x62, 0x5);
	/* always detect CONFIG5 HS (headphones) as invalid because of removal
	 * interrupt issues and use software workaround to detect its insertion */
	lm49453_set_reg_on_page(codec, LM49453_PAGE2_SELECT, 0x5B, 0x0);

	/* set default debounce time */
	/* TODO: create control for this in codec drv */
	snd_soc_write(codec, LM49453_P0_HSD_PD_DBNC_REG, 0xA);
	/* jack remove debounce time as 192ms */
	snd_soc_write(codec, LM49453_P0_HSD_COMP_H_DBNC_TIME_REG, 0xA);
	/* button press debounce time */
	snd_soc_write(codec, LM49453_P0_HSD_COMP_L_DBNC_TIME_REG, 0x83);

	/* set HSD block to detect only CONFIG1 (LRGM) type headsets */
	snd_soc_write(codec, LM49453_P0_HSD_PIN3_4_CFG_REG, LM49453_JACK_CONFIG1);
	/* set btn long press timing to the smallest value */
	snd_soc_update_bits(codec, LM49453_P0_HSD_PPB_TIMEOUT1_REG, 0xF0, 0);

	snd_soc_write(codec, LM49453_P0_HSD_IRQ_MASK1_REG, 0xF0);
	snd_soc_update_bits(codec, LM49453_P0_PMC_SETUP_REG,
			    LM49453_PMC_SETUP_CHIP_EN,
			    LM49453_CHIP_EN_HSD_DETECT);

	/* For AMIC, we need to configure the DAC ref for our machine to 1.8Vpp
	 * levels to get full scale signals for AMIC record
	 * These registers are in page 2, so set page table to 2 and write them
	 */
	snd_soc_write(codec, LM49453_PAGE_REG, 0x2);
	snd_soc_write(codec, 0xF5, 0x02);
	snd_soc_write(codec, 0x1E, 0xD5);
	snd_soc_write(codec, LM49453_PAGE_REG, 0x0);
	return 0;
}

static unsigned int rates_8000_16000[] = {
	8000,
	16000,
};

static struct snd_pcm_hw_constraint_list constraints_8000_16000 = {
	.count = ARRAY_SIZE(rates_8000_16000),
	.list  = rates_8000_16000,
};

static unsigned int rates_48000[] = {
	48000,
};

static struct snd_pcm_hw_constraint_list constraints_48000 = {
	.count = ARRAY_SIZE(rates_48000),
	.list  = rates_48000,
};

static int mrfld_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_48000);
}

static struct snd_soc_ops mrfld_ops = {
	.startup = mrfld_startup,
	.hw_params = mrfld_hw_params,
};

static int mrfld_8k_16k_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_8000_16000);
}

static struct snd_soc_ops mrfld_8k_16k_ops = {
	.startup = mrfld_8k_16k_startup,
	.hw_params = mrfld_hw_params,
};

static struct snd_soc_compr_ops mrfld_compr_ops = {
	.set_params = mrfld_compr_set_params,
};

struct snd_soc_dai_link mrfld_msic_dailink[] = {
	[MERR_SALTBAY_AUDIO] = {
		.name = "Merrifield Audio Port",
		.stream_name = "Audio",
		.cpu_dai_name = "Headset-cpu-dai",
		.codec_dai_name = "LM49453 Headset",
		.codec_name = "lm49453.1-001a",
		.platform_name = "sst-platform",
		.init = mrfld_init,
		.ignore_suspend = 1,
		.ops = &mrfld_ops,
		.playback_count = 3,
	},
	[MERR_SALTBAY_COMPR] = {
		.name = "Merrifield Compress Port",
		.stream_name = "Compress",
		.cpu_dai_name = "Compress-cpu-dai",
		.codec_dai_name = "LM49453 Headset",
		.codec_name = "lm49453.1-001a",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.compr_ops = &mrfld_compr_ops,
	},
	[MERR_SALTBAY_VOIP] = {
		.name = "Merrifield VOIP Port",
		.stream_name = "Voip",
		.cpu_dai_name = "Voip-cpu-dai",
		.codec_dai_name = "LM49453 Headset",
		.codec_name = "lm49453.1-001a",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &mrfld_8k_16k_ops,
	},
	[MERR_SALTBAY_PROBE] = {
		.name = "Merrifield Probe Port",
		.stream_name = "Probe",
		.cpu_dai_name = "Probe-cpu-dai",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.platform_name = "sst-platform",
		.playback_count = 8,
		.capture_count = 8,
	},
	[MERR_SALTBAY_AWARE] = {
		.name = "Merrifield Aware Port",
		.stream_name = "Aware",
		.cpu_dai_name = "Loopback-cpu-dai",
		.codec_dai_name = "LM49453 Headset",
		.codec_name = "lm49453.1-001a",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &mrfld_8k_16k_ops,
	},
	[MERR_SALTBAY_VAD] = {
		.name = "Merrifield VAD Port",
		.stream_name = "Vad",
		.cpu_dai_name = "Loopback-cpu-dai",
		.codec_dai_name = "LM49453 Headset",
		.codec_name = "lm49453.1-001a",
		.platform_name = "sst-platform",
		.init = NULL,
		.ignore_suspend = 1,
		.ops = &mrfld_8k_16k_ops,
	},
	[MERR_SALTBAY_POWER] = {
		.name = "Virtual Power Port",
		.stream_name = "Power",
		.cpu_dai_name = "Power-cpu-dai",
		.platform_name = "sst-platform",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
};

#ifdef CONFIG_PM_SLEEP
static int snd_mrfld_prepare(struct device *dev)
{
	pr_debug("In %s device name\n", __func__);
	snd_soc_suspend(dev);
	return 0;
}

static void snd_mrfld_complete(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_resume(dev);
}

static int snd_mrfld_poweroff(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_poweroff(dev);
	return 0;
}
#else
#define snd_mrfld_prepare NULL
#define snd_mrfld_complete NULL
#define snd_mrfld_poweroff NULL
#endif

/* SoC card */
static struct snd_soc_card snd_soc_card_mrfld = {
	.name = "lm49453-audio",
	.dai_link = mrfld_msic_dailink,
	.num_links = ARRAY_SIZE(mrfld_msic_dailink),
	.set_bias_level_post = mrfld_set_bias_level_post,
	.dapm_widgets = widgets,
	.num_dapm_widgets = ARRAY_SIZE(widgets),
	.dapm_routes = map,
	.num_dapm_routes = ARRAY_SIZE(map),
};

static int snd_mrfld_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	struct mrfld_mc_private *drv;
	struct mrfld_audio_platform_data *pdata;

	pr_debug("Entry %s\n", __func__);

	drv = kzalloc(sizeof(*drv), GFP_ATOMIC);
	if (!drv) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}
	pdata = pdev->dev.platform_data;
	hs_gpio[MRFLD_HSDET].gpio = pdata->codec_gpio;

	/* register the soc card */
	snd_soc_card_mrfld.dev = &pdev->dev;
	snd_soc_card_set_drvdata(&snd_soc_card_mrfld, drv);
	ret_val = snd_soc_register_card(&snd_soc_card_mrfld);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		goto unalloc;
	}
	platform_set_drvdata(pdev, &snd_soc_card_mrfld);
	pr_info("%s successful\n", __func__);
	return ret_val;

unalloc:
	kfree(drv);
	return ret_val;
}

static int snd_mrfld_mc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct mrfld_mc_private *drv = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);
	snd_soc_jack_free_gpios(&drv->jack, ARRAY_SIZE(hs_gpio), hs_gpio);
	kfree(drv);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void snd_mrfld_mc_shutdown(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct mrfld_mc_private *drv = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);
	snd_soc_jack_free_gpios(&drv->jack, ARRAY_SIZE(hs_gpio), hs_gpio);
}

const struct dev_pm_ops snd_mrfld_mc_pm_ops = {
	.prepare = snd_mrfld_prepare,
	.complete = snd_mrfld_complete,
	.poweroff = snd_mrfld_poweroff,
};

static struct platform_driver snd_mrfld_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "mrfld_lm49453",
		.pm = &snd_mrfld_mc_pm_ops,
	},
	.probe = snd_mrfld_mc_probe,
	.remove = snd_mrfld_mc_remove,
	.shutdown = snd_mrfld_mc_shutdown,
};

static int snd_mrfld_driver_init(void)
{
	pr_info("Merrifield Machine Driver mrfld_lm49453 registerd\n");
	return platform_driver_register(&snd_mrfld_mc_driver);
}

static void snd_mrfld_driver_exit(void)
{
	pr_debug("In %s\n", __func__);
	platform_driver_unregister(&snd_mrfld_mc_driver);
}

static int snd_mrfld_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;

	if (rpdev == NULL) {
		pr_err("rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}

	dev_info(&rpdev->dev, "Probed snd_mrfld rpmsg device\n");

	ret = snd_mrfld_driver_init();

out:
	return ret;
}

static void snd_mrfld_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	snd_mrfld_driver_exit();
	dev_info(&rpdev->dev, "Removed snd_mrfld rpmsg device\n");
}

static void snd_mrfld_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
				int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
			data, len,  true);
}

static struct rpmsg_device_id snd_mrfld_rpmsg_id_table[] = {
	{ .name = "rpmsg_msic_mrfld_audio" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, snd_mrfld_rpmsg_id_table);

static struct rpmsg_driver snd_mrfld_rpmsg = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= snd_mrfld_rpmsg_id_table,
	.probe		= snd_mrfld_rpmsg_probe,
	.callback	= snd_mrfld_rpmsg_cb,
	.remove		= snd_mrfld_rpmsg_remove,
};

static int __init snd_mrfld_rpmsg_init(void)
{
	return register_rpmsg_driver(&snd_mrfld_rpmsg);
}
late_initcall(snd_mrfld_rpmsg_init);

static void __exit snd_mrfld_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&snd_mrfld_rpmsg);
}
module_exit(snd_mrfld_rpmsg_exit);

MODULE_DESCRIPTION("ASoC Intel(R) Merrifield MID Machine driver");
MODULE_AUTHOR("Vinod Koul <vinod.koul@linux.intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:mrfld-audio");
