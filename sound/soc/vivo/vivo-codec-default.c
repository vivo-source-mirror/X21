//liuxudong create for codec class only have smart PA
#define DEBUG 1
#include <linux/init.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/printk.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <sound/asound.h>
#include <linux/ratelimit.h>
#include <linux/debugfs.h>
#include <linux/wait.h>
#include <sound/tlv.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>

#include <linux/timer.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/log2.h>

#include <linux/regulator/consumer.h>

#define VIVO_CODEC_NAME "vivo-codec"

static int vivo_codec_soc_probe(struct snd_soc_codec *codec)
{
	int rc = 0;

	pr_debug("%s()\n", __func__);

	return rc;
}

static int vivo_codec_soc_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static int vivo_codec_soc_suspend(struct snd_soc_codec *codec)
{
	return 0;
}

static int vivo_codec_soc_resume(struct snd_soc_codec *codec)
{
	return 0;
}

static unsigned int vivo_codec_read(struct snd_soc_codec *codec, unsigned int count)
{
	return 0;
}

static int vivo_codec_write(struct snd_soc_codec *codec, unsigned int count, unsigned int cmd)
{

	return 0;
}

static int vivo_codec_set_fmt(struct snd_soc_dai *dai,
                              unsigned int fmt)
{
	pr_debug("%s:fmt 0x%x\n", __func__, fmt);

	return 0;
}

static int vivo_codec_mute_stream(struct snd_soc_dai *dai,
                                  int mute, int stream)
{
	int ret = 0;

	pr_debug("%s()\n", __func__);

	return ret;
}

static struct snd_soc_dai_ops vivo_codec_ops = {
	.set_fmt	  = vivo_codec_set_fmt,
	.mute_stream = vivo_codec_mute_stream,
};

static struct snd_soc_dai_driver vivo_codec_dai[] = {
	{
		.name = "vivo-hifi-rx",
		.id = 0,
		.playback = {
			.stream_name = "vivo HiFi Playback",
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &vivo_codec_ops,
	},
	{
		.name = "vivo-hifi-tx",
		.id = 1,
		.capture = {
			.stream_name = "vivo HiFi Capture",
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
			.rate_max = 192000,
			.rate_min = 8000,
			.channels_min = 1,
			.channels_max = 2,
		},
		.ops = &vivo_codec_ops,
	},
};

static const char * const hp_path_text[]= { "Normal",};

static const struct soc_enum hp_path_enum =
    SOC_ENUM_SINGLE(0, 0, 1, hp_path_text);

static const struct snd_kcontrol_new hp_path_mux =
    SOC_DAPM_ENUM("HP Path Mux", hp_path_enum);


static const struct snd_soc_dapm_widget vivo_codec_dapm_widgets[] = {

	SND_SOC_DAPM_AIF_IN_E("I2S In", "VIVO HiFi Playback", 0, SND_SOC_NOPM,
	                      0, 0, NULL, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("I2S Out", "VIVO HiFi Capture", 0, SND_SOC_NOPM,
	                       0, 0, NULL, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_OUTPUT("SPKER"),

	SND_SOC_DAPM_DAC_E("SPK DAC", NULL, 0, 0, 0
	                   , NULL, SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("SMART SPK PA", SND_SOC_NOPM, 0, 0, NULL,
	                   0, NULL,
	                   SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	/* compatible with msm8x16.vivo_codec_audio_map */
	SND_SOC_DAPM_MUX("HPOUT Path", SND_SOC_NOPM, 0, 0, &hp_path_mux),
};

static const struct snd_soc_dapm_route vivo_codec_map[] = {

	{"SPEAKER", NULL, "SMART SPK PA"},
	{"SMART SPK PA", NULL, "SPK DAC"},
	{"SPK DAC", NULL, "I2S In"},
};

static struct snd_soc_codec_driver soc_vivo_codec_driver = {
	.probe = 	vivo_codec_soc_probe,
	.remove = 	vivo_codec_soc_remove,
	.suspend =	vivo_codec_soc_suspend,
	.resume = 	vivo_codec_soc_resume,
	.read = vivo_codec_read,
	.write = vivo_codec_write,
	.dapm_widgets = vivo_codec_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(vivo_codec_dapm_widgets),
	.dapm_routes = vivo_codec_map,
	.num_dapm_routes = ARRAY_SIZE(vivo_codec_map),
};

static int vivo_codec_probe(struct platform_device *pdev)
{
	int rc = 0;

	pr_info("%s:enter\n",__func__);
	if(!pdev->dev.of_node)
		return -ENODEV;
	dev_set_name(&pdev->dev,"%s","vivo-codec");

	rc = snd_soc_register_codec(&pdev->dev,&soc_vivo_codec_driver,
	                            vivo_codec_dai,ARRAY_SIZE(vivo_codec_dai));
	if(rc < 0)
		pr_err("soc register error %s,rc=%d\n",__func__,rc);
	pr_info("%s:leave\n",__func__);

	return 0;
}
static int vivo_codec_remove(struct platform_device *pdev)
{
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id device_codec_of_match[] = {
	{.compatible = "vivo,vivo-codec-default",},
	{ },
};
#else
#define device_codec_of_match 0
#endif

static struct platform_driver vivo_codec_driver = {
	.probe = vivo_codec_probe,
	.remove = vivo_codec_remove,
	.driver = {
		.name = VIVO_CODEC_NAME,
		.owner = THIS_MODULE,
		.of_match_table = device_codec_of_match,
	},
};

static int __init vivo_codec_init(void)
{
	int rc = 0;
	rc = platform_driver_register(&vivo_codec_driver);
	if(rc < 0)
		pr_err("platform register failed (%s)\n",__func__);
	return 0;
}

static void __exit vivo_codec_exit(void)
{
	platform_driver_unregister(&vivo_codec_driver);
	return;
}

module_init(vivo_codec_init);
module_exit(vivo_codec_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("leichenji <leichenji@vivo.com>");
MODULE_DESCRIPTION(" vivo codec driver");

