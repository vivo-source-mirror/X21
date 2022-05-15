/* Add by Lei Chenji <leichenji@vivo.com.cn> for vivo codec. */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/soc-dapm.h>
#include <linux/regulator/consumer.h>
#include "vivo-codec-common.h"
#include "ak4376.h"

#define VIVO_CODEC_SPK_CONTROL 0
#define VIVO_CODEC_HP_CONTROL 1

#define VIVO_HIFI_DEBUG 0

#if VIVO_HIFI_DEBUG
static int hifi_delay0 = 0;
#endif

struct vivo_codec_prv {
	struct snd_soc_codec *codec;
	bool is_hifi_clk_on;
	struct clk *hifi_xi_clk;
	struct audio_params params;
	unsigned int hifi_dac_power_gpio;
	unsigned int hifi_dac_reset_gpio;
	bool hifi_dac_on;
	int cache[2];

	struct vivo_codec_function *fun;
};

static struct vivo_codec_prv *vivo_codec;

static int vivo_codec_hw_reset(enum vivo_codec_id id);
static int vivo_codec_power_down(enum vivo_codec_id id);

static int hifi_mute_mode = 3;
static int hifi_mode_status = 0;//shijianxing add:AK4376 mode status flag

static const char *hifi_mode[]= {"Off", "Normal", "HiFi"}; //shijianxing ad:AK4376 HiFi and Low Power Mode
static const char *hifi_mute_modes[]= {"None", "Left", "Right", "Both"};
static const struct soc_enum vivo_audio_enum[] = {
	SOC_ENUM_SINGLE_EXT(4,hifi_mute_modes),
	SOC_ENUM_SINGLE_EXT(3,hifi_mode),
};

static unsigned int vivo_codec_read(
    struct snd_soc_codec *codec,
    unsigned int reg)
{
	struct vivo_codec_prv *vivo_codec_prv = vivo_codec;

	if (!vivo_codec_prv) {
		pr_err("%s:vivo_codec_prv is NULL\n",
		       __func__);
		return -EINVAL;
	}

	pr_debug("%s:reg 0x%x\n", __func__, reg);

	if (reg <= VIVO_CODEC_HP_CONTROL)
		return vivo_codec_prv->cache[reg];

	return 0;
}

static int vivo_codec_write(
    struct snd_soc_codec *codec,
    unsigned int reg, unsigned int value)
{
	struct vivo_codec_prv *vivo_codec_prv = vivo_codec;

	if (!vivo_codec_prv) {
		pr_err("%s:vivo_codec_prv is NULL\n",
		       __func__);
		return -EINVAL;
	}

	pr_debug("%s:reg 0x%x, value 0x%x\n", __func__, reg, value);

	vivo_codec_prv->cache[reg] = value;
	if (reg == VIVO_CODEC_SPK_CONTROL) {
		if (!value) {
			vivo_codec_power_down(VIVO_CODEC_SMART_PA);
			pr_info("%s: speaker off\n", __func__);
		} else if (value) {
			vivo_codec_hw_reset(VIVO_CODEC_SMART_PA);
			pr_info("%s: speaker on\n", __func__);
		}
	}

	return 0;
}

static int vivo_codec_power_up(enum vivo_codec_id id)
{
	pr_info("%s:id %d\n", __func__, id);

	switch (id) {
	case VIVO_CODEC_HIFI_DAC:
		break;
	case VIVO_CODEC_HIFI_CLK:
		break;
	case VIVO_CODEC_SMART_PA:
		break;
	default:
		pr_err("%s() invalid id %d\n", __func__, id);
	}

	return 0;
}

static int vivo_codec_power_down(enum vivo_codec_id id)
{
//	int ret = 0;

	pr_info("%s:id %d\n", __func__, id);

	switch (id) {
	case VIVO_CODEC_HIFI_DAC:
#if 0
		if (vivo_codec->hifi_dac_reset_gpio > 0) {
			ret = gpio_direction_output(
			          vivo_codec->hifi_dac_reset_gpio,
			          0);
			if (ret) {
				pr_err("%s: Can not set ak4375_rst_gpio", __func__);
				return ret;
			}

			msleep(40);
		}
#endif
		break;
	case VIVO_CODEC_HIFI_CLK:
		break;
	case VIVO_CODEC_SMART_PA:
		break;
	default:
		pr_err("%s() invalid id %d\n", __func__, id);
	}
	return 0;
}

static int vivo_codec_hw_reset(enum vivo_codec_id id)
{
//	int ret;
	pr_info("%s:id %d\n", __func__, id);

	switch (id) {
	case VIVO_CODEC_HIFI_DAC:
#if 0
		if (vivo_codec->hifi_dac_reset_gpio > 0) {
			ret = gpio_direction_output(
			          vivo_codec->hifi_dac_reset_gpio,
			          0);
			if (ret) {
				pr_err("%s: Can not set ak4375_rst_gpio", __func__);
				return ret;
			}

			msleep(40);

			ret = gpio_direction_output(
			          vivo_codec->hifi_dac_reset_gpio,
			          1);
			if (ret) {
				pr_err("%s: Can not set ak4375_rst_gpio", __func__);
				return ret;
			}

			msleep(10);
		}
#endif
		break;
	case VIVO_CODEC_HIFI_CLK:
		break;
	case VIVO_CODEC_SMART_PA:
		break;
	default:
		pr_err("%s() invalid id %d\n", __func__, id);
	}

	return 0;
}

static int vivo_codec_mclk_enable(enum vivo_codec_id id, unsigned long rate)
{
	pr_info("%s enter rate %lu\n", __func__, rate);

	switch (id) {
	case VIVO_CODEC_HIFI_DAC:
		if (vivo_codec->hifi_xi_clk)
			clk_prepare_enable(vivo_codec->hifi_xi_clk);
		break;
	case VIVO_CODEC_HIFI_CLK:
		if (vivo_codec->hifi_xi_clk)
			clk_prepare_enable(vivo_codec->hifi_xi_clk);
		break;
	case VIVO_CODEC_SMART_PA:
		break;
	default:
		pr_err("%s() invalid id %d\n", __func__, id);
	}

	return 0;
}

static int vivo_codec_mclk_disable(enum vivo_codec_id id)
{
	switch (id) {
	case VIVO_CODEC_HIFI_DAC:
		if (vivo_codec->hifi_xi_clk)
			clk_disable_unprepare(vivo_codec->hifi_xi_clk);
		break;
	case VIVO_CODEC_HIFI_CLK:
		if (vivo_codec->hifi_xi_clk)
			clk_disable_unprepare(vivo_codec->hifi_xi_clk);
		break;
	case VIVO_CODEC_SMART_PA:
		break;
	default:
		pr_err("%s() invalid id %d\n", __func__, id);
	}

	return 0;
}

static int vivo_codec_hp_event(struct snd_soc_dapm_widget *w,
                               struct snd_kcontrol *kcontrol, int event)
{
	struct vivo_codec_prv *vivo_codec_prv = vivo_codec;

	pr_info("%s:event %d, %s%s\n", __func__, event,
	        event & (SND_SOC_DAPM_PRE_PMU |
	                 SND_SOC_DAPM_POST_PMU) ? "power up" : "",
	        event & (SND_SOC_DAPM_PRE_PMD |
	                 SND_SOC_DAPM_POST_PMD) ? "power down" : "");

	if (!vivo_codec_prv) {
		pr_err("%s:vivo_codec_prv is NULL\n",
		       __func__);
		return -EINVAL;
	}

	/* hifi_mute_mode = 0 is hifi and 3 is normal, add by zgb */
	pr_info("%s:hifi_mute_mode %d\n", __func__, hifi_mute_mode);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
	case SND_SOC_DAPM_POST_PMU:

		break;
	case SND_SOC_DAPM_PRE_PMD:
	case SND_SOC_DAPM_POST_PMD:

#if VIVO_HIFI_DEBUG
		usleep_range(hifi_delay0 *1000, hifi_delay0 * 1000);
#else
		usleep_range(40 *1000, 40 * 1000);
#endif
		break;
	}

	return 0;
}

static int hifi_codec_dac_event(struct snd_soc_dapm_widget *w,
                                struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;
	struct ak4376_params akmparam;
	//shijianxing add for ak4376 mode change
	akmparam.mode = hifi_mode_status;
	vivo_codec->params.private_params = (void *)&akmparam;
	pr_info("%s hifi mode %d,event %d\n",__func__,akmparam.mode,event);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		if (vivo_codec && vivo_codec->fun && vivo_codec->fun->hifi_dac_enable)
			ret = vivo_codec->fun->hifi_dac_enable(&vivo_codec->params, true);
		hifi_mute_mode = 0; //zgb add for pop noise
		break;
	case SND_SOC_DAPM_POST_PMD:
		hifi_mute_mode = 3; //zgb add for pop noise
		/* disable external dac ak4375 here */
		if (vivo_codec && vivo_codec->fun &&
		    vivo_codec->fun->hifi_dac_enable)
			ret = vivo_codec->fun->hifi_dac_enable(&vivo_codec->params, false);
		break;
	}
	pr_debug("%s() leave.\n", __func__);

	return ret;
}

static const struct snd_kcontrol_new headphone_rx_mixer_controls[] = {
	SOC_DAPM_SINGLE("I2S_RX", VIVO_CODEC_HP_CONTROL,
	                0, 1, 0),
};

static const struct snd_soc_dapm_widget vivo_codec_dapm_widgets[] = {
	SND_SOC_DAPM_HP("HP", vivo_codec_hp_event),

	/* AIF Definitions */
	SND_SOC_DAPM_OUTPUT("HiFi HPOUT"),
	SND_SOC_DAPM_AIF_IN_E("I2S_IN", "vivo HiFi Playback", 0, SND_SOC_NOPM,
	                      I2S_IN, 0, NULL,
	                      SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_DAC_E("HiFi DAC", NULL, SND_SOC_NOPM, 0, 0,
	                   hifi_codec_dac_event, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_PGA_E("HiFi AMP", SND_SOC_NOPM, 0, 0, NULL, 0,
	                   NULL, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_AIF_OUT_E("I2S_OUT", "vivo HiFi Capture", 0, SND_SOC_NOPM,
	                       I2S_OUT, 0, NULL,
	                       SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	/* Mixer Definitions */
	SND_SOC_DAPM_MIXER("Headphone Rx Mixer", SND_SOC_NOPM, 0, 0,
	                   headphone_rx_mixer_controls, ARRAY_SIZE(headphone_rx_mixer_controls)),
};

static const struct snd_soc_dapm_route vivo_codec_audio_map[] = {
	{"HP", NULL, "HiFi HPOUT"},
	/* headphone rx audio route  */
	{"HiFi HPOUT", NULL, "HiFi AMP"},
	{"HiFi AMP", NULL, "HiFi DAC"},
	{"HiFi DAC", NULL, "Headphone Rx Mixer"},
	{"Headphone Rx Mixer", "I2S_RX", "I2S_IN"},
};

static int vivo_hifi_mute_get(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = hifi_mute_mode;
	pr_info("%s : %d\n",__func__,(int)(ucontrol->value.enumerated.item[0]));
	return 0;
}

static int vivo_hifi_mute_put(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	int changed=0;
	hifi_mute_mode = ucontrol->value.integer.value[0];
	pr_info("%s : %d\n",__func__,(int)(ucontrol->value.integer.value[0]));
	if (vivo_codec && vivo_codec->fun && vivo_codec->fun->hifi_dac_mute)
		vivo_codec->fun->hifi_dac_mute(hifi_mute_mode & 3);
	return changed;
}

#if VIVO_HIFI_DEBUG
static int vivo_hifi_get(struct snd_kcontrol *kcontrol,
                         struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 0;//get_mic_switch_state();
//	pr_debug("%s : %d\n",__func__,(int)(ucontrol->value.enumerated.item[0]));
	return 0;
}

static int vivo_hifi_put(struct snd_kcontrol *kcontrol,
                         struct snd_ctl_elem_value *ucontrol)
{
	int shift = ((struct soc_multi_mixer_control *)
	             kcontrol->private_value)->shift;
//	int enable = ucontrol->value.integer.value[0];
	int ret = 0;
	struct ak4376_params akmparam;

	switch(shift) {
	case 4:
		akmparam.mode = hifi_mode_status;
		vivo_codec->params.private_params = (void *)&akmparam;
		if (vivo_codec && vivo_codec->fun && vivo_codec->fun->hifi_dac_enable)
			ret = vivo_codec->fun->hifi_dac_enable(&vivo_codec->params, false);
		break;
	case 5:
		hifi_delay0 = ucontrol->value.integer.value[0];
		pr_info("%s() delay0 %d ms.\n", __func__, hifi_delay0);
		break;
	case 6:
		if (ucontrol->value.integer.value[0])
			clk_prepare_enable(vivo_codec->hifi_xi_clk);
		else
			clk_disable_unprepare(vivo_codec->hifi_xi_clk);
		break;
	}
	return 1;
}
#endif
/*
*shijianxing add:AK4376 HiFi Mode and Low Power Mode change control
*/
static int vivo_hifi_mode_get(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = hifi_mode_status;
	pr_info("%s : %d\n",__func__,(int)(ucontrol->value.enumerated.item[0]));
	return 0;
}

static int vivo_hifi_mode_put(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_value *ucontrol)
{
	hifi_mode_status = ucontrol->value.integer.value[0];
	pr_info("%s : %d\n",__func__,(int)(ucontrol->value.integer.value[0]));

	return 0;
}

static const struct snd_kcontrol_new vivo_codec_controls[] = {
	SOC_ENUM_EXT("HiFi Mute",vivo_audio_enum[0],
	             vivo_hifi_mute_get, vivo_hifi_mute_put),
	SOC_ENUM_EXT("HiFi Mode Switch",vivo_audio_enum[1],
	             vivo_hifi_mode_get, vivo_hifi_mode_put),

#if VIVO_HIFI_DEBUG
	SOC_SINGLE_EXT("HiFi Switch Sel", SND_SOC_NOPM, 1, 1, 0,
	               vivo_hifi_get, vivo_hifi_put),
	SOC_SINGLE_EXT("HiFi 5v Switch", SND_SOC_NOPM, 2, 1, 0,
	               vivo_hifi_get, vivo_hifi_put),
	SOC_SINGLE_EXT("HiFi 3v3 Switch", SND_SOC_NOPM, 3, 1, 0,
	               vivo_hifi_get, vivo_hifi_put),
	SOC_SINGLE_EXT("HiFi Dac Switch", SND_SOC_NOPM, 4, 1, 0,
	               vivo_hifi_get, vivo_hifi_put),
	SOC_SINGLE_EXT("HiFi Delay0", SND_SOC_NOPM, 5, 1000, 0,
	               vivo_hifi_get, vivo_hifi_put),
	SOC_SINGLE_EXT("HiFi 19.2M Clk", SND_SOC_NOPM, 6, 1, 0,
	               vivo_hifi_get, vivo_hifi_put),
#endif
};


static int vivo_codec_hw_params(struct snd_pcm_substream *substream,
                                struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct vivo_codec_prv *vivo_codec_prv = vivo_codec;
	int format, rate;

	if (!vivo_codec_prv || !vivo_codec_prv->fun) {
		pr_err("%s:vivo_codec_prv or vivo_codec_prv->fun is NULL\n",
		       __func__);
		return -EINVAL;
	}

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		format = SNDRV_PCM_FORMAT_S16_LE;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		format = SNDRV_PCM_FORMAT_S24_LE;
		break;
	default:
		return -EINVAL;
	}

	rate = params_rate(params);

	pr_info("%s: format %d, rate %d \n", __func__, format, rate);

	vivo_codec_prv->params.pcm_format = format;

	vivo_codec_prv->params.rate = rate;
	vivo_codec_prv->params.sys_clk = 19200000;

	return 0;
}

static int vivo_codec_set_fmt(struct snd_soc_dai *dai,
                              unsigned int fmt)
{
	struct vivo_codec_prv *vivo_codec_prv = vivo_codec;

	pr_info("%s:fmt 0x%x\n", __func__, fmt);

	if (!vivo_codec_prv) {
		pr_err("%s:vivo_codec_prv is NULL\n",
		       __func__);
		return -EINVAL;
	}

	vivo_codec_prv->params.i2s_format = fmt;

	return 0;
}

/* zgb change for dispel pop noise */
static int vivo_codec_dai_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	int ret = 0;
	pr_info("%s: mute %d\n", __func__, mute);

	return ret;
}
/* zgb end */

static struct snd_soc_dai_ops vivo_codec_ops = {
	.hw_params = vivo_codec_hw_params,
	.set_fmt = vivo_codec_set_fmt,
	//.digital_mute = vivo_codec_dai_digital_mute, //zgb change to mute_stream
	.mute_stream = vivo_codec_dai_mute_stream,
};

static struct snd_soc_dai_ops vivo_codec_tx_ops = {
	.hw_params = vivo_codec_hw_params,
	.set_fmt = vivo_codec_set_fmt,
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
		.ops = &vivo_codec_tx_ops,
	},
};

static int vivo_codec_suspend(struct snd_soc_codec *codec)
{
	return 0;
}

static int vivo_codec_resume(struct snd_soc_codec *codec)
{
	return 0;
}

static int vivo_codec_probe(struct snd_soc_codec *codec)
{
	struct vivo_codec_prv *vivo_codec_prv = vivo_codec;
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);

	pr_info("%s\n", __func__);

	if (!vivo_codec_prv) {
		pr_err("%s:vivo_codec_prv is NULL\n",
		       __func__);
		return -EINVAL;
	}

	vivo_codec_prv->codec = codec;

	snd_soc_dapm_ignore_suspend(dapm, "HP");
	snd_soc_dapm_ignore_suspend(dapm, "HiFi HPOUT");
	snd_soc_dapm_ignore_suspend(dapm, "vivo HiFi Playback");

	return 0;
}

static int vivo_codec_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static struct snd_soc_codec_driver vivo_codec_driver = {
	.probe = 	vivo_codec_probe,
	.remove = 	vivo_codec_remove,
	.suspend =	vivo_codec_suspend,
	.resume = 	vivo_codec_resume,
	.read = vivo_codec_read,
	.write = vivo_codec_write,
	.controls = vivo_codec_controls,
	.num_controls = ARRAY_SIZE(vivo_codec_controls),
	.dapm_widgets = vivo_codec_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(vivo_codec_dapm_widgets),
	.dapm_routes = vivo_codec_audio_map,
	.num_dapm_routes = ARRAY_SIZE(vivo_codec_audio_map),
};

static void vivo_codec_parse_dt(struct platform_device *pdev,
                                struct vivo_codec_prv *priv)
{
	int ret = 0;
	const char *model = NULL;

	if (!priv) {
		pr_err("%s() priv is NULL\n", __func__);
		return;
	}
	if (!priv) {
		pr_err("%s() priv is NULL\n", __func__);
		return;
	}

	/* parse clks */
	priv->hifi_xi_clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->hifi_xi_clk)) {
		pr_err("%s() Could not get hifi_xi_clk\n", __func__);
	}
#if 0
	/* parse gpios */
	priv->hifi_dac_power_gpio=
	    of_get_named_gpio(pdev->dev.of_node, "vivo,dac-power-gpio", 0);
	ret = gpio_request(priv->hifi_dac_power_gpio,"hifi dac power");
	if(ret)
		pr_err("%s:gpio %d for dac power gpio request failed...\n", __func__,
		       priv->hifi_dac_power_gpio);

	priv->hifi_dac_reset_gpio =
	    of_get_named_gpio(pdev->dev.of_node, "vivo,dac-reset-gpio", 0);
	ret = gpio_request(priv->hifi_dac_reset_gpio,"hifi dac reset");
	if(ret)
		pr_err("%s:gpio %d for dac reset gpio request failed...\n", __func__,
		       priv->hifi_dac_reset_gpio);
#endif
	ret = of_property_read_string(pdev->dev.of_node,
	                              "vivo,model", &model);
	if (ret) {
		pr_err("%s read model property failed.\n", __func__);
	} else {
		pr_info("%s model %s.\n", __func__, model);
		if (!strcmp(model, "PD1602A"))
			;
	}

	return;
}

static int vivo_codec_platform_probe(
    struct platform_device *pdev)
{
	struct vivo_codec_prv *vivo_codec_prv;
	struct vivo_codec_function *vivo_codec_function;
	int ret = 0;

	pr_info("%s() enter\n", __func__);

	vivo_codec_function = kmalloc(sizeof(struct vivo_codec_function), GFP_KERNEL);
	if (!vivo_codec_function) {
		pr_err("%s:vivo_codec_function malloc failed\n", __func__);
		return -ENOMEM;
	}

	vivo_codec_prv = kmalloc(sizeof(struct vivo_codec_prv), GFP_KERNEL);
	if (!vivo_codec_prv) {
		pr_err("%s:vivo_codec_prv malloc failed\n", __func__);
		ret = -ENOMEM;
		goto err_kmalloc;
	}
	vivo_codec_parse_dt(pdev, vivo_codec_prv);

	vivo_codec_function->power_up = vivo_codec_power_up;
	vivo_codec_function->power_down = vivo_codec_power_down;
	vivo_codec_function->hw_reset = vivo_codec_hw_reset;
	vivo_codec_function->mclk_enable = vivo_codec_mclk_enable;
	vivo_codec_function->mclk_disable = vivo_codec_mclk_disable;
	vivo_codec_function->get_hp_switch_state = NULL;
	vivo_codec_function->set_hp_switch_state = NULL;
	vivo_codec_function->get_mic_switch_state = NULL;//get_mic_switch_state;
	vivo_codec_function->set_mic_switch_state = NULL;//set_mic_switch_state;

	vivo_codec_function->hifi_clk_enable = NULL;
	vivo_codec_function->hifi_dac_enable = NULL;
	vivo_codec_function->hifi_dac_mute = NULL;
	vivo_codec_function->smart_pa_enable = NULL;
	vivo_codec_function->smart_pa_set_mode = NULL;
	vivo_codec_function->smart_pa_mute = NULL;
	vivo_codec_function->mbhc_get_hp_impedance = NULL;

	set_vivo_codec_function(vivo_codec_function);

//	gpio_direction_output(vivo_codec_prv->hifi_dac_power_gpio,1);//legen

	pr_info("%s() vivo_codec_fun(%p)%p\n",
	        __func__, &vivo_codec_function, vivo_codec_function);

	vivo_codec_prv->params.rate = 48000;
	vivo_codec_prv->params.pcm_format = SNDRV_PCM_FORMAT_S16_LE;
	vivo_codec_prv->params.i2s_format = SND_SOC_DAIFMT_CBS_CFS |
	                                    SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_I2S;
	vivo_codec_prv->params.private_params = NULL;
	vivo_codec_prv->cache[VIVO_CODEC_SPK_CONTROL] = false;
	vivo_codec_prv->cache[VIVO_CODEC_HP_CONTROL] = false;
	vivo_codec_prv->hifi_dac_on = false;
	vivo_codec_prv->is_hifi_clk_on = false;
	vivo_codec_prv->codec = NULL;
	vivo_codec_prv->fun = vivo_codec_function;
	vivo_codec = vivo_codec_prv;

	platform_set_drvdata(pdev, vivo_codec_function);

	dev_set_name(&pdev->dev, "%s", "hifi-codec");

	ret = snd_soc_register_codec(&pdev->dev, &vivo_codec_driver,
	                             vivo_codec_dai, ARRAY_SIZE(vivo_codec_dai));
	if(ret < 0) {
		pr_err("soc register error %s,rc=%d\n", __func__, ret);
		goto err_register_codec;
	}

	pr_info("%s:complete\n", __func__);

	return 0;

err_register_codec:
	if (vivo_codec_prv) {
		kfree(vivo_codec_prv);
		vivo_codec = NULL;
	}
err_kmalloc:
	if (vivo_codec_function) {
		kfree(vivo_codec_function);
		set_vivo_codec_function(NULL);
	}

	return ret;
}

static int vivo_codec_platform_remove(
    struct platform_device *pdev)
{
	if (vivo_codec && vivo_codec->fun) {
		kfree(vivo_codec->fun);
		vivo_codec->fun = NULL;
	}

	if (vivo_codec) {
		kfree(vivo_codec);
		vivo_codec = NULL;
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id vivo_codec_of_match[] = {
	{.compatible = "vivo,hifi-codec-TD1608",},
	{},
};
#else
#define vivo_codec_of_match 0
#endif

static struct platform_driver vivo_codec_platform_driver = {
	.probe = vivo_codec_platform_probe,
	.remove = vivo_codec_platform_remove,
	.driver = {
		.name = "hifi-codec-TD1608",
		.owner = THIS_MODULE,
		.of_match_table = vivo_codec_of_match,
	},
};

module_platform_driver(vivo_codec_platform_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lei Chenji <leichenji@vivo.com.cn> ");
MODULE_DESCRIPTION(" vivo hifi codec driver for PD1602A");
