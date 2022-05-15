#include <linux/device.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/stat.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/qpnp/pin.h>
#include "fp_id.h"

#define MAX_TIMES		7

struct vreg_config {
	char *name;
	unsigned long vmin;
	unsigned long vmax;
	int ua_load;
};
/*static const struct vreg_config const vreg_conf[] = {
	{ "vcc_fpc", 1800000UL, 1800000UL, 10, },
	{ "vcc_goodix", 2800000UL, 2800000UL, 10, },
};*/

static const char * const pctl_names[] = {
	"fp_id_gpio_up",
	"fp_id_gpio_down",
};

static struct qpnp_pin_cfg gpio_pullup_input = {
	.mode = 0,
	.pull = 0,
	.vin_sel = 0,
	.src_sel = 0,
	.out_strength = 1,
	.master_en = 1,
};

static struct qpnp_pin_cfg gpio_pulldown_input = {
	.mode = 0,
	.pull = 4,
	.vin_sel = 0,
	.src_sel = 0,
	.out_strength = 1,
	.master_en = 1,
};

static int fp_gpio = -1;
static int fp_id = -1;
static int fp_up = -1;
static int fp_down = -1;
static int count_reset_high_ground;
static int count_reset_high_suspend;
static int count_reset_low_ground;
static int count_reset_low_suspend;
const char *fp_project_name;
/*
static int count_ground = 0;//fp_id pin connect to ground
static int count_suspend = 0;//fp_id pin suspend
struct regulator *fp_vreg[ARRAY_SIZE(vreg_conf)];
*/
struct pinctrl *fingerprint_pinctrl;
struct pinctrl_state *pinctrl_state[ARRAY_SIZE(pctl_names)];

#define DEVFS_MODE_RO (S_IRUSR|S_IRGRP|S_IROTH)
struct attribute fp_id_attr = {
	.name = "fp_id",
	.mode = DEVFS_MODE_RO,
};
static struct attribute *our_own_sys_attrs[] = {
	&fp_id_attr,
	NULL,
};

/*static int vreg_setup(struct device *dev, const char *name,
	bool enable)
{
	size_t i;
	int rc;
	struct regulator *vreg;
	
	printk("vreg_setup start\n");
	for (i = 0; i < ARRAY_SIZE(fp_vreg); i++) {
		const char *n = vreg_conf[i].name;
		if (!strncmp(n, name, strlen(n)))
			goto found;
	}
	dev_err(dev, "Regulator %s not found\n", name);
	return -EINVAL;
found:
	vreg = fp_vreg[i];
	if (enable) {
		if (!vreg) {
			vreg = regulator_get(dev, name);
			if (!vreg) {
				dev_err(dev, "Unable to get  %s\n", name);
				return -ENODEV;
			}
		}
		printk("vreg_setup 111111\n");
		if (regulator_count_voltages(vreg) > 0) {
			rc = regulator_set_voltage(vreg, vreg_conf[i].vmin,
					vreg_conf[i].vmax);
			if (rc)
				dev_err(dev,
					"Unable to set voltage on %s, %d\n",
					name, rc);
		}
		rc = regulator_set_optimum_mode(vreg, vreg_conf[i].ua_load);
		if (rc < 0)
			dev_err(dev, "Unable to set current on %s, %d\n",
					name, rc);
		rc = regulator_enable(vreg);
		if (rc) {
			dev_err(dev, "error enabling %s: %d\n", name, rc);
			regulator_put(vreg);
			vreg = NULL;
		}
		fp_vreg[i] = vreg;
	} else {
		if (vreg) {
			if (regulator_is_enabled(vreg)) {
				regulator_disable(vreg);
				dev_dbg(dev, "disabled %s\n", name);
			}
			regulator_put(vreg);
			fp_vreg[i] = NULL;
		}
		rc = 0;
	}
	return rc;
}*/

static int select_pin_ctl(struct device *dev, const char *name)
{
	size_t i;
	int rc;
	for (i = 0; i < ARRAY_SIZE(pinctrl_state); i++) {
		const char *n = pctl_names[i];
		if (!strncmp(n, name, strlen(n))) {
			rc = pinctrl_select_state(fingerprint_pinctrl, pinctrl_state[i]);
			if (rc)
				dev_err(dev, "cannot select '%s'\n", name);
			else
				dev_dbg(dev, "Selected '%s'\n", name);
			goto exit;
		}
	}
	rc = -EINVAL;
	dev_err(dev, "%s:'%s' not found\n", __func__, name);
exit:
	return rc;
}

static ssize_t fp_id_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	char *fp_frame_id ;
	if (FPC_FPC1022 == fp_id) {
		fp_frame_id = "fpc_1022";
	} else if (FPC_FPC1245 == fp_id) {
		fp_frame_id = "fpc_1245";
	} else if (GOODIX_GF5116M == fp_id) {
		fp_frame_id = "goodix_5116";
	} else if (GOODIX_GF52X6 == fp_id) {
		fp_frame_id = "goodix_5216";
	} else if (GOODIX_GF3208 == fp_id) {
		fp_frame_id = "goodix_3208";
	} else if (GOODIX_GF5269 == fp_id) {
		fp_frame_id = "goodix_5269";
	} else if (fp_id == SYNAPTICS_FS9501) {
		fp_frame_id = "udfp_syna_fs9501";
	} else if (GOODIX_GF5288 == fp_id) {
		fp_frame_id = "goodix_5288";
	} else if (GOODIX_GF9118 ==  fp_id) {
		fp_frame_id = "udfp_goodix_gf9118";
	} else if (GOODIX_GF9518 ==  fp_id) {
		fp_frame_id = "udfp_goodix_gf9518";
	} else {
		fp_frame_id = "default";
	}
	printk("fp_project_name:%s, get_fp_id=%d, fp_frame_id=%s, fp_up=%d, fp_down=%d, fp_gpio=%d.\n", fp_project_name, get_fp_id(), fp_frame_id, fp_up, fp_down, fp_gpio);
	printk("%s: count_reset_high_ground=%d, count_reset_high_suspend=%d, count_reset_low_ground=%d, count_reset_low_suspend=%d.\n", __func__, count_reset_high_ground, count_reset_high_suspend, count_reset_low_ground, count_reset_low_suspend);
	return snprintf(buf, strlen(fp_frame_id) + 1, "%s", fp_frame_id);
}

int get_fp_id(void)
{
	printk("get_fp_id , fp_id=%d, fp_up=%d, fp_down=%d\n", fp_id, fp_up, fp_down);
	return fp_id;
}
EXPORT_SYMBOL(get_fp_id);

static void fp_id_object_release(struct kobject *kobj)
{
	/* nothing to do temply */
	return;
}

static const struct sysfs_ops fp_id_object_sysfs_ops = {
	.show = fp_id_object_show,
};
static struct kobj_type fp_id_object_type = {
	.sysfs_ops	= &fp_id_object_sysfs_ops,
	.release	= fp_id_object_release,
	.default_attrs = our_own_sys_attrs,
};

struct kobject kobj;
static int
fp_id_probe(struct platform_device *pdev)
{
	int ret;
	/*int fp_gpio;*/
	int vdd_en_gpio;
	int i;

	ret = kobject_init_and_add(&kobj, &fp_id_object_type, NULL, "fp_id");
	if (ret) {
		printk("%s: Create fp_id error!\n", __func__);
		return -ERRORFP;
	}

	ret = of_property_read_string(pdev->dev.of_node, "vivo,project-name", &fp_project_name);
	if (ret) {
		printk("%s:vivo,project-name property do not find\n", __func__);
		fp_project_name = "default";
		return -ERRORFP;
	}
	printk("%s:vivo,project-name = %s\n", __func__, fp_project_name);

	if ((!strncmp(fp_project_name, "PD1814F_EX", 10)) || (!strncmp(fp_project_name, "PD1816", 6))) {
		fp_id = GOODIX_GF9518;
		printk("%s: return gf9518 directly\n", __func__);
		return 0;
	}

	fp_gpio = of_get_named_gpio(pdev->dev.of_node, "fp_id,gpios", 0);
	if (fp_gpio < 0) {
		printk("%s: get fp_id gpio failed!\n", __func__);
		return -ERRORFP;
	}
	printk("%s:fp gpio: %d \n", __func__, fp_gpio);

	ret = devm_gpio_request(&pdev->dev, fp_gpio, "fp_id,gpios");
	if (ret) {
		printk("%s: request fp_id gpio failed!\n", __func__);
		return -ERRORFP;
	}

	if (!strncmp(fp_project_name, "PD1710", 6)) {
		vdd_en_gpio = of_get_named_gpio(pdev->dev.of_node, "fp_id,gpio_vdd_en", 0);
		if (vdd_en_gpio < 0) {
			printk("%s: get fpc vdd_en gpio failed!\n", __func__);
			return -ERRORFP;
		}

		ret = devm_gpio_request(&pdev->dev, vdd_en_gpio, "fp_id,gpio_vdd_en");
		if (ret) {
			printk("%s: request fpc vdd_en gpio failed!\n", __func__);
			return -ERRORFP;
		}

		gpio_direction_output(vdd_en_gpio, 1);
		usleep_range(100, 200);

		ret = qpnp_pin_config(fp_gpio, &gpio_pullup_input);
		mdelay(5);
		if (ret) {
			printk("%s: qpnp_pin_config gpio_pullup_input failed! ret:%d.\n", __func__, ret);
			return -ERRORFP;
		}

		for (i = 0; i < MAX_TIMES; i++) {
			mdelay(1);
			ret = gpio_get_value(fp_gpio);
			if (ret == 0)
				count_reset_high_ground++;
			else
				count_reset_high_suspend++;
		}
		fp_up = (count_reset_high_ground > count_reset_high_suspend) ? 0:1;

		ret = qpnp_pin_config(fp_gpio, &gpio_pulldown_input);
		mdelay(5);
		if (ret) {
			printk("%s: qpnp_pin_config gpio_pulldown_input failed! ret:%d.\n", __func__, ret);
			return -ERRORFP;
		}

		for (i = 0; i < MAX_TIMES; i++) {
			mdelay(1);
			ret = gpio_get_value(fp_gpio);
			if (ret == 0)
				count_reset_low_ground++;
			else
				count_reset_low_suspend++;
		}
		fp_down = (count_reset_low_ground > count_reset_low_suspend) ? 0:1;

		if ((fp_up == 0) && (fp_down == 0)) {
			fp_id = GOODIX_GF3208;
			ret = qpnp_pin_config(fp_gpio, &gpio_pulldown_input);
		} else if ((fp_up == 1) && (fp_down == 1)) {
			fp_id = FPC_FPC1022;
			ret = qpnp_pin_config(fp_gpio, &gpio_pullup_input);
		}

		gpio_direction_output(vdd_en_gpio, 0);
		usleep_range(100, 200);

		devm_gpio_free(&pdev->dev, fp_gpio);
		devm_gpio_free(&pdev->dev, vdd_en_gpio);
		return 0;
	}

	fingerprint_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(fingerprint_pinctrl)) {
		if (PTR_ERR(fingerprint_pinctrl) == -EPROBE_DEFER) {
			printk("%s: pinctrl not ready!\n", __func__);
			return -ERRORFP;
		}
		printk("%s: Target does not use pinctrl\n", __func__);
		fingerprint_pinctrl = NULL;
		return -ERRORFP;
	}

	for (i = 0; i < ARRAY_SIZE(pinctrl_state); i++) {
		const char *n = pctl_names[i];
		struct pinctrl_state *state =
			pinctrl_lookup_state(fingerprint_pinctrl, n);
		if (IS_ERR(state)) {
			printk("%s: cannot find '%s'\n", __func__, n);
			return -ERRORFP;
		}
		printk("%s: found pin control %s\n", __func__, n);
		pinctrl_state[i] = state;
	}

	ret = select_pin_ctl(&pdev->dev, "fp_id_gpio_up");
	mdelay(5);
	if (ret)
		return -ERRORFP;
	for (i = 0; i < MAX_TIMES; i++) {
		mdelay(1);
		ret = gpio_get_value(fp_gpio);
		if (ret == 0)
			count_reset_high_ground++;
		else
			count_reset_high_suspend++;
	}
	fp_up = (count_reset_high_ground > count_reset_high_suspend) ? 0:1;
	printk("%s: set fp-id pull up,get gpio value = %d\n", __func__, fp_up);
	ret = select_pin_ctl(&pdev->dev, "fp_id_gpio_down");
	mdelay(5);
	if (ret)
		return -ERRORFP;
	for (i = 0; i < MAX_TIMES; i++) {
		mdelay(1);
		ret = gpio_get_value(fp_gpio);
		if (ret == 0)
			count_reset_low_ground++;
		else
			count_reset_low_suspend++;
	}
	fp_down = (count_reset_low_ground > count_reset_low_suspend) ? 0:1;
	printk("%s: set fp-id pull down,get gpio value = %d\n", __func__, fp_down);

	if (!strncmp(fp_project_name, "TD1608", 6)) {
		fp_id = FPC_FPC1245;
		printk("TD1608 direct return FPC_FPC1245: 0x%x\n", fp_id);
		ret = select_pin_ctl(&pdev->dev, "fp_id_gpio_up");
		if (ret) {
			printk("%s: set fp-id pull down error,get gpio value = %d\n", __func__, gpio_get_value(fp_gpio));
			return -ERRORFP;
		}
	} else if (!strncmp(fp_project_name, "PD1728B", 7)) {
		if ((fp_up == 0) && (fp_down == 0)) {
			fp_id = GOODIX_GF9118;
			ret = select_pin_ctl(&pdev->dev, "fp_id_gpio_down");
			printk("PD1728B return GOODIX_GF9118: 0x%x\n", fp_id);
			if (ret) {
				printk("%s: set fp-id pull down error,get gpio value = %d\n", __func__, gpio_get_value(fp_gpio));
				return -ERRORFP;
			}
		} else if ((fp_up == 1) && (fp_down == 0)) {
			fp_id = SYNAPTICS_FS9501;
			ret = select_pin_ctl(&pdev->dev, "fp_id_gpio_down");
			printk("PD1728B return SYNAPTICS_FS9501: 0x%x\n", fp_id);
			if (ret) {
				printk("%s: set fp-id pull down error,get gpio value = %d\n", __func__, gpio_get_value(fp_gpio));
				return -ERRORFP;
			}
		}
	} else if (!strncmp(fp_project_name, "PD1728", 6)) {
		fp_id = GOODIX_GF5288;
		printk("PD1728 direct return GOODIX_GF5288: 0x%x\n", fp_id);
		ret = select_pin_ctl(&pdev->dev, "fp_id_gpio_down");
		if (ret) {
			printk("%s: set fp-id pull down error,get gpio value = %d\n", __func__, gpio_get_value(fp_gpio));
			return -ERRORFP;
		}
	} else if (!strncmp(fp_project_name, "PD1730C", 7)) {
		if ((fp_up == 0) && (fp_down == 0)) {
			fp_id = GOODIX_GF3208;
			ret = select_pin_ctl(&pdev->dev, "fp_id_gpio_down");
			printk("PD1730C return GOODIX_GF3208: 0x%x\n", fp_id);
			if (ret) {
				printk("%s: set fp-id pull down error,get gpio value = %d\n", __func__, gpio_get_value(fp_gpio));
				return -ERRORFP;
			}
		} else if ((fp_up == 1) && (fp_down == 0)) {
			fp_id = GOODIX_GF5288;
			ret = select_pin_ctl(&pdev->dev, "fp_id_gpio_down");
			printk("PD1730C return GOODIX_GF5288: 0x%x\n", fp_id);
			if (ret) {
				printk("%s: set fp-id pull down error,get gpio value = %d\n", __func__, gpio_get_value(fp_gpio));
				return -ERRORFP;
			}
		}
	} else if ((!strncmp(fp_project_name, "VTD1702", 7)) || (!strncmp(fp_project_name, "PD1710", 6)) || (!strncmp(fp_project_name, "PD1709", 6))) {
		vdd_en_gpio = of_get_named_gpio(pdev->dev.of_node, "fp_id,gpio_vdd_en", 0);
		if (vdd_en_gpio < 0) {
			printk("%s: get fpc vdd_en gpio failed!\n", __func__);
			return -ERRORFP;
		}

		ret = devm_gpio_request(&pdev->dev, vdd_en_gpio, "fp_id,gpio_vdd_en");
		if (ret) {
			printk("%s: request fpc vdd_en gpio failed!\n",  __func__);
			return -ERRORFP;
		}

		gpio_direction_output(vdd_en_gpio, 1);
		usleep_range(100, 200);

		ret = select_pin_ctl(&pdev->dev, "fp_id_gpio_up");
		mdelay(5);
		if (ret)
			return -ERRORFP;

		for (i = 0; i < MAX_TIMES; i++) {
			mdelay(1);
			ret = gpio_get_value(fp_gpio);
			if (ret == 0)
				count_reset_high_ground++;
			else
				count_reset_high_suspend++;
		}
		fp_up = (count_reset_high_ground > count_reset_high_suspend) ? 0:1;

		ret = select_pin_ctl(&pdev->dev, "fp_id_gpio_down");
		mdelay(5);
		if (ret)
			return -ERRORFP;

		for (i = 0; i < MAX_TIMES; i++) {
			mdelay(1);
			ret = gpio_get_value(fp_gpio);
			if (ret == 0)
				count_reset_low_ground++;
			else
				count_reset_low_suspend++;
		}
		fp_down = (count_reset_low_ground > count_reset_low_suspend) ? 0:1;

		if ((fp_up == 0) && (fp_down == 0)) {
			fp_id = GOODIX_GF3208;
			ret = select_pin_ctl(&pdev->dev, "fp_id_gpio_down");
		} else if ((fp_up == 1) && (fp_down == 1)) {
			fp_id = FPC_FPC1022;
			ret = select_pin_ctl(&pdev->dev, "fp_id_gpio_up");
		}

		gpio_direction_output(vdd_en_gpio, 0);
		usleep_range(100, 200);

		devm_gpio_free(&pdev->dev, fp_gpio);
		devm_gpio_free(&pdev->dev, vdd_en_gpio);
    }
	return 0;
}

static int
fp_id_remove(struct platform_device *pdev)
{
	printk("fp_id  remove.\n");
	kobject_del(&kobj);
    return 0;
}

static struct of_device_id fp_id_match_table[] = {
	{ .compatible = "fp-id",},
	{},
};

static struct platform_driver fp_id_driver = {
	.probe      = fp_id_probe,
	.remove     = fp_id_remove,
	.driver = {
		.name   = "fp_id",
		.owner  = THIS_MODULE,
		.of_match_table = fp_id_match_table,
    },
};

static int __init fp_id_init(void)
{

    return platform_driver_register(&fp_id_driver);
}
module_init(fp_id_init);

static void __exit fp_id_exit(void)
{
    platform_driver_unregister(&fp_id_driver);

}
module_exit(fp_id_exit);

MODULE_AUTHOR("Xiaot BBK Corporation");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");
