/*
 * tfa98xx.c   tfa98xx codec module
 *
 * Copyright (c) 2015 NXP Semiconductors
 *
 *  Author: Sebastien Jan <sjan@baylibre.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define DEBUG

#include <linux/module.h>
#include <linux/i2c.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/version.h>
#include <linux/input.h>

#include "config.h"

#define I2C_RETRIES 50
#define I2C_RETRY_DELAY 5 /* ms */
/* TODO: remove genregs usage? */
#ifdef N1A
#include "tfa98xx_genregs_N1A12.h"
#else
#include "tfa98xx_genregs_N1C.h"
#endif
#include "tfa9891_genregs.h"

#include "tfa98xx_tfafieldnames.h"
#include "tfa_internal.h"
#include "tfa.h"
#include "tfa_service.h"
#include "tfa_container.h"
#include "tfa98xx_parameters.h"
#include "tfa98xx-debug-common.h"

#define TFA98XX_VERSION		"2.10.2"

#ifdef pr_debug
#undef pr_debug
#endif

#define pr_debug pr_info


/* Change volume selection behavior:
 * Uncomment following line to generate a profile change when updating
 * a volume control (also changes to the profile of the modified  volume
 * control)
 */
/*#define TFA98XX_ALSA_CTRL_PROF_CHG_ON_VOL	1
*/


/* Supported rates and data formats */
/*
#define TFA98XX_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | \
                       SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
                       SNDRV_PCM_RATE_48000)
*/
#define TFA98XX_RATES SNDRV_PCM_RATE_8000_48000

#define TFA98XX_FORMATS	SNDRV_PCM_FMTBIT_S16_LE

#define XMEM_TAP_ACK  0x0122
#define XMEM_TAP_READ 0x010f
#define PARAM_GET_LSMODEL	0x86

static LIST_HEAD(profile_list); /* list of user selectable profiles */

static int tfa98xx_kmsg_regs = 1;
static int tfa98xx_ftrace_regs = 1;
static int smart_pa_switch_enable = 0;

static struct tfa98xx *tfa98xx_devices[4] = {NULL, NULL, NULL, NULL};
static int tfa98xx_registered_handles = 0;
static int tfa98xx_vsteps[4]= {0,0,0,0};
static int tfa98xx_profile = 0; /* store profile */
static int tfa98xx_prof_vsteps[10] = {0}; /* store vstep per profile (single device) */
static int tfa98xx_mixer_profiles = 0; /* number of user selectable profiles */
static int tfa98xx_mixer_profile = 0; /* current mixer profile */
//static int tfa98xx_profile_status = 0;//shijianxing add
static char *dflt_prof_name = "";
module_param(dflt_prof_name, charp, S_IRUGO);

static int no_start = 0;
module_param(no_start, int, S_IRUGO);
MODULE_PARM_DESC(no_start, "do not start the work queue; for debugging via user\n");


static void tfa98xx_interrupt_restore(struct tfa98xx *tfa98xx);
static int tfa98xx_get_fssel(unsigned int rate);

static int get_profile_from_list(char *buf, int id);
static int get_profile_id_for_sr(int id, unsigned int rate);
static void tfa98xx_dsp_init(struct tfa98xx *tfa98xx);
int tfa98xx_boot_init(int force);

static struct tfa98xx_calib tfa98xx_cal = {
	.calibrate = tfa98xx_boot_init,
	.mi2s_ready = 0,
};

struct tfa98xx_rate {
	unsigned int rate;
	unsigned int fssel;
};
static struct tfa98xx *tfa98xx_priv;//shijianxing add

int tfa9891_read_reg(unsigned int address)
{
	struct tfa98xx *tfa98xx = tfa98xx_priv;
	unsigned int value = 0;
	int ret = -EINVAL;

	if (tfa98xx) {
		ret = regmap_read(tfa98xx->regmap, address, &value);
		if (!ret) {
			ret = value;
			pr_debug("[SmartPA-%d] read reg 0x%x value 0x%x\n", __LINE__, address, value);
		} else {
			pr_err("[SmartPA-%d] read reg 0x%x error %d\n", __LINE__, address, ret);
		}
	}

	return ret;
}

static struct tfa98xx_rate rate_to_fssel[] = {
	{ 8000, 0 },
	{ 11025, 1 },
	{ 12000, 2 },
	{ 16000, 3 },
	{ 22050, 4 },
	{ 24000, 5 },
	{ 32000, 6 },
	{ 44100, 7 },
	{ 48000, 8 },
};
extern enum Tfa98xx_Error tfa_dsp_cmd_id_write_read(Tfa98xx_handle_t handle,
        unsigned char module_id,unsigned char param_id, int num_bytes,
        unsigned char data[]);
extern void tfa98xx_convert_bytes2data(int num_bytes, const unsigned char bytes[],
                                       int data[]);

int tfa98xx_reset_mtp_dbg(void);
int tfa98xx_check_mtp_dbg(void);
int tfa98xx_init_dbg(char *buffer, int size);
int tfa98xx_read_freq_dbg(char *buffer, int size);
void tfa98xx_read_prars_dbg(int temp[5], unsigned char addr);

static DEFINE_MUTEX(tfa98xx_mutex);
static LIST_HEAD(tfa98xx_list);
#define list_all_tfa98xx(tfa98xx) list_for_each_entry(tfa98xx, &tfa98xx_list, list)

/* Wrapper for tfa start */
static enum tfa_error tfa98xx_tfa_start(struct tfa98xx *tfa98xx, int next_profile, int *vstep)
{
	enum tfa_error err;
	struct snd_soc_codec *codec = tfa98xx->codec;

	pr_info("[SmartPA-%d]start\n",__LINE__);
	err = tfa_start(next_profile, vstep);

	pr_info("[SmartPA-%d]start:Reg status:\n",__LINE__);
	pr_info("[SmartPA-%d] Read System Ctrl Reg value 0x%x\n", __LINE__,
	        snd_soc_read(codec, TFA98XX_SYS_CTRL));
	pr_info("[SmartPA-%d]start: Read Audio Reg value 0x%x\n", __LINE__,
	        snd_soc_read(codec, TFA98XX_I2SREG));
	pr_info("[SmartPA-%d]start: Read Status Reg value 0x%x\n", __LINE__,
	        snd_soc_read(codec, TFA98XX_STATUSREG));

	/* A cold start erases the configuration, including interrupts setting.
	 * Restore it if required
	 */
	tfa98xx_interrupt_restore(tfa98xx);
	return err;
}

#ifdef CONFIG_DEBUG_FS
static struct snd_kcontrol *mi2s_clk_control;
//static struct snd_kcontrol *mi2s_form_control;

static int find_control_set_valu(const char *ctl_name,
                                 int set_val,struct snd_kcontrol **tfa_control)
{
	struct tfa98xx *tfa98xx = tfa98xx_priv;
	struct snd_soc_codec *codec;
	struct snd_kcontrol *tmp_control;
	struct snd_ctl_elem_value control;

	if (!tfa98xx || !tfa98xx->codec) {
		pr_err("[SmartPA-%d]control_set_valu: Amplifier is NULL\n",
		       __LINE__);
		return -EINVAL;
	}

	if (!ctl_name) {
		pr_err("[SmartPA-%d]control_set_valu: control name is NULL\n",
		       __LINE__);
		return -EINVAL;
	}

	tmp_control = *tfa_control;
	codec = tfa98xx->codec;

	if (!tmp_control) {
		list_for_each_entry(tmp_control,
		                    &codec->component.card->snd_card->controls, list)
		if (strstr(tmp_control->id.name, ctl_name)) {
			break;
		}

		if (!strstr(tmp_control->id.name, ctl_name))
			tmp_control = NULL;

		pr_err("[SmartPA-%d]control_set_valu: %s %s\n", __LINE__, ctl_name,
		       tmp_control ? "found" : "not found");

		if (!tfa_control)
			return -EINVAL;

		*tfa_control = tmp_control;
	}

	snd_power_lock(codec->component.card->snd_card);
	if (tmp_control->put && tmp_control->get) {
		control.value.integer.value[0] = set_val;
		tmp_control->put(tmp_control, &control);
	}
	snd_power_unlock(codec->component.card->snd_card);

	return 0;
}

static int tfa98xx_mi2s_clk_enable(void)
{
	int ret;
	pr_info("[SmartPA-%d]mi2s_clk_enable:set Vivo MI2S Clock\n", __LINE__);
	ret = find_control_set_valu("Vivo MI2S Clock", 1, &mi2s_clk_control);
	if(ret)
		return ret;
	return 0;
}

static int tfa98xx_mi2s_clk_disable(void)
{
	int ret;
	pr_info("[SmartPA-%d]mi2s_clk_disable:set Vivo MI2S Clock\n", __LINE__);
	ret = find_control_set_valu("Vivo MI2S Clock", 0, &mi2s_clk_control);
	if(ret)
		return ret;
	return 0;
}

/* OTC reporting
 * Returns the MTP0 OTC bit value
 */
static int tfa98xx_dbgfs_otc_get(void *data, u64 *val)
{
	struct i2c_client *i2c = (struct i2c_client *)data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	struct tfa98xx_control *otc = &(handles_local[tfa98xx->handle].dev_ops.controls.otc);
	enum Tfa98xx_Error err, status;
	unsigned short value;

	mutex_lock(&tfa98xx->dsp_lock);
	status = tfa98xx_open(tfa98xx->handle);
	if (status) {
		mutex_unlock(&tfa98xx->dsp_lock);
		return -EBUSY;
	}

	err = tfa98xx_get_mtp(tfa98xx->handle, &value);
	tfa98xx_close(tfa98xx->handle);
	mutex_unlock(&tfa98xx->dsp_lock);

	if (otc->deferrable) {
		if (err != Tfa98xx_Error_Ok && err != Tfa98xx_Error_NoClock) {
			pr_err("[SmartPA-%d]dbgfs_otc_get: Unable to check DSP access: %d\n", __LINE__,err);
			return -EIO;
		} else if (err == Tfa98xx_Error_NoClock) {
			if (otc->rd_valid) {
				/* read cached value */
				*val = otc->rd_value;
				pr_debug("[SmartPA-%d]dbgfs_otc_get: Returning cached value of OTC: %llu\n", __LINE__,*val);
			} else {
				pr_info("[SmartPA-%d]dbgfs_otc_get: OTC value never read!\n",__LINE__);
				return -EIO;
			}
			return 0;
		}
	}

	*val = (value & TFA98XX_KEY2_PROTECTED_MTP0_MTPOTC_MSK)
	       >> TFA98XX_KEY2_PROTECTED_MTP0_MTPOTC_POS;
	pr_debug("tfa98xx: OTC : %d\n", value&1);

	if (otc->deferrable) {
		otc->rd_value = *val;
		otc->rd_valid = true;
	}

	return 0;
}

static int tfa98xx_dbgfs_otc_set(void *data, u64 val)
{
	struct i2c_client *i2c = (struct i2c_client *)data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	struct tfa98xx_control *otc = &(handles_local[tfa98xx->handle].dev_ops.controls.otc);
	enum Tfa98xx_Error err, status;

	if (val != 0 && val != 1) {
		pr_err("[SmartPA-%d]dbgfs_otc_set: Unexpected value %llu\n\n",__LINE__, val);
		return -EINVAL;
	}
	mutex_lock(&tfa98xx->dsp_lock);
	status = tfa98xx_open(tfa98xx->handle);
	if (status) {
		mutex_unlock(&tfa98xx->dsp_lock);
		return -EBUSY;
	}
	err = tfa98xx_set_mtp(tfa98xx->handle,
	                      (val << TFA98XX_KEY2_PROTECTED_MTP0_MTPOTC_POS)
	                      & TFA98XX_KEY2_PROTECTED_MTP0_MTPOTC_MSK,
	                      TFA98XX_KEY2_PROTECTED_MTP0_MTPOTC_MSK);
	tfa98xx_close(tfa98xx->handle);
	mutex_unlock(&tfa98xx->dsp_lock);

	if (otc->deferrable) {
		if (err != Tfa98xx_Error_Ok && err != Tfa98xx_Error_NoClock) {
			pr_err("[SmartPA-%d]dbgfs_otc_set: Unable to check DSP access: %d\n", __LINE__,err);
			return -EIO;
		} else if (err == Tfa98xx_Error_NoClock) {
			/* defer OTC */
			otc->wr_value = val;
			otc->triggered = true;
			pr_debug("[SmartPA-%d]dbgfs_otc_set: Deferring write to OTC (%d)\n", __LINE__,otc->wr_value);
			return 0;
		}
	}

	/* deferrable: cache the value for subsequent offline read */
	if (otc->deferrable) {
		otc->rd_value = val;
		otc->rd_valid = true;
	}

	pr_debug("[SmartPA-%d]dbgfs_otc_set: otc < %llu\n",__LINE__, val);

	return 0;
}

static int tfa98xx_dbgfs_mtpex_get(void *data, u64 *val)
{
	struct i2c_client *i2c = (struct i2c_client *)data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	enum Tfa98xx_Error err, status;
	unsigned short value;

	mutex_lock(&tfa98xx->dsp_lock);
	status = tfa98xx_open(tfa98xx->handle);
	if (status) {
		mutex_unlock(&tfa98xx->dsp_lock);
		return -EBUSY;
	}
	err = tfa98xx_get_mtp(tfa98xx->handle, &value);
	tfa98xx_close(tfa98xx->handle);
	mutex_unlock(&tfa98xx->dsp_lock);

	if (err != Tfa98xx_Error_Ok) {
		pr_err("[SmartPA-%d]dbgfs_mtpex_get: Unable to check DSP access: %d\n",__LINE__, err);
		return -EIO;
	}

	*val = (value & TFA98XX_KEY2_PROTECTED_MTP0_MTPEX_MSK)
	       >> TFA98XX_KEY2_PROTECTED_MTP0_MTPEX_POS;
	pr_debug("[SmartPA-%d]dbgfs_mtpex_get: MTPEX : %d\n", __LINE__,value & 2 >> 1);

	return 0;
}

static int tfa98xx_dbgfs_mtpex_set(void *data, u64 val)
{
	struct i2c_client *i2c = (struct i2c_client *)data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	struct tfa98xx_control *mtpex = &(handles_local[tfa98xx->handle].dev_ops.controls.mtpex);
	enum Tfa98xx_Error err, status;

	if (val != 0) {
		pr_err("[SmartPA-%d]dbgfs_mtpex_set: Can only clear MTPEX (0 value expected)\n",__LINE__);
		return -EINVAL;
	}

	mutex_lock(&tfa98xx->dsp_lock);
	status = tfa98xx_open(tfa98xx->handle);
	if (status) {
		mutex_unlock(&tfa98xx->dsp_lock);
		return -EBUSY;
	}
	err = tfa98xx_set_mtp(tfa98xx->handle, 0,
	                      TFA98XX_KEY2_PROTECTED_MTP0_MTPEX_MSK);
	tfa98xx_close(tfa98xx->handle);
	mutex_unlock(&tfa98xx->dsp_lock);

	if (mtpex->deferrable) {
		if (err != Tfa98xx_Error_Ok && err != Tfa98xx_Error_NoClock) {
			pr_err("[SmartPA-%d]dbgfs_mtpex_set: Unable to check DSP access: %d\n",__LINE__, err);
			return -EIO;
		} else if (err == Tfa98xx_Error_NoClock) {
			/* defer OTC */
			mtpex->wr_value = 0;
			mtpex->triggered = true;
			pr_debug("[SmartPA-%d]dbgfs_mtpex_set: Deferring write to MTPEX (%d)\n",__LINE__, mtpex->wr_value);
			return 0;
		}
	}

	pr_debug("[SmartPA-%d]dbgfs_mtpex_set: mtpex < 0\n",__LINE__);

	return 0;
}

static int tfa98xx_dbgfs_temp_get(void *data, u64 *val)
{
	struct i2c_client *i2c = (struct i2c_client *)data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	enum Tfa98xx_Error status;

	mutex_lock(&tfa98xx->dsp_lock);
	status = tfa98xx_open(tfa98xx->handle);
	if (status) {
		mutex_unlock(&tfa98xx->dsp_lock);
		return -EBUSY;
	}
	*val = tfa98xx_get_exttemp(tfa98xx->handle);
	tfa98xx_close(tfa98xx->handle);
	mutex_unlock(&tfa98xx->dsp_lock);

	return 0;
}

static int tfa98xx_dbgfs_temp_set(void *data, u64 val)
{
	struct i2c_client *i2c = (struct i2c_client *)data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	enum Tfa98xx_Error status;

	mutex_lock(&tfa98xx->dsp_lock);
	status = tfa98xx_open(tfa98xx->handle);
	if (status) {
		mutex_unlock(&tfa98xx->dsp_lock);
		return -EBUSY;
	}
	tfa98xx_set_exttemp(tfa98xx->handle, (short)val);
	tfa98xx_close(tfa98xx->handle);
	mutex_unlock(&tfa98xx->dsp_lock);

	return 0;
}

/*
 * calibration:
 * write key phrase to the 'calibration' file to trigger a new calibration
 * read the calibration file once to get the calibration result
 */
/* tfa98xx_deferred_calibration_status - called from tfaRunWaitCalibration */
void tfa98xx_deferred_calibration_status(Tfa98xx_handle_t handle, int calibrateDone)
{
	struct tfa98xx *tfa98xx = tfa98xx_devices[handle];
	struct tfa98xx_control *calib = &(handles_local[handle].dev_ops.controls.calib);

	pr_info("SmartPA calibration status\n");

	if (calib->wr_value) {
		/* a calibration was programmed from the calibration file
		 * interface
		 */
		switch (calibrateDone) {
		case 1:
			/* calibration complete ! */
			calib->wr_value = false; /* calibration over */
			calib->rd_valid = true;  /* result available */
			calib->rd_value = true;  /* result valid */
			tfa_dsp_get_calibration_impedance(tfa98xx->handle);
			wake_up_interruptible(&tfa98xx->wq);
			break;
		case 0:
			pr_info("[SmartPA-%d]calibration_status: Calibration not complete, still waiting...\n",__LINE__);
			break;
		case -1:
			pr_info("[SmartPA-%d]calibration_status: Calibration failed\n",__LINE__);
			calib->wr_value = false; /* calibration over */
			calib->rd_valid = true;  /* result available */
			calib->rd_value = false; /* result not valid */
			wake_up_interruptible(&tfa98xx->wq);
			break;
		default:
			pr_info("[SmartPA-%d]calibration_status: Unknown calibration status: %d\n",__LINE__,
			        calibrateDone);
		}
	}
}

static ssize_t tfa98xx_dbgfs_start_get(struct file *file,
                                       char __user *user_buf, size_t count,
                                       loff_t *ppos)
{
	struct i2c_client *i2c = file->private_data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	struct tfa98xx_control *calib = &(handles_local[tfa98xx->handle].dev_ops.controls.calib);
	char *str;
	int ret;

	ret = wait_event_interruptible(tfa98xx->wq, calib->wr_value == false);

	if (ret == -ERESTARTSYS) {
		/* interrupted by signal */
		return ret;
	}

	if (!calib->rd_valid)
		/* no calibration result available - skip */
		return 0;

	if (calib->rd_value) {
		/* Calibration already complete, return result */
		str = kmalloc(PAGE_SIZE, GFP_KERNEL);
		if (!str)
			return -ENOMEM;
		ret = print_calibration(tfa98xx->handle, str, PAGE_SIZE);
		if (ret < 0) {
			kfree(str);
			return ret;
		}
		ret = simple_read_from_buffer(user_buf, count, ppos, str, ret);

		pr_debug("[SmartPA-%d]dbgfs_start_get: %s",__LINE__, str);
		kfree(str);
		calib->rd_value = false;
	} else {
		/* Calibration failed, return the error code */
		const char estr[] = "-1\n";
		ret = copy_to_user(user_buf, estr, sizeof(estr));
		if (ret)
			return -EFAULT;
		ret =  sizeof(estr);
	}
	calib->rd_valid = false;
	return ret;
}
#if 0
static ssize_t tfa98xx_dbgfs_start_set(struct file *file,
                                       const char __user *user_buf,
                                       size_t count, loff_t *ppos)
{
	struct i2c_client *i2c = file->private_data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	struct tfa98xx_control *calib = &(handles_local[tfa98xx->handle].dev_ops.controls.calib);
	enum Tfa98xx_Error ret;
	char buf[32];
	const char ref[] = "please calibrate now";
	int buf_size;

	/* check string length, and account for eol */
	if (count > sizeof(ref) + 1 || count < (sizeof(ref) - 1))
		return -EINVAL;

	buf_size = min(count, (size_t)(sizeof(buf)-1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	buf[buf_size] = 0;

	/* Compare string, excluding the trailing \0 and the potentials eol */
	if (strncmp(buf, ref, sizeof(ref) - 1))
		return -EINVAL;

	/* Do not open/close tfa98xx: not required by tfa_clibrate */
	mutex_lock(&tfa98xx->dsp_lock);
	ret = tfa_calibrate(tfa98xx->handle);
	mutex_unlock(&tfa98xx->dsp_lock);

	if(ret) {
		pr_info("Calibration start failed (%d), deferring...\n", ret);
		calib->triggered = true;
	} else {
		pr_info("Calibration started\n");
	}
	calib->wr_value = true;  /* request was triggered from here */
	calib->rd_valid = false; /* result not available */
	calib->rd_value = false; /* result not valid (dafault) */

	return count;
}
#endif
static ssize_t tfa98xx_dbgfs_start_set(struct file *file,
                                       const char __user *ubuf,
                                       size_t count, loff_t *ppos)
{
	unsigned int kbuf[2];
	char *temp;
	struct tfa98xx *tfa98xx = tfa98xx_priv;
	int ret = 0;

	if (!tfa98xx) {
		pr_err("[SmartPA-%d]dbgfs_start_set: tfa98xx is NULL\n",__LINE__);
		return -1;
	}

	temp = kmalloc(count, GFP_KERNEL);
	if (!temp) {
		return -ENOMEM;
	}

	ret = copy_from_user(temp, (void __user *)ubuf, count);
	ret = sscanf(temp, "%d", &kbuf[0]);

	pr_err("[SmartPA-%d]dbgfs_start_set:cnt %d, kbuf[0] %d\n",__LINE__,(int)count, kbuf[0]);

	switch(kbuf[0]) {
	case 1:
		mutex_lock(&tfa98xx->dsp_lock);

		tfa98xx_dsp_init(tfa98xx);
		tfa98xx->dsp_init = TFA98XX_DSP_INIT_DONE;

		mutex_unlock(&tfa98xx->dsp_lock);
		break;
	case 2:
		mutex_lock(&tfa98xx->dsp_lock);
		tfa_stop();
		mutex_unlock(&tfa98xx->dsp_lock);
		break;
	case 3:
		tfa98xx_mi2s_clk_enable();
		tfa98xx_boot_init(1);
		tfa98xx_mi2s_clk_disable();
		break;
	case 4:
		tfa98xx_mi2s_clk_enable();
		msleep(10000);
		tfa98xx_mi2s_clk_disable();
		break;
	case 8:
		tfa98xx_mi2s_clk_enable();
		mutex_lock(&tfa98xx->dsp_lock);
		ret = tfa98xx_open(tfa98xx->handle);
		if (!ret) {
			ret = tfa98xx_set_mtp(tfa98xx->handle, 0,
			                      TFA98XX_KEY2_PROTECTED_MTP0_MTPEX_MSK);
			tfa98xx_close(tfa98xx->handle);
		}
		mutex_unlock(&tfa98xx->dsp_lock);
		tfa98xx_mi2s_clk_disable();
		pr_err("[SmartPA-%d]dbgfs_start_set:reset mtp \n",__LINE__);
		break;
	default:
		pr_err("[SmartPA-%d]dbgfs_start_set:no supported cmd %d\n",__LINE__, kbuf[0]);
	}
	kfree(temp);

	return count;
}

static ssize_t tfa98xx_dbgfs_r_read(struct file *file,
                                    char __user *user_buf, size_t count,
                                    loff_t *ppos)
{
	struct i2c_client *i2c = file->private_data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	char *str;
	uint16_t status;
	int ret, calibrate_done;

	mutex_lock(&tfa98xx->dsp_lock);
	ret = tfa98xx_open(tfa98xx->handle);
	if (ret) {
		mutex_unlock(&tfa98xx->dsp_lock);
		return -EBUSY;
	}

	/* Need to ensure DSP is access-able, use mtp read access for this
	 * purpose
	 */
	ret = tfa98xx_get_mtp(tfa98xx->handle, &status);
	if (ret) {
		ret = -EIO;
		goto r_c_err;
	}

	ret = tfaRunWaitCalibration(tfa98xx->handle, &calibrate_done);
	if (ret) {
		ret = -EIO;
		goto r_c_err;
	}

	str = kmalloc(PAGE_SIZE, GFP_KERNEL);
	if (!str) {
		ret = -ENOMEM;
		goto r_c_err;
	}

	switch (calibrate_done) {
	case 1:
		/* calibration complete ! */
		tfa_dsp_get_calibration_impedance(tfa98xx->handle);
		ret = print_calibration(tfa98xx->handle, str, PAGE_SIZE);
		break;
	case 0:
	case -1:
		ret = scnprintf(str, PAGE_SIZE, "%d\n", calibrate_done);
		break;
	default:
		pr_err("[SmartPA-%d]dbgfs_r_read: Unknown calibration status: %d\n",__LINE__, calibrate_done);
		ret = -EINVAL;
	}
	pr_debug("[SmartPA-%d]dbgfs_r_read: calib_done: %d - ret = %d - %s",__LINE__, calibrate_done, ret, str);

	if (ret < 0)
		goto r_err;

	ret = simple_read_from_buffer(user_buf, count, ppos, str, ret);

r_err:
	kfree(str);
r_c_err:
	tfa98xx_close(tfa98xx->handle);
	mutex_unlock(&tfa98xx->dsp_lock);
	return ret;
}

static ssize_t tfa98xx_dbgfs_version_read(struct file *file,
        char __user *user_buf, size_t count,
        loff_t *ppos)
{
	char str[] = TFA98XX_VERSION "\n";
	int ret;

	ret = simple_read_from_buffer(user_buf, count, ppos, str, sizeof(str));

	return ret;
}

static ssize_t tfa98xx_dbgfs_accounting_get(struct file *file,
        char __user *user_buf, size_t count,
        loff_t *ppos)
{
	struct i2c_client *i2c = file->private_data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	char str[255];
	int ret;
	int n = 0;

	n += snprintf(&str[n], sizeof(str)-1-n, "Wait4Src\t= %d\n",  tfa98xx->count_wait_for_source_state);
	n += snprintf(&str[n], sizeof(str)-1-n, "NOCLK\t\t= %d\n",  tfa98xx->count_noclk);

	str[n+1] = '\0'; /* in case str is not large enough */

	ret = simple_read_from_buffer(user_buf, count, ppos, str, n+1);

	return ret;
}

static ssize_t tfa98xx_dbgfs_read(struct file *file,
                                  char __user *buf, size_t count,
                                  loff_t *pos)
{
	struct tfa98xx *tfa98xx = tfa98xx_priv;
	const int size = 512;
	char buffer[size];
	int n = 0;

	pr_info("[SmartPA-%d]dbgfs_read:tfa98xx revision number:%d\n",
	        __LINE__, tfa98xx->rev);

	n += scnprintf(buffer+n, size-n, "SmartPA-0x%x %s\n",
	               tfa98xx->i2c->addr, tfa98xx->dev_state ? "OK" : "ERROR");
	buffer[n] = 0;

	return simple_read_from_buffer(buf, count, pos, buffer, n);
}

static ssize_t tfa98xx_dbgfs_check_reg_read(struct file *file,
        char __user *buf, size_t count,
        loff_t *pos)
{
	struct tfa98xx *tfa98xx = tfa98xx_priv;
	const int size = 512;
	char buffer[size];
	int val, ret;
	int n = 0;

	if (smart_pa_switch_enable &&
	    (tfa98xx->flags & TFA98XX_FLAG_TFA9890_FAM_DEV)) {
		val = tfa9891_read_reg(TFA98XX_STATUSREG);
		pr_info("[SmartPA-%d]dbgfs_check_reg_read:STATUSREG: 0x%x\n", __LINE__,val);
		/* Ignoring bit SPKS due after evaluation */
		ret = (val & 0xfbff) == 0xd05f ? 1 : 0;
	} else {
		pr_info(" SmartPA has been closed, return OK\n");
		ret = 1;
	}

	n += scnprintf(buffer+n, size-n, "SmartPA-0x%x %s\n",
	               tfa98xx->i2c->addr, ret ? "OK" : "ERROR");
	buffer[n] = 0;

	return simple_read_from_buffer(buf, count, pos, buffer, n);
}

static int tfa98xx_dbgfs_pga_gain_get(void *data, u64 *val)
{
	struct i2c_client *i2c = (struct i2c_client *)data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	int err;
	unsigned int value;

	/*	*val = TFA_GET_BF(tfa98xx->handle, SAAMGAIN);*/
	err = regmap_read(tfa98xx->regmap, TFA98XX_CTRL_SAAM_PGA, &value);
	*val = (value & TFA98XX_CTRL_SAAM_PGA_SAAMGAIN_MSK) >>
	       TFA98XX_CTRL_SAAM_PGA_SAAMGAIN_POS;
	return 0;
}

static int tfa98xx_dbgfs_pga_gain_set(void *data, u64 val)
{
	struct i2c_client *i2c = (struct i2c_client *)data;
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);
	int err;
	unsigned int value;

	value = val & 0xffff;
	if (value > 7)
		return -EINVAL;
	/*	TFA_SET_BF(tfa98xx->handle, SAAMGAIN, value);*/
	err = regmap_update_bits(tfa98xx->regmap, TFA98XX_CTRL_SAAM_PGA,
	                         TFA98XX_CTRL_SAAM_PGA_SAAMGAIN_MSK,
	                         value << TFA98XX_CTRL_SAAM_PGA_SAAMGAIN_POS);
	return err;
}

/* Direct registers access - provide register address in hex */
#define TFA98XX_DEBUGFS_REG_SET(__reg)					\
static int tfa98xx_dbgfs_reg_##__reg##_set(void *data, u64 val)		\
{									\
	struct i2c_client *i2c = (struct i2c_client *)data;		\
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);		\
	unsigned int ret, value;					\
									\
	ret = regmap_write(tfa98xx->regmap, 0x##__reg, (val & 0xffff));	\
	value = val & 0xffff;						\
	return 0;							\
}									\
static int tfa98xx_dbgfs_reg_##__reg##_get(void *data, u64 *val)	\
{									\
	struct i2c_client *i2c = (struct i2c_client *)data;		\
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);		\
	unsigned int value;						\
	int ret;							\
	ret = regmap_read(tfa98xx->regmap, 0x##__reg, &value);		\
	*val = value;							\
	return 0;							\
}									\
DEFINE_SIMPLE_ATTRIBUTE(tfa98xx_dbgfs_reg_##__reg##_fops, tfa98xx_dbgfs_reg_##__reg##_get,	\
						tfa98xx_dbgfs_reg_##__reg##_set, "0x%llx\n");

#define VAL(str) #str
#define TOSTRING(str) VAL(str)
#define TFA98XX_DEBUGFS_REG_CREATE_FILE(__reg, __name)				\
	debugfs_create_file(TOSTRING(__reg) "-" TOSTRING(__name), S_IRUGO|S_IWUGO, dbg_reg_dir,\
					i2c, &tfa98xx_dbgfs_reg_##__reg##_fops);


TFA98XX_DEBUGFS_REG_SET(00);
TFA98XX_DEBUGFS_REG_SET(01);
TFA98XX_DEBUGFS_REG_SET(02);
TFA98XX_DEBUGFS_REG_SET(03);
TFA98XX_DEBUGFS_REG_SET(04);
TFA98XX_DEBUGFS_REG_SET(05);
TFA98XX_DEBUGFS_REG_SET(06);
TFA98XX_DEBUGFS_REG_SET(07);
TFA98XX_DEBUGFS_REG_SET(08);
TFA98XX_DEBUGFS_REG_SET(09);
TFA98XX_DEBUGFS_REG_SET(0A);
TFA98XX_DEBUGFS_REG_SET(0B);
TFA98XX_DEBUGFS_REG_SET(0F);
TFA98XX_DEBUGFS_REG_SET(10);
TFA98XX_DEBUGFS_REG_SET(11);
TFA98XX_DEBUGFS_REG_SET(12);
TFA98XX_DEBUGFS_REG_SET(13);
TFA98XX_DEBUGFS_REG_SET(22);
TFA98XX_DEBUGFS_REG_SET(25);

DEFINE_SIMPLE_ATTRIBUTE(tfa98xx_dbgfs_calib_otc_fops, tfa98xx_dbgfs_otc_get,
                        tfa98xx_dbgfs_otc_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(tfa98xx_dbgfs_calib_mtpex_fops, tfa98xx_dbgfs_mtpex_get,
                        tfa98xx_dbgfs_mtpex_set, "%llu\n");
DEFINE_SIMPLE_ATTRIBUTE(tfa98xx_dbgfs_calib_temp_fops, tfa98xx_dbgfs_temp_get,
                        tfa98xx_dbgfs_temp_set, "%llu\n");

DEFINE_SIMPLE_ATTRIBUTE(tfa98xx_dbgfs_pga_gain_fops, tfa98xx_dbgfs_pga_gain_get,
                        tfa98xx_dbgfs_pga_gain_set, "%llu\n");

static const struct file_operations tfa98xx_dbgfs_calib_start_fops = {
	.open = simple_open,
	.read = tfa98xx_dbgfs_start_get,
	.write = tfa98xx_dbgfs_start_set,
	.llseek = default_llseek,
};

static const struct file_operations tfa98xx_dbgfs_r_fops = {
	.open = simple_open,
	.read = tfa98xx_dbgfs_r_read,
	.llseek = default_llseek,
};

static const struct file_operations tfa98xx_dbgfs_version_fops = {
	.open = simple_open,
	.read = tfa98xx_dbgfs_version_read,
	.llseek = default_llseek,
};

static const struct file_operations tfa98xx_dbgfs_accounting_fops = {
	.open = simple_open,
	.read = tfa98xx_dbgfs_accounting_get,
	.llseek = default_llseek,
};

static const struct file_operations tfa98xx_dbgfs_ic_fops = {
	.open = simple_open,
	.read = tfa98xx_dbgfs_read,
};

static const struct file_operations tfa98xx_dbgfs_check_reg_fops = {
	.open = simple_open,
	.read = tfa98xx_dbgfs_check_reg_read,
};

static void tfa98xx_debug_init(struct tfa98xx *tfa98xx, struct i2c_client *i2c)
{
	char name[50];
	struct dentry *dbg_reg_dir;

	scnprintf(name, MAX_CONTROL_NAME, "audio-%s-%x", i2c->name, i2c->addr);
	tfa98xx->dbg_dir = debugfs_create_dir(name, NULL);
	debugfs_create_file("OTC", S_IRUGO|S_IWUGO, tfa98xx->dbg_dir,
	                    i2c, &tfa98xx_dbgfs_calib_otc_fops);
	debugfs_create_file("MTPEX", S_IRUGO|S_IWUGO, tfa98xx->dbg_dir,
	                    i2c, &tfa98xx_dbgfs_calib_mtpex_fops);
	debugfs_create_file("TEMP", S_IRUGO|S_IWUGO, tfa98xx->dbg_dir,
	                    i2c, &tfa98xx_dbgfs_calib_temp_fops);
	debugfs_create_file("calibrate", S_IRUGO|S_IWUGO, tfa98xx->dbg_dir,
	                    i2c, &tfa98xx_dbgfs_calib_start_fops);
	debugfs_create_file("R", S_IRUGO, tfa98xx->dbg_dir,
	                    i2c, &tfa98xx_dbgfs_r_fops);
	debugfs_create_file("version", S_IRUGO, tfa98xx->dbg_dir,
	                    i2c, &tfa98xx_dbgfs_version_fops);
	debugfs_create_file("accounting", S_IRUGO, tfa98xx->dbg_dir,
	                    i2c, &tfa98xx_dbgfs_accounting_fops);
	debugfs_create_file("i2c", 0444, tfa98xx->dbg_dir,
	                    i2c, &tfa98xx_dbgfs_ic_fops);
	debugfs_create_file("check_reg", S_IRUGO, tfa98xx->dbg_dir,
	                    i2c, &tfa98xx_dbgfs_check_reg_fops);
	/* Direct registers access */
	if (tfa98xx->flags & TFA98XX_FLAG_TFA9890_FAM_DEV) {
		dbg_reg_dir = debugfs_create_dir("regs", tfa98xx->dbg_dir);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(00, STATUS);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(01, BATTERYVOLTAGE);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(02, TEMPERATURE);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(03, REVISIONNUMBER);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(04, I2SREG);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(05, BAT_PROT);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(06, AUDIO_CTR);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(07, DCDCBOOST);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(08, SPKR_CALIBRATION);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(09, SYS_CTRL);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(0A, I2S_SEL_REG);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(0B, HIDDEN_MTP_KEY2);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(0F, INTERRUPT_REG);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(10, PDM_CTRL);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(11, PDM_OUT_CTRL);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(12, PDM_DS4_R);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(13, PDM_DS4_L);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(22, CTRL_SAAM_PGA);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(25, MISC_CTRL);
	}

	if (tfa98xx->flags & TFA98XX_FLAG_TFA9897_FAM_DEV) {
		dbg_reg_dir = debugfs_create_dir("regs", tfa98xx->dbg_dir);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(00, STATUS);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(01, BATTERYVOLTAGE);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(02, TEMPERATURE);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(03, REVISIONNUMBER);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(04, I2SREG);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(05, BAT_PROT);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(06, AUDIO_CTR);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(07, DCDCBOOST);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(08, SPKR_CALIBRATION);
		TFA98XX_DEBUGFS_REG_CREATE_FILE(09, SYS_CTRL);
	}
	if (tfa98xx->flags & TFA98XX_FLAG_SAAM_AVAILABLE) {
		pr_info("SmartPA: Adding pga_gain debug interface\n");
		debugfs_create_file("pga_gain", S_IRUGO, tfa98xx->dbg_dir,
		                    tfa98xx->i2c,
		                    &tfa98xx_dbgfs_pga_gain_fops);
	}
}

static void tfa98xx_debug_remove(struct tfa98xx *tfa98xx)
{
	if (tfa98xx->dbg_dir)
		debugfs_remove_recursive(tfa98xx->dbg_dir);
}
#endif

static int tfa98xx_get_vstep(struct snd_kcontrol *kcontrol,
                             struct snd_ctl_elem_value *ucontrol)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,16,0)
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
	int mixer_profile = kcontrol->private_value;
	int profile = get_profile_id_for_sr(mixer_profile, tfa98xx->rate);
	int vstep = tfa98xx_prof_vsteps[profile];
	ucontrol->value.integer.value[0] =
	    tfacont_get_max_vstep(tfa98xx->handle, profile)
	    - vstep - 1;
	return 0;
}

static int tfa98xx_set_vstep(struct snd_kcontrol *kcontrol,
                             struct snd_ctl_elem_value *ucontrol)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,16,0)
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
	int mixer_profile = kcontrol->private_value;
	int profile = get_profile_id_for_sr(mixer_profile, tfa98xx->rate);
	int value = ucontrol->value.integer.value[0];
	int vstep = tfa98xx_prof_vsteps[profile];
	int vsteps = tfacont_get_max_vstep(tfa98xx->handle, profile);
	int new_vstep;

	pr_info("[SmartPA-%d]vstep set()\n",__LINE__);

	if (no_start != 0)
		return 0;

	if (vstep == vsteps - value - 1)
		return 0;

	new_vstep = vsteps - value - 1;

	if (new_vstep < 0)
		new_vstep = 0;

	tfa98xx_prof_vsteps[profile] = new_vstep;

#ifndef TFA98XX_ALSA_CTRL_PROF_CHG_ON_VOL
	if (profile == tfa98xx_profile) {
#endif
		/* this is the active profile, program the new vstep */
		tfa98xx_vsteps[0] = new_vstep;
		tfa98xx_vsteps[1] = new_vstep;

#ifndef TFA98XX_ALSA_CTRL_PROF_CHG_ON_VOL
	}
#endif

	pr_info("[SmartPA-%d]: vstep:%d, (control value: %d) - profile %d\n", __LINE__,new_vstep,
	        value, profile);

	return 1;
}

static int tfa98xx_info_vstep(struct snd_kcontrol *kcontrol,
                              struct snd_ctl_elem_info *uinfo)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,16,0)
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
	int mixer_profile = kcontrol->private_value;
	int profile = get_profile_id_for_sr(mixer_profile, tfa98xx->rate);
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;// TODO handles_local[dev_idx].spkr_count
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = tfacont_get_max_vstep(tfa98xx->handle, profile) - 1;
	pr_debug("[SmartPA-%d]info_vstep: vsteps count: %d [prof=%d]\n", __LINE__,tfacont_get_max_vstep(tfa98xx->handle, profile),
	         profile);
	return 0;
}

static int tfa98xx_get_profile(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = tfa98xx_mixer_profile;
	return 0;
}

static int tfa98xx_set_profile(struct snd_kcontrol *kcontrol,
                               struct snd_ctl_elem_value *ucontrol)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,16,0)
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
#else
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
#endif
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);

	int profile_count = tfa98xx_mixer_profiles;
	int profile = tfa98xx_mixer_profile;
	int new_profile = ucontrol->value.integer.value[0];
	int prof_idx;

	pr_info("SmartPA set profile.\n");

	if (no_start != 0)
		return 0;

	if (new_profile == profile)
		//	tfa98xx_profile_status = 0;
		return 0;

	if (new_profile >= profile_count)
		return 0;

	/* get the container profile for the requested sample rate */
	prof_idx = get_profile_id_for_sr(new_profile, tfa98xx->rate);
	if (prof_idx < 0) {
		pr_err("[SmartPA-%d]set_profile: sample rate [%d] not supported for this mixer profile [%d].\n", __LINE__,tfa98xx->rate, new_profile);
		return 0;
	}
	pr_debug("[SmartPA-%d]set_profile: selected container profile [%d]\n",__LINE__, prof_idx);

	/* update mixer profile */
	tfa98xx_mixer_profile = new_profile;

	/* update 'real' profile (container profile) */
	tfa98xx_profile = prof_idx;
	tfa98xx_vsteps[0] = tfa98xx_prof_vsteps[prof_idx];
	tfa98xx_vsteps[1] = tfa98xx_prof_vsteps[prof_idx];
	//tfa98xx_profile_status = 1;

	/* Flag DSP as invalidated as the profile change may invalidate the
	 * current DSP configuration. That way, further stream start can
	 * trigger a tfa_start.
	 */
	tfa98xx->dsp_init = TFA98XX_DSP_INIT_INVALIDATED;

	return 1;
}

static struct snd_kcontrol_new *tfa98xx_controls;

/* copies the profile basename (i.e. part until .) into buf */
static void get_profile_basename(char* buf, char* profile)
{
	int cp_len = 0, idx = 0;
	char *pch;

	pch = strchr(profile, '.');
	idx = pch - profile;
	cp_len = (pch != NULL) ? idx : (int) strlen(profile);
	memcpy(buf, profile, cp_len);
	buf[cp_len] = 0;
}

/* return the profile name accociated with id from the profile list */
static int get_profile_from_list(char *buf, int id)
{
	struct tfa98xx_baseprofile *bprof;

	list_for_each_entry(bprof, &profile_list, list) {
		if (bprof->item_id == id) {
			strcpy(buf, bprof->basename);
			return 0;
		}
	}

	return -1;
}

/* search for the profile in the profile list */
static int is_profile_in_list(char *profile, int len)
{
	struct tfa98xx_baseprofile *bprof;

	list_for_each_entry(bprof, &profile_list, list) {
		if (0 == strncmp(bprof->basename, profile, len))
			return 1;
	}

	return 0;
}

/*
 * for the profile with id, look if the requested samplerate is
 * supported, if found return the (container)profile for this
 * samplerate, on error or if not found return -1
 */
static int get_profile_id_for_sr(int id, unsigned int rate)
{
	int idx = 0;
	struct tfa98xx_baseprofile *bprof;

	list_for_each_entry(bprof, &profile_list, list) {
		if (id == bprof->item_id) {
			idx = tfa98xx_get_fssel(rate);
			if (idx < 0) {
				/* samplerate not supported */
				return -1;
			}

			return bprof->sr_rate_sup[idx];
		}
	}

	/* profile not found */
	return -1;
}

/* check if this profile is a calibration profile */
static int is_calibration_profile(char *profile)
{
	if (strstr(profile, ".cal") != NULL)
		return 1;
	return 0;
}

/*
 * adds the (container)profile index of the samplerate found in
 * the (container)profile to a fixed samplerate table in the (mixer)profile
 */
static int add_sr_to_profile(struct tfa98xx *tfa98xx, char *basename, int len, int profile)
{
	struct tfa98xx_baseprofile *bprof;
	int idx = 0;
	unsigned int sr = 0;

	list_for_each_entry(bprof, &profile_list, list) {
		if (0 == strncmp(bprof->basename, basename, len)) {
			/* add supported samplerate for this profile */
			sr = tfa98xx_get_profile_sr(tfa98xx->handle, profile);
			if (!sr) {
				pr_err("[SmartPA-%d]sr_to_profile: unable to identify supported sample rate for %s\n",__LINE__, bprof->basename);
				return -1;
			}

			/* get the index for this samplerate */
			idx = tfa98xx_get_fssel(sr);
			if (idx < 0 || idx >= TFA98XX_NUM_RATES) {
				pr_err("[SmartPA-%d]sr_to_profile: invalid index for samplerate %d\n",__LINE__, idx);
				return -1;
			}

			/* enter the (container)profile for this samplerate at the corresponding index */
			bprof->sr_rate_sup[idx] = profile;

			pr_debug("[SmartPA-%d]sr_to_profile: added profile:samplerate = [%d:%d] for mixer profile: %s\n",__LINE__, profile, sr, bprof->basename);
		}
	}

	return 0;
}

static int tfa98xx_info_profile(struct snd_kcontrol *kcontrol,
                                struct snd_ctl_elem_info *uinfo)
{
	char profile_name[MAX_CONTROL_NAME] = {0};
	int count = tfa98xx_mixer_profiles, err = -1;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = count;

	if (uinfo->value.enumerated.item >= count)
		uinfo->value.enumerated.item = count - 1;

	err = get_profile_from_list(profile_name, uinfo->value.enumerated.item);
	if (err != 0)
		return -EINVAL;

	strcpy(uinfo->value.enumerated.name, profile_name);

	return 0;
}

static int tfa98xx_create_controls(struct tfa98xx *tfa98xx)
{
	int prof, nprof, mix_index = 0;
	int  nr_controls = 0, id = 0;
	char *name;
	struct tfa98xx_baseprofile *bprofile;

	/* Create the following controls:
	 *  - enum control to select the active profile
	 *  - one volume control for each profile hosting a vstep
	 */

	nr_controls = 1; 	 /* Profile control */
	if (tfa98xx_dev_family(tfa98xx->handle) == 1)
		nr_controls += 1; /* Stop control */
	/* allocate the tfa98xx_controls base on the nr of profiles */
	nprof = tfaContMaxProfile(tfa98xx->handle);

	for (prof = 0; prof < nprof; prof++) {
		if (tfacont_get_max_vstep(tfa98xx->handle, prof))
			nr_controls++; /* Playback Volume control */
	}

	tfa98xx_controls = devm_kzalloc(tfa98xx->codec->dev,
	                                nr_controls * sizeof(tfa98xx_controls[0]), GFP_KERNEL);
	if(!tfa98xx_controls)
		return -ENOMEM;

	/* Create a mixer timer for selecting the active profile */
	name = devm_kzalloc(tfa98xx->codec->dev, MAX_CONTROL_NAME, GFP_KERNEL);
	if (!name)
		return -ENOMEM;
	scnprintf(name, MAX_CONTROL_NAME, "%s Profile", tfa98xx->fw.name);
	tfa98xx_controls[mix_index].name = name;
	tfa98xx_controls[mix_index].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
	tfa98xx_controls[mix_index].info = tfa98xx_info_profile;
	tfa98xx_controls[mix_index].get = tfa98xx_get_profile;
	tfa98xx_controls[mix_index].put = tfa98xx_set_profile;
	printk("[SmartPA-%d] profile control:%s \n",__LINE__,tfa98xx_controls[mix_index].name);
	// tfa98xx_controls[mix_index].private_value = profs; /* save number of profiles */
	mix_index++;

	/* create mixer items for each profile that has volume */
	for (prof = 0; prof < nprof; prof++) {
		/* create an new empty profile */
		bprofile = devm_kzalloc(tfa98xx->codec->dev, sizeof(*bprofile), GFP_KERNEL);
		if (!bprofile)
			return -ENOMEM;

		bprofile->len = 0;
		bprofile->item_id = -1;
		INIT_LIST_HEAD(&bprofile->list);

		/* copy profile name into basename until the . */
		get_profile_basename(bprofile->basename, tfaContProfileName(tfa98xx->handle, prof));
		bprofile->len = strlen(bprofile->basename);

		/*
		 * search the profile list for a profile with basename, if it is not found then
		 * add it to the list and add a new mixer control (if it has vsteps)
		 * also, if it is a calibration profile, do not add it to the list
		 */
		if (is_profile_in_list(bprofile->basename, bprofile->len) == 0 &&
		    is_calibration_profile(tfaContProfileName(tfa98xx->handle, prof)) == 0) {
			/* the profile is not present, add it to the list */
			list_add(&bprofile->list, &profile_list);
			bprofile->item_id = id++;

			pr_debug("[SmartPA-%d]: profile added [%d]: %s\n", __LINE__,bprofile->item_id, bprofile->basename);

			if (tfacont_get_max_vstep(tfa98xx->handle, prof)) {
				name = devm_kzalloc(tfa98xx->codec->dev, MAX_CONTROL_NAME, GFP_KERNEL);
				if (!name)
					return -ENOMEM;

				scnprintf(name, MAX_CONTROL_NAME, "%s %s Playback Volume",
				          tfa98xx->fw.name, bprofile->basename);

				tfa98xx_controls[mix_index].name = name;
				tfa98xx_controls[mix_index].iface = SNDRV_CTL_ELEM_IFACE_MIXER;
				tfa98xx_controls[mix_index].info = tfa98xx_info_vstep;
				tfa98xx_controls[mix_index].get = tfa98xx_get_vstep;
				tfa98xx_controls[mix_index].put = tfa98xx_set_vstep;
				tfa98xx_controls[mix_index].private_value = prof; /* save profile index */
				mix_index++;
			}
		}

		/* look for the basename profile in the list of mixer profiles and add the
		   container profile index to the supported samplerates of this mixer profile */
		add_sr_to_profile(tfa98xx, bprofile->basename, bprofile->len, prof);
	}

	/* set the number of user selectable profiles in the mixer */
	tfa98xx_mixer_profiles = id;

	return snd_soc_add_codec_controls(tfa98xx->codec,
	                                  tfa98xx_controls, mix_index);
}

static void *tfa98xx_devm_kstrdup(struct device *dev, char *buf)
{
	char *str = devm_kzalloc(dev, strlen(buf) + 1, GFP_KERNEL);
	if (!str)
		return str;
	memcpy(str, buf, strlen(buf));
	return str;
}

static int tfa98xx_append_i2c_address(struct device *dev,
                                      struct i2c_client *i2c,
                                      struct snd_soc_dapm_widget *widgets,
                                      int num_widgets,
                                      struct snd_soc_dai_driver *dai_drv,
                                      int num_dai)
{
	char buf[50];
	int i;
	int i2cbus = i2c->adapter->nr;
	int addr = i2c->addr;
	if (dai_drv && num_dai > 0) {
		switch(addr) {
		case 0x34:
			for(i = 0; i < num_dai; i++) {
				//snprintf(buf, 50, "%s-%x-%x",dai_drv[i].name, i2cbus,
				//	addr);
				snprintf(buf, 50, "%s-1",dai_drv[i].name);
				dai_drv[i].name = tfa98xx_devm_kstrdup(dev, buf);
				printk("[SmartPA-%d] dai_drv.name:%s\n",__LINE__,dai_drv[i].name);
				//snprintf(buf, 50, "%s-%x-%x",
				//			dai_drv[i].playback.stream_name,
				//			i2cbus, addr);
				snprintf(buf, 50, "%s-1",dai_drv[i].playback.stream_name);
				dai_drv[i].playback.stream_name = tfa98xx_devm_kstrdup(dev, buf);
				printk("[SmartPA-%d] dai_drv.name:%s\n",__LINE__,dai_drv[i].playback.stream_name);

				//snprintf(buf, 50, "%s-%x-%x",
				//			dai_drv[i].capture.stream_name,
				//			i2cbus, addr);
				snprintf(buf, 50, "%s-1",dai_drv[i].capture.stream_name);
				dai_drv[i].capture.stream_name = tfa98xx_devm_kstrdup(dev, buf);
				printk("[SmartPA-%d]dai_drv.name:%s\n",__LINE__,dai_drv[i].capture.stream_name);
			}
			break;
		case 0x35:
			for(i = 0; i < num_dai; i++) {
				snprintf(buf, 50, "%s-2",dai_drv[i].name);
				dai_drv[i].name = tfa98xx_devm_kstrdup(dev, buf);
				printk("[SmartPA-%d] dai_drv.name:%s\n",__LINE__,dai_drv[i].name);

				snprintf(buf, 50, "%s-2",dai_drv[i].playback.stream_name);
				dai_drv[i].playback.stream_name = tfa98xx_devm_kstrdup(dev, buf);
				printk("[SmartPA-%d] dai_drv.name:%s\n",__LINE__,dai_drv[i].playback.stream_name);

				snprintf(buf, 50, "%s-2",dai_drv[i].capture.stream_name);
				dai_drv[i].capture.stream_name = tfa98xx_devm_kstrdup(dev, buf);
				printk("[SmartPA-%d] dai_drv.name:%s\n",__LINE__,dai_drv[i].capture.stream_name);
			}

			break;
		case 0x36:
			for(i = 0; i < num_dai; i++) {
				snprintf(buf, 50, "%s-3",dai_drv[i].name);
				dai_drv[i].name = tfa98xx_devm_kstrdup(dev, buf);
				printk("[SmartPA-%d]dai_drv.name:%s\n",__LINE__,dai_drv[i].name);

				snprintf(buf, 50, "%s-3",dai_drv[i].playback.stream_name);
				dai_drv[i].playback.stream_name = tfa98xx_devm_kstrdup(dev, buf);
				printk("[SmartPA-%d] dai_drv.name:%s\n",__LINE__,dai_drv[i].playback.stream_name);

				snprintf(buf, 50, "%s-3",dai_drv[i].capture.stream_name);
				dai_drv[i].capture.stream_name = tfa98xx_devm_kstrdup(dev, buf);
				printk("[SmartPA-%d] dai_drv.name:%s\n",__LINE__,dai_drv[i].capture.stream_name);
			}

			break;
		case 0x37:
			for(i = 0; i < num_dai; i++) {
				snprintf(buf, 50, "%s-4",dai_drv[i].name);
				dai_drv[i].name = tfa98xx_devm_kstrdup(dev, buf);
				printk("[SmartPA-%d]dai_drv.name:%s\n",__LINE__,dai_drv[i].name);

				snprintf(buf, 50, "%s-4",dai_drv[i].playback.stream_name);
				dai_drv[i].playback.stream_name = tfa98xx_devm_kstrdup(dev, buf);
				printk("[SmartPA-%d] dai_drv.name:%s\n",__LINE__,dai_drv[i].playback.stream_name);

				snprintf(buf, 50, "%s-4",dai_drv[i].capture.stream_name);
				dai_drv[i].capture.stream_name = tfa98xx_devm_kstrdup(dev, buf);
				printk("[SmartPA-%d] dai_drv.name:%s\n",__LINE__,dai_drv[i].capture.stream_name);
			}

			break;
		}
	}
	/* the idea behind this is convert:
	 * SND_SOC_DAPM_AIF_IN("AIF IN", "AIF Playback", 0, SND_SOC_NOPM, 0, 0),
	 * into:
	 * SND_SOC_DAPM_AIF_IN("AIF IN", "AIF Playback-2-36", 0, SND_SOC_NOPM, 0, 0),
	 */
	if (widgets && num_widgets > 0)
		for(i = 0; i < num_widgets; i++) {
			if(!widgets[i].sname)
				continue;
			if((widgets[i].id == snd_soc_dapm_aif_in)
			   || (widgets[i].id == snd_soc_dapm_aif_out)) {
				snprintf(buf, 50, "%s-%x-%x", widgets[i].sname,
				         i2cbus, addr);
				widgets[i].sname = tfa98xx_devm_kstrdup(dev, buf);
				printk("[SmartPA-%d] dai_drv.name:%s\n",__LINE__,widgets[i].sname);
			}
		}

	return 0;
}

static int smart_pa_get_switch_mixer(struct snd_kcontrol *kcontrol,
                                     struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = smart_pa_switch_enable;
	pr_info("[SmartPA-%d]get_switch_mixer: smart pa enable %ld\n", __LINE__,
	        ucontrol->value.integer.value[0]);
	return 0;
}

static int smart_pa_put_switch_mixer(struct snd_kcontrol *kcontrol,
                                     struct snd_ctl_elem_value *ucontrol)
{
	pr_info("[SmartPA-%d]put_switch_mixer: smart pa enable %ld\n", __LINE__,
	        ucontrol->value.integer.value[0]);

	snd_soc_dapm_put_volsw(kcontrol, ucontrol);

	smart_pa_switch_enable = ucontrol->value.integer.value[0];
	return 0;
}

static const struct snd_kcontrol_new smart_pa_ctl =
    SOC_SINGLE_EXT("Switch", SND_SOC_NOPM, 0, 1, 0,
                   smart_pa_get_switch_mixer, smart_pa_put_switch_mixer);

static struct snd_soc_dapm_widget tfa98xx_dapm_widgets_common[] = {
	/* Stream widgets */
	SND_SOC_DAPM_AIF_IN("AIF IN", "AIF Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF OUT", "AIF Capture", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_OUTPUT("OUTL"),
	SND_SOC_DAPM_INPUT("AEC Loopback"),
	SND_SOC_DAPM_OUTPUT("SPKL OUT"),
	SND_SOC_DAPM_SWITCH("SmartPA", SND_SOC_NOPM, 0, 1, &smart_pa_ctl),
};

static struct snd_soc_dapm_widget tfa98xx_dapm_widgets_stereo[] = {
	SND_SOC_DAPM_OUTPUT("OUTR"),
};

static struct snd_soc_dapm_widget tfa98xx_dapm_widgets_saam[] = {
	SND_SOC_DAPM_INPUT("SAAM MIC"),
};

static struct snd_soc_dapm_widget tfa9888_dapm_inputs[] = {
	SND_SOC_DAPM_INPUT("DMIC1"),
	SND_SOC_DAPM_INPUT("DMIC2"),
	SND_SOC_DAPM_INPUT("DMIC3"),
	SND_SOC_DAPM_INPUT("DMIC4"),
};

static const struct snd_soc_dapm_route tfa98xx_dapm_routes_common[] = {
	{ "OUTL", NULL, "AIF IN" },
	{ "AIF OUT", NULL, "AEC Loopback" },
	{ "SPKL OUT", NULL, "SmartPA"},
	{ "SmartPA", "Switch", "OUTL" },

};

static const struct snd_soc_dapm_route tfa98xx_dapm_routes_saam[] = {
	{ "AIF OUT", NULL, "SAAM MIC" },
};

static const struct snd_soc_dapm_route tfa98xx_dapm_routes_stereo[] = {
	{ "OUTR", NULL, "AIF IN" },
};

static const struct snd_soc_dapm_route tfa9888_input_dapm_routes[] = {
	{ "AIF OUT", NULL, "DMIC1" },
	{ "AIF OUT", NULL, "DMIC2" },
	{ "AIF OUT", NULL, "DMIC3" },
	{ "AIF OUT", NULL, "DMIC4" },
};

static void tfa98xx_add_widgets(struct tfa98xx *tfa98xx)
{
#if LINUX_VERSION_CODE <= KERNEL_VERSION(3,18,0)
	struct snd_soc_dapm_context *dapm = &tfa98xx->codec->dapm;
#else
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(tfa98xx->codec);
#endif
	struct snd_soc_dapm_widget *widgets;
	unsigned int num_dapm_widgets = ARRAY_SIZE(tfa98xx_dapm_widgets_common);

	widgets = devm_kzalloc(&tfa98xx->i2c->dev,
	                       sizeof(struct snd_soc_dapm_widget) *
	                       ARRAY_SIZE(tfa98xx_dapm_widgets_common),
	                       GFP_KERNEL);
	if (!widgets)
		return;
	memcpy(widgets, tfa98xx_dapm_widgets_common,
	       sizeof(struct snd_soc_dapm_widget) *
	       ARRAY_SIZE(tfa98xx_dapm_widgets_common));

	tfa98xx_append_i2c_address(&tfa98xx->i2c->dev,
	                           tfa98xx->i2c,
	                           widgets,
	                           num_dapm_widgets,
	                           NULL,
	                           0);

	snd_soc_dapm_new_controls(dapm, widgets,
	                          ARRAY_SIZE(tfa98xx_dapm_widgets_common));
	snd_soc_dapm_add_routes(dapm, tfa98xx_dapm_routes_common,
	                        ARRAY_SIZE(tfa98xx_dapm_routes_common));

	if (tfa98xx->flags & TFA98XX_FLAG_STEREO_DEVICE) {
		snd_soc_dapm_new_controls(dapm, tfa98xx_dapm_widgets_stereo,
		                          ARRAY_SIZE(tfa98xx_dapm_widgets_stereo));
		snd_soc_dapm_add_routes(dapm, tfa98xx_dapm_routes_stereo,
		                        ARRAY_SIZE(tfa98xx_dapm_routes_stereo));
	}

	if (tfa98xx->flags & TFA98XX_FLAG_MULTI_MIC_INPUTS) {
		snd_soc_dapm_new_controls(dapm, tfa9888_dapm_inputs,
		                          ARRAY_SIZE(tfa9888_dapm_inputs));
		snd_soc_dapm_add_routes(dapm, tfa9888_input_dapm_routes,
		                        ARRAY_SIZE(tfa9888_input_dapm_routes));
	}

	if (tfa98xx->flags & TFA98XX_FLAG_SAAM_AVAILABLE) {
		snd_soc_dapm_new_controls(dapm, tfa98xx_dapm_widgets_saam,
		                          ARRAY_SIZE(tfa98xx_dapm_widgets_saam));
		snd_soc_dapm_add_routes(dapm, tfa98xx_dapm_routes_saam,
		                        ARRAY_SIZE(tfa98xx_dapm_routes_saam));
	}
}


/* Match tfa98xx device structure with a valid DSP handle */
/* TODO  can be removed once we pass the device struct in stead of handles
	The check in tfa98xx_register_dsp() is implicitly done in tfa_probe() /tfa98xx_cnt_slave2idx(_)
*/
static int tfa98xx_register_dsp(struct tfa98xx *tfa98xx)
{
	int i, handle = -1;
	u8 slave;

	pr_info("SmartPA register_dsp\n");

	for (i = 0; i < tfa98xx_cnt_max_device(); i++) {
		if (tfaContGetSlave(i, &slave) != Tfa98xx_Error_Ok)
			goto reg_err;
		pr_debug("[SmartPA-%d]register_dsp: i=%d - dev = 0x%x\n", __LINE__, i, slave);
		if (slave == tfa98xx->i2c->addr) {
			handle = i;
			break;
		}
	}
	if (handle != -1) {
		tfa98xx_devices[handle] = tfa98xx;
		pr_info(
		    "[SmartPA-%d]register_dsp: Registered DSP instance with handle %d\n",__LINE__,
		    handle);
		tfa98xx_registered_handles++;
		return handle;
	}
reg_err:
	pr_err("[SmartPA-%d]register_dsp: Unable to match I2C address 0x%x with a container device\n",
	       __LINE__,tfa98xx->i2c->addr);
	return -EINVAL;
}

static void tfa98xx_unregister_dsp(struct tfa98xx *tfa98xx)
{
	tfa98xx_registered_handles--;

	tfa98xx_devices[tfa98xx->handle] = NULL;
	pr_info("[SmartPA-%d]unregister_dsp: Un-registered DSP instance with handle %d\n",__LINE__,
	        tfa98xx->handle);
}


/* I2C wrapper functions */
enum Tfa98xx_Error tfa98xx_write_register16(Tfa98xx_handle_t handle,
        unsigned char subaddress,
        unsigned short value)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	struct tfa98xx *tfa98xx;
	int ret;
	int retries = I2C_RETRIES;
	static int i2c_error = 1;

	if (tfa98xx_devices[handle]) {
		tfa98xx = tfa98xx_devices[handle];
		if (!tfa98xx || !tfa98xx->regmap) {
			pr_err("[SmartPA-%d]write_register16:No regmap available\n",__LINE__);
			return Tfa98xx_Error_Bad_Parameter;
		}
retry:
		ret = regmap_write(tfa98xx->regmap, subaddress, value);
		if (ret < 0) {
			pr_warn("[SmartPA-%d]write_register16: i2c error, retries left: %d\n", __LINE__,retries);
			if (retries) {
				retries--;
				msleep(I2C_RETRY_DELAY);
				goto retry;
			}
			if (i2c_error) {
				i2c_error = 0;
			}
			return Tfa98xx_Error_Fail;
		}
		if (tfa98xx_kmsg_regs)
			pr_debug("[SmartPA-%d]write_register16:   WR reg=0x%02x, val=0x%04x %s\n",__LINE__,
			         subaddress, value,
			         ret<0? "Error!!" : "");

		if(tfa98xx_ftrace_regs)
			tfa98xx_trace_printk("\ttfa98xx: WR     reg=0x%02x, val=0x%04x %s\n",
			                     subaddress, value,
			                     ret<0? "Error!!" : "");
	} else {
		pr_err("[SmartPA-%d]write_register16: No device available\n",__LINE__);
		error = Tfa98xx_Error_Fail;
	}
	return error;
}

enum Tfa98xx_Error tfa98xx_read_register16(Tfa98xx_handle_t handle,
        unsigned char subaddress,
        unsigned short *val)
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	struct tfa98xx *tfa98xx;
	unsigned int value;
	int retries = I2C_RETRIES;
	int ret;

	if (tfa98xx_devices[handle]) {
		tfa98xx = tfa98xx_devices[handle];
		if (!tfa98xx || !tfa98xx->regmap) {
			pr_err("[SmartPA-%d]read_register16:No regmap available\n",__LINE__);
			return Tfa98xx_Error_Bad_Parameter;
		}
retry:
		ret = regmap_read(tfa98xx->regmap, subaddress, &value);
		if (ret < 0) {
			pr_warn("[SmartPA-%d]read_register16: i2c error at subaddress 0x%x, retries left: %d\n",__LINE__, subaddress, retries);
			if (retries) {
				retries--;
				msleep(I2C_RETRY_DELAY);
				goto retry;
			}
			return Tfa98xx_Error_Fail;
		}
		*val = value & 0xffff;

		if (tfa98xx_kmsg_regs)
			pr_info("[SmartPA-%d]read_register16: RD   reg=0x%02x, val=0x%04x %s\n",__LINE__,
			        subaddress, *val,
			        ret<0? "Error!!" : "");
		if (tfa98xx_ftrace_regs)
			tfa98xx_trace_printk("\ttfa98xx: RD     reg=0x%02x, val=0x%04x %s\n",
			                     subaddress, *val,
			                     ret<0? "Error!!" : "");
	} else {
		pr_err("[SmartPA-%d]read_register16: No device available\n",__LINE__);
		error = Tfa98xx_Error_Fail;
	}
	return error;
}

enum Tfa98xx_Error tfa98xx_read_data(Tfa98xx_handle_t handle,
                                     unsigned char reg,
                                     int len, unsigned char value[])
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	struct tfa98xx *tfa98xx;
	struct i2c_client *tfa98xx_client;
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
			.flags = 0,
			.len = 1,
			.buf = &reg,
		}, {
			.flags = I2C_M_RD,
			.len = len,
			.buf = value,
		},
	};

	if (tfa98xx_devices[handle] && tfa98xx_devices[handle]->i2c) {
		tfa98xx = tfa98xx_devices[handle];
		tfa98xx_client = tfa98xx->i2c;
		msgs[0].addr = tfa98xx_client->addr;
		msgs[1].addr = tfa98xx_client->addr;

		do {
			err = i2c_transfer(tfa98xx_client->adapter, msgs,
			                   ARRAY_SIZE(msgs));
			if (err != ARRAY_SIZE(msgs))
				msleep_interruptible(I2C_RETRY_DELAY);
		} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

		if (err != ARRAY_SIZE(msgs)) {
			pr_err("[SmartPA-%d]read_data: read transfer error %d\n",__LINE__,
			       err);
			error = Tfa98xx_Error_Fail;
		}

		if (tfa98xx_kmsg_regs)
			pr_debug("[SmartPA-%d]read_data: RD-DAT reg=0x%02x, len=%d\n",__LINE__,
			         reg, len);
		if (tfa98xx_ftrace_regs)
			tfa98xx_trace_printk("\t\ttfa98xx: RD-DAT reg=0x%02x, len=%d\n",
			                     reg, len);
	} else {
		pr_err("[SmartPA-%d]read_data: No device available\n",__LINE__);
		error = Tfa98xx_Error_Fail;
	}
	return error;
}

enum Tfa98xx_Error tfa98xx_write_raw(Tfa98xx_handle_t handle,
                                     int len,
                                     const unsigned char data[])
{
	enum Tfa98xx_Error error = Tfa98xx_Error_Ok;
	struct tfa98xx *tfa98xx;
	int ret;
	int retries = I2C_RETRIES;

	if (tfa98xx_devices[handle]) {
		tfa98xx = tfa98xx_devices[handle];
retry:
		ret = i2c_master_send(tfa98xx->i2c, data, len);
		if (ret < 0) {
			pr_warn("[SmartPA-%d]write_raw: i2c error, retries left: %d\n",__LINE__, retries);
			if (retries) {
				retries--;
				msleep(I2C_RETRY_DELAY);
				goto retry;
			}
		}

		if (ret == len) {
			if (tfa98xx_kmsg_regs)
				pr_debug("[SmartPA-%d]write_raw:   WR-RAW len=%d\n",__LINE__, len);
			if (tfa98xx_ftrace_regs)
				tfa98xx_trace_printk("\t\ttfa98xx: WR-RAW len=%d\n", len);
			return Tfa98xx_Error_Ok;
		}
		pr_err("[SmartPA-%d]write_raw:  WR-RAW (len=%d) Error I2C send size mismatch %d\n",__LINE__, len, ret);
		error = Tfa98xx_Error_Fail;
	} else {
		pr_err("[SmartPA-%d]write_raw: No device available\n",__LINE__);
		error = Tfa98xx_Error_Fail;
	}
	return error;
}

/* Interrupts management */

static void tfa98xx_interrupt_restore_tfa2(struct tfa98xx *tfa98xx)
{
	unsigned int base_addr_inten = TFA_FAM(tfa98xx->handle,INTENVDDS) >> 8;

	/* Write interrupt enable registers */
	regmap_write(tfa98xx->regmap, base_addr_inten + 0,
	             handles_local[tfa98xx->handle].interrupt_enable[0]);
	regmap_write(tfa98xx->regmap, base_addr_inten + 1,
	             handles_local[tfa98xx->handle].interrupt_enable[1]);
	regmap_write(tfa98xx->regmap, base_addr_inten + 2,
	             handles_local[tfa98xx->handle].interrupt_enable[2]);
}

static void tfa98xx_interrupt_enable_tfa2(struct tfa98xx *tfa98xx, bool enable)
{
	unsigned int base_addr_inten = TFA_FAM(tfa98xx->handle,INTENVDDS) >> 8;

	if (enable) {
		tfa98xx_interrupt_restore_tfa2(tfa98xx);
	} else {
		regmap_write(tfa98xx->regmap, base_addr_inten + 0, 0);
		regmap_write(tfa98xx->regmap, base_addr_inten + 1, 0);
		regmap_write(tfa98xx->regmap, base_addr_inten + 2, 0);
	}
}

/* Initial configuration of interrupt masks of devices for TFA1 family
 * Disable all interrupts by default.
 */
static void tfa98xx_interrupt_setup_tfa1(struct tfa98xx *tfa98xx)
{
	uint16_t ie_reg = 0;

	/* disable all interrupt sources */
	ie_reg = TFA98XX_INTERRUPT_REG_VDDD |
	         TFA98XX_INTERRUPT_REG_OTDD |
	         TFA98XX_INTERRUPT_REG_OVDD |
	         TFA98XX_INTERRUPT_REG_UVDD |
	         TFA98XX_INTERRUPT_REG_OCDD |
	         TFA98XX_INTERRUPT_REG_CLKD |
	         TFA98XX_INTERRUPT_REG_DCCD |
	         TFA98XX_INTERRUPT_REG_SPKD |
	         TFA98XX_INTERRUPT_REG_WDD;
	/* preserve reserved value */
	ie_reg |= 1 << 9;

	/* Store requested setup */
	handles_local[tfa98xx->handle].interrupt_enable[0] = ie_reg;
	handles_local[tfa98xx->handle].interrupt_status[0] = 0;

	pr_info("[SmartPA-%d]interrupt_setup_tfa1: Initial interrupts setup: ICR = 0x%04x\n",__LINE__, ie_reg);
}

/* Restore for 1st generation of devices */
static void tfa98xx_interrupt_restore_tfa1(struct tfa98xx *tfa98xx)
{
	unsigned int ie_reg = 0;

	regmap_read(tfa98xx->regmap, TFA98XX_INTERRUPT_REG, &ie_reg);

	if (ie_reg != handles_local[tfa98xx->handle].interrupt_enable[0]) {
		ie_reg = handles_local[tfa98xx->handle].interrupt_enable[0];

		/* Write interrupt enable registers */
		regmap_write(tfa98xx->regmap, TFA98XX_INTERRUPT_REG, ie_reg);

		pr_info("[SmartPA-%d]interrupt_restore_tfa1: Restored interrupts: ICR = 0x%04x\n",__LINE__,
		        ie_reg);
	} else {
		pr_info("[SmartPA-%d]interrupt_restore_tfa1: No interrupt restore needed\n",__LINE__);
	}

}

/* Enable for 1st generation of devices */
static void tfa98xx_interrupt_enable_tfa1(struct tfa98xx *tfa98xx, bool enable)
{
	handles_local[tfa98xx->handle].interrupt_enable[0] &= ~TFA98XX_INTERRUPT_REG_INT;
	handles_local[tfa98xx->handle].interrupt_enable[0] |= enable << TFA98XX_INTERRUPT_REG_INT_POS;

	tfa98xx_interrupt_restore_tfa1(tfa98xx);
}

static void tfa98xx_interrupt_setup_tfa2(struct tfa98xx *tfa98xx)
{
	uint16_t ie_reg;
	handles_local[tfa98xx->handle].interrupt_enable[0] = 0;
	ie_reg = 0;
	TFA_SET_BF_VALUE(tfa98xx->handle, IEMWSRC, 1, &ie_reg);
	handles_local[tfa98xx->handle].interrupt_enable[1] = ie_reg;
	handles_local[tfa98xx->handle].interrupt_enable[2] = 0;
}
/* Initial SW configuration for interrupts. Does not enable HW interrupts. */
static void tfa98xx_interrupt_setup(struct tfa98xx *tfa98xx)
{
	if (tfa98xx->flags & TFA98XX_FLAG_SKIP_INTERRUPTS)
		return;

	if (tfa98xx->flags & TFA98XX_FLAG_TFA9890_FAM_DEV)
		tfa98xx_interrupt_setup_tfa1(tfa98xx);
	else
		tfa98xx_interrupt_setup_tfa2(tfa98xx);
}

/* Restore interrupt setup in case it would be lost (at device cold-start) */
static void tfa98xx_interrupt_restore(struct tfa98xx *tfa98xx)
{
	if (tfa98xx->flags & TFA98XX_FLAG_SKIP_INTERRUPTS)
		return;

	if (tfa98xx_dev_family(tfa98xx->handle) == 2)
		tfa98xx_interrupt_restore_tfa2(tfa98xx);
	else
		tfa98xx_interrupt_restore_tfa1(tfa98xx);
}

/* global enable / disable interrupts */
static void tfa98xx_interrupt_enable(struct tfa98xx *tfa98xx, bool enable)
{
	if (tfa98xx->flags & TFA98XX_FLAG_SKIP_INTERRUPTS)
		return;

	if (tfa98xx_dev_family(tfa98xx->handle) == 2)
		tfa98xx_interrupt_enable_tfa2(tfa98xx, enable);
	else
		tfa98xx_interrupt_enable_tfa1(tfa98xx, enable);
}

/* Firmware management
 * Downloaded once only at module init
 * FIXME: may need to review that (one per instance of codec device?)
 */
static const char *fw_name = "tfa98xx.cnt";
static nxpTfaContainer_t *container;

static int tfa98xx_container_loaded(const struct firmware *cont, struct tfa98xx *tfa98xx)
{
	enum tfa_error tfa_err;
	int container_size;
	int handle;

	tfa98xx->dsp_fw_state = TFA98XX_DSP_FW_FAIL;

	pr_info("\n");

	if (!cont) {
		pr_err("[SmartPA-%d]container_loaded: Failed to read %s\n",__LINE__, fw_name);
		return -1;
	}

	printk("[SmartPA-%d]container_loaded: loaded %s - size: %zu\n",__LINE__, fw_name,
	       cont ? cont->size : 0);

	container = kzalloc(cont->size, GFP_KERNEL);
	if (!container) {
		release_firmware(cont);
		pr_err("[SmartPA-%d]container_loaded: Error allocating memory\n",__LINE__);
		return -2;
	}

	container_size = cont->size;
	memcpy(container, cont->data, container_size);
	release_firmware(cont);

	pr_debug("[SmartPA-%d]container_loaded: %.2s%.2s\n", __LINE__,container->version, container->subversion);
	pr_debug("[SmartPA-%d] container_loaded:%.8s\n", __LINE__,container->customer);
	pr_debug("[SmartPA-%d] container_loaded:%.8s\n", __LINE__,container->application);
	pr_debug("[SmartPA-%d] container_loaded:%.8s\n", __LINE__,container->type);
	pr_debug("[SmartPA-%d] container_loaded:%d ndev\n", __LINE__,container->ndev);
	pr_debug("[SmartPA-%d] container_loaded:%d nprof\n", __LINE__,container->nprof);

	tfa_err = tfa_load_cnt(container, container_size);
	if (tfa_err != tfa_error_ok) {
		pr_err("[SmartPA-%d]container_loaded: Cannot load container file, aborting\n",__LINE__);
		return -3;
	}

	/* register codec with dsp */
	tfa98xx->handle = tfa98xx_register_dsp(tfa98xx);
	if (tfa98xx->handle < 0) {
		pr_err("[SmartPA-%d]container_loaded: Cannot register with DSP, aborting\n",__LINE__);
		return -4;
	}

	if (tfa_probe(tfa98xx->i2c->addr << 1, &handle) != Tfa98xx_Error_Ok) {
		pr_err("[SmartPA-%d]container_loaded: Failed to probe  @ 0x%.2x\n", __LINE__,tfa98xx->i2c->addr);
		return -5;
	}

	/* prefix is the application name from the cnt */
	tfa_cnt_get_app_name(tfa98xx->fw.name);

	/* Override default profile if requested */
	if (strcmp(dflt_prof_name, "")) {
		unsigned int i;
		for (i = 0; i < tfaContMaxProfile(tfa98xx->handle); i++) {
			if (strcmp(tfaContProfileName(tfa98xx->handle, i),
			           dflt_prof_name) == 0) {
				tfa98xx_profile = i;
				pr_info(
				    "[SmartPA-%d]container_loaded: changing default profile to %s (%d)\n",__LINE__,
				    dflt_prof_name, tfa98xx_profile);
				break;
			}
		}
		if (i >= tfaContMaxProfile(tfa98xx->handle))
			pr_info(
			    "[SmartPA-%d]container_loaded: Default profile override failed (%s profile not found)\n",__LINE__,
			    dflt_prof_name);
	}

	tfa98xx->dsp_fw_state = TFA98XX_DSP_FW_OK;
	pr_debug("[SmartPA-%d]container_loaded: Firmware init complete\n",__LINE__);
	if (no_start != 0)
		return 0;
	/* Only controls for master device */
	if (tfa98xx->handle == 0)
		tfa98xx_create_controls(tfa98xx);

	if (tfa98xx->flags & TFA98XX_FLAG_DSP_START_ON_MUTE) {
		tfa98xx_interrupt_enable(tfa98xx, true);
		pr_debug("[SmartPA-%d]container_loaded:interrupt_enable:TRUE\n",__LINE__);
		return 0;
	}
	return 0;
}

static int tfa98xx_load_container(struct tfa98xx *tfa98xx)
{
	const struct firmware *cont = NULL;
	int ret = 0;
	tfa98xx->dsp_fw_state = TFA98XX_DSP_FW_PENDING;

	ret = request_firmware(&cont, fw_name, tfa98xx->dev);

	if (ret == 0) {
		ret = tfa98xx_container_loaded(cont, tfa98xx);
	}

	if (ret != 0) {
		pr_err("[SmartPA-%d] load fail %d\n", __LINE__, ret);
	}
	return ret;
}

static void tfa98xx_dsp_init(struct tfa98xx *tfa98xx)
{
	int ret;

	pr_info("\n");

	pr_info("[SmartPA-%d]dsp_init: dsp state:%d firmware state:%d\n",
	        __LINE__, tfa98xx->dsp_init, tfa98xx->dsp_fw_state);
	if (tfa98xx->dsp_fw_state != TFA98XX_DSP_FW_OK) {
		pr_err("[SmartPA-%d]dsp_init: Skipping tfa_start (no FW: %d)\n",__LINE__, tfa98xx->dsp_fw_state);
		return;
	}

	if(tfa98xx->dsp_init == TFA98XX_DSP_INIT_DONE) {
		pr_err("[SmartPA-%d]dsp_init: Stream already started, skipping DSP power-on\n",__LINE__);
		return;
	}

	mutex_lock(&tfa98xx->dsp_lock);

	tfa98xx->dsp_init = TFA98XX_DSP_INIT_PENDING;

	/* directly try to start DSP */
	ret = tfa98xx_tfa_start(tfa98xx, tfa98xx_profile, tfa98xx_vsteps);
	if (ret != Tfa98xx_Error_Ok) {
		tfa98xx->dsp_init = TFA98XX_DSP_INIT_FAIL;
		pr_err("[SmartPA-%d]dsp_init: tfa_start failed! (err %d)\n", __LINE__,ret);
	} else {
		/* Subsystem ready, tfa init complete */
		pr_info("[SmartPA-%d]dsp_init: SmartPA start success (%d)\n",__LINE__,
		        tfa98xx->init_count);
	}

	mutex_unlock(&tfa98xx->dsp_lock);
	return;
}

static int tfa98xx_dsp_power_on(struct tfa98xx *tfa98xx)
{
	pr_info("\n");

retry:
	if(tfa98xx->calibration) {
		/* start the DSP using the latest profile / vstep */
		mutex_lock(&tfa98xx->dsp_lock);
		if (!tfa98xx_tfa_start(tfa98xx, tfa98xx_profile,
		                       tfa98xx_vsteps)) {
			pr_info("[SmartPA-%d]dsp_power_on: SmartPA init done\n",__LINE__);
			tfa98xx->dsp_init = TFA98XX_DSP_INIT_DONE;
		}
		mutex_unlock(&tfa98xx->dsp_lock);
		return 0;
	} else {
		if (tfa98xx->need_init || tfa98xx_cal.mi2s_ready) {
			pr_info("[SmartPA-%d]dsp_power_on need_init %d mi2s_ready %d\n", __LINE__,
			        tfa98xx->need_init, tfa98xx_cal.mi2s_ready);
			tfa98xx_boot_init(1);
			tfa98xx->need_init = 0;
			tfa98xx_cal.mi2s_ready = 0;

			goto retry;
		} else {
			pr_err("[SmartPA-%d]dsp_power_on not calibration \n", __LINE__);
		}
	}

	return -1;
}

static int tfa98xx_reset_Mtp(struct tfa98xx *tfa98xx)
{
	enum Tfa98xx_Error err, status;
	int stable;

	pr_info("[SmartPA-%d]reset_Mtp\n",__LINE__);
	mutex_lock(&tfa98xx->dsp_lock);
	status = tfa98xx_open(tfa98xx->handle);
	if (status) {
		mutex_unlock(&tfa98xx->dsp_lock);
		return -EBUSY;
	}
	if (TFA_GET_BF(tfa98xx->handle, PWDN)) {
		pr_info("[SmartPA-%d]reset_Mtp() PLL in Powerdown, do coldStartup\n", __LINE__);
		tfaRunColdStartup(tfa98xx->handle, 0);
	}

	tfa98xx_dsp_system_stable(tfa98xx->handle, &stable);
	if (stable == 0) {
		pr_info("[SmartPA-%d]reset_Mtp: No Clock\n",__LINE__);
		mutex_unlock(&tfa98xx->dsp_lock);
		return Tfa98xx_Error_NoClock;
	}

	err = tfa98xx_set_mtp(tfa98xx->handle, TFA98XX_KEY2_PROTECTED_MTP0_MTPOTC,
	                      TFA98XX_KEY2_PROTECTED_MTP0_MTPOTC_MSK|TFA98XX_KEY2_PROTECTED_MTP0_MTPEX_MSK);
	tfa98xx->calibration = 0;
	tfa98xx_close(tfa98xx->handle);
	mutex_unlock(&tfa98xx->dsp_lock);

	pr_info("[SmartPA-%d]reset_Mtp: mtpex < 0\n",__LINE__);

	return 0;
}

static int tfa98xx_calibration_done(int valu,struct tfa98xx *tfa98xx)
{
	pr_info("SmartPA calibration_done\n");
	if ( !tfa98xx) {
		printk("[SmartPA-%d]calibration_done Amplifier is NULL\n",
		       __LINE__);
		return 0;
	}
	pr_info("[SmartPA-%d](calibration_done) impedance:%d ,max:%d,min:%d\n",__LINE__,valu,tfa98xx->para.imped_max,tfa98xx->para.imped_min);
	return !(valu<tfa98xx->para.imped_min || valu>tfa98xx->para.imped_max);

}

int tfa98xx_boot_init(int force)
{
	struct tfa98xx *tfa98xx = tfa98xx_priv;
	u32 imped_valu = 0;

	if (!tfa98xx)
		return -EINVAL;

	usleep_range(5000, 5000); //I2C operation must be delayed 20ms after I2S
	pr_info("SmartPA boot_init.\n");

	if (force) {
		tfa98xx_reset_Mtp(tfa98xx);
		usleep_range(5000, 5000);
	}

	tfa98xx_dsp_init(tfa98xx);

	imped_valu = tfa_get_calibration_info(tfa98xx->handle, 0);
	pr_info("[SmartPA-%d]boot_init: imped_value:%d\n",__LINE__,imped_valu);


	if (tfa98xx->dsp_init != TFA98XX_DSP_INIT_FAIL) {
		tfa98xx->calibration = tfa98xx_calibration_done(imped_valu, tfa98xx);
		tfa98xx->imped_val = imped_valu;
		if (!tfa98xx->calibration)
			pr_err("[SmartPA-%d]boot_init:calibration impedance %01d.%03d is irrational\n",
			       __LINE__,(int)imped_valu/1000,(int)imped_valu%1000);
	} else {
		tfa98xx->imped_val = 0;
		tfa98xx->calibration = 0;
	}

	if (tfa98xx->calibration) {
		pr_info("[SmartPA-%d]boot_init:calibration impedance %01d.%03d \n",
		        __LINE__, (int)imped_valu/1000, (int)imped_valu%1000);
		tfa98xx->dsp_init = TFA98XX_DSP_INIT_STOPPED;
	} else {
		tfa98xx_reset_Mtp(tfa98xx);
		usleep_range(5000, 5000);
		pr_err("[SmartPA-%d]boot_init:calibrate failed.\n", __LINE__);
		tfa98xx->dsp_init = TFA98XX_DSP_INIT_FAIL;
	}

	tfa_stop();

	return 0;
}

static void tfa98xx_interrupt(struct work_struct *work)
{
	struct tfa98xx *tfa98xx = container_of(work, struct tfa98xx, interrupt_work.work);
	unsigned int base_addr_inten = TFA_FAM(tfa98xx->handle,INTENVDDS) >> 8;
	unsigned int base_addr_ist   = TFA_FAM(tfa98xx->handle,ISTVDDS) >> 8;
	unsigned int base_addr_icl   = TFA_FAM(tfa98xx->handle,ICLVDDS) >> 8;
	//unsigned int base_addr_ipo   = TFA_FAM(tfa98xx->handle,IPOVDDS) >> 8;
	u32 out1, out2, out3;

	pr_info("[SmartPA-%d]interrupt\n",__LINE__);

	regmap_read(tfa98xx->regmap, base_addr_ist + 0, &out1);
	regmap_read(tfa98xx->regmap, base_addr_ist + 1, &out2);
	regmap_read(tfa98xx->regmap, base_addr_ist + 2, &out3);

	out1 &= handles_local[tfa98xx->handle].interrupt_enable[0];
	out2 &= handles_local[tfa98xx->handle].interrupt_enable[1];
	out3 &= handles_local[tfa98xx->handle].interrupt_enable[2];

	if (out1) {
		/* clear and enable interrupt(s) again */
		regmap_write(tfa98xx->regmap, base_addr_icl + 0, out1);
		regmap_write(tfa98xx->regmap, base_addr_inten + 0,
		             handles_local[tfa98xx->handle].interrupt_enable[0]);
	}

	if (out2) {
		/* manager wait for source state */
		if (TFA_GET_BF_VALUE(tfa98xx->handle, ISTMWSRC, out2) > 0) {
			int manwait1 = TFA_GET_BF(tfa98xx->handle, MANWAIT1);

			if (manwait1 > 0) {
				pr_info("[SmartPA-%d]interrupt: entering wait for source state\n",__LINE__);
				tfa98xx->count_wait_for_source_state++;
				/* set AMPC and AMPE to make sure the amp is enabled */
				pr_info("[SmartPA-%d]interrupt: setting AMPC and AMPE to 1 (default) \n",__LINE__);
				TFA_SET_BF(tfa98xx->handle, AMPC, 1);
				TFA_SET_BF(tfa98xx->handle, AMPE, 1);

				/* set MANSCONF here, the manager will continue if clock is there */
				TFA_SET_BF(tfa98xx->handle, MANSCONF, 1);
			} else {
				/* Now we can switch profile with internal clock it is not required to call tfa_start */

				pr_info("[SmartPA-%d]interrupt: leaving wait for source state\n",__LINE__);

				TFA_SET_BF(tfa98xx->handle, MANSCONF, 0);
			}

			if (manwait1 > 0)
				TFA_SET_BF(tfa98xx->handle, IPOMWSRC, 0);
			else
				TFA_SET_BF(tfa98xx->handle, IPOMWSRC, 1);
		}

		/* clear and enable interrupt(s) again */
		regmap_write(tfa98xx->regmap, base_addr_icl + 1, out2);
		regmap_write(tfa98xx->regmap, base_addr_inten + 1,
		             handles_local[tfa98xx->handle].interrupt_enable[1]);
	}

	if (out3) {
		/* clear and enable interrupt(s) again */
		regmap_write(tfa98xx->regmap, base_addr_icl + 2, out3);
		regmap_write(tfa98xx->regmap, base_addr_inten + 2,
		             handles_local[tfa98xx->handle].interrupt_enable[2]);
	}

}

static int tfa98xx_startup(struct snd_pcm_substream *substream,
                           struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
	unsigned int sr;
	int len, prof, nprof = tfaContMaxProfile(tfa98xx->handle), idx = 0;
	char *basename;

	if (!smart_pa_switch_enable)
		return 0;

	pr_info("SmartPA startup\n");

	/*
	 * Support CODEC to CODEC links,
	 * these are called with a NULL runtime pointer.
	 */
	if (!substream->runtime)
		return 0;
	if (no_start != 0)
		return 0;
	basename = devm_kzalloc(tfa98xx->codec->dev, MAX_CONTROL_NAME, GFP_KERNEL);
	if (!basename)
		return -ENOMEM;

	/* copy profile name into basename until the . */
	get_profile_basename(basename, tfaContProfileName(tfa98xx->handle, tfa98xx_profile));
	len = strlen(basename);

	/* loop over all profiles and get the supported samples rate(s) from
	 * the profiles with the same basename
	 */
	for (prof = 0; prof < nprof; prof++) {
		if (0 == strncmp(basename, tfaContProfileName(tfa98xx->handle, prof), len)) {

			/* Check which sample rate is supported with current profile,
			 * and enforce this.
			 */
			sr = tfa98xx_get_profile_sr(tfa98xx->handle, prof);
			if (!sr)
				pr_err("[SmartPA-%d]startup: Unable to identify supported sample rate\n",__LINE__);
			tfa98xx->rate_constraint_list[idx++] = sr;
			tfa98xx->rate_constraint.count += 1;
		}
	}
	return 0;
#if 0
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
	                                  SNDRV_PCM_HW_PARAM_RATE,
	                                  &tfa98xx->rate_constraint);
#endif
}

static int tfa98xx_set_dai_sysclk(struct snd_soc_dai *codec_dai,
                                  int clk_id, unsigned int freq, int dir)
{
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec_dai->codec);

	if (!smart_pa_switch_enable)
		return 0;

	tfa98xx->sysclk = freq;
	return 0;
}

static int tfa98xx_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
//	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(dai->codec);
//	struct snd_soc_codec *codec = dai->codec;

	if (!smart_pa_switch_enable)
		return 0;

	pr_info("\n");
	pr_debug("[SmartPA-%d]set_fmt: fmt=0x%x\n", __LINE__,fmt);
#if 0//shijianxing add:no need to set fmt
	/* Supported mode: regular I2S, slave, or PDM */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS) {
			dev_err(codec->dev, "Invalid Codec master mode\n");
			return -EINVAL;
		}
		break;
	case SND_SOC_DAIFMT_PDM:
		break;
	default:
		dev_err(codec->dev, "Unsupported DAI format %d\n",
		        fmt & SND_SOC_DAIFMT_FORMAT_MASK);
		return -EINVAL;
	}

	tfa98xx->audio_mode = fmt & SND_SOC_DAIFMT_FORMAT_MASK;
#endif
	return 0;
}

static int tfa98xx_get_fssel(unsigned int rate)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(rate_to_fssel); i++) {
		if (rate_to_fssel[i].rate == rate) {
			return rate_to_fssel[i].fssel;
		}
	}
	return -EINVAL;
}

static int tfa98xx_hw_params(struct snd_pcm_substream *substream,
                             struct snd_pcm_hw_params *params,
                             struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
	unsigned int rate;
	int prof_idx;

	if (!smart_pa_switch_enable)
		return 0;

	pr_info("\n");

	/* Supported */
	rate = params_rate(params);
	pr_debug("[SmartPA-%d]hw_params: Requested rate: %d, sample size: %d, physical size: %d\n",__LINE__,
	         rate, snd_pcm_format_width(params_format(params)),
	         snd_pcm_format_physical_width(params_format(params)));

	if (params_channels(params) > 2) {
		pr_warn("[SmartPA-%d]hw_params: Unusual number of channels: %d\n",__LINE__, params_channels(params));
	}
	if (no_start != 0)
		return 0;
	/* check if samplerate is supported for this mixer profile */
	prof_idx = get_profile_id_for_sr(tfa98xx_mixer_profile, rate);
	if (prof_idx < 0) {
		pr_err("[SmartPA-%d]hw_params: invalid sample rate %d.\n", __LINE__,rate);
		return -EINVAL;
	}
	pr_debug("[SmartPA-%d]hw_params: mixer profile:container profile = [%d:%d]\n",__LINE__, tfa98xx_mixer_profile, prof_idx);

	/* update 'real' profile (container profile) */
	tfa98xx_profile = prof_idx;

	/* update to new rate */
	tfa98xx->rate = rate;

	return 0;
}

static int tfa98xx_mute(struct snd_soc_dai *dai, int mute, int stream)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
//	int ready = 0;

	if (!smart_pa_switch_enable)
		return 0;

	pr_info("\n");
	pr_info("[SmartPA-%d]: mute state: %d\n", __LINE__,mute);

	if (!(tfa98xx->flags & TFA98XX_FLAG_DSP_START_ON_MUTE)) {
		pr_err("[SmartPA-%d] dsp no mute.\n",__LINE__);
		return 0;
	}

	if (no_start) {
		pr_err("[SmartPA-%d] no_start parameter set no tfa_start or tfa_stop, returning\n",__LINE__);
		return 0;
	}

	if (mute) {
		/* stop DSP only when both playback and capture streams
		 * are deactivated
		 */
#if 0
		if (stream == SNDRV_PCM_STREAM_PLAYBACK)
			tfa98xx->pstream = 0;
		else
			tfa98xx->cstream = 0;
		if (tfa98xx->pstream != 0)// || tfa98xx->cstream != 0)
			return 0;
#endif
		if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
			pr_info("[SmartPA-%d]mute:playback stream\n",__LINE__);
			mutex_lock(&tfa98xx->dsp_lock);
			if (tfa98xx->pstream) {
				tfa_stop();
				tfa98xx->dsp_init = TFA98XX_DSP_INIT_STOPPED;
			}
			mutex_unlock(&tfa98xx->dsp_lock);
			usleep_range(5000, 5000);
			tfa98xx->pstream = 0;
		} else
			tfa98xx->cstream = 0;

	} else {
		pr_info("[SmartPA-%d]mute: sample rate %d.\n", __LINE__,tfa98xx->rate);
		/* smart_pa_clk_check(); */
#if 0
		if (stream == SNDRV_PCM_STREAM_PLAYBACK)
			tfa98xx->pstream = 1;
		else
			tfa98xx->cstream = 1;
#endif
		if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
			pr_info("[SmartPA-%d]mute:playback stream\n",__LINE__);
			usleep_range(10000, 10000);
			/* Start DSP */
			if (!tfa98xx_dsp_power_on(tfa98xx))
				tfa98xx->pstream = 1;
		} else
			tfa98xx->cstream = 1;
	}

	return 0;
}

static const struct snd_soc_dai_ops tfa98xx_dai_ops = {
	.startup = tfa98xx_startup,
	.set_fmt = tfa98xx_set_fmt,
	.set_sysclk = tfa98xx_set_dai_sysclk,
	.hw_params = tfa98xx_hw_params,
	.mute_stream = tfa98xx_mute,
};

static struct snd_soc_dai_driver tfa98xx_dai[] = {
	{
		.name = "SmartPA",
		.base = TFA98XX_TDM_CONFIG0 - 1,
		.id = 1,
		.playback = {
			.stream_name = "SmartPA Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = TFA98XX_RATES,
			.formats = TFA98XX_FORMATS,
		},
		.capture = {
			.stream_name = "SmartPA Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = TFA98XX_RATES,
			.formats = TFA98XX_FORMATS,
		},
		.ops = &tfa98xx_dai_ops,
		.symmetric_rates = 1,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
		.symmetric_channels = 1,
		.symmetric_samplebits = 1,
#endif
	},
	/*	{
			.name = "SmartPA",
			.base = TFA98XX_TDM_CONFIG0 - 1,
			.id = 1,
			.playback = {
				.stream_name = "SmartPA Playback",
				.channels_min = 1,
				.channels_max = 2,
				.rates = TFA98XX_RATES,
				.formats = TFA98XX_FORMATS,
			},
			.capture = {
				 .stream_name = "SmartPA Capture",
				 .channels_min = 1,
				 .channels_max = 2,
				 .rates = TFA98XX_RATES,
				 .formats = TFA98XX_FORMATS,
			 },
			.ops = &tfa98xx_dai_ops,
			.symmetric_rates = 1,
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,14,0)
			.symmetric_channels = 1,
			.symmetric_samplebits = 1,
	#endif
		},
	*/
};

static int tfa98xx_probe(struct snd_soc_codec *codec)
{
	/* struct i2c_client *i2c = to_i2c_client(codec->dev); */
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);
	int ret;

	pr_info("SmartPA probe.\n");

	tfa98xx->rate_constraint.list = &tfa98xx->rate_constraint_list[0];
	tfa98xx->rate_constraint.count =
	    ARRAY_SIZE(tfa98xx->rate_constraint_list);

	/* setup work queue, will be used to initial DSP on first boot up */
	tfa98xx->tfa98xx_wq = create_singlethread_workqueue("tfa98xx");
	if (!tfa98xx->tfa98xx_wq)
		return -ENOMEM;

	INIT_DELAYED_WORK(&tfa98xx->interrupt_work, tfa98xx_interrupt);

	tfa98xx->codec = codec;
	ret = tfa98xx_load_container(tfa98xx);
	pr_info("[SmartPA-%d]probe: Container loading completed: %d\n", __LINE__,ret);

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,16,0)
	codec->control_data = tfa98xx->regmap;
	ret = snd_soc_codec_set_cache_io(codec, 8, 16, SND_SOC_REGMAP);
	if (ret != 0) {
		pr_err("[SmartPA-%d]probe: Failed to set cache I/O: %d\n", __LINE__,ret);
		return ret;
	}
#endif

	tfa98xx->calib = &tfa98xx_cal;

	mutex_lock(&tfa98xx_mutex);
	list_add(&tfa98xx->list, &tfa98xx_list);
	mutex_unlock(&tfa98xx_mutex);

	tfa98xx_add_widgets(tfa98xx);

	pr_info("SmartPA codec registered (%s)",
	        tfa98xx->fw.name);
	// disable I2SDOE
	snd_soc_update_bits(codec, TFA98XX_I2SREG,
	                    TFA98XX_I2SREG_I2SDOE_MSK, 0);

	snd_soc_dapm_ignore_suspend(dapm, "SPKL OUT");
	snd_soc_dapm_sync(dapm);

	pr_info("[SmartPA-%d]_probe leave.\n",__LINE__);
	return ret;
}

static int tfa98xx_remove(struct snd_soc_codec *codec)
{
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
	pr_info("[SmartPA-%d]remove\n",__LINE__);

	cancel_delayed_work_sync(&tfa98xx->interrupt_work);
	if (tfa98xx->tfa98xx_wq)
		destroy_workqueue(tfa98xx->tfa98xx_wq);

	return 0;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,16,0)
struct regmap *tfa98xx_get_regmap(struct device *dev)
{
	struct tfa98xx *tfa98xx = dev_get_drvdata(dev);

	return tfa98xx->regmap;
}
#endif
static struct snd_soc_codec_driver soc_codec_dev_tfa98xx = {
	.probe =	tfa98xx_probe,
	.remove =	tfa98xx_remove,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,16,0)
	.get_regmap = tfa98xx_get_regmap,
#endif
};


static bool tfa98xx_writeable_register(struct device *dev, unsigned int reg)
{
	/* enable read access for all registers */
	return 1;
}

static bool tfa98xx_readable_register(struct device *dev, unsigned int reg)
{
	/* enable read access for all registers */
	return 1;
}

static bool tfa98xx_volatile_register(struct device *dev, unsigned int reg)
{
	/* enable read access for all registers */
	return 1;
}

static const struct regmap_config tfa98xx_regmap = {
	.reg_bits = 8,
	.val_bits = 16,

	.max_register = TFA98XX_MAX_REGISTER,
	.writeable_reg = tfa98xx_writeable_register,
	.readable_reg = tfa98xx_readable_register,
	.volatile_reg = tfa98xx_volatile_register,
	.cache_type = REGCACHE_NONE,
};


static void tfa98xx_irq_tfa2(struct tfa98xx *tfa98xx)
{
	unsigned int base_addr_inten = TFA_FAM(tfa98xx->handle,INTENVDDS) >> 8;
	unsigned int base_addr_ist   = TFA_FAM(tfa98xx->handle,ISTVDDS) >> 8;
	//unsigned int base_addr_icl   = TFA_FAM(tfa98xx->handle,ICLVDDS) >> 8;
	//unsigned int base_addr_ipo   = TFA_FAM(tfa98xx->handle,IPOVDDS) >> 8;
	u32 en1, en2, en3;
	u32 out1 = 0, out2 = 0, out3 = 0;

	pr_info("[SmartPA-%d]irq 2\n",__LINE__);

	regmap_read(tfa98xx->regmap, base_addr_inten + 0, &en1);
	regmap_read(tfa98xx->regmap, base_addr_inten + 1, &en2);
	regmap_read(tfa98xx->regmap, base_addr_inten + 2, &en3);

	regmap_read(tfa98xx->regmap, base_addr_ist + 0, &out1);
	regmap_read(tfa98xx->regmap, base_addr_ist + 1, &out2);
	regmap_read(tfa98xx->regmap, base_addr_ist + 2, &out3);

	pr_info("[SmartPA-%d]: interrupt1: 0x%.4x (enabled: 0x%.4x)\n",__LINE__, out1, en1);
	pr_info("[SmartPA-%d]: interrupt2: 0x%.4x (enabled: 0x%.4x)\n", __LINE__,out2, en2);
	pr_info("[SmartPA-%d]: interrupt3: 0x%.4x (enabled: 0x%.4x)\n",__LINE__, out3, en3);

	out1 &= en1;
	out2 &= en2;
	out3 &= en3;

	en1 = handles_local[tfa98xx->handle].interrupt_enable[0] ^ out1;
	en2 = handles_local[tfa98xx->handle].interrupt_enable[1] ^ out2;
	en3 = handles_local[tfa98xx->handle].interrupt_enable[2] ^ out3;

	regmap_write(tfa98xx->regmap, base_addr_inten + 0, en1);
	regmap_write(tfa98xx->regmap, base_addr_inten + 1, en2);
	regmap_write(tfa98xx->regmap, base_addr_inten + 2, en3);

	if (out1 || out2 || out3)
		queue_delayed_work(tfa98xx->tfa98xx_wq, &tfa98xx->interrupt_work, 0);
}

static void __tfa98xx_irq(struct tfa98xx *tfa98xx)
{
	uint16_t val;
	uint16_t ie = handles_local[tfa98xx->handle].interrupt_status[0];

	val = snd_soc_read(tfa98xx->codec, TFA98XX_STATUSREG);

	pr_info("[SmartPA-%d]: interrupt: 0x%04x (enabled: 0x%04x)\n",__LINE__, val, ie);
#ifdef DEBUG
	if (!(val & ie)) {
		unsigned int ireg;
		/* interrupt triggered while all interrupt sources supposedly
		 * disabled
		 */
		ireg = snd_soc_read(tfa98xx->codec, TFA98XX_INTERRUPT_REG);
		pr_info("[SmartPA-%d]: ICR: 0x%04x\n", __LINE__,ireg);
	}
#endif
	val &= ie;
}

static irqreturn_t tfa98xx_irq(int irq, void *data)
{
	struct tfa98xx *tfa98xx = data;

	if (tfa98xx_dev_family(tfa98xx->handle) == 2)
		tfa98xx_irq_tfa2(tfa98xx);
	else
		__tfa98xx_irq(tfa98xx);

	return IRQ_HANDLED;
}

static int tfa98xx_ext_reset(struct tfa98xx *tfa98xx)
{
	if (tfa98xx && gpio_is_valid(tfa98xx->reset_gpio)) {
		gpio_set_value_cansleep(tfa98xx->reset_gpio, 1);
		usleep_range(1000, 1000);
		gpio_set_value_cansleep(tfa98xx->reset_gpio, 0);
		usleep_range(1000, 1000);
	}

	/* Give time for power to be stable */

	return 0;
}

static int tfa98xx_parse_dt(struct device *dev, struct tfa98xx *tfa98xx,
                            struct device_node *np)
{
	struct tfa98xx_para para;
	const char *config = NULL;
	int temp,ret = 0;

	tfa98xx->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if (tfa98xx->reset_gpio < 0)
		pr_info("[SmartPA-%d]parse_dt: No reset GPIO provided, will not HW reset device\n",__LINE__);

	tfa98xx->irq_gpio =  of_get_named_gpio(np, "irq-gpio", 0);
	if (tfa98xx->irq_gpio < 0)
		pr_info("[SmartPA-%d]parse_dt: No IRQ GPIO provided.\n",__LINE__);

	ret = of_property_read_string(np, "vivo,tfa98xx-config", &config);
	if (config)
		fw_name = config;

	ret = of_property_read_u32(np, "vivo,tfa98xx-impedance-min", &temp);
	para.imped_min= ( !ret)? (int)temp:500;

	ret = of_property_read_u32(np, "vivo,tfa98xx-impedance-max", &temp);
	para.imped_max= ( !ret)? (int)temp:1100;

	ret = of_property_read_u32(np, "vivo,tfa98xx-frequency-min", &temp);
	para.fres_min= ( !ret)? (int)temp:600;

	ret = of_property_read_u32(np, "vivo,tfa98xx-frequency-max", &temp);
	para.fres_max= ( !ret)? (int)temp:1000;

	ret = of_property_read_u32(np, "vivo,tfa98xx-Qt-min", &temp);
	para.Qt= ( !ret)? (int)temp:125;

	tfa98xx->para = para;

	return 0;
}

static ssize_t tfa98xx_reg_write(struct file *filp, struct kobject *kobj,
                                 struct bin_attribute *bin_attr,
                                 char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tfa98xx *tfa98xx = dev_get_drvdata(dev);

	if (count != 1) {
		pr_debug("[SmartPA-%d]parse_dt: invalid register address",__LINE__);
		return -EINVAL;
	}

	tfa98xx->reg = buf[0];

	return 1;
}

static ssize_t tfa98xx_rw_write(struct file *filp, struct kobject *kobj,
                                struct bin_attribute *bin_attr,
                                char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tfa98xx *tfa98xx = dev_get_drvdata(dev);
	u8 *data;
	int ret;
	int retries = I2C_RETRIES;

	data = kmalloc(count+1, GFP_KERNEL);
	if (data == NULL) {
		pr_debug("[SmartPA-%d]rw_write: can not allocate memory\n",__LINE__);
		return  -ENOMEM;
	}

	data[0] = tfa98xx->reg;
	memcpy(&data[1], buf, count);

retry:
	ret = i2c_master_send(tfa98xx->i2c, data, count+1);
	if (ret < 0) {
		pr_warn("[SmartPA-%d]rw_write: i2c error, retries left: %d\n", __LINE__,retries);
		if (retries) {
			retries--;
			msleep(I2C_RETRY_DELAY);
			goto retry;
		}
	}

	kfree(data);
	return ret;
}

static ssize_t tfa98xx_rw_read(struct file *filp, struct kobject *kobj,
                               struct bin_attribute *bin_attr,
                               char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct tfa98xx *tfa98xx = dev_get_drvdata(dev);
	struct i2c_msg msgs[] = {
		{
			.addr = tfa98xx->i2c->addr,
			.flags = 0,
			.len = 1,
			.buf = &tfa98xx->reg,
		},
		{
			.addr = tfa98xx->i2c->addr,
			.flags = I2C_M_RD,
			.len = count,
			.buf = buf,
		},
	};
	int ret;
	int retries = I2C_RETRIES;
retry:
	ret = i2c_transfer(tfa98xx->i2c->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret < 0) {
		pr_warn("[SmartPA-%d]rw_read: i2c error, retries left: %d\n", __LINE__,retries);
		if (retries) {
			retries--;
			msleep(I2C_RETRY_DELAY);
			goto retry;
		}
		return ret;
	}
	/* ret contains the number of i2c messages send */
	return 1 + ((ret > 1) ? count : 0);
}

static struct bin_attribute dev_attr_rw = {
	.attr = {
		.name = "rw",
		.mode = S_IRUSR | S_IWUSR,
	},
	.size = 0,
	.read = tfa98xx_rw_read,
	.write = tfa98xx_rw_write,
};

static struct bin_attribute dev_attr_reg = {
	.attr = {
		.name = "reg",
		.mode = S_IWUSR,
	},
	.size = 0,
	.read = NULL,
	.write = tfa98xx_reg_write,
};

void tfa98xx_get_client(struct i2c_client **client, unsigned char addr)
{
	struct tfa98xx *tfa98xx,*prev = NULL;
	list_all_tfa98xx(tfa98xx) {
		prev = tfa98xx;
		if (tfa98xx->i2c->addr * 2 == addr)
			break;
	}
	tfa98xx = prev;
	if (!tfa98xx)
		return;
	*client = tfa98xx->i2c;
	return;
}

static int tfa98xx_check_Mtp(struct tfa98xx *tfa98xx)
{
	enum Tfa98xx_Error err, status;
	unsigned short value;
	unsigned short mtpex = 0;

	mutex_lock(&tfa98xx->dsp_lock);
	status = tfa98xx_open(tfa98xx->handle);
	if (status) {
		mutex_unlock(&tfa98xx->dsp_lock);
		return -EBUSY;
	}

	status = TFA_GET_BF(tfa98xx->handle, PWDN);
	if (status) {
		pr_info("[SmartPA-%d]check_Mtp: PLL in Powerdown, do coldStartup\n", __LINE__);
		tfaRunColdStartup(tfa98xx->handle, 0);
	}
	err = tfa98xx_get_mtp(tfa98xx->handle, &value);
	if (status) {
		pr_info("[SmartPA-%d]check_Mtp: now need to powerdown tfa\n", __LINE__);
		tfa_stop();
	}
	tfa98xx_close(tfa98xx->handle);
	mutex_unlock(&tfa98xx->dsp_lock);

	if (err != Tfa98xx_Error_Ok) {
		pr_err("[SmartPA-%d]check_Mtp: Unable to check DSP access: %d\n", __LINE__, err);
		return -EIO;
	}
	mtpex = (value & 2) >> 1;
	pr_debug("[SmartPA-%d]check_Mtp: value: 0x%x, MTPEX : %d\n", __LINE__, value, mtpex);
	return mtpex;
}

int tfa98xx_reset_mtp_dbg()
{
	struct tfa98xx *tfa98xx;
	pr_info("[SmartPA-%d]reset_mtp_dbg: reset mtp Enter.\n", __LINE__);
	tfa98xx_mi2s_clk_enable();
	list_all_tfa98xx(tfa98xx) {
		tfa98xx_reset_Mtp(tfa98xx);
		tfa_stop();
		pr_info("[SmartPA-%d]reset_mtp_dbg:%s()\n",__LINE__, tfa98xx->name);
	}
	tfa98xx_mi2s_clk_disable();
	pr_info("[SmartPA-%d]reset_mtp_dbg: reset mtp complete.\n", __LINE__);
	return 0;
}
int tfa98xx_init_dbg(char *buffer, int size)
{
	int ret = 0, n = 0, done = 1;
	int course = 0, fine = 0;
	struct tfa98xx *tfa98xx = NULL;
	ret= tfa98xx_mi2s_clk_enable();
	if (!ret) {
		ret = tfa98xx_boot_init(1);
	}
	ret = tfa98xx_mi2s_clk_disable();
	list_all_tfa98xx(tfa98xx) {
		done &= tfa98xx_calibration_done(tfa98xx->imped_val, tfa98xx);
		course = tfa98xx->imped_val / 1000;
		fine = tfa98xx->imped_val % 1000;
		n += scnprintf(buffer + n, size - n,
		               "current status:[%s]\n mono: impedance %02d.%02d ohm, valid range(%02d.%02d ~ %02d.%02d ohm). \n",
		               tfa98xx->name, course, fine, tfa98xx->para.imped_min / 1000,
		               tfa98xx->para.imped_min % 1000, tfa98xx->para.imped_max / 1000,
		               tfa98xx->para.imped_max % 1000);
	}
	n += scnprintf(buffer + n, size - n, "Calibrate result: %s\n", done ? "OKAY(impedance ok)." : "ERROR!");
	buffer[n] = 0;
	return done;
}

#if 0
static int tfa98xx_dsp_get_speaker_freq(struct tfa98xx *tfa98xx)
{
	unsigned char bytes[3 * 141];
	int data[141];
	int error = 0;
	int fRes,fResInit;
	mutex_lock(&tfa98xx->dsp_lock);
	error = tfa_dsp_cmd_id_write_read(tfa98xx->handle, 1,
	                                  PARAM_GET_LSMODEL, 423, bytes);
	mutex_unlock(&tfa98xx->dsp_lock);
	if(error)
		pr_err("[SmartPA-%d]speaker_freq: error\n", __LINE__);
	tfa98xx_convert_bytes2data(sizeof(bytes), bytes, data);
	fRes = data[135];
	fResInit = data[136];
	pr_info("[SmartPA-%d]speaker_freq::f0 = %d fRes = %d\n",__LINE__,fRes,fResInit);
	return fRes;
}
#endif
int tfa98xx_read_freq_dbg(char *buffer, int size)
{
#if 0
	struct tfa98xx *tfa98xx = NULL;
	int n = 0;
	int fRes;
	int i = 0,j = 0;
	pr_info("[SmartPA-%d]freq_dbg:",__LINE__);
	list_all_tfa98xx(tfa98xx) {
		pr_info("%s:tfa98xx fRes min = %d fRes min = %d\n",
		        tfa98xx->name, tfa98xx->para.fres_min, tfa98xx->para.fres_max);
		while ((i++ < 15) && (j < 5)) {
			fRes = tfa98xx_dsp_get_speaker_freq(tfa98xx);
			if ((fRes < tfa98xx->para.fres_min) || (fRes > tfa98xx->para.fres_max))
				j = 0;
			else
				j++;
			n += scnprintf(buffer+n, size-n, "f0 = %d \n", fRes);
			pr_info("%s:tfa98xx fRes = %d i = %d j = %d\n",
			        tfa98xx->name, fRes, i, j);
			msleep(500);
		}
		if(j == 5)
			n += scnprintf(buffer+n, size-n, "PASS\n");
		else
			n += scnprintf(buffer+n, size-n, "FAIL\n");
	}
	buffer[n] = 0;
#endif
	const char msg[] = "Not support!\n";
	memcpy(buffer, msg, sizeof(msg));
	return 0;
}

int tfa98xx_check_mtp_dbg()
{
	struct tfa98xx *tfa98xx = NULL;
	int ret = 1;

	tfa98xx_mi2s_clk_enable();
	list_all_tfa98xx(tfa98xx) {
		ret &= tfa98xx_check_Mtp(tfa98xx);
	}
	tfa98xx_mi2s_clk_disable();

	return ret;
}

void tfa98xx_read_prars_dbg(int temp[5], unsigned char addr)
{
	struct tfa98xx *tfa98xx,*prev = NULL;
	list_all_tfa98xx(tfa98xx) {
		prev = tfa98xx;
		if (tfa98xx->i2c->addr * 2 == addr)
			break;
	}
	tfa98xx = prev;
	if (!tfa98xx)
		return;
	temp[0] = tfa98xx->para.fres_max;
	temp[1] = tfa98xx->para.fres_min;
	temp[2] = tfa98xx->para.Qt;
	temp[3] = tfa98xx->para.imped_max;
	temp[4] = tfa98xx->para.imped_min;
}

static int tfa98xx_i2c_probe(struct i2c_client *i2c,
                             const struct i2c_device_id *id)
{
	struct snd_soc_dai_driver *dai;
	struct tfa98xx *tfa98xx;
	struct device_node *np = i2c->dev.of_node;
	int irq_flags;
	unsigned int reg;
	int ret;

	pr_info("SmartPA i2c probe\n");

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		pr_err("[SmartPA-%d]i2c_probe: check_functionality failed\n",__LINE__);
		return -EIO;
	}
	//shijianxing add
	ret = dev_set_name(&i2c->dev, "%s", "tfa98xx");
	if (ret||ERR_PTR(ret))
		return ret;

	tfa98xx = devm_kzalloc(&i2c->dev, sizeof(struct tfa98xx),
	                       GFP_KERNEL);
	if (tfa98xx == NULL)
		return -ENOMEM;

	tfa98xx->dev = &i2c->dev;
	tfa98xx->i2c = i2c;
	tfa98xx->name = dev_name(&i2c->dev);
	tfa98xx->dsp_init = TFA98XX_DSP_INIT_STOPPED;
	tfa98xx->rate = 48000; /* init to the default sample rate (48kHz) */
	tfa98xx->dev_state = 0;/* check out bad tfa98xx IC @fanyongxiang*/

	tfa98xx->regmap = devm_regmap_init_i2c(i2c, &tfa98xx_regmap);
	if (IS_ERR(tfa98xx->regmap)) {
		ret = PTR_ERR(tfa98xx->regmap);
		pr_err("[SmartPA-%d]i2c_probe: Failed to allocate register map: %d\n",__LINE__,
		       ret);
		goto err;
	}

	i2c_set_clientdata(i2c, tfa98xx);
	mutex_init(&tfa98xx->dsp_lock);
	init_waitqueue_head(&tfa98xx->wq);

	if (np) {
		ret = tfa98xx_parse_dt(&i2c->dev, tfa98xx, np);
		if (ret) {
			pr_err("[SmartPA-%d]i2c_probe: Failed to parse DT node\n",__LINE__);
			goto err;
		}
		if (no_start)
			tfa98xx->irq_gpio = -1;
	} else {
		tfa98xx->reset_gpio = -1;
		tfa98xx->irq_gpio = -1;
	}

	if (gpio_is_valid(tfa98xx->reset_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, tfa98xx->reset_gpio,
		                            GPIOF_OUT_INIT_LOW, "TFA98XX_RST");
		if (ret)
			goto err;
	}

	if (gpio_is_valid(tfa98xx->irq_gpio)) {
		ret = devm_gpio_request_one(&i2c->dev, tfa98xx->irq_gpio,
		                            GPIOF_DIR_IN, "TFA98XX_INT");
		if (ret)
			goto err;
	}

	/* Power up! */
	tfa98xx_ext_reset(tfa98xx);

	if (no_start == 0) {
		ret = regmap_read(tfa98xx->regmap, 0x03, &reg);
		if (ret < 0) {
			pr_err("[SmartPA-%d]i2c_probe: Failed to read Revision register: %d\n",__LINE__,
			       ret);
		} else {
			tfa98xx->dev_state = 1;
		}

		tfa98xx->rev = reg & 0xFF;
		switch (reg & 0xff) {
		case 0x88: /* tfa9888 */
			pr_info("[SmartPA-%d]i2c_probe: chipid detected\n",__LINE__);
			tfa98xx->flags |= TFA98XX_FLAG_STEREO_DEVICE;
			tfa98xx->flags |= TFA98XX_FLAG_MULTI_MIC_INPUTS;
			break;
		case 0x80: /* tfa9890 */
		case 0x81: /* tfa9890 */
			pr_info("[SmartPA-%d]i2c_probe: chipid detected\n",__LINE__);
			tfa98xx->flags |= TFA98XX_FLAG_DSP_START_ON_MUTE;
			tfa98xx->flags |= TFA98XX_FLAG_SKIP_INTERRUPTS;
			tfa98xx->flags |= TFA98XX_FLAG_TFA9890_FAM_DEV;
			break;
		case 0x92: /* tfa9891 */
			pr_info("[SmartPA-%d]i2c_probe: chipid detected\n",__LINE__);
			tfa98xx->flags |= TFA98XX_FLAG_DSP_START_ON_MUTE;
			tfa98xx->flags |= TFA98XX_FLAG_SAAM_AVAILABLE;
			tfa98xx->flags |= TFA98XX_FLAG_TAPDET_AVAILABLE;
			tfa98xx->flags |= TFA98XX_FLAG_TFA9890_FAM_DEV;
			break;
		case 0x97:
			pr_info("[SmartPA-%d]i2c_probe: chipid detected\n",__LINE__);
			tfa98xx->flags |= TFA98XX_FLAG_DSP_START_ON_MUTE;
			tfa98xx->flags |= TFA98XX_FLAG_SKIP_INTERRUPTS;
			tfa98xx->flags |= TFA98XX_FLAG_TFA9897_FAM_DEV;
			break;
		default:
			tfa98xx->dev_state = 0;
			pr_info("[SmartPA-%d]i2c_probe: Unsupported device revision (0x%x)\n",__LINE__,reg & 0xff);
			pr_info("[SmartPA-%d]i2c_probe: default detect TFA9891\n",__LINE__);
			tfa98xx->flags |= TFA98XX_FLAG_DSP_START_ON_MUTE;
			tfa98xx->flags |= TFA98XX_FLAG_SAAM_AVAILABLE;
			tfa98xx->flags |= TFA98XX_FLAG_TAPDET_AVAILABLE;
			tfa98xx->flags |= TFA98XX_FLAG_TFA9890_FAM_DEV;
		}
	}

	/* Modify the stream names, by appending the i2c device address.
	 * This is used with multicodec, in order to discriminate the devices.
	 * Stream names appear in the dai definition and in the stream  	 .
	 * We create copies of original structures because each device will
	 * have its own instance of this structure, with its own address.
	 */
	dai = devm_kzalloc(&i2c->dev, sizeof(tfa98xx_dai), GFP_KERNEL);
	if (!dai)
		return -ENOMEM;
	memcpy(dai, tfa98xx_dai, sizeof(tfa98xx_dai));

	tfa98xx_append_i2c_address(&i2c->dev,
	                           i2c,
	                           NULL,
	                           0,
	                           dai,
	                           ARRAY_SIZE(tfa98xx_dai));

	ret = snd_soc_register_codec(&i2c->dev,
	                             &soc_codec_dev_tfa98xx, dai,
	                             ARRAY_SIZE(tfa98xx_dai));

	if (ret < 0) {
		pr_err("[SmartPA-%d]i2c_probe:Failed to register Amplifier: %d\n",__LINE__, ret);
		goto err_off;
	}

	if (gpio_is_valid(tfa98xx->irq_gpio) &&
	    !(tfa98xx->flags & TFA98XX_FLAG_SKIP_INTERRUPTS)) {
		/* register irq handler */
		irq_flags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
		ret = devm_request_threaded_irq(&i2c->dev,
		                                gpio_to_irq(tfa98xx->irq_gpio),
		                                NULL, tfa98xx_irq, irq_flags,
		                                "tfa98xx", tfa98xx);
		if (ret != 0) {
			pr_err("[SmartPA-%d]i2c_probe: Failed to request IRQ %d: %d\n",__LINE__,
			       gpio_to_irq(tfa98xx->irq_gpio), ret);
			goto err_off;
		}
		tfa98xx_interrupt_setup(tfa98xx);
	} else {
		pr_info("[SmartPA-%d]i2c_probe: Skipping IRQ registration\n",__LINE__);
		/* disable feature support if gpio was invalid */
		tfa98xx->flags |= TFA98XX_FLAG_SKIP_INTERRUPTS;
	}
	tfa98xx_priv = tfa98xx;//shijianxing add

#ifdef CONFIG_DEBUG_FS
	tfa98xx_debug_init(tfa98xx, i2c);
#endif
	/* Register the sysfs files for climax backdoor access */
	ret = device_create_bin_file(&i2c->dev, &dev_attr_rw);
	if (ret)
		pr_info("[SmartPA-%d]i2c_probe: error creating sysfs files\n",__LINE__);
	ret = device_create_bin_file(&i2c->dev, &dev_attr_reg);
	if (ret)
		pr_info("[SmartPA-%d]i2c_probe: error creating sysfs files\n",__LINE__);

	pr_info("[SmartPA-%d]i2c_probe: Probe completed successfully!\n", __LINE__);

	ret = tfa98xx_debug_probe(i2c);
	if (ret != 0) {
		pr_err("[SmartPA-%d]i2c_probe:Failed to probe debug interface: %d\n",__LINE__,
		       ret);
	}

	pr_info("[SmartPA-%d]i2c_probe: leave.\n",__LINE__);

	return 0;

err_off:
	tfa98xx_unregister_dsp(tfa98xx);
err:
	return ret;
}

static int tfa98xx_i2c_remove(struct i2c_client *i2c)
{
	struct tfa98xx *tfa98xx = i2c_get_clientdata(i2c);

	pr_info("[SmartPA-%d]i2c_remove\n",__LINE__);

	cancel_delayed_work_sync(&tfa98xx->interrupt_work);

	device_remove_bin_file(&i2c->dev, &dev_attr_reg);
	device_remove_bin_file(&i2c->dev, &dev_attr_rw);
#ifdef CONFIG_DEBUG_FS
	tfa98xx_debug_remove(tfa98xx);
#endif

	tfa98xx_unregister_dsp(tfa98xx);

	snd_soc_unregister_codec(&i2c->dev);

	if (gpio_is_valid(tfa98xx->irq_gpio))
		devm_gpio_free(&i2c->dev, tfa98xx->irq_gpio);
	if (gpio_is_valid(tfa98xx->reset_gpio))
		devm_gpio_free(&i2c->dev, tfa98xx->reset_gpio);

	return 0;
}

static const struct i2c_device_id tfa98xx_i2c_id[] = {
	{ "tfa98xx", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tfa98xx_i2c_id);

#ifdef CONFIG_OF
static struct of_device_id tfa98xx_dt_match[] = {
	{ .compatible = "nxp,tfa98xx" },
	{ .compatible = "nxp,tfa9890" },
	{ .compatible = "nxp,tfa9891" },
	{ .compatible = "nxp,tfa9888" },
	{ },
};
#endif

static struct i2c_driver tfa98xx_i2c_driver = {
	.driver = {
		.name = "tfa98xx",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tfa98xx_dt_match),
	},
	.probe =    tfa98xx_i2c_probe,
	.remove =   tfa98xx_i2c_remove,
	.id_table = tfa98xx_i2c_id,
};

static int trace_level = 1;
module_param(trace_level, int, S_IRUGO);
MODULE_PARM_DESC(trace_level, "TFA98xx debug trace level (0=off, bits:1=verbose,2=regdmesg,3=regftrace).");
static int __init tfa98xx_i2c_init(void)
{
	int ret = 0;

	printk("SmartPA driver version %s\n", TFA98XX_VERSION);

	/* Enable debug traces */
	tfa_verbose(trace_level);
	tfa98xx_kmsg_regs = trace_level & 2;
	tfa98xx_ftrace_regs = trace_level & 4;

	ret = i2c_add_driver(&tfa98xx_i2c_driver);

	return ret;
}
module_init(tfa98xx_i2c_init);


static void __exit tfa98xx_i2c_exit(void)
{
	i2c_del_driver(&tfa98xx_i2c_driver);

	kfree(container);
}
module_exit(tfa98xx_i2c_exit);

MODULE_DESCRIPTION("ASoC TFA98XX driver");
MODULE_LICENSE("GPL");
