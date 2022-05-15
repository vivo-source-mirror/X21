#include <linux/module.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/printk.h>
#include <linux/ratelimit.h>
#include <linux/debugfs.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/wait.h>
#include <linux/regulator/consumer.h>
#include <linux/i2c.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/clk.h>

#include "vivo-codec-common.h"

#include "ak4376.h"
#define ak4376_DEV_NAME "ak4376"
#define ak4376_PHYS_NAME "ak4376_phys_dev"
static bool ak4376_available = false;

enum {
	ak4375_VERSION_0,
	ak4375_VERSION_A,
	AK4376_VERSION_0,
	ak4376_VERSION_NONE

} ak4376_VERSION;
int ak4376_version = ak4376_VERSION_NONE;

#ifndef BBK_IQOO_AUDIO_DEBUG
#define BBK_IQOO_AUDIO_DEBUG
#endif

#ifdef BBK_IQOO_AUDIO_DEBUG
static struct dentry *ak4376_debugfs_root;
static struct dentry *ak4376_debugfs_reg;
static struct dentry *ak4376_debugfs_i2c;

static u8 ak4376_regs[] = {
	0X00,
	0X01,
	0X02,
	0X03,
	0X04,
	0X05,
	0X06,
	0X07,
	0X08,
	0X09,
	0X0A,
	0X0B,
	0X0C,
	0X0D,
	0X0E,
	0X0F,
	0X10,
	0X11,
	0X12,
	0X13,
	0X14,
	0X15,
	0X24,
};

#endif

static u8 ak4376_reg_disable[6][2] = {
	{ 0x03,0x00 },
	{ 0x01,0x31 }, //shijianxing: PowerDown CP2--->LDO1P/LDO1N--->CP1
	{ 0x02,0x00 },
	{ 0x01,0x01 }, //shijianxing:PowerDown LDO1P/LDO1N
	{ 0xff,0xff },
	{ 0x01,0x00 }  //shijianxing: PowerDown CP1
};
static u8 ak4375_reg_enable[34][2] = {
	{ 0x06,0x00 },
	{ 0x07,0x21 },
	{ 0x08,0x00 },
	{ 0x09,0x00 },
	{ 0x0A,0x00 },
	{ 0x0B,0x19 },
	{ 0x0C,0x19 },
	{ 0x0D,0x73 },
	{ 0x0E,0x01 },
	{ 0x0F,0x00 },
	{ 0x10,0x00 },
	{ 0x11,0x00 },
	{ 0x12,0x27 },
	{ 0x14,0x09 },
	{ 0x15,0x20 },
	{ 0x24,0x00 },
	{ 0x13,0x01 },
	{ 0x00,0x01 },
	{ 0x05,0x0A },
	{ 0xff,0xff },
	{ 0x01,0x01 },
	{ 0xff,0xff },
	{ 0xff,0xff },
	{ 0xff,0xff },
	{ 0xff,0xff },
	{ 0x01,0x31 },
	{ 0xff,0xff },
	{ 0x02,0x01 },
	{ 0x01,0x33 },
	{ 0xff,0xff },
	{ 0xff,0xff },
	{ 0xff,0xff },
	{ 0x03,0x03 },
	{ 0x04,0x00 }
};
//shijianxing add for AK4376
static u8 ak4376_reg_enable[36][2] = {
	{ 0x26,0x20 },	//DAC adjustment1 init
	{ 0x2A,0x05 },	//DAC adjustment2 init
	{ 0x06,0x00 },
	{ 0x07,0x21 },
	{ 0x08,0x00 },
	{ 0x09,0x00 },
	{ 0x0A,0x00 },
	{ 0x0B,0x11 },
	{ 0x0C,0x19 },
	{ 0x0D,0x0B },
	{ 0x0E,0x01 },
	{ 0x0F,0x00 },
	{ 0x10,0x00 },
	{ 0x11,0x00 },
	{ 0x12,0x27 },
	{ 0x14,0x09 },
	{ 0x15,0x40 },
	{ 0x24,0x00 },
	{ 0x13,0x01 },
	{ 0x00,0x01 },
	{ 0x05,0x09 },
	{ 0xff,0xff },
	{ 0x01,0x01 },	//shijianxing M:PowerUp CP1--->LDO1P/LDO1N--->CP2
	{ 0xff,0xff },
	{ 0xff,0xff },
	{ 0xff,0xff },
	{ 0xff,0xff },
	{ 0x01,0x31 },	//shijianxing:PowerUp LDO1P/LDO1N
	{ 0xff,0xff },
	{ 0x02,0x01 },
	{ 0x01,0x33 }, //shijianxing:PowerUp CP2
	{ 0xff,0xff },
	{ 0xff,0xff },
	{ 0xff,0xff },
	{ 0x03,0x03 },
	{ 0x04,0x00 }
};
//shijianxing add:Low Power Mode for AK4376
static u8 ak4376_normal_reg_enable[36][2] = {
	{ 0x26,0x20 },	//DAC adjustment1 init
	{ 0x2A,0x05 },	//DAC adjustment2 init
	{ 0x06,0x00 },
	{ 0x07,0x21 },
	{ 0x08,0x00 },
	{ 0x09,0x00 },
	{ 0x0A,0x00 },
	{ 0x0B,0x11 },
	{ 0x0C,0x19 },
	{ 0x0D,0x0B },
	{ 0x0E,0x01 },
	{ 0x0F,0x00 },
	{ 0x10,0x00 },
	{ 0x11,0x00 },
	{ 0x12,0x27 },
	{ 0x14,0x09 },
	{ 0x15,0x40 },
	{ 0x24,0x40 },	//DSMLPM=1
	{ 0x13,0x01 },
	{ 0x00,0x01 },
	{ 0x05,0x0a },	//48K
	{ 0xff,0xff },
	{ 0x01,0x01 },	//shijianxing M:PowerUp CP1--->LDO1P/LDO1N--->CP2
	{ 0xff,0xff },
	{ 0xff,0xff },
	{ 0xff,0xff },
	{ 0xff,0xff },
	{ 0x01,0x31 },
	{ 0xff,0xff },
	{ 0x02,0x11 },	//LPMode=1
	{ 0x01,0x33 },
	{ 0xff,0xff },
	{ 0xff,0xff },
	{ 0xff,0xff },
	{ 0x03,0x03 },
	{ 0x04,0x00 }
};
static int reg_addr_tbl[] = {
	0x05,
	0x08,
	0x0e,
	0x10,
	0x12,
	0x14,
	0x15
};

static int reg_value_tbl[][7] = {
	{0x0a, 0x0a, 0x01, 0x00, 0x4f, 0x09, 0x01}, //ak4376 48k 16bit
	{0x0a, 0x0a, 0x01, 0x00, 0x27, 0x09, 0x00}, //ak4376 48k 24bit
	{0x0e, 0x0e, 0x01, 0x00, 0x13, 0x09, 0x00}, //ak4376 96k 24bit
	{0x12, 0x12, 0x01, 0x00, 0x09, 0x09, 0x00}, //ak4376 192k 24bit
	{0x0a, 0x0a, 0x01, 0x00, 0x4f, 0x09, 0x21}, //ak4376a 48k 16bit
	{0x09, 0x09, 0x00, 0x18, 0x92, 0x09, 0x30}, //ak4376a 44.1k 24bit
	{0x0a, 0x0a, 0x01, 0x00, 0x27, 0x09, 0x20}, //ak4376a 48k 24bit
	{0x0d, 0x0d, 0x00, 0x18, 0x92, 0x04, 0x30}, //ak4376a 88.2k 24bit
	{0x0e, 0x0e, 0x01, 0x01, 0x27, 0x04, 0x20}, //ak4376a 96k 24bit
	{0x71, 0x71, 0x00, 0x18, 0x92, 0x04, 0x30}, //ak4376a 176.4k 24bit
	{0x72, 0x72, 0x01, 0x03, 0x27, 0x04, 0x20}, //ak4376a 192k 24bit
	//shijianxing add for ak4376 for slave mode
	{0x09, 0x00, 0x00, 0x18, 0x92, 0x09, 0x50}, //ak4376 44.1k 24bit
	{0x0a, 0x00, 0x00, 0x09, 0x3f, 0x09, 0x52}, //ak4376 48k 24bit
	{0x0d, 0x00, 0x00, 0x18, 0x92, 0x04, 0x50}, //ak4376 88.2k 24bit
	{0x0e, 0x00, 0x00, 0x09, 0x3f, 0x04, 0x52}, //ak4376 96k 24bit
	{0x71, 0x00, 0x00, 0x18, 0x92, 0x04, 0x50}, //ak4376 176.4k 24bit
	{0x72, 0x00, 0x00, 0x09, 0x3f, 0x04, 0x52}, //ak4376 192k 24bit
	//shijianxing add for ak4376 for master mode
	{0x0a, 0x00, 0x01, 0x00, 0x27, 0x09, 0x40}, //ak4376 48k 24bit
	{0x0e, 0x00, 0x01, 0x01, 0x27, 0x04, 0x40}, //ak4376 96k 24bit
	{0x72, 0x00, 0x01, 0x03, 0x27, 0x04, 0x40}, //ak4376 192k 24bit

	// 44.1k series 24bit platform master mode
	{0x09, 0x00, 0x01, 0x00, 0x27, 0x09, 0x40}, //ak4376 44.1k 24bit
	{0x0d, 0x00, 0x01, 0x01, 0x27, 0x04, 0x40}, //ak4376 88.2k 24bit
	{0x71, 0x00, 0x01, 0x04, 0x31, 0x04, 0x40}, //ak4376 176.4k 24bit

	{0x0a, 0x00, 0x01, 0x00, 0x4f, 0x09, 0x41}, //ak4376 48k 16bit
};

struct ak4376_data {
	struct i2c_client *client;
	char *driver_name;
	//int vdd_en_gpio;
	int rst_gpio;
	int volume;
	struct regulator *hifi_1v8_regulator;
	struct clk *hifi_mclk;
	struct vivo_codec_function *fun;
	struct mutex lock;
	int reg_size;
	int *reg_table;
};
static struct ak4376_data *ak4376_data;

static int ak4376_phys_open(struct inode *inode, struct file *filep)
{
	filep->private_data = inode->i_private;
	return 0;
}
int ak4376_phys_release (struct inode *inode, struct file *filep)
{
	int ret =0;
	return ret;
}
long ak4376_phys_ioctl (struct file *filep, unsigned int cmd, unsigned long args)
{
	int ret = 0;

	pr_info("[HiFi-%d]ioctl \n",__LINE__);
	return (long)ret;
}


static struct file_operations ak4376_phys_fops = {
	.open = ak4376_phys_open,
	.unlocked_ioctl = ak4376_phys_ioctl,
	.release = ak4376_phys_release,
};

static struct miscdevice ak4376_phys_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = ak4376_PHYS_NAME,
	.fops = &ak4376_phys_fops,
};
static int ak4376_i2c_read_byte(u8 reg)
{
	int ret = 0;
	u8 buf = 0xff;

	ret = i2c_master_send(ak4376_data->client,&reg,1);
	if(ret < 0) {
		pr_err("[HiFi-%d] i2c send cmd error reg=%d \n",__LINE__,reg);
		return ret;
	}
	ret = i2c_master_recv(ak4376_data->client,&buf,1);
	if(ret <0 ) {
		pr_err("[HiFi-%d] i2c recv error \n ",__LINE__);
		return ret;
	}
	//pr_err("%s: reg = 0x%x value = 0x%x\n", __func__, reg, buf);

	return buf;

}
static int ak4376_i2c_write_byte(u8 reg,u8 data)
{
	int ret = 0;
	u8 cmd[2];
	static int i2c_err = 1;

	cmd[0] = reg;
	cmd[1] = data;

	ret = i2c_master_send(ak4376_data->client,cmd,sizeof(cmd));
	if (ret < 1) {
		pr_info("[HiFi-%d] i2c write send error cmd[0]=%d,cmd[1]=%d\n",__LINE__,cmd[0],cmd[1]);
		if (i2c_err) {
			i2c_err = 0;
		}
	}
	pr_info("[HiFi-%d]: reg = 0x%x value = 0x%x\n", __LINE__, reg, data);

	return ret;
}

int ak4376_reset(int reset_pin)
{
	gpio_direction_output(reset_pin,0);
	usleep_range(2000, 3000);
	gpio_direction_output(reset_pin,1);
	usleep_range(10000, 10000);
	return 0;
}
int ak4376_mute(int mute)
{

	return 0;
}

static int do_reg_write_check(u8 reg, u8 val)
{
	u8 reg_val = 0;
	int retry = 5;
	int ret = 0;
	char logbuf[100];
	reg_val = ak4376_i2c_read_byte(reg);
	while((val != reg_val)&&(retry > 0)) {
		ak4376_i2c_write_byte(reg, val);
		reg_val = ak4376_i2c_read_byte(reg);
		retry --;
	}
	if(retry == 5)
		ret = 0;
	else if((retry >= 0)&&(val == reg_val)) {
		sprintf(logbuf,"Write 0x%2x to 0x%2x success, retry %d times\n",
		        val, reg, 5-retry);
		ret = 0;
	} else if(!retry) {
		sprintf(logbuf,"Write 0x%2x to 0x%2x failed, retry 5 times\n",
		        val, reg);
		ret = -1;
	}
	return ret;
}
static int ak4376_setting_special_params(struct ak4376_params *params)
{
	int ret = 0;

	pr_info("[HiFi-%d] params->mode %d\n",__LINE__,params->mode);
	if (params->mode < 0) {
		pr_err("[HiFi-%d] no params to set\n", __LINE__);
		return 0;
	}

	ret = params->mode;
	return ret;
}

int ak4376_enable(struct audio_params *params, bool enable)
{
	int i, j;
	int ret = 0;
	int format = params->pcm_format;
	int sample_rate = params->rate;
	int reg_value, reg_addr;
	int mode_sel = 0;
	int hifi_mode_status;
	int *reg_vals;

	if(ak4376_data->client == NULL) {
		pr_err("[HiFi-%d] client is NULL \n ",__LINE__);
		return -EFAULT;
	}
	if(!ak4376_available) {
		pr_err("[HiFi-%d] not avail \n ",__LINE__);
		return 0;
	}

	if (params->private_params)
		hifi_mode_status = ak4376_setting_special_params((struct ak4376_params *)(params->private_params));

	switch (params->rate) {
	case 44100:
		sample_rate = 0;
		break;
	case 48000:
		sample_rate = 1;
		break;
	case 88200:
		sample_rate = 2;
		break;
	case 96000:
		sample_rate = 3;
		break;
	case 176400:
		sample_rate = 4;
		break;
	case 192000:
		sample_rate = 5;
		break;
	default:
		sample_rate = 1;
		break;
	}

	if (ak4376_version == AK4376_VERSION_0 && ((params->i2s_format &
	        SND_SOC_DAIFMT_MASTER_MASK) == SND_SOC_DAIFMT_CBS_CFS)) {
		switch (params->rate) {
		case 48000:
			sample_rate = 6;
			break;
		case 96000:
			sample_rate = 7;
			break;
		case 192000:
			sample_rate = 8;
			break;
		case 44100:
			sample_rate = 9;
			break;
		case 88200:
			sample_rate = 10;
			break;
		case 176400:
			sample_rate = 11;
		default:
			break;
		}
	}

	pr_info("[HiFi-%d]:enable %d, mode %d, format = %d, sample_rate = %d params->rate = %d\n",
	        __LINE__,(int)enable, hifi_mode_status, format, sample_rate, params->rate);
	mutex_lock(&ak4376_data->lock);
	if (enable&&(hifi_mode_status == 2)) {//shijianxing add: HiFi Mode enable
		if (ak4376_data->hifi_mclk) {
			clk_prepare_enable(ak4376_data->hifi_mclk);
			pr_info("[HiFi-%d] MCLK prepare.\n",__LINE__);
		}
		usleep_range(2000, 2000);
		ak4376_reset(ak4376_data->rst_gpio);

		if (ak4376_version == ak4375_VERSION_A) {
			if (format == SNDRV_PCM_FORMAT_S24_LE) {
				if ((sample_rate >= 0) && (sample_rate <= 5))
					mode_sel = sample_rate + 5;
				else
					mode_sel = 6;
			} else
				mode_sel = 4;
		} else if (ak4376_version == AK4376_VERSION_0) {
			if (format == SNDRV_PCM_FORMAT_S24_LE) {
				if ((sample_rate >= 0) && (sample_rate < 12))
					mode_sel = sample_rate + 11;//shijianxing add
				else
					mode_sel = 11;//shijianxing add for default 48K
			} else
				mode_sel = 23;//shijianxing add for 48K 16bits
		} else {
			if (format == SNDRV_PCM_FORMAT_S24_LE) {
				if (sample_rate == 1)
					mode_sel = 1;
				else if (sample_rate == 3)
					mode_sel = 2;
				else if (sample_rate == 5)
					mode_sel = 3;

			} else
				mode_sel = 0;
		}
		pr_info("[HiFi-%d]:mode_sel = %d,ak4376_version = %d\n",__LINE__, mode_sel,ak4376_version);
		/*
		*shijianxing add: must be set PMHPL and PMHPR bit to 0 before changing the mode
		*/
		if (ak4376_version == AK4376_VERSION_0) {
			ret = ak4376_i2c_write_byte(0x03,0x00);
			if(ret < 0) {
				pr_err("[HiFi-%d]:write regsister 0x03 failed\n",__LINE__);
				goto end;
			}
		}

		for(i=0; i<36; i++) {
			if (ak4376_version == AK4376_VERSION_0) {
				reg_addr = ak4376_reg_enable[i][0];
				reg_value = ak4376_reg_enable[i][1];
			} else {
				reg_addr = ak4375_reg_enable[i][0];
				reg_value = ak4375_reg_enable[i][1];
			}
			for (j = 0; j < 7; j++) {
				if (reg_addr_tbl [j] == reg_addr) {
					reg_value = reg_value_tbl[mode_sel][j];
					break;
				}
			}
			//shijianxing add:volume setting by project dtsi,default:0x0B,0dB
			if (reg_addr == 0x0d && ak4376_data->volume > 0) {
				reg_value = ak4376_data->volume;
				pr_info("[HiFi-%d]:volume = 0x%x\n",__LINE__, reg_value);
			}

			if (reg_addr == 0xff) {
				usleep_range(2000, 2000);
				continue;
			}

			for (j = 0; j < ak4376_data->reg_size; j++) {
				reg_vals = ak4376_data->reg_table;
				if (reg_vals[ 2 * j] == reg_addr) {
					reg_value = reg_vals[ 2 * j + 1];
					pr_info("[HiFi-%d]: + new addr 0x%x val 0x%x.\n",__LINE__,reg_addr, reg_value);
					break;
				}
			}

			ret = ak4376_i2c_write_byte(reg_addr,reg_value);
			if(ret < 0) {
				pr_err("[HiFi-%d]:check i2c addr= 0x%x,value= 0x%x\n",__LINE__,reg_addr, reg_value);
				goto end;
			}
			ret = do_reg_write_check(reg_addr,reg_value);
			if(ret < 0) {
				pr_err("[HiFi-%d]:check i2c addr= 0x%x,value= 0x%x\n",__LINE__,reg_addr, reg_value);
				goto end;
			}
		}
		//msleep(26);//shijianxing add for HPL/HPR Amplifier power on time,25.9ms@44.1k,23.9ms@other samplerates
		ret = 0;
	} else if(enable&&(hifi_mode_status == 1)) { //shijianxing add: Low Power Mode enable
		//shijianxing M:Low Power Mode SmapleRate always 48K,MCLK not need
		ak4376_reset(ak4376_data->rst_gpio);

		if (ak4376_version == ak4375_VERSION_A) {
			if (format == SNDRV_PCM_FORMAT_S24_LE) {
				if ((sample_rate >= 0) && (sample_rate <= 5))
					mode_sel = sample_rate + 5;
				else
					mode_sel = 6;
			} else
				mode_sel = 4;
		} else if (ak4376_version == AK4376_VERSION_0) {
			if (format == SNDRV_PCM_FORMAT_S24_LE) {
				if ((sample_rate >= 0) && (sample_rate < 12))
					mode_sel = sample_rate + 11;//shijianxing add
				else
					mode_sel = 11;//shijianxing add for default 48K
			} else
				mode_sel = 23;//shijianxing add for 48K 16bits
		} else {
			if (format == SNDRV_PCM_FORMAT_S24_LE) {
				if (sample_rate == 1)
					mode_sel = 1;
				else if (sample_rate == 3)
					mode_sel = 2;
				else if (sample_rate == 5)
					mode_sel = 3;
			} else
				mode_sel = 0;
		}
		pr_info("[HiFi-%d]:mode_sel = %d,ak4376_version = %d\n",__LINE__, mode_sel,ak4376_version);
		/*
		*shijianxing add: must be set PMHPL and PMHPR bit to 0 before changing the mode
		*/
		if (ak4376_version == AK4376_VERSION_0) {
			ret = ak4376_i2c_write_byte(0x03,0x00);
			if(ret < 0) {
				pr_info("[HiFi-%d]:check i2c addr= 0x%x,value= 0x%x\n",__LINE__,reg_addr, reg_value);
				goto end;
			}
		}

		for(i=0; i<36; i++) {

			if (ak4376_version == AK4376_VERSION_0) {
				reg_addr = ak4376_normal_reg_enable[i][0];
				reg_value = ak4376_normal_reg_enable[i][1];
			} else {
				reg_addr = ak4375_reg_enable[i][0];
				reg_value = ak4375_reg_enable[i][1];
			}

			for (j = 0; j < 7; j++) {
				if (reg_addr_tbl [j] == reg_addr) {
					reg_value = reg_value_tbl[mode_sel][j];
					break;
				}
			}
			if (reg_addr == 0xff) {
				usleep_range(2000, 2000);
				continue;
			}

			for (j = 0; j < ak4376_data->reg_size; j++) {
				reg_vals = ak4376_data->reg_table;
				if (reg_vals[ 2 * j] == reg_addr) {
					reg_value = reg_vals[ 2 * j + 1];
					pr_info("[HiFi-%d]: + new addr 0x%x val 0x%x.\n",__LINE__,reg_addr, reg_value);
					break;
				}
			}

			ret = ak4376_i2c_write_byte(reg_addr,reg_value);
			if(ret < 0) {
				pr_err("[HiFi-%d]:check i2c addr= 0x%x,value= 0x%x\n",__LINE__,reg_addr, reg_value);
				goto end;
			}
			ret = do_reg_write_check(reg_addr,reg_value);
			if(ret < 0) {
				pr_err("[HiFi-%d]:check i2c addr= 0x%x,value= 0x%x\n",__LINE__,reg_addr, reg_value);
				goto end;
			}
		}
		//msleep(26);//shijianxing add for HPL/HPR Amplifier power on time,25.9ms@44.1k,23.9ms@other samplerates
		ret = 0;
	} else {

		for(i=0; i<6; i++) {
			reg_addr = ak4376_reg_disable[i][0];
			reg_value = ak4376_reg_disable[i][1];
			if (reg_addr == 0xff) {
				usleep_range(2000, 2000);
				continue;
			}
			ret = ak4376_i2c_write_byte(reg_addr,reg_value);
			if(ret < 0) {
				pr_err("[HiFi-%d]:check i2c addr= 0x%x,value= 0x%x\n",__LINE__,reg_addr, reg_value);
				goto end;
			}
			ret = do_reg_write_check(reg_addr,reg_value);
			if(ret < 0) {
				pr_err("[HiFi-%d]:check i2c addr= 0x%x,value= 0x%x\n",__LINE__,reg_addr, reg_value);
				goto end;
			}
		}

		gpio_direction_output(ak4376_data->rst_gpio,0);
		usleep_range(5000, 5000);
		if(hifi_mode_status != 1) {
			pr_info("[HiFi-%d] MCLK close.\n",__LINE__);
			if (ak4376_data->hifi_mclk)
				clk_disable_unprepare(ak4376_data->hifi_mclk);
		}
		ret = 0;
	}
end:
	mutex_unlock(&ak4376_data->lock);
	pr_info("[HiFi-%d]exist\n",__LINE__);
	ret = 0;
	return ret;
}

#ifdef BBK_IQOO_AUDIO_DEBUG
static int ak4376_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}
static ssize_t ak4376_debug_write(struct file *filp,
                                  const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	int ret = 0;
	unsigned int kbuf[2];
	char *temp;

	temp = kmalloc(cnt, GFP_KERNEL);
	if (!temp) {
		return -ENOMEM;
	}

	ret = copy_from_user(temp, (void __user *)ubuf, cnt);
	ret = sscanf(temp,"%x %x",&kbuf[0],&kbuf[1]);
	if(!ret) {
		kfree(temp);
		return -EFAULT;
	}
	pr_info("[HiFi-%d]kbuf[0]=0x%x,kbuf[1]=0x%x cnt =%d\n",__LINE__,kbuf[0],kbuf[1],(int)cnt);

	if(kbuf[0]==0xff) {
		pr_info("[HiFi-%d]gpio =0 disable voltage convert \n",__LINE__);
	} else {

		pr_info("[HiFi-%d]gpio =1 enable voltage convert \n",__LINE__);
	}

	ak4376_i2c_write_byte(kbuf[0],kbuf[1]);
	kfree(temp);

	return cnt;
}
static ssize_t ak4376_debug_read(struct file *file, char __user *buf,
                                 size_t count, loff_t *pos)
{
	int i;
	const int size = 512;
	u8 data;
	char buffer[size];
	int n = 0;

	for(i = 0; i < sizeof(ak4376_regs); i++) {
		data = ak4376_i2c_read_byte(ak4376_regs[i]);
		n += scnprintf(buffer+n,size-n,"reg{%x}:%x \n",ak4376_regs[i],data);
	}

	buffer[n] = 0;

	pr_info("[HiFi-%d]============caught codec reg end =============\n",__LINE__);
	return simple_read_from_buffer(buf, count, pos, buffer, n);

}

static struct file_operations ak4376_debugfs_fops = {
	.open = ak4376_debug_open,
	.read = ak4376_debug_read,
	.write = ak4376_debug_write,
};

static ssize_t ak4376_debug_i2c_read(struct file *file, char __user *buf,
                                     size_t count, loff_t *pos)
{
	const int size = 512;
	char buffer[size];
	int n = 0;

	pr_info("[HiFi-%d]i2c read enter.\n", __LINE__);

	n += scnprintf(buffer+n, size-n, "HiFi-0x%x %s\n", ak4376_data->client->addr,
	               ak4376_VERSION_NONE != ak4376_version ? "OK" : "ERROR");
	buffer[n] = 0;

	return simple_read_from_buffer(buf, count, pos, buffer, n);
}

static struct file_operations ak4376_i2c_debugfs_fops = {
	.open = ak4376_debug_open,
	.read = ak4376_debug_i2c_read,
};

static void ak4376_debugfs_init(void)
{
	ak4376_debugfs_root = debugfs_create_dir("audio-ak4376", NULL);
	if (!ak4376_debugfs_root) {
		pr_err("[HiFi-%d] debugfs create dir error\n", __LINE__);
	} else if (IS_ERR(ak4376_debugfs_root)) {
		pr_err("[HiFi-%d] Kernel not support debugfs \n", __LINE__);
		ak4376_debugfs_root = NULL;
	}

	ak4376_debugfs_reg = debugfs_create_file("reg", 0644, ak4376_debugfs_root,
	                     NULL, &ak4376_debugfs_fops);
	if (!ak4376_debugfs_reg) {
		pr_err("[HiFi-%d] debugfs create fail \n",__LINE__);
	}

	ak4376_debugfs_i2c = debugfs_create_file("i2c", 0444, ak4376_debugfs_root,
	                     NULL, &ak4376_i2c_debugfs_fops);
	if (!ak4376_debugfs_i2c) {
		pr_err("[HiFi-%d] i2c create fail \n",__LINE__);
	}

	return ;
}

static void ak4376_debugfs_deinit(void)
{
	debugfs_remove(ak4376_debugfs_i2c);
	debugfs_remove(ak4376_debugfs_reg);
	debugfs_remove(ak4376_debugfs_root);
	return ;
}
#endif

static void ak4376_reg_table(const struct device_node *np, struct ak4376_data *ak4376)
{
	int reg_size, ret;
	int *array;
	if (of_find_property(np, "vivo,reg_table", &reg_size)) {
		reg_size /= sizeof(int);
		if (!(reg_size % 2)) {
			array = kzalloc(sizeof(int) * reg_size, GFP_KERNEL);
			if (!array) {
				ak4376->reg_size = 0;
				pr_err("[HiFi-%d] Out of Memory\n",__LINE__);
				return;
			}
			ret = of_property_read_u32_array(np, "vivo,reg_table", array, reg_size);
			if (!ret) {
				ak4376->reg_size = reg_size/2;
				ak4376->reg_table = array;
				pr_err("[HiFi-%d] special params \n",__LINE__);
				return;
			}
		}
	}
	ak4376->reg_size = 0;
	pr_info("[HiFi-%d]  no special params \n",__LINE__);
	return;
}

static int ak4376_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct vivo_codec_function *ak4376_function = NULL;

	pr_info("[HiFi-%d] probe start\n",__LINE__);

#ifdef BBK_IQOO_AUDIO_DEBUG
	ak4376_debugfs_init();
#endif

	ak4376_function = get_vivo_codec_function();

	if (!ak4376_function) {
		pr_err("[HiFi-%d]  codec not ready \n", __LINE__);
		return -EPROBE_DEFER;
	}
	ak4376_data = kzalloc(sizeof(struct ak4376_data), GFP_KERNEL);
	if (!ak4376_data) {
		pr_err("[HiFi-%d] kzalloc failed\n",__LINE__);
		return -ENOMEM;
	}
	ak4376_data->fun = ak4376_function;
	ak4376_data->fun->hifi_dac_enable = ak4376_enable;
	ak4376_data->fun->hifi_dac_mute = ak4376_mute;
	/* parse regulator */
	ak4376_data->hifi_1v8_regulator = regulator_get(&client->dev, "vivo,hifi-1v8");
	if(!ak4376_data->hifi_1v8_regulator || IS_ERR(ak4376_data->hifi_1v8_regulator))
		pr_err("[HiFi-%d]get 1v8 regulator failed\n",__LINE__);
	else {
		if ((regulator_count_voltages(ak4376_data->hifi_1v8_regulator) > 0)
		    && regulator_can_change_voltage(ak4376_data->hifi_1v8_regulator)) {
			ret = regulator_set_voltage(ak4376_data->hifi_1v8_regulator, 1800000,
			                            1800000);
			if (ret) {
				pr_err("[HiFi-%d] regulator set_vtg failed rc=%d\n",__LINE__, ret);
				regulator_put(ak4376_data->hifi_1v8_regulator);
			}
		}
	}
	// this 1.8v supply need to be always on.
	if (ak4376_data->hifi_1v8_regulator && !IS_ERR(ak4376_data->hifi_1v8_regulator))
		ret = regulator_enable(ak4376_data->hifi_1v8_regulator);
	msleep(5);
	/* parse gpios */
	ak4376_data->rst_gpio = of_get_named_gpio(client->dev.of_node,"ext-dac-rst-gpio",0);
	ret = gpio_request(ak4376_data->rst_gpio, "ak4376-reset-gpio");
	if(ret < 0) {
		pr_err("[HiFi-%d] :gpio %d for dac reset gpio request failed.\n", __LINE__,
		       ak4376_data->rst_gpio);
	}
	pr_info("[HiFi-%d] :rst_gpio=%d\n", __LINE__, ak4376_data->rst_gpio);

	ak4376_data->client = client;
	ak4376_data->driver_name = ak4376_DEV_NAME;

	if(!i2c_check_functionality(client->adapter,I2C_FUNC_I2C))
		pr_err("[HiFi-%d] i2c check funtion error \n",__LINE__);

	ak4376_data->hifi_mclk = clk_get(&client->dev,"ak43xx_mclk");

	if (IS_ERR(ak4376_data->hifi_mclk)) {
		pr_err("[HiFi-%d] Get main 19.2M clock: %d\n",__LINE__,(int)PTR_ERR(ak4376_data->hifi_mclk));
		ak4376_data->hifi_mclk = NULL;
	} else {
		pr_info("[HiFi-%d] :get 19.2M clock suscess!!\n", __LINE__);
		clk_set_rate(ak4376_data->hifi_mclk, 19200000);
	}

	ak4376_reset(ak4376_data->rst_gpio);

	ret = ak4376_i2c_read_byte(0x15);
	if (ret < 0) {
		pr_err("[HiFi-%d] ak4376 device not exist\n",__LINE__);
		goto err_gpio;
	}
	ret = ((ret & 0xe0) >> 5);
	pr_info("[HiFi-%d]  0x15 = 0x%x\n",__LINE__, ret);
	switch (ret) {
	case 0:
		ak4376_version = ak4375_VERSION_0;
		pr_info("[HiFi-%d] VERSION_0\n",__LINE__);
		break;
	case 1:
		ak4376_version = ak4375_VERSION_A;
		pr_info("[HiFi-%d]  VERSION_A\n",__LINE__);
		break;
	case 2:
		ak4376_version = AK4376_VERSION_0;
		pr_info("[HiFi-%d] VERSION_0\n",__LINE__);
		break;
	default:
		ak4376_version = ak4376_VERSION_NONE;
		pr_err("[HiFi-%d]  Unsupported device revision\n",__LINE__);
		break;
	}
	ak4376_available = true;
	mutex_init(&ak4376_data->lock);

	//shijianxing add:reset gpio by probe finish
	gpio_direction_output(ak4376_data->rst_gpio,0);

	ak4376_reg_table(client->dev.of_node, ak4376_data);

	ret = misc_register(&ak4376_phys_dev);
	if(ret < 0) {
		pr_err("[HiFi-%d]  ak4376 phys misc device register error\n",__LINE__);
		goto err_gpio;
	}
err_gpio:

	return ret;

}

/*PDN power down before AVDD @fanyongxiang*/
static void ak4376_i2c_shutdown(struct i2c_client *client)
{
	gpio_direction_output(ak4376_data->rst_gpio, 0);
	msleep(5);
	if (ak4376_data->hifi_1v8_regulator
		&& !IS_ERR(ak4376_data->hifi_1v8_regulator))
		regulator_disable(ak4376_data->hifi_1v8_regulator);
}

static int ak4376_i2c_remove(struct i2c_client *client)
{
#ifdef BBK_IQOO_AUDIO_DEBUG
	ak4376_debugfs_deinit();
#endif

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id device_ak4376_of_match[] = {
	{.compatible = "ak,ak4376",},
	{},
};
#else
#define device_ak4376_of_match 0
#endif

static const struct i2c_device_id ak4376_i2c_id[] = {
	{ "ak4376", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ak4376_i2c_id);

static struct i2c_driver ak4376_i2c_driver = {
	.driver = {
		.name = "ak4376-codec",
		.owner = THIS_MODULE,
		.of_match_table = device_ak4376_of_match,
	},
	.probe 		= ak4376_i2c_probe,
	.remove 	= ak4376_i2c_remove,
	.shutdown   = ak4376_i2c_shutdown,
	.id_table = ak4376_i2c_id,
};

module_i2c_driver(ak4376_i2c_driver);

MODULE_DESCRIPTION("ASoC ak4376 codec driver");
MODULE_AUTHOR("Houzheng.Shen <shenhouzheng@iqoo.com>");
MODULE_LICENSE("GPL");
