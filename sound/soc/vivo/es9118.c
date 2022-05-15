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
#include "es9118.h"

#undef pr_debug
#define pr_debug pr_info

#define ES9118_DEV_NAME "es9118"

static unsigned char dac_parameters[][2] = {
	{1,0x80},
	{14,0x45},
	{2,0xb4},
	{5,0x0},
	{4,0xff},
	{6,0x07},
	{32,0x80},
	{0xff,0xff},
	{29,0x0d},
	{0xff,0xff},
	{0xff,0xff},
	{0xff,0xff},
	{46,0x80},
	{32,0x83},
	{0xff,0xff},
	{0xff,0xff},
	{0xff,0xff},
	{0xff,0xff},
	{0xff,0xff},
	{46,0x0},
	{5,0x7f},
	{0xff,0xff},
	{0xff,0xff},
	{0xff,0xff},
	{4,0x0},
	{2,0x34},
	{15,0x0},
	{16,0x0},
	{17,0xff},
	{18,0xff},
	{19,0xff},
	{20,0x54},
	{27,0xc4},

};

#ifdef CONFIG_DEBUG_FS
static struct dentry *es9118_debugfs_root;
static struct dentry *es9118_debugfs_reg;
static struct dentry *es9118_debugfs_enable;
static struct dentry *es9118_debugfs_i2c;

#endif

struct es9118_data {
	struct i2c_client *client;
	char *driver_name;

	/* state */
	bool on;
	bool chipid_ok;
	/*
	* work_mode here is a unsigned int type,
	* bit 0 ~ 7 for headphone impedance,
	* bit 8 ~ 9 for channel mode
	* bit 12 for I2S Master Mode
	* other bits reserved.
	*/
	struct mutex lock;
	struct audio_params params;
	int param_row;
	int param_col;
	unsigned int *ic_params;

	struct vivo_codec_function *fun;
	/*pinctrl config about gpio*/
	int rst_gpio;
	struct regulator *hifi_1v8_regulator;
};
static struct es9118_data *es9118_data;

static u8 es9118_i2c_read_byte(struct es9118_data *edata, u8 reg)
{
	int ret = 0;
	u8 buf = 0xff;

	ret = i2c_master_send(edata->client,&reg,1);
	if(ret < 0) {
		pr_err("[HiFi-%d] i2c send cmd error reg=%d \n",__LINE__,reg);
		return ret;
	}
	ret = i2c_master_recv(edata->client,&buf,1);
	if(ret <0 ) {
		pr_err("[HiFi-%d] i2c recv error \n ",__LINE__);
		return ret;
	}

	return buf;

}

static u8 es9118_i2c_write_byte(struct es9118_data *edata, u8 reg,u8 data)
{
	int ret = 0;
	u8 cmd[2];
	static int i2c_error = 1;

	cmd[0] = reg;
	cmd[1] = data;

	ret = i2c_master_send(edata->client,cmd,sizeof(cmd));
	if (ret < 1) {
		pr_err("[HiFi-%d] i2c send error cmd[0]=%d,cmd[1]=%d\n",__LINE__,cmd[0],cmd[1]);
		if (i2c_error) {
			i2c_error = 0;
		}
	}

	return ret;
}

int es9118_setting_special_params(struct es9118_data *edata, struct es9118_params *params)
{
	int i = 0;
	struct es9118_reg_peer *peer;

	if (params->size <= 0 || params->peer == NULL) {
		pr_warn("[HiFi-%d] no params to set\n", __LINE__);
		return 0;
	}

	for (i = 0; i < params->size; i++) {
		peer = params->peer + i;
		if (peer) {
			pr_debug("[HiFi-%d] write value 0x%x to addr 0x%x.\n",
			         __LINE__, peer->val, peer->addr);
			es9118_i2c_write_byte(edata, peer->addr, peer->val);
		} else
			pr_err("[HiFi-%d] peer is NULL, ignore writing.\n", __LINE__);
	}
	return 0;
}

static int es9118_power_enable(struct es9118_data *edata, int enable)
{
	int ret = 0;
	if (!edata || !edata->hifi_1v8_regulator) {
		pr_err("[HiFi-%d]params is NULL\n", __LINE__);
		return -EINVAL;
	}

	pr_debug("[HiFi-%d] Power supply setting enable:%d.\n",__LINE__,enable);
	if(enable) {
		if (edata->rst_gpio > 0) {
			ret = gpio_direction_output(edata->rst_gpio, 0);//RESETb -> 1
			if (ret) {
				pr_err("[HiFi-%d] Can not set rst_gpio", __LINE__);
				return ret;
			}
			usleep_range(2000, 2200);
		}
		ret = regulator_enable(edata->hifi_1v8_regulator);
	} else {
		if (edata->rst_gpio > 0) {
			ret = gpio_direction_output(edata->rst_gpio, 0);
			if (ret) {
				pr_err("[HiFi-%d]: Can not set rst_gpio", __LINE__);
				return ret;
			}
			usleep_range(2000, 2200);
		}
		ret = regulator_disable(edata->hifi_1v8_regulator);
	}

	return 0;
}

static int es9118_reset(struct es9118_data *edata)
{
	int ret = 0;

	pr_debug("[HiFi-%d] reset.\n",__LINE__);
	udelay(5000);
	if (edata && edata->rst_gpio > 0) {
		ret = gpio_direction_output(edata->rst_gpio, 1);
		if (ret) {
			pr_err("[HiFi-%d] Can not set rst_gpio", __LINE__);
			return ret;
		}
		udelay(10000);
		gpio_direction_output(edata->rst_gpio, 0);
		udelay(10);//DVDD should not fall below 1V
		gpio_direction_output(edata->rst_gpio, 1);
		udelay(5000);
	}

	return 0;
}

static int do_reg_write_check(struct es9118_data *edata, u8 reg, u8 val)
{
	u8 reg_val = 0;
	int retry = 5;
	int ret = 0;
	char logbuf[100];
	reg_val = es9118_i2c_read_byte(edata, reg);
	while((val != reg_val)&&(retry > 0)) {
		es9118_i2c_write_byte(edata, reg, val);
		reg_val = es9118_i2c_read_byte(edata, reg);
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

/* int es9118_normal_enable(bool enable)
{
	int ret = 0;
	if (!es9118_data || !es9118_data->hifi_1v8_regulator) {
		pr_err("%s:params is NULL\n", __func__);
		return -EINVAL;
	}

	pr_info("%s enable:%d.\n",__func__,enable);
	if(enable){
		if (es9118_data->rst_gpio > 0) {
			ret = gpio_direction_output(es9118_data->rst_gpio, 0);
			if (ret) {
				pr_err("%s: Can not set rst_gpio", __func__);
				return ret;
			}
		}
		ret = regulator_enable(es9118_data->hifi_1v8_regulator);
		usleep(5000);
	}else{
		usleep(5000);
		ret = regulator_disable(es9118_data->hifi_1v8_regulator);
	}
} */

int es9118_enable( struct audio_params *params, bool enable)
{
	u8 reg_val;
	int para_num = sizeof(dac_parameters) / 2;
	int clk_div;
	int i = 0;
	int ret = 0;

	if (es9118_data->client == NULL) {
		pr_err("[HiFi-%d] client is NULL \n ",__LINE__);
		return -EFAULT;
	}

	if (!params) {
		pr_err("[HiFi-%d]:params is NULL\n", __LINE__);
		return -EINVAL;
	}
	memcpy(&es9118_data->params, params, sizeof(struct audio_params));
	pr_info("[HiFi-%d]:enable %d, pcm_format 0x%x, params->i2s_format 0x%x\n",
	        __LINE__, enable, params->pcm_format, params->i2s_format);

	mutex_lock(&es9118_data->lock);

	if (enable) {
		es9118_power_enable(es9118_data, 1);
		es9118_reset(es9118_data);

		/*set master or slave mode by bit7*/
		reg_val = es9118_i2c_read_byte(es9118_data, 10);
		reg_val &= ~(0x1 << 7);

		/*
		* platform always setting slave mode
		*/
		if ((params->i2s_format &SND_SOC_DAIFMT_MASTER_MASK) ==SND_SOC_DAIFMT_CBM_CFM) {
			pr_info("[HiFi-%d] set chip master mode.\n",__LINE__);
			/* Set master mode use DAC supply I2S clock*/
			reg_val |= (0x1 << 7);

			/* Set divider
			  * LRCK = BCLK / 64, BCLK = MCLK / n
			  * when divider is 0, n = 8.
			  * when divider is 1, n = 16.
			  * when divider is 2 or 3, n = 32.
			  */
			reg_val &= ~(0x3 << 5);
			clk_div = params->sys_clk / params->rate / 64;
			pr_info("[HiFi-%d] sys_clk:%lu clk_div:%d \n",__LINE__, params->sys_clk, clk_div);
			if ((params->sys_clk % params->rate) == 0 &&
			    (params->sys_clk / params->rate % 64) == 0) {
				if (clk_div == 4) {
					reg_val |= 0x1 << 5;
				} else if (clk_div == 8) {
					reg_val |= 0x2 << 5;
				} else if (clk_div == 16) {
					reg_val |= 0x3 << 5;
				} else
					pr_err("[HiFi-%d]:error div %d", __LINE__, clk_div);
			} else
				pr_err("[HiFi-%d]: error sys_clk %lu and rate %d",
				       __LINE__, params->sys_clk, params->rate);

		}
		es9118_i2c_write_byte(es9118_data, 10, reg_val);


		for (i = 0; i < para_num; i++) {
			if (dac_parameters[i][0] == 0xff) {
				usleep_range(2000, 2200);;
				continue;
			}
#if 0
			if ((dac_parameters[i][0] == 32)&&(dac_parameters[i][1] == 0x83)) {
				printk("%s reg#32 delay 120ms for pop.\n",__func__);
				usleep(120000);
			}
#endif
			es9118_i2c_write_byte(es9118_data, dac_parameters[i][0], dac_parameters[i][1]);
			pr_info("[HiFi-%d] reg[%d]:0x%x\n",__LINE__,dac_parameters[i][0],dac_parameters[i][1]);
			ret = do_reg_write_check(es9118_data, dac_parameters[i][0], dac_parameters[i][1]);
			if(ret < 0)
				goto end;
		}

		/* setting special params */
		if (params->private_params)
			es9118_setting_special_params(es9118_data, (struct es9118_params *)(params->private_params));

		es9118_data->on = true;
		ret = 0;

	} else {

		es9118_i2c_write_byte(es9118_data, 14, 0x5);
		es9118_i2c_write_byte(es9118_data, 2, 0xb4);
		es9118_i2c_write_byte(es9118_data, 5, 0x0);
		es9118_i2c_write_byte(es9118_data, 4, 0xff);
		es9118_i2c_write_byte(es9118_data, 20, 0xff);
		usleep_range(5000, 5100);
		es9118_i2c_write_byte(es9118_data, 32, 0x80);
		usleep_range(2000, 2200);
		es9118_i2c_write_byte(es9118_data, 29, 0x0d);
		usleep_range(10000, 11000);
		es9118_power_enable(es9118_data, 0);
		es9118_data->on = false;
		ret = 0;
	}
end:
	mutex_unlock(&es9118_data->lock);
	pr_info("[HiFi-%d]enable end.\n",__LINE__);
	return ret;
}

int es9118_mute(int mute)
{
	int reg_val = 0;

	pr_info("[HiFi-%d]:mute = %d\n",__LINE__, mute);
	if (!es9118_data->on) {
		pr_warn("[HiFi-%d] es9118(addr 0x%.2x) not enabled, exit.\n",
		        __LINE__, es9118_data->client->addr);
		return 0;
	}
	/*mute by reg#7 bit0 */
	reg_val = es9118_i2c_read_byte(es9118_data, 7);
	if (mute) {
		//usleep_range(10000, 11000);
		reg_val = reg_val | 0x1;
		es9118_i2c_write_byte(es9118_data, 7, reg_val);
		es9118_i2c_write_byte(es9118_data, 14, 0x08);
	} else {
		reg_val = reg_val & 0xfe;
		es9118_i2c_write_byte(es9118_data, 7, reg_val);
		es9118_i2c_write_byte(es9118_data, 14, 0x0a);
		//usleep_range(10000, 11000);
	}

	pr_info("[HiFi-%d]mute end.\n",__LINE__);
	return 0;
}


#ifdef CONFIG_DEBUG_FS
static int es9118_debug_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;

	return 0;
}

static int es9118_debug_release (struct inode *inode, struct file *filep)
{
	int ret =0;

	filep->private_data = NULL;
	pr_info("[HiFi-%d]release\n", __LINE__);
	return ret;
}

static ssize_t es9118_debug_write(struct file *filp,
                                  const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	int ret = 0;
	unsigned int kbuf[2];

	ret = sscanf(ubuf,"%x %x",&kbuf[0],&kbuf[1]);
	if(!ret)
		return -EFAULT;

	pr_info("[HiFi-%d] kbuf[0] = 0x%x,kbuf[1]= 0x%x cnt = %lu\n",__LINE__,
	        kbuf[0], kbuf[1], cnt);

	/* if dac is not on, return */
	if (!es9118_data->on) {
		pr_info("[HiFi-%d] sorry, dac(addr 0x%.2x) is off!\n",
		        __LINE__, es9118_data->client->addr);
		return 0;
	}
	/* if dac is on, write registers */
	es9118_i2c_write_byte(es9118_data, kbuf[0],kbuf[1]);
	do_reg_write_check(es9118_data, kbuf[0],kbuf[1]);

	return cnt;
}

static ssize_t es9118_debug_read(struct file *file, char __user *buf,
                                 size_t count, loff_t *pos)
{
	int i;
	const int size = 1024;
	u8 data;
	char buffer[size];
	int n = 0;


	/* if dac is not on, return */
	if (!es9118_data->on) {
		n = scnprintf(buffer, size, "Sorry, dac(addr 0x%.2x) is off!\n",
		              es9118_data->client->addr);
		buffer[n] = 0;
		pr_info("[HiFi-%d] %s", __LINE__, buffer);
		return simple_read_from_buffer(buf, count, pos, buffer, n);
	}

	/* if dac is on, dump registers */
	for(i = 0; i < 47; i++) {
		data = es9118_i2c_read_byte(es9118_data, i);
		n += scnprintf(buffer+n,size-n,"reg{%d}:%x \n",i,data);
	}

	/*Read only register*/
	for(i = 64; i < 77; i++) {
		data = es9118_i2c_read_byte(es9118_data, i);
		n += scnprintf(buffer+n,size-n,"reg{%d}:%x \n",i,data);
	}

	buffer[n] = 0;
	pr_info("[HiFi-%d]:==========catch HiFi reg====addr%x======\n%s\n",
	        __LINE__, es9118_data->client->addr, buffer);
	pr_info("============caught HiFi reg end =============\n");
	return simple_read_from_buffer(buf, count, pos, buffer, n);
}

static struct file_operations es9118_debugfs_fops = {
	.open = es9118_debug_open,
	.read = es9118_debug_read,
	.write = es9118_debug_write,
	.release = es9118_debug_release,
};

static ssize_t es9118_enable_debug_write(struct file *filp,
        const char __user *ubuf, size_t cnt, loff_t *ppos)
{
	int ret = 0;
	int enable;

	ret = sscanf(ubuf,"%x",&enable);
	if(!ret)
		return -EFAULT;

	pr_info("[HiFi-%d]enable = %d, cnt = %lu\n",__LINE__,
	        enable, cnt);

	es9118_power_enable(es9118_data, enable);

	return cnt;
}

static ssize_t es9118_enable_debug_read(struct file *file, char __user *buf,
                                        size_t count, loff_t *pos)
{
	const int size = 512;
	char buffer[size];
	int n = 0;


	n = scnprintf(buffer, size, "es9118 status %s\n",
	              es9118_data->on?"On":"Off");
	buffer[n] = 0;
	pr_info("[HiFi-%d] status %s\n", __LINE__,es9118_data->on?"On":"Off");
	return simple_read_from_buffer(buf, count, pos, buffer, n);
}

static struct file_operations es9118_enable_debugfs_fops = {
	.read = es9118_enable_debug_read,
	.write = es9118_enable_debug_write,
};
static ssize_t es9118_i2c_debug_read(struct file *file, char __user *buf,
                                     size_t count, loff_t *pos)
{
	const int size = 512;
	char buffer[size];
	int n = 0;

	pr_info("[HiFi-%d]debug read enter\n", __LINE__);

	n += scnprintf(buffer+n, size-n, "HiFi-0x%x %s\n", es9118_data->client->addr,
	               es9118_data->chipid_ok ? "OK" : "ERROR");

	buffer[n] = 0;

	return simple_read_from_buffer(buf, count, pos, buffer, n);
}

static struct file_operations es9118_i2c_debugfs_fops = {
	.read = es9118_i2c_debug_read,
};

static void es9118_debugfs_init(void)
{
	es9118_debugfs_root = debugfs_create_dir("audio-es9118",NULL);
	if(!es9118_debugfs_root) {
		pr_err("[HiFi-%d] debugfs create dir error\n",__LINE__);
	} else if(IS_ERR(es9118_debugfs_root)) {
		pr_err("[HiFi-%d] Kernel not support debugfs \n",__LINE__);
		es9118_debugfs_root = NULL;
	}

	es9118_debugfs_reg = debugfs_create_file("reg",0644,es9118_debugfs_root,NULL,&es9118_debugfs_fops);
	if(!es9118_debugfs_reg) {
		pr_err("[HiFi-%d] debugfs create fail \n",__LINE__);
	}
	es9118_debugfs_enable = debugfs_create_file("enable",0644,es9118_debugfs_root,NULL,&es9118_enable_debugfs_fops);
	if(!es9118_debugfs_reg) {
		pr_err("[HiFi-%d] debugfs create fail \n",__LINE__);
	}
	es9118_debugfs_i2c = debugfs_create_file("i2c",0644,es9118_debugfs_root,NULL,&es9118_i2c_debugfs_fops);
	if(!es9118_debugfs_reg) {
		pr_err("[HiFi-%d] debugfs create fail \n",__LINE__);
	}

	return ;
}

static void es9118_debugfs_deinit(void)
{
	debugfs_remove(es9118_debugfs_reg);
	debugfs_remove(es9118_debugfs_root);
	return ;
}

#endif

static int es9118_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct vivo_codec_function *function = NULL;
	int ret = 0;
	u8 chipid = 0x0;
	int retry = 5;

	pr_info("[HiFi-%d]probe start.\n",__LINE__);
#ifdef CONFIG_DEBUG_FS
	es9118_debugfs_init();
#endif
	function = get_vivo_codec_function();
	if (!function) {
		pr_err("[HiFi-%d]vivo_codec_function is NULL\n",
		       __LINE__);
		return -EPROBE_DEFER;
	}

	es9118_data = kzalloc(sizeof(struct es9118_data), GFP_KERNEL);

	if(!es9118_data) {
		pr_err("[HiFi-%d] kzalloc failed\n",__LINE__);
		return -ENOMEM;
	}

	if(client->dev.of_node) {
		pr_info("[HiFi-%d]: gpio and regulator request.\n", __LINE__);
		/*dac reset gpio request*/
		es9118_data->rst_gpio = of_get_named_gpio(client->dev.of_node, "vivo,hifi-rst-gpio", 0);
		ret = gpio_request(es9118_data->rst_gpio,"hifi reset gpio");
		if(ret)
			pr_err("[HiFi-%d]:gpio %d for dac reset gpio request failed.\n", __LINE__,es9118_data->rst_gpio);

		/*regulators request*/
		es9118_data->hifi_1v8_regulator = regulator_get(&client->dev, "vivo,hifi-1v8");
		if(!es9118_data->hifi_1v8_regulator || IS_ERR(es9118_data->hifi_1v8_regulator))
			pr_err("[HiFi-%d]Get 1v8 regulator failed\n",__LINE__);
		else {
			if ((regulator_count_voltages(es9118_data->hifi_1v8_regulator) > 0)
			    && regulator_can_change_voltage(es9118_data->hifi_1v8_regulator)) {
				ret = regulator_set_voltage(es9118_data->hifi_1v8_regulator, 1800000,
				                            1800000);
				if (ret) {
					pr_err("[HiFi-%d]regulator set_vd failed rc=%d\n",__LINE__, ret);
					regulator_put(es9118_data->hifi_1v8_regulator);
				}
			}
		}
	}

	es9118_data->client = client;
	es9118_data->driver_name = ES9118_DEV_NAME;
	es9118_data->fun = function;
	function->hifi_dac_enable = es9118_enable;
	function->hifi_dac_mute = es9118_mute;

	if (!i2c_check_functionality(client->adapter,I2C_FUNC_I2C)) {
		pr_err("[HiFi-%d] i2c check funtion error \n",
		       __LINE__);
	}

	es9118_power_enable(es9118_data, 1);
	if (function->mclk_enable)
		function->mclk_enable(VIVO_CODEC_HIFI_CLK, 48000);
	es9118_reset(es9118_data);

	/*Reg#64 chip id:[7:2]=110000*/
	chipid = es9118_i2c_read_byte(es9118_data, ES9118_ID_REG);
	pr_info("[HiFi-%d]:reg#64 value: 0x%x\n", __LINE__, chipid);

	chipid = chipid >> 2;
	pr_info("[HiFi-%d]:chip id = 0x%x\n", __LINE__, chipid);
	while (retry--) {
		if (ES9118_CHIP_ID != (chipid & 0x30)) {
			usleep_range(2000, 2200);;
			chipid = es9118_i2c_read_byte(es9118_data, ES9118_ID_REG);
		} else
			break;
	}

	if (ES9118_CHIP_ID == (chipid & 0x30)) {
		es9118_data->chipid_ok = true;
	} else {
		es9118_data->chipid_ok = false;
		pr_err("[HiFi-%d] chip id error 0x%x\n",
		       __LINE__, chipid);
	}

	mutex_init(&es9118_data->lock);

	usleep_range(2000, 2200);;
	if (function->mclk_disable)
		function->mclk_disable(VIVO_CODEC_HIFI_CLK);
	es9118_power_enable(es9118_data, 0);
	es9118_data->on = false;

	pr_info("[HiFi-%d]probe end.\n",__LINE__);
	return ret;

}

/*static int es9118_i2c_suspend(struct i2c_client *client,  pm_message_t mesg)
{
	return 0;
}

static int es9118_i2c_resume(struct i2c_client *client)
{
	return 0;
}*/

static int es9118_i2c_remove(struct i2c_client *client)
{

#ifdef CONFIG_DEBUG_FS
	es9118_debugfs_deinit();
#endif
	return 0;
}

/* PDN power down before AVDD @fanyongxiang */
static void es9118_i2c_shutdown(struct i2c_client *client)
{
	es9118_power_enable(es9118_data, 0);
	return ;
}

#ifdef CONFIG_OF
static const struct of_device_id device_es9118_of_match[] = {
	{.compatible = "ess,es9118",},
	{},
};
#else
#define device_es9118_of_match 0
#endif

static const struct i2c_device_id es9118_i2c_id[] = {
	{ ES9118_DEV_NAME, 0 },
	{ }
};

static struct i2c_driver es9118_i2c_driver = {
	.probe              = es9118_i2c_probe,
	//.suspend 			= es9118_i2c_suspend,
	//.resume 			= es9118_i2c_resume,
	.remove             = es9118_i2c_remove,
	.shutdown			= es9118_i2c_shutdown,
	.driver = {
		.name           = ES9118_DEV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = device_es9118_of_match,
	},
	.id_table = es9118_i2c_id,
};

module_i2c_driver(es9118_i2c_driver);

MODULE_DESCRIPTION("es9118 i2c driver");
MODULE_AUTHOR("shijianxing <shijianxing@vivo.com.cn>");
MODULE_LICENSE("GPL");
