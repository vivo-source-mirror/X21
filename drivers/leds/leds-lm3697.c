 /*
 * Copyright (C) 2008 -2009 Motorola, Inc.
 * Author: Alina Yakovleva < qvdh43@motorola.com >
 *
 * This program is free software; you can redistribute it and / or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <asm/io.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/sysfs.h>
#include <linux/hwmon-sysfs.h>
#include <linux/gpio_event.h>
#include <linux/leds.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/list.h>
#include <linux/platform_data/leds-lm3697.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#elif defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/pwm.h>
#include <linux/qpnp/pin.h>

#define MODULE_NAME "leds_lm3697"
#define LM3697_DEBUG 1

#if 0 /* / zsl */
static struct pwm_device *lm3697_pwm;
static int bl_pwm_period = -1;
static int bl_pwm_lpg_chan = -1;
static int bl_pwm_gpio = -1;
static int lm3697_pwm_enable;

static void lm3697_pwm_set(int level);
#endif
static struct i2c_client *lm3697_client;
static int old_backlight_value;

struct lm3697_data {
uint16_t addr;
	struct i2c_client *client;
	struct input_dev *idev;
	struct lm3697_platform_data *pdata;
	struct led_classdev led_dev;
	struct led_classdev led_dev_tcmd;
	struct led_classdev led_dev_nr;
	struct led_classdev button_led;
	struct led_classdev button_led_tcmd;
	struct led_classdev webtop_led;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#elif defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif
	unsigned initialized;
unsigned in_webtop_mode;
	uint8_t revision;
	uint8_t enable_reg;
	atomic_t suspended;
	atomic_t in_suspend; /* Whether the driver is in TCMD SUSPEND mode */
	unsigned bvalue; /* Current brightness register value */
	unsigned saved_bvalue; /* Brightness before TCMD SUSPEND */
	struct work_struct work;
};

static DEFINE_MUTEX(lm3697_mutex);

struct lm3697_reg {
	const char *name;
	uint8_t reg;
};

struct lm3697_reg lm3697_regs[] = {
	{"OUTPUT_CFG_REG", LM3697_OUTPUT_CFG_REG},
	{"CONTROL_A_START_UP_RAMP_REG", LM3697_CONTROL_A_START_UP_RAMP_REG},
	{"CONTROL_B_START_UP_RAMP_REG", LM3697_CONTROL_B_START_UP_RAMP_REG},
	{"RUN_TIME_RAMP_TIME_REG", LM3697_RUN_TIME_RAMP_TIME_REG},
	{"RUN_TIME_RAMP_CFG_REG", LM3697_RUN_TIME_RAMP_CFG_REG},
	{"CTRL_BR_CFG_REG", LM3697_CTRL_BR_CFG_REG},
	{"CTRL_A_FS_CURR_REG", LM3697_CTRL_A_FS_CURR_REG},
	{"CTRL_B_FS_CURR_REG", LM3697_CTRL_B_FS_CURR_REG},
	{"FEEDBACK_ENABLE_REG", LM3697_FEEDBACK_ENABLE_REG},
	{"BOOST_CONTROL_REG", LM3697_BOOST_CONTROL_REG},
	{"PWM_CFG_REG", LM3697_PWM_CFG_REG},
	{"CONTROL_A_BRIGHT_LSB_REG", LM3697_CONTROL_A_BRIGHT_LSB_REG},
	{"CONTROL_A_BRIGHT_MSB_REG", LM3697_CONTROL_A_BRIGHT_MSB_REG},
	{"CONTROL_B_BRIGHT_LSB_REG", LM3697_CONTROL_B_BRIGHT_LSB_REG},
	{"CONTROL_B_BRIGHT_MSB_REG", LM3697_CONTROL_B_BRIGHT_MSB_REG},
	{"CONTROL_BANK_ENABLE_REG", LM3697_CONTROL_BANK_ENABLE_REG},
	{"HVLED_OPEN_FAULTS_REG", LM3697_HVLED_OPEN_FAULTS_REG},
	{"HVLED_SHORT_FAULTS_REG", LM3697_HVLED_SHORT_FAULTS_REG},
	{"LED_FAULT_ENABLE_REG", LM3697_LED_FAULT_ENABLE_REG},
};


static int lm3697_read_reg (struct i2c_client *client,
	unsigned reg, uint8_t *value)
{
	uint8_t buf[1];
	int ret = 0;

	if (!client) {
		printk ("%s: client is NULL\n", __func__);
		return -EINVAL;
	}

	if (!value)
		return -EINVAL;
	buf[0] = reg;
	ret = i2c_master_send (client, buf, 1);
	if (ret > 0) {
		msleep_interruptible (1);
		ret = i2c_master_recv (client, buf, 1);
		if (ret > 0)
			*value = buf[0];
	}
	return ret;
}

static int lm3697_write_reg (struct i2c_client *client,
	unsigned reg, uint8_t value, const char *caller)
{
	uint8_t buf[2] = {reg, value};
	int ret = 0;

	if (!client) {
		printk ("%s: client is NULL\n", __func__);
		return -EINVAL;
	}

	ret = i2c_master_send (client, buf, 2);
	if (ret < 0)
		printk ("%s: i2c_master_send error %d\n",
			caller, ret);
	return ret;
}

static int lm3697_init_registers(void)
{
	int ret = 0;

	struct i2c_client *client;

	client = lm3697_client;

	lm3697_write_reg(client, LM3697_OUTPUT_CFG_REG, 0x04, __func__);
	lm3697_write_reg(client, LM3697_RUN_TIME_RAMP_TIME_REG, 0x12, __func__);
	lm3697_write_reg(client, LM3697_CTRL_BR_CFG_REG, 0x00, __func__);
	lm3697_write_reg(client, LM3697_CTRL_A_FS_CURR_REG, 0xf2, __func__);
	lm3697_write_reg(client, LM3697_BOOST_CONTROL_REG, 0x0D, __func__);
	lm3697_write_reg(client, LM3697_PWM_CFG_REG, 0x0D, __func__);
	lm3697_write_reg(client, LM3697_CONTROL_BANK_ENABLE_REG, 0x01, __func__);
	lm3697_write_reg(client, LM3697_AUTO_FRE_THRESHOLD_REG, 0x60, __func__);/* <= 45, fre:500K, > 45, fre:1M */
	return ret;
}


void bbk_backlight_disable(void)
{
		struct i2c_client *client;
	client = lm3697_client;

	if (!client) {
		printk ("%s: client is NULL\n", __func__);
		return;
	}

	lm3697_write_reg (client, LM3697_CONTROL_BANK_ENABLE_REG, 0x00, __func__);
	lm3697_write_reg (client, LM3697_FEEDBACK_ENABLE_REG, 0x00, __func__);
}

static void lm3697_backlight_set (struct led_classdev *led_cdev,
				enum led_brightness value)
{
	struct i2c_client *client;
	client = lm3697_client;

	if (!client) {
		printk ("%s:, client is NULL: [value=%d] \n", __func__, value);
		return;
	}
	mutex_lock (&lm3697_mutex);

	if (value <= 0) {
		lm3697_write_reg (client, LM3697_CONTROL_BANK_ENABLE_REG, 0x00, __func__);
		lm3697_write_reg (client, LM3697_FEEDBACK_ENABLE_REG, 0x00, __func__);
		lm3697_write_reg (client, LM3697_CONTROL_A_BRIGHT_LSB_REG, 0x00, __func__);
		lm3697_write_reg (client, LM3697_CONTROL_A_BRIGHT_MSB_REG, 0x00, __func__);
		old_backlight_value = 0;
	} else {
		if (old_backlight_value == 0)
			lm3697_init_registers();

		lm3697_write_reg (client, LM3697_CONTROL_A_BRIGHT_LSB_REG, 0x00, __func__);
		lm3697_write_reg (client, LM3697_CONTROL_A_BRIGHT_MSB_REG, value, __func__);
		lm3697_write_reg (client, LM3697_FEEDBACK_ENABLE_REG, 0x03, __func__);
		lm3697_write_reg (client, LM3697_CONTROL_BANK_ENABLE_REG, 0x01, __func__);
		old_backlight_value = value;
	}

	mutex_unlock (&lm3697_mutex);
	printk ("lm3697 set: [value=%d] \n", value);
	return;
}


void lm3697_brightness_set(int value)
{
	struct i2c_client *client;
	int backlight_lsb = 0;
	int backlight_msb = 0;

	client = lm3697_client;

	if (!client) {
		printk ("%s:, client is NULL: [value=%d] \n", __func__, value);
		return;
	}
	mutex_lock (&lm3697_mutex);

	if (value <= 0) {
		lm3697_write_reg (client, LM3697_CONTROL_BANK_ENABLE_REG, 0x00, __func__);
		lm3697_write_reg (client, LM3697_FEEDBACK_ENABLE_REG, 0x00, __func__);
		lm3697_write_reg (client, LM3697_CONTROL_A_BRIGHT_LSB_REG, 0x00, __func__);
		lm3697_write_reg (client, LM3697_CONTROL_A_BRIGHT_MSB_REG, 0x00, __func__);
		old_backlight_value = 0;
	} else {
		if (old_backlight_value == 0)
			lm3697_init_registers();

		backlight_lsb = value & 0x07;
		backlight_msb = (value >> 3) & 0xFF;
		printk ("lm3697 set: backlight_lsb is %0x,  backlight_msb is %02x\n", backlight_lsb, backlight_msb);

		lm3697_write_reg (client, LM3697_CONTROL_A_BRIGHT_LSB_REG, backlight_lsb, __func__);
		lm3697_write_reg (client, LM3697_CONTROL_A_BRIGHT_MSB_REG, backlight_msb, __func__);
		lm3697_write_reg (client, LM3697_FEEDBACK_ENABLE_REG, 0x03, __func__);
		/*lm3697_write_reg (client, LM3697_CONTROL_BANK_ENABLE_REG, 0x01, __func__); */

		if (old_backlight_value == 0) {
			lm3697_write_reg (client, LM3697_CONTROL_BANK_ENABLE_REG, 0x01, __func__);
		}
		old_backlight_value = value;
	}

	mutex_unlock (&lm3697_mutex);
	printk ("lm3697 set: [value=%d] \n", value);
	return;
}

static ssize_t lm3697_registers_show (struct device *dev,
					  struct device_attribute *attr, char *buf)
{
struct i2c_client *client = container_of(dev->parent,
struct i2c_client, dev);
struct lm3697_data *driver_data = i2c_get_clientdata(client);
int reg_count = sizeof(lm3697_regs) / sizeof(lm3697_regs[0]);
	int i, n = 0;
	uint8_t value;

if (atomic_read(&driver_data->suspended)) {
printk(KERN_INFO "%s: can't read: driver suspended\n",
__func__);
n += scnprintf(buf + n, PAGE_SIZE - n,
"Unable to read LM3697 registers: driver suspended\n");
} else {
printk(KERN_INFO "%s: reading registers\n", __func__);
for (i = 0, n = 0; i < reg_count; i++) {
lm3697_read_reg(client, lm3697_regs[i].reg, &value);
n += scnprintf(buf + n, PAGE_SIZE - n,
"%-20s (0x%x) = 0x%02X\n",
lm3697_regs[i].name, lm3697_regs[i].reg, value);
}
}

return n;
}

static ssize_t lm3697_registers_store (struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t size)
{
struct i2c_client *client = container_of(dev->parent, struct i2c_client,
dev);
struct lm3697_data *driver_data = i2c_get_clientdata(client);
unsigned reg;
unsigned value;

if (atomic_read(&driver_data->suspended)) {
printk(KERN_INFO "%s: can't write: driver suspended\n",
__func__);
return -ENODEV;
}
sscanf(buf, "%x %x", &reg, &value);
if (value > 0xFF)
return -EINVAL;
printk(KERN_INFO "%s: writing reg 0x%x = 0x%x\n", __func__, reg, value);
	mutex_lock(&lm3697_mutex);
lm3697_write_reg(client, reg, (uint8_t)value, __func__);
	mutex_unlock(&lm3697_mutex);

	return size;
}
static DEVICE_ATTR(registers, 0644, lm3697_registers_show, lm3697_registers_store);

static int lm3697_suspend (struct i2c_client *client, pm_message_t mesg)
{
	struct lm3697_data *driver_data = i2c_get_clientdata(client);
	printk ("%s: called with pm message %d\n",
		__func__, mesg.event);

	if (!driver_data) {
		printk ("%s: driver_data is NULL\n", __func__);
		return -ENODEV;
	}

	atomic_set(&driver_data->suspended, 1);

	bbk_backlight_disable();

	if (driver_data->pdata->power_off) {
		driver_data->pdata->power_off();
	}

	if (driver_data->pdata->bkg_enable) {
	    driver_data->pdata->bkg_enable(0);
	}

	return 0;
}

static int lm3697_configure (struct lm3697_data *driver_data)
{
	int ret = 0;

	ret = lm3697_init_registers();
	return ret;
}

static int lm3697_resume (struct i2c_client *client)
{
	struct lm3697_data *driver_data = i2c_get_clientdata(client);
	printk ("%s: resuming\n", __func__);

	if (!driver_data) {
		printk ("%s: driver_data is NULL\n", __func__);
		return -ENODEV;
	}
	if (driver_data->pdata->power_on) {
	    driver_data->pdata->power_on();
	}
	if (driver_data->pdata->bkg_enable) {
	    driver_data->pdata->bkg_enable(1);
	}

	mutex_lock (&lm3697_mutex);
	lm3697_configure (driver_data);
	mutex_unlock (&lm3697_mutex);
	atomic_set(&driver_data->suspended, 0);

	printk ("%s: driver resumed\n", __func__);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void lm3697_early_suspend (struct early_suspend *handler)
{
	struct lm3697_data *driver_data;

driver_data = container_of(handler, struct lm3697_data, early_suspend);
	lm3697_suspend (driver_data->client, PMSG_SUSPEND);
}

static void lm3697_late_resume (struct early_suspend *handler)
{
struct lm3697_data *driver_data;

driver_data = container_of(handler, struct lm3697_data, early_suspend);

	lm3697_resume (driver_data->client);
}
#elif defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	struct lm3697_data *ddata =
		container_of(self, struct lm3697_data, fb_notif);

		printk ("%s lm3697 fb callback\n", __func__);

	if (evdata && evdata->data && event == FB_EVENT_BLANK && ddata) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			lm3697_resume(ddata->client);
		else if (*blank == FB_BLANK_POWERDOWN)
			lm3697_suspend(ddata->client, PMSG_SUSPEND);
	}

	return 0;
}
#endif

#if 0

static void lm3697_pwm_set(int level)
{
	int ret;
	u32 duty;

	if (lm3697_pwm == NULL) {
		pr_err("%s: no PWM\n", __func__);
		return;
	}

/*	duty = level * bl_pwm_period; */
	duty = level;
/*
	pr_err("%s: pwm_period=%d pwm_gpio=%d pwm_lpg_chan=%d\n",
			__func__,  bl_pwm_period,
				bl_pwm_gpio, bl_pwm_lpg_chan);

	pr_err("%s: level=%d duty=%d pwm_enable=%d\n", __func__,
					level, duty, lm3697_pwm_enable);
 */

/*
		if (lm3697_pwm_enable) {
		pwm_disable(lm3697_pwm);
		lm3697_pwm_enable = 0;
	}
 */

	ret = pwm_config(lm3697_pwm, duty, bl_pwm_period);
	if (ret) {
		pr_err("%s: pwm_config() failed err=%d.\n", __func__, ret);
		return;
	}

	ret = pwm_enable(lm3697_pwm);
	if (ret)
		pr_err("%s: pwm_enable() failed err=%d\n", __func__, ret);


	if (level == 0) {
		if (lm3697_pwm_enable) {
			pr_err("pwm disable ,level is %d\n", level);
			pwm_disable(lm3697_pwm);
		}
		pwm_disable(lm3697_pwm);
		lm3697_pwm_enable = 0;
		return;
	}
}
#endif

static int bl_gpio_en = -1;

static int bkg_lm3697_init(void)
{
/*	int rc = -1;

	if (bl_gpio_en == -1) {
		pr_err("%s: unable to get backlight enable gpio\n", __func__);
		return -1;
	}

	rc = gpio_request(bl_gpio_en, "bkg_en");
	if (rc) {
		 pr_err("'%s'(%d) gpio_request failed, rc=%d\n", "bkg_en", bl_gpio_en, rc);
		 goto error;
	}

	rc = gpio_direction_output(bl_gpio_en, 1);
	if (rc) {
		pr_err("%s: Unable to set direction\n", __func__);
		goto error;
	}
	return 0;
error:
	gpio_free(bl_gpio_en); */
	return 0;
}
static int bkg_lm3697_enable(unsigned en)
{
/*	int rc = -1;
	static unsigned bkg_config = 0;

	if (!(bkg_config || en)) {
		rc = gpio_tlmm_config(GPIO_CFG  (bl_gpio_en, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("'%s'(%d) gpio_config failed, rc=%d\n", "bkg_en", bl_gpio_en, rc);
			goto error1;
		}
		bkg_config = 1;
	}

	rc = gpio_direction_output(bl_gpio_en, en);
	if (rc) {
		pr_err("%s: Unable to set direction\n", __func__);
		return rc;
	}
error1: */
	return 0;
}

static int lm3697_parse_dt(struct device *dev, struct lm3697_platform_data *pdata)
{
	int rc;

	struct device_node *np = dev->of_node;
	u32 temp_val;
	u32 bl_gpio_en_flags;


	if (dev == NULL) {
		printk ("%s: dev is NULL\n", __func__);
		return -EINVAL;
	}

	if (pdata == NULL) {
		printk ("%s: pdata is NULL\n", __func__);
		return -EINVAL;
	}

	/* reset, irq gpio info */
	bl_gpio_en = of_get_named_gpio_flags(np, "lm3697,bl-enable-gpio",
				0, &bl_gpio_en_flags);

	rc = of_property_read_u32(np, "lm3697, ctrl_a_fs_current", &temp_val);
	if (rc && (rc != -EINVAL))
		dev_err(dev, "Unable to read attn_polarity value\n");
	else if (rc != -EINVAL)
		pdata->ctrl_a_fs_current = (unsigned)temp_val;
	rc = of_property_read_u32(np, "lm3697, ctrl_b_fs_current", &temp_val);
	if (rc && (rc != -EINVAL))
		dev_err(dev, "Unable to read attn_polarity value\n");
	else if (rc != -EINVAL)
		pdata->ctrl_b_fs_current = (unsigned)temp_val;
	rc = of_property_read_u32(np, "lm3697, ctrl_a_mapping_mode", &temp_val);
	if (rc && (rc != -EINVAL))
		dev_err(dev, "Unable to read attn_polarity value\n");
	else if (rc != -EINVAL)
		pdata->ctrl_a_mapping_mode = (unsigned)temp_val;
	rc = of_property_read_u32(np, "lm3697, ctrl_b_mapping_mode", &temp_val);
	if (rc && (rc != -EINVAL))
		dev_err(dev, "Unable to read attn_polarity value\n");
	else if (rc != -EINVAL)
		pdata->ctrl_b_mapping_mode = (unsigned)temp_val;
	rc = of_property_read_u32(np, "lm3697, ctrl_a_pwm", &temp_val);
	if (rc && (rc != -EINVAL))
		dev_err(dev, "Unable to read attn_polarity value\n");
	else if (rc != -EINVAL)
		pdata->ctrl_a_pwm = (unsigned)temp_val;
	rc = of_property_read_u32(np, "lm3697, ctrl_b_pwm", &temp_val);
	if (rc && (rc != -EINVAL))
		dev_err(dev, "Unable to read attn_polarity value\n");
	else if (rc != -EINVAL)
		pdata->ctrl_b_pwm = (unsigned)temp_val;

	pdata->init = bkg_lm3697_init;
	pdata->bkg_enable = bkg_lm3697_enable;

	return 0;
}


/* This function is called by i2c_probe */
static int lm3697_probe (struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
	struct lm3697_platform_data *pdata = NULL;
	struct lm3697_data *driver_data;

	printk ("%s: enter \n", __func__);

	printk (KERN_INFO "%s: enter, I2C address = 0x%x, flags = 0x%x\n",
		__func__, client->addr, client->flags);

	if (!client) {
		printk ("%s: client is NULL\n", __func__);
		return -EINVAL;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct lm3697_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		ret = lm3697_parse_dt(&client->dev, pdata);
		if (ret)
			return ret;
	} else {
		pdata = client->dev.platform_data;
	}

	if (pdata == NULL) {
		printk ("%s: platform data required\n", __func__);
		return -EINVAL;
	}

	/* We should be able to read and write byte data */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk ("%s: I2C_FUNC_I2C not supported\n",
			__func__);
		return -ENOTSUPP;
	}
	driver_data = kzalloc(sizeof(struct lm3697_data), GFP_KERNEL);
	if (driver_data == NULL) {
		printk ("%s: kzalloc failed\n", __func__);
		return -ENOMEM;
	}
	memset (driver_data, 0, sizeof (*driver_data));

	driver_data->client = client;
	driver_data->pdata = pdata;
	if (driver_data->pdata->ctrl_a_fs_current > 0xFF)
		driver_data->pdata->ctrl_a_fs_current = 0xFF;
	if (driver_data->pdata->ctrl_b_fs_current > 0xFF)
		driver_data->pdata->ctrl_b_fs_current = 0xFF;

	i2c_set_clientdata (client, driver_data);

	/* Initialize chip */
	if (pdata->init) {
		pdata->init();
	}

	if (pdata->power_on) {
		pdata->power_on();
	}

	 /* Register LED class */
	driver_data->led_dev.name = LM3697_LED_NAME;
	driver_data->led_dev.brightness_set = lm3697_backlight_set;
	ret = led_classdev_register (&client->dev, &driver_data->led_dev);
	if (ret) {
		 printk ("%s: led_classdev_register %s failed: %d\n",
		   __func__, LM3697_LED_NAME, ret);
		/*goto led_register_failed; */
	}

	atomic_set(&driver_data->in_suspend, 0);
	atomic_set(&driver_data->suspended, 0);

	ret = device_create_file(driver_data->led_dev.dev, &dev_attr_registers);
	if (ret) {
	  printk ("%s: unable to create registers device file for %s: %d\n",
			__func__, LM3697_LED_NAME, ret);
		goto device_create_file_failed2;
	}

	dev_set_drvdata (&client->dev, driver_data);

#ifdef CONFIG_HAS_EARLYSUSPEND
	driver_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 5;
	driver_data->early_suspend.suspend = lm3697_early_suspend,
	driver_data->early_suspend.resume = lm3697_late_resume,
	register_early_suspend (&driver_data->early_suspend);
#elif defined(CONFIG_FB)
	driver_data->fb_notif.notifier_call = fb_notifier_callback;
#endif

	/* Initialize brightness to whatever bootloader wrote into the register */
/*	lm3697_read_reg(client, LM3697_CTRL_A_ZT5_REG, */
/*		(uint8_t *)&driver_data->led_dev.brightness); */



	lm3697_client = client;
	driver_data->initialized = 1;
	return 0;

device_create_file_failed2:
	device_remove_file (driver_data->led_dev.dev, &dev_attr_registers);
/*setup_failed: */
	kfree (driver_data);
	return ret;
}

static int lm3697_remove (struct i2c_client *client)
{
	struct lm3697_data *driver_data = i2c_get_clientdata(client);

	device_remove_file (driver_data->led_dev.dev, &dev_attr_registers);
	kfree (driver_data);
	return 0;
}

static const struct i2c_device_id lm3697_id[] = {
	{ LM3697_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static struct of_device_id lm3697_match_table[] = {
	{ .compatible = "lm3697,lm3697_i2c", },
	{ },
};
#else
#define KTD2026_MATCH_TABLE NULL
#endif

/* This is the I2C driver that will be inserted */
static struct i2c_driver lm3697_driver = {
	.driver = {
		.owner	 = THIS_MODULE,
		.name = LM3697_NAME,
		.of_match_table = lm3697_match_table,
	},
	.id_table = lm3697_id,
	.probe = lm3697_probe,
	.remove = lm3697_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
#if !defined(CONFIG_FB)
	.suspend = lm3697_suspend,
	.resume = lm3697_resume,
#endif
#endif
};
module_i2c_driver(lm3697_driver);
MODULE_DESCRIPTION("LM3697 DISPLAY BACKLIGHT DRIVER");
MODULE_LICENSE("GPL v2");
