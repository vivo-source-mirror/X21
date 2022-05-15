#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/serial_core.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#include "max989xx-debug-common.h"

#define AM_DEV_NAME   "max989xx"

struct max989xx_msg {
	char msgs[256];
	char reserved[252];
	int msg_result;
};

struct max989xx_prars {
	int fRes_max;
	int fRes_min;
	int Qt;
	int impedance_max;
	int impedance_min;
};
#define TFA_CTL_IOC_MAGIC  'T'
#define TFA_IOCTL_SPK_REST  _IOW(TFA_CTL_IOC_MAGIC, 0x01, int)
#define TFA_IOCTL_SPK_INTS   _IOR(TFA_CTL_IOC_MAGIC, 0x02, struct max989xx_msg)
#define TFA_IOCTL_SPK_INTT  _IOW(TFA_CTL_IOC_MAGIC, 0x03, int)
#define TFA_IOCTL_SPK_RFDES 	_IOR(TFA_CTL_IOC_MAGIC, 0x04, struct max989xx_msg)
#define TFA_IOCTL_SPK_CHCK _IOR(TFA_CTL_IOC_MAGIC, 0x05, int)
#define TFA_IOCTL_SPK_PRARS _IOR(TFA_CTL_IOC_MAGIC, 0x06, struct max989xx_prars)
#define TFA_IOCTL_SPK_ADDR  _IOW(TFA_CTL_IOC_MAGIC, 0x07, unsigned char)
#define TFA_IOCTL_SPK_MTP_BACKUP _IOR(TFA_CTL_IOC_MAGIC, 0x08, int)

//extern int max989xx_reset_mtp_dbg(void);
//extern int max989xx_check_mtp_dbg(void);
extern int max989xx_init_dbg(char *buffer, int size);
extern int max989xx_read_freq_dbg(char *buffer, int size);
extern void max989xx_read_prars_dbg(int temp[5], unsigned char addr);
extern void max989xx_get_client(struct i2c_client **client, unsigned char addr);
//extern int max989xx_debug_mtp_backup(int force);

static struct i2c_client * max989xx_debug_client = NULL;
static unsigned char last_addr = 0;

static ssize_t max989xx_debug_read (struct file *file,
                                    char __user *buf, size_t count, loff_t *offset)
{
	char *tmp;
	int ret;

	max989xx_get_client(&max989xx_debug_client, last_addr);
	tmp = kmalloc(count, GFP_KERNEL);
	if (tmp == NULL)
		return -ENOMEM;

	ret = i2c_master_recv(max989xx_debug_client, tmp, count);
	if (ret >= 0)
		ret = copy_to_user(buf, tmp, count) ? -EFAULT : ret;
	else printk("[MAX989XX]%s: transfer error %d\n", __func__, ret);
	kfree(tmp);
	return ret;
}

static ssize_t max989xx_debug_write (struct file *file,
                                     const char __user *buf, size_t count, loff_t *offset)
{
	char *tmp;
	int ret;

	max989xx_get_client(&max989xx_debug_client, last_addr);
	tmp = memdup_user(buf, count);
	if (IS_ERR(tmp))
		return PTR_ERR(tmp);
	ret = i2c_master_send(max989xx_debug_client, tmp, count);
	if(ret < 0)
		printk("[MAX989XX]%s: transfer error %d\n", __func__, ret);
	kfree(tmp);
	return ret;
}
static long  max989xx_debug_ioctl (struct file *file,
                                   unsigned int cmd, unsigned long arg)
{
	int  ret = 0, check = 0;
	int temp[5];
	struct max989xx_msg msg;
	struct max989xx_msg __user *_msg = (struct max989xx_msg __user *)arg;
	struct max989xx_prars prars;
	struct max989xx_prars __user *_prars = (struct max989xx_prars __user *)arg;

	memset(&prars, 0, sizeof(struct max989xx_prars));
	memset(&msg, 0, sizeof(struct max989xx_msg));
	switch(cmd) {
	/* Reset MTP */
	case TFA_IOCTL_SPK_REST:
		printk("max989xx_ioctl SPK_REST\n");
		//max989xx_reset_mtp_dbg();
		break;
	/* calibrate */
	case TFA_IOCTL_SPK_INTS:
		printk("max989xx_ioctl SPK_INTS\n");
		check = max989xx_init_dbg((char *)&msg, sizeof(struct max989xx_msg));
		msg.msg_result = check;
		ret = copy_to_user((char __user *)_msg, &msg, sizeof(struct max989xx_msg));
		break;
	case TFA_IOCTL_SPK_INTT:

		printk("max989xx_ioctl SPK_INT\n");
		break;
	case TFA_IOCTL_SPK_RFDES:
		usleep_range(10*1000, 10*1000);
		printk("max989xx_ioctl SPK_ReadFDes\n");
		ret = max989xx_read_freq_dbg((char *)&msg, sizeof(struct max989xx_msg));
		ret = copy_to_user((char __user *)_msg, &msg, sizeof(struct max989xx_msg));
		break;
	/* checkmtp */
	case TFA_IOCTL_SPK_CHCK:
		printk("max989xx_ioctl SPK Check MtpEx\n");
		//check = max989xx_check_mtp_dbg();
		pr_info("%s check %d.\n", __func__, check);
		//ret = copy_to_user((__user int*)arg, &check, sizeof(int));
		break;
	case TFA_IOCTL_SPK_PRARS:
		printk("max989xx_ioctl SPK Read f0 and Qt\n");
		max989xx_read_prars_dbg(temp, last_addr);
		prars.fRes_max = temp[0];
		prars.fRes_min = temp[1];
		prars.Qt = temp[2];
		prars.impedance_max = temp[3];
		prars.impedance_min = temp[4];

		ret = copy_to_user((char __user *)_prars, &prars, sizeof(struct max989xx_prars));
		pr_info("max989xx_ioctl %d %d %d\n",temp[0],temp[1],temp[2]);
		break;
	case TFA_IOCTL_SPK_ADDR:
		ret = copy_from_user(&last_addr, (void __user *)arg, sizeof(unsigned char));
		printk("max989xx_ioctl addr %x\n", last_addr);
		break;
	case TFA_IOCTL_SPK_MTP_BACKUP:
		//	check = max989xx_debug_mtp_backup(0);
		pr_info("%s mtp backup %d.\n", __func__, check);
	//	ret = copy_to_user((__user int*)arg, &check, sizeof(int));
	default:
		printk("max989xx Fail IOCTL command no such ioctl cmd = %x\n", cmd);
		ret = -1;
		break;
	}

	return ret;
}

static int max989xx_debug_open(
    struct inode *inode, struct file *file)
{
	printk("[MAX989XX]%s\n", __func__);
	return 0;
}

int max989xx_debug_release(
    struct inode *inode, struct file *file)
{
	printk("[MAX989XX]%s\n", __func__);
	return 0;
}

static const struct file_operations max989xx_debug_fileops = {
	.owner = THIS_MODULE,
	.open  = max989xx_debug_open,
	.read  = max989xx_debug_read,
	.write = max989xx_debug_write,
	.unlocked_ioctl = max989xx_debug_ioctl,
	.release = max989xx_debug_release,
};

static struct miscdevice max989xx_debug_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AM_DEV_NAME,
	.fops = &max989xx_debug_fileops,
};

int max989xx_debug_probe(struct i2c_client *client)
{
	int err = 0;

	printk("%s\n",__func__);

	if (max989xx_debug_client)
		return 0;

	err = misc_register(&max989xx_debug_device);
	if (err) {
		printk("%s: max989xx_device register failed\n", __func__);
		return err;
	}

	max989xx_debug_client = client;

	return 0;
}

MODULE_DESCRIPTION("max989xx debug driver");
MODULE_AUTHOR("chenjinquan <chenjinquan@vivo.com>");
MODULE_LICENSE("GPL");
