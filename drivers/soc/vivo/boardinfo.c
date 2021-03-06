/*
 * Copyright (C) 2016 vivo Co., Ltd.
 * YangChun <yangchun@vivo.com.cn>
 *
 * This driver is used to export hardware board information for users
 *
**/

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/module.h>
#include <linux/err.h>
#include <linux/sys_soc.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>
#include <soc/qcom/socinfo.h>
#include <soc/qcom/smem.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#define BOARD_REV_LEN 16
#define BOARD_NAME_LEN 24
#define VIVO_VENDOR_LEN 8
#define VIVO_CPU_TYPE_LEN 8
#define FREQ_STR_LEN 8
#define INVALID_CPU_FREQ "0"
#define INVALID_CPU_TYPE "unkown"
#define CPU_REVISION_ADDR 0x000A607C
#define VIVO_HW_VERSION_MASK (0xF<<28)

//store in shared memory
struct boardinfo_smem{
	uint32_t board_id;
	char board_rev[BOARD_REV_LEN];
	uint32_t size;
	uint32_t type;
	char board_name[BOARD_NAME_LEN];
	uint32_t ddr_manufacture_id;
	uint32_t lcm_id;
	uint32_t dp_status;	/*add wuyuanhan, dp image load or not.*/
	uint32_t reserved;//make len as a multiple of 8
} *boardinfo_smem;

struct boardinfo_ext{
	char vendor[VIVO_VENDOR_LEN];
	unsigned int cpu_freq;
	char cpu_type[VIVO_CPU_TYPE_LEN];
    char user_cpu_freq[FREQ_STR_LEN];				// max cpu_freq|string|unit GHz|for setting app display to user... 2018/10/25  wuyuanhan, 
	unsigned int core_num;
} *boardinfo_ext;

typedef struct freq_base_map
{
	uint32_t board_id;
	uint32_t act_freq;
	char user_freq[FREQ_STR_LEN];
    char cpu_type[VIVO_CPU_TYPE_LEN];
} freq_base_map_t;

#if 0

维护原则:
1、用户显示CPU频率来源于cpu芯片手册 /sys/bus/soc/devices/soc1/user_cpu_freq，单位GHz.
2、实际CPU工作频率，以/sys/bus/cpu/devices/cpu7/cpufreq/cpuinfo_max_freq & /sys/bus/soc/devices/soc1/cpu_freq 为准
3、freq_maps 映射表支持一个boardid，使用不同频率不同CPU的情况。
4、每个Boarid 都需要配置，不然读取到的CPU频率为0，CPU型号为 unkonw.
5、map表匹配原则:
   5.1 匹配boardid，未找到 设置无效频率"0"
   5.2 只匹配到boardid，未配置到实际工作频率cpu,获取默认显示cpu频率，CPU型号信息
   5.3 匹配到boardid&配置到实际工作频率cpu,对应实际频率cpu频率的显示CPU频率，CPU型号信息

/* -----------------  1.  该平台涉及到的所有CPU手册 CPU工作频率信息                   ----------------------- */
SDM660:
4xKryo Gold 2.2GHz 1MB L2 + 
4xKryo Silver 1.8GHz 1MB L2

SDM660-3:
4xKryo Gold 2.2GHz 1MB L2 + 
4xKryo Silver 1.8GHz 1MB L2

SDM630:
4xA53 2.2GHz 1MB L2 + 
4xA53 1.8GHz 512KB L2

SDM636:
4xKryo Gold 1.8GHz 1MB L2 + 
4xKryo Silver 1.6GHz 512KB L2

/* -----------------  2.  该平台xbl/sbl下 board_id 信息拷贝，新增board_id时，需要同步更新  ------------------ */
static vivo_board_id_t board_ids[] = {
	{0,"SDM660",},
	{2,"VTD1702",},
	{3,"VTD1702F_EX",},
	{4,"PD1728",},
	{5,"PD1728F_EX",},
	{6,"PD1729",},
	{7,"PD1729F_EX",},
	{8,"PD1709",},
	{9,"PD1710",},
	{10,"PD1709F_EX",},
	{11,"PD1710F_EX",},
	{12,"VTD1801",},
	{20,"PD1730C",},
	{21,"PD1730CF_EX",},
	{22,"PD1814F_EX",},
	{23,"PD1816",},
	{24,"PD1814",},
	{25,"PD1730G",}, //sdm660-3
};
#endif

	
#if 0
660是2.2,660-3是1.95, 636是1.8
1728是+32,1816是+64
sdm660 2.2: 2208000
sdm660-3 1.95:1958400
sdm636 1.8: 1804800
660-3有：1814、1814f、1816、1730DF
636：1730D
其他都是660；
1730C、1730D是共board id; 
1730CF、1730DF是共board id, 但与1730C不同
#endif

static freq_base_map_t freq_maps[] = 
{
	{0, 0, "2.2", "660"},
	{2, 0, "2.2", "660"},//VTD1702
	{3, 0, "2.2", "660"},//VTD1702F_EX
	{4, 0, "2.2", "660"},//PD1728
	{5, 0, "2.2", "660"},//PD1728F_EX
	{6, 0, "2.2", "660"},//PD1729
	{7, 0, "2.2", "660"},//PD1729F_EX
	{8, 0, "2.2", "660"},//PD1709
	{9, 0, "2.2", "660"},//PD1710
	{10, 0, "2.2", "660"},//PD1709F_EX
	{11, 0, "2.2", "660"},//PD1710F_EX
	{12, 0, "2.2", "660"},//
	{20, 2208000, "2.2", "660"},//PD1730C 660
	{20, 1804800, "1.8", "636"},//PD1730D 636
	{21, 2208000, "2.2", "660"},//PD1730CF_EX 660
	{21, 1958400, "1.95", "660"},//PD1730DF_EX 660-3
	{22, 0, "1.95", "660"},//PD1814F_EX 660-3
	{23, 0, "1.95", "660"},//PD1816 660-3
	{87, 0, "1.95", "660"},//PD1816B 660-3
	{24, 0, "1.95", "660"},//PD1814 660-3
	{36, 0, "2.2", "660"},//PD1728B
	{37, 0, "2.2", "660"},//PD1728BF_EX
	{25, 0, "1.95", "660"},//PD1730G 660-3
};

static ssize_t vivo_show_board_id(struct device *dev, struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n",boardinfo_smem->board_id);
}

static ssize_t vivo_show_board_name(struct device *dev,struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",boardinfo_smem->board_name);
}
static ssize_t vivo_show_ddr_manufacture_id(struct device *dev,struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n",boardinfo_smem->ddr_manufacture_id);
}
static ssize_t vivo_show_vendor(struct device *dev,struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",boardinfo_ext->vendor);
}
static ssize_t vivo_show_cpu_freq(struct device *dev,struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n",boardinfo_ext->cpu_freq);
}
static ssize_t vivo_show_cpu_type(struct device *dev,struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n",boardinfo_ext->cpu_type);
}
static ssize_t vivo_show_user_cpu_freq(struct device *dev,struct device_attribute *attr,char *buf)
{ 
    return snprintf(buf, PAGE_SIZE, "%s\n",boardinfo_ext->user_cpu_freq);
}

static ssize_t vivo_show_core_num(struct device *dev,struct device_attribute *attr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n",boardinfo_ext->core_num);
}

static struct device_attribute vivo_vendor = __ATTR(vendor, S_IRUGO, vivo_show_vendor,  NULL);

static struct device_attribute vivo_board_id = __ATTR(board_id, S_IRUGO,vivo_show_board_id, NULL);

static struct device_attribute vivo_board_name = __ATTR(board_name, S_IRUGO,vivo_show_board_name, NULL);

static struct device_attribute vivo_ddrinfo = __ATTR(ddrinfo, S_IRUGO, vivo_show_ddr_manufacture_id,  NULL);

static struct device_attribute vivo_cpu_freq = __ATTR(cpu_freq, S_IRUGO,vivo_show_cpu_freq, NULL);

static struct device_attribute vivo_cpu_user_freq = __ATTR(user_cpu_freq, S_IRUGO,vivo_show_user_cpu_freq, NULL);

static struct device_attribute vivo_cpu_type = __ATTR(cpu_type, S_IRUGO, vivo_show_cpu_type, NULL);	

static struct device_attribute vivo_core_num = __ATTR(core_num, S_IRUGO, vivo_show_core_num, NULL);			

static void __init populate_soc_sysfs_files(struct device *vivo_soc_device)
{
	device_create_file(vivo_soc_device, &vivo_board_id);
	device_create_file(vivo_soc_device, &vivo_board_name);
	device_create_file(vivo_soc_device, &vivo_vendor);
	device_create_file(vivo_soc_device, &vivo_ddrinfo);
	device_create_file(vivo_soc_device, &vivo_cpu_freq);
    device_create_file(vivo_soc_device, &vivo_cpu_user_freq);
	device_create_file(vivo_soc_device, &vivo_cpu_type);
	device_create_file(vivo_soc_device, &vivo_core_num);
	return;
}
	
static unsigned int vivo_get_cpu_freq(void)
{
	int cpu_id = 0;
	unsigned int max_freq = 0;
	unsigned int cur_freq = 0;
	int num_cpus = num_possible_cpus();
	for (cpu_id = 0; cpu_id < num_cpus; cpu_id++) {
		cur_freq = cpufreq_quick_get_max(cpu_id);
		if (cur_freq > max_freq) {
			max_freq = cur_freq;
		}
	}

	return max_freq;
}

static void get_user_cpu_freq_and_type(void ){
    
	int i = 0;
	int default_index = -1;
 
	for (i = 0; i < (sizeof (freq_maps) / sizeof (freq_maps[0])); i++) {
		if (freq_maps[i].board_id == boardinfo_smem->board_id) {
			if ((freq_maps[i].act_freq == boardinfo_ext->cpu_freq) && (freq_maps[i].act_freq != 0)) {				
				pr_err("vivo board_info:Set user cpu max freq : %sGHz\n", freq_maps[i].user_freq); 
				strncpy(boardinfo_ext->user_cpu_freq, freq_maps[i].user_freq, FREQ_STR_LEN);
                strncpy(boardinfo_ext->cpu_type, freq_maps[i].cpu_type, VIVO_CPU_TYPE_LEN);
                return;
			} else if (freq_maps[i].act_freq == 0) {
			    default_index = i;
			}
		}
	}
	
	if (default_index >= 0) {
		pr_err("vivo board_info:Set user cpu max freq : %sGHz\n", freq_maps[i].user_freq);
        strncpy(boardinfo_ext->user_cpu_freq, freq_maps[default_index].user_freq, VIVO_CPU_TYPE_LEN);
        strncpy(boardinfo_ext->cpu_type, freq_maps[default_index].cpu_type, VIVO_CPU_TYPE_LEN);
		return;
	} else {
	    strncpy(boardinfo_ext->user_cpu_freq, INVALID_CPU_FREQ, FREQ_STR_LEN);
        strncpy(boardinfo_ext->cpu_type, INVALID_CPU_TYPE, VIVO_CPU_TYPE_LEN);
		pr_err("vivo board_info: error: Need to set cpu max freq for user!!!\n"); 
		return;
	}
}

static void vivo_boardinfo_ext_init(void){
	
	boardinfo_ext = kzalloc(sizeof(*boardinfo_ext), GFP_KERNEL);
	if (!boardinfo_ext) {
		pr_err("boardinfo_ext alloc failed!\n");
		return;
	}
    //cpu max frequency
    boardinfo_ext->cpu_freq = vivo_get_cpu_freq();
    
	//user frequency & type of cpu
	get_user_cpu_freq_and_type();
	//core number
	boardinfo_ext->core_num = num_possible_cpus();
	
	//vendor
	strncpy(boardinfo_ext->vendor,"vivo",VIVO_VENDOR_LEN); //vivo
	pr_err("vivo cpu_freq:%u user_cpu_freq:%sGHz core_num=%u cpu_type=%s\n",boardinfo_ext->cpu_freq,
            boardinfo_ext->user_cpu_freq,
			boardinfo_ext->core_num,
			boardinfo_ext->cpu_type);
}
static int __init vivo_boardinfo_init_sysfs(void)
{
	struct device *vivo_soc_device;
	struct soc_device *soc_dev;
	struct soc_device_attribute *soc_dev_attr;

	if (!boardinfo_smem) {
		pr_err("No boardinfo found!\n");
		return -ENODEV;
	}

	soc_dev_attr = kzalloc(sizeof(*soc_dev_attr), GFP_KERNEL);
	if (!soc_dev_attr) {
		pr_err("Soc Device alloc failed!\n");
		return -ENOMEM;
	}

	soc_dev = soc_device_register(soc_dev_attr);
	if (IS_ERR_OR_NULL(soc_dev)) {
		kfree(soc_dev_attr);
		pr_err("Soc device register failed\n");
		return -EIO;
	}
		
	vivo_soc_device = soc_device_to_device(soc_dev);
	
	populate_soc_sysfs_files(vivo_soc_device);
	
	//extra information init
	vivo_boardinfo_ext_init();
	
	return 0;
}
late_initcall(vivo_boardinfo_init_sysfs);

static void vivo_boardinfo_print(void)
{
	pr_info("board_id=%d, board_version=%s, type=%d, board_name:%s\n",
		boardinfo_smem->board_id, boardinfo_smem->board_rev, 
		boardinfo_smem->type, boardinfo_smem->board_name);
}

int __init vivo_boardinfo_init(void)
{
	static bool boardinfo_init_done;
	unsigned size;
	if (boardinfo_init_done)
		return 0;

	boardinfo_smem = smem_get_entry(SMEM_ID_VENDOR0, &size, 0,
				 SMEM_ANY_HOST_FLAG);

	if (IS_ERR_OR_NULL(boardinfo_smem))
		BUG_ON("Can't find SMEM_ID_VENDOR0 for vivo boardinfo!\n");

	vivo_boardinfo_print();
	boardinfo_init_done = true;
	return 0;
}
subsys_initcall(vivo_boardinfo_init);
