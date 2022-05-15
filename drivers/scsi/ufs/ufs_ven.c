#include <linux/async.h>
#include <scsi/ufs/ioctl.h>
#include <linux/nls.h>
#include <linux/of.h>
#include <linux/blkdev.h>
#include <linux/sort.h>
#include "ufshcd.h"
#include "ufshci.h"
#include "ufs_quirks.h"
#include "ufs-debugfs.h"
#include "ufs-qcom.h"
#include "ufs_ven.h"

#ifdef CONFIG_UFS_IO_STATISTICS

extern int ufshcd_read_health_desc(struct ufs_hba *hba, u8 *buf, u32 size);

static void ufshcd_io_stats_init_intern(struct ufs_hba *hba);

/* static io_ele_t cmd_buf[UFS_STAT_CMD_COUNT * sizeof(io_ele_t)]; */
/* static u16 idx_buf[UFS_STAT_CMD_COUNT]; */
static io_ele_t *cmd_buf;
static u16 *idx_buf;

unsigned int ufs_support_size[] = { /* sector unit, refer to UFS_STAT_SIZE_MAX */
		4	* 1024 / 512,
		512	* 1024 / 512,
};

unsigned int ufs_support_gear[UFS_STAT_GEAR_MAX] = {
	UFS_HS_G1,		/* HS Gear 1 */
	UFS_HS_G3,		/* HS Gear 3 */
};

unsigned char *ufs_support_gear_str[UFS_STAT_GEAR_MAX] = {
	"UFS_HS_G1",
	"UFS_HS_G3",
};

unsigned char *ufs_support_size_str[UFS_STAT_SIZE_MAX] = {
	"4K",
	"512K",
	"OTHR"
};

unsigned char *ufs_support_cmd_type_str[CMD_TYPE_MAX] = { /* refer to cmd_types */
	"r",
	"w",
	"d",
	"f",
};

static int ufshcd_io_stats_uevent_report(struct ufs_hba *hba,
				unsigned int gear, unsigned int size, int cmd_type,
				char *except_msg, char *msg)
{
	/* char envp_ext[4][MAX_ENV_LEN]; */
	/* char *envp[4] = {NULL}; */
	/* int idx = 0; */

	dev_err(hba->dev, "[%s:%d] ufs_io [%s][%s][%s] exception=%s, %s\n",
			__func__, __LINE__,
			ufs_support_gear_str[gear],
			ufs_support_cmd_type_str[cmd_type],
			ufs_support_size_str[size],
			except_msg,
			msg);

	/* snprintf(envp_ext[0], MAX_ENV_LEN, "subsystem=kernel_monitor_engine"); */
	/* snprintf(envp_ext[1], MAX_ENV_LEN, "exception=%s", except_msg); */
	/* snprintf(envp_ext[2], MAX_ENV_LEN, msg); */
	/* for (idx = 0; idx < ARRAY_SIZE(envp); idx++) */
		/* envp[idx] = envp_ext[idx]; */
	/* return kernel_monitor_report(envp); */
	return 0;
}

static inline unsigned int ufs_io_stats_size(struct request *rq)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ufs_support_size); i++)
		if (blk_rq_sectors(rq) == ufs_support_size[i])
			break;
	return i;
}

void ufshcd_io_stats(struct ufs_hba *hba, struct ufshcd_lrb *lrbp)
{
	unsigned long flags;
	io_stats_info_t *stat;
	io_stats_cmd_t *stat_cmd;
	io_ele_t ce;
	struct scsi_cmnd *cmd = lrbp->cmd;
	struct request *rq = cmd ? cmd->request : NULL;
	char cmd_op;
	int cmd_type;
	unsigned int size, gear, rq_size;
	ktime_t delta;
	s64 time;

	if (!rq || !cmd)
		return;

	for (gear = 0; gear < ARRAY_SIZE(ufs_support_gear); gear++)
		if (hba->pwr_info.gear_tx == ufs_support_gear[gear])
			break;
	if (gear == ARRAY_SIZE(ufs_support_gear)) {
		dev_err(hba->dev, "[%s:%d] Gear %d is not supported.\n",
				__func__, __LINE__, gear);
		return;
	}

	cmd_op = cmd->cmnd[0];
	size = ufs_io_stats_size(rq);
	rq_size = blk_rq_sectors(rq);
	if (cmd_op == READ_6 || cmd_op == READ_10 || cmd_op == READ_16)
		cmd_type = CMD_READ;
	else if (cmd_op == WRITE_6 || cmd_op == WRITE_10 || cmd_op == WRITE_16)
		cmd_type = CMD_WRITE;
	else if (cmd_op == UNMAP) {
		cmd_type = CMD_DISCARD;
		size = UFS_STAT_SIZE_OTHER;
	} else if (cmd_op == SYNCHRONIZE_CACHE) {
		cmd_type = CMD_FLUSH;
		size = UFS_STAT_SIZE_OTHER;
		rq_size = 0;
	} else {
		/* dev_err(hba->dev, "[%s:%d] cmd %#x is not supported.\n", */
				/* __func__, __LINE__, cmd_op); */
		return;
	}

	delta = ktime_sub(lrbp->complete_time_stamp,
			lrbp->issue_time_stamp);
	if (ktime_to_ms(delta)  > UFS_STAT_LATENCY_MAX_TIME_MS) {
		char msg[50];

		snprintf(msg, sizeof(msg), "latency=%lldms",
				ktime_to_ms(delta));
		ufshcd_io_stats_uevent_report(hba, gear, size, cmd_type,
				"latency_exception_single_cmd_time", msg);
	}

	if (hba->io_stats.stat_cmd_enable) {
		ce.op = cmd_op;
		ce.lun = lrbp->lun;
		ce.lba = blk_rq_pos(rq);
		if (cmd_type == CMD_FLUSH)
			ce.lba = 0;
		ce.cmd_type = cmd_type;
		ce.size = rq_size;
		ce.time = delta;
		ce.tstamp = lrbp->issue_time_stamp;
	}
	spin_lock_irqsave(&hba->io_stats.lock, flags);
	stat = &hba->io_stats.stat[gear][size][cmd_type];
	stat->io_cnt++;
	stat->size_cnt += rq_size;
	if (ktime_compare(delta, stat->lat_max) > 0)
		stat->lat_max = delta;
	if (ktime_compare(delta, stat->lat_min) < 0)
		stat->lat_min = delta;
	stat->lat = ktime_add(stat->lat, delta);

	if (hba->io_stats.stat_cmd_enable) {
		stat_cmd = &hba->io_stats.stat_cmd;
		stat_cmd->cmd[stat_cmd->pos] = ce;
		stat_cmd->pos = (stat_cmd->pos + 1) % UFS_STAT_CMD_COUNT;
	}

	spin_unlock_irqrestore(&hba->io_stats.lock, flags);

	if (hba->io_stats.log_enable) {
		time = ktime_to_us(stat->lat);
		pr_err("ufs_io [%s][%s][%s] sz %u t %lld avg %lld\n",
				ufs_support_gear_str[gear],
				ufs_support_cmd_type_str[cmd_type],
				ufs_support_size_str[size],
				rq_size * 512,
				ktime_to_us(delta),
				time / stat->io_cnt
			   );
	}
}

static ssize_t
show_io_stats(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	unsigned long flags;
	int i, j, k;
	struct seq_file m;
	io_stats_info_t stat;
	s64 time;

	memset(&m, 0, sizeof(struct seq_file));
	m.buf = buf;
	m.size = PAGE_SIZE;

	seq_printf(&m, "%-17s: %d\n%-17s: %d\n%-17s: %d\n\n",
			"stat_cmd_enable",
			hba->io_stats.stat_cmd_enable,
			"log_enable",
			hba->io_stats.log_enable,
			"load_thre",
			hba->io_stats.load_thre
			);

	seq_printf(&m, "%-20s %16s %16s %16s %16s %16s\n",
			"Item", "IO Count", "Size",
			"Latency Avg(us)", "Latency Max(us)", "Latency Min(us)");
	for (i = 0; i < UFS_STAT_GEAR_MAX; i++)
		for (k = 0; k < CMD_TYPE_MAX; k++)
			for (j = 0; j < UFS_STAT_SIZE_MAX; j++) {
				spin_lock_irqsave(&hba->io_stats.lock, flags);
				stat = hba->io_stats.stat[i][j][k];
				spin_unlock_irqrestore(&hba->io_stats.lock, flags);

				if ((((k == CMD_DISCARD) || (k == CMD_FLUSH))
							&& (j != UFS_STAT_SIZE_OTHER))
						|| (stat.io_cnt == 0))
					continue;

				time = ktime_to_us(stat.lat);
				seq_printf(&m,
						"[%s][%s][%4s] %16llu %16llu %16lld %16lld %16lld\n",
						ufs_support_gear_str[i],
						ufs_support_cmd_type_str[k],
						ufs_support_size_str[j],
						stat.io_cnt,
						stat.size_cnt * 512,
						time / stat.io_cnt,
						ktime_to_us(stat.lat_max),
						ktime_to_us(stat.lat_min)
						);
			}

	return m.count;
}

static ssize_t
set_io_stats(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	int64_t value;

	if (kstrtoll(buf, 0, &value))
		return 0;

	if (!value) {
		ufshcd_io_stats_init_intern(hba);
	} else
		dev_err(hba->dev, "[%s:%d] ufs: Input %lld is not valid.\n",
				__func__, __LINE__, value);

	return count;
}

static DEVICE_ATTR(io_stats, S_IRUGO | S_IWUSR,
		show_io_stats, set_io_stats);

struct cmd_sta_tmp {
	u_int64_t	size_cnt;						/* total access size */
	u_int64_t	io_cnt;							/* total access counter */
	ktime_t		lat_sum;
	ktime_t		lat_max;
};
#define		UFS_CMD_STRING		"ufs_cmd"
static ssize_t
show_io_stat_kmsg(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	unsigned long flags;
	int i, j, cnt;
	unsigned int pos;
	struct seq_file m;
	io_stats_cmd_t *stat_cmd;
	ktime_t	lat;
	unsigned long long first_time, last_time;
	struct cmd_sta_tmp sta_tmp[CMD_TYPE_MAX];
	ktime_t	a;

	if (!cmd_buf) {
		seq_printf(&m, "cmd_buf is NULL\n");
		return m.count;
	}

	a = ktime_get();
	memset(&m, 0, sizeof(struct seq_file));
	memset(&sta_tmp, 0, sizeof(sta_tmp));
	for (j = 0; j < ARRAY_SIZE(sta_tmp); j++) {
		sta_tmp[j].lat_sum =  ktime_set(0, 0); /* init as 0 seconds */
		sta_tmp[j].lat_max =  ktime_set(0, 0); /* init as 0 seconds */
	}
	m.buf = buf;
	m.size = PAGE_SIZE;

	if (!hba->io_stats.stat_cmd_enable) {
		seq_printf(&m, "io_stat_cmd is disabled\n");
		return m.count;
	}

	lat = ktime_set(0, 0); /* init as 0 seconds */
	stat_cmd = &hba->io_stats.stat_cmd;
	pos = stat_cmd->pos;
	/* pr_err("[%s] pos %u.\n", UFS_CMD_STRING, pos); */
	for (i = 0, cnt = 0; i < UFS_STAT_CMD_COUNT; i++) {
		spin_lock_irqsave(&hba->io_stats.lock, flags);
		cmd_buf[cnt] = stat_cmd->cmd[pos];
		spin_unlock_irqrestore(&hba->io_stats.lock, flags);

		if (ktime_to_us(cmd_buf[cnt].tstamp)) {
			pr_err("[%s] lun %d cmd %x T %s lba=0x%llx len=%d tstamp=%lld lat=%lld us\n",
					UFS_CMD_STRING,
					cmd_buf[cnt].lun,
					cmd_buf[cnt].op,
					ufs_support_cmd_type_str[cmd_buf[cnt].cmd_type],
					(unsigned long long)cmd_buf[cnt].lba,
					cmd_buf[cnt].size,
					ktime_to_us(cmd_buf[cnt].tstamp),
					ktime_to_us(cmd_buf[cnt].time));

			lat = ktime_add(lat, cmd_buf[cnt].time);

			for (j = 0; j < ARRAY_SIZE(sta_tmp); j++) {
				if (cmd_buf[cnt].cmd_type == j) {
					sta_tmp[j].size_cnt += cmd_buf[cnt].size;
					sta_tmp[j].io_cnt++;
					sta_tmp[j].lat_sum = ktime_add(sta_tmp[j].lat_sum, cmd_buf[cnt].time);
					if (ktime_compare(cmd_buf[cnt].time, sta_tmp[j].lat_max) > 0)
						sta_tmp[j].lat_max = cmd_buf[cnt].time;
				}
			}

			cnt++;
		}
		pos = (pos + 1) % UFS_STAT_CMD_COUNT;
	}

	first_time = ktime_to_us(cmd_buf[0].tstamp);
	last_time = ktime_to_us(cmd_buf[cnt - 1].tstamp);

	pr_err("[%s] %-20s: %llu\n", UFS_CMD_STRING, "First time stamp(ns)", first_time);
	pr_err("[%s] %-20s: %llu\n", UFS_CMD_STRING, "Last time stamp(ns)", last_time);
	pr_err("[%s] %-20s: %llu\n", UFS_CMD_STRING, "Average latency(us)", ktime_to_us(lat) / cnt);
	pr_err("[%s] %-20s: %d\n", UFS_CMD_STRING, "IO counter", cnt);

	pr_err("[%s] %1s %8s %8s %16s %16s\n",
			UFS_CMD_STRING, "T", "Size", "Counter", "Lat-Avg(us)", "Lat-max(us)");
	for (j = 0; j < ARRAY_SIZE(sta_tmp); j++) {
		pr_err("[%s] %1s %8llu %8llu %16llu %16llu\n",
				UFS_CMD_STRING,
				ufs_support_cmd_type_str[j],
				sta_tmp[j].size_cnt,
				sta_tmp[j].io_cnt,
				ktime_to_us(sta_tmp[j].lat_sum) / sta_tmp[j].io_cnt,
				ktime_to_us(sta_tmp[j].lat_max));
	}

	pr_err("[%s] take_time %llu us.\n",
			UFS_CMD_STRING,
			ktime_to_us(ktime_sub(ktime_get(), a)));

	seq_printf(&m, "show_io_stats_kmsg successfully.\n");
	return m.count;
}
static DEVICE_ATTR(io_stat_kmsg, S_IRUGO, show_io_stat_kmsg, NULL);

static ssize_t
show_io_sum(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	unsigned long flags;
	int i, j, k;
	struct seq_file m;
	io_stats_info_t *stat;
	u_int64_t size;

	memset(&m, 0, sizeof(struct seq_file));
	m.buf = buf;
	m.size = PAGE_SIZE;

	for (k = 0; k < CMD_FLUSH; k++)
		for (j = 0; j < UFS_STAT_SIZE_MAX; j++) {
			if ((k == CMD_DISCARD) && (j != UFS_STAT_SIZE_OTHER))
				continue;

			size = 0;
			for (i = 0; i < UFS_STAT_GEAR_MAX; i++) {
				spin_lock_irqsave(&hba->io_stats.lock, flags);
				stat = &hba->io_stats.stat[i][j][k];
				size += stat->size_cnt;
				spin_unlock_irqrestore(&hba->io_stats.lock, flags);
			}

			seq_printf(&m,
					"%s-%-4s: %16llu\n",
					ufs_support_cmd_type_str[k],
					ufs_support_size_str[j],
					size * 512
					);
		}

	return m.count;
}

static DEVICE_ATTR(io_sum, S_IRUGO | S_IWUSR,
		show_io_sum, set_io_stats);

static int cmd_cmp_func(const void *a, const void *b)
{
	io_ele_t *ia = &cmd_buf[*(u16 *)a];
	io_ele_t *ib = &cmd_buf[*(u16 *)b];

	return ktime_compare(ib->time, ia->time);
}

static void cmd_swap_func(void *a, void *b, int size)
{
	u16 t = *(u16 *)a;
	*(u16 *)a = *(u16 *)b;
	*(u16 *)b = t;
}

static ssize_t
show_io_stat_cmd(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	unsigned long flags;
	int i, j, id, cnt, max;
	unsigned int pos;
	struct seq_file m;
	io_stats_cmd_t *stat_cmd;
	ktime_t	lat;
	unsigned long long first_time, last_time;
	struct cmd_sta_tmp sta_tmp[CMD_TYPE_MAX];
	ktime_t	a;

	if (!cmd_buf || !idx_buf) {
		seq_printf(&m, "cmd_buf=%p idx_buf=%p\n", cmd_buf, idx_buf);
		return m.count;
	}

	a = ktime_get();
	memset(&m, 0, sizeof(struct seq_file));
	memset(&sta_tmp, 0, sizeof(sta_tmp));
	m.buf = buf;
	m.size = PAGE_SIZE;

	if (!hba->io_stats.stat_cmd_enable) {
		seq_printf(&m, "io_stat_cmd is disabled\n");
		return m.count;
	}

	lat = ktime_set(0, 0); /* init as 0 seconds */
	stat_cmd = &hba->io_stats.stat_cmd;
	pos = stat_cmd->pos;
	/* dev_err(hba->dev, "[%s:%d] pos %u.\n", __func__, __LINE__, pos); */
	for (i = 0, cnt = 0; i < UFS_STAT_CMD_COUNT; i++) {
		spin_lock_irqsave(&hba->io_stats.lock, flags);
		cmd_buf[cnt] = stat_cmd->cmd[pos];
		spin_unlock_irqrestore(&hba->io_stats.lock, flags);

		idx_buf[i] = i;
		if (ktime_to_us(cmd_buf[cnt].tstamp)) {
			/* pr_err("[%s] lun %d cmd %x T %s lba=0x%llx len=%d tstamp=%lld lat=%lld us\n", */
					/* "ufs_cmd", */
					/* cmd_buf[cnt].lun, */
					/* cmd_buf[cnt].op, */
					/* ufs_support_cmd_type_str[cmd_buf[cnt].cmd_type], */
					/* (unsigned long long)cmd_buf[cnt].lba, */
					/* cmd_buf[cnt].size, */
					/* ktime_to_us(cmd_buf[cnt].tstamp), */
					/* ktime_to_us(cmd_buf[cnt].time)); */

			lat = ktime_add(lat, cmd_buf[cnt].time);

			for (j = 0; j < ARRAY_SIZE(sta_tmp); j++) {
				if (cmd_buf[cnt].cmd_type == j) {
					sta_tmp[j].size_cnt += cmd_buf[cnt].size;
					sta_tmp[j].io_cnt++;
				}
			}

			cnt++;
		}
		pos = (pos + 1) % UFS_STAT_CMD_COUNT;
	}

	first_time = ktime_to_us(cmd_buf[0].tstamp);
	last_time = ktime_to_us(cmd_buf[cnt - 1].tstamp);
	sort(idx_buf, cnt, sizeof(idx_buf[0]), cmd_cmp_func, cmd_swap_func);

	seq_printf(&m, "%-20s: %llu\n", "First time stamp(ns)", first_time);
	seq_printf(&m, "%-20s: %llu\n", "Last time stamp(ns)", last_time);
	seq_printf(&m, "%-20s: %llu\n", "Average latency(us)", ktime_to_us(lat) / cnt);
	seq_printf(&m, "%-20s: %d\n", "IO counter", cnt);

	seq_printf(&m, "\n%1s %8s %8s\n", "T", "Size", "Counter");
	for (j = 0; j < ARRAY_SIZE(sta_tmp); j++) {
		seq_printf(&m, "%1s %8llu %8llu\n",
				ufs_support_cmd_type_str[j],
				sta_tmp[j].size_cnt,
				sta_tmp[j].io_cnt);
	}

	max = min(cnt, UFS_STAT_CMD_SYS_MAX_COUNT);
	seq_printf(&m,
			"\n%1s %1s %2s %8s %8s %16s %8s\n",
			"L",
			"T",
			"CM",
			"LBA",
			"Size",
			"Time Stamp",
			"Latency (us)");
	for (i = 0; i < max; i++) {
		id = idx_buf[i];
		seq_printf(&m,
				"%1d %1s %2x %8llx %8d %16llu %8llu\n",
				cmd_buf[id].lun,
				ufs_support_cmd_type_str[cmd_buf[id].cmd_type],
				cmd_buf[id].op,
				(unsigned long long)cmd_buf[id].lba,
				cmd_buf[id].size,
				ktime_to_us(cmd_buf[id].tstamp),
				ktime_to_us(cmd_buf[id].time));
	}
	dev_err(hba->dev, "[%s:%d] take_time %llu us.\n",
			__func__, __LINE__,
			ktime_to_us(ktime_sub(ktime_get(), a)));

	return m.count;
}

static DEVICE_ATTR(io_stat_cmd, S_IRUGO, show_io_stat_cmd, NULL);

static ssize_t
set_log_enable(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	int64_t value;
	unsigned long flags;

	if (kstrtoll(buf, 0, &value))
		return 0;

	spin_lock_irqsave(&hba->io_stats.lock, flags);

	hba->io_stats.log_enable = (unsigned int)value;

	spin_unlock_irqrestore(&hba->io_stats.lock, flags);

	return count;
}

static DEVICE_ATTR(log_enable, S_IWUSR,
		NULL, set_log_enable);

static ssize_t
set_stat_cmd_enable(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	int64_t value;
	unsigned long flags;

	if (kstrtoll(buf, 0, &value))
		return 0;

	spin_lock_irqsave(&hba->io_stats.lock, flags);

	hba->io_stats.stat_cmd_enable = (unsigned int)value;

	spin_unlock_irqrestore(&hba->io_stats.lock, flags);

	return count;
}

static DEVICE_ATTR(stat_cmd_enable, S_IWUSR,
		NULL, set_stat_cmd_enable);

static ssize_t
set_load_thre(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	int64_t value;
	unsigned long flags;

	if (kstrtoll(buf, 0, &value))
		return 0;

	spin_lock_irqsave(&hba->io_stats.lock, flags);

	hba->io_stats.load_thre = (unsigned int)value;

	spin_unlock_irqrestore(&hba->io_stats.lock, flags);

	return count;
}

static DEVICE_ATTR(load_thre, S_IWUSR,
		NULL, set_load_thre);

static ssize_t
show_load(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	unsigned long flags;
	unsigned int load;

	if (!ufshcd_is_clkscaling_supported(hba))
		return snprintf(buf, PAGE_SIZE, "clk_scaling is not enabled!\n");

	spin_lock_irqsave(&hba->io_stats.lock, flags);
	load = hba->io_stats.load;
	spin_unlock_irqrestore(&hba->io_stats.lock, flags);

	return snprintf(buf, PAGE_SIZE, "%u\n", load);
}

static DEVICE_ATTR(load, S_IRUGO, show_load, NULL);

static int ufs_vend_scsi_execute(struct scsi_device *sdev, const unsigned char *cmd,
		 int data_direction, void *buffer, unsigned bufflen,
		 int timeout, int retries, int *resid,
		 u64 flags)
{
	char *sense = NULL;
	struct request *req;
	int write = (data_direction == DMA_TO_DEVICE);
	int ret = DRIVER_ERROR << 24;
	struct scsi_sense_hdr sshdr;

	sense = kzalloc(SCSI_SENSE_BUFFERSIZE, GFP_NOIO);
	if (!sense)
		return DRIVER_ERROR << 24;

	req = blk_get_request(sdev->request_queue, write, __GFP_RECLAIM);
	if (IS_ERR(req))
		return ret;
	blk_rq_set_block_pc(req);

	if (bufflen &&	blk_rq_map_kern(sdev->request_queue, req,
					buffer, bufflen, __GFP_RECLAIM))
		goto out;

	req->cmd_len = 16; /* The cmd length is nonstandard. */
	memcpy(req->cmd, cmd, req->cmd_len);
	req->sense = sense;
	req->sense_len = 0;
	req->retries = retries;
	req->timeout = timeout;
	req->cmd_flags |= flags | REQ_QUIET | REQ_PREEMPT;

	/*
	 * head injection *required* here otherwise quiesce won't work
	 */
	blk_execute_rq(req->q, NULL, req, 1);

	/*
	 * Some devices (USB mass-storage in particular) may transfer
	 * garbage data together with a residue indicating that the data
	 * is invalid.  Prevent the garbage from being misinterpreted
	 * and prevent security leaks by zeroing out the excess data.
	 */
	if (unlikely(req->resid_len > 0 && req->resid_len <= bufflen))
		memset(buffer + (bufflen - req->resid_len), 0, req->resid_len);

	if (resid)
		*resid = req->resid_len;
	ret = req->errors;
 out:
	blk_put_request(req);

	if (ret) {
		pr_err("%s: cmd %#x opcode %#x failed with err %#x\n",
				__func__, cmd[0], cmd[1], ret);

		scsi_normalize_sense(sense, SCSI_SENSE_BUFFERSIZE, &sshdr);
		scsi_print_sense_hdr(sdev, "VendorCmd", &sshdr);

		print_hex_dump(KERN_ERR, "sshdr: ", DUMP_PREFIX_OFFSET,
				16, 1, &sshdr, sizeof(sshdr), false);
		/* print_hex_dump(KERN_ERR, "VendorCmd: ", DUMP_PREFIX_OFFSET, */
				/* 16, 1, cmd, req->cmd_len, false); */
	}

	kfree(sense);

	return ret;
}

static int ufs_vend_set_psw(struct ufs_hba *hba, struct scsi_device *sdp)
{
	unsigned char cmd[] = {UFS_VENDOR_COMMAND,
				UFS_VENDOR_OP_CODE_SET_PSW,
				GET_BYTE(UFS_VENDOR_PSW, 3),
				GET_BYTE(UFS_VENDOR_PSW, 2),
				GET_BYTE(UFS_VENDOR_PSW, 1),
				GET_BYTE(UFS_VENDOR_PSW, 0),
				0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	return ufs_vend_scsi_execute(sdp, cmd, DMA_NONE, NULL,
				0, msecs_to_jiffies(1000),
				UFS_VENDOR_MAX_RETRIES, NULL, 0);
}

static int ufs_vend_enter_mode_internal(struct ufs_hba *hba, struct scsi_device *sdp)
{
	unsigned char cmd[] = {UFS_VENDOR_COMMAND,
				UFS_VENDOR_OP_CODE_ENTER_MODE,
				GET_BYTE(UFS_VENDOR_SIGNATURE, 3),
				GET_BYTE(UFS_VENDOR_SIGNATURE, 2),
				GET_BYTE(UFS_VENDOR_SIGNATURE, 1),
				GET_BYTE(UFS_VENDOR_SIGNATURE, 0),
				GET_BYTE(UFS_VENDOR_PSW, 3),
				GET_BYTE(UFS_VENDOR_PSW, 2),
				GET_BYTE(UFS_VENDOR_PSW, 1),
				GET_BYTE(UFS_VENDOR_PSW, 0),
				0, 0, 0, 0, 0, 0};

	return ufs_vend_scsi_execute(sdp, cmd, DMA_NONE, NULL,
				0, msecs_to_jiffies(1000),
				UFS_VENDOR_MAX_RETRIES, NULL, 0);
}

static int ufs_vend_enter_mode(struct ufs_hba *hba, struct scsi_device *sdp)
{
	int ret;

	ret = ufs_vend_enter_mode_internal(hba, sdp);
	if (!ret)
		return ret;

	ret = ufs_vend_set_psw(hba, sdp);
	if (ret) {
		dev_err(hba->dev, "[%s:%d] ufs: Failed to set psw : %#x\n",
				__func__, __LINE__, ret);
	}

	return ufs_vend_enter_mode_internal(hba, sdp);
}

static int ufs_vend_exit_mode(struct ufs_hba *hba, struct scsi_device *sdp)
{
	unsigned char cmd[] = {UFS_VENDOR_COMMAND,
				UFS_VENDOR_OP_CODE_EXIT_MODE,
				0, 0, 0, 0, 0, 0, 0, 0,
				0, 0, 0, 0, 0, 0};

	return ufs_vend_scsi_execute(sdp, cmd, DMA_NONE, NULL,
				0, msecs_to_jiffies(1000),
				UFS_VENDOR_MAX_RETRIES, NULL, 0);
}

static int ufs_vend_nand_info_report(struct ufs_hba *hba,
		struct scsi_device *sdp, char *buffer, int buf_len)
{
	unsigned char cmd[] = {UFS_VENDOR_COMMAND,
				UFS_VENDOR_OP_CODE_NAND_INFO_REPORT,
				0, 0,
				0x01, /* Read descriptor */
				0x0A, /* Nand info report */
				0, 0, 0, 0, 0, 0,
				0, 0, 0, buf_len};

	return ufs_vend_scsi_execute(sdp, cmd, DMA_FROM_DEVICE, buffer,
				buf_len, msecs_to_jiffies(1000),
				UFS_VENDOR_MAX_RETRIES, NULL, 0);
}

static ssize_t
show_nand_info(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct ufs_hba *hba = dev_get_drvdata(dev);
	struct scsi_device *sdp;
	unsigned long flags;
	struct seq_file m;
	char *buffer;
	struct nand_info *nif;
	u8 desc_buf[QUERY_DESC_HEALTH_DEF_SIZE];
	int ret;

	memset(&m, 0, sizeof(struct seq_file));
	m.buf = buf;
	m.size = PAGE_SIZE;

	spin_lock_irqsave(hba->host->host_lock, flags);
	sdp = hba->sdev_ufs_lu0;
	if (sdp) {
		ret = scsi_device_get(sdp);
		if (!ret && !scsi_device_online(sdp)) {
			ret = -ENODEV;
			scsi_device_put(sdp);
		}
	} else {
		ret = -ENODEV;
	}
	spin_unlock_irqrestore(hba->host->host_lock, flags);

	if (ret) {
		seq_printf(&m, "Failed to get device %#x\n", ret);
		goto out;
	}

	ret = ufs_vend_enter_mode(hba, sdp);
	if (ret) {
		seq_printf(&m, "Failed to enter vendor mode : %#x\n", ret);
		goto dev_put;
	}

	buffer = kzalloc(UFS_VENDOR_NAND_INFO_SIZE, GFP_KERNEL);
	if (!buffer) {
		ret = -ENOMEM;
		goto exit_vend;
	}

	ret = ufs_vend_nand_info_report(hba, sdp, buffer,
			UFS_VENDOR_NAND_INFO_SIZE);
	if (ret) {
		seq_printf(&m, "Failed to read nand info : %#x\n", ret);
		goto free_buf;
	}

	nif = (struct nand_info *)buffer;
	nif->max_slc_erase_cycle = be32_to_cpu(nif->max_slc_erase_cycle);
	nif->min_slc_erase_cycle = be32_to_cpu(nif->min_slc_erase_cycle);
	nif->avg_slc_erase_cycle = be32_to_cpu(nif->avg_slc_erase_cycle);
	nif->max_mlc_erase_cycle = be32_to_cpu(nif->max_mlc_erase_cycle);
	nif->min_mlc_erase_cycle = be32_to_cpu(nif->min_mlc_erase_cycle);
	nif->avg_mlc_erase_cycle = be32_to_cpu(nif->avg_mlc_erase_cycle);
	nif->read_reclaim_cnt    = be32_to_cpu(nif->read_reclaim_cnt);
	nif->init_bad_blk        = be32_to_cpu(nif->init_bad_blk);
	nif->runtime_bad_blk     = be32_to_cpu(nif->runtime_bad_blk);
	nif->remain_reserved_blk = be32_to_cpu(nif->remain_reserved_blk);
	nif->write_data          = be32_to_cpu(nif->write_data);
	nif->initialize_cnt      = be32_to_cpu(nif->initialize_cnt);
	nif->ffu_success_cnt     = be32_to_cpu(nif->ffu_success_cnt);

	pm_runtime_get_sync(hba->dev);
	ret = ufshcd_read_health_desc(hba, desc_buf,
			QUERY_DESC_HEALTH_DEF_SIZE);
	pm_runtime_put_sync(hba->dev);
	if (ret) {
		pr_err("%s: Failed to read health descriptor with err %#x\n",
				__func__, ret);
		memset(desc_buf, 0, sizeof(desc_buf));
	}

	seq_printf(&m,
			"ufsid    : %s"
			"model    : %.16s\n"
			"fw_ver   : %4.4s\n"
			"life_a   : %d\n"
			"life_b   : %d\n"
			"life_eof : %d\n"
			"max_slc  : %d\n"
			"min_slc  : %d\n"
			"avg_slc  : %d\n"
			"max_mlc  : %d\n"
			"min_mlc  : %d\n"
			"avg_mlc  : %d\n"
			"rd_recl  : %d\n"
			"itbb     : %d\n"
			"rtbb     : %d\n"
			"rsv_blk  : %d\n"
			"w_size   : %d\n"
			"init_cnt : %d\n"
			"ffu_cnt  : %d\n"
			,
			hba->ufsid,
			sdp->model,
			sdp->rev,
			desc_buf[2],
			desc_buf[3],
			desc_buf[4],
			nif->max_slc_erase_cycle,
			nif->min_slc_erase_cycle,
			nif->avg_slc_erase_cycle,
			nif->max_mlc_erase_cycle,
			nif->min_mlc_erase_cycle,
			nif->avg_mlc_erase_cycle,
			nif->read_reclaim_cnt,
			nif->init_bad_blk,
			nif->runtime_bad_blk,
			nif->remain_reserved_blk,
			nif->write_data,
			nif->initialize_cnt,
			nif->ffu_success_cnt
			);
	dev_err(hba->dev, "Read ufs vend nand info ok\n");

free_buf:
	kfree(buffer);
exit_vend:
	ret = ufs_vend_exit_mode(hba, sdp);
	if (ret)
		dev_err(hba->dev,
				"[%s:%d] ufs: Failed to exit vendor mode : %#x\n",
				__func__, __LINE__, ret);
dev_put:
	scsi_device_put(sdp);
out:
	return m.count;
}

static DEVICE_ATTR(nand_info, S_IRUGO, show_nand_info, NULL);

static struct attribute *dev_attrs_io_stats[] = {
	&dev_attr_io_stats.attr,
	&dev_attr_io_sum.attr,
	&dev_attr_log_enable.attr,
	&dev_attr_load.attr,
	&dev_attr_load_thre.attr,
	&dev_attr_nand_info.attr,
	&dev_attr_io_stat_cmd.attr,
	&dev_attr_io_stat_kmsg.attr,
	&dev_attr_stat_cmd_enable.attr,
	NULL,
};

static struct attribute_group dev_attr_grp_io_stats = {
	.name = "ufs_io_stats",
	.attrs = dev_attrs_io_stats,
};

void ufs_io_stats_sysfs_create(struct ufs_hba *hba)
{
	int err;

	err = sysfs_create_group(&hba->dev->kobj, &dev_attr_grp_io_stats);
	if (err)
		pr_err("%s: failed to create sysfs group with err %#x\n",
				__func__, err);
}

void ufs_io_stats_sysfs_remove(struct ufs_hba *hba)
{
	sysfs_remove_group(&hba->dev->kobj, &dev_attr_grp_io_stats);
}

static void ufshcd_io_stats_init_intern(struct ufs_hba *hba)
{
	int i, j, k;
	io_stats_info_t *stat;
	unsigned long flags;

	spin_lock_irqsave(&hba->io_stats.lock, flags);

	memset(&hba->io_stats.stat, 0,
			(UFS_STAT_GEAR_MAX * UFS_STAT_SIZE_MAX
			 * CMD_TYPE_MAX * sizeof(io_stats_info_t)));

	for (i = 0; i < UFS_STAT_GEAR_MAX; i++)
		for (j = 0; j < UFS_STAT_SIZE_MAX; j++)
			for (k = 0; k < CMD_TYPE_MAX; k++) {
				stat = &hba->io_stats.stat[i][j][k];
				stat->lat_min = ktime_set(1000, 0); /* init as 1000 seconds */
				stat->lat_max = ktime_set(0, 0); /* init as 0 seconds */
			}

	spin_unlock_irqrestore(&hba->io_stats.lock, flags);
}


void ufshcd_io_stats_get_load(struct ufs_hba *hba,
		struct devfreq_dev_status *status)
{
	unsigned long flags;

	spin_lock_irqsave(&hba->io_stats.lock, flags);
	hba->io_stats.load = (status->busy_time * 100) / status->total_time;
	spin_unlock_irqrestore(&hba->io_stats.lock, flags);

	if ((hba->io_stats.load_thre
			&& (hba->io_stats.load > hba->io_stats.load_thre))
			&& __ratelimit(&hba->io_stats.ratelimit))
		dev_err(hba->dev, "[Warn] Load is high %u%% : total=%lu busy=%lu\n",
				hba->io_stats.load,
				status->total_time,
				status->busy_time);
}

void ufshcd_scsi_device0_get(struct ufs_hba *hba, struct scsi_device *sdev)
{
	if (sdev->lun == 0) {
		hba->sdev_ufs_lu0 = sdev;

		printk("%s:%d: gch set lun %lld sdev %p q %p n",
				__func__, __LINE__, sdev->lun, sdev, sdev->request_queue);
	}
}

void ufshcd_io_stats_init(struct ufs_hba *hba)
{
	spin_lock_init(&hba->io_stats.lock);
	ufs_io_stats_sysfs_create(hba);
	ufshcd_io_stats_init_intern(hba);

	hba->io_stats.stat_cmd_enable = true;
	/* hba->io_stats.log_enable = true; */
	hba->io_stats.load_thre = 0;

	cmd_buf = kmalloc(UFS_STAT_CMD_COUNT * sizeof(io_ele_t), GFP_KERNEL);
	if (!cmd_buf)
		dev_err(hba->dev, "Failed to alloc cmd_buf!\n");

	idx_buf = kmalloc(UFS_STAT_CMD_COUNT * sizeof(u16), GFP_KERNEL);
	if (!idx_buf)
		dev_err(hba->dev, "Failed to alloc idx_buf!\n");

	/* Not more than 5 times every 5 seconds. */
	ratelimit_state_init(&hba->io_stats.ratelimit,
			DEFAULT_RATELIMIT_INTERVAL, 5);
}

void ufshcd_io_stats_remove(struct ufs_hba *hba)
{
	ufs_io_stats_sysfs_remove(hba);

	kfree(cmd_buf);
	kfree(idx_buf);
}

#else

void ufshcd_scsi_device0_get(struct ufs_hba *hba, struct scsi_device *sdev)
{
}

void ufshcd_io_stats_init(struct ufs_hba *hba)
{
}

void ufshcd_io_stats_remove(struct ufs_hba *hba)
{
}

void ufshcd_io_stats(struct ufs_hba *hba, struct ufshcd_lrb *lrbp)
{
}

void ufshcd_io_stats_get_load(struct ufs_hba *hba,
		struct devfreq_dev_status *status)
{
}

#endif
