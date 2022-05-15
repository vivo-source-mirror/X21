#ifndef _UFS_VEN_H
#define _UFS_VEN_H

#include <linux/timer.h>
#include <linux/devfreq.h>
#include <linux/kfifo.h>

#ifdef CONFIG_UFS_IO_STATISTICS
typedef enum{
	CMD_READ			= 0,
	CMD_WRITE			= 1,
	CMD_DISCARD			= 2,
	CMD_FLUSH			= 3,
	CMD_TYPE_MAX		= 4,
} cmd_types;

#define		UFS_STAT_SIZE_MAX				3
#define		UFS_STAT_SIZE_OTHER				(UFS_STAT_SIZE_MAX - 1)
#define		UFS_STAT_GEAR_MAX				2
#define		UFS_STAT_LATENCY_MAX_TIME_MS	300
#define		UFS_STAT_CMD_COUNT				(4 << 10)
#define		UFS_STAT_CMD_SYS_MAX_COUNT		20

typedef struct {
	u_int64_t	io_cnt;							/* total access counter */
	u_int64_t	size_cnt;						/* total access size */
	ktime_t		lat;
	ktime_t		lat_max;
	ktime_t		lat_min;
} io_stats_info_t;

typedef struct __io_ele {
	int			  cmd_type;
	char		  op;
	char		  lun;
	sector_t	  lba;
	unsigned int  size;							/* access size */
	ktime_t       tstamp;						/* issue time */
	ktime_t       time;							/* latency */
} io_ele_t;

typedef struct {
	io_ele_t cmd[UFS_STAT_CMD_COUNT];
	unsigned int pos;
} io_stats_cmd_t;

struct io_stats {
	bool					log_enable;
	bool					stat_cmd_enable;
	spinlock_t				lock;
	unsigned int			load;
	unsigned int			load_thre;
	struct ratelimit_state	ratelimit;
	io_stats_info_t			stat[UFS_STAT_GEAR_MAX][UFS_STAT_SIZE_MAX][CMD_TYPE_MAX];
	io_stats_cmd_t			stat_cmd;
};

#define		GET_BYTE(x, y) (((x) >> (y*8)) & 0xff)

#define		UFS_VENDOR_COMMAND						0xC0
#define		UFS_VENDOR_OP_CODE_ENTER_MODE			0x00
#define		UFS_VENDOR_OP_CODE_EXIT_MODE			0x01
#define		UFS_VENDOR_OP_CODE_SET_PSW				0x03
#define		UFS_VENDOR_OP_CODE_NAND_INFO_REPORT		0x40

#define		UFS_VENDOR_PSW							0x5649564F
#define		UFS_VENDOR_SIGNATURE					0x5C3823AE
#define		UFS_VENDOR_MAX_RETRIES					1
#define		UFS_VENDOR_NAND_INFO_SIZE				0x4C

struct nand_info {
	uint32_t   length;
	uint8_t    desc_type;
	uint8_t    reserved_1[3];
	uint32_t   max_slc_erase_cycle;
	uint32_t   min_slc_erase_cycle;
	uint32_t   avg_slc_erase_cycle;
	uint32_t   max_mlc_erase_cycle;
	uint32_t   min_mlc_erase_cycle;
	uint32_t   avg_mlc_erase_cycle;
	uint32_t   read_reclaim_cnt;
	uint32_t   init_bad_blk;
	uint32_t   runtime_bad_blk;
	uint32_t   remain_reserved_blk;
	uint8_t    reserved_2[8];
	uint32_t   write_data;
	uint32_t   initialize_cnt;
	uint32_t   ffu_success_cnt;
	uint8_t    reserved_3[8];
} __attribute__((packed));

#endif

struct ufshcd_lrb;
struct ufs_hba;

void ufshcd_io_stats_init (struct ufs_hba *hba);
void ufshcd_io_stats_remove(struct ufs_hba *hba);
void ufshcd_io_stats(struct ufs_hba *hba, struct ufshcd_lrb *lrbp);
void ufshcd_io_stats_get_load (struct ufs_hba *hba,
		struct devfreq_dev_status *status);
void ufshcd_scsi_device0_get(struct ufs_hba *hba, struct scsi_device *sdev);

#endif
