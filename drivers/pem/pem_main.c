/*
 *  linux/drivers/pem/pem_main.c
 *
 *  Copyright 2017-2017 vivo pem daizhihui
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <pem.h>

#define CMD_BUF_SIZE 126
#define EVENT_BUF_SIZE 4094
#define PROCESS_BUF_ADD_FOR_ONE 4096
#define PROCESS_BUF_SIZE 8192
#define APROCESS_BUF_SIZE 32760
#define PROCESS_LIMIT_SIZE 200
#define EVENT_DISCARD_SIZE 3790
#define EVENT_MAX_SIZE 300
#define BUF_COUNT 2
#define PROCESS_NAME_LEN 100
#define THML_BUF_SIZE  4094

static const char *root_name = "pem";
static const char *event_name = "event";
static const int eventIdx;
static const char *cmd_name = "cmd";
static const int cmdIdx = 1;
static const char *process_name = "process";
static const char *aprocess_name = "aprocess";
static const char *pem_ioctl = "pem_ioctl";
static const char *thml_type_name = "thml_type";
static const char *thml_para_name = "thml_para";

static DECLARE_WAIT_QUEUE_HEAD(event_poll_wait);

struct pem_data {
	char *buf[BUF_COUNT];
	size_t size[BUF_COUNT];
	size_t eventPos;
	size_t lastEventPos;
	void *pem_task;
	u32 netUid[MAX_ARGS];
	spinlock_t eventLock;
	struct mutex cmdLock;
	struct proc_dir_entry *pem_root;
	struct pem_cmd cmd;
	struct pem_module *mdl;
};
static struct pem_data mdata = {
	.eventPos = 100000,
	.lastEventPos = 0,
	.mdl = NULL,
};

struct process_file_data {
	char *buf;
	size_t size;
	size_t limitSize;
	size_t remainSize;
	int uid;
};

struct thml_file_data {
    char  *para_size;
    char   type_size[16];
    struct mutex paraLock;
    struct mutex typeLock;
};

static struct thml_file_data thml_data = {
    .para_size = NULL,
};

void addPemEvent(char which, const char *fmt, ...)
{
	va_list args;
	int argSize = 0;

	spin_lock(&mdata.eventLock);
	if (mdata.eventPos < EVENT_DISCARD_SIZE) {
		mdata.buf[eventIdx][mdata.eventPos++] = which;
		if (fmt != NULL) {
			mdata.buf[eventIdx][mdata.eventPos++] = ',';
			va_start(args, fmt);
			argSize = vsnprintf(mdata.buf[eventIdx]+mdata.eventPos, 298, fmt, args);
			va_end(args);
			if (argSize > 0)
				mdata.eventPos += argSize;
		}
		mdata.buf[eventIdx][mdata.eventPos++] = ';';
		mdata.buf[eventIdx][mdata.eventPos] = '\0';
		pem_info(" pem_driver: addPemEvent which(%c), argSize(%d), eventPos(%ld)\n",
				which, argSize, mdata.eventPos);
		mdata.lastEventPos = mdata.eventPos;
	}
	spin_unlock(&mdata.eventLock);
	wake_up(&event_poll_wait);
}

inline void addPemEventNoArgs(char which)
{
	spin_lock(&mdata.eventLock);
	if (mdata.eventPos < EVENT_DISCARD_SIZE) {
		mdata.buf[eventIdx][mdata.eventPos++] = which;
		mdata.buf[eventIdx][mdata.eventPos++] = ';';
		mdata.buf[eventIdx][mdata.eventPos] = '\0';
		pem_info(" pem_driver: addPemEventNoArgs which(%c), eventPos(%ld)\n", which, mdata.eventPos);
		mdata.lastEventPos = mdata.eventPos;
	}
	spin_unlock(&mdata.eventLock);
	wake_up(&event_poll_wait);
}

void send_sk_info(u32 uid, u16 port, u32 addr)
{
	addPemEvent('n', "%d,%d,%8x", uid, port, addr);
}

ssize_t pem_event_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	int err;

	if (file->private_data == mdata.pem_task) {
		if (mdata.eventPos > 0) {
			spin_lock(&mdata.eventLock);
			err = copy_to_user(buf, mdata.buf[eventIdx], mdata.eventPos);
			pem_info(" pem_driver: pem_event_read size(%ld) rsize(%ld) err(%d)\n",
					size, mdata.eventPos, err);
			if (err) {
				spin_unlock(&mdata.eventLock);
				return -EFAULT;
			}
			size = mdata.eventPos;
			mdata.eventPos = 0;
			spin_unlock(&mdata.eventLock);
			return size;
		}
		return 0;
	}

	err = mdata.lastEventPos - *ppos;
	if (err <= 0)
		return 0;
	if (size > err)
		size = err;
	if (size <= 0 || size > mdata.size[eventIdx])
		return 0;
	err = copy_to_user(buf, mdata.buf[eventIdx] + *ppos, size);
	pem_info(" pem_driver: pem_event_read size(%ld) lastEventPos(%ld) err(%d)\n", size, mdata.lastEventPos, err);
	if (!err) {
		*ppos += size;
		return size;
	}
	return -EFAULT;
}

static int pem_event_open(struct inode *inode, struct file *file)
{
	file->private_data = current;
	pem_info(" pem_driver: pem_proc_open event pid(%d)\n", ((struct task_struct *)file->private_data)->pid);
	return 0;
}

static int pem_event_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	pem_info(" pem_driver: pem_event_release\n");
	return 0;
}

static inline bool checkBuffer(size_t count, int idx)
{
	if (count < 1)
		return true;
	if (mdata.size[idx] < count) {
		if (mdata.buf[idx] != NULL)
			kfree(mdata.buf[idx]);
		mdata.buf[idx] = kmalloc(count+2, GFP_KERNEL);
		if (mdata.buf[idx] == NULL)
			return true;
		mdata.size[idx] = count;
		mdata.buf[idx][count+1] = '\0';
	}
	return false;
}

static unsigned int pem_poll(struct file *file, poll_table *wait)
{
	poll_wait(file, &event_poll_wait, wait);
	if (mdata.eventPos > 0)
		return POLLIN | POLLRDNORM;
	return 0;
}


static ssize_t pem_event_write(struct file *fp, const char __user *user_buff, size_t count, loff_t *pos)
{
	if (mdata.eventPos >= EVENT_DISCARD_SIZE || count <= 0)
		return -ENOMEM;
	spin_lock(&mdata.eventLock);
	if (mdata.eventPos + count > mdata.size[eventIdx]) {
		spin_unlock(&mdata.eventLock);
		return -ENOMEM;
	}
	if (copy_from_user(mdata.buf[eventIdx]+mdata.eventPos, user_buff, count)) {
		pem_warn(" pem_driver: event copy_from_user fail\n");
		spin_unlock(&mdata.eventLock);
		return -EINVAL;
	}
	mdata.eventPos += count;
	if (count > 1) {
		if (mdata.buf[eventIdx][mdata.eventPos-1] == '\n') {
			if (mdata.buf[eventIdx][mdata.eventPos-2] == ';') {
				mdata.buf[eventIdx][mdata.eventPos-1] = '\0';
				mdata.eventPos--;
			} else {
				mdata.buf[eventIdx][mdata.eventPos-1] = ';';
				mdata.buf[eventIdx][mdata.eventPos] = '\0';
			}
		} else {
			if (mdata.buf[eventIdx][mdata.eventPos-1] == ';') {
				mdata.buf[eventIdx][mdata.eventPos] = '\0';
			} else {
				mdata.buf[eventIdx][mdata.eventPos++] = ';';
				mdata.buf[eventIdx][mdata.eventPos] = '\0';
			}
		}
	} else {
		mdata.buf[eventIdx][mdata.eventPos] = '\0';
	}
	mdata.lastEventPos = mdata.eventPos;
	spin_unlock(&mdata.eventLock);

	pem_info(" pem_driver: pem_event_write count(%ld) buf(%s)\n", count, mdata.buf[eventIdx]);
	wake_up(&event_poll_wait);

	return count;
}

static int pem_cmd_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", mdata.buf[cmdIdx]);
	return 0;
}

static int pem_cmd_open(struct inode *inode, struct file *file)
{
	pem_info(" pem_driver: pem_proc_open cmd\n");
	return single_open(file, pem_cmd_show, (void *)&mdata);
}

inline void setNetUid(const struct pem_cmd *cmd)
{
	int i, ret;

	for (i = 0; i < cmd->argCnt; i++) {
		ret = kstrtou32(cmd->args[i], 10, &mdata.netUid[i]);
		if (ret) {
			pem_warn(" pem_driver: to parser %s error\n", cmd->args[i]);
			mdata.netUid[i] = 0;
		}
		pem_warn(" pem_driver: find netUid %u\n", mdata.netUid[i]);
	}
#if IS_ENABLED(CONFIG_CGROUP_NET_PRIO)
	set_skb_fg_uid(mdata.netUid, cmd->argCnt);
	pem_warn(" pem_driver: find netUid cnt = %d\n", cmd->argCnt);
#endif
}

static inline void execMainCmd(struct pem_cmd *cmd)
{
	if (cmd->what == 's') {
		mdata.pem_task = current;
		pem_info(" pem_driver: init pem event read\n");
	} else if (cmd->what == 'm') {
		if (cmd->argCnt < 1)
			return;
		setNetUid(cmd);
	}
}

static inline void execCmd(struct pem_cmd *cmd) 
{
	struct pem_module *tmpMdl = mdata.mdl;

	while (tmpMdl) {
		if (cmd->fun == tmpMdl->fun) {
			tmpMdl->execCmd(cmd);
			return;
		}
		tmpMdl = tmpMdl->next;
	}

	if (cmd->fun == '#')
		execMainCmd(cmd);
	else
		pem_warn(" pem_driver: can not suppot cmd fun(%c) what(%c)\n", cmd->fun, cmd->what);
}

static inline int getCmd(char *cmds, size_t count, int cur)
{
	for (; cur < count; cur++) {
		if (cmds[cur] > 'A') {
			mdata.cmd.fun = '0';
			mdata.cmd.what = cmds[cur];
			return cur;
		}
		if (cmds[cur] == ';' || cmds[cur] == '\n')
			continue;
		if (cmds[cur] == ',') {
			for (; cur < count; cur++) {
				if (cmds[cur] == ';' || cmds[cur] == '\n')
					break;
			}
			continue;
		}
		mdata.cmd.fun = cmds[cur];
		cur++;
		if (cur == count || cmds[cur] == ';' || cmds[cur] == '\n')
			continue;
		if (cmds[cur] == ',') {
			for (; cur < count; cur++) {
				if (cmds[cur] == ';' || cmds[cur] == '\n')
					break;
			}
			continue;
		}
		mdata.cmd.what = cmds[cur];
		return cur;
	}
	return cur+100;
}

static inline void parserCmd(char *cmds, size_t count) 
{
	int i = getCmd(cmds, count, 0) + 1;

	if (i > count)
		return;
	for (mdata.cmd.argCnt = 0; i <= count; i++) {
		if (cmds[i] == ';' || cmds[i] == '\n' || i == count) {
			if (mdata.cmd.argCnt > 0 && mdata.cmd.args[mdata.cmd.argCnt-1] == (cmds + i))
				mdata.cmd.argCnt--;
			cmds[i] = 0;
			execCmd(&mdata.cmd);
			if (++i < count)
				i = getCmd(cmds, count, i);
			mdata.cmd.argCnt = 0;
		} else if (cmds[i] == ',') {
			if (mdata.cmd.argCnt > 0 && mdata.cmd.args[mdata.cmd.argCnt-1] == (cmds + i))
				mdata.cmd.argCnt--;
			if (mdata.cmd.argCnt < MAX_ARGS) {
				mdata.cmd.args[mdata.cmd.argCnt] = cmds + i + 1;
				mdata.cmd.argCnt++;
			}
			cmds[i] = 0;
		}
	}
}

static ssize_t pem_cmd_store(struct file *fp, const char __user *user_buff, size_t count, loff_t *pos)
{
	if (checkBuffer(count, cmdIdx))
		return -ENOMEM;

	mutex_lock(&mdata.cmdLock);
	if (copy_from_user(mdata.buf[cmdIdx], user_buff, count)) {
		pem_warn(" pem_driver: cmd copy_from_user fail\n");
		mutex_unlock(&mdata.cmdLock);
		return -EINVAL;
	}
	parserCmd(mdata.buf[cmdIdx], count);
	mutex_unlock(&mdata.cmdLock);

	return count;
}

static int pem_process_open(struct inode *inode, struct file *file)
{
	const struct cred *cred;
	struct process_file_data *process_data = kmalloc(sizeof(struct process_file_data), GFP_KERNEL);

	if (process_data != NULL) {
		process_data->buf = kmalloc(PROCESS_BUF_SIZE, GFP_KERNEL);
		if (process_data->buf != NULL) {
			file->private_data = process_data;
			process_data->remainSize = 0;
			cred = get_task_cred(current);
			process_data->uid = cred->uid.val;
			process_data->size = PROCESS_BUF_SIZE;
			process_data->limitSize = process_data->size - PROCESS_LIMIT_SIZE;
			pem_info(" pem_driver: pem_process_open uid(%d)\n", process_data->uid);
			return 0;
		}
		kfree(process_data);
	}
	pem_info(" pem_driver: pem_process_open fail, because kmalloc mem\n");
	file->private_data = NULL;
	return -ENOMEM;
}

static int pem_process_release(struct inode *inode, struct file *file)
{
	if (file->private_data != NULL) {
		if (((struct process_file_data *)file->private_data)->buf != NULL)
			kfree(((struct process_file_data *)file->private_data)->buf);
		kfree(file->private_data);
	}
	pem_info(" pem_driver: pem_process_release\n");
	return 0;
}

static int pem_aprocess_open(struct inode *inode, struct file *file)
{
	const struct cred *cred;
	struct process_file_data *process_data = kmalloc(sizeof(struct process_file_data), GFP_KERNEL);

	if (process_data != NULL) {
		process_data->buf = kmalloc(APROCESS_BUF_SIZE, GFP_KERNEL);
		if (process_data->buf != NULL) {
			file->private_data = process_data;
			process_data->remainSize = 0;
			cred = get_task_cred(current);
			process_data->uid = cred->uid.val;
			process_data->size = APROCESS_BUF_SIZE;
			process_data->limitSize = process_data->size - PROCESS_LIMIT_SIZE;
			pem_info(" pem_driver: pem_aprocess_open uid(%d)\n", process_data->uid);
			return 0;
		}
		kfree(process_data);
	}
	pem_info(" pem_driver: pem_aprocess_open fail, because kmalloc mem\n");
	file->private_data = NULL;
	return -ENOMEM;

}

static int pem_aprocess_release(struct inode *inode, struct file *file)
{
	pem_info(" pem_driver: pem_aprocess_release\n");
	return pem_process_release(inode, file);
}

static int readProcess(struct process_file_data *process_data)
{
	int spCnt;
	unsigned long flags;
	cputime_t utime, stime;
	struct task_struct *task;

	if (process_data == NULL)
		return -EINVAL;
	process_data->remainSize = 0;
	read_lock(&tasklist_lock);
	for_each_process(task) {
		if (lock_task_sighand(task, &flags)) {
			thread_group_cputime_adjusted(task, &utime, &stime);
			unlock_task_sighand(task, &flags);
			utime += stime;
			spCnt = snprintf(process_data->buf + process_data->remainSize,
					PROCESS_LIMIT_SIZE, "%d,%ld\n", task->pid, utime);
			if (spCnt > 0)
				process_data->remainSize += spCnt;
			if (process_data->remainSize > process_data->limitSize) {
				process_data->remainSize = 0;
				break;
			}
		} else {
			pem_info(" pem_driver: pid(%d) can not lock_task_sighand\n", task->pid);
		}
	}
	read_unlock(&tasklist_lock);

	if (process_data->remainSize == 0) {
		kfree(process_data->buf);
		process_data->buf = kmalloc(PROCESS_BUF_ADD_FOR_ONE+process_data->size, GFP_KERNEL);
		if (process_data->buf != NULL) {
			process_data->size += PROCESS_BUF_ADD_FOR_ONE;
			process_data->limitSize = process_data->size - PROCESS_LIMIT_SIZE;
			pem_info(" pem_driver: readProcess to realloc mem, size(%ld)\n", process_data->size);
			return readProcess(process_data);
		}
		return -ENOMEM;
	}

	return 0;
}

ssize_t pem_process_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	int err;
	size_t cpSize;
	struct process_file_data *process_data = (struct process_file_data *)file->private_data;

	if (process_data->remainSize <= 0) {
		if (*ppos > 0) {
			*ppos = 0;
			return 0;
		}
		err = readProcess(process_data);
		if (err) {
			pem_info(" pem_driver: pem_process_read error(%d)\n", err);
			return err;
		}
	}

	if (size > process_data->remainSize)
		cpSize = process_data->remainSize;
	else
		cpSize = size;
	err = copy_to_user(buf, process_data->buf + *ppos, cpSize);
	if (!err) {
		process_data->remainSize -= cpSize;
		if (cpSize < size && process_data->uid == 1000)
			*ppos = 0;
		else
			*ppos += cpSize;
		return cpSize;
	}

	return -EFAULT;
}

static inline int dealProcessName(char *name, int spCnt)
{
	int i = 0;
	int lastFind = 0;

	while (i < spCnt) {
		if (name[i] == '/')
			lastFind = i;
		i++;
	}
	spCnt = spCnt - lastFind - 1;
	memcpy(name, name + lastFind + 1, spCnt);
	return spCnt;
}

static int readAProcess(struct process_file_data *process_data)
{
	int spCnt;
	const struct cred *cred;
	unsigned long flags;
	cputime_t utime, stime;
	struct task_struct *task;
	char *processName;
	char name[PROCESS_NAME_LEN];

	if (process_data == NULL)
		return -EINVAL;
	spCnt = 0;
	process_data->remainSize = 0;
	rcu_read_lock();
	for_each_process(task) {
		if (lock_task_sighand(task, &flags)) {
			thread_group_cputime_adjusted(task, &utime, &stime);
			unlock_task_sighand(task, &flags);
			utime += stime;
			cred = get_task_cred(task);
			processName = task->comm;
			if (!(task->flags & PF_KTHREAD)) {
				get_task_struct(task);
				rcu_read_unlock();
				spCnt = get_cmdline(task, name, PROCESS_NAME_LEN);
				rcu_read_lock();
				put_task_struct(task);
				if (spCnt > 0) {
					if (name[0] == '/')
						spCnt = dealProcessName(name, spCnt);
					if (spCnt < PROCESS_NAME_LEN)
						name[spCnt] = 0;
					else
						name[PROCESS_NAME_LEN-1] = 0;
					processName = name;
				}
			}

			spCnt = snprintf(process_data->buf + process_data->remainSize,
				PROCESS_LIMIT_SIZE, "%d,%ld,%d,%s\n", task->pid, utime, cred->uid.val, processName);

			if (spCnt > 0)
				process_data->remainSize += spCnt;
			if (process_data->remainSize > process_data->limitSize) {
				process_data->remainSize = 0;
				break;
			}
		} else {
			pem_info(" pem_driver: pid(%d) can not lock_task_sighand\n", task->pid);
		}
	}
	rcu_read_unlock();

	if (process_data->remainSize == 0) {
		kfree(process_data->buf);
		process_data->buf = kmalloc(PROCESS_BUF_ADD_FOR_ONE+process_data->size, GFP_KERNEL);
		if (process_data->buf != NULL) {
			process_data->size += PROCESS_BUF_ADD_FOR_ONE;
			process_data->limitSize = process_data->size - PROCESS_LIMIT_SIZE;
			pem_info(" pem_driver: readAProcess to realloc mem, size(%ld)\n", process_data->size);
			return readAProcess(process_data);
		}
		return -ENOMEM;
	}

	return 0;
}


ssize_t pem_aprocess_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	int err;
	size_t cpSize;
	struct process_file_data *process_data = (struct process_file_data *)file->private_data;

	if (process_data->remainSize <= 0) {
		if (*ppos > 0) {
			*ppos = 0;
			return 0;
		}
		err = readAProcess(process_data);
		if (err) {
			pem_info(" pem_driver: pem_aprocess_read error(%d)\n", err);
			return err;
		}
	}

	if (size > process_data->remainSize)
		cpSize = process_data->remainSize;
	else
		cpSize = size;
	err = copy_to_user(buf, process_data->buf + *ppos, cpSize);
	if (!err) {
		process_data->remainSize -= cpSize;
		if (cpSize < size && process_data->uid == 1000)
			*ppos = 0;
		else
			*ppos += cpSize;
		return cpSize;
	}

	return -EFAULT;
}

static int pem_thml_para_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", thml_data.para_size);
	return 0;
}

static int pem_thml_para_open(struct inode *inode, struct file *file)
{
	pem_info(" pem_driver:pem_thml_para_open\n");
	return single_open(file, pem_thml_para_show, (void *)&thml_data);
}
#ifdef CONFIG_MTK_PLATFORM
extern int check_para_from_pem(char *buf, int len);
#endif

static ssize_t pem_thml_para_store(struct file *fp, const char __user *user_buff, size_t count, loff_t *pos)
{
	memset(thml_data.para_size, 0, THML_BUF_SIZE);
	pem_info(" pem_driver: pem_thml_para_store data len: %ld\n", count);
	mutex_lock(&thml_data.paraLock);
	if (copy_from_user(thml_data.para_size, user_buff, count)) {
		pem_warn(" pem_driver: thermal para copy_from_user fail\n");
		mutex_unlock(&thml_data.paraLock);
		return -EINVAL;
	}
	mutex_unlock(&thml_data.paraLock);
	pem_info(" pem_driver: pem_thml_para_store type: %s\n", thml_data.para_size);
#ifdef CONFIG_MTK_PLATFORM
	/*check_para_from_pem(thml_data.para_size, count);*/
#endif
	return count;
}

static int pem_thml_para_release(struct inode *inode, struct file *file)
{
	pem_info(" pem_driver: pem_thml_para_release\n");
	return 0;
}


static int pem_thml_type_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", thml_data.type_size);
	return 0;
}

static int pem_thml_type_open(struct inode *inode, struct file *file)
{
	pem_info(" pem_driver: pem_thml_type_open type: %s\n", thml_data.type_size);
	return single_open(file, pem_thml_type_show, (void *)&thml_data);
}
#ifdef CONFIG_MTK_PLATFORM
extern int vivo_thermal_parameter_config(char *buf);
#endif

static ssize_t pem_thml_type_store(struct file *fp, const char __user *user_buff, size_t count, loff_t *pos)
{
	memset(thml_data.type_size, 0, sizeof(thml_data.type_size)/sizeof(char));

	mutex_lock(&thml_data.typeLock);
	if (copy_from_user(thml_data.type_size, user_buff, count)) {
		pem_warn(" pem_driver: thermal type copy_from_user fail\n");
		mutex_unlock(&thml_data.typeLock);
		return -EINVAL;
	}
	mutex_unlock(&thml_data.typeLock);
	pem_info(" pem_driver: pem_thml_type_store type: %s\n", thml_data.type_size);
#ifdef CONFIG_MTK_PLATFORM
	vivo_thermal_parameter_config(thml_data.type_size);
#endif
	return count;
}


static const struct file_operations pem_event_fops = {
	.owner = THIS_MODULE,
	.open = pem_event_open,
	.read = pem_event_read,
	.write = pem_event_write,
	.release = pem_event_release,
	.poll = pem_poll,
};

static const struct file_operations pem_cmd_fops = {
	.owner = THIS_MODULE,
	.open = pem_cmd_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.write = pem_cmd_store,
	.release = single_release,
};

static const struct file_operations pem_process_fops = {
	.owner = THIS_MODULE,
	.open = pem_process_open,
	.read = pem_process_read,
	.release = pem_process_release,
};

static const struct file_operations pem_aprocess_fops = {
	.owner = THIS_MODULE,
	.open = pem_aprocess_open,
	.read = pem_aprocess_read,
	.release = pem_aprocess_release,
};

static const struct file_operations pem_thml_type_fops = {
	.owner = THIS_MODULE,
	.open = pem_thml_type_open,
	.read = seq_read,
	.write = pem_thml_type_store,
	.release = single_release,
};

/* hechenhui add for pem_ioctl begin */
/*--------------------DEV OP------------------------*/
static int device_show(struct seq_file *m, void *v)
{
    pem_info(" pem_driver, device_show \n");
	return 0;
}

static int device_open(struct inode *inode, struct file *file)
{
    pem_info(" pem_driver, device_open \n");
	return single_open(file, device_show, inode->i_private);
}

static long device_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	ssize_t ret = 0;
	int pid;
	unsigned long flags;
	struct task_struct *task;
	cputime_t utime, stime;
	if (get_user(pid, (int __user *)arg))
		return -EFAULT;
	task = pid_task(find_vpid(pid), PIDTYPE_PID);
	if (task != NULL) {
		if (lock_task_sighand(task, &flags)) {
			thread_group_cputime_adjusted(task, &utime, &stime);
			unlock_task_sighand(task, &flags);
			utime += stime;
			return utime;
		} else {
			pem_info(" pem_driver: pid(%d) can not lock_task_sighand\n", task->pid);
		}
	}
	return ret;
}

static const struct file_operations pem_ioctl_fops = {
	.unlocked_ioctl = device_ioctl,
	.compat_ioctl = device_ioctl,
	.open = device_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
/* hechenhui add for pem_ioctl end */

static const struct file_operations pem_thml_para_fops = {
	.owner = THIS_MODULE,
	.open = pem_thml_para_open,
	.read = seq_read,
	.write = pem_thml_para_store,
	.release = pem_thml_para_release,
};


static inline int node_init(struct proc_dir_entry *root)
{
	umode_t cMode = 0666;
	struct proc_dir_entry *pem_entry = proc_create_data(event_name,
			cMode, root, &pem_event_fops, (void *)&mdata);

	if (pem_entry == NULL) {
		pem_warn(" pem_driver: Create file /proc/pem/%s error!\n", event_name); 
		return -ENOMEM;
	}
	pem_info(" pem_driver: Create dir /proc/pem/%s!\n", event_name);

	pem_entry = proc_create_data(cmd_name,
			cMode, root, &pem_cmd_fops, (void *)&mdata);
	if (pem_entry == NULL) {
		pem_warn(" pem_driver: Create file /proc/pem/%s error!\n", cmd_name); 
		remove_proc_entry(event_name, root);
		return -ENOMEM;
	}
	pem_info(" pem_driver: Create dir /proc/pem/%s!\n", cmd_name);

	pem_entry = proc_create_data(process_name,
			0444, root, &pem_process_fops, (void *)&mdata);
	if (pem_entry == NULL) {
		pem_warn(" pem_driver: Create file /proc/pem/%s error!\n", process_name); 
		remove_proc_entry(event_name, root);
		remove_proc_entry(cmd_name, root);
		return -ENOMEM;
	}
	pem_info(" pem_driver: Create dir /proc/pem/%s!\n", process_name);

	pem_entry = proc_create_data(aprocess_name,
			0444, root, &pem_aprocess_fops, (void *)&mdata);
	if (pem_entry == NULL) {
		pem_warn(" pem_driver: Create file /proc/pem/%s error!\n", aprocess_name);
		remove_proc_entry(event_name, root);
		remove_proc_entry(cmd_name, root);
		remove_proc_entry(process_name, root);
		return -ENOMEM;
	}
	pem_info(" pem_driver: Create dir /proc/pem/%s!\n", aprocess_name);

	/* hechenhui add begin */
	pem_entry = proc_create_data(pem_ioctl,
			cMode, root, &pem_ioctl_fops, (void *)&mdata);
	if (pem_entry == NULL) {
		remove_proc_entry(event_name, root);
		remove_proc_entry(cmd_name, root);
		remove_proc_entry(process_name, root);
		remove_proc_entry(aprocess_name, root);
		return -ENOMEM;
	}
	pem_info(" pem_driver: Create dir /proc/pem/%s!\n", pem_ioctl);
	/* hechenhui add end */

	pem_entry = proc_create_data(thml_para_name,
			cMode, root, &pem_thml_para_fops, (void *)&thml_data);
	if (pem_entry == NULL) {
		pem_warn(" pem_driver: Create file /proc/pem/%s error!\n", thml_para_name); 
		return -ENOMEM;
	}
	if (thml_data.para_size == NULL) {
		thml_data.para_size = (char *)kzalloc(THML_BUF_SIZE * sizeof(char), GFP_KERNEL);
	}
	memset(thml_data.para_size, 0, THML_BUF_SIZE);
	pem_info(" pem_driver: Create dir /proc/pem/%s!\n", thml_para_name);

    pem_entry = proc_create_data(thml_type_name,
			cMode, root, &pem_thml_type_fops, (void *)&thml_data);
	if (pem_entry == NULL) {
		pem_warn(" pem_driver: Create file /proc/pem/%s error!\n", thml_type_name); 
		return -ENOMEM;
	}
	memset(thml_data.type_size, 0, sizeof(thml_data.type_size)/sizeof(char));
	pem_info(" pem_driver: Create dir /proc/pem/%s!\n", thml_type_name);
    

	checkBuffer(EVENT_BUF_SIZE, eventIdx);
	if (mdata.size[eventIdx] > EVENT_DISCARD_SIZE)
		mdata.eventPos = 0;
	checkBuffer(CMD_BUF_SIZE, cmdIdx);
	mutex_init(&mdata.cmdLock);
	spin_lock_init(&mdata.eventLock);
    mutex_init(&thml_data.paraLock);
    mutex_init(&thml_data.typeLock);
	return 0;
}

static inline void _module_attach(struct pem_module *mdl)
{
	struct pem_module *tmpMdl;

	if (mdl == NULL) {
		pem_warn(" pem_driver: _module_attach fail,because mdl is null\n");
		return;
	}
	if (mdl->execCmd == NULL || mdl->fun == 0) {
		pem_warn(" pem_driver: _module_attach fail,because execCmd is null or fun is 0\n");
		return;
	}
	if (mdata.mdl == NULL) {
		mdata.mdl = mdl;
		pem_warn(" pem_driver: _module_attach success, fun is %d(%c)\n", (int)mdl->fun, mdl->fun);
		return;
	}
	tmpMdl = mdata.mdl;
	while (tmpMdl) {
		if (tmpMdl->fun == mdl->fun) {
			pem_warn(" pem_driver: _module_attach fail, because has existed fun %d(%c)\n",
					(int)mdl->fun, mdl->fun);
			return;
		}
		if (tmpMdl->next == NULL) {
			tmpMdl->next = mdl;
			pem_info(" pem_driver: _module_attach success, fun is %d(%c)\n", (int)mdl->fun, mdl->fun);
			return;
		}
		tmpMdl = tmpMdl->next;
	}
}


DECLARATION_ATTACH(policy);
DECLARATION_ATTACH(cpulimit);
static inline void pem_module_attach(void)
{
	_module_attach(PEM_ATTACH(policy, mdata.pem_root));
	_module_attach(PEM_ATTACH(cpulimit, mdata.pem_root));
}

static int __init pem_init(void)
{
	mdata.pem_root = proc_mkdir(root_name, NULL);
	if (mdata.pem_root == NULL) { 
		pem_warn(" pem_driver: Create dir /proc/%s error!\n", root_name);
		return -ENODEV; 
	}
	pem_info(" pem_driver: Create dir /proc/%s\n", root_name);

#if IS_ENABLED(CONFIG_CGROUP_NET_PRIO)
	skb_prioidx_spin_lock_init();
#endif

	node_init(mdata.pem_root);
	pem_module_attach();

	return 0;
}

static void __exit pem_exit(void)
{
	struct pem_module *tmpMdl;

	while (mdata.mdl) {
		tmpMdl = mdata.mdl;
		mdata.mdl = mdata.mdl->next;
		if (tmpMdl->remove)
			tmpMdl->remove(mdata.pem_root);
		kfree(tmpMdl);
	}
	if (thml_data.para_size != NULL) {
		kfree(thml_data.para_size);
		thml_data.para_size = NULL;
	}

	remove_proc_entry(event_name, mdata.pem_root);
	remove_proc_entry(cmd_name, mdata.pem_root);
	remove_proc_entry(process_name, mdata.pem_root);
	remove_proc_entry(pem_ioctl, mdata.pem_root);
	remove_proc_entry(root_name, NULL);

	if (mdata.buf[eventIdx] != NULL)
		kfree(mdata.buf[eventIdx]);
	if (mdata.buf[cmdIdx] != NULL)
		kfree(mdata.buf[cmdIdx]);
}

module_init(pem_init);
module_exit(pem_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Driver For Pem");
MODULE_AUTHOR("vivo pem daizhihui");
