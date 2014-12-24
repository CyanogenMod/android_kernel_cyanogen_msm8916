

#include <linux/kernel.h>
#include <linux/export.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/notifier.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/scatterlist.h>

#include <linux/genhd.h>
#include <linux/reboot.h>
#include <linux/yl_params.h>
#include <linux/mutex.h>

/* Most platform except for NVIDIA use this defination. */
#define YL_PARAMS_NAME	"params"
#define YL_MISC_NAME	"misc"
#undef  pr_fmt
#define pr_fmt(fmt)	"YLLOG: YL-params: " fmt

const char *yl_params_map[] = {
	[YL_DEVICE]		= "DEVICE",
	[YL_CONFIGURATION]	= "CONFIGURATION",
	[YL_PRODUCTLINE]	= "PRODUCTLINE",
	[YL_DYNAMIC]		= "DYNAMIC",
	[YL_GUARD]		= "GUARD",
	[YL_CMDLINE]		= "CMDLINE",
	[YL_TOUCHSCREEN0]	= "TOUCHSCREEN0",
	[YL_TOUCHSCREEN1]	= "TOUCHSCREEN1",
	[YL_TOUCHSCREEN2]	= "TOUCHSCREEN2",
	[YL_TOUCHSCREEN3]	= "TOUCHSCREEN3",
	[YL_RESERVE0]		= "RESERVE0",
	[YL_RESERVE1]		= "RESERVE1",
	[YL_PROJECT0]		= "PROJECT0",
	[YL_PROJECT1]		= "PROJECT1",
	[YL_PROJECT2]		= "PROJECT2",
	[YL_PROJECT3]		= "PROJECT3",
	[YL_FETCH_PASSWD]	= "FETCH_PASSWD",
	[YL_FCT_DIAG]		= "FCT_DIAG",
	[YL_RCP]		= "RCP",
	[YL_RETURNZERO]		= "RETURNZERO",
};

enum preload_state {
	PRELOAD_EMPTY = 0,
	PRELOAD_CLEAR,
	PRELOAD_DIRTY
};

static enum preload_state preload[YL_PARAMS_COUNT];
static char yl_params[YL_PARAMS_COUNT][ONE_BLOCK_SIZE];

static struct miscdevice yl_params_dev1;

/* resources should be protect by yl_param_lock:
 * preload, yl_params, kernel_buf,
 * */
static DEFINE_MUTEX(yl_param_lock);

struct yl_params_data {
	struct mmc_card	*card;
	struct hd_struct *part;
};
static struct yl_params_data yl_params_priv;
static struct yl_params_data yl_misc_priv;

/*
 * Fill in the mmc_request structure given a set of transfer parameters.
 */
static void yl_params_prepare_mrq(struct mmc_card *card,
		struct mmc_request *mrq, struct scatterlist *sg, unsigned sg_len,
		unsigned dev_addr, unsigned blocks, unsigned blksz, int write)
{
	BUG_ON(!mrq || !mrq->cmd || !mrq->data || !mrq->stop);

	if (blocks > 1) {
		mrq->cmd->opcode = write ?
			MMC_WRITE_MULTIPLE_BLOCK : MMC_READ_MULTIPLE_BLOCK;
	} else {
		mrq->cmd->opcode = write ?
			MMC_WRITE_BLOCK : MMC_READ_SINGLE_BLOCK;
	}

	mrq->cmd->arg = dev_addr;
	if (!mmc_card_blockaddr(card))
		mrq->cmd->arg <<= 9;

	mrq->cmd->flags = MMC_RSP_R1 | MMC_CMD_ADTC;

	if (blocks == 1)
		mrq->stop = NULL;
	else {
		mrq->stop->opcode = MMC_STOP_TRANSMISSION;
		mrq->stop->arg = 0;
		mrq->stop->flags = MMC_RSP_R1B | MMC_CMD_AC;
	}

	mrq->data->blksz = blksz;
	mrq->data->blocks = blocks;
	mrq->data->flags = write ? MMC_DATA_WRITE : MMC_DATA_READ;
	mrq->data->sg = sg;
	mrq->data->sg_len = sg_len;

	mmc_set_data_timeout(mrq->data, card);
}

/*
 * Wait for the card to finish the busy state
 */
static int yl_params_wait_busy(struct mmc_card *card)
{
	int ret, busy;
	struct mmc_command cmd;

	busy = 0;
	do {
		memset(&cmd, 0, sizeof(struct mmc_command));

		cmd.opcode = MMC_SEND_STATUS;
		cmd.arg = card->rca << 16;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

		ret = mmc_wait_for_cmd(card->host, &cmd, 0);
		if (ret)
			break;

		if (!busy && !(cmd.resp[0] & R1_READY_FOR_DATA)) {
			busy = 1;
			pr_info("%s: Warning: Host did not "
					"wait for busy state to end.\n",
					mmc_hostname(card->host));
		}
	} while (!(cmd.resp[0] & R1_READY_FOR_DATA));

	return ret;
}

/*
 * Checks that a normal transfer didn't have any errors
 */
static int yl_params_check_res(struct mmc_request *mrq)
{
	int ret = 0;

	BUG_ON(!mrq || !mrq->cmd || !mrq->data);

	if (mrq->cmd->error)
		ret = mrq->cmd->error;
	else if (mrq->data->error)
		ret = mrq->data->error;
	else if (mrq->stop && mrq->stop->error)
		ret = mrq->stop->error;
	else if (mrq->data->bytes_xfered !=
			mrq->data->blocks * mrq->data->blksz)
		ret = -EIO;
	else
		ret = mrq->data->bytes_xfered;

	if (ret < 0)
		pr_err("%s return ret=%d\n", __func__, ret);
	return ret;
}

static int yl_params_transfer(struct yl_params_data *priv,
		sector_t offset, const char *buf, int write)
{
	struct mmc_request mrq;
	struct mmc_command cmd;
	struct mmc_command stop;
	struct mmc_data data;
	struct scatterlist sg;
	struct mmc_card *card;

	memset(&mrq, 0, sizeof(struct mmc_request));
	memset(&cmd, 0, sizeof(struct mmc_command));
	memset(&data, 0, sizeof(struct mmc_data));
	memset(&stop, 0, sizeof(struct mmc_command));

	mrq.cmd = &cmd;
	mrq.data = &data;
	mrq.stop = &stop;

	sg_init_one(&sg, buf, ONE_BLOCK_SIZE);

	card = priv->card;

	mmc_rpm_hold(card->host, &card->dev);
	mmc_claim_host(card->host);
	yl_params_prepare_mrq(card, &mrq, &sg, 1,
			priv->part->start_sect + offset,
			1, ONE_BLOCK_SIZE, write);
	mmc_wait_for_req(card->host, &mrq);
	yl_params_wait_busy(card);
	mmc_release_host(card->host);
	mmc_rpm_release(card->host, &card->dev);

	return yl_params_check_res(&mrq);
}

static int yl_params_get_pageoffset(const char *buf)
{
	int i;

	for (i = 0; i < YL_PARAMS_COUNT; i++) {
		if (!strncmp(yl_params_map[i], buf, TAG_LENGTH)) {
			pr_debug("find tag name is [%s]\n", buf);
			return i;
		}
	}
	return -ERANGE;
}

/* caller should only provide buffer of size 512 bytes exactly,
 * */
static ssize_t yl_params_read_locked(char *buf)
{
	int block;
	ssize_t len = ONE_BLOCK_SIZE;

	block = yl_params_get_pageoffset(buf);
	if (block < 0) {
		pr_err("Can't find tag %s, line %d!\n", buf, __LINE__);
		return -ERANGE;
	}

	if (preload[block] == PRELOAD_EMPTY) {
		len = yl_params_transfer(&yl_params_priv, block,
				yl_params[block], false);
		if (len == ONE_BLOCK_SIZE)
			preload[block] = PRELOAD_CLEAR;
	}
	if (len == ONE_BLOCK_SIZE)
		memcpy(buf, yl_params[block], ONE_BLOCK_SIZE);
	else
		pr_err("read block %s failed, ret %zu\n", buf, len);
	return len;
}

/* caller should only provide buffer of size 512 bytes exactly,
 * as only read-modify-write is accept
 * */
static ssize_t yl_params_write_locked(const char *buf)
{
	int block;
	ssize_t len;
	int retry = 0;

	block = yl_params_get_pageoffset(buf);
	if(block < 0) {
		pr_err("Can't find tag %s, line %d!\n", buf, __LINE__);
		return -ERANGE;
	}

	while (1) {
		len = yl_params_transfer(&yl_params_priv, block, buf, true);
		if (len == ONE_BLOCK_SIZE) {
			memcpy(yl_params[block], buf, ONE_BLOCK_SIZE);
			preload[block] = PRELOAD_CLEAR;
			break;
		} else {
			preload[block] = PRELOAD_DIRTY;
			retry ++;
			if (retry < 3)
				pr_err("write block %s failed, retry\n", buf);
			else {
				pr_err("write block %s failed 3 times, stop\n", buf);
				break;
			}
		}
	}
	/* FIXME if write transfer failed, what data is in EMMC really?
	 * Is it same as pre-loaded, or as written buf, or not consistent?
	 * And what should we deal with pre-loaded data?
	 * */
	return len;
}

ssize_t yl_params_kernel_read(uint8_t *buf, ssize_t count)
{
	ssize_t len;

	if (count != ONE_BLOCK_SIZE)
		return -EINVAL;

	mutex_lock(&yl_param_lock);
	len = yl_params_read_locked(buf);
	mutex_unlock(&yl_param_lock);
	return len;
}
EXPORT_SYMBOL_GPL(yl_params_kernel_read);

ssize_t yl_params_kernel_write(const uint8_t *buf, ssize_t count)
{
	ssize_t len;

	if (count != ONE_BLOCK_SIZE)
		return -EINVAL;

	mutex_lock(&yl_param_lock);
	len = yl_params_write_locked(buf);
	mutex_unlock(&yl_param_lock);
	return len;
}
EXPORT_SYMBOL_GPL(yl_params_kernel_write);

static char kernel_buf[ONE_BLOCK_SIZE];
static ssize_t yl_params_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	ssize_t len;
	size_t count_temp;

	if (count <= TAG_LENGTH)
 		pr_debug("%s read count %zu\n", __func__, count);

	mutex_lock(&yl_param_lock);
	count_temp = copy_from_user(kernel_buf, buf,
			count < TAG_LENGTH ? count : TAG_LENGTH);
	if (count_temp != 0)
		pr_info("%s copy from left %zu\n", __func__, count_temp);

	len = yl_params_read_locked(kernel_buf);
	if (len > 0) {
		if (len > count)
			len = count;
		count_temp = copy_to_user(buf + TAG_LENGTH, kernel_buf + TAG_LENGTH, len > TAG_LENGTH ? len - TAG_LENGTH : 0);
		if (count_temp) {
			pr_info("%s copy to left %zu\n", __func__, count_temp);
			len -= count_temp;
		}
	}
	mutex_unlock(&yl_param_lock);
	return len;
}

static ssize_t yl_params_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	ssize_t len;

	if (count <= TAG_LENGTH)
 		pr_debug("%s write count %zu\n", __func__, count);

	/* We only support reading a maximum of a flash block */
	if (count > ONE_BLOCK_SIZE)
		count = ONE_BLOCK_SIZE;

	mutex_lock(&yl_param_lock);
	len = yl_params_read_locked(kernel_buf);
	if (len > 0) {
		ssize_t res;
		res = copy_from_user(kernel_buf, buf, count);
		if (res != 0)
			pr_info("%s copy left %zu\n", __func__, res);
		len = count - res;

		res = yl_params_write_locked(kernel_buf);
		if (res <= 0)
			len = res;
	}
	mutex_unlock(&yl_param_lock);
	return len;
}

static char *preload_str(enum preload_state state)
{
	if (state == PRELOAD_EMPTY)
		return "empty";
	if (state == PRELOAD_CLEAR)
		return "clear";
	if (state == PRELOAD_DIRTY)
		return "dirty";
	return "unknown";
}

ssize_t show_preload(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned int i;
	unsigned int len = 0;
	mutex_lock(&yl_param_lock);
	for (i = 0; i < YL_PARAMS_COUNT; i++)
		len += sprintf(buf + len, "%16s\t%s\n",
				yl_params_map[i],
				preload_str(preload[i]));
	mutex_unlock(&yl_param_lock);
	return len;
}
DEVICE_ATTR(preload, S_IRUSR | S_IRGRP, show_preload, NULL);

static const struct file_operations yl_params_fops = {
	.owner		= THIS_MODULE,
	.read		= yl_params_read,
	.write		= yl_params_write,
};

static struct miscdevice yl_params_dev1 = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "yl_params1",
	.fops = &yl_params_fops,
};

static char recovery_buf[ONE_BLOCK_SIZE];
static int yl_reboot(struct notifier_block *nb, unsigned long event, void *unused)
{
	if (event == SYS_RESTART && unused && !strcmp(unused, "recovery")) {
		int ret;

		ret = yl_params_transfer(&yl_misc_priv, 0, recovery_buf, true);
		if (!ret)
			pr_err("write recover flag FAILED!\n");
	}
	pr_err("system off event=%ld\n", event);
	return NOTIFY_DONE;
}

static struct notifier_block yl_reboot_fg_notifier = {
	.notifier_call = yl_reboot,
};

int yl_params_init(struct mmc_card *card)
{
	unsigned int res;

	if (yl_params_priv.card) {
		pr_debug("re-init denied, old card %p, new card %p!\n",
				yl_params_priv.card, card);
		return -EEXIST;
	}

	strcpy(recovery_buf, "boot-recovery");
	/* add ourselves to the reboot_notifier_list.
	 * We support adb reboot-recovery,
	 * system can't catch this command,
	 * so it's kernel duty to write boot-recovery flag.
	 * As we have a common reboot mechanism using scratch register,
	 * which also support recovery,
	 * why still is this flag needed?
	 * Enn, maybe we dare of power cut-off during reboot.
	 * */
	res = register_reboot_notifier(&yl_reboot_fg_notifier);
	if (res != 0)
		pr_debug("can't register reboot notifier\n");

	yl_params_priv.card = card;
	yl_misc_priv.card = card;

	res = misc_register(&yl_params_dev1);
	/* If params v1 register fail, it doesn't matter */
	if (res) {
		pr_debug("regist yl_params1 failed %d\n", res);
		goto out;
	}

	device_create_file(yl_params_dev1.this_device, &dev_attr_preload);
	return 0;

out:
	unregister_reboot_notifier(&yl_reboot_fg_notifier);
	pr_err("%s: Driver Initialisation failed\n", __FILE__);
	return res;
}

void notify_ylparams(struct hd_struct *part)
{
	/* long long ago, it is
	 * char *pname = part->partition_name;
	 * */
	char *pname = part->info->volname;

	if (!strcmp(pname, YL_PARAMS_NAME))
		yl_params_priv.part = part;
	else if (!strcmp(pname, YL_MISC_NAME))
		yl_misc_priv.part = part;
	else
		return;
}
