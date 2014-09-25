/**
 * Hillcrest SensorHub driver
 *
 * Copyright (C) 2013-14 Hillcrest Labs, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>
 */

#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include "sensorhub-core.h"

#undef pr_fmt
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#define SENSORHUB_SYSFS_DEBUG
#define SET_FEATURE_BUFSIZE (62)
#define GET_FEATURE_BUFSIZE (62)

inline struct sensorhub_drv_data * from_misc_dev(struct device *dev) {
	struct miscdevice * misc = (struct miscdevice *) dev_get_drvdata(dev);
	return container_of(misc, struct sensorhub_drv_data, misc);
}

static const char * sensorhub_core_states[] = {
	"unknown",            // SENSORHUB_STATE_UNKNOWN
	"app",                // SENSORHUB_STATE_APP
	"bootloader",         // SENSORHUB_STATE_BOOTLOADER
	"bootloader-noimage", // SENSORHUB_STATE_BOOTLOADER_NOIMAGE
	"bootloader-hungapp", // SENSORHUB_STATE_BOOTLOADER_HUNG_APP
	"download",           // SENSORHUB_STATE_DOWNLOAD
	"download",           // SENSORHUB_STATE_DOWNLOAD_ENTER
	"download-failed"	  // SENSORHUB_STATE_DOWNLOAD_FAILED
};

static ssize_t sensorhub_sysfs_show_state(struct device *dev, struct device_attribute *attr, char *buf) {
	struct sensorhub_drv_data * ddata = from_misc_dev(dev);
	if (ddata->state < 0 && ddata->state >= ARRAY_SIZE(sensorhub_core_states)) {
		return -EFAULT;
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", sensorhub_core_states[ddata->state]);
}

static ssize_t sensorhub_sysfs_store_state(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	struct sensorhub_drv_data * ddata = from_misc_dev(dev);
	int rc;

	if (strncmp(buf, "update\n", count) != 0 && strncmp(buf, "update", count) != 0) {
		return -EINVAL;
	}

	rc = sensorhub_core_dfu_request(ddata);
	return rc ? rc : count;

}

static ssize_t sensorhub_sysfs_show_version(struct device *dev, struct device_attribute *attr, char *buf) {
	struct sensorhub_drv_data * ddata = from_misc_dev(dev);
	return scnprintf(buf, PAGE_SIZE,
		"0x%04x\n", ddata->versionID
	);
}

static ssize_t sensorhub_sysfs_show_product(struct device *dev, struct device_attribute *attr, char *buf) {
	struct sensorhub_drv_data * ddata = from_misc_dev(dev);
	return scnprintf(buf, PAGE_SIZE,
		"0x%04x\n", ddata->productID
	);
}

static ssize_t sensorhub_sysfs_show_vendor(struct device *dev, struct device_attribute *attr, char *buf) {
	struct sensorhub_drv_data * ddata = from_misc_dev(dev);
	return scnprintf(buf, PAGE_SIZE,
		"0x%04x\n", ddata->vendorID
	);
}

static ssize_t sensorhub_sysfs_show_report_descriptor(struct device *dev, struct device_attribute *attr, char *buf) {
	struct sensorhub_drv_data * ddata = from_misc_dev(dev);
	if (!ddata->report_descriptor.buf) {
		return -ENODATA;
	}

	if (ddata->report_descriptor.len > PAGE_SIZE) {
		return -EFAULT;
	}

	memcpy(buf, ddata->report_descriptor.buf, ddata->report_descriptor.len);
	return ddata->report_descriptor.len;
}

static ssize_t sensorhub_sysfs_store_reset(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	struct sensorhub_drv_data * ddata = from_misc_dev(dev);
	int rc;
	enum sensorhub_reset reset;

	if (strncmp(buf, "hard", 4) == 0) {
		reset = SENSORHUB_RESET_HARD;
	} else {
		reset = SENSORHUB_RESET_SOFT;
	}

	rc = sensorhub_core_reset(ddata, reset, true);
	return rc ? rc : count;
}

#ifdef DEBUG
#define SENSORHUB_BUILD_INFO \
	"Version:        " SENSORHUB_VERSION "\n" \
	"Build date:     " __DATE__ " " __TIME__ "\n"
#else
#define SENSORHUB_BUILD_INFO ""
#endif
static ssize_t sensorhub_sysfs_show_stats(struct device *dev, struct device_attribute *attr, char *buf) {
	struct sensorhub_drv_data * ddata = from_misc_dev(dev);
	struct sensorhub_client * client;
	unsigned int count = 0;

	mutex_lock(&ddata->clients_lock);
	list_for_each_entry(client, &ddata->clients, list) {
		count++;
	};
	mutex_unlock(&ddata->clients_lock);

	return scnprintf(buf, PAGE_SIZE,
		"Name:           %s\n"
		"Flags:          %lx\n"
		"Interrupts:     %u\n"
		"Reset count:    %u\n"
		"Dropped:        %u\n"
		"Errors:         %u\n"
		"Received:       %u\n"
		"Received bytes: %u\n"
		"Sent:           %u\n"
		"Sent bytes:     %u\n"
		"Suspends:       %u\n"
		"Retries:        %u\n"
		"Clients:        %u\n"
		SENSORHUB_BUILD_INFO
		, ddata->name
		, ddata->flags
		, ddata->stats.interrupts
		, ddata->stats.resetCount
		, ddata->stats.dropped
		, ddata->stats.errorCount
		, ddata->stats.received
		, ddata->stats.receivedBytes
		, ddata->stats.sent
		, ddata->stats.sentBytes
		, ddata->stats.suspendCount
		, ddata->stats.retries
		, count
	);
}

#ifdef SENSORHUB_SYSFS_DEBUG
static ssize_t sensorhub_sysfs_store_insert(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	struct sensorhub_drv_data * ddata = from_misc_dev(dev);
	int ret;

	// print_hex_dump(KERN_DEBUG, "insert: ", DUMP_PREFIX_OFFSET, 16, 1, buf, count, true);

	struct sensorhub_input_event event = {
		.timestamp = sensorhub_get_time_ns()
	};

	ret = sensorhub_core_notify_input(ddata, &event, buf, count);
	if (ret > 0) {
		// we dropped some stuff
		return -EAGAIN;
	}
	wake_up_interruptible(&ddata->wait);

	return ret ? ret : count;
}

static ssize_t sensorhub_sysfs_store_force_hangup(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {
	struct sensorhub_drv_data * ddata = from_misc_dev(dev);
	int ret;

	ret = sensorhub_core_hangup(ddata);
	return ret ? ret : count;
}
#endif

static DEVICE_ATTR(state, S_IRUGO | S_IWUSR, sensorhub_sysfs_show_state, sensorhub_sysfs_store_state);
static DEVICE_ATTR(version, S_IRUGO, sensorhub_sysfs_show_version, NULL);
static DEVICE_ATTR(product, S_IRUGO, sensorhub_sysfs_show_product, NULL);
static DEVICE_ATTR(vendor, S_IRUGO, sensorhub_sysfs_show_vendor, NULL);
static DEVICE_ATTR(descriptor, S_IRUGO, sensorhub_sysfs_show_report_descriptor, NULL);
static DEVICE_ATTR(reset, S_IWUSR, NULL, sensorhub_sysfs_store_reset);
static DEVICE_ATTR(stats, S_IRUGO, sensorhub_sysfs_show_stats, NULL);

#ifdef SENSORHUB_SYSFS_DEBUG
static DEVICE_ATTR(insert, S_IWUSR, NULL, sensorhub_sysfs_store_insert);
static DEVICE_ATTR(force_hangup, S_IWUSR, NULL, sensorhub_sysfs_store_force_hangup);
// static DEVICE_ATTR(force_getinput, S_IWUSR, NULL, sensorhub_sysfs_store_force_getinput);
#endif

static struct attribute * sensorhub_drv_attributes[] = {
	&dev_attr_state.attr,
	&dev_attr_version.attr,
	&dev_attr_product.attr,
	&dev_attr_vendor.attr,
	&dev_attr_descriptor.attr,
	&dev_attr_reset.attr,

#ifdef SENSORHUB_SYSFS_DEBUG
	&dev_attr_stats.attr,
	&dev_attr_insert.attr,
	&dev_attr_force_hangup.attr,
	// &dev_attr_force_getinput.attr,
#endif

	NULL /* keep last*/
};

static const struct attribute_group sensorhub_drv_attr_group = {
	.attrs = sensorhub_drv_attributes,
};

static int sensorhub_sysfs_register(struct sensorhub_drv_data * ddata) {
	return sysfs_create_group(&ddata->dev->kobj, &sensorhub_drv_attr_group);
}

static void sensorhub_sysfs_unregister(struct sensorhub_drv_data * ddata) {
	sysfs_remove_group(&ddata->dev->kobj,  &sensorhub_drv_attr_group);
}

// A report set operation initiated from user space
// first byte of user_buf must be the report id
// type - output or feature
static int sensorhub_core_usr_set(struct sensorhub_drv_data * ddata,
								 enum sensorhub_report_type type,
								 const u8 __user* user_buf, size_t user_len) {

	u8 buf[SET_FEATURE_BUFSIZE];
	int ret = 0;
	if (user_len > sizeof(buf)) {
		// the user report is too big for us to handle
		return -EMSGSIZE;
	}

	if (copy_from_user(buf, user_buf, user_len)) {
		return -EFAULT;
	}

	ret = ddata->transport->set_report(ddata, type,
										 buf[0], buf + 1, user_len - 1);
	if (ret > 0) {
		return ret + 1; // include the report id in num bytes written.
	}

	return ret;
}

static int __sensorhub_drv_count = 0;

// File Operations ------------------------------------------------------------
// sensorhub file open fop
static int sensorhub_fop_open(struct inode * inode, struct file * file) {
	struct sensorhub_drv_data * ddata =
			container_of(file->private_data, struct sensorhub_drv_data, misc);

	int rc = 0;
	struct sensorhub_client * client;

	// create a entry for this client
	dev_dbg(ddata->dev, "%s opened by %s. %s%s%s%s\n" \
			, ddata->name, current->comm
			, file->f_flags & O_RDONLY   ? "RO" : ""
			, file->f_flags & O_WRONLY   ? "WO" : ""
			, file->f_flags & O_RDWR     ? "RW" : ""
			, file->f_flags & O_NONBLOCK ? " N" : " B"
	);
	client = kzalloc(sizeof(struct sensorhub_client), GFP_KERNEL);
	if (!client) {
		dev_err(ddata->dev, "Failed allocating sensorhub_client\n");
		rc = -ENOMEM;
		goto exit;
	}

	spin_lock_init(&client->lock);
	client->ddata = ddata;
	client->resetCount = ddata->stats.resetCount;
	INIT_KFIFO(client->queue);

	mutex_lock(&ddata->clients_lock);
	list_add(&client->list, &ddata->clients);
	mutex_unlock(&ddata->clients_lock);

	file->private_data = client;
exit:
	return rc;
}

// sensorhub file release fop
static int sensorhub_fop_release(struct inode * inode, struct file * file) {
	struct sensorhub_client * client = file->private_data;
	struct sensorhub_drv_data * ddata = client->ddata;
	dev_dbg(ddata->dev, "%s closed by %s\n", ddata->name, current->comm);
	mutex_lock(&ddata->clients_lock);
	list_del(&client->list);
	mutex_unlock(&ddata->clients_lock);
	kfree(client);
	return 0;
}

// sensorhub file read fop
static ssize_t sensorhub_fop_read(struct file * file, char __user * buf,
		size_t count, loff_t * ppos) {

	int ret;
	size_t copied_meta;
	size_t copied_report;
	struct sensorhub_client * client = file->private_data;
	struct sensorhub_drv_data * ddata = client->ddata;

	if (!(file->f_flags & O_NONBLOCK)) {
		// wait for data or a disconnect
		ret = wait_event_interruptible(
			ddata->wait,
			!kfifo_is_empty(&client->queue) || test_bit(SENSOR_HUB_DISCONNECTED, &ddata->flags)
		);
		if (ret) {
			return ret;
		}
	}

	if (client->resetCount != ddata->stats.resetCount) {
		dev_dbg(ddata->dev, "reset count not in sync!!!");
		// hardware has reset, need this for readers not using poll
		client->resetCount = ddata->stats.resetCount;
		return -ECONNRESET;
	}

	if (test_bit(SENSOR_HUB_DISCONNECTED, &ddata->flags)) {
		// device is shutting down, notify client what we have disconnected
		return -ENODEV;
	}

	if (kfifo_is_empty(&client->queue)) {
		if (file->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		} else {
			return 0; // ??? shouldn't get here...
		}
	}

	spin_lock(&client->lock);
	if (client->report_metadata) {
		// copy the metadata struct to the user buffer first
		int ret1 = kfifo_to_user(&client->queue, buf, count, &copied_meta);
		// append the report to it (fill in the var-length msg)
		int ret2 = kfifo_to_user(&client->queue, buf + copied_meta, count - copied_meta, &copied_report);
		if (ret1 || ret2) {
			ret = ret1 || ret2;
		} else {
			ret = copied_meta + copied_report;
		}
	} else {
		// skip over the metadata part
		kfifo_skip(&client->queue);
		ret = kfifo_to_user(&client->queue, buf, count, &copied_report);
		ret = ret ? ret : copied_report;
	}
	spin_unlock(&client->lock);

	return ret;
}

// sensorhub file read fop
static ssize_t sensorhub_fop_write(struct file * file, const char __user * buf,
		size_t count, loff_t * ppos) {

	struct sensorhub_client * client = file->private_data;
	struct sensorhub_drv_data * ddata = client->ddata;

	// TODO -- how should we handle this w/ O_NONBLOCK?
	return sensorhub_core_usr_set(ddata, SENSORHUB_REPORT_OUTPUT, buf, count);
}

// sensorhub file ioctl fop
static long sensorhub_fop_ioctl(struct file *file, unsigned int cmd, unsigned long arg) {
	struct sensorhub_client * client = file->private_data;
	struct sensorhub_drv_data * ddata = client->ddata;


	void __user *user_arg = (void __user*) arg;
	ssize_t user_len = _IOC_SIZE(cmd);
	long ret = 0;

	//dev_dbg(ddata->dev, "%s():%d -- cmd=%x, user_len=%zd\n", __FUNCTION__, __LINE__, cmd, user_len);
	if (!user_len) {
		return -EMSGSIZE;
	}

	if (_IOC_NR(cmd) == _IOC_NR(SH_IOC_GET_SYSFS_PATH(0))) {
		if (user_len < (strlen(ddata->name) + 1)) {
			return -EMSGSIZE;
		}

		ret = copy_to_user(user_arg, ddata->name, user_len);
		return ret ? -EFAULT : user_len;
	}

	if (_IOC_NR(cmd) == _IOC_NR(SH_IOC_GET_FEATURE(0))) {
		u8 buf[GET_FEATURE_BUFSIZE];
		u8 feature;

		// user_len = 1 byte report id + report data
		size_t reportLen = user_len - 1; /* ignore the first byte*/

		if (WARN_ON(reportLen < 1)) {
			// user report must contain at least 1 byte report id
			return -EMSGSIZE;
		}

		if (get_user(feature, (u8 __user *) user_arg)) {
			return -EFAULT;
		}

		ret =  ddata->transport->get_report(ddata, SENSORHUB_REPORT_FEATURE,
						feature, buf, sizeof(buf));
		if (ret < 0) {
			return ret;
		}

		if (ret + 1 <= user_len) {
			// All is good, copy report, leaving id in place, to user buffer
			if (copy_to_user(user_arg + 1, buf, ret)) {
				return -EFAULT;
			}

			return ret + 1;
		} else {
			return -EMSGSIZE;
		}

		return ret;
	}

	if (_IOC_NR(cmd) == _IOC_NR(SH_IOC_SET_FEATURE(0))) {
		return sensorhub_core_usr_set(ddata, SENSORHUB_REPORT_FEATURE,
									  user_arg, user_len);
	}

	if (cmd == SH_IOC_SET_REPORT_METADATA) {
		int enabled;
		if (get_user(enabled, (u8 __user *) user_arg)) {
			return -EFAULT;
		}
		client->report_metadata = enabled;
		return 0;
	}

	return -EINVAL;
}

// sensorhub file poll fop
static unsigned int sensorhub_fop_poll(struct file * file, poll_table * pt) {
	struct sensorhub_client * client = file->private_data;
	struct sensorhub_drv_data * ddata = client->ddata;

	unsigned int mask = 0;

	poll_wait(file, &ddata->wait, pt);
	if (test_bit(SENSOR_HUB_DISCONNECTED, &ddata->flags)) {
		return POLLHUP | POLLERR;
	}

	if (client->resetCount != ddata->stats.resetCount) {
		client->resetCount = ddata->stats.resetCount;
		mask |= POLLHUP; /* notify that we reconnected */
	}

	if (kfifo_len(&client->queue)) {
		mask |= POLLIN | POLLRDNORM;
	}

	return mask;
}

static const struct file_operations sensorhub_drv_fops = {
	.owner   = THIS_MODULE,
	.open    = sensorhub_fop_open,
	.release = sensorhub_fop_release,
	.read    = sensorhub_fop_read,
	.write   = sensorhub_fop_write,
	.poll    = sensorhub_fop_poll,
	.unlocked_ioctl = sensorhub_fop_ioctl,
};

// Dispatch a single HID event to all clients
int sensorhub_core_notify_input(struct sensorhub_drv_data * ddata,
                                const struct sensorhub_input_event * input,
                                const u8 * buf, size_t len) {

	struct sensorhub_client * client;
	bool dropped = false;

	if (WARN_ON(!input)) {
		return -EINVAL;
	}

	if (len != input->len) {
		return -EINVAL;
	}

	mutex_lock(&ddata->clients_lock);
	if (list_empty(&ddata->clients)) {
		ddata->stats.dropped++;
		dropped = true;
	} else {
		list_for_each_entry(client, &ddata->clients, list) {
			// ensure that there is enough room for this record in the fifo
			// we place two entries (metadata event without input) and
			// the actual report
			size_t needed = sizeof(*input) + SENSORHUB_FIFO_REC_SIZE
						  + len + SENSORHUB_FIFO_REC_SIZE;
			spin_lock(&client->lock);
			while (kfifo_avail(&client->queue) < needed) {
				// drop a pair of metadata and report records
				kfifo_skip(&client->queue);
				kfifo_skip(&client->queue);
				ddata->stats.dropped++;
				dropped = true;
			}
			kfifo_in(&client->queue, input, sizeof(*input));
			kfifo_in(&client->queue, buf, len);
			spin_unlock(&client->lock);
		}
	}
	ddata->stats.received++;
	ddata->stats.receivedBytes += len;
	mutex_unlock(&ddata->clients_lock);

	return dropped ? 1 : 0;
}

// Reset the hub and wait until a power on ack is received
int sensorhub_core_reset(struct sensorhub_drv_data * ddata, enum sensorhub_reset reset, bool wait) {
	int ret = 0;

	set_bit(SENSOR_HUB_RESET_PENDING, &ddata->flags);
	ret = ddata->transport->reset(ddata, reset);

	// Wait for isr to receive the reset message and clear the bit
	if (!ret && wait) {
		ret = wait_event_interruptible_timeout(
			ddata->wait,
			!test_bit(SENSOR_HUB_RESET_PENDING, &ddata->flags),
			msecs_to_jiffies(5000)
		);

		if (ret > 0) {
			return 0;
		} else {
			dev_dbg(ddata->dev, "timedout waiting for reset\n");
			return -ETIMEDOUT;
		}
	}

	return ret;
}

int sensorhub_core_notify_reset(struct sensorhub_drv_data * ddata) {
	struct sensorhub_client * client;
	dev_dbg(ddata->dev, "hardware reset\n");

	// flush the buffers
	list_for_each_entry(client, &ddata->clients, list) {
		spin_lock(&client->lock);
		while (!kfifo_is_empty(&client->queue)) {
			kfifo_skip(&client->queue);
			++ddata->stats.dropped;
		}
		spin_unlock(&client->lock);
	};

	++ddata->stats.resetCount;
	clear_bit(SENSOR_HUB_RESET_PENDING, &ddata->flags);
	return 0;
}

// Force clients to disconnect
int sensorhub_core_hangup(struct sensorhub_drv_data * ddata) {
	set_bit(SENSOR_HUB_DISCONNECTED, &ddata->flags);
	wake_up_interruptible(&ddata->wait);
	// TODO - Do we add a wait queue here to wait for all clients to disconnect?
	return 0;
}

int sensorhub_core_dfu_request(struct sensorhub_drv_data * ddata) {

	int rc;
	const struct firmware * fw = NULL;

	// TODO formal name
	char fw_name[] = "FSP3xx.bin";

	if (IS_ERR_OR_NULL(ddata)) {
		BUG_ON(1);
		return -EINVAL;
	}

	if (ddata->state == SENSORHUB_STATE_DOWNLOAD) {
		dev_warn(ddata->dev, "DFU: already in dfu mode\n");
		return -EBUSY;
	}

	if (!ddata->transport->dfu_perform ||
		!ddata->transport->dfu_abort   ||
		!ddata->transport->dfu_validate) {
		dev_warn(ddata->dev, "DFU: not supported\n");
		return -ENOSYS;
	}

	rc = request_firmware(&fw, fw_name, ddata->dev);
	if (rc || IS_ERR_OR_NULL(fw)) {
		dev_warn(ddata->dev, "DFU: %s: request_firmware() returned with error: %d\n", __func__, rc);
		goto exit_abort;
	}

	rc = ddata->transport->dfu_validate(ddata, fw->data, fw->size);
	if (rc) {
		dev_err(ddata->dev, "DFU: %s: firmware file did not pass validation checks\n", __func__);
		goto exit_fw_release;
	}

	rc = ddata->transport->dfu_perform(ddata, fw->data, fw->size);
	if (!rc) {
		dev_dbg(ddata->dev, "DFU: completed successfully\n");
	} else {
		dev_dbg(ddata->dev, "DFU: %s: dfu_perform() returned with error: %d\n", __func__, rc);
	}

	release_firmware(fw);
	sysfs_notify(&ddata->dev->kobj, NULL, "state");

	return rc;

exit_fw_release:
	release_firmware(fw);
exit_abort:
	ddata->transport->dfu_abort(ddata);
	sysfs_notify(&ddata->dev->kobj, NULL, "state");
	return rc;
}

// Allocate Driver instance
int sensorhub_core_register(struct sensorhub_drv_data * ddata) {
	int err = 0;

	// add static asserts here:
	BUILD_BUG_ON(SENSOR_HUB_FLAGS_LAST > (sizeof(ddata->flags) * 8));

	if (ddata->dev) {
		pr_err("ddata struct's dev field should not be initialized at this time");
		return -EINVAL;
	}

	mutex_init(&ddata->clients_lock);
	INIT_LIST_HEAD(&ddata->clients);
	init_waitqueue_head(&ddata->wait);
	scnprintf(ddata->name, sizeof(ddata->name),
			  "sensorhub%d", __sensorhub_drv_count++);
	ddata->misc.minor = MISC_DYNAMIC_MINOR;
	ddata->misc.name  = ddata->name;
	ddata->misc.fops  = &sensorhub_drv_fops;

	// init misc device
	err = misc_register(&ddata->misc);
	if (err < 0) {
		pr_err("misc_register failed\n");
		goto exit_misc_failed;
	}
	// create this shortcut;
	ddata->dev = get_device(ddata->misc.this_device);

	err = sensorhub_sysfs_register(ddata);
	if (err)
		goto exit_sysfs_failed;

	goto exit;

exit_sysfs_failed:
	misc_deregister(&ddata->misc);
	put_device(ddata->dev);
	ddata->dev = NULL;
exit_misc_failed:
exit:
	return err;
}

void sensorhub_core_unregister(struct sensorhub_drv_data * ddata) {
	dev_dbg(ddata->dev, "unregister %s\n", ddata->name);
	sensorhub_core_hangup(ddata);
	sensorhub_sysfs_unregister(ddata);
	put_device(ddata->dev);
	ddata->dev = NULL;
	misc_deregister(&ddata->misc);
}

EXPORT_SYMBOL_GPL(sensorhub_core_notify_input);
EXPORT_SYMBOL_GPL(sensorhub_core_reset);
EXPORT_SYMBOL_GPL(sensorhub_core_notify_reset);
EXPORT_SYMBOL_GPL(sensorhub_core_hangup);
EXPORT_SYMBOL_GPL(sensorhub_core_dfu_request);
EXPORT_SYMBOL_GPL(sensorhub_core_register);
EXPORT_SYMBOL_GPL(sensorhub_core_unregister);

MODULE_DESCRIPTION("Freespace SensorHub Driver");
MODULE_AUTHOR("Khalid Zubair <kzubair@hcrest.com>");
MODULE_VERSION(SENSORHUB_VERSION);
MODULE_LICENSE("GPL");
