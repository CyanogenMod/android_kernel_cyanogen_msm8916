/*
 * Copyright (c) 2014 Yamaha Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include "yas.h"


#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533
#define YAS_MSM_NAME		"compass"
#define YAS_MSM_VENDOR		"Yamaha"
#define YAS_MSM_VERSION		(1)
#define YAS_MSM_HANDLE		(1)
#define YAS_MSM_TYPE		(2)
#define YAS_MSM_MIN_DELAY	(10000)
#define YAS_MSM_MAX_RANGE	(1200)
#define YAS_MSM_RESOLUTION	"1"
#define YAS_MSM_SENSOR_POWER	"0.40"
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
#define YAS_MSM_NAME		"compass"
#define YAS_MSM_VENDOR		"Yamaha"
#define YAS_MSM_VERSION		(1)
#define YAS_MSM_HANDLE		(1)
#define YAS_MSM_TYPE		(2)
#define YAS_MSM_MIN_DELAY	(10000)
#define YAS_MSM_MAX_RANGE	(2000)
#define YAS_MSM_RESOLUTION	"1"
#define YAS_MSM_SENSOR_POWER	"0.28"
#endif

static struct i2c_client *this_client;
int compass_yas533_enable=0;

struct yas_state {
	struct mutex lock;
	struct yas_mag_driver mag;
	struct input_dev *input_dev;
	struct delayed_work work;
	int32_t poll_delay;
	atomic_t enable;
	int32_t compass_data[3];
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend sus;
#endif
	struct device *dev;
	struct class *class;
};

static int yas_device_open(int32_t type)
{
	return 0;
}

static int yas_device_close(int32_t type)
{
	return 0;
}

static int yas_device_write(int32_t type, uint8_t addr, const uint8_t *buf,
		int len)
{
	uint8_t tmp[2];
	if (sizeof(tmp) - 1 < len)
		return -1;
	tmp[0] = addr;
	memcpy(&tmp[1], buf, len);
	if (i2c_master_send(this_client, tmp, len + 1) < 0)
		return -1;
	return 0;
}

static int yas_device_read(int32_t type, uint8_t addr, uint8_t *buf, int len)
{
	struct i2c_msg msg[2];
	int err;
	msg[0].addr = this_client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &addr;
	msg[1].addr = this_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = buf;
	err = i2c_transfer(this_client->adapter, msg, 2);
	if (err != 2) {
		dev_err(&this_client->dev,
				"i2c_transfer() read error: "
				"slave_addr=%02x, reg_addr=%02x, err=%d\n",
				this_client->addr, addr, err);
		return err;
	}
	return 0;
}

static void yas_usleep(int us)
{
	usleep_range(us, us + 1000);
}

static uint32_t yas_current_time(void)
{
	return jiffies_to_msecs(jiffies);
}

static int yas_enable(struct yas_state *st)
{
	if (!atomic_cmpxchg(&st->enable, 0, 1)) {
		mutex_lock(&st->lock);
		st->mag.set_enable(1);
		mutex_unlock(&st->lock);
		schedule_delayed_work(&st->work, 0);
	}
	return 0;
}

static int yas_disable(struct yas_state *st)
{
	if (atomic_cmpxchg(&st->enable, 1, 0)) {
		cancel_delayed_work_sync(&st->work);
		mutex_lock(&st->lock);
		st->mag.set_enable(0);
		mutex_unlock(&st->lock);
	}
	return 0;
}

/* Sysfs interface */
static ssize_t yas_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", YAS_MSM_NAME);
}

static ssize_t yas_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", YAS_MSM_VENDOR);
}

static ssize_t yas_version_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", YAS_MSM_VERSION);
}

static ssize_t yas_handle_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", YAS_MSM_HANDLE);
}

static ssize_t yas_type_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", YAS_MSM_TYPE);
}

static ssize_t yas_max_range_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", YAS_MSM_MAX_RANGE);
}

static ssize_t yas_resolution_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", YAS_MSM_RESOLUTION);
}

static ssize_t yas_sensor_power_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", YAS_MSM_SENSOR_POWER);
}

static ssize_t yas_min_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", YAS_MSM_MIN_DELAY);
}

static ssize_t yas_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	return sprintf(buf, "%d\n", atomic_read(&st->enable));
}

static ssize_t yas_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int enable;
	if (kstrtoint(buf, 10, &enable) < 0)
		return -EINVAL;
	if (enable)
		yas_enable(st);
	else
		yas_disable(st);
	return count;
}

static ssize_t yas_position_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int ret;
	mutex_lock(&st->lock);
	ret = st->mag.get_position();
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return sprintf(buf, "%d\n", ret);
}

static ssize_t yas_position_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int ret, position;
	sscanf(buf, "%d\n", &position);
	mutex_lock(&st->lock);
	ret = st->mag.set_position(position);
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return count;
}

static ssize_t yas_poll_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int32_t poll_delay;
	mutex_lock(&st->lock);
	poll_delay = st->poll_delay;
	mutex_unlock(&st->lock);
	return sprintf(buf, "%d\n", poll_delay);
}

static ssize_t yas_poll_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int poll_delay;
	if (kstrtoint(buf, 10, &poll_delay) < 0)
		return -EINVAL;
	if (poll_delay <= 0)
		poll_delay = 0;
	mutex_lock(&st->lock);
	if (st->mag.set_delay(poll_delay) == YAS_NO_ERROR)
		st->poll_delay = poll_delay;
	mutex_unlock(&st->lock);
	return count;
}

#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
static ssize_t yas_hard_offset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int8_t hard_offset[3];
	int ret;
	mutex_lock(&st->lock);
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533
	ret = st->mag.ext(YAS532_GET_HW_OFFSET, hard_offset);
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
	ret = st->mag.ext(YAS537_GET_HW_OFFSET, hard_offset);
#endif
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return sprintf(buf, "%d %d %d\n", hard_offset[0], hard_offset[1],
			hard_offset[2]);
}
#endif

#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533
static ssize_t yas_hard_offset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int32_t tmp[3];
	int8_t hard_offset[3];
	int ret, i;
	sscanf(buf, "%d %d %d\n", &tmp[0], &tmp[1], &tmp[2]);
	for (i = 0; i < 3; i++)
		hard_offset[i] = (int8_t)tmp[i];
	mutex_lock(&st->lock);
	ret = st->mag.ext(YAS532_SET_HW_OFFSET, hard_offset);
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return count;
}

static ssize_t yas_self_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	struct yas532_self_test_result r;
	int ret;
	mutex_lock(&st->lock);
	ret = st->mag.ext(YAS532_SELF_TEST, &r);
	mutex_unlock(&st->lock);

	return sprintf(buf, "%d %d %d %d %d %d %d %d %d %d %d\n",
			ret, r.id, r.xy1y2[0], r.xy1y2[1], r.xy1y2[2],
			r.dir, r.sx, r.sy, r.xyz[0], r.xyz[1], r.xyz[2]);
}

static ssize_t yas_self_test_noise_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int32_t xyz_raw[3];
	int ret;
	mutex_lock(&st->lock);
	ret = st->mag.ext(YAS532_SELF_TEST_NOISE, xyz_raw);
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return sprintf(buf, "%d %d %d\n", xyz_raw[0], xyz_raw[1], xyz_raw[2]);
}
#endif

#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
static ssize_t yas_self_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	struct yas537_self_test_result r;
	int ret;
	mutex_lock(&st->lock);
	ret = st->mag.ext(YAS537_SELF_TEST, &r);
	mutex_unlock(&st->lock);

	return sprintf(buf, "%d %d %d %d %d %d %d %d\n", ret, r.id, r.dir,
			r.sx, r.sy, r.xyz[0], r.xyz[1], r.xyz[2]);
}

static ssize_t yas_self_test_noise_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int32_t xyz_raw[3];
	int ret;
	mutex_lock(&st->lock);
	ret = st->mag.ext(YAS537_SELF_TEST_NOISE, xyz_raw);
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return sprintf(buf, "%d %d %d\n", xyz_raw[0], xyz_raw[1], xyz_raw[2]);
}

static ssize_t yas_mag_average_sample_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int8_t mag_average_sample;
	int ret;
	mutex_lock(&st->lock);
	ret = st->mag.ext(YAS537_GET_AVERAGE_SAMPLE, &mag_average_sample);
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return sprintf(buf, "%d\n", mag_average_sample);
}

static ssize_t yas_mag_average_sample_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int32_t tmp;
	int8_t mag_average_sample;
	int ret;
	sscanf(buf, "%d\n", &tmp);
	mag_average_sample = (int8_t)tmp;
	mutex_lock(&st->lock);
	ret = st->mag.ext(YAS537_SET_AVERAGE_SAMPLE, &mag_average_sample);
	mutex_unlock(&st->lock);
	if (ret < 0)
		return -EFAULT;
	return count;
}
#endif

static ssize_t yas_data_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct yas_state *st = i2c_get_clientdata(this_client);
	int32_t last[3], i;
	mutex_lock(&st->lock);
	for (i = 0; i < 3; i++)
		last[i] = st->compass_data[i];
	mutex_unlock(&st->lock);
	return sprintf(buf, "%d %d %d\n", last[0], last[1], last[2]);
}

static DEVICE_ATTR(name, S_IRUGO, yas_name_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO, yas_vendor_show, NULL);
static DEVICE_ATTR(version, S_IRUGO, yas_version_show, NULL);
static DEVICE_ATTR(handle, S_IRUGO, yas_handle_show, NULL);
static DEVICE_ATTR(type, S_IRUGO, yas_type_show, NULL);
static DEVICE_ATTR(max_range, S_IRUGO, yas_max_range_show, NULL);
static DEVICE_ATTR(resolution, S_IRUGO, yas_resolution_show, NULL);
static DEVICE_ATTR(sensor_power, S_IRUGO, yas_sensor_power_show, NULL);
static DEVICE_ATTR(min_delay, S_IRUGO, yas_min_delay_show, NULL);
static DEVICE_ATTR(poll_delay, S_IRUGO|S_IWUSR|S_IWGRP, yas_poll_delay_show,
		yas_poll_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR|S_IWGRP, yas_enable_show,
		yas_enable_store);
static DEVICE_ATTR(data, S_IRUGO, yas_data_show, NULL);
static DEVICE_ATTR(position, S_IRUGO|S_IWUSR, yas_position_show,
		yas_position_store);
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533
static DEVICE_ATTR(hard_offset, S_IRUGO|S_IWUSR, yas_hard_offset_show,
		yas_hard_offset_store);
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
static DEVICE_ATTR(self_test, S_IRUSR, yas_self_test_show, NULL);
static DEVICE_ATTR(self_test_noise, S_IRUSR, yas_self_test_noise_show, NULL);
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
static DEVICE_ATTR(hard_offset, S_IRUGO|S_IWUSR, yas_hard_offset_show, NULL);
static DEVICE_ATTR(mag_average_sample, S_IRUGO|S_IWUSR,
		yas_mag_average_sample_show, yas_mag_average_sample_store);
#endif

static struct attribute *yas_attributes[] = {
	&dev_attr_name.attr,
	&dev_attr_vendor.attr,
	&dev_attr_version.attr,
	&dev_attr_handle.attr,
	&dev_attr_type.attr,
	&dev_attr_max_range.attr,
	&dev_attr_resolution.attr,
	&dev_attr_sensor_power.attr,
	&dev_attr_min_delay.attr,
	&dev_attr_poll_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_data.attr,
	&dev_attr_position.attr,
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS532 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS533 \
	|| YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
	&dev_attr_hard_offset.attr,
	&dev_attr_self_test.attr,
	&dev_attr_self_test_noise.attr,
#endif
#if YAS_MAG_DRIVER == YAS_MAG_DRIVER_YAS537
	&dev_attr_mag_average_sample.attr,
#endif
	NULL
};
static struct attribute_group yas_attribute_group = {
	.attrs = yas_attributes
};

static void yas_work_func(struct work_struct *work)
{
	struct yas_state *st
		= container_of((struct delayed_work *)work,
			struct yas_state, work);
	struct yas_data mag[1];
	int32_t poll_delay;
	uint32_t time_before, time_after;
	int ret, i;

	time_before = yas_current_time();
	mutex_lock(&st->lock);
	ret = st->mag.measure(mag, 1);
	if (ret == 1) {
		for (i = 0; i < 3; i++)
			st->compass_data[i] = mag[0].xyz.v[i];
	}
	poll_delay = st->poll_delay;
	mutex_unlock(&st->lock);
	if (ret == 1) {
		/* report magnetic data in [nT] */
		input_report_abs(st->input_dev, ABS_X, mag[0].xyz.v[0]);
		input_report_abs(st->input_dev, ABS_Y, mag[0].xyz.v[1]);
		input_report_abs(st->input_dev, ABS_Z, mag[0].xyz.v[2]);
		input_sync(st->input_dev);
	}
	time_after = yas_current_time();
	poll_delay = poll_delay - (time_after - time_before);
	if (poll_delay <= 0)
		poll_delay = 1;
	schedule_delayed_work(&st->work, msecs_to_jiffies(poll_delay));
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void yas_early_suspend(struct early_suspend *h)
{
	struct yas_state *st = container_of(h, struct yas_state, sus);
	if (atomic_read(&st->enable)) {
		cancel_delayed_work_sync(&st->work);
		st->mag.set_enable(0);
	}
}


static void yas_late_resume(struct early_suspend *h)
{
	struct yas_state *st = container_of(h, struct yas_state, sus);
	if (atomic_read(&st->enable)) {
		st->mag.set_enable(1);
		schedule_delayed_work(&st->work, 0);
	}
}
#endif

static int yas_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct yas_state *st = NULL;
	struct input_dev *input_dev = NULL;
	int ret, i;
  printk(KERN_ERR "taojun_yas533\n");
	this_client = i2c;
	input_dev = input_allocate_device();
	if (input_dev == NULL) {
		ret = -ENOMEM;
		goto error_free;
	}
	st = kzalloc(sizeof(struct yas_state), GFP_KERNEL);
	if (st == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}
	i2c_set_clientdata(i2c, st);

	input_dev->name = YAS_MSM_NAME;
	input_dev->dev.parent = &i2c->dev;
	input_dev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_X, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, INT_MIN, INT_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_Z, INT_MIN, INT_MAX, 0, 0);

	input_set_drvdata(input_dev, st);
	atomic_set(&st->enable, 0);
	st->input_dev = input_dev;
	st->poll_delay = YAS_DEFAULT_SENSOR_DELAY;
	st->mag.callback.device_open = yas_device_open;
	st->mag.callback.device_close = yas_device_close;
	st->mag.callback.device_write = yas_device_write;
	st->mag.callback.device_read = yas_device_read;
	st->mag.callback.usleep = yas_usleep;
	st->mag.callback.current_time = yas_current_time;
	INIT_DELAYED_WORK(&st->work, yas_work_func);
	mutex_init(&st->lock);
#ifdef CONFIG_HAS_EARLYSUSPEND
	st->sus.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	st->sus.suspend = yas_early_suspend;
	st->sus.resume = yas_late_resume;
	register_early_suspend(&st->sus);
	printk(KERN_ERR "taojun_yasf_suspend");
#endif
	for (i = 0; i < 3; i++)
		st->compass_data[i] = 0;

	ret = input_register_device(input_dev);
	if (ret)
		goto error_free_device;

#if 1 /* FIXME */
	st->class = class_create(THIS_MODULE, "sensors_compass");
	if (IS_ERR(st->class)) {
		pr_err("couldn't create sysfs class\n");
		goto error_unregister_data;
	}
#else
	st->class = &sensors_class;
#endif

	st->dev = device_create(st->class, NULL, 0, "%s", YAS_MSM_NAME);
	if (unlikely(IS_ERR(st->dev))) {
		ret = PTR_ERR(st->dev);
		st->dev = NULL;
		goto error_destroy_class;
	}

	ret = sysfs_create_group(&st->dev->kobj, &yas_attribute_group);
	if (ret)
		goto error_unregister_device;
	ret = yas_mag_driver_init(&st->mag);
	if (ret < 0) {
		ret = -EFAULT;
		goto error_remove_sysfs;
	}
	ret = st->mag.init();
	if (ret < 0) {
		ret = -EFAULT;
		goto error_remove_sysfs;
	}
	printk(KERN_ERR "taojun_yas533_probe_suc\n");
	return 0;

error_remove_sysfs:
	sysfs_remove_group(&st->dev->kobj, &yas_attribute_group);
error_unregister_device:
	device_unregister(st->dev);
error_destroy_class:
	class_destroy(st->class);
error_unregister_data:
	input_unregister_device(input_dev);
error_free_device:
	input_free_device(input_dev);
error_free:
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&st->sus);
#endif
	kfree(st);
error_ret:
	i2c_set_clientdata(i2c, NULL);
	this_client = NULL;
	return ret;
}

static int yas_remove(struct i2c_client *i2c)
{
	struct yas_state *st = i2c_get_clientdata(i2c);
	if (st != NULL) {
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&st->sus);
#endif
		yas_disable(st);
		st->mag.term();
		sysfs_remove_group(&st->dev->kobj,
				&yas_attribute_group);
		input_unregister_device(st->input_dev);
		input_free_device(st->input_dev);
		device_unregister(st->dev);
		class_destroy(st->class);
		kfree(st);
		this_client = NULL;
	}
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int yas_suspend(struct device *dev)
{
  /*struct i2c_client *i2c = dev_get_drvdata(dev);
	struct yas_state *st = i2c_get_clientdata(i2c);
	compass_yas533_enable=atomic_read(&st->enable);
	printk(KERN_ERR "taojun_yas_suspend_CONFIG_PM_SLEEP=%d\n",compass_yas533_enable);
	if (1==compass_yas533_enable) {
		cancel_delayed_work_sync(&st->work);
		st->mag.set_enable(0);
	}*/
	return 0;
}

static int yas_resume(struct device *dev)
{
	/*struct i2c_client *i2c = dev_get_drvdata(dev);
	struct yas_state *st = i2c_get_clientdata(i2c);
	//if (atomic_read(&st->enable)) {
	 if(compass_yas533_enable){
		st->mag.set_enable(1);
		schedule_delayed_work(&st->work, 0);
	}*/
	return 0;
}

static SIMPLE_DEV_PM_OPS(yas_pm_ops, yas_suspend, yas_resume);
#define YAS_PM_OPS (&yas_pm_ops)
#else
#define YAS_PM_OPS NULL
#endif

static const struct i2c_device_id yas_id[] = {
	{YAS_MSM_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, yas_id);

static struct of_device_id yas_match_table[] = {
	{ .compatible = YAS_MSM_NAME,},
	{ },
};

static struct i2c_driver yas_driver = {
	.driver = {
		.name	= YAS_MSM_NAME,
		.owner	= THIS_MODULE,
		.pm	= YAS_PM_OPS,
		.of_match_table = yas_match_table,
	},
	.probe		= yas_probe,
	.remove		= yas_remove,
	.id_table	= yas_id,
};
static int __init yas_driver_init(void)
{
	return i2c_add_driver(&yas_driver);
}

static void __exit yas_driver_exit(void)
{
	i2c_del_driver(&yas_driver);
}

module_init(yas_driver_init);
module_exit(yas_driver_exit);

MODULE_DESCRIPTION("Yamaha Magnetometer I2C driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION("1.6.2.1020d");
