/* drivers/input/misc/hshcal001a_i2c.c
 *
 * Humidity device driver for I2C (HSHCAL001A)
 *
 * Copyright (C) 2012 ALPS ELECTRIC CO., LTD. All Rights Reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifdef ALPS_HMD_DEBUG
#define DEBUG 1
#endif

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#ifdef CONFIG_OF
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

#define HSHCAL_DRIVER_NAME		"hshcal001a"
#define HSHCAL_LOG_TAG			"[HSHCAL], "
#define HSHCAL_INPUT_DEVICE_NAME	"alps_humidity"

#define HSHCAL_DELAY(us)	usleep_range(us, us)

#define I2C_RETRIES		5

enum hshcal_sns_type {
	SNS_TYPE_NULL = 0x00,
	SNS_TYPE_HMD  = 0x01,
	SNS_TYPE_TMP  = 0x02,
	SNS_TYPE_BOTH = 0x03,
};

/* Register for hshcal */
#define HSHCAL_STR		0x0C	/* Self test response  */
#define HSHCAL_INF1		0x0D	/* More info Version   */
#define HSHCAL_INF2		0x0E	/* More info ALPS      */
#define HSHCAL_WIA		0x0F	/* Who I am            */
#define HSHCAL_HUMI1		0x10	/* Humidity output1    */
#define HSHCAL_HUMI2		0x11	/* Humidity output2    */
#define HSHCAL_TEMP1		0x12	/* Temperature output1 */
#define HSHCAL_TEMP2		0x13	/* Temperature output2 */
#define HSHCAL_STAT		0x18	/* Status              */
#define HSHCAL_CTL1		0x1B	/* Control1            */
#define HSHCAL_RDAT1		0x20	/* Reserved data1      */
#define HSHCAL_RDAT2		0x21	/* Reserved data2      */
#define HSHCAL_RDAT3		0x22	/* Reserved data3      */
#define HSHCAL_RDAT4		0x23	/* Reserved data4      */

/* Action Command Register for hshcal */
#define HSHCAL_REST		0x30	/* Reset command         */
#define HSHCAL_DET		0x31	/* Sensor detect command */
#define HSHCAL_STST		0x32	/* Self test command     */

#define HSHCAL_CHIP_ID		0x2311

/* Self-test resiter value */
#define HSHCAL_ST_REG_DEF	0x55
#define HSHCAL_ST_REG_PASS	0xAA

/* Self-test error number */
#define HSHCAL_ST_OK		0x00
#define HSHCAL_ST_ERR_I2C	0x01
#define HSHCAL_ST_ERR_INIT	0x02
#define HSHCAL_ST_ERR_1ST	0x03
#define HSHCAL_ST_ERR_2ND	0x04

#define HSHCAL_MLT_DATA_ACCESS_NUM	4
#define HSHCAL_SIG_DATA_ACCESS_NUM	2
#define HSHCAL_INITIALL_DELAY		20

struct hshcal_data {
	struct input_dev	*input;
	struct i2c_client	*i2c;
	struct delayed_work	work_data;
	struct mutex		lock;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend	early_suspend_h;
#endif
	unsigned int		delay_msec;
	bool			factive;
	int			fena;
	bool			fsuspend;
};


/*--------------------------------------------------------------------------
 * i2c read/write function
 *--------------------------------------------------------------------------*/
static int hshcal_i2c_read(struct i2c_client *i2c, u8 *rxData, int length)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr	= i2c->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxData,
		},
		{
			.addr	= i2c->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxData,
		 },
	};

	do {
		err = i2c_transfer(i2c->adapter, msgs, ARRAY_SIZE(msgs));
	} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

	if (err != ARRAY_SIZE(msgs)) {
		dev_err(&i2c->adapter->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int hshcal_i2c_write(struct i2c_client *i2c, u8 *txData, int length)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr	= i2c->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txData,
		},
	};

	do {
		err = i2c_transfer(i2c->adapter, msgs, ARRAY_SIZE(msgs));
	} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

	if (err != ARRAY_SIZE(msgs)) {
		dev_err(&i2c->adapter->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}


/*--------------------------------------------------------------------------
 * hshcal function
 *--------------------------------------------------------------------------*/
static int hshcal_get_humidity_temperature_data(struct hshcal_data *hshcal,
					int *ht)
{
	int err = -1;
	u8  sx[HSHCAL_MLT_DATA_ACCESS_NUM];

	if (hshcal->fsuspend)
		return err;
	sx[0] = HSHCAL_HUMI1;
	err = hshcal_i2c_read(hshcal->i2c, sx,
		HSHCAL_MLT_DATA_ACCESS_NUM);
	if (err < 0)
		return err;
	ht[SNS_TYPE_HMD-1] = (int) (((u16)sx[1] << 8) | (u16)sx[0]);
	ht[SNS_TYPE_TMP-1] = (int) (((u16)sx[3] << 8) | (u16)sx[2]);
	dev_dbg(&hshcal->i2c->adapter->dev,
		HSHCAL_LOG_TAG "humidity:%d, temp:%d\n",
		ht[SNS_TYPE_HMD-1], ht[SNS_TYPE_TMP-1]);

	return err;
}

static int hshcal_get_humidity_data(struct hshcal_data *hshcal,
					int *ht)
{
	int err = -1;
	u8  sx[HSHCAL_SIG_DATA_ACCESS_NUM];

	if (hshcal->fsuspend)
		return err;
	sx[0] = HSHCAL_HUMI1;
	err = hshcal_i2c_read(hshcal->i2c, sx,
		HSHCAL_SIG_DATA_ACCESS_NUM);
	if (err < 0)
		return err;
	ht[SNS_TYPE_HMD-1] = (int) (((u16)sx[1] << 8) | (u16)sx[0]);
	dev_dbg(&hshcal->i2c->adapter->dev,
		HSHCAL_LOG_TAG "humidity:%d\n",
		ht[SNS_TYPE_HMD-1]);

	return err;
}

static int hshcal_get_temperature_data(struct hshcal_data *hshcal,
					int *ht)
{
	int err = -1;
	u8  sx[HSHCAL_SIG_DATA_ACCESS_NUM];

	if (hshcal->fsuspend)
		return err;
	sx[0] = HSHCAL_TEMP1;
	err = hshcal_i2c_read(hshcal->i2c, sx,
		HSHCAL_SIG_DATA_ACCESS_NUM);
	if (err < 0)
		return err;
	ht[SNS_TYPE_TMP-1] = (int) (((u16)sx[1] << 8) | (u16)sx[0]);
	dev_dbg(&hshcal->i2c->adapter->dev,
		HSHCAL_LOG_TAG "temp:%d\n",
		ht[SNS_TYPE_TMP-1]);

	return err;
}

static int hshcal_force_setup(struct hshcal_data *hshcal)
{
	u8 buf = HSHCAL_DET;
	return hshcal_i2c_write(hshcal->i2c, &buf, 1);
}

static void hshcal_measure_start(struct hshcal_data *hshcal)
{
	dev_dbg(&hshcal->i2c->adapter->dev,
		HSHCAL_LOG_TAG "%s\n", __func__);

	if (!hshcal->factive) {
		hshcal_force_setup(hshcal);
		schedule_delayed_work(&hshcal->work_data,
			msecs_to_jiffies(hshcal->delay_msec));
	}
	if (!hshcal->fsuspend)
		hshcal->factive = true;
}

static void hshcal_measure_stop(struct hshcal_data *hshcal)
{
	dev_dbg(&hshcal->i2c->adapter->dev,
		HSHCAL_LOG_TAG "%s\n", __func__);

	if (!hshcal->fsuspend)
		hshcal->factive = false;
	cancel_delayed_work(&hshcal->work_data);
}

static void hshcal_get_hardware_data(
			struct hshcal_data *hshcal, int *ht)
{
	hshcal_force_setup(hshcal);
	HSHCAL_DELAY(15000);
	hshcal_get_humidity_temperature_data(hshcal, ht);
}

static int hshcal_self_test(struct hshcal_data *hshcal)
{
	u8 sx;

	/* Get inital value of self-test register  */
	sx = HSHCAL_STR;
	hshcal_i2c_read(hshcal->i2c, &sx, 1);
	sx = HSHCAL_STR;
	if (hshcal_i2c_read(hshcal->i2c, &sx, 1))
		return HSHCAL_ST_ERR_I2C;
	dev_dbg(&hshcal->i2c->adapter->dev,
		HSHCAL_LOG_TAG "STR reg. initial value, %02X\n", sx);
	if (sx != HSHCAL_ST_REG_DEF) {
		dev_err(&hshcal->i2c->adapter->dev, HSHCAL_LOG_TAG
			"Err: Initial value of STR reg. is %02X\n", sx);
		return HSHCAL_ST_ERR_INIT;
	}

	/* do self-test */
	sx = HSHCAL_STST;
	if (hshcal_i2c_write(hshcal->i2c, &sx, 1))
		return HSHCAL_ST_ERR_I2C;
	HSHCAL_DELAY(3000);

	/* Get 1st value of self-test register  */
	sx = HSHCAL_STR;
	if (hshcal_i2c_read(hshcal->i2c, &sx, 1))
		return HSHCAL_ST_ERR_I2C;
	dev_dbg(&hshcal->i2c->adapter->dev,
		HSHCAL_LOG_TAG "STR reg. 1st value, %02X\n", sx);
	if (sx != HSHCAL_ST_REG_PASS) {
		dev_err(&hshcal->i2c->adapter->dev, HSHCAL_LOG_TAG
			"Err: 1st value of STR reg. is %02X\n", sx);
		return HSHCAL_ST_ERR_1ST;
	}
	HSHCAL_DELAY(3000);

	/* Get 2nd value of self-test-A register  */
	sx = HSHCAL_STR;
	if (hshcal_i2c_read(hshcal->i2c, &sx, 1))
		return HSHCAL_ST_ERR_I2C;
	dev_dbg(&hshcal->i2c->adapter->dev,
		HSHCAL_LOG_TAG "STR reg. 2nd value, %02X\n", sx);
	if (sx != HSHCAL_ST_REG_DEF) {
		dev_err(&hshcal->i2c->adapter->dev, HSHCAL_LOG_TAG
			"Err: 2nd value of STR reg. is %02X\n", sx);
		return HSHCAL_ST_ERR_2ND;
	}

	return HSHCAL_ST_OK;
}

static int hshcal_register_init(struct hshcal_data *hshcal)
{
	int ht[2], ret = 0;
	u8  buf[2];
	u16 chip_info;

	dev_dbg(&hshcal->i2c->adapter->dev,
		HSHCAL_LOG_TAG "%s\n", __func__);

	buf[0] = HSHCAL_INF1;
	ret = hshcal_i2c_read(hshcal->i2c, buf, 2);
	if (ret < 0)
		return ret;
	chip_info = (u16)((buf[1]<<8) | buf[0]);
	dev_dbg(&hshcal->i2c->adapter->dev,
		HSHCAL_LOG_TAG "chip_info, 0x%04X\n", chip_info);
	if (chip_info != HSHCAL_CHIP_ID) {
		dev_err(&hshcal->i2c->adapter->dev, HSHCAL_LOG_TAG
			"chipID error(0x%04X).\n", chip_info);
		return -1;
	}

	mutex_lock(&hshcal->lock);
	buf[0] = HSHCAL_REST;
	hshcal_i2c_write(hshcal->i2c, buf, 1);
	HSHCAL_DELAY(15000);

	buf[0] = HSHCAL_CTL1;
	buf[1] = 0x72;
	hshcal_i2c_write(hshcal->i2c, buf, 2);
	HSHCAL_DELAY(15000);

	hshcal_get_hardware_data(hshcal, ht);
#if 0
	dev_info(&hshcal->i2c->adapter->dev,
		HSHCAL_LOG_TAG "humidity:%d, temp:%d\n",
		ht[SNS_TYPE_HMD], ht[SNS_TYPE_TMP]);
#endif
	mutex_unlock(&hshcal->lock);

	return 0;
}


/*--------------------------------------------------------------------------
 * sysfs
 *--------------------------------------------------------------------------*/
static ssize_t hshcal_enableH_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct hshcal_data *hshcal = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", (hshcal->fena & SNS_TYPE_HMD) ? 1 : 0);
}

static ssize_t hshcal_enableH_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t size)
{
	struct hshcal_data *hshcal = dev_get_drvdata(dev);
	int new_value;

	if (sysfs_streq(buf, "1"))
		new_value = 1;
	else if (sysfs_streq(buf, "0"))
		new_value = 0;
	else {
		dev_err(&hshcal->i2c->adapter->dev,
			HSHCAL_LOG_TAG "%s: invalid value %d\n",
			__func__, *buf);
		return -EINVAL;
	}

	dev_dbg(&hshcal->i2c->adapter->dev,
		HSHCAL_LOG_TAG "%s, Humidity enable = %d\n",
		__func__, new_value);

	mutex_lock(&hshcal->lock);
	if (new_value) {
		hshcal->fena |= SNS_TYPE_HMD;
		hshcal_measure_start(hshcal);
	} else {
		hshcal->fena &= ~(SNS_TYPE_HMD);
		hshcal_measure_stop(hshcal);
	}
	mutex_unlock(&hshcal->lock);

	return size;
}

static ssize_t hshcal_enableT_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct hshcal_data *hshcal = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", (hshcal->fena & SNS_TYPE_TMP) ? 1 : 0);
}

static ssize_t hshcal_enableT_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t size)
{
	struct hshcal_data *hshcal = dev_get_drvdata(dev);
	int new_value;

	if (sysfs_streq(buf, "1"))
		new_value = 1;
	else if (sysfs_streq(buf, "0"))
		new_value = 0;
	else {
		dev_err(&hshcal->i2c->adapter->dev,
			HSHCAL_LOG_TAG "%s: invalid value %d\n",
			__func__, *buf);
		return -EINVAL;
	}

	dev_dbg(&hshcal->i2c->adapter->dev,
		HSHCAL_LOG_TAG "%s, Temperature enable = %d\n",
		__func__, new_value);

	mutex_lock(&hshcal->lock);
	if (new_value) {
		hshcal->fena |= SNS_TYPE_TMP;
		hshcal_measure_start(hshcal);
	} else {
		hshcal->fena &= ~(SNS_TYPE_TMP);
		hshcal_measure_stop(hshcal);
	}
	mutex_unlock(&hshcal->lock);

	return size;
}

static ssize_t hshcal_delay_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct hshcal_data *hshcal = dev_get_drvdata(dev);
	return sprintf(buf, "%d\n", hshcal->delay_msec);
}

static ssize_t hshcal_delay_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t size)
{
	int err;
	long new_delay;
	struct hshcal_data *hshcal = dev_get_drvdata(dev);

	err = strict_strtol(buf, 10, &new_delay);
	if (err < 0)
		return err;

	if (new_delay < 20)
		new_delay = 20;
	else if (new_delay > 200)
		new_delay = 200;

	mutex_lock(&hshcal->lock);
	hshcal->delay_msec = (int)new_delay;
	mutex_unlock(&hshcal->lock);

	dev_dbg(&hshcal->i2c->adapter->dev,
		HSHCAL_LOG_TAG "%s, rate = %d (msec)\n",
		__func__, hshcal->delay_msec);

	return size;
}

static ssize_t hshcal_self_test_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret = -1;
	struct hshcal_data *hshcal = dev_get_drvdata(dev);

	dev_dbg(&hshcal->i2c->adapter->dev,
		HSHCAL_LOG_TAG "%s\n", __func__);


	if (!hshcal->factive) {
		mutex_lock(&hshcal->lock);
		ret = hshcal_self_test(hshcal);
		mutex_unlock(&hshcal->lock);
		dev_dbg(&hshcal->i2c->adapter->dev,
			HSHCAL_LOG_TAG "Self test result : %d\n", ret);
	} else
		dev_err(&hshcal->i2c->adapter->dev,
			HSHCAL_LOG_TAG "Please turn off sensor\n");

	return sprintf(buf, "%d\n", ret);
}

static ssize_t hshcal_get_hw_data_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ht[2];
	struct hshcal_data *hshcal = dev_get_drvdata(dev);

	if (!hshcal->factive) {
		mutex_lock(&hshcal->lock);
		hshcal_get_hardware_data(hshcal, ht);
		mutex_unlock(&hshcal->lock);
		dev_dbg(&hshcal->i2c->adapter->dev,
			HSHCAL_LOG_TAG "%s: hmd = %d, tmp = %d\n",
			__func__, ht[0], ht[1]);
	} else
		dev_err(&hshcal->i2c->adapter->dev,
			HSHCAL_LOG_TAG "Please turn off sensor\n");

	return sprintf(buf, "%d,%d\n", ht[0], ht[1]);
}

static struct device_attribute attributes[] = {
	__ATTR(enableH, S_IWUGO | S_IRUGO,
		hshcal_enableH_show, hshcal_enableH_store),
	__ATTR(enableT, S_IWUGO | S_IRUGO,
		hshcal_enableT_show, hshcal_enableT_store),
	__ATTR(delay, S_IWUGO | S_IRUGO,
		hshcal_delay_show, hshcal_delay_store),
	__ATTR(self_test,  S_IWUGO | S_IRUGO,
		hshcal_self_test_show, NULL),
	__ATTR(get_hw_data,  S_IWUGO | S_IRUGO,
		hshcal_get_hw_data_show, NULL)
};

static int hshcal_create_sysfs(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto out_sysfs;
	return 0;

out_sysfs:
	for (; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "Unable to create interface\n");
	return -EIO;
}

static void hshcal_remove_sysfs(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}


/*--------------------------------------------------------------------------
 * suspend/resume function
 *--------------------------------------------------------------------------*/
static int hshcal_suspend(struct device *dev)
{
	struct i2c_client *client = NULL;
	struct hshcal_data *hshcal = NULL;
	pr_info("%s\n", __func__);

	if (NULL == dev) {
		pr_err("%s: dev is NULL\n", __func__);
		return 0;
	}

	client = to_i2c_client(dev);
	hshcal = i2c_get_clientdata(client);
	if (NULL == hshcal) {
		pr_err("%s: dev is NULL\n", __func__);
		return 0;
	}

	mutex_lock(&hshcal->lock);
	hshcal->fsuspend = true;
	hshcal_measure_stop(hshcal);
	mutex_unlock(&hshcal->lock);

	return 0;
}

static int hshcal_resume(struct device *dev)
{
	struct i2c_client *client = NULL;
	struct hshcal_data *hshcal = NULL;
	pr_info("%s\n", __func__);

	if (NULL == dev) {
		pr_err("%s: dev is NULL\n", __func__);
		return 0;
	}

	client = to_i2c_client(dev);
	hshcal = i2c_get_clientdata(client);
	if (NULL == hshcal) {
		pr_err("%s: dev is NULL\n", __func__);
		return 0;
	}

	mutex_lock(&hshcal->lock);
	if (hshcal->factive)
		hshcal_measure_start(hshcal);
	hshcal->fsuspend = false;
	mutex_unlock(&hshcal->lock);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void hshcal_early_suspend(struct early_suspend *handler)
{
	struct hshcal_data *hshcal = container_of(handler,
		struct hshcal_data, early_suspend_h);
	hshcal_suspend(&hshcal->i2c->dev);
}

static void hshcal_early_resume(struct early_suspend *handler)
{
	struct hshcal_data *hshcal = container_of(handler,
		struct hshcal_data, early_suspend_h);
	hshcal_resume(&hshcal->i2c->dev);
}
#endif


/*--------------------------------------------------------------------------
 * work function
 *--------------------------------------------------------------------------*/
static void hshcal_polling(struct work_struct *work)
{
	int ht[] = {0, 0}, rc = -1;
	struct hshcal_data *hshcal = container_of(work,
		struct hshcal_data, work_data.work);

	mutex_lock(&hshcal->lock);
	if (hshcal->factive) {
		if (hshcal->fena == SNS_TYPE_BOTH)
			rc = hshcal_get_humidity_temperature_data(hshcal, ht);
		else if (hshcal->fena == SNS_TYPE_HMD)
			rc = hshcal_get_humidity_data(hshcal, ht);
		else if (hshcal->fena == SNS_TYPE_TMP)
			rc = hshcal_get_temperature_data(hshcal, ht);
		if (rc == 0) {
			if (hshcal->fena & SNS_TYPE_HMD)
				input_event(hshcal->input,
					EV_REL, REL_X, ht[0]);
			if (hshcal->fena & SNS_TYPE_TMP)
				input_event(hshcal->input,
					EV_REL, REL_Y, ht[1]);
			input_sync(hshcal->input);
		}
		hshcal_force_setup(hshcal);
		schedule_delayed_work(&hshcal->work_data,
			msecs_to_jiffies(hshcal->delay_msec));
	}
	mutex_unlock(&hshcal->lock);
}


/*--------------------------------------------------------------------------
 * i2c device
 *--------------------------------------------------------------------------*/
static int hshcal_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int rc;
	struct hshcal_data *hshcal;

#ifdef CONFIG_LPM_MODE
    extern unsigned int poweroff_charging;
    extern unsigned int recovery_mode;
	if (1 == poweroff_charging || 1 == recovery_mode) {
        printk(KERN_ERR"%s: probe exit, lpm=%d recovery=%d\n", __func__, poweroff_charging, recovery_mode);
		return -ENODEV;
	}
#endif

	dev_dbg(&client->adapter->dev,
		HSHCAL_LOG_TAG "%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->adapter->dev, "client not i2c capable\n");
		rc = -ENODEV;
		goto out_region;
	}

	hshcal = kzalloc(sizeof(struct hshcal_data), GFP_KERNEL);
	if (!hshcal) {
		dev_err(&client->adapter->dev,
			"failed to allocate memory for module data\n");
		rc = -ENOMEM;
		goto out_region;
	}
	hshcal->i2c = client;
	i2c_set_clientdata(client, hshcal);

	mutex_init(&hshcal->lock);

	hshcal->delay_msec = HSHCAL_INITIALL_DELAY;

	rc = hshcal_register_init(hshcal);
	if (rc) {
		rc = -ENOMEM;
		dev_err(&client->adapter->dev, "hshcal_register_init\n");
		goto out_kzalloc;
	}
	dev_dbg(&client->adapter->dev,
		"initialize %s sensor\n", HSHCAL_DRIVER_NAME);

	hshcal->input = input_allocate_device();
	if (!hshcal->input) {
		rc = -ENOMEM;
		dev_err(&client->adapter->dev, "input_allocate_device\n");
		goto out_kzalloc;
	}
	input_set_drvdata(hshcal->input, hshcal);
	dev_dbg(&client->adapter->dev, "input_allocate_device\n");

	hshcal->input->name		= HSHCAL_INPUT_DEVICE_NAME;
	hshcal->input->id.bustype	= BUS_I2C;
	hshcal->input->evbit[0]		= BIT_MASK(EV_REL);
	input_set_capability(hshcal->input, EV_REL, REL_X);
	input_set_capability(hshcal->input, EV_REL, REL_Y);

	rc = input_register_device(hshcal->input);
	if (rc) {
		rc = -ENOMEM;
		dev_err(&client->adapter->dev, "input_register_device\n");
		goto out_idev_allc;
	}
	dev_dbg(&client->adapter->dev, "input_register_device\n");

	INIT_DELAYED_WORK(&hshcal->work_data, hshcal_polling);

	rc = hshcal_create_sysfs(&hshcal->input->dev);
	if (rc) {
		rc = -ENOMEM;
		dev_err(&client->adapter->dev, "hshcal_create_sysfs\n");
		goto out_idev_reg;
	}
	dev_dbg(&client->adapter->dev, "hshcal_create_sysfs\n");

#ifdef CONFIG_HAS_EARLYSUSPEND
	hshcal->early_suspend_h.suspend = hshcal_early_suspend;
	hshcal->early_suspend_h.resume  = hshcal_early_resume;
	register_early_suspend(&hshcal->early_suspend_h);
	dev_dbg(&client->adapter->dev, "register_early_suspend\n");
#endif

	hshcal->factive = false;
	hshcal->fena = SNS_TYPE_NULL;
	hshcal->fsuspend = false;
	dev_info(&client->adapter->dev,
		HSHCAL_LOG_TAG "detected %s humidity sensor\n",
		HSHCAL_DRIVER_NAME);

	return 0;

out_idev_reg:
	input_unregister_device(hshcal->input);
out_idev_allc:
	input_free_device(hshcal->input);
out_kzalloc:
	i2c_set_clientdata(client, NULL);
	kfree(hshcal);
out_region:

	return rc;
}

static int hshcal_remove(struct i2c_client *client)
{
	struct hshcal_data *hshcal = i2c_get_clientdata(client);
	if (NULL == hshcal) {
		pr_err("%s: clientdata is NULL\n", __func__);
		return 0;
	}

	dev_dbg(&client->adapter->dev, "%s\n", __func__);
	hshcal_measure_stop(hshcal);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&hshcal->early_suspend_h);
#endif
	hshcal_remove_sysfs(&hshcal->input->dev);
	input_unregister_device(hshcal->input);
	input_free_device(hshcal->input);
	kfree(hshcal);
	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id hshcal_match_table[] = {
	{ .compatible = HSHCAL_DRIVER_NAME,},
	{ },
};
#endif

static const struct i2c_device_id hshcal_id[] = {
	{ HSHCAL_DRIVER_NAME, 0 },
	{ }
};

#if (defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND))
static SIMPLE_DEV_PM_OPS(hshcal_pm_ops, hshcal_suspend, hshcal_resume);
#endif

static struct i2c_driver hshcal_driver = {
	.probe		= hshcal_probe,
	.remove		= hshcal_remove,
	.id_table	= hshcal_id,
	.driver		= {
		.name	= HSHCAL_DRIVER_NAME,
	#ifdef CONFIG_OF
		.of_match_table = hshcal_match_table,
    #endif
	#if (defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND))
		.pm	= &hshcal_pm_ops,
	#endif
	},
};


static int __init hshcal_init(void)
{
	pr_debug(HSHCAL_LOG_TAG "%s\n", __func__);
	return i2c_add_driver(&hshcal_driver);
}

static void __exit hshcal_exit(void)
{
	pr_debug(HSHCAL_LOG_TAG "%s\n", __func__);
	i2c_del_driver(&hshcal_driver);
}

module_init(hshcal_init);
module_exit(hshcal_exit);

MODULE_DESCRIPTION("Alps Humidity Input Device");
MODULE_AUTHOR("ALPS ELECTRIC CO., LTD.");
MODULE_LICENSE("GPL v2");
