/* drivers/misc/mpu6880.c - mpu6880  driver
 *
 * Copyright (C) 2003-2015 Invensense Corporation.
 * Author: Mingyang Li <myli@invensense.com>
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


#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/interrupt.h>


#include <linux/pm.h>
#include <linux/pm_runtime.h>


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/platform_device.h>
#include <linux/input-polldev.h>

#include <linux/sensors/mpu6880.h>
#include <linux/sensors/sensparams.h>

#define MPU_DEVICE_NAME "mpu6880"
#define MPU6880_POLL_INTERVAL                  200
#define MPU6880_POLL_INTERVAL_MIN       1
#define MPU6880_POLL_INTERVAL_MAX   5000
#define MPU6880_CAPTURE_TIMES           100
#define MPU6880_ACCEL_SENSITIVE     -16384
#define CAL_CONVERT 16384
#define CAL_FUZZ    9800    //5000 //200//20
#define GYRO_CAL_FUZZ  8000

int inv_debug = 0;
#define inv_info(fmt, arg...)   \
    do{  \
        if (inv_debug) \
            printk("%s "fmt, MPU_DEVICE_NAME, ##arg);  \
        }while(0)

static int mpu6880_read_accel_xyz(struct mpu6880_device *dev);
static int mpu6880_read_gyro_xyz(struct mpu6880_device *dev);


static int mpu6880_read(struct i2c_client *client, unsigned char addr)
{
    int ret = 0;

    ret = i2c_smbus_read_byte_data(client, addr);
    if (ret < 0)
        printk("failed to read register 0x%02x, error %d\n",
            addr, ret);

    return ret;
}

static int mpu6880_write(struct i2c_client *client, unsigned char addr, unsigned char data)
{
    int ret = 0;

    ret = i2c_smbus_write_byte_data(client, addr, data);
    if (ret < 0) {
        printk( "failed to write to register 0x%02x, error %d\n",
            addr, ret);
        return ret;
    }

    return 0;
}

static int mpu6880_read_pwrm_regster(struct mpu6880_device *dev, unsigned char *pwrm1,
    unsigned char *pwrm2)
{
    unsigned char data = 0;

    data = mpu6880_read(dev->client, MPU6880_PWR_MGMT_1);
    if (data  < 0)
    {
        printk(KERN_ERR"Mpu6880 read power register 1 error\n");
        return data;
    }
    msleep(10);

    *pwrm1 = data;

    data = mpu6880_read(dev->client, MPU6880_PWR_MGMT_2);
    if (data  < 0)
    {
        printk(KERN_ERR"Mpu6880 read power register 2 error\n");
        return data;
    }
    msleep(10);

    *pwrm2 = data;

    return 0;
}


static int mpu6880_write_pwrm_regster(struct mpu6880_device *dev, unsigned char pwrm1,
    unsigned char pwrm2)
{
    int res = 0;

    res = mpu6880_write(dev->client, MPU6880_PWR_MGMT_2, pwrm2);
    if (res  < 0)
    {
        printk(KERN_ERR"Mpu6880 write power register 2 error\n");
        return res;
    }

    msleep(20);

    res = mpu6880_write(dev->client, MPU6880_PWR_MGMT_1, pwrm1);
    if (res  < 0)
    {
        printk(KERN_ERR"Mpu6880 write power register 1 error\n");
        return res;
    }

    msleep(20);

    return 0;
}


static void mpu6880_accel_enable(struct mpu6880_device *dev)
{
    struct mpu6880_device *mpu = dev;
    unsigned char pwrm1 = 0;
    unsigned char pwrm2 = 0;
    unsigned char data = 0;
    int res = 0;

    mutex_lock(&mpu->mutex);
    res = mpu6880_read_pwrm_regster(mpu, &pwrm1, &pwrm2);
    if (res < 0)
    {
        printk(KERN_ERR"%s: read PWR register error\n", __func__);
        mutex_unlock(&mpu->mutex);
        return ;
    }

    if (pwrm2 & MPU6880_PWRM2_ACCEL_DISABLE)
    {
        data = pwrm2 & (~MPU6880_PWRM2_ACCEL_DISABLE);
        printk("%s accelerator sensor all axis disabled(old: 0x%x),  begin to enable(new: 0x%x )\n", __func__, pwrm2, data);

        pwrm2 = data;
    }

    if (pwrm1 & MPU6880_PWRM1_SLEEP)
    {
        data = pwrm1 & (~MPU6880_PWRM1_SLEEP);
        printk("%s device is in sleep mode(old: 0x%x),  begin to enable(new: 0x%x )\n", __func__, pwrm1, data);

        pwrm1 = data;
    }

    res = mpu6880_write_pwrm_regster(mpu, pwrm1, pwrm2);
    if (res < 0)
    {
        printk(KERN_ERR"%s: write PWR register error\n", __func__);
        mutex_unlock(&mpu->mutex);
        return ;
    }

    mutex_unlock(&mpu->mutex);
}


static void mpu6880_accel_disable(struct mpu6880_device *dev)
{
    struct mpu6880_device *mpu = dev;
    unsigned char pwrm1 = 0;
    unsigned char pwrm2 = 0;
    unsigned char data = 0;
    int res = 0;

    mutex_lock(&mpu->mutex);
    res = mpu6880_read_pwrm_regster(mpu, &pwrm1, &pwrm2);
    if (res < 0)
    {
        printk(KERN_ERR"%s: read PWR register error\n", __func__);
        mutex_unlock(&mpu->mutex);
        return;
    }

    if (!(pwrm2 & MPU6880_PWRM2_ACCEL_DISABLE)  )
    {
        data = pwrm2 |MPU6880_PWRM2_ACCEL_DISABLE;
        printk("%s accelerator sensor all axis enable(old: 0x%x),  begin to disable(new: 0x%x )\n", __func__, pwrm2, data);

        pwrm2 = data;
    }


    if (mpu->gyro_status == 0)
    {
        data = pwrm1 |MPU6880_PWRM1_SLEEP;
        printk("%s gyro  has in sleep mode(old: 0x%x),  device  begin to sleep(new: 0x%x )\n", __func__, pwrm1, data);

        pwrm1 = data;
    }

    res = mpu6880_write_pwrm_regster(mpu, pwrm1, pwrm2);
    if (res < 0)
    {
        printk(KERN_ERR"%s: write PWR register error\n", __func__);
        mutex_unlock(&mpu->mutex);
        return;
    }

    mutex_unlock(&mpu->mutex);
}

static void mpu6880_gyro_enable(struct mpu6880_device *dev)
{
    struct mpu6880_device *mpu = dev;
    unsigned char pwrm1 = 0;
    unsigned char pwrm2 = 0;
    unsigned char data = 0;
    int res = 0;

    mutex_lock(&mpu->mutex);
    res = mpu6880_read_pwrm_regster(mpu, &pwrm1, &pwrm2);
    if (res < 0)
    {
        printk(KERN_ERR"%s: read PWR register error\n", __func__);
        mutex_unlock(&mpu->mutex);
        return ;
    }

    if (pwrm2 & MPU6880_PWRM2_GYRO_DISABLE)
    {
        data = pwrm2 & (~MPU6880_PWRM2_GYRO_DISABLE);
        printk("%s accelerator sensor all axis disabled(old: 0x%x),  begin to enable(new: 0x%x )\n", __func__, pwrm2, data);

        pwrm2 = data;
    }

    if (pwrm1 & MPU6880_PWRM1_SLEEP)
    {
        data = pwrm1 & (~MPU6880_PWRM1_SLEEP);
        printk("%s device is in sleep mode(old: 0x%x),  begin to enable(new: 0x%x )\n", __func__, pwrm1, data);

        pwrm1 = data;
    }

    res = mpu6880_write_pwrm_regster(mpu, pwrm1, pwrm2);
    if (res < 0)
    {
        printk(KERN_ERR"%s: write PWR register error\n", __func__);

        mutex_unlock(&mpu->mutex);
        return ;
    }

    mutex_unlock(&mpu->mutex);
}


static void mpu6880_gyro_disable(struct mpu6880_device *dev)
{
    struct mpu6880_device *mpu = dev;
    unsigned char pwrm1 = 0;
    unsigned char pwrm2 = 0;
    unsigned char data = 0;
    int res = 0;

    mutex_lock(&mpu->mutex);
    res = mpu6880_read_pwrm_regster(mpu, &pwrm1, &pwrm2);
    if (res < 0)
    {
        printk(KERN_ERR"%s: read PWR register error\n", __func__);
        mutex_unlock(&mpu->mutex);
        return;
    }

    if (!(pwrm2 & MPU6880_PWRM2_GYRO_DISABLE)  )
    {
        data = pwrm2 |MPU6880_PWRM2_GYRO_DISABLE;
        printk("%s gyro sensor all axis enable(old: 0x%x),  begin to disable(new: 0x%x )\n", __func__, pwrm2, data);

        pwrm2 = data;
    }


    if (mpu->accel_status == 0)
    {
        data = pwrm1 |MPU6880_PWRM1_SLEEP;
        printk("%s gyro  has in sleep mode(old: 0x%x),  device  begin to sleep(new: 0x%x )\n", __func__, pwrm1, data);

        pwrm1 = data;
    }

    res = mpu6880_write_pwrm_regster(mpu, pwrm1, pwrm2);
    if (res < 0)
    {
        printk(KERN_ERR"%s: write PWR register error\n", __func__);
        mutex_unlock(&mpu->mutex);
        return;
    }

    mutex_unlock(&mpu->mutex);
}

static void mpu6880_reset_delay(struct mpu6880_device *dev)
{
    struct mpu6880_device *mpu =dev;

    mutex_lock(&mpu->mutex);
    mpu->poll_time = (mpu->accel_poll) < (mpu->gyro_poll) ? (mpu->accel_poll) : (mpu->gyro_poll);
    mpu->input_dev->poll_interval = mpu->poll_time;
    mutex_unlock(&mpu->mutex);
}

static int mpu6880_accel_gyro_calibration(struct mpu6880_device *dev)
{
    int i = 0;
    int err = 0;
    int acc_ret=0;
    int gyro_ret=0;
    long int sum_accel[3] = {0, 0, 0};
    long int sum_gyro[3] = {0, 0, 0};
  long int gsensor_xavg = 0,gsensor_yavg = 0,gsensor_zavg = 0;
  long int gyro_xavg = 0,gyro_yavg = 0,gyro_zavg = 0;

    for (i = 0; i < MPU6880_CAPTURE_TIMES; i++)
    {
        err = mpu6880_read_accel_xyz(dev);
        if (err < 0)
            printk(KERN_ERR"in %s read accel data error\n", __func__);

        err = mpu6880_read_gyro_xyz(dev);
        if (err < 0)
            printk(KERN_ERR"in %s read gyro data error\n", __func__);

        sum_accel[0] += dev->accel_data[0];
        sum_accel[1] += dev->accel_data[1];
        sum_accel[2] += dev->accel_data[2];

        sum_gyro[0] += dev->gyro_data[0];
        sum_gyro[1] += dev->gyro_data[1];
        sum_gyro[2] += dev->gyro_data[2];

        printk("%d times, read accel data is %d %d %d, gyro data is %d %d %d \n",
            i,  dev->accel_data[0],  dev->accel_data[1],  dev->accel_data[2],
            dev->gyro_data[0], dev->gyro_data[1], dev->gyro_data[2]);
    }

    /*dev->accel_offset[0] = sum_accel[0]/MPU6880_CAPTURE_TIMES;
    dev->accel_offset[1] = sum_accel[1]/MPU6880_CAPTURE_TIMES;
    dev->accel_offset[2] = MPU6880_ACCEL_SENSITIVE - sum_accel[2]/MPU6880_CAPTURE_TIMES;*/
    gsensor_xavg = sum_accel[0]/MPU6880_CAPTURE_TIMES;
    gsensor_yavg = sum_accel[1]/MPU6880_CAPTURE_TIMES;
    gsensor_zavg = sum_accel[2]/MPU6880_CAPTURE_TIMES;

    printk(KERN_INFO "%s: gsensor_xavg = %ld, gsensor_yavg = %ld, gsensor_zavg = %ld\n", __func__, gsensor_xavg, gsensor_yavg, gsensor_zavg);

  gyro_xavg = sum_gyro[0]/MPU6880_CAPTURE_TIMES;
    gyro_yavg = sum_gyro[1]/MPU6880_CAPTURE_TIMES;
    gyro_zavg = sum_gyro[2]/MPU6880_CAPTURE_TIMES;

    if(((gyro_xavg > -GYRO_CAL_FUZZ)&&(gyro_xavg < GYRO_CAL_FUZZ))&&((gyro_yavg > -GYRO_CAL_FUZZ)&&(gyro_yavg < GYRO_CAL_FUZZ))&&
          ((gyro_zavg > -GYRO_CAL_FUZZ)&&(gyro_zavg < GYRO_CAL_FUZZ)))
  {
            dev->gyro_offset[0] = gyro_xavg;
            dev->gyro_offset[1] = gyro_yavg;
            dev->gyro_offset[2] = gyro_zavg;
            gyro_ret = 0;
    } else {
            printk(KERN_ERR "mpu6880 gyro calibration fail\n");
            gyro_ret = -1;
  }
  if(((gsensor_xavg > -CAL_FUZZ)&&(gsensor_xavg < CAL_FUZZ))&&((gsensor_yavg > -CAL_FUZZ)&&(gsensor_yavg < CAL_FUZZ))&&
          ((gsensor_zavg > (CAL_CONVERT - CAL_FUZZ))&&(gsensor_zavg < (CAL_CONVERT + CAL_FUZZ))))
  {
          dev->accel_offset[0] = gsensor_xavg;
          dev->accel_offset[1] = gsensor_yavg;
          dev->accel_offset[2] = gsensor_zavg - CAL_CONVERT;
          //KMSGINF(&acceld->client->dev, "%s(9.8): success!\n", __func__);
          acc_ret = 0;
  } else if (((gsensor_xavg > -CAL_FUZZ)&&(gsensor_xavg < CAL_FUZZ))&&((gsensor_yavg > -CAL_FUZZ)&&(gsensor_yavg < CAL_FUZZ))&&
          ((gsensor_zavg > (-CAL_CONVERT - CAL_FUZZ))&&(gsensor_zavg < (-CAL_CONVERT + CAL_FUZZ))))
  {
          dev->accel_offset[0] = gsensor_xavg;
          dev->accel_offset[1] = gsensor_yavg;
          dev->accel_offset[2] = gsensor_zavg + CAL_CONVERT;
          //KMSGINF(&acceld->client->dev, "%s(-9.8): success!\n", __func__);
          acc_ret = 0;
  } else {
          //KMSGINF(&acceld->client->dev, "%s: failed!\n", __func__);
          acc_ret = -1;
          printk(KERN_ERR "mpu6880 gsensor calibration fail\n");
  }
  dev->acc_key=acc_ret ? 2 : 1;
  dev->gyro_key=gyro_ret ? 2 : 1;
    printk(" accel offset is %d %d %d, gyro offset is %d %d %d,acc_key=%d,gyro_key=%d\n",
        dev->accel_offset[0],  dev->accel_offset[1],  dev->accel_offset[2],
        dev->gyro_offset[0], dev->gyro_offset[1], dev->gyro_offset[2],dev->acc_key,dev->gyro_key);
  return 0;

}

static ssize_t mpu6880_accel_enable_show(struct device *dev, struct device_attribute *attr,
                 char *buf)
{
    struct input_polled_dev *polldev = dev_get_drvdata(dev);
    struct mpu6880_device *mpu = (struct mpu6880_device *)polldev->private;

    inv_info("%s : called\n", __func__);
    return sprintf(buf, "%d\n",  mpu->accel_status);
}

static ssize_t mpu6880_accel_enable_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    struct input_polled_dev *polldev = dev_get_drvdata(dev);
    struct mpu6880_device *mpu = (struct mpu6880_device *)polldev->private;

    int changed = 0;
    int err= 0;
    unsigned long val;

    err = strict_strtoul(buf, 10, &val);
    if (err)
        printk(KERN_ERR"strict_strtoul error is %s : return %d\n", __func__, err);

    inv_info("in %s: data is %ld\n", __func__, val);

    changed = (val == mpu->accel_status) ? 0 : 1;

    if (changed)
    {
        printk("accel status changed, now state is %s\n", (val > 0) ? "active" : "disable");
        if (val)
        {
            mpu6880_accel_enable(mpu);
            mpu->accel_status = 1;
        }
        else
        {
            mpu6880_accel_disable(mpu);
            mpu->accel_status = 0;
        }
    }

    return count;
}


static ssize_t mpu6880_accel_delay_show(struct device *dev, struct device_attribute *attr,
                 char *buf)
{
    struct input_polled_dev *polldev = dev_get_drvdata(dev);
    struct mpu6880_device *mpu = (struct mpu6880_device *)polldev->private;

    inv_info("%s : called\n", __func__);
    return sprintf(buf, "%d\n",  mpu->accel_poll);
}

static ssize_t mpu6880_accel_delay_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    struct input_polled_dev *polldev = dev_get_drvdata(dev);
    struct mpu6880_device *mpu = (struct mpu6880_device *)polldev->private;

    int err = 0;
    unsigned long val;

    err = strict_strtoul(buf, 10, &val);
    if (err)
        printk(KERN_ERR"strict_strtoul error is %s : return %d\n", __func__, err);

    inv_info("in %s: data is %ld\n", __func__, val);

    mpu->accel_poll = val;

    mpu6880_reset_delay(mpu);

    return count;
}



static ssize_t mpu6880_gyro_enable_show(struct device *dev, struct device_attribute *attr,
                 char *buf)
{
    struct input_polled_dev *polldev = dev_get_drvdata(dev);
    struct mpu6880_device *mpu = (struct mpu6880_device *)polldev->private;

    inv_info("%s : called name %s\n", __func__, mpu->input_dev->input->name);

    return sprintf(buf, "%d\n",  mpu->gyro_status);

}

static ssize_t mpu6880_gyro_enable_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    struct input_polled_dev *polldev = dev_get_drvdata(dev);
    struct mpu6880_device *mpu = (struct mpu6880_device *)polldev->private;
    int err = 0;
    int changed = 0;
    unsigned long val;

    err = strict_strtoul(buf, 10, &val);
    if (err)
        printk(KERN_ERR"strict_strtoul error is %s : return %d\n", __func__, err);

    inv_info("in %s: data is %ld\n", __func__,  val);

    changed = (val == mpu->gyro_status) ? 0 : 1;

    if (changed)
    {
        printk("gyro status changed, now state is %s\n", (val > 0) ? "active" : "disable");
        if (val)
        {
            mpu6880_gyro_enable(mpu);
            mpu->gyro_status = 1;
        }
        else
        {
            mpu6880_gyro_disable(mpu);
            mpu->gyro_status = 0;
        }
    }

    return count;
}

static ssize_t mpu6880_gyro_delay_show(struct device *dev, struct device_attribute *attr,
                 char *buf)
{
    struct input_polled_dev *polldev = dev_get_drvdata(dev);
    struct mpu6880_device *mpu = (struct mpu6880_device *)polldev->private;

    inv_info("%s : called name %s\n", __func__, mpu->input_dev->input->name);

    return sprintf(buf, "%d\n",  mpu->gyro_poll);

}

static ssize_t mpu6880_gyro_delay_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    struct input_polled_dev *polldev = dev_get_drvdata(dev);
    struct mpu6880_device *mpu = (struct mpu6880_device *)polldev->private;

    int err = 0;
    unsigned long val;

    err = strict_strtoul(buf, 10, &val);
    if (err)
        printk(KERN_ERR"strict_strtoul error is %s : return %d\n", __func__, err);

    inv_info("in %s: data is %ld\n",__func__,  val);

    mpu->gyro_poll = val;

    mpu6880_reset_delay(mpu);

    return count;
}

static ssize_t mpu6880_device_delay_show(struct device *dev, struct device_attribute *attr,
                 char *buf)
{
    struct input_polled_dev *polldev = dev_get_drvdata(dev);
    struct mpu6880_device *mpu = (struct mpu6880_device *)polldev->private;

    inv_info("%s : called name %s (delay: %d)\n", __func__, mpu->input_dev->input->name, mpu->poll_time);

    return sprintf(buf, "%d\n",  mpu->input_dev->poll_interval);

}

static ssize_t mpu6880_device_delay_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    struct input_polled_dev *polldev = dev_get_drvdata(dev);
    struct mpu6880_device *mpu = (struct mpu6880_device *)polldev->private;

    unsigned long val;

    int err = 0;
    err = strict_strtoul(buf, 10, &val);
    if (err)
        printk(KERN_ERR"strict_strtoul error is %s : return %d\n", __func__, err);

    inv_info("in %s: data is %ld\n",__func__,  val);

    mpu->poll_time = val;
    mpu->input_dev->poll_interval = mpu->poll_time;

    return count;
}

static ssize_t mpu6880_calibration_show(struct device *dev, struct device_attribute *attr,
                 char *buf)
{
    struct input_polled_dev *polldev = dev_get_drvdata(dev);
    struct mpu6880_device *mpu = (struct mpu6880_device *)polldev->private;
    struct acc_offset acc_cal_data;
    struct acc_offset *acc_cal_ptr = &acc_cal_data;
  struct gyro_offset gyro_cal_data;
    struct gyro_offset *gyro_cal_ptr = &gyro_cal_data;
    inv_info("%s : called name %s\n", __func__, mpu->input_dev->input->name);

    sensparams_read_from_flash(SENSPARAMS_TYPE_ACCEL, (unsigned char*)acc_cal_ptr, sizeof(acc_cal_data));
    mpu->accel_offset[0] = ((struct acc_offset *)acc_cal_ptr)->x;
        mpu->accel_offset[1] = ((struct acc_offset *)acc_cal_ptr)->y;
        mpu->accel_offset[2] = ((struct acc_offset *)acc_cal_ptr)->z;
    mpu->acc_key         = ((struct acc_offset *)acc_cal_ptr)->key;
  sensparams_read_from_flash(SENSPARAMS_TYPE_RESV1, (unsigned char*)gyro_cal_ptr, sizeof(gyro_cal_data));
      mpu->gyro_offset[0] = ((struct gyro_offset *)gyro_cal_ptr)->x;
        mpu->gyro_offset[1] = ((struct gyro_offset *)gyro_cal_ptr)->y;
        mpu->gyro_offset[2] = ((struct gyro_offset *)gyro_cal_ptr)->z;
    mpu->gyro_key        = ((struct gyro_offset *)gyro_cal_ptr)->key;
    printk(KERN_ERR "%s read: gsensor_xoffset = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->x);
    printk(KERN_ERR "%s read: gsensor_yoffset = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->y);
    printk(KERN_ERR "%s read: gsensor_zoffset = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->z);
    printk(KERN_ERR "%s read: gyro_xoffset = %d\n",  __func__, ((struct gyro_offset *)gyro_cal_ptr)->x);
    printk(KERN_ERR "%s read: gyro_yoffset = %d\n",  __func__, ((struct gyro_offset *)gyro_cal_ptr)->y);
    printk(KERN_ERR "%s read: gyro_zoffset = %d\n",  __func__, ((struct gyro_offset *)gyro_cal_ptr)->z);
    printk(KERN_ERR "%s read: acc_CalStatus = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->key);
    printk(KERN_ERR "%s read: gyro_CalStatus = %d\n",  __func__, ((struct gyro_offset *)gyro_cal_ptr)->key);

    return sprintf(buf, "AX%d AY%d AZ%d GX%d GY%d GZ%d AK%d GK%d\n",  mpu->accel_offset[0], mpu->accel_offset[1], mpu->accel_offset[2],
                mpu->gyro_offset[0],  mpu->gyro_offset[1],  mpu->gyro_offset[2],mpu->acc_key,mpu->gyro_key);

}

static ssize_t mpu6880_calibration_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    struct input_polled_dev *polldev = dev_get_drvdata(dev);
    struct mpu6880_device *mpu = (struct mpu6880_device *)polldev->private;
    unsigned long val;
    int err = 0;
  struct acc_offset acc_cal_data;
    struct acc_offset *acc_cal_ptr = &acc_cal_data;
    struct gyro_offset gyro_cal_data;
    struct gyro_offset *gyro_cal_ptr = &gyro_cal_data;
    //before calibration, save accel and gyro status
    int pre_accel = mpu->accel_status;
    int pre_gyro = mpu->gyro_status;


    err = strict_strtoul(buf, 10, &val);
    if (err)
        printk(KERN_ERR"strict_strtoul error is %s : return %d\n", __func__, err);

    if (val != 1)
        return count;
    //close accel and gyro to calibration

    mpu->accel_status = 0;
    mpu->gyro_status = 0;

    if (pre_accel == 0)
    {
        mpu6880_accel_enable(mpu);
    }

    if (pre_gyro == 0)
    {
        mpu6880_gyro_enable(mpu);
    }

    mpu6880_accel_gyro_calibration(mpu);
    ((struct acc_offset *)acc_cal_ptr)->x = mpu->accel_offset[0];
    ((struct acc_offset *)acc_cal_ptr)->y = mpu->accel_offset[1];
    ((struct acc_offset *)acc_cal_ptr)->z = mpu->accel_offset[2];
    ((struct acc_offset *)acc_cal_ptr)->key = mpu->acc_key;

    printk(KERN_ERR "%s write: xoffset = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->x);
    printk(KERN_ERR "%s write: yoffset = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->y);
    printk(KERN_ERR "%s write: zoffset = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->z);
    printk(KERN_ERR "%s write: CalStatus = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->key);
    sensparams_write_to_flash(SENSPARAMS_TYPE_ACCEL, (unsigned char*)acc_cal_ptr, sizeof(acc_cal_data));

    ((struct gyro_offset *)gyro_cal_ptr)->x = mpu->gyro_offset[0];
    ((struct gyro_offset *)gyro_cal_ptr)->y = mpu->gyro_offset[1];
    ((struct gyro_offset *)gyro_cal_ptr)->z = mpu->gyro_offset[2];
    ((struct gyro_offset *)gyro_cal_ptr)->key =mpu->gyro_key;
    printk(KERN_ERR "%s write: xoffset = %d\n",  __func__, ((struct gyro_offset *)gyro_cal_ptr)->x);
    printk(KERN_ERR "%s write: yoffset = %d\n",  __func__, ((struct gyro_offset *)gyro_cal_ptr)->y);
    printk(KERN_ERR "%s write: zoffset = %d\n",  __func__, ((struct gyro_offset *)gyro_cal_ptr)->z);
    printk(KERN_ERR "%s write: CalStatus = %d\n",  __func__, ((struct gyro_offset *)gyro_cal_ptr)->key);

    sensparams_write_to_flash(SENSPARAMS_TYPE_RESV1, (unsigned char*)gyro_cal_ptr, sizeof(gyro_cal_data));
    /*if(copy_to_user((struct acc_offset *)arg, acc_cal_ptr, sizeof(acc_cal_data)))
    {
        dev_err(&mpu->client->dev,  "%s:  Calibrate copy_to_user failed!\n", __func__);
        return -EFAULT;
    }*/
    //after calibration restore previous accel and gyro status
    mpu->accel_status = pre_accel;
    mpu->gyro_status = pre_gyro;

    if (pre_accel == 0)
    {
        mpu6880_accel_disable(mpu);
    }

    if (pre_gyro == 0)
    {
        mpu6880_gyro_disable(mpu);
    }

    return count;
}


static ssize_t mpu6880_debug_show(struct device *dev, struct device_attribute *attr,
                 char *buf)
{
    inv_info("%s : called inv_debug  is  %d\n", __func__, inv_debug);

    return sprintf(buf, "%d\n",  inv_debug);

}

static ssize_t mpu6880_debug_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    unsigned long val;
    int err = 0;

    err = strict_strtoul(buf, 10, &val);
    if (err)
        printk(KERN_ERR"strict_strtoul error is %s : return %d\n", __func__, err);

    inv_info("in %s: data is %ld\n",__func__,  val);

    inv_debug = val;

    return count;
}

static DEVICE_ATTR(accel_enable,  S_IWGRP|S_IWUSR | S_IRUGO, mpu6880_accel_enable_show,  mpu6880_accel_enable_store);
static DEVICE_ATTR(accel_delay,  S_IWGRP|S_IWUSR | S_IRUGO, mpu6880_accel_delay_show,  mpu6880_accel_delay_store);
static DEVICE_ATTR(gyro_enable,  S_IWGRP|S_IWUSR | S_IRUGO, mpu6880_gyro_enable_show,  mpu6880_gyro_enable_store);
static DEVICE_ATTR(gyro_delay,  S_IWGRP|S_IWUSR | S_IRUGO, mpu6880_gyro_delay_show,  mpu6880_gyro_delay_store);
static DEVICE_ATTR(device_delay,  S_IWGRP|S_IWUSR | S_IRUGO, mpu6880_device_delay_show,  mpu6880_device_delay_store);
static DEVICE_ATTR(calibration,  S_IWGRP|S_IWUSR | S_IRUGO, mpu6880_calibration_show,  mpu6880_calibration_store);
static DEVICE_ATTR(debug,  S_IWGRP|S_IWUSR | S_IRUGO, mpu6880_debug_show,  mpu6880_debug_store);

static int mpu6880_read_accel_xyz(struct mpu6880_device *dev)
{
    struct i2c_client *client = dev->client;
    u8 buff[6];
    int err;

    err = i2c_smbus_read_i2c_block_data(client, MPU6880_ACCEL_XOUT_H, 6, buff);
    if (err < 0) {
        printk( "failed to read block data at 0x%02x, error %d\n",
            MPU6880_ACCEL_XOUT_H, err);
        return err;
    }

    dev->accel_data[0] = ((buff[0] << 8) & 0xff00) | (buff[1] & 0xff);
    dev->accel_data [1] = ((buff[2] << 8) & 0xff00) | (buff[3] & 0xff);
    dev->accel_data [2] = ((buff[4] << 8) & 0xff00) | (buff[5] & 0xff);

    return 0;
}


static int mpu6880_read_gyro_xyz(struct mpu6880_device *dev)
{
    struct i2c_client *client = dev->client;
    u8 buff[6];
    int err;

    err = i2c_smbus_read_i2c_block_data(client, MPU6880_GYRO_XOUT_H, 6, buff);
    if (err < 0) {
        printk( "failed to read block data at 0x%02x, error %d\n",
            MPU6880_GYRO_XOUT_H, err);
        return err;
    }

    dev->gyro_data[0] = ((buff[0] << 8) & 0xff00) | (buff[1] & 0xff);
    dev->gyro_data [1] = ((buff[2] << 8) & 0xff00) | (buff[3] & 0xff);
    dev->gyro_data [2] = ((buff[4] << 8) & 0xff00) | (buff[5] & 0xff);

    return 0;
}


static void mpu6880_read_sensors_data(struct mpu6880_device *device)
{
    struct mpu6880_device *dev = device;

    if (dev->accel_status)
    {
        mpu6880_read_accel_xyz(dev);
        inv_info("accel data is read finished, %d  %d  %d\n", dev->accel_data[0],
            dev->accel_data[1], dev->accel_data[2]);
    }

    if (dev->gyro_status)
    {
        mpu6880_read_gyro_xyz(dev);
        inv_info("gyro data is read finished,  %d  %d  %d\n", dev->gyro_data[0],
            dev->gyro_data[1], dev->gyro_data[2]);
    }
}

static void mpu6880_report_value(struct mpu6880_device *device)
{
    struct mpu6880_device *dev = device;
    ktime_t timestamp;

    timestamp = ktime_get_boottime();

    if (dev->accel_status) {
        input_report_abs(dev->input_dev->input,ABS_X,   dev->accel_data[0]-dev->accel_offset[0]);
        input_report_abs(dev->input_dev->input, ABS_Y,  dev->accel_data[1]-dev->accel_offset[1]);
        input_report_abs(dev->input_dev->input,ABS_Z,  dev->accel_data[2]-dev->accel_offset[2]);
        input_event(dev->input_dev->input, EV_SYN, SYN_TIME_SEC, ktime_to_timespec(timestamp).tv_sec);
        input_event(dev->input_dev->input, EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(timestamp).tv_nsec);
        input_sync(dev->input_dev->input);
    }

    if (dev->gyro_status) {
        input_report_abs(dev->input_dev->input,ABS_RX, dev->gyro_data[0]-dev->gyro_offset[0]);
        input_report_abs(dev->input_dev->input, ABS_RY, dev->gyro_data[1]-dev->gyro_offset[1]);
        input_report_abs(dev->input_dev->input,ABS_RZ, dev->gyro_data[2]-dev->gyro_offset[2]);
        input_event(dev->input_dev->input, EV_SYN, SYN_TIME_SEC, ktime_to_timespec(timestamp).tv_sec);
        input_event(dev->input_dev->input, EV_SYN, SYN_TIME_NSEC, ktime_to_timespec(timestamp).tv_nsec);
        input_sync(dev->input_dev->input);
    }
}

static void mma6880_poll(struct input_polled_dev *dev)
{
    struct mpu6880_device *mpu = (struct mpu6880_device *)dev->private;

    mpu6880_read_sensors_data(mpu);

    mpu6880_report_value(mpu);

}

static int mpu6880_device_init(struct i2c_client *client)
{
    int res = 0;
    int test=0;
    res = mpu6880_write(client, MPU6880_PWR_MGMT_1,0x80);
    if (res)
    {
    inv_info(" Reset MPU6880 B1 error,res: %d!\n", res);
        return res;
    }

    msleep(40);

    res = mpu6880_write(client, MPU6880_GYRO_CONFIG,0x18);  //config gyro for 2000dps
    if (res)
    {
        inv_info("MPU6880 Configure Gyro FS error,res: %d!\n", res);
        return res;
    }
    msleep(10);
    res = mpu6880_write(client, MPU6880_CONFIG,0x03);  //config dlpf=41hz
    if (res)
    {
        inv_info("MPU6880 Configure Gyro dlpf error,res: %d!\n", res);
        return res;
    }

    msleep(10);

    res = mpu6880_write(client, MPU6880_ACCEL_CONFIG,0x00);  //config Accel for +_2G
    if (res)
    {
        inv_info("MPU6880 Configure Accel FS error,res: %d!\n", res);
        return res;
    }

    msleep(10);

    res = mpu6880_write(client, MPU6880_ACCEL_CONFIG2,0x03); //config dlpf=41hz
    if (res)
    {
        printk("MPU6880 Configure Accel dlpf,res: %d!\n", res);
        return res;
    }


    res = mpu6880_write(client, MPU6880_PWR_MGMT_2,0x3F); //set accl and gyro all axis into standby mode
    if (res)
    {
        printk("MPU6880 Configure pwrm error,res: %d!\n", res);
        return res;
    }

    msleep(10);


    res = mpu6880_write(client, MPU6880_PWR_MGMT_1,0x41);
    if (res)
    {
        printk("MPU6880 into sleep mode error,res: %d!\n", res);
        return res;
    }

    msleep(10);
    test = mpu6880_read(client, MPU6880_PWR_MGMT_1);

        printk(KERN_ERR "taojun=%d\n",test);

    return 0;

}

static struct attribute *mpu6880_sysfs_attrs[] = {
    &dev_attr_accel_enable.attr,
    &dev_attr_accel_delay.attr,
    &dev_attr_gyro_enable.attr,
    &dev_attr_gyro_delay.attr,
    &dev_attr_device_delay.attr,
    &dev_attr_calibration.attr,
    &dev_attr_debug.attr,

    NULL
};

struct attribute_group mpu6880_attrs = {
    .attrs = mpu6880_sysfs_attrs,
};

int mpu6880_probe(struct i2c_client *client,
          const struct i2c_device_id *devid)
{
    struct mpu6880_device *mpu;
    int err;
    printk(KERN_ERR "mpu6880 driver: ini1111t\n");
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        dev_err(&client->dev, "platform data is NULL. exiting.\n");
        err = -ENODEV;
        goto exit_check_functionality_failed;
    }

    mpu = kzalloc(sizeof(struct mpu6880_device), GFP_KERNEL);
    if (!mpu) {
        dev_err(&client->dev,
            "failed to allocate memory for module data\n");
        err = -ENOMEM;
        goto exit_alloc_data_failed;
    }

    mutex_init(&mpu->mutex);

//  i2c_set_clientdata(client, mpu);

    err  = mpu6880_device_init(client);
    if (err )
    {
        printk(KERN_ERR"MPU6880 init fail\n");
        goto exit_input_dev_alloc_failed;
    }
    mpu->client = client;
printk(KERN_ERR"MPdddd faidddl\n");
    mpu->input_dev = input_allocate_polled_device();
    if (!mpu->input_dev) {
        err = -ENOMEM;
        dev_err(&mpu->client->dev,
            "input device allocate failed\n");
        goto exit_input_dev_alloc_failed;
    }
    printk(KERN_ERR"MPdddd fail\n");
    mpu->input_dev->private = mpu;

printk(KERN_ERR"MPdddd ttaoddl\n");
    mpu->input_dev->input->name = MPU_DEVICE_NAME;
    mpu->input_dev->input->phys = MPU_DEVICE_NAME "/input0";
    mpu->input_dev->input->id.bustype   = BUS_I2C;
    mpu->input_dev->input->dev.parent = &mpu->client->dev;
    mpu->input_dev->poll        = mma6880_poll;
    mpu->input_dev->poll_interval   = MPU6880_POLL_INTERVAL;
    mpu->input_dev->poll_interval_max   = MPU6880_POLL_INTERVAL_MAX;
    mpu->input_dev->poll_interval_min   = MPU6880_POLL_INTERVAL_MIN;
    mpu->accel_poll = MPU6880_POLL_INTERVAL;
    mpu->gyro_poll = MPU6880_POLL_INTERVAL;

    //set_bit(EV_ABS, mpu->input_dev->input->evbit);
  mpu->input_dev->input->evbit[0] = BIT_MASK(EV_ABS);
    /* x-axis acceleration */
    input_set_abs_params(mpu->input_dev->input, ABS_X, -32768, 32767, 0, 0);
    /* y-axis acceleration */
    input_set_abs_params(mpu->input_dev->input, ABS_Y, -32768, 32767, 0, 0);
    /* z-axis acceleration */
    input_set_abs_params(mpu->input_dev->input, ABS_Z, -32768, 32767, 0, 0);

    /* Gyro X-axis */
    input_set_abs_params(mpu->input_dev->input, ABS_RX, -32768, 32767, 0, 0);
    /* Gyro Y-axis */
    input_set_abs_params(mpu->input_dev->input,  ABS_RY, -32768, 32767, 0, 0);
    /* Gyro Z-axis */
    input_set_abs_params(mpu->input_dev->input, ABS_RZ, -32768, 32767, 0, 0);
printk(KERN_ERR"MPdddd ddddd\n");
    err = input_register_polled_device(mpu->input_dev);
    if (err) {
        pr_err("mpu6880_probe: Unable to register input device: %s\n",
                     mpu->input_dev->input->name);
        goto exit_input_register_device_failed;
    }
    printk(KERN_ERR"MPbbbbtaojunddd\n");
//  bma150->input = ipoll_dev->input;
    //mpu->input_dev->private = mpu;
//  mpu->input_dev->close(mpu->input_dev);
  i2c_set_clientdata(client, mpu);
    if (sysfs_create_group(&mpu->input_dev->input->dev.kobj,
                   &mpu6880_attrs))
    {
        printk(KERN_ERR"%s create sysfile error\n", __func__);
        goto exit_input_register_device_failed;
    }

    printk("mpu6880 probe successfully\n");

    return 0;

exit_input_register_device_failed:
    input_unregister_polled_device(mpu->input_dev);
exit_input_dev_alloc_failed:
    kfree(mpu);
exit_alloc_data_failed:
exit_check_functionality_failed:
    return err;
}

static int /*__devexit*/ mpu6880_remove(struct i2c_client *client)
{
    struct mpu6880_device *mpu = i2c_get_clientdata(client);
    input_unregister_polled_device(mpu->input_dev);
    kfree(mpu);
    return 0;
}

static const struct i2c_device_id mpu6880_id[] = {
    { "mpu6880", 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, mpu6880_id);

static struct of_device_id mpu6880_match_table[] = {
    { .compatible = "mpu6880",},
    { },
};

static struct i2c_driver mpu6880_driver = {
    .driver = {
        .name = "mpu6880",
        .owner=THIS_MODULE,
        .of_match_table = mpu6880_match_table,
    },
    .probe = mpu6880_probe,
    .remove = mpu6880_remove,
    .id_table = mpu6880_id,

};

static int __init mpu6880_init(void)
{
    inv_info("mpu6880 driver: init\n");
    printk(KERN_ERR "mpu6880 driver: init\n");
    return i2c_add_driver(&mpu6880_driver);
}

static void __exit mpu6880_exit(void)
{
    inv_info("mpu6880 driver: exit\n");
    i2c_del_driver(&mpu6880_driver);
}



MODULE_AUTHOR("Mingyang Li <myli@invensense.com>");
MODULE_DESCRIPTION("mpu6880 driver");
MODULE_LICENSE("GPL");

module_init(mpu6880_init);
//late_initcall(mpu6880_init);
module_exit(mpu6880_exit);
