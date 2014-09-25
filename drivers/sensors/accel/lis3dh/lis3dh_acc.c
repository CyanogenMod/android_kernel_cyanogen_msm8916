/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
 *
 * File Name          : lis3dh_acc.c
 * Authors            : MSH - Motion Mems BU - Application Team
 *		      : Matteo Dameno (matteo.dameno@st.com)
 *		      : Carmine Iascone (carmine.iascone@st.com)
 *		      : Both authors are willing to be considered the contact
 *		      : and update points for the driver.
 * Version            : V.1.0.8
 * Date               : 2010/Apr/01
 * Description        : LIS3DH accelerometer sensor API
 *
 *******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
 *
 ******************************************************************************
 Revision 1.0.0 05/11/09
 First Release;
 Revision 1.0.3 22/01/2010
  Linux K&R Compliant Release;
 Revision 1.0.5 16/08/2010
  modified _get_acceleration_data function;
  modified _update_odr function;
  manages 2 interrupts;
 Revision 1.0.6 15/11/2010
  supports sysfs;
  no more support for ioctl;
 Revision 1.0.7 26/11/2010
  checks for availability of interrupts pins
  correction on FUZZ and FLAT values;
 Revision 1.0.8 2010/Apr/01
  corrects a bug in interrupt pin management in 1.0.7

 ******************************************************************************/

#include	<linux/err.h>
#include	<linux/errno.h>
#include	<linux/delay.h>
#include	<linux/fs.h>
#include	<linux/i2c.h>
#include	<linux/input.h>
#include	<linux/workqueue.h>
#include	<linux/irq.h>
#include	<linux/gpio.h>
#include	<linux/interrupt.h>
#include	<linux/slab.h>
#include   <linux/module.h>

/***************************************/
//longjiang add 20130620
#ifdef CONFIG_OF
#include <linux/of.h>
#endif
//add end
/***************************************/

//longjiang add platform code 20130520
#include <linux/regulator/consumer.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/sensors/lis3dh.h>
#include <linux/sensors/sensparams.h>

#ifdef CONFIG_YL_SENSORS_INFO
#include <linux/sensors/sensors_info.h>
#endif
/* Debug Message Flags */
#define YL_KMSG_INF	1	/* Print kernel debug message for info */

#if YL_KMSG_INF
#define KMSGINF(format, ...)	\
		dev_info(format, ## __VA_ARGS__)
#else
#define KMSGINF(format, ...)
#endif

static struct lis3dh_acc_data* this_acc=0; /*added by liwenpeng*/
#define CAL_FUZZ	157 //200//20
#define CAL_CONVERT 1024

//add end

#define	G_MAX		16000

#define SENSITIVITY_2G		1	/**	mg/LSB	*/
#define SENSITIVITY_4G		2	/**	mg/LSB	*/
#define SENSITIVITY_8G		4	/**	mg/LSB	*/
#define SENSITIVITY_16G		12	/**	mg/LSB	*/

/* Accelerometer Sensor Operating Mode */
#define LIS3DH_ACC_ENABLE	0x01
#define LIS3DH_ACC_DISABLE	0x00

#define	HIGH_RESOLUTION		0x08

#define	AXISDATA_REG		0x28
#define WHOAMI_LIS3DH_ACC	0x33	/*	Expected content for WAI */

/*	CONTROL REGISTERS	*/
#define WHO_AM_I		0x0F	/*	WhoAmI register		*/
#define	TEMP_CFG_REG		0x1F	/*	temper sens control reg	*/
/* ctrl 1: ODR3 ODR2 ODR ODR0 LPen Zenable Yenable Zenable */
#define	CTRL_REG1		0x20	/*	control reg 1		*/
#define	CTRL_REG2		0x21	/*	control reg 2		*/
#define	CTRL_REG3		0x22	/*	control reg 3		*/
#define	CTRL_REG4		0x23	/*	control reg 4		*/
#define	CTRL_REG5		0x24	/*	control reg 5		*/
#define	CTRL_REG6		0x25	/*	control reg 6		*/

#define	FIFO_CTRL_REG		0x2E	/*	FiFo control reg	*/

#define	INT_CFG1		0x30	/*	interrupt 1 config	*/
#define	INT_SRC1		0x31	/*	interrupt 1 source	*/
#define	INT_THS1		0x32	/*	interrupt 1 threshold	*/
#define	INT_DUR1		0x33	/*	interrupt 1 duration	*/

#define	TT_CFG			0x38	/*	tap config		*/
#define	TT_SRC			0x39	/*	tap source		*/
#define	TT_THS			0x3A	/*	tap threshold		*/
#define	TT_LIM			0x3B	/*	tap time limit		*/
#define	TT_TLAT			0x3C	/*	tap time latency	*/
#define	TT_TW			0x3D	/*	tap time window		*/
/*	end CONTROL REGISTRES	*/

#define ENABLE_HIGH_RESOLUTION	1

#define LIS3DH_ACC_PM_OFF		0x00
#define LIS3DH_ACC_ENABLE_ALL_AXES	0x07

#define PMODE_MASK			0x08
#define ODR_MASK			0XF0

#define ODR1		0x10  /* 1Hz output data rate */
#define ODR10		0x20  /* 10Hz output data rate */
#define ODR25		0x30  /* 25Hz output data rate */
#define ODR50		0x40  /* 50Hz output data rate */
#define ODR100		0x50  /* 100Hz output data rate */
#define ODR200		0x60  /* 200Hz output data rate */
#define ODR400		0x70  /* 400Hz output data rate */
#define ODR1250		0x90  /* 1250Hz output data rate */

#define	IA			0x40
#define	ZH			0x20
#define	ZL			0x10
#define	YH			0x08
#define	YL			0x04
#define	XH			0x02
#define	XL			0x01
/* */
/* CTRL REG BITS*/
#define	CTRL_REG3_I1_AOI1	0x40
#define	CTRL_REG6_I2_TAPEN	0x80
#define	CTRL_REG6_HLACTIVE	0x02
/* */
#define NO_MASK			0xFF
#define INT1_DURATION_MASK	0x7F
#define	INT1_THRESHOLD_MASK	0x7F
#define TAP_CFG_MASK		0x3F
#define	TAP_THS_MASK		0x7F
#define	TAP_TLIM_MASK		0x7F
#define	TAP_TLAT_MASK		NO_MASK
#define	TAP_TW_MASK		NO_MASK


/* TAP_SOURCE_REG BIT */
#define	DTAP			0x20
#define	STAP			0x10
#define	SIGNTAP			0x08
#define	ZTAP			0x04
#define	YTAP			0x02
#define	XTAZ			0x01


#define	FUZZ			0
#define	FLAT			0
#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		2 //5 vincent 20120207
#define	I2C_AUTO_INCREMENT	0x80

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RES_CTRL_REG6		5

#define	RES_INT_CFG1		6
#define	RES_INT_THS1		7
#define	RES_INT_DUR1		8

#define	RES_TT_CFG		9
#define	RES_TT_THS		10
#define	RES_TT_LIM		11
#define	RES_TT_TLAT		12
#define	RES_TT_TW		13

#define	RES_TEMP_CFG_REG	14
#define	RES_REFERENCE_REG	15
#define	RES_FIFO_CTRL_REG	16

#define	RESUME_ENTRIES		17
/* end RESUME STATE INDICES */


//vincent
struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lis3dh_acc_odr_table[] = {
		{    1, ODR1250 },
		{    3, ODR400  },
		{    5, ODR200  },
		{   10, ODR100  },
		{   20, ODR50   },
		{   40, ODR25   },
		{  100, ODR10   },
		{ 1000, ODR1    },
};

struct lis3dh_acc_data {
	struct i2c_client *client;
	struct lis3dh_acc_platform_data *pdata;

	struct mutex lock;
	struct delayed_work input_work;

	struct input_dev *input_dev;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	atomic_t enabled;
	int on_before_suspend;

	u8 sensitivity;

	u8 resume_state[RESUME_ENTRIES];

	int irq1;
	struct work_struct irq1_work;
	struct workqueue_struct *irq1_work_queue;
	int irq2;
	struct work_struct irq2_work;
	struct workqueue_struct *irq2_work_queue;

	u8 reg_addr;

	//longjiang add platform code start 20130520
	int accel_cali[3];
	//add end
#ifdef CONFIG_SENSOR_POWER
	struct regulator *vcc_i2c;
	struct regulator *vdd_ana;
#endif
};

static int lis3dh_acc_i2c_read(struct lis3dh_acc_data *acc,
				u8 * buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg	msgs[] = {
		{
			.addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = acc->client->addr,
			.flags = (acc->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&acc->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lis3dh_acc_i2c_write(struct lis3dh_acc_data *acc, u8 * buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
		 .addr = acc->client->addr,
			.flags = acc->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(acc->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&acc->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lis3dh_acc_hw_init(struct lis3dh_acc_data *acc)
{
	int err = -1;
	u8 buf[7];

	KMSGINF(&acc->client->dev, "%s: hw init start\n", __func__);

	buf[0] = WHO_AM_I;
	err = lis3dh_acc_i2c_read(acc, buf, 1);
	if (err < 0) {
	dev_warn(&acc->client->dev, "Error reading WHO_AM_I: is device "
		"available/working?\n");
		goto err_firstread;
	} else
		acc->hw_working = 1;
	if (buf[0] != WHOAMI_LIS3DH_ACC) {
	dev_err(&acc->client->dev,
		"device unknown. Expected: 0x%x,"
		" Replies: 0x%x\n", WHOAMI_LIS3DH_ACC, buf[0]);
		err = -1; /* choose the right coded error */
		goto err_unknown_device;
	}

	buf[0] = CTRL_REG1;
	buf[1] = acc->resume_state[RES_CTRL_REG1];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = TEMP_CFG_REG;
	buf[1] = acc->resume_state[RES_TEMP_CFG_REG];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = FIFO_CTRL_REG;
	buf[1] = acc->resume_state[RES_FIFO_CTRL_REG];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | TT_THS);
	buf[1] = acc->resume_state[RES_TT_THS];
	buf[2] = acc->resume_state[RES_TT_LIM];
	buf[3] = acc->resume_state[RES_TT_TLAT];
	buf[4] = acc->resume_state[RES_TT_TW];
	err = lis3dh_acc_i2c_write(acc, buf, 4);
	if (err < 0)
		goto err_resume_state;
	buf[0] = TT_CFG;
	buf[1] = acc->resume_state[RES_TT_CFG];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = (I2C_AUTO_INCREMENT | INT_THS1);
	buf[1] = acc->resume_state[RES_INT_THS1];
	buf[2] = acc->resume_state[RES_INT_DUR1];
	err = lis3dh_acc_i2c_write(acc, buf, 2);
	if (err < 0)
		goto err_resume_state;
	buf[0] = INT_CFG1;
	buf[1] = acc->resume_state[RES_INT_CFG1];
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		goto err_resume_state;


	buf[0] = (I2C_AUTO_INCREMENT | CTRL_REG2);
	buf[1] = acc->resume_state[RES_CTRL_REG2];
	buf[2] = acc->resume_state[RES_CTRL_REG3];
	buf[3] = acc->resume_state[RES_CTRL_REG4];
	buf[4] = acc->resume_state[RES_CTRL_REG5];
	buf[5] = acc->resume_state[RES_CTRL_REG6];
	err = lis3dh_acc_i2c_write(acc, buf, 5);
	if (err < 0)
		goto err_resume_state;

	acc->hw_initialized = 1;
	KMSGINF(&acc->client->dev, "%s: hw init done\n", __func__);
	return 0;

err_firstread:
	acc->hw_working = 0;
err_unknown_device:
err_resume_state:
	acc->hw_initialized = 0;
	dev_err(&acc->client->dev, "hw init error 0x%x,0x%x: %d\n", buf[0],
			buf[1], err);
	return err;
}

static void lis3dh_acc_device_power_off(struct lis3dh_acc_data *acc)
{
	int err;
	u8 buf[2] = { CTRL_REG1, LIS3DH_ACC_PM_OFF };
	err = lis3dh_acc_i2c_write(acc, buf, 1);
	if (err < 0)
		dev_err(&acc->client->dev, "soft power off failed: %d\n", err);

	if (acc->pdata->power_off) {
		if(acc->pdata->gpio_int1 >= 0)
			disable_irq_nosync(acc->irq1);
		if(acc->pdata->gpio_int2 >= 0)
			disable_irq_nosync(acc->irq2);
		acc->pdata->power_off();
		acc->hw_initialized = 0;
	}

	if (acc->hw_initialized) {
		if(acc->pdata->gpio_int1 >= 0)
			disable_irq_nosync(acc->irq1);
		if(acc->pdata->gpio_int2 >= 0)
			disable_irq_nosync(acc->irq2);
		acc->hw_initialized = 0;
	}

}

static int lis3dh_acc_device_power_on(struct lis3dh_acc_data *acc)
{
	int err = -1;

	if (acc->pdata->power_on) {
		err = acc->pdata->power_on();
		if (err < 0) {
			dev_err(&acc->client->dev,
					"power_on failed: %d\n", err);
			return err;
		}
		if(acc->pdata->gpio_int1 >= 0)
			enable_irq(acc->irq1);
		if(acc->pdata->gpio_int2 >= 0)
			enable_irq(acc->irq2);
	}

	if (!acc->hw_initialized) {
		err = lis3dh_acc_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) {
			lis3dh_acc_device_power_off(acc);
			return err;
		}
		//vincent add for return qucketly when err 20120207
		if(err < 0){
			return err;
		}
		//vincent add for return qucketly when err 20120207
	}

	if (acc->hw_initialized) {
		if(acc->pdata->gpio_int1 >= 0)
			enable_irq(acc->irq1);
		if(acc->pdata->gpio_int2 >= 0)
			enable_irq(acc->irq2);
	}
	return 0;
}

static irqreturn_t lis3dh_acc_isr1(int irq, void *dev)
{
	struct lis3dh_acc_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq1_work_queue, &acc->irq1_work);
	printk(KERN_INFO "%s: isr1 queued\n", LIS3DH_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static irqreturn_t lis3dh_acc_isr2(int irq, void *dev)
{
	struct lis3dh_acc_data *acc = dev;

	disable_irq_nosync(irq);
	queue_work(acc->irq2_work_queue, &acc->irq2_work);
	printk(KERN_INFO "%s: isr2 queued\n", LIS3DH_ACC_DEV_NAME);

	return IRQ_HANDLED;
}

static void lis3dh_acc_irq1_work_func(struct work_struct *work)
{

	struct lis3dh_acc_data *acc =
	container_of(work, struct lis3dh_acc_data, irq1_work);
	/* TODO  add interrupt service procedure.
		 ie:lis3dh_acc_get_int1_source(acc); */
	;
	/*  */
	printk(KERN_INFO "%s: IRQ1 triggered\n", LIS3DH_ACC_DEV_NAME);
//exit:
	enable_irq(acc->irq1);
}

static void lis3dh_acc_irq2_work_func(struct work_struct *work)
{

	struct lis3dh_acc_data *acc =
	container_of(work, struct lis3dh_acc_data, irq2_work);
	/* TODO  add interrupt service procedure.
		 ie:lis3dh_acc_get_tap_source(acc); */
	;
	/*  */

	printk(KERN_INFO "%s: IRQ2 triggered\n", LIS3DH_ACC_DEV_NAME);
//exit:
	enable_irq(acc->irq2);
}

int lis3dh_acc_update_g_range(struct lis3dh_acc_data *acc, u8 new_g_range)
{
	int err=-1;

	u8 sensitivity;
	u8 buf[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = LIS3DH_ACC_FS_MASK | HIGH_RESOLUTION;

	switch (new_g_range) {
	case LIS3DH_ACC_G_2G:

		sensitivity = SENSITIVITY_2G;
		break;
	case LIS3DH_ACC_G_4G:

		sensitivity = SENSITIVITY_4G;
		break;
	case LIS3DH_ACC_G_8G:

		sensitivity = SENSITIVITY_8G;
		break;
	case LIS3DH_ACC_G_16G:

		sensitivity = SENSITIVITY_16G;
		break;
	default:
		dev_err(&acc->client->dev, "invalid g range requested: %u\n",
				new_g_range);
		return -EINVAL;
	}

	if (atomic_read(&acc->enabled)) {
		/* Updates configuration register 4,
		* which contains g range setting */
		buf[0] = CTRL_REG4;
		err = lis3dh_acc_i2c_read(acc, buf, 1);
		if (err < 0)
			goto error;
		init_val = buf[0];
		acc->resume_state[RES_CTRL_REG4] = init_val;
		new_val = new_g_range | HIGH_RESOLUTION;
		updated_val = ((mask & new_val) | ((~mask) & init_val));
		buf[1] = updated_val;
		buf[0] = CTRL_REG4;
		err = lis3dh_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			goto error;
		acc->resume_state[RES_CTRL_REG4] = updated_val;
		acc->sensitivity = sensitivity;
	}


	return err;
error:
	dev_err(&acc->client->dev, "update g range failed 0x%x,0x%x: %d\n",
			buf[0], buf[1], err);

	return err;
}

int lis3dh_acc_update_odr(struct lis3dh_acc_data *acc, int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config[2];

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(lis3dh_acc_odr_table) - 1; i >= 0; i--) {
		if (lis3dh_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
			break;
	}
	config[1] = lis3dh_acc_odr_table[i].mask;

	config[1] |= LIS3DH_ACC_ENABLE_ALL_AXES;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&acc->enabled)) {
		config[0] = CTRL_REG1;
		err = lis3dh_acc_i2c_write(acc, config, 1);
		if (err < 0)
			goto error;
		acc->resume_state[RES_CTRL_REG1] = config[1];
	}

	return err;

error:
	dev_err(&acc->client->dev, "update odr failed 0x%x,0x%x: %d\n",
			config[0], config[1], err);

	return err;
}



static int lis3dh_acc_register_write(struct lis3dh_acc_data *acc, u8 *buf,
		u8 reg_address, u8 new_value)
{
	int err = -1;

	/* Sets configuration register at reg_address
	 *  NOTE: this is a straight overwrite  */
		buf[0] = reg_address;
		buf[1] = new_value;
		err = lis3dh_acc_i2c_write(acc, buf, 1);
		if (err < 0)
			return err;
	return err;
}

static int lis3dh_acc_get_acceleration_data(struct lis3dh_acc_data *acc,
		int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s16 hw_d[3] = { 0 };

	acc_data[0] = (I2C_AUTO_INCREMENT | AXISDATA_REG);
	err = lis3dh_acc_i2c_read(acc, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (((s16) ((acc_data[1] << 8) | acc_data[0])) >> 4);
	hw_d[1] = (((s16) ((acc_data[3] << 8) | acc_data[2])) >> 4);
	hw_d[2] = (((s16) ((acc_data[5] << 8) | acc_data[4])) >> 4);

	hw_d[0] = hw_d[0] * acc->sensitivity;
	hw_d[1] = hw_d[1] * acc->sensitivity;
	hw_d[2] = hw_d[2] * acc->sensitivity;
      #if 0
  	xyz[0] = (((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
		   : (hw_d[acc->pdata->axis_map_x])) );
	xyz[1] = (((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
		   : (hw_d[acc->pdata->axis_map_y])) );
	xyz[2] = (((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
		   : (hw_d[acc->pdata->axis_map_z])) );
	printk(KERN_INFO "%s ddddd_read x=%d, y=%d, z=%d\n",
			LIS3DH_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
    #endif
	xyz[0] = (((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
		   : (hw_d[acc->pdata->axis_map_x])) - acc->accel_cali[acc->pdata->axis_map_x]);
	xyz[1] = (((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
		   : (hw_d[acc->pdata->axis_map_y])) - acc->accel_cali[acc->pdata->axis_map_y]);
	xyz[2] = (((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
		   : (hw_d[acc->pdata->axis_map_z])) - acc->accel_cali[acc->pdata->axis_map_z]);
	
    #if 0
	printk(KERN_INFO "%s read x=%d, y=%d, z=%d\n",
			LIS3DH_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
    #endif
	return err;
}

static void lis3dh_acc_report_values(struct lis3dh_acc_data *acc,
					int *xyz)
{
	input_report_abs(acc->input_dev, ABS_X, xyz[0]);
	input_report_abs(acc->input_dev, ABS_Y, xyz[1]);
	input_report_abs(acc->input_dev, ABS_Z, xyz[2]);
	input_sync(acc->input_dev);
    #if 0
	printk(KERN_INFO"%s read x=%d, y=%d, z=%d\n",
			LIS3DH_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);	
    #endif
}

static int lis3dh_acc_enable(struct lis3dh_acc_data *acc)
{
	int err;

	if (!atomic_cmpxchg(&acc->enabled, 0, 1)) {
		err = lis3dh_acc_device_power_on(acc);
		if (err < 0) {
			atomic_set(&acc->enabled, 0);
			return err;
		}
		schedule_delayed_work(&acc->input_work,
			msecs_to_jiffies(acc->pdata->poll_interval));
	}

	return 0;
}

static int lis3dh_acc_disable(struct lis3dh_acc_data *acc)
{
	if (atomic_cmpxchg(&acc->enabled, 1, 0)) {
		cancel_delayed_work_sync(&acc->input_work);
		lis3dh_acc_device_power_off(acc);
	}

	return 0;
}

static ssize_t read_single_reg(struct device *dev, char *buf, u8 reg)
{
	ssize_t ret;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	int err;

	u8 data = reg;
	err = lis3dh_acc_i2c_read(acc, &data, 1);
	if (err < 0)
		return err;
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;

}

static int write_reg(struct device *dev, const char *buf, u8 reg,
		u8 mask, int resumeIndex)
{
	int err = -1;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	u8 x[2];
	u8 new_val;
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	new_val=((u8) val & mask);
	x[0] = reg;
	x[1] = new_val;
	err = lis3dh_acc_register_write(acc, x,reg,new_val);
	if (err < 0)
		return err;
	acc->resume_state[resumeIndex] = new_val;
	return err;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	mutex_lock(&acc->lock);
	val = acc->pdata->poll_interval;
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;
	if (!interval_ms)
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->pdata->poll_interval = interval_ms;
	lis3dh_acc_update_odr(acc, interval_ms);
	mutex_unlock(&acc->lock);
	return size;
}

static ssize_t attr_get_range(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	char val;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	char range = 2;
	mutex_lock(&acc->lock);
	val = acc->pdata->g_range ;
	switch (val) {
	case LIS3DH_ACC_G_2G:
		range = 2;
		break;
	case LIS3DH_ACC_G_4G:
		range = 4;
		break;
	case LIS3DH_ACC_G_8G:
		range = 8;
		break;
	case LIS3DH_ACC_G_16G:
		range = 16;
		break;
	}
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->pdata->g_range = val;
	lis3dh_acc_update_g_range(acc, val);
	mutex_unlock(&acc->lock);
	return size;
}

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	int val = atomic_read(&acc->enabled);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
    
	if (val)
		lis3dh_acc_enable(acc);
	else
		lis3dh_acc_disable(acc);

	return size;
}

static ssize_t attr_set_intconfig1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev,buf,INT_CFG1,NO_MASK,RES_INT_CFG1);
}

static ssize_t attr_get_intconfig1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev,buf,INT_CFG1);
}

static ssize_t attr_set_duration1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev,buf,INT_DUR1,INT1_DURATION_MASK,RES_INT_DUR1);
}

static ssize_t attr_get_duration1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev,buf,INT_DUR1);
}

static ssize_t attr_set_thresh1(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev,buf,INT_THS1,INT1_THRESHOLD_MASK,RES_INT_THS1);
}

static ssize_t attr_get_thresh1(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev,buf,INT_THS1);
}

static ssize_t attr_get_source1(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev,buf,INT_SRC1);
}

static ssize_t attr_set_click_cfg(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev,buf,TT_CFG,TAP_CFG_MASK,RES_TT_CFG);
}

static ssize_t attr_get_click_cfg(struct device *dev,
		struct device_attribute *attr,	char *buf)
{

	return read_single_reg(dev,buf,TT_CFG);
}

static ssize_t attr_get_click_source(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev,buf,TT_SRC);
}

static ssize_t attr_set_click_ths(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev,buf,TT_THS,TAP_THS_MASK,RES_TT_THS);
}

static ssize_t attr_get_click_ths(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev,buf,TT_THS);
}

static ssize_t attr_set_click_tlim(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev,buf,TT_LIM,TAP_TLIM_MASK,RES_TT_LIM);
}

static ssize_t attr_get_click_tlim(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev,buf,TT_LIM);
}

static ssize_t attr_set_click_tlat(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev,buf,TT_TLAT,TAP_TLAT_MASK,RES_TT_TLAT);
}

static ssize_t attr_get_click_tlat(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev,buf,TT_TLAT);
}

static ssize_t attr_set_click_tw(struct device *dev,
		struct device_attribute *attr,	const char *buf, size_t size)
{
	return write_reg(dev,buf,TT_TLAT,TAP_TW_MASK,RES_TT_TLAT);
}

static ssize_t attr_get_click_tw(struct device *dev,
		struct device_attribute *attr,	char *buf)
{
	return read_single_reg(dev,buf,TT_TLAT);
}

/* PAY ATTENTION: These DEBUG funtions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	u8 x[2];
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	x[0] = acc->reg_addr;
	mutex_unlock(&acc->lock);
	x[1] = val;
	rc = lis3dh_acc_i2c_write(acc, x, 1);
	/*TODO: error need to be managed */
	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	int rc;
	u8 data;

	mutex_lock(&acc->lock);
	data = acc->reg_addr;
	mutex_unlock(&acc->lock);
	rc = lis3dh_acc_i2c_read(acc, &data, 1);
	/*TODO: error need to be managed */
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;
	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;
	mutex_lock(&acc->lock);
	acc->reg_addr = val;
	mutex_unlock(&acc->lock);
	return size;
}

static struct device_attribute attributes[] = {

	__ATTR(pollrate_ms, 0664, attr_get_polling_rate, attr_set_polling_rate),
	__ATTR(range, 0664, attr_get_range, attr_set_range),
	__ATTR(enable, 0664, attr_get_enable, attr_set_enable),
	__ATTR(int1_config, 0664, attr_get_intconfig1, attr_set_intconfig1),
	__ATTR(int1_duration, 0664, attr_get_duration1, attr_set_duration1),
	__ATTR(int1_threshold, 0664, attr_get_thresh1, attr_set_thresh1),
	__ATTR(int1_source, 0444, attr_get_source1, NULL),
	__ATTR(click_config, 0664, attr_get_click_cfg, attr_set_click_cfg),
	__ATTR(click_source, 0444, attr_get_click_source, NULL),
	__ATTR(click_threshold, 0664, attr_get_click_ths, attr_set_click_ths),
	__ATTR(click_timelimit, 0664, attr_get_click_tlim, attr_set_click_tlim),
	__ATTR(click_timelatency, 0664, attr_get_click_tlat,
							attr_set_click_tlat),
	__ATTR(click_timewindow, 0664, attr_get_click_tw, attr_set_click_tw),

	__ATTR(reg_value, 0600, attr_reg_get, attr_reg_set),
	__ATTR(reg_addr, 0200, NULL, attr_addr_set),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

static void lis3dh_acc_input_work_func(struct work_struct *work)
{
	struct lis3dh_acc_data *acc;

	int xyz[3] = { 0 };
	int err;

	acc = container_of((struct delayed_work *)work,
			struct lis3dh_acc_data,	input_work);

	mutex_lock(&acc->lock);
	err = lis3dh_acc_get_acceleration_data(acc, xyz);
	if (err < 0)
		dev_err(&acc->client->dev, "get_acceleration_data failed\n");
	else{
		lis3dh_acc_report_values(acc, xyz);
	}
	schedule_delayed_work(&acc->input_work, msecs_to_jiffies(
			acc->pdata->poll_interval));
	mutex_unlock(&acc->lock);
 #if 0
	printk(KERN_INFO "%s: acc->pdata->poll_interval=%d\n",
			__func__, acc->pdata->poll_interval);	
 #endif
}

static int gsensor_open(struct inode *inode, struct file *file)
{
    	return 0;
}

static int gsensor_release(struct inode *inode, struct file *file)
{
	return 0;
}

//longjiang add platform code start 20130520
static int lis3dh_acc_read_data(struct lis3dh_acc_data *acc,
		int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s16 hw_d[3] = { 0 };

	acc_data[0] = (I2C_AUTO_INCREMENT | AXISDATA_REG);
	err = lis3dh_acc_i2c_read(acc, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (((s16) ((acc_data[1] << 8) | acc_data[0])) >> 4);
	hw_d[1] = (((s16) ((acc_data[3] << 8) | acc_data[2])) >> 4);
	hw_d[2] = (((s16) ((acc_data[5] << 8) | acc_data[4])) >> 4);

	hw_d[0] = hw_d[0] * acc->sensitivity;
	hw_d[1] = hw_d[1] * acc->sensitivity;
	hw_d[2] = hw_d[2] * acc->sensitivity;

	xyz[acc->pdata->axis_map_x] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
		   : (hw_d[acc->pdata->axis_map_x]));
	xyz[acc->pdata->axis_map_y] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
		   : (hw_d[acc->pdata->axis_map_y]));
	xyz[acc->pdata->axis_map_z] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
		   : (hw_d[acc->pdata->axis_map_z]));
	
    #if 1
	printk(KERN_INFO "%s read x=%d, y=%d, z=%d\n",
			LIS3DH_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
    #endif
	return err;
}

static int lis3dh_acc_Calibration(int count)
{
	int ret = 0;
	int i = 0;
	short xavg = 0, yavg = 0, zavg = 0;
	struct lis3dh_acc_data *acc = this_acc;
	int xyz[3] = {0};

	memset(&this_acc->accel_cali, 0, sizeof(this_acc->accel_cali));

	mdelay(100);
	for(i = 0; i < count; i++)
	{
		mdelay(20);
		ret = lis3dh_acc_read_data(acc, xyz);
    		if(ret < 0)
		{
			dev_err(&acc->client->dev, "%s: read data failed\n", __func__);
			return ret;
		}
		xavg += xyz[acc->pdata->axis_map_x];
		yavg += xyz[acc->pdata->axis_map_y];
		zavg += xyz[acc->pdata->axis_map_z];	
	}
	xavg = xavg/count;
	yavg = yavg/count;
	zavg = zavg/count;
	KMSGINF(&acc->client->dev, "%s: xavg = %d, yavg = %d, zavg = %d\n", __func__, xavg, yavg, zavg);
	if(((xavg > -CAL_FUZZ)&&(xavg < CAL_FUZZ))&&((yavg > -CAL_FUZZ)&&(yavg < CAL_FUZZ))&&
		((zavg > (CAL_CONVERT - CAL_FUZZ))&&(zavg < (CAL_CONVERT + CAL_FUZZ))))
	{
		this_acc->accel_cali[acc->pdata->axis_map_x] = xavg;
		this_acc->accel_cali[acc->pdata->axis_map_y] = yavg;
		this_acc->accel_cali[acc->pdata->axis_map_z] = zavg - CAL_CONVERT;
		KMSGINF(&acc->client->dev, "%s(9.8): success!\n", __func__);
		ret = 0;
	}
	else if(((xavg > -CAL_FUZZ)&&(xavg < CAL_FUZZ))&&((yavg > -CAL_FUZZ)&&(yavg < CAL_FUZZ))&&
		 ((zavg > (-CAL_CONVERT - CAL_FUZZ))&&(zavg < (-CAL_CONVERT + CAL_FUZZ))))

	{
		this_acc->accel_cali[acc->pdata->axis_map_x] = xavg;
		this_acc->accel_cali[acc->pdata->axis_map_y] = yavg;
		this_acc->accel_cali[acc->pdata->axis_map_z] = zavg + CAL_CONVERT;
		KMSGINF(&acc->client->dev, "%s(-9.8): success!\n", __func__);
		ret = 0;
	}
	else
	{
		KMSGINF(&acc->client->dev, "%s: failed!\n", __func__);
		ret = -1;
	}

	return ret;
}
//add end

static long gsensor_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	short flag;
	//longjiang add platform code start 20130520
	int ret = 0;
	struct lis3dh_acc_data *acc = this_acc;
	struct acc_offset acc_cal_data;
	struct acc_offset *acc_cal_ptr = &acc_cal_data;
	//add end

	KMSGINF(&acc->client->dev, "%s: cmd = %u\n", __func__, cmd);
	
	switch (cmd) {
	case GSENSOR_IOCTL_APP_SET_AFLAG:
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		if (flag < 0 || flag > 1)
			return -EINVAL;
		printk(KERN_INFO "%s: %s: flag = %s\n", LIS3DH_ACC_DEV_NAME, __func__, flag ? "Open" : "Close");
    if(this_acc!=0)
    {
       if(flag==1)
           lis3dh_acc_enable(this_acc);
       else if(flag==0)
			     lis3dh_acc_disable(this_acc);
		 }
		break;

	case GSENSOR_IOCTL_APP_SET_DELAY:
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		acc->pdata->poll_interval = flag;
	  lis3dh_acc_update_odr(acc, flag);
		break;

	//longjiang add platform code start 20130520
	case GSENSOR_IOCTL_APP_CALIBRATE:
		KMSGINF(&acc->client->dev, "%s: Calibrate start\n",  __func__);
			
		ret = lis3dh_acc_Calibration(10);
		if(ret < 0)
			dev_err(&acc->client->dev, "%s: set default offset!\n", __func__);
				
		((struct acc_offset *)acc_cal_ptr)->x = this_acc->accel_cali[acc->pdata->axis_map_x];
		((struct acc_offset *)acc_cal_ptr)->y = this_acc->accel_cali[acc->pdata->axis_map_y];
		((struct acc_offset *)acc_cal_ptr)->z = this_acc->accel_cali[acc->pdata->axis_map_z];
		((struct acc_offset *)acc_cal_ptr)->key = ret ? 2 : 1;
			
		KMSGINF(&acc->client->dev, "%s write: xoffset = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->x);
		KMSGINF(&acc->client->dev, "%s write: yoffset = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->y);
		KMSGINF(&acc->client->dev, "%s write: zoffset = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->z);
		KMSGINF(&acc->client->dev, "%s write: CalStatus = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->key);
		sensparams_write_to_flash(SENSPARAMS_TYPE_ACCEL, (unsigned char*)acc_cal_ptr, sizeof(acc_cal_data));
		if(copy_to_user((struct acc_offset *)arg, acc_cal_ptr, sizeof(acc_cal_data)))
		{
			dev_err(&acc->client->dev,  "%s:  Calibrate copy_to_user failed!\n", __func__);	
			return -EFAULT;
		}

		break;
		
	case GSENSOR_IOCTL_APP_OFFSET:

        sensparams_read_from_flash(SENSPARAMS_TYPE_ACCEL, (unsigned char*)acc_cal_ptr, sizeof(acc_cal_data));
        this_acc->accel_cali[acc->pdata->axis_map_x] = ((struct acc_offset *)acc_cal_ptr)->x;
		this_acc->accel_cali[acc->pdata->axis_map_y] = ((struct acc_offset *)acc_cal_ptr)->y;
		this_acc->accel_cali[acc->pdata->axis_map_z] = ((struct acc_offset *)acc_cal_ptr)->z;
		KMSGINF(&acc->client->dev, "%s read: xoffset = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->x);
		KMSGINF(&acc->client->dev, "%s read: yoffset = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->y);
		KMSGINF(&acc->client->dev, "%s read: zoffset = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->z);
		KMSGINF(&acc->client->dev, "%s read: CalStatus = %d\n",  __func__, ((struct acc_offset *)acc_cal_ptr)->key);
		if(copy_to_user((short *)arg, &acc_cal_ptr->key, sizeof(short)))//added by wudongxing for factory pattern.2013.11.07
		{
			KMSGINF(&acc->client->dev,  "%s:  get offset copy_to_user failed!\n", __func__);
			return -EFAULT;
		}   
		break;
	//add end
	
	default:
		break;
	
	}
    return 0;
}

//vincent
static int lis3dh_acc_input_open(struct input_dev *input)
{
	//struct lis3dh_acc_data *acc = input_get_drvdata(input);

	//return lis3dh_acc_enable(acc);
	return 0;
}

void lis3dh_acc_input_close(struct input_dev *dev)
{
	//struct lis3dh_acc_data *acc = input_get_drvdata(dev);

	//lis3dh_acc_disable(acc);
	return ;
}

static int lis3dh_acc_validate_pdata(struct lis3dh_acc_data *acc)
{
	acc->pdata->poll_interval = max(acc->pdata->poll_interval,
			acc->pdata->min_interval);

	if (acc->pdata->axis_map_x > 2 ||
		acc->pdata->axis_map_y > 2 ||
		 acc->pdata->axis_map_z > 2) {
		dev_err(&acc->client->dev, "invalid axis_map value "
			"x:%u y:%u z%u\n", acc->pdata->axis_map_x,
				acc->pdata->axis_map_y, acc->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (acc->pdata->negate_x > 1 || acc->pdata->negate_y > 1
			|| acc->pdata->negate_z > 1) {
		dev_err(&acc->client->dev, "invalid negate value "
			"x:%u y:%u z:%u\n", acc->pdata->negate_x,
				acc->pdata->negate_y, acc->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (acc->pdata->poll_interval < acc->pdata->min_interval) {
		dev_err(&acc->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lis3dh_acc_input_init(struct lis3dh_acc_data *acc)
{
	int err;

	INIT_DELAYED_WORK(&acc->input_work, lis3dh_acc_input_work_func);
	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		dev_err(&acc->client->dev, "input device allocation failed\n");
		goto err0;
	}

	acc->input_dev->open = lis3dh_acc_input_open;
	acc->input_dev->close = lis3dh_acc_input_close;
	acc->input_dev->name = GSENSOR_INPUT_NAME;//longjiang add platform code 20130520
	acc->input_dev->id.bustype = BUS_I2C;
	acc->input_dev->dev.parent = &acc->client->dev;

	input_set_drvdata(acc->input_dev, acc);

	set_bit(EV_ABS, acc->input_dev->evbit);
	/*	next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, acc->input_dev->absbit);
	/*	next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, acc->input_dev->absbit);

	input_set_abs_params(acc->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(acc->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(acc->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);
	/*	next is used for interruptA sources data if the case */
	input_set_abs_params(acc->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*	next is used for interruptB sources data if the case */
	input_set_abs_params(acc->input_dev, ABS_WHEEL, INT_MIN, INT_MAX, 0, 0);

	err = input_register_device(acc->input_dev);
	if (err) {
		dev_err(&acc->client->dev,
				"unable to register input device %s\n",
				acc->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(acc->input_dev);
err0:
	return err;
}

static void lis3dh_acc_input_cleanup(struct lis3dh_acc_data *acc)
{
	input_unregister_device(acc->input_dev);
	input_free_device(acc->input_dev);
}

static struct file_operations gsensor_fops = {
	.owner = THIS_MODULE,
	.open = gsensor_open,
	.release = gsensor_release,
	.unlocked_ioctl = gsensor_ioctl,
};

static struct miscdevice gsensor_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = GSENSOR_DEVICE_NAME,//longjiang add platform code 20130520
	.fops = &gsensor_fops,
};

/***************************************/
#ifdef CONFIG_SENSOR_POWER
static int sensors_power_init(struct lis3dh_acc_data *acc, bool on)
{
   	int rc;

    	if (!on)
        		goto pwr_deinit;

    	acc->vdd_ana = regulator_get(&acc->client->dev, "vdd_ana");
    	if (IS_ERR(acc->vdd_ana)) {
        		rc = PTR_ERR(acc->vdd_ana);
        		dev_err(&acc->client->dev,
            		"Regulator get failed vdd_ana rc=%d\n", rc);
        		return rc;
    	}

    	if (regulator_count_voltages(acc->vdd_ana) > 0) {
        		rc = regulator_set_voltage(acc->vdd_ana, 2600000,3300000);
        		if (rc) {
            		dev_err(&acc->client->dev,
                			"Regulator set_vtg failed vdd_ana rc=%d\n", rc);
            		goto reg_vdd_ana_put;
        		}
    	}

    	acc->vcc_i2c = regulator_get(&acc->client->dev, "vcc_i2c");
    	if (IS_ERR(acc->vcc_i2c)) {
        		rc = PTR_ERR(acc->vcc_i2c);
        		dev_err(&acc->client->dev,
            		"Regulator get failed vcc_i2c rc=%d\n", rc);
        		goto reg_vdd_ana_set_vtg;
    	}

    	if (regulator_count_voltages(acc->vcc_i2c) > 0) {
        		rc = regulator_set_voltage(acc->vcc_i2c, 1800000,1800000);
        		if (rc) {
            		dev_err(&acc->client->dev,
            			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
            		goto reg_vcc_i2c_put;
        		}
    	}
		
    	return 0;

reg_vcc_i2c_put:
   	regulator_put(acc->vcc_i2c);

reg_vdd_ana_set_vtg:
    	if (regulator_count_voltages(acc->vdd_ana) > 0)
        		regulator_set_voltage(acc->vdd_ana, 0, 3300000);
reg_vdd_ana_put:
    	regulator_put(acc->vdd_ana);
    	return rc;

pwr_deinit:
	if (regulator_count_voltages(acc->vdd_ana) > 0)
		regulator_set_voltage(acc->vdd_ana, 0, 3300000);
	regulator_put(acc->vdd_ana);
	
	if (regulator_count_voltages(acc->vcc_i2c) > 0)
        		regulator_set_voltage(acc->vcc_i2c, 0, 1800000);
	regulator_put(acc->vcc_i2c);
	
    	return 0;
}

static int sensors_power_on(struct lis3dh_acc_data *acc, bool on)
{
    	int rc;

    	if (!on)
        		goto power_off;

    	rc = regulator_enable(acc->vdd_ana);
    	if (rc) {
        		dev_err(&acc->client->dev,
            		"Regulator vdd_ana enable failed rc=%d\n", rc);
        		return rc;
    	}
    	rc = regulator_enable(acc->vcc_i2c);
    	if (rc) {
        		dev_err(&acc->client->dev,
            		"Regulator vcc_i2c enable failed rc=%d\n", rc);
        		regulator_disable(acc->vdd_ana);
    	}

    	return rc;

power_off:
    	rc = regulator_disable(acc->vdd_ana);
    	if (rc) {
        		dev_err(&acc->client->dev,
            		"Regulator vdd_ana disable failed rc=%d\n", rc);
        		return rc;
    	}
    	rc = regulator_disable(acc->vcc_i2c);
    	if (rc) {
        		dev_err(&acc->client->dev,
            		"Regulator vcc_i2c disable failed rc=%d\n", rc);
       		regulator_enable(acc->vdd_ana);
    	}
		
    	return rc;
}
#endif

#ifdef CONFIG_OF
/*static int lis3dh_get_dt_axis_map(struct device *dev, char *name,
				struct lis3dh_acc_platform_data *pdata)
{
	u32 axis_map[3];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int map_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;
	
	map_size = prop->length / sizeof(u32);
	if (map_size != 3) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, axis_map, map_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (strncmp(name, "lis3dh,axis_map", sizeof("lis3dh,axis_map")) == 0) {
		pdata->axis_map_x = axis_map[0];
		pdata->axis_map_y = axis_map[1];
		pdata->axis_map_z = axis_map[2];
	} else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	printk("%s: x = %d, y = %d, z = %d\n", __func__, pdata->axis_map_x, pdata->axis_map_y, pdata->axis_map_z);	

	return 0;
}

static int lis3dh_get_dt_negate(struct device *dev, char *name,
				struct lis3dh_acc_platform_data *pdata)
{
	u32 negate[3];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int negate_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;
	
	negate_size = prop->length / sizeof(u32);
	if (negate_size != 3) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, negate, negate_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (strncmp(name, "lis3dh,negate", sizeof("lis3dh,negate")) == 0) {
		pdata->negate_x = negate[0];
		pdata->negate_y = negate[1];
		pdata->negate_z = negate[2];
	} else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	printk("%s: negate_x = %d, negate_y = %d, negate_z = %d\n", __func__, pdata->negate_x, pdata->negate_y, pdata->negate_z);	

	return 0;
}*/

static int lis3dh_parse_dt(struct device *dev, struct lis3dh_acc_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	u32 temp_val;

	printk(KERN_INFO "%s: start!\n", __func__);
	rc = of_property_read_u32(np, "lis3dh,poll_interval", &temp_val);
	if (rc && (rc != -EINVAL)){
		dev_err(dev, "Unable to read poll_interval\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->poll_interval= temp_val;

	rc = of_property_read_u32(np, "lis3dh,min_interval", &temp_val);
	if (rc && (rc != -EINVAL)){
		dev_err(dev, "Unable to read min_interval\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->min_interval= temp_val;

	rc = of_property_read_u32(np, "lis3dh,g_range", &temp_val);
	if (rc && (rc != -EINVAL)){
		dev_err(dev, "Unable to read accel_irq_use_drdy\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->g_range= temp_val;

	rc = of_property_read_u32(np, "lis3dh,axis_map_x", &temp_val);
	if (rc && (rc != -EINVAL)){
		dev_err(dev, "Unable to read accel_res\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->axis_map_x= temp_val;

	rc = of_property_read_u32(np, "lis3dh,axis_map_y", &temp_val);
	if (rc && (rc != -EINVAL)){
		dev_err(dev, "Unable to read accel_direction\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->axis_map_y= temp_val;
	
	rc = of_property_read_u32(np, "lis3dh,axis_map_z", &temp_val);
	if (rc && (rc != -EINVAL)){
		dev_err(dev, "Unable to read accel_g_range\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->axis_map_z= (u8)temp_val;
	
	rc = of_property_read_u32(np, "lis3dh,negate_x", &temp_val);
	if (rc && (rc != -EINVAL)){
		dev_err(dev, "Unable to read accel_g_range\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->negate_x= (u8)temp_val;
	
	rc = of_property_read_u32(np, "lis3dh,negate_y", &temp_val);
	if (rc && (rc != -EINVAL)){
		dev_err(dev, "Unable to read accel_g_range\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->negate_y= (u8)temp_val;

	rc = of_property_read_u32(np, "lis3dh,negate_z", &temp_val);
	if (rc && (rc != -EINVAL)){
		dev_err(dev, "Unable to read accel_g_range\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->negate_z= (u8)temp_val;
		
	/*rc = of_property_read_u32(np, "lis3dh,gpio_int1", &temp_val);
	if (rc && (rc != -EINVAL)){
		dev_err(dev, "Unable to read gpio_int1\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->gpio_int1= (u8)temp_val;
	
	rc = of_property_read_u32(np, "lis3dh,gpio_int2", &temp_val);
	if (rc && (rc != -EINVAL)){
		dev_err(dev, "Unable to read gpio_int2\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->gpio_int2= (u8)temp_val;*/
  pdata->gpio_int1= -1;
  pdata->gpio_int2=-1;
	printk("%s: poll_interval = %d, min_interval = %d, g_range = %u\n", __func__, pdata->poll_interval, pdata->min_interval, pdata->g_range);	

	return 0;
}
#endif
/***************************************/

#ifdef CONFIG_YL_SENSORS_INFO
static sensor_info lis3dh_acc_info;
static int  lis3dh_acc_get_calibrated(void)
{
    struct acc_offset acc_cal_data;
	
    sensparams_read_from_flash(SENSPARAMS_TYPE_ACCEL,(unsigned char *)&acc_cal_data, sizeof(struct acc_offset));

	return acc_cal_data.key;
}
static void lis3dh_acc_set_sensor_info(struct i2c_client *client,int chip_id) 
{
    lis3dh_acc_info.calibrated = &lis3dh_acc_get_calibrated;
	lis3dh_acc_info.device_id  = client->addr;
	lis3dh_acc_info.chip_id    = chip_id;
	lis3dh_acc_info.vendor_name= "st";
	lis3dh_acc_info.sensor_name= "LIS3DH";
	sensor_set_info(SI_ACCELEROMETER,&lis3dh_acc_info);
		
}

#endif


static int lis3dh_acc_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct lis3dh_acc_data *acc;
	struct lis3dh_acc_platform_data *pdata;//longjiang add 20130620

	int err = -1;

	printk("..func:%s...probe func start..pc trace..\n",__func__);
	#ifndef CONFIG_OF
	if (client->dev.platform_data == NULL) {
	    dev_err(&client->dev, "platform data is NULL. exiting.\n");
	    err = -ENODEV;
	    goto exit_check_functionality_failed;
	}
	#endif
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
	    dev_err(&client->dev, "client not i2c capable\n");
	    err = -ENODEV;
	    goto exit_check_functionality_failed;
	}
	acc = kzalloc(sizeof(struct lis3dh_acc_data), GFP_KERNEL);
	if (acc == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for module data: "
					"%d\n", err);
		goto exit_check_functionality_failed;
	}

	mutex_init(&acc->lock);
	mutex_lock(&acc->lock);

	acc->client = client;
	i2c_set_clientdata(client, acc);
	this_acc=acc;

	#ifdef CONFIG_OF
        if(client->dev.of_node){
        	pdata = devm_kzalloc(&client->dev,
        		sizeof(struct lis3dh_acc_platform_data), GFP_KERNEL);
        	if (!pdata) {
        		dev_err(&client->dev, "Failed to allocate memory\n");
        		goto err_mutexunlock;
        	}
        	err = lis3dh_parse_dt(&client->dev, pdata);
        	if (err){
        		dev_err(&client->dev, "Failed to parse dt\n");
        		devm_kfree(&client->dev, pdata);
        		goto err_mutexunlock;
        	}
        }else{
        	pdata = client->dev.platform_data;
        	if (!pdata)
        		goto err_mutexunlock;
        }
	acc->pdata = pdata;
	#else
         acc->pdata = devm_kzalloc(&client->dev,sizeof(*acc->pdata), GFP_KERNEL);
         if (acc->pdata == NULL) {
         	err = -ENOMEM;
         	dev_err(&client->dev,
         			"failed to allocate memory for pdata: %d\n",
         			err);
         	goto err_mutexunlock;
         }
         memcpy(acc->pdata, client->dev.platform_data, sizeof(*acc->pdata));
	#endif
	err = lis3dh_acc_validate_pdata(acc);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}

	if (acc->pdata->init) {
		err = acc->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err_pdata_init;
		}
	}

	if(acc->pdata->gpio_int1 >= 0){
		acc->irq1 = gpio_to_irq(acc->pdata->gpio_int1);
		printk(KERN_INFO "%s: %s has set irq1 to irq: %d "
							"mapped on gpio:%d\n",
			LIS3DH_ACC_DEV_NAME, __func__, acc->irq1,
							acc->pdata->gpio_int1);
	}

	if(acc->pdata->gpio_int2 >= 0){
		acc->irq2 = gpio_to_irq(acc->pdata->gpio_int2);
		printk(KERN_INFO "%s: %s has set irq2 to irq: %d "
							"mapped on gpio:%d\n",
			LIS3DH_ACC_DEV_NAME, __func__, acc->irq2,
							acc->pdata->gpio_int2);
	}

	memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));

	acc->resume_state[RES_CTRL_REG1] = LIS3DH_ACC_ENABLE_ALL_AXES;
	acc->resume_state[RES_CTRL_REG2] = 0x00;
	acc->resume_state[RES_CTRL_REG3] = 0x00;
	acc->resume_state[RES_CTRL_REG4] = 0x00;
	acc->resume_state[RES_CTRL_REG5] = 0x00;
	acc->resume_state[RES_CTRL_REG6] = 0x00;

	acc->resume_state[RES_TEMP_CFG_REG] = 0x00;
	acc->resume_state[RES_FIFO_CTRL_REG] = 0x00;
	acc->resume_state[RES_INT_CFG1] = 0x00;
	acc->resume_state[RES_INT_THS1] = 0x00;
	acc->resume_state[RES_INT_DUR1] = 0x00;

	acc->resume_state[RES_TT_CFG] = 0x00;
	acc->resume_state[RES_TT_THS] = 0x00;
	acc->resume_state[RES_TT_LIM] = 0x00;
	acc->resume_state[RES_TT_TLAT] = 0x00;
	acc->resume_state[RES_TT_TW] = 0x00;

	/***************************************/
	  #ifdef CONFIG_SENSOR_POWER
	sensors_power_init(this_acc, true);
	sensors_power_on(this_acc, true);
	msleep(10);
  #endif
	/***************************************/

	err = lis3dh_acc_device_power_on(acc);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	atomic_set(&acc->enabled, 1); //vincent

	err = lis3dh_acc_update_g_range(acc, acc->pdata->g_range);
	if (err < 0) {
		dev_err(&client->dev, "update_g_range failed\n");
		goto  err_power_off;
	}

	err = lis3dh_acc_update_odr(acc, acc->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto  err_power_off;
	}

	err = lis3dh_acc_input_init(acc);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto err_power_off;
	}

	err = create_sysfs_interfaces(&client->dev);
	if (err < 0) {
		dev_err(&client->dev,
		   "device LIS3DH_ACC_DEV_NAME sysfs register failed\n");
		goto err_input_cleanup;
	}
	//lis3dh_acc_device_power_off(acc);

	/* As default, do not report information */
	atomic_set(&acc->enabled, 0);

	err= misc_register(&gsensor_device);
	if (err < 0) {
		dev_err(&client->dev, "Failed to register misc device\n");
	    	goto err_remove_sysfs_int;
	}

	if(acc->pdata->gpio_int1 >= 0){
		INIT_WORK(&acc->irq1_work, lis3dh_acc_irq1_work_func);
		acc->irq1_work_queue =
			create_singlethread_workqueue("lis3dh_acc_wq1");
		if (!acc->irq1_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
					"cannot create work queue1: %d\n", err);
			goto err_remove_sysfs_int;
		}
		err = request_irq(acc->irq1, lis3dh_acc_isr1,
				IRQF_TRIGGER_RISING, "lis3dh_acc_irq1", acc);
		if (err < 0) {
			dev_err(&client->dev, "request irq1 failed: %d\n", err);
			goto err_destoyworkqueue1;
		}
		disable_irq_nosync(acc->irq1);
	}

	if(acc->pdata->gpio_int2 >= 0){
		INIT_WORK(&acc->irq2_work, lis3dh_acc_irq2_work_func);
		acc->irq2_work_queue =
			create_singlethread_workqueue("lis3dh_acc_wq2");
		if (!acc->irq2_work_queue) {
			err = -ENOMEM;
			dev_err(&client->dev,
					"cannot create work queue2: %d\n", err);
			goto err_free_irq1;
		}
		err = request_irq(acc->irq2, lis3dh_acc_isr2,
				IRQF_TRIGGER_RISING, "lis3dh_acc_irq2", acc);
		if (err < 0) {
			dev_err(&client->dev, "request irq2 failed: %d\n", err);
			goto err_destoyworkqueue2;
		}
		disable_irq_nosync(acc->irq2);
	}

	mutex_unlock(&acc->lock);
	
#ifdef CONFIG_YL_SENSORS_INFO
	lis3dh_acc_set_sensor_info(client,WHOAMI_LIS3DH_ACC);
#endif

    printk("..func:%s...probe func success..pc trace..\n",__func__);
	return 0;

err_destoyworkqueue2:
	if(acc->pdata->gpio_int2 >= 0)
		destroy_workqueue(acc->irq2_work_queue);
err_free_irq1:
	free_irq(acc->irq1, acc);
err_destoyworkqueue1:
	if(acc->pdata->gpio_int1 >= 0)
		destroy_workqueue(acc->irq1_work_queue);
err_remove_sysfs_int:
	remove_sysfs_interfaces(&client->dev);
err_input_cleanup:
	lis3dh_acc_input_cleanup(acc);
err_power_off:
	lis3dh_acc_device_power_off(acc);
err_pdata_init:
	if (acc->pdata->exit)
		acc->pdata->exit();
exit_kfree_pdata:
	devm_kfree(&client->dev, acc->pdata);
err_mutexunlock:
	mutex_unlock(&acc->lock);
//err_freedata:
	kfree(acc);
exit_check_functionality_failed:
	printk(KERN_ERR "%s: Driver Init failed\n", LIS3DH_ACC_DEV_NAME);
	return err;
}

static int __devexit lis3dh_acc_remove(struct i2c_client *client)
{
	struct lis3dh_acc_data *acc = i2c_get_clientdata(client);

	if(acc->pdata->gpio_int1 >= 0){
		free_irq(acc->irq1, acc);
		gpio_free(acc->pdata->gpio_int1);
		destroy_workqueue(acc->irq1_work_queue);
	}

	if(acc->pdata->gpio_int2 >= 0){
		free_irq(acc->irq2, acc);
		gpio_free(acc->pdata->gpio_int2);
		destroy_workqueue(acc->irq2_work_queue);
	}

	lis3dh_acc_input_cleanup(acc);
	lis3dh_acc_device_power_off(acc);
	remove_sysfs_interfaces(&client->dev);

	if (acc->pdata->exit)
		acc->pdata->exit();
	kfree(acc->pdata);
	kfree(acc);

	return 0;
}

#ifdef CONFIG_PM
static int lis3dh_acc_resume(struct i2c_client *client)
{
	struct lis3dh_acc_data *acc = i2c_get_clientdata(client);

	if (acc->on_before_suspend)
		return lis3dh_acc_enable(acc);
	return 0;
}

static int lis3dh_acc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct lis3dh_acc_data *acc = i2c_get_clientdata(client);

	acc->on_before_suspend = atomic_read(&acc->enabled);
	if (1==acc->on_before_suspend)
		return lis3dh_acc_disable(acc);
	return 0;
}
#else
#define lis3dh_acc_suspend	NULL
#define lis3dh_acc_resume	NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id lis3dh_acc_id[]
		= { { LIS3DH_ACC_DEV_NAME, 0 }, { }, };

MODULE_DEVICE_TABLE(i2c, lis3dh_acc_id);

/***************************************/
//longjiang add 20130620
#ifdef CONFIG_OF
static const struct of_device_id lis3dh_of_match[] = {
	{ .compatible = "lis3dh,lis3dhtr", },
	{ }
};
MODULE_DEVICE_TABLE(of, lis3dh_of_match);
#endif
//add end
/***************************************/

static struct i2c_driver lis3dh_acc_driver = {
	.driver = {
			.owner = THIS_MODULE,
			.name = LIS3DH_ACC_DEV_NAME,
			/***************************************/
			//longjiang add 20130620
			#ifdef CONFIG_OF
			.of_match_table = lis3dh_of_match,
			#endif
			//add end
			/***************************************/
		  },
	.probe = lis3dh_acc_probe,
	.remove = __devexit_p(lis3dh_acc_remove),
	.suspend = lis3dh_acc_suspend,
	.resume = lis3dh_acc_resume,
	.id_table = lis3dh_acc_id,
};

static int __init lis3dh_acc_init(void)
{
	printk(KERN_INFO "%s: start!\n", __func__);
	return i2c_add_driver(&lis3dh_acc_driver);
}

static void __exit lis3dh_acc_exit(void)
{
	i2c_del_driver(&lis3dh_acc_driver);
	return;
}

module_init(lis3dh_acc_init);
module_exit(lis3dh_acc_exit);

MODULE_DESCRIPTION("lis3dh digital accelerometer sysfs driver");
MODULE_AUTHOR("Matteo Dameno, Carmine Iascone, STMicroelectronics");
MODULE_LICENSE("GPL");

