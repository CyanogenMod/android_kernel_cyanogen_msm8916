/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "msm_sensor.h"

#ifdef CDBG
#undef CDBG
#endif
#ifdef CDBG_HIGH
#undef CDBG_HIGH
#endif

//#define OV5648_DEBUG
#ifdef OV5648_DEBUG
#define CDBG(fmt, args...) pr_info(fmt, ##args)
#define CDBG_HIGH(fmt, args...) pr_info(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#define CDBG_HIGH(fmt, args...) pr_info(fmt, ##args)
#endif

#define OV5648_SENSOR_NAME "ov5648"
#ifdef CONFIG_MACH_YULONG
#define Module_OFILM 7
#define Module_TECH  6
#endif

DEFINE_MSM_MUTEX(ov5648_mut);

static struct msm_sensor_ctrl_t ov5648_s_ctrl;

#ifdef CONFIG_MACH_YULONG
static int first_time_power_up_ov5648 = 1;
#endif

static struct msm_sensor_power_setting ov5648_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 10,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info ov5648_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id ov5648_i2c_id[] = {
	{OV5648_SENSOR_NAME,
		(kernel_ulong_t)&ov5648_s_ctrl},
	{ }
};

static int32_t msm_ov5648_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ov5648_s_ctrl);
}

static struct i2c_driver ov5648_i2c_driver = {
	.id_table = ov5648_i2c_id,
	.probe  = msm_ov5648_i2c_probe,
	.driver = {
		.name = OV5648_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov5648_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

#ifdef CONFIG_MACH_YULONG
int32_t ov5648_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc;
	if (first_time_power_up_ov5648 &&
			msm_sensor_is_probed(s_ctrl->sensordata->sensor_info->position)) {
		CDBG("do not probe ov5648,just return\n");
		return -1;
	}

	rc = msm_sensor_power_up(s_ctrl);
	if (!rc) {
		CDBG("ov5648 probe ok,set first_time_power_up_ov5648 to 0.\n");
		first_time_power_up_ov5648 = 0;
	}
	return rc;
}

int32_t ov5648_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc;
	unsigned short rdata;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x3018, &rdata,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s:read status failed of reg(0x3018) is %d\n", __func__, rc);
	}

	rdata |= 0x18;

	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3018, rdata,
		MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s:write status failed of reg(0x3018) is %d\n", __func__, rc);
	}

	msleep(40);
	return msm_sensor_power_down(s_ctrl);
}

struct otp_struct {
	uint8_t module_integrator_id;
	uint8_t lens_id;
	uint16_t rg_ratio;
	uint16_t bg_ratio;
	uint16_t user_data[2];
	uint16_t light_rg;
	uint16_t light_bg;
};

int check_otp(struct msm_sensor_ctrl_t *s_ctrl, int index)
{
	uint16_t flag;
	int  i;
	uint16_t rg, bg;
	if (index == 1)	{
		// read otp --Bank 0
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d84, 0xc0,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d85, 0x00,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d86, 0x0f,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d81, 0x01,MSM_CAMERA_I2C_BYTE_DATA);
		msleep(5);

		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d05, &flag,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d07, &rg,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d08, &bg,MSM_CAMERA_I2C_BYTE_DATA);
	}else if (index == 2) {
		// read otp --Bank 0
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d84, 0xc0,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d85, 0x00,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d86, 0x0f,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d81, 0x01,MSM_CAMERA_I2C_BYTE_DATA);
		msleep(5);

		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d0e, &flag,MSM_CAMERA_I2C_BYTE_DATA);
		// read otp --Bank 1
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d84, 0xc0,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d85, 0x10,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d86, 0x1f,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d81, 0x01,MSM_CAMERA_I2C_BYTE_DATA);
		msleep(5);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d00, &rg,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d01, &bg,MSM_CAMERA_I2C_BYTE_DATA);
	}else if (index == 3) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d84, 0xc0,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d85, 0x10,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d86, 0x1f,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d81, 0x01,MSM_CAMERA_I2C_BYTE_DATA);
		msleep(5);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d07, &flag,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d09, &rg,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d0a, &bg,MSM_CAMERA_I2C_BYTE_DATA);
	}
	flag = flag & 0x80;
	// clear otp buffer
	for (i=0;i<16;i++) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,(0x3d00 + i), 0X00,MSM_CAMERA_I2C_BYTE_DATA);
	}
	printk(KERN_ERR"YULONG:feiyc:%s,flag=%d,rg=%d,gb=%d\n",__func__,flag,rg,bg);
	if (flag) {
		return 1;
	} else {
		if (rg == 0 && bg == 0) {
			return 0;
		} else {
			return 2;
		}
	}
}

// index: index of otp group. (1, 2, 3)
// return:0,
int read_otp(struct msm_sensor_ctrl_t *s_ctrl,int index, struct otp_struct *otp_ptr)
{
	int i;
	uint16_t temp;
	uint16_t temp1;

	// read otp into buffer
	if (index == 1) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d84, 0xc0,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d85, 0x00,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d86, 0x0f,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d81, 0x01,MSM_CAMERA_I2C_BYTE_DATA);
		msleep(5);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d05, &temp1 ,MSM_CAMERA_I2C_BYTE_DATA);
		otp_ptr->module_integrator_id = temp1  & 0x7f;
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d06, &temp1 ,MSM_CAMERA_I2C_BYTE_DATA);
		otp_ptr->lens_id = temp1;
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d0b, &temp ,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d07, &temp1 ,MSM_CAMERA_I2C_BYTE_DATA);
		otp_ptr->rg_ratio =(temp1<<2) |  (temp>>6);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d08, &temp1 ,MSM_CAMERA_I2C_BYTE_DATA);
		otp_ptr->bg_ratio =(temp1<<2) | ((temp>>4) & 0x03);
		otp_ptr->light_rg =0x00;
		otp_ptr->light_bg =0x00;
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d09, &temp1 ,MSM_CAMERA_I2C_BYTE_DATA);
		otp_ptr->user_data[0] = temp1;
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d0a, &temp1 ,MSM_CAMERA_I2C_BYTE_DATA);
		otp_ptr->user_data[1] = temp1;
	} else if (index == 2) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d84, 0xc0,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d85, 0x00,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d86, 0x0f,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d81, 0x01,MSM_CAMERA_I2C_BYTE_DATA);
		msleep(5);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d0e, &temp1 ,MSM_CAMERA_I2C_BYTE_DATA);
		otp_ptr->module_integrator_id = temp1  & 0x7f;
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d0f, &temp1 ,MSM_CAMERA_I2C_BYTE_DATA);
		otp_ptr->lens_id = temp1;
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d84, 0xc0,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d85, 0x10,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d86, 0x1f,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d81, 0x01,MSM_CAMERA_I2C_BYTE_DATA);
		msleep(5);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d04, &temp ,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d00, &temp1 ,MSM_CAMERA_I2C_BYTE_DATA);
		otp_ptr->rg_ratio =(temp1<<2) |  (temp>>6);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d01, &temp1 ,MSM_CAMERA_I2C_BYTE_DATA);
		otp_ptr->bg_ratio =(temp1<<2) | ((temp>>4) & 0x03);
		otp_ptr->light_rg =0x00;
		otp_ptr->light_bg =0x00;
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d02, &temp1 ,MSM_CAMERA_I2C_BYTE_DATA);
		otp_ptr->user_data[0] = temp1;
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d03, &temp1 ,MSM_CAMERA_I2C_BYTE_DATA);
		otp_ptr->user_data[1] = temp1;
	}else if (index == 3){
		// read otp --Bank 1
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d84, 0xc0,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d85, 0x10,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d86, 0x1f,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x3d81, 0x01,MSM_CAMERA_I2C_BYTE_DATA);
		msleep(5);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d07, &temp1 ,MSM_CAMERA_I2C_BYTE_DATA);
		otp_ptr->module_integrator_id = temp1  & 0x7f;
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d08, &temp1 ,MSM_CAMERA_I2C_BYTE_DATA);
		otp_ptr->lens_id = temp1;
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d0d, &temp ,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d09, &temp1 ,MSM_CAMERA_I2C_BYTE_DATA);
		otp_ptr->rg_ratio =(temp1<<2) |  (temp>>6);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d0a, &temp1 ,MSM_CAMERA_I2C_BYTE_DATA);
		otp_ptr->bg_ratio =(temp1<<2) | ((temp>>4) & 0x03);
		otp_ptr->light_rg =0x00;
		otp_ptr->light_bg =0x00;
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d0b, &temp1 ,MSM_CAMERA_I2C_BYTE_DATA);
		otp_ptr->user_data[0] = temp1;
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x3d0c, &temp1 ,MSM_CAMERA_I2C_BYTE_DATA);
		otp_ptr->user_data[1] = temp1;
	}

	// clear otp buffer
	for (i=0; i<16; i++) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,(0x3d00 + i), 0X00,MSM_CAMERA_I2C_BYTE_DATA);
	}
	return 0;
}

// R_gain, sensor red gain of AWB, 0x400 =1
// G_gain, sensor green gain of AWB, 0x400 =1
// B_gain, sensor blue gain of AWB, 0x400 =1
// return 0;
static int update_awb_gain(struct msm_sensor_ctrl_t *s_ctrl,uint16_t R_gain, uint16_t G_gain, uint16_t B_gain)
{
	if (R_gain>0x400) {

		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x5186, R_gain>>8,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x5187, R_gain & 0x00ff,MSM_CAMERA_I2C_BYTE_DATA);
	}
	if (G_gain>0x400) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x5188, G_gain>>8,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x5189, G_gain & 0x00ff,MSM_CAMERA_I2C_BYTE_DATA);
	}
	if (B_gain>0x400) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x518a, B_gain>>8,MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,0x518b, B_gain & 0x00ff,MSM_CAMERA_I2C_BYTE_DATA);
	}
	return 0;
}

// call this function after OV5648 initialization
// return:0 update success
// 1, no OTP
int update_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct otp_struct current_otp;
	int i;
	int otp_index = 0;
	int temp;
	uint16_t R_gain, G_gain, B_gain, G_gain_R, G_gain_B;
	uint16_t rg,bg;
	uint16_t RG_Ratio_Typical ;
	uint16_t BG_Ratio_Typical ;

	// R/G and B/G of current camera module is read out from sensor OTP
	// check first OTP with valid data
	for(i=1;i<=3;i++) {
		temp = check_otp(s_ctrl,i);
		if (temp == 2) {
			otp_index = i;
			break;
		}
	}
	if (i>3) {
		printk("otp no valid otp data\n");
		// no valid wb OTP data
		return 1;
	}

	read_otp(s_ctrl,otp_index, &current_otp);

	CDBG("module_integrator_id : %d\n",current_otp.module_integrator_id);
	CDBG("lens_id : %d\n",current_otp.lens_id);
	CDBG("rg_ratio : 0x%x\n",current_otp.rg_ratio);
	CDBG("bg_ratio : 0x%x\n",current_otp.bg_ratio);
	CDBG("user_data[0] : %d\n",current_otp.user_data[0]);
	CDBG("user_data[1] : %d\n",current_otp.user_data[1]);
	CDBG("light_rg : %d\n",current_otp.light_rg);
	CDBG("light_bg : %d\n",current_otp.light_bg);

	//add by muyuezhong for module compatibility.
	//zq modify
	if(!(s_ctrl->module_id & 0xFF))
		s_ctrl->module_id |= current_otp.module_integrator_id;
	CDBG("YULONG:feiyc:%s,s_ctrl->module_id=0x%x\n",__func__,s_ctrl->module_id);

	//add by muyuezhong for otp compatibility.
	switch(current_otp.module_integrator_id)
	{
		case Module_OFILM:
			RG_Ratio_Typical = 0x1A7;
			BG_Ratio_Typical = 0x156;
			CDBG_HIGH("Module is ofilm\n");
			break;
		case Module_TECH:
			RG_Ratio_Typical = 0x19A;
			BG_Ratio_Typical = 0x138;
			CDBG_HIGH("Module is tech\n");
			break;
		default:
			RG_Ratio_Typical = 0x172;
			BG_Ratio_Typical = 0x179;
			CDBG_HIGH("Module is not defined\n");
			break;
	}

	if (current_otp.light_rg==0) {
		// no light source information in OTP
		rg = current_otp.rg_ratio;
	} else {
		// light source information found in OTP
		rg = current_otp.rg_ratio * (current_otp.light_rg +512) / 1024;
	}

	if (current_otp.light_bg==0) {
		// no light source information in OTP
		bg = current_otp.bg_ratio;
	} else {
		// light source information found in OTP
		bg = current_otp.bg_ratio * (current_otp.light_bg +512) / 1024;
	}

	//calculate G gain
	//0x400 = 1x gain
	CDBG_HIGH("rg = 0x%x,RG_Ratio_Typical = 0x%x\n",rg,RG_Ratio_Typical);
	CDBG_HIGH("bg = 0x%x,BG_Ratio_Typical = 0x%x\n",bg,BG_Ratio_Typical);

	if (bg < BG_Ratio_Typical) {
		if (rg< RG_Ratio_Typical) {
			// current_otp.bg_ratio < BG_Ratio_typical &&
			// current_otp.rg_ratio < RG_Ratio_typical
			G_gain = 0x400;
			B_gain = 0x400 * BG_Ratio_Typical / bg;
			R_gain = 0x400 * RG_Ratio_Typical / rg;
		} else {
			// current_otp.bg_ratio < BG_Ratio_typical &&
			// current_otp.rg_ratio >= RG_Ratio_typical
			R_gain = 0x400;
			G_gain = 0x400 * rg / RG_Ratio_Typical;
			B_gain = G_gain * BG_Ratio_Typical /bg;
		}
	} else {
		if (rg < RG_Ratio_Typical) {
			// current_otp.bg_ratio >= BG_Ratio_typical &&
			// current_otp.rg_ratio < RG_Ratio_typical
			B_gain = 0x400;
			G_gain = 0x400 * bg / BG_Ratio_Typical;
			R_gain = G_gain * RG_Ratio_Typical / rg;
		} else {
			// current_otp.bg_ratio >= BG_Ratio_typical &&
			// current_otp.rg_ratio >= RG_Ratio_typical
			G_gain_B = 0x400 * bg / BG_Ratio_Typical;
			G_gain_R = 0x400 * rg / RG_Ratio_Typical;
			if (G_gain_B > G_gain_R ) {
				B_gain = 0x400;
				G_gain = G_gain_B;
				R_gain = G_gain * RG_Ratio_Typical /rg;
			} else {
				R_gain = 0x400;
				G_gain = G_gain_R;
				B_gain = G_gain * BG_Ratio_Typical / bg;
			}
		}
	}

	CDBG("R_gain = 0x%x, G_gain = 0x%x, B_gain = 0x%x\n",R_gain, G_gain, B_gain);
	update_awb_gain(s_ctrl,R_gain, G_gain, B_gain);
	return 0;
}

int ov5648_update_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
	uint16_t data;

	CDBG("enter ov5648_update_otp\n");
	update_otp(s_ctrl);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x5186, &data,MSM_CAMERA_I2C_BYTE_DATA);
	CDBG("otp data : 0x%x\n",data);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x5187, &data,MSM_CAMERA_I2C_BYTE_DATA);
	CDBG("otp data : 0x%x\n",data);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x5188, &data,MSM_CAMERA_I2C_BYTE_DATA);
	CDBG("otp data : 0x%x\n",data);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x5189, &data,MSM_CAMERA_I2C_BYTE_DATA);
	CDBG("otp data : 0x%x\n",data);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x518a, &data,MSM_CAMERA_I2C_BYTE_DATA);
	CDBG("otp data : 0x%x\n",data);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,0x518b, &data,MSM_CAMERA_I2C_BYTE_DATA);
	CDBG("otp data : 0x%x\n",data);
	CDBG("exit ov5648_update_otp\n");
	return 0;
}

static struct msm_sensor_fn_t ov5648_sensor_fn_t = {
#ifdef CONFIG_COMPAT
	.sensor_config32 = msm_sensor_config32,
#endif
	.sensor_config = msm_sensor_config,
	.sensor_power_up = ov5648_power_up,
	.sensor_power_down = ov5648_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
	.sensor_update_otp = ov5648_update_otp,
	.sensor_prepare_otp = ov5648_update_otp,
};
#endif

static struct msm_sensor_ctrl_t ov5648_s_ctrl = {
	.sensor_i2c_client = &ov5648_sensor_i2c_client,
	.power_setting_array.power_setting = ov5648_power_setting,
	.power_setting_array.size =
			ARRAY_SIZE(ov5648_power_setting),
	.msm_sensor_mutex = &ov5648_mut,
	.sensor_v4l2_subdev_info = ov5648_subdev_info,
	.sensor_v4l2_subdev_info_size =
			ARRAY_SIZE(ov5648_subdev_info),
#ifdef CONFIG_MACH_YULONG
	.func_tbl = &ov5648_sensor_fn_t,
#endif
};

static const struct of_device_id ov5648_dt_match[] = {
	{
		.compatible = "ovti,ov5648",
		.data = &ov5648_s_ctrl
	},
	{}
};

MODULE_DEVICE_TABLE(of, ov5648_dt_match);

static struct platform_driver ov5648_platform_driver = {
	.driver = {
		.name = "ovti,ov5648",
		.owner = THIS_MODULE,
		.of_match_table = ov5648_dt_match,
	},
};

static int32_t ov5648_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;

	match = of_match_device(ov5648_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init ov5648_init_module(void)
{
	int32_t rc = 0;

	rc = platform_driver_probe(&ov5648_platform_driver,
		ov5648_platform_probe);
	if (!rc)
		return rc;
	return i2c_add_driver(&ov5648_i2c_driver);
}

static void __exit ov5648_exit_module(void)
{
	if (ov5648_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov5648_s_ctrl);
		platform_driver_unregister(&ov5648_platform_driver);
	} else
		i2c_del_driver(&ov5648_i2c_driver);
	return;
}

module_init(ov5648_init_module);
module_exit(ov5648_exit_module);
MODULE_DESCRIPTION("ov5648");
MODULE_LICENSE("GPL v2");
