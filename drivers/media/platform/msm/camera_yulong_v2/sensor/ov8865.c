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
#include "msm_cci.h"
#define OV8865_SENSOR_NAME "ov8865"
DEFINE_MSM_MUTEX(ov8865_mut);

static struct msm_sensor_ctrl_t ov8865_s_ctrl;

static int first_time_power_up_ov8865 = 1;

#if defined (CONFIG_BOARD_CP9191) || defined (CONFIG_BOARD_CP9191_T00)
static struct msm_sensor_power_setting ov8865_power_setting[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 2,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 2,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 10,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 2,
		.use_system_power = 1,
	},
};
#else
static struct msm_sensor_power_setting ov8865_power_setting[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 2,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 2,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 10,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 2,
		.use_system_power = 1,
	},
};
#endif

#if defined (CONFIG_BOARD_CP9191) || defined (CONFIG_BOARD_CP9191_T00)
static struct msm_sensor_power_setting ov8865_power_down_setting[] = {
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 2,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 0,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 2,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDIG,	//DVDD
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,		//DOVDD
		.config_val = 0,
		.delay = 0,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,	//AVDD
		.config_val = GPIO_OUT_LOW,
		.delay = 2,
		.use_system_power = 1,
	},
};
#else
static struct msm_sensor_power_setting ov8865_power_down_setting[] = {
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 2,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 0,
		.delay = 0,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 2,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDIG,	//DVDD
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,		//DOVDD
		.config_val = 0,
		.delay = 0,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,	//AVDD
		.config_val = GPIO_OUT_LOW,
		.delay = 2,
		.use_system_power = 1,
	},
};
#endif

static struct v4l2_subdev_info ov8865_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id ov8865_i2c_id[] = {
	{OV8865_SENSOR_NAME, (kernel_ulong_t)&ov8865_s_ctrl},
	{ }
};

static int32_t msm_ov8865_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ov8865_s_ctrl);
}

static struct i2c_driver ov8865_i2c_driver = {
	.id_table = ov8865_i2c_id,
	.probe  = msm_ov8865_i2c_probe,
	.driver = {
		.name = OV8865_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov8865_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id ov8865_dt_match[] = {
	{.compatible = "qcom,ov8865", .data = &ov8865_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov8865_dt_match);

int32_t ov8865_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
     int32_t rc;
     if (first_time_power_up_ov8865 && frontCameraProbeInfo()) {
	 	//printk("do not probe ov8865,just return\n");
        rc = -1;
        return rc;
	}
	rc = msm_sensor_power_up(s_ctrl);
    if (0 == rc) {
       //printk("ov8865 probe ok,set first_time_power_up_ov8865 to 0.\n");
	   first_time_power_up_ov8865 = 0;
    }
	return rc;
}

struct otp_struct {
	uint16_t module_integrator_id;
	uint16_t lens_id;
	uint16_t production_year;
	uint16_t production_month;
	uint16_t production_day;
	uint16_t rg_ratio;
	uint16_t bg_ratio;
	uint16_t light_rg;
	uint16_t light_bg;
	uint16_t lenc[62];
	uint16_t VCM_start;
	uint16_t VCM_end;
	uint16_t VCM_dir;
};

static struct otp_struct current_otp;
static int R_gain, G_gain, B_gain;

#if defined(CONFIG_BOARD_CP8730L) || defined (CONFIG_BOARD_CP8729)
int RG_Ratio_Typical = 0x11E;
int BG_Ratio_Typical = 0x11C;
#else
int RG_Ratio_Typical = 0x11A;
int BG_Ratio_Typical = 0x11E;
#endif


// index: index of otp group. (1, 2, 3)
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
int check_otp_info(struct msm_sensor_ctrl_t *s_ctrl, int index)
{
	uint16_t flag;
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d84, 0xC0, MSM_CAMERA_I2C_BYTE_DATA);
	//partial mode OTP write start address
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d88, 0x70, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d89, 0x10, MSM_CAMERA_I2C_BYTE_DATA);
	// partial mode OTP write end address
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d8A, 0x70, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d8B, 0x10, MSM_CAMERA_I2C_BYTE_DATA);
	// read otp into buffer
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d81, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	msleep(5);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x7010, &flag, MSM_CAMERA_I2C_BYTE_DATA);

	//select group
	if (index == 1) {
		flag = (flag>>6) & 0x03;
	} else if (index == 2) {
		flag = (flag>>4) & 0x03;
	} else if (index ==3) {
		flag = (flag>>2) & 0x03;
	}

	//printk("%s:index = %d,flag = %d\n", __func__, index, (int)flag);

	// clear otp buffer
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x7010, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	if (flag == 0x00) {
		return 0;
	} else if (flag & 0x02) {
		return 1;
	} else {
		return 2;
	}
}

// index: index of otp group. (1, 2, 3)
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
int check_otp_wb(struct msm_sensor_ctrl_t *s_ctrl, int index)
{
	uint16_t flag;
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d84, 0xC0, MSM_CAMERA_I2C_BYTE_DATA);
	//partial mode OTP write start address
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d88, 0x70, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d89, 0x20, MSM_CAMERA_I2C_BYTE_DATA);
	// partial mode OTP write end address
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d8A, 0x70, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d8B, 0x20, MSM_CAMERA_I2C_BYTE_DATA);
	// read otp into buffer
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d81, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	msleep(5);
	//select group
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x7020, &flag, MSM_CAMERA_I2C_BYTE_DATA);

	if (index == 1)	{
		flag = (flag>>6) & 0x03;
	} else if (index == 2) {
		flag = (flag>>4) & 0x03;
	} else if (index == 3) {
		flag = (flag>>2) & 0x03;
	}

	//printk("%s:index = %d,flag = %d\n", __func__, index, (int)flag);

	// clear otp buffer
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x7020, 0x00, MSM_CAMERA_I2C_BYTE_DATA);

	if (flag == 0x00) {
		return 0;
	} else if (flag & 0x02) {
		return 1;
	} else {
		return 2;
	}
}

// index: index of otp group. (1, 2, 3)
// code: 0 for start code, 1 for stop code
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
int check_otp_VCM(struct msm_sensor_ctrl_t *s_ctrl, int index, int code)
{
	uint16_t flag;
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d84, 0xC0, MSM_CAMERA_I2C_BYTE_DATA);
	//partial mode OTP write start address
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d88, 0x70, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d89, 0x30, MSM_CAMERA_I2C_BYTE_DATA);
	// partial mode OTP write end address
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d8A, 0x70, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d8B, 0x30, MSM_CAMERA_I2C_BYTE_DATA);
	// read otp into buffer
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d81, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	msleep(5);
	//select group
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x7030, &flag, MSM_CAMERA_I2C_BYTE_DATA);

	if (index == 1) {
		flag = (flag>>6) & 0x03;
	} else if (index == 2) {
		flag = (flag>>4) & 0x03;
	} else if (index == 3) {
		flag = (flag>>2) & 0x03;
	}

	//printk("%s:index = %d,flag = %d, \n", __func__, index, (int)flag);

	// clear otp buffer
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x7030, 0x00, MSM_CAMERA_I2C_BYTE_DATA);

	if (flag == 0x00) {
		return 0;
	} else if (flag & 0x02) {
		return 1;
	} else {
		return 2;
	}
}

// index: index of otp group. (1, 2, 3)
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
int check_otp_lenc(struct msm_sensor_ctrl_t *s_ctrl, int index)
{
	uint16_t flag;

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d84, 0xC0, MSM_CAMERA_I2C_BYTE_DATA);
	//partial mode OTP write start address
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d88, 0x70, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d89, 0x3A, MSM_CAMERA_I2C_BYTE_DATA);
	// partial mode OTP write end address
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d8A, 0x70, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d8B, 0x3A, MSM_CAMERA_I2C_BYTE_DATA);
	// read otp into buffer
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d81, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	msleep(5);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x703a, &flag, MSM_CAMERA_I2C_BYTE_DATA);

	if (index == 1)	{
		flag = (flag>>6) & 0x03;
	} else if (index == 2) {
		flag = (flag>>4) & 0x03;
	} else if (index == 3) {
		flag = (flag>> 2)& 0x03;
	}

	//printk("%s:index = %d,flag = %d, \n", __func__, index, (int)flag);

	// clear otp buffer
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x703a, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	if (flag == 0x00) {
		return 0;
	} else if (flag & 0x02) {
		return 1;
	} else {
		return 2;
	}
}

// index: index of otp group. (1, 2, 3)
// otp_ptr: pointer of otp_struct
// return: 0,
int read_otp_info(struct msm_sensor_ctrl_t *s_ctrl, int index, struct otp_struct *otp_ptr)
{
	int i;
	int start_addr = 0, end_addr = 0;

	//printk("%s:the index of reading is:%d\n", __FUNCTION__, index);

	if (index == 1) {
		start_addr = 0x7011;
		end_addr = 0x7015;
	}
	else if (index == 2) {
		start_addr = 0x7016;
		end_addr = 0x701a;
	}
	else if (index == 3) {
		start_addr = 0x701b;
		end_addr = 0x701f;
	}

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d84, 0xC0, MSM_CAMERA_I2C_BYTE_DATA);
	//partial mode OTP write start address
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d88, (start_addr >> 8) & 0xff, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d89, start_addr & 0xff, MSM_CAMERA_I2C_BYTE_DATA);
	// partial mode OTP write end address
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d8A, (end_addr >> 8) & 0xff, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d8B, end_addr & 0xff, MSM_CAMERA_I2C_BYTE_DATA);
	// read otp into buffer
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d81, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	msleep(5);

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, start_addr, &((*otp_ptr).module_integrator_id), MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, start_addr + 1, &((*otp_ptr).lens_id), MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, start_addr + 2, &((*otp_ptr).production_year), MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, start_addr + 3, &((*otp_ptr).production_month), MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, start_addr + 4, &((*otp_ptr).production_day), MSM_CAMERA_I2C_BYTE_DATA);

	printk("%s:module_integrator_id=%d,lens_id=%d,production_year=%d,production_month=%d,production_day=%d\n",__func__, (int)((*otp_ptr).module_integrator_id),(int)((*otp_ptr).lens_id), (int)((*otp_ptr).production_year), (int)((*otp_ptr).production_month), (int)((*otp_ptr).production_day));

	// clear otp buffer
	for (i = start_addr; i <= end_addr; i++) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, i, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	}

	return 0;
}

// index: index of otp group. (1, 2, 3)
// otp_ptr: pointer of otp_struct
// return: 0,
int read_otp_wb(struct msm_sensor_ctrl_t *s_ctrl, int index, struct otp_struct *otp_ptr)
{
	int i;
	uint16_t temp;
	uint16_t rg_ratio_temp = 0, bg_ratio_temp = 0, light_rg_temp = 0, light_bg_temp = 0;
	int start_addr = 0, end_addr = 0;

	//printk("%s:the index of reading is:%d\n", __FUNCTION__, index);

	if (index == 1) {
		start_addr = 0x7021;
		end_addr = 0x7025;
	} else if (index == 2) {
		start_addr = 0x7026;
		end_addr = 0x702a;
	} else if (index == 3) {
		start_addr = 0x702b;
		end_addr = 0x702f;
	}

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d84, 0xC0, MSM_CAMERA_I2C_BYTE_DATA);
	//partial mode OTP write start address
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d88, ((start_addr >> 8) & 0xff), MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d89, (start_addr & 0xff), MSM_CAMERA_I2C_BYTE_DATA);
	// partial mode OTP write end address
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d8A, ((end_addr >> 8) & 0xff), MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d8B, (end_addr & 0xff), MSM_CAMERA_I2C_BYTE_DATA);
	// read otp into buffer
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d81, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	msleep(5);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, start_addr + 4, &temp, MSM_CAMERA_I2C_BYTE_DATA);

	//printk("%s:temp = %d\n", __func__, (int)temp);

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, start_addr, &rg_ratio_temp, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, start_addr + 1, &bg_ratio_temp, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, start_addr + 2, &light_rg_temp, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, start_addr + 3, &light_bg_temp, MSM_CAMERA_I2C_BYTE_DATA);

	(*otp_ptr).rg_ratio = (rg_ratio_temp<<2) + ((temp>>6) & 0x03);
	(*otp_ptr).bg_ratio = (bg_ratio_temp<<2) + ((temp>>4) & 0x03);
	(*otp_ptr).light_rg = (light_rg_temp<<2) + ((temp>>2) & 0x03);
	(*otp_ptr).light_bg = (light_bg_temp<<2) + (temp & 0x03);

	//printk("%s:rg_ratio = %d,bg_ratio=%d,light_rg=%d,light_bg=%d\n", __func__, (*otp_ptr).rg_ratio,(int)((*otp_ptr).bg_ratio),(int)((*otp_ptr).light_rg),(int)((*otp_ptr).light_bg));

	// clear otp buffer
	for (i = start_addr; i <= end_addr; i++) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, i, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	}
	return 0;
}

// index: index of otp group. (1, 2, 3)
// code: 0 start code, 1 stop code
// return: 0
int read_otp_VCM(struct msm_sensor_ctrl_t *s_ctrl, int index, struct otp_struct * otp_ptr)
{
	int i;
	uint16_t temp;
	int start_addr = 0, end_addr = 0;

	//printk("%s:the index of reading is:%d\n", __FUNCTION__, index);

	if (index == 1) {
		start_addr = 0x7031;
		end_addr = 0x7033;
	}
	else if (index == 2) {
		start_addr = 0x7034;
		end_addr = 0x7036;
	}
	else if (index == 3) {
		start_addr = 0x7037;
		end_addr = 0x7039;
	}

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d84, 0xC0, MSM_CAMERA_I2C_BYTE_DATA);
	//partial mode OTP write start address
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d88, (start_addr >> 8) & 0xff, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d89, start_addr & 0xff, MSM_CAMERA_I2C_BYTE_DATA);
	// partial mode OTP write end address
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d8A, (end_addr >> 8) & 0xff, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d8B, end_addr & 0xff, MSM_CAMERA_I2C_BYTE_DATA);
	// read otp into buffer
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d81, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	msleep(5);
	//flag and lsb of VCM start code
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, start_addr+2, &temp, MSM_CAMERA_I2C_BYTE_DATA);
	(* otp_ptr).VCM_start = (s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, start_addr, &((* otp_ptr).VCM_start), MSM_CAMERA_I2C_BYTE_DATA)<<2) | ((temp>>6) & 0x03);
	(* otp_ptr).VCM_end = (s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, start_addr + 1, &((* otp_ptr).VCM_end), MSM_CAMERA_I2C_BYTE_DATA)<< 2) | ((temp>>4) & 0x03);
	(* otp_ptr).VCM_dir = (temp>>2) & 0x03;

	// clear otp buffer
	for (i=start_addr; i<=end_addr; i++) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, i, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	}

	return 0;
}

// index: index of otp group. (1, 2, 3)
// otp_ptr: pointer of otp_struct
// return: 0,
int read_otp_lenc(struct msm_sensor_ctrl_t *s_ctrl, int index, struct otp_struct *otp_ptr)
{
	int i;
	int start_addr = 0, end_addr = 0;

	//printk("%s:the index of reading is:%d\n", __FUNCTION__, index);

	if (index == 1) {
		start_addr = 0x703b;
		end_addr = 0x7078;
	} else if (index == 2) {
		start_addr = 0x7079;
		end_addr = 0x70b6;
	} else if (index == 3) {
		start_addr = 0x70b7;
		end_addr = 0x70f4;
	}

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d84, 0xC0, MSM_CAMERA_I2C_BYTE_DATA);
	//partial mode OTP write start address
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d88, (start_addr >> 8) & 0xff, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d89, start_addr & 0xff, MSM_CAMERA_I2C_BYTE_DATA);
	// partial mode OTP write end address
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d8A, (end_addr >> 8) & 0xff, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d8B, end_addr & 0xff, MSM_CAMERA_I2C_BYTE_DATA);
	// read otp into buffer
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3d81, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	msleep(10);

	for(i = 0; i < 62; i++) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, (start_addr + i), &((*otp_ptr).lenc[i]), MSM_CAMERA_I2C_BYTE_DATA);
		//printk("%s:lenc[%d] = %d\n",__func__, i, (int)((*otp_ptr).lenc[i]));
	}

	// clear otp buffer
	for (i = start_addr; i <= end_addr; i++) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, i, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
	}

	return 0;
}

// R_gain, sensor red gain of AWB, 0x400 =1
// G_gain, sensor green gain of AWB, 0x400 =1
// B_gain, sensor blue gain of AWB, 0x400 =1
// return 0;
static int update_awb_gain(struct msm_sensor_ctrl_t *s_ctrl, int R_gain, int G_gain, int B_gain)
{
	//printk("%s:updated R_gain=%d,G_gain=%d,B_gain=%d\n", __FUNCTION__, R_gain, G_gain, B_gain);

	if (R_gain > 0x1000) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x5018, R_gain>>8, MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x5019, R_gain & 0x00ff, MSM_CAMERA_I2C_BYTE_DATA);
	}

	if (G_gain > 0x1000) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x501A, G_gain>>8, MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x501B, G_gain & 0x00ff, MSM_CAMERA_I2C_BYTE_DATA);
	}

	if (B_gain > 0x1000) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x501C, B_gain>>8, MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x501D, B_gain & 0x00ff, MSM_CAMERA_I2C_BYTE_DATA);
	}
	return 0;
}

// otp_ptr: pointer of otp_struct
int update_lenc(struct msm_sensor_ctrl_t *s_ctrl, struct otp_struct *otp_ptr)
{
	uint16_t i, temp;

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client, 0x5000, &temp, MSM_CAMERA_I2C_BYTE_DATA);
	//printk("%s:origin temp = %d\n", __func__, temp);
	temp = 0x80 | temp;
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x5000, temp, MSM_CAMERA_I2C_BYTE_DATA);
	//printk("%s:after temp = %d\n", __func__, temp);
	for (i = 0; i < 62; i++) {
		//printk("%s:lenc[%d] = %d\n",__func__, i, (int)((*otp_ptr).lenc[i]));
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x5800 + i, (*otp_ptr).lenc[i], MSM_CAMERA_I2C_BYTE_DATA);
	}

	return 0;
}

// call this function after OV8865 initialization
// return value: 0 update success
// 1, no OTP
//int update_otp_wb(struct msm_sensor_ctrl_t *s_ctrl)
int prepare_update_otp_wb(struct msm_sensor_ctrl_t *s_ctrl)
{
    int i;
    int otp_index;
    int temp;
	int rg,bg;
	int nR_G_gain, nB_G_gain, nG_G_gain;
	int nBase_gain;

	// R/G and B/G of current camera module is read out from sensor OTP
	// check first OTP with valid data
	for (i=1; i <= 3; i++) {
		temp = check_otp_wb(s_ctrl, i);
		if (temp == 2) {
			otp_index = i;
			break;
		}
	}
	if (i>3) {
		// no valid wb OTP data
		return 1;
	}

	read_otp_wb(s_ctrl, otp_index, &current_otp);

	if (current_otp.light_rg == 0) {
		// no light source information in OTP, light factor = 1
		rg = current_otp.rg_ratio;
	} else {
		rg = current_otp.rg_ratio * (current_otp.light_rg +512) / 1024;
	}

	if (current_otp.light_bg == 0) {
		// not light source information in OTP, light factor = 1
		bg = current_otp.bg_ratio;
	} else {
		bg = current_otp.bg_ratio * (current_otp.light_bg +512) / 1024;
	}

	//calculate G gain
	nR_G_gain = (RG_Ratio_Typical*0x1000) / rg;
	nB_G_gain = (BG_Ratio_Typical*0x1000) / bg;
	nG_G_gain = 0x1000;

	if (nR_G_gain < 0x1000 || nB_G_gain < 0x1000) {
		if (nR_G_gain < nB_G_gain)
			nBase_gain = nR_G_gain;
		else
			nBase_gain = nB_G_gain;
	} else {
		nBase_gain = nG_G_gain;
	}

	R_gain = 0x1000 * nR_G_gain / (nBase_gain);
	B_gain = 0x1000 * nB_G_gain / (nBase_gain);
	G_gain = 0x1000 * nG_G_gain / (nBase_gain);

	//update_awb_gain(s_ctrl, R_gain, G_gain, B_gain);

	return 0;
}

// call this function after OV8865 initialization
// return value: 0 update success
// 1, no OTP
//int update_otp_lenc(struct msm_sensor_ctrl_t *s_ctrl)
int prepare_update_otp_lenc(struct msm_sensor_ctrl_t *s_ctrl)
{
	int i;
	int otp_index;
	int temp;

	// check first lens correction OTP with valid data
	for(i = 1;i <= 3;i++) {
		temp = check_otp_lenc(s_ctrl, i);
		if (temp == 2) {
			otp_index = i;
			break;
		}
	}
	if (i > 3) {
		// no valid WB OTP data
		return 1;
	}
	read_otp_lenc(s_ctrl, otp_index, &current_otp);
	//update_lenc(s_ctrl, &current_otp);
	// success
	return 0;
}

int get_otp_info(struct msm_sensor_ctrl_t *s_ctrl)
{
	int i;
	int otp_index;
	int temp;

	for (i = 1;i <= 3; i++) {
		temp = check_otp_info(s_ctrl, i);
		if (temp == 2) {
			otp_index = i;
			break;
		}
	}
	if (i > 3) {
		// no valid WB OTP data
		return 1;
	}

	read_otp_info(s_ctrl, otp_index, &current_otp);

	//add by muyuezhong for module compatibility.
	//zq modify
	if(!(s_ctrl->module_id & 0xFF))
		s_ctrl->module_id |= current_otp.module_integrator_id;

	return 0;
}

int ov8865_sensor_prepare_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
      int rc;
	#if 0
      uint16_t reg0x0103;
	//printk("enter ov8865_sensor_prepare_otp\n");
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client, 0x0103, &reg0x0103, MSM_CAMERA_I2C_BYTE_DATA);
      //printk("read reg0x0103(B): %d.\n", reg0x0103);
	reg0x0103 = 0;
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client, 0x0103, &reg0x0103, MSM_CAMERA_I2C_WORD_DATA);
      //printk("read reg0x0103(W): %d.\n", reg0x0103);
	#endif
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			s_ctrl->sensor_i2c_client,
 			 0x0103, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc)
	     printk("stream on 0x0103 failed\n");
	msleep(50);
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			s_ctrl->sensor_i2c_client,
 			 0x0100, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc)
	     printk("stream on failed\n");
      rc = get_otp_info(s_ctrl);
	if (rc)
	     printk("get_otp_info failed\n");
	rc |= prepare_update_otp_lenc(s_ctrl);
	if (rc)
	     printk("prepare_update_otp_lenc failed\n");
	rc |= prepare_update_otp_wb(s_ctrl);
 	if (rc)
	     printk("prepare_update_otp_wb failed\n");
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			s_ctrl->sensor_i2c_client,
 			 0x0100, 0x00, MSM_CAMERA_I2C_BYTE_DATA);
      //printk("exit ov8865_sensor_prepare_otp\n");

      return rc;
}

int ov8865_update_otp(struct msm_sensor_ctrl_t *s_ctrl)
{ 
	//printk("enter ov8865_update_otp\n");
       //get_otp_info(s_ctrl);
	//update_otp_lenc(s_ctrl);
	update_lenc(s_ctrl, &current_otp);
	//update_otp_wb(s_ctrl);
 	update_awb_gain(s_ctrl, R_gain, G_gain, B_gain);
      //printk("exit ov8865_update_otp\n");

       return 0;
}

static struct msm_sensor_fn_t ov8865_sensor_fn_t = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = ov8865_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
	.sensor_update_otp = ov8865_update_otp,
	.sensor_prepare_otp = ov8865_sensor_prepare_otp,
};

static struct platform_driver ov8865_platform_driver = {
	.driver = {
		.name = "qcom,ov8865",
		.owner = THIS_MODULE,
		.of_match_table = ov8865_dt_match,
	},
};

static int32_t ov8865_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(ov8865_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init ov8865_init_module(void)
{
	int32_t rc = 0;
	printk("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ov8865_platform_driver,
		ov8865_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&ov8865_i2c_driver);
}

static void __exit ov8865_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ov8865_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov8865_s_ctrl);
		platform_driver_unregister(&ov8865_platform_driver);
	} else
		i2c_del_driver(&ov8865_i2c_driver);
	return;
}

static struct msm_sensor_ctrl_t ov8865_s_ctrl = {
	.sensor_i2c_client = &ov8865_sensor_i2c_client,
	.power_setting_array.power_setting = ov8865_power_setting,
	.power_setting_array.power_down_setting = ov8865_power_down_setting,	//added by congpeichao
	.power_setting_array.size = ARRAY_SIZE(ov8865_power_setting),
	.power_setting_array.size_down = ARRAY_SIZE(ov8865_power_down_setting),	//added by congpeichao
	.msm_sensor_mutex = &ov8865_mut,
	.sensor_v4l2_subdev_info = ov8865_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov8865_subdev_info),
	.func_tbl = &ov8865_sensor_fn_t,
	.sensor_prepare_otp = 1, //get update data from OTP in bootup
};

module_init(ov8865_init_module);
module_exit(ov8865_exit_module);
MODULE_DESCRIPTION("ov8865");
MODULE_LICENSE("GPL v2");
