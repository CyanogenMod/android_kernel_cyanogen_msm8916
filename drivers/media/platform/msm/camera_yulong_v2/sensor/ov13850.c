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

#define OV13850_DEBUG
//#undef OV13850_DEBUG
#ifdef OV13850_DEBUG
#define CDBG(fmt, args...) printk(fmt, ##args)
#define CDBG_HIGH(fmt, args...) printk(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#define CDBG_HIGH(fmt, args...) printk(fmt, ##args)
#endif
#define OV13850_SENSOR_NAME "ov13850"
#define Module_OFILM 7
#define Module_TECH  6
#define Module_FOXCONN 17

DEFINE_MSM_MUTEX(ov13850_mut);

static struct msm_sensor_ctrl_t ov13850_s_ctrl;

static int first_time_power_up_ov13850 = 1;
static struct msm_sensor_power_setting ov13850_power_setting[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 5,
    .use_system_power = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 0,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
    .use_system_power = 1,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 1,
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
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
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
		.delay = 0,
		.use_system_power = 1,
	},

};

static struct v4l2_subdev_info ov13850_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id ov13850_i2c_id[] = {
	{OV13850_SENSOR_NAME, (kernel_ulong_t)&ov13850_s_ctrl},
	{ }
};

static int32_t msm_ov13850_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &ov13850_s_ctrl);
}

static struct i2c_driver ov13850_i2c_driver = {
	.id_table = ov13850_i2c_id,
	.probe  = msm_ov13850_i2c_probe,
	.driver = {
		.name = OV13850_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client ov13850_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

int32_t ov13850_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
    	int32_t rc;
	if (first_time_power_up_ov13850 && rearCameraProbeInfo()) {
	 	printk(KERN_ERR"do not probe ov13850,just return\n");
        	rc = -1;
        	return rc;
	}

	rc = msm_sensor_power_up(s_ctrl);
   	if (0 == rc) {
       		printk(KERN_ERR"ov13850 probe ok,set first_time_power_up_ov13850 to 0.\n");
	   	first_time_power_up_ov13850 = 0;
    	}
	return rc;
}

//opt codes
#define OTP_DRV_START_ADDR 0x7220
#define OTP_DRV_INFO_GROUP_COUNT 3
#define OTP_DRV_INFO_SIZE 5
#define OTP_DRV_AWB_GROUP_COUNT 3
#define OTP_DRV_AWB_SIZE 5
#define OTP_DRV_VCM_GROUP_COUNT 3
#define OTP_DRV_VCM_SIZE 3
#define OTP_DRV_LSC_GROUP_COUNT 3
#define OTP_DRV_LSC_SIZE 62
#define OTP_DRV_OIS_GROUP_COUNT 3
#define OTP_DRV_OIS_SIZE 33
#define OTP_DRV_CL_GROUP_COUNT 3
#define OTP_DRV_CL_SIZE 7
#define OTP_DRV_LSC_REG_ADDR 0x5200

int RG_Ratio_Typical = 0x127;
int BG_Ratio_Typical = 0x14E;

struct otp_struct {
	int module_integrator_id;
	int lens_id;
	int production_year;
	int production_month;
	int production_day;
	int rg_ratio;
	int bg_ratio;
	int light_rg;
	int light_bg;
	int lenc[OTP_DRV_LSC_SIZE];
	int VCM_start;
	int VCM_end;
	int VCM_dir;
	int OIS_DriverID;
	int OIS[OTP_DRV_OIS_SIZE];
	int CL_Version;
	int CL_Bias;
	int CL_Offset;
	int CL_INF;
	int CL_MACRO;
};

uint16_t OV13850_read_i2c(struct msm_sensor_ctrl_t *s_ctrl,int addr)
{
	uint16_t temp = 0;
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,addr,&temp,MSM_CAMERA_I2C_BYTE_DATA);
	return temp;
}

void OV13850_write_i2c(struct msm_sensor_ctrl_t *s_ctrl,int addr,int data)
{
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,addr,data,MSM_CAMERA_I2C_BYTE_DATA);
}

// call this function before access OTP
int OV13850_OTP_access_start(struct msm_sensor_ctrl_t *s_ctrl)
{
	int temp;
	OV13850_write_i2c(s_ctrl,0x0100,0x01);
	mdelay(10);
	temp = OV13850_read_i2c(s_ctrl, 0x5002);
	temp = 0x05;//temp & 0xfd; // set bit 1 to '0' //05
	OV13850_write_i2c(s_ctrl, 0x5002, temp);
	return 0;
}

// call this function after access OTP
int OV13850_OTP_access_end(struct msm_sensor_ctrl_t *s_ctrl)
{
	int temp;
	temp = OV13850_read_i2c(s_ctrl, 0x5002);
	temp = temp | 0x02; // set bit 1 to '1'
	OV13850_write_i2c(s_ctrl, 0x5002, temp);
	OV13850_write_i2c(s_ctrl,0x0100,0x00);
	mdelay(5);
	return 0;
}
// index: index of otp group. (1, 2,бн, OTP_DRV_INFO_GROUP_COUNT)
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
int check_otp_info(struct msm_sensor_ctrl_t *s_ctrl,int index)
{
	int flag;
	int nFlagAddress = OTP_DRV_START_ADDR;

	OV13850_OTP_access_start(s_ctrl);
	OV13850_write_i2c(s_ctrl, 0x3d84, 0xC0);//c0
	//manual mode OTP write start address
	OV13850_write_i2c(s_ctrl, 0x3d88, (nFlagAddress>>8) & 0xff);//72
	OV13850_write_i2c(s_ctrl, 0x3d89, nFlagAddress & 0xff);//20
	// manual mode OTP write end address
	OV13850_write_i2c(s_ctrl, 0x3d8A, (nFlagAddress>>8) & 0xff );//72
	OV13850_write_i2c(s_ctrl, 0x3d8B, nFlagAddress & 0xff );//20
	// read otp into buffer
	OV13850_write_i2c(s_ctrl, 0x3d81, 0x01);//01
	mdelay(5);
	flag = OV13850_read_i2c(s_ctrl, nFlagAddress);//40
	//select group
	if(index==1)
	{
		flag = (flag>>6) & 0x03;
	}
	else if(index==2)
	{
		flag = (flag>>4) & 0x03;
	}
	else
	{
		flag = (flag>>2) & 0x03;
	}
	// clear otp buffer
	OV13850_write_i2c(s_ctrl, nFlagAddress, 0x00);
	OV13850_OTP_access_end(s_ctrl);
	if (flag == 0x00) {
		return 0;
	}
	else if (flag & 0x02) {
		return 1;
	}
	else {
		return 2;
	}
}
// index: index of otp group. (1, 2,бн, OTP_DRV_AWB_GROUP_COUNT)
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
int check_otp_wb(struct msm_sensor_ctrl_t *s_ctrl,int index)
{
	int flag;
	int nFlagAddress = OTP_DRV_START_ADDR + 1 + OTP_DRV_INFO_GROUP_COUNT*OTP_DRV_INFO_SIZE;
	OV13850_OTP_access_start(s_ctrl);
	OV13850_write_i2c(s_ctrl, 0x3d84, 0xC0);
	// manual mode OTP write start address
	OV13850_write_i2c(s_ctrl, 0x3d88, (nFlagAddress>>8) & 0xff );
	OV13850_write_i2c(s_ctrl, 0x3d89, nFlagAddress & 0xff);
	// manual mode OTP write end address
	OV13850_write_i2c(s_ctrl, 0x3d8A, (nFlagAddress>>8) & 0xff );
	OV13850_write_i2c(s_ctrl, 0x3d8B, nFlagAddress & 0xff);
	// read otp into buffer
	OV13850_write_i2c(s_ctrl, 0x3d81, 0x01);
	mdelay(5);
	//select group
	flag = OV13850_read_i2c(s_ctrl, nFlagAddress);
	if(index==1)
	{
		flag = (flag>>6) & 0x03;
	}
	else if(index==2)
	{
		flag = (flag>>4) & 0x03;
	}
	else
	{
		flag =( flag>>2) & 0x03;
	}
	// clear otp buffer
	OV13850_write_i2c(s_ctrl, nFlagAddress, 0x00);
	OV13850_OTP_access_end(s_ctrl);
	if (flag == 0x00) {
		return 0;
	}
	else if (flag & 0x02) {
		return 1;
	}
	else {
		return 2;
	}
}
// index: index of otp group. (1, 2,бн, OTP_DRV_VCM_GROUP_COUNT)
// code: 0 for start code, 1 for stop code
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
int check_otp_VCM(struct msm_sensor_ctrl_t *s_ctrl,int index)
{
	int flag;
	int nFlagAddress= OTP_DRV_START_ADDR + 1 + OTP_DRV_INFO_GROUP_COUNT*OTP_DRV_INFO_SIZE 
		+ 1 + OTP_DRV_AWB_GROUP_COUNT*OTP_DRV_AWB_SIZE;
	OV13850_OTP_access_start(s_ctrl);
	OV13850_write_i2c(s_ctrl, 0x3d84, 0xC0);
	// manual mode OTP write start address
	OV13850_write_i2c(s_ctrl, 0x3d88, (nFlagAddress>>8) & 0xff );
	OV13850_write_i2c(s_ctrl, 0x3d89, nFlagAddress & 0xff);
	// manual mode OTP write end address
	OV13850_write_i2c(s_ctrl, 0x3d8A, (nFlagAddress>>8) & 0xff );
	OV13850_write_i2c(s_ctrl, 0x3d8B, nFlagAddress & 0xff);
	// read otp into buffer
	OV13850_write_i2c(s_ctrl, 0x3d81, 0x01);
	mdelay(5);
	//select group
	flag = OV13850_read_i2c(s_ctrl, nFlagAddress);
	if(index==1)
	{
		flag = (flag>>6) & 0x03;
	}
	else if(index==2)
	{
		flag = (flag>>4) & 0x03;
	}
	else
	{
		flag = (flag>>2) & 0x03;
	}
	// clear otp buffer
	OV13850_write_i2c(s_ctrl, nFlagAddress, 0x00);
	OV13850_OTP_access_end(s_ctrl);
	if (flag == 0x00) {
		return 0;
	}
	else if (flag & 0x02) {
		return 1;
	}
	else {
		return 2;
	}
}
// index: index of otp group. (1, 2,...,OTP_DRV_LSC_GROUP_COUNT)
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
int check_otp_lenc(struct msm_sensor_ctrl_t *s_ctrl,int index)
{
	int flag;
	int nFlagAddress = OTP_DRV_START_ADDR + 1 + OTP_DRV_INFO_GROUP_COUNT*OTP_DRV_INFO_SIZE
		+ 1 + OTP_DRV_AWB_GROUP_COUNT*OTP_DRV_AWB_SIZE
		+ 1 + OTP_DRV_VCM_GROUP_COUNT*OTP_DRV_VCM_SIZE ;
	OV13850_OTP_access_start(s_ctrl);
	OV13850_write_i2c(s_ctrl, 0x3d84, 0xc0);
	// manual mode OTP write start address
	OV13850_write_i2c(s_ctrl, 0x3d88, (nFlagAddress>>8) & 0xff );
	OV13850_write_i2c(s_ctrl, 0x3d89, nFlagAddress & 0xff);
	// manual mode OTP write end address
	OV13850_write_i2c(s_ctrl, 0x3d8A, (nFlagAddress>>8) & 0xff );
	OV13850_write_i2c(s_ctrl, 0x3d8B, nFlagAddress & 0xff);
	// read otp into buffer
	OV13850_write_i2c(s_ctrl, 0x3d81, 0x01);
	mdelay(5);
	flag = OV13850_read_i2c(s_ctrl, nFlagAddress);
	if(index==1)
	{
		flag = (flag>>6) & 0x03;
	}
	else if(index==2)
	{
		flag = (flag>>4) & 0x03;
	}
	else
	{
		flag = (flag>> 2)& 0x03;
	}
	// clear otp buffer
	OV13850_write_i2c(s_ctrl, nFlagAddress, 0x00);
	OV13850_OTP_access_end(s_ctrl);
	if (flag == 0x00) {
		return 0;
	}
	else if (flag & 0x02) {
		return 1;
	}
	else {
		return 2;
	}
}
// index: index of otp group. (1, 2,...,OTP_DRV_INFO_GROUP_COUNT)
// otp_ptr: pointer of otp_struct
// return: 0,
int read_otp_info(struct msm_sensor_ctrl_t *s_ctrl,int index, struct otp_struct *otp_ptr)
{
	int i;
	int nFlagAddress = OTP_DRV_START_ADDR;
	int start_addr, end_addr;
	start_addr = nFlagAddress + 1 + (index-1)*OTP_DRV_INFO_SIZE;
	end_addr = start_addr + OTP_DRV_INFO_SIZE - 1;
	
	OV13850_OTP_access_start(s_ctrl);
	OV13850_write_i2c(s_ctrl, 0x3d84, 0xC0);
	// manual mode OTP write start address
	OV13850_write_i2c(s_ctrl, 0x3d88, (start_addr >> 8) & 0xff);
	OV13850_write_i2c(s_ctrl, 0x3d89, start_addr & 0xff);
	// manual mode OTP write end address
	OV13850_write_i2c(s_ctrl, 0x3d8A, (end_addr >> 8) & 0xff);
	OV13850_write_i2c(s_ctrl, 0x3d8B, end_addr & 0xff);
	// read otp into buffer
	OV13850_write_i2c(s_ctrl, 0x3d81, 0x01);
	mdelay(5);
	//OV13850_write_i2c(s_ctrl,0x0100,0x01);
	(*otp_ptr).module_integrator_id = OV13850_read_i2c(s_ctrl, start_addr);
	(*otp_ptr).lens_id = OV13850_read_i2c(s_ctrl, start_addr + 1);
	(*otp_ptr).production_year = OV13850_read_i2c(s_ctrl, start_addr + 2);
	(*otp_ptr).production_month = OV13850_read_i2c(s_ctrl, start_addr + 3);
	(*otp_ptr).production_day = OV13850_read_i2c(s_ctrl, start_addr + 4);
	
	printk(KERN_ERR"module_integrator_id : %d\n",(*otp_ptr).module_integrator_id );
  	printk(KERN_ERR"lens_id : %d\n",(*otp_ptr).lens_id);
  	printk(KERN_ERR"production_year : %d\n",(*otp_ptr).production_year);
  	printk(KERN_ERR"production_month : %d\n",(*otp_ptr).production_month);
  	printk(KERN_ERR"production_day : %d\n",(*otp_ptr).production_day);
    
  if(!(s_ctrl->module_id & 0xFF))
		s_ctrl->module_id |= (*otp_ptr).module_integrator_id;
	
	switch((*otp_ptr).module_integrator_id)
	{
		case Module_FOXCONN:
			RG_Ratio_Typical = 0x127;
			BG_Ratio_Typical = 0x14E;
      			CDBG_HIGH("ov13850 Module is foxconn\n");
      			break;
    		case Module_TECH:
    			RG_Ratio_Typical = 0x129;
			BG_Ratio_Typical = 0x12C;
			CDBG_HIGH("ov13850 Module is tech\n");
			break;
		default:
			RG_Ratio_Typical = 0x172;
			BG_Ratio_Typical = 0x179;
			CDBG_HIGH("ov13850 Module is not defined\n");
			break;
	}
	// clear otp buffer
	for (i=start_addr;i<=end_addr;i++) {
		OV13850_write_i2c(s_ctrl, i, 0x00);
	}
	OV13850_OTP_access_end(s_ctrl);
	return 0;
}
// index: index of otp group. (1, 2,...,OTP_DRV_AWB_GROUP_COUNT)
// otp_ptr: pointer of otp_struct
// return: 0,
int read_otp_wb(struct msm_sensor_ctrl_t *s_ctrl,int index, struct otp_struct *otp_ptr)
{
	int i;
	int temp;
	int start_addr, end_addr;
	int nFlagAddress = OTP_DRV_START_ADDR+1+OTP_DRV_INFO_GROUP_COUNT*OTP_DRV_INFO_SIZE;
	start_addr = nFlagAddress + 1 + (index-1)* OTP_DRV_AWB_SIZE;
	end_addr = start_addr + OTP_DRV_AWB_SIZE - 1;
	OV13850_OTP_access_start(s_ctrl);
	OV13850_write_i2c(s_ctrl, 0x3d84, 0xC0);
	// manual mode OTP write start address
	OV13850_write_i2c(s_ctrl, 0x3d88, (start_addr >> 8) & 0xff);
	OV13850_write_i2c(s_ctrl, 0x3d89, start_addr & 0xff);
	// manual mode OTP write end address
	OV13850_write_i2c(s_ctrl, 0x3d8A, (end_addr >> 8) & 0xff);
	OV13850_write_i2c(s_ctrl, 0x3d8B, end_addr & 0xff);
	// read otp into buffer
	OV13850_write_i2c(s_ctrl, 0x3d81, 0x01);
	mdelay(5);
	temp = OV13850_read_i2c(s_ctrl, start_addr + 4);
	(*otp_ptr).rg_ratio = (OV13850_read_i2c(s_ctrl, start_addr )<<2) + ((temp>>6) & 0x03);
	(*otp_ptr).bg_ratio = (OV13850_read_i2c(s_ctrl, start_addr + 1)<<2) + ((temp>>4) & 0x03);
	(*otp_ptr).light_rg = (OV13850_read_i2c(s_ctrl, start_addr + 2) <<2) + ((temp>>2) & 0x03);
	(*otp_ptr).light_bg = (OV13850_read_i2c(s_ctrl, start_addr + 3)<<2) + (temp & 0x03);
	printk(KERN_ERR"YULONG:feiyc:%s,rg_ratio=0x%x\n",__func__,(*otp_ptr).rg_ratio);
	printk(KERN_ERR"YULONG:feiyc:%s,bg_ratio=0x%x\n",__func__,(*otp_ptr).bg_ratio);
	printk(KERN_ERR"YULONG:feiyc:%s,light_rg=0x%x\n",__func__,(*otp_ptr).light_rg);
	printk(KERN_ERR"YULONG:feiyc:%s,light_bg=0x%x\n",__func__,(*otp_ptr).light_bg);
	// clear otp buffer
	for (i=start_addr;i<=end_addr;i++) {
		OV13850_write_i2c(s_ctrl, i, 0x00);
	}
	OV13850_OTP_access_end(s_ctrl);
	return 0;
}
// index: index of otp group. (1, 2,...,OTP_DRV_VCM_GROUP_COUNT)
// code: 0 start code, 1 stop code
// return: 0
int read_otp_VCM(struct msm_sensor_ctrl_t *s_ctrl,int index, struct otp_struct * otp_ptr)
{
	int i;
	int temp;
	int start_addr, end_addr;
	int nFlagAddress = OTP_DRV_START_ADDR + 1 + OTP_DRV_INFO_GROUP_COUNT*OTP_DRV_INFO_SIZE
	+ 1 + OTP_DRV_AWB_GROUP_COUNT*OTP_DRV_AWB_SIZE;
	start_addr = nFlagAddress + 1 + (index-1)*OTP_DRV_VCM_SIZE;
	end_addr = start_addr + OTP_DRV_VCM_SIZE - 1;
	OV13850_OTP_access_start(s_ctrl);
	OV13850_write_i2c(s_ctrl, 0x3d84, 0xC0);
	// manual mode OTP write start address
	OV13850_write_i2c(s_ctrl, 0x3d88, (start_addr>>8));
	OV13850_write_i2c(s_ctrl, 0x3d89, (start_addr & 0xff));
	// manual mode OTP write end address
	OV13850_write_i2c(s_ctrl, 0x3d8A, (end_addr>>8));
	OV13850_write_i2c(s_ctrl, 0x3d8B, (end_addr & 0xff));
	// read otp into buffer
	OV13850_write_i2c(s_ctrl, 0x3d81, 0x01);
	mdelay(5);
	//flag and lsb of VCM start code
	temp = OV13850_read_i2c(s_ctrl,  start_addr + 2);
	(* otp_ptr).VCM_start = (OV13850_read_i2c(s_ctrl, start_addr)<<2) | ((temp>>6) & 0x03);
	(* otp_ptr).VCM_end = (OV13850_read_i2c(s_ctrl, start_addr + 1) << 2) | ((temp>>4) & 0x03);
	(* otp_ptr).VCM_dir = (temp>>2) & 0x03;
	// clear otp buffer
	for (i=start_addr;i<=end_addr;i++) {
		OV13850_write_i2c(s_ctrl, i, 0x00);
	}
	OV13850_OTP_access_end(s_ctrl);
	return 0;
}
// index: index of otp group. (1, 2,...,OTP_DRV_LSC_GROUP_COUNT)
// otp_ptr: pointer of otp_struct
// return: 0,
int read_otp_lenc(struct msm_sensor_ctrl_t *s_ctrl,int index, struct otp_struct *otp_ptr)
{
	int i;
	int start_addr, end_addr;
	int nFlagAddress= OTP_DRV_START_ADDR + 1 + OTP_DRV_INFO_GROUP_COUNT* OTP_DRV_INFO_SIZE
	+ 1 + OTP_DRV_AWB_GROUP_COUNT* OTP_DRV_AWB_SIZE
	+ 1 + OTP_DRV_VCM_GROUP_COUNT* OTP_DRV_VCM_SIZE;
	start_addr = nFlagAddress+1+(index-1)*OTP_DRV_LSC_SIZE ;
	end_addr = start_addr+OTP_DRV_LSC_SIZE-1;
	OV13850_OTP_access_start(s_ctrl);
	OV13850_write_i2c(s_ctrl, 0x3d84, 0xC0);
	// manual mode OTP write start address
	OV13850_write_i2c(s_ctrl, 0x3d88, (start_addr>>8));
	OV13850_write_i2c(s_ctrl, 0x3d89, (start_addr & 0xff));
	// manual mode OTP write end address
	OV13850_write_i2c(s_ctrl, 0x3d8A, (end_addr>>8));
	OV13850_write_i2c(s_ctrl, 0x3d8B, (end_addr & 0xff));
	// read otp into buffer
	OV13850_write_i2c(s_ctrl, 0x3d81, 0x01);
	mdelay(10);
	for(i=0;i<OTP_DRV_LSC_SIZE;i++) {
		(* otp_ptr).lenc[i]=OV13850_read_i2c(s_ctrl, start_addr + i);
	}
	// clear otp buffer
	for (i=start_addr;i<=end_addr;i++) {
		OV13850_write_i2c(s_ctrl, i, 0x00);
	}
	OV13850_OTP_access_end(s_ctrl);
	return 0;
}
// R_gain, sensor red gain of AWB, 0x400 =1
// G_gain, sensor green gain of AWB, 0x400 =1
// B_gain, sensor blue gain of AWB, 0x400 =1
// return 0;
int update_awb_gain(struct msm_sensor_ctrl_t *s_ctrl, int R_gain, int G_gain, int B_gain)
{
	if (R_gain>=0x400) {
		OV13850_write_i2c(s_ctrl, 0x5056, R_gain>>8);
		OV13850_write_i2c(s_ctrl, 0x5057, R_gain & 0x00ff);
	}
	if (G_gain>=0x400) {
		OV13850_write_i2c(s_ctrl, 0x5058, G_gain>>8);
		OV13850_write_i2c(s_ctrl, 0x5059, G_gain & 0x00ff);
	}
	if (B_gain>=0x400) {
		OV13850_write_i2c(s_ctrl, 0x505a, B_gain>>8);
		OV13850_write_i2c(s_ctrl, 0x505b, B_gain & 0x00ff);
	}
	return 0;
}
// otp_ptr: pointer of otp_struct
int update_lenc(struct msm_sensor_ctrl_t *s_ctrl,struct otp_struct * otp_ptr)
{
	int i, temp;
	temp = OV13850_read_i2c(s_ctrl, 0x5000);
	temp = 0x01 | temp;
	OV13850_write_i2c(s_ctrl, 0x5000, temp);
	for(i=0;i<OTP_DRV_LSC_SIZE;i++) {
		OV13850_write_i2c(s_ctrl, OTP_DRV_LSC_REG_ADDR + i, (*otp_ptr).lenc[i]);
	}
	return 0;
}
// call this function after OV13850 initialization
// return value: 0 update success
// 1, no OTP
int update_otp_wb(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct otp_struct current_otp;
	int i;
	int otp_index;// bank 1,2,3
	int temp;
	int R_gain, G_gain, B_gain, G_gain_R, G_gain_B;
	int rg,bg;
	// R/G and B/G of current camera module is read out from sensor OTP
	// check first OTP with valid data
	for(i=1;i<=OTP_DRV_AWB_GROUP_COUNT;i++) {
		temp = check_otp_wb(s_ctrl,i);
		if (temp == 2) {
			otp_index = i;
			break;
		}
	}
	if (i>OTP_DRV_AWB_GROUP_COUNT) {
	// no valid wb OTP data
		return 1;
	}
	// set right bank
	read_otp_wb(s_ctrl,otp_index, &current_otp);
	if(current_otp.light_rg==0) {
		// no light source information in OTP, light factor = 1
		rg = current_otp.rg_ratio;
	}
	else {
		rg = current_otp.rg_ratio * (current_otp.light_rg + 512) / 1024;
	}
	if(current_otp.light_bg==0) {
		// not light source information in OTP, light factor = 1
		bg = current_otp.bg_ratio;
	}
	else {
		bg = current_otp.bg_ratio * (current_otp.light_bg + 512) / 1024;
	}
	//calculate G gain
	//0x400 = 1x gain
	CDBG_HIGH("rg = 0x%x,RG_Ratio_Typical = 0x%x\n",rg,RG_Ratio_Typical);
  	CDBG_HIGH("bg = 0x%x,BG_Ratio_Typical = 0x%x\n",bg,BG_Ratio_Typical);
  
	if(bg < BG_Ratio_Typical) {
		if (rg< RG_Ratio_Typical) {
			// current_otp.bg_ratio < BG_Ratio_typical &&
			// current_otp.rg_ratio < RG_Ratio_typical
			G_gain = 0x400;
			B_gain = 0x400 * BG_Ratio_Typical / bg;
			R_gain = 0x400 * RG_Ratio_Typical / rg;
		}
	else {
		// current_otp.bg_ratio < BG_Ratio_typical &&
		// current_otp.rg_ratio >= RG_Ratio_typical
		R_gain = 0x400;
		G_gain = 0x400 * rg / RG_Ratio_Typical;
		B_gain = G_gain * BG_Ratio_Typical /bg;
		}
	}
	else {
		if (rg < RG_Ratio_Typical) {
		// current_otp.bg_ratio >= BG_Ratio_typical &&
		// current_otp.rg_ratio < RG_Ratio_typical
		B_gain = 0x400;
		G_gain = 0x400 * bg / BG_Ratio_Typical;
		R_gain = G_gain * RG_Ratio_Typical / rg;
		}
	else {
		// current_otp.bg_ratio >= BG_Ratio_typical &&
		// current_otp.rg_ratio >= RG_Ratio_typical
		G_gain_B = 0x400 * bg / BG_Ratio_Typical;
		G_gain_R = 0x400 * rg / RG_Ratio_Typical;
		if(G_gain_B > G_gain_R ) {
			B_gain = 0x400;
			G_gain = G_gain_B;
			R_gain = G_gain * RG_Ratio_Typical /rg;
		}
		else {
			R_gain = 0x400;
			G_gain = G_gain_R;
			B_gain = G_gain * BG_Ratio_Typical / bg;
			}
		}
	}
	printk("R_gain = 0x%x, G_gain = 0x%x, B_gain = 0x%x\n",R_gain, G_gain, B_gain);
	update_awb_gain(s_ctrl,R_gain, G_gain, B_gain);
	return 0;
}
// call this function after OV13850 initialization
// return value: 0 update success
// 1, no OTP
int update_otp_lenc(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct otp_struct current_otp;
	int i;
	int otp_index;// bank 4,8,12
	int temp;
	// check first lens correction OTP with valid data
	for(i=1;i<=OTP_DRV_LSC_GROUP_COUNT;i++) {
		temp = check_otp_lenc(s_ctrl,i);
		if (temp == 2) {
			otp_index = i;//otp_index = i<<2;
			break;
		}
	}
	if (i>OTP_DRV_LSC_GROUP_COUNT) {
		// no valid wb OTP data
		return 1;
	}
	read_otp_lenc(s_ctrl,otp_index, &current_otp);
	update_lenc(s_ctrl,&current_otp);
	// success
	return 0;
}

int ov13850_update_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
	update_otp_lenc(s_ctrl);
 	update_otp_wb(s_ctrl);
  return 0;
}

int ov13850_prepare_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
	struct otp_struct current_otp;
	int i;
	int otp_index;// bank 4,8,12
	int temp;
	// check first lens correction OTP with valid data
	for(i=1;i<=OTP_DRV_LSC_GROUP_COUNT;i++) {
		temp = check_otp_info(s_ctrl,i);
		if (temp == 2) {
			otp_index = i;
			break;
		}
	}
	if (i>OTP_DRV_LSC_GROUP_COUNT) {	
		// no valid wb OTP data
		return 1;
	}
	read_otp_info(s_ctrl,otp_index, &current_otp);
	return 0;
}
static struct msm_sensor_fn_t ov13850_sensor_fn_t = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = ov13850_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
	.sensor_update_otp = ov13850_update_otp,
	.sensor_prepare_otp = ov13850_prepare_otp,
};

static struct msm_sensor_ctrl_t ov13850_s_ctrl = {
	.sensor_i2c_client = &ov13850_sensor_i2c_client,
	.power_setting_array.power_setting = ov13850_power_setting,
	.power_setting_array.size = ARRAY_SIZE(ov13850_power_setting),
	.msm_sensor_mutex = &ov13850_mut,
	.sensor_v4l2_subdev_info = ov13850_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(ov13850_subdev_info),
	.func_tbl = &ov13850_sensor_fn_t,
	.sensor_prepare_otp = 1, //get update data from OTP in bootup
};

static const struct of_device_id ov13850_dt_match[] = {
	{.compatible = "qcom,ov13850", .data = &ov13850_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, ov13850_dt_match);
static struct platform_driver ov13850_platform_driver = {
	.driver = {
		.name = "qcom,ov13850",
		.owner = THIS_MODULE,
		.of_match_table = ov13850_dt_match,
	},
};

static int32_t ov13850_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(ov13850_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init ov13850_init_module(void)
{
	int32_t rc = 0;
	printk("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&ov13850_platform_driver,
		ov13850_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&ov13850_i2c_driver);
}

static void __exit ov13850_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (ov13850_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&ov13850_s_ctrl);
		platform_driver_unregister(&ov13850_platform_driver);
	} else
		i2c_del_driver(&ov13850_i2c_driver);
	return;
}

module_init(ov13850_init_module);
module_exit(ov13850_exit_module);
MODULE_DESCRIPTION("ov13850");
MODULE_LICENSE("GPL v2");
