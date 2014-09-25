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
#define IMX219_SENSOR_NAME "imx219"
DEFINE_MSM_MUTEX(imx219_mut);

static struct msm_sensor_ctrl_t imx219_s_ctrl;
int imx219_update_otp(struct msm_sensor_ctrl_t *s_ctrl);

static struct msm_sensor_power_setting imx219_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,//dovdd low
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 10,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_GPIO,//avdd low
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = 0,
		.delay = 10,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_GPIO,//dvdd low
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = 0,
		.delay = 10,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_VREG,//af_vcc low
		.seq_val = CAM_VAF,
		.config_val = 0,
		.delay = 50,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,//SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 50,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_GPIO,//avdd high
		.seq_val = SENSOR_GPIO_VANA,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_GPIO,//dvdd high
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,//SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 10,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,//	.config_val = 24000000,
		.delay = 10,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 10,
		.use_system_power = 1,
	},
};

static struct v4l2_subdev_info imx219_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

static const struct i2c_device_id imx219_i2c_id[] = {
	{IMX219_SENSOR_NAME, (kernel_ulong_t)&imx219_s_ctrl},
	{ }
};

static int32_t msm_imx219_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &imx219_s_ctrl);
}

static struct i2c_driver imx219_i2c_driver = {
	.id_table = imx219_i2c_id,
	.probe  = msm_imx219_i2c_probe,
	.driver = {
		.name = IMX219_SENSOR_NAME,
	},
};
/////////IMX219 otp info add by linlukun 2041-6-13 start//////////////

#define OTP_DRV_LSC_SIZE  70  //280
#define OTP_DRV_START_ADDR 0x3204
#define OTP_DRV_END_ADDR 0x3243
#define OTP_DRV_FLAG_ADDR 0x3204

struct otp_struct {
	unsigned char flag;//effective-0x01 ineffective-0x00 
	unsigned char year;
	unsigned char month;
	unsigned char day;
	unsigned char module_integrator_id;//0x07
	unsigned char lens_id;
	unsigned char vcm_id;
	unsigned char focus_type; //af-0x00 ff-0x01
	unsigned char driver_id;
	unsigned char ir_bg; //ir-0x00 bg-0x01
	unsigned char otp_ver;
	unsigned char project_id;
	unsigned char custom_info;//null
	unsigned char lenc[OTP_DRV_LSC_SIZE];//280
	unsigned char check_sum;//for checking
};
uint32_t myreg;

uint16_t IMX219_read_i2c(struct msm_sensor_ctrl_t *s_ctrl, int addr)
{
	uint16_t temp = 0;
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(s_ctrl->sensor_i2c_client,addr,&temp,MSM_CAMERA_I2C_BYTE_DATA);
	return temp;
}

void IMX219_write_i2c(struct msm_sensor_ctrl_t *s_ctrl, int addr,int data)
{
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client,addr,data,MSM_CAMERA_I2C_BYTE_DATA);
}


int IMX219_readPage(struct msm_sensor_ctrl_t *s_ctrl, int page, unsigned char *out_buf)
{
 int cnt;
 int flag;
 int temp;
 flag = 0 ;

 if(page<0 || page>11 ){
  printk(KERN_ERR"[page =%d] %s\n", page, __func__);
  return -1;
 }
 if(out_buf==NULL) return -1;
 IMX219_write_i2c(s_ctrl, 0x0100,0x00);
 //12MHz,target25us
 IMX219_write_i2c(s_ctrl, 0x3302,0x01);
 IMX219_write_i2c(s_ctrl, 0x3303,0x2c);
 IMX219_write_i2c(s_ctrl, 0x012A,0x0c);
 IMX219_write_i2c(s_ctrl, 0x012B,0x00);
 //Set ECC here OFF
 IMX219_write_i2c(s_ctrl, 0x3300,0x08);
 //set Read mode
 IMX219_write_i2c(s_ctrl, 0x3200,0x01);
 //check read status OK?
 cnt = 0;

 while(IMX219_read_i2c(s_ctrl, (0x3201) & 0x01)==0)
 {
   cnt++;
   msleep(10);
   if(cnt==5) break;
 }
 //setpage
 IMX219_write_i2c(s_ctrl, 0x3202, page);
 
 flag = (unsigned char)IMX219_read_i2c(s_ctrl, OTP_DRV_FLAG_ADDR);
 if(page == 0 || page == 5)
 {
	  if(flag == 0)
	  {
	   return 0;
	  }
		printk(KERN_ERR"--------------------------------------\n");
	  temp = IMX219_read_i2c(s_ctrl, 0x3205); 
	  printk(KERN_ERR"module_integrator_id : %d\n",temp);
	  temp = IMX219_read_i2c(s_ctrl, 0x320A); 
	  printk(KERN_ERR"lens_id : %d\n",temp);
	  temp = IMX219_read_i2c(s_ctrl, 0x3207); 
	  printk(KERN_ERR"production_year : %d\n",temp);
	  temp = IMX219_read_i2c(s_ctrl, 0x3208); 
	  printk(KERN_ERR"production_month : %d\n",temp);
	  temp = IMX219_read_i2c(s_ctrl, 0x3209); 
	  printk(KERN_ERR"production_day : %d\n",temp);
		printk(KERN_ERR"--------------------------------------\n");  
 } 
 
 //read OTP buffer to PC buffer
 for(myreg=OTP_DRV_START_ADDR;  myreg <= OTP_DRV_END_ADDR;  myreg++)
 {
  out_buf[myreg-0x3204]=(unsigned char)IMX219_read_i2c(s_ctrl, (myreg));
 }
 //check read status OK?
 cnt=0;
 while(IMX219_read_i2c(s_ctrl, (0x3201) & 0x01)==0)
 {
  cnt++;
  msleep(10);
  if(cnt==5) break;
 }

 return 1;
}


////////////////////////// œ«ŽÓOTP¿ÕŒäÖÐ¶Á³öÀŽµÄLSCÊýŸÝÓŠÓÃ»ØSensor/////////////////////////
void IMX219_ApplyLSC(struct msm_sensor_ctrl_t *s_ctrl, unsigned char *LSCData)
{

 int i; 

#if 1
	{
		 for ( i = 0; i < OTP_DRV_LSC_SIZE; i++ )
		 {
		  IMX219_write_i2c(s_ctrl, 0xd200+i, (int)(0xff&LSCData[i]));
		 }
		 for ( i = 0; i < OTP_DRV_LSC_SIZE; i++ )
		 {
		  IMX219_write_i2c(s_ctrl, 0xd248+i, (int)(0xff&LSCData[i+70]));
		 }
		 for ( i = 0; i < OTP_DRV_LSC_SIZE; i++ )
		 {
		  IMX219_write_i2c(s_ctrl, 0xd290+i, (int)(0xff&LSCData[i+140]));
		 }
		 for ( i = 0; i < OTP_DRV_LSC_SIZE; i++ )
		 {
		  IMX219_write_i2c(s_ctrl, 0xd2d8+i, (int)(0xff&LSCData[i+210]));
		 }
	}

#endif
 
 //enable it 
#if 1
 IMX219_write_i2c(s_ctrl,  0x0190, 0x01);
 IMX219_write_i2c(s_ctrl,  0x0192, 0x00);//0x00
 IMX219_write_i2c(s_ctrl,  0x0191, 0x00);
 IMX219_write_i2c(s_ctrl,  0x0193, 0x00);
 IMX219_write_i2c(s_ctrl,  0x01a4, 0x03);
#endif
}
///////////////////////////////////////////////////////////////////////



/////////////////////////// ŽÓOTP¿ÕŒä¶Á³öÖ®Ç°ÐŽÈëµÄLSCÊýŸÝ /////////////////
void IMX219_ReadLSC(struct msm_sensor_ctrl_t *s_ctrl)
{

 unsigned char tempData[512]={0};
 int i ,rc, group;
 unsigned int sum;

 for(group = 2; group >0; group--)
 {
	  for(i = 0; i < 5; i++)
	  {
		   rc = IMX219_readPage(s_ctrl,(5*(group -1) + i),tempData+i*64);
		   
		   if(rc == 0||rc == -1)
		   	{
			    printk("otp info is not in group %d\n", group);
			    break;
		    }
	  }
 }
  
 sum = 0;
 for (i = 1; i < 16+280; i++ )
 {
  sum += tempData[i];
  sum = sum%0xff;
 }

 #if 1
 if( sum != tempData[296] )
 {
  printk("otp info check error !\n");
  return ;
 }
#endif
 IMX219_ApplyLSC(s_ctrl, tempData+16);
}
 
int imx219_update_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
	IMX219_ReadLSC(s_ctrl);
	return 1;
}

int imx219_prepare_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
/*
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
*/
	return 0;
}
static struct msm_camera_i2c_client imx219_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

/////////IMX219 otp info add by linlukun 2041-6-13 end////////////////
static struct msm_sensor_fn_t imx219_sensor_fn_t = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
	.sensor_update_otp = imx219_update_otp,
	.sensor_prepare_otp = imx219_prepare_otp,
};

static struct msm_sensor_ctrl_t imx219_s_ctrl = {
	.sensor_i2c_client = &imx219_sensor_i2c_client,
	.power_setting_array.power_setting = imx219_power_setting,
	.power_setting_array.size = ARRAY_SIZE(imx219_power_setting),
	.msm_sensor_mutex = &imx219_mut,
	.sensor_v4l2_subdev_info = imx219_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(imx219_subdev_info),
	.func_tbl = &imx219_sensor_fn_t,
	.sensor_prepare_otp = 1, //get update data from OTP in bootup
};

static const struct of_device_id imx219_dt_match[] = {
	{.compatible = "qcom,imx219", .data = &imx219_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, imx219_dt_match);

static struct platform_driver imx219_platform_driver = {
	.driver = {
		.name = "qcom,imx219",
		.owner = THIS_MODULE,
		.of_match_table = imx219_dt_match,
	},
};

static int32_t imx219_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(imx219_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init imx219_init_module(void)
{
	int32_t rc = 0;
	pr_err("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&imx219_platform_driver,
		imx219_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&imx219_i2c_driver);
}

static void __exit imx219_exit_module(void)
{
	pr_debug("%s:%d\n", __func__, __LINE__);
	if (imx219_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&imx219_s_ctrl);
		platform_driver_unregister(&imx219_platform_driver);
	} else {
		i2c_del_driver(&imx219_i2c_driver);
	}
	return;
}

module_init(imx219_init_module);
module_exit(imx219_exit_module);
MODULE_DESCRIPTION("imx219");
MODULE_LICENSE("GPL v2");

