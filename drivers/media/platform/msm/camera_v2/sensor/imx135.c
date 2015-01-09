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

#ifdef CDBG
#undef CDBG
#endif
#ifdef CDBG_HIGH
#undef CDBG_HIGH
#endif

#ifdef IMX135_DEBUG
#define CDBG(fmt, args...) printk(fmt, ##args)
#define CDBG_HIGH(fmt, args...) printk(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#define CDBG_HIGH(fmt, args...) printk(fmt, ##args)
#endif

#define IMX135_SENSOR_NAME "imx135"
DEFINE_MSM_MUTEX(imx135_mut);

static struct msm_sensor_ctrl_t imx135_s_ctrl;

#ifdef CONFIG_MACH_YULONG
static int first_time_power_up_imx135 = 1;
#endif

static struct msm_sensor_power_setting imx135_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VAF,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 30,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 30,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct v4l2_subdev_info imx135_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

static const struct i2c_device_id imx135_i2c_id[] = {
	{IMX135_SENSOR_NAME, (kernel_ulong_t)&imx135_s_ctrl},
	{ }
};

static int32_t msm_imx135_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &imx135_s_ctrl);
}

static struct i2c_driver imx135_i2c_driver = {
	.id_table = imx135_i2c_id,
	.probe  = msm_imx135_i2c_probe,
	.driver = {
		.name = IMX135_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client imx135_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id imx135_dt_match[] = {
	{.compatible = "qcom,imx135", .data = &imx135_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, imx135_dt_match);

#ifdef CONFIG_MACH_YULONG
struct otp_struct {
	unsigned char production_year;
	unsigned char production_month;
	unsigned char production_day;
	unsigned char module_vendor;
	unsigned char lens_id;
	unsigned char VCM_id;
	unsigned char driverIC_id;
	unsigned char colorTemperature;
	unsigned char rg_ratio_h;
	unsigned char rg_ratio_l;
	unsigned char bg_ratio_h;
	unsigned char bg_ratio_l;
	unsigned char bggr_ratio_h;
	unsigned char bggr_ratio_l;
#ifdef IMX135_OTP_ROLLOFF
	unsigned char lenc_A4[253];
	unsigned char lenc_A6[251];
#endif
};

static struct otp_struct current_otp;
static int imx135_sensor_prepare_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
	int num_byte;
	int addr;
	uint16_t bak_sid;
	uint16_t bak_retries;
	uint16_t bak_id_map;
	enum cci_i2c_master_t bak_cci_i2c_master;
	struct msm_camera_cci_ctrl cci_ctrl;
	struct msm_camera_cci_client *cci_client = NULL;
	int32_t rc2 = 0;
	unsigned char *buf = NULL;

	CDBG("[YZY] enter prepare otp\n");
	cci_client = s_ctrl->sensor_i2c_client->cci_client;
	bak_sid = cci_client->sid;
	bak_retries = cci_client->retries;
	bak_id_map = cci_client->id_map;
	bak_cci_i2c_master = cci_client->cci_i2c_master;

	cci_client->retries = 3;
	cci_client->id_map = 0;
	cci_client->cci_i2c_master = s_ctrl->cci_i2c_master;
	cci_ctrl.cmd = MSM_CCI_I2C_READ;
	cci_ctrl.cci_info = cci_client;

	num_byte = 43;
	addr = 0x00;
	buf = kzalloc(num_byte, GFP_KERNEL);
	if (!buf) {
		pr_err("%s:%d no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}
	cci_client->sid = 0xA0 >> 1;

	cci_ctrl.cfg.cci_i2c_read_cfg.addr = addr;
	cci_ctrl.cfg.cci_i2c_read_cfg.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	cci_ctrl.cfg.cci_i2c_read_cfg.data = buf;
	cci_ctrl.cfg.cci_i2c_read_cfg.num_byte = num_byte;
	rc2 = v4l2_subdev_call(cci_client->cci_subdev, core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
	CDBG("%s line %d rc = %d\n", __func__, __LINE__, rc2);
	current_otp.production_year = buf[3];
	current_otp.production_month = buf[4];
	current_otp.production_day = buf[5];
	current_otp.module_vendor = buf[6];
	current_otp.lens_id = buf[7];
	current_otp.VCM_id = buf[8];
	current_otp.driverIC_id = buf[9];
	current_otp.colorTemperature = buf[10];
	current_otp.rg_ratio_h = buf[15];
	current_otp.rg_ratio_l = buf[16];
	current_otp.bg_ratio_h = buf[17];
	current_otp.bg_ratio_l = buf[18];
	current_otp.bggr_ratio_h = buf[19];
	current_otp.bggr_ratio_l = buf[20];
	CDBG("wbl otp_data.module_vendor = 0x%x\n",current_otp.module_vendor);
	CDBG("0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",current_otp.rg_ratio_h,current_otp.rg_ratio_l,current_otp.bg_ratio_h,current_otp.bg_ratio_l,current_otp.bggr_ratio_h,current_otp.bggr_ratio_l);
	kfree(buf);

	if(!(s_ctrl->module_id & 0xFF))
		s_ctrl->module_id |= current_otp.module_vendor;

#ifdef IMX135_OTP_ROLLOFF
	num_byte = 253;
	addr = 0x03;
	cci_client->sid = 0xA4 >> 1;
	cci_ctrl.cfg.cci_i2c_read_cfg.addr = addr;
	cci_ctrl.cfg.cci_i2c_read_cfg.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	cci_ctrl.cfg.cci_i2c_read_cfg.data = current_otp.lenc_A4;
	cci_ctrl.cfg.cci_i2c_read_cfg.num_byte = num_byte;
	rc2 = v4l2_subdev_call(cci_client->cci_subdev, core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
	CDBG("%s line %d rc = %d\n", __func__, __LINE__, rc2);

	num_byte = 251;
	addr = 0x00;
	cci_client->sid = 0xA6 >> 1;
	cci_ctrl.cfg.cci_i2c_read_cfg.addr = addr;
	cci_ctrl.cfg.cci_i2c_read_cfg.addr_type = MSM_CAMERA_I2C_BYTE_ADDR;
	cci_ctrl.cfg.cci_i2c_read_cfg.data = current_otp.lenc_A6;
	cci_ctrl.cfg.cci_i2c_read_cfg.num_byte = num_byte;
	rc2 = v4l2_subdev_call(cci_client->cci_subdev, core, ioctl, VIDIOC_MSM_CCI_CFG, &cci_ctrl);
	CDBG("%s line %d rc = %d\n", __func__, __LINE__, rc2);
#endif
	cci_client->sid = bak_sid;
	cci_client->retries = bak_retries;
	cci_client->id_map = bak_id_map;
	cci_client->cci_i2c_master = bak_cci_i2c_master;

	return 0;
}

#ifdef IMX135_OTP_AWB
#define IMX135_RG_RATIO_TYPICAL_VALUE 0x24B
#define IMX135_BG_RATIO_TYPICAL_VALUE 0x297

static int imx135_update_awb(struct msm_sensor_ctrl_t *s_ctrl, struct otp_struct *otp_ptr)
{
	uint16_t R_gain,B_gain,G_gain;
	uint16_t G_test_R, G_test_B;
	uint16_t bg_ratio_hl;//16 bit bg ratio
	uint16_t rg_ratio_hl;//16 bit rg ratio
	bg_ratio_hl = otp_ptr->bg_ratio_h <<8 | otp_ptr->bg_ratio_l;
	rg_ratio_hl = otp_ptr->rg_ratio_h <<8 | otp_ptr->rg_ratio_l;
	if(bg_ratio_hl < IMX135_BG_RATIO_TYPICAL_VALUE)
	{
		if (rg_ratio_hl < IMX135_RG_RATIO_TYPICAL_VALUE) {
			G_gain = 0x100;
			B_gain = 0x100 * IMX135_BG_RATIO_TYPICAL_VALUE / bg_ratio_hl;
			R_gain = 0x100 * IMX135_RG_RATIO_TYPICAL_VALUE / rg_ratio_hl;
		} else {
			R_gain = 0x100;
			G_gain = 0x100 * rg_ratio_hl / IMX135_RG_RATIO_TYPICAL_VALUE;
			B_gain = G_gain * IMX135_BG_RATIO_TYPICAL_VALUE / bg_ratio_hl;
		}
	} else {
		if (rg_ratio_hl < IMX135_RG_RATIO_TYPICAL_VALUE) {
			B_gain = 0x100;
			G_gain = 0x100 * bg_ratio_hl / IMX135_BG_RATIO_TYPICAL_VALUE;
			R_gain = G_gain * IMX135_RG_RATIO_TYPICAL_VALUE / rg_ratio_hl;
		} else {
			G_test_B = bg_ratio_hl / IMX135_BG_RATIO_TYPICAL_VALUE;
			G_test_R = rg_ratio_hl / IMX135_RG_RATIO_TYPICAL_VALUE;
			if(G_test_B > G_test_R ){
				B_gain = 0x100;
				G_gain = 0x100 * G_test_B;
				R_gain = 0x100 * G_test_B * IMX135_RG_RATIO_TYPICAL_VALUE / rg_ratio_hl;
			} else {
				R_gain = 0x100;
				G_gain = 0x100 * G_test_R;
				B_gain = 0x100 * G_test_R * IMX135_BG_RATIO_TYPICAL_VALUE / bg_ratio_hl;
			}
		}
	}
	CDBG("R_Gain = 0x%x,B_gain=0x%x,G_gain=0x%x\n",R_gain,B_gain,G_gain);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x020E, (G_gain>>8)&0x0F, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x020F, G_gain &0xFFF, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0210, ( R_gain>>8)&0x0F, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0211, R_gain &0xFF, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0212, (B_gain>>8)&0x0F, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0213, B_gain &0xFF, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0214, (G_gain>>8)&0x0F, MSM_CAMERA_I2C_BYTE_DATA);
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0215, G_gain &0xFF, MSM_CAMERA_I2C_BYTE_DATA);
	return 0;
}
#endif

#ifdef IMX135_OTP_ROLLOFF
static int lsc_init_on_cnt = 0;
static int imx135_update_lenc(struct msm_sensor_ctrl_t *s_ctrl, struct otp_struct *otp_ptr)
{
	uint16_t i;
	CDBG("%s enter\n",__func__);
	if(3 == lsc_init_on_cnt){
		CDBG("just return\n");
	}
	if(lsc_init_on_cnt < 3){
		lsc_init_on_cnt++;
	}
	for (i = 0; i < 253; i++) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x4800 + i, (*otp_ptr).lenc_A4[i], MSM_CAMERA_I2C_BYTE_DATA);
	}
	for (i = 0; i < 251; i++) {
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x48FD + i, (*otp_ptr).lenc_A6[i], MSM_CAMERA_I2C_BYTE_DATA);
	}
	if(lsc_init_on_cnt <=3){
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x4500, 0x1f, MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x0700, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
		s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(s_ctrl->sensor_i2c_client, 0x3a63, 0x01, MSM_CAMERA_I2C_BYTE_DATA);
	}
	CDBG("%s exit\n",__func__);
	return 0;
}
#endif

static int imx135_update_otp(struct msm_sensor_ctrl_t *s_ctrl)
{
#ifdef IMX135_OTP_AWB
	imx135_update_awb(s_ctrl, &current_otp);
#endif
#ifdef IMX135_OTP_ROLLOFF
	imx135_update_lenc(s_ctrl, &current_otp);
#endif
	return 0;
}

static int32_t imx135_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc;
	if(first_time_power_up_imx135 &&
		msm_sensor_is_probed(s_ctrl->sensordata->sensor_info->position)){
		pr_info("do not probe imx135,just return\n");
		return -1;
	}
	rc = msm_sensor_power_up(s_ctrl);
	if(!rc){
		pr_info("imx135 probe ok,set first_time_power_up_imx135 to 0");
		first_time_power_up_imx135 = 0;
	}
	return rc;
}

static struct msm_sensor_fn_t imx135_sensor_fn_t = {
#ifdef CONFIG_COMPAT
	.sensor_config32 = msm_sensor_config32,
#endif
	.sensor_config = msm_sensor_config,
	.sensor_power_up = imx135_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
	.sensor_update_otp = imx135_update_otp,
	.sensor_prepare_otp = imx135_sensor_prepare_otp,
};
#endif

static struct platform_driver imx135_platform_driver = {
	.driver = {
		.name = "qcom,imx135",
		.owner = THIS_MODULE,
		.of_match_table = imx135_dt_match,
	},
};

static int32_t imx135_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(imx135_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init imx135_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&imx135_platform_driver,
		imx135_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&imx135_i2c_driver);
}

static void __exit imx135_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (imx135_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&imx135_s_ctrl);
		platform_driver_unregister(&imx135_platform_driver);
	} else
		i2c_del_driver(&imx135_i2c_driver);
	return;
}

static struct msm_sensor_ctrl_t imx135_s_ctrl = {
	.sensor_i2c_client = &imx135_sensor_i2c_client,
	.power_setting_array.power_setting = imx135_power_setting,
	.power_setting_array.size = ARRAY_SIZE(imx135_power_setting),
	.msm_sensor_mutex = &imx135_mut,
	.sensor_v4l2_subdev_info = imx135_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(imx135_subdev_info),
#ifdef CONFIG_MACH_YULONG
	.func_tbl = &imx135_sensor_fn_t,
#endif
};

module_init(imx135_init_module);
module_exit(imx135_exit_module);
MODULE_DESCRIPTION("imx135");
MODULE_LICENSE("GPL v2");
