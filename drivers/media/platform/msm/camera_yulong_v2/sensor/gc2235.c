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
#define GC2235_SENSOR_NAME "gc2235"
DEFINE_MSM_MUTEX(gc2235_mut);

static struct msm_sensor_ctrl_t gc2235_s_ctrl;

static int first_time_power_up_gc2235 = 1;

static struct msm_sensor_power_setting gc2235_power_setting[] = {
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
		.use_system_power = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
		.use_system_power = 1,
	},
	/*
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,//DVDD
		.config_val = 0,
		.delay = 2,
		.use_system_power = 1,
	},
	*/
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
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
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 5,
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
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 5,
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

static struct v4l2_subdev_info gc2235_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id gc2235_i2c_id[] = {
	{GC2235_SENSOR_NAME, (kernel_ulong_t)&gc2235_s_ctrl},
	{ }
};

static int32_t msm_gc2235_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	return msm_sensor_i2c_probe(client, id, &gc2235_s_ctrl);
}

static struct i2c_driver gc2235_i2c_driver = {
	.id_table = gc2235_i2c_id,
	.probe  = msm_gc2235_i2c_probe,
	.driver = {
		.name = GC2235_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client gc2235_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static const struct of_device_id gc2235_dt_match[] = {
	{.compatible = "qcom,gc2235", .data = &gc2235_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, gc2235_dt_match);

int32_t gc2235_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
     int32_t rc;
     if (first_time_power_up_gc2235) {//if (first_time_power_up_gc2235 && frontCameraProbeInfo()) {
	 	printk("do not probe gc2235,just return\n");
        rc = -1;
        return rc;
	}

	rc = msm_sensor_power_up(s_ctrl);
    if (0 == rc) {
       printk("gc2235 probe ok,set first_time_power_up_gc2235 to 0.\n");
	   first_time_power_up_gc2235 = 0;
    }
	return rc;
}

/*static struct msm_sensor_fn_t gc2235_sensor_fn_t = {
	.sensor_config = msm_sensor_config,
	.sensor_power_up = gc2235_power_up,
	.sensor_power_down = msm_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
};*/

static struct platform_driver gc2235_platform_driver = {
	.driver = {
		.name = "qcom,gc2235",
		.owner = THIS_MODULE,
		.of_match_table = gc2235_dt_match,
	},
};

static int32_t gc2235_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	printk("xuyihai: %s:%d\n", __func__, __LINE__);
	match = of_match_device(gc2235_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init gc2235_init_module(void)
{
	int32_t rc = 0;
	printk("xuyihai: %s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&gc2235_platform_driver,
		gc2235_platform_probe);
	if (!rc)
		return rc;
	pr_err("xuyihai: %s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&gc2235_i2c_driver);
}

static void __exit gc2235_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (gc2235_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&gc2235_s_ctrl);
		platform_driver_unregister(&gc2235_platform_driver);
	} else
		i2c_del_driver(&gc2235_i2c_driver);
	return;
}

static struct msm_sensor_ctrl_t gc2235_s_ctrl = {
	.sensor_i2c_client = &gc2235_sensor_i2c_client,
	.power_setting_array.power_setting = gc2235_power_setting,
	.power_setting_array.size = ARRAY_SIZE(gc2235_power_setting),
	.msm_sensor_mutex = &gc2235_mut,
	.sensor_v4l2_subdev_info = gc2235_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(gc2235_subdev_info),
	//.func_tbl = &gc2235_sensor_fn_t,
};

module_init(gc2235_init_module);
module_exit(gc2235_exit_module);
MODULE_DESCRIPTION("gc2235");
MODULE_LICENSE("GPL v2");
