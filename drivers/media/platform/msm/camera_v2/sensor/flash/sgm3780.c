/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
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
#include <linux/module.h>
#include <linux/export.h>
#include "msm_led_flash.h"

#define FLASH_NAME "rohm-flash,sgm3780"

/*#define CONFIG_MSMB_CAMERA_DEBUG*/
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

static struct msm_led_flash_ctrl_t fctrl;

static const struct of_device_id sgm3780_trigger_dt_match[] = {
	{.compatible = "rohm-flash,sgm3780", .data = &fctrl},
	{}
};

MODULE_DEVICE_TABLE(of, sgm3780_trigger_dt_match);

static int msm_flash_sgm3780_platform_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	match = of_match_device(sgm3780_trigger_dt_match, &pdev->dev);
	if (!match)
	{
		return -EFAULT;
	}
	return msm_flash_gpio_probe(pdev, match->data);
}

static struct platform_driver sgm3780_platform_driver = {
	.probe = msm_flash_sgm3780_platform_probe,
	.driver = {
		.name = "rohm-flash,sgm3780",
		.owner = THIS_MODULE,
		.of_match_table = sgm3780_trigger_dt_match,
	},
};

static int __init msm_flash_sgm3780_init_module(void)
{
	int32_t rc = 0;
	rc = platform_driver_register(&sgm3780_platform_driver);
	if (!rc)
		return rc;
	pr_debug("%s:%d rc %d\n", __func__, __LINE__, rc);
	return 0;
}

static void __exit msm_flash_sgm3780_exit_module(void)
{
	if (fctrl.pdev)
		platform_driver_unregister(&sgm3780_platform_driver);
}

static struct msm_flash_fn_t sgm3780_func_tbl = {
	.flash_get_subdev_id = msm_led_gpio_trigger_get_subdev_id,
	.flash_led_config = msm_led_gpio_trigger_config,
	.flash_led_init = msm_flash_led_gpio_init,
	.flash_led_release = msm_flash_led_gpio_release,
	.flash_led_off = msm_flash_led_gpio_off,
	.flash_led_low = msm_flash_led_gpio_low,
	.flash_led_high = msm_flash_led_gpio_high,
};

static struct msm_led_flash_ctrl_t fctrl = {
	//.flash_i2c_client = &sgm3780_i2c_client,
	//.reg_setting = &sgm3780_regs,
	.func_tbl = &sgm3780_func_tbl,
};

/*subsys_initcall(msm_flash_i2c_add_driver);*/
module_init(msm_flash_sgm3780_init_module);
module_exit(msm_flash_sgm3780_exit_module);
MODULE_DESCRIPTION("sgm3780 FLASH");
MODULE_LICENSE("GPL v2");
