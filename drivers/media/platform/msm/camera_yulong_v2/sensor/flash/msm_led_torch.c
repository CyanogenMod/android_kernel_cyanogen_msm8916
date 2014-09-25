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

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/gpio.h>
#include "msm_led_flash.h"
#include "msm_camera_io_util.h"

#if 0	//added by congpeichao from feiyongchao
static struct led_trigger *torch_trigger;
#endif

//added by congpeichao from feiyongchao start 
static struct gpio torch_gpio[]={
	{933,0,"FLASH"},	//	offset +902
	{934,0,"TORCH"},	//	offset +902
};
//added by congpeichao from feiyongchao end

static void msm_led_torch_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
#if 0	//added by congpeichao from feiyongchao
	if (!torch_trigger) {
		pr_err("No torch trigger found, can't set brightness\n");
		return;
	}

	led_trigger_event(torch_trigger, value);
#endif

//added by congpeichao from feiyongchao start 
	int rc = 0;
	rc = msm_camera_request_gpio_table(torch_gpio,2,1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return;
	}
	
	if(value == 0)
	{
		gpio_set_value_cansleep(
			torch_gpio[0].gpio,
			GPIO_OUT_LOW);
			
		gpio_set_value_cansleep(
			torch_gpio[1].gpio,
			GPIO_OUT_LOW);
	}
	else if(value > 0 && value <= 127)
	{
		gpio_set_value_cansleep(
			torch_gpio[0].gpio,
			GPIO_OUT_LOW);
			
		gpio_set_value_cansleep(
			torch_gpio[1].gpio,
			GPIO_OUT_HIGH);
	}
	else if(value > 127 && value <= 255)
	{
		gpio_set_value_cansleep(
			torch_gpio[0].gpio,
			GPIO_OUT_HIGH);
			
		gpio_set_value_cansleep(
			torch_gpio[1].gpio,
			GPIO_OUT_LOW);
	}
	else
	{
		pr_err("%s: invalid value\n", __func__);
	}
	
	rc = msm_camera_request_gpio_table(torch_gpio,2,0);
	if (rc < 0) {
		pr_err("%s: free gpio failed\n", __func__);
		return;
	}
	
	return;
//added by congpeichao from feiyongchao end

};

static struct led_classdev msm_torch_led = {
	.name			= "torch-light",
	.brightness_set	= msm_led_torch_brightness_set,
	.brightness		= LED_OFF,
};

int32_t msm_led_torch_create_classdev(struct platform_device *pdev,
				void *data)
{
	int rc;
#if 0	//added by congpeichao from feiyongchao
	struct msm_led_flash_ctrl_t *fctrl =
		(struct msm_led_flash_ctrl_t *)data;

	if (!fctrl || !fctrl->torch_trigger) {
		pr_err("Invalid fctrl or torch trigger\n");
		return -EINVAL;
	}

	torch_trigger = fctrl->torch_trigger;
#endif
	msm_led_torch_brightness_set(&msm_torch_led, LED_OFF);

	rc = led_classdev_register(&pdev->dev, &msm_torch_led);
	if (rc) {
		pr_err("Failed to register led dev. rc = %d\n", rc);
		return rc;
	}

	return 0;
};
