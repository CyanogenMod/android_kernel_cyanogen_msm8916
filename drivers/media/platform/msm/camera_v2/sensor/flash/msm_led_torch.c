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

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#ifdef CONFIG_MACH_YULONG
#include <linux/gpio.h>
#endif
#include <linux/module.h>
#include "msm_led_flash.h"
#ifdef CONFIG_MACH_YULONG
#include "msm_camera_io_util.h"
#endif

#ifndef CONFIG_MACH_YULONG
static struct led_trigger *torch_trigger;
#endif

#ifdef CONFIG_MACH_YULONG
static struct gpio torch_gpio[]={
	{933,0,"FLASH"},
	{934,0,"TORCH"},
};
#endif

static void msm_led_torch_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
#ifndef CONFIG_MACH_YULONG
	if (!torch_trigger) {
		pr_err("No torch trigger found, can't set brightness\n");
		return;
	}

	led_trigger_event(torch_trigger, value);

#else
	int rc = 0;
	rc = msm_camera_request_gpio_table(torch_gpio, 2, 1);
	if (rc < 0) {
		pr_err("%s: request gpio failed\n", __func__);
		return;
	}

	if(value == 0) {
		gpio_set_value_cansleep(torch_gpio[0].gpio, GPIO_OUT_LOW);
		gpio_set_value_cansleep(torch_gpio[1].gpio, GPIO_OUT_LOW);
	} else if (value > 0 && value <= 127) {
		gpio_set_value_cansleep(torch_gpio[0].gpio, GPIO_OUT_LOW);
		gpio_set_value_cansleep(torch_gpio[1].gpio, GPIO_OUT_HIGH);
	} else if(value > 127 && value <= 255) {
		gpio_set_value_cansleep(torch_gpio[0].gpio, GPIO_OUT_HIGH);
		gpio_set_value_cansleep(torch_gpio[1].gpio, GPIO_OUT_LOW);
	} else {
		pr_err("%s: invalid value\n", __func__);
	}

	rc = msm_camera_request_gpio_table(torch_gpio, 2, 0);
	if (rc < 0) {
		pr_err("%s: free gpio failed\n", __func__);
	}
#endif
};

static struct led_classdev msm_torch_led[MAX_LED_TRIGGERS] = {
	{
		.name		= "torch-light0",
		.brightness_set	= msm_led_torch_brightness_set,
		.brightness	= LED_OFF,
	},
	{
		.name		= "torch-light1",
		.brightness_set	= msm_led_torch_brightness_set,
		.brightness	= LED_OFF,
	},
	{
		.name		= "torch-light2",
		.brightness_set	= msm_led_torch_brightness_set,
		.brightness	= LED_OFF,
	},
};

int32_t msm_led_torch_create_classdev(struct platform_device *pdev,
				void *data)
{
	int32_t i, rc = 0;
#ifndef CONFIG_MACH_YULONG
	struct msm_led_flash_ctrl_t *fctrl =
		(struct msm_led_flash_ctrl_t *)data;

	if (!fctrl) {
		pr_err("Invalid fctrl\n");
		return -EINVAL;
	}

	for (i = 0; i < fctrl->torch_num_sources; i++) {
		if (fctrl->torch_trigger[i]) {
			torch_trigger = fctrl->torch_trigger[i];
			msm_led_torch_brightness_set(&msm_torch_led[i],
				LED_OFF);

			rc = led_classdev_register(&pdev->dev,
				&msm_torch_led[i]);
			if (rc) {
				pr_err("Failed to register %d led dev. rc = %d\n",
						i, rc);
				return rc;
			}
		} else {
			pr_err("Invalid fctrl->torch_trigger[%d]\n", i);
			return -EINVAL;
		}
	}
#else
	i = 0;
	msm_led_torch_brightness_set(&msm_torch_led[i], LED_OFF);
	rc = led_classdev_register(&pdev->dev,
			&msm_torch_led[i]);
	if (rc) {
		pr_err("Failed to register %d led dev. rc = %d\n",
				i, rc);
		return rc;
	}
#endif

	return 0;
};
