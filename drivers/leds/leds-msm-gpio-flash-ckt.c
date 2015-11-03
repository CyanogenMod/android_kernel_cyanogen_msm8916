
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
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/printk.h>
#include <linux/list.h>
#include <linux/pinctrl/consumer.h>

/* #define CONFIG_GPIO_FLASH_DEBUG */
#undef CDBG
#ifdef CONFIG_GPIO_FLASH_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

#define LED_GPIO_FLASH_DRIVER_NAME	"qcom,leds-gpio-flash-ckt"
#define LED_TRIGGER_DEFAULT		"none"

#define GPIO_OUT_LOW          (0 << 1)
#define GPIO_OUT_HIGH         (1 << 1)

extern uint32_t get_camera_id(void);

enum msm_flash_seq_type_t {
	FLASH_EN,
	FLASH_NOW,
	TORCH_EN,
	FLASH_SELECT,
};

struct msm_flash_ctrl_seq {
	enum msm_flash_seq_type_t seq_type;
	uint8_t flash_on_val;
	uint8_t torch_on_val;
	uint8_t flash_off_val;
};

struct led_gpio_flash_data {
	int flash_en;
	int flash_now;
	int torch_en;
	int flash_rear;
	int flash_front;
	int brightness;
	struct led_classdev cdev;
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_default;
	struct msm_flash_ctrl_seq *ctrl_seq;
};

static struct of_device_id led_gpio_flash_of_match[] = {
	{.compatible = LED_GPIO_FLASH_DRIVER_NAME,},
	{},
};

static void led_gpio_brightness_set(struct led_classdev *led_cdev,
				    enum led_brightness value)
{
	int rc = 0;
	struct led_gpio_flash_data *flash_led =
	    container_of(led_cdev, struct led_gpio_flash_data, cdev);

	int brightness = value;
	int flash_en = 0, flash_now = 0;
	int torch_en = 0, flash_select = 0;
	int flash_select_gpio;

	if (brightness > LED_HALF) {
		flash_en =
			flash_led->ctrl_seq[FLASH_EN].flash_on_val;
		flash_now =
			flash_led->ctrl_seq[FLASH_NOW].flash_on_val;
		torch_en =
			flash_led->ctrl_seq[TORCH_EN].flash_on_val;
		flash_select =
			flash_led->ctrl_seq[FLASH_SELECT].flash_on_val;
	} else if (brightness > LED_OFF) {
		flash_en =
			flash_led->ctrl_seq[FLASH_EN].torch_on_val;
		flash_now =
			flash_led->ctrl_seq[FLASH_NOW].torch_on_val;
		torch_en =
			flash_led->ctrl_seq[TORCH_EN].torch_on_val;
		flash_select =
			flash_led->ctrl_seq[FLASH_SELECT].torch_on_val;
	} else {
		flash_en = 0;
		flash_now = 0;
		torch_en = 0;
		flash_select = 0;
	}
	CDBG("%s:flash_en=%d, flash_now=%d\n", __func__, flash_en, flash_now);
	CDBG("%s:torch_en=%d, flash_select=%d\n", __func__, torch_en, flash_select);

	rc = gpio_direction_output(flash_led->flash_en, flash_en);
	if (rc) {
		pr_err("%s: Failed to set gpio %d\n", __func__,
		       flash_led->flash_en);
		goto err;
	}
	rc = gpio_direction_output(flash_led->flash_now, flash_now);
	if (rc) {
		pr_err("%s: Failed to set gpio %d\n", __func__,
		       flash_led->flash_now);
		goto err;
	}
	rc = gpio_direction_output(flash_led->torch_en, torch_en);
	if (rc) {
		pr_err("%s: Failed to set gpio %d\n", __func__,
		       flash_led->torch_en);
		goto err;
	}
	flash_select_gpio = get_camera_id() ?
		flash_led->flash_front : flash_led->flash_rear;
	rc = gpio_direction_output(flash_select_gpio, flash_select);
	if (rc) {
		pr_err("%s: Failed to set gpio %d\n", __func__,
		       flash_select_gpio);
		goto err;
	}
	flash_led->brightness = brightness;
err:
	return;
}

static enum led_brightness led_gpio_brightness_get(struct led_classdev
						   *led_cdev)
{
	struct led_gpio_flash_data *flash_led =
	    container_of(led_cdev, struct led_gpio_flash_data, cdev);
	return flash_led->brightness;
}

int led_gpio_flash_probe(struct platform_device *pdev)
{
	int rc = 0;
	const char *temp_str;
	struct led_gpio_flash_data *flash_led = NULL;
	struct device_node *node = pdev->dev.of_node;
	const char *seq_name = NULL;
	uint32_t count = 0;
	uint32_t *array_flash_seq = NULL;
	uint32_t *array_torch_seq = NULL;
	int i = 0;
	flash_led = devm_kzalloc(&pdev->dev, sizeof(struct led_gpio_flash_data),
				 GFP_KERNEL);
	if (flash_led == NULL) {
		dev_err(&pdev->dev, "%s:%d Unable to allocate memory\n",
			__func__, __LINE__);
		return -ENOMEM;
	}

	flash_led->cdev.default_trigger = LED_TRIGGER_DEFAULT;
	rc = of_property_read_string(node, "linux,default-trigger", &temp_str);
	if (!rc)
		flash_led->cdev.default_trigger = temp_str;

	flash_led->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(flash_led->pinctrl)) {
		pr_err("%s:failed to get pinctrl\n", __func__);
		return PTR_ERR(flash_led->pinctrl);
	}

	flash_led->gpio_state_default = pinctrl_lookup_state(flash_led->pinctrl,
		"flash_default");
	if (IS_ERR(flash_led->gpio_state_default)) {
		pr_err("%s:can not get active pinstate\n", __func__);
		return -EINVAL;
	}

	rc = pinctrl_select_state(flash_led->pinctrl,
		flash_led->gpio_state_default);
	if (rc)
		pr_err("%s:set state failed!\n", __func__);

	flash_led->flash_en = of_get_named_gpio(node, "qcom,flash-en", 0);
	if (flash_led->flash_en < 0) {
		dev_err(&pdev->dev,
			"Looking up %s property in node %s failed. rc =  %d\n",
			"flash-en", node->full_name, flash_led->flash_en);
		goto error_pinctrl;
	} else {
		rc = gpio_request(flash_led->flash_en, "FLASH_EN");
		if (rc) {
			dev_err(&pdev->dev,
				"%s: Failed to request gpio %d,rc = %d\n",
				__func__, flash_led->flash_en, rc);

			goto error_pinctrl;
		}
	}

	flash_led->flash_now = of_get_named_gpio(node, "qcom,flash-now", 0);
	if (flash_led->flash_now < 0) {
		dev_err(&pdev->dev,
			"Looking up %s property in node %s failed. rc =  %d\n",
			"flash-now", node->full_name, flash_led->flash_now);
		goto error_pinctrl;
	} else {
		rc = gpio_request(flash_led->flash_now, "FLASH_NOW");
		if (rc) {
			dev_err(&pdev->dev,
				"%s: Failed to request gpio %d,rc = %d\n",
				__func__, flash_led->flash_now, rc);
			goto error_pinctrl;
		}
	}

	flash_led->torch_en = of_get_named_gpio(node, "qcom,torch-en", 0);
	if (flash_led->torch_en < 0) {
		dev_err(&pdev->dev,
			"Looking up %s property in node %s failed. rc =  %d\n",
			"torch_en", node->full_name, flash_led->torch_en);
		goto error_pinctrl;
	} else {
		rc = gpio_request(flash_led->torch_en, "TORCH_EN");
		if (rc) {
			dev_err(&pdev->dev,
				"%s: Failed to request gpio %d,rc = %d\n",
				__func__, flash_led->torch_en, rc);
			goto error_pinctrl;
		}
	}

	flash_led->flash_rear = of_get_named_gpio(node, "qcom,flash-rear", 0);
	if (flash_led->flash_rear < 0) {
		dev_err(&pdev->dev,
			"Looking up %s property in node %s failed. rc =  %d\n",
			"flash-rear", node->full_name, flash_led->flash_rear);
		goto error_pinctrl;
	} else {
		rc = gpio_request(flash_led->flash_rear, "FLASH_NOW");
		if (rc) {
			dev_err(&pdev->dev,
				"%s: Failed to request gpio %d,rc = %d\n",
				__func__, flash_led->flash_rear, rc);
			goto error_pinctrl;
		}
	}

	flash_led->flash_front = of_get_named_gpio(node, "qcom,flash-front", 0);
	if (flash_led->flash_front < 0) {
		dev_err(&pdev->dev,
			"Looking up %s property in node %s failed. rc =  %d\n",
			"flash-front", node->full_name, flash_led->flash_front);
		goto error_pinctrl;
	} else {
		rc = gpio_request(flash_led->flash_front, "FLASH_NOW");
		if (rc) {
			dev_err(&pdev->dev,
				"%s: Failed to request gpio %d,rc = %d\n",
				__func__, flash_led->flash_front, rc);
			goto error_pinctrl;
		}
	}

	rc = of_property_read_string(node, "linux,name", &flash_led->cdev.name);
	if (rc) {
		dev_err(&pdev->dev, "%s: Failed to read linux name. rc = %d\n",
			__func__, rc);
		goto error_pinctrl;
	}

	if (!of_get_property(node, "qcom,flash-seq-val", &count)) {
		dev_err(&pdev->dev, "%s: Failed to read flash sequence size\n",
				__func__);
		goto error_pinctrl;
	}

	count /= sizeof(uint32_t);
	if (!count) {
		dev_err(&pdev->dev, "%s: Failed: flash sequence size is 0\n",
				__func__);
		goto error_pinctrl;
	}

	array_flash_seq = devm_kzalloc(&pdev->dev, sizeof(uint32_t) * count,
			GFP_KERNEL);
	if (!array_flash_seq) {
		dev_err(&pdev->dev, "%s:%d: Failed to allocate memory\n",
				__func__, __LINE__);
		goto error_pinctrl;
	}

	rc = of_property_read_u32_array(node, "qcom,flash-seq-val",
		array_flash_seq, count);

	if (rc < 0) {
		pr_err("%s get flash op seq failed %d\n",
			__func__, __LINE__);
		goto error_flash;
	}

	array_torch_seq = devm_kzalloc(&pdev->dev, sizeof(uint32_t) * count,
			GFP_KERNEL);
	if (!array_torch_seq) {
		dev_err(&pdev->dev, "%s:%d: Failed to allocate memory\n",
				__func__, __LINE__);
		goto error_flash;
	}

	rc = of_property_read_u32_array(node, "qcom,torch-seq-val",
		array_torch_seq, count);

	if (rc < 0) {
		pr_err("%s get torch op seq failed %d\n",
			__func__, __LINE__);
		goto error_torch;
	}

	flash_led->ctrl_seq = devm_kzalloc(&pdev->dev,
			sizeof(struct msm_flash_ctrl_seq) * count, GFP_KERNEL);
	if (!flash_led->ctrl_seq) {
		dev_err(&pdev->dev, "%s:%d: Failed to allocate memory\n",
				__func__, __LINE__);
		goto error_torch;
	}

	for (i = 0; i < count; i++) {
		rc = of_property_read_string_index(node,
			"qcom,op-seq", i,
			&seq_name);
		CDBG("%s seq_name[%d] = %s\n", __func__, i,
			seq_name);
		if (rc < 0)
			dev_err(&pdev->dev, "%s failed %d\n",
				__func__, __LINE__);

		if (!strcmp(seq_name, "flash_en")) {
			flash_led->ctrl_seq[FLASH_EN].seq_type =
				FLASH_EN;
			CDBG("%s:%d seq_type[%d] %d\n", __func__, __LINE__,
				i, flash_led->ctrl_seq[FLASH_EN].seq_type);
			if (array_flash_seq[i] == 0)
				flash_led->ctrl_seq[FLASH_EN].flash_on_val =
					GPIO_OUT_LOW;
			else
				flash_led->ctrl_seq[FLASH_EN].flash_on_val =
					GPIO_OUT_HIGH;

			if (array_torch_seq[i] == 0)
				flash_led->ctrl_seq[FLASH_EN].torch_on_val =
					GPIO_OUT_LOW;
			else
				flash_led->ctrl_seq[FLASH_EN].torch_on_val =
					GPIO_OUT_HIGH;
		} else if (!strcmp(seq_name, "flash_now")) {
			flash_led->ctrl_seq[FLASH_NOW].seq_type =
				FLASH_NOW;
			CDBG("%s:%d seq_type[%d] %d\n", __func__, __LINE__,
				i, flash_led->ctrl_seq[i].seq_type);
			if (array_flash_seq[i] == 0)
				flash_led->ctrl_seq[FLASH_NOW].flash_on_val =
					GPIO_OUT_LOW;
			else
				flash_led->ctrl_seq[FLASH_NOW].flash_on_val =
					GPIO_OUT_HIGH;

			if (array_torch_seq[i] == 0)
				flash_led->ctrl_seq[FLASH_NOW].torch_on_val =
					GPIO_OUT_LOW;
			 else
				flash_led->ctrl_seq[FLASH_NOW].torch_on_val =
					GPIO_OUT_HIGH;
		} else if (!strcmp(seq_name, "torch_en")) {
			flash_led->ctrl_seq[TORCH_EN].seq_type =
				TORCH_EN;
			CDBG("%s:%d seq_type[%d] %d\n", __func__, __LINE__,
				i, flash_led->ctrl_seq[i].seq_type);
			if (array_flash_seq[i] == 0)
				flash_led->ctrl_seq[TORCH_EN].flash_on_val =
					GPIO_OUT_LOW;
			else
				flash_led->ctrl_seq[TORCH_EN].flash_on_val =
					GPIO_OUT_HIGH;

			if (array_torch_seq[i] == 0)
				flash_led->ctrl_seq[TORCH_EN].torch_on_val =
					GPIO_OUT_LOW;
			 else
				flash_led->ctrl_seq[TORCH_EN].torch_on_val =
					GPIO_OUT_HIGH;
		} else if (!strcmp(seq_name, "flash_select")) {
			flash_led->ctrl_seq[FLASH_SELECT].seq_type =
				FLASH_SELECT;
			CDBG("%s:%d seq_type[%d] %d\n", __func__, __LINE__,
				i, flash_led->ctrl_seq[i].seq_type);
			if (array_flash_seq[i] == 0)
				flash_led->ctrl_seq[FLASH_SELECT].flash_on_val =
					GPIO_OUT_LOW;
			else
				flash_led->ctrl_seq[FLASH_SELECT].flash_on_val =
					GPIO_OUT_HIGH;

			if (array_torch_seq[i] == 0)
				flash_led->ctrl_seq[FLASH_SELECT].torch_on_val =
					GPIO_OUT_LOW;
			 else
				flash_led->ctrl_seq[FLASH_SELECT].torch_on_val =
					GPIO_OUT_HIGH;
		}

	}

	platform_set_drvdata(pdev, flash_led);
	flash_led->cdev.max_brightness = LED_FULL;
	flash_led->cdev.brightness_set = led_gpio_brightness_set;
	flash_led->cdev.brightness_get = led_gpio_brightness_get;

	rc = led_classdev_register(&pdev->dev, &flash_led->cdev);
	if (rc) {
		dev_err(&pdev->dev, "%s: Failed to register led dev. rc = %d\n",
			__func__, rc);
		goto error;
	}
	pr_err("%s:probe successfully!\n", __func__);
	return 0;

error:
	devm_kfree(&pdev->dev, flash_led->ctrl_seq);
error_torch:
	devm_kfree(&pdev->dev, array_torch_seq);
error_flash:
	devm_kfree(&pdev->dev, array_flash_seq);
error_pinctrl:
	if (IS_ERR(flash_led->pinctrl))
		devm_pinctrl_put(flash_led->pinctrl);
	devm_kfree(&pdev->dev, flash_led);
	return rc;
}

int led_gpio_flash_remove(struct platform_device *pdev)
{
	struct led_gpio_flash_data *flash_led =
	    (struct led_gpio_flash_data *)platform_get_drvdata(pdev);
	if (IS_ERR(flash_led->pinctrl))
		devm_pinctrl_put(flash_led->pinctrl);
	led_classdev_unregister(&flash_led->cdev);
	devm_kfree(&pdev->dev, flash_led);
	return 0;
}

static struct platform_driver led_gpio_flash_driver = {
	.probe = led_gpio_flash_probe,
	.remove = led_gpio_flash_remove,
	.driver = {
		   .name = LED_GPIO_FLASH_DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = led_gpio_flash_of_match,
		   }
};

static int __init led_gpio_flash_init(void)
{
	return platform_driver_register(&led_gpio_flash_driver);
}

static void __exit led_gpio_flash_exit(void)
{
	return platform_driver_unregister(&led_gpio_flash_driver);
}

late_initcall(led_gpio_flash_init);
module_exit(led_gpio_flash_exit);

MODULE_DESCRIPTION("QCOM GPIO LEDs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("leds:leds-msm-gpio-flash");
