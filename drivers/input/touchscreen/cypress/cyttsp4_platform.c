/*
 * cyttsp4_platform.c
 * Cypress TrueTouch(TM) Standard Product V4 Platform Module.
 * For use with Cypress Txx4xx parts.
 * Supported parts include:
 * TMA4XX
 * TMA1036
 *
 * Copyright (C) 2014 Cypress Semiconductor
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/err.h>

/* cyttsp */
#include <linux/cyttsp4_bus.h>
#include <linux/cyttsp4_core.h>

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_PLATFORM_FW_UPGRADE
#include "cyttsp4_img.h"
static struct cyttsp4_touch_firmware cyttsp4_firmware = {
	.img = cyttsp4_img,
	.size = ARRAY_SIZE(cyttsp4_img),
	.ver = cyttsp4_ver,
	.vsize = ARRAY_SIZE(cyttsp4_ver),
};
#else
static struct cyttsp4_touch_firmware cyttsp4_firmware = {
	.img = NULL,
	.size = 0,
	.ver = NULL,
	.vsize = 0,
};
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP4_PLATFORM_TTCONFIG_UPGRADE
#include "cyttsp4_params.h"
static struct touch_settings cyttsp4_sett_param_regs = {
	.data = (uint8_t *)&cyttsp4_param_regs[0],
	.size = ARRAY_SIZE(cyttsp4_param_regs),
	.tag = 0,
};

static struct touch_settings cyttsp4_sett_param_size = {
	.data = (uint8_t *)&cyttsp4_param_size[0],
	.size = ARRAY_SIZE(cyttsp4_param_size),
	.tag = 0,
};

static struct cyttsp4_touch_config cyttsp4_ttconfig = {
	.param_regs = &cyttsp4_sett_param_regs,
	.param_size = &cyttsp4_sett_param_size,
	.fw_ver = ttconfig_fw_ver,
	.fw_vsize = ARRAY_SIZE(ttconfig_fw_ver),
};
#else
static struct cyttsp4_touch_config cyttsp4_ttconfig = {
	.param_regs = NULL,
	.param_size = NULL,
	.fw_ver = NULL,
	.fw_vsize = 0,
};
#endif

struct cyttsp4_loader_platform_data _cyttsp4_loader_platform_data = {
	.fw = &cyttsp4_firmware,
	.ttconfig = &cyttsp4_ttconfig,
	.flags = CY_LOADER_FLAG_CALIBRATE_AFTER_FW_UPGRADE | CY_LOADER_FLAG_CHECK_TTCONFIG_VERSION,
};

int cyttsp4_xres(struct cyttsp4_core_platform_data *pdata,
		struct device *dev)
{
	int rst_gpio = pdata->rst_gpio;
	int rc = 0;

	gpio_set_value(rst_gpio, 1);
	msleep(20);
	gpio_set_value(rst_gpio, 0);
	msleep(40);
	gpio_set_value(rst_gpio, 1);
	msleep(20);
	dev_info(dev,
		"%s: RESET CYTTSP gpio=%d r=%d \n", __func__,
		pdata->rst_gpio, rc);
	return rc;
}

static int power_init(struct cyttsp4_core_platform_data *data, bool on,
						struct device *dev)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	data->vdd = regulator_get(dev->parent, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(dev,
				"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, 2600000, 3300000);
		if (rc) {
			dev_err(dev, "Regulator set_vtg failed vdd rc=%d\n",
						rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(dev->parent, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(dev, "Regulator get failed vcc_i2c rc=%d\n",
					rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, 1800000, 1800000);
		if (rc) {
			dev_err(dev, "Regulator set_vtg failed vcc_i2c rc=%d\n",
						rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, 3300000);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, 3300000);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, 1800000);

	regulator_put(data->vcc_i2c);
	return 0;
}

static int power_on(struct cyttsp4_core_platform_data *data, bool on,
						struct device *dev)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(dev,
				"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(dev, "Regulator vcc_i2c enable failed rc=%d\n",
					rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(dev, "Regulator vdd disable failed rc=%d\n",
					rc);
		return rc;
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(dev, "Regulator vcc_i2c disable failed rc=%d\n",
					rc);
	}

	return rc;
}

static int cyttsp4_pinctrl_init(struct cyttsp4_core_platform_data *data, int on, struct device *dev)
{
	if (!on) {
		goto pinctrl_put;
	}

	data->pinctrl = devm_pinctrl_get(dev->parent);
	if (IS_ERR_OR_NULL(data->pinctrl)) {
		dev_err(dev, "%s:failed to get pinctrl!\n", __func__);
		return PTR_ERR(data->pinctrl);
	}

	data->gpio_state_active = pinctrl_lookup_state(data->pinctrl, "pmx_ts_active");
	if (IS_ERR_OR_NULL(data->gpio_state_active)) {
		dev_err(dev, "%s:failed to look up pmx_ts_active state!\n", __func__);
		return PTR_ERR(data->gpio_state_active);
	}

	data->gpio_state_suspend = pinctrl_lookup_state(data->pinctrl, "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(data->gpio_state_suspend)) {
		dev_err(dev, "%s:failed to look up pmx_ts_suspend state!\n", __func__);
		return PTR_ERR(data->gpio_state_suspend);
	}

	return 0;

pinctrl_put:
	devm_pinctrl_put(data->pinctrl);
	return 0;
}

static int cyttsp4_pinctrl_select(struct cyttsp4_core_platform_data *data, struct device *dev, int on)
{
	struct pinctrl_state *pins_state;
	int ret;

	pins_state = on ? data->gpio_state_active : data->gpio_state_suspend;
	if (IS_ERR_OR_NULL(pins_state)) {
		dev_err(dev, "not a valid '%s' pinstate\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
		return PTR_ERR(pins_state);
	}

	ret = pinctrl_select_state(data->pinctrl, pins_state);
	if (ret) {
		dev_err(dev, "cat no set '%s' pinstate\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
		return ret;
	}

	return 0;
}

int cyttsp4_init(struct cyttsp4_core_platform_data *pdata,
		int on, struct device *dev)
{
	int rst_gpio = pdata->rst_gpio;
	int irq_gpio = pdata->irq_gpio;
	int rc = 0;

	if (on) {
		rc = power_init(pdata, 1, dev);
		if (rc)
			dev_err(dev, "%s: power init failed\n", __func__);

		rc = power_on(pdata, 1, dev);
		if (rc)
			dev_err(dev, "%s: power on failed\n", __func__);

		rc = cyttsp4_pinctrl_init(pdata, 1, dev);
		if (rc)
			dev_err(dev, "%s: pinctrl init failed\n", __func__);

		rc = cyttsp4_pinctrl_select(pdata, dev, 1);
		if (rc)
			dev_err(dev, "%s: pinctrl select failed\n", __func__);

		rc = gpio_request(rst_gpio, NULL);
		if (rc < 0) {
			gpio_free(rst_gpio);
			rc = gpio_request(rst_gpio, NULL);
		}
		if (rc < 0) {
			dev_err(dev,
				"%s: Fail request gpio=%d\n", __func__,
				rst_gpio);
		} else {
			rc = gpio_direction_output(rst_gpio, 1);
			if (rc < 0) {
				pr_err("%s: Fail set output gpio=%d\n",
					__func__, rst_gpio);
				gpio_free(rst_gpio);
			} else {
				rc = gpio_request(irq_gpio, NULL);
				if (rc < 0) {
					gpio_free(irq_gpio);
					rc = gpio_request(irq_gpio,
						NULL);
				}
				if (rc < 0) {
					dev_err(dev,
						"%s: Fail request gpio=%d\n",
						__func__, irq_gpio);
					gpio_free(rst_gpio);
				} else {
					gpio_direction_input(irq_gpio);
				}
			}
		}
	} else {
		rc = cyttsp4_pinctrl_select(pdata, dev, 0);
		if(rc)
			dev_err(dev, "%s: pinctrl select failed\n", __func__);

		rc = cyttsp4_pinctrl_init(pdata, 0, dev);
		if (rc)
			dev_err(dev, "%s: pinctrl init failed\n", __func__);

		rc = power_on(pdata, 0, dev);
		if (rc)
			dev_err(dev, "%s: power on failed\n", __func__);
		rc = power_init(pdata, 0, dev);
		if (rc)
			dev_err(dev, "%s: power init failed\n", __func__);

		gpio_free(rst_gpio);
		gpio_free(irq_gpio);
	}

	dev_info(dev,
		"%s: INIT CYTTSP RST gpio=%d and IRQ gpio=%d r=%d\n",
		__func__, rst_gpio, irq_gpio, rc);
	return rc;
}

static int cyttsp4_wakeup(struct cyttsp4_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
	int irq_gpio = pdata->irq_gpio;
	int rc = 0;

	if (ignore_irq)
		atomic_set(ignore_irq, 1);
	rc = gpio_direction_output(irq_gpio, 0);
	if (rc < 0) {
		if (ignore_irq)
			atomic_set(ignore_irq, 0);
		dev_err(dev,
			"%s: Fail set output gpio=%d\n",
			__func__, irq_gpio);
	} else {
		udelay(2000);
		rc = gpio_direction_input(irq_gpio);
		if (ignore_irq)
			atomic_set(ignore_irq, 0);
		if (rc < 0) {
			dev_err(dev,
				"%s: Fail set input gpio=%d\n",
				__func__, irq_gpio);
		}
	}

	dev_info(dev,
		"%s: WAKEUP CYTTSP gpio=%d r=%d\n", __func__,
		irq_gpio, rc);
	return rc;
}

static int cyttsp4_sleep(struct cyttsp4_core_platform_data *pdata,
		struct device *dev, atomic_t *ignore_irq)
{
	return 0;
}

int cyttsp4_power(struct cyttsp4_core_platform_data *pdata,
		int on, struct device *dev, atomic_t *ignore_irq)
{
	if (on)
		return cyttsp4_wakeup(pdata, dev, ignore_irq);

	return cyttsp4_sleep(pdata, dev, ignore_irq);
}

int cyttsp4_irq_stat(struct cyttsp4_core_platform_data *pdata,
		struct device *dev)
{
	return gpio_get_value(pdata->irq_gpio);
}

#ifdef CYTTSP4_DETECT_HW
int cyttsp4_detect(struct cyttsp4_core_platform_data *pdata,
		struct device *dev, cyttsp4_platform_read read)
{
	int retry = 3;
	int rc;
	char buf[1];

	while (retry--) {
		/* Perform reset, wait for 100 ms and perform read */
		dev_vdbg(dev, "%s: Performing a reset\n", __func__);
		pdata->xres(pdata, dev);
		msleep(100);
		rc = read(dev, 0, buf, 1);
		if (!rc)
			return 0;

		dev_vdbg(dev, "%s: Read unsuccessful, try=%d\n",
			__func__, 3 - retry);
	}

	return rc;
}
#endif
