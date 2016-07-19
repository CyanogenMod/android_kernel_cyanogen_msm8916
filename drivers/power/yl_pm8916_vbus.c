/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/spmi.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/bitops.h>
#include "yl_pm8916_vbus.h"

#ifdef CONFIG_MACH_CP8675
#define TW_GLOVE_SWITCH 1
#else
#define TW_GLOVE_SWITCH 0
#endif

#define CREATE_MASK(NUM_BITS, POS) \
	((unsigned char) (((1 << (NUM_BITS)) - 1) << (POS)))
#define LBC_MASK(MSB_BIT, LSB_BIT) \
	CREATE_MASK(MSB_BIT - LSB_BIT + 1, LSB_BIT)

/* Interrupt offsets */
#define INT_RT_STS				0x10

/* USB CHARGER PATH peripheral register offsets */
#define USB_PTH_STS_REG				0x09
#define USB_IN_VALID_MASK			LBC_MASK(7, 6)
#define USBIN_RT_STS				BIT(1)

/* CHARGER peripheral register offset */
#define CHG_OPTION_REG				0x08
#define CHG_OPTION_MASK				BIT(7)
#define CHG_PERPH_RESET_CTRL3_REG		0xDA

#define PERP_SUBTYPE_REG			0x05
#define SEC_ACCESS                              0xD0

/* Linear peripheral subtype values */
#define LBC_CHGR_SUBTYPE			0x15
#define LBC_BAT_IF_SUBTYPE			0x16
#define LBC_USB_PTH_SUBTYPE			0x17
#define LBC_MISC_SUBTYPE			0x18

/* USB CHARGER PATH peripheral register offsets */
#define USB_SUSP_REG				0x47
#define USB_SUSPEND_BIT				BIT(0)

#define YL_PM8916_VBUS_DEV_NAME	"yulong,yl_pm8916_vbus"

/* usb_interrupts */

struct yl_pm8916_vbus_irq {
	int		irq;
	unsigned long	disabled;
};

enum {
	USBIN_VALID,
	BATT_PRES,
	MAX_IRQS,
};

/*
 * struct yl_pm8916_vbus_chip - device information
 * @dev:			device pointer to access the parent
 * @spmi:			spmi pointer to access spmi information
 * @chgr_base:			charger peripheral base address
 * @usb_chgpth_base:		USB charge path peripheral base address
 * @usb_present:		present status of usb
 * @usb_psy			power supply to export information to
 *				userspace
 *
 */
struct yl_pm8916_vbus_chip {
	struct device			*dev;
	struct spmi_device		*spmi;
	u16				base;
	u16				chgr_base;
	u16				usb_chgpth_base;
	u16				bat_if_base;
	struct yl_pm8916_vbus_irq		irqs[MAX_IRQS];
	bool				usb_present;
	int				batt_present;
	unsigned int			cfg_disable_follow_on_reset;
	struct power_supply		*usb_psy;
	struct power_supply		*batt_psy;
	struct qpnp_vadc_chip           *vadc_dev;

};

static int __qpnp_lbc_read(struct spmi_device *spmi, u16 base,
			u8 *value, int count)
{
	int rc;
	rc = spmi_ext_register_readl(spmi->ctrl, spmi->sid, base, value, count);
	if(rc)
		pr_err("SPMI read failed rc = %d\n", rc);
	return rc;
}
static int __qpnp_lbc_write(struct spmi_device *spmi, u16 base,
			u8 *val, int count)
{
	int rc;

	rc = spmi_ext_register_writel(spmi->ctrl, spmi->sid, base, val,
					count);
	if (rc)
		pr_err("SPMI write failed base=0x%02x sid=0x%02x rc=%d\n",
				base, spmi->sid, rc);

	return rc;
}

static int __qpnp_lbc_write_base(struct spmi_device *spmi, u16 addr,
			u8 mask, u8 val)
{
	int rc;
	u8 reg;

	rc = __qpnp_lbc_read(spmi, addr, &reg,  1);
	if(rc) {
		pr_err("read failed add = %03X, rc = %d\n", addr, rc);
		return rc;
	}

	reg &= ~mask;
	reg |= val & mask;
	rc = __qpnp_lbc_write(spmi, addr, &reg, 1);
	if(rc)
		pr_err("write failed addr = %03X, val = %02x, mask = %02x, reg = %02x, rc = %d\n",
					addr, val, mask, reg, rc);

	return rc;
}

static int __qpnp_lbc_secure_write(struct spmi_device *spmi, u16 base,
				u16 offset, u8 *val, int count)
{
	int rc;
	u8 reg_val;

	reg_val = 0xA5;
	rc = __qpnp_lbc_write(spmi, base + SEC_ACCESS, &reg_val, 1);
	if (rc) {
		pr_err("SPMI read failed base=0x%02x sid=0x%02x rc=%d\n",
				base + SEC_ACCESS, spmi->sid, rc);
		return rc;
	}

	rc = __qpnp_lbc_write(spmi, base + offset, val, 1);
	if (rc)
		pr_err("SPMI write failed base=0x%02x sid=0x%02x rc=%d\n",
				base + SEC_ACCESS, spmi->sid, rc);

	return rc;
}

static struct yl_pm8916_vbus_chip *this_chip;

static int yl_pm8916_vbus_read(struct yl_pm8916_vbus_chip *chip, u8 *val,
						u16 base, int count)
{
	int rc = 0;
	struct spmi_device *spmi = chip->spmi;

	if (base == 0) {
		pr_err("base cannot be zero base=0x%02x sid=0x%02x rc=%d\n",
							base, spmi->sid, rc);
		return -EINVAL;
	}

	rc = spmi_ext_register_readl(spmi->ctrl, spmi->sid, base, val, count);
	if (rc) {
		pr_err("SPMI read failed base=0x%02x sid=0x%02x rc=%d\n",
							base, spmi->sid, rc);
		return rc;
	}
	return 0;
}

static int yl_pm8916_vbus_is_usb_chg_plugged_in(struct yl_pm8916_vbus_chip *chip)
{
	u8 usbin_valid_rt_sts;
	int rc;

	rc = yl_pm8916_vbus_read(chip, &usbin_valid_rt_sts,
				 chip->usb_chgpth_base + USB_PTH_STS_REG, 1);
	if (rc) {
		pr_err("spmi read failed: addr=%03X, rc=%d\n",
				chip->usb_chgpth_base + USB_PTH_STS_REG, rc);
		return rc;
	}

	pr_debug("usb_path_sts 0x%x\n", usbin_valid_rt_sts);

	return (usbin_valid_rt_sts & USB_IN_VALID_MASK) ? 1 : 0;
}

#if TW_GLOVE_SWITCH
u32 yl_is_charger_in = 0; 
static struct delayed_work gtp_charger_status_check_work;
static struct workqueue_struct * gtp_charger_status_check_workqueue = NULL;
static void gtp_charger_status_check_func(struct work_struct *);
void yl_chg_status_changed(void);

u32 yl_get_charger_status(void)
{
    return yl_is_charger_in;  ///get charger status here. linronghui. 2014.03.07
}
EXPORT_SYMBOL(yl_get_charger_status);

static void gtp_charger_status_check_func(struct work_struct *work)
{
    yl_chg_status_changed(); 
}
#endif
int get_yl_pm8916_vbus_status(void)
{
	if (!this_chip) {
		pr_err("this_chip is unavailable\n");
		//return -EINVAL;
		return 1;
	}
	
	return yl_pm8916_vbus_is_usb_chg_plugged_in(this_chip);
}
EXPORT_SYMBOL(get_yl_pm8916_vbus_status);

#define BATT_MVOLT_DEFAULT      3800  /* 3800MV */
int get_yl_pm8916_batt_mvol(void)
{
	int rc = 0;
	struct qpnp_vadc_result results;
	if (!this_chip) {
		pr_err("this_chip is unavailable, battery voltage use default value 3800mV\n");
		//return -EINVAL;
		return BATT_MVOLT_DEFAULT;
	}
	
	rc = qpnp_vadc_read(this_chip->vadc_dev, VBAT_SNS, &results);
	if (rc) {
		pr_err("unable to read vbat rc = %d battery voltage use default value 3800mV\n", rc);
		//return 0;
		return BATT_MVOLT_DEFAULT;
	}
	
	pr_debug("read vbat= %lld \n", results.physical);
	return (int)results.physical/1000;	

}

int get_yl_pm8916_batt_temp(void)
{
	int rc = 0;
	struct qpnp_vadc_result results;
	if (!this_chip) {
		pr_err("this_chip is unavailable\n");
		return -EINVAL;
	}
	
	rc = qpnp_vadc_read(this_chip->vadc_dev, LR_MUX1_BATT_THERM, &results);
	if (rc) {
		pr_err("unable to read batt temp rc = %d \n", rc);
		return 0;
	}
	
	pr_debug("read batt temp= %lld \n", results.physical);
	return (int)results.physical;	

}

#define BATT_PRES_BIT BIT(7)
#define CHGR_BAT_IF_PRES_STATUS			0x08
int is_battery_present(void)
{
	u8 batt_present;
	int rc;

	if (!this_chip) {
		pr_err("this_chip is unavailable\n");
		return -EINVAL;
	}

	rc = yl_pm8916_vbus_read(this_chip, &batt_present,
				this_chip->bat_if_base + CHGR_BAT_IF_PRES_STATUS, 1);
	if(rc){
		pr_err("Couldn't read battery status read failed rc=%d\n", rc);
		return 0;
	}

	return (batt_present & BATT_PRES_BIT) ? 1 : 0;
}

#define BMS_SOC_REG			0xB0
#define BMS_OCV_REG			0xB1 /* B1 & B2 */
#define SOC_STORAGE_MASK		0xFE
#define OCV_INVALID			0xFFFF
#define SOC_INVALID			0xFF

int yl_backup_ocv_soc(int ocv_uv, int soc)
{
	int rc;
	
	if (!this_chip) {
		pr_err("this_chip is unavailable\n");
		return -EINVAL;
	}

	rc = __qpnp_lbc_write(this_chip->spmi, this_chip->base + BMS_OCV_REG, 
				(u8 *)&ocv_uv, 2);
	if(rc)
		pr_err("Unable to backup OCV rc=%d\n", rc);

	rc = __qpnp_lbc_write_base(this_chip->spmi, this_chip->base + BMS_SOC_REG,
				SOC_STORAGE_MASK, (soc +1) << 1);
	if(rc)
		pr_err("Unable to backup SOC rc=%d\n", rc);

	pr_debug("ocv_uv = %d, soc = %d\n", ocv_uv, soc);

	return rc;
}

int yl_read_shutdown_ocv_soc(int *ocv_uv, int *soc)
{
	u8 stored_soc = 0;
	u16 stored_ocv = 0;
	int rc;

	if (!this_chip) {
		pr_err("this_chip is unavailable\n");
		return -EINVAL;
	}
	
	rc = __qpnp_lbc_read(this_chip->spmi, this_chip->base + BMS_OCV_REG,
				(u8 *)&stored_ocv, 2);
	if (rc) {
		pr_err("failed to read addr = %d %d\n",
				this_chip->base + BMS_OCV_REG, rc);
		return -EINVAL;
	}

	/* if shutdwon ocv is invalid, reject shutdown soc too */
	if (!stored_ocv || (stored_ocv == OCV_INVALID)) {
		pr_err("shutdown OCV %d - invalid\n", stored_ocv);
		*ocv_uv= OCV_INVALID;
		*soc = SOC_INVALID;
		return -EINVAL;
	}

	*ocv_uv = stored_ocv;

	/*
	 * The previous SOC is stored in the first 7 bits of the register as
	 * (Shutdown SOC + 1). This allows for register reset values of both
	 * 0x00 and 0xFF.
	 */
	rc = __qpnp_lbc_read(this_chip->spmi, this_chip->base + BMS_SOC_REG,
	 			&stored_soc, 1);
	
	if (rc) {
			pr_err("failed to read addr = %d %d\n",
					this_chip->base + BMS_SOC_REG, rc);
			return -EINVAL;
		}
	
		if (!stored_soc || stored_soc == SOC_INVALID) {
			*soc = SOC_INVALID;
			*ocv_uv = OCV_INVALID;
			return -EINVAL;
		} else {
			*soc = (stored_soc >> 1) - 1;
		}
	
		pr_debug("shutdown_ocv=%d shutdown_soc=%d\n",
				*ocv_uv, *soc);

	return 0;
}


int read_yl_pm8916_batt_id(void)
{
	int rc = 0;
	struct qpnp_vadc_result results;
	if (!this_chip) {
		pr_err("this_chip is unavailable\n");
		return -EINVAL;
	}
	
	rc = qpnp_vadc_read(this_chip->vadc_dev, LR_MUX2_BAT_ID, &results);
	if (rc) {
		pr_err("unable to read batt id rc = %d \n", rc);
		return 0;
	}
	
	pr_debug("read batt id= %lld \n", results.physical);
	return (int)results.physical;	

}

static irqreturn_t yl_pm8916_vbus_batt_pres_irq_handler(int irq, void *_chip)
{
	struct yl_pm8916_vbus_chip *chip = _chip;
	int batt_present;

	batt_present = is_battery_present();
	pr_err("batt-pres triggered: %d\n", batt_present);

	if (chip->batt_present ^ batt_present) {
		chip->batt_present = batt_present;
		pr_debug("power supply changed batt_psy\n");
		power_supply_changed(chip->batt_psy);

	}
	return IRQ_HANDLED;
}


static irqreturn_t yl_pm8916_vbus_usbin_valid_irq_handler(int irq, void *_chip)
{
	struct yl_pm8916_vbus_chip *chip = _chip;
	int usb_present;

	usb_present = yl_pm8916_vbus_is_usb_chg_plugged_in(chip);
	printk(KERN_ERR"*===========================*\n");
	printk(KERN_ERR"VBUS_IRQ triggered: %s\n", usb_present?"VALID":"INVALID");
	printk(KERN_ERR"*===========================*\n");

    #if TW_GLOVE_SWITCH
	yl_is_charger_in = (u32)usb_present;
	queue_delayed_work(gtp_charger_status_check_workqueue, &gtp_charger_status_check_work, msecs_to_jiffies(10)); 
	#endif
	if (chip->usb_present ^ usb_present) {
		chip->usb_present = usb_present;
		pr_err("%s[%d] Updating usb_psy PRESENT property\n",__func__, __LINE__);
		power_supply_set_present(chip->usb_psy, chip->usb_present);
		
		//if (chip->usb_present) {
			power_supply_set_battery_charged(chip->batt_psy);
		//}
	}
	
	return IRQ_HANDLED;
}

#define SPMI_REQUEST_IRQ(chip, idx, rc, irq_name, threaded, flags, wake)\
do {									\
	if (rc)								\
		break;							\
	if (chip->irqs[idx].irq) {					\
		if (threaded)						\
			rc = devm_request_threaded_irq(chip->dev,	\
				chip->irqs[idx].irq, NULL,		\
				yl_pm8916_vbus_##irq_name##_irq_handler,	\
				flags, #irq_name, chip);		\
		else							\
			rc = devm_request_irq(chip->dev,		\
				chip->irqs[idx].irq,			\
				yl_pm8916_vbus_##irq_name##_irq_handler,	\
				flags, #irq_name, chip);		\
		if (rc < 0) {						\
			pr_err("Unable to request " #irq_name " %d\n",	\
								rc);	\
		} else {						\
			rc = 0;						\
			if (wake)					\
				enable_irq_wake(chip->irqs[idx].irq);	\
		}							\
	}								\
} while (0)

#define SPMI_GET_IRQ_RESOURCE(chip, rc, resource, idx, name)		\
do {									\
	if (rc)								\
		break;							\
									\
	rc = spmi_get_irq_byname(chip->spmi, resource, #name);		\
	if (rc < 0) {							\
		pr_err("Unable to get irq resource " #name "%d\n", rc);	\
	} else {							\
		chip->irqs[idx].irq = rc;				\
		rc = 0;							\
	}								\
} while (0)

static int yl_pm8916_vbus_request_irqs(struct yl_pm8916_vbus_chip *chip)
{
	int rc = 0;

	SPMI_REQUEST_IRQ(chip, BATT_PRES, rc, batt_pres, 1,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING
			| IRQF_ONESHOT, 1);
	

	SPMI_REQUEST_IRQ(chip, USBIN_VALID, rc, usbin_valid, 0,
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, 1);

	return 0;
}

static int yl_pm8916_vbus_get_irqs(struct yl_pm8916_vbus_chip *chip, u8 subtype,
					struct spmi_resource *spmi_resource)
{
	int rc = 0;

	switch (subtype) {
	case LBC_USB_PTH_SUBTYPE:
		SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
						USBIN_VALID, usbin-valid);
		if (rc < 0)
			return -ENXIO;
		break;
	case LBC_BAT_IF_SUBTYPE:
		SPMI_GET_IRQ_RESOURCE(chip, rc, spmi_resource,
						BATT_PRES, batt-pres);
		break;
	};

	return 0;
}

/* Get/Set initial state of charger */
static void determine_initial_status(struct yl_pm8916_vbus_chip *chip)
{
	u8 reg_val;
	int rc;

	chip->usb_present = yl_pm8916_vbus_is_usb_chg_plugged_in(chip);
	power_supply_set_present(chip->usb_psy, chip->usb_present);
	/*
	 * Set USB psy online to avoid userspace from shutting down if battery
	 * capacity is at zero and no chargers online.
	 */
	if (chip->usb_present)
		power_supply_set_online(chip->usb_psy, 1);
	
	/* if check the USB is present, need to inform charge */
	if (chip->usb_present) {
		power_supply_set_battery_charged(chip->batt_psy);
	}

	/*
	 * Configure peripheral reset control
	 * This is a workaround only for SLT testing.
	 */
	if (chip->cfg_disable_follow_on_reset) {
		reg_val = 0x0;
		rc = __qpnp_lbc_secure_write(chip->spmi, chip->chgr_base,
				CHG_PERPH_RESET_CTRL3_REG, &reg_val, 1);
		if (rc)
			pr_err("Failed to configure PERPH_CTRL3 rc=%d\n", rc);
		else
			pr_warn("Charger is not following PMIC reset\n");
	}
}
static int yl_pm8916_vbus_probe(struct spmi_device *spmi)
{
	u8 subtype;
	struct yl_pm8916_vbus_chip *chip;
	struct resource *resource;
	struct spmi_resource *spmi_resource;
	struct power_supply *usb_psy;
	struct power_supply *batt_psy;
	int rc = 0;
        pr_err("====probe enter===\n");
	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		pr_err("usb supply not found deferring probe\n");
		return -EPROBE_DEFER;
	}
	//batt_psy = power_supply_get_by_name("usb");
	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		pr_err("battery supply not found deferring probe\n");
		return -EPROBE_DEFER;
	}

	chip = devm_kzalloc(&spmi->dev, sizeof(struct yl_pm8916_vbus_chip),
				GFP_KERNEL);
	if (chip == NULL) {
		pr_err("memory allocation failed.\n");
		return -ENOMEM;
	}

	chip->usb_psy = usb_psy;
	chip->batt_psy = batt_psy;
	pr_err("-----------handler  name=%s\n", chip->batt_psy->name);
	chip->dev = &spmi->dev;
	chip->spmi = spmi;
	dev_set_drvdata(&spmi->dev, chip);
	device_init_wakeup(&spmi->dev, 1);
	
	chip->vadc_dev = qpnp_get_vadc(chip->dev, "chg");
	if (IS_ERR(chip->vadc_dev)) {
		rc = PTR_ERR(chip->vadc_dev);
		if (rc != -EPROBE_DEFER)
			pr_err("vadc_dev prop missing rc = %d \n", rc);
		goto fail_chg_enable;
	}

	spmi_for_each_container_dev(spmi_resource, spmi) {
		if (!spmi_resource) {
			pr_err("spmi resource absent\n");
			rc = -ENXIO;
			goto fail_chg_enable;
		}

		resource = spmi_get_resource(spmi, spmi_resource,
							IORESOURCE_MEM, 0);
		if (!(resource && resource->start)) {
			pr_err("node %s IO resource absent!\n",
						spmi->dev.of_node->full_name);
			rc = -ENXIO;
			goto fail_chg_enable;
		}

		pr_err("Node name = %s\n", spmi_resource->of_node->name);
		if(strcmp("qcom,vm-bms",spmi_resource->of_node->name) == 0){
			chip->base = resource->start;
			pr_err("chip->base = %x\n", chip->base);
			continue;
		}

		rc = yl_pm8916_vbus_read(chip, &subtype,
				resource->start + PERP_SUBTYPE_REG, 1);
		if (rc) {
			pr_err("Peripheral subtype read failed rc=%d\n", rc);
			goto fail_chg_enable;
		}

		switch (subtype) {
		case LBC_USB_PTH_SUBTYPE:
			pr_err("USB subtype=0x%x\n", subtype);
			chip->usb_chgpth_base = resource->start;
			
			rc = yl_pm8916_vbus_get_irqs(chip, subtype, spmi_resource);
			if (rc)
				goto fail_chg_enable;
			break;


		case LBC_BAT_IF_SUBTYPE:
			pr_err("bat_if  subtype=0x%x\n", subtype);
			chip->bat_if_base = resource->start;

			rc = yl_pm8916_vbus_get_irqs(chip, subtype, spmi_resource);
			if (rc)
				goto fail_chg_enable;
			break;

		default:
			pr_err("Invalid peripheral subtype=0x%x\n", subtype);
			//rc = -EINVAL;
		}

	}

	this_chip = chip;
	chip->batt_present = is_battery_present();
	/* Initial check if USB already inserted */
	determine_initial_status(chip);

	#if TW_GLOVE_SWITCH
	if(yl_pm8916_vbus_is_usb_chg_plugged_in(chip))
		    yl_is_charger_in=1; 
	
	INIT_DELAYED_WORK(&gtp_charger_status_check_work, gtp_charger_status_check_func);
    gtp_charger_status_check_workqueue = create_workqueue("gtp_charger_status_check");

	#endif
	rc = yl_pm8916_vbus_request_irqs(chip);
	if (rc) {
		pr_err("failed to request interrupts %d\n", rc);
		goto fail_chg_enable;
	}
	//yl_pm8916_vbus_usbin_valid_irq_handler(chip->irqs[USBIN_VALID].irq,chip);

	/* add for fix the bug lost vbus irq when remove vbus  */
	rc = __qpnp_lbc_write_base(chip->spmi, chip->usb_chgpth_base + USB_SUSP_REG,
				USB_SUSPEND_BIT, 1);
	if(rc)
		pr_err("Unable to set USB_SUSP_REG rc=%d\n", rc);
	

	pr_info("LBC probed USB is %s\n", chip->usb_present ? "connected" :"not conneccted");
        pr_info("=====probe success====\n");
	return 0;

fail_chg_enable:
	dev_set_drvdata(&spmi->dev, NULL);
	return rc;
}

static int yl_pm8916_vbus_remove(struct spmi_device *spmi)
{
	dev_set_drvdata(&spmi->dev, NULL);
	return 0;
}

static struct of_device_id yl_pm8916_vbus_match_table[] = {
	{ .compatible = YL_PM8916_VBUS_DEV_NAME, },
	{}
};

static struct spmi_driver yl_pm8916_vbus_driver = {
	.probe		= yl_pm8916_vbus_probe,
	.remove		= yl_pm8916_vbus_remove,
	.driver		= {
		.name		= YL_PM8916_VBUS_DEV_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= yl_pm8916_vbus_match_table,
	},
};

/*
 * yl_pm8916_vbus_init() - register spmi driver for charge
 */
static int __init yl_pm8916_vbus_init(void)
{
	return spmi_driver_register(&yl_pm8916_vbus_driver);
}
module_init(yl_pm8916_vbus_init);

static void __exit yl_pm8916_vbus_exit(void)
{
	spmi_driver_unregister(&yl_pm8916_vbus_driver);
}
module_exit(yl_pm8916_vbus_exit);

MODULE_DESCRIPTION("yulong add pm8916 vubs driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" YL_PM8916_VBUS_DEV_NAME);
