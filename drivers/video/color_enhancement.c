/*
 *  linux/drivers/video/color_enhancement.c
 *
 *  Copyright (C) 2013 Yulong
 *
 */

#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/ioctl.h>

#include <linux/color_enhancement.h>

static struct color_enhancement_t ce_data;

ssize_t color_enhancement_read(struct file *filp, char __user *buff,
		size_t count, loff_t *offp)
{
	return 0;
}

ssize_t color_enhancement_write(struct file *filp, const char __user *buff,
		size_t count, loff_t *offp)
{
	return 0;
}

static long color_enhancement_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	struct color_enhancement_t *ce = file->private_data;
	int param;

	if (!ce->impl_ops)
		return -ENOTTY;

	switch (cmd) {
	case CE_ENABLE:
		if (ce->impl_ops->enable)
			return ce->impl_ops->enable(ce);
		break;

	case CE_DISABLE:
		if (ce->impl_ops->disable)
			return ce->impl_ops->disable(ce);
		break;

	case CE_SET_LEVEL:
		get_user(param, (int __user *)arg);
		if (ce->impl_ops->set_level)
			return ce->impl_ops->set_level(ce, param);
		break;

	case CE_RESTORE_DEFAULT:
		if (ce->impl_ops->restore_default)
			return ce->impl_ops->restore_default(ce);
		break;

	case CE_SET_CABC_OFF:
		if (ce->impl_ops->cabc_off)
			return ce->impl_ops->cabc_off(ce);
		break;

	case CE_SET_CABC_UI_MODE:
		if (ce->impl_ops->ui_mode)
			return ce->impl_ops->ui_mode(ce);
		break;

	case CE_SET_CABC_STILL_MODE:
		if (ce->impl_ops->still_mode)
			return ce->impl_ops->still_mode(ce);
		break;

	case CE_SET_CABC_MOVING_MODE:
		if (ce->impl_ops->moving_mode)
			return ce->impl_ops->moving_mode(ce);
		break;

	case CE_SET_DAYLIGHT_MODE:
		break;

	case CE_SET_ACO_ON:
		if (ce->impl_ops->aco)
			return ce->impl_ops->aco(ce, 1);
		break;

	case CE_SET_ACO_OFF:
		if (ce->impl_ops->aco)
			return ce->impl_ops->aco(ce, 0);
		break;

	case CE_SET_OSC_ON:
		if (ce->impl_ops->osc)
			return ce->impl_ops->osc(ce, 1);
		break;

	case CE_SET_OSC_OFF:
		if (ce->impl_ops->osc)
			return ce->impl_ops->osc(ce, 0);
		break;

	case CE_INIT_SETTING:
		get_user(param, (int __user*)arg);
		if (ce->impl_ops->init_setting)
			return ce->impl_ops->init_setting(ce, param);
		break;
	}
	return -ENOTTY;
}

static int color_enhancement_open(struct inode *inode, struct file *file)
{
	if (file->private_data != &ce_data)
		file->private_data = &ce_data;
	return nonseekable_open(inode, file);
}

static int color_enhancement_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations ce_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = color_enhancement_read,
	.write = color_enhancement_write,
	.unlocked_ioctl = color_enhancement_ioctl,
	.open = color_enhancement_open,
	.release = color_enhancement_release,
};

static struct miscdevice ce_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "color_enhancement",
	.fops = &ce_fops,
};

/* Implementation specific operations */
extern struct ce_impl_ops_t ce_impl_ops;

static struct color_enhancement_t ce_data = {
	.impl_ops = &ce_impl_ops,
	.initialized = false,
};

static int __init color_enhancement_init(void)
{
	int ret;

	if (!ce_data.initialized) {
		ret = misc_register(&ce_dev);
		if (ret) {
			pr_err("%s: Unable to register color enhance: %d\n",
					__func__, ret);
			return ret;
		}

		if (ce_data.impl_ops->init)
			ce_data.impl_ops->init(&ce_data);

		ce_data.initialized = true;
	}

	return 0;
}
module_init(color_enhancement_init);

static void __exit color_enhancement_exit(void)
{
	if (ce_data.initialized) {
		misc_deregister(&ce_dev);
		ce_data.initialized = false;
	}
}
module_exit(color_enhancement_exit);

MODULE_AUTHOR("Yulong");
MODULE_DESCRIPTION("Yulong color enhancement driver");
