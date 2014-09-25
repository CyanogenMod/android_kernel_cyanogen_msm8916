/* FPC1080 Swipe Sensor Driver
 *
 * Copyright (c) 2011 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#define DEBUG

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/mm.h>

#include <linux/workqueue.h>
#include <linux/cpumask.h>
#include <linux/smp.h>

#include <linux/sensors/fpc1080.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Fingerprint Cards AB <tech@fingerprints.com>");
MODULE_DESCRIPTION("FPC1080 swipe sensor driver.");

/* -------------------------------------------------------------------- */
/* fpc1080 sensor commands and registers				*/
/* -------------------------------------------------------------------- */
#define FPC1080_ACTIVATE_SLEEP			0x28
#define FPC1080_ACTIVATE_DEEP_SLEEP		0x2C
#define FPC1080_ACTIVATE_NAVIGATION		0x30
#define FPC1080_ACTIVATE_IDLE			0x34
#define FPC1080_RD_INTERRUPT_WITH_CLEAR		0x64
#define FPC1080_RD_INTERRUPT_WITH_NO_CLEAR	0x18
#define FPC1080_WAIT_FOR_FINGER_PRESENT		0x24
#define FPC1080_READ_AND_CAPTURE_IMAGE		0xCC
#define FPC1080_READ_IMAGE_DATA			0xC4
#define FPC1080_CAPTURE_IMAGE			0xC0
#define FPC1080_SET_SMART_REF			0x48

#define FPC1080_FNGR_DWN_MIN			0x38
#define FPC1080_FNGR_DWN_MAX			0x3C
#define FPC1080_FNGR_DWN_MID			0x4C
#define FPC1080_DX_THRESHOLD			0x54
#define FPC1080_DY_THRESHOLD			0x58
#define FPC1080_FNGR_DET_THRESHOLD		0x5C
#define FPC1080_FNGR_LOST_THRESHOLD		0x60
#define FPC1080_ADC_OFFSET			0xA0
#define FPC1080_ADC_GAIN			0xA4
#define FPC1080_PXL_SETUP			0xA8
#define FPC1080_NAV_CNTR			0x80
#define FPC1080_FNGR_DRIVE_CONF			0x1C
#define FPC1080_SMRT_DATA			0x10
#define FPC1080_STATUS				0x14
#define FPC1080_REG_HW_ID			0x9C

#define FPC1080_STATUS_IRQ			(1 << 0u)
#define FPC1080_STATUS_FSM_IDLE			(1 << 1u)

#define FPC1080_SMRT_MOTION_EST_BIT_8		(1 << 0u)
#define FPC1080_SMRT_MOTION_EST_BIT_9		(1 << 1u)
#define FPC1080_SMRT_SHORT_CLICK		(1 << 2u)
#define FPC1080_SMRT_LONG_CLICK			(1 << 3u)
#define FPC1080_SMRT_X_SIGN			(1 << 4u)
#define FPC1080_SMRT_Y_SIGN			(1 << 5u)
#define FPC1080_SMRT_X_BYTE			4
#define FPC1080_SMRT_Y_BYTE			3
#define FPC1080_SMRT_MO_CNTR_BYTE		2
#define FPC1080_SMRT_MO_EST_BYTE		1
#define FPC1080_SMRT_BITS			0

#define FPC1080_SPI_FNGR_DRV_TST		(1 << 2u)
#define FPC1080_SPI_FNGR_DRV_EXT		(1 << 1u)
#define FPC1080_SPI_SMRT_SENS_EN		(1 << 0u)

#define	FPC1080_PATTERN1_XREG			0x78
#define FPC1080_PATTERN2_XREG			0x7C

#define FPC1080_IRQ_REBOOT			0xFF
#define FPC1080_IRQ_CMD_DONE			(1 << 7u)
#define FPC1080_IRQ_DY				(1 << 6u)
#define FPC1080_IRQ_DX				(1 << 5u)
#define FPC1080_IRQ_FING_LOST			(1 << 4u)
#define FPC1080_IRQ_SHORT_CLICK			(1 << 3u)
#define FPC1080_IRQ_LONG_CLICK			(1 << 2u)
#define FPC1080_IRQ_FING_UP			(1 << 1u)
#define FPC1080_IRQ_FING_DOWN			(1 << 0u)


/* -------------------------------------------------------------------- */
/* fpc1080 driver constants						*/
/* -------------------------------------------------------------------- */
#define FPC1080_HARDWARE_ID			0x1A

#define FPC1080_SYNCED_REG_SIZE			2
#define FPC1080_MOTION_THRESHOLD		25
#define FPC1080_MOTON_FRAMES_THERSHOLD		50

#define FPC1080_MAJOR				235

#define FPC1080_DEFAULT_IRQ_TIMEOUT		(100 * HZ / 1000)
#define FPC1080_FRAME_SIZE			(128 * 8)
#define FPC1080_MAX_FRAMES			256
#define FPC1080_IMAGE_BUFFER_SIZE		(FPC1080_MAX_FRAMES * \
						 FPC1080_FRAME_SIZE)

#define FPC1080_SPI_CLOCK_SPEED			(12 * 1000 * 1000)

#define FPC1080_DEV_NAME                        "fpc1080"
#define FPC1080_CLASS_NAME                      "fpsensor"
#define FPC1080_WORKER_THREAD_NAME		"fpc1080worker"

#define FPC1080_IOCTL_MAGIC_NO			0xFC

#define FPC1080_IOCTL_START_CAPTURE	_IO(FPC1080_IOCTL_MAGIC_NO, 0)
#define FPC1080_IOCTL_ABORT_CAPTURE	_IO(FPC1080_IOCTL_MAGIC_NO, 1)
#define FPC1080_IOCTL_CAPTURE_SINGLE	_IOW(FPC1080_IOCTL_MAGIC_NO, 2, int)

#define FPC1080_KEY_FINGER_PRESENT		188

enum {
	FPC1080_THREAD_IDLE_MODE = 0,
	FPC1080_THREAD_CAPTURE_MODE,
	FPC1080_THREAD_NAV_MODE,
	FPC1080_THREAD_EXIT
};

/* -------------------------------------------------------------------- */
/* global variables							*/
/* -------------------------------------------------------------------- */
static int fpc1080_device_count;

/* -------------------------------------------------------------------- */
/* fpc1080 data types							*/
/* -------------------------------------------------------------------- */
struct fpc1080_thread_task {
	int mode;
	int should_stop;
	struct semaphore sem_idle;
	wait_queue_head_t wait_job;
	struct task_struct *thread;
};

struct fpc1080_diag {
	u8 selftest;
	u32 capture_time;
	u32 frames_captured;
	u32 frames_stored;
};

struct fpc1080_data {
	struct spi_device *spi;
	struct class *class;
	struct device *device;
	struct cdev cdev;
	dev_t devno;
	u32 reset_gpio;
	u32 irq_gpio;
	u32 irq;
	u32 data_offset;
	u32 avail_data;
	wait_queue_head_t waiting_data_avail;
	int interrupt_done;
	wait_queue_head_t waiting_interrupt_return;
	struct semaphore mutex;
	u8 *huge_buffer;
	u32 current_frame;
	int capture_done;
	struct fpc1080_thread_task thread_task;
	struct fpc1080_adc_setup adc_setup;
	struct fpc1080_diag diag;
	int open_counter;
    struct work_struct probe_work;
    struct fpc1080_platform_data *pdata;
#ifdef CONFIG_FPC1080_NAVIGATION
	struct input_dev *nav_dev;
#endif
};

struct fpc1080_attribute {
	struct device_attribute attr;
	size_t offset;
};

/* -------------------------------------------------------------------- */
/* function prototypes							*/
/* -------------------------------------------------------------------- */
static int __init fpc1080_init(void);

static void __exit fpc1080_exit(void);

static int __devinit fpc1080_probe(struct spi_device *spi);

static int __devexit fpc1080_remove(struct spi_device *spi);

static int fpc1080_suspend(struct device *dev);

static int fpc1080_resume(struct device *dev);

static int fpc1080_open(struct inode *inode, struct file *file);

static ssize_t fpc1080_write(struct file *file, const char *buff,
					size_t count, loff_t *ppos);

static ssize_t fpc1080_read(struct file *file, char *buff,
				size_t count, loff_t *ppos);

static int fpc1080_release(struct inode *inode, struct file *file);

static unsigned int fpc1080_poll(struct file *file, poll_table *wait);

static long fpc1080_ioctl(struct file *filp,
			  unsigned int cmd,
			  unsigned long arg);

static int fpc1080_reset(struct fpc1080_data *fpc1080);

static ssize_t fpc1080_show_attr_adc_setup(struct device *dev,
				struct device_attribute *attr, char *buf);

static ssize_t fpc1080_store_attr_adc_setup(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);

static ssize_t fpc1080_show_attr_diag(struct device *dev,
				struct device_attribute *attr, char *buf);

static ssize_t fpc1080_store_attr_diag(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count);

#ifdef CONFIG_FPC1080_NAVIGATION
static int fpc1080_stop_navigation(struct fpc1080_data *fpc1080);
static int fpc1080_start_navigation(struct fpc1080_data *fpc1080);
static int fpc1080_nav_task(struct fpc1080_data *fpc1080);
#endif

static int fpc1080_wreg(struct fpc1080_data *fpc1080, u8 addr, u8 value);
static int fpc1080_write_adc_setup(struct fpc1080_data *fpc1080);
static int fpc1080_cmd_wait_irq(struct fpc1080_data *fpc1080, u8 cmd, u8 *irq);
static int fpc1080_spi_rd_image(struct fpc1080_data *fpc1080, bool capture);
static int fpc1080_selftest_short(struct fpc1080_data *fpc1080);

/* -------------------------------------------------------------------- */
/* External interface							*/
/* -------------------------------------------------------------------- */
module_init(fpc1080_init);
module_exit(fpc1080_exit);

static const struct dev_pm_ops fpc1080_pm = {
	.suspend = fpc1080_suspend,
	.resume = fpc1080_resume
};

#ifdef CONFIG_OF
static int fpc1080_get_configs(struct device *dev, char *name, u32 *array)
{
	int i, rc;
	struct property *prop;
	struct device_node *np = dev->of_node;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	rc = of_property_read_u32_array(np, name, array, prop->length/sizeof(u32));
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "%s: Unable to read %s\n", __func__, name);
		return rc;
	}

	dev_info(dev, "%s size is %d\n", name, prop->length/sizeof(u32));
	for (i = 0; i < prop->length/sizeof(u32); i++) {
		dev_info(dev, "arrary[%d]=%d, ", i, array[i]);
	}
	dev_info(dev, "\n");
	return rc;
}

static int fpc1080_parse_dt(struct device *dev, struct fpc1080_platform_data *pdata)
{
	int ret;
	u32 value;
	u32 array[12];
	enum of_gpio_flags flags;
	struct device_node *np = dev->of_node;
	
	/* parse irq and request gpio */
	value = of_get_named_gpio_flags(np, "fpc,reset-gpio", 0, &flags);
	pdata->reset_gpio = value;
	value = of_get_named_gpio_flags(np, "fpc,irq-gpio", 0, &flags);
	pdata->irq_gpio = value;
	dev_info(dev, "%s: irq gpio num is %d, reset gpio num is %d\n", __func__, pdata->irq_gpio, pdata->reset_gpio);

	if (pdata->irq_gpio < 0 || pdata->reset_gpio < 0) {
		return -EINVAL;
	}

	/* general_reg */
	ret = fpc1080_get_configs(dev, "fpc,adc_setup", array);
	if (ret) {
		dev_err(dev, "Looking up %s property in node %s failed", "fpc,adc_setup", np->full_name);
		return -ENODEV;
	}
	pdata->adc_setup.offset = array[0];
	pdata->adc_setup.gain = array[1];
	pdata->adc_setup.pxl_setup = array[2];

	/* proximity_reg */
	ret = fpc1080_get_configs(dev, "fpc,nav", array);
	if (ret) {
		dev_err(dev, "Looking up %s property in node %s failed", "fpc,nav", np->full_name);
		return -ENODEV;
	}
	pdata->nav.finger_down_min = array[0];
	pdata->nav.finger_down_mid = array[1];
	pdata->nav.finger_down_max = array[2];
	pdata->nav.finger_detect_thr = array[3];
	pdata->nav.finger_lost_thr = array[4];
	pdata->nav.dx_thr = array[5];
	pdata->nav.dy_thr = array[6];
	pdata->nav.nav_cntr = array[7];
	pdata->nav.off_x_axis_thr = array[8];
	pdata->nav.off_y_axis_thr = array[9];
	pdata->nav.enabled = array[10];
#if 0
	ret = of_property_read_string(np, "atmel,vcc_i2c_supply", &pdata->ts_vcc_i2c);
	ret = of_property_read_string(np, "atmel,vdd_ana_supply", &pdata->ts_vdd_ana);
	printk("pwr_en=%d, sleep_pwr_en=%d, vcc_i2c=%s, vdd_ana=%s\n", pdata->pwr_en,
			pdata->sleep_pwr_en, pdata->ts_vcc_i2c, pdata->ts_vdd_ana);
#endif
	return 0;	
}

static struct of_device_id fpc1080_match_table[] = {
	{ .compatible = FPC1080_DEV_NAME,},
	{ },
};
#else
#define fpc1080_match_table NULL
#endif

static struct spi_driver fpc1080_driver = {
	.driver = {
		.name	= FPC1080_DEV_NAME,
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
		.pm = &fpc1080_pm,
	#ifdef CONFIG_OF
		.of_match_table = fpc1080_match_table,
	#endif
	},
	.probe	= fpc1080_probe,
	.remove	= __devexit_p(fpc1080_remove)
};

static const struct file_operations fpc1080_fops = {
	.owner = THIS_MODULE,
	.open  = fpc1080_open,
	.write = fpc1080_write,
	.read  = fpc1080_read,
	.release = fpc1080_release,
	.poll = fpc1080_poll,
	.unlocked_ioctl = fpc1080_ioctl
};

/* -------------------------------------------------------------------- */
/* devfs								*/
/* -------------------------------------------------------------------- */
#define FPC1080_ATTR(__grp, __field, __mode)				\
{									\
	.attr = __ATTR(__field, (__mode),				\
	fpc1080_show_attr_##__grp,					\
	fpc1080_store_attr_##__grp),					\
	.offset = offsetof(struct fpc1080_##__grp, __field)		\
}

#define FPC1080_DEV_ATTR(_grp, _field, _mode)				\
struct fpc1080_attribute fpc1080_attr_##_field =			\
					FPC1080_ATTR(_grp, _field, (_mode))

#define ADC_SETUP_MODE (S_IWUSR | S_IWGRP | S_IWOTH)

static FPC1080_DEV_ATTR(adc_setup, gain,	ADC_SETUP_MODE);
static FPC1080_DEV_ATTR(adc_setup, offset,	ADC_SETUP_MODE);
static FPC1080_DEV_ATTR(adc_setup, pxl_setup,	ADC_SETUP_MODE);

static struct attribute *fpc1080_adc_attrs[] = {
	&fpc1080_attr_gain.attr.attr,
	&fpc1080_attr_offset.attr.attr,
	&fpc1080_attr_pxl_setup.attr.attr,
	NULL
};

static const struct attribute_group fpc1080_adc_attr_group = {
	.attrs = fpc1080_adc_attrs,
	.name = "adc_setup"
};

#define DIAG_MODE (S_IRUSR | S_IRGRP | S_IROTH)

static FPC1080_DEV_ATTR(diag, selftest,		DIAG_MODE);
static FPC1080_DEV_ATTR(diag, capture_time,	DIAG_MODE);
static FPC1080_DEV_ATTR(diag, frames_captured,	DIAG_MODE);
static FPC1080_DEV_ATTR(diag, frames_stored,	DIAG_MODE);

static struct attribute *fpc1080_diag_attrs[] = {
	&fpc1080_attr_selftest.attr.attr,
	&fpc1080_attr_capture_time.attr.attr,
	&fpc1080_attr_frames_captured.attr.attr,
	&fpc1080_attr_frames_stored.attr.attr,
	NULL
};

static const struct attribute_group fpc1080_diag_attr_group = {
	.attrs = fpc1080_diag_attrs,
	.name = "diag"
};


/* -------------------------------------------------------------------- */
/* function definitions							*/
/* -------------------------------------------------------------------- */
static ssize_t fpc1080_show_attr_adc_setup(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return -ENOTTY;
	/*u8 *target;
	struct fpc1080_data *fpc1080;
	struct fpc1080_attribute *fpc_attr;
	fpc1080 = dev_get_drvdata(dev);
	fpc_attr = container_of(attr, struct fpc1080_attribute, attr);

	target = ((u8 *)&fpc1080->nav_settings) + fpc_attr->offset;
	return scnprintf(buf, PAGE_SIZE, "%s: %i\n", attr->attr.name,
				*target);*/

}

/* -------------------------------------------------------------------- */
static ssize_t fpc1080_store_attr_adc_setup(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	u8 *target;
	u8 tmp;
	struct fpc1080_data *fpc1080;
	struct fpc1080_attribute *fpc_attr;
	fpc1080 = dev_get_drvdata(dev);
	fpc_attr = container_of(attr, struct fpc1080_attribute, attr);

	if ((sscanf(buf, "%hhu", &tmp)) <= 0)
		return -EINVAL;

	target = ((u8 *)&fpc1080->adc_setup) + fpc_attr->offset;
	*target = tmp;

	return strnlen(buf, count);
}

/* -------------------------------------------------------------------- */
static ssize_t fpc1080_show_attr_diag(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fpc1080_data *fpc1080;
	struct fpc1080_attribute *fpc_attr;

	fpc1080 = dev_get_drvdata(dev);
	fpc_attr = container_of(attr, struct fpc1080_attribute, attr);

	if (fpc_attr->offset == offsetof(struct fpc1080_diag, selftest)) {

		fpc1080_selftest_short(fpc1080);

		return scnprintf(buf, PAGE_SIZE, "%i\n",
				 fpc1080->diag.selftest);
	}

	if (fpc_attr->offset == offsetof(struct fpc1080_diag, capture_time)) {
		return scnprintf(buf, PAGE_SIZE, "%i\n",
				 fpc1080->diag.capture_time);
	}

	if(fpc_attr->offset == offsetof(struct fpc1080_diag, frames_captured)) {
		return scnprintf(buf, PAGE_SIZE, "%i\n",
				 fpc1080->diag.frames_captured);
	}

	if (fpc_attr->offset == offsetof(struct fpc1080_diag, frames_stored)) {
		return scnprintf(buf, PAGE_SIZE, "%i\n",
				 fpc1080->diag.frames_stored);
	}

	return -ENOENT;
}

/* -------------------------------------------------------------------- */
static ssize_t fpc1080_store_attr_diag(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	return -EPERM;
}

/* -------------------------------------------------------------------- */
static int fpc1080_wait_for_irq(struct fpc1080_data *fpc1080,
							int timeout)
{
	int result;

	if (!timeout) {
		result = wait_event_interruptible(
				fpc1080->waiting_interrupt_return,
				fpc1080->interrupt_done);
	} else {
		result = wait_event_interruptible_timeout(
				fpc1080->waiting_interrupt_return,
				fpc1080->interrupt_done, timeout);
	}

	if (result < 0) {
		dev_err(&fpc1080->spi->dev, "wait_event_interruptible "
				"interrupted by signal.\n");
		return result;
	}

	if (result || !timeout) {
		fpc1080->interrupt_done = 0;
		return 0;
	}

	return -ETIMEDOUT;
}

/* -------------------------------------------------------------------- */
static int fpc1080_spi_wr_rd(struct fpc1080_data *fpc1080, u8 *tx,
					u8 *rx, unsigned int length)
{
	int error;
	struct spi_message m;
	struct spi_transfer t = {
		.cs_change = 0,  
		.delay_usecs = 0,
		.speed_hz = FPC1080_SPI_CLOCK_SPEED,
		.tx_buf = tx,
		.rx_buf = rx,
		.len = length,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	error = spi_sync(fpc1080->spi, &m);
	if (error) {
		dev_err(&fpc1080->spi->dev, "spi_sync failed.\n");
		return error;
	}

	fpc1080->avail_data = length;
	fpc1080->data_offset = 0;
	return 0;
}

/* -------------------------------------------------------------------- */
static int fpc1080_spi_wr_reg(struct fpc1080_data *fpc1080, u8 addr,
						u8 value, unsigned int size)
{
	int error;
	fpc1080->huge_buffer[0] = addr;
	fpc1080->huge_buffer[1] = value;
	error = fpc1080_spi_wr_rd(fpc1080, fpc1080->huge_buffer,
				fpc1080->huge_buffer + size, size);
	if (error)
		return error;

	fpc1080->data_offset = size > FPC1080_SYNCED_REG_SIZE ?
				size + FPC1080_SYNCED_REG_SIZE : size + 1;
	return 0;
}

/* -------------------------------------------------------------------- */
static int fpc1080_reset(struct fpc1080_data *fpc1080)
{
	int error = 0;
	gpio_set_value(fpc1080->reset_gpio, 0);
	udelay(1000);
	gpio_set_value(fpc1080->reset_gpio, 1);
	udelay(1250);
	error = gpio_get_value(fpc1080->irq_gpio) ? 0 : -EIO;
	if (error) {
		printk(KERN_INFO "reset timed out, waiting again..\n");
		udelay(1250);
		error = gpio_get_value(fpc1080->irq_gpio) ? 0 : -EIO;
	}

	disable_irq(fpc1080->irq);
	fpc1080->interrupt_done = 0;
	enable_irq(fpc1080->irq);

	if (error) {
		dev_err(&fpc1080->spi->dev, "irq after reset timed out\n");
		return -EIO;
	}

	error = fpc1080_spi_wr_reg(fpc1080, FPC1080_RD_INTERRUPT_WITH_CLEAR, 0, 2);
	if (error) {
		return error;
	}

	if (fpc1080->huge_buffer[fpc1080->data_offset] != FPC1080_IRQ_REBOOT) {
		dev_err(&fpc1080->spi->dev, "unexpected response at reset.\n");
		return -EIO;
	}
	fpc1080->data_offset = 0;
	fpc1080->avail_data = 0;
	return 0;
}

/* -------------------------------------------------------------------- */
static int fpc1080_capture_single(struct fpc1080_data *fpc1080, int mode)
{
	int error;
	u8 pat1, pat2, irq;
	pat1 = 0x55;
	pat2 = 0xAA;
	switch (mode) {
	case 0:
		break;
	case 1:
		pat1 = 0xAA;
		pat2 = 0x55;

	case 2:
		error = fpc1080_wreg(fpc1080, FPC1080_FNGR_DRIVE_CONF,
				     FPC1080_SPI_FNGR_DRV_TST);
		if (error)
			return error;
		error = fpc1080_wreg(fpc1080, FPC1080_PATTERN1_XREG, pat1);
		if (error)
			return error;
		error = fpc1080_wreg(fpc1080, FPC1080_PATTERN2_XREG, pat2);
		if (error)
			return error;
		break;
	default:
		return -EINVAL;

	}

	error = fpc1080_write_adc_setup(fpc1080);
	if (error)
		return error;
	error = fpc1080_cmd_wait_irq(fpc1080, FPC1080_CAPTURE_IMAGE, &irq);
	if (error)
		return error;
	return fpc1080_spi_rd_image(fpc1080, 0);
}

/* -------------------------------------------------------------------- */
static int fpc1080_spi_rd_image(struct fpc1080_data *fpc1080, bool capture)
{
	int error;
	u8 tx[2];
	struct spi_message spi_mess;
	struct spi_transfer trans_rd_cap1;
	struct spi_transfer trans_rd_cap2;
	memset(&trans_rd_cap1, 0 , sizeof(struct spi_transfer));
	memset(&trans_rd_cap2, 0 , sizeof(struct spi_transfer));
	if (fpc1080->current_frame >= FPC1080_MAX_FRAMES)
		return -ENOBUFS;

	tx[0] = capture ? FPC1080_READ_AND_CAPTURE_IMAGE :
			  FPC1080_READ_IMAGE_DATA;

	trans_rd_cap1.cs_change = 0;
	trans_rd_cap1.delay_usecs = 0;
	trans_rd_cap1.speed_hz = FPC1080_SPI_CLOCK_SPEED;
	trans_rd_cap1.tx_buf = tx;
	trans_rd_cap1.rx_buf = NULL;
	trans_rd_cap1.len = 2;

	trans_rd_cap2.cs_change = 0;
	trans_rd_cap2.delay_usecs = 0;
	trans_rd_cap2.speed_hz = FPC1080_SPI_CLOCK_SPEED;
	trans_rd_cap2.tx_buf = NULL;
	trans_rd_cap2.rx_buf = fpc1080->huge_buffer +
			       fpc1080->current_frame * FPC1080_FRAME_SIZE;
	trans_rd_cap2.len = 1024;

	spi_message_init(&spi_mess);
	spi_message_add_tail(&trans_rd_cap1, &spi_mess);
	spi_message_add_tail(&trans_rd_cap2, &spi_mess);

	error = spi_sync(fpc1080->spi, &spi_mess);
	if (error)
		return error;

	fpc1080->current_frame++;
	fpc1080->avail_data += FPC1080_FRAME_SIZE;
	wake_up_interruptible(&fpc1080->waiting_data_avail);

	return 0;
}

/* -------------------------------------------------------------------- */
static int fpc1080_spi_read_wait_irq(struct fpc1080_data *fpc1080, u8 *irq)
{
	int error;
	u8 buf[4];
	struct spi_message m;

	struct spi_transfer t = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = FPC1080_SPI_CLOCK_SPEED,
		.tx_buf = buf,
		.rx_buf = buf + 2,
		.len = 2,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};
	buf[0] = FPC1080_RD_INTERRUPT_WITH_CLEAR;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	while (1) {
		error = fpc1080_wait_for_irq(fpc1080,
					     FPC1080_DEFAULT_IRQ_TIMEOUT);
		if (error == 0)
			break;
		if (fpc1080->thread_task.should_stop)
			return -EINTR;
		if (error != -ETIMEDOUT)
			return error;
	}
	if (error)
		return error;

	error = spi_sync(fpc1080->spi, &m);

	if (error) {
		dev_err(&fpc1080->spi->dev, "spi_sync failed.\n");
		return error;
	}

	*irq = buf[3];
	return 0;
}

/* -------------------------------------------------------------------- */
static int fpc1080_cmd_wait_irq(struct fpc1080_data *fpc1080, u8 cmd, u8 *irq)
{
	int error;
	struct spi_message m;
	struct spi_transfer t = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = FPC1080_SPI_CLOCK_SPEED,
		.tx_buf = &cmd,
		.rx_buf = NULL,
		.len = 1,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	error = spi_sync(fpc1080->spi, &m);

	if (error) {
		dev_err(&fpc1080->spi->dev, "spi_sync failed.\n");
		return error;
	}

	return fpc1080_spi_read_wait_irq(fpc1080, irq);
}

/* -------------------------------------------------------------------- */
static int fpc1080_spi_is_motion(struct fpc1080_data *fpc1080)
{
	int error;
	u8 tx[7];
	u8 rx[7];

	struct spi_message m;

	struct spi_transfer t = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = FPC1080_SPI_CLOCK_SPEED,
		.tx_buf = tx,
		.rx_buf = rx,
		.len = 7,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};

	tx[0] = FPC1080_SMRT_DATA;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);


	error = spi_sync(fpc1080->spi, &m);

	if (error) {
		dev_err(&fpc1080->spi->dev, "spi_sync failed.\n");
		return error;
	}
	return ((rx[2 + FPC1080_SMRT_X_BYTE] > 0) ||
		(rx[2 + FPC1080_SMRT_Y_BYTE] >= 3));
}

/* -------------------------------------------------------------------- */
static int fpc1080_wreg(struct fpc1080_data *fpc1080, u8 addr, u8 value)
{
	int error;
	u8 tx[2];

	struct spi_message m;

	struct spi_transfer t = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = FPC1080_SPI_CLOCK_SPEED,
		.tx_buf = tx,
		.rx_buf = NULL,
		.len = 2,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 0,
	};

	tx[0] = addr;
	tx[1] = value;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	error = spi_sync(fpc1080->spi, &m);

	if (error)
		dev_err(&fpc1080->spi->dev, "spi_sync failed.\n");

	dev_dbg(&fpc1080->spi->dev, "wrote %X to register %X\n", value, addr);
	return error;
}

/* -------------------------------------------------------------------- */
static int fpc1080_thread_goto_idle(struct fpc1080_data *fpc1080)
{
	fpc1080->thread_task.should_stop = 1;
	fpc1080->thread_task.mode = FPC1080_THREAD_IDLE_MODE;
	if (down_interruptible(&fpc1080->thread_task.sem_idle))
		return -ERESTARTSYS;

	up(&fpc1080->thread_task.sem_idle);

	return 0;
}

/* -------------------------------------------------------------------- */
static int fpc1080_start_thread(struct fpc1080_data *fpc1080, int mode)
{
	 fpc1080->thread_task.should_stop = 0;
	 fpc1080->thread_task.mode = mode;
	 wake_up_interruptible(&fpc1080->thread_task.wait_job);

	 return 0;
}

/* -------------------------------------------------------------------- */
static int fpc1080_write_adc_setup(struct fpc1080_data *fpc1080)
{
	int error;
	error = fpc1080_wreg(fpc1080, FPC1080_ADC_GAIN,
			     fpc1080->adc_setup.gain);
	if (error)
		return error;

	error = fpc1080_wreg(fpc1080, FPC1080_ADC_OFFSET,
			     fpc1080->adc_setup.offset);
	if (error)
		return error;

	return fpc1080_wreg(fpc1080, FPC1080_PXL_SETUP,
			    fpc1080->adc_setup.pxl_setup);
}

/* -------------------------------------------------------------------- */
#ifdef CONFIG_FPC1080_NAVIGATION
static int fpc1080_write_nav_setup(struct fpc1080_data *fpc1080)
{
	if (fpc1080_wreg(fpc1080, FPC1080_FNGR_DWN_MIN, 50))
		goto err;
	if (fpc1080_wreg(fpc1080, FPC1080_FNGR_DWN_MID, 120))
		goto err;
	if (fpc1080_wreg(fpc1080, FPC1080_FNGR_DWN_MAX, 255))
		goto err;
	if (fpc1080_wreg(fpc1080, FPC1080_FNGR_DET_THRESHOLD, 25))
		goto err;
	if (fpc1080_wreg(fpc1080, FPC1080_FNGR_LOST_THRESHOLD, 25))
		goto err;
	if (fpc1080_wreg(fpc1080, FPC1080_DX_THRESHOLD, 80))
		goto err;
	if (fpc1080_wreg(fpc1080, FPC1080_DY_THRESHOLD, 50))
		goto err;
	if (fpc1080_wreg(fpc1080, FPC1080_NAV_CNTR, 90))
		goto err;

	return 0;
err:
	return -EIO;
}
#endif

/* -------------------------------------------------------------------- */
static int fpc1080_capture_task(struct fpc1080_data *fpc1080)
{
	int error;
	int keep_image;

	u8 irq;
	u32 total_captures;

	struct timespec ts_start, ts_end, ts_delta;

	error = fpc1080_write_adc_setup(fpc1080);
	if (error)
		goto out;

	error = fpc1080_wreg(fpc1080, FPC1080_FNGR_DRIVE_CONF,
			     (FPC1080_SPI_FNGR_DRV_EXT |
			      FPC1080_SPI_SMRT_SENS_EN));
	if (error)
		goto out;

	error = fpc1080_cmd_wait_irq(fpc1080,
				     FPC1080_WAIT_FOR_FINGER_PRESENT,
				     &irq);
	if (error)
		goto out;

	getnstimeofday(&ts_start);

	error = fpc1080_cmd_wait_irq(fpc1080, FPC1080_CAPTURE_IMAGE, &irq);
	if (error)
		goto out;

	keep_image = 1;
	total_captures = 0;

	while (1) {
		if (fpc1080->current_frame >= FPC1080_MAX_FRAMES)
			break;

		if (fpc1080->thread_task.should_stop) {
			error = -EINTR;
			break;
		}
		total_captures++;
		if (keep_image) {
			error = fpc1080_cmd_wait_irq(fpc1080,
						     FPC1080_SET_SMART_REF,
						     &irq);
			if (error)
				goto out;

			error = fpc1080_spi_rd_image(fpc1080, 1);
			if (error)
				break;

			fpc1080_spi_read_wait_irq(fpc1080, &irq);

		} else {
			error = fpc1080_cmd_wait_irq(fpc1080,
						     FPC1080_CAPTURE_IMAGE,
						     &irq);
			if (error)
				break;
		}

		if (irq & FPC1080_IRQ_FING_UP)
			break;

		error = fpc1080_spi_is_motion(fpc1080);
		if (error < 0)
			break;

		keep_image = error;
	}

	getnstimeofday(&ts_end);
	ts_delta = timespec_sub(ts_end, ts_start);

	fpc1080->diag.capture_time = ts_delta.tv_nsec / NSEC_PER_MSEC;
	fpc1080->diag.capture_time += (ts_delta.tv_sec * MSEC_PER_SEC);

	fpc1080->diag.frames_stored = fpc1080->current_frame;
	fpc1080->diag.frames_captured = total_captures;

	if (fpc1080->diag.capture_time > 0) {
		dev_dbg(&fpc1080->spi->dev,
			"captured %lu frames (%lu kept) in %lu  ms (%lu fps)\n",
			(long unsigned int)fpc1080->diag.frames_captured,
			(long unsigned int)fpc1080->diag.frames_stored,
			(long unsigned int)fpc1080->diag.capture_time,
			(long unsigned int)(total_captures * MSEC_PER_SEC /
					    fpc1080->diag.capture_time));
	}

out:
	if (error) {
		fpc1080->avail_data = 0;
		dev_err(&fpc1080->spi->dev,
			"capture_task failed with error %i\n", error);
	}
	fpc1080->capture_done = 1;
	wake_up_interruptible(&fpc1080->waiting_data_avail);
	return error;
}

/* -------------------------------------------------------------------- */
static int threadfn(void *_fpc1080)
{
	struct fpc1080_data *fpc1080 = _fpc1080;

	while (!kthread_should_stop()) {

		up(&fpc1080->thread_task.sem_idle);
		wait_event_interruptible(fpc1080->thread_task.wait_job,
			fpc1080->thread_task.mode != FPC1080_THREAD_IDLE_MODE);

		down(&fpc1080->thread_task.sem_idle);

		switch (fpc1080->thread_task.mode) {
		case FPC1080_THREAD_CAPTURE_MODE:
			fpc1080_capture_task(fpc1080);
			break;

#ifdef CONFIG_FPC1080_NAVIGATION
		case FPC1080_THREAD_NAV_MODE:
			fpc1080_nav_task(fpc1080);
			break;
#endif
		default:
			break;
		}

		if(fpc1080->thread_task.mode != FPC1080_THREAD_EXIT) {
			fpc1080->thread_task.mode = FPC1080_THREAD_IDLE_MODE;
		}
	}
	return 0;
}

/* -------------------------------------------------------------------- */
static int fpc1080_abort_capture(struct fpc1080_data *fpc1080)
{
	fpc1080_thread_goto_idle(fpc1080);
	fpc1080->avail_data = 0;
	fpc1080->current_frame = 0;
	fpc1080->data_offset = 0;
	return fpc1080_reset(fpc1080);
}

/* -------------------------------------------------------------------- */
static int fpc1080_start_capture(struct fpc1080_data *fpc1080)
{
	int error = 0;
	error = fpc1080_abort_capture(fpc1080);

	if (error)
		return error;

	fpc1080_start_thread(fpc1080, FPC1080_THREAD_CAPTURE_MODE);
	fpc1080->capture_done = 0;

	return error;
}

/* -------------------------------------------------------------------- */
#ifdef CONFIG_FPC1080_NAVIGATION
static int fpc1080_stop_navigation(struct fpc1080_data *fpc1080)
{
	fpc1080_thread_goto_idle(fpc1080);
	return fpc1080_reset(fpc1080);
}
#endif

/* -------------------------------------------------------------------- */
#ifdef CONFIG_FPC1080_NAVIGATION
static int fpc1080_start_navigation(struct fpc1080_data *fpc1080)
{
	int error = 0;
	error = fpc1080_stop_navigation(fpc1080);

	if (error)
		return error;	

	fpc1080_start_thread(fpc1080, FPC1080_THREAD_NAV_MODE);
	return 0;
}
#endif

/* -------------------------------------------------------------------- */
#ifdef CONFIG_FPC1080_NAVIGATION
static int fpc1080_nav_thread_wait_fp_event(struct fpc1080_data *fpc1080)
{
	int status;

	printk(KERN_INFO "fpc1080: wait_fp_event\n");

	status = fpc1080_spi_wr_reg(fpc1080,
				    FPC1080_WAIT_FOR_FINGER_PRESENT, 0, 1);

	if (status)
		return status;

	while (1) {
		status = fpc1080_wait_for_irq(fpc1080,
					FPC1080_DEFAULT_IRQ_TIMEOUT);
		if (status == 0)
			break;
		if (fpc1080->thread_task.should_stop)
			return 0;
		if (status != -ETIMEDOUT)
			return status;
	}

	status = fpc1080_spi_wr_reg(fpc1080,
			FPC1080_RD_INTERRUPT_WITH_CLEAR, 0, 2);
	if (status)
		return status;

	input_report_key(fpc1080->nav_dev, FPC1080_KEY_FINGER_PRESENT, 1);
	input_report_key(fpc1080->nav_dev, FPC1080_KEY_FINGER_PRESENT, 0);
	input_sync(fpc1080->nav_dev);
	printk(KERN_INFO "fpc1080: event synced\n");
	return 1;
}
#endif

/* -------------------------------------------------------------------- */
#ifdef CONFIG_FPC1080_NAVIGATION
static int fpc1080_nav_task(struct fpc1080_data *fpc1080)
{
	u8 irq;
	int status;
	u8 *data;
	u8 dx;
	u8 dy;
	u8 key;

	if (fpc1080_spi_wr_reg(fpc1080, FPC1080_FNGR_DRIVE_CONF, 2, 2))
		goto err;

	if (fpc1080_write_adc_setup(fpc1080))
		goto err;

	if (fpc1080_write_nav_setup(fpc1080))
		goto err;

	status = fpc1080_nav_thread_wait_fp_event(fpc1080);
	if (status < 0)
		goto err;
	else if (status == 0)
		goto done;

	if (fpc1080_spi_wr_reg(fpc1080, FPC1080_ACTIVATE_NAVIGATION, 0, 1))
		goto err;

	while (!fpc1080->thread_task.should_stop) {
		while (1) {
			status = fpc1080_wait_for_irq(fpc1080,
						FPC1080_DEFAULT_IRQ_TIMEOUT);
			if (status == 0)
				break;
			if (fpc1080->thread_task.should_stop)
				goto done;
			if (status != -ETIMEDOUT)
				goto err;
		}
		if (fpc1080_spi_wr_reg(fpc1080,
				FPC1080_RD_INTERRUPT_WITH_NO_CLEAR, 0, 2))
			goto err;
		irq = fpc1080->huge_buffer[fpc1080->data_offset];

		if (irq & FPC1080_IRQ_FING_LOST) {
			if (fpc1080_spi_wr_reg(fpc1080,
					FPC1080_RD_INTERRUPT_WITH_CLEAR, 0, 2))
					goto err;

			status = fpc1080_nav_thread_wait_fp_event(fpc1080);
			if (status < 0)
				goto err;
			else if (status == 0)
				goto done;

			if (fpc1080_spi_wr_reg(fpc1080,
				FPC1080_ACTIVATE_NAVIGATION, 0, 1))
					goto err;
		} else {
			if (fpc1080_spi_wr_reg(fpc1080, FPC1080_SMRT_DATA,
									0, 7))
				goto err;
			data = fpc1080->huge_buffer + fpc1080->data_offset;
			dx = data[FPC1080_SMRT_X_BYTE];
			dy = data[FPC1080_SMRT_Y_BYTE];
			key = 0;

			switch (irq) {
			case FPC1080_IRQ_DX:
				key = (data[FPC1080_SMRT_BITS] &
				FPC1080_SMRT_X_SIGN ? KEY_LEFT : KEY_RIGHT);
			break;
			case FPC1080_IRQ_DY:
				key = (data[FPC1080_SMRT_BITS] &
				FPC1080_SMRT_Y_SIGN ? KEY_UP : KEY_DOWN);
			break;
			case FPC1080_IRQ_SHORT_CLICK:
			case FPC1080_IRQ_LONG_CLICK:
				if (dx + dy < FPC1080_MOTION_THRESHOLD &&
					data[FPC1080_SMRT_MO_CNTR_BYTE]  <
					FPC1080_MOTON_FRAMES_THERSHOLD) {
					key = KEY_ENTER;
				}
			default:
			break;
			}

			if (key) {
				input_report_key(fpc1080->nav_dev, key, 1);
				input_report_key(fpc1080->nav_dev, key, 0);
				input_sync(fpc1080->nav_dev);
			}

			if (fpc1080_spi_wr_reg(fpc1080,
					FPC1080_RD_INTERRUPT_WITH_CLEAR, 0, 2))
				goto err;
			if (fpc1080_spi_wr_reg(fpc1080, FPC1080_SMRT_DATA,
									0, 7))
				goto err;
		}

	}
	goto done;
err:
	dev_err(&fpc1080->spi->dev, "fpc1080nav task stopped, "
					"due to previous error.\n");

	while (!fpc1080->thread_task.should_stop)
		; /* Note: semicolon goes here to satisfy style checker */

done:
	return 0;
}
#endif

/* -------------------------------------------------------------------- */
irqreturn_t fpc1080_interrupt(int irq, void *_fpc1080)
{
	struct fpc1080_data *fpc1080 = _fpc1080;
	if (gpio_get_value(fpc1080->irq_gpio)) {
		//printk("fpc1080_interrupt\n");
		fpc1080->interrupt_done = 1;
		wake_up_interruptible(&fpc1080->waiting_interrupt_return);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

/* -------------------------------------------------------------------- */
static int fpc1080_selftest_short(struct fpc1080_data *fpc1080)
{
	int error;

	error = fpc1080_abort_capture(fpc1080);
	if (error) {
		dev_err(&fpc1080->spi->dev, "fpc1080 selftest, "
						"reset fail on entry.\n");
		goto err;
	}

	error = fpc1080_spi_wr_reg(fpc1080, FPC1080_REG_HW_ID, 0, 2);
	if (error) {
		dev_err(&fpc1080->spi->dev, "fpc1080 selftest, "
						"read HW-ID fail.\n");
		goto err;
	}

	if (fpc1080->huge_buffer[fpc1080->data_offset] != FPC1080_HARDWARE_ID) {
		dev_err(&fpc1080->spi->dev, "fpc1080 selftest, "
						"HW-ID mismatch.\n");
		error = -EIO;
		goto err;
	}

	error = fpc1080_wreg(fpc1080, FPC1080_CAPTURE_IMAGE, 0);
	if (error) {
		dev_err(&fpc1080->spi->dev, "fpc1080 selftest, "
						"capture cmd fail.\n");
		goto err;
	}

	error = fpc1080_wait_for_irq(fpc1080, FPC1080_DEFAULT_IRQ_TIMEOUT);
	if (error) {
		dev_err(&fpc1080->spi->dev, "fpc1080 selftest, "
						"irq timeout.\n");
		goto err;
	}

	error = fpc1080_abort_capture(fpc1080);
	if (error) {
		dev_err(&fpc1080->spi->dev, "fpc1080 selftest, "
						"reset fail on exit.\n");
		goto err;
	}

err:
	fpc1080->diag.selftest = (error == 0) ? 1 : 0;

#ifdef CONFIG_FPC1080_NAVIGATION
	fpc1080_start_navigation(fpc1080);
#endif

	return error;
}

/* -------------------------------------------------------------------- */
static int fpc1080_open(struct inode *inode, struct file *file)
{
	struct fpc1080_data *fpc1080;
	int error = 0;
	fpc1080 = container_of(inode->i_cdev, struct fpc1080_data, cdev);

	if (down_interruptible(&fpc1080->mutex))
		return -ERESTARTSYS;

	file->private_data = fpc1080;

	if(fpc1080->open_counter == 0) {
		error = fpc1080_abort_capture(fpc1080);
#ifdef CONFIG_FPC1080_NAVIGATION
		error = fpc1080_stop_navigation(fpc1080);
#else
		error = fpc1080_reset(fpc1080);
#endif
	}

	if(!error) fpc1080->open_counter++;

	up(&fpc1080->mutex);
	return error;
}

/* -------------------------------------------------------------------- */
static int fpc1080_release(struct inode *inode, struct file *file)
{
	int status;
	struct fpc1080_data *fpc1080 = file->private_data;
	status = 0;
	if (down_interruptible(&fpc1080->mutex))
		return -ERESTARTSYS;

	fpc1080_abort_capture(fpc1080);

	if(fpc1080->open_counter > 0) fpc1080->open_counter--;

	if (fpc1080->open_counter == 0) {
#ifdef CONFIG_FPC1080_NAVIGATION
		status = fpc1080_start_navigation(fpc1080);
#else
		status = fpc1080_spi_wr_reg(fpc1080,
				FPC1080_ACTIVATE_DEEP_SLEEP, 0, 1);
#endif
	}
	up(&fpc1080->mutex);
	return status;
}

/* -------------------------------------------------------------------- */
static ssize_t fpc1080_read(struct file *file, char *buff,
				size_t count, loff_t *ppos)
{
	int error;
	unsigned int max_dat;

	struct fpc1080_data *fpc1080 = file->private_data;
	error = 0;

	if (down_interruptible(&fpc1080->mutex))
		return -ERESTARTSYS;

	if (!fpc1080->capture_done) {
		error = wait_event_interruptible(
				fpc1080->waiting_data_avail,
				(fpc1080->capture_done || fpc1080->avail_data));
	}

	max_dat = (count > fpc1080->avail_data) ? fpc1080->avail_data : count;
	if (max_dat) {
		error = copy_to_user(buff,
			&fpc1080->huge_buffer[fpc1080->data_offset], max_dat);
		if (error)
			goto out;

		fpc1080->data_offset += max_dat;
		fpc1080->avail_data -= max_dat;
		error = max_dat;
	}
out:
	up(&fpc1080->mutex);
	return error;
}

/* -------------------------------------------------------------------- */
static ssize_t fpc1080_write(struct file *file, const char *buff,
					size_t count, loff_t *ppos)
{
	return -ENOTTY;
}

/* -------------------------------------------------------------------- */
static unsigned int fpc1080_poll(struct file *file, poll_table *wait)
{
	unsigned int ret = 0;
	struct fpc1080_data *fpc1080 = file->private_data;

	if (down_interruptible(&fpc1080->mutex))
		return -ERESTARTSYS;

	if (fpc1080->avail_data == 0 && !fpc1080->capture_done)
		poll_wait(file, &fpc1080->waiting_data_avail, wait);

	if (fpc1080->avail_data > 0)
		ret |= (POLLIN | POLLRDNORM);
	else if (fpc1080->capture_done)
		ret |= POLLHUP;

	up(&fpc1080->mutex);

	return ret;
}

/* -------------------------------------------------------------------- */
static long fpc1080_ioctl(struct file *filp, unsigned int cmd,
					unsigned long arg) {
	int error;
	struct fpc1080_data *fpc1080 = filp->private_data;
	error = 0;
	if (down_interruptible(&fpc1080->mutex))
		return -ERESTARTSYS;

	switch (cmd) {
	case FPC1080_IOCTL_START_CAPTURE:
		error = fpc1080_start_capture(fpc1080);
		break;
	case FPC1080_IOCTL_ABORT_CAPTURE:
		error = fpc1080_abort_capture(fpc1080);
		break;
	case FPC1080_IOCTL_CAPTURE_SINGLE:
		error = fpc1080_abort_capture(fpc1080);
		if (error)
			break;

		error = fpc1080_capture_single(fpc1080, arg);
		break;
	default:
		error = -ENOTTY;
		break;
	}
	up(&fpc1080->mutex);
	return error;
}

/* -------------------------------------------------------------------- */
static int fpc1080_cleanup(struct fpc1080_data *fpc1080)
{

#ifdef CONFIG_FPC1080_NAVIGATION

	if (fpc1080->nav_dev)
		input_unregister_device(fpc1080->nav_dev);

#endif
	if (fpc1080->thread_task.thread) {
		fpc1080->thread_task.should_stop = 1;
		fpc1080->thread_task.mode = FPC1080_THREAD_EXIT;
		wake_up_interruptible(&fpc1080->thread_task.wait_job);
		kthread_stop(fpc1080->thread_task.thread);
	}

	if (!IS_ERR_OR_NULL(fpc1080->device))
		device_destroy(fpc1080->class, fpc1080->devno);

	class_destroy(fpc1080->class);

	if (fpc1080->irq >= 0)
		free_irq(fpc1080->irq, fpc1080);

	if (gpio_is_valid(fpc1080->irq_gpio))
		gpio_free(fpc1080->irq_gpio);

	if (gpio_is_valid(fpc1080->reset_gpio))
		gpio_free(fpc1080->reset_gpio);

	if (fpc1080->huge_buffer) {
		free_pages((unsigned long)fpc1080->huge_buffer,
			   get_order(FPC1080_IMAGE_BUFFER_SIZE));
	}

	kfree(fpc1080);

	return 0;
}

static void fpc1080_probe_work(struct work_struct *work) {
	int error;
	struct fpc1080_data *fpc1080 = container_of(work, struct fpc1080_data, probe_work);

	init_waitqueue_head(&fpc1080->waiting_interrupt_return);
	init_waitqueue_head(&fpc1080->waiting_data_avail);
	memset(&(fpc1080->diag), 0, sizeof(fpc1080->diag));

	error = gpio_request(fpc1080->pdata->reset_gpio, "fpc1080_reset");
	if (error) {
		dev_err(&fpc1080->spi->dev,
			"fpc1080_probe - gpio_request (reset) failed.\n");
		goto err_free_pdata;
	}

	fpc1080->reset_gpio = fpc1080->pdata->reset_gpio;
	error = gpio_direction_output(fpc1080->reset_gpio, 1);
	if (error) {
		dev_err(&fpc1080->spi->dev,
		"fpc1080_probe - gpio_direction_output(reset) failed.\n");

		goto err_free_pdata;
	}

	error = gpio_request(fpc1080->pdata->irq_gpio, "fpc1080 irq");
	if (error) {
		dev_err(&fpc1080->spi->dev, "gpio_request (irq) failed.\n");
		goto err_free_pdata;
	}

	fpc1080->irq_gpio = fpc1080->pdata->irq_gpio;
	error = gpio_direction_input(fpc1080->irq_gpio);
	if (error) {
		dev_err(&fpc1080->spi->dev,
			"gpio_direction_input (irq) failed.\n");
		goto err_free_pdata;
	}

	fpc1080->irq = gpio_to_irq(fpc1080->irq_gpio);
	if (fpc1080->irq < 0) {
		dev_err(&fpc1080->spi->dev, "gpio_to_irq failed.\n");
		error = fpc1080->irq;
		goto err_free_pdata;
	}

	error = request_irq(fpc1080->irq, fpc1080_interrupt,
			IRQF_TRIGGER_RISING, "fpc1080", fpc1080);
	if (error) {
		dev_err(&fpc1080->spi->dev, "request_irq %i failed.\n",
			fpc1080->irq);

		fpc1080->irq = -EINVAL;
		goto err_free_pdata;
	}

	dev_info(&fpc1080->spi->dev, "%s: setup spi\n", __func__);
	fpc1080->spi->mode = SPI_MODE_0;
	fpc1080->spi->bits_per_word = 8;
	error = spi_setup(fpc1080->spi);
	if (error) {
		dev_err(&fpc1080->spi->dev, "spi_setup failed\n");
		goto err_free_pdata;
	}

	dev_info(&fpc1080->spi->dev, "%s: fpc1080 reset\n", __func__);
	error = fpc1080_reset(fpc1080);
	if (error)
		goto err_free_pdata;
	dev_info(&fpc1080->spi->dev, "%s: fpc1080_reset() success\n", __func__);

	error = fpc1080_spi_wr_reg(fpc1080, FPC1080_REG_HW_ID, 0, 2);
	if (error)
		goto err_free_pdata;
	dev_info(&fpc1080->spi->dev, "%s: get hardware id success\n", __func__);

	if (fpc1080->huge_buffer[fpc1080->data_offset] != FPC1080_HARDWARE_ID) {
		dev_err(&fpc1080->spi->dev,
			"hardware id mismatch: %x expected %x\n",
			fpc1080->huge_buffer[fpc1080->data_offset],
			FPC1080_HARDWARE_ID);

		error = -EIO;
		goto err_free_pdata;
	}
	dev_info(&fpc1080->spi->dev, "hardware id: %x\n",
		 fpc1080->huge_buffer[fpc1080->data_offset]);

	fpc1080->class = class_create(THIS_MODULE, FPC1080_CLASS_NAME);
	if (IS_ERR(fpc1080->class)) {
		dev_err(&fpc1080->spi->dev, "failed to create class.\n");
		error = PTR_ERR(fpc1080->class);
		goto err_free_pdata;
	}

	fpc1080->devno = MKDEV(FPC1080_MAJOR, fpc1080_device_count++);
	fpc1080->device = device_create(fpc1080->class, NULL, fpc1080->devno,
						NULL, "%s", FPC1080_DEV_NAME);

	if (IS_ERR(fpc1080->device)) {
		dev_err(&fpc1080->spi->dev, "device_create failed.\n");
		error = PTR_ERR(fpc1080->device);
		goto err_free_pdata;
	}

#ifdef CONFIG_FPC1080_NAVIGATION
	fpc1080->nav_dev = input_allocate_device();
	if (!fpc1080->nav_dev) {
		dev_err(&fpc1080->spi->dev, "input_allocate_device failed.\n");
		error  = -ENOMEM;
		goto err_free_pdata;
	}

	fpc1080->nav_dev->name = FPC1080_DEV_NAME;

	set_bit(EV_KEY, fpc1080->nav_dev->evbit);
	set_bit(KEY_ENTER, fpc1080->nav_dev->keybit);
	set_bit(KEY_UP, fpc1080->nav_dev->keybit);
	set_bit(KEY_DOWN, fpc1080->nav_dev->keybit);
	set_bit(KEY_RIGHT, fpc1080->nav_dev->keybit);
	set_bit(KEY_LEFT, fpc1080->nav_dev->keybit);
	set_bit(FPC1080_KEY_FINGER_PRESENT, fpc1080->nav_dev->keybit);
    /* bitmap_fill(fpc1080->nav_dev->keybit, KEY_MAX); */
	error = input_register_device(fpc1080->nav_dev);

	if (error) {
		dev_err(&fpc1080->spi->dev, "input_register_device failed.\n");
		input_free_device(fpc1080->nav_dev);
		fpc1080->nav_dev = NULL;
		goto err_free_pdata;
	}
#endif

	fpc1080->adc_setup.gain = fpc1080->pdata->adc_setup.gain;
	fpc1080->adc_setup.offset = fpc1080->pdata->adc_setup.offset;
	fpc1080->adc_setup.pxl_setup = fpc1080->pdata->adc_setup.pxl_setup;
	
	sema_init(&fpc1080->mutex, 0);
	error = sysfs_create_group(&fpc1080->spi->dev.kobj, &fpc1080_adc_attr_group);
	if (error) {
		dev_err(&fpc1080->spi->dev, "sysf_create_group failed.\n");
		goto err_free_pdata;
	}

	error = sysfs_create_group(&fpc1080->spi->dev.kobj, &fpc1080_diag_attr_group);
	if (error) {
		dev_err(&fpc1080->spi->dev, "sysf_create_group failed.\n");
		goto err_sysf_1;
	}

	error = register_chrdev_region(fpc1080->devno, 1, FPC1080_DEV_NAME);
	if (error) {
		dev_err(&fpc1080->spi->dev,
		  "fpc1080_probe - register_chrdev_region failed.\n");

		goto err_sysf_2;
	}

	cdev_init(&fpc1080->cdev, &fpc1080_fops);
	fpc1080->cdev.owner = THIS_MODULE;

	error = cdev_add(&fpc1080->cdev, fpc1080->devno, 1);
	if (error) {
		dev_err(&fpc1080->spi->dev, "cdev_add failed.\n");
		goto err_chrdev;
	}

	init_waitqueue_head(&fpc1080->thread_task.wait_job);
	sema_init(&fpc1080->thread_task.sem_idle, 0);
	fpc1080->thread_task.mode = FPC1080_THREAD_IDLE_MODE;
	fpc1080->thread_task.thread = kthread_run(threadfn, fpc1080, "%s",
						 FPC1080_WORKER_THREAD_NAME);

	if (IS_ERR(fpc1080->thread_task.thread)) {
		dev_err(&fpc1080->spi->dev, "kthread_run failed.\n");
		goto err_chrdev;
	}

#ifdef CONFIG_FPC1080_NAVIGATION
	error = fpc1080_start_navigation(fpc1080);
	if (error)
		goto err_cdev;
#else
	error = fpc1080_spi_wr_reg(fpc1080, FPC1080_ACTIVATE_DEEP_SLEEP, 0, 1);
	if (error)
		goto err_cdev;
#endif
	dev_info(&fpc1080->spi->dev, "%s: probe success\n", __func__);

	up(&fpc1080->mutex);
	return;

err_cdev:
	cdev_del(&fpc1080->cdev);

err_chrdev:
	unregister_chrdev_region(fpc1080->devno, 1);

err_sysf_2:
	sysfs_remove_group(&fpc1080->spi->dev.kobj, &fpc1080_diag_attr_group);

err_sysf_1:
	sysfs_remove_group(&fpc1080->spi->dev.kobj, &fpc1080_adc_attr_group);

err_free_pdata:
#ifdef CONFIG_OF
	kfree(fpc1080->pdata);
#endif
	spi_set_drvdata(fpc1080->spi, NULL);
	fpc1080_cleanup(fpc1080);
	return;
}

/* -------------------------------------------------------------------- */
static int __devinit fpc1080_probe(struct spi_device *spi) {
	int cpu, error = 0;
	struct fpc1080_data *fpc1080 = NULL;
	struct fpc1080_platform_data *fpc1080_pdata = NULL;

#ifdef CONFIG_LPM_MODE
    extern unsigned int poweroff_charging;
    extern unsigned int recovery_mode;
	if (1 == poweroff_charging || 1 == recovery_mode) {
        printk(KERN_ERR"%s: probe exit, lpm=%d recovery=%d\n", __func__, poweroff_charging, recovery_mode);
		return -ENODEV;
	}
#endif

	dev_info(&spi->dev, "%s: probe start\n", __func__);

	fpc1080 = kzalloc(sizeof(*fpc1080), GFP_KERNEL);
	if (!fpc1080) {
		dev_err(&spi->dev,
		"failed to allocate memory for struct fpc1080_data\n");

		return -ENOMEM;
	}
	fpc1080->huge_buffer =
		(u8 *)__get_free_pages(GFP_KERNEL,
				       get_order(FPC1080_IMAGE_BUFFER_SIZE));

	if (!fpc1080->huge_buffer) {
		dev_err(&fpc1080->spi->dev, "failed to get free pages\n");
		return -ENOMEM;
	}
	dev_info(&spi->dev, "%s: huge_buffer alloc success\n", __func__);
	spi_set_drvdata(spi, fpc1080);
	fpc1080->spi = spi;
	fpc1080->reset_gpio = -EINVAL;
	fpc1080->irq_gpio = -EINVAL;
	fpc1080->irq = -EINVAL;
	fpc1080->open_counter = 0;

#ifdef CONFIG_OF
	fpc1080_pdata = kzalloc(sizeof(*fpc1080_pdata), GFP_KERNEL);
	if (!fpc1080_pdata) {
		dev_err(&spi->dev, "faild to alloc pdata memory\n");
		error = -ENOMEM;	/* out of memory */
		goto err;
	}
	error = fpc1080_parse_dt(&spi->dev, fpc1080_pdata);
	if (error) {
		dev_err(&spi->dev, "faild to parse dt\n");
		goto err_free_pdata;
	}
#else
	fpc1080_pdata = spi->dev.platform_data;
#endif
	if (!fpc1080_pdata) {
		dev_err(&fpc1080->spi->dev,
				"spi->dev.platform_data is NULL.\n");
		error = -EINVAL;
		goto err_free_pdata;
	}
    fpc1080->pdata = fpc1080_pdata;

    INIT_WORK(&(fpc1080->probe_work), fpc1080_probe_work);
    cpu = cpumask_next(smp_processor_id(), cpu_online_mask);
    if (cpu == NR_CPUS) { 
        cpu = cpumask_first(cpu_online_mask);
	}

    printk(KERN_ERR"%s: nr_cpus(%d) cur cpu(%d), netx cpu(%d)\n", __func__, NR_CPUS, smp_processor_id(), cpu);
    schedule_work_on(cpu, &fpc1080->probe_work);
	return 0;

err_free_pdata:
#ifdef CONFIG_OF
	kfree(fpc1080_pdata);
#endif

err:
	fpc1080_cleanup(fpc1080);
	spi_set_drvdata(spi, NULL);
	return error;
}

/* -------------------------------------------------------------------- */
static int __devexit fpc1080_remove(struct spi_device *spi)
{
	struct fpc1080_data *fpc1080 = spi_get_drvdata(spi);
	if (NULL == fpc1080) return 0;

#ifdef CONFIG_FPC1080_NAVIGATION
	fpc1080_stop_navigation(fpc1080);
#endif
	sysfs_remove_group(&fpc1080->spi->dev.kobj, &fpc1080_adc_attr_group);
	sysfs_remove_group(&fpc1080->spi->dev.kobj, &fpc1080_diag_attr_group);

	fpc1080_spi_wr_reg(fpc1080, FPC1080_ACTIVATE_DEEP_SLEEP, 0, 1);

	cdev_del(&fpc1080->cdev);
	unregister_chrdev_region(fpc1080->devno, 1);
	fpc1080_cleanup(fpc1080);
	spi_set_drvdata(spi, NULL);
	return 0;
}

/* -------------------------------------------------------------------- */
static int fpc1080_suspend(struct device *dev)
{
#ifdef CONFIG_FPC1080_NAVIGATION
	struct fpc1080_data *fpc1080 = dev_get_drvdata(dev);
	if (NULL == fpc1080) return 0;
	fpc1080_stop_navigation(fpc1080);
	fpc1080_spi_wr_reg(fpc1080, FPC1080_ACTIVATE_DEEP_SLEEP, 0, 1);
#endif
	return 0;
}

/* -------------------------------------------------------------------- */
static int fpc1080_resume(struct device *dev)
{
#ifdef CONFIG_FPC1080_NAVIGATION
	struct fpc1080_data *fpc1080 = dev_get_drvdata(dev);
	if (NULL == fpc1080) return 0;
	fpc1080_start_navigation(fpc1080);
#endif
	return 0;
}

/* -------------------------------------------------------------------- */
static int __init fpc1080_init(void)
{
	if (spi_register_driver(&fpc1080_driver))
		return -EINVAL;

	return 0;
}

/* -------------------------------------------------------------------- */
static void __exit fpc1080_exit(void)
{
	spi_unregister_driver(&fpc1080_driver);
}

/* -------------------------------------------------------------------- */
