/* drivers/input/misc/cm32131.c - cm32131 light sensor driver
    int mProxEnabled;
 *
 * Copyright (C) 2013 Capella Microsystems Inc.
 *                                    
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/sensors/lightsensor.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <asm/mach-types.h>
#include <linux/sensors/cm32131.h>
#include <linux/fs.h>
#include <asm/setup.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

#include <linux/workqueue.h>
#include <linux/cpumask.h>
#include <linux/smp.h>

#define D(x...) pr_info(x)

#define I2C_RETRY_COUNT 2

#define NEAR_DELAY_TIME ((100 * HZ) / 1000)

#define CONTROL_INT_ISR_REPORT        0x00
#define CONTROL_ALS                   0x01

#define CALIBRATION_FILE_PATH	"/efs/cal_data"
#define CHANGE_SENSITIVITY 5 // in percent

static int record_init_fail = 0;
static void sensor_irq_do_work(struct work_struct *work);
static DECLARE_WORK(sensor_irq_work, sensor_irq_do_work);

struct cm32131_info {
	struct class *cm32131_class;
	struct device *ls_dev;

	struct input_dev *ls_input_dev;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif

	struct i2c_client *i2c_client;
	struct workqueue_struct *lp_wq;

	int intr_pin;
	int als_enable;

	uint16_t *adc_table;
	uint16_t cali_table[10];
	int irq;

	int ls_calibrate;
	int (*power)(int, uint8_t); /* power to the chip */

	uint32_t als_kadc;
	uint32_t als_gadc;
	uint16_t cal_data;
	uint16_t golden_adc;

	int lightsensor_opened;
	uint8_t ALS_cmd_address;
	uint8_t check_interrupt_add;

	int current_lux_level;
	uint16_t current_adc;

	unsigned long j_start;
	unsigned long j_end;

	uint16_t is_cmd;
	uint8_t record_clear_int_fail;
	struct work_struct probe_work;
	struct cm32131_platform_data *pdata;
};

static struct cm32131_info *lp_info;
static int enable_log = 0;
static int fLevel=-1;
static struct mutex als_enable_mutex, als_disable_mutex, als_get_adc_mutex;
static struct mutex CM32131_control_mutex;
static int lightsensor_enable(struct cm32131_info *lpi);
static int lightsensor_disable(struct cm32131_info *lpi);
static int initial_cm32131(struct cm32131_info *lpi);

static int32_t als_kadc;
static int control_and_report(struct cm32131_info *lpi, uint8_t mode, uint8_t cmd_enable);

#ifdef CONFIG_OF
static int cm32131_get_configs(struct device *dev, char *name, u32 *array)
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
		dev_info(dev, "arrary[%d]=%04X, ", i, array[i]);
	}
	dev_info(dev, "\n");
	return rc;
}

static int cm32131_parse_dt(struct device *dev, struct cm32131_platform_data *pdata)
{
	int i, ret;
	u32 value;
	u32 array[16];
	enum of_gpio_flags flags;
	struct device_node *np = dev->of_node;
	
	/* parse irq and request gpio */
	value = of_get_named_gpio_flags(np, "capella,intr", 0, &flags);
	pdata->intr = value;
	dev_info(dev, "%s: irq gpio num is %d\n", __func__, pdata->intr);
	if (pdata->intr < 0) {
		return -EINVAL;
	}
	gpio_request(pdata->intr, "gpio_cm32131_intr");
	gpio_direction_input(pdata->intr);
	
	/* general_reg */
	ret = cm32131_get_configs(dev, "capella,configs", array);
	if (ret) {
		dev_err(dev, "Looking up %s property in node %s failed", "capella,configs", np->full_name);
		return -ENODEV;
	}
	for (i = 0; i < 10; i++) {
		pdata->levels[i] = array[i];
	}
	pdata->ALS_slave_address = array[10];
	pdata->check_interrupt_add = array[11];
	pdata->is_cmd = array[12];
#if 0
	ret = of_property_read_string(np, "atmel,vcc_i2c_supply", &pdata->ts_vcc_i2c);
	ret = of_property_read_string(np, "atmel,vdd_ana_supply", &pdata->ts_vdd_ana);
	printk("pwr_en=%d, sleep_pwr_en=%d, vcc_i2c=%s, vdd_ana=%s\n", pdata->pwr_en,
			pdata->sleep_pwr_en, pdata->ts_vcc_i2c, pdata->ts_vdd_ana);
#endif
	return 0;	
}

static struct of_device_id cm32131_match_table[] = {
	{ .compatible = CM32131_I2C_NAME,},
	{ },
};
#else
#define cm32131_match_table NULL
#endif

static int I2C_RxData(uint16_t slaveAddr, uint8_t cmd, uint8_t *rxData, int length)
{
	uint8_t loop_i;
	int val;
	uint8_t subaddr[1];
	struct cm32131_info *lpi = lp_info;
	struct i2c_msg msg[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = 1,
		 .buf = subaddr,
		 },
		{
		 .addr = slaveAddr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};

	subaddr[0] = cmd;

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {

		if (i2c_transfer(lp_info->i2c_client->adapter, msg, 2) > 0)
			break;

		val = gpio_get_value(lpi->intr_pin);
		/*check intr GPIO when i2c error*/
		if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
			D("[PS][CM32131 error] %s, i2c err, slaveAddr 0x%x ISR gpio %d  = %d, record_init_fail %d \n",
				__func__, slaveAddr, lpi->intr_pin, val, record_init_fail);

		msleep(10);
	}
	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[PS_ERR][CM32131 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int I2C_RxData2(uint16_t slaveAddr, uint8_t *rxData, int length)
{
	uint8_t loop_i;
	int val;
	struct cm32131_info *lpi = lp_info;

	struct i2c_msg msg[] = {
		{
		 .addr = slaveAddr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {

		if (i2c_transfer(lp_info->i2c_client->adapter, msg, 1) > 0)
			break;

		val = gpio_get_value(lpi->intr_pin);
		/*check intr GPIO when i2c error*/
		if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
			D("[PS][CM32131 error] %s, i2c err, slaveAddr 0x%x ISR gpio %d  = %d, record_init_fail %d \n",
				__func__, slaveAddr, lpi->intr_pin, val, record_init_fail);

		msleep(10);
	}
	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[PS_ERR][CM32131 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int I2C_TxData(uint16_t slaveAddr, uint8_t *txData, int length)
{
	uint8_t loop_i;
	int val;
	struct cm32131_info *lpi = lp_info;
	struct i2c_msg msg[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(lp_info->i2c_client->adapter, msg, 1) > 0)
			break;

		val = gpio_get_value(lpi->intr_pin);
		/*check intr GPIO when i2c error*/
		if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
			D("[PS][CM32131 error] %s, i2c err, slaveAddr 0x%x, value 0x%x, ISR gpio%d  = %d, record_init_fail %d\n",
				__func__, slaveAddr, txData[0], lpi->intr_pin, val, record_init_fail);

		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		printk(KERN_ERR "[PS_ERR][CM32131 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int _cm32131_I2C_Read_Byte(uint16_t slaveAddr, uint8_t *pdata)
{
	uint8_t buffer = 0;
	int ret = 0;

	if (pdata == NULL)
		return -EFAULT;

	ret = I2C_RxData2(slaveAddr, &buffer, 1);
	if (ret < 0) {
		pr_err(
			"[PS_ERR][CM32131 error]%s: I2C_RxData fail, slave addr: 0x%x\n",
			__func__, slaveAddr);
		return ret;
	}

	*pdata = buffer;
#if 0
	/* Debug use */
	printk(KERN_DEBUG "[CM32131] %s: I2C_RxData[0x%x] = 0x%x\n",
		__func__, slaveAddr, *pdata);
#endif
	return ret;
}

static int _cm32131_I2C_Read_Word(uint16_t slaveAddr, uint8_t cmd, uint16_t *pdata)
{
	uint8_t buffer[2];
	int ret = 0;

	if (pdata == NULL)
		return -EFAULT;

	ret = I2C_RxData(slaveAddr, cmd, buffer, 2);
	if (ret < 0) {
		pr_err(
			"[PS_ERR][CM32131 error]%s: I2C_RxData fail [0x%x, 0x%x]\n",
			__func__, slaveAddr, cmd);
		return ret;
	}

	*pdata = (buffer[1]<<8)|buffer[0];
#if 0
	/* Debug use */
	printk(KERN_DEBUG "[CM32131] %s: I2C_RxData[0x%x, 0x%x] = 0x%x\n",
		__func__, slaveAddr, cmd, *pdata);
#endif
	return ret;
}

static int _cm32131_I2C_Write_Word(uint16_t SlaveAddress, uint8_t cmd, uint16_t data)
{
	char buffer[3];
	int ret = 0;
#if 0
	/* Debug use */
	printk(KERN_DEBUG
	"[CM32131] %s: _cm32131_I2C_Write_Word[0x%x, 0x%x, 0x%x]\n",
		__func__, SlaveAddress, cmd, data);
#endif
	buffer[0] = cmd;
	buffer[1] = (uint8_t)(data&0xff);
	buffer[2] = (uint8_t)((data&0xff00)>>8);	
	
	ret = I2C_TxData(SlaveAddress, buffer, 3);
	if (ret < 0) {
		pr_err("[PS_ERR][CM32131 error]%s: I2C_TxData fail\n", __func__);
		return -EIO;
	}

	return ret;
}

static int get_ls_adc_value(uint16_t *als_step, bool resume)
{
	struct cm32131_info *lpi = lp_info;
	int ret = 0;

	if (als_step == NULL)
		return -EFAULT;

	/* Read ALS data: */
	ret = _cm32131_I2C_Read_Word(lpi->ALS_cmd_address, ALS_DATA, als_step);
	if (ret < 0) {
		pr_err(
			"[LS][CM32131 error]%s: _cm32131_I2C_Read_Word fail\n",
			__func__);
		return -EIO;
	}

	return ret;
}


static int set_lsensor_range(uint16_t low_thd, uint16_t high_thd)
{
	int ret = 0;
	struct cm32131_info *lpi = lp_info;

	_cm32131_I2C_Write_Word(lpi->ALS_cmd_address, ALS_HW, high_thd);
	_cm32131_I2C_Write_Word(lpi->ALS_cmd_address, ALS_LW, low_thd);

	return ret;
}

static void sensor_irq_do_work(struct work_struct *work)
{
	struct cm32131_info *lpi = lp_info;
	control_and_report(lpi, CONTROL_INT_ISR_REPORT, 0);

	enable_irq(lpi->irq);
}

static irqreturn_t cm32131_irq_handler(int irq, void *data)
{
	struct cm32131_info *lpi = data;

	disable_irq_nosync(lpi->irq);
	if (enable_log)
		D("[PS][CM32131] %s\n", __func__);

	queue_work(lpi->lp_wq, &sensor_irq_work);

	return IRQ_HANDLED;
}

static int als_power(int enable)
{
	struct cm32131_info *lpi = lp_info;

	if (lpi->power)
		lpi->power(LS_PWR_ON, 1);

	return 0;
}

static void ls_initial_cmd(struct cm32131_info *lpi)
{	
	/*must disable l-sensor interrupt befrore IST create*//*disable ALS func*/ 		
	lpi->is_cmd |= CM32131_ALS_SD;
	_cm32131_I2C_Write_Word(lpi->ALS_cmd_address, ALS_CMD, lpi->is_cmd);
	_cm32131_I2C_Write_Word(lpi->ALS_cmd_address, 0x03, 0);
}

#if 0
static int lightsensor_get_cal_data(struct cm32131_info *lpi)
{
	struct file *cal_filp = NULL;
	int err = 0;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	D("[PS][CM32131] %s: open\n", __func__);
	cal_filp = filp_open(CALIBRATION_FILE_PATH, O_RDONLY, 0666);
	if (IS_ERR(cal_filp))
	{
		err = PTR_ERR(cal_filp);
		if (err != -ENOENT)
			pr_err("%s: Can't open calibration data file\n", __func__);
		set_fs(old_fs);
		return err;
	}

	D("[PS][CM32131] %s: read\n", __func__);
	err = cal_filp->f_op->read(cal_filp,
		(char *)&lpi->cal_data, sizeof(uint16_t), &cal_filp->f_pos);
	if (err != sizeof(uint16_t))
	{
		pr_err("%s: Can't read the calibration data from file\n", __func__);
		err = -EIO;
	}

	D("[PS][CM32131] %s: cal_data = %d\n", __func__, lpi->cal_data);

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return err;
}
#endif

static void lightsensor_set_kvalue(struct cm32131_info *lpi)
{
	if (!lpi) {
		pr_err("[LS][CM32131 error]%s: ls_info is empty\n", __func__);
		return;
	}

	D("[LS][CM32131] %s: ALS calibrated als_kadc=0x%x\n",
			__func__, als_kadc);

	if (als_kadc >> 16 == ALS_CALIBRATED)
		lpi->als_kadc = als_kadc & 0xFFFF;
	else {
		lpi->als_kadc = 0;
		D("[LS][CM32131] %s: no ALS calibrated\n", __func__);
	}

	if (lpi->als_kadc && lpi->golden_adc > 0) {
		lpi->als_kadc = (lpi->als_kadc > 0 && lpi->als_kadc < 0x1000) ?
				lpi->als_kadc : lpi->golden_adc;
		lpi->als_gadc = lpi->golden_adc;
	} else {
		lpi->als_kadc = 1;
		lpi->als_gadc = 1;
	}
	D("[LS][CM32131] %s: als_kadc=0x%x, als_gadc=0x%x\n",
		__func__, lpi->als_kadc, lpi->als_gadc);
}

static int lightsensor_update_table(struct cm32131_info *lpi)
{
	uint32_t tmpData[10];
	int i;
	for (i = 0; i < 10; i++) {
		tmpData[i] = (uint32_t)(*(lpi->adc_table + i))
				* lpi->als_kadc / lpi->als_gadc ;
		if( tmpData[i] <= 0xFFFF ){
      lpi->cali_table[i] = (uint16_t) tmpData[i];		
    } else {
      lpi->cali_table[i] = 0xFFFF;    
    }         
		D("[LS][CM32131] %s: Calibrated adc_table: data[%d], %x\n",
			__func__, i, lpi->cali_table[i]);
	}

	return 0;
}

static int lightsensor_enable(struct cm32131_info *lpi)
{
	int ret = -EIO;
	
	mutex_lock(&als_enable_mutex);
	D("[LS][CM32131] %s\n", __func__);

	if (lpi->als_enable) {
		D("[LS][CM32131] %s: already enabled\n", __func__);
		ret = 0;
	} else
  	ret = control_and_report(lpi, CONTROL_ALS, 1);
	
	mutex_unlock(&als_enable_mutex);
	return ret;
}

static int lightsensor_disable(struct cm32131_info *lpi)
{
	int ret = -EIO;
	mutex_lock(&als_disable_mutex);
	D("[LS][CM32131] %s\n", __func__);

	if ( lpi->als_enable == 0 ) {
		D("[LS][CM32131] %s: already disabled\n", __func__);
		ret = 0;
	} else
    ret = control_and_report(lpi, CONTROL_ALS, 0);
	
	mutex_unlock(&als_disable_mutex);
	return ret;
}

static int lightsensor_open(struct inode *inode, struct file *file)
{
	struct cm32131_info *lpi = lp_info;
	int rc = 0;

	D("[LS][CM32131] %s\n", __func__);
	if (lpi->lightsensor_opened) {
		pr_err("[LS][CM32131 error]%s: already opened\n", __func__);
		rc = -EBUSY;
	}
	lpi->lightsensor_opened = 1;
	return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
	struct cm32131_info *lpi = lp_info;

	D("[LS][CM32131] %s\n", __func__);
	lpi->lightsensor_opened = 0;
	return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int rc, val;
	struct cm32131_info *lpi = lp_info;

	/*D("[CM32131] %s cmd %d\n", __func__, _IOC_NR(cmd));*/

	switch (cmd) {
	case LIGHTSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg)) {
			rc = -EFAULT;
			break;
		}
		D("[LS][CM32131] %s LIGHTSENSOR_IOCTL_ENABLE, value = %d\n",
			__func__, val);
		rc = val ? lightsensor_enable(lpi) : lightsensor_disable(lpi);
		break;
	case LIGHTSENSOR_IOCTL_GET_ENABLED:
		val = lpi->als_enable;
		D("[LS][CM32131] %s LIGHTSENSOR_IOCTL_GET_ENABLED, enabled %d\n",
			__func__, val);
		rc = put_user(val, (unsigned long __user *)arg);
		break;
	default:
		pr_err("[LS][CM32131 error]%s: invalid cmd %d\n",
			__func__, _IOC_NR(cmd));
		rc = -EINVAL;
	}

	return rc;
}

static const struct file_operations lightsensor_fops = {
	.owner = THIS_MODULE,
	.open = lightsensor_open,
	.release = lightsensor_release,
	.unlocked_ioctl = lightsensor_ioctl
};

static struct miscdevice lightsensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor",
	.fops = &lightsensor_fops
};

static ssize_t ls_adc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret;
	struct cm32131_info *lpi = lp_info;

	D("[LS][CM32131] %s: ADC = 0x%04X, Lux Level = %d \n",
		__func__, lpi->current_adc, lpi->current_lux_level);
	ret = sprintf(buf, "ADC[0x%04X] => lux level %d\n",
		lpi->current_adc, lpi->current_lux_level);

	return ret;
}

static ssize_t ls_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cm32131_info *lpi = lp_info;

	ret = sprintf(buf, "Light sensor Auto Enable = %d\n",
			lpi->als_enable);

	return ret;
}

static ssize_t ls_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret = 0;
	int ls_auto;
	struct cm32131_info *lpi = lp_info;

	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1 && ls_auto != 147)
		return -EINVAL;

	if (ls_auto) {
		ret = lightsensor_enable(lpi);
	} else {
		ret = lightsensor_disable(lpi);
	}

	D("[LS][CM32131] %s: lpi->als_enable = %d, lpi->ls_calibrate = %d, ls_auto=%d\n",
		__func__, lpi->als_enable, lpi->ls_calibrate, ls_auto);

	if (ret < 0)
		pr_err(
		"[LS][CM32131 error]%s: set auto light sensor fail\n",
		__func__);

	return count;
}

static ssize_t ls_kadc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cm32131_info *lpi = lp_info;
	int ret;

	ret = sprintf(buf, "kadc = 0x%x",
			lpi->als_kadc);

	return ret;
}

static ssize_t ls_kadc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cm32131_info *lpi = lp_info;
	int kadc_temp = 0;

	sscanf(buf, "%d", &kadc_temp);

	mutex_lock(&als_get_adc_mutex);
  if(kadc_temp != 0) {
		lpi->als_kadc = kadc_temp;
		if(  lpi->als_gadc != 0){
  		if (lightsensor_update_table(lpi) < 0)
				printk(KERN_ERR "[LS][CM32131 error] %s: update ls table fail\n", __func__);
  	} else {
			printk(KERN_INFO "[LS]%s: als_gadc =0x%x wait to be set\n",
					__func__, lpi->als_gadc);
  	}		
	} else {
		printk(KERN_INFO "[LS]%s: als_kadc can't be set to zero\n",
				__func__);
	}
				
	mutex_unlock(&als_get_adc_mutex);
	return count;
}

static ssize_t ls_gadc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cm32131_info *lpi = lp_info;
	int ret;

	ret = sprintf(buf, "gadc = 0x%x\n", lpi->als_gadc);

	return ret;
}

static ssize_t ls_gadc_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cm32131_info *lpi = lp_info;
	int gadc_temp = 0;

	sscanf(buf, "%d", &gadc_temp);
	
	mutex_lock(&als_get_adc_mutex);
  if(gadc_temp != 0) {
		lpi->als_gadc = gadc_temp;
		if(  lpi->als_kadc != 0){
  		if (lightsensor_update_table(lpi) < 0)
				printk(KERN_ERR "[LS][CM32131 error] %s: update ls table fail\n", __func__);
  	} else {
			printk(KERN_INFO "[LS]%s: als_kadc =0x%x wait to be set\n",
					__func__, lpi->als_kadc);
  	}		
	} else {
		printk(KERN_INFO "[LS]%s: als_gadc can't be set to zero\n",
				__func__);
	}
	mutex_unlock(&als_get_adc_mutex);
	return count;
}

static ssize_t ls_adc_table_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	unsigned length = 0;
	int i;

	for (i = 0; i < 10; i++) {
		length += sprintf(buf + length,
			"[CM32131]Get adc_table[%d] =  0x%x ; %d, Get cali_table[%d] =  0x%x ; %d, \n",
			i, *(lp_info->adc_table + i),
			*(lp_info->adc_table + i),
			i, *(lp_info->cali_table + i),
			*(lp_info->cali_table + i));
	}
	return length;
}

static ssize_t ls_adc_table_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cm32131_info *lpi = lp_info;
	char *token[10];
	uint16_t tempdata[10];
	int i;

	printk(KERN_INFO "[LS][CM32131]%s\n", buf);
	for (i = 0; i < 10; i++) {
		token[i] = strsep((char **)&buf, " ");
		tempdata[i] = simple_strtoul(token[i], NULL, 16);
		if (tempdata[i] < 1 || tempdata[i] > 0xffff) {
			printk(KERN_ERR
			"[LS][CM32131 error] adc_table[%d] =  0x%x Err\n",
			i, tempdata[i]);
			return count;
		}
	}
	mutex_lock(&als_get_adc_mutex);
	for (i = 0; i < 10; i++) {
		lpi->adc_table[i] = tempdata[i];
		printk(KERN_INFO
		"[LS][CM32131]Set lpi->adc_table[%d] =  0x%x\n",
		i, *(lp_info->adc_table + i));
	}
	if (lightsensor_update_table(lpi) < 0)
		printk(KERN_ERR "[LS][CM32131 error] %s: update ls table fail\n",
		__func__);
	mutex_unlock(&als_get_adc_mutex);
	D("[LS][CM32131] %s\n", __func__);

	return count;
}

static uint8_t ALS_CONF = 0;
static ssize_t ls_conf_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "ALS_CONF = %x\n", ALS_CONF);
}

static ssize_t ls_conf_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cm32131_info *lpi = lp_info;
	int value = 0;
	sscanf(buf, "0x%x", &value);

	ALS_CONF = value;
	printk(KERN_INFO "[LS]set ALS_CONF = %x\n", ALS_CONF);
	_cm32131_I2C_Write_Word(lpi->ALS_cmd_address, ALS_CMD, ALS_CONF);
	return count;
}

static ssize_t ls_fLevel_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "fLevel = %d\n", fLevel);
}

static ssize_t ls_fLevel_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct cm32131_info *lpi = lp_info;
	int value=0;
	sscanf(buf, "%d", &value);
	(value>=0)?(value=min(value,10)):(value=max(value,-1));
	fLevel=value;
	input_report_abs(lpi->ls_input_dev, ABS_MISC, fLevel);
	input_sync(lpi->ls_input_dev);
	printk(KERN_INFO "[LS]set fLevel = %d\n", fLevel);

	msleep(1000);
	fLevel=-1;
	return count;
}

static ssize_t ls_cal_data_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cm32131_info *lpi = lp_info;
	int ret;

	ret = sprintf(buf, "%d\n", lpi->cal_data);

	return ret;
}

static ssize_t ls_cal_data_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	uint16_t new_cal_data = 0;
	struct cm32131_info *lpi = lp_info;	
	struct file *cal_filp = NULL;
	mm_segment_t old_fs;
	int err = 0;

	sscanf(buf, "%d", (int *)&new_cal_data);
	if (new_cal_data != 0)
	{
		lpi->cal_data = new_cal_data;
	}
	else  // reset calibration data
	{
		lpi->cal_data = 1000;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH,
			O_CREAT | O_TRUNC | O_WRONLY, 0666);
	if (IS_ERR(cal_filp))
	{
		pr_err("%s: Can't open calibration file\n", __func__);
		set_fs(old_fs);
		err = PTR_ERR(cal_filp);
		return err;
	}

	err = cal_filp->f_op->write(cal_filp,
		(char *)&lpi->cal_data, sizeof(uint16_t), &cal_filp->f_pos);
	if (err != sizeof(uint16_t))
	{
		pr_err("%s: Can't write the calibration data to file\n", __func__);
		err = -EIO;
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return count;
}

static struct device_attribute dev_attr_light_enable =
//__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP, ls_enable_show, ls_enable_store);
__ATTR(enable, S_IRWXUGO, ls_enable_show, ls_enable_store);

static struct device_attribute dev_attr_light_kadc =
//__ATTR(kadc, S_IRUGO | S_IWUSR | S_IWGRP, ls_kadc_show, ls_kadc_store);
__ATTR(kadc, S_IRWXUGO, ls_kadc_show, ls_kadc_store);

static struct device_attribute dev_attr_light_gadc =
//__ATTR(gadc, S_IRUGO | S_IWUSR | S_IWGRP, ls_gadc_show, ls_gadc_store);
__ATTR(gadc, S_IRWXUGO, ls_gadc_show, ls_gadc_store);

static struct device_attribute dev_attr_light_adc_table =
//__ATTR(adc_table, S_IRUGO | S_IWUSR | S_IWGRP, ls_adc_table_show, ls_adc_table_store);
__ATTR(adc_table, S_IRWXUGO, ls_adc_table_show, ls_adc_table_store);

static struct device_attribute dev_attr_light_conf =
//__ATTR(conf, S_IRUGO | S_IWUSR | S_IWGRP, ls_conf_show, ls_conf_store);
__ATTR(conf, S_IRWXUGO, ls_conf_show, ls_conf_store);

static struct device_attribute dev_attr_light_fLevel =
//__ATTR(fLevel, S_IRUGO | S_IWUSR | S_IWGRP, ls_fLevel_show, ls_fLevel_store);
__ATTR(fLevel, S_IRWXUGO, ls_fLevel_show, ls_fLevel_store);

static struct device_attribute dev_attr_light_adc =
//__ATTR(adc, S_IRUGO | S_IWUSR | S_IWGRP, ls_adc_show, NULL);
__ATTR(adc, S_IRWXUGO, ls_adc_show, NULL);

static struct device_attribute dev_attr_light_cal_data =
//__ATTR(cali, S_IRUGO | S_IWUSR | S_IWGRP, ls_cal_data_show, ls_cal_data_store);
__ATTR(cali, S_IRWXUGO, ls_cal_data_show, ls_cal_data_store);

static struct attribute *light_sysfs_attrs[] = {
	&dev_attr_light_enable.attr,
	&dev_attr_light_kadc.attr,
	&dev_attr_light_gadc.attr,
	&dev_attr_light_adc_table.attr,
	&dev_attr_light_conf.attr,
	&dev_attr_light_fLevel.attr,
	&dev_attr_light_adc.attr,
	&dev_attr_light_cal_data.attr,
	NULL
};

static struct attribute_group light_attribute_group = {
	.attrs = light_sysfs_attrs,
};

static int lightsensor_setup(struct cm32131_info *lpi)
{
	int ret;

	lpi->ls_input_dev = input_allocate_device();
	if (!lpi->ls_input_dev) {
		pr_err(
			"[LS][CM32131 error]%s: could not allocate ls input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->ls_input_dev->name = "cm32131-ls";
	set_bit(EV_ABS, lpi->ls_input_dev->evbit);
	input_set_abs_params(lpi->ls_input_dev, ABS_MISC, 0, 9, 0, 0);

	ret = input_register_device(lpi->ls_input_dev);
	if (ret < 0) {
		pr_err("[LS][CM32131 error]%s: can not register ls input device\n",
				__func__);
		goto err_free_ls_input_device;
	}

	ret = misc_register(&lightsensor_misc);
	if (ret < 0) {
		pr_err("[LS][CM32131 error]%s: can not register ls misc device\n",
				__func__);
		goto err_unregister_ls_input_device;
	}

	return ret;

err_unregister_ls_input_device:
	input_unregister_device(lpi->ls_input_dev);
err_free_ls_input_device:
	input_free_device(lpi->ls_input_dev);
	return ret;
}

static int initial_cm32131(struct cm32131_info *lpi)
{
	int val, ret, fail_counter = 0;
	uint8_t add = 0;

	val = gpio_get_value(lpi->intr_pin);
	D("[LS][CM32131] %s, INTERRUPT GPIO val = %d\n", __func__, val);

	check_interrupt_gpio:
	if (fail_counter >= 1) {
		D("[LS][CM32131] %s, initial fail_counter = %d\n", __func__, fail_counter);
		if (record_init_fail == 0)
			record_init_fail = 1;
		return -ENOMEM;/*If devices without cm32131 chip and did not probe driver*/
	}
	lpi->is_cmd = lpi->is_cmd | CM32131_ALS_SD; 
	ret = _cm32131_I2C_Write_Word(lpi->ALS_cmd_address, ALS_CMD, lpi->is_cmd);
	if ((ret < 0) && (fail_counter < 1)) {	
		fail_counter++;
		val = gpio_get_value(lpi->intr_pin);
		if( val == 0 ){
			D("[LS][CM32131] %s, interrupt GPIO val = %d, , inital fail_counter %d\n",
					__func__, val, fail_counter);
			ret =_cm32131_I2C_Read_Byte(lpi->check_interrupt_add, &add);
			D("[LS][CM32131] %s, check_interrupt_add value = 0x%x, ret %d\n",
					__func__, add, ret);
		}
		val = gpio_get_value(lpi->intr_pin);
		if( val == 0 ){
			D("[LS][CM32131] %s, interrupt GPIO val = %d, , inital fail_counter %d\n",
					__func__, val, fail_counter);
			ret =_cm32131_I2C_Read_Byte(lpi->check_interrupt_add, &add);
			D("[LS][CM32131] %s, check_interrupt_add value = 0x%x, ret %d\n",
					__func__, add, ret);
		}
		goto check_interrupt_gpio;
	}

	return 0;
}

static int cm32131_setup(struct cm32131_info *lpi)
{
	int ret = 0;

	als_power(1);
	msleep(5);

	ret = initial_cm32131(lpi);
	if (ret < 0) {
		pr_err(
			"[LS_ERR][CM32131 error]%s: fail to initial cm32131 (%d)\n",
			__func__, ret);
		return ret;
	}

	/*Default disable L sensor*/
	ls_initial_cmd(lpi);
	ret = request_any_context_irq(lpi->irq,
			cm32131_irq_handler,
			IRQF_TRIGGER_LOW,
			"cm32131",
			lpi);
	if (ret < 0) {
		pr_err(
			"[LS][CM32131 error]%s: req_irq(%d) fail for gpio %d (%d)\n",
			__func__, lpi->irq, lpi->intr_pin, ret);
	}

	return ret;
}

static int cm32131_suspend(struct device *dev) {
    struct cm32131_info *lpi = lp_info;
	D("[LS][CM32131] %s\n", __func__);

	if (NULL == lpi) {
		pr_err("[LS][CM32131] %s: lpi is NULL\n", __func__);
		return 0;
	}

	if (lpi->als_enable)
		lightsensor_disable(lpi);

	return 0;
}

static int cm32131_resume(struct device *dev) {
    struct cm32131_info *lpi = lp_info;
	D("[LS][CM32131] %s\n", __func__);

	if (NULL == lpi) {
		pr_err("[LS][CM32131] %s: lpi is NULL\n", __func__);
		return 0;
	}

	if (!lpi->als_enable)
		lightsensor_enable(lpi);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cm32131_early_suspend(struct early_suspend *h)
{
	struct cm32131_info *lpi = lp_info;
	D("[LS][CM32131] %s\n", __func__);

	if (NULL == lpi) {
		pr_err("[LS][CM32131] %s: lpi is NULL\n", __func__);
		return;
	}

	cm32131_suspend(&lpi->i2c_client->dev);
}

static void cm32131_late_resume(struct early_suspend *h)
{
	struct cm32131_info *lpi = lp_info;
	D("[LS][CM32131] %s\n", __func__);

	if (NULL == lpi) {
		pr_err("[LS][CM32131] %s: lpi is NULL\n", __func__);
		return;
	}

	cm32131_resume(&lpi->i2c_client->dev);
}
#endif

static void cm32131_probe_work(struct work_struct *work) {
	int ret = 0;
	struct cm32131_info *lpi = container_of(work, struct cm32131_info, probe_work);
	
	mutex_init(&CM32131_control_mutex);
	mutex_init(&als_enable_mutex);
	mutex_init(&als_disable_mutex);
	mutex_init(&als_get_adc_mutex);

	ret = lightsensor_setup(lpi);
	if (ret < 0) {
		pr_err("[LS][CM32131 error]%s: lightsensor_setup error!!\n",
			__func__);
		goto err_lightsensor_setup;
	}

	lpi->cal_data = 1000;
#if 0
	/* open calibration data */
	ret = lightsensor_get_cal_data(lpi);
	if (ret < 0 && ret != -ENOENT) {
		pr_err("%s: lightsensor_get_cal_data() failed\n",
			__func__);
	}
#endif

	D("[PS][CM32131] %s: cal_data=%d\n", __func__, lpi->cal_data);
	//SET LUX STEP FACTOR HERE
	// if adc raw value 1000 eqauls to 286 lux
	// the following will set the factor 286/1000
	// and lpi->golden_adc = 286;  
	// set als_kadc = (ALS_CALIBRATED <<16) | 1000;
	als_kadc = (ALS_CALIBRATED <<16) | 1000;
	lpi->golden_adc = 286;

	//ls calibrate always set to 1 
	lpi->ls_calibrate = 1;

	lightsensor_set_kvalue(lpi);
	ret = lightsensor_update_table(lpi);
	if (ret < 0) {
		pr_err("[LS][CM32131 error]%s: update ls table fail\n",
			__func__);
		goto err_lightsensor_update_table;
	}

	lpi->lp_wq = create_singlethread_workqueue("cm32131_wq");
	if (!lpi->lp_wq) {
		pr_err("[PS][CM32131 error]%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}

	ret = cm32131_setup(lpi);
	if (ret < 0) {
		pr_err("[PS_ERR][CM32131 error]%s: cm32131_setup error!\n", __func__);
		goto err_cm32131_setup;
	}

	lpi->cm32131_class = class_create(THIS_MODULE, "optical_sensors");
	if (IS_ERR(lpi->cm32131_class)) {
		ret = PTR_ERR(lpi->cm32131_class);
		lpi->cm32131_class = NULL;
		goto err_create_class;
	}

	lpi->ls_dev = device_create(lpi->cm32131_class,
				NULL, 0, "%s", "lightsensor");
	if (unlikely(IS_ERR(lpi->ls_dev))) {
		ret = PTR_ERR(lpi->ls_dev);
		lpi->ls_dev = NULL;
		goto err_create_ls_device;
	}

	/* register the attributes */
	ret = sysfs_create_group(&lpi->ls_input_dev->dev.kobj,
			&light_attribute_group);
	if (ret) {
		pr_err("[LS][CM3232 error]%s: could not create sysfs group\n", __func__);
		goto err_sysfs_create_group_light;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	lpi->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	lpi->early_suspend.suspend = cm32131_early_suspend;
	lpi->early_suspend.resume = cm32131_late_resume;
	register_early_suspend(&lpi->early_suspend);
#endif

	D("[PS][CM32131] %s: Probe success!\n", __func__);
	lpi->als_enable=0;
	return;

err_sysfs_create_group_light:
	device_unregister(lpi->ls_dev);
err_create_ls_device:
	class_destroy(lpi->cm32131_class);
err_create_class:
err_cm32131_setup:
	destroy_workqueue(lpi->lp_wq);
	input_unregister_device(lpi->ls_input_dev);
	input_free_device(lpi->ls_input_dev);
err_create_singlethread_workqueue:
err_lightsensor_update_table:
	mutex_destroy(&CM32131_control_mutex);
	misc_deregister(&lightsensor_misc);
err_lightsensor_setup:
	mutex_destroy(&als_enable_mutex);
	mutex_destroy(&als_disable_mutex);
	mutex_destroy(&als_get_adc_mutex);
#ifdef CONFIG_OF
	kfree(lpi->pdata);
#endif
	i2c_set_clientdata(lpi->i2c_client, NULL);
	kfree(lpi);
	lpi = NULL;
	lp_info = NULL;
	return;
}

static int cm32131_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	int cpu, ret = 0;
	struct cm32131_info *lpi;
	struct cm32131_platform_data *pdata;

#ifdef CONFIG_LPM_MODE
    extern unsigned int poweroff_charging;
    extern unsigned int recovery_mode;
	if (1 == poweroff_charging || 1 == recovery_mode) {
        printk(KERN_ERR"%s: probe exit, lpm=%d recovery=%d\n", __func__, poweroff_charging, recovery_mode);
		return -ENODEV;
	}
#endif

	D("[LS][CM32131] %s\n", __func__);
	lpi = kzalloc(sizeof(struct cm32131_info), GFP_KERNEL);
	if (!lpi) {
		return -ENOMEM;
	}

	/*D("[CM32131] %s: client->irq = %d\n", __func__, client->irq);*/
#ifdef CONFIG_OF
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err("[LS][CM32131 error]%s: faild to alloc pdata memory\n", __func__);
		ret = -ENOMEM;	/* out of memory */
		goto err_platform_data_null;
	}

	ret = cm32131_parse_dt(&client->dev, pdata);
	if (ret) {
		pr_err("[LS][CM32131 error]%s: faild to parse dt\n", __func__);
		goto err_parse_dt;
	}
#else
	pdata = client->dev.platform_data;
	if (!pdata) {
		pr_err("[LS][CM32131 error]%s: Assign platform_data error!!\n",
			__func__);
		ret = -EBUSY;
		goto err_parse_dt;
	}
#endif
	lpi->i2c_client = client;
	lpi->irq = client->irq;
	i2c_set_clientdata(client, lpi);
	/*D("[CM32131] %s: client->irq = %d\n", __func__, client->irq);*/

	lpi->intr_pin = pdata->intr;
	lpi->adc_table = pdata->levels;
	lpi->power = pdata->power;
	lpi->ALS_cmd_address = pdata->ALS_slave_address;
	lpi->check_interrupt_add = pdata->check_interrupt_add;
	lpi->is_cmd  = pdata->is_cmd;
	lpi->j_start = 0;
	lpi->j_end = 0;
	lpi->record_clear_int_fail=0;
	if (pdata->is_cmd == 0) {
		lpi->is_cmd  = CM32131_ALS_SM_2 | CM32131_ALS_IT_250MS | CM32131_ALS_PERS_1 | CM32131_ALS_DEFAULT;
	}

	lpi->pdata = pdata;
	lp_info = lpi;

	INIT_WORK(&(lpi->probe_work), cm32131_probe_work);
    cpu = cpumask_next(smp_processor_id(), cpu_online_mask);
    if (cpu == NR_CPUS) { 
        cpu = cpumask_first(cpu_online_mask);
	}

    printk(KERN_ERR"%s: nr_cpus(%d) cur cpu(%d), netx cpu(%d)\n", __func__, NR_CPUS, smp_processor_id(), cpu);
    schedule_work_on(cpu, &lpi->probe_work);
	return 0;

err_parse_dt:
#ifdef CONFIG_OF
	kfree(pdata);
#endif
err_platform_data_null:
	kfree(lpi);
	return ret;
}
   
static int control_and_report( struct cm32131_info *lpi, uint8_t mode, uint8_t cmd_enable ) {
	int ret=0;
	uint16_t adc_value = 0;	
	int val;
	int fail_counter = 0;
	uint8_t add = 0;
	uint16_t low_thd;
	uint32_t high_thd;
	uint32_t lux_level;
	uint32_t tmp;

	mutex_lock(&CM32131_control_mutex);

	while(1){
		val = gpio_get_value(lpi->intr_pin);  
		D("[CM32131] %s, interrupt GPIO val = %d, fail_counter %d\n",
			__func__, val, fail_counter);
		
		val = gpio_get_value(lpi->intr_pin);
		if( val == 0){
			ret = _cm32131_I2C_Read_Byte(lpi->check_interrupt_add, &add);
			D("[CM32131] %s, interrupt GPIO val = %d, check_interrupt_add value = 0x%x, ret %d\n",
			  __func__, val, add, ret);
		}
		val = gpio_get_value(lpi->intr_pin);
		if( val == 0){
			ret = _cm32131_I2C_Read_Byte(lpi->check_interrupt_add, &add);
			D("[CM32131] %s, interrupt GPIO val = %d, check_interrupt_add value = 0x%x, ret %d\n",
			  __func__, val, add, ret);
		}

		lpi->is_cmd &= ~CM32131_ALS_INT_EN;
		ret = _cm32131_I2C_Write_Word(lpi->ALS_cmd_address, ALS_CMD, lpi->is_cmd);
		if( ret == 0 ){
			break;
		}
		else {	
			fail_counter++;
			val = gpio_get_value(lpi->intr_pin);
			D("[CM32131] %s, interrupt GPIO val = %d, , inital fail_counter %d\n",
				__func__, val, fail_counter);
		}
		if (fail_counter >= 10) {
			D("[CM32131] %s, clear INT fail_counter = %d\n", __func__, fail_counter);
			if (lpi->record_clear_int_fail == 0) {
				lpi->record_clear_int_fail = 1;
			}
			ret=-EIO;
			goto error_clear_interrupt;
		}
	}

	if( mode == CONTROL_ALS ){
		if(cmd_enable){
			lpi->is_cmd &= ~CM32131_ALS_SD;
		}
		else {
			lpi->is_cmd |= CM32131_ALS_SD;
		}
		_cm32131_I2C_Write_Word(lpi->ALS_cmd_address, ALS_CMD, lpi->is_cmd);
		lpi->als_enable=cmd_enable;
	}

	if((mode == CONTROL_ALS) && (cmd_enable==1)){
		input_report_abs(lpi->ls_input_dev, ABS_MISC, -1);
		input_sync(lpi->ls_input_dev);
		msleep(100);
	}

	if (lpi->als_enable){
		get_ls_adc_value(&adc_value, 0);
		tmp = (adc_value*lpi->als_gadc)/lpi->als_kadc;
		lux_level = tmp*lpi->cal_data/1000;
		D("[LS][CM32131] %s: raw_adc=0x%04X, als_gadc=0x%04X, \
				cal_data=0x%04X, als_kadc=0x%04X, ls_calibrate = %d\n",
				__func__, adc_value, lpi->als_gadc, lpi->cal_data, lpi->als_kadc, lpi->ls_calibrate);

		lpi->is_cmd |= CM32131_ALS_INT_EN;

		// set interrupt high/low threshold
		low_thd = adc_value - adc_value * CHANGE_SENSITIVITY /100;
		high_thd = adc_value + adc_value * CHANGE_SENSITIVITY /100;
		if (high_thd > 65535) {
			high_thd = 65535;
		}
		ret = set_lsensor_range(low_thd, (uint16_t)high_thd);	  

		D("[CM32131] %s: ADC=0x%04X, Lux Level=%d, l_thd = 0x%x, h_thd = 0x%x \n",
			__func__, adc_value, lux_level, low_thd, high_thd);
		lpi->current_lux_level = lux_level;
		lpi->current_adc = adc_value;    
		input_report_abs(lpi->ls_input_dev, ABS_MISC, lux_level);
		input_sync(lpi->ls_input_dev);
	}

	ret = _cm32131_I2C_Write_Word(lpi->ALS_cmd_address, ALS_CMD, lpi->is_cmd);
	if( ret == 0 ){
		D("[CM32131] %s, re-enable INT OK\n", __func__);
	}
	else{
		D("[CM32131] %s, re-enable INT FAIL\n", __func__);
	}

error_clear_interrupt:
	mutex_unlock(&CM32131_control_mutex);
	return ret;
}

static const struct i2c_device_id cm32131_i2c_id[] = {
	{CM32131_I2C_NAME, 0},
	{}
};

#if (defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND))
static SIMPLE_DEV_PM_OPS(cm32131_pm_ops, cm32131_suspend, cm32131_resume);
#endif

static struct i2c_driver cm32131_driver = {
	.id_table = cm32131_i2c_id,
	.probe = cm32131_probe,
	.driver = {
		.name = CM32131_I2C_NAME,
		.owner = THIS_MODULE,
	#if (defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND))
		.pm	= &cm32131_pm_ops,
	#endif
	#ifdef CONFIG_OF
		.of_match_table = cm32131_match_table,
	#endif
	},
};

static int __init cm32131_init(void)
{
	return i2c_add_driver(&cm32131_driver);
}

static void __exit cm32131_exit(void)
{
	i2c_del_driver(&cm32131_driver);
}

module_init(cm32131_init);
module_exit(cm32131_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CM32131 Light Sensor Driver");
MODULE_AUTHOR("Capella Microsystems");
