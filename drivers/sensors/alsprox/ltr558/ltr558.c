
/***********************************************************************
 **
 **  Copyright (C), 2013-2015, Yulong Tech. Co., Ltd.
 **  FileName:		ltr558.c
 **  Description:	Linux device driver for ltr558 ambient light and proximity sensors 
 **  Author:		longjiang
 **  Version:		1.00
 **  Date:             	2013-09-25                                              
 **
 ***********************************************************************/


#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/poll.h>
#include <linux/wakelock.h>
#include <asm/gpio.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <linux/slab.h>
#include <linux/completion.h>

//longjiang add start 20130925
#include <linux/sensors/ltr558.h>
#include <linux/sensors/sensparams.h>
#include <linux/regulator/consumer.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif
//add end

#define DRIVER_VERSION "1.0"
#define DEVICE_I2C_ID "LTR_558ALS"   //for i2c_device_id addr=0x46 

#define PROX_THRESHOLD_HI  155//0x120  //300
#define PROX_THRESHOLD_LO  110//0xc8     //200

#define ALS_INTEGRATION_TIME 100

#define I2C_RETRY_DELAY		5
#define I2C_RETRIES		2 
#define LTR558_PS_MODE_MASK 0x02
#define LTR558_ALS_MODE_MASK 0x02
#define LTR558_POWER_ON_DELAY 600

//longjiang add start 20130520
#define SENSOR_SW_RESET 0x04
static int prox_enabled = 0;
static int last_prox_data = 0 ;
//add end

static struct work_struct irq_workqueue;

static int prox_active = 0;
static int light_active = 0;

static atomic_t calibrate = ATOMIC_INIT(0);

static int ps_gainrange;
static int als_gainrange;
static int itg_tm;
static int als_integration_time;

struct ltr558_data {
	struct i2c_client *client;
	struct mutex lock;
	//longjiang add start 20130520
	struct wake_lock   wake_lock;
	unsigned int als_ps_int;
	//add end
	struct input_dev *input_dev_light;
	struct input_dev *input_dev_proximity;
   	int prox_threshold_hi;
	int prox_threshold_lo;
	u8  reg_addr;
#ifdef CONFIG_SENSOR_POWER
	struct regulator *vcc_i2c;
	struct regulator *vdd_ana;
#endif
        int als_coff1;
        int als_coff2;
};

static struct ltr558_data *the_data = NULL;

struct prox_calibrate{
        int prox_data;
};
static struct prox_calibrate prox_cal_info[20];

#ifdef CONFIG_YL_DEBUG
	#define YL_DEBUG(fmt, args...) pr_info(fmt, ##args)
#else
	#define YL_DEBUG(fmt, args...)
#endif

static int ltr558_als_read(int gainrange);
static int ltr558_dev_init(void);

// I2C Read
static int ltr558_i2c_read_reg(u8 regnum)
{
	int val = -1;
	int tries = 0;

	/*
	 * i2c_smbus_read_byte_data - SMBus "read byte" protocol
	 * @client: Handle to slave device
	 * @command: Byte interpreted by slave
	 *
	 * This executes the SMBus "read byte" protocol, returning negative errno
	 * else a data byte received from the device.
	 */

	do{
		val = i2c_smbus_read_byte_data(the_data->client, regnum);	
		if(val < 0){		
			printk(KERN_ERR"[ltr558]:error:i2c_read failed in %s \n", __func__);
			msleep_interruptible(I2C_RETRY_DELAY);//continue;
		}
	}while((val < 0) && (++tries < I2C_RETRIES));

	return val;
}

static int ltr558_dbg_i2c_read(struct ltr558_data *ltr558_info, u8 addr, u8 *data, int len)
{
	int err;

	struct i2c_msg msgs[] = {
		{
		 .addr = ltr558_info->client->addr,
		 .flags = ltr558_info->client->flags & I2C_M_TEN,
		 .len = 1,
		 .buf = &addr,
		 },
		{
		 .addr = ltr558_info->client->addr,
		 .flags = (ltr558_info->client->flags & I2C_M_TEN) | I2C_M_RD,
		 .len = len,
		 .buf = data,
		 },
	};
	err = i2c_transfer(ltr558_info->client->adapter, msgs, 2);

	if (err != 2)
		dev_err(&ltr558_info->client->dev, "read transfer error\n");
	else
		err = 0;

	return err;
}


// I2C Write
static int ltr558_i2c_write_reg(u8 regnum, u8 value)
{
    int32_t ret = -1;
	uint8_t tries = 0;

	/*
	 * i2c_smbus_write_byte_data - SMBus "write byte" protocol
	 * @client: Handle to slave device
	 * @command: Byte interpreted by slave
	 * @value: Byte being written
	 *
	 * This executes the SMBus "write byte" protocol, returning negative errno
	 * else zero on success.
	 */

	do{
    	ret = i2c_smbus_write_byte_data(the_data->client, regnum, value);	
		//printk(KERN_ERR"[STKPS]:tries =%d in %s \n",tries, __func__);//test
		if(ret){		
			printk(KERN_ERR"[ltr558]:error:i2c_write failed in %s \n", __func__);
			msleep_interruptible(I2C_RETRY_DELAY);//continue;
		}
	}while((ret != 0) && (++tries < I2C_RETRIES));
	
    return ret;

}

//longjiang add start 20130913
#ifdef CONFIG_OF
static int ltr558_parse_dt(struct device *dev, struct ltr558_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
        u32 temp_val1,temp_val2;
        int rc;

	pdata->gpio_int = of_get_named_gpio_flags(np, "ltr558,gpio_int",
		0, &pdata->irq_gpio_flags);

        pdata->als_coff1 = 1;
        pdata->als_coff2 = 1;
        
        rc = of_property_read_u32(np, "ltr558,als_coff1", &temp_val1);               
        if (rc != -EINVAL)
        {
              rc = of_property_read_u32(np, "ltr558,als_coff2", &temp_val2);
              if (rc != -EINVAL)
              {
                     pdata->als_coff1 = temp_val1;
                     pdata->als_coff2 = temp_val2;
              }
        }
       
               
	//YL_DEBUG("%s: %d, %d coff(1,2)=%d %d\n", __func__, pdata->gpio_int, pdata->irq_gpio_flags,pdata->als_coff1,pdata->als_coff2);
        printk("%s: %d, %d coff(1,2)=%d %d\n", __func__, pdata->gpio_int, pdata->irq_gpio_flags,pdata->als_coff1,pdata->als_coff2);      
 
	return 0;
}
#endif
//add end

static int ltr558_prox_poll(struct prox_calibrate *prox_cal_info)
{
	int psval_lo, psval_hi, psdata;

	psval_lo = ltr558_i2c_read_reg(LTR558_PS_DATA_0);
	if (psval_lo < 0){
		psdata = psval_lo;
		return -ENODATA;
	}
		
	psval_hi = ltr558_i2c_read_reg(LTR558_PS_DATA_1);
	if (psval_hi < 0){
		psdata = psval_hi;
		return -ENODATA;
	}
		
	prox_cal_info->prox_data = ((psval_hi & 7)* 256) + psval_lo;
    
	return 0;
}
static int ltr558_prox_calibrate(struct prox_offset *taos_cal_ptr, int count)
{
	int prox_sum = 0;
	int prox_max = 0;
	int prox_mean;
	int i;
	int ret = 0;
	unsigned char hi_string[5] = {0};

	for (i = 0; i < count; i++) {
	    if ((ret = ltr558_prox_poll(&prox_cal_info[i])) < 0) {
	        printk(KERN_ERR "[ltr558]: call to prox_poll failed in ioctl prox_calibrate\n");
	        return (ret);
	    }
	    prox_sum += prox_cal_info[i].prox_data;
	    if (prox_cal_info[i].prox_data > prox_max)
	        prox_max = prox_cal_info[i].prox_data;
	    mdelay(100);
	}

    	prox_mean = prox_sum/count;
    	YL_DEBUG("[ltr558]: func:%s,prox_mean = %d, count = %d , pc trace....\n",__func__,prox_mean,count);
    	if(prox_mean < 100){
		the_data->prox_threshold_hi = (u16)(20 * prox_mean/10);
	}
    	else if(prox_mean < 300){
  		the_data->prox_threshold_hi = (u16)(20 * prox_mean/10);   
	}
	else if(prox_mean < 600){
		the_data->prox_threshold_hi = (u16)(18 * prox_mean/10);
	}
    	else{
        		printk(KERN_ERR "Clibrate failed!\n");
	}

	the_data->prox_threshold_lo = 8 * the_data->prox_threshold_hi/10;
	if(the_data->prox_threshold_hi < PROX_THRESHOLD_HI){
	    the_data->prox_threshold_hi = PROX_THRESHOLD_HI;
	    the_data->prox_threshold_lo = PROX_THRESHOLD_LO;
	}
	if(the_data->prox_threshold_hi > 900){
		the_data->prox_threshold_hi = 900;
		the_data->prox_threshold_lo = 750;
	}

	//longjiang add start 20130520
	hi_string[0] = the_data->prox_threshold_hi & 0x00ff;
	hi_string[1] = (the_data->prox_threshold_hi & 0xff00) >> 8;
	hi_string[2] = the_data->prox_threshold_lo & 0x00ff;
	hi_string[3] = (the_data->prox_threshold_lo & 0xff00) >> 8;
	hi_string[4] = prox_mean < 600 ? 1 : 2;

	sensparams_write_to_flash(SENSPARAMS_TYPE_PROX, hi_string, 5);

	((struct prox_offset *)taos_cal_ptr)->x = (unsigned short)(the_data->prox_threshold_hi);
	((struct prox_offset *)taos_cal_ptr)->y = (unsigned short)(the_data->prox_threshold_lo);
	((struct prox_offset *)taos_cal_ptr)->z = (unsigned short)(prox_mean);
	((struct prox_offset *)taos_cal_ptr)->key = prox_mean < 600 ? 1 : 2;
	//add end

	YL_DEBUG("[ltr558]:>>>>>>>%s :-- prox_threshold_hi = %d\n",__func__, the_data->prox_threshold_hi );
	YL_DEBUG("[ltr558]:>>>>>>>%s :-- prox_threshold_lo = %d\n",__func__, the_data->prox_threshold_lo );
	YL_DEBUG("[ltr558]:>>>>>>>%s :-- CrossTalk = %d\n",__func__, prox_mean);            

	return ret;
}

/* 
 * ###############
 * ## PS CONFIG ##

 * ###############
 */
static int ltr558_ps_enable(int gainrange)
{
	int error=0;
	int setgain;
	char buf[4] = {0};
	char ps_reg[5] = {0};
	int ps_thd_h = 0,ps_thd_l = 0;
	int ps_reg_contr = 0;
  #ifdef YL_DEBUG_TEST//added by maleijie 20121214 for enable invalidate
	struct ltr558_data *ltr558_info =  i2c_get_clientdata( the_data->client);
	u8 data[8];
	int ret;
  #endif
	/* =============== 
	 * ** IMPORTANT **
	 * ===============
	 * Other settings like timing and threshold to be set here, if required.
 	 * Not set and kept as device default for now.
 	 */
      
	// we can get interrupts definitly 
	buf[0] = 0x0;
	buf[1] = 0x0;
	buf[2] = the_data->prox_threshold_hi & 0x0ff;  //up
	buf[3] = (the_data->prox_threshold_hi >> 8) & 0x07;

	//buf[0] = the_data->prox_threshold_lo & 0x0ff;
	//buf[1] = (the_data->prox_threshold_lo >> 8) & 0x07;
	//buf[2] = the_data->prox_threshold_hi & 0x0ff;  //up
	//buf[3] = (the_data->prox_threshold_hi >> 8) & 0x07;
	ltr558_i2c_write_reg(LTR558_PS_THRES_LOW_0, buf[0]);
	ltr558_i2c_write_reg(LTR558_PS_THRES_LOW_1, buf[1]);
	ltr558_i2c_write_reg(LTR558_PS_THRES_UP_0,  buf[2]);
	ltr558_i2c_write_reg(LTR558_PS_THRES_UP_1,  buf[3]);

	switch (gainrange) {
	case LTR558_PS_RANGE1:
		setgain = LTR558_MODE_PS_ON_Gain1;//03
		break;

	case LTR558_PS_RANGE2:
		setgain = LTR558_MODE_PS_ON_Gain4;
		break;

	case LTR558_PS_RANGE4:
		setgain = LTR558_MODE_PS_ON_Gain8;
		break;

	case LTR558_PS_RANGE8:
		setgain = LTR558_MODE_PS_ON_Gain16;
		break;

	default:
		setgain = LTR558_MODE_PS_ON_Gain1;
		break;
	}

	ltr558_i2c_write_reg(LTR558_PS_CONTR, setgain); 
	mdelay(LTR558_WAKEUP_DELAY);

	YL_DEBUG("[ltr558]:LTR558_WAKEUP_DELAY 10ms after write cmd registes\n");

	ps_reg[0] = ltr558_i2c_read_reg(LTR558_PS_CONTR);
	ps_reg[1] = ltr558_i2c_read_reg(LTR558_PS_THRES_UP_0);
	ps_reg[2] = ltr558_i2c_read_reg(LTR558_PS_THRES_UP_1);
	ps_reg[3] = ltr558_i2c_read_reg(LTR558_PS_THRES_LOW_0);
	ps_reg[4] = ltr558_i2c_read_reg(LTR558_PS_THRES_LOW_1);
  #ifdef YL_DEBUG_TEST//added by maleijie 20121214 for enable invalidate
	ret = ltr558_dbg_i2c_read(ltr558_info, LTR558_ALS_DATA_CH1_0, &data[0], 7);
	YL_DEBUG("[ltr558]:>>>>>>>%s:0x88=%2X 0x89=%2X 0x8A=%2X 0x8B=%2X 0x8C=%2X  0x8D=%2X 0x8E=%2X\n", __func__,data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
  #endif
	ps_thd_h = ((ps_reg[2] & 7)* 256) + ps_reg[1];
	ps_thd_l = ((ps_reg[4] & 7)* 256) + ps_reg[3];
	ps_reg_contr = ps_reg[0];
	
	YL_DEBUG("[ltr558]:>>>>>>>%s:ps_thd_h = %d, ps_thd_l = %d, ps_reg_contr = %d\n",
						__func__, ps_thd_h, ps_thd_l, ps_reg_contr);
	
	return error;
}


// Put PS into Standby mode
static int ltr558_ps_disable(void)
{
	int error;
	error = ltr558_i2c_write_reg(LTR558_PS_CONTR, LTR558_MODE_PS_StdBy); 
	return error;
}

static int ltr558_ps_read(void)
{
	int psval_lo, psval_hi, psdata;
	int ret = 0;
	char buf[4]={0};
   	int data = 0;
	int i;//longjiang add 20130328

	psval_lo = ltr558_i2c_read_reg(LTR558_PS_DATA_0);
	if (psval_lo < 0){
		psdata = psval_lo;
		goto out;
	}
		
	psval_hi = ltr558_i2c_read_reg(LTR558_PS_DATA_1);
	if (psval_hi < 0){
		psdata = psval_hi;
		goto out;
	}
		
	psdata = ((psval_hi & 7)* 256) + psval_lo;
	printk("[ltr558]:>>>%s : primitive prox_data psdata = %d\n", __func__, psdata);

	if (psdata < the_data->prox_threshold_lo){   //far
		data = 5;
		last_prox_data = data;//longjiang add 20130508
		if(0 == atomic_read(&calibrate)) { //cal flag
			input_event(the_data->input_dev_proximity, EV_MSC, MSC_SCAN, data);//longjiang modify for fastmmi 20131104
			input_sync(the_data->input_dev_proximity);
		}else{ // calibrate state now
			goto out;
		}  
		buf[0] = 0x0;
		buf[1] = 0x0;
		buf[2] = the_data->prox_threshold_hi & 0x0ff;  //up
		buf[3] = (the_data->prox_threshold_hi >> 8) & 0x07;
	}else if (psdata > the_data->prox_threshold_hi){ //near
		/*longjiang add start 20130418*/
		const int count = 4;
		for(i = 0; i < count; i++)
		{
			mdelay(60);
			psval_lo = ltr558_i2c_read_reg(LTR558_PS_DATA_0);
			psval_hi = ltr558_i2c_read_reg(LTR558_PS_DATA_1);
			psdata = ((psval_hi & 7)* 256) + psval_lo;
			YL_DEBUG("%s: psdata = %d\n", __func__, psdata);

			if(psdata < the_data->prox_threshold_lo)
			{
				break;
			}
		}
		
		if(count == i)
		{
			data = 3;
		}else{
			data = 5;
		}
		last_prox_data = data;//longjiang add 20130508
		 
		if(5 == data)
			goto out;
	    /*longjiang add end 20130418*/
		if(0 == atomic_read(&calibrate)){
			input_event(the_data->input_dev_proximity, EV_MSC, MSC_SCAN, data);//longjiang modify for fastmmi 20131104
			input_sync(the_data->input_dev_proximity);
		}else{  // calibrate state now
			goto out;
		}
		buf[0] = the_data->prox_threshold_lo & 0x0ff;
		buf[1] = (the_data->prox_threshold_lo >> 8) & 0x07;
		buf[2] = 0xff;  //up
		buf[3] = 0xff;
	}

   	ltr558_i2c_write_reg(LTR558_PS_THRES_LOW_0,buf[0]);
   	ltr558_i2c_write_reg(LTR558_PS_THRES_LOW_1,buf[1]);
   	ltr558_i2c_write_reg(LTR558_PS_THRES_UP_0, buf[2]);
   	ltr558_i2c_write_reg(LTR558_PS_THRES_UP_1, buf[3]);

	ret = data;
 
out:
	return ret;
}


/* 
 * ################
 * ## ALS CONFIG ##
 * ################
 */
static int ltr558_als_enable(int gainrange)
{
	int error=0;
	char als_reg[6] = {0};
	int als_thd_h = 0,als_thd_l = 0;
	int als_reg_contr = 0;
	int als_irq_presist = 0;
  	//int final_lux_val;
  #ifdef YL_DEBUG_TEST//added by maleijie 20121214 for enable invalidate
	struct ltr558_data *ltr558_info =  i2c_get_clientdata( the_data->client);
	u8 data[8];
	int ret;
  #endif
	/* =============== 
	 * ** IMPORTANT **
	 * ===============
	 * Other settings like timing and threshold to be set here, if required.
 	 * Not set and kept as device default for now.
 	 */

	ltr558_i2c_write_reg(LTR558_ALS_THRES_LOW_0, 0xff); //0xff
	ltr558_i2c_write_reg(LTR558_ALS_THRES_LOW_1, 0xff); //0xff
	ltr558_i2c_write_reg(LTR558_ALS_THRES_UP_0, 0); //0
	ltr558_i2c_write_reg(LTR558_ALS_THRES_UP_1, 0); //0

	//ltr558_i2c_write_reg(LTR558_INTERRUPT_PERSIST, 0x00);//add by binz for test
	ltr558_i2c_write_reg(LTR558_INTERRUPT, 0x0a);    //add by yulong

	if (gainrange == 1)
		error = ltr558_i2c_write_reg(LTR558_ALS_CONTR, LTR558_MODE_ALS_ON_Range1);
	else if (gainrange == 2)
		error = ltr558_i2c_write_reg(LTR558_ALS_CONTR, LTR558_MODE_ALS_ON_Range2);//03
	else
		error = 1;//flase arg value
        
	mdelay(LTR558_WAKEUP_DELAY);

	als_reg[0] = ltr558_i2c_read_reg(LTR558_ALS_CONTR);
	als_reg[1] = ltr558_i2c_read_reg(LTR558_ALS_THRES_UP_0);
	als_reg[2] = ltr558_i2c_read_reg(LTR558_ALS_THRES_UP_1);
	als_reg[3] = ltr558_i2c_read_reg(LTR558_ALS_THRES_LOW_0);
	als_reg[4] = ltr558_i2c_read_reg(LTR558_ALS_THRES_LOW_1);
	als_reg[5] = ltr558_i2c_read_reg(LTR558_INTERRUPT_PERSIST);
  #ifdef YL_DEBUG_TEST//added by maleijie 20121214 for enable invalidate
	ret = ltr558_dbg_i2c_read(ltr558_info, LTR558_ALS_DATA_CH1_0, &data[0], 7);
	YL_DEBUG("[ltr558]:>>>>>>>%s:0x88=%2X 0x89=%2X 0x8A=%2X 0x8B=%2X 0x8C=%2X  0x8D=%2X 0x8E=%2X\n", __func__,data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
  #endif
	als_thd_h = (als_reg[2]* 256) + als_reg[1];
	als_thd_l = (als_reg[4]* 256) + als_reg[3];
	als_reg_contr = als_reg[0];
	als_irq_presist= als_reg[5];
	
	YL_DEBUG("[ltr558]:>>>>>>>%s:als_thd_h = %d, als_thd_l = %d, als_reg_contr = %d,als_irq_presist =%d\n",
						__func__, als_thd_h, als_thd_l, als_reg_contr, als_irq_presist);
  
	return error;
}

// Put ALS into Standby mode
static int ltr558_als_disable(void)
{
	int error;
	error = ltr558_i2c_write_reg(LTR558_ALS_CONTR, LTR558_MODE_ALS_StdBy); 
	return error;
}

static int ltr558_set_als_threshold(int chanle_val)// add by binz
{
	int als_threshold_lo_param,als_threshold_hi_param;
	int als_buf[4];
	int i,ret,error;
   
	als_threshold_hi_param = (12*chanle_val)/10;
	if (als_threshold_hi_param >= 65535)
		als_threshold_hi_param = 65535;	 
	als_threshold_lo_param = (8*chanle_val)/10;

	als_buf[0] = als_threshold_hi_param & 0x0ff;
	als_buf[1] = als_threshold_hi_param >> 8;
	als_buf[2] = als_threshold_lo_param & 0x0ff;
	als_buf[3] = als_threshold_lo_param >> 8;

	YL_DEBUG("[ltr558]:als_thd_h_0 = %x,als_thd_h_1 = %x,als_thd_l_0 = %x,als_thd_l_1 = %x,als_thd_h = %d,als_thd_l= %d\n",
					als_buf[0],als_buf[1],als_buf[2],als_buf[3],als_threshold_hi_param,als_threshold_lo_param);

	for( i=0; i<4; i++ ) { 
		error = ltr558_i2c_write_reg(LTR558_ALS_THRES_UP_0+i, als_buf[i]); 
		if(error<0){
			printk(KERN_ERR " [ltr558]:i2c_smbus_write_byte_data failed in taos als threshold set\n");
			return (ret);
		}
	}
    
	return ret;
}

static int ltr558_als_read(int gainrange)
{
	int alsval_ch0_lo, alsval_ch0_hi, alsval_ch0;
	int alsval_ch1_lo, alsval_ch1_hi, alsval_ch1;

	int ratio;
	int lux_val;
	int ch0_coeff,ch1_coeff;

	alsval_ch1_lo = ltr558_i2c_read_reg(LTR558_ALS_DATA_CH1_0);
	alsval_ch1_hi = ltr558_i2c_read_reg(LTR558_ALS_DATA_CH1_1);
	alsval_ch1 = (alsval_ch1_hi * 256) + alsval_ch1_lo;

	alsval_ch0_lo = ltr558_i2c_read_reg(LTR558_ALS_DATA_CH0_0);
	alsval_ch0_hi = ltr558_i2c_read_reg(LTR558_ALS_DATA_CH0_1);
	alsval_ch0 = (alsval_ch0_hi * 256) + alsval_ch0_lo;
	
	printk("[ltr558]:als_ch1_h_0 = %x,als_ch1_h_1 = %x,als_ch0_l_0 = %x,als_ch0_l_1 = %x,alsval_ch1 = %d,alsval_ch0 =%d\n",
		alsval_ch1_lo,alsval_ch1_hi,alsval_ch0_lo,alsval_ch0_hi,alsval_ch1,alsval_ch0);

	ltr558_set_als_threshold(alsval_ch0);//add by binz,  value of channel_0  make the als_irq 

	//ratio = alsval_ch1 / alsval_ch0;

	if((alsval_ch0+alsval_ch1) == 0){
		lux_val = 0;//add by binz	
		printk(KERN_ERR "[ltr558]:Both CH0 and CH1 are zeros\n");
		return lux_val;
	}else{
		ratio = (alsval_ch1 * 1000)  / (alsval_ch0+alsval_ch1 );   //*1000
	}
	
	// YL_DEBUG("[ltr558]:>>>>>%s: ratio = %d \n", __func__, ratio);
	if (ratio > 850)
	{
	   ch0_coeff = 0;
	   ch1_coeff = 0;
	}else if(ratio > 640)
	{
	   ch0_coeff = 16900;  //*10000
	   ch1_coeff = 1690;
	}else if(ratio > 450)
	{
	   ch0_coeff = 37725;
	   ch1_coeff = 13363;
	}else{
	   ch0_coeff = 17743;
	   ch1_coeff = -11059;
	}

	lux_val = (alsval_ch0*ch0_coeff - alsval_ch1*ch1_coeff)/10000; //0 to 75 in lux

    switch (als_integration_time)
    {
       case 100:
            itg_tm = 1;
            break;
       case 200:
            itg_tm = 2;
            break;
       case 400:
            itg_tm = 3;
            break;
       default:
            itg_tm = 1;
	}
         
	if(1== als_gainrange){
	lux_val = lux_val/itg_tm/200;
	}

	if(lux_val < 0)  //add by binz, avoid report negative value 
		lux_val = 0;

        lux_val = lux_val * the_data->als_coff1/the_data->als_coff2;
        printk("%s: lux=%d\n",__func__,lux_val);

	return lux_val; /**pc modify from.[lux_val/itg_tm/200].12.04.17**/
	
	// Compute Lux data from ALS data (ch0 and ch1)
	// For Ratio < 0.69:
	// 1.3618*CH0 Ã¢â‚?1.5*CH1
	// For 0.69 <= Ratio < 1:
	// 0.57*CH0 Ã¢â‚?0.345*CH1
	// For high gain, divide the calculated lux by 150.

}

static int ltr558_open(struct inode *inode, struct file *file)
{
	YL_DEBUG("[ltr558]:>>>>%s: >>>>>>>>>>>>>\n", __func__);
	return 0;
}

static int ltr558_release(struct inode *inode, struct file *file)
{
	YL_DEBUG("[ltr558]:>>>>%s: >>>>>>>>>>>>>\n", __func__);
	return 0;
}

static long ltr558_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int init_ps_gain = 0;
	int ret = 0;
	u8 prox_param[5] = {0};
	//longjiang add start 20130520
	struct prox_offset taos_cal_data;
	struct prox_offset *taos_cal_ptr = &taos_cal_data;
	//add end

	switch (cmd) {
		case LTR558_IOCTL_ALS_ON:
			if(0 ==  prox_active) {//add by binz
				if (ltr558_als_enable(als_gainrange))
					return -EIO;
				light_active = 1;
				YL_DEBUG("[ltr558]:>>>>%s: enable ltr558_als light \n", __func__);
			}else{           
				 YL_DEBUG("[ltr558]:>>>>%s: ALS_ON: The ltr558_ps is on ! \n", __func__);
			}
			break;
		
		case LTR558_IOCTL_ALS_OFF:         	
			if(0 ==  prox_active) { //add by binz
				if (ltr558_als_disable())
					return -EIO;
				light_active = 0;
				
				cancel_work_sync(&irq_workqueue);//add by binz
				YL_DEBUG("[ltr558]:>>>>%s:  disable ltr558_als light & prox  \n", __func__);
			}else{
				YL_DEBUG("[ltr558]:>>>>%s: ALS_OFF: The ltr558_ps is on ! \n", __func__);
			}
			break;

		case LTR558_IOCTL_PROX_ON:	
			YL_DEBUG("[ltr558]: %s: PROX_ON Entry\n", __func__);	
			if (1 == atomic_read(&calibrate)){
	           		atomic_set(&calibrate, 0);
			}
						
			ltr558_dev_init();
			if(light_active == 1){//add by binz
				ltr558_als_disable();
			}
			
			if (ltr558_ps_enable(ps_gainrange)) //add by binz
				return -EIO;
			
			if( ltr558_prox_calibrate(taos_cal_ptr,5) < 0){
				printk("%s: PROX_ON ltr558_prox fast calibrate  failed!\n", __func__);
			}
			
			//longjiang add start 20130520
			ret = irq_set_irq_wake(gpio_to_irq(the_data->als_ps_int), 1);
			if(ret < 0){
				printk("[ltr558]: %s: PROX_ON irq_wake failed!\n", __func__);
			}
			prox_active = 1;
			prox_enabled = 1;
			//add end
			
			break;
			
		case LTR558_IOCTL_PROX_OFF:
			//longjiang add start 20130520
			prox_enabled = 0;
			ret = irq_set_irq_wake(gpio_to_irq(the_data->als_ps_int), 0);
			if(ret < 0){
				printk("%s: PROX_OFF irq_wake failed!\n", __func__);
			}
			//add end
			
			if(prox_active == 1){
				if (ltr558_ps_disable())
    				return -EIO;
				prox_active = 0;
				if (0 == light_active){
					cancel_work_sync(&irq_workqueue);		   
					YL_DEBUG("[ltr558]:>>>>%s:  disable ltr558_als light & prox end\n", __func__);
				}else{
					ltr558_als_enable(als_gainrange);//add by binz
					YL_DEBUG("[ltr558]:>>>>%s:  disable ltr558_als proximity end\n", __func__);
				}
			}
			break;
			
		case LTR558_IOCTL_PROX_CALIBRATE:  	
			YL_DEBUG("%s: PROX_CALIBRATE Entry\n", __func__);
                                
			atomic_set(&calibrate, 1);
			if(prox_active || light_active)
            		cancel_work_sync(&irq_workqueue);
                                
			ltr558_ps_disable();//entry standby mode --no interrupts, no measurement ,no data report
			ltr558_i2c_write_reg(LTR558_PS_LED, 0x7b);
 
			if((prox_active == 1)&&(light_active == 1))
				ltr558_i2c_write_reg(LTR558_INTERRUPT, 0x0a);
			if((prox_active == 1)&&(light_active == 0))
				ltr558_i2c_write_reg(LTR558_INTERRUPT, 0x08);
                                   
			the_data->prox_threshold_hi = 0x7ff;
			the_data->prox_threshold_lo = 0x0;
                                 
			//entry ps activity mode
			if (ps_gainrange > 0){
				ltr558_ps_enable(ps_gainrange);
			}else{
				init_ps_gain = LTR558_PS_RANGE2;//LTR558_PS_RANGE1;
				ps_gainrange = init_ps_gain;
				ltr558_ps_enable(init_ps_gain);
			}
                               
			if((ret = ltr558_prox_calibrate(taos_cal_ptr,20)) < 0){
				ret = -EINVAL;
			}
			
			//longjiang add start 20130520
			if(copy_to_user((struct taos_offset *)arg, taos_cal_ptr, sizeof(taos_cal_data)))
			{
				printk(KERN_ERR "%s: data trans error,use default offset ! \r\n",__func__);			
			}
			//add end
                           
			YL_DEBUG("%s: PROX_CALIBRATE end\n", __func__);
			break;

		//longjiang add start 20130520
		case LTR558_IOCTL_PROX_OFFSET:
			YL_DEBUG("%s: PROX_OFFSET Entry\n", __func__);
			sensparams_read_from_flash(SENSPARAMS_TYPE_PROX, prox_param, 5);
			if(prox_param[4] == 1){
				the_data->prox_threshold_hi = (prox_param[1] << 8) | prox_param[0];
				the_data->prox_threshold_lo = (prox_param[3] << 8) | prox_param[2];
			}

			printk(KERN_INFO"%s: prox_hig = [%5d]\n", __func__, the_data->prox_threshold_hi);
			printk(KERN_INFO"%s: prox_low = [%5d] , prox_param[4] = %d\n", __func__, the_data->prox_threshold_lo,prox_param[4]); 

			break;
		//add end
			
		default:
			ret = -EINVAL;
			break;
	}

	return ret;
}

#ifdef CONFIG_SENSOR_POWER
static int sensors_power_on(struct ltr558_data *pdata, bool on)
{
	int rc,is_enabled=0;
	printk("[LTR558]:Enter %s\n",__func__);
	if (!on)
        	goto power_off;
	
	is_enabled =  regulator_is_enabled(the_data->vdd_ana);
	printk("[LTR558]: %s, is_enabled = %d \n",__func__,is_enabled);
	rc = regulator_enable(pdata->vdd_ana);
	if (rc) {
		dev_err(&pdata->client->dev,
			"Regulator vdd_ana enable failed rc=%d\n", rc);
		return rc;
	}
	rc = regulator_enable(pdata->vcc_i2c);
	if (rc) {
        	dev_err(&pdata->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(pdata->vdd_ana);
	}
	if(!rc&&(is_enabled == 1)){
		msleep(LTR558_POWER_ON_DELAY);
	}
	return rc;

power_off:
	rc = regulator_disable(pdata->vdd_ana);
	if (rc) {
		dev_err(&pdata->client->dev,
			"Regulator vdd_ana disable failed rc=%d\n", rc);
		return rc;
	}
	rc = regulator_disable(pdata->vcc_i2c);
	if (rc) {
		dev_err(&pdata->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
		regulator_enable(pdata->vdd_ana);
	}
	return rc;
}

static int sensors_power_init(struct ltr558_data *pdata, bool on)
{
	int rc;
	printk("[LTR558]:Enter %s\n",__func__);

	if (!on)
		goto pwr_deinit;

	pdata->vdd_ana = regulator_get(&pdata->client->dev, "vdd_ana");
	if (IS_ERR(pdata->vdd_ana)) {
		rc = PTR_ERR(pdata->vdd_ana);
		dev_err(&pdata->client->dev,
			"Regulator get failed vdd_ana rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(pdata->vdd_ana) > 0) {
		rc = regulator_set_voltage(pdata->vdd_ana, 2750000,2850000);
		if (rc) {
			dev_err(&pdata->client->dev,
				"Regulator set_vtg failed vdd_ana rc=%d\n", rc);
			goto reg_vdd_ana_put;
		}
    	}

	pdata->vcc_i2c = regulator_get(&pdata->client->dev, "vcc_i2c");
	if (IS_ERR(pdata->vcc_i2c)) {
		rc = PTR_ERR(pdata->vcc_i2c);
		dev_err(&pdata->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_ana_set_vtg;
	}

	if (regulator_count_voltages(pdata->vcc_i2c) > 0) {
		rc = regulator_set_voltage(pdata->vcc_i2c, 1800000,1800000);
		if (rc) {
			dev_err(&pdata->client->dev,
				"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}
	return 0;

reg_vcc_i2c_put:
	regulator_put(pdata->vcc_i2c);
reg_vdd_ana_set_vtg:
	if (regulator_count_voltages(pdata->vdd_ana) > 0)
		regulator_set_voltage(pdata->vdd_ana, 0, 2850000);
reg_vdd_ana_put:
	regulator_put(pdata->vdd_ana);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(pdata->vdd_ana) > 0)
		regulator_set_voltage(pdata->vdd_ana, 0, 2850000);
	regulator_put(pdata->vdd_ana);
	if (regulator_count_voltages(pdata->vcc_i2c) > 0)
		regulator_set_voltage(pdata->vcc_i2c, 0, 1800000);
	regulator_put(pdata->vcc_i2c);
	return 0;
}
#endif

static struct file_operations ltr558_fops = {
	.owner		= THIS_MODULE,
	.open		= ltr558_open,
	.release	= ltr558_release,
	.unlocked_ioctl = ltr558_ioctl,
};
static struct miscdevice ltr558_dev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= LTR558_DEVICE_NAME,
	.fops	= &ltr558_fops,
};

//static void ltr558_schedwork(struct work_struct *work)
static int ltr558_get_data(void)
{
	int als_ps_status;
	int interrupt, newdata;
   	int prox_val = 0;
	int lux_val = 0;
	int ret = 0;//longjiang add 20130508

	als_ps_status = ltr558_i2c_read_reg(LTR558_ALS_PS_STATUS);
	interrupt = als_ps_status & 10;
	newdata = als_ps_status & 5;
	
	YL_DEBUG("[ltr558]: %s: interrupt = %d, newdata = %d\n",__func__, interrupt, newdata);

	switch (interrupt){
		case 2:
			// PS interrupt
			if ((newdata == 1) | (newdata == 5)){
				prox_val = ltr558_ps_read();
			//	ps_data_changed = 1;           //for poll
			}
			//wake_up_interruptible(&ps_waitqueue);//for poll
			break;

		case 8:
			// ALS interrupt
			if ((newdata == 4) | (newdata == 5)){
				lux_val = ltr558_als_read(als_gainrange);
			//	als_data_changed = 1;          //for poll
				input_report_abs(the_data->input_dev_light, ABS_MISC, lux_val);
                                	input_sync(the_data->input_dev_light);
			}
			//wake_up_interruptible(&als_waitqueue);
			break;

		case 10:
			// Both interrupt
			if ((newdata == 1) | (newdata == 5)){
				prox_val = ltr558_ps_read();
			//	ps_data_changed = 1;          //for poll
			}
			//wake_up_interruptible(&ps_waitqueue);

			if ((newdata == 4) | (newdata == 5)){
				lux_val = ltr558_als_read(als_gainrange);
			//	als_data_changed = 1;          //for poll
				input_report_abs(the_data->input_dev_light, ABS_MISC, lux_val);
                                	input_sync(the_data->input_dev_light);
			}
			//wake_up_interruptible(&als_waitqueue);
			break;
		default :
			printk("[ltr558]:%s error interrupt num %d !\n",__func__,interrupt);
			break;
	  }
        
	if(prox_val)
		ret +=1;
	
	return ret;
}

//longjiang add start 20130520
static void ltr558_schedwork(struct work_struct *work)
{
	ltr558_get_data();
	enable_irq(gpio_to_irq(the_data->als_ps_int));
}
//add end

static irqreturn_t ltr558_irq_handler(int irq, void *dev_id)
{
	wake_lock_timeout(&the_data->wake_lock, 3*HZ);//longjiang add 20130423
	YL_DEBUG("[ltr558]:>>>>>>>%s>>>>>>>\n",__func__);
	
	/* disable an irq without waiting */
	disable_irq_nosync(gpio_to_irq(the_data->als_ps_int));

	/* schedule_work - put work task in global workqueue
	 * @work: job to be done
	 *
	 * Returns zero if @work was already on the kernel-global workqueue and
	 * non-zero otherwise.
	 *
	 * This puts a job in the kernel-global workqueue if it was not already
	 * queued and leaves it in the same position on the kernel-global
	 * workqueue otherwise.
	 */
	schedule_work(&irq_workqueue);
	return IRQ_HANDLED;
}

static int ltr558_dev_init(void)
{
	int error;

	int init_ps_gain;
	int init_als_gain;

	//mdelay(PON_DELAY);
	//YL_DEBUG("[ltr558]:>>>>>%s: 600ms\n",__func__);
	//full Gain1 at startup
	init_ps_gain = LTR558_PS_RANGE2;//LTR558_PS_RANGE1;
	ps_gainrange = init_ps_gain;

	init_als_gain = LTR558_ALS_RANGE2_64K; /**pc modify from LTR558_ALS_RANGE1_320.12.04.17**/
	als_gainrange = init_als_gain;
	als_integration_time = ALS_INTEGRATION_TIME;

	ltr558_i2c_write_reg(LTR558_INTERRUPT, 0x0b);
	ltr558_i2c_write_reg(LTR558_PS_LED, 0x7b);    
	ltr558_i2c_write_reg(LTR558_PS_N_PULSES, 0x0C);	//0xA0	//yulong add 20131115
	ltr558_i2c_write_reg(LTR558_PS_MEAS_RATE, 0x00);

	error = 0;

//	out:
	return error;
}

/*return : 0--suc; non_zero fail*/
static int ltr558_inputdev_init (void)
{
   	int ret = 0 ;

	/* alloc light input */
   	the_data->input_dev_light = input_allocate_device();
   	if (the_data->input_dev_light == NULL) {
		return -ENOMEM;
   	}

   	the_data->input_dev_light->name = LTR558_INPUT_NAME_L; //longjiang add platform code 20130520
   	the_data->input_dev_light->id.bustype = BUS_I2C;
	input_set_capability(the_data->input_dev_light,EV_ABS,ABS_MISC);
   	input_set_abs_params(the_data->input_dev_light,ABS_MISC, 0, 1000, 0, 0);//vincent for siji
   	ret = input_register_device(the_data->input_dev_light);
	if (ret != 0){
		YL_DEBUG("[ltr558]:fail to register input device in %s ", __func__); 
		input_free_device(the_data->input_dev_light);
		goto out;
   	}

	/* alloc proximity input */
    the_data->input_dev_proximity = input_allocate_device();
   	if (the_data->input_dev_proximity == NULL) {
   	    input_unregister_device(the_data->input_dev_light);
		input_free_device(the_data->input_dev_light);
		return -ENOMEM;
   	}
   
   	the_data->input_dev_proximity->name = LTR558_INPUT_NAME_P; //longjiang add platform code 20130520
   	the_data->input_dev_proximity->id.bustype = BUS_I2C;
   	set_bit(EV_MSC,the_data->input_dev_proximity->evbit);//longjiang modify for fastmmi 20131104
   	input_set_capability(the_data->input_dev_proximity,EV_MSC,MSC_SCAN);//longjiang modify for fastmmi 20131104
   	ret = input_register_device(the_data->input_dev_proximity);
	if (ret != 0){
		YL_DEBUG("[ltr558]:fail to register input device in %s ", __func__); 
		input_unregister_device(the_data->input_dev_light);
		input_free_device(the_data->input_dev_light);
		input_free_device(the_data->input_dev_proximity);
		goto out;
   	}
out:
	return ret;
}

//longjiang add start 20130520
static void ltr558_inputdev_uninit (void)
{
	if(the_data->input_dev_light)
	{
		input_unregister_device(the_data->input_dev_light);
		the_data->input_dev_light = NULL;
	}

    if(the_data->input_dev_proximity)
	{
		input_unregister_device(the_data->input_dev_proximity);
		the_data->input_dev_proximity = NULL;
	}
}
//add end

// ***** Start ************add for debug by binz************
static ssize_t ltr558_show_all_regs(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltr558_data *ltr558_info = i2c_get_clientdata(client);
	u8 data[25];
	int ret;

	ret = ltr558_dbg_i2c_read(ltr558_info, LTR558_ALS_CONTR, data, 20);//0x80	
	ret = ltr558_dbg_i2c_read(ltr558_info, LTR558_ALS_THRES_UP_0, &data[20], 4);//0x97
	ret = ltr558_dbg_i2c_read(ltr558_info, LTR558_INTERRUPT_PERSIST, &data[24], 1);//0x9E	
	ret = sprintf(buf, "0x80=%2X 0x81=%2X 0x82=%2X 0x83=%2X 0x84=%2X 0x85=%2X  0x86=%2X 0x87=%2X 0x88=%2X 0x89=%2X 0x8A=%2X 0x8B=%2X 0x8C=%2X  0x8D=%2X 0x8E=%2X  0x8F=%2X\n,0x90=%2X  0x91=%2X 0x92=%2X  0x93=%2X, 0x97=%2X  0x98=%2X 0x99=%2X  0x9A=%2X, 0x9E=%2X\n", 
					   data[0], data[1], data[2], data[3], data[4], data[5], data[6],data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15], data[16],data[17], data[18], data[19], data[20], data[21], data[22], data[23], data[24]);
	return ret;	
}

static ssize_t ltr558_reg_addr_store(struct device *dev,
					struct device_attribute *attr,const char *buf, size_t count)
{
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	the_data->reg_addr = val;

	return count;
}

static ssize_t ltr558_reg_simple_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t ret;
	u8 data;
	
	data = ltr558_i2c_read_reg(the_data->reg_addr);
	/*TODO: error need to be managed */
	ret = sprintf(buf, "addr_0x%02x = 0x%02x\n",the_data->reg_addr, data);
	return ret;
}

static ssize_t ltr558_reg_simple_store(struct device *dev,
					struct device_attribute *attr,const char *buf, size_t count)
{
	unsigned long val;
	int rc;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	rc = ltr558_i2c_write_reg(the_data->reg_addr, val);   
	return count;
}

static DEVICE_ATTR(show_all_regs, S_IRUGO, ltr558_show_all_regs, NULL);
static DEVICE_ATTR(reg_addr, S_IWUSR, NULL, ltr558_reg_addr_store);
static DEVICE_ATTR(reg_simple, S_IRUGO|S_IWUSR, ltr558_reg_simple_show, ltr558_reg_simple_store);

static struct attribute *ltr558_attribute[] = {
	&dev_attr_show_all_regs.attr,
	&dev_attr_reg_addr.attr,
	&dev_attr_reg_simple.attr,
	NULL
};

static struct attribute_group ltr558_attribute_group = {
	.attrs = ltr558_attribute
};
// ***** End ************add for debug by binz************

static int ltr558_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
   	int manufac_id;
	char sw_reset = 0;//longjiang add 20130520
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct ltr558_platform_data *pdata = NULL;
	
	ret = gpio_tlmm_config(GPIO_CFG(65, 2, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
				GPIO_CFG_ENABLE);

	printk("[LTR558]...func:%s...probe start..pc trace..\n",__func__);

	/* Return 1 if adapter supports everything we need, 0 if not. */
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WRITE_BYTE | I2C_FUNC_SMBUS_READ_BYTE_DATA))
	{
		printk(KERN_ERR "%s: LTR-558ALS functionality check failed.\n", __func__);
		ret = -EIO;
		goto out;
	}
         

  //longjiang add start 20130913
  #ifdef CONFIG_OF
	if(client->dev.of_node){
		pdata = devm_kzalloc(&client->dev, sizeof(struct ltr558_platform_data), GFP_KERNEL);
		if(NULL == pdata){
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
		ret = ltr558_parse_dt(&client->dev, pdata);
		if(ret){
			dev_err(&client->dev, "Get pdata failed from Device Tree\n");
			return ret;
		}
	}else{
		pdata = client->dev.platform_data;
		if(NULL == pdata){
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
	}
  #else
    pdata = client->dev.platform_data;
	if (NULL == pdata) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

        pdata->als_coff1 = 1;
        pdata->als_coff2 = 1;

  #endif
  //add end

	manufac_id = i2c_smbus_read_byte_data(client, LTR558_MANUFACTURER_ID);
    printk(KERN_INFO "%s: als&ps sensor is selected: manufac_id = %d \n", __func__, manufac_id);
	if (manufac_id != 5)
	{
		ret = manufac_id;
		goto failed_alloc_memory;
	}
        
	/* data memory allocation */
	the_data = (struct ltr558_data*)kzalloc(sizeof(struct ltr558_data), GFP_KERNEL);
	if (the_data == NULL) 
	{
		printk(KERN_ERR "%s: LTR-558ALS kzalloc failed.\n", __func__);
		ret = -ENOMEM;
		goto failed_alloc_memory;
	}

	wake_lock_init(&the_data->wake_lock, WAKE_LOCK_SUSPEND, "ltr558_wake_lock");//longjiang add 20130423
	the_data->client = client;
	i2c_set_clientdata(client, the_data);
    
#ifdef CONFIG_SENSOR_POWER  
	sensors_power_init(the_data, true); 
	sensors_power_on(the_data, true);  
#endif

	the_data->prox_threshold_hi = PROX_THRESHOLD_HI;
	the_data->prox_threshold_lo = PROX_THRESHOLD_LO;

	INIT_WORK(&irq_workqueue, ltr558_schedwork);

	ret = ltr558_inputdev_init();
	if(ret)
	{
		printk(KERN_ERR "%s: LTR-558ALS ltr558_inputdev_init failed!\n", __func__);
		goto failed_inputdev_init;
	}

        the_data->als_coff1 = pdata->als_coff1;
        the_data->als_coff2 = pdata->als_coff2;

	//longjiang add start 20130520
	the_data->als_ps_int = pdata->gpio_int;
	if(the_data->als_ps_int)
	{
		ret = request_irq(gpio_to_irq(the_data->als_ps_int), ltr558_irq_handler, IRQF_TRIGGER_LOW, "ltr558_irq", NULL);
		if (ret != 0) {
			printk(KERN_ALERT "%s: LTR-558ALS request irq failed.\n", __func__);
			//gpio_free(the_data->als_ps_int);
			goto failed_gpio_irq;
		}
	}

	sw_reset = i2c_smbus_read_byte_data(client, LTR558_ALS_CONTR);
	sw_reset |= SENSOR_SW_RESET;
	ltr558_i2c_write_reg(LTR558_ALS_CONTR, sw_reset);
	YL_DEBUG("%s: als&ps device sw_reset:0x80 = 0x%02x.\n", __func__, sw_reset);
	//add end

	ret = ltr558_dev_init();
	if (ret) {
		printk(KERN_ERR "%s: LTR-558ALS device init failed.\n", __func__);
		goto failed_dev_init;
	}

	ret = misc_register(&ltr558_dev);
	if (ret) {
		printk(KERN_ERR "%s: LTR-558ALS misc_register failed.\n", __func__);
		goto failed_misc_register;
	}

	ret = sysfs_create_group(&client->dev.kobj, &ltr558_attribute_group);//add by binz
	if (ret)
	{
	    printk(KERN_ERR "%s: LTR-558ALS sysfs_create_group failed.\n", __func__);
		goto  failed_sysfs_create_group;
	}
		
#ifdef YL_DEBUG_TEST
	ltr558_ps_enable(ps_gainrange);
	ltr558_als_enable(als_gainrange);
#endif
	ret = 0;

	printk("[LTR558]...func:%s...probe success..pc trace..\n",__func__);
	goto out;
	
failed_sysfs_create_group:
	misc_deregister(&ltr558_dev);
failed_misc_register:
failed_dev_init:
failed_gpio_irq:
	ltr558_inputdev_uninit();
failed_inputdev_init:
#ifdef CONFIG_SENSOR_POWER
      sensors_power_on(the_data, false);
      sensors_power_init(the_data, false);	
#endif
	kfree(the_data);
failed_alloc_memory:
	//kfree(pdata);
out:
	return ret;
}

static int ltr558_remove(struct i2c_client *client)
{
	kfree(i2c_get_clientdata(client));

	free_irq(gpio_to_irq(the_data->als_ps_int), NULL);
	gpio_free(the_data->als_ps_int);

	ltr558_ps_disable();//first
	ltr558_als_disable();//interrupts diable too
	
	wake_lock_destroy(&the_data->wake_lock);
#ifdef CONFIG_SENSOR_POWER
	sensors_power_on(the_data, false);
	sensors_power_init(the_data, false);
#endif
	misc_deregister(&ltr558_dev);
	ltr558_inputdev_uninit();
	
	return 0;
}

static int ltr558_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret=0;
	char ret_ps_contr = 0;
	char ret_als_contr = 0;

	YL_DEBUG("***%s*** prox_enabled = %d\n", __func__, prox_enabled);
	if(!prox_enabled)
	{
		disable_irq(gpio_to_irq(the_data->als_ps_int));
		
		ret_ps_contr = ltr558_i2c_read_reg(LTR558_PS_CONTR);	
		ret_als_contr = ltr558_i2c_read_reg(LTR558_ALS_CONTR);

		if(ret_ps_contr & LTR558_PS_MODE_MASK){
			ret = ltr558_ps_disable();
			printk(KERN_ERR">>>>%s: ps_disable in the suspend!\n",__func__);
		}
		
		if(ret_als_contr & LTR558_ALS_CONTR){
			ret = ltr558_als_disable();			
			printk(KERN_ERR">>>>%s: als_disable in the suspend!\n",__func__);
		}

		YL_DEBUG(KERN_ERR">>>>%s,ps_contr = %d ,als_contr=%d >>>>\n",__func__,ret_ps_contr,ret_als_contr);
	}

	return ret;
}

static int ltr558_resume(struct i2c_client *client)
{
	int ret=0;

	YL_DEBUG("***%s*** prox_enabled = %d\n", __func__, prox_enabled);
	if(!prox_enabled)
	{
		if (prox_active || light_active){
			ret = ltr558_dev_init(); 
			if(ret){
				printk(KERN_ERR "%s: LTR-558ALS device init failed.\n", __func__);
				return ret;
			}
		}
		if(prox_active)
			ltr558_ps_enable(ps_gainrange); // first
		if(light_active)
			ltr558_als_enable(als_gainrange);
	}
	//longjiang add start 20130520
 	else
	{
		int ret_data = 0; 
		disable_irq_nosync(gpio_to_irq(the_data->als_ps_int));
 		ret_data = ltr558_get_data();
		YL_DEBUG("%s ret_data = %d, last_prox_data = %d\n", __func__,ret_data,last_prox_data);
		//add (last_prox_data == 5)) for making sure that LCD can be on by pressing power-key,chenyunzhe  
		if (!(((unsigned int)ret_data) & 0x1) && (last_prox_data == 5))  
		{
			input_event(the_data->input_dev_proximity, EV_MSC, MSC_SCAN, 4);//longjiang modify for fastmmi 20131104
			input_sync(the_data->input_dev_proximity);
			mdelay(LTR558_WAKEUP_DELAY);
			input_event(the_data->input_dev_proximity, EV_MSC, MSC_SCAN, last_prox_data);//longjiang modify for fastmmi 20131104
			input_sync(the_data->input_dev_proximity);
			YL_DEBUG("%s ret_data = %d, last_prox_data = %d\n",__func__,ret_data,last_prox_data);
		}		    
	}
	enable_irq(gpio_to_irq(the_data->als_ps_int));
	//add end

	return ret;
}

static const struct i2c_device_id ltr558_id[] = {
	{ DEVICE_I2C_ID, 0 },
	{}
};

//longjiang add start 2013092
#ifdef CONFIG_OF
static struct of_device_id ltr558_match_table[] = {
	{ .compatible = "ltr558",},
	{ },
};
#else
#define ltr558_match_table NULL
#endif
//add end

static struct i2c_driver ltr558_driver = {
	.probe = ltr558_probe,
	.remove = ltr558_remove,
	.id_table = ltr558_id,
	.driver = {
		.owner = THIS_MODULE,
		.name = LTR558_DRIVER_NAME,
	  //longjiang add start 20130925
	  #ifdef CONFIG_OF
        .of_match_table = ltr558_match_table,
      #endif
	  //add end
	},
	.suspend = ltr558_suspend,
	.resume = ltr558_resume,
};

static int __init ltr558_driverinit(void)
{
	printk(KERN_ALERT "<<< %s: LTR-558ALS Driver Module LOADED >>>\n", __func__);
	return i2c_add_driver(&ltr558_driver);
}

static void __exit ltr558_driverexit(void)
{
	i2c_del_driver(&ltr558_driver);
	printk(KERN_ALERT ">>> %s: LTR-558ALS Driver Module REMOVED <<<\n", __func__);
	printk(KERN_INFO "%s\n", "0012345678900");
}

module_init(ltr558_driverinit)
module_exit(ltr558_driverexit)

MODULE_AUTHOR("Lite-On Technology Corp");
MODULE_DESCRIPTION("LTR-558ALS Driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION(DRIVER_VERSION);
