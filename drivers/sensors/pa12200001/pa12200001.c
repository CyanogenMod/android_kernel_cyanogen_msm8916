/*
 * This file is part of the PA12200001 sensor driver.
 * PA12200001 is combined proximity, ambient light sensor and IRLED.
 *
 * Contact: Alan Hsiao <alanhsiao@txc.com.tw>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/string.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
//#include <linux/earlysuspend.h> //changed by taokai 2014.7.1
#include <asm/atomic.h>
#include <linux/sensors/pa12200001.h>
#include <linux/sensors/alsprox_common.h>
#include <linux/sensors/sensparams.h>

#include <linux/of.h>
#include <linux/of_gpio.h>

//#include <linux/yl_pt_ap_feature.h>


#define PA12200001_DRV_NAME "pa12200001"
#define DRIVER_VERSION      "1.00"

//#define DEVICE_NAME_ALPS  ALSPROX_INPUT_NAME


#define PA12_IOCTL_ALS_ON            ALSPROX_IOCTL_ALS_ON
#define PA12_IOCTL_ALS_OFF           ALSPROX_IOCTL_ALS_OFF
#define PA12_IOCTL_PROX_ON           ALSPROX_IOCTL_PROX_ON
#define PA12_IOCTL_PROX_OFF          ALSPROX_IOCTL_PROX_OFF
#define PA12_IOCTL_PROX_CALIBRATE    ALSPROX_IOCTL_PROX_CALIBRATE
#define PA12_IOCTL_PROX_OFFSET       ALSPROX_IOCTL_PROX_OFFSET
#define PA12_IOCTL_PHONE_STATE       ALSPROX_IOCTL_PHONE_STATE

#define APS_TAG                  "[pa12200001]: "
#define APS_FUN(f)               pr_info(APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    pr_err(APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    pr_info(APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    pr_debug(APS_TAG"%s : "fmt, __FUNCTION__, ##args)


static int prox_active = 0;
static int light_active = 0;
unsigned short cal_tmp[4]={0};

struct pa12200001_data {

    struct      i2c_client *client;
    struct      mutex lock;
    int         irq;
    struct      pa12200001_platform_data *pdata;

    struct      workqueue_struct *wq;
//  struct      early_suspend early_suspend;//changed by taokai 2014.7.1

    struct      input_dev *light_input_dev;
    //struct      input_dev  *input_dev;

    struct      work_struct work_light;
    struct      hrtimer light_timer;
    ktime_t     light_poll_delay;

    struct      input_dev *proximity_input_dev;
    struct      work_struct work_proximity;
    struct      hrtimer proximity_timer;
    ktime_t     proximity_poll_delay;

    struct      wake_lock prx_wake_lock;
    ulong       enable;

    unsigned int       als_ps_int;
    /* PS Calibration */
    u16         crosstalk;
    u8      crosstalk_base;
};



struct pa12200001_data *this_data;

/*
 * pa12200001 range
 */
static int pa12200001_range[4] = {149,1363,2833,13897};
u8 suspend_mode;

/* Object status, near=0, far=1 */
static int intr_flag = 1;
static int temp_flag = 1;

/*
 * Set the polling mode(=1) or int mode(=0) here.
 * Please check PA12_PS_SET in pa12200001.h */
u8 als_polling  = 1;
u8 ps_polling   = 0;
/* Please check PA12_PS_SET in pa12200001.h */

/*----------------------------------------------------------------------------*/
/* I2C Read */
static int i2c_read_reg(struct i2c_client *client,u8 reg,u8 *data)
{
  //u8 reg_value[1];
    u8 databuf[2];
    int res = 0;
    int i=0;
    databuf[0]= reg;

    for(i=0;i < I2C_RETRY;i++){
      if(i2c_master_send(client,databuf,0x1) < 0){
        APS_ERR("i2c_master_send function err\n");
        msleep_interruptible(I2C_RETRY_DELAY);
        res= -1;
      }else{
        res=0;
        break;
      }
    }

    for(i=0;i < I2C_RETRY ;i++){
      if(i2c_master_recv(client,data,0x1) < 0){
        APS_ERR("i2c_master_recv function err\n");
        msleep_interruptible(I2C_RETRY_DELAY);
        res= -1;
      }else{
        res=0;
        break;
      }
    }
    return res;
}
/* I2C Write */
static int i2c_write_reg(struct i2c_client *client,u8 reg,u8 value)
{

    u8 databuf[2];
    int res=0;
    int i=0;

    databuf[0] = reg;
    databuf[1] = value;
    for(i=0;i < I2C_RETRY ;i++){
      if(i2c_master_send(client,databuf,0x2) < 0){
        APS_ERR("i2c_master_send function err\n");
        msleep_interruptible(I2C_RETRY_DELAY);
        res=-1;

      }else{
        res=0;
        break;
      }
    }

    return res;
}
/****************************************/
/*
          calibrate
*/
/****************************************/
extern int yl_params_kernel_emmc_write(const char *buf,size_t count);
extern int yl_params_kernel_emmc_read(const char *buf,size_t count);

/*
static void pa12200001_write_crosstalk_to_flash(unsigned char* input)
{
    struct ProductlineInfo Info;
    APS_LOG("%s: write input = %s\n",__func__, input);
    memcpy(&Info.SyncByte, "PRODUCTLINE", sizeof("PRODUCTLINE"));
    yl_params_kernel_emmc_read((char*)&Info, 512);
    if(NULL != input){
        memcpy(&Info.LightProxInfo, input, 4);
    }
    yl_params_kernel_emmc_write((char*)&Info, sizeof(struct ProductlineInfo));
}
static void pa12200001_read_crosstalk_from_flash(unsigned char* output)
{
    struct ProductlineInfo Info;
    memcpy(&Info.SyncByte, "PRODUCTLINE", sizeof("PRODUCTLINE"));
    yl_params_kernel_emmc_read((char*)&Info, 512);
    if(NULL != output){
        memcpy(output, &Info.LightProxInfo, 4);
    }
    APS_LOG("%s: read output = %s\n", __func__, output);
}
#endif
*/

/*----------------------------------------------------------------------------*/
/*
 * internally used functions
 */
static int pa12200001_set_ps_prst(struct i2c_client *client,int pts)
{
    int ret;
    u8 regdata = 0;
    mutex_lock(&this_data->lock);
    ret=i2c_read_reg(client, REG_CFG1, &regdata);
    regdata = regdata & 0xF3; //clear PS PRST bits
    switch(pts){
    case 1:
        regdata = regdata | 0x04 ;
        break;
    case 2:
        regdata = regdata | 0x08 ;
        break;
    case 3:
        regdata = regdata | 0x0C ;
        break;
    default:
        break;
    }
    ret=i2c_write_reg(client,REG_CFG1,regdata);//set PS PRST
    mutex_unlock(&this_data->lock);
    return ret;
}
static int pa12200001_clear_intr(struct i2c_client *client)
{
    struct pa12200001_data *data = i2c_get_clientdata(client);
    u8  regdata;
    int ret;

    mutex_lock(&data->lock);
    ret = i2c_read_reg(client, REG_CFG2, &regdata);
    ret = i2c_write_reg(client, REG_CFG2, regdata & 0xFC);
    mutex_unlock(&data->lock);

    if (ret < 0){
        APS_ERR("i2c_read function err\n");
        return -1;
    }
    return 0;
}
/* mode */
static int pa12200001_get_mode(struct i2c_client *client)
{
    struct pa12200001_data *data = i2c_get_clientdata(client);
    int ret;
    u8  regdata=0;//csl modify 20140107

    mutex_lock(&data->lock);
    ret = i2c_read_reg(client, REG_CFG0, &regdata);
    mutex_unlock(&data->lock);
    return (regdata & 0x03);//get ps and als mode
}

static int pa12200001_set_mode(struct i2c_client *client, int mode)
{
    struct pa12200001_data *data = i2c_get_clientdata(client);
    int ret;
    u8  regdata=0;//csl modify 20140107

    if (mode != pa12200001_get_mode(client))
    {
        if (als_polling)
        {
            /* Enable/Disable ALS */
            if (ALS_ACTIVE & mode)
                hrtimer_start(&data->light_timer, data->light_poll_delay, HRTIMER_MODE_REL);
            else
            {
                hrtimer_cancel(&data->light_timer);
                cancel_work_sync(&data->work_light);
            }
        }

        if (ps_polling)
        {
            /* Enable/Disable PS */
            if ((mode & 0x02) == PS_ACTIVE)
            {
                //wake_lock(&data->prx_wake_lock);
                hrtimer_start(&data->proximity_timer, data->proximity_poll_delay, HRTIMER_MODE_REL);
            }
            else
            {
                //wake_unlock(&data->prx_wake_lock);
                hrtimer_cancel(&data->proximity_timer);
                cancel_work_sync(&data->work_proximity);
            }
        }

        APS_LOG("mode: %d\n",mode);

        mutex_lock(&data->lock);
        ret = i2c_read_reg(client, REG_CFG0, &regdata);
        regdata = regdata & 0xFC;
        ret = i2c_write_reg(client, REG_CFG0, regdata | mode);
        mutex_unlock(&data->lock);
    }
    return 0;
}


static int pa12200001_ps_enable(int flag)
{
    u8 regdata;
printk("zb test :enter %s flag=%d at %d\n",__func__,flag,__LINE__);
    if(flag){
        mutex_lock(&this_data->lock);
        i2c_read_reg(this_data->client, REG_CFG0, &regdata);
        regdata = regdata & 0xFD;
        i2c_write_reg(this_data->client, REG_CFG0, regdata | 0x02);
        mutex_unlock(&this_data->lock);
        /* If use ps_polling start timer */
        if(ps_polling){
        hrtimer_start(&this_data->proximity_timer, this_data->proximity_poll_delay, HRTIMER_MODE_REL);
        }
    }
   else{
    mutex_lock(&this_data->lock);
        i2c_read_reg(this_data->client, REG_CFG0, &regdata);
        regdata = regdata & 0xFD;//close ps
        i2c_write_reg(this_data->client, REG_CFG0, regdata);
        mutex_unlock(&this_data->lock);
        /* If use ps_polling stop timer */
        if(ps_polling){
        hrtimer_cancel(&this_data->proximity_timer);
        cancel_work_sync(&this_data->work_proximity);
        }
    }
    return 0;
}

static int pa12200001_als_enable(int flag)
{
    u8 regdata;
    if(flag){
        mutex_lock(&this_data->lock);
        i2c_read_reg(this_data->client, REG_CFG0, &regdata);
        regdata = regdata & 0xFE;//first clean 0
        i2c_write_reg(this_data->client, REG_CFG0, regdata | 0x01);//enable als
        mutex_unlock(&this_data->lock);
        if(als_polling){
        hrtimer_start(&this_data->light_timer, this_data->light_poll_delay, HRTIMER_MODE_REL);
        }
    }else{
        mutex_lock(&this_data->lock);
        i2c_read_reg(this_data->client, REG_CFG0, &regdata);
        regdata = regdata & 0xFE;
        i2c_write_reg(this_data->client, REG_CFG0, regdata);
        mutex_unlock(&this_data->lock);
        if(als_polling){
        hrtimer_cancel(&this_data->light_timer);
        cancel_work_sync(&this_data->work_light);
        }
    }
    return 0;
}
/* range */
static int pa12200001_get_range(struct i2c_client *client)
{
    struct pa12200001_data *data = i2c_get_clientdata(client);
    int ret = 0;
    u8  regdata=0;//csl modify 20140107
    mutex_lock(&data->lock);
    ret = i2c_read_reg(client, REG_CFG0, &regdata);
    mutex_unlock(&data->lock);
    return ((regdata >> 4) & 0x03);//to get the afsr
}
/*static int pa12200001_set_range(struct i2c_client *client, int range)
{
    struct pa12200001_data *data;
    int ret;
    u8  regdata;

    mutex_lock(&data->lock);
    ret = i2c_read_reg(client, REG_CFG0, &regdata);
    regdata = regdata & 0x03;
    regdata = regdata | (range << 4);
    ret = i2c_write_reg(client, REG_CFG0, regdata);
    mutex_unlock(&data->lock);

    return ret;
}*/

/* ALS set threshold */
/*
static int pa12200001_set_althres(struct i2c_client *client, int val)
{
    struct pa12200001_data *data = i2c_get_clientdata(client);
    int ret;
    u8  msb, lsb;

    msb = val >> 8;
    lsb = val & 0xFF;

    mutex_lock(&data->lock);
    ret = i2c_write_reg(client, REG_ALS_TL_LSB, lsb);
    ret = i2c_write_reg(client, REG_ALS_TL_MSB, msb);
    mutex_unlock(&data->lock);

    return ret;
}

static int pa12200001_set_ahthres(struct i2c_client *client, int val)
{
    struct pa12200001_data *data = i2c_get_clientdata(client);
    int ret;
    u8  msb, lsb;

    msb = val >> 8;
    lsb = val & 0xFF;

    mutex_lock(&data->lock);
    ret = i2c_write_reg(client, REG_ALS_TH_LSB, lsb);
    ret = i2c_write_reg(client, REG_ALS_TH_MSB, msb);
    mutex_unlock(&data->lock);

    return ret;
}
*/

/* PX get threshold */
/*
static int pa12200001_get_plthres(struct i2c_client *client)
{
    struct pa12200001_data *data = i2c_get_clientdata(client);
    int ret;
    u8 regdata;

    mutex_lock(&data->lock);
    ret = i2c_read_reg(client, REG_PS_TL, &regdata);
    mutex_unlock(&data->lock);

    return regdata;
}


static int pa12200001_get_phthres(struct i2c_client *client)
{
    struct pa12200001_data *data = i2c_get_clientdata(client);
    int ret;
    u8 regdata;

    mutex_lock(&data->lock);
    ret = i2c_read_reg(client, REG_PS_TH, &regdata);
    mutex_unlock(&data->lock);

    return regdata;
}
*/
/* PX set threshold */
static int pa12200001_set_plthres(struct i2c_client *client, int val)
{
    struct pa12200001_data *data = i2c_get_clientdata(client);
    int ret;

    mutex_lock(&data->lock);
    ret = i2c_write_reg(client, REG_PS_TL, val);
    mutex_unlock(&data->lock);

    return ret;
}

static int pa12200001_set_phthres(struct i2c_client *client, int val)
{
    struct pa12200001_data *data = i2c_get_clientdata(client);
    int ret;

    mutex_lock(&data->lock);
    ret = i2c_write_reg(client, REG_PS_TH, val);
    mutex_unlock(&data->lock);

    return ret;
}

/*
static int pa12200001_set_pscrosstalk(struct i2c_client *client, int val)
{
    struct pa12200001_data *data = i2c_get_clientdata(client);
    int ret;

    mutex_lock(&data->lock);
    ret = i2c_write_reg(client, REG_PS_OFFSET, val);
    mutex_unlock(&data->lock);

    return ret;
}*/
static int pa12200001_get_pscrosstalk(struct i2c_client *client)
{
    struct pa12200001_data *data = i2c_get_clientdata(client);
    int ret;
    u8  regdata=0;//csl modify 20140107

    mutex_lock(&data->lock);
    ret = i2c_read_reg(client, REG_PS_OFFSET, &regdata);
    mutex_unlock(&data->lock);

    return regdata;
}
void pa12_swap(u8 *x, u8 *y)
{
        u8 temp = *x;
        *x = *y;
        *y = temp;
}
static int pa12200001_run_calibration(struct i2c_client *client)
{
    struct pa12200001_data *data = i2c_get_clientdata(client);
    int i, j;
    int ret;
    u16 sum_of_pdata = 0,als_data=0;
    u8 temp_pdata[20];//csl modify 20140107
    int xtalk_data=0;
    u8 cfg0data=0, cfg2data=0;
     u8 temp_als_data[2];
    unsigned int ArySize = 20;

    if(ps_polling){
        hrtimer_cancel(&this_data->proximity_timer);
        cancel_work_sync(&this_data->work_proximity);
    }
  APS_LOG("%s: START proximity sensor calibration\n", __func__);
    sum_of_pdata = 0;
    mutex_lock(&data->lock);

    i2c_read_reg(client, REG_CFG0, &cfg0data);
    i2c_write_reg(client, REG_CFG0, (cfg0data & 0xCF) | 0x03);  /* PS &ALS Enable ALS Gain = 125 lux */

    i2c_read_reg(client, REG_CFG2, &cfg2data);
    i2c_write_reg(client, REG_CFG2, cfg2data & 0x28); /*Offset mode & disable intr from ps*/

    i2c_write_reg(client, REG_PS_OFFSET, 0x00);//csl modify 20140107 //Set crosstalk = 0

    msleep(200);  //Intergrating Time need 100ms
  i2c_read_reg(client, REG_ALS_DATA_LSB, temp_als_data);
  i2c_read_reg(client, REG_ALS_DATA_MSB, temp_als_data+1);
  als_data=(temp_als_data[1] << 8 )|(temp_als_data[0]);
  if(als_data > PA12_CAL_MAX_ALS_COUNT)
    {
            APS_LOG("%s: ALS ENVIRONMENT IS TOO BRIGHT-> "
                               "cross_talk is set to DEFAULT\n", __func__);
            //data->crosstalk = PA12_PS_OFFSET_DEFAULT;
            //i2c_write_reg(client, REG_PS_OFFSET, data->crosstalk);
            i2c_write_reg(client, REG_CFG0, cfg0data);
            i2c_write_reg(client, REG_CFG2, cfg2data | 0xC0);
            mutex_unlock(&data->lock);
            return 3;//TOO Bright
    }
    for(i = 0; i < 20; i++)
   {
          msleep(15);
        ret = i2c_read_reg(client,REG_PS_DATA,temp_pdata+i);
        //APS_LOG("temp_data = %d\n", temp_pdata[i]);
    }
    mutex_unlock(&data->lock);
    /* pdata sorting */
    for (i = 0; i < ArySize - 1; i++)
        for (j = i+1; j < ArySize; j++)
            if (temp_pdata[i] > temp_pdata[j])
                pa12_swap(temp_pdata + i, temp_pdata + j);
    /* calculate the cross-talk using central 10 data */
    for (i = 14; i < 18; i++)
    {
        //APS_LOG("%s: temp_pdata = %d\n", __func__, temp_pdata[i]);
        sum_of_pdata = sum_of_pdata + temp_pdata[i];
    }
    data->crosstalk = sum_of_pdata/4;
    xtalk_data= (int) data->crosstalk ;
    APS_LOG("%s: sum_of_pdata = %d   cross_talk = %d\n",
                        __func__, sum_of_pdata, data->crosstalk);
  /* Restore CFG2 (Normal mode) and Measure base x-talk */
    mutex_lock(&data->lock);
    i2c_write_reg(client, REG_CFG0, cfg0data);
    i2c_write_reg(client, REG_CFG2, cfg2data | 0xC0);
    mutex_unlock(&data->lock);
    if (data->crosstalk > this_data->pdata->pa12_ps_offset_max)
    {
       //data->crosstalk = data->pdata->pa12_ps_offset_default;
       //i2c_write_reg(client, REG_PS_OFFSET, data->crosstalk);
       return 2;  //struct fail
     }
     data->crosstalk += PA12_PS_OFFSET_EXTRA;

CROSSTALKBASE_RECALIBRATION:
            mutex_lock(&data->lock);
          i2c_write_reg(client, REG_CFG0, cfg0data | 0x02); //PS On
        i2c_write_reg(client, REG_PS_OFFSET, data->crosstalk); //Write offset value to register 0x10

        for(i = 0; i < 10; i++)
        {
            msleep(15);
            i2c_read_reg(client,REG_PS_DATA,temp_pdata+i);
            APS_LOG("temp_data = %d\n", temp_pdata[i]);
        }

        mutex_unlock(&data->lock);

        /* calculate the cross-talk_base using central 5 data */

        sum_of_pdata = 0;

        for (i = 3; i < 8; i++)
        {
            APS_LOG("%s: temp_pdata = %d\n", __func__, temp_pdata[i]);
            sum_of_pdata = sum_of_pdata + temp_pdata[i];
        }

        data->crosstalk_base = sum_of_pdata/5;
            APS_LOG("%s: sum_of_pdata = %d   cross_talk_base = %d\n",
                            __func__, sum_of_pdata, data->crosstalk_base);

        if(data->crosstalk_base > 0)
        {
            data->crosstalk += 1;
            goto CROSSTALKBASE_RECALIBRATION;
        }
      mutex_lock(&data->lock);  //Restore CFG0
      i2c_write_reg(client, REG_CFG0, cfg0data);
        mutex_unlock(&data->lock);
    APS_LOG("%s: FINISH proximity sensor calibration\n", __func__);
    if(ps_polling)
    { hrtimer_start(&data->proximity_timer, data->proximity_poll_delay, HRTIMER_MODE_REL);}
    return 1;
}
#if 1
static int pa12200001_fast_run_calibration(struct i2c_client *client)
{
    struct pa12200001_data *data = i2c_get_clientdata(client);
    int i, j;
    u16 sum_of_pdata = 0;
        u8 temp_pdata[4];//csl modify 20140107
    u8 cfg0data=0, cfg2data=0, cfg3data=0;

        int xtalk_data=0;

    unsigned int ArySize = 4;

        APS_LOG("%s: START fast proximity sensor calibration\n", __func__);

    mutex_lock(&data->lock);
    i2c_read_reg(client, REG_CFG0, &cfg0data);
    i2c_read_reg(client, REG_CFG2, &cfg2data); /*Offset mode & disable intr from ps*/
    i2c_read_reg(client, REG_CFG3, &cfg3data);

    APS_LOG("cfg0 = %x\n cfg2 = %x\n cfg3 = %x\n",cfg0data, cfg2data, cfg3data);

    i2c_write_reg(client, REG_CFG0, cfg0data | 1<<1);  /* PS Enable */
    i2c_write_reg(client, REG_CFG2, cfg2data & 0x28); /*Offset mode & disable intr from ps*/
    i2c_write_reg(client,REG_CFG3, cfg3data & 0xC7); /*Most short PS Sleep time 6.25 ms*/
    i2c_write_reg(client, REG_PS_OFFSET, 0x00);//csl modify 20140107 //Set crosstalk = 0
  msleep(20);
  for(i = 0; i < ArySize; i++){
        msleep(10);
    i2c_read_reg(client,REG_PS_DATA,temp_pdata+i);
        APS_LOG("temp_data = %d\n", temp_pdata[i]);
  }
  mutex_unlock(&data->lock);
    /* pdata sorting */
    for (i = 0; i < ArySize - 1; i++)
        for (j = i+1; j < ArySize; j++)
            if (temp_pdata[i] > temp_pdata[j])
                pa12_swap(temp_pdata + i, temp_pdata + j);
    /* calculate the cross-talk using central 10 data */
     for (i = 1; i < 3; i++) {
        APS_LOG("%s: temp_pdata = %d\n", __func__, temp_pdata[i]);
        sum_of_pdata = sum_of_pdata + temp_pdata[i];
    }
    xtalk_data= sum_of_pdata/2 + PA12_PS_OFFSET_EXTRA;

    if((xtalk_data>=data->crosstalk) && (xtalk_data < (data->crosstalk + this_data->pdata->pa12_fast_cal_tolerance))){    //Max Value is 150,and must be grater than x-talk cal value
        APS_LOG("%s: sum_of_pdata = %d   cross_talk = %d\n",
                        __func__, sum_of_pdata, xtalk_data);
        /*Write offset value to 0x10*/
         mutex_lock(&data->lock);
        i2c_write_reg(client, REG_PS_OFFSET,xtalk_data);
         mutex_unlock(&data->lock);
    }else{
        APS_LOG("%s: invalid calibrated data\n", __func__);
         mutex_lock(&data->lock);
        i2c_write_reg(client, REG_PS_OFFSET,data->crosstalk);
         mutex_unlock(&data->lock);
    }

    APS_LOG("%s: FINISH proximity sensor calibration\n", __func__);


    /*Restore Original Status*/
     mutex_lock(&data->lock);
    i2c_write_reg(client,REG_CFG0,cfg0data); //restore CFG0 Initail status
  i2c_write_reg(client,REG_CFG2,cfg2data | 0xC0); //restore CFG2 Initail status
    i2c_write_reg(client,REG_CFG3,cfg3data); //restore CFG3 Initail status
  mutex_unlock(&data->lock);
    return xtalk_data;
}
#endif

static int pa12200001_get_als_value(struct i2c_client *client)
{
    struct pa12200001_data *data = i2c_get_clientdata(client);
    //u8 lsb, msb;
    u8 lsb=0;//csl modify 20140107
    u8 msb=0;//csl modify 20140107
    int ret;

    mutex_lock(&data->lock);
    ret = i2c_read_reg(client, REG_ALS_DATA_LSB, &lsb);
    ret = i2c_read_reg(client, REG_ALS_DATA_MSB, &msb);
    mutex_unlock(&data->lock);

    return ((msb << 8) | lsb);
}
static int pa12200001_get_lux_value(struct i2c_client *client)
{
    int als_adc = pa12200001_get_als_value(client);
    int lux;
    int range = pa12200001_get_range(client);

    lux = (als_adc * pa12200001_range[range]) >> 12;
    return lux;
}

static int pa12200001_get_ps_value(struct i2c_client *client)
{
    struct pa12200001_data *data = i2c_get_clientdata(client);
    u8  regdata;//csl modify 20140107
    int ret;

    mutex_lock(&data->lock);
    ret = i2c_read_reg(client, REG_PS_DATA, &regdata);
    mutex_unlock(&data->lock);

    return regdata;
    //return ret;//csl modify 20140107
}

static int pa12200001_get_object(struct i2c_client *client)
{
    u8 psdata = pa12200001_get_ps_value(client);
    int ret;
    if(ps_polling)
    {
        APS_LOG("PS:%d\n",psdata);
        if(psdata < this_data->pdata->pa12_ps_th_low)
            intr_flag = 5;
        if(psdata > this_data->pdata->pa12_ps_th_high)
            intr_flag = 3;
    }else{
        switch (this_data->pdata->pa12_int_type)
        {
            case 0: /* Window Type */
                if(intr_flag == 5){
                    //if(psdata > PA12_PS_TH_FAR_HIGH)
                  if(psdata > this_data->pdata->pa12_ps_th_high)
                  {
                        intr_flag = 3;
                        //ret = pa12200001_set_plthres(client, REG_PS_TL, PA12_PS_TH_NEAR_LOW);
                        //ret = pa12200001_set_phthres(client, REG_PS_TH, PA12_PS_TH_NEAR_HIGH);
                        //ret = pa12200001_set_plthres(client, PA12_PS_TH_NEAR_LOW);//csl modify 20140107
                        //ret = pa12200001_set_phthres(client, PA12_PS_TH_NEAR_HIGH);//csl modify 20140107
                        ret = pa12200001_set_plthres(client, this_data->pdata->pa12_ps_th_max);
                        ret = pa12200001_set_phthres(client, this_data->pdata->pa12_ps_th_low);
                        ret = pa12200001_set_ps_prst(client,1);//Set PS PRST = 2 points
                        APS_LOG("------near------ps = %d\n",psdata);
                    }
                }else if(intr_flag == 3){
                    if(psdata < this_data->pdata->pa12_ps_th_low){
                        intr_flag = 5;
                        //ret = pa12200001_set_plthres(client, REG_PS_TL, PA12_PS_TH_FAR_LOW);
                        //ret = pa12200001_set_phthres(client, REG_PS_TH, PA12_PS_TH_FAR_HIGH);
                        //ret = pa12200001_set_plthres(client, PA12_PS_TH_FAR_LOW);
                        //ret = pa12200001_set_phthres(client, PA12_PS_TH_FAR_HIGH);
                        ret = pa12200001_set_plthres(client, this_data->pdata->pa12_ps_th_min);
                        ret = pa12200001_set_phthres(client, this_data->pdata->pa12_ps_th_high);
                        ret = pa12200001_set_ps_prst(client,this_data->pdata->pa12_ps_prst);
                        APS_LOG("------far------ps = %d\n",psdata);
                    }
                }
                pa12200001_clear_intr(client);
                break;

            case 1: /* Hysteresis Type */
                if(psdata > this_data->pdata->pa12_ps_th_high){
                    intr_flag = 3;
                    pa12200001_set_ps_prst(client,1);//Set PS PRST = 2 points
                    APS_LOG("-Hysteresis--near------ps = %d\n",psdata);
                }else if(psdata < this_data->pdata->pa12_ps_th_low){
                    intr_flag = 5;
                    pa12200001_set_ps_prst(client,this_data->pdata->pa12_ps_prst);
                    APS_LOG("-Hysteresis---far------ps = %d\n",psdata);
                }
                /* No need to clear interrupt flag !! */
                goto EXIT_GET_OBJ;
                break;
        }

    }

EXIT_GET_OBJ:
    return intr_flag;
}
#if 0
static int pa12200001_get_intstat(struct i2c_client *client)
{
//  struct pa12200001_data *data = i2c_get_clientdata(client);
    int ret;
    u8  regdata=0;

//  mutex_lock(&data->lock);
    ret = i2c_read_reg(client, REG_CFG2, &regdata);
//  mutex_unlock(&data->lock);

    regdata = regdata & 0x03;
    return regdata;
}
#endif
/*----------------------------------------------------------------------------*/
/* Device Tree */
//add by taokai 2014.7.31
static int pa12200001_parse_configs(struct device *dev, char *name, u32 *array) {
    int rc;
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

/*  dev_err(dev, "%s size is %d\n", name, prop->length/sizeof(u32));
    for (i = 0; i < prop->length/sizeof(u32); i++) {
        dev_info(dev, "arrary[%d]=%d, ", i, array[i]);
    }
    dev_info(dev, "\n");*/
    return rc;
}

static int pa12200001_parse_dt(struct device *dev, struct pa12200001_platform_data *pdata)
{
    int rc = 0;
    int index = 0;
    u32 array[17];
    struct device_node *np = dev->of_node;

    array[14] = PA12_FAST_PS_CAL;
    array[15] = PA12_FAST_CAL_TOLERANCE;
    array[16] = PA12_PS_OFFSET_MAX;

    pdata->gpio_int = of_get_named_gpio_flags(np, "pa12200001,gpio_int",
        0, &pdata->irq_gpio_flags);

    /* general_reg */
    rc = pa12200001_parse_configs(dev, "pa12200001,cfgs", array);
    if (rc) {
        dev_err(dev, "Looking up %s property in node %s failed", "pa12200001,cfgs", np->full_name);
        return -ENODEV;
    }

    pdata->pa12_ps_th_high                      = array[index++];
    pdata->pa12_ps_th_low                       = array[index++];
    pdata->pa12_ps_th_max                       = array[index++];
    pdata->pa12_ps_th_min                       = array[index++];
    pdata->pa12_ps_offset_default               = array[index++];
    pdata->pa12_als_gain                        = array[index++];
    pdata->pa12_led_curr                        = array[index++];
    pdata->pa12_ps_prst                         = array[index++];
    pdata->pa12_als_prst                        = array[index++];
    pdata->pa12_int_set                         = array[index++];
    pdata->pa12_ps_mode                         = array[index++];
    pdata->pa12_int_type                        = array[index++];
    pdata->pa12_ps_period                       = array[index++];
    pdata->pa12_als_period                      = array[index++];
    pdata->pa12_ps_fast_run_cal                 = array[index++];
    pdata->pa12_fast_cal_tolerance              = array[index++];
    pdata->pa12_ps_offset_max                   = array[index++];

    printk("%s:pa12_ps_th_high=%d\n",__func__,pdata->pa12_ps_th_high);
    printk("%s:pa12_ps_th_low=%d\n",__func__,pdata->pa12_ps_th_low);
    printk("%s:pa12_ps_th_max=%d\n",__func__,pdata->pa12_ps_th_max);
    printk("%s:pa12_ps_th_min=%d\n",__func__,pdata->pa12_ps_th_min);
    printk("%s:pa12_ps_offset_default=%d\n",__func__,pdata->pa12_ps_offset_default);
    printk("%s:pa12_als_gain=%d\n",__func__,pdata->pa12_als_gain);
    printk("%s:pa12_led_curr=%d\n",__func__,pdata->pa12_led_curr);
    printk("%s:pa12_ps_prst=%d\n",__func__,pdata->pa12_ps_prst);
    printk("%s:pa12_als_prst=%d\n",__func__,pdata->pa12_als_prst);
    printk("%s:pa12_int_set=%d\n",__func__,pdata->pa12_int_set);
    printk("%s:pa12_ps_mode=%d\n",__func__,pdata->pa12_ps_mode);
    printk("%s:pa12_int_type=%d\n",__func__,pdata->pa12_int_type);
    printk("%s:pa12_ps_period=%d\n",__func__,pdata->pa12_ps_period);
    printk("%s:pa12_als_period=%d\n",__func__,pdata->pa12_als_period);
    printk("%s:pa12_ps_fast_run_cal=%d\n",__func__,pdata->pa12_ps_fast_run_cal);
    printk("%s:pa12_fast_cal_tolerance=%d\n",__func__,pdata->pa12_fast_cal_tolerance);
    printk("%s:pa12_ps_offset_max=%d\n",__func__,pdata->pa12_ps_offset_max);

    return rc;
}
//add end
/*----------------------------------------------------------------------------*/
/* For HAL to Enable PS */
static ssize_t pa12200001_show_enable_ps_sensor(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    return sprintf(buf, "%d\n", (PS_ACTIVE & pa12200001_get_mode(client))?1:0);
}
static ssize_t pa12200001_store_enable_ps_sensor(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct pa12200001_data *data = i2c_get_clientdata(client);
    unsigned long val = simple_strtoul(buf, NULL, 10);
    //unsigned long flags;
    int mode=0;

    printk("%s: enable ps sensor ( %ld)\n", __func__, val);

    if ((val != 0) && (val != 1))//tag debug (chucuo)node ,1:enable,0:disable
    {
        printk("%s: enable ps sensor=%ld\n", __func__, val);
        return count;
    }

    mode = pa12200001_get_mode(client);//PS and ALS is or not enable
    if(val == 1) {
        //turn on ps  sensor
        mode |= PS_ACTIVE;
        set_bit(PS_ACTIVE, &data->enable); //ps-active enable
    }
    else {
        mode &= ~PS_ACTIVE;
        clear_bit(PS_ACTIVE, &data->enable);//ps-active disenable
    }
    pa12200001_set_mode(client, mode);

    return count;
}
static DEVICE_ATTR(enable_ps_sensor, S_IWUSR | S_IRUGO,
                   pa12200001_show_enable_ps_sensor, pa12200001_store_enable_ps_sensor);
/* For HAL to Enable ALS */
static ssize_t pa12200001_show_enable_als_sensor(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    return sprintf(buf, "%d\n", (ALS_ACTIVE & pa12200001_get_mode(client))?1:0);

}
static ssize_t pa12200001_store_enable_als_sensor(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct pa12200001_data *data = i2c_get_clientdata(client);
    unsigned long val = simple_strtoul(buf, NULL, 10);
    //unsigned long flags;
    int mode=0;

    printk("%s: enable als sensor ( %ld)\n", __func__, val);

    if ((val != 0) && (val != 1))
    {
        printk("%s: enable als sensor=%ld\n", __func__, val);
        return count;
    }

    mode = pa12200001_get_mode(client);
    if(val == 1)
  {
        //turn on light  sensor
        mode |= ALS_ACTIVE;
        set_bit(ALS_ACTIVE,&data->enable);
    }
    else {
        mode &= ~ALS_ACTIVE;
        clear_bit(ALS_ACTIVE, &data->enable);
    }
    pa12200001_set_mode(client, mode);

    return count;
}
static DEVICE_ATTR(enable_als_sensor, S_IWUSR | S_IRUGO,
                   pa12200001_show_enable_als_sensor, pa12200001_store_enable_als_sensor);
/* For HAL */
static ssize_t pa12200001_show_als_poll_delay(struct device *dev,
                struct device_attribute *attr, char *buf)
{
//  struct i2c_client *client = to_i2c_client(dev);
//  struct LSC_data *data = i2c_get_clientdata(client);
/*pls finish by yourself*/
    return sprintf(buf, "%d\n", 0); // return in micro-second
}

static ssize_t pa12200001_store_als_poll_delay(struct device *dev,
                    struct device_attribute *attr, const char *buf, size_t count)
{
    //struct i2c_client *client = to_i2c_client(dev);
    //struct LSC_data *data = i2c_get_clientdata(client);
    unsigned long val = simple_strtoul(buf, NULL, 10);
    //int ret;
    //int poll_delay=0;
    //unsigned long flags;

    if (val<5000)
        val = 5000; // minimum 5ms

/*pls finish by yourself*/
    //data->light_poll_delay=ns_to_ktime(val * NSEC_PER_MSEC);

    return count;
}

static DEVICE_ATTR(als_poll_delay, S_IWUSR | S_IRUGO,
                   pa12200001_show_als_poll_delay, pa12200001_store_als_poll_delay);
/* PS Value */
static ssize_t pa12200001_show_ps(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct pa12200001_data *data = i2c_get_clientdata(client);

    return sprintf(buf, "PS = %d\n", pa12200001_get_ps_value(data->client));
}
static DEVICE_ATTR(ps, S_IRUGO,
                   pa12200001_show_ps, NULL);
/* ALS Value */
static ssize_t pa12200001_show_als(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    //struct pa12200001_data *data = i2c_get_clientdata(client);//csl modify 20140107

    return sprintf(buf, "als = %d\n", pa12200001_get_als_value(client));
}
static DEVICE_ATTR(als, S_IRUGO,
                   pa12200001_show_als, NULL);
/* Write/Read Register data */
static ssize_t pa12200001_show_reg(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct pa12200001_data *data = i2c_get_clientdata(client);
    int ret;
    int i = 0;
    int count = 0;
    u8  regdata=0;

    mutex_lock(&data->lock);
    for(i=0;i <18 ;i++)
    {
        ret = i2c_read_reg(client, 0x00+i, &regdata);

        if(ret<0)
        {
           break;
        }
        else
        count += sprintf(buf+count,"[%x] = (%x)\n",0x00+i,regdata);
    }
    mutex_unlock(&data->lock);

    return count;
}
static ssize_t pa12200001_store_reg(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct pa12200001_data *data = i2c_get_clientdata(client);
    int addr, cmd;
    int ret;//csl modify 20140107

    if(2 != sscanf(buf, "%x %x", &addr, &cmd))//2 means two parametre
    {
        APS_ERR("invalid format: '%s'\n", buf);
        return 0;
    }

    mutex_lock(&data->lock);
    ret = i2c_write_reg(client, addr, cmd);
    mutex_unlock(&data->lock);

    return count;
}
static DEVICE_ATTR(reg, S_IWUSR | S_IRUGO,
                   pa12200001_show_reg, pa12200001_store_reg);
/* PS Calibration */
static ssize_t pa12200001_store_ps_calibration(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t count)//csl modify 20140107
{
    struct i2c_client *client = to_i2c_client(dev);//csl modify 20140107
    int xtalk;
    u8 prox_param[4];

    xtalk = pa12200001_run_calibration(client);
    prox_param[0] = xtalk;
    prox_param[1] = this_data->pdata->pa12_ps_th_high;
    prox_param[2] = this_data->pdata->pa12_ps_th_low;
    prox_param[3] = this_data->crosstalk;
    sensparams_write_to_flash(SENSPARAMS_TYPE_PROX, prox_param, 4);
    return xtalk;
}
static ssize_t pa12200001_show_ps_calibration(struct device *dev,
                struct device_attribute *attr, char *buf)//csl modify 20140107
{
    struct i2c_client *client = to_i2c_client(dev);//csl modify 20140107

    u8 regdata;

    regdata = pa12200001_get_pscrosstalk(client);
    //return=sprintf(buf, "PS Crosstalk = %u\n", regdata);
    return sprintf(buf, "PS Crosstalk = %d\n", regdata);//csl modify 20140107
}
static DEVICE_ATTR(ps_calibration, S_IWUSR | S_IRUGO,
                   pa12200001_show_ps_calibration, pa12200001_store_ps_calibration);

static ssize_t pa12200001_show_flash(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    u8  prox_param[4];
    sensparams_read_from_flash(SENSPARAMS_TYPE_PROX,prox_param, 4);//
    return sprintf(buf, "Status = %d,TH High = %d,TH Low = %d,X-talk =%d\n", prox_param[0],prox_param[1],prox_param[2],prox_param[3]);
}
static DEVICE_ATTR(readflash, S_IRUGO,
                   pa12200001_show_flash, NULL); //para 1:node name,/para 2:permission 3:permission function of read 4:permission function of write

/*work que function*/
static void pa12200001_work_func_proximity(struct work_struct *work)
{
    struct pa12200001_data *data = container_of(work,
                        struct pa12200001_data, work_proximity);
    int Pval;

    Pval=pa12200001_get_object(data->client);

    APS_LOG("PS value: %d\n", Pval);

    input_event(data->proximity_input_dev, EV_MSC, MSC_SCAN, Pval);
    input_sync(data->proximity_input_dev);

}
static void pa12200001_work_func_light(struct work_struct *work)
{
    struct pa12200001_data *data = container_of(work, struct pa12200001_data, work_light);
    int Aval;

    Aval = pa12200001_get_lux_value(data->client);
    APS_DBG("ALS lux value: %d\n", Aval);

    input_event(data->light_input_dev, EV_MSC, MSC_SCAN, Aval);
    input_sync(data->light_input_dev);
}

/* assume this is ISR */
static irqreturn_t pa12200001_irq(int irq, void *info)
{

    struct i2c_client *client=(struct i2c_client *)info;
    struct pa12200001_data *data = i2c_get_clientdata(client);

#if 0
    u8 int_stat;
    int_stat = pa12200001_get_intstat(data->client);
    printk(KERN_INFO"pa12200001:%s:int_stat=%d\n",__func__,int_stat);
    /* ALS int */
    if ((int_stat & ALS_INT_ACTIVE) && !als_polling)
    {
        queue_work(data->wq, &data->work_light);
    }
    /* PS int */
    if ((int_stat & PS_INT_ACTIVE) && !ps_polling)
    {
        queue_work(data->wq, &data->work_proximity);
    }
#endif
    printk(KERN_INFO"pa12200001:%s :irq handle\n",__func__);
    queue_work(data->wq, &data->work_proximity);

    return IRQ_HANDLED;
}

/*assume this is timer*/
static enum hrtimer_restart pa12200001_light_timer_func(struct hrtimer *timer)
{
    struct pa12200001_data *data = container_of(timer, struct pa12200001_data, light_timer);

    queue_work(data->wq, &data->work_light);
    hrtimer_forward_now(&data->light_timer, data->light_poll_delay);

    return HRTIMER_RESTART;
}

static enum hrtimer_restart pa12200001_pxy_timer_func(struct hrtimer *timer)
{
    struct pa12200001_data *data = container_of(timer, struct pa12200001_data, proximity_timer);

    queue_work(data->wq, &data->work_proximity);
    hrtimer_forward_now(&data->proximity_timer, data->proximity_poll_delay);

    return HRTIMER_RESTART;
}

//static void pa12200001_timer_init(struct LSC_data *data)
static void pa12200001_timer_init(struct pa12200001_data *data)//csl modify 20140107
{
    if (ps_polling)
    {
        /* proximity hrtimer settings. */
        hrtimer_init(&data->proximity_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        data->proximity_poll_delay = ns_to_ktime(500 * NSEC_PER_MSEC);
        data->proximity_timer.function = pa12200001_pxy_timer_func;
    }

    if (als_polling)
    {
        /* light hrtimer settings. */
        hrtimer_init(&data->light_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
        data->light_poll_delay = ns_to_ktime(300 * NSEC_PER_MSEC);
        data->light_timer.function = pa12200001_light_timer_func;
    }
}
/*--------------*///device node

static struct attribute *pa12200001_attributes[] = {
    &dev_attr_enable_ps_sensor.attr,
    &dev_attr_enable_als_sensor.attr,
    &dev_attr_als_poll_delay.attr,
    &dev_attr_als.attr,
    &dev_attr_ps.attr,
    &dev_attr_reg.attr,
    &dev_attr_ps_calibration.attr,
    &dev_attr_readflash.attr,
    NULL
};

static const struct attribute_group pa12200001_attr_group = {
    .attrs = pa12200001_attributes,
};

/*
 * Initialization function
 */

static int pa12200001_init_client(struct i2c_client *client)
{
    struct pa12200001_data *data = i2c_get_clientdata(client);
    int ret;
    /* Initialize Sensor */
    mutex_lock(&data->lock);
    ret=i2c_write_reg(client,REG_CFG0,
        (data->pdata->pa12_als_gain << 4));//ALS Gain

    ret=i2c_write_reg(client,REG_CFG1,
        ((data->pdata->pa12_led_curr    << 4)| (data->pdata->pa12_ps_prst << 2)| (data->pdata->pa12_als_prst)));// LED Current,PS PRST,ALS PRST

    ret=i2c_write_reg(client,REG_CFG2,
        ((data->pdata->pa12_ps_mode << 6)| (data->pdata->pa12_int_set << 2)));//Noram mode,PS/ALS Interrput enable

    ret=i2c_write_reg(client,REG_CFG3,
        ((data->pdata->pa12_int_type    << 6)| (data->pdata->pa12_ps_period << 3)| (data->pdata->pa12_als_period)));//Interrupt Type,PS sleep time,ALS sleep time

    ret=i2c_write_reg(client,REG_PS_SET,0x03); //PS set should be 0x03

    ret=i2c_write_reg(client,REG_PS_OFFSET, data->pdata->pa12_ps_offset_default); //x-talk offset

    if(!ps_polling)
    {
        /* Set PS threshold */
            ret=i2c_write_reg(client,REG_PS_TH,data->pdata->pa12_ps_th_high); //set TH threshold
            ret=i2c_write_reg(client,REG_PS_TL,data->pdata->pa12_ps_th_low); //set TL threshold
    }
    mutex_unlock(&data->lock);

    if(ret < 0)
    {
        APS_ERR("i2c_send function err\n");
        goto EXIT_ERR;
    }

    return 0;

    EXIT_ERR:
    APS_ERR("pa12200001 init dev fail!!!!: %d\n", ret);
    return ret;
}

static int pa12200001_suspend(struct i2c_client *client, pm_message_t mesg)
{
    APS_LOG("pa12200001 suspend");
    if(light_active)//ALS ON
    {
    APS_LOG("Disalbe ALS for suspend");
    pa12200001_als_enable(0);
    }
    if(prox_active)//PS ON
    {
    APS_LOG("Keep PS awake for suspend");
    #if 0 // yinchao delete wake_lock @20140901
    wake_lock(&this_data->prx_wake_lock);//Keep system awake if PS is enable
    #endif
    }
   APS_LOG("pa12200001 suspend end");
    #if 0  //some problem in following code , maybe crash the system
    struct pa12200001_data *data = container_of(h, struct pa12200001_data, early_suspend);

    APS_LOG("%s line %d \r\n",__func__,__LINE__);


   suspend_mode = pa12200001_get_mode(data->client);

    if(test_bit(ALS_ACTIVE, &data->enable))
        pa12200001_set_mode(data->client, (suspend_mode & ~ALS_ACTIVE));

    if (suspend_mode & PS_ACTIVE)
    {
        APS_LOG("suspend_mode=%d\r\n",suspend_mode);
        if (!ps_polling) {
        enable_irq_wake(data->pdata->irq);
        }
        wake_lock(&data->prx_wake_lock);
    }
    #endif
    return 0;
}

static int pa12200001_resume(struct i2c_client *client)
{
    APS_LOG("pa12200001 resume");
    if(light_active)//ALS ON
    {
    APS_LOG("Enable ALS for resume");
    pa12200001_als_enable(1);
    }
    if(prox_active)//PS ON
    {
    APS_LOG(" Wake_Unlock PS for Resume");
    #if 0 // yinchao delete wake_lock @20140901
    wake_unlock(&this_data->prx_wake_lock);//Unlock wake lock, force not to sleep
    #endif
    }
    #if 0 //some problem in following code , maybe crash the system
    //struct pa12200001_data *data = container_of(h, struct LSC_data, early_suspend);
    struct pa12200001_data *data = container_of(h, struct pa12200001_data, early_suspend);//csl modify 20140107

    if (suspend_mode & PS_ACTIVE)
    {
        APS_LOG("suspend_mode=%d\r\n",suspend_mode);
        if (!ps_polling) {
            disable_irq_wake(data->pdata->irq);
        }
        wake_unlock(&data->prx_wake_lock);
    }

    if(test_bit(ALS_ACTIVE,&data->enable))
    {
        pa12200001_set_mode(data->client, ((test_bit(PS_ACTIVE,&data->enable))?(suspend_mode | ALS_ACTIVE)|PS_ACTIVE:(suspend_mode | ALS_ACTIVE)&~PS_ACTIVE));
    }
    #endif
    return 0;
}


static long pa12200001_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{

//  int init_ps_gain = 0;
    int ret = 0;
    int xtalk=0;
    int i=0;
    u8  prox_param[4];
    u8 first_open_prox = 1;
     switch (cmd) {
        case PA12_IOCTL_ALS_ON:
            pa12200001_als_enable(1);
            light_active = 1;
            APS_LOG("[pa12]:>>>>%s: enable pa12_als light \n", __func__);
       break;
        case PA12_IOCTL_ALS_OFF:
            pa12200001_als_enable(0);
            light_active = 0;
  /*
            if(0 ==  prox_active) { //add by binz
                    //need ALS ON

                light_active = 0;
                //cancel_work_sync(&irq_workqueue);//add by binz
                APS_LOG("[pa12]:>>>>%s:  disable pa12_als light & prox  \n", __func__);
            }else{
                APS_LOG("[pa12]:>>>>%s: PA12_IOCTL_ALS_OFF: The pa12200001_ps is on ! \n", __func__);
                }
    */
            break;

        case PA12_IOCTL_PROX_ON:
            if(this_data->pdata->pa12_ps_fast_run_cal)
               pa12200001_fast_run_calibration(this_data->client);




            pa12200001_ps_enable(1);
            printk("zb test :enter PA12_IOCTL_PROX_ON %d at %d\n",PA12_FAST_PS_CAL,__LINE__);

            msleep(60);

            {
                int val = 0;
                val = pa12200001_get_object(this_data->client);

                printk("-!!!!-chenyz!!!!:val=%d,first_open_prox=%d,temp_flag=%d\n",val,first_open_prox,temp_flag);

                if(((1 == temp_flag)&&(1 == first_open_prox))||((temp_flag != 1)&&(temp_flag == val)))//temp is not exit the first time
                {
                    if(temp_flag != 3){     //judge the state of proximity sensor is far
                        input_event(this_data->proximity_input_dev, EV_MSC, MSC_SCAN, val);
                        input_sync(this_data->proximity_input_dev);
                    }
                }

            }



#if 0  //chenyunzhe remove this codes below for no need to force PS report system  status becase of MISC_SCAN 
            if(!ps_polling)//force trigger once
            {
                intr_flag = 0;
                APS_LOG("ioctr ---intr_flag--- = %d\n",intr_flag);
                queue_work(this_data->wq, &this_data->work_proximity);
            }
#endif



            enable_irq(this_data->als_ps_int);
            prox_active = 1;



            // yinchao add irq_wake @20140901
            ret = irq_set_irq_wake(this_data->als_ps_int, 1);
            if (ret)
            {
                printk(KERN_ERR "irq_set_irq_wake(on) fail\n");

                return ret;
            }
            APS_LOG("[pa12]:>>>>%s: enable pa12_als proximity end\n", __func__);
            break;

        case PA12_IOCTL_PROX_OFF:
            // yinchao add irq_wake @20140901
            ret = irq_set_irq_wake(this_data->als_ps_int, 0);
            if (ret)
            {
                printk(KERN_ERR "irq_set_irq_wake(off) fail\n");

                return ret;
            }
            pa12200001_ps_enable(0);
            first_open_prox = 0;
            temp_flag = intr_flag;
            disable_irq_nosync(this_data->als_ps_int);
            prox_active = 0;

            break;

        case PA12_IOCTL_PROX_CALIBRATE:

            APS_LOG("Run PS Calibration !\n");
            //pa12200001_als_enable(0); //release timer before calibration
            xtalk=pa12200001_run_calibration(this_data->client);//Run PS Calibration
            //if(prox_active)
            //pa12200001_ps_enable(1);

        cal_tmp[0] = xtalk;
            cal_tmp[1] = (unsigned short) this_data->pdata->pa12_ps_th_high;
            cal_tmp[2] = (unsigned short) this_data->pdata->pa12_ps_th_low;
            cal_tmp[3] = (unsigned short) this_data->crosstalk;
            for(i=0 ; i < 4 ; i++)
            {prox_param[i]=(u8) cal_tmp[i];}

            if(cal_tmp[0] == 1){
            sensparams_write_to_flash(SENSPARAMS_TYPE_PROX, prox_param, 4);
            }else{
            sensparams_write_to_flash(SENSPARAMS_TYPE_PROX, prox_param, 1);
            }
            if (copy_to_user((unsigned short*)arg, cal_tmp, sizeof(cal_tmp)))
                return -EFAULT;
            break;

        case PA12_IOCTL_PROX_OFFSET:
            APS_LOG("Read PROX_PRAMS from flash  !\n");
            sensparams_read_from_flash(SENSPARAMS_TYPE_PROX,prox_param, 4);
            mutex_lock(&this_data->lock);
            this_data->crosstalk=prox_param[3];
            i2c_write_reg(this_data->client, REG_PS_OFFSET,prox_param[3]);
      mutex_unlock(&this_data->lock);
            break;

        default:
            ret = -EINVAL;
            break;
    }

    return ret;
}

static int pa12200001_open(struct inode *inode, struct file *file)
{
    u8 prox_param[4]={0};
    struct pa12200001_data *data = i2c_get_clientdata(this_data->client);
    APS_LOG("pa122_open_start read flash");
    sensparams_read_from_flash(SENSPARAMS_TYPE_PROX, prox_param, 4);
    if(prox_param[0] == 1)
    {
         mutex_lock(&this_data->lock);
    i2c_write_reg(this_data->client, REG_PS_OFFSET,prox_param[3]);
       mutex_unlock(&this_data->lock);
    data->crosstalk = prox_param[3];
    }
   else{
          mutex_lock(&this_data->lock);
    i2c_write_reg(this_data->client, REG_PS_OFFSET,data->pdata->pa12_ps_offset_default);
          mutex_unlock(&this_data->lock);
    data->crosstalk = data->pdata->pa12_ps_offset_default;
    }
    APS_LOG("pa122_open_read flash finish");

    return 0;
}

static int pa12200001_release(struct inode *inode, struct file *file)
{
    return 0;
}

static struct file_operations pa12200001_fops = {
    .owner      = THIS_MODULE,
    .open       = pa12200001_open,
    .release    = pa12200001_release,
    .unlocked_ioctl = pa12200001_ioctl,
};

static struct miscdevice pa12200001_dev = {
    .minor  = MISC_DYNAMIC_MINOR,
    .name   = ALSPROX_DEVICE_NAME,
    .fops   = &pa12200001_fops,
};

/*
 * I2C init/probing/exit functions
 */

static int /*__devinit*/ pa12200001_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct pa12200001_data *data;
    struct pa12200001_platform_data *pdata=NULL;
    int err = 0;
    int ret = 0;
    //u8 prox_param[4]={0};

    //Device Tree Modify
    if (client->dev.of_node) {
        pdata = devm_kzalloc(&client->dev, sizeof(struct pa12200001_platform_data), GFP_KERNEL);
        if (NULL == pdata) {
            dev_err(&client->dev, "Failed to allocate memory\n");
            return -ENOMEM;
        }
        ret = pa12200001_parse_dt(&client->dev, pdata);
        if (ret) {
            dev_err(&client->dev, "Get pdata failed from Device Tree\n");
            return ret;
        }
    } else {
        pdata = client->dev.platform_data;
        if (NULL == pdata) {
            dev_err(&client->dev, "pdata is NULL\n");
            return -ENOMEM;
        }
    }
    //Device Tree Modify

    APS_DBG("probe start!\n" );


    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
        return -EIO;

    data = kzalloc(sizeof(struct pa12200001_data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    data->client = client;
    data->pdata  = pdata;
    i2c_set_clientdata(client, data);
    mutex_init(&data->lock);

    wake_lock_init(&data->prx_wake_lock, WAKE_LOCK_SUSPEND, "prx_wake_lock");

    INIT_WORK(&data->work_proximity, pa12200001_work_func_proximity);
    INIT_WORK(&data->work_light, pa12200001_work_func_light);

    pa12200001_timer_init(data);

    /* initialize the AP321X chip */
    err = pa12200001_init_client(client);
    if (err)
        goto exit_kfree;
    this_data = data;

    /* allocate input_device */
    data->light_input_dev= input_allocate_device();
    if (!data->light_input_dev) {
        printk(KERN_ERR"%s: allocate light input device fail !\n", __func__);
        ret = -1;
        goto exit_kfree;
    }
    data->light_input_dev->name = PA12200001_INPUT_NAME_L;
    data->light_input_dev->id.bustype = BUS_I2C;
    set_bit(EV_MSC, data->light_input_dev->evbit);//longjiang modify for fastmmi 20131104
    input_set_capability(data->light_input_dev, EV_MSC, MSC_SCAN);//longjiang modify for fastmmi 20131104
    /* light Lux data */
    ret = input_register_device(data->light_input_dev);
    if (ret != 0) {
        printk(KERN_ERR"%s: light_input_register_device failed ! \n", __func__);
        goto exit_free_dev_als;
    }
    data->proximity_input_dev= input_allocate_device();
    if (!data->proximity_input_dev) {
        printk(KERN_ERR"%s: allocate proximity input device fail !\n", __func__);
        ret = -1;
        goto exit_kfree;
    }
    data->proximity_input_dev->name = PA12200001_INPUT_NAME_P;
    data->proximity_input_dev->id.bustype = BUS_I2C;
    set_bit(EV_ABS, data->proximity_input_dev->evbit);//longjiang modify for fastmmi 20131104
    input_set_capability(data->proximity_input_dev, EV_MSC, MSC_SCAN);
    /* proximity data */
    ret = input_register_device(data->proximity_input_dev);
    if (ret != 0) {
        printk(KERN_ERR"%s: proximity_input_register_device failed ! \n", __func__);
        goto exit_free_dev_als;
    }

    //register misc//
    ret = misc_register(&pa12200001_dev);
    if (ret) {
        printk(KERN_ERR "%s: PA12 ALS misc_register failed.\n", __func__);
        goto failed_misc_register;
    }

    ret = sysfs_create_group(&client->dev.kobj, &pa12200001_attr_group);//csl modify 20140107
    if (ret) {
        APS_LOG("could not create sysfs group\n");
        goto exit_unregister_dev_ps;
    }

    data->wq = create_singlethread_workqueue("pa12200001_wq");
    if (!data->wq) {
        APS_LOG("could not create workqueue\n");
        goto exit_work;
    }

    if (!als_polling || !ps_polling)//if lianzhezhongyouyigeweiyi
    {

        gpio_request(pdata->gpio_int, PA12200001_DRV_NAME);
        gpio_direction_input(pdata->gpio_int);

        data->als_ps_int = gpio_to_irq(pdata->gpio_int);//shenqignzhognduanhao

        err = request_irq(data->als_ps_int,pa12200001_irq, (data->pdata->pa12_int_type == 1) ? (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING) : IRQF_TRIGGER_FALLING,
                          PA12200001_DRV_NAME,(void *)client);
/*
        err = request_threaded_irq(data->pdata->irq,pa12200001_irq, NULL,
                (data->pdata->pa12_int_type == 1) ? (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING) : IRQF_TRIGGER_FALLING,
                PA12200001_DRV_NAME, (void *)client);
*/
        if (err != 0) {
            APS_DBG("request irq fail!, irq = %d\n", data->als_ps_int);
//          goto exit_remove_sysfs;
        }else{
        APS_DBG("request irq success, irq = %d\n", data->als_ps_int);
        disable_irq_nosync(data->als_ps_int);
        }
    }

    //Read X-talk Calibration value from flash
#if 0
    sensparams_read_from_flash(SENSPARAMS_TYPE_PROX, prox_param, 4);
    if(prox_param[0] == 1)
    {
    i2c_write_reg(client, REG_PS_OFFSET,prox_param[3]);
    data->crosstalk = prox_param[3];
    }
   else{
    i2c_write_reg(client, REG_PS_OFFSET,data->pdata->pa12_ps_offset_default);
    data->crosstalk = data->pdata->pa12_ps_offset_default;
    }
#endif
        #if 0
#ifdef CONFIG_PM
    data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
    data->early_suspend.suspend = pa12200001_early_suspend;
    data->early_suspend.resume = pa12200001_late_resume;
    register_early_suspend(&data->early_suspend);
#endif /* CONFIG_PM */
    #endif

    device_init_wakeup(&client->dev, 1);
    dev_info(&client->dev, "Driver version %s enabled\n", DRIVER_VERSION);
    APS_DBG("probe end!\n" );
    return 0;

//exit_remove_sysfs:
//  destroy_workqueue(data->wq);

failed_misc_register:
    misc_deregister(&pa12200001_dev);

exit_work:
    sysfs_remove_group(&client->dev.kobj, &pa12200001_attr_group);//csl modify 20140107

exit_unregister_dev_ps:
    input_unregister_device(data->light_input_dev);
    input_unregister_device(data->proximity_input_dev);
//exit_free_dev_ps:
    //input_free_device(data->input_dev);

exit_free_dev_als:
    input_free_device(data->light_input_dev);
    input_free_device(data->proximity_input_dev);

exit_kfree:
    wake_lock_destroy(&data->prx_wake_lock);
    mutex_destroy(&data->lock);
    kfree(data);
    return err;
}

static int /*__devexit*/ pa12200001_remove(struct i2c_client *client)
{
    struct pa12200001_data *data = i2c_get_clientdata(client);

    if (pa12200001_get_mode(client)& PS_ACTIVE)
    {
        if (ps_polling)
        hrtimer_cancel(&data->proximity_timer);
        cancel_work_sync(&data->work_proximity);
    }
    if (pa12200001_get_mode(client) & ALS_ACTIVE)
    {
        if (als_polling)
        hrtimer_cancel(&data->light_timer);
        cancel_work_sync(&data->work_light);
    }

    input_unregister_device(data->light_input_dev);
    input_unregister_device(data->proximity_input_dev);

    input_free_device(data->light_input_dev);
    input_free_device(data->proximity_input_dev);
//    input_free_device(data->proximity_input_dev);

    if (!als_polling || !ps_polling)
        free_irq(data->als_ps_int, data);

    sysfs_remove_group(&client->dev.kobj, &pa12200001_attr_group);

    /* Power down the device */
    pa12200001_set_mode(client, 0);

    destroy_workqueue(data->wq);
    mutex_destroy(&data->lock);
    wake_lock_destroy(&data->prx_wake_lock);
//#ifdef CONFIG_PM
//  unregister_early_suspend(&data->early_suspend);
//#endif
    kfree(data);

    return 0;
}


static const struct i2c_device_id pa12200001_id[] = {
    { PA12200001_DRV_NAME, 0 },
    {}
};
MODULE_DEVICE_TABLE(i2c, pa12200001_id);

static struct of_device_id txc_match_table[] = {
    { .compatible = "pa12200001",},
    { },
};

static struct i2c_driver pa12200001_driver = {
    .driver = {
        .name   = PA12200001_DRV_NAME,
        .owner  = THIS_MODULE,
        .of_match_table = txc_match_table,
    },
    .probe  = pa12200001_probe,
    .remove = pa12200001_remove ,/*__devexit_p(pa12200001_remove),*/
    .id_table = pa12200001_id,
    .suspend   = pa12200001_suspend,    //Power Manager
    .resume    = pa12200001_resume,     //Power Manager
};



static int __init pa12200001_init(void)
{
    return i2c_add_driver(&pa12200001_driver);
}

static void __exit pa12200001_exit(void)
{
    i2c_del_driver(&pa12200001_driver);
}

MODULE_AUTHOR("Sensor Team, TXC");
MODULE_DESCRIPTION("PA12200001 ambient light + proximity sensor driver.");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

late_initcall(pa12200001_init);
module_exit(pa12200001_exit);
