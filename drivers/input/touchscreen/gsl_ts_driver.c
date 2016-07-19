/* drivers/input/touchscreen/gslX68X.h
 * 
 * 2010 - 2013 SLIEAD Technology.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be a reference 
 * to you, when you are integrating the SLIEAD's CTP IC into your system, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * General Public License for more details.
 * 
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/byteorder/generic.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif
#include <linux/firmware.h>
#include <linux/proc_fs.h>
#include <linux/regulator/consumer.h> 
#include <linux/of_gpio.h>
#include <linux/time.h>
#include "gsl_ts_driver.h"

#ifdef GSL_REPORT_POINT_SLOT
    #include <linux/input/mt.h>
#endif

/** TODO don't need this anymore. ddata->hw_lock gives the same protection */
static struct mutex gsl_i2c_lock;

/* Print Information */
#ifdef GSL_DEBUG 
#define print_info(fmt, args...)   \
        do{                              \
                printk("[tp-gsl][%s]"fmt,__func__, ##args);     \
        }while(0)
#else
#define print_info(fmt, args...)   //
#endif

/* Timer Function */
#ifdef GSL_TIMER
#define GSL_TIMER_CHECK_CIRCLE        200
#define GSL_TIMER_WATCHDOG_INTERVAL   HZ /* 1 second */
static struct delayed_work gsl_timer_check_work;
static struct workqueue_struct *gsl_timer_workqueue = NULL;
#endif

/* Gesture Resume */
//#define GSL_GESTURE
#ifdef GSL_GESTURE
typedef enum{
	GE_DISABLE = 0,
	GE_ENABLE = 1,
	GE_WAKEUP = 2,
	GE_NOWORK =3,
}GE_T;
static GE_T gsl_gesture_status = GE_DISABLE;
static volatile unsigned int gsl_gesture_flag = 0;
static char gsl_gesture_c = 0;
static bool dozing = false;
struct timeval startup;
#endif
struct timer_list startup_timer;
static bool timer_expired = false;

static volatile int gsl_halt_flag = 0;

/* Proximity Sensor */
//#define GSL_PROXIMITY_SENSOR
#ifdef GSL_PROXIMITY_SENSOR
#include <linux/wakelock.h>
#include <linux/sensors.h>
	//zhangpeng add start
	//zhangpeng add end
#endif

#if defined(GSL_PROXIMITY_SENSOR)
	static u8 bNeedResumeTp = 0;
#endif

/* Process for Android Debug Bridge */
#ifdef TPD_PROC_DEBUG
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/seq_file.h>
//static struct proc_dir_entry *gsl_config_proc = NULL;
#define GSL_CONFIG_PROC_FILE "gsl_config"
#define CONFIG_LEN 31
static char gsl_read[CONFIG_LEN];
static u8 gsl_data_proc[8] = {0};
static u8 gsl_proc_flag = 0;
#endif

#define GSL_TEST_TP
#ifdef GSL_TEST_TP
extern void gsl_write_test_config(unsigned int cmd,int value);
extern unsigned int gsl_read_test_config(unsigned int cmd);
extern int gsl_obtain_array_data_ogv(unsigned short *ogv,int i_max,int j_max);
extern int gsl_obtain_array_data_dac(unsigned int *dac,int i_max,int j_max);
extern int gsl_tp_module_test(char *buf,int size);
#define GSL_PARENT_PROC_NAME 	"silead-ito-test"	//"touchscreen"
#define GSL_OPENHSORT_PROC_NAME "debug"			//"ctp_openshort_test"
#endif
static char tp_info[20];

static void gsl_sw_init(struct i2c_client *client);

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,unsigned long event, void *data);
#endif

#ifdef GSL_GESTURE
#define NUM_GESTURES KEY_F4
static DECLARE_BITMAP(gesture_bmp, NUM_GESTURES);
#endif

#ifdef TOUCH_VIRTUAL_KEYS
static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{

	return sprintf(buf,
         __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":120:900:40:40"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":240:900:40:40"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":360:900:40:40"
	 "\n");
}

static struct kobj_attribute virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.GSL_TP",
		.mode = S_IRUGO,
	},
	.show = &virtual_keys_show,
};

static struct attribute *properties_attrs[] = {
	&virtual_keys_attr.attr,
	NULL
};

static struct attribute_group properties_attr_group = {
	.attrs = properties_attrs,
};

static void gsl_ts_virtual_keys_init(void)
{
	int ret;
	struct kobject *properties_kobj;

	print_info("%s\n",__func__);

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
			&properties_attr_group);
	if (!properties_kobj || ret)
		pr_err("failed to create board_properties\n");
}

#endif


/*define golbal variable*/
static struct gsl_ts_data *ddata = NULL;

static int gsl_read_interface(struct i2c_client *client, u8 reg, u8 *buf, u32 num)
{
	int err = 0;
	u8 temp = reg;
	mutex_lock(&gsl_i2c_lock);
	if(temp < 0x80)
	{
		i2c_master_send(client,&temp,1);
		err = i2c_master_recv(client,&buf[0],num);
	}
	else
	{
		i2c_master_send(client,&temp,1);
		err = i2c_master_recv(client,&buf[0],num);
	}
	mutex_unlock(&gsl_i2c_lock);

	if (err < 0)
		return err;

	return (err == num) ? 0 : -EIO;
}

static int gsl_read32(struct i2c_client *client, u8 reg, u32 *out) {
	int rc = gsl_read_interface(client, reg, (u8 *) out, 4);
	if (rc)
		return rc;

	*out  = le32_to_cpu(*out);
	return 0;
}

static int gsl_ito_read_interface(struct i2c_client *client, u8 reg, u8 *buf, u32 num)
{
	int err = 0;
	u8 temp = reg;
	mutex_lock(&gsl_i2c_lock);
	if(temp < 0x80)
	{
		temp = (temp+8)&0x5c;
			i2c_master_send(client,&temp,1);	
			err = i2c_master_recv(client,&buf[0],4);
			
			temp = reg;
			i2c_master_send(client,&temp,1);	
			err = i2c_master_recv(client,&buf[0],4);
	}
	i2c_master_send(client,&reg,1);
	err = i2c_master_recv(client,&buf[0],num);
	mutex_unlock(&gsl_i2c_lock);
	return (err == num)?1:-1;

}

static int gsl_write_interface(struct i2c_client *client, const u8 reg, u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[1];
	int err;
	u8 tmp_buf[num+1];
	tmp_buf[0] = reg;
	memcpy(tmp_buf + 1, buf, num);
	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = num + 1;
	xfer_msg[0].flags = client->flags & I2C_M_TEN;
	xfer_msg[0].buf = tmp_buf;
	//xfer_msg[0].timing = 400;

	mutex_lock(&gsl_i2c_lock);
	err= i2c_transfer(client->adapter, xfer_msg, 1);
	mutex_unlock(&gsl_i2c_lock);
	return err;	
//	return i2c_transfer(client->adapter, xfer_msg, 1) == 1 ? 0 : -EFAULT;
}

#if 0
static int gsl_ts_write(struct i2c_client *client, const u8 reg, u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[1];
	int err;
	u8 tmp_buf[num+1];
	tmp_buf[0] = reg;
	memcpy(tmp_buf + 1, buf, num);
	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = num + 1;
	xfer_msg[0].flags = client->flags & I2C_M_TEN;
	xfer_msg[0].buf = tmp_buf;
	//xfer_msg[0].timing = 400;

	mutex_lock(&gsl_i2c_lock);
	err= i2c_transfer(client->adapter, xfer_msg, 1);
	mutex_unlock(&gsl_i2c_lock);
	return err;	
//	return i2c_transfer(client->adapter, xfer_msg, 1) == 1 ? 0 : -EFAULT;
}


static int gsl_ts_read(struct i2c_client *client, u8 reg, u8 *buf, u32 num)
{

	int err = 0;
	u8 temp = reg;
	mutex_lock(&gsl_i2c_lock);
	if(temp < 0x80)
	{
		temp = (temp+8)&0x5c;
		i2c_master_send(client,&temp,1);
		err = i2c_master_recv(client,&buf[0],4);

		temp = reg;
		i2c_master_send(client,&temp,1);
		err = i2c_master_recv(client,&buf[0],4);
	}
	i2c_master_send(client,&reg,1);
	err = i2c_master_recv(client,&buf[0],num);
	mutex_unlock(&gsl_i2c_lock);
	return err;
}
#endif

static int gsl_ts_read_version(void)
{
	u8  buf[4] = {0};
	int ret = 0; 
	buf[0] = 0x03;
	gsl_write_interface(ddata->client, 0xf0, buf, 4);
	gsl_read_interface(ddata->client, 0x04, buf, 4);
	print_info("[%s] The firmware version is %x%x%x%x",__func__, buf[3], buf[2], buf[1] ,buf[0]);

	ret = (buf[3]<<24) | (buf[2]<<16) | (buf[1]<<8) | (buf[0]);

	return ret;
}

#ifdef GSL_GESTURE
#define READ_LEN 64
static int gsl_ts_write(struct i2c_client *client, u8 addr, u8 *pdata,
			int datalen)
{
	int ret = 0;
	u8 tmp_buf[128];
	unsigned int bytelen = 0;
	if (datalen > 125) {
		print_info( "%s too big datalen = %d!\n",
							__func__, datalen);
		return -EPERM;
	}

	tmp_buf[0] = addr;
	bytelen++;

	if (datalen != 0 && pdata != NULL) {
		memcpy(&tmp_buf[bytelen], pdata, datalen);
		bytelen += datalen;
	}

	ret = i2c_master_send(client, tmp_buf, bytelen);
	return ret;
}


static int gsl_ts_read(struct i2c_client *client, u8 addr, u8 *pdata,
		       unsigned int datalen)
{
	int ret = 0;

	if (datalen > 126) {
		print_info( "%s too big datalen = %d!\n",
				__func__, datalen);
		return -EPERM;
	}

	ret = gsl_ts_write(client, addr, NULL, 0);
	if (ret < 0) {
		print_info( "%s set data address fail!\n", __func__);
		return ret;
	}

	return i2c_master_recv(client, pdata, datalen);
}
static unsigned int gsl_read_oneframe_data(unsigned int *data,
				unsigned int addr,unsigned int len)
{
	int i = 0;
	u8 reg_a[4]={0x0a,0x00,0x00,0x00};
	u8 buf_a5[4]={0x00,0x00,0x00,0xa5};
	u8 buf_zero[4]={0x00,0x00,0x00,0x00};
	u32 buf32;
	unsigned long end_time;

	printk("tp-gsl-gesture %s\n",__func__);
	printk("gsl_read_oneframe_data:::addr=%x,len=%x\n",addr,len);

	while(1)
	{
		gsl_ts_write(ddata->client,0xf0,&reg_a[0], 4);
		buf_a5[2] = i;
		gsl_ts_write(ddata->client,0x08,&buf_a5[0], 4);
		end_time = jiffies + msecs_to_jiffies(100);
		while(1)
		{
			gsl_ts_read(ddata->client,0xbc,(u8 *)&buf32,4);
			if((buf32 & 0xffff) == 0xa500+i)
			{
				int k;
				for(k=0;k<15;k++)
				{
					gsl_ts_read(ddata->client,0x80+k*4,(u8 *)&buf32,4);
					data[i] = buf32;
					if(++i >= len)
					{
						gsl_ts_write(ddata->client,0xf0,&reg_a[0], 4);
						gsl_ts_write(ddata->client,0x08,&buf_zero[0], 4);
						return 1;
					}
				}
				break;
			}
			if (time_before(jiffies, end_time))
				continue;
			gsl_ts_write(ddata->client,0xf0,&reg_a[0], 4);
			gsl_ts_write(ddata->client,0x08,&buf_zero[0], 4);
			return 0;
		}
	}
	return 1;
}
#endif

static void gsl_load_fw(struct i2c_client *client,const struct fw_data *GSL_DOWNLOAD_DATA,int data_len)
{
	u8 buf[4] = {0};
	//u8 send_flag = 1;
	u8 addr=0;
	u32 source_line = 0;
	u32 source_len = data_len;//ARRAY_SIZE(GSL_DOWNLOAD_DATA);
	unsigned long start_time_jf;

	print_info("=============gsl_load_fw start==============\n");
	start_time_jf = jiffies;
	for (source_line = 0; source_line < source_len; source_line++) 
	{
		/* init page trans, set the page val */
		addr = (u8)GSL_DOWNLOAD_DATA[source_line].offset;
		memcpy(buf,&GSL_DOWNLOAD_DATA[source_line].val,4);
		gsl_write_interface(client, addr, buf, 4);	
	}
	dev_info(&client->dev, "gsl_load_fw: took %u ms\n",
		jiffies_to_msecs(jiffies - start_time_jf));
	print_info("=============gsl_load_fw end==============\n");
}

static void gsl_io_control(struct i2c_client *client)
{
#if GSL9XX_VDDIO_1800
	u8 buf[4] = {0};
	int i;
	for(i=0;i<2;i++){
		buf[0] = 0;
		buf[1] = 0;
		buf[2] = 0xfe;
		buf[3] = 0x1;
		gsl_write_interface(client,0xf0,buf,4);
		buf[0] = 0x5;
		buf[1] = 0;
		buf[2] = 0;
		buf[3] = 0x80;
		gsl_write_interface(client,0x78,buf,4);
		mdelay(5);
	}
	//msleep(50);
#endif
}

static void startup_timer_callback(unsigned long data)
{
	timer_expired = true;
	del_timer(&startup_timer);
}

static void gsl_start_core(struct i2c_client *client)
{
	//u8 tmp = 0x00;
	u8 buf[4] = {0};
	buf[0]=0;
	gsl_write_interface(client,0xe0,buf,4);
#ifdef GSL_ALG_ID
	{
	gsl_DataInit(gsl_cfg_table[gsl_cfg_index].data_id);
	}
#endif	
	if (!timer_pending(&startup_timer)) {
		timer_expired = false;
		setup_timer(&startup_timer, startup_timer_callback, 0);
		mod_timer(&startup_timer, jiffies + msecs_to_jiffies(100));
	}
}

static void gsl_reset_core(struct i2c_client *client)
{
	u8 buf[4] = {0x00};
	
	gpio_set_value(GSL_RST_GPIO_NUM,0);
	mdelay(2);
	gpio_set_value(GSL_RST_GPIO_NUM,1);
	mdelay(2);

	buf[0] = 0x04;
	gsl_write_interface(client,0xe4,buf,4);
	mdelay(2);
	
	buf[0] = 0;
	gsl_write_interface(client,0xbc,buf,4);
	mdelay(2);

	gsl_io_control(client);
}

#ifdef GSL_GESTURE
static void gsl_reset_core_without_vddio(struct i2c_client *client)
{
	u8 buf[4] = {0x00};
	
	gpio_set_value(GSL_RST_GPIO_NUM,0);
	mdelay(2);
	gpio_set_value(GSL_RST_GPIO_NUM,1);

	mdelay(2);

	buf[0] = 0x04;
	gsl_write_interface(client,0xe4,buf,4);
	mdelay(2);
	
	buf[0] = 0;
	gsl_write_interface(client,0xbc,buf,4);
	mdelay(2);
}
#endif

static void gsl_clear_reg(struct i2c_client *client)
{
	u8 buf[4]={0};

	gpio_set_value(GSL_RST_GPIO_NUM,0);
	mdelay(2);
	gpio_set_value(GSL_RST_GPIO_NUM,1);

	mdelay(10);
	buf[0]=0x3;
	gsl_write_interface(client,0x80,buf,4);
	mdelay(2);
	buf[0]=0x4;
	gsl_write_interface(client,0xe4,buf,4);
	mdelay(2);
	buf[0]=0x0;
	gsl_write_interface(client,0xe0,buf,4);
	mdelay(10);
	//clear reg

}

#ifdef GSL_TEST_TP
void gsl_I2C_ROnePage(unsigned int addr, char *buf)
{
	u8 tmp_buf[4]={0};
	tmp_buf[3]=(u8)(addr>>24);
	tmp_buf[2]=(u8)(addr>>16);
	tmp_buf[1]=(u8)(addr>>8);
	tmp_buf[0]=(u8)(addr);
	#if 0
	gsl_ts_write(ddata->client,0xf0,tmp_buf,4);
	gsl_ts_read(ddata->client,0,buf,128);
	#else
	gsl_write_interface(ddata->client,0xf0,tmp_buf,4);
	gsl_ito_read_interface(ddata->client,0,buf,128);
	#endif
}
EXPORT_SYMBOL(gsl_I2C_ROnePage);
void gsl_I2C_RTotal_Address(unsigned int addr,unsigned int *data)
{
	u8 tmp_buf[4]={0};	
	tmp_buf[3]=(u8)((addr/0x80)>>24);
	tmp_buf[2]=(u8)((addr/0x80)>>16);
	tmp_buf[1]=(u8)((addr/0x80)>>8);
	tmp_buf[0]=(u8)((addr/0x80));
	#if 0
	gsl_ts_write(ddata->client,0xf0,tmp_buf,4);
	gsl_ts_read(ddata->client,addr%0x80,tmp_buf,4);
	#else
	gsl_write_interface(ddata->client,0xf0,tmp_buf,4);
	gsl_ito_read_interface(ddata->client,addr%0x80,tmp_buf,4);
	#endif
	*data = tmp_buf[0]|(tmp_buf[1]<<8)|(tmp_buf[2]<<16)|(tmp_buf[3]<<24);
}
EXPORT_SYMBOL(gsl_I2C_RTotal_Address);
#endif

#ifdef TPD_PROC_DEBUG
#define GSL_APPLICATION
#ifdef GSL_APPLICATION
static void gsl_read_MorePage(struct i2c_client *client,u32 addr,u8 *buf,u32 num)
{
	int i;
	u8 tmp_buf[4] = {0};
	u8 tmp_addr;
	for(i=0;i<num/8;i++){
		tmp_buf[0]=(char)((addr+i*8)/0x80);
		tmp_buf[1]=(char)(((addr+i*8)/0x80)>>8);
		tmp_buf[2]=(char)(((addr+i*8)/0x80)>>16);
		tmp_buf[3]=(char)(((addr+i*8)/0x80)>>24);
		gsl_write_interface(client,0xf0,tmp_buf,4);
		tmp_addr = (char)((addr+i*8)%0x80);
		gsl_ito_read_interface(client,tmp_addr,(buf+i*8),8);
	}
	if(i*8<num){
		tmp_buf[0]=(char)((addr+i*8)/0x80);
		tmp_buf[1]=(char)(((addr+i*8)/0x80)>>8);
		tmp_buf[2]=(char)(((addr+i*8)/0x80)>>16);
		tmp_buf[3]=(char)(((addr+i*8)/0x80)>>24);
		gsl_write_interface(client,0xf0,tmp_buf,4);
		tmp_addr = (char)((addr+i*8)%0x80);
		gsl_ito_read_interface(client,tmp_addr,(buf+i*8),4);
	}
}
#endif
static int char_to_int(char ch)
{
	if(ch>='0' && ch<='9')
		return (ch-'0');
	else
		return (ch-'a'+10);
}

//static int gsl_config_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
static int gsl_config_read_proc(struct seq_file *m,void *v)
{
	char temp_data[5] = {0};
	//int i;
	unsigned int tmp=0;
	if('v'==gsl_read[0]&&'s'==gsl_read[1])
	{
#ifdef GSL_ALG_ID
		tmp=gsl_version_id();
#else 
		tmp=0x20121215;
#endif
		seq_printf(m,"version:%x\n",tmp);
	}
	else if('r'==gsl_read[0]&&'e'==gsl_read[1])
	{
		if('i'==gsl_read[3])
		{
#ifdef GSL_ALG_ID 
			tmp=(gsl_data_proc[5]<<8) | gsl_data_proc[4];
			seq_printf(m,"gsl_config_data_id[%d] = ",tmp);
			if(tmp>=0&&tmp<gsl_cfg_table[gsl_cfg_index].data_size)
				seq_printf(m,"%d\n",gsl_cfg_table[gsl_cfg_index].data_id[tmp]); 
#endif
		}
		else 
		{
			gsl_write_interface(ddata->client,0xf0,&gsl_data_proc[4],4);
			gsl_ito_read_interface(ddata->client,gsl_data_proc[0],temp_data,4);
			seq_printf(m,"offset : {0x%02x,0x",gsl_data_proc[0]);
			seq_printf(m,"%02x",temp_data[3]);
			seq_printf(m,"%02x",temp_data[2]);
			seq_printf(m,"%02x",temp_data[1]);
			seq_printf(m,"%02x};\n",temp_data[0]);
		}
	}
#ifdef GSL_APPLICATION
	else if('a'==gsl_read[0]&&'p'==gsl_read[1]){
		char *buf;
		int temp1;
		tmp = (unsigned int)(((gsl_data_proc[2]<<8)|gsl_data_proc[1])&0xffff);
		buf=kzalloc(tmp,GFP_KERNEL);
		if(buf==NULL)
			return -1;
		if(3==gsl_data_proc[0]){
			gsl_read_interface(ddata->client,gsl_data_proc[3],buf,tmp);
			if(tmp < m->size){
				memcpy(m->buf,buf,tmp);
			}
		}else if(4==gsl_data_proc[0]){
			temp1=((gsl_data_proc[6]<<24)|(gsl_data_proc[5]<<16)|
				(gsl_data_proc[4]<<8)|gsl_data_proc[3]);
			gsl_read_MorePage(ddata->client,temp1,buf,tmp);
			if(tmp < m->size){
				memcpy(m->buf,buf,tmp);
			}
		}
		kfree(buf);
	}
#endif
	return 0;
}
//ssize_t (*write) (struct file *, const char __user *, size_t, loff_t *);
static ssize_t gsl_config_write_proc(struct file *file, const char __user *buffer, size_t count, loff_t *data)
{
	u8 buf[8] = {0};
	char temp_buf[CONFIG_LEN];
	char *path_buf;
	int tmp = 0;
	int tmp1 = 0;
	print_info("[tp-gsl][%s] \n",__func__);
	if(count > 512)
	{
		print_info("size not match [%d:%zu]\n", CONFIG_LEN, count);
        	return -EFAULT;
	}
	path_buf=kzalloc(count,GFP_KERNEL);
	if(!path_buf)
	{
		print_info("alloc path_buf memory error \n");
		return -1;
	}	
	if(copy_from_user(path_buf, buffer, count))
	{
		print_info("copy from user fail\n");
		goto exit_write_proc_out;
	}
	memcpy(temp_buf,path_buf,(count<CONFIG_LEN?count:CONFIG_LEN));
	print_info("[tp-gsl][%s][%s]\n",__func__,temp_buf);
#ifdef GSL_APPLICATION
	if('a'!=temp_buf[0]||'p'!=temp_buf[1]){
#endif
	buf[3]=char_to_int(temp_buf[14])<<4 | char_to_int(temp_buf[15]);	
	buf[2]=char_to_int(temp_buf[16])<<4 | char_to_int(temp_buf[17]);
	buf[1]=char_to_int(temp_buf[18])<<4 | char_to_int(temp_buf[19]);
	buf[0]=char_to_int(temp_buf[20])<<4 | char_to_int(temp_buf[21]);
	
	buf[7]=char_to_int(temp_buf[5])<<4 | char_to_int(temp_buf[6]);
	buf[6]=char_to_int(temp_buf[7])<<4 | char_to_int(temp_buf[8]);
	buf[5]=char_to_int(temp_buf[9])<<4 | char_to_int(temp_buf[10]);
	buf[4]=char_to_int(temp_buf[11])<<4 | char_to_int(temp_buf[12]);
#ifdef GSL_APPLICATION
	}
#endif
	if('v'==temp_buf[0]&& 's'==temp_buf[1])//version //vs
	{
		memcpy(gsl_read,temp_buf,4);
		print_info("gsl version\n");
	}
	else if('s'==temp_buf[0]&& 't'==temp_buf[1])//start //st
	{
	#ifdef GSL_TIMER	
		cancel_delayed_work_sync(&gsl_timer_check_work);
	#endif
		gsl_proc_flag = 1;
		gsl_reset_core(ddata->client);
	}
	else if('e'==temp_buf[0]&&'n'==temp_buf[1])//end //en
	{
		msleep(20);
		gsl_reset_core(ddata->client);
		gsl_start_core(ddata->client);
		gsl_proc_flag = 0;
	}
	else if('r'==temp_buf[0]&&'e'==temp_buf[1])//read buf //
	{
		memcpy(gsl_read,temp_buf,4);
		memcpy(gsl_data_proc,buf,8);
	}
	else if('w'==temp_buf[0]&&'r'==temp_buf[1])//write buf
	{
		gsl_write_interface(ddata->client,buf[4],buf,4);
	}
	
#ifdef GSL_ALG_ID
	else if('i'==temp_buf[0]&&'d'==temp_buf[1])//write id config //
	{
		tmp1=(buf[7]<<24)|(buf[6]<<16)|(buf[5]<<8)|buf[4];
		tmp=(buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];
		if(tmp1>=0 && tmp1<gsl_cfg_table[gsl_cfg_index].data_size)
		{
			gsl_cfg_table[gsl_cfg_index].data_id[tmp1] = tmp;
		}
	}
#endif
#ifdef GSL_APPLICATION
	else if('a'==temp_buf[0]&&'p'==temp_buf[1]){
		if(1==path_buf[3]){
			tmp=((path_buf[5]<<8)|path_buf[4]);
			gsl_write_interface(ddata->client,path_buf[6],&path_buf[10],tmp);
		}else if(2==path_buf[3]){
			tmp = ((path_buf[5]<<8)|path_buf[4]);
			tmp1=((path_buf[9]<<24)|(path_buf[8]<<16)|(path_buf[7]<<8)
				|path_buf[6]);
			buf[0]=(char)((tmp1/0x80)&0xff);
			buf[1]=(char)(((tmp1/0x80)>>8)&0xff);
			buf[2]=(char)(((tmp1/0x80)>>16)&0xff);
			buf[3]=(char)(((tmp1/0x80)>>24)&0xff);
			buf[4]=(char)(tmp1%0x80);
			gsl_write_interface(ddata->client,0xf0,buf,4);
			gsl_write_interface(ddata->client,buf[4],&path_buf[10],tmp);
		}else if(3==path_buf[3]||4==path_buf[3]){
			memcpy(gsl_read,temp_buf,4);
			memcpy(gsl_data_proc,&path_buf[3],7);
		}
	}
#endif
exit_write_proc_out:
	kfree(path_buf);
	return count;
}
static int gsl_server_list_open(struct inode *inode,struct file *file)
{
	return single_open(file,gsl_config_read_proc,NULL);
}
static const struct file_operations gsl_seq_fops = {
	.open = gsl_server_list_open,
	.read = seq_read,
	.release = single_release,
	.write = gsl_config_write_proc,   
	.owner = THIS_MODULE,
};
#endif

#ifdef GSL_TIMER
static void gsl_watchdog(struct work_struct *work)
{
	struct gsl_ts_data *ts = ddata;
	struct i2c_client *gsl_client = ts->client;
	u32 tmp;
	int rc;

	if(!mutex_trylock(&ddata->hw_lock))
		goto queue;

	rc = gsl_read32(gsl_client, 0xb0, &tmp);
	if (rc) {
		dev_err(&gsl_client->dev,
			"watchdog: error reading reg#b0: %d\n", rc);
		goto exit_reinit;
	}

	if (tmp != 0x5a5a5a5a) {
		dev_err(&gsl_client->dev, "watchdog: invalid reg#b0 value = %#08x\n",
			tmp);
		goto exit_reinit;
	}

	rc = gsl_read32(gsl_client, 0xb4, &tmp);
	if (rc) {
		dev_err(&gsl_client->dev,
			"watchdog: error reading reg#b4: %d\n", rc);
		goto exit_reinit;
	}

	if (tmp == ddata->watchdog_counter) {
		dev_err(&gsl_client->dev, "watchdog: hang detected. counter stuck at %#08x\n",
			tmp);
		goto exit_reinit;
	}
	ddata->watchdog_counter = tmp;

	/* version 1.4.0 or later than 1.4.0 read 0xbc for esd checking */
	rc = gsl_read32(gsl_client, 0xbc, &tmp);
	if (rc) {
		dev_err(&gsl_client->dev,
			"watchdog: error reading reg#bc: %d\n", rc);
		goto exit_reinit;
	}

	if (tmp != 0) {
		dev_err(&gsl_client->dev, "watchdog: unexpected reg#bc value = %#08x\n",
			tmp);
		goto exit_reinit;
	}

	goto exit;

exit_reinit:
	dev_warn(&gsl_client->dev, "watchdog: full re-init needed\n");
	gsl_sw_init(gsl_client);
exit:
	mutex_unlock(&ddata->hw_lock);

queue:
	queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work,
		GSL_TIMER_WATCHDOG_INTERVAL);
}
#endif



static int gsl_compatible_id(struct i2c_client *client)
{
	u8 buf[4];
	int i,err;
	for(i=0;i<5;i++)
	{
		err = gsl_read_interface(client,0xfc,buf,4);
		print_info("[tp-gsl] 0xfc = {0x%02x%02x%02x%02x}\n",buf[3],buf[2],
			buf[1],buf[0]);
		if(!(err<0))
		{
			err = 1;		
			break;	
		}
	}
	return err;	
}

struct regulator *vcc_ana;
struct regulator *vcc_dig;
struct regulator *vcc_i2c;

static int gsl_regulator_configure(struct i2c_client *client, bool on)
{
	int rc;

	
	if (on == false)
		goto hw_shutdown;

	   vcc_ana = regulator_get(&client->dev, "vdd");
	if (IS_ERR(vcc_ana)) {
		rc = PTR_ERR(vcc_ana);
		dev_err(&client->dev,
			"Regulator get failed vcc_ana rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(vcc_ana) > 0) {
		rc = regulator_set_voltage(vcc_ana, 2700000,
							3300000);
		if (rc) {
			dev_err(&client->dev,
				"regulator set_vtg failed rc=%d\n", rc);
			goto error_set_vtg_vcc_ana;
		}
	}

	
	    vcc_i2c = regulator_get(&client->dev, "vcc_i2c");
		if (IS_ERR(vcc_i2c)) {
			rc = PTR_ERR(vcc_i2c);
			dev_err(&client->dev,
				"Regulator get failed rc=%d\n",	rc);
			goto error_get_vtg_i2c;
		}
		if (regulator_count_voltages(vcc_i2c) > 0) {
			rc = regulator_set_voltage(vcc_i2c,
				1800000, 1800000);
			if (rc) {
				dev_err(&client->dev,
					"regulator set_vtg failed rc=%d\n", rc);
				goto error_set_vtg_i2c;
			}
		}
	

	return 0;

error_set_vtg_i2c:
	regulator_put(vcc_i2c);
error_get_vtg_i2c:
if (regulator_count_voltages(vcc_ana) > 0)
		regulator_set_voltage(vcc_ana, 0, 3300000);
error_set_vtg_vcc_ana:
	regulator_put(vcc_ana);
	return rc;

hw_shutdown:
	if (regulator_count_voltages(vcc_ana) > 0)
		regulator_set_voltage(vcc_ana, 0, 3300000);
	regulator_put(vcc_ana);

	if (regulator_count_voltages(vcc_i2c) > 0)
		regulator_set_voltage(vcc_i2c, 0,
						1800000);
	regulator_put(vcc_i2c);

	return 0;
}

static int gsl_power_on(struct i2c_client *client, bool on)
{
	int rc;

	if (on == false)
		goto power_off;

	rc = regulator_enable(vcc_ana);
	if (rc) {
		dev_err(&client->dev,
			"Regulator vcc_ana enable failed rc=%d\n", rc);
		goto error_reg_en_vcc_ana;
	}

		rc = regulator_enable(vcc_i2c);
		if (rc) {
			dev_err(&client->dev,
				"Regulator vcc_i2c enable failed rc=%d\n", rc);
			goto error_reg_en_vcc_i2c;
		}
	

	msleep(130);

	return 0;

error_reg_en_vcc_i2c:

	regulator_disable(vcc_ana);
error_reg_en_vcc_ana:
	return rc;

power_off:
	regulator_disable(vcc_ana);
	regulator_disable(vcc_i2c);
	
	msleep(50);
	return 0;
}

static void gsl_hw_init(void)
{
	//add power
	//GSL_POWER_ON();	
	//
       gsl_regulator_configure(ddata->client, true);
       gsl_power_on(ddata->client, true);

	
	gpio_request(GSL_IRQ_GPIO_NUM,GSL_IRQ_NAME);
	gpio_request(GSL_RST_GPIO_NUM,GSL_RST_NAME);
	gpio_direction_output(GSL_RST_GPIO_NUM,1);	
	gpio_direction_input(GSL_IRQ_GPIO_NUM);

	msleep(5);
	gpio_set_value(GSL_RST_GPIO_NUM,0);
	msleep(10);	
	gpio_set_value(GSL_RST_GPIO_NUM,1);
	msleep(20);
	//	
}
static void gsl_sw_init(struct i2c_client *client)
{
	//int temp;
	//struct fw_data *fw = GSLx68x_FW;
	if(1==ddata->gsl_sw_flag)
		return;

	dev_dbg(&client->dev, "begin sw init\n");
	ddata->gsl_sw_flag = 1;
	
	gpio_set_value(GSL_RST_GPIO_NUM, 0);
	mdelay(10);
	gpio_set_value(GSL_RST_GPIO_NUM, 1);
	mdelay(10);	

	gsl_clear_reg(client);
	gsl_reset_core(client);
	{
	gsl_load_fw(client,gsl_cfg_table[gsl_cfg_index].fw,
		gsl_cfg_table[gsl_cfg_index].fw_size);
	}
	gsl_start_core(client);

	ddata->gsl_sw_flag = 0;
}

static void check_mem_data(struct i2c_client *client)
{
#if 0
	u8 read_buf[4]  = {0};
	mdelay(30);
	gsl_read_interface(client,0xb0,read_buf,4);
	if (read_buf[3] != 0x5a || read_buf[2] != 0x5a 
		|| read_buf[1] != 0x5a || read_buf[0] != 0x5a)
	{
		print_info("0xb4 ={0x%02x%02x%02x%02x}\n",
			read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		gsl_sw_init(client);
	}
#else
	u8 i = 0;
	u8 read_buf[4]  = {0};
	for(; i<6; i++)
	{
		gsl_read_interface(client,0xb0,read_buf,4);
			if (read_buf[3] != 0x5a || read_buf[2] != 0x5a 
		|| read_buf[1] != 0x5a || read_buf[0] != 0x5a)
		{
			mdelay(5);
		}
		else
		{
			break;
		}
	}
	if (read_buf[3] != 0x5a || read_buf[2] != 0x5a 
		|| read_buf[1] != 0x5a || read_buf[0] != 0x5a)
	{
		print_info("0xb4 ={0x%02x%02x%02x%02x}\n",
			read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		dev_warn(&client->dev, "fw corruption detected, reloading\n");
		gsl_sw_init(client);
	}
#endif	
}

#define GSL_CHIP_NAME	"gslx68x"
static ssize_t gsl_sysfs_version_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	//ssize_t len=0;
	int count = 0;
	u32 tmp;
	u8 buf_tmp[4];
	//char *ptr = buf;
	count += scnprintf(buf,PAGE_SIZE,"sileadinc:");
	count += scnprintf(buf+count,PAGE_SIZE-count,GSL_CHIP_NAME);

#ifdef GSL_TIMER
	count += scnprintf(buf+count,PAGE_SIZE-count,":0001-1:");
#else
	count += scnprintf(buf+count,PAGE_SIZE-count,":0001-0:");
#endif

#ifdef TPD_PROC_DEBUG
	count += scnprintf(buf+count,PAGE_SIZE-count,"0002-1:");
#else
	count += scnprintf(buf+count,PAGE_SIZE-count,"0002-0:");
#endif

#ifdef GSL_PROXIMITY_SENSOR//TPD_PROXIMITY
	count += scnprintf(buf+count,PAGE_SIZE-count,"0003-1:");
#else
	count += scnprintf(buf+count,PAGE_SIZE-count,"0003-0:");
#endif

#ifdef GSL_DEBUG
	count += scnprintf(buf+count,PAGE_SIZE-count,"0004-1:");
#else
	count += scnprintf(buf+count,PAGE_SIZE-count,"0004-0:");
#endif

#ifdef GSL_ALG_ID
	tmp = gsl_version_id();
	count += scnprintf(buf+count,PAGE_SIZE-count,"%08x:",tmp);
	count += scnprintf(buf+count,PAGE_SIZE-count,"%08x:",
		gsl_cfg_table[gsl_cfg_index].data_id[0]);
#endif
	buf_tmp[0]=0x3;buf_tmp[1]=0;buf_tmp[2]=0;buf_tmp[3]=0;
	gsl_write_interface(ddata->client,0xf0,buf_tmp,4);
	gsl_read_interface(ddata->client,4,buf_tmp,4);
	count += scnprintf(buf+count,PAGE_SIZE-count,"%02x%02x%02x%02x\n",
		buf_tmp[3],buf_tmp[2],buf_tmp[1],buf_tmp[0]);

    return count;
}

static DEVICE_ATTR(version, 0444, gsl_sysfs_version_show, NULL);

#ifdef GSL_PROXIMITY_SENSOR
char flag_tp_down = 0;
static struct wake_lock ps_wake_lock;
char ps_data_state[1] = {1};
static int proximity_enable = 0;
static u8 gsl_psensor_data[8]={0};
enum
{
       DISABLE_CTP_PS = 0,
       ENABLE_CTP_PS = 1,
       RESET_CTP_PS
};

static void gsl_gain_psensor_data(struct i2c_client *client)
{
	int tmp = 0;
	u8 buf[4]={0};
	/**************************/
	buf[0]=0x3;
	gsl_write_interface(client,0xf0,buf,4);
	tmp = gsl_write_interface(client,0x0,&gsl_psensor_data[0],4);
	if(tmp <= 0)
	{
		 gsl_write_interface(client,0x0,&gsl_psensor_data[0],4);
	}
	/**************************/

	buf[0]=0x4;
	gsl_write_interface(client,0xf0,buf,4);
	tmp = gsl_write_interface(client,0x0,&gsl_psensor_data[4],4);
	if(tmp <= 0)
	{
		gsl_write_interface(client,0x0,&gsl_psensor_data[4],4);
	}

	
}

static ssize_t show_proximity_sensor_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	print_info("%s : GSL_proximity_sensor_status = %d\n",__func__,ps_data_state[0]);
	return sprintf(buf, "0x%02x   \n",ps_data_state[0]);
}

static ssize_t show_proximity_sensor_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	print_info("%s : GSL_proximity_enable = %d\n",__func__,proximity_enable);
	return sprintf(buf, "0x%02x   \n",proximity_enable);
}
static ssize_t store_proximity_sensor_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	//int rc;
	int val;
	u8 ps_store_data[4];
	if(buf != NULL && size != 0)
	{
		sscanf(buf,"%d",&val);
		print_info("%s : val =%d\n",__func__,val);
		if(DISABLE_CTP_PS == val)
		{
			proximity_enable = 0;
			print_info("DISABLE_CTP_PS buf=%d,size=%d,val=%d\n", *buf, size,val);

			/* Writing 0x00 put into {0xf0,0x03} */
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x03;
			gsl_write_interface(ddata->client,0xf0,ps_store_data,4);
			ps_store_data[3] = gsl_psensor_data[3];
			ps_store_data[2] = gsl_psensor_data[2];
			ps_store_data[1] = gsl_psensor_data[1];
			ps_store_data[0] = gsl_psensor_data[0];
			gsl_write_interface(ddata->client,0,ps_store_data,4);

			/* Writing 0x00 put into {0xf0,0x04} */
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x04;
			gsl_write_interface(ddata->client,0xf0,ps_store_data,4);
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x00;
			gsl_write_interface(ddata->client,0,ps_store_data,4);
			
			//if (rc < 0)
			//{
			//	print_info("%s :write err val =0 ++++++++++++++++\n",__func__);
			//}
			msleep(200);
			print_info("RESET_CTP_PS buf=%d\n", *buf);
			wake_lock(&ps_wake_lock);
		}

		else if(ENABLE_CTP_PS == val)
		{
			wake_lock(&ps_wake_lock);
			proximity_enable = 1;
			print_info("ENABLE_CTP_PS buf=%d,size=%d,val=%d\n", *buf, size,val);

			/* Writing 0x5a5a5a5a put into {0xf0,0x03}*/
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x03;
			gsl_write_interface(ddata->client,0xf0,ps_store_data,4);
			ps_store_data[3] = 0x5a;
			ps_store_data[2] = 0x5a;
			ps_store_data[1] = 0x5a;
			ps_store_data[0] = 0x5a;
			gsl_write_interface(ddata->client,0x00,ps_store_data,4);

			print_info("[GSL] The value of {0x03,0x00} is = %x %x %x %x\n",buf[3],buf[2],buf[1],buf[0]);	

			/* Writing 0x02 put into {0xf0,0x03} */
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x04;
			gsl_write_interface(ddata->client,0xf0,ps_store_data,4);
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x02;
			gsl_write_interface(ddata->client,0x00,ps_store_data,4);

			print_info("[GSL] The value of {0x03,0x00} is = %x %x %x %x\n",buf[3],buf[2],buf[1],buf[0]);
			
			//if (rc < 0)
			//{
			//	print_info("%s :write err++++++++++++++++\n",__func__);
			//}
		}
	}

	return size;
}

static int psensor_ps_set_enable(struct sensors_classdev *sensors_cdev, unsigned int enable)
{
	//int rc;
	u8 ps_store_data[4]={0};
	int i=0;
	
		if(DISABLE_CTP_PS == enable)
		{
			i=0;
			proximity_enable = 0;
			ps_data_state[0] = 1;	
	
			/* Writing 0x00 put into {0xf0,0x03} */
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x03;
			gsl_write_interface(ddata->client,0xf0,ps_store_data,4);
			ps_store_data[3] = gsl_psensor_data[3];
			ps_store_data[2] = gsl_psensor_data[2];
			ps_store_data[1] = gsl_psensor_data[1];
			ps_store_data[0] = gsl_psensor_data[0];
			gsl_write_interface(ddata->client,0,ps_store_data,4);

			/* Writing 0x00 put into {0xf0,0x04} */
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x04;
			gsl_write_interface(ddata->client,0xf0,ps_store_data,4);
			ps_store_data[3] = gsl_psensor_data[7];
			ps_store_data[2] = gsl_psensor_data[6];
			ps_store_data[1] = gsl_psensor_data[5];
			ps_store_data[0] = gsl_psensor_data[4];
			gsl_write_interface(ddata->client,0,ps_store_data,4);
			
			msleep(200);
			wake_unlock(&ps_wake_lock);
		}

		else if(ENABLE_CTP_PS == enable)
		{
			i=0;
			wake_lock(&ps_wake_lock);
			proximity_enable = 1;

			ps_data_state[0] = 1;
	
	
			/* Writing 0x5a5a5a5a put into {0xf0,0x03}*/
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x03;
			gsl_write_interface(ddata->client,0xf0,ps_store_data,4);
			ps_store_data[3] = 0x5a;
			ps_store_data[2] = 0x5a;
			ps_store_data[1] = 0x5a;
			ps_store_data[0] = 0x5a;
			gsl_write_interface(ddata->client,0x00,ps_store_data,4);

			print_info("[%s] The value of {0x03,0x00} is = %x %x %x %x\n",__func__,
				ps_store_data[3],ps_store_data[2],ps_store_data[1],ps_store_data[0]);	

			/* Writing 0x02 put into {0xf0,0x03} */
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x04;
			gsl_write_interface(ddata->client,0xf0,ps_store_data,4);
			ps_store_data[3] = 0x00;
			ps_store_data[2] = 0x00;
			ps_store_data[1] = 0x00;
			ps_store_data[0] = 0x02;
			gsl_write_interface(ddata->client,0x00,ps_store_data,4);
			

			print_info("[%s] The value of {0x04,0x00} is = %x %x %x %x\n",__func__,
				ps_store_data[3],ps_store_data[2],ps_store_data[1],ps_store_data[0]);	
			
		}
		ps_data_state[0] = 1; 
	return 0;
}

static DEVICE_ATTR(proximity_sensor_enable, 0777, show_proximity_sensor_enable, store_proximity_sensor_enable);
static DEVICE_ATTR(proximity_sensor_status, 0777, show_proximity_sensor_status,NULL);

#endif


#ifdef GSL_GESTURE
static void gsl_enter_doze(struct gsl_ts_data *ts, bool bCharacterGesture)
{
	u8 buf[4] = {0};
#if 0
	u32 tmp;
	gsl_reset_core(ts->client);
	temp = ARRAY_SIZE(GSLX68X_FW_GESTURE);
	gsl_load_fw(ts->client,GSLX68X_FW_GESTURE,temp);
	gsl_start_core(ts->client);
	msleep(1000);		
#endif
	WARN_ON(dozing);
	dozing = true;

	buf[0] = 0xa;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;
	gsl_write_interface(ts->client,0xf0,buf,4);
	buf[0] = 0;
	buf[1] = 0;
	buf[2] = 0x3;
	buf[3] = 0x5a;
	if(bCharacterGesture == true)
	{
		buf[2] = 0x1;	
	}
	gsl_write_interface(ts->client,0x8,buf,4);
	//gsl_gesture_status = GE_NOWORK;
	gsl_gesture_status = GE_ENABLE;

	dev_dbg(&ts->client->dev, "entering doze mode (gesture mode:%d)\n",
		(int) bCharacterGesture);
}
static void gsl_quit_doze(struct gsl_ts_data *ts)
{
	u8 buf[4] = {0};
	//u32 tmp;

	WARN_ON(!dozing);

	/* disable the IRQ while we go reset the peripheral */
	disable_irq(ts->client->irq);

	gsl_gesture_status = GE_DISABLE;

	gpio_direction_output(GSL_IRQ_GPIO_NUM,0);
	gpio_set_value(GSL_RST_GPIO_NUM,0);
	mdelay(5);
	gpio_set_value(GSL_RST_GPIO_NUM,1);
	mdelay(20);
	gpio_direction_input(GSL_IRQ_GPIO_NUM);
	
	buf[0] = 0xa;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0;
	gsl_write_interface(ts->client,0xf0,buf,4);
	buf[0] = 0;
	buf[1] = 0;
	buf[2] = 0;
	buf[3] = 0x5a;
	gsl_write_interface(ts->client,0x8,buf,4);
	mdelay(5);

#if 0
	gsl_reset_core(ddata->client);
	temp = ARRAY_SIZE(GSLX68X_FW_CONFIG);
	//gsl_load_fw();
	gsl_load_fw(ddata->client,GSLX68X_FW_CONFIG,temp);
	gsl_start_core(ddata->client);
#endif
	dozing = false;

	/* all done, re-enable IRQ */
	enable_irq(ts->client->irq);

	dev_dbg(&ts->client->dev, "exiting doze mode\n");
}

static ssize_t gsl_sysfs_tpgesture_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", gsl_gesture_flag);
}
static ssize_t gsl_sysfs_tpgesturet_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	mutex_lock(&ddata->hw_lock);
	if(buf[0] == '0'){
		gsl_gesture_flag = 0;  
	}else if(buf[0] == '1'){
		gsl_gesture_flag = 1;
	}else if(buf[0] == '2'){
	//enable character gesture
		gsl_gesture_flag = 2;
	}
	mutex_unlock(&ddata->hw_lock);

	return count;
}
static DEVICE_ATTR(gesture, 0664, gsl_sysfs_tpgesture_show, gsl_sysfs_tpgesturet_store);
#endif

static struct attribute *gsl_attrs[] = {

	&dev_attr_version.attr,
		
#ifdef GSL_GESTURE
	&dev_attr_gesture.attr,
#endif

#ifdef GSL_PROXIMITY_SENSOR
	&dev_attr_proximity_sensor_enable.attr,
	&dev_attr_proximity_sensor_status.attr,
#endif

	NULL
};
static const struct attribute_group gsl_attr_group = {
	.attrs = gsl_attrs,
};

#if GSL_HAVE_TOUCH_KEY
static int gsl_report_key(struct input_dev *idev,int x,int y)
{
	int i;
	for(i=0;i<GSL_KEY_NUM;i++)
	{
		if(x > gsl_key_data[i].x_min &&
			x < gsl_key_data[i].x_max &&
			y > gsl_key_data[i].y_min &&
			y < gsl_key_data[i].y_max)
		{
			ddata->gsl_key_state = i+1;
			input_report_key(idev,gsl_key_data[i].key,1);
			input_sync(idev);
			return 1;
		}
	}
	return 0;
}
#endif
static void gsl_report_point(struct input_dev *idev, struct gsl_touch_info *cinfo)
{
	int i; 
	u32 gsl_point_state = 0;
	u32 temp=0;
	if(cinfo->finger_num>0 && cinfo->finger_num<6)
	{
		ddata->gsl_up_flag = 0;
		gsl_point_state = 0;
	#if GSL_HAVE_TOUCH_KEY
		if(1==cinfo->finger_num)
		{
			if(cinfo->x[0] > GSL_MAX_X || cinfo->y[0] > GSL_MAX_Y)
			{
				gsl_report_key(idev,cinfo->x[0],cinfo->y[0]);
				return;		
			}
		}
	#endif
		for(i=0;i<cinfo->finger_num;i++)
		{
			gsl_point_state |= (0x1<<cinfo->id[i]);	
			//print_info("id = %d, x = %d, y = %d \n",cinfo->id[i], 
			//	cinfo->x[i],cinfo->y[i]);
		#ifdef GSL_REPORT_POINT_SLOT
			input_mt_slot(idev, cinfo->id[i] - 1);
			input_report_abs(idev, ABS_MT_TRACKING_ID, cinfo->id[i]-1);
			input_report_abs(idev, ABS_MT_TOUCH_MAJOR, GSL_PRESSURE);
			input_report_abs(idev, ABS_MT_POSITION_X, cinfo->x[i]);
			input_report_abs(idev, ABS_MT_POSITION_Y, cinfo->y[i]);	
			input_report_abs(idev, ABS_MT_WIDTH_MAJOR, 1);
		
		#else 
			input_report_key(idev, BTN_TOUCH, 1);
			input_report_abs(idev, ABS_MT_POSITION_X, cinfo->x[i]);
			input_report_abs(idev, ABS_MT_POSITION_Y, cinfo->y[i]);	
			input_report_abs(idev, ABS_MT_TOUCH_MAJOR, GSL_PRESSURE);
			input_report_abs(idev, ABS_MT_WIDTH_MAJOR, 1);
			input_report_abs(idev, ABS_MT_TRACKING_ID, cinfo->id[i]-1);
			input_mt_sync(idev);		
		#endif
		}
	}
	else if(cinfo->finger_num == 0)
	{
		gsl_point_state = 0;
	//	ddata->gsl_point_state = 0;
		if(1 == ddata->gsl_up_flag)
		{
			return;
		}
		ddata->gsl_up_flag = 1;

	#ifdef GSL_PROXIMITY_SENSOR
		input_report_abs(ddata->input_dev_ps/*idev*/, ABS_DISTANCE, ps_data_state[0]);
		input_sync(ddata->input_dev_ps);
	#endif
		
	#if GSL_HAVE_TOUCH_KEY
		if(ddata->gsl_key_state > 0)
		{
			if(ddata->gsl_key_state < GSL_KEY_NUM+1)
			{
				input_report_key(idev,gsl_key_data[ddata->gsl_key_state - 1].key,0);
				input_sync(idev);
			}
		}
	#endif
	#ifndef GSL_REPORT_POINT_SLOT
		input_report_key(idev, BTN_TOUCH, 0);
		//input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 0);
		//input_report_abs(idev, ABS_MT_WIDTH_MAJOR, 0);
		input_mt_sync(idev);
	#endif
	}

	temp = gsl_point_state & ddata->gsl_point_state;
	temp = (~temp) & ddata->gsl_point_state;
#ifdef GSL_REPORT_POINT_SLOT
	for(i=1;i<6;i++)
	{
		if(temp & (0x1<<i))
		{
			input_mt_slot(idev, i-1);
			input_report_abs(idev, ABS_MT_TRACKING_ID, -1);
			input_mt_report_slot_state(idev, MT_TOOL_FINGER, false);
		}
	}
#endif	
	ddata->gsl_point_state = gsl_point_state;
	input_sync(idev);
}


static irqreturn_t gsl_ts_isr(int irq, void *priv)
{
	int rc,tmp;
	u8 buf[44] = {0};
	int tmp1=0;
	struct gsl_ts_data *ddata = priv;
	struct gsl_touch_info *cinfo = ddata->cinfo;
	struct i2c_client *client = ddata->client;
	struct input_dev *idev = ddata->idev;
	
	#if defined(GSL_GESTURE)&&defined(GSL_DEBUG)
	unsigned int test_count = 0;
	#endif

	#ifdef GSL_PROXIMITY_SENSOR
	u8 tmp_prox = 0;
	#endif
	
	//if(1 == ddata->gsl_sw_flag)
	//	goto schedule;
    #ifdef GSL_TIMER
	if(!mutex_trylock(&ddata->hw_lock))
		goto i2c_lock_schedule;
    #endif
	
	#ifdef TPD_PROC_DEBUG
		if(gsl_proc_flag == 1){
			goto schedule;
		}
	#endif

	/* Gesture Resume */
	#ifdef GSL_GESTURE
		//print_info("GSL:::0x80=%02x%02x%02x%02x[%d]\n",buf[3],buf[2],buf[1],buf[0],test_count++);
		//print_info("GSL:::0x84=%02x%02x%02x%02x\n",buf[7],buf[6],buf[5],buf[4]);
		//print_info("GSL:::0x88=%02x%02x%02x%02x\n",buf[11],buf[10],buf[9],buf[8]);
		if(GE_ENABLE == gsl_gesture_status && ((gsl_gesture_flag == 1)||(gsl_gesture_flag == 2)))
		{
			if (!timer_expired)
			{
				printk("gsl_report_work: delta time <= 100.\n");
				goto schedule;
			}
		}
	#endif
	
	/* read data from DATA_REG */
	rc = gsl_read_interface(client, 0x80, buf, 44);
	if (rc < 0) 
	{
		goto schedule;
	}

	if (buf[0] == 0xff) {
		goto schedule;
	}

	cinfo->finger_num = buf[0];
	for(tmp=0;tmp<(cinfo->finger_num>10 ? 10:cinfo->finger_num);tmp++)
	{
		cinfo->y[tmp] = (buf[tmp*4+4] | ((buf[tmp*4+5])<<8));
		cinfo->x[tmp] = (buf[tmp*4+6] | ((buf[tmp*4+7] & 0x0f)<<8));
		cinfo->id[tmp] = buf[tmp*4+7] >> 4;
		//print_info("tp-gsl  x = %d y = %d \n",cinfo->x[tmp],cinfo->y[tmp]);
	}
	
	//print_info("111 finger_num= %d\n",cinfo->finger_num);
#ifdef GSL_ALG_ID
	cinfo->finger_num = (buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|(buf[0]);
	gsl_alg_id_main(cinfo);
	tmp1=gsl_mask_tiaoping();
	//print_info("[tp-gsl] tmp1=%x\n",tmp1);
	if(tmp1>0&&tmp1<0xffffffff)
	{
		buf[0]=0xa;
		buf[1]=0;
		buf[2]=0;
		buf[3]=0;
		gsl_write_interface(client,0xf0,buf,4);
		buf[0]=(u8)(tmp1 & 0xff);
		buf[1]=(u8)((tmp1>>8) & 0xff);
		buf[2]=(u8)((tmp1>>16) & 0xff);
		buf[3]=(u8)((tmp1>>24) & 0xff);
		//print_info("tmp1=%08x,buf[0]=%02x,buf[1]=%02x,buf[2]=%02x,buf[3]=%02x\n", tmp1,buf[0],buf[1],buf[2],buf[3]);
		gsl_write_interface(client,0x8,buf,4);
	}
#endif

/* Proximity Sensor */
#ifdef GSL_PROXIMITY_SENSOR
	gsl_read_interface(client, 0xac, buf, 4);
	tmp_prox = buf[0]&0x0f;
	if(proximity_enable==1)
	{
	  	if(tmp_prox == 0x01)		/* Closing */
	  	{
			ps_data_state[0] = 0;
			input_report_abs(ddata->input_dev_ps, ABS_DISTANCE, ps_data_state[0]);
			input_sync(ddata->input_dev_ps);
	  	}
	  	else if(tmp_prox == 0x00) 	/* Leaving*/
	  	{
			ps_data_state[0] = 1;
			input_report_abs(ddata->input_dev_ps, ABS_DISTANCE, ps_data_state[0]);
			input_sync(ddata->input_dev_ps);
	  	}
	}
#endif

/* Gesture Resume */
#ifdef GSL_GESTURE
	
		if(GE_ENABLE == gsl_gesture_status && ((gsl_gesture_flag == 1)||(gsl_gesture_flag == 2))){
			int tmp_c;
			u8 key_data = 0;
			tmp_c = gsl_obtain_gesture();
			print_info("gsl_obtain_gesture():tmp_c=0x%x[%d]\n",tmp_c,test_count++);
			print_info("gsl_obtain_gesture():tmp_c=0x%x\n",tmp_c);
			switch(tmp_c){
			case (int)'C':
				key_data = KEY_C;
				break;
			case (int)'E':
				key_data = KEY_E;
				break;
			case (int)'W':
				key_data = KEY_W;
				break;
			case (int)'O':
				key_data = KEY_O;
				break;
			case (int)'M':
				key_data = KEY_M;
				break;
			case (int)'Z':
				key_data = KEY_Z;
				break;
			case (int)'V':
				key_data = KEY_V;
				break;
			case (int)'S':
				key_data = KEY_S;
				break;
			case (int)'*':	
				key_data = KEY_WAKEUP;
				break;/* double click */
				case (int)0xa1fa:
				key_data = KEY_F1;
				break;/* right */
			case (int)0xa1fd:
				key_data = KEY_F2;
				break;/* down */
			case (int)0xa1fc:	
				key_data = KEY_F3;
				break;/* up */
			case (int)0xa1fb:	/* left */
				key_data = KEY_F4;
				break;	
			
			}
	
			if(!test_bit(key_data, gesture_bmp)){
				gsl_reset_core_without_vddio(client);	
				gsl_start_core(client);
			}
			else {
				gsl_gesture_c = (char)(tmp_c & 0xff);
				gsl_gesture_status = GE_WAKEUP;
				dev_dbg(&client->dev, "wake up gesture: %#02x '%c'\n",
					gsl_gesture_c, gsl_gesture_c);
				wake_lock_timeout(&ddata->gesture_wake_lock,
					GSL_GESTURE_WAKELOCK_DUR);

				//input_report_key(tpd->dev,key_data,1);
				input_report_key(idev, KEY_WAKEUP, 1);
				input_sync(idev);
				//input_report_key(tpd->dev,key_data,0);
				input_report_key(idev, KEY_WAKEUP, 0);
				input_sync(idev);
			}
			goto schedule;
		}
#endif

	//print_info("222 finger_num= %d\n",cinfo->finger_num);
	gsl_report_point(idev,cinfo);
	
schedule:
#ifdef GSL_TIMER
	mutex_unlock(&ddata->hw_lock);
i2c_lock_schedule:
#endif	
	return IRQ_HANDLED;
}

static int gsl_request_input_dev(struct gsl_ts_data *ddata)

{
	struct input_dev *input_dev;
	int err;
#if GSL_HAVE_TOUCH_KEY
	int i;
#endif
	/*allocate input device*/
	print_info("==input_allocate_device=\n");
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		goto err_allocate_input_device_fail;
	}
	ddata->idev = input_dev;
	/*set input parameter*/	
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT,input_dev->propbit);
#ifdef GSL_REPORT_POINT_SLOT
	__set_bit(EV_REP, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT,input_dev->propbit);
	input_mt_init_slots(input_dev,5,0);
#else 
	__set_bit(BTN_TOUCH, input_dev->keybit);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID,0,5,0,0);
#endif
	__set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	__set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
#ifdef TOUCH_VIRTUAL_KEYS
	__set_bit(KEY_MENU,  input_dev->keybit);
	__set_bit(KEY_BACK,  input_dev->keybit);
	__set_bit(KEY_HOME,  input_dev->keybit);    
#endif
#if GSL_HAVE_TOUCH_KEY
	for(i=0;i<GSL_KEY_NUM;i++)
	{
		__set_bit(gsl_key_data[i].key, input_dev->keybit);
	}
#endif
	
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, GSL_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, GSL_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);

	input_dev->name = GSL_TS_NAME;		//dev_name(&client->dev)

	/*register input device*/
	err = input_register_device(input_dev);
	if (err) {
		goto err_register_input_device_fail;
	}
	return 0;
err_register_input_device_fail:
	input_free_device(input_dev);			
err_allocate_input_device_fail:
	return err;
}

#if defined(CONFIG_FB)
static void gsl_ts_suspend(void)
{
	u32 tmp;
#ifndef GSL_GESTURE
	struct i2c_client *client = ddata->client;
#endif

	mutex_lock(&ddata->hw_lock);
	print_info("==gslX68X_ts_suspend=\n");
	//version info
	print_info("[tp-gsl]the last time of debug:%x\n",TPD_DEBUG_TIME);

	//if(1==ddata->gsl_sw_flag)
	//	return;
	ddata->gsl_halt_flag = 1;

#if defined(GSL_PROXIMITY_SENSOR)
	bNeedResumeTp = 0;
#endif

#ifdef GSL_PROXIMITY_SENSOR
  if(proximity_enable == 1)
  {
	  flag_tp_down = 1;
	  goto exit_unlock;
  }
  
  /*zhangpeng add start*/
  /*zhangpeng add end*/
#endif

	
#ifdef GSL_ALG_ID
	tmp = gsl_version_id();	
	print_info("[tp-gsl]the version of alg_id:%x\n",tmp);
#endif

#ifdef GSL_TIMER	
	cancel_delayed_work_sync(&gsl_timer_check_work);
#endif

/*Guesture Resume*/
#ifdef GSL_GESTURE
	/* always doze */
	gsl_enter_doze(ddata, gsl_gesture_flag == 2);
#else
	disable_irq_nosync(client->irq);
	gpio_set_value(GSL_RST_GPIO_NUM, 0);
	gsl_power_on(client, false);
#endif

#ifdef GSL_PROXIMITY_SENSOR
exit_unlock:
#endif
	mutex_unlock(&ddata->hw_lock);
	return;
}

static void gsl_ts_resume(void)
{	
	struct i2c_client *client = ddata->client;
	print_info("==gslX68X_ts_resume=\n");

	mutex_lock(&ddata->hw_lock);
	//if(1==ddata->gsl_sw_flag){
	//	ddata->gsl_halt_flag = 0;
	//	return;
	//}

	/* Proximity Sensor */
	#ifdef GSL_PROXIMITY_SENSOR

		if(proximity_enable == 1)
		{
			if(flag_tp_down == 1)
			{
				flag_tp_down = 0;
			}
			goto exit_unlock;
		}

		//zhangpeng add start
		//zhangpeng add end
	
  	#endif

	/*Gesture Resume*/
	#ifdef GSL_GESTURE
		WARN_ON(!dozing);
		gsl_quit_doze(ddata);
	#else
		gsl_power_on(client, true);
		gpio_set_value(GSL_RST_GPIO_NUM, 1);
		msleep(20);
		enable_irq(client->irq);
	#endif
	
	gsl_reset_core(client);	
	gsl_start_core(client);
	check_mem_data(client);
	
#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1){
		goto exit_unlock;
	}
#endif
	
#ifdef GSL_TIMER
	ddata->watchdog_counter = 0;
	queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, 300);
#endif

	ddata->gsl_halt_flag = 0;

#if defined(GSL_PROXIMITY_SENSOR) || defined(TPD_PROC_DEBUG)
exit_unlock:
#endif
	mutex_unlock(&ddata->hw_lock);
	return;

}

static bool unblanked_once = false;
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	
	if (evdata && evdata->data && event == FB_EVENT_BLANK ){
		blank = evdata->data;
		print_info("fb_notifier_callback blank=%d\n",*blank);
		if (*blank == FB_BLANK_UNBLANK) {
			if (unblanked_once)
				gsl_ts_resume();
		} else if (*blank == FB_BLANK_POWERDOWN) {
			unblanked_once = true;
			gsl_ts_suspend();
		}
	}

	return 0;
}
#endif

#if defined(CONFIG_FB)
static int gsl_parse_dt(struct device *dev, struct gsl_ts_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	print_info("%s\n",__func__);
	/* reset, irq gpio info */
	pdata->reset_gpio_number= of_get_named_gpio_flags(np, "silead,reset-gpio",
				0, &pdata->reset_gpio_flags);
	pdata->irq_gpio_number= of_get_named_gpio_flags(np, "silead,irq-gpio",
				0, &pdata->irq_gpio_flags);
       print_info("%s reset: %d, irq: %d\n",__func__,pdata->reset_gpio_number,pdata->irq_gpio_number);
	return 0;
}
#else
static int gsl_parse_dt(struct device *dev, struct mxt_platform_data *pdata)
{
	return -ENODEV;
}
#endif

#ifdef GSL_TEST_TP
//static int test_flag = 0;
static ssize_t gsl_test_show(void)
{
	static int gsl_test_flag = 0; 
	//ssize_t len=0;
	char *tmp_buf;
	int err;
	//int i;
	int result = 0;
	printk("enter gsl_test_show start::gsl_test_flag  = %d\n",gsl_test_flag);
	if(gsl_test_flag == 1){
		return 0;	
	}
	gsl_test_flag = 1;
	tmp_buf = kzalloc(3*1024,GFP_KERNEL);
	if(!tmp_buf){
		printk(" kzalloc  kernel  fail\n");
		return 0;
		}
	printk("why=========%s::begin\n",__func__);
	err = gsl_tp_module_test(tmp_buf,3*1024);
	printk("why=========%s::end\n",__func__);

	printk("enter gsl_test_show end\n");
	if(err > 0){
		printk("tp test pass\n");
		result = 1;

	}else{
		printk("tp test failure\n");
		result = 0;
	}
	/*
	for(i=0;i<3;i++){
		sprintf(&buf[len], "%s", &tmp_buf[i*1024]);
		len += strlen(&tmp_buf[i*1024]);
	}  */
	kfree(tmp_buf);
	gsl_test_flag = 0; 
	return result;
}

static ssize_t gsl_openshort_proc_read(struct file *file, char __user *buf,size_t count, loff_t *ppos)
{
	char *ptr = buf;
	int test_result  = 0;
	if(*ppos)
	{
		printk("tp test again return\n");
		return 0;
	}
	*ppos += count;

	test_result = gsl_test_show();
	//test_result = 1;

	if(1 == test_result)
	{
		printk("tp test pass\n");
		//sprintf(ptr, "result=%d\n", 1);
		ptr += sprintf(ptr, "%d", 1);
	}
	else
	{
		printk("tp test failure\n");
		//sprintf(ptr, "result=%d\n", 0);
		ptr += sprintf(ptr, "%d", 0);
	}
	return test_result;
}

static ssize_t gsl_openshort_proc_write(struct file *filp, const char __user *userbuf,size_t count, loff_t *ppos)
{
	return -1;
}

//static s32 ctp_openshort_proc_read(struct file *file, char __user *user_buf,size_t count, loff_t *ppos);
//static s32 ctp_openshort_proc_write(struct file *filp, const char __user *userbuf,size_t count, loff_t *ppos);
static const struct file_operations gsl_openshort_procs_fops =
{
    .write = gsl_openshort_proc_write,
    .read = gsl_openshort_proc_read,
    .open = simple_open,
    .owner = THIS_MODULE,
};

void create_ctp_proc(void)
{
	struct proc_dir_entry *gsl_device_proc = NULL;
	struct proc_dir_entry *gsl_openshort_proc = NULL;
	//struct proc_dir_entry *gsl_rawdata_proc = NULL;

	printk("why>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");

	gsl_device_proc = proc_mkdir(GSL_PARENT_PROC_NAME, NULL);
	if(gsl_device_proc == NULL)
    	{
        	printk("gsl915: create parent_proc fail\n");
        	return;
    	}

	gsl_openshort_proc = proc_create(GSL_OPENHSORT_PROC_NAME, 0777, gsl_device_proc, &gsl_openshort_procs_fops);
    	if (gsl_openshort_proc == NULL)
    	{
        	printk("gsl915: create openshort_proc fail\n");
    	}
}

#endif

static int gsl_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	int err = 0;
	int ret;
	int version;
	
	#ifdef GSL_PROXIMITY_SENSOR
			struct input_dev *input_dev_ps;
	#endif
	
	char *info = tp_info;

	struct gsl_ts_platform_data *pdata;
	print_info("%s\n",__func__);

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct gsl_ts_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = gsl_parse_dt(&client->dev, pdata);
		if (err)
			return err;
	} else
		pdata = client->dev.platform_data;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
        print_info("%s: I2c doesn't work\n", __func__);
		goto exit_check_functionality_failed;
	}

	print_info("==kzalloc=");
	ddata = kzalloc(sizeof(struct gsl_ts_data), GFP_KERNEL);
	if (ddata==NULL)
	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}
	
	ddata->gsl_halt_flag = 0;
	ddata->gsl_sw_flag = 0;
	ddata->gsl_up_flag = 0;
	ddata->gsl_point_state = 0;
#if GSL_HAVE_TOUCH_KEY
	ddata->gsl_key_state = 0;
#endif	
	ddata->cinfo = kzalloc(sizeof(struct gsl_touch_info),GFP_KERNEL);
	if(ddata->cinfo == NULL)
	{
		err = -ENOMEM;
		goto exit_alloc_cinfo_failed;
	}
	mutex_init(&gsl_i2c_lock);
	mutex_init(&ddata->hw_lock);
		
	ddata->client = client;
	i2c_set_clientdata(client, ddata);
	print_info("I2C addr=%x\n", client->addr);	
	gsl_hw_init();
	err = gsl_compatible_id(client);
	if(err < 0)
	{
		goto exit_i2c_transfer_fail; 
	}
	/*request input system*/	
	err = gsl_request_input_dev(ddata);
	if(err < 0)
	{
		goto exit_i2c_transfer_fail;	
	}

	
	/*register early suspend*/
	print_info("==register_early_suspend =\n");
	
	#if defined(CONFIG_FB)
	ddata->fb_notif.notifier_call = fb_notifier_callback;
	err = fb_register_client(&ddata->fb_notif);
	if (err)
		dev_err(&ddata->client->dev,
			"Unable to register fb_notifier: %d\n",
			err);
	#endif

	/*gesture resume*/
	#ifdef GSL_GESTURE
		gsl_GestureExternInt(gsl_model_extern,sizeof(gsl_model_extern) / sizeof(unsigned int) / 18);
		gsl_FunIICRead(gsl_read_oneframe_data);
	#endif
	
	/*gsl of software init*/
	gsl_sw_init(client);
	msleep(20);
	check_mem_data(client);

#ifdef TOUCH_VIRTUAL_KEYS
	gsl_ts_virtual_keys_init();
#endif

#ifdef TPD_PROC_DEBUG
#if 0
	gsl_config_proc = create_proc_entry(GSL_CONFIG_PROC_FILE, 0666, NULL);
	if (gsl_config_proc == NULL)
	{
		print_info("create_proc_entry %s failed\n", GSL_CONFIG_PROC_FILE);
	}
	else
	{
		gsl_config_proc->read_proc = gsl_config_read_proc;
		gsl_config_proc->write_proc = gsl_config_write_proc;
	}
#else
	proc_create(GSL_CONFIG_PROC_FILE,0666,NULL,&gsl_seq_fops);
#endif
	gsl_proc_flag = 0;
#endif
	
#ifdef GSL_TIMER
	INIT_DELAYED_WORK(&gsl_timer_check_work, gsl_watchdog);
	gsl_timer_workqueue = create_workqueue("gsl_timer_check");
	queue_delayed_work(gsl_timer_workqueue, &gsl_timer_check_work, GSL_TIMER_CHECK_CIRCLE);
#endif
	ret = sysfs_create_group(&client->dev.kobj,&gsl_attr_group);

#ifdef GSL_GESTURE
		input_set_capability(ddata->idev, EV_KEY, KEY_WAKEUP);//б┴бщ2иви║?ии?б┴ио?ж╠ик3
		input_set_capability(ddata->idev, EV_KEY, KEY_C);
		input_set_capability(ddata->idev, EV_KEY, KEY_E);
		input_set_capability(ddata->idev, EV_KEY, KEY_O);
		input_set_capability(ddata->idev, EV_KEY, KEY_W);
		input_set_capability(ddata->idev, EV_KEY, KEY_M);
		input_set_capability(ddata->idev, EV_KEY, KEY_Z);
		input_set_capability(ddata->idev, EV_KEY, KEY_V);
		input_set_capability(ddata->idev, EV_KEY, KEY_S);
		/*input_set_capability(tpd->dev, EV_KEY, KEY_F1);
		input_set_capability(tpd->dev, EV_KEY, KEY_F2);
		input_set_capability(tpd->dev, EV_KEY, KEY_F3);
		input_set_capability(tpd->dev, EV_KEY, KEY_F4);  */
#endif

#ifdef GSL_PROXIMITY_SENSOR
	input_dev_ps = input_allocate_device();
	if (!input_dev_ps) {
		print_info("%s: Failed to allocate input device ps\n", __func__);
	}

	ddata->input_dev_ps = input_dev_ps;
	__set_bit(EV_ABS, input_dev_ps->evbit);
	input_set_abs_params(input_dev_ps, ABS_DISTANCE, 0, 1, 0, 0);
	input_dev_ps->name = "proximity";
	err = input_register_device(input_dev_ps);
	
	gsl_gain_psensor_data(ddata->client);
	ddata->ps_cdev = sensors_proximity_cdev;
	ddata->ps_cdev.sensors_enable = psensor_ps_set_enable;
	ddata->ps_cdev.sensors_poll_delay = NULL;
	err = sensors_classdev_register(&client->dev, &ddata->ps_cdev);
#endif

#if defined(GSL_PROXIMITY_SENSOR)
	   wake_lock_init(&ps_wake_lock, WAKE_LOCK_SUSPEND, "ps_wakelock");
#endif

#ifdef GSL_TEST_TP
		create_ctp_proc();
#endif

	//zhangpeng add for TW test.
	//add for The hardware information begin
	version = gsl_ts_read_version();
	if( version == 0 ){
		print_info("[%s] No firmware version information",__func__);
	}

	info += sprintf(info,"KOTL,");
	info += sprintf(info,"GSL915,");
	info += sprintf(info,"%x",version);
	//end

#ifdef GSL_GESTURE
	set_bit(KEY_WAKEUP, gesture_bmp);
	wake_lock_init(&ddata->gesture_wake_lock,
		WAKE_LOCK_SUSPEND, "gsl_ts_gesture");
#endif

	/* TODO - get IRQ from device tree */
	client->irq = GSL_IRQ_NUM;

	dev_info(&client->dev, "%s IRQ number is %d\n", client->name, client->irq);
	err = request_threaded_irq(client->irq, NULL, gsl_ts_isr,
		IRQF_TRIGGER_RISING | IRQF_ONESHOT, client->name, ddata);
	if (err < 0) {
		dev_err(&client->dev, "gslX68X_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

#ifdef GSL_GESTURE
	if (gsl_gesture_flag)
		enable_irq_wake(client->irq);
#endif

	//is_tp_driver_loaded = 1;
	print_info("%s: ==probe over =\n",__func__);
	return 0;

exit_irq_request_failed:
#ifdef GSL_GESTURE
	wake_lock_destroy(&ddata->gesture_wake_lock);
#endif
	#if defined(CONFIG_FB)
	if (fb_unregister_client(&ddata->fb_notif))
		dev_err(&client->dev,
			"Error occurred while unregistering fb_notifier.\n");
	#endif
 
	input_unregister_device(ddata->idev);
	input_free_device(ddata->idev);
exit_i2c_transfer_fail:
	gpio_free(GSL_RST_GPIO_NUM);
	gpio_free(GSL_IRQ_GPIO_NUM);
	i2c_set_clientdata(client, NULL);
	kfree(ddata->cinfo);
exit_alloc_cinfo_failed:
	kfree(ddata);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int  gsl_ts_remove(struct i2c_client *client)
{

	print_info("==gslX68X_ts_remove=\n");
	
#ifdef GSL_GESTURE
	wake_lock_destroy(&ddata->gesture_wake_lock);
#endif
	if (fb_unregister_client(&ddata->fb_notif))
			dev_err(&client->dev,"Error occurred while unregistering fb_notifier.\n");

	free_irq(client->irq,ddata);
	input_unregister_device(ddata->idev);
	input_free_device(ddata->idev);
	gpio_free(GSL_RST_GPIO_NUM);
	gpio_free(GSL_IRQ_GPIO_NUM);
	
	i2c_set_clientdata(client, NULL);
	//sprd_free_gpio_irq(client->irq);
	kfree(ddata->cinfo);
	kfree(ddata);

	return 0;
}


static const struct i2c_device_id gsl_ts_id[] = {
	{ GSL_TS_NAME, GSL_TS_ADDR },
	{ }
};

#ifdef CONFIG_PM
static int gsl_ts_pm_suspend(struct device *dev)
{
	struct gsl_ts_data *ddata = dev_get_drvdata(dev);

#ifdef GSL_GESTURE
	/* expect the screen to be blanked when suspended */
	if (WARN_ON(!dozing))
		return -EAGAIN;
#endif

	/* there shouldn't be any pending delayed work when dozing */
	if (WARN_ON(delayed_work_pending(&gsl_timer_check_work)))
		return -EAGAIN;

	disable_irq(ddata->client->irq);

#ifdef GSL_GESTURE
	if (gsl_gesture_flag) {
		dev_dbg(dev, "suspend: wake up enabled\n");
		enable_irq_wake(ddata->client->irq);
	} else {
		dev_dbg(dev, "suspend: wake up disabled\n");
	}
#else
	dev_dbg(dev, "suspend: wake up disabled\n");
#endif

	return 0;
}

static int gsl_ts_pm_resume(struct device *dev)
{
	struct gsl_ts_data *ddata = dev_get_drvdata(dev);

#ifdef GSL_GESTURE
	if (gsl_gesture_flag)
		disable_irq_wake(ddata->client->irq);
#endif

	enable_irq(ddata->client->irq);

	dev_dbg(dev, "resume\n");
	return 0;
}

SIMPLE_DEV_PM_OPS(gsl_ts_pm_ops, gsl_ts_pm_suspend, gsl_ts_pm_resume);
#else
#define gsl_ts_pm_ops NULL
#endif /* CONFIG_PM */

MODULE_DEVICE_TABLE(i2c, gsl_ts_id);

#if defined(CONFIG_FB)
static struct of_device_id gsl_match_table[] = {
	{ .compatible = "silead,gsl-tp",},
	{ },
};
#else
#define gsl_match_table NULL
#endif

static struct i2c_driver gsl_ts_driver = {
	.driver = {
		.name = GSL_TS_NAME,
        .owner    = THIS_MODULE,
		.of_match_table = gsl_match_table,
		.pm = &gsl_ts_pm_ops,
	},
	.probe = gsl_ts_probe,
	.remove = gsl_ts_remove,
	.id_table	= gsl_ts_id,
};
 

module_i2c_driver(gsl_ts_driver);

MODULE_AUTHOR("sileadinc");
MODULE_DESCRIPTION("GSL Series Driver");
MODULE_LICENSE("GPL");

