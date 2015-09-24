#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/string.h>
#include <linux/cdev.h>
//#include <linux/earlysuspend.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/irq.h>
#include <linux/wakelock.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>
#include <linux/proc_fs.h>

#include <asm/unistd.h>
#include <asm/uaccess.h>

#ifdef  CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR
//#include <linux/input/vir_ps.h>
#include <linux/sensors.h>
#endif
#ifdef CONFIG_HY_DRV_ASSIST
#include <linux/hy-assist.h>
#endif

#define TPD_DEVICE_MSG "msg26XX"

#define U8         unsigned char
#define U16        unsigned short
#define U32        unsigned int
#define S8         signed char
#define S16        signed short
#define S32        signed int

#define u8         unsigned char
#define u16        unsigned short
#define u32        unsigned int
#define s8         signed char
#define s16        signed short
#define s32        signed int
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>

#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define FT_SUSPEND_LEVEL 1
#endif

struct msg2xxx_platform_data {

	const char *name;
	const char *fw_name;
	u32 irqflags;
	u32 irq_gpio;
	u32 irq_gpio_flags;
	u32 reset_gpio;
	u32 reset_gpio_flags;
	u32 family_id;
	u32 x_max;
	u32 y_max;
	u32 x_min;
	u32 y_min;
	u32 panel_minx;
	u32 panel_miny;
	u32 panel_maxx;
	u32 panel_maxy;
	u32 group_id;
	u32 hard_rst_dly;
	u32 soft_rst_dly;
	u32 num_max_touches;
	bool fw_vkey_support;
	bool no_force_update;
	bool i2c_pull_up;
	bool ignore_id_check;
	int (*power_init) (bool);
	int (*power_on) (bool);
};

struct msg2xxx_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct msg2xxx_platform_data *pdata;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	bool loading_fw;
	u8 family_id;
	struct dentry *dir;
	u16 addr;
	bool suspended;
	char *ts_info;
	u8 *tch_data;
	u32 tch_data_len;
	u8 fw_ver[3];
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	struct work_struct event_work;
	struct workqueue_struct *ts_workqueue;
	struct mutex int_lock;
#ifdef CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR
	struct sensors_classdev ps_cdev;
	struct input_dev *ps_input_dev;
	bool ps_enabled;
	struct delayed_work poll_work;
	unsigned long delay;
	struct workqueue_struct *wq;
	struct wake_lock ps_wakelock;
#endif
};
static struct msg2xxx_data *this_data;
#define VTG_MIN_UV		2600000
#define VTG_MAX_UV		3300000
#define I2C_VTG_MIN_UV	1800000
#define I2C_VTG_MAX_UV	1800000

#define MSG_COORDS_ARR_SIZE	4

#if 0
#define TPD_DEBUG(a,arg...) printk(KERN_ERR "msg" ": " a,##arg)
#else
#define TPD_DEBUG(arg...)
#endif

#define LCD_WIDTH               (480)   //modify : 分辨率，根据项目实际情况修改。
#define LCD_HEIGHT              (854)   //modify : 分辨率，根据项目实际情况修改。
#define TPD_WIDTH               (2048)
#define TPD_HEIGHT              (2048)
#define TPD_OK                  (0)
#define MAX_TOUCH_NUM           (5)
#define REPORT_PACKET_LENGTH    (43)
#define FW_ADDR_MSG_TP      (0x4C>>1)
#define FW_ADDR_MSG         (0xC4>>1)

static u8 g_ic_type = 0;
static int int_flag=0;

static 	struct input_dev *input=NULL;
struct i2c_client *tpd_i2c_client = NULL;
static struct work_struct msg_wq;
//static struct early_suspend early_suspend;
static char* fw_version = NULL;
static unsigned short curr_ic_major=0;
static unsigned short curr_ic_minor=0;


#ifdef CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR
int tsps_msg2xxx_enable_set(struct sensors_classdev *sensors_cdev,
						unsigned int enabled);
int tsps_msg2xxx_enable(struct msg2xxx_data *ps_data,
						unsigned int enabled);
int tsps_msg2xxx_data(void);

static void ps_poll_work_func(struct work_struct *work)
{
	int dist= 0;

	dist=tsps_msg2xxx_data();
	if (dist >= 0) {
		input_report_abs(this_data->ps_input_dev, ABS_DISTANCE, dist);
		input_sync(this_data->ps_input_dev);
		//printk("%s: ps dist:%d\n", __func__, dist);
	}
	queue_delayed_work(this_data->wq, &this_data->poll_work, msecs_to_jiffies(this_data->delay));

}

static ssize_t tp_ps_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    	int32_t enable;
	struct msg2xxx_data *ps_data =  dev_get_drvdata(dev);

	enable = (ps_data->ps_enabled)?1:0;

	return scnprintf(buf, PAGE_SIZE, "%d\n", enable);
}

static ssize_t tp_ps_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct msg2xxx_data *ps_data =  dev_get_drvdata(dev);
	uint8_t en;
	if (sysfs_streq(buf, "1"))
		en = 1;
	else if (sysfs_streq(buf, "0"))
		en = 0;
	else
	{
		printk(KERN_ERR "%s, invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}
	dev_dbg(dev, "%s: Enable PS : %d\n", __func__, en);
	
       tsps_msg2xxx_enable(ps_data, en);
	   
  	return size;
}

static ssize_t tp_ps_distance_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct msg2xxx_data *ps_data =  dev_get_drvdata(dev);
       int32_t dist=0;

	dist=tsps_msg2xxx_data();
	input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, dist);
	input_sync(ps_data->ps_input_dev);
	dev_err(dev, "%s: ps dist %d\n", __func__, dist);

	return scnprintf(buf, PAGE_SIZE, "%d\n", dist);
}


static ssize_t tp_ps_distance_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct msg2xxx_data *ps_data =  dev_get_drvdata(dev);
	unsigned long value = 0;
	int ret;
	ret = kstrtoul(buf, 10, &value);
	if(ret < 0)
	{
		printk(KERN_ERR "%s:kstrtoul failed, ret=0x%x\n",
			__func__, ret);
		return ret;
	}
    
	input_report_abs(ps_data->ps_input_dev, ABS_DISTANCE, value);
	input_sync(ps_data->ps_input_dev);

	dev_dbg(dev, "%s: ps value: %ld\n", __func__, value);

	return size;
}


static struct device_attribute ps_enable_attribute = __ATTR(enable,0666,tp_ps_enable_show,tp_ps_enable_store);
static struct device_attribute ps_distance_attribute = __ATTR(distance,0666,tp_ps_distance_show, tp_ps_distance_store);

static struct attribute *tp_ps_attrs [] =
{
    &ps_enable_attribute.attr,
    &ps_distance_attribute.attr,
    NULL
};

static struct attribute_group tp_ps_attribute_group = {
	.attrs = tp_ps_attrs,
};
#endif

#define MAX_KEY_NUM    (3)
static int tpd_key_array[MAX_KEY_NUM] = {0};//{ KEY_APPSELECT, KEY_BACK,KEY_HOMEPAGE,KEY_SEARCH};//modify : 根据项目实际情况修改

#define FIRMWARE_UPDATE
#ifdef FIRMWARE_UPDATE
static u8 bFwUpdating = 0;
#if 1
#define TPD_DEBUG_UPDATE(a,arg...) printk("msg_update" ": " a,##arg)
#else
#define TPD_DEBUG_UPDATE(arg...)
#endif

/*Modified by zhanghaibin for ito-test,20141119*/
#ifdef CONFIG_MSTAR_CTP_ITO_TEST
static int mito_test_create_entry(void);
static void _msg_create_file_for_ito_test(void);
static int ito_test_create_entry(void);
static void ito_test_remove_entry(void);//added by zhanghaibin
#endif
static void _msg_create_file_for_fwUpdate(void);

//#define AUTO_FIRMWARE_UPDATE
#ifdef AUTO_FIRMWARE_UPDATE
static void _msg_auto_updateFirmware(void);
#endif

#define PROC_FIRMWARE_UPDATE
#ifdef PROC_FIRMWARE_UPDATE
static int _msg_create_file_for_fwUpdate_proc(void);
#endif
#endif

//#define TP_PRINT
#ifdef TP_PRINT
static U8 g_bTpp = 0;
static struct mutex tpp_lock;
static void tp_print_create_entry(void);
#endif

//#define TP_DEBUG_ON	
#ifdef TP_DEBUG_ON
#include <linux/proc_fs.h>
#define DEBUG_AUTHORITY 0777
static struct proc_dir_entry *tp_debug = NULL;
static struct proc_dir_entry *debug_on = NULL;
#define PROC_TP_DEBUG      "tp_debug"
#define PROC_DEBUG_ON      "debug_on"
static int tp_debug_on=0;
static void tp_debug_create_entry(void);
#endif

#ifdef CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR
static u8 bEnableTpProximity = 0;
static u8 bFaceClosingTp = 1;
static int late_enable_proximity=0;
static void _msg_enable_proximity(void);
static int _msg_create_file_for_proximity_proc(void);
#endif


#if defined(FIRMWARE_UPDATE)||defined(CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR)
static u8 bNeedResumeTp = 0;
#endif

typedef struct
{
    U16 id;
    U16 x;
    U16 y;
    U16 p;
}   touchPoint_t;

typedef struct
{
    touchPoint_t point[MAX_TOUCH_NUM];
    U8 count;
    U8 keycode;
    U8 bKey;
    U8 bPoint;
    U8 bUp;
}   touchInfo_t;

#ifdef CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR
static struct sensors_classdev sensors_proximity_cdev = {
	.name = "proximity",
	.vendor = "tp",
	.version = 1,
	.handle = SENSORS_PROXIMITY_HANDLE,
	.type = SENSOR_TYPE_PROXIMITY,
	.max_range = "5.0",
	.resolution = "5.0",
	.sensor_power = "0.1",
	.min_delay = 0,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 200,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};
#endif
static void _msg_disable_irq(void);
static void _msg_enable_irq(void);
static void _msg_resetHW(void);

static int _msg_ReadI2CSeq(U8 addr, U8* read_data, U16 size)
{
    int rc;
    tpd_i2c_client->addr = addr;

    rc = i2c_master_recv(tpd_i2c_client, read_data, size);

    if( rc < 0 )
    {
        TPD_DEBUG("_msg_ReadI2CSeq error %d,addr=%d\n", rc,addr);
    }
    return rc;
}

static int _msg_WriteI2CSeq(U8 addr, U8* data, U16 size)
{
    int rc;
    tpd_i2c_client->addr = addr;

    rc = i2c_master_send(tpd_i2c_client, data, size);

    if( rc < 0 )
    {
        TPD_DEBUG("_msg_WriteI2CSeq error %d,addr = %d,data[0]=%d\n", rc, addr,data[0]);
    }
    return rc;
}

static void _msg_WriteReg8Bit( U8 bank, U8 addr, U8 data )
{
    U8 tx_data[4] = {0x10, bank, addr, data};
    _msg_WriteI2CSeq ( FW_ADDR_MSG, &tx_data[0], 4 );
}

static void _msg_WriteReg( U8 bank, U8 addr, U16 data )
{
    U8 tx_data[5] = {0x10, bank, addr, data & 0xFF, data >> 8};
    _msg_WriteI2CSeq ( FW_ADDR_MSG, &tx_data[0], 5 );
}

static unsigned short _msg_ReadReg( U8 bank, U8 addr )
{
    U8 tx_data[3] = {0x10, bank, addr};
    U8 rx_data[2] = {0};

    _msg_WriteI2CSeq ( FW_ADDR_MSG, &tx_data[0], 3 );
    _msg_ReadI2CSeq ( FW_ADDR_MSG, &rx_data[0], 2 );
    return ( rx_data[1] << 8 | rx_data[0] );
}

static void _msg_EnterSerialDebugMode(void)
{
    U8 data[5];

    /// change mode
    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;
    _msg_WriteI2CSeq(FW_ADDR_MSG, &data[0], 5);

    /// stop mcu
    data[0] = 0x37;
    _msg_WriteI2CSeq(FW_ADDR_MSG, &data[0], 1);

    /// IIC use bus
    data[0] = 0x35;
    _msg_WriteI2CSeq(FW_ADDR_MSG, &data[0], 1);

    /// IIC reshape
    data[0] = 0x71;
    _msg_WriteI2CSeq(FW_ADDR_MSG, &data[0], 1);
}

static void _msg_ExitSerialDebugMode(void)
{
    U8 data[1];

    /// IIC not use bus
    data[0] = 0x34;
    _msg_WriteI2CSeq(FW_ADDR_MSG, &data[0], 1);

    /// not stop mcu
    data[0] = 0x36;
    _msg_WriteI2CSeq(FW_ADDR_MSG, &data[0], 1);

    /// change mode
    data[0] = 0x45;
    _msg_WriteI2CSeq(FW_ADDR_MSG, &data[0], 1);
}

static void _msg_GetVersion(void)
{
    U8 tx_data[3] = {0};
    U8 rx_data[4] = {0};
    U16 Major = 0, Minor = 0;
    int rc_w = 0, rc_r = 0;

    TPD_DEBUG("_msg_GetVersion\n");

    tx_data[0] = 0x53;
    tx_data[1] = 0x00;
    tx_data[2] = 0x2A;
    rc_w = _msg_WriteI2CSeq(FW_ADDR_MSG_TP, &tx_data[0], 3);
    TPD_DEBUG("***rc_w=%d ***\n", rc_w);
    mdelay(50);
    rc_r = _msg_ReadI2CSeq(FW_ADDR_MSG_TP, &rx_data[0], 4);
    TPD_DEBUG("***rc_r=%d ***\n", rc_r);
    mdelay(50);

    Major = (rx_data[1]<<8) + rx_data[0];
    Minor = (rx_data[3]<<8) + rx_data[2];
    TPD_DEBUG("***major = %d ***\n", Major);
    TPD_DEBUG("***minor = %d ***\n", Minor);

    if(rc_w<0||rc_r<0)
    {
        curr_ic_major = 0xffff;
        curr_ic_minor = 0xffff;
    }
    else
    {
        curr_ic_major = Major;
        curr_ic_minor = Minor;
    }
    printk(KERN_ERR "curr_ic_major=%d,curr_ic_minor=%d\n",curr_ic_major,curr_ic_minor);
}
//modify :当前项目使用各家tp厂的主版本号，必须根据项目实际情况增减修改
#ifdef AUTO_FIRMWARE_UPDATE
#define TP_OF_XXX1 1
#define TP_OF_XXX2 2
#define TP_OF_XXX3 3
static u16 _msg_GetVersion_MoreTime(void)
{
    int version_check_time = 0;
    for(version_check_time=0;version_check_time<5;version_check_time++)
    {
        _msg_GetVersion();
        TPD_DEBUG_UPDATE("version_check_time=%d;curr_ic_major=%d;curr_ic_minor=%d \n",version_check_time,curr_ic_major,curr_ic_minor);
        if(TP_OF_XXX1==curr_ic_major
           ||TP_OF_XXX2==curr_ic_major
           ||TP_OF_XXX3==curr_ic_major)//modify :当前项目使用各家tp厂的主版本号，必须根据项目实际情况增减修改
        {
            break;
        }
        else if(version_check_time<3)
        {
            mdelay(100);
        }
        else
        {
            _msg_resetHW();
        }
    }
    return curr_ic_major;
}
#endif
static u8 _msg_GetIcType(void)
{
	_msg_resetHW();
    _msg_EnterSerialDebugMode();
    mdelay ( 300 );

    // stop mcu
    _msg_WriteReg8Bit ( 0x0F, 0xE6, 0x01 );
    // disable watch dog
    _msg_WriteReg ( 0x3C, 0x60, 0xAA55 );
    // get ic type
    g_ic_type = (0xff)&(_msg_ReadReg(0x1E, 0xCC));
    printk("_msg_GetIcType,g_ic_type=%d",g_ic_type);

    if(g_ic_type!=1//msg2133
        &&g_ic_type!=2//msg21xxA
        &&g_ic_type!=3)//msg26xx
    {
        g_ic_type = 0;
    }

    _msg_ExitSerialDebugMode();
    _msg_resetHW();

    return g_ic_type;

}

static U8 _msg_CalChecksum( U8 *msg, S32 s32Length )
{
    S32 s32Checksum = 0;
    S32 i;

    for ( i = 0 ; i < s32Length; i++ )
    {
        s32Checksum += msg[i];
    }

    return (U8)( ( -s32Checksum ) & 0xFF );
}

static S32 _msg_ParseInfo_21xx(touchInfo_t *info)
{
   // U32 i=0;
    U8 data[8] = {0};
    U8 checksum = 0;
    U32 x = 0, y = 0;
    U32 delta_x = 0, delta_y = 0;

#ifdef TP_PRINT
    if (g_bTpp)
    {
        mutex_lock(&tpp_lock);
        _msg_ReadI2CSeq( FW_ADDR_MSG_TP, &data[0], 8 );
        mutex_unlock(&tpp_lock);
    }
    else
#endif
    {
        _msg_ReadI2CSeq( FW_ADDR_MSG_TP, &data[0], 8 );
    }
    checksum = _msg_CalChecksum(&data[0], 7);
    TPD_DEBUG("check sum: [%x] == [%x]? \n", data[7], checksum);
#ifdef TP_DEBUG_ON
    if(tp_debug_on)
    {
        printk(KERN_ERR "check sum: [%x] == [%x]? \n", data[7], checksum);
    }
#endif

    if(data[7] != checksum)
    {
        TPD_DEBUG("WRONG CHECKSUM\n");
    #ifdef TP_DEBUG_ON
        if(tp_debug_on)
        {
            printk(KERN_ERR "WRONG CHECKSUM\n");
        }
    #endif
        return -1;
    }

    if(data[0] != 0x52)
    {
        TPD_DEBUG("WRONG HEADER\n");
    #ifdef TP_DEBUG_ON
        if(tp_debug_on)
        {
            printk(KERN_ERR "WRONG HEADER\n");
        }
    #endif
        return -1;
    }

    if( ( data[1] == 0xFF ) && ( data[2] == 0xFF ) && ( data[3] == 0xFF ) && ( data[4] == 0xFF ) && ( data[6] == 0xFF ) )
    {
        if(data[5]==0xFF||data[5]==0)
        {
            info->bUp = 1;
        }
        else if(data[5]==1||data[5]==2||data[5]==4||data[5]==8)
        {
            info->bKey = 1;
            if(data[5]==1)
            {
                info->keycode=0;
            }
            else if(data[5]==2)
            {
                info->keycode=1;
            }
            else if(data[5]==4)
            {
                info->keycode=2;
            }
            else if(data[5]==8)
            {
                info->keycode=3;
            }
        }
    #ifdef CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR
        else if(bEnableTpProximity
                &&(data[5]==0x80||data[5]==0x40))
        {
            if(data[5]==0x80 && bFaceClosingTp)
            {
                bFaceClosingTp = 0;
            }
            else if(data[5]==0x40 && !bFaceClosingTp)
            {
                bFaceClosingTp = 1;
            }
            printk("bEnableTpProximity=%d;bFaceClosingTp=%d;data[5]=%x;\n",bEnableTpProximity,bFaceClosingTp,data[5]);
            return -1;
        }
    #endif
        else
        {
            TPD_DEBUG("WRONG KEY\n");
            return -1;
        }
    }
    else
    {
        info->bPoint = 1;
        x = ( ( ( data[1] & 0xF0 ) << 4 ) | data[2] );
        y = ( ( ( data[1] & 0x0F ) << 8 ) | data[3] );
        delta_x = ( ( ( data[4] & 0xF0 ) << 4 ) | data[5] );
        delta_y = ( ( ( data[4] & 0x0F ) << 8 ) | data[6] );

        if(delta_x==0&&delta_y==0)
        {
            info->point[0].x = x * this_data->pdata->x_max / TPD_WIDTH;
            info->point[0].y = y *  this_data->pdata->y_max/ TPD_HEIGHT;
            info->point[0].p = 20;
            info->point[0].id = 0;
            info->count=1;
        }
        else
        {
            u32 x2=0, y2=0;
            if( delta_x > 2048 )
            {
                delta_x -= 4096;
            }
            if( delta_y > 2048 )
            {
                delta_y -= 4096;
            }
            x2 = ( u32 )( (s16)x + (s16)delta_x );
            y2 = ( u32 )( (s16)y + (s16)delta_y );
            info->point[0].x = x * this_data->pdata->x_max / TPD_WIDTH;
            info->point[0].y = y *  this_data->pdata->y_max/ TPD_HEIGHT;
            info->point[0].p = 20;
            info->point[0].id = 0;
            info->point[1].x = x2 * this_data->pdata->x_max / TPD_WIDTH;
            info->point[1].y = y2 *  this_data->pdata->y_max/ TPD_HEIGHT;
            info->point[1].p = 20;
            info->point[1].id = 1;
            info->count=2;
        }
    }
    return TPD_OK;
}

static S32 _msg_ParseInfo_26xx(touchInfo_t *info)
{
    U32 i=0;
    U8 data[REPORT_PACKET_LENGTH] = {0};
    U8 checksum = 0;
    U32 x = 0, y = 0;

#ifdef TP_PRINT
    if (g_bTpp)
    {
        mutex_lock(&tpp_lock);
        _msg_ReadI2CSeq( FW_ADDR_MSG_TP, &data[0], REPORT_PACKET_LENGTH );
        mutex_unlock(&tpp_lock);
    }
    else
#endif
    {
        _msg_ReadI2CSeq( FW_ADDR_MSG_TP, &data[0], REPORT_PACKET_LENGTH );
    }

    checksum = _msg_CalChecksum(&data[0], (REPORT_PACKET_LENGTH-1));
    TPD_DEBUG("check sum: [%x] == [%x]? \n", data[REPORT_PACKET_LENGTH-1], checksum);
#ifdef TP_DEBUG_ON
	 if(tp_debug_on)
	 {
	 	printk(KERN_ERR "check sum: [%x] == [%x]? \n", data[REPORT_PACKET_LENGTH-1], checksum);
	 }
#endif

    if(data[REPORT_PACKET_LENGTH-1] != checksum)
    {
        TPD_DEBUG("WRONG CHECKSUM\n");
#ifdef TP_DEBUG_ON
	if(tp_debug_on)
	{
		printk(KERN_ERR "WRONG CHECKSUM\n");
	}
#endif
        return -1;
    }

    if(data[0] != 0x5A)
    {
        TPD_DEBUG("WRONG HEADER\n");
#ifdef TP_DEBUG_ON
	if(tp_debug_on)
	{
	printk(KERN_ERR "WRONG HEADER\n");
	}
#endif
        return -1;
    }

    for(i = 0; i < MAX_TOUCH_NUM; i++)
    {
        if((data[(4*i)+1] == 0xFF) && (data[(4*i)+2] == 0xFF) && (data[(4*i)+3] == 0xFF))
        {
            continue;
        }

        x = (((data[(4*i)+1] & 0xF0) << 4) | (data[(4*i)+2]));
        y = (((data[(4*i)+1] & 0x0F) << 8) | (data[(4*i)+3]));

        info->point[info->count].x = x * this_data->pdata->x_max / TPD_WIDTH;
        info->point[info->count].y = y *  this_data->pdata->y_max/ TPD_HEIGHT;
        info->point[info->count].p = data[4*(i+1)];
        info->point[info->count].id = i;

        TPD_DEBUG("point[%d]: (%d, %d) = %d\n", info->point[info->count].id, info->point[info->count].x, info->point[info->count].y, info->point[info->count].p);

        info->count++;
    }

    if(info->count>0)
    {
        info->bPoint = 1;
        return TPD_OK;
    }
    else if(1==data[REPORT_PACKET_LENGTH-2]
          ||2==data[REPORT_PACKET_LENGTH-2]
          ||4==data[REPORT_PACKET_LENGTH-2]
          ||8==data[REPORT_PACKET_LENGTH-2])
    {
        if(data[REPORT_PACKET_LENGTH-2]==1)
        {
            info->keycode=0;
        }
        else if(data[REPORT_PACKET_LENGTH-2]==2)
        {
            info->keycode=1;
        }
        else if(data[REPORT_PACKET_LENGTH-2]==4)
        {
            info->keycode=2;
        }
        else if(data[REPORT_PACKET_LENGTH-2]==8)
        {
            info->keycode=3;
        }
        info->bKey = 1;
        return TPD_OK;
    }
#ifdef CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR
    else if(bEnableTpProximity
            &&(data[REPORT_PACKET_LENGTH-2]==0x80||data[REPORT_PACKET_LENGTH-2]==0x40))
    {
        if(data[REPORT_PACKET_LENGTH-2]==0x80 && bFaceClosingTp)
        {
            bFaceClosingTp = 0;
        }
        else if(data[REPORT_PACKET_LENGTH-2]==0x40 && !bFaceClosingTp)
        {
            bFaceClosingTp = 1;
        }
        printk("bEnableTpProximity=%d;bFaceClosingTp=%d;data[REPORT_PACKET_LENGTH-2]=%x;\n",bEnableTpProximity,bFaceClosingTp,data[REPORT_PACKET_LENGTH-2]);
#ifdef TP_DEBUG_ON
	if(tp_debug_on)
	{
	printk(KERN_ERR "bEnableTpProximity=%d;bFaceClosingTp=%d;data[REPORT_PACKET_LENGTH-2]=%x;\n",bEnableTpProximity,bFaceClosingTp,data[REPORT_PACKET_LENGTH-2]);
	}
#endif
        return -1;
    }
#endif
    else if(0xff==data[REPORT_PACKET_LENGTH-2])
    {
        info->bUp = 1;
        return TPD_OK;
    }
    return -1;

}

static void tpd_down(int x, int y, int p, int id)
{
#ifdef SWAP_X_Y
	int temp;
	temp = x;
	x = y;
	y = temp;
#endif
#ifdef REVERSE_X
	x = this_data->pdata->x_max-x;
#endif
#ifdef REVERSE_Y
	y =  this_data->pdata->y_max-y;
#endif
#ifdef TP_DEBUG_ON
	if(tp_debug_on)
	{
		printk(KERN_ERR "%s (%d,%d)\n",__func__,x,y);
	}
#endif
	input_report_key(input, BTN_TOUCH, 1);
	input_report_abs(input, ABS_MT_TOUCH_MAJOR, 1);
	input_report_abs(input, ABS_MT_POSITION_X, x);
	input_report_abs(input, ABS_MT_POSITION_Y, y);

	/*Modified by zhanghaibin for reporting 2 points,20140812*/
	if (g_ic_type == 3)
      { 
    		input_report_abs(input, ABS_MT_PRESSURE, p);    
    		input_report_abs(input, ABS_MT_TRACKING_ID, id); 
      }

	input_mt_sync(input);
}

static void tpd_up(void)
{
#ifdef TP_DEBUG_ON
	if(tp_debug_on)
	{
		printk(KERN_ERR "%s \n",__func__);
	}
#endif
	input_report_key(input, BTN_TOUCH, 0);
	input_mt_sync(input);
}

static irqreturn_t msg_interrupt(int irq, void *dev_id)
{
	/*Modified by zhanghaibin for TP can't work somtimes,becaue of nest,20140708*/
  	mutex_lock(&this_data->int_lock);
	if (int_flag == 0)
	{
		int_flag = 1;
	    	 _msg_disable_irq();
		schedule_work(&msg_wq);
	}
	mutex_unlock(&this_data->int_lock);
	
    return IRQ_HANDLED;
}

static void msg_do_work(struct work_struct *work)
{
    S32 ret = -1;
    touchInfo_t info;
    int i = 0;
    static int have_vk = 0;
    static int pre_vk = 0;
	
    memset(&info, 0x0, sizeof(info));
	
    if (g_ic_type == 2)
    {
        ret = _msg_ParseInfo_21xx(&info);
    }
    else if (g_ic_type == 3)
    {
        ret = _msg_ParseInfo_26xx(&info);
    }

    if (TPD_OK == ret)
    {
        if (info.bKey)
        {
        	TPD_DEBUG("msg_do_work  info.keycode =%x\n",info.keycode);
		#ifdef TP_DEBUG_ON
		 if(tp_debug_on)
		 {
		 	printk(KERN_ERR "msg_do_work  info.keycode =%x, tpd_key_array[info.keycode]=%d\n",info.keycode, tpd_key_array[info.keycode]);
		 }
		#endif
		 input_report_key(input, BTN_TOUCH, 1);
              input_report_key(input, tpd_key_array[info.keycode], 1);
 	      	 have_vk=1;
	        pre_vk = info.keycode;

        }
        else if(info.bPoint)
        {
            //have_vk=0;//Modified by zhanghaibin for vk up,20140917
            for(i=0; i<info.count; i++)
            {
                tpd_down(info.point[i].x, info.point[i].y, info.point[i].p, info.point[i].id);
            }
        }
        else if(info.bUp)
        {
             if(1==have_vk)
            	{
            		#ifdef TP_DEBUG_ON
            		 if(tp_debug_on)
			 {
			 	printk(KERN_ERR "msg_do_work  pre_vk =%x, tpd_key_array[pre_vk]=%d\n",pre_vk,tpd_key_array[pre_vk]);
			 }
			 #endif
            		input_report_key(input, BTN_TOUCH, 0);
                    input_report_key(input,  tpd_key_array[pre_vk], 0);
            		have_vk = 0;
            	}
		else
		{
			tpd_up();
		}
        }

        input_sync(input);
    }	
	_msg_enable_irq();
	
	mutex_lock(&this_data->int_lock);
	int_flag=0;
	mutex_unlock(&this_data->int_lock);
}


static void _msg_init_input(void)//modify :输入子系统，根据项目修改
{
	int err;
	int i;

	TPD_DEBUG("%s: msg21xx_i2c_client->name:%s\n", __func__,tpd_i2c_client->name);
	input = input_allocate_device();
	input->name = tpd_i2c_client->name;
	input->phys = "I2C";
	input->id.bustype = BUS_I2C;
	input->dev.parent = &tpd_i2c_client->dev;


	set_bit(EV_ABS, input->evbit);
	set_bit(EV_SYN, input->evbit);
	set_bit(BTN_TOUCH, input->keybit);
	set_bit(INPUT_PROP_DIRECT, input->propbit);
	set_bit(EV_KEY, input->evbit);
	set_bit(BTN_MISC,input->keybit);
       set_bit(KEY_OK, input->keybit);

	for (i = 0; i < MAX_KEY_NUM; i++) {
		input_set_capability(input, EV_KEY, tpd_key_array[i]);
	}


	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, 2, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 2, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, this_data->pdata->x_max , 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, this_data->pdata->y_max, 0, 0);

	err = input_register_device(input);
	TPD_DEBUG("error = %d\n", err);
}

static void _msg_set_rst_high(void)//modify: reset脚拉高, 根据项目修改
{
     gpio_set_value(this_data->pdata->reset_gpio, 1);
}

static void _msg_set_rst_low(void)//modify: reset脚拉低, 根据项目修改
{
    gpio_set_value(this_data->pdata->reset_gpio, 0);
}

static void _msg_resetHW(void)
{
    _msg_set_rst_low();
    mdelay (100);
   _msg_set_rst_high();
    mdelay(100);
}

static void _msg_disable_irq(void)//modify: 中断disable函数, 根据项目修改
{
   	disable_irq_nosync(tpd_i2c_client->irq);
}
static void _msg_enable_irq(void)//modify: 中断enable函数, 根据项目修改
{
    	enable_irq(tpd_i2c_client->irq);
}

static void tpd_resume( struct device *dev)
{
#if defined(FIRMWARE_UPDATE)||defined(CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR)
    if(bNeedResumeTp)
    {
	TPD_DEBUG("TPD wake up\n");
	_msg_set_rst_high();
	mdelay(100);
	_msg_enable_irq();
	TPD_DEBUG("TPD wake up done\n");
	/*Added by zhanghaibin for lcd backlight be turned on lately by press powerkey,20140725*/
	 #if defined(CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR)
	 if (late_enable_proximity)
	 {
	  	//enable proximity
		_msg_enable_proximity();
		queue_delayed_work(this_data->wq, &this_data->poll_work, msecs_to_jiffies(this_data->delay));
		late_enable_proximity=0;
	 }
	 #endif
	 bNeedResumeTp=0;
    }
    else
    {
        TPD_DEBUG("no need resume tp\n");
    }
#else
    TPD_DEBUG("TPD wake up\n");
    _msg_set_rst_high();
    mdelay(100);
    _msg_enable_irq();
    TPD_DEBUG("TPD wake up done\n");
#endif
}

static void tpd_suspend( struct device *dev )
{
     u8 i;
    TPD_DEBUG("TPD enter sleep\n");
	/*fix tp can't work when quick boot,zhanghaibin,20140911*/
//#if defined(FIRMWARE_UPDATE)||defined(CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR)
    //bNeedResumeTp = 0;
//#endif
#ifdef CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR
    if(bEnableTpProximity)
    {
        TPD_DEBUG("TPD canot enter sleep bEnableTpProximity=%d\n",bEnableTpProximity);
        return;
    }
#endif
#ifdef FIRMWARE_UPDATE
    if(bFwUpdating)
    {
        TPD_DEBUG("TPD canot enter sleep bFwUpdating=%d\n",bFwUpdating);
        return;
    }
#endif
	/*fix tp can't work when quick boot,zhanghaibin,20140911*/
	if (bNeedResumeTp)
	{
		TPD_DEBUG("TPD already in sleep\n");
		return;
	}
    	_msg_disable_irq();
	  /* Added by zhanghabin for touch ending,20140307*/
	for (i = 0; i < MAX_TOUCH_NUM; i++) {
		input_mt_slot(input, i);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(input, false);
	input_sync(input);

	_msg_set_rst_low();
#if defined(FIRMWARE_UPDATE)||defined(CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR)
    bNeedResumeTp = 1;
#endif
    TPD_DEBUG("TPD enter sleep done\n");
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct msg2xxx_data *msg2xxx_data =
		container_of(self, struct msg2xxx_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			msg2xxx_data && msg2xxx_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			tpd_resume(&msg2xxx_data->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			tpd_suspend(&msg2xxx_data->client->dev);
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void msg2xxx_early_suspend(struct early_suspend *handler)
{
	struct msg2xxx_data *data = container_of(handler,
						   struct msg2xxx_data,
						   early_suspend);

	tpd_suspend(&data->client->dev);
}

static void msg2xxx_late_resume(struct early_suspend *handler)
{
	struct msg2xxx_data *data = container_of(handler,
						   struct msg2xxx_data,
						   early_suspend);

	tpd_resume(&data->client->dev);
}
#endif
#ifdef CONFIG_PM
static const struct dev_pm_ops msg2xxx_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#endif
};
#endif

#ifdef CONFIG_OF
static int msg2xxx_get_dt_coords(struct device *dev, char *name,
				struct msg2xxx_platform_data *pdata)
{
	u32 coords[MSG_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != MSG_COORDS_ARR_SIZE) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "mstar,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	} else if (!strcmp(name, "mstar,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

static int msg2xxx_parse_dt(struct device *dev,
			struct msg2xxx_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val, num_buttons;
	//u32 button_map[MAX_BUTTONS];

	pdata->name = "mstar";
	rc = of_property_read_string(np, "mstar,name", &pdata->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read name\n");
		return rc;
	}

	rc = msg2xxx_get_dt_coords(dev, "mstar,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = msg2xxx_get_dt_coords(dev, "mstar,display-coords", pdata);
	if (rc)
		return rc;

	pdata->i2c_pull_up = of_property_read_bool(np,
						"mstar,i2c-pull-up");

	pdata->no_force_update = of_property_read_bool(np,
						"mstar,no-force-update");
	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "mstar,reset-gpio",
				0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "mstar,irq-gpio",
				0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;

	prop = of_find_property(np, "mstar,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_KEY_NUM)
			return -EINVAL;

		rc = of_property_read_u32_array(np,
			"mstar,button-map", tpd_key_array,
			num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read key codes\n");
			return rc;
		}
	}

	return 0;
}
#else
static int msg2xxx_parse_dt(struct device *dev,
			struct msg2xxx_platform_data *pdata)
{
	return -ENODEV;
}
#endif

static int msg2xxx_power_init(struct msg2xxx_data *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, VTG_MIN_UV,
					   VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, I2C_VTG_MIN_UV,
					   I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, VTG_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}

static int msg2xxx_power_on(struct msg2xxx_data *data, bool on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc=regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
		}
	}

	return rc;
}

#ifdef CONFIG_HY_DRV_ASSIST
static ssize_t msg2xxx_ic_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    if (g_ic_type == 2)
    	return sprintf(buf, "%s\n","Mstar.msg21xxA");
    else if (g_ic_type == 3)
    	return sprintf(buf, "%s\n","Mstar.msg263xM");
    else
    {
    	printk("error IC type=%d\n",g_ic_type);
	return sprintf(buf, "%s\n","Mstar.msg263xM");
    }
}
static ssize_t msg2xxx_fw_ver_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{

	//_msg_GetVersion();//Modified by zhanghaibin for tp version reading,20140923
	pr_info("%s curr_ic_major,curr_ic_minor(%x,%x)",__func__,curr_ic_major,curr_ic_minor);
	return sprintf(buf, "%d.%d\n", curr_ic_major,curr_ic_minor);
}
static ssize_t msg2xxx_tp_vendor_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    //_msg_GetVersion();//Modified by zhanghaibin for tp version reading,20140923
    pr_info("%s curr_ic_major=%x",__func__,curr_ic_major);
    /*curr_ic_major=1:rongna
    **curr_ic_major=2:yuda
    **curr_ic_major=3:helitai
    **curr_ic_major=4:zhongxianwei
    **curr_ic_major=5:junda
    */
    return sprintf(buf, "%d\n", curr_ic_major);
}
#endif

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	struct msg2xxx_platform_data *pdata;
	struct msg2xxx_data *data;
	int err;

	TPD_DEBUG("TPD probe\n");
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct msg2xxx_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = msg2xxx_parse_dt(&client->dev, pdata);
		if (err) {
			dev_err(&client->dev, "DT parsing failed\n");
			return err;
		}
	} else
		pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "Invalid pdata\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C not supported\n");
		return -ENODEV;
	}

	data = devm_kzalloc(&client->dev,
			sizeof(struct msg2xxx_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}
	data->client = client;
	data->pdata = pdata;
	this_data = data;
	i2c_set_clientdata(client, data);
	if (pdata->power_init) {
		err = pdata->power_init(true);
		if (err) {
			dev_err(&client->dev, "power init failed");
			return err;
		}
	} else {
		err = msg2xxx_power_init(data, true);
		if (err) {
			dev_err(&client->dev, "power init failed");
			return err;
		}
	}

	if (pdata->power_on) {
		err = pdata->power_on(true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	} else {
		err = msg2xxx_power_on(data, true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	}

	if (gpio_is_valid(pdata->irq_gpio)) {
		err = gpio_request(pdata->irq_gpio, "msg2xxx_irq_gpio");
		if (err) {
			dev_err(&client->dev, "irq gpio request failed");
			goto pwr_off;
		}
		err = gpio_direction_input(pdata->irq_gpio);
		if (err) {
			dev_err(&client->dev,
				"set_direction for irq gpio failed\n");
			goto free_irq_gpio;
		}
	}

	if (gpio_is_valid(pdata->reset_gpio)) {
		err = gpio_request(pdata->reset_gpio, "msg2xxx_reset_gpio");
		if (err) {
			dev_err(&client->dev, "reset gpio request failed");
			goto free_irq_gpio;
		}

		err = gpio_direction_output(pdata->reset_gpio, 0);
		if (err) {
			dev_err(&client->dev,
				"set_direction for reset gpio failed\n");
			goto free_reset_gpio;
		}
		msleep(10);//data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}
	msleep(100);

	tpd_i2c_client = client;

	mutex_init(&data->int_lock);
	
	if(0==_msg_GetIcType())
	{
	    printk("The currnet ic is not MSG\n");
	    goto error_mutex_destroy;
	}

	//read fw-version,zhanghaibin,20140923
	_msg_GetVersion();
	
	_msg_init_input();

#ifdef CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR
	data->ps_input_dev = input_allocate_device();
	if (data->ps_input_dev==NULL)
	{
		printk(KERN_ERR "%s: could not allocate ps device\n", __func__);
		err = -ENOMEM;
		//goto err_ps_input_allocate;
	}
	data->ps_input_dev->name = "proximity";
	set_bit(EV_ABS, data->ps_input_dev->evbit);
	
	input_set_abs_params(data->ps_input_dev, ABS_DISTANCE, 0,1, 0, 0);
	
	err = input_register_device(data->ps_input_dev);
	if (err<0)
	{
		printk(KERN_ERR "%s: can not register ps input device\n", __func__);
		goto err_ps_input_register;
	}

	
	err = sysfs_create_group(&data->ps_input_dev->dev.kobj, &tp_ps_attribute_group);
	if (err < 0)
	{
		printk(KERN_ERR "%s:could not create sysfs group for ps\n", __func__);
	}
	
	input_set_drvdata(data->ps_input_dev, data);
#endif
	INIT_WORK(&msg_wq, msg_do_work);
	printk("client->irq=%d\n",client->irq);
	err = request_threaded_irq(client->irq, NULL,
	                            msg_interrupt,
	                            IRQF_TRIGGER_RISING | IRQF_ONESHOT,
	                            client->dev.driver->name, data);
	if (err) {
		dev_err(&client->dev, "request irq failed\n");
		goto exit_create_singlethread;
	}

#ifdef CONFIG_MSTAR_CTP_ITO_TEST
	mito_test_create_entry();
	/*Added by zhanghaibin for ito-test,20140716*/
	_msg_create_file_for_ito_test();
	ito_test_create_entry();
#endif

#ifdef TP_PRINT
    tp_print_create_entry();
#endif

#ifdef TP_DEBUG_ON
    tp_debug_create_entry();
#endif

#ifdef CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR
    _msg_create_file_for_proximity_proc();
#endif

#ifdef FIRMWARE_UPDATE
    _msg_create_file_for_fwUpdate();
#ifdef PROC_FIRMWARE_UPDATE
    _msg_create_file_for_fwUpdate_proc();
#endif
#ifdef AUTO_FIRMWARE_UPDATE
    _msg_auto_updateFirmware();
#endif
#endif

#if defined(CONFIG_FB)
	data->fb_notif.notifier_call = fb_notifier_callback;

	err = fb_register_client(&data->fb_notif);

	if (err)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n",
			err);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
						    FT_SUSPEND_LEVEL;
	data->early_suspend.suspend = msg2xxx_early_suspend;
	data->early_suspend.resume = msg2xxx_late_resume;
	register_early_suspend(&data->early_suspend);
#endif
#ifdef CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR
	data->wq = create_singlethread_workqueue("ps_wq");
	if (!data->wq) {
		printk("ps create wq failed!\n");
		//goto err_create_wq;
	}
	//Added by zhanghaibin for ps can't turn on the screen when system sleep,20140818
	wake_lock_init(&data->ps_wakelock,WAKE_LOCK_SUSPEND, "ps_wakelock");
	INIT_DELAYED_WORK(&data->poll_work, ps_poll_work_func);
	data->delay = 150;
	
	data->ps_cdev = sensors_proximity_cdev;
	data->ps_cdev.sensors_enable = tsps_msg2xxx_enable_set;
	err = sensors_classdev_register(&client->dev, &data->ps_cdev);
	if (err)
		goto err_class_sysfs;
#endif

#if 0//def  CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR
	tsps_assist_register_callback("msg2xxxM",&tsps_msg2xxx_enable,&tsps_msg2xxx_data);
#endif

//Added by zhanghaibin for TP version.
#ifdef CONFIG_HY_DRV_ASSIST
	ctp_assist_register_attr("ic",&msg2xxx_ic_show,NULL);
	ctp_assist_register_attr("fw_ver",&msg2xxx_fw_ver_show,NULL);
	ctp_assist_register_attr("tp_vendor",&msg2xxx_tp_vendor_show,NULL);
#endif

    TPD_DEBUG("TPD probe done\n");
    
    return TPD_OK;   

#ifdef CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR
err_class_sysfs:
	destroy_workqueue(data->wq);
	wake_lock_destroy(&data->ps_wakelock);
#endif

exit_create_singlethread:
	//destroy_workqueue(data->ts_workqueue);
#ifdef CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR
	sysfs_remove_group(&data->ps_input_dev->dev.kobj, &tp_ps_attribute_group);
	input_unregister_device(data->ps_input_dev);
err_ps_input_register:
	input_free_device(data->ps_input_dev);
#endif
error_mutex_destroy:
	mutex_destroy(&data->int_lock);
free_reset_gpio:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
free_irq_gpio:
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
pwr_off:
	if (pdata->power_on)
		pdata->power_on(false);
	else
		msg2xxx_power_on(data, false);
pwr_deinit:
	if (pdata->power_init)
		pdata->power_init(false);
	else
		msg2xxx_power_init(data, false);

	return err;

}

static int __exit tpd_remove(struct i2c_client *client)
{
    	struct msg2xxx_data *data = i2c_get_clientdata(client);

	TPD_DEBUG("TPD removed\n");
#if defined(CONFIG_FB)
	if (fb_unregister_client(&data->fb_notif))
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif
	free_irq(client->irq, data);

	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);

	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);

	if (data->pdata->power_on)
		data->pdata->power_on(false);
	else
		msg2xxx_power_on(data, false);

	if (data->pdata->power_init)
		data->pdata->power_init(false);
	else
		msg2xxx_power_init(data, false);

	input_unregister_device(data->input_dev);

#ifdef CONFIG_MSTAR_CTP_ITO_TEST
    ito_test_remove_entry();
#endif

#ifdef TP_PRINT
    if (g_bTpp)
    {
        mutex_destroy(&tpp_lock);
        g_bTpp = 0;
    }
#endif
    mutex_destroy(&data->int_lock);
#ifdef CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR
    wake_lock_destroy(&data->ps_wakelock);
#endif
    return TPD_OK;
}

static const struct i2c_device_id tpd_id[] = {
	{ TPD_DEVICE_MSG, 0 },
	{ }
};

#ifdef CONFIG_OF
static struct of_device_id msg2xxx_match_table[] = {
	{ .compatible = "mstar,msg2xxx",},
	{ },
};
#else
#define msg2xxx_match_table NULL
#endif

static struct i2c_driver tpd_i2c_driver =
{
    .driver = {
        .name = TPD_DEVICE_MSG,
	.of_match_table = msg2xxx_match_table,
#ifdef CONFIG_PM
	.pm = &msg2xxx_pm_ops,
#endif
    },
    .probe = tpd_probe,
    //.remove = __devexit_p(tpd_remove),
    .remove = tpd_remove,
    .id_table = tpd_id,
};

static void _msg_register_device_and_driver(void)//modify :根据项目实际情况注册设备与驱动
{
	i2c_add_driver(&tpd_i2c_driver);
}
static void _msg_del_device_and_driver(void)//modify :根据项目实际情况卸载设备和驱动
{
    i2c_del_driver(&tpd_i2c_driver);
}
static int __init _msg_ts_init(void)
{
	TPD_DEBUG( "%s\n", __func__);
    _msg_register_device_and_driver();
	return 0;
}

static void __exit _msg_ts_exit(void)
{
    TPD_DEBUG( "%s\n", __func__);
	_msg_del_device_and_driver();
}

module_init(_msg_ts_init);
module_exit(_msg_ts_exit);

#ifdef FIRMWARE_UPDATE

#define FW_ADDR_MSG_UPDATE (0x4c>>1)
#define CTP_AUTHORITY 0777  //modify: 0777为打开apk升级功能，0664为关闭apk升级功能
static U8 temp[40][1024];
static U32 crc32_table[256];
static S32 FwDataCnt;
struct class *msg2xxx_firmware_class;
struct device *msg2xxx_firmware_cmd_dev;

static U32 _CRC_doReflect( U32 ref, S8 ch )
{
    U32 value = 0;
    U32 i = 0;

    for ( i = 1; i < ( ch + 1 ); i++ )
    {
        if ( ref & 1 )
        {
            value |= 1 << ( ch - i );
        }
        ref >>= 1;
    }

    return value;
}

U32 _CRC_getValue( U32 text, U32 prevCRC )
{
    U32 ulCRC = prevCRC;

    ulCRC = ( ulCRC >> 8 ) ^ crc32_table[ ( ulCRC & 0xFF ) ^ text];

    return ulCRC ;
}

static void _CRC_initTable( void )
{
    U32 magicnumber = 0x04c11db7;
    U32 i, j;

    for ( i = 0; i <= 0xFF; i++ )
    {
        crc32_table[i] = _CRC_doReflect ( i, 8 ) << 24;
        for ( j = 0; j < 8; j++ )
        {
            crc32_table[i] = ( crc32_table[i] << 1 ) ^ ( crc32_table[i] & ( 0x80000000L ) ? magicnumber : 0 );
        }
        crc32_table[i] = _CRC_doReflect ( crc32_table[i], 32 );
    }
}
typedef enum
{
    EMEM_ALL = 0,
    EMEM_MAIN,
    EMEM_INFO,
} EMEM_TYPE_t;
static S32 _updateFirmware_cash ( EMEM_TYPE_t emem_type )
{
    U32 i, j;
    U32 crc_main, crc_main_tp;
    U32 crc_info, crc_info_tp;
    U16 reg_data = 0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;

    /////////////////////////
    // Erase
    /////////////////////////

    TPD_DEBUG_UPDATE("erase 0\n");
    _msg_resetHW();
    _msg_EnterSerialDebugMode();
    mdelay ( 300 );

    TPD_DEBUG_UPDATE ("erase 1\n");

    // stop mcu
    _msg_WriteReg8Bit ( 0x0F, 0xE6, 0x01 );

    // disable watch dog
    _msg_WriteReg ( 0x3C, 0x60, 0xAA55 );
    TPD_DEBUG_UPDATE("erase 2\n");
    // set PROGRAM password
    _msg_WriteReg ( 0x16, 0x1A, 0xABBA );

    // clear pce
    _msg_WriteReg8Bit ( 0x16, 0x18, 0x80 );
    mdelay ( 10 );

    TPD_DEBUG_UPDATE ("erase 3\n");
    // trigger erase
    if ( emem_type == EMEM_ALL )
    {
        _msg_WriteReg8Bit ( 0x16, 0x0E, 0x08 ); //all
    }
    else
    {
        _msg_WriteReg8Bit ( 0x16, 0x0E, 0x04 ); //main
    }
     mdelay ( 10 );
    TPD_DEBUG_UPDATE("erase 4\n");
    do {
        reg_data = _msg_ReadReg ( 0x16, 0x10 );
    } while ( (reg_data & 0x0002) != 0x0002 );
    TPD_DEBUG_UPDATE("erase 5\n");
    // clear pce
    _msg_WriteReg8Bit ( 0x16, 0x18, 0x80 );
    mdelay ( 10 );
    _msg_WriteReg ( 0x16, 0x00, 0x0000 );
    _msg_WriteReg ( 0x16, 0x1A, 0x0000 );
    TPD_DEBUG_UPDATE("erase 6\n");
    //set pce to high
    _msg_WriteReg8Bit ( 0x16, 0x18, 0x40 );
    mdelay ( 10 );
    TPD_DEBUG_UPDATE("erase 7\n");
    //wait pce to ready
    do {
      reg_data = _msg_ReadReg ( 0x16, 0x10 );
    } while ( (reg_data & 0x0004) != 0x0004 );

    TPD_DEBUG_UPDATE ("erase OK\n");

    /////////////////////////
    // Program
    /////////////////////////

        TPD_DEBUG_UPDATE ("program 0\n");

    _msg_resetHW();//must
    _msg_EnterSerialDebugMode();
    mdelay ( 300 );

        TPD_DEBUG_UPDATE ("program 1\n");

    // Check_Loader_Ready: polling 0x3CE4 is 0x1C70
    do {
        reg_data = _msg_ReadReg ( 0x3C, 0xE4 );
    } while ( reg_data != 0x1C70 );

        TPD_DEBUG_UPDATE ("program 2\n");
    if ( emem_type == EMEM_ALL )
    {
        _msg_WriteReg ( 0x3C, 0xE4, 0xE38F ); //all
    }
    else
    {
        _msg_WriteReg ( 0x3C, 0xE4, 0x7731 );  //main
    }

    mdelay ( 100 );

    // Check_Loader_Ready2Program: polling 0x3CE4 is 0x2F43
    do {
        reg_data = _msg_ReadReg ( 0x3C, 0xE4 );
    } while ( reg_data != 0x2F43 );

        TPD_DEBUG_UPDATE ("program 3\n");

    // prepare CRC & send data
    _CRC_initTable ();

    for ( i = 0; i < (32+8); i++ ) // main 32 KB  + info 8KB
    {
        if ( i > 31 )
        {
            for ( j = 0; j < 1024; j++ )
            {
                crc_info = _CRC_getValue ( temp[i][j], crc_info);
            }
        }
        else if ( i < 31 )
        {
            for ( j = 0; j < 1024; j++ )
            {
                crc_main = _CRC_getValue ( temp[i][j], crc_main);
            }
        }
        else if ( i == 31 )
        {
            temp[i][1014] = 0x5A;
            temp[i][1015] = 0xA5;

            for ( j = 0; j < 1016; j++ )
            {
                crc_main = _CRC_getValue ( temp[i][j], crc_main);
            }
        }
	/*fix white display,zhanghaibin,20140620*/
        for(j=0; j<1; j++)
        {
            TPD_DEBUG_UPDATE ("i=%d,j=%d\n",i,j);
            _msg_WriteI2CSeq ( FW_ADDR_MSG_UPDATE, &temp[i][j*1024], 1024);
        }
        mdelay ( 100 );

        // Check_Program_Done: polling 0x3CE4 is 0xD0BC
        do {
            reg_data = _msg_ReadReg ( 0x3C, 0xE4 );
        } while ( reg_data != 0xD0BC );

        // Continue_Program
        _msg_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }

        TPD_DEBUG_UPDATE ("program 4\n");

    // Notify_Write_Done
    _msg_WriteReg ( 0x3C, 0xE4, 0x1380 );
    mdelay ( 100 );

        TPD_DEBUG_UPDATE ("program 5\n");

    // Check_CRC_Done: polling 0x3CE4 is 0x9432
    do {
       reg_data = _msg_ReadReg ( 0x3C, 0xE4 );
    } while ( reg_data != 0x9432 );

        TPD_DEBUG_UPDATE ("program 6\n");

    // check CRC
    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    // read CRC from TP
    crc_main_tp = _msg_ReadReg ( 0x3C, 0x80 );
    crc_main_tp = ( crc_main_tp << 16 ) | _msg_ReadReg ( 0x3C, 0x82 );
    crc_info_tp = _msg_ReadReg ( 0x3C, 0xA0 );
    crc_info_tp = ( crc_info_tp << 16 ) | _msg_ReadReg ( 0x3C, 0xA2 );

    TPD_DEBUG ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
               crc_main, crc_info, crc_main_tp, crc_info_tp );


    FwDataCnt = 0;
    _msg_ExitSerialDebugMode();
    _msg_resetHW();

    if ( crc_main_tp != crc_main || crc_info_tp != crc_info )//all
    {
        TPD_DEBUG_UPDATE ( "update FAILED\n" );

        return -1;
    }

    TPD_DEBUG_UPDATE ( "update OK\n" );

    return TPD_OK;
}

static int drvTP_erase_emem_c33 ( EMEM_TYPE_t emem_type )
{
    // stop mcu
    _msg_WriteReg ( 0x0F, 0xE6, 0x0001 );

    //disable watch dog
    _msg_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    _msg_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    // set PROGRAM password
    _msg_WriteReg8Bit ( 0x16, 0x1A, 0xBA );
    _msg_WriteReg8Bit ( 0x16, 0x1B, 0xAB );

    //proto.MstarWriteReg(F1.loopDevice, 0x1618, 0x80);
    _msg_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    if ( emem_type == EMEM_ALL )
    {
        _msg_WriteReg8Bit ( 0x16, 0x08, 0x10 ); //mark
    }

    _msg_WriteReg8Bit ( 0x16, 0x18, 0x40 );
    mdelay ( 10 );

    _msg_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    // erase trigger
    if ( emem_type == EMEM_MAIN )
    {
        _msg_WriteReg8Bit ( 0x16, 0x0E, 0x04 ); //erase main
    }
    else
    {
        _msg_WriteReg8Bit ( 0x16, 0x0E, 0x08 ); //erase all block
    }

    return ( 1 );
}

static ssize_t firmware_update_c33 ( struct device *dev, struct device_attribute *attr,
                                     const char *buf, size_t size, EMEM_TYPE_t emem_type )
{
//    u8  life_counter[2];
    u32 i, j;
    u32 crc_main, crc_main_tp;
    u32 crc_info, crc_info_tp;

    int update_pass = 1;
    u16 reg_data = 0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;
    TPD_DEBUG_UPDATE("111111111111");

    _msg_resetHW();
    _msg_EnterSerialDebugMode();
    mdelay ( 300 );

    //erase main
    TPD_DEBUG_UPDATE("aaaaaaaaaaa");
    drvTP_erase_emem_c33 ( EMEM_MAIN );
    mdelay ( 1000 );

    _msg_resetHW();
    _msg_EnterSerialDebugMode();
    mdelay ( 300 );

    /////////////////////////
    // Program
    /////////////////////////

    //polling 0x3CE4 is 0x1C70
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        do
        {
            reg_data = _msg_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0x1C70 );
    }

    switch ( emem_type )
    {
        case EMEM_ALL:
            _msg_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks
            break;
        case EMEM_MAIN:
            _msg_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for main block
            break;
        case EMEM_INFO:
            _msg_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for info block

            _msg_WriteReg8Bit ( 0x0F, 0xE6, 0x01 );

            _msg_WriteReg8Bit ( 0x3C, 0xE4, 0xC5 ); //
            _msg_WriteReg8Bit ( 0x3C, 0xE5, 0x78 ); //

            _msg_WriteReg8Bit ( 0x1E, 0x04, 0x9F );
            _msg_WriteReg8Bit ( 0x1E, 0x05, 0x82 );

            _msg_WriteReg8Bit ( 0x0F, 0xE6, 0x00 );
            mdelay ( 100 );
            break;
    }
    TPD_DEBUG_UPDATE("bbbbbbbbbbbbbb");
    // polling 0x3CE4 is 0x2F43
    do
    {
        reg_data = _msg_ReadReg ( 0x3C, 0xE4 );
    }
    while ( reg_data != 0x2F43 );
    TPD_DEBUG_UPDATE("ccccccccccccc");

    // calculate CRC 32
    _CRC_initTable ();

    for ( i = 0; i < 32; i++ ) // total  33 KB : 2 byte per R/W
    {
        if ( i == 31 )
        {
            temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
            temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

            for ( j = 0; j < 1016; j++ )
            {
                crc_main = _CRC_getValue ( temp[i][j], crc_main);
            }
        }
        else
        {
            for ( j = 0; j < 1024; j++ )
            {
                crc_main = _CRC_getValue ( temp[i][j], crc_main);
            }
        }

        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
        TPD_DEBUG_UPDATE("dddddddddddddd");
        _msg_WriteI2CSeq ( FW_ADDR_MSG_UPDATE, temp[i], 1024);

        TPD_DEBUG_UPDATE("eeeeeeeeeeee");
        // polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = _msg_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );
        TPD_DEBUG_UPDATE("ffffffffffffff");
        _msg_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }

    TPD_DEBUG_UPDATE("ggggggggg");
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // write file done and check crc
        _msg_WriteReg ( 0x3C, 0xE4, 0x1380 );
        TPD_DEBUG_UPDATE("hhhhhhhhhhhhhh");
    }
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // polling 0x3CE4 is 0x9432
        TPD_DEBUG_UPDATE("iiiiiiiiii");
        do
        {
            reg_data = _msg_ReadReg ( 0x3C, 0xE4 );
        }while ( reg_data != 0x9432 );
        TPD_DEBUG_UPDATE("jjjjjjjjjjjjj");
    }

    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        TPD_DEBUG_UPDATE("kkkkkkkkkkk");
        // CRC Main from TP
        crc_main_tp = _msg_ReadReg ( 0x3C, 0x80 );
        crc_main_tp = ( crc_main_tp << 16 ) | _msg_ReadReg ( 0x3C, 0x82 );

        // CRC Info from TP
        crc_info_tp = _msg_ReadReg ( 0x3C, 0xA0 );
        crc_info_tp = ( crc_info_tp << 16 ) | _msg_ReadReg ( 0x3C, 0xA2 );
    }
    TPD_DEBUG_UPDATE ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
               crc_main, crc_info, crc_main_tp, crc_info_tp );

    //drvDB_ExitDBBUS();
    TPD_DEBUG_UPDATE("lllllllllllll");
    update_pass = 1;
	if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        if ( crc_main_tp != crc_main )
            update_pass = 0;

        /*if ( crc_info_tp != crc_info )
            update_pass = 0;*/
    }

    if ( !update_pass )
    {
        TPD_DEBUG_UPDATE ( "update_C33 failed\n" );
	_msg_resetHW();
        FwDataCnt = 0;
        return 0;
    }

    TPD_DEBUG_UPDATE ( "update_C33 OK\n" );
    _msg_resetHW();
    FwDataCnt = 0;
    return size;
}

////////////////////////////////////////////////////////////////////////////////
static ssize_t firmware_update_show ( struct device *dev,
                                      struct device_attribute *attr, char *buf )
{
    TPD_DEBUG_UPDATE("*** firmware_update_show fw_version = %s***\n", fw_version);

    return sprintf ( buf, "%s\n", fw_version );
}

static ssize_t firmware_update_store ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t size )
{
    bFwUpdating = 1;
    _msg_disable_irq();

    TPD_DEBUG_UPDATE("*** update fw size = %d ***\n", FwDataCnt);

    if (g_ic_type == 2)
    {
        size = firmware_update_c33 ( dev, attr, buf, size, EMEM_MAIN );
    }
    else if (g_ic_type == 3)
    {
        if( 0 != _updateFirmware_cash (EMEM_ALL) )
        {
            size = 0;
            TPD_DEBUG_UPDATE ( "update failed\n" );
        }
        else
        {
           // size = 1;
           // TPD_DEBUG_UPDATE ( "update successfull size=%ld\n" ,size);
        }
    }

    _msg_enable_irq();
    bFwUpdating = 0;
    return size;
}

static DEVICE_ATTR(update, CTP_AUTHORITY, firmware_update_show, firmware_update_store);

static ssize_t firmware_version_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    TPD_DEBUG_UPDATE("*** firmware_version_show fw_version = %s***\n", fw_version);

    return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_version_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{
    _msg_GetVersion();
    if(fw_version == NULL)
    {
        fw_version = kzalloc(sizeof(*fw_version)*8, GFP_KERNEL);
    }
    sprintf(fw_version, "%03d%03d", curr_ic_major, curr_ic_minor);
    TPD_DEBUG_UPDATE("*** fw_version = %s ***\n", fw_version);
    if(buf != NULL)
        TPD_DEBUG_UPDATE("buf = %c ***\n", buf[0]);

    return size;
}

static DEVICE_ATTR(version, CTP_AUTHORITY, firmware_version_show, firmware_version_store);

static ssize_t firmware_data_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
    TPD_DEBUG_UPDATE("*** firmware_data_show FwDataCnt = %d***\n", FwDataCnt);

    return sprintf(buf, "%d\n", FwDataCnt);
}

static ssize_t firmware_data_store(struct device *dev,
                                   struct device_attribute *attr, const char *buf, size_t size)
{
    int count = size / 1024;
    int i;

    for( i=0; i<count; i++ )
    {
        memcpy(temp[FwDataCnt], buf+(i*1024), 1024);
        FwDataCnt++;
    }

    TPD_DEBUG_UPDATE("*** FwDataCnt = %d ***\n", FwDataCnt);
    if(buf != NULL)
        TPD_DEBUG_UPDATE("buf = %c ***\n", buf[0]);

    return size;
}

static DEVICE_ATTR(data, CTP_AUTHORITY, firmware_data_show, firmware_data_store);

//Added by zhanghaibin for apk can get IC type,20140929
ssize_t DrvMainFirmwareChipTypeShow(struct device *pDevice, struct device_attribute *pAttr, char *pBuf)
{
    pr_err("*** %s() ***\n", __func__);

    return sprintf(pBuf, "%d", g_ic_type);
}

ssize_t DrvMainFirmwareChipTypeStore(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    pr_err("*** %s() ***\n", __func__);

//    g_ChipType = DrvIcFwLyrGetChipType();

    return nSize;
}

static DEVICE_ATTR(chip_type, 0777,DrvMainFirmwareChipTypeShow, DrvMainFirmwareChipTypeStore);

static void _msg_create_file_for_fwUpdate(void)
{
    msg2xxx_firmware_class = class_create(THIS_MODULE, "ms-touchscreen-msg20xx");
    if (IS_ERR(msg2xxx_firmware_class))
        pr_err("Failed to create class(firmware)!\n");
    msg2xxx_firmware_cmd_dev = device_create(msg2xxx_firmware_class, NULL, 0, NULL, "device");
    if (IS_ERR(msg2xxx_firmware_cmd_dev))
        pr_err("Failed to create device(msg2xxx_firmware_cmd_dev)!\n");

    // version
    if (device_create_file(msg2xxx_firmware_cmd_dev, &dev_attr_version) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_version.attr.name);
    // update
    if (device_create_file(msg2xxx_firmware_cmd_dev, &dev_attr_update) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_update.attr.name);
    // data
    if (device_create_file(msg2xxx_firmware_cmd_dev, &dev_attr_data) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_data.attr.name);

	// chip type
    if (device_create_file(msg2xxx_firmware_cmd_dev, &dev_attr_chip_type) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_chip_type.attr.name);

    dev_set_drvdata(msg2xxx_firmware_cmd_dev, NULL);
}
#ifdef AUTO_FIRMWARE_UPDATE
//modify: 根据项目实际使用tp厂增减修改。
static unsigned char tp_of_xxx1_update_bin[]=
{
    #include "tp_of_xxx1_update_bin.i"
};
static unsigned char tp_of_xxx2_update_bin[]=
{
    #include "tp_of_xxx2_update_bin.i"
};
static unsigned char tp_of_xxx3_update_bin[]=
{
    #include "tp_of_xxx3_update_bin.i"
};

static int _auto_updateFirmware_cash(void *unused)
{
    int update_time = 0;
    ssize_t ret = 0;
    for(update_time=0;update_time<5;update_time++)
    {
        TPD_DEBUG_UPDATE("update_time = %d\n",update_time);
        ret = firmware_update_store(NULL, NULL, NULL, 0);
        if(ret==1)
        {
            TPD_DEBUG_UPDATE("AUTO_UPDATE OK!!!,update_time=%d\n",update_time);
            return 0;
        }
    }
    TPD_DEBUG_UPDATE("AUTO_UPDATE failed!!!,update_time=%d\n",update_time);
    return 0;
}

static void _msg_auto_updateFirmware(void)
{
    U16 tp_type = 0;
    unsigned short update_bin_major=0;
    unsigned short update_bin_minor=0;
    unsigned char *update_bin = NULL;

    tp_type=_msg_GetVersion_MoreTime();

    //modify :当前项目使用各家tp厂的主版本号，必须根据项目实际情况增减修改
    if(TP_OF_XXX1==tp_type)
    {
        update_bin = tp_of_xxx1_update_bin;
    }
    else if(TP_OF_XXX2==tp_type)
    {
        update_bin = tp_of_xxx2_update_bin;
    }
    else if(TP_OF_XXX3==tp_type)
    {
        update_bin = tp_of_xxx3_update_bin;
    }
    else
    {
        TPD_DEBUG_UPDATE("AUTO_UPDATE choose tp type failed,curr_ic_major=%d\n",tp_type);
        return;
    }

    FwDataCnt = 0;

	update_bin_major = update_bin[2*16+9]<<8|update_bin[2*16+10];
    update_bin_minor = update_bin[2*16+11]<<8|update_bin[2*16+12];
    TPD_DEBUG_UPDATE("bin_major = %d \n",update_bin_major);
    TPD_DEBUG_UPDATE("bin_minor = %d \n",update_bin_minor);

	if(update_bin_major==tp_type
        &&update_bin_minor>curr_ic_minor)
	{
	    int i = 0;
		for (i = 0; i < 40; i++)
		{
		    firmware_data_store(NULL, NULL, &(update_bin[i*1024]), 1024);
		}
        kthread_run(_auto_updateFirmware_cash, 0, "MSG21XXA_fw_auto_update");
	}
    else
    {
        TPD_DEBUG_UPDATE("AUTO_UPDATE not done,curr_ic_major=%d;curr_ic_minor=%d;update_bin_major=%d;update_bin_minor=%d\n",tp_type,curr_ic_minor,update_bin_major,update_bin_minor);
    }
}
#endif
#ifdef PROC_FIRMWARE_UPDATE
static ssize_t proc_version_read(struct file *file, char __user * buffer, size_t count,
		       loff_t * ppos)
{
    int cnt= 0;

    firmware_version_show(NULL,NULL, buffer);
	
    return cnt;
}

static ssize_t proc_version_write(struct file *file,
	const char __user *user_buf,
	size_t count, loff_t *ppos)
{    
    firmware_version_store(NULL, NULL, NULL, 0);
    return count;
}

static ssize_t proc_update_write(struct file *file,
	const char __user *user_buf,
	size_t count, loff_t *ppos)
{
    count = (unsigned long)firmware_update_store(NULL, NULL, NULL, (size_t)count);
    return count;
}

static ssize_t proc_data_read(struct file *file, char __user * buffer, size_t count,
		       loff_t * ppos)
{
    int cnt= 0;
    
    firmware_data_show(NULL, NULL, buffer);

    return cnt;
}

static ssize_t proc_data_write(struct file *file,
	const char __user *user_buf,
	size_t count, loff_t *ppos)
{
    firmware_data_store(NULL, NULL, user_buf, 0);
    return count;
}
#define CTP_AUTHORITY_PROC 0777 
static const struct file_operations fw_version_proc_fops = {
	.owner		= THIS_MODULE,
	.read		= proc_version_read,
	.write		= proc_version_write,
	//.release	= single_release,
};
static const struct file_operations fw_data_proc_fops = {
	.owner		= THIS_MODULE,
	.read		= proc_data_read,
	.write		= proc_data_write,
	//.release	= single_release,
};
static const struct file_operations fw_update_proc_fops = {
	.owner		= THIS_MODULE,
	.write		= proc_update_write,
	//.release	= single_release,
};
static int _msg_create_file_for_fwUpdate_proc(void)
{  
    struct proc_dir_entry *msg_class_proc = NULL;
    struct proc_dir_entry *msg_msg20xx_proc = NULL;
    struct proc_dir_entry *msg_device_proc = NULL;
    struct proc_dir_entry *msg_version_proc = NULL;
    struct proc_dir_entry *msg_update_proc = NULL;
    struct proc_dir_entry *msg_data_proc = NULL;
    
    msg_class_proc = proc_mkdir("class", NULL);
    if (!msg_class_proc)
		return -ENOMEM;
	
    msg_msg20xx_proc = proc_mkdir("ms-touchscreen-msg20xx",msg_class_proc);
    if (!msg_msg20xx_proc)
		return -ENOMEM;
	
    msg_device_proc = proc_mkdir("device",msg_msg20xx_proc);
    if (!msg_device_proc)
		return -ENOMEM;
    
    msg_version_proc = proc_create("version", CTP_AUTHORITY_PROC, msg_device_proc,&fw_version_proc_fops);
    if (msg_version_proc == NULL) 
    {
        TPD_DEBUG_UPDATE("create_proc_entry msg_version_proc failed\n");
	  goto fail1;
    } 
    
    msg_data_proc = proc_create("data", CTP_AUTHORITY_PROC,msg_device_proc,&fw_data_proc_fops);
    if (msg_data_proc == NULL) 
    {
        TPD_DEBUG_UPDATE("create_proc_entry msg_data_proc failed\n");
          goto fail2;
    } 
    
    msg_update_proc = proc_create("update", CTP_AUTHORITY_PROC, msg_device_proc,&fw_update_proc_fops);
    if (msg_update_proc == NULL) 
    {
        TPD_DEBUG_UPDATE("create_proc_entry msg_update_proc failed\n");
	  goto fail3;
    } 
     return 0;
	 
 fail3: remove_proc_entry("data", msg_device_proc);
 fail2: remove_proc_entry("version", msg_device_proc);
 fail1: 
 	remove_proc_entry("device", msg_msg20xx_proc);
	remove_proc_entry("ms-touchscreen-msg20xx", msg_class_proc);
	remove_proc_entry("class", NULL);
    return -ENOMEM;
}
#endif
#endif
#ifdef CONFIG_TOUCHSCREEN_PROXIMITY_SENSOR
 
static void _msg_enable_proximity(void)
{
    U8 tx_data[4] = {0};
    U8 rx_data[2] = {0};
    U8 i = 0;

    bEnableTpProximity = 0;
    TPD_DEBUG("_msg_enable_proximity!!!");
    for(i=0;i<5;i++)
    {
        tx_data[0] = 0x52;
        tx_data[1] = 0x00;
	 if (g_ic_type == 2)
       	tx_data[2] = 0x4a;
	 else  if (g_ic_type == 3)
		tx_data[2] = 0x47;
	 else
	 	printk("IC type error!\n");
        tx_data[3] = 0xa0;
        _msg_WriteI2CSeq(FW_ADDR_MSG_TP, &tx_data[0], 4);
        mdelay(100);
        tx_data[0] = 0x53;
        tx_data[1] = 0x00;
	 if (g_ic_type == 2)
		tx_data[2] = 0x4a;
	 else if (g_ic_type == 3)
	 	tx_data[2] = 0x47;
	 else
	 	printk("IC type error!\n");
        _msg_WriteI2CSeq(FW_ADDR_MSG_TP, &tx_data[0], 3);
        mdelay(50);
        _msg_ReadI2CSeq(FW_ADDR_MSG_TP, &rx_data[0], 2);
        mdelay(50);
	 //printk("[zhang]%s rx_data[0]=%x\n",__func__,rx_data[0]);
        if(0xa0==rx_data[0])
        {
            bEnableTpProximity = 1;
            break;
        }
        else if(i<2)
        {
            mdelay(100);
        }
        else
        {
            _msg_resetHW();
        }
    }
    if(bEnableTpProximity)
    {
        TPD_DEBUG("Proximity enable success!!!");
    }
    else
    {
        TPD_DEBUG("Proximity enable failed!!!");
    }
}
static void _msg_disable_proximity(void)
{
    U8 tx_data[4] = {0};
    U8 rx_data[2] = {0};
    U8 i = 0;
    TPD_DEBUG("_msg_disable_proximity!!");
    for(i=0;i<3;i++)
    {
        tx_data[0] = 0x52;
        tx_data[1] = 0x00;
	  if (g_ic_type == 2)
       	tx_data[2] = 0x4a;
	 else  if (g_ic_type == 3)
		tx_data[2] = 0x47;
	 else
	 	printk("IC type error!\n");
        tx_data[3] = 0xa1;
        _msg_WriteI2CSeq(FW_ADDR_MSG_TP, &tx_data[0], 4);
        mdelay(100);
        tx_data[0] = 0x53;
        tx_data[1] = 0x00;
	  if (g_ic_type == 2)
       	tx_data[2] = 0x4a;
	  else  if (g_ic_type == 3)
		tx_data[2] = 0x47;
	  else
	 	printk("IC type error!\n");
        _msg_WriteI2CSeq(FW_ADDR_MSG_TP, &tx_data[0], 3);
        mdelay(50);
        _msg_ReadI2CSeq(FW_ADDR_MSG_TP, &rx_data[0], 2);
        mdelay(50);
	 //printk("[zhang]%s rx_data[0]=%x\n",__func__,rx_data[0]);
        if(0xa1==rx_data[0])
        {
            break;
        }
        else if(i<2)
        {
            mdelay(100);
        }
        else
        {
            _msg_resetHW();
            break;
        }
    }
    bEnableTpProximity = 0;
    if(!bEnableTpProximity)
    {
        TPD_DEBUG("Proximity disable success!!!");
    }
    else
    {
        TPD_DEBUG("Proximity disable failed!!!");
    }
    bFaceClosingTp=1;
}

int tsps_msg2xxx_enable(struct msg2xxx_data *ps_data,
						unsigned int enabled)
{
	printk(KERN_ERR "%s enabled=%d\n",__func__,enabled);
	this_data->ps_enabled = enabled;
	if (enabled)
	{
		wake_lock(&this_data->ps_wakelock);
		/*Added by zhanghaibin for lcd backlight be turned on lately  by press powerkey,20140725*/
		if (bNeedResumeTp)
		{
			late_enable_proximity=1;//late enable ps
			return 0;
		}
		_msg_enable_proximity();
		queue_delayed_work(this_data->wq, &this_data->poll_work, msecs_to_jiffies(this_data->delay));
	}
	else
	{
		_msg_disable_proximity();
		cancel_delayed_work_sync(&this_data->poll_work);
		wake_unlock(&this_data->ps_wakelock);
	}
	return 0;
}

int tsps_msg2xxx_enable_set(struct sensors_classdev *sensors_cdev,
						unsigned int enabled)
{
	struct msg2xxx_data *ps_data = container_of(sensors_cdev,
						struct msg2xxx_data, ps_cdev);
	
	printk(KERN_ERR "%s enabled=%d\n",__func__,enabled);
	tsps_msg2xxx_enable(ps_data,enabled);
	
	return 0;
}
int tsps_msg2xxx_data(void)
{
    //printk(KERN_ERR "bFaceClosingTp=%d ",bFaceClosingTp);
    return bFaceClosingTp;
}

//static int proc_ps_read_to_debug(char *page, char **start, off_t off, int count, int *eof, void *data)
static ssize_t proc_ps_read_to_debug(struct file *file, char __user * buffer, size_t count,
		       loff_t * ppos)
{
    int cnt= 0;

    U8 tx_data[3] = {0};
    U8 rx_data[2] = {0};
    TPD_DEBUG("bEnableTpProximity=%d;bFaceClosingTp=%d\n",bEnableTpProximity,bFaceClosingTp);
    tx_data[0] = 0x53;
    tx_data[1] = 0x00;
    tx_data[2] = 0x47;
    _msg_WriteI2CSeq(FW_ADDR_MSG_TP, &tx_data[0], 3);
    mdelay(50);
    _msg_ReadI2CSeq(FW_ADDR_MSG_TP, &rx_data[0], 2);
    mdelay(50);
    TPD_DEBUG("rx_data[0]=%d\n",rx_data[0]);

    return cnt;
}

//static int proc_ps_write_to_enable(struct file *file, const char *buffer, unsigned long count, void *data)
static ssize_t proc_ps_write_to_enable(struct file *file,
	const char __user *user_buf,
	size_t count, loff_t *ppos)
{
    int value = -1;

    if(NULL!=user_buf)
    {
        sscanf(user_buf,"%d",&value);
    }
    if(value==1)
    {
        _msg_enable_proximity();
        TPD_DEBUG("_msg_enable_proximity,bEnableTpProximity=%d",bEnableTpProximity);
    }
    else if(value==0)
    {
        _msg_disable_proximity();
        TPD_DEBUG("_msg_disable_proximity,bEnableTpProximity=%d",bEnableTpProximity);
    }
    else
    {
        TPD_DEBUG("proc_ps_write_to_enable wrong input");
    }
    return count;
}
static const struct file_operations ps_proc_fops = {
	.owner		= THIS_MODULE,
	.read		= proc_ps_read_to_debug,
	.write		= proc_ps_write_to_enable,
	//.release	= single_release,
};
static int _msg_create_file_for_proximity_proc(void)
{  
    struct proc_dir_entry *msg_tp_ps = NULL;
    struct proc_dir_entry *msg_ps_proc = NULL;

    msg_tp_ps = proc_mkdir("tp_ps", NULL);
    if (!msg_tp_ps)
    	return -ENOMEM;
	
    msg_ps_proc = proc_create("ps", 0777, msg_tp_ps,&ps_proc_fops);
    if (msg_ps_proc == NULL) 
    {
        TPD_DEBUG("create_proc_entry tp_ps failed\n");
	  goto fail;
    } 
    return 0;
    fail: 
 	 remove_proc_entry("tp_ps", NULL);
        return -ENOMEM;
}
#endif

#ifdef CONFIG_MSTAR_CTP_ITO_TEST
//add .h here


u8 bItoTestDebug = 0;
#define ITO_TEST_DEBUG(format, ...) \
{ \
    if(bItoTestDebug) \
    { \
        printk(KERN_ERR "ito_test ***" format "\n", ## __VA_ARGS__); \
        mdelay(5); \
    } \
}
#define ITO_TEST_DEBUG_MUST(format, ...)	printk(KERN_ERR "ito_test ***" format "\n", ## __VA_ARGS__);mdelay(5)



s16  s16_raw_data_1[48] = {0};
s16  s16_raw_data_2[48] = {0};
s16  s16_raw_data_3[48] = {0};
u8 ito_test_keynum = 0;
u8 ito_test_dummynum = 0;
u8 ito_test_trianglenum = 0;
u8 ito_test_2r = 0;
u8 g_LTP = 1;
uint16_t *open_1 = NULL;
uint16_t *open_1B = NULL;
uint16_t *open_2 = NULL;
uint16_t *open_2B = NULL;
uint16_t *open_3 = NULL;
u8 *MAP1 = NULL;
u8 *MAP2=NULL;
u8 *MAP3=NULL;
u8 *MAP40_1 = NULL;
u8 *MAP40_2 = NULL;
u8 *MAP40_3 = NULL;
u8 *MAP40_4 = NULL;
u8 *MAP41_1 = NULL;
u8 *MAP41_2 = NULL;
u8 *MAP41_3 = NULL;
u8 *MAP41_4 = NULL;

#define ITO_TEST_ADDR_TP  (0x4C>>1)
#define ITO_TEST_ADDR_REG (0xC4>>1)
#define REG_INTR_FIQ_MASK           0x04
#define FIQ_E_FRAME_READY_MASK      ( 1 << 8 )
#define MAX_CHNL_NUM (48)
#define SSBIT0  (1<<0)
#define SSBIT1  (1<<1)
#define SSBIT2  (1<<1)
#define SSBIT5  (1<<5)
#define SSBIT11 (1<<11)
#define SSBIT15 (1<<15)

//adair
#define TP_OF_RN       (1) 
#define TP_OF_YUDA	 (2)
#define TP_OF_HLT     (3) 

static int ito_test_i2c_read(u8 addr, u8* read_data, u16 size)//adair
{
   	int rc;

	struct i2c_msg msgs[] =
    {
		{
			.addr = addr,
			.flags = I2C_M_RD,
			.len = size,
			.buf = read_data,
		},
	};

	rc = i2c_transfer(this_data->client->adapter, msgs, 1);
	if( rc < 0 )
    {
		ITO_TEST_DEBUG("read iic error rc=%d,addr=%d\n", rc,addr);
	}
	return rc;
}

static int ito_test_i2c_write(u8 addr, u8* data, u16 size)//adair:
{
   	int rc;
	struct i2c_msg msgs[] =
    {
		{
			.addr = addr,
			.flags = 0,
			.len = size,
			.buf = data,
		},
	};
	rc = i2c_transfer(this_data->client->adapter, msgs, 1);
	if( rc < 0 )
    {
		ITO_TEST_DEBUG("write iic error rc=%d,addr=%d\n", rc,addr);
	}
	return rc;
}
static void ito_test_reset(void)//adair:
{
	gpio_set_value(this_data->pdata->reset_gpio, 0);
    ITO_TEST_DEBUG("reset tp\n");
    mdelay (100);
    gpio_set_value(this_data->pdata->reset_gpio, 1);
    mdelay(100);
}
static void ito_test_disable_irq(void)//adair
{
	disable_irq_nosync(this_data->client->irq);
}
static void ito_test_enable_irq(void)//adair:
{
	enable_irq(this_data->client->irq);
}
u8 bSetIIcRateByProc = 0;
static void ito_test_set_iic_rate(u32 iicRate)//adair:
{
	#ifdef CONFIG_I2C_SPRD
        sprd_i2c_ctl_chg_clk(msg21xx_i2c_client->adapter->nr, iicRate);
        mdelay(100);
	#endif
}

static void ito_test_WriteReg( u8 bank, u8 addr, u16 data )
{
    u8 tx_data[5] = {0x10, bank, addr, data & 0xFF, data >> 8};
    ito_test_i2c_write( ITO_TEST_ADDR_REG, &tx_data[0], 5 );
}
static void ito_test_WriteReg8Bit( u8 bank, u8 addr, u8 data )
{
    u8 tx_data[4] = {0x10, bank, addr, data};
    ito_test_i2c_write ( ITO_TEST_ADDR_REG, &tx_data[0], 4 );
}
static unsigned short ito_test_ReadReg( u8 bank, u8 addr )
{
    u8 tx_data[3] = {0x10, bank, addr};
    u8 rx_data[2] = {0};

    ito_test_i2c_write( ITO_TEST_ADDR_REG, &tx_data[0], 3 );
    ito_test_i2c_read ( ITO_TEST_ADDR_REG, &rx_data[0], 2 );
    return ( rx_data[1] << 8 | rx_data[0] );
}
static u32 ito_test_get_TpType(void)
{
    u8 tx_data[3] = {0};
    u8 rx_data[4] = {0};
    u32 Major = 0, Minor = 0;

    ITO_TEST_DEBUG("GetTpType\n");

    tx_data[0] = 0x53;
    tx_data[1] = 0x00;
    tx_data[2] = 0x2A;
    ito_test_i2c_write(ITO_TEST_ADDR_TP, &tx_data[0], 3);
    mdelay(50);
    ito_test_i2c_read(ITO_TEST_ADDR_TP, &rx_data[0], 4);
    Major = (rx_data[1]<<8) + rx_data[0];
    Minor = (rx_data[3]<<8) + rx_data[2];

    ITO_TEST_DEBUG("***TpTypeMajor = %d ***\n", Major);
    ITO_TEST_DEBUG("***TpTypeMinor = %d ***\n", Minor);

    return Major%10;

}
static u32 ito_test_choose_TpType(void)
{
    u32 tpType = 0;
    u8 i = 0;
    open_1 = NULL;
    open_1B = NULL;
    open_2 = NULL;
    open_2B = NULL;
    open_3 = NULL;
    MAP1 = NULL;
    MAP2 = NULL;
    MAP3 = NULL;
    MAP40_1 = NULL;
    MAP40_2 = NULL;
    MAP40_3 = NULL;
    MAP40_4 = NULL;
    MAP41_1 = NULL;
    MAP41_2 = NULL;
    MAP41_3 = NULL;
    MAP41_4 = NULL;
    ito_test_keynum = 0;
    ito_test_dummynum = 0;
    ito_test_trianglenum = 0;
    ito_test_2r = 0;

    for(i=0;i<20;i++)
    {
        tpType = ito_test_get_TpType();
        ITO_TEST_DEBUG("tpType=%d;i=%d;\n",tpType,i);
         if(TP_OF_HLT==tpType || TP_OF_RN==tpType||TP_OF_YUDA==tpType)//adair:
        {
            break;
        }
        else if(i<10)
        {
            mdelay(100);
        }
        else
        {
            ito_test_reset();
        }
    }
    if(TP_OF_RN==tpType)
    {
        open_1 = open_1_RN;
        open_1B = open_1B_RN;
        open_2 = open_2_RN;
        open_2B = open_2B_RN;
        open_3 = open_3_RN;
        MAP1 = MAP1_RN;
        MAP2 = MAP2_RN;
        MAP3 = MAP3_RN;
        MAP40_1 = MAP40_1_RN;
        MAP40_2 = MAP40_2_RN;
        MAP40_3 = MAP40_3_RN;
        MAP40_4 = MAP40_4_RN;
        MAP41_1 = MAP41_1_RN;
        MAP41_2 = MAP41_2_RN;
        MAP41_3 = MAP41_3_RN;
        MAP41_4 = MAP41_4_RN;
        ito_test_keynum = NUM_KEY_RN;
        ito_test_dummynum = NUM_DUMMY_RN;
        ito_test_trianglenum = NUM_SENSOR_RN;
        ito_test_2r = ENABLE_2R_RN;
    }
    else
    {
        tpType = 0;
    }
    return tpType;
}
static void ito_test_EnterSerialDebugMode(void)
{
    u8 data[5];

    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, &data[0], 5);

    data[0] = 0x37;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, &data[0], 1);

    data[0] = 0x35;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, &data[0], 1);

    data[0] = 0x71;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, &data[0], 1);
}
static uint16_t ito_test_get_num( void )
{
    uint16_t    num_of_sensor,i;
    uint16_t 	RegValue1,RegValue2;

    num_of_sensor = 0;

    RegValue1 = ito_test_ReadReg( 0x11, 0x4A);
    ITO_TEST_DEBUG("ito_test_get_num,RegValue1=%d\n",RegValue1);
    if ( ( RegValue1 & SSBIT1) == SSBIT1 )
    {
    	RegValue1 = ito_test_ReadReg( 0x12, 0x0A);
    	RegValue1 = RegValue1 & 0x0F;

    	RegValue2 = ito_test_ReadReg( 0x12, 0x16);
    	RegValue2 = (( RegValue2 >> 1 ) & 0x0F) + 1;

    	num_of_sensor = RegValue1 * RegValue2;
    }
	else
	{
	    for(i=0;i<4;i++)
	    {
	        num_of_sensor+=(ito_test_ReadReg( 0x12, 0x0A)>>(4*i))&0x0F;
	    }
	}
    ITO_TEST_DEBUG("ito_test_get_num,num_of_sensor=%d\n",num_of_sensor);
    return num_of_sensor;
}
static void ito_test_polling( void )
{
    uint16_t    reg_int = 0x0000;
    uint8_t     dbbus_tx_data[5];
    uint8_t     dbbus_rx_data[4];
    uint16_t    reg_value;


    reg_int = 0;

    ito_test_WriteReg( 0x13, 0x0C, SSBIT15 );
    ito_test_WriteReg( 0x12, 0x14, (ito_test_ReadReg(0x12,0x14) | SSBIT0) );

    ITO_TEST_DEBUG("polling start\n");
    while( ( reg_int & SSBIT0 ) == 0x0000 )
    {
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x3D;
        dbbus_tx_data[2] = 0x18;
        ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 3);
        ito_test_i2c_read(ITO_TEST_ADDR_REG,  dbbus_rx_data, 2);
        reg_int = dbbus_rx_data[1];
    }
    ITO_TEST_DEBUG("polling end\n");
    reg_value = ito_test_ReadReg( 0x3D, 0x18 );
    ito_test_WriteReg( 0x3D, 0x18, reg_value & (~SSBIT0) );
}
static uint16_t ito_test_get_data_out( int16_t* s16_raw_data )
{
    uint8_t     i,dbbus_tx_data[8];
    uint16_t    raw_data[48]={0};
    uint16_t    num_of_sensor;
    uint16_t    reg_int;
    uint8_t		dbbus_rx_data[96]={0};

    num_of_sensor = ito_test_get_num();
    if(num_of_sensor>11)
    {
        ITO_TEST_DEBUG("danger,num_of_sensor=%d\n",num_of_sensor);
        return num_of_sensor;
    }

    reg_int = ito_test_ReadReg( 0x3d, REG_INTR_FIQ_MASK<<1 );
    ito_test_WriteReg( 0x3d, REG_INTR_FIQ_MASK<<1, (reg_int & (uint16_t)(~FIQ_E_FRAME_READY_MASK) ) );
    ito_test_polling();
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x13;
    dbbus_tx_data[2] = 0x40;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 3);
    mdelay(20);
    ito_test_i2c_read(ITO_TEST_ADDR_REG, &dbbus_rx_data[0], (num_of_sensor * 2));
    mdelay(100);
    for(i=0;i<num_of_sensor * 2;i++)
    {
        ITO_TEST_DEBUG("dbbus_rx_data[%d]=%d\n",i,dbbus_rx_data[i]);
    }

    reg_int = ito_test_ReadReg( 0x3d, REG_INTR_FIQ_MASK<<1 );
    ito_test_WriteReg( 0x3d, REG_INTR_FIQ_MASK<<1, (reg_int | (uint16_t)FIQ_E_FRAME_READY_MASK ) );


    for( i = 0; i < num_of_sensor; i++ )
    {
        raw_data[i] = ( dbbus_rx_data[ 2 * i + 1] << 8 ) | ( dbbus_rx_data[2 * i] );
        s16_raw_data[i] = ( int16_t )raw_data[i];
    }

    return(num_of_sensor);
}


static void ito_test_send_data_in( uint8_t step )
{
    uint16_t	i;
    uint8_t 	dbbus_tx_data[512];
    uint16_t 	*Type1=NULL;

    ITO_TEST_DEBUG("ito_test_send_data_in step=%d\n",step);
	if( step == 4 )
    {
        Type1 = &open_1[0];
    }
    else if( step == 5 )
    {
        Type1 = &open_2[0];
    }
    else if( step == 6 )
    {
        Type1 = &open_3[0];
    }
    else if( step == 9 )
    {
        Type1 = &open_1B[0];
    }
    else if( step == 10 )
    {
        Type1 = &open_2B[0];
    }

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0x00;
    for( i = 0; i <= 0x3E ; i++ )
    {
        dbbus_tx_data[3+2*i] = Type1[i] & 0xFF;
        dbbus_tx_data[4+2*i] = ( Type1[i] >> 8 ) & 0xFF;
    }
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 3+0x3F*2);

    dbbus_tx_data[2] = 0x7A * 2;
    for( i = 0x7A; i <= 0x7D ; i++ )
    {
        dbbus_tx_data[3+2*(i-0x7A)] = 0;
        dbbus_tx_data[4+2*(i-0x7A)] = 0;
    }
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 3+8);

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x12;

    dbbus_tx_data[2] = 5 * 2;
    dbbus_tx_data[3] = Type1[128+5] & 0xFF;
    dbbus_tx_data[4] = ( Type1[128+5] >> 8 ) & 0xFF;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);

    dbbus_tx_data[2] = 0x0B * 2;
    dbbus_tx_data[3] = Type1[128+0x0B] & 0xFF;
    dbbus_tx_data[4] = ( Type1[128+0x0B] >> 8 ) & 0xFF;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);

    dbbus_tx_data[2] = 0x12 * 2;
    dbbus_tx_data[3] = Type1[128+0x12] & 0xFF;
    dbbus_tx_data[4] = ( Type1[128+0x12] >> 8 ) & 0xFF;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);

    dbbus_tx_data[2] = 0x15 * 2;
    dbbus_tx_data[3] = Type1[128+0x15] & 0xFF;
    dbbus_tx_data[4] = ( Type1[128+0x15] >> 8 ) & 0xFF;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);

     #if 1//for AC mod --showlo ;for can't exit from polling in dataout,e.g p520l,20140911
    dbbus_tx_data[1] = 0x13;
    dbbus_tx_data[2] = 0x12 * 2;
    dbbus_tx_data[3] = 0X30;
    dbbus_tx_data[4] = 0X30;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);        

    
    dbbus_tx_data[2] = 0x14 * 2;
    dbbus_tx_data[3] = 0X30;
    dbbus_tx_data[4] = 0X30;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);     

    
    dbbus_tx_data[1] = 0x12;
    for (i = 0x0D; i <= 0x10;i++ )//for AC noise(++)
    {
        dbbus_tx_data[2] = i * 2;
        dbbus_tx_data[3] = Type1[128+i] & 0xFF;
        dbbus_tx_data[4] = ( Type1[128+i] >> 8 ) & 0xFF;
        ito_test_i2c_write( ITO_TEST_ADDR_REG,  dbbus_tx_data,5 );  
    }

    for (i = 0x16; i <= 0x18; i++)//for AC noise
    {
        dbbus_tx_data[2] = i * 2;
        dbbus_tx_data[3] = Type1[128+i] & 0xFF;
        dbbus_tx_data[4] = ( Type1[128+i] >> 8 ) & 0xFF;
        ito_test_i2c_write( ITO_TEST_ADDR_REG, dbbus_tx_data,5 );  
    }
#endif
}

static void ito_test_set_v( uint8_t Enable, uint8_t Prs)
{
    uint16_t    u16RegValue;


    u16RegValue = ito_test_ReadReg( 0x12, 0x08);
    u16RegValue = u16RegValue & 0xF1;
    if ( Prs == 0 )
    {
    	ito_test_WriteReg( 0x12, 0x08, u16RegValue| 0x0C);
    }
    else if ( Prs == 1 )
    {
    	ito_test_WriteReg( 0x12, 0x08, u16RegValue| 0x0E);
    }
    else
    {
    	ito_test_WriteReg( 0x12, 0x08, u16RegValue| 0x02);
    }

    if ( Enable )
    {
        u16RegValue = ito_test_ReadReg( 0x11, 0x06);
        ito_test_WriteReg( 0x11, 0x06, u16RegValue| 0x03);
    }
    else
    {
        u16RegValue = ito_test_ReadReg( 0x11, 0x06);
        u16RegValue = u16RegValue & 0xFC;
        ito_test_WriteReg( 0x11, 0x06, u16RegValue);
    }

}

static void ito_test_set_c( uint8_t Csub_Step )
{
    uint8_t i;
    uint8_t dbbus_tx_data[MAX_CHNL_NUM+3];
    uint8_t HighLevel_Csub = false;
    uint8_t Csub_new;

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0x84;
    for( i = 0; i < MAX_CHNL_NUM; i++ )
    {
		Csub_new = Csub_Step;
        HighLevel_Csub = false;
        if( Csub_new > 0x1F )
        {
            Csub_new = Csub_new - 0x14;
            HighLevel_Csub = true;
        }

        dbbus_tx_data[3+i] =    Csub_new & 0x1F;
        if( HighLevel_Csub == true )
        {
            dbbus_tx_data[3+i] |= SSBIT5;
        }
    }
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, MAX_CHNL_NUM+3);

    dbbus_tx_data[2] = 0xB4;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, MAX_CHNL_NUM+3);
}

static void ito_test_sw( void )
{
    ito_test_WriteReg( 0x11, 0x00, 0xFFFF );
    ito_test_WriteReg( 0x11, 0x00, 0x0000 );
    mdelay( 50 );
}

void Disable_noise_detect( void )
{
    ito_test_WriteReg8Bit( 0x13, 0x02, (ito_test_ReadReg(0x13,0x01) & ~(SSBIT2|SSBIT0|SSBIT1 ) ) );
}

static void ito_test_first( uint8_t item_id , int16_t* s16_raw_data)
{
	uint8_t     loop;
	//uint8_t     dbbus_tx_data[9];
	uint8_t     i,j;
    int16_t     s16_raw_data_tmp[48]={0};
	uint8_t     num_of_sensor, num_of_sensor2,total_sensor=0;
	uint16_t	u16RegValue;
    uint8_t 	*pMapping=NULL;


	num_of_sensor = 0;
	num_of_sensor2 = 0;

    ITO_TEST_DEBUG("ito_test_first item_id=%d\n",item_id);
	ito_test_WriteReg( 0x0F, 0xE6, 0x01 );

	ito_test_WriteReg( 0x1E, 0x24, 0x0500 );
	ito_test_WriteReg( 0x1E, 0x2A, 0x0000 );
	ito_test_WriteReg( 0x1E, 0xE6, 0x6E00 );
	ito_test_WriteReg( 0x1E, 0xE8, 0x0071 );

    if ( item_id == 40 )
    {
        pMapping = &MAP1[0];
        total_sensor = ito_test_trianglenum/2;
    }
    else if( item_id == 41 )
    {
        pMapping = &MAP2[0];
        total_sensor = ito_test_trianglenum/2;
    }
    else if( item_id == 42 )
    {
        pMapping = &MAP3[0];
        total_sensor =  ito_test_trianglenum + ito_test_keynum+ ito_test_dummynum;
    }


	loop = 1;
	if ( item_id != 42 )
	{
		if ( ito_test_2r )
		{
			if ( ito_test_trianglenum/2 > 11 )
				loop = 2;
		}
		else
		{
			if ( ( ito_test_trianglenum/2 + ito_test_keynum + ito_test_dummynum ) > 11 )
				loop = 2;
		}
	}
    ITO_TEST_DEBUG("loop=%d\n",loop);
	for ( i = 0; i < loop; i++ )
	{
		if ( i == 0 )
		{
			ito_test_send_data_in( item_id - 36 );
		}
		else
		{
			if ( item_id == 40 )
				ito_test_send_data_in( 9 );
			else
				ito_test_send_data_in( 10 );
		}
             Disable_noise_detect();
		ito_test_set_v(1,0);
		u16RegValue = ito_test_ReadReg( 0x11, 0x0E);
		ito_test_WriteReg( 0x11, 0x0E, u16RegValue | SSBIT11 );

		if ( g_LTP == 1 )
	    	ito_test_set_c( 32 );
		else
	    	ito_test_set_c( 0 );

		ito_test_sw();

		if ( i == 0 )
        {
            num_of_sensor=ito_test_get_data_out(  s16_raw_data_tmp );
            ITO_TEST_DEBUG("num_of_sensor=%d;\n",num_of_sensor);
        }
		else
        {
            num_of_sensor2=ito_test_get_data_out(  &s16_raw_data_tmp[num_of_sensor] );
            ITO_TEST_DEBUG("num_of_sensor=%d;num_of_sensor2=%d\n",num_of_sensor,num_of_sensor2);
        }
	}
    for ( j = 0; j < total_sensor ; j ++ )
	{
		if ( g_LTP == 1 )
			s16_raw_data[pMapping[j]] = s16_raw_data_tmp[j] + 4096;
		else
			s16_raw_data[pMapping[j]] = s16_raw_data_tmp[j];
	}

	return;
}
typedef enum
{
	ITO_TEST_OK = 0,
	ITO_TEST_FAIL,
	ITO_TEST_GET_TP_TYPE_ERROR,
} ITO_TEST_RET;
ITO_TEST_RET ito_test_second (u8 item_id)
{
	u8 i = 0;

	s32  s16_raw_data_jg_tmp1 = 0;
	s32  s16_raw_data_jg_tmp2 = 0;
	s32  s16_raw_data_jg_tmp3 = 0;
	s32  s16_raw_data_jg_tmp4 = 0;

	s32  jg_tmp1_avg_Th_max =0;
	s32  jg_tmp1_avg_Th_min =0;
	s32  jg_tmp2_avg_Th_max =0;
	s32  jg_tmp2_avg_Th_min =0;
	s32  jg_tmp3_avg_Th_max =0;
	s32  jg_tmp3_avg_Th_min =0;
	s32  jg_tmp4_avg_Th_max =0;
	s32  jg_tmp4_avg_Th_min =0;



	s32  Th_fst = 500;  // new threshold, make sure all data not less than 1000.
	u8  Th_Tri = 35;    // non-border threshold
	u8  Th_bor = 45;    // border threshold

	if ( item_id == 40 )
    {
        for (i=0; i<(ito_test_trianglenum/4)-2; i++)
        {
			s16_raw_data_jg_tmp1 += s16_raw_data_1[MAP40_1[i]];  //first region: non-border
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp2 += s16_raw_data_1[MAP40_2[i]];  //first region: border
		}

		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
        {
			s16_raw_data_jg_tmp3 += s16_raw_data_1[MAP40_3[i]];  //second region: non-border
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp4 += s16_raw_data_1[MAP40_4[i]];  //second region: border
		}
    }
    else if( item_id == 41 )
    {
        for (i=0; i<(ito_test_trianglenum/4)-2; i++)
        {
			s16_raw_data_jg_tmp1 += s16_raw_data_2[MAP41_1[i]];  //first region: non-border
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp2 += s16_raw_data_2[MAP41_2[i]];  //first region: border
		}
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
        {
			s16_raw_data_jg_tmp3 += s16_raw_data_2[MAP41_3[i]];  //second region: non-border
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp4 += s16_raw_data_2[MAP41_4[i]];  //second region: border
		}
    }

	    jg_tmp1_avg_Th_max = (s16_raw_data_jg_tmp1 / ((ito_test_trianglenum/4)-2)) * ( 100 + Th_Tri) / 100 ;
	    jg_tmp1_avg_Th_min = (s16_raw_data_jg_tmp1 / ((ito_test_trianglenum/4)-2)) * ( 100 - Th_Tri) / 100 ;
        jg_tmp2_avg_Th_max = (s16_raw_data_jg_tmp2 / 2) * ( 100 + Th_bor) / 100 ;
	    jg_tmp2_avg_Th_min = (s16_raw_data_jg_tmp2 / 2) * ( 100 - Th_bor) / 100 ;
		jg_tmp3_avg_Th_max = (s16_raw_data_jg_tmp3 / ((ito_test_trianglenum/4)-2)) * ( 100 + Th_Tri) / 100 ;
	    jg_tmp3_avg_Th_min = (s16_raw_data_jg_tmp3 / ((ito_test_trianglenum/4)-2)) * ( 100 - Th_Tri) / 100 ;
        jg_tmp4_avg_Th_max = (s16_raw_data_jg_tmp4 / 2) * ( 100 + Th_bor) / 100 ;
	    jg_tmp4_avg_Th_min = (s16_raw_data_jg_tmp4 / 2) * ( 100 - Th_bor) / 100 ;


        ITO_TEST_DEBUG("sum1=%d;max1=%d;min1=%d;sum2=%d;max2=%d;min2=%d;sum3=%d;max3=%d;min3=%d;sum4=%d;max4=%d;min4=%d;\n",s16_raw_data_jg_tmp1,jg_tmp1_avg_Th_max,jg_tmp1_avg_Th_min,s16_raw_data_jg_tmp2,jg_tmp2_avg_Th_max,jg_tmp2_avg_Th_min,s16_raw_data_jg_tmp3,jg_tmp3_avg_Th_max,jg_tmp3_avg_Th_min,s16_raw_data_jg_tmp4,jg_tmp4_avg_Th_max,jg_tmp4_avg_Th_min);

	if ( item_id == 40 )
	{
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
	    {
			if (s16_raw_data_1[MAP40_1[i]] > jg_tmp1_avg_Th_max || s16_raw_data_1[MAP40_1[i]] < jg_tmp1_avg_Th_min || s16_raw_data_1[MAP40_1[i]] < Th_fst)
				return ITO_TEST_FAIL;
		}

		for (i=0; i<2; i++)
	    {
			if (s16_raw_data_1[MAP40_2[i]] > jg_tmp2_avg_Th_max || s16_raw_data_1[MAP40_2[i]] < jg_tmp2_avg_Th_min || s16_raw_data_1[MAP40_2[i]] < Th_fst)
				return ITO_TEST_FAIL;
		}

		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
	    {
			if (s16_raw_data_1[MAP40_3[i]] > jg_tmp3_avg_Th_max || s16_raw_data_1[MAP40_3[i]] < jg_tmp3_avg_Th_min || s16_raw_data_1[MAP40_3[i]] < Th_fst)
				return ITO_TEST_FAIL;
		}

		for (i=0; i<2; i++)
	    {
			if (s16_raw_data_1[MAP40_4[i]] > jg_tmp4_avg_Th_max || s16_raw_data_1[MAP40_4[i]] < jg_tmp4_avg_Th_min || s16_raw_data_1[MAP40_4[i]] < Th_fst)
				return ITO_TEST_FAIL;
		}
	}

	if ( item_id == 41 )
	{
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
	    {
			if (s16_raw_data_2[MAP41_1[i]] > jg_tmp1_avg_Th_max || s16_raw_data_2[MAP41_1[i]] < jg_tmp1_avg_Th_min || s16_raw_data_2[MAP41_1[i]] < Th_fst)
				return ITO_TEST_FAIL;
		}

		for (i=0; i<2; i++)
	    {
			if (s16_raw_data_2[MAP41_2[i]] > jg_tmp2_avg_Th_max || s16_raw_data_2[MAP41_2[i]] < jg_tmp2_avg_Th_min || s16_raw_data_2[MAP41_2[i]] < Th_fst)
				return ITO_TEST_FAIL;
		}

		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
	    {
			if (s16_raw_data_2[MAP41_3[i]] > jg_tmp3_avg_Th_max || s16_raw_data_2[MAP41_3[i]] < jg_tmp3_avg_Th_min || s16_raw_data_2[MAP41_3[i]] < Th_fst)
				return ITO_TEST_FAIL;
		}

		for (i=0; i<2; i++)
	    {
			if (s16_raw_data_2[MAP41_4[i]] > jg_tmp4_avg_Th_max || s16_raw_data_2[MAP41_4[i]] < jg_tmp4_avg_Th_min || s16_raw_data_2[MAP41_4[i]] < Th_fst)
				return ITO_TEST_FAIL;
		}


	}

	return ITO_TEST_OK;

}
#if 0 //defined CONFIG_ARCH_MSM8610_W7_YULONG_CP7236
extern void qcom_i2c_clk_ctl_change(struct i2c_adapter *adap, int i2cRate);
#endif
static ITO_TEST_RET ito_test_interface(void)
{
    ITO_TEST_RET ret = ITO_TEST_OK;
    uint16_t i = 0;
 
       //printk(KERN_ERR "ito_test_interface111111\n");
	#if 0 //defined CONFIG_ARCH_MSM8610_W7_YULONG_CP7236
       qcom_i2c_clk_ctl_change(this_data->client->adapter, 50000);
	#endif
	//printk(KERN_ERR "ito_test_interface22222\n");

	ITO_TEST_DEBUG("start\n");
	ito_test_disable_irq();
	ito_test_reset();
	if(!ito_test_choose_TpType())
	{
	    ITO_TEST_DEBUG("choose tpType fail\n");
	    ret = ITO_TEST_GET_TP_TYPE_ERROR;
	    goto ITO_TEST_END;
	}
	ito_test_EnterSerialDebugMode();
	mdelay(100);
	ITO_TEST_DEBUG("EnterSerialDebugMode\n");
	ito_test_WriteReg8Bit ( 0x0F, 0xE6, 0x01 );
	ito_test_WriteReg ( 0x3C, 0x60, 0xAA55 );
	ITO_TEST_DEBUG("stop mcu and disable watchdog V.005\n");
	mdelay(50);

	for(i = 0;i < 48;i++)
	{
		s16_raw_data_1[i] = 0;
		s16_raw_data_2[i] = 0;
		s16_raw_data_3[i] = 0;
	}

    ito_test_first(40, s16_raw_data_1);
    ITO_TEST_DEBUG("40 get s16_raw_data_1\n");
    if(ITO_TEST_FAIL==ito_test_second(40))
    {
        ret = ITO_TEST_FAIL;
        goto ITO_TEST_END;
    }

    ito_test_first(41, s16_raw_data_2);
    ITO_TEST_DEBUG("41 get s16_raw_data_2\n");
    if(ITO_TEST_FAIL==ito_test_second(41))
    {
        ret = ITO_TEST_FAIL;
        goto ITO_TEST_END;
    }

    ito_test_first(42, s16_raw_data_3);
    ITO_TEST_DEBUG("42 get s16_raw_data_3\n");
    

    ITO_TEST_END:
	ito_test_reset();
	ito_test_enable_irq();
#if 0 //defined CONFIG_ARCH_MSM8610_W7_YULONG_CP7236
    qcom_i2c_clk_ctl_change(this_data->client->adapter, 100000);
#endif

    ITO_TEST_DEBUG("end\n");
    return ret;
}
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#define ITO_TEST_AUTHORITY 0777
static struct proc_dir_entry *msg_ito_test = NULL;
static struct proc_dir_entry *debug = NULL;
static struct proc_dir_entry *ito_test = NULL;
static struct proc_dir_entry *debug_on_off = NULL;
static struct proc_dir_entry *set_iic_rate = NULL;
#define PROC_MSG_ITO_TESE      "msg-ito-test"
#define PROC_ITO_TEST_DEBUG      "debug"
#define PROC_ITO_TEST_DEBUG_ON_OFF     "debug-on-off"
#define PROC_ITO_TEST_SET_IIC_RATE     "set-iic-rate"
ITO_TEST_RET g_ito_test_ret = ITO_TEST_OK;
#define PROC_ITO_TEST      "ito_test"
static int ito_test_proc_read(struct file *file, char __user * buffer, size_t count,
		       loff_t * ppos) 
{
	int ret;
	int err;

	ret = ito_test_interface();
	printk("ito_test_interface()=%d\n",ret);
	err = copy_to_user(buffer, &ret, sizeof(int));
	if (err)
		goto Efault;
	return err;
Efault:
	err = -EFAULT;
	return err;
}

#if 0
static int ito_test_proc_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
	char *buf;

	if (count < 1)
	return -EINVAL;

	buf = kmalloc(count, GFP_KERNEL);
	if (!buf)
	return -ENOMEM;

	if (copy_from_user(buf, buffer, count)) {
		kfree(buf);
		return -EFAULT;
	}

	if (buf[0] == '0') {
		g_ito_test_ret=0;
	} else if (buf[0] == '1') {
		g_ito_test_ret=1;
	} else {
		kfree(buf);
		return -EINVAL;
	}

	kfree(buf);
	return count;
}
#endif
//static int ito_test_proc_read_debug(char *page, char **start, off_t off, int count, int *eof, void *data)
static int ito_test_proc_read_debug(struct file *file, char __user * buffer, size_t count,
		       loff_t * ppos) 
{
    int cnt= 0;

    g_ito_test_ret = ito_test_interface();
    if(ITO_TEST_OK==g_ito_test_ret)
    {
        ITO_TEST_DEBUG_MUST("ITO_TEST_OK");
    }
    else if(ITO_TEST_FAIL==g_ito_test_ret)
    {
        ITO_TEST_DEBUG_MUST("ITO_TEST_FAIL");
    }
    else if(ITO_TEST_GET_TP_TYPE_ERROR==g_ito_test_ret)
    {
        ITO_TEST_DEBUG_MUST("ITO_TEST_GET_TP_TYPE_ERROR");
    }

    return cnt;
}

static int ito_test_proc_write_debug(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    u16 i = 0;
    mdelay(5);
    ITO_TEST_DEBUG_MUST("ito_test_ret = %d",g_ito_test_ret);
    mdelay(5);
    for(i=0;i<48;i++)
    {
        ITO_TEST_DEBUG_MUST("data_1[%d]=%d;\n",i,s16_raw_data_1[i]);
    }
    mdelay(5);
    for(i=0;i<48;i++)
    {
        ITO_TEST_DEBUG_MUST("data_2[%d]=%d;\n",i,s16_raw_data_2[i]);
    }
    mdelay(5);
    for(i=0;i<48;i++)
    {
        ITO_TEST_DEBUG_MUST("data_3[%d]=%d;\n",i,s16_raw_data_3[i]);
    }
    mdelay(5);
    return count;
}
static int ito_test_proc_read_debug_on_off(struct file *file, char __user * buffer, size_t count,
		       loff_t * ppos) 
{
    int cnt= 0;

    bItoTestDebug = 1;
    ITO_TEST_DEBUG_MUST("on debug bItoTestDebug = %d",bItoTestDebug);

    return cnt;
}

static int ito_test_proc_write_debug_on_off(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    bItoTestDebug = 0;
    ITO_TEST_DEBUG_MUST("off debug bItoTestDebug = %d",bItoTestDebug);
    return count;
}

static int ito_test_proc_read_set_iic_rate(struct file *file, char __user * buffer, size_t count,
		       loff_t * ppos) 
{
    int cnt= 0;

    bSetIIcRateByProc = 1;
    ito_test_set_iic_rate(400000);
    ITO_TEST_DEBUG_MUST("on set iic rate bSetIIcRateByProc = %d",bSetIIcRateByProc);

    return cnt;
}

static int ito_test_proc_write_set_iic_rate(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    bSetIIcRateByProc = 0;
    ito_test_set_iic_rate(100000);
    ITO_TEST_DEBUG_MUST("off set iic rate bSetIIcRateByProc = %d",bSetIIcRateByProc);
    return count;
}

static const struct file_operations ito_debug_proc_fops = {
	.owner		= THIS_MODULE,
	.read		= ito_test_proc_read_debug,
	.write		= ito_test_proc_write_debug,
};
static const struct file_operations ito_debugOnOff_proc_fops = {
	.owner		= THIS_MODULE,
	.read		= ito_test_proc_read_debug_on_off,
	.write		= ito_test_proc_write_debug_on_off,
};
static const struct file_operations ito_test_proc_fops = {
	.owner		= THIS_MODULE,
	.read		= ito_test_proc_read,
	//.write		= ito_test_proc_write,
};
static const struct file_operations ito_set_iic_rate_proc_fops = {
	.owner		= THIS_MODULE,
	.read		= ito_test_proc_read_set_iic_rate,
	.write		= ito_test_proc_write_set_iic_rate,
};
static int ito_test_create_entry(void)
{
    msg_ito_test = proc_mkdir(PROC_MSG_ITO_TESE, NULL);
    debug = proc_create(PROC_ITO_TEST_DEBUG, ITO_TEST_AUTHORITY, msg_ito_test,&ito_debug_proc_fops);
    debug_on_off= proc_create(PROC_ITO_TEST_DEBUG_ON_OFF, ITO_TEST_AUTHORITY,msg_ito_test,&ito_debugOnOff_proc_fops);
    set_iic_rate= proc_create(PROC_ITO_TEST_SET_IIC_RATE, ITO_TEST_AUTHORITY, msg_ito_test,&ito_set_iic_rate_proc_fops);
    ito_test = proc_create(PROC_ITO_TEST, ITO_TEST_AUTHORITY, msg_ito_test,&ito_test_proc_fops);

    if (!msg_ito_test)
		return -ENOMEM;
	
    if (NULL==debug)
    {
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST DEBUG failed\n");
	  goto fail1;
    }
    
    //Added by zhanghaibin
    if (NULL==ito_test)
    {
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST DEBUG failed\n");
	 goto fail2;
    }
    
    if (NULL==debug_on_off)
    {
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST ON OFF failed\n");
	  goto fail3;
    }
    else
    
    if (NULL==set_iic_rate)
    {
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST SET IIC RATE failed\n");
	  goto fail4;
    }
    return 0;
	
 fail4: remove_proc_entry(PROC_ITO_TEST_DEBUG_ON_OFF, msg_ito_test);
 fail3: remove_proc_entry(PROC_ITO_TEST, msg_ito_test);
 fail2: remove_proc_entry(PROC_ITO_TEST_DEBUG, msg_ito_test);
 fail1: remove_proc_entry(PROC_MSG_ITO_TESE, NULL);
    return -ENOMEM;
}

static void ito_test_remove_entry(void)
{
	remove_proc_entry(PROC_ITO_TEST_DEBUG, msg_ito_test);
	remove_proc_entry(PROC_ITO_TEST_DEBUG_ON_OFF, msg_ito_test);
	remove_proc_entry(PROC_ITO_TEST_SET_IIC_RATE, msg_ito_test);
	remove_proc_entry(PROC_ITO_TEST, msg_ito_test);
	remove_proc_entry(PROC_MSG_ITO_TESE, 0);
}

#endif

#ifdef CONFIG_MSTAR_CTP_ITO_TEST
///////////////////////////////////////////////////////////////////////////
//const
#define BIT0  0x0001
#define BIT1  0x0002
#define BIT2  0x0004
#define BIT3  0x0008
#define BIT4  0x0010
#define BIT5  0x0020
#define BIT6  0x0040
#define BIT7  0x0080
#define BIT8  0x0100
#define BIT9  0x0200
#define BIT10  0x0400
#define BIT11  0x0800
#define BIT12  0x1000
#define BIT13  0x2000
#define BIT14  0x4000
#define BIT15  0x8000

#define MAX_CHANNEL_DRV  28
#define MAX_CHANNEL_SEN  14
#define MAX_MUTUAL_NUM  (MAX_CHANNEL_DRV * MAX_CHANNEL_SEN)
#define FIR_THRES    6553
#define FIR_RATIO    55 //25 Modified by zhanghaibin,20141017
s32 deltaC[MAX_MUTUAL_NUM] = {0};
s32 g_result[MAX_MUTUAL_NUM] = {0};
uint8_t g_mode[MAX_MUTUAL_NUM] = {0};

uint16_t First(void)
{
    uint16_t checkState = 0;
    checkState = _msg_ReadReg(0x3C, 0xDE);
    return checkState;
}

void Second(void)
{
    _msg_WriteReg(0x0F, 0xE6, 0x0001);
    mdelay(10);
}

void Third(void)
{
    uint16_t u16Temp;
    u16Temp = _msg_ReadReg(0x11, 0x4A);
    u16Temp |= BIT0;
    _msg_WriteReg(0x11, 0x4A, u16Temp);
    u16Temp = _msg_ReadReg(0x11, 0x16);
    u16Temp |= (BIT2 | BIT0);
    _msg_WriteReg(0x11, 0x16, u16Temp);
}

uint16_t Second_1(void)
{
    uint16_t s1 = 0;
    uint16_t u16RegData = 0;

    u16RegData = _msg_ReadReg(0x10, 0x2E);
    s1 = u16RegData & 0x000F;

    return s1;
}

uint16_t Second_2(void)
{
    uint16_t s2 = 0;
    uint16_t u16RegData = 0;

    u16RegData = _msg_ReadReg(0x12, 0x16);
    s2 = ((u16RegData & 0xFF00) >> 8) + 1;

    return s2;
}

#define DUMMY_CNT   3
void Forth(uint8_t *mode)
{
    uint16_t i,j;
    uint16_t f2;
    uint16_t f3;
    uint16_t f1;
    uint16_t ana4Num;
    uint8_t u8dataAna4[DUMMY_CNT];
    uint8_t u8dataAna3[DUMMY_CNT];
    uint8_t u8dataAna41[200];
    uint8_t u8dataAna31[0xC0];
    uint8_t mode_Temp[MAX_MUTUAL_NUM];

    f1= MAX_MUTUAL_NUM;
    f2= Second_1();
    f3= Second_2();
    ana4Num = MAX_MUTUAL_NUM - 0xC0;

    if (ana4Num > 0)
    {
	mdelay(100);
	for (i = 0; i < ana4Num; i++)
	{
	    u8dataAna41[i] = 0;
	}
	u8dataAna3[0] = 0x10;
	u8dataAna3[1] = 0x15;
	u8dataAna3[2] = 0x00;
	_msg_WriteI2CSeq(FW_ADDR_MSG, &u8dataAna3[0], 3);
	_msg_ReadI2CSeq(FW_ADDR_MSG, &u8dataAna41[0], 200);

        f1 -= (uint16_t)ana4Num;
    }

    for (i = 0; i < f1; i++)
    {
        u8dataAna31[i] = 0;
    }

    mdelay(100);
    u8dataAna4[0] = 0x10;
    u8dataAna4[1] = 0x10;
    u8dataAna4[2] = 0x40;
    _msg_WriteI2CSeq(FW_ADDR_MSG, &u8dataAna4[0], 3);
    _msg_ReadI2CSeq(FW_ADDR_MSG, &u8dataAna31[0], 192);

        for (i = 0; i < 0xc0; i++)
        {
            mode_Temp[i] = u8dataAna31[i];
        }
        for (i = 0xc0; i <(0xc0 + ana4Num); i++)
        {
            mode_Temp[i] = u8dataAna41[i - 0xc0];
        }
        for(i = 0;i < f3;i++)
        {
            for(j = 0;j < f2;j++)
            {
                mode[j * f3 + i] = mode_Temp[i * MAX_CHANNEL_SEN + j];
            }
        }
}

void Fifth(void)
{
    uint16_t u16Temp;
    u16Temp = _msg_ReadReg(0x13, 0x02);
    u16Temp &= (~(BIT2 | BIT1 | BIT0));
    _msg_WriteReg(0x13, 0x02, u16Temp);
}

void Sixth(void)
{
    _msg_WriteReg(0x11, 0x00, 0xFFFF);
    _msg_WriteReg(0x11, 0x00, 0x0000);
    mdelay(100);
}

void Second_4(void)
{
    uint16_t u16Temp;
    _msg_WriteReg(0x13, 0x0C, BIT15);
    u16Temp = _msg_ReadReg(0x12, 0x14);
    u16Temp |= BIT0;
    _msg_WriteReg(0x12, 0x14, u16Temp);
}

void Second_3(uint16_t resultData[][MAX_CHANNEL_DRV], uint16_t a, uint16_t b)
{
    uint16_t u16RegData;
    uint16_t i,j;
    uint16_t u16Temp;
    uint8_t tx_data[3];
    uint8_t u8ShotData1[190];
    uint8_t u8ShotData2[255];//594>0xff

    u16Temp = _msg_ReadReg(0x3D, 0x08);
    u16Temp &= (~(BIT8 | BIT4 ));
    _msg_WriteReg(0x3D, 0x08, u16Temp);

    Second_4();
    u16RegData = 0;
    while (0x0000 == (u16RegData & BIT8))
    {
        u16RegData = _msg_ReadReg(0x3D, 0x18);
    }

    for(i = 0; i < 190; i++)
    {
        u8ShotData1[i] = 0;
    }
    for(i = 0; i < 255; i++)
    {
        u8ShotData2[i] = 0;
    }

    mdelay(100);
    tx_data[0] = 0x10;
    tx_data[1] = 0x13;
    tx_data[2] = 0x42;
    _msg_WriteI2CSeq(FW_ADDR_MSG, &tx_data[0], 3);
    _msg_ReadI2CSeq(FW_ADDR_MSG, &u8ShotData1[0], 190);

    mdelay(100);
    tx_data[0] = 0x10;
    tx_data[1] = 0x20;
    tx_data[2] = 0x00;
    _msg_WriteI2CSeq(FW_ADDR_MSG, &tx_data[0], 3);
    _msg_ReadI2CSeq(FW_ADDR_MSG, &u8ShotData2[0], 255);

    for (j = 0; j < a; j++)
    {
        for (i = 0; i < b; i++)
        {
            if ((j <= 5) || ((j == 6) && (i <= 10)))
            {
                u16RegData = (uint16_t)(u8ShotData1[(j * 14 + i) * 2] | u8ShotData1[(j * 14 + i) * 2 + 1] << 8);
                resultData[i][ j] = (short)u16RegData;
            }
            else
            {
                if ((j == 6) && (i > 10))
                {
                    u16RegData = (uint16_t)(u8ShotData2[((j - 6) * 14 + (i - 11)) * 2] | u8ShotData2[((j - 6) * 14 + (i - 11)) * 2 + 1] << 8);
                    resultData[i][ j] = (short)u16RegData;
                }
                else
                {
                    u16RegData = (uint16_t)(u8ShotData2[6 + ((j - 7) * 14 + i) * 2] | u8ShotData2[6 + ((j - 7) * 14 + i) * 2 + 1] << 8);
                    resultData[i][ j] = (short)u16RegData;
                }
            }
        }
    }

    u16Temp = _msg_ReadReg(0x3D, 0x08);
    u16Temp |= (BIT8 | BIT4 );
    _msg_WriteReg(0x3D, 0x08, u16Temp);
}

void Seventh(s32 *target)
{
    s16 sTmp;
    uint16_t rawData[MAX_CHANNEL_SEN][MAX_CHANNEL_DRV];
    uint16_t i, j;
    uint16_t t1 = 0, t2 = 0, shift = 0;

    t2 = Second_1();
    t1 = Second_2();
    Second_3(rawData, t1, t2);

    for (i = 0; i < t2; i++)
    {
        for (j = 0; j < t1; j++)
        {
            shift = (uint16_t)(i * t1 + j);
            sTmp = (s16)rawData[i][j];
            target[shift] = sTmp;
        }
    }
}

/*Modified by zhanghaibin for TP ito-test gets fail sometimes,20140714.*/
int ITOTest_m(void)
{
    int ret = -1;
    s32 s16Prev, s16Delta;
    uint16_t i = 0, j = 0, k = 0;
    uint16_t checkState = 0;
    uint16_t t1 = 0, t2 = 0;

    _msg_disable_irq();

    for (k = 0; ((k < 3) && (ret == -1)); k++)
    {
        ret = 0;
        t1=0;
        t2=0;

        for (j = 0; ((j < 10) && (t1 <= 2)); j++)
        {
            _msg_resetHW();
            _msg_EnterSerialDebugMode();
            mdelay(300);

            checkState=0;
            for (i = 0; ((i < 20) && (checkState == 0 || checkState == 1)); i++)
            {
                checkState = First();
                mdelay(100);
            }

            if (checkState == 0 || checkState == 1)
            {
                ret = -2;
                goto ito_end;
            }
            printk("statecnt=%d\n", i);

            Second();
            t2 = Second_1();
            t1 = Second_2();
        }

        if (t1 <= 2)
        {
            ret = -3;
            goto ito_end;
        }
        printk("sensorcnt=%d\n", j);

        Third();
        Forth(g_mode);
        Fifth();
        Sixth();
        Seventh(deltaC);

        for (i = 0; i < t2; i++)
        {
            printk("\nSen[%02d]\t", i);
            for (j = 0; j < t1; j++)
            {
    		g_result[i * t1 + j] = (4464*g_mode[i * t1 + j]-deltaC[i * t1 + j]);
    		printk("%d\t", g_result[i * t1 + j]);
            }
        }
        printk("\n\n\n");
        for (j = 0; (j < t1) && (ret == 0); j++)
        {
            for (i = 0; (i < t2) && (ret == 0); i++)
            {
                if (g_result[i * t1 + j] < FIR_THRES)
                {
                    ret = -1;
                	  printk("\nsen%d,drv%d,MIN_Thres=%d\t", i,j,g_result[i * t1 + j]);
                }

                if (i > 0)
                {
                    s16Delta = g_result[i * t1 + j] > s16Prev ? (g_result[i * t1 + j]-s16Prev) : (s16Prev-g_result[i * t1 + j]);
                    if (s16Delta > s16Prev*FIR_RATIO/100)
                    {
                        ret = -1;
                        printk("\nsen%d,drv%d,MAX_Ratio=%d,%d\t", i,j,s16Delta,s16Prev);
                    }
                }
                s16Prev = g_result[i * t1 + j];
            }
		//compare tail data and head data of col,zhanghaibin,20141017
		s16Delta=g_result[j] > g_result[(t2-1)*t1+j] ? (g_result[j]-g_result[(t2-1)*t1+j]) : (g_result[(t2-1)*t1+j]-g_result[j]);
		printk("\ndrv%d,MAX_Ratio=%d,%d,%d\t", j,s16Delta,g_result[(t2-1)*t1+j],g_result[j]);
		if (s16Delta>g_result[(t2-1)*t1+j] *FIR_RATIO/100)
		{
			ret=-1;
			printk("\n fail! drv%d,MAX_Ratio=%d,%d,%d\t", j,s16Delta,g_result[(t2-1)*t1+j],g_result[j]);
		}
        }
        printk("ret=%d k=%d\n", ret,k);
    }

ito_end:
    _msg_ExitSerialDebugMode();
    _msg_resetHW();
    mdelay(300);
    _msg_enable_irq();
    return ret;
}

#include <linux/proc_fs.h>
#define ITO_TEST_M_AUTHORITY 0777 
static struct proc_dir_entry *msg_ito_test_m = NULL;
static struct proc_dir_entry *debug_m = NULL;
static struct proc_dir_entry *ito_test_m = NULL;
#define PROC_MSG_ITO_TESE_M      "msg-ito-test-m"
#define PROC_ITO_TEST_DEBUG      "debug"
#define PROC_ITO_TEST      "ito_test"

struct class *msg2xxx_ito_test_class;
struct device *msg2xxx_ito_test_cmd_dev;
static int ito_test_m_proc_read_debug(struct file *file, char __user * buffer, size_t count,
		       loff_t * ppos) 
{
    int cnt= 0;
    
    cnt=ITOTest_m();
    printk("ITOTest_m=%d\n",cnt);
	
    return cnt;
}
static int ito_test_proc_read_m(struct file *file, char __user * buffer, size_t count,
		       loff_t * ppos)
{  
	int ret;
	int err;

	ret = ITOTest_m();
	printk("ITOTest_m=%d\n",ret);
	err = copy_to_user(buffer, &ret, sizeof(int));
	if (err)
		goto Efault;
	return err;
Efault:
	err = -EFAULT;
	return err;
	//return sprintf(buffer, "%d\n", ret);
}

static const struct file_operations ito_debug_proc_fops_m = {
	.owner		= THIS_MODULE,
	.read		= ito_test_m_proc_read_debug,
};
static const struct file_operations ito_test_proc_fops_m = {
	.owner		= THIS_MODULE,
	.read		= ito_test_proc_read_m,
};

static int mito_test_create_entry(void)
{
    msg_ito_test_m = proc_mkdir(PROC_MSG_ITO_TESE_M, NULL);
    debug_m = proc_create(PROC_ITO_TEST_DEBUG, ITO_TEST_M_AUTHORITY, msg_ito_test_m,&ito_debug_proc_fops_m);
    ito_test_m = proc_create(PROC_ITO_TEST, ITO_TEST_M_AUTHORITY, msg_ito_test_m,&ito_test_proc_fops_m);

    if (!msg_ito_test_m)
		return -ENOMEM;

    if (NULL==debug_m) 
    {
        printk("create_proc_entry ITO TEST DEBUG failed\n");
	 goto fail1;
    } 

    //Added by zhanghaibin
    if (NULL==ito_test_m) 
    {
        printk("create_proc_entry ITO TEST failed\n");
	 goto fail2;
    } 
    return 0;
 fail2: remove_proc_entry(PROC_ITO_TEST_DEBUG, msg_ito_test_m);
 fail1: remove_proc_entry(PROC_MSG_ITO_TESE_M, NULL);
    return -ENOMEM;
}


static ssize_t firmware_ito_test_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
 	int ret=-1;

	if (g_ic_type == 2)
	{
		ret = ito_test_interface();
	}
	else if (g_ic_type == 3)
	{
		ret = ITOTest_m();
	}
	else
	{
		printk("ic type error\n");
	}
	
	printk("ITOTest result=%d\n",ret);
	
	return sprintf(buf, "%d\n", ret);
}
/*Added by zhanghaibin to avoid warning when probe,20141107*/
ssize_t firmware_ito_test_store(struct device *pDevice, struct device_attribute *pAttr, const char *pBuf, size_t nSize)
{
    pr_err("*** %s() ***\n", __func__);

    return nSize;
}

static DEVICE_ATTR(ito_test, ITO_TEST_M_AUTHORITY, firmware_ito_test_show, firmware_ito_test_store);

static void _msg_create_file_for_ito_test(void)
{  
    msg2xxx_ito_test_class = class_create(THIS_MODULE, "msg-ito-test-m");
    if (IS_ERR(msg2xxx_ito_test_class))
        pr_err("Failed to create class(ito_test)!\n");
    msg2xxx_ito_test_cmd_dev = device_create(msg2xxx_ito_test_class, NULL, 0, NULL, "device");
    if (IS_ERR(msg2xxx_ito_test_cmd_dev))
        pr_err("Failed to create device(msg2xxx_firmware_cmd_dev)!\n");

    // ito-test
    if (device_create_file(msg2xxx_ito_test_cmd_dev, &dev_attr_ito_test) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_ito_test.attr.name);
   
    dev_set_drvdata(msg2xxx_ito_test_cmd_dev, NULL);
}

#endif

#ifdef TP_PRINT
#include <linux/proc_fs.h>

#define TP_PRINT_AUTHORITY 0777
#define PROC_TP_PRINT_MSG   "msgtpp"
#define PROC_TP_PRINT_NODE  "tpp"

static struct proc_dir_entry *msg_tpp = NULL;
static struct proc_dir_entry *tpp = NULL;
static u16 InfoAddr = 0x0F, PoolAddr = 0x10, TransLen = 256;
static u16 cnt, head, tail;
static u8 row, units;
static u8 sleep1 = 10, sleep2 = 10;

static int tp_print_proc_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    u16 i, j;
    u16 left, offset;
    u8 dbbus_tx_data[3] = {0};
    u8 u8Data;
    s16 s16Data;
    s32 s32Data;
    char *buf = NULL;
    int rc = 0;

    if ((InfoAddr != 0x0F) && (PoolAddr != 0x10))
    {
        printk("tpp: \n");

        offset = 0;
        left = cnt*row*units;
        buf = kmalloc(left, GFP_KERNEL);
        if (buf != NULL)
        {
            while (left > 0)
            {
                dbbus_tx_data[0] = 0x53;
                dbbus_tx_data[1] = ((PoolAddr + offset) >> 8) & 0xFF;
                dbbus_tx_data[2] = (PoolAddr + offset) & 0xFF;
                mutex_lock(&tpp_lock);
                if (sleep1 > 0)
                    msleep(sleep1);
                _msg_WriteI2CSeq(FW_ADDR_MSG_TP, &dbbus_tx_data[0], 3);
                _msg_ReadI2CSeq(FW_ADDR_MSG_TP, &buf[offset], left > TransLen ? TransLen : left);
                if (sleep2 > 0)
                    msleep(sleep2);
                mutex_unlock(&tpp_lock);

                if (left > TransLen)
                {
                    left -= TransLen;
                    offset += TransLen;
                }
                else
                {
                    left = 0;
                }
            }

            i = head;
            while ((cnt*row*units > 0) && (rc >= 0))
            {
                if (i < cnt)
                {
                    if (i == tail)
                    {
                        rc = -1;
                    }
                }
                else
                {
                    if (tail >= cnt)
                    {
                        rc = -1;
                    }
                    else
                    {
                        i = 0;
                    }
                }

                if (rc >= 0)
                {
                    printk("tpp: ");
                    for (j = 0; j < row; j++)
                    {
                        if (units == 1)
                        {
                            u8Data = buf[i*row*units + j*units];
                            printk("%d\t", u8Data);
                        }
                        else if (units == 2)
                        {
                            s16Data = buf[i*row*units + j*units] + (buf[i*row*units + j*units + 1] << 8);
                            printk("%d\t", s16Data);
                        }
                        else if (units == 4)
                        {
                            s32Data = buf[i*row*units + j*units] + (buf[i*row*units + j*units + 1] << 8) + (buf[i*row*units + j*units + 2] << 16) + (buf[i*row*units + j*units + 3] << 24);
                            printk("%d\t", s32Data);
                        }
                    }
                    printk("\n");
                    i++;
                }
            }

            kfree(buf);
        }
    }

    *eof = 1;
    return 0;
}

static void tp_print_create_entry(void)
{
    u8 dbbus_tx_data[3] = {0};
    u8 dbbus_rx_data[8] = {0};

    dbbus_tx_data[0] = 0x53;
    dbbus_tx_data[1] = 0x00;
    dbbus_tx_data[2] = 0x56;
    _msg_WriteI2CSeq(FW_ADDR_MSG_TP, &dbbus_tx_data[0], 3);
    _msg_ReadI2CSeq(FW_ADDR_MSG_TP, &dbbus_rx_data[0], 4);
    InfoAddr = (dbbus_rx_data[1]<<8) + dbbus_rx_data[0];
    PoolAddr = (dbbus_rx_data[3]<<8) + dbbus_rx_data[2];
    printk("InfoAddr=0x%X\n", InfoAddr);
    printk("PoolAddr=0x%X\n", PoolAddr);

    if ((InfoAddr != 0x0F) && (PoolAddr != 0x10))
    {
        g_bTpp = 1;
        mutex_init(&tpp_lock);
        msg_tpp = proc_mkdir(PROC_TP_PRINT_MSG, NULL);
        tpp = create_proc_entry(PROC_TP_PRINT_NODE, TP_PRINT_AUTHORITY, msg_tpp);
        if (NULL == tpp)
        {
            printk("tp_print_create_entry failed\n");
        }
        else
        {
            tpp->read_proc = tp_print_proc_read;
            tpp->write_proc = NULL;
            printk("tp_print_create_entry OK\n");
        }

        msleep(10);
        dbbus_tx_data[0] = 0x53;
        dbbus_tx_data[1] = (InfoAddr >> 8) & 0xFF;
        dbbus_tx_data[2] = InfoAddr & 0xFF;
        _msg_WriteI2CSeq(FW_ADDR_MSG_TP, &dbbus_tx_data[0], 3);
        _msg_ReadI2CSeq(FW_ADDR_MSG_TP, &dbbus_rx_data[0], 8);

        units = dbbus_rx_data[0];
        row = dbbus_rx_data[1];
        cnt = dbbus_rx_data[2];
        head = dbbus_rx_data[3];
        tail = dbbus_rx_data[4];
        sleep1 = dbbus_rx_data[5]>>4;
        sleep2 = dbbus_rx_data[5]&0x0F;
        TransLen = (dbbus_rx_data[7]<<8) + dbbus_rx_data[6];
        printk("tpp: row=%d, units=%d\n", row, units);
        printk("tpp: cnt=%d, head=%d, tail=%d\n", cnt, head, tail);
        printk("tpp: sleep1=%d, sleep2=%d, TransLen=%d\n", sleep1, sleep2, TransLen);
    }
    else
    {
        g_bTpp = 0;
    }
}
#endif

#ifdef TP_DEBUG_ON
static int tp_debug_proc_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
    char *buf;

    if (count < 1)
    return -EINVAL;

    buf = kmalloc(count, GFP_KERNEL);
    if (!buf)
    return -ENOMEM;

    if (copy_from_user(buf, buffer, count))
    {
        kfree(buf);
        return -EFAULT;
    }

    if (buf[0] >= '0' && buf[0] <= '9')
    {
        tp_debug_on = buf[0] - '0';
    }
    else
    {
        kfree(buf);
        return -EINVAL;
    }

    kfree(buf);
    return count;
}

static int tp_debug_proc_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    printk("tp_debug_on=%d\n",tp_debug_on);
    *eof = 1;
    return sprintf(page, "%d\n", tp_debug_on);
}

static void tp_debug_create_entry(void)
{
    tp_debug = proc_mkdir(PROC_TP_DEBUG, NULL);
    debug_on = create_proc_entry(PROC_DEBUG_ON, DEBUG_AUTHORITY, tp_debug);

    if (NULL==debug_on)
    {
        printk("tp_debug_create_entry failed\n");
    }
    else
    {
        debug_on->read_proc = tp_debug_proc_read;
        debug_on->write_proc = tp_debug_proc_write;
        printk("tp_debug_create_entry OK\n");
    }
}
#endif
MODULE_DESCRIPTION("msg Touchscreen Driver");
MODULE_LICENSE("GPL");
