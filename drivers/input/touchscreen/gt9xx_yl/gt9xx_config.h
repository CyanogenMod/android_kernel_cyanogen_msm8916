#ifndef _LINUX_GOODIX_CONFIG_H
#define	_LINUX_GOODIX_CONFIG_H

#ifdef CONFIG_MACH_CP8675
const char * TW_IC_PREFIX_NAME = "GT970";
#else
const char * TW_IC_PREFIX_NAME = "GT9158";
#endif
//***************************PART1:ON/OFF define*******************************
#define GTP_CUSTOM_CFG        1
#define GTP_DRIVER_SEND_CFG   1
#define GTP_HAVE_TOUCH_KEY    1
#define GTP_POWER_CTRL_SLEEP  0
#define GTP_AUTO_UPDATE       1
#define GTP_CHANGE_X2Y        0


/* begin to delete this macro and it will be define in include/linux/gt9xx.h by liushilong@yulong.com on 2014-11-6 14:57*/
//#define GTP_ESD_PROTECT       1
/* end to delete this macro and it will be define in include/linux/gt9xx.h by liushilong@yulong.com on 2014-11-6 14:57*/


#define GTP_CREATE_WR_NODE    1
#define GTP_ICS_SLOT_REPORT   0

#define GUP_USE_HEADER_FILE   0

#define GTP_SLIDE_WAKEUP      1

#define GTP_DBL_CLK_WAKEUP    0
#define YL_COVER_SWITCH_FUNC  0



#define GTP_MAX_TOUCH         5

/* begin to transplant TW GTP_ESD_PROTECT code from 8675_C00, 1s GTP_ESD_CHECK_CIRCLE * HZ, it will be transform to jiffy by liushilong@yulong.com on 2014-11-6 15:00*/
#define GTP_ESD_CHECK_CIRCLE  2
/* begin to transplant TW GTP_ESD_PROTECT code from 8675_C00, 1s GTP_ESD_CHECK_CIRCLE * HZ, it will be transform to jiffy by liushilong@yulong.com on 2014-11-6 15:00*/

//#define TW_GLOVE_SWITCH       0

#ifdef CONFIG_MACH_CP8675
#define GTP_COB		      1
#else
#define GTP_COB		      0
#endif 
#if GTP_HAVE_TOUCH_KEY
    #define GTP_KEY_TAB	 {KEY_MENU, KEY_HOME, KEY_BACK}
#endif

struct touch_panel_info
{
        unsigned char  tp_id;        	/*touch panel factory id*/
	unsigned char *tp_name;    	/*touch panel factory name */
	unsigned char *firmware;   	/*firmware for this factory's touch panel*/
	unsigned int   firmware_size; 
};

//[YULONG BEGIN] Upload new touchscreen configs to solve touchscreen failure in games occationally by litao3@yulong.com 2014-12-23
#ifdef CONFIG_MACH_CP8675
static unsigned char CTP_CFG_GROUP1[]=  
{
    #include "coolpad_8675_00_V46_20141111_finger.cfg"
};
static unsigned char CTP_CFG_GROUP1_glove[]=  
{
    #include "coolpad_8675_00_V46_20141111_glove.cfg"
};
#ifdef CONFIG_TOUCHSCREEN_GT9XX_YL_COVER_WINDOW_CFG
static unsigned char CTP_CFG_GROUP1_window[]=  
{
    #include "coolpad_8675_00_V46_20141111_window.cfg"
};
#endif

static unsigned char CTP_CFG_GROUP2[]=  
{
    #include "coolpad_8675_02_V46_20141111_finger.cfg"
};
static unsigned char CTP_CFG_GROUP2_glove[]=  
{
    #include "coolpad_8675_02_V46_20141111_glove.cfg"
};
#ifdef CONFIG_TOUCHSCREEN_GT9XX_YL_COVER_WINDOW_CFG
static unsigned char CTP_CFG_GROUP2_window[]=  
{
    #include "coolpad_8675_02_V46_20141111_window.cfg"
};
#endif

static unsigned char CTP_CFG_GROUP3[]=  
{
    #include "coolpad_8675_01_V46_20141111_finger.cfg"
};
static unsigned char CTP_CFG_GROUP3_glove[]=  
{
    #include "coolpad_8675_01_V46_20141111_glove.cfg"
};
#ifdef CONFIG_TOUCHSCREEN_GT9XX_YL_COVER_WINDOW_CFG
static unsigned char CTP_CFG_GROUP3_window[]=  
{
    #include "coolpad_8675_01_V46_20141111_window.cfg"
};
#endif
//[YULONG END]

struct touch_panel_info yl_cfg[] = {
  {0x00, "Boen", CTP_CFG_GROUP1, sizeof(CTP_CFG_GROUP1)},
  {0x02, "Ofilm", CTP_CFG_GROUP2, sizeof(CTP_CFG_GROUP2)},
  {0x01, "Each", CTP_CFG_GROUP3, sizeof(CTP_CFG_GROUP3)},
};

struct touch_panel_info yl_cfg_glove[] = {
  {0x00, "Boen", CTP_CFG_GROUP1_glove, sizeof(CTP_CFG_GROUP1_glove)},
  {0x02, "Ofilm", CTP_CFG_GROUP2_glove, sizeof(CTP_CFG_GROUP2_glove)},
  {0x01, "Each", CTP_CFG_GROUP3_glove, sizeof(CTP_CFG_GROUP3_glove)},
};
#ifdef CONFIG_TOUCHSCREEN_GT9XX_YL_COVER_WINDOW_CFG
struct touch_panel_info yl_cfg_window[] = {
  {0x00, "Boen", CTP_CFG_GROUP1_window, sizeof(CTP_CFG_GROUP1_window)},
  {0x02, "Ofilm", CTP_CFG_GROUP2_window, sizeof(CTP_CFG_GROUP2_window)},
  {0x01, "Each", CTP_CFG_GROUP3_window, sizeof(CTP_CFG_GROUP3_window)},
};
#endif

#else
static unsigned char CTP_CFG_GROUP1[]=  
{
	#include "coolapd_7625_00_V47_20140730_finger.cfg"
};
static unsigned char CTP_CFG_GROUP1_glove[]=  
{
	#include "coolapd_7625_00_V47_20140730_glove.cfg"
};


 static unsigned char CTP_CFG_GROUP2[]=
{
   #include "coolapd_7625_01_V47_20140730_finger.cfg"
};
 static unsigned char CTP_CFG_GROUP2_glove[]=
{
   #include "coolapd_7625_01_V47_20140730_glove.cfg"
};

static unsigned char CTP_CFG_GROUP3[]=
{
    #include "coolapd_7625_02_V47_20140730_finger.cfg"
};
static unsigned char CTP_CFG_GROUP3_glove[]=
{
    #include "coolapd_7625_02_V47_20140730_glove.cfg"  
};

struct touch_panel_info yl_cfg[] = {
  {0x00, "boen", CTP_CFG_GROUP1, sizeof(CTP_CFG_GROUP1)},
  {0x01, "junda", CTP_CFG_GROUP2, sizeof(CTP_CFG_GROUP2)},
  {0x02, "ofilm", CTP_CFG_GROUP3, sizeof(CTP_CFG_GROUP3)},
};
struct touch_panel_info yl_cfg_glove[] = {
  {0x00, "boen", CTP_CFG_GROUP1_glove, sizeof(CTP_CFG_GROUP1_glove)},
  {0x01, "junda", CTP_CFG_GROUP2_glove, sizeof(CTP_CFG_GROUP2_glove)},
  {0x02, "ofilm", CTP_CFG_GROUP3_glove, sizeof(CTP_CFG_GROUP3_glove)},  
};
#endif

#endif
