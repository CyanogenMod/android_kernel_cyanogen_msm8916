#ifndef _TOUCHSCREEN_H
#define _TOUCHSCREEN_H

typedef enum
{
	ID_MAIN	=0,
	ID_SUB		=1,
	ID_INVALID	=2,
}touch_id_type;

//手写相关部分
typedef enum
{
	MODE_INVALID=0,
	MODE_NORMAL,
	MODE_HANDWRITE,
	MODE_GLOVE,
	MODE_WINDOW,
	MODE_MAX,
}touch_mode_type;

typedef enum
{
	OREITATION_INVALID	=0,
	OREITATION_0       	=1,
	OREITATION_90 		=2,
	OREITATION_180		=3,
	OREITATION_270		=4,
}touch_oreitation_type;

typedef struct touchscreen_funcs {
 touch_id_type touch_id;			// 0--外屏；1---内屏
 int touch_type;					// 1---电容屏，2---电阻屏
 int (*active)(void);				// 1--当前使用状态，0--待机状态
 int (*firmware_need_update)(void);// 1--需要升级固件，0--固件已经是最新
 int (*firmware_do_update)(void);	//系统写"update"
 int (*need_calibrate)(void);		// 1--需要校准，0--不需要校准
 int (*calibrate)(void);				//系统写"calibrate"
 int (*get_firmware_version)(char * );//返回长度
 int (*reset_touchscreen)(void);	//通信写"reset"
 touch_mode_type (*get_mode)(void);//输入法"handwrite" "normal"
 int (*set_mode)(touch_mode_type );//输入法"handwrite" "normal"
 touch_oreitation_type (*get_oreitation)(void);//传感器应用"oreitation:X"
 int (*set_oreitation)(touch_oreitation_type );	//传感器应用"oreitation:X"
 int (*read_regs)(char * );			//buf[256]: ef ab [寄存器]：值
 int (*write_regs)(const char * );	//buf[256]: ef ab [寄存器]：值
 int (*debug)(int );				//开关调试模式
 int (*get_vendor)(char *);
 int (*get_wakeup_gesture)(char *);
 int (*get_gesture_ctrl)(char *);
 int (*gesture_ctrl)(const char *);
 int (*get_charger_state)(char *);
 int (*set_charger_state)(const char *);
}touchscreen_ops_tpye;

struct virtual_keys_button {
	int x;
	int y;
	int width;
	int height;
	unsigned int code;
	unsigned int key_status;
};

struct tw_platform_data{
	int  (*init)(void);
	void  (*release)(void);
	int  (*power) (int on);
	int  (*reset) (int ms);
	int  (*suspend)(void);
	int  (*resume)(void);
	unsigned char (*get_id_pin)(void);
	struct virtual_keys_button *buttons;
	int nbuttons;
	unsigned long  irqflag;
	unsigned int gpio_irq;
	unsigned int gpio_reset;
	int screen_x;
	int screen_y;
	int key_debounce;
	int pwr_en;
	unsigned int sleep_pwr_en;
	const char *ts_vcc_i2c;
	const char *ts_vdd;
};

#endif /* _TOUCHSCREEN_H */

/**********************************************************************
* 函数名称：touchscreen_set_ops

* 功能描述：设置触摸屏节点操作函数

* 输入参数：touchscreen_ops_tpye

* 输出参数：NONE

* 返回值      ：0---成功，-1---失败

* 其它说明：

* 修改日期         修改人	              修改内容
* --------------------------------------------------------------------
* 2011/11/19	   冯春松                  创 建
**********************************************************************/
extern int touchscreen_set_ops(touchscreen_ops_tpye *ops);
