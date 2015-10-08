/**************************************************************************/
/*                                                                        */
/* Copyright (c) 2008-2010  YULONG Company             　　　　　　　     */
/*                 宇龙计算机通信科技（深圳）有限公司  版权所有 2008-2010 */
/*                                                                        */
/* PROPRIETARY RIGHTS of YULONG Company are involved in the  　　　　　　 */
/* subject matter of this material.  All manufacturing, reproduction, use,*/
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement.  The recipient of this software implicitly accepts  */ 
/* the terms of the license.                                              */
/* 本软件文档资料是宇龙公司的资产,任何人士阅读和使用本资料必须获得        */
/* 相应的书面授权,承担保密责任和接受相应的法律约束.                       */
/*                                                                        */
/*                                                                        */
/**************************************************************************/

/**************************************************************************
**  Copyright (C), 2000-2010, Yulong Tech. Co., Ltd.
**  FileName:          MPCOMM.h  
**  Author:            黄捷峰
**  Version :          1.00
**  Date:              2009-06-17
**  Description:       头文件，宏定义ioctrol的值
**                     声明函数，定义一些结构体
**  History:         
**  <author>      <time>      <version >      <desc>
**   黄捷峰      2009-06-17    1.00           创建
**   黄捷峰      2009-10-12    1.01           为了兼容以前的通信程序，修改ioctrolcmd宏定义的值
**                                           
**************************************************************************/

#ifndef __ANDROID_DEBUG_H__
#define __ANDROID_DEBUG_H__

#ifdef __cplusplus
extern "C" {
#endif


#define IOCTL_UART_MAGIC                        'D'
#define IOCTL_UART_SWITCH_AP                   	_IOWR(IOCTL_UART_MAGIC, 1, unsigned long )
#define IOCTL_UART_SWITCH_WIFI_LOG            	_IOWR(IOCTL_UART_MAGIC, 2, unsigned long )
#define IOCTL_UART_SWITCH_MODEM               	_IOWR(IOCTL_UART_MAGIC, 3, unsigned long )
#define IOCTL_UART_SWITCH_WIFI_RF_TEST       	_IOWR(IOCTL_UART_MAGIC, 4, unsigned long )
#define IOCTL_USB_SWITCH_MODEM              	_IOWR(IOCTL_UART_MAGIC, 5, unsigned long )
#define IOCTL_USB_SWITCH_AP                   	_IOWR(IOCTL_UART_MAGIC, 6, unsigned long )
#define IOCTL_GET_CPU_VERSION                  	_IOWR(IOCTL_UART_MAGIC, 7, unsigned long )

#ifdef __cplusplus
}
#endif

struct ANDROID_DEBUG_DEV
{
	struct cdev android_debug_cdev;/*cdev结构体*/
	int major;
	int name;
};

#endif // __MPCOMM_H__
