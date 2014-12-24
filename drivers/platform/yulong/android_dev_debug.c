/****************************************************************************/
/*                                                                          */
/*             Copyright (c) 2004-2009  YULONG Company             　　　　 */
/*      宇龙计算机通信科技（深圳）有限公司  版权所有 2004-2009              */
/*                                                                          */
/* PROPRIETARY RIGHTS of YULONG Company are involved in the  　             */
/* subject matter of this material.  All manufacturing, reproduction, use,  */
/* and sales rights pertaining to this subject matter are governed by the   */
/* license agreement.  The recipient of this software implicitly accepts    */
/* the terms of the license.                                                */
/* 本软件文档资料是宇龙公司的资产,任何人士阅读和使用本资料必须获得          */
/* 相应的书面授权,承担保密责任和接受相应的法律约束.                         */
/*                                                                          */
/****************************************************************************/

/**************************************************************************
**  FileName:          MPCOMM.c  
**  Author:            黄捷峰
**  Version :          1.00
**  Date:              2009-06-17
**  Description:       向上层应用提供一些操作Modem的接口，比如打开、关闭、复位、唤醒、查询Modem等接口，
**                     这些接口上层应用通过ioctrol下发消息调用
**  History:         
**  <author>      <time>      <version >      <desc>
**   黄捷峰      2009-06-17    1.00           创建
**   黄捷峰      2009-10-12    1.01           在复位函数MPC_HWReset重新配置中断引脚
**                                           
**************************************************************************/

/* 
 * MP Communication module
 */
#include <linux/module.h>
#include <linux/types.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <asm/io.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/memory.h>
#include <linux/poll.h>
#include <mach/gpio.h>//为了使用设置gpio的函数
#include <linux/kernel.h>
#include <linux/slab.h>
#include "android_debug.h"


#define USB_SEL		100
#define SEL_UART	132

int android_debug_major;//预定义android_device主设备号
struct ANDROID_DEBUG_DEV *g_pandroid_debug_dev = NULL;
static struct class *sg_pandroid_debug_class = NULL;

static int android_debug_release(struct inode *inode, struct file *filp);
static int android_debug_open(struct inode *inode, struct file *filp);
static long android_debug_iocontrol(struct file *filp, unsigned  int cmd, unsigned long arg);
static int android_debug_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos);
static int android_debug_write(struct file *filp, const char __user *buf,  size_t count, loff_t *ppos);
static loff_t android_debug_seek(struct file *filp, loff_t offset, int orig);

static struct file_operations android_debug_fops = 
{
	.owner      = THIS_MODULE,
	.open        = android_debug_open,
	.release     = android_debug_release,
	.write       = android_debug_write,	
	.read        = android_debug_read,
	.llseek      = android_debug_seek,
	.unlocked_ioctl       = android_debug_iocontrol,
};


/**************************************************************************
* Function    : MPC_Resume
* Description : 电源管理函数，唤醒时会调用
* Calls       :
* Called By   :
*          1、当系统要唤醒时，由电源管理调用；
* Input :  struct platform_device *dev
* Output : 无    
* Return : 无  
* others : 无
**************************************************************************/
static int android_debug_resume(struct platform_device *dev)
{
	return 0;
}


/**************************************************************************
* Function    : MPC_Suspend
* Description : 电源管理函数，待机时会调用到
* Calls       :
* Called By   :
*          1、当系统要待机时，由电源管理调用；
* Input :  
*          1、struct platform_device *dev
*          2、pm_message_t state
* Output : 无    
* Return : 无  
* others : 无
**************************************************************************/
static int android_debug_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}




/**************************************************************************
* Function    : MPC_Probe
* Description : 字符设备探测函数，用于获取硬件资源信息，中断申请等
* Calls       :
* Called By   :
*          1、系统启动加载设备时调用；
* Input :  
*          1、*pdev  平台设备指针，通过此函数获得对应的硬件信息
* Output : 无    
* Return : 
*          1、成功返回0
*          2、失败返回非0 
* others : 
**************************************************************************/
static int android_debug_probe(struct platform_device *pdev)
{
	printk("++android_debug_probe()!\r\n");
	if(NULL == pdev)
	{
		printk(KERN_ERR"pdev =NULL\r\n");
		return -ENOMEM;	
	}

	printk("--android_debug_probe()!\r\n");

	return 0;
}


/**************************************************************************
* Function    : MPC_Remove
* Description : 删除硬件资源信息，中断申请等
* Calls       :
* Called By   :
*          1、当要删除设备时，由kernel调用；
* Input :  
*          1、*dev 平台设备指针 
* Output : 无    
* Return : 返回0
* others : 
**************************************************************************/
static int android_debug_remove(struct platform_device *dev)
{
	if(NULL == dev)
	{
		printk(KERN_INFO"dev =NULL\r\n");
		return -ENOMEM;	
	}
	
	if(NULL != g_pandroid_debug_dev)
	{
		kfree(g_pandroid_debug_dev);
		g_pandroid_debug_dev = NULL;
	}
	cdev_del(&g_pandroid_debug_dev->android_debug_cdev);//删除cdev
       unregister_chrdev_region(MKDEV(android_debug_major, 0), 1);//注销设备区域
	return 0;
}


static struct platform_driver ANDROID_DEBUG_DRIVER = 
{
	.probe		= android_debug_probe,
	.remove		= android_debug_remove,
	.suspend	= android_debug_suspend,
	.resume		= android_debug_resume,
	.driver		= 
	{
		.name	= "ANDROID_DEBUG",
		.owner	= THIS_MODULE,
	},	
};



/**************************************************************************
* Function    : modem_setup_cdev
* Description : 初始化设备结构体，并把它添加到系统的设备，初始化设备操作指针
* Calls       :
* Called By   :
*          1、在系统初始化时由MPC_Init调用
* Input :  
*          1、*dev  设备结构体指针 
*          2、index  次设备号起始值 
* Output : 无    
* Return : 无  
* others : 
**************************************************************************/
void android_debug_setup_cdev(struct ANDROID_DEBUG_DEV *dev, int dev_index)
{
	int err = 0;
    int dev_number = 0;
    int maxdevicenum = 1;

	printk("++android_debug_setup_cdev()!\r\n");
	if(dev == NULL)
	{
		printk(KERN_ERR"dev =NULL\r\n");
		return;	
	}

	dev_number = MKDEV(android_debug_major, dev_index);
	
	cdev_init(&dev->android_debug_cdev, &android_debug_fops);
	
	dev->android_debug_cdev.owner = THIS_MODULE;
	dev->android_debug_cdev.ops   = &android_debug_fops;
	
	err = cdev_add(&dev->android_debug_cdev, dev_number, maxdevicenum);	
	if(err)
	{
		printk(KERN_ERR"android_debug_cdev: Error %d adding android_debug_cdev\r\n",err);	
	}

	printk("--android_debug_setup_cdev()!\r\n");
}

void switch_tail_to_wifi_log(void)
{


}
void switch_tail_to_wifi_rf(void)
{


}

void switch_tail_to_ap_usb(void)
{
	gpio_direction_output(USB_SEL, 0);
	gpio_direction_output(SEL_UART, 0);
}

void switch_tail_to_ap_uart(void)
{
	gpio_direction_output(USB_SEL, 1);
	gpio_direction_output(SEL_UART, 0);
}

void switch_tail_to_modem(void)
{
	gpio_direction_output(USB_SEL, 1);
	gpio_direction_output(SEL_UART, 1);
}

static void uart_switch(int uartSwitch)
{
	switch(uartSwitch)
	{
		case IOCTL_UART_SWITCH_AP:
			printk("uart switch ap\r\n");
			switch_tail_to_ap_uart();

			break;

		case IOCTL_UART_SWITCH_WIFI_LOG:
			printk("uart switch wifi log\r\n");
			switch_tail_to_wifi_log();

			break;

		case IOCTL_UART_SWITCH_MODEM:
			printk("uart switch modem\r\n");
			switch_tail_to_modem();
	
			break;
			
		case IOCTL_UART_SWITCH_WIFI_RF_TEST:
			printk("uart switch wifi rf test\r\n");
			switch_tail_to_wifi_rf();
			printk("uart switch successful\n");
			break;

		default:
			printk("other uart switch\r\n");
		break;
	}
}

static void usb_switch(int usbSwitch)
{
	switch(usbSwitch)
	{
		case IOCTL_USB_SWITCH_MODEM:


			break;

		case IOCTL_USB_SWITCH_AP:
			switch_tail_to_ap_usb();
			break;
			
		default:
			printk("Don't support usb switch\r\n");
		break;
	}

	return;
}
/**************************************************************************
* Function    : MPC_Init
* Description : 初始化设备
* Calls       :
* Called By   :
*          1、在系统启动加载设备时，由kernel调用；
* Input :  无 
* Output : 无    
* Return : 
*          1、成功返回0，
*          2、失败返回其它值  
* others : 
**************************************************************************/
static int __init android_debug_init(void)
{
	int dwRet = 0;
	dev_t dev_number = 0;
	int base_number = 0;
	int max_device_number = 1;

	printk("++android_debug_init()!\r\n");

    dwRet = alloc_chrdev_region(&dev_number, base_number, max_device_number, "android-debug-device");	
    if(!dwRet)
    {
		android_debug_major = MAJOR(dev_number);
		printk(KERN_DEBUG"android_debug_init: android_debug major number is %d!\r\n",android_debug_major);
    }
    else
    {
		printk(KERN_ERR"android_debug_init: request android_debug major number fail!\r\n");
		goto out;
    }

	g_pandroid_debug_dev = kmalloc(sizeof(struct ANDROID_DEBUG_DEV), GFP_KERNEL);	
	if(!g_pandroid_debug_dev)/*alloc failure*/	
	{		
		printk(KERN_ERR"android_debug_init: kmalloc g_pandroid_debug_dev fail!\r\n");
		dwRet = -ENOMEM;		
		goto erro_unchr;
	}	
	memset(g_pandroid_debug_dev, 0, sizeof(struct ANDROID_DEBUG_DEV));

	android_debug_setup_cdev(g_pandroid_debug_dev, base_number);

	//创建设备节点
	sg_pandroid_debug_class = class_create(THIS_MODULE, "android-debug-class");
	if(IS_ERR(sg_pandroid_debug_class))
	{
		printk(KERN_ERR"android_debug_init: android_debug_device register node fail!\r\n");
	}
	device_create(sg_pandroid_debug_class, NULL, MKDEV(android_debug_major, base_number), NULL, "yl_android_device");

	dwRet = platform_driver_register(&ANDROID_DEBUG_DRIVER);
	if(dwRet != 0)
	{
		printk(KERN_ERR"android_debug_init: platform_driver_register of android_debug is failure!\r\n");
		goto erro_undrv;
	}

	printk("--android_debug_init()!\r\n");
	return dwRet;//成功返回0
	
erro_undrv:	
	if(NULL != g_pandroid_debug_dev)
	{
		kfree(g_pandroid_debug_dev);
		g_pandroid_debug_dev = NULL;	
	}	
	
erro_unchr:
	unregister_chrdev_region(dev_number, max_device_number);
	
out:
	printk(KERN_ERR"--android_debug_init Fail!\r\n");	
	return dwRet;	
} 


/**************************************************************************
* Function    : MPC_Deinit
* Description : 卸载设备，释放内存、句柄等
* Calls       :
* Called By   :
*          1、卸载设备时由应用调用；
* Input :  无 
* Output : 无    
* Return : 无  
* others : 
**************************************************************************/
// Device deinit - devices are expected to close down.
// The device manager does not check the return code.
static void __exit android_debug_deinit(void)
{
	return;	
}


/**************************************************************************
* Function    : MPC_Open
* Description : 打开设备
* Calls       :
* Called By   :
*          1、当要对设备进行操作之前，通过open函数调用；
* Input :  
*          1、*inode 设备节点指针
*          2、*filp  设备文件指针
* Output : 无    
* Return : 成功返回0 
* others : 
**************************************************************************/
static int android_debug_open(struct inode *inode, struct file *filp)
{
    printk("android_debug_open: +-\r\n");
    return 0;
} 


/**************************************************************************
* Function    : MPC_Release
* Description : 关闭设备
* Calls       :
* Called By   :
*          1、当要关闭设备时，由应用通过release函数调用；
* Input :  
*          1、*inode 设备节点指针
*          2、*filp  设备文件指针
* Output : 无    
* Return : 返回0
* others : 
**************************************************************************/
static int android_debug_release(struct inode *inode, struct file *filp)
{
	return 0;
} 


/**************************************************************************
* Function    : MPC_IOControl
* Description : 通过cmd参数下发消息，实现对模块的各种控制
* Calls       :
* Called By   :
*          1、上层应用通过ioctl函数调用；
* Input :  
*          1、*inodep 设备节点指针
*          2、*filp 文件指针
*          3、cmd  控制码，通过该参数下发消息
*          4、arg  上层应用和底层驱动互传数据的参数
* Output : 
*          1、arg  上层应用和底层驱动互传数据的参数   
* Return : 
*          1、成功返回：0
*          2、失败返回：-EINVAL，cmd命令不存在时返回  
* others : 
*    1、黄捷峰     2009-09-22    由于传进来的参数arg是一个结构体，结构体中
*                                又带有指针，所以要对结构体的指针操作时，还
*                                要用一次copy_to_user/copy_from_user     
**************************************************************************/
//extern void set_alarm_test(int value);
static long android_debug_iocontrol(struct file *filp, unsigned  int cmd, unsigned long arg)
{
	//void __user *tmp = (void __user *)arg;
	printk("android_debug_iocontrol: ++\r\n");	
    
	switch(cmd)
	{
		case IOCTL_UART_SWITCH_AP:
			printk("android_debug_iocontrol(): IOCTL_UART_SWITCH_AP\r\n");
			uart_switch(IOCTL_UART_SWITCH_AP);
			break;
			
		case IOCTL_UART_SWITCH_WIFI_LOG://WIFI调试信息
			printk("android_debug_iocontrol(): IOCTL_UART_SWITCH_WIFI_LOG\r\n");
			uart_switch(IOCTL_UART_SWITCH_WIFI_LOG);
			break;			

		case IOCTL_UART_SWITCH_MODEM:
			printk("android_debug_iocontrol(): IOCTL_UART_SWITCH_MODEM\r\n");
			uart_switch(IOCTL_UART_SWITCH_MODEM);		
			break;

		case IOCTL_UART_SWITCH_WIFI_RF_TEST:
			printk("android_debug_iocontrol(): IOCTL_UART_SWITCH_WIFI_RF_TEST\r\n");
			uart_switch(IOCTL_UART_SWITCH_WIFI_RF_TEST);
			break;

		case IOCTL_USB_SWITCH_MODEM:
			printk("android_debug_iocontrol(): IOCTL_USB_SWITCH_MODEM\n");
			usb_switch(IOCTL_USB_SWITCH_MODEM);
			break;

		case IOCTL_USB_SWITCH_AP:
			printk("android_debug_iocontrol(): IOCTL_USB_SWITCH_AP\n");
			usb_switch(IOCTL_USB_SWITCH_AP);
			break;

		default:
			printk(KERN_WARNING"No supporting cmd\r\n");
			return -EINVAL;
	}

	printk("android_debug_iocontrol: --\r\n");
   	return 0;
}  


/**************************************************************************
* Function    : MPC_Read
* Description : 模块读函数，该函数目前暂未用到
* Calls       :
* Called By   :
*             1、上层应用通过read函数调用；
* Input :  
*             1、*filp 文件指针
*             2、*ppos 
* Output : 
*             1、*buf  要读的数据缓冲区指针
*             2、count，要读的数据字节    
* Return : 返回值为0   
* others : 无
**************************************************************************/	
static int android_debug_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos)
{
	return 0;
}


/**************************************************************************
* Function    : MPC_Write
* Description : 模块写函数，该函数目前暂未用到
* Calls       :
* Called By   :
*             1、上层应用通过write函数调用
* Input :  
*             1、*filp 文件指针
*             2、*buf  要写的数据缓冲区指针
*             3、count，要写的数据字节
*             4、*ppos
* Output : 无    
* Return : 返回值为0  
* others : 无
**************************************************************************/
static int android_debug_write(struct file *filp, const char __user *buf,  size_t count, loff_t *ppos)
{
	char arg[32] = {0};
	int ret=0;
	ret=copy_from_user(arg, buf, sizeof(arg)-1);
	printk("%s:arg=%s\n", __func__, arg);
	if(!strncmp(arg, "usb", strlen("usb")))
	{
		printk(KERN_ERR "%s:tail switch to ap usb\n", __func__);
    		switch_tail_to_ap_usb();
	}
	else if(!strncmp(arg, "uart", strlen("uart")))
	{
		printk(KERN_ERR "%s:tail switch to ap uart\n", __func__);
		switch_tail_to_ap_uart();
	}
	return 0;
}


/**************************************************************************
* Function    : MPC_Seek
* Description : 指针定位函数，该函数目前暂未用到
* Calls       :
* Called By   :
*          1、上层应用通过seek调用；
* Input :  
*          1、*filp 文件指针
*          2、offset 指针偏移
*          3、orig
* Output : 无    
* Return : 返回值0  
* others : 无
**************************************************************************/
static loff_t android_debug_seek(struct file *filp, loff_t offset, int orig)
{
	return 0;
}


module_init(android_debug_init);
module_exit(android_debug_deinit);
MODULE_AUTHOR("Huang Jiefeng");
MODULE_LICENSE("GPL");
