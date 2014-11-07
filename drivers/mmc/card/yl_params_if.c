#include <linux/yl_params.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/kernel.h>

#include <linux/export.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/notifier.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/scatterlist.h>

#include <linux/genhd.h>
#include <linux/reboot.h>

#include <linux/mutex.h>
#include <linux/yl_params.h>

unsigned char yl_read_params_charge_mode(unsigned char *chg_mode)
{
	int ret = 0;
	char params[512];
	memset(params,0,sizeof(params));
	strncpy(params, "RETURNZERO", 10); 
	ret = yl_params_kernel_read(params,sizeof(params));
	if(ret != sizeof(params))
	{
		printk(KERN_ERR "YL_PARAMS:%s_read params error\r\n",__func__);
		return 0;
	}
	memcpy(chg_mode, &params[RETURNZERO_USB_CHARGER_TYPE], sizeof(char));
	return 1;
}
EXPORT_SYMBOL_GPL(yl_read_params_charge_mode);

unsigned char yl_write_params_charge_mode(unsigned char *buf)
{
	int ret = 0;
	char params[512];
	memset(params,0,sizeof(params));  
	strncpy(params, "RETURNZERO", 10);
	ret = yl_params_kernel_read(params,sizeof(params));
	if(ret != sizeof(params))
	{
		printk(KERN_ERR "YL_PARAMS:%s_read params error\r\n",__func__);
		return 0;
	}
	memcpy(&params[RETURNZERO_USB_CHARGER_TYPE], buf, sizeof(char));
	ret = yl_params_kernel_write(params, sizeof(params));
	if(ret != sizeof(params))
	{
	  	printk(KERN_ERR "YL_PARAMS:%s_write params error\r\n",__func__);
	  	return 0;
	}
	return 1;
}
EXPORT_SYMBOL_GPL(yl_write_params_charge_mode);




