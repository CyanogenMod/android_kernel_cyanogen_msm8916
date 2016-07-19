/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2000-2010  YULONG Company             　　　　　　　           */
/*         宇龙计算机通信科技（深圳）有限公司  版权所有 2000-2010               */
/*                                                                              */
/* PROPRIETARY RIGHTS of YULONG Company are involved in the                     */
/* subject matter of this material.  All manufacturing, reproduction, use,      */
/* and sales rights pertaining to this subject matter are governed by the       */
/* license agreement.  The recipient of this sofPARAMSare implicitly accepts        */
/* the terms of the license.                                                    */
/* 本软件文档资料是宇龙公司的资产,任何人士阅读和使用本资料必须获得              */
/* 相应的书面授权,承担保密责任和接受相应的法律约束.                             */
/*                                                                              */
/********************************************************************************/

/**************************************************************************
**  Copyright (C), 2000-2010, Yulong Tech. Co., Ltd.
**  FileName:         Coolpad_hall.c
**  Author:            shuaixinzhong
**  Version :          1.00
**  Date:              2010-12-20
**  Description:       hall driver
**
**  History:
**  <author>           <time>      <version >      <desc>
**  shuaixinzhong      2010-12-20     1.00           创建
**  changxuejian        2010-12-21     2.00           modify
**************************************************************************/
#include <asm/io.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <asm/atomic.h>
#include <linux/sensors/coolpad_hall.h>
#include <linux/wakelock.h>

#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif
//#define  CONFIG_TOUCHSCREEN_GT9XX_YL_TW_GLOVE_SWITCH

struct hall_info {
    struct hall_platform_data *pdata;
    struct switch_dev sdev;
    struct input_dev *input;
    struct mutex mutex_lock;
    int enabled;

    atomic_t hall_state;
    struct wake_lock  wake_lock_timeout;

    struct hrtimer timer;
    ktime_t debounce_time;

    const char *which;
};
static struct hall_info *hall_dev;
static int hall_old_status = 0;
static int hall_last_status = 0;

static struct workqueue_struct *g_hall_work_queue;
static void hall_detection_work(struct work_struct *work);
static DECLARE_WORK(g_hall_work, hall_detection_work);

#ifdef  CONFIG_TOUCHSCREEN_GT9XX_YL_TW_GLOVE_SWITCH
extern int glove_windows_switch(int in_hall);
#endif  //CONFIG_TOUCHSCREEN_GT9XX_YL_TW_GLOVE_SWITCH

#ifdef CONFIG_LPM_MODE
extern unsigned int poweroff_charging;
extern unsigned int recovery_mode;
#endif

ssize_t hall_print_state(struct switch_dev *sdev, char *buf)
{
    int res;

    if (hall_last_status)
        res = sprintf(buf, "%s", "leave");
    else
        res = sprintf(buf, "%s", "near");
    //begin shihaobin@yulong.com 2014/11/25 delete to resolve system crush down in FastMMI mode due to HALL sensor
    //printk(KERN_ERR "HALL:YLLOG:%s", (hall_last_status)?"leave":"near");
    //begin shihaobin@yulong.com 2014/11/25 delete to resolve system crush down in FastMMI mode due to HALL sensor
    return res;
}

#ifdef  CONFIG_TOUCHSCREEN_GT9XX_YL_TW_GLOVE_SWITCH
static void tw_mode_switch( int hall_status )
{
    if (hall_status) {
        if ( glove_windows_switch(0) )
            return;
        printk(KERN_ERR "HALL:YLLOG:windows mode OFF in kernel complete.\n");
    } else {
        if ( glove_windows_switch(1) )
            return;
        printk(KERN_ERR "HALL:YLLOG:windows mode ON in kernel complete.\n");
    }

    return;
}
#else
static void tw_mode_switch( int hall_status )
{
    return;
}
#endif  //CONFIG_TOUCHSCREEN_GT9XX_YL_TW_GLOVE_SWITCH

static void hall_detection_work(struct work_struct *work)
{
    int i = 0;
    int hall_status = 0;
    int state[2] = {0};
    struct hall_platform_data *pdata = hall_dev->pdata;

    if ( 2 == pdata->hall_num && strcmp(hall_dev->which, "left") ) {
        for ( i= 0; i < pdata->hall_num; i++ ) {
            state[i] = gpio_get_value(pdata->hall[i].gpio);
            printk(KERN_ERR "HALL:YLLOG: ===%s:current status is %s===\n",
                pdata->hall[i].name, (state[i]) ? "leave":"near");
        }

        if ( state[0] == state[1] ) {

            hall_last_status = state[0];
            switch_set_state(&hall_dev->sdev, hall_last_status);
            printk(KERN_ERR "HALL:YLLOG: ===report BY double hall:status is %s===\n",
                (hall_last_status) ? "leave" : "near");
            tw_mode_switch( hall_last_status );
        } else {
            printk(KERN_ERR "HALL:YLLOG: ===two hall sensors are in diff status===\n");
        }

        for ( i = 0; i < pdata->hall_num; i++ )
            enable_irq(pdata->hall[i].irq);

        return;
    }

    hall_status = gpio_get_value(pdata->hall[0].gpio);

    printk(KERN_ERR "HALL:YLLOG: hall_detection_work exit: old=%d curr=%d last=%d.\n",
         hall_old_status, hall_status, hall_last_status);

    if(hall_old_status == hall_status) {
        if(hall_last_status != hall_status) {
            hall_last_status = hall_status;

            mutex_lock(&hall_dev->mutex_lock);
            switch_set_state(&hall_dev->sdev, hall_status);
            printk(KERN_ERR "HALL:YLLOG: ===report BY single hall:status is %s===\n",
                (hall_status) ? "leave" : "near");
            tw_mode_switch( hall_status );
            mutex_unlock(&hall_dev->mutex_lock);
        }
    }

    for ( i = 0; i < pdata->hall_num; i++ )
            enable_irq(pdata->hall[i].irq);
}

static enum hrtimer_restart hall_detect_timer_func(struct hrtimer *data)
{
    printk(KERN_ERR "HALL:YLLOG: ===liwei11 in hall_detect_timer_func function===\n");
    queue_work(g_hall_work_queue, &g_hall_work);
    return HRTIMER_NORESTART;
}

static irqreturn_t  gpio_hall_isr(int irq, void *dev_id)
{
    struct hall_platform_data *pdata = hall_dev->pdata;
    unsigned long irq_flags;
    int status;
    int i;

    wake_lock_timeout(&hall_dev->wake_lock_timeout, 2*HZ);

    for ( i = 0; i < pdata->hall_num; i++) {
        disable_irq_nosync (pdata->hall[i].irq);
        status = gpio_get_value(pdata->hall[i].gpio);
        local_irq_save(irq_flags);
        irq_set_irq_type(pdata->hall[i].irq, (status) ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH);
        local_irq_restore(irq_flags);

        if ( irq == pdata->hall[i].irq ) {
            hall_dev->which = pdata->hall[i].name;
            hall_old_status = status;
        }

    }
    printk(KERN_ERR "HALL:YLLOG: === %s hall trigger INT===\n", hall_dev->which );

    hrtimer_start(&hall_dev->timer, hall_dev->debounce_time, HRTIMER_MODE_REL);

    printk(KERN_ERR "HALL:YLLOG: ===liwei11 after hrtimer_start===\n");

    return IRQ_HANDLED;
 }

static ssize_t enable_show(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
    int ret;

    ret = sprintf(buf, "%d", hall_dev->enabled);
    return ret;
}

static ssize_t enable_store(struct device *dev,
                    struct device_attribute *attr,const char *buf, size_t count)
{
    int i;
    unsigned long val;
    struct hall_platform_data *pdata = hall_dev->pdata;
    printk(KERN_ERR "HALL:YLLOG: ===liwei11 in enable_store===\n");


    if (strict_strtoul(buf, 8, &val))
    return -EINVAL;

    printk(KERN_ERR "HALL:YLLOG: ===liwei11 in enable_store after return EINVAL===\n");

    if (0 == val) {
        if (hall_dev->enabled) {
            for ( i = 0; i < pdata->hall_num; i++) {
                disable_irq(pdata->hall[i].irq);
                irq_set_irq_wake(pdata->hall[i].irq, 0);
            }
            hall_dev->enabled = 0;
            printk(KERN_ERR "HALL:YLLOG:HALL disable irq.\n");
        }
    } else {
        if (0 == hall_dev->enabled) {
            for ( i = 0; i < pdata->hall_num; i++) {
                enable_irq(pdata->hall[i].irq);
                irq_set_irq_wake(pdata->hall[i].irq, 1);
            }

            hall_last_status = gpio_get_value(pdata->hall[0].gpio);
            switch_set_state(&hall_dev->sdev, hall_last_status);
            tw_mode_switch( hall_last_status );

            hall_dev->enabled = 1;
            printk(KERN_ERR "HALL:YLLOG:HALL enable irq.\n");
        }
    }
    return count;
}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR, enable_show, enable_store);

static struct attribute *enable_attribute[] = {
    &dev_attr_enable.attr,
    NULL,
};

static struct attribute_group enable_attribute_group = {
    .attrs = enable_attribute
};

#ifdef CONFIG_OF
static int hall_get_devtree_pdata(struct device *dev, struct hall_platform_data *pdata)
{
        struct device_node *dt_node = NULL;
    struct device_node *pp = NULL;
    struct hall_node *hall = NULL;
        enum of_gpio_flags flags;
    int i = 0;

    dt_node = dev->of_node;
        if (!dt_node)
                return -ENODEV;

        memset(pdata, 0, sizeof *pdata);
        pdata->name = of_get_property(dt_node, "switch_name", NULL);

    pdata->hall_num = 0;
    pp = NULL;
    while ((pp = of_get_next_child(dt_node, pp)))
        pdata->hall_num++;
    printk(KERN_ERR "HALL:YLLOG:===%d===\n",  pdata->hall_num);

    if (pdata->hall_num == 0)
        return -ENODEV;

    hall = kzalloc(pdata->hall_num * (sizeof *hall), GFP_KERNEL);
    if (!hall)
        return -ENOMEM;

    pp = NULL;
    i = 0;
    while ((pp = of_get_next_child(dt_node, pp))) {
            hall[i].name = of_get_property(pp, "hall_name", NULL);
        hall[i].gpio = of_get_gpio_flags(pp, 0, &flags);
        hall[i].wakeup = of_property_read_bool(pp, "wakeup");
        printk(KERN_ERR "HALL:YLLOG:===%s hall:gpio = %d,wakeup = %d===\n",
             hall[i].name, hall[i].gpio, hall[i].wakeup);
        i++;
    }
    pdata->hall = hall;

    printk(KERN_ERR "HALL:YLLOG:success get hall devtree pdata!\n");

    return 0;
}

static struct of_device_id hall_of_match[] = {
        { .compatible = "hall_switch", },
        {},
};
#else
static int hall_get_devtree_pdata(struct device *dev, struct hall_platform_data *pdata)
{
        return -ENODEV;
}
#endif

static int hall_probe(struct platform_device *pdev)
{
    int ret, hall_status = 0;
    struct hall_platform_data *pdata;
    int i;

    printk(KERN_ERR "HALL:YLLOG: Registering hall driver\n");

    pdata = kzalloc(sizeof(struct hall_platform_data), GFP_KERNEL);
    if (!pdata)
        return -ENOMEM;

    ret = hall_get_devtree_pdata(&pdev->dev, pdata);
    if (ret)
        return -ENODEV;
    pdev->dev.platform_data = pdata;

    hall_dev = kzalloc(sizeof(struct hall_info), GFP_KERNEL);
    if (!hall_dev)
        return -ENOMEM;

    hall_dev->pdata = pdata;
    hall_dev->which = "unknown";
    hall_dev->sdev.name = pdata->name;
    hall_dev->enabled = 0;
    hall_dev->sdev.print_state = hall_print_state;
    hall_dev->debounce_time = ktime_set(0, 50000000);  /* 50 ms */

    platform_set_drvdata(pdev, hall_dev);

    mutex_init(&hall_dev->mutex_lock);
    wake_lock_init(&hall_dev->wake_lock_timeout, WAKE_LOCK_SUSPEND, "hall_wake_lock_timeout");
    hrtimer_init(&hall_dev->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    hall_dev->timer.function = hall_detect_timer_func;

    ret = switch_dev_register(&hall_dev->sdev);
    if (ret < 0)
        goto err_switch_dev_register;

    g_hall_work_queue = create_workqueue("hall_detect");
    if ( !g_hall_work_queue ) {
        goto err_create_work_queue;
    }

  //===============get gpio irq===============//
    for (i = 0; i < pdata->hall_num; i++) {
        ret = gpio_request(pdata->hall[i].gpio, "hall_detect");
        if (ret < 0){
            printk(KERN_ERR "HALL:YLLOG:error in gpio request (%d)\n", i);
            goto err_irq_setup_failed;
        }
        pdata->hall[i].irq = gpio_to_irq( pdata->hall[i].gpio );
        if ( pdata->hall[i].irq < 0 ) {
            printk(KERN_ERR "HALL:YLLOG:error in gpio to irq (%d)\n", i);
            goto err_irq_setup_failed;
        }

        hall_status = gpio_get_value(pdata->hall[i].gpio);
        hall_last_status = hall_status;
        //switch_set_state(&hall_dev->sdev, hall_status);
        //printk(KERN_ERR "HALL:YLLOG:init hall status is (%s)\n", (hall_status));
        ret = request_irq(pdata->hall[i].irq, gpio_hall_isr,
        hall_status ? IRQF_TRIGGER_LOW : IRQF_TRIGGER_HIGH,"hall_detect",NULL);
        if (ret < 0){
            printk(KERN_ERR "HALL:YLLOG:error in setup irq (%d)\n", i);
            goto err_irq_setup_failed;
        }

        ret = irq_set_irq_wake(pdata->hall[i].irq, 1);
        if (ret < 0){
                        printk(KERN_ERR "HALL:YLLOG:error in setup irq wake (%d)\n", i);
                        goto err_irq_setup_failed;
        }

        if ( !(hall_dev->enabled) ) {
            hall_last_status = 1;
            switch_set_state(&hall_dev->sdev, 1);
            disable_irq(pdata->hall[i].irq);
            irq_set_irq_wake(pdata->hall[i].irq, 0);
        }
    }

    ret = sysfs_create_group(&hall_dev->sdev.dev->kobj, &enable_attribute_group);
    if (ret) {
        printk(KERN_ERR "HALL:YLLOG:%s: hall enable sysfs_create_group failed.\n", __func__);
    }

    printk(KERN_ERR "HALL:YLLOG:: Success to register driver\n");

    return 0;

err_irq_setup_failed:
    while( --i >= 0 ) {
        free_irq(pdata->hall[i].irq, 0);
        gpio_free(pdata->hall[i].gpio);
    }
err_create_work_queue:
    destroy_workqueue(g_hall_work_queue);
err_switch_dev_register:
    switch_dev_unregister(&hall_dev->sdev);

    return ret;
}

static int hall_remove(struct platform_device *pdev)
{
    int i;
    struct hall_platform_data *pdata = hall_dev->pdata;

    for ( i = 0; i < pdata->hall_num; i++) {
        gpio_free(pdata->hall[i].gpio);
        free_irq(pdata->hall[i].irq, 0);
    }
    destroy_workqueue(g_hall_work_queue);
    switch_dev_unregister(&hall_dev->sdev);

    return 0;
}

static struct platform_driver hall_driver = {
    .probe      =hall_probe,
    .remove     = hall_remove,
    .driver     = {
        .name       = "hall_switch",
        .owner      = THIS_MODULE,
#ifdef CONFIG_OF
                .of_match_table = hall_of_match,
#endif
    },
};

static int __init hall_init(void)
{
#ifdef CONFIG_LPM_MODE
    printk(KERN_ERR "HALL:YLLOG: module init, poweroff = %d, recovery =%d\n",
        poweroff_charging, recovery_mode);
    if (1 == poweroff_charging || 1 == recovery_mode)
        return 0;
#endif//CONFIG_LPM_MODE

    return platform_driver_register(&hall_driver);
}

static void __exit hall_exit(void)
{
    platform_driver_unregister(&hall_driver);
}
MODULE_AUTHOR("<coolpad.com>");
MODULE_DESCRIPTION("Hall Switch Sensor driver");
MODULE_LICENSE("GPL");

module_init(hall_init);
module_exit(hall_exit);
