#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <yl_debug/management.h>


static DEFINE_MUTEX(module_management_lock);
static LIST_HEAD(module_management_handlers);

int mmi_sleep_test_flag = 0;
#ifdef CONFIG_APP_PROFILE
int app_profile_disable_flag = 0;
#endif
int yulong_register_module_management(struct module_management *handler)
{
  struct module_management *pos;
  	if(!(handler->name)){
			    printk(KERN_ERR "%s:name is NULL.\n", __func__);
	return -1;
	}
     if(strlen(handler->name) >= (sizeof(struct modules_state) - 1)){                                   
		 printk(KERN_ERR "%s:Error! device name %s is too long.It would load to an error! \n", __func__,handler->name);
	     return -1;
	   }

	mutex_lock(&module_management_lock);		
		list_for_each_entry(pos, &module_management_handlers, link) {
         if(pos->name){
		     if (!strcmp(pos->name, handler->name)){
		       	mutex_unlock(&module_management_lock);	
		   		 printk(KERN_ERR
		     "%s: %s is failed,there was a same name.\n", __func__, handler->name);		     
			return -1;
		   }
          }	
		}        
		list_add_tail(&handler->link, &module_management_handlers);	
	mutex_unlock(&module_management_lock);	
return 0;

}
EXPORT_SYMBOL(yulong_register_module_management);


int yulong_unregister_module_management(struct module_management *handler)
{

  	struct list_head *pos;
     pos = &module_management_handlers;
 	mutex_lock(&module_management_lock);
	list_del(&handler->link);
	mutex_unlock(&module_management_lock);
    
   return 0;
}
EXPORT_SYMBOL(yulong_unregister_module_management);

	



static ssize_t debug_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{


	int count,i=0;
    int state;
	struct module_management *pos;
	count = 0;
	mutex_lock(&module_management_lock);
	
	list_for_each_entry(pos, &module_management_handlers, link) {
		if(pos->name){
			i++;
             state = pos->get(pos);
			 printk(KERN_ERR "[%d]:%s %s\n",i,pos->name,((!state) ? "off" : ((state == 1) ? "on" : "unknow")));
			 count +=   sprintf(buf + count, "%s:%s,",pos->name, ((!state) ? "2" : ((state == 1) ? "1" : "0")));

		}
	}
	mutex_unlock(&module_management_lock);

   buf[count]  = '\0';
return count;

}





int handle_commad(struct module_management *pos, const char *cona){
 int state = -1;
 if(!strcmp(cona, "1")){	
        if(pos->control) {  							         								               
	     state = pos->control(pos,SET_MODULE_ON);	
         printk(KERN_ERR "\nSetting %s on is %s .\n",pos->name,(state ? "fail":"succeesful"));				
           }
  } else  if(!strcmp(cona, "2")){	
      if(pos->control) {  							         								               
		  state = pos->control(pos,SET_MODULE_OFF);
          printk(KERN_ERR "\nSetting %s off is %s .\n",pos->name,(state ? "fail":"succeesful"));				
          }
  } else {
										    
        printk(KERN_ERR " Parameter is error from userspace.\n");		
        } 
return state;
}



static ssize_t debug_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{

	char *name;
  char *cona;
	char buffer[64];
	int error = -1;
	struct module_management *pos;
  strcpy(buffer,buf);
	cona = strim(buffer);
    name = strsep(&cona, ":");	

    if(name == NULL){       
         printk(KERN_ERR "%s: input device name is NULL! \n", __func__);  
         return -1;        
      }                  
  mutex_lock(&module_management_lock);	        
	 list_for_each_entry(pos, &module_management_handlers, link) {
	 if(pos->name){		 
			 if (!strcmp(pos->name, name)){										    
                    if(cona == NULL)
                           cona = "state";		
		               error = handle_commad(pos, cona);	     
                       break;    
		    		      }
		    		 }
		    }	  			

	mutex_unlock(&module_management_lock);		
return (error ? error : n);	
}

static ssize_t power_modules_show(struct kobject *kobj, struct kobj_attribute *attr,
char *buf)
{
	int count,i=0;
	struct module_management *pos;
	count = 0;
	mutex_lock(&module_management_lock);
	
	list_for_each_entry(pos, &module_management_handlers, link) {
		if(pos->name){
			i++;
			printk(KERN_ERR "[%d]:%s %s\n",i,pos->name,pos->get(pos)?"on":"off");
			count +=   sprintf(buf + count, "[%d]:%s %s\n",i,pos->name, (pos->get(pos)?"on":"off"));
		}
	}
	mutex_unlock(&module_management_lock);
   dump_stack();
return count;

}

static ssize_t power_modules_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{

 printk(KERN_ERR "%s\n",__func__);
return n;
}


static ssize_t mmi_sleep_test_show(struct kobject *kobj, struct kobj_attribute *attr,
char *buf)
{
	int count = 0;

	count +=   sprintf(buf , "%d\n",mmi_sleep_test_flag);

return count;

}

static ssize_t mmi_sleep_test_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	int ret = 0;
	unsigned long data;

	ret = kstrtoul(buf, 0, &data);
	
	if (ret)
		return ret;
	
	mmi_sleep_test_flag = ( int )data;

 //printk(KERN_ERR "%s\n",__func__);
return n;
}

#ifdef CONFIG_APP_PROFILE
static ssize_t app_profile_disable_show(struct kobject *kobj, struct kobj_attribute *attr,
char *buf)
{
	int count = 0;

	count +=   sprintf(buf , "%d\n",app_profile_disable_flag);

return count;

}

static ssize_t app_profile_disable_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	int ret = 0;
	unsigned long data;

	ret = kstrtoul(buf, 0, &data);
	
	if (ret)
		return ret;
	
	app_profile_disable_flag = ( int )data;

 //printk(KERN_ERR "%s\n",__func__);
return n;
}
#endif
power_attri(debug);
power_attri(power_modules);
power_attri(mmi_sleep_test);
#ifdef CONFIG_APP_PROFILE

power_attri(app_profile_disable);
#endif

static struct attribute *group[] = {
	&debug_attr.attr,
	&power_modules_attr.attr,
	&mmi_sleep_test_attr.attr,
#ifdef CONFIG_APP_PROFILE
	&app_profile_disable_attr.attr,
#endif
	NULL,
};


static struct attribute_group group_attr = {
	.attrs = group,
};



static int __init yulong_module_management_init(void)
{


	return sysfs_create_group(power_kobj, &group_attr);
}
module_init(yulong_module_management_init);

static void __exit yulong_module_management_exit(void)
{
	return sysfs_remove_group(power_kobj, &group_attr);
}
module_exit(yulong_module_management_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("yulong_module_management");
MODULE_ALIAS("platform:" PM8XXX_SPK_DEV_NAME);

