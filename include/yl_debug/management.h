

#ifndef _LINUX_YULONG_MODULE_MANAGEMENT_H
#define _LINUX_YULONG_MODULE_MANAGEMENT_H
#include <linux/kobject.h>




/* The early_suspend structure defines suspend and resume hooks to be called
 * when the user visible sleep state of the system changes, and a level to
 * control the order. They can be used to turn off the screen and input
 * devices that are not used for wakeup.
 * Suspend handlers are called in low to high level order, resume handlers are
 * called in the opposite order. If, when calling register_early_suspend,
 * the suspend handlers have already been called without a matching call to the
 * resume handlers, the suspend handler will be called directly from
 * register_early_suspend. This direct call can violate the normal level order.
 */
enum {
	SET_MODULE_OFF = 0,
	SET_MODULE_ON = 1,
	STATUS_ON=1,
	STATUS_OFF=0
};

struct module_management{

	struct list_head link;
	const char  *name;
	int (*get)(struct module_management *h);
	int (*control)(struct module_management *h,unsigned int cmd);
	void * private_data;

};

struct modules_state{

	 char  name[15];
	 unsigned char value;

};

#define power_attri(_name) \
 struct kobj_attribute _name##_attr = {	\
	.attr	= {				\
		.name = __stringify(_name),	\
		.mode = 0644,			\
	},					\
	.show	= _name##_show,			\
	.store	= _name##_store,		\
}

int yulong_register_module_management(struct module_management *handler);
int yulong_unregister_module_management(struct module_management *handler);




#endif//_LINUX_YULONG_MODULE_MANAGEMENT_H
