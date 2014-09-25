#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/yl_phone_mode.h>

static char yl_phone_mode[4] = {'0', '0', '0', '0'};

int yl_phone_mode_is_xianwang(void)
{
	return yl_phone_mode[0] == COMM_RUN_MODE_XIANWANG;
}

static int __init yl_phone_mode_setup(char *mode)
{
	int size;

	size = strlen(mode);
	if (size > 4)
		size = 4;
	memcpy(yl_phone_mode, mode, size);
	return 1;
}
__setup("yl_phone_mode=", yl_phone_mode_setup);
