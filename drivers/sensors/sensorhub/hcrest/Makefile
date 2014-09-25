# Support for Hillcrest Labs sensor hub

ifneq ($(KERNELRELEASE),)

# Kbuild configuration

# get the sensorhub sub project's version or check dist-version for version info
my_src_dir := $(shell dirname $(lastword $(MAKEFILE_LIST)))
my_git_ver := $(shell \
		cd $(my_src_dir); \
		( [ -d .git/ ] && ( git describe --dirty --always  ))  \
		|| cat dist-version || echo unknown )

obj-$(CONFIG_SENSOR_HUB_CORE) += sensorhub-core.o
obj-$(CONFIG_SENSOR_HUB_I2C)  += sensorhub-i2c.o
obj-$(CONFIG_SENSOR_HUB_SPI)  += sensorhub-spi.o

ccflags-$(CONFIG_SENSOR_HUB_DEBUG) +=-DDEBUG -DSENSORHUB_VERSION=\"$(my_git_ver)\"

else

# Building as External Modules
#
# Build Vars:
# 	KERNEL_SRC - Location of Kernel Source
# 	KERNEL_OBJ - Location of Kernel Build

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build

KERNEL_OBJ ?=

ifdef KERNEL_OBJ
KERNELOUTPARAM := O=$(KERNEL_OBJ)
endif

all:	modules

modules:
	$(MAKE) -C $(KERNEL_SRC) $(KERNELOUTPARAM) M=$(PWD) modules

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions modules.order Module.symvers *.tmp *.log

push: all
	@[ -f sensorhub-core.ko ] && adb push sensorhub-core.ko /vendor/lib/modules/ || true
	@[ -f sensorhub-i2c.ko  ] && adb push sensorhub-i2c.ko /vendor/lib/modules/  || true
	@[ -f sensorhub-spi.ko  ] && adb push sensorhub-spi.ko /vendor/lib/modules/  || true

pushscripts:
	adb push scripts /system/xbin/

pushall: all push pushscripts


endif
