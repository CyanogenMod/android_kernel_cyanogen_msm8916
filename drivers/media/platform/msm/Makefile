#
# Makefile for the QCOM spcific video device drivers
# based on V4L2.
#

obj-$(CONFIG_MSM_VIDC_V4L2)     += vidc/
obj-$(CONFIG_MSM_WFD) += wfd/
obj-y += broadcast/
obj-$(CONFIG_DVB_MPQ) += dvb/
ifeq ($(CONFIG_MACH_YULONG),y)
obj-$(CONFIG_MSMB_CAMERA) += camera_yulong_v2/
else
obj-$(CONFIG_MSMB_CAMERA) += camera_v2/
endif
obj-$(CONFIG_MSM_VPU) += vpu/
