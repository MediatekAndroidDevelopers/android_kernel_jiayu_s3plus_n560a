#
# Makefile for misc devices that really don't fit anywhere else.
#
include $(srctree)/drivers/misc/mediatek/Makefile.custom

define FILTER_PROJECT_DRV
ifeq ($(wildcard $(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/imgsensor/$(1)),)
platform_drv += $(1)
endif
endef

obj-y				+= kd_sensorlist.o
# Vanzo:wangfei on: Wed, 12 Nov 2014 21:06:42 +0800
# for compatible the old style ,so use CUSTOM_KERNEL_IMGSENSOR
custom_kernel_imgsensor := $(addsuffix /, $(shell echo $(CUSTOM_KERNEL_IMGSENSOR)))
ifeq ($(custom_kernel_imgsensor),)
# End of Vanzo:wangfei

$(foreach drv,$(subst $\",,$(CONFIG_CUSTOM_KERNEL_IMGSENSOR)),$(eval $(call FILTER_PROJECT_DRV,$(drv))))
custom_kernel_imgsensor := $(addsuffix /, $(shell echo $(platform_drv)))
# Vanzo:wangfei on: Thu, 13 Nov 2014 16:15:59 +0800
# added for project config prio the defconfig
endif
# End of Vanzo:wangfei

$(info "the custom_kernel_imgsensor:$(custom_kernel_imgsensor)")
obj-y += $(custom_kernel_imgsensor)
$(info imgsensor drv by platform $(platform_drv))
