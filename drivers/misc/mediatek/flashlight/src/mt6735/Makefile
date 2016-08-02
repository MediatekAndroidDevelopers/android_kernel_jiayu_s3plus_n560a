#
# Makefile for misc devices that really don't fit anywhere else.
#
include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-y				+= kd_flashlightlist.o

obj-y				+= strobe_main_sid2_part2.o
obj-y				+= strobe_sub_sid2_part1.o
obj-y				+= strobe_sub_sid2_part2.o
obj-y				+= sub_strobe.o
obj-y				+= strobe_main_sid1_part2.o
obj-y				+= strobe_part_id.o
obj-y				+= strobe_sub_sid1_part2.o

ifeq ($(wildcard $(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/flashlight/strobe_main_sid2_part1.c),)
obj-y				+= strobe_main_sid2_part1.o
endif

ifeq ($(wildcard $(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/flashlight),)
#ifeq ($(wildcard $(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/flashlight),)


# Vanzo:wangfei on: Wed, 12 Nov 2014 21:06:42 +0800
project_name:=$(shell echo $(CONFIG_VANZO_INNER_PROJECT_NAME))
ifeq ($(project_name),)
# End of Vanzo:wangfei

custom_kernel_flashlight := $(addsuffix /, $(shell echo $(CONFIG_CUST_KERNEL_FLASHLIGHT)))

obj-y += $(custom_kernel_flashlight)
# Vanzo:wangfei on: Wed, 14 Jan 2015 15:01:47 +0800
# added flashlight custom
endif
# End of Vanzo:wangfei




endif
