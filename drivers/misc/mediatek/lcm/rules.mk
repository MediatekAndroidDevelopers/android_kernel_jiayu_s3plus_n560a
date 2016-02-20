#
# Makefile for misc devices that really don't fit anywhere else.
#
LOCAL_DIR := $(GET_LOCAL_DIR)

# Vanzo:wangfei on: Wed, 12 Nov 2014 21:08:36 +0800
# added for aosp management to import our variable
project_name:=$(shell echo $(VANZO_INNER_PROJECT_NAME))
ifneq ($(strip $(project_name)),)
-include $(srctree)/../zprojects/$(project_name)/$(project_name).mk
ccflags-y += -I$(VANZO_PROJECT_HEADERS)
# End of Vanzo:wangfei
# Vanzo:wangfei on: Tue, 23 Dec 2014 17:20:12 +0800
# here use the CUSTOM_LK_LCM to prio the CUSTOM_LK_LCM when zprojects is open
 
ifneq ($(strip $(CUSTOM_KERNEL_LCM)),)
CONFIG_CUSTOM_LK_LCM = $(CUSTOM_KERNEL_LCM)
endif
endif
# End of Vanzo:wangfei
LCM_DEFINES := $(shell echo $(CONFIG_CUSTOM_LK_LCM) | tr a-z A-Z)
DEFINES += $(foreach LCM,$(LCM_DEFINES),$(LCM))
DEFINES += MTK_LCM_PHYSICAL_ROTATION=\"$(MTK_LCM_PHYSICAL_ROTATION)\"

LCM_LISTS := $(subst ",,$(CONFIG_CUSTOM_LK_LCM))
OBJS += $(foreach LCM,$(LCM_LISTS),$(LOCAL_DIR)/$(LCM)/$(addsuffix .o, $(LCM)))
OBJS += $(LOCAL_DIR)/mt65xx_lcm_list.o


