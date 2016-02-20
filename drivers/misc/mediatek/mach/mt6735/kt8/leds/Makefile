
include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Vanzo:wangfei on: Wed, 12 Nov 2014 21:06:42 +0800
project_name:=$(shell echo $(VANZO_INNER_PROJECT_NAME))
ifneq ($(project_name),)
custom_kernel_leds:= $(addsuffix /, $(shell echo $(CUSTOM_KERNEL_LEDS)))
$(info "the custom_kernel_leds:$(custom_kernel_leds)")
ifneq ($(custom_kernel_leds),)
obj-y += $(custom_kernel_leds)
else
$(info "Warning!Why custom_kernel_leds is empty?")
endif
else
# End of Vanzo:wangfei
obj-$(CONFIG_MTK_LEDS) += mt65xx/

# Vanzo:wangfei on: Wed, 14 Jan 2015 17:37:53 +0800
# for add leds custom
endif
# End of Vanzo:wangfei
