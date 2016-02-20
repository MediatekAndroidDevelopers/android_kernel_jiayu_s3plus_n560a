include $(srctree)/drivers/misc/mediatek/Makefile.custom
ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/include/trustzone/utils/tlutils \
                   -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/include/trustzone/spi/Tlspi/inc
endif

ifeq ($(CONFIG_TRUSTONIC_TEE_SUPPORT),y)
ccflags-y += -I$(srctree)/drivers/misc/mediatek/gud/$(MTK_PLATFORM)/gud/MobiCoreKernelApi/include \
             -I$(srctree)/drivers/misc/mediatek/gud/$(MTK_PLATFORM)/gud/MobiCoreKernelApi/public 
endif

obj-$(CONFIG_MTK_SPI) += spi.o spi-dev.o
# Vanzo:dingge on: Tue, 21 Apr 2015 15:57:44 +0800
# add fingerprint framework

project_name:=$(shell echo $(VANZO_INNER_PROJECT_NAME))
ifneq ($(project_name),)

custom_kernel_fingerprint:= $(addprefix fingerprint/,$(addsuffix /, $(shell echo $(CUSTOM_KERNEL_FINGERPRINT))))

$(info "custom_kernel_fingerprint:$(custom_kernel_fingerprint)")
obj-y += $(custom_kernel_fingerprint)

endif

# End of Vanzo: dingge
