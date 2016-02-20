include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/focaltech

obj-y	+=  focaltech_driver.o
obj-y	+=  focaltech_ex_fun.o
obj-y	+=  focaltech_ctl.o

# Vanzo:songlixin on: Sun, 20 Apr 2014 13:49:02 +0800
ifeq ($(VANZO_TOUCHPANEL_GESTURES_SUPPORT), yes)
obj-y += ft_gesture_lib.o
endif
# End of Vanzo:songlixin
