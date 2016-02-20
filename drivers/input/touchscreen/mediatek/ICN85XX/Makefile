include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/ICN85XX/
ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/ICN85XX/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/ICN85XX/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  flash.o
obj-y	+=  icn85xx.o

