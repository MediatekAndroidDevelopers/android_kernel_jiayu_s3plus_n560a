include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/msg22xx/
ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/msg22xx/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/msg22xx/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  mstar_drv_common.o
obj-y	+=  mstar_drv_ic_fw_porting_layer.o
obj-y	+=  mstar_drv_main.o
obj-y	+=  mstar_drv_mtk.o
obj-y	+=  mstar_drv_platform_interface.o
obj-y	+=  mstar_drv_platform_porting_layer.o
obj-y	+=  mstar_drv_utility_adaption.o
obj-y	+=  mstar_drv_self_fw_control.o
obj-y	+=  mstar_drv_self_mp_test.o
