////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2014 MStar Semiconductor, Inc.
// All rights reserved.
//
// Unless otherwise stipulated in writing, any and all information contained
// herein regardless in any format shall remain the sole proprietary of
// MStar Semiconductor Inc. and be kept in strict confidence
// (??MStar Confidential Information??) by the recipient.
// Any unauthorized act including without limitation unauthorized disclosure,
// copying, use, reproduction, sale, distribution, modification, disassembling,
// reverse engineering and compiling of the contents of MStar Confidential
// Information is unlawful and strictly prohibited. MStar hereby reserves the
// rights to any and all damages, losses, costs and expenses resulting therefrom.
//
////////////////////////////////////////////////////////////////////////////////

/**
 *
 * @file    mstar_drv_platform_porting_layer.h
 *
 * @brief   This file defines the interface of touch screen
 *
 *
 */

#ifndef __MSTAR_DRV_PLATFORM_PORTING_LAYER_H__
#define __MSTAR_DRV_PLATFORM_PORTING_LAYER_H__

/*--------------------------------------------------------------------------*/
/* INCLUDE FILE                                                             */
/*--------------------------------------------------------------------------*/

#include "mstar_drv_common.h"

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM)

#include <mach/board.h>
#include <mach/gpio.h>

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
#include <mach/regulator.h>
#include <linux/regulator/consumer.h>
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
#include <linux/input/vir_ps.h> 
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION

#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
#include <linux/regulator/consumer.h>
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

#ifdef CONFIG_ENABLE_NOTIFIER_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif //CONFIG_ENABLE_NOTIFIER_FB

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
#include <linux/input/vir_ps.h> 
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION


#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)

#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/hwmsen_helper.h>
//#include <linux/hw_module_info.h>

#include <linux/namei.h>
#include <linux/vmalloc.h>

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <mach/mt_gpio.h>

#include <cust_eint.h>
#include "tpd.h"
#include "cust_gpio_usage.h"
#include <pmic_drv.h>

#endif

/*--------------------------------------------------------------------------*/
/* PREPROCESSOR CONSTANT DEFINITION                                         */
/*--------------------------------------------------------------------------*/

/*
 * Note.
 * Please change the below GPIO pin setting to follow the platform that you are using(EX. MediaTek, Spreadtrum, Qualcomm).
 */
#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM)

// TODO : Please FAE colleague to confirm with customer device driver engineer about the value of RST and INT GPIO setting
#define MS_TS_MSG_IC_GPIO_RST   GPIO_TOUCH_RESET //53 //35 
#define MS_TS_MSG_IC_GPIO_INT   GPIO_TOUCH_IRQ   //52 //37

#ifdef CONFIG_TP_HAVE_KEY
#define TOUCH_KEY_MENU (139) //229
#define TOUCH_KEY_HOME (172) //102
#define TOUCH_KEY_BACK (158)
#define TOUCH_KEY_SEARCH (217)

#define MAX_KEY_NUM (4)
#endif //CONFIG_TP_HAVE_KEY

#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)

// TODO : Please FAE colleague to confirm with customer device driver engineer about the value of RST and INT GPIO setting
#define MS_TS_MSG_IC_GPIO_RST   0
#define MS_TS_MSG_IC_GPIO_INT   1

#ifdef CONFIG_TP_HAVE_KEY
#define TOUCH_KEY_MENU (139) //229
#define TOUCH_KEY_HOME (172) //102
#define TOUCH_KEY_BACK (158)
#define TOUCH_KEY_SEARCH (217)

#define MAX_KEY_NUM (4)
#endif //CONFIG_TP_HAVE_KEY

#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)

#define MS_TS_MSG_IC_GPIO_RST   (GPIO_CTP_RST_PIN)
#define MS_TS_MSG_IC_GPIO_INT   (GPIO_CTP_EINT_PIN)

#define TPD_POWER_SOURCE_CUSTOM MT6328_POWER_LDO_VGP1

#ifdef CONFIG_TP_HAVE_KEY
#define TOUCH_KEY_MENU    KEY_MENU 
#define TOUCH_KEY_HOME    KEY_HOMEPAGE 
#define TOUCH_KEY_BACK    KEY_BACK
#define TOUCH_KEY_SEARCH  KEY_SEARCH

#define MAX_KEY_NUM (4)
#endif //CONFIG_TP_HAVE_KEY

#endif

#ifdef TPD_MSTAR_PROXIMITY_SENSOR
extern int tpd_get_MStarps_value(void);
extern int tpd_MStar_Proximity_Sensor_operate(void* self, uint32_t command, void* buff_in, int size_in,
					   void* buff_out, int size_out, int* actualout);
extern int DrvFwCtrl_MStar_Proximity_sensor(int enable);
#endif

/*--------------------------------------------------------------------------*/
/* GLOBAL FUNCTION DECLARATION                                              */
/*--------------------------------------------------------------------------*/

extern void DrvPlatformLyrDisableFingerTouchReport(void);
extern void DrvPlatformLyrEnableFingerTouchReport(void);
extern void DrvPlatformLyrFingerTouchPressed(s32 nX, s32 nY, s32 nPressure, s32 nId);
extern void DrvPlatformLyrFingerTouchReleased(s32 nX, s32 nY);
extern s32 DrvPlatformLyrInputDeviceInitialize(struct i2c_client *pClient);
extern void DrvPlatformLyrSetIicDataRate(struct i2c_client *pClient, u32 nIicDataRate);
extern void DrvPlatformLyrTouchDevicePowerOff(void);
extern void DrvPlatformLyrTouchDevicePowerOn(void);
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
extern void DrvPlatformLyrTouchDeviceRegulatorPowerOn(void);
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON
extern void DrvPlatformLyrTouchDeviceRegisterEarlySuspend(void);
extern s32 DrvPlatformLyrTouchDeviceRegisterFingerTouchInterruptHandler(void);
extern s32 DrvPlatformLyrTouchDeviceRemove(struct i2c_client *pClient);
extern s32 DrvPlatformLyrTouchDeviceRequestGPIO(void);        
extern void DrvPlatformLyrTouchDeviceResetHw(void);
#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
extern int DrvPlatformLyrGetTpPsData(void);
#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
extern void DrvPlatformLyrTpPsEnable(int nEnable);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
extern int DrvPlatformLyrTpPsOperate(void* pSelf, u32 nCommand, void* pBuffIn, int nSizeIn, void* pBuffOut, int nSizeOut, int* pActualOut);
#endif
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION
        
#endif  /* __MSTAR_DRV_PLATFORM_PORTING_LAYER_H__ */
