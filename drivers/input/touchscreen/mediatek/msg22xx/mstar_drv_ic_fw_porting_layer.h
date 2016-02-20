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
 * @file    mstar_drv_ic_fw_porting_layer.h
 *
 * @brief   This file defines the interface of touch screen
 *
 *
 */

#ifndef __MSTAR_DRV_IC_FW_PORTING_LAYER_H__
#define __MSTAR_DRV_IC_FW_PORTING_LAYER_H__

/*--------------------------------------------------------------------------*/
/* INCLUDE FILE                                                             */
/*--------------------------------------------------------------------------*/

#include "mstar_drv_common.h"
#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
#include "mstar_drv_mutual_fw_control.h"
#ifdef CONFIG_ENABLE_ITO_MP_TEST
#include "mstar_drv_mutual_mp_test.h"
#endif //CONFIG_ENABLE_ITO_MP_TEST
#elif defined(CONFIG_ENABLE_CHIP_MSG21XXA) || defined(CONFIG_ENABLE_CHIP_MSG22XX)
#include "mstar_drv_self_fw_control.h"
#ifdef CONFIG_ENABLE_ITO_MP_TEST
#include "mstar_drv_self_mp_test.h"
#endif //CONFIG_ENABLE_ITO_MP_TEST
#endif

/*--------------------------------------------------------------------------*/
/* PREPROCESSOR CONSTANT DEFINITION                                         */
/*--------------------------------------------------------------------------*/


/*--------------------------------------------------------------------------*/
/* GLOBAL FUNCTION DECLARATION                                              */
/*--------------------------------------------------------------------------*/

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
extern void DrvIcFwLyrOpenGestureWakeup(u32 *pWakeupMode);
extern void DrvIcFwLyrCloseGestureWakeup(void);

#ifdef CONFIG_ENABLE_GESTURE_DEBUG_MODE
extern void DrvIcFwLyrOpenGestureDebugMode(u8 nGestureFlag);
extern void DrvIcFwLyrCloseGestureDebugMode(void);
#endif //CONFIG_ENABLE_GESTURE_DEBUG_MODE

#endif //CONFIG_ENABLE_GESTURE_WAKEUP

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
extern u16 DrvIcFwLyrChangeFirmwareMode(u16 nMode);
extern void DrvIcFwLyrGetFirmwareInfo(FirmwareInfo_t *pInfo);
#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
extern u16 DrvIcFwLyrGetFirmwareMode(void);
#endif //CONFIG_ENABLE_CHIP_MSG26XXM
extern void DrvIcFwLyrRestoreFirmwareModeToLogDataMode(void);
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
extern void DrvIcFwLyrCheckFirmwareUpdateBySwId(void);
#endif //CONFIG_UPDATE_FIRMWARE_BY_SW_ID

extern void DrvIcFwLyrOptimizeCurrentConsumption(void);
extern u8 DrvIcFwLyrGetChipType(void);
extern void DrvIcFwLyrGetCustomerFirmwareVersion(u16 *pMajor, u16 *pMinor, u8 **ppVersion);
extern void DrvIcFwLyrGetPlatformFirmwareVersion(u8 **ppVersion);
extern void DrvIcFwLyrHandleFingerTouch(u8 *pPacket, u16 nLength);
extern u32 DrvIcFwLyrIsRegisterFingerTouchInterruptHandler(void);
extern s32 DrvIcFwLyrUpdateFirmware(u8 szFwData[][1024], EmemType_e eEmemType);
extern s32 DrvIcFwLyrUpdateFirmwareBySdCard(const char *pFilePath);

#ifdef CONFIG_ENABLE_ITO_MP_TEST
extern void DrvIcFwLyrCreateMpTestWorkQueue(void);
extern void DrvIcFwLyrScheduleMpTestWork(ItoTestMode_e eItoTestMode);
extern void DrvIcFwLyrGetMpTestDataLog(ItoTestMode_e eItoTestMode, u8 *pDataLog, u32 *pLength);
extern void DrvIcFwLyrGetMpTestFailChannel(ItoTestMode_e eItoTestMode, u8 *pFailChannel, u32 *pFailChannelCount);
extern s32 DrvIcFwLyrGetMpTestResult(void);
#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
extern void DrvIcFwLyrGetMpTestScope(TestScopeInfo_t *pInfo);
#endif //CONFIG_ENABLE_CHIP_MSG26XXM
#endif //CONFIG_ENABLE_ITO_MP_TEST

#ifdef CONFIG_ENABLE_SEGMENT_READ_FINGER_TOUCH_DATA
#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
extern void DrvIcFwLyrGetTouchPacketAddress(u16 *pDataAddress, u16 *pFlagAddress);
#endif //CONFIG_ENABLE_CHIP_MSG26XXM
#endif //CONFIG_ENABLE_SEGMENT_READ_FINGER_TOUCH_DATA

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
extern s32 DrvIcFwLyrEnableProximity(void);
extern s32 DrvIcFwLyrDisableProximity(void);
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION

#endif  /* __MSTAR_DRV_IC_FW_PORTING_LAYER_H__ */
