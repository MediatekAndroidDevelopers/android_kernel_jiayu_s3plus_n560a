/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "ddp_debug.h"

unsigned int gResetRDMAEnable = 1;
unsigned int gOVLBackground = 0x0;
unsigned int gEnableIRQ = 0;
unsigned int gUltraEnable = 1;
unsigned long int gRDMAUltraSetting = 0;
unsigned long int gRDMAFIFOLen = 32;

unsigned int disp_low_power_enlarge_blanking = 0;
unsigned int disp_low_power_disable_ddp_clock = 0;
unsigned int disp_low_power_disable_fence_thread = 0;
unsigned int disp_low_power_remove_ovl = 1;

unsigned int gSkipIdleDetect = 0;
#ifdef DISP_ENABLE_SODI_FOR_VIDEO_MODE
unsigned int gEnableSODIControl = 1;
  /* workaround for SVP IT, todo: please K fix it */
#if defined(CONFIG_TRUSTONIC_TEE_SUPPORT) && defined(CONFIG_MTK_SEC_VIDEO_PATH_SUPPORT)
unsigned int gPrefetchControl = 0;
#else
unsigned int gPrefetchControl = 1;
#endif
#else
unsigned int gEnableSODIControl = 0;
unsigned int gPrefetchControl = 0;
#endif

unsigned int gEnableSWTrigger = 0;
unsigned int gEnableMutexRisingEdge = 0;
unsigned int gDisableSODIForTriggerLoop = 1;

unsigned int gDumpConfigCMD = 0;
unsigned int gDumpESDCMD = 0;

unsigned int gResetOVLInAALTrigger = 0;
unsigned int gDisableOVLTF = 0;

unsigned int gDumpMemoutCmdq = 0;

unsigned int gEnableReduceRegWrite = 0;
unsigned int gEnableDSIStateCheck = 0;
unsigned int gMutexFreeRun = 1;

// copied from mtkfb_debug.c
unsigned int g_mobilelog = 1;
unsigned int g_fencelog = 0;
unsigned int g_loglevel = 3;
unsigned int g_rcdlevel = 0;

unsigned int ddp_debug_analysis_to_buffer(void)
{
	return 0;
}
