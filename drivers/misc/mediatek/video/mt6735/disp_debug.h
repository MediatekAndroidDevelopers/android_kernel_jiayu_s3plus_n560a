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

#ifndef __DISP_DEBUG_H
#define __DISP_DEBUG_H

#include "ddp_dump.h"

extern unsigned int gResetRDMAEnable;
extern unsigned int gOVLBackground;
extern unsigned int gEnableIRQ;
extern unsigned int gUltraEnable;
extern unsigned long int gRDMAUltraSetting;
extern unsigned long int gRDMAFIFOLen;

extern unsigned int disp_low_power_enlarge_blanking;
extern unsigned int disp_low_power_disable_ddp_clock;
extern unsigned int disp_low_power_disable_fence_thread;
extern unsigned int disp_low_power_remove_ovl;

extern unsigned int gSkipIdleDetect;
extern unsigned int gEnableSODIControl;
extern unsigned int gPrefetchControl;

extern unsigned int gEnableSWTrigger;
extern unsigned int gEnableMutexRisingEdge;
extern unsigned int gDisableSODIForTriggerLoop;

extern unsigned int gDumpConfigCMD;
extern unsigned int gDumpESDCMD;

extern unsigned int gResetOVLInAALTrigger;
extern unsigned int gDisableOVLTF;

extern unsigned int gDumpMemoutCmdq;

extern unsigned int g_mobilelog;
extern unsigned int g_fencelog;
extern unsigned int g_loglevel;
extern unsigned int g_rcdlevel;

#define MSG_FUNC_ENTER()
#define MSG_FUNC_LEAVE()

unsigned int ddp_debug_analysis_to_buffer(void);

#endif				/* __MTKFB_DEBUG_H */
