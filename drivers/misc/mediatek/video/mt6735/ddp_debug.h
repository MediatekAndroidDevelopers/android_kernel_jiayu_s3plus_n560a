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

#ifndef __DDP_DEBUG_H__
#define __DDP_DEBUG_H__

#include "ddp_dump.h"
#include "ddp_mmp.h"

#define dprec_string_max_length         512
#define dprec_dump_max_length           (1024 * 16 * 4)
#define LOGGER_BUFFER_SIZE              (16 * 1024)
#define ERROR_BUFFER_COUNT              2
#define FENCE_BUFFER_COUNT              22
#define DEBUG_BUFFER_COUNT              4
#define DUMP_BUFFER_COUNT               2
#define STATUS_BUFFER_COUNT             1
#define DPREC_ERROR_LOG_BUFFER_LENGTH   \
	(1024 * 16 + LOGGER_BUFFER_SIZE * \
	(ERROR_BUFFER_COUNT + FENCE_BUFFER_COUNT + DEBUG_BUFFER_COUNT + DUMP_BUFFER_COUNT + STATUS_BUFFER_COUNT))

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

#define DISP_ENABLE_SODI_FOR_VIDEO_MODE

#endif				/* __DDP_DEBUG_H__ */
