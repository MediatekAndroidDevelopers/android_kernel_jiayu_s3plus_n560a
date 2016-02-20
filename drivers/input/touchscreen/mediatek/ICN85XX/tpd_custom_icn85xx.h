/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

#ifndef TOUCHPANEL_H__
#define TOUCHPANEL_H__

#ifdef MT6572
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#endif

#define TPD_SLIDE_WAKEUP                    1
#define TPD_TYPE_CAPACITIVE
#define TPD_TYPE_RESISTIVE
#define TPD_POWER_SOURCE
#define TPD_I2C_NUMBER				0
#define TPD_WAKEUP_TRIAL			60
#define TPD_WAKEUP_DELAY			100

//#define TPD_ROTATE_90

#define TPD_POWER_SOURCE_CUSTOM	MT6323_POWER_LDO_VGP1

#define TPD_OK   0
#define TPD_ICN85XX_SUPPORT_PROC_FS		0//1
#define TPD_ICN85XX_SUPPORT_SYSFS		1
#define TPD_ICN85XX_SUPPORT_FW_UPDATE		1
#define TPD_ICN85XX_COMPILE_FW_WITH_DRIVER	1
#define TPD_ICN85XX_FORCE_UPDATE_FW		0
#define CTP_REPORT_PROTOCOL   0

#define TPD_ICN85XX_I2C_DMA_SUPPORT    1

#define TPD_ICN85XX_I2C_ADDR			0x90
#define TPD_ICN85XX_PROG_I2C_ADDR		(0x60 >> 1)

#define TPD_DELAY				(2 * HZ / 100)
#define TPD_RES_X               720
#define TPD_RES_Y                1280
#define TPD_CALIBRATION_MATRIX			{962, 0, 0, 0, 1600, 0, 0, 0};


#define TPD_HAVE_BUTTON
#define TPD_KEY_COUNT	3
#define TPD_KEYS		{KEY_MENU, KEY_HOMEPAGE, KEY_BACK}
/* {button_center_x, button_center_y, button_width, button_height*/
#define TPD_KEYS_DIM	{{120, 1380, 60, 40},{360, 1380, 100, 40},{600, 1380, 60, 40}}


#endif

