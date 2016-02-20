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

/*****************************************************************************
 *  Copyright Statement:
 *  --------------------
 *  This software is protected by Copyright and the information contained
 *  herein is confidential. The software may not be copied and the information
 *  contained herein may not be used or disclosed except with the written
 *  permission of MediaTek Inc. (C) 2008
 *
 *  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 *  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 *  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
 *  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 *  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 *  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 *  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 *  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
 *  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
 *  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
 *  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
 *
 *  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
 *  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 *  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 *  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
 *  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 *  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
 *  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
 *  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
 *  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
 *  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
 *
 *****************************************************************************/
#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#include <linux/xlog.h>
#include <mach/mt_pm_ldo.h>
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)


#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

#ifndef FALSE
#define FALSE 0
#endif

//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
#define LCM_DSI_CMD_MODE									0
#define LCM_ID_HX8394 0x94
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_init_power(void)
{
}

static void lcm_suspend_power(void)
{
}

static void lcm_resume_power(void)
{
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	// enable tearing-free
	params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

	//params->dsi.mode   = BURST_VDO_MODE; 
	params->dsi.mode   = SYNC_EVENT_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.word_count=720*3;
	params->dsi.vertical_sync_active				= 2;//2//
	//params->dsi.vertical_backporch					= 5;//8
	//params->dsi.vertical_frontporch					= 20;//6
	params->dsi.vertical_backporch					= 12;//8
	params->dsi.vertical_frontporch					= 18;//6
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 
	params->dsi.horizontal_sync_active				= 80;//86 20
	params->dsi.horizontal_backporch				= 80;//55 50
	params->dsi.horizontal_frontporch				= 80;//55	50
	params->dsi.horizontal_active_pixel			= FRAME_WIDTH;

        params->dsi.PLL_CLOCK = 208;
}

static void lcm_init(void)
{
	unsigned int data_array[16];

	data_array[0]=0x00043902;
	data_array[1]=0x9483ffb9;//b0
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]=0x00033902;
	data_array[1]=0x008333ba;//b3
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]=0x00103902;
	data_array[1]=0x12126cb1;//b1
	data_array[2]=0xf1110426;//
	data_array[3]=0x23543a81;//
	data_array[4]=0x58d2c080;//b4//58d28080
	dsi_set_cmdq(data_array, 5, 1);
	
	data_array[0]=0x000d3902;
	data_array[1]=0x51ff00b4;//b4
	data_array[2]=0x035a595a;//b4
	data_array[3]=0x0170015a;//b4
	data_array[4]=0x00000070;//b4
	dsi_set_cmdq(data_array, 5, 1);
		
	/*data_array[0]=0x000c3902;
	data_array[1]=0x0e6400b2;//b4
	data_array[2]=0x081c220d;//b4
	data_array[3]=0x004d1c08;//b4
	dsi_set_cmdq(data_array, 4, 1);*/

	data_array[0]=0x00023902;
	data_array[1]=0x000007bc;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0]=0x00043902;
	data_array[1]=0x010e41bF;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]=0x00263902;
	data_array[1]=0x000f00d3;//b4
	data_array[2]=0x00100740;
	data_array[3]=0x00081008;
	data_array[4]=0x0e155408;
	data_array[5]=0x15020e05;
	data_array[6]=0x47060506;
	data_array[7]=0x4b0a0a44;
	data_array[8]=0x08070710;
	data_array[9]=0x0a000000;
	data_array[10]=0x0000000;
	dsi_set_cmdq(data_array, 11, 1);

	
	data_array[0]=0x002d3902;
	data_array[1]=0x1b1a1ad5;
	data_array[2]=0x0201001b;
	data_array[3]=0x06050403;
	data_array[4]=0x0a090807;
	data_array[5]=0x1825240b;
	data_array[6]=0x18272618;
	data_array[7]=0x18181818;
	data_array[8]=0x18181818;
	data_array[9]=0x18181818;
	data_array[10]=0x20181818;
	data_array[11]=0x18181821;
	data_array[12]=0x00000018;
	dsi_set_cmdq(data_array, 13, 1);

	data_array[0]=0x002d3902;
	data_array[1]=0x1b1a1ad6;
	data_array[2]=0x090a0b1b;
	data_array[3]=0x05060708;
	data_array[4]=0x01020304;
	data_array[5]=0x58202100;
	data_array[6]=0x18262758;
	data_array[7]=0x18181818;
	data_array[8]=0x18181818;
	data_array[9]=0x18181818;
	data_array[10]=0x25181818;
	data_array[11]=0x18181824;
	data_array[12]=0x00000018;
	dsi_set_cmdq(data_array, 13, 1);
	
	
	data_array[0]=0x00023902;
	data_array[1]=0x000009cc;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0]=0x00033902;
	data_array[1]=0x001430c0;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0]=0x00053902;
	data_array[1]=0x40c000c7;
	data_array[2]=0x000000c0;
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]=0x00033902;
	data_array[1]=0x006c6cb6;
	dsi_set_cmdq(data_array, 2, 1);
		
    data_array[0]=0x002b3902;
data_array[1]=0x171300e0;
data_array[2]=0x223f3a33;
data_array[3]=0x0d0c093b;
data_array[4]=0x13110e18;
data_array[5]=0x11071211;
data_array[6]=0x13001713;
data_array[7]=0x3f3a3317;
data_array[8]=0x0c093b22;
data_array[9]=0x110e180d;
data_array[10]=0x07121113;
data_array[11]=0x00171311;
	dsi_set_cmdq(data_array, 12, 1);
	
	data_array[0]=0x00023902;
	data_array[1]=0x000000bd;//b0
	dsi_set_cmdq(data_array, 2, 1);
	

	data_array[0]=0x002c3902;
	data_array[1]=0x060001c1;
	data_array[2]=0x271d140c;
	data_array[3]=0x4941382f;
	data_array[4]=0x69615951;
	data_array[5]=0x89817971;
	data_array[6]=0xa9a19991;
	data_array[7]=0xcac1b9b2;
	data_array[8]=0xeae2d8d1;
	data_array[9]=0x38fff7f0;
	data_array[10]=0xc10b3ffc;
	data_array[11]=0xc00df113;
	dsi_set_cmdq(data_array, 12, 1);
	
	data_array[0]=0x00023902;
	data_array[1]=0x000001bd;//b0
	dsi_set_cmdq(data_array, 2, 1);
	
	
	data_array[0]=0x002b3902;
	data_array[1]=0x0c0600c1;
	data_array[2]=0x2f271d14;
	data_array[3]=0x51494138;
	data_array[4]=0x71696159;
	data_array[5]=0x91898179;
	data_array[6]=0xb2a9a199;
	data_array[7]=0xd1cac1b9;
	data_array[8]=0xf0eae2d8;
	data_array[9]=0xfc38fff7;
	data_array[10]=0x13c10b3f;
	data_array[11]=0x00c00df1;
	dsi_set_cmdq(data_array, 12, 1);
	
	data_array[0]=0x00023902;
	data_array[1]=0x000002bd;//b0
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]=0x002b3902;
	data_array[1]=0x0c0600c1;
	data_array[2]=0x2f271d14;
	data_array[3]=0x51494138;
	data_array[4]=0x71696159;
	data_array[5]=0x91898179;
	data_array[6]=0xb2a9a199;
	data_array[7]=0xd1cac1b9;
	data_array[8]=0xf0eae2d8;
	data_array[9]=0xfc38fff7;
	data_array[10]=0x13c10b3f;
	data_array[11]=0x00c00df1;
	dsi_set_cmdq(data_array, 12, 1);
	
	data_array[0]=0x00023902;
	data_array[1]=0x00000336;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x00110500; //0x11,exit sleep mode,1byte
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(200);

	data_array[0] = 0x00290500; //0x11,exit sleep mode,1byte
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(10);
}                                       

static void lcm_suspend(void)
{
	//push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	unsigned int data_array[16];

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(150);

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(20);

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(120);
}


static void lcm_resume(void)
{
#ifndef BUILD_LK
	lcm_init();
	//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
#endif
}

static unsigned int lcm_compare_id(void)
{
	unsigned int id,id0,id1, id2, id3,id4;
	unsigned char buffer[5];
	unsigned int array[5];

	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(150);

	array[0]=0x00043902;
	array[1]=0x9483FFB9;// page enable
	dsi_set_cmdq(&array, 2, 1);
	MDELAY(10);

	array[0]=0x00023902;
	array[1]=0x000013ba;
	dsi_set_cmdq(&array, 2, 1);
	MDELAY(10);

	array[0] = 0x00023700;// return byte number
	dsi_set_cmdq(&array, 1, 1);
	MDELAY(10);

	read_reg_v2(0xF4, buffer, 2);
	id0 = buffer[0]; 
	read_reg_v2(0xdc, buffer, 2);
	id1 = buffer[0];//1a 0d
	id = id0;

	return (LCM_ID_HX8394 == id)?1:0;
}

static unsigned int lcm_esd_check(void)
{
	int temp0=0,temp1=0,temp2=0,temp3=0;

#ifndef BUILD_LK
	char  buffer[3];
	int   array[4];

	//printk("xcz enter lcm_esd_check---\n");

	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x0a, buffer, 1);
        temp0 = buffer[0];
        read_reg_v2(0x09, buffer, 3);
        temp1 = buffer[0];//0x80
        temp2 = buffer[1];//0x73

	//printk("xcz enter lcm_esd_check---, temp0 = %x, temp1 = %x, temp2 = %x,\n", temp0, temp1, temp2);
	if((0x1c == temp0)&&(0x80 == temp1)&&(0x73 == temp2))
	{
		//printk("%s %d\n FALSE", __func__, __LINE__);
		return FALSE;
	}
	else
	{		 
		return TRUE;
	}
#endif
}

static unsigned int lcm_esd_recover(void)
{
#ifndef BUILD_LK
	//printk("xcz enter lcm_esd_recover---\n");
#endif

	lcm_init();	
	return TRUE;
}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER hx8394d_cmi50_xl_hd_lcm_drv =
{
	.name           = "hx8394d_cmi50_xl_hd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,
	//.esd_check	= lcm_esd_check,
	//.esd_recover	= lcm_esd_recover,
	.init_power        = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
};
