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

#ifdef BUILD_LK
    #include <string.h>
    #include <platform/mt_pmic.h>
	#include <platform/mt_gpio.h>
    #include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#else
    #include <linux/string.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/upmu_common.h>

    #if defined(BUILD_LK)
        #include <asm/arch/mt_gpio.h>
    #else
        #include <mach/mt_gpio.h>
    #endif
#endif
#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(854)
#define LCM_ID   (0x800902)

#define REGFLAG_DELAY             							0XFFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#define SET_GPIO_OUT(n, v)  (lcm_util.set_gpio_out((n), (v)))
#define read_reg_v2(cmd, buffer, buffer_size)  lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static void lcm_init_power(void)
{
}

static void lcm_suspend_power(void)
{
}

static void lcm_resume_power(void)
{
}


static void lcd_power_en(unsigned char enabled)
{
#if 0
    if (enabled)
    {
    

		hwPowerOn(MT65XX_POWER_LDO_VGP6, VOL_3300, "LCM");  //lcd_power_en(1);
   

        //       mt_set_gpio_out(GPIO_LCM_PWR, GPIO_OUT_ZERO);
    }
#endif
}

static struct LCM_setting_table lcm_initialization_setting[] = {
{0x00, 1 , {0x00}},
{0xFF,  3 ,{0x80,0x09,0x01}},// enable EXTC

{0x00, 1 , {0x80}},// shift address
{0xFF,  2 ,{0x80,0x09}},// enable Orise mode
	
{0x00, 1 , {0x03}},
{0xFF, 1 , {0x01}},	  
	
{0x00, 1 , {0x90}},
{0xB3, 1 , {0x02}},
	
{0x00, 1 , {0x92}},
{0xB3, 1 , {0x45}},

{0x00, 1 , {0xA6}},
{0xB3, 2 , {0x20,0x01}},	
	
{0x00, 1 , {0xA3}},
{0xC0, 1 , {0x1B}},

{0x00, 1 , {0xB4}},// Column Inversion
{0xC0, 1 , {0x50}},//10

{0x00, 1 , {0x81}},
{0xC4, 1 , {0x04}},	
	
{0x00, 1 , {0x80}},
{0xC5, 1 , {0x03}},

{0x00, 1 , {0x90}},// mclk shift 
{0xC0, 6 , {0x00,0x44,0x00,0x00,0x00,0x03}},


{0x00, 1 , {0xA6}},// Horizontal sync shift 
{0xC1, 3 , {0x01,0x00,0x00}},			
	
{0x00, 1 , {0xA0}},
{0xC1, 1 , {0xEA}},
	
{0x00, 1 , {0x8B}},// for ULPM mode
{0xB0, 1 , {0x40}},

{0x00, 1 , {0x82}},
{0xC5, 1 , {0x03}},
	
{0x00, 1 , {0x90}},
//{0xC5, 5 , {0x96,0x2B,0x04,0x7B,0x33}},
{0xC5, 6 , {0xd6,0x2B,0x04,0x7B,0x55,0x55}},
	
{0x00, 1 , {0x00}},// GVDD
{0xD8, 2 , {0x8f,0x8f}},//70
	

{0x00, 1 , {0x00}},
{0xD9, 1 , {0x1d}},//2f

{0x00, 1 , {0x81}},
{0xC1, 1 , {0x3D}},	

{0x00, 1 , {0x00}},
{0xE1, 16 , {0x08,0x13,0x19,0x0D,0x06,0x0D,0x0A,0x08,0x05,0x08,0x0E,0x09,0x0F,0x0D,0x07,0x03}},//2.2+ 
                       								// 255 251  247  239   231  203  175  147  108  80   52   
{0x00, 1 , {0x00}},
{0xE2, 16 , {0x08,0x13,0x19,0x0D,0x06,0x0D,0x0A,0x08,0x05,0x08,0x0E,0x09,0x0F,0x0D,0x07,0x03}}, //2.2-   
                      								//V0     V4	V8  V16   V24   V52  V80  V108 V147 V175 
{0x00, 1 , {0x80}},// shift address
{0xCE, 12 , {0x86,0x01,0x00,0x85,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
													
{0x00, 1 , {0xa0}},// shift address
{0xCE, 14 , {0x18,0x03,0x03,0x5a,0x00,0x00,0x00,0x18,0x02,0x03,0x5b,0x00,0x00,0x00}},

{0x00, 1 , {0xb0}},// shift address
{0xCE, 14, {0x18,0x05,0x03,0x5c,0x00,0x00,0x00,0x18,0x04,0x03,0x5B,0x00,0x00,0x00}},	
	

//CECx : clkb1, clkb2
{0x00, 1 , {0xc0}},// shift address
{0xCE, 14 , {0x18,0x05,0x03,0x5A,0x00,0x00,0x00,0x18,0x04,0x03,0x5b,0x00,0x00,0x00}},

//CEDx : clkb3, clkb4
{0x00, 1 , {0xd0}},// shift address
{0xCE, 14 , {0x18,0x03,0x03,0x5C,0x00,0x00,0x00,0x18,0x02,0x03,0x5d,0x00,0x00,0x00}},
	
//CFCx : 
{0x00, 1 , {0xc0}},// shift address
{0xCf, 10 , {0x01,0x01,0x20,0x20,0x00,0x00,0x01,0x00,0x00,0x00}},
//CFDx : 

{0x00, 1 , {0xd0}},
{0xcf, 1 , {0x00}},	

	
//--------------------------------------------------------------------------------
//		initial setting 3 < Panel setting >
//--------------------------------------------------------------------------------

// CB8x

{0x00, 1 , {0x80}},// shift address
{0xcb, 10 , {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	
// cb9x
{0x00, 1 , {0x90}},// shift address
{0xCb, 15, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

// cbax
{0x00, 1 , {0xa0}},// shift address
{0xCb, 15, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

// cbbx
{0x00, 1 , {0xb0}},// shift address
{0xCb, 10, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

// cbcx
{0x00, 1 , {0xc0}},// shift address
{0xCb, 15, {0x00,0x04,0x04,0x04,0x04,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	
// cbdx  
{0x00, 1 , {0xd0}},// shift address
{0xCb, 15, {0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x04,0x04,0x04,0x04,0x00,0x00,0x00,0x00}},
	
// cbex
{0x00, 1 , {0xe0}},// shift address
{0xCb, 10, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	
// cbfx
{0x00, 1 , {0xf0}},// shift address
{0xCb, 10, {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}},

// cc8x
{0x00, 1 , {0x80}},// shift address
{0xCC, 10, {0x00,0x26,0x09,0x0b,0x01,0x25,0x00,0x00,0x00,0x00}},
	
// cc9x
{0x00, 1 , {0x90}},// shift address
{0xCc, 15, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x26,0x0a,0x0c,0x02}},
	
// ccax
{0x00, 1 , {0xa0}},// shift address
{0xCc, 15, {0x25,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
		
// ccbx
{0x00, 1 , {0xb0}},// shift address
{0xCc, 10, {0x00,0x25,0x10,0x0e,0x02,0x26,0x00,0x00,0x00,0x00}},	
	
// cccx
{0x00, 1 , {0xc0}},// shift address
{0xCc, 15, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x25,0x0f,0x0d,0x01}},	

// ccdx
{0x00, 1 , {0xd0}},// shift address
{0xCc, 15, {0x26,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
	
{0x35, 1 , {0x00}},

{0x00, 1 , {0x00}},
{0x44, 1 , {0x00,0x50}},

{0x11, 1 , {0x00}},
{REGFLAG_DELAY, 120, {}},
{0x29, 1,{0x00}},
{REGFLAG_DELAY, 10, {}},
{REGFLAG_END_OF_TABLE, 0x00, {}}

};


static struct LCM_setting_table lcm_set_window[] = {
	{0x2A,	4,	{0x00, 0x00, (FRAME_WIDTH>>8), (FRAME_WIDTH&0xFF)}},
	{0x2B,	4,	{0x00, 0x00, (FRAME_HEIGHT>>8), (FRAME_HEIGHT&0xFF)}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
	// Sleep Out
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	// Display ON
	{0x29, 1, {0x00}},
        {REGFLAG_DELAY, 120, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
        {REGFLAG_DELAY, 120, {}},

	// Sleep Mode On
	{0x10, 1, {0x00}},
        {REGFLAG_DELAY, 120, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for(i = 0; i < count; i++) {

        unsigned int cmd;
        cmd = table[i].cmd;

        switch (cmd) {

            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;

            case REGFLAG_END_OF_TABLE :
                break;

            default:
		dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }

}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	// enable tearing-free
	params->dbi.te_mode 			= LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = SYNC_EVENT_VDO_MODE;
		//params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_TWO_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	// Not support in MT6573
	params->dsi.packet_size=256;

	// Video mode setting
	params->dsi.intermediat_buffer_num = 2;

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active				= 8;//4
	params->dsi.vertical_backporch					= 16;//16
	params->dsi.vertical_frontporch					= 16;//15
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 50;//4
	params->dsi.horizontal_backporch				= 90;//50
	params->dsi.horizontal_frontporch				= 90;//50
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	// Bit rate calculation
#if 0//def CONFIG_MT6589_FPGA
        params->dsi.pll_div1=1;             // div1=0,1,2,3;div1_real=1,2,4,4
        params->dsi.pll_div2=0;             // div2=0,1,2,3;div2_real=1,2,4,4
        params->dsi.fbk_div =15;
#else
    params->dsi.PLL_CLOCK = 215;//215
#endif
}


static void lcm_init(void)
{
	SET_RESET_PIN(1);
	MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{

   lcm_init();
	//push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	
}

static void lcm_resume(void)
{
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	data_array[3]= 0x00053902;
	data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[5]= (y1_LSB);
	data_array[6]= 0x002c3909;

	dsi_set_cmdq(&data_array, 7, 0);

}


static void lcm_setbacklight(unsigned int level)
{
	unsigned int default_level = 145;
	unsigned int mapped_level = 0;

	//for LGE backlight IC mapping table
	if(level > 255)
			level = 255;

	if(level >0)
			mapped_level = default_level+(level)*(255-default_level)/(255);
	else
			mapped_level=0;

	// Refresh value of backlight level.
	lcm_backlight_level_setting[0].para_list[0] = mapped_level;

	push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);
}

static unsigned int lcm_compare_id()
{       

	unsigned int id = 0;
	unsigned int id1 = 0,id2 = 0,id3 = 0,id4 = 0,id5 = 0,id6 = 0;
	unsigned char buffer[6],buffer1[2];
	unsigned int array[16];

	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(25);
	SET_RESET_PIN(1);
	MDELAY(50);

	array[0]=0x00023902;
	array[1]=0x00000000;
	dsi_set_cmdq(array, 2, 1);
	array[0]=0x00043902;
	array[1]=0x010980ff;
	dsi_set_cmdq(array, 2, 1);
	array[0]=0x00023902;
	array[1]=0x00008000;
	dsi_set_cmdq(array, 2, 1);
	array[0]=0x00033902;
	array[1]=0x000980ff;
	dsi_set_cmdq(array, 2, 1);

	array[0] = 0x00053700;// set return byte number
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xA1, &buffer, 5);
	array[0] = 0x00013700;// set return byte number
	dsi_set_cmdq(array, 1, 1);
	array[0]=0x00023902;
	array[1]=0x00005000;
	dsi_set_cmdq(array, 2, 1);
	MDELAY(1);
	read_reg_v2(0xF8, &buffer1, 1);

	id = (buffer[2] << 16) | (buffer[3] << 8) | buffer1[0];
#if defined(BUILD_LK)
	printf("%s, buffer[0]=0x%x,buffer[1]=0x%x,buffer[2]=0x%x,buffer[3]=0x%x buffer[4]=0x%x buffer1[0]=0x%x id = 0x%x\n",
	__func__,buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer1[0],id);
#else
  printk("%s, buffer[0]=0x%x,buffer[1]=0x%x,buffer[2]=0x%x,buffer[3]=0x%x buffer[4]=0x%x buffer1[0]=0x%x id = 0x%x\n",
	__func__,buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer1[0],id);
#endif
	return (LCM_ID == id)?1:0;
}

LCM_DRIVER otm8018a_boe50_cxd_fwvga_lcm_drv =
{
	.name		= "otm8018a_boe50_cxd_fwvga",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
    .init_power        = lcm_init_power,
    .resume_power = lcm_resume_power,
    .suspend_power = lcm_suspend_power,
#if (LCM_DSI_CMD_MODE)
	.set_backlight	= lcm_setbacklight,
	.update         = lcm_update,
#endif
	.compare_id     = lcm_compare_id,
};
