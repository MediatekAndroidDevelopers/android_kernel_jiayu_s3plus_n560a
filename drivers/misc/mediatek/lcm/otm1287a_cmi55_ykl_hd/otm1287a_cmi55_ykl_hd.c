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

#if defined(BUILD_LK)
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#else
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/upmu_common.h>
#endif

#if !defined(BUILD_LK)
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#if defined(BUILD_LK)
	#define LCM_DEBUG  printf
	#define LCM_FUNC_TRACE() printf("huyl [uboot] %s\n",__func__)
#else
	#define LCM_DEBUG  printk
	#define LCM_FUNC_TRACE() printk("huyl [kernel] %s\n",__func__)
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  			(720)
#define FRAME_HEIGHT 			(1280)
#define LCM_OTM1287_ID			(0x1287)

#define REGFLAG_DELAY          		(0XFEE)
#define REGFLAG_END_OF_TABLE      	(0xFFFF)	// END OF REGISTERS MARKER

#ifndef FALSE
#define FALSE 0
#endif

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = { 0 };

#define SET_RESET_PIN(v)    		(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 					(lcm_util.udelay(n))
#define MDELAY(n) 					(lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   


struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static void lcm_init_power(void)
{
#ifdef GPIO_LCM_LED_EN
    mt_set_gpio_mode(GPIO_LCM_LED_EN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCM_LED_EN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCM_LED_EN, GPIO_OUT_ONE);
#endif
}


static void lcm_suspend_power(void)
{
#ifdef GPIO_LCM_LED_EN
    mt_set_gpio_mode(GPIO_LCM_LED_EN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCM_LED_EN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCM_LED_EN, GPIO_OUT_ZERO);
#endif
}

static void lcm_resume_power(void)
{
#ifdef GPIO_LCM_LED_EN
    mt_set_gpio_mode(GPIO_LCM_LED_EN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_LCM_LED_EN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_LCM_LED_EN, GPIO_OUT_ONE);
#endif
}
static struct LCM_setting_table lcm_initialization_setting[] = {

{0x00,1,{0x00}},
{0xff,3,{0x12,0x87,0x01}},	//EXTC=1
{0x00,1,{0x80}},	        //Orise mode enable
{0xff,2,{0x12,0x87}},
       
{0x00,1,{0x92}},				
{0xff,2,{0x30,0x02}},		//MIPI 4 Lane


{0x00,1,{0x80}},             //TCON Setting
{0xc0,9,{0x00,0x64,0x00,0x10,0x10,0x00,0x64,0x10,0x10}},
       
{0x00,1,{0x90}},             //Panel Timing Setting
{0xc0,6,{0x00,0x5c,0x00,0x01,0x00,0x04}},
       
{0x00,1,{0xb3}},             //Interval Scan Frame: 0 frame, column inversion
{0xc0,2,{0x00,0x55}},
       
{0x00,1,{0x81}},             //frame rate:60Hz
{0xc1,1,{0x55}},


{0x00,1,{0xa0}},             //dcdc setting
{0xc4,14,{0x05,0x10,0x04,0x02,0x05,0x15,0x11,0x05,0x10,0x07,0x02,0x05,0x15,0x11}},

{0x00,1,{0xb0}},             //clamp voltage setting
{0xc4,2,{0x00,0x00}},
       
{0x00,1,{0x91}},             //VGH=16V, VGL=-12V, pump ratio:VGH=8x, VGL=-5x
{0xc5,2,{0x69,0xd2}},
       
{0x00,1,{0x00}},             //GVDD=5.008V, NGVDD=-5.008V
{0xd8,2,{0xc7,0xc7}},     
       
{0x00,1,{0x00}},             //VCOM=-1.63V
{0xd9,1,{0x54}},             
       
{0x00,1,{0xb3}},             //VDD_18V=1.7V, LVDSVDD=1.6V
{0xc5,1,{0x84}},
       
{0x00,1,{0xbb}},             //LVD voltage level setting
{0xc5,1,{0x8a}},
       
{0x00,1,{0x82}},		//chopper
{0xC4,1,{0x0a}},
       
{0x00,1,{0xc6}},		//debounce
{0xb0,1,{0x03}},


{0x00,1,{0x00}},             //ID1
{0xd0,1,{0x40}},
       
{0x00,1,{0x00}},             //ID2, ID3
{0xd1,2,{0x00,0x00}},


{0x00,1,{0xb2}},             //VGLO1
{0xf5,2,{0x00,0x00}},
       
{0x00,1,{0xb6}},             //VGLO2
{0xf5,2,{0x00,0x00}},
       
{0x00,1,{0x94}},             //VCL pump dis
{0xf5,2,{0x00,0x00}},
       
{0x00,1,{0xd2}},             //VCL reg. en
{0xf5,2,{0x06,0x15}},

{0x00,1,{0xb4}},             //VGLO1/2 Pull low setting
{0xc5,1,{0xcc}},	       //d[7] vglo1 d[6] vglo2 => 0: pull vss, 1: pull vgl


{0x00,1,{0x90}},             //Mode-3
{0xf5,4,{0x02,0x11,0x02,0x15}},
       
{0x00,1,{0x90}},             //2xVPNL, 1.5*=00, 2*=50, 3*=a0
{0xc5,1,{0x50}},
       
{0x00,1,{0x94}},             //Frequency
{0xc5,1,{0x55}},


{0x00,1,{0x80}},             //panel timing state control
{0xcb,11,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0x90}},             //panel timing state control
{0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xa0}},             //panel timing state control
{0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xb0}},             //panel timing state control
{0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xc0}},             //panel timing state control
{0xcb,15,{0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x00,0x05,0x00,0x00,0x00,0x00}},

{0x00,1,{0xd0}},             //panel timing state control
{0xcb,15,{0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05}},

{0x00,1,{0xe0}},             //panel timing state control
{0xcb,14,{0x05,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x00,0x00}},

{0x00,1,{0xf0}},             //panel timing state control
{0xcb,11,{0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}},


{0x00,1,{0x80}},             //panel pad mapping control
{0xcc,15,{0x29,0x2a,0x0a,0x0c,0x0e,0x10,0x12,0x14,0x06,0x00,0x08,0x00,0x00,0x00,0x00}},

{0x00,1,{0x90}},             //panel pad mapping control
{0xcc,15,{0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x29,0x2a,0x09,0x0b,0x0d,0x0f,0x11,0x13}},

{0x00,1,{0xa0}},             //panel pad mapping control
{0xcc,14,{0x05,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00}},

{0x00,1,{0xb0}},             //panel pad mapping control
{0xcc,15,{0x29,0x2a,0x13,0x11,0x0f,0x0d,0x0b,0x09,0x01,0x00,0x07,0x00,0x00,0x00,0x00}},

{0x00,1,{0xc0}},             //panel pad mapping control
{0xcc,15,{0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x29,0x2a,0x14,0x12,0x10,0x0e,0x0c,0x0a}},

{0x00,1,{0xd0}},             //panel pad mapping control
{0xcc,14,{0x02,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00}},


{0x00,1,{0x80}},             //panel VST setting
{0xce,12,{0x89,0x05,0x00,0x88,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0x90}},             //panel VEND setting
{0xce,14,{0x54,0xfc,0x00,0x54,0xfd,0x00,0x55,0x00,0x00,0x55,0x01,0x00,0x00,0x00}},

{0x00,1,{0xa0}},             //panel CLKA1/2 setting
{0xce,14,{0x58,0x07,0x05,0x08,0x00,0x00,0x00,0x58,0x06,0x05,0x09,0x00,0x00,0x00}},

{0x00,1,{0xb0}},             //panel CLKA3/4 setting
{0xce,14,{0x58,0x05,0x05,0x0A,0x00,0x00,0x00,0x58,0x04,0x05,0x0B,0x00,0x00,0x00}},

{0x00,1,{0xc0}},             //panel CLKb1/2 setting
{0xce,14,{0x58,0x03,0x05,0x0C,0x00,0x00,0x00,0x58,0x02,0x05,0x0D,0x00,0x00,0x00}},

{0x00,1,{0xd0}},             //panel CLKb3/4 setting
{0xce,14,{0x58,0x01,0x05,0x0E,0x00,0x00,0x00,0x58,0x00,0x05,0x0F,0x00,0x00,0x00}},

{0x00,1,{0x80}},             //panel CLKc1/2 setting
{0xcf,14,{0x50,0x00,0x05,0x10,0x00,0x00,0x00,0x50,0x01,0x05,0x11,0x00,0x00,0x00}},

{0x00,1,{0x90}},             //panel CLKc3/4 setting
{0xcf,14,{0x50,0x02,0x05,0x12,0x00,0x00,0x00,0x50,0x03,0x05,0x13,0x00,0x00,0x00}},

{0x00,1,{0xa0}},             //panel CLKd1/2 setting
{0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xb0}},             //panel CLKd3/4 setting
{0xcf,14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xc0}},             //panel ECLK setting
{0xcf,11,{0x3D,0x3D,0x20,0x20,0x00,0x00,0x01,0x01,0x20,0x00,0x00}},

{0x00,1,{0xb5}},
{0xc5,6,{0x0b,0x95,0xff,0x0b,0x95,0xff}},

{0x00,1,{0x00}},////////////////////////////////////////////////////////////////
{0xE1,20,{0x07,0x44,0x52,0x60,0x70,0x7D,0x7E,0xA5,0x92,0xA9,0x5E,0x4B,0x61,0x40,0x3F,0x34,0x27,0x1B,0x0A,0x04}},

{0x00,1,{0x00}},
{0xE2,20,{0x07,0x44,0x52,0x60,0x6F,0x7E,0x7E,0xA5,0x92,0xA8,0x5E,0x4C,0x61,0x40,0x3F,0x34,0x27,0x1B,0x0A,0x04}},

{0x00,1,{0x00}},             //Orise mode disable				
{0xff,3,{0xff,0xff,0xff}},


{0x36,1,{0x00}},

{0x35,1,{0x00}},//TE

{0x11,1,{0x00}},
 {REGFLAG_DELAY,120,{}},
{0x29,1,{0x00}},//Display ON

{REGFLAG_DELAY,20,{}},

{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_out_setting[] = {
	// Sleep Out
	{0x11, 0, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	// Display ON
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 20, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_in_setting[] = {
	// Display off sequence
	{0x28, 0, {0x00}},
	{REGFLAG_DELAY, 30, {}},
	// Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 150, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count,
		       unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {

		unsigned cmd;
		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count,
					table[i].para_list, force_update);
		}

	}

}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS * util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS * params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	// enable tearing-free
	params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

	params->dsi.mode = SYNC_PULSE_VDO_MODE;	//BURST_VDO_MODE;

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	params->dsi.intermediat_buffer_num = 0;	//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.word_count = 720 * 3;

		params->dsi.vertical_sync_active				= 0x3;// 3    2
		params->dsi.vertical_backporch					= 0x0E;// 20   1
		params->dsi.vertical_frontporch					= 0x10; // 1  12
		params->dsi.vertical_active_line				= FRAME_HEIGHT;

		params->dsi.horizontal_sync_active				= 0x04;// 50  2
		params->dsi.horizontal_backporch				= 0x22 ;
		params->dsi.horizontal_frontporch				= 0x18 ;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

    params->dsi.PLL_CLOCK =200;

}

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);

	push_table(lcm_initialization_setting,
		   sizeof(lcm_initialization_setting) /
		   sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
    unsigned int data_array[16];

    data_array[0] = 0x00280500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(10);

	data_array[0] = 0x00100500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(150);
}

static void lcm_resume(void)
{

	lcm_init(); 
	//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) /	sizeof(struct LCM_setting_table), 1);

}

extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
static int adc_read_vol(void)
{
	int adc[1];
	int data[4] ={0,0,0,0};
	int sum = 0;
	int adc_vol=0;
	int num = 0;

	for(num=0;num<10;num++)
	{
		IMM_GetOneChannelValue(1, data, adc);
		sum+=(data[0]*100+data[1]);
	}
	adc_vol = sum/10;

#if defined(BUILD_LK)
	printf("sunxing adc_vol is %d\n",adc_vol);
#else
	printk("sunxing adc_vol is %d\n",adc_vol);
#endif

	return (adc_vol<30) ? 0 : 1;
}

static unsigned int lcm_compare_id(void)
{
	/* unsigned int id0, id1, id2, id3, id4; */
	unsigned char buffer[5];
	unsigned int array[5];

	int id = 0;
    SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(10);	

	// Set Maximum return byte = 1
	array[0] = 0x00053700;
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xA1, buffer, 5);
	/* id_adc = adc_read_vol(); */
	/* id0 = buffer[0]; */
	/* id1 = buffer[1]; */
	/* id2 = buffer[2]; */
	/* id3 = buffer[3]+id_adc; */
	/* id4 = buffer[4]; */

	//id = (buffer[2] << 8) | buffer[3] + adc_read_vol();
    id = (buffer[2] << 8) | buffer[3];

#if defined(BUILD_LK)
	printf("%s, ID = %x, {%x, %x, %x, %x, %x} \n", __func__,
		   id, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
#else
	printk("%s, ID = %x, {%x, %x, %x, %x, %x} \n", __func__,
		   id, buffer[0], buffer[1], buffer[2], buffer[3], buffer[4]);
#endif

	return (LCM_OTM1287_ID == id) ? 1 : 0;
}


LCM_DRIVER otm1287a_cmi55_ykl_hd_lcm_drv= {
	.name = "otm1287a_cmi55_ykl_hd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
    .init_power        = lcm_init_power,
    .resume_power = lcm_resume_power,
    .suspend_power = lcm_suspend_power,

};
