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

#define FRAME_WIDTH  										(480)
#define FRAME_HEIGHT 										(854)
#define LCM_NT35512_ID   (0x5512)

#define REGFLAG_DELAY             							0XFFE
#define REGFLAG_END_OF_TABLE      							0xFFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0


// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        (lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update))
#define dsi_set_cmdq(pdata, queue_size, force_update)		(lcm_util.dsi_set_cmdq(pdata, queue_size, force_update))
#define wrtie_cmd(cmd)										(lcm_util.dsi_write_cmd(cmd))
#define write_regs(addr, pdata, byte_nums)					(lcm_util.dsi_write_regs(addr, pdata, byte_nums))
#define read_reg(cmd)											(lcm_util.dsi_dcs_read_lcm_reg(cmd))
#define read_reg_v2(cmd, buffer, buffer_size)   				(lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size))


static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};
static void lcm_init_power(void)
{
	#ifdef GPIO_CTP_1V8_EN_PIN 
    mt_set_gpio_mode(GPIO_CTP_1V8_EN_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_CTP_1V8_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_1V8_EN_PIN, GPIO_OUT_ONE);
	#endif

}


static void lcm_suspend_power(void)
{
	#ifdef GPIO_CTP_1V8_EN_PIN 
    mt_set_gpio_mode(GPIO_CTP_1V8_EN_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_CTP_1V8_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_1V8_EN_PIN, GPIO_OUT_ZERO);
	#endif
}

static void lcm_resume_power(void)
{
	#ifdef GPIO_CTP_1V8_EN_PIN 
    mt_set_gpio_mode(GPIO_CTP_1V8_EN_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_CTP_1V8_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_1V8_EN_PIN, GPIO_OUT_ONE);
	#endif
}

static struct LCM_setting_table lcm_initialization_setting[] = {
{0xF0,5,{0x55,0xAA,0x52,0x08,0x01}},
{0xB0,1,{0x0D}},
{0xB6,1,{0x36}},
{0xB1,1,{0x0D}},
{0xB7,1,{0x24}},
{0xB2,1,{0x00}},
{0xB8,1,{0x36}},
{0xBF,1,{0x01}},
{0xB3,1,{0x07}},
{0xB9,1,{0x34}},
{0xB5,1,{0x07}},
{0xBA,1,{0x14}},
{0xBC,3,{0x00,0x78,0x00}},
{0xBD,3,{0x00,0x78,0x00}},
{0xBE,2,{0x00,0x56}},
{0xD1,52,{0x00,0x78,0x00,0x7B,0x00,0x87,0x00,0x92,0x00,0x9C,0x00,0xAE,0x00,0xC0,0x00,0xE0,0x00,0xFC,0x01,0x2F,0x01,0x5C,0x01,0xA7,0x01,0xE8,0x01,0xEB,0x02,0x2B,0x02,0x71,0x02,0x9C,0x02,0xDA,0x02,0xFC,0x03,0x2C,0x03,0x4D,0x03,0x6E,0x03,0x91,0x03,0xB6,0x03,0xBF,0x03,0xF9}},
{0xD2,52,{0x00,0xA0,0x00,0xA3,0x00,0xAD,0x00,0xB6,0x00,0xBE,0x00,0xCF,0x00,0xDE,0x00,0xFA,0x01,0x14,0x01,0x44,0x01,0x6D,0x01,0xB4,0x01,0xF4,0x01,0xF5,0x02,0x33,0x02,0x76,0x02,0xA1,0x02,0xDD,0x02,0xFE,0x03,0x2E,0x03,0x4D,0x03,0x6F,0x03,0x87,0x03,0xAA,0x03,0xC0,0x03,0xF9}},
{0xD3,52,{0x00,0x00,0x00,0x09,0x00,0x1D,0x00,0x2F,0x00,0x40,0x00,0x5C,0x00,0x75,0x00,0xA2,0x00,0xC9,0x01,0x08,0x01,0x3E,0x01,0x95,0x01,0xDD,0x01,0xE3,0x02,0x23,0x02,0x6C,0x02,0x98,0x02,0xD4,0x02,0xF5,0x03,0x2A,0x03,0x45,0x03,0x65,0x03,0x96,0x03,0xAD,0x03,0xF9,0x03,0xF9}},
{0xD4,52,{0x00,0x78,0x00,0x7B,0x00,0x87,0x00,0x92,0x00,0x9C,0x00,0xAE,0x00,0xC0,0x00,0xE0,0x00,0xFC,0x01,0x2F,0x01,0x5C,0x01,0xA7,0x01,0xE8,0x01,0xEB,0x02,0x2B,0x02,0x71,0x02,0x9C,0x02,0xDA,0x02,0xFC,0x03,0x2C,0x03,0x4D,0x03,0x6E,0x03,0x91,0x03,0xB6,0x03,0xBF,0x03,0xF9}},
{0xD5,52,{0x00,0xA0,0x00,0xA3,0x00,0xAD,0x00,0xB6,0x00,0xBE,0x00,0xCF,0x00,0xDE,0x00,0xFA,0x01,0x14,0x01,0x44,0x01,0x6D,0x01,0xB4,0x01,0xF4,0x01,0xF5,0x02,0x33,0x02,0x76,0x02,0xA1,0x02,0xDD,0x02,0xFE,0x03,0x2E,0x03,0x4D,0x03,0x6F,0x03,0x87,0x03,0xAA,0x03,0xC0,0x03,0xF9}},
{0xD6,52,{0x00,0x00,0x00,0x09,0x00,0x1D,0x00,0x2F,0x00,0x40,0x00,0x5C,0x00,0x75,0x00,0xA2,0x00,0xC9,0x01,0x08,0x01,0x3E,0x01,0x95,0x01,0xDD,0x01,0xE3,0x02,0x23,0x02,0x6C,0x02,0x98,0x02,0xD4,0x02,0xF5,0x03,0x2A,0x03,0x45,0x03,0x65,0x03,0x96,0x03,0xAD,0x03,0xF9,0x03,0xF9}},
{0xF0,5,{0x55,0xAA,0x52,0x08,0x00}},
{0xB5,1,{0x6B}},
{0xB6,1,{0x05}},
{0xB7,2,{0x77,0x77}},
{0xB8,4,{0x01,0x03,0x03,0x03}},
{0xBC,1,{0x01}},
{0xDE,1,{0x1E}},
{0xB1,3,{0x6A,0x00,0x01}},
{0xCE,11,{0x00,0xFF,0xFF,0xFF,0xFF,0xB0,0xC1,0xD2,0xE3,0x0A,0x80}},

{0x11,1,{0x00}},//SLEEP OUT
{REGFLAG_DELAY,120,{}},
                                 				                                                                                
{0x29,1,{0x00}},//Display ON 
{REGFLAG_DELAY,20,{}},	
{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	//{0x13, 0, {0x00}},
	{0x11, 0, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 10, {}},
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_mode_in_setting[] = {
	// Display off sequence
	//{0x22, 0, {0x00}},
	{REGFLAG_DELAY, 50, {}},
	{0x28, 0, {0x00}},
	{REGFLAG_DELAY, 20, {}},

    // Sleep Mode On
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 200, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_compare_id_setting[] = {
	// Display off sequence
	{0xF0,	5,	{0x55, 0xaa, 0x52,0x08,0x00}},
	{REGFLAG_DELAY, 10, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
				//MDELAY(2);
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

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	// enable tearing-free
	params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

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
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
	params->dsi.packet_size=256;
	params->dsi.intermediat_buffer_num = 2;

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch					= 16;
	params->dsi.vertical_frontporch					= 15;
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 4;
	params->dsi.horizontal_backporch				= 50;
	params->dsi.horizontal_frontporch				= 50;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	// Bit rate calculation
	//1 Every lane speed
//	params->dsi.pll_div1 = 0;	// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
//	params->dsi.pll_div2 = 1;	// div2=0,1,2,3;div1_real=1,2,4,4
//	params->dsi.fbk_div = 14;	//12;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
    params->dsi.PLL_CLOCK = 206;

    params->dsi.esd_check_enable = 1;
}

static void lcm_init(void)
{
	SET_RESET_PIN(1);
	MDELAY(10); 
	SET_RESET_PIN(0);
	MDELAY(10); 	
	SET_RESET_PIN(1);
	MDELAY(120);      

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
    /*SET_RESET_PIN(1);
	MDELAY(10); 
	SET_RESET_PIN(0);
	MDELAY(5); 	
	SET_RESET_PIN(1);
	MDELAY(120);  */
	push_table(lcm_sleep_mode_in_setting, sizeof(lcm_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_resume(void)
{
    lcm_init();
//	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}
         
static unsigned int lcm_compare_id(void)
{
  unsigned int id = 0;
  unsigned char buffer[5];
  unsigned int array[16];

  SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
  MDELAY(10);
  SET_RESET_PIN(0);
  MDELAY(50);
  SET_RESET_PIN(1);
  MDELAY(50);

  push_table(lcm_compare_id_setting, sizeof(lcm_compare_id_setting) / sizeof(struct LCM_setting_table), 1);

  array[0] = 0x00033700;
  dsi_set_cmdq(array,1,1);
  MDELAY(5);
  read_reg_v2(0xC5, buffer, 3);
  id = ((buffer[0] << 8) | buffer[1]); //we only need ID

#if (defined BUILD_LK)
  printf("[nt35512] %s, id = 0x%x, buffer = {0x%x, 0x%x, 0x%x}\n",
       __func__, id, buffer[0],buffer[1],buffer[2]);
#else
  printk("[nt35512] %s, id = 0x%x, buffer = {0x%x, 0x%x, 0x%x}\n",
       __func__, id, buffer[0],buffer[1],buffer[2]);
#endif

  return (LCM_NT35512_ID == id)?1:0;

}


LCM_DRIVER nt35512_fwvga_ivo50_longteng_lcm_drv =
{
	.name		= "nt35512_fwvga_ivo50_longteng",
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
