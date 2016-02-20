/* Copyright Statement:
*
* This software/firmware and related documentation ("MediaTek Software") 
are
* protected under relevant copyright laws. The information contained herein
* is confidential and proprietary to MediaTek Inc. and/or its licensors.
* Without the prior written permission of MediaTek inc. and/or its 
licensors,
* any reproduction, modification, use or disclosure of MediaTek Software,
* and information contained herein, in whole or in part, shall be 
strictly prohibited.
*/
/* MediaTek Inc. (C) 2010. All rights reserved.
*
* BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND 
AGREES
* THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
* RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO 
RECEIVER ON
* AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL 
WARRANTIES,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED 
WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
* NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
* SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
* SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY 
TO SUCH
* THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER 
EXPRESSLY ACKNOWLEDGES
* THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD 
PARTY ALL PROPER LICENSES
* CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE 
FOR ANY MEDIATEK
* SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A 
PARTICULAR
* STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND 
MEDIATEK'S ENTIRE AND
* CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED 
HEREUNDER WILL BE,
* AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT 
ISSUE,
* OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
* MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
* The following software/firmware and/or related documentation 
("MediaTek Software")
* have been modified by MediaTek Inc. All revisions are subject to any 
receiver's
* applicable license agreements with MediaTek Inc.
*/

/***************************************************************************** 

* Copyright Statement:
* --------------------
* This software is protected by Copyright and the information contained
* herein is confidential. The software may not be copied and the 
information
* contained herein may not be used or disclosed except with the written
* permission of MediaTek Inc. (C) 2008
*
* BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
* THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
* RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO 
BUYER ON
* AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL 
WARRANTIES,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED 
WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
* NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
* SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
* SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO 
SUCH
* THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
* NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
* SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
* BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
* LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER 
WILL BE,
* AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT 
ISSUE,
* OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
* MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
* THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
* WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
* LAWS PRINCIPLES. ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF 
AND
* RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, 
UNDER
* THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/ 



#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#else
#include <linux/string.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/upmu_common.h>
#endif


#include "lcm_drv.h"

#if defined(BUILD_LK)
#else

#include <linux/proc_fs.h> //proc file use
#endif


// 
// Local Constants
// 

#define FRAME_WIDTH (480)
#define FRAME_HEIGHT (854)

#define LCM_ID   (0x801906)
static unsigned int lcm_compare_id(void);

#define REGFLAG_DELAY (0XFE)
#define REGFLAG_END_OF_TABLE (0x100) // END OF REGISTERS MARKER


#define LCM_DSI_CMD_MODE 0

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

// 
// Local Variables
// 

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v) (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

static unsigned int lcm_esd_test = FALSE; ///only for ESD test

// 
// Local Functions
// 

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

struct LCM_setting_table {
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


static struct LCM_setting_table lcm_initialization_setting[] = {
    {0x00,1,{0x00}},
    {0xFF,3,{0x80,0x19,0x01}},
    
    {0x00,1,{0x80}},
    {0xFF,2,{0x80,0x19}},
    
    {0x00,1,{0x80}},
    {0xce,6,{0x83,0x01,0x1a,0x82,0x01,0x1a}},
    
    {0x00,1,{0x90}},
    {0xce,6,{0x13,0x54,0x1a,0x13,0x55,0x1a}},
    
    {0x00,1,{0xa0}},
    {0xce,14,{0x18,0x05,0x03,0x55,0x00,0x1a,0x00,0x18,0x04,0x03,0x56,0x00,0x1a,0x00}},
    
    {0x00,1,{0xb0}},
    {0xce,14,{0x18,0x07,0x03,0x57,0x00,0x1a,0x00,0x18,0x06,0x03,0x58,0x00,0x1a,0x00}},
    
    {0x00,1,{0xc0}},
    {0xce,14,{0x18,0x07,0x03,0x57,0x00,0x12,0x1a,0x18,0x06,0x03,0x58,0x00,0x12,0x1a}},
    
    {0x00,1,{0xd0}},
    {0xce,14,{0x18,0x05,0x03,0x58,0x00,0x12,0x1a,0x18,0x04,0x03,0x59,0x00,0x12,0x1a}},
    
    {0x00,1,{0xc0}},
    {0xcf,10,{0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x00,0x06}},
    
    {0x00,1,{0x80}},
    {0xcb,9,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    
    {0x00,1,{0x90}},
    {0xcb,15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    
    {0x00,1,{0xa0}},
    {0xcb,1,{0x00}},
    
    {0x00,1,{0xa5}},
    {0xcb,10,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    
    {0x00,1,{0xb0}},
    {0xcb,6,{0x00,0x00,0x00,0x00,0x00,0x00}},
    
    {0x00,1,{0xc0}},
    {0xcb,15,{0x15,0x00,0x15,0x15,0x15,0x15,0x15,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    
    {0x00,1,{0xd0}},
    {0xcb,1,{0x00}},
    
    {0x00,1,{0xd5}},
    {0xcb,10,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x15}},
    
    {0x00,1,{0xe0}},
    {0xcb,6,{0x15,0x15,0x15,0x15,0x00,0x15}},
    
    {0x00,1,{0xf0}},
    {0xcb,8,{0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}},
    
    {0x00,1,{0x80}},
    {0xcc,10,{0x01,0x00,0x0b,0x09,0x0f,0x0d,0x05,0x00,0x00,0x00}},
    
    {0x00,1,{0x90}},
    {0xcc,6,{0x00,0x00,0x00,0x00,0x00,0x00}},
    
    {0x00,1,{0x9a}},
    {0xcc,5,{0x00,0x00,0x00,0x00,0x00}},
    
    {0x00,1,{0xa0}},
    {0xcc,11,{0x00,0x00,0x00,0x00,0x06,0x0e,0x10,0x0a,0x0c,0x00,0x02}},
    
    {0x00,1,{0xb0}},
    {0xcc,10,{0x06,0x00,0x0c,0x0a,0x0e,0x10,0x02,0x00,0x00,0x00}},
    
    {0x00,1,{0xc0}},
    {0xcc,6,{0x00,0x00,0x00,0x00,0x00,0x00}},
    
    {0x00,1,{0xca}},
    {0xcc,5,{0x00,0x00,0x00,0x00,0x00}},
    
    {0x00,1,{0xd0}},
    {0xcc,11,{0x00,0x00,0x00,0x00,0x01,0x0f,0x0d,0x09,0x0b,0x00,0x05}},
    
    {0x00,1,{0x90}},                        // OTM8019A VGH=15V VGL=-12V
        {0xC5,2,{0x6E,0x79}}, //4E
    
    {0x00,1,{0x00}},                        // GVDD/NGVDD +/-5.0V
        {0xD8,2,{0x7f,0x7f}},   //97    
    
    {0x00,1,{0x00}},                        // VCOM        -1V
        {0xD9,1,{0x3b}}, //39 
    
    {0x00,1,{0x80}},
    {0xC4,1,{0x30}},
    
    {0x00,1,{0xA0}},
    {0xC1,1,{0xe8}},
    
    {0x00,1,{0x80}},
    {0xc1,2,{0x00,0x04}},//03
    
    {0x00,1,{0x90}},
    {0xC0,6,{0x00,0x15,0x00,0x00,0x00,0x03}},
    
    
    {0x00,1,{0x98}},
    {0xc0,1,{0x00}},
    
    {0x00,1,{0xA9}},
    {0xc0,1,{0x0A}},
    
    {0x00,1,{0xB0}},
    {0xc1,3,{0x20,0x00,0x00}},
    
    {0x00,1,{0xE1}},
    {0xc0,2,{0x40,0x30}},
    
    {0x00,1,{0x90}},
    {0xB6,2,{0xB4,0x5A}},
    
    {0x00,1,{0x90}},
    {0xB3,1,{0x02}},
    
    {0x00,1,{0x92}},
    {0xB3,1,{0x40}},
    
    
    {0x00,1,{0xB4}},
    {0xc0,1,{0x70}}, //Column Inversion
    
        {0x00,1,{0xA2}},   
        {0xc0,3,{0x00,0x09,0x04}},//add
    {0x00,1,{0x00}},
    {0xE1,20,{0x00,0x0f,0x1a,0x28,0x36,0x45,0x47,0x74,0x68,0x83,0x7e,0x66,0x76,0x4f,0x4b,0x3b,0x2a,0x1c,0x12,0x00}},
    
    {0x00,1,{0x00}},
    {0xE2,20,{0x00,0x0f,0x1a,0x28,0x36,0x45,0x47,0x74,0x68,0x83,0x7e,0x66,0x76,0x4f,0x4b,0x3b,0x2a,0x1c,0x12,0x00}},
    
    {0x00,1,{0x80}},
    {0xC4,1,{0x30}},
    
    {0x00,1,{0x98}},
    {0xC0,1,{0x00}},
    
    {0x00,1,{0xa9}},
    {0xC0,1,{0x0A}},    //0x06
    
    {0x00,1,{0xb0}},
    {0xC1,3,{0x20,0x00,0x00}}, //
    
    {0x00,1,{0xe1}},
    {0xC0,2,{0x40,0x30}}, //0x40,0x18
    
    
    {0x00,1,{0x80}},
    {0xC1,2,{0x03,0x33}},
    
    {0x00,1,{0xA0}},
    {0xC1,1,{0xe8}},
    
    {0x00,1,{0x90}},
    {0xb6,1,{0xb4}},    //command fial
    
       {REGFLAG_DELAY,10,{}},
    {0x00,1,{0x00}},
    {0xfb,1,{0x01}},
    
       {REGFLAG_DELAY,10,{}},
    {0x00,1,{0x00}},
    {0xff,3,{0xff,0xff,0xff}},
    
  {0x13,1,{0x00}},
   {REGFLAG_DELAY, 10, {}},
    
    {0x11,1,{0x00}},
     {REGFLAG_DELAY,120,{}},
       {0x29,1,{0x00}},//Display ON 
       {REGFLAG_DELAY,30,{}},
       {REGFLAG_END_OF_TABLE, 0x00, {}}  
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
// Sleep Out
{0x11, 1, {0x00}},
{REGFLAG_DELAY, 150, {}},
// Display ON
//{0x2C, 1, {0x00}},
//{0x13, 1, {0x00}},
{0x29, 1, {0x00}},
{REGFLAG_DELAY, 200, {}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
// Display off sequence
   {0x22, 1, {0x00}},
   {REGFLAG_DELAY, 50, {}},
{0x28, 1, {0x00}},
{REGFLAG_DELAY, 20, {}},

// Sleep Mode On
{0x10, 1, {0x00}},
{REGFLAG_DELAY, 150, {}},

{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_compare_id_setting[] = {
// Display off sequence
{0xf0, 5, {0x55, 0xaa, 0x52, 0x08, 0x01}},
{REGFLAG_DELAY, 10, {}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_backlight_level_setting[] = {
{0x51, 1, {0xFF}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int 
count, unsigned char force_update)
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
}
}

}


// 
// LCM Driver Implementations
// 

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
#if 1
    // enable tearing-free
    params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
    params->dbi.te_edge_polarity = LCM_POLARITY_RISING;
    
    params->dsi.mode = SYNC_EVENT_VDO_MODE;//BURST_VDO_MODE;
    /* Command mode setting */
    params->dsi.LANE_NUM = LCM_TWO_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size=256;
    // Video mode setting
    params->dsi.intermediat_buffer_num = 2;
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    params->dsi.vertical_sync_active = 4;
    params->dsi.vertical_backporch = 16; //16
    params->dsi.vertical_frontporch = 15; //15
    params->dsi.vertical_active_line = FRAME_HEIGHT;
    params->dsi.horizontal_sync_active = 10;
    params->dsi.horizontal_backporch = 64; //64
    params->dsi.horizontal_frontporch = 64;
    params->dsi.horizontal_blanking_pixel = 60;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;
#else
        params->dbi.te_mode                 =LCM_DBI_TE_MODE_VSYNC_ONLY;//LCM_DBI_TE_MODE_DISABLED;// LCM_DBI_TE_MODE_VSYNC_ONLY;
        params->dbi.te_edge_polarity        = LCM_POLARITY_RISING;

        #if (LCM_DSI_CMD_MODE)
        params->dsi.mode   = CMD_MODE;
        #else
        params->dsi.mode   = BURST_VDO_MODE;
        #endif
        params->dsi.packet_size=256;

        //1 Three lane or Four lane
        params->dsi.LANE_NUM                = LCM_FOUR_LANE;
        //The following defined the fomat for data coming from LCD engine.
        params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
        params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
        params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
        params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
        params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
//      params->dsi.word_count=720*3;
        params->dsi.vertical_sync_active                = 10;
        params->dsi.vertical_backporch                  = 10;
        params->dsi.vertical_frontporch                 = 10;
        params->dsi.vertical_active_line                = FRAME_HEIGHT;

        params->dsi.horizontal_sync_active              = 10;
        params->dsi.horizontal_backporch                = 60;
        params->dsi.horizontal_frontporch               = 90;
        params->dsi.horizontal_active_pixel             = FRAME_WIDTH;
#endif
params->dsi.PLL_CLOCK = 185; //dpi clock customization: should config 
//params->dsi.noncont_clock = 1;
//params->dsi.noncont_clock_period = 1;
}

static void lcm_init(void)
{

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);

    push_table(lcm_initialization_setting, 
            sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
#ifdef BUILD_LK
    printf("[erick-lk]%s\n", __func__);
#else
    printk("[erick-k]%s\n", __func__);
#endif
}


static void lcm_suspend(void)
{
#ifndef BUILD_LK
SET_RESET_PIN(1);
MDELAY(10);
SET_RESET_PIN(0);
MDELAY(10);
SET_RESET_PIN(1);
MDELAY(120);
push_table(lcm_deep_sleep_mode_in_setting, 
sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct 
LCM_setting_table), 1); //wqtao. enable
SET_RESET_PIN(0);
#ifdef BUILD_LK
printf("[erick-lk]%s\n", __func__);
#else
printk("[erick-k]%s\n", __func__);
#endif
#endif
}


static void lcm_resume(void)
{

/* Vanzo:luanshijun on: Mon, 27 Apr 2015 19:30:54 +0800
 * #107769 ,modify wake screen erro
 * lcm_compare_id();
 */
// End of Vanzo:luanshijun
#ifndef BUILD_LK
lcm_init();

//push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);

#ifdef BUILD_LK
printf("[erick-lk]%s\n", __func__);
#else
printk("[erick-k]%s\n", __func__);
#endif
#endif
}

#if (LCM_DSI_CMD_MODE)
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
dsi_set_cmdq(data_array, 3, 1);

data_array[0]= 0x00053902;
data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
data_array[2]= (y1_LSB);
dsi_set_cmdq(data_array, 3, 1);

data_array[0]= 0x00290508; //HW bug, so need send one HS packet
dsi_set_cmdq(data_array, 1, 1);

data_array[0]= 0x002c3909;
dsi_set_cmdq(data_array, 1, 0);
}
#endif

static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
	static int ok_count = 0;
	 unsigned char buffer[12];
	 unsigned int array[16];
	 int i;
	 
	if(lcm_esd_test)
  {
      lcm_esd_test = 0;
      return 1;
  }
	 	
	array[0] = 0x00013700;
	dsi_set_cmdq(array, 1, 1);
	//MDELAY(10);
	read_reg_v2(0x0A, buffer, 1);
	 
	printk(" ### esd check buffer[0] = 0x%x\r\n",buffer[0]);
	if(buffer[0] == 0x9C)
		return 0;
	else
		return 1;
 			
#endif
}

static unsigned int lcm_esd_recover(void)
{
  lcm_init();
	
#ifndef BUILD_LK	
	printk("### lcm_esd_recover \r\n");
#endif
	
	return 1;
}

static unsigned int lcm_compare_id(void)
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

// 
// Get LCM Driver Hooks
// 
LCM_DRIVER otm8019a_auo45_ykl_fwvga_lcm_drv =
{
    .name = "otm8019a_auo45_ykl_fwvga",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params = lcm_get_params,
    .init = lcm_init,
    .suspend = lcm_suspend,
    .resume = lcm_resume,
    .compare_id = lcm_compare_id,
    .init_power        = lcm_init_power,
    .resume_power = lcm_resume_power,
    .suspend_power = lcm_suspend_power,
    .esd_check = lcm_esd_check,
    .esd_recover = lcm_esd_recover,
};
