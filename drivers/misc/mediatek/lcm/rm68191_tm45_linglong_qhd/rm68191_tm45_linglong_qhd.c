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
     #include <platform/upmu_common.h>
     #include <platform/mt_gpio.h>
     #include <platform/mt_i2c.h>
     #include <platform/mt_pmic.h>
     #include <string.h>
#else
    #include <linux/string.h>
    #if defined(BUILD_UBOOT)
        #include <asm/arch/mt_gpio.h>
    #else
        #include <linux/xlog.h>
        #include <mach/mt_gpio.h>
        #include <mach/mt_pm_ldo.h>
        #include <mach/upmu_common.h>
    #endif
#endif
#include "lcm_drv.h"
#include <cust_gpio_usage.h>



// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (540)
#define FRAME_HEIGHT (960)

//for rm68180
#define REGFLAG_DELAY             						(0XFEF)//0xAB
#define REGFLAG_END_OF_TABLE      						(0XFFF)//0x00

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};
LCM_DSI_MODE_SWITCH_CMD lcm_switch_mode1_cmd;
#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define LCM_DSI_CMD_MODE        (0)

#define RM68191_LCM_ID       (0x8191)

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_compare_id_setting[] = {
	// Display off sequence
	{0xF0, 5,{0x55,0xAA,0x52,0x08,0x01}},
	{REGFLAG_DELAY, 10, {}},

    // Sleep Mode On
//	{0xC3, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_initialization_setting[] = {
{0xF0, 5,{0x55, 0xAA, 0x52, 0x08, 0x03}},

{0x90, 9,{0x03, 0x16, 0x05, 0x00, 0x00, 0x00, 0x32, 0x00, 0x32}},

{0x91, 9,{0x01, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00}},

{0x92, 11,{0x40, 0x07, 0x08, 0x09, 0x0A, 0x00, 0x28, 0x00, 0x28, 0x03, 0x04}},

{0x94, 8,{0x00, 0x08, 0x06, 0x03, 0xC9, 0x03, 0xCB, 0x0C}},

{0x95, 16,{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},

{0x99, 2,{0x00, 0x00}},

{0x9A, 11,{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},

{0x9B, 6,{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},

{0x9C, 2,{0x00, 0x00}},

{0x9D, 8,{0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00}},

{0x9E, 2,{0x00, 0x00}},

{0xA0, 10,{0x95, 0x14, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}},

{0xA1, 10,{0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x09, 0x1F, 0x1F, 0x1F}},

{0xA2, 10,{0x0B, 0x1F, 0x01, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}},

{0xA3, 10,{0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}},

{0xA4, 10,{0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x00, 0x1F, 0x0A}},

{0xA5, 10,{0x1F, 0x1F, 0x1F, 0x08, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}},

{0xA6, 10,{0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x14, 0x15}},

{0xA7, 10,{0x94, 0x15, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}},

{0xA8, 10,{0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x0A, 0x1F, 0x1F, 0x1F}},

{0xA9, 10,{0x08, 0x1F, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}},

{0xAA, 10,{0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}},

{0xAB, 10,{0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x01, 0x1F, 0x09}},

{0xAC, 10,{0x1F, 0x1F, 0x1F, 0x0B, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F}},

{0xAD, 10,{0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x15, 0x14}},

{0xF0, 5,{0x55, 0xAA, 0x52, 0x08, 0x00}},

{0xFC, 1,{0x00}},

{0xBC, 3,{0x00, 0x00, 0x00}},

{0xB8, 4,{0x01, 0x8F, 0xBF, 0x8F}},


{0xF0, 5,{0x55, 0xAA, 0x52, 0x08, 0x01}},

{0xD1, 16,{0x00, 0x00, 0x00, 0x12, 0x00, 0x31, 0x00, 0x4A, 0x00, 0x5F, 0x00, 0x83, 0x00, 0xA1, 0x00, 0xD2}},

{0xD2, 16,{0x00, 0xFA, 0x01, 0x39, 0x01, 0x6C, 0x01, 0xBC, 0x01, 0xFE, 0x01, 0xFF, 0x02, 0x38, 0x02, 0x77}},

{0xD3, 16,{0x02, 0x9D, 0x02, 0xCD, 0x02, 0xEC, 0x03, 0x13, 0x03, 0x29, 0x03, 0x41, 0x03, 0x49, 0x03, 0x4C}},

{0xD4, 4,{0x03, 0x50, 0x03, 0x55}},

{0xD5, 16,{0x00, 0x00, 0x00, 0x12, 0x00, 0x31, 0x00, 0x4A, 0x00, 0x5F, 0x00, 0x83, 0x00, 0xA1, 0x00, 0xD2}},

{0xD6, 16,{0x00, 0xFA, 0x01, 0x39, 0x01, 0x6C, 0x01, 0xBC, 0x01, 0xFE, 0x01, 0xFF, 0x02, 0x38, 0x02, 0x77}},

{0xD7, 16,{0x02, 0x9D, 0x02, 0xCD, 0x02, 0xEC, 0x03, 0x13, 0x03, 0x29, 0x03, 0x41, 0x03, 0x49, 0x03, 0x4C}},

{0xD8, 4,{0x03, 0x50, 0x03, 0x55}},

{0xD9, 16,{0x00, 0x00, 0x00, 0x12, 0x00, 0x31, 0x00, 0x4A, 0x00, 0x5F, 0x00, 0x83, 0x00, 0xA1, 0x00, 0xD2}},

{0xDD, 16,{0x00, 0xFA, 0x01, 0x39, 0x01, 0x6C, 0x01, 0xBC, 0x01, 0xFE, 0x01, 0xFF, 0x02, 0x38, 0x02, 0x77}},

{0xDE, 16,{0x02, 0x9D, 0x02, 0xCD, 0x02, 0xEC, 0x03, 0x13, 0x03, 0x29, 0x03, 0x41, 0x03, 0x49, 0x03, 0x4C}},

{0xDF, 4,{0x03, 0x50, 0x03, 0x55}},

{0xE0, 16,{0x00, 0x00, 0x00, 0x12, 0x00, 0x31, 0x00, 0x4A, 0x00, 0x5F, 0x00, 0x83, 0x00, 0xA1, 0x00, 0xD2}},

{0xE1, 16,{0x00, 0xFA, 0x01, 0x39, 0x01, 0x6C, 0x01, 0xBC, 0x01, 0xFE, 0x01, 0xFF, 0x02, 0x38, 0x02, 0x77}},

{0xE2, 16,{0x02, 0x9D, 0x02, 0xCD, 0x02, 0xEC, 0x03, 0x13, 0x03, 0x29, 0x03, 0x41, 0x03, 0x49, 0x03, 0x4C}},

{0xE3, 4,{0x03, 0x50, 0x03, 0x55}},

{0xE4, 16,{0x00, 0x00, 0x00, 0x12, 0x00, 0x31, 0x00, 0x4A, 0x00, 0x5F, 0x00, 0x83, 0x00, 0xA1, 0x00, 0xD2}},

{0xE5, 16,{0x00, 0xFA, 0x01, 0x39, 0x01, 0x6C, 0x01, 0xBC, 0x01, 0xFE, 0x01, 0xFF, 0x02, 0x38, 0x02, 0x77}},

{0xE6, 16,{0x02, 0x9D, 0x02, 0xCD, 0x02, 0xEC, 0x03, 0x13, 0x03, 0x29, 0x03, 0x41, 0x03, 0x49, 0x03, 0x4C}},

{0xE7, 4,{0x03, 0x50, 0x03, 0x55}},

{0xE8, 16,{0x00, 0x00, 0x00, 0x12, 0x00, 0x31, 0x00, 0x4A, 0x00, 0x5F, 0x00, 0x83, 0x00, 0xA1, 0x00, 0xD2}},

{0xE9, 16,{0x00, 0xFA, 0x01, 0x39, 0x01, 0x6C, 0x01, 0xBC, 0x01, 0xFE, 0x01, 0xFF, 0x02, 0x38, 0x02, 0x77}},

{0xEA, 16,{0x02, 0x9D, 0x02, 0xCD, 0x02, 0xEC, 0x03, 0x13, 0x03, 0x29, 0x03, 0x41, 0x03, 0x49, 0x03, 0x4C}},

{0xEB, 4,{0x03, 0x50, 0x03, 0x55}},


{0xB0, 3,{0x0A, 0x0A, 0x0A}},

{0xB1, 3,{0x0A, 0x0A, 0x0A}},

{0xB6, 3,{0x34, 0x34, 0x34}},

{0xB7, 3,{ 0x34, 0x34, 0x34}},

{0xB3, 3,{0x12, 0x12, 0x12}},

{0xB9, 3,{0x24, 0x24, 0x24}},

{0xB4, 3,{0x08, 0x08, 0x08}},

{0xBA, 3,{0x14, 0x14, 0x14}},

{0xBC, 3,{0x00, 0x70, 0x00}},

{0xBD, 3,{0x00, 0x70, 0x00}},

{0xBE, 1,{0x58}},//4A 50 5A nita

{0x35, 1,{0x00}},

{0x11, 0,{0x00}},

{REGFLAG_DELAY, 120, {}},

{0x29, 0,{0x00}},

{REGFLAG_DELAY, 50, {}},	

{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
	// Sleep Out
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 120, {}},

	// Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 10, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_sleep_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},

	// Sleep Mode On
	{0x10, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},

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
    params->dbi.te_mode 			=LCM_DBI_TE_MODE_DISABLED;// LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;
    
  params->dsi.mode = SYNC_EVENT_VDO_MODE;   //SYNC_PULSE_VDO_MODE;
    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_TWO_LANE;

    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;
    
	params->dsi.packet_size=256;
    params->dsi.intermediat_buffer_num = 0;
    
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 10;//16
	params->dsi.vertical_frontporch = 12;//16
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 8;
	params->dsi.horizontal_backporch = 32;//64;
	params->dsi.horizontal_frontporch = 32;//64;	//21;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

    params->dsi.PLL_CLOCK = 221;
}

static void lcm_init(void)
{
    //lcd_power_en(0);
    //lcd_power_en(1);

    SET_RESET_PIN(1);
	MDELAY(5);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(100);

#if defined(BUILD_LK)
    printf("lk lcm_init\n");
#else
    printk("kernel lcm_init\n");
#endif
  
  //push_table(lcm_sleep_out_setting,sizeof(lcm_sleep_out_setting) /sizeof(struct LCM_setting_table), 1);
  
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
#if defined(BUILD_LK)
    printf("lk lcm_suspend\n");
#else
    printk("kernel lcm_suspend\n");
#endif

    push_table(lcm_sleep_in_setting,sizeof(lcm_sleep_in_setting) /sizeof(struct LCM_setting_table), 1);

	SET_RESET_PIN(0);
	MDELAY(10);
}

static void lcm_resume(void)
{
#if defined(BUILD_LK)
    printf("lk lcm_resume\n");
#else
    printk("kernel lcm_resume\n");
#endif

	lcm_init();
}

static unsigned int lcm_compare_id(void)
{
	unsigned int id;
	unsigned char buffer[5];
	unsigned int array[5];


    //lcd_power_en(1);

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(100);
/*
	push_table(lcm_compare_id_setting,
			sizeof(lcm_compare_id_setting) /
			sizeof(struct LCM_setting_table), 1);
*/
	array[0] = 0x00063902;// read id return two byte,version and id
	array[1] = 0x52AA55F0;
	array[2] = 0x00000108;
	dsi_set_cmdq(array, 3, 1);
	MDELAY(10);
	
	array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xc5, buffer, 2);
	id = ((buffer[0] << 8) | buffer[1]);
#if defined(BUILD_LK)
printf("%s, [rm68191_ctc50_jhzt]  buffer[0] = [0x%x] buffer[2] = [0x%x] ID = [0x%x]\n",__func__, buffer[0], buffer[1], id);
#else
printk("%s, [rm68191_ctc50_jhzt]  buffer[0] = [0x%x] buffer[2] = [0x%x] ID = [0x%x]\n",__func__, buffer[0], buffer[1], id);
#endif

    //lcd_power_en(0);
    
    return (RM68191_LCM_ID == id)?1:0;
}

static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
    char  buffer[3];
    int   array[4];

    array[0] = 0x00013700;
    dsi_set_cmdq(array, 1, 1);

    read_reg_v2(0x53, buffer, 1);

    if(buffer[0] != 0x24)
    {
        printk("[LCM ERROR] [0x53]=0x%02x\n", buffer[0]);
        return TRUE;
    }
    else
    {
        printk("[LCM NORMAL] [0x53]=0x%02x\n", buffer[0]);
        return FALSE;
    }
#else
    return FALSE;
#endif

}

unsigned int lcm_ata_check(unsigned char *buffer)
{
#ifndef BUILD_LK
    unsigned int ret = 0;
    unsigned int x0 = FRAME_WIDTH/4;
    unsigned int x1 = FRAME_WIDTH*3/4;

    unsigned char x0_MSB = ((x0>>8)&0xFF);
    unsigned char x0_LSB = (x0&0xFF);
    unsigned char x1_MSB = ((x1>>8)&0xFF);
    unsigned char x1_LSB = (x1&0xFF);

    unsigned int data_array[3];
    unsigned char read_buf[4];
    printk("ATA check size = 0x%x,0x%x,0x%x,0x%x\n",x0_MSB,           x0_LSB,x1_MSB,x1_LSB);
    data_array[0]= 0x0005390A;//HS packet
    data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
    data_array[2]= (x1_LSB);
    dsi_set_cmdq(data_array, 3, 1);

    data_array[0] = 0x00043700;// read id return two byte,            version and id
    dsi_set_cmdq(data_array, 1, 1);

    read_reg_v2(0x2A, read_buf, 4);

    if((read_buf[0] == x0_MSB) && (read_buf[1] == x0_LSB)
        && (read_buf[2] == x1_MSB) && (read_buf[3] == x1_LSB))
        ret = 1;

        ret = 0;

    x0 = 0;
    x1 = FRAME_WIDTH - 1;

    x0_MSB = ((x0>>8)&0xFF);
    x0_LSB = (x0&0xFF);
    x1_MSB = ((x1>>8)&0xFF);
    x1_LSB = (x1&0xFF);

    data_array[0]= 0x0005390A;//HS packet
    data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
    data_array[2]= (x1_LSB);
    dsi_set_cmdq(data_array, 3, 1);
    return ret;
#else
    return 0;
#endif
}




void* lcm_switch_mode1(int mode)
{
#ifndef BUILD_LK
//customization: 1. V2C config 2 values, C2V config 1 value;         2.      config mode control register
    if(mode == 0)
    {//V2C
        lcm_switch_mode1_cmd.mode = CMD_MODE;
        lcm_switch_mode1_cmd.addr = 0xBB;// mode control addr
        lcm_switch_mode1_cmd.val[0]= 0x13;//enabel GRAM               firstly,      ensure writing one frame to GRAM
        lcm_switch_mode1_cmd.val[1]= 0x10;//disable video             mode        secondly
    }
    else
    {//C2V
        lcm_switch_mode1_cmd.mode = SYNC_PULSE_VDO_MODE;
        lcm_switch_mode1_cmd.addr = 0xBB;
        lcm_switch_mode1_cmd.val[0]= 0x03;//disable GRAM              and          enable video mode
    }
    return (void*)(&lcm_switch_mode1_cmd);
#else
    return NULL;
#endif
}

LCM_DRIVER rm68191_tm45_linglong_qhd_lcm_drv =
{
	.name			= "rm68191_tm45_linglong_qhd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id    = lcm_compare_id,
};
