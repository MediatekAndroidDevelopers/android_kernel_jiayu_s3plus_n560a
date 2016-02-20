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

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)
#define LCM_ID			(0x1284)

#define REGFLAG_DELAY          		(0XFEE)
#define REGFLAG_END_OF_TABLE      	(0xFFFF)	// END OF REGISTERS MARKER

#ifndef FALSE
#define FALSE 0
#endif
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};
LCM_DSI_MODE_SWITCH_CMD lcm_switch_mode1_cmd;
#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))
#define REGFLAG_DELAY                                           0xFC
#define REGFLAG_END_OF_TABLE                                0xFD   // END OF REGISTERS MARKER


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
unsigned cmd;
unsigned char count;
unsigned char para_list[64];
};

#define   LCM_DSI_CMD_MODE							0

static struct LCM_setting_table lcm_initialization_setting[] = {
//////////////20140909-add/start/////////////////
{0x00,1,{0x00}},
{0xff, 3,{0x12,0x84,0x01}},

{0x00,1,{0x80}},        //Orise mode enable
{0xff, 2,{0x12,0x84}},

{0x00, 1,{0x92}},
{0xff, 2,{0x20,0x02}},

{0x00,1,{0x80}},            //TCON Setting
{0xc0, 9,{0x00,0x64,0x00,0x10,0x10,0x00,0x64,0x10,0x10}},

{0x00,1,{0x90}},            //Panel Timing Setting
{0xc0, 6,{0x00,0x5c,0x00,0x01,0x00,0x04}},

{0x00,1,{0xb3}},            //Interval Scan Frame: 0 frame, column inversion
{0xc0, 2,{0x00,0x55}},

{0x00,1,{0x81}},            //frame rate:60Hz
{0xc1,1,{0x66}},//55
         
{0x00,1,{0xa0}},
{0xc4,14,{0x05,0x10,0x04,0x02,0x05,0x15,0x11,0x05,0x10,0x07,0x02,0x05,0x15,0x11}},

{0x00,1,{0xb0}},            //clamp voltage setting
{0xc4,2,{0x00,0x00}},


{0x00, 1,{0x91}},
{0xc5, 2,{0xa6,0xd2}},

{0x00,1,{0x00}},             //GVDD=4.204V, NGVDD=-4.204V 
{0xd8, 2,{0xc7,0xc7}},

{0x00,1,{0x00}},         //VCOM=0.240V
{0xd9, 1,{0x6a}},//40

{0x00, 1,{0xb3}},
{0xc5, 1,{0x84}},

{0x00,1,{0xbb}},            //LVD voltage level setting
{0xc5, 1,{0x8a}},

{0x00,1,{0x82}},		//chopper
{0xc4, 1,{0x0a}},

{0x00,1,{0xc6}},		//debounce
{0xb0,1,{0x03}},

{0x00, 1,{0xc2}},
{0xf5, 1,{0x40}},

{0x00, 1,{0xc3}},
{0xf5, 1,{0x85}},

{0x00,1,{0x00}},             //ID1
{0xd0,1,{0x40}},

{0x00,1,{0x00}},             //ID2, ID3
{0xd1,2,{0x00,0x00}},

{0x00, 1,{0xb2}},
{0xf5, 2,{0x00,0x00}},

{0x00, 1,{0xb4}},
{0xf5, 2,{0x00,0x00}},

{0x00, 1,{0xb6}},
{0xf5, 2,{0x00,0x00}},

{0x00, 1,{0xb8}},
{0xf5, 2,{0x00,0x00}},

{0x00,1,{0x94}},  //VCL on  	
{0xf5, 2,{0x00,0x00}},

{0x00, 1,{0xd2}},
{0xf5, 2,{0x06,0x15}},

{0x00,1,{0xb4}},             //VGLO1/2 Pull low setting
{0xc5, 1,{0xcc}},

{0x00,1,{0x90}},             //Mode-3
{0xf5, 4,{0x02,0x11,0x02,0x15}},

{0x00,1,{0x90}},             //2xVPNL, 1.5*=00, 2*=50, 3*=a0
{0xc5,1,{0xa0}},

{0x00,1,{0x94}},             //Frequency
{0xc5,1,{0x66}},

{0x00, 1,{0xa0}},
{0xc1, 1,{0x02}},

{0x00, 1,{0x80}},
{0xd6, 1,{0x58}},//fae

//-------------------- panel timing state control --------------------//
{0x00,1,{0x80}},            //panel timing state control
{0xcb,11,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0x90}},            //panel timing state control
{0xcb, 15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xa0}},            //panel timing state control
{0xcb, 15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xb0}},            //panel timing state control
{0xcb, 15,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xc0}},            //panel timing state control
{0xcb, 15,{0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x00,0x05,0x00,0x00,0x00,0x00}},

{0x00,1,{0xd0}},            //panel timing state control
{0xcb, 15,{0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05}},

{0x00,1,{0xe0}},            //panel timing state control
{0xcb, 14,{0x05,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x00,0x00}},

{0x00,1,{0xf0}},            //panel timing state control
{0xcb,11,{0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}},

//-------------------- panel pad mapping control --------------------//
{0x00,1,{0x80}},             //panel pad mapping control
{0xcc, 15,{0x29,0x2a,0x0a,0x0c,0x0e,0x10,0x12,0x14,0x06,0x00,0x08,0x00,0x00,0x00,0x00}},

{0x00,1,{0x90}},             //panel pad mapping control
{0xcc, 15,{0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x29,0x2a,0x09,0x0b,0x0d,0x0f,0x11,0x13}},

{0x00,1,{0xa0}},             //panel pad mapping control
{0xcc, 14,{0x05,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x00}},

{0x00,1,{0xb0}},             //panel pad mapping control
{0xcc, 15,{0x29,0x2a,0x13,0x11,0x0f,0x0d,0x0b,0x09,0x01,0x00,0x07,0x00,0x00,0x00,0x00}},

{0x00,1,{0xc0}},             //panel pad mapping control
{0xcc, 15,{0x00,0x00,0x00,0x00,0x05,0x00,0x00,0x29,0x2a,0x14,0x12,0x10,0x0e,0x0c,0x0a}},

{0x00,1,{0xd0}},             //panel pad mapping control
{0xcc, 14,{0x02,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x00,0x00}},

//-------------------- panel timing setting --------------------//
{0x00,1,{0x80}},             //panel VST setting
{0xce, 12,{0x89,0x05,0x10,0x88,0x05,0x10,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0x90}},
{0xce,14,{0x54,0xfc,0x10,0x54,0xfd,0x10,0x55,0x00,0x10,0x55,0x01,0x10,0x00,0x00}},

{0x00,1,{0xa0}},            //panel CLKA1/2 setting
{0xce, 14,{0x58,0x07,0x04,0xfc,0x00,0x10,0x00,0x58,0x06,0x04,0xfd,0x00,0x10,0x00}},

{0x00,1,{0xb0}},            //panel CLKA3/4 setting
{0xce, 14,{0x58,0x05,0x04,0xfe,0x00,0x10,0x00,0x58,0x04,0x04,0xff,0x00,0x10,0x00}},

{0x00,1,{0xc0}},            //panel CLKb1/2 setting
{0xce, 14,{0x58,0x03,0x05,0x00,0x00,0x10,0x00,0x58,0x02,0x05,0x01,0x00,0x10,0x00}},

{0x00,1,{0xd0}},            //panel CLKb3/4 setting
{0xce, 14,{0x58,0x01,0x05,0x02,0x00,0x10,0x00,0x58,0x00,0x05,0x03,0x00,0x10,0x00}},

{0x00,1,{0x80}},            //panel CLKc1/2 setting
{0xcf, 14,{0x50,0x00,0x05,0x04,0x00,0x10,0x00,0x50,0x01,0x05,0x05,0x00,0x10,0x00}},

{0x00,1,{0x90}},            //panel CLKc3/4 setting
{0xcf, 14,{0x50,0x02,0x05,0x06,0x00,0x10,0x00,0x50,0x03,0x05,0x07,0x00,0x10,0x00}},

{0x00,1,{0xa0}},            //panel CLKd1/2 setting
{0xcf, 14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xb0}},            //panel CLKd3/4 setting
{0xcf, 14,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

{0x00,1,{0xc0}},             //panel ECLK setting
{0xcf, 11,{0x39,0x39,0x20,0x20,0x00,0x00,0x01,0x01,0x20,0x00,0x00}},

{0x00, 1,{0xb5}},
{0xc5, 6,{0x0b,0x95,0xff,0x0b,0x95,0xff}},

{0x00,1,{0x00}}, 
{0xe1, 20,{0x00,0x2e,0x3d,0x49,0x5a,0x66,0x67,0x8e,0x7d,0x93,0x72,0x5e,0x72,0x4f,0x4d,0x41,0x32,0x25,0x19,0x00}},

{0x00,1,{0x00}}, 
{0xe2, 20,{0x00,0x2e,0x3d,0x49,0x5a,0x67,0x68,0x8f,0x7d,0x93,0x72,0x5f,0x72,0x4f,0x4d,0x41,0x32,0x25,0x19,0x00}},


{0x00,1,{0x00}},             //Orise mode disable
{0xff,3,{0xff,0xff,0xff}},

{0x11,1,{0x00}},//  
{REGFLAG_DELAY,120,{}},                                                                                                 
                                   				                                                                                
{0x29,1,{0x00}}, //Display on (}},
{REGFLAG_DELAY,30,{}},  {REGFLAG_END_OF_TABLE, 0x00, {}}
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
static void lcm_init(void)
{

    SET_RESET_PIN(1);
    MDELAY(5);
    SET_RESET_PIN(0);
    MDELAY(5);
    SET_RESET_PIN(1);
    MDELAY(150);

    push_table(lcm_initialization_setting,
            sizeof(lcm_initialization_setting) /
            sizeof(struct LCM_setting_table), 1);
}
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

        #if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
        #else
	params->dsi.mode   = SYNC_EVENT_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
        #endif

		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
    params->dsi.LANE_NUM = LCM_THREE_LANE;
		//The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    params->dsi.intermediat_buffer_num = 0;	//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage
		// Video mode setting
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    params->dsi.word_count = 720 * 3;

	params->dsi.vertical_sync_active				= 8;//2//      //4
	params->dsi.vertical_backporch					= 16;//8       //12
	params->dsi.vertical_frontporch					= 16;//6       //15
		params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active				= 8;//86 20    //50
	params->dsi.horizontal_backporch				= 48;//55 50    //90
	params->dsi.horizontal_frontporch				= 48;//55	50  //90
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

	    //params->dsi.LPX=8;

		// Bit rate calculation
		params->dsi.PLL_CLOCK = 208;

	params->dsi.noncont_clock = TRUE;
    params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable = 1;
    params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
    params->dsi.lcm_esd_check_table[0].count        = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;

}


static void lcm_suspend(void)
{
	unsigned int data_array[16];
	//unsigned char buffer[2];

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
        MDELAY(20);

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
        MDELAY(120);

}

static void lcm_resume(void)
{
	unsigned int data_array[16];
	data_array[0] = 0x00110500; // Sleep Out
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(120);
	data_array[0] = 0x00290500; // Display On
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(20);
}
static unsigned int lcm_compare_id(void)
{
    unsigned int id0, id1, id2, id3, id4;
    unsigned char buffer[5];
    unsigned int array[5];


    SET_RESET_PIN(1);
	MDELAY(10);
    SET_RESET_PIN(0);
	MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(100);
   

    // Set Maximum return byte = 1
    array[0] = 0x00053700;
    dsi_set_cmdq(array, 1, 1);

    read_reg_v2(0xA1, buffer, 5);
    id0 = buffer[0];
    id1 = buffer[1];
    id2 = buffer[2];
    id3 = buffer[3];
    id4 = buffer[4];

#if defined(BUILD_LK)
    printf("%s, otm1283a Module ID = {%x, %x, %x, %x, %x} \n", __func__, id0,
            id1, id2, id3, id4);
#else
    printk("%s, otm1283a Module ID = {%x, %x, %x, %x,%x} \n", __func__, id0,
            id1, id2, id3, id4);
#endif
    id0 = (id2 << 8) | id3;
    //return (LCM_ID == ((id2 << 8) | id3)) ? 1 : 0;

#if defined(BUILD_LK)
    printf("otm1283a Module ID = %x \n", id0);
#else
    printk("otm1283a Module ID = %x \n", id0);
#endif

#if 1
    if(id0 == LCM_ID)
        return 1;
    else
        return 0;
#else
    return 1;
#endif
}

LCM_DRIVER otm1284a_cmi50_cs_hd_lcm_drv =
 {
    .name = "otm1284a_cmi50_cs_hd",
     .set_util_funcs = lcm_set_util_funcs,
     .get_params     = lcm_get_params,
     .init           = lcm_init,
     .suspend        = lcm_suspend,
     .resume         = lcm_resume,
    .compare_id = lcm_compare_id,
 #if (LCM_DSI_CMD_MODE)
     .update         = lcm_update,
 #endif
     .init_power        = lcm_init_power,
     .resume_power = lcm_resume_power,
     .suspend_power = lcm_suspend_power,

     };
