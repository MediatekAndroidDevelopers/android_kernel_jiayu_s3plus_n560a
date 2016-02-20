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


#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL,fmt)
#else
#define LCD_DEBUG(fmt)  printk(fmt)
#endif


const static unsigned char LCD_MODULE_ID = 0x02;
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

#define REGFLAG_DELAY             							0xFC
#define REGFLAG_END_OF_TABLE      							0xFD   // END OF REGISTERS MARKER
#warning "klutu"
#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

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

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

struct LCM_setting_table {
  unsigned cmd;
  unsigned char count;
  unsigned char para_list[64];
};

static bool lcm_is_init = false;
static void lcm_init_power(void)
{
}

static void lcm_suspend_power(void)
{
}

static void lcm_resume_power(void)
{
}

static struct LCM_setting_table lcm_initialization_setting_tm[] = {

{0xf0, 5,{0x55,0xaa,0x52,0x08,0x00}},
{0xff, 4,{0xaa,0x55,0xa5,0x80}},
{0xb1, 2,{0x68,0x21}},
{0xbd, 5,{0x01,0xa3,0x20,0x10,0x01}},
{0x6f, 1,{0x02}},
{0xb8, 1,{0x08}},
{0xbb, 2,{0x11,0x11}},
{0xbc, 2,{0x05,0x05}},
{0xb6, 1,{0x01}},
{0xc8, 1,{0x83}},
{0xf0, 5,{0x55,0xaa,0x52,0x08,0x01}},
{0xb0, 2,{0x0f,0x0f}},
{0xb1, 2,{0x0f,0x0f}},
{0xce, 1,{0x66}},
{0xc0, 1,{0x0c}},
{0xb5, 2,{0x05,0x05}},
{0xbe, 1,{0x45}},
{0xb3, 2,{0x2b,0x2b}},
{0xb4, 2,{0x0f,0x0f}},
{0xb9, 2,{0x44,0x44}},
{0xba, 2,{0x24,0x24}},
{0xbc, 2,{0xa0,0x03}},
{0xbd, 2,{0xa0,0x03}},
{0xca, 1,{0x00}},
{0xf0, 5,{0x55,0xaa,0x52,0x08,0x02}},
{0xee, 1,{0x01}},
{0xef, 4,{0x11,0x08,0x16,0x19}},
{0xb0, 16,{0x00,0x00,0x00,0x0a,0x00,0x1a,0x00,0x28,0x00,0x33,0x00,0x47,0x00,0x58,0x00,0x74}},
{0xb1, 16,{0x00,0x8d,0x00,0xdf,0x01,0x2b,0x01,0xa4,0x02,0x06,0x02,0x09,0x02,0x64,0x02,0xc8}},
{0xb2, 16,{0x03,0x09,0x03,0x57,0x03,0x8a,0x03,0xae,0x03,0xc4,0x03,0xdc,0x03,0xe9,0x03,0xf8}},
{0xb3, 4,{0x03,0xfe,0x03,0xff}},
{0x6f, 1,{0x02}},
{0xf8, 1,{0x44}},
{0x6f, 1,{0x0f}},
{0xfa, 1,{0x14}},
{0x6f, 1,{0x02}},
{0xf7, 1,{0xef}},
{0x6f, 1,{0x0a}},
{0xf7, 1,{0x02}},
{0x6f, 1,{0x0c}},
{0xf4, 1,{0xd8}},
{0x6f, 1,{0x0e}},
{0xf4, 1,{0x44}},
{0x6f, 1,{0x17}},
{0xf4, 1,{0x60}},
{0x6f, 1,{0x01}},
{0xf9, 1,{0x46}},
{0xf0, 5,{0x55,0xaa,0x52,0x08,0x06}},
{0xb0, 2,{0x2e,0x2e}},
{0xb1, 2,{0x2e,0x2e}},
{0xb2, 2,{0x2e,0x2e}},
{0xb3, 2,{0x2e,0x09}},
{0xb4, 2,{0x0b,0x11}},
{0xb5, 2,{0x17,0x19}},
{0xb6, 2,{0x23,0x1d}},
{0xb7, 2,{0x25,0x1f}},
{0xb8, 2,{0x01,0x03}},
{0xb9, 2,{0x2e,0x2e}},
{0xba, 2,{0x2e,0x2e}},
{0xbb, 2,{0x02,0x00}},
{0xbc, 2,{0x1e,0x24}},
{0xbd, 2,{0x1c,0x22}},
{0xbe, 2,{0x18,0x16}},
{0xbf, 2,{0x10,0x0a}},
{0xc0, 2,{0x08,0x2e}},
{0xc1, 2,{0x2e,0x2e}},
{0xc2, 2,{0x2e,0x2e}},
{0xc3, 2,{0x2e,0x2e}},
{0xe5, 2,{0x13,0x12}},
{0xc4, 2,{0x2e,0x2e}},
{0xc5, 2,{0x2e,0x2e}},
{0xc6, 2,{0x2e,0x2e}},
{0xc7, 2,{0x2e,0x02}},
{0xc8, 2,{0x00,0x18}},
{0xc9, 2,{0x12,0x10}},
{0xca, 2,{0x24,0x1e}},
{0xcb, 2,{0x22,0x1c}},
{0xcc, 2,{0x0a,0x08}},
{0xcd, 2,{0x2e,0x2e}},
{0xce, 2,{0x2e,0x2e}},
{0xcf, 2,{0x09,0x0b}},
{0xd0, 2,{0x1d,0x23}},
{0xd1, 2,{0x1f,0x25}},
{0xd2, 2,{0x11,0x13}},
{0xd3, 2,{0x19,0x01}},
{0xd4, 2,{0x03,0x2e}},
{0xd5, 2,{0x2e,0x2e}},
{0xd6, 2,{0x2e,0x2e}},
{0xd7, 2,{0x2e,0x2e}},
{0xe6, 2,{0x16,0x17}},
{0xd8, 5,{0x00,0x00,0x00,0x00,0x00}},
{0xd9, 5,{0x00,0x00,0x00,0x00,0x00}},
{0xe7, 1,{0x00}},
{0xf0, 5,{0x55,0xaa,0x52,0x08,0x05}},
{0xed, 1,{0x30}},
{0xf0, 5,{0x55,0xaa,0x52,0x08,0x03}},
{0xb0, 2,{0x20,0x00}},
{0xb1, 2,{0x20,0x00}},
{0xf0, 5,{0x55,0xaa,0x52,0x08,0x05}},
{0xb0, 2,{0x17,0x06}},
{0xb8, 1,{0x00}},
{0xbd, 5,{0x0f,0x03,0x03,0x00,0x03}},
{0xb1, 2,{0x17,0x06}},
{0xb9, 2,{0x00,0x03}},
{0xb2, 2,{0x17,0x06}},
{0xba, 2,{0x00,0x03}},
{0xb3, 2,{0x17,0x06}},
{0xbb, 2,{0x00,0x00}},
{0xb4, 2,{0x17,0x06}},
{0xb5, 2,{0x17,0x06}},
{0xb6, 2,{0x17,0x06}},
{0xb7, 2,{0x17,0x06}},
{0xbc, 2,{0x00,0x01}},
{0xe5, 1,{0x06}},
{0xe6, 1,{0x06}},
{0xe7, 0,{}},
{0x06, 0,{}},
{0xe8, 1,{0x06}},
{0xe9, 1,{0x0a}},
{0xea, 1,{0x06}},
{0xeb, 1,{0x06}},
{0xec, 1,{0x06}},
{0xf0, 5,{0x55,0xaa,0x52,0x08,0x05}},
{0xc0, 1,{0x07}},
{0xc1, 1,{0x05}},
{0xf0, 5,{0x55,0xaa,0x52,0x08,0x03}},
{0xb2, 5,{0x04,0x00,0x52,0x01,0x51}},
{0xb3, 5,{0x04,0x00,0x52,0x01,0x51}},
{0xf0, 5,{0x55,0xaa,0x52,0x08,0x05}},
{0xc4, 1,{0x82}},
{0xc5, 1,{0x80}},
{0xf0, 5,{0x55,0xaa,0x52,0x08,0x03}},
{0xb6, 5,{0x04,0x00,0x52,0x01,0x51}},
{0xb7, 5,{0x04,0x00,0x52,0x01,0x51}},
{0xf0, 5,{0x55,0xaa,0x52,0x08,0x05}},
{0xc8, 2,{0x03,0x20}},
{0xc9, 2,{0x01,0x21}},
{0xca, 2,{0x03,0x20}},
{0xcb, 2,{0x07,0x20}},
{0xf0, 5,{0x55,0xaa,0x52,0x08,0x03}},
{0xc4, 1,{0x60}},
{0xc5, 1,{0x40}},
{0xf0, 5,{0x55,0xaa,0x52,0x08,0x03}},
{0xba, 5,{0x44,0x00,0x60,0x01,0x72}},
{0xbb, 5,{0x44,0x00,0x60,0x01,0x72}},
{0xbc, 5,{0x53,0x00,0x03,0x00,0x48}},
{0xbd, 5,{0x53,0x00,0x03,0x00,0x48}},
{0xf0, 5,{0x55,0xaa,0x52,0x08,0x05}},
{0xd1, 11,{0x03,0x05,0x00,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0xd2, 11,{0x03,0x05,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0xd3, 11,{0x00,0x05,0x04,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0xd4, 11,{0x00,0x05,0x04,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
{0xf0, 5,{0x55,0xaa,0x52,0x08,0x03}},
{0xc4, 1,{0x40}},
{0xc5, 1,{0x40}},
{0xf0, 5,{0x55,0xaa,0x52,0x08,0x00}},
{0x6f, 1,{0x08}},
{0xf2, 1,{0x70}},
{0x6f, 1,{0x02}},
{0xb8, 1,{0x08}},
{0x6f, 1,{0x07}},
{0xf4, 1,{0xf0}},
{0xf0, 5,{0x55,0xaa,0x52,0x08,0x01}},
{0xcb, 1,{0x92}},
{0xcd, 1,{0x08}},
{0x6f, 1,{0x01}},
{0xcd, 1,{0x01}},
{0x6f, 1,{0x01}},
{0xcb, 1,{0x8a}},
{0xbf, 1,{0x60}},
{0xd3, 1,{0x40}},
{0x6f, 1,{0x11}},
{0xf3, 1,{0x01}},

{0x35,1,{0x00}},
{0x11,1,{0x00}},
{REGFLAG_DELAY, 120, {}},
  // Display ON
{0x29, 1, {0x00}},
{REGFLAG_DELAY, 50, {}},

{REGFLAG_END_OF_TABLE, 0x00, {}},
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++)
    {
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

static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type   = LCM_TYPE_DSI;

    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    // enable tearing-free
    params->dbi.te_mode                 = LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity        = LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode   = BURST_VDO_MODE;
#endif

    // DSI
    /* Command mode setting */
    //1 Three lane or Four lane
    params->dsi.LANE_NUM                = LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    //params->dsi.packet_size=256;

    // Video mode setting       
    params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    params->dsi.word_count=720*3;

	params->dsi.vertical_sync_active = 2; //1
	params->dsi.vertical_backporch = 14;   //1
	params->dsi.vertical_frontporch = 14;  //6
	params->dsi.vertical_active_line = FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 4;
	params->dsi.horizontal_backporch = 100;
	params->dsi.horizontal_frontporch = 100;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	//params->dsi.LPX=8;

	// Bit rate calculation
	//1 Every lane speed
    params->dsi.PLL_CLOCK = 240;
    params->dsi.compatibility_for_nvk = 1;
    params->dsi.ssc_disable = 1;

    params->dsi.esd_check_enable            = 1;
    params->dsi.customization_esd_check_enable  = 0;


}


/*to prevent electric leakage*/
static void lcm_id_pin_handle(void)
{
  //    mt_set_gpio_pull_select(GPIO_DISP_ID0_PIN,GPIO_PULL_UP);
  //    mt_set_gpio_pull_select(GPIO_DISP_ID1_PIN,GPIO_PULL_DOWN);
}

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(50);

    push_table(lcm_initialization_setting_tm, sizeof(lcm_initialization_setting_tm) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
    SET_RESET_PIN(0);
    MDELAY(5);
}
static void lcm_resume(void)
{
    lcm_init();
}

static unsigned int lcm_compare_id(void)
{
    unsigned int id=0;
    unsigned char buffer[3];
    unsigned int array[16];
    unsigned int data_array[16];

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(50);

    SET_RESET_PIN(1);
    MDELAY(120);

    data_array[0] = 0x00063902;
    data_array[1] = 0x52AA55F0;
    data_array[2] = 0x00000108;
    dsi_set_cmdq(&data_array, 3, 1);

    array[0] = 0x00033700;// read id return two byte,version and id
    dsi_set_cmdq(array, 1, 1);

    read_reg_v2(0xC5, buffer, 3);
    id = buffer[1]; //we only need ID
#ifdef BUILD_LK
    printf("%s, LK nt35521 debug: nt35521 id = 0x%08x buffer[0]=0x%08x,buffer[1]=0x%08x,buffer[2]=0x%08x\n", __func__, id,buffer[0],buffer[1],buffer[2]);
#else
    printk("%s, LK nt35521 debug: nt35521 id = 0x%08x buffer[0]=0x%08x,buffer[1]=0x%08x,buffer[2]=0x%08x\n", __func__, id,buffer[0],buffer[1],buffer[2]);
#endif

    // if(id == LCM_ID_NT35521)
    if(buffer[0]==0x55 && buffer[1]==0x21)
        return 1;
    else
        return 0;
}


LCM_DRIVER nt35521_auo55_datong_hd_lcm_drv =
{
    .name			= "nt35521_auo55_datong_hd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
    .init_power     = lcm_init_power,
    .resume_power   = lcm_resume_power,
    .suspend_power  = lcm_suspend_power,
    };
