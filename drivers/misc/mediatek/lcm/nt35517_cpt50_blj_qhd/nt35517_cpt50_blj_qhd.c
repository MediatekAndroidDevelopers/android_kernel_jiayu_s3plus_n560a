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
#include <platform/mtk_auxadc_sw.h>
#include <platform/mtk_auxadc_hw.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#else
#include <linux/string.h>
#include <mach/mt_gpio.h>
#include <linux/proc_fs.h> //proc file use
#include <mach/mt_pm_ldo.h>
#include <mach/mt_gpio.h>
#include <cust_pmic.h>
#endif
#include "lcm_drv.h"
#include <cust_gpio_usage.h>

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH                  (540)
#define FRAME_HEIGHT                 (960)
#define LCM_ID                       (0x5517)

#define REGFLAG_DELAY               (0XFE)
#define REGFLAG_END_OF_TABLE        (0x100) // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE         0
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

#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))

#define UDELAY(n)                                           (lcm_util.udelay(n))
#define MDELAY(n)                                           (lcm_util.mdelay(n))

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)       lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                      lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                  lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                           lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)               lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

struct LCM_setting_table {
  unsigned cmd;
  unsigned char count;
  unsigned char para_list[64];
};
static bool lcm_is_init = false;
static void lcm_init_power(void)
{
#ifdef 	GPIO_LCM_LED_EN
  mt_set_gpio_mode(GPIO_LCM_LED_EN, GPIO_MODE_00);
  mt_set_gpio_dir(GPIO_LCM_LED_EN, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_LCM_LED_EN, GPIO_OUT_ZERO);
#endif  
}
static void lcm_suspend_power(void)
{
#ifdef 	GPIO_LCM_LED_EN
  mt_set_gpio_mode(GPIO_LCM_LED_EN, GPIO_MODE_00);
  mt_set_gpio_dir(GPIO_LCM_LED_EN, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_LCM_LED_EN, GPIO_OUT_ZERO);
#endif
}
static void lcm_resume_power(void)
{
#ifdef 	GPIO_LCM_LED_EN
  mt_set_gpio_mode(GPIO_LCM_LED_EN, GPIO_MODE_00);
  mt_set_gpio_dir(GPIO_LCM_LED_EN, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_LCM_LED_EN, GPIO_OUT_ONE);
#endif  
}

static struct LCM_setting_table lcm_initialization_setting[] = {
  //OTM9608A_LG5.0_MIPI_20141119_gamma2.2
  {0x00, 1, {0x00}},
  {0xff, 3, {0x96,0x08,0x01}},

  {0x00, 1, {0x80}},
  {0xff, 2, {0x96,0x08}},

  {0x00, 1, {0xb1}},
  {0xb0, 2, {0x03,0x06}},

  {0x00, 1, {0x00}},
  {0xa0, 1, {0x00}},

  {0x00, 1, {0xb7}},
  {0xb0, 1, {0x10}},

  {0x00, 1, {0xc0}},
  {0xb0, 1, {0x55}},

  {0x00, 1, {0x80}},
  {0xb3, 5, {0x00,0x00,0x20,0x00,0x00}},

  {0x00, 1, {0xa0}},
  {0xb3, 2, {0x10,0x00}},

  {0x00, 1, {0xc0}},
  {0xb3, 1, {0x09}},

  {0x00, 1, {0x80}},
  {0xc0, 9, {0x00,0x48,0x00,0x10,0x10,0x00,0x47,0x10,0x10}},

  {0x00, 1, {0x92}},
  {0xc0, 4, {0x00,0x10,0x00,0x13}},

  {0x00, 1, {0xa0}},
  {0xc0, 1, {0x00}},

  {0x00, 1, {0xa2}},
  {0xc0, 3, {0x0c,0x05,0x02}},

  {0x00, 1, {0xb3}},
  {0xc0, 2, {0x00,0x10}},

  {0x00, 1, {0x81}},
  {0xc1, 1, {0x66}},

  {0x00, 1, {0x80}},
  {0xc4, 3, {0x30,0x84,0xfc}},

  {0x00, 1, {0x88}},
  {0xc4, 1, {0x40}},

  {0x00, 1, {0xa0}},
  {0xc4, 8, {0x33,0x09,0x90,0x2b,0x33,0x09,0x90,0x54}},

  {0x00, 1, {0x80}},
  {0xc5, 4, {0x08,0x00,0xa0,0x11}},

  {0x00, 1, {0x90}},
  {0xc5, 7, {0xd6,0x78,0x00,0x57,0x55,0x55,0x34}},//0xd6,0x78,0x00,0x57,0x33,0x33,0x34

  {0x00, 1, {0xa0}},
  {0xc5, 7, {0xd6,0x57,0x00,0x57,0x55,0x55,0x34}},//0x96,0x57,0x00,0x57,0x33,0x33,0x34

  {0x00, 1, {0xb0}},
  {0xc5, 7, {0x04,0xac,0x01,0x00,0x71,0xb1,0x83}},

  {0x00, 1, {0x80}},
  {0xc6, 1, {0x64}},

  {0x00, 1, {0xb0}},
  {0xc6, 5, {0x03,0x10,0x00,0x1f,0x12}},

  {0x00, 1, {0x81}},
  {0xd6, 1, {0x00}},

  {0x00, 1, {0x80}},
  {0xcb, 10, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

  {0x00, 1, {0x90}},
  {0xcb, 15, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

  {0x00, 1, {0xa0}},
  {0xcb, 15, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

  {0x00, 1, {0xb0}},
  {0xcb, 10, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

  {0x00, 1, {0xc0}},
  {0xcb, 15, {0x04,0x04,0x04,0x04,0x08,0x04,0x08,0x04,0x08,0x04,0x08,0x04,0x04,0x04,0x08}},

  {0x00, 1, {0xd0}},
  {0xcb, 15, {0x08,0x00,0x00,0x00,0x00,0x04,0x04,0x04,0x04,0x08,0x04,0x08,0x04,0x08,0x04}},

  {0x00, 1, {0xe0}},
  {0xcb, 10, {0x08,0x04,0x04,0x04,0x08,0x08,0x00,0x00,0x00,0x00}},

  {0x00, 1, {0xf0}},
  {0xcb, 10, {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}},

  {0x00, 1, {0x80}},
  {0xcc, 10, {0x26,0x25,0x23,0x24,0x00,0x0f,0x00,0x0d,0x00,0x0b}},

  {0x00, 1, {0x90}},
  {0xcc, 15, {0x00,0x09,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x26,0x25,0x21,0x22,0x00}},

  {0x00, 1, {0xa0}},
  {0xcc, 15, {0x10,0x00,0x0e,0x00,0x0c,0x00,0x0a,0x02,0x04,0x00,0x00,0x00,0x00,0x00,0x00}},

  {0x00, 1, {0xb0}},
  {0xcc, 10, {0x25,0x26,0x21,0x22,0x00,0x0a,0x00,0x0c,0x00,0x0e}},

  {0x00, 1, {0xc0}},
  {0xcc, 15, {0x00,0x10,0x04,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x25,0x26,0x23,0x24,0x00}},

  {0x00, 1, {0xd0}},
  {0xcc, 15, {0x09,0x00,0x0b,0x00,0x0d,0x00,0x0f,0x03,0x01,0x00,0x00,0x00,0x00,0x00,0x00}},

  {0x00, 1, {0x80}},
  {0xce, 12, {0x8a,0x03,0x06,0x89,0x03,0x06,0x88,0x03,0x06,0x87,0x03,0x06}},

  {0x00, 1, {0x90}},
  {0xce, 14, {0xf0,0x00,0x00,0xf0,0x00,0x00,0xf0,0x00,0x00,0xf0,0x00,0x00,0x00,0x00}},

  {0x00, 1, {0xa0}},
  {0xce, 14, {0x38,0x02,0x03,0xc1,0x00,0x06,0x00,0x38,0x01,0x03,0xc2,0x00,0x06,0x00}},

  {0x00, 1, {0xb0}},
  {0xce, 14, {0x38,0x00,0x03,0xc3,0x00,0x06,0x00,0x30,0x00,0x03,0xc4,0x00,0x06,0x00}},

  {0x00, 1, {0xc0}},
  {0xce, 14, {0x38,0x06,0x03,0xbd,0x00,0x06,0x00,0x38,0x05,0x03,0xbe,0x00,0x06,0x00}},

  {0x00, 1, {0xd0}},
  {0xce, 14, {0x38,0x04,0x03,0xbf,0x00,0x06,0x00,0x38,0x03,0x03,0xc0,0x00,0x06,0x00}},

  {0x00, 1, {0x80}},
  {0xcf, 14, {0xf0,0x00,0x00,0x10,0x00,0x00,0x00,0xf0,0x00,0x00,0x10,0x00,0x00,0x00}},

  {0x00, 1, {0x90}},
  {0xcf, 14, {0xf0,0x00,0x00,0x10,0x00,0x00,0x00,0xf0,0x00,0x00,0x10,0x00,0x00,0x00}},

  {0x00, 1, {0xa0}},
  {0xcf, 14, {0xf0,0x00,0x00,0x10,0x00,0x00,0x00,0xf0,0x00,0x00,0x10,0x00,0x00,0x00}},

  {0x00, 1, {0xb0}},
  {0xcf, 14, {0xf0,0x00,0x00,0x10,0x00,0x00,0x00,0xf0,0x00,0x00,0x10,0x00,0x00,0x00}},

  {0x00, 1, {0xc0}},
  {0xcf, 10, {0x02,0x02,0x20,0x20,0x00,0x00,0x01,0x00,0x00,0x02}},

  {0x00, 1, {0x00}},
  {0xd8, 2, {0x87,0x87}},

  {0x00, 1, {0x00}},
  {0xd9, 1, {0x65}},//65  61   

  {0x00, 1, {0x00}},
  {0xe1, 16, {0x01,0x0b,0x11,0x0d,0x06,0x0d,0x0a,0x08,0x05,0x08,0x0e,0x08,0x0f,0x13,0x0d,0x08}},

  {0x00, 1, {0x00}},
  {0xe2, 16, {0x01,0x0c,0x12,0x0d,0x06,0x0d,0x0a,0x08,0x05,0x08,0x0e,0x08,0x0f,0x13,0x0d,0x08}},

  {0x00, 1, {0x00}},
  {0xff, 3, {0xff,0xff,0xff}},

  // Setting ending by predefined flag
  {0x11,1,{0x00}},
  {REGFLAG_DELAY, 120, {}},

  {0x29,1,{0x00}},
  {REGFLAG_DELAY, 20, {}},
  {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
  // Sleep Out
  {0x11, 1, {0x00}},
  {REGFLAG_DELAY, 120, {}},
  // Display ON
  {0x29, 1, {0x00}},
  {REGFLAG_DELAY, 20, {}},
  {REGFLAG_END_OF_TABLE, 0x00, {}}
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

static void lcm_get_params(LCM_PARAMS *params)
{
  memset(params, 0, sizeof(LCM_PARAMS));
  params->type   = LCM_TYPE_DSI;
  params->width  = FRAME_WIDTH;
  params->height = FRAME_HEIGHT;

  // enable tearing-free
  params->dbi.te_mode             = LCM_DBI_TE_MODE_DISABLED;
  params->dbi.te_edge_polarity        = LCM_POLARITY_RISING;
  params->dsi.mode   = SYNC_EVENT_VDO_MODE;
  // DSI
  /* Command mode setting */
  params->dsi.LANE_NUM                = LCM_TWO_LANE;
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
  params->dsi.vertical_sync_active                = 4;
  params->dsi.vertical_backporch                  = 16;
  params->dsi.vertical_frontporch                 = 20;
  params->dsi.vertical_active_line                = FRAME_HEIGHT; 
  params->dsi.horizontal_sync_active              = 8;
  params->dsi.horizontal_backporch                = 46;//64
  params->dsi.horizontal_frontporch               = 46;//64
  params->dsi.horizontal_blanking_pixel              = 60;
  params->dsi.horizontal_active_pixel            = FRAME_WIDTH;
  // Bit rate calculation

  params->dsi.PLL_CLOCK=220;//247;
} 

static void lcm_init(void)
{
  SET_RESET_PIN(1);
  MDELAY(10);
  SET_RESET_PIN(0);
  MDELAY(20);
  SET_RESET_PIN(1);
  MDELAY(120);

  push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
  SET_RESET_PIN(1);
  MDELAY(10);
  SET_RESET_PIN(0);
  MDELAY(20);
  SET_RESET_PIN(1);
  MDELAY(120);
  push_table(lcm_deep_sleep_mode_in_setting,sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
  SET_RESET_PIN(0);
}


static void lcm_resume(void)
{
  lcm_init();
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
  unsigned int id = 0;
  unsigned char buffer[6];
  unsigned int array[16];

  array[0] = 0x00013708;
  dsi_set_cmdq(array, 1, 1);
  read_reg_v2(0x0a, buffer, 1);
  printk("otm9605a lcm_esd_check %x %x\n",buffer[0],buffer[1]);
  //printk("[%s] tek.xing esd check: id = %x\n", __FUNCTION__, id);

  if(buffer[0] == 0x9c)//00 //9c
  {
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
  lcm_init();
#endif
  return TRUE;
}

static unsigned int lcm_compare_id(void)
{
    unsigned int  data_array[16];
    unsigned char buffer_c5[3];
    unsigned char buffer_04[3];

    SET_RESET_PIN(1);
    MDELAY(20);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(50);

    data_array[0] = 0x00033700;
    dsi_set_cmdq(data_array, 1, 1);
    read_reg_v2(0x04, buffer_04, 3);

    data_array[0] = 0x00033700;
    dsi_set_cmdq(data_array, 1, 1);
    read_reg_v2(0x04, buffer_04, 3);

    data_array[0]=0x00063902;
    data_array[1]=0x52AA55F0;
    data_array[2]=0x00000108;
    dsi_set_cmdq(&data_array,3,1);

    data_array[0] = 0x00033700;
    dsi_set_cmdq(data_array, 1, 1);
    read_reg_v2(0xC5, buffer_c5, 3);
#if defined(BUILD_LK)
printf("%s, [nt35517_cpt47_blj_qhd]  buffer_c5[0] = [0x%x] buffer_c5[1] = [0x%x]\n",__func__, buffer_c5[0], buffer_c5[1]);
#else
printk("%s, [nt35517_cpt47_blj_qhd]  buffer_c5[0] = [0x%x] buffer_c5[1] = [0x%x]\n",__func__, buffer_c5[0], buffer_c5[1]);
#endif

    if ((buffer_c5[0]==0x55)&&(buffer_c5[1]==0x17)){
        return 1;
    }else{
        return 0;
    }


}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER nt35517_cpt50_blj_qhd_lcm_drv = 
{
  .name           = "nt35517_cpt50_qhd",
  .set_util_funcs = lcm_set_util_funcs,
  .get_params     = lcm_get_params,
  .init           = lcm_init,
  .suspend        = lcm_suspend,
  .resume         = lcm_resume,   
  .compare_id    = lcm_compare_id,    
  .init_power     = lcm_init_power,
  .resume_power = lcm_resume_power,
  .suspend_power = lcm_suspend_power,
  // .esd_check   = lcm_esd_check, 
  // .esd_recover   = lcm_esd_recover, 
#if (LCM_DSI_CMD_MODE)
  .update         = lcm_update,
#endif
};

