
#ifdef BUILD_LK
#include <stdio.h>
#include <string.h>
#else
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (854)
#define LCM_ID       (0x79)
#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

static unsigned int lcm_compare_id(void);


#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifdef BUILD_LK
#define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

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
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};
static struct LCM_setting_table lcm_initialization_setting[] = {
//{0xBC,2,{0x04,0x00);

{0xB9,3, {0xFF,0x83,0x79}},  
 // Set Power 
{0xB1,20, {0x44,0x18,0x18,0x31,0x31,0x50,0xD0,0xEE,0x54,0x80,0x38,0x38,0xF8,0x22,0x22,0x22,0x00,0x80,0x30,0x00}},  
   
 // Set Display 
{0xB2,9, {0x82,0xFE,0x0A,0x04,0x00,0x50,0x11,0x42,0x1D}},  
 // Set CYC 
{0xB4,10, {0x04,0x44,0x00,0x7b,0x00,0x7b,0x22,0x84,0x23,0x84}},  
 // Set Panel 
{0xCC,1, {0x02}},  
 // Set Offset 
{0xD2,1, {0x33}},  
 // Set GIP_0 
{0xD3,29, {0x00,0x07,0x00,0x00,0x01,0x0A,0x0A,0x32,0x10,0x05,0x00,0x05,0x03,0x69,0x03,0x69,0x00,0x08,0x00,0x08,0x37,0x33,0x07,0x07,0x37,0x07,0x07,0x37,0x08}},  
// Set GIP_1 
{0xD5,32, {0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x02,0x03,0x00,0x01,0x06,0x07,0x04,0x05,0x18,0x18,0x20,0x21,0x18,0x18,0x24,0x25,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},  
  
 // Set GIP_2 
{0xD6,32, {0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x05,0x04,0x07,0x06,0x01,0x00,0x03,0x02,0x18,0x18,0x25,0x24,0x18,0x18,0x21,0x20,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},  
  
 // Set Gamma 
{0xE0,42, {0x00,0x09,0x0D,0x33,0x3A,0x3F,0x1A,0x39,0x07,0x0B,0x0C,0x17,0x0E,0x11,0x13,0x11,0x13,0x07,0x10,0x11,0x16,0x00,0x08,0x0E,0x33,0x3B,0x3F,0x19,0x39,0x07,0x0A,0x0D,0x18,0x0E,0x10,0x12,0x11,0x12,0x05,0x11,0x12,0x17}}, 
 // Set VCOM 
{0xB6,2, {0x82,0x82}},   //89  84+   79- 
{0x11,1,{0x00}},  // Sleep-Out 
{REGFLAG_DELAY,120, {}},   
{0x29,1,{0x00}}, 
{REGFLAG_DELAY,20, {}}, 
{REGFLAG_END_OF_TABLE, 0x00, {}} 


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

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_DELAY, 10, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 10, {}},
    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
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
/*
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
            
                if (cmd != 0xFF && cmd != 0x2C && cmd != 0x3C) {
                    //#if defined(BUILD_UBOOT)
                    //	printf("[DISP] - uboot - REG_R(0x%x) = 0x%x. \n", cmd, table[i].para_list[0]);
                    //#endif
                    while(read_reg(cmd) != table[i].para_list[0]);		
                }
        }
    }
}
*/

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
#if 0
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type = LCM_TYPE_DSI;
    params->width = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    params->dsi.mode = CMD_MODE;
    params->dsi.LANE_NUM = LCM_TWO_LANE;
    params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
    params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.PLL_CLOCK = 221;
memset(params, 0, sizeof(LCM_PARAMS));
params->type = LCM_TYPE_DSI;
params->width = FRAME_WIDTH;
params->height = FRAME_HEIGHT;

// enable tearing-free
params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
//params->dbi.te_edge_polarity = LCM_POLARITY_RISING;
params->dsi.mode = BURST_VDO_MODE;
// DSI
/* Command mode setting */
params->dsi.LANE_NUM = LCM_TWO_LANE;
//The following defined the fomat for data coming from LCD engine.
params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
// Video mode setting
params->dsi.intermediat_buffer_num = 2;
params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
//params->dsi.word_count=540*3; 
params->dsi.vertical_sync_active = 5;
params->dsi.vertical_backporch = 5;
params->dsi.vertical_frontporch = 6; //15
params->dsi.vertical_active_line = FRAME_HEIGHT;
params->dsi.horizontal_sync_active = 48;
params->dsi.horizontal_backporch = 48; //64
params->dsi.horizontal_frontporch = 48;
params->dsi.horizontal_active_pixel = FRAME_WIDTH;
#endif

memset(params, 0, sizeof(LCM_PARAMS));
params->type = LCM_TYPE_DSI;
params->width = FRAME_WIDTH;
params->height = FRAME_HEIGHT;

// enable tearing-free
params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
params->dbi.te_edge_polarity = LCM_POLARITY_RISING;

//params->dsi.mode = SYNC_EVENT_VDO_MODE;
params->dsi.mode = BURST_VDO_MODE;
// DSI
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
params->dsi.vertical_sync_active = 5;   //4
params->dsi.vertical_backporch = 5; //16
params->dsi.vertical_frontporch = 6; //15
params->dsi.vertical_active_line = FRAME_HEIGHT;
params->dsi.horizontal_sync_active = 48;
params->dsi.horizontal_backporch = 48; //64
params->dsi.horizontal_frontporch = 48;
params->dsi.horizontal_blanking_pixel = 60;
params->dsi.horizontal_active_pixel = FRAME_WIDTH;

// Bit rate calculation
params->dsi.PLL_CLOCK = 210; //dpi clock customization: should config 
}


static void init_lcm_registers(void)
{
    unsigned int data_array[16];

    data_array[0] = 0x00043902;
    data_array[1] = 0x7983ffb9;
    dsi_set_cmdq(data_array, 2, 1); 

    data_array[0] = 0x00153902;
    data_array[1] = 0x181844b1;
    data_array[2] = 0xd0503131;
    data_array[3] = 0x388054ee;
    data_array[4] = 0x2222f838;
    data_array[5] = 0x30800022;
    data_array[6] = 0x00000000;
    dsi_set_cmdq(data_array, 7, 1); 

    data_array[0] = 0x000a3902;
    data_array[1] = 0x0afe82b2;
    data_array[2] = 0x11500004;
    data_array[3] = 0x00001d42;
    dsi_set_cmdq(data_array, 4, 1); 

    data_array[0] = 0x000b3902;
    data_array[1] = 0x004404b4;
    data_array[2] = 0x227b007b;
    data_array[3] = 0x00842384;
    dsi_set_cmdq(data_array, 4, 1); 

    data_array[0] = 0x00023902;
    data_array[1] = 0x000002cc;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0x00023902;
    data_array[1] = 0x000033d2;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0x001e3902;
    data_array[1] = 0x000700d3;
    data_array[2] = 0x0a0a0100;
    data_array[3] = 0x00051032;
    data_array[4] = 0x03690305;
    data_array[5] = 0x00080069;
    data_array[6] = 0x07333708;
    data_array[7] = 0x07073707;
    data_array[8] = 0x00000837;
    dsi_set_cmdq(data_array, 9, 1);

    data_array[0] = 0x00213902;
    data_array[1] = 0x181818d5;
    data_array[2] = 0x18181818;
    data_array[3] = 0x00030218;
    data_array[4] = 0x04070601;
    data_array[5] = 0x20181805;
    data_array[6] = 0x24181821;
    data_array[7] = 0x18181825;
    data_array[8] = 0x18181818;
    data_array[9] = 0x00000018;
    dsi_set_cmdq(data_array, 10, 1);

    data_array[0] = 0x00213902;
    data_array[1] = 0x181818d6;
    data_array[2] = 0x18181818;
    data_array[3] = 0x07040518;
    data_array[4] = 0x03000106;
    data_array[5] = 0x25181802;
    data_array[6] = 0x21181824;
    data_array[7] = 0x18181820;
    data_array[8] = 0x18181818;
    data_array[9] = 0x00000018;
    dsi_set_cmdq(data_array, 10, 1);

    #if 0   //gamma2.2
    data_array[0] = 0x002b3902;
    data_array[1] = 0x060300e0;
    data_array[2] = 0x13171010;
    data_array[3] = 0x0c0a072f;
    data_array[4] = 0x15120e17;
    data_array[5] = 0x13071413;
    data_array[6] = 0x03001813;
    data_array[7] = 0x17101006;
    data_array[8] = 0x0a072f13;
    data_array[9] = 0x120e170c;
    data_array[10] = 0x07141315;
    data_array[11] = 0x00181313;
    dsi_set_cmdq(data_array, 12, 1);    
    #else   //gamma2.5  
    data_array[0] = 0x002b3902;
    data_array[1] = 0x090300e0;
    data_array[2] = 0x17171010;
    data_array[3] = 0x0c0a0733;
    data_array[4] = 0x16130f17;
    data_array[5] = 0x170b1413;
    data_array[6] = 0x03001a1b;
    data_array[7] = 0x17101009;
    data_array[8] = 0x0a073317;
    data_array[9] = 0x130f170c;
    data_array[10] = 0x0b141316;
    data_array[11] = 0x001a1b17;
    dsi_set_cmdq(data_array, 12, 1);
    #endif

    data_array[0] = 0x00033902;
    data_array[1] = 0x008080b6;
    dsi_set_cmdq(data_array, 2, 1);

    data_array[0] = 0x00110500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);
    data_array[0] = 0x00290500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(15);
}

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(1);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);
    push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}
/*
static void lcm_init(void)
{
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(1);
    SET_RESET_PIN(1);
    MDELAY(10);

    init_lcm_registers();
}
*/

static void lcm_suspend(void)
{

    push_table(lcm_deep_sleep_mode_in_setting,
            sizeof(lcm_deep_sleep_mode_in_setting) /
            sizeof(struct LCM_setting_table), 1);
    /*SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(50);*/

}

static void lcm_resume(void)
{
    //lcm_init();
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
    unsigned int data_array[16];


#if defined(BUILD_LK)
    printf("%s, %d\n", __func__, level);
#else
    printk("lcm_setbacklight = %d\n", level);
#endif
  
    if(level > 255) 
        level = 255;
    
    data_array[0]= 0x00023902;
    data_array[1] =(0x51|(level<<8));
    dsi_set_cmdq(data_array, 2, 1);
}

static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
        if(lcm_esd_test)
        {
            lcm_esd_test = FALSE;
            return TRUE;
        }

        /// please notice: the max return packet size is 1
        /// if you want to change it, you can refer to the following marked code
        /// but read_reg currently only support read no more than 4 bytes....
        /// if you need to read more, please let BinHan knows.
        /*
                unsigned int data_array[16];
                unsigned int max_return_size = 1;
                
                data_array[0]= 0x00003700 | (max_return_size << 16);    
                
                dsi_set_cmdq(&data_array, 1, 1);
        */

        if(read_reg(0xB6) == 0x42)
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
    lcm_init();

    return TRUE;
}

static unsigned int lcm_compare_id(void)
{
    unsigned char buffer[2];
    unsigned int data_array[16];
    unsigned int id0 = 0, id1 = 0;
    unsigned int id = 0;

    SET_RESET_PIN(1);   //NOTE:should reset LCM firstly
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);

    //*************Enable CMD2 Page1  *******************//
    data_array[0] = 0x00043902;
    data_array[1] = 0x7983FFB9;
    // data_array[2]=0x00000108;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(10);
/*
    data_array[0] = 0x00033902;
    data_array[1] = 0x009351ba;
    dsi_set_cmdq(data_array, 2, 1);
*/
    data_array[0] = 0x00023700; // read id return two byte,version and id
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(10);

    //read_reg_v2(0xF4, buffer, 1);
    read_reg_v2(0x04, buffer, 2);
    id0 = buffer[0];    //we only need ID
    id1 = buffer[1];    //we test buffer 1

    /* NOTE: Here id0 = 0x79, id1 = 0xc0( NOT 0x83); */
    /* id = (id0 << 8) | id1; */
    id = id1;

#if defined(BUILD_LK)
    printf("%s,[darren] hx8379c id0 = 0x%x, id1 = 0x%x, id = 0x%x\n", __func__, id0,id1,id);
#else
    printk("%s,[darren] klutu hx8379 id0 = 0x%x, id1 = 0x%x, id = 0x%x\n", __func__, id0,id1,id);
#endif

    return (LCM_ID == id) ? 1 : 0;

}

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER hx8379c_cmi45_blj_fwvga_lcm_drv = 
{
    .name           = "hx8379c_cmi45_blj_fwvga",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .set_backlight  = lcm_setbacklight,
    .compare_id     = lcm_compare_id,
    .init_power        = lcm_init_power,
    .resume_power = lcm_resume_power,
    .suspend_power = lcm_suspend_power,
    //.update         = lcm_update,
    //.esd_check      = lcm_esd_check,
    //.esd_recover    = lcm_esd_recover,
};

