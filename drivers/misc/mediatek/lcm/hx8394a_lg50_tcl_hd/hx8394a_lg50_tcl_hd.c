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

#define LCM_ID (0x94)
#define GPIO_LCD_RST_EN      (GPIO28 | 0x80000000)
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
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

#define   LCM_DSI_CMD_MODE							0


static void lcm_register(void)
{

	unsigned int data_array[40];
	//////////////////////////////////////////////lrzadd0821////for1222
	

   data_array[0] = 0x00043902;                          
    data_array[1] = 0x9483ffB9; 
    dsi_set_cmdq(&data_array, 2, 1); 
    MDELAY(1);
  

	data_array[0] = 0x00033902;
    data_array[1] = 0x008373Ba;
    dsi_set_cmdq(&data_array, 2, 1);
    MDELAY(1);
    
	data_array[0] = 0x00113902;                          
       data_array[1] = 0x07007cB1; 
	data_array[2] = 0x1111018d;
	data_array[3] = 0x3F3F322A;
	data_array[4] = 0xE6011247;
	data_array[5] = 0x000000E2;
    dsi_set_cmdq(&data_array, 6, 1); 
    MDELAY(1);
	 
   data_array[0] = 0x00073902;                          
    data_array[1] = 0x08c800B2; 
	data_array[2] = 0x00220004; 
    dsi_set_cmdq(&data_array, 3, 1); 
    MDELAY(1);

      data_array[0]=0x00173902;                          
       data_array[1] = 0x320680B4; 
	data_array[2] = 0x15320310; 
	data_array[3] = 0x08103208;                          
       data_array[4] = 0x05430433; 
	data_array[5] = 0x063F0437; 
	data_array[6] = 0x00066161; 
       dsi_set_cmdq(&data_array, 7, 1);
	MDELAY(1);

	data_array[0] = 0x00053902;                          
       data_array[1] = 0x100206BF; 
       data_array[2] = 0x00000004; 
       dsi_set_cmdq(&data_array, 3, 1);
	MDELAY(1);
	 
       data_array[0] = 0xfab61500;
       dsi_set_cmdq(data_array, 1, 1);
       MDELAY(1);

	data_array[0] = 0x00213902;                          
    data_array[1] = 0x000000D5;
	data_array[2] = 0x01000A00;
	data_array[3] = 0x0000CC00;
	data_array[4] = 0x88888800;
	data_array[5] = 0x88888888;
	data_array[6] = 0x01888888;
	data_array[7] = 0x01234567;
	data_array[8] = 0x88888823;
	data_array[9] = 0x00000088;
    dsi_set_cmdq(&data_array, 10, 1); 
    MDELAY(1);
	 
       data_array[0] = 0x09cc1500;
       dsi_set_cmdq(data_array, 1, 1);
       MDELAY(1);
	
	data_array[0] = 0x00053902;                          
       data_array[1] = 0x001000c7; 
       data_array[2] = 0x00000010; 
       dsi_set_cmdq(&data_array, 3, 1);
	MDELAY(1);


       data_array[0] = 0x32d41500;
       dsi_set_cmdq(data_array, 1, 1);
       MDELAY(1);

	data_array[0] = 0x002B3902;                          
       data_array[1] = 0x050300E0; 
	data_array[2] = 0x0d3F332B;                          
       data_array[3] = 0x0D0b052b;
	data_array[4] = 0x13131311;                          
       data_array[5] = 0x03001710;
	data_array[6] = 0x3F332B05;                          
       data_array[7] = 0x0b052b0d;
	data_array[8] = 0x1313110D;                          
       data_array[9] = 0x09171013;
       data_array[10]= 0x09110716;
       data_array[11]= 0x00110716;
       dsi_set_cmdq(&data_array, 12, 1);
	MDELAY(1);
	   
    data_array[0] = 0x00110500;
    dsi_set_cmdq(data_array, 1, 1);
  	MDELAY(150);//200
  	
    data_array[0] = 0x00290500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(30);

}

static void lcm_init(void)
{

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(5);
    SET_RESET_PIN(1);
    MDELAY(120);	
	
	lcm_register();
	//push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
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

#if 0//(LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_PULSE_VDO_MODE;//BURST_VDO_MODE; 
#endif
	
    params->dbi.te_mode                 = LCM_DBI_TE_MODE_DISABLED;
    params->dbi.te_edge_polarity        = LCM_POLARITY_RISING;


	// DSI
	/* Command mode setting */
	//1 Three lane or Four lane
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size=256;

    // Video mode setting
    params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory     leakage
	// Video mode setting		
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
		
	params->dsi.vertical_sync_active				= 4;
	params->dsi.vertical_backporch					= 12;
	params->dsi.vertical_frontporch					= 15;
	params->dsi.vertical_active_line				= FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active				= 40;
	params->dsi.horizontal_backporch				= 86;
	params->dsi.horizontal_frontporch				= 86;
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
	
	//params->dsi.LPX=8; 

	// Bit rate calculation
	params->dsi.PLL_CLOCK = 240;//220,258,285
    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	

/* Vanzo:songlixin on: Tue, 24 Mar 2015 09:17:27 +0800
 */
 #ifdef VANZO_LCM_ESD_CHECK_SUPPORT
    params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable      = 1;
    params->dsi.lcm_esd_check_table[0].cmd          = 0x0a;
    params->dsi.lcm_esd_check_table[0].count        = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1c;
#endif
// End of Vanzo:songlixin
}
static void lcm_suspend(void)
{
	unsigned int data_array[16];
	//unsigned char buffer[2];

  /*data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
        MDELAY(20);

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
        MDELAY(120);*/
        SET_RESET_PIN(1);
        MDELAY(10);
	      SET_RESET_PIN(0);
      	MDELAY(10);
	      SET_RESET_PIN(1);
	     MDELAY(120);

}


static void lcm_resume(void)
{
	   

	
	lcm_init();
	
}
static unsigned int lcm_compare_id(void)
{
	int array[4];
	char buffer[5];
	char id_high=0;
	char id_low=0;
	int id=0;

	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(200);

	array[0] = 0x00043902;
	array[1] = 0x9483FFB9;
	dsi_set_cmdq(array, 2, 1);	

	array[0] = 0x00023700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	
	read_reg_v2(0xF4, buffer, 2);
	id = buffer[0]; //we only need ID
	
	#ifdef BUILD_LK
		printf("hx8394 uboot %s \n", __func__);
		printf("%s id = 0x%08x \n", __func__, id);
	#else
		printk("hx8394 kernel %s \n", __func__);
		printk("%s id = 0x%08x \n", __func__, id);
	#endif
	  
	return (id == LCM_ID) ? 1 : 0;
}

LCM_DRIVER hx8394a_lg50_tcl_hd_lcm_drv =
 {
     .name           = "hx8394a_lg50_tcl_hd",
     .set_util_funcs = lcm_set_util_funcs,
     .get_params     = lcm_get_params,
     .init           = lcm_init,
     .suspend        = lcm_suspend,
     .resume         = lcm_resume,
     //.compare_id     = lcm_compare_id,
 #if (LCM_DSI_CMD_MODE)
     .update         = lcm_update,
 #endif
     .init_power        = lcm_init_power,
     .resume_power = lcm_resume_power,
     .suspend_power = lcm_suspend_power,
 
     };
