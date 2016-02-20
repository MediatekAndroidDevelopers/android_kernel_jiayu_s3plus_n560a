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

#define FRAME_WIDTH  (1080)
#define FRAME_HEIGHT (1920)

#define LCM_ID_RM63315          0x3311
#define GPIO_LCM_RST      (GPIO146 | 0x80000000)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))
#define REGFLAG_DELAY                                           0xFC
#define REGFLAG_END_OF_TABLE                                0xFD   // END OF REGISTERS MARKER


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define dsi_lcm_set_gpio_out(pin, out)										lcm_util.set_gpio_out(pin, out)
#define dsi_lcm_set_gpio_mode(pin, mode)									lcm_util.set_gpio_mode(pin, mode)
#define dsi_lcm_set_gpio_dir(pin, dir)										lcm_util.set_gpio_dir(pin, dir)
#define dsi_lcm_set_gpio_pull_enable(pin, en)								lcm_util.set_gpio_pull_enable(pin, en)

#define   LCM_DSI_CMD_MODE							0

static bool lcm_is_init = false;
static void lcm_init_power(void)
{

  mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
  mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);

  mt_set_gpio_mode(GPIO_LCM_PWR_EN, GPIO_MODE_00);
  mt_set_gpio_dir(GPIO_LCM_PWR_EN, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ONE);
}


static void lcm_suspend_power(void)
{
  mt_set_gpio_mode(GPIO_LCM_PWR_EN, GPIO_MODE_00);
  mt_set_gpio_dir(GPIO_LCM_PWR_EN, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ZERO);

  mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
  mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
  mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);

  mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
}

static void lcm_resume_power(void)
{
  mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
  mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ONE);
}

static void init_lcm_registers(void)
{
  unsigned int data_array[16];

	data_array[0]=0x00022902;
	data_array[1]=0x000004b0;//cmd mode
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]=0x00022902;
	data_array[1]=0x000001d6;//cmd mode
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]=0x00072902;//Interface_setting(Video mode)
	data_array[1]=0x000014b3;//cmd mode
	data_array[2]=0x00000000;//cmd mode
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]=0x00032902;//Interface_ID_setting(DSI 4lane)///////////
	data_array[1]=0x00000cb4;//cmd mode
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]=0x00032902;//DSI_control
	data_array[1]=0x00d33ab6;//cmd mode
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]=0x00232902;//Display_Setting
	data_array[1]=0x406084c1;//cmd mode//0x406004c1
	data_array[2]=0xce6fffeb;//cmd mode
	data_array[3]=0x0217ffff;//cmd mode
	data_array[4]=0xb1ae7358;//cmd mode
	data_array[5]=0xffffc620;//cmd mode
	data_array[6]=0x5ffff31f;//cmd mode
	data_array[7]=0x10101010;//cmd mode
	data_array[8]=0x02010000;//cmd mode
	data_array[9]=0x00010002;//cmd mode
	dsi_set_cmdq(data_array, 10, 1);

	data_array[0]=0x00082902;// (sub pix-inv:0x30)
	data_array[1]=0x80f731c2;//cmd mode
	data_array[2]=0x00000808;//cmd mode
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]=0x000172902;////Source_Timing_Setting
	data_array[1]=0x000070c4;//cmd mode
	data_array[2]=0x00040000;//cmd mode
	data_array[3]=0x060c0003;//cmd mode
	data_array[4]=0x00000000;//cmd mode
	data_array[5]=0x03000400;//cmd mode
	data_array[6]=0x00060c00;//cmd mode
	dsi_set_cmdq(data_array, 7, 1);

	data_array[0]=0x00292902;
	data_array[1]=0x007900c6;//cmd mode
	data_array[2]=0x00790079;//cmd mode
	data_array[3]=0x00000000;//cmd mode
	data_array[4]=0x00790079;//cmd mode
	data_array[5]=0x07191079;//cmd mode
	data_array[6]=0x79000100;//cmd mode
	data_array[7]=0x79007900;//cmd mode
	data_array[8]=0x00000000;//cmd mode
	data_array[9]=0x79007900;//cmd mode
	data_array[10]=0x19107900;//cmd mode
	data_array[11]=0x00000007;//cmd mode
	dsi_set_cmdq(data_array, 12, 1);

	data_array[0]=0x00192902;//Gamma_Setting_A
	data_array[1]=0x1e1601C7;//cmd mode
	data_array[2]=0x3f4b3427;//cmd mode
	data_array[3]=0x70685e53;//cmd mode
	data_array[4]=0x1e16017c;//cmd mode
	data_array[5]=0x3f4b3427;//cmd mode
	data_array[6]=0x70685e53;//cmd mode
	data_array[7]=0x0000007c;//cmd mode
	dsi_set_cmdq(data_array, 8, 1);

	data_array[0]=0x00192902;//Gamma_Setting_B
	data_array[1]=0x1e1601C8;//cmd mode
	data_array[2]=0x3f4b3427;//cmd mode
	data_array[3]=0x70685e53;//cmd mode
	data_array[4]=0x1e16017c;//cmd mode
	data_array[5]=0x3f4b3427;//cmd mode
	data_array[6]=0x70685e53;//cmd mode
	data_array[7]=0x0000007c;//cmd mode
	dsi_set_cmdq(data_array, 8, 1);

	data_array[0]=0x00192902;//Gamma_Setting_C
	data_array[1]=0x1e1601C9;//cmd mode
	data_array[2]=0x3f4b3427;//cmd mode
	data_array[3]=0x70685e53;//cmd mode
	data_array[4]=0x1e16017c;//cmd mode
	data_array[5]=0x3f4b3427;//cmd mode
	data_array[6]=0x70685e53;//cmd mode
	data_array[7]=0x0000007c;//cmd mode
	dsi_set_cmdq(data_array, 8, 1);

	data_array[0]=0x00212902;//Color_enhancement set ce
	data_array[1]=0x80a000Ca;//cmd mode 
	data_array[2]=0x80808080;//cmd mode 
	data_array[3]=0x00200c80;//cmd mode 
	data_array[4]=0x374a0aff;//cmd mode 
	data_array[5]=0x0cf855a0;//cmd mode 
	data_array[6]=0x2010200c;//cmd mode 
	data_array[7]=0x10000020;//cmd mode 
	data_array[8]=0x3f3f3f10;//cmd mode 
	data_array[9]=0x0000003f;//cmd mode
	dsi_set_cmdq(data_array, 10, 1);

	data_array[0]=0x000a2902;//Panel PIN Control
	data_array[1]=0x3FFC31CB;//cmd mode
	data_array[2]=0x0000008C;//cmd mode
	data_array[3]=0x0000C000;//cmd mode
	dsi_set_cmdq(data_array, 4, 1);

	data_array[0]=0x00022902;//Panel Interface Control (Type B)
	data_array[1]=0x00000acc;//cmd mode
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]=0x000f2902;//Power Setting 1
	data_array[1]=0x190000D0;//cmd mode
	data_array[2]=0x19999918;//cmd mode
	data_array[3]=0x55008901;//cmd mode
	data_array[4]=0x0001d959;//cmd mode
	dsi_set_cmdq(data_array, 5, 1);

	data_array[0]=0x001b2902;//Power Setting for Internal Power
	data_array[1]=0xBB331BD3;//cmd mode
	data_array[2]=0x3333C4CC;//cmd mode
	data_array[3]=0x00010033;//cmd mode
	data_array[4]=0x0DA0D8A0;//cmd mode
	data_array[5]=0x22443348;//cmd mode
	data_array[6]=0x53480270;//cmd mode
	data_array[7]=0x0099BF3D;//cmd mode
	dsi_set_cmdq(data_array, 8, 1);

	data_array[0]=0x00082902;
	data_array[1]=0x000006d5;//cmd mode
	data_array[2]=0x38013801;//cmd mode
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(60);

	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);

	MDELAY(200);
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
		 params->dsi.mode   = BURST_VDO_MODE;
        #endif

		// DSI
		/* Command mode setting */
		//1 Three lane or Four lane
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

		// Highly depends on LCD driver capability.
		// Not support in MT6573
		params->dsi.packet_size=256;

		// Video mode setting
		params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;


		params->dsi.vertical_sync_active				= 2;
		params->dsi.vertical_backporch					= 6;
		params->dsi.vertical_frontporch					= 8;
		params->dsi.vertical_active_line				= FRAME_HEIGHT;

		params->dsi.horizontal_sync_active				= 10;
		params->dsi.horizontal_backporch				= 100;
		params->dsi.horizontal_frontporch				= 100;
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

		// Bit rate calculation
		//1 Every lane speed
/*
		params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		params->dsi.pll_div2=0;		// div2=0,1,2,3;div1_real=1,2,4,4
		params->dsi.fbk_div =15;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)
*/
     params->dsi.PLL_CLOCK =450;

}

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(20);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(150);
    init_lcm_registers();
}

static void lcm_suspend(void)
{
	unsigned int data_array[16];
	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
        MDELAY(120);

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
        MDELAY(120);

	data_array[0] = 0x00022902;
	data_array[1] = 0x000004b0;
	dsi_set_cmdq(data_array, 2, 1);
	
	data_array[0] = 0x00000500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00000500;
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00022902;
	data_array[1] = 0x000001b1;
	dsi_set_cmdq(data_array, 2, 1);	
	MDELAY(30);	

        /* mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00); */
        /* mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT); */
        mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ZERO);
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

#if 1
static unsigned int lcm_compare_id(void)
{
    unsigned int id = 0;
    unsigned char buffer[4];
    unsigned int array[16];

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);

    array[0]=0x00022902;//Panel Interface Control (Type B)
    array[1]=0x000004b0;//cmd mode
    dsi_set_cmdq(array, 2, 1);

    array[0] = 0x00053700;// read id return two byte,version and id
    dsi_set_cmdq(array, 1, 1);

    read_reg_v2(0xBF, buffer, 4);
    id = (buffer[2] << 8) | buffer[3] ;//we only need ID

    if(id == LCM_ID_RM63315)
        return 1;
    else
        return 0;
}
#endif

LCM_DRIVER r63315_fhd_lg50_truly_lcm_drv = {
/* LCM_DRIVER r63315_lg55_hrb_fhd_lcm_drv = */
/* { */
    .name			= "r63315_fhd_lg50_truly",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	/* .compare_id     = lcm_compare_id, */
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    .init_power        = lcm_init_power,
    .resume_power = lcm_resume_power,
    .suspend_power = lcm_suspend_power,

    };

