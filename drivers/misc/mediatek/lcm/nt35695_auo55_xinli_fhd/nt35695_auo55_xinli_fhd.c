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

#define LCM_ID_NT35695 (0xf5)
#define GPIO_LCD_RST_EN      (GPIO28 | 0x80000000)
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



static void lcm_init_power(void)
{
}


static void lcm_suspend_power(void)
{
}

static void lcm_resume_power(void)
{
}

static void lcd_power_en(unsigned char enabled)
{
}

static void init_lcm_registers(void)
{
}
struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};
//update initial param for IC nt35520 0.01
static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28,0,{}},
	{0x10,0,{}},
	{REGFLAG_DELAY, 120, {}}
};
	
//update initial param for IC nt35520 0.01
static struct LCM_setting_table lcm_initialization_setting[] = {
   
	{0xFF,1,{0x24}},
	{0xFB,1,{0x01}},					
	{0xC5,1,{0x30}},
	{REGFLAG_DELAY, 20, {}},
	{0xC7,1,{0x02}},	
		
	{0xFF,1,{0x20}},						
	{0xFB,1,{0x01}},						
	{0x00,1,{0x01}},						
	{0x01,1,{0x55}},						
	{0x02,1,{0x45}},						
	{0x03,1,{0x55}},						
	{0x05,1,{0x50}},						
	{0x06,1,{0xA8}},						
	{0x07,1,{0xAD}},						
	{0x08,1,{0x0C}},						
	{0x0B,1,{0xAA}},						
	{0x0C,1,{0xAA}},						
	{0x0E,1,{0xB0}},						
	{0x0F,1,{0xB3}},						
	{0x11,1,{0x28}},						
	{0x12,1,{0x10}},						
	{0x13,1,{0x01}},						
	{0x14,1,{0x4A}},						
	{0x15,1,{0x12}},						
	{0x16,1,{0x12}},						
	{0x30,1,{0x01}},						
	{0x72,1,{0x31}},						
	{0x58,1,{0x82}},						
	{0x59,1,{0x00}},						
	{0x5A,1,{0x02}},						
	{0x5B,1,{0x00}},						
	{0x5C,1,{0x82}},						
	{0x5D,1,{0x80}},						
	{0x5E,1,{0x02}},						
	{0x5F,1,{0x00}},						
	{0xFF,1,{0x24}},						
	{0xFB,1,{0x01}},						
	{0x00,1,{0x01}},						
	{0x01,1,{0x0B}},						
	{0x02,1,{0x0C}},						
	{0x03,1,{0x89}},						
	{0x04,1,{0x8A}},						
	{0x05,1,{0x0F}},						
	{0x06,1,{0x10}},						
	{0x07,1,{0x10}},						
	{0x08,1,{0x1C}},						
	{0x09,1,{0x00}},						
	{0x0A,1,{0x00}},						
	{0x0B,1,{0x00}},						
	{0x0C,1,{0x00}},						
	{0x0D,1,{0x13}},						
	{0x0E,1,{0x15}},						
	{0x0F,1,{0x17}},						
	{0x10,1,{0x01}},						
	{0x11,1,{0x0B}},						
	{0x12,1,{0x0C}},						
	{0x13,1,{0x89}},						
	{0x14,1,{0x8A}},						
	{0x15,1,{0x0F}},						
	{0x16,1,{0x10}},						
	{0x17,1,{0x10}},						
	{0x18,1,{0x1C}},						
	{0x19,1,{0x00}},						
	{0x1A,1,{0x00}},						
	{0x1B,1,{0x00}},						
	{0x1C,1,{0x00}},						
	{0x1D,1,{0x13}},						
	{0x1E,1,{0x15}},						
	{0x1F,1,{0x17}},						
	{0x20,1,{0x00}},						
	{0x21,1,{0x01}},						
	{0x22,1,{0x00}},						
	{0x23,1,{0x40}},						
	{0x24,1,{0x40}},						
	{0x25,1,{0x6D}},						
	{0x26,1,{0x40}},						
	{0x27,1,{0x40}},						
	{0x29,1,{0xD8}},						
	{0x2A,1,{0x2A}},						
	{0x4B,1,{0x03}},						
	{0x4C,1,{0x11}},						
	{0x4D,1,{0x10}},						
	{0x4E,1,{0x01}},						
	{0x4F,1,{0x01}},						
	{0x50,1,{0x10}},						
	{0x51,1,{0x00}},						
	{0x52,1,{0x80}},						
	{0x53,1,{0x00}},						
	{0x54,1,{0x07}},						
	{0x55,1,{0x25}},						
	{0x56,1,{0x00}},						
	{0x58,1,{0x07}},						
	{0x5B,1,{0x43}},						
	{0x5C,1,{0x00}},						
	{0x5F,1,{0x73}},						
	{0x60,1,{0x73}},						
	{0x63,1,{0x22}},						
	{0x64,1,{0x00}},						
	{0x67,1,{0x08}},						
	{0x68,1,{0x04}},						
	{0x7A,1,{0x80}},						
	{0x7B,1,{0x91}},						
	{0x7C,1,{0xD8}},						
	{0x7D,1,{0x60}},						
	{0x93,1,{0x06}},	//	Page	0,1,{	power-related	setting	
  {0x94,1,{0x06}},
  {0x8A,1,{0x00}},
  {0x9B,1,{0x0F}},
  {0xB3,1,{0xC0}},
  {0xB4,1,{0x00}},
  {0xB5,1,{0x00}},
  {0xB6,1,{0x21}},
  {0xB7,1,{0x22}},
  {0xB8,1,{0x07}},
  {0xB9,1,{0x07}},
  {0xBA,1,{0x22}},
  {0xBD,1,{0x20}},
  {0xBE,1,{0x07}},
  {0xBF,1,{0x07}},
  {0xC1,1,{0x6D}},
  {0xE3,1,{0x00}},
  {0xEC,1,{0x00}},
  {0xFF,1,{0x10}},
 
  {0xBB ,1,{0x03 }},

	{0xFF,1,{0x20}},						
	{0xFB,1,{0x01}},						
	{0x75,1,{0x00}},						
	{0x76,1,{0x7C}},						
	{0x77,1,{0x00}},						
	{0x78,1,{0x93}},						
	{0x79,1,{0x00}},						
	{0x7A,1,{0xB6}},						
	{0x7B,1,{0x00}},						
	{0x7C,1,{0xD0}},						
	{0x7D,1,{0x00}},						
	{0x7E,1,{0xE4}},						
	{0x7F,1,{0x00}},						
	{0x80,1,{0xF6}},						
	{0x81,1,{0x01}},						
	{0x82,1,{0x06}},						
	{0x83,1,{0x01}},						
	{0x84,1,{0x14}},						
	{0x85,1,{0x01}},						
	{0x86,1,{0x21}},						
	{0x87,1,{0x01}},						
	{0x88,1,{0x4D}},						
	{0x89,1,{0x01}},						
	{0x8A,1,{0x70}},						
	{0x8B,1,{0x01}},						
	{0x8C,1,{0xA6}},						
	{0x8D,1,{0x01}},						
	{0x8E,1,{0xD0}},						
	{0x8F,1,{0x02}},						
	{0x90,1,{0x12}},						
	{0x91,1,{0x02}},	//	Page	0,1,{	power-related	setting				
	{0x92,1,{0x47}},						
	{0x93,1,{0x02}},						
	{0x94,1,{0x48}},						
	{0x95,1,{0x02}},						
	{0x96,1,{0x77}},						
	{0x97,1,{0x02}},						
	{0x98,1,{0xAB}},						
	{0x99,1,{0x02}},						
	{0x9A,1,{0xCC}},						
	{0x9B,1,{0x02}},						
	{0x9C,1,{0xF2}},						
	{0x9D,1,{0x03}},						
	{0x9E,1,{0x10}},						
	{0x9F,1,{0x03}},						
	{0xA0,1,{0x3E}},						
	{0xA2,1,{0x03}},						
	{0xA3,1,{0x4A}},						
	{0xA4,1,{0x03}},						
	{0xA5,1,{0x57}},						
	{0xA6,1,{0x03}},						
	{0xA7,1,{0x68}},						
	{0XA9,1,{0x03}},						
	{0xAA,1,{0x78}},						
	{0xAB,1,{0x03}},						
	{0xAC,1,{0x8A}},						
	{0xAD,1,{0x03}},						
	{0xAE,1,{0x9B}},						
	{0xAF,1,{0x03}},						
	{0xB0,1,{0xA5}},						
	{0xB1,1,{0x03}},						
	{0xB2,1,{0xA6}},						
	{0xB3,1,{0x00}},						
	{0xB4,1,{0x7C}},						
	{0xB5,1,{0x00}},						
	{0xB6,1,{0x93}},						
	{0xB7,1,{0x00}},						
	{0xB8,1,{0xB6}},						
	{0xB9,1,{0x00}},						
	{0xBA,1,{0xD0}},						
	{0xBB,1,{0x00}},						
	{0xBC,1,{0xE4}},						
	{0xBD,1,{0x00}},						
	{0xBE,1,{0xF6}},						
	{0xBF,1,{0x01}},						
	{0xC0,1,{0x06}},						
	{0xC1,1,{0x01}},						
	{0xC2,1,{0x14}},						
	{0xC3,1,{0x01}},						
	{0xC4,1,{0x21}},						
	{0xC5,1,{0x01}},						
	{0xC6,1,{0x4D}},						
	{0xC7,1,{0x01}},						
	{0xC8,1,{0x70}},						
	{0xC9,1,{0x01}},						
	{0xCA,1,{0xA6}},						
	{0xCB,1,{0x01}},						
	{0xCC,1,{0xD0}},						
	{0xCD,1,{0x02}},						
	{0xCE,1,{0x12}},						
	{0xCF,1,{0x02}},											
	{0xD0,1,{0x47}},						
	{0xD1,1,{0x02}},						
	{0xD2,1,{0x48}},						
	{0xD3,1,{0x02}},						
	{0xD4,1,{0x77}},						
	{0xD5,1,{0x02}},						
	{0xD6,1,{0xAB}},						
	{0xD7,1,{0x02}},						
	{0xD8,1,{0xCC}},						
	{0xD9,1,{0x02}},						
	{0xDA,1,{0xF2}},						
	{0xDB,1,{0x03}},						
	{0xDC,1,{0x10}},						
	{0xDD,1,{0x03}},						
	{0xDE,1,{0x3E}},						
	{0xDF,1,{0x03}},						
	{0xE0,1,{0x4A}},						
	{0xE1,1,{0x03}},						
	{0xE2,1,{0x57}},						
	{0xE3,1,{0x03}},						
	{0xE4,1,{0x68}},						
	{0xE5,1,{0x03}},						
	{0xE6,1,{0x78}},						
	{0xE7,1,{0x03}},						
	{0xE8,1,{0x8A}},						
	{0xE9,1,{0x03}},						
	{0xEA,1,{0x9B}},						
	{0xEB,1,{0x03}},						
	{0xEC,1,{0xA5}},						
	{0xED,1,{0x03}},						
	{0xEE,1,{0xA6}},						
	{0xEF,1,{0x00}},						
	{0xF0,1,{0x7C}},						
	{0xF1,1,{0x00}},						
	{0xF2,1,{0x93}},						
	{0xF3,1,{0x00}},						
	{0xF4,1,{0xB6}},						
	{0xF5,1,{0x00}},						
	{0xF6,1,{0xD0}},						
	{0xF7,1,{0x00}},						
	{0xF8,1,{0xE4}},						
	{0xF9,1,{0x00}},						
	{0xFA,1,{0xF6}},						
	{0xFF,1,{0x21}},						
	{0xFB,1,{0x01}},						
	{0x00,1,{0x01}},						
	{0x01,1,{0x06}},						
	{0x02,1,{0x01}},						
	{0x03,1,{0x14}},						
	{0x04,1,{0x01}},						
	{0x05,1,{0x21}},						
	{0x06,1,{0x01}},						
	{0x07,1,{0x4D}},						
	{0x08,1,{0x01}},						
	{0x09,1,{0x70}},						
	{0x0A,1,{0x01}},						
	{0x0B,1,{0xA6}},						
	{0x0C,1,{0x01}},						
	{0x0D,1,{0xD0}},						
	{0x0E,1,{0x02}},											
	{0x0F,1,{0x12}},						
	{0x10,1,{0x02}},						
	{0x11,1,{0x47}},						
	{0x12,1,{0x02}},						
	{0x13,1,{0x48}},						
	{0x14,1,{0x02}},						
	{0x15,1,{0x77}},						
	{0x16,1,{0x02}},						
	{0x17,1,{0xAB}},						
	{0x18,1,{0x02}},						
	{0x19,1,{0xCC}},						
	{0x1A,1,{0x02}},						
	{0x1B,1,{0xF2}},				
	{0x1C,1,{0x03}},						
	{0x1D,1,{0x10}},						
	{0x1E,1,{0x03}},						
	{0x1F,1,{0x3E}},						
	{0x20,1,{0x03}},						
	{0x21,1,{0x4A}},						
	{0x22,1,{0x03}},						
	{0x23,1,{0x57}},						
	{0x24,1,{0x03}},						
	{0x25,1,{0x68}},						
	{0x26,1,{0x03}},						
	{0x27,1,{0x78}},						
	{0x28,1,{0x03}},						
	{0x29,1,{0x8A}},						
	{0x2A,1,{0x03}},						
	{0x2B,1,{0x9B}},						
	{0x2D,1,{0x03}},						
	{0x2F,1,{0xA5}},						
	{0x30,1,{0x03}},						
	{0x31,1,{0xA6}},						
	{0x32,1,{0x00}},						
	{0x33,1,{0x7C}},						
	{0x34,1,{0x00}},						
	{0x35,1,{0x93}},						
	{0x36,1,{0x00}},						
	{0x37,1,{0xB6}},						
	{0x38,1,{0x00}},						
	{0x39,1,{0xD0}},						
	{0x3A,1,{0x00}},						
	{0x3B,1,{0xE4}},						
	{0x3D,1,{0x00}},						
	{0x3F,1,{0xF6}},						
	{0x40,1,{0x01}},						
	{0x41,1,{0x06}},						
	{0x42,1,{0x01}},						
	{0x43,1,{0x14}},						
	{0x44,1,{0x01}},						
	{0x45,1,{0x21}},						
	{0x46,1,{0x01}},						
	{0x47,1,{0x4D}},						
	{0x48,1,{0x01}},						
	{0x49,1,{0x70}},						
	{0x4A,1,{0x01}},						
	{0x4B,1,{0xA6}},						
	{0x4C,1,{0x01}},						
	{0x4D,1,{0xD0}},						
	{0x4E,1,{0x02}},						
	{0x4F,1,{0x12}},											
	{0x50,1,{0x02}},						
	{0x51,1,{0x47}},						
	{0x52,1,{0x02}},						
	{0x53,1,{0x48}},						
	{0x54,1,{0x02}},						
	{0x55,1,{0x77}},						
	{0x56,1,{0x02}},						
	{0x58,1,{0xAB}},						
	{0x59,1,{0x02}},						
	{0x5A,1,{0xCC}},						
	{0x5B,1,{0x02}},						
	{0x5C,1,{0xF2}},						
	{0x5D,1,{0x03}},						
	{0x5E,1,{0x10}},						
	{0x5F,1,{0x03}},						
	{0x60,1,{0x3E}},						
	{0x61,1,{0x03}},						
	{0x62,1,{0x4A}},						
	{0x63,1,{0x03}},						
	{0x64,1,{0x57}},						
	{0x65,1,{0x03}},						
	{0x66,1,{0x68}},						
	{0x67,1,{0x03}},						
	{0x68,1,{0x78}},						
	{0x69,1,{0x03}},						
	{0x6A,1,{0x8A}},						
	{0x6B,1,{0x03}},						
	{0x6C,1,{0x9B}},						
	{0x6D,1,{0x03}},						
	{0x6E,1,{0xA5}},						
	{0x6F,1,{0x03}},						
	{0x70,1,{0xA6}},						
	{0x71,1,{0x00}},						
	{0x72,1,{0x7C}},						
	{0x73,1,{0x00}},						
	{0x74,1,{0x93}},						
	{0x75,1,{0x00}},						
	{0x76,1,{0xB6}},						
	{0x77,1,{0x00}},						
	{0x78,1,{0xD0}},						
	{0x79,1,{0x00}},						
	{0x7A,1,{0xE4}},						
	{0x7B,1,{0x00}},						
	{0x7C,1,{0xF6}},						
	{0x7D,1,{0x01}},						
	{0x7E,1,{0x06}},						
	{0x7F,1,{0x01}},						
	{0x80,1,{0x14}},						
	{0x81,1,{0x01}},						
	{0x82,1,{0x21}},						
	{0x83,1,{0x01}},						
	{0x84,1,{0x4D}},						
	{0x85,1,{0x01}},						
	{0x86,1,{0x70}},						
	{0x87,1,{0x01}},						
	{0x88,1,{0xA6}},						
	{0x89,1,{0x01}},						
	{0x8A,1,{0xD0}},						
	{0x8B,1,{0x02}},						
	{0x8C,1,{0x12}},											
	{0x8D,1,{0x02}},						
	{0x8E,1,{0x47}},						
	{0x8F,1,{0x02}},						
	{0x90,1,{0x48}},						
	{0x91,1,{0x02}},						
	{0x92,1,{0x77}},						
	{0x93,1,{0x02}},						
	{0x94,1,{0xAB}},						
	{0x95,1,{0x02}},						
	{0x96,1,{0xCC}},						
	{0x97,1,{0x02}},						
	{0x98,1,{0xF2}},						
	{0x99,1,{0x03}},						
	{0x9A,1,{0x10}},						
	{0x9B,1,{0x03}},						
	{0x9C,1,{0x3E}},						
	{0x9D,1,{0x03}},						
	{0x9E,1,{0x4A}},						
	{0x9F,1,{0x03}},						
	{0xA0,1,{0x57}},						
	{0xA2,1,{0x03}},						
	{0xA3,1,{0x68}},						
	{0xA4,1,{0x03}},						
	{0xA5,1,{0x78}},						
	{0xA6,1,{0x03}},						
	{0xA7,1,{0x8A}},						
	{0XA9,1,{0x03}},						
	{0xAA,1,{0x9B}},						
	{0xAB,1,{0x03}},						
	{0xAC,1,{0xA5}},						
	{0xAD,1,{0x03}},						
	{0xAE,1,{0xA6}},						
	{0xAF,1,{0x00}},						
	{0xB0,1,{0x7C}},						
	{0xB1,1,{0x00}},						
	{0xB2,1,{0x93}},						
	{0xB3,1,{0x00}},						
	{0xB4,1,{0xB6}},						
	{0xB5,1,{0x00}},						
	{0xB6,1,{0xD0}},						
	{0xB7,1,{0x00}},						
	{0xB8,1,{0xE4}},						
	{0xB9,1,{0x00}},						
	{0xBA,1,{0xF6}},						
	{0xBB,1,{0x01}},						
	{0xBC,1,{0x06}},						
	{0xBD,1,{0x01}},						
	{0xBE,1,{0x14}},						
	{0xBF,1,{0x01}},						
	{0xC0,1,{0x21}},						
	{0xC1,1,{0x01}},						
	{0xC2,1,{0x4D}},						
	{0xC3,1,{0x01}},						
	{0xC4,1,{0x70}},						
	{0xC5,1,{0x01}},						
	{0xC6,1,{0xA6}},						
	{0xC7,1,{0x01}},						
	{0xC8,1,{0xD0}},						
	{0xC9,1,{0x02}},						
	{0xCA,1,{0x12}},											
	{0xCB,1,{0x02}},						
	{0xCC,1,{0x47}},						
	{0xCD,1,{0x02}},						
	{0xCE,1,{0x48}},						
	{0xCF,1,{0x02}},						
	{0xD0,1,{0x77}},						
	{0xD1,1,{0x02}},						
	{0xD2,1,{0xAB}},						
	{0xD3,1,{0x02}},						
	{0xD4,1,{0xCC}},						
	{0xD5,1,{0x02}},						
	{0xD6,1,{0xF2}},						
	{0xD7,1,{0x03}},						
	{0xD8,1,{0x10}},						
	{0xD9,1,{0x03}},						
	{0xDA,1,{0x3E}},						
	{0xDB,1,{0x03}},						
	{0xDC,1,{0x4A}},						
	{0xDD,1,{0x03}},						
	{0xDE,1,{0x57}},						
	{0xDF,1,{0x03}},						
	{0xE0,1,{0x68}},						
	{0xE1,1,{0x03}},						
	{0xE2,1,{0x78}},						
	{0xE3,1,{0x03}},						
	{0xE4,1,{0x8A}},						
	{0xE5,1,{0x03}},						
	{0xE6,1,{0x9B}},						
	{0xE7,1,{0x03}},						
	{0xE8,1,{0xA5}},						
	{0xE9,1,{0x03}},						
	{0xEA,1,{0xA6}},						
	{0xFF,1,{0x10}},	
	
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	{0x11,0,{0}},	//	Return	To	CMD1		
	
	{REGFLAG_DELAY, 120, {}},												
	{0x29,0,{}},
  {REGFLAG_DELAY, 120, {}},		
	//{0x51,1,{0xFF}},	//	write	display	brightness		
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
                if(table[i].count <= 10)
                    MDELAY(table[i].count);
                else
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
		params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
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
//		params->dsi.word_count=720*3;


		params->dsi.vertical_sync_active				= 2;
		params->dsi.vertical_backporch					= 2;
		params->dsi.vertical_frontporch					= 10;
		params->dsi.vertical_active_line				= FRAME_HEIGHT;

		params->dsi.horizontal_sync_active				= 20;
		params->dsi.horizontal_backporch				= 60;
		params->dsi.horizontal_frontporch				= 90;
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
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);

push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1); 
}

static void lcm_suspend(void)
{
	unsigned int data_array[16];
	//unsigned char buffer[2];

#if 0//ndef BUILD_LK
	data_array[0] = 0x00013700;// read id return two byte,version and id
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0xFE, buffer, 1);
	printk("%s, kernel nt35596 horse debug: nt35596 id = 0x%08x\n", __func__, buffer[0]);
#endif

	data_array[0]=0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
        MDELAY(120);

	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
        MDELAY(120);

    SET_RESET_PIN(1);
        MDELAY(50);
    SET_RESET_PIN(0);
        MDELAY(50);
}


static void lcm_resume(void)
{
	//unsigned int data_array[16];
	//unsigned char buffer[2];

	lcm_init();

#if 0//ndef BUILD_LK
	data_array[0] = 0x00013700;// read id return two byte,version and id
	dsi_set_cmdq(data_array, 1, 1);

	read_reg_v2(0xFE, buffer, 1);
	printk("%s, kernel nt35596 horse debug: nt35596 id = 0x%08x\n", __func__, buffer[0]);
#endif

	//TC358768_DCS_write_1A_0P(0x11); // Sleep Out
	//MDELAY(150);

	//TC358768_DCS_write_1A_0P(0x29); // Display On

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
	unsigned int id=0;
	unsigned char buffer[2];
	unsigned int array[16];

    SET_RESET_PIN(1);
    MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);
    
    array[0] = 0x00023902;
    array[1] = 0x000020FF;
    dsi_set_cmdq(array, 2, 1);
    MDELAY(10);
	  
	  array[0] = 0x00023700;// read id return two byte,version and id
	  dsi_set_cmdq(array, 1, 1);
    
	  read_reg_v2(0x3b, buffer, 2);
	id 	= buffer[0]; //we only need ID
#ifdef BUILD_LK
		printf("%s, LK nt35695 debug: nt35695 id = 0x%08x\n", __func__, id);
#else
		printk("%s, kernel nt35695 horse debug: nt35695 id = 0x%08x\n", __func__, id);
#endif

    if(id == LCM_ID_NT35695)
    	return 1;
    else
        return 0;
}
#endif

LCM_DRIVER nt35695_auo55_xinli_fhd_lcm_drv =
{
    .name			= "nt35695_auo55_xinli_fhd",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    .init_power        = lcm_init_power,
    .resume_power = lcm_resume_power,
    .suspend_power = lcm_suspend_power,

    };
