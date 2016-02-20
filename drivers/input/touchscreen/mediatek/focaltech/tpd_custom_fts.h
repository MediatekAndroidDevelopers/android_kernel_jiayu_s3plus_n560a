#ifndef TOUCHPANEL_H__
#define TOUCHPANEL_H__

#include <linux/hrtimer.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
//#include <linux/io.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/rtpm_prio.h>

#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

#include <cust_eint.h>
#include <linux/jiffies.h>

struct Upgrade_Info {
        u8 CHIP_ID;
        u8 FTS_NAME[20];
        u8 TPD_MAX_POINTS;
        u8 AUTO_CLB;
	u16 delay_aa;		/*delay of write FT_UPGRADE_AA */
	u16 delay_55;		/*delay of write FT_UPGRADE_55 */
	u8 upgrade_id_1;	/*upgrade id 1 */
	u8 upgrade_id_2;	/*upgrade id 2 */
	u16 delay_readid;	/*delay of read id */
	u16 delay_earse_flash; /*delay of earse flash*/
};
//extern Upgrade_Info fts_updateinfo_curr;

/**********************Custom define begin**********************************************/

#if defined(CONFIG_ARCH_MT6582)
#define TPD_POWER_SOURCE_CUSTOM MT6323_POWER_LDO_VGP1
#define TPD_POWER_SOURCE_1800 MT6323_POWER_LDO_VGP3
#elif defined(CONFIG_ARCH_MT6592)
#define TPD_POWER_SOURCE_1800 MT6323_POWER_LDO_VGP3
#endif
#define IIC_PORT 				1
//#define TPD_PROXIMITY					// if need the PS funtion,enable this MACRO

/*
///// ***** virtual key  definition  ***** /////

Below are the recommend  virtual key definition for different resolution TPM. 

HVGA  320x480    2key ( (80,530);(240,530) )           3key  ( (80,530);(160;530);(240,530) )          4key   ( (40,530);(120;530);(200,530);(280,530)  ) 
WVGA  480x800   2key ( (80,900);(400,900) )           3key  ( (80,900);(240,900);(400,900) )          4key   ( (60,900);(180;900);(300,900);(420,900)  ) 
FWVGA 480x854  2key ( (80,900);(400,900) )           3key  ( (80,900);(240,900);(400,900) )          4key   ( (60,900);(180;900);(300,900);(420,900)  ) 
QHD  540x960     2key ( (90,1080);(450,1080) )           3key  ( (90,1080);(270,1080);(450,1080) )          4key   ( (90,1080);(180;1080);(360,1080);(450,1080)  ) 
HD    1280x720    2key ( (120,1350);(600,1350) )           3key  ( (120,1350);(360,1350);(600,1350) )          4key   ( (120,1080);(240;1080);(480,1080);(600,1080)  )
FHD   1920x1080  2key ( (160,2100);(920,2100) )           3key  ( (160,2100);(540,2100);(920,2100) )          4key   ( (160,2100);(320;1080);(600,1080);(920,2100)  )
*/

#define TPD_HAVE_BUTTON	// if have virtual key,need define the MACRO
#define TPD_BUTTON_HEIGH        (40)  //100
/*
#define TPD_KEY_COUNT           3    //  4
#define TPD_KEYS                {KEY_MENU,KEY_HOME,KEY_BACK}
#define TPD_KEYS2                {KEY_MENU,KEY_HOMEPAGE,KEY_BACK}
*/
#if (defined(WVGA) || defined(CU_WVGA) || defined(CMCC_WVGA) || defined(CMCC_LTE_WVGA))

#define TPD_KEY_COUNT           4
#define TPD_KEYS                {KEY_HOMEPAGE, KEY_MENU, KEY_BACK, KEY_SEARCH}
#define TPD_KEYS_DIM            {{60,870,60,50},{180,870,60,50},{300,870,60,50},{420,870,60,50}}
#define TPD_WIDTH				480
#define TPD_HEIGHT				800

#elif (defined(FWVGA) || defined(CU_FWVGA) || defined(CMCC_FWVGA) || defined(CMCC_LTE_FWVGA))

#define TPD_KEY_COUNT           4
#define TPD_KEYS                {KEY_HOMEPAGE, KEY_MENU, KEY_BACK, KEY_SEARCH}
#define TPD_KEYS_DIM            {{60,920,60,50},{180,920,60,50},{300,920,60,50},{420,920,60,50}}
#define TPD_WIDTH				480
#define TPD_HEIGHT				854

#elif (defined(QHD) || defined(CU_QHD) || defined(CMCC_QHD) || defined(CMCC_LTE_QHD))

#define TPD_KEY_COUNT           4
#define TPD_KEYS                {KEY_HOMEPAGE, KEY_MENU, KEY_BACK, KEY_SEARCH}
#define TPD_KEYS_DIM            {{85,1030,60,50},{185,1030,60,50},{350,1030,60,50},{500,1030,60,50}}
#define TPD_WIDTH				540
#define TPD_HEIGHT				960

#elif (defined(HD) || defined(HD720) || defined(CU_HD720) || defined(CMCC_HD720)|| defined(CMCC_LTE_HD720))

#define TPD_KEY_COUNT           4
#define TPD_KEYS                {KEY_HOMEPAGE, KEY_MENU, KEY_BACK, KEY_SEARCH}
#define TPD_KEYS_DIM            {{90,1350,60,50},{270,1350,60,50},{430,1350,60,50},{630,1350,60,50}}
#define TPD_WIDTH				720
#define TPD_HEIGHT				1280

#elif (defined(FHD) || defined(CU_FHD) || defined(CMCC_FHD) || defined(CMCC_LTE_FHD))

#define TPD_KEY_COUNT           3
#define TPD_KEYS                {KEY_MENU,KEY_HOMEPAGE, KEY_BACK}
#define TPD_KEYS_DIM            {{200,2100,100,100},{500,2100,100,100},{800,2100,100,100}}
#define TPD_WIDTH				1080
#define TPD_HEIGHT				1920

#elif (defined(HVGA))

#define TPD_KEY_COUNT           4
#define TPD_KEYS                {KEY_HOMEPAGE, KEY_MENU, KEY_BACK, KEY_SEARCH}
#define TPD_KEYS_DIM            {{40,530,60,50},{120,530,60,50},{200,530,60,50},{280,530,60,50}}
#define TPD_WIDTH				320
#define TPD_HEIGHT				480

#elif (defined(LQHD))

#define TPD_KEY_COUNT           4
#define TPD_KEYS                {KEY_HOMEPAGE, KEY_MENU, KEY_BACK, KEY_SEARCH}
#define TPD_KEYS_DIM            {{50,1030,60,50},{185,1030,60,50},{350,1030,60,50},{500,1030,60,50}}
#define TPD_WIDTH				540
#define TPD_HEIGHT				960

#else

#define TPD_KEY_COUNT           4
#define TPD_KEYS                {KEY_HOMEPAGE, KEY_MENU, KEY_BACK, KEY_SEARCH}
#define TPD_KEYS_DIM            {{60,920,60,50},{180,920,60,50},{300,920,60,50},{420,920,60,50}}
#define TPD_WIDTH				480
#define TPD_HEIGHT				854

#endif
/*********************Custom Define end*************************************************/

#define TPD_NAME    "FT"

/* Pre-defined definition */
#define TPD_TYPE_CAPACITIVE
#define TPD_TYPE_RESISTIVE
#define TPD_POWER_SOURCE         
#define TPD_I2C_NUMBER           		0
#define TPD_WAKEUP_TRIAL         		60
#define TPD_WAKEUP_DELAY         		100

#define TPD_VELOCITY_CUSTOM_X 			15
#define TPD_VELOCITY_CUSTOM_Y 			20




#define TPD_DELAY                		(2*HZ/100)
//#define TPD_RES_X                		480
//#define TPD_RES_Y                		800
#define TPD_CALIBRATION_MATRIX  		{962,0,0,0,1600,0,0,0};

//#define TPD_HAVE_CALIBRATION
//#define TPD_HAVE_TREMBLE_ELIMINATION




/******************************************************************************/
/*Chip Device Type*/
#define IC_FT5X06						0	/*x=2,3,4*/
#define IC_FT5606						1	/*ft5506/FT5606/FT5816*/
#define IC_FT5316						2	/*ft5x16*/
#define IC_FT6208						3  	/*ft6208*/
#define IC_FT6x06     					       4	/*ft6206/FT6306*/
#define IC_FT5x06i     					5	/*ft5306i*/
#define IC_FT5x36     					       6	/*ft5336/ft5436/FT5436i*/


/*register address*/
#define FT_REG_CHIP_ID				0xA3    //chip ID 
#define FT_REG_FW_VER				0xA6   //FW  version 
#define FT_REG_VENDOR_ID			0xA8   // TP vendor ID 


#define TPD_MAX_POINTS_2                        2
#define TPD_MAX_POINTS_5                        5
#define TPD_MAXPOINTS_10                        10
#define AUTO_CLB_NEED                              1
#define AUTO_CLB_NONEED                          0

//#endif
#define TPD_SYSFS_DEBUG
/* Vanzo:yangzhihong on: Tue, 05 May 2015 14:30:47 +0800
 * #107982 
//#define FTS_CTL_IIC
 */
// End of Vanzo:yangzhihong
#define FTS_APK_DEBUG
#ifdef TPD_SYSFS_DEBUG
//#define TPD_AUTO_UPGRADE				// if need upgrade CTP FW when POWER ON,pls enable this MACRO
#endif


#define FTS_DBG
#ifdef FTS_DBG
#define DBG(fmt, args...) 				printk("[FTS]" fmt, ## args)
#else
#define DBG(fmt, args...) 				do{}while(0)
#endif

#endif /* TOUCHPANEL_H__ */
