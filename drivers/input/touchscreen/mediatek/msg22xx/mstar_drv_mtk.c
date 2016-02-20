////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2014 MStar Semiconductor, Inc.
// All rights reserved.
//
// Unless otherwise stipulated in writing, any and all information contained
// herein regardless in any format shall remain the sole proprietary of
// MStar Semiconductor Inc. and be kept in strict confidence
// (??MStar Confidential Information??) by the recipient.
// Any unauthorized act including without limitation unauthorized disclosure,
// copying, use, reproduction, sale, distribution, modification, disassembling,
// reverse engineering and compiling of the contents of MStar Confidential
// Information is unlawful and strictly prohibited. MStar hereby reserves the
// rights to any and all damages, losses, costs and expenses resulting therefrom.
//
////////////////////////////////////////////////////////////////////////////////

/**
 *
 * @file    mstar_drv_mtk.c
 *
 * @brief   This file defines the interface of touch screen
 *
 *
 */

/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/hwmsen_helper.h>
//#include <linux/hw_module_info.h>

#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/namei.h>
#include <linux/vmalloc.h>

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <mach/mt_gpio.h>

#include <cust_eint.h>

#include "tpd.h"
#include "cust_gpio_usage.h"

#include "mstar_drv_platform_interface.h"

/*=============================================================*/
// CONSTANT VALUE DEFINITION
/*=============================================================*/

#define MSG_TP_IC_NAME "msg2xxx" //"msg21xxA" or "msg22xx" or "msg26xxM" /* Please define the mstar touch ic name based on the mutual-capacitive ic or self capacitive ic that you are using */
#define I2C_BUS_ID   (1)       // i2c bus id : 0 or 1

#define TPD_OK (0)

/*=============================================================*/
// EXTERN VARIABLE DECLARATION
/*=============================================================*/

#ifdef CONFIG_TP_HAVE_KEY
extern const int g_TpVirtualKey[];

#ifdef CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE
extern const int g_TpVirtualKeyDimLocal[][4];
#endif //CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE
#endif //CONFIG_TP_HAVE_KEY

extern struct tpd_device *tpd;

/*=============================================================*/
// LOCAL VARIABLE DEFINITION
/*=============================================================*/

/*
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT] = TPD_WARP_END;
#endif

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8] = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif
*/
struct i2c_client *g_I2cClient = NULL;

//static int boot_mode = 0;

/*=============================================================*/
// FUNCTION DECLARATION
/*=============================================================*/

/*=============================================================*/
// FUNCTION DEFINITION
/*=============================================================*/

/* probe function is used for matching and initializing input device */
static int /*__devinit*/ tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    TPD_DMESG("TPD probe\n");   
    
    if (client == NULL)
    {
        TPD_DMESG("i2c client is NULL\n");
        return -1;
    }
    g_I2cClient = client;
    
    MsDrvInterfaceTouchDeviceSetIicDataRate(g_I2cClient, 100000); // 100 KHZ

    if(MsDrvInterfaceTouchDeviceProbe(g_I2cClient, id)<0)
      tpd_load_status = 0;
    else
      tpd_load_status = 1;

    TPD_DMESG("TPD probe done\n");
    
    return TPD_OK;   
}

static int tpd_detect(struct i2c_client *client, struct i2c_board_info *info) 
{
    strcpy(info->type, TPD_DEVICE);    
//    strcpy(info->type, MSG_TP_IC_NAME);
    
    return TPD_OK;
}

static int /*__devexit*/ tpd_remove(struct i2c_client *client)
{   
    TPD_DEBUG("TPD removed\n");
    
    MsDrvInterfaceTouchDeviceRemove(client);
    
    return TPD_OK;
}

static struct i2c_board_info __initdata i2c_tpd = {I2C_BOARD_INFO(MSG_TP_IC_NAME, (0x4C>>1))};

/* The I2C device list is used for matching I2C device and I2C device driver. */
static const struct i2c_device_id tpd_device_id[] =
{
    {MSG_TP_IC_NAME, 0},
    {}, /* should not omitted */ 
};

MODULE_DEVICE_TABLE(i2c, tpd_device_id);

static struct i2c_driver tpd_i2c_driver = {
    .driver = {
        .name = MSG_TP_IC_NAME,
    },
    .probe = tpd_probe,
    //.remove = __devexit_p(tpd_remove),
    .remove = tpd_remove,
    .id_table = tpd_device_id,
    .detect = tpd_detect,
};

static int tpd_local_init(void)
{  
    TPD_DMESG("TPD init device driver (Built %s @ %s)\n", __DATE__, __TIME__);
/*
    // Software reset mode will be treated as normal boot
    boot_mode = get_boot_mode();
    if (boot_mode == 3) 
    {
        boot_mode = NORMAL_BOOT;    
    }
*/
    if (i2c_add_driver(&tpd_i2c_driver) != 0)
    {
        TPD_DMESG("unable to add i2c driver.\n");
         
        return -1;
    }
    
    if (tpd_load_status == 0) 
    {
        TPD_DMESG("add error touch panel driver.\n");

        i2c_del_driver(&tpd_i2c_driver);
        return -1;
    }

#ifdef CONFIG_TP_HAVE_KEY
#ifdef CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE     
    // initialize tpd button data
    tpd_button_setting(4, g_TpVirtualKey, g_TpVirtualKeyDimLocal); //MAX_KEY_NUM
#endif //CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE  
#endif //CONFIG_TP_HAVE_KEY  

/*
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);    
#endif  
*/
    TPD_DMESG("TPD init done %s, %d\n", __FUNCTION__, __LINE__);  
        
    return TPD_OK; 
}

static void tpd_resume(struct early_suspend *h)
{
    TPD_DMESG("TPD wake up\n");
    
    MsDrvInterfaceTouchDeviceResume(h);
    
    TPD_DMESG("TPD wake up done\n");
}

static void tpd_suspend(struct early_suspend *h)
{
    TPD_DMESG("TPD enter sleep\n");

    MsDrvInterfaceTouchDeviceSuspend(h);

    TPD_DMESG("TPD enter sleep done\n");
} 

static struct tpd_driver_t tpd_device_driver = {
     .tpd_device_name = MSG_TP_IC_NAME,
     .tpd_local_init = tpd_local_init,
     .suspend = tpd_suspend,
     .resume = tpd_resume,
#ifdef CONFIG_TP_HAVE_KEY
#ifdef CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE
     .tpd_have_button = 1,
#else
     .tpd_have_button = 0,
#endif //CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE        
#endif //CONFIG_TP_HAVE_KEY        
};

static int __init tpd_driver_init(void) 
{
    TPD_DMESG("touch panel driver init\n");

    i2c_register_board_info(I2C_BUS_ID, &i2c_tpd, 1);
    if (tpd_driver_add(&tpd_device_driver) < 0)
    {
        TPD_DMESG("TPD add driver failed\n");
    }
     
    return 0;
}
 
static void __exit tpd_driver_exit(void) 
{
    TPD_DMESG("touch panel driver exit\n");
    
    tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);
MODULE_LICENSE("GPL");
