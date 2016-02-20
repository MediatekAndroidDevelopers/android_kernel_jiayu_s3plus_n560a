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
 * @file    mstar_drv_platform_porting_layer.c
 *
 * @brief   This file defines the interface of touch screen
 *
 *
 */
 
/*=============================================================*/
// INCLUDE FILE
/*=============================================================*/

#include "mstar_drv_platform_porting_layer.h"
#include "mstar_drv_ic_fw_porting_layer.h"
#include "mstar_drv_platform_interface.h"
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
/*=============================================================*/
// EXTREN VARIABLE DECLARATION
/*=============================================================*/
#ifdef TPD_MSTAR_PROXIMITY_SENSOR
extern u8 tpd_proximity_flag;
extern u8 tpd_proximity_detect;
#endif

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
extern struct kset *g_TouchKSet;
extern struct kobject *g_TouchKObj;
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION
extern u8 g_FaceClosingTp;
#endif //CONFIG_ENABLE_PROXIMITY_DETECTION

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
extern struct tpd_device *tpd;
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
extern struct regulator *g_ReguVdd;
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON
#endif

/*=============================================================*/
// LOCAL VARIABLE DEFINITION
/*=============================================================*/

struct mutex g_Mutex;
static struct work_struct _gFingerTouchWork;

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
#ifdef CONFIG_ENABLE_NOTIFIER_FB
static struct notifier_block _gFbNotifier;
#else
static struct early_suspend _gEarlySuspend;
#endif //CONFIG_ENABLE_NOTIFIER_FB
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM || CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifndef CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM
static DECLARE_WAIT_QUEUE_HEAD(_gWaiter);
static struct task_struct *_gThread = NULL;
static int _gTpdFlag = 0;
#endif //CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM

/*=============================================================*/
// GLOBAL VARIABLE DEFINITION
/*=============================================================*/

#ifdef CONFIG_TP_HAVE_KEY
const int g_TpVirtualKey[] = {TOUCH_KEY_MENU, TOUCH_KEY_HOME, TOUCH_KEY_BACK, TOUCH_KEY_SEARCH};

#ifdef CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE
#define BUTTON_W (100)
#define BUTTON_H (100)

const int g_TpVirtualKeyDimLocal[MAX_KEY_NUM][4] = {{BUTTON_W/2*1,TOUCH_SCREEN_Y_MAX+BUTTON_H/2,BUTTON_W,BUTTON_H},{BUTTON_W/2*3,TOUCH_SCREEN_Y_MAX+BUTTON_H/2,BUTTON_W,BUTTON_H},{BUTTON_W/2*5,TOUCH_SCREEN_Y_MAX+BUTTON_H/2,BUTTON_W,BUTTON_H},{BUTTON_W/2*7,TOUCH_SCREEN_Y_MAX+BUTTON_H/2,BUTTON_W,BUTTON_H}};
#endif //CONFIG_ENABLE_REPORT_KEY_WITH_COORDINATE
#endif //CONFIG_TP_HAVE_KEY

struct input_dev *g_InputDevice = NULL;
static int _gIrq = -1;

/*=============================================================*/
// LOCAL FUNCTION DEFINITION
/*=============================================================*/

/* read data through I2C then report data to input sub-system when interrupt occurred */
static void _DrvPlatformLyrFingerTouchDoWork(struct work_struct *pWork)
{
    DBG("*** %s() ***\n", __func__);
    
    DrvIcFwLyrHandleFingerTouch(NULL, 0);

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
//    enable_irq(MS_TS_MSG_IC_GPIO_INT);
    enable_irq(_gIrq);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
#endif
}

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
/* The interrupt service routine will be triggered when interrupt occurred */
static irqreturn_t _DrvPlatformLyrFingerTouchInterruptHandler(s32 nIrq, void *pDeviceId)
{
    DBG("*** %s() ***\n", __func__);

//    disable_irq_nosync(MS_TS_MSG_IC_GPIO_INT);
    disable_irq_nosync(_gIrq);
    schedule_work(&_gFingerTouchWork);

    return IRQ_HANDLED;
}
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
static void _DrvPlatformLyrFingerTouchInterruptHandler(void)
{
#ifdef CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM
    mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
    schedule_work(&_gFingerTouchWork);
#else    
    mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

    _gTpdFlag = 1;
    wake_up_interruptible(&_gWaiter); 
#endif //CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM
}

#ifndef CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM
static int _DrvPlatformLyrFingerTouchHandler(void *pUnUsed)
{
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
    sched_setscheduler(current, SCHED_RR, &param);
	
    do
    {
        set_current_state(TASK_INTERRUPTIBLE);
        wait_event_interruptible(_gWaiter, _gTpdFlag != 0);
        _gTpdFlag = 0;
        
        set_current_state(TASK_RUNNING);

        DrvIcFwLyrHandleFingerTouch(NULL, 0);

        mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		
    } while (!kthread_should_stop());
	
    return 0;
}
#endif //CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM
#endif

/*=============================================================*/
// GLOBAL FUNCTION DEFINITION
/*=============================================================*/

#ifdef CONFIG_ENABLE_REGULATOR_POWER_ON
void DrvPlatformLyrTouchDeviceRegulatorPowerOn(void)
{
#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
    s32 nRetVal = 0;

    DBG("*** %s() ***\n", __func__);
    
    nRetVal = regulator_set_voltage(g_ReguVdd, 2800000, 2800000); // For specific SPRD BB chip(ex. SC7715) or QCOM BB chip(ex. MSM8610), need to enable this function call for correctly power on Touch IC.

    if (nRetVal)
    {
        DBG("Could not set to 2800mv.\n");
    }
    regulator_enable(g_ReguVdd);

    mdelay(20);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
//    hwPowerOn(MT6328_POWER_LDO_VGP1, VOL_2800, "TP"); // For specific MTK BB chip(ex. MT6582), need to enable this function call for correctly power on Touch IC.
//    hwPowerOn(PMIC_APP_CAP_TOUCH_VDD, VOL_2800, "TP"); // For specific MTK BB chip(ex. MT6735), need to enable this function call for correctly power on Touch IC.


// End of Vanzo:wangfei
    // power on CTP
    mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
#endif
}
#endif //CONFIG_ENABLE_REGULATOR_POWER_ON

void DrvPlatformLyrTouchDevicePowerOn(void)
{
    DBG("*** %s() ***\n", __func__);
    
#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
    gpio_direction_output(MS_TS_MSG_IC_GPIO_RST, 1);
//    gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 1); 
    udelay(100); 
    gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 0);
    udelay(100); 
    gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 1);
    mdelay(25); 
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
    mt_set_gpio_mode(MS_TS_MSG_IC_GPIO_RST, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(MS_TS_MSG_IC_GPIO_RST, GPIO_DIR_OUT);
    mt_set_gpio_out(MS_TS_MSG_IC_GPIO_RST, GPIO_OUT_ONE);  
    udelay(100); 

    mt_set_gpio_mode(MS_TS_MSG_IC_GPIO_RST, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(MS_TS_MSG_IC_GPIO_RST, GPIO_DIR_OUT);
    mt_set_gpio_out(MS_TS_MSG_IC_GPIO_RST, GPIO_OUT_ZERO);  
    udelay(100); 

#ifdef TPD_CLOSE_POWER_IN_SLEEP
    hwPowerDown(TPD_POWER_SOURCE, "TP"); 
    mdelay(100);
    hwPowerOn(TPD_POWER_SOURCE, VOL_2800, "TP"); 
    mdelay(10);  // reset pulse
#endif //TPD_CLOSE_POWER_IN_SLEEP

    mt_set_gpio_mode(MS_TS_MSG_IC_GPIO_RST, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(MS_TS_MSG_IC_GPIO_RST, GPIO_DIR_OUT);
    mt_set_gpio_out(MS_TS_MSG_IC_GPIO_RST, GPIO_OUT_ONE);
    mdelay(25); 
#endif
}

void DrvPlatformLyrTouchDevicePowerOff(void)
{
    DBG("*** %s() ***\n", __func__);
    
    DrvIcFwLyrOptimizeCurrentConsumption();
#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
//    gpio_direction_output(MS_TS_MSG_IC_GPIO_RST, 0);
    gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 0);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
    mt_set_gpio_mode(MS_TS_MSG_IC_GPIO_RST, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(MS_TS_MSG_IC_GPIO_RST, GPIO_DIR_OUT);
    mt_set_gpio_out(MS_TS_MSG_IC_GPIO_RST, GPIO_OUT_ZERO);  
#ifdef TPD_CLOSE_POWER_IN_SLEEP
    hwPowerDown(TPD_POWER_SOURCE, "TP");
#endif //TPD_CLOSE_POWER_IN_SLEEP
#endif    
}

void DrvPlatformLyrTouchDeviceResetHw(void)
{
    DBG("*** %s() ***\n", __func__);
    
#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
    gpio_direction_output(MS_TS_MSG_IC_GPIO_RST, 1);
//    gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 1); 
    gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 0);
    mdelay(100); 
    gpio_set_value(MS_TS_MSG_IC_GPIO_RST, 1);
    mdelay(100); 
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
    mt_set_gpio_mode(MS_TS_MSG_IC_GPIO_RST, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(MS_TS_MSG_IC_GPIO_RST, GPIO_DIR_OUT);
    mt_set_gpio_out(MS_TS_MSG_IC_GPIO_RST, GPIO_OUT_ONE);
    mdelay(10);
    mt_set_gpio_mode(MS_TS_MSG_IC_GPIO_RST, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(MS_TS_MSG_IC_GPIO_RST, GPIO_DIR_OUT);
    mt_set_gpio_out(MS_TS_MSG_IC_GPIO_RST, GPIO_OUT_ZERO);  
    mdelay(50);
    mt_set_gpio_mode(MS_TS_MSG_IC_GPIO_RST, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(MS_TS_MSG_IC_GPIO_RST, GPIO_DIR_OUT);
    mt_set_gpio_out(MS_TS_MSG_IC_GPIO_RST, GPIO_OUT_ONE);
    mdelay(50); 
#endif
}

void DrvPlatformLyrDisableFingerTouchReport(void)
{
    DBG("*** %s() ***\n", __func__);

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
//    disable_irq(MS_TS_MSG_IC_GPIO_RST);
    disable_irq(_gIrq);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
    mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
#endif
}

void DrvPlatformLyrEnableFingerTouchReport(void)
{
    DBG("*** %s() ***\n", __func__);

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
//    enable_irq(MS_TS_MSG_IC_GPIO_RST);
    enable_irq(_gIrq);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
#endif
}

void DrvPlatformLyrFingerTouchPressed(s32 nX, s32 nY, s32 nPressure, s32 nId)
{
    DBG("*** %s() ***\n", __func__);
    DBG("point touch pressed\n");

    input_report_key(g_InputDevice, BTN_TOUCH, 1);
#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
    input_report_abs(g_InputDevice, ABS_MT_TRACKING_ID, nId);
#endif //CONFIG_ENABLE_CHIP_MSG26XXM
    input_report_abs(g_InputDevice, ABS_MT_TOUCH_MAJOR, 1);
    input_report_abs(g_InputDevice, ABS_MT_WIDTH_MAJOR, 1);
    input_report_abs(g_InputDevice, ABS_MT_POSITION_X, nX);
    input_report_abs(g_InputDevice, ABS_MT_POSITION_Y, nY);

    input_mt_sync(g_InputDevice);

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_TP_HAVE_KEY    
    if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
    {   
        tpd_button(nX, nY, 1);  
    }
#endif //CONFIG_TP_HAVE_KEY

    TPD_EM_PRINT(nX, nY, nX, nY, nPressure-1, 1);
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
}

void DrvPlatformLyrFingerTouchReleased(s32 nX, s32 nY)
{
    DBG("*** %s() ***\n", __func__);
    DBG("point touch released\n");

    input_report_key(g_InputDevice, BTN_TOUCH, 0);
    input_mt_sync(g_InputDevice);

#ifdef CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef CONFIG_TP_HAVE_KEY 
    if (FACTORY_BOOT == get_boot_mode() || RECOVERY_BOOT == get_boot_mode())
    {   
       tpd_button(nX, nY, 0); 
//       tpd_button(0, 0, 0); 
    }            
#endif //CONFIG_TP_HAVE_KEY    

    TPD_EM_PRINT(nX, nY, nX, nY, 0, 0);
#endif //CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM
#ifdef VANZO_DEVICE_NAME_SUPPORT
{
    extern void v_set_dev_name(int id, char *name);
    v_set_dev_name(2, "msg22xx");
}
#endif
}

s32 DrvPlatformLyrInputDeviceInitialize(struct i2c_client *pClient)
{
    s32 nRetVal = 0;

    DBG("*** %s() ***\n", __func__);

    mutex_init(&g_Mutex);

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
    /* allocate an input device */
    g_InputDevice = input_allocate_device();
    if (g_InputDevice == NULL)
    {
        DBG("*** input device allocation failed ***\n");
        return -ENOMEM;
    }

    g_InputDevice->name = pClient->name;
    g_InputDevice->phys = "I2C";
    g_InputDevice->dev.parent = &pClient->dev;
    g_InputDevice->id.bustype = BUS_I2C;
    
    /* set the supported event type for input device */
    set_bit(EV_ABS, g_InputDevice->evbit);
    set_bit(EV_SYN, g_InputDevice->evbit);
    set_bit(EV_KEY, g_InputDevice->evbit);
    set_bit(BTN_TOUCH, g_InputDevice->keybit);
    set_bit(INPUT_PROP_DIRECT, g_InputDevice->propbit);

#ifdef CONFIG_TP_HAVE_KEY
    {
        u32 i;
        for (i = 0; i < MAX_KEY_NUM; i ++)
        {
            input_set_capability(g_InputDevice, EV_KEY, g_TpVirtualKey[i]);
        }
    }
#endif

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
    input_set_capability(g_InputDevice, EV_KEY, KEY_U);
    input_set_capability(g_InputDevice, EV_KEY, KEY_UP);
    input_set_capability(g_InputDevice, EV_KEY, KEY_DOWN);
    input_set_capability(g_InputDevice, EV_KEY, KEY_LEFT);
    input_set_capability(g_InputDevice, EV_KEY, KEY_RIGHT);
    input_set_capability(g_InputDevice, EV_KEY, KEY_W);
    input_set_capability(g_InputDevice, EV_KEY, KEY_Z);
    input_set_capability(g_InputDevice, EV_KEY, KEY_V);
    input_set_capability(g_InputDevice, EV_KEY, KEY_O);
    input_set_capability(g_InputDevice, EV_KEY, KEY_M);
    input_set_capability(g_InputDevice, EV_KEY, KEY_C);
    input_set_capability(g_InputDevice, EV_KEY, KEY_E);
    input_set_capability(g_InputDevice, EV_KEY, KEY_S);
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

/*
#ifdef CONFIG_TP_HAVE_KEY
    set_bit(TOUCH_KEY_MENU, g_InputDevice->keybit); //Menu
    set_bit(TOUCH_KEY_HOME, g_InputDevice->keybit); //Home
    set_bit(TOUCH_KEY_BACK, g_InputDevice->keybit); //Back
    set_bit(TOUCH_KEY_SEARCH, g_InputDevice->keybit); //Search
#endif
*/

#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
    input_set_abs_params(g_InputDevice, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
#endif //CONFIG_ENABLE_CHIP_MSG26XXM
    input_set_abs_params(g_InputDevice, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(g_InputDevice, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
    input_set_abs_params(g_InputDevice, ABS_MT_POSITION_X, TOUCH_SCREEN_X_MIN, TOUCH_SCREEN_X_MAX, 0, 0);
    input_set_abs_params(g_InputDevice, ABS_MT_POSITION_Y, TOUCH_SCREEN_Y_MIN, TOUCH_SCREEN_Y_MAX, 0, 0);

    /* register the input device to input sub-system */
    nRetVal = input_register_device(g_InputDevice);
    if (nRetVal < 0)
    {
        DBG("*** Unable to register touch input device ***\n");
    }
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
    g_InputDevice = tpd->dev;
/*
    g_InputDevice->phys = "I2C";
    g_InputDevice->dev.parent = &pClient->dev;
    g_InputDevice->id.bustype = BUS_I2C;
    
    // set the supported event type for input device 
    set_bit(EV_ABS, g_InputDevice->evbit);
    set_bit(EV_SYN, g_InputDevice->evbit);
    set_bit(EV_KEY, g_InputDevice->evbit);
    set_bit(BTN_TOUCH, g_InputDevice->keybit);
    set_bit(INPUT_PROP_DIRECT, g_InputDevice->propbit);

#ifdef CONFIG_TP_HAVE_KEY
    {
        u32 i;
        for (i = 0; i < MAX_KEY_NUM; i ++)
        {
            input_set_capability(g_InputDevice, EV_KEY, g_TpVirtualKey[i]);
        }
    }
#endif
*/
#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
    input_set_capability(g_InputDevice, EV_KEY, KEY_U);
    input_set_capability(g_InputDevice, EV_KEY, KEY_UP);
    input_set_capability(g_InputDevice, EV_KEY, KEY_DOWN);
    input_set_capability(g_InputDevice, EV_KEY, KEY_LEFT);
    input_set_capability(g_InputDevice, EV_KEY, KEY_RIGHT);
    input_set_capability(g_InputDevice, EV_KEY, KEY_W);
    input_set_capability(g_InputDevice, EV_KEY, KEY_Z);
    input_set_capability(g_InputDevice, EV_KEY, KEY_V);
    input_set_capability(g_InputDevice, EV_KEY, KEY_O);
    input_set_capability(g_InputDevice, EV_KEY, KEY_M);
    input_set_capability(g_InputDevice, EV_KEY, KEY_C);
    input_set_capability(g_InputDevice, EV_KEY, KEY_E);
    input_set_capability(g_InputDevice, EV_KEY, KEY_S);
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

/*
#if defined(CONFIG_ENABLE_CHIP_MSG26XXM)
    input_set_abs_params(g_InputDevice, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
#endif //CONFIG_ENABLE_CHIP_MSG26XXM
    input_set_abs_params(g_InputDevice, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(g_InputDevice, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
    input_set_abs_params(g_InputDevice, ABS_MT_POSITION_X, TOUCH_SCREEN_X_MIN, TOUCH_SCREEN_X_MAX, 0, 0);
    input_set_abs_params(g_InputDevice, ABS_MT_POSITION_Y, TOUCH_SCREEN_Y_MIN, TOUCH_SCREEN_Y_MAX, 0, 0);
*/
#endif

    return nRetVal;    
}

s32 DrvPlatformLyrTouchDeviceRequestGPIO(void)
{
    s32 nRetVal = 0;

    DBG("*** %s() ***\n", __func__);
    
#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
    nRetVal = gpio_request(MS_TS_MSG_IC_GPIO_RST, "C_TP_RST");     
    if (nRetVal < 0)
    {
        DBG("*** Failed to request GPIO %d, error %d ***\n", MS_TS_MSG_IC_GPIO_RST, nRetVal);
    }

    nRetVal = gpio_request(MS_TS_MSG_IC_GPIO_INT, "C_TP_INT");    
    if (nRetVal < 0)
    {
        DBG("*** Failed to request GPIO %d, error %d ***\n", MS_TS_MSG_IC_GPIO_INT, nRetVal);
    }
#endif

    return nRetVal;    
}

s32 DrvPlatformLyrTouchDeviceRegisterFingerTouchInterruptHandler(void)
{
    s32 nRetVal = 0;

    DBG("*** %s() ***\n", __func__);

    if (DrvIcFwLyrIsRegisterFingerTouchInterruptHandler())
    {    	
#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
        /* initialize the finger touch work queue */ 
        INIT_WORK(&_gFingerTouchWork, _DrvPlatformLyrFingerTouchDoWork);

        _gIrq = gpio_to_irq(MS_TS_MSG_IC_GPIO_INT);

        /* request an irq and register the isr */
        nRetVal = request_irq(_gIrq/*MS_TS_MSG_IC_GPIO_INT*/, _DrvPlatformLyrFingerTouchInterruptHandler,
                      IRQF_TRIGGER_RISING /* | IRQF_NO_SUSPEND *//* IRQF_TRIGGER_FALLING */,
                      "msg2xxx", NULL);
        if (nRetVal != 0)
        {
            DBG("*** Unable to claim irq %d; error %d ***\n", MS_TS_MSG_IC_GPIO_INT, nRetVal);
        }
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
        mt_set_gpio_mode(MS_TS_MSG_IC_GPIO_INT, GPIO_CTP_EINT_PIN_M_EINT);
        mt_set_gpio_dir(MS_TS_MSG_IC_GPIO_INT, GPIO_DIR_IN);
        mt_set_gpio_pull_enable(MS_TS_MSG_IC_GPIO_INT, GPIO_PULL_ENABLE);
        mt_set_gpio_pull_select(MS_TS_MSG_IC_GPIO_INT, GPIO_PULL_UP);

        mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
        mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_TYPE/* EINTF_TRIGGER_RISING */, _DrvPlatformLyrFingerTouchInterruptHandler, 1);

        mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

#ifdef CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM
        /* initialize the finger touch work queue */ 
        INIT_WORK(&_gFingerTouchWork, _DrvPlatformLyrFingerTouchDoWork);
#else
        _gThread = kthread_run(_DrvPlatformLyrFingerTouchHandler, 0, TPD_DEVICE);
        if (IS_ERR(_gThread))
        { 
            nRetVal = PTR_ERR(_gThread);
            DBG("Failed to create kernel thread: %d\n", nRetVal);
        }
#endif //CONFIG_USE_IRQ_INTERRUPT_FOR_MTK_PLATFORM
#endif
    }
    
    return nRetVal;    
}	

#ifdef TPD_MSTAR_PROXIMITY_SENSOR
int tpd_get_MStarps_value(void)
{
    DBG("*** %s() ***\n", __func__);

	return tpd_proximity_detect;
}


int tpd_MStar_Proximity_Sensor_operate(void* self, uint32_t command, void* buff_in, int size_in,
				   void* buff_out, int size_out, int* actualout)
{


	int err = 0;
	int value;
	hwm_sensor_data *sensor_data;
	DBG("quark tpd_MStarps_operate  command = %d  ",command);
	switch (command)
	{
	case SENSOR_DELAY:
		if((buff_in == NULL) || (size_in < sizeof(int)))
		{
			err = -EINVAL;
		}
		// Do nothing
		break;

	case SENSOR_ENABLE:
		if((buff_in == NULL) || (size_in < sizeof(int)))
		{
			err = -EINVAL;
		}
		else
		{
			value = *(int *)buff_in;
			if(value)
			{
				if(( DrvFwCtrl_MStar_Proximity_sensor(1) != 0))
				{
					DBG("enable ps fail: %d\n", err);
					return -1;
				}
			}
			else
			{
				if(( DrvFwCtrl_MStar_Proximity_sensor(0) != 0))
				{
					DBG("disable ps fail: %d\n", err);
					return -1;
				}
			}
		}
		break;

	case SENSOR_GET_DATA:
		if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
		{
			DBG("get sensor data parameter error!\n");
			err = -EINVAL;
		}
		else
		{
			sensor_data = (hwm_sensor_data *)buff_out;

			sensor_data->values[0] = tpd_get_MStarps_value();
			sensor_data->value_divide = 1;
			sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
		}
		break;

	default:
		DBG(" tpd MStar proxmy sensor operate function no this parameter %d!\n", command);
		err = -1;
		break;
	}

	return err;

}
#endif
void DrvPlatformLyrTouchDeviceRegisterEarlySuspend(void)
{
#ifdef TPD_MSTAR_PROXIMITY_SENSOR
		int err = 0;
		struct hwmsen_object obj_ps;
#endif

    DBG("*** %s() ***\n", __func__);

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
#ifdef CONFIG_ENABLE_NOTIFIER_FB
    _gFbNotifier.notifier_call = MsDrvInterfaceTouchDeviceFbNotifierCallback;
    fb_register_client(&_gFbNotifier);
#else
    _gEarlySuspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
    _gEarlySuspend.suspend = MsDrvInterfaceTouchDeviceSuspend;
    _gEarlySuspend.resume = MsDrvInterfaceTouchDeviceResume;
    register_early_suspend(&_gEarlySuspend);
#endif //CONFIG_ENABLE_NOTIFIER_FB   
#endif    
#ifdef TPD_MSTAR_PROXIMITY_SENSOR
	obj_ps.polling = 0;//interrupt mode
	obj_ps.sensor_operate = tpd_MStar_Proximity_Sensor_operate;

	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		//goto exit_create_attr_failed;
	}
#endif
}

/* remove function is triggered when the input device is removed from input sub-system */
s32 DrvPlatformLyrTouchDeviceRemove(struct i2c_client *pClient)
{
    DBG("*** %s() ***\n", __func__);

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
//    free_irq(MS_TS_MSG_IC_GPIO_INT, g_InputDevice);
    free_irq(_gIrq, g_InputDevice);
    gpio_free(MS_TS_MSG_IC_GPIO_INT);
    gpio_free(MS_TS_MSG_IC_GPIO_RST);
    input_unregister_device(g_InputDevice);
#endif    
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    kset_unregister(g_TouchKSet);
    kobject_put(g_TouchKObj);
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

    return 0;
}

void DrvPlatformLyrSetIicDataRate(struct i2c_client *pClient, u32 nIicDataRate)
{
    DBG("*** %s() nIicDataRate = %d ***\n", __func__, nIicDataRate);

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM)
    // TODO : Please FAE colleague to confirm with customer device driver engineer for how to set i2c data rate on SPRD platform
    sprd_i2c_ctl_chg_clk(pClient->adapter->nr, nIicDataRate); 
    mdelay(100);
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
    // TODO : Please FAE colleague to confirm with customer device driver engineer for how to set i2c data rate on QCOM platform
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
    pClient->timing = nIicDataRate/1000;
#endif
}

//------------------------------------------------------------------------------//

#ifdef CONFIG_ENABLE_PROXIMITY_DETECTION

int DrvPlatformLyrGetTpPsData(void)
{
    DBG("*** %s() g_FaceClosingTp = %d ***\n", __func__, g_FaceClosingTp);
	
    return g_FaceClosingTp;
}

#if defined(CONFIG_TOUCH_DRIVER_RUN_ON_SPRD_PLATFORM) || defined(CONFIG_TOUCH_DRIVER_RUN_ON_QCOM_PLATFORM)
void DrvPlatformLyrTpPsEnable(int nEnable)
{
    DBG("*** %s() nEnable = %d ***\n", __func__, nEnable);

    if (nEnable)
    {
        DrvIcFwLyrEnableProximity();
    }
    else
    {
        DrvIcFwLyrDisableProximity();
    }
}
#elif defined(CONFIG_TOUCH_DRIVER_RUN_ON_MTK_PLATFORM)
int DrvPlatformLyrTpPsOperate(void* pSelf, u32 nCommand, void* pBuffIn, int nSizeIn,
				   void* pBuffOut, int nSizeOut, int* pActualOut)
{
    int nErr = 0;
    int nValue;
    hwm_sensor_data *pSensorData;

    switch (nCommand)
    {
        case SENSOR_DELAY:
            if ((pBuffIn == NULL) || (nSizeIn < sizeof(int)))
            {
                nErr = -EINVAL;
            }
            // Do nothing
            break;

        case SENSOR_ENABLE:
            if ((pBuffIn == NULL) || (nSizeIn < sizeof(int)))
            {
                nErr = -EINVAL;
            }
            else
            {
                nValue = *(int *)pBuffIn;
                if (nValue)
                {
                    if (DrvIcFwLyrEnableProximity() < 0)
                    {
                        DBG("Enable ps fail: %d\n", nErr);
                        return -1;
                    }
                }
                else
                {
                    if (DrvIcFwLyrDisableProximity() < 0)
                    {
                        DBG("Disable ps fail: %d\n", nErr);
                        return -1;
                    }
                }
            }
            break;

        case SENSOR_GET_DATA:
            if ((pBuffOut == NULL) || (nSizeOut < sizeof(hwm_sensor_data)))
            {
                DBG("Get sensor data parameter error!\n");
                nErr = -EINVAL;
            }
            else
            {
                pSensorData = (hwm_sensor_data *)pBuffOut;

                pSensorData->values[0] = DrvPlatformLyrGetTpPsData();
                pSensorData->value_divide = 1;
                pSensorData->status = SENSOR_STATUS_ACCURACY_MEDIUM;
            }
            break;

       default:
           DBG("Un-recognized parameter %d!\n", nCommand);
           nErr = -1;
           break;
    }

    return nErr;
}
#endif

#endif //CONFIG_ENABLE_PROXIMITY_DETECTION

//------------------------------------------------------------------------------//

