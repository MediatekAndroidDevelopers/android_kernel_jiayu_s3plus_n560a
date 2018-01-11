#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
/* Vanzo:wuzhiyong on: Mon, 16 Mar 2015 14:16:57 +0800
 * sub_flashlight driver
 */ 
#include <mach/upmu_common.h>
#include <mach/upmu_hw.h>
//End of Vanzo:wuzhiyong
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>
#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#include "kd_flashlight.h"
/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
#define TAG_NAME "[sub_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_INFO  "%s: " fmt, __func__ ,##arg)
#define PK_WARN(fmt, arg...)        xlog_printk(ANDROID_LOG_WARNING, TAG_NAME, KERN_WARNING  "%s: " fmt, __func__ ,##arg)
#define PK_NOTICE(fmt, arg...)      xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_NOTICE  "%s: " fmt, __func__ ,##arg)
#define PK_INFO(fmt, arg...)        xlog_printk(ANDROID_LOG_INFO   , TAG_NAME, KERN_INFO  "%s: " fmt, __func__ ,##arg)
#define PK_TRC_FUNC(f)              xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME,  "<%s>\n", __func__);
#define PK_TRC_VERBOSE(fmt, arg...) xlog_printk(ANDROID_LOG_VERBOSE, TAG_NAME,  fmt, ##arg)
#define PK_ERROR(fmt, arg...)       xlog_printk(ANDROID_LOG_ERROR  , TAG_NAME, KERN_ERR "%s: " fmt, __func__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

#define GPIO_ENT (GPIO90 | 0x80000000) 


/* Vanzo:wuzhiyong on: Mon, 16 Mar 2015 14:16:57 +0800
 * sub_flashlight driver
 */ 
#define NLED_OFF 0
#define NLED_ON 1
static DEFINE_SPINLOCK(g_strobeSMPLock);
static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

static int g_duty=-1;
static int g_timeOutTimeMs=0;


typedef enum{
    PMIC_PWM_0 = 0,
    PMIC_PWM_1 = 1,
    PMIC_PWM_2 = 2
} MT65XX_PMIC_PWM_NUMBER;
typedef enum{
    //32K clock
    ISINK_1KHZ = 0,
    ISINK_200HZ = 4,
    ISINK_5HZ = 199,
    ISINK_2HZ = 499,
    ISINK_1HZ = 999,
    ISINK_05HZ = 1999,
    ISINK_02HZ = 4999,
    ISINK_01HZ = 9999,
    //2M clock
    ISINK_2M_20KHZ = 2,
    ISINK_2M_1KHZ = 61,
    ISINK_2M_200HZ = 311,
    ISINK_2M_5HZ = 12499,
    ISINK_2M_2HZ = 31249,
    ISINK_2M_1HZ = 62499
} MT65XX_PMIC_ISINK_FSEL;


typedef enum{
    ISINK_0 = 0,  //4mA
    ISINK_1 = 1,  //8mA
    ISINK_2 = 2,  //12mA
    ISINK_3 = 3,  //16mA
    ISINK_4 = 4,  //20mA
    ISINK_5 = 5   //24mA
} MT65XX_PMIC_ISINK_STEP;


static struct work_struct workTimeOut;
static void work_timeOutFunc(struct work_struct *data);

static int FL_Enable(void)
{
/* Vanzo:dingge on: Fri, 29 May 2015 11:46:07 +0800
 * TODO: replace this line with your comment
    pmic_set_register_value(PMIC_RG_DRV_32K_CK_PDN,0x0); // Disable     power down  
    pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_PDN,0);
    pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_CKSEL,0);
    pmic_set_register_value(PMIC_ISINK_CH0_MODE,PMIC_PWM_0);
    pmic_set_register_value(PMIC_ISINK_CH0_STEP,ISINK_5);//24mA
    pmic_set_register_value(PMIC_ISINK_DIM0_DUTY,15);
    pmic_set_register_value(PMIC_ISINK_DIM0_FSEL,ISINK_1KHZ);//1KHz
    pmic_set_register_value(PMIC_ISINK_CH0_EN,NLED_ON);
 */
    mt_set_gpio_out(GPIO_ENT,GPIO_OUT_ONE);

// End of Vanzo: dingge
    return 0;
}

static int FL_Disable(void)
{
/* Vanzo:dingge on: Fri, 29 May 2015 11:46:33 +0800
 * TODO: replace this line with your comment
    pmic_set_register_value(PMIC_RG_DRV_32K_CK_PDN,0x0); // Disable     power down  
    pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_PDN,0);
    pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_CKSEL,0);
    pmic_set_register_value(PMIC_ISINK_CH0_MODE,PMIC_PWM_0);
    pmic_set_register_value(PMIC_ISINK_CH0_STEP,ISINK_5);//24mA
    pmic_set_register_value(PMIC_ISINK_DIM0_DUTY,15);
    pmic_set_register_value(PMIC_ISINK_DIM0_FSEL,ISINK_1KHZ);//1KHz
    pmic_set_register_value(PMIC_ISINK_CH0_EN,NLED_OFF);
 */
    mt_set_gpio_out(GPIO_ENT,GPIO_OUT_ZERO);
// End of Vanzo: dingge
    PK_DBG(" sub_FL_Disable line=%d\n",__LINE__);

    return 0;
}

static int FL_dim_duty(kal_uint32 duty)
{
    g_duty=duty;
    PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
    return 0;
}

static int FL_Init(void)
{
    // ISINK0
/* Vanzo:dingge on: Fri, 29 May 2015 11:47:15 +0800
 * TODO: replace this line with your comment
    pmic_set_register_value(PMIC_RG_DRV_32K_CK_PDN,0x0); // Disable     power down  
    pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_PDN,0);
    pmic_set_register_value(PMIC_RG_DRV_ISINK0_CK_CKSEL,0);
    pmic_set_register_value(PMIC_ISINK_CH0_MODE,PMIC_PWM_0);
    pmic_set_register_value(PMIC_ISINK_CH0_STEP,ISINK_5);//24mA
    pmic_set_register_value(PMIC_ISINK_DIM0_DUTY,15);
    pmic_set_register_value(PMIC_ISINK_DIM0_FSEL,ISINK_1KHZ);//1KHz
    pmic_set_register_value(PMIC_ISINK_CH0_EN,NLED_OFF);
    INIT_WORK(&workTimeOut, work_timeOutFunc);
 */
    if(mt_set_gpio_mode(GPIO_ENT,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_ENT,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_ENT,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
// End of Vanzo: dingge
    PK_DBG(" FL_Init line=%d\n",__LINE__);

    return 0;
}

static int FL_Uninit(void)
{
    FL_Disable();
    return 0;
}

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("ledTimeOut_callback\n");
}


static enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
static void timerInit(void)
{
    g_timeOutTimeMs=1000; //1s
    hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
    g_timeOutTimer.function=ledTimeOutCallback;

}
//End of Vanzo:wuzhiyong

static int sub_strobe_ioctl(unsigned int cmd, unsigned long arg)
{
	PK_DBG("sub dummy ioctl");
/* Vanzo:wuzhiyong on: Mon, 16 Mar 2015 14:16:57 +0800
 * sub_flashlight driver
 */ 
    int i4RetValue = 0;
    int ior_shift;
    int iow_shift;
    int iowr_shift;
    ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
    iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
    iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
    PK_DBG("constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d      iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {

        case FLASH_IOC_SET_TIME_OUT_TIME_MS:
            PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",arg);
            g_timeOutTimeMs=arg;
        break;

        case FLASH_IOC_SET_DUTY :
            PK_DBG("FLASHLIGHT_DUTY: %d\n",arg);
            FL_dim_duty(arg);
            break;

        case FLASH_IOC_SET_STEP:
            PK_DBG("FLASH_IOC_SET_STEP: %d\n",arg);

            break;

        case FLASH_IOC_SET_ONOFF :
            PK_DBG("FLASHLIGHT_ONOFF: %d\n",arg);
            if(arg==1)
            {
                if(g_timeOutTimeMs!=0)
                {
                    ktime_t ktime;
                    ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
                    hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL     );
                }
                FL_Enable();
            }
            else
            {
                FL_Disable();
                hrtimer_cancel( &g_timeOutTimer );
            }
            break;
        default :
            PK_DBG(" No such command \n");
            i4RetValue = -EPERM;
            break;
    }
    return i4RetValue;
    //return 0;
//End of Vanzo:wuzhiyong
}

static int sub_strobe_open(void *pArg)
{
    PK_DBG("sub dummy open");
/* Vanzo:wuzhiyong on: Mon, 16 Mar 2015 14:16:57 +0800
 * sub_flashlight driver
 */ 
    if (0 == strobe_Res)
    {
        FL_Init();
        timerInit();
    }
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
    spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        //i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
//End of Vanzo:wuzhiyong

    return 0;

}

static int sub_strobe_release(void *pArg)
{
    PK_DBG("sub dummy release");
/* Vanzo:wuzhiyong on: Mon, 16 Mar 2015 14:16:57 +0800
 * sub_flashlight driver
 */ 
    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

        FL_Uninit();
    }
//End of Vanzo:wuzhiyong

    return 0;

}

FLASHLIGHT_FUNCTION_STRUCT	subStrobeFunc=
{
	sub_strobe_open,
	sub_strobe_release,
	sub_strobe_ioctl
};


MUINT32 subStrobeInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &subStrobeFunc;
    }
    return 0;
}





