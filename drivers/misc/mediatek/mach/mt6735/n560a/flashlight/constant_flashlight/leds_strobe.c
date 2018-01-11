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
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif

#include <linux/i2c.h>
#include <linux/leds.h>


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
#define TAG_NAME "leds_strobe.c"
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

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;


static int g_timeOutTimeMs=0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif


#define STROBE_DEVICE_ID 0xCE


static struct work_struct workTimeOut;

#define FLASH_GPIO_ENF (GPIO42 | 0x80000000)
#define FLASH_GPIO_ENT (GPIO43 | 0x80000000)
#define GPIO_LED_EN  (GPIO43 | 0x80000000)


#define LM3646_REG_ENABLE      0x01
#define LM3646_REG_TIMING      0x04
#define LM3646_REG_FLASHTORCH_MAX   0x05
#define LM3646_REG_FLASH_LED1  0x06
#define LM3646_REG_TORCH_LED1  0x07


/*****************************************************************************
Functions
*****************************************************************************/
//extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
//extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int FL_ReadReg(int reg);
extern int FL_WriteReg(int reg, int value);


static void work_timeOutFunc(struct work_struct *data);
#if 0
int FL_ReadReg(int reg)
{
    char buf[2];
    char bufR[2];
    buf[0]=reg;
    iReadRegI2C(buf , 1, bufR,1, STROBE_DEVICE_ID);
    PK_DBG("qq reg=%x val=%x qq\n", buf[0],bufR[0]);
    return (int)bufR[0];
}

int FL_WriteReg(int reg, int data)
{
    char buf[2];
    buf[0]=reg;
    buf[1]=data;
	
    iWriteRegI2C(buf, 2, STROBE_DEVICE_ID);

   return 0;
}
#endif

#define e_DutyNum 16
#define TORCHDUTYNUM 2
static int isMovieMode[e_DutyNum+1][e_DutyNum+1] = 
{ 
   {-1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  	{1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  	{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
};

static int torchLED1Reg[e_DutyNum+1] = {0,32,32,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//93,187
static int torchMaxReg[TORCHDUTYNUM+1][TORCHDUTYNUM+1] = 
{
   {-1, 0x30, 0x30},
	{0x30, 0x30,-1},
	{0x30,-1,-1}
/*   {-1, 0x30, 0x70},
	{0x30, 0x70,-1},
	{0x70,-1,-1}*/
};

static int flashLED1Reg[e_DutyNum+1] = {0,7,15,23,31,39,47,55,63,71,79,87,95,103,111,119,127};//93,187,280,374,468,562,655,749,843,937,1030,1124,1218,1312,1405,1499

static int flashMaxReg[e_DutyNum+1][e_DutyNum+1] = 
{	
	   {-1,0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15},
       {0, 1, 2, 3, 4, 5, 6, 7, 8, 9,10, 11, 12, 13, 14, 15, -1},
	   {1, 2, 3, 4, 5, 6, 7, 8, 9,10,11, 12, 13, 14, 15, -1, -1},
	   {2, 3, 4, 5, 6, 7, 8, 9,10,11,12, 13, 14, 15, -1, -1, -1},
	   {3, 4, 5, 6, 7, 8, 9,10,11,12,13, 14, 15, -1, -1, -1, -1},
	   {4, 5, 6, 7, 8, 9,10,11,12,13,14, 15, -1, -1, -1, -1, -1},
	   {5, 6, 7, 8, 9,10,11,12,13,14,15, -1, -1, -1, -1, -1, -1},
	   {6, 7, 8, 9,10,11,12,13,14,15,-1, -1, -1, -1, -1, -1, -1},
	   {7, 8, 9,10,11,12,13,14,15,-1,-1, -1, -1, -1, -1, -1, -1},
	   {8, 9,10,11,12,13,14,15,-1,-1,-1, -1, -1, -1, -1, -1, -1},
	   {9,10,11,12,13,14,15,-1,-1,-1,-1, -1, -1, -1, -1, -1, -1},
	  {10,11,12,13,14,15,-1,-1,-1,-1,-1, -1, -1, -1, -1, -1, -1},
	  {11,12,13,14,15,-1,-1,-1,-1,-1,-1, -1, -1, -1, -1, -1, -1},
	  {12,13,14,15,-1,-1,-1,-1,-1,-1,-1, -1, -1, -1, -1, -1, -1},
	  {13,14,15,-1,-1,-1,-1,-1,-1,-1,-1, -1, -1, -1, -1, -1, -1},
	  {14,15,-1,-1,-1,-1,-1,-1,-1,-1,-1, -1, -1, -1, -1, -1, -1},
	  {15,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, -1, -1, -1, -1, -1, -1}

    

};


int m_duty1=0;
int m_duty2=0;
int LED1Closeflag = 0;
int LED2Closeflag = 0;

int FlashIc_Enable(void)
{
    if(mt_set_gpio_out(GPIO_LED_EN,GPIO_OUT_ONE)){PK_DBG(" set gpio failed!! \n");}
	PK_DBG("FlashIc_Enable!\n");
}

int FlashIc_Disable(void)
{
    if(FL_ReadReg(0x00)!=0x11) {
	    mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ZERO);
	    mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ZERO);
    }else {
	    if(mt_set_gpio_out(GPIO_LED_EN,GPIO_OUT_ZERO)){PK_DBG(" set gpio failed!! \n");}
    }
	PK_DBG("FlashIc_Disable!\n");
}


int flashEnable_LM3646_1(void)
{
	int temp;
	return 0;
}
int flashDisable_LM3646_1(void)
{
	int temp;
    return 0;
}

int setDuty_LM3646_1(int duty)
{

	if(duty<0)
		duty=0;
	else if(duty>=e_DutyNum)
		duty=e_DutyNum-1;
	m_duty1=duty;
	
	return 0;
}



int flashEnable_LM3646_2(void)
{
	int temp;

	PK_DBG("flashEnable_LM3646_2\n");
	PK_DBG("LED1Closeflag = %d, LED2Closeflag = %d\n", LED1Closeflag, LED2Closeflag);
	PK_DBG("FL_ReadReg(0x00) = %d\n",  FL_ReadReg(0x00));

	if(FL_ReadReg(0x00)!=0x11) {
		if((LED1Closeflag == 1) && (LED2Closeflag == 1))
		{
			mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ZERO);
			mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ZERO);
		}
		else if(LED1Closeflag == 1)
		{
			if(isMovieMode[0][m_duty2+1] == 1)
			{
				mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ZERO);
				mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ONE);
			}
			else
			{
				mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ONE);
				mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ZERO);
			}
		}
		else if(LED2Closeflag == 1)
		{
			if(isMovieMode[m_duty1+1][0] == 1)
			{
				mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ZERO);
				mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ONE);
			}
			else
			{
				mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ONE);
				mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ZERO);
			}
		}
		else
		{
			if(isMovieMode[m_duty1+1][m_duty2+1] == 1)
			{
				mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ZERO);
				mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ONE);
			}
			else
			{
				mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ONE);
				mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ZERO);
			}
		}
	} else {

		FL_WriteReg(LM3646_REG_TIMING, 0x47);//set timing
		if((LED1Closeflag == 1) && (LED2Closeflag == 1))
		{
			FL_WriteReg(LM3646_REG_ENABLE, 0xE0);//close
			FlashIc_Disable();			
		}
		else if(LED1Closeflag == 1)
		{
			if(isMovieMode[0][m_duty2+1] == 1)
			{
				FL_WriteReg(LM3646_REG_ENABLE, 0xE2);//torch mode
			}
			else
			{
				FL_WriteReg(LM3646_REG_ENABLE, 0xE3);//flash mode
			}
		}
		else if(LED2Closeflag == 1)
		{
			if(isMovieMode[m_duty1+1][0] == 1)
			{
				FL_WriteReg(LM3646_REG_ENABLE, 0xE2);//torch mode
			}
			else
			{
				FL_WriteReg(LM3646_REG_ENABLE, 0xE3);//flash mode		
			}
		}
		else
		{
			if(isMovieMode[m_duty1+1][m_duty2+1] == 1)
			{
				FL_WriteReg(LM3646_REG_ENABLE, 0xE2);//torch mode
			}
			else
			{
				FL_WriteReg(LM3646_REG_ENABLE, 0xE3);//flash mode
			}
		}
	}
	return 0;

}
int flashDisable_LM3646_2(void)
{
	int temp;

	PK_DBG("flashDisable_LM3646_2\n");
	PK_DBG("LED1Closeflag = %d, LED2Closeflag = %d\n", LED1Closeflag, LED2Closeflag);
	PK_DBG("FL_ReadReg(0x00) = %d\n",  FL_ReadReg(0x00));

	if(FL_ReadReg(0x00)!=0x11){
		if((LED1Closeflag == 1) && (LED2Closeflag == 1))
		{
			mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ZERO);
			mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ZERO);
		}
		else if(LED1Closeflag == 1)
		{
			if(isMovieMode[0][m_duty2+1] == 1)
			{
				mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ZERO);
				mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ONE);
			}
			else
			{
				mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ONE);
				mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ZERO);
			}
		}
		else if(LED2Closeflag == 1)
		{
			if(isMovieMode[m_duty1+1][0] == 1)
			{
				mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ZERO);
				mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ONE);
			}
			else
			{
				mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ONE);
				mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ZERO);
			}
		}
		else
		{
			if(isMovieMode[m_duty1+1][m_duty2+1] == 1)
			{
				mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ZERO);
				mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ONE);
			}
			else
			{
				mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ONE);
				mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ZERO);
			}
		}
	} else {

		FL_WriteReg(LM3646_REG_TIMING, 0x47);//set timing
		if((LED1Closeflag == 1) && (LED2Closeflag == 1))
		{
			FL_WriteReg(LM3646_REG_ENABLE, 0xE0);//close
			FlashIc_Disable();			
		}
		else if(LED1Closeflag == 1)
		{
			if(isMovieMode[0][m_duty2+1] == 1)
			{
				FL_WriteReg(LM3646_REG_ENABLE, 0xE2);//torch mode
			}
			else
			{
				FL_WriteReg(LM3646_REG_ENABLE, 0xE3);//flash mode
			}
		}
		else if(LED2Closeflag == 1)
		{
			if(isMovieMode[m_duty1+1][0] == 1)
			{
				FL_WriteReg(LM3646_REG_ENABLE, 0xE2);//torch mode
			}
			else
			{
				FL_WriteReg(LM3646_REG_ENABLE, 0xE3);//flash mode		
			}
		}
		else
		{
			if(isMovieMode[m_duty1+1][m_duty2+1] == 1)
			{
				FL_WriteReg(LM3646_REG_ENABLE, 0xE2);//torch mode
			}
			else
			{
				FL_WriteReg(LM3646_REG_ENABLE, 0xE3);//flash mode
			}
		}
	}
	
	return 0;
}


int setDuty_LM3646_2(int duty)
{
	if(duty<0)
		duty=0;
	else if(duty>=e_DutyNum)
		duty=e_DutyNum-1;
	m_duty2=duty;

	PK_DBG("setDuty_LM3646_2:m_duty = %d, m_duty2 = %d!\n", m_duty1, m_duty2);
	PK_DBG("LED1Closeflag = %d, LED2Closeflag = %d\n", LED1Closeflag, LED2Closeflag);

	if(FL_ReadReg(0x00)!=0x11) return 0;

	if((LED1Closeflag == 1) && (LED2Closeflag == 1))
	{
		
	}
	else if(LED1Closeflag == 1)
	{
		if(isMovieMode[0][m_duty2+1] == 1)
		{
			FL_WriteReg(LM3646_REG_TORCH_LED1, torchLED1Reg[1]);
			FL_WriteReg(LM3646_REG_FLASHTORCH_MAX, torchMaxReg[1][1]);
		}
		else
		{
			FL_WriteReg(LM3646_REG_FLASH_LED1, flashLED1Reg[5]);
			FL_WriteReg(LM3646_REG_FLASHTORCH_MAX, flashMaxReg[5+1][5+1]);	
		}
	}
	else if(LED2Closeflag == 1)
	{
		if(isMovieMode[m_duty1+1][0] == 1)
		{
			FL_WriteReg(LM3646_REG_TORCH_LED1, torchLED1Reg[1]);
			FL_WriteReg(LM3646_REG_FLASHTORCH_MAX, torchMaxReg[1][1]);
		}
		else
		{
			FL_WriteReg(LM3646_REG_FLASH_LED1, flashLED1Reg[5]);
			FL_WriteReg(LM3646_REG_FLASHTORCH_MAX, flashMaxReg[5+1][5+1]);	
		}		
	}
	else
	{
		if(isMovieMode[m_duty1+1][m_duty2+1] == 1)
		{
			FL_WriteReg(LM3646_REG_TORCH_LED1, torchLED1Reg[1]);
			FL_WriteReg(LM3646_REG_FLASHTORCH_MAX, torchMaxReg[1][1]);
		}
		else
		{
			FL_WriteReg(LM3646_REG_FLASH_LED1, flashLED1Reg[5]);
			FL_WriteReg(LM3646_REG_FLASHTORCH_MAX, flashMaxReg[5+1][5+1]);	
		}
	}

	return 0;
}


int FL_Enable(void)
{

	PK_DBG(" FL_Enable line=%d\n",__LINE__);


    return 0;
}



int FL_Disable(void)
{
	PK_DBG(" FL_Disable line=%d\n",__LINE__);
    return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
    setDuty_LM3646_1(duty);

    PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
    return 0;
}




int FL_Init(void)
{
	PK_DBG("LED1_FL_Init!\n");
    if(mt_set_gpio_mode(GPIO_LED_EN,GPIO_MODE_00)){PK_DBG(" set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_LED_EN,GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_LED_EN,GPIO_OUT_ZERO)){PK_DBG(" set gpio failed!! \n");}


    INIT_WORK(&workTimeOut, work_timeOutFunc);
    PK_DBG(" FL_Init line=%d\n",__LINE__);
    return 0;
}


int FL_Uninit(void)
{
	PK_DBG("LED1_FL_Uninit!\n");
	FlashIc_Disable();
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("LED1TimeOut_callback\n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
  	INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;

}



static int constant_flashlight_ioctl(MUINT32 cmd, MUINT32 arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("LM3646_LED1_constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {

		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",arg);
			g_timeOutTimeMs=arg;
		break;


    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("FLASHLIGHT_DUTY: %d\n",arg);
			m_duty1 = arg;
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
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
				LED1Closeflag = 0;
    			FL_Enable();
    		}
    		else
    		{
    			LED1Closeflag = 1;
    			FL_Disable();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
    	case FLASH_IOC_SET_REG_ADR:
    	    break;
    	case FLASH_IOC_SET_REG_VAL:
    	    break;
    	case FLASH_IOC_SET_REG:
    	    break;
    	case FLASH_IOC_GET_REG:
    	    break;



		default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

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
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
    PK_DBG(" constant_flashlight_release\n");

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

    PK_DBG(" Done\n");

    return 0;

}


FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

    return 0;
}

EXPORT_SYMBOL(strobe_VDIrq);


