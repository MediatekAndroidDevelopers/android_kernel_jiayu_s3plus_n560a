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
#include <linux/i2c.h>
#include <linux/kthread.h>
#include "aw2013.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include <mach/mt_gpio.h>
#include <mach/mt_gpt.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#define USE_UNLOCKED_IOCTL

//s_add new breathlight driver here

//e_add new breathlight driver here
/******************************************************************************
 * Definition
******************************************************************************/
#ifndef TRUE
#define TRUE KAL_TRUE
#endif
#ifndef FALSE
#define FALSE KAL_FALSE
#endif

/* device name and major number */
#define BREATHLIGHT_DEVNAME            "breathlight"

#define DELAY_MS(ms) {mdelay(ms);}	//unit: ms(10^-3)
#define DELAY_US(us) {mdelay(us);}	//unit: us(10^-6)
#define DELAY_NS(ns) {mdelay(ns);}	//unit: ns(10^-9)
/*
    non-busy dealy(/kernel/timer.c)(CANNOT be used in interrupt context):
        ssleep(sec)
        msleep(msec)
        msleep_interruptible(msec)

    kernel timer
*/

#define ALLOC_DEVNO

/******************************************************************************
 * Project specified configuration
******************************************************************************/
#define AW2013_I2C_ADDR 0x8a
#define AW2013_I2C_BUSNUM 2

/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[BREATHLIGHT]"

#define DEBUG_BREATHLIGHT
#ifdef DEBUG_BREATHLIGHT
#define BL_DBG(fmt, arg...) printk(PFX "%s: " fmt, __FUNCTION__ ,##arg)
#define BL_TRC(fmt, arg...) printk(PFX "<%s>\n", __FUNCTION__);
#define BL_ERR(fmt, arg...) printk(PFX "%s: " fmt, __FUNCTION__ ,##arg)
#else
#define BL_DBG(a,...)
#define BL_TRC(a,...)
#define BL_ERR(a,...)
#endif
/*****************************************************************************

*****************************************************************************/
#define BREATHLIGHT_DEVICE "aw2013"

static struct task_struct *thread = NULL;
static struct task_struct *probe_thread = NULL;
static struct i2c_client *aw2013_i2c_client = NULL;

static const struct i2c_device_id bl_id[] = { {BREATHLIGHT_DEVICE, 0}, {} };
static struct i2c_board_info __initdata i2c_aw2013 = { I2C_BOARD_INFO("aw2013", (AW2013_I2C_ADDR >> 1)) };

//      static unsigned short force[] = {0,AW2013_I2C_ADDR,I2C_CLIENT_END,I2C_CLIENT_END};
//      static const unsigned short * const forces[] = { force, NULL };
//      static struct i2c_client_address_data addr_data = { .forces = forces, };

static int aw2013_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	aw2013_i2c_client = client;

	//add i2c probe code later

	BL_DBG("[aw2013_probe] E\n");
	return 0;
}

static int aw2013_remove(struct i2c_client *client)
{
	BL_DBG("[aw2013_remove] E\n");
	return 0;
}

static int aw2013_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	BL_DBG("[aw2013_detect] E\n");

	strcpy(info->type, BREATHLIGHT_DEVICE);
	return 0;
}

static struct i2c_driver aw2013_i2c_driver = {
    .driver = {
           .name = BREATHLIGHT_DEVICE,
//              .owner = THIS_MODULE,
           },
    .probe = aw2013_probe,
    .remove = aw2013_remove,
    .id_table = bl_id,
    .detect = aw2013_detect,
//       .address_data = &addr_data,
};

/*****************************************************************************

*****************************************************************************/
#ifdef USE_UNLOCKED_IOCTL
static int breathlight_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#else
static int breathlight_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
	int i4RetValue = 0;

//	BL_DBG("%x, %x \n", cmd, arg);

	switch (cmd) {
	case BREATHLIGHT_IOCTL_INIT:
		BL_DBG("BREATHLIGHT_IOCTL_INIT\n");
		break;

	case BREATHLIGHT_IOCTL_MODE:
		BL_DBG("BREATHLIGHT_IOCTL_MODE\n");
		break;

	case BREATHLIGHT_IOCTL_CURRENT:
		BL_DBG("BREATHLIGHT_IOCTL_CURRENT\n");
		break;

	case BREATHLIGHT_IOCTL_ONOFF:
		BL_DBG("BREATHLIGHT_IOCTL_ONOFF\n");
		break;

	case BREATHLIGHT_IOCTL_PWM_ONESHOT:
		BL_DBG("BREATHLIGHT_IOCTL_ONOFF\n");
		break;

	case BREATHLIGHT_IOCTL_TIME:
		BL_DBG("BREATHLIGHT_IOCTL_ONOFF\n");
		break;

	default:
		break;
	}

	return i4RetValue;
}

static ssize_t breathlight_read(struct file *file, const char __user * user_buf, size_t count,loff_t * ppos)
{
	int i4RetValue = 0;
	BL_DBG("[breathlight_read] E\n");

	return i4RetValue;
}

static ssize_t breathlight_write(struct file *file, char __user * buf, size_t size, loff_t * ppos)
{
	int i4RetValue = 0;
	BL_DBG("[breathlight_write] E\n");

	return size;
}

static int breathlight_open(struct inode *inode, struct file *file)
{
	int i4RetValue = 0;
	BL_DBG("[breathlight_open] E\n");

	return i4RetValue;
}

static int breathlight_release(struct inode *inode, struct file *file)
{
	int i4RetValue = 0;
	BL_DBG("[breathlight_release] E\n");

	return i4RetValue;
}

/*****************************************************************************/
/* Kernel interface */
static struct file_operations breathlight_fops = {
	.owner = THIS_MODULE,
#ifdef USE_UNLOCKED_IOCTL
	.unlocked_ioctl = breathlight_ioctl,
#else
	.ioctl = breathlight_ioctl,
#endif
	.open = breathlight_open,
	.release = breathlight_release,
	.read = breathlight_read,
	.write = breathlight_write,
};

/*****************************************************************************
Driver interface
*****************************************************************************/
struct breathlight_data {
	spinlock_t lock;
	wait_queue_head_t read_wait;
	struct semaphore sem;
};
static struct class *breathlight_class = NULL;
static struct device *breathlight_device = NULL;
//static struct breathlight_data breathlight_private;
static dev_t breathlight_devno;
static struct cdev breathlight_cdev;

static int breath_num = 0;
static int breath_time = 0;
/****************************************************************************/

static unsigned char g_Ledctrl1 = 0, g_Ledctrl2 = 0, g_LedCurrentSet = 1, g_LedAudioGainSet = 4;

void RE_IIC_WriteReg(unsigned char index, unsigned char value)
{
	unsigned char databuf[2];
	int res;

	databuf[0] = index;
	databuf[1] = value;
	res = i2c_master_send(aw2013_i2c_client, databuf, 0x2);
	if (res <= 0) {
		BL_ERR("[breathlight_init] i2c_master_send fail\n");
	}
}

enum {
  LED_OFF = 0,
  LED_RED,
  LED_GREEN,
  LED_BLUE,
  LED_CONST_RED,
  LED_CONST_GREEN,
  LED_CONST_BLUE,
  LED_CONST_RG,
  LED_CONST_RB,
  LED_CONST_GB,
  LED_CONST_RGB,
  LED_BREATH_RG,
  LED_BREATH_RB,
  LED_BREATH_GB,
  LED_BREATH_RGB,
  AUTO_BREATH,
  AUTO_BREATH_ON_SHUTDOWN,
  BRIGHT_STATUS,
  FREE_BUTTON,
  LED_MAX
};

/*********************************************************/
//Chip on
/********************************************************/
void Led_ChipOn()
{
	RE_IIC_WriteReg(AW2013_REG_CHIP_ONFF, 1);
}

/*********************************************************/
//Chip off by Software
/********************************************************/
void Led_ChipOff()
{
	RE_IIC_WriteReg(AW2013_REG_CHIP_ONFF, 0);
}

//以下为调节呼吸效果的参数
#define Imax          0x02   //LED最大电流配置,0x00=0mA,0x01=5mA,0x02=10mA,0x03=15mA,
#define Rise_time   0x03   //LED呼吸上升时间,0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Hold_time   0x03   //LED呼吸到最亮时的保持时间0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s
#define Fall_time     0x05   //LED呼吸下降时间,0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Off_time      0x04   //LED呼吸到灭时的保持时间0x00=013s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Delay_time   0x00   //LED呼吸启动后的延迟时间0x00=0s,0x01=0.13s,0x02=0.26s,0x03=0.52s,0x04=1.04s,0x05=2.08s,0x06=4.16s,0x07=8.32s,0x08=16.64s
#define Period_Num  0x00   //LED呼吸次数0x00=无限次,0x01=1次,0x02=2次.....0x0f=15次
void Led_SetLed(UINT16 Ch)
{
    int j=0;

	switch (Ch) {
	case 0:		//Led 1
		RE_IIC_WriteReg(AW2013_REG_PWM_OUT, 1<<Ch);
		break;
	case 1:		//Led 2
		RE_IIC_WriteReg(AW2013_REG_PWM_OUT, 1<<Ch);
		break;
	case 2:		//Led 3
		RE_IIC_WriteReg(AW2013_REG_PWM_OUT, 1<<Ch);
		break;
    default:

		RE_IIC_WriteReg(AW2013_REG_PWM_OUT, 0);
        break;

	}
    for(j=0;j<50000;j++);//delay 5us
}


void Led_SetBreath(UINT16 Ch, UINT8 riseTime, UINT8 holdTime, UINT8 fallTime, UINT8 offTime,UINT8 delayTime,UINT8 periodNum)
{
	if ((Ch > 2) || (riseTime > 7) || (holdTime > 5) || (fallTime > 7) || (offTime > 7)||(delayTime>8)||(periodNum>15))
		return;

    RE_IIC_WriteReg(0x37+Ch*3, riseTime<<4|holdTime);   //led0  t_rise=0.52s  && t_hold=1.04s
    RE_IIC_WriteReg(0x38+Ch*3, fallTime<<4|offTime);    //led0  t_fall=0.52s  && t_off=4.16s
    RE_IIC_WriteReg(0x39+Ch*3, delayTime<<4|periodNum); //led0  t_Delay=0s && cnt=无穷大
}

void Led_SetCurrent(UINT8 id,UINT8 iMax)
{
     if ((iMax > 0x3)||(id>2))
        return;

     RE_IIC_WriteReg(0x31+id, 0x70|iMax);
}
void Led_SetLevel(UINT8 id,UINT8 level)
{
     if ((level > 0xff)||(id>2))
        return;

     RE_IIC_WriteReg(0x34+id, level);
}


void Led_BreathPause(UINT8 RgbRM, UINT8 RgbHT)
{
	RE_IIC_WriteReg(AW2013_REG_RST, (RgbRM << 4) | RgbHT);
}


///////////////////AW2013呼吸灯程序
void led_flash_aw2013( unsigned int id)   //id = 0/1/2，分别对应LED0 LED1 LED2
{
       unsigned char reg_data;
       unsigned int    reg_buffer[8];
       int j=0;
       RE_IIC_WriteReg(0x01, 0x01);       // enable LED 不使用中断
       RE_IIC_WriteReg(0x31+id, 0x70|Imax);   //config mode, IMAX
       RE_IIC_WriteReg(0x37+id*3, Rise_time<<4 |Hold_time);   //led0  t_rise=0.52s  && t_hold=1.04s
       RE_IIC_WriteReg(0x38+id*3, Fall_time<<4 |Off_time);    //led0  t_fall=0.52s  && t_off=4.16s
       RE_IIC_WriteReg(0x39+id*3, Delay_time<<4 |Period_Num); //led0  t_Delay=0s && cnt=无穷大
       RE_IIC_WriteReg(0x30, 1<<id);  //led on
       for (j=0; j < 50000; j++);//需延时5us以上
 }

void led_breath_turnoff()
{
  Led_ChipOff();
  Led_SetLed(0xff);
}
// rgb 0 -> R  1 -> B  2 -> G
// mode 0->const mode   1->breath mode
// level  0~255
void Led_Set_Mode(int levelR,int levelG,int levelB,int mode)
{
	if(mode>1)
		mode = 1;
	led_breath_turnoff();
	if(mode)
	{
		Led_ChipOn();
		RE_IIC_WriteReg(0x31,0x73);
		RE_IIC_WriteReg(0x32,0x73);
		RE_IIC_WriteReg(0x33,0x73);
		RE_IIC_WriteReg(0x34,levelR);
		RE_IIC_WriteReg(0x35,levelB);
		RE_IIC_WriteReg(0x36,levelG);
		Led_SetBreath(0,3,2,3,2,0,0);
		Led_SetBreath(1,3,2,3,2,0,0);
		Led_SetBreath(2,3,2,3,2,0,0);
		RE_IIC_WriteReg(0x30, 0x07);  //led on
	}
	else
	{
		Led_ChipOn();
		RE_IIC_WriteReg(0x31, 0x03);
		RE_IIC_WriteReg(0x34, levelR);
		RE_IIC_WriteReg(0x32, 0x03);
		RE_IIC_WriteReg(0x35, levelB);
		RE_IIC_WriteReg(0x33, 0x03);
		RE_IIC_WriteReg(0x36, levelG);
		RE_IIC_WriteReg(0x30, 0x07);  //led on
	}
}
void led_breath_turnon();
void led_test(int test_num)
{
  breath_num = test_num;
  led_breath_turnon();
}

void led_breath_turnon()
{
    switch (breath_num)
    {

	  case LED_GREEN:
        Led_ChipOn();
        Led_SetCurrent(0,3);
        Led_SetLevel(0,0xff);
        if(!breath_time)
          Led_SetBreath(0,3,2,3,2,0,0);
        else
          Led_SetBreath(0,4,3,4,3,0,0);
        Led_SetLed(0);
	  break;

	  case LED_BLUE:
        Led_ChipOn();
        Led_SetCurrent(1,3);
        Led_SetLevel(1,0xff);
        if(!breath_time)
          Led_SetBreath(1,3,2,3,2,0,0);
        else
          Led_SetBreath(1,4,3,4,3,0,0);
        Led_SetLed(1);
	  break;

	  case LED_RED:
        Led_ChipOn();
        Led_SetCurrent(2,3);
        Led_SetLevel(2,0xff);
        if(!breath_time)
          Led_SetBreath(2,3,2,3,2,0,0);
        else
          Led_SetBreath(2,4,3,4,3,0,0);
        Led_SetLed(2);
	  break;

      case LED_CONST_GREEN:
        Led_ChipOn();
        RE_IIC_WriteReg(0x31, 0x03);
        RE_IIC_WriteReg(0x34, 0xff);
        Led_SetLed(0);
      break;

      case LED_CONST_BLUE:
        Led_ChipOn();
        RE_IIC_WriteReg(0x32, 0x03);
        RE_IIC_WriteReg(0x35, 0xff);
        Led_SetLed(1);

      break;

      case LED_CONST_RED:
        Led_ChipOn();
        RE_IIC_WriteReg(0x33, 0x03);
        RE_IIC_WriteReg(0x36, 0xff);
        Led_SetLed(2);
        break;
      case LED_CONST_GB://G+B
        Led_ChipOn();
        RE_IIC_WriteReg(0x31, 0x03);
        RE_IIC_WriteReg(0x34, 0xff);
        RE_IIC_WriteReg(0x32, 0x03);
        RE_IIC_WriteReg(0x35, 0xff);
        RE_IIC_WriteReg(0x30, 0x03);  //led on
      break;

      case LED_CONST_RG://G+R
        Led_ChipOn();
        RE_IIC_WriteReg(0x31, 0x03);
        RE_IIC_WriteReg(0x34, 0xff);
        RE_IIC_WriteReg(0x33, 0x03);
        RE_IIC_WriteReg(0x36, 0xff);
        RE_IIC_WriteReg(0x30, 0x05);  //led on
      break;

      case LED_CONST_RB://B+R
        Led_ChipOn();
        RE_IIC_WriteReg(0x32, 0x03);
        RE_IIC_WriteReg(0x35, 0xff);
        RE_IIC_WriteReg(0x33, 0x03);
        RE_IIC_WriteReg(0x36, 0xff);
        RE_IIC_WriteReg(0x30, 0x06);  //led on
        break;
      case LED_CONST_RGB://G+B+R
        Led_ChipOn();
        RE_IIC_WriteReg(0x31, 0x03);
        RE_IIC_WriteReg(0x34, 0xff);
        RE_IIC_WriteReg(0x32, 0x03);
        RE_IIC_WriteReg(0x35, 0xff);
        RE_IIC_WriteReg(0x33, 0x03);
        RE_IIC_WriteReg(0x36, 0xff);
        RE_IIC_WriteReg(0x30, 0x07);  //led on
        break;
      case LED_BREATH_GB://G+B
        Led_ChipOn();
        RE_IIC_WriteReg(0x31, 0x73);
        RE_IIC_WriteReg(0x34, 0xff);
        Led_SetBreath(0,3,2,3,2,0,0);
        RE_IIC_WriteReg(0x32, 0x73);
        RE_IIC_WriteReg(0x35, 0xff);
        Led_SetBreath(1,3,2,3,2,0,0);
        RE_IIC_WriteReg(0x30, 0x03);  //led on
      break;

      case LED_BREATH_RG://G+R
        Led_ChipOn();
        RE_IIC_WriteReg(0x31, 0x73);
        RE_IIC_WriteReg(0x34, 0xff);
        Led_SetBreath(0,3,2,3,2,0,0);
        RE_IIC_WriteReg(0x33, 0x73);
        RE_IIC_WriteReg(0x36, 0xff);
        Led_SetBreath(2,3,2,3,2,0,0);
        RE_IIC_WriteReg(0x30, 0x05);  //led on
      break;

      case LED_BREATH_RB://B+R
        Led_ChipOn();
        RE_IIC_WriteReg(0x32, 0x73);
        RE_IIC_WriteReg(0x35, 0xff);
        Led_SetBreath(1,3,2,3,2,0,0);
        RE_IIC_WriteReg(0x33, 0x73);
        RE_IIC_WriteReg(0x36, 0xff);
        Led_SetBreath(2,3,2,3,2,0,0);
        RE_IIC_WriteReg(0x30, 0x06);  //led on
        break;
      case LED_BREATH_RGB://G+B+R
        Led_ChipOn();
        RE_IIC_WriteReg(0x31, 0x73);
        RE_IIC_WriteReg(0x34, 0xff);
        Led_SetBreath(0,3,2,3,2,0,0);
        RE_IIC_WriteReg(0x32, 0x73);
        RE_IIC_WriteReg(0x35, 0xff);
        Led_SetBreath(1,3,2,3,2,0,0);
        RE_IIC_WriteReg(0x33, 0x73);
        RE_IIC_WriteReg(0x36, 0xff);
        Led_SetBreath(2,3,2,3,2,0,0);
        RE_IIC_WriteReg(0x30, 0x07);  //led on
        break;
      case AUTO_BREATH:
        RE_IIC_WriteReg(0x01, 0x01);    // enable LED 不使用中断
        RE_IIC_WriteReg(0x31, 0x73);    //config mode, IMAX
        RE_IIC_WriteReg(0x32, 0x73);    //config mode, IMAX
        RE_IIC_WriteReg(0x33, 0x73);    //config mode, IMAX
        RE_IIC_WriteReg(0x34, 0xff);    // LED0 level,
        RE_IIC_WriteReg(0x35, 0xff);    // LED1 level,
        RE_IIC_WriteReg(0x36, 0xff);    // LED0 level,
        RE_IIC_WriteReg(0x37, 0x34);    //led0  t_rise=1.04s  && t_hold=2.08s
        RE_IIC_WriteReg(0x38, 0x40);    //led0  t_fall=2.08s  && t_off=0s
        RE_IIC_WriteReg(0x39, 0x00);    //led0  t_Delay=0s && cnt=无穷大
        RE_IIC_WriteReg(0x3A, 0x34);    //led1  t_rise=1.04s  && t_hold=2.08s
        RE_IIC_WriteReg(0x3B, 0x40);    //led1  t_fall=2.08s  && t_off=0s
        RE_IIC_WriteReg(0x3C, 0x00);    //led1  t_Delay=0s && cnt=无穷大
        RE_IIC_WriteReg(0x3D, 0x34);    //led2  t_rise=1.04s  && t_hold=2.08s
        RE_IIC_WriteReg(0x3E, 0x40);    //led2  t_fall=2.08s  && t_off=0s
        RE_IIC_WriteReg(0x3F, 0x00);    //led2  t_Delay=0s && cnt=无穷大
        RE_IIC_WriteReg(0x30, 0x07);    //led on
        break;

      case AUTO_BREATH_ON_SHUTDOWN:
        RE_IIC_WriteReg(0x01, 0x01);    // enable LED 不使用中断
        RE_IIC_WriteReg(0x31, 0x73);    //config mode, IMAX
        RE_IIC_WriteReg(0x32, 0x73);    //config mode, IMAX
        RE_IIC_WriteReg(0x33, 0x73);    //config mode, IMAX
        RE_IIC_WriteReg(0x34, 0xff);    // LED0 level,
        RE_IIC_WriteReg(0x35, 0xff);    // LED1 level,
        RE_IIC_WriteReg(0x36, 0xff);    // LED0 level,
        RE_IIC_WriteReg(0x37, 0x23);    //led0  t_rise=0.52s  && t_hold=1.04s
        RE_IIC_WriteReg(0x38, 0x35);    //led0  t_fall=0.52s  && t_off=4.16s
        RE_IIC_WriteReg(0x39, 0x00);    //led0  t_Delay=0s && cnt=无穷大
        RE_IIC_WriteReg(0x3A, 0x23);    //led1  t_rise=0.52s  && t_hold=1.04s
        RE_IIC_WriteReg(0x3B, 0x35);    //led1  t_fall=0.52s  && t_off=4.16s
        RE_IIC_WriteReg(0x3C, 0x00);    //led1  t_Delay=0s && cnt=无穷大
        RE_IIC_WriteReg(0x3D, 0x23);    //led2  t_rise=0.52s  && t_hold=1.04s
        RE_IIC_WriteReg(0x3E, 0x35);    //led2  t_fall=0.52s  && t_off=4.16s
        RE_IIC_WriteReg(0x3F, 0x00);    //led2  t_Delay=0s && cnt=无穷大
        RE_IIC_WriteReg(0x30, 0x07);    //led on
        break;

      case BRIGHT_STATUS:
        RE_IIC_WriteReg(0x01, 0x01);    // enable LED 不使用中断
        RE_IIC_WriteReg(0x31, 0x63);    //config mode, IMAX
        RE_IIC_WriteReg(0x32, 0x63);    //config mode, IMAX
        RE_IIC_WriteReg(0x33, 0x63);    //config mode, IMAX
        RE_IIC_WriteReg(0x37, 0x20);    //led0  t_rise=0.52s  && t_hold=0s
        RE_IIC_WriteReg(0x38, 0x20);    //led0  t_fall=0.52s  && t_off=0s
        RE_IIC_WriteReg(0x3A, 0x20);    //led1  t_rise=0.52s  && t_hold=0s
        RE_IIC_WriteReg(0x3B, 0x20);    //led1  t_fall=0.52s  && t_off=0s
        RE_IIC_WriteReg(0x3D, 0x20);    //led2  t_rise=0.52s  && t_hold=0s
        RE_IIC_WriteReg(0x3B, 0x20);    //led2  t_fall=0.52s  && t_off=0s
        RE_IIC_WriteReg(0x34, 90);      // LED0 level,
        RE_IIC_WriteReg(0x35, 90);      //LED1 level,
        RE_IIC_WriteReg(0x36, 90);      //LED2 level,
        RE_IIC_WriteReg(0x30, 0x07);    //led on
        break;

      case FREE_BUTTON:
        RE_IIC_WriteReg(0x01, 0x01);    // enable LED 不使用中断
        RE_IIC_WriteReg(0x31, 0x63);    //config mode, IMAX
        RE_IIC_WriteReg(0x32, 0x63);    //config mode, IMAX
        RE_IIC_WriteReg(0x33, 0x63);    //config mode, IMAX
        RE_IIC_WriteReg(0x30, 0x07);    //led on
        RE_IIC_WriteReg(0x34, 255);     // LED0 level,
        RE_IIC_WriteReg(0x35, 255);     // LED1 level,
        RE_IIC_WriteReg(0x36, 255);     // LED2 level,
        msleep(500);
        RE_IIC_WriteReg(0x34, 90);      // LED0 level,
        RE_IIC_WriteReg(0x35, 90);      // LED1 level,
        RE_IIC_WriteReg(0x36, 90);      // LED2 level,
        break;

	  case LED_OFF:
        led_breath_turnoff();
	    break;
    }
}

static size_t breaghlight_show_mode(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	BL_DBG();

	return snprintf(buf, PAGE_SIZE, "breaghlight_show_mode\n");
}

static size_t breathlight_store_mode(struct device *dev, struct device_attribute *attr,
				     const char *buf, size_t size)
{
	BL_DBG();

	return size;
}

static size_t breaghlight_show_pwm(struct device *dev, struct device_attribute *attr,
				   char *buf)
{
	BL_DBG();

	return snprintf(buf, PAGE_SIZE, "breaghlight_show_pwm\n");
}

static size_t breathlight_store_pwm(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	BL_DBG();

	return size;
}

static size_t breaghlight_show_open(struct device *dev, struct device_attribute *attr, char *buf)
{
	BL_DBG();

	return snprintf(buf, PAGE_SIZE, "breaghlight_show_open\n");
}

static size_t breathlight_store_open(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	BL_DBG("[breathlight_dev_open] E\n");

	sscanf(buf, "%d", &breath_num);
	BL_DBG("breathlight num=%d\n",breath_num);
    led_breath_turnon();
	return size;
}

static size_t breaghlight_show_rgb(struct device *dev, struct device_attribute *attr, char *buf)
{
	BL_DBG();

	return snprintf(buf, PAGE_SIZE, "this rgb has four parameter R_level G_level B_level mode\n");
}

static size_t breathlight_store_rgb(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int mode,levelr,levelg,levelb;
	if(4 == sscanf(buf,"%d %d %d %d",&levelr,&levelg,&levelb,&mode))
    Led_Set_Mode(levelr,levelg,levelb,mode);
	return size;
}

static size_t breaghlight_show_time(struct device *dev, struct device_attribute *attr, char *buf)
{
	BL_DBG();

	return snprintf(buf, PAGE_SIZE, "breaghlight_show_time\n");
}

static size_t breathlight_store_time(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	BL_DBG("[breathlight_dev_time] E\n");

	sscanf(buf, "%d", &breath_time);
	BL_DBG("breathlight time =%d\n",breath_time);
	return size;
}

static DEVICE_ATTR(mode, 0664, breaghlight_show_mode, breathlight_store_mode);
static DEVICE_ATTR(pwm, 0664, breaghlight_show_pwm, breathlight_store_pwm);
static DEVICE_ATTR(open, 0666, breaghlight_show_open, breathlight_store_open);
static DEVICE_ATTR(rgb, 0666, breaghlight_show_rgb, breathlight_store_rgb);
static DEVICE_ATTR(time, 0666, breaghlight_show_time, breathlight_store_time);

static struct device_attribute *breathlight_attr_list[] = {
	&dev_attr_mode,
	&dev_attr_pwm,
	&dev_attr_open,
	&dev_attr_rgb,
	&dev_attr_time,
};

static int breathlight_create_attr(struct device *dev)
{
	int idx, err = 0;
	int num =
	    (int) sizeof(breathlight_attr_list) /
	    sizeof(breathlight_attr_list[0]);

	if (!dev)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		if (err =
		    device_create_file(dev, breathlight_attr_list[idx]))
			break;
	}
	return err;
}

static int breathlight_delete_attr(struct device *dev)
{
	int idx, err = 0;
	int num =
	    (int) sizeof(breathlight_attr_list) /
	    sizeof(breathlight_attr_list[0]);

	if (!dev)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) {
		device_remove_file(dev, breathlight_attr_list[idx]);
	}

	return err;
}

static int breathlight_probe(struct platform_device *dev)
{
	int ret = 0, err = 0;

	BL_DBG("[breathlight_probe] start\n");

#ifdef ALLOC_DEVNO		//dynamic allocate device no
	
    ret = alloc_chrdev_region(&breathlight_devno, 0, 1,	BREATHLIGHT_DEVNAME);
	if (ret) {
		BL_ERR("[breathlight_probe] alloc_chrdev_region fail: %d\n", ret);
		goto breathlight_probe_error;
	} else {
		BL_DBG("[breathlight_probe] major: %d, minor: %d\n",
		       MAJOR(breathlight_devno), MINOR(breathlight_devno));
	}
	cdev_init(&breathlight_cdev, &breathlight_fops);
	breathlight_cdev.owner = THIS_MODULE;
	err = cdev_add(&breathlight_cdev, breathlight_devno, 1);
	if (err) {
		BL_ERR("[breathlight_probe] cdev_add fail: %d\n", err);
		goto breathlight_probe_error;
	}
#else
#define BREATHLIGHT_MAJOR 242
	ret = register_chrdev(BREATHLIGHT_MAJOR, BREATHLIGHT_DEVNAME, &breathlight_fops);
	if (ret != 0) {
		BL_ERR("[breathlight_probe] Unable to register chardev on major=%d (%d)\n", BREATHLIGHT_MAJOR, ret);
		return ret;
	}
	breathlight_devno = MKDEV(BREATHLIGHT_MAJOR, 0);
#endif


	breathlight_class = class_create(THIS_MODULE, "breathlightdrv");
	if (IS_ERR(breathlight_class)) {
		BL_ERR("[breathlight_probe] Unable to create class, err = %d\n",(int) PTR_ERR(breathlight_class));
		goto breathlight_probe_error;
	}

	breathlight_device = device_create(breathlight_class, NULL, breathlight_devno, NULL, BREATHLIGHT_DEVNAME);
	if (NULL == breathlight_device) {
		BL_ERR("[breathlight_probe] device_create fail\n");
		goto breathlight_probe_error;
	}

	if (breathlight_create_attr(breathlight_device))
		BL_ERR("create_attr fail\n");

	if (i2c_add_driver(&aw2013_i2c_driver) != 0) {
		BL_ERR("unable to add i2c driver.\n");
		return -1;
	}

	/*initialize members */
//    spin_lock_init(&breathlight_private.lock);
//    init_waitqueue_head(&breathlight_private.read_wait);
//    init_MUTEX(&breathlight_private.sem);

	BL_DBG("[breathlight_probe] Done\n");
	return 0;

      breathlight_probe_error:
#ifdef ALLOC_DEVNO
	if (err == 0)
		cdev_del(&breathlight_cdev);
	if (ret == 0)
		unregister_chrdev_region(breathlight_devno, 1);
#else
	if (ret == 0)
		unregister_chrdev(MAJOR(breathlight_devno),
				  BREATHLIGHT_DEVNAME);
#endif
	return -1;
}

static int breathlight_remove(struct platform_device *dev)
{

	BL_DBG("[breathlight_remove] start\n");

	if (breathlight_delete_attr(breathlight_device))
		BL_ERR("delete_attr fail\n");

#ifdef ALLOC_DEVNO
	cdev_del(&breathlight_cdev);
	unregister_chrdev_region(breathlight_devno, 1);
#else
	unregister_chrdev(MAJOR(breathlight_devno), BREATHLIGHT_DEVNAME);
#endif
	device_destroy(breathlight_class, breathlight_devno);
	class_destroy(breathlight_class);

	BL_DBG("[breathlight_remove] Done\n");
	return 0;
}




/* Vanzo:dingge on: Fri, 03 Apr 2015 10:33:13 +0800
 * TODO: replace this line with your comment
 */
static int breathlight_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    return 0;
}

static int breathlight_resume(struct platform_device *pdev)
{
    return 0;
}
// End of Vanzo: dingge

/* Vanzo:dingge on: Fri, 03 Apr 2015 09:43:15 +0800
 * TODO: update aw2013 driver
static struct platform_device breathlight_platform_device = {
	.name = BREATHLIGHT_DEVNAME,	
	.id = -1,
};
 */
// End of Vanzo: dingge

/* Vanzo:dingge on: Fri, 03 Apr 2015 10:32:50 +0800
 * TODO: replace this line with your comment
 */
#ifdef CONFIG_OF
static const struct of_device_id breathlight_of_match[] = {
    { .compatible = "mediatek,breathlight", },
    {},
};
#endif
// End of Vanzo: dingge

static struct platform_driver breathlight_platform_driver = {
	.probe = breathlight_probe,
/* Vanzo:dingge on: Fri, 03 Apr 2015 09:41:25 +0800
 * TODO: update aw2013 driver
 */
    .suspend = breathlight_suspend,
    .resume = breathlight_resume,
// End of Vanzo: dingge
	.remove = breathlight_remove,
	.driver = {
		   .name = BREATHLIGHT_DEVNAME,
		   .owner = THIS_MODULE,
/* Vanzo:dingge on: Fri, 03 Apr 2015 09:41:09 +0800
 * TODO: update aw2013 driver
 */
     #ifdef CONFIG_OF
     .of_match_table = breathlight_of_match,
     #endif
// End of Vanzo: dingge
		   },

};

static int __init breathlight_init(void)
{
	int ret = 0;
	BL_DBG("[breathlight_init] start\n");

	i2c_register_board_info(AW2013_I2C_BUSNUM, &i2c_aw2013, 1);
/* Vanzo:dingge on: Fri, 03 Apr 2015 09:42:58 +0800
 * TODO: update aw2013 driver
	ret = platform_device_register(&breathlight_platform_device);
	if (ret != 0) {
		BL_ERR
		    ("[breathlight_init] platform_device_register fail\n");
		return ret;
	}
 */
// End of Vanzo: dingge
	ret = platform_driver_register(&breathlight_platform_driver);
	if (ret) {
		BL_ERR
		    ("[breathlight_init] platform_driver_register fail\n");
		return ret;
	}

	BL_DBG("[breathlight_init] done!\n");
	return ret;
}

static void __exit breathlight_exit(void)
{
	BL_DBG("[breathlight_exit] start\n");
	//platform_device_unregister(&breathlight_platform_device);
	platform_driver_unregister(&breathlight_platform_driver);
	BL_DBG("[breathlight_exit] done!\n");
}

/*****************************************************************************/
module_init(breathlight_init);
module_exit(breathlight_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Arnold Lu <lubaoquan@vanzotec.com>");
MODULE_DESCRIPTION("Breath LED light");
