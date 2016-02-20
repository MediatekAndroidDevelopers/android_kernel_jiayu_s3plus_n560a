/*
Copyright (c) 2010 by ilitek Technology.
All rights reserved.

ilitek I2C touch screen driver for Android platform

Author:	  Steward Fu
Maintain:Michael Hsu 
Version: 1
History:
2010/10/26 Firstly released
2010/10/28 Combine both i2c and hid function together
2010/11/02 Support interrupt trigger for I2C interface
2010/11/10 Rearrange code and add new IOCTL command
2010/11/23 Support dynamic to change I2C address
2010/12/21 Support resume and suspend functions
2010/12/23 Fix synchronous problem when application and driver work at the same time
2010/12/28 Add erasing background before calibrating touch panel
2011/01/13 Rearrange code and add interrupt with polling method
2011/01/14 Add retry mechanism
2011/01/17 Support multi-point touch
2011/01/21 Support early suspend function
2011/02/14 Support key button function
2011/02/18 Rearrange code
2011/03/21 Fix counld not report first point
2011/03/25 Support linux 2.36.x 
2011/05/31 Added "echo dbg > /dev/ilitek_ctrl" to enable debug message
Added "echo info > /dev/ilitek_ctrl" to show tp informaiton
Added VIRTUAL_KEY_PAD to enable virtual key pad
Added CLOCK_INTERRUPT to change interrupt from Level to Edge
Changed report behavior from Interrupt to Interrupt with Polling
Added disable irq when doing firmware upgrade via APK, it needs to use APK_1.4.9
2011/06/21 Avoid button is pressed when press AA
2011/08/03 Added ilitek_i2c_calibration function
2011/08/18 Fixed multi-point tracking id
Added ROTATE_FLAG to change x-->y, y-->x
Fixed when draw line from non-AA to AA, the line will not be appeared on screen.
2011/09/29 Added Stop Polling in Interrupt mode
Fixed Multi-Touch return value
Added release last point
2011/10/26 Fixed ROTATE bug
Added release key button when finger up.
Added ilitek_i2c_calibration_status for read calibration status
2011/11/09 Fixed release last point issue
enable irq when i2c error.
2011/11/28 implement protocol 2.1.
2012/02/10 Added muti_touch key.
'a5'5b'a4'4af1'82'a2u-32738?u31720?u22841?
2012/04/02 'f8'c2'f8'50 input_report_key('f9'67'f1'4fandroid 4.0 event inputf2'd4f0'b0'b5'ad'd7f3'fd)

*/
#include "tpd.h"
#include <linux/module.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/dma-mapping.h>

#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <cust_eint.h>

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#include "cust_gpio_usage.h"
#include "ilitek.h"


//#ifdef AGOLD_CTP_ILITEK_UPGRADE
#define SET_RESET
#if 0
static unsigned char CTPM_FW[]={
#include "ILI2113A_Header.ili"
};
#endif
//#define GESTURE
	#ifdef GESTURE
	#define GESTURE_FUN_1	1 //Study gesture function
	#define GESTURE_FUN_2	2 //A function
	#define GESTURE_FUN GESTURE_FUN_1
	#define _DOUBLE_CLICK_
	#define GESTURE_H
#include "gesture_parameter.h"
#include "gesture.h"
volatile static char int_Flag;
volatile static char update_Flag;
static int update_timeout;
int gesture_flag ,gesture_count,getstatus;	
#endif

//#define IC_TYPE 2116
//#endif //AGOLD_CTP_ILITEK_UPGRADE


#define TPD_HAVE_BUTTON
#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
#endif

struct touch_vitual_key_map_t
{
   int point_x;
   int point_y;
};
static struct touch_vitual_key_map_t touch_key_point_maping_array[]=GTP_KEY_MAP_ARRAY;

#define VIRTUAL_KEY_PAD  
#define TOUCH_POINT      0x80
#define TOUCH_KEY        0xC0
#define RELEASE_KEY      0x40
#define RELEASE_POINT    0x00

/* #define TPD_POWER_SOURCE_CUSTOM PMIC_APP_CAP_TOUCH_VDD	 */
#define TPD_POWER_SOURCE		PMIC_APP_CAP_TOUCH_VDD

#define ILITEK_I2C_RETRY_COUNT			 3
#define ILITEK_I2C_DRIVER_NAME			 "ilitek_i2c"
#define ILITEK_FILE_DRIVER_NAME			 "ilitek_file"
#define ILITEK_DEBUG_LEVEL			 KERN_ALERT //KERN_INFO
#define ILITEK_ERROR_LEVEL			 KERN_ALERT

// i2c command for ilitek touch screen
#define ILITEK_TP_CMD_READ_DATA			 0x10
#define ILITEK_TP_CMD_READ_SUB_DATA		 0x11
#define ILITEK_TP_CMD_GET_RESOLUTION		 0x20
#define ILITEK_TP_CMD_GET_FIRMWARE_VERSION	 0x40
#define ILITEK_TP_CMD_GET_PROTOCOL_VERSION	 0x42
#define	ILITEK_TP_CMD_CALIBRATION		 0xCC
#define	ILITEK_TP_CMD_CALIBRATION_STATUS	 0xCD
#define ILITEK_TP_CMD_ERASE_BACKGROUND		 0xCE

// define the application command
#define ILITEK_IOCTL_BASE                       100
#define ILITEK_IOCTL_I2C_WRITE_DATA             _IOWR(ILITEK_IOCTL_BASE, 0, unsigned char*)
#define ILITEK_IOCTL_I2C_WRITE_LENGTH           _IOWR(ILITEK_IOCTL_BASE, 1, int)
#define ILITEK_IOCTL_I2C_READ_DATA              _IOWR(ILITEK_IOCTL_BASE, 2, unsigned char*)
#define ILITEK_IOCTL_I2C_READ_LENGTH            _IOWR(ILITEK_IOCTL_BASE, 3, int)
#define ILITEK_IOCTL_USB_WRITE_DATA             _IOWR(ILITEK_IOCTL_BASE, 4, unsigned char*)
#define ILITEK_IOCTL_USB_WRITE_LENGTH           _IOWR(ILITEK_IOCTL_BASE, 5, int)
#define ILITEK_IOCTL_USB_READ_DATA              _IOWR(ILITEK_IOCTL_BASE, 6, unsigned char*)
#define ILITEK_IOCTL_USB_READ_LENGTH            _IOWR(ILITEK_IOCTL_BASE, 7, int)
#define ILITEK_IOCTL_I2C_UPDATE_RESOLUTION      _IOWR(ILITEK_IOCTL_BASE, 8, int)
#define ILITEK_IOCTL_USB_UPDATE_RESOLUTION      _IOWR(ILITEK_IOCTL_BASE, 9, int)
#define ILITEK_IOCTL_I2C_SET_ADDRESS            _IOWR(ILITEK_IOCTL_BASE, 10, int)
#define ILITEK_IOCTL_I2C_UPDATE                 _IOWR(ILITEK_IOCTL_BASE, 11, int)
#define ILITEK_IOCTL_STOP_READ_DATA             _IOWR(ILITEK_IOCTL_BASE, 12, int)
#define ILITEK_IOCTL_START_READ_DATA            _IOWR(ILITEK_IOCTL_BASE, 13, int)
#define ILITEK_IOCTL_GET_INTERFANCE				_IOWR(ILITEK_IOCTL_BASE, 14, int)//default setting is i2c interface
#define ILITEK_IOCTL_I2C_SWITCH_IRQ				_IOWR(ILITEK_IOCTL_BASE, 15, int)

#ifdef GESTURE
#define ILITEK_IOCTL_UPDATE_FLAG				_IOWR(ILITEK_IOCTL_BASE, 16, int)
#define ILITEK_IOCTL_I2C_UPDATE_FW				_IOWR(ILITEK_IOCTL_BASE, 18, int)
#define ILITEK_IOCTL_I2C_GESTURE_FLAG			_IOWR(ILITEK_IOCTL_BASE, 26, int)
#define ILITEK_IOCTL_I2C_GESTURE_RETURN			_IOWR(ILITEK_IOCTL_BASE, 27, int)
#define ILITEK_IOCTL_I2C_GET_GESTURE_MODEL		_IOWR(ILITEK_IOCTL_BASE, 28, int)
#define ILITEK_IOCTL_I2C_LOAD_GESTURE_LIST		_IOWR(ILITEK_IOCTL_BASE, 29, int)
#endif
static int ilitek_i2c_register_device(void);
//static void ilitek_set_input_param(struct input_dev*, int, int, int);
static int ilitek_i2c_read_tp_info(void);
static int ilitek_init(void);
static void ilitek_exit(void);

// i2c functions
static int ilitek_i2c_transfer(struct i2c_client*, struct i2c_msg*, int);
static int ilitek_i2c_read(struct i2c_client*, uint8_t, uint8_t*, int);
static int ilitek_i2c_process_and_report(void);
static int ilitek_i2c_suspend(struct i2c_client*, pm_message_t);
static int ilitek_i2c_resume(struct i2c_client*);
//static void ilitek_i2c_shutdown(struct i2c_client*);
static int ilitek_i2c_probe(struct i2c_client*, const struct i2c_device_id*);
static int ilitek_i2c_remove(struct i2c_client*);
static void ilitek_i2c_isr(void);
static void ilitek_i2c_irq_work_queue_func(struct work_struct*);
// file operation functions
static int ilitek_file_open(struct inode*, struct file*);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
static long ilitek_file_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#else
static int  ilitek_file_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg);
#endif
static int ilitek_file_open(struct inode*, struct file*);
static ssize_t ilitek_file_write(struct file*, const char*, size_t, loff_t*);
static ssize_t ilitek_file_read(struct file*, char*, size_t, loff_t*);
static int ilitek_file_close(struct inode*, struct file*);
static void ilitek_i2c_irq_enable(void);//luca 20120120
static void ilitek_i2c_irq_disable(void);//luca 20120120
static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#ifdef AGOLD_CTP_ILITEK_UPGRADE
static int ilitek_2116_upgrade_firmware(void);
static int ilitek_2113_upgrade_firmware(void);
#endif


struct i2c_data {
	// input device
	struct input_dev *input_dev;
	// i2c client
	struct i2c_client *client;
	// polling thread
	struct task_struct *thread;        
	unsigned char firmware_ver[4];
	// maximum x
	int max_x;
	// maximum y
	int max_y;
	// maximum touch point
	int max_tp;
	// maximum key button
	int max_btn;
	// the total number of x channel
	int x_ch;
	// the total number of y channel
	int y_ch;
	// check whether i2c driver is registered success
	int valid_i2c_register;
	// check whether input driver is registered success
	int valid_input_register;
	// check whether the i2c enter suspend or not
	int stop_polling;
	// read semaphore
	struct semaphore wr_sem;
	// protocol version
	int protocol_ver;
	// valid irq request
	int valid_irq_request;
	// work queue for interrupt use only
	struct workqueue_struct *irq_work_queue;
	// work struct for work queue
	struct work_struct irq_work;
	struct timer_list timer;
	int irq_status;
	//irq_status enable:1 disable:0
	struct completion complete;

};

// device data
struct dev_data {
	// device number
	dev_t devno;
	// character device
	struct cdev cdev;
	// class device
	struct class *class;
};

// global variables
extern struct tpd_device *tpd;
static struct i2c_data i2c;
static struct dev_data dev;
static char DBG_FLAG=1;
static char Report_Flag;
static int sensitivity_state = -1;
static int boot_mode = 0;
static int power=0; 
static int hall_resume=1;
static int hall_failed = 0;
static u8 *I2CDMABuf_va = NULL;
static dma_addr_t I2CDMABuf_pa = NULL;

//static struct i2c_client *i2c_client = NULL;
static const struct i2c_device_id tpd_i2c_id[] = {{"ilitek",0},{}};
static unsigned short force[] = {0, 0x82, I2C_CLIENT_END,I2C_CLIENT_END};
static const unsigned short * const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces,};
static struct i2c_board_info __initdata i2c_tpd={ I2C_BOARD_INFO("ilitek", (0x82>>1))};

static void tpd_down(int x, int y, int p)
{
	#ifdef TPD_HAVE_BUTTON
		if(MTK_LCM_PHYSICAL_ROTATION == 270 || MTK_LCM_PHYSICAL_ROTATION == 90)
		{
			if(boot_mode!=NORMAL_BOOT && x>=TPD_RES_Y) 
			{ 
				int temp;
				temp = y;
				y = x;
				x = TPD_RES_X-temp;
				tpd_button(x, y, 1);
				return;
			}
			else if(boot_mode!=NORMAL_BOOT && y>=TPD_RES_Y)
			{
				tpd_button(x, y, 1);
				return;		
			}
		}
		else if(MTK_LCM_PHYSICAL_ROTATION == 180)
		{
			if(boot_mode!=NORMAL_BOOT && y>=TPD_RES_Y) 
			{ 
				tpd_button(LCM_WIDTH-x, LCM_HEIGHT-y, 1);
				return;
			}
		}
		else
		{
			if(boot_mode!=NORMAL_BOOT && y>=TPD_RES_Y) 
			{ 
				tpd_button(x, y, 1);
				return;
			}
		}
	#endif
	input_report_key(tpd->dev, BTN_TOUCH,  1);//Luca 20120402
	input_event(tpd->dev, EV_ABS, ABS_MT_TRACKING_ID,p);//(buf[0]&0x3f)-1
	input_event(tpd->dev, EV_ABS, ABS_MT_POSITION_X, x);
	input_event(tpd->dev, EV_ABS, ABS_MT_POSITION_Y, y);
	input_event(tpd->dev, EV_ABS, ABS_MT_TOUCH_MAJOR, 1);
	input_mt_sync(tpd->dev);
}

static void tpd_up(int x, int y,int p)
{
	#ifdef TPD_HAVE_BUTTON
		if(MTK_LCM_PHYSICAL_ROTATION == 270 || MTK_LCM_PHYSICAL_ROTATION == 90)
		{
			if(boot_mode!=NORMAL_BOOT && x>=TPD_RES_Y) 
			{ 
				int temp;
				temp = y;
				y = x;
				x = TPD_RES_X-temp;
				tpd_button(x, y, 0);
				return;
			}
			else if(boot_mode!=NORMAL_BOOT && y>=TPD_RES_Y)
			{
				tpd_button(x, y, 0);
				return;		
			}
		}
		else if(MTK_LCM_PHYSICAL_ROTATION == 180)
		{
			if(boot_mode!=NORMAL_BOOT && y>=TPD_RES_Y) 
			{ 
				tpd_button(LCM_WIDTH-x, LCM_HEIGHT-y, 0);
				return;
			}
		}
		else
		{
			if(boot_mode!=NORMAL_BOOT && y>=TPD_RES_Y) 
			{ 
				tpd_button(x, y, 0);
				return;
			}
		}
	#endif
	input_report_key(tpd->dev, BTN_TOUCH, 0);
	input_mt_sync(tpd->dev);
}

static struct i2c_driver ilitek_i2c_driver =
{                       
	.probe = ilitek_i2c_probe,                                   
	.remove = ilitek_i2c_remove,                           
	.detect = tpd_i2c_detect,                           
	.driver.name = "ilitek", 
	.id_table = tpd_i2c_id,                             
	.address_list = (const unsigned short*) forces,                        
}; 

// declare file operations
struct file_operations ilitek_fops = {
	#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
	.unlocked_ioctl = ilitek_file_ioctl,
	#else
	.ioctl = ilitek_file_ioctl,
	#endif
	.read = ilitek_file_read,
	.write = ilitek_file_write,
	.open = ilitek_file_open,
	.release = ilitek_file_close,
};


static int ilitek_file_open(struct inode *inode, struct file *filp)
{
	printk("%sn",__func__);
	return 0; 
}

static int ilitek_i2c_calibration(size_t count)
{

	int ret;
	unsigned char buffer[128]={0};
	struct i2c_msg msgs[] = {
		{.addr = i2c.client->addr, .flags = 0, .len = count, .buf = buffer,}
	};

	buffer[0] = ILITEK_TP_CMD_ERASE_BACKGROUND;
	msgs[0].len = 1;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
	if(ret < 0){
		printk(ILITEK_DEBUG_LEVEL "%s, i2c erase background, failed\n", __func__);
	}
	else{
		printk(ILITEK_DEBUG_LEVEL "%s, i2c erase background, success\n", __func__);
	}

	buffer[0] = ILITEK_TP_CMD_CALIBRATION;
	msgs[0].len = 1;
	msleep(2000);
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
	msleep(1000);
	return ret;
}

static int ilitek_i2c_calibration_status(size_t count)
{
	int ret;
	unsigned char buffer[128]={0};
	struct i2c_msg msgs[] = {
		{.addr = i2c.client->addr, .flags = 0, .len = count, .buf = buffer,}
	};
	buffer[0] = ILITEK_TP_CMD_CALIBRATION_STATUS;
	ilitek_i2c_transfer(i2c.client, msgs, 1);
	msleep(500);
	ilitek_i2c_read(i2c.client, ILITEK_TP_CMD_CALIBRATION_STATUS, buffer, 1);
	printk("%s, i2c calibration status:0x%X\n",__func__,buffer[0]);
	ret=buffer[0];
	return ret;
}

static ssize_t ilitek_file_write( struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
	int ret;
	unsigned char buffer[128]={0};

	// before sending data to touch device, we need to check whether the device is working or not
	if(i2c.valid_i2c_register == 0)
	{
		printk(ILITEK_ERROR_LEVEL "%s, i2c device driver doesn't be registered\n", __func__);
		return -1;
	}

	// check the buffer size whether it exceeds the local buffer size or not
	if(count > 128)
	{
		printk(ILITEK_ERROR_LEVEL "%s, buffer exceed 128 bytes\n", __func__);
		return -1;
	}

	// copy data from user space
	ret = copy_from_user(buffer, buf, count-1);
	if(ret < 0)
	{
		printk(ILITEK_ERROR_LEVEL "%s, copy data from user space, failed\n", __func__);
		return -1;
	}

	// parsing command
	if(strcmp(buffer, "calibrate") == 0)
	{
		ret=ilitek_i2c_calibration(count);
		if(ret < 0){
			printk(ILITEK_DEBUG_LEVEL "%s, i2c send calibration command, failed\n", __func__);
		}
		else{
			printk(ILITEK_DEBUG_LEVEL "%s, i2c send calibration command, success\n", __func__);
		}
		ret=ilitek_i2c_calibration_status(count);
		if(ret == 0x5A){
			printk(ILITEK_DEBUG_LEVEL "%s, i2c calibration, success\n", __func__);
		}
		else if (ret == 0xA5){
			printk(ILITEK_DEBUG_LEVEL "%s, i2c calibration, failed\n", __func__);
		}
		else{
			printk(ILITEK_DEBUG_LEVEL "%s, i2c calibration, i2c protoco failed\n", __func__);
		}
		return count;
	}else if(strcmp(buffer, "dbg") == 0){
		DBG_FLAG=!DBG_FLAG;
		printk("%s, %s message(%X).\n",__func__,DBG_FLAG?"Enabled":"Disabled",DBG_FLAG);
	}else if(strcmp(buffer, "info") == 0){
		ilitek_i2c_read_tp_info();
	}else if(strcmp(buffer, "report") == 0){
		Report_Flag=!Report_Flag;
	}
	#ifdef GESTURE
	else if(strcmp(buffer, "readgesturelist") ==0)
	{
	  readgesturelist();
	}
	#endif
	return -1;
}


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)
static long ilitek_file_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
#else
static int  ilitek_file_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
#endif
{
	static unsigned char buffer[64]={0};
	static int len=0,i;
	int ret = 0;	
	struct i2c_msg msgs[] = {
		{.addr = i2c.client->addr, .flags = 0, .len = len, .buf = buffer,}
	};

	// parsing ioctl command
	switch(cmd)
	{
		case ILITEK_IOCTL_I2C_WRITE_DATA:
			ret = copy_from_user(buffer, (unsigned char*)arg, len);
			if(ret < 0)
			{
				printk(ILITEK_ERROR_LEVEL "%s, copy data from user space, failed\n", __func__);
				return -1;
			}
			ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
			if(ret < 0)
			{
				printk(ILITEK_ERROR_LEVEL "%s, i2c write, failed\n", __func__);
				return -1;
			}
		break;
		case ILITEK_IOCTL_I2C_READ_DATA:
			msgs[0].flags = I2C_M_RD;

			ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
			if(ret < 0)
			{
				printk(ILITEK_ERROR_LEVEL "%s, i2c read, failed\n", __func__);
				return -1;
			}
			ret = copy_to_user((unsigned char*)arg, buffer, len);

			if(ret < 0)
			{
				printk(ILITEK_ERROR_LEVEL "%s, copy data to user space, failed\n", __func__);
				return -1;
			}
		break;
		case ILITEK_IOCTL_I2C_WRITE_LENGTH:
		case ILITEK_IOCTL_I2C_READ_LENGTH:
			len = arg;
		break;
		case ILITEK_IOCTL_I2C_UPDATE_RESOLUTION:
		case ILITEK_IOCTL_I2C_SET_ADDRESS:
		case ILITEK_IOCTL_I2C_UPDATE:
			break;
		case ILITEK_IOCTL_START_READ_DATA:
			i2c.stop_polling = 0;
			if(i2c.client->irq != 0 )
			ilitek_i2c_irq_enable();
		break;
		case ILITEK_IOCTL_STOP_READ_DATA:
			i2c.stop_polling = 1;
			if(i2c.client->irq != 0 )
			ilitek_i2c_irq_disable();
		break;
		case ILITEK_IOCTL_I2C_SWITCH_IRQ:
			ret = copy_from_user(buffer, (unsigned char*)arg, 1);
			if (buffer[0]==0)
			{
				if(i2c.client->irq != 0 )
				{
					ilitek_i2c_irq_disable();
				}
			}
			else
			{
				if(i2c.client->irq != 0 )
				{
					ilitek_i2c_irq_enable();				
				}
			}
		break;
	#ifdef GESTURE
		case ILITEK_IOCTL_UPDATE_FLAG:
			update_timeout = 1;
			update_Flag = arg;
			DBG("%s,update_Flag=%d\n",__func__,update_Flag);
			break;
		case ILITEK_IOCTL_I2C_UPDATE_FW:
			ret = copy_from_user(buffer, (unsigned char*)arg, 35);
			if(ret < 0){
				printk(ILITEK_ERROR_LEVEL "%s, copy data from user space, failed\n", __func__);
				return -1;
			}
			int_Flag = 0;
			update_timeout = 0;
			msgs[0].len = buffer[34];
			ret = ilitek_i2c_transfer(i2c.client, msgs, 1);	
			#ifndef CLOCK_INTERRUPT
			ilitek_i2c_irq_enable();
			#endif
			if(ret < 0){
				printk(ILITEK_ERROR_LEVEL "%s, i2c write, failed\n", __func__);
				return -1;
			}
			break;
		case ILITEK_IOCTL_I2C_GESTURE_FLAG:
			gesture_flag = arg;
			printk("%s,gesture_flag=%d\n",__func__,gesture_flag);
			break;
		case ILITEK_IOCTL_I2C_GESTURE_RETURN:
			buffer[0] = getstatus;
			printk("%s,getstatus=%d\n",__func__,getstatus);
			ret = copy_to_user((unsigned char*)arg, buffer, 1);
			if(ret < 0){
				printk(ILITEK_ERROR_LEVEL "%s, copy data to user space, failed\n", __func__);
				return -1;
			}
			//getstatus = 0;
			break;
			#if GESTURE_FUN==GESTURE_FUN_1
		case ILITEK_IOCTL_I2C_GET_GESTURE_MODEL:
			for(i = 0; i<32 ;i=i+2){
				buffer[i] = gesture_model_value_x(i/2);
				buffer[i+1]= gesture_model_value_y(i/2);
				printk("x[%d]=%d,y[%d]=%d\n",i/2,buffer[i],i/2,buffer[i+1]);
			}
			ret = copy_to_user((unsigned char*)arg, buffer, 32);
			if(ret < 0){
				printk(ILITEK_ERROR_LEVEL "%s, copy data to user space, failed\n", __func__);
				return -1;
			}
			//getstatus = 0;
			break;
		case ILITEK_IOCTL_I2C_LOAD_GESTURE_LIST:
			printk("start\n");
			readgesturelist();
			printk("end--------------\n");
			break;
			#endif	
	#endif			
		default:
		return -1;
	}
	return 0;
}


static ssize_t ilitek_file_read( struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	return 0;
}


static int ilitek_file_close( struct inode *inode, struct file *filp)
{
	printk("%s\n",__func__);
	return 0;
}



static int ilitek_dma_i2c_read(struct i2c_client *client, unsigned char *buf, int len)
{
	int i = 0, err = 0;

	if(len < 8)
	{
		client->addr = client->addr & I2C_MASK_FLAG;
		return i2c_master_recv(client, buf, len);
	}
	else
	{
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
		err = i2c_master_recv(client, (u8 *)I2CDMABuf_pa, len);

		if(err < 0)
		{
			return err;
		}

		for(i = 0; i < len; i++)
		{
			buf[i] = I2CDMABuf_va[i];
		}
	}
}



static int ilitek_dma_i2c_write(struct i2c_client *client, unsigned char *pbt_buf, int dw_len)
{
	int i = 0;
	for(i = 0 ; i < dw_len; i++)
	{
		I2CDMABuf_va[i] = pbt_buf[i];
	}

	if(dw_len <= 8)
	{
		client->addr = client->addr & I2C_MASK_FLAG;
		return i2c_master_send(client, pbt_buf, dw_len);
	}
	else
	{
		client->addr = client->addr & I2C_MASK_FLAG | I2C_DMA_FLAG;
		return i2c_master_send(client, (u8 *)I2CDMABuf_pa, dw_len);
	}
}


static int ilitek_i2c_transfer(struct i2c_client *client, struct i2c_msg *msgs, int cnt)
{
	int i,ret=0, count=ILITEK_I2C_RETRY_COUNT;
	for(i=0;i<cnt;i++)
	{
		if(msgs[i].len <= 8)
		{
			msgs[i].addr &= I2C_MASK_FLAG;
			msgs[i].timing = 400;		 
			msgs[i].ext_flag = 0;  
			while(count >= 0)
			{
				count-= 1;
				ret = i2c_transfer(client->adapter, &msgs[i], 1);
				if((count == 0)&&(ret<0))
				{
					printk("%s i2c transfer <=8 bytes error",__func__);
					return ret;
				}
				if(ret < 0)
				{				  
					msleep(500);
					continue;
				}
				break;
			}
		}	 
		else
		{
			msgs[i].ext_flag = 0;
			if(msgs[i].flags == I2C_M_RD)				
			ret = ilitek_dma_i2c_read(client,msgs[i].buf,msgs[i].len);
			else if(msgs[i].flags == 0)
			ret = ilitek_dma_i2c_write(client,msgs[i].buf,msgs[i].len); 		   
			if(ret<0)
				printk("%s i2c transfer >8 bytes error",__func__);					  
		}		 
	}
	return ret;
}


static int ilitek_i2c_read( struct i2c_client *client, uint8_t cmd,  uint8_t *data,  int length)
{
int ret;
struct i2c_msg msgs[] = {
{.addr = client->addr, .flags = 0, .len = 1, .buf = &cmd,},
{.addr = client->addr, .flags = I2C_M_RD, .len = length, .buf = data,}
};

ret = ilitek_i2c_transfer(client, msgs, 2);
if(ret < 0){
printk(ILITEK_ERROR_LEVEL "%s, i2c read error, ret %d,addr %x \n", __func__, ret,client->addr);
}
return ret;
}


static int ilitek_i2c_process_and_report(void)
{
	int i, len, ret, x=0, y=0,key,org_x=0, org_y=0,mult_tp_id,j;
	unsigned char key_id=0,key_flag=1;
	static unsigned char last_id=0;
     #ifdef GESTURE
	int gesture_mode=0;
	 unsigned char release_counter = 0;
     #endif		
     struct input_dev *input = i2c.input_dev;
	unsigned char buf[9]={0};
	unsigned char tp_id;
	int key_x=0,key_y=0;
	// read i2c data from device

	ret = ilitek_i2c_read(i2c.client, ILITEK_TP_CMD_READ_DATA, buf, 8);
	if(ret < 0){
		return ret;
	}

	// multipoint process
	if(i2c.protocol_ver & 0x200)
	{
		len = buf[0];
		ret = 1;
		if(len > 16){
			return ret;
		}
		printk("[mcz]len = %d\n",len);
		// read touch point
		for(i=0; i<len; i++)
		{
			// parse point

			if(ilitek_i2c_read(i2c.client, ILITEK_TP_CMD_READ_SUB_DATA, buf, 5))
			{
				x = (((int)buf[1]) << 8) + buf[2];
				y = (((int)buf[3]) << 8) + buf[4];
				//y = i2c.max_y - y;
				//printk("\n###################### x= %d, y = %d ######\n", x , y);

				mult_tp_id = buf[0];

				switch ((mult_tp_id & 0xC0))
				{
					#ifdef VIRTUAL_KEY_PAD
					case RELEASE_KEY:
						//release key
						//printk("Key: Release\n");
						if(boot_mode == NORMAL_BOOT)
						{
							input_report_key(input, BTN_TOUCH,  0);//mcz20140111
							input_mt_sync(tpd->dev);
						}
						else
						{      
							input_report_key(tpd->dev, BTN_TOUCH, 0);
							tpd_button(key_x, key_y, 0);
							//input_mt_sync(tpd->dev);
						}
					break;

					case TOUCH_KEY:

						key_id = buf[1] - 1;
						//printk("Presskey:key_id=%x \n",key_id);

						if(key_id<0||key_id>3)break;
                                          key_x = touch_key_point_maping_array[key_id].point_x;
						key_y = touch_key_point_maping_array[key_id].point_y;

						if(boot_mode == NORMAL_BOOT)
						{
						//	printk("[mcz]key_x = %d,key_y = %d\n",key_x,key_y);							
							input_report_key(input, BTN_TOUCH,  1);
							input_event(input, EV_ABS, ABS_MT_TRACKING_ID,11);
							input_event(input, EV_ABS, ABS_MT_POSITION_X, key_x);
							input_event(input, EV_ABS, ABS_MT_POSITION_Y, key_y);
							input_event(input, EV_ABS, ABS_MT_TOUCH_MAJOR, 1);
							input_mt_sync(input);
						}
						else
						{       
							tpd_button(key_x, key_y, 1);						        
						}
					break;

					#endif	
					case TOUCH_POINT:
					{		
			#ifdef GESTURE
						if(gesture_flag && i == 0){
						#if GESTURE_FUN == GESTURE_FUN_1	
							gesture_mode = GestureMatchProcess(1,gesture_count,x,y);
							gesture_count++;
						#endif
						#if GESTURE_FUN == GESTURE_FUN_2
							gesture_ili(x, y,gesture_count);
						#endif
							if(gesture_count == 0)
								gesture_count++;
						}
			   #endif						
			
								tpd_down(x,y,((buf[0]&0x3f)-1));
					}
					break;

					case RELEASE_POINT:
						
                                   #ifdef GESTURE
					release_counter++;					
						if (release_counter == len)
						   {
							if(gesture_flag )
							{
								gesture_count = 0;
								getstatus = 0;
								
								#if GESTURE_FUN == GESTURE_FUN_1
								gesture_mode = GestureMatchProcess(0,gesture_count,x,y);
								getstatus = GetGesture();
								switch(getstatus)
								{
								  case 'a':
								  	             keycode = KEY_A;
                                                                            printk("Input Gesture 'C'\n");
                                                                            break;
								  case 'b':
								  	             keycode = KEY_B;
                                                                            printk("Input Gesture 'C'\n");
                                                                            break;
								  case 'c':
								  	             keycode = KEY_C;
                                                                            printk("Input Gesture 'C'\n");
                                                                            break;
								  case 'd':
								  	             keycode = KEY_D;
                                                                            printk("Input Gesture 'C'\n");
                                                                            break;
								  case 'e':
								  	             keycode = KEY_E;
                                                                            printk("Input Gesture 'C'\n");
                                                                            break;
								  case 'm':
								  	             keycode = KEY_M;
                                                                            printk("Input Gesture 'C'\n");
                                                                            break;
								  default:     break;
								  	
							       }
							            input_report_key(tpd->dev, keycode, 1);
							            input_sync(tpd->dev);
							            input_report_key(tpd->dev, keycode, 0);
							            input_sync(tpd->dev);								
								#endif
								#if GESTURE_FUN == GESTURE_FUN_2
								getstatus = gesture_ili(x, y,2);
								#endif
							   }	
							}
						#endif
						break;

					default:
					break;
				}
			}
		}
		// release point
		if(len == 0){
			printk("Release again");
			if(boot_mode==NORMAL_BOOT)
			{
				input_report_key(input, BTN_TOUCH,  0);
				input_mt_sync(input);
			}
			#ifdef TPD_HAVE_BUTTON
			else if(tpd->btn_state) 
			{ 
				tpd_button(0, 0, 0);
			}
			#endif
		}//len就是点的个数，len较大说明有防止丢点机制，这里是没点的时候的保险措施。
		input_sync(input);
	}
	return 0;
}

static void ilitek_i2c_irq_work_queue_func( struct work_struct *work)
{
	int ret;

	struct i2c_data *priv = container_of(work, struct i2c_data, irq_work);
	ret = ilitek_i2c_process_and_report();
	ilitek_i2c_irq_enable();


}


static void ilitek_i2c_isr(void)
{
      // printk("%s,enter\n",__func__);
	if(i2c.irq_status ==1){
	//disable_irq_nosync(i2c.client->irq);
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	//printk("intrruput happend\n");
	i2c.irq_status = 0;
	}
	queue_work(i2c.irq_work_queue, &i2c.irq_work);

}

static void ilitek_i2c_irq_enable(void)
{
	if (i2c.irq_status == 0)
	{
		i2c.irq_status = 1;//for apk
		//enable_irq(i2c.client->irq);
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		printk("enable\n");
	}

}

static void ilitek_i2c_irq_disable(void)
{
	if (i2c.irq_status == 1){
		i2c.irq_status = 0;
		//disable_irq(i2c.client->irq);
		mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
		printk("disablen");
	}
}

static int ilitek_i2c_suspend( struct i2c_client *client, pm_message_t mesg)
{

	int ret;
	unsigned char buffer=0;
	#ifdef GESTURE
	gesture_flag = 1;
      #else
	
	hall_resume=0;
		struct i2c_msg msgs = {.addr = i2c.client->addr, .flags = 0, .len = 0, .buf = &buffer,};
		buffer = 0x30;
		msgs.len = 1;
		ret = ilitek_i2c_transfer(i2c.client, &msgs, 1);
		mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
		/* if(ret<0)  */
		/* { */
		/* 	hwPowerDown(TPD_POWER_SOURCE,"TP"); */
		/* 	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO); */
		/* 	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT); */
		/* 	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO); */
		/* 	power=1; */
		/* 	printk("suspend by poweroff!!"); */
		/* } */
		/* else  */
		/* 	printk("suspend by change register!!"); */
        #endif
	return 0;
}


static int ilitek_i2c_resume(struct i2c_client *client)
{ 
	char buffer=0;
	//wake_up_interruptible(&ilitek_waiter);
#ifdef GESTURE
	gesture_flag = 0;
#else		
	/* if(power==1) */
	/* { */
	/* 	hwPowerOn(TPD_POWER_SOURCE, VOL_2800, "TP"); */
	/* 	msleep(50); */
	/* 	power=0;   */
	/* 	printk("resume by poweron!!");     */
	/* } */
	/* //reset */

	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(20);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(20);
	hall_resume=1;
	#if defined(AGOLD_CTP_HALL_SENSITIVITY)
	if(hall_failed == 1)
	{    
		hall_failed = 0;
		int state = 0;
		int state_compare = 0;
		int i = 0;
		if(1 == sensitivity_state)//hall close 
		{
			state = 0xB0;
			printk("whl APK sendmsg close in resume!");
		}
		else if(0 == sensitivity_state)//hall far
		{	
			state = 0xB1;
			printk("whl APK sendmsg far in resume!");		
		}
		mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM); 
		struct i2c_msg msg[] = 
		{
			{
			.addr	= i2c.client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &state,
			},
		};
		//send command 2 times
		i2c_transfer(i2c.client->adapter, msg, 1);
		printk("whl msg send succes in resume !");
		msleep(50);
		i2c_transfer(i2c.client->adapter, msg, 1);
		//ilitek_i2c_read(i2c.client, 0xb3, &buffer, 1);
		//printk("whl buffer in resyme=%x\n !",buffer);
		msleep(50);

	}
	#endif

	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
       #endif
	return 0;
}



static int ilitek_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{

	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)){
		printk(ILITEK_ERROR_LEVEL "%s, I2C_FUNC_I2C not support\n", __func__);
		return -1;
	}
	i2c.client = client;
       ilitek_i2c_register_device();
	input_set_capability(tpd->dev, EV_KEY, KEY_HOME);
	input_set_capability(tpd->dev, EV_KEY, KEY_BACK);
	input_set_capability(tpd->dev, EV_KEY, KEY_MENU);
	#ifdef GESTURE
	input_set_capability(tpd->dev, EV_KEY, KEY_A);
	input_set_capability(tpd->dev, EV_KEY, KEY_B);
	input_set_capability(tpd->dev, EV_KEY, KEY_C);
	input_set_capability(tpd->dev, EV_KEY, KEY_D);
	input_set_capability(tpd->dev, EV_KEY, KEY_E);
	input_set_capability(tpd->dev, EV_KEY, KEY_M);
	#endif
	return 0;
}

static int ilitek_i2c_remove( struct i2c_client *client)
{
	printk(ILITEK_DEBUG_LEVEL "%s\n", __func__);
	i2c.stop_polling = 1;
	return 0;
}


static int ilitek_i2c_read_info(struct i2c_client *client,uint8_t cmd,  uint8_t *data,  int length)
{
	int ret;
	struct i2c_msg msgs_cmd[] = 
	{
		{.addr = client->addr, .flags = 0, .len = 1, .buf = &cmd,},
	};

	struct i2c_msg msgs_ret[] = 
	{
		{.addr = client->addr, .flags = I2C_M_RD, .len = length, .buf = data,}
	};
	printk("iic address =%x \n",client->addr);
	ret = ilitek_i2c_transfer(client, msgs_cmd, 1);
	if(ret < 0)
	{
		printk(ILITEK_ERROR_LEVEL "%s, i2c read error, ret %d,addr %x \n", __func__, ret,client->addr);
		return -1;
	}

	msleep(10);
	ret = ilitek_i2c_transfer(client, msgs_ret, 1);
	if(ret < 0)
	{
		printk(ILITEK_ERROR_LEVEL "%s, i2c read error, ret %d,addr %x \n", __func__, ret,client->addr);
		return -1;
	}
	return ret;
}


static int ilitek_i2c_read_tp_info( void)
{
	int res_len,i;
	unsigned char buf[32]={0};

	// read firmware version
	if(ilitek_i2c_read_info(i2c.client, ILITEK_TP_CMD_GET_FIRMWARE_VERSION, buf, 4) < 0){
		tpd_load_status = -1;
		return -1;
	}//读固件版本号

	for(i = 0;i<4;i++)  i2c.firmware_ver[i] = buf[i];
	printk(ILITEK_DEBUG_LEVEL "%s, firmware version %d.%d.%d.%d\n", __func__, buf[0], buf[1], buf[2], buf[3]);

	// read protocol version
	res_len = 6;
	if(ilitek_i2c_read_info(i2c.client, ILITEK_TP_CMD_GET_PROTOCOL_VERSION, buf, 2) < 0){
		return -1;
	}//读协议类型	
	i2c.protocol_ver = (((int)buf[0]) << 8) + buf[1];
	printk(ILITEK_DEBUG_LEVEL "%s, protocol version: %d.%d\n", __func__, buf[0], buf[1]);
	if(i2c.protocol_ver >= 0x200){
		res_len = 8;
	}

	// read touch resolution
	i2c.max_tp = 2;
	if(ilitek_i2c_read_info(i2c.client, ILITEK_TP_CMD_GET_RESOLUTION, buf, res_len) < 0){
		return -1;
	}//读最大分辨率
	if(i2c.protocol_ver >= 0x200){
		// maximum touch point
		i2c.max_tp = buf[6];
		// maximum button number
		i2c.max_btn = buf[7];
	}

	i2c.max_y = buf[0];
	i2c.max_y+= ((int)buf[1]) * 256;
	i2c.max_x = buf[2];
	i2c.max_x+= ((int)buf[3]) * 256;
	i2c.x_ch = buf[4];
	i2c.y_ch = buf[5];
	printk(ILITEK_DEBUG_LEVEL "%s, max_x: %d, max_y: %d, ch_x: %d, ch_y: %d\n", 
	__func__, i2c.max_x, i2c.max_y, i2c.x_ch, i2c.y_ch);
	printk(ILITEK_DEBUG_LEVEL "%s, max_tp: %d, max_btn: %d\n", __func__, i2c.max_tp, i2c.max_btn);
	return 0;
}


//#ifdef AGOLD_CTP_ILITEK_UPGRADE
static int ilitek_i2c_reset(void)
{
	int ret = 0;
	#ifndef SET_RESET
	static unsigned char buffer[64]={0};
	struct i2c_msg msgs[] = {
		{.addr = i2c.client->addr, .flags = 0, .len = 1, .buf = buffer,}
    };
	buffer[0] = 0x60;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
	#else
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
		msleep(50);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
		msleep(100);
	#endif
	msleep(100);
	return ret; 
}
static int 
ilitek_i2c_only_read(
	struct i2c_client *client,
	uint8_t *data, 
	int length)
{
	int ret;
    struct i2c_msg msgs[] = {
		{.addr = client->addr, .flags = I2C_M_RD, .len = length, .buf = data,}
    };
    ret = ilitek_i2c_transfer(client, msgs, 1);
	if(ret < 0){
		printk(ILITEK_ERROR_LEVEL "%s, i2c read error, ret %d\n", __func__, ret);
	}
	return ret;
}

static int 
ilitek_i2c_only_write(
	struct i2c_client *client,
	uint8_t *data, 
	int length)
{
	int ret;
    struct i2c_msg msgs[] = {
		{.addr = client->addr, .flags = 0, .len = length, .buf = data,}
    };

    ret = ilitek_i2c_transfer(client, msgs, 1);
	if(ret < 0){
		printk(ILITEK_ERROR_LEVEL "%s, i2c write error, ret %d\n", __func__, ret);
	}
	return ret;
}
static int inwrite(unsigned int address) {
	char outbuff[64];
	int data, ret;
	outbuff[0] = 0x25;
	outbuff[1] = (char)((address & 0x000000FF) >> 0);
	outbuff[2] = (char)((address & 0x0000FF00) >> 8);
	outbuff[3] = (char)((address & 0x00FF0000) >> 16);
	ret = ilitek_i2c_only_write(i2c.client, outbuff, 4);
	ret = ilitek_i2c_only_read(i2c.client, outbuff, 4);
	data = (outbuff[0] + outbuff[1] * 256 + outbuff[2] * 256 * 256 + outbuff[3] * 256 * 256 * 256);
	printk("%s, data=0x%x, outbuff[0]=%x, outbuff[1]=%x, outbuff[2]=%x, outbuff[3]=%x\n", __func__, data,outbuff[0], outbuff[1], outbuff[2], outbuff[3]);
	return data;
}

static int outwrite(unsigned int address, unsigned int data) {
	int ret;
	char outbuff[64];
	outbuff[0] = 0x25;
	outbuff[1] = (char)((address & 0x000000FF) >> 0);
	outbuff[2] = (char)((address & 0x0000FF00) >> 8);
	outbuff[3] = (char)((address & 0x00FF0000) >> 16);
	outbuff[4] = (char)((data & 0x000000FF) >> 0);
	outbuff[5] = (char)((data & 0x0000FF00) >> 8);
	outbuff[6] = (char)((data & 0x00FF0000) >> 16);
	outbuff[7] = (char)((data & 0xFF000000) >> 24);
	ret = ilitek_i2c_only_write(i2c.client, outbuff, 8);
	return 0;   
}

static void clear_program_key(void) {
	int ret;
	char buf[64];
	buf[0] = 0X25;
	buf[1] = 0X14;
	buf[2] = 0X10;
	buf[3] = 0X04;
	buf[4] = 0x00;
	buf[5] = 0x00;
	ret = ilitek_i2c_only_write(i2c.client, buf, 6);
}

static void set_standby_key(unsigned int chip_id_H, unsigned int chip_id_L) {
	int ret;
	char buf[64];
	//Set StandBy Key
	buf[0] = 0x25;
	buf[1] = 0x10;
	buf[2] = 0x00;
	buf[3] = 0x04;
	buf[4] = chip_id_L;
	buf[5] = chip_id_H;
	ret = ilitek_i2c_only_write(i2c.client, buf, 6);
}

static void mtp_control_reg(int data) {
	char buf[64];
	int i, ret;
	buf[0] = 0x0;
	for(i = 0; i < 500; i++) {
		if(buf[0] == 0) {
			buf[0] = 0x25;
			buf[1] = data;
			buf[2] = 0x10;
			buf[3] = 0x04;
			ret = ilitek_i2c_only_write(i2c.client, buf, 4);

			ret = ilitek_i2c_only_read(i2c.client, buf, 4);

			msleep(1);
		} else {
			break;
		}
	}

	if(i == 500) {
		printk("%s,mtp_control_reg\n",__func__);
	} else {
		outwrite(0x41030, 0x00000000);
	}


}

static void set_standby(int data) {
	int ret;
	char buf[64];
	buf[0] = 0x25;
	buf[1] = 0x24;
	buf[2] = 0x00;
	buf[3] = 0x04;
	buf[4] = 0x01;
	ret = ilitek_i2c_only_write(i2c.client, buf, 5);
	mtp_control_reg(data);
}
#if 0
/**********************************************************begin*******************************************************************/
#if (IC_TYPE==2116)
static int ilitek_2116_upgrade_firmware(void)
{
	int ret=0,upgrade_status=0,i,j,k = 0,ap_len = 0,df_len = 0,ic_model = 0,ucCount = 0;
	unsigned char buf[128]={0},chip_id_H = 0,chip_id_L = 0;
	unsigned long ap_startaddr,df_startaddr,ap_endaddr,df_endaddr,ap_checksum = 0,df_checksum = 0,ice_checksum = 0,temp = 0;
	unsigned char firmware_ver[4];
	unsigned int  bl_ver = 0,flow_flag = 0,uiData,usStart,usEnd,usSize;
	struct i2c_msg msgs[] = {
		{.addr = i2c.client->addr, .flags = 0, .len = 0, .buf = buf,}
	};
	ap_startaddr = ( CTPM_FW[0] << 16 ) + ( CTPM_FW[1] << 8 ) + CTPM_FW[2];
	ap_endaddr = ( CTPM_FW[3] << 16 ) + ( CTPM_FW[4] << 8 ) + CTPM_FW[5];
	ap_checksum = ( CTPM_FW[6] << 16 ) + ( CTPM_FW[7] << 8 ) + CTPM_FW[8];
	firmware_ver[0] = CTPM_FW[18];
	firmware_ver[1] = CTPM_FW[19];
	firmware_ver[2] = CTPM_FW[20];
	firmware_ver[3] = CTPM_FW[21];
	//printk("start=0x%06x,end=0x%06x,checksum=0x%06x\n",ap_startaddr,ap_endaddr,ap_checksum);
	ilitek_i2c_reset();
	msleep(100);
	buf[0]=0xF2;
	buf[1]=0x01;
	msgs[0].len = 2;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
	msleep(10);
	buf[0]=0x61;
	msgs[0].len = 1;
	ilitek_i2c_read(i2c.client, 0x61, buf, 5);
	if(buf[0]==0x07 && buf[1] == 0x00) {
		chip_id_H = 0x21;
		chip_id_L = 0x15;
		ic_model = (chip_id_H << 8) + chip_id_L;
		//printk("IC is 0x%4X\n",ic_model);
	}
	if(buf[0]==0x08 && buf[1] == 0x00) {
		chip_id_H = 0x21;
		chip_id_L = 0x16;
		ic_model = (chip_id_H << 8) + chip_id_L;
		//printk("IC is 0x%4X\n",ic_model);
	}
	if((buf[0]==0xFF && buf[1] == 0xFF)||(buf[0]==0x00 && buf[1] == 0x00)){
		//printk("IC is NULL\n");
	}

	buf[0] = 0x25;
	buf[1] = 0x62;
	buf[2] = 0x10;
	buf[3] = 0x18;
	msgs[0].len = 4;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
	
	buf[0] = 0x25;
	buf[1] = 0x9b;
	buf[2] = 0x00;
	buf[3] = 0x04;
	msgs[0].len = 4;
	ilitek_i2c_only_write(i2c.client, buf, 4);
	ilitek_i2c_only_read(i2c.client, buf, 1);
	
	if(buf[0] == 0x01 || buf[0] == 0x02 || buf[0] == 0x03 || buf[0] == 0x04 || buf[0] == 0x11 || buf[0] == 0x12 || buf[0] == 0x13 || buf[0] == 0x14) {
		//printk("%s, ic is 2115\n", __func__);
		chip_id_H = 0x21;
		chip_id_L = 0x15;
		ic_model = (chip_id_H << 8) + chip_id_L;
	}
	if(buf[0] == 0x81 || buf[0] == 0x82 || buf[0] == 0x83 || buf[0] == 0x84 || buf[0] == 0x91 || buf[0] == 0x92 || buf[0] == 0x93 || buf[0] == 0x94) {
		//printk("%s, ic is 2116\n", __func__);
		chip_id_H = 0x21;
		chip_id_L = 0x16;
		ic_model = (chip_id_H << 8) + chip_id_L;
	}
	//printk("ic=0x%04x\n",ic_model);
	
	buf[0] = 0x25;
	buf[1] = 0x00;
	buf[2] = 0x20;
	buf[3] = 0x04;
	buf[4] = 0x27;
	msgs[0].len = 5;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
	
	buf[0] = 0x25;
	buf[1] = 0x10;
	buf[2] = 0x00;
	buf[3] = 0x04;
	buf[4] = chip_id_L;
	buf[5] = chip_id_H;
	msgs[0].len = 6;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
	
	uiData = inwrite(0x42008);
	
	buf[0] = 0x25;
	buf[1] = 0x08;
	buf[2] = 0x20;
	buf[3] = 0x04;
	buf[4] = ((unsigned char)((uiData & 0x000000FF) >> 0));
	buf[5] = ((unsigned char)((uiData & 0x0000FF00) >> 8));
	buf[6] = ((unsigned char)((uiData & 0x00FF0000) >> 16));
	buf[7] = ((unsigned char)((uiData & 0xFF000000) >> 24) | 4);
	msgs[0].len = 8;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
	
	buf[0] = 0x25;
	buf[1] = 0x24;
	buf[2] = 0x00;
	buf[3] = 0x04;
	buf[4] = 0x01;
	msgs[0].len = 5;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
	
	buf[0] = 0x25;
	buf[1] = 0x00;
	buf[2] = 0x20;
	buf[3] = 0x04;
	buf[4] = 0x00;	 
	msgs[0].len = 5;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
	clear_program_key();
	
	set_standby_key(chip_id_H, chip_id_L);
	//3.set preprogram
	//3-1 set program key
	buf[0] = 0X25;
	buf[1] = 0X14;
	buf[2] = 0X10;
	buf[3] = 0X04;
	buf[4] = 0xAE;
	buf[5] = 0x7E;
	msgs[0].len = 6;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
	//3-2 set standby key
	set_standby_key(chip_id_H, chip_id_L);
	//
	buf[0] = 0x25;
	buf[1] = 0x00;
	buf[2] = 0x20;
	buf[3] = 0x04;
	buf[4] = 0x27;
	msgs[0].len = 5;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
	//Setting program start address and size (0x41018)
	usEnd = usEnd / 16;
	usSize = usEnd - usStart;
	//printk("usSize=0x%x",usSize);
	buf[0] = 0X25;
	buf[1] = 0X18;
	buf[2] = 0X10;
	buf[3] = 0X04;
	buf[4] = (char)((0x0000 & 0x00FF) >> 0);
	buf[5] = (char)((0x0000 & 0xFF00) >> 8);
	buf[6] = 0xEF;
	buf[7] = 0x07;
	msgs[0].len = 8;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
	//d.Setting pre-program data (0x4101c)
	buf[0] = 0X25;
	buf[1] = 0X1C;
	buf[2] = 0X10;
	buf[3] = 0X04;
	buf[4] = 0xFF;
	buf[5] = 0xFF;
	buf[6] = 0xFF;
	buf[7] = 0xFF;
	msgs[0].len = 8;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
	//e.Enable pre-program (0x41032)
	buf[0] = 0X25;
	buf[1] = 0X32;
	buf[2] = 0X10;
	buf[3] = 0X04;
	buf[4] = 0XB6;
	msgs[0].len = 5;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
	//f.Enable standby (0x40024)
	//g.Wait program time (260us*size) or check MTP_busy (0x4102c)
	set_standby(0x32);
	//h.Clear program key (0x41014)
	clear_program_key();
	//4. Set chip erase
	for(i = 0; i < 3; i++) {
		buf[0] = 0X25;
		buf[1] = 0X24;
		buf[2] = 0X10;
		buf[3] = 0X04;
		buf[4] = (char)((0x7FFF & 0x00FF) >> 0);
		buf[5] = (char)((0x7FFF & 0xFF00) >> 8);
		msgs[0].len = 6;
		ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
		//printk("%s, a.Setting lock page (0x41024), jni_write_data, ret = %d\n", __func__, ret);
		temp = inwrite(0x41024);
		//printk("temp = 0x%08x\n",temp);
		if((temp | 0x00FF) == 0x7FFF) {
			break;
		}
	}
	//b.Setting chip erase key (0x41010)
	buf[0] = 0X25;
	buf[1] = 0X10;
	buf[2] = 0X10;
	buf[3] = 0X04;
	buf[4] = 0XAE;
	buf[5] = 0XCE;
	buf[6] = 0X00;
	buf[7] = 0X00;
	msgs[0].len = 8;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);	
	//printk("%s, b.Setting chip erase key (0x41010), jni_write_data, ret = %d\n", __func__, ret);
	//c.Setting standby key (0x40010)
	//printk("%s, c.Setting standby key (0x40010)\n", __func__);
	set_standby_key(chip_id_H,chip_id_L);

	//d.Enable chip erase (0x41030)
	buf[0] = 0X25;
	buf[1] = 0X30;
	buf[2] = 0X10;
	buf[3] = 0X04;
	buf[4] = 0X7E;
	msgs[0].len = 5;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);	
	//printk("%s, d.Enable chip erase (0x41030), jni_write_data, ret = %d\n", __func__, ret);

	//e.Enable standby (0x40024)
	//f.Wait chip erase (94ms) or check MTP_busy (0x4102c)
	//printk("%s, Enable standby\n", __func__);
	set_standby(0x30);

	//g.Clear lock page (0x41024)
	buf[0] = 0X25;
	buf[1] = 0X24;
	buf[2] = 0X10;
	buf[3] = 0X04;
	buf[4] = 0xFF;
	buf[5] = 0xFF;
	msgs[0].len = 6;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);	
	//printk("%s, g.Clear lock page (0x41024), jni_write_data, ret = %d\n", __func__, ret);

	//Set sector erase
	for(i = 0x78; i <= 0x7E; i++) {
	//a.Setting sector erase key (0x41012)
	buf[0] = 0X25;
	buf[1] = 0X12;
	buf[2] = 0X10;
	buf[3] = 0X04;
	buf[4] = 0X12;
	buf[5] = 0XA5;
	ret = ilitek_i2c_only_write(i2c.client, buf, 6);
	//printk("%s, a.Setting sector erase key (0x41012), jni_write_data, ret = %d\n", __func__, ret);
	//b.Setting standby key (0x40010)
	set_standby_key(chip_id_H, chip_id_L);

	//c.Setting sector number (0x41018)
	buf[0] = 0X25;
	buf[1] = 0X18;
	buf[2] = 0X10;
	buf[3] = 0X04;
	buf[4] = (char)((i & 0x00FF) >> 0);
	buf[5] = (char)((i & 0xFF00) >> 8);
	buf[6] = 0x00;
	buf[7] = 0x00;
	ret = ilitek_i2c_only_write(i2c.client, buf, 8);
	//printk("%s, c.Setting sector number (0x41018), jni_write_data, ret = %d\n", __func__, ret);

	//d.Enable sector erase (0x41031)
	buf[0] = 0X25;
	buf[1] = 0X31;
	buf[2] = 0X10;
	buf[3] = 0X04;
	buf[4] = 0XA5;
	ret = ilitek_i2c_only_write(i2c.client, buf, 5);
	//printk("%s, d.Enable sector erase (0x41031), jni_write_data, ret = %d\n", __func__, ret);

    //e.Enable standby (0x40024)
	//f.Wait chip erase (94ms) or check MTP_busy (0x4102c)
	set_standby(0x31);
	}
	
	if(((ap_endaddr + 1) % 4) != 0) {
		ap_endaddr += 4;
	}
	buf[0] = 0X25;
	buf[1] = 0X14;
	buf[2] = 0X10;
	buf[3] = 0X04;
	buf[4] = 0xAE;
	buf[5] = 0x7E;
	msgs[0].len = 6;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);	
	//printk("%s, jni_write_data, ret = %d", __func__, ret);

	j = 0;
	for(i = ap_startaddr; i < ap_endaddr; i += 16) {
		buf[0] = 0x25;
		buf[3] = (char)((i  & 0x00FF0000) >> 16);
		buf[2] = (char)((i  & 0x0000FF00) >> 8);
		buf[1] = (char)((i  & 0x000000FF));
		for(k = 0; k < 16; k++) {
			buf[4 + k] = CTPM_FW[i + 32 + k];
		}
		msgs[0].len = 20;
		ret = ilitek_i2c_transfer(i2c.client, msgs, 1);	
		
	/*	upgrade_status = ((i * 100)) / ap_endaddr;
		if(upgrade_status > j){
			printk("%cILITEK: Firmware Upgrade(AP), %02d%c. ",0x0D,upgrade_status,'%');
			j = j+10;
		}*/
	}
	//printk("\nILITEK:%s, upgrade firmware completed\n", __func__);
	buf[0] = 0X25;
	buf[1] = 0X14;
	buf[2] = 0X10;
	buf[3] = 0X04;
	buf[4] = 0x00;
	buf[5] = 0x00;
	msgs[0].len = 6;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);	
	//printk("%s, jni_write_data, ret = %d", __func__, ret);
	buf[0] = 0x25;
	buf[1] = 0x9b;
	buf[2] = 0x00;
	buf[3] = 0x04;
	ilitek_i2c_only_write(i2c.client, buf, 4);
	ilitek_i2c_only_read(i2c.client, buf, 1);

	//check 2115 or 2116 again
	if(buf[0] == 0x01 || buf[0] == 0x02 || buf[0] == 0x03 || buf[0] == 0x04 || buf[0] == 0x11 || buf[0] == 0x12 || buf[0] == 0x13 || buf[0] == 0x14) {
		//printk("%s, ic is 2115\n", __func__);
		chip_id_H = 0x21;
		chip_id_L = 0x15;
		ic_model = (chip_id_H << 8) + chip_id_L;
	}
	if(buf[0] == 0x81 || buf[0] == 0x82 || buf[0] == 0x83 || buf[0] == 0x84 || buf[0] == 0x91 || buf[0] == 0x92 || buf[0] == 0x93 || buf[0] == 0x94) {
		//printk("%s, ic is 2116\n", __func__);
		chip_id_H = 0x21;
		chip_id_L = 0x16;
		ic_model = (chip_id_H << 8) + chip_id_L;
	}
	set_standby_key(chip_id_H, chip_id_L);
	//get checksum start and end address
	buf[0] = 0x25;
	buf[1] = 0x20;
	buf[2] = 0x10;
	buf[3] = 0x04;
	buf[4] = (char)((ap_startaddr & 0x00FF) >> 0);
	buf[5] = (char)((ap_startaddr & 0xFF00) >> 8);
	buf[6] = (char)(((ap_endaddr) & 0x00FF) >> 0);
	buf[7] = (char)(((ap_endaddr ) & 0xFF00) >> 8);
	msgs[0].len = 8;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);	
	//printk("%s, jni_write_data, ret = %d", __func__, ret);

	buf[0] = 0x25;
	buf[1] = 0x38;
	buf[2] = 0x10;
	buf[3] = 0x04;
	buf[4] = 0x01;
	msgs[0].len = 5;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);	
	//printk("%s, jni_write_data, ret = %d", __func__, ret);

	buf[0] = 0x25;
	buf[1] = 0x33;
	buf[2] = 0x10;
	buf[3] = 0x04;
	buf[4] = 0xCD;
	msgs[0].len = 5;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);	
	//printk("%s, jni_write_data, ret = %d", __func__, ret);

	buf[0] = 0x25;
	buf[1] = 0x24;
	buf[2] = 0x00;
	buf[3] = 0x04;
	buf[4] = 0x01;
	msgs[0].len = 5;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);	
	//printk("%s, jni_write_data, ret = %d", __func__, ret);

	for(i = 0; i < 500; i++) {
		ilitek_i2c_only_write(i2c.client, buf, 8);
		ilitek_i2c_only_read(i2c.client, buf, 1);
		if(buf[0] == 0x01) {
			break;
		} else {
			msleep(1);
		}

	}
	if(i == 500) {
		printk("%s,update finished ,please reboot\n");
	}

	//Get checksum
	//ice_checksum = inwrite(0x41028);
/*	if(ap_checksum != ice_checksum) {
		printk("checksum error, hex checksum = 0x%6x, ic checksum = 0x%6x\n", ap_checksum, ice_checksum);
	} else {
		printk("checksum equal, hex checksum = 0x%6x, ic checksum = 0x%6x\n", ap_checksum, ice_checksum);
	}
*/
	//RESET CPU
	buf[0] = 0x25;
	buf[1] = 0x40;
	buf[2] = 0x00;
	buf[3] = 0x04;
	buf[4] = 0xAE;
	msgs[0].len = 5;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);	
	//printk("%s, jni_write_data, ret = %d\n", __func__, ret);
	msleep(10);
	////1.Software Reset
	//1-1 Clear Reset Done Flag
	outwrite(0x41048, 0x00000000);
	//1-2 Set CDC/Core Reset
	outwrite(0x41040, 0xAE003F03);
	for(i = 0; i < 5000 ;i++){
		//msleep(1);
		udelay(200);
		if((inwrite(0x040048)&0x00010000) == 0x00010000)
			break;
	}
	//Exit ICE
	buf[0] = 0x1B;
	buf[1] = 0x62;
	buf[2] = 0x10;
	buf[3] = 0x18;
	msgs[0].len = 4;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);	
	//printk("%s, jni_write_data, ret = %d\n", __func__, ret);
	//msleep(1000);
	msleep(100);

	if(ap_checksum == ice_checksum) {
		for(i = 0; i < 50 ; i++) {
			buf[0] = 0x80;
			msgs[0].len = 1;
			ilitek_i2c_read(i2c.client, 0x61, buf, 1);			
			msleep(10);
			if(buf[0] == 0x50){
				printk("System is not busy,i=%d\n",i);
				break;
			}
		}
	}
	if(i == 50 && buf[0] != 0x50){
		printk("update pass please reboot,buf=0x%x\n",buf[0]);
	}
}

/******************************************************end**********************************************************************/   
//#else     
static int ilitek_2113_upgrade_firmware(void)
{
	int ret=0,upgrade_status=0,i,j,k = 0,ap_len = 0,df_len = 0;
	unsigned char buffer[128]={0};
	unsigned long ap_startaddr,df_startaddr,ap_endaddr,df_endaddr,ap_checksum = 0,df_checksum = 0;
	unsigned char firmware_ver[4];
	struct i2c_msg msgs[] = {
		{.addr = i2c.client->addr, .flags = 0, .len = 0, .buf = buffer,}
	};
	ap_startaddr = ( CTPM_FW[0] << 16 ) + ( CTPM_FW[1] << 8 ) + CTPM_FW[2];
	ap_endaddr = ( CTPM_FW[3] << 16 ) + ( CTPM_FW[4] << 8 ) + CTPM_FW[5];
	ap_checksum = ( CTPM_FW[6] << 16 ) + ( CTPM_FW[7] << 8 ) + CTPM_FW[8];
	df_startaddr = ( CTPM_FW[9] << 16 ) + ( CTPM_FW[10] << 8 ) + CTPM_FW[11];
	df_endaddr = ( CTPM_FW[12] << 16 ) + ( CTPM_FW[13] << 8 ) + CTPM_FW[14];
	df_checksum = ( CTPM_FW[15] << 16 ) + ( CTPM_FW[16] << 8 ) + CTPM_FW[17];
	firmware_ver[0] = CTPM_FW[18];
	firmware_ver[1] = CTPM_FW[19];
	firmware_ver[2] = CTPM_FW[20];

	firmware_ver[3] = CTPM_FW[21];
	df_len = ( CTPM_FW[22] << 16 ) + ( CTPM_FW[23] << 8 ) + CTPM_FW[24];
	ap_len = ( CTPM_FW[25] << 16 ) + ( CTPM_FW[26] << 8 ) + CTPM_FW[27];
	printk("ap_startaddr=0x%d,ap_endaddr=0x%d,ap_checksum=0x%d\n",ap_startaddr,ap_endaddr,ap_checksum);
	printk("df_startaddr=0x%d,df_endaddr=0x%d,df_checksum=0x%d\n",df_startaddr,df_endaddr,df_checksum);	
	buffer[0]=0xc0;
	msgs[0].len = 1;
	ret = ilitek_i2c_read(i2c.client, 0xc0, buffer, 1);
	if(ret < 0){
		return 3;
	}
	msleep(30);
	printk("ic. mode =%d\n",buffer[0]);

	if(buffer[0]!=0x55)
	{
		for(i=0;i<4;i++)
		{
			printk("i2c.firmware_ver[%d]=%d,firmware_ver[%d]=%d\n",i,i2c.firmware_ver[i],i,firmware_ver[i]);

			if((i2c.firmware_ver[i] > firmware_ver[i])||((i == 3) && (i2c.firmware_ver[3] == firmware_ver[3]))){
				return 1;				
			}
			else if(i2c.firmware_ver[i] < firmware_ver[i]){
				break;
			}

		}	

		buffer[0]=0xc4;
		msgs[0].len = 1;
		ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
		if(ret < 0){
			return 3;
		}
		msleep(30);
		buffer[0]=0xc2;
		ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
		if(ret < 0){
			return 3;
		}		
		msleep(100);
	}
	buffer[0]=0xc0;
	msgs[0].len = 1;
	ret = ilitek_i2c_read(i2c.client, 0xc0, buffer, 1);
	if(ret < 0){
		return 3;
	}

	msleep(30);
	printk("ILITEK:%s, upgrade firmware...\n", __func__);
	buffer[0]=0xc4;
	msgs[0].len = 10;
	buffer[1] = 0x5A;
	buffer[2] = 0xA5;
	buffer[3] = 0;
	buffer[4] = CTPM_FW[3];
	buffer[5] = CTPM_FW[4];
	buffer[6] = CTPM_FW[5];
	buffer[7] = CTPM_FW[6];
	buffer[8] = CTPM_FW[7];
	buffer[9] = CTPM_FW[8];
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
	if(ret < 0){
		return 3;
	}

	msleep(30);

	buffer[0]=0xc4;
	msgs[0].len = 10;
	buffer[1] = 0x5A;
	buffer[2] = 0xA5;
	buffer[3] = 1;
	buffer[4] = 0;
	buffer[5] = 0;
	buffer[6] = 0;
	buffer[7] = 0;
	buffer[8] = 0;
	buffer[9] = 0;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
	if(ret < 0){
		return 3;
	}

	msleep(30);

	j=0;
	for(i=0; i < df_len; i+=32)
	{
		j+= 1;
		if((j % 16) == 1){
			msleep(60);
		}
		for(k=0; k<32; k++){
			buffer[1 + k] = CTPM_FW[i + 32 + k];
		}

		buffer[0]=0xc3;
		msgs[0].len = 33;
		ret = ilitek_i2c_transfer(i2c.client, msgs, 1);	
		if(ret < 0){
			return 3;
		}
		upgrade_status = (i * 100) / df_len;
		msleep(10);
		printk("%cILITEK: Firmware Upgrade(Data flash), %02d%c. ",0x0D,upgrade_status,'%');
	}

	buffer[0]=0xc4;
	msgs[0].len = 10;
	buffer[1] = 0x5A;
	buffer[2] = 0xA5;
	buffer[3] = 0;
	buffer[4] = CTPM_FW[3];
	buffer[5] = CTPM_FW[4];
	buffer[6] = CTPM_FW[5];
	buffer[7] = CTPM_FW[6];
	buffer[8] = CTPM_FW[7];
	buffer[9] = CTPM_FW[8];
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
	if(ret < 0){
		return 3;
	}
	msleep(30);

	j=0;
	for(i = 0; i < ap_len; i+=32)
	{
		j+= 1;
		if((j % 16) == 1){
			msleep(30);
		}
		for(k=0; k<32; k++){
			buffer[1 + k] = CTPM_FW[i + df_len + 32 + k];
		}

		buffer[0]=0xc3;
		msgs[0].len = 33;
		ret = ilitek_i2c_transfer(i2c.client, msgs, 1);	
		if(ret < 0){
			return 3;
		}
		upgrade_status = (i * 100) / ap_len;
		msleep(10);
		printk("%cILITEK: Firmware Upgrade(AP), %02d%c. ",0x0D,upgrade_status,'%');
	}

	printk("ILITEK:%s, upgrade firmware completed\n", __func__);
	buffer[0]=0xc4;
	msgs[0].len = 1;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
	if(ret < 0){
		return 3;
	}

	msleep(30);
	buffer[0]=0xc1;
	ret = ilitek_i2c_transfer(i2c.client, msgs, 1);
	if(ret < 0){
		return 3;
	}

	buffer[0]=0xc0;
	msgs[0].len = 1;
	ret = ilitek_i2c_read(i2c.client, 0xc0, buffer, 1);
	if(ret < 0){
		return 3;
	}
	msleep(30);
	printk("ic. mode =%d , it's  %s \n",buffer[0],((buffer[0] == 0x5A)?"AP MODE":"BL MODE"));

	msleep(100);
	return 2;
}
//#endif
#endif //AGOLD_CTP_ILITEK_UPGRADE
#endif
static int ilitek_i2c_register_device(void)
{
	int err;
	int i = 0;
       unsigned char buffer[2]={0};
	   
	int ret = 0;

	//printk("\n@@@func :%s, line :%d", __func__, __LINE__);

	if(ret == 0)
	{
		i2c.valid_i2c_register = 1;
		printk(ILITEK_DEBUG_LEVEL "%s, add i2c device, success\n", __func__);
		if(i2c.client == NULL)
		{
			printk(ILITEK_ERROR_LEVEL "%s, no i2c board information\n", __func__);
			return -1;
		}
	//	printk(ILITEK_DEBUG_LEVEL "%s, client.addr: 0x%X\n", __func__, (unsigned int)i2c.client->addr);
	//	printk(ILITEK_DEBUG_LEVEL "%s, client.adapter: 0x%X\n", __func__, (unsigned int)i2c.client->adapter);
	//	printk(ILITEK_DEBUG_LEVEL "%s, client.driver: 0x%X\n", __func__, (unsigned int)i2c.client->driver);
		if((i2c.client->addr == 0) || (i2c.client->adapter == 0) || (i2c.client->driver == 0))
		{
			printk(ILITEK_ERROR_LEVEL "%s, invalid register\n", __func__);
			return ret;
		}
		//register dma
		I2CDMABuf_va = (u8 *)dma_alloc_coherent(&tpd->dev->dev, 16384, &I2CDMABuf_pa, GFP_KERNEL);
		if(!I2CDMABuf_va)
		{
			printk("[TPD] dma_alloc_coherent error\n");
			return -1;
		}
		i2c.client->ext_flag |= I2C_DMA_FLAG;
		// read touch parameter
		ret =ilitek_i2c_read_tp_info();
		if(ret < 0)
		{
			return ret;
		}       
		//warren close firmware
		#ifdef AGOLD_CTP_ILITEK_UPGRADE
	/*for(i=0;i<4;i++){
		printk("i2c.firmware_ver[%d]=%d,firmware_ver[%d]=%d\n",i,i2c.firmware_ver[i],i,CTPM_FW[i + 18]);
		if((i2c.firmware_ver[i] > CTPM_FW[i + 18])||((i == 3) && (i2c.firmware_ver[3] == CTPM_FW[3 + 18]))){
			ret = 1;
			break;
		}
		else if(i2c.firmware_ver[i] < CTPM_FW[i + 18]){
			break;
		}
	}*/

	//if(ret != 1) 
	//{
	   ret = ilitek_i2c_read(i2c.client, 0x61, buffer, 1);
		if(ret < 0)
		{
		   printk("read i2c error\n");
		}
	    buffer[1] =buffer[0];
	   ret = ilitek_i2c_read(i2c.client, 0x61, buffer, 1);
		if(ret < 0)
		{
		   printk("read i2c error\n");
		}	//upgrade fw
             printk("2113buffer=%d ,%d\n",buffer[0],buffer[1]);
		if((buffer[0]==0x05)||(buffer[1]==0x05))
		ilitek_2113_upgrade_firmware();
		else
		{
		  for(i=0;i<4;i++)
		  	{
				printk("i2c.firmware_ver[%d]=%d,firmware_ver[%d]=%d\n",i,i2c.firmware_ver[i],i,CTPM_FW[i + 18]);
				if((i2c.firmware_ver[i] > CTPM_FW[i + 18])||((i == 3) && (i2c.firmware_ver[3] == CTPM_FW[3 + 18]))){
					ret = 1;
					break;
				}
				else if(i2c.firmware_ver[i] < CTPM_FW[i + 18]){
					break;
				}
			   }
		  if(ret!=1)
		   ilitek_2116_upgrade_firmware();
	         else
		    printk("Do not need update\n");		  
		}
	//}
	/*else
		printk("Do not need update\n");*/
		#endif

		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
		msleep(50);
		mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
		msleep(100);

		ret=ilitek_i2c_read_tp_info();
		if(ret < 0)
		{
			return ret;
		}       
		i2c.input_dev = tpd->dev;

		// == => polling mode, != => interrup mode
		i2c.irq_work_queue = create_singlethread_workqueue("ilitek_i2c_irq_queue");
		if(i2c.irq_work_queue)
		{
			INIT_WORK(&i2c.irq_work, ilitek_i2c_irq_work_queue_func);
				mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
				mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
				mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_DISABLE);
				mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
			       mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, ilitek_i2c_isr, 1); 
			    msleep(50);
			    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
			    i2c.irq_status = 1;
			    msleep(10);

		}
	}
	tpd_load_status = 1;
	return 0;
}


static int ilitek_init(void)
{
	int ret = 0;

	printk("\n@@@func :%s, line :%d", __func__, __LINE__);


	printk(ILITEK_DEBUG_LEVEL "%s\n", __func__);

	// initialize global variable
	memset(&dev, 0, sizeof(struct dev_data));
	memset(&i2c, 0, sizeof(struct i2c_data));

	// initialize mutex object
	#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 37)	
	init_MUTEX(&i2c.wr_sem);
	#else
	sema_init(&i2c.wr_sem,1);
	#endif

	i2c.wr_sem.count = 1;

	// register i2c device
	ret = i2c_add_driver(&ilitek_i2c_driver);//
	if(ret < 0)
	{
		printk(ILITEK_ERROR_LEVEL "%s, register i2c device, error\n", __func__);
		return ret;
	}

	// allocate character device driver buffer
	ret = alloc_chrdev_region(&dev.devno, 0, 1, ILITEK_FILE_DRIVER_NAME);
	if(ret)
	{
		printk(ILITEK_ERROR_LEVEL "%s, can't allocate chrdev\n", __func__);
		return ret;
	}
	printk(ILITEK_DEBUG_LEVEL "%s, register chrdev(%d, %d)\n", __func__, MAJOR(dev.devno), MINOR(dev.devno));

	// initialize character device driver
	cdev_init(&dev.cdev, &ilitek_fops);
	dev.cdev.owner = THIS_MODULE;
	ret = cdev_add(&dev.cdev, dev.devno, 1);
	if(ret < 0)
	{
		printk(ILITEK_ERROR_LEVEL "%s, add character device error, ret %d\n", __func__, ret);
		return ret;
	}
	dev.class = class_create(THIS_MODULE, ILITEK_FILE_DRIVER_NAME);
	if(IS_ERR(dev.class))
	{
		printk(ILITEK_ERROR_LEVEL "%s, create class, error\n", __func__);
		return ret;
	}
	device_create(dev.class, NULL, dev.devno, NULL, "ilitek_ctrl");
	//proc_create("ilitek_ctrl", 0666, NULL, &ilitek_fops);
	Report_Flag=0;
	return 0;
}


static void ilitek_exit(void)
{

	if(i2c.client->irq != 0)
	{
		if(i2c.valid_irq_request != 0)
		{
			free_irq(i2c.client->irq, &i2c);
			printk(ILITEK_DEBUG_LEVEL "%s, free irq\n", __func__);
			if(i2c.irq_work_queue)
			{
				destroy_workqueue(i2c.irq_work_queue);
				printk(ILITEK_DEBUG_LEVEL "%s, destory work queue\n", __func__);
			}
		}
	}
	else
	{
		if(i2c.thread != NULL)
		{
			kthread_stop(i2c.thread);
			printk(ILITEK_DEBUG_LEVEL "%s, stop i2c thread\n", __func__);
		}
	}
	if(i2c.valid_i2c_register != 0)
	{
		i2c_del_driver(&ilitek_i2c_driver);
		printk(ILITEK_DEBUG_LEVEL "%s, delete i2c driver\n", __func__);
	}
	if(i2c.valid_input_register != 0)
	{
		input_unregister_device(i2c.input_dev);
		printk(ILITEK_DEBUG_LEVEL "%s, unregister i2c input device\n", __func__);
	}

	// delete character device driver
	cdev_del(&dev.cdev);
	unregister_chrdev_region(dev.devno, 1);
	device_destroy(dev.class, dev.devno);
	class_destroy(dev.class);
	printk(ILITEK_DEBUG_LEVEL "%s\n", __func__);
}

static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, "mtk-tpd");
	return 0;
}

static int tpd_local_init(void) 
{
	printk("\n@@@func :%s, line :%d", __func__, __LINE__);
	boot_mode = get_boot_mode();
	if(boot_mode==3||boot_mode==7) boot_mode = NORMAL_BOOT;
	//power on, need confirm with SA

    // power up sequence
#ifdef GPIO_CTP_EN_PIN
    mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
#endif

#ifdef TPD_POWER_SOURCE_CUSTOM
    hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#endif

#ifdef TPD_POWER_SOURCE_1800
    hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif

    // this two lines is power on
	/* hwPowerOn(TPD_POWER_SOURCE, VOL_2800, "TP"); */
	msleep(50);    

	// set INT mode

	// reset
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(50);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	msleep(50);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(200);// 目前，上电，设置引脚而已。


	if(ilitek_init()!=0)
	{
		TPD_DMESG("unable to add i2c driver.\n");
		i2c_del_driver(&ilitek_i2c_driver);
		return -1;
	}
	#ifdef TPD_HAVE_BUTTON     
	tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
	#endif  

	tpd_type_cap = 1;
	return 0;
}

#if defined(AGOLD_CTP_HALL_SENSITIVITY)
static int agold_tpd_set_sensitivity(unsigned long arg)
{
	printk("%s\n",__func__);
	int ret = 0;
	int state = 0;
	int state_compare = 0;//compare with state
	int i = 0;
	char buffer=0;
	//arg = 1, set high sensitivity, else set normal
	if(arg){
		sensitivity_state = 1;
		printk("whl HALL close in function!");
		state = 0xB0;

	}else{
		sensitivity_state = 0;
		printk("whl HALL far in function!");
		state = 0xB1;

	}
	printk("[mu] arg = %d, state = %d\n", arg, state);
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM); 

	struct i2c_msg msg[] = {
		{
			.addr	= i2c.client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= &state,
		},
	};
	//send command three times
	if(hall_resume==0)
	{
		hall_failed = 1;
	}
	else
	{
		hall_failed = 0;
		i2c_transfer(i2c.client->adapter, msg, 1);
		printk("whl msg send succes in function !");
		msleep(50);
		i2c_transfer(i2c.client->adapter, msg, 1);
		msleep(150);
		//ilitek_i2c_read(i2c.client, 0xb3, &buffer, 1);
		//printk("whl buffer in funtion=%x\n !",buffer);
	}

	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	return 0;
}
#endif


static struct tpd_driver_t tpd_device_driver =
{
	.tpd_device_name = "ili2113a",
	.tpd_local_init = tpd_local_init,
	.suspend = ilitek_i2c_suspend,
	.resume = ilitek_i2c_resume,
	#ifdef TPD_HAVE_BUTTON
	.tpd_have_button = 1,
	#else
	.tpd_have_button = 0,
	#endif	
	#if defined(AGOLD_CTP_HALL_SENSITIVITY)
	.tpd_set_sensitivity = agold_tpd_set_sensitivity,
	#endif	
};


static int __init tpd_driver_init(void)
{
	TPD_DMESG("MediaTek ili2113a touch panel driver init\n");
	i2c_register_board_info(1, &i2c_tpd, 1);
	if ( tpd_driver_add(&tpd_device_driver) < 0)
		TPD_DMESG("add generic driver failed\n");

	return 0;
}

static void __exit tpd_driver_exit(void)
{
	TPD_DMESG("MediaTek ili2113a touch panel driver exit\n");
	//input_unregister_device(tpd->dev);
	tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);

MODULE_AUTHOR("Steward_Fu");
MODULE_DESCRIPTION("ILITEK I2C touch driver for Android platform");
MODULE_LICENSE("GPL");

