/*
 * This software is licensed under the terms of the GNU General Public 
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms. 
 * This program is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * GNU General Public License for more details. 
 * * VERSION      	DATE			AUTHOR          Note
 *    1.0		  2013-7-16			Focaltech        initial  based on MTK platform
 * 
 */

#include "tpd.h"

#include "tpd_custom_fts.h"
#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif
#ifdef TPD_SYSFS_DEBUG
#include "focaltech_ex_fun.h"
#endif
#ifdef TOUCH_PS
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#endif

#include "cust_gpio_usage.h"



//#define FT5X06_GESTRUE_SUPPORT

#ifdef CONFIG_VANZO_TOUCHPANEL_GESTURES_SUPPORT
#define GESTURE_LEFT		0x20
#define GESTURE_RIGHT		0x21
#define GESTURE_UP		    0x22
#define GESTURE_DOWN		0x23
#define GESTURE_DOUBLECLICK	0x24
#define GESTURE_O		    0x30
#define GESTURE_W		    0x31
#define GESTURE_M		    0x32
#define GESTURE_E		    0x33
#define GESTURE_C		    0x34
#define GESTURE_V			0x54
#define GESTURE_S			0x46
#define GESTURE_Z			0x41

#include "ft_gesture_lib.h"
#include <linux/suspend.h>
static  bool  onof=true;
#define FTS_GESTRUE_POINTS 255
#define FTS_GESTRUE_POINTS_ONETIME  62
#define FTS_GESTRUE_POINTS_HEADER 8
#define FTS_GESTURE_OUTPUT_ADRESS 0xD3
#define FTS_GESTURE_OUTPUT_UNIT_LENGTH 4

suspend_state_t get_suspend_state(void);
static S16 coordinate_x[150];
static S16 coordinate_y[150];
static int gpointnum;
#endif

struct Upgrade_Info fts_updateinfo[] =
{
    {0x55,"FT5x06",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 1, 2000},
    {0x08,"FT5606",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x06, 100, 2000},
    {0x0a,"FT5x16",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x07, 1, 1500},
    {0x05,"FT6208",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,60, 30, 0x79, 0x05, 10, 2000},
    {0x06,"FT6x06",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,100, 30, 0x79, 0x08, 10, 2000},
    {0x55,"FT5x06i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 1, 2000},
    {0x14,"FT5336",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
    {0x13,"FT3316",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
    {0x12,"FT5436i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
    {0x11,"FT5336i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,30, 30, 0x79, 0x11, 10, 2000},
    {0x36,"FT6336",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 1, 2000},
};

struct Upgrade_Info fts_updateinfo_curr;

#ifdef TOUCH_PS
#define APS_ERR(fmt,arg...)           	printk("<<proximity>> "fmt"\n",##arg)
#define TPD_PROXIMITY_DEBUG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)
#define TPD_PROXIMITY_DMESG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)

static u8 tpd_proximity_flag 			= 0;
static u8 tpd_proximity_detect 		= 1;//0-->close ; 1--> far away
static u8 tpd_proximity_flag_one 		= 0; //add for tpd_proximity by wangdongfang
extern int touch_set_ps(bool val);
#endif

static struct input_dev *input_dev = NULL;  //kaka
static struct class *ft5336_class;//kaka
extern struct tpd_device *tpd;

struct i2c_client *i2c_client = NULL;
struct task_struct *thread = NULL;

static DECLARE_WAIT_QUEUE_HEAD(waiter);
static DEFINE_MUTEX(i2c_access);


static void tpd_eint_interrupt_handler(void);
// start:Here maybe need port to different platform,like MT6575/MT6577
extern void mt_eint_unmask(unsigned int line);
extern void mt_eint_mask(unsigned int line);
/*
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
*/
// End:Here maybe need port to different platform,like MT6575/MT6577

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info);
static int tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);

static int tpd_flag 					= 0;
static int tpd_halt						= 0;
static int point_num 					= 0;
static int p_point_num 					= 0;



//#define TPD_CLOSE_POWER_IN_SLEEP
#define TPD_OK 							0
//register define
#define DEVICE_MODE 					0x00
#define GEST_ID 						0x01
#define TD_STATUS 						0x02
//point1 info from 0x03~0x08
//point2 info from 0x09~0x0E
//point3 info from 0x0F~0x14
//point4 info from 0x15~0x1A
//point5 info from 0x1B~0x20
//register define

#define TPD_RESET_ISSUE_WORKAROUND

#define TPD_MAX_RESET_COUNT 			3
//extern int tpd_mstar_status ;  // compatible mstar and ft6306 chenzhecong

#ifdef TPD_HAVE_BUTTON 
static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;

static int tpd_keys_local2[TPD_KEY_COUNT] = TPD_KEYS; //TPD_KEYS2 yangzhihong
#endif
#ifdef CONFIG_VANZO_TOUCHPANEL_GESTURES_SUPPORT
static int tpd_gestrue_keys[] = {KEY_RIGHT,KEY_LEFT,KEY_UP,KEY_DOWN,KEY_U,KEY_O,KEY_W,KEY_M,KEY_E,KEY_C,KEY_V,KEY_S,KEY_Z};
#define TPD_GESTRUE_KEY_CNT	(sizeof( tpd_gestrue_keys )/sizeof( tpd_gestrue_keys[0] ))
#endif

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

#define VELOCITY_CUSTOM_FT5206
#ifdef VELOCITY_CUSTOM_FT5206
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

// for magnify velocity********************************************

#ifndef TPD_VELOCITY_CUSTOM_X
#define TPD_VELOCITY_CUSTOM_X 			10
#endif
#ifndef TPD_VELOCITY_CUSTOM_Y
#define TPD_VELOCITY_CUSTOM_Y 			10
#endif

#define TOUCH_IOC_MAGIC 				'A'

#define TPD_GET_VELOCITY_CUSTOM_X _IO(TOUCH_IOC_MAGIC,0)
#define TPD_GET_VELOCITY_CUSTOM_Y _IO(TOUCH_IOC_MAGIC,1)


static int g_v_magnify_x =TPD_VELOCITY_CUSTOM_X;
static int g_v_magnify_y =TPD_VELOCITY_CUSTOM_Y;

static int tpd_misc_open(struct inode *inode, struct file *file)
{
    /*
       file->private_data = adxl345_i2c_client;

       if(file->private_data == NULL)
       {
       printk("tpd: null pointer!!\n");
       return -EINVAL;
       }
     */
    return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int tpd_misc_release(struct inode *inode, struct file *file)
{
    //file->private_data = NULL;
    return 0;
}
/*----------------------------------------------------------------------------*/
//static int adxl345_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
//       unsigned long arg)
static long tpd_unlocked_ioctl(struct file *file, unsigned int cmd,
        unsigned long arg)
{
    //struct i2c_client *client = (struct i2c_client*)file->private_data;
    //struct adxl345_i2c_data *obj = (struct adxl345_i2c_data*)i2c_get_clientdata(client);	
    //char strbuf[256];
    void __user *data;

    long err = 0;

    if(_IOC_DIR(cmd) & _IOC_READ)
    {
        err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    }
    else if(_IOC_DIR(cmd) & _IOC_WRITE)
    {
        err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    }

    if(err)
    {
        printk("tpd: access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
        return -EFAULT;
    }

    switch(cmd)
    {
        case TPD_GET_VELOCITY_CUSTOM_X:
            data = (void __user *) arg;
            if(data == NULL)
            {
                err = -EINVAL;
                break;	  
            }			

            if(copy_to_user(data, &g_v_magnify_x, sizeof(g_v_magnify_x)))
            {
                err = -EFAULT;
                break;
            }				 
            break;

        case TPD_GET_VELOCITY_CUSTOM_Y:
            data = (void __user *) arg;
            if(data == NULL)
            {
                err = -EINVAL;
                break;	  
            }			

            if(copy_to_user(data, &g_v_magnify_y, sizeof(g_v_magnify_y)))
            {
                err = -EFAULT;
                break;
            }				 
            break;

        default:
            printk("tpd: unknown IOCTL: 0x%08x\n", cmd);
            err = -ENOIOCTLCMD;
            break;
    }

    return err;
}


static struct file_operations tpd_fops = {
    .owner = THIS_MODULE,   //kaka
    .open = tpd_misc_open,
    .release = tpd_misc_release,
    .unlocked_ioctl = tpd_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tpd_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = TPD_NAME,
    .fops = &tpd_fops,
};

//**********************************************
#endif

struct touch_info {
    int y[5];
    int x[5];
    int p[5];
    int id[5];
    int xy[5];
};
/*
enum{
    DOUBLE_CLICK_OFF = 0,
    DOUBLE_CLICK_ON,
};

static ssize_t enable_ft5336_show(struct class *dev, 
        struct class_attribute *attr, char *buf)
{
    return sprintf(buf, "kaka--> %s\n", "gesenable");
}

static ssize_t enable_ft5336_store(struct class *class, 
        struct class_attribute *attr, const char *buf, size_t count)
{
    unsigned long state;
    ssize_t ret = -EINVAL;
    // printk("kaka enable_ft5336_store\n");
    // char s=&buf[0];
    // printk("kaka  buf ==%c\n",s);
    ret = kstrtoul(&buf[0], 10, &state);

    //	printk("kaka kstrtoul ret==%d\n",ret);
    //	printk("kaka state==%d\n",state);
    if(ret<0)
    {
        //printk("kaka Write buf error\n");
        return -ENOBUFS;
    }
    else
    {
        printk("kaka Write buf success\n");
        if(state==DOUBLE_CLICK_ON)
        {
            printk("kaka  state==%ld\n",state);
            printk("kaka success\n");
			#ifdef FT5X06_GESTRUE_SUPPORT
            onof=true;
            printk("kaka onof==%d",onof);
			#endif

        }
        else	
        {
            printk("kaka  state==%ld\n",state);
			#ifdef FT5X06_GESTRUE_SUPPORT
            onof=false;
			#endif
        }						
    }


    return count;
}

static CLASS_ATTR(gesenable, 0777, enable_ft5336_show, enable_ft5336_store);
*/
//kaka 
static const struct i2c_device_id ft5206_tpd_id[] = {{TPD_NAME,0},{}};
//unsigned short force[] = {0,0x70,I2C_CLIENT_END,I2C_CLIENT_END}; 
//static const unsigned short * const forces[] = { force, NULL };
//static struct i2c_client_address_data addr_data = { .forces = forces, };
static struct i2c_board_info __initdata ft5206_i2c_tpd={ I2C_BOARD_INFO(TPD_NAME, (0x70>>1))};

static struct i2c_driver tpd_i2c_driver = {
    .driver = {
        .name 	= TPD_NAME,
        .owner 	= THIS_MODULE,   //kaka
    },
    .probe 		= tpd_probe,
    .remove 	= tpd_remove,
    .id_table 	= ft5206_tpd_id,
    .detect 	= tpd_detect,
    // 	.shutdown	= tpd_shutdown,
    //  .address_data = &addr_data,
};


static  void tpd_down(int x, int y, int p) {
    // input_report_abs(tpd->dev, ABS_PRESSURE, p);
    input_report_key(tpd->dev, BTN_TOUCH, 1);
    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 20);
    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    //printk("D[%4d %4d %4d] ", x, y, p);
    /* track id Start 0 */
    input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, p); 
    input_mt_sync(tpd->dev);
#ifndef MT6572
    if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
#endif	
    {   
        tpd_button(x, y, 1);  
    }
    TPD_EM_PRINT(x, y, x, y, p-1, 1);
}

static  void tpd_up(int x, int y) {
    //input_report_abs(tpd->dev, ABS_PRESSURE, 0);
    input_report_key(tpd->dev, BTN_TOUCH, 0);
    //input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
    //input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    //input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    //printk("U[%4d %4d %4d] ", x, y, 0);
    input_mt_sync(tpd->dev);
    TPD_EM_PRINT(x, y, x, y, 0, 0);
#ifndef MT6572
    if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
#endif
    {   
        tpd_button(x, y, 0); 
    }   		 
}


#ifdef CONFIG_VANZO_TOUCHPANEL_GESTURES_SUPPORT
static void check_gesture(int gesture_id)
{
	//unsigned char data[8] = {0};
//	int gesture_id = 0;
	int keycode = 0;

    //mutex_lock(&i2c_access);
    //i2c_smbus_read_i2c_block_data(i2c_client, 0xd3, 8, &(data[0]));
    // mutex_unlock(&i2c_access);
    //  msleep(100);

	//gesture_id = fetch_object_sample(dataxx, datayy, dataxy, pointnum, time_stamp);

    // printk("kaka gesture_id==%d\n ",gesture_id);
    switch(gesture_id)
    {
        case GESTURE_LEFT:
            	printk("kaka GESTURE_LEFT\n");

            input_report_key(tpd->dev, KEY_LEFT, 1);
            input_sync(tpd->dev);
            input_report_key(tpd->dev, KEY_LEFT, 0);
            input_sync(tpd->dev);
            break;
        case GESTURE_RIGHT:
            	printk("kaka GESTURE_RIGHT\n");
            input_report_key(tpd->dev, KEY_RIGHT, 1);
            input_sync(tpd->dev);
            input_report_key(tpd->dev, KEY_RIGHT, 0);
            input_sync(tpd->dev);

            break;
        case GESTURE_UP:
            	printk("kaka GESTURE_UP\n");
            input_report_key(tpd->dev, KEY_UP, 1);
            input_sync(tpd->dev);
            input_report_key(tpd->dev, KEY_UP, 0);
            input_sync(tpd->dev);

            break;
        case GESTURE_DOWN:
            	printk("kaka GESTURE_DOWN\n");
            input_report_key(tpd->dev, KEY_DOWN, 1);
            input_sync(tpd->dev);
            input_report_key(tpd->dev, KEY_DOWN, 0);
            input_sync(tpd->dev);

            break;
        case GESTURE_DOUBLECLICK:
            	printk("kaka GESTURE_DOUBLECLICK\n");
            input_report_key(tpd->dev, KEY_U, 1);
            input_sync(tpd->dev);
            input_report_key(tpd->dev, KEY_U, 0);
            input_sync(tpd->dev);

            break;
        case GESTURE_O:
            	printk("kaka GESTURE_O\n");
            input_report_key(tpd->dev, KEY_O, 1);
            input_sync(tpd->dev);
            input_report_key(tpd->dev, KEY_O, 0);
            input_sync(tpd->dev);
            break;
        case GESTURE_W:
            	printk("kaka GESTURE_W\n");
            input_report_key(tpd->dev, KEY_W, 1);
            input_sync(tpd->dev);
            input_report_key(tpd->dev, KEY_W, 0);
            input_sync(tpd->dev);

            break;
        case GESTURE_M:
            	printk("kaka GESTURE_M\n");
            input_report_key(tpd->dev, KEY_M, 1);
            input_sync(tpd->dev);
            input_report_key(tpd->dev, KEY_M, 0);
            input_sync(tpd->dev);

            break;
        case GESTURE_E:
            	printk("kaka GESTURE_E\n");
            input_report_key(tpd->dev, KEY_E, 1);
            input_sync(tpd->dev);
            input_report_key(tpd->dev, KEY_E, 0);
            input_sync(tpd->dev);

            break;
        case GESTURE_C:
            	printk("kaka GESTURE_C\n");
            input_report_key(tpd->dev, KEY_C, 1);
            input_sync(tpd->dev);
            input_report_key(tpd->dev, KEY_C, 0);
            input_sync(tpd->dev);
            break;
        case GESTURE_V:
            	printk("kaka GESTURE_V\n");
            input_report_key(tpd->dev, KEY_V, 1);
            input_sync(tpd->dev);
            input_report_key(tpd->dev, KEY_V, 0);
            input_sync(tpd->dev);
            break;
        case GESTURE_S:
            	printk("kaka GESTURE_S\n");
            input_report_key(tpd->dev, KEY_S, 1);
            input_sync(tpd->dev);
            input_report_key(tpd->dev, KEY_S, 0);
            input_sync(tpd->dev);
            break;
        case GESTURE_Z:
            	printk("kaka GESTURE_Z\n");
            input_report_key(tpd->dev, KEY_Z, 1);
            input_sync(tpd->dev);
            input_report_key(tpd->dev, KEY_Z, 0);
            input_sync(tpd->dev);
            break;			
        default:

            break;
    }
    if(keycode > 0){
        //	    virkey_report(keycode,1,0);
        //    virkey_report(keycode,0,1);
    }
}


static int ft5x0x_read_Touchdata(void)
{
    unsigned char buf[FTS_GESTRUE_POINTS * 2] = { 0 };
    int ret = -1;
    int i = 0;
    buf[0] = 0xd3;
    int gestrue_id = 0;
    short pointnum = 0;
    ret = fts_i2c_Read(i2c_client, buf, 1, buf, FTS_GESTRUE_POINTS_HEADER);
    if (ret < 0)
    {
       // dev_err(&data->client->dev, "%s read touchdata failed.\n",            __func__);
        return ret;
    }
    /* FW ?¡À?¨®??3?¨º?¨º? */
    if (0x24 == buf[0])
    {
        gestrue_id = 0x24;
        check_gesture(gestrue_id);
        return -1;
    }

    pointnum = (short)(buf[1]) & 0xff;
    buf[0] = 0xd3;

    ret = fts_i2c_Read(i2c_client, buf, 1, buf, (pointnum * 4 + 8));
    if (ret < 0)
    {
      //  dev_err(&data->client->dev, "%s read touchdata failed.\n",            __func__);
        return ret;
    }

   gestrue_id = fetch_object_sample(buf, pointnum);

   //printk("%s line %d pointnum %d\n",__func__,__LINE__,pointnum);
   for(i = 0;i < pointnum;i++)
   {
       coordinate_x[i] = ((((s16) buf[0 + (4 * i)]) & 0x0F) << 8 | (((s16) buf[1 + (4 * i)])& 0xFF));//X¿¿
       coordinate_y[i] = ((((s16) buf[2 + (4 * i)]) & 0x0F) << 8 | (((s16) buf[3 + (4 * i)]) & 0xFF));//Y¿¿
	   //printk("x %d y %d \n",coordinate_x[i],coordinate_y[i]);
   }
   //printk("\n");
   
   gpointnum = pointnum;
   
   check_gesture(gestrue_id);
    return -1;
}
#endif


static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
{
    int i = 0;
    //#if (TPD_MAX_POINTS==2)
    //	char data[35] = {0};
    //#else
    //	char data[16] = {0};
    //#endif	
    char data[128] = {0};
    u16 high_byte,low_byte,reg;
    u8 report_rate =0;

    p_point_num = point_num;
    if (tpd_halt)
    {
        TPD_DMESG( "tpd_touchinfo return ..\n");
        return false;
    }
    mutex_lock(&i2c_access);


    reg = 0x00;
    fts_i2c_Read(i2c_client, &reg, 1, data, 64);
    mutex_unlock(&i2c_access);
    //TPD_DEBUG("received raw data from touch panel as following:\n");
    //TPD_DEBUG("[data[0]=%x,data[1]= %x ,data[2]=%x ,data[3]=%x ,data[4]=%x ,data[5]=%x]\n",data[0],data[1],data[2],data[3],data[4],data[5]);
    //TPD_DEBUG("[data[9]=%x,data[10]= %x ,data[11]=%x ,data[12]=%x]\n",data[9],data[10],data[11],data[12]);
    //TPD_DEBUG("[data[15]=%x,data[16]= %x ,data[17]=%x ,data[18]=%x]\n",data[15],data[16],data[17],data[18]);

    /*get the number of the touch points*/
    point_num= data[2] & 0x0f;

    //TPD_DEBUG("point_num =%d\n",point_num);

    for(i = 0; i < point_num; i++)  
    {
        cinfo->p[i] = data[3+6*i] >> 6; //event flag 
        cinfo->id[i] = data[3+6*i+2]>>4; //touch id
        /*get the X coordinate, 2 bytes*/
        high_byte = data[3+6*i];
        high_byte <<= 8;
        high_byte &= 0x0f00;
        low_byte = data[3+6*i + 1];
        cinfo->x[i] = high_byte |low_byte;

        //cinfo->x[i] =  cinfo->x[i] * 480 >> 11; //calibra

        /*get the Y coordinate, 2 bytes*/

        high_byte = data[3+6*i+2];
        high_byte <<= 8;
        high_byte &= 0x0f00;
        low_byte = data[3+6*i+3];
        cinfo->y[i] = high_byte |low_byte;
       // cinfo->xy[i] = data[3+6*i+4];
        //cinfo->y[i]=  cinfo->y[i] * 800 >> 11;
    }
    printk(" cinfo->x[0] = %d, cinfo->y[0] = %d, cinfo->p[0] = %d\n", cinfo->x[0], cinfo->y[0], cinfo->p[0]);	
    //TPD_DEBUG(" cinfo->x[1] = %d, cinfo->y[1] = %d, cinfo->p[1] = %d\n", cinfo->x[1], cinfo->y[1], cinfo->p[1]);		
    //TPD_DEBUG(" cinfo->x[2]= %d, cinfo->y[2]= %d, cinfo->p[2] = %d\n", cinfo->x[2], cinfo->y[2], cinfo->p[2]);	



    return true;
};

#ifdef TOUCH_PS
int tpd_read_ps(void)
{
    tpd_proximity_detect;
    return 0;    
}

static int tpd_get_ps_value(void)
{
    return tpd_proximity_detect;
}

static int tpd_enable_ps(int enable)
{
    u8 state;
    int ret = -1;

    i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &state);
    printk("[proxi_5206]read: 999 0xb0's value is 0x%02X\n", state);
    if (enable){
        state |= 0x01;
        tpd_proximity_flag = 1;
        TPD_PROXIMITY_DEBUG("[proxi_5206]ps function is on\n");	
    }else{
        state &= 0x00;	
        tpd_proximity_flag = 0;
        TPD_PROXIMITY_DEBUG("[proxi_5206]ps function is off\n");
		touch_set_ps(1);
    }

    ret = i2c_smbus_write_i2c_block_data(i2c_client, 0xB0, 1, &state);
    TPD_PROXIMITY_DEBUG("[proxi_5206]write: 0xB0's value is 0x%02X\n", state);
    return 0;
}
/*
int tpd_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
        void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value;

    hwm_sensor_data *sensor_data;
    TPD_DEBUG("[proxi_5206]command = 0x%02X\n", command);		
    switch (command)
    {
        case SENSOR_DELAY:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Set delay parameter error!\n");
                err = -EINVAL;
            }
            // Do nothing
            break;

        case SENSOR_ENABLE:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Enable sensor parameter error!\n");
                err = -EINVAL;
            }
            else
            {				
                value = *(int *)buff_in;
                if(value)
                {		
                    if((tpd_enable_ps(1) != 0))
                    {
                        APS_ERR("enable ps fail: %d\n", err); 
                        return -1;
                    }
                }
                else
                {
                    if((tpd_enable_ps(0) != 0))
                    {
                        APS_ERR("disable ps fail: %d\n", err); 
                        return -1;
                    }
                }
            }
            break;

        case SENSOR_GET_DATA:
            if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
            {
                APS_ERR("get sensor data parameter error!\n");
                err = -EINVAL;
            }
            else
            {

                sensor_data = (hwm_sensor_data *)buff_out;				

                if((err = tpd_read_ps()))
                {
                    err = -1;;
                }
                else
                {
                    sensor_data->values[0] = tpd_get_ps_value();
                    TPD_PROXIMITY_DEBUG("huang sensor_data->values[0] 1082 = %d\n", sensor_data->values[0]);
                    sensor_data->value_divide = 1;
                    sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
                }	

            }
            break;
        default:
            APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;
    }

    return err;	
}
*/
#endif

static int touch_event_handler(void *unused)
{ 
    struct touch_info cinfo, pinfo;
    int i=0;

    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
    sched_setscheduler(current, SCHED_RR, &param);

#ifdef TOUCH_PS
    int err;
    hwm_sensor_data sensor_data;
    u8 proximity_status;
    u8 state;
#endif

    do
    {
        mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
        set_current_state(TASK_INTERRUPTIBLE); 
        wait_event_interruptible(waiter,tpd_flag!=0);

        tpd_flag = 0;

        set_current_state(TASK_RUNNING);

#ifdef CONFIG_VANZO_TOUCHPANEL_GESTURES_SUPPORT
 if(get_suspend_state() == PM_SUSPEND_MEM  
	&& !((FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode()))
#ifdef TOUCH_PS
	&& (1 != tpd_proximity_flag)
#endif
	)
     {
        ft5x0x_read_Touchdata();
        continue;
    }
#endif

#ifdef TOUCH_PS
        if (tpd_proximity_flag == 1)
        {
            i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &state);
            TPD_PROXIMITY_DEBUG("proxi_5206 0xB0 state value is 1131 0x%02X\n", state);

            if(!(state&0x01))
            {
                tpd_enable_ps(1);
            }

            i2c_smbus_read_i2c_block_data(i2c_client, 0x01, 1, &proximity_status);
            TPD_PROXIMITY_DEBUG("proxi_5206 0x01 value is 1139 0x%02X\n", proximity_status);

            if (proximity_status == 0xC0)
            {
                tpd_proximity_detect = 0;	
				touch_set_ps(0);
            }
            else if(proximity_status == 0xE0)
            {
                tpd_proximity_detect = 1;
				touch_set_ps(1);
            }
			/*
            TPD_PROXIMITY_DEBUG("tpd_proximity_detect 1149 = %d\n", tpd_proximity_detect);

            if ((err = tpd_read_ps()))
            {
                TPD_PROXIMITY_DMESG("proxi_5206 read ps data 1156: %d\n", err);	
            }
            sensor_data.values[0] = tpd_get_ps_value();
            sensor_data.value_divide = 1;
            sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
            if ((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
            {
                TPD_PROXIMITY_DMESG(" proxi_5206 call hwmsen_get_interrupt_data failed= %d\n", err);	
			*/
        }  
#endif

        if (tpd_touchinfo(&cinfo, &pinfo)) 
        {
            //TPD_DEBUG("point_num = %d\n",point_num);
            TPD_DEBUG_SET_TIME;
            if(point_num >0) 
            {
                for(i =0; i<point_num; i++)//only support 3 point
                {
                    tpd_down(cinfo.x[i], cinfo.y[i], cinfo.id[i]);

                }
                input_sync(tpd->dev);
            }
            else  
            {
                tpd_up(cinfo.x[0], cinfo.y[0]);
                //TPD_DEBUG("release --->\n"); 
                //input_mt_sync(tpd->dev);

                input_sync(tpd->dev);
            }
        }

        if(tpd_mode==12)
        {
            //power down for desence debug
            //power off, need confirm with SA
#ifdef TPD_POWER_SOURCE_CUSTOM
            hwPowerDown(TPD_POWER_SOURCE_CUSTOM, "TP");
#endif

#ifdef TPD_POWER_SOURCE_1800
            hwPowerDown(TPD_POWER_SOURCE_1800, "TP");
#endif
#ifdef GPIO_CTP_EN_PIN
			mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
			mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
			mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);
#endif

            msleep(20);
        }
    }while(!kthread_should_stop());

    return 0;
}

static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info) 
{
    strcpy(info->type, TPD_DEVICE);	
    return 0;
}

static void tpd_eint_interrupt_handler(void)
{
    //TPD_DEBUG("TPD interrupt has been triggered\n");
    TPD_DEBUG_PRINT_INT;
    tpd_flag = 1;
    wake_up_interruptible(&waiter);
}

void focaltech_get_upgrade_array(void)
{

    u8 chip_id;
    u32 i;

    i2c_smbus_read_i2c_block_data(i2c_client,FT_REG_CHIP_ID,1,&chip_id);

    printk("%s chip_id = %x\n", __func__, chip_id);

    for(i=0;i<sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info);i++)
    {
        if(chip_id==fts_updateinfo[i].CHIP_ID)
        {
            memcpy(&fts_updateinfo_curr, &fts_updateinfo[i], sizeof(struct Upgrade_Info));
            break;
        }
    }

    if(i >= sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info))
    {
        memcpy(&fts_updateinfo_curr, &fts_updateinfo[0], sizeof(struct Upgrade_Info));
    }
}

#ifdef TPD_HIGH_SENSITIVITY_SUPPORT
static int _is_in_high_sensitivity_mode = 0;
static ssize_t focaltech_highsensitivity_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	ssize_t num_read_chars = 0;

	num_read_chars = snprintf(buf, PAGE_SIZE, "%d\n", _is_in_high_sensitivity_mode);

	return num_read_chars;
}

static ssize_t focaltech_highsensitivity_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
    u8 wr_buf[2];

	if(count == 0)
		return;

	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

    if(buf[0] == '1' && _is_in_high_sensitivity_mode == 0){
        fts_write_reg(i2c_client, 0xC0, 0x01);
        _is_in_high_sensitivity_mode = 1;
    }
    if(buf[0] == '0' && _is_in_high_sensitivity_mode == 1){
        fts_write_reg(i2c_client, 0xC0, 0x0);
        _is_in_high_sensitivity_mode = 0;
    }

    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    return count;
}

static DEVICE_ATTR(highsensitivity, 0777, focaltech_highsensitivity_show, focaltech_highsensitivity_store);
#endif

#ifdef CONFIG_VANZO_TOUCHPANEL_GESTURES_SUPPORT 
static int _is_open_gesture_mode = 0;
static ssize_t focaltech_gesture_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	ssize_t num_read_chars = 0;

	num_read_chars = snprintf(buf, PAGE_SIZE, "%d\n", _is_open_gesture_mode);

	return num_read_chars;
}

static ssize_t focaltech_gesture_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	if(count == 0)
		return;

	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	if(buf[0] == '1' && _is_open_gesture_mode == 0){
		_is_open_gesture_mode = 1;
	}
	if(buf[0] == '0' && _is_open_gesture_mode == 1){
		_is_open_gesture_mode = 0;
	}

    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
    return count;
}

#ifdef BUILD_CTS
static DEVICE_ATTR(gesture, 0664, focaltech_gesture_show, focaltech_gesture_store);
#else
static DEVICE_ATTR(gesture, 0777, focaltech_gesture_show, focaltech_gesture_store);
#endif

static ssize_t focaltech_coordinate_x_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	ssize_t num_read_chars = 0, count =0;
	int i;
	for (i=0; i< gpointnum; i++)
	{
		count = sprintf(buf, "%d ", coordinate_x[i]);
		buf += count;
		num_read_chars += count;
	}

	return num_read_chars;
}

#ifdef BUILD_CTS
static DEVICE_ATTR(coordinate_x, 0664, focaltech_coordinate_x_show, NULL);
#else
static DEVICE_ATTR(coordinate_x, 0777, focaltech_coordinate_x_show, NULL);
#endif

static ssize_t focaltech_coordinate_y_show(struct device *dev,struct device_attribute *attr,char *buf)
{
	ssize_t num_read_chars = 0, count =0;
	int i;
	for (i=0; i< gpointnum; i++)
	{
		count = sprintf(buf, "%d ", coordinate_y[i]);
		buf += count;
		num_read_chars += count;
	}

	return num_read_chars;
}

#ifdef BUILD_CTS
static DEVICE_ATTR(coordinate_y, 0664, focaltech_coordinate_y_show, NULL);
#else
static DEVICE_ATTR(coordinate_y, 0777, focaltech_coordinate_y_show, NULL);
#endif
#endif

static struct attribute *focaltech_attributes[] = {
#ifdef TPD_HIGH_SENSITIVITY_SUPPORT
	&dev_attr_highsensitivity.attr,
#endif
#ifdef CONFIG_VANZO_TOUCHPANEL_GESTURES_SUPPORT
	&dev_attr_gesture.attr,
	&dev_attr_coordinate_x.attr,
	&dev_attr_coordinate_y.attr,
#endif
    NULL
};

static struct attribute_group focaltech_attribute_group = {
    .attrs = focaltech_attributes
};

static int tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
{	 
    int retval = TPD_OK;
    int ret;//kaka
    char data;
    u8 report_rate=0;
    int err=0;
    int reset_count = 0;
    u8 chip_id,i;

reset_proc:   
    i2c_client = client;

    //power on, need confirm with SA
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
    msleep(5);
    TPD_DMESG(" fts ic reset\n");

#ifdef TPD_POWER_SOURCE_CUSTOM
    hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#endif

#ifdef TPD_POWER_SOURCE_1800
    hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif 

#ifdef GPIO_CTP_EN_PIN
    mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);
#endif

    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);

#if 0//def TPD_CLOSE_POWER_IN_SLEEP	 
    hwPowerDown(TPD_POWER_SOURCE,"TP");
    hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP");
    msleep(100);

#else
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
    msleep(5);
    TPD_DMESG(" fts ic reset\n");
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
#endif

    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
/*
    mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
    mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
    mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1); 
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
*/	
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 0);
    mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

    msleep(400);

#ifdef CONFIG_VANZO_TOUCHPANEL_GESTURES_SUPPORT
    init_para(TPD_WIDTH,TPD_HEIGHT,86,0,0);
    //fts_write_reg(i2c_client, 0xd0, 0x01);
	for(i=0; i<TPD_GESTRUE_KEY_CNT; i++)
	{
		input_set_capability(tpd->dev, EV_KEY, tpd_gestrue_keys[i]);
	}
#endif


    if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
    {
        TPD_DMESG("I2C transfer error, line: %d\n", __LINE__);
#ifdef TPD_RESET_ISSUE_WORKAROUND
        if ( reset_count < TPD_MAX_RESET_COUNT )
        {
            reset_count++;
            goto reset_proc;
        }
#endif
        return -1; 
    }

    tpd_load_status = 1;
    //   tpd_mstar_status =0 ;  // compatible mstar and ft6306 chenzhecong

    focaltech_get_upgrade_array();

#ifdef FTS_APK_DEBUG
    ft5x0x_create_apk_debug_channel(client);
#endif
#ifdef  TPD_SYSFS_DEBUG
    fts_create_sysfs(i2c_client);
#endif

    if ((ret = sysfs_create_group(&i2c_client->dev.kobj, &focaltech_attribute_group))) {
        printk("create attr fail\n");
        return -1;
    }

#ifdef FTS_CTL_IIC
    if (ft_rw_iic_drv_init(i2c_client) < 0)
  //      TPD_DMESG(TPD_DEVICE, "%s:[FTS] create fts control iic driver failed\n",__func__);
#endif

    //kaka 5336
    //create sys interface
	/*
    ft5336_class = class_create(THIS_MODULE, "syna");
    ret = class_create_file(ft5336_class, &class_attr_gesenable);
    if (ret) {
        printk("ft5336 gensenable create sys interface ERROR\n");
    }
	*/
	
    //kaka
#ifdef VELOCITY_CUSTOM_FT5206
    if((err = misc_register(&tpd_misc_device)))
    {
        printk("mtk_tpd: tpd_misc_device register failed\n");

    }
#endif

#ifdef TPD_AUTO_UPGRADE
    printk("********************Enter CTP Auto Upgrade********************\n");
    fts_ctpm_auto_upgrade(i2c_client);
#endif
    thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
    if (IS_ERR(thread))
    { 
        retval = PTR_ERR(thread);
        TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
    }

    TPD_DMESG("FTS Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");

#if 0 //def TPD_PROXIMITY
    struct hwmsen_object obj_ps;

    obj_ps.polling = 0;//interrupt mode
    obj_ps.sensor_operate = tpd_ps_operate;
    if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
    {
        APS_ERR("proxi_fts attach fail = %d\n", err);
    }
    else
    {
        APS_ERR("proxi_fts attach ok = %d\n", err);
    }		
#endif
#ifdef VANZO_DEVICE_NAME_SUPPORT
{
    extern void v_set_dev_name(int id, char *name);
    v_set_dev_name(2, "focaltech");
}
#endif
    return 0;

}

static int tpd_remove(struct i2c_client *client)
{

#ifdef FTS_APK_DEBUG
    ft5x0x_release_apk_debug_channel();
#endif
#ifdef TPD_SYSFS_DEBUG
    fts_release_sysfs(client);
#endif
	sysfs_remove_group(&client->dev.kobj, &focaltech_attribute_group);

#ifdef FTS_CTL_IIC
    ft_rw_iic_drv_exit();
#endif

    TPD_DEBUG("TPD removed\n");

    return 0;
}

static int tpd_local_init(void)
{
    TPD_DMESG("FTS I2C Touchscreen Driver (Built %s @ %s)\n", __DATE__, __TIME__);

    if(i2c_add_driver(&tpd_i2c_driver)!=0)
    {
        TPD_DMESG("FTS unable to add i2c driver.\n");
        return -1;
    }
    if(tpd_load_status == 0) 
    {
        TPD_DMESG("FTS add error touch panel driver.\n");
        i2c_del_driver(&tpd_i2c_driver);
        return -1;
    }

#ifdef TPD_HAVE_BUTTON  
    //kaka   
    if (get_boot_mode()==NORMAL_BOOT)
    {
        printk("get_boot_mode NORMAL_BOOT");
        tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local2, tpd_keys_dim_local);
    }
    else
    {
        tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);
        printk("get_boot_mode FACTORY_BOOT");
    }			


#endif   

#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif  
    TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);  
    tpd_type_cap = 1;
    return 0; 
}

static void tpd_resume( struct early_suspend *h )
{
    //int retval = TPD_OK;
    //char data;
    static char data2 = 0x00; //0x03
#ifdef TOUCH_PS	
    if (tpd_proximity_flag == 1)
    {
        if(tpd_proximity_flag_one == 1)
        {
            tpd_proximity_flag_one = 0;	
            TPD_DMESG(TPD_DEVICE " tpd_proximity_flag_one \n"); 
            return;
        }
    }
#endif	

    TPD_DMESG("TPD wake up\n");
#if  0//def TPD_CLOSE_POWER_IN_SLEEP	
    hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP");

#else
#ifdef CONFIG_VANZO_TOUCHPANEL_GESTURES_SUPPORT
    if(_is_open_gesture_mode)
    {
      i2c_smbus_write_i2c_block_data(i2c_client, 0xd0, 1, &data2);  //TP enter sleep mode  kaka
      msleep(5);
      //i2c_smbus_write_i2c_block_data(i2c_client, 0xd1, 1, &data1);  //TP enter sleep mode  kaka
      // msleep(5);
      // i2c_smbus_write_i2c_block_data(i2c_client, 0xd2, 1, &data1);  //TP enter sleep mode  kaka
    }

#endif
    mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
    msleep(10);  
    // mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    // mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(10); 
#endif

    {
        mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  
        msleep(30);
        tpd_halt = 0;

    }
    /* for resume debug
       if((i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1, &data))< 0)
       {
       TPD_DMESG("resume I2C transfer error, line: %d\n", __LINE__);
       }
     */
    tpd_up(0,0);
    input_sync(tpd->dev);
    TPD_DMESG("TPD wake up done\n");
    //return retval;
}

static void tpd_suspend( struct early_suspend *h )
{

    static char data = 0x1; //0x03
    static char data1 = 0x1f; //0x03 
    static char data2=0x03;
    //	printk("kaka onof on\n");
    //	printk("kaka onof==%d",onof);
#ifdef TOUCH_PS
	//printk("kaka %s line %d tpd_proximity_flag %d\n",__func__,__LINE__,tpd_proximity_flag);
	if(tpd_proximity_flag)
	{
		tpd_proximity_flag_one = 1;	
		return;
	}
#endif

#ifdef CONFIG_VANZO_TOUCHPANEL_GESTURES_SUPPORT
    if(_is_open_gesture_mode)
    {  
        //	printk("kaka  double click on\n");
        i2c_smbus_write_i2c_block_data(i2c_client, 0xd0, 1, &data);  //TP enter sleep mode  kaka
        msleep(5);	
       //i2c_smbus_write_i2c_block_data(i2c_client, 0xd1, 1, &data1);  //TP enter sleep mode  kaka
       // msleep(5);
       // i2c_smbus_write_i2c_block_data(i2c_client, 0xd2, 1, &data1);  //TP enter sleep mode  kaka
    }
    else
#endif	
    {
        //	printk("kaka   tpd_halt on\n");
        tpd_halt = 1;	
        // TPD_DMESG("TPD enter sleep\n");
        mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
        mutex_lock(&i2c_access);
        i2c_smbus_write_i2c_block_data(i2c_client, 0xA5, 1, &data2);  //TP enter sleep mode
        mutex_unlock(&i2c_access);

    } 

} 


static struct tpd_driver_t tpd_device_driver = {
    .tpd_device_name = TPD_NAME,
    .tpd_local_init = tpd_local_init,
    .suspend = tpd_suspend,
    .resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
    .tpd_have_button = 1,
#else
    .tpd_have_button = 0,
#endif		
#ifdef TOUCH_PS
	 .tpd_ps = tpd_enable_ps,
#endif		
};
/* called when loaded into kernel */
static int __init tpd_driver_init(void) {
    printk("MediaTek FTS touch panel driver init\n");
    i2c_register_board_info(IIC_PORT, &ft5206_i2c_tpd, 1);
    if(tpd_driver_add(&tpd_device_driver) < 0)
        TPD_DMESG("add FTS driver failed\n");
    return 0;
}

/* should never be called */
static void __exit tpd_driver_exit(void) {
    TPD_DMESG("MediaTek FTS touch panel driver exit\n");
    //input_unregister_device(tpd->dev);
    tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);


