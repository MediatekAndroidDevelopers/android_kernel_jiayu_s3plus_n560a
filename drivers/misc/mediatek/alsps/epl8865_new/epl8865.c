/* drivers/hwmon/mt6516/amit/epl8865.c - EPL8865 ALS/PS driver
 *
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/** VERSION: 1.04**/

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>
#include <linux/hwmsen_helper.h>
#include "epl8865.h"    //"epl8800.h"
#include "epl8865_gesture.h"
#include <linux/input/mt.h>


#include <mach/devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>

#include <alsps.h>
#include <linux/batch.h>


/******************************************************************************
 * extern functions
*******************************************************************************/
extern void mt_eint_mask(unsigned int eint_num);
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);


#define POWER_NONE_MACRO MT65XX_POWER_NONE

/*-------------------------MT6516&MT6575 define-------------------------------*/

/******************************************************************************
 *  configuration
 ******************************************************************************/
#define POLLING_MODE_GES		0   //0
#define POLLING_MODE_HS		    0

#define LUX_PER_COUNT			700		// 0.7

static int ALS_INTT				= EPL_INTT_ALS_20;
static int PS_INTT 					= EPL_INTT_PS_40;//EPL_INTT_PS_150;//EPL_INTT_PS_90;

#define HS_INTT_CENTER			EPL_INTT_PS_40
static int HS_INTT 				= HS_INTT_CENTER;

static int GES_CURRENT 			= EPL_DRIVE_200MA;
static int GES_INTT				= EPL_INTT_ALS_250;//EPL_INTT_ALS_500  parker 2014/9/2
static int GES_CYCLE			= 1;
static int GES_GAIN				= 0;

#define PS_DELAY 				40
#define ALS_DELAY 				120
#define GES_DELAY 		      	5
#define HS_DELAY 			    30

#define LOW_GAIN_DATA_SHEET		1
/******************************************************************************
*******************************************************************************/

#define TXBYTES 					2
#define RXBYTES					2

#define PACKAGE_SIZE 			8
#define I2C_RETRY_COUNT 		2

#define EPL8865_DEV_NAME   		 "EPL8865"

// for heart rate
static struct mutex sensor_mutex;
static bool change_int_time = false;
static int hs_count=0;
static int hs_idx=0;
static int show_hs_raws_flag=0;
static int hs_als_flag=0;
static volatile int gesture = 0;
static int ges_mode_curr_state = 0;

typedef struct _epl_ps_als_factory
{
    bool cal_file_exist;
    bool cal_finished;
    u16 ps_cal_h;
    u16 ps_cal_l;
    char ps_s1[16];
    char ps_s2[16];
};

typedef struct _epl_raw_data
{
    u8 raw_bytes[PACKAGE_SIZE];
    u16 renvo;
    u16 ps_state;
    u16 ps_raw;
    u16 als_ch0_raw;
    u16 als_ch1_raw;
    u16 als_lux;
    u16 ges_data[4];
    u16 hs_data[200];
	int event;
	struct _epl_ps_als_factory ps_als_factory;
	bool ps_suspend;
} epl_raw_data;

/*----------------------------------------------------------------------------*/
#define APS_TAG                 	  	"[ALS/PS] "
#define APS_FUN(f)              	  	printk(KERN_INFO APS_TAG"%s\n", __func__)
#define APS_ERR(fmt, args...)    	printk(KERN_ERR  APS_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    	printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    	printk(KERN_INFO fmt, ##args)

/*----------------------------------------------------------------------------*/
static struct i2c_client *epl8865_i2c_client = NULL;


/*----------------------------------------------------------------------------*/
static const struct i2c_device_id epl8865_i2c_id[] = {{"EPL8865",0},{}};
static struct i2c_board_info __initdata i2c_EPL8865= { I2C_BOARD_INFO("EPL8865", (0x82>>1))};
/*the adapter id & i2c address will be available in customization*/
//static unsigned short epl8865_force[] = {0x00, 0x92, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const epl8865_forces[] = { epl8865_force, NULL };
//static struct i2c_client_address_data epl8865_addr_data = { .forces = epl8865_forces,};


/*----------------------------------------------------------------------------*/
static int epl8865_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int epl8865_i2c_remove(struct i2c_client *client);
static int epl8865_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);


/*----------------------------------------------------------------------------*/
static int epl8865_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int epl8865_i2c_resume(struct i2c_client *client);

static void epl8865_eint_func(void);
static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd);

static struct epl8865_priv *g_epl8865_ptr = NULL;

/*----------------------------------------------------------------------------*/
typedef enum
{
    CMC_BIT_ALS   	= 1,
    CMC_BIT_PS     	= 2,
    CMC_BIT_GES  	= 4,
    CMC_BIT_HS  		= 8,
} CMC_BIT;

/*----------------------------------------------------------------------------*/
struct epl8865_i2c_addr      /*define a series of i2c slave address*/
{
    u8  write_addr;
    u8  ps_thd;     /*PS INT threshold*/
};

/*----------------------------------------------------------------------------*/
struct epl8865_priv
{
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct delayed_work  eint_work;
    struct delayed_work  polling_work;
    struct delayed_work  report_tp_work;
    struct input_dev *gs_input_dev;

    /*i2c address group*/
    struct epl8865_i2c_addr  addr;

    int 		polling_mode_ges;
    int 		polling_mode_hs;
    int		ir_type;

    /*misc*/
    atomic_t    	trace;
    atomic_t   	als_suspend;
    atomic_t    	ps_suspend;
    atomic_t	ges_suspend;
    atomic_t	hs_suspend;

    /*data*/
    u16		lux_per_count;
    ulong       	enable;         	/*record HAL enalbe status*/
    ulong      	pending_intr;   	/*pending interrupt*/

    /*data*/
    u16         	als_level_num;
    u16         	als_value_num;
    u32         	als_level[C_CUST_ALS_LEVEL-1];
    u32         	als_value[C_CUST_ALS_LEVEL];
};


extern struct alsps_hw *epl8865_get_cust_alsps_hw(void);

static int  epl8865_local_init(void);
static int  epl8865_remove(void);
static int epl8865_init_flag =-1; // 0<==>OK -1 <==> fail

static struct alsps_init_info epl8865_init_info = {
        .name = "epl8865",
        .init = epl8865_local_init,
        .uninit = epl8865_remove,
};

/*----------------------------------------------------------------------------*/
static struct i2c_driver epl8865_i2c_driver =
{
    .probe     	= epl8865_i2c_probe,
    .remove     = epl8865_i2c_remove,
    .detect     	= epl8865_i2c_detect,
    .suspend    = epl8865_i2c_suspend,
    .resume     = epl8865_i2c_resume,
    .id_table   	= epl8865_i2c_id,
    //.address_data = &epl8865_addr_data,
    .driver = {
        //.owner          = THIS_MODULE,
        .name           = EPL8865_DEV_NAME,
    },
};


static struct epl8865_priv *epl8865_obj = NULL;
static struct platform_driver epl8865_alsps_driver;
static struct wake_lock ps_lock;
static epl_raw_data	gRawData;

/*
//====================I2C write operation===============//
//regaddr: ELAN epl8865 Register Address.
//bytecount: How many bytes to be written to epl8865 register via i2c bus.
//txbyte: I2C bus transmit byte(s). Single byte(0X01) transmit only slave address.
//data: setting value.
//
// Example: If you want to write single byte to 0x1D register address, show below
//	      epl8865_I2C_Write(client,0x1D,0x01,0X02,0xff);
//
*/
static int epl8865_I2C_Write(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount, uint8_t txbyte, uint8_t data)
{
    uint8_t buffer[2];
    int ret = 0;
    int retry;

    buffer[0] = (regaddr<<3) | bytecount ;
    buffer[1] = data;

    for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
    {
        ret = i2c_master_send(client, buffer, txbyte);

        if (ret == txbyte)
        {
            break;
        }

        APS_ERR("i2c write error,TXBYTES %d\n",ret);
        mdelay(10);
    }


    if(retry>=I2C_RETRY_COUNT)
    {
        APS_ERR("i2c write retry over %d\n", I2C_RETRY_COUNT);
        return -EINVAL;
    }

    return ret;
}




/*
//====================I2C read operation===============//
*/
static int epl8865_I2C_Read(struct i2c_client *client)
{
    uint8_t buffer[RXBYTES];
    int ret = 0, i =0;
    int retry;

    for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
    {
        ret = i2c_master_recv(client, buffer, RXBYTES);

        if (ret == RXBYTES)
            break;

        APS_ERR("i2c read error,RXBYTES %d\r\n",ret);
        mdelay(10);
    }

    if(retry>=I2C_RETRY_COUNT)
    {
        APS_ERR("i2c read retry over %d\n", I2C_RETRY_COUNT);
        return -EINVAL;
    }

    for(i=0; i<RXBYTES; i++)
        gRawData.raw_bytes[i] = buffer[i];

    return ret;
}


static int elan_epl8865_I2C_Read_long(struct i2c_client *client, int bytecount)
{
    uint8_t buffer[bytecount];
    int ret = 0, i =0;
    int retry;

    for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
    {
        ret = i2c_master_recv(client, buffer, bytecount);

        if (ret == bytecount)
            break;

        APS_ERR("i2c read error,RXBYTES %d\r\n",ret);
        mdelay(10);
    }

    if(retry>=I2C_RETRY_COUNT)
    {
        APS_ERR("i2c read retry over %d\n", I2C_RETRY_COUNT);
        return -EINVAL;
    }

    for(i=0; i<bytecount; i++)
        gRawData.raw_bytes[i] = buffer[i];

    return ret;
}

static void epl8865_notify_event(int event)
{
    struct input_dev *idev = epl8865_obj->gs_input_dev;
	int x1, y1, x2, y2;
	int delta_x, delta_y, i;
	int reportv;
	x1 = y1 = x2 = y2 = 0;
    GES_LOG("notify event %d\n",event);

	  if((event == 2)||(event == 3))
		gesture = event-1;
	  else if(event == 5)
	    gesture = event-2;
	  else
		gesture = event;
		GES_LOG("epl8865 epl8865_report_ges ges_mode_curr_state=%d,gesture=%d,line=%d\n",ges_mode_curr_state,gesture,__LINE__);
   if (0 == ges_mode_curr_state)
   {
			switch (gesture) {
	case 1://0:
		x1 = x2 = 300;
		y1 = 700;
		y2 = 500;
		reportv = 1;//KEYCODE_UP;//GES_DOWN;
		break;
	case 2:
		x1 = x2 = 300;
		y1 = 750;
		y2 = 950;
		reportv = 2;//KEYCODE_DOWN;//GES_LEFT;
		break;
	case 3://4:
		x1 = 250;
		x2 = 450;
		y1 = y2 = 720;
		reportv = 3;//KEYCODE_RIGHT;//GES_UP;
		break;
	case 4://6:
        x1 = 300;//800;
        x2 = 100;//400;
        y1 = y2 = 720;//1000;
		reportv = 4;//KEYCODE_LEFT;//GES_RIGHT;
		break;
	default:
		return;
	}
	GES_LOG("epl8865 epl8865_report_ges ges_mode_curr_state=%d,line=%d\n",ges_mode_curr_state,__LINE__);

		delta_x = (x2 - x1) / 5;
		delta_y = (y2 - y1) / 5;
		for (i = 0; i < 5; i++) {
			//input_report_abs(obj->gs_input_dev, ABS_MT_TRACKING_ID, 0);
			input_report_key(idev, BTN_TOUCH, 1);
			input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 1);
			input_report_abs(idev, ABS_MT_POSITION_Y, (y1 + delta_y * i));
			input_report_abs(idev, ABS_MT_POSITION_X, (x1 + delta_x * i));
			//input_report_abs(idevv, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(idev);
			input_sync(idev);
			msleep(1);
		}
		input_report_key(idev, BTN_TOUCH, 0);
		input_mt_sync(idev);
		input_sync(idev);

	 }
	else
	{
    if(event<EVENT_UNKNOWN)
    {
        input_report_key(idev, KEYCODE_ARRAY[event], 1);
        input_report_key(idev, KEYCODE_ARRAY[event], 0);
        input_sync(idev);
    }
  }
}

static void epl8865_hs_enable(struct epl8865_priv *epld, bool interrupt, bool full_enable)
{
    int ret;
    uint8_t regdata = 0;
    struct i2c_client *client = epld->client;

    if(full_enable)
    {
        regdata =  EPL_INT_CH1 | EPL_IR_INTERNAL | (interrupt? EPL_INT_FRAME_ENABLE : EPL_INT_DISABLE );
        ret = epl8865_I2C_Write(client,REG_6,W_SINGLE_BYTE,0x02, regdata);

        regdata =  EPL_DRIVE_200MA| EPL_IR_MODE_CURRENT;
        epl8865_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,regdata);

        regdata = EPL_EN_VOS_ENABLED | EPL_EN_GFIN_ENABLED | EPL_DOC_ON_DISABLED;
        ret =   epl8865_I2C_Write(client,REG_28,W_SINGLE_BYTE,0x02,regdata);

        regdata =  EPL_PS_MODE |EPL_12BIT_ADC | EPL_L_GAIN|EPL_S_SENSING_MODE;
        ret = epl8865_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

        regdata = EPL_CK_FAST_1M | EPL_IRONCTRL_OFF;
        ret = epl8865_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02, regdata);

        regdata = HS_INTT |EPL_SENSING_1_TIME;
        ret = epl8865_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);
        ret = epl8865_I2C_Write(client,REG_8,W_SINGLE_BYTE,0X02,EPL_C_RESET);

    }

    ret = epl8865_I2C_Write(client,REG_8,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);

    if(epld->polling_mode_hs == 1){
        msleep(HS_DELAY);
    }
}

static void epl8865_gesture_enable(struct epl8865_priv *epld, bool interrupt, bool full_enable)
{
    int ret;
    uint8_t regdata = 0;
    struct i2c_client *client = epld->client;

    if(full_enable)
    {
        regdata =  EPL_INT_CH1 | EPL_IR_INTERNAL | (interrupt? EPL_INT_FRAME_ENABLE : EPL_INT_DISABLE );
        ret = epl8865_I2C_Write(client,REG_6,W_SINGLE_BYTE,0x02, regdata);

        regdata = GES_CURRENT | EPL_IR_MODE_CURRENT;
        epl8865_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,regdata);

        regdata = EPL_EN_VOS_ENABLED | EPL_EN_GFIN_ENABLED | EPL_DOC_ON_DISABLED;
        epl8865_I2C_Write(client,REG_28,W_SINGLE_BYTE,0x02,regdata);

        regdata =  EPL_GES_MODE |EPL_8BIT_ADC | GES_GAIN |EPL_S_SENSING_MODE;
        ret = epl8865_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

        regdata = GES_INTT | (GES_CYCLE<<5);
        ret = epl8865_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);
        ret = epl8865_I2C_Write(client,REG_8,W_SINGLE_BYTE,0X02,EPL_C_RESET);
    }

    if(epld->ir_type==0)
    {
        regdata =  EPL_CK_FAST_2M | EPL_IRONCTRL_ON;
        ret = epl8865_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02, regdata);
        epld->ir_type = 1;
    }
    else
    {
        regdata =  EPL_CK_FAST_2M | EPL_IRONCTRL_OFF;
        ret = epl8865_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02, regdata);
        epld->ir_type = 0;
    }

    ret = epl8865_I2C_Write(client,REG_8,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);

}

//~~~~ add by ELAN Robert at 2014/2/8

static int elan_calibration_atoi(char* s)
{
    int num=0,flag=0;
    int i=0;
    //printk("[ELAN] %s\n", __func__);
    for(i=0; i<=strlen(s); i++)
    {
        if(s[i] >= '0' && s[i] <= '9')
            num = num * 10 + s[i] -'0';
        else if(s[0] == '-' && i==0)
            flag =1;
        else
            break;
    }
    if(flag == 1)
        num = num * -1;
    return num;
}

static int elan_calibaration_read(struct epl8865_priv *epl_data)
{
	struct file *fp_h;
	struct file *fp_l;
	struct i2c_client *client = epl_data->client;
	mm_segment_t fs;
	loff_t pos;
	APS_LOG("[ELAN] %s\n", __func__);

      //modify by ELAN Robert, checking calibration exist
	if(gRawData.ps_als_factory.cal_file_exist == 1)
	{
		//fp_h = filp_open("/data/alsps/h-threshold.dat", O_RDWR, 0777);
		fp_h = filp_open("/data/data/com.eminent.ps.calibration/h-threshold.dat", O_RDWR, 0777); //modify by ELAN Robert at 2014/03/27
		if (IS_ERR(fp_h))
		{
			APS_ERR("[ELAN]create file_h error\n");
			gRawData.ps_als_factory.cal_file_exist = 0;

		}

		//fp_l = filp_open("/data/alsps/l-threshold.dat", O_RDWR, 0777);
		fp_l = filp_open("/data/data/com.eminent.ps.calibration/l-threshold.dat", O_RDWR, 0777); //modify by ELAN Robert at 2014/03/27

		if (IS_ERR(fp_l))
		{
			APS_ERR("[ELAN]create file_l error\n");
			gRawData.ps_als_factory.cal_file_exist = 0;
		}
	}

	//modify by ELAN Robert, open calibration and read high / low threshold to hw structure. if open file fail, high / low threshold will use default.
	if(gRawData.ps_als_factory.cal_file_exist == 1)
	{
        int read_ret = 0;
		fs = get_fs();
		set_fs(KERNEL_DS);
		pos = 0;
		//gRawData.als_factory.s1 = {NULL, NULL, NULL, NULL, NULL};
		read_ret = vfs_read(fp_h, gRawData.ps_als_factory.ps_s1, sizeof(gRawData.ps_als_factory.ps_s1), &pos);
		gRawData.ps_als_factory.ps_s1[read_ret] = '\0';

		pos = 0;
		//gRawData.als_factory.s2 = {NULL, NULL, NULL, NULL, NULL};
		read_ret = vfs_read(fp_l, gRawData.ps_als_factory.ps_s2, sizeof(gRawData.ps_als_factory.ps_s2), &pos);
		gRawData.ps_als_factory.ps_s2[read_ret] = '\0';


		filp_close(fp_h, NULL);
		filp_close(fp_l, NULL);
		set_fs(fs);

		gRawData.ps_als_factory.ps_cal_h = elan_calibration_atoi(gRawData.ps_als_factory.ps_s1);
		gRawData.ps_als_factory.ps_cal_l = elan_calibration_atoi(gRawData.ps_als_factory.ps_s2);
		epl_data->hw->ps_threshold_high = gRawData.ps_als_factory.ps_cal_h;
		epl_data->hw->ps_threshold_low = gRawData.ps_als_factory.ps_cal_l;
		APS_LOG("[ELAN] read cal_h: %d , cal_l : %d\n", gRawData.ps_als_factory.ps_cal_h,gRawData.ps_als_factory.ps_cal_l);
	}

	gRawData.ps_als_factory.cal_finished = 1;
	return 0;
}
//add by ELAN Robert at 2014/2/8~~~~

static int epl8865_psensor_enable(struct epl8865_priv *epl_data, int enable)
{
    int ret = 0;
    int ps_state;
    uint8_t regdata;
    struct i2c_client *client = epl_data->client;
	
	//add by ELAN Robert at 2014/11/20
    hwm_sensor_data sensor_data;
	int err;
    APS_LOG("[ELAN epl8865] %s enable = %d\n", __func__, enable);

    ret = epl8865_I2C_Write(client,REG_6,W_SINGLE_BYTE,0x02, EPL_INT_DISABLE |  EPL_INT_CH1);

    if(enable)
    {
        regdata =  EPL_PS_MODE |EPL_10BIT_ADC | EPL_H_GAIN ;
        regdata	=regdata | (epl_data->hw->polling_mode_ps==0? EPL_C_SENSING_MODE : EPL_S_SENSING_MODE);
        ret = epl8865_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

        regdata = PS_INTT|EPL_SENSING_8_TIME;
        ret = epl8865_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);

        regdata = EPL_DRIVE_200MA | EPL_IR_MODE_CURRENT;
        epl8865_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,regdata);

         if(gRawData.ps_als_factory.cal_finished == 0 &&  gRawData.ps_als_factory.cal_file_exist ==1)
		    ret=elan_calibaration_read(epl_data);

        APS_LOG("[%s] cal_finished = %d\, cal_file_exist = %d\n", __func__, gRawData.ps_als_factory.cal_finished , gRawData.ps_als_factory.cal_file_exist);

        set_psensor_intr_threshold(epl_data->hw ->ps_threshold_low,epl_data->hw ->ps_threshold_high);

        ret = epl8865_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02, EPL_CK_FAST_1M | EPL_IRONCTRL_OFF);

        ret = epl8865_I2C_Write(client,REG_8,W_SINGLE_BYTE,0X02,EPL_C_RESET);
        ret = epl8865_I2C_Write(client,REG_8,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);

        msleep(PS_DELAY);
#if 1    //add  flank elan  2014/9/2
        epl8865_I2C_Write(client,REG_16,R_TWO_BYTE,0x01,0x00);
        epl8865_I2C_Read(client);
        gRawData.ps_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
#if LOW_GAIN_DATA_SHEET
		gRawData.ps_raw = gRawData.ps_raw >> 6;
    if(gRawData.ps_raw > 900)
       gRawData.ps_raw = 1023;
    if(gRawData.ps_raw < 10)
       gRawData.ps_raw = 0;
#endif
#endif
        if(epl_data->hw->polling_mode_ps==0)
        {
            epl8865_I2C_Write(client,REG_13,R_SINGLE_BYTE,0x01,0);
            epl8865_I2C_Read(client);
            ps_state= !((gRawData.raw_bytes[0]&0x04)>>2);

            if(gRawData.ps_state != ps_state)
            {
            	//add by ELAN Robert at 2014/11/20
            	gRawData.ps_state = ps_state;
				ps_report_interrupt_data(gRawData.ps_state);

                regdata =   EPL_INT_FRAME_ENABLE | EPL_INT_CH1;
                ret = epl8865_I2C_Write(client,REG_6,W_SINGLE_BYTE,0x02, regdata);
            }
            else
            {
                regdata =  EPL_INT_ACTIVE_LOW | EPL_INT_CH1;
                ret = epl8865_I2C_Write(client,REG_6,W_SINGLE_BYTE,0x02, regdata);
            }
        }

    }
    else
    {
        ret = epl8865_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,EPL_S_SENSING_MODE);
    }

    if(ret<0)
    {
        APS_ERR("[ELAN epl8865 error]%s: ps enable %d fail\n",__func__,ret);
    }
    else
    {
        ret = 0;
    }

    return ret;
}


static int epl8865_lsensor_enable(struct epl8865_priv *epl_data, int enable)
{
    int ret = 0;
    uint8_t regdata;
    struct i2c_client *client = epl_data->client;

    APS_LOG("[ELAN epl8865] %s enable = %d\n", __func__, enable);

    if(enable)
    {
        regdata = EPL_INT_DISABLE;
        ret = epl8865_I2C_Write(client,REG_6,W_SINGLE_BYTE,0x02, regdata);

        regdata =  EPL_ALS_MODE |EPL_10BIT_ADC | EPL_AUTO_GAIN |EPL_S_SENSING_MODE;;
        ret = epl8865_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02, regdata);

        regdata = ALS_INTT| EPL_SENSING_16_TIME;
        ret = epl8865_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02, regdata);

        ret = epl8865_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02, EPL_CK_FAST_1M | EPL_IRONCTRL_OFF);

        ret = epl8865_I2C_Write(client,REG_10,W_SINGLE_BYTE,0X02, EPL_GO_MID);
        ret = epl8865_I2C_Write(client,REG_11,W_SINGLE_BYTE,0x02, EPL_GO_LOW);

        ret = epl8865_I2C_Write(client,REG_8,W_SINGLE_BYTE,0X02, EPL_C_RESET);
        ret = epl8865_I2C_Write(client,REG_8,W_SINGLE_BYTE,0x02, EPL_C_START_RUN);
        msleep(ALS_DELAY);
    }


    if(ret<0)
    {
        APS_ERR("[ELAN epl8865 error]%s: als_enable %d fail\n",__func__,ret);
    }
    else
    {
        ret = 0;
    }

    return ret;
}

static void epl8865_read_hs(void)
{
    mutex_lock(&sensor_mutex);
    struct epl8865_priv *epld = epl8865_obj;
    struct i2c_client *client = epld->client;
    int max_frame = 200;
    int idx = hs_idx+hs_count;
    u16 data;


    epl8865_I2C_Write(client,REG_16,R_TWO_BYTE,0x01,0x00);
    elan_epl8865_I2C_Read_long(client, 2);
    data=(gRawData.raw_bytes[1]<<8)|gRawData.raw_bytes[0];


    if(data>60800&& HS_INTT>HS_INTT_CENTER-5)
    {
        HS_INTT--;
        change_int_time=true;
    }
    else if(data>6400 && data <25600 && HS_INTT<HS_INTT_CENTER+5)
    {
        HS_INTT++;
        change_int_time=true;
    }
    else
    {
        change_int_time=false;

        if(idx>=max_frame)
            idx-=max_frame;

        gRawData.hs_data[idx] = data;

        if(hs_count>=max_frame)
        {
            hs_idx++;
            if(hs_idx>=max_frame)
                hs_idx=0;
        }

        hs_count++;
        if(hs_count>=max_frame)
            hs_count=max_frame;
    }
    mutex_unlock(&sensor_mutex);

}

static void epl8865_gesture_rawdata(void)
{
    struct epl8865_priv *epld = epl8865_obj;
    struct i2c_client *client = epld->client;

    epl8865_I2C_Write(client,REG_14,R_EIGHT_BYTE,0x01,0x00);
    elan_epl8865_I2C_Read_long(client, 8);

    gRawData.ges_data[0]=(gRawData.raw_bytes[1]<<8 | gRawData.raw_bytes[0])>>7;
    gRawData.ges_data[1]=(gRawData.raw_bytes[3]<<8 | gRawData.raw_bytes[2])>>7;
    gRawData.ges_data[2]=(gRawData.raw_bytes[5]<<8 | gRawData.raw_bytes[4])>>7;
    gRawData.ges_data[3]=(gRawData.raw_bytes[7]<<8 | gRawData.raw_bytes[6])>>7;

    add_gesture_data(gRawData.ges_data, epld->ir_type);
/*
    if(epld->ir_type==0)
    {
        int event = detect_gesture_event();
        if(event!=EVENT_UNKNOWN)
            epl8865_notify_event(event);
    }
*/
    if(epld->ir_type==0)
    {
        gRawData.event = detect_gesture_event();
        if(gRawData.event!=EVENT_UNKNOWN)
			schedule_delayed_work(&epld->report_tp_work, msecs_to_jiffies(0));
    }
}



static int epl8865_get_als_value(struct epl8865_priv *obj, u16 als)
{
    int idx;
    int invalid = 0;
    int lux = 0;

    lux = (als * obj->lux_per_count)/1000;

    for(idx = 0; idx < obj->als_level_num; idx++)
    {
        if(lux < obj->hw->als_level[idx])
        {
            break;
        }
    }

    if(idx >= obj->als_value_num)
    {
        APS_ERR("exceed range\n");
        idx = obj->als_value_num - 1;
    }

    if(!invalid)
    {
        gRawData.als_lux = obj->hw->als_value[idx];
        APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
        return obj->hw->als_value[idx];
    }
    else
    {
        APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);
        return gRawData.als_lux;
    }
}


static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
    int ret = 0;
	u16 high_thd_buf, low_thd_buf;
    struct epl8865_priv *epld = epl8865_obj;
    struct i2c_client *client = epld->client;

    uint8_t high_msb ,high_lsb, low_msb, low_lsb;
	high_thd_buf = high_thd;
	low_thd_buf = low_thd;

#if LOW_GAIN_DATA_SHEET
	high_thd_buf = high_thd_buf << 6;
	low_thd_buf = low_thd_buf << 6;
#endif

    APS_LOG("%s\n", __func__);

    high_msb = (uint8_t) (high_thd_buf >> 8);
    high_lsb   = (uint8_t) (high_thd_buf & 0x00ff);
    low_msb  = (uint8_t) (low_thd_buf >> 8);
    low_lsb    = (uint8_t) (low_thd_buf & 0x00ff);

    APS_LOG("%s: low_thd = 0x%X, high_thd = 0x%x \n",__func__, low_thd, high_thd);

    epl8865_I2C_Write(client,REG_2,W_SINGLE_BYTE,0x02,high_lsb);
    epl8865_I2C_Write(client,REG_3,W_SINGLE_BYTE,0x02,high_msb);
    epl8865_I2C_Write(client,REG_4,W_SINGLE_BYTE,0x02,low_lsb);
    epl8865_I2C_Write(client,REG_5,W_SINGLE_BYTE,0x02,low_msb);

    return ret;
}



/*----------------------------------------------------------------------------*/
static void epl8865_dumpReg(struct i2c_client *client)
{
    APS_LOG("chip id REG 0x00 value = %8x\n", i2c_smbus_read_byte_data(client, 0x00));
    APS_LOG("chip id REG 0x01 value = %8x\n", i2c_smbus_read_byte_data(client, 0x08));
    APS_LOG("chip id REG 0x02 value = %8x\n", i2c_smbus_read_byte_data(client, 0x10));
    APS_LOG("chip id REG 0x03 value = %8x\n", i2c_smbus_read_byte_data(client, 0x18));
    APS_LOG("chip id REG 0x04 value = %8x\n", i2c_smbus_read_byte_data(client, 0x20));
    APS_LOG("chip id REG 0x05 value = %8x\n", i2c_smbus_read_byte_data(client, 0x28));
    APS_LOG("chip id REG 0x06 value = %8x\n", i2c_smbus_read_byte_data(client, 0x30));
    APS_LOG("chip id REG 0x07 value = %8x\n", i2c_smbus_read_byte_data(client, 0x38));
    APS_LOG("chip id REG 0x09 value = %8x\n", i2c_smbus_read_byte_data(client, 0x48));
    APS_LOG("chip id REG 0x0D value = %8x\n", i2c_smbus_read_byte_data(client, 0x68));
    APS_LOG("chip id REG 0x0E value = %8x\n", i2c_smbus_read_byte_data(client, 0x70));
    APS_LOG("chip id REG 0x0F value = %8x\n", i2c_smbus_read_byte_data(client, 0x71));
    APS_LOG("chip id REG 0x10 value = %8x\n", i2c_smbus_read_byte_data(client, 0x80));
    APS_LOG("chip id REG 0x11 value = %8x\n", i2c_smbus_read_byte_data(client, 0x88));
    APS_LOG("chip id REG 0x13 value = %8x\n", i2c_smbus_read_byte_data(client, 0x98));

}


/*----------------------------------------------------------------------------*/
int epl8865_hw8k_init_device(struct i2c_client *client)
{
    APS_LOG("epl8865_hw8k_init_device.........\r\n");

    epl8865_i2c_client=client;

    APS_LOG(" I2C Addr==[0x%x],line=%d\n",epl8865_i2c_client->addr,__LINE__);

    return 0;
}

/*----------------------------------------------------------------------------*/
int epl8865_get_addr(struct alsps_hw *hw, struct epl8865_i2c_addr *addr)
{
    if(!hw || !addr)
    {
        return -EFAULT;
    }
    addr->write_addr= hw->i2c_addr[0];
    return 0;
}


/*----------------------------------------------------------------------------*/
static void epl8865_power(struct alsps_hw *hw, unsigned int on)
{
    static unsigned int power_on = 0;

    //APS_LOG("power %s\n", on ? "on" : "off");
    if(hw->power_id != POWER_NONE_MACRO)
    {
        if(power_on == on)
        {
            APS_LOG("ignore power control: %d\n", on);
        }
        else if(on)
        {
            if(!hwPowerOn(hw->power_id, hw->power_vol, "EPL8865"))
            {
                APS_ERR("power on fails!!\n");
            }
        }
        else
        {
            if(!hwPowerDown(hw->power_id, "EPL8865"))
            {
                APS_ERR("power off fail!!\n");
            }
        }
    }
    power_on = on;

}



/*----------------------------------------------------------------------------*/
static int epl8865_check_intr(struct i2c_client *client)
{
    struct epl8865_priv *obj = i2c_get_clientdata(client);
    int mode;

    //APS_LOG("int pin = %d\n", mt_get_gpio_in(GPIO_ALS_EINT_PIN));

    //if (mt_get_gpio_in(GPIO_ALS_EINT_PIN) == 1) /*skip if no interrupt*/
    //   return 0;

    epl8865_I2C_Write(obj->client,REG_13,R_SINGLE_BYTE,0x01,0);
    epl8865_I2C_Read(obj->client);
    mode =((gRawData.raw_bytes[0]>>3)&0x07);
    APS_LOG("mode %d\n", mode);

    if(mode==0x01)// PS
    {
        set_bit(CMC_BIT_PS, &obj->pending_intr);
    }
    else
    {
        clear_bit(CMC_BIT_PS, &obj->pending_intr);
    }

//    APS_LOG("check intr: 0x%08X\n", obj->pending_intr);

    return 0;

}


/*----------------------------------------------------------------------------*/

int epl8865_read_als(struct i2c_client *client, u16 *data)
{
    struct epl8865_priv *obj = i2c_get_clientdata(client);
    uint8_t setting;
    u16 ch1;

    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    epl8865_I2C_Write(client,REG_13,R_SINGLE_BYTE,0x01,0);
    epl8865_I2C_Read(client);
    setting = gRawData.raw_bytes[0];
    if(((setting>>3)&7)!=0x00)
    {
        APS_ERR("read als data in wrong mode\n");
    }

    epl8865_I2C_Write(obj->client,REG_16,R_TWO_BYTE,0x01,0x00);
    epl8865_I2C_Read(obj->client);
    ch1 = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];

    // FIX: mid gain and low gain cannot report ff in auton gain
    if(setting>>6 ==2&& ch1==65535)
    {
        APS_LOG("setting %d, gain %x, als %d\n", setting, setting>>6,  ch1);
        APS_LOG("skip FF in auto gain\n\n");
    }
    else
    {
        switch (setting>>6)
        {
            case EPL_H_GAIN:
                ch1 = ch1  >> 6;
                break;

            case EPL_M_GAIN:
                ch1 = ch1  >> 3;
                break;
        }

        *data =  ch1;
        APS_LOG("read als raw data = %d\n",ch1);
    }

    return 0;
}


/*----------------------------------------------------------------------------*/
long epl8865_read_ps(struct i2c_client *client, u16 *data)
{
    struct epl8865_priv *obj = i2c_get_clientdata(client);
    uint8_t setting;

    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    epl8865_I2C_Write(obj->client,REG_13,R_SINGLE_BYTE,0x01,0);
    epl8865_I2C_Read(obj->client);
    setting = gRawData.raw_bytes[0];
    if(((setting>>3)&7)!=0x01)
    {
        APS_ERR("read ps data in wrong mode\n");
    }
    gRawData.ps_state= !((gRawData.raw_bytes[0]&0x04)>>2);


    epl8865_I2C_Write(obj->client,REG_16,R_TWO_BYTE,0x01,0x00);
    epl8865_I2C_Read(obj->client);
    gRawData.ps_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
#if LOW_GAIN_DATA_SHEET
	gRawData.ps_raw = gRawData.ps_raw >> 6;
    if(gRawData.ps_raw > 900)
       gRawData.ps_raw = 1023;
    if(gRawData.ps_raw < 50)
       gRawData.ps_raw = 0;
#endif

    *data = gRawData.ps_raw ;
    APS_LOG("read ps raw data = %d\n", gRawData.ps_raw);
    APS_LOG("read ps binary data = %d\n", gRawData.ps_state);

    return 0;
}

void  gesture_calibration(void)
{
    struct file *fp;
    mm_segment_t fs;
    loff_t pos;
    int value;
    char buffer[10]= {0};

    fp = filp_open("/data/data/com.eminent.gesture.calibration/gesture.dat", O_RDONLY, S_IRUSR);
    if (IS_ERR(fp))
    {
        APS_ERR("NO gesture calibration file\n");
        return;
    }
    fs = get_fs();
    set_fs(KERNEL_DS);
    pos = 0;
    vfs_read(fp, buffer, sizeof(buffer), &pos);

    sscanf(buffer, "%d", &value);
    crosstalk = value;
    zoom_enabled = true;

    filp_close(fp, NULL);
    set_fs(fs);
}


void epl8865_restart_polling(void)
{
    struct epl8865_priv *obj = epl8865_obj;
    cancel_delayed_work(&obj->polling_work);
    schedule_delayed_work(&obj->polling_work, msecs_to_jiffies(50));
}

void epl8865_report_tp_work(struct work_struct *work)
{
	struct input_dev *idev = epl8865_obj->gs_input_dev;
	int x1, y1, x2, y2;
	int delta_x, delta_y, i;
	int reportv;
	int ges_report;
	x1 = y1 = x2 = y2 = 0;
    GES_LOG("notify event %d\n",gRawData.event);

	if((gRawData.event == 2)||(gRawData.event == 3))
	ges_report = gRawData.event-1;
	else if(gRawData.event == 5)
	ges_report = gRawData.event-2;
	else
	ges_report = gRawData.event;
	GES_LOG("epl8865 epl8865_report_ges ges_mode_curr_state=%d,ges_report=%d,line=%d\n",ges_mode_curr_state,ges_report,__LINE__);

		switch (ges_report) {
			case 4://0:
				x1 = x2 = 300;
				y1 = 700;
				y2 = 500;
				reportv = 1;//KEYCODE_UP;//GES_DOWN;
				break;
			case 3:
				x1 = x2 = 300;
				y1 = 750;
				y2 = 950;
				reportv = 2;//KEYCODE_DOWN;//GES_LEFT;
				break;
			case 1://4:
				x1 = 250;
				x2 = 450;
				y1 = y2 = 720;
				reportv = 3;//KEYCODE_RIGHT;//GES_UP;
				break;
			case 2://6:
				x1 = 300;//800;
				x2 = 100;//400;
				y1 = y2 = 720;//1000;
				reportv = 4;//KEYCODE_LEFT;//GES_RIGHT;
				break;
			default:
				return;
		}
	if (0 == ges_mode_curr_state)
	{
		GES_LOG("epl8865 epl8865_report_ges ges_mode_curr_state=%d,line=%d\n",ges_mode_curr_state,__LINE__);
			delta_x = (x2 - x1) / 5;
			delta_y = (y2 - y1) / 5;
			for (i = 0; i < 5; i++) {
				//input_report_abs(obj->gs_input_dev, ABS_MT_TRACKING_ID, 0);
				input_report_key(idev, BTN_TOUCH, 1);
				input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 1);
				input_report_abs(idev, ABS_MT_POSITION_Y, (y1 + delta_y * i));
				input_report_abs(idev, ABS_MT_POSITION_X, (x1 + delta_x * i));
				//input_report_abs(idevv, ABS_MT_WIDTH_MAJOR, 1);
				input_mt_sync(idev);
				input_sync(idev);
				msleep(1);
			}
			input_report_key(idev, BTN_TOUCH, 0);
			input_mt_sync(idev);
			input_sync(idev);
	}
	else
	{
		gesture = reportv;
		if(gRawData.event<EVENT_UNKNOWN)
		{
			input_report_key(idev, KEYCODE_ARRAY[gRawData.event], 1);
			input_report_key(idev, KEYCODE_ARRAY[gRawData.event], 0);
			input_sync(idev);
		}
	}
}

void epl8865_polling_work(struct work_struct *work)
{
    struct epl8865_priv *obj = epl8865_obj;
    struct i2c_client *client = obj->client;

    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
    bool enable_ges = test_bit(CMC_BIT_GES, &obj->enable) && atomic_read(&obj->ges_suspend)==0;
    bool enable_hs = test_bit(CMC_BIT_HS, &obj->enable) && atomic_read(&obj->hs_suspend)==0;

    APS_LOG("als / ps / gesture /hs enable: %d / %d / %d / %d \n", enable_als, enable_ps ,enable_ges, enable_hs);

    cancel_delayed_work(&obj->polling_work);

    if((enable_ps && obj->hw->polling_mode_ps == 1) || (enable_als==true && enable_ges==false && enable_hs==false) ||(enable_ps && enable_als) )
    {
        schedule_delayed_work(&obj->polling_work, msecs_to_jiffies(ALS_DELAY+2*PS_DELAY+30));
    }

    if(enable_als &&  (enable_ges==false || enable_ps==true))
    {
        epl8865_lsensor_enable(obj, 1);
        epl8865_read_als(client, &gRawData.als_ch1_raw);
    }

    if(enable_hs)
    {
        if (obj->polling_mode_hs==0)
        {
            epl8865_hs_enable(obj, true, true);
        }
        else
        {
            epl8865_read_hs();
            epl8865_hs_enable(obj, false, true);
            schedule_delayed_work(&obj->polling_work, msecs_to_jiffies(5));
        }
    }

    else if(enable_ps)
    {
        epl8865_psensor_enable(obj, 1);
        if(obj->hw->polling_mode_ps==1)
        {
            epl8865_read_ps(client, &gRawData.ps_raw);
        }
    }
    else if(enable_ges)
    {
        if (obj->polling_mode_ges==0)
        {
            epl8865_gesture_enable(obj, true, true);
        }
        else
        {
            epl8865_gesture_rawdata();
            epl8865_gesture_enable(obj, false, true);
            schedule_delayed_work(&obj->polling_work, msecs_to_jiffies(5));
        }
    }

	//add by ELAN Robert at 2014/11/19
	if(gRawData.ps_suspend)
		cancel_delayed_work(&obj->polling_work);	
		
    if(enable_als==false && enable_ps==false && enable_ges==false && enable_hs==false)
    {
        APS_LOG("disable sensor\n");
        cancel_delayed_work(&obj->polling_work);
        epl8865_I2C_Write(client,REG_6,W_SINGLE_BYTE,0x02,EPL_INT_DISABLE); // modify flank 2014/9/2
		//epl8865_I2C_Write(client,REG_10,W_SINGLE_BYTE,0x02,EPL_INT_DISABLE);
        epl8865_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,EPL_S_SENSING_MODE);
    }

}


/*----------------------------------------------------------------------------*/
void epl8865_eint_func(void)
{
    struct epl8865_priv *obj = g_epl8865_ptr;

    // APS_LOG(" interrupt fuc\n");

    if(!obj)
    {
        return;
    }

    mt_eint_mask(CUST_EINT_ALS_NUM);

    schedule_delayed_work(&obj->eint_work, 0);
}



/*----------------------------------------------------------------------------*/
static void epl8865_eint_work(struct work_struct *work)
{
    struct epl8865_priv *epld = g_epl8865_ptr;
    hwm_sensor_data sensor_data;
    int err;

    if(test_bit(CMC_BIT_HS, &epld->enable) && atomic_read(&epld->hs_suspend)==0)
    {
        epl8865_read_hs();
        epl8865_hs_enable(epld, true, change_int_time);
    }

    else if(test_bit(CMC_BIT_PS, &epld->enable))
    {
        APS_LOG("xxxxx eint work\n");
        if((err = epl8865_check_intr(epld->client)))
        {
            APS_ERR("check intrs: %d\n", err);
        }

        if(epld->pending_intr)
        {
            epl8865_I2C_Write(epld->client,REG_13,R_SINGLE_BYTE,0x01,0);
            epl8865_I2C_Read(epld->client);
            gRawData.ps_state= !((gRawData.raw_bytes[0]&0x04)>>2);
            APS_LOG("ps state = %d\n", gRawData.ps_state);

            epl8865_I2C_Write(epld->client,REG_16,R_TWO_BYTE,0x01,0x00);
            epl8865_I2C_Read(epld->client);
            gRawData.ps_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
#if LOW_GAIN_DATA_SHEET
	gRawData.ps_raw = gRawData.ps_raw >> 6;
    if(gRawData.ps_raw > 900)
       gRawData.ps_raw = 1023;
    if(gRawData.ps_raw < 10)
       gRawData.ps_raw = 0;
#endif

            APS_LOG("ps raw_data = %d\n", gRawData.ps_raw);

//            sensor_data.values[0] = gRawData.ps_state;
//            sensor_data.value_divide = 1;
//            sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;

            //let up layer to know
            ps_report_interrupt_data(gRawData.ps_state);
        }

        epl8865_I2C_Write(epld->client,REG_6,W_SINGLE_BYTE,0x02,EPL_INT_ACTIVE_LOW | EPL_INT_CH1);
        epl8865_I2C_Write(epld->client,REG_8,W_SINGLE_BYTE,0x02,EPL_DATA_UNLOCK);
    }
    else if(test_bit(CMC_BIT_GES, &epld->enable) && atomic_read(&epld->ges_suspend)==0)
    {
        epl8865_gesture_rawdata();
        epl8865_gesture_enable(epld, true, false);
    }

    mt_eint_unmask(CUST_EINT_ALS_NUM);
}



/*----------------------------------------------------------------------------*/
int epl8865_setup_eint(struct i2c_client *client)
{
    struct epl8865_priv *obj = i2c_get_clientdata(client);

    APS_LOG("epl8865_setup_eint\n");


    g_epl8865_ptr = obj;

    /*configure to GPIO function, external interrupt*/

    mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);


	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINTF_TRIGGER_FALLING, epl8865_eint_func, 0);

	mt_eint_unmask(CUST_EINT_ALS_NUM);
    return 0;
}




/*----------------------------------------------------------------------------*/
static int epl8865_init_client(struct i2c_client *client)
{
    struct epl8865_priv *obj = i2c_get_clientdata(client);
    int err=0;

    APS_LOG("[Agold spl] I2C Addr==[0x%x],line=%d\n",epl8865_i2c_client->addr,__LINE__);

    /*  interrupt mode */


    APS_FUN();

    if(obj->hw->polling_mode_ps == 0)
    {
        mt_eint_mask(CUST_EINT_ALS_NUM);

        if((err = epl8865_setup_eint(client)))
        {
            APS_ERR("setup eint: %d\n", err);
            return err;
        }
        APS_LOG("epl8865 interrupt setup\n");
    }


    if((err = epl8865_hw8k_init_device(client)) != 0)
    {
        APS_ERR("init dev: %d\n", err);
        return err;
    }


    if((err = epl8865_check_intr(client)))
    {
        APS_ERR("check/clear intr: %d\n", err);
        return err;
    }


    /*  interrupt mode */
//if(obj->hw->polling_mode_ps == 0)
    //     mt65xx_eint_unmask(CUST_EINT_ALS_NUM);

    return err;
}


/*----------------------------------------------------------------------------*/
static ssize_t epl8865_show_reg(struct device_driver *ddri, char *buf)
{
    if(!epl8865_obj)
    {
        APS_ERR("epl8865_obj is null!!\n");
        return 0;
    }
    ssize_t len = 0;
    struct i2c_client *client = epl8865_obj->client;

    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x00 value = %8x\n", i2c_smbus_read_byte_data(client, 0x00));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x01 value = %8x\n", i2c_smbus_read_byte_data(client, 0x08));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x02 value = %8x\n", i2c_smbus_read_byte_data(client, 0x10));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x03 value = %8x\n", i2c_smbus_read_byte_data(client, 0x18));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x04 value = %8x\n", i2c_smbus_read_byte_data(client, 0x20));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x05 value = %8x\n", i2c_smbus_read_byte_data(client, 0x28));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x06 value = %8x\n", i2c_smbus_read_byte_data(client, 0x30));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x07 value = %8x\n", i2c_smbus_read_byte_data(client, 0x38));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x09 value = %8x\n", i2c_smbus_read_byte_data(client, 0x48));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0D value = %8x\n", i2c_smbus_read_byte_data(client, 0x68));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0E value = %8x\n", i2c_smbus_read_byte_data(client, 0x70));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0F value = %8x\n", i2c_smbus_read_byte_data(client, 0x71));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x10 value = %8x\n", i2c_smbus_read_byte_data(client, 0x80));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x11 value = %8x\n", i2c_smbus_read_byte_data(client, 0x88));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x13 value = %8x\n", i2c_smbus_read_byte_data(client, 0x98));

    return len;

}

/*----------------------------------------------------------------------------*/
static ssize_t epl8865_show_status(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct epl8865_priv *epld = epl8865_obj;
    u16 ch0, ch1, ch2, ch3;

    if(!epl8865_obj)
    {
        APS_ERR("epl8865_obj is null!!\n");
        return 0;
    }
    epl8865_I2C_Write(epld->client,REG_8,W_SINGLE_BYTE,0x02,EPL_DATA_LOCK);

    epl8865_I2C_Write(epld->client,REG_14,R_TWO_BYTE,0x01,0x00);
    epl8865_I2C_Read(epld->client);
    ch0 = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
    epl8865_I2C_Write(epld->client,REG_16,R_TWO_BYTE,0x01,0x00);
    epl8865_I2C_Read(epld->client);
    ch1 = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
    epl8865_I2C_Write(epld->client,REG_18,R_TWO_BYTE,0x01,0x00);
    epl8865_I2C_Read(epld->client);
    ch2 = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
    epl8865_I2C_Write(epld->client,REG_20,R_TWO_BYTE,0x01,0x00);
    epl8865_I2C_Read(epld->client);
    ch3 = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];

    epl8865_I2C_Write(epld->client,REG_8,W_SINGLE_BYTE,0x02,EPL_DATA_UNLOCK);

    len += snprintf(buf+len, PAGE_SIZE-len, "als/ps/ges enable is %d-%d-%d\n",test_bit(CMC_BIT_ALS, &epld->enable) ,test_bit(CMC_BIT_PS, &epld->enable) ,test_bit(CMC_BIT_GES, &epld->enable) );
    len += snprintf(buf+len, PAGE_SIZE-len, "als/ps int time is %d-%d\n",ALS_INTT, PS_INTT);
    len += snprintf(buf+len, PAGE_SIZE-len, "ch0 ch1 ch2 ch3 raw is (%d) (%d)(%d) (%d) \n",ch0, ch1, ch2, ch3);
    len += snprintf(buf+len, PAGE_SIZE-len, "ps threshold is %d / %d\n",epld->hw->ps_threshold_low, epld->hw->ps_threshold_high);
    len += snprintf(buf+len, PAGE_SIZE-len, "gesture polling mode is %d\n",epld->polling_mode_ges);
    len += snprintf(buf+len, PAGE_SIZE-len, "gesture work is %d \n",WORK_TH);
    len += snprintf(buf+len, PAGE_SIZE-len, "gesture int time: %d,  opt: %d\n", GES_INTT, opt);
    len += snprintf(buf+len, PAGE_SIZE-len, "gesture avg length: %d, crosstalk: %d\n", avg_length, crosstalk);
    len += snprintf(buf+len, PAGE_SIZE-len, "heart int time: %d\n", HS_INTT);

    return len;
}


/*----------------------------------------------------------------------------*/
static ssize_t epl8865_show_renvo(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    len += snprintf(buf+len, PAGE_SIZE-len, "%x",gRawData.renvo);
    return len;
}


/*----------------------------------------------------------------------------*/
static ssize_t epl8865_store_als_enable(struct device_driver *ddri, const char *buf, size_t count)
{
    uint16_t mode=0;
    struct epl8865_priv *obj = epl8865_obj;
    APS_FUN();

    sscanf(buf, "%hu",&mode);

    if(mode)
        set_bit(CMC_BIT_ALS, &obj->enable);
    else
        clear_bit(CMC_BIT_ALS, &obj->enable);

    epl8865_restart_polling();
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl8865_store_als_int_time(struct device_driver *ddri, const char *buf, size_t count)
{
    if(!epl8865_obj)
    {
        APS_ERR("epl8865_obj is null!!\n");
        return 0;
    }

    sscanf(buf, "%d", &ALS_INTT);
    APS_LOG("als int time is %d\n", ALS_INTT);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl8865_store_ps_enable(struct device_driver *ddri, const char *buf, size_t count)
{
    uint16_t mode=0;
    struct epl8865_priv *obj = epl8865_obj;
    APS_FUN();

    sscanf(buf, "%hu",&mode);

    if(mode)
        set_bit(CMC_BIT_PS, &obj->enable);
    else
        clear_bit(CMC_BIT_PS, &obj->enable);

    epl8865_restart_polling();
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl8865_show_ps_cal_raw(struct device_driver *ddri, char *buf)
{
    struct epl8865_priv *obj = epl8865_obj;
    u16 ch1;
    u32 ch1_all=0;
    int count =5;
    int i;
    ssize_t len = 0;

    if(!epl8865_obj)
    {
        APS_ERR("epl8865_obj is null!!\n");
        return 0;
    }


    set_bit(CMC_BIT_PS, &obj->enable);
    epl8865_restart_polling();

    for(i=0; i<count; i++)
    {
        //elan_epl2182_psensor_enable(obj, 1);
        msleep(PS_DELAY);
        if(obj->hw->polling_mode_ps == 0)
        {
            epl8865_I2C_Write(obj->client,REG_16,R_TWO_BYTE,0x01,0x00);
            epl8865_I2C_Read(obj->client);
            gRawData.ps_raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
#if LOW_GAIN_DATA_SHEET
	gRawData.ps_raw = gRawData.ps_raw >> 6;
    if(gRawData.ps_raw > 900)
       gRawData.ps_raw = 1023;
    if(gRawData.ps_raw < 10)
       gRawData.ps_raw = 0;
#endif

        }
        APS_LOG("epl8865_show_ps_cal_raw: gRawData.ps_raw=%d \r\n", gRawData.ps_raw);
		ch1_all = ch1_all+ gRawData.ps_raw;

    }

    ch1 = (u16)ch1_all/count;
	APS_LOG("epl8865_show_ps_cal_raw =  %d\n", ch1);

    len += snprintf(buf+len, PAGE_SIZE-len, "%d \r\n", ch1);

	return len;
}
static ssize_t epl8865_show_ps(struct device_driver *ddri, char *buf)
{
    struct epl8865_priv *obj = epl8865_obj;
    u16 ch1;
    u32 ch1_all=0;
    int count =5;
    int i;
    ssize_t len = 0;

    if(!epl8865_obj)
    {
        APS_ERR("epl8865_obj is null!!\n");
        return 0;
    }
	//add flank 2014/9/2
	set_bit(CMC_BIT_PS, &obj->enable);
    set_bit(CMC_BIT_ALS, &obj->enable);
    epl8865_restart_polling();
    msleep(50);
	//add flank 2014/9/2
 //   epl8865_read_ps(obj->client, &gRawData.ps_raw);  //modify flank 2014/9/2
    APS_LOG("epl8865_show_ps: gRawData.ps_raw=%d \r\n", gRawData.ps_raw);
    len += snprintf(buf+len, PAGE_SIZE-len, "%d \r\n", gRawData.ps_raw);
	return len;
}

static ssize_t epl8865_show_ps_threshold(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct epl8865_priv *obj = epl8865_obj;

    len += snprintf(buf+len, PAGE_SIZE-len, "gRawData.ps_als_factory(H/L): %d/%d \r\n", gRawData.ps_als_factory.ps_cal_h, gRawData.ps_als_factory.ps_cal_l);
    len += snprintf(buf+len, PAGE_SIZE-len, "ps_threshold(H/L): %d/%d \r\n", epl8865_obj->hw->ps_threshold_high, epl8865_obj->hw->ps_threshold_low);
    return len;
}



/*----------------------------------------------------------------------------*/
static ssize_t epl8865_store_ps_int_time(struct device_driver *ddri, const char *buf, size_t count)
{
    if(!epl8865_obj)
    {
        APS_ERR("epl8865_obj is null!!\n");
        return 0;
    }
    sscanf(buf, "%d", &PS_INTT);
    APS_LOG("ps int time is %d\n", PS_INTT);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl8865_store_ps_threshold(struct device_driver *ddri, const char *buf, size_t count)
{
    if(!epl8865_obj)
    {
        APS_ERR("epl8865_obj is null!!\n");
        return 0;
    }

    sscanf(buf, "%d,%d", &epl8865_obj->hw->ps_threshold_low, &epl8865_obj->hw->ps_threshold_high);
    gRawData.ps_als_factory.ps_cal_h = epl8865_obj->hw->ps_threshold_high;
    gRawData.ps_als_factory.ps_cal_l = epl8865_obj->hw->ps_threshold_low;

    epl8865_restart_polling();

    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl8865_store_ps_polling_mode(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl8865_priv *obj = epl8865_obj;
    struct i2c_client *client = obj->client;
    struct hwmsen_object obj_ps;

    obj_ps.self = obj;
    sscanf(buf, "%d",&obj->hw->polling_mode_ps);

    if(obj->hw->polling_mode_ps==0)
    {
        obj_ps.polling = 0;
        epl8865_setup_eint(client);
    }
    else
    {
        obj_ps.polling = 1;
    }
    return count;
}


/*----------------------------------------------------------------------------*/
static ssize_t epl8865_show_ges_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
    struct epl8865_priv *obj = epl8865_obj;
    bool ges_enable = test_bit(CMC_BIT_GES, &obj->enable);
	GES_LOG("epl8865_gesture_show ges_enable %d\n",ges_enable);
	return snprintf(buf, PAGE_SIZE, "%d\n", ges_enable);
}

static ssize_t epl8865_store_ges_enable(struct device_driver *ddri, const char *buf, size_t count)
{
    uint16_t mode=0;
    struct epl8865_priv *obj = epl8865_obj;
    APS_FUN();

    sscanf(buf, "%hu",&mode);

    if(mode)
        set_bit(CMC_BIT_GES, &obj->enable);
    else
        clear_bit(CMC_BIT_GES, &obj->enable);

    APS_LOG("enable %d\n", mode);

    is_work_triger=false;
    is_hold_triger=false;

    if(mode)
    {
        start_idx=0;
        ges_count=0;
        obj->ir_type = 0;
      //  gesture_calibration();  //parker 2014/9/2
    }

    epl8865_restart_polling();
    return count;
}


/*----------------------------------------------------------------------------*/
static ssize_t epl8865_show_ges_raws(struct device_driver *ddri, char *buf)
{
    u16 *tmp = (u16*)buf;
    u16 length= ges_count;
    int byte_count=2+length*2;
    int i=0;
    int start = 0;

    tmp[0]= length;
    for(i=start; i<length; i++)
    {
        tmp[i+1]= ges_raws[i];
        ges_raws[i]=0;
    }

    return byte_count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl8865_store_ges_int_time(struct device_driver *ddri, const char *buf, size_t count)
{
    sscanf(buf, "%d",&GES_INTT);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl8865_store_ges_cycle(struct device_driver *ddri, const char *buf, size_t count)
{
    sscanf(buf, "%d",&GES_CYCLE);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl8865_store_ges_opt(struct device_driver *ddri, const char *buf, size_t count)
{
    sscanf(buf, "%d",&opt);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl8865_store_ges_avg_length(struct device_driver *ddri, const char *buf, size_t count)
{
    sscanf(buf, "%d",&avg_length);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl8865_store_ges_debug(struct device_driver *ddri, const char *buf, size_t count)
{
    sscanf(buf, "%d",&GESTURE_DEBUG);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl8865_store_ges_work_threshold(struct device_driver *ddri, const char *buf, size_t count)
{
    sscanf(buf, "%d",&WORK_TH);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl8865_store_ges_polling_mode(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl8865_priv *epld = epl8865_obj;
    sscanf(buf, "%d",&epld->polling_mode_ges);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl8865_store_ges_gain(struct device_driver *ddri, const char *buf, size_t count)
{
    sscanf(buf, "%d",&GES_GAIN);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl8865_store_ges_mode(struct device_driver *ddri, const char *buf, size_t count)
{
    sscanf(buf, "%d",&ges_mode);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl8865_show_ges_mode_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{

	GES_LOG("epl8865_gesture_show ges_mode_curr_state %d\n",ges_mode_curr_state);
	return snprintf(buf, PAGE_SIZE, "%d\n", ges_mode_curr_state);
}

static ssize_t epl8865_store_ges_mode_state(struct device_driver *ddri, const char *buf, size_t count)
{
    sscanf(buf, "%d",&ges_mode_curr_state);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl8865_gesture_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	GES_LOG("epl8865_gesture_show gesture %d\n",gesture);
	return snprintf(buf, PAGE_SIZE, "%d\n", gesture);
}

static ssize_t epl8865_gesture_store(struct device_driver *ddri, const char *buf, size_t count)
{
    sscanf(buf, "%d",&gesture);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl8865_store_ges_crosstalk(struct device_driver *ddri, const char *buf, size_t count)
{
    sscanf(buf, "%d",&crosstalk);
    zoom_enabled = true;
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl8865_store_ges_zoom_delta(struct device_driver *ddri, const char *buf, size_t count)
{
    sscanf(buf, "%d",&zoom_delta);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl8865_store_hs_enable(struct device_driver *ddri, const char *buf, size_t count)
{
    uint16_t mode=0;
    struct epl8865_priv *obj = epl8865_obj;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
    APS_FUN();

    sscanf(buf, "%hu",&mode);

    if(mode){
        if(enable_als == true){
            atomic_set(&obj->als_suspend, 1);
            hs_als_flag = 1;
            if(obj->polling_mode_hs == 1)
                msleep(ALS_DELAY);
        }
        set_bit(CMC_BIT_HS, &obj->enable);
    }
    else{
        clear_bit(CMC_BIT_HS, &obj->enable);
        if(obj->polling_mode_hs == 1)
                msleep(HS_DELAY);

        if(hs_als_flag == 1){
            atomic_set(&obj->als_suspend, 0);
            hs_als_flag = 0;

        }

    }

    if(mode)
    {
        hs_idx=0;
        hs_count=0;
    }

    epl8865_restart_polling();
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl8865_show_hs_raws(struct device_driver *ddri, char *buf)
{
    mutex_lock(&sensor_mutex);

    u16 *tmp = (u16*)buf;
    u16 length= hs_count;
    int byte_count=2+length*2;
    int i=0;
    int start = hs_idx;

    tmp[0]= length;

    if(length == 0){
        tmp[0] = 1;
        length = 1;
        show_hs_raws_flag = 1;
    }
    for(i=start; i<length; i++){
        if(show_hs_raws_flag == 1){
            tmp[i+1] = 0;
            show_hs_raws_flag = 0;
        }
        else{
            tmp[i+1] = gRawData.hs_data[i];
        }

    }

    hs_count=0;
    hs_idx=0;
    mutex_unlock(&sensor_mutex);

    return byte_count;
}

static ssize_t epl8865_store_hs_polling(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl8865_priv *obj = epl8865_obj;
    APS_FUN();

    sscanf(buf, "%d",&(obj->polling_mode_hs));

    APS_LOG("HS polling mode: %d\n", obj->polling_mode_hs);

    return count;
}


/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(elan_status,					S_IRUGO  | S_IWUSR, epl8865_show_status,  	  		NULL										);
static DRIVER_ATTR(elan_reg,    					S_IRUGO  | S_IWUSR, epl8865_show_reg,   				NULL										);
static DRIVER_ATTR(elan_renvo,    				S_IRUGO  | S_IWUSR, epl8865_show_renvo,   				NULL										);
static DRIVER_ATTR(als_enable,					S_IRUGO  | S_IWUSR, NULL,   							epl8865_store_als_enable					);
static DRIVER_ATTR(als_int_time,     				S_IRUGO  | S_IWUSR, NULL,   					 		epl8865_store_als_int_time					);
static DRIVER_ATTR(ps_cal_raw, 				S_IRUGO  | S_IWUSR, epl8865_show_ps_cal_raw, 	  	NULL										);
static DRIVER_ATTR(ps, 				S_IRUGO  | S_IWUSR, epl8865_show_ps, 	  	NULL										);
static DRIVER_ATTR(ps_enable,					S_IRUGO  | S_IWUSR, NULL,   							epl8865_store_ps_enable					);
static DRIVER_ATTR(ps_int_time,     				S_IRUGO  | S_IWUSR, NULL,   							epl8865_store_ps_int_time					);
static DRIVER_ATTR(ps_threshold,     			S_IWUSR | S_IRUGO, epl8865_show_ps_threshold,   					 		epl8865_store_ps_threshold					);
static DRIVER_ATTR(ps_polling_mode,			S_IRUGO  | S_IWUSR, NULL,   							epl8865_store_ps_polling_mode				);
static DRIVER_ATTR(ges_raws, 					S_IRUGO  | S_IWUSR, epl8865_show_ges_raws, 	  		NULL										);
#ifdef BUILD_CTS
static DRIVER_ATTR(ges_enable,					S_IRUGO  | S_IWUSR, epl8865_show_ges_enable, epl8865_store_ges_enable);
#else
static DRIVER_ATTR(ges_enable,					0777, epl8865_show_ges_enable, epl8865_store_ges_enable);
#endif
static DRIVER_ATTR(ges_mode,				S_IRUGO  | S_IWUSR, NULL,								epl8865_store_ges_mode					);
static DRIVER_ATTR(ges_int_time,				S_IRUGO  | S_IWUSR, NULL,   					 		epl8865_store_ges_int_time					);
static DRIVER_ATTR(ges_polling_mode,			S_IRUGO  | S_IWUSR, NULL,   					 		epl8865_store_ges_polling_mode				);
static DRIVER_ATTR(ges_cycle,					S_IRUGO  | S_IWUSR, NULL,   					 		epl8865_store_ges_cycle						);
static DRIVER_ATTR(ges_opt,					S_IRUGO  | S_IWUSR, NULL,   					 		epl8865_store_ges_opt						);
static DRIVER_ATTR(ges_avg_length,				S_IRUGO  | S_IWUSR, NULL,   							epl8865_store_ges_avg_length				);
static DRIVER_ATTR(ges_debug,					S_IRUGO  | S_IWUSR, NULL,   					 		epl8865_store_ges_debug					);
static DRIVER_ATTR(ges_work_threshold,			S_IRUGO  | S_IWUSR, NULL,   							epl8865_store_ges_work_threshold			);
static DRIVER_ATTR(ges_gain,					S_IRUGO  | S_IWUSR, NULL,   							epl8865_store_ges_gain						);
static DRIVER_ATTR(ges_crosstalk,				S_IRUGO  | S_IWUSR, NULL,   							epl8865_store_ges_crosstalk					);
static DRIVER_ATTR(ges_zoom_delta,				S_IRUGO  | S_IWUSR, NULL,   							epl8865_store_ges_zoom_delta				);
#ifdef BUILD_CTS
static DRIVER_ATTR(gesture,   S_IRUGO | S_IWUSR, epl8865_gesture_show, epl8865_gesture_store);
static DRIVER_ATTR(ges_mode_state,   S_IRUGO | S_IWUSR, epl8865_show_ges_mode_state, epl8865_store_ges_mode_state);
#else
static DRIVER_ATTR(gesture,   0777, epl8865_gesture_show, epl8865_gesture_store);
static DRIVER_ATTR(ges_mode_state,   0777, epl8865_show_ges_mode_state, epl8865_store_ges_mode_state);
#endif
static DRIVER_ATTR(hs_enable,					S_IRUGO  | S_IWUSR, NULL,   							epl8865_store_hs_enable					);
static DRIVER_ATTR(hs_raws,					S_IRUGO  | S_IWUSR, epl8865_show_hs_raws, 	  		NULL										);
static DRIVER_ATTR(hs_polling,					S_IRUGO  | S_IWUSR, NULL, epl8865_store_hs_polling);

/*----------------------------------------------------------------------------*/
static struct driver_attribute * epl8865_attr_list[] =
{
    &driver_attr_elan_status,
    &driver_attr_elan_reg,
    &driver_attr_elan_renvo,
    &driver_attr_als_enable,
    &driver_attr_als_int_time,
    &driver_attr_ps_enable,
    &driver_attr_ps_cal_raw,
    &driver_attr_ps,
    &driver_attr_ps_int_time,
    &driver_attr_ps_threshold,
    &driver_attr_ps_polling_mode,
    &driver_attr_ges_cycle,
    &driver_attr_ges_enable,
    &driver_attr_ges_mode,
    &driver_attr_ges_int_time,
    &driver_attr_ges_polling_mode,
    &driver_attr_ges_opt,
    &driver_attr_ges_avg_length,
    &driver_attr_ges_debug,
    &driver_attr_ges_work_threshold,
    &driver_attr_ges_raws,
    &driver_attr_ges_gain,
    &driver_attr_ges_crosstalk,
    &driver_attr_ges_zoom_delta,
    &driver_attr_gesture,
    &driver_attr_ges_mode_state,
    &driver_attr_hs_enable,
    &driver_attr_hs_raws,
    &driver_attr_hs_polling,
};

/*----------------------------------------------------------------------------*/
static int epl8865_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(epl8865_attr_list)/sizeof(epl8865_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if(err = driver_create_file(driver, epl8865_attr_list[idx]))
        {
            APS_ERR("driver_create_file (%s) = %d\n", epl8865_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}



/*----------------------------------------------------------------------------*/
static int epl8865_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(epl8865_attr_list)/sizeof(epl8865_attr_list[0]));

    if (!driver)
        return -EINVAL;

    for (idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, epl8865_attr_list[idx]);
    }

    return err;
}



/******************************************************************************
 * Function Configuration
******************************************************************************/
static int epl8865_open(struct inode *inode, struct file *file)
{
    file->private_data = epl8865_i2c_client;

    APS_FUN();

    if (!file->private_data)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }

    return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int epl8865_release(struct inode *inode, struct file *file)
{
    APS_FUN();
    file->private_data = NULL;
    return 0;
}

/*----------------------------------------------------------------------------*/
static long epl8865_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct i2c_client *client = (struct i2c_client*)file->private_data;
    struct epl8865_priv *obj = i2c_get_clientdata(client);
    int err = 0;
    void __user *ptr = (void __user*) arg;
    int dat;
    uint32_t enable;

    clear_bit(CMC_BIT_GES, &obj->enable);

    switch (cmd)
    {
        case ALSPS_SET_PS_MODE:
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }

            if(enable)
            {
                set_bit(CMC_BIT_PS, &obj->enable);
            }
            else
            {
                clear_bit(CMC_BIT_PS, &obj->enable);
            }
            epl8865_restart_polling();
            break;


        case ALSPS_GET_PS_MODE:
            enable=test_bit(CMC_BIT_PS, &obj->enable);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;


        case ALSPS_GET_PS_DATA:
            dat = gRawData.ps_state;

            APS_LOG("ioctl ps state value = %d \n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;


        case ALSPS_GET_PS_RAW_DATA:
            dat = gRawData.ps_raw;

            APS_LOG("ioctl ps raw value = %d \n", dat);
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;


        case ALSPS_SET_ALS_MODE:
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            if(enable)
            {
                set_bit(CMC_BIT_ALS, &obj->enable);
                epl8865_restart_polling();
            }
            else
            {
                clear_bit(CMC_BIT_ALS, &obj->enable);
            }

            break;



        case ALSPS_GET_ALS_MODE:
            enable=test_bit(CMC_BIT_ALS, &obj->enable);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;



        case ALSPS_GET_ALS_DATA:
            dat = epl8865_get_als_value(obj, gRawData.als_ch1_raw);
            APS_LOG("ioctl get als data = %d\n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;


        case ALSPS_GET_ALS_RAW_DATA:
            dat = gRawData.als_ch1_raw;
            APS_LOG("ioctl get als raw data = %d\n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_PS_THRESHOLD_HIGH:
            dat = obj->hw ->ps_threshold_high;
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_PS_THRESHOLD_LOW:
            dat = obj->hw ->ps_threshold_low;
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        default:
            APS_ERR("%s not supported = 0x%04x", __func__, cmd);
            err = -ENOIOCTLCMD;
            break;
    }

err_out:
    return err;
}


/*----------------------------------------------------------------------------*/
static struct file_operations epl8865_fops =
{
    .owner = THIS_MODULE,
    .open = epl8865_open,
    .release = epl8865_release,
    .unlocked_ioctl = epl8865_unlocked_ioctl,
};


/*----------------------------------------------------------------------------*/
static struct miscdevice epl8865_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "als_ps",
    .fops = &epl8865_fops,
};


/*----------------------------------------------------------------------------*/
static int epl8865_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
    struct epl8865_priv *obj = i2c_get_clientdata(client);
    APS_FUN();

    if(msg.event == PM_EVENT_SUSPEND)
    {
        if(!obj)
        {
            APS_ERR("null pointer!!\n");
            return -EINVAL;
        }
		//mask by ELAN Robert at 2014/11/19
		#if 0
        atomic_set(&obj->als_suspend, 1);
        atomic_set(&obj->ps_suspend, 1);
        atomic_set(&obj->ges_suspend, 1);
        atomic_set(&obj->hs_suspend, 1);

        if(test_bit(CMC_BIT_PS,  &obj->enable) && obj->hw->polling_mode_ps==0)
            epl8865_restart_polling();
		#endif
        epl8865_power(obj->hw, 0);
    }

    return 0;

}



/*----------------------------------------------------------------------------*/
static int epl8865_i2c_resume(struct i2c_client *client)
{
    struct epl8865_priv *obj = i2c_get_clientdata(client);
    int err;
    APS_FUN();

    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }

    epl8865_power(obj->hw, 1);
//mask by ELAN Robert at 2014/11/19
#if 0
    msleep(50);

    atomic_set(&obj->ps_suspend, 0);

    if(err = epl8865_init_client(client))
    {
        APS_ERR("initialize client fail!!\n");
        return err;
    }

    if(obj->hw->polling_mode_ps == 0)
        epl8865_setup_eint(client);


    if(test_bit(CMC_BIT_PS,  &obj->enable))
        epl8865_restart_polling();
#endif
    return 0;
}


/*----------------------------------------------------------------------------*/
int epl8865_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
                       void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value;
    hwm_sensor_data* sensor_data;
    struct epl8865_priv *obj = (struct epl8865_priv *)self;

    APS_LOG("epl8865_ps_operate command = %x\n",command);
    switch (command)
    {
        case SENSOR_DELAY:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Set delay parameter error!\n");
                err = -EINVAL;
            }
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
                APS_LOG("ps enable = %d\n", value);


                if(value)
                {
                    if(obj->hw->polling_mode_ps==0)
                        gRawData.ps_state = 2;
                    set_bit(CMC_BIT_PS, &obj->enable);
                    wake_lock(&ps_lock);
                }
                else
                {
                    clear_bit(CMC_BIT_PS, &obj->enable);
                    wake_unlock(&ps_lock);
                }
                epl8865_restart_polling();
            }

            break;



        case SENSOR_GET_DATA:
            APS_LOG(" get ps data !!!!!!\n");
            if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
            {
                APS_ERR("get sensor data parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                APS_LOG("---SENSOR_GET_DATA---\n\n");

                sensor_data = (hwm_sensor_data *)buff_out;
                sensor_data->values[0] = gRawData.ps_state;
                sensor_data->values[1] = gRawData.ps_raw;
                sensor_data->value_divide = 1;
                sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
            }
            break;


        default:
            APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;



    }

    return err;

}



int epl8865_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
                        void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value;
    hwm_sensor_data* sensor_data;
    struct epl8865_priv *obj = (struct epl8865_priv *)self;

    APS_FUN();
    APS_LOG("epl8865_als_operate command = %x\n",command);

    switch (command)
    {
        case SENSOR_DELAY:
            if((buff_in == NULL) || (size_in < sizeof(int)))
            {
                APS_ERR("Set delay parameter error!\n");
                err = -EINVAL;
            }
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
                    set_bit(CMC_BIT_ALS, &obj->enable);
                    epl8865_restart_polling();
                }
                else
                {
                    clear_bit(CMC_BIT_ALS, &obj->enable);
                }

            }
            break;


        case SENSOR_GET_DATA:
            APS_LOG("get als data !!!!!!\n");
            if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
            {
                APS_ERR("get sensor data parameter error!\n");
                err = -EINVAL;
            }
            else
            {
                sensor_data = (hwm_sensor_data *)buff_out;
                sensor_data->values[0] = epl8865_get_als_value(obj, gRawData.als_ch1_raw);
                sensor_data->value_divide = 1;
                sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
                APS_LOG("get als data->values[0] = %d\n", sensor_data->values[0]);
            }
            break;

        default:
            APS_ERR("light sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;



    }

    return err;

}


/*----------------------------------------------------------------------------*/

static int epl8865_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
    strcpy(info->type, EPL8865_DEV_NAME);
    return 0;
}

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int als_open_report_data(int open)
{
  //should queuq work to report event if  is_report_input_direct=true
  return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int als_enable_nodata(int en)
{
  int res = 0;
 struct epl8865_priv *obj = epl8865_obj;
  APS_LOG("epl8865_obj als enable value = %d\n", en);
  if(en)
  {
    set_bit(CMC_BIT_ALS, &obj->enable);
    epl8865_restart_polling();
  }
  else
  {
    clear_bit(CMC_BIT_ALS, &obj->enable);
  }

  return 0;
}

static int als_set_delay(u64 ns)
{
  return 0;
}

static int als_get_data(int* value, int* status)
{
  int err = 0;
  struct epl8865_priv *obj = NULL;
  if(!epl8865_obj)
  {
    APS_ERR("epl8865_obj is null!!\n");
    return -1;
  }
  obj = epl8865_obj;
  {
    *value = epl8865_get_als_value(obj, gRawData.als_ch1_raw);
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;
  }

  return err;
}

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int ps_open_report_data(int open)
{
  //should queuq work to report event if  is_report_input_direct=true
  return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int ps_enable_nodata(int en)
{
  int res = 0;
  struct epl8865_priv *obj = NULL;
  if(!epl8865_obj)
  {
    APS_ERR("epl8865_obj is null!!\n");
    return -1;
  }
  obj = epl8865_obj;
  if(en)
  {
    if(obj->hw->polling_mode_ps==0)
      gRawData.ps_state = 2;
    set_bit(CMC_BIT_PS, &obj->enable);
  }
  else
  {
    clear_bit(CMC_BIT_PS, &obj->enable);
  }
  epl8865_restart_polling();
  return 0;

}

static int ps_set_delay(u64 ns)
{
  return 0;
}

static int ps_get_data(int* value, int* status)
{
  *value = gRawData.ps_state;
  *status = SENSOR_STATUS_ACCURACY_MEDIUM;
  return 0;
}

/*----------------------------------------------------------------------------*/
static int epl8865_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct epl8865_priv *obj;
    struct hwmsen_object obj_ps, obj_als;
    int err = 0, i=0;
    struct als_control_path als_ctl={0};
    struct als_data_path als_data={0};
    struct ps_control_path ps_ctl={0};
    struct ps_data_path ps_data={0};

    APS_FUN();

    epl8865_dumpReg(client);
    client->timing = 400;

    if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
    {
        err = -ENOMEM;
        goto exit;
    }

    memset(obj, 0, sizeof(*obj));

    epl8865_obj = obj;
    obj->hw = epl8865_get_cust_alsps_hw();

    epl8865_get_addr(obj->hw, &obj->addr);

    obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
    obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);
    BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
    memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
    BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
    memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));

    INIT_DELAYED_WORK(&obj->eint_work, epl8865_eint_work);
    INIT_DELAYED_WORK(&obj->polling_work, epl8865_polling_work);
	INIT_DELAYED_WORK(&obj->report_tp_work, epl8865_report_tp_work);


    obj->client = client;

    obj->gs_input_dev = input_allocate_device();
    set_bit(EV_KEY, obj->gs_input_dev->evbit);
    set_bit(EV_REL, obj->gs_input_dev->evbit);
    set_bit(EV_ABS, obj->gs_input_dev->evbit);
    set_bit(ABS_X, obj->gs_input_dev->absbit);
	set_bit(ABS_Y, obj->gs_input_dev->absbit);
	set_bit(ABS_PRESSURE, obj->gs_input_dev->absbit);
	set_bit(BTN_TOUCH, obj->gs_input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, obj->gs_input_dev->propbit);
	set_bit(ABS_DISTANCE, obj->gs_input_dev->absbit);
	set_bit(ABS_MT_TRACKING_ID, obj->gs_input_dev->absbit);
	set_bit(ABS_MT_TOUCH_MAJOR, obj->gs_input_dev->absbit);
	set_bit(ABS_MT_TOUCH_MINOR, obj->gs_input_dev->absbit);
	set_bit(ABS_MT_POSITION_X, obj->gs_input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, obj->gs_input_dev->absbit);

    obj->gs_input_dev->evbit[0] |= BIT_MASK(EV_REP);
    obj->gs_input_dev->keycodemax = 500;
    obj->gs_input_dev->name ="elan_gesture";

	input_set_abs_params(obj->gs_input_dev, ABS_DISTANCE, 0, 1, 0, 0);
	input_set_abs_params(obj->gs_input_dev, ABS_MT_POSITION_X, 0, 540, 0, 0);
	input_set_abs_params(obj->gs_input_dev, ABS_MT_POSITION_Y, 0, 960, 0, 0);
	input_set_abs_params(obj->gs_input_dev, ABS_MT_TOUCH_MAJOR, 0, 100, 0, 0);
	input_set_abs_params(obj->gs_input_dev, ABS_MT_TOUCH_MINOR, 0, 100, 0, 0);
	input_set_abs_params(obj->gs_input_dev, ABS_X, 0, 540, 0, 0);
	input_set_abs_params(obj->gs_input_dev, ABS_Y, 0, 960, 0, 0);
	input_abs_set_res(obj->gs_input_dev, ABS_X, 540);
	input_abs_set_res(obj->gs_input_dev, ABS_Y, 960);
	input_set_abs_params(obj->gs_input_dev, ABS_PRESSURE, 0, 255, 0, 0);

    for(i=0; i<EVENT_UNKNOWN; i++)
        obj->gs_input_dev->keybit[BIT_WORD(KEYCODE_ARRAY[i])] |= BIT_MASK(KEYCODE_ARRAY[i]);

    if (input_register_device(obj->gs_input_dev))
        APS_ERR("register input error\n");

    mutex_init(&sensor_mutex);


    i2c_set_clientdata(client, obj);

    atomic_set(&obj->trace, 0x00);
    atomic_set(&obj->als_suspend, 0);
    atomic_set(&obj->ps_suspend, 0);
    atomic_set(&obj->ges_suspend, 0);
    atomic_set(&obj->hs_suspend, 0);

    obj->polling_mode_ges = POLLING_MODE_GES;
    obj->lux_per_count = LUX_PER_COUNT;
    obj->enable = 0;
    obj->pending_intr = 0;
    obj->ir_type=0;

    epl8865_i2c_client = client;

    err =  epl8865_I2C_Write(client,REG_0,W_SINGLE_BYTE,0x02, EPL_S_SENSING_MODE);
    if(err < 0)
    {
      goto exit_init_failed;
    }
    err = epl8865_I2C_Write(client,REG_6,W_SINGLE_BYTE,0x02, EPL_INT_DISABLE);
    if(err < 0)
    {
      goto exit_init_failed;
    }

    err =  epl8865_I2C_Write(client,0x17,R_TWO_BYTE,0x01,0x00);
    if(err < 0)
    {
      goto exit_init_failed;
    }
    err = epl8865_I2C_Read(client);
    if(err < 0)
    {
      goto exit_init_failed;
    }
    gRawData.renvo = (gRawData.raw_bytes[1]<<8)|gRawData.raw_bytes[0];

    if(err = epl8865_init_client(client))
    {
        goto exit_init_failed;
    }


    if(err = misc_register(&epl8865_device))
    {
        APS_ERR("epl8865_device register failed\n");
        goto exit_misc_device_register_failed;
    }

    if(err = epl8865_create_attr(&(epl8865_init_info.platform_diver_addr->driver)))
    {
        APS_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }


    gRawData.ps_als_factory.cal_file_exist = 0;
    gRawData.ps_als_factory.cal_finished = 0;


    als_ctl.open_report_data= als_open_report_data;
    als_ctl.enable_nodata = als_enable_nodata;
    als_ctl.set_delay  = als_set_delay;
    als_ctl.is_report_input_direct = false;
    als_ctl.is_support_batch = obj->hw->is_batch_supported_als;

    err = als_register_control_path(&als_ctl);
    if(err)
    {
      APS_ERR("register fail = %d\n", err);
      goto exit_sensor_obj_attach_fail;
    }

    als_data.get_data = als_get_data;
    als_data.vender_div = 100;
    err = als_register_data_path(&als_data);
    if(err)
    {
      APS_ERR("tregister fail = %d\n", err);
      goto exit_sensor_obj_attach_fail;
    }


    ps_ctl.open_report_data= ps_open_report_data;
    ps_ctl.enable_nodata = ps_enable_nodata;
    ps_ctl.set_delay  = ps_set_delay;
    ps_ctl.is_report_input_direct = false;
    ps_ctl.is_support_batch = obj->hw->is_batch_supported_ps;

    err = ps_register_control_path(&ps_ctl);
    if(err)
    {
      APS_ERR("register fail = %d\n", err);
      goto exit_sensor_obj_attach_fail;
    }

    ps_data.get_data = ps_get_data;
    ps_data.vender_div = 100;
    err = ps_register_data_path(&ps_data);
    if(err)
    {
      APS_ERR("tregister fail = %d\n", err);
      goto exit_sensor_obj_attach_fail;
    }

    err = batch_register_support_info(ID_LIGHT,obj->hw->is_batch_supported_als,100, 0);
    if(err)
    {
      APS_ERR("register light batch support err = %d\n", err);
      goto exit_sensor_obj_attach_fail;
    }

    err = batch_register_support_info(ID_PROXIMITY,obj->hw->is_batch_supported_ps,100, 0);
    if(err)
    {
      APS_ERR("register proximity batch support err = %d\n", err);
      goto exit_sensor_obj_attach_fail;
    }

    if(obj->hw->polling_mode_ps ==0|| obj->polling_mode_ges==0 || obj->polling_mode_hs == 0)
        epl8865_setup_eint(client);

	//add by ELAN Robert at 2014/11/19 
	gRawData.ps_suspend = false;
	
/* Vanzo:yangzhihong on: Thu, 20 Nov 2014 10:01:03 +0800
 */
    wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "epl8865 wakelock");
// End of Vanzo:yangzhihong

  epl8865_init_flag = 0;
    APS_LOG("%s: OK\n", __func__);
    return 0;
exit_create_attr_failed:
exit_sensor_obj_attach_fail:
    misc_deregister(&epl8865_device);
exit_misc_device_register_failed:
exit_init_failed:
    //i2c_detach_client(client);
//	exit_kfree:
    kfree(obj);
exit:
    epl8865_i2c_client = NULL;

    epl8865_init_flag = -1;
    APS_ERR("%s: err = %d\n", __func__, err);
    return err;



}



/*----------------------------------------------------------------------------*/
static int epl8865_i2c_remove(struct i2c_client *client)
{
    int err;

    if(err = epl8865_delete_attr(&(epl8865_init_info.platform_diver_addr->driver)))
    {
        APS_ERR("epl8865_delete_attr fail: %d\n", err);
    }

    if(err = misc_deregister(&epl8865_device))
    {
        APS_ERR("misc_deregister fail: %d\n", err);
    }

    epl8865_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));

    return 0;
}

/*----------------------------------------------------------------------------*/
static int epl8865_remove(void)
{
    struct alsps_hw *hw = epl8865_get_cust_alsps_hw();

    APS_FUN();
    epl8865_power(hw, 0);
    i2c_del_driver(&epl8865_i2c_driver);
    return 0;
}
static int epl8865_local_init(void)
{
   struct alsps_hw *hw = epl8865_get_cust_alsps_hw();
  APS_FUN();

  epl8865_power(hw, 1);
  if(i2c_add_driver(&epl8865_i2c_driver))
  {
    APS_ERR("add driver error\n");
    return -1;
  }
  if(-1 == epl8865_init_flag)
  {
     return -1;
  }

  return 0;
}
/*----------------------------------------------------------------------------*/
static int __init epl8865_init(void)
{
    struct alsps_hw *hw = epl8865_get_cust_alsps_hw();
    APS_FUN();
    i2c_register_board_info(hw->i2c_num, &i2c_EPL8865, 1);
    alsps_driver_add(&epl8865_init_info);

    return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit epl8865_exit(void)
{
    APS_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(epl8865_init);
module_exit(epl8865_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("renato.pan@eminent-tek.com");
MODULE_DESCRIPTION("EPL8865 ALPsr driver");
MODULE_LICENSE("GPL");
