/* drivers/i2c/chips/sc7a30.c - SC7A30 motion sensor driver
 *
 *
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
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/completion.h>
#include <linux/kthread.h>
#include <linux/fs.h> 
#include <linux/fcntl.h>
#include <linux/syscalls.h>
#include "cust_acc.h"
#include "accel.h"
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include "sc7a30.h"
#include <linux/hwmsen_helper.h>
#include <cust_eint.h>
#include <mach/eint.h>
#ifdef MT6516
	#include <mach/mt6516_devs.h>
	#include <mach/mt6516_typedefs.h>
	#include <mach/mt6516_gpio.h>
	#include <mach/mt6516_pll.h>
#elif defined MT6573
	#include <mach/mt6573_devs.h>
	#include <mach/mt6573_typedefs.h>
	#include <mach/mt6573_gpio.h>
	#include <mach/mt6573_pll.h>
#elif defined MT6575
	#include <mach/mt6575_devs.h>
	#include <mach/mt6575_typedefs.h>
	#include <mach/mt6575_gpio.h>
	#include <mach/mt6575_pm_ldo.h>
#elif defined MT6577
	#include <mach/mt6577_devs.h>
	#include <mach/mt6577_typedefs.h>
	#include <mach/mt6577_gpio.h>
	#include <mach/mt6577_pm_ldo.h>
#else	
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif

#ifdef MT6516
	#define POWER_NONE_MACRO MT6516_POWER_NONE
#else
	#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#define POWER_NONE_MACRO MT65XX_POWER_NONE

#define SC7A30_EINT_MODE

/*----------------------------------------------------------------------------*/
//#define I2C_DRIVERID_SC7A30 345
/*----------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/
#define CONFIG_SC7A30_LOWPASS   /*apply low pass filter on output*/       
/*----------------------------------------------------------------------------*/
#define SC7A30_AXIS_X          0
#define SC7A30_AXIS_Y          1
#define SC7A30_AXIS_Z          2
#define SC7A30_AXES_NUM        3
#define SC7A30_DATA_LEN        6
#define SC7A30_DEV_NAME        "SC7A30"


#define SC7A30_ENABLE			1
#define SC7A30_XOUT_L			0x28
#define SC7A30_XOUT_H			0x29
#define SC7A30_YOUT_L			0x2A
#define SC7A30_YOUT_H			0x2B
#define SC7A30_ZOUT_L			0x2C
#define SC7A30_ZOUT_H			0x2D
#define SC7A30_MODE			0x20
#define SC7A30_MODE1			0x21
#define SC7A30_MODE2			0x22
#define SC7A30_MODE3			0x23
#define SC7A30_BOOT			0x24
#define SC7A30_STATUS			0x27
#define SC7A30_50HZ			0x40
#define SC7A30_100HZ			0x50
#define SC7A30_200HZ			0x60
#define SC7A30_400HZ			0x70
#define SC7A30_RANGE			2000000

#define CALIBRATION_NUM		20//40
#define AXIS_X_Y_RANGE_LIMIT	200
#define AXIS_X_Y_AVG_LIMIT	400
#define AXIS_Z_RANGE		200
#define AXIS_Z_DFT_G		1000
#define GOTO_CALI		100
#define FAILTO_CALI		101

#define SC7A30_PRECISION        12
#define SC7A30_BOUNDARY		(0x1 << (SC7A30_PRECISION - 1))
#define SC7A30_GRAVITY_STEP	(SC7A30_RANGE / SC7A30_BOUNDARY)
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id sc7a30_i2c_id[] = {{SC7A30_DEV_NAME,0},{}};
/*the adapter id will be available in customization*/
//static struct i2c_board_info __initdata i2c_sc7a30={ I2C_BOARD_INFO("SC7A30", SC7A30_I2C_SLAVE_ADDR>>1)};
static struct i2c_board_info __initdata i2c_sc7a30={ I2C_BOARD_INFO(SC7A30_DEV_NAME, 0x1d)};
//static unsigned short sc7a30_force[] = {0x00, SC7A30_I2C_SLAVE_ADDR, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const sc7a30_forces[] = { sc7a30_force, NULL };
//static struct i2c_client_address_data sc7a30_addr_data = { .forces = sc7a30_forces,};

/*----------------------------------------------------------------------------*/


extern struct acc_hw *get_cust_acc_hw(void);


static int sc7a30_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int sc7a30_i2c_remove(struct i2c_client *client);
static int sc7a30_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);

#ifndef CONFIG_HAS_EARLYSUSPEND
static int sc7a30_suspend(struct i2c_client *client, pm_message_t msg);
static int sc7a30_resume(struct i2c_client *client);
#endif

static int  sc7a30_local_init(void);
static int  sc7a30_remove(void);

//static int sc7a30_init_flag =-1; // 0<==>OK -1 <==> fail
static struct workqueue_struct *sc7a30_workqueue = NULL;//allen


static int sc7a30_acc_get_data( int *xyz);

struct SC7A30_acc{
    int    x;
    int    y;
    int    z;
} ;
/*----------------------------------------------------------------------------*/
typedef enum {
    ADX_TRC_FILTER  = 0x01,
    ADX_TRC_RAWDATA = 0x02,
    ADX_TRC_IOCTL   = 0x04,
    ADX_TRC_CALI	= 0X08,
    ADX_TRC_INFO	= 0X10,
} ADX_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor{
    u8  whole;
    u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
#define CHECK_CODE_SIZE 22 
static char stkcheckcode[CHECK_CODE_SIZE][CHECK_CODE_SIZE];
/*----------------------------------------------------------------------------*/
struct data_filter {
    s16 raw[C_MAX_FIR_LENGTH][SC7A30_AXES_NUM];
    int sum[SC7A30_AXES_NUM];
    int num;
    int idx;
};

/*------------------------------------------allen-------------------------------*/
static struct sensor_init_info sc7a30_init_info = {
	//	.name = "sc7a30",
	  .name = "SC7A30",
		.init = sc7a30_local_init,
		.uninit = sc7a30_remove,
	
};

/*----------------------------------------------------------------------------*/
struct sc7a30_i2c_data {
    struct i2c_client *client;
    struct acc_hw *hw;
    struct hwmsen_convert   cvt;
    
    /*misc*/
    struct work_struct	eint_work;				
    struct data_resolution *reso;
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
	atomic_t				filter;
    s16                     cali_sw[SC7A30_AXES_NUM+1];

  

    /*data*/
    s8                      offset[SC7A30_AXES_NUM+1];  /*+1: for 4-byte alignment*/
    s16                     data[SC7A30_AXES_NUM+1];

#if defined(CONFIG_SC7A30_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif 
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif     
};
/*----------------------------------------------------------------------------*/



static struct i2c_driver sc7a30_i2c_driver = {
    .driver = {
       // .owner          = THIS_MODULE,
        .name           = SC7A30_DEV_NAME,
    },
	.probe      		= sc7a30_i2c_probe,
	.remove    			= sc7a30_i2c_remove,
	//.detect				= sc7a30_i2c_detect,
#if !defined(CONFIG_HAS_EARLYSUSPEND)    
    .suspend            = sc7a30_suspend,
    .resume             = sc7a30_resume,
#endif
	.id_table = sc7a30_i2c_id,
//	.address_data = &sc7a30_addr_data,
//	 .address_list = (const unsigned short*)sc7a30_forces,
};


struct sc7a30_data_s {        
        struct i2c_client       *client;

		struct delayed_work		dwork;//allen
		int    time_of_cali;

} sc7a30_data;

struct Cali_Data {
	//mis p and n
	unsigned char xpmis; //x axis positive mismatch to write
	unsigned char xnmis; //x axis negtive mismatch to write
	unsigned char ypmis;
	unsigned char ynmis;
	unsigned char zpmis;
	unsigned char znmis;
	//off p and n
	unsigned char xpoff; //x axis positive offset to write
	unsigned char xnoff; //x axis negtive offset to write
	unsigned char ypoff;
	unsigned char ynoff;
	unsigned char zpoff;
	unsigned char znoff;
	//mid mis and off
	unsigned char xmmis; //x axis middle mismatch to write
	unsigned char ymmis; //y axis middle mismatch to write
	unsigned char zmmis; //z axis middle mismatch to write
	unsigned char xmoff; //x axis middle offset to write
	unsigned char ymoff; //y axis middle offset to write
	unsigned char zmoff; //z axis middle offset to write
	//output p and n
	signed int xpoutput; //x axis output of positive mismatch
	signed int xnoutput; //x axis output of negtive mismatch
	signed int ypoutput;
	signed int ynoutput;
	signed int zpoutput;
	signed int znoutput;	
	//output
	signed int xfoutput; //x axis the best or the temporary output
	signed int yfoutput; //y axis the best or the temporary output
	signed int zfoutput; //z axis the best or the temporary output
	//final and temp flag
	unsigned char xfinalf; //x axis final flag:if 1,calibration finished
	unsigned char yfinalf; //y axis final flag:if 1,calibration finished
	unsigned char zfinalf; //z axis final flag:if 1,calibration finished
	unsigned char xtempf;  //x axis temp flag:if 1,the step calibration finished
	unsigned char ytempf;  //y axis temp flag:if 1,the step calibration finished
	unsigned char ztempf;  //z axis temp flag:if 1,the step calibration finished
	
	unsigned char xaddmis;	//x axis mismtach register address
	unsigned char yaddmis;	//y axis mismtach register address
	unsigned char zaddmis;	//z axis mismtach register address
	unsigned char xaddoff;	//x axis offset register address
	unsigned char yaddoff;	//y axis offset register address
	unsigned char zaddoff;	//z axis offset register address
	
	
	unsigned char (*MisDataSpaceConvert)(unsigned char continuous);	//mismatch space convert function pointer
	unsigned char (*OffDataSpaceConvert)(unsigned char continuous);	//offset space convert function pointer
	
	
	
	
	
		
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *sc7a30_i2c_client = NULL;
//static struct platform_driver sc7a30_gsensor_driver;
static struct sc7a30_i2c_data *obj_i2c_data = NULL;
static bool sensor_power = false;
static GSENSOR_VECTOR3D gsensor_gain, gsensor_offset;
static char selftestRes[10] = {0};
static struct input_dev *sc7a30_input_dev;
static int impact_level;
/*----------------------------------------------------------------------------*/
#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               printk(KERN_INFO GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(KERN_ERR GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk(KERN_INFO GSE_TAG fmt, ##args)
/*----------------------------------------------------------------------------*/
static struct data_resolution sc7a30_data_resolution[] = {
 /*8 combination by {FULL_RES,RANGE}*/
    {{ 18, 0}, 56},  //refer to datasheet {{ 15, 6}, 64} /*+/-2g  in 7-bit resolution:  15.6 mg/LSB  1g is devided into 64 block*/ 
    {{ 62, 5}, 16},   /*+/-8g  in 7-bit resolution:  62.5 mg/LSB 1g is devided into 16 block*/
};
/*----------------------------------------------------------------------------*/
static struct data_resolution sc7a30_offset_resolution = {{15, 6}, 64};
 
int sc7a30_hwmsen_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
	u8 beg = addr; 
	struct i2c_msg msgs[2] = 
	{
		{
			.addr = client->addr,	 
			.flags = 0,
			.len = 1,				 
			.buf= &beg
		},
		{
			.addr = client->addr,	 
			.flags = I2C_M_RD,
			.len = len, 			 
			.buf = data,
		}
	};
	int err;

	if (!client)
		return -EINVAL;
	else if (len > C_I2C_FIFO_SIZE) 
	{		 
		GSE_LOG(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	if (err != 2) 
	{
		GSE_LOG("i2c_transfer error: (%d %p %d) %d\n", addr, data, len, err);
		err = -EIO;
	}
	else 
	{
		err = 0;/*no error*/
	}
	return err;
}


static int hwmsen_read_byte_sr(struct i2c_client *client, u8 addr, u8 *data)
{	
	return sc7a30_hwmsen_read_block(client, addr, data, 1);			

}



static void dumpReg(struct i2c_client *client)
{
  int i=0;
  u8 addr = 0x20;
  u8 regdata=0;
  for(i=0; i<3 ; i++)
  {
    //dump all
    hwmsen_read_byte_sr(client,addr,&regdata);
	GSE_LOG("Reg addr=%x regdata=%x\n",addr,regdata);
	addr++;
	
	
  }
}




static void SC7A30_power(struct acc_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;

	if(hw->power_id != POWER_NONE_MACRO)		// have externel LDO
	{        
		GSE_LOG("power %s\n", on ? "on" : "off");
		if(power_on == on)	// power status not change
		{
			GSE_LOG("ignore power control: %d\n", on);
		}
		else if(on)	// power on
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "SC7A30"))
			{
				GSE_ERR("power on fails!!\n");
			}
		}
		else	// power off
		{
			if (!hwPowerDown(hw->power_id, "SC7A30"))
			{
				GSE_ERR("power off fail!!\n");
			}			  
		}
	}
	power_on = on;    
}
/*----------------------------------------------------------------------------*/
static int SC7A30_SetDataResolution(struct sc7a30_i2c_data *obj)
{
	int err;
	u8  dat, reso;

	if(err = hwmsen_read_byte_sr(obj->client, SC7A30_REG_CTL_REG1, &dat))
	{
		GSE_ERR("write data format fail!!\n");
		return err;
	}

	/*the data_reso is combined by 3 bits: {FULL_RES, DATA_RANGE}*/
	reso  = (dat & SC7A30_RANGE_8G) ? (0x01) : (0x00);
	
	

	if(reso < sizeof(sc7a30_data_resolution)/sizeof(sc7a30_data_resolution[0]))
	{        
		obj->reso = &sc7a30_data_resolution[reso];
		return 0;
	}
	else
	{
		return -EINVAL;


	}
}
/*----------------------------------------------------------------------------*/
static int SC7A30_ReadData(struct i2c_client *client, s16 data[SC7A30_AXES_NUM])
{
	struct sc7a30_i2c_data *priv = i2c_get_clientdata(client);        
	u8 addr = SC7A30_REG_DATAX0;
	u8 buf[SC7A30_DATA_LEN] = {0};
	int err = 0;

	if(NULL == client)
	{
		err = -EINVAL;
	}
	
	else
	{
            
		if(hwmsen_read_byte_sr(client, 0x29,&buf[0]))
	    {
		   GSE_ERR("read X register err!\n");
		     return -1;
	    }
		
	    data[SC7A30_AXIS_X] = (s8)buf[0];//(s16)(buf[0]|0x0000);
	    if(hwmsen_read_byte_sr(client, 0x2b,&buf[0]))
	    {
		   GSE_ERR("read X register err!\n");
		    return -1;
	     }
	     data[SC7A30_AXIS_Y] = (s8)buf[0];//(s16)(buf[0]|0x0000);
	 
	    if(hwmsen_read_byte_sr(client, 0x2d, &buf[0]))
	    {
		  GSE_ERR("read X register err!\n");
		   return -1;
	  
	    }
	    data[SC7A30_AXIS_Z] = (s8)buf[0];//(s16)(buf[0]|0x0000);

		if(atomic_read(&priv->trace) & ADX_TRC_RAWDATA)
		{
			GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d]\n", data[SC7A30_AXIS_X], data[SC7A30_AXIS_Y], data[SC7A30_AXIS_Z],
		                               data[SC7A30_AXIS_X], data[SC7A30_AXIS_Y], data[SC7A30_AXIS_Z]);
		}
#ifdef CONFIG_SC7A30_LOWPASS
		if(atomic_read(&priv->filter))
		{
			if(atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend))
			{
				int idx, firlen = atomic_read(&priv->firlen);   
				if(priv->fir.num < firlen)
				{                
					priv->fir.raw[priv->fir.num][SC7A30_AXIS_X] = data[SC7A30_AXIS_X];
					priv->fir.raw[priv->fir.num][SC7A30_AXIS_Y] = data[SC7A30_AXIS_Y];
					priv->fir.raw[priv->fir.num][SC7A30_AXIS_Z] = data[SC7A30_AXIS_Z];
					priv->fir.sum[SC7A30_AXIS_X] += data[SC7A30_AXIS_X];
					priv->fir.sum[SC7A30_AXIS_Y] += data[SC7A30_AXIS_Y];
					priv->fir.sum[SC7A30_AXIS_Z] += data[SC7A30_AXIS_Z];
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
							priv->fir.raw[priv->fir.num][SC7A30_AXIS_X], priv->fir.raw[priv->fir.num][SC7A30_AXIS_Y], priv->fir.raw[priv->fir.num][SC7A30_AXIS_Z],
							priv->fir.sum[SC7A30_AXIS_X], priv->fir.sum[SC7A30_AXIS_Y], priv->fir.sum[SC7A30_AXIS_Z]);
					}
					priv->fir.num++;
					priv->fir.idx++;
				}
				else
				{
					idx = priv->fir.idx % firlen;
					priv->fir.sum[SC7A30_AXIS_X] -= priv->fir.raw[idx][SC7A30_AXIS_X];
					priv->fir.sum[SC7A30_AXIS_Y] -= priv->fir.raw[idx][SC7A30_AXIS_Y];
					priv->fir.sum[SC7A30_AXIS_Z] -= priv->fir.raw[idx][SC7A30_AXIS_Z];
					priv->fir.raw[idx][SC7A30_AXIS_X] = data[SC7A30_AXIS_X];
					priv->fir.raw[idx][SC7A30_AXIS_Y] = data[SC7A30_AXIS_Y];
					priv->fir.raw[idx][SC7A30_AXIS_Z] = data[SC7A30_AXIS_Z];
					priv->fir.sum[SC7A30_AXIS_X] += data[SC7A30_AXIS_X];
					priv->fir.sum[SC7A30_AXIS_Y] += data[SC7A30_AXIS_Y];
					priv->fir.sum[SC7A30_AXIS_Z] += data[SC7A30_AXIS_Z];
					priv->fir.idx++;
					data[SC7A30_AXIS_X] = priv->fir.sum[SC7A30_AXIS_X]/firlen;
					data[SC7A30_AXIS_Y] = priv->fir.sum[SC7A30_AXIS_Y]/firlen;
					data[SC7A30_AXIS_Z] = priv->fir.sum[SC7A30_AXIS_Z]/firlen;
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
						priv->fir.raw[idx][SC7A30_AXIS_X], priv->fir.raw[idx][SC7A30_AXIS_Y], priv->fir.raw[idx][SC7A30_AXIS_Z],
						priv->fir.sum[SC7A30_AXIS_X], priv->fir.sum[SC7A30_AXIS_Y], priv->fir.sum[SC7A30_AXIS_Z],
						data[SC7A30_AXIS_X], data[SC7A30_AXIS_Y], data[SC7A30_AXIS_Z]);
					}
				}
			}
		}	
#endif         
	}
	return err;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int SC7A30_ResetCalibration(struct i2c_client *client)
{
	struct sc7a30_i2c_data *obj = i2c_get_clientdata(client);	

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	return 0;     
}
/*----------------------------------------------------------------------------*/
static int SC7A30_ReadCalibration(struct i2c_client *client, int dat[SC7A30_AXES_NUM])
{
    struct sc7a30_i2c_data *obj = i2c_get_clientdata(client);

    dat[obj->cvt.map[SC7A30_AXIS_X]] = obj->cvt.sign[SC7A30_AXIS_X]*obj->cali_sw[SC7A30_AXIS_X];
    dat[obj->cvt.map[SC7A30_AXIS_Y]] = obj->cvt.sign[SC7A30_AXIS_Y]*obj->cali_sw[SC7A30_AXIS_Y];
    dat[obj->cvt.map[SC7A30_AXIS_Z]] = obj->cvt.sign[SC7A30_AXIS_Z]*obj->cali_sw[SC7A30_AXIS_Z];                        
                                       
    return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static int SC7A30_WriteCalibration(struct i2c_client *client, int dat[SC7A30_AXES_NUM])
{
	struct sc7a30_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	int cali[SC7A30_AXES_NUM];


	GSE_FUN();
	if(!obj || ! dat)
	{
		GSE_ERR("null ptr!!\n");
		return -EINVAL;
	}
	else
	{        
		s16 cali[SC7A30_AXES_NUM];
		cali[obj->cvt.map[SC7A30_AXIS_X]] = obj->cvt.sign[SC7A30_AXIS_X]*obj->cali_sw[SC7A30_AXIS_X];
		cali[obj->cvt.map[SC7A30_AXIS_Y]] = obj->cvt.sign[SC7A30_AXIS_Y]*obj->cali_sw[SC7A30_AXIS_Y];
		cali[obj->cvt.map[SC7A30_AXIS_Z]] = obj->cvt.sign[SC7A30_AXIS_Z]*obj->cali_sw[SC7A30_AXIS_Z]; 
		cali[SC7A30_AXIS_X] += dat[SC7A30_AXIS_X];
		cali[SC7A30_AXIS_Y] += dat[SC7A30_AXIS_Y];
		cali[SC7A30_AXIS_Z] += dat[SC7A30_AXIS_Z];

		obj->cali_sw[SC7A30_AXIS_X] += obj->cvt.sign[SC7A30_AXIS_X]*dat[obj->cvt.map[SC7A30_AXIS_X]];
        obj->cali_sw[SC7A30_AXIS_Y] += obj->cvt.sign[SC7A30_AXIS_Y]*dat[obj->cvt.map[SC7A30_AXIS_Y]];
        obj->cali_sw[SC7A30_AXIS_Z] += obj->cvt.sign[SC7A30_AXIS_Z]*dat[obj->cvt.map[SC7A30_AXIS_Z]];
	} 

	return err;
}
/*----------------------------------------------------------------------------*/
static int SC7A30_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[10];    
	int res = 0;
	return SC7A30_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int SC7A30_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2];    
	int res = 0;
	u8 addr = SC7A30_REG_CTL_REG1;
	struct sc7a30_i2c_data *obj = i2c_get_clientdata(client);
	
	
	if(enable == sensor_power)
	{
		GSE_LOG("Sensor power status is newest!\n");
		return SC7A30_SUCCESS;
	}

	if(hwmsen_read_byte_sr(client, addr, &databuf[0]))
	{
		GSE_ERR("read power ctl register err!\n");
		return SC7A30_ERR_I2C;
	}

	databuf[0] &= ~SC7A30_MEASURE_MODE;
	
	if(enable == TRUE)
	{
		databuf[0] |= SC7A30_MEASURE_MODE;
	}
	else
	{
		// do nothing
	}
	databuf[1] = databuf[0];
	databuf[0] = SC7A30_REG_CTL_REG1;
	

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		GSE_LOG("set power mode failed!\n");
		return SC7A30_ERR_I2C;
	}
	else if(atomic_read(&obj->trace) & ADX_TRC_INFO)
	{
		GSE_LOG("set power mode ok %d!\n", databuf[1]);
	}

	sensor_power = enable;

	mdelay(20);
	
	return SC7A30_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int SC7A30_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct sc7a30_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10];
	u8 addr = SC7A30_REG_CTL_REG1;
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);

	if(hwmsen_read_byte_sr(client, addr, &databuf[0]))
	{
		GSE_ERR("read reg_ctl_reg1 register err!\n");
		return SC7A30_ERR_I2C;
	}

	databuf[0] &= ~SC7A30_RANGE_8G;
	
	if(SC7A30_RANGE_8G == dataformat)
	{
		databuf[0] |= SC7A30_RANGE_8G;
	}
	else
	{
	    GSE_LOG("set 2g range\n");
		// do nothing
	}
	databuf[1] = databuf[0];
	databuf[0] = SC7A30_REG_CTL_REG1;
	

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return SC7A30_ERR_I2C;
	}
	

	return SC7A30_SetDataResolution(obj);    
}
/*----------------------------------------------------------------------------*/
static int SC7A30_SetBWRate(struct i2c_client *client, u8 bwrate)
{
	u8 databuf[10];
	u8 addr = SC7A30_REG_CTL_REG1;
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);
	
	if(hwmsen_read_byte_sr(client, addr, &databuf[0]))
	{
		GSE_ERR("read reg_ctl_reg1 register err!\n");
		return SC7A30_ERR_I2C;
	}

	databuf[0] &= ~SC7A30_BW_400HZ;
	
	if(SC7A30_BW_400HZ == bwrate)
	{
		databuf[0] |= SC7A30_BW_400HZ;
	}
	else
	{
	     GSE_LOG("set DataRate 100Hz\n");
		// do nothing
	}
	databuf[1] = databuf[0];
	databuf[0] = SC7A30_REG_CTL_REG1;

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return SC7A30_ERR_I2C;
	}
	
	return SC7A30_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
//enalbe data ready interrupt
static int SC7A30_SetIntEnable(struct i2c_client *client, u8 intenable)
{
	u8 databuf[10];
	u8 addr = SC7A30_REG_CTL_REG1;
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10); 

	if(hwmsen_read_byte_sr(client, addr, &databuf[0]))
	{
		GSE_ERR("read reg_ctl_reg1 register err!\n");
		return SC7A30_ERR_I2C;
	}

	databuf[0] &= ~SC7A30_DATA_READY;
	
	if(SC7A30_DATA_READY == intenable)
	{
		databuf[0] |= SC7A30_DATA_READY;
	}
	else
	{
	     GSE_LOG("Disable dataready INT\n");
		// do nothing
	}
	databuf[1] = databuf[0];
	databuf[0] = SC7A30_REG_CTL_REG1;
	
	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return SC7A30_ERR_I2C;
	}
	
	return SC7A30_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int SC7A30_Init(struct i2c_client *client, int reset_cali)
{
	struct sc7a30_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;
/*
	res = SC7A30_CheckDeviceID(client); 
	if(res != SC7A30_SUCCESS)
	{
		return res;
	}	
*/
    // first clear reg1
    res = hwmsen_write_byte(client,SC7A30_REG_CTL_REG1,0x00);
	if(res != SC7A30_SUCCESS)
	{
		return res;
	}


	res = SC7A30_SetPowerMode(client, false);
	if(res != SC7A30_SUCCESS)
	{
		return res;
	}
	

	res = SC7A30_SetBWRate(client, ~SC7A30_BW_400HZ);//400 or 100 no other choice
	if(res != SC7A30_SUCCESS )
	{
		return res;
	}

	res = SC7A30_SetDataFormat(client, ~SC7A30_RANGE_8G);//8g or 2G no oher choise
	if(res != SC7A30_SUCCESS) 
	{
		return res;
	}
	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;

	/*res = SC7A30_SetIntEnable(client, SC7A30_DATA_READY);        
	if(res != SC7A30_SUCCESS)
	{
		return res;
	}*/
	

	if(NULL != reset_cali)
	{ 
		//reset calibration only in power on
		res = SC7A30_ResetCalibration(client);
		if(res != SC7A30_SUCCESS)
		{
			return res;
		}
	}

#ifdef SC7A30_EINT_MODE
	res = hwmsen_write_byte(client,0x25,0x00);	
	res = hwmsen_write_byte(client,0x21,0x0d);
	res = hwmsen_write_byte(client,0x3a,0x20);    //设置碰撞等级
	res = hwmsen_write_byte(client,0x3b,0x7f);	
	res = hwmsen_write_byte(client,0x3c,0x60);   	
	res = hwmsen_write_byte(client,0x38,0x15);	
	res = hwmsen_write_byte(client,0x22,0x80);
	res = hwmsen_write_byte(client,0x20,0x37);	
	res = hwmsen_write_byte(client,0x23,0x80);
#endif

	printk("sc7a30_init_client OK!\n");
#ifdef CONFIG_SC7A30_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif

	return SC7A30_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int SC7A30_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];    

	memset(databuf, 0, sizeof(u8)*10);

	if((NULL == buf)||(bufsize<=30))
	{
		return -1;
	}
	
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "SC7A30 Chip");
	return 0;
}
/*----------------------------------------------------------------------------*/
static int SC7A30_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	struct sc7a30_i2c_data *obj = (struct sc7a30_i2c_data*)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[SC7A30_AXES_NUM];
	int res = 0;
	memset(databuf, 0, sizeof(u8)*10);

	if(NULL == buf)
	{
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	if(sensor_power == FALSE)
	{
		res = SC7A30_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on sc7a30 error %d!\n", res);
		}
		msleep(20);
	}

	if(res = SC7A30_ReadData(client, obj->data))
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{
		obj->data[SC7A30_AXIS_X] += obj->cali_sw[SC7A30_AXIS_X];
		obj->data[SC7A30_AXIS_Y] += obj->cali_sw[SC7A30_AXIS_Y];
		obj->data[SC7A30_AXIS_Z] += obj->cali_sw[SC7A30_AXIS_Z];
		
		/*remap coordinate*/
		acc[obj->cvt.map[SC7A30_AXIS_X]] = obj->cvt.sign[SC7A30_AXIS_X]*obj->data[SC7A30_AXIS_X];
		acc[obj->cvt.map[SC7A30_AXIS_Y]] = obj->cvt.sign[SC7A30_AXIS_Y]*obj->data[SC7A30_AXIS_Y];
		acc[obj->cvt.map[SC7A30_AXIS_Z]] = obj->cvt.sign[SC7A30_AXIS_Z]*obj->data[SC7A30_AXIS_Z];

		//GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[SC7A30_AXIS_X], acc[SC7A30_AXIS_Y], acc[SC7A30_AXIS_Z]);
		/// printk("  sc7a30 Mapped gsensor data: %d, %d, %d!\n", acc[SC7A30_AXIS_X], acc[SC7A30_AXIS_Y], acc[SC7A30_AXIS_Z]);
		//Out put the mg
		acc[SC7A30_AXIS_X] = acc[SC7A30_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[SC7A30_AXIS_Y] = acc[SC7A30_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[SC7A30_AXIS_Z] = acc[SC7A30_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;		
		

		sprintf(buf, "%04x %04x %04x", acc[SC7A30_AXIS_X], acc[SC7A30_AXIS_Y], acc[SC7A30_AXIS_Z]);
		if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)//atomic_read(&obj->trace) & ADX_TRC_IOCTL
		{
			GSE_LOG("gsensor data: %s!\n", buf);
			dumpReg(client);
		}
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int SC7A30_ReadRawData(struct i2c_client *client, char *buf)
{
	struct sc7a30_i2c_data *obj = (struct sc7a30_i2c_data*)i2c_get_clientdata(client);
	int res = 0;

	if (!buf || !client)
	{
		return EINVAL;
	}
	
	if(res = SC7A30_ReadData(client, obj->data))
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return EIO;
	}
	else
	{
		sprintf(buf, "%04x %04x %04x", obj->data[SC7A30_AXIS_X], 
			obj->data[SC7A30_AXIS_Y], obj->data[SC7A30_AXIS_Z]);
	
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int SC7A30_InitSelfTest(struct i2c_client *client)
{
	int res = 0;
	u8  data;

	res = SC7A30_SetBWRate(client, SC7A30_BW_400HZ);
	if(res != SC7A30_SUCCESS ) 
	{
		return res;
	}
	
	res = hwmsen_read_byte_sr(client, SC7A30_REG_CTL_REG1, &data);
	if(res != SC7A30_SUCCESS)
	{
		return res;
	}
	

	res = SC7A30_SetDataFormat(client, SC7A30_SELF_TEST|data);
	if(res != SC7A30_SUCCESS) 
	{
		return res;
	}
	
	res = SC7A30_SetPowerMode(client,true);
	if(res != SC7A30_SUCCESS) 
	{
		return res;
	}
	
	return SC7A30_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int SC7A30_JudgeTestResult(struct i2c_client *client, s32 prv[SC7A30_AXES_NUM], s32 nxt[SC7A30_AXES_NUM])
{
    struct criteria {
        int min;
        int max;
    };
	
    struct criteria self[4][3] = {
        {{-35, 5}, {-35, 5}, {-40, -3}},//test range 8g
        {{-35, 5}, {-35, 5}, {-40, -3}},//test range 2g
                   
    };
    struct criteria (*ptr)[3] = NULL;
    u8 format;
    int res;
    if(res = hwmsen_read_byte_sr(client, SC7A30_REG_CTL_REG1, &format))
        return res;
    if(format & SC7A30_RANGE_8G)
        ptr = &self[0];
    else 
        ptr = &self[1];
    

    if (!ptr) {
        GSE_ERR("null pointer\n");
        return -EINVAL;
    }

    if (((nxt[SC7A30_AXIS_X] - prv[SC7A30_AXIS_X]) > (*ptr)[SC7A30_AXIS_X].max) ||
        ((nxt[SC7A30_AXIS_X] - prv[SC7A30_AXIS_X]) < (*ptr)[SC7A30_AXIS_X].min)) {
        GSE_ERR("X is over range\n");
        res = -EINVAL;
    }
    if (((nxt[SC7A30_AXIS_Y] - prv[SC7A30_AXIS_Y]) > (*ptr)[SC7A30_AXIS_Y].max) ||
        ((nxt[SC7A30_AXIS_Y] - prv[SC7A30_AXIS_Y]) < (*ptr)[SC7A30_AXIS_Y].min)) {
        GSE_ERR("Y is over range\n");
        res = -EINVAL;
    }
    if (((nxt[SC7A30_AXIS_Z] - prv[SC7A30_AXIS_Z]) > (*ptr)[SC7A30_AXIS_Z].max) ||
        ((nxt[SC7A30_AXIS_Z] - prv[SC7A30_AXIS_Z]) < (*ptr)[SC7A30_AXIS_Z].min)) {
        GSE_ERR("Z is over range\n");
        res = -EINVAL;
    }
    return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = sc7a30_i2c_client;
	char strbuf[SC7A30_BUFSIZE];
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	SC7A30_ReadChipInfo(client, strbuf, SC7A30_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = sc7a30_i2c_client;
	char strbuf[SC7A30_BUFSIZE];
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	SC7A30_ReadSensorData(client, strbuf, SC7A30_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);            
}
/*----------------------------------------------------------------------------*/
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = sc7a30_i2c_client;
	struct sc7a30_i2c_data *obj;

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);

	int err, len = 0, mul;
	int tmp[SC7A30_AXES_NUM];

	
	if(err = SC7A30_ReadCalibration(client, tmp))
	{
		return -EINVAL;
	}
	else
	{    
		mul = obj->reso->sensitivity/sc7a30_offset_resolution.sensitivity;
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,                        
			obj->offset[SC7A30_AXIS_X], obj->offset[SC7A30_AXIS_Y], obj->offset[SC7A30_AXIS_Z],
			obj->offset[SC7A30_AXIS_X], obj->offset[SC7A30_AXIS_Y], obj->offset[SC7A30_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1, 
			obj->cali_sw[SC7A30_AXIS_X], obj->cali_sw[SC7A30_AXIS_Y], obj->cali_sw[SC7A30_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n", 
			obj->offset[SC7A30_AXIS_X]*mul + obj->cali_sw[SC7A30_AXIS_X],
			obj->offset[SC7A30_AXIS_Y]*mul + obj->cali_sw[SC7A30_AXIS_Y],
			obj->offset[SC7A30_AXIS_Z]*mul + obj->cali_sw[SC7A30_AXIS_Z],
			tmp[SC7A30_AXIS_X], tmp[SC7A30_AXIS_Y], tmp[SC7A30_AXIS_Z]);
		
		return len;
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, char *buf, size_t count)
{
	struct i2c_client *client = sc7a30_i2c_client;  
	int err, x, y, z;
	int dat[SC7A30_AXES_NUM];

	if(!strncmp(buf, "rst", 3))
	{
		if(err = SC7A30_ResetCalibration(client))
		{
			GSE_ERR("reset offset err = %d\n", err);
		}	
	}
	else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z))
	{
		dat[SC7A30_AXIS_X] = x;
		dat[SC7A30_AXIS_Y] = y;
		dat[SC7A30_AXIS_Z] = z;
		if(err = SC7A30_WriteCalibration(client, dat))
		{
			GSE_ERR("write calibration err = %d\n", err);
		}		
	}
	else
	{
		GSE_ERR("invalid format\n");
	}
	
	return count;
}
/*----------------------------------------------------------------------------*/

static ssize_t show_power_status(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = sc7a30_i2c_client;
	struct sc7a30_i2c_data *obj;
	u8 data;

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);
	hwmsen_read_byte_sr(client,SC7A30_REG_CTL_REG1,&data);

    return snprintf(buf, PAGE_SIZE, "%x\n", data);
}

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static ssize_t show_selftest_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = sc7a30_i2c_client;
	struct sc7a30_i2c_data *obj;
	int result =0;
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	GSE_LOG("fwq  selftestRes value =%s\n",selftestRes); 
	return snprintf(buf, 10, "%s\n", selftestRes);
}

/*----------------------------------------------------------------------------*/
static ssize_t store_selftest_value(struct device_driver *ddri, char *buf, size_t count)
{
	struct item{
	s16 raw[SC7A30_AXES_NUM];
	};
	
	struct i2c_client *client = sc7a30_i2c_client;  
	int idx, res, num;
	struct item *prv = NULL, *nxt = NULL;
	s32 avg_prv[SC7A30_AXES_NUM] = {0, 0, 0};
	s32 avg_nxt[SC7A30_AXES_NUM] = {0, 0, 0};


	if(1 != sscanf(buf, "%d", &num))
	{
		GSE_ERR("parse number fail\n");
		return count;
	}
	else if(num == 0)
	{
		GSE_ERR("invalid data count\n");
		return count;
	}

	prv = kzalloc(sizeof(*prv) * num, GFP_KERNEL);
	nxt = kzalloc(sizeof(*nxt) * num, GFP_KERNEL);
	if (!prv || !nxt)
	{
		goto exit;
	}


    res = SC7A30_SetPowerMode(client,true);
	if(res != SC7A30_SUCCESS ) //
	{
		return res;
	}
	msleep(20);
	
	GSE_LOG("NORMAL:\n");
	for(idx = 0; idx < num; idx++)
	{
		if(res = SC7A30_ReadData(client, prv[idx].raw))
		{            
			GSE_ERR("read data fail: %d\n", res);
			goto exit;
		}
		
		avg_prv[SC7A30_AXIS_X] += prv[idx].raw[SC7A30_AXIS_X];
		avg_prv[SC7A30_AXIS_Y] += prv[idx].raw[SC7A30_AXIS_Y];
		avg_prv[SC7A30_AXIS_Z] += prv[idx].raw[SC7A30_AXIS_Z];        
		GSE_LOG("[%5d %5d %5d]\n", prv[idx].raw[SC7A30_AXIS_X], prv[idx].raw[SC7A30_AXIS_Y], prv[idx].raw[SC7A30_AXIS_Z]);
	}
	
	avg_prv[SC7A30_AXIS_X] /= num;
	avg_prv[SC7A30_AXIS_Y] /= num;
	avg_prv[SC7A30_AXIS_Z] /= num;  

	res = SC7A30_SetPowerMode(client,false);
	if(res != SC7A30_SUCCESS ) //
	{
		return res;
	}

	/*initial setting for self test*/
	SC7A30_InitSelfTest(client);
	msleep(50);
	GSE_LOG("SELFTEST:\n");    
	for(idx = 0; idx < num; idx++)
	{
		if(res = SC7A30_ReadData(client, nxt[idx].raw))
		{            
			GSE_ERR("read data fail: %d\n", res);
			goto exit;
		}
		avg_nxt[SC7A30_AXIS_X] += nxt[idx].raw[SC7A30_AXIS_X];
		avg_nxt[SC7A30_AXIS_Y] += nxt[idx].raw[SC7A30_AXIS_Y];
		avg_nxt[SC7A30_AXIS_Z] += nxt[idx].raw[SC7A30_AXIS_Z];        
		GSE_LOG("[%5d %5d %5d]\n", nxt[idx].raw[SC7A30_AXIS_X], nxt[idx].raw[SC7A30_AXIS_Y], nxt[idx].raw[SC7A30_AXIS_Z]);
	}
	
	avg_nxt[SC7A30_AXIS_X] /= num;
	avg_nxt[SC7A30_AXIS_Y] /= num;
	avg_nxt[SC7A30_AXIS_Z] /= num;    

	GSE_LOG("X: %5d - %5d = %5d \n", avg_nxt[SC7A30_AXIS_X], avg_prv[SC7A30_AXIS_X], avg_nxt[SC7A30_AXIS_X] - avg_prv[SC7A30_AXIS_X]);
	GSE_LOG("Y: %5d - %5d = %5d \n", avg_nxt[SC7A30_AXIS_Y], avg_prv[SC7A30_AXIS_Y], avg_nxt[SC7A30_AXIS_Y] - avg_prv[SC7A30_AXIS_Y]);
	GSE_LOG("Z: %5d - %5d = %5d \n", avg_nxt[SC7A30_AXIS_Z], avg_prv[SC7A30_AXIS_Z], avg_nxt[SC7A30_AXIS_Z] - avg_prv[SC7A30_AXIS_Z]); 

	if(!SC7A30_JudgeTestResult(client, avg_prv, avg_nxt))
	{
		GSE_LOG("SELFTEST : PASS\n");
		strcpy(selftestRes,"y");
	}	
	else
	{
		GSE_LOG("SELFTEST : FAIL\n");
		strcpy(selftestRes,"n");
	}
	
	exit:
	/*restore the setting*/    
	SC7A30_Init(client, 0);
	kfree(prv);
	kfree(nxt);
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_SC7A30_LOWPASS
	struct i2c_client *client = sc7a30_i2c_client;
	struct sc7a30_i2c_data *obj = i2c_get_clientdata(client);
	if(atomic_read(&obj->firlen))
	{
		int idx, len = atomic_read(&obj->firlen);
		GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for(idx = 0; idx < len; idx++)
		{
			GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][SC7A30_AXIS_X], obj->fir.raw[idx][SC7A30_AXIS_Y], obj->fir.raw[idx][SC7A30_AXIS_Z]);
		}
		
		GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[SC7A30_AXIS_X], obj->fir.sum[SC7A30_AXIS_Y], obj->fir.sum[SC7A30_AXIS_Z]);
		GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[SC7A30_AXIS_X]/len, obj->fir.sum[SC7A30_AXIS_Y]/len, obj->fir.sum[SC7A30_AXIS_Z]/len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, char *buf, size_t count)
{
#ifdef CONFIG_SC7A30_LOWPASS
	struct i2c_client *client = sc7a30_i2c_client;  
	struct sc7a30_i2c_data *obj = i2c_get_clientdata(client);
	int firlen;

	if(1 != sscanf(buf, "%d", &firlen))
	{
		GSE_ERR("invallid format\n");
	}
	else if(firlen > C_MAX_FIR_LENGTH)
	{
		GSE_ERR("exceeds maximum filter length\n");
	}
	else
	{ 
		atomic_set(&obj->firlen, firlen);
		if(NULL == firlen)
		{
			atomic_set(&obj->fir_en, 0);
		}
		else
		{
			memset(&obj->fir, 0x00, sizeof(obj->fir));
			atomic_set(&obj->fir_en, 1);
		}
	}
#endif    
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct sc7a30_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, char *buf, size_t count)
{
	struct sc7a30_i2c_data *obj = obj_i2c_data;
	int trace;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}	
	else
	{
		GSE_ERR("invalid content: '%s', length = %ld\n", buf, count);
	}
	
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;    
	struct sc7a30_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}	
	
	if(obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n", 
	            obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);   
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	return len;    
}

static ssize_t SC7A30_3_axis_Calibration(struct device *dev,struct device_attribute *attr ,const char *buf, size_t count)
{
	unsigned temp[3];
	
		
	//Write_Input(0x1e, 0x05);   
    //auto_calibration_instant_mtp(0,0,-64);
   	 
		//Write_Input(0x1e, 0x15);  
		mdelay(5);		
       // Write_Input(0x1e, 0);
		printk(KERN_INFO "run calibration finished\n");



    	mdelay(5);
	return 1;
}

#ifdef SC7A30_EINT_MODE
static ssize_t show_impact_level_value(struct device_driver *ddri, char *buf)
{
	u8 res =0;
	struct sc7a30_i2c_data *data = obj_i2c_data;
	if(NULL == data)
	{
		printk(KERN_ERR "sc7a30_i2c_data is null!!\n");
		return -1;
	}
	hwmsen_read_byte(sc7a30_i2c_client,0x3a,&res);
	return sprintf(buf, "%d\n", res);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_impact_level_value(struct device_driver *ddri, const char *buf, size_t count)
{
	int res = 0;
	struct sc7a30_i2c_data *data = obj_i2c_data;

	if(NULL == data)
	{
		printk(KERN_ERR "sc7a30_i2c_data is null!!\n");
		return count;
	}

	if(1 == sscanf(buf, "%d", &res))
	{
		printk("read 0x3a data : %d !!!\n",res);
		hwmsen_write_byte(sc7a30_i2c_client,0x3a,res);    //设置碰撞等级
	}
	else
	{
		printk(KERN_ERR "invalid format = '%s'\n", buf);
	}

	return count;
}
#endif
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,             S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(sensordata,           S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, show_cali_value,          store_cali_value);
static DRIVER_ATTR(power,                S_IRUGO, show_power_status,          NULL);
static DRIVER_ATTR(selftest,   S_IWUSR | S_IRUGO, show_selftest_value,      store_selftest_value);
static DRIVER_ATTR(firlen,     S_IWUSR | S_IRUGO, show_firlen_value,        store_firlen_value);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, show_trace_value,         store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, show_status_value,        NULL);
static DRIVER_ATTR(calibration_run, 0777,        NULL, SC7A30_3_axis_Calibration);
#ifdef SC7A30_EINT_MODE
static DRIVER_ATTR(impact_level,      S_IRUGO | S_IWUSR, show_impact_level_value, store_impact_level_value);
#endif
/*----------------------------------------------------------------------------*/
static struct driver_attribute *sc7a30_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_power,         /*show power reg*/
	&driver_attr_selftest,     /*self control: 0: disable, 1: enable*/
	&driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,
#ifdef SC7A30_EINT_MODE
	&driver_attr_impact_level,
#endif
	//&driver_attr_calibration_run,   sys/device/platform/g-sensor/driver
};
/*----------------------------------------------------------------------------*/
static int sc7a30_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(sc7a30_attr_list)/sizeof(sc7a30_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(err = driver_create_file(driver, sc7a30_attr_list[idx]))
		{            
			GSE_ERR("driver_create_file (%s) = %d\n", sc7a30_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int sc7a30_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(sc7a30_attr_list)/sizeof(sc7a30_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, sc7a30_attr_list[idx]);
	}
	

	return err;
}

/*----------------------------------------------------------------------------*/
int gsensor_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value, sample_delay;	
	struct sc7a30_i2c_data *priv = (struct sc7a30_i2c_data*)self;
	hwm_sensor_data* gsensor_data;
	char buff[SC7A30_BUFSIZE];
	printk("sc7a30 gsensor_operate!!!\n");
	GSE_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
		    printk("SENSOR_DELAY  enter in !!!\n");
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value <= 5)
				{
					sample_delay = SC7A30_BW_400HZ;
				}
				else if(value <= 10)
				{
					sample_delay = ~SC7A30_BW_400HZ;
				}
				else
				{
					sample_delay = ~SC7A30_BW_400HZ;
				}
				
				err = SC7A30_SetBWRate(priv->client, sample_delay);
				if(err != SC7A30_SUCCESS ) //0x2C->BW=100Hz
				{
					GSE_ERR("Set delay parameter error!\n");
				}

				if(value >= 50)
				{
					atomic_set(&priv->filter, 0);
				}
				else
				{	
				   #if defined(CONFIG_SC7A30_LOWPASS)
					priv->fir.num = 0;
					priv->fir.idx = 0;
					priv->fir.sum[SC7A30_AXIS_X] = 0;
					priv->fir.sum[SC7A30_AXIS_Y] = 0;
					priv->fir.sum[SC7A30_AXIS_Z] = 0;
					atomic_set(&priv->filter, 1);
                                   #endif
				}
			}
			printk("SENSOR_DELAY  ok!!!\n");
			break;

		case SENSOR_ENABLE:
			printk("SENSOR_enable  enter in !!!\n");
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
			    
				value = *(int *)buff_in;
				GSE_LOG("enable value=%d, sensor_power =%d\n",value,sensor_power);
				if(((value == 0) && (sensor_power == false)) ||((value == 1) && (sensor_power == true)))
				{
					GSE_LOG("Gsensor device have updated!\n");
				}
				else
				{
					err = SC7A30_SetPowerMode( priv->client, !sensor_power);
					printk("SENSOR_enable  ok !!!\n");
				}
			}
			break;

		case SENSOR_GET_DATA:
			printk("SENSOR_GET_DATA  enter in !!!\n");
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				GSE_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				gsensor_data = (hwm_sensor_data *)buff_out;
				SC7A30_ReadSensorData(priv->client, buff, SC7A30_BUFSIZE);
				sscanf(buff, "%x %x %x", &gsensor_data->values[0], 
					&gsensor_data->values[1], &gsensor_data->values[2]);				
				gsensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;				
				gsensor_data->value_divide = 1000;
				printk("SENSOR_GET_DATA  ok !!!\n");
			}
			break;
		default:
			GSE_ERR("gsensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int sc7a30_open(struct inode *inode, struct file *file)
{
	file->private_data = sc7a30_i2c_client;
	printk("sc7a30_open!!!\n");
	if(file->private_data == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	printk("sc7a30_open222\n");
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int sc7a30_release(struct inode *inode, struct file *file)
{
	printk("sc7a30_release!!!\n");
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/
//static int sc7a30_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
 //      unsigned long arg)
static int sc7a30_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)       
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct sc7a30_i2c_data *obj = (struct sc7a30_i2c_data*)i2c_get_clientdata(client);	
	char strbuf[SC7A30_BUFSIZE];
	void __user *data;
	SENSOR_DATA sensor_data;
	int err = 0;
	int cali[3];
	printk("sc7a30_unlocked_ioctl!!!!!!!!!!!!!!!\n");
	GSE_FUN(f);
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
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case GSENSOR_IOCTL_INIT:
			printk("sc7a30_ioctl GSENSOR_IOCTL_INIT!!!!\n");
			SC7A30_Init(client, 0);			
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			printk("sc7a30_ioctl GSENSOR_IOCTL_READ_CHIPINFO!!!!\n");
			SC7A30_ReadChipInfo(client, strbuf, SC7A30_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;
			}				 
			break;	  

		case GSENSOR_IOCTL_READ_SENSORDATA:
			printk("sc7a30_ioctl GSENSOR_IOCTL_READ_SENSORDATA!!!!\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			SC7A30_ReadSensorData(client, strbuf, SC7A30_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}				 
			break;

		case GSENSOR_IOCTL_READ_GAIN:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &gsensor_gain, sizeof(GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

	/*	case GSENSOR_IOCTL_READ_OFFSET:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			if(copy_to_user(data, &gsensor_offset, sizeof(GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}				 
			break;*/

		case GSENSOR_IOCTL_READ_RAW_DATA:
			printk("sc7a30_ioctl GSENSOR_IOCTL_READ_RAW_DATA!!!!\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			//SC7A30_ReadRawData(client, &strbuf);
			SC7A30_ReadRawData(client, strbuf);
			if(copy_to_user(data, &strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
			break;	  

		case GSENSOR_IOCTL_SET_CALI:
			printk("sc7a30_ioctl GSENSOR_IOCTL_SET_CALI!!!!\n");
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&sensor_data, data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;	  
			}
			if(atomic_read(&obj->suspend))
			{
				GSE_ERR("Perform calibration in suspend state!!\n");
				err = -EINVAL;
			}
			else
			{
				cali[SC7A30_AXIS_X] = sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[SC7A30_AXIS_Y] = sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[SC7A30_AXIS_Z] = sensor_data.z * obj->reso->sensitivity / GRAVITY_EARTH_1000;			  
				err = SC7A30_WriteCalibration(client, cali);			 
			}
			break;

		case GSENSOR_IOCTL_CLR_CALI:
			err = SC7A30_ResetCalibration(client);
			break;

		case GSENSOR_IOCTL_GET_CALI:
			printk("sc7a30_ioctl GSENSOR_IOCTL_GET_CALI!!!!\n");
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(err = SC7A30_ReadCalibration(client, cali))
			{
				break;
			}
			
			sensor_data.x = cali[SC7A30_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.y = cali[SC7A30_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.z = cali[SC7A30_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}		
			break;
		

		default:
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}


/*----------------------------------------------------------------------------*/
static struct file_operations sc7a30_fops = {
	.owner = THIS_MODULE,
	.open = sc7a30_open,
	.release = sc7a30_release,
	.unlocked_ioctl = sc7a30_unlocked_ioctl,//sc7a30_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice sc7a30_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &sc7a30_fops,
};
/*----------------------------------------------------------------------------*/
#ifndef CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int sc7a30_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct sc7a30_i2c_data *obj = i2c_get_clientdata(client);    
	int err = 0;
	u8 dat;
	GSE_FUN();    

	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(obj == NULL)
		{
			GSE_ERR("null pointer!!\n");
			return -EINVAL;
		}
		//read old data
		if ((err = hwmsen_read_byte_sr(client, SC7A30_REG_CTL_REG1, &dat))) 
		{
           GSE_ERR("write data format fail!!\n");
           return err;
        }
		dat = dat&0b10111111;
		atomic_set(&obj->suspend, 1);
		if(err = hwmsen_write_byte(client, SC7A30_REG_CTL_REG1, dat))
		{
			GSE_ERR("write power control fail!!\n");
			return err;
		}        
		SC7A30_power(obj->hw, 0);
	}
	return err;
}
/*----------------------------------------------------------------------------*/
static int sc7a30_resume(struct i2c_client *client)
{
	struct sc7a30_i2c_data *obj = i2c_get_clientdata(client);        
	int err;
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}

	SC7A30_power(obj->hw, 1);
	if(err = SC7A30_Init(client, 0))
	{
		GSE_ERR("initialize client fail!!\n");
		return err;        
	}
	atomic_set(&obj->suspend, 0);

	return 0;
}
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/

static void sc7a30_early_suspend(struct early_suspend *h) 
{
	struct sc7a30_i2c_data *obj = container_of(h, struct sc7a30_i2c_data, early_drv);   
	int err;
	GSE_FUN();    

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}
	atomic_set(&obj->suspend, 1); 
	/*
	if(err = hwmsen_write_byte(obj->client, SC7A30_REG_POWER_CTL, 0x00))
	{
		GSE_ERR("write power control fail!!\n");
		return;
	}  
	*/
	if(err = SC7A30_SetPowerMode(obj->client, false))
	{
		GSE_ERR("write power control fail!!\n");
		return;
	}

	sensor_power = false;
	
	SC7A30_power(obj->hw, 0);
}
/*----------------------------------------------------------------------------*/
static void sc7a30_late_resume(struct early_suspend *h)
{
	struct sc7a30_i2c_data *obj = container_of(h, struct sc7a30_i2c_data, early_drv);         
	int err;
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}

	SC7A30_power(obj->hw, 1);
	if(err = SC7A30_Init(obj->client, 0))
	{
		GSE_ERR("initialize client fail!!\n");
		return;        
	}
	atomic_set(&obj->suspend, 0);    
}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/
/*----------------------------------------------------------------------------*/
/*static int sc7a30_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) 
{    
	strcpy(info->type, SC7A30_DEV_NAME);
	return 0;
}*/


static int sc7a30_acc_get_data( int *xyz)
{
	
	int ret = -1;
        int hw_d[3] = { 0 };
	signed short temp_data[3];
	
	int cnt_r = 0;
	char reg,data;
	char buffer1[3],buffer2[3] = {0};		
	
	memset(buffer1, 0, 3);
	memset(buffer2, 0, 3);

	while(1)
	{
		reg = SC7A30_STATUS;
		hwmsen_read_byte_sr(sc7a30_i2c_client, reg,data);
		if ((data & 0x08) != 0  ){
			break;
		}
		msleep(1);
		cnt_r++;
		if(cnt_r > 40)
			break;
			
	}

	
	reg = SC7A30_XOUT_L;
	hwmsen_read_byte_sr(sc7a30_i2c_client, reg,buffer2[1]);  
        reg = SC7A30_XOUT_H;
	hwmsen_read_byte_sr(sc7a30_i2c_client, reg,buffer2[0]);

	temp_data[0] = (signed short)( (buffer2[0]   << 8 | buffer1[0]) >> 4 );

	reg = SC7A30_YOUT_L;
	hwmsen_read_byte_sr(sc7a30_i2c_client, reg,buffer2[1]);
	reg = SC7A30_YOUT_H;
	hwmsen_read_byte_sr(sc7a30_i2c_client, reg,buffer2[0]);
	
        temp_data[1] = (signed short)( (buffer2[0]   << 8 | buffer1[0]) >> 4 );
	   
	reg = SC7A30_ZOUT_L;
	hwmsen_read_byte_sr(sc7a30_i2c_client, reg,buffer2[1]);	
	reg= SC7A30_ZOUT_H;
	hwmsen_read_byte_sr(sc7a30_i2c_client, reg,buffer2[0]);
	
	temp_data[2] = (signed short)( (buffer2[0]   << 8 | buffer1[0]) >> 4 );

	hw_d[0] = (temp_data[0] & 0x0800) ? (temp_data[0] | 0xFFFFF800) : (temp_data[0]);
	hw_d[1] = (temp_data[1] & 0x0800) ? (temp_data[1] | 0xFFFFF800) : (temp_data[1]);
	hw_d[2] = (temp_data[2] & 0x0800) ? (temp_data[2] | 0xFFFFF800) : (temp_data[2]);



	xyz[0] = hw_d[0] * 1000;             
	xyz[1] = hw_d[1] * 1000;          
	xyz[2] = hw_d[2] * 1000;

	return ret;
}
/*
static void sc7a30_work_handler(struct work_struct *work)
{

	int xyz[3] ={0, 0, 0};
	int ret =0;
	int adj_ok;

	int auto_count = 0;
 	int max_x=0, max_y=0, max_z=0, min_x=0, min_y=0, min_z=0;
 	int sum_x = 0,sum_y = 0,sum_z = 0;

	struct sc7a30_data_s *data =
		container_of(work, struct sc7a30_data_s, dwork.work);

	while (1) {
		
		ret = sc7a30_acc_get_data(xyz);
		
		if(ret == -1)
		{
			printk("Line %d:error!\n",__LINE__);
			continue;
		}

		xyz[0] /= 1024;
		xyz[1] /= 1024;
		xyz[2] /= 1024;

		if (auto_count == 0) {
			max_x = min_x = xyz[0];
			max_y = min_y = xyz[1];
			max_z = min_z = xyz[2];
		}

		max_x = max_x > xyz[0] ? max_x : xyz[0];
		max_y = max_y > xyz[1] ? max_y : xyz[1];
		max_z = max_z > xyz[2] ? max_z : xyz[2];
		min_x = min_x > xyz[0] ? xyz[0] : min_x;
		min_y = min_y > xyz[1] ? xyz[1] : min_y;
		min_z = min_z > xyz[2] ? xyz[2] : min_z;

		sum_x = sum_x + xyz[0];
		sum_y = sum_y + xyz[1];
		sum_z = sum_z + xyz[2];


		printk("x : %d  y : %d z : %d   sum_z = %d,auto_count = %d\n",xyz[0], xyz[1], xyz[2],sum_z, auto_count);
		
		if (auto_count > CALIBRATION_NUM-1) {
			printk("x-x : %d\n", max_x - min_x);
			printk("y-y : %d\n", max_y - min_y);
			printk("z-z : %d\n", max_z - min_z);

			if (max_x - min_x < AXIS_X_Y_RANGE_LIMIT 
					&& max_y - min_y < AXIS_X_Y_RANGE_LIMIT 
					&& max_z - min_z < AXIS_X_Y_RANGE_LIMIT ) {
				printk("sum-x-before : %d\n", sum_x);
				printk("sum-y-before : %d\n", sum_y);
				printk("sum-z-before : %d\n", sum_z);

				//ok
				sum_x /= CALIBRATION_NUM;
				sum_y /= CALIBRATION_NUM;
				sum_z /= CALIBRATION_NUM;
				printk("sum-x : %d\n", sum_x);
				printk("sum-y : %d\n", sum_y);
				printk("sum-z : %d\n", sum_z);

				//if (sum_x < AXIS_X_Y_AVG_LIMIT && sum_y < AXIS_X_Y_AVG_LIMIT) {
					if(1){
					if (sum_z > AXIS_Z_RANGE) {
						//auto_SC7A30_3_axis_Calibration( 0, 0, 64);
						adj_ok=auto_calibration_instant_mtp(0,0,64);
						auto_count = GOTO_CALI;
					}
					else if ( sum_z < -AXIS_Z_RANGE) {
						//auto_SC7A30_3_axis_Calibration( 0, 0, -64);
						adj_ok=auto_calibration_instant_mtp(0,0,-64);
						auto_count = GOTO_CALI;
					}
					else {
						printk("Line %d:error!\n",__LINE__);
						auto_count = FAILTO_CALI;
					}
				}
				else if( sum_x < AXIS_X_Y_AVG_LIMIT || sum_y < AXIS_X_Y_AVG_LIMIT) {
					if ( sum_z > AXIS_Z_DFT_G - AXIS_Z_RANGE && sum_z < AXIS_Z_DFT_G + AXIS_Z_RANGE) {
						auto_SC7A30_3_axis_Calibration( 0, 0, 64);
						auto_count = GOTO_CALI;
					}
					else if ( sum_z < -(AXIS_Z_DFT_G - AXIS_Z_RANGE) && sum_z > -(AXIS_Z_DFT_G + AXIS_Z_RANGE)) {
						auto_SC7A30_3_axis_Calibration( 0, 0, -64);
						auto_count = GOTO_CALI;
					}
					else {
						auto_count = FAILTO_CALI;
						printk("Line %d:error!\n",__LINE__);
					}
					
				}
				else {
					auto_count = FAILTO_CALI;
					printk("Line %d:error!\n",__LINE__);
				}
				if(adj_ok==1)
					{
						printk("Calibration succeed\n");
						//Write_Input(  0x1e, 0x05);
					}
					else
					{
						printk("Calibration failed\n");
					}	
				
			}
			else {
				auto_count = FAILTO_CALI;
				printk("Line %d:error!\n",__LINE__);
			}
		}

		if(auto_count == GOTO_CALI){
			break;
		}

		if (auto_count == FAILTO_CALI) {
			sum_x = sum_y = sum_z = max_x =  max_y =  max_z =  min_x =  min_y =  min_z = 0;
			auto_count = -1;
			data->time_of_cali++;
			if (data->time_of_cali > 1) {
				printk("calibration failed !!\n");
				break;
			}
		}
		auto_count++;
		msleep(20);
		//mutex_unlock(&(data->allen_mutex) );//allen
	}
}


static int sc7a30_acc_cali(struct i2c_client *client)
{
	struct sc7a30_data_s *data = (struct sc7a30_data_s *) i2c_get_clientdata(client);
	//__cancel_delayed_work(&data->dwork);

	queue_delayed_work(sc7a30_workqueue, 
			&data->dwork,
			msecs_to_jiffies(500));

	printk("cali !!!\n");
	Write_Input(0X20,0X77);

	//sc7a30_acc_hw_init();
	return 0;
}
*/

#ifdef SC7A30_EINT_MODE
void sc7a30_eint_handler(void)
{
    mt_eint_mask(CUST_EINT_GSENSOR_NUM);
    printk("%s do !!!\n\n",__func__);
    input_report_key(sc7a30_input_dev,KEY_F12,1);
    input_sync(sc7a30_input_dev);
    input_report_key(sc7a30_input_dev,KEY_F12,0);
    input_sync(sc7a30_input_dev);
    mt_eint_unmask(CUST_EINT_GSENSOR_NUM);
}

static void sc7a30_eint_work(struct work_struct *work)
{
	u8 databuf[2] = {0}; 
	struct sc7a30_i2c_data *obj = obj_i2c_data;

	printk("%s do!!!\n",__func__);
	if(obj == NULL)
	{
		GSE_ERR("obj_i2c_data is null pointer!!\n");
		goto sc7a30_eint_work_exit;
	}	
	//mt_eint_mask(CUST_EINT_GSENSOR_NUM);
	sc7a30_eint_handler();
sc7a30_eint_work_exit:
	mt_eint_unmask(CUST_EINT_GSENSOR_NUM);
}


static void sc7a30_eint_func(void)
{
	struct sc7a30_i2c_data *priv = obj_i2c_data;
	printk("%s do!!!\n",__func__);
	if(!priv)
	{
		return;
	}	
	schedule_work(&priv->eint_work);
}

static int sc7a30_setup_eint(void)
{
#ifdef GPIO_EXT_GSENSOR_EINT_PIN
		mt_set_gpio_dir(GPIO_EXT_GSENSOR_EINT_PIN, GPIO_DIR_IN);
		mt_set_gpio_mode(GPIO_EXT_GSENSOR_EINT_PIN, GPIO_EXT_GSENSOR_EINT_PIN_M_EINT);
		mt_set_gpio_pull_enable(GPIO_EXT_GSENSOR_EINT_PIN, true);
		mt_set_gpio_pull_select(GPIO_EXT_GSENSOR_EINT_PIN, GPIO_PULL_UP);
	
		mt_eint_set_hw_debounce(CUST_EINT_GSENSOR_NUM, CUST_EINT_GSENSOR_DEBOUNCE_CN);
		mt_eint_registration(CUST_EINT_GSENSOR_NUM, CUST_EINT_GSENSOR_TYPE, sc7a30_eint_func, 0);

		mt_eint_unmask(CUST_EINT_GSENSOR_NUM);
#endif
	return 0;
}

int sc7a30_acc_input_init(void)
{
    printk("%s do !!!\n",__func__);
    int error = 0;
    sc7a30_input_dev = input_allocate_device();
    if(!sc7a30_input_dev)
    {
        printk("Input_allocate_device failed");
        error  = -ENOMEM;
    }
    if(!error)
    {
        sc7a30_input_dev->name = "sc7a30";
        set_bit(EV_KEY,sc7a30_input_dev->evbit);
        set_bit(KEY_F12,sc7a30_input_dev->keybit);
        input_set_capability(sc7a30_input_dev,EV_KEY,KEY_HOME);
        error = input_register_device(sc7a30_input_dev);
    }
    if(error)
    {
        printk("register input device failed");
        input_free_device(sc7a30_input_dev);
        sc7a30_input_dev = NULL;
    }
    return error;

}
#endif
/*----------------------------------------------------------------------------*/
static int sc7a30_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;

	struct sc7a30_i2c_data *obj;
	struct hwmsen_object sobj;
	char reg_cali;
	int err = 0;
      //allen  struct sc7a30_data_s* data = &sc7a30_data;
	GSE_FUN();
    printk("sc7a30_i2c_probe enter\n");
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	  
	memset(obj, 0, sizeof(struct sc7a30_i2c_data));

#ifdef SC7A30_EINT_MODE
	INIT_WORK(&obj->eint_work, sc7a30_eint_work);
#endif

	obj->hw = get_cust_acc_hw();
	
	if(err = hwmsen_get_convert(obj->hw->direction, &obj->cvt))
	{
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}
   printk("sc7a30_i2c_probe enter333333333333333\n");
	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client,obj);

	//data->client  = client;
	//i2c_set_clientdata(new_client, data);
	
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	
	hwmsen_read_byte_sr(client,0x0f,&reg_cali);
    printk("!!!!!!!!!!!!!!!chip_id is %x \n",reg_cali);
	
#ifdef CONFIG_SC7A30_LOWPASS
	if(obj->hw->firlen > C_MAX_FIR_LENGTH)
	{
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	}	
	else
	{
		atomic_set(&obj->firlen, obj->hw->firlen);
	}
	
	if(atomic_read(&obj->firlen) > 0)
	{
		atomic_set(&obj->fir_en, 1);
	}
	
#endif

	sc7a30_i2c_client = new_client;	

	if(err = SC7A30_Init(new_client, 1))
	{
		goto exit_init_failed;
	}
	

	if(err = misc_register(&sc7a30_device))
	{
		GSE_ERR("sc7a30_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	if(err = sc7a30_create_attr(&(sc7a30_init_info.platform_diver_addr->driver)))
     //   if(err = sc7a30_create_attr(&sc7a30_gsensor_driver.driver))
	{
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	hwmsen_read_byte_sr(sc7a30_i2c_client,0x0f,&reg_cali);
        printk("!!!!!!!!!!!!!!!chip_id is %x \n",reg_cali);

	/*if((reg_cali & 0x1) == 0){

		if (sc7a30_workqueue == NULL) {
			sc7a30_workqueue = create_workqueue("sc7a30_acc");
			if (sc7a30_workqueue == NULL) {
		    	//	printk("line %d workque fail!!!!!!!!!!!!!!\n",__LINE__);	
			}
		}		

		INIT_DELAYED_WORK(&data->dwork, sc7a30_work_handler);
		// printk("line %d workque !!!!!!!!!!!!!\n",__LINE__);
		sc7a30_acc_cali(client);
		// printk("line %d workque !!!!!!!!!!!\n",__LINE__);			
	}
	else {
	

	}
*/

#ifndef SC7A30_EINT_MODE
	sobj.self = obj;
    sobj.polling = 1;
    sobj.sensor_operate = gsensor_operate;
	if(err = hwmsen_attach(ID_ACCELEROMETER, &sobj))
	{
		GSE_ERR("attach fail = %d\n", err);
		goto exit_kfree;
	}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = sc7a30_early_suspend,
	obj->early_drv.resume   = sc7a30_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif 

#ifdef SC7A30_EINT_MODE
	err = sc7a30_setup_eint();
	sc7a30_acc_input_init();
#endif

	GSE_LOG("%s: OK\n", __func__);
	// hwmsen_read_byte_sr(sc7a30_i2c_client,SC7A30_REG_CTL_REG1,&reg_cali);
     printk("sc7a30_i2c_probe OK\n");

	return 0;

	exit_create_attr_failed:
	misc_deregister(&sc7a30_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(new_client);
	exit_kfree:
	kfree(obj);
	exit:
	GSE_ERR("%s: err = %d\n", __func__, err);        
	return err;
}

/*----------------------------------------------------------------------------*/
static int sc7a30_i2c_remove(struct i2c_client *client)
{
	int err = 0;	
	
	//if(err = sc7a30_delete_attr(&sc7a30_gsensor_driver.driver))
	err = sc7a30_delete_attr(&sc7a30_init_info.platform_diver_addr->driver);	
	{
		GSE_ERR("sc7a30_delete_attr fail: %d\n", err);
	}

	if(err = misc_deregister(&sc7a30_device))
	{
		GSE_ERR("misc_deregister fail: %d\n", err);
	}

#ifndef SC7A30_EINT_MODE
	if(err = hwmsen_detach(ID_ACCELEROMETER))
#endif

	sc7a30_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}
/*----------------------------------------------------------------------------*/
static int sc7a30_probe(struct platform_device *pdev) 
{
	struct acc_hw *hw =  get_cust_acc_hw();
        //struct acc_hw *hw = sc7a30_get_cust_acc_hw();
	GSE_FUN();

	SC7A30_power(hw, 1);
	//sc7a30_force[0] = hw->i2c_num;
	if(i2c_add_driver(&sc7a30_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}
	return 0;

}
/*----------------------------------------------------------------------------*/
static int sc7a30_remove(void)
{
    struct acc_hw *hw =  get_cust_acc_hw();
   //struct acc_hw *hw = sc7a30_get_cust_acc_hw();
    GSE_FUN();    
    SC7A30_power(hw, 0);    
    i2c_del_driver(&sc7a30_i2c_driver);
    return 0;
}


/*-----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
#if 0
static struct platform_driver sc7a30_gsensor_driver = {
	.probe      = sc7a30_probe,
	.remove     = sc7a30_remove,    
	.driver     = {
		.name  = "gsensor",
		.owner = THIS_MODULE,
	}
};
#endif

static int  sc7a30_local_init(void)
{
	struct acc_hw *hw = get_cust_acc_hw();

	SC7A30_power(hw, 1);

	if (i2c_add_driver(&sc7a30_i2c_driver)) {
		GSE_ERR("add driver error\n");
		return -1;
	}
	return 0;
}


/*----------------------------------------------------------------------------*/
static int __init sc7a30_init(void)
{
    struct acc_hw *hw = get_cust_acc_hw();
	GSE_LOG("%s: i2c_number=%d\n", __func__, hw->i2c_num);
	i2c_register_board_info( hw->i2c_num, &i2c_sc7a30, 1);
	acc_driver_add(&sc7a30_init_info);
	printk("======sc7a30_init END======\n");
	return 0;    
}
/*----------------------------------------------------------------------------*/
static void __exit sc7a30_exit(void)
{
	GSE_FUN();
	//platform_driver_unregister(&sc7a30_gsensor_driver);
}
/*----------------------------------------------------------------------------*/
module_init(sc7a30_init);
module_exit(sc7a30_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("SC7A30 I2C driver");
MODULE_AUTHOR("Chunlei.Wang@mediatek.com");
