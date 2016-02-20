/* QMA6981 motion sensor driver
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/errno.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/earlysuspend.h>
#include "qma6981.h"
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <cust_acc.h>

#include <linux/irq.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <accel.h>
#include <linux/batch.h>
#include <cust_acc.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <linux/hwmsen_helper.h>

static int qma6981_init_flag =-1; // 0<==>OK -1 <==> fail

#define QMA6981_MOTION_VIRTUAL_SENSOR 0

#define QMA6981_MAJOR	101
#define QMA6981_MINOR	4

#define __isNegative(_val)  (_val&0x80)

#define QMA6981_RETRY_COUNT 1
/*range*/
#define QMA6981_RANGE_2G   (1<<0)
#define QMA6981_RANGE_4G   (1<<1)
#define QMA6981_RANGE_8G   (1<<2)
#define QMA6981_RANGE_16G  (1<<3)


#define QMA6981_XOUTL			0x01	// 4-bit output value X
#define QMA6981_XOUTH			0x02	// 6-bit output value X
#define QMA6981_YOUTL			0x03	
#define QMA6981_YOUTH			0x04	
#define QMA6981_ZOUTL			0x05	
#define QMA6981_ZOUTH			0x06	
#define QMA6981_TILT			0x03	// Tilt status
#define QMA6981_SRST			0x04	// Sampling Rate Status
#define QMA6981_SPCNT			0x05	// Sleep Count
#define QMA6981_INTSU			0x17	// Interrupt Setup
#define QMA6981_MODE			0x11	// Mode
#define QMA6981_SR			0x10	// Auto-Wake/Sleep and Debounce Filter
#define QMA6981_PDET			0x09	// Tap Detection
#define QMA6981_PD			0x0a	// Tap Debounce Count
#define QMA6981_INT_MAP0		0x19	// INT MAP
#define QMA6981_RANGE                   0x0f    //range set register
#define QMA6981_INT_STAT		0x09    //interrupt statues

#define QMA6981_FIFO_WTMK		0x31	// FIFO water mark level
#define QMA6981_FIFO_CONFIG		0x3e	// fifo configure
#define QMA6981_FIFO_DATA		0x3f	//fifo data out 

#define QMA6981_CHIP_ID		0x00
#define QMA6981_DIE_ID			0x07

#define QMA6981_DEV_NAME        "qma6981"
#define QMA6981_BUFSIZE		256
static struct i2c_client *this_client = NULL;
static struct mutex read_i2c_xyz;
static struct acc_init_info qma6981_init_info;

#define QMA6981_AXIS_X          0
#define QMA6981_AXIS_Y          1
#define QMA6981_AXIS_Z          2


#define DEBUG 1
static struct i2c_board_info __initdata i2c_qma6981={ I2C_BOARD_INFO(QMA6981_DEV_NAME, (QMA6981_I2C_SLAVE_ADDR))};
/*
 * QMA6981 acc data
 * brief Structure containing acc field values for x,y and z-axis in
 * signed short
*/

struct QMA6981_t {
	short	x, /**< x-axis acc field data. Range -512 to 512. */
			y, /**< y-axis acc field data. Range -512 to 512. */
			z; /**< z-axis acc filed data. Range -512 to 512. */
};
typedef enum {
    	QMA_FUN_DEBUG  = 0x01,
	QMA_DATA_DEBUG = 0X02,
	QMA_HWM_DEBUG  = 0X04,
	QMA_CTR_DEBUG  = 0X08,
	QMA_I2C_DEBUG  = 0x10,
} QMA_TRC;

struct QMA6981_data{
	struct i2c_client *client;
	struct acc_hw *hw;
	struct QMA6981_platform_data *pdata;
	short xy_sensitivity;
	short z_sensitivity;

	struct mutex lock;
	struct mutex motionLock;
	struct delayed_work work;
	struct delayed_work motionWork;
	struct input_dev *input;
	//struct miscdevice qma_misc;
	struct hwmsen_convert   cvt;
	int delay_ms;

	int enabled;
	atomic_t                trace;
	atomic_t                suspend;

	s16                     cali_sw[QMA6981_AXES_NUM+1];

	/*data*/
	s8                      offset[QMA6981_AXES_NUM+1];  /*+1: for 4-byte alignment*/
	s16                     data[QMA6981_AXES_NUM+1];
	u8                      bandwidth;
	//struct completion data_updated;
	//wait_queue_head_t state_wq;
	    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(USE_EARLY_SUSPEND)
    	struct early_suspend    early_drv;
#endif 
};

static struct QMA6981_data *acc;

static struct class *qma_acc_dev_class;
static bool sensor_power = true;
static bool enable_status = false;

static char QMA6981_i2c_write(unsigned char reg_addr,
				    unsigned char *data,
				    unsigned char len);

static char QMA6981_i2c_read(unsigned char reg_addr,
				   unsigned char *data,
				   unsigned char len);

static int QMA6981_read_acc_xyz(int *data);
static int QMA6981_create_attr(struct device_driver *driver);
static int QMA6981_SetPowerCTRL(struct i2c_client *client, bool enable);
static int QMA6981_SetBWRate(struct i2c_client *client, u8 bwrate);
static int qma6981_initialize(struct i2c_client *client);
//static int qma6981_stepCounter_enable(struct i2c_client *client);
static int ready_to_read_data(void);

static int I2C_RxData(char *rxData, int length)
{	
	uint8_t loop_i;    
	int res = 0;
	#if DEBUG	
	int i;
	struct i2c_client *client = this_client;	
	struct QMA6981_data *data = i2c_get_clientdata(client);	
	char addr = rxData[0];
	#endif	/* Caller should check parameter validity.*/	
    //printk("qma6981 i2c_rxdata\n");
	if((rxData == NULL) || (length < 1))	
	{	
		printk("qma6981 I2C_RxData error");
		return -EINVAL;	
	}	
	//for(loop_i = 0; loop_i < QMA6981_RETRY_COUNT; loop_i++)	
	{		
		this_client->addr = this_client->addr & I2C_MASK_FLAG | I2C_WR_FLAG;		
		res = i2c_master_send(this_client, (const char*)rxData, ((length<<0X08) | 0X01));		
		if(res <= 0)		
		{			
			return QMA6981_ERR_I2C;		
		}		
		//printk("QMA6981 i2c_read retry %d times\n", loop_i);		
		mdelay(10);	
	}    	
	//if(loop_i >= QMA6981_RETRY_COUNT)	
	//{		
	//	printk("qma6981 %s retry over %d\n", __func__, QMA6981_RETRY_COUNT);		
	//	return -EIO;	
	//}
	#if 0	
	if(atomic_read(&data->trace) & QMA_I2C_DEBUG)	
	{		
		//printk("qma6981 RxData: len=%02x, addr=%02x\n", length, addr);		
		for(i = 0; i < length; i++)		
		{			
			//printk("qma6981 %02x\n", rxData[i]);		
		}	    
	}
	#endif	
	return 0;
}

static int I2C_TxData(char *txData, int length)
{
	uint8_t loop_i;

#if DEBUG
	int i;
	struct i2c_client *client = this_client;
	struct QMA6981_data *data = i2c_get_clientdata(client);
#endif

	/* Caller should check parameter validity.*/
	if ((txData == NULL) || (length < 2))
	{
		return -EINVAL;
	}

	this_client->addr = this_client->addr & I2C_MASK_FLAG;
	//for(loop_i = 0; loop_i < QMA6981_RETRY_COUNT; loop_i++)
	//{
	    	if (NULL == this_client){        
				printk(KERN_ERR "%s this_client is null \n", __func__);
		}else{
			if(i2c_master_send(this_client, (const char*)txData, length) <= 0){
			        return QMA6981_ERR_I2C;
			}
			//{
			//	break;
			//}
			//printk("I2C_TxData delay!\n");
			mdelay(10);
		}
	//}

	//if(loop_i >= QMA6981_RETRY_COUNT)
	//{
	//	printk(KERN_ERR "%s retry over %d\n", __func__, QMA6981_RETRY_COUNT);
	//	return -EIO;
	//}
#if 0//DEBUG
	{
		//printk("TxData: len=%02x, addr=%02x\n  data=", length, txData[0]);
		for(i = 0; i < (length-1); i++)
		{
			//printk(" %02x", txData[i + 1]);
		}
		//printk("\n");
	}
#endif
	return 0;
}

/* X,Y and Z-axis accelerometer data readout
 * param *acc pointer to \ref QMA6981_t structure for x,y,z data readout
 * note data will be read by multi-byte protocol into a 6 byte structure
*/
typedef struct {
    int i:10;
    int rsv:22;
}testtypedef;
	
static int QMA6981_read_raw_xyz(int *data){
	int res;	
	unsigned char databuf[6];	
	int output[3]={ 0 };	
	
	unsigned char databuf2[1];	
	int output2[1]={ 0 };	

	testtypedef out;

#if 0
/************************add offset*******************/
	databuf2[0] = 0x00;		
	if(res = I2C_RxData(databuf2, 1)){
		//printk("QMA6981_stepcounter error!!!");
		return -EFAULT;	
	}	

	output2[0] = (int)databuf2[0];
	//printk("QMA6981_resume %d\n",output2[0]);
	if((int)output2[0] == 0xa9){
	   // printk("QMA6981_resume!!!!!!!!\n");
	    ready_to_read_data();
	}
/************************add offset end*******************/
#endif

	databuf[0] = QMA6981_XOUTL;		
	if(res = I2C_RxData(databuf, 6)){
		printk("QMA6981_XOUTL error!!!");
		return -EFAULT;	
	}	

	//printk("RAW_DATA_BEFORE %d,%d,%d,%d,%d,%d\n",databuf[0],databuf[1],databuf[2],databuf[3],databuf[4],databuf[5]);
	//output[0] = (short int)( ((short)(databuf[1] & 0x7f) << 2) | ( (databuf[0] >> 6)&0x03 ) );
	//output[1] = (short int)( ((short)(databuf[3] & 0x7f) << 2) | ( (databuf[2] >> 6)&0x03 ) );
	//output[2] = (short int)( ((short)(databuf[5] & 0x7f) << 2) | ( (databuf[4] >> 6)&0x03 ) );
	
	//printk("RAW_DATA_MID %d,%d,%d\n",output[0],output[1],output[2]);
	
//	data[0] = __isNegative(databuf[1]) ? ( ( ~((output[0])-1) )&0x1ff )*-1 : output[0];
//	data[1] = __isNegative(databuf[3]) ? ( ( ~((output[1])-1) )&0x1ff )*-1 : output[1];
//	data[2] = __isNegative(databuf[5]) ? ( ( ~((output[2])-1) )&0x1ff )*-1 : output[2];


 	data[0] = out.i = (int)((databuf[1]<<2) |( databuf[0]>>6));
	data[1] = out.i = (int)((databuf[3]<<2) |( databuf[2]>>6));
	data[2] = out.i = (int)((databuf[5]<<2) |( databuf[4]>>6));

	//printk("RAW_DATA_AFTER %d,%d,%d\n",data[0],data[1],data[2]);
//dingge	
	data[1] = data[1] + data[2]*(8/100);
}

static int QMA6981_read_acc_xyz(int *data){
	
	int raw[3]={0};
	int acc[3]={0};
	int temp;
	struct i2c_client *client = this_client;	
	struct QMA6981_data *obj = i2c_get_clientdata(client);
	
	QMA6981_read_raw_xyz(raw);
//dingge
	acc[0] = raw[0]*1000/256*(98/10);
	acc[1] = raw[1]*1000/256*(98/10);
	acc[2] = raw[2]*1000/256*(98/10);

	//printk("qma6981 obj->cvt.map[QMA6981_AXIS_X]:%d\n",obj->cvt.map[QMA6981_AXIS_X]);
	//printk("qma6981 obj->cvt.map[QMA6981_AXIS_Y]:%d\n",obj->cvt.map[QMA6981_AXIS_Y]);
	//printk("qma6981 obj->cvt.map[QMA6981_AXIS_Z]:%d\n",obj->cvt.map[QMA6981_AXIS_Z]);

	//printk("qma6981 obj->cvt.sign[QMA6981_AXIS_X]:%d\n",obj->cvt.sign[QMA6981_AXIS_X]);
	//printk("qma6981 obj->cvt.sign[QMA6981_AXIS_X]:%d\n",obj->cvt.sign[QMA6981_AXIS_Y]);
	//printk("qma6981 obj->cvt.sign[QMA6981_AXIS_Z]:%d\n",obj->cvt.sign[QMA6981_AXIS_Z]);

	data[obj->cvt.map[QMA6981_AXIS_X]] = obj->cvt.sign[QMA6981_AXIS_X]*acc[QMA6981_AXIS_X];
	data[obj->cvt.map[QMA6981_AXIS_Y]] = obj->cvt.sign[QMA6981_AXIS_Y]*acc[QMA6981_AXIS_Y];
	data[obj->cvt.map[QMA6981_AXIS_Z]] = obj->cvt.sign[QMA6981_AXIS_Z]*acc[QMA6981_AXIS_Z];
	
	data[QMA6981_AXIS_X] = data[QMA6981_AXIS_X]+obj->cali_sw[QMA6981_AXIS_X];
	data[QMA6981_AXIS_Y] = data[QMA6981_AXIS_Y]+obj->cali_sw[QMA6981_AXIS_Y];
	data[QMA6981_AXIS_Z] = data[QMA6981_AXIS_Z]+obj->cali_sw[QMA6981_AXIS_Z];
	printk("qma6981 AFTER x:%d,y:%d,z:%d\n",data[QMA6981_AXIS_X],data[QMA6981_AXIS_Y],data[QMA6981_AXIS_Z]);

	return 0;
}

static int QMA6981_read_stepCounter(int *data){
	int res;	
	unsigned char databuf[2];	
	int output[3]={ 0 };	

	databuf[0] = 0x1c;		
	if(res = I2C_RxData(databuf, 2)){
		printk("QMA6981_stepcounter error!!!");
		return -EFAULT;	
	}	

	*data = databuf[0] + databuf[1] * 16 * 16;
	return 0;
}

/* Set the Gain range */
int QMA6981_set_range(int range)
{
	int err = 0;
	unsigned char data;

	data = (unsigned char)range;


	if (acc->client == NULL)  /*  No global client pointer? */
		return -1;

	err = i2c_smbus_write_byte_data(acc->client,QMA6981_RANGE,data);

	if (err) {
			printk("i2c write error\n");
	}
	return err;

}

/* Get the Gain range */
int QMA6981_get_range(unsigned char* range)
{
	int err = 0;
	unsigned char r = 0;

	err = QMA6981_i2c_read(QMA6981_RANGE,&r,1);
	*range = r;

	return err;
}


/*  i2c write routine for qma6981 accelerometer */
static char QMA6981_i2c_write(unsigned char reg_addr,
				    unsigned char *data,
				    unsigned char len)
{
	int dummy;
	int i;

	if (acc->client == NULL)  /*  No global client pointer? */
		return -1;
	for (i = 0; i < len; i++) {
		dummy = i2c_smbus_write_byte_data(acc->client,
						  reg_addr++, data[i]);
		if (dummy) {
			printk("i2c write error\n");
			return dummy;
		}
	}
	return 0;
}

/*  i2c read routine for QMA6981 data reg  */
static char QMA6981_i2c_read(unsigned char reg_addr,
				   unsigned char *data,
				   unsigned char len)
{
	char dummy = 0;
	int i = 0;

	if (acc->client == NULL)  /*  No global client pointer? */
		return -1;
	while (i < len) {
		dummy = i2c_smbus_read_byte_data(acc->client,
						 reg_addr++);

		if (dummy >= 0) {
			data[i] = dummy;
			i++;
		} else {
			printk(" i2c read error\n ");
			return dummy;
		}
		dummy = len;
	}
	return dummy;
}


static void qma6981_start_measure(struct QMA6981_data *qma6981)
{
	/* Goto active mode for configuration */
	i2c_smbus_write_byte_data(qma6981->client, QMA6981_MODE, 0xa0);
}

static void qma6981_stop_measure(struct QMA6981_data *qma6981)
{
	/* Goto standby mode for configuration */
	i2c_smbus_write_byte_data(qma6981->client, QMA6981_MODE, 0x00);
}

static int qma6981_enable(struct QMA6981_data *qma6981)
{
	dev_dbg(&qma6981->client->dev, "start measure!\n");
	//printk("!!!!qma6981_enable!!!!!");

	schedule_delayed_work(&qma6981->work,
			msecs_to_jiffies(qma6981->delay_ms));
	return 0;
}

static int qma6981_disable(struct QMA6981_data *qma6981)
{
	dev_dbg(&qma6981->client->dev, "stop measure!\n");
	//printk("!!!!qma6981_disable!!!!!");
	cancel_delayed_work(&qma6981->work);

	return 0;
}

/*----------------------------------------------------------------------------*/
static int QMA6981_ReadOffset(struct i2c_client *client, s8 ofs[QMA6981_AXES_NUM])
{    
	int err = 0;

	ofs[1]=ofs[2]=ofs[0]=0x00;

	//printk("qma6981 offesx=%x, y=%x, z=%x\n",ofs[0],ofs[1],ofs[2]);
	
	return err;    
}

/*----------------------------------------------------------------------------*/
static int QMA6981_ResetCalibration(struct i2c_client *client)
{
	struct QMA6981_data *obj = i2c_get_clientdata(client);
	int err = 0;
	//printk("qma6981 QMA6981_ResetCalibration\n");
	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));
	return err;    
}

/*----------------------------------------------------------------------------*/
static int QMA6981_ReadCalibrationEx(struct i2c_client *client, int act[QMA6981_AXES_NUM], int raw[QMA6981_AXES_NUM])
{  
	/*raw: the raw calibration data; act: the actual calibration data*/
	struct QMA6981_data *obj = i2c_get_clientdata(client);
	int mul;

	mul = 0;//only SW Calibration, disable HW Calibration
	//printk("qma6981 QMA6981_ReadCalibrationEx\n");
	raw[QMA6981_AXIS_X] = obj->offset[QMA6981_AXIS_X]*mul + obj->cali_sw[QMA6981_AXIS_X];
	raw[QMA6981_AXIS_Y] = obj->offset[QMA6981_AXIS_Y]*mul + obj->cali_sw[QMA6981_AXIS_Y];
	raw[QMA6981_AXIS_Z] = obj->offset[QMA6981_AXIS_Z]*mul + obj->cali_sw[QMA6981_AXIS_Z];

	act[obj->cvt.map[QMA6981_AXIS_X]] = obj->cvt.sign[QMA6981_AXIS_X]*raw[QMA6981_AXIS_X];
	act[obj->cvt.map[QMA6981_AXIS_Y]] = obj->cvt.sign[QMA6981_AXIS_Y]*raw[QMA6981_AXIS_Y];
	act[obj->cvt.map[QMA6981_AXIS_Z]] = obj->cvt.sign[QMA6981_AXIS_Z]*raw[QMA6981_AXIS_Z];                        
	                       
	return 0;
}

/*----------------------------------------------------------------------------*/
static int QMA6981_WriteCalibration(struct i2c_client *client, int dat[QMA6981_AXES_NUM])
{
	struct QMA6981_data *obj = i2c_get_clientdata(client);
	int err;
	int cali[QMA6981_AXES_NUM], raw[QMA6981_AXES_NUM];


	//printk("qma6981PDATE: (%+3d %+3d %+3d)\n", 
	//	dat[QMA6981_AXIS_X], dat[QMA6981_AXIS_Y], dat[QMA6981_AXIS_Z]);


	obj->cali_sw[QMA6981_AXIS_X] = dat[QMA6981_AXIS_X];
	obj->cali_sw[QMA6981_AXIS_Y] = dat[QMA6981_AXIS_Y];
	obj->cali_sw[QMA6981_AXIS_Z] = dat[QMA6981_AXIS_Z];	


	return err;
}
/*----------------------------------------------------------------------------*/
static int QMA6981_ReadCalibration(struct i2c_client *client, int dat[QMA6981_AXES_NUM])
{
    struct QMA6981_data *obj = i2c_get_clientdata(client);
    int mul;
  
    dat[QMA6981_AXIS_X] = obj->cali_sw[QMA6981_AXIS_X];
    dat[QMA6981_AXIS_Y] = obj->cali_sw[QMA6981_AXIS_Y];
    dat[QMA6981_AXIS_Z] = obj->cali_sw[QMA6981_AXIS_Z];                      
    return 0;
}

static void qma6981_work(struct work_struct *work)
{
	int ret;
	struct QMA6981_data *qma6981 = container_of((struct delayed_work *)work,struct QMA6981_data, work);
	unsigned char data[6];
	int vec[3] = {0};
	mutex_lock(&qma6981->lock);

	if (!qma6981->enabled)
		goto out;

	ret = QMA6981_read_acc_xyz(vec);
	if (ret < 0) {
		dev_err(&qma6981->client->dev, "error read data\n");
	} else {

		input_report_abs(qma6981->input, ABS_X, vec[0]);
		input_report_abs(qma6981->input, ABS_Y, vec[1]);
		input_report_abs(qma6981->input, ABS_Z, vec[2]);
		input_sync(qma6981->input);
	
		schedule_delayed_work(&qma6981->work,msecs_to_jiffies(qma6981->delay_ms));
	}

out:
	mutex_unlock(&qma6981->lock);
}

static int qma6981_input_init(struct QMA6981_data *qma6981)
{
	struct input_dev *dev;
	int ret;
	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	dev->name = "qma6981";
	dev->id.bustype = BUS_I2C;

	set_bit(EV_ABS, dev->evbit);
	set_bit(ABS_X, dev->absbit);
	set_bit(ABS_Y, dev->absbit);
	set_bit(ABS_Z, dev->absbit);
	input_set_abs_params(dev, ABS_X, -512, 512, 4, 4);
	input_set_abs_params(dev, ABS_Y, -512, 512, 4, 4);
	input_set_abs_params(dev, ABS_Z, -512, 512, 4, 4);
	input_set_drvdata(dev, qma6981);

	ret = input_register_device(dev);
	if (ret < 0) {
		input_free_device(dev);
		return ret;
	}

	qma6981->input= dev;
	return 0;
}

static ssize_t attr_get_poll(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct QMA6981_data *qma6981 = dev_get_drvdata(dev);
	int pollms;

	mutex_lock(&qma6981->lock);
	pollms = qma6981->delay_ms;
	mutex_unlock(&qma6981->lock);

	return sprintf(buf, "%d\n", pollms);
}

#define QMA6981_MAX_RATE 75

static ssize_t attr_set_poll(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct QMA6981_data *qma6981 = dev_get_drvdata(dev);
	unsigned long pollms = 0;

	if (strict_strtoul(buf, 10, &pollms) || pollms <= 0)
		return -EINVAL;

	if (pollms < (1000 / QMA6981_MAX_RATE))
		pollms = 1000 / QMA6981_MAX_RATE + 5;

	mutex_lock(&qma6981->lock);
	qma6981->delay_ms = pollms;
	mutex_unlock(&qma6981->lock);

	return size;
}
static DRIVER_ATTR(poll, S_IRUGO | S_IWUSR, attr_get_poll, attr_set_poll);

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct QMA6981_data *qma6981 = dev_get_drvdata(dev);
	int val;

	mutex_lock(&qma6981->lock);
	val = qma6981->enabled;
	mutex_unlock(&qma6981->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct QMA6981_data *qma6981 = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	dev_dbg(&qma6981->client->dev, "val=%lu\n", val);

	mutex_lock(&qma6981->lock);
	if (val) {
		qma6981_enable(qma6981);
	} else {
		qma6981_disable(qma6981);
	}
	qma6981->enabled = val;
	mutex_unlock(&qma6981->lock);

	return size;
}


/*----------------------------------------------------------------------------*/
static int qma6981_init_client(struct i2c_client *client, int reset_cali)
{

	return QMA6981_SUCCESS;
}

/*----------------------------------------------------------------------------*/
static int qma6981_ReadChipInfo(char *buf, int bufsize)
{
	int res;	
	unsigned char databuf[1];
	unsigned char databuf2[2];
	int output[1]={ 0 };
	int output2[2] = {0};
	char strbuf[QMA6981_BUFSIZE];
	if((!buf)||(bufsize <= QMA6981_BUFSIZE -1))
	{
		return -1;
	}
	if(!this_client)
	{
		*buf = 0;
		return -2;
	}


	databuf[0] = QMA6981_CHIP_ID;		
	if(res = I2C_RxData(databuf, 1)){
		printk("QMA6981_CHIP_ID error!!!");
		return -EFAULT;	
	}
	output[0] = (int)databuf[0];

	databuf2[0] = QMA6981_DIE_ID;		
	if(res = I2C_RxData(databuf2, 2)){
		printk("QMA6981_DIE_ID error!!!");
		return -EFAULT;	
	}
	output2[0] = (int)databuf2[0];
	output2[1] = (int)databuf2[1];
	
	sprintf(strbuf, "chipid:%d dieid:%d %d\n", output[0], output2[0], output2[1]);
	
	sprintf(buf, "%s\n", strbuf);
	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	char strbuf[QMA6981_BUFSIZE];
	qma6981_ReadChipInfo(strbuf, QMA6981_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);
}


/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct QMA6981_data *obj = i2c_get_clientdata(this_client);
	if(NULL == obj)
	{
		printk(KERN_ERR "QMA6981_data is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, char *buf, size_t count)
{
	struct QMA6981_data *obj = i2c_get_clientdata(this_client);
	int trace;
	if(NULL == obj)
	{
		printk(KERN_ERR "QMA6981_data is null!!\n");
		return 0;
	}

	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}
	else
	{
		printk(KERN_ERR "invalid content: '%s', length = %ld\n", buf, count);
	}

	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = this_client;
	struct QMA6981_data *obj;
	int err, len = 0, mul;
	int tmp[QMA6981_AXES_NUM];

	if(NULL == client)
	{
		printk("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);



	if(0 != (err = QMA6981_ReadOffset(client, obj->offset)))
	{
		return -EINVAL;
	}
	else if(0 != (err = QMA6981_ReadCalibration(client, tmp)))
	{
		return -EINVAL;
	}
	else
	{    
		mul = 1000/256*9.8;
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,                        
			obj->offset[QMA6981_AXIS_X], obj->offset[QMA6981_AXIS_Y], obj->offset[QMA6981_AXIS_Z],
			obj->offset[QMA6981_AXIS_X], obj->offset[QMA6981_AXIS_Y], obj->offset[QMA6981_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1, 
			obj->cali_sw[QMA6981_AXIS_X], obj->cali_sw[QMA6981_AXIS_Y], obj->cali_sw[QMA6981_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n", 
			obj->offset[QMA6981_AXIS_X]*mul + obj->cali_sw[QMA6981_AXIS_X],
			obj->offset[QMA6981_AXIS_Y]*mul + obj->cali_sw[QMA6981_AXIS_Y],
			obj->offset[QMA6981_AXIS_Z]*mul + obj->cali_sw[QMA6981_AXIS_Z],
			tmp[QMA6981_AXIS_X], tmp[QMA6981_AXIS_Y], tmp[QMA6981_AXIS_Z]);
		
		return len;
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = this_client;  
	int err, x, y, z;
	int dat[QMA6981_AXES_NUM];

	if(!strncmp(buf, "rst", 3))
	{
		if(0 != (err = QMA6981_ResetCalibration(client)))
		{
			//printk("reset offset err = %d\n", err);
		}	
	}
	else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z))
	{
		dat[QMA6981_AXIS_X] = x;
		dat[QMA6981_AXIS_Y] = y;
		dat[QMA6981_AXIS_Z] = z;
		if(0 != (err = QMA6981_WriteCalibration(client, dat)))
		{
			//printk("write calibration err = %d\n", err);
		}		
	}
	else
	{
		printk("invalid format\n");
	}
	
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{

	int sensordata[3];
	char strbuf[QMA6981_BUFSIZE];

	QMA6981_read_acc_xyz(&sensordata);

	sprintf(strbuf, "%d %d %d\n", sensordata[0],sensordata[1],sensordata[2]);

	return sprintf(buf, "%s\n", strbuf);
}

/*----------------------------------------------------------------------------*/
static ssize_t show_power_status_value(struct device_driver *ddri, char *buf)
{
	u8 databuf[2];    
	u8 addr = QMA6981_REG_POWER_CTL;
	if(hwmsen_read_block(this_client, addr, databuf, 0x01))
	{
		printk("read power ctl register err!\n");
		return QMA6981_ERR_I2C;
	}
    
	if(sensor_power)
		printk("G sensor is in work mode, sensor_power = %d\n", sensor_power);
	else
		printk("G sensor is in standby mode, sensor_power = %d\n", sensor_power);

	return snprintf(buf, PAGE_SIZE, "%x\n", databuf[0]);
}


static DRIVER_ATTR(trace,       S_IRUGO | S_IWUSR, show_trace_value, store_trace_value);
static DRIVER_ATTR(chipinfo,    S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, show_cali_value,          store_cali_value);
static DRIVER_ATTR(status,               S_IRUGO, show_power_status_value,        NULL);

static struct driver_attribute *attributes[] = {
	&driver_attr_trace,
	&driver_attr_chipinfo,
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_status,
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++) {
		if (device_create_file(dev, attributes + i))
			goto error;
	}
	return 0;

error:
	for ( ; i >= 0; i--)
		device_remove_file(dev, attributes + i);
		dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}


struct QMA6981_data *qma6981;

#ifdef CONFIG_COMPAT
static long qma6981_compat_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
    long err = 0;

    void __user *arg32 = compat_ptr(arg);

    if (!file->f_op || !file->f_op->unlocked_ioctl)
        return -ENOTTY;
	switch (cmd) {
	
	case COMPAT_GSENSOR_IOCTL_INIT:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
            err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_INIT, arg32);
            if (err){
                printk("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
                return err;
            }
        break;


	case COMPAT_GSENSOR_IOCTL_READ_CHIPINFO:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
            err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_CHIPINFO, arg32);
            if (err){
                printk("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
                return err;
            }
        break;
	case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
            err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA, arg32);
            if (err){
                printk("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
                return err;
            }
        break;
	case COMPAT_GSENSOR_IOCTL_READ_RAW_DATA:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
            err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_RAW_DATA, arg32);
            if (err){
                printk("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
                return err;
            }
        break;
	case COMPAT_GSENSOR_IOCTL_SET_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
            err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, arg32);
            if (err){
                printk("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
                return err;
            }
        break;
	case COMPAT_GSENSOR_IOCTL_CLR_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
            err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_CLR_CALI, arg32);
            if (err){
                printk("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
                return err;
            }
        break;
	case COMPAT_GSENSOR_IOCTL_GET_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
            err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_CALI, arg32);
            if (err){
                printk("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
                return err;
            }
        break;
	default:
		break;
	}
	return 0;
}
    #endif


/*  ioctl command for QMA6981 device file */
static long qma6981_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	int err = 0;
	unsigned char data[6];
	void __user *val;
	char strbuf[QMA6981_BUFSIZE];
	int vec[3] = {0},st_counter;
	SENSOR_DATA sensor_data;
	int cali[3];

	struct i2c_client *client = (struct i2c_client*)file->private_data;

	struct QMA6981_data *obj = (struct QMA6981_data*)i2c_get_clientdata(client);	

	printk("qma6981_unlocked_ioctl - cmd=%u, arg = %lu\n" , cmd, arg);

	/* check QMA6981_client */
	if (&qma6981->client == NULL) {
		#if DEBUG
		printk(KERN_ERR "I2C driver not install\n");
		#endif
		return -EFAULT;
	}

	switch (cmd) {
	
	case GSENSOR_IOCTL_INIT:
		qma6981_initialize(client);
		break;
	case GSENSOR_IOCTL_READ_CHIPINFO:
		val = (void __user *) arg;
		if(val == NULL)
		{
			err = -EINVAL;
			break;	  
		}
		
		qma6981_ReadChipInfo(strbuf, QMA6981_BUFSIZE);
		if(copy_to_user(val, strbuf, strlen(strbuf)+1))
		{
			err = -EFAULT;
			break;
		}	
		break;
	case GSENSOR_IOCTL_READ_SENSORDATA:

		val = (void __user *) arg;
		if(val == NULL)
		{
			err = -EINVAL;
			break;	  
		}
		

		QMA6981_read_acc_xyz(vec);
		sprintf(strbuf, "%04x %04x %04x", vec[0], vec[1], vec[2]);
		if(copy_to_user(val, strbuf, strlen(strbuf)+1))
		{
			err = -EFAULT;
			break;	  
		}	
		break;

	case GSENSOR_IOCTL_READ_RAW_DATA:
		val = (void __user *) arg;
		if(val == NULL)
		{
			err = -EINVAL;
			break;	  
		}
		QMA6981_read_raw_xyz(vec);
		
		sprintf(strbuf, "%04x %04x %04x", vec[0], vec[1], vec[2]);
		if(copy_to_user(val, strbuf, strlen(strbuf)+1))
		{
			err = -EFAULT;
			break;	  
		}
		break;	  

	case GSENSOR_IOCTL_SET_CALI:
		//printk("qma6981 ioctl GSENSOR_IOCTL_SET_CALI\n");
		val = (void __user*)arg;
		if(val == NULL)
		{
			err = -EINVAL;
			break;	  
		}
		if(copy_from_user(&sensor_data, val, sizeof(sensor_data)))
		{
			err = -EFAULT;
			break;	  
		}
		if(atomic_read(&obj->suspend))
		{
			//printk("Perform calibration in suspend state!!\n");
			err = -EINVAL;
		}
		else
		{
			cali[QMA6981_AXIS_X] = sensor_data.x;
			cali[QMA6981_AXIS_Y] = sensor_data.y;
			cali[QMA6981_AXIS_Z] = sensor_data.z;			  
			err = QMA6981_WriteCalibration(client, cali);			 
		}
		break;

	case GSENSOR_IOCTL_CLR_CALI:
		//printk("qma6981 ioctl GSENSOR_IOCTL_CLR_CALI\n");
		err = QMA6981_ResetCalibration(client);
		break;

	case GSENSOR_IOCTL_GET_CALI:
		//printk("qma6981 ioctl GSENSOR_IOCTL_GET_CALI\n");
		val = (void __user*)arg;
		if(val == NULL)
		{
			err = -EINVAL;
			break;	  
		}
		if(0 != (err = QMA6981_ReadCalibration(client, cali)))
		{
			break;
		}
		//printk("qma6981 get cali.x,y,z[%d,%d,%d]\n",cali[QMA6981_AXIS_X],cali[QMA6981_AXIS_Y],cali[QMA6981_AXIS_Z]);
		sensor_data.x = cali[QMA6981_AXIS_X];
		sensor_data.y = cali[QMA6981_AXIS_Y];
		sensor_data.z = cali[QMA6981_AXIS_Z];
		//printk("qma6981 get sensor_data.x,y,z[%d,%d,%d]\n",sensor_data.x,sensor_data.y,sensor_data.z);
		if(copy_to_user(val, &sensor_data, sizeof(sensor_data)))
		{
			err = -EFAULT;
			break;
		}		
		break;
/*	case GSENSOR_IOCTL_ENABLE_STEP_COUNTER:
		err = qma6981_stepCounter_enable(client);
		break;
	case GSENSOR_IOCTL_READ_STEP_COUNTER:
		val = (void __user *) arg;
		if(val == NULL)
		{
			err = -EINVAL;
			break;	  
		}
		

		QMA6981_read_stepCounter(&st_counter);
		sprintf(strbuf, "%04x", st_counter);
		if(copy_to_user(val, strbuf, strlen(strbuf)+1))
		{
			err = -EFAULT;
			break;	  
		}	
		break;
*/	
	default:
		break;
	}
	return 0;
}

static int qma6981_initialize(struct i2c_client *client)
{
	int ret = 0;
	unsigned char data[2] = {0};


				
	data[0] = 0x0F;
	data[1] = 0x01;
	ret = I2C_TxData(data,2);
//    printk("qma6981_initialize  %d\n",ret);
    if(ret < 0)
	{
      printk("qma6981_initialize fail: %d\n",ret);
	  return ret;
	 }
	data[0] = 0x10;
	data[1] = 0x2c;
	ret = I2C_TxData(data,2);
    if(ret < 0)
		return ret;
      //printk("qma6981_initialize fail: %d\n",ret);
	data[0] = 0x11;
	data[1] = 0x80;
	ret = I2C_TxData(data,2);
    if(ret < 0)
      return ret;
	data[0] = 0x4A;
	data[1] = 0x08;
	ret = I2C_TxData(data,2);
    if(ret < 0)
      return ret;
	data[0] = 0x5F;
	data[1] = 0x70;
	ret = I2C_TxData(data,2);
    if(ret < 0)
      return ret;

	/************************add offset*******************/	
	{
		unsigned char databuf2[1];	
		int output2[1]={ 0 };	
		int res;
		databuf2[0] = 0x00;		
		if(res = I2C_RxData(databuf2, 1)){
			return -EFAULT;	
		}	

		output2[0] = (int)databuf2[0];
		if((int)output2[0] == 0xa9){
			// printk("QMA6981_resume!!!!!!!!\n");
			ready_to_read_data();
		}
	}
	/************************add offset*******************/
	
	return 0;
}
#if 0
static int qma6981_stepCounter_enable(struct i2c_client *client)
{
	int ret = 0;
	unsigned char data[2] = {0};
	printk("qma6981_stepCounter_init  %d\n",ret);
#if 0
	data[0] = 0x0F;
	data[1] = 0x01;
	ret = I2C_TxData(data,2);
    if(ret < 0)
      printk("qma6981_initialize fail: %d\n",ret);
	
	data[0] = 0x10;
	data[1] = 0x2c;
	ret = I2C_TxData(data,2);
    if(ret < 0)
      printk("qma6981_initialize fail: %d\n",ret);
	data[0] = 0x11;
	data[1] = 0x80;
	ret = I2C_TxData(data,2);
    if(ret < 0)
      printk("qma6981_initialize fail: %d\n",ret);
	data[0] = 0x4A;
	data[1] = 0x08;
	ret = I2C_TxData(data,2);
    if(ret < 0)
      printk("qma6981_initialize fail: %d\n",ret);
	data[0] = 0x5F;
	data[1] = 0x70;
	ret = I2C_TxData(data,2);
    if(ret < 0)
      printk("qma6981_initialize fail: %d\n",ret);
#endif

	data[0] = 0x11;
	data[1] = 0X8a;
	ret = I2C_TxData(data,2);
    	if(ret < 0)
       	printk("qma6981_initialize fail: %d\n",ret);

	data[0] = 0x10;
	data[1] = 0x0C;
	ret = I2C_TxData(data,2);
    	if(ret < 0)
       	printk("qma6981_initialize fail: %d\n",ret);


	data[0] = 0x13;
	data[1] = 0X20;
	ret = I2C_TxData(data,2);
    	if(ret < 0)
       	printk("qma6981_initialize fail: %d\n",ret);

	data[0] = 0x14;
	data[1] = 0x0f;
	ret = I2C_TxData(data,2);
    	if(ret < 0)
       	printk("qma6981_initialize fail: %d\n",ret);

	data[0] = 0x15;
	data[1] = 0x0f;
	ret = I2C_TxData(data,2);
    	if(ret < 0)
       	printk("qma6981_initialize fail: %d\n",ret);

	data[0] = 0x32;
	data[1] = 0x00;
	ret = I2C_TxData(data,2);
    	if(ret < 0)
       	printk("qma6981_initialize fail: %d\n",ret);

	data[0] = 0x16;
	data[1] = 0x08;
	ret = I2C_TxData(data,2);
    	if(ret < 0)
       	printk("qma6981_initialize fail: %d\n",ret);

	data[0] = 0x12;
	data[1] = 0x8c;
	ret = I2C_TxData(data,2);
    	if(ret < 0)
       	printk("qma6981_initialize fail: %d\n",ret);

	return 0;
}
#endif


/*----------------------------------------------------------------------------*/
static int qma6981_open(struct inode *inode, struct file *file)
{
	file->private_data = this_client;

	if(file->private_data == NULL)
	{
		printk("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int qma6981_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static const struct file_operations qma6981_fops = {
	.owner = THIS_MODULE,
	.open = qma6981_open,
	.release = qma6981_release,
	.unlocked_ioctl = qma6981_unlocked_ioctl,
    #ifdef CONFIG_COMPAT
    .compat_ioctl = qma6981_compat_ioctl,
    #endif
};

static const struct miscdevice qma6981_miscdevice = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &qma6981_fops,
};

#ifdef QMA6981_MOTION_VIRTUAL_SENSOR

static unsigned char last_tilt = 0;
struct input_dev * qma6981_motion_dev;

/*-----------------------------------------------------------------------------
 * Motion Interrupt handler
 */

static void qma6981_worker(struct work_struct *work)
{
	unsigned char val = 0;
	 QMA6981_i2c_read(QMA6981_INT_STAT,&val,1);

	if ((val & 0x40) && (val & 0x40) != (last_tilt & 0x40)) {
		//printk("QMA6981 ORIENT motion detected!\n");
		//printk("QMA6981 title is %d!\n",val);
		input_report_abs(qma6981_motion_dev, ABS_X, val);
		input_sync(qma6981_motion_dev);
	}	
	/* Save current status */
	last_tilt = val;
	//mutex_unlock(&qma6981->motionLock);
}


static irqreturn_t qma6981_interrupt(int irq, void *dev_id)
{
	schedule_delayed_work(&qma6981->motionWork,msecs_to_jiffies(0));
	return IRQ_HANDLED;
}

static int qma6981_virtual_motion_input_init()
{
	struct input_dev *dev;
	int ret;
	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	dev->name = "qma6981_motion";
	dev->id.bustype = BUS_I2C;

	set_bit(EV_ABS, dev->evbit);
	set_bit(ABS_X, dev->absbit);
	//set_bit(ABS_Y, dev->absbit);
	//set_bit(ABS_Z, dev->absbit);
	input_set_abs_params(dev, ABS_X, -512, 512, 4, 4);
	//input_set_abs_params(dev, ABS_Y, -512, 512, 4, 4);
	//input_set_abs_params(dev, ABS_Z, -512, 512, 4, 4);
	input_set_drvdata(dev, qma6981);

	ret = input_register_device(dev);
	if (ret < 0) {
		input_free_device(dev);
		return ret;
	}

	qma6981_motion_dev = dev;
	return 0;
}

#endif

static int QMA6981_SetPowerCTRL(struct i2c_client *client, bool enable)
{
    int res = 0;
    struct QMA6981_data *obj = i2c_get_clientdata(client);
	
    u8 databuf[2];

    if (enable == sensor_power)
    {
//        printk("Sensor power status is newest!\n");
        return QMA6981_SUCCESS;
    }

    databuf[0] = QMA6981_REG_POWER_CTL;   
    res = i2c_master_send(client, databuf, 0x1);
    if (res <= 0)
    {
        return QMA6981_ERR_I2C;
    }

    udelay(500);

    databuf[0] = 0x0;        
    res = i2c_master_recv(client, databuf, 1);
    if (res <= 0)
    {
        return QMA6981_ERR_I2C;
    }

    if (enable == FALSE)
    {
        databuf[0]=0x00;
    }
    else
    {
        databuf[0]=0x80;
    }
    databuf[1] = databuf[0];
    databuf[0] = QMA6981_REG_POWER_CTL;

    res = i2c_master_send(client, databuf, 0x2);

    if (res <= 0)
    {
//        printk("set power mode failed!\n");
        return QMA6981_ERR_I2C;
    }

    if (atomic_read(&obj->trace) & QMA_I2C_DEBUG)
    {
        printk("set power mode ok %d!\n", databuf[1]);
    }

    sensor_power = enable;
    return QMA6981_SUCCESS;    
}

static int qma6981_open_report_data(int open)
{
	return 0;
}

static int qma6981_enable_nodata(int en)
{
    int err = 0;

	if(((en == 0) && (sensor_power == false)) ||((en == 1) && (sensor_power == true)))
	{
		enable_status = sensor_power;
		//printk("Gsensor device have updated!\n");
	}
	else
	{
		enable_status = !sensor_power;
		if (atomic_read(&qma6981->suspend) == 0)
		{

			err = QMA6981_SetPowerCTRL(qma6981->client, enable_status);
			//printk("Gsensor not in suspend gsensor_SetPowerMode!, enable_status = %d\n",enable_status);
		}
		else
		{
			//printk("Gsensor in suspend and can not enable or disable!enable_status = %d\n",enable_status);
		}
	}

    if(err != 0)
	{
		printk("gsensor_enable_nodata fail!\n");
		return -1;
	}

//    printk("gsensor_enable_nodata OK!!!\n");
	return 0;
}

/*----------------------------------------------------------------------------*/
static int QMA6981_SetBWRate(struct i2c_client *client, u8 bwrate)
{
    struct QMA6981_data *obj = i2c_get_clientdata(client);
    u8 databuf[10];    
    int res = 0;

#ifdef GSENSOR_UT
    GSE_FUN();
#endif

    if( (obj->bandwidth != bwrate) || (atomic_read(&obj->suspend)) )
    {    
        memset(databuf, 0, sizeof(u8)*10);    
    
        /* read */
        databuf[0] = QMA6981_REG_BW_RATE;
        res = i2c_master_send(client, databuf, 0x1);
        if (res <= 0)
        {
            return QMA6981_ERR_I2C;
        }
    
        udelay(500);
    
        databuf[0] = 0x0;        
        res = i2c_master_recv(client, databuf, 0x01);
        if (res <= 0)
        {
            return QMA6981_ERR_I2C;
        }
    
    
        /* write */
        databuf[1] = databuf[0] | bwrate;
        databuf[0] = QMA6981_REG_BW_RATE;    
    
        res = i2c_master_send(client, databuf, 0x2);
    
        if (res <= 0)
        {
            return QMA6981_ERR_I2C;
        }

    obj->bandwidth = bwrate;
    }

    return 0;    
}
static int qma6981_set_delay(long ns)
{
    int err = 0;
    int value;
#ifdef CUSTOM_KERNEL_SENSORHUB
    SCP_SENSOR_HUB_DATA req;
    int len;
#else//#ifdef CUSTOM_KERNEL_SENSORHUB
	int sample_delay;
#endif//#ifdef CUSTOM_KERNEL_SENSORHUB

#ifdef GSENSOR_UT
    GSE_FUN();
#endif

    value = (int)ns/1000/1000;
    printk("gsensor_set_delay1 (value = %d,ns = %ld)\n",value,ns);

#ifdef CUSTOM_KERNEL_SENSORHUB
    req.set_delay_req.sensorType = ID_ACCELEROMETER;
    req.set_delay_req.action = SENSOR_HUB_SET_DELAY;
    req.set_delay_req.delay = value;
    len = sizeof(req.activate_req);
    err = SCP_sensorHub_req_send(&req, &len, 1);
    if (err)
    {
//        printk("SCP_sensorHub_req_send!\n");
        return err;
    }
#else//#ifdef CUSTOM_KERNEL_SENSORHUB    
	if(value <= 5)
	{
		sample_delay = QMA6981_BW_125HZ;
	}
	else if(value <= 10)
	{
		sample_delay = QMA6981_BW_62HZ;
	}
	else
	{
		sample_delay = QMA6981_BW_31HZ;
	}

	err = QMA6981_SetBWRate(qma6981->client, sample_delay);

	if(err != 0 ) //0x2C->BW=100Hz
	{
		printk("Set delay parameter error!\n");
        return -1;
	}

#endif//#ifdef CUSTOM_KERNEL_SENSORHUB
    
    printk("gsensor_set_delay2 (value = %d,sample_delay = %d)\n",value , sample_delay);

	return 0;
}

static int qma6981_get_data(int* x ,int* y,int* z, int* status)
{
	int vec[3]={0};

	QMA6981_read_acc_xyz(vec);

	*x = vec[0];
	*y = vec[1];
	*z = vec[2];
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
    return 0;
}

static int QMA6981_probe(struct i2c_client *client, const struct i2c_device_id *devid)
{

	//struct device *dev;
	int err = 0;
	unsigned char val;
	//struct QMA6981_data *obj;
	struct acc_control_path ctl={0};
	struct acc_data_path data={0};
	struct i2c_client *new_client;
	
	//printk(KERN_ALERT "QMA6981_probe --- 1\n");

	qma6981 = kzalloc(sizeof(struct QMA6981_data), GFP_KERNEL);

	if (qma6981 == NULL) {
		err = -ENOMEM;
		goto exit;
	}
	//printk(KERN_ALERT "QMA6981_probe --- 2\n");

	
	memset(qma6981, 0, sizeof(struct QMA6981_data));

	qma6981->hw = qma6981_get_cust_acc_hw();

	//printk(KERN_ALERT "QMA6981_probe --- 3\n");
	
	if(0 != (err = hwmsen_get_convert(qma6981->hw->direction, &qma6981->cvt)))
	{
		printk("invalid direction: %d\n", qma6981->hw->direction);
		goto exit;
	}
	//printk(KERN_ALERT "QMA6981_probe --- 4\n");
	qma6981->client = client;
	this_client = qma6981->client;	 
	i2c_set_clientdata(this_client, qma6981);

 
	qma6981->delay_ms = 100;
    	client->timing = 100;
	qma6981->hw = qma6981_get_cust_acc_hw();
	//printk(KERN_ALERT "QMA6981_probe --- 5\n");
	if(0 > qma6981_initialize(client)){
		goto exit_kfree;
	}	

	mutex_init(&qma6981->lock);
	mutex_init(&read_i2c_xyz);
//#ifdef CUSTOM_KERNEL_SENSORHUB
	INIT_DELAYED_WORK(&qma6981->work, qma6981_work);
//#endif
	//printk(KERN_ALERT "QMA6981_probe --- 6\n");
	acc = qma6981;
    	atomic_set(&qma6981->trace, 0);
    	atomic_set(&qma6981->suspend, 0);
	/* Create input device for qma6981 */
	err = qma6981_input_init(qma6981);
	if (err < 0) {
		dev_err(&client->dev, "error init input dev interface\n");
		goto exit_kfree;
	}
	//printk(KERN_ALERT "QMA6981_probe --- 7\n");
	err = misc_register(&qma6981_miscdevice);
	if(err){
		dev_err(&client->dev, "misc register failed\n");
		goto exit_kfree_sysfs;
	}
	//printk(KERN_ALERT "QMA6981_probe --- 8\n");
	if(0 != (err = QMA6981_create_attr(&qma6981_init_info.platform_diver_addr->driver)))
	{
		dev_err(&client->dev,"create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	//printk(KERN_ALERT "QMA6981_probe --- 9\n");
	if(0 != (err = qma6981_init_client(qma6981->client, 1)))
	{
		goto exit_kfree;
	}
	//printk(KERN_ALERT "QMA6981_probe --- 10\n");
	ctl.open_report_data= qma6981_open_report_data;
	ctl.enable_nodata =qma6981_enable_nodata;
	ctl.set_delay  = qma6981_set_delay;
	ctl.is_report_input_direct = false;
#ifdef CUSTOM_KERNEL_SENSORHUB
    ctl.is_support_batch = qma6981->hw->is_batch_supported;
#else
    ctl.is_support_batch = false;
#endif
	//printk(KERN_ALERT "QMA6981_probe --- 11\n");
	err = acc_register_control_path(&ctl);
	if(err)
	{
	 	//printk("register acc control path err\n");
		goto exit_create_attr_failed;
	}
	//printk(KERN_ALERT "QMA6981_probe --- 12\n");
	data.get_data = qma6981_get_data;
	data.vender_div = 1000;
	err = acc_register_data_path(&data);
	if(err)
	{
	 	printk("register acc data path err\n");
		goto exit_create_attr_failed;
	}

	err = batch_register_support_info(ID_ACCELEROMETER,ctl.is_support_batch, 1000,0);

	//printk("QMA6981 device created successfully\n");

    qma6981_init_flag = 0;
	
	return 0;


exit_create_attr_failed:
	misc_deregister(&qma6981_miscdevice);
#ifdef QMA6981_MOTION_VIRTUAL_SENSOR
exit_virtual_input_dev:
	input_unregister_device(qma6981_motion_dev);
#endif

exit_kfree_sysfs:
	remove_sysfs_interfaces(&qma6981->input->dev);


exit_kfree_input:
	input_unregister_device(qma6981->input);
exit_kfree:
	kfree(qma6981);
exit:
    qma6981_init_flag = -1;
	return err;
}

static int qma6981_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(attributes)/sizeof(attributes[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, attributes[idx]);
	}
	

	return err;
}

static int QMA6981_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(attributes)/sizeof(attributes[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(0 != (err = driver_create_file(driver, attributes[idx])))
		{            
			printk("attributes (%s) = %d\n", attributes[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}

static int QMA6981_i2c_remove(struct i2c_client *client)
{
	int err = 0;
	struct QMA6981_data *dev = i2c_get_clientdata(client);
	//#if DEBUG
		//printk("QMA6981 driver removing\n");
	//#endif
#ifdef QMA6981_MOTION_VIRTUAL_SENSOR
	free_irq(qma6981->pdata->irq, qma6981->input);
	input_unregister_device(qma6981_motion_dev);
#endif
	
	if(0 != (err = qma6981_delete_attr(&(qma6981_init_info.platform_diver_addr->driver))))
	{
		printk("qma6981_delete_attr fail: %d\n", err);
	}
	
	
	misc_deregister(&qma6981_miscdevice);
	device_destroy(qma_acc_dev_class, MKDEV(QMA6981_MAJOR, 0));
	class_destroy(qma_acc_dev_class);
	unregister_chrdev(QMA6981_MAJOR, "QMA6981");
	remove_sysfs_interfaces(&qma6981->input->dev);
	kfree(dev);
	acc->client = NULL;
	i2c_unregister_device(client);
	this_client = NULL;
	return 0;


}



/*----------------------------------------------------------------------------*/
static void qma6981_power(struct acc_hw *hw, unsigned int on)
{
	static unsigned int power_on = 0;

	if(hw->power_id != MT65XX_POWER_NONE)
	{
		//printk("power %s\n", on ? "on" : "off");
		if(power_on == on)
		{
			//printk("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "qma6981"))
			{
				printk(KERN_ERR "power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "qma6981"))
			{
				printk(KERN_ERR "power off fail!!\n");
			}
		}
	}
	power_on = on;
}

static int qma6981_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2];
	int res = 0;

	struct QMA6981_data *obj = i2c_get_clientdata(client);


	if(enable == true)
	{
		if(qma6981_enable(client))
		{
			printk("qma6981: set power mode failed!\n");
			return -1;
		}
		else
		{
			//printk("qma6981: set power mode enable ok!\n");
		}
	}
	else
	{
		if(qma6981_disable(client))
		{
			printk("qma6981: set power mode failed!\n");
			return -1;
		}
		else
		{
			//printk("qma6981: set power mode disable ok!\n");
		}
	}

	return 0;
}

static int ready_to_read_data(void)
{
	int ret = 0;
	unsigned char data[2] = {0};
	//printk("qma6981_ready_to_read_data\n");

	data[0] = 0x27;
	data[1] = 0Xd4;
	ret = I2C_TxData(data,2);
    	if(ret < 0)
       		printk("qma6981_ready_to_read_data fail: %d\n",ret);
	data[0] = 0x28;
	data[1] = 0XF7;
	ret = I2C_TxData(data,2);
    	if(ret < 0)
       		printk("qma6981_ready_to_read_data fail: %d\n",ret);

	data[0] = 0x29;
	data[1] = 0Xe5;
	ret = I2C_TxData(data,2);
    	if(ret < 0)
       		printk("qma6981_ready_to_read_data fail: %d\n",ret);
}

#if !defined(CONFIG_HAS_EARLYSUSPEND) || !defined(USE_EARLY_SUSPEND)
/*----------------------------------------------------------------------------*/
static int QMA6981_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct QMA6981_data *obj = i2c_get_clientdata(client);

	//printk("QMA6981_suspend\n");
	atomic_set(&obj->suspend, 1);
	if(msg.event == PM_EVENT_SUSPEND)
	{
		qma6981_power(obj->hw, 0);
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int QMA6981_resume(struct i2c_client *client)
{
	struct QMA6981_data *obj = i2c_get_clientdata(client);
	int err;
		
	//printk("QMA6981_resume\n");
	
	qma6981_power(obj->hw, 1);
	if(0 != (err = qma6981_init_client(client, 0)))
	{
		//printk("initialize client fail!!\n");
		return err;        
	}
	atomic_set(&obj->suspend, 0);

	return 0;
}
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void QMA6981_early_suspend(struct early_suspend *h)
{
	struct QMA6981_data *obj = container_of(h, struct QMA6981_data, early_drv);

	if(NULL == obj)
	{
		printk(KERN_ERR "null pointer!!\n");
		return;
	}
	if(qma6981_SetPowerMode(obj->client, false))
	{
		printk("qma6981: write power control fail!!\n");
		return;
	}

}
/*----------------------------------------------------------------------------*/
static void QMA6981_late_resume(struct early_suspend *h)
{
	struct QMA6981_data *obj = container_of(h, struct QMA6981_data, early_drv);


	if(NULL == obj)
	{
		printk(KERN_ERR "null pointer!!\n");
		return;
	}

	if(qma6981_SetPowerMode(obj->client, true))
	{
		printk("qma6981: write power control fail!!\n");
		return;
	}

}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/



static const struct i2c_device_id QMA6981_id[] = {{QMA6981_DEV_NAME,0},{}};

static struct i2c_driver QMA6981_driver = {
//	.class = I2C_CLASS_HWMON,
	.probe = QMA6981_probe,
	.remove = QMA6981_i2c_remove,
	.id_table = QMA6981_id,
#if !defined(CONFIG_HAS_EARLYSUSPEND) || !defined(USE_EARLY_SUSPEND)
	.suspend = QMA6981_suspend,
	.resume = QMA6981_resume,
#endif
	.driver = {
		.owner = THIS_MODULE,
		.name = "qma6981",
	},
	
};

static int  qma6981_local_init(void)
{
    struct acc_hw *hw = qma6981_get_cust_acc_hw();
	//printk("qma6981_local_init before power\n");

	qma6981_power(hw, 1);//power on
	if(i2c_add_driver(&QMA6981_driver))
	{
		//printk("add driver error\n");
		return -1;
	}
	//printk("fwq loccal init---\n");

    if(-1 == qma6981_init_flag)
    {
       printk("qma6981, add driver fail\n");
       return -1;
    }
    return 0;
}

static int qma6981_local_remove(void)
{
    struct acc_hw *hw = qma6981_get_cust_acc_hw();

    //GSE_FUN();    
    qma6981_power(hw, 0);    
    i2c_del_driver(&QMA6981_driver);
    return 0;
}


static struct acc_init_info qma6981_init_info = {
		.name = "qma6981",
		.init = qma6981_local_init,
		.uninit = qma6981_local_remove,
	
};

static int __init QMA6981_init(void)
{
	int ret = 0;
	struct acc_hw *hw = qma6981_get_cust_acc_hw();
	//printk(KERN_ALERT "QMA6981_init\n");
	i2c_register_board_info(hw->i2c_num, &i2c_qma6981, 1);
	/* add i2c driver for QMA6981 accelerometer */
	acc_driver_add(&qma6981_init_info);
	//printk(KERN_ALERT "QMA6981_init, ret = %d\n", ret);
	return ret;
}

static void __exit QMA6981_exit(void)
{
	//#if DEBUG
	//printk("QM66981 exit\n");
	//#endif
	i2c_del_driver(&QMA6981_driver);
	return;
}

module_init(QMA6981_init);
module_exit(QMA6981_exit);

MODULE_DESCRIPTION("QST QMA6981 Acc driver");
MODULE_AUTHOR("QST");
MODULE_LICENSE("GPL");

