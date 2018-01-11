/* qmc6983.c - qmc6983 compass driver
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
#include <asm/atomic.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/earlysuspend.h>
#include <linux/time.h>
#include <linux/hrtimer.h>
#include <linux/completion.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>


#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>


#include <cust_mag.h>
#include "qmc6983.h"
#include <linux/hwmsen_helper.h>
/*----------------------------------------------------------------------------*/
#define DEBUG 0
#define QMC6983_DEV_NAME         "qmc6983"
#define DRIVER_VERSION          "1.0.1"
/*----------------------------------------------------------------------------*/
#define QMC6983_DEBUG		1
#define QMC6983_DEBUG_MSG	1
#define QMC6983_DEBUG_FUNC	1
#define QMC6983_DEBUG_DATA	1
#define MAX_FAILURE_COUNT	3
#define QMC6983_RETRY_COUNT	10
#define	QMC6983_BUFSIZE		0x20

#define QMC6983_AD0_CMP		1

#define QMC6983_AXIS_X            0
#define QMC6983_AXIS_Y            1
#define QMC6983_AXIS_Z            2
#define QMC6983_AXES_NUM          3

#define QMC6983_DEFAULT_DELAY 100
#define CALIBRATION_DATA_SIZE   12


#if QMC6983_DEBUG_MSG
#define QMCDBG(format, ...)	printk(KERN_INFO "qmc6983 " format "\n", ## __VA_ARGS__)
#else
#define QMCDBG(format, ...)
#endif

#if QMC6983_DEBUG_FUNC
#define QMCFUNC(func) printk(KERN_INFO "qmc6983 " func " is called\n")
#else
#define QMCFUNC(func)
#endif

#define MSE_TAG					"[Msensor] "
#define MSE_FUN(f)				printk(MSE_TAG"QMC6983---%s\n", __func__)
#define MSE_ERR(fmt, args...)		printk(KERN_ERR MSE_TAG"%s %d : "fmt, __func__, __LINE__, ##args)
#define MSE_LOG(fmt, args...)		printk(MSE_TAG fmt, ##args)



static struct i2c_client *this_client = NULL;
extern int mag_driver_add(struct mag_init_info* obj);

static short qmcd_delay = QMC6983_DEFAULT_DELAY;


// calibration msensor and orientation data
static int sensor_data[CALIBRATION_DATA_SIZE];
static struct mutex sensor_data_mutex;
static struct mutex read_i2c_xyz;
static struct mutex read_i2c_temperature;
static struct mutex read_i2c_register;
static unsigned char regbuf[2] = {0};

static DECLARE_WAIT_QUEUE_HEAD(data_ready_wq);
static DECLARE_WAIT_QUEUE_HEAD(open_wq);

static atomic_t open_flag = ATOMIC_INIT(0);
static atomic_t m_flag = ATOMIC_INIT(0);
static atomic_t o_flag = ATOMIC_INIT(0);

/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id qmc6983_i2c_id[] = {{QMC6983_DEV_NAME,0},{}};
//static struct i2c_board_info __initdata i2c_qmc6983={ I2C_BOARD_INFO("qmc6983", (0X2c))};
/*the adapter id will be available in customization*/
//static unsigned short qmc6983_force[] = {0x00, QMC6983_I2C_ADDR, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const qmc6983_forces[] = { qmc6983_force, NULL };
//static struct i2c_client_address_data qmc6983_addr_data = { .forces = qmc6983_forces,};
/*----------------------------------------------------------------------------*/

/* Maintain  cust info here */
struct mag_hw mag_cust;
static struct mag_hw *hw = &mag_cust;

/* For  driver get cust info */
struct mag_hw *get_cust_mag(void) {
    return &mag_cust;
}
static int qmc6983_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int qmc6983_i2c_remove(struct i2c_client *client);
//static int qmc6983_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);

static int qmc6983_suspend(struct i2c_client *client, pm_message_t msg);
static int qmc6983_resume(struct i2c_client *client);

static int qmc6983_init_flag =0; 
static int qmc6983_remove(void);
static int qmc6983_local_init(void);

static int counter = 0;

DECLARE_COMPLETION(data_updated);
//struct completion data_updated;

/*----------------------------------------------------------------------------*/
typedef enum {
    QMC_FUN_DEBUG  = 0x01,
	QMC_DATA_DEBUG = 0X02,
	QMC_HWM_DEBUG  = 0X04,
	QMC_CTR_DEBUG  = 0X08,
	QMC_I2C_DEBUG  = 0x10,
} QMC_TRC;


/*----------------------------------------------------------------------------*/
struct qmc6983_i2c_data {
    struct i2c_client *client;
    struct mag_hw *hw;
    atomic_t layout;
    atomic_t trace;
	struct hwmsen_convert   cvt;
	//add for qmc6983 start    for layout direction and M sensor sensitivity------------------------
#if 0
	struct QMC6983_platform_data *pdata;
#endif
	short xy_sensitivity;
	short z_sensitivity;
	//add for qmc6983 end-------------------------
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif
};

struct delayed_work data_avg_work;
#define DATA_AVG_DELAY 6
/*----------------------------------------------------------------------------*/
static struct i2c_driver qmc6983_i2c_driver = {
    .driver = {
//      .owner = THIS_MODULE,
        .name  = QMC6983_DEV_NAME,
    },
	.probe      = qmc6983_i2c_probe,
	.remove     = qmc6983_i2c_remove,
//	.detect     = qmc6983_i2c_detect,
#if !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend    = qmc6983_suspend,
	.resume     = qmc6983_resume,
#endif
	.id_table = qmc6983_i2c_id,
	//.address_data = &qmc6983_addr_data,
};

static struct sensor_init_info qmc6983_init_info = {
    .name = "qmc6983",
    .init = qmc6983_local_init,
    .uninit = qmc6983_remove,
  
};
static int I2C_RxData(char *rxData, int length)
{
	uint8_t loop_i;
    int res = 0;

#if DEBUG
	int i;
	struct i2c_client *client = this_client;
	struct qmc6983_i2c_data *data = i2c_get_clientdata(client);
	char addr = rxData[0];
#endif


	/* Caller should check parameter validity.*/
	if((rxData == NULL) || (length < 1))
	{
		return -EINVAL;
	}

	for(loop_i = 0; loop_i < QMC6983_RETRY_COUNT; loop_i++)
	{
		this_client->addr = this_client->addr & I2C_MASK_FLAG | I2C_WR_FLAG;
		res = i2c_master_send(this_client, (const char*)rxData, ((length<<0X08) | 0X01));
		if(res > 0)
		{
			break;
		}
		mdelay(10);
	}
    
	if(loop_i >= QMC6983_RETRY_COUNT)
	{
		return -EIO;
	}
#if DEBUG
	if(atomic_read(&data->trace) & QMC_I2C_DEBUG)
	{
		printk(KERN_INFO "RxData: len=%02x, addr=%02x\n  data=", length, addr);
		for(i = 0; i < length; i++)
		{
			printk(KERN_INFO " %02x", rxData[i]);
		}
	    printk(KERN_INFO "\n");
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
	struct qmc6983_i2c_data *data = i2c_get_clientdata(client);
#endif

	/* Caller should check parameter validity.*/
	if ((txData == NULL) || (length < 2))
	{
		return -EINVAL;
	}

	this_client->addr = this_client->addr & I2C_MASK_FLAG;
	for(loop_i = 0; loop_i < QMC6983_RETRY_COUNT; loop_i++)
	{
		if(i2c_master_send(this_client, (const char*)txData, length) > 0)
		{
			break;
		}
		mdelay(10);
	}

	if(loop_i >= QMC6983_RETRY_COUNT)
	{
		return -EIO;
	}
#if DEBUG
	if(atomic_read(&data->trace) & QMC_I2C_DEBUG)
	{
		printk(KERN_INFO "TxData: len=%02x, addr=%02x\n  data=", length, txData[0]);
		for(i = 0; i < (length-1); i++)
		{
			printk(KERN_INFO " %02x", txData[i + 1]);
		}
		printk(KERN_INFO "\n");
	}
#endif
	return 0;
}

static int send_data(struct i2c_client *client, char addr, char d)
{
		unsigned char data[2];

		data[0] = addr;
		data[1] = d;
		return I2C_TxData(data, 2);
}

#define FILTER_WINDOW 3
static int mdatax[FILTER_WINDOW] = {0};
static int mdatay[FILTER_WINDOW] = {0};
static int mdataz[FILTER_WINDOW] = {0};

static int get_latest_mag_xyz(int *data)
{
	int ret = 0;
	int i;
	MSE_FUN();
	mutex_lock(&read_i2c_xyz);
	data[0] = data[1] = data[2] = 0;
	for(i=0; i<FILTER_WINDOW; i++){
		data[0] += mdatax[i];
		data[1] += mdatay[i];
		data[2] += mdataz[i];
	}
	//MSE_LOG("QMC6983 Count = [%d, %d, %d] _before\n", data[0], data[1], data[2]);
	data[0] = data[0] / FILTER_WINDOW;
	data[1] = data[1] / FILTER_WINDOW;
	data[2] = data[2] / FILTER_WINDOW;
	//MSE_LOG("QMC6983 Count = [%d, %d, %d] _after\n", data[0], data[1], data[2]);
	mutex_unlock(&read_i2c_xyz);
	return ret;
}

static int firstData = 1;

/* X,Y and Z-axis magnetometer data readout
 * param *mag pointer to \ref QMC6983_t structure for x,y,z data readout
 * note data will be read by multi-byte protocol into a 6 byte structure
 */
static int QMC6983_read_mag_xyz(int *data)
{
	int res;
	unsigned char mag_data[6];
	unsigned char databuf[6];
	int hw_d[3] = { 0 };
	int temp = 0;
	int output[3]={ 0 };
	unsigned char rdy = 0;
	struct i2c_client *client = this_client;
	struct qmc6983_i2c_data *clientdata = i2c_get_clientdata(client);
	int i;

    MSE_FUN();

	/* Check status register for data availability */
	int t1 = 0;
	while(!(rdy & 0x07) && t1<3){
		msleep(2);
		databuf[0]=STA_REG_ONE;
		res=I2C_RxData(databuf,1);
		rdy=databuf[0];
		//MSE_LOG("QMC6983 Status register is (%02X)\n", rdy);
		t1 ++;
	}

	//MSE_LOG("QMC6983 read mag_xyz begin\n");

	mutex_lock(&read_i2c_xyz);

	databuf[0] = OUT_X_L;
	//res = I2C_RxData(mag_data, 6);/*only can read one by one*/
	if(res = I2C_RxData(databuf, 6)){
		mutex_unlock(&read_i2c_xyz);
		return -EFAULT;
	}
	for(i=0;i<6;i++)
		mag_data[i]=databuf[i];
	mutex_unlock(&read_i2c_xyz);


	hw_d[0] = (short) (((mag_data[1]) << 8) | mag_data[0]);
	hw_d[1] = (short) (((mag_data[3]) << 8) | mag_data[2]);
	hw_d[2] = (short) (((mag_data[5]) << 8) | mag_data[4]);


	hw_d[0] = hw_d[0] * 1000 / clientdata->xy_sensitivity;
	hw_d[1] = hw_d[1] * 1000 / clientdata->xy_sensitivity;
	hw_d[2] = hw_d[2] * 1000 / clientdata->z_sensitivity;

	output[clientdata->cvt.map[QMC6983_AXIS_X]] = clientdata->cvt.sign[QMC6983_AXIS_X]*hw_d[QMC6983_AXIS_X];
	output[clientdata->cvt.map[QMC6983_AXIS_Y]] = clientdata->cvt.sign[QMC6983_AXIS_Y]*hw_d[QMC6983_AXIS_Y];
	output[clientdata->cvt.map[QMC6983_AXIS_Z]] = clientdata->cvt.sign[QMC6983_AXIS_Z]*hw_d[QMC6983_AXIS_Z];

	data[0] = output[QMC6983_AXIS_X];
	data[1] = output[QMC6983_AXIS_Y];
	data[2] = output[QMC6983_AXIS_Z];

	return res;
}

static void qmc6983_work(struct work_struct *work)
{
	int ret;

	int data[3];
	int needRetry = 0;
	counter++;
	if(counter == FILTER_WINDOW){
		complete_all(&data_updated);
		init_completion(&data_updated);
		counter = 0;
	}

	ret = QMC6983_read_mag_xyz(data);

	schedule_delayed_work(&data_avg_work, msecs_to_jiffies(DATA_AVG_DELAY));
}

/* Set the Gain range */
int QMC6983_set_range(short range)
{
	int err = 0;
	unsigned char data[2];
	struct qmc6983_i2c_data *obj = i2c_get_clientdata(this_client);

	int ran ;
	switch (range) {
	case QMC6983_RNG_2G:
		ran = RNG_2G;
		break;
	case QMC6983_RNG_8G:
		ran = RNG_8G;
		break;
	case QMC6983_RNG_12G:
		ran = RNG_12G;
		break;
	case QMC6983_RNG_20G:
		ran = RNG_20G;
		break;
	default:
		return -EINVAL;
	}

	obj->xy_sensitivity = 16000/ran;//10000;
	obj->z_sensitivity = 16000/ran;//10000;

	data[0] = CTL_REG_ONE;
	err = I2C_RxData(data, 1);

	data[0] &= 0xcf;
	data[0] |= (range << 4);
	data[1] = data[0];

	data[0] = CTL_REG_ONE;
	//err = I2C_TxData(data, 2);
	return err;

}

/* Set the sensor mode */
int QMC6983_set_mode(char mode)
{
	int err = 0;
	unsigned char data[2];
	data[0] = CTL_REG_ONE;
	err = I2C_RxData(data, 1);

	data[0] &= 0xfc;
	data[0] |= mode;
	data[1] = data[0];

	data[0] = CTL_REG_ONE;
	MSE_LOG("QMC6983 in QMC6983_set_mode, data[1] = [%02x]", data[1]);
	err = I2C_TxData(data, 2);

	return err;
}

int QMC6983_set_ratio(char ratio)
{
	int err = 0;
	unsigned char data[2];
	data[0] = 0x0b;//RATIO_REG;
	data[1] = 0x01;//ratio;
	err = I2C_TxData(data, 2);
	return err;
}

int QMC6983_set_output_data_rate(char rate)
{
	int err = 0;
	unsigned char data[2];
	data[0] = CTL_REG_ONE;
	err = I2C_RxData(data, 1);

	data[0] &= 0xf3;
	data[0] |= (rate << 2);
	data[1] = data[0];

	data[0] = CTL_REG_ONE;
	return err;
}

int QMC6983_set_oversample_ratio(char ratio)
{
	int err = 0;
	unsigned char data[2];
	data[0] = CTL_REG_ONE;
	err = I2C_RxData(data, 1);

	data[0] &= 0x3f;
	data[0] |= (ratio << 6);
	data[1] = data[0];

	data[0] = CTL_REG_ONE;
	//err = I2C_TxData(data, 2);

	return err;
}

static void qmc6983_start_measure(struct i2c_client *client)
{

	unsigned char data[2];
	int err;

	data[1] = 0x1d;
	data[0] = CTL_REG_ONE;
	err = I2C_TxData(data, 2);

}

static void qmc6983_stop_measure(struct i2c_client *client)
{

	unsigned char data[2];
	int err;

	data[1] = 0x1c;
	data[0] = CTL_REG_ONE;
	err = I2C_TxData(data, 2);
}

static int qmc6983_enable(struct i2c_client *client)
{
	QMCDBG("start measure!\n");
	qmc6983_start_measure(client);

	QMC6983_set_range(QMC6983_RNG_20G);
	QMC6983_set_ratio(4);				//the ratio must not be 0, different with qmc5983


	return 0;
}

static int qmc6983_disable(struct i2c_client *client)
{
	QMCDBG("stop measure!\n");
	qmc6983_stop_measure(client);

	return 0;
}



/*----------------------------------------------------------------------------*/
static atomic_t dev_open_count;

static int qmc6983_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2];
	int res = 0;

	struct qmc6983_i2c_data *obj = i2c_get_clientdata(client);


	if(enable == TRUE)
	{
		if(qmc6983_enable(client))
		{
			printk("qmc6983: set power mode failed!\n");
			return -1;
		}
		else
		{
			printk("qmc6983: set power mode enable ok!\n");
		}
	}
	else
	{
		if(qmc6983_disable(client))
		{
			printk("qmc6983: set power mode failed!\n");
			return -1;
		}
		else
		{
			printk("qmc6983: set power mode disable ok!\n");
		}
	}

	return 0;
}

/*----------------------------------------------------------------------------*/
static void qmc6983_power(struct mag_hw *hw, unsigned int on)
{
	static unsigned int power_on = 0;

	if(hw->power_id != MT65XX_POWER_NONE)
	{
		if(power_on == on)
		{
			QMCDBG("ignore power control: %d\n", on);
		}
		else if(on)
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "qmc6983"))
			{
				printk(KERN_ERR "power on fails!!\n");
			}
		}
		else
		{
			if(!hwPowerDown(hw->power_id, "qmc6983"))
			{
				printk(KERN_ERR "power off fail!!\n");
			}
		}
	}
	power_on = on;
}

// Daemon application save the data
static int ECS_SaveData(int buf[12])
{
#if DEBUG
	struct i2c_client *client = this_client;
	struct qmc6983_i2c_data *data = i2c_get_clientdata(client);
#endif

	mutex_lock(&sensor_data_mutex);
	memcpy(sensor_data, buf, sizeof(sensor_data));
	mutex_unlock(&sensor_data_mutex);

#if DEBUG
		MSE_LOG("Get daemon data: %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d!\n",
			sensor_data[0],sensor_data[1],sensor_data[2],sensor_data[3],
			sensor_data[4],sensor_data[5],sensor_data[6],sensor_data[7],
			sensor_data[8],sensor_data[9],sensor_data[10],sensor_data[11]);
	//}
#endif

	return 0;

}

static int ECS_GetOpenStatus(void)
{
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) != 0));
	return atomic_read(&open_flag);
}


/*----------------------------------------------------------------------------*/
static int qmc6983_ReadChipInfo(char *buf, int bufsize)
{
	if((!buf)||(bufsize <= QMC6983_BUFSIZE -1))
	{
		return -1;
	}
	if(!this_client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "qmc6983 Chip");
	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	char strbuf[QMC6983_BUFSIZE];
	qmc6983_ReadChipInfo(strbuf, QMC6983_BUFSIZE);
	return sprintf(buf, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{

	int sensordata[3];
	char strbuf[QMC6983_BUFSIZE];

	QMC6983_read_mag_xyz(&sensordata);

	sprintf(strbuf, "%d %d %d\n", sensordata[0],sensordata[1],sensordata[2]);

	return sprintf(buf, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_posturedata_value(struct device_driver *ddri, char *buf)
{
	int tmp[3];
	char strbuf[QMC6983_BUFSIZE];
	tmp[0] = sensor_data[0] * CONVERT_O / CONVERT_O_DIV;
	tmp[1] = sensor_data[1] * CONVERT_O / CONVERT_O_DIV;
	tmp[2] = sensor_data[2] * CONVERT_O / CONVERT_O_DIV;
	sprintf(strbuf, "%d, %d, %d\n", tmp[0],tmp[1], tmp[2]);

	return sprintf(buf, "%s\n", strbuf);;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_layout_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = this_client;
	struct qmc6983_i2c_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "(%d, %d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
		data->hw->direction,atomic_read(&data->layout),	data->cvt.sign[0], data->cvt.sign[1],
		data->cvt.sign[2],data->cvt.map[0], data->cvt.map[1], data->cvt.map[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_layout_value(struct device_driver *ddri, char *buf, size_t count)
{
	struct i2c_client *client = this_client;
	struct qmc6983_i2c_data *data = i2c_get_clientdata(client);
	int layout = 0;

	if(1 == sscanf(buf, "%d", &layout))
	{
		atomic_set(&data->layout, layout);
		if(!hwmsen_get_convert(layout, &data->cvt))
		{
			printk(KERN_ERR "HWMSEN_GET_CONVERT function error!\r\n");
		}
		else if(!hwmsen_get_convert(data->hw->direction, &data->cvt))
		{
			printk(KERN_ERR "invalid layout: %d, restore to %d\n", layout, data->hw->direction);
		}
		else
		{
			printk(KERN_ERR "invalid layout: (%d, %d)\n", layout, data->hw->direction);
			hwmsen_get_convert(0, &data->cvt);
		}
	}
	else
	{
		printk(KERN_ERR "invalid format = '%s'\n", buf);
	}

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = this_client;
	struct qmc6983_i2c_data *data = i2c_get_clientdata(client);
	ssize_t len = 0;

	if(data->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n",
			data->hw->i2c_num, data->hw->direction, data->hw->power_id, data->hw->power_vol);
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}

	len += snprintf(buf+len, PAGE_SIZE-len, "OPEN: %d\n", atomic_read(&dev_open_count));
	return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct qmc6983_i2c_data *obj = i2c_get_clientdata(this_client);
	if(NULL == obj)
	{
		printk(KERN_ERR "qmc6983_i2c_data is null!!\n");
		return 0;
	}

	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));
	return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, char *buf, size_t count)
{
	struct qmc6983_i2c_data *obj = i2c_get_clientdata(this_client);
	int trace;
	if(NULL == obj)
	{
		printk(KERN_ERR "qmc6983_i2c_data is null!!\n");
		return 0;
	}

	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}
	else
	{
		printk(KERN_ERR "invalid content: '%s', length = %zu\n", buf, count);
	}

	return count;
}
static ssize_t show_daemon_name(struct device_driver *ddri, char *buf)
{
	char strbuf[QMC6983_BUFSIZE];
	sprintf(strbuf, "qmc6983d");
	return sprintf(buf, "%s", strbuf);
}
static ssize_t show_temperature_value(struct device_driver *ddri, char *buf)
{

	    int res;
		char strbuf[QMC6983_BUFSIZE];
		unsigned char mag_temperature[2];
		unsigned char databuf[2];
		int hw_temperature=0;
		unsigned char rdy = 0;
		struct i2c_client *client = this_client;
		struct qmc6983_i2c_data *clientdata = i2c_get_clientdata(client);
	
		MSE_FUN();
	
		mutex_lock(&read_i2c_temperature);
	
		databuf[0] = TEMP_L_REG;
		if(res = I2C_RxData(databuf, 1)){
			mutex_unlock(&read_i2c_temperature);
			return -EFAULT;
		}
		mag_temperature[0]=databuf[0];
	
		databuf[0] = TEMP_H_REG;
		if(res = I2C_RxData(databuf, 1)){
			mutex_unlock(&read_i2c_temperature);
			return -EFAULT;
		}
		mag_temperature[1]=databuf[0];
	
		mutex_unlock(&read_i2c_temperature);
		
	
		MSE_LOG("QMC6983 read_i2c_temperature[%02x, %02x]\n",
		mag_temperature[0], mag_temperature[1]);
	
		hw_temperature = ((mag_temperature[1]) << 8) | mag_temperature[0];

		MSE_LOG("QMC6983 temperature = %d\n",hw_temperature);  
	   
	   sprintf(strbuf, "temperature = %d\n", hw_temperature);
	   
	   return sprintf(buf, "%s\n", strbuf);
}
static ssize_t show_WRregisters_value(struct device_driver *ddri, char *buf)
{
        int res;
		char strbuf[QMC6983_BUFSIZE];
		unsigned char databuf[2];
		int hw_registers=0;
		unsigned char rdy = 0;
		struct i2c_client *client = this_client;
		struct qmc6983_i2c_data *clientdata = i2c_get_clientdata(client);
	
		MSE_FUN();
	  
	
		
		databuf[0] = regbuf[0];
		if(res = I2C_RxData(databuf, 1)){
			return -EFAULT;
		}
				
		MSE_LOG("QMC6983 hw_registers = 0x%02x\n",databuf[0]);  
	   
	    sprintf(strbuf, "hw_registers = 0x%02x\n", databuf[0]);
	   
	   return sprintf(buf, "%s\n", strbuf);
}
static ssize_t store_WRregisters_value(struct device_driver *ddri, char *buf, size_t count)
{
	    struct qmc6983_i2c_data *obj = i2c_get_clientdata(this_client);
	    unsigned char tempbuf[1] = {0};
		unsigned char data[2] = {0};
	    int err = 0;
		if(NULL == obj)
		{
			printk(KERN_ERR "qmc6983_i2c_data is null!!\n");
			return 0;
		}
		tempbuf[0] = *buf;
		printk(KERN_ERR "QMC6938:store_WRregisters_value: 0x%2x \n", tempbuf[0]);
	    data[1] = tempbuf[0];
	    data[0] = regbuf[0];
	    err = I2C_TxData(data, 2);
	    if(err != 0)
		   printk(KERN_ERR "QMC6938: write registers 0x%2x  ---> 0x%2x success! \n", regbuf[0],tempbuf[0]);

		return count;
}

static ssize_t show_registers_value(struct device_driver *ddri, char *buf)
{
       
		char strbuf[QMC6983_BUFSIZE];
				
	    MSE_LOG("QMC6983 hw_registers = 0x%02x\n",regbuf[0]);  
	   
	    sprintf(strbuf, "hw_registers = 0x%02x\n", regbuf[0]);
	   
	    return sprintf(buf, "%s\n", strbuf);
}
static ssize_t store_registers_value(struct device_driver *ddri, char *buf, size_t count)
{
	    struct qmc6983_i2c_data *obj = i2c_get_clientdata(this_client);
	
		if(NULL == obj)
		{
			printk(KERN_ERR "qmc6983_i2c_data is null!!\n");
			return 0;
		}
		regbuf[0] = *buf;
		printk(KERN_ERR "QMC6938: REGISTERS = 0x%2x\n", regbuf[0]);
		return count;

}
static ssize_t show_dumpallreg_value(struct device_driver *ddri, char *buf)
{
       
		 int res;
		 int i =0;
		char strbuf[300];
		char tempstrbuf[24];
		unsigned char databuf[2];
		int length=0;
		unsigned char rdy = 0;
		struct i2c_client *client = this_client;
		struct qmc6983_i2c_data *clientdata = i2c_get_clientdata(client);
	
		MSE_FUN();
	  
		/* Check status register for data availability */	
		for(i =0;i<12;i++)
		{
		      
		       databuf[0] = i;
		       res = I2C_RxData(databuf, 1);
			   if(res < 0)
			   	 MSE_LOG("QMC6983 dump registers 0x%02x failed !\n", i);

			   length = sprintf(tempstrbuf, "reg[0x%2x] =  0x%2x \n",i, databuf[0]);
			   sprintf(strbuf+length*i, "  %s \n",tempstrbuf);
		}
	   
	   return sprintf(buf, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(dumpallreg,  S_IRUGO , show_dumpallreg_value, NULL);
static DRIVER_ATTR(WRregisters, S_IRUGO | S_IWUGO, show_WRregisters_value, store_WRregisters_value);
static DRIVER_ATTR(registers,   S_IRUGO | S_IWUGO, show_registers_value, store_registers_value);
static DRIVER_ATTR(temperature, S_IRUGO, show_temperature_value, NULL);
static DRIVER_ATTR(daemon,      S_IRUGO, show_daemon_name, NULL);
static DRIVER_ATTR(chipinfo,    S_IRUGO, show_chipinfo_value, NULL);
static DRIVER_ATTR(sensordata,  S_IRUGO, show_sensordata_value, NULL);
static DRIVER_ATTR(posturedata, S_IRUGO, show_posturedata_value, NULL);
static DRIVER_ATTR(layout,      S_IRUGO | S_IWUSR, show_layout_value, store_layout_value);
static DRIVER_ATTR(status,      S_IRUGO, show_status_value, NULL);
static DRIVER_ATTR(trace,       S_IRUGO | S_IWUSR, show_trace_value, store_trace_value);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *qmc6983_attr_list[] = {
	&driver_attr_dumpallreg,
    &driver_attr_WRregisters,
	&driver_attr_registers,
    &driver_attr_temperature,
    &driver_attr_daemon,
	&driver_attr_chipinfo,
	&driver_attr_sensordata,
	&driver_attr_posturedata,
	&driver_attr_layout,
	&driver_attr_status,
	&driver_attr_trace,
};
/*----------------------------------------------------------------------------*/
static int qmc6983_create_attr(struct device_driver *driver)
{
	int idx, err = 0;
	int num = (int)(sizeof(qmc6983_attr_list)/sizeof(qmc6983_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(err = driver_create_file(driver, qmc6983_attr_list[idx]))
		{
			printk(KERN_ERR "driver_create_file (%s) = %d\n", qmc6983_attr_list[idx]->attr.name, err);
			break;
		}
	}

	return err;
}
/*----------------------------------------------------------------------------*/
static int qmc6983_delete_attr(struct device_driver *driver)
{
	int idx;
	int num = (int)(sizeof(qmc6983_attr_list)/sizeof(qmc6983_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}


	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, qmc6983_attr_list[idx]);
	}

	return 0;
}


/*----------------------------------------------------------------------------*/
static int qmc6983_open(struct inode *inode, struct file *file)
{
	struct qmc6983_i2c_data *obj = i2c_get_clientdata(this_client);
	int ret = -1;

	if(atomic_read(&obj->trace) & QMC_CTR_DEBUG)
	{
		QMCDBG("Open device node:qmc6983\n");
	}
	ret = nonseekable_open(inode, file);

	return ret;
}
/*----------------------------------------------------------------------------*/
static int qmc6983_release(struct inode *inode, struct file *file)
{
	struct qmc6983_i2c_data *obj = i2c_get_clientdata(this_client);
	atomic_dec(&dev_open_count);
	if(atomic_read(&obj->trace) & QMC_CTR_DEBUG)
	{
		QMCDBG("Release device node:qmc6983\n");
	}
	return 0;
}

#ifdef CONFIG_COMPAT
static int qmc6983_compat_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
    long ret =0;

    void __user *arg32 = compat_ptr(arg);

    if (!file->f_op || !file->f_op->unlocked_ioctl)
        return -ENOTTY;
    switch (cmd) {
         case COMPAT_QMC6983_SET_RANGE:
             //printk("akm8963_compat_ioctl COMPAT_ECS_IOCTL_WRITE\n");
             if(arg32 == NULL)
             {
                 QMCDBG("invalid argument.");
                 return -EINVAL;
             }

             ret = file->f_op->unlocked_ioctl(file, COMPAT_QMC6983_SET_RANGE,
                            (unsigned long)arg32);
             if (ret){
                QMCDBG("ECS_IOCTL_WRITE unlocked_ioctl failed.");
                return ret;
             }

             break;
         case COMPAT_QMC6983_SET_BANDWIDTH:
             //printk("akm8963_compat_ioctl COMPAT_ECS_IOCTL_WRITE\n");
             if(arg32 == NULL)
             {
                 QMCDBG("invalid argument.");
                 return -EINVAL;
             }

             ret = file->f_op->unlocked_ioctl(file, COMPAT_QMC6983_SET_BANDWIDTH,
                            (unsigned long)arg32);
             if (ret){
                QMCDBG("ECS_IOCTL_WRITE unlocked_ioctl failed.");
                return ret;
             }

             break;
         case COMPAT_QMC6983_SET_MODE:
             //printk("akm8963_compat_ioctl COMPAT_ECS_IOCTL_WRITE\n");
             if(arg32 == NULL)
             {
                 QMCDBG("invalid argument.");
                 return -EINVAL;
             }

             ret = file->f_op->unlocked_ioctl(file, COMPAT_QMC6983_SET_MODE,
                            (unsigned long)arg32);
             if (ret){
                QMCDBG("ECS_IOCTL_WRITE unlocked_ioctl failed.");
                return ret;
             }

             break;
         case COMPAT_QMC6983_READ_MAGN_XYZ:
             //printk("akm8963_compat_ioctl COMPAT_ECS_IOCTL_WRITE\n");
             if(arg32 == NULL)
             {
                 QMCDBG("invalid argument.");
                 return -EINVAL;
             }

             ret = file->f_op->unlocked_ioctl(file, COMPAT_QMC6983_READ_MAGN_XYZ,
                            (unsigned long)arg32);
             if (ret){
                QMCDBG("ECS_IOCTL_WRITE unlocked_ioctl failed.");
                return ret;
             }

             break;
         case COMPAT_QMC6983_SET_REGISTER_A:
             //printk("akm8963_compat_ioctl COMPAT_ECS_IOCTL_WRITE\n");
             if(arg32 == NULL)
             {
                 QMCDBG("invalid argument.");
                 return -EINVAL;
             }

             ret = file->f_op->unlocked_ioctl(file, COMPAT_QMC6983_SET_REGISTER_A,
                            (unsigned long)arg32);
             if (ret){
                QMCDBG("ECS_IOCTL_WRITE unlocked_ioctl failed.");
                return ret;
             }

             break;
         case COMPAT_QMC6983_SELF_TEST:
             //printk("akm8963_compat_ioctl COMPAT_ECS_IOCTL_WRITE\n");
             if(arg32 == NULL)
             {
                 QMCDBG("invalid argument.");
                 return -EINVAL;
             }

             ret = file->f_op->unlocked_ioctl(file, COMPAT_QMC6983_SELF_TEST,
                            (unsigned long)arg32);
             if (ret){
                QMCDBG("ECS_IOCTL_WRITE unlocked_ioctl failed.");
                return ret;
             }

             break;
         case COMPAT_ECS_IOCTL_SET_YPR:
             //printk("akm8963_compat_ioctl COMPAT_ECS_IOCTL_WRITE\n");
             if(arg32 == NULL)
             {
                 QMCDBG("invalid argument.");
                 return -EINVAL;
             }

             ret = file->f_op->unlocked_ioctl(file, COMPAT_ECS_IOCTL_SET_YPR,
                            (unsigned long)arg32);
             if (ret){
                QMCDBG("ECS_IOCTL_WRITE unlocked_ioctl failed.");
                return ret;
             }

             break;
         case COMPAT_ECS_IOCTL_GET_OPEN_STATUS:
             //printk("akm8963_compat_ioctl COMPAT_ECS_IOCTL_SET_MODE\n");
             if(arg32 == NULL)
             {
                 QMCDBG("invalid argument.");
                 return -EINVAL;
             }

             ret = file->f_op->unlocked_ioctl(file, COMPAT_ECS_IOCTL_GET_OPEN_STATUS,
                            (unsigned long)(arg32));
             if (ret){
                QMCDBG("ECS_IOCTL_SET_MODE unlocked_ioctl failed.");
                return ret;
             }
             break;
         case COMPAT_ECOMPASS_IOC_GET_MFLAG:
             //printk("akm8963_compat_ioctl COMPAT_ECS_IOCTL_SET_MODE\n");
             if(arg32 == NULL)
             {
                 QMCDBG("invalid argument.");
                 return -EINVAL;
             }

             ret = file->f_op->unlocked_ioctl(file, COMPAT_ECOMPASS_IOC_GET_MFLAG,
                            (unsigned long)(arg32));
             if (ret){
                QMCDBG("ECS_IOCTL_SET_MODE unlocked_ioctl failed.");
                return ret;
             }         
             break;  
         case COMPAT_ECOMPASS_IOC_GET_OFLAG:
             //printk("akm8963_compat_ioctl COMPAT_ECS_IOCTL_SET_MODE\n");
             if(arg32 == NULL)
             {
                 QMCDBG("invalid argument.");
                 return -EINVAL;
             }

             ret = file->f_op->unlocked_ioctl(file, COMPAT_ECOMPASS_IOC_GET_OFLAG,
                            (unsigned long)(arg32));
             if (ret){
                QMCDBG("ECS_IOCTL_SET_MODE unlocked_ioctl failed.");
                return ret;
             }         
             break; 
         case COMPAT_ECS_IOCTL_GET_DELAY:
             //printk("akm8963_compat_ioctl COMPAT_ECS_IOCTL_SET_MODE\n");
             if(arg32 == NULL)
             {
                 QMCDBG("invalid argument.");
                 return -EINVAL;
             }

             ret = file->f_op->unlocked_ioctl(file, COMPAT_ECS_IOCTL_GET_DELAY,
                            (unsigned long)(arg32));
             if (ret){
                QMCDBG("ECS_IOCTL_SET_MODE unlocked_ioctl failed.");
                return ret;
             }         
             break; 
         case COMPAT_MSENSOR_IOCTL_READ_CHIPINFO:
             //printk("akm8963_compat_ioctl COMPAT_ECS_IOCTL_SET_MODE\n");
             if(arg32 == NULL)
             {
                 QMCDBG("invalid argument.");
                 return -EINVAL;
             }

             ret = file->f_op->unlocked_ioctl(file, COMPAT_MSENSOR_IOCTL_READ_CHIPINFO,
                            (unsigned long)(arg32));
             if (ret){
                QMCDBG("ECS_IOCTL_SET_MODE unlocked_ioctl failed.");
                return ret;
             }         
             break; 
         case COMPAT_MSENSOR_IOCTL_READ_SENSORDATA:
             //printk("akm8963_compat_ioctl COMPAT_ECS_IOCTL_SET_MODE\n");
             if(arg32 == NULL)
             {
                 QMCDBG("invalid argument.");
                 return -EINVAL;
             }

             ret = file->f_op->unlocked_ioctl(file, COMPAT_MSENSOR_IOCTL_READ_SENSORDATA,
                            (unsigned long)(arg32));
             if (ret){
                QMCDBG("ECS_IOCTL_SET_MODE unlocked_ioctl failed.");
                return ret;
             }
             break;
         case COMPAT_ECOMPASS_IOC_GET_LAYOUT:
             //printk("akm8963_compat_ioctl COMPAT_ECS_IOCTL_SET_MODE\n");
             if(arg32 == NULL)
             {
                 QMCDBG("invalid argument.");
                 return -EINVAL;
             }

             ret = file->f_op->unlocked_ioctl(file, COMPAT_ECOMPASS_IOC_GET_LAYOUT,
                            (unsigned long)(arg32));
             if (ret){
                QMCDBG("ECS_IOCTL_SET_MODE unlocked_ioctl failed.");
                return ret;
             }
             break;
         case COMPAT_MSENSOR_IOCTL_SENSOR_ENABLE:
             //printk("akm8963_compat_ioctl COMPAT_ECS_IOCTL_SET_MODE\n");
             if(arg32 == NULL)
             {
                 QMCDBG("invalid argument.");
                 return -EINVAL;
             }

             ret = file->f_op->unlocked_ioctl(file, COMPAT_MSENSOR_IOCTL_SENSOR_ENABLE,
                            (unsigned long)(arg32));
             if (ret){
                QMCDBG("ECS_IOCTL_SET_MODE unlocked_ioctl failed.");
                return ret;
             }
             break;
         case COMPAT_MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:
             //printk("akm8963_compat_ioctl COMPAT_ECS_IOCTL_SET_MODE\n");
             if(arg32 == NULL)
             {
                 QMCDBG("invalid argument.");
                 return -EINVAL;
             }

             ret = file->f_op->unlocked_ioctl(file, COMPAT_MSENSOR_IOCTL_READ_FACTORY_SENSORDATA,
                            (unsigned long)(arg32));
             if (ret){
                QMCDBG("ECS_IOCTL_SET_MODE unlocked_ioctl failed.");
                return ret;
             }
             break;
         default:
             printk(KERN_ERR "%s not supported = 0x%04x", __func__, cmd);
             return -ENOIOCTLCMD;
             break;
    }
    return ret;
}
#endif

/*----------------------------------------------------------------------------*/
static int qmc6983_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	/* NOTE: In this function the size of "char" should be 1-byte. */
	char buff[QMC6983_BUFSIZE];				/* for chip information */

	int value[12];			/* for SET_YPR */
	int delay;				/* for GET_DELAY */
	int status; 				/* for OPEN/CLOSE_STATUS */
	short sensor_status;		/* for Orientation and Msensor status */
	unsigned char data[16] = {0};
	int vec[3] = {0};
	struct i2c_client *client = this_client;
	struct qmc6983_i2c_data *clientdata = i2c_get_clientdata(client);
	hwm_sensor_data osensor_data;
	uint32_t enable;

	short sbuf[3];
	char cmode = 0;
	int err;

	switch (cmd)
	{

	case QMC6983_SET_RANGE:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		err = QMC6983_set_range(*data);
		return err;

	case QMC6983_SET_BANDWIDTH:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
//		err = QMC6983_set_bandwidth(*data);
		return err;

	case QMC6983_SET_MODE:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
//		err = QMC6983_set_mode(*data);
		return err;

	case QMC6983_READ_MAGN_XYZ:
		if(argp == NULL){
			printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
			break;
		}
		err = QMC6983_read_mag_xyz(vec);
		printk(KERN_INFO "mag_data[%d, %d, %d]\n",
				vec[0],vec[1],vec[2]);
			if(copy_to_user(argp, vec, sizeof(vec)))
			{
				return -EFAULT;
			}
			break;

	case QMC6983_SET_REGISTER_A: /* New QMC6983 operation */
		if (copy_from_user(data, (unsigned char *)arg, 3) != 0) {
			#if DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
//		err = QMC6983_set_registerA(data[0],data[1],data[2]); /* char avg,char rate,char mode */
		return err;
	case QMC6983_SELF_TEST: /* New QMC6983 operation */
			if (copy_from_user(data, (unsigned char *)arg, 1) != 0) {
			#if DEBUG
			printk(KERN_ERR "copy_from_user error\n");
			#endif
			return -EFAULT;
		}
		cmode = data[0];
		//err = QMC6983_self_test(cmode, sbuf);
		if (copy_to_user(argp,sbuf,3) != 0)
		{
			#if DEBUG
			printk(KERN_ERR "copy_to error\n");
			#endif
			return -EFAULT;
		}
		return err;

/*------------------------------for daemon------------------------*/
		case ECS_IOCTL_SET_YPR:
			if(argp == NULL)
			{
				QMCDBG("invalid argument.");
				return -EINVAL;
			}
			if(copy_from_user(value, argp, sizeof(value)))
			{
				QMCDBG("copy_from_user failed.");
				return -EFAULT;
			}
			ECS_SaveData(value);
			break;

		case ECS_IOCTL_GET_OPEN_STATUS:
			status = ECS_GetOpenStatus();
			if(copy_to_user(argp, &status, sizeof(status)))
			{
				QMCDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;

		case ECOMPASS_IOC_GET_MFLAG:
			sensor_status = atomic_read(&m_flag);
			if(copy_to_user(argp, &sensor_status, sizeof(sensor_status)))
			{
				QMCDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;

		case ECOMPASS_IOC_GET_OFLAG:
			sensor_status = atomic_read(&o_flag);
			if(copy_to_user(argp, &sensor_status, sizeof(sensor_status)))
			{
				QMCDBG("copy_to_user failed.");
				return -EFAULT;
			}
			break;
		case ECS_IOCTL_GET_DELAY:
            delay = qmcd_delay;
            if (copy_to_user(argp, &delay, sizeof(delay))) {
                 QMCDBG("copy_to_user failed.");
                 return -EFAULT;
            }
            break;
/*------------------------------for ftm------------------------*/

		case MSENSOR_IOCTL_READ_CHIPINFO:       //reserved?
			if(argp == NULL)
			{
				printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
				break;
			}

			qmc6983_ReadChipInfo(buff, QMC6983_BUFSIZE);
			if(copy_to_user(argp, buff, strlen(buff)+1))
			{
				return -EFAULT;
			}
			break;

		case MSENSOR_IOCTL_READ_SENSORDATA:	//for daemon
			if(argp == NULL)
			{
				MSE_LOG("IO parameter pointer is NULL!\r\n");
				break;
			}

			QMC6983_read_mag_xyz(vec);

			MSE_LOG("mag_data[%d, %d, %d]\n",
					vec[0],vec[1],vec[2]);
			sprintf(buff, "%x %x %x", vec[0], vec[1], vec[2]);
			if(copy_to_user(argp, buff, strlen(buff)+1))
			{
				return -EFAULT;
			}

				break;


		case ECOMPASS_IOC_GET_LAYOUT:   //not use
			status = atomic_read(&clientdata->layout);
			if(copy_to_user(argp, &status, sizeof(status)))
			{
				MSE_LOG("copy_to_user failed.");
				return -EFAULT;
			}
			break;

		case MSENSOR_IOCTL_SENSOR_ENABLE:

			if(argp == NULL)
			{
				printk(KERN_ERR "IO parameter pointer is NULL!\r\n");
				break;
			}
			if(copy_from_user(&enable, argp, sizeof(enable)))
			{
				QMCDBG("copy_from_user failed.");
				return -EFAULT;
			}
			else
			{
			    printk( "MSENSOR_IOCTL_SENSOR_ENABLE enable=%d!\r\n",enable);
				if(1 == enable)
				{
					atomic_set(&o_flag, 1);
					atomic_set(&open_flag, 1);
				}
				else
				{
					atomic_set(&o_flag, 0);
					if(atomic_read(&m_flag) == 0)
					{
						atomic_set(&open_flag, 0);
					}
				}
				wake_up(&open_wq);

			}

			break;

		case MSENSOR_IOCTL_READ_FACTORY_SENSORDATA:
			if(argp == NULL)
			{
				MSE_ERR( "IO parameter pointer is NULL!\r\n");
				break;
			}

			mutex_lock(&sensor_data_mutex);

			osensor_data.values[0] = sensor_data[8];
			osensor_data.values[1] = sensor_data[9];
			osensor_data.values[2] = sensor_data[10];
			osensor_data.status = sensor_data[11];
			osensor_data.value_divide = CONVERT_O_DIV;

			mutex_unlock(&sensor_data_mutex);

            if(copy_to_user(argp, &osensor_data, sizeof(hwm_sensor_data)))
			{
				return -EFAULT;
			}

			break;

		default:
			printk(KERN_ERR "%s not supported = 0x%04x", __func__, cmd);
			return -ENOIOCTLCMD;
			break;
		}

	return 0;
}
/*----------------------------------------------------------------------------*/
static struct file_operations qmc6983_fops = {
//	.owner = THIS_MODULE,
	.open = qmc6983_open,
	.release = qmc6983_release,
	.unlocked_ioctl = qmc6983_unlocked_ioctl,
    #ifdef CONFIG_COMPAT
    .compat_ioctl = qmc6983_compat_ioctl,
    #endif
};
/*----------------------------------------------------------------------------*/
static struct miscdevice qmc6983_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "msensor",
    .fops = &qmc6983_fops,
};
/*----------------------------------------------------------------------------*/
int qmc6983_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* msensor_data;

#if DEBUG
	struct i2c_client *client = this_client;
	struct qmc6983_i2c_data *data = i2c_get_clientdata(client);
#endif

#if DEBUG
	if(atomic_read(&data->trace) & QMC_FUN_DEBUG)
	{
		QMCFUNC("qmc6983_operate");
	}
#endif
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR "Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value <= 20)
				{
					qmcd_delay = 20;
				}
				qmcd_delay = 20;//value;
			}
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR "Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{

				value = *(int *)buff_in;
				if(value == 1)
				{
					atomic_set(&m_flag, 1);
					atomic_set(&open_flag, 1);
				}
				else
				{
					atomic_set(&m_flag, 0);
					if(atomic_read(&o_flag) == 0)
					{
						atomic_set(&open_flag, 0);
					}
				}
				wake_up(&open_wq);

				// TODO: turn device into standby or normal mode
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				printk(KERN_ERR "get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				msensor_data = (hwm_sensor_data *)buff_out;
				mutex_lock(&sensor_data_mutex);

				msensor_data->values[0] = sensor_data[4] * CONVERT_M;
				msensor_data->values[1] = sensor_data[5] * CONVERT_M;
				msensor_data->values[2] = sensor_data[6] * CONVERT_M;
				msensor_data->status = sensor_data[7];
				msensor_data->value_divide = CONVERT_M_DIV;

				mutex_unlock(&sensor_data_mutex);
#if DEBUG
				if(atomic_read(&data->trace) & QMC_HWM_DEBUG)
				{
					QMCDBG("Hwm get m-sensor data: %d, %d, %d. divide %d, status %d!\n",
						msensor_data->values[0],msensor_data->values[1],msensor_data->values[2],
						msensor_data->value_divide,msensor_data->status);
				}
#endif
			}
			break;
		default:
			printk(KERN_ERR "msensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}

/*----------------------------------------------------------------------------*/
int qmc6983_orientation_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data* osensor_data;
#if DEBUG
	struct i2c_client *client = this_client;
	struct qmc6983_i2c_data *data = i2c_get_clientdata(client);
#endif

#if DEBUG
	if(atomic_read(&data->trace) & QMC_FUN_DEBUG)
	{
		QMCFUNC("qmc6983_orientation_operate");
	}
#endif

	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR "Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value <= 20)
				{
					qmcd_delay = 20;
				}
				qmcd_delay = value;
			}
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				printk(KERN_ERR "Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{

				value = *(int *)buff_in;
				if(value == 1)
				{
					atomic_set(&o_flag, 1);
					atomic_set(&open_flag, 1);
				}
				else
				{
					atomic_set(&o_flag, 0);
					if(atomic_read(&m_flag) == 0)
					{
						atomic_set(&open_flag, 0);
					}
				}
				wake_up(&open_wq);
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				printk(KERN_ERR "get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				osensor_data = (hwm_sensor_data *)buff_out;
				mutex_lock(&sensor_data_mutex);

				osensor_data->values[0] = sensor_data[8] * CONVERT_O;
				osensor_data->values[1] = sensor_data[9] * CONVERT_O;
				osensor_data->values[2] = sensor_data[10] * CONVERT_O;
				osensor_data->status = sensor_data[11];
				osensor_data->value_divide = CONVERT_O_DIV;

				mutex_unlock(&sensor_data_mutex);
#if DEBUG
			//if(atomic_read(&data->trace) & QMC_HWM_DEBUG)
			//{	QMCDBG
				MSE_LOG("Hwm get o-sensor data: %d, %d, %d. divide %d, status %d!\n",
					osensor_data->values[0],osensor_data->values[1],osensor_data->values[2],
					osensor_data->value_divide,osensor_data->status);
			//}
#endif
			}
			break;
		default:
			printk(KERN_ERR "gsensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}

	return err;
}

/*----------------------------------------------------------------------------*/
#ifndef	CONFIG_HAS_EARLYSUSPEND
/*----------------------------------------------------------------------------*/
static int qmc6983_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct qmc6983_i2c_data *obj = i2c_get_clientdata(client);

	if(msg.event == PM_EVENT_SUSPEND)
	{
		qmc6983_power(obj->hw, 0);
	}
	return 0;
}
/*----------------------------------------------------------------------------*/
static int qmc6983_resume(struct i2c_client *client)
{
	struct qmc6983_i2c_data *obj = i2c_get_clientdata(client);

	qmc6983_power(obj->hw, 1);

	return 0;
}
/*----------------------------------------------------------------------------*/
#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/
static void qmc6983_early_suspend(struct early_suspend *h)
{
	struct qmc6983_i2c_data *obj = container_of(h, struct qmc6983_i2c_data, early_drv);

	if(NULL == obj)
	{
		printk(KERN_ERR "null pointer!!\n");
		return;
	}
	if(qmc6983_SetPowerMode(obj->client, false))
	{
		printk("qmc6983: write power control fail!!\n");
		return;
	}

}
/*----------------------------------------------------------------------------*/
static void qmc6983_late_resume(struct early_suspend *h)
{
	struct qmc6983_i2c_data *obj = container_of(h, struct qmc6983_i2c_data, early_drv);


	if(NULL == obj)
	{
		printk(KERN_ERR "null pointer!!\n");
		return;
	}

	if(qmc6983_SetPowerMode(obj->client, true))
	{
		printk("qmc6983: write power control fail!!\n");
		return;
	}

}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/

/*----------------------------------------------------------------------------*/
static int qmc6983_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	MSE_FUN();
	struct i2c_client *new_client;
	struct qmc6983_i2c_data *data;
	char tmp[2];
	int err = 0;
	unsigned char databuf[6] = {0,0,0,0,0,0};
	struct hwmsen_object sobj_m, sobj_o;

	int vec[3]={0,0,0};

	int i=0;

	MSE_LOG("qmc6983 i2c probe 1\n");
	if(!(data = kmalloc(sizeof(struct qmc6983_i2c_data), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(data, 0, sizeof(struct qmc6983_i2c_data));

	data->hw = hw;

	printk(KERN_ALERT "QMC6983_mod addr after,addr = %d\n",client->addr);

	if (hwmsen_get_convert(data->hw->direction, &data->cvt)) {
        	printk(KERN_ERR "QMC6983 invalid direction: %d\n", data->hw->direction);
        	goto exit_kfree;
    	}

	atomic_set(&data->layout, data->hw->direction);
	atomic_set(&data->trace, 0);

	MSE_LOG("qmc6983 i2c probe 2\n");

	mutex_init(&sensor_data_mutex);
	mutex_init(&read_i2c_xyz);
	mutex_init(&read_i2c_temperature);
	mutex_init(&read_i2c_register);

	init_waitqueue_head(&data_ready_wq);
	init_waitqueue_head(&open_wq);

	data->client = client;
	new_client = data->client;
	i2c_set_clientdata(new_client, data);

	this_client = new_client;
	//this_client->timing=400;

#if QMC6983_AD0_CMP
	MSE_LOG("qmc6983 i2c probe 2\n");

	unsigned short int temp = 0;
	int dummy = 0;

	temp = (client->addr) >> 7;
	temp = temp << 7;
	client->addr = temp | 0x002d;
	MSE_LOG("QMC6983_probe 2 - 1, Addr = [%u]\n", temp);
	dummy = send_data(client, 0x18, 0x80);
	if (dummy) {
		printk(KERN_INFO "i2c write error, [Addr=%x]\n", client->addr);
	}

	temp = (client->addr) >> 7;
	temp = temp << 7;
	client->addr = temp | 0x002c;
	MSE_LOG("QMC6983_probe 2 - 2, Addr = [%u]\n", temp);
	dummy = send_data(client, 0x18, 0x80);
	if (dummy) {
		printk(KERN_INFO "i2c write error, [Addr=%x]\n", client->addr);
	}

	MSE_LOG("qmc6983 i2c probe 3\n");

#endif

	MSE_LOG("qmc6983 i2c probe 3\n");
#if 1
	/* read chip id */
	databuf[0] = 0x0d;
	if(I2C_RxData(databuf, 1)<0){
		MSE_ERR("QMC6983 I2C_RxData error!\n");
		goto exit_i2c_failed;
	}
	if (databuf[0] == 0xff) {
		MSE_LOG("QMC6983 I2C driver registered!\n");
	} else {
		MSE_LOG("QMC6983 check ID faild!\n");
		goto exit_i2c_failed;
	}
	MSE_LOG("QMC6983  i2c probe 4\n");
#endif

	INIT_DELAYED_WORK(&data_avg_work, qmc6983_work);

	qmc6983_SetPowerMode(new_client, true);

	MSE_LOG("qmc6983 i2c probe 5\n");

	/* Register sysfs attribute */
	//if(err = qmc6983_create_attr(&qmc_sensor_driver.driver))
	if(err = qmc6983_create_attr(&qmc6983_init_info.platform_diver_addr->driver))
	{
		printk(KERN_ERR "create attribute err = %d\n", err);
		goto exit_sysfs_create_group_failed;
	}


	if(err = misc_register(&qmc6983_device))
	{
		printk(KERN_ERR "qmc6983_device register failed\n");
		goto exit_misc_device_register_failed;	}

	sobj_m.self = data;
	sobj_m.polling = 1;
	sobj_m.sensor_operate = qmc6983_operate;

	if(err = hwmsen_attach(ID_MAGNETIC, &sobj_m))
	{
		printk(KERN_ERR "attach fail = %d\n", err);
		goto exit_kfree;
	}

	sobj_o.self = data;
	sobj_o.polling = 1;
	sobj_o.sensor_operate = qmc6983_orientation_operate;

	if(err = hwmsen_attach(ID_ORIENTATION, &sobj_o))
	{
		printk(KERN_ERR "attach fail = %d\n", err);
		goto exit_kfree;
	}


	init_completion(&data_updated);

//#if CONFIG_HAS_EARLYSUSPEND
#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_EARLYSUSPEND) 
	data->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	data->early_drv.suspend  = qmc6983_early_suspend,
	data->early_drv.resume   = qmc6983_late_resume,
	register_early_suspend(&data->early_drv);
#endif
    qmc6983_init_flag = 0;
	QMCDBG("%s: OK\n", __func__);
	return 0;

	exit_i2c_failed:
	exit_sysfs_create_group_failed:
	exit_misc_device_register_failed:
	exit_kfree:
	kfree(data);
	exit:
	printk(KERN_ERR "%s: err = %d\n", __func__, err);
    qmc6983_init_flag = -1;
	return err;
}
/*----------------------------------------------------------------------------*/
static int qmc6983_i2c_remove(struct i2c_client *client)
{
	int err;

	if(err = qmc6983_delete_attr(&qmc6983_init_info.platform_diver_addr->driver))
	{
		printk(KERN_ERR "qmc6983_delete_attr fail: %d\n", err);
	}

	this_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	misc_deregister(&qmc6983_device);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int qmc6983_local_init(void)
{
	qmc6983_power(hw, 1);
	MSE_FUN();
	atomic_set(&dev_open_count, 0);
	//qmc6983_force[0] = hw->i2c_num;

	if(i2c_add_driver(&qmc6983_i2c_driver))
	{
		printk(KERN_ERR "add driver error\n");
		return -1;
	}
    if(-1 == qmc6983_init_flag)
    {
      QMCDBG("%s: FAIL\n", __func__);
      return -1;
    }
	return 0;
}
/*----------------------------------------------------------------------------*/
static int qmc6983_remove(void)
{
	qmc6983_power(hw, 0);
	atomic_set(&dev_open_count, 0);
	i2c_del_driver(&qmc6983_i2c_driver);
	QMCDBG("%s: OK\n", __func__);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int __init qmc6983_init(void)
{
    const char *name = "mediatek,qmc6983";
    hw =    get_mag_dts_func(name, hw);
    if (!hw)
    hw = qmc6983_get_cust_mag_hw();

    struct i2c_board_info i2c_qmc6983={ I2C_BOARD_INFO("qmc6983", hw->i2c_addr[0])};

	i2c_register_board_info(hw->i2c_num, &i2c_qmc6983, 1);
    mag_driver_add(&qmc6983_init_info);
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit qmc6983_exit(void)
{
    //platform_driver_unregister(&qmc_sensor_driver);
}
/*----------------------------------------------------------------------------*/
module_init(qmc6983_init);
module_exit(qmc6983_exit);

MODULE_AUTHOR("QST Corp");
MODULE_DESCRIPTION("qmc6983 compass driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
