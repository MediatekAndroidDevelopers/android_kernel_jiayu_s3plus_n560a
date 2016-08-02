/* drivers/hwmon/mt6516/amit/epl259x.c - EPL259x ALS/PS driver
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

#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include <asm/io.h>
#include <cust_eint.h>
#include <cust_alsps.h>

#include <linux/hwmsen_helper.h>
#include "epl259x.h"
#include <linux/input/mt.h>

#ifdef CONFIG_ARCH_MT6753
#define MT6753 1
#endif

#if defined(MT6575) || defined(MT6571) || defined(MT6589) || defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
#include <mach/devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif


//add for fix resume issue
#include <linux/earlysuspend.h>
#include <linux/wakelock.h>
//add for fix resume issue end

#define MTK_LTE         1

/*  set   debug_mode */
static int debug_mode_flag = 0;

#if MTK_LTE
#include <alsps.h>
#endif
/******************************************************************************
 * extern functions
*******************************************************************************/
#if defined(MT6575) || defined(MT6571) || defined(MT6589)
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);
#endif

#if defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
extern void mt_eint_unmask(unsigned int line);
extern void mt_eint_mask(unsigned int line);
extern void mt_eint_set_polarity(kal_uint8 eintno, kal_bool ACT_Polarity);
extern void mt_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt_eint_set_sens(kal_uint8 eintno, kal_bool sens);
//extern void mt_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
//                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
//                                     kal_bool auto_umask);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif


/*-------------------------MT6516&MT6575 define-------------------------------*/
#define POWER_NONE_MACRO MT65XX_POWER_NONE



/******************************************************************************
 *  configuration
 ******************************************************************************/
#define LUX_PER_COUNT			1100//700		// 0.7

#define COMMON_DEBUG    1
#define ALS_DEBUG       1
#define PS_DEBUG        1
#define SHOW_DBG        1

#define PS_GES          0
#define PS_DYN_K        1
#define PS_DYN_K_STR    0
#define ALS_DYN_INTT    1
#define HS_ENABLE       0

/******************************************************************************
*******************************************************************************/

#define TXBYTES 				2
#define RXBYTES					2
#define PACKAGE_SIZE 			48
#define I2C_RETRY_COUNT 		2
#define EPL_DEV_NAME   		    "EPL259x"
#define DRIVER_VERSION          "2.0.6"

struct input_dev dev;
struct hwmsen_object *ps_hw, * als_hw;
static struct epl_sensor_priv *epl_sensor_obj = NULL;
static struct platform_driver epl_sensor_alsps_driver;
static struct wake_lock ps_lock;
static struct mutex sensor_mutex;
bool polling_flag = true;
bool eint_flag = true;

#if PS_DYN_K
static int dynk_polling_delay = 200;
int dynk_min_ps_raw_data = 0xffff;
int dynk_max_ir_data;

u32 dynk_thd_low = 0;
u32 dynk_thd_high = 0;

int dynk_low_offset;
int dynk_high_offset;

bool dynk_change_flag = false;
u16 dynk_change_thd_max;
u16 dynk_thd_offset;

u8 dynk_last_status = 0;

#if PS_DYN_K_STR
bool dynk_enhance_flag = true;
u8 dynk_enhance_integration_time;
u8 dynk_enhance_gain;
u8 dynk_enhance_adc;
u16 dynk_enhance_ch0;
u16 dynk_enhance_ch1;
u16 dynk_enhance_max_ch0;
#endif

#endif

#if ALS_DYN_INTT
//Dynamic INTT
int dynamic_intt_idx;
int dynamic_intt_init_idx = 1;	//initial dynamic_intt_idx
int c_gain;
int dynamic_intt_lux = 0;

uint16_t dynamic_intt_high_thr;
uint16_t dynamic_intt_low_thr;
uint32_t dynamic_intt_max_lux = 12000;
uint32_t dynamic_intt_min_lux = 0;
uint32_t dynamic_intt_min_unit = 1000;

static int als_dynamic_intt_intt[] = {EPL_ALS_INTT_8192, EPL_ALS_INTT_64};
static int als_dynamic_intt_value[] = {8192, 64};
static int als_dynamic_intt_gain[] = {EPL_GAIN_MID, EPL_GAIN_MID};
static int als_dynamic_intt_high_thr[] = {60000, 53000};
static int als_dynamic_intt_low_thr[] = {200, 200};
static int als_dynamic_intt_intt_num =  sizeof(als_dynamic_intt_value)/sizeof(int);
#endif

#if HS_ENABLE
static struct mutex hs_sensor_mutex;
static bool hs_enable_flag = false;
#endif
static epl_optical_sensor epl_sensor;
int i2c_max_count=8;
static bool first_als_report_flag = true;

//ps calibration file location
static const char ps_cal_file[]="/data/data/com.eminent.ps.calibration/ps.dat";

//als calibration file location
static const char als_cal_file[]="/data/data/com.eminent.ps.calibration/als.dat";

static int PS_h_offset = 3000;
static int PS_l_offset = 2000;
static int PS_MAX_XTALK = 30000;

int als_frame_time = 0;
int ps_frame_time = 0;

#if PS_GES
static bool ps_ges_enable_flag = false;
u16 ges_threshold_low = 400;
u16 ges_threshold_high = 500;
#define KEYCODE_LEFT			KEY_LEFTALT
bool ps_ges_suspend_flag = false;
#endif

typedef struct _epl_raw_data
{
    u8 raw_bytes[PACKAGE_SIZE];
} epl_raw_data;
static epl_raw_data	gRawData;

extern struct alsps_hw *epl259x_get_cust_alsps_hw(void);
#if MTK_LTE
static int alsps_init_flag =-1; // 0<==>OK -1 <==> fail
static int alsps_local_init(void);
static int alsps_remove();
static struct alsps_init_info epl_sensor_init_info = {
		.name = EPL_DEV_NAME,
		.init = alsps_local_init,
		.uninit = alsps_remove,

};
#endif

/*----------------------------------------------------------------------------*/
#define APS_TAG                 	  	"[ALS/PS] "
#define APS_FUN(f)              	  	printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    	    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    	    printk(KERN_INFO APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    	    printk(KERN_INFO fmt, ##args)

/*----------------------------------------------------------------------------*/
static struct i2c_client *epl_sensor_i2c_client = NULL;

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id epl_sensor_i2c_id[] = {{EPL_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_epl_sensor= { I2C_BOARD_INFO(EPL_DEV_NAME, (0x94>>1))};

/*----------------------------------------------------------------------------*/
static int epl_sensor_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int epl_sensor_i2c_remove(struct i2c_client *client);
static int epl_sensor_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int epl_sensor_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int epl_sensor_i2c_resume(struct i2c_client *client);
static void epl_sensor_eint_func(void);

static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd);
static int set_lsensor_intr_threshold(uint16_t low_thd, uint16_t high_thd);

int epl_sensor_ps_operate(void* self, uint32_t command, void* buff_in, int size_in, void* buff_out, int size_out, int* actualout);
int epl_sensor_als_operate(void* self, uint32_t command, void* buff_in, int size_in, void* buff_out, int size_out, int* actualout);
#if PS_DYN_K
void epl_sensor_restart_dynk_polling(void);
#endif
void epl_sensor_update_mode(struct i2c_client *client);
int epl_sensor_read_ps(struct i2c_client *client);
int epl_sensor_read_als_status(struct i2c_client *client);
static int ps_sensing_time(int intt, int adc, int cycle);
static int als_sensing_time(int intt, int adc, int cycle);

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
typedef enum
{
    CMC_BIT_ALS   	= 1,
    CMC_BIT_PS     	= 2,
#if HS_ENABLE
    CMC_BIT_HS     	= 4,
#endif
#if PS_GES
    CMC_BIT_GES     = 8,
#endif
} CMC_BIT;

typedef enum
{
    CMC_BIT_RAW   			= 0x0,
    CMC_BIT_PRE_COUNT     	= 0x1,
    CMC_BIT_DYN_INT			= 0x2,
    CMC_BIT_DEF_LIGHT		= 0x4,
    CMC_BIT_TABLE			= 0x8,
} CMC_ALS_REPORT_TYPE;

#if ALS_DYN_INTT
typedef enum
{
    CMC_BIT_LSRC_NON		= 0x0,
    CMC_BIT_LSRC_SCALE     	= 0x1,
    CMC_BIT_LSRC_SLOPE		= 0x2,
    CMC_BIT_LSRC_BOTH       = 0x3,
} CMC_LSRC_REPORT_TYPE;
#endif

/*----------------------------------------------------------------------------*/
struct epl_sensor_i2c_addr      /*define a series of i2c slave address*/
{
    u8  write_addr;
    u8  ps_thd;     /*PS INT threshold*/
};

/*----------------------------------------------------------------------------*/
struct epl_sensor_priv
{
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct delayed_work  eint_work;
#if PS_DYN_K
    struct delayed_work  dynk_thd_polling_work;
#endif
#if HS_ENABLE || PS_GES
	struct delayed_work  polling_work;
#endif
    struct input_dev *gs_input_dev;
    struct input_dev *hb_input_dev;

    /*i2c address group*/
    struct epl_sensor_i2c_addr  addr;

    /*misc*/
    atomic_t    trace;
    atomic_t   	als_suspend;
#if HS_ENABLE
	atomic_t   	hs_suspend;
#endif
#if PS_GES
	atomic_t   	ges_suspend;
#endif
    atomic_t    ps_suspend;

    /*data*/
    u16		    lux_per_count;
    ulong       enable;         	/*record HAL enalbe status*/
    ulong      	pending_intr;   	/*pending interrupt*/

    /*data*/
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL-1];
    u32         als_value[C_CUST_ALS_LEVEL];
#if ALS_DYN_INTT
    uint32_t ratio;
    uint32_t last_ratio;
    int c_gain_h; // fluorescent (C1)
    int c_gain_l; // incandescent (C2)
    uint32_t lsource_thd_high; //different light source boundary (N) fluorescent (C1)
    uint32_t lsource_thd_low; //different light source boundary (N) incandescent (C2)
#endif
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;
#endif
};

/*----------------------------------------------------------------------------*/
static struct i2c_driver epl_sensor_i2c_driver =
{
    .probe     	= epl_sensor_i2c_probe,
    .remove     = epl_sensor_i2c_remove,
    .detect     = epl_sensor_i2c_detect,
    .suspend    = epl_sensor_i2c_suspend,
    .resume     = epl_sensor_i2c_resume,
    .id_table   = epl_sensor_i2c_id,
    //.address_data = &epl_sensor_addr_data,
    .driver = {
        //.owner  = THIS_MODULE,
        .name   = EPL_DEV_NAME,
    },
};



/*
//====================I2C write operation===============//
*/
static int epl_sensor_I2C_Write_Cmd(struct i2c_client *client, uint8_t regaddr, uint8_t data, uint8_t txbyte)
{
    uint8_t buffer[2];
    int ret = 0;
    int retry;

    buffer[0] = regaddr;
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

static int epl_sensor_I2C_Write(struct i2c_client *client, uint8_t regaddr, uint8_t data)
{
    int ret = 0;
    ret = epl_sensor_I2C_Write_Cmd(client, regaddr, data, 0x02);
    return ret;
}


/*
//====================I2C read operation===============//
*/
static int epl_sensor_I2C_Read(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount)
{
    int ret = 0;
    int retry;
    int read_count=0, rx_count=0;

    while(bytecount>0)
    {
        epl_sensor_I2C_Write_Cmd(client, regaddr+read_count, 0x00, 0x01);

        for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
        {
            rx_count = bytecount>i2c_max_count?i2c_max_count:bytecount;
            ret = i2c_master_recv(client, &gRawData.raw_bytes[read_count], rx_count);

            if (ret == rx_count)
                break;

            APS_ERR("i2c read error,RXBYTES %d\r\n",ret);
            mdelay(10);
        }

        if(retry>=I2C_RETRY_COUNT)
        {
            APS_ERR("i2c read retry over %d\n", I2C_RETRY_COUNT);
            return -EINVAL;
        }
        bytecount-=rx_count;
        read_count+=rx_count;
    }

    return ret;
}

#if PS_GES
static void epl_sensor_notify_event(void)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    struct input_dev *idev = epld->gs_input_dev;

    APS_LOG("  --> LEFT\n\n");
    input_report_key(idev, KEYCODE_LEFT, 1);
    input_report_key(idev,  KEYCODE_LEFT, 0);
    input_sync(idev);
}
#endif
/*
//====================write global variable===============//
*/
static void write_global_variable(struct i2c_client *client)
{
    u8 buf;
     struct epl_sensor_priv *obj = epl_sensor_obj;
#if HS_ENABLE
    bool enable_hs = test_bit(CMC_BIT_HS, &obj->enable) && atomic_read(&obj->hs_suspend)==0;
#endif
#if PS_GES
    bool enable_ges = test_bit(CMC_BIT_GES, &obj->enable) && atomic_read(&obj->ges_suspend)==0;
#endif
    //wake up chip
    buf = epl_sensor.reset | epl_sensor.power;
    epl_sensor_I2C_Write(client, 0x11, buf);

    /* read revno*/
    epl_sensor_I2C_Read(client, 0x20, 2);
    epl_sensor.revno = gRawData.raw_bytes[0] | gRawData.raw_bytes[1] << 8;

    /*chip refrash*/
    epl_sensor_I2C_Write(client, 0xfd, 0x8e);
    epl_sensor_I2C_Write(client, 0xfe, 0x22);
    epl_sensor_I2C_Write(client, 0xfe, 0x02);
    epl_sensor_I2C_Write(client, 0xfd, 0x00);

    epl_sensor_I2C_Write(client, 0xfc, EPL_A_D | EPL_NORMAL| EPL_GFIN_ENABLE | EPL_VOS_ENABLE | EPL_DOC_ON);

#if HS_ENABLE
	if(enable_hs)
	{
	    epl_sensor.mode = EPL_MODE_PS;
	    epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | EPL_MODE_IDLE);

		/*hs setting*/
		buf = epl_sensor.hs.integration_time | epl_sensor.hs.gain;
		epl_sensor_I2C_Write(client, 0x03, buf);

		buf = epl_sensor.hs.adc | epl_sensor.hs.cycle;
		epl_sensor_I2C_Write(client, 0x04, buf);

		buf = epl_sensor.hs.ir_on_control | epl_sensor.hs.ir_mode | epl_sensor.hs.ir_driver;
		epl_sensor_I2C_Write(client, 0x05, buf);

		buf = epl_sensor.hs.compare_reset | epl_sensor.hs.lock;



		epl_sensor_I2C_Write(client, 0x1b, buf);

	}
#if !PS_GES	/*PS_GES*/
	else
#elif PS_GES
    else if(enable_ges)
#endif  /*PS_GES*/
#endif

#if PS_GES

#if !HS_ENABLE /*HS_ENABLE*/
    if(enable_ges)
#endif  /*HS_ENABLE*/
    {
        /*ges setting*/
        buf = epl_sensor.ges.integration_time | epl_sensor.ges.gain;
        epl_sensor_I2C_Write(client, 0x03, buf);

        buf = epl_sensor.ges.adc | epl_sensor.ges.cycle;
        epl_sensor_I2C_Write(client, 0x04, buf);

        buf = epl_sensor.ges.ir_on_control | epl_sensor.ges.ir_mode | epl_sensor.ges.ir_drive;
        epl_sensor_I2C_Write(client, 0x05, buf);

        buf = epl_sensor.interrupt_control | epl_sensor.ges.persist |epl_sensor.ges.interrupt_type;
        epl_sensor_I2C_Write(client, 0x06, buf);

        buf = epl_sensor.ges.compare_reset | epl_sensor.ges.lock;
        epl_sensor_I2C_Write(client, 0x1b, buf);

        epl_sensor_I2C_Write(client, 0x22, (u8)(epl_sensor.ges.cancelation& 0xff));
        epl_sensor_I2C_Write(client, 0x23, (u8)((epl_sensor.ges.cancelation & 0xff00) >> 8));
        set_psensor_intr_threshold(epl_sensor.ges.low_threshold, epl_sensor.ges.high_threshold);
    }
    else
#endif
    {
        /*ps setting*/
        buf = epl_sensor.ps.integration_time | epl_sensor.ps.gain;
        epl_sensor_I2C_Write(client, 0x03, buf);

        buf = epl_sensor.ps.adc | epl_sensor.ps.cycle;
        epl_sensor_I2C_Write(client, 0x04, buf);

        buf = epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive;
        epl_sensor_I2C_Write(client, 0x05, buf);

        buf = epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type;
        epl_sensor_I2C_Write(client, 0x06, buf);

        buf = epl_sensor.ps.compare_reset | epl_sensor.ps.lock;
        epl_sensor_I2C_Write(client, 0x1b, buf);

        epl_sensor_I2C_Write(client, 0x22, (u8)(epl_sensor.ps.cancelation& 0xff));
        epl_sensor_I2C_Write(client, 0x23, (u8)((epl_sensor.ps.cancelation & 0xff00) >> 8));
        set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);

        /*als setting*/
        buf = epl_sensor.als.integration_time | epl_sensor.als.gain;
        epl_sensor_I2C_Write(client, 0x01, buf);

        buf = epl_sensor.als.adc | epl_sensor.als.cycle;
        epl_sensor_I2C_Write(client, 0x02, buf);

        buf = epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type;
        epl_sensor_I2C_Write(client, 0x07, buf);

        buf = epl_sensor.als.compare_reset | epl_sensor.als.lock;
        epl_sensor_I2C_Write(client, 0x12, buf);

        set_lsensor_intr_threshold(epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
	}

    //set mode and wait
    buf = epl_sensor.wait | epl_sensor.mode;
    epl_sensor_I2C_Write(client, 0x00, buf);

}

static int write_factory_calibration(struct epl_sensor_priv *epl_data, char* ps_data, int ps_cal_len)
{
    struct file *fp_cal;

	mm_segment_t fs;
	loff_t pos;

	APS_FUN();
    pos = 0;

	fp_cal = filp_open(ps_cal_file, O_CREAT|O_RDWR|O_TRUNC, 0755/*S_IRWXU*/);
	if (IS_ERR(fp_cal))
	{
		APS_ERR("[ELAN]create file_h error\n");
		return -1;
	}

    fs = get_fs();
	set_fs(KERNEL_DS);

	vfs_write(fp_cal, ps_data, ps_cal_len, &pos);

    filp_close(fp_cal, NULL);

	set_fs(fs);

	return 0;
}
static bool read_factory_calibration(void)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    struct file *fp;
    mm_segment_t fs;
    loff_t pos;
    char buffer[100]= {0};
    if(epl_sensor.ps.factory.calibration_enable && !epl_sensor.ps.factory.calibrated)
    {
		fp = filp_open(ps_cal_file, O_RDWR, S_IRUSR);
        if (IS_ERR(fp))
        {
            APS_ERR("NO calibration file\n");
            epl_sensor.ps.factory.calibration_enable =  false;
        }
        else
        {
            pos = 0;
            fs = get_fs();
            set_fs(KERNEL_DS);
            vfs_read(fp, buffer, sizeof(buffer), &pos);
            filp_close(fp, NULL);

            sscanf(buffer, "%hu,%hu,%hu", &epl_sensor.ps.factory.cancelation, &epl_sensor.ps.factory.high_threshold, &epl_sensor.ps.factory.low_threshold);
            set_fs(fs);

            epl_sensor.ps.high_threshold = epl_sensor.ps.factory.high_threshold;
            epl_sensor.ps.low_threshold = epl_sensor.ps.factory.low_threshold;
            epl_sensor.ps.cancelation = epl_sensor.ps.factory.cancelation;
        }

        epl_sensor_I2C_Write(obj->client,0x22, (u8)(epl_sensor.ps.cancelation& 0xff));
        epl_sensor_I2C_Write(obj->client,0x23, (u8)((epl_sensor.ps.cancelation & 0xff00) >> 8));
        set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);

        epl_sensor.ps.factory.calibrated = true;
    }

    if(epl_sensor.als.factory.calibration_enable && !epl_sensor.als.factory.calibrated)
    {
        fp = filp_open(ps_cal_file, O_RDWR, S_IRUSR);
        if (IS_ERR(fp))
        {
            APS_ERR("NO calibration file\n");
            epl_sensor.als.factory.calibration_enable =  false;
        }
        else
        {
            pos = 0;
            fs = get_fs();
            set_fs(KERNEL_DS);
            vfs_read(fp, buffer, sizeof(buffer), &pos);
            filp_close(fp, NULL);

            sscanf(buffer, "%hu", &epl_sensor.als.factory.lux_per_count);
            set_fs(fs);
        }
        epl_sensor.als.factory.calibrated = true;
    }
    return true;
}

static int epl_run_ps_calibration(struct epl_sensor_priv *epl_data)
{
    struct epl_sensor_priv *epld = epl_data;
    bool enable_ps = test_bit(CMC_BIT_PS, &epld->enable) && atomic_read(&epld->ps_suspend)==0;
    u16 ch1=0;
    u32 ch1_all=0;
    int count =5, i;
    int ps_hthr=0, ps_lthr=0, ps_cancelation=0, ps_cal_len = 0;
    char ps_calibration[20];


    if(PS_MAX_XTALK < 0)
    {
        APS_ERR("[%s]:Failed: PS_MAX_XTALK < 0 \r\n", __func__);
        return -EINVAL;
    }

    if(enable_ps == 0)
    {
        set_bit(CMC_BIT_PS, &epld->enable);
        epl_sensor_update_mode(epld->client);
    }

    polling_flag = false;

    for(i=0; i<count; i++)
    {
        msleep(50);
    	switch(epl_sensor.mode)
    	{
    		case EPL_MODE_PS:
    		case EPL_MODE_ALS_PS:
    		    if(enable_ps == true && polling_flag == true && eint_flag == true)
    		 	    epl_sensor_read_ps(epld->client);
    			ch1 = epl_sensor.ps.data.data;
		    break;
    	}

    	ch1_all = ch1_all + ch1;
    	if(epl_sensor.wait == EPL_WAIT_SINGLE)
    		epl_sensor_I2C_Write(epld->client,0x11, epl_sensor.power | epl_sensor.reset);
    }


    ch1 = (u16)(ch1_all/count);

    if(ch1 > PS_MAX_XTALK)
    {
        APS_ERR("[%s]:Failed: ch1 > max_xtalk(%d) \r\n", __func__, ch1);
        return -EINVAL;
    }
    else if(ch1 <= 0)
    {
        APS_ERR("[%s]:Failed: ch1 = 0\r\n", __func__);
        return -EINVAL;
    }

    ps_hthr = ch1 + PS_h_offset;
    ps_lthr = ch1 + PS_l_offset;

    ps_cal_len = sprintf(ps_calibration, "%d,%d,%d", ps_cancelation, ps_hthr, ps_lthr);

    if(write_factory_calibration(epld, ps_calibration, ps_cal_len) < 0)
    {
        APS_ERR("[%s] create file error \n", __func__);
        return -EINVAL;
    }

    epl_sensor.ps.low_threshold = ps_lthr;
	epl_sensor.ps.high_threshold = ps_hthr;
	set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);

	APS_LOG("[%s]: ch1 = %d\n", __func__, ch1);

    polling_flag = true;
	return ch1;
}


static void set_als_ps_intr_type(struct i2c_client *client, bool ps_polling, bool als_polling)
{

    //set als / ps interrupt control mode and trigger type
	switch((ps_polling << 1) | als_polling)
	{
		case 0: // ps and als interrupt
			epl_sensor.interrupt_control = 	EPL_INT_CTRL_ALS_OR_PS;
			epl_sensor.als.interrupt_type = EPL_INTTY_ACTIVE;
			epl_sensor.ps.interrupt_type = EPL_INTTY_ACTIVE;
		break;

		case 1: //ps interrupt and als polling
			epl_sensor.interrupt_control = 	EPL_INT_CTRL_PS;
			epl_sensor.als.interrupt_type = EPL_INTTY_DISABLE;
			epl_sensor.ps.interrupt_type = EPL_INTTY_ACTIVE;
		break;

		case 2: // ps polling and als interrupt
			epl_sensor.interrupt_control = 	EPL_INT_CTRL_ALS;
			epl_sensor.als.interrupt_type = EPL_INTTY_ACTIVE;
			epl_sensor.ps.interrupt_type = EPL_INTTY_DISABLE;
		break;

		case 3: //ps and als polling
			epl_sensor.interrupt_control = 	EPL_INT_CTRL_ALS_OR_PS;
			epl_sensor.als.interrupt_type = EPL_INTTY_DISABLE;
			epl_sensor.ps.interrupt_type = EPL_INTTY_DISABLE;
		break;
	}
}
/*
//====================initial global variable===============//
*/
static void initial_global_variable(struct i2c_client *client, struct epl_sensor_priv *obj)
{
    //general setting
    epl_sensor.power = EPL_POWER_ON;
    epl_sensor.reset = EPL_RESETN_RUN;
    epl_sensor.mode = EPL_MODE_IDLE;
    epl_sensor.wait = EPL_WAIT_40_MS;
    epl_sensor.osc_sel = EPL_OSC_SEL_1MHZ;

    //als setting
    epl_sensor.als.polling_mode = obj->hw->polling_mode_als;
    epl_sensor.als.integration_time = EPL_ALS_INTT_16;
    epl_sensor.als.gain = EPL_GAIN_MID;
    epl_sensor.als.adc = EPL_PSALS_ADC_13;
    epl_sensor.als.cycle = EPL_CYCLE_16;
    epl_sensor.als.interrupt_channel_select = EPL_ALS_INT_CHSEL_1;
    epl_sensor.als.persist = EPL_PERIST_1;
    epl_sensor.als.compare_reset = EPL_CMP_RESET;
    epl_sensor.als.lock = EPL_UN_LOCK;
	if (!debug_mode_flag)
	{
		epl_sensor.als.report_type = CMC_BIT_RAW; //CMC_BIT_DYN_INT;
	}
	else
	{
		epl_sensor.als.report_type = CMC_BIT_TABLE; //CMC_BIT_DYN_INT;
	}
    epl_sensor.als.high_threshold = obj->hw->als_threshold_high;
    epl_sensor.als.low_threshold = obj->hw->als_threshold_low;

    //als factory
    epl_sensor.als.factory.calibration_enable =  false;
    epl_sensor.als.factory.calibrated = false;
    epl_sensor.als.factory.lux_per_count = LUX_PER_COUNT;
#if ALS_DYN_INTT

    epl_sensor.als.lsrc_type = CMC_BIT_LSRC_NON;

    if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
    {
        dynamic_intt_idx = dynamic_intt_init_idx;
        epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
        epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
        dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
        dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
    }

    c_gain = 21000; // 21000/1000=21

    obj->lsource_thd_high = 1900; //different light source boundary (N) (1.9)
    obj->lsource_thd_low = 1500;   //1.5
    obj->c_gain_h = 21000; //fluorescent (C1) (21)
    obj->c_gain_l = 15000;  //incandescent (C2)   (15)

#endif

    //ps setting
    epl_sensor.ps.polling_mode = obj->hw->polling_mode_ps;
    epl_sensor.ps.integration_time = EPL_PS_INTT_272;
    epl_sensor.ps.gain = EPL_GAIN_LOW;
#if PS_DYN_K
    dynk_max_ir_data = 50000; // ps max ch0
    dynk_low_offset = 500;
    dynk_high_offset = 800;
    dynk_change_thd_max = 30000; //If NEAR, change H/L
    dynk_thd_offset = 0; //500; //If NEAR, change H/L offset
#if PS_DYN_K_STR
    dynk_enhance_max_ch0 = 50000; //enhance max ch0
    dynk_enhance_integration_time = EPL_ALS_INTT_64;
    dynk_enhance_gain = EPL_GAIN_LOW;
    dynk_enhance_adc = EPL_PSALS_ADC_12;
#endif
#endif
    epl_sensor.ps.adc = EPL_PSALS_ADC_11;
    epl_sensor.ps.cycle = EPL_CYCLE_64;
    epl_sensor.ps.persist = EPL_PERIST_1;
    epl_sensor.ps.ir_on_control = EPL_IR_ON_CTRL_ON;
    epl_sensor.ps.ir_mode = EPL_IR_MODE_CURRENT;
    epl_sensor.ps.ir_drive = EPL_IR_DRIVE_100;
    epl_sensor.ps.compare_reset = EPL_CMP_RESET;
    epl_sensor.ps.lock = EPL_UN_LOCK;
    epl_sensor.ps.high_threshold = obj->hw->ps_threshold_high;
    epl_sensor.ps.low_threshold = obj->hw->ps_threshold_low;

    //ps factory
    epl_sensor.ps.factory.calibration_enable =  false;
    epl_sensor.ps.factory.calibrated = false;
    epl_sensor.ps.factory.cancelation= 0;

#if HS_ENABLE
	//hs setting
    epl_sensor.hs.integration_time = EPL_PS_INTT_384;
	epl_sensor.hs.integration_time_max = EPL_PS_INTT_272;
	epl_sensor.hs.integration_time_min = EPL_PS_INTT_32;
    epl_sensor.hs.gain = EPL_GAIN_LOW;
    epl_sensor.hs.adc = EPL_PSALS_ADC_11;
    epl_sensor.hs.cycle = EPL_CYCLE_4;
    epl_sensor.hs.ir_on_control = EPL_IR_ON_CTRL_ON;
    epl_sensor.hs.ir_mode = EPL_IR_MODE_CURRENT;
    epl_sensor.hs.ir_driver = EPL_IR_DRIVE_200;
    epl_sensor.hs.compare_reset = EPL_CMP_RESET;
    epl_sensor.hs.lock = EPL_UN_LOCK;
	epl_sensor.hs.low_threshold = 6400;
	epl_sensor.hs.mid_threshold = 25600;
	epl_sensor.hs.high_threshold = 60800;
#endif

#if PS_GES
    //ps setting
    epl_sensor.ges.polling_mode = obj->hw->polling_mode_ps;
    epl_sensor.ges.integration_time = EPL_PS_INTT_384;
    epl_sensor.ges.gain = EPL_GAIN_LOW;
    epl_sensor.ges.adc = EPL_PSALS_ADC_12;
    epl_sensor.ges.cycle = EPL_CYCLE_2;
    epl_sensor.ges.persist = EPL_PERIST_1;
    epl_sensor.ges.ir_on_control = EPL_IR_ON_CTRL_ON;
    epl_sensor.ges.ir_mode = EPL_IR_MODE_CURRENT;
    epl_sensor.ges.ir_drive = EPL_IR_DRIVE_200;
    epl_sensor.ges.compare_reset = EPL_CMP_RESET;
    epl_sensor.ges.lock = EPL_UN_LOCK;
    epl_sensor.ges.high_threshold = ges_threshold_high;
    epl_sensor.ges.low_threshold = ges_threshold_low;
#endif
    //set als / ps interrupt control mode and trigger type
    set_als_ps_intr_type(client, epl_sensor.ps.polling_mode, epl_sensor.als.polling_mode);
    //write setting to sensor
    write_global_variable(client);
}

#if ALS_DYN_INTT
long raw_convert_to_lux(u16 raw_data)
{
    long lux = 0;
    long dyn_intt_raw = 0;
    dyn_intt_raw = (raw_data * 10) / (10*als_dynamic_intt_value[dynamic_intt_idx] / als_dynamic_intt_value[1]); //float calculate

    APS_LOG("[%s]: dyn_intt_raw=%ld \r\n", __func__, dyn_intt_raw);

    if(dyn_intt_raw > 0xffff)
        epl_sensor.als.dyn_intt_raw = 0xffff;
    else
        epl_sensor.als.dyn_intt_raw = dyn_intt_raw;

    lux = c_gain * epl_sensor.als.dyn_intt_raw;
#if ALS_DEBUG
    APS_LOG("[%s]:raw_data=%d, epl_sensor.als.dyn_intt_raw=%d, lux=%ld\r\n", __func__, raw_data, epl_sensor.als.dyn_intt_raw, lux);
#endif

    if(lux >= (dynamic_intt_max_lux*dynamic_intt_min_unit)){
#if ALS_DEBUG
        APS_LOG("[%s]:raw_convert_to_lux: change max lux\r\n", __func__);
#endif
        lux = dynamic_intt_max_lux * dynamic_intt_min_unit;
    }
    else if(lux <= (dynamic_intt_min_lux*dynamic_intt_min_unit)){
#if ALS_DEBUG
        APS_LOG("[%s]:raw_convert_to_lux: change min lux\r\n", __func__);
#endif
        lux = dynamic_intt_min_lux * dynamic_intt_min_unit;
    }

    return lux;
}
#endif

static int epl_sensor_get_als_value(struct epl_sensor_priv *obj, u16 als)
{
    int idx;
    int invalid = 0;
#if ALS_DYN_INTT
	long now_lux=0, lux_tmp=0;
    bool change_flag = false;
#endif

    switch(epl_sensor.als.report_type)
    {
        case CMC_BIT_RAW:
            return als;
            break;

        case CMC_BIT_PRE_COUNT:
            return (als * epl_sensor.als.factory.lux_per_count)/1000;
            break;

        case CMC_BIT_TABLE:
            for(idx = 0; idx < obj->als_level_num; idx++)
            {
                if(als < obj->hw->als_level[idx])
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
                APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
                return obj->hw->als_value[idx];
            }
            else
            {
                APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);
                return als;
            }
            break;
#if ALS_DYN_INTT
		case CMC_BIT_DYN_INT:

            if(epl_sensor.als.lsrc_type != CMC_BIT_LSRC_NON)
            {
                long luxratio = 0;
                epl_sensor_read_als_status(obj->client);

                if (epl_sensor.als.data.channels[0] == 0)
                {
                   epl_sensor.als.data.channels[0] = 1;
                   APS_ERR("[%s]:read ch0 data is 0 \r\n", __func__);
                }

                luxratio = (long)((als*dynamic_intt_min_unit) / epl_sensor.als.data.channels[0]); //lux ratio (A=CH1/CH0)

                obj->ratio = luxratio;
                if((epl_sensor.als.saturation >> 5) == 0)
                {
                    if(epl_sensor.als.lsrc_type == CMC_BIT_LSRC_SCALE || epl_sensor.als.lsrc_type == CMC_BIT_LSRC_BOTH)
                    {
                        if(obj->ratio == 0){
                            obj->last_ratio = luxratio;
                        }
                        else{
                            obj->last_ratio = (luxratio + obj->last_ratio*9)  / 10;
                        }

                        if (obj->last_ratio >= obj->lsource_thd_high)  //fluorescent (C1)
                        {
                            c_gain = obj->c_gain_h;
                        }
                        else if (obj->last_ratio <= obj->lsource_thd_low)   //incandescent (C2)
                        {
                            c_gain = obj->c_gain_l;
                        }
                        else if(epl_sensor.als.lsrc_type == CMC_BIT_LSRC_BOTH)
                        {
                            int a = 0, b = 0, c = 0;
                            a = (obj->c_gain_h - obj->c_gain_l) * dynamic_intt_min_unit / (obj->lsource_thd_high - obj->lsource_thd_low);
                            b = (obj->c_gain_h) - ((a * obj->lsource_thd_high)/dynamic_intt_min_unit );
                            c = ((a * obj->last_ratio)/dynamic_intt_min_unit) + b;

                            if(c > obj->c_gain_h)
                                c_gain = obj->c_gain_h;
                            else if (c < obj->c_gain_l)
                                c_gain = obj->c_gain_l;
                            else
                                c_gain = c;
                        }
                    }
                    else
                    {
                        if (luxratio >= obj->lsource_thd_high)  //fluorescent (C1)
                        {
                            c_gain = obj->c_gain_h;
                        }
                        else if (luxratio <= obj->lsource_thd_low)   //incandescent (C2)
                        {
                            c_gain = obj->c_gain_l;
                        }
                        else{ /*mix*/
                            int a = 0, b = 0, c = 0;
                            a = (obj->c_gain_h - obj->c_gain_l) * dynamic_intt_min_unit / (obj->lsource_thd_high - obj->lsource_thd_low);
                            b = (obj->c_gain_h) - ((a * obj->lsource_thd_high)/dynamic_intt_min_unit );
                            c = ((a * luxratio)/dynamic_intt_min_unit) + b;

                            if(c > obj->c_gain_h)
                                c_gain = obj->c_gain_h;
                            else if (c < obj->c_gain_l)
                                c_gain = obj->c_gain_l;
                            else
                                c_gain = c;
                        }
                    }
#if ALS_DEBUG
                     APS_LOG("[%s]:ch0=%d, ch1=%d, c_gain=%d, obj->ratio=%d, obj->last_ratio=%d \r\n\n",
                                      __func__,epl_sensor.als.data.channels[0], als, c_gain, obj->ratio, obj->last_ratio);
#endif
                }
                else
                {
                    APS_LOG("[%s]: ALS saturation(%d) \r\n", __func__, (epl_sensor.als.saturation >> 5));
                }
            }

#if ALS_DEBUG
            APS_LOG("[%s]: dynamic_intt_idx=%d, als_dynamic_intt_value=%d, dynamic_intt_gain=%d, als=%d \r\n",
                                    __func__, dynamic_intt_idx, als_dynamic_intt_value[dynamic_intt_idx], als_dynamic_intt_gain[dynamic_intt_idx], als);
#endif

            if(als > dynamic_intt_high_thr)
        	{
          		if(dynamic_intt_idx == (als_dynamic_intt_intt_num - 1)){
                    als = dynamic_intt_high_thr;
          		    lux_tmp = raw_convert_to_lux(als);
#if ALS_DEBUG
        	      	APS_LOG(">>>>>>>>>>>>>>>>>>>>>>>> INTT_MAX_LUX\r\n");
#endif
          		}
                else{
                    change_flag = true;
        			als  = dynamic_intt_high_thr;
              		lux_tmp = raw_convert_to_lux(als);
                    dynamic_intt_idx++;
#if ALS_DEBUG
                    APS_LOG(">>>>>>>>>>>>>>>>>>>>>>>>change INTT high: %d, raw: %d \r\n", dynamic_intt_idx, als);
#endif
                }
            }
            else if(als < dynamic_intt_low_thr)
            {
                if(dynamic_intt_idx == 0){
                    //als = dynamic_intt_low_thr;
                    lux_tmp = raw_convert_to_lux(als);
#if ALS_DEBUG
                    APS_LOG(">>>>>>>>>>>>>>>>>>>>>>>> INTT_MIN_LUX\r\n");
#endif
                }
                else{
                    change_flag = true;
        			als  = dynamic_intt_low_thr;
                	lux_tmp = raw_convert_to_lux(als);
                    dynamic_intt_idx--;
#if ALS_DEBUG
                    APS_LOG(">>>>>>>>>>>>>>>>>>>>>>>>change INTT low: %d, raw: %d \r\n", dynamic_intt_idx, als);
#endif
                }
            }
            else
            {
            	lux_tmp = raw_convert_to_lux(als);
            }

            now_lux = lux_tmp;
            dynamic_intt_lux = now_lux/dynamic_intt_min_unit;

            if(change_flag == true)
            {
                int als_time = 0, ps_time = 0;
                bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
                bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;

                epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
                epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
                dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
                dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
                epl_sensor_update_mode(obj->client);
                change_flag = false;
            }
            return dynamic_intt_lux;

		break;
#endif
    }

    return 0;
}


static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    struct i2c_client *client = epld->client;
    uint8_t high_msb ,high_lsb, low_msb, low_lsb;


    high_msb = (uint8_t) (high_thd >> 8);
    high_lsb = (uint8_t) (high_thd & 0x00ff);
    low_msb  = (uint8_t) (low_thd >> 8);
    low_lsb  = (uint8_t) (low_thd & 0x00ff);

    APS_LOG("%s: low_thd = %d, high_thd = %d \n", __FUNCTION__, low_thd, high_thd);

    epl_sensor_I2C_Write(client, 0x0c, low_lsb);
    epl_sensor_I2C_Write(client, 0x0d, low_msb);
    epl_sensor_I2C_Write(client, 0x0e, high_lsb);
    epl_sensor_I2C_Write(client, 0x0f, high_msb);

    return 0;
}

static int set_lsensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    struct i2c_client *client = epld->client;
    uint8_t high_msb ,high_lsb, low_msb, low_lsb;

    high_msb = (uint8_t) (high_thd >> 8);
    high_lsb = (uint8_t) (high_thd & 0x00ff);
    low_msb  = (uint8_t) (low_thd >> 8);
    low_lsb  = (uint8_t) (low_thd & 0x00ff);

    epl_sensor_I2C_Write(client, 0x08, low_lsb);
    epl_sensor_I2C_Write(client, 0x09, low_msb);
    epl_sensor_I2C_Write(client, 0x0a, high_lsb);
    epl_sensor_I2C_Write(client, 0x0b, high_msb);

    APS_LOG("%s: low_thd = %d, high_thd = %d \n", __FUNCTION__, low_thd, high_thd);

    return 0;
}


/*----------------------------------------------------------------------------*/
static void epl_sensor_dumpReg(struct i2c_client *client)
{
    APS_LOG("chip id REG 0x00 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x00));
    APS_LOG("chip id REG 0x01 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x01));
    APS_LOG("chip id REG 0x02 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x02));
    APS_LOG("chip id REG 0x03 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x03));
    APS_LOG("chip id REG 0x04 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x04));
    APS_LOG("chip id REG 0x05 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x05));
    APS_LOG("chip id REG 0x06 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x06));
    APS_LOG("chip id REG 0x07 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x07));
    APS_LOG("chip id REG 0x11 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x11));
    APS_LOG("chip id REG 0x12 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x12));
    APS_LOG("chip id REG 0x1B value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1B));
    APS_LOG("chip id REG 0x20 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x20));
    APS_LOG("chip id REG 0x21 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x21));
    APS_LOG("chip id REG 0x24 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x24));
    APS_LOG("chip id REG 0x25 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x25));
    APS_LOG("chip id REG 0xfb value = 0x%x\n", i2c_smbus_read_byte_data(client, 0xfb));
    APS_LOG("chip id REG 0xfc value = 0x%x\n", i2c_smbus_read_byte_data(client, 0xfc));
}

/*----------------------------------------------------------------------------*/
int epl259x_hw8k_init_device(struct i2c_client *client)
{
    APS_LOG("epl259x_hw8k_init_device.........\r\n");

    epl_sensor_i2c_client = client;

    APS_LOG(" I2C Addr==[0x%x],line=%d\n", epl_sensor_i2c_client->addr,__LINE__);

    return 0;
}

/*----------------------------------------------------------------------------*/
int epl_sensor_get_addr(struct alsps_hw *hw, struct epl_sensor_i2c_addr *addr)
{
    if(!hw || !addr)
    {
        return -EFAULT;
    }
    addr->write_addr= hw->i2c_addr[0];
    return 0;
}


/*----------------------------------------------------------------------------*/
static void epl_sensor_power(struct alsps_hw *hw, unsigned int on)
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
            if(!hwPowerOn(hw->power_id, hw->power_vol, EPL_DEV_NAME))
            {
                APS_ERR("power on fails!!\n");
            }
        }
        else
        {
            if(!hwPowerDown(hw->power_id, EPL_DEV_NAME))
            {
                APS_ERR("power off fail!!\n");
            }
        }
    }
    power_on = on;
}

/*----------------------------------------------------------------------------*/

int epl_sensor_read_als(struct i2c_client *client)
{
    struct epl_sensor_priv *obj = i2c_get_clientdata(client);

    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    epl_sensor_I2C_Read(obj->client, 0x13, 4);
#if PS_DYN_K && PS_DYN_K_STR
    if(dynk_enhance_flag == false)
    {
        dynk_enhance_ch0 = ((gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0]))>>2;
        dynk_enhance_ch1 = ((gRawData.raw_bytes[3]<<8) | gRawData.raw_bytes[2]))>>2;

        APS_LOG("read dynk_enhance_ch0 = %d\n", dynk_enhance_ch0);
        APS_LOG("read dynk_enhance_ch1 = %d\n", dynk_enhance_ch1);
    }
    else
#endif
    {
        epl_sensor.als.data.channels[0] = ((gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0])>>3;
        epl_sensor.als.data.channels[1] = ((gRawData.raw_bytes[3]<<8) | gRawData.raw_bytes[2])>>3;

        APS_LOG("read als channe 0 = %d\n", epl_sensor.als.data.channels[0]);
        APS_LOG("read als channe 1 = %d\n", epl_sensor.als.data.channels[1]);
    }


    return 0;
}

int epl_sensor_read_als_status(struct i2c_client *client)
{
    struct epl_sensor_priv *obj = i2c_get_clientdata(client);
    u8 buf;

    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    epl_sensor_I2C_Read(obj->client, 0x12, 1);

    buf = gRawData.raw_bytes[0];

    epl_sensor.als.saturation = (buf & 0x20);
    epl_sensor.als.compare_high = (buf & 0x10);
    epl_sensor.als.compare_low = (buf & 0x08);
    epl_sensor.als.interrupt_flag = (buf & 0x04);
    epl_sensor.als.compare_reset = (buf & 0x02);
    epl_sensor.als.lock = (buf & 0x01);

#if PS_DEBUG
	APS_LOG("als: ~~~~ ALS ~~~~~ \n");
	APS_LOG("als: buf = 0x%x\n", buf);
	APS_LOG("als: sat = 0x%x\n", epl_sensor.als.saturation);
	APS_LOG("als: cmp h = 0x%x, l = %d\n", epl_sensor.als.compare_high, epl_sensor.als.compare_low);
	APS_LOG("als: int_flag = 0x%x\n",epl_sensor.als.interrupt_flag);
	APS_LOG("als: cmp_rstn = 0x%x, lock = 0x%0x\n", epl_sensor.als.compare_reset, epl_sensor.als.lock);
#endif
    return 0;
}

/*----------------------------------------------------------------------------*/
int epl_sensor_read_ps(struct i2c_client *client)
{

    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    epl_sensor_I2C_Read(client,0x1c, 4);

	if (debug_mode_flag)
	{
		epl_sensor.ps.data.ir_data = ((gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0]);
		epl_sensor.ps.data.data = ((gRawData.raw_bytes[3]<<8) | gRawData.raw_bytes[2]);
	}
	else
	{
		epl_sensor.ps.data.ir_data = ((gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0])>>2;
		epl_sensor.ps.data.data = ((gRawData.raw_bytes[3]<<8) | gRawData.raw_bytes[2])>>2;

		if(epl_sensor.ps.data.data > 1023){
			epl_sensor.ps.data.data = 1023;
		}else if(epl_sensor.ps.data.data < 5){
			epl_sensor.ps.data.data = 0;
		}
	}

	APS_LOG("[%s]: data = %d\n", __func__, epl_sensor.ps.data.data);
	APS_LOG("[%s]: ir data = %d\n", __func__, epl_sensor.ps.data.ir_data);

    if(epl_sensor.wait == EPL_WAIT_SINGLE)
	    epl_sensor_I2C_Write(client, 0x11, epl_sensor.power | epl_sensor.reset);

    return 0;
}

int epl_sensor_read_ps_status(struct i2c_client *client)
{
    u8 buf;
#if PS_GES
    struct epl_sensor_priv *obj = epl_sensor_obj;
    u8 new_ps_state;
    u8 ges_saturation;
    bool enable_ges = test_bit(CMC_BIT_GES, &obj->enable) && atomic_read(&obj->ges_suspend)==0;
#endif
    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    epl_sensor_I2C_Read(client, 0x1b, 1);
    buf = gRawData.raw_bytes[0];

#if PS_GES
    ges_saturation = (buf & 0x20);
    new_ps_state = (buf & 0x08) >> 3;
    APS_LOG("[%s]:new_ps_state=%d, ps_ges_suspend_flag=%d \r\n", __func__, new_ps_state, ps_ges_suspend_flag);
    if(enable_ges == 1 && epl_sensor.ges.polling_mode == 1)
    {
        if( new_ps_state == 0 && (epl_sensor.ps.compare_low>>3 == 1) && ps_ges_suspend_flag == false && ges_saturation == 0)
            epl_sensor_notify_event();
    }
    else if(enable_ges == 1 && epl_sensor.ges.polling_mode == 0)
    {
        if(new_ps_state == 0 && ps_ges_suspend_flag == false && ges_saturation == 0)
           epl_sensor_notify_event();
    }
#endif

    epl_sensor.ps.saturation = (buf & 0x20);
    epl_sensor.ps.compare_high = (buf & 0x10);
    epl_sensor.ps.compare_low = (buf & 0x08);
    epl_sensor.ps.interrupt_flag = (buf & 0x04);
    epl_sensor.ps.compare_reset = (buf & 0x02);
    epl_sensor.ps.lock= (buf & 0x01);

#if PS_DEBUG
	APS_LOG("ps: ~~~~ PS ~~~~~ \n");
	APS_LOG("ps: buf = 0x%x\n", buf);
	APS_LOG("ps: sat = 0x%x\n", epl_sensor.ps.saturation);
	APS_LOG("ps: cmp h = 0x%x, l = 0x%x\n", epl_sensor.ps.compare_high, epl_sensor.ps.compare_low);
	APS_LOG("ps: int_flag = 0x%x\n",epl_sensor.ps.interrupt_flag);
	APS_LOG("ps: cmp_rstn = 0x%x, lock = %x\n", epl_sensor.ps.compare_reset, epl_sensor.ps.lock);
#endif
    return 0;
}

#if HS_ENABLE
int epl_sensor_read_hs(struct i2c_client *client)
{
    u8 buf;

    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }
    mutex_lock(&hs_sensor_mutex);
    epl_sensor_I2C_Read(client,0x1e, 2);
    epl_sensor.hs.raw = (gRawData.raw_bytes[1]<<8) | gRawData.raw_bytes[0];
    if(epl_sensor.hs.dynamic_intt == true && epl_sensor.hs.raw>epl_sensor.hs.high_threshold && epl_sensor.hs.integration_time > epl_sensor.hs.integration_time_min)
    {
		epl_sensor.hs.integration_time -= 4;
		buf = epl_sensor.hs.integration_time | epl_sensor.hs.gain;
		epl_sensor_I2C_Write(client, 0x03, buf);
    }
    else if(epl_sensor.hs.dynamic_intt == true && epl_sensor.hs.raw>epl_sensor.hs.low_threshold && epl_sensor.hs.raw <epl_sensor.hs.mid_threshold && epl_sensor.hs.integration_time <  epl_sensor.hs.integration_time_max)
	{
		epl_sensor.hs.integration_time += 4;
		buf = epl_sensor.hs.integration_time | epl_sensor.hs.gain;
		epl_sensor_I2C_Write(client, 0x03, buf);
	}

	mutex_unlock(&hs_sensor_mutex);

    if(epl_sensor.hs.raws_count<200)
    {
		epl_sensor.hs.raws[epl_sensor.hs.raws_count] = epl_sensor.hs.raw;
        epl_sensor.hs.raws_count++;
    }

    return 0;
}
#endif

#if PS_DYN_K
void epl_sensor_reset_dynk_thd(u8 last_status, u8 now_status)
{
    if(last_status==0 && now_status==1)
    {
#if PS_DYN_K_STR
        if( (epl_sensor.ps.saturation == 0) && (epl_sensor.ps.data.ir_data < dynk_max_ir_data) && (dynk_enhance_ch0 < dynk_enhance_max_ch0) )
#else
        if( (epl_sensor.ps.saturation == 0)&&(epl_sensor.ps.data.ir_data < dynk_max_ir_data) )
#endif
        {
            dynk_min_ps_raw_data = epl_sensor.ps.data.data;
        }
        dynk_thd_low = dynk_min_ps_raw_data + dynk_low_offset;
	    dynk_thd_high = dynk_min_ps_raw_data + dynk_high_offset;

        if(dynk_thd_low>65534)
            dynk_thd_low = 65534;
        if(dynk_thd_high>65535)
            dynk_thd_high = 65535;
#if PS_DEBUG
	    APS_LOG("[%s]:restart dynk ps raw = %d, min = %d, ir_data = %d\n", __func__, epl_sensor.ps.data.data, dynk_min_ps_raw_data, epl_sensor.ps.data.ir_data);
	    APS_LOG("[%s]:restart dynk thre_l = %d, thre_h = %d\n", __func__, dynk_thd_low, dynk_thd_high);
#endif
        eint_flag = false;
	    set_psensor_intr_threshold((u16)dynk_thd_low, (u16)dynk_thd_high);
        eint_flag = true;
    }
    else if(last_status==1 && now_status==0)
    {
        dynk_change_flag = true;
    }
}
#if PS_DYN_K_STR
void epl_sensor_enhance_enable(struct i2c_client *client, bool enable)
{
    int enh_time=0, ps_time=0, total_time=0;
    bool cmp_flag = false;
    u8 buf, now_cmp_l;
    ps_time = ps_sensing_time(epl_sensor.ps.integration_time, epl_sensor.ps.adc, epl_sensor.ps.cycle);

    polling_flag = false;

    //mutex_lock(&sensor_mutex);
    epl_sensor_I2C_Read(client, 0x1b, 1);
    buf = gRawData.raw_bytes[0];
    epl_sensor.ps.compare_high = (buf & 0x10);
	now_cmp_l = (buf & 0x08);

	if(now_cmp_l != epl_sensor.ps.compare_low)
	{
	    APS_LOG("[%s]: buf=0x%x, now_cmp_l=0x%x, epl_sensor.ps.compare_low=0x%x \r\n", __func__, buf, now_cmp_l, epl_sensor.ps.compare_low);
        //PS unlock and reset
        epl_sensor.ps.compare_reset = EPL_CMP_RESET;
        epl_sensor.ps.lock = EPL_UN_LOCK;
        epl_sensor_I2C_Write(client, 0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);

        epl_sensor.ps.interrupt_flag = EPL_INT_CLEAR;
	    epl_sensor.als.interrupt_flag = EPL_INT_CLEAR;
        cmp_flag = true;
	}

    epl_sensor_I2C_Write(client, 0x00, epl_sensor.wait | EPL_MODE_IDLE);

    if(enable == true){
        enh_time = als_sensing_time(dynk_enhance_integration_time, dynk_enhance_adc, epl_sensor.als.cycle) / 2;
        epl_sensor_I2C_Write(client, 0x01, dynk_enhance_integration_time | dynk_enhance_gain);
        epl_sensor_I2C_Write(client, 0x02, dynk_enhance_adc | epl_sensor.als.cycle);
        epl_sensor_I2C_Write(client, 0xfc, EPL_A_D | EPL_NORMAL| EPL_GFIN_ENABLE | EPL_VOS_ENABLE | EPL_DOC_OFF);
    }
    else
    {
        enh_time = als_sensing_time(epl_sensor.als.integration_time, epl_sensor.als.adc, epl_sensor.als.cycle);
        epl_sensor_I2C_Write(client, 0x01, epl_sensor.als.integration_time | epl_sensor.als.gain);
        epl_sensor_I2C_Write(client, 0x02, epl_sensor.als.adc | epl_sensor.als.cycle);
        epl_sensor_I2C_Write(client, 0xfc, EPL_A_D | EPL_NORMAL| EPL_GFIN_ENABLE | EPL_VOS_ENABLE | EPL_DOC_ON);
    }

    als_frame_time = enh_time;
    ps_frame_time = ps_time;

    epl_sensor_I2C_Write(client, 0x00, epl_sensor.wait | epl_sensor.mode);

    if(cmp_flag == true)
    {
        //PS unlock and run
        epl_sensor.ps.compare_reset = EPL_CMP_RUN;
        epl_sensor.ps.lock = EPL_UN_LOCK;
        epl_sensor_I2C_Write(client, 0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);
        cmp_flag = false;
    }

    //mutex_unlock(&sensor_mutex);

    total_time = ps_time+enh_time+wait_value[epl_sensor.wait>>4];
    if(2*total_time >= dynk_polling_delay)
    {
        dynk_polling_delay = 2*total_time+50;
        APS_LOG("[%s]: dynk_polling_delay=%d \r\n", __func__, dynk_polling_delay);
    }
    if(epl_sensor.als.polling_mode == 1)
    {
        msleep(total_time);
        APS_LOG("[%s] PS+ALS(%dms)\r\n", __func__, ps_time+enh_time+wait_value[epl_sensor.wait>>4]);
    }

    if(enable == true)
    {
        if(epl_sensor.ps.polling_mode == 1)
        {
            epl_sensor_read_ps(client);
        }
        epl_sensor_read_als(client);
    }

    if(epl_sensor.ps.interrupt_flag == EPL_INT_TRIGGER)
    {
        //PS unlock and run
        epl_sensor.ps.compare_reset = EPL_CMP_RUN;
        epl_sensor.ps.lock = EPL_UN_LOCK;
        epl_sensor_I2C_Write(client, 0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);
#if defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
        mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
        mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif
    }

    if(epl_sensor.als.interrupt_flag == EPL_INT_TRIGGER && epl_sensor.als.polling_mode == 0)
    {
        //ALS unlock and run
       	epl_sensor.als.compare_reset = EPL_CMP_RUN;
    	epl_sensor.als.lock = EPL_UN_LOCK;
    	epl_sensor_I2C_Write(client, 0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
#if defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
        mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
        mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif
    }

    polling_flag = true;
}
#endif

void epl_sensor_restart_dynk_polling(void)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    cancel_delayed_work(&obj->dynk_thd_polling_work);
    schedule_delayed_work(&obj->dynk_thd_polling_work, msecs_to_jiffies(2*dynk_polling_delay));
}

void epl_sensor_dynk_thd_polling_work(struct work_struct *work)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;

    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;

    APS_LOG("[%s]:als / ps enable: %d / %d\n", __func__,enable_als, enable_ps);

    if(enable_ps == true)
    {
#if PS_DYN_K_STR
        dynk_enhance_flag = false;
        if(polling_flag == true && eint_flag == true)
        {
            epl_sensor_enhance_enable(obj->client, true);
        }
#endif
        if(polling_flag == true && eint_flag == true && epl_sensor.ps.polling_mode == 0)
        {
            mutex_lock(&sensor_mutex);
            epl_sensor_read_ps_status(obj->client);
            epl_sensor_read_ps(obj->client);
            mutex_unlock(&sensor_mutex);
        }
#if PS_DYN_K_STR
        if( (dynk_min_ps_raw_data > epl_sensor.ps.data.data)
            && (epl_sensor.ps.saturation == 0)
            && (epl_sensor.ps.data.ir_data < dynk_max_ir_data)
            && (dynk_enhance_ch0 < dynk_enhance_max_ch0) )
#else
        if( (dynk_min_ps_raw_data > epl_sensor.ps.data.data)
            && (epl_sensor.ps.saturation == 0)
            && (epl_sensor.ps.data.ir_data < dynk_max_ir_data) )
#endif
        {
            dynk_min_ps_raw_data = epl_sensor.ps.data.data;
            dynk_thd_low = dynk_min_ps_raw_data + dynk_low_offset;
		    dynk_thd_high = dynk_min_ps_raw_data + dynk_high_offset;

            if(dynk_thd_low>65534)
                dynk_thd_low = 65534;
            if(dynk_thd_high>65535)
                dynk_thd_high = 65535;

		    APS_LOG("[%s]:dyn ps raw = %d, min = %d, ir_data = %d\n", __func__, epl_sensor.ps.data.data, dynk_min_ps_raw_data, epl_sensor.ps.data.ir_data);
            eint_flag = false;
            mutex_lock(&sensor_mutex);
		    set_psensor_intr_threshold((u16)dynk_thd_low, (u16)dynk_thd_high);
            mutex_unlock(&sensor_mutex);
            eint_flag = true;
		    APS_LOG("[%s]:dyn k thre_l = %d, thre_h = %d\n", __func__, dynk_thd_low, dynk_thd_high);
        }
        else if(dynk_change_flag==true && (epl_sensor.ps.data.data>dynk_change_thd_max) && ( (epl_sensor.ps.compare_low >> 3)==0 ))
        {
            dynk_change_flag = false;
            dynk_thd_low += dynk_thd_offset;
	        dynk_thd_high += dynk_thd_offset;

	        if(dynk_thd_low>65534)
                dynk_thd_low = 65534;
            if(dynk_thd_high>65535)
                dynk_thd_high = 65535;

            eint_flag = false;
            mutex_lock(&sensor_mutex);
            set_psensor_intr_threshold((u16)dynk_thd_low, (u16)dynk_thd_high);
            mutex_unlock(&sensor_mutex);
            eint_flag = true;
            APS_LOG("[%s]: epl_sensor.ps.data.data=%d, L/H=%d/%d \r\n", __func__, epl_sensor.ps.data.data, dynk_thd_low, dynk_thd_high);
        }
#if PS_DYN_K_STR
        if(polling_flag == true && eint_flag == true)
        {
            epl_sensor_enhance_enable(obj->client, false);
        }
        dynk_enhance_flag = true;
#endif
        schedule_delayed_work(&obj->dynk_thd_polling_work, msecs_to_jiffies(dynk_polling_delay));
    }

}
#endif
/************************************************************************/
//for 3637
static int als_sensing_time(int intt, int adc, int cycle)
{
    long sensing_us_time;
    int sensing_ms_time;
    int als_intt, als_adc, als_cycle;

    als_intt = als_intt_value[intt>>2];
    als_adc = adc_value[adc>>3];
    als_cycle = cycle_value[cycle];
#if COMMON_DEBUG
    APS_LOG("ALS: INTT=%d, ADC=%d, Cycle=%d \r\n", als_intt, als_adc, als_cycle);
#endif

    sensing_us_time = (als_intt + als_adc*2*2) * 2 * als_cycle;
    sensing_ms_time = sensing_us_time / 1000;

#if COMMON_DEBUG
    APS_LOG("[%s]: sensing=%d ms \r\n", __func__, sensing_ms_time);
#endif
    return (sensing_ms_time + 5);
}

static int ps_sensing_time(int intt, int adc, int cycle)
{
    long sensing_us_time;
    int sensing_ms_time;
    int ps_intt, ps_adc, ps_cycle;

    ps_intt = ps_intt_value[intt>>2];
    ps_adc = adc_value[adc>>3];
    ps_cycle = cycle_value[cycle];
#if COMMON_DEBUG
    APS_LOG("PS: INTT=%d, ADC=%d, Cycle=%d \r\n", ps_intt, ps_adc, ps_cycle);
#endif

    sensing_us_time = (ps_intt*3 + ps_adc*2*3) * ps_cycle;
    sensing_ms_time = sensing_us_time / 1000;
#if COMMON_DEBUG
    APS_LOG("[%s]: sensing=%d ms\r\n", __func__, sensing_ms_time);
#endif


    return (sensing_ms_time + 5);
}

static int epl_sensor_get_wait_time(int ps_time, int als_time)
{
    int wait_idx = 0;
    int wait_time = 0;

    wait_time = als_time - ps_time;
    if(wait_time < 0){
        wait_time = 0;
    }
#if COMMON_DEBUG
    APS_LOG("[%s]: wait_len = %d \r\n", __func__, wait_len);
#endif
    for(wait_idx = 0; wait_idx < wait_len; wait_idx++)
	{
	    if(wait_time < wait_value[wait_idx])
	    {
	        break;
	    }
	}
	if(wait_idx >= wait_len){
        wait_idx = wait_len - 1;
	}

#if COMMON_DEBUG
	APS_LOG("[%s]: wait_idx = %d, wait = %dms \r\n", __func__, wait_idx, wait_value[wait_idx]);
#endif
	return (wait_idx << 4);
}
/************************************************************************/
#if HS_ENABLE || PS_GES
void epl_sensor_restart_polling(void)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    int queue_flag = work_busy(&obj->polling_work);
    cancel_delayed_work(&obj->polling_work);
    APS_LOG("[%s]: queue_flag=%d \r\n", __func__, queue_flag);
    if(queue_flag == 0)
    {
        schedule_delayed_work(&obj->polling_work, msecs_to_jiffies(50));
    }
    else
    {
        schedule_delayed_work(&obj->polling_work, msecs_to_jiffies(200));
    }
}

void epl_sensor_polling_work(struct work_struct *work)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
#if HS_ENABLE
    bool enable_hs = test_bit(CMC_BIT_HS, &obj->enable) && atomic_read(&obj->hs_suspend)==0;
    APS_LOG("[%s]: enable_hs=%d \r\n", __func__, enable_hs);
#endif
#if PS_GES
    bool enable_ges = test_bit(CMC_BIT_GES, &obj->enable) && atomic_read(&obj->ges_suspend)==0;
    APS_LOG("[%s]: enable_ges=%d \r\n", __func__, enable_ges);
#endif

    cancel_delayed_work(&obj->polling_work);

#if HS_ENABLE
    if(enable_hs)
    {
		schedule_delayed_work(&obj->polling_work, msecs_to_jiffies(20));
        epl_sensor_read_hs(obj->client);
    }
#if PS_GES  /*PS_GES*/
    else if (enable_ges && epl_sensor.ges.polling_mode==1)
#endif /*PS_GES*/
#endif

#if PS_GES
#if !HS_ENABLE /*HS_ENABLE*/
    if (enable_ges && epl_sensor.ges.polling_mode==1)
#endif  /*HS_ENABLE*/
    {
        schedule_delayed_work(&obj->polling_work, msecs_to_jiffies(200));
        epl_sensor_read_ps_status(obj->client);
    }
#endif

}
#endif

void epl_sensor_update_mode(struct i2c_client *client)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    int als_time = 0, ps_time = 0;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
#if HS_ENABLE
	bool enable_hs = test_bit(CMC_BIT_HS, &obj->enable) && atomic_read(&obj->hs_suspend)==0;
#endif
#if PS_GES
    bool enable_ges = test_bit(CMC_BIT_GES, &obj->enable) && atomic_read(&obj->ges_suspend)==0;
#endif
    als_time = als_sensing_time(epl_sensor.als.integration_time, epl_sensor.als.adc, epl_sensor.als.cycle);
    ps_time = ps_sensing_time(epl_sensor.ps.integration_time, epl_sensor.ps.adc, epl_sensor.ps.cycle);

    als_frame_time = als_time;
    ps_frame_time = ps_time;

    polling_flag = false;

#if PS_GES
    APS_LOG("mode selection =0x%x\n", (enable_ps|enable_ges) | (enable_als << 1));
#else
	APS_LOG("mode selection =0x%x\n", enable_ps | (enable_als << 1));
#endif

#if HS_ENABLE
    if(enable_hs)
    {
        APS_LOG("[%s]: HS mode \r\n", __func__);
        epl_sensor_restart_polling();
    }
	else
#endif

	{   /*PS_GES and HS_ENABLE*/

        //PS unlock and reset
        epl_sensor.ps.compare_reset = EPL_CMP_RESET;
        epl_sensor.ps.lock = EPL_UN_LOCK;
        epl_sensor_I2C_Write(obj->client, 0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);

        //ALS unlock and reset
        epl_sensor.als.compare_reset = EPL_CMP_RESET;
    	epl_sensor.als.lock = EPL_UN_LOCK;
    	epl_sensor_I2C_Write(obj->client, 0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);

        epl_sensor.ps.interrupt_flag = EPL_INT_CLEAR;
        epl_sensor.als.interrupt_flag = EPL_INT_CLEAR;
	    //epl_sensor_I2C_Write(client, 0x11, EPL_RESETN_RESET | EPL_POWER_OFF);
#if PS_GES
        //**** mode selection ****
        switch((enable_als << 1) | (enable_ps|enable_ges))
#else
        //**** mode selection ****
        switch((enable_als << 1) | enable_ps)
#endif

        {
            case 0: //disable all
                epl_sensor.mode = EPL_MODE_IDLE;
                break;

            case 1: //als = 0, ps = 1
#if PS_DYN_K && PS_DYN_K_STR
                epl_sensor.mode = EPL_MODE_ALS_PS;
#else
                epl_sensor.mode = EPL_MODE_PS;
#endif
                break;

            case 2: //als = 1, ps = 0
                epl_sensor.mode = EPL_MODE_ALS;
                break;

            case 3: //als = 1, ps = 1
                epl_sensor.mode = EPL_MODE_ALS_PS;
                break;
        }

        //**** write setting ****
        // step 1. set sensor at idle mode
        // step 2. uplock and reset als / ps status
        // step 3. set sensor at operation mode
        // step 4. delay sensing time
        // step 5. unlock and run als / ps status

        epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | EPL_MODE_IDLE);

        // initial factory calibration variable
        read_factory_calibration();


        epl_sensor_I2C_Write(client, 0x02, epl_sensor.als.adc | epl_sensor.als.cycle);
        set_als_ps_intr_type(client, epl_sensor.ps.polling_mode, epl_sensor.als.polling_mode);

        epl_sensor_I2C_Write(obj->client, 0x06, epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type);

        epl_sensor_I2C_Write(obj->client, 0x07, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);

#if ALS_DYN_INTT
        if(epl_sensor.als.report_type == CMC_BIT_DYN_INT){
    	    epl_sensor_I2C_Write(client, 0x01, epl_sensor.als.integration_time | epl_sensor.als.gain);
        }
#endif

        if(enable_als == true && enable_ps == false)
        {
            epl_sensor_I2C_Write(client, 0xfc, EPL_A_D | EPL_NORMAL | EPL_GFIN_ENABLE | EPL_VOS_ENABLE | EPL_DOC_ON);
        }

        if(epl_sensor.mode == EPL_MODE_ALS_PS && epl_sensor.als.polling_mode == 0 && epl_sensor.ps.polling_mode == 0){
            int wait = 0;
            wait = epl_sensor_get_wait_time(ps_time, als_time);
            epl_sensor_I2C_Write(client, 0x00, wait | epl_sensor.mode);
            epl_sensor.wait = wait;
            APS_LOG("[%s]: epl_sensor.als.polling_mode=%d \r\n", __func__, epl_sensor.als.polling_mode);
        }
        else{
#if PS_GES
            if(enable_ges == 1 && enable_ps == 0)
            {
                set_psensor_intr_threshold(epl_sensor.ges.low_threshold, epl_sensor.ges.high_threshold);
                epl_sensor_I2C_Write(client, 0x00, EPL_WAIT_2_MS | epl_sensor.mode);
            }
            else
#endif
                epl_sensor_I2C_Write(client, 0x00, epl_sensor.wait | epl_sensor.mode);
        }

        //PS unlock and run
        epl_sensor.ps.compare_reset = EPL_CMP_RUN;
        epl_sensor.ps.lock = EPL_UN_LOCK;
        epl_sensor_I2C_Write(obj->client, 0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);

        //ALS unlock and run
       	epl_sensor.als.compare_reset = EPL_CMP_RUN;
    	epl_sensor.als.lock = EPL_UN_LOCK;
    	epl_sensor_I2C_Write(obj->client, 0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);

        //epl_sensor_I2C_Write(client, 0x11, epl_sensor.reset | epl_sensor.power);
#if COMMON_DEBUG
        //**** check setting ****
        if(enable_ps == 1)
        {
            APS_LOG("[%s] PS:low_thd = %d, high_thd = %d \n",__func__, epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
        }

        if(enable_als == 1 && epl_sensor.als.polling_mode == 0)
        {
            APS_LOG("[%s] ALS:low_thd = %d, high_thd = %d \n",__func__, epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
        }
#if PS_GES
        if(enable_ges)
        {
            APS_LOG("[%s] GES:low_thd = %d, high_thd = %d \n",__func__, epl_sensor.ges.low_threshold, epl_sensor.ges.high_threshold);

        }
#endif
    	APS_LOG("[%s] reg0x00= 0x%x\n", __func__,  epl_sensor.wait | epl_sensor.mode);
    	APS_LOG("[%s] reg0x07= 0x%x\n", __func__, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);
    	APS_LOG("[%s] reg0x06= 0x%x\n", __func__, epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type);
    	APS_LOG("[%s] reg0x11= 0x%x\n", __func__, epl_sensor.power | epl_sensor.reset);
    	APS_LOG("[%s] reg0x12= 0x%x\n", __func__, epl_sensor.als.compare_reset | epl_sensor.als.lock);
    	APS_LOG("[%s] reg0x1b= 0x%x\n", __func__, epl_sensor.ps.compare_reset | epl_sensor.ps.lock);
#endif

        if(epl_sensor.mode == EPL_MODE_PS)
        {
            msleep(ps_time);
            APS_LOG("[%s] PS only(%dms)\r\n", __func__, ps_time);
        }
        else if (epl_sensor.mode == EPL_MODE_ALS)
        {
            msleep(als_time);
            APS_LOG("[%s] ALS only(%dms)\r\n", __func__, als_time);
        }
        else if (epl_sensor.mode == EPL_MODE_ALS_PS && epl_sensor.als.polling_mode == 1)
        {
            msleep(ps_time+als_time+wait_value[epl_sensor.wait>>4]);
            APS_LOG("[%s] PS+ALS(%dms)\r\n", __func__, ps_time+als_time+wait_value[epl_sensor.wait>>4]);
        }

        if(epl_sensor.ps.interrupt_flag == EPL_INT_TRIGGER)
        {
            //PS unlock and run
            epl_sensor.ps.compare_reset = EPL_CMP_RUN;
            epl_sensor.ps.lock = EPL_UN_LOCK;
            epl_sensor_I2C_Write(obj->client, 0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);
#if defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
            mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
            mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif
        }

        if(epl_sensor.als.interrupt_flag == EPL_INT_TRIGGER)
        {
            //ALS unlock and run
           	epl_sensor.als.compare_reset = EPL_CMP_RUN;
        	epl_sensor.als.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(obj->client, 0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
#if defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
            mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
            mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif
        }
    }   /*PS_GES and HS_ENABLE*/
#if PS_DYN_K
    if(enable_ps == 1)
    {
        epl_sensor_restart_dynk_polling();
    }
#endif

#if PS_GES
    if(enable_ges == 1 && epl_sensor.ges.polling_mode == 1)
    {
        epl_sensor_restart_polling();
    }
#endif

    polling_flag = true;

#if PS_DYN_K_STR
    if(enable_als)
    {
        APS_LOG("[%s]: ALS mode is enabled ! \r\n", __func__);
        dynk_enhance_flag = true;
    }
#endif
}


/*----------------------------------------------------------------------------*/
void epl_sensor_eint_func(void)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;

    // APS_LOG(" interrupt fuc\n");

    if(!obj)
    {
        return;
    }
#if defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
    mt_eint_mask(CUST_EINT_ALS_NUM);
#else
    mt65xx_eint_mask(CUST_EINT_ALS_NUM);
#endif
    schedule_delayed_work(&obj->eint_work, 0);
}

static void epl_sensor_report_ps_status(void)
{
    int err;
#if MTK_LTE
    int ps_state;
#else
    hwm_sensor_data sensor_data;
#endif

#if PS_DEBUG
    APS_FUN();
#endif

#if MTK_LTE
    ps_state = epl_sensor.ps.compare_low >> 3;
    APS_LOG("[%s]: epl_sensor.ps.data.data=%d, ps_status=%d \r\n", __func__, epl_sensor.ps.data.data, ps_state);
    err = ps_report_interrupt_data(ps_state);
    if(err != 0)
    {
        APS_ERR("epl_sensor_eint_work err: %d\n", err);
    }
#else
    sensor_data.values[0] = epl_sensor.ps.compare_low >> 3;
#if PS_DEBUG
	sensor_data.values[1] = epl_sensor.ps.data.data;
#else
    sensor_data.values[1] = 0;
#endif
	sensor_data.value_divide = 1;
	sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
    APS_LOG("[%s]: epl_sensor.ps.data.data=%d, ps_status=%d \r\n", __func__, epl_sensor.ps.data.data, sensor_data.values[0]);
	if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
	{
	    APS_ERR("get interrupt data failed\n");
	    APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
	}
#endif
}

static void epl_sensor_intr_als_report_lux(void)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
#if MTK_LTE
    u16 lux;
#else
    hwm_sensor_data sensor_data;
#endif
    int err;

    APS_LOG("[%s]: IDEL MODE \r\n", __func__);
    epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | EPL_MODE_IDLE);

    epl_sensor_read_als(obj->client);
#if MTK_LTE
    lux = epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);
    //if((err = als_report_interrupt_data(lux)))
    {
        APS_ERR("epl_sensor call als_report_interrupt_data fail = %d\n", err);
    }
    APS_LOG("[%s]: ALS Lux=%d \r\n", __func__, lux);
#else
    sensor_data.values[0] = epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);
	sensor_data.values[1] = epl_sensor.als.data.channels[1];
	sensor_data.value_divide = 1;
	sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
#endif
	epl_sensor.als.compare_reset = EPL_CMP_RESET;
	epl_sensor.als.lock = EPL_UN_LOCK;
	epl_sensor_I2C_Write(obj->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);

	//set dynamic threshold
	if(epl_sensor.als.compare_high >> 4)
	{
		epl_sensor.als.high_threshold = epl_sensor.als.high_threshold + 250;
		epl_sensor.als.low_threshold = epl_sensor.als.low_threshold + 250;

		if (epl_sensor.als.high_threshold > 60000)
		{
			epl_sensor.als.high_threshold = epl_sensor.als.high_threshold - 250;
			epl_sensor.als.low_threshold = epl_sensor.als.low_threshold - 250;
		}
	}
	if(epl_sensor.als.compare_low>> 3)
	{
		epl_sensor.als.high_threshold = epl_sensor.als.high_threshold - 250;
		epl_sensor.als.low_threshold = epl_sensor.als.low_threshold - 250;

		if (epl_sensor.als.high_threshold < 250)
		{
			epl_sensor.als.high_threshold = epl_sensor.als.high_threshold + 250;
			epl_sensor.als.low_threshold = epl_sensor.als.low_threshold + 250;
		}
	}

    if(epl_sensor.als.high_threshold < epl_sensor.als.low_threshold)
	{
	    APS_LOG("[%s]:recover default setting \r\n", __FUNCTION__);
	    epl_sensor.als.high_threshold = obj->hw->als_threshold_high;
	    epl_sensor.als.low_threshold = obj->hw->als_threshold_low;
	}
#if !MTK_LTE
	if((err = hwmsen_get_interrupt_data(ID_LIGHT, &sensor_data)))
	{
	    APS_ERR("get interrupt data failed\n");
	    APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
	}
#endif
	//write new threshold
	set_lsensor_intr_threshold(epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);

    epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | epl_sensor.mode);
    APS_LOG("[%s]: MODE=0x%x \r\n", __func__, epl_sensor.mode);
}

/*----------------------------------------------------------------------------*/
static void epl_sensor_eint_work(struct work_struct *work)
{
    hwm_sensor_data sensor_data;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
#if PS_GES
    bool enable_ges = test_bit(CMC_BIT_GES, &obj->enable) && atomic_read(&obj->ges_suspend)==0;
#endif
    bool isWork;
    int err;

    eint_flag = false;
    //mutex_lock(&sensor_mutex);
    APS_LOG("xxxxxxxxxxx\n\n");
#if PS_GES
    if(enable_ges && epl_sensor.ges.polling_mode == 0)
    {
        mutex_lock(&sensor_mutex);
        epl_sensor_read_ps_status(obj->client);
        mutex_unlock(&sensor_mutex);
        //if(polling_flag == true)
        //{
            //GES unlock and run
    		epl_sensor.ges.compare_reset = EPL_CMP_RUN;
    		epl_sensor.ges.lock = EPL_UN_LOCK;
    		epl_sensor_I2C_Write(obj->client,0x1b, epl_sensor.ges.compare_reset | epl_sensor.ges.lock);
		//}
    }
#endif
    if(enable_ps && epl_sensor.ps.polling_mode == 0)
    {
        mutex_lock(&sensor_mutex);
        epl_sensor_read_ps_status(obj->client);
        mutex_unlock(&sensor_mutex);
        if(epl_sensor.ps.interrupt_flag == EPL_INT_TRIGGER && !obj->hw->polling_mode_ps)
        {
#if PS_DEBUG || PS_DYN_K
            mutex_lock(&sensor_mutex);
		    epl_sensor_read_ps(obj->client);
		    mutex_unlock(&sensor_mutex);
#endif
            wake_lock_timeout(&ps_lock, 2*HZ);
            epl_sensor_report_ps_status();
#if PS_DYN_K
            //epl_sensor_reset_dynk_thd(dynk_last_status, (epl_sensor.ps.compare_low >> 3));
            dynk_last_status = (epl_sensor.ps.compare_low >> 3);
#endif
            //if(polling_flag == true)
            //{
                //PS unlock and run
        		epl_sensor.ps.compare_reset = EPL_CMP_RUN;
        		epl_sensor.ps.lock = EPL_UN_LOCK;
        		epl_sensor_I2C_Write(obj->client,0x1b, epl_sensor.ps.compare_reset | epl_sensor.ps.lock);
            //}
        }
#if PS_DYN_K_STR
        if(enable_als == 0 && epl_sensor.als.polling_mode == 0)
        {
            //ALS unlock and run
    		epl_sensor.als.compare_reset = EPL_CMP_RUN;
    		epl_sensor.als.lock = EPL_UN_LOCK;
    		epl_sensor_I2C_Write(obj->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
        }
#endif
    }
    if(enable_als && obj->hw->polling_mode_als == 0)
    {
        mutex_lock(&sensor_mutex);
        epl_sensor_read_als_status(obj->client);
        mutex_unlock(&sensor_mutex);
        if(epl_sensor.als.interrupt_flag == EPL_INT_TRIGGER && !obj->hw->polling_mode_als)
        {
            epl_sensor_intr_als_report_lux();
            //if(polling_flag == true)
            //{
                //ALS unlock and run
        		epl_sensor.als.compare_reset = EPL_CMP_RUN;
        		epl_sensor.als.lock = EPL_UN_LOCK;
        		epl_sensor_I2C_Write(obj->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
            //}
        }
    }

    //mutex_unlock(&sensor_mutex);
    eint_flag = true;
    //if(polling_flag == true)
    //{
#if defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
        mt_eint_unmask(CUST_EINT_ALS_NUM);
#else
        mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif
    //}

}



/*----------------------------------------------------------------------------*/
int epl_sensor_setup_eint(struct i2c_client *client)
{
    APS_LOG("epl_sensor_setup_eint\n");

    /*configure to GPIO function, external interrupt*/

    mt_set_gpio_mode(GPIO_ALS_EINT_PIN, GPIO_ALS_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_ALS_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_ALS_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_ALS_EINT_PIN, GPIO_PULL_UP);

#if defined(MT6575) || defined(MT6571) || defined(MT6589)
    mt65xx_eint_set_sens(CUST_EINT_ALS_NUM, CUST_EINT_EDGE_SENSITIVE);
    mt65xx_eint_set_polarity(CUST_EINT_ALS_NUM, CUST_EINT_ALS_POLARITY);
    mt65xx_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
    mt65xx_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_EN, CUST_EINT_POLARITY_LOW, epl_sensor_eint_func, 0);
    mt65xx_eint_unmask(CUST_EINT_ALS_NUM);
#endif

#if defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
	mt_eint_set_hw_debounce(CUST_EINT_ALS_NUM, CUST_EINT_ALS_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_ALS_NUM, CUST_EINT_ALS_TYPE, epl_sensor_eint_func, 0);

	mt_eint_unmask(CUST_EINT_ALS_NUM);
#endif

    return 0;
}




/*----------------------------------------------------------------------------*/
static int epl_sensor_init_client(struct i2c_client *client)
{
    struct epl_sensor_priv *obj = i2c_get_clientdata(client);
    int err=0;

    APS_LOG("I2C Addr==[0x%x],line=%d\n", epl_sensor_i2c_client->addr, __LINE__);

    /*  interrupt mode */


    APS_FUN();

    if(obj->hw->polling_mode_ps == 0)
    {
#if defined(MT6582) || defined(MT6592) || defined(MT6752) || defined(MT6735) || defined(MT6753)
        mt_eint_mask(CUST_EINT_ALS_NUM);
#else
        mt65xx_eint_mask(CUST_EINT_ALS_NUM);
#endif

        if((err = epl_sensor_setup_eint(client)))
        {
            APS_ERR("setup eint: %d\n", err);
            return err;
        }
        APS_LOG("epl_sensor interrupt setup\n");
    }


    if((err = epl259x_hw8k_init_device(client)) != 0)
    {
        APS_ERR("init dev: %d\n", err);
        return err;
    }


    /*  interrupt mode */
    //if(obj->hw->polling_mode_ps == 0)
    //     mt65xx_eint_unmask(CUST_EINT_ALS_NUM);

    return err;
}


/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_show_reg(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct i2c_client *client = epl_sensor_obj->client;

    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x00 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x00));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x01 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x01));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x02 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x02));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x03 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x03));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x04 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x04));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x05 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x05));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x06 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x06));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x07 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x07));
    if(epl_sensor.als.polling_mode == 0)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x08 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x08));
        len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x09 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x09));
        len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0A value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0A));
        len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0B value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0B));
    }
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0C value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0C));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0D value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0D));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0E value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0E));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0F value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x0F));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x11 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x11));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x12 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x12));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x13 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x13));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x14 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x14));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x15 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x15));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x16 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x16));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1B value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1B));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1c value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1C));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1d value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1D));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1e value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1E));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x1f value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x1F));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x22 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x22));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x23 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x23));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x24 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x24));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x25 value = 0x%x\n", i2c_smbus_read_byte_data(client, 0x25));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0xFC value = 0x%x\n", i2c_smbus_read_byte_data(client, 0xFC));

    return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_show_status(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct epl_sensor_priv *epld = epl_sensor_obj;

    bool enable_ps = test_bit(CMC_BIT_PS, &epld->enable) && atomic_read(&epld->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &epld->enable) && atomic_read(&epld->als_suspend)==0;
#if HS_ENABLE
	bool enable_hs = test_bit(CMC_BIT_HS, &epld->enable) && atomic_read(&epld->hs_suspend)==0;
#endif
    if(!epl_sensor_obj)
    {
        APS_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }

    len += snprintf(buf+len, PAGE_SIZE-len, "chip is %s, ver is %s \n", EPL_DEV_NAME, DRIVER_VERSION);
    len += snprintf(buf+len, PAGE_SIZE-len, "als/ps polling is %d-%d\n", epl_sensor.als.polling_mode, epl_sensor.ps.polling_mode);
    len += snprintf(buf+len, PAGE_SIZE-len, "wait = %d, mode = %d\n",epl_sensor.wait >> 4, epl_sensor.mode);
    len += snprintf(buf+len, PAGE_SIZE-len, "interrupt control = %d\n", epl_sensor.interrupt_control >> 4);
    len += snprintf(buf+len, PAGE_SIZE-len, "frame time ps=%dms, als=%dms\n", ps_frame_time, als_frame_time);
#if HS_ENABLE
    if(enable_hs)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "hs adc= %d\n", epl_sensor.hs.adc>>3);
        len += snprintf(buf+len, PAGE_SIZE-len, "hs int_time= %d\n", epl_sensor.hs.integration_time>>2);
        len += snprintf(buf+len, PAGE_SIZE-len, "hs cycle= %d\n", epl_sensor.hs.cycle);
        len += snprintf(buf+len, PAGE_SIZE-len, "hs gain= %d\n", epl_sensor.hs.gain);
        len += snprintf(buf+len, PAGE_SIZE-len, "hs ch1 raw= %d\n", epl_sensor.hs.raw);
    }
#endif
    if(enable_ps)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "PS: \n");
        len += snprintf(buf+len, PAGE_SIZE-len, "INTEG = %d, gain = %d\n", epl_sensor.ps.integration_time >> 2, epl_sensor.ps.gain);
        len += snprintf(buf+len, PAGE_SIZE-len, "ADC = %d, cycle = %d, ir drive = %d\n", epl_sensor.ps.adc >> 3, epl_sensor.ps.cycle, epl_sensor.ps.ir_drive);
        len += snprintf(buf+len, PAGE_SIZE-len, "saturation = %d, int flag = %d\n", epl_sensor.ps.saturation >> 5, epl_sensor.ps.interrupt_flag >> 2);
        len += snprintf(buf+len, PAGE_SIZE-len, "Thr(L/H) = (%d/%d)\n", epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
#if PS_DYN_K
#if PS_DYN_K_STR
        len += snprintf(buf+len, PAGE_SIZE-len, "Dyn enhance ch0 = %d \n", dynk_enhance_ch0);
#endif
        len += snprintf(buf+len, PAGE_SIZE-len, "Dyn thr(L/H) = (%d/%d)\n", dynk_thd_low, dynk_thd_high);
#endif
        len += snprintf(buf+len, PAGE_SIZE-len, "pals data = %d, data = %d\n", epl_sensor.ps.data.ir_data, epl_sensor.ps.data.data);
    }
    if(enable_als)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "ALS: \n");
        len += snprintf(buf+len, PAGE_SIZE-len, "INTEG = %d, gain = %d\n", epl_sensor.als.integration_time >> 2, epl_sensor.als.gain);
        len += snprintf(buf+len, PAGE_SIZE-len, "ADC = %d, cycle = %d\n", epl_sensor.als.adc >> 3, epl_sensor.als.cycle);
#if ALS_DYN_INTT
    if(epl_sensor.als.lsrc_type != CMC_BIT_LSRC_NON)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "lsource_thd_low=%d, lsource_thd_high=%d \n", epld->lsource_thd_low, epld->lsource_thd_high);
        len += snprintf(buf+len, PAGE_SIZE-len, "saturation = %d\n", epl_sensor.als.saturation >> 5);

        if(epl_sensor.als.lsrc_type == CMC_BIT_LSRC_SCALE || epl_sensor.als.lsrc_type == CMC_BIT_LSRC_BOTH)
        {
            len += snprintf(buf+len, PAGE_SIZE-len, "real_ratio = %d\n", epld->ratio);
            len += snprintf(buf+len, PAGE_SIZE-len, "use_ratio = %d\n", epld->last_ratio);
        }
        else if(epl_sensor.als.lsrc_type == CMC_BIT_LSRC_SLOPE)
        {
            len += snprintf(buf+len, PAGE_SIZE-len, "ratio = %d\n", epld->ratio);
        }
    }
    if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "c_gain = %d\n", c_gain);
        len += snprintf(buf+len, PAGE_SIZE-len, "dynamic_intt_lux = %d\n", dynamic_intt_lux);
    }
#endif
    if(epl_sensor.als.polling_mode == 0)
        len += snprintf(buf+len, PAGE_SIZE-len, "Thr(L/H) = (%d/%d)\n", epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
    len += snprintf(buf+len, PAGE_SIZE-len, "ch0 = %d, ch1 = %d\n", epl_sensor.als.data.channels[0], epl_sensor.als.data.channels[1]);
    }

    return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_als_enable(struct device_driver *ddri, const char *buf, size_t count)
{
    uint16_t mode=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
    APS_FUN();

    sscanf(buf, "%hu", &mode);

    if(enable_als != mode)
    {
        if(mode)
        {
            set_bit(CMC_BIT_ALS, &obj->enable);
#if ALS_DYN_INTT
            if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
            {
                dynamic_intt_idx = dynamic_intt_init_idx;
                epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
                epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
                dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
                dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
            }
#endif
        }
        else
        {
            clear_bit(CMC_BIT_ALS, &obj->enable);
        }
        epl_sensor_update_mode(obj->client);
    }

    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_ps_enable(struct device_driver *ddri, const char *buf, size_t count)
{
    uint16_t mode=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;

    APS_FUN();

    sscanf(buf, "%hu", &mode);
    if(enable_ps != mode)
    {
        if(mode)
        {
            //wake_lock(&ps_lock);
            set_bit(CMC_BIT_PS, &obj->enable);
#if PS_DYN_K
            set_psensor_intr_threshold(65534, 65535);   // dont use first ps status
            dynk_min_ps_raw_data = 0xffff;
            dynk_change_flag = false;
#if PS_DYN_K_STR
            dynk_enhance_flag = false;
#endif

#endif
        }
        else
        {
            clear_bit(CMC_BIT_PS, &obj->enable);
#if PS_DYN_K
            cancel_delayed_work(&obj->dynk_thd_polling_work);
#endif
            //wake_unlock(&ps_lock);
        }
        epl_sensor_update_mode(obj->client);
    }
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_show_cal_raw(struct device_driver *ddri, char *buf)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    u16 ch1=0;
    u32 ch1_all=0;
    int count =5;
    int i;
    ssize_t len = 0;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;

    if(!epl_sensor_obj)
    {
        APS_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }

    for(i=0; i<count; i++)
    {
        msleep(50);
        switch(epl_sensor.mode)
        {
#if PS_DYN_K_STR
            case EPL_MODE_ALS_PS:
#else
            case EPL_MODE_PS:
#endif
#if !PS_DYN_K
                if(enable_ps == true && polling_flag == true && eint_flag == true && epl_sensor.ps.polling_mode == 0)
                    epl_sensor_read_ps(obj->client);
#endif
                ch1 = epl_sensor.ps.data.data;
                break;

            case EPL_MODE_ALS:
                if(enable_als == true && polling_flag == true && eint_flag == true && epl_sensor.als.polling_mode == 0)
                    epl_sensor_read_als(obj->client);
                ch1 = epl_sensor.als.data.channels[1];
                break;
        }

        ch1_all = ch1_all + ch1;
        if(epl_sensor.wait == EPL_WAIT_SINGLE)
            epl_sensor_I2C_Write(obj->client, 0x11,  epl_sensor.power | epl_sensor.reset);
    }

    ch1 = (u16)ch1_all/count;

    APS_LOG("cal_raw = %d \r\n" , ch1);

    len += snprintf(buf + len, PAGE_SIZE - len, "%d \r\n", ch1);

    return  len;
}


/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_threshold(struct device_driver *ddri,const char *buf, size_t count)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    int low, high;
    APS_FUN();
    if(!obj)
    {
        APS_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }

    sscanf(buf, "%d,%d", &low, &high);

    switch(epl_sensor.mode)
    {
#if PS_DYN_K_STR
        case EPL_MODE_ALS_PS:
#else
        case EPL_MODE_PS:
#endif
            obj->hw->ps_threshold_low = low;
            obj->hw->ps_threshold_high = high;
            epl_sensor.ps.low_threshold = low;
            epl_sensor.ps.high_threshold = high;
            set_psensor_intr_threshold(epl_sensor.ps.low_threshold, epl_sensor.ps.high_threshold);
            break;

        case EPL_MODE_ALS:
            obj->hw->als_threshold_low = low;
            obj->hw->als_threshold_high = high;
            epl_sensor.als.low_threshold = low;
            epl_sensor.als.high_threshold = high;
            set_lsensor_intr_threshold(epl_sensor.als.low_threshold, epl_sensor.als.high_threshold);
            break;

    }

    return  count;
}

static ssize_t epl_sensor_show_threshold(struct device_driver *ddri, char *buf)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    ssize_t len = 0;

    if(!obj)
    {
        APS_ERR("epl_sensor_obj is null!!\n");
        return 0;
    }

    switch(epl_sensor.mode)
    {
#if PS_DYN_K_STR
        case EPL_MODE_ALS_PS:
#else
        case EPL_MODE_PS:
#endif
            len += snprintf(buf + len, PAGE_SIZE - len, "obj->hw->ps_threshold_low=%d \r\n", obj->hw->ps_threshold_low);
            len += snprintf(buf + len, PAGE_SIZE - len, "obj->hw->ps_threshold_high=%d \r\n", obj->hw->ps_threshold_high);
            break;

        case EPL_MODE_ALS:
            len += snprintf(buf + len, PAGE_SIZE - len, "obj->hw->als_threshold_low=%d \r\n", obj->hw->als_threshold_low);
            len += snprintf(buf + len, PAGE_SIZE - len, "obj->hw->als_threshold_high=%d \r\n", obj->hw->als_threshold_high);
            break;

    }
    return  len;

}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_wait_time(struct device_driver *ddri, const char *buf, size_t count)
{
    int val;

    sscanf(buf, "%d",&val);

    epl_sensor.wait = (val & 0xf) << 4;

    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_gain(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	int value = 0;
    APS_FUN();

    sscanf(buf, "%d", &value);

    value = value & 0x03;

	switch (epl_sensor.mode)
	{
#if PS_DYN_K_STR
        case EPL_MODE_ALS_PS:
#else
        case EPL_MODE_PS:
#endif
            epl_sensor.ps.gain = value;
	        epl_sensor_I2C_Write(epld->client, 0x03, epl_sensor.ps.integration_time | epl_sensor.ps.gain);
		break;

        case EPL_MODE_ALS: //als
            epl_sensor.als.gain = value;
	        epl_sensor_I2C_Write(epld->client, 0x01, epl_sensor.als.integration_time | epl_sensor.als.gain);
		break;

    }

	epl_sensor_update_mode(epld->client);

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_mode(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    int value=0;
    APS_FUN();

    clear_bit(CMC_BIT_PS, &obj->enable);
    clear_bit(CMC_BIT_ALS, &obj->enable);

    sscanf(buf, "%d",&value);

    switch (value)
    {
        case 0:
            epl_sensor.mode = EPL_MODE_IDLE;
            break;

        case 1:
            //set_bit(CMC_BIT_ALS, &obj->enable);
            epl_sensor.mode = EPL_MODE_ALS;
            break;

        case 2:
            //set_bit(CMC_BIT_PS, &obj->enable);
            epl_sensor.mode = EPL_MODE_PS;
            break;

        case 3:
            //set_bit(CMC_BIT_ALS, &obj->enable);
            //set_bit(CMC_BIT_PS, &obj->enable);
            epl_sensor.mode = EPL_MODE_ALS_PS;
            break;
    }

    epl_sensor_update_mode(obj->client);

    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_ir_mode(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d", &value);

    switch (epl_sensor.mode)
    {
#if PS_DYN_K_STR
        case EPL_MODE_ALS_PS:
#else
        case EPL_MODE_PS:
#endif
            switch(value)
            {
                case 0:
                    epl_sensor.ps.ir_mode = EPL_IR_MODE_CURRENT;
                    break;

                case 1:
                    epl_sensor.ps.ir_mode = EPL_IR_MODE_VOLTAGE;
                    break;
            }

            epl_sensor_I2C_Write(obj->client, 0x05, epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive);
         break;
    }

    epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | epl_sensor.mode);

    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_ir_contrl(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    uint8_t  data;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d",&value);

    switch (epl_sensor.mode)
    {
#if PS_DYN_K_STR
        case EPL_MODE_ALS_PS:
#else
        case EPL_MODE_PS:
#endif
        switch(value)
        {
            case 0:
                epl_sensor.ps.ir_on_control = EPL_IR_ON_CTRL_OFF;
                break;

            case 1:
                epl_sensor.ps.ir_on_control = EPL_IR_ON_CTRL_ON;
                break;
        }

        data = epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive;
        APS_LOG("%s: 0x05 = 0x%x\n", __FUNCTION__, data);

        epl_sensor_I2C_Write(obj->client, 0x05, epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive);
         break;
    }

    epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | epl_sensor.mode);

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_ir_drive(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d", &value);

    switch(epl_sensor.mode)
    {
#if PS_DYN_K_STR
        case EPL_MODE_ALS_PS:
#else
        case EPL_MODE_PS:
#endif
        epl_sensor.ps.ir_drive = (value & 0x03);
        epl_sensor_I2C_Write(obj->client, 0x05, epl_sensor.ps.ir_on_control | epl_sensor.ps.ir_mode | epl_sensor.ps.ir_drive);
        break;
    }

    epl_sensor_I2C_Write(obj->client, 0x00, epl_sensor.wait | epl_sensor.mode);

    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_interrupt_type(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d",&value);

    switch (epl_sensor.mode)
    {
#if PS_DYN_K_STR
        case EPL_MODE_ALS_PS:
#else
        case EPL_MODE_PS:
#endif
            if(!obj->hw->polling_mode_ps)
            {
                epl_sensor.ps.interrupt_type = value & 0x03;
                epl_sensor_I2C_Write(obj->client, 0x06, epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type);
                APS_LOG("%s: 0x06 = 0x%x\n", __FUNCTION__, epl_sensor.interrupt_control | epl_sensor.ps.persist |epl_sensor.ps.interrupt_type);
            }
            break;

        case EPL_MODE_ALS: //als
            if(!obj->hw->polling_mode_als)
            {
                epl_sensor.als.interrupt_type = value & 0x03;
                epl_sensor_I2C_Write(obj->client, 0x07, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);
                APS_LOG("%s: 0x07 = 0x%x\n", __FUNCTION__, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);
            }
            break;
    }

    epl_sensor_update_mode(obj->client);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_integration(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d",&value);

    switch (epl_sensor.mode)
    {
#if PS_DYN_K_STR
        case EPL_MODE_ALS_PS:
#else
        case EPL_MODE_PS:
#endif
            epl_sensor.ps.integration_time = (value & 0xf) << 2;
            epl_sensor_I2C_Write(obj->client, 0x03, epl_sensor.ps.integration_time | epl_sensor.ps.gain);
            epl_sensor_I2C_Read(obj->client, 0x03, 1);
            APS_LOG("%s: 0x03 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.ps.integration_time | epl_sensor.ps.gain, gRawData.raw_bytes[0]);
            break;

        case EPL_MODE_ALS: //als
            epl_sensor.als.integration_time = (value & 0xf) << 2;
            epl_sensor_I2C_Write(obj->client, 0x01, epl_sensor.als.integration_time | epl_sensor.als.gain);
            epl_sensor_I2C_Read(obj->client, 0x01, 1);
            APS_LOG("%s: 0x01 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.als.integration_time | epl_sensor.als.gain, gRawData.raw_bytes[0]);
            break;

    }

    epl_sensor_update_mode(obj->client);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_adc(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d",&value);

    switch (epl_sensor.mode)
    {
#if PS_DYN_K_STR
        case EPL_MODE_ALS_PS:
#else
        case EPL_MODE_PS:
#endif
            epl_sensor.ps.adc = (value & 0x3) << 3;
            epl_sensor_I2C_Write(obj->client, 0x04, epl_sensor.ps.adc | epl_sensor.ps.cycle);
            epl_sensor_I2C_Read(obj->client, 0x04, 1);
            APS_LOG("%s: 0x04 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.ps.adc | epl_sensor.ps.cycle, gRawData.raw_bytes[0]);
            break;

        case EPL_MODE_ALS: //als
            epl_sensor.als.adc = (value & 0x3) << 3;
            epl_sensor_I2C_Write(obj->client, 0x02, epl_sensor.als.adc | epl_sensor.als.cycle);
            epl_sensor_I2C_Read(obj->client, 0x02, 1);
            APS_LOG("%s: 0x02 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.als.adc | epl_sensor.als.cycle, gRawData.raw_bytes[0]);
            break;
    }

    epl_sensor_update_mode(obj->client);
    return count;
}

static ssize_t epl_sensor_store_cycle(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;

    APS_FUN();

    sscanf(buf, "%d",&value);

    switch (epl_sensor.mode)
    {
#if PS_DYN_K_STR
        case EPL_MODE_ALS_PS:
#else
        case EPL_MODE_PS:
#endif
            epl_sensor.ps.cycle = (value & 0x7);
            epl_sensor_I2C_Write(obj->client, 0x04, epl_sensor.ps.adc | epl_sensor.ps.cycle);
            epl_sensor_I2C_Read(obj->client, 0x04, 1);
            APS_LOG("%s: 0x04 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.ps.adc | epl_sensor.ps.cycle, gRawData.raw_bytes[0]);
            break;

        case EPL_MODE_ALS: //als
            epl_sensor.als.cycle = (value & 0x7);
            epl_sensor_I2C_Write(obj->client, 0x02, epl_sensor.als.adc | epl_sensor.als.cycle);
            epl_sensor_I2C_Read(obj->client, 0x02, 1);
            APS_LOG("%s: 0x02 = 0x%x (0x%x)\n", __FUNCTION__, epl_sensor.als.adc | epl_sensor.als.cycle, gRawData.raw_bytes[0]);
            break;
    }

    epl_sensor_update_mode(obj->client);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_als_report_type(struct device_driver *ddri, const char *buf, size_t count)
{
    int value=0;
    APS_FUN();

    sscanf(buf, "%d", &value);
    epl_sensor.als.report_type = value & 0xf;

    return count;
}


static ssize_t epl_sensor_store_ps_polling_mode(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
#if !MTK_LTE
    struct hwmsen_object obj_ps;
#endif

    sscanf(buf, "%d",&epld->hw->polling_mode_ps);
#if !MTK_LTE
    hwmsen_detach(ID_PROXIMITY);
#endif
    APS_LOG("epld->hw->polling_mode_ps=%d \r\n", epld->hw->polling_mode_ps);

    epl_sensor.ps.polling_mode = epld->hw->polling_mode_ps;

#if !MTK_LTE
    obj_ps.self = epl_sensor_obj;
    obj_ps.polling = epld->hw->polling_mode_ps;
    obj_ps.sensor_operate = epl_sensor_ps_operate;
#if PS_GES
    epl_sensor.ges.polling_mode = epld->hw->polling_mode_ps;
#endif
    if(hwmsen_attach(ID_PROXIMITY, &obj_ps))
    {
        APS_ERR("[%s]: attach fail !\n", __FUNCTION__);
    }
#else

#if PS_GES
    epl_sensor.ges.polling_mode = epld->hw->polling_mode_ps;
#endif

#endif
    epl_sensor_update_mode(epld->client);

    return count;
}

static ssize_t epl_sensor_store_als_polling_mode(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
#if !MTK_LTE
    struct hwmsen_object obj_ps;
#endif
    sscanf(buf, "%d",&epld->hw->polling_mode_als);
#if !MTK_LTE
    hwmsen_detach(ID_LIGHT);
#endif
    APS_LOG("epld->hw->polling_mode_als=%d \r\n", epld->hw->polling_mode_als);
    epl_sensor.als.polling_mode = epld->hw->polling_mode_als;
#if !MTK_LTE
    obj_ps.self = epl_sensor_obj;
    obj_ps.polling = epld->hw->polling_mode_als;
    obj_ps.sensor_operate = epl_sensor_als_operate;

    if(hwmsen_attach(ID_LIGHT, &obj_ps))
    {
        APS_ERR("[%s]: attach fail !\n", __FUNCTION__);
    }
#endif
    epl_sensor_update_mode(epld->client);

    return count;
}
static ssize_t epl_sensor_store_ps_w_calfile(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
    int ps_hthr=0, ps_lthr=0, ps_cancelation=0;
    int ps_cal_len = 0;
    char ps_calibration[20];
	APS_FUN();

	if(!epl_sensor_obj)
    {
        APS_ERR("epl_obj is null!!\n");
        return 0;
    }
    sscanf(buf, "%d,%d,%d",&ps_cancelation, &ps_hthr, &ps_lthr);

    ps_cal_len = sprintf(ps_calibration, "%d,%d,%d",  ps_cancelation, ps_hthr, ps_lthr);

    write_factory_calibration(epld, ps_calibration, ps_cal_len);
	return count;
}
/*----------------------------------------------------------------------------*/

static ssize_t epl_sensor_store_unlock(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
    int mode;
    APS_FUN();

    sscanf(buf, "%d",&mode);

    APS_LOG("mode = %d \r\n", mode);
	switch (mode)
	{
		case 0: //PS unlock and run
        	epl_sensor.ps.compare_reset = EPL_CMP_RUN;
        	epl_sensor.ps.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);
		break;

		case 1: //PS unlock and reset
        	epl_sensor.ps.compare_reset = EPL_CMP_RESET;
        	epl_sensor.ps.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);
		break;

		case 2: //ALS unlock and run
        	epl_sensor.als.compare_reset = EPL_CMP_RUN;
        	epl_sensor.als.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
		break;

        case 3: //ALS unlock and reset
    		epl_sensor.als.compare_reset = EPL_CMP_RESET;
        	epl_sensor.als.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
        break;

		case 4: //ps+als
		    //PS unlock and run
        	epl_sensor.ps.compare_reset = EPL_CMP_RUN;
        	epl_sensor.ps.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x1b, epl_sensor.ps.compare_reset |epl_sensor.ps.lock);

			//ALS unlock and run
        	epl_sensor.als.compare_reset = EPL_CMP_RUN;
        	epl_sensor.als.lock = EPL_UN_LOCK;
        	epl_sensor_I2C_Write(epld->client,0x12, epl_sensor.als.compare_reset | epl_sensor.als.lock);
		break;
	}
    /*double check PS or ALS lock*/


	return count;
}

static ssize_t epl_sensor_store_als_ch_sel(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
    int ch_sel;
    APS_FUN();

    sscanf(buf, "%d",&ch_sel);

    APS_LOG("channel selection = %d \r\n", ch_sel);
	switch (ch_sel)
	{
		case 0: //ch0
		    epl_sensor.als.interrupt_channel_select = EPL_ALS_INT_CHSEL_0;
		break;

		case 1: //ch1
        	epl_sensor.als.interrupt_channel_select = EPL_ALS_INT_CHSEL_1;
		break;
	}
    epl_sensor_I2C_Write(epld->client,0x07, epl_sensor.als.interrupt_channel_select | epl_sensor.als.persist | epl_sensor.als.interrupt_type);

    epl_sensor_update_mode(epld->client);

	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_store_als_lux_per_count(struct device_driver *ddri, const char *buf, size_t count)
{
    sscanf(buf, "%hu",&epl_sensor.als.factory.lux_per_count);
    return count;
}

static ssize_t epl_sensor_store_ps_cal_enable(struct device_driver *ddri, const char *buf, size_t count)
{
    APS_FUN();

    sscanf(buf, "%d",(int*)&epl_sensor.ps.factory.calibration_enable);

    return count;
}

static ssize_t epl_sensor_store_ps_cancelation(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
    int cancelation;
    APS_FUN();

    sscanf(buf, "%d",&cancelation);

    epl_sensor.ps.cancelation = cancelation;

    APS_LOG("epl_sensor.ps.cancelation = %d \r\n", epl_sensor.ps.cancelation);

    epl_sensor_I2C_Write(epld->client,0x22, (u8)(epl_sensor.ps.cancelation& 0xff));
    epl_sensor_I2C_Write(epld->client,0x23, (u8)((epl_sensor.ps.cancelation & 0xff00) >> 8));

	return count;
}

static ssize_t epl_sensor_show_ps_polling(struct device_driver *ddri, char *buf)
{
    u16 *tmp = (u16*)buf;
    tmp[0]= epl_sensor.ps.polling_mode;
    return 2;
}

static ssize_t epl_sensor_show_als_polling(struct device_driver *ddri, char *buf)
{
    u16 *tmp = (u16*)buf;
    tmp[0]= epl_sensor.als.polling_mode;
    return 2;
}

static ssize_t epl_sensor_show_ps_run_cali(struct device_driver *ddri, char *buf)
{
	struct epl_sensor_priv *epld = epl_sensor_obj;
	ssize_t len = 0;
    int ret;

    APS_FUN();

    ret = epl_run_ps_calibration(epld);

    len += snprintf(buf+len, PAGE_SIZE-len, "ret = %d\r\n", ret);

	return len;
}

static ssize_t epl_sensor_show_pdata(struct device_driver *ddri, char *buf)
{
      struct epl_sensor_priv *epld = epl_sensor_obj;
      ssize_t len = 0;
      bool enable_ps = test_bit(CMC_BIT_PS, &epld->enable) && atomic_read(&epld->ps_suspend)==0;
      APS_FUN();
#if !PS_DYN_K
      if(enable_ps == true && polling_flag == true && eint_flag == true && epl_sensor.ps.polling_mode == 0)
      {
        mutex_lock(&sensor_mutex);
        epl_sensor_read_ps(epld->client);
        mutex_unlock(&sensor_mutex);
      }
#endif
      len += snprintf(buf + len, PAGE_SIZE - len, "%d", epl_sensor.ps.data.data);
      return len;

}

static ssize_t epl_sensor_show_ps(struct device_driver *ddri, char *buf)
{
      struct epl_sensor_priv *epld = epl_sensor_obj;
      ssize_t len = 0;
      bool enable_ps = test_bit(CMC_BIT_PS, &epld->enable) && atomic_read(&epld->ps_suspend)==0;
      APS_FUN();
#if !PS_DYN_K
      if(enable_ps == true && polling_flag == true && eint_flag == true && epl_sensor.ps.polling_mode == 0)
      {
        mutex_lock(&sensor_mutex);
        epl_sensor_read_ps(epld->client);
        mutex_unlock(&sensor_mutex);
      }
#endif
      len += snprintf(buf + len, PAGE_SIZE - len, "%d", epl_sensor.ps.data.data);
      return len;

}

static ssize_t epl_sensor_show_als_data(struct device_driver *ddri, char *buf)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    ssize_t len = 0;
    bool enable_als = test_bit(CMC_BIT_ALS, &epld->enable) && atomic_read(&epld->als_suspend)==0;
    APS_FUN();

#if ALS_DYN_INTT
    if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
    {
        APS_LOG("[%s]: dynamic_intt_lux = %d \r\n", __func__, dynamic_intt_lux);
        len += snprintf(buf + len, PAGE_SIZE - len, "%d", dynamic_intt_lux);
    }
    else
#endif
    {
        if(enable_als == true && polling_flag == true && eint_flag == true && epl_sensor.als.polling_mode == 0)
        {
            mutex_lock(&sensor_mutex);
    	    epl_sensor_read_als(epld->client);
    	    mutex_unlock(&sensor_mutex);
    	}
    	len += snprintf(buf + len, PAGE_SIZE - len, "%d", epl_sensor.als.data.channels[1]);
    }
    return len;

}

#if PS_DYN_K
static ssize_t epl_sensor_store_dyn_offset(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int dyn_h,dyn_l;
    APS_FUN();

    sscanf(buf, "%d,%d",&dyn_l, &dyn_h);

    dynk_low_offset = dyn_l;
    dynk_high_offset = dyn_h;

    return count;
}

static ssize_t epl_sensor_store_dyn_thd_offset(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int thd_offset;
    APS_FUN();

    sscanf(buf, "%d",&thd_offset);
    dynk_thd_offset = thd_offset;

    return count;
}

static ssize_t epl_sensor_store_dyn_change_thd_max(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int thd_max;
    APS_FUN();

    sscanf(buf, "%d",&thd_max);
    dynk_change_thd_max = thd_max;

    return count;
}

static ssize_t epl_sensor_store_dyn_max_ir_data(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int max_ir_data;
    APS_FUN();

    sscanf(buf, "%d",&max_ir_data);
    dynk_max_ir_data = max_ir_data;

    return count;
}

#if PS_DYN_K_STR
static ssize_t epl_sensor_store_dyn_enhance_max_ch0(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int max_ch0;
    APS_FUN();

    sscanf(buf, "%d",&max_ch0);
    dynk_enhance_max_ch0 = max_ch0;

    return count;
}

static ssize_t epl_sensor_store_dyn_enhance_gain_intt(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int gain, intt;
    APS_FUN();

    sscanf(buf, "%d,%d",&gain, &intt);

    dynk_enhance_integration_time = intt;
    dynk_enhance_gain = gain;

    return count;
}
#endif

#endif

#if ALS_DYN_INTT
static ssize_t epl_sensor_store_c_gain(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int c_h,c_l;
    APS_FUN();

    if(epl_sensor.als.lsrc_type == CMC_BIT_LSRC_NON)
    {
        sscanf(buf, "%d",&c_h);
        c_gain = c_h;
        APS_LOG("c_gain = %d \r\n", c_gain);
    }
    else
    {
        sscanf(buf, "%d,%d",&c_l, &c_h);
        epld->c_gain_h = c_h;
        epld->c_gain_l = c_l;
    }

	return count;
}

static ssize_t epl_sensor_store_lsrc_type(struct device_driver *ddri, const char *buf, size_t count)
{
    int type;
    struct epl_sensor_priv *epld = epl_sensor_obj;
    APS_FUN();

    sscanf(buf, "%d",&type);

    epl_sensor.als.lsrc_type = type;

    APS_LOG("epl_sensor.als.lsrc_type = %d \r\n", epl_sensor.als.lsrc_type);

	return count;
}

static ssize_t epl_sensor_store_lsrc_thd(struct device_driver *ddri, const char *buf, size_t count)
{
    int lsrc_thrl, lsrc_thrh;
    struct epl_sensor_priv *epld = epl_sensor_obj;
    APS_FUN();

    sscanf(buf, "%d,%d",&lsrc_thrl, &lsrc_thrh);

    epld->lsource_thd_low = lsrc_thrl;
    epld->lsource_thd_high = lsrc_thrh;

    APS_LOG("lsource_thd=(%d,%d) \r\n", epld->lsource_thd_low, epld->lsource_thd_high);

	return count;
}

#endif

static ssize_t epl_sensor_store_reg_write(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *epld = epl_sensor_obj;
    int reg;
    int data;
    APS_FUN();

    sscanf(buf, "%x,%x",&reg, &data);

    APS_LOG("[%s]: reg=0x%x, data=0x%x", __func__, reg, data);

    if(reg == 0x00 && ((data & 0x0f) == EPL_MODE_PS || (data & 0x0f) == EPL_MODE_ALS_PS))
    {
        set_bit(CMC_BIT_PS, &epld->enable);
    }
    else
    {
        clear_bit(CMC_BIT_PS, &epld->enable);
    }

    epl_sensor_I2C_Write(epld->client, reg, data);

    return count;
}


static ssize_t epl_sensor_show_renvo(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;

    APS_FUN();
    APS_LOG("gRawData.renvo=0x%x \r\n", epl_sensor.revno);

    len += snprintf(buf+len, PAGE_SIZE-len, "%x", epl_sensor.revno);

    return len;
}
#if HS_ENABLE
static ssize_t epl_sensor_store_hs_enable(struct device_driver *ddri, const char *buf, size_t count)
{
    uint16_t mode=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;
	bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
    APS_FUN();

    sscanf(buf, "%hu",&mode);


    if(mode > 0)
	{
	    if(enable_als == 1)
	    {
            clear_bit(CMC_BIT_ALS, &obj->enable);
            hs_enable_flag = true;
	    }
		epl_sensor.hs.integration_time = epl_sensor.hs.integration_time_max;
		epl_sensor.hs.raws_count=0;
        set_bit(CMC_BIT_HS, &obj->enable);

        if(mode == 2)
        {
            epl_sensor.hs.dynamic_intt = false;
        }
        else
        {
            epl_sensor.hs.dynamic_intt = true;
        }
	}
    else
	{
        clear_bit(CMC_BIT_HS, &obj->enable);
        if(hs_enable_flag == true)
        {
            set_bit(CMC_BIT_ALS, &obj->enable);
            hs_enable_flag = false;
        }

	}
	write_global_variable(obj->client);
    epl_sensor_update_mode(obj->client);

    return count;
}

static ssize_t epl_sensor_store_hs_int_time(struct device_driver *ddri, const char *buf, size_t count)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
	int value;
	u8 intt_buf;

    sscanf(buf, "%d",&value);

	mutex_lock(&hs_sensor_mutex);
	epl_sensor.hs.integration_time = value<<2;
	intt_buf = epl_sensor.hs.integration_time | epl_sensor.hs.gain;
	epl_sensor_I2C_Write(obj->client, 0x03, intt_buf);
	mutex_unlock(&hs_sensor_mutex);

    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl_sensor_show_hs_raws(struct device_driver *ddri, char *buf)
{

    mutex_lock(&hs_sensor_mutex);

    u16 *tmp = (u16*)buf;
    int byte_count=2+epl_sensor.hs.raws_count*2;
    int i=0;
    APS_FUN();
    tmp[0]= epl_sensor.hs.raws_count;

    for(i=0; i<epl_sensor.hs.raws_count; i++){
        tmp[i+1] = epl_sensor.hs.raws[i];
    }

    epl_sensor.hs.raws_count=0;
    mutex_unlock(&hs_sensor_mutex);

    return byte_count;
}
#endif

#if PS_GES
static ssize_t epl_sensor_store_ges_enable(struct device_driver *ddri, const char *buf, size_t count)
{
    int ges_enable=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;
    struct i2c_client *client = obj->client;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    APS_FUN();

    sscanf(buf, "%d",&ges_enable);
    APS_LOG("[%s]: enable_ps=%d, ges_enable=%d \r\n", __func__, enable_ps, ges_enable);

    if(enable_ps == 0)
    {
        if(ges_enable == 1)
        {
            set_bit(CMC_BIT_GES, &obj->enable);
        }
        else
        {
            clear_bit(CMC_BIT_GES, &obj->enable);
        }
    }

    write_global_variable(obj->client);
    epl_sensor_update_mode(client);
    return count;
}

static ssize_t epl_sensor_store_ges_polling_mode(struct device_driver *ddri, const char *buf, size_t count)
{
    int ges_mode=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;
    struct i2c_client *client = obj->client;

    APS_FUN();
    sscanf(buf, "%d",&ges_mode);
    APS_LOG("[%s]: ges_mode=%d \r\n", __func__, ges_mode);

    epl_sensor.ges.polling_mode = ges_mode;

    epl_sensor_update_mode(client);
    return count;
}

static ssize_t epl_sensor_store_ges_thd(struct device_driver *ddri, const char *buf, size_t count)
{
    int ges_l=0, ges_h=0;
    struct epl_sensor_priv *obj = epl_sensor_obj;
    struct i2c_client *client = obj->client;

    APS_FUN();
    sscanf(buf, "%d,%d",&ges_l, &ges_h);
    epl_sensor.ges.low_threshold = ges_l;
    epl_sensor.ges.high_threshold = ges_h;
    APS_LOG("[%s]: ges_thd=%d,%d \r\n", __func__, epl_sensor.ges.low_threshold, epl_sensor.ges.high_threshold);

    write_global_variable(obj->client);
    epl_sensor_update_mode(client);
    return count;
}

#endif

/*=== debug_mode setting ===*/
static ssize_t epl_sensor_store_debugMode(struct device_driver *ddri, const char *buf, size_t count)
{
	struct epl_sensor_priv *obj = epl_sensor_obj;
	struct i2c_client *client = epl_sensor_obj->client;
    int tmp = 0;
    APS_FUN();

    sscanf(buf, "%d", &tmp);
	if (debug_mode_flag != tmp)
	{
		initial_global_variable(client, obj);
		debug_mode_flag = tmp;
	}

    return count;
}

/*CTS --> S_IWUSR | S_IRUGO*/
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(elan_status,					S_IROTH  | S_IWOTH, epl_sensor_show_status,  	  		NULL										);
static DRIVER_ATTR(elan_reg,    				S_IROTH  | S_IWOTH, epl_sensor_show_reg,   				NULL										);
static DRIVER_ATTR(als_enable,					S_IROTH  | S_IWOTH, NULL,   							epl_sensor_store_als_enable					);
static DRIVER_ATTR(als_report_type,				S_IROTH  | S_IWOTH, NULL,								epl_sensor_store_als_report_type			);
static DRIVER_ATTR(als_polling_mode,			S_IROTH  | S_IWOTH, epl_sensor_show_als_polling,   		epl_sensor_store_als_polling_mode			);
static DRIVER_ATTR(als_lux_per_count,			S_IROTH  | S_IWOTH, NULL,   					 		epl_sensor_store_als_lux_per_count			);
static DRIVER_ATTR(ps_enable,					S_IROTH  | S_IWOTH, NULL,   							epl_sensor_store_ps_enable					);
static DRIVER_ATTR(ps_polling_mode,			    S_IROTH  | S_IWOTH, epl_sensor_show_ps_polling,   		epl_sensor_store_ps_polling_mode			);
static DRIVER_ATTR(ir_mode,					    S_IROTH  | S_IWOTH, NULL,   							epl_sensor_store_ir_mode					);
static DRIVER_ATTR(ir_drive,					S_IROTH  | S_IWOTH, NULL,   							epl_sensor_store_ir_drive					);
static DRIVER_ATTR(ir_on,						S_IROTH  | S_IWOTH, NULL,								epl_sensor_store_ir_contrl					);
static DRIVER_ATTR(interrupt_type,				S_IROTH  | S_IWOTH, NULL,								epl_sensor_store_interrupt_type				);
static DRIVER_ATTR(integration,					S_IROTH  | S_IWOTH, NULL,								epl_sensor_store_integration				);
static DRIVER_ATTR(gain,					    S_IROTH  | S_IWOTH, NULL,								epl_sensor_store_gain					    );
static DRIVER_ATTR(adc,					        S_IROTH  | S_IWOTH, NULL,								epl_sensor_store_adc						);
static DRIVER_ATTR(cycle,						S_IROTH  | S_IWOTH, NULL,								epl_sensor_store_cycle						);
static DRIVER_ATTR(mode,						S_IROTH  | S_IWOTH, NULL,   							epl_sensor_store_mode						);
static DRIVER_ATTR(wait_time,					S_IROTH  | S_IWOTH, NULL,   					 		epl_sensor_store_wait_time					);
static DRIVER_ATTR(set_threshold,     			S_IROTH  | S_IWOTH, epl_sensor_show_threshold,                epl_sensor_store_threshold			);
static DRIVER_ATTR(cal_raw, 					S_IROTH  | S_IWOTH, epl_sensor_show_cal_raw, 	  		NULL										);
static DRIVER_ATTR(unlock,				        S_IROTH  | S_IWOTH, NULL,			                    epl_sensor_store_unlock						);
static DRIVER_ATTR(als_ch,				        S_IROTH  | S_IWOTH, NULL,			                    epl_sensor_store_als_ch_sel					);
static DRIVER_ATTR(ps_cancel,				    S_IROTH  | S_IWOTH, NULL,			                    epl_sensor_store_ps_cancelation				);
static DRIVER_ATTR(run_ps_cali, 				S_IROTH  | S_IWOTH, epl_sensor_show_ps_run_cali, 	  	NULL								    	);
static DRIVER_ATTR(pdata,                       S_IROTH  | S_IWOTH, epl_sensor_show_pdata,              NULL                                        );
static DRIVER_ATTR(ps,                       S_IROTH  | S_IWOTH, epl_sensor_show_ps,              NULL                                        );
static DRIVER_ATTR(als_data,                    S_IROTH  | S_IWOTH, epl_sensor_show_als_data,           NULL                                        );
static DRIVER_ATTR(debug_mode,					S_IROTH  | S_IWOTH, NULL,   		epl_sensor_store_debugMode			);
#if PS_DYN_K
static DRIVER_ATTR(dyn_offset,                  S_IROTH  | S_IWOTH, NULL,                               epl_sensor_store_dyn_offset                 );
static DRIVER_ATTR(dyn_thd_offset,              S_IROTH  | S_IWOTH, NULL,                               epl_sensor_store_dyn_thd_offset             );
static DRIVER_ATTR(dyn_change_max,              S_IROTH  | S_IWOTH, NULL,                               epl_sensor_store_dyn_change_thd_max         );
static DRIVER_ATTR(dyn_max_ir_data,             S_IROTH  | S_IWOTH, NULL,                               epl_sensor_store_dyn_max_ir_data            );
#if PS_DYN_K_STR
static DRIVER_ATTR(dyn_enh_max_ch0,              S_IROTH  | S_IWOTH, NULL,                               epl_sensor_store_dyn_enhance_max_ch0       );
static DRIVER_ATTR(dyn_enh_gain_intt,              S_IROTH  | S_IWOTH, NULL,                               epl_sensor_store_dyn_enhance_gain_intt       );
#endif
#endif
#if ALS_DYN_INTT
static DRIVER_ATTR(als_dyn_c_gain,              S_IROTH  | S_IWOTH, NULL,                               epl_sensor_store_c_gain                     );
static DRIVER_ATTR(als_dyn_lsrc_type,           S_IROTH  | S_IWOTH, NULL,                               epl_sensor_store_lsrc_type                  );
static DRIVER_ATTR(als_dyn_lsrc_thd,            S_IROTH  | S_IWOTH, NULL,                               epl_sensor_store_lsrc_thd                   );
#endif
static DRIVER_ATTR(i2c_w,                       S_IROTH  | S_IWOTH, NULL,                               epl_sensor_store_reg_write                  );
static DRIVER_ATTR(elan_renvo,                  S_IROTH  | S_IWOTH, epl_sensor_show_renvo,              NULL                                        );
#if HS_ENABLE
static DRIVER_ATTR(hs_enable,					S_IROTH  | S_IWOTH, NULL,                               epl_sensor_store_hs_enable                  );
static DRIVER_ATTR(hs_int_time,					S_IROTH  | S_IWOTH, NULL,                               epl_sensor_store_hs_int_time                );
static DRIVER_ATTR(hs_raws,					    S_IROTH  | S_IWOTH, epl_sensor_show_hs_raws,            NULL                                        );
#endif
#if PS_GES
static DRIVER_ATTR(ges_enable,					S_IROTH  | S_IWOTH, NULL,                               epl_sensor_store_ges_enable                 );
static DRIVER_ATTR(ges_polling_mode,			S_IROTH  | S_IWOTH, NULL,                               epl_sensor_store_ges_polling_mode           );
static DRIVER_ATTR(ges_thd,			            S_IROTH  | S_IWOTH, NULL,                               epl_sensor_store_ges_thd                    );
#endif

/*----------------------------------------------------------------------------*/
static struct driver_attribute * epl_sensor_attr_list[] =
{
    &driver_attr_elan_status,
    &driver_attr_elan_reg,
    &driver_attr_als_enable,
    &driver_attr_als_report_type,
    &driver_attr_als_polling_mode,
    &driver_attr_als_lux_per_count,
    &driver_attr_ps_enable,
    &driver_attr_ps_polling_mode,
    &driver_attr_elan_renvo,
#if HS_ENABLE
    &driver_attr_hs_enable,
	&driver_attr_hs_int_time,
    &driver_attr_hs_raws,
#endif
    &driver_attr_mode,
    &driver_attr_ir_mode,
    &driver_attr_ir_drive,
    &driver_attr_ir_on,
    &driver_attr_interrupt_type,
    &driver_attr_cal_raw,
    &driver_attr_set_threshold,
    &driver_attr_wait_time,
    &driver_attr_integration,
    &driver_attr_gain,
    &driver_attr_adc,
    &driver_attr_cycle,
    &driver_attr_unlock,
    &driver_attr_ps_cancel,
    &driver_attr_als_ch,
    &driver_attr_run_ps_cali,
    &driver_attr_pdata,
    &driver_attr_ps,
    &driver_attr_als_data,
	&driver_attr_debug_mode,
#if PS_DYN_K
    &driver_attr_dyn_offset,
    &driver_attr_dyn_thd_offset,
    &driver_attr_dyn_change_max,
    &driver_attr_dyn_max_ir_data,
#if PS_DYN_K_STR
    &driver_attr_dyn_enh_max_ch0,
    &driver_attr_dyn_enh_gain_intt,
#endif
#endif
#if ALS_DYN_INTT
    &driver_attr_als_dyn_c_gain,
    &driver_attr_als_dyn_lsrc_type,
    &driver_attr_als_dyn_lsrc_thd,
#endif
    &driver_attr_i2c_w,
#if PS_GES
    &driver_attr_ges_enable,
    &driver_attr_ges_polling_mode,
    &driver_attr_ges_thd,
#endif
};

/*----------------------------------------------------------------------------*/
static int epl_sensor_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(epl_sensor_attr_list)/sizeof(epl_sensor_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if((err = driver_create_file(driver, epl_sensor_attr_list[idx])))
        {
            APS_ERR("driver_create_file (%s) = %d\n", epl_sensor_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}



/*----------------------------------------------------------------------------*/
static int epl_sensor_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(epl_sensor_attr_list)/sizeof(epl_sensor_attr_list[0]));

    if (!driver)
        return -EINVAL;

    for (idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, epl_sensor_attr_list[idx]);
    }

    return err;
}



/******************************************************************************
 * Function Configuration
******************************************************************************/
static int epl_sensor_open(struct inode *inode, struct file *file)
{
    file->private_data = epl_sensor_i2c_client;

    APS_FUN();

    if (!file->private_data)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }

    return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int epl_sensor_release(struct inode *inode, struct file *file)
{
    APS_FUN();
    file->private_data = NULL;
    return 0;
}

/*----------------------------------------------------------------------------*/
static long epl_sensor_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct i2c_client *client = (struct i2c_client*)file->private_data;
    struct epl_sensor_priv *obj = i2c_get_clientdata(client);
    int err = 0;
#if 1
    int ps_result;
	int ps_cali;
	int threshold[2];
#endif
    void __user *ptr = (void __user*) arg;
    int dat;
    uint32_t enable;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
    APS_LOG("%s cmd = 0x%04x", __FUNCTION__, cmd);
    switch (cmd)
    {
        case ALSPS_SET_PS_MODE:
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }

            if(enable_ps != enable)
            {
                if(enable)
                {
                    //wake_lock(&ps_lock);
                    set_bit(CMC_BIT_PS, &obj->enable);
#if PS_DYN_K
                    set_psensor_intr_threshold(65534, 65535);   //dont use first ps status
                    dynk_min_ps_raw_data = 0xffff;
                    dynk_change_flag = false;
#if PS_DYN_K_STR
                    dynk_enhance_flag = false;
#endif

#endif
                }
                else
                {
                    clear_bit(CMC_BIT_PS, &obj->enable);
#if PS_DYN_K
                    cancel_delayed_work(&obj->dynk_thd_polling_work);
#endif
                    //wake_unlock(&ps_lock);
                }
                epl_sensor_update_mode(obj->client);
            }
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
            if(enable_ps == 0)
            {
                set_bit(CMC_BIT_PS, &obj->enable);
                epl_sensor_update_mode(obj->client);
            }
#if !PS_DYN_K
            if(polling_flag == true && eint_flag == true)
            {
                mutex_lock(&sensor_mutex);
                epl_sensor_read_ps_status(obj->client);
                mutex_unlock(&sensor_mutex);
            }
#endif
            dat = epl_sensor.ps.compare_low >> 3;

            APS_LOG("ioctl ps state value = %d \n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_PS_RAW_DATA:
            if(enable_ps == 0)
            {
                set_bit(CMC_BIT_PS, &obj->enable);
                epl_sensor_update_mode(obj->client);
            }
#if !PS_DYN_K
            if(polling_flag == true && eint_flag == true)
            {
                mutex_lock(&sensor_mutex);
                epl_sensor_read_ps(obj->client);
                mutex_unlock(&sensor_mutex);
            }
#endif
            dat = epl_sensor.ps.data.data;

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
            if(enable_als != enable)
            {
                if(enable)
                {
                    set_bit(CMC_BIT_ALS, &obj->enable);
#if ALS_DYN_INTT
                    if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
                    {
                        dynamic_intt_idx = dynamic_intt_init_idx;
                        epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
                        epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
                        dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
                        dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
                    }
#endif
                }
                else
                {
                    clear_bit(CMC_BIT_ALS, &obj->enable);
                }
                epl_sensor_update_mode(obj->client);
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
            if(enable_als == 0)
            {
                set_bit(CMC_BIT_ALS, &obj->enable);
                epl_sensor_update_mode(obj->client);
            }

            if(enable_als == true && polling_flag == true && eint_flag == true)
            {
                mutex_lock(&sensor_mutex);
                epl_sensor_read_als(obj->client);
                mutex_unlock(&sensor_mutex);
            }

            dat = epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);

            APS_LOG("ioctl get als data = %d\n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

        case ALSPS_GET_ALS_RAW_DATA:
            if(enable_als == 0)
            {
                set_bit(CMC_BIT_ALS, &obj->enable);
                epl_sensor_update_mode(obj->client);
            }

            if(enable_als == true && polling_flag == true && eint_flag == true)
            {
                mutex_lock(&sensor_mutex);
                epl_sensor_read_als(obj->client);
                mutex_unlock(&sensor_mutex);
            }
#if ALS_DYN_INTT
            if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
            {
                epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);
                dat = dynamic_intt_lux;
                APS_LOG("ALS_DYN_INTT: ioctl get als raw data = %d\n", dat);
            }
            else
#endif
            {
                dat = epl_sensor.als.data.channels[1];
                APS_LOG("ioctl get als raw data = %d\n", dat);
            }

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;

/*----------------------------------for factory mode test---------------------------------------*/
		case ALSPS_GET_PS_TEST_RESULT:
            if(enable_ps == 0)
            {
                set_bit(CMC_BIT_PS, &obj->enable);
                epl_sensor_update_mode(obj->client);
            }
            if(polling_flag == true && eint_flag == true)
            {
                mutex_lock(&sensor_mutex);
                epl_sensor_read_ps(obj->client);
                mutex_unlock(&sensor_mutex);
            }


            if(epl_sensor.ps.data.data > obj->hw->ps_threshold_high)
			{
			    ps_result = 0;
			}
			else
			    ps_result = 1;

			APS_LOG("[%s] ps_result = %d \r\n", __func__, ps_result);

			if(copy_to_user(ptr, &ps_result, sizeof(ps_result)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
#if 0 //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

		case ALSPS_IOCTL_CLR_CALI:
#if 0
			if(copy_from_user(&dat, ptr, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(dat == 0)
				obj->ps_cali = 0;
#else

            APS_LOG("[%s]: ALSPS_IOCTL_CLR_CALI \r\n", __func__);
#endif
			break;

		case ALSPS_IOCTL_GET_CALI:
#if 0
			ps_cali = obj->ps_cali ;
			APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
			if(copy_to_user(ptr, &ps_cali, sizeof(ps_cali)))
			{
				err = -EFAULT;
				goto err_out;
			}
#else
            APS_LOG("[%s]: ALSPS_IOCTL_GET_CALI \r\n", __func__);
#endif
			break;

		case ALSPS_IOCTL_SET_CALI:
#if 0
			if(copy_from_user(&ps_cali, ptr, sizeof(ps_cali)))
			{
				err = -EFAULT;
				goto err_out;
			}

			obj->ps_cali = ps_cali;
			APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
#else
            APS_LOG("[%s]: ALSPS_IOCTL_SET_CALI \r\n", __func__);
#endif
			break;

		case ALSPS_SET_PS_THRESHOLD:
			if(copy_from_user(threshold, ptr, sizeof(threshold)))
			{
				err = -EFAULT;
				goto err_out;
			}
#if 0
			APS_ERR("%s set threshold high: 0x%x, low: 0x%x\n", __func__, threshold[0],threshold[1]);
			atomic_set(&obj->ps_thd_val_high,  (threshold[0]+obj->ps_cali));
			atomic_set(&obj->ps_thd_val_low,  (threshold[1]+obj->ps_cali));//need to confirm
			set_psensor_threshold(obj->client);
#else
            APS_LOG("[%s] set threshold high: %d, low: %d\n", __func__, threshold[0],threshold[1]);
            obj->hw->ps_threshold_high = threshold[0];
            obj->hw->ps_threshold_low = threshold[1];
            set_psensor_intr_threshold(obj->hw->ps_threshold_low, obj->hw->ps_threshold_high);
#endif
			break;
#endif //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		case ALSPS_GET_PS_THRESHOLD_HIGH:
#if 0
			APS_ERR("%s get threshold high before cali: 0x%x\n", __func__, atomic_read(&obj->ps_thd_val_high));
			threshold[0] = atomic_read(&obj->ps_thd_val_high) - obj->ps_cali;
			APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
			APS_ERR("%s get threshold high: 0x%x\n", __func__, threshold[0]);
#else
            threshold[0] = obj->hw->ps_threshold_high;
            APS_LOG("[%s] get threshold high: %d\n", __func__, threshold[0]);
#endif
			if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_THRESHOLD_LOW:
#if 0
			APS_ERR("%s get threshold low before cali: 0x%x\n", __func__, atomic_read(&obj->ps_thd_val_low));
			threshold[0] = atomic_read(&obj->ps_thd_val_low) - obj->ps_cali;
			APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
			APS_ERR("%s get threshold low: 0x%x\n", __func__, threshold[0]);
#else
            threshold[0] = obj->hw->ps_threshold_low;
            APS_LOG("[%s] get threshold low: %d\n", __func__, threshold[0]);
#endif
			if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
		/*------------------------------------------------------------------------------------------*/
        default:
            APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
            err = -ENOIOCTLCMD;
            break;
    }

err_out:
    return err;
}


/*----------------------------------------------------------------------------*/
static struct file_operations epl_sensor_fops =
{
    .owner = THIS_MODULE,
    .open = epl_sensor_open,
    .release = epl_sensor_release,
    .unlocked_ioctl = epl_sensor_unlocked_ioctl,
};


/*----------------------------------------------------------------------------*/
static struct miscdevice epl_sensor_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "als_ps",
    .fops = &epl_sensor_fops,
};


/*----------------------------------------------------------------------------*/
static int epl_sensor_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
    struct epl_sensor_priv *obj = i2c_get_clientdata(client);
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
    APS_FUN();
#if 0
    if(msg.event == PM_EVENT_SUSPEND)
    {
        if(!obj)
        {
            APS_ERR("null pointer!!\n");
            return -EINVAL;
        }
        atomic_set(&obj->als_suspend, 1);
#if HS_ENABLE
        atomic_set(&obj->hs_suspend, 1);
#endif
#if PS_GES
        atomic_set(&obj->ges_suspend, 1);
#endif
        if(enable_ps == 1){
            atomic_set(&obj->ps_suspend, 0);
            APS_LOG("[%s]: ps enable \r\n", __func__);
        }
        else{
            atomic_set(&obj->ps_suspend, 1);
            APS_LOG("[%s]: ps disable \r\n", __func__);
            epl_sensor_update_mode(obj->client);
        }

        epl_sensor_power(obj->hw, 0);
    }
#endif
    return 0;

}



/*----------------------------------------------------------------------------*/
static int epl_sensor_i2c_resume(struct i2c_client *client)
{
    struct epl_sensor_priv *obj = i2c_get_clientdata(client);
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
    APS_FUN();
#if 0
    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }
    atomic_set(&obj->ps_suspend, 0);
    atomic_set(&obj->als_suspend, 0);
#if HS_ENABLE
    atomic_set(&obj->hs_suspend, 0);
#endif
#if PS_GES
    atomic_set(&obj->ges_suspend, 0);
#endif
    if(enable_ps == 1)
    {
        APS_LOG("[%s]: ps enable \r\n", __func__);
    }
    else
    {
        APS_LOG("[%s]: ps disable \r\n", __func__);
        epl_sensor_update_mode(obj->client);
    }

    epl_sensor_power(obj->hw, 1);
#endif
    return 0;
}



/*----------------------------------------------------------------------------*/
static void epl_sensor_early_suspend(struct early_suspend *h)
{
    /*early_suspend is only applied for ALS*/
    struct epl_sensor_priv *obj = container_of(h, struct epl_sensor_priv, early_drv);
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;

    APS_FUN();

    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return;
    }

    atomic_set(&obj->als_suspend, 1);
#if HS_ENABLE
    atomic_set(&obj->hs_suspend, 1);
#endif
#if PS_GES
    ps_ges_suspend_flag = true;
    atomic_set(&obj->ges_suspend, 1);
#endif
    if(enable_ps == 1){
        //atomic_set(&obj->ps_suspend, 0);

        APS_LOG("[%s]: ps enable \r\n", __func__);
    }
    else{
        //atomic_set(&obj->ps_suspend, 1);
        APS_LOG("[%s]: ps disable \r\n", __func__);
        epl_sensor_update_mode(obj->client);
    }


}



/*----------------------------------------------------------------------------*/
static void epl_sensor_late_resume(struct early_suspend *h)
{
    /*late_resume is only applied for ALS*/
    struct epl_sensor_priv *obj = container_of(h, struct epl_sensor_priv, early_drv);
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable);
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable);

    APS_FUN();

    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return;
    }

    atomic_set(&obj->als_suspend, 0);
    atomic_set(&obj->ps_suspend, 0);
#if HS_ENABLE
    atomic_set(&obj->hs_suspend, 0);
#endif
#if PS_GES
    atomic_set(&obj->ges_suspend, 0);
#endif
    if(enable_ps == 1)
    {
        //atomic_set(&obj->ps_suspend, 0);
        APS_LOG("[%s]: ps enable \r\n", __func__);
    }
    else
    {
        APS_LOG("[%s]: ps disable \r\n", __func__);
        epl_sensor_update_mode(obj->client);
    }
#if PS_GES
    ps_ges_suspend_flag = false;
#endif

}

#if MTK_LTE /*MTK_LTE start .................*/
/*--------------------------------------------------------------------------------*/
static int als_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int als_enable_nodata(int en)
{
    struct epl_sensor_priv *obj = epl_sensor_obj;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
	if(!obj)
	{
		APS_ERR("obj is null!!\n");
		return -1;
	}
	APS_LOG("[%s] als enable en = %d\n", __func__, en);

    if(enable_als != en)
    {
        if(en)
        {
            set_bit(CMC_BIT_ALS, &obj->enable);
#if ALS_DYN_INTT
            if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
            {
                dynamic_intt_idx = dynamic_intt_init_idx;
                epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
                epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
                dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
                dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
            }
#endif
        }
        else
        {
            clear_bit(CMC_BIT_ALS, &obj->enable);
        }
        epl_sensor_update_mode(obj->client);
    }

	return 0;
}
/*--------------------------------------------------------------------------------*/
static int als_set_delay(u64 ns)
{
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int als_get_data(int* value, int* status)
{
	int err = 0;
	struct epl_sensor_priv *obj = epl_sensor_obj;
	if(!obj)
	{
		APS_ERR("obj is null!!\n");
		return -1;
	}

#if PS_DYN_K && PS_DYN_K_STR
    if(polling_flag == true && eint_flag == true && dynk_enhance_flag == true)
#else
    if(polling_flag == true && eint_flag == true)
#endif
    {
        mutex_lock(&sensor_mutex);
        epl_sensor_read_als(obj->client);
        if(epl_sensor.wait == EPL_WAIT_SINGLE)
            epl_sensor_I2C_Write(obj->client,0x11,  epl_sensor.power | epl_sensor.reset);
        mutex_unlock(&sensor_mutex);
    }
    else
    {
#if PS_DYN_K && PS_DYN_K_STR
        APS_LOG("[%s]: epl_sensor_update_mode(%d) or eint_work(%d) is running, dynk_enhance_flag=%d !\r\n", __func__, polling_flag, eint_flag, dynk_enhance_flag);
#else
        APS_LOG("[%s]: epl_sensor_update_mode(%d) or eint_work(%d) is running !\r\n", __func__, polling_flag, eint_flag);
#endif
    }

    if(epl_sensor.als.report_type != CMC_BIT_DYN_INT){
        *value = epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);
    }
#if ALS_DYN_INTT
    else
    {
#if PS_DYN_K && PS_DYN_K_STR
        if(polling_flag == true && eint_flag == true && dynk_enhance_flag == true){
#else
        if(polling_flag == true && eint_flag == true){
#endif
            epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);
            APS_LOG("[%s]: dynamic_intt_lux=%d \r\n", __func__, dynamic_intt_lux);
        }
        *value = dynamic_intt_lux;
    }
#endif

    *status = SENSOR_STATUS_ACCURACY_MEDIUM;
    APS_LOG("[%s]:*value = %d\n", __func__, *value);

	return err;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int ps_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int ps_enable_nodata(int en)
{
	struct epl_sensor_priv *obj = epl_sensor_obj;
    struct i2c_client *client = obj->client;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
#if HS_ENABLE
    bool enable_hs = test_bit(CMC_BIT_HS, &obj->enable) && atomic_read(&obj->hs_suspend)==0;
#endif
#if PS_GES
    bool enable_ges = test_bit(CMC_BIT_GES, &obj->enable) && atomic_read(&obj->ges_suspend)==0;
#endif
    APS_LOG("ps enable = %d\n", en);
#if HS_ENABLE
    if(enable_hs == 1 && en == 1)
    {
        clear_bit(CMC_BIT_HS, &obj->enable);
        if(hs_enable_flag == true)
        {
            set_bit(CMC_BIT_ALS, &obj->enable);
            hs_enable_flag = false;
        }
        write_global_variable(obj->client);
        APS_LOG("[%s] Disable HS and recover ps setting \r\n", __func__);
    }
#endif
#if PS_GES
    if(enable_ges == 1 && en == 1)
    {
        clear_bit(CMC_BIT_GES, &obj->enable);
        write_global_variable(obj->client);
        ps_ges_enable_flag = true;
        APS_LOG("[%s] Disable GES and recover ps setting \r\n", __func__);
    }
    else if (ps_ges_enable_flag == true && en == 0)
    {
        set_bit(CMC_BIT_GES, &obj->enable);
        write_global_variable(obj->client);
        ps_ges_enable_flag = false;
        APS_LOG("[%s] enable GES and recover ges setting \r\n", __func__);
    }
#endif
    if(enable_ps != en)
    {
        if(en)
        {
            //wake_lock(&ps_lock);
            set_bit(CMC_BIT_PS, &obj->enable);
#if PS_DYN_K
            set_psensor_intr_threshold(65534, 65535);   //dont use first ps status
            dynk_min_ps_raw_data = 0xffff;
            dynk_change_flag = false;
#if PS_DYN_K_STR
            dynk_enhance_flag = false;
#endif

#endif
        }
        else
        {

            clear_bit(CMC_BIT_PS, &obj->enable);
#if PS_DYN_K
            cancel_delayed_work(&obj->dynk_thd_polling_work);
#endif
            //wake_unlock(&ps_lock);
        }
        epl_sensor_update_mode(client);
    }

	return 0;

}
/*--------------------------------------------------------------------------------*/
static int ps_set_delay(u64 ns)
{
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int ps_get_data(int* value, int* status)
{

    int err = 0;
    struct epl_sensor_priv *obj = epl_sensor_obj;
    struct i2c_client *client = obj->client;

    APS_LOG("---SENSOR_GET_DATA---\n\n");
#if PS_DYN_K && PS_DYN_K_STR
    if(polling_flag == true && eint_flag == true && dynk_enhance_flag == true)
#else
    if(polling_flag == true && eint_flag == true)
#endif
    {
        mutex_lock(&sensor_mutex);

        epl_sensor_read_ps_status(obj->client);
        epl_sensor_read_ps(obj->client);

        if(epl_sensor.wait == EPL_WAIT_SINGLE)
            epl_sensor_I2C_Write(obj->client,0x11,  epl_sensor.power | epl_sensor.reset);

        mutex_unlock(&sensor_mutex);
	 }
	 else
	 {
#if PS_DYN_K && PS_DYN_K_STR
        APS_LOG("[%s]: epl_sensor_update_mode(%d) or eint_work(%d) is running, dynk_enhance_flag=%d !\r\n", __func__, polling_flag, eint_flag, dynk_enhance_flag);
#else
        APS_LOG("[%s]: epl_sensor_update_mode(%d) or eint_work(%d) is running\r\n", __func__, polling_flag, eint_flag);
#endif
	 }

#if PS_DYN_K
     epl_sensor_reset_dynk_thd(dynk_last_status, (epl_sensor.ps.compare_low >> 3));
     dynk_last_status = (epl_sensor.ps.compare_low >> 3);
#endif

    *value = epl_sensor.ps.compare_low >> 3;
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;

    APS_LOG("[%s]:*value = %d\n", __func__, *value);

	return err;
}
/*----------------------------------------------------------------------------*/

#else   /*MTK_LTE*/

/*----------------------------------------------------------------------------*/
int epl_sensor_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
                       void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value;
    hwm_sensor_data* sensor_data;
    struct epl_sensor_priv *obj = (struct epl_sensor_priv *)self;
    struct i2c_client *client = obj->client;
    bool enable_ps = test_bit(CMC_BIT_PS, &obj->enable) && atomic_read(&obj->ps_suspend)==0;
#if HS_ENABLE
    bool enable_hs = test_bit(CMC_BIT_HS, &obj->enable) && atomic_read(&obj->hs_suspend)==0;
#endif
#if PS_GES
    bool enable_ges = test_bit(CMC_BIT_GES, &obj->enable) && atomic_read(&obj->ges_suspend)==0;
#endif
    APS_LOG("epl_sensor_ps_operate command = %x\n",command);
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
#if HS_ENABLE
                if(enable_hs == 1 && value == 1)
                {
                    clear_bit(CMC_BIT_HS, &obj->enable);
                    if(hs_enable_flag == true)
                    {
                        set_bit(CMC_BIT_ALS, &obj->enable);
                        hs_enable_flag = false;
                    }
                    write_global_variable(obj->client);
                    APS_LOG("[%s] Disable HS and recover ps setting \r\n", __func__);
                }
#endif
#if PS_GES
                if(enable_ges == 1 && value == 1)
                {
                    clear_bit(CMC_BIT_GES, &obj->enable);
                    write_global_variable(obj->client);
                    ps_ges_enable_flag = true;
                    APS_LOG("[%s] Disable GES and recover ps setting \r\n", __func__);
                }
                else if (ps_ges_enable_flag == true && value == 0)
                {
                    set_bit(CMC_BIT_GES, &obj->enable);
                    write_global_variable(obj->client);
                    ps_ges_enable_flag = false;
                    APS_LOG("[%s] enable GES and recover ges setting \r\n", __func__);
                }
#endif
                if(enable_ps != value)
                {
                    if(value)
                    {
                        //wake_lock(&ps_lock);
                        set_bit(CMC_BIT_PS, &obj->enable);
#if PS_DYN_K
                        set_psensor_intr_threshold(65534, 65535);   //dont use first ps status
                        dynk_min_ps_raw_data = 0xffff;
                        dynk_change_flag = false;
#if PS_DYN_K_STR
                        dynk_enhance_flag = false;
#endif

#endif
                    }
                    else
                    {

                        clear_bit(CMC_BIT_PS, &obj->enable);
#if PS_DYN_K
                        cancel_delayed_work(&obj->dynk_thd_polling_work);
#endif
                        //wake_unlock(&ps_lock);
                    }
                    epl_sensor_update_mode(client);
                }
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
#if PS_DYN_K && PS_DYN_K_STR
                if(polling_flag == true && eint_flag == true && dynk_enhance_flag == true)
#else
                if(polling_flag == true && eint_flag == true)
#endif
                {
                    mutex_lock(&sensor_mutex);

                    epl_sensor_read_ps_status(client);
                    epl_sensor_read_ps(client);

                    if(epl_sensor.wait == EPL_WAIT_SINGLE)
                        epl_sensor_I2C_Write(client,0x11,  epl_sensor.power | epl_sensor.reset);

                    mutex_unlock(&sensor_mutex);
     		    }
     		    else
     		    {
#if PS_DYN_K && PS_DYN_K_STR
                    APS_LOG("[%s]: epl_sensor_update_mode(%d) or eint_work(%d) is running, dynk_enhance_flag=%d !\r\n", __func__, polling_flag, eint_flag, dynk_enhance_flag);
#else
                    APS_LOG("[%s]: epl_sensor_update_mode(%d) or eint_work(%d) is running\r\n", __func__, polling_flag, eint_flag);
#endif
     		    }
#if PS_DYN_K
                epl_sensor_reset_dynk_thd(dynk_last_status, (epl_sensor.ps.compare_low >> 3));
                dynk_last_status = (epl_sensor.ps.compare_low >> 3);
#endif


                sensor_data = (hwm_sensor_data *)buff_out;
                sensor_data->values[0] = epl_sensor.ps.compare_low >> 3;
                sensor_data->values[1] = epl_sensor.ps.data.data;
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



int epl_sensor_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
                        void* buff_out, int size_out, int* actualout)
{
    int err = 0;
    int value = 0;
    hwm_sensor_data* sensor_data;
    struct epl_sensor_priv *obj = (struct epl_sensor_priv *)self;
    struct i2c_client *client = obj->client;
    bool enable_als = test_bit(CMC_BIT_ALS, &obj->enable) && atomic_read(&obj->als_suspend)==0;
    APS_FUN();
    APS_LOG("epl_sensor_als_operate command = %x\n",command);

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
                APS_LOG("als enable = %d\n", value);

                value = *(int *)buff_in;
                if(enable_als != value)
                {
                    if(value)
                    {
                        set_bit(CMC_BIT_ALS, &obj->enable);
#if ALS_DYN_INTT
                        if(epl_sensor.als.report_type == CMC_BIT_DYN_INT)
                        {
                            dynamic_intt_idx = dynamic_intt_init_idx;
                            epl_sensor.als.integration_time = als_dynamic_intt_intt[dynamic_intt_idx];
                            epl_sensor.als.gain = als_dynamic_intt_gain[dynamic_intt_idx];
                            dynamic_intt_high_thr = als_dynamic_intt_high_thr[dynamic_intt_idx];
                            dynamic_intt_low_thr = als_dynamic_intt_low_thr[dynamic_intt_idx];
                        }
#endif
                    }
                    else
                    {
                        clear_bit(CMC_BIT_ALS, &obj->enable);
                    }
                    epl_sensor_update_mode(client);
                }
            }
            break;


        case SENSOR_GET_DATA:
            APS_LOG("get als data !!!!!!\n");
#if HS_ENABLE
            if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)) || first_als_report_flag == true || hs_enable_flag == true)
#else
            if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)) || first_als_report_flag == true)
#endif
            {
                APS_ERR("get sensor data parameter error!\n");
#if PS_DYN_K && PS_DYN_K_STR
                if(polling_flag == true && eint_flag == true && dynk_enhance_flag == true)
#else
                if(polling_flag == true && eint_flag == true)
#endif
                    first_als_report_flag = false;
                err = -EINVAL;
            }
            else
            {
#if PS_DYN_K && PS_DYN_K_STR
                if(polling_flag == true && eint_flag == true && dynk_enhance_flag == true)
#else
                if(polling_flag == true && eint_flag == true)
#endif
                {
                    mutex_lock(&sensor_mutex);
                    epl_sensor_read_als(client);
                    if(epl_sensor.wait == EPL_WAIT_SINGLE)
                        epl_sensor_I2C_Write(client,0x11,  epl_sensor.power | epl_sensor.reset);
                    mutex_unlock(&sensor_mutex);
                }
                else
                {
#if PS_DYN_K && PS_DYN_K_STR
                    APS_LOG("[%s]: epl_sensor_update_mode(%d) or eint_work(%d) is running, dynk_enhance_flag=%d !\r\n", __func__, polling_flag, eint_flag, dynk_enhance_flag);
#else
                    APS_LOG("[%s]: epl_sensor_update_mode(%d) or eint_work(%d) is running !\r\n", __func__, polling_flag, eint_flag);
#endif
                }
                sensor_data = (hwm_sensor_data *)buff_out;

                if(epl_sensor.als.report_type != CMC_BIT_DYN_INT){
                    sensor_data->values[0] = epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);
                }
#if ALS_DYN_INTT
                else{
#if PS_DYN_K && PS_DYN_K_STR
                    if(polling_flag == true && eint_flag == true && dynk_enhance_flag == true){
#else
                    if(polling_flag == true && eint_flag == true){
#endif
                        epl_sensor_get_als_value(obj, epl_sensor.als.data.channels[1]);
                        APS_LOG("[%s]: dynamic_intt_lux=%d \r\n", __func__, dynamic_intt_lux);
                    }
                    sensor_data->values[0] = dynamic_intt_lux;
                }
#endif
                sensor_data->values[1] = epl_sensor.als.data.channels[1];
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
#endif /*MTK_LTE end .................*/

/*----------------------------------------------------------------------------*/

static int epl_sensor_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
    strcpy(info->type, EPL_DEV_NAME);
    return 0;
}


/*----------------------------------------------------------------------------*/
static int epl_sensor_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct epl_sensor_priv *obj;
    struct hwmsen_object obj_ps, obj_als;
#if MTK_LTE
    struct als_control_path als_ctl={0};
	struct als_data_path als_data={0};
	struct ps_control_path ps_ctl={0};
	struct ps_data_path ps_data={0};
#endif
    int err = 0, i=0;
    APS_FUN();
    client->addr=0x49;
    epl_sensor_dumpReg(client);

    if((i2c_smbus_read_byte_data(client, 0x21)) != EPL_REVNO){ //check chip
        APS_LOG("elan ALS/PS sensor is failed. \n");
	err=-1;
        goto exit;
    }

    client->timing = 400;

    if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
    {
        err = -ENOMEM;
        goto exit;
    }

    memset(obj, 0, sizeof(*obj));

    epl_sensor_obj = obj;
    obj->hw = epl259x_get_cust_alsps_hw();

    epl_sensor_get_addr(obj->hw, &obj->addr);

    obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
    obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);
    BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
    memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
    BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
    memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
#if PS_GES
    atomic_set(&obj->ges_suspend, 0);
    obj->gs_input_dev = input_allocate_device();
    set_bit(EV_KEY, obj->gs_input_dev->evbit);
    set_bit(EV_REL, obj->gs_input_dev->evbit);
    set_bit(EV_ABS, obj->gs_input_dev->evbit);
    obj->gs_input_dev->evbit[0] |= BIT_MASK(EV_REP);
    obj->gs_input_dev->keycodemax = 500;
    obj->gs_input_dev->name ="elan_gesture";
    obj->gs_input_dev->keybit[BIT_WORD(KEYCODE_LEFT)] |= BIT_MASK(KEYCODE_LEFT);
    if (input_register_device(obj->gs_input_dev))
        APS_ERR("register input error\n");
#endif

    INIT_DELAYED_WORK(&obj->eint_work, epl_sensor_eint_work);
#if PS_DYN_K
    INIT_DELAYED_WORK(&obj->dynk_thd_polling_work, epl_sensor_dynk_thd_polling_work);
#endif
#if HS_ENABLE
    atomic_set(&obj->hs_suspend, 0);
	INIT_DELAYED_WORK(&obj->polling_work, epl_sensor_polling_work);
	mutex_init(&hs_sensor_mutex);
#endif
    obj->client = client;

    mutex_init(&sensor_mutex);
    wake_lock_init(&ps_lock, WAKE_LOCK_SUSPEND, "ps wakelock");

    i2c_set_clientdata(client, obj);

    atomic_set(&obj->trace, 0x00);
    atomic_set(&obj->als_suspend, 0);
    atomic_set(&obj->ps_suspend, 0);

    obj->enable = 0;
    obj->pending_intr = 0;

    epl_sensor_i2c_client = client;

    //initial global variable and write to senosr
    initial_global_variable(client, obj);

    if((err = epl_sensor_init_client(client)))
    {
        goto exit_init_failed;
    }

    if((err = misc_register(&epl_sensor_device)))
    {
        APS_ERR("epl_sensor_device register failed\n");
        goto exit_misc_device_register_failed;
    }
#if MTK_LTE /*MTK_LTE start .................*/
    if((err = epl_sensor_create_attr(&epl_sensor_init_info.platform_diver_addr->driver)))
    {
        APS_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }

    als_ctl.open_report_data= als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
	als_ctl.is_report_input_direct = epl_sensor.als.polling_mode==0? true:false; //false;
#ifdef CUSTOM_KERNEL_SENSORHUB
	als_ctl.is_support_batch = true;
#else
    als_ctl.is_support_batch = false;
#endif

	err = als_register_control_path(&als_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_create_attr_failed;
	}


	ps_ctl.open_report_data= ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
	ps_ctl.is_report_input_direct = epl_sensor.ps.polling_mode==0? true:false; //false;
#ifdef CUSTOM_KERNEL_SENSORHUB
	ps_ctl.is_support_batch = true;
#else
    ps_ctl.is_support_batch = false;
#endif

	err = ps_register_control_path(&ps_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_create_attr_failed;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_create_attr_failed;
	}

#else /*MTK_LTE */
    if((err = epl_sensor_create_attr(&epl_sensor_alsps_driver.driver)))
    {
        APS_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }

    obj_ps.self = epl_sensor_obj;

    ps_hw = &obj_ps;

    if( obj->hw->polling_mode_ps)
    {
        obj_ps.polling = 1;
        APS_LOG("ps_interrupt == false\n");
    }
    else
    {
        obj_ps.polling = 0;//interrupt mode
        APS_LOG("ps_interrupt == true\n");
    }

    obj_ps.sensor_operate = epl_sensor_ps_operate;

    if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
    {
        APS_ERR("attach fail = %d\n", err);
        goto exit_create_attr_failed;
    }


    obj_als.self = epl_sensor_obj;

    als_hw = &obj_als;

    if( obj->hw->polling_mode_als)
    {
        obj_als.polling = 1;
        APS_LOG("ps_interrupt == false\n");
    }
    else
    {
        obj_als.polling = 0;//interrupt mode
        APS_LOG("ps_interrupt == true\n");
    }

    obj_als.sensor_operate = epl_sensor_als_operate;
    APS_LOG("als polling mode\n");


    if((err = hwmsen_attach(ID_LIGHT, &obj_als)))
    {
        APS_ERR("attach fail = %d\n", err);
        goto exit_create_attr_failed;
    }
#endif  /*MTK_LTE end .................*/

#if defined(CONFIG_HAS_EARLYSUSPEND)
    obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
    obj->early_drv.suspend  = epl_sensor_early_suspend,
    obj->early_drv.resume   = epl_sensor_late_resume,
    register_early_suspend(&obj->early_drv);
#endif

    if(obj->hw->polling_mode_ps == 0 || obj->hw->polling_mode_als == 0)
        epl_sensor_setup_eint(client);

#if MTK_LTE
    alsps_init_flag = 0;
#endif

    APS_LOG("%s: OK\n", __FUNCTION__);
    return 0;

exit_create_attr_failed:
    misc_deregister(&epl_sensor_device);
exit_misc_device_register_failed:
exit_init_failed:
    //i2c_detach_client(client);
//	exit_kfree:
    kfree(obj);
exit:
    epl_sensor_i2c_client = NULL;
#if MTK_LTE
    alsps_init_flag = -1;
#endif
    APS_ERR("%s: err = %d\n", __FUNCTION__, err);
    return err;



}



/*----------------------------------------------------------------------------*/
static int epl_sensor_i2c_remove(struct i2c_client *client)
{
    int err;
#if MTK_LTE
    if((err = epl_sensor_delete_attr(&epl_sensor_init_info.platform_diver_addr->driver)))
    {
        APS_ERR("epl_sensor_delete_attr fail: %d\n", err);
    }
#else
    if((err = epl_sensor_delete_attr(&epl_sensor_i2c_driver.driver)))
    {
        APS_ERR("epl_sensor_delete_attr fail: %d\n", err);
    }
#endif
    if((err = misc_deregister(&epl_sensor_device)))
    {
        APS_ERR("misc_deregister fail: %d\n", err);
    }

    epl_sensor_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));

    return 0;
}


#if !MTK_LTE
/*----------------------------------------------------------------------------*/
static int epl_sensor_probe(struct platform_device *pdev)
{
    struct alsps_hw *hw = epl259x_get_cust_alsps_hw();

    epl_sensor_power(hw, 1);

    if(i2c_add_driver(&epl_sensor_i2c_driver))
    {
        APS_ERR("add driver error\n");
        return -1;
    }
    return 0;
}



/*----------------------------------------------------------------------------*/
static int epl_sensor_remove(struct platform_device *pdev)
{
    struct alsps_hw *hw = epl259x_get_cust_alsps_hw();
    APS_FUN();
    epl_sensor_power(hw, 0);

    APS_ERR("EPL259x remove \n");
    i2c_del_driver(&epl_sensor_i2c_driver);
    return 0;
}


#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{ .compatible = "mediatek,als_ps", },
	{},
};
#endif

/*----------------------------------------------------------------------------*/
static struct platform_driver epl_sensor_alsps_driver =
{
    .probe      = epl_sensor_probe,
    .remove     = epl_sensor_remove,
    .driver     = {
        .name  = "als_ps",
        //.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
#endif
    }
};
#endif

#if MTK_LTE
/*----------------------------------------------------------------------------*/
static int alsps_local_init(void)
{
    struct alsps_hw *hw = epl259x_get_cust_alsps_hw();
	//printk("fwq loccal init+++\n");

	epl_sensor_power(hw, 1);

	if(i2c_add_driver(&epl_sensor_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	}

	if(-1 == alsps_init_flag)
	{
	   return -1;
	}
	//printk("fwq loccal init---\n");
	return 0;
}
/*----------------------------------------------------------------------------*/
static int alsps_remove()
{
    struct alsps_hw *hw = epl259x_get_cust_alsps_hw();
    APS_FUN();
    epl_sensor_power(hw, 0);

    APS_ERR("epl_sensor remove \n");

    i2c_del_driver(&epl_sensor_i2c_driver);
    return 0;
}
#endif

/*----------------------------------------------------------------------------*/
static int __init epl_sensor_init(void)
{
    struct alsps_hw *hw = epl259x_get_cust_alsps_hw();
    APS_FUN();
    i2c_register_board_info(hw->i2c_num, &i2c_epl_sensor, 1);
#if MTK_LTE
    alsps_driver_add(&epl_sensor_init_info);
#else
    if(platform_driver_register(&epl_sensor_alsps_driver))
    {
        APS_ERR("failed to register driver");
        return -ENODEV;
    }
#endif



    return 0;
}

/*----------------------------------------------------------------------------*/
static void __exit epl_sensor_exit(void)
{
    APS_FUN();
#if !MTK_LTE
    platform_driver_unregister(&epl_sensor_alsps_driver);
#endif
}
/*----------------------------------------------------------------------------*/
module_init(epl_sensor_init);
module_exit(epl_sensor_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("renato.pan@eminent-tek.com");
MODULE_DESCRIPTION("EPL259x ALPsr driver");
MODULE_LICENSE("GPL");
