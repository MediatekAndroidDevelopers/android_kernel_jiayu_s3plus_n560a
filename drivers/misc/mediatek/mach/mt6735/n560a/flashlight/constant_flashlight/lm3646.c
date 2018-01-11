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
#include <mach/upmu_common.h>

#include <mach/mt_gpio.h>		// For gpio control

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



/*
0  1  2  3   4   5   6   7   8   9   10  11  12  13
25 50 75 100 125 150 300 400 500 600 700 800 900 1000
*/

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
	#define logI PK_DBG_FUNC
	#define logE(fmt, arg...)         printk(KERN_ERR PFX "%s: " fmt, __func__ ,##arg)
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

//#define STROBE_DEVICE_ID 0x67
#define STROBE_DEVICE_ID 0xCE

/*****************************************************************************
Functions
*****************************************************************************/

//extern int FlashReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
//extern int FlashWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);

static struct i2c_client *LM3646_i2c_client = NULL;




struct LM3646_platform_data {
	u8 torch_pin_enable;    // 1:  TX1/TORCH pin isa hardware TORCH enable
	u8 pam_sync_pin_enable; // 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input
	u8 thermal_comp_mode_enable;// 1: LEDI/NTC pin in Thermal Comparator Mode
	u8 strobe_pin_disable;  // 1 : STROBE Input disabled
	u8 vout_mode_enable;  // 1 : Voltage Out Mode enable
};

struct LM3646_chip_data {
	struct i2c_client *client;

	//struct led_classdev cdev_flash;
	//struct led_classdev cdev_torch;
	//struct led_classdev cdev_indicator;

	struct LM3646_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};


static int LM3646_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret=0;
	struct LM3646_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret =  i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_ERR("failed writting at 0x%02x\n", reg);
	return ret;
}

static int LM3646_read_reg(struct i2c_client *client, u8 reg)
{
	int val=0;
	struct LM3646_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val =  i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);


	return val;
}

static int LM3646_chip_init(struct LM3646_chip_data *chip)
{
	return 0;
}

static int LM3646_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct LM3646_chip_data *chip;
	struct LM3646_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	PK_DBG("LM3646_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		printk(KERN_ERR  "LM3646 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct LM3646_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if(pdata == NULL){ //values are set to Zero.
		PK_ERR("LM3646 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct LM3646_platform_data),GFP_KERNEL);
		chip->pdata  = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata  = pdata;
	if(LM3646_chip_init(chip)<0)
		goto err_chip_init;

	LM3646_i2c_client = client;
	PK_DBG("LM3646 Initializing is done \n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	PK_ERR("LM3646 probe is failed \n");
	return -ENODEV;
}

static int LM3646_remove(struct i2c_client *client)
{
	struct LM3646_chip_data *chip = i2c_get_clientdata(client);

    if(chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}


#define LM3646_NAME "leds-LM3646"
static const struct i2c_device_id LM3646_id[] = {
	{LM3646_NAME, 0},
	{}
};

static struct i2c_driver LM3646_i2c_driver = {
	.driver = {
		.name  = LM3646_NAME,
	},
	.probe	= LM3646_probe,
	.remove   = LM3646_remove,
	.id_table = LM3646_id,
};

struct LM3646_platform_data LM3646_pdata = {0, 0, 0, 0, 0};
static struct i2c_board_info __initdata i2c_LM3646={ I2C_BOARD_INFO(LM3646_NAME, 0x67), \
													.platform_data = &LM3646_pdata,};

static int __init LM3646_init(void)
{
	printk("LM3646_init\n");
	//i2c_register_board_info(2, &i2c_LM3646, 1);
	i2c_register_board_info(2, &i2c_LM3646, 1);


	return i2c_add_driver(&LM3646_i2c_driver);
}

static void __exit LM3646_exit(void)
{
	i2c_del_driver(&LM3646_i2c_driver);
}


module_init(LM3646_init);
module_exit(LM3646_exit);

MODULE_DESCRIPTION("Flash driver for LM3646");
MODULE_AUTHOR("pw <pengwei@mediatek.com>");
MODULE_LICENSE("GPL v2");


int FL_ReadReg(int reg)
{
     int val;
    val = LM3646_read_reg(LM3646_i2c_client, reg);
    return (int)val;
}

int FL_WriteReg(int reg, int value)
{
	char buf[2];
	PK_DBG("[LM3646]++++%s++++ reg=0x%2x value=0x%2x\n", __func__, reg, value);
	buf[0] = reg;
	buf[1] = value;
	PK_DBG("[LM3646]----%s----\n", __func__);

    LM3646_write_reg(LM3646_i2c_client, buf[0], buf[1]);
    return 0;
}
