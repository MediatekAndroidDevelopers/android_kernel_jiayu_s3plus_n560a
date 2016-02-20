
#ifndef TOUCHPANEL_H__
#define TOUCHPANEL_H__


//---I2C driver info.---
#define NVT_I2C_NAME	"NVT-ts"
#define I2C_HW_Address	0x70
#define I2C_FW_Address	0x01

#define MAX_TRANSACTION_LENGTH	8
#define I2C_DEVICE_ADDRESS_LEN	1
#define MAX_I2C_TRANSFER_SIZE	(MAX_TRANSACTION_LENGTH - I2C_DEVICE_ADDRESS_LEN)
#define I2C_BUS_NUMBER			1
#define I2C_MASTER_CLOCK		400

#define TPD_POWER_SOURCE_CUSTOM PMIC_APP_CAP_TOUCH_VDD
//---Touch info.---
#define TPD_MAX_WIDTH		720
#define TPD_MAX_HEIGHT		1280
#define TPD_MAX_POINTS_NUM	5
#define TPD_KEY_NUM			3
#if TPD_KEY_NUM > 0
#define TPD_HAVE_BUTTON_TYPE_COOR
#ifdef TPD_HAVE_BUTTON_TYPE_COOR
#define TPD_KEYS	{KEY_MENU, KEY_HOMEPAGE/*KEY_HOME*/, KEY_BACK, KEY_SEARCH}
#define TPD_KEYS_DIM	{{90,1340,60,60},{270,1340,60,60},{450,1340,60,60},{630,1340,60,60}}
#else
const int touch_key_array[TPD_KEY_NUM]={
	KEY_MENU,
	KEY_HOMEPAGE/*KEY_HOME*/,
	KEY_BACK,
};
#endif
#endif


//---Customized func.---
#define NVT_TOUCH_CTRL_DRIVER	1
#define BOOT_UPDATE_FIRMWARE	0
#define MT_PROTOCOL_B			0
#define	CHARGER_DETECT			0
#define TP_PROXIMITY			0
#define WAKEUP_GESTURE			0
#if WAKEUP_GESTURE
const uint16_t gesture_key_array[]={
	KEY_POWER,	//GESTURE_WORD_C
	KEY_POWER,	//GESTURE_WORD_W
	KEY_POWER,	//GESTURE_WORD_V
	KEY_POWER,	//GESTURE_DOUBLE_CLICK
	KEY_POWER,	//GESTURE_WORD_Z
	KEY_POWER,	//GESTURE_WORD_M
	KEY_POWER,	//GESTURE_WORD_O
	KEY_POWER,	//GESTURE_WORD_e
	KEY_POWER,	//GESTURE_WORD_S
	KEY_POWER,	//GESTURE_SLIDE_UP
	KEY_POWER,	//GESTURE_SLIDE_DOWN
	KEY_POWER,	//GESTURE_SLIDE_LEFT
	KEY_POWER,	//GESTURE_SLIDE_RIGHT	
};
#endif

struct nvt_ts_data{
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct nvt_work;
	struct delayed_work nvt_fwu_work;
	uint16_t addr;
	char phys[32];
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t x_num;
	uint8_t y_num;
	uint8_t max_touch_num;
	uint8_t max_button_num;
	uint32_t int_trigger_type;
};


#if NVT_TOUCH_CTRL_DRIVER
struct nvt_flash_data{
	rwlock_t lock;
	unsigned char bufferIndex;
	unsigned int length;
	struct i2c_client *client;
};
#endif


//---TP Driver Return Value---
#define TPD_OK		0
#define TPD_FAIL	(-1)


#endif /* TOUCHPANEL_H__ */


