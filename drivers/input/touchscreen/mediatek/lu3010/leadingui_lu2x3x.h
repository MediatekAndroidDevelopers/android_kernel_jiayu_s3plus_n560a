/*
 * leadingui_lu2x3x.h
 *
 * Copyright (C) 2015 LeadingUI Co.,Ltd.
 * Author: jaehan@leadingui.com
 * Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LEAINGUI_LU2X3X_H__
#define __LEAINGUI_LU2X3X_H__

#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>

#define LEADINGUI_TS_NAME			"leadingui_ts"

#define LEADINGUI_I2C_ADDR			(0x1C >> 1)

#define PLATFORMDATA_DEVICETREE		0	/* Declaring Platform data in Device Tree */
#define BUILT_IN_UPGRADE			0
#define LUI_WATCHDOG_TIMER			1

#if (LUI_WATCHDOG_TIMER)
#define LUI_WATCHDOG_TIMEOUT		1000
#endif

#define LUI_MAX_FINGER				10	/* supported max fingers */

#define LUI_EVENT_NO			0x00
#define LUI_EVENT_ABSOLUTE		0x01
#define LUI_EVENT_KEY			0x02
#define LUI_EVENT_GESTURE		0x04
#define LUI_EVENT_RELATIVE		0x08
#define LUI_EVENT_PRESSURE		0x10
#define LUI_EVENT_DEBUG			0x80

enum {
	LUI_TOUCH_PRESS		= 1,
	LUI_TOUCH_MOVE		= 2,
	LUI_TOUCH_RELEASE	= 3
};

enum {
	LUI_KEY_RELEASE		= 1,
	LUI_KEY_PRESS		= 2,
};

enum {
	LUI_TS_POWER_OFF = 0,
	LUI_TS_POWER_ON,
	LUI_TS_POWER_SUSPEND,
	LUI_TS_POWER_RESUME
};

#define DEVICE_EVENT_FAIL_MASK (LUI_EVENT_ABSOLUTE|LUI_EVENT_KEY|LUI_EVENT_PRESSURE)

#define U32_FINGER_X_MASK		0x00000FFF
#define U32_FINGER_Y_MASK		0x00FFF000
#define U32_FINGER_ID_MASK		0x1F000000
#define U32_FINGER_STATUS_MASK	0x60000000

#define FINGER_GET_X_POSITION(x) (x & FINGER_X_MASK)
#define FINGER_GET_Y_POSITION(x) (x & FINGER_Y_MASK >> 12)
#define FINGER_GET_ID        (x) (x & FINGER_ID_MASK >> 24)
#define FINGER_GET_STATUS    (x) (x & FINGER_STATUS_MASK >> 29)

#define FINGER_EVENT_REG	0x0000
#define FINGER_DEBUG_REG	0x0000

/* device id */
#define DEVICE_LU2010		0x2010
#define DEVICE_LU3010		0x3010
#define DEVICE_LU3100		0x3100
#define DEVICE_LU310A		0x310A

/* device type */
enum {
	DT_LU2010 = 0x05,
	DT_LU3010 = 0x06,
	DT_LU3100 = 0x08,
	DT_LU310A = 0x09,
};

/* Power State */
enum {
	POWER_OFF = 0,
	POWER_ON,
	POWER_SUSPEND,
	POWER_WAKE,
};


struct leadingui_platform_data {

	u16 allow_device_id;	/* supported device id */

	int max_x;
	int max_y;
	int max_keys;

	bool support_pressure;
	int max_pressure;		/* This attribute will be ignored if the support_pressure is false. */
	int max_touch_major;
	int max_touch_minor;
	int *key_map;

	bool use_regulator;
	bool use_reset_gpio;
	bool support_suspend;
};

struct touch_data{
	u16 state;
	u16 x;
	u16 y;
	u16 width_major;
	u16 width_minor;
	u16 pressure;
};

struct key_data{
	u8 code;
	int state;
};

struct lu3_touch_event {
	u8 status;
	u8 event;
	u8 tcount;
	u8 key;
}__attribute__ ((packed));

struct lu2_touch_event {
	u8 status;
	u8 event;
	u8 tcount;
	u16 key;
}__attribute__ ((packed));

struct device_pressure_data {
	u8  value;	/* pressure Max histo value */
	u8  count;	/* pressure histo count */
};

#define PRESSURE_DATA_LENGTH	sizeof(struct device_pressure_data);

struct lu3_touch_data {
	u16 x		 :12;
	u16 y		 :12;
	u8  id		 :5;
	u8  status	 :2;
	u8  reserved :1;
	struct device_pressure_data pressure;
}__attribute__ ((packed));

struct lu2_touch_data {
	u16 x		 :11;
	u16 y		 :11;
	u8  id		 :5;
	u8  status	 :2;
	u8  reserved :3;
}__attribute__ ((packed));

/* f/w upgrade mode */
enum {
	BUILT_IN,
	UMS,
};

/* f/w upgrade state */
enum {
	FI_IDLE = 0x0000,
	FI_FILE_READ,
	FI_CODE_READ,
	FI_FW_WRITE,
	FI_FW_READ,
	FI_FW_VERIFY,
};

/* f/w result */
enum {
	FI_RESULT_READY = 0,
	FI_RESULT_OK,
	FI_RESULT_ERROR,
};

#define MAX_PATH				260

#define FI_FILE_READ_MASK		0xFFF0
#define FI_CODE_READ_MASK		0xFFF0
#define FI_FW_WRITE_MASK		0xFF0F
#define FI_FW_READ_MASK			0xF0FF
#define FI_FW_VERIFY_MASK		0x0FFF

struct leadingui_fw_info {
	struct mutex 	mutex;

	u16   version;		/* for mfp file version. */
	u32   checksum; 	/* for mfp file checksum. */

	char fw_name[MAX_PATH];
	u8    *fw_data;
	u16   fw_ver;
	u16   fw_length;

	bool  is_updating;
	u8    state;
	u16   results;
};

struct leadingui_ts_data {
	char					phys[64]; /* device physical location */
	struct leadingui_platform_data *pdata;

	atomic_t				device_init; /* init sync */
	struct i2c_client		*client;

	struct input_dev		*input_dev;
	struct early_suspend	early_suspend;

	u16						device_id;	/* current device id */
	u16						fw_version; /* H: major L:minor */

	struct touch_data		cur_tdata[LUI_MAX_FINGER];
	struct key_data			cur_tkey;

	struct delayed_work		work_init;
	struct work_struct		work_fw_upgrade;
	struct work_struct		work_recover;

	int 					power_state;
	int 					recovery_err_cnt;

	struct leadingui_fw_info fw_info;

#if (LUI_WATCHDOG_TIMER)
	struct timer_list		wd_timer;
	struct work_struct		wd_timer_work;
	bool					touch_reported; /* touch reported flag*/
#endif

	bool 					irq_enabled;
	struct mutex 			mutex_lock;
};

#define LEADINGUI_INFO                  1
#define LEADINGUI_WARNING_MSG           1

#define LEADINGUI_TRACE_MSG             0
#define LEADINGUI_TRACE_FUNC            0
#define LEADINGUI_TRACE_POWER           0
#define LEADINGUI_TRACE_WD_TIMER        0
#define LEADINGUI_TRACE_TOUCH           0

#define LUI_TRACE(fmt, args...)\
	if (LEADINGUI_TRACE_MSG) \
		printk(KERN_ERR "[leadingui] " fmt, ##args)

#define LUI_TRACE_FUNC(fmt, args...)\
	if (LEADINGUI_TRACE_FUNC) \
		printk(KERN_ERR "[leadingui] [%s] " fmt, __FUNCTION__, ##args)

#define LUI_TRACE_POWER(fmt, args...)\
	if (LEADINGUI_TRACE_POWER) \
		printk(KERN_ERR "[leadingui] " fmt, ##args)

#define LUI_TRACE_WD_TIMER(fmt, args...)\
	if (LEADINGUI_TRACE_WD_TIMER) \
		printk(KERN_ERR "[leadingui] " fmt, ##args)

#define LUI_ERROR(fmt, args...)	\
		printk(KERN_ERR "[leadingui] [%s %d] " fmt, __FUNCTION__, __LINE__, ##args)

#define LUI_WARNING(fmt, args...)	\
	if (LEADINGUI_WARNING_MSG) \
		printk(KERN_ERR "[leadingui] [%s %d] " fmt, __FUNCTION__, __LINE__, ##args)

#define LUI_INFO(fmt, args...)\
	if (LEADINGUI_INFO) \
		printk(KERN_ERR "[leadingui] " fmt, ##args)

#define LUI_TOUCH_DEBUG(fmt, args...)\
	if (LEADINGUI_TRACE_TOUCH) \
		printk(KERN_ERR "[leadingui] " fmt, ##args)

#define LUI_REPORT_TIME(fmt, args...)\
	if (CHECK_REPORT_TIME) \
		printk(KERN_ERR "[leadingui] " fmt, ##args)


#endif//__LEAINGUI_LU2X3X_H__
