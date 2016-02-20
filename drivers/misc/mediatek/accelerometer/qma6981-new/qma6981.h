
#ifndef __QMA6981_H__
#define __QMA6981_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define QMA6981_I2C_SLAVE_ADDR		0x13
#define QMA6981_REG_POWER_CTL		0x11
#define QMA6981_REG_BW_RATE			0x10

#define QMA6981_BW_31HZ             0x63   //19.8
#define QMA6981_BW_62HZ             0x64   //35.7
#define QMA6981_BW_125HZ           0x65   //66.96


#define QMA6981_ERR_I2C                     -1
#define QMA6981_SUCCESS						0


#define QMA6981_OSX		-0.55
#define QMA6981_OSY		4
#define QMA6981_OXZ		2.35
#define QMA6981_SFZY		-0.0102
/************************************************/
/* 	Accelerometer section defines	 	*/
/************************************************/

/* Accelerometer Sensor Full Scale */
#define QMA6981_2G		0x01
#define QMA6981_4G		0x02
#define QMA6981_8G		0x04
#define QMA6981_16G		0x08

#define QMA6981_AXES_NUM        3


/* Magnetic Sensor Operating Mode */
#define QMA6981_NORMAL_MODE	0x00
#define QMA6981_POS_BIAS	0x01
#define QMA6981_NEG_BIAS	0x02
#define QMA6981_CC_MODE		0x00
#define QMA6981_SC_MODE		0x01
#define QMA6981_IDLE_MODE	0x02
#define QMA6981_SLEEP_MODE	0x03

/* Magnetometer output data rate  */
#define QMA6981_ODR_75		0x00	/* 0.75Hz output data rate */
#define QMA6981_ODR1_5		0x04	/* 1.5Hz output data rate */
#define QMA6981_ODR3_0		0x08	/* 3Hz output data rate */
#define QMA6981_ODR7_5		0x0C	/* 7.5Hz output data rate */
#define QMA6981_ODR15		0x10	/* 15Hz output data rate */
#define QMA6981_ODR30		0x14	/* 30Hz output data rate */
#define QMA6981_ODR75		0x18	/* 75Hz output data rate */
#define QMA6981_ODR220		0x1C	/* 220Hz output data rate */


#ifdef __KERNEL__

struct QMA6981_platform_data {

	int irq;	
	
	u8 h_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;

	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

};
#endif /* __KERNEL__ */
extern struct acc_hw* qma6981_get_cust_acc_hw(void); 

#endif  /* __QMA6981_H__ */
