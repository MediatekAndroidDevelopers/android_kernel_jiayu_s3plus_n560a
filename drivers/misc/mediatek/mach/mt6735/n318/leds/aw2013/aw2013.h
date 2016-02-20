#ifndef __BREATHLIGHT_H__
#define __BREATHLIGHT_H__

#include <linux/ioctl.h>

/*************************************************
*
**************************************************/
#define	AW2013_REG_CHIP_ONFF  0x01
#define	AW2013_REG_PWM_OUT	 0x30
#define AW2013_REG_RST 0xff


//In KERNEL mode,SHOULD be sync with mediatype.h
//CHECK before remove or modify
//#undef BOOL
//#define BOOL signed int
#ifndef _MEDIA_TYPES_H
typedef unsigned char MUINT8;
typedef unsigned short MUINT16;
typedef unsigned int MUINT32;
typedef signed char MINT8;
typedef signed short MINT16;
typedef signed int MINT32;
#endif


/* cotta-- added for high current solution */

/* cotta-- time limit of strobe watch dog timer. unit : ms */

typedef struct {
	int (*breathlight_open) (void *pArg);
	int (*breathlight_release) (void *pArg);
	int (*breathlight_ioctl) (MUINT32 cmd, MUINT32 arg);
} BREATHLIGHT_FUNCTION_STRUCT, *PBREATHLIGHT_FUNCTION_STRUCT;

typedef struct {
	MUINT32 breathlightId;
	 MUINT32(*breathlightInit) (PBREATHLIGHT_FUNCTION_STRUCT * pfFunc);
} KD_BREATHLIGHT_INIT_FUNCTION_STRUCT,
    *pKD_BREATHLIGHT_INIT_FUNCTION_STRUCT;

#define BREATHLIGHT_MAGIC					0x8A
#define BREATHLIGHT_IOCTL_INIT				_IO(BREATHLIGHT_MAGIC,  0x01)
#define BREATHLIGHT_IOCTL_MODE				_IOW(BREATHLIGHT_MAGIC, 0x02, int)
#define BREATHLIGHT_IOCTL_CURRENT			_IOW(BREATHLIGHT_MAGIC, 0x03, int)
#define BREATHLIGHT_IOCTL_ONOFF				_IOW(BREATHLIGHT_MAGIC, 0x04, int)
#define BREATHLIGHT_IOCTL_PWM_ONESHOT		_IOW(BREATHLIGHT_MAGIC, 0x05, int)
#define BREATHLIGHT_IOCTL_TIME				_IOW(BREATHLIGHT_MAGIC, 0x06, int)
extern void led_flash_aw2013( unsigned int id );
extern void led_breath_turnoff();
extern void led_test(int test_num);
#endif
