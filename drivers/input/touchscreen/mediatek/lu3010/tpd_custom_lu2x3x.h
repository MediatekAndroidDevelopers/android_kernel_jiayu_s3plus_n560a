#ifndef TOUCHPANEL_H__
#define TOUCHPANEL_H__

#include <linux/hrtimer.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
//#include <linux/io.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/interrupt.h>
#include <linux/time.h>
#include <linux/rtpm_prio.h>

#include <linux/proc_fs.h>
#include <asm/uaccess.h>

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <mach/mt_gpio.h>

#include <cust_eint.h>
#include <linux/jiffies.h>
#include <pmic_drv.h>
#include <cust_i2c.h>


/* Pre-defined definition */
#define TPD_TYPE_CAPACITIVE
#define TPD_POWER_SOURCE	PMIC_APP_CAP_TOUCH_VDD //MT6323_POWER_LDO_VGP1
#define TPD_I2C_NUMBER		1
#define TPD_I2C_ADDR		(0x1C >> 1)
#define TPD_WAKEUP_TRIAL	60
#define TPD_WAKEUP_DELAY	100


//#define TPD_HAVE_TREMBLE_ELIMINATION

/* Define the virtual button mapping */
//#define TPD_HAVE_BUTTON
#define TPD_BUTTON_HEIGH        (100)
#define TPD_KEY_COUNT           3
#define TPD_KEYS                {KEY_MENU, KEY_HOME, KEY_BACK}
#define TPD_KEYS_DIM            {{80,850,160,TPD_BUTTON_HEIGH}, \
                                {240,850,160,TPD_BUTTON_HEIGH}, \
                                {400,850,160,TPD_BUTTON_HEIGH}}


/* Define the touch dimension */
#ifdef TPD_HAVE_BUTTON
#define TPD_TOUCH_HEIGH_RATIO	80
#define TPD_DISPLAY_HEIGH_RATIO	73
#endif

#endif /* TOUCHPANEL_H__ */

