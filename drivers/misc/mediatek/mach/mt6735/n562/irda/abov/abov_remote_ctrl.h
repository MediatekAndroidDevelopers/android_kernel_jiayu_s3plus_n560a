#ifndef _ABOV_REMOTE_CTRL_H_ 
#define _ABOV_REMOTE_CTRL_H_

struct remote_ctrl_data 
{
    struct i2c_client *client;          // represents the slave device
	unsigned int	   init_gpio;
	unsigned int	   reset_gpio;

	u32 			   init_gpio_flags;
	u32 			   reset_gpio_flags;
};

#define REMOTE_CTRL_SLAVE_ADDR	(0xA0>>1)
//#define REMOTE_CTRL_SLAVE_ADDR	(0xD2>>1) // TEST
#define REMOTE_CTRL_DEV		"remote_ctrl"


#define RC_GPIO_MODE_INPUT			0
#define RC_GPIO_MODE_OUTPUT			1


/* 
 * Magic Number
 * ============
 * Refer to file ioctl-number.txt for allocation
 */
#define REMOTECTRL_IOCTL_MAGIC      		'r'

/* IOCTLs for remote-ctrl device */
#define REMOTE_CTRL_CTL_GET_GPIO_MODE		0x1000
#define REMOTE_CTRL_CTL_SET_GPIO_MODE		0x1001

#define REMOTE_CTRL_CTL_GET_GPIO_VALUE		0x1002
#define REMOTE_CTRL_CTL_SET_GPIO_VALUE		0x1003

#define REMOTE_CTRL_CTL_SET_RESET_VALUE		0x1004

#define REMOTE_CTRL_CTL_MODE_BOOTLOADER		_IOW(REMOTECTRL_IOCTL_MAGIC, 0x05, int)
#define REMOTE_CTRL_CTL_MODE_USERIR			_IOW(REMOTECTRL_IOCTL_MAGIC, 0x06, int)

#define REMOTE_CTRL_WRITE					_IOW(REMOTECTRL_IOCTL_MAGIC, 0x07, int)
#define REMOTE_CTRL_READ					_IOW(REMOTECTRL_IOCTL_MAGIC, 0x08, int)
#endif
