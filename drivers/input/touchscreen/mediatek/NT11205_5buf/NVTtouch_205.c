
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
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
#include <linux/dma-mapping.h>

#include <linux/syscalls.h>
#include <linux/reboot.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/completion.h>
#include <asm/uaccess.h>

#include "NVTtouch_205.h"

#ifdef MT6575
#include <mach/mt6575_pm_ldo.h>
#include <mach/mt6575_typedefs.h>
#include <mach/mt6575_boot.h>
#endif
#ifdef MT6577
#include <mach/mt6577_pm_ldo.h>
#include <mach/mt6577_typedefs.h>
#include <mach/mt6577_boot.h>
#endif
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

#include "tpd.h"
#include <cust_eint.h>

#ifndef TPD_NO_GPIO
#include "cust_gpio_usage.h"
#endif

#if TP_PROXIMITY
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#endif

#include <mach/mt_pm_ldo.h> 

#if BOOT_UPDATE_FIRMWARE
#include "NVT_firmware_205.h"
static struct workqueue_struct *nvt_fwu_wq;
#endif


#if CHARGER_DETECT
static int b_usb_plugin = 0;
#endif

//extern kal_bool upmu_chr_det(upmu_chr_list_enum chr);
extern struct tpd_device *tpd;

static int tpd_flag = 0;
static int tpd_halt = 0;

static struct task_struct *thread = NULL;


#ifdef TPD_HAVE_BUTTON_TYPE_COOR
static int tpd_keys_down=0;
static int tpd_keys_local[TPD_KEY_NUM] = TPD_KEYS;
static int tpd_keys_dim_local[TPD_KEY_NUM][4] = TPD_KEYS_DIM;
#endif


static DECLARE_WAIT_QUEUE_HEAD(waiter);
static int __init tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __init tpd_i2c_remove(struct i2c_client *client);
static int tpd_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);

#ifdef MT6575
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(kal_uint8 eintno, kal_uint32 ms);
extern kal_uint32 mt65xx_eint_set_sens(kal_uint8 eintno, kal_bool sens);
extern void mt65xx_eint_registration(kal_uint8 eintno, kal_bool Dbounce_En,
                                     kal_bool ACT_Polarity, void (EINT_FUNC_PTR)(void),
                                     kal_bool auto_umask);
#endif
#ifdef MT6577
extern void mt65xx_eint_unmask(unsigned int line);
extern void mt65xx_eint_mask(unsigned int line);
extern void mt65xx_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern unsigned int mt65xx_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt65xx_eint_registration(unsigned int eint_num, unsigned int is_deb_en, unsigned int pol, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif


//tpd i2c
static struct i2c_client *i2c_client = NULL;
struct nvt_ts_data *ts;
static const struct i2c_device_id nt11205_tpd_id[] = {{NVT_I2C_NAME, I2C_BUS_NUMBER},{}};
//static unsigned short force[] = {0, I2C_FW_Address, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short * const forces[] = {force, NULL};
//static struct i2c_client_address_data addr_data = {.forces = forces, };

static struct i2c_board_info __initdata i2c_tpd={ I2C_BOARD_INFO(NVT_I2C_NAME, I2C_FW_Address)};


static struct i2c_driver tpd_i2c_driver = {
	.driver = 
	{
		.name = NVT_I2C_NAME,
		//.owner = THIS_MODULE,
	},
	.probe = tpd_i2c_probe,
	.remove = tpd_i2c_remove,
	.id_table = nt11205_tpd_id,
	.detect = tpd_i2c_detect,
	//.address_data = &addr_data,
};


#if TPD_KEY_NUM > 0
void tpd_nt11205_key_init(void)
{
    int i = 0;

#ifdef TPD_HAVE_BUTTON_TYPE_COOR
	tpd_button_setting(TPD_KEY_NUM, tpd_keys_local, tpd_keys_dim_local);
#else
    for(i=0;i<TPD_KEY_NUM;i++)
    {
        __set_bit(touch_key_array[i], tpd->dev->keybit);
    }
#endif
}
#endif



static int i2c_read_bytes( struct i2c_client *client, u8 addr, u8 *rxbuf, int len )
{
    u8 retry;
    u16 left = len;
    u16 offset = 0;

    if ( rxbuf == NULL )
    {
        return TPD_FAIL;
    }

    //TPD_DMESG("i2c_read_bytes to device %02X address %04X len %d\n", client->addr, addr, len );
	
    while ( left > 0 )
    {
        if ( left > MAX_TRANSACTION_LENGTH )
        {
            rxbuf[offset] = ( addr+offset ) & 0xFF;
            i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG | I2C_WR_FLAG | I2C_RS_FLAG;
            retry = 0;
            while ( i2c_master_send(i2c_client, &rxbuf[offset], (MAX_TRANSACTION_LENGTH << 8 | 1)) < 0 )
           //while ( i2c_smbus_read_i2c_block_data(i2c_client, offset,8,&rxbuf[offset] )
            {
                retry++;

                if ( retry == 5 )
                {
                    i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;
                    TPD_DMESG("I2C read 0x%X length=%d failed\n", addr + offset, MAX_TRANSACTION_LENGTH);
                    return -1;
                }
            }
            left -= MAX_TRANSACTION_LENGTH;
            offset += MAX_TRANSACTION_LENGTH;
        }
        else
        {

            //rxbuf[0] = addr;
            rxbuf[offset] = ( addr+offset ) & 0xFF;
            i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG | I2C_WR_FLAG | I2C_RS_FLAG;

            retry = 0;
			//while ( i2c_smbus_read_i2c_block_data(i2c_client, offset,left,&rxbuf[offset] )
            while ( i2c_master_send(i2c_client, &rxbuf[offset], (left<< 8 | 1)) < 0 )
            {
                retry++;

                if ( retry == 5 )
                {
                    i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;
                    TPD_DMESG("I2C read 0x%X length=%d failed\n", addr + offset, left);
                    return TPD_FAIL;
                }
            }
            left = 0;
        }
    }

    i2c_client->addr = i2c_client->addr & I2C_MASK_FLAG;

    return TPD_OK;
}


static int i2c_write_bytes( struct i2c_client *client, u16 addr, u8 *txbuf, int len )
{
    u8 buffer[MAX_TRANSACTION_LENGTH];
    u16 left = len;
    u16 offset = 0;
    u8 retry = 0;

    struct i2c_msg msg =
    {
        .addr = ((client->addr&I2C_MASK_FLAG )|(I2C_ENEXT_FLAG )),
        .flags = 0,
        .buf = buffer
    };

    if ( txbuf == NULL )
    {
        return TPD_FAIL;
    }

   	//TPD_DEBUG("i2c_write_bytes to device %02X address %04X len %d\n", client->addr, addr, len );

    while ( left > 0 )
    {
        retry = 0;
        buffer[0] = ( addr+offset ) & 0xFF;

        if ( left > MAX_I2C_TRANSFER_SIZE )
        {
            memcpy( &buffer[I2C_DEVICE_ADDRESS_LEN], &txbuf[offset], MAX_I2C_TRANSFER_SIZE );
            msg.len = MAX_TRANSACTION_LENGTH;
            left -= MAX_I2C_TRANSFER_SIZE;
            offset += MAX_I2C_TRANSFER_SIZE;
        }
        else
        {
            memcpy( &buffer[I2C_DEVICE_ADDRESS_LEN], &txbuf[offset], left );
            msg.len = left + I2C_DEVICE_ADDRESS_LEN;
            left = 0;
        }

        TPD_DEBUG("byte left %d offset %d\n", left, offset );

        while ( i2c_transfer( client->adapter, &msg, 1 ) != 1 )
        {
            retry++;
            if ( retry == 5 )
            {
                TPD_DEBUG("I2C write 0x%X%X length=%d failed\n", buffer[0], buffer[1], len);
                return TPD_FAIL;
            }
            else
        	{
             	TPD_DEBUG("I2C write retry %d addr 0x%X%X\n", retry, buffer[0], buffer[1]);
        	}
        }
    }

    return TPD_OK;
}



static int tpd_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	strcpy(info->type, NVT_I2C_NAME);
	return TPD_OK;
}

static void nvt_hw_reset(void)
{
	//---trigger rst-pin to reset (pull low for 10ms)---	
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	msleep(20);
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);  
    msleep(10);
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
}



/*******************************************************
  Create Device Node (Proc Entry)
*******************************************************/
#if NVT_TOUCH_CTRL_DRIVER
static struct proc_dir_entry *NVT_proc_entry;
#define DEVICE_NAME	"NVTflash"

int nvt_flash_write(struct file *file, const char __user *buff, size_t count, loff_t *offp)
{
	struct i2c_msg msgs[2];	
	char *str;
	int ret = -1;
	int retries = 0;
	unsigned char tmpaddr;

	//TPD_DMESG("tpd nvt_flash_write\n");
	
	file->private_data = (uint8_t *)kmalloc(64, GFP_KERNEL);
	str = file->private_data;
	if(copy_from_user(str, buff, count))
		return -EFAULT;

	//TPD_DMESG("tpd str[0]=%x, str[1]=%x, str[2]=%x, str[3]=%x\n", str[0], str[1], str[2], str[3]);
	
	tmpaddr = i2c_client->addr;
	i2c_client->addr = str[0];

	//---change sw_reset to hw_reset---
	/*if(str[0]==0x70)
	{
		if(str[2]==0x00 && str[3]==0x5A)
		{
			nvt_hw_reset();
			return 1;
		}
	}*/
	
	while(retries < 20)
	{
		//ret = i2c_smbus_write_i2c_block_data(i2c_client, str[2], str[1]-1, &str[3]);
		ret = i2c_write_bytes(i2c_client, str[2], &str[3], str[1]-1);

		if(ret == TPD_OK)
			break;
		else
			printk("%s error, retries=%d\n", __func__, retries);

		retries++;
	}	
	i2c_client->addr = tmpaddr;
	
	return ret;
}

int nvt_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
	struct i2c_msg msgs[2];	 
	char *str;
	int ret = -1;
	int retries = 0;
	unsigned char tmpaddr;

	//TPD_DMESG("tpd nvt_flash_read\n");

	file->private_data = (uint8_t *)kmalloc(64, GFP_KERNEL);
	str = file->private_data;
	if(copy_from_user(str, buff, count))
		return -EFAULT;

	//TPD_DMESG("tpd str[0]=%x, str[1]=%x, str[2]=%x, str[3]=%x\n", str[0], str[1], str[2], str[3]);
		
	tmpaddr = i2c_client->addr;
	i2c_client->addr = str[0];

	while(retries < 20)
	{
		ret = i2c_read_bytes(i2c_client, str[2], &str[3], str[1]-1);
		if(ret == TPD_OK)
			break;
		else
			printk("%s error, retries=%d\n", __func__, retries);

		retries++;
	}
	i2c_client->addr = tmpaddr;

	// copy buff to user if i2c transfer 	
	if(retries < 20)
	{
		if(copy_to_user(buff, str, count))
			return -EFAULT;
	}
	
	return ret;
}

int nvt_flash_open(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev;

	dev = kmalloc(sizeof(struct nvt_flash_data), GFP_KERNEL);
	if(dev == NULL)
		return -ENOMEM;

	rwlock_init(&dev->lock);
	file->private_data = dev;

	return 0;
}

int nvt_flash_close(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev = file->private_data;

	if(dev)
		kfree(dev);
	
	return 0;   
}

struct file_operations nvt_flash_fops = {
	.owner = THIS_MODULE,
	.open = nvt_flash_open,
	.release = nvt_flash_close,
	.write = nvt_flash_write,
	.read = nvt_flash_read,
};

static int nvt_flash_proc_init(void)
{
	int ret=0;
	
  	NVT_proc_entry = proc_create(DEVICE_NAME, 0666, NULL,&nvt_flash_fops);
	if(NVT_proc_entry == NULL)
	{
		printk("%s: couldn't create proc entry!\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	else
	{
		printk("%s: create proc entry success!\n", __func__);
		//NVT_proc_entry->proc_fops = &nvt_flash_fops;
	}
	printk("============================================================\n");
	printk("Create /proc/NVTflash\n");
	printk("============================================================\n");
	return 0;
}
#endif


#if TP_PROXIMITY
#define TPD_PROXIMITY_ENABLE_REG	0xA4
static u8 tpd_proximity_flag = 0;
static u8 tpd_proximity_detect = 1;	//0-->close ; 1--> far away

static s32 tpd_proximity_get_value(void)
{
    return tpd_proximity_detect;
}

static s32 tpd_proximity_enable(s32 enable)
{
    u8 state;
    s32 ret = -1;
    
	TPD_DMESG("tpd_proximity_enable enable=%d\n",enable);
	
    if (enable)
    {
        state = 1;
        tpd_proximity_flag = 1;
        TPD_DMESG("tpd_proximity_enable on.\n");
    }
    else
    {
        state = 0;
        tpd_proximity_flag = 0;
        TPD_DMESG("tpd_proximity_enable off.\n");
    }

    ret = i2c_write_bytes(i2c_client, TPD_PROXIMITY_ENABLE_REG, &state, 2);
    if (ret < 0)
    {
        TPD_DMESG("tpd %s proximity cmd failed.\n", state ? "enable" : "disable");
        return ret;
    }

    TPD_DMESG("tpd proximity function %s success.\n", state ? "enable" : "disable");
    return 0;
}

s32 tpd_proximity_operate(void *self, u32 command, void *buff_in, s32 size_in,
                   void *buff_out, s32 size_out, s32 *actualout)
{
    s32 err = 0;
    s32 value;
    hwm_sensor_data *sensor_data;

    printk("tpd_proximity_operate enter... ");

    switch (command)
    {
        case SENSOR_DELAY:
            if ((buff_in == NULL) || (size_in < sizeof(int)))
            {
                TPD_DMESG("tpd Set delay parameter error!");
                err = -EINVAL;
            }

            // Do nothing
            break;

        case SENSOR_ENABLE:
            if ((buff_in == NULL) || (size_in < sizeof(int)))
            {
                TPD_DMESG("tpd Enable sensor parameter error!");
                err = -EINVAL;
            }
            else
            {
                value = *(int *)buff_in;
                err = tpd_proximity_enable(value);
            }

            break;

        case SENSOR_GET_DATA:
            if ((buff_out == NULL) || (size_out < sizeof(hwm_sensor_data)))
            {
                TPD_DMESG("tpd Get sensor data parameter error!");
                err = -EINVAL;
            }
            else
            {
                sensor_data = (hwm_sensor_data *)buff_out;
                sensor_data->values[0] = tpd_proximity_get_value();
                sensor_data->value_divide = 1;
                sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
            }
            break;

        default:
            TPD_DMESG("tpd proximy sensor operate function no this parameter %d!\n", command);
            err = -1;
            break;
    }

    return err;
}

static int tpd_proximity_event(u8 proximity_status)
{
	int ret = 0;
    hwm_sensor_data sensor_data;
    u8 point_data[20];

	TPD_DMESG("tpd_proximity_flag = %d, proximity_status = %d\n", tpd_proximity_flag, proximity_status);

	if (tpd_proximity_flag == 1)
	{
		if (proximity_status == 0x03)	//Proximity is far	
		{
			tpd_proximity_detect = 1;
		}
		else if (proximity_status == 0x01)	//Proximity is near
		{
			tpd_proximity_detect = 0;
		}

		TPD_DMESG("tpd PROXIMITY STATUS:0x%02X\n", tpd_proximity_detect);
		
		//map and store data to hwm_sensor_data
		sensor_data.values[0] = tpd_proximity_get_value();
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
		
		//report to the up-layerhwmsen	
		ret = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data);
		if (ret)
		{
			TPD_DMESG("tpd Call hwmsen_get_interrupt_data fail = %d\n", ret);
		}
	}

	return ret;
}

int tpd_proximity_init(void)
{
    int err = 0;
	struct hwmsen_object obj_ps;

    //obj_ps.self = cm3623_obj;
    obj_ps.polling = 0;         //0--interrupt mode; 1--polling mode;
    obj_ps.sensor_operate = tpd_proximity_operate;

    if ((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
    {
        TPD_DMESG("tpd hwmsen attach fail, return:%d.", err);
    }
}
#endif


#if WAKEUP_GESTURE
#define GESTURE_WORD_C			12
#define GESTURE_WORD_W			13
#define GESTURE_WORD_V			14
#define GESTURE_DOUBLE_CLICK	15
#define GESTURE_WORD_Z			16
#define GESTURE_WORD_M			17
#define GESTURE_WORD_O			18
#define GESTURE_WORD_e			19
#define GESTURE_WORD_S			20
#define GESTURE_SLIDE_UP		21
#define GESTURE_SLIDE_DOWN		22
#define GESTURE_SLIDE_LEFT		23
#define GESTURE_SLIDE_RIGHT		24

static unsigned char bTouchIsAwake=1;

void nvt_ts_wakeup_gesture_report(unsigned char gesture_id)
{
	unsigned int keycode=0;

	TPD_DMESG("gesture_id = %d\n", gesture_id);

	switch(gesture_id)
	{
		case GESTURE_WORD_C:
			TPD_DMESG("Gesture : Word-C.\n");
			keycode = gesture_key_array[0];
			break;
		case GESTURE_WORD_W:
			TPD_DMESG("Gesture : Word-W.\n");
			keycode = gesture_key_array[1];
			break;
		case GESTURE_WORD_V:
			TPD_DMESG("Gesture : Word-V.\n");
			keycode = gesture_key_array[2];
			break;
		case GESTURE_DOUBLE_CLICK:
			TPD_DMESG("Gesture : Double Click.\n");
			keycode = gesture_key_array[3];
			break;
		case GESTURE_WORD_Z:
			TPD_DMESG("Gesture : Word-Z.\n");
			keycode = gesture_key_array[4];
			break;
		case GESTURE_WORD_M:
			TPD_DMESG("Gesture : Word-M.\n");
			keycode = gesture_key_array[5];
			break;
		case GESTURE_WORD_O:
			TPD_DMESG("Gesture : Word-O.\n");
			keycode = gesture_key_array[6];
			break;
		case GESTURE_WORD_e:
			TPD_DMESG("Gesture : Word-e.\n");
			keycode = gesture_key_array[7];
			break;
		case GESTURE_WORD_S:
			TPD_DMESG("Gesture : Word-S.\n");
			keycode = gesture_key_array[8];
			break;
		case GESTURE_SLIDE_UP:
			TPD_DMESG("Gesture : Slide UP.\n");
			keycode = gesture_key_array[9];
			break;
		case GESTURE_SLIDE_DOWN:
			TPD_DMESG("Gesture : Slide DOWN.\n");
			keycode = gesture_key_array[10];
			break;
		case GESTURE_SLIDE_LEFT:
			TPD_DMESG("Gesture : Slide LEFT.\n");
			keycode = gesture_key_array[11];
			break;
		case GESTURE_SLIDE_RIGHT:
			TPD_DMESG("Gesture : Slide RIGHT.\n");
			keycode = gesture_key_array[12];
			break;
		default:
			break;
	}

	if(keycode > 0)
	{
		input_report_key(tpd->dev, keycode, 1);
		input_sync(tpd->dev);
		input_report_key(tpd->dev, keycode, 0);
		input_sync(tpd->dev);
	}

	msleep(250);
}
#endif


#if CHARGER_DETECT
#define TPD_CHARGER_STATE_REG	0xE8
void tpd_usb_plugin(int plugin)
{
    u8 state;
	int ret = -1;

	TPD_DMESG("tpd_usb_plugin usb detect: b_usb_plugin=%d, tpd_halt=%d.\n", b_usb_plugin, tpd_halt);
	if(tpd_halt)
        return;
	
	switch(plugin) {
        case 0:
            TPD_DMESG("tpd No charger.\n");
            state = 0xF0;
            ret = i2c_write_bytes(i2c_client, TPD_CHARGER_STATE_REG, &state, 2);
            if (ret < 0)
            {
                TPD_DMESG("tpd_usb_plugin 0x%02X cmd failed.\n", state);
            }
            break;
        case 1:
            TPD_DMESG("tpd VBUS charger.\n");
            state = 0xF1;
            ret = i2c_write_bytes(i2c_client, TPD_CHARGER_STATE_REG, &state, 2);
            if (ret < 0)
            {
                TPD_DMESG("tpd_usb_plugin 0x%02X cmd failed.\n", state);
            }
            break;
        case 2:
            TPD_DMESG("tpd AC charger.\n");
            state = 0xF2;
            ret = i2c_write_bytes(i2c_client, TPD_CHARGER_STATE_REG, &state, 2);
            if (ret < 0)
            {
                TPD_DMESG("tpd_usb_plugin 0x%02X cmd failed.\n", state);
            }
            break;
        default:
            TPD_DMESG("tpd Others.\n");
            state = 0xF0;
            ret = i2c_write_bytes(i2c_client, TPD_CHARGER_STATE_REG, &state, 2);
            if (ret < 0)
            {
                TPD_DMESG("tpd_usb_plugin 0x%02X cmd failed.\n", state);
            }
            break;
    }
}
EXPORT_SYMBOL(tpd_usb_plugin);
#endif



/*******************************************************
  Auto Update FW in Probe
*******************************************************/
#if BOOT_UPDATE_FIRMWARE
static int nvt_i2c_read_bytes(unsigned char addr, unsigned char reg, unsigned char *data, int len)
{
	unsigned char tmpaddr;
	int ret;
	
	tmpaddr = i2c_client->addr;
	i2c_client->addr = addr;
	ret = i2c_smbus_read_i2c_block_data(i2c_client, reg, len, data);
	i2c_client->addr = tmpaddr;

	if(ret < 0)
	{
		//TPD_DMESG("nvt_i2c_read_bytes failed.\n");
	    return 0;
	}
	else
	{
	    return 1;
	}
}

static int nvt_i2c_write_bytes(unsigned char addr, unsigned char reg, unsigned char *data, int len)
{
	unsigned char tmpaddr;
	int ret;
	
	tmpaddr = i2c_client->addr;
	i2c_client->addr = addr;
	ret = i2c_smbus_write_i2c_block_data(i2c_client, reg, len, data);
	i2c_client->addr = tmpaddr;

	if(ret < 0)
	{
		//TPD_DMESG("nvt_i2c_write_bytes failed.\n");
	    return 0;
	}
	else
	{
	    return 1;
	}
}

int Check_FW_Ver(void)
{
	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	I2C_Buf[0] = 0x78;
	nvt_i2c_read_bytes(I2C_HW_Address, I2C_Buf[0], &I2C_Buf[1], 1);
	
	TPD_DMESG("IC FW Ver = %d\n", I2C_Buf[1]);
	TPD_DMESG("Bin FW Ver = %d\n", BUFFER_DATA[0x7F00]);
	if(I2C_Buf[1]>BUFFER_DATA[0x7F00])
		return 1;
	else
		return 0;
}

int Check_CheckSum(void)
{
	uint8_t I2C_Buf[64];
	uint8_t buf2[64];
	int i, j, k, Retry_Counter=0;
	int addr=0;
	uint8_t addrH, addrL;
	unsigned short RD_Filechksum, WR_Filechksum;

	WR_Filechksum = 0;


	I2C_Buf[0]=0x00;
	I2C_Buf[1]=0x5A;
	nvt_i2c_write_bytes(I2C_HW_Address, I2C_Buf[0], &I2C_Buf[1], 1);

	msleep(1000);


	I2C_Buf[0]=0xFF;
	I2C_Buf[1]=0x3F;
	I2C_Buf[2]=0xE8;
	nvt_i2c_write_bytes(I2C_FW_Address, I2C_Buf[0], &I2C_Buf[1], 2);

	I2C_Buf[0]=0x00;
	I2C_Buf[1]=0xEA;
	nvt_i2c_write_bytes(I2C_FW_Address, I2C_Buf[0], &I2C_Buf[1], 1);

	addr = 0;
	for(i=0;i<(BUFFER_LENGTH)/128;i++)
	{
		for(j=0;j<16;j++)
		{
			unsigned char tmp=0;
			addrH = addr>>8;
			addrL = addr&0xFF;
			for(k=0;k<8;k++)
			{
				tmp+=BUFFER_DATA[i*128+j*8+k];
			}
			tmp = tmp+addrH+addrL+8;
			tmp = (255-tmp)+1;
			WR_Filechksum+=tmp;
			addr+=8;
		}
	}

	msleep(800);

	do
	{
		msleep(10);
		I2C_Buf[0]=0xFF;
		I2C_Buf[1]=0x3F;
		I2C_Buf[2]=0xF8;
		nvt_i2c_write_bytes(I2C_FW_Address, I2C_Buf[0], &I2C_Buf[1], 2);

		buf2[0]=0x00;
		buf2[1]=0x00;
		buf2[2]=0x00;
		buf2[3]=0x00;
		nvt_i2c_read_bytes(I2C_FW_Address, buf2[0], &buf2[1], 3);

		Retry_Counter++;
		msleep(10);

	}while((Retry_Counter<20)&& (buf2[1]!=0xAA));

	//---------------------------------------------------------------------------------------

	if(buf2[1]==0xAA)
	{
		RD_Filechksum=(buf2[2]<<8)+buf2[3];
		if(RD_Filechksum==WR_Filechksum)
		{
			TPD_DMESG("%s : firmware checksum match.\n", __func__);
			return 1;	// checksum match
		}
		else
		{
			TPD_DMESG("%s : firmware checksum not match!!\n", __func__);
			return 0;	// checksum not match
		}
	}
	else
	{
		TPD_DMESG("%s : read firmware checksum timeout!!\n", __func__);
		return -1;	// read checksum failed
	}
}

void Update_Firmware(void)
{
	uint8_t I2C_Buf[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	int i = 0;
	int j = 0;
	unsigned int Flash_Address = 0;
	unsigned int Row_Address = 0;
	uint8_t CheckSum[16]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};	// 128/8 = 16 times ;
	struct i2c_client *client = ts->client;
	int ret;

	//-------------------------------
	// Step1 --> initial BootLoader
	//-------------------------------
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0xA5;
	nvt_i2c_write_bytes(I2C_HW_Address, I2C_Buf[0], &I2C_Buf[1], 1);

	msleep(2);
  
	// Initiate Flash Block
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x00;
	nvt_i2c_write_bytes(I2C_HW_Address, I2C_Buf[0], &I2C_Buf[1], 1);

	msleep(20);

	// Read status
	I2C_Buf[0] = 0x00;
	nvt_i2c_read_bytes(I2C_HW_Address, I2C_Buf[0], &I2C_Buf[1], 1);
	if (I2C_Buf[1] != 0xAA)
	{
		TPD_DMESG("Program: init get status(0x%2X) error.", I2C_Buf[1]);
		return;
	}
	TPD_DMESG("Program: init get status(0x%2X) success.", I2C_Buf[1]);

	//---------------------------------------------------------
 	// Step 2 : Erase 
 	//---------------------------------------------------------
	I2C_Buf[0]=0x00;
	I2C_Buf[1]=0x66;
	I2C_Buf[2]=0x00;
	I2C_Buf[3]=0x0E;
	I2C_Buf[4]=0x01;
	I2C_Buf[5]=0xB4;
	I2C_Buf[6]=0x3D;
	nvt_i2c_write_bytes(I2C_HW_Address, I2C_Buf[0], &I2C_Buf[1], 6);

	while(1)
	{
		msleep(1);
		nvt_i2c_read_bytes(I2C_HW_Address, I2C_Buf[0], &I2C_Buf[1], 1);
		if(I2C_Buf[1]==0xAA)
			break;
	}

	I2C_Buf[0]=0x00;
	I2C_Buf[1]=0x66;
	I2C_Buf[2]=0x00;
	I2C_Buf[3]=0x0F;
	I2C_Buf[4]=0x01;
	I2C_Buf[5]=0xEF;
	I2C_Buf[6]=0x01;
	nvt_i2c_write_bytes(I2C_HW_Address, I2C_Buf[0], &I2C_Buf[1], 6);

	while(1)
	{
		msleep(1);
		nvt_i2c_read_bytes(I2C_HW_Address, I2C_Buf[0], &I2C_Buf[1], 1);
		if(I2C_Buf[1]==0xAA)
			break;
	}
	

	for (i = 0 ; i < BUFFER_LENGTH/4096 ; i++)	// 32K = 8 times
	{
		Row_Address = i * 4096; 															

		// Erase Flash	
		I2C_Buf [0] = 0x00;
		I2C_Buf [1] = 0x33;
		I2C_Buf [2] = (uint8_t)((Row_Address & 0xFF00) >> 8 );	// Address High Byte  
		I2C_Buf [3] = (uint8_t)(Row_Address & 0x00FF);	// Address Low Byte 
		nvt_i2c_write_bytes(I2C_HW_Address, I2C_Buf[0], &I2C_Buf[1], 3);
		msleep(15);	// Delay 15 ms 
			  
		// Read Erase status
		nvt_i2c_read_bytes(I2C_HW_Address, I2C_Buf[0], &I2C_Buf[1], 1);
			  
		// check status        
		if (I2C_Buf[1] != 0xAA)
		{
			TPD_DMESG("Program: erase(0x%02X) error.", I2C_Buf[1]);
			return;
		}			
	}

	TPD_DMESG("Program: erase(0x%02X) success.", I2C_Buf[1]);

	Flash_Address = 0;
        		
	//////////////////////////////////////////////////////////////////////////////////// 		
	//----------------------------------------
	// Step3. Host write 128 bytes to IC  
	//----------------------------------------
	TPD_DMESG("Program: write begin, please wait...");

    for(j=0;j<BUFFER_LENGTH/128;j++)
	{
    	Flash_Address=(j)*128;

	   	for (i = 0 ; i < 16*4 ; i++, Flash_Address += 2)	// 128/8 = 16 times for One Row program
		{
    		// write bin data to IC
  			I2C_Buf[0] = 0x00;
			I2C_Buf[1] = 0x55;	//Flash write command
			I2C_Buf[2] = (uint8_t)(Flash_Address  >> 8);	//Flash address [15:8]
			I2C_Buf[3] = (uint8_t)(Flash_Address & 0xFF);	//Flash address [7:0]
			I2C_Buf[4] = 0x02;	//Flash write length (byte)
			I2C_Buf[6] = BUFFER_DATA[Flash_Address + 0];	//Binary data 1
			I2C_Buf[7] = BUFFER_DATA[Flash_Address + 1];	//Binary data 2
			//I2C_Buf[8] = BUFFER_DATA[Flash_Address + 2];	//Binary data 3
			//I2C_Buf[9] = BUFFER_DATA[Flash_Address + 3];	//Binary data 4
			//I2C_Buf[10] = BUFFER_DATA[Flash_Address + 4];   //Binary data 5
			//I2C_Buf[11] = BUFFER_DATA[Flash_Address + 5];	//Binary data 6
			//I2C_Buf[12] = BUFFER_DATA[Flash_Address + 6];	//Binary data 7
			//I2C_Buf[13] = BUFFER_DATA[Flash_Address + 7];	//Binary data 8

			// Calculate a check sum by Host controller.
			// Checksum = / (FLASH_ADRH+FLASH_ADRL+LENGTH+
			//               Binary_Data1+Binary_Data2+Binary_Data3+Binary_Data4+
			//               Binary_Data5+Binary_Data6+Binary_Data7+Binary_Data8) + 1
			//CheckSum[i] = ~(I2C_Buf[2] + I2C_Buf[3] + I2C_Buf[4] + I2C_Buf[6] + I2C_Buf[7] +
            //	          I2C_Buf[8] + I2C_Buf[9] + I2C_Buf[10] + I2C_Buf[11] + I2C_Buf[12] +
            //    	      I2C_Buf[13]) + 1;
			CheckSum[i] = (~(I2C_Buf[2]+I2C_Buf[3]+I2C_Buf[4]+I2C_Buf[6]+I2C_Buf[7]))+1;
			// Load check sum to I2C Buffer
			I2C_Buf[5] = CheckSum[i];
			//nvt_i2c_write_bytes(I2C_HW_Address, I2C_Buf[0], &I2C_Buf[1], 13);
			nvt_i2c_write_bytes(I2C_HW_Address, I2C_Buf[0], &I2C_Buf[1], 7);
		}
		msleep(10);
		
		// Read status
		I2C_Buf[0] = 0x00;
		while(1)
		{
		       msleep(1);
			nvt_i2c_read_bytes(I2C_HW_Address, I2C_Buf[0], &I2C_Buf[1], 1);
			if(I2C_Buf[1]==0xAA)
				break;
		}
    }

	//////////////////////////////////////////////////////////////////////////////////// 		
	//----------------------------------------
	// Step4. Verify  
	//----------------------------------------
	TPD_DMESG("Program: Verify begin, please wait...");
	ret=Check_CheckSum();
	if(ret==1)
		TPD_DMESG("Program: Verify Pass!");
	else if(ret==0)
		TPD_DMESG("Program: Verify NG!");
	else if(ret==-1)
		TPD_DMESG("Program: Verify FW not return!");

	//---write i2c command to reset---			
	//I2C_Buf[0] = 0x00;
	//I2C_Buf[1] = 0x5A;
	//CTP_I2C_WRITE(ts->client, I2C_HW_Address, I2C_Buf, 2);
	
	//---trigger rst-pin to reset---
	nvt_hw_reset();

	msleep(500);
	TPD_DMESG("Program: END");
}

void Boot_Update_Firmware(struct work_struct *work)
{
	struct i2c_client *client = ts->client;
	int ret=0;
	
	ret = Check_CheckSum();

	nvt_hw_reset();
	msleep(500);

	if(ret==-1) // read firmware checksum failed
	{
		Update_Firmware();
	}
	else if(ret==0&&(Check_FW_Ver()==0))	// (fw checksum not match) && (bin fw version > ic fw version)
	{
		TPD_DMESG("%s : firmware version not match.\n", __func__);
		Update_Firmware();
	}

}
#endif





static void tpd_down(int id, int x, int y, int w, int p)
{
#if MT_PROTOCOL_B
	input_mt_slot(tpd->dev, id);
	input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, true);
	input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);	
#else	// TPD_REPORT_XY_MODE == MODE_A
    input_report_abs(tpd->dev, ABS_MT_PRESSURE, 100);
    input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 100);
	
	input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);

    input_report_key(tpd->dev, BTN_TOUCH, 1);
    input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
    input_mt_sync(tpd->dev);
#endif
}

static void tpd_up(int id, int x, int y, int w, int p)
{
#if MT_PROTOCOL_B
	input_mt_slot(tpd->dev, id);
	input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
	//input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
#else	// TPD_REPORT_XY_MODE == MODE_A
	input_report_key(tpd->dev, BTN_TOUCH, 0);

	//input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
	//input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	//input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	//input_report_abs(tpd->dev, ABS_MT_PRESSURE, 0);
	//input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 0);
	//input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR, 0);
	//input_report_abs(tpd->dev, ABS_MT_PRESSURE,	0);
	input_mt_sync(tpd->dev);
#endif
}

void tpd_eint_interrupt_handler(void)
{
    TPD_DEBUG_PRINT_INT;
	tpd_flag = 1;
	wake_up_interruptible(&waiter);
}

static void tpd_work_func(void)
{
	unsigned char point_state[TPD_MAX_POINTS_NUM] = {0};
	unsigned char buf[62] = {0};
	unsigned char checksum = 0;
	unsigned char point_count = 0;	
	int index, pos, id;
	int ret, ret1, ret2,ret3, ret4, ret5, ret6;
	unsigned int x=0, y=0, w=0, p=0;

	TPD_DMESG("tpd_work_func\n");


//	ret1 = i2c_read_bytes(i2c_client, 0, &buf[0], 8);
//	ret2 = i2c_read_bytes(i2c_client, 8, &buf[8], 8);
//	ret3 = i2c_read_bytes(i2c_client, 16, &buf[16], 8);
//	ret4 = i2c_read_bytes(i2c_client, 24, &buf[24], 8);
//	printk("-----------@lgx----------------3----------=%x,=%x,=%x,=%x,=%x,=%x,\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);
	ret1 = i2c_read_bytes(i2c_client, 0, &buf[0], 62);
/*	
	printk("i2c_read_bytes 0~5   = 0x%2X,0x%2X,0x%2X,0x%2X,0x%2X,0x%2X\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);
	printk("i2c_read_bytes 6~11  = 0x%2X,0x%2X,0x%2X,0x%2X,0x%2X,0x%2X\n",buf[6],buf[7],buf[8],buf[9],buf[10],buf[11]);
	printk("i2c_read_bytes 12~17 = 0x%2X,0x%2X,0x%2X,0x%2X,0x%2X,0x%2X\n",buf[12],buf[13],buf[14],buf[15],buf[16],buf[17]);
	printk("i2c_read_bytes 18~23 = 0x%2X,0x%2X,0x%2X,0x%2X,0x%2X,0x%2X\n",buf[18],buf[19],buf[20],buf[21],buf[22],buf[23]);
	printk("i2c_read_bytes 24~29 = 0x%2X,0x%2X,0x%2X,0x%2X,0x%2X,0x%2X\n",buf[24],buf[25],buf[26],buf[27],buf[28],buf[29]);
	printk("i2c_read_bytes 30~35 = 0x%2X,0x%2X,0x%2X,0x%2X,0x%2X,0x%2X\n",buf[30],buf[31],buf[32],buf[33],buf[34],buf[35]);
*/

	id = (unsigned int)(buf[0]>>3);

#if WAKEUP_GESTURE
	if(bTouchIsAwake == 0)
	{	
		nvt_ts_wakeup_gesture_report(id);
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
		return;
	}
#endif


#if TP_PROXIMITY
	ret = tpd_proximity_event(buf[0]&0x07);
#endif

		
	for(index = 0; index < TPD_MAX_POINTS_NUM; index++)
	{
		pos = 6*index;
		id = (unsigned int)(buf[pos+0]>>3) - 1;

		if((buf[pos]&0x07) == 0x03) // finger up (break)
		{
			continue;//input_report_key(ts->input_dev, BTN_TOUCH, 0);
		}
		else if(((buf[pos]&0x07) == 0x01) || ((buf[pos]&0x07) == 0x02)) //finger down (enter&moving)
		{	
			x = (unsigned int)(buf[pos+1]<<4) + (unsigned int) (buf[pos+3]>>4);
			y = (unsigned int)(buf[pos+2]<<4) + (unsigned int) (buf[pos+3]&0x0f);

			if((x < 0) || (y < 0))
				continue;
			if((x > TPD_MAX_WIDTH)||(y > TPD_MAX_HEIGHT))
				continue;

			//TPD_DMESG("tpd_do---wn, x=%d, y=%d\n", x, y);
			tpd_down(id, x, y, w, p);

			point_count++;
		}
	}	
	
	if(point_count == 0)
	{
#ifdef TPD_HAVE_BUTTON_TYPE_COOR
		if(tpd_keys_down==0)
			tpd_up(0,0,0,0,0);
#else
		tpd_up(0,0,0,0,0);
#endif
	}
	
	input_sync(tpd->dev);
	
		
#if TPD_KEY_NUM > 0
	#ifdef TPD_HAVE_BUTTON_TYPE_COOR
	if(buf[60]==0xF8)
	{
		// key released
		if(buf[61]==0)
		{
			tpd_keys_down=0;
			if(point_count == 0)
			{
				tpd_up(0,0,0,0,0);
				input_sync(tpd->dev);
				TPD_DMESG("key released\n");
			}		
		}

		// key[0] pressed
		if((buf[61]>>0)&(0x01))
		{
			tpd_keys_down=1;
			tpd_down(0,90,1340,60,60);
			input_sync(tpd->dev);
			TPD_DMESG("key[0] pressed\n");
		}			
		
		// key[1] pressed
		if((buf[61]>>1)&(0x01))
		{
			tpd_keys_down=1;
			tpd_down(0,270,1340,60,60);
			input_sync(tpd->dev);
			TPD_DMESG("key[1] pressed\n");
		}

		// key[2] pressed
		if((buf[61]>>2)&(0x01))
		{
			tpd_keys_down=1;
			tpd_down(0,450,1340,60,60);
			input_sync(tpd->dev);
			TPD_DMESG("key[1] pressed\n");
		}

		// key[3] pressed
		if((buf[61]>>3)&(0x01))
		{
			tpd_keys_down=1;
			tpd_down(0,630,1340,60,60);
			input_sync(tpd->dev);
			TPD_DMESG("key[1] pressed\n");
		}		
		
//		input_sync(tpd->dev);
	}			
	#else		
	if(buf[60]==0xF8)
	{
		for(index=0; index<TPD_KEY_NUM; index++)
		{
			input_report_key(tpd->dev, touch_key_array[index], ((buf[61]>>index)&(0x01)));
		}
	}
	else
	{
		for(index=0; index<TPD_KEY_NUM; index++)
		{
			input_report_key(tpd->dev, touch_key_array[index], 0);
		}
	}
		input_sync(tpd->dev);
	#endif		
#endif
}


static int tpd_event_handler(void *unused)
{
    struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };

    sched_setscheduler(current, SCHED_RR, &param);

	do
	{
		mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		set_current_state(TASK_INTERRUPTIBLE);
		
		while (tpd_halt)
		{
			tpd_flag = 0;
			msleep(20);
		} 
		
		wait_event_interruptible(waiter, tpd_flag != 0);
		
		tpd_flag = 0;
		TPD_DEBUG_SET_TIME;
		set_current_state(TASK_RUNNING);

		if ( tpd == NULL || tpd->dev == NULL )
		{
			continue;
		}

		// tpd work process function
		tpd_work_func();
		
    }
	while (!kthread_should_stop());

    return TPD_OK;
}

static int __init tpd_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err=0;
	int retry = 0;
	int retry0 = 0,ret = -1;;
	unsigned char buf[] = {0};

	TPD_DMESG("MediaTek touch panel i2c probe\n");
    TPD_DMESG("probe handle -- novatek\n");
	ts = kmalloc(sizeof(struct nvt_ts_data), GFP_KERNEL);
	i2c_client = client;

	//if(0 == tpd_load_status)
	//{
		TPD_DMESG("kpd_i2c_probe \n");
#if defined(GPIO_CTP_EN_PIN)
    // power on CTP
    mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ONE);

#else   // ( defined(MT6575) || defined(MT6577) || defined(MT6589) )

    #ifdef TPD_POWER_SOURCE_CUSTOM
        hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
    #else
        hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
    #endif
    #ifdef TPD_POWER_SOURCE_1800
        hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
    #endif

#endif
	nvt_hw_reset();

	// set INT mode
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
	//mt65xx_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	//mt_eint_set_sens(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_SENSITIVE);
	//mt65xx_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	//mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	//mt65xx_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN, CUST_EINT_TOUCH_PANEL_POLARITY, tpd_eint_interrupt_handler, 1); 
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, tpd_eint_interrupt_handler, 1);
	//mt65xx_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
	msleep(100);
	
	for(retry0=5; retry0>=0; retry0--)
	{
		ret = i2c_smbus_read_i2c_block_data(client, I2C_HW_Address, 5, buf);
		dev_info(&client->dev, "buf[0]=%d, buf[1]=%d, buf[2]=%d, buf[3]=%d, buf[4]=%d\n",
			buf[0], buf[1], buf[2], buf[3], buf[4]);
		
		if(ret <= 0)	// i2c read failed
		{
			dev_info(&client->dev,  "i2c read test failed at retry=%d.\n", retry0);
		}
		else	// i2c read succeed
		{
			dev_info(&client->dev,  "i2c read test succeed.\n");
			break;
		}

		if(retry0 == 0)
			return -1;	
	}
 
	thread = kthread_run(tpd_event_handler, 0, TPD_DEVICE);
    if (IS_ERR(thread))
	{
        err = PTR_ERR(thread);
        TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", err);
    }
	
    msleep(1000);

    tpd_load_status = 1;

    TPD_DMESG("MediaTek touch panel i2c probe success\n");

#if TP_PROXIMITY
    tpd_proximity_init();
#endif

#if WAKEUP_GESTURE
	for(retry = 0; retry < (sizeof(gesture_key_array)/sizeof(gesture_key_array[0])); retry++)
	{
		input_set_capability(tpd->dev, EV_KEY, gesture_key_array[retry]);
	}
#endif

#if NVT_TOUCH_CTRL_DRIVER
	nvt_flash_proc_init();
#endif

#if BOOT_UPDATE_FIRMWARE
	nvt_fwu_wq = create_singlethread_workqueue("nvt_fwu_wq");
	if(!nvt_fwu_wq)
	{
		printk("%s : nvt_fwu_wq create workqueue failed.\n", __func__);
		return -ENOMEM; 
	}
	INIT_DELAYED_WORK(&ts->nvt_fwu_work, Boot_Update_Firmware);
	queue_delayed_work(nvt_fwu_wq, &ts->nvt_fwu_work, msecs_to_jiffies(4000));
#endif

    return TPD_OK;
}

static int __init tpd_i2c_remove(struct i2c_client *client)
{
	TPD_DMESG("call func tpd_i2c_remove\n");
    return TPD_OK;
}

int tpd_local_init(void)
{
	TPD_DMESG("tpd_local_init  start 0\n");

    if(i2c_add_driver(&tpd_i2c_driver)!=0)
	{
    	TPD_DMESG("unable to add i2c driver.\n");
    	return TPD_FAIL;
    }

 	input_set_abs_params(tpd->dev, ABS_MT_TRACKING_ID, 0, 5, 0, 0);
	
#if WAKEUP_GESTURE
	input_set_capability(tpd->dev, EV_KEY, KEY_POWER);
#endif
	
	TPD_DMESG("tpd_local_init  start 1\n");
    if(tpd_load_status == 0)
    {
    	TPD_DMESG("add error touch panel driver.\n");
    	i2c_del_driver(&tpd_i2c_driver);
    	return TPD_FAIL;
    }
	
	TPD_DMESG("tpd_local_init start 2\n");
	
#if TPD_KEY_NUM > 0
	tpd_nt11205_key_init();
#endif

    TPD_DMESG("end %s, %d\n", __func__, __LINE__);

    tpd_type_cap = 1;

    return TPD_OK;
}

/* Function to manage low power suspend */
void tpd_suspend(struct i2c_client *client, pm_message_t message)
{
	int retval = TPD_OK;
	u8 buf[4]={0};

	
	TPD_DEBUG("call function tpd_suspend\n");

    tpd_halt = 1;

#if TP_PROXIMITY
	if(tpd_proximity_flag == 1)
	{
		return;
	}
#endif


#if WAKEUP_GESTURE
	bTouchIsAwake = 0;
	TPD_DMESG("Enable touch wakeup gesture.\n");

	//---write i2c command to enter "wakeup gesture mode"---
	buf[0]=0x88;
	buf[1]=0x55;
	buf[2]=0xAA;
	buf[3]=0xA6;
	i2c_smbus_write_i2c_block_data(i2c_client, buf[0], 3, &buf[1]);
#else
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	//---write i2c command to enter "deep sleep mode"---
	buf[0]=0x88;
	buf[1]=0x55;
	buf[2]=0xAA;
	buf[3]=0xA5;
	i2c_smbus_write_i2c_block_data(i2c_client, buf[0], 3, &buf[1]);
#endif

	return retval;
}

/* Function to manage power-on resume */
void tpd_resume(struct i2c_client *client)
{
    TPD_DEBUG("call function tpd_resume\n");

#if TP_PROXIMITY
	if(tpd_proximity_flag == 1)
	{
		return;
	}
#endif

	tpd_halt = 0;

#if WAKEUP_GESTURE
	nvt_hw_reset();
	bTouchIsAwake = 1;
#else
	nvt_hw_reset();
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
#endif


#if CHARGER_DETECT
	msleep(200);
	tpd_usb_plugin(b_usb_plugin);
#endif
}

static struct tpd_driver_t tpd_device_driver = 
{
	.tpd_device_name = NVT_I2C_NAME,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
#if TPD_KEY_NUM > 0
	.tpd_have_button = 1,
#else
	.tpd_have_button = 0,
#endif
};

/* called when loaded into kernel */
static int __init tpd_driver_init(void)
{
    TPD_DMESG("touch panel driver init tpd_driver_init\n");
	
	i2c_register_board_info(I2C_BUS_NUMBER, &i2c_tpd, 1);
    if(tpd_driver_add(&tpd_device_driver) < 0)
    {
    	TPD_DMESG("add generic driver failed\n");
    }

    return TPD_OK;
}

/* should never be called */
static void __exit tpd_driver_exit(void)
{
    TPD_DMESG("touch panel driver exit tpd_driver_exit\n");

    tpd_driver_remove(&tpd_device_driver);
}

module_init(tpd_driver_init);
module_exit(tpd_driver_exit);


