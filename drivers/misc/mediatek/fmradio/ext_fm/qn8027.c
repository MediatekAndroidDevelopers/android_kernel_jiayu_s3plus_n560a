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
#include <linux/cdev.h>

#if defined(MT6575) || defined(MT6571) || defined(MT6589) || defined(MT6582) || defined(MT6592) || defined(MT6735) || defined(MT6752) || defined(MT6753)
#include <mach/devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif


//add for fix resume issue
#include <linux/earlysuspend.h>
#include <linux/wakelock.h>

/*----------------------------------------------------------------------------*/
#define FM_DEV_NAME "QN8027"
#define FM_DRIVER_VERSION "test_version"
static const struct i2c_device_id fm_i2c_id[] = {{FM_DEV_NAME,0},{}};
static struct i2c_board_info __initdata fm_i2c_info= { I2C_BOARD_INFO(FM_DEV_NAME, (0x58>>1))};
static struct i2c_client * qn8027_client;
static struct cdev * g_qn8027_CharDrv = NULL;
static int poweron;

static int  esky_iReadReg(uint8_t addr, uint8_t *val)
{
    int n;
    char b;

    // first, send addr to AR1000
    n = i2c_master_send(qn8027_client, (char*)&addr, 1);
    if (n < 0)
    {
        printk("QN8027_read send:0x%X err:%d\n", addr, n);
        return -1;
    }

    // second, receive two byte from AR1000
    n = i2c_master_recv(qn8027_client, &b, 1);
    if (n < 0)
    {
        printk("QN8027_read recv:0x%X err:%d\n", addr, n);
        return -1;
    }
    *val = (uint8_t)b;
    return 0;
}

uint8_t esky_iWriteReg( uint8_t addr, uint8_t val)
{
    int n;
    char b[2];

    b[0] = addr;
    b[1] = (char)val;

    n = i2c_master_send(qn8027_client, b, 2);
    if (n < 0)
    {
        printk("QN8027_read send:0x%X err:%d\n", addr, n);
        return -1;
    }

    return 0;
}

void cxF_SetRegBit(uint8_t reg, uint8_t bitMask, uint8_t data_val) 
{
	uint8_t temp;
	esky_iReadReg(reg,&temp);
	temp &= (uint8_t)(~bitMask);
	temp |= data_val;
	esky_iWriteReg(reg, temp);
}

void cxD_TXSetPower(uint8_t gain)   //  调整功率是0x00-------------0x40    左大右小
{
    uint8_t value = 0;
    value |= 0x40;  
    value |= gain;
    esky_iWriteReg(0x1f, value);
}

int qn8027_reg_init()
{
    //change the crystal setting
    mt_set_gpio_mode(GPIO_EXT_FM_EN_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_EXT_FM_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_EXT_FM_EN_PIN, GPIO_OUT_ONE);  

   esky_iWriteReg(0x03,0x10);	// Crystal source
   esky_iWriteReg(0x04,0xB2);	// Crystal:12MHz
   //   cxF_SetRegBit(0x03,0xC0,0x40);	// Digital Clock
   //   cxF_SetRegBit(0x04,0x80,0x80);	// Crystal:24MHz   

   //..................................................
   //chip reset

    cxF_SetRegBit(0x00,0x40,0x40);
    mdelay(200);				//delay 20 ms
    cxF_SetRegBit(0x00,0x40,0x00);
   
    esky_iWriteReg(0x18,0xe4);	// Improve SNR
    esky_iWriteReg(0x1b,0xf0);    // Increase PA max output power

    cxF_SetRegBit(0x00,0x40,0x22);  //TX mode
    //set( pre and de time)、(pa off time)、（TX pilot frequency deviation)

    esky_iWriteReg(0x02,0xb9);


    //set TX frequency deviation

    esky_iWriteReg(0x11,0x81); 
    cxD_TXSetPower(0x00);       //change TX power
    uint8_t ival;
    int i;
    for(i=0x00;i<0x50;i++)
    {
	esky_iReadReg(i,&ival);
	printk("esky_iReadReg %x->%x \n",i,ival);
    }
    return 1;
}

void power_on(int i)
{
    poweron=i;
    mt_set_gpio_mode(GPIO_EXT_FM_EN_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_EXT_FM_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_EXT_FM_EN_PIN, GPIO_OUT_ONE);  

}
/*
static int QN8027_SetCh(u16 freq)
{
    /// calculate ch para
    //printk("=====================  %s, %d \n", __func__, __LINE__);
    u8 tStep;
    u8 tCh;
    u16 f; 
    
    f = FREQ2CHREG(freq); 
    // set to reg: CH
    tCh = (u8) f;
    //cxD_WriteReg(CH, tCh);
    esky_iWriteReg(CH, tCh);
    // set to reg: CH_STEP
    //tStep = cxD_ReadReg(CH_STEP);
    esky_iReadReg(CH_STEP,&tStep);
    tStep &= ~CH_CH;
    tStep |= ((u8) (f >> 8) & CH_CH);
    //cxD_WriteReg(CH_STEP, tStep);
    esky_iWriteReg(CH_STEP, tStep);
}
*/

static size_t fm_store_open(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	printk("[fm_dev_open] E\n");

	qn8027_reg_init();
	return size;
}

static DEVICE_ATTR(open, 0666, NULL, fm_store_open);
/*
static struct device_attribute *fm_attr_list[] = {
	&dev_attr_open,
	&dev_attr_time,
};
*/
static int fm_ops_open(struct inode *inode, struct file *filp)
{

    if(poweron)
    {
          printk("qn8027 dev is already open\n");
          return 0;
    }
    power_on(1);
    msleep(5);
    if(qn8027_reg_init()){
	printk("qn8027_init success \n");
    }
    else
    {
	printk("qn8027_init fail  \n");
	return -1;
    }
    return 0;
}
static long fm_ops_ioctl_qn(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;

    printk("%s\n", __func__);
/*
    switch(cmd)
    {
        case FM_IOCTL_POWER:
        {
            break;
        }
        case FreqSet:
        {       
           cur_freq = arg;
           //QN8027_SetCh(cur_freq);
           break;
        }
        default:
            return -EPERM;
    }*/
    return ret;
}

static int fm_ops_release(struct inode *inode, struct file *filp)
{
    int err = 0;
    power_on(0);    
    return err;
}

static struct file_operations fm_ops = {
    .owner = THIS_MODULE,
    .open = fm_ops_open,
    .release = fm_ops_release,
    .unlocked_ioctl = fm_ops_ioctl_qn,
};

static int fm_init(struct i2c_client *client)
{
    int err;
    uint8_t value=0;
    struct device* qn8027_device = NULL;
    struct class *qn8027_class = NULL;
    dev_t g_qn8027_devno;
    qn8027_client=client;
    power_on(0);

    printk("==================== %s %d \n", __func__, __LINE__);

    err = alloc_chrdev_region(&g_qn8027_devno, 0, 1, FM_DEV_NAME);
    if (err) {
        printk("alloc dev_t failed\n");
        return -1;
    }

    g_qn8027_CharDrv = cdev_alloc();
    if(NULL == g_qn8027_CharDrv)
    {
        unregister_chrdev_region(g_qn8027_devno, 1);
        return -ENOMEM;
    }

    cdev_init(g_qn8027_CharDrv, &fm_ops);

    g_qn8027_CharDrv->owner = THIS_MODULE;

    err = cdev_add(g_qn8027_CharDrv, g_qn8027_devno, 1);
    if (err) {
        printk("alloc dev_t failed\n");
        return -1;
    }

    qn8027_class = class_create(THIS_MODULE, FM_DEV_NAME);
    if (IS_ERR(qn8027_class)) {
        err = PTR_ERR(qn8027_class);
        printk("class_create err:%d\n", err);
        return err;            
    }
    qn8027_device = device_create(qn8027_class, NULL, g_qn8027_devno, NULL, FM_DEV_NAME);
    err = device_create_file(qn8027_device, &dev_attr_open);
    if(err){
	    printk("create enable file for qn8027 err!\n"); 
            return err;
    }

    //printk("  %d i2c_client driver_name:%s,0x01 value=%x \n",__FILE__, qn8027_client->name,value);
    
    printk("qn8027_i2c_probe: end ,driver probe OK \n");
    
    msleep(5);
    
    
    return 0;
}

static s32 fm_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
        int err;
	uint8_t value=0;
	u16 version_info;
	printk(" ===================== %s , %d \n", __func__ , __LINE__);

        if ((err = fm_init(client)))
        {
                printk("fm_init ERR:%d\n", err);
                goto ERR_EXIT;
        }

        return 0;

ERR_EXIT:
        return err;
}

static int fm_i2c_remove(struct i2c_client *client)
{
	return 0;
}
static int fm_i2c_detect(struct i2c_client *client,struct i2c_board_info *info)
{
    printk("fm_i2c_detect\n");
    strcpy(info->type, FM_DEV_NAME);
    return 0;
}

kal_uint8 g_reg_value_qn8027=0;

static struct i2c_driver qn8027_i2c_driver =
{
    .probe     	= fm_i2c_probe,
    .remove     = fm_i2c_remove,
    .detect     = fm_i2c_detect,
    //.suspend    = epl_sensor_i2c_suspend,
    //.resume     = epl_sensor_i2c_resume,
    .id_table   = fm_i2c_id,
    //.address_data = &epl_sensor_addr_data,
    .driver = {
        //.owner  = THIS_MODULE,
        .name   = FM_DEV_NAME,
    },
};

static int qn8027_probe(struct platform_device *pdev) 
{
    printk("%s !!!\n", __func__);

    i2c_add_driver(&qn8027_i2c_driver);
    return 0;
}
static int qn8027_remove(struct platform_device *pdev)
{
    printk("%s !!!\n", __func__);

    i2c_del_driver(&qn8027_i2c_driver);
    return 0;
}
static int qn8027_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    return 0;
}

static int qn8027_resume(struct platform_device *pdev)
{
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id qn8027_of_match[] = {
    { .compatible = "mediatek,QN8027", },
    {},
};
#endif

// platform structure
static struct platform_driver g_qn8027_driver = {
    .probe      = qn8027_probe,
    .remove	= qn8027_remove,
    .suspend = qn8027_suspend,
    .resume = qn8027_resume,
    .driver		= {
        .name	= "QN8027",
        .owner	= THIS_MODULE,
        #ifdef CONFIG_OF
        .of_match_table = qn8027_of_match,
        #endif

    }
};

static int __init qn8027_init(void)
{
    printk("%s: fm driver version: %s\n", __func__, FM_DRIVER_VERSION);
   
    int err = -1;
    int ret=0;
    
    i2c_register_board_info(3, &fm_i2c_info, 1);
    
    if(platform_driver_register(&g_qn8027_driver)){
        printk("failed to register qn8027 driver\n");
        return -ENODEV;
    }
    return 0;
}

static void __exit qn8027_exit(void)
{
	platform_driver_unregister(&g_qn8027_driver);
}

module_init(qn8027_init);
module_exit(qn8027_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("QN8027 I2C Driver");
MODULE_AUTHOR("lhm@dx.com");
MODULE_VERSION(FM_DRIVER_VERSION);
