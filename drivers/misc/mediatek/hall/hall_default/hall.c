#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/interrupt.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/ctype.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/workqueue.h>
#include <linux/switch.h>
#include <linux/delay.h>

#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/kthread.h>
#include <linux/time.h>

#include <linux/string.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_reg_base.h>
#include <mach/irqs.h>

#include "hall.h"
#include <mach/mt_boot.h>
#include <cust_eint.h>
#include <cust_gpio_usage.h>
#include <mach/mt_gpio.h>
#include <mach/battery_common.h>

#ifdef CONFIG_KERNEL_HALL_MAD_SUPPORT
#include <linux/input.h>
#include <linux/input/hall.h>
#endif

//#define GPIO_HALL_EINT_PIN GPIO116	//move to dct
//#define CUST_EINT_HALL_NUM 11		//move to dct

int hall_cur_eint_state = HALL_FAR;
#if defined(VANZO_COMMON_APPLE_CHARGING)
int usb_cur_eint_state = USB_PLUG_OUT;
#endif

extern void mt_eint_mask(unsigned int eint_num);                                                                                                                         
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms); 
extern void mt_eint_set_polarity(unsigned int eint_num, unsigned int pol);
extern unsigned int mt_eint_set_sens(unsigned int eint_num, unsigned int sens);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
extern void mt_eint_print_status(void);

static struct workqueue_struct * hall_eint_workqueue = NULL;
static struct work_struct hall_eint_work;

static struct switch_dev hall_data;

static struct input_dev * hall_pwrdev;
static DEFINE_MUTEX(pwrkeyworklock);

#if defined(VANZO_COMMON_APPLE_CHARGING)
static struct workqueue_struct * apple_eint_workqueue = NULL;
static struct work_struct apple_eint_work;
extern void apple_set_charging_state(INT32 event);
extern void apple_boost_voltage_on(int is_on);
extern void apple_pchr_turn_on_charging(BOOL is_enable);
int apple_vbus_android_connect_vbus_usb_flag=1;
void apple_unmask_eint_plug_in(void)
{
	usb_cur_eint_state=mt_get_gpio_in(GPIO_PLUG_IN_EINT_PIN);
	mt_eint_set_polarity(CUST_EINT_PLUG_IN_NUM, 1-usb_cur_eint_state);
	mt_eint_unmask(CUST_EINT_PLUG_IN_NUM);
}
void apple_mask_eint_plug_in(void)
{
	mt_eint_mask(CUST_EINT_PLUG_IN_NUM);
}
void apple_vbus_android_disconnect_vbus_iphone(void)
{
	mt_set_gpio_mode(GPIO_EN_I6_CHAR_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_EN_I6_CHAR_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_EN_I6_CHAR_PIN, GPIO_OUT_ZERO);
}
void apple_vbus_android_connect_vbus_iphone(void)
{
	mt_set_gpio_mode(GPIO_EN_I6_CHAR_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_EN_I6_CHAR_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_EN_I6_CHAR_PIN, GPIO_OUT_ONE);
}
void apple_vbus_android_disconnect_vbus_usb(void)
{
	mt_set_gpio_mode(GPIO_EN_VBUS_USB_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_EN_VBUS_USB_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_EN_VBUS_USB_PIN, GPIO_OUT_ONE);
#if !defined(VANZO_COMMON_APPLE_CHARGING_NEW_STYLE)
	apple_unmask_eint_plug_in();
	apple_vbus_android_connect_vbus_usb_flag=0;
#endif
}
void apple_vbus_android_connect_vbus_usb(void)
{
	mt_set_gpio_mode(GPIO_EN_VBUS_USB_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_EN_VBUS_USB_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_EN_VBUS_USB_PIN, GPIO_OUT_ZERO);
#if !defined(VANZO_COMMON_APPLE_CHARGING_NEW_STYLE)
	apple_vbus_android_connect_vbus_usb_flag=1;
#endif
}
void apple_dataline_android_connect_dataline_iphone(void)
{
	mt_set_gpio_mode(GPIO_SW_EN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_SW_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SW_EN_PIN, GPIO_OUT_ZERO);

	mt_set_gpio_mode(GPIO_SW_SEL_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_SW_SEL_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SW_SEL_PIN, GPIO_OUT_ONE);
}
void apple_dataline_android_connect_dataline_usb(void)
{
	mt_set_gpio_mode(GPIO_SW_EN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_SW_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SW_EN_PIN, GPIO_OUT_ZERO);

	mt_set_gpio_mode(GPIO_SW_SEL_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_SW_SEL_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SW_SEL_PIN, GPIO_OUT_ZERO);
}
void apple_dataline_iphone_short_circuit(void)
{
	mt_set_gpio_mode(GPIO_SW2_EN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_SW2_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SW2_EN_PIN, GPIO_OUT_ZERO);

	mt_set_gpio_mode(GPIO_SW2_SEL_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_SW2_SEL_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SW2_SEL_PIN, GPIO_OUT_ONE);
}
void apple_dataline_iphone_no_short_circuit(void)
{
	mt_set_gpio_mode(GPIO_SW2_EN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_SW2_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SW2_EN_PIN, GPIO_OUT_ZERO);

	mt_set_gpio_mode(GPIO_SW2_SEL_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_SW2_SEL_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_SW2_SEL_PIN, GPIO_OUT_ZERO);
}
void apple_otg_open(void)
{
	mt_set_gpio_mode(GPIO_USB_ID_LOW_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_USB_ID_LOW_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_USB_ID_LOW_PIN, GPIO_OUT_ONE);
#if defined(VANZO_COMMON_APPLE_CHARGING_NEW_STYLE)
	mt_set_gpio_mode(GPIO_EN_I6_CHAR_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_EN_I6_CHAR_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_EN_I6_CHAR_PIN, GPIO_OUT_ONE);
#endif
}
void apple_otg_close(void)
{
	mt_set_gpio_mode(GPIO_USB_ID_LOW_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_USB_ID_LOW_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_USB_ID_LOW_PIN, GPIO_OUT_ZERO);
#if defined(VANZO_COMMON_APPLE_CHARGING_NEW_STYLE)
	mt_set_gpio_mode(GPIO_EN_I6_CHAR_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_EN_I6_CHAR_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_EN_I6_CHAR_PIN, GPIO_OUT_ZERO);
#endif
}
void apple_android_only(void)
{
	apple_vbus_android_disconnect_vbus_iphone();
	apple_dataline_iphone_no_short_circuit();
	apple_dataline_android_connect_dataline_usb();
	apple_otg_close();
	apple_vbus_android_connect_vbus_usb();
}
void apple_android_iphone_idle(void)
{
	apple_vbus_android_disconnect_vbus_iphone();
	apple_vbus_android_disconnect_vbus_usb();
	apple_dataline_iphone_no_short_circuit();
	apple_dataline_android_connect_dataline_usb();
	apple_otg_close();
}
void apple_android_iphone_charging(void)
{
	apple_vbus_android_disconnect_vbus_usb();
	apple_dataline_android_connect_dataline_usb();
	apple_dataline_iphone_short_circuit();
	apple_otg_open();
	apple_vbus_android_connect_vbus_iphone();
}
void apple_android_iphone_communication(void)
{
	apple_vbus_android_disconnect_vbus_usb();
	apple_dataline_iphone_no_short_circuit();
	apple_dataline_android_connect_dataline_iphone();
	apple_otg_open();
	apple_boost_voltage_on(1);
	apple_vbus_android_connect_vbus_iphone();
}
void apple_android_iphone_communication_charging(void)
{
	apple_vbus_android_connect_vbus_usb();
	apple_dataline_iphone_no_short_circuit();
	apple_dataline_android_connect_dataline_iphone();
	apple_otg_open();
	apple_boost_voltage_on(0);
	apple_pchr_turn_on_charging(1);
	apple_vbus_android_connect_vbus_iphone();
}
void apple_ac_android_charging(void)
{
	apple_vbus_android_disconnect_vbus_iphone();
	apple_dataline_android_connect_dataline_usb();
	apple_otg_close();
	apple_vbus_android_connect_vbus_usb();
}
void apple_ac_iphone_charging(void)
{
	apple_vbus_android_disconnect_vbus_usb();
	apple_vbus_android_disconnect_vbus_iphone();
	apple_dataline_android_connect_dataline_usb();
	apple_dataline_iphone_short_circuit();
	apple_otg_close();
	apple_vbus_android_connect_vbus_iphone();
	apple_vbus_android_connect_vbus_usb();
}
kal_bool apple_usb_is_plug_in(void)
{
	if(usb_cur_eint_state ==  USB_PLUG_IN )
		return KAL_TRUE;
	else
		return KAL_FALSE;
}
void apple_eint_work_callback(struct work_struct *work)
{
	mt_eint_mask(CUST_EINT_PLUG_IN_NUM);
	if(usb_cur_eint_state ==  USB_PLUG_IN )
	{
		printk("entry apple_eint_func: USB_PLUG_IN_EVENT\n");
#if defined(VANZO_COMMON_APPLE_CHARGING_NEW_STYLE)
#else
		apple_set_charging_state(USB_PLUG_IN_EVENT);
#endif
	}
	else
	{
		printk("entry apple_eint_func: USB_PLUG_OUT_EVENT\n");
#if defined(VANZO_COMMON_APPLE_CHARGING_NEW_STYLE)
#else
		apple_set_charging_state(USB_PLUG_OUT_EVENT);
#endif
	}
	mt_eint_unmask(CUST_EINT_PLUG_IN_NUM);
}
void apple_eint_func(void)
{
	int ret;
	if(usb_cur_eint_state ==  USB_PLUG_IN )
	{
		mt_eint_set_polarity(CUST_EINT_PLUG_IN_NUM, 1);
		usb_cur_eint_state = USB_PLUG_OUT;
	}
	else
	{
		mt_eint_set_polarity(CUST_EINT_PLUG_IN_NUM, 0);
		usb_cur_eint_state = USB_PLUG_IN;
	}

	ret = queue_work(apple_eint_workqueue, &apple_eint_work); 
}
static inline int apple_setup_eint(void)
{
	HALL_FUNC();
	usb_cur_eint_state=mt_get_gpio_in(GPIO_PLUG_IN_EINT_PIN);

	mt_set_gpio_mode(GPIO_PLUG_IN_EINT_PIN, GPIO_PLUG_IN_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_PLUG_IN_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_PLUG_IN_EINT_PIN, GPIO_PULL_DISABLE);

    //mt65xx_eint_set_sens(CUST_EINT_HALL_NUM, CUST_EINT_HALL_SENSITIVE);
	mt_eint_set_hw_debounce(CUST_EINT_PLUG_IN_NUM,CUST_EINT_PLUG_IN_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_PLUG_IN_NUM, usb_cur_eint_state?CUST_EINTF_TRIGGER_LOW:CUST_EINTF_TRIGGER_HIGH,  apple_eint_func, 0);
#if defined(VANZO_COMMON_APPLE_CHARGING_NEW_STYLE)
	mt_eint_unmask(CUST_EINT_PLUG_IN_NUM);
#endif

	return 0;
}
#endif
void hall_eint_work_callback(struct work_struct *work)
{
#ifdef CONFIG_KERNEL_HALL_MAD_SUPPORT
        int key_code = 0;
#endif
	HALL_FUNC();
        mt_eint_mask(CUST_EINT_HALL_NUM);
	if(hall_cur_eint_state == HALL_NEAR)
	{
		HALL_DEBUG("HALL_NEAR\n");
#if defined(VANZO_COMMON_APPLE_CHARGING)
#if defined(VANZO_COMMON_APPLE_CHARGING_NEW_STYLE)
	apple_dataline_android_connect_dataline_iphone();
	apple_otg_open();
#else
	apple_android_iphone_communication();
	apple_set_charging_state(HALL_NERA_EVENT);
#endif
#endif
                switch_set_state((struct switch_dev *)&hall_data, HALL_NEAR);
#ifdef CONFIG_KERNEL_HALL_MAD_SUPPORT
                key_code = KEY_F2; //doze to display clock
#endif
	}
	else
	{
		HALL_DEBUG("HALL_FAR\n");
#if defined(VANZO_COMMON_APPLE_CHARGING)
#if defined(VANZO_COMMON_APPLE_CHARGING_NEW_STYLE)
	apple_otg_close();
	apple_dataline_android_connect_dataline_usb();
#else
	apple_android_only();
	apple_set_charging_state(HALL_FAR_EVENT);
#endif
#endif
                switch_set_state((struct switch_dev *)&hall_data, HALL_FAR);
#ifdef CONFIG_KERNEL_HALL_MAD_SUPPORT
                key_code = KEY_F3; // power on
#endif
	}
#ifdef CONFIG_KERNEL_HALL_MAD_SUPPORT
        if (mutex_trylock(&pwrkeyworklock) && key_code != 0) {
    	        input_event(hall_pwrdev, EV_KEY, key_code, 1);
    	        input_event(hall_pwrdev, EV_SYN, 0, 0);
    	        msleep(60);
    	        input_event(hall_pwrdev, EV_KEY, key_code, 0);
    	        input_event(hall_pwrdev, EV_SYN, 0, 0);
	        mutex_unlock(&pwrkeyworklock);
        }
#endif
        mt_eint_unmask(CUST_EINT_HALL_NUM);
}

void hall_eint_func(void)
{
	int ret;
	
	HALL_FUNC();
	if(hall_cur_eint_state ==  HALL_FAR ) 
	{
		mt_eint_set_polarity(CUST_EINT_HALL_NUM, 1);
		hall_cur_eint_state = HALL_NEAR;
	}
	else
	{
		mt_eint_set_polarity(CUST_EINT_HALL_NUM, 0);
		hall_cur_eint_state = HALL_FAR;
	}

	ret = queue_work(hall_eint_workqueue, &hall_eint_work); 
}

static inline int hall_setup_eint(void)
{
	HALL_FUNC();
	
	mt_set_gpio_mode(GPIO_HALL_EINT_PIN, GPIO_HALL_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_HALL_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_HALL_EINT_PIN, GPIO_PULL_DISABLE);

    //mt65xx_eint_set_sens(CUST_EINT_HALL_NUM, CUST_EINT_HALL_SENSITIVE);
	mt_eint_set_hw_debounce(CUST_EINT_HALL_NUM,CUST_EINT_HALL_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_HALL_NUM, hall_cur_eint_state?CUST_EINTF_TRIGGER_LOW:CUST_EINTF_TRIGGER_HIGH,  hall_eint_func, 0);
	mt_eint_unmask(CUST_EINT_HALL_NUM);  

	return 0;
}

static int hall_probe(struct platform_device *dev)
{
	int ret = 0;
	
	HALL_FUNC();

	bool curr_state;
	mt_set_gpio_mode(GPIO_HALL_EINT_PIN, 0);
	mt_set_gpio_dir(GPIO_HALL_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_HALL_EINT_PIN, GPIO_PULL_DISABLE);
	curr_state = mt_get_gpio_in(GPIO_HALL_EINT_PIN);
	printk("%s line %d curr_state %d\n",__func__,__LINE__, curr_state);

	if(!curr_state){
		hall_data.name = "hall";
		hall_data.index = 0;
		hall_data.state = HALL_FAR;
		hall_cur_eint_state = HALL_FAR;
	}else{
		hall_data.name = "hall";
		hall_data.index = 0;
		hall_data.state = HALL_NEAR;
		hall_cur_eint_state = HALL_NEAR;
	}

	ret = switch_dev_register(&hall_data);
	if(ret)
	{
		HALL_DEBUG("switch_dev_register return %d\n", ret);
	}

	hall_eint_workqueue = create_singlethread_workqueue("hall_eint");
	INIT_WORK(&hall_eint_work, hall_eint_work_callback);

	hall_setup_eint();
#if defined(VANZO_COMMON_APPLE_CHARGING)
	apple_eint_workqueue = create_singlethread_workqueue("apple_eint");
	INIT_WORK(&apple_eint_work, apple_eint_work_callback);
	apple_setup_eint();
#endif
#ifdef CONFIG_KERNEL_HALL_MAD_SUPPORT
	input_set_capability(hall_pwrdev, EV_KEY, KEY_F2);
	input_set_capability(hall_pwrdev, EV_KEY, KEY_F3);
#endif
	
	return 0;
}

#ifdef CONFIG_KERNEL_HALL_MAD_SUPPORT
/* PowerKey setter */
void hall_setdev(struct input_dev * input_device) {
	hall_pwrdev = input_device;
	HALL_DEBUG("set hall_pwrdev: %s\n", hall_pwrdev->name);
}
#endif

static int hall_remove(struct platform_device *dev)
{
	HALL_FUNC();

	destroy_workqueue(hall_eint_workqueue);
	switch_dev_unregister(&hall_data);

	return 0;
}
#if 0
struct platform_device hall_device = {
  .name = "hall_driver",
  .id = -1,
};
#endif
static int hall_suspend(struct platform_device *pdev, pm_message_t mesg)
{
    return 0;
}

static int hall_resume(struct platform_device *pdev)
{
    return 0;
}
#ifdef CONFIG_OF
static const struct of_device_id hall_of_match[] = {
    { .compatible = "mediatek,hall_driver", },
    {},
};
#endif


static struct platform_driver hall_driver = {
	.probe = hall_probe,
	.suspend = hall_suspend,
	.resume = hall_resume,
	.remove = hall_remove,
	.driver = {
		.name = "hall_driver",
        #ifdef CONFIG_OF
        .of_match_table = hall_of_match,
        #endif

	},
};

static int hall_mod_init(void)
{
	int ret = 0;
int retval = 0;
#if 0
	HALL_FUNC();
    retval = platform_device_register(&hall_device);
    printk("register hall device\n");

    if(retval != 0)
    {
      printk("platform_device_register hall error:(%d)\n", retval);
    }
    else
    {
      printk("platform_device_register hall done!\n");
    }
#endif	
	if(platform_driver_register(&hall_driver) != 0)
	{
		HALL_DEBUG("unable to register hall driver\n");
		return -1;
	}
	
	return 0;
}

static void hall_mod_exit(void)
{
	HALL_FUNC();

	platform_driver_unregister(&hall_driver);
}

module_init(hall_mod_init);
module_exit(hall_mod_exit);

MODULE_DESCRIPTION("Vanzo Hall driver");
MODULE_AUTHOR("AL <lubaoquan@vanzotec.com>");
MODULE_LICENSE("GPL");
