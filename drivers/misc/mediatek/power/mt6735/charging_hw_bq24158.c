/*****************************************************************************
 *
 * Filename:
 * ---------
 *    charging_pmic.c
 *
 * Project:
 * --------
 *   ALPS_Software
 *
 * Description:
 * ------------
 *   This file implements the interface between BMT and ADC scheduler.
 *
 * Author:
 * -------
 *  Oscar Liu
 *
 *============================================================================
  * $Revision:   1.0  $
 * $Modtime:   11 Aug 2005 10:28:16  $
 * $Log:   //mtkvs01/vmdata/Maui_sw/archives/mcu/hal/peripheral/inc/bmt_chr_setting.h-arc  $
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <mach/charging.h>
#include "bq24158.h"
#include <mach/upmu_common.h>
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#include <mach/upmu_hw.h>
#include <linux/xlog.h>
#include <linux/delay.h>
#include <mach/mt_sleep.h>
#include <mach/mt_boot.h>
#include <mach/system.h>
#include <cust_charging.h>

 // ============================================================ //
 //define
 // ============================================================ //
#define STATUS_OK	0
#define STATUS_UNSUPPORTED	-1
#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))


 // ============================================================ //
 //global variable
 // ============================================================ //
static CHARGER_TYPE g_charger_type = CHARGER_UNKNOWN;
#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
#define WIRELESS_CHARGER_EXIST_STATE 0
int wireless_charger_gpio_number   = (168 | 0x80000000); 
#endif

#if 1
#include <cust_gpio_usage.h>
int gpio_number   = GPIO_SWCHARGER_EN_PIN; 
int gpio_off_mode = GPIO_SWCHARGER_EN_PIN_M_GPIO;
int gpio_on_mode  = GPIO_SWCHARGER_EN_PIN_M_GPIO;
#else
int gpio_number   = (19 | 0x80000000); 
int gpio_off_mode = 0;
int gpio_on_mode  = 0;
#endif
int gpio_off_dir  = GPIO_DIR_OUT;
int gpio_off_out  = GPIO_OUT_ONE;
int gpio_on_dir   = GPIO_DIR_OUT;
int gpio_on_out   = GPIO_OUT_ZERO;

kal_bool charging_type_det_done = KAL_TRUE;

const kal_uint32 VBAT_CV_VTH[]=
{
	BATTERY_VOLT_03_500000_V,   BATTERY_VOLT_03_520000_V,	BATTERY_VOLT_03_540000_V,   BATTERY_VOLT_03_560000_V,
	BATTERY_VOLT_03_580000_V,   BATTERY_VOLT_03_600000_V,	BATTERY_VOLT_03_620000_V,   BATTERY_VOLT_03_640000_V,
	BATTERY_VOLT_03_660000_V,	BATTERY_VOLT_03_680000_V,	BATTERY_VOLT_03_700000_V,	BATTERY_VOLT_03_720000_V,
	BATTERY_VOLT_03_740000_V,	BATTERY_VOLT_03_760000_V,	BATTERY_VOLT_03_780000_V,	BATTERY_VOLT_03_800000_V,
	BATTERY_VOLT_03_820000_V,	BATTERY_VOLT_03_840000_V,	BATTERY_VOLT_03_860000_V,	BATTERY_VOLT_03_880000_V,
	BATTERY_VOLT_03_900000_V,	BATTERY_VOLT_03_920000_V,	BATTERY_VOLT_03_940000_V,	BATTERY_VOLT_03_960000_V,
	BATTERY_VOLT_03_980000_V,	BATTERY_VOLT_04_000000_V,	BATTERY_VOLT_04_020000_V,	BATTERY_VOLT_04_040000_V,
	BATTERY_VOLT_04_060000_V,	BATTERY_VOLT_04_080000_V,	BATTERY_VOLT_04_100000_V,	BATTERY_VOLT_04_120000_V,
	BATTERY_VOLT_04_140000_V,   BATTERY_VOLT_04_160000_V,	BATTERY_VOLT_04_180000_V,   BATTERY_VOLT_04_200000_V,
	BATTERY_VOLT_04_220000_V,   BATTERY_VOLT_04_240000_V,	BATTERY_VOLT_04_260000_V,   BATTERY_VOLT_04_280000_V,
	BATTERY_VOLT_04_300000_V,   BATTERY_VOLT_04_320000_V,	BATTERY_VOLT_04_340000_V,   BATTERY_VOLT_04_360000_V,	
	BATTERY_VOLT_04_380000_V,   BATTERY_VOLT_04_400000_V,	BATTERY_VOLT_04_420000_V,   BATTERY_VOLT_04_440000_V	
	
};

const kal_uint32 CS_VTH[]=
{
	CHARGE_CURRENT_550_00_MA,   CHARGE_CURRENT_650_00_MA,	CHARGE_CURRENT_750_00_MA, CHARGE_CURRENT_850_00_MA,
	CHARGE_CURRENT_950_00_MA,   CHARGE_CURRENT_1050_00_MA,	CHARGE_CURRENT_1150_00_MA, CHARGE_CURRENT_1250_00_MA
}; 

 const kal_uint32 INPUT_CS_VTH[]=
 {
	 CHARGE_CURRENT_100_00_MA,	 CHARGE_CURRENT_500_00_MA,	 CHARGE_CURRENT_800_00_MA, CHARGE_CURRENT_MAX
 }; 

 const kal_uint32 VCDT_HV_VTH[]=
 {
	  BATTERY_VOLT_04_200000_V, BATTERY_VOLT_04_250000_V,	  BATTERY_VOLT_04_300000_V,   BATTERY_VOLT_04_350000_V,
	  BATTERY_VOLT_04_400000_V, BATTERY_VOLT_04_450000_V,	  BATTERY_VOLT_04_500000_V,   BATTERY_VOLT_04_550000_V,
	  BATTERY_VOLT_04_600000_V, BATTERY_VOLT_06_000000_V,	  BATTERY_VOLT_06_500000_V,   BATTERY_VOLT_07_000000_V,
	  BATTERY_VOLT_07_500000_V, BATTERY_VOLT_08_500000_V,	  BATTERY_VOLT_09_500000_V,   BATTERY_VOLT_10_500000_V		  
 };

static kal_uint32 charging_error = false;
 // ============================================================ //
 // function prototype
 // ============================================================ //

static kal_uint32 charging_get_error_state(void);
static kal_uint32 charging_set_error_state(void *data);
 
 // ============================================================ //
 //extern variable
 // ============================================================ //
 
 // ============================================================ //
 //extern function
 // ============================================================ //
 extern bool mt_usb_is_device(void);
 extern void Charger_Detect_Init(void);
 extern void Charger_Detect_Release(void);
 extern int hw_charging_get_charger_type(void);
 extern void mt_power_off(void);
 
 extern bool get_usb_current_unlimited(void);
 
 // ============================================================ //
 kal_uint32 charging_value_to_parameter(const kal_uint32 *parameter, const kal_uint32 array_size, const kal_uint32 val)
{
	if (val < array_size)
	{
		return parameter[val];
	}
	else
	{
		battery_xlog_printk(BAT_LOG_CRTI, "Can't find the parameter \r\n");	
		return parameter[0];
	}
}

 
 kal_uint32 charging_parameter_to_value(const kal_uint32 *parameter, const kal_uint32 array_size, const kal_uint32 val)
{
	kal_uint32 i;

	for(i=0;i<array_size;i++)
	{
		if (val == *(parameter + i))
		{
				return i;
		}
	}

    battery_xlog_printk(BAT_LOG_CRTI, "NO register value match \r\n");
	//TODO: ASSERT(0);	// not find the value
	return 0;
}


 static kal_uint32 bmt_find_closest_level(const kal_uint32 *pList,kal_uint32 number,kal_uint32 level)
 {
	 kal_uint32 i;
	 kal_uint32 max_value_in_last_element;
 
	 if(pList[0] < pList[1])
		 max_value_in_last_element = KAL_TRUE;
	 else
		 max_value_in_last_element = KAL_FALSE;
 
	 if(max_value_in_last_element == KAL_TRUE)
	 {
		 for(i = (number-1); i != 0; i--)	 //max value in the last element
		 {
			 if(pList[i] <= level)
			 {
				 return pList[i];
			 }	  
		 }

 		 battery_xlog_printk(BAT_LOG_CRTI, "Can't find closest level, small value first \r\n");
		 return pList[0];
		 //return CHARGE_CURRENT_0_00_MA;
	 }
	 else
	 {
		 for(i = 0; i< number; i++)  // max value in the first element
		 {
			 if(pList[i] <= level)
			 {
				 return pList[i];
			 }	  
		 }

		 battery_xlog_printk(BAT_LOG_CRTI, "Can't find closest level, large value first \r\n"); 	 
		 return pList[number -1];
  		 //return CHARGE_CURRENT_0_00_MA;
	 }
 }


static void hw_bc11_dump_register(void)
{
}

 
 

 static kal_uint32 charging_hw_init(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	static bool charging_init_flag = KAL_FALSE;
	
	mt_set_gpio_mode(gpio_number,gpio_on_mode);  
	mt_set_gpio_dir(gpio_number,gpio_on_dir);
	mt_set_gpio_out(gpio_number,gpio_on_out);
#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
	mt_set_gpio_mode(wireless_charger_gpio_number,0); // 0:GPIO mode
	mt_set_gpio_dir(wireless_charger_gpio_number,0); // 0: input, 1: output
#endif
	battery_xlog_printk(BAT_LOG_FULL, "gpio_number=0x%x,gpio_on_mode=%d,gpio_off_mode=%d\n", gpio_number, gpio_on_mode, gpio_off_mode);
	
    pmic_set_register_value(PMIC_RG_USBDL_RST,1);//force leave USBDL mode
#if defined(CONFIG_HIGH_BATTERY_VOLTAGE_SUPPORT)
	bq24158_config_interface_liao(0x06,0x77); // ISAFE = 1250mA, VSAFE = 4.34V
#else
	bq24158_config_interface_liao(0x06,0x70);
#endif
	    
	bq24158_config_interface_liao(0x00,0xC0);	//kick chip watch dog
	bq24158_config_interface_liao(0x01,0xd8);//0xb8);	//TE=1, CE=0, HZ_MODE=0, OPA_MODE=0
	bq24158_config_interface_liao(0x05,0x03);
    
	bq24158_config_interface_liao(0x04,0x1A); //146mA
	if ( !charging_init_flag ) {   
		bq24158_config_interface_liao(0x04,0x1A); //146mA
		charging_init_flag = KAL_TRUE;
	}        
 	return status;
 }


 static kal_uint32 charging_dump_register(void *data)
 {
 	kal_uint32 status = STATUS_OK;

	bq24158_dump_register();
   	
	return status;
 }	


 static kal_uint32 charging_enable(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 enable = *(kal_uint32*)(data);

	if(KAL_TRUE == enable) {
		bq24158_set_ce(0);
		bq24158_set_hz_mode(0);
		bq24158_set_opa_mode(0);
	} else {

#if defined(CONFIG_USB_MTK_HDRC_HCD)
   		if(mt_usb_is_device())
#endif 			
		{
			mt_set_gpio_mode(gpio_number,gpio_off_mode);  
			mt_set_gpio_dir(gpio_number,gpio_off_dir);
			mt_set_gpio_out(gpio_number,gpio_off_out);

			// bq24158_set_ce(1);
			if (charging_get_error_state()) {
				bq24158_set_ce(0x1);
				battery_xlog_printk(BAT_LOG_CRTI,"[charging_enable] bq24158_set_hz_mode(0x1)\n");
				bq24158_set_hz_mode(0x1);	// disable power path
			}
		}
	}
	return status;
 }


 static kal_uint32 charging_set_cv_voltage(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint16 register_value;
	kal_uint32 cv_value = *(kal_uint32 *)(data);

	register_value = charging_parameter_to_value(VBAT_CV_VTH, GETARRAYNUM(VBAT_CV_VTH) ,*(kal_uint32 *)(data));
	bq24158_set_oreg(register_value); 

	return status;
 } 	


 static kal_uint32 charging_get_current(void *data)
 {
    kal_uint32 status = STATUS_OK;
    kal_uint32 array_size;
    kal_uint8 reg_value;
	
    //Get current level
    array_size = GETARRAYNUM(CS_VTH);
    bq24158_read_interface(0x1, &reg_value, 0x3, 0x6);	//IINLIM
    *(kal_uint32 *)data = charging_value_to_parameter(CS_VTH,array_size,reg_value);
	
    return status;
 }  
  


 static kal_uint32 charging_set_current(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 set_chr_current;
	kal_uint32 array_size;
	kal_uint32 register_value;
	kal_uint32 current_value = *(kal_uint32 *)data;

	if(current_value <= CHARGE_CURRENT_350_00_MA)
	{
		bq24158_set_io_level(1);
	}
	else
	{
		bq24158_set_io_level(0);
		array_size = GETARRAYNUM(CS_VTH);
		set_chr_current = bmt_find_closest_level(CS_VTH, array_size, current_value);
		register_value = charging_parameter_to_value(CS_VTH, array_size ,set_chr_current);
		bq24158_set_iocharge(register_value);
	}
	return status;
 } 	
 

 static kal_uint32 charging_set_input_current(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 set_chr_current;
	kal_uint32 array_size;
	kal_uint32 register_value;

    if(*(kal_uint32 *)data > CHARGE_CURRENT_500_00_MA)
    {
        register_value = 0x3;
    }
    else
    {
    	array_size = GETARRAYNUM(INPUT_CS_VTH);
    	set_chr_current = bmt_find_closest_level(INPUT_CS_VTH, array_size, *(kal_uint32 *)data);
    	register_value = charging_parameter_to_value(INPUT_CS_VTH, array_size ,set_chr_current);	
    }
    
    bq24158_set_input_charging_current(register_value);

	return status;
 } 	


 static kal_uint32 charging_get_charging_status(void *data)
 {
 	kal_uint32 status = STATUS_OK;
	kal_uint32 ret_val;

	ret_val = bq24158_get_chip_status();
	
	if(ret_val == 0x2)
		*(kal_uint32 *)data = KAL_TRUE;
	else
		*(kal_uint32 *)data = KAL_FALSE;
	
	return status;
 } 	


 static kal_uint32 charging_reset_watch_dog_timer(void *data)
 {
	 kal_uint32 status = STATUS_OK;
 
	 bq24158_set_tmr_rst(1);
	 
	 return status;
 }
 
 
  static kal_uint32 charging_set_hv_threshold(void *data)
  {
	 kal_uint32 status = STATUS_OK;
 
	 kal_uint32 set_hv_voltage;
	 kal_uint32 array_size;
	 kal_uint16 register_value;
	 kal_uint32 voltage = *(kal_uint32*)(data);
	 
	 array_size = GETARRAYNUM(VCDT_HV_VTH);
	 set_hv_voltage = bmt_find_closest_level(VCDT_HV_VTH, array_size, voltage);
	 register_value = charging_parameter_to_value(VCDT_HV_VTH, array_size ,set_hv_voltage);
     pmic_set_register_value(PMIC_RG_VCDT_HV_VTH,register_value);
 
	 return status;
  }
 
 
  static kal_uint32 charging_get_hv_status(void *data)
  {
	   kal_uint32 status = STATUS_OK;
 
	   *(kal_bool*)(data) = pmic_get_register_value(PMIC_RGS_VCDT_HV_DET);
	   
	   return status;
  }


 static kal_uint32 charging_get_battery_status(void *data)
 {
   kal_uint32 status = STATUS_OK;
   kal_uint32 val = 0;
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
   *(kal_bool*)(data) = 0; // battery exist
   battery_log(BAT_LOG_CRTI,"bat exist for evb\n");
#else
   val=pmic_get_register_value(PMIC_BATON_TDET_EN);
   battery_log(BAT_LOG_FULL,"[charging_get_battery_status] BATON_TDET_EN =     %d\n", val);
   if (val) {
     pmic_set_register_value(PMIC_BATON_TDET_EN,1);
     pmic_set_register_value(PMIC_RG_BATON_EN,1);
     *(kal_bool*)(data) = pmic_get_register_value(PMIC_RGS_BATON_UNDET);
   } else {
     *(kal_bool*)(data) =  KAL_FALSE;
   }   
#endif

   return status;

 }


 static kal_uint32 charging_get_charger_det_status(void *data)
 {
   kal_uint32 status = STATUS_OK;

#if defined(CONFIG_MTK_FPGA)
   *(kal_bool*)(data) = 1;
   battery_log(BAT_LOG_CRTI,"chr exist for fpga\n");
#else
   *(kal_bool*)(data) = pmic_get_register_value(PMIC_RGS_CHRDET);
#endif

   return status;

 }


kal_bool charging_type_detection_done(void)
{
	 return charging_type_det_done;
}


 static kal_uint32 charging_get_charger_type(void *data)
 {
	 kal_uint32 status = STATUS_OK;
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	 *(CHARGER_TYPE*)(data) = STANDARD_HOST;
#else
     *(CHARGER_TYPE*)(data) = hw_charging_get_charger_type();
#endif
	 return status;
}

static kal_uint32 charging_get_is_pcm_timer_trigger(void *data)
{
    kal_uint32 status = STATUS_OK;

    if(slp_get_wake_reason() == WR_PCM_TIMER)
        *(kal_bool*)(data) = KAL_TRUE;
    else
        *(kal_bool*)(data) = KAL_FALSE;

    battery_xlog_printk(BAT_LOG_CRTI, "slp_get_wake_reason=%d\n", slp_get_wake_reason());
       
    return status;
}

static kal_uint32 charging_set_platform_reset(void *data)
{
    kal_uint32 status = STATUS_OK;

    battery_xlog_printk(BAT_LOG_CRTI, "charging_set_platform_reset\n");
 
    arch_reset(0,NULL);
        
    return status;
}

static kal_uint32 charging_get_platfrom_boot_mode(void *data)
{
    kal_uint32 status = STATUS_OK;
  
    *(kal_uint32*)(data) = get_boot_mode();

    battery_xlog_printk(BAT_LOG_CRTI, "get_boot_mode=%d\n", get_boot_mode());
         
    return status;
}

static kal_uint32 charging_set_power_off(void *data)
{
    kal_uint32 status = STATUS_OK;

    battery_xlog_printk(BAT_LOG_CRTI, "charging_set_power_off\n");
    mt_power_off();

    return status;
}

static kal_uint32 charging_get_power_source(void *data)
{
	kal_uint32 status = STATUS_UNSUPPORTED;

	return status;
}

static kal_uint32 charging_get_csdac_full_flag(void *data)
{
	return STATUS_UNSUPPORTED;	
}

static kal_uint32 charging_set_ta_current_pattern(void *data)
{
	return STATUS_UNSUPPORTED;	
}

static kal_uint32 charging_get_error_state(void)
{
	return charging_error;
}

static kal_uint32 charging_set_error_state(void *data)
{
	kal_uint32 status = STATUS_OK;
	charging_error = *(kal_uint32*)(data);
	
	return status;
}

 static kal_uint32 (* const charging_func[CHARGING_CMD_NUMBER])(void *data)=
 {
 	  charging_hw_init
	,charging_dump_register  	
	,charging_enable
	,charging_set_cv_voltage
	,charging_get_current
	,charging_set_current
	,charging_set_input_current
	,charging_get_charging_status
	,charging_reset_watch_dog_timer
	,charging_set_hv_threshold
	,charging_get_hv_status
	,charging_get_battery_status
	,charging_get_charger_det_status
	,charging_get_charger_type
	,charging_get_is_pcm_timer_trigger
	,charging_set_platform_reset
	,charging_get_platfrom_boot_mode
	,charging_set_power_off
	,charging_get_power_source
	,charging_get_csdac_full_flag
	,charging_set_ta_current_pattern
	,charging_set_error_state
 };

 
 /*
 * FUNCTION
 *		Internal_chr_control_handler
 *
 * DESCRIPTION															 
 *		 This function is called to set the charger hw
 *
 * CALLS  
 *
 * PARAMETERS
 *		None
 *	 
 * RETURNS
 *		
 *
 * GLOBALS AFFECTED
 *	   None
 */
 kal_int32 chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
 {
	 kal_int32 status;
	 if(cmd < CHARGING_CMD_NUMBER)
		 status = charging_func[cmd](data);
	 else
		 return STATUS_UNSUPPORTED;
 
	 return status;
 }


