#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>
#include <mach/upmu_common.h>

static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 2,
	.polling_mode_ps = 0,
	.power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
	.power_vol  = VOL_DEFAULT,          /*LDO is not used*/
	.i2c_addr   = {0x0C, 0x48, 0x78, 0x00},
	.als_level	= {20, 45, 70, 90, 150, 300, 500, 700, 1150, 2250, 4500, 8000, 15000, 30000, 50000},
	.als_value	= {50, 100, 300, 500, 800, 1000, 1300, 1500, 1800, 2000, 3000, 6000, 10000, 20000, 40000, 60000},
	.ps_threshold_low = 200,
	.ps_threshold_high = 250,
};
struct alsps_hw *epl2182_get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}

