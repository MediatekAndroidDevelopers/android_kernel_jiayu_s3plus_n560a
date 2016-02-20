#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>
#include <mach/upmu_common.h>

static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 2,
	.i2c_addr = {0x60,0,0,0},
	.polling_mode_ps =0,
	.polling_mode_als =1,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    //.i2c_addr   = {0x0C, 0x48, 0x78, 0x00},
    //.als_level  = { 0,  1,  1,   7,  15,  15,  100, 1000, 2000,  3000,  6000, 10000, 14000, 18000, 20000},
    //.als_value  = {40, 40, 90,  90, 160, 160,  225,  320,  640,  1280,  1280,  2600,  2600, 2600,  10240, 10240},
    /* MTK: modified to support AAL */
    .als_level  = {0, 213, 711, 1143, 2630, 6803, 6909, 10736, 19968, 24783, 29231, 42000, 51141, 51141, 51141},
    .als_value  = {0, 135, 303, 501, 1005, 2005, 3209, 5002, 8006, 10010, 12000, 16010, 20000, 20000, 20000, 20000},

    .ps_threshold_high = 32,
    .ps_threshold_low = 22,
    .is_batch_supported_ps = false,
    .is_batch_supported_als = false,
};
struct alsps_hw *get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}

