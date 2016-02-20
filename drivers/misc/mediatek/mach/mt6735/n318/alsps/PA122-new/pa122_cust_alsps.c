#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>

static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 2,
    .polling_mode_ps =0, //0:Interrput,1:Polling
    .polling_mode_als =1,
    .power_id   = MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_DEFAULT,          /*LDO is not used*/
    //.i2c_addr   = {0x0C, 0x48, 0x78, 0x00},
    .als_level  = { 50, 100, 200, 500, 4000, 8000, 12000, 16000, 20000, 24000, 28000, 32000, 36000, 40000, 65535},
    .als_value  = {0, 15, 32, 80, 625, 1250, 1875, 2500, 3125, 3750, 4375, 5000, 5625, 6250, 10240, 10240},
    .ps_threshold_high = 500,
    .ps_threshold_low = 400,
};
struct alsps_hw *pa122_get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}

