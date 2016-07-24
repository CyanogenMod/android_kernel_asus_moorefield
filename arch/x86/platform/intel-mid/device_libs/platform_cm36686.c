
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include "platform_cm36686.h"
#include <linux/cm36686.h>
#include <linux/HWVersion.h>
extern int Read_PROJ_ID(void);

void *cm36686_platform_data(void *info)
{
    static struct CM36686_platform_data cm36686_data = {
        .adc_table = { 0x0A, 0xA0, 0xE1, 0x140, 0x280,0x500, 0xA28, 0x16A8, 0x1F40, 0x2800},
        .power = NULL,
        .slave_addr = CM36686_slave_addr,
        .ps_close_thd_set = 0x90,
        .ps_away_thd_set = 0x50,
		.intr_pin = 44,
		.als_it = CM36686_ALS_IT_160ms,
        .ls_cmd = CM36686_ALS_IT_160ms | CM36686_ALS_GAIN_2,
        .ps_conf1_val = CM36686_PS_ITB_1 | CM36686_PS_DR_1_320 | CM36686_PS_IT_1_6T | CM36686_PS_PERS_2 | CM36686_PS_RES_1 |CM36686_PS_INT_IN_AND_OUT,
        .ps_conf3_val = CM36686_PS_MS_NORMAL | CM36686_PS_PROL_255 | CM36686_PS_SMART_PERS_ENABLE,
    };
    return &cm36686_data;
}

