/*
 * platform_ftxxxx.c: ftxxxx platform data initilization file
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <asm/intel_scu_flis.h>

#include "platform_ftxxxx.h"

static struct ftxxxx_platform_data ftxxxx_pdata = {
	.gpio_irq = FTXXXX_INT_PIN,
	.gpio_reset = FTXXXX_RESET_PIN,
	.screen_max_x = TOUCH_MAX_X,
	.screen_max_y = TOUCH_MAX_Y,
};

static struct i2c_board_info bus7_i2c_devices[] = {
	{
		I2C_BOARD_INFO(FTXXXX_NAME, (0x70 >> 1)),
		.platform_data = &ftxxxx_pdata,
	},
};
//static struct spi_board_info spi_devices[] = {};

void get_ftxxxx_platformdata(void *info)
{
	return ftxxxx_pdata;
}

static int __init ftxxxx_init(void)
{
	int ret;

	ret = i2c_register_board_info(7, &bus7_i2c_devices, 1);
	printk("[%s]i2c_register_board_info : %d \n",__func__ ,ret);

	return ret;
}

module_init(ftxxxx_init);


