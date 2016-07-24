
#include <linux/i2c.h>
#include <linux/kernel.h>

static struct i2c_board_info __initdata ccpage0_i2c_device = {
	I2C_BOARD_INFO("crystalcove_page0", 0x5e),
};

static int __init ccpage0_platform_init(void)
{
	return i2c_register_board_info(7, &ccpage0_i2c_device, 1);
}
module_init(ccpage0_platform_init);

