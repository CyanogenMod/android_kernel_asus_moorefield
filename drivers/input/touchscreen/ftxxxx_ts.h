#ifndef __LINUX_ftxxxx_TS_H__
#define __LINUX_ftxxxx_TS_H__

#include <linux/version.h>

#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 8, 0))
#if defined(MODULE) || defined(CONFIG_HOTPLUG)
#define __devexit_p(x) x
#else
#define __devexit_p(x) NULL
#endif
/* Used for HOTPLUG */
#define __devinit        __section(.devinit.text) __cold notrace
#define __devinitdata    __section(.devinit.data)
#define __devinitconst   __section(.devinit.rodata)
#define __devexit        __section(.devexit.text) __exitused __cold notrace
#define __devexitdata    __section(.devexit.data)
#define __devexitconst   __section(.devexit.rodata)
#endif

/* -- dirver configure -- */
#define CFG_MAX_TOUCH_POINTS	10

#define PRESS_MAX	0xFF
#define FT_PRESS	0x08

#define FTXXXX_NAME	"ftxxxx_ts"

#define TOUCH_MAX_X	720
#define TOUCH_MAX_Y	1280
#ifdef CONFIG_ZS570ML
#define FTXXXX_INT_PIN	49
#define FTXXXX_INT_PIN_NAME	"ftxxxx-int"
#define FTXXXX_RESET_PIN	191
#define FTXXXX_RESET_PIN_NAME	"ftxxxx-reset"
#else
#define FTXXXX_INT_PIN	120
#define FTXXXX_INT_PIN_NAME	"ftxxxx-int"
#define FTXXXX_RESET_PIN	191
#define FTXXXX_RESET_PIN_NAME	"ftxxxx-reset"
#endif 

#define FT_MAX_ID	0x0F
#define FT_TOUCH_STEP	6
#define FT_TOUCH_X_H_POS		3
#define FT_TOUCH_X_L_POS		4
#define FT_TOUCH_Y_H_POS		5
#define FT_TOUCH_Y_L_POS		6
#define FT_TOUCH_XY_POS		    7
#define FT_TOUCH_MISC			8
#define FT_TOUCH_EVENT_POS		3
#define FT_TOUCH_ID_POS			5

#define POINT_READ_BUF	(3 + FT_TOUCH_STEP * CFG_MAX_TOUCH_POINTS)

/*register address*/
#define FTXXXX_REG_FW_VER		0xA6
#define FTXXXX_REG_POINT_RATE	0x88
#define FTXXXX_REG_THGROUP	0x80
#define FTXXXX_REG_VENDOR_ID	0xA8

#define FTXXXX_ENABLE_IRQ	1
#define FTXXXX_DISABLE_IRQ	0

int ftxxxx_i2c_Read(struct i2c_client *client, char *writebuf, int writelen,
		    char *readbuf, int readlen);
int ftxxxx_i2c_Write(struct i2c_client *client, char *writebuf, int writelen);

void ftxxxx_reset_tp(int HighOrLow);

void ftxxxx_Enable_IRQ(struct i2c_client *client, int enable);

/* The platform data for the Focaltech ftxxxx touchscreen driver */
struct ftxxxx_platform_data {
	uint32_t gpio_irq;		// IRQ port
	uint32_t irq_cfg;
	
	uint32_t gpio_wakeup;		// Wakeup support
	uint32_t wakeup_cfg;

	uint32_t gpio_reset;		// Reset support
	uint32_t reset_cfg;

	int screen_max_x;
	int screen_max_y;
	int pressure_max;
};

#endif
