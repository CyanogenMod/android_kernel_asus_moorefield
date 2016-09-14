/*
 * Definitions and platform data for Analog Devices
 * Backlight drivers RT4532
 *
 */

#ifndef __LINUX_I2C_RT4532_H
#define __LINUX_I2C_RT4532_H

#define DEVICE_ID_REG		0x00
#define MANUFACTURE_REG		0x01
#define CONFIG_REG		0x02
#define TIMING_REG		0x03
#define BL_PWM_CTL_REG		0x04
#define FLAG_REG		0x05
#define PWM_EN		BIT(6)
#define PWM_EN_MASK		0x40


extern int rt4532_brightness_set(int level);

#endif /* __LINUX_I2C_RT4532_H */
