/*
 * Intel Baytrail PWM driver.
 *
 * Copyright (C) 2013 Intel corporation.
 *
 * ----------------------------------------------------------------------------
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ----------------------------------------------------------------------------
 *
 */
#define PWM_BYT_CLK_KHZ	25000
int pwm_byt_init(struct device *pdev, void __iomem *base,
		int pwm_num, unsigned int clk_khz);
void pwm_byt_remove(struct device *dev);
extern const struct dev_pm_ops pwm_byt_pm;
