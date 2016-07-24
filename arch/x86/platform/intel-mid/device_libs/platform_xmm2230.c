/*
 * platform_xmm2330.c: xmm2230 platform data initilization file
 *
 * (C) Copyright 2014 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/lnw_gpio.h>
#include <linux/spi/ifx_modem.h>
#include <asm/intel-mid.h>
#include "platform_xmm2230.h"

#define XMM2330_SPI_SPEED_HZ 12500000

static short spi_bus_num = -1;

void __init *xmm2230_platform_data(void *info)
{
	static struct ifx_modem_platform_data xmm2230_pdata;
        struct spi_board_info *spi_info = (struct spi_board_info *)info;

        if(spi_info != NULL)
                spi_bus_num = spi_info->bus_num;
	xmm2230_pdata.srdy = get_gpio_by_name("xmm2230_srdy");
	xmm2230_pdata.mrdy = get_gpio_by_name("xmm2230_mrdy");
	xmm2230_pdata.max_hz = XMM2330_SPI_SPEED_HZ;
	xmm2230_pdata.use_dma = true;

	return &xmm2230_pdata;
}

static int get_spi_gpios(int spi_bus_num, int *clk, int *ss, int *rxd, int *txd)
{
        switch(spi_bus_num) {
                case 6:
                       *clk = 116;
                       *ss = 117;
                       *rxd = 118;
                       *txd = 119;
                       return 0;
        }
        return 1;
}

static int config_output_low(int gpio)
{
        int ret;
        ret = gpio_request(gpio, "xmm2230");
        if(ret)
        {
                pr_err("xmm2230 gpio_request %d failed. status=%d\n", gpio, ret);
                return ret;
        }
        lnw_gpio_set_alt(gpio, LNW_GPIO);
        gpio_direction_output(gpio, 0);
        gpio_free(gpio);
        return 0;
}

static int config_alt(int gpio)
{
        int ret;
        ret = gpio_request(gpio, "xmm2230");
        if(ret)
        {
                pr_err("xmm2230 gpio_request %d failed. status=%d\n", gpio, ret);
                return ret;
        }
        gpio_direction_input(gpio);
        lnw_gpio_set_alt(gpio, 1);
        gpio_free(gpio);
        return 0;
}

void xmm2230_disable_spi(bool disable)
{
        int clk, ss, rxd, txd, ret;
        if(get_spi_gpios(spi_bus_num, &clk, &ss, &rxd, &txd)) {
                pr_err("xmm2230_disable_spi unknown bus %d\n", spi_bus_num);
                return;
        }
        if(disable) {
                if(config_output_low(clk))
                        return;
                if(config_output_low(ss))
                        return;
                if(config_output_low(rxd))
                        return;
                if(config_output_low(txd))
                        return;
        } else {
                if(config_alt(clk))
                        return;
                if(config_alt(ss))
                        return;
                if(config_alt(rxd))
                        return;
                if(config_alt(txd))
                        return;
        }
}
