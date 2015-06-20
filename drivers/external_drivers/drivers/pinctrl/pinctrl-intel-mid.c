/*
 * pinctrl-intel-mid.c Driver for the Intel MID pin controller
 *
 * Copyright (c) 2013,  Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>

#include <asm/intel_scu_flis.h>
#include <asm/intel_mid_pinctrl.h>

/* Pin names for the pinctrl subsystem */
static const struct pinctrl_pin_desc ctp_pins[] = {
	PINCTRL_PIN(0, "i2s_2_clk"),
	PINCTRL_PIN(1, "i2s_2_fs"),
	PINCTRL_PIN(2, "i2s_2_rxd"),
	PINCTRL_PIN(3, "i2s_2_txd"),
	PINCTRL_PIN(4, "msic_reset_b"),
	PINCTRL_PIN(5, "spi_0_clk"),
	PINCTRL_PIN(6, "spi_0_sdi"),
	PINCTRL_PIN(7, "spi_0_sdo"),
	PINCTRL_PIN(8, "spi_0_ss"),
	PINCTRL_PIN(9, "svid_clkout"),
	PINCTRL_PIN(10, "svid_clksynch"),
	PINCTRL_PIN(11, "svid_din"),
	PINCTRL_PIN(12, "svid_dout"),
	PINCTRL_PIN(13, "usb_ulpi_clk"),
	PINCTRL_PIN(14, "usb_ulpi_data0"),
	PINCTRL_PIN(15, "usb_ulpi_data1"),
	PINCTRL_PIN(16, "usb_ulpi_data2"),
	PINCTRL_PIN(17, "usb_ulpi_data3"),
	PINCTRL_PIN(18, "usb_ulpi_data4"),
	PINCTRL_PIN(19, "usb_ulpi_data5"),
	PINCTRL_PIN(20, "usb_ulpi_data6"),
	PINCTRL_PIN(21, "usb_ulpi_data7"),
	PINCTRL_PIN(22, "usb_ulpi_dir"),
	PINCTRL_PIN(23, "usb_ulpi_nxt"),
	PINCTRL_PIN(24, "usb_ulpi_refclk"),
	PINCTRL_PIN(25, "usb_ulpi_stp"),
	PINCTRL_PIN(26, "ulpi1lpc_gpe_b"),
	PINCTRL_PIN(27, "ulpi1lpc_lpc_ad0"),
	PINCTRL_PIN(28, "ulpi1lpc_lpc_ad1"),
	PINCTRL_PIN(29, "ulpi1lpc_lpc_ad2"),
	PINCTRL_PIN(30, "ulpi1lpc_lpc_ad3"),
	PINCTRL_PIN(31, "ulpi1lpc_lpc_clkout"),
	PINCTRL_PIN(32, "ulpi1lpc_lpc_clkrun"),
	PINCTRL_PIN(33, "ulpi1lpc_lpc_frame_b"),
	PINCTRL_PIN(34, "ulpi1lpc_lpc_reset_b"),
	PINCTRL_PIN(35, "ulpi1lpc_lpc_serirq"),
	PINCTRL_PIN(36, "ulpi1lpc_lpc_smi_b"),
	PINCTRL_PIN(37, "ulpi1lpc_usb_ulpi_1_clk"),
	PINCTRL_PIN(38, "ulpi1lpc_usb_ulpi_1_data0"),
	PINCTRL_PIN(39, "ulpi1lpc_usb_ulpi_1_data1"),
	PINCTRL_PIN(40, "ulpi1lpc_usb_ulpi_1_data2"),
	PINCTRL_PIN(41, "ulpi1lpc_usb_ulpi_1_data3"),
	PINCTRL_PIN(42, "ulpi1lpc_usb_ulpi_1_data4"),
	PINCTRL_PIN(43, "ulpi1lpc_usb_ulpi_1_data5"),
	PINCTRL_PIN(44, "ulpi1lpc_usb_ulpi_1_data6"),
	PINCTRL_PIN(45, "ulpi1lpc_usb_ulpi_1_data7"),
	PINCTRL_PIN(46, "ulpi1lpc_usb_ulpi_1_dir"),
	PINCTRL_PIN(47, "ulpi1lpc_usb_ulpi_1_nxt"),
	PINCTRL_PIN(48, "ulpi1lpc_usb_ulpi_1_refclk"),
	PINCTRL_PIN(49, "ulpi1lpc_usb_ulpi_1_stp"),
	PINCTRL_PIN(50, "kbd_dkin0"),
	PINCTRL_PIN(51, "kbd_dkin1"),
	PINCTRL_PIN(52, "kbd_dkin2"),
	PINCTRL_PIN(53, "kbd_dkin3"),
	PINCTRL_PIN(54, "kbd_mkin0"),
	PINCTRL_PIN(55, "kbd_mkin1"),
	PINCTRL_PIN(56, "kbd_mkin2"),
	PINCTRL_PIN(57, "kbd_mkin3"),
	PINCTRL_PIN(58, "kbd_mkin4"),
	PINCTRL_PIN(59, "kbd_mkin5"),
	PINCTRL_PIN(60, "kbd_mkin6"),
	PINCTRL_PIN(61, "kbd_mkin7"),
	PINCTRL_PIN(62, "kbd_mkout0"),
	PINCTRL_PIN(63, "kbd_mkout1"),
	PINCTRL_PIN(64, "kbd_mkout2"),
	PINCTRL_PIN(65, "kbd_mkout3"),
	PINCTRL_PIN(66, "kbd_mkout4"),
	PINCTRL_PIN(67, "kbd_mkout5"),
	PINCTRL_PIN(68, "kbd_mkout6"),
	PINCTRL_PIN(69, "kbd_mkout7"),
	PINCTRL_PIN(70, "camerasb10"),
	PINCTRL_PIN(71, "camerasb4"),
	PINCTRL_PIN(72, "camerasb5"),
	PINCTRL_PIN(73, "camerasb6"),
	PINCTRL_PIN(74, "camerasb7"),
	PINCTRL_PIN(75, "camerasb8"),
	PINCTRL_PIN(76, "camerasb9"),
	PINCTRL_PIN(77, "i2c_4_scl"),
	PINCTRL_PIN(78, "i2c_4_sda"),
	PINCTRL_PIN(79, "i2c_5_scl"),
	PINCTRL_PIN(80, "i2c_5_sda"),
	PINCTRL_PIN(81, "intd_dsi_te1"),
	PINCTRL_PIN(82, "intd_dsi_te2"),
	PINCTRL_PIN(83, "stio_0_cd_b"),
	PINCTRL_PIN(84, "stio_0_clk"),
	PINCTRL_PIN(85, "stio_0_cmd"),
	PINCTRL_PIN(86, "stio_0_dat0"),
	PINCTRL_PIN(87, "stio_0_dat1"),
	PINCTRL_PIN(88, "stio_0_dat2"),
	PINCTRL_PIN(89, "stio_0_dat3"),
	PINCTRL_PIN(90, "stio_0_dat4"),
	PINCTRL_PIN(91, "stio_0_dat5"),
	PINCTRL_PIN(92, "stio_0_dat6"),
	PINCTRL_PIN(93, "stio_0_dat7"),
	PINCTRL_PIN(94, "stio_0_wp_b"),
	PINCTRL_PIN(95, "camerasb0"),
	PINCTRL_PIN(96, "camerasb1"),
	PINCTRL_PIN(97, "camerasb2"),
	PINCTRL_PIN(98, "camerasb3"),
	PINCTRL_PIN(99, "ded_gpio10"),
	PINCTRL_PIN(100, "ded_gpio11"),
	PINCTRL_PIN(101, "ded_gpio12"),
	PINCTRL_PIN(102, "ded_gpio13"),
	PINCTRL_PIN(103, "ded_gpio14"),
	PINCTRL_PIN(104, "ded_gpio15"),
	PINCTRL_PIN(105, "ded_gpio16"),
	PINCTRL_PIN(106, "ded_gpio17"),
	PINCTRL_PIN(107, "ded_gpio18"),
	PINCTRL_PIN(108, "ded_gpio19"),
	PINCTRL_PIN(109, "ded_gpio20"),
	PINCTRL_PIN(110, "ded_gpio21"),
	PINCTRL_PIN(111, "ded_gpio22"),
	PINCTRL_PIN(112, "ded_gpio23"),
	PINCTRL_PIN(113, "ded_gpio24"),
	PINCTRL_PIN(114, "ded_gpio25"),
	PINCTRL_PIN(115, "ded_gpio26"),
	PINCTRL_PIN(116, "ded_gpio27"),
	PINCTRL_PIN(117, "ded_gpio28"),
	PINCTRL_PIN(118, "ded_gpio29"),
	PINCTRL_PIN(119, "ded_gpio30"),
	PINCTRL_PIN(120, "ded_gpio8"),
	PINCTRL_PIN(121, "ded_gpio9"),
	PINCTRL_PIN(122, "mpti_nidnt_clk"),
	PINCTRL_PIN(123, "mpti_nidnt_data0"),
	PINCTRL_PIN(124, "mpti_nidnt_data1"),
	PINCTRL_PIN(125, "mpti_nidnt_data2"),
	PINCTRL_PIN(126, "mpti_nidnt_data3"),
	PINCTRL_PIN(127, "stio_1_clk"),
	PINCTRL_PIN(128, "stio_1_cmd"),
	PINCTRL_PIN(129, "stio_1_dat0"),
	PINCTRL_PIN(130, "stio_1_dat1"),
	PINCTRL_PIN(131, "stio_1_dat2"),
	PINCTRL_PIN(132, "stio_1_dat3"),
	PINCTRL_PIN(133, "stio_2_clk"),
	PINCTRL_PIN(134, "stio_2_cmd"),
	PINCTRL_PIN(135, "stio_2_dat0"),
	PINCTRL_PIN(136, "stio_2_dat1"),
	PINCTRL_PIN(137, "stio_2_dat2"),
	PINCTRL_PIN(138, "stio_2_dat3"),
	PINCTRL_PIN(139, "coms_int0"),
	PINCTRL_PIN(140, "coms_int1"),
	PINCTRL_PIN(141, "coms_int2"),
	PINCTRL_PIN(142, "coms_int3"),
	PINCTRL_PIN(143, "ded_gpio4"),
	PINCTRL_PIN(144, "ded_gpio5"),
	PINCTRL_PIN(145, "ded_gpio6"),
	PINCTRL_PIN(146, "ded_gpio7"),
	PINCTRL_PIN(147, "i2s_0_clk"),
	PINCTRL_PIN(148, "i2s_0_fs"),
	PINCTRL_PIN(149, "i2s_0_rxd"),
	PINCTRL_PIN(150, "i2s_0_txd"),
	PINCTRL_PIN(151, "i2s_1_clk"),
	PINCTRL_PIN(152, "i2s_1_fs"),
	PINCTRL_PIN(153, "i2s_1_rxd"),
	PINCTRL_PIN(154, "i2s_1_txd"),
	PINCTRL_PIN(155, "mslim_1_bclk"),
	PINCTRL_PIN(156, "mslim_1_bdat"),
	PINCTRL_PIN(157, "resetout_b"),
	PINCTRL_PIN(158, "spi_2_clk"),
	PINCTRL_PIN(159, "spi_2_sdi"),
	PINCTRL_PIN(160, "spi_2_sdo"),
	PINCTRL_PIN(161, "spi_2_ss0"),
	PINCTRL_PIN(162, "spi_2_ss1"),
	PINCTRL_PIN(163, "spi_3_clk"),
	PINCTRL_PIN(164, "spi_3_sdi"),
	PINCTRL_PIN(165, "spi_3_sdo"),
	PINCTRL_PIN(166, "spi_3_ss0"),
	PINCTRL_PIN(167, "spi_3_ss1"),
	PINCTRL_PIN(168, "uart_0_cts"),
	PINCTRL_PIN(169, "uart_0_rts"),
	PINCTRL_PIN(170, "uart_0_rx"),
	PINCTRL_PIN(171, "uart_0_tx"),
	PINCTRL_PIN(172, "uart_1_rx"),
	PINCTRL_PIN(173, "uart_1_sd"),
	PINCTRL_PIN(174, "uart_1_tx"),
	PINCTRL_PIN(175, "uart_2_rx"),
	PINCTRL_PIN(176, "uart_2_tx"),
	PINCTRL_PIN(177, "aclkph"),
	PINCTRL_PIN(178, "dclkph"),
	PINCTRL_PIN(179, "dsiclkph"),
	PINCTRL_PIN(180, "ierr"),
	PINCTRL_PIN(181, "jtag_tckc"),
	PINCTRL_PIN(182, "jtag_tdic"),
	PINCTRL_PIN(183, "jtag_tdoc"),
	PINCTRL_PIN(184, "jtag_tmsc"),
	PINCTRL_PIN(185, "jtag_trst_b"),
	PINCTRL_PIN(186, "lclkph"),
	PINCTRL_PIN(187, "lfhclkph"),
	PINCTRL_PIN(188, "osc_clk_ctrl0"),
	PINCTRL_PIN(189, "osc_clk_ctrl1"),
	PINCTRL_PIN(190, "osc_clk_out0"),
	PINCTRL_PIN(191, "osc_clk_out1"),
	PINCTRL_PIN(192, "osc_clk_out2"),
	PINCTRL_PIN(193, "osc_clk_out3"),
	PINCTRL_PIN(194, "prochot_b"),
	PINCTRL_PIN(195, "thermtrip_b"),
	PINCTRL_PIN(196, "uclkph"),
	PINCTRL_PIN(197, "ded_gpio31"),
	PINCTRL_PIN(198, "ded_gpio32"),
	PINCTRL_PIN(199, "ded_gpio33"),
	PINCTRL_PIN(200, "hdmi_cec"),
	PINCTRL_PIN(201, "i2c_3_scl_hdmi_ddc"),
	PINCTRL_PIN(202, "i2c_3_sda_hdmi_ddc"),
	PINCTRL_PIN(203, "i2c_0_scl"),
	PINCTRL_PIN(204, "i2c_0_sda"),
	PINCTRL_PIN(205, "i2c_1_scl"),
	PINCTRL_PIN(206, "i2c_1_sda"),
	PINCTRL_PIN(207, "i2c_2_scl"),
	PINCTRL_PIN(208, "i2c_2_sda"),
	PINCTRL_PIN(209, "spi_1_clk"),
	PINCTRL_PIN(210, "spi_1_sdi"),
	PINCTRL_PIN(211, "spi_1_sdo"),
	PINCTRL_PIN(212, "spi_1_ss0"),
	PINCTRL_PIN(213, "spi_1_ss1"),
	PINCTRL_PIN(214, "spi_1_ss2"),
	PINCTRL_PIN(215, "spi_1_ss3"),
	PINCTRL_PIN(216, "spi_1_ss4"),
};

struct intel_mid_pinctrl_drvdata {
	struct device *dev;
	struct pinctrl_dev *pctl;
	struct pinstruct_t *pin_t;
	int pin_num;
};

#define DRIVER_NAME "intel-mid-pinctrl"

struct ctp_pin_group {
	const char *name;
	const unsigned int *pins;
	const unsigned num_pins;
};

static const struct ctp_pin_group ctp_pin_groups[] = {
	/* Need to be filled */
};

static int ctp_list_groups(struct pinctrl_dev *pctldev, unsigned selector)
{
	if (selector >= ARRAY_SIZE(ctp_pin_groups))
		return -EINVAL;
	return 0;
}

static const char *ctp_get_group_name(struct pinctrl_dev *pctldev,
				       unsigned selector)
{
	if (selector >= ARRAY_SIZE(ctp_pin_groups))
		return NULL;
	return ctp_pin_groups[selector].name;
}

static int ctp_get_group_pins(struct pinctrl_dev *pctldev, unsigned selector,
			       const unsigned **pins,
			       unsigned *num_pins)
{
	if (selector >= ARRAY_SIZE(ctp_pin_groups))
		return -EINVAL;
	*pins = ctp_pin_groups[selector].pins;
	*num_pins = ctp_pin_groups[selector].num_pins;
	return 0;
}

static void ctp_pin_dbg_show(struct pinctrl_dev *pctldev, struct seq_file *s,
		   unsigned offset)
{
	seq_printf(s, " " DRIVER_NAME);
}
static struct pinctrl_ops ctp_pctrl_ops = {
	.list_groups = ctp_list_groups,
	.get_group_name = ctp_get_group_name,
	.get_group_pins = ctp_get_group_pins,
	.pin_dbg_show = ctp_pin_dbg_show,
};

int ctp_pin_config_get(struct pinctrl_dev *pctldev,
			unsigned pin,
			unsigned long *config)
{
	struct intel_mid_pinctrl_drvdata *impd =
				pinctrl_dev_get_drvdata(pctldev);

	u32 flis_addr, off, data, mask;
	int ret, pos;

	/* bit[11:8] indicate the configuration type,
	 * bit[7:0] indicate the value
	 */
	u8 val = *config & 0xFF;
	u8 type = (*config & 0xF00) >> 8;
	enum pin_config_param param = (enum pin_config_param) type;

	if (pin < 0 || pin >= impd->pin_num)
		return -EINVAL;

	if (impd->pin_t[pin].valid == false)
		return -EINVAL;

	flis_addr = impd->pin_t[pin].bus_address;

	switch (param) {
	case PIN_CONFIG_BIAS_PULL_UP:
	case PIN_CONFIG_BIAS_PULL_DOWN:
		off = impd->pin_t[pin].pullup_offset;
		pos = impd->pin_t[pin].pullup_lsb_pos;
		mask = PULL_MASK;
		break;
	case PIN_CONFIG_MUX:
		off = impd->pin_t[pin].direction_offset;
		pos = impd->pin_t[pin].direction_lsb_pos;
		mask = MUX_MASK;
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		off = impd->pin_t[pin].open_drain_offset;
		pos = impd->pin_t[pin].open_drain_bit;
		mask = OPEN_DRAIN_MASK;
		break;
	default:
		dev_err(impd->dev, "illegal configuration requested\n");
		return -EINVAL;
	}

	ret = intel_scu_ipc_read_shim(&data, flis_addr, off);
	if (ret) {
		dev_err(impd->dev, "read shim failed, addr = 0x%x, off = 0x%x\n",
			flis_addr, off);
		return ret;
	}

	val = (data >> pos) & mask;
	*config &= (~0xFF);
	*config |= val;

	pr_debug("read: data = 0x%x, val = 0x%x\n", data, val);

	return 0;
}

int ctp_pin_config_set(struct pinctrl_dev *pctldev,
			unsigned pin,
			unsigned long config)
{
	struct intel_mid_pinctrl_drvdata *impd
			= pinctrl_dev_get_drvdata(pctldev);

	u32 flis_addr, off, data, mask;
	int ret, pos;

	/* bit[11:8] indicate the configuration type,
	 * bit[7:0] indicate the value
	 */
	u8 val = config & 0xFF;
	u8 type = (config & 0xF00) >> 8;
	enum pin_config_param param = (enum pin_config_param) type;

	if (pin < 0 || pin >= impd->pin_num)
		return -EINVAL;

	if (impd->pin_t[pin].valid == false)
		return -EINVAL;

	flis_addr = impd->pin_t[pin].bus_address;

	switch (param) {
	case PIN_CONFIG_BIAS_PULL_UP:
	case PIN_CONFIG_BIAS_PULL_DOWN:
		off = impd->pin_t[pin].pullup_offset;
		pos = impd->pin_t[pin].pullup_lsb_pos;
		mask = (PULL_MASK << pos);
		break;
	case PIN_CONFIG_MUX:
		off = impd->pin_t[pin].direction_offset;
		pos = impd->pin_t[pin].direction_lsb_pos;
		mask = (MUX_MASK << pos);
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		off = impd->pin_t[pin].open_drain_offset;
		pos = impd->pin_t[pin].open_drain_bit;
		mask = (OPEN_DRAIN_MASK << pos);
		break;
	default:
		dev_err(impd->dev, "illegal configuration requested\n");
		return -EINVAL;
	}

	data = (val << pos);
	pr_debug("addr = 0x%x, off = 0x%x, pos = %d, mask = 0x%x, data = 0x%x\n",
			flis_addr, off, pos, mask, data);

	ret = intel_scu_ipc_update_shim(data, mask, flis_addr, off);
	if (ret) {
		dev_err(impd->dev, "update shim failed\n");
		return ret;
	}

	return 0;
}

static struct pinconf_ops ctp_pconf_ops = {
	.is_generic = true,
	.pin_config_get = ctp_pin_config_get,
	.pin_config_set = ctp_pin_config_set,
};

static struct pinctrl_desc intel_mid_pin_descs[] = {
	[ctp_pin_desc]  = {
		.name = "ctp_pinctrl",
		.pins = ctp_pins,
		.npins = ARRAY_SIZE(ctp_pins),
		.pctlops = &ctp_pctrl_ops,
		.confops = &ctp_pconf_ops,
		.owner = THIS_MODULE,
	},
	[mrfl_pin_desc] = {
		.name = "mrfl_pinctrl",
	},
};

static int intel_mid_pinctrl_probe(struct platform_device *pdev)
{
	int ret;
	struct intel_mid_pinctrl_drvdata *impd;
	struct intel_mid_pinctrl_platform_data *pdata =
					pdev->dev.platform_data;
	int i;

	if (!pdata) {
		dev_err(&pdev->dev, "No pinctrl platform data\n");
		ret = -EINVAL;
		goto out;
	}

	/* Create state holders etc for this driver */
	impd = devm_kzalloc(&pdev->dev, sizeof(*impd), GFP_KERNEL);
	if (!impd)
		return -ENOMEM;

	impd->dev = &pdev->dev;

	impd->pin_t = pdata->pin_t;
	impd->pin_num = pdata->pin_num;

	for (i = 0; i < ARRAY_SIZE(intel_mid_pin_descs); i++) {
		if (!strcmp(pdata->name, intel_mid_pin_descs[i].name)) {
			impd->pctl = pinctrl_register(&intel_mid_pin_descs[i],
							&pdev->dev, impd);
			if (!impd->pctl) {
				dev_err(&pdev->dev,
				"could not register %s pinctrl driver\n",
					pdata->name);
				ret = -EINVAL;
				goto err1;
			}

			break;
		}
	}

	if (i == ARRAY_SIZE(intel_mid_pin_descs)) {
		dev_err(&pdev->dev, "could not find match of desc\n");
		ret = -ENODEV;
		goto err1;
	}

	platform_set_drvdata(pdev, impd);

	dev_info(&pdev->dev, "initialized Intel MID pinctrl driver\n");

	return 0;
err1:
	devm_kfree(&pdev->dev, impd);

out:
	return ret;
}

static int intel_mid_pinctrl_remove(struct platform_device *pdev)
{
	struct intel_mid_pinctrl_drvdata *impd = platform_get_drvdata(pdev);

	pinctrl_unregister(impd->pctl);
	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, impd);
	return 0;
}

static struct platform_driver intel_mid_pinctrl_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe = intel_mid_pinctrl_probe,
	.remove = intel_mid_pinctrl_remove,
};

static int __init intel_mid_pinctrl_init(void)
{
	return platform_driver_register(&intel_mid_pinctrl_driver);
}
arch_initcall(intel_mid_pinctrl_init);

static void __exit intel_mid_pinctrl_exit(void)
{
	platform_driver_unregister(&intel_mid_pinctrl_driver);
}
module_exit(intel_mid_pinctrl_exit);

MODULE_DESCRIPTION("Intel MID pin control driver");
MODULE_LICENSE("GPL v2");
