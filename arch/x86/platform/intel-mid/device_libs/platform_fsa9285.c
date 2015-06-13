
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/lnw_gpio.h>
#include <linux/gpio.h>
#include <linux/extcon/extcon-fsa9285.h>
#include <linux/power/smb347-charger.h>
#include <asm/intel-mid.h>
#include <asm/intel_crystalcove_pwrsrc.h>
#define OTG_MUX_GPIO	3
#define OTG_XSD_GPIO	58

/*
 * function get_platform_data and
 * related fucntions are added as
 * WA to support multiple platforms
 * based on SPID.
 */
static struct fsa9285_pdata fsa_pdata;
static inline int fsa_dummy_vbus_enable(void)
{
	return 0;
}
static inline int fsa_dummy_vbus_disable(void)
{
	return 0;
}
static inline int fsa_dummy_sdp_pre_setup(void)
{
	return 0;
}
static inline int fsa_dummy_sdp_post_setup(void)
{
	return 0;
}

static struct i2c_board_info __initdata fsa9285_i2c_device = {
	I2C_BOARD_INFO("fsa9285", 0x25),
};

void *fsa9285_platform_data(void)
{
	int ret = 0;

	fsa_pdata.enable_vbus = crystal_cove_enable_vbus;
	fsa_pdata.disable_vbus = crystal_cove_disable_vbus;

	if (INTEL_MID_BOARD(3, TABLET, BYT, BLK, PRO, 8PR0) ||
		INTEL_MID_BOARD(3, TABLET, BYT, BLK, ENG, 8PR0) ||
		INTEL_MID_BOARD(3, TABLET, BYT, BLK, PRO, 8PR1) ||
		INTEL_MID_BOARD(3, TABLET, BYT, BLK, ENG, 8PR1)) {

		/* Get SMB347 platform data for BYT-FFRD8 PR0/PR1 targets */
		fsa_pdata.sdp_pre_setup = smb347_disable_charger;
		fsa_pdata.sdp_post_setup = smb347_enable_charger;
	} else {
		/* Else consider dummy data */
		fsa_pdata.sdp_pre_setup = fsa_dummy_sdp_pre_setup;
		fsa_pdata.sdp_post_setup = fsa_dummy_sdp_post_setup;
	}

	ret = gpio_request(OTG_MUX_GPIO, "fsa-otg-mux");
	if (ret) {
		pr_err("unable to request GPIO pin\n");
		/* WA for FFRD8 as USB3 mux not available */
		fsa_pdata.mux_gpio = -1;
	} else {
		fsa_pdata.mux_gpio = OTG_MUX_GPIO;
	}

	if (INTEL_MID_BOARD(3, TABLET, BYT, BLK, PRO, 10PR11) ||
		INTEL_MID_BOARD(3, TABLET, BYT, BLK, ENG, 10PR11)) {
		ret = gpio_request(OTG_XSD_GPIO, "fsa-otg-xsd");
		if (ret) {
			pr_err("unable to request GPIO pin\n");
			fsa_pdata.xsd_gpio = -1;
		} else {
			/* configure pin number for FFRD10 */
			fsa_pdata.xsd_gpio = OTG_XSD_GPIO;
			/* workaround for pin setting */
			lnw_gpio_set_alt(OTG_XSD_GPIO, 0);
		}
	} else {
		/* FFRD8 doesn't need this pin */
		fsa_pdata.xsd_gpio = -1;
	}

	return &fsa_pdata;
}
