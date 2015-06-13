
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/lnw_gpio.h>
#include <linux/gpio.h>
#include <linux/acpi.h>
#include <acpi/acpi_bus.h>
#include <linux/extcon/extcon-smsc375x.h>
#include <asm/intel-mid.h>
#include <asm/intel_crystalcove_pwrsrc.h>
#include <asm/intel_bytcr_bcntl.h>
#include <asm/dc_xpwr_pwrsrc.h>
#include <linux/dc_ti_pwrsrc.h>
#include <asm/intel_em_config.h>

#define EM_CONFIG_USB_COMP_MASK (1 << 0)
static struct smsc375x_pdata smsc_pdata;
bool valid_entry;

/* dummy functions */
static int smsc375x_enable_vbus(void *ptr)
{
	return 0;
}
static int smsc375x_disable_vbus(void *ptr)
{
	return 0;
}
static int smsc375x_is_vbus_online(void *ptr)
{
	return 0;
}

static acpi_status pmic_acpi_parse(acpi_handle handle,
				u32 lvl, void *context, void **rv)
{
	valid_entry = true;
	return AE_OK;
}

void *smsc375x_platform_data(void)
{
	acpi_status status;
	int ret = 0;
	struct em_config_oem1_data oem1_data;


	smsc_pdata.enable_vbus = intel_bytcr_boost_enable;
	smsc_pdata.disable_vbus = intel_bytcr_boost_disable;
	if (ACPI_SUCCESS(acpi_get_devices("INT33F4",
			pmic_acpi_parse, NULL, NULL)) && valid_entry) {
		pr_info("X-Power PMIC ACPI entry[INT33F4] found\n");
		smsc_pdata.is_vbus_online = dc_xpwr_vbus_on_status;
	} else if (ACPI_SUCCESS(acpi_get_devices("INT33F5",
			pmic_acpi_parse, NULL, NULL)) && valid_entry) {
		pr_info("TI PMIC ACPI entry[INT33F5] found\n");
		smsc_pdata.is_vbus_online = dc_ti_vbus_on_status;
	} else {
		pr_info("default Crytsal Cove PMIC path\n");
		smsc_pdata.is_vbus_online = crystal_cove_vbus_on_status;
	}

	memset(&oem1_data, 0, sizeof(struct em_config_oem1_data));
	ret = em_config_get_oem1_data(&oem1_data);
	/* If usb override  is set, then platform can voilate USB spec */
	if ((ret > 0) && !(oem1_data.fpo_0 & EM_CONFIG_USB_COMP_MASK))
		smsc_pdata.charging_compliance_override = false;
	else
		smsc_pdata.charging_compliance_override = true;

	return &smsc_pdata;
}
