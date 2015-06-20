#if (defined CONFIG_GPU_BURST) || (defined CONFIG_GPU_BURST_MODULE)
#include <linux/module.h>

#include "gburst_interface.h"

static struct gburst_interface_s gd_interface;


/**
 * gburst_interface_set_data() - Provide some gburst data for hooks
 * inside the graphics driver.
 * @gb_interface: Data to allow callback to gburst module.
 *
 * Also, the symbol dependency will establish a load order dependency for
 * the case where both the graphics driver  and the gburst driver are modules,
 * ensuring that the graphics driver is loaded and initialized before gburst.
 */
void gburst_interface_set_data(struct gburst_interface_s *gb_interface)
{
	gd_interface = *gb_interface;
}


/*  Leave this export in place, even if built-in, as it allows easy compilation
    testing of gburst as a module. */
EXPORT_SYMBOL(gburst_interface_set_data);


/**
 * gburst_interface_power_state_set() - gfx drv calls to indicate power state.
 * @st_on: 1 if power coming on, 0 if power going off.
 */
void gburst_interface_power_state_set(int st_on)
{
	if (gd_interface.gbs_power_state_set && gd_interface.gbs_priv)
		gd_interface.gbs_power_state_set(gd_interface.gbs_priv, st_on);
}

#endif /* if (defined CONFIG_GPU_BURST) || (defined CONFIG_GPU_BURST_MODULE) */
