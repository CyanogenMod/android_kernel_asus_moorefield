#include "vied_nci_eq_device.h"

#include "vied_config_host.h"
#include <vied/vied_subsystem_access.h>
#include "assert_support.h"
#include "misc_support.h"
#include "vied_nci_eq_host_reg_access.h"

/* Used for Host to access Event Queue register */

void event_queue_ip_reg_store(
    const vied_nci_eq_device_t  dev,
    const unsigned int	       reg,
    const unsigned int		       value)
{
	vied_subsystem_store_32(0, dev + reg, value);
}

unsigned int event_queue_ip_reg_load(
    const vied_nci_eq_device_t  dev,
    const unsigned int              reg)
{
	return vied_subsystem_load_32(0, dev + reg);
}

void event_queue_op_reg_store(
    const vied_nci_eq_device_t  dev,
    const unsigned int	       reg,
    const unsigned int		       value)
{
    NOT_USED(dev);
    NOT_USED(reg);
    NOT_USED(value);
    assert(0); /* this port is not accessible from the host */
}

unsigned int event_queue_op_reg_load(const unsigned int reg)
{
    NOT_USED(reg);
    assert(0); /* this port is not accessible from the host */
    return 0;
}
