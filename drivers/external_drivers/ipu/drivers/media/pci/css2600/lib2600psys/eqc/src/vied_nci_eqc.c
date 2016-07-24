#include <hrt/api.h>
#include <hrt/system.h>

#include "vied_nci_eqc.h"

static const unsigned int vied_nci_config_map[EQC_NO_DEVICES] = {
	EQC_DMA0_CFG_ADDR,
	EQC_IPFD_CFG_ADDR,
	EQC_PSA_CFG_ADDR,
	EQC_ISA_CFG_ADDR,
	EQC_GDC_CFG_ADDR
};

/* Forward declarations of internally used functions */
static void vied_nci_eqc_set_reg(unsigned int base_addr, unsigned int reg, unsigned int value);

static void vied_nci_eqc_set_reg(unsigned int base_addr, unsigned int reg, unsigned int value) {
	hrt_master_port_store(base_addr + reg * EQC_REG_ALIGNEMENT, &value, IPF_CFG_BUS_DATA_WIDTH/8);
}

int vied_nci_eqc_config(vied_nci_eqc_device_id_t device_id, const struct vied_nci_eqc_map p_id_eqc[])
{
	unsigned int id_eqc;
	unsigned int map_address;

	for(id_eqc = 0; id_eqc < NO_MAP_ENTRIES_EQC; id_eqc++) {
		map_address = p_id_eqc[id_eqc].base_addr + vied_nci_config_map[device_id];
		vied_nci_eqc_set_reg(map_address, 0, p_id_eqc[id_eqc].alow_addr);	/* Alow address */
		vied_nci_eqc_set_reg(map_address, 1, p_id_eqc[id_eqc].ahigh_addr);	/* Ahigh address */
		vied_nci_eqc_set_reg(map_address, 2, p_id_eqc[id_eqc].ack_addr);	/* Queue reserve address */
		vied_nci_eqc_set_reg(map_address, 3, 0);				/* reserved */
		}

	return 0;
}

vied_nci_eqc_handle_t vied_nci_eqc_open(vied_nci_eqc_device_id_t device_id)
{
	/* For now the device_id is not used */

	NOT_USED(device_id);

	return device_id;
}


int vied_nci_eqc_close(vied_nci_eqc_device_id_t device_id)
{

	/* For now the device_id is not used */

	NOT_USED(device_id);

	return 0;
}
