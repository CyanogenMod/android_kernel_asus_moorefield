#ifndef _EQ_SPACE_CHECKER_H_
#define _EQ_SPACE_CHECKER_H_

#include "psys.h"
#include <master_to_slave_hrt.h>
#include "misc_support.h"

/* From Host to DMA0 */
#define HOST_2_AB_DMA0_ADDR  _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_dmae0i_sl_in /* host to DMA0 */
#define AB_2_DMA0_EQC_ADDR   _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_dmae0i_mt_out_to_processing_system_dma_logic_eqc_dmae0_sp1

/* From AB to PSA */
#define AB_2_PSA_EQC_ADDR    _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_gp_mt_out_to_processing_system_unps_logic_eqc_psa_sp1

#define HOST_2_DMA0_EQC_ADDR ((HOST_2_AB_DMA0_ADDR) + (AB_2_DMA0_EQC_ADDR))
#define HOST_2_PSA_EQC_ADDR  ((HOST_2_AB_PSA_ADDR) + (AB_2_PSA_EQC_ADDR))

#define EQC_DMA0_CFG_ADDR		HOST_2_DMA0_EQC_ADDR
#define EQC_IPFD_CFG_ADDR		HOST_2_IPFD_EQC_ADDR
#define EQC_PSA_CFG_ADDR		HOST_2_PSA_EQC_ADDR
#define EQC_ISA_CFG_ADDR		0                      /* Not known yet */
#define EQC_GDC_CFG_ADDR		0                      /* Not known yet */

#define NO_MAP_ENTRIES_EQC		8
#define EQC_REG_ALIGNEMENT		4

/* Configure bus sizes */
#define IPF_CFG_BUS_DATA_WIDTH		32
#define IPF_CFG_BUS_ADDR_WIDTH		32
#define IPF_VCT_BUS_DATA_WIDTH		512

#define EQC_NO_DEVICES			5

struct vied_nci_eqc_map {
	unsigned int base_addr;
	unsigned int alow_addr;
	unsigned int ahigh_addr;
	unsigned int ack_addr;
};

/* Device ids accessing the EQC module */
typedef enum {
	VIED_NCI_EQC_DMA_ID = 0,
	VIED_NCI_EQC_IPFD,
	VIED_NCI_EQC_PSA,
	VIED_NCI_EQC_ISA,
	VIED_NCI_EQC_GDC
} vied_nci_eqc_device_id_t;

typedef int vied_nci_eqc_handle_t;

/* Static map of all 8 cells */
static const struct vied_nci_eqc_map p_id_eqc[NO_MAP_ENTRIES_EQC] = {
	{0,    0x1000, 0x1010, 0x1700}, /* SPC */
	{0x10, 0x21000, 0x21010, 0x21700}, /* SPP0 */
	{0x20, 0x31000, 0x31010, 0x31700}, /* SPP1 */
	{0x30, 0x41000, 0x41010, 0x41700}, /* SPPF */
	{0x40, 0x001C1000, 0x001C1010, 0x001C1700}, /* ISP0 not accesible yet in A0, should be fixed in C0 */
	{0x50, 0x00241000, 0x00241010, 0x00241700}, /* ISP1 not accesible yet in A0, should be fixed in C0 */
	{0x60, 0x002C1000, 0x002C1010, 0x002C1700}, /* ISP2 not accesible yet in A0, should be fixed in C0 */
	{0x70, 0x00341000, 0x00341010, 0x00341700}  /* ISP3 not accesible yet in A0, should be fixed in C0 */
};

vied_nci_eqc_handle_t vied_nci_eqc_open(vied_nci_eqc_device_id_t device_id);

int vied_nci_eqc_config(vied_nci_eqc_device_id_t device_id, const struct vied_nci_eqc_map p_id_eqc[]);

int vied_nci_eqc_close(vied_nci_eqc_device_id_t device_id);

#endif /* _EQ_SPACE_CHECKER_H_ */
