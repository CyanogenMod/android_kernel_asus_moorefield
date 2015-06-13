#ifndef __EHCI_SPH_PCI_H
#define __EHCI_SPH_PCI_H

/* SPH registers definition, used for bypass TLL mode
 * and power control
 */
#define CLV_SPH_HOSTPC		0xb4	/* SPH HOSTPC */
#define CLV_SPH_HOSTPC_PTS	(1<<30)	/* SPH HOSTPC PTS */
#define CLV_SPHCFG		0x400	/* SPHCFG (16bit) */
#define CLV_SPHCFG_ULPI1TYPE	(1<<0)	/* SPHCFG ULPI1TYPE */
#define CLV_SPHCFG_REFCKDIS	(1<<1)	/* ULPI 1 REF-CLOCK */

struct ehci_sph_pdata {
	unsigned		has_gpio:1;	/* has sph gpio or not */
	int			gpio_cs_n;	/* CS_N gpio  */
	int			gpio_reset_n;	/* RESET_N gpio */
	unsigned		enabled:1;	/* enable flag */
};

#endif
