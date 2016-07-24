/*
 * intel_soc_pmc.c - This driver provides interface to configure the Power
 * Management Controller (PMC).
 * Copyright (c) 2013, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/suspend.h>
#include <linux/time.h>
#include <linux/intel_mid_pm.h>
#include <linux/pm_qos.h>
#include <asm/intel_mid_pcihelpers.h>

#define BYT_S3_HINT		0x64

#define	S0IX_REGISTERS_OFFSET	0x80

#define	S0IR_TMR_OFFSET		0x80
#define	S0I1_TMR_OFFSET		0x84
#define	S0I2_TMR_OFFSET		0x88
#define	S0I3_TMR_OFFSET		0x8c
#define	S0_TMR_OFFSET		0x90

#define	S0IX_WAKE_EN		0x3c
#define	D3_STS0			0xA0
#define	D3_STS1			0xA4
#define FUNC_DIS		0x34
#define FUNC_DIS2		0x38
#define PMC_PSS			0x98

#define	PMC_MMIO_BAR		1
#define	BASE_ADDRESS_MASK	0xFFFFFFFE00

/* Disable PMC SOIX_WAKE_EN events coming from:
 * - LPC clock run
 * - GPIO_SUS ored dedicated IRQs
 * - GPIO_SCORE ored dedicated IRQs
 * - GPIO_SUS shared IRQ
 * - GPIO_SCORE shared IRQ
 */
#define	PMC_WAKE_EN_SETTING     0xe3ffcf

#define PM_SUPPORT		0x21

#define ISP_POS			7
#define ISP_SUB_CLASS		0x80

#define PUNIT_PORT		0x04
#define PWRGT_CNT		0x60
#define PWRGT_STATUS		0x61
#define VED_SS_PM0		0x32
#define ISP_SS_PM0		0x39
#define MIO_SS_PM		0x3B
#define SSS_SHIFT		24
#define RENDER_POS		0
#define MEDIA_POS		2
#define DISPLAY_POS		6

#define DSP_SSS_CHT		0x36
#define DSP_SSS_POS_CHT		16

#define MAX_POWER_ISLANDS	16
#define ISLAND_UP		0x0
#define ISLAND_DOWN		0x1
/*Soft reset*/
#define ISLAND_SR		0x2

/* Soft reset mask */
#define SR_MASK			0x2

#define NC_PM_SSS		0x3F

#define GFX_LSS_INDEX		1

#define PMC_D0I0_MASK		0
#define PMC_D0I1_MASK		1
#define PMC_D0I2_MASK		2
#define PMC_D0I3_MASK		3

#define BITS_PER_LSS		2
#define PCI_ID_ANY		(~0)
#define SUB_CLASS_MASK		0xFF00

#define SC_NUM_DEVICES 36
#define NC_NUM_DEVICES 6

#define MTPMC_VAL 0x20
#define DYN_MODE 0x105

#define LPCC_ADDR 0xFED08084
#define PLT_CLK(x)	(0x60 + (0x04 * (x)))
#define PLT_CLK_MAXCOUNT	6
#define MTPMC 0xB0
#define VLV_PM_STS 0xC
const char *pg_device_cht[] = {
	"0 - Reserved", "1 - SATA", "2 - HDA", "3 - SEC",
	"4 - PCIE 5", "5 - LPSS ", "6 - LPE", "7 - UFS",
	"8 - Reserved", "9 - Reserved", "10 - Reserved",
	"11 - UXD(xDCI)", "12 - UXD FD SX(xDCI)",
	"13 - Reserved ", "14 - Reserved ",
	"15 - UX Engine(xHCI) ", "16 - USB SUS ",
	"17 - GMM", "18 - ISH ", "19 - Reserved",
	"20 - Reserved", "21 - Reserved",
	"22 - Reserved", "23 - Reserved",
	"24 - Reserved", "25 - Reserved",
	"26 - DFX Master ", "27 - DFX Cluster1",
	"28 - DFX Cluster2", "29 - DFX Cluster3",
	"30 - DFX Cluster4", "31 - DFX Cluster5"
};

const char *d3_device_cht[] = {
	"0 - LPSS 0 DMA 1", "1 - LPSS 0 PMW 0",
	"2 - LPSS 0 PMW 0", "3 - LPSS 0 UART1",
	"4 - LPSS 0 UART2", "5 - LPSS SPI",
	"6 - LPSS 0 function 6", "7 - LPSS 0 function 7",
	"8 - SCC EMMC", "9 - SCC SD",
	"10 - SCC SDIO", "11 - MIPI", "12 - HDA",
	"13 - LPE", "14 - OTG", "15 - UFS ",
	"16 - GBE", "17 - SATA", "18 - xHCI ", "19 - SEC",
	"20 - PCIE function 0", "21 - PCIE function 1",
	"22 - PCIE function 2", "23 - PCIE function 3",
	"24 - LPSS 1 DMA1", "25 - LPSS 1 I2c 1",
	"26 - LPSS 1 I2c 2", "27 - LPSS 1 I2c 3",
	"28 - LPSS 1 I2c 4", "29 - LPSS 1 I2c 5",
	"30 - LPSS 1 I2c 6", "31 - LPSS 1 I2c 7",
	"32 - SMB", "33 -GMM", "34 - ISH"
};

const char *d3_device_byt[] = {
	"0 - LPSS 0 function 0 (DMA)",
	"1 - LPSS 0 function 1 (PWM#1)",
	"2 - LPSS 0 function 2 (PWM#2)",
	"3 - LPSS 0 function 3 (HSUART#1)",
	"4 - LPSS 0 function 4 (HSUART#2)",
	"5 - LPSS 0 function 5 (SPI)",
	"6 - LPSS 0 function 6",
	"7 - LPSS 0 function 7",
	"8 - SCC function 0 (eMMC)",
	"9 - SCC function 1 (SDIO)",
	"10 - SCC function 2 (SDCARD)",
	"11 - MIPI",
	"12 - HDA",
	"13 - LPE",
	"14 - OTG",
	"15 - USH",
	"16 - GBE",
	"17 - SATA",
	"18 - USB",
	"19 - SEC",
	"20 - PCIE function 0",
	"21 - PCIE function 1",
	"22 - PCIE function 2",
	"23 - PCIE function 3",
	"24 - LPSS 1 function 0 (DMA)",
	"25 - LPSS 1 function 1 (I2C#)",
	"26 - LPSS 1 function 2 (I2C#)",
	"27 - LPSS 1 function 3 (I2C#)",
	"28 - LPSS 1 function 4 (I2C#)",
	"29 - LPSS 1 function 5 (I2C#)",
	"30 - LPSS 1 function 6 (I2C#)",
	"31 - LPSS 1 function 7 (I2C#)",
	"32 - SMB",
	"33 - USH Super speed PHY",
	"34 - OTG Super speed PHY",
	"35 - DFX"
};

enum system_state {
	STATE_S0IR,
	STATE_S0I1,
	STATE_S0I2,
	STATE_S0I3,
	STATE_S0,
	STATE_S3,
	STATE_MAX
};

struct pmc_dev {
	u32 base_address;
	u32 __iomem *pmc_registers;
	u32 __iomem *s0ix_wake_en;
	u32 __iomem *d3_sts0;
	u32 __iomem *d3_sts1;
	u32 __iomem *func_dis;
	u32 __iomem *func_dis2;
	u32 __iomem *pmc_pss;
	u32 __iomem *pc[PLT_CLK_MAXCOUNT];
	u32 __iomem *lpcc;
	u32 __iomem *mtpmc_1;
	u32 __iomem *vlv_pm_sts;
	struct pci_dev const *pdev;
	spinlock_t nc_ready_lock;
	struct mutex pc_mutex;
	u32 state_residency[STATE_MAX];
	u32 state_resi_offset[STATE_MAX];
	u32 residency_total;
	u32 s3_count;
	struct platform_device *tco_wdt_dev;
};

char *states[] = {
	"S0IR",
	"S0I1",
	"S0I2",
	"S0I3",
	"S0",
	"S3",
};

/* REVISIT, FIXME: paranoia's sake */
struct pmc_dev *pmc;

static char *dstates[] = {"D0", "D0i1", "D0i2", "D0i3"};
struct nc_device {
	char *name;
	int reg;
	int sss_pos;
};

const struct nc_device nc_devices_byt[] = {
	{ "GFX RENDER", PWRGT_STATUS,  RENDER_POS },
	{ "GFX MEDIA", PWRGT_STATUS, MEDIA_POS },
	{ "DISPLAY", PWRGT_STATUS,  DISPLAY_POS },
	{ "VED", VED_SS_PM0, SSS_SHIFT},
	{ "ISP", ISP_SS_PM0, SSS_SHIFT},
	{ "MIO", MIO_SS_PM, SSS_SHIFT},
};

const struct nc_device nc_devices_cht[] = {
	{ "GFX RENDER", PWRGT_STATUS,  RENDER_POS },
	{ "GFX MEDIA", PWRGT_STATUS, MEDIA_POS },
	{ "DSP", DSP_SSS_CHT,  DSP_SSS_POS_CHT },
	{ "VED", VED_SS_PM0, SSS_SHIFT},
	{ "ISP", ISP_SS_PM0, SSS_SHIFT},
	{ "MIO", MIO_SS_PM, SSS_SHIFT},
};

static int pmc_wait_for_nc_pmcmd_complete(int verify_mask,
				int status_mask, int state_type , int reg)
{
	int pwr_sts;
	int count = 0;

	while (true) {
		if (reg == PWRGT_CNT)
			pwr_sts = intel_mid_msgbus_read32(PUNIT_PORT,
							PWRGT_STATUS);
		else {
			pwr_sts = intel_mid_msgbus_read32(PUNIT_PORT, reg);
			pwr_sts = pwr_sts >> SSS_SHIFT;
		}
		if (state_type == ISLAND_DOWN ||
				state_type == ISLAND_SR) {
			if ((pwr_sts & status_mask) ==
					(verify_mask & status_mask))
				break;
			else
				udelay(10);
		} else if (state_type == ISLAND_UP) {
			if ((~pwr_sts & status_mask)  ==
					(~verify_mask & status_mask))
				break;
			else
				udelay(10);
		}

		count++;
		if (WARN_ONCE(count > 500000, "Timed out waiting for P-Unit"))
			return -EBUSY;
	}
	return 0;
}

/*
 * FIXME: The functions below are legacy and subject to change. They are kept
 * for backward compatibility.
 */
int pmc_nc_get_power_state(int islands, int reg)
{
	int pwr_sts, i, lss, ret = 0;
	unsigned long flags;

	if (unlikely(!pmc))
		return -EAGAIN;

	spin_lock_irqsave(&pmc->nc_ready_lock, flags);

	pwr_sts = intel_mid_msgbus_read32(PUNIT_PORT, reg);
	if (reg != PWRGT_STATUS)
		pwr_sts = pwr_sts >> SSS_SHIFT;

	for (i = 0; i < MAX_POWER_ISLANDS; i++) {
		lss = islands & (0x1 << i);
		if (lss) {
			ret = (pwr_sts >> (BITS_PER_LSS * i)) & PMC_D0I3_MASK;
			break;
		}
	}

	spin_unlock_irqrestore(&pmc->nc_ready_lock, flags);

	return ret;
}
EXPORT_SYMBOL(pmc_nc_get_power_state);

int pmc_nc_set_power_state(int islands, int state_type, int reg)
{
	u32 pwr_sts = 0;
	u32 pwr_mask = 0;
	int i, lss, mask;
	int ret = 0;
	int status_mask = 0;
	unsigned long flags;

	if (unlikely(!pmc))
		return -EAGAIN;

	spin_lock_irqsave(&pmc->nc_ready_lock, flags);

	pwr_sts = intel_mid_msgbus_read32(PUNIT_PORT, reg);
	pwr_mask = pwr_sts;

	for (i = 0; i < MAX_POWER_ISLANDS; i++) {
		lss = islands & (0x1 << i);
		if (lss) {
			mask = PMC_D0I3_MASK << (BITS_PER_LSS * i);
			status_mask = status_mask | mask;
			if (state_type == ISLAND_DOWN)
				pwr_mask |= mask;
			else if (state_type == ISLAND_UP)
				pwr_mask &= ~mask;
			/* Soft reset case */
			else if (state_type == ISLAND_SR) {
				pwr_mask &= ~mask;
				mask = SR_MASK << (BITS_PER_LSS * i);
				pwr_mask |= mask;
			}
		}
	}

	intel_mid_msgbus_write32(PUNIT_PORT, reg, pwr_mask);
	ret = pmc_wait_for_nc_pmcmd_complete(pwr_mask,
				status_mask, state_type, reg);

	spin_unlock_irqrestore(&pmc->nc_ready_lock, flags);

	return ret;
}
EXPORT_SYMBOL(pmc_nc_set_power_state);

#define CLK_CONFG_BIT_POS		0
#define CLK_CONFG_BIT_LEN		2
#define CLK_CONFG_D3_GATED		0
#define CLK_CONFG_FORCE_ON		1
#define CLK_CONFG_FORCE_OFF		2

#define CLK_FREQ_TYPE_BIT_POS		2
#define CLK_FREQ_TYPE_BIT_LEN		1
#define CLK_FREQ_TYPE_XTAL		0	/* 25 MHz */
#define CLK_FREQ_TYPE_PLL		1	/* 19.2 MHz */

#define REG_MASK(n)		(((1 << (n##_BIT_LEN)) - 1) << (n##_BIT_POS))
#define REG_SET_FIELD(r, n, v)	(((r) & ~REG_MASK(n)) | \
				 (((v) << (n##_BIT_POS)) & REG_MASK(n)))
#define REG_GET_FIELD(r, n)	(((r) & REG_MASK(n)) >> n##_BIT_POS)

int pmc_pc_set_freq(int clk_num, int freq_type)
{
	if (unlikely(!pmc))
		return -EAGAIN;

	if (clk_num < 0 && clk_num > PLT_CLK_MAXCOUNT) {
		dev_err(&pmc->pdev->dev,
			"Clock number out of range (%d)\n", clk_num);
		return -EINVAL;
	}

	if (freq_type != CLK_FREQ_TYPE_XTAL &&
	    freq_type != CLK_FREQ_TYPE_PLL) {
		dev_err(&pmc->pdev->dev, "wrong clock type\n");
		return -EINVAL;
	}

	mutex_lock(&pmc->pc_mutex);
	writel(REG_SET_FIELD(readl(pmc->pc[clk_num]), CLK_FREQ_TYPE, freq_type),
		pmc->pc[clk_num]);
	mutex_unlock(&pmc->pc_mutex);

	return 0;
}
EXPORT_SYMBOL(pmc_pc_set_freq);

/*
 * pmc_pc_configure - Configure the specified platform clock
 * @clk_num: Platform clock number (i.e. 0, 1, 2, ...,5)
 * @conf:      Clock gating:
 *		0   - Clock gated on D3 state
 *		1   - Force on
 *		2,3 - Force off
 */
int pmc_pc_configure(int clk_num, u32 conf)
{
	if (unlikely(!pmc))
		return -EAGAIN;

	if (clk_num < 0 && clk_num > PLT_CLK_MAXCOUNT) {
		dev_err(&pmc->pdev->dev,
			"Clock number out of range (%d)\n", clk_num);
		return -EINVAL;
	}

	if (conf != CLK_CONFG_D3_GATED &&
	    conf != CLK_CONFG_FORCE_ON &&
	    conf != CLK_CONFG_FORCE_OFF) {
		dev_err(&pmc->pdev->dev,
			"Invalid clock configuration requested\n");
		return -EINVAL;
	}

	mutex_lock(&pmc->pc_mutex);
	writel(REG_SET_FIELD(readl(pmc->pc[clk_num]), CLK_CONFG, conf),
		pmc->pc[clk_num]);
	mutex_unlock(&pmc->pc_mutex);
	return 0;
}
EXPORT_SYMBOL(pmc_pc_configure);

static inline u32 pmc_register_read(int reg_offset)
{
	return readl(pmc->pmc_registers + reg_offset);
}

static void print_residency_per_state(struct seq_file *s, int state)
{
	struct pmc_dev *pmc_cxt = (struct pmc_dev *)s->private;
	u32 rem_time, rem_res = 0;
	u64 rem_res_reduced = 0;
	u64 residency = pmc_cxt->state_residency[state];

	/* Counter increments every 32 us. */
	u64 time = residency << 5;
	residency *= 100;

	if (pmc_cxt->residency_total) {
		rem_res = do_div(residency, pmc_cxt->residency_total);
		rem_res_reduced = (u64)rem_res * 1000;
		do_div(rem_res_reduced,  pmc_cxt->residency_total);
	}
	rem_time = do_div(time, USEC_PER_SEC);
	seq_printf(s, "%s \t\t %.6llu.%.6u \t\t %.2llu.%.3llu", states[state],
			time, rem_time, residency, rem_res_reduced);
	if (state == STATE_S3)
		seq_printf(s, " \t\t %u\n", pmc_cxt->s3_count);
	else
		seq_printf(s, " \t\t %s\n", "--");
}

static int pmc_devices_state_show(struct seq_file *s, void *unused)
{
	struct pmc_dev *pmc_cxt = (struct pmc_dev *)s->private;
	int i, j;
	u32 val, nc_pwr_sts, reg, reg_d3sts0, reg_d3sts1, reg_func_dis,
		reg_func_dis2, tmp, pg_status;
	u8 bit, bit1;
	u32 ignore_flag;
	/*All Reserved bit removed*/
	u32 pg_ignore_flag = 0x3F86701;
	const char **d3_device;
	const char **pg_device;
	const struct nc_device *nc_devices;
	int cpu_dma_qos = pm_qos_request(PM_QOS_CPU_DMA_LATENCY);

	pmc_cxt->residency_total = 0;

	/* Read s0ix residency counters */
	for (i = STATE_S0IR; i < STATE_S3; i++) {
		pmc_cxt->state_residency[i] = pmc_register_read(i) -
					pmc_cxt->state_resi_offset[i];
		pmc_cxt->residency_total += pmc_cxt->state_residency[i];
	}
	/* In S3 (over S0i3), PMC will increase S0i3 residencey */
	pmc_cxt->state_residency[STATE_S0I3] -=
		pmc_cxt->state_residency[STATE_S3];

	reg_d3sts0 = readl(pmc_cxt->d3_sts0);
	pr_info("d3sts0 = %x\n", reg_d3sts0);
	reg_d3sts1 = readl(pmc_cxt->d3_sts1);
	pr_info("d3sts1 = %x\n", reg_d3sts1);
	reg_func_dis = readl(pmc_cxt->func_dis);
	pr_info("func_dis = %x\n", reg_func_dis);
	reg_func_dis2 = readl(pmc_cxt->func_dis2);
	pr_info("func_dis2 = %x\n", reg_func_dis2);
	pg_status = readl(pmc_cxt->pmc_pss);
	pr_info("Power Gate status = %x\n", pg_status);

	seq_puts(s, "\nPM_QOS LATENCIES\n");
	seq_printf(s, "CPU_DMA_QOS LATENCY= %d [us]\n\n", cpu_dma_qos);
	seq_printf(s, "CSTATE_EXIT_LATENCY_C1 = %d [us]\n", CSTATE_EXIT_LATENCY_C1);
	seq_printf(s, "CSTATE_EXIT_LATENCY_C6 = %d [us]\n", CSTATE_EXIT_LATENCY_C6);
	seq_printf(s, "CSTATE_EXIT_LATENCY_S0i1 = %d [us]\n", CSTATE_EXIT_LATENCY_S0i1);
	seq_printf(s, "CSTATE_EXIT_LATENCY_S0i3 = %d [us]\n\n", CSTATE_EXIT_LATENCY_S0i3);

	if (platform_is(INTEL_ATOM_BYT)) {
		nc_devices = nc_devices_byt;
		d3_device = d3_device_byt;
		/* swap bits 1 and 2 of func_dis2 to align them in order */
		tmp = reg_func_dis2;
		i = 1;
		j = 2;
		if (((tmp & (1 << i)) >> i) ^ ((tmp & (1 << j)) >> j)) {
			tmp ^= (1 << i);
			tmp ^= (1 << j);
		}
		reg_func_dis2 = tmp;
	} else {
		nc_devices = nc_devices_cht;
		d3_device = d3_device_cht;
		/* shift 3rd & 4th bits to 1st & 2nd position to align them in
		 * order and ignore 3rd bit of d3_sts1 as it is reserved */
		reg_func_dis2 &= 0x19;
		tmp = reg_func_dis2 & 0x1;
		reg_func_dis2 >>= 2;
		reg_func_dis2 |= tmp;
	}

	seq_puts(s, "State \t\t Time[sec] \t\t Residency[%%] \t\t Count\n");
	for (i = STATE_S0IR; i < STATE_MAX; i++)
		print_residency_per_state(s, i);

	seq_puts(s, "\n\nNORTH COMPLEX DEVICES :\n");

	/* WA: first read after S0ix (x>1) transitions is returning 0.
	 * Do a dummy read to make sure status are properly reflectd
	 * from next read onwards.
	 */
	nc_pwr_sts = intel_mid_msgbus_read32(PUNIT_PORT, PWRGT_STATUS);
	for (i = 0; i < NC_NUM_DEVICES; i++) {
		reg = nc_devices[i].reg;
		nc_pwr_sts = intel_mid_msgbus_read32(PUNIT_PORT, reg);
		nc_pwr_sts >>= nc_devices[i].sss_pos;
		val = nc_pwr_sts & PMC_D0I3_MASK;
		seq_printf(s, "%9s : %s\n", nc_devices[i].name, dstates[val]);
	}

	seq_puts(s, "\nSOUTH COMPLEX DEVICES :\n");

	for (j = 0; j < (SC_NUM_DEVICES-2); j++) {
		if (j >= 32) {
			ignore_flag = reg_func_dis2 & 0x01;
			reg_func_dis2 >>= 1;
			if (ignore_flag) {
				reg_d3sts1 >>= 1;
				continue;
			}
			bit  = reg_d3sts1 & 0x01;
			seq_printf(s, "%9s  : %s\n", *(d3_device + j),
					bit ? "D0i3" : "D0");
			reg_d3sts1 = reg_d3sts1 >> 1;
			continue;
		}
		ignore_flag = reg_func_dis & 0x01;
		reg_func_dis >>= 1;
		if (ignore_flag) {
			reg_d3sts0 >>= 1;
			continue;
		}
		bit  = reg_d3sts0 & 0x01;
		seq_printf(s, "%9s  : %s\n", *(d3_device + j),
				bit ? "D0i3" : "D0");
		reg_d3sts0 = reg_d3sts0 >> 1;
	}

	seq_puts(s, "\n");

	if (platform_is(INTEL_ATOM_CHT)) {
		seq_puts(s, "\nIP POWER GATE STATUS :\n");
		pg_device = pg_device_cht;
		for (j = 0; j < (SC_NUM_DEVICES-5); j++) {
					bit  = pg_status & 0x01;
					bit1 = pg_ignore_flag & 0x01;
			if (bit1 != 1)
				seq_printf(s, "%9s  : %s\n", *(pg_device + j),
						bit ? "is PG" : "not PG");
			pg_status >>= 1;
			pg_ignore_flag >>= 1;
			continue;
		}

		seq_puts(s, "\n");
	}
	return 0;
}


static int devices_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, pmc_devices_state_show, inode->i_private);
}

static ssize_t pmu_devices_state_write(struct file *file,
		const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[32];
	int buf_size = min(count, sizeof(buf)-1), state;
	char *clear_msg = "clear";
	int clear_msg_len = strlen(clear_msg);

	if (copy_from_user(buf, userbuf, buf_size))
		return -EFAULT;

	buf[buf_size] = 0;
	if (clear_msg_len + 1 == buf_size &&
		 !strncmp(buf, clear_msg, clear_msg_len)) {
		pmc->s3_count = 0;
		pmc->state_residency[STATE_S3] = 0;
		/* Mark the offset of S0iX residency counter in PMC */
		for (state = STATE_S0IR; state < STATE_S3; state++)
			pmc->state_resi_offset[state] =
				pmc_register_read(state);
	}
	return buf_size;
}

static const struct file_operations devices_state_operations = {
	.open           = devices_state_open,
	.read           = seq_read,
	.write		= pmu_devices_state_write,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int nc_set_power_show(struct seq_file *s, void *unused)
{
	return 0;
}

static ssize_t nc_set_power_write(struct file *file,
		const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[64];
	int islands, state, reg, buf_size;

	buf_size = count < 64 ? count : 64;

	if (copy_from_user(buf, userbuf, buf_size))
		return -EFAULT;

	if (sscanf(buf, "%d %d %d", &islands, &state, &reg) != 3)
		return -EFAULT;

	pmc_nc_set_power_state(islands, state, reg);
	return count;
}

static int nc_set_power_open(struct inode *inode, struct file *file)
{
	return single_open(file, nc_set_power_show, inode->i_private);
}

static const struct file_operations nc_set_power_operations = {
	.open           = nc_set_power_open,
	.read           = seq_read,
	.write          = nc_set_power_write,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int s0i3_enable_show(struct seq_file *s, void *unused)
{
	return 0;
}

static ssize_t s0i3_enable_write(struct file *file,
		const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[64];
	int val;
	unsigned int buf_size;
	u32 mtpmc_1 = 0;
		buf_size = count < 64 ? count : 64;

	if (copy_from_user(buf, userbuf, buf_size))
		return -EFAULT;

	if (sscanf(buf, "%d", &val) != 1)
		return -EFAULT;
	if (val == 1) {
		pr_info(KERN_INFO "Platform Clocks forced OFF\n");
		{
			int i;

			for (i = 0; i < PLT_CLK_MAXCOUNT; i++)
				pmc_pc_configure(i, 0);
		}
		pr_info(KERN_INFO "Force LPCC clk\n");
		writel(DYN_MODE, pmc->lpcc);
		pr_info(KERN_INFO "PLL force off mtpmc\n");
		mtpmc_1 = readl(pmc->mtpmc_1);
		mtpmc_1 |= MTPMC_VAL;
		writel(mtpmc_1, pmc->mtpmc_1);
		pr_info(KERN_INFO "Wait to clear VLV PM STS\n");
		do {
			pr_info(KERN_INFO "... vlv_pm_sts = %x\n", readl(pmc->vlv_pm_sts));
		} while ((readl(pmc->vlv_pm_sts)));
		pr_info(KERN_INFO "S0i3 enabled\n");
	}
	return count;
}
static int s0i3_enable_open(struct inode *inode, struct file *file)
{
	return single_open(file, s0i3_enable_show, inode->i_private);
}

static const struct file_operations s0i3_enable_operations = {
	.open           = s0i3_enable_open,
	.read           = seq_read,
	.write          = s0i3_enable_write,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int func_dis_show(struct seq_file *s, void *unused)
{
	return 0;
}

static ssize_t func_dis_write(struct file *file,
		const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[64];
	int val;
	unsigned int buf_size;
	u32 mtpmc_1 = 0;
	u32 fd;

	buf_size = count < 64 ? count : 64;

	if (copy_from_user(buf, userbuf, buf_size))
		return -EFAULT;

	if (sscanf(buf, "%d", &val) != 1)
		return -EFAULT;

	if (val < (SC_NUM_DEVICES-4)) {
		fd = readl(pmc->func_dis);
		fd |= (1<<val);
		writel(fd, pmc->func_dis);
		pr_info("Disabling IP %s\n", d3_device_cht[val]);
	} else if (val < (SC_NUM_DEVICES-1)) {
		fd = readl(pmc->func_dis2);
		fd |= (1<<val);
		writel(fd, pmc->func_dis2);
		pr_info("Disabling IP %s\n", d3_device_cht[val]);
	} else
		pr_err("Invalid South IP\n");

	return count;
}
static int func_dis_open(struct inode *inode, struct file *file)
{
	return single_open(file, func_dis_show, inode->i_private);
}

static const struct file_operations func_dis_operations = {
	.open           = func_dis_open,
	.read           = seq_read,
	.write          = func_dis_write,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int sc_set_power_show(struct seq_file *s, void *unused)
{
	return 0;
}

static ssize_t sc_set_power_write(struct file *file,
		const char __user *userbuf, size_t count, loff_t *ppos)
{
	char buf[64];
	unsigned int device, state, function, buf_size;
	struct pci_dev *pdev = NULL;

	buf_size = count < 64 ? count : 64;

	if (copy_from_user(buf, userbuf, buf_size))
		return -EFAULT;

	if (sscanf(buf, "%u %u %u", &device, &function, &state) != 3)
		return -EFAULT;

	state &= PCI_D3hot;

	while ((pdev = pci_get_device(PCI_ID_ANY, PCI_ID_ANY, pdev)) != NULL) {
		if (PCI_DEVFN(device, function) == pdev->devfn) {
			dev_dbg(&pdev->dev, "Forced to %s\n", dstates[state]);
			pci_set_power_state(pdev, state);
			break;
		}
	}

	return count;
}

static int sc_set_power_open(struct inode *inode, struct file *file)
{
	return single_open(file, sc_set_power_show, NULL);
}

static const struct file_operations sc_set_power_operations = {
	.open           = sc_set_power_open,
	.read           = seq_read,
	.write          = sc_set_power_write,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int pmc_suspend_enter(suspend_state_t state)
{
	u32 temp = 0, last_s0i3_residency, s3_res;

	if (platform_is(INTEL_ATOM_CHT))
		writel((PMC_WAKE_EN_SETTING & ~(0x4000)), pmc->s0ix_wake_en);

	if (state != PM_SUSPEND_MEM)
		return -EINVAL;

	last_s0i3_residency = pmc_register_read(STATE_S0I3);
	trace_printk("s3_entry\n");

	__monitor((void *)&temp, 0, 0);
	smp_mb();
	__mwait(BYT_S3_HINT, 1);

	trace_printk("s3_exit\n");

	if (platform_is(INTEL_ATOM_CHT))
		writel(PMC_WAKE_EN_SETTING, pmc->s0ix_wake_en);

	s3_res = pmc_register_read(STATE_S0I3) - last_s0i3_residency;
	if (s3_res) {
		pmc->state_residency[STATE_S3] += s3_res;
		pmc->s3_count += 1;
	}

	return 0;
}

static void put_driverless_pci_devices_in_d0i3(void)
{
	struct pci_dev *pdev = NULL;
	u16 pmcsr;

	while ((pdev = pci_get_device(PCI_ID_ANY, PCI_ID_ANY, pdev))
								!= NULL) {
		pci_read_config_word(pdev, pdev->pm_cap + PCI_PM_CTRL, &pmcsr);

		/* In case, device doesn't have driver and it's in D0,
		 * put it in D0i3 */
		if (IS_ERR_OR_NULL(pdev->dev.driver) &&
				!(pmcsr & PMC_D0I3_MASK)) {
			dev_info(&pdev->dev, "put device in D0i3\n");
			pmcsr |= PMC_D0I3_MASK;
			pci_write_config_word(pdev, pdev->pm_cap +
						PCI_PM_CTRL, pmcsr);
		}
	}
}

static int mid_suspend_prepare(void)
{
	put_driverless_pci_devices_in_d0i3();
	return 0;
}

static const struct platform_suspend_ops pmc_suspend_ops = {
	.valid = suspend_valid_only_mem,
	.prepare = mid_suspend_prepare,
	.enter = pmc_suspend_enter,
};

static DEFINE_PCI_DEVICE_TABLE(pmc_pci_tbl) = {
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x0F1C)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x229C)},
	{0,}
};
MODULE_DEVICE_TABLE(pci, pmc_pci_tbl);

#define BYT_TCO_IRQ 51
#define BYT_TCO_IRQ_SETTINGS IORESOURCE_IRQ_HIGHEDGE

#define CHT_TCO_IRQ 108
#define CHT_TCO_IRQ_SETTINGS IORESOURCE_IRQ_LOWEDGE

static struct resource tco_warn_interrupt[] = {
	{	.name = "warning_interrupt",
		.start = BYT_TCO_IRQ,
		.end = BYT_TCO_IRQ,
		.flags = IORESOURCE_IRQ | BYT_TCO_IRQ_SETTINGS
	},
	{	.name = "warning_interrupt",
		.start = CHT_TCO_IRQ,
		.end = CHT_TCO_IRQ,
		.flags = IORESOURCE_IRQ | CHT_TCO_IRQ_SETTINGS
	}
};

static void pmc_pci_register_tco_device(struct pci_dev *pdev,
					struct pmc_dev *pmc_cxt)
{
	unsigned int i;
	struct resource *res = NULL;

	for (i = 0 ; i < ARRAY_SIZE(pmc_pci_tbl) ; i++)
		if (pmc_pci_tbl[i].device == pdev->device) {
			if (i < ARRAY_SIZE(tco_warn_interrupt))
				res = &tco_warn_interrupt[i];
			break;
		}

	pmc_cxt->tco_wdt_dev =
		platform_device_register_resndata(&pdev->dev, "iTCO_wdt", -1,
						  res, res != NULL ? 1 : 0,
						  NULL, 0);

	if (IS_ERR(pmc_cxt->tco_wdt_dev)) {
		dev_err(&pdev->dev,
			"Failed to create iTCO_wdt platform device\n");
		return;
	}
}

static int pmc_pci_probe(struct pci_dev *pdev,
				const struct pci_device_id *id)
{
	int error = 0, state;
	struct dentry *d1, *d2, *d3, *d4;
	struct pmc_dev *pmc_cxt;
	int i;

	pmc_cxt = devm_kzalloc(&pdev->dev,
			sizeof(struct pmc_dev), GFP_KERNEL);

	if (!pmc_cxt) {
		dev_err(&pdev->dev, "Failed to allocate memory for pmc_cxt.\n");
		return -ENOMEM;
	}

	pmc = pmc_cxt;

	if (pci_enable_device(pdev)) {
		dev_err(&pdev->dev, "Failed to initialize PMC as PCI device\n");
		error = -EFAULT;
		goto exit_err;
	}

	pci_read_config_dword(pdev, PCI_CB_LEGACY_MODE_BASE,
					&pmc_cxt->base_address);
	pmc_cxt->base_address &= BASE_ADDRESS_MASK;

	if (pci_request_region(pdev, PMC_MMIO_BAR, "pmc_driver")) {
		dev_err(&pdev->dev, "Failed to allocate requested PCI region\n");
		error = -EFAULT;
		goto exit_err;
	}

	pmc_cxt->pmc_registers = devm_ioremap_nocache(&pdev->dev,
		pmc_cxt->base_address + S0IX_REGISTERS_OFFSET, 20);

	pmc_cxt->s0ix_wake_en = devm_ioremap_nocache(&pdev->dev,
		pmc_cxt->base_address + S0IX_WAKE_EN, 4);

	pmc_cxt->d3_sts0 = devm_ioremap_nocache(&pdev->dev,
		pmc_cxt->base_address + D3_STS0, 4);
	pmc_cxt->d3_sts1 = devm_ioremap_nocache(&pdev->dev,
		pmc_cxt->base_address + D3_STS1, 4);

	pmc_cxt->func_dis = devm_ioremap_nocache(&pdev->dev,
		pmc_cxt->base_address + FUNC_DIS, 4);
	pmc_cxt->func_dis2 = devm_ioremap_nocache(&pdev->dev,
		pmc_cxt->base_address + FUNC_DIS2, 4);

	pmc_cxt->pmc_pss = devm_ioremap_nocache(&pdev->dev,
		pmc_cxt->base_address + PMC_PSS, 4);
/* These changes should be reverted once PMC bugs are fixed*/
	{
		int i;

		for (i = 0; i < PLT_CLK_MAXCOUNT; i++)
			pmc_cxt->pc[i] = devm_ioremap_nocache(&pdev->dev,
				pmc_cxt->base_address + PLT_CLK(i), 4);
	}
	pmc_cxt->lpcc = devm_ioremap_nocache(&pdev->dev,
		LPCC_ADDR, 4);
	pmc_cxt->mtpmc_1 = devm_ioremap_nocache(&pdev->dev,
		pmc_cxt->base_address + MTPMC, 4);
	pmc_cxt->vlv_pm_sts = devm_ioremap_nocache(&pdev->dev,
		pmc_cxt->base_address + VLV_PM_STS, 4);


	if (!pmc_cxt->pmc_registers || !pmc_cxt->s0ix_wake_en) {
		dev_err(&pdev->dev, "Failed to map PMC registers.\n");
		error = -EFAULT;
		goto err_release_region;
	}

	suspend_set_ops(&pmc_suspend_ops);

	spin_lock_init(&pmc->nc_ready_lock);
	mutex_init(&pmc->pc_mutex);

	pci_set_drvdata(pdev, pmc_cxt);

	/* Register the Watchdog device */
	pmc_pci_register_tco_device(pdev, pmc_cxt);

	/* /sys/kernel/debug/mid_pmu_states */
	d1 = debugfs_create_file("mid_pmu_states", S_IFREG | S_IRUGO,
				NULL, pmc_cxt, &devices_state_operations);
	if (!d1) {
		dev_err(&pdev->dev, "Can not create a debug file\n");
		error = -ENOMEM;
		goto err_release_region;
	}

	/* /sys/kernel/debug/nc_set_power */
	d2 = debugfs_create_file("nc_set_power", S_IFREG | S_IRUGO,
				NULL, pmc_cxt, &nc_set_power_operations);

	if (!d2) {
		dev_err(&pdev->dev, "Can not create a debug file\n");
		error = -ENOMEM;
		debugfs_remove(d1);
		goto err_release_region;
	}

	/* Mark the offset of S0iX residency counter in PMC */
	for (state = STATE_S0IR; state < STATE_S3; state++)
		pmc_cxt->state_resi_offset[state] = pmc_register_read(state);

	/* /sys/kernel/debug/sc_set_power */
	d3 = debugfs_create_file("sc_set_power", S_IFREG | S_IRUGO,
				NULL, NULL, &sc_set_power_operations);

	if (platform_is(INTEL_ATOM_CHT)) {
		/* temperory debugfs entry for S0i3 enabling */
		d3 = debugfs_create_file("s0i3_enable", S_IFREG | S_IRUGO,
							NULL, NULL, &s0i3_enable_operations);

		if (!d3) {
			dev_err(&pdev->dev, "Can not create a debug file\n");
			error = -ENOMEM;
			debugfs_remove(d1);
			debugfs_remove(d2);
			goto err_release_region;
		}

		d4 = debugfs_create_file("func_dis", S_IFREG | S_IRUGO,
								NULL, NULL, &func_dis_operations);

		if (!d4) {
			dev_err(&pdev->dev, "Can not create a debug file\n");
			error = -ENOMEM;
			debugfs_remove(d1);
			debugfs_remove(d2);
			goto err_release_region;
		}


		pr_info("Forcing all platform clocks to OFF\n");
		for (i = 0; i < PLT_CLK_MAXCOUNT; i++)
			pmc_pc_configure(i, CLK_CONFG_FORCE_OFF);
	}
	writel(PMC_WAKE_EN_SETTING, pmc_cxt->s0ix_wake_en);

	return 0;

err_release_region:
	platform_device_unregister(pmc_cxt->tco_wdt_dev);
	pci_release_region(pdev, PMC_MMIO_BAR);
exit_err:
	dev_err(&pdev->dev, "Initialization failed\n");

	return error;
}

static struct pci_driver pmc_pci_driver = {
	.name = "pmc",
	.id_table = pmc_pci_tbl,
	.probe = pmc_pci_probe,
};

module_pci_driver(pmc_pci_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel ATOM Platform Power Management Controller (PMC) Driver");
