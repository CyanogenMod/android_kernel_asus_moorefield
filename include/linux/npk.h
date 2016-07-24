#ifndef _NPK_H_
#define _NPK_H_

#include <linux/ioctl.h>

#define NPK_IOCTL_MAGIC 'n'

/* ioctl commands to allocate/release an STH channel for SVEN instrumentation in user-space */
#define NPKIOC_ALLOC_PAGE_SVEN  _IOWR(NPK_IOCTL_MAGIC, 1, struct sth_page_request_info)
#define NPKIOC_FREE_PAGE_SVEN	_IOW(NPK_IOCTL_MAGIC, 1, struct sth_page_request_info)

/* ioctl commands to allocate/release a CSR-driven mode buffer for NPK trace */
#define NPKIOC_ALLOC_BUFF       _IOWR(NPK_IOCTL_MAGIC, 2, struct npk_csr_info)
#define NPKIOC_FREE_BUFF	_IOW(NPK_IOCTL_MAGIC, 2, u32)

/* ioctl commands to allocate/release a linked-list mode buffer for NPK trace */
#define NPKIOC_ALLOC_WINDOW	_IOWR(NPK_IOCTL_MAGIC, 3, struct npk_win_info)
#define NPKIOC_FREE_WINDOW	_IOW(NPK_IOCTL_MAGIC, 3, u32)

/* ioctl commands to read/write NPK registers */
#define NPKIOC_REG_READ         _IOWR(NPK_IOCTL_MAGIC, 4, struct npk_reg_cmd)
#define NPKIOC_REG_WRITE	_IOW(NPK_IOCTL_MAGIC, 4, struct npk_reg_cmd)
#define NPKIOC_PCI_REG_READ	_IOWR(NPK_IOCTL_MAGIC, 5, struct pci_cfg_cmd)
#define NPKIOC_PCI_REG_WRITE	_IOW(NPK_IOCTL_MAGIC, 5, struct pci_cfg_cmd)
#define NPKIOC_MEM_READ         _IOWR(NPK_IOCTL_MAGIC, 6, struct npk_mem_cmd)
#define NPKIOC_MEM_WRITE	_IOW(NPK_IOCTL_MAGIC, 6, struct npk_mem_cmd)

/* ioctl command to print debug info on NPK trace buffer */
#define NPKIOC_PRINT_MSU_INFO   _IOW(NPK_IOCTL_MAGIC, 7, u32)

/* ioctl commands to start/stop/configure trace */
#define NPKIOC_START_TRACE      _IO(NPK_IOCTL_MAGIC, 1)
#define NPKIOC_STOP_TRACE       _IO(NPK_IOCTL_MAGIC, 2)
#define NPKIOC_CONFIGURE_TRACE  _IOW(NPK_IOCTL_MAGIC, 8, struct npk_cfg)

union dn_sth_reg {
	u8 d8;
	u16 d16;
	u32 d32;
	u64 d64;
} __packed;

/* SW master/channel memory map (MIPI STPv2) */
struct sven_sth_channel {
	union dn_sth_reg Dn;
	union dn_sth_reg DnM;
	union dn_sth_reg DnTS;
	union dn_sth_reg DnMTS;
	u64 USER;
	u64 USER_TS;
	u32 FLAG;
	u32 FLAG_TS;
	u32 MERR;
	u32 reserved;
} __packed;

struct sth_page_request_info {
	u32 info_size;
	u32 channel_size;
	u32 offset;
	u32 length;
};

struct npk_csr_info {
	u32 idx;
	u32 size;
};

struct npk_win_info {
	u32 idx;
	u32 num_windows;
	u32 window_size;
	u32 num_blks;
	u32 blk_size;
	u32 pad;
};

struct npk_reg_cmd {
	u32 offset;
	u32 data;
};

struct pci_cfg_cmd {
	u32 offset;
	u32 data;
	u32 size;
};

struct npk_mem_cmd {
	u64 addr;
	u64 data;
	u32 size;
	u32 pad;
};

/* npk_cfg struct holds the NPK trace configuration (sources, destination):
 *   - swdest_bmp[n], n in [0..31], is a bitmap for Master IDs [8n..(8n+7)]
 *     to enable Master ID N, do swdest_bmp[N << 3] |= (1 << (N & 0x7))
 *   - gswtdest: indicates if SW Master IDs 256+ shall be enabled (0: no, 1:yes)
 *   - output_port: indicates the destination port (0: MSC0, 1: MSC1, 2: PTI)
 *   - pti_clk_divider: for PTI clock, use NorthPeak clock divided by 2^n, n in [0..3]
 *   - pti_mode: PTI port output mode (1: 4-bit, 2: 8-bit, 3: 12-bit, 4: 16-bit)
 *   - pti_pattern: PTI training/calibration pattern ID to use (0 means no pattern)
 *   - msc_mode: MSCn mode (0: CSR, 1: multi-block, 2: ExI, 3: MTB)
 *   - msc_wrap_en: indicates if wrapping is enabled for MSCn
 */
struct npk_cfg {
	u8 swdest_bmp[32];
	u8 gswtdest;
	u8 output_port;
	u8 pti_clk_divider;
	u8 pti_mode;
	u8 pti_pattern;
	u8 msc_mode;
	u8 msc_wrap_en;
};

void *npk_alloc_sth_sven_ptr(int cpu);
void npk_free_sth_sven_ptr(int cpu, void __iomem *channel);

#endif /* _NPK_H_ */
