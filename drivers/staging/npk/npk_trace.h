#include <linux/dma-mapping.h>

#ifndef _NPK_TRACE_H
#define _NPK_TRACE_H

#define MSU_MODE_NONE 0
#define MSU_MODE_CSR  1
#define MSU_MODE_WIN  2

#define BLK_ENTRY_SIZE sizeof(struct msu_blk_entry)
#define BLK_ENTRIES_PER_PAGE (PAGE_SIZE / BLK_ENTRY_SIZE)

#define SW_TAG_LAST_BLK (1 << 0)
#define SW_TAG_LAST_WIN (1 << 1)

#define MAX_SINGLE_BLOCK_BUFFER_SIZE 0x100000
#define MAX_NUM_WINDOWS 32
#define MAX_WINDOW_SIZE 0x200000

struct msu_block {
	void *blk;
	dma_addr_t blk_pa;
};

struct msu_window {
	struct msu_block *blk_list;
	unsigned long nr_of_blks;
};

struct msu_ctx {
	void *mem;
	dma_addr_t mem_pa;
	u32 size;
	struct msu_window *win_list;
	int nr_of_wins;
	u32 win_size;
	int mode;
	bool used;
	u32 readpos;
	u32 len;
	u32 wrapstat;
	u32 idx;
	u32 left;
	u32 right;
};

union software_tag {
	u32 raw;
	struct {
		u32 last_block:1;
		u32 last_window:1;
		u32 reserved:30;
	} bits;
} __packed;

union hardware_tag {
	u32 raw;
	struct {
		u32 trigger_present:1;
		u32 block_wrapped:1;
		u32 window_wrapped:1;
		u32 end_block:1;
		u32 reserved:28;
	} bits;
} __packed;

struct msu_blk_entry {
	union software_tag sw_tag;
	u32 blk_size;
	u32 next_blk_addr;
	u32 next_win_addr;
	u32 reserved[4];
	union hardware_tag hw_tag;
	u32 len;
	u32 timestamp_l;
	u32 timestamp_h;
	u32 reserved1[4];
} __packed;

struct msu_win_ctx {
	int nr_wins;
	int nr_blks;
	int cur_win;
	bool win_wrap;
	bool data_ready;
};

int npk_trace_start(void);
int npk_trace_stop(void);
int npk_win_get_ctx(struct msu_win_ctx *ctx);
int npk_win_get_ordered_blocks(int win, struct scatterlist *sg_array);

#endif /* _NPK_TRACE_H */
