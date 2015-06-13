#include <linux/console.h>
#include <linux/cpu.h>
#include <linux/percpu.h>
#include <linux/npk.h>
#include <asm/fixmap.h>

/* Software can send messages to North Peak device by writing to an MMIO space
 * that is divided in several Master/Channel regions.
 * One master is reserved for each CPU. Within that master, several channels are
 * reserved for early printk messages.
 */
DEFINE_PER_CPU(unsigned long[CONFIG_NPK_NUM_CHANNELS_FOR_EARLY_PRINTK], sth_phys_addr);

/* Trace messages can be nested, e.g. if they are generated in interrupt.
 * In case of nested trace messages, different channels are used for each
 * message.
 */
DEFINE_PER_CPU(int, nesting_level);

#define STH_OUT_D32(channel, data, start)			\
	do { if (start)					\
			iowrite32(data, &channel->DnTS.d32);	\
		else						\
			iowrite32(data, &channel->Dn.d32);	\
	} while (0)

#define STH_OUT_D16(channel, data, start)			\
	do { if (start)					\
			iowrite16(data, &channel->DnTS.d16);	\
		else						\
			iowrite16(data, &channel->Dn.d16);	\
	} while (0)

#define STH_OUT_D8(channel, data, start)			\
	do { if (start)					\
			iowrite8(data, &channel->DnTS.d8);	\
		else						\
			iowrite8(data, &channel->Dn.d8);	\
	} while (0)

#define STH_OUT_FLAG(channel)			\
	iowrite32(0, &channel->FLAG);

static int early_npk_console_init(void)
{
	int i, j;

	get_online_cpus();
	for_each_online_cpu(i) {
		for (j = 0; j < CONFIG_NPK_NUM_CHANNELS_FOR_EARLY_PRINTK; j++)
			per_cpu(sth_phys_addr, i)[j] = CONFIG_NPK_SW_BAR +
				i * CONFIG_NPK_CHANNEL_COUNT +
				(CONFIG_NPK_FIRST_CHANNEL_FOR_EARLY_PRINTK + j) * sizeof(struct sven_sth_channel);
		per_cpu(nesting_level, i) = 0;
	}
	put_online_cpus();

	return 0;
}

static void early_npk_write(struct console *con, const char *buf, unsigned len)
{
	int i = 0, level, cpu;
	int num_d32;
	int num_trailing_bytes;
	struct sven_sth_channel *channel;
	unsigned long phys_addr;
	const u8 *p = buf;

	cpu = get_cpu();
	level = per_cpu(nesting_level, cpu)++;
	if (level >= CONFIG_NPK_NUM_CHANNELS_FOR_EARLY_PRINTK) {
		/* Nesting level is bigger than supported, drop the message */
		per_cpu(nesting_level, cpu)--;
		put_cpu();
		return;
	}

	phys_addr = per_cpu(sth_phys_addr, cpu)[level];

	set_fixmap_nocache(FIX_EARLYCON_MEM_BASE, phys_addr);
	channel = (struct sven_sth_channel *)
		(__fix_to_virt(FIX_EARLYCON_MEM_BASE) +
		 (phys_addr & (PAGE_SIZE - 1)));

	num_d32 = len >> 2;
	num_trailing_bytes = len - (num_d32 << 2);

	if (num_d32 > 0) {
		STH_OUT_D32(channel, *(u32 *)p, true);
		p += 4;
		i = 1;
	}

	for (; i < num_d32; i++) {
		STH_OUT_D32(channel, *(u32 *)p, false);
		p += 4;
	}

	switch (num_trailing_bytes) {
	case 1:
		STH_OUT_D8(channel, *(u8 *)p, (i == 0));
		break;
	case 2:
		STH_OUT_D16(channel, *(u16 *)p, (i == 0));
		break;
	case 3:
		STH_OUT_D16(channel, *(u16 *)p, (i == 0));
		p += 2;
		STH_OUT_D8(channel, *(u8 *)p, false);
		break;
	}

	STH_OUT_FLAG(channel);

	per_cpu(nesting_level, cpu)--;
	put_cpu();
}

struct console early_npk_console = {
	.name = "earlynpk",
	.early_setup = early_npk_console_init,
	.write = early_npk_write,
	.flags = CON_PRINTBUFFER,
	.index = -1,
};
