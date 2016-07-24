#undef TRACE_SYSTEM
#define TRACE_SYSTEM gfx_idle

#if !defined(_TRACE_GFX_IDLE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_GFX_IDLE_H

#include <linux/tracepoint.h>

DECLARE_EVENT_CLASS(gfx_idle,

		TP_PROTO(int cpu),

		TP_ARGS(cpu),

		TP_STRUCT__entry(
			__field(int, cpu)
			),

		TP_fast_assign(
			__entry->cpu = cpu;
			),

		TP_printk("cpu=%d", __entry->cpu)
		);

DEFINE_EVENT(gfx_idle, gfx_idle_entry,
		TP_PROTO(int cpu), TP_ARGS(cpu));


DEFINE_EVENT(gfx_idle, gfx_idle_exit,
		TP_PROTO(int cpu), TP_ARGS(cpu));

DEFINE_EVENT(gfx_idle, gfx_idle_poweroff,
		TP_PROTO(int cpu), TP_ARGS(cpu));

#endif /* _TRACE_GFX_IDLE_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
