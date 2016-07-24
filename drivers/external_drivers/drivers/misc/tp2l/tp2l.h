#ifndef __TP2L_H
#define __TP2L_H

#include <linux/ftrace_event.h>
#include <linux/spinlock.h>
#include <linux/pti.h>

#define MAX_ENTRY_SIZE 4096

struct tp2l_element {
	char *system;
	char *name;
	void *probe_fn;
	int (*reg_fn)(void *probe, void *data);
	int (*unreg_fn)(void *probe, void *data);
	struct list_head list;
};

static DEFINE_SPINLOCK(tp2l_lock);
static struct trace_seq tp2l_seq;
static struct trace_seq tp2l_tmp_seq;
static char *tp2l_raw_buf[MAX_ENTRY_SIZE];

#endif /* __TP2L_H */




/* Do a first inclusion of the trace header file */
#include TP2L_INCLUDE_FILE

/* Let the trace headers be re-read */
#define TRACE_HEADER_MULTI_READ

#undef TRACE_EVENT
#define TRACE_EVENT(call, proto, args, tstruct, assign, print)	\
	DECLARE_EVENT_CLASS(call,				\
			    PARAMS(proto),			\
			    PARAMS(args),			\
			    PARAMS(tstruct),			\
			    PARAMS(assign),			\
			    PARAMS(print));			\
	DEFINE_EVENT(call, call, PARAMS(proto), PARAMS(args));

#undef DECLARE_EVENT_CLASS
#define DECLARE_EVENT_CLASS(call, proto, args, tstruct, assign, print)

#undef DEFINE_EVENT
#define DEFINE_EVENT(template, call, proto, args)

#undef TP_perf_assign
#define TP_perf_assign(args...)



/*************************************************************
 * Including tp2l.h file with DEFINE_TP2L_PROBES being defined
 * results in:
 *  - creating a TP2L element for each tracepoint event that is
 *    defined in the tracepoint header file (TP2L_INCLUDE_FILE)
 *  - defining a TP2L probe for each tracepoint event (the probe
 *    will output the event data in ascii format to the PTI port)
 */
#ifdef DEFINE_TP2L_PROBES

/* 1st step:
 * For each tracepoint event that is defined (via TRACE_EVENT
 * or DEFINE_EVENT) in the trace header file (TP2L_INCLUDE_FILE):
 *   - declare a tp2l_raw_<event> structure - which corresponds
 *     to the structure defined by TP_STRUCT__entry
 *   - define a tp2l_element named tp2l_<event>
 */
#undef __field
#define __field(type, item)		type	item;

#undef __field_ext
#define __field_ext(type, item, filter_type)	type	item;

#undef __array
#define __array(type, item, len)	type	item[len];

#undef __dynamic_array
#define __dynamic_array(type, item, len) u32 __data_loc_##item;

#undef __string
#define __string(item, src) __dynamic_array(char, item, -1)

#undef TP_STRUCT__entry
#define TP_STRUCT__entry(args...) args

#undef DECLARE_EVENT_CLASS
#define DECLARE_EVENT_CLASS(name, proto, args, tstruct, assign, print)	\
	struct tp2l_raw_##name {					\
		tstruct							\
		char			__data[0];			\
	};

#undef DEFINE_EVENT
#define DEFINE_EVENT(template, call, proto, args)			\
	static struct tp2l_element tp2l_##call = {			\
		.system = __stringify(TRACE_SYSTEM),			\
		.name = __stringify(call),				\
		.reg_fn = (void *)register_trace_##call,		\
		.unreg_fn = (void *)unregister_trace_##call,		\
	};

#include TP2L_INCLUDE_FILE



/* 2nd step:
 * For each tracepoint event that is defined in the trace header
 * file (TP2L_INCLUDE_FILE):
 *   - declare a tp2l_data_offsets_<event> structure (it will be
 *     used exclusively if the event has some __dynamic_array fields
 *     to store the offset and length of the dynamic data)
 */
#undef __field
#define __field(type, item)

#undef __field_ext
#define __field_ext(type, item, filter_type)

#undef __array
#define __array(type, item, len)

#undef __dynamic_array
#define __dynamic_array(type, item, len)	u32 item;

#undef __string
#define __string(item, src) __dynamic_array(char, item, -1)

#undef DECLARE_EVENT_CLASS
#define DECLARE_EVENT_CLASS(call, proto, args, tstruct, assign, print)	\
	struct tp2l_data_offsets_##call {				\
		tstruct;						\
	};

#undef DEFINE_EVENT
#define DEFINE_EVENT(template, name, proto, args)

#include TP2L_INCLUDE_FILE



/* 3rd step:
 * For each tracepoint event that is defined in the trace header
 * file (TP2L_INCLUDE_FILE):
 *   - define a tp2l_get_offsets_<event> function (it will be
 *     used exclusively if the event has some __dynamic_array fields
 *     to fill the tp2l_data_offsets_<event> structure)
 */
#undef __field
#define __field(type, item)

#undef __field_ext
#define __field_ext(type, item, filter_type)

#undef __array
#define __array(type, item, len)

#undef __dynamic_array
#define __dynamic_array(type, item, len)				\
	__data_offsets->item = __data_size +				\
			       offsetof(typeof(*entry), __data);	\
	__data_offsets->item |= (len * sizeof(type)) << 16;		\
	__data_size += (len) * sizeof(type);

#undef __string
#define __string(item, src) __dynamic_array(char, item, strlen(src) + 1)

#undef DECLARE_EVENT_CLASS
#define DECLARE_EVENT_CLASS(call, proto, args, tstruct, assign, print)	\
	static inline notrace int tp2l_get_offsets_##call(		\
		struct tp2l_data_offsets_##call *__data_offsets, proto)	\
	{								\
		int __data_size = 0;					\
		struct tp2l_raw_##call __maybe_unused *entry;		\
									\
		tstruct;						\
									\
		return __data_size;					\
	}

#undef DEFINE_EVENT
#define DEFINE_EVENT(template, name, proto, args)

#include TP2L_INCLUDE_FILE



/* 4th step:
 * For each tracepoint event that is defined in the trace header
 * file (TP2L_INCLUDE_FILE), define a tp2l_probe_<event> function
 * This is the tp2l probe, which is responsible for outputting the
 * event in ascii format (as defined by its TP_printk) to the PTI
 */
#undef __entry
#define __entry entry

#undef __field
#define __field(type, item)

#undef __array
#define __array(type, item, len)

#undef __dynamic_array
#define __dynamic_array(type, item, len)			\
	__entry->__data_loc_##item = __data_offsets.item;

#undef __string
#define __string(item, src) __dynamic_array(char, item, -1)

#undef __assign_str
#define __assign_str(dst, src) strcpy(__get_str(dst), src);

#undef __get_dynamic_array
#define __get_dynamic_array(field)					\
		((void *)__entry + (__entry->__data_loc_##field & 0xffff))

#undef __get_str
#define __get_str(field) ((char *)__get_dynamic_array(field))

#undef TP_fast_assign
#define TP_fast_assign(args...) args

#undef TP_printk
#define TP_printk(fmt, args...) fmt "\n", args	\

#undef __print_flags
#define __print_flags(flag, delim, flag_array...)			\
	({								\
		static struct trace_print_flags __flags[] =		\
			{ flag_array, { -1, NULL } };			\
		ftrace_print_flags_seq(&tp2l_tmp_seq,			\
				       delim, flag, __flags);		\
	})

#undef __print_symbolic
#define __print_symbolic(value, symbol_array...)			\
	({								\
		static struct trace_print_flags symbols[] =		\
			{ symbol_array, { -1, NULL } };			\
		ftrace_print_symbols_seq(&tp2l_tmp_seq,			\
					 value, symbols);		\
	})

#undef __print_symbolic_u64
#if BITS_PER_LONG == 32
#define __print_symbolic_u64(value, symbol_array...)			\
	({								\
		static struct trace_print_flags symbols[] =		\
			{ symbol_array, { -1, NULL } };			\
		ftrace_print_symbols_seq_u64(&tp2l_tmp_seq,		\
					     value, symbols);		\
	})
#else
#define __print_symbolic_u64(value, symbol_array...)			\
			__print_symbolic(value, symbol_array)
#endif

#undef __print_hex
#define __print_hex(buf, buf_len...)					\
	ftrace_print_hex_seq(&tp2l_tmp_seq, buf, buf_len)

#undef DECLARE_EVENT_CLASS
#define DECLARE_EVENT_CLASS(call, proto, args, tstruct, assign, print)	\
static notrace void							\
tp2l_probe_##call(void *__data, proto)					\
{									\
	struct tp2l_element *tp2l_info;					\
	struct tp2l_data_offsets_##call __maybe_unused __data_offsets;	\
	struct tp2l_raw_##call *entry =					\
		(struct tp2l_raw_##call *)tp2l_raw_buf;			\
	int __data_size;						\
	unsigned long irq_flags;					\
									\
	__data_size = tp2l_get_offsets_##call(&__data_offsets, args);	\
									\
	if (sizeof(*entry) + __data_size > MAX_ENTRY_SIZE)		\
		return;							\
									\
	spin_lock_irqsave(&tp2l_lock, irq_flags);			\
									\
	tstruct								\
									\
	{ assign; }							\
									\
	tp2l_info = (struct tp2l_element *)__data;			\
	trace_seq_init(&tp2l_seq);					\
	trace_seq_init(&tp2l_tmp_seq);					\
	trace_seq_printf(&tp2l_seq, "%s:%s ", tp2l_info->system,	\
			 tp2l_info->name);				\
	trace_seq_printf(&tp2l_seq, print);				\
									\
	pti_writedata(mc, tp2l_seq.buffer, tp2l_seq.len, true);		\
									\
	spin_unlock_irqrestore(&tp2l_lock, irq_flags);			\
}

#undef DEFINE_EVENT
#define DEFINE_EVENT(template, call, proto, args)

#include TP2L_INCLUDE_FILE



#endif /* DEFINE_TP2L_PROBES */





/***********************************************************
 * Including tp2l.h file with ADD_TP2L_ELEMENT being defined
 * results in:
 *  - for each tracepoint event defined in the tracepoint
 *    header file (TP2L_INCLUDE_FILE), add the associated
 *    tp2l element to the tp2l_list
 */
#ifdef ADD_TP2L_ELEMENT

#undef DEFINE_EVENT
#define DEFINE_EVENT(template, call, proto, args)		\
	INIT_LIST_HEAD(&tp2l_##call.list);			\
	list_add_tail(&tp2l_##call.list, &tp2l_list);		\
	tp2l_##call.probe_fn = (void *)tp2l_probe_##template;

#include TP2L_INCLUDE_FILE

#endif /* ADD_TP2L_ELEMENT */





#undef DEFINE_EVENT
#undef DECLARE_EVENT_CLASS
#undef TRACE_EVENT
#undef TRACE_HEADER_MULTI_READ
#undef TP2L_INCLUDE_FILE

