#undef TRACE_SYSTEM
#define TRACE_SYSTEM tp2l_test

#if !defined(_TP2L_TEST_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TP2L_TEST_TRACE_H

#include <linux/tracepoint.h>

TRACE_EVENT(silly,

	    TP_PROTO(unsigned long time, unsigned long count),

	    TP_ARGS(time, count),

	    TP_STRUCT__entry(
			     __field(unsigned long, time)
			     __field(unsigned long, count)
			     ),

	    TP_fast_assign(
			   __entry->time = time;
			   __entry->count = count;
			   ),

	    TP_printk("time=%lu count=%lu",
		      __entry->time, __entry->count)
	    );

DECLARE_EVENT_CLASS(crazy,

		    TP_PROTO(unsigned long time, unsigned long count),

		    TP_ARGS(time, count),

		    TP_STRUCT__entry(
				     __field(unsigned long, time)
				     __field(unsigned long, count)
				     ),

		    TP_fast_assign(
				   __entry->time = time;
				   __entry->count = count;
				   ),

		    TP_printk("time=%lu count=%lu",
			      __entry->time, __entry->count)
		    );

DEFINE_EVENT(crazy, crazy1,

	     TP_PROTO(unsigned long time, unsigned long count),

	     TP_ARGS(time, count)
	     );

DEFINE_EVENT(crazy, crazy2,

	     TP_PROTO(unsigned long time, unsigned long count),

	     TP_ARGS(time, count)
	     );

#endif /* _TP2L_TEST_TRACE_H */

/* This part must be outside protection */
#undef TRACE_INCLUDE_PATH
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_PATH .
#define TRACE_INCLUDE_FILE tp2l_test_trace
#include <trace/define_trace.h>
