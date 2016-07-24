#ifndef __IA_CSS_ISYS_STREAM_DEVICE_H_INCLUDED__
#define __IA_CSS_ISYS_STREAM_DEVICE_H_INCLUDED__

#include "ia_css_isys_fw_bridged_types.h"  /* ia_css_isys_stream_cfg_data */
#include "type_support.h"

/* Description : This layer is used by the stream controller and
				 performs actions for one specific stream(HW configuration, open,
				 frame capture)
*/

#define EVENT_SID_MAX				(64)	// 2^6

#define PID_FROM_EVENT(event)		(((event)&0x03F00000)>>20)
#define SID_FROM_EVENT(event)		(((event)&0xFC000000)>>26)
#define STREAMID_FROM_EVENT(event)	(((PID_FROM_EVENT(event))-1)&0x1F)

#define INFO_FROM_EVENT_SID(event,ibuf_id,proc_id)						\
do {																	\
	unsigned int sid = SID_FROM_EVENT(event);							\
	ibuf_id = ((sid&0x30)>>4);											\
	proc_id = (sid&0x0F);												\
} while(0)
#define INFO_FROM_EVENT_PID(event,is_acc,stream_id)						\
do {																	\
	unsigned int pid = PID_FROM_EVENT(event);							\
	is_acc = ((pid&0x20)>>5);											\
	stream_id = ((pid-1)&0x1F);											\
} while(0)
#define INFO_FROM_EVENT_SIDPID(event,ibuf_id,proc_id,is_acc,stream_id)	\
do {																	\
	INFO_FROM_EVENT_SID(event,ibuf_id,proc_id);							\
	INFO_FROM_EVENT_PID(event,is_acc,stream_id);						\
} while(0)

#define INFO_TO_EVENT_SID(ibuf_id,proc_id)								\
(																		\
	(((ibuf_id)<<4)&0x30) |												\
	((proc_id)&0x0F)													\
)
#define INFO_TO_EVENT_PID(is_acc,stream_id)								\
(																		\
	(((is_acc)<<5)&0x20) |												\
	(((stream_id)+1)&0x01F)												\
)
#define INFO_TO_EVENT_SIDPID(ibuf_id,proc_id,is_acc,stream_id)			\
(																		\
	((INFO_TO_EVENT_SID(ibuf_id,proc_id))<<6) |							\
	(INFO_TO_EVENT_PID(is_acc,stream_id))								\
)

enum ibufctrl_id {
	IA_CSS_ISYS_IBUFCTRL_RESERVED = 0,
	IA_CSS_ISYS_IBUFCTRL_MG = 1,
	IA_CSS_ISYS_IBUFCTRL_ISL = 2,
	IA_CSS_ISYS_IBUFCTRL_CSI2 = 3,
	N_IA_CSS_ISYS_IBUFCTRL = 4,
};

/* Return type for stream device API functions */
enum istream_err{
	IA_CSS_ISYS_STREAM_NO_ERROR = 0,
	IA_CSS_ISYS_STREAM_CREATE_STREAM_FAIL,
	IA_CSS_ISYS_STREAM_CONFIGURE_STREAM_FAIL,
	IA_CSS_ISYS_STREAM_OPEN_STREAM_FAIL,
	IA_CSS_ISYS_STREAM_TRANSFER_FAIL,
	IA_CSS_ISYS_STREAM_SYNC_FAIL,
	N_ISTREAM
};

/* Context for stream device API functions */
struct istream_ctx {
	unsigned int stream_id;
	struct ia_css_isys_stream_cfg_data_comm stream_cfg_data;
};

/* Stream Device function calls */

enum istream_err istream_configure(
	struct istream_ctx **ppistr_ctx,
	const unsigned int stream_id,
	const struct ia_css_isys_stream_cfg_data_comm *pstream_dev_cfg
);

enum istream_err istream_open(
	struct istream_ctx *pistr_ctx
);

enum istream_err istream_capture(
	struct istream_ctx *pistr_ctx,
	const struct ia_css_isys_frame_buff_set_comm *frame_buf
);

enum istream_err istream_sync(
	struct istream_ctx *pistr_ctx
);
#endif /* __IA_CSS_ISYS_STREAM_DEVICE_H_INCLUDED__ */

