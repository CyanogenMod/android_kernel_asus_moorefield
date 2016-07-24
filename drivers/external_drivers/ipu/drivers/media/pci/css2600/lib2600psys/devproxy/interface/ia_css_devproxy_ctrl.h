/*
 * ia_css_devproxy_ctrl.h
 *
 *  Created on: Nov 28, 2013
 *      Author: jboogers
 */

#ifndef IA_CSS_DEVPROXY_CTRL_H_
#define IA_CSS_DEVPROXY_CTRL_H_

#include "type_support.h" /* for size_t */
#include "ia_css_devproxy_ctrl_service_pids.h"

#include "storage_class.h"

#ifndef __INLINE_DEVPROXY_CTRL__
#define IA_CSS_DEVPROXY_CTRL_STORAGE_CLASS_H STORAGE_CLASS_EXTERN
#define IA_CSS_DEVPROXY_CTRL_STORAGE_CLASS_C
#else				/* __INLINE_VIED_NCI_EQ__ */
#define IA_CSS_DEVPROXY_CTRL_STORAGE_CLASS_H STORAGE_CLASS_INLINE
#define IA_CSS_DEVPROXY_CTRL_STORAGE_CLASS_C STORAGE_CLASS_INLINE
#endif				/* __INLINE_VIED_NCI_EQ__ */

typedef struct ia_css_devproxy_ctrl_reference {
	uint32_t ref_type;
	uint32_t size;
	union {
		uint32_t memory;
		uint32_t offset;
	} ref;
} ia_css_devproxy_ctrl_reference_t;

IA_CSS_DEVPROXY_CTRL_STORAGE_CLASS_H
void ia_css_devproxy_ctrl_init(
	unsigned int service_pid, /* [in] the service to address */
	unsigned int channel_num, /* [in] the channel to use */
	ia_css_devproxy_ctrl_reference_t *ref   /* [in] reference to the data */
);


IA_CSS_DEVPROXY_CTRL_STORAGE_CLASS_H
void ia_css_devproxy_ctrl_next(
	unsigned int service_pid, /* [in] the service to address */
	unsigned int channel_num  /* [in] the channel to use */
);

IA_CSS_DEVPROXY_CTRL_STORAGE_CLASS_H
void ia_css_devproxy_ctrl_done(
	unsigned int service_pid, /* [in] the service to address */
	unsigned int channel_num  /* [in] the channel to use */
);

void ia_css_devproxy_ctrl_startup(void);
void ia_css_devproxy_ctrl_shutdown(void);

#ifdef __INLINE_DEVPROXY_CTRL__
#include "ia_css_devproxy_ctrl_inline.h"
#endif

#endif /*IA_CSS_DEVPROXY_CTRL_H_*/
