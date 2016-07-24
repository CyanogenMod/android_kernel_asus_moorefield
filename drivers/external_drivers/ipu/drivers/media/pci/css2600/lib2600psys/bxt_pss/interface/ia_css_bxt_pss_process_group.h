#ifndef __IA_CSS_BXT_PSS_PROCESS_GROUP_H_
#define __IA_CSS_BXT_PSS_PROCESS_GROUP_H_

#include "ia_css_psys_process_group_cmd_impl.h"

int ia_css_bxt_pss_process_group_load_addtional(
	ia_css_process_group_t					*process_group);

int ia_css_bxt_pss_process_group_run(
	ia_css_process_group_t					*process_group);

int ia_css_bxt_pss_process_group_stop(
	ia_css_process_group_t					*process_group);

/* TODO: remove next functions when moved internally */

bool process_group_ready(void);
int platform_load(void);
int platform_start(void);
int platform_stop(void);


#endif /* __IA_CSS_BXT_PSS_PROCESS_GROUP_H_ */
