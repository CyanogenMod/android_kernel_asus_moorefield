#ifndef __IA_CSS_BXT_PSS_PROGRAM_GROUPS_H_INCLUDED__
#define __IA_CSS_BXT_PSS_PROGRAM_GROUPS_H_INCLUDED__

#include "ia_css_program_group_param_types.h"
#include "ia_css_psys_manifest_types.h"

#define IA_CSS_BXT_PSS_PG_SPECIFICATION_4K60 0
#define IA_CSS_BXT_PSS_PG_SPECIFICATION_SMALL 1

ia_css_program_group_param_t *ia_css_bxt_pss_program_group_param_create(
	const unsigned int			specification,
	const ia_css_program_group_manifest_t	*manifest);

ia_css_program_group_manifest_t *ia_css_bxt_pss_program_group_manifest_create(
	const unsigned int			specification);

#endif /*__IA_CSS_BXT_PSS_PROGRAM_GROUPS_H_INCLUDED__ */
