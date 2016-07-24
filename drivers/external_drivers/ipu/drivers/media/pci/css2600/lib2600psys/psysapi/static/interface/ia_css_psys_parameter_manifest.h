#ifndef __IA_CSS_PSYS_PARAM_MANIFEST_H_INCLUDED__
#define __IA_CSS_PSYS_PARAM_MANIFEST_H_INCLUDED__

/*! \file */

/** @file ia_css_psys_parameter_manifest.h
 *
 * Define the methods on the parameter manifest object
 */

#include <ia_css_psys_manifest_types.h>

/*! Compute the size of storage required for allocating the parameter manifest object

 @return 0 on error
 */
extern size_t ia_css_sizeof_parameter_manifest(void);

/*! Print the parameter manifest object to file/stream

 @param	manifest[in]			parameter manifest object
 @param	fid[out]				file/stream handle

 @return < 0 on error
 */
extern int ia_css_parameter_manifest_print(
	const ia_css_parameter_manifest_t		*manifest,
	void									*fid);

#endif /* __IA_CSS_PSYS_PARAM_MANIFEST_H_INCLUDED__ */
