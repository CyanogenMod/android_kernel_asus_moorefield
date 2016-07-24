#ifndef __IA_CSS_PSYS_PROGRAM_MANIFEST_HSYS_USER_H_INCLUDED__
#define __IA_CSS_PSYS_PROGRAM_MANIFEST_HSYS_USER_H_INCLUDED__

/*! \file */

/** @file ia_css_psys_program_manifest.hsys.user.h
 *
 * Define the methods on the program manifest object: Hsys user interface
 */

#include <ia_css_psys_manifest_types.h>

/*! Print the program manifest object to file/stream

 @param	manifest[in]			program manifest object
 @param	fid[out]				file/stream handle

 @return < 0 on error
 */
extern int ia_css_program_manifest_print(
	const ia_css_program_manifest_t			*manifest,
	void									*fid);

#endif /* __IA_CSS_PSYS_PROGRAM_MANIFEST_HSYS_USER_H_INCLUDED__ */
