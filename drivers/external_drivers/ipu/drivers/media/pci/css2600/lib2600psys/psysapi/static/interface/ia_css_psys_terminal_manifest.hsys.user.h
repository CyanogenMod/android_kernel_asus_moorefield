#ifndef __IA_CSS_PSYS_TERMINAL_MANIFEST_HSYS_USER_H_INCLUDED__
#define __IA_CSS_PSYS_TERMINAL_MANIFEST_HSYS_USER_H_INCLUDED__

/*! \file */

/** @file ia_css_psys_terminal.hsys.user.h
 *
 * Define the methods on the termianl manifest object: Hsys user interface
 */

#include <ia_css_psys_manifest_types.h>

/*! Print the terminal manifest object to file/stream

 @param	manifest[in]			terminal manifest object
 @param	fid[out]				file/stream handle

 @return < 0 on error
 */
extern int ia_css_terminal_manifest_print(
	const ia_css_terminal_manifest_t		*manifest,
	void									*fid);

#endif /* __IA_CSS_PSYS_TERMINAL_MANIFEST_HSYS_USER_H_INCLUDED__ */
