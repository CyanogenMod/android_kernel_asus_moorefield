#ifndef __IA_CSS_PSYS_TERMINAL_MANIFEST_H_INCLUDED__
#define __IA_CSS_PSYS_TERMINAL_MANIFEST_H_INCLUDED__

/*! \file */

/** @file ia_css_psys_terminal_manifest.h
 *
 * Define the methods on the terminal manifest object that are not part of a single interface
 */

#include <ia_css_psys_manifest_types.h>

#include <ia_css_psys_terminal_manifest.sim.h>

#include <ia_css_psys_terminal_manifest.hsys.user.h>

#include <ia_css_program_group_data.h>			/* ia_css_frame_format_bitmap_t */
#include <ia_css_kernel_bitmap.h>				/* ia_css_kernel_bitmap_t */

#include <type_support.h>						/* size_t */


/*! Check if the terminal manifest object specifies a (cached) parameter terminal type

 @param	manifest[in]			terminal manifest object

 @return is_parameter_terminal, false on error
 */
extern bool ia_css_is_terminal_manifest_parameter_terminal(
	const ia_css_terminal_manifest_t		*manifest);

/*! Check if the terminal manifest object specifies a data terminal type

 @param	manifest[in]			terminal manifest object

 @return is_data_terminal, false on error
 */
extern bool ia_css_is_terminal_manifest_data_terminal(
	const ia_css_terminal_manifest_t		*manifest);

/*! Get the stored size of the terminal manifest object

 @param	manifest[in]			terminal manifest object

 @return size, 0 on error
 */
extern size_t ia_css_terminal_manifest_get_size(
	const ia_css_terminal_manifest_t		*manifest);

/*! Get the (pointer to) the program group manifest parent of the terminal manifest object

 @param	manifest[in]			terminal manifest object

 @return the pointer to the parent, NULL on error
 */
extern ia_css_program_group_manifest_t *ia_css_terminal_manifest_get_parent(
	const ia_css_terminal_manifest_t		*manifest);

/*! Set the (pointer to) the program group manifest parent of the terminal manifest object

 @param	manifest[in]			terminal manifest object
 @param	terminal_offset[in]		this terminal's offset from program_group_manifest base address.

 @return < 0 on error
 */
extern int ia_css_terminal_manifest_set_parent_offset(
	ia_css_terminal_manifest_t				*manifest,
	int32_t terminal_offset);

/*! Get the type of the terminal manifest object

 @param	manifest[in]			terminal manifest object

 @return terminal type, limit value on error
 */
extern ia_css_terminal_type_t ia_css_terminal_manifest_get_type(
	const ia_css_terminal_manifest_t		*manifest);

/*! Set the type of the terminal manifest object

 @param	manifest[in]			terminal manifest object
 @param	terminal_type[in]		terminal type

 @return < 0 on error
 */
extern int ia_css_terminal_manifest_set_type(
	ia_css_terminal_manifest_t				*manifest,
	const ia_css_terminal_type_t			terminal_type);


/*! Get the number of parameter section on the (param) terminal manifest object

 @param	manifest[in]			(param) terminal manifest object

 @return section count, 0 on error
 */
extern uint8_t ia_css_param_terminal_manifest_get_section_count(
	const ia_css_param_terminal_manifest_t	*manifest);

/*! Get the (pointer to) indexed parameter manifest in the (param) terminal manifest object

 @param	manifest[in]			(param) terminal manifest object
 @param	section_index[in]		index of the parameter section manifest object

 @return parameter section manifest, NULL on error
 */
extern ia_css_parameter_manifest_t *ia_css_param_terminal_manifest_get_parameter_manifest(
	const ia_css_param_terminal_manifest_t	*manifest,
	const unsigned int						section_index);

/*! Get the supported frame types of the (data) terminal manifest object

 @param	manifest[in]			(data) terminal manifest object

 @return frame format bitmap, 0 on error
 */
extern ia_css_frame_format_bitmap_t ia_css_data_terminal_manifest_get_frame_format_bitmap(
	const ia_css_data_terminal_manifest_t	*manifest);

/*! Set the chosen frame type for the (data) terminal manifest object

 @param	manifest[in]			(data) terminal manifest object

 @return frame format bitmap, 0 on error
 */
extern int ia_css_data_terminal_manifest_set_frame_format_bitmap(
	ia_css_data_terminal_manifest_t	*manifest,
	ia_css_frame_format_bitmap_t bitmap);

/*! Get the connection bitmap of the (data) terminal manifest object

 @param	manifest[in]			(data) terminal manifest object

 @return connection bitmap, 0 on error
 */
extern ia_css_connection_bitmap_t ia_css_data_terminal_manifest_get_connection_bitmap(
	const ia_css_data_terminal_manifest_t	*manifest);

/*! Get the kernel dependency of the (data) terminal manifest object

 @param	manifest[in]			(data) terminal manifest object

 @return kernel bitmap, 0 on error
 */
extern ia_css_kernel_bitmap_t ia_css_data_terminal_manifest_get_kernel_bitmap(
	const ia_css_data_terminal_manifest_t	*manifest);

/*! Set the kernel dependency of the (data) terminal manifest object

 @param	manifest[in]			(data) terminal manifest object
 @param	kernel_bitmap[in]		kernel dependency bitmap

 @return < 0 on error
 */
extern int ia_css_data_terminal_manifest_set_kernel_bitmap(
	ia_css_data_terminal_manifest_t			*manifest,
	const ia_css_kernel_bitmap_t			kernel_bitmap);

/*! Set the unique kernel dependency of the (data) terminal manifest object

 @param	manifest[in]			(data) terminal manifest object
 @param	index[in]				kernel dependency bitmap index

 @return < 0 on error
 */
extern int ia_css_data_terminal_manifest_set_kernel_bitmap_unique(
	ia_css_data_terminal_manifest_t			*manifest,
	const unsigned int						index);

/*! Set the min size of the (data) terminal manifest object

 @param	manifest[in]			(data) terminal manifest object
 @param	min_size[in]			Minimum size of the frame array

 */
extern void ia_css_data_terminal_manifest_set_min_size(
	ia_css_data_terminal_manifest_t	*manifest,
	const uint16_t			min_size[IA_CSS_N_DATA_DIMENSION]);

/*! Set the max size of the (data) terminal manifest object

 @param	manifest[in]			(data) terminal manifest object
 @param	min_size[in]			Maximum size of the frame array

 */
extern void ia_css_data_terminal_manifest_set_max_size(
	ia_css_data_terminal_manifest_t	*manifest,
	const uint16_t			max_size[IA_CSS_N_DATA_DIMENSION]);

/*! Get the min size of the (data) terminal manifest object

 @param	manifest[in]			(data) terminal manifest object
 @param	min_size[in]			Minimum size of the frame array

 */
extern void ia_css_data_terminal_manifest_get_min_size(
	const ia_css_data_terminal_manifest_t	*manifest,
	uint16_t			min_size[IA_CSS_N_DATA_DIMENSION]);

/*! Get the max size of the (data) terminal manifest object

 @param	manifest[in]			(data) terminal manifest object
 @param	min_size[in]			Maximum size of the frame array

 */
extern void ia_css_data_terminal_manifest_get_max_size(
	const ia_css_data_terminal_manifest_t	*manifest,
	uint16_t			max_size[IA_CSS_N_DATA_DIMENSION]);


#endif /* __IA_CSS_PSYS_TERMINAL_MANIFEST_H_INCLUDED__ */
