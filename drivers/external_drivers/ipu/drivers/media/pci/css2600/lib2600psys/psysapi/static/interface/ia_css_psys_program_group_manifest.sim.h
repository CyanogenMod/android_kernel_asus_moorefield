#ifndef __IA_CSS_PSYS_PROGRAM_GROUP_MANIFEST_SIM_H_INCLUDED__
#define __IA_CSS_PSYS_PROGRAM_GROUP_MANIFEST_SIM_H_INCLUDED__

/*! \file */

/** @file ia_css_psys_program_group_manifest.sim.h
 *
 * Define the methods on the program group manifest object: Simulation only
 */

#include <ia_css_psys_manifest_types.h>

#include <type_support.h>					/* uint8_t */

/*! Create a program group manifest object from specification

 @param	specification[in]		specification (index)

 @return NULL on error
 */
extern ia_css_program_group_manifest_t *ia_css_program_group_manifest_create(
	const unsigned int						specification);

/*! Destroy the program group manifest object

 @param	manifest[in]			program group manifest

 @return NULL
 */
extern ia_css_program_group_manifest_t *ia_css_program_group_manifest_destroy(
	ia_css_program_group_manifest_t			*manifest);

/*! Compute the size of storage required for allocating the program group manifest object

 @param	program_count[in]				Number of programs in the program group
 @param	terminal_count[in]				Number of terminals on the program group
 @param	program_dependency_count[in]	Array[program_count] with the program dependencies
 @param	terminal_dependency_count[in]	Array[program_count] with the terminal dependencies
 @param	terminal_type[in]				Array[terminal_count] with the terminal type
 @param section_count[in]				section count.

 @return 0 on error
 */
extern size_t ia_css_sizeof_program_group_manifest(
	const uint8_t							program_count,
	const uint8_t							terminal_count,
	const uint8_t							*program_dependency_count,
	const uint8_t							*terminal_dependency_count,
	const ia_css_terminal_type_t					*terminal_type,
	const uint16_t							section_count);

/*! Create (the storage for) the program group manifest object

 @param	program_count[in]				Number of programs in the program group
 @param	terminal_count[in]				Number of terminals on the program group
 @param	program_dependency_count[in]	Array[program_count] with the program dependencies
 @param	terminal_dependency_count[in]	Array[program_count] with the terminal dependencies
 @param	terminal_type[in]				Array[terminal_count] with the terminal type

 @return NULL on error
 */
extern ia_css_program_group_manifest_t *ia_css_program_group_manifest_alloc(
	const uint8_t							program_count,
	const uint8_t							terminal_count,
	const uint8_t							*program_dependency_count,
	const uint8_t							*terminal_dependency_count,
	const ia_css_terminal_type_t			*terminal_type);

/*! Free (the storage of) the program group manifest object

 @param	manifest[in]			program group manifest

 @return NULL
 */
extern ia_css_program_group_manifest_t *ia_css_program_group_manifest_free(
	ia_css_program_group_manifest_t			*manifest);

#endif /* __IA_CSS_PSYS_PROGRAM_GROUP_MANIFEST_SIM_H_INCLUDED__ */



