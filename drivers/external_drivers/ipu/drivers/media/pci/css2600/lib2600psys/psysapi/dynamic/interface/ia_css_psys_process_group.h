#ifndef __IA_CSS_PSYS_PROCESS_GROUP_H_INCLUDED__
#define __IA_CSS_PSYS_PROCESS_GROUP_H_INCLUDED__

/*! \file */

/** @file ia_css_psys_process_group.h
 *
 * Define the methods on the process object that are not part of a single interface
 */

#include <ia_css_psys_process_types.h>

#include <type_support.h>					/* uint8_t */

/*
 * Creation
 */
#include <ia_css_psys_process_group.hsys.user.h>

/*
 * Registration of user contexts / callback info
 * External resources
 * Sequencing resources
 */
#include <ia_css_psys_process_group.hsys.kernel.h>

/*
 * Dispatcher
 */
#include <ia_css_psys_process_group.psys.h>

/*
 * Access to sub-structure handles / fields
 */

/*! Get the number of fragments on the process group

 @param	process_group[in]		process group object

 Note: Future change is to have a fragment count per
 independent subgraph

 @return the fragment count, 0 on error
 */
extern uint16_t ia_css_process_group_get_fragment_count(
	const ia_css_process_group_t			*process_group);

/*! Get the number of processes on the process group

 @param	process_group[in]		process group object

 @return the process count, 0 on error
 */
extern uint8_t ia_css_process_group_get_process_count(
	const ia_css_process_group_t			*process_group);

/*! Get the number of terminals on the process group

 @param	process_group[in]		process group object

 Note: Future change is to have a terminal count per
 independent subgraph

 @return the terminal count, 0 on error
 */
extern uint8_t ia_css_process_group_get_terminal_count(
	const ia_css_process_group_t			*process_group);


/*! Get the (pointer to) the indexed terminal of the process group object

 @param	process_group[in]		process group object
 @param	terminal_index[in]		index of the terminal

 @return the pointer to the terminal, NULL on error
 */
extern ia_css_terminal_t *ia_css_process_group_get_terminal(
	const ia_css_process_group_t			*process_group,
	const unsigned int						terminal_index);

/*! Get the (pointer to) the indexed process of the process group object

 @param	process_group[in]		process group object
 @param	process_index[in]		index of the process

 @return the pointer to the process, NULL on error
 */
extern ia_css_process_t *ia_css_process_group_get_process(
	const ia_css_process_group_t			*process_group,
	const unsigned int						process_index);

/*! Get the stored size of the process group object

 @param	process_group[in]				process group object

 @return size, 0 on error
 */
extern size_t ia_css_process_group_get_size(
	const ia_css_process_group_t			*process_group);

/*! Get the state of the the process group object

 @param	process_group[in]		process group object

 @return state, limit value on error
 */
extern ia_css_process_group_state_t ia_css_process_group_get_state(
	const ia_css_process_group_t			*process_group);

/*! Get the unique ID of program group used by the process group object

 @param	process_group[in]		process group object

 @return ID, 0 on error
 */
extern ia_css_program_group_ID_t ia_css_process_group_get_program_group_ID(
	const ia_css_process_group_t			*process_group);

/*! Get the resource bitmap of the process group

 @param	process_group[in]		process group object

 @return the reource bitmap
 */
extern vied_nci_resource_bitmap_t ia_css_process_group_get_resource_bitmap(
	const ia_css_process_group_t			*process_group);

/*! Set the resource bitmap of the process group

 @param	process_group[in]		process group object
 @param	resource_bitmap[in]		the resource bitmap

 @return < 0 on error
 */
extern int ia_css_process_group_set_resource_bitmap(
	ia_css_process_group_t					*process_group,
	const vied_nci_resource_bitmap_t		resource_bitmap);

#endif /* __IA_CSS_PSYS_PROCESS_GROUP_H_INCLUDED__ */
