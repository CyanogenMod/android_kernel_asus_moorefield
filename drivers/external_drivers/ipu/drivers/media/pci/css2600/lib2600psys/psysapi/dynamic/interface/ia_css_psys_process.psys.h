#ifndef __IA_CSS_PSYS_PROCESS_PSYS_H_INCLUDED__
#define __IA_CSS_PSYS_PROCESS_PSYS_H_INCLUDED__

/*! \file */

/** @file ia_css_psys_process.psys.h
 *
 * Define the methods on the process object: Psys embedded interface
 */

#include <ia_css_psys_process_types.h>

/*
 * Process manager
 */

/*! Acquire the resources specificed in process object

 @param	process[in]				process object

 Postcondition: This is a try process if any of the
 resources is not available, all succesfully acquired
 ones will be release and the function will return an
 error

 @return < 0 on error
 */
extern int ia_css_process_acquire(
	ia_css_process_t						*process);

/*! Release the resources specificed in process object

 @param	process[in]				process object

 @return < 0 on error
 */
extern int ia_css_process_release(
	ia_css_process_t						*process);


#endif /* __IA_CSS_PSYS_PROCESS_PSYS_H_INCLUDED__ */
