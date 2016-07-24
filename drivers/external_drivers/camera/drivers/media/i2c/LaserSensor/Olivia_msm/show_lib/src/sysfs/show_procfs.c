/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-05
*
*/

#include "show_procfs.h"
#include "show_log.h"

/** @brief Create proc file
*	
*	@param file_name the file name which you want to create
*	@param mode the access right for owner of the file (e.g. 0776)
*	@param parent the proc entry
*	@param proc_fops the pointer which points to file_operations struct. It will be a entry point of system call after the driver register it for kernel. 
*
*/
int create_proc_file(const char *file_name, umode_t mode, struct proc_dir_entry *parent, const struct file_operations *proc_fops){

	int status = 1;
	struct proc_dir_entry *proc_file;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Create proc file */
	proc_file = proc_create(file_name, mode, parent, proc_fops);

	if (proc_file) {
              LOG_Handler(LOG_DBG, "%s %s sucessed!\n", __func__, file_name);
        } else {
              LOG_Handler(LOG_ERR, "%s %s failed!\n", __func__, file_name);
		status = -PROC_ERR;
        }

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return status;
}

