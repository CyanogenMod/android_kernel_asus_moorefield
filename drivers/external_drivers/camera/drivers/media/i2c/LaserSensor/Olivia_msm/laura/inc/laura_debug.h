/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-07
*
*/

#ifndef __LINUX_SHOW_LAURA_SENSOR_DEBUG_H
#define __LINUX_SHOW_LAURA_SENSOR_DEBUG_H

#include <linux/types.h>
#include "show_sysfs.h"
#include "show_debug.h"
#include "laser_focus_hepta.h"

/* The switch if record the debug information */
#ifndef NON_RECORD_DEBUG_INFO
#define NON_RECORD_DEBUG_INFO false
#endif
#ifndef RECORD_DEBUG_INFO
#define RECORD_DEBUG_INFO true
#endif

/* The dump register range  */
#ifndef DUMP_REGISTER_RANGE_MIN
#define DUMP_REGISTER_RANGE_MIN 0x000
#endif
#ifndef DUMP_REGISTER_RANGE_MAX
#define DUMP_REGISTER_RANGE_MAX 0x100
#endif

/* laura read range debug */
int laura_debug_register_dump(struct seq_file *vfile, int DMax, int errorStatus, bool record_in_file);
/* laura debug */
int laura_debug_dump(struct seq_file *vfile, void *v);
/* vl6180x dump register value */
int dump_laura_register_read(struct seq_file *vfile, void *v);

#endif
