/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-05
*
*/

#ifndef __LINUX_SHOW_SENSOR_TIMER_H
#define __LINUX_SHOW_SENSOR_TIMER_H

#include <linux/timer.h>
#include <linux/types.h>

/* Display current time */
struct timeval get_current_time(void);
void O_get_current_time(struct timeval* now);
/* Check if timeout happen */
bool is_timeout(struct timeval start, struct timeval now, int timeout);
#if 0
bool is_timeout(struct timeval start, int timeout, char* reason);
#endif
#endif
