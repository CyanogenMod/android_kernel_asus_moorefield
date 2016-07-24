/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-05
*
*/

#ifndef __LINUX_SHOW_SENSOR_MUTEX_H
#define __LINUX_SHOW_SENSOR_MUTEX_H

#include <linux/slab.h>

struct mutex;

/* Mutex error status */
#define EMUTEX 1;	/* Mutex is NULL*/

/* Mutex controller */
enum mutex_ctrl {
	MUTEX_ALLOCATE,
	MUTEX_INIT,
	MUTEX_LOCK,
	MUTEX_TRYLOCK,
	MUTEX_UNLOCK,
	MUTEX_DESTROY,
};

/* Allocate memroy for mutex*/
int _mutex_allocate(struct mutex **_mutex);
/* Initialize mutex */
int _mutex_init(struct mutex *_mutex);
/* Lock mutex */
int __mutex_lock(struct mutex *_mutex);
/* Try lock mutex */
int _mutex_trylock(struct mutex *_mutex);
/* Unlock mutex */
int _mutex_unlock(struct mutex *_mutex);
/* Destroy mutex */
int _mutex_destroy(struct mutex *_mutex);
/* Free memory for mutex */
int _mutex_free(struct mutex *_mutex);

#endif
