/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-05
*
*/

#include "show_mutex.h"
#include "show_log.h"
struct mutex hptg_mutex;

/** @brief Allocate memory for mutex
*	
*	@param _mutex the mutex lock
*
*/
int _mutex_allocate(struct mutex **_mutex){
	int rc = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/*if(!_mutex){
		LOG_Handler(LOG_ERR, "%s: failed: invalid mutex %p\n", __func__, _mutex);
		return -EMUTEX; 
	}*/

	/* Allocate memory for mutex */
    /*    *_mutex = kzalloc(sizeof(*_mutex), GFP_KERNEL);
        if (!*_mutex) {
               LOG_Handler(LOG_ERR, "%s: failed: no memory to mutex %p", __func__, _mutex);
               return -ENOMEM;
        }*/
	*_mutex=&hptg_mutex;
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return rc;
}

/** @brief Initialize mutex
*	
*	@param _mutex the mutex lock
*
*/
int _mutex_init(struct mutex *_mutex){
	int rc = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	if(!_mutex){
		LOG_Handler(LOG_ERR, "%s: failed: invalid mutex %p\n", __func__, _mutex);
		return -EMUTEX; 
	}

	/* Initialize mutex */
	mutex_init(_mutex);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return rc;
}

/** @brief Lock mutex
*	
*	@param _mutex the mutex lock
*
*/
int __mutex_lock(struct mutex *_mutex){
	int rc = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	if(!_mutex){
		LOG_Handler(LOG_ERR, "%s: failed: invalid mutex %p\n", __func__, _mutex);
		return -EMUTEX; 
	}

	/* Lock mutex */
	mutex_lock(_mutex);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return rc;
}

/** @brief Try lock mutex
*	
*	@param _mutex the mutex lock
*
*/
int _mutex_trylock(struct mutex *_mutex){
	int rc = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	if(!_mutex){
		LOG_Handler(LOG_ERR, "%s: failed: invalid mutex %p\n", __func__, _mutex);
		return -EMUTEX; 
	}

	/* try lock mutex */
	mutex_trylock(_mutex);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return rc;
}

/** @brief Unlock mutex
*	
*	@param _mutex the mutex lock
*
*/
int _mutex_unlock(struct mutex *_mutex){
	int rc = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	if(!_mutex){
		LOG_Handler(LOG_ERR, "%s: failed: invalid mutex %p\n", __func__, _mutex);
		return -EMUTEX; 
	}

	/* Unlock mutex */
	mutex_unlock(_mutex);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return rc;
}

/** @brief Destroy mutex
*	
*	@param _mutex the mutex lock
*
*/
int _mutex_destroy(struct mutex *_mutex){
	int rc = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	if(!_mutex){
		LOG_Handler(LOG_ERR, "%s: failed: invalid mutex %p\n", __func__, _mutex);
		return -EMUTEX; 
	}

	/* Destroy mutex */
	mutex_destroy(_mutex);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return rc;
}

/** @brief Free memory for mutex
*	
*	@param _mutex the mutex lock
*
*/
int _mutex_free(struct mutex *_mutex){
	int rc = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/*if(!_mutex){
		LOG_Handler(LOG_ERR, "%s: failed: invalid mutex %p\n", __func__, _mutex);
		return -EMUTEX; 
	}*/
	
	kfree(_mutex);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return rc;
}

