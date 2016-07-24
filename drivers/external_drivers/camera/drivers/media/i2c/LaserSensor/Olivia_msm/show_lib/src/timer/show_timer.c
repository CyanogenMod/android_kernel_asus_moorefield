/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-05
*
*/

#include "show_timer.h"
#include "show_log.h"

/** @brief Display current time
*	
*/
void O_get_current_time(struct timeval* now){
	do_gettimeofday(now);
}
struct timeval get_current_time(void){
	struct timeval now;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	do_gettimeofday(&now);

	/*
		tv_sec: second
		tv_usec: microseconds 
	*/
	//LOG_Handler(LOG_DBG, "%s: Current UTC: %lu (%lu)\n", __func__, now.tv_sec, now.tv_usec);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return now;
}

/** @brief Check if timeout happen (ms) 
*	
*	@param start the start time
*	@param now the current time
*	@param timeout the timeout value(ms)
*
*/
bool is_timeout(struct timeval start, struct timeval now, int timeout){

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
 
       if((((now.tv_sec*1000000)+now.tv_usec)-((start.tv_sec*1000000)+start.tv_usec)) > (timeout*1000)){
              LOG_Handler(LOG_ERR, "%s: Timeout!!\n", __func__);
        	return true;
       }

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return false;
}

#if 0
/** @brief Check if timeout happen (ms) 
*	
*	@param start the start time
*	@param now the current time
*	@param timeout the timeout value(ms)
*
*/
bool is_timeout(struct timeval start, int timeout, char* reason){
	struct timeval now;
	now = get_current_time();

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
 
       if((((now.tv_sec*1000000)+now.tv_usec)-((start.tv_sec*1000000)+start.tv_usec)) > (timeout*1000)){
              LOG_Handler(LOG_ERR, "%s: Timeout(%d): %s!!\n", __func__, timeout, reason);
        	return true;
       }

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return false;
}
#endif