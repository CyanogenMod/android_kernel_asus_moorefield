/*
*
*	Author:	Jheng-Siou, Cai
*	Time:	2015-06
*
*/

#include "show_verify.h"
#include "show_log.h"

/** @brief Verify whether or not the range is legal
*	
*	@param RawRange the range detected
*	@param ctrl the contorller which spec will be used
*
*/
bool Violation_Calibration_Spec(int RawRange, int ctrl){
        int cal_spec = 0, cal_spec_high = 0, cal_spec_low = 0;
        bool cal_spec_flag = false;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

        switch(ctrl){
		  /* Offset spec */
                case LASER_FOCUS_OFFSET_SPEC:
			   /* Get spec */
                        cal_spec = Sysfs_read_int(OFFSET_SPEC_FILE, 6);
			   /* Get upper bound */
                        cal_spec_high =  DEVICE_OFFSET_CAL_RANGE*10 + ((DEVICE_OFFSET_CAL_RANGE*cal_spec)/100);
			   /* Get lower bound */
                        cal_spec_low = DEVICE_OFFSET_CAL_RANGE*10 - ((DEVICE_OFFSET_CAL_RANGE*cal_spec)/100);
                        break;
		  /* Cross talk spec */
                case LASER_FOCUS_CROSS_TALK_SPEC:
			   /* Get spec */
			   cal_spec = Sysfs_read_int(CROSS_TALK_SPEC_FILE,6);
			   /* Get upper bound */
                        cal_spec_high =  DEVICE_CROSSTALK_CAL_RANGE*10 + ((DEVICE_CROSSTALK_CAL_RANGE*cal_spec)/100);
			   /* Get lower bound */
                        cal_spec_low = DEVICE_CROSSTALK_CAL_RANGE*10 - ((DEVICE_CROSSTALK_CAL_RANGE*cal_spec)/100);
                        break;
                default:
                        LOG_Handler(LOG_ERR, "%s: command fail(%d) !!\n", __func__, ctrl);
                        return false;
        }

	/* No spec or spec is 0 */
        if(cal_spec == 0){
                cal_spec_flag = false;
        }
        else{
		  /* Check whether or not the range is violation  */
                cal_spec_flag =  (((cal_spec_high < (RawRange*10)) || (cal_spec_low > (RawRange*10))));
                if(cal_spec_flag){
                        LOG_Handler(LOG_ERR, "%s: calibration value out of spec\n", __func__);
                }
        }

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

        return cal_spec_flag;
}
