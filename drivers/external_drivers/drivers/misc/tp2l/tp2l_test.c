#define CREATE_TRACE_POINTS
#include "tp2l_test_trace.h"
#undef CREATE_TRACE_POINTS

static int silly_count;
static int crazy_count;

static int set_tp2l_test(const char *buf, struct kernel_param *kp)
{
	char event_name[32];

	if (sscanf(buf, "%s", event_name) != 1)
		return -EINVAL;

	if (strncmp("silly",
		    event_name, strlen("silly")) == 0) {
		silly_count++;
		trace_silly(jiffies, silly_count);
	} else if (strncmp("crazy1",
			   event_name, strlen("crazy1")) == 0) {
		crazy_count++;
		trace_crazy1(jiffies, crazy_count);
	} else if (strncmp("crazy2",
			   event_name, strlen("crazy2")) == 0) {
		crazy_count++;
		trace_crazy2(jiffies, crazy_count);
	}
	return 0;
}

static int get_tp2l_test(char *buf, struct kernel_param *kp)
{
	return sprintf(buf, "silly_count: %i\ncrazy_count: %i",
		       silly_count, crazy_count);
}

module_param_call(trace_tp2l_test, set_tp2l_test, get_tp2l_test, NULL, 0644);
MODULE_PARM_DESC(trace_tp2l_test, "log trace msg for tp2l_test_silly event");
