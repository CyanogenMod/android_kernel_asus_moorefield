#define TP2E_CRASH_DATA_LEN 10

static char trace_tp2e_crash_str[4 * (TP2E_CRASH_DATA_LEN + 1)] = {'\0',};

static int set_tp2e_crash(const char *val, const struct kernel_param *kp)
{
	char ev_name[TP2E_CRASH_DATA_LEN],
	  data0[TP2E_CRASH_DATA_LEN],
	  data1[TP2E_CRASH_DATA_LEN],
	  data2[TP2E_CRASH_DATA_LEN];
	int ret = -EINVAL;
	unsigned int add_steps;

	if (sscanf(val, "%s %s %s %s %d", ev_name, data0, data1, data2,
			&add_steps) != 5)
		return ret;

	memcpy(trace_tp2e_crash_str, val, strlen(val));

	trace_tp2e_generic_event(TP2E_EV_INFO, "tp2e_test", ev_name,
				 data0, data1, data2, "", "", "", "",add_steps);

	return 0;
}

static int get_tp2e_crash(char *buf,const struct kernel_param *kp)
{
	size_t len = strlen(trace_tp2e_crash_str);
	memcpy(buf, trace_tp2e_crash_str, len);
	return len;
}

static struct kernel_param_ops trigger_ops = {
	.set = set_tp2e_crash,
	.get = get_tp2e_crash,
};

module_param_cb(trace_tp2e_crash_str, &trigger_ops, NULL, 0644);
MODULE_PARM_DESC(trace_tp2e_crash_str, "log trace tp2e "
		"crash <ev_name> <data0> <data1> <data2> <additional steps>");
