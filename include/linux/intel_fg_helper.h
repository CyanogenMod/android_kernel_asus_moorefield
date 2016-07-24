#ifndef __INTEL_FG_HELPER_H_
#define __INTEL_FG_HELPER_H_

int intel_fg_get_data(void *data, int len);
int intel_fg_set_data(void *data, int len);
int intel_fg_set_restore_fn(int (*restore_fn)(void *data, int len));
int intel_fg_set_store_fn(int (*store_fn)(void *data, int len));

#endif
