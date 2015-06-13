#ifndef __INTEL_BYT_BUTTONS_H_
#define __INTEL_BYT_BUTTONS_H_

struct byt_keys_button {
	unsigned int code;	/* input event code */
	unsigned int type;	/* input event type */
	const char *desc;
	int active_low;
	int wakeup;
};

struct byt_keys_platform_data {
	struct byt_keys_button *buttons;
	int nbuttons;
	unsigned int rep:1;	/* enable/disable auto repeat */
};

#endif

