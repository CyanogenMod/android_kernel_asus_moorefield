/*
* This file present the generic interface to services
*/

#ifndef _DEVPROXY_SERVICE_H_
#define _DEVPROXY_SERVICE_H_

#include "type_support.h" /* for size_t */
#include "vied_nci_eq_device.h"
#include "vied_nci_acc.h"


/*---------------------------------------------------------------------------*/
/* forward declaration */
typedef struct devproxy_service* devproxy_service_instance_t;

/*---------------------------------------------------------------------------*/
/* these are the generic functions offered by all services */

typedef void (*devproxy_service_ctrl_init_t)(
	devproxy_service_instance_t instance, /* [in] the service to address */
	void* state,  /* [in] to what state shall be initialized */
	size_t size   /* [in] size of the state buffer */
);

typedef void (*devproxy_service_ctrl_next_t)(
	devproxy_service_instance_t instance /* [in] the service to address */
);

typedef void (*devproxy_service_ctrl_done_t)(
	devproxy_service_instance_t instance, /* [in] the service to address */
	void* state,  /* [out] where to store the end state */
	size_t size   /* [in] size of the state buffer */
);

typedef void (*devproxy_service_handle_device_event_t)(
	devproxy_service_instance_t instance ,/* [in] the service to address */
	vied_nci_eq_token_t device_event
);


/*---------------------------------------------------------------------------*/
/* the states of the device controlled by a service */
typedef enum {
	S0_IDLE,         /* waiting for the "init" command */
	S1_INITIALIZING, /* waiting for the device */
	S2_READY,        /* no device operation is pending */
	S3_RUNNING,      /* a device operation is ongoing but the device is ready for more commands */
	S4_QUEUE_FULL,   /* the device can take no more commands */
	S_MAX
} state_t;

/*---------------------------------------------------------------------------*/
/* the generic constant configuration of a service */
typedef struct devproxy_service_configuration {
	unsigned int          service_pid;   /* the pid that this service responds to */
	unsigned int          device_handle; /* the device controlled by this service */
	unsigned int          device_sid;    /* the device controlled by this service */
	unsigned int          nof_config_sets;
} devproxy_service_configuration_t;

/*---------------------------------------------------------------------------*/
/* the generic run time configuration of service */
typedef struct devproxy_service_runtime_configuration {
	/* run-time configuration provided via the parameters of the "init" function */
	vied_nci_eq_token_t   on_done_token; /* the token to be sent after "done" has finished*/
	vied_device_id		  on_done_eq;	 /* the event queue to send the above token */
} devproxy_service_runtime_configuration_t;

/*---------------------------------------------------------------------------*/
/* the generic vtbl function pointers of a service */
typedef struct devproxy_service_functions {
	devproxy_service_ctrl_init_t           init;
	devproxy_service_ctrl_next_t           next;
	devproxy_service_ctrl_done_t           done;
	devproxy_service_handle_device_event_t handle_device_event;
} devproxy_service_functions_t;

/*---------------------------------------------------------------------------*/
/* the context of a service */
typedef struct devproxy_service {
	/* configuration (set by the client)*/
	devproxy_service_configuration_t config;

	/* vtbl (set by the client)*/
	devproxy_service_functions_t     func;

	/* */
	devproxy_service_runtime_configuration_t runtime_config;

	/* operational state (not to be touched by the client) */
	state_t               state;        /* the current state of the state machine */
	unsigned int          nof_config_sets_in_use;
	unsigned int          next_config_set;
	unsigned int          nof_pending_next_events; /* typically 0 or few pending */
	unsigned int          nof_pending_done_events; /* typically 0 or 1 pending */
	void*                 pending_done_event_state_arg;
	size_t                pending_done_event_size_arg;
} devproxy_service_t;

typedef void (*devproxy_service_func_ptr_t)(
	void *state
);

struct devproxy_service_func {
	unsigned int id; /* pid/msg combination */
};


/*---------------------------------------------------------------------------*/

#endif //_DEVPROXY_SERVICE_H_
