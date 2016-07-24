#ifndef __IA_CSS_SYSCOM_TYPES_H__
#define __IA_CSS_SYSCOM_TYPES_H__


/**
 * return values of css_sys API functions
 */
typedef enum {
	ia_css_sys_success,
	ia_css_sys_err_invalid_argument,
	ia_css_sys_err_already_initialized,
	ia_css_sys_err_not_initialized,
	ia_css_sys_err_not_idle,
	ia_css_sys_err_not_suspended,
	ia_css_sys_err_request_queue_full,
	ia_css_sys_err_resonse_queue_empty
} ia_css_subsystem_return_t;


/* Idle conditions */
typedef enum {
	ia_css_sys_idle_never,
	ia_css_sys_idle_on_empty_request_queue,
	ia_css_sys_idle_after_every_request
	ia_css_sys_idle_asap,
} ia_css_sys_idle_condition_t;

/*
 * Enumeration of generic response types
 */
typedef enum {
  ia_css_subsystem_response_idle,
  ia_css_subsystem_response_error,
} ia_css_subsystem_response_type_t;

/*
 * Enumeration of interrupts
 */
typedef enum {
  ia_css_subsystem_irq_response,
  ia_css_subsystem_irq_watchdog,
} ia_css_subsystem_irq_unkown


/**
  * Initialzation/configuration parameter struct
  * user parameters for init: public
  */
typedef struct {
	ia_css_subsystem_base_address_t	subsystem_base_address;
	ia_css_page_table_base_t		page_table_base_address; /* for all mmu's*/
	ia_css_firmware_t			firmware;
	unsigned int			num_request_queues;
	unsigned int			request_queue_size; // array?
	unsigned int			response_queue_size; // array?
	ia_css_subsystem_idle_condition_t	idle_condition;
	ia_css_irq_shape			irq_shape;	/* level or pulse */
} ia_css_subsystem_config_t;


/**
 * Request
 */
typedef struct {
  uint32_t	type;		/* type of request */
  uint32_t	id;		/* user's id of the request buffer, for response */
  unit32_t	buffer; 	/* subsystem's virtual address of request buffer */
  uint32_t	size;		/* size of the request buffer in bytes */
  uint32_t	_reserved4;	/* padding to align to one DMA/DDR word */
  uint32_t	_reserved5;
  uint32_t	_reserved6;
  uint32_t	_reserved7;
} ia_css_subsystem_request_t;


/**
 * Response
 *
typedef struct {
  uint32_t 	type;		/* type of response */
  uint32_t	id;		/* user's id of buffer, from the request */
  uint32_t      data0; 		/* e.g., the subsystem's virtual address of buffer */
  uint32_t	data1;  	/* additional response data */
  uint32_t	_reserved4;	/* padding to align to one DMA/DDR word */
  uint32_t	_reserved5;
  uint32_t	_reserved6;
  uint32_t	_reserved7;
} ia_css_subsystem_response_t;


#endif /*__IA_CSS_SYSCOM_TYPES_H__*/
