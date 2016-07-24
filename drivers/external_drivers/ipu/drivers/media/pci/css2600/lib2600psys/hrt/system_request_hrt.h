#ifndef _system_request_hrt_h_
#define _system_request_hrt_h_

/* The kinds of system requests */
typedef enum {
  /* print current collected statistics and reset counters  */
  hrt_system_request_kind_print_statistics,
} hrt_system_request_kind;

/* Each request kind has a corresponding type */

/* type corressponding to the print_statistics request kind */
typedef struct {
  const char *filename;
} hrt_system_request_print_statistics_t, *hrt_system_request_print_statistics;

/* The request type. It can hold each of the types corresponding to a
   request kind. */
typedef struct {
  hrt_system_request_kind kind;
  union {
    hrt_system_request_print_statistics_t print_statistics;
  } sub;
} hrt_system_request_t, *hrt_system_request;

/* functions to prepare a system request */
void hrt_system_print_statistics_request(const char *filename, hrt_system_request request);

#endif // _system_request_hrt_h_
