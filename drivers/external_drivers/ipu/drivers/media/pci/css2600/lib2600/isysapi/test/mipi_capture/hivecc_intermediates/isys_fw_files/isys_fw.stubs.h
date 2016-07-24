/* macros for passing parameters to and reading results from entry functions */

#define _hrt_pass_params_isys_fw()

#define _hrt_read_result_isys_fw() 0

/* isys_fw: check address known, default stub, hrt_start_isys_fw, hrt_end_isys_fw */

#ifndef HIVE_ADDR_isys_fw_entry
#error -----------------------------------------------------------------------------------------
#error Address for function isys_fw not defined, please include mapfile before including stubs file
#error -----------------------------------------------------------------------------------------
#endif

#define isys_fw() hrt_run_isys_fw()

#ifdef __GNUC__
HRT_INLINE void hrt_run_isys_fw(void) __attribute__((unused));
#endif

HRT_INLINE void hrt_run_isys_fw(void)
{
  _hrt_pass_params_isys_fw();
  hrt_cell_run_function(CELL, isys_fw);
}

#ifdef __GNUC__
HRT_INLINE void hrt_start_isys_fw(void) __attribute__((unused));
#endif

HRT_INLINE void hrt_start_isys_fw(void)
{
  _hrt_pass_params_isys_fw();
  hrt_cell_start_function(CELL, isys_fw);
}


