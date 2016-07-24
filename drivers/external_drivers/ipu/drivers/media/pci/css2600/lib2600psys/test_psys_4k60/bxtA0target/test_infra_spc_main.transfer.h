#ifndef _test_infra_spc_main_transfer_h_
#define _test_infra_spc_main_transfer_h_

#define _hrt_dummy_use_blob_test_infra_spc_main()\
    { char *blob = _hrt_blob_test_infra_spc_main.data; blob = blob; }
#define _hrt_transfer_embedded_test_infra_spc_main(code_func, data_func, bss_func, view_table_func, icache_master_func, args...) \
  {\
      code_func(icache, 0x0, (0), 0xB84, ## args);  icache_master_func(icache, 0, 0, ## args);\
      data_func(sp2600_control_dmem, 0x0, (0+ 2948), 0x54, ## args);\
      bss_func(sp2600_control_dmem, 0x54, 0x4, ## args);\
  \
  }

#define _hrt_size_of_test_infra_spc_main ( 0+ 2948+ 84 )
#define _hrt_text_is_p2
#define _hrt_section_size_test_infra_spc_main_icache 0xB84
/* properties of section .icache: */
#define _hrt_icache_instructions_of_test_infra_spc_main 0x000002E1
#define _hrt_icache_instruction_size_of_test_infra_spc_main 0x00000004
#define _hrt_icache_instruction_aligned_size_of_test_infra_spc_main 0x00000004
#define _hrt_icache_target_of_test_infra_spc_main 0x00000000
#define _hrt_icache_source_of_test_infra_spc_main (0)
#define _hrt_icache_size_of_test_infra_spc_main 0x00000B84

/* properties of section .data.dmem: */
#define _hrt_data_target_of_test_infra_spc_main 0x00000000
#define _hrt_data_source_of_test_infra_spc_main (0+ 2948)
#define _hrt_data_size_of_test_infra_spc_main 0x00000054

/* properties of section .bss.dmem: */
#define _hrt_bss_target_of_test_infra_spc_main 0x00000054
#define _hrt_bss_size_of_test_infra_spc_main 0x00000004

#define _hrt_text_instructions_of_test_infra_spc_main 0x00000000
#define _hrt_text_instruction_size_of_test_infra_spc_main 0x00000000
#define _hrt_text_instruction_aligned_size_of_test_infra_spc_main 0x00000000
#define _hrt_text_target_of_test_infra_spc_main 0x00000000
#define _hrt_text_source_of_test_infra_spc_main 0x00000000
#define _hrt_text_size_of_test_infra_spc_main 0x00000000



#endif /* _test_infra_spc_main_transfer_h_ */
