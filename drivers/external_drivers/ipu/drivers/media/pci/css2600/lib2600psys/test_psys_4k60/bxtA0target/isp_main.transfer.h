#ifndef _isp_main_transfer_h_
#define _isp_main_transfer_h_

#define _hrt_dummy_use_blob_isp_main()\
    { char *blob = _hrt_blob_isp_main.data; blob = blob; }
#define _hrt_transfer_embedded_isp_main(code_func, data_func, bss_func, view_table_func, icache_master_func, args...) \
  {\
      code_func(icache, 0x0, (0), 0x78680, ## args);  icache_master_func(icache, 0, 0, ## args);\
      data_func(isp2600_base_dmem, 0x0, (0+ 493184), 0x61C, ## args);\
      bss_func(isp2600_base_dmem, 0x620, 0x73C, ## args);\
  \
  }

#define _hrt_size_of_isp_main ( 0+ 493184+ 1564 )
#define _hrt_text_is_p2
#define _hrt_section_size_isp_main_icache 0x78680
/* properties of section .icache: */
#define _hrt_icache_instructions_of_isp_main 0x00001E1A
#define _hrt_icache_instruction_size_of_isp_main 0x00000040
#define _hrt_icache_instruction_aligned_size_of_isp_main 0x00000040
#define _hrt_icache_target_of_isp_main 0x00200000
#define _hrt_icache_source_of_isp_main (0)
#define _hrt_icache_size_of_isp_main 0x00078680

/* properties of section .data.base_dmem: */
#define _hrt_data_target_of_isp_main 0x00000000
#define _hrt_data_source_of_isp_main (0+ 493184)
#define _hrt_data_size_of_isp_main 0x0000061C

/* properties of section .bss.base_dmem: */
#define _hrt_bss_target_of_isp_main 0x00000620
#define _hrt_bss_size_of_isp_main 0x0000073C

#define _hrt_text_instructions_of_isp_main 0x00000000
#define _hrt_text_instruction_size_of_isp_main 0x00000000
#define _hrt_text_instruction_aligned_size_of_isp_main 0x00000000
#define _hrt_text_target_of_isp_main 0x00000000
#define _hrt_text_source_of_isp_main 0x00000000
#define _hrt_text_size_of_isp_main 0x00000000



#endif /* _isp_main_transfer_h_ */
