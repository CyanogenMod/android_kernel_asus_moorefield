#ifndef _isys_fw_transfer_h_
#define _isys_fw_transfer_h_

#define _hrt_dummy_use_blob_isys_fw()\
    { char *blob = _hrt_blob_isys_fw.data; blob = blob; }
#define _hrt_transfer_embedded_isys_fw(code_func, data_func, bss_func, view_table_func, icache_master_func, args...) \
  {\
      code_func(icache, 0x0, (0), 0xBBE4, ## args);  icache_master_func(icache, 0, 0, ## args);\
      data_func(sp2600_control_dmem, 0x8, (0+ 48100), 0x19BC, ## args);\
      bss_func(sp2600_control_dmem, 0x19C8, 0xA5E8, ## args);\
  \
  }

#define _hrt_size_of_isys_fw ( 0+ 48100+ 6588 )
#define _hrt_text_is_p2
#define _hrt_section_size_isys_fw_icache 0xBBE4
/* properties of section .icache: */
#define _hrt_icache_instructions_of_isys_fw 0x00002EF9
#define _hrt_icache_instruction_size_of_isys_fw 0x00000004
#define _hrt_icache_instruction_aligned_size_of_isys_fw 0x00000004
#define _hrt_icache_target_of_isys_fw 0x00000000
#define _hrt_icache_source_of_isys_fw (0)
#define _hrt_icache_size_of_isys_fw 0x0000BBE4

/* properties of section .data.dmem: */
#define _hrt_data_target_of_isys_fw 0x00000008
#define _hrt_data_source_of_isys_fw (0+ 48100)
#define _hrt_data_size_of_isys_fw 0x000019BC

/* properties of section .bss.dmem: */
#define _hrt_bss_target_of_isys_fw 0x000019C8
#define _hrt_bss_size_of_isys_fw 0x0000A5E8

#define _hrt_text_instructions_of_isys_fw 0x00000000
#define _hrt_text_instruction_size_of_isys_fw 0x00000000
#define _hrt_text_instruction_aligned_size_of_isys_fw 0x00000000
#define _hrt_text_target_of_isys_fw 0x00000000
#define _hrt_text_source_of_isys_fw 0x00000000
#define _hrt_text_size_of_isys_fw 0x00000000



#endif /* _isys_fw_transfer_h_ */
