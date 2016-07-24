#ifndef _devproxy_transfer_h_
#define _devproxy_transfer_h_

#define _hrt_dummy_use_blob_devproxy()\
    { char *blob = _hrt_blob_devproxy.data; blob = blob; }
#define _hrt_transfer_embedded_devproxy(code_func, data_func, bss_func, view_table_func, icache_master_func, args...) \
  {\
      code_func(icache, 0x0, (0), 0x4B0C, ## args);  icache_master_func(icache, 0, 0, ## args);\
      data_func(sp2600_proxy_dmem, 0x6F8, (0+ 19212), 0x490, ## args);\
      bss_func(sp2600_proxy_dmem, 0xB88, 0x78, ## args);\
  \
  }

#define _hrt_size_of_devproxy ( 0+ 19212+ 1168 )
#define _hrt_text_is_p2
#define _hrt_section_size_devproxy_icache 0x4B0C
/* properties of section .icache: */
#define _hrt_icache_instructions_of_devproxy 0x000012C3
#define _hrt_icache_instruction_size_of_devproxy 0x00000004
#define _hrt_icache_instruction_aligned_size_of_devproxy 0x00000004
#define _hrt_icache_target_of_devproxy 0x00000000
#define _hrt_icache_source_of_devproxy (0)
#define _hrt_icache_size_of_devproxy 0x00004B0C

/* properties of section .data.dmem: */
#define _hrt_data_target_of_devproxy 0x000006F8
#define _hrt_data_source_of_devproxy (0+ 19212)
#define _hrt_data_size_of_devproxy 0x00000490

/* properties of section .bss.dmem: */
#define _hrt_bss_target_of_devproxy 0x00000B88
#define _hrt_bss_size_of_devproxy 0x00000078

#define _hrt_text_instructions_of_devproxy 0x00000000
#define _hrt_text_instruction_size_of_devproxy 0x00000000
#define _hrt_text_instruction_aligned_size_of_devproxy 0x00000000
#define _hrt_text_target_of_devproxy 0x00000000
#define _hrt_text_source_of_devproxy 0x00000000
#define _hrt_text_size_of_devproxy 0x00000000



#endif /* _devproxy_transfer_h_ */
