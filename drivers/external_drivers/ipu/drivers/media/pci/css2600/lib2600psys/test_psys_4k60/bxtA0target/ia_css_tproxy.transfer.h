#ifndef _ia_css_tproxy_transfer_h_
#define _ia_css_tproxy_transfer_h_

#define _hrt_dummy_use_blob_ia_css_tproxy()\
    { char *blob = _hrt_blob_ia_css_tproxy.data; blob = blob; }
#define _hrt_transfer_embedded_ia_css_tproxy(code_func, data_func, bss_func, view_table_func, icache_master_func, args...) \
  {\
      code_func(icache, 0x0, (0), 0x5F8C, ## args);  icache_master_func(icache, 0, 0, ## args);\
      data_func(sp2600_proxy_dmem, 0xB08, (0+ 24460), 0x71C, ## args);\
      bss_func(sp2600_proxy_dmem, 0x1228, 0x2580, ## args);\
  \
  }

#define _hrt_size_of_ia_css_tproxy ( 0+ 24460+ 1820 )
#define _hrt_text_is_p2
#define _hrt_section_size_ia_css_tproxy_icache 0x5F8C
/* properties of section .icache: */
#define _hrt_icache_instructions_of_ia_css_tproxy 0x000017E3
#define _hrt_icache_instruction_size_of_ia_css_tproxy 0x00000004
#define _hrt_icache_instruction_aligned_size_of_ia_css_tproxy 0x00000004
#define _hrt_icache_target_of_ia_css_tproxy 0x00000000
#define _hrt_icache_source_of_ia_css_tproxy (0)
#define _hrt_icache_size_of_ia_css_tproxy 0x00005F8C

/* properties of section .data.dmem: */
#define _hrt_data_target_of_ia_css_tproxy 0x00000B08
#define _hrt_data_source_of_ia_css_tproxy (0+ 24460)
#define _hrt_data_size_of_ia_css_tproxy 0x0000071C

/* properties of section .bss.dmem: */
#define _hrt_bss_target_of_ia_css_tproxy 0x00001228
#define _hrt_bss_size_of_ia_css_tproxy 0x00002580

#define _hrt_text_instructions_of_ia_css_tproxy 0x00000000
#define _hrt_text_instruction_size_of_ia_css_tproxy 0x00000000
#define _hrt_text_instruction_aligned_size_of_ia_css_tproxy 0x00000000
#define _hrt_text_target_of_ia_css_tproxy 0x00000000
#define _hrt_text_source_of_ia_css_tproxy 0x00000000
#define _hrt_text_size_of_ia_css_tproxy 0x00000000



#endif /* _ia_css_tproxy_transfer_h_ */
