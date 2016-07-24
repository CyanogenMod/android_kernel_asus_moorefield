#ifndef _HRT_EMBED_H_
#define _HRT_EMBED_H_

#define _hrt_cell_dummy_use_blob(prog) \
	HRTCAT(_hrt_dummy_use_blob_, prog)()
#define _hrt_program_transfer_func(prog) \
	HRTCAT(_hrt_transfer_embedded_, prog)
#define _hrt_program_blob(prog) \
	(HRTCAT(_hrt_blob_, prog).data)
#define hrt_embedded_program_size(prog) \
	HRTCAT(_hrt_size_of_, prog)
#define hrt_embedded_program_text_size(prog) \
	HRTCAT(_hrt_text_size_of_, prog)

#endif /* _HRT_EMBED_H_ */
