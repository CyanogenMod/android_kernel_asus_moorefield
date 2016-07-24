#ifndef _IA_CSS_TPROXY_CLIENT_H_
#define _IA_CSS_TPROXY_CLIENT_H_

#include <type_support.h>
#include <storage_class.h>
#include "ia_css_tproxy_local.h"

#ifndef __INLINE_TPROXY__
#define STORAGE_CLASS_TPROXY_H STORAGE_CLASS_EXTERN
#define STORAGE_CLASS_TPROXY_C
#include "ia_css_tproxy_client_public.h"
#else
#define STORAGE_CLASS_TPROXY_H STORAGE_CLASS_INLINE
#define STORAGE_CLASS_TPROXY_C STORAGE_CLASS_INLINE
#include "ia_css_tproxy_client_public.h"
#include "ia_css_tproxy_client_private.h"
#endif

#endif /*_IA_CSS_TPROXY_CLIENT_H_*/
