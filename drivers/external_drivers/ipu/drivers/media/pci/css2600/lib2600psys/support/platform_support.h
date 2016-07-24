#ifndef __PLATFORM_SUPPORT_H_INCLUDED__
#define __PLATFORM_SUPPORT_H_INCLUDED__

#if defined(_MSC_VER)
#define CSS_ALIGN(d, a) _declspec(align(a)) d

#elif defined(__HIVECC)
#define CSS_ALIGN(d, a) d __attribute__((aligned(a)))
#elif defined(__KERNEL__)
#define CSS_ALIGN(d, a) d __attribute__((aligned(a)))

#elif defined(__GNUC__)
#define CSS_ALIGN(d, a) d __attribute__((aligned(a)))
#else
#endif

#include "type_support.h"	/*needed for the include in stdint.h for various environments */
#include "storage_class.h"

#define MAX_ALIGNMENT						8
#define aligned_uint8(type,obj)				CSS_ALIGN(uint8_t obj,1)
#define aligned_int8(type,obj)				CSS_ALIGN(int8_t obj,1)
#define aligned_uint16(type,obj)			CSS_ALIGN(uint16_t obj,2)
#define aligned_int16(type,obj)				CSS_ALIGN(int16_t obj,2)
#define aligned_uint32(type,obj)			CSS_ALIGN(uint32_t obj,4)
#define aligned_int32(type,obj)				CSS_ALIGN(int32_t obj,4)

#if defined(__HIVECC)		/* needed as long as hivecc does not define the type (u)int64_t */
#define aligned_uint64(type,obj)			CSS_ALIGN(unsigned long long obj,8)
#define aligned_int64(type,obj)				CSS_ALIGN(signed long long obj,8)
#else
#define aligned_uint64(type,obj)			CSS_ALIGN(uint64_t obj,8)
#define aligned_int64(type,obj)				CSS_ALIGN(int64_t obj,8)
#endif
#define aligned_enum(enum_type,obj)			CSS_ALIGN(uint32_t obj,4)
//#define aligned_struct(struct_type,obj)		CSS_ALIGN(struct_type obj,MAX_ALIGNMENT)
#define aligned_struct(struct_type,obj)		struct_type obj

#endif /* __PLATFORM_SUPPORT_H_INCLUDED__ */
