#ifndef _TOOLS_LE_BYTESHIFT_H
#define _TOOLS_LE_BYTESHIFT_H

#if defined(linux)
#include <linux/types.h>
#else
#include <stdint.h>
typedef int8_t   __s8;
typedef uint8_t  __u8;
typedef int16_t  __s16;
typedef uint16_t __u16;
typedef int32_t  __s32;
typedef uint32_t __u32;
typedef int64_t  __s64;
typedef uint64_t __u64;
#endif

static inline __u16 __get_unaligned_le16(const __u8 *p)
{
	return p[0] | p[1] << 8;
}

static inline __u32 __get_unaligned_le32(const __u8 *p)
{
	return p[0] | p[1] << 8 | p[2] << 16 | p[3] << 24;
}

static inline __u64 __get_unaligned_le64(const __u8 *p)
{
	return (__u64)__get_unaligned_le32(p + 4) << 32 |
	       __get_unaligned_le32(p);
}

static inline void __put_unaligned_le16(__u16 val, __u8 *p)
{
	*p++ = val;
	*p++ = val >> 8;
}

static inline void __put_unaligned_le32(__u32 val, __u8 *p)
{
	__put_unaligned_le16(val >> 16, p + 2);
	__put_unaligned_le16(val, p);
}

static inline void __put_unaligned_le64(__u64 val, __u8 *p)
{
	__put_unaligned_le32(val >> 32, p + 4);
	__put_unaligned_le32(val, p);
}

static inline __u16 get_unaligned_le16(const void *p)
{
	return __get_unaligned_le16((const __u8 *)p);
}

static inline __u32 get_unaligned_le32(const void *p)
{
	return __get_unaligned_le32((const __u8 *)p);
}

static inline __u64 get_unaligned_le64(const void *p)
{
	return __get_unaligned_le64((const __u8 *)p);
}

static inline void put_unaligned_le16(__u16 val, void *p)
{
	__put_unaligned_le16(val, p);
}

static inline void put_unaligned_le32(__u32 val, void *p)
{
	__put_unaligned_le32(val, p);
}

static inline void put_unaligned_le64(__u64 val, void *p)
{
	__put_unaligned_le64(val, p);
}

#endif /* _TOOLS_LE_BYTESHIFT_H */
