#ifndef _HRT_NUMERIC_H
#define _HRT_NUMERIC_H

#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif

static inline unsigned int
_hrt_align_down(unsigned int a, unsigned int b)
{
  return a - (a % b);
}

static inline unsigned int
_hrt_align_down_p2(unsigned int a, unsigned int b)
{
  /* assert(is_power_of_2(b)); */
  return a & -b;
}

static inline unsigned int
_hrt_align_up(unsigned int a, unsigned int b)
{
  return _hrt_align_down(a+b-1, b);
} 

static inline unsigned int
_hrt_align_up_p2(unsigned int a, unsigned int b)
{
  /* assert(is_power_of_2(b)); */
  return (a+b-1) & -b;
}

static inline unsigned int
_hrt_div_down(unsigned int a, unsigned int b)
{
  return a / b;
}

static inline unsigned int
_hrt_div_nearest(unsigned int a, unsigned int b)
{
  return (a+(b/2)) / b;
}

static inline unsigned int
_hrt_div_up(unsigned int a, unsigned int b)
{
  return (a+b-1) / b;
}

static inline unsigned int
_hrt_exp2i(unsigned int x)
{
  return 1 << x;
}

static inline unsigned int
_hrt_is_p2(unsigned int x)
{
  return (x&(x-1)) == 0;
}

static inline unsigned int
_hrt_log2i(unsigned int x)
{
  unsigned int t = 0;
  x >>= 1;
  while (x)
  {
    x >>= 1;
    t++;
  }
  return t;
}

/* get next larger power of 2 (http://aggregate.org/MAGIC) */
static inline unsigned int
_hrt_p2_up(unsigned int x)
{
        x |= (x >> 1);
        x |= (x >> 2);
        x |= (x >> 4);
        x |= (x >> 8);
        x |= (x >> 16);
        return x+1;
}

static inline unsigned int
_hrt_p2_down(unsigned int x)
{
  unsigned int y = _hrt_p2_up(x);
  return (x == y) ? y : y<<1;
}

#endif /* _HRT_NUMERIC_H */
