#ifndef _HRT_DEFS_H_
#define _HRT_DEFS_H_

#ifndef HRTCAT
#define _HRTCAT(m, n)     m##n
#define HRTCAT(m, n)      _HRTCAT(m, n)
#endif

#ifndef HRTSTR
#define _HRTSTR(x)   #x
#define HRTSTR(x)    _HRTSTR(x)
#endif

#ifndef HRTMIN
#define HRTMIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef HRTMAX
#define HRTMAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#endif /* _HRT_DEFS_H_ */
