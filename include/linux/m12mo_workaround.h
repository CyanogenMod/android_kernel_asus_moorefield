//=== Usage: #include <linux/m12mo_workaround.h> ===//

#ifndef _M12MO_ISP_WORKAROUND_H
#define _M12MO_ISP_WORKAROUND_H

#ifdef CONFIG_VIDEO_M12MO
int m12mo_getOpenIntelISP(void);
void m12mo_setOpenIntelISP(bool b);
void m12mo_startCapture(void);
int m12mo_read_fac(u8 len, u8 category, u8 reg, u32 *val);
int m12mo_write_fac(u8 len, u8 category, u8 reg, u32 val);
int m12mo_s_power_fac(int on);
int m12mo_status_fac(void);
void notify_m12mo_atomisp_dead(void);
#else
int m12mo_getOpenIntelISP(void){ return 0;}
void m12mo_setOpenIntelISP(bool b){ return ;}
void m12mo_startCapture(void){ return ;}
int m12mo_read_fac(u8 len, u8 category, u8 reg, u32 *val){ return 0;}
int m12mo_write_fac(u8 len, u8 category, u8 reg, u32 val){ return 0;}
int m12mo_s_power_fac(int on){ return 0;}
int m12mo_status_fac(void){ return 0;}
void notify_m12mo_atomisp_dead(void){ return ;}
#endif

#endif
