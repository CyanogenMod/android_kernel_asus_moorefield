//=== Usage: #include <linux/m10mo_workaround.h> ===//

#ifndef _M10MO_ISP_WORKAROUND_H
#define _M10MO_ISP_WORKAROUND_H

#ifdef CONFIG_VIDEO_M10MO
int getOpenIntelISP(void);
void setOpenIntelISP(bool b);
void m10mo_startCapture(void);
int m10mo_read_fac(u8 len, u8 category, u8 reg, u32 *val);
int m10mo_read_laser(u8 len, u8 category, u8 reg, u32 *val);
int m10mo_write_fac(u8 len, u8 category, u8 reg, u32 val);
int m10mo_s_power_fac(int on);
int m10mo_s_power_for_ov5670(int on);
int m10mo_status_fac(void);
void notify_m10mo_atomisp_dead(void);
void notify_m10mo_front_camera_power_status(u8 power);
#else
int getOpenIntelISP(void) { return 0;}
void setOpenIntelISP(bool b) { return ;}
void m10mo_startCapture(void) { return ;}
int m10mo_read_fac(u8 len, u8 category, u8 reg, u32 *val) { return 0;}
int m10mo_write_fac(u8 len, u8 category, u8 reg, u32 val) { return 0;}
int m10mo_s_power_fac(int on) { return 0;}
int m10mo_status_fac(void) { return 0;}
void notify_m10mo_atomisp_dead(void) { return ;}
#endif

#endif
