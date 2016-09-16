extern int getOpenIntelISP(void);
extern void setOpenIntelISP(bool b);
extern void startCapture(void);
extern int m10mo_read_fac(u8 len, u8 category, u8 reg, u32 *val);
extern int m10mo_write_fac(u8 len, u8 category, u8 reg, u32 val);
extern int m10mo_s_power_fac(int on);
