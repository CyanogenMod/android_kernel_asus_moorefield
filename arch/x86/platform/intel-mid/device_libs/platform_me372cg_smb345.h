/*
 * smb345 platform data header file
 */
#ifndef _PLATFORM_SMB345_H_
#define _PLATFORM_SMB345_H_

#define SMB345_STAT_GPIO "chg_stat"
#define SMB345_INOK_GPIO	"CHG_INOK#"
#define SMB346_OTG_GPIO		"CHG_OTG"
extern void *smb345_platform_data(void *info) __attribute__((weak));
#endif
