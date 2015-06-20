// stdafx.h : Include the standard system Include files
// or include files without frequently modified

#pragma once

#ifndef BUILD_UG31XX_LIB

#ifndef	uG31xx_OS_ANDROID

#ifndef CONFIG_ASUS_UG31XX_BATTERY

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN            // 從 Windows 標頭排除不常使用的成員
#endif

#include "targetver.h"

#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS      // 明確定義部分的 CString 建構函式

#include <afxwin.h>         // MFC 核心與標準元件
#include <afxext.h>         // MFC 擴充功能

#ifndef _AFX_NO_OLE_SUPPORT
#include <afxole.h>         // MFC OLE 類別
#include <afxodlgs.h>       // MFC OLE 對話方塊類別
#include <afxdisp.h>        // MFC Automation 類別
#endif // _AFX_NO_OLE_SUPPORT

#ifndef _AFX_NO_DB_SUPPORT
#include <afxdb.h>                      // MFC ODBC 資料庫類別
#endif // _AFX_NO_DB_SUPPORT

#ifndef _AFX_NO_DAO_SUPPORT
#include <afxdao.h>                     // MFC DAO 資料庫類別
#endif // _AFX_NO_DAO_SUPPORT

#ifndef _AFX_NO_OLE_SUPPORT
#include <afxdtctl.h>           // MFC 支援的 Internet Explorer 4 通用控制項
#endif
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h>                     // MFC 支援的 Windows 通用控制項
#endif // _AFX_NO_AFXCMN_SUPPORT

#endif  ///< end of CONFIG_ASUS_UG31XX_BATTERY

#endif	///< end of uG31xx_OS_ANDROID

#endif  ///< end of BUILD_UG31XX_LIB

