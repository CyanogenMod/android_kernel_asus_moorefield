/* 
* drivers/input/touchscreen/ftxxxx_ex_fun.c
*
* FocalTech ftxxxx expand function for debug. 
*
* Copyright (c) 2014  Focaltech Ltd.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*Note:the error code of EIO is the general error in this file.
*/

//#define DEBUG
#include "ftxxxx_ex_fun.h"
#include "ftxxxx_ts.h"
#include "test_lib.h"

#include <linux/mount.h>
#include <linux/netdevice.h>
#include <linux/unistd.h>
#include <linux/proc_fs.h>
#include <linux/version.h>
#include <linux/string.h>
#include <../fs/proc/internal.h>
#include <linux/HWVersion.h>
#include <asm/intel-mid.h>

int factory_test=1;
short databuf_short[625];

static int FWvendorid = 0;
static int CTPM_FW_SIZE = 0;
static unsigned char *CTPM_FW;
static unsigned char CTPM_FW_ZE550ML_31[] = {
#include "ASUS_ZE550ML_5446_0x31_0xB1_20151105_app.i"
};
static unsigned char CTPM_FW_ZE550ML_71[] = {
#include "ASUS_ZE550ML_5446_0x71_0xBE_20151105_app.i"
};
static unsigned char CTPM_FW_ZE550ML_72[] = {
#include "ASUS_ZE550ML_5446_0x72_0xBF_20151111_app.i"
};
static unsigned char CTPM_FW_ZE550ML_81[] = {
#include "ASUS_ZE550ML_5446_0x81_0xBE_20151105_app.i"
};
static unsigned char CTPM_FW_ZE550ML_82[] = {
#include "ASUS_ZE550ML_5446_0x82_0xBD_20151105_app.i"
};
static unsigned char CTPM_FW_ZE551ML_31[] = {
#include "ASUS_ZE551ML_5446_0x31_0xB3_20151026_app.i"
};
static unsigned char CTPM_FW_ZE551ML_71[] = {
#include "ASUS_ZE551ML_5446_0x71_0xC3_20151026_app.i"
};
static unsigned char CTPM_FW_ZE551ML_73[] = {
#include "ASUS_ZE551ML_5446_0x73_0xC2_20151026_app.i"
};
static unsigned char CTPM_FW_ZE551ML_75[] = {
#include "ASUS_ZE551ML_5446_0x75_0x4A_20150109_app.i"
};
static unsigned char CTPM_FW_ZE551ML_81[] = {
#include "ASUS_ZE551ML_5446_0x81_0xC4_20151026_app.i"
};
static unsigned char CTPM_FW_ZE551ML_83[] = {
#include "ASUS_ZE551ML_5446_0x83_0xC3_20151026_app.i"
};
static unsigned char CTPM_FW_ZE551ML_85[] = {
#include "ASUS_ZE551ML_5446_0x80_0x41_20141212_app.i"
};
static unsigned char CTPM_FW_ZX551ML_61[] = {
#include "ASUS_ZX551ML_5446_0x61_0x43_20151111_app.i"
};
static unsigned char CTPM_FW_ZX550ML_71[] = {
#include "ASUS_ZX550ML_5446_0x71_0x22_20141230_app.i"
};
static unsigned char CTPM_FW_ZX551ML_81[] = {
#include "ASUS_ZX551ML_5446_0x81_0x43_20151111_app.i"
};

//zax 20141116 ++++++++++++++
int Save_rawData1[TX_NUM_MAX][RX_NUM_MAX];
int TX_NUM;
int RX_NUM;


int SCab_1;
int SCab_2;
int SCab_3;
int SCab_4;
int SCab_5;
int SCab_6;
int SCab_7;
int SCab_8;
//zax 20141116 -------------------

//<ASUS_Glove+>
int glove_mode = 0;
//<ASUS_Glove->
//<ASUS_DTP+>
//#ifdef ASUS_TOUCH_DTP_WAKEUP
int dclick_mode = 0;
//#endif
//<ASUS_DTP->
//<ASUS_Gesture+>
//#ifdef ASUS_TOUCH_GESTURE_MODE
int gesture_mode = 0;
//#endif
//<ASUS_Gesture->
//<ASUS_COVER+>
//#ifdef ASUS_COVER_MODE
int cover_mode = 0;
//#endif
//<ASUS_COVER->
bool keypad_enable = true;

int HidI2c_To_StdI2c(struct i2c_client * client)
{
	u8 auc_i2c_write_buf[10] = {0};
	u8 reg_val[10] = {0};
	int iRet = 0;

	auc_i2c_write_buf[0] = 0xEB;
	auc_i2c_write_buf[1] = 0xAA;
	auc_i2c_write_buf[2] = 0x09;

	reg_val[0] = reg_val[1] =  reg_val[2] = 0x00;

       iRet = ftxxxx_i2c_Write(client, auc_i2c_write_buf, 3);

	msleep(10);
	iRet = ftxxxx_i2c_Read(client, auc_i2c_write_buf, 0, reg_val, 3);
	dev_dbg(&client->dev, "Change to STDI2cValue,REG1 = 0x%x,REG2 = 0x%x,REG3 = 0x%x, iRet=%d\n",
					reg_val[0], reg_val[1], reg_val[2], iRet);	

	if (reg_val[0] == 0xEB
		&& reg_val[1] == 0xAA
		&& reg_val[2] == 0x08) 
	{
		dev_dbg(&client->dev, "HidI2c_To_StdI2c successful.\n");
		iRet = 1;
	}
	else
	{
		pr_err("HidI2c_To_StdI2c error.\n");
		iRet = 0;
	}

	return iRet;
}

struct Upgrade_Info{
	u16		delay_aa;		/*delay of write FT_UPGRADE_AA*/
	u16		delay_55;		/*delay of write FT_UPGRADE_55*/
	u8		upgrade_id_1;	/*upgrade id 1*/
	u8		upgrade_id_2;	/*upgrade id 2*/
	u16		delay_readid;	/*delay of read id*/
};

struct Upgrade_Info upgradeinfo;

static struct mutex g_device_mutex;

int ftxxxx_write_reg(struct i2c_client * client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};
	buf[0] = regaddr;
	buf[1] = regvalue;

	return ftxxxx_i2c_Write(client, buf, sizeof(buf));
}

int ftxxxx_read_reg(struct i2c_client * client, u8 regaddr, u8 * regvalue)
{
	return ftxxxx_i2c_Read(client, &regaddr, 1, regvalue, 1);
}

struct i2c_client *G_Client = NULL;

int FTS_I2c_Read(unsigned char * wBuf, int wLen, unsigned char *rBuf, int rLen)
{
	if(NULL == G_Client)
	{
		return -1;
	}

	return ftxxxx_i2c_Read(G_Client, wBuf, wLen, rBuf, rLen);
}

int FTS_I2c_Write(unsigned char * wBuf, int wLen)
{	
	if(NULL == G_Client)
	{
		return -1;
	}	

	return ftxxxx_i2c_Write(G_Client, wBuf, wLen);
}


int fts_ctpm_auto_clb(struct i2c_client * client)
{
	unsigned char uc_temp;
	unsigned char i ;

	/*start auto CLB*/
	msleep(200);
	ftxxxx_write_reg(client, 0, 0x40);  
	msleep(100);   /*make sure already enter factory mode*/
	ftxxxx_write_reg(client, 2, 0x4);  /*write command to start calibration*/
	msleep(300);

	for(i=0;i<100;i++)
	{
		ftxxxx_read_reg(client, 0, &uc_temp);
		if (0x0 == ((uc_temp&0x70)>>4))  /*return to normal mode, calibration finish*/
		{
			break;
		}
		msleep(20);	    
	}

	/*calibration OK*/
	ftxxxx_write_reg(client, 0, 0x40);  /*goto factory mode for store*/
	msleep(200);   /*make sure already enter factory mode*/
	ftxxxx_write_reg(client, 2, 0x5);  /*store CLB result*/
	msleep(300);
	ftxxxx_write_reg(client, 0, 0x0); /*return to normal mode*/ 
	msleep(300);
	/*store CLB result OK*/

	return 0;
}

/*
upgrade with *.i file
*/
int fts_ctpm_fw_upgrade_with_i_file(struct i2c_client * client)
{
	u8 * pbt_buf = NULL;
	int i_ret;
	int fw_len = CTPM_FW_SIZE;

	/*judge the fw that will be upgraded
	* if illegal, then stop upgrade and return.
	*/
	if(fw_len<8 || fw_len>54*1024)
	{
		pr_err("FW length error\n");
		return -EIO;
	}	

	/*FW upgrade*/
	pbt_buf = CTPM_FW;
	/*call the upgrade function*/
	i_ret =  fts_ctpm_fw_upgrade(client, pbt_buf, CTPM_FW_SIZE);
	if (i_ret != 0)
	{
		dev_err(&client->dev, "[FTS] upgrade failed. err=%d.\n", i_ret);
	}
	else
	{
#ifdef AUTO_CLB
		fts_ctpm_auto_clb(client);  /*start auto CLB*/
#endif
	}
	return i_ret;
}

u8 fts_ctpm_get_i_file_ver(void)
{
	u16 ui_sz;
	ui_sz = CTPM_FW_SIZE;

	if (ui_sz > 2)
	{
		return CTPM_FW[ui_sz - 2];
	}
	else
	{
		return 0x00; /*default value*/
	}
}

int fts_ctpm_auto_upgrade(struct i2c_client * client)
{
	u8 uc_tp_vender_id_A8;
	u8 uc_tp_vender_id;
	u8 uc_host_fm_ver=FTXXXX_REG_FW_VER;
	u8 uc_tp_fm_ver;
	int i_ret;
	char projectcode[32];

	printk("[FTS] %s start\n", __func__);

	ftxxxx_read_reg(client, FTXXXX_REG_VENDOR_ID, &uc_tp_vender_id_A8);
	fts_ctpm_fw_upgrade_ReadVendorID(client, &uc_tp_vender_id);
	fts_ctpm_fw_upgrade_ReadProjectCode(client, projectcode);
	printk("[FTS] FTXXXX_REG_VENDOR_ID: 0x%x\n", FTXXXX_REG_VENDOR_ID);
	printk("[FTS] Device TP vendor id(0xA8): 0x%x\n", uc_tp_vender_id_A8);
	printk("[FTS] Device TP vendor id(0xD784): 0x%x\n", uc_tp_vender_id);
	printk("[FTS] Device TP project code(0xD7A0): %s\n", projectcode);
	if ((strcmp(projectcode, "ZE550ML") == 0) && (uc_tp_vender_id == 0x30) && (Read_LCD_ID() == ZE550ML_LCD_ID_OTM_TM))
	{
		printk("[FTS] Project: ZE550ML, LCD: TM, TP: Biel\n");
		FWvendorid = 31;
		CTPM_FW = CTPM_FW_ZE550ML_31;
		CTPM_FW_SIZE = sizeof(CTPM_FW_ZE550ML_31);
	}
	else if ((strcmp(projectcode, "ZE550ML") == 0) && (uc_tp_vender_id == 0x70) && (Read_LCD_ID() == ZE550ML_LCD_ID_OTM_TM))
	{
		printk("[FTS] Project: ZE550ML, LCD: TM, TP: LCE\n");
		FWvendorid = 71;
		CTPM_FW = CTPM_FW_ZE550ML_71;
		CTPM_FW_SIZE = sizeof(CTPM_FW_ZE550ML_71);
	}
	else if ((strcmp(projectcode, "ZE550ML") == 0) && (uc_tp_vender_id == 0x70) && (Read_LCD_ID() == ZE550ML_LCD_ID_OTM_CPT))
	{
		printk("[FTS] Project: ZE550ML, LCD: CPT, TP: LCE\n");
		FWvendorid = 72;
		CTPM_FW = CTPM_FW_ZE550ML_72;
		CTPM_FW_SIZE = sizeof(CTPM_FW_ZE550ML_72);
	}
	else if ((strcmp(projectcode, "ZE550ML") == 0) && (uc_tp_vender_id == 0x80) && (Read_LCD_ID() == ZE550ML_LCD_ID_OTM_TM))
	{
		printk("[FTS] Project: ZE550ML, LCD: TM, TP: Jtouch\n");
		FWvendorid = 81;
		CTPM_FW = CTPM_FW_ZE550ML_81;
		CTPM_FW_SIZE = sizeof(CTPM_FW_ZE550ML_81);
	}
	else if ((strcmp(projectcode, "ZE550ML") == 0) && (uc_tp_vender_id == 0x80) && (Read_LCD_ID() == ZE550ML_LCD_ID_OTM_CPT))
	{
		printk("[FTS] Project: ZE550ML, LCD: CPT, TP: Jtouch\n");
		FWvendorid = 82;
		CTPM_FW = CTPM_FW_ZE550ML_82;
		CTPM_FW_SIZE = sizeof(CTPM_FW_ZE550ML_82);
	}
	else if ((strcmp(projectcode, "ZE551ML") == 0) && (uc_tp_vender_id == 0x30) && (Read_LCD_ID() == ZE551ML_LCD_ID_NT_TM))
	{
		printk("[FTS] Project: ZE551ML, LCD: TM, TP: Biel\n");
		FWvendorid = 31;
		CTPM_FW = CTPM_FW_ZE551ML_31;
		CTPM_FW_SIZE = sizeof(CTPM_FW_ZE551ML_31);
	}
	else if ((strcmp(projectcode, "ZE551ML") == 0) && (uc_tp_vender_id == 0x70) && (Read_LCD_ID() == ZE551ML_LCD_ID_NT_TM))
	{
		printk("[FTS] Project: ZE551ML, LCD: TM, TP: LCE\n");
		FWvendorid = 71;
		CTPM_FW = CTPM_FW_ZE551ML_71;
		CTPM_FW_SIZE = sizeof(CTPM_FW_ZE551ML_71);
	}
	else if ((strcmp(projectcode, "ZE551ML") == 0) && (uc_tp_vender_id == 0x70) && (Read_LCD_ID() == ZE551ML_LCD_ID_NT_AUO))
	{
		printk("[FTS] Project: ZE551ML, LCD: AUO, TP: LCE\n");
		FWvendorid = 73;
		CTPM_FW = CTPM_FW_ZE551ML_73;
		CTPM_FW_SIZE = sizeof(CTPM_FW_ZE551ML_73);
	}
	else if ((strcmp(projectcode, "ZE551ML") == 0) && (uc_tp_vender_id == 0x70) && (Read_LCD_ID() == ZE551ML_LCD_ID_OTM_INX))
	{
		printk("[FTS] Project: ZE551ML, LCD: INX, TP: LCE\n");
		FWvendorid = 75;
		CTPM_FW = CTPM_FW_ZE551ML_75;
		CTPM_FW_SIZE = sizeof(CTPM_FW_ZE551ML_75);
	}
	else if ((strcmp(projectcode, "ZE551ML") == 0) && (uc_tp_vender_id == 0x80) && (Read_LCD_ID() == ZE551ML_LCD_ID_NT_TM))
	{
		printk("[FTS] Project: ZE551ML, LCD: TM, TP: Jtouch\n");
		FWvendorid = 81;
		CTPM_FW = CTPM_FW_ZE551ML_81;
		CTPM_FW_SIZE = sizeof(CTPM_FW_ZE551ML_81);
	}
	else if ((strcmp(projectcode, "ZE551ML") == 0) && (uc_tp_vender_id == 0x80) && (Read_LCD_ID() == ZE551ML_LCD_ID_NT_AUO))
	{
		printk("[FTS] Project: ZE551ML, LCD: AUO, TP: Jtouch\n");
		FWvendorid = 83;
		CTPM_FW = CTPM_FW_ZE551ML_83;
		CTPM_FW_SIZE = sizeof(CTPM_FW_ZE551ML_83);
	}
	else if ((strcmp(projectcode, "ZE551ML") == 0) && (uc_tp_vender_id == 0x80) && (Read_LCD_ID() == ZE551ML_LCD_ID_OTM_INX))
	{
		printk("[FTS] Project: ZE551ML, LCD: INX, TP: Jtouch\n");
		FWvendorid = 85;
		CTPM_FW = CTPM_FW_ZE551ML_85;
		CTPM_FW_SIZE = sizeof(CTPM_FW_ZE551ML_85);
	}
	else if ((strcmp(projectcode, "ZX551ML") == 0) && (uc_tp_vender_id == 0x61) && (Read_TP_ID() == 1))
	{
		printk("[FTS] Project: ZX551ML, TP: GIS\n");
		FWvendorid = 61;
		CTPM_FW = CTPM_FW_ZX551ML_61;
		CTPM_FW_SIZE = sizeof(CTPM_FW_ZX551ML_61);
	}
	else if (strcmp(projectcode, "ZX550ML") == 0 && uc_tp_vender_id == 0x71)
	{
		printk("[FTS] Project: ZX550ML, TP: LCE\n");
		FWvendorid = 71;
		CTPM_FW = CTPM_FW_ZX550ML_71;
		CTPM_FW_SIZE = sizeof(CTPM_FW_ZX550ML_71);
	}
	else if ((strcmp(projectcode, "ZX551ML") == 0) && (uc_tp_vender_id == 0x81) && (Read_TP_ID() == 0))
	{
		printk("[FTS] Project: ZX551ML, TP: Jtouch\n");
		FWvendorid = 81;
		CTPM_FW = CTPM_FW_ZX551ML_81;
		CTPM_FW_SIZE = sizeof(CTPM_FW_ZX551ML_81);
	}
	else
	{
		printk("[FTS] Others, No Update\n");
		CTPM_FW = CTPM_FW_ZE551ML_81;
		CTPM_FW_SIZE = sizeof(CTPM_FW_ZE551ML_81);
		return 0;
	}

	ftxxxx_read_reg(client, FTXXXX_REG_FW_VER, &uc_tp_fm_ver);
	uc_host_fm_ver = fts_ctpm_get_i_file_ver();
	printk("[FTS] FTXXXX_REG_FW_VER: 0x%x\n", FTXXXX_REG_FW_VER);
	printk("[FTS] Device TP FW version: 0x%x\n", uc_tp_fm_ver);
	printk("[FTS] Image TP FW version: 0x%x\n", uc_host_fm_ver);

	if ( uc_tp_fm_ver == FTXXXX_REG_FW_VER  ||   /*the firmware in touch panel maybe corrupted*/
		uc_tp_fm_ver != uc_host_fm_ver  /*the firmware in host flash is new, need upgrade*/
		)
	{
		msleep(100);
		printk("[FTS] uc_tp_fm_ver = 0x%x, uc_host_fm_ver = 0x%x\n",
			uc_tp_fm_ver, uc_host_fm_ver);
		i_ret = fts_ctpm_fw_upgrade_with_i_file(client);    
		if (i_ret == 0)
		{
			msleep(300);
			uc_host_fm_ver = fts_ctpm_get_i_file_ver();
			printk("[FTS] upgrade to new version 0x%x\n", uc_host_fm_ver);
		}
		else
		{
			dev_err(&client->dev, "[FTS] upgrade failed ret=%d.\n", i_ret);
			return -EIO;
		}
	}

	printk("[FTS] %s end\n", __func__);

	return 0;
}

/*
*get upgrade information depend on the ic type
*/
static void fts_get_upgrade_info(struct Upgrade_Info * upgrade_info)
{
	switch(DEVICE_IC_TYPE)
	{
		/*case IC_ftxxxx:
		upgrade_info->delay_55 = ftxxxx_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = ftxxxx_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = ftxxxx_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = ftxxxx_UPGRADE_ID_2;
		upgrade_info->delay_readid = ftxxxx_UPGRADE_READID_DELAY;
		break;
		case IC_FT5606:
		upgrade_info->delay_55 = FT5606_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5606_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5606_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5606_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5606_UPGRADE_READID_DELAY;
		break;
		case IC_FT5316:
		upgrade_info->delay_55 = FT5316_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5316_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5316_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5316_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5316_UPGRADE_READID_DELAY;
		break;
		case IC_FT5X36:
		upgrade_info->delay_55 = FT5X36_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5X36_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5X36_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5X36_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5X36_UPGRADE_READID_DELAY;
		break;*/
	case IC_FT5X46:
		upgrade_info->delay_55 = FT5X46_UPGRADE_55_DELAY;
		upgrade_info->delay_aa = FT5X46_UPGRADE_AA_DELAY;
		upgrade_info->upgrade_id_1 = FT5X46_UPGRADE_ID_1;
		upgrade_info->upgrade_id_2 = FT5X46_UPGRADE_ID_2;
		upgrade_info->delay_readid = FT5X46_UPGRADE_READID_DELAY;
		break;
	default:
		break;
	}
}
#define FTS_UPGRADE_LOOP	5

int ftxxxxEnterUpgradeMode(struct i2c_client * client, int iAddMs, bool bIsSoft)
{
	//bool bSoftResetOk = false;

	u32 i = 0;		
	u8 auc_i2c_write_buf[10];		
	int i_ret;	

	u8 reg_val[4] = {0};

	if(bIsSoft)//reset by App
	{
		fts_get_upgrade_info(&upgradeinfo);
		HidI2c_To_StdI2c(client);msleep(10);

		for (i = 0; i < FTS_UPGRADE_LOOP; i++) 
		{
			/*********Step 1:Reset  CTPM *****/
			//write 0xaa to register 0xfc
			i_ret = ftxxxx_write_reg(client, 0xfc, FT_UPGRADE_AA);
			msleep(upgradeinfo.delay_aa);	
			//write 0x55 to register 0xfc 
			i_ret = ftxxxx_write_reg(client, 0xfc, FT_UPGRADE_55);

			msleep(200);

			HidI2c_To_StdI2c(client);msleep(10);

			//auc_i2c_write_buf[0] = 0xfc;
			//auc_i2c_write_buf[1] = 0x66;
			//i_ret = ftxxxx_write_reg(client, auc_i2c_write_buf[0], auc_i2c_write_buf[1]);

			//if(i_ret < 0)
			//dev_err(&client->dev, "[FTS] failed writing  0x66 to register 0xbc or oxfc! \n");

			//msleep(50);

			/*********Step 2:Enter upgrade mode *****/
			auc_i2c_write_buf[0] = FT_UPGRADE_55;		
			auc_i2c_write_buf[1] = FT_UPGRADE_AA;
			i_ret = ftxxxx_i2c_Write(client, auc_i2c_write_buf, 2);
			if(i_ret < 0)
			{
				dev_err(&client->dev, "[FTS] failed writing  0xaa ! \n");
				continue;
			}

			/*********Step 3:check READ-ID***********************/
			msleep(1);
			auc_i2c_write_buf[0] = 0x90;
			auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;

			reg_val[0] = reg_val[1] = 0x00;

			ftxxxx_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);

			if (reg_val[0] == upgradeinfo.upgrade_id_1
				&& reg_val[1] == upgradeinfo.upgrade_id_2) {
					dev_dbg(&client->dev, "[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
						reg_val[0], reg_val[1]);
					break;
			} else {
				dev_err(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
					reg_val[0], reg_val[1]);

				continue;
			}
		}
	}
	else//reset by hardware reset pin
	{		
		fts_get_upgrade_info(&upgradeinfo);

		for (i = 0; i < FTS_UPGRADE_LOOP; i++) 
		{
			/*********Step 1:Reset  CTPM *****/			
			ftxxxx_reset_tp(0);
			msleep(10);
			ftxxxx_reset_tp(1);

			msleep(8+iAddMs);	//time (5~20ms)

			//HidI2c_To_StdI2c(client);msleep(10);

			/*********Step 2:Enter upgrade mode *****/
			auc_i2c_write_buf[0] = FT_UPGRADE_55;
			auc_i2c_write_buf[1] = FT_UPGRADE_AA;
			i_ret = ftxxxx_i2c_Write(client, auc_i2c_write_buf, 2);
			if(i_ret < 0)
			{
				dev_err(&client->dev, "[FTS] failed writing  0xaa ! \n");
				continue;
			}

			/*********Step 3:check READ-ID***********************/
			msleep(1);
			auc_i2c_write_buf[0] = 0x90;
			auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;

			reg_val[0] = reg_val[1] = 0x00;

			ftxxxx_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);

			if (reg_val[0] == upgradeinfo.upgrade_id_1
				&& reg_val[1] == upgradeinfo.upgrade_id_2) {
					dev_dbg(&client->dev, "[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
						reg_val[0], reg_val[1]);
					break;
			} else {
				dev_err(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
					reg_val[0], reg_val[1]);

				continue;
			}
		}
	}
       
		
	return 0;
}

int  fts_ctpm_fw_upgrade(struct i2c_client * client, u8* pbt_buf, u32 dw_lenth)
{
	u8 reg_val[4] = {0};
	u32 i = 0;
	u32 packet_number;
	u32 j;
	u32 temp;
	u32 lenght;
	u8 packet_buf[FTS_PACKET_LENGTH + 6];
	u8 auc_i2c_write_buf[10];
	u8 bt_ecc;
	int i_ret;

	i_ret = HidI2c_To_StdI2c(client);

	if(i_ret == 0)
	{
		dev_err(&client->dev, "HidI2c change to StdI2c fail ! \n");
	}

	fts_get_upgrade_info(&upgradeinfo);

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		/*********Step 1:Reset  CTPM *****/
		ftxxxx_write_reg(client, 0xfc, FT_UPGRADE_AA);
		msleep(upgradeinfo.delay_aa);
		ftxxxx_write_reg(client, 0xfc, FT_UPGRADE_55);
		msleep(5);
		/*********Step 2:Enter upgrade mode *****/
		i_ret = HidI2c_To_StdI2c(client);

		if(i_ret == 0)
		{
			dev_err(&client->dev, "HidI2c change to StdI2c fail ! \n");
			continue;
		}
		msleep(10);
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		auc_i2c_write_buf[1] = FT_UPGRADE_AA;
		i_ret = ftxxxx_i2c_Write(client, auc_i2c_write_buf, 2);
		if(i_ret < 0)
		{
			dev_err(&client->dev, "failed writing  0x55 and 0xaa ! \n");
			continue;
		}

		/*********Step 3:check READ-ID***********************/
//break; //cyh1
		msleep(1);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;

		reg_val[0] = reg_val[1] = 0x00;

		ftxxxx_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);

		if (reg_val[0] == upgradeinfo.upgrade_id_1
			&& reg_val[1] == upgradeinfo.upgrade_id_2) {
				pr_info("Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
					reg_val[0], reg_val[1]);
				break;
		} else {
			dev_err(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);

			continue;
		}
	}
	if (i >= FTS_UPGRADE_LOOP )
		return -EIO;

	/*Step 4:erase app and panel paramenter area*/
	pr_info("Step 4:erase app and panel paramenter area\n");
	auc_i2c_write_buf[0] = 0x61;
	ftxxxx_i2c_Write(client, auc_i2c_write_buf, 1);     //erase app area 
	msleep(1350);

	for(i = 0;i < 15;i++)
	{
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		ftxxxx_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 2);

		if(0xF0==reg_val[0] && 0xAA==reg_val[1])
		{
			break;
		}
		msleep(50);
	}

       //write bin file length to FW bootloader.
	auc_i2c_write_buf[0] = 0xB0;
	auc_i2c_write_buf[1] = (u8) ((dw_lenth >> 16) & 0xFF);
	auc_i2c_write_buf[2] = (u8) ((dw_lenth >> 8) & 0xFF);
	auc_i2c_write_buf[3] = (u8) (dw_lenth & 0xFF);

	ftxxxx_i2c_Write(client, auc_i2c_write_buf, 4);

	/*********Step 5:write firmware(FW) to ctpm flash*********/
	bt_ecc = 0;
	pr_info("Step 5:write firmware(FW) to ctpm flash\n");

	//dw_lenth = dw_lenth - 8;
	temp = 0;
	packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
	packet_buf[0] = 0xbf;
	packet_buf[1] = 0x00;

	for (j = 0; j < packet_number; j++) {
		temp = j * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		lenght = FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (lenght >> 8);
		packet_buf[5] = (u8) lenght;

		for (i = 0; i < FTS_PACKET_LENGTH; i++) {
			packet_buf[6 + i] = pbt_buf[j * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}
		ftxxxx_i2c_Write(client, packet_buf, FTS_PACKET_LENGTH + 6);
		//msleep(10);

		for(i = 0;i < 30;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			ftxxxx_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 2);

			if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
			{
				break;
			}
			//msleep(1);

		}
	}

	if ((dw_lenth) % FTS_PACKET_LENGTH > 0) {
		temp = packet_number * FTS_PACKET_LENGTH;
		packet_buf[2] = (u8) (temp >> 8);
		packet_buf[3] = (u8) temp;
		temp = (dw_lenth) % FTS_PACKET_LENGTH;
		packet_buf[4] = (u8) (temp >> 8);
		packet_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			packet_buf[6 + i] = pbt_buf[packet_number * FTS_PACKET_LENGTH + i];
			bt_ecc ^= packet_buf[6 + i];
		}        
		ftxxxx_i2c_Write(client, packet_buf, temp + 6);

		for(i = 0;i < 30;i++)
		{
			auc_i2c_write_buf[0] = 0x6a;
			reg_val[0] = reg_val[1] = 0x00;
			ftxxxx_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 2);

			if ((j + 0x1000) == (((reg_val[0]) << 8) | reg_val[1]))
			{
				break;
			}
			//msleep(1);

		}
	}

	msleep(50);

	/*********Step 6: read out checksum***********************/
	/*send the opration head */
	pr_info("Step 6: read out checksum\n");
	auc_i2c_write_buf[0] = 0x64;
	ftxxxx_i2c_Write(client, auc_i2c_write_buf, 1); 
	msleep(300);

	temp = 0;
	auc_i2c_write_buf[0] = 0x65;
	auc_i2c_write_buf[1] = (u8)(temp >> 16);
	auc_i2c_write_buf[2] = (u8)(temp >> 8);
	auc_i2c_write_buf[3] = (u8)(temp);
	temp = dw_lenth;
	auc_i2c_write_buf[4] = (u8)(temp >> 8);
	auc_i2c_write_buf[5] = (u8)(temp);
	i_ret = ftxxxx_i2c_Write(client, auc_i2c_write_buf, 6); 
	msleep(dw_lenth/256);

	for(i = 0;i < 100;i++)
	{
		auc_i2c_write_buf[0] = 0x6a;
		reg_val[0] = reg_val[1] = 0x00;
		ftxxxx_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 2);

		if (0xF0==reg_val[0] && 0x55==reg_val[1])
		{
			break;
		}
		//msleep(1);
	}

	auc_i2c_write_buf[0] = 0x66;
	ftxxxx_i2c_Read(client, auc_i2c_write_buf, 1, reg_val, 1);
	if (reg_val[0] != bt_ecc) 
	{
		dev_err(&client->dev, "[FTS]--ecc error! FW=%02x bt_ecc=%02x\n",
			reg_val[0],
			bt_ecc);

		return -EIO;
	}
	printk(KERN_WARNING "[FTS]checksum %X %X \n",reg_val[0],bt_ecc);       
	/*********Step 7: reset the new FW***********************/
	pr_info("Step 7: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	ftxxxx_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(200);   //make sure CTP startup normally 
	i_ret = HidI2c_To_StdI2c(client);//Android to Std i2c.

	if(i_ret == 0)
	{
		dev_err(&client->dev, "HidI2c change to StdI2c fail ! \n");
	}

	return 0;
}

/* sysfs debug*/

/*
*get firmware size

@firmware_name:firmware name

note:the firmware default path is sdcard.
if you want to change the dir, please modify by yourself.
*/
static int ftxxxx_GetFirmwareSize(char * firmware_name)
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic; 
	off_t fsize = 0; 
	char filepath[128];
	memset(filepath, 0, sizeof(filepath));

	sprintf(filepath, "%s", firmware_name);

	if(NULL == pfile){
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if(IS_ERR(pfile)){
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}
	inode=pfile->f_dentry->d_inode; 
	magic=inode->i_sb->s_magic;
	fsize=inode->i_size; 
	filp_close(pfile, NULL);

	return fsize;
}

/*
*read firmware buf for .bin file.

@firmware_name: fireware name
@firmware_buf: data buf of fireware

note:the firmware default path is sdcard. 
if you want to change the dir, please modify by yourself.
*/
static int ftxxxx_ReadFirmware(char * firmware_name, unsigned char * firmware_buf)
{
	struct file* pfile = NULL;
	struct inode *inode;
	unsigned long magic; 
	off_t fsize; 
	char filepath[128];
	loff_t pos;

	mm_segment_t old_fs;
	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s", firmware_name);
	if(NULL == pfile){
		pfile = filp_open(filepath, O_RDONLY, 0);
	}
	if(IS_ERR(pfile)){
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}
	inode=pfile->f_dentry->d_inode; 
	magic=inode->i_sb->s_magic;
	fsize=inode->i_size; 
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;

	vfs_read(pfile, firmware_buf, fsize, &pos);

	filp_close(pfile, NULL);
	set_fs(old_fs);
	return 0;
}

/*
upgrade with *.bin file
*/

int fts_ctpm_fw_upgrade_with_app_file(struct i2c_client * client, char * firmware_name)
{
	u8* pbt_buf = NULL;
	int i_ret;
	int fwsize = ftxxxx_GetFirmwareSize(firmware_name);
	if(fwsize <= 0)
	{
		dev_err(&client->dev, "%s ERROR:Get firmware size failed\n", __FUNCTION__);
		return -EIO;
	}
	if(fwsize<8 || fwsize>54*1024)
	{
		dev_err(&client->dev, "FW length error\n");
		return -EIO;
	}

	/*=========FW upgrade========================*/
	pbt_buf = (unsigned char *) kmalloc(fwsize+1,GFP_ATOMIC);
	if(ftxxxx_ReadFirmware(firmware_name, pbt_buf))
	{
		dev_err(&client->dev, "%s() - ERROR: request_firmware failed\n", __FUNCTION__);
		kfree(pbt_buf);
		return -EIO;
	}

	/*call the upgrade function*/
	i_ret =  fts_ctpm_fw_upgrade(client, pbt_buf, fwsize);
	if (i_ret != 0)
	{
		dev_err(&client->dev, "%s() - ERROR:[FTS] upgrade failed i_ret = %d.\n",__FUNCTION__,  i_ret);
	}
	else
	{

#ifdef AUTO_CLB
		fts_ctpm_auto_clb(client);  /*start auto CLB*/
#endif
	}
	kfree(pbt_buf);

	return i_ret;
}

static ssize_t ftxxxx_tpfwver_show(struct device *dev,
struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;
	u8	   fwver = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	mutex_lock(&g_device_mutex);
	if(ftxxxx_read_reg(client, FTXXXX_REG_FW_VER, &fwver) < 0)
		num_read_chars = snprintf(buf, PAGE_SIZE, "get tp fw version fail!\n");
	else
		num_read_chars = snprintf(buf, PAGE_SIZE, "%02X\n", fwver);

	mutex_unlock(&g_device_mutex);
	return num_read_chars;
}

static ssize_t ftxxxx_tpfwver_store(struct device *dev,
struct device_attribute *attr,
	const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}

static ssize_t ftxxxx_tprwreg_show(struct device *dev,
struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

static ssize_t ftxxxx_tprwreg_store(struct device *dev,
struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);	
	ssize_t num_read_chars = 0;
	int retval;
	//u32 wmreg=0;
	long unsigned int wmreg=0;
	u8 regaddr=0xff,regvalue=0xff;
	u8 valbuf[5]={0};
	memset(valbuf, 0, sizeof(valbuf));

	mutex_lock(&g_device_mutex);
	num_read_chars = count - 1;

	if(num_read_chars!=2)
	{
		if(num_read_chars!=4)
		{
			dev_err(dev, "please input 2 or 4 character\n");
			goto error_return;
		}
	}

	memcpy(valbuf, buf, num_read_chars);
	retval = strict_strtoul(valbuf, 16, &wmreg);
	if (0 != retval)
	{
		dev_err(dev, "%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"\n", __FUNCTION__, buf);
		goto error_return;
	}

	if(2 == num_read_chars)
	{
		//read register
		regaddr = wmreg;
		if(ftxxxx_read_reg(client, regaddr, &regvalue) < 0)
			dev_err(dev, "Could not read the register(0x%02x)\n", regaddr);
		else
			printk("[FTS] the register(0x%02x) is 0x%02x\n", regaddr, regvalue);
	}
	else
	{
		regaddr = wmreg>>8;
		regvalue = wmreg;
		if(ftxxxx_write_reg(client, regaddr, regvalue)<0)
			dev_err(dev, "Could not write the register(0x%02x)\n", regaddr);
		else
			printk("[FTS] Write 0x%02x into register(0x%02x) successful\n", regvalue, regaddr);
	}
error_return:
	mutex_unlock(&g_device_mutex);

	return count;
}

int iHigh=1;
static ssize_t ftxxxx_fwupdate_show(struct device *dev,
struct device_attribute *attr, char *buf)
{
	if(iHigh)
		{
		iHigh=0;
		ftxxxx_reset_tp(1);
		}
	else
		{
		iHigh=1;
		ftxxxx_reset_tp(0);
		}
	/* place holder for future use */
	return -EPERM;
}

/*upgrade from *.i*/
static ssize_t ftxxxx_fwupdate_store(struct device *dev,
struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct ftxxxx_ts_data *data = NULL;
	u8 uc_host_fm_ver;int i_ret;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	data = (struct ftxxxx_ts_data *) i2c_get_clientdata( client );

	mutex_lock(&g_device_mutex);

	disable_irq(client->irq);
#if 0
	/*i_ret = fts_ctpm_fw_upgrade_with_i_file(client);    
	if (i_ret == 0)
	{
		msleep(300);
		uc_host_fm_ver = fts_ctpm_get_i_file_ver();
		dev_dbg(dev, "%s [FTS] upgrade to new version 0x%x\n", __FUNCTION__, uc_host_fm_ver);
	}
	else
	{
		dev_err(dev, "%s ERROR:[FTS] upgrade failed ret=%d.\n", __FUNCTION__, i_ret);
	}*/

	ftxxxxEnterUpgradeMode(client, 0 ,false);
#endif
//cyh1
	fts_ctpm_auto_upgrade(client);
	enable_irq(client->irq);

	mutex_unlock(&g_device_mutex);

	return count;
}

static ssize_t ftxxxx_fwupgradeapp_show(struct device *dev,
struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}
//upgrade from app.bin
static ssize_t ftxxxx_fwupgradeapp_store(struct device *dev,
struct device_attribute *attr,
	const char *buf, size_t count)
{
	char fwname[128];
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[count-1] = '\0';

	mutex_lock(&g_device_mutex);
	disable_irq(client->irq);

	fts_ctpm_fw_upgrade_with_app_file(client, fwname);

	enable_irq(client->irq);

	mutex_unlock(&g_device_mutex);

	return count;
}

static ssize_t ftxxxx_ftsgetprojectcode_show(struct device *dev,
struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;
	char projectcode[32]; 
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	memset(projectcode, 0, sizeof(projectcode));
	mutex_lock(&g_device_mutex);
	if(fts_ctpm_fw_upgrade_ReadProjectCode(client, projectcode) < 0)
	{
		num_read_chars = snprintf(buf, PAGE_SIZE, "get projcet code fail!\n");
	}
	else
	{
		num_read_chars = snprintf(buf, PAGE_SIZE, "%s\n", projectcode);
	}

	mutex_unlock(&g_device_mutex);
	return num_read_chars;


}

static ssize_t ftxxxx_ftsgetprojectcode_store(struct device *dev,
struct device_attribute *attr,
	const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}

static ssize_t ftxxxx_ftreset_ic(struct device *dev, 
struct device_attribute *attr, 
	const char *buf, size_t count)
{
	/* place holder for future use */
	ftxxxx_reset_tp(0);
	msleep(50);
	ftxxxx_reset_tp(1);
	return -EPERM;
}

static ssize_t ftxxxx_tpvendorid_show(struct device *dev,
struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;
	u8 uc_tp_vender_id;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
/*
	mutex_lock(&g_device_mutex);
	if(ftxxxx_read_reg(client, FTXXXX_REG_VENDOR_ID, &uc_tp_vender_id) < 0)
	{
		num_read_chars = snprintf(buf, PAGE_SIZE, "get tp vendor id fail!\n");
		printk("[FTS] get tp vendor id fail!\n");
	}
	else
	{
		num_read_chars = snprintf(buf, PAGE_SIZE, "%x\n", uc_tp_vender_id);
		printk("[FTS] TP Vendor ID: 0x%x\n", uc_tp_vender_id);
	}
	mutex_unlock(&g_device_mutex);
*/
	mutex_lock(&g_device_mutex);
	if(fts_ctpm_fw_upgrade_ReadVendorID(client, &uc_tp_vender_id) < 0)
	{
		num_read_chars = snprintf(buf, PAGE_SIZE, "get tp vendor id fail!\n");
		printk("[FTS] get tp vendor id fail!\n");
	}
	else
	{
		num_read_chars = snprintf(buf, PAGE_SIZE, "%x\n", uc_tp_vender_id);
		printk("[FTS] TP Vendor ID: 0x%x\n", uc_tp_vender_id);
	}
	mutex_unlock(&g_device_mutex);

	return num_read_chars;
}

static ssize_t ftxxxx_tpvendorid_store(struct device *dev,
struct device_attribute *attr,
	const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}

//<ASUS_Glove+>
static ssize_t ftxxxx_glove_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	//struct ftxxxx_ts_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			glove_mode);
}

static ssize_t ftxxxx_glove_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	//struct ftxxxx_ts_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	glove_mode = input;
	printk("[ftxxxx] glove mode: %d\n", glove_mode);

	if (glove_mode == 1) {
		if(ftxxxx_write_reg(client, 0xC0, 1)<0)	//Enable
			dev_err(dev, "Could not write the register 0xC0\n");
	} else {
		if(ftxxxx_write_reg(client, 0xC0, 0)<0)	//Disable
			dev_err(dev, "Could not write the register 0xC0\n");
	}

	return count;
}
//<ASUS_Glove->

//<ASUS_DTP+>
//#ifdef ASUS_TOUCH_DTP_WAKEUP
static ssize_t ftxxxx_dclick_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	//struct ftxxxx_ts_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			dclick_mode);
}

static ssize_t ftxxxx_dclick_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	//struct ftxxxx_ts_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	dclick_mode = input;
	printk("[ftxxxx] dclick mode: %d\n", dclick_mode);

	return count;
}
//#endif
//<ASUS_DTP->

//<ASUS_Gesture+>
//#ifdef ASUS_TOUCH_GESTURE_MODE
static ssize_t ftxxxx_gesture_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	//struct ftxxxx_ts_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			gesture_mode);
}

static ssize_t ftxxxx_gesture_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	char input[7] = {0};
	int value=0, i;
	//struct ftxxxx_ts_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	if (sscanf(buf, "%s", &input) != 1)
		return -EINVAL;

	for (i=0;i<7;i++)
	{
		value += (((int)input[i])-48)<<(6-i);
	}
	gesture_mode = value;
	printk("[ftxxxx] gesture mode: %d\n", gesture_mode);

	return count;
}
//#endif
//<ASUS_Gesture->

//<ASUS_COVER+>
//#ifdef ASUS_COVER_MODE
static ssize_t ftxxxx_cover_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	//struct ftxxxx_ts_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	return snprintf(buf, PAGE_SIZE, "%u\n",
			cover_mode);
}

static ssize_t ftxxxx_cover_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	//struct ftxxxx_ts_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	cover_mode = input;
	printk("[ftxxxx] cover mode: %d\n", cover_mode);

	if (cover_mode == 1) {
		if(ftxxxx_write_reg(client, 0xC1, 1)<0)	//Enable
			dev_err(dev, "Could not write the register 0xC1\n");
		if(ftxxxx_write_reg(client, 0xC3, 2)<0)	//Enable
			dev_err(dev, "Could not write the register 0xC3\n");
	} else {
		if(ftxxxx_write_reg(client, 0xC1, 0)<0)	//Disable
			dev_err(dev, "Could not write the register 0xC1\n");
		if(ftxxxx_write_reg(client, 0xC3, 0)<0)	//Disable
			dev_err(dev, "Could not write the register 0xC3\n");
	}

	return count;
}
//#endif
//<ASUS_COVER->

static ssize_t ftxxxx_keypad_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	//struct ftxxxx_ts_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", keypad_enable);
}

static ssize_t ftxxxx_keypad_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int input;
	struct ftxxxx_ts_data *data = NULL;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	data = (struct ftxxxx_ts_data *) i2c_get_clientdata( client );
	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	keypad_enable = (input != 0);

	if (keypad_enable) {
		set_bit(KEY_BACK, data->input_dev->keybit);
		set_bit(KEY_HOME, data->input_dev->keybit);
		set_bit(KEY_MENU, data->input_dev->keybit);
	} else {
		clear_bit(KEY_BACK, data->input_dev->keybit);
		clear_bit(KEY_HOME, data->input_dev->keybit);
		clear_bit(KEY_MENU, data->input_dev->keybit);
	}

	printk("[ftxxxx] dclick mode: %d\n", keypad_enable);

	return count;
}

static ssize_t ftxxxx_enable_touch_show(struct device *dev,
struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	ftxxxx_reset_tp(0);
	msleep(10);
	ftxxxx_reset_tp(1);
	msleep(80);

	return snprintf(buf, PAGE_SIZE, "1\n");
}

static ssize_t ftxxxx_enable_touch_store(struct device *dev,
struct device_attribute *attr,
	const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}

static ssize_t ftxxxx_disable_touch_show(struct device *dev,
struct device_attribute *attr, char *buf)
{
	int ret;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	ret = ftxxxx_write_reg(client, 0xa5, 3);

	return snprintf(buf, PAGE_SIZE, "%d\n" ,ret);
}

static ssize_t ftxxxx_disable_touch_store(struct device *dev,
struct device_attribute *attr,
	const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}

static ssize_t ftxxxx_fwvendorid_show(struct device *dev,
struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	return snprintf(buf, PAGE_SIZE, "%d\n" ,FWvendorid);
}

static ssize_t ftxxxx_fwvendorid_store(struct device *dev,
struct device_attribute *attr,
	const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}

#define FTXXXX_INI_FILEPATH "/"
static int ftxxxx_GetInISize(char *config_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];
	memset(filepath, 0, sizeof(filepath));

	sprintf(filepath, "%s%s", FTXXXX_INI_FILEPATH, config_name);

	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);

	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

static int ftxxxx_ReadInIData(char *config_name,
	char *config_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FTXXXX_INI_FILEPATH, config_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", filepath);
		return -EIO;
	}

	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, config_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	return 0;
}
static int ftxxxx_get_testparam_from_ini(char *config_name)
{
	char *filedata = NULL;

	int inisize = ftxxxx_GetInISize(config_name);

	pr_info("inisize = %d \n ", inisize);
	if (inisize <= 0) {
		pr_err("%s ERROR:Get firmware size failed\n",
			__func__);
		return -EIO;
	}

	filedata = kmalloc(inisize + 1, GFP_ATOMIC);

	if (ftxxxx_ReadInIData(config_name, filedata)) {
		pr_err("%s() - ERROR: request_firmware failed\n",
			__func__);
		kfree(filedata);
		return -EIO;
	} else {
		pr_info("ftxxxx_ReadInIData successful\n");
	}

	SetParamData(filedata);
	return 0;
}
//zax 20141116 ++++++++++++++
int CSV()
{
	int count = 0,frame_flag=0;
	int err = 0;
	int i = 0, j = 0,space=(11+TX_NUM);
	int flag[8];
	//struct file *filp = NULL;
	//mm_segment_t oldfs = { 0 };

	
	struct file *pfile = NULL;
	struct file *pfile1 = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	loff_t pos;
	mm_segment_t old_fs;
	//char filepath[128];
	char *databuf = NULL;
	char *databuf1 = NULL;
	ssize_t err1;
	
	mutex_lock(&g_device_mutex);
	

	
	//memset(filepath, 0, sizeof(filepath));
	//sprintf(filepath, "/data/fts_scap_sample");
	//printk("save auto test data to %s\n", filepath);

	databuf = kmalloc(10 * 1024, GFP_ATOMIC);
	if (databuf == NULL) {
		pr_err("alloc data buf fail !\n");
		return -1;
	}

	memset(databuf, 0, sizeof(databuf));

	databuf1 = kmalloc(10 * 1024, GFP_ATOMIC);
	if (databuf1 == NULL) {
		pr_err("alloc data buf fail !\n");
		return -1;
	}

	memset(databuf1, 0, sizeof(databuf1));
	//sprintf(databuf, "Project Code: %s\n", g_projectcode);

	
	if (NULL == pfile)
		pfile = filp_open("/data/save_raw_data.csv", O_WRONLY|O_CREAT|O_TRUNC, 0664);

	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", "");
		return -EIO;
	}

	old_fs = get_fs();
	set_fs(get_ds());
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	pos = 0;

	
	
	
			
	flag[0]=SCab_1;		
	flag[1]=SCab_2;	
	flag[2]=SCab_3;	
	flag[3]=SCab_4;
	flag[4]=SCab_5;		
	flag[5]=SCab_6;	
	flag[6]=SCab_7;	
	flag[7]=SCab_8;
	printk("[FTS][%s]	raw data \n", __func__);

	//filp = filp_open("/mnt/sdcard/11.csv", O_RDWR | O_CREAT, S_IRUSR);
	//if (IS_ERR(filp)) 
	//{
	//	printk("[Focal][TOUCH_ERR] %s: open /data/1.csv failed\n", __func__);
	//	return 0;
	//}
	//oldfs = get_fs();
	//set_fs(get_ds());

				
	//count += sprintf(buf + count,"TestItem Num, 3, RawData Test, 7, %d, %d, 11, 1, SCap CB Test, 9, %d, %d, %d, 1, SCap CB Test, 9, %d, %d, %d, 2, SCap RawData Test, 10, %d, %d, %d, 1, SCap RawData Test, 10, %d, %d, %d, 2\n",TX_NUM,RX_NUM,(SCab_1+SCab_2),RX_NUM,(11+TX_NUM),(SCab_3+SCab_4),RX_NUM,(11+TX_NUM+SCab_1+SCab_2),(SCab_5+SCab_6),RX_NUM,(11+TX_NUM+SCab_1+SCab_2+SCab_3+SCab_4),(SCab_7+SCab_8),RX_NUM,(11+TX_NUM+SCab_1+SCab_2+SCab_3+SCab_4+SCab_5+SCab_6));
	
	count += sprintf(databuf + count,"ECC, 85, 170, IC Name, FT5X46, IC Code, 21\n");
	count += sprintf(databuf + count,"TestItem Num, 3, RawData Test, 7, %d, %d, 11, 2, ",TX_NUM,RX_NUM);
	frame_flag=0;
	for(i=0;i<2;i++)
	{
		if((flag[2*i]+flag[2*i+1])>0)
		{
			frame_flag++;
			count += sprintf(databuf + count,"SCap CB Test, 9, %d, %d, %d, %d, ",(flag[2*i]+flag[2*i+1]),RX_NUM,space,frame_flag);
			space+=flag[2*i]+flag[2*i+1];
		}
	}
	frame_flag=0;
	for(i=2;i<4;i++)
	{
		if((flag[2*i]+flag[2*i+1])>0)
		{
			frame_flag++;
			count += sprintf(databuf + count,"SCap RawData Test, 10, %d, %d, %d, %d, ",(flag[2*i]+flag[2*i+1]),RX_NUM,space,frame_flag);
			space+=flag[2*i]+flag[2*i+1];
		}
	}



	count += sprintf(databuf + count,"\n");
	for(i=0;i<8;i++)
		count += sprintf(databuf + count,"\n");
	//printk("[FTS][%s]	123 data result = %d !\n", __func__, err);
	focal_save_scap_sample1();
	//printk("[FTS][%s]	789 result = !\n");
	for (i = 0; i < TX_NUM+8; i++) {
		if(i>=TX_NUM && flag[i-TX_NUM]==0)
		{			
			continue;
		}
		for (j = 0; j < RX_NUM; j++) 
		{
			if(i>=TX_NUM && j==TX_NUM && ((i-TX_NUM)%2)==1)
			{			
				break;
			}
			
				
			count += sprintf(databuf + count,"%5d, ",  Save_rawData1[i][j]);
			//msleep(2000);
		}
		//msleep(10000);
		count += sprintf(databuf + count,"\n");
	}
	/*Print RawData End*/		

	//printk("[FTS][%s]	456data result = !\n");
	err1 = vfs_write(pfile, databuf, count, &pos);
	if (err1 < 0)
		pr_err("write scap sample fail!\n");
	filp_close(pfile, NULL);
	set_fs(old_fs);
	//printk("[FTS][%s]	1111111111111 result = !\n");
	kfree(databuf);
	
	//filp->f_op->write(filp, buf, count, &filp->f_pos);
	//set_fs(oldfs);
	//filp_close(filp, NULL);


	kfree(databuf1);
	mutex_unlock(&g_device_mutex);
	printk("[FTS][%s]	result = !\n", __func__);
	return 1;
}
int TXT()
{
	int count = 0,frame_flag=0;
	int err = 0;
	int i = 0, j = 0,space=(11+TX_NUM);
	int flag[8];
	//struct file *filp = NULL;
	//mm_segment_t oldfs = { 0 };

	
	struct file *pfile = NULL;
	struct file *pfile1 = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	loff_t pos;
	mm_segment_t old_fs;
	//char filepath[128];
	char *databuf = NULL;
	char *databuf1 = NULL;
	ssize_t err1;
	
	mutex_lock(&g_device_mutex);
	

	
	//memset(filepath, 0, sizeof(filepath));
	//sprintf(filepath, "/data/fts_scap_sample");
	//DBG("save auto test data to %s\n", filepath);

	databuf = kmalloc(10 * 1024, GFP_ATOMIC);
	if (databuf == NULL) {
		pr_err("alloc data buf fail !\n");
		return -1;
	}

	memset(databuf, 0, sizeof(databuf));

	databuf1 = kmalloc(10 * 1024, GFP_ATOMIC);
	if (databuf1 == NULL) {
		pr_err("alloc data buf fail !\n");
		return -1;
	}

	memset(databuf1, 0, sizeof(databuf1));
	//sprintf(databuf, "Project Code: %s\n", g_projectcode);

	
	if (NULL == pfile)
		pfile = filp_open("/data/save_raw_data.txt", O_WRONLY|O_CREAT|O_TRUNC, 0664);

	if (IS_ERR(pfile)) {
		pr_err("error occured while opening file %s.\n", "");
		return -EIO;
	}

	old_fs = get_fs();
	set_fs(get_ds());
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	pos = 0;

	
	
	
			
	flag[0]=SCab_1;		
	flag[1]=SCab_2;	
	flag[2]=SCab_3;	
	flag[3]=SCab_4;
	flag[4]=SCab_5;		
	flag[5]=SCab_6;	
	flag[6]=SCab_7;	
	flag[7]=SCab_8;
	printk("[FTS][%s]	raw data \n", __func__);

	//filp = filp_open("/mnt/sdcard/11.csv", O_RDWR | O_CREAT, S_IRUSR);
	//if (IS_ERR(filp)) 
	//{
	//	printk("[Focal][TOUCH_ERR] %s: open /data/1.csv failed\n", __func__);
	//	return 0;
	//}
	//oldfs = get_fs();
	//set_fs(get_ds());

				
	
	//printk("[FTS][%s]	123 data result = %d !\n", __func__, err);
	focal_save_scap_sample1();
	//printk("[FTS][%s]	789 result = !\n");
		
	count=0;			
	for (i = 0; i < TX_NUM+8; i++) {
		if(i>=TX_NUM && flag[i-TX_NUM]==0)
		{			
			continue;
		}
		for (j = 0; j < RX_NUM; j++) 
		{
			if(i>=TX_NUM && j==TX_NUM && ((i-TX_NUM)%2)==1)
			{			
				break;
			}
			
				
			count += sprintf(databuf + count,"%d,",  Save_rawData1[i][j]);
			//msleep(2000);
		}
		//msleep(10000);
		count += sprintf(databuf + count,"\n");
	}
	//printk("[FTS][%s]	456data result = !\n");
	err1 = vfs_write(pfile, databuf, count, &pos);
	if (err1 < 0)
		pr_err("write scap sample fail!\n");
	filp_close(pfile, NULL);
	set_fs(old_fs);
	//printk("[FTS][%s]	1111111111111 result = !\n");
	kfree(databuf);
	
	//filp->f_op->write(filp, buf, count, &filp->f_pos);
	//set_fs(oldfs);
	//filp_close(filp, NULL);


	kfree(databuf1);
	mutex_unlock(&g_device_mutex);
	printk("[FTS][%s]	result = !\n", __func__);
	return 1;
}
static ssize_t ftxxxx_ftsscaptest_show(struct device *dev,
struct device_attribute *attr, char *buf)
{	
/*	int count = 0,frame_flag=0;
	int err = 0;
	int i = 0, j = 0,space=(11+TX_NUM);
	int flag[8];
	struct file *filp = NULL;
	mm_segment_t oldfs = { 0 };
	
	
	mutex_lock(&g_device_mutex);
	

	


	
	
	
			
	flag[0]=SCab_1;		
	flag[1]=SCab_2;	
	flag[2]=SCab_3;	
	flag[3]=SCab_4;
	flag[4]=SCab_5;		
	flag[5]=SCab_6;	
	flag[6]=SCab_7;	
	flag[7]=SCab_8;
	printk("[FTS][%s]	raw data \n", __func__);

	filp = filp_open("/data/save_raw_data.csv", O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
	if (IS_ERR(filp)) 
	{
		printk("[FTS][TOUCH_ERR] %s: open /data/save_raw_data.csv failed\n", __func__);
		return 0;
	}
	oldfs = get_fs();
	set_fs(get_ds());

				
	//count += sprintf(buf + count,"TestItem Num, 3, RawData Test, 7, %d, %d, 11, 1, SCap CB Test, 9, %d, %d, %d, 1, SCap CB Test, 9, %d, %d, %d, 2, SCap RawData Test, 10, %d, %d, %d, 1, SCap RawData Test, 10, %d, %d, %d, 2\n",TX_NUM,RX_NUM,(SCab_1+SCab_2),RX_NUM,(11+TX_NUM),(SCab_3+SCab_4),RX_NUM,(11+TX_NUM+SCab_1+SCab_2),(SCab_5+SCab_6),RX_NUM,(11+TX_NUM+SCab_1+SCab_2+SCab_3+SCab_4),(SCab_7+SCab_8),RX_NUM,(11+TX_NUM+SCab_1+SCab_2+SCab_3+SCab_4+SCab_5+SCab_6));
	
	count += sprintf(buf + count,"ECC, 85, 170, IC Name, FT5X46, IC Code, 21\n");
	count += sprintf(buf + count,"TestItem Num, 3, RawData Test, 7, %d, %d, 11, 2, ",TX_NUM,RX_NUM);
	frame_flag=0;
	for(i=0;i<2;i++)
	{
		if((flag[2*i]+flag[2*i+1])>0)
		{
			frame_flag++;
			count += sprintf(buf + count,"SCap CB Test, 9, %d, %d, %d, %d, ",(flag[2*i]+flag[2*i+1]),RX_NUM,space,frame_flag);
			space+=flag[2*i]+flag[2*i+1];
		}
	}
	frame_flag=0;
	for(i=2;i<4;i++)
	{
		if((flag[2*i]+flag[2*i+1])>0)
		{
			frame_flag++;
			count += sprintf(buf + count,"SCap RawData Test, 10, %d, %d, %d, %d, ",(flag[2*i]+flag[2*i+1]),RX_NUM,space,frame_flag);
			space+=flag[2*i]+flag[2*i+1];
		}
	}



	count += sprintf(buf + count,"\n");
	for(i=0;i<8;i++)
		count += sprintf(buf + count,"\n");
	printk("[FTS][%s]	123 data result = %d !\n", __func__, err);
	focal_save_scap_sample1();
	
	for (i = 0; i < TX_NUM+8; i++) {
		if(i>=TX_NUM && flag[i-TX_NUM]==0)
		{			
			continue;
		}
		for (j = 0; j < RX_NUM; j++) 
		{
			if(i>=TX_NUM && j==TX_NUM && ((i-TX_NUM)%2)==1)
			{			
				break;
			}
			
				
			count += sprintf(buf + count,"%5d, ",  Save_rawData1[i][j]);
			//msleep(2000);
		}
		//msleep(10000);
		count += sprintf(buf + count,"\n");
	}
	//Print RawData End		


	
	
	filp->f_op->write(filp, buf, count, &filp->f_pos);
	set_fs(oldfs);
	filp_close(filp, NULL);


	msleep(1000);
	

	filp = filp_open("/data/save_raw_data.txt", O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
	if (IS_ERR(filp)) 
	{
		printk("[FTS][TOUCH_ERR] %s: open /data/save_raw_data.txt failed\n", __func__);
		return 0;
	}
	oldfs = get_fs();
	set_fs(get_ds());

	count=0;			
	for (i = 0; i < TX_NUM+8; i++) {
		if(i>=TX_NUM && flag[i-TX_NUM]==0)
		{			
			continue;
		}
		for (j = 0; j < RX_NUM; j++) 
		{
			if(i>=TX_NUM && j==TX_NUM && ((i-TX_NUM)%2)==1)
			{			
				break;
			}
			
				
			count += sprintf(buf + count,"%d,",  Save_rawData1[i][j]);
			//msleep(2000);
		}
		//msleep(10000);
		count += sprintf(buf + count,"\n");
	}
	
	
	filp->f_op->write(filp, buf, count, &filp->f_pos);
	set_fs(oldfs);
	filp_close(filp, NULL);
	mutex_unlock(&g_device_mutex);
*/
	TXT();
	CSV();
	return scnprintf (buf, PAGE_SIZE,"%d\n",factory_test); 
	
}
//zax 20141116 --------------------
static ssize_t ftxxxx_ftsscaptest_store(struct device *dev,
struct device_attribute *attr,
	const char *buf, size_t count)
{
	/* place holder for future use */
	char cfgname[128];
	//short *TestData=NULL;
	int iTxNumber=0;
	int iRxNumber=0;
	int i=0, j=0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	G_Client=client;

	//	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	memset(cfgname, 0, sizeof(cfgname));
	sprintf(cfgname, "%s", buf);
	cfgname[count-1] = '\0';

	//Init_I2C_Read_Func(FTS_I2c_Read);
	//Init_I2C_Write_Func(FTS_I2c_Write);
	//StartTestTP();

	mutex_lock(&g_device_mutex);

	Init_I2C_Write_Func(FTS_I2c_Write);
	Init_I2C_Read_Func(FTS_I2c_Read);
	if(ftxxxx_get_testparam_from_ini(cfgname) <0)
	{
		printk("[FTS]get testparam from ini failure\n");
//		factory_test = -1;
	}
	else 
	{
		printk("[FTS]tp test Start...\n");
		if(true == StartTestTP())
		{
			printk("[FTS]tp test pass\n");
			factory_test = 0;
		}
		else
		{
			printk("[FTS]tp test failure\n");
			factory_test = 1;
		}

		FreeTestParamData();
	}

	mutex_unlock(&g_device_mutex);

	return count;
}

/****************************************/
/* sysfs */
/*get the fw version
*example:cat ftstpfwver
*/
static DEVICE_ATTR(ftstpfwver, S_IRUGO|S_IWUSR, ftxxxx_tpfwver_show, ftxxxx_tpfwver_store);

/*upgrade from *.i 
*example: echo 1 > ftsfwupdate
*/
static DEVICE_ATTR(ftsfwupdate, S_IRUGO|S_IWUSR, ftxxxx_fwupdate_show, ftxxxx_fwupdate_store);

/*read and write register
*read example: echo 88 > ftstprwreg ---read register 0x88
*write example:echo 8807 > ftstprwreg ---write 0x07 into register 0x88
*
*note:the number of input must be 2 or 4.if it not enough,please fill in the 0.
*/
static DEVICE_ATTR(ftstprwreg, S_IRUGO|S_IWUSR, ftxxxx_tprwreg_show, ftxxxx_tprwreg_store);

/*upgrade from app.bin 
*example:echo "*_app.bin" > ftsfwupgradeapp
*/
static DEVICE_ATTR(ftsfwupgradeapp, S_IRUGO|S_IWUSR, ftxxxx_fwupgradeapp_show, ftxxxx_fwupgradeapp_store);

static DEVICE_ATTR(ftsgetprojectcode, S_IRUGO|S_IWUSR, ftxxxx_ftsgetprojectcode_show, ftxxxx_ftsgetprojectcode_store);

static DEVICE_ATTR(ftsscaptest, S_IRUGO|S_IWUSR, ftxxxx_ftsscaptest_show, ftxxxx_ftsscaptest_store);

static DEVICE_ATTR(ftresetic, S_IRUGO|S_IWUSR, NULL, ftxxxx_ftreset_ic);

static DEVICE_ATTR(ftstpvendorid, S_IRUGO|S_IWUSR, ftxxxx_tpvendorid_show, ftxxxx_tpvendorid_store);

//<ASUS_Glove+>
static DEVICE_ATTR(ftsglovemode, S_IRUGO|S_IWUSR, ftxxxx_glove_mode_show, ftxxxx_glove_mode_store);
//<ASUS_Glove->

//<ASUS_DTP+>
//#ifdef ASUS_TOUCH_DTP_WAKEUP
static DEVICE_ATTR(ftsdclickmode, S_IRUGO|S_IWUSR, ftxxxx_dclick_mode_show, ftxxxx_dclick_mode_store);
//#endif
//<ASUS_DTP->

//<ASUS_Gesture+>
//#ifdef ASUS_TOUCH_GESTURE_MODE
static DEVICE_ATTR(ftsgesturemode, S_IRUGO|S_IWUSR, ftxxxx_gesture_mode_show, ftxxxx_gesture_mode_store);
//#endif
//<ASUS_Gesture->

//<ASUS_COVER+>
//#ifdef ASUS_COVER_MODE
static DEVICE_ATTR(ftscovermode, S_IRUGO|S_IWUSR, ftxxxx_cover_mode_show, ftxxxx_cover_mode_store);
//#endif
//<ASUS_COVER->
static DEVICE_ATTR(ftskeypadenable, S_IRUGO|S_IWUSR|S_IWGRP, ftxxxx_keypad_enable_show, ftxxxx_keypad_enable_store);
static DEVICE_ATTR(ftsenabletouch, S_IRUGO|S_IWUSR, ftxxxx_enable_touch_show, ftxxxx_enable_touch_store);
static DEVICE_ATTR(ftsdisabletouch, S_IRUGO|S_IWUSR, ftxxxx_disable_touch_show, ftxxxx_disable_touch_store);
static DEVICE_ATTR(ftsfwvendorid, S_IRUGO|S_IWUSR, ftxxxx_fwvendorid_show, ftxxxx_fwvendorid_store);

/*add your attr in here*/
static struct attribute *ftxxxx_attributes[] = {
	&dev_attr_ftstpfwver.attr,
	&dev_attr_ftsfwupdate.attr,
	&dev_attr_ftstprwreg.attr,
	&dev_attr_ftsfwupgradeapp.attr,
	&dev_attr_ftsgetprojectcode.attr,	
	&dev_attr_ftsscaptest.attr,
	&dev_attr_ftresetic.attr,
	&dev_attr_ftstpvendorid.attr,
//<ASUS_Glove+>
	&dev_attr_ftsglovemode.attr,
//<ASUS_Glove->
//<ASUS_DTP+>
//#ifdef ASUS_TOUCH_DTP_WAKEUP
	&dev_attr_ftsdclickmode.attr,
//#endif
//<ASUS_DTP->
//<ASUS_Gesture+>
//#ifdef ASUS_TOUCH_GESTURE_MODE
	&dev_attr_ftsgesturemode.attr,
//#endif
//<ASUS_Gesture->
//<ASUS_COVER+>
//#ifdef ASUS_COVER_MODE
	&dev_attr_ftscovermode.attr,
//#endif
//<ASUS_COVER->
	&dev_attr_ftskeypadenable.attr,
	&dev_attr_ftsenabletouch.attr,
	&dev_attr_ftsdisabletouch.attr,
	&dev_attr_ftsfwvendorid.attr,
	NULL
};

static struct attribute_group ftxxxx_attribute_group = {
	.attrs = ftxxxx_attributes
};

/*create sysfs for debug*/
int ftxxxx_create_sysfs(struct i2c_client * client)
{
	int err;
	err = sysfs_create_group(&client->dev.kobj, &ftxxxx_attribute_group);
	if (0 != err)
	{
		dev_err(&client->dev, "%s() - ERROR: sysfs_create_group() failed.error code: %d\n", __FUNCTION__, err);
		sysfs_remove_group(&client->dev.kobj, &ftxxxx_attribute_group);
		return -EIO;
	}
	else
	{		
		mutex_init(&g_device_mutex);
		dev_dbg(&client->dev, "ftxxxx:%s() - sysfs_create_group() succeeded. \n", __FUNCTION__);
	}
	HidI2c_To_StdI2c(client);
	
	return err;
}

int ftxxxx_remove_sysfs(struct i2c_client * client)
{
	sysfs_remove_group(&client->dev.kobj, &ftxxxx_attribute_group);
	mutex_destroy(&g_device_mutex);

	return 0;
}

/*create apk debug channel*/
#define PROC_UPGRADE			0
#define PROC_READ_REGISTER		1
#define PROC_WRITE_REGISTER		2
#define PROC_AUTOCLB			4
#define PROC_UPGRADE_INFO		5
#define PROC_WRITE_DATA			6
#define PROC_READ_DATA			7


#define PROC_NAME	"ftxxxx-debug"
static unsigned char proc_operate_mode = PROC_UPGRADE;
static struct proc_dir_entry *ftxxxx_proc_entry;
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
static ssize_t ftxxxx_debug_read(struct file *file, char __user *buf, size_t count, loff_t *ppos);
static ssize_t ftxxxx_debug_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos);
#else
static int ftxxxx_debug_read( char *page, char **start,	off_t off, int count, int *eof, void *data );
static int ftxxxx_debug_write(struct file *filp, const char __user *buff, unsigned long len, void *data);
#endif
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
static const struct file_operations ftxxxx_proc_fops = {
	.owner = THIS_MODULE,
	.read = ftxxxx_debug_read,
	.write = ftxxxx_debug_write,
};
#endif

#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
/*interface of write proc*/
static ssize_t ftxxxx_debug_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	struct i2c_client *client = (struct i2c_client *)ftxxxx_proc_entry->data;
	unsigned char writebuf[FTS_PACKET_LENGTH];
	int buflen = count;
	int writelen = 0;
	int ret = 0;

	if (copy_from_user(&writebuf, buf, buflen)) {
		dev_err(&client->dev, "%s:copy from user error\n", __func__);
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		{
			char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			dev_dbg(&client->dev, "%s\n", upgrade_file_path);
			disable_irq(client->irq);

			ret = fts_ctpm_fw_upgrade_with_app_file(client, upgrade_file_path);

			enable_irq(client->irq);
			if (ret < 0) {
				dev_err(&client->dev, "%s:upgrade failed.\n", __func__);
				return ret;
			}
		}
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = ftxxxx_i2c_Write(client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = ftxxxx_i2c_Write(client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		dev_dbg(&client->dev, "%s: autoclb\n", __func__);
		fts_ctpm_auto_clb(client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = count - 1;
		ret = ftxxxx_i2c_Write(client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	default:
		break;
	}

	return count;
}

/*interface of read proc*/
static ssize_t ftxxxx_debug_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{

	struct i2c_client *client = (struct i2c_client *)ftxxxx_proc_entry->data;
	int ret = 0;
	//	unsigned char buf[PAGE_SIZE];
	unsigned char *buffer=NULL;
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	buffer = kmalloc(PAGE_SIZE, GFP_KERNEL);

	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		/*after calling ftxxxx_debug_write to upgrade*/
		regaddr = 0xA6;
		ret = ftxxxx_read_reg(client, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buffer, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buffer, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = ftxxxx_i2c_Read(client, NULL, 0, buffer, readlen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:read iic error\n", __func__);
			return ret;
		} 
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		ret = ftxxxx_i2c_Read(client, NULL, 0, buffer, readlen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:read iic error\n", __func__);
			return ret;
		}

		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}

	memcpy(buf, buffer, num_read_chars);
	kfree(buffer);

	return num_read_chars;
}
#else
/*interface of write proc*/
static int ftxxxx_debug_write(struct file *filp, 
	const char __user *buff, unsigned long len, void *data)
{
	struct i2c_client *client = (struct i2c_client *)ftxxxx_proc_entry->data;
	unsigned char writebuf[FTS_PACKET_LENGTH];
	int buflen = len;
	int writelen = 0;
	int ret = 0;

	if (copy_from_user(&writebuf, buff, buflen)) {
		dev_err(&client->dev, "%s:copy from user error\n", __func__);
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		{
			char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			dev_dbg(&client->dev, "%s\n", upgrade_file_path);
			disable_irq(client->irq);

			ret = fts_ctpm_fw_upgrade_with_app_file(client, upgrade_file_path);

			enable_irq(client->irq);
			if (ret < 0) {
				dev_err(&client->dev, "%s:upgrade failed.\n", __func__);
				return ret;
			}
		}
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = ftxxxx_i2c_Write(client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = ftxxxx_i2c_Write(client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		dev_dbg(&client->dev, "%s: autoclb\n", __func__);
		fts_ctpm_auto_clb(client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = len - 1;
		ret = ftxxxx_i2c_Write(client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	default:
		break;
	}

	return len;
}

/*interface of read proc*/
static int ftxxxx_debug_read( char *page, char **start,
	off_t off, int count, int *eof, void *data )
{
	struct i2c_client *client = (struct i2c_client *)ftxxxx_proc_entry->data;
	int ret = 0;
	//	unsigned char buf[PAGE_SIZE];
	unsigned char *buf=NULL;
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	buf = kmalloc(PAGE_SIZE, GFP_KERNEL);

	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		/*after calling ftxxxx_debug_write to upgrade*/
		regaddr = 0xA6;
		ret = ftxxxx_read_reg(client, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = ftxxxx_i2c_Read(client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:read iic error\n", __func__);
			return ret;
		} 
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		ret = ftxxxx_i2c_Read(client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&client->dev, "%s:read iic error\n", __func__);
			return ret;
		}

		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}

	memcpy(page, buf, num_read_chars);
	kfree(buf);

	return num_read_chars;
}
#endif
int ftxxxx_create_apk_debug_channel(struct i2c_client * client)
{
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
	ftxxxx_proc_entry = proc_create(PROC_NAME, 0664, NULL, &ftxxxx_proc_fops);
	ftxxxx_proc_entry->data = client;
#else
	ftxxxx_proc_entry = create_proc_entry(PROC_NAME, 0664, NULL);
#endif
	if (NULL == ftxxxx_proc_entry) {
		dev_err(&client->dev, "Couldn't create proc entry!\n");
		return -ENOMEM;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
	} else {
		dev_info(&client->dev, "Create proc entry success!\n");
		ftxxxx_proc_entry->data = client;
		ftxxxx_proc_entry->write_proc = ftxxxx_debug_write;
		ftxxxx_proc_entry->read_proc = ftxxxx_debug_read;
#endif
	}
	return 0;
}

void ftxxxx_release_apk_debug_channel(void)
{
	if (ftxxxx_proc_entry)
#if (LINUX_VERSION_CODE > KERNEL_VERSION(3, 10, 0))
		proc_remove(PROC_NAME);
#else
		remove_proc_entry(PROC_NAME, NULL);
#endif
}

u8 G_ucVenderID=0;
int  fts_ctpm_fw_upgrade_ReadVendorID(struct i2c_client * client, u8 *ucPVendorID)
{
	u8 reg_val[4] = {0};
	u32 i = 0;
	u8 auc_i2c_write_buf[10];	
	int i_ret;

	*ucPVendorID=0;

	i_ret = HidI2c_To_StdI2c(client);

	if(i_ret == 0)
	{
		dev_err(&client->dev, "HidI2c change to StdI2c fail ! \n");
	}

	fts_get_upgrade_info(&upgradeinfo);

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		/*********Step 1:Reset  CTPM *****/
		ftxxxx_write_reg(client, 0xfc, FT_UPGRADE_AA);
		msleep(upgradeinfo.delay_aa);
		ftxxxx_write_reg(client, 0xfc, FT_UPGRADE_55);
		msleep(200);
		/*********Step 2:Enter upgrade mode *****/
		i_ret = HidI2c_To_StdI2c(client);

		if(i_ret == 0)
		{
			dev_err(&client->dev, "HidI2c change to StdI2c fail ! \n");
			//continue;
		}
		msleep(10);
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		auc_i2c_write_buf[1] = FT_UPGRADE_AA;
		i_ret = ftxxxx_i2c_Write(client, auc_i2c_write_buf, 2);
		if(i_ret < 0)
		{
			dev_err(&client->dev, "failed writing  0x55 and 0xaa ! \n");
			continue;
		}

		/*********Step 3:check READ-ID***********************/
		msleep(10);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;

		reg_val[0] = reg_val[1] = 0x00;

		ftxxxx_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);

		if (reg_val[0] == upgradeinfo.upgrade_id_1
			&& reg_val[1] == upgradeinfo.upgrade_id_2) {
				dev_dbg(&client->dev, "[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
					reg_val[0], reg_val[1]);
				break;
		} else {
			dev_err(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);

			continue;
		}
	}
	if (i >= FTS_UPGRADE_LOOP )
		return -EIO;

	/*********Step 4: read vendor id from app param area***********************/	
	msleep(10);
	auc_i2c_write_buf[0] = 0x03;
	auc_i2c_write_buf[1] = 0x00;
	auc_i2c_write_buf[2] = 0xd7;
	auc_i2c_write_buf[3] = 0x84;

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		ftxxxx_i2c_Write(client, auc_i2c_write_buf, 4);     //send param addr
		msleep(5);
		reg_val[0] = reg_val[1] = 0x00;
		i_ret = ftxxxx_i2c_Read(client, auc_i2c_write_buf, 0, reg_val, 2);

		if(G_ucVenderID != reg_val[0])
		{
		       //*ucPVendorID=0;
			*ucPVendorID=reg_val[0];
			dev_dbg(&client->dev, "In upgrade Vendor ID Mismatch, REG1 = 0x%x, REG2 = 0x%x, Definition:0x%x, i_ret=%d\n",  reg_val[0], reg_val[1], G_ucVenderID, i_ret);
		}
		else
		{
		       *ucPVendorID=reg_val[0];
			dev_dbg(&client->dev, "In upgrade Vendor ID, REG1 = 0x%x, REG2 = 0x%x\n",  reg_val[0],  reg_val[1]);
			break;
		}
	}
	pr_info("[FTS] Vendor ID = 0x%x \n", reg_val[0]);
	msleep(50);
	
	/*********Step 5: reset the new FW***********************/
	dev_dbg(&client->dev, "Step 5: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	ftxxxx_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(200);   //make sure CTP startup normally 
	i_ret = HidI2c_To_StdI2c(client);//Android to Std i2c.

	if(i_ret == 0)
	{
		dev_err(&client->dev, "HidI2c change to StdI2c fail ! \n");
	}
	msleep(10);

	return 0;
}
int  fts_ctpm_fw_upgrade_ReadProjectCode(struct i2c_client * client, char * pProjectCode)
{
	u8 reg_val[4] = {0};
	u32 i = 0;
	u8 j = 0;
	u8 auc_i2c_write_buf[10];	
	int i_ret;

	u32  temp;

	i_ret = HidI2c_To_StdI2c(client);

	if(i_ret == 0)
	{
		dev_err(&client->dev, "HidI2c change to StdI2c fail ! \n");
	}

	fts_get_upgrade_info(&upgradeinfo);

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		/*********Step 1:Reset  CTPM *****/
		ftxxxx_write_reg(client, 0xfc, FT_UPGRADE_AA);
		msleep(upgradeinfo.delay_aa);
		ftxxxx_write_reg(client, 0xfc, FT_UPGRADE_55);
		msleep(200);
		/*********Step 2:Enter upgrade mode *****/
		i_ret = HidI2c_To_StdI2c(client);

		if(i_ret == 0)
		{
			dev_err(&client->dev, "HidI2c change to StdI2c fail ! \n");
			//continue;
		}
		msleep(10);
		auc_i2c_write_buf[0] = FT_UPGRADE_55;
		auc_i2c_write_buf[1] = FT_UPGRADE_AA;
		i_ret = ftxxxx_i2c_Write(client, auc_i2c_write_buf, 2);
		if(i_ret < 0)
		{
			dev_err(&client->dev, "failed writing  0x55 and 0xaa ! \n");
			continue;
		}

		/*********Step 3:check READ-ID***********************/
		msleep(10);
		auc_i2c_write_buf[0] = 0x90;
		auc_i2c_write_buf[1] = auc_i2c_write_buf[2] = auc_i2c_write_buf[3] = 0x00;

		reg_val[0] = reg_val[1] = 0x00;

		ftxxxx_i2c_Read(client, auc_i2c_write_buf, 4, reg_val, 2);

		if (reg_val[0] == upgradeinfo.upgrade_id_1
			&& reg_val[1] == upgradeinfo.upgrade_id_2) {
				dev_dbg(&client->dev, "[FTS] Step 3: READ OK CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
					reg_val[0], reg_val[1]);
				break;
		} else {
			dev_err(&client->dev, "[FTS] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",
				reg_val[0], reg_val[1]);

			continue;
		}
	}
	if (i >= FTS_UPGRADE_LOOP )
		return -EIO;

	/*********Step 4: read vendor id from app param area***********************/	
	msleep(10);
	/*auc_i2c_write_buf[0] = 0x03;
	auc_i2c_write_buf[1] = 0x00;
	auc_i2c_write_buf[2] = 0xd7;
	auc_i2c_write_buf[3] = 0xa0;

	for (i = 0; i < FTS_UPGRADE_LOOP; i++) {
		ftxxxx_i2c_Write(client, auc_i2c_write_buf, 4);     //send param addr
		msleep(5);
		reg_val[0] = reg_val[1] = 0x00;
		i_ret = ftxxxx_i2c_Read(client, auc_i2c_write_buf, 0, reg_val, 4);

		if(G_ucVenderID != reg_val[0])
		{
		       *pProjectCode=0;
			dev_dbg(&client->dev, "In upgrade Vendor ID Mismatch, REG1 = 0x%x, REG2 = 0x%x, Definition:0x%x, i_ret=%d\n",  reg_val[0], reg_val[1], G_ucVenderID, i_ret);
		}
		else
		{
		       *pProjectCode=reg_val[0];
			dev_dbg(&client->dev, "In upgrade Vendor ID, REG1 = 0x%x, REG2 = 0x%x\n",  reg_val[0],  reg_val[1]);
			break;
		}
	}
	*/
	/*read project code*/

	auc_i2c_write_buf[0] = 0x03;
	auc_i2c_write_buf[1] = 0x00;
	for (j=0;j<33;j++)
	{
		//if (is_5336_new_bootloader == BL_VERSION_Z7 || is_5336_new_bootloader == BL_VERSION_GZF)
			//temp = 0x07d0 + j;
		//else
		temp = 0xD7A0 + j;
		auc_i2c_write_buf[2] = (u8)(temp>>8);
		auc_i2c_write_buf[3] = (u8)temp;

		ftxxxx_i2c_Write(client, auc_i2c_write_buf, 4);     //send param addr
		msleep(5);
		ftxxxx_i2c_Read(client, auc_i2c_write_buf, 0, pProjectCode+j, 1);

		if (*(pProjectCode+j) == '\0')
			break;
	}
	pr_info("[FTS] Project code = %s \n", pProjectCode);
	msleep(50);
	
	/*********Step 5: reset the new FW***********************/
	dev_dbg(&client->dev, "Step 5: reset the new FW\n");
	auc_i2c_write_buf[0] = 0x07;
	ftxxxx_i2c_Write(client, auc_i2c_write_buf, 1);
	msleep(200);   //make sure CTP startup normally 
	i_ret = HidI2c_To_StdI2c(client);//Android to Std i2c.

	if(i_ret == 0)
	{
		dev_err(&client->dev, "HidI2c change to StdI2c fail ! \n");
	}
	msleep(10);

	return 0;
}

