/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#if 0
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>

#include <linux/platform_device.h>

#include <linux/HWVersion.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#endif

#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/m10mo_workaround.h>
#include <asm/intel-mid.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-device.h>
#include <linux/m10mo.h>
#include <linux/HWVersion.h>
#include <asm/intel-mid.h>

int io3730_check_EC_status(u8 *buf);
int io3730_get_fw_vertion(u8 *buf);
u8  version_Ascii2Hex(u8 c);

u32 ene_fw_version = 0;

#define IO3730_FW_NAME  "IO3730_fw.bin"
#define FW_SIZE         (11*1024)


#undef DBG_LOG
#define DBG_LOG(fmt, args...) pr_info(fmt, ##args)


// SMB command definition
#define CMD_SET_ADR     0x00
#define CMD_READ_BYTE   0x81
#define CMD_READ_WORD   0x82
#define CMD_READ_BLOCK  0x80
#define CMD_WRITE_BYTE  0x01
#define CMD_WRITE_WORD  0x02
#define CMD_WRITE_BLOCK 0x03

// ISP command definition
#define M10MO_ENE_ROUTER_CATEGORY   0x04
    #define M10MO_ENE_ROUTER_SLAVE_ADDR_H   0x78
    #define M10MO_ENE_ROUTER_SLAVE_ADDR_L   0x79
    #define M10MO_ENE_ROUTER_LENGTH         0x6E

    #define M10MO_ENE_ROUTER_DATA1          0x78
    #define M10MO_ENE_ROUTER_DATA2          0x79
    #define M10MO_ENE_ROUTER_DATA3          0x7A
    #define M10MO_ENE_ROUTER_DATA4          0x7B
    #define M10MO_ENE_ROUTER_DATA5          0x7C
    #define M10MO_ENE_ROUTER_DATA6          0x7D
    #define M10MO_ENE_ROUTER_DATA7          0x7E
    #define M10MO_ENE_ROUTER_DATA8          0x7F

    #define M10MO_ENE_ROUTER_STATUS         0x6F
        #define M10MO_ENE_REASON_OK                 0x00
        #define M10MO_ENE_REASON_NG                 0xFF
        #define M10MO_ENE_REASON_NONE               0x01
        #define M10MO_ENE_REASON_REJECT             0x02

#define M10MO_ENE_CMD_CATEGORY      0x0D
    #define M10MO_ENE_ROUTER_TRIGGER_CMD    0x7F
        #define M10MO_ENE_WRITE_BLOCK               0x03
        #define M10MO_ENE_READ_BLOCK                0x80
        #define M10MO_ENE_SET_ADDRESS               0x00
        #define M10MO_ENE_READ_ADDRESS              0x11


static int io3730_i2c_set_adr(u16 reg)
{
    int rc;
    u32 val;

    rc = m10mo_write_fac(0x02, M10MO_ENE_ROUTER_CATEGORY, M10MO_ENE_ROUTER_SLAVE_ADDR_H, reg);
    rc = m10mo_write_fac(0x01, M10MO_ENE_CMD_CATEGORY, M10MO_ENE_ROUTER_TRIGGER_CMD, M10MO_ENE_SET_ADDRESS);

    rc = m10mo_read_fac(0x01, M10MO_ENE_ROUTER_CATEGORY, M10MO_ENE_ROUTER_STATUS, &val);
    if (M10MO_ENE_REASON_OK == val)
    {
 	    //printk("io3730 set reg:0x%x\n", reg);
    }
    else
    {
 	    DBG_LOG("io3730 set reg fail:0x%x, 0x%x\n", reg, val);
    }

    return 0;
}

static int io3730_i2c_read_regs_i2c_block(u16 start_reg, u8 *buf, int byte_cnt)
{
    int bytes_this_read,bytes_idx;
    int bytes_left = byte_cnt;
    int reg_addr = start_reg;
    int rc = 0;
    u32 val;
    u32 i2c_read_data;

    while(bytes_left)
    {
        // there are max 4 bytes for data access.
        bytes_this_read = min(4, bytes_left);

        rc = io3730_i2c_set_adr(reg_addr);
        // read 4 byte
        rc = m10mo_write_fac(0x01,M10MO_ENE_ROUTER_CATEGORY,M10MO_ENE_ROUTER_LENGTH,4); 
        rc = m10mo_write_fac(0x01, M10MO_ENE_CMD_CATEGORY, M10MO_ENE_ROUTER_TRIGGER_CMD, M10MO_ENE_READ_BLOCK);
        rc = m10mo_read_fac(0x01, M10MO_ENE_ROUTER_CATEGORY,M10MO_ENE_ROUTER_STATUS,&val);
        if (M10MO_ENE_REASON_OK == val)
        {
            rc = m10mo_read_fac(0x04,M10MO_ENE_ROUTER_CATEGORY,M10MO_ENE_ROUTER_DATA1, &i2c_read_data);
    
     	    //printk("io3730 data:0x%x", i2c_read_data);
        }
        else
        {
    	    DBG_LOG("io3730 data fail rc:0x%x  val:0x%x\n", rc, val);
			return -EFAULT;
        }

        for (bytes_idx = 0; bytes_idx < bytes_this_read; bytes_idx++)
        {
            buf[bytes_idx] = (u8)(i2c_read_data >> 24);
            i2c_read_data <<= 8;
        }


        bytes_left -= bytes_this_read;
        buf += bytes_this_read;
        reg_addr += bytes_this_read;

    }

    return rc;
}

static int io3730_i2c_write_regs(u16 start_reg, u8 *buf, int byte_cnt)
{
    int bytes_this_write;
    int bytes_left = byte_cnt;
    int reg_addr = start_reg;
    u32 i2c_write_data;
    u32 val;
    int rc;

    while (bytes_left)
    {
        bytes_this_write = min(8, bytes_left); // we can do block write for max 8 bytes.
        rc = io3730_i2c_set_adr(reg_addr);

        if ( 4 == bytes_this_write)
        {
            i2c_write_data = (u32)buf[0] << 24 | (u32)buf[1] << 16 | (u32)buf[2] << 8 | (u32)buf[3];
            rc = m10mo_write_fac(0x04,M10MO_ENE_ROUTER_CATEGORY,M10MO_ENE_ROUTER_DATA1,i2c_write_data);

            rc = m10mo_write_fac(0x01,M10MO_ENE_ROUTER_CATEGORY,M10MO_ENE_ROUTER_LENGTH,4); 
            rc = m10mo_write_fac(0x01, M10MO_ENE_CMD_CATEGORY, M10MO_ENE_ROUTER_TRIGGER_CMD, M10MO_ENE_WRITE_BLOCK);
            rc = m10mo_read_fac(0x01, M10MO_ENE_ROUTER_CATEGORY,M10MO_ENE_ROUTER_STATUS,&val);
            if (M10MO_ENE_REASON_OK == val)
            {
         	    //printk("io3730 set data reg:0x%x, data: 0x%x", reg_addr, i2c_write_data);
            }
            else
            {
        	    DBG_LOG("io3730 set data fail rc:0x%x  val:0x%x\n", rc, val);
            }
        }
        else if( 7 == bytes_this_write || 8 == bytes_this_write)
        {
            i2c_write_data = (u32)buf[0] << 24 | (u32)buf[1] << 16 | (u32)buf[2] << 8 | (u32)buf[3];
            rc = m10mo_write_fac(0x04, M10MO_ENE_ROUTER_CATEGORY, M10MO_ENE_ROUTER_DATA1, i2c_write_data);

            i2c_write_data = (u32)buf[4] << 24 | (u32)buf[5] << 16 | (u32)buf[6] << 8 | (u32)buf[7];
            rc = m10mo_write_fac(0x04, M10MO_ENE_ROUTER_CATEGORY, M10MO_ENE_ROUTER_DATA5, i2c_write_data);

            rc = m10mo_write_fac(0x01,M10MO_ENE_ROUTER_CATEGORY,M10MO_ENE_ROUTER_LENGTH,bytes_this_write); 
            rc = m10mo_write_fac(0x01, M10MO_ENE_CMD_CATEGORY, M10MO_ENE_ROUTER_TRIGGER_CMD, M10MO_ENE_WRITE_BLOCK);
            rc = m10mo_read_fac(0x01, M10MO_ENE_ROUTER_CATEGORY,M10MO_ENE_ROUTER_STATUS,&val);
            if (M10MO_ENE_REASON_OK == val)
            {
         	    //printk("io3730 set data reg:0x%x, data: 0x%x", reg_addr, i2c_write_data);
            }
            else
            {
        	    DBG_LOG("io3730 set data fail rc:0x%x  val:0x%x\n", rc, val);
            }
        }
        else
        {
		    printk("io3730 do not support this data lengh %d\n", bytes_this_write);
            return -EIO;
        }

        bytes_left -= bytes_this_write;
        buf += bytes_this_write;
        reg_addr += bytes_this_write;
    }

    return 0;
}

static int io3730_set_byte_cmd(u16 start_reg, u8 cmd)
{
    int rc;
    u32 val;

    rc = io3730_i2c_set_adr(start_reg);

    rc = m10mo_write_fac(0x01, M10MO_ENE_ROUTER_CATEGORY, M10MO_ENE_ROUTER_DATA1, (u32)cmd);
    rc = m10mo_write_fac(0x01,M10MO_ENE_ROUTER_CATEGORY,M10MO_ENE_ROUTER_LENGTH,1); 
    rc = m10mo_write_fac(0x01, M10MO_ENE_CMD_CATEGORY, M10MO_ENE_ROUTER_TRIGGER_CMD, M10MO_ENE_WRITE_BLOCK);
    rc = m10mo_read_fac(0x01, M10MO_ENE_ROUTER_CATEGORY,M10MO_ENE_ROUTER_STATUS,&val);
    if (M10MO_ENE_REASON_OK == val)
    {
 	    printk("io3730 set cmd reg:0x%x, cmd: 0x%x", start_reg, cmd);
    }
    else
    {
	    DBG_LOG("io3730 set cmd fail rc:0x%x  val:0x%x\n", rc, val);
    }

    return rc;
}

static int io3730_set_program_cmd(u16 start_reg, u8 cmd, u32 data_reg)
{
    u8 i2cdata[9]={0};
    u16 cmd_reg;
    int rc;

    cmd_reg = start_reg + 7;
    io3730_set_byte_cmd(cmd_reg, cmd);

	i2cdata[0]= (u8)(data_reg >> 16);
	i2cdata[1]= (u8)(data_reg >> 8);
	i2cdata[2]= (u8)(data_reg & 0x00FF);
	i2cdata[3]= 0x00;
	i2cdata[4]= 0x00; 
	i2cdata[5]= 0x80; // always 128 bytes
	i2cdata[6]= 0x00; // status

	rc = io3730_i2c_write_regs(start_reg, i2cdata, 7);
	if(rc)
	{
		printk("io3730 set flash data address fail 0x%04x, %d\n", cmd_reg, rc);
	}
    return rc;
}

int io3730_check_cmd_ready(void)
{
    int i = 1;
    u8 i2cdata[8]={0};
    int rc;

	while(i<10)
	{
		rc = io3730_i2c_read_regs_i2c_block(0x80F6, i2cdata, 1);
    	if(rc)
    	{
    		printk("io3730 cmd fail!!! %d\n", rc);
    	}
		if(i2cdata[0] == 0x02) // cmd OK
		{
		    printk("io3730 cmd OK!!!\n");
			return 0;
		}
        else
        {
            if ( i >= 10)
            {
		        printk("io3730 cmd not ready: %d", i2cdata[0]);
            }
        }
		msleep(20);
		i++;
	}

    return i;
}


int io3730_fw_update_flow(const u8 *data)
{
    #define MAX_RETRY_TIME      10
    u8 i2cdata[128]={0};
    int rc;
    int i, j, Retry_i = 0;
    u32 data_reg;
	int img_offset = 0;


	i = 0;
Re_entry_maskRom:
	// 1. stop watchdog
    rc = io3730_set_byte_cmd(0xFE80, 0x01);
	if(rc)
	{
		printk("io3730 step 1 fail %d\n", rc);
	}
    rc = io3730_check_cmd_ready();
	if(rc)
    {
        printk("io3730 cmd not ready: %d\n ", __LINE__);
    }

    // 2. Set Update Flag
    printk("\nMicroP updateMicroPFirmwareTest: Set Update Flag\n");
    rc = io3730_set_byte_cmd(0xF012, 0x01);
	if(rc)
	{
		printk("io3730 step 2 fail %d\n", rc);
	}
    rc = io3730_check_cmd_ready();
	if(rc)
    {
        printk("io3730 cmd not ready: %d\n ", __LINE__);
    }

	// 3. Switch 8051 Fetch Code
    rc = io3730_set_byte_cmd(0xF010, 0x01);
	if(rc)
	{
		printk("io3730 step 3 fail %d\n", rc);
	}
    rc = io3730_check_cmd_ready();
	if(rc)
    {
        printk("io3730 cmd not ready: %d\n ", __LINE__);
    }

	// 4. Set CODE_SEL[0]=0 for XBI IDLE
    rc = io3730_set_byte_cmd(0xF011, 0x00);
	if(rc)
	{
		printk("io3730 step 4 fail %d\n", rc);
	}
    rc = io3730_check_cmd_ready();
	if(rc)
    {
        printk("io3730 cmd not ready: %d\n ", __LINE__);
    }

	// 5. Set E51_RST = 0 to re-start 8051
    rc = io3730_set_byte_cmd(0xF010, 0x00);
	if(rc)
	{
		printk("io3730 step 5 fail %d\n", rc);
	}
    rc = io3730_check_cmd_ready();
	if(rc)
    {
        printk("io3730 cmd not ready: %d\n ", __LINE__);
    }

	// 6. check io3730 in maskROM
    msleep(50);
	i2cdata[0]=0x00;
	rc = io3730_i2c_read_regs_i2c_block(0xF012, i2cdata, 1);
	if(rc)
	{
		printk("io3730 step 6 fail %d\n", rc);
	}
	if((i2cdata[0] & 0x02) == 0x02)
	{
        printk("io3730 step 6 good");
	}
    else
    {
        if ( ++i >= MAX_RETRY_TIME)
        {
	        printk("io3730 step 6 fail");
            goto error;
        }
        goto Re_entry_maskRom;
    }
    rc = io3730_check_cmd_ready();
	if(rc)
    {
        printk("io3730 cmd not ready: %d\n ", __LINE__);
    }

    #if 0 // earse all
    rc = io3730_set_byte_cmd(0x80F7, 0x10);
    rc = io3730_set_byte_cmd(0x80F6, 0x00);
    printk("io3730 earse all: %d\n ", __LINE__);
    #endif

    // 7. earse & program flash
    #define ENE_FLASH_OFFSET  0 
    data_reg = 0 + ENE_FLASH_OFFSET;
    img_offset = 0;
    while( data_reg < FW_SIZE + ENE_FLASH_OFFSET)
    {
Retry_Programming:
        printk("io3730 step 7 data_reg:0x%x\n", data_reg);

#if 1
        // 7.1 earse flash
        rc = io3730_set_program_cmd(0x80F0, 0x11, data_reg);
    	if(rc)
        {
    		printk("io3730 step 7.1 fail %d\n", rc);
    	}
        rc = io3730_check_cmd_ready();
    	if(rc)
        {
            printk("io3730 cmd not ready: %d\n ", __LINE__);
        }

        // 7.2 set data
        rc = io3730_i2c_write_regs(0x8000, (u8*)(data+img_offset), 128);
    	if(rc)
    	{
    		printk("io3730 step 7.2 fail %d\n", rc);
    	}
        rc = io3730_check_cmd_ready();
    	if(rc)
        {
            printk("io3730 cmd not ready: %d\n ", __LINE__);
        }

        // 7.3 programming flash
        rc = io3730_set_program_cmd(0x80F0, 0x20, data_reg);
    	if(rc)
    	{
    		printk("io3730 step 7.3 fail %d\n", rc);
    	}
        rc = io3730_check_cmd_ready();
    	if(rc)
        {
            printk("io3730 cmd not ready: %d\n ", __LINE__);
        }
#endif
        // 7.4 read flash
        rc = io3730_set_program_cmd(0x80F0, 0x30, data_reg);
    	if(rc)
        {
    		printk("io3730 step 7.4 fail %d\n", rc);
    	}
        rc = io3730_check_cmd_ready();
    	if(rc)
        {
            printk("io3730 cmd not ready: %d\n ", __LINE__);
        }
		rc = io3730_i2c_read_regs_i2c_block(0x8000, i2cdata, 128);
    	if(rc)
        {
    		printk("io3730 step 7.4 read flash fail %d\n", rc);
    	}

        // 7.5 verify flash
        for (i = 0; i < 128; i++)
        {
            if ( *(data+img_offset+i) != i2cdata[i])
            {
                printk("io3730 image data:\n");
                for (j = 0; j < 128/8; j++)
                {
                    printk("io3730 data[%02x]: %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x\n", j
                                                                                                , *(data+img_offset+j*8+0)
                                                                                                , *(data+img_offset+j*8+1)
                                                                                                , *(data+img_offset+j*8+2)
                                                                                                , *(data+img_offset+j*8+3)
                                                                                                , *(data+img_offset+j*8+4)
                                                                                                , *(data+img_offset+j*8+5)
                                                                                                , *(data+img_offset+j*8+6)
                                                                                                , *(data+img_offset+j*8+7));
                }

                printk("io3730 flash data:\n");
                for (j = 0; j < 128/8; j++)
                {
                    printk("io3730 data[%02x]: %02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x\n", j
                                                                                                , i2cdata[j*8+0]
                                                                                                , i2cdata[j*8+1]
                                                                                                , i2cdata[j*8+2]
                                                                                                , i2cdata[j*8+3]
                                                                                                , i2cdata[j*8+4]
                                                                                                , i2cdata[j*8+5]
                                                                                                , i2cdata[j*8+6]
                                                                                                , i2cdata[j*8+7]);
                }

                if ( ++Retry_i >= MAX_RETRY_TIME)
                {
                    printk("io3730 Programming is failed\n");
                    goto error;
                }
                goto Retry_Programming;
            }
            else
            {
                Retry_i = 0;
            }
        }

        // next address
		img_offset += 128;
        data_reg += 128;
    }

    // 8. finish
    rc = io3730_set_byte_cmd(0x80F7, 0x80);
	if(rc)
	{
		printk("io3730 step 8.1 fail %d\n", rc);
	}
    rc = io3730_set_byte_cmd(0x80F6, 0x00);
	if(rc)
	{
		printk("io3730 step 8.2 fail %d\n", rc);
	}
    rc = io3730_check_cmd_ready();
	if(rc)
    {
        printk("io3730 cmd not ready: %d\n ", __LINE__);
    }

	// 9. check io3730 in status
	i2cdata[0]=0x00;
	rc = io3730_i2c_read_regs_i2c_block(0xF012, i2cdata, 1);
	if(rc)
	{
		printk("io3730 0xF012 fail: %d\n", rc);
	}

	printk("io3730 step 0xF012: %d\n", i2cdata[0]);


	i2cdata[0]=0x00;
	rc = io3730_i2c_read_regs_i2c_block(0xF011, i2cdata, 1);
	if(rc)
	{
		printk("io3730 0xF011 fail: %d\n", rc);
	}

	printk("io3730 step 0xF011: %d\n", i2cdata[0]);


    printk("io3730 flash update OK!!!\n");
    return 0;

error:
    printk("io3730 flash update fail!!!\n");
    return -1;
}


int io3730_auto_fw_update(struct m10mo_device *m10mo_dev)
{
	struct v4l2_subdev *sd = &m10mo_dev->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
    const struct firmware *fw;
    int err;
    int rc;
    const u8 *fw_ptr;
    u8 i2cdata[16];
    int EC_status; 
    int timeout = 0;
    u32 val;

	switch (Read_PROJ_ID()) {
		case PROJ_ID_ZX550ML:
			printk("io3730 auto update start\n"); 
			break;
		default:
			printk("io3730 auto update is not support this platform\n");
			return 0;
			break;
	}

    err = request_firmware(&fw, IO3730_FW_NAME, &client->dev);
    if (err)
    {
        printk("io3730 request_firmware is fail\n");
        goto error;
    }
    fw_ptr = fw->data;
    printk("io3730 image version: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", *(fw_ptr+0x1000),
                                                                        *(fw_ptr+0x1001),
                                                                        *(fw_ptr+0x1002),
                                                                        *(fw_ptr+0x1003),
                                                                        *(fw_ptr+0x1004),
                                                                        *(fw_ptr+0x1005),
                                                                        *(fw_ptr+0x1006));

    m10mo_s_power_fac(1);


    msleep(200); // wait ENE ready
    while (timeout < 30)
    {
        timeout++;
        rc = m10mo_read_fac(0x01, 0x04, 0x0A, &val);
        if (!rc)
        {
            if ((val & 0x03) == 0x03)
            {
                printk("io3730 is ready %d\n", val);            
                break;
            }
        }
        if (timeout >= 30)
        {
            printk("io3730 ENE has some issue.\n");
        }
        msleep(10);
    }

    m10mo_write_fac(0x01, 0x04, 0x09, 0x00); // stop ISP access io3730 .
    msleep(30);

    // check EC status
	i2cdata[0]=0x00;
	rc = io3730_check_EC_status(i2cdata);
    if (rc)
    {
        printk("io3730 status fail %d\n", rc);
        goto error;
    }
    else
    {
        EC_status = i2cdata[0];
        printk("io3730 status %d\n", EC_status);
    }

    rc =io3730_get_fw_vertion(i2cdata);
    if (rc)
    {
        printk("io3730 ISP has some issue.\n");
        ene_fw_version = 0;
        goto error;
    }

    #if 1
    // EC waitting maskROM & version is not correct
    if ( !( EC_status & 0x01)
        || version_Ascii2Hex(i2cdata[ 9]) != *(fw_ptr+0x1004)
        || version_Ascii2Hex(i2cdata[10]) != *(fw_ptr+0x1005)
        || version_Ascii2Hex(i2cdata[11]) != *(fw_ptr+0x1006) )
    #else
    // EC waitting maskROM
    if ( !( EC_status & 0x01))
    #endif
    {
        printk("io3730 auto update.\n");
        rc = io3730_fw_update_flow(fw_ptr);
        if (!rc){
            ene_fw_version = *(fw_ptr+0x1004) << 8 
                            | *(fw_ptr+0x1005) << 4 
                            | *(fw_ptr+0x1006);
        }else{
            ene_fw_version = 0;
        }
        printk("io3730 version: V%03x\n", ene_fw_version);
    }
    else
    {      
        printk("io3730 F/W as OK");
    }

    m10mo_s_power_fac(0);
    release_firmware(fw);
    return 0;

error:
    m10mo_s_power_fac(0);
    release_firmware(fw);
    return -EFAULT;
}
EXPORT_SYMBOL(io3730_auto_fw_update);

u8  version_Ascii2Hex(u8 c)
{
    if (c > 0x60) c -= 0x20; // ToUpper

    return c > '9' ? c - 'A' + 10 : c - '0';
}

int io3730_get_fw_vertion(u8 *buf)
{
    int rc;

	rc = io3730_i2c_read_regs_i2c_block(0x831D, buf, 12);
	if(rc)
    {
		printk("io3730 version get fail %d\n", rc);
        goto error;
	}

    printk("io3730 flash version: 0x%x 0x%x 0x%x \n",   buf[9],
                                                        buf[10],
                                                        buf[11]);

    ene_fw_version = version_Ascii2Hex(buf[9]) << 8 
                    | version_Ascii2Hex(buf[10]) << 4 
                    | version_Ascii2Hex(buf[11]);


    printk("io3730 flash version: %s\n", (char*)buf);

    return 0;

error:
    return -EFAULT;
}

int io3730_check_EC_status(u8 *buf)
{
    int rc;

	rc = io3730_i2c_read_regs_i2c_block(0xF011, buf, 1);

    return rc;
}
