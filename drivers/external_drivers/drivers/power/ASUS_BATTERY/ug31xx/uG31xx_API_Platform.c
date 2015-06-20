/**
 * @filename  uG31xx_API_Platform.c
 *
 *  Source of function required by uG31xx driver
 *
 * @author  AllenTeng <allen_teng@upi-semi.com>
 * @note
 */

#include "stdafx.h"     //windows need this??
#include "uG31xx_Platform.h"
#include "uG31xx_API_Platform.h"
#include <asm/intel_scu_ipc.h>

#define BATT_DATA_DEBUG
#define BACKUP_BATTERY_KEY      0xBB
#define UMIP_REF_FG_TBL         0x806   /* 2 bytes */
#define BATT_FG_TBL_BODY        14      /* 144 bytes */
#define BATT_UG31_FG_TBL_BODY   0x820
#define BATT_UG31_RESERVED      2   /* 1st byte: which battery cell.
2nd byte: had backup before */

/**
 * ug31xx_restore_config_data - restore config data
 * @name : Power Supply name
 * @data : config data output pointer
 * @len : length of config data
 *
 */
int ug31xx_restore_config_data(const char *name, u8 *data, int len)
{
        int mip_offset, ret;
        int i;

        pr_info("%s:\n", __func__);

        /* Read the fuel gauge config data from umip */
        mip_offset = BATT_UG31_FG_TBL_BODY + BATT_UG31_RESERVED;
        //mip_offset = UMIP_REF_FG_TBL + BATT_FG_TBL_BODY;
        ret = intel_scu_ipc_read_mip(data, len, mip_offset, 0);

        if (ret) {
                UG31_LOGE("%s: * umip read failed *\n", name);
        }
#ifdef BATT_DATA_DEBUG
        else {
                for (i=0; i<len; i++)
                        printk("0x%02X ", *(data + i));
                printk("\n");
        }
#endif

        return ret;
}

/**
 * ug31xx_save_config_data - save config data
 * @name : Power Supply name
 * @data : config data input pointer
 * @len : length of config data
 *
 */
int ug31xx_save_config_data(const char *name, u8 *data, int len)
{
        int mip_offset, ret;
        int i;

        pr_info("%s:\n", __func__);

        /* write the fuel gauge config data to umip */
        mip_offset = BATT_UG31_FG_TBL_BODY + BATT_UG31_RESERVED;
        //mip_offset = UMIP_REF_FG_TBL + BATT_FG_TBL_BODY;
        ret = intel_scu_ipc_write_umip(data, len, mip_offset);

        if (ret) {
                UG31_LOGE("%s: * umip write failed *\n", name);
        }
#ifdef BATT_DATA_DEBUG
        else {
                for (i=0; i<len; i++)
                        printk("0x%02X ", *(data + i));
                printk("\n");
        }
#endif

        return ret;
}

int ug31xx_read_backup_tag(const char *name, u8 *data)
{
        int mip_offset, ret;

        pr_debug("%s:\n", __func__);

        mip_offset = BATT_UG31_FG_TBL_BODY;
        ret = intel_scu_ipc_read_mip(data, 1, mip_offset, 0);

        if (ret)
                UG31_LOGE("%s: * umip read failed *\n", name);

        return ret;
}

int ug31xx_write_backup_tag(const char *name, u8 *data)
{
        int mip_offset, ret;

        pr_warn("%s:\n", __func__);

        mip_offset = BATT_UG31_FG_TBL_BODY;
        ret = intel_scu_ipc_write_umip(data, 1, mip_offset);

        if (ret)
                UG31_LOGE("%s: * umip write failed *\n", name);

        return ret;
}

#ifdef  uG31xx_OS_WINDOWS

typedef char mm_segment_t;
typedef char loff_t;

typedef struct file {
        FILE *fp;
} fileType;

#define O_RDONLY  (1<<0)
#define O_WRONLY  (1<<1)
#define O_RDWR    (3<<0)
#define O_CREAT   (1<<2)
#define O_APPEND  (1<<3)

static struct file BackupFile;

/**
   * @brief filp_open
   *
   *  Open file
   *
   * @para  path  address of file
   * @para  cntl  FILE_CNTL parameter
   * @para  misc  dummy parameter
   * @return  address of BackupFile
   */
struct file * filp_open(const wchar_t *path, int cntl, int misc) {
        if(BackupFile.fp != _UPI_NULL_) {
                fclose(BackupFile.fp);
        }
        BackupFile.fp = _UPI_NULL_;

        switch(cntl) {
        case  (O_RDONLY):
                _wfopen_s(&BackupFile.fp, path, _T("rb, ccs=UTF-8"));
                break;
        case  (O_CREAT | O_RDWR):
                _wfopen_s(&BackupFile.fp, path, _T("w+b, ccs=UTF-8"));
                break;
        case  (O_RDWR):
                _wfopen_s(&BackupFile.fp, path, _T("r+b, ccs=UTF-8"));
                break;
        case  (O_CREAT | O_APPEND | O_WRONLY):
                _wfopen_s(&BackupFile.fp, path, _T("a+b, ccs=UTF-8"));
                break;
        default:
                _wfopen_s(&BackupFile.fp, path, _T("rb, ccs=UTF-8"));
                break;
        }
        return (&BackupFile);
}

/**
   * @brief filp_close
   *
   *  Close file
   *
   * @para  fp  address of struct file
   * @para  misc  dummy value
   * @return  _UPI_NULL_
   */
void filp_close(struct file *fp, int misc)
{
        if(fp->fp != _UPI_NULL_) {
                fclose(fp->fp);
        }
}

/**
   * @brief vfs_write
   *
   *  Write data to binary file
   *
   * @para  fp  address of struct file
   * @para  data  address of data to be written
   * @para  size  size to be written
   * @para  pos start position in the file
   * @return  size be written
   */
size_t vfs_write(struct file *fp, char *data, int size, loff_t *pos)
{
        return (fwrite(data, sizeof(char), size, fp->fp));
}

/**
   * @brief vfs_read
   *
   *  Read data from binary file
   *
   * @para  fp  address of struct file
   * @para  data  address of data to be read
   * @para  size  size to be read
   * @para  pos start position in the file
   * @return  size be read
   */
size_t vfs_read(struct file *fp, char *data, int size, loff_t *pos)
{
        return (fread(data, sizeof(char), size, fp->fp));
}

/**
   * @brief get_fs
   *
   *  Dummy function
   *
   * @return  0
   */
mm_segment_t get_fs(void)
{
        return (0);
}

/**
   * @brief set_fs
   *
   *  Dummy function
   *
   * @para  value mm_segment_t value
   * @return  _UPI_NULL_
   */
void set_fs(mm_segment_t value)
{
}

/**
   * @brief get_ds
   *
   *  Dummy function
   *
   * @return  0
   */
mm_segment_t get_ds(void)
{
        return (0);
}

#endif  ///< end of uG31xx_OS_WINDOWS

/**
   * @brief IS_ERR
   *
   *  Check file is opened or not
   *
   * @para  fp  address of struct file
   * @return  _UPI_TRUE_ if file is opened
   */
#if defined(uG31xx_BOOT_LOADER) || defined(ANDROID_SHELL_ALGORITHM)

#define is_err

#else   ///< else of defined(uG31xx_BOOT_LOADER) || defined(ANDROID_SHELL_ALGORITHM)

_upi_bool_ is_err(struct file *fp)
{
#ifdef  uG31xx_OS_WINDOWS

        return ((fp->fp == _UPI_NULL_) ? _UPI_TRUE_ : _UPI_FALSE_);

#else   ///< else of uG31xx_OS_WINDOWS

        return (IS_ERR(fp) ? _UPI_TRUE_ : _UPI_FALSE_);

#endif  ///< end of uG31xx_OS_WINDOWS
}

#endif  ///< end of defined(uG31xx_BOOT_LOADER) || defined(ANDROID_SHELL_ALGORITHM)

#ifndef uG31xx_OS_WINDOWS

char *shell_ap_name = NULL;

/**
 * @brief set_shell_ap_name
 *
 *  Set shell AP name
 *
 * @para  apname  address of AP name
 * @return  NULL
 */
void set_shell_ap_name(char *apname)
{
        shell_ap_name = apname;
}

static _upi_u8_ file_op_status = 0;

/**
 * @brief get_file_op_status
 *
 *  Get file_op_status
 *
 * @return  file_op_status
 */
_upi_u8_ get_file_op_status(void)
{
        return (file_op_status);
}

/**
 * @brief set_file_op_status_bit
 *
 *  Set bit of file_op_status
 *
 * @para  bit_sts bit of file_op_sts to be set
 * @return  file_op_status
 */
_upi_u8_ set_file_op_status_bit(_upi_u8_ bit_sts)
{
        file_op_status = file_op_status | bit_sts;
        return (file_op_status);
}

/**
 * @brief clear_file_op_status_bit
 *
 *  Clear bit of file_op_status
 *
 * @para  bit_sts bit of file_op_sts to be set
 * @return  file_op_status
 */
_upi_u8_ clear_file_op_status_bit(_upi_u8_ bit_sts)
{
        file_op_status = file_op_status & (~bit_sts);
        return (file_op_status);
}

#endif  ///< end of uG31xx_OS_WINDOWS

/**
 * @brief is_file_exist
 *
 *  Check file is existed or not
 *
 * @para  filename  address of filename string
 * @return  _UPI_TRUE_ if file is existed
 */
#ifdef  uG31xx_OS_WINDOWS
_upi_bool_ is_file_exist(const wchar_t *filename)
#else   ///< else of uG31xx_OS_WINDOWS
_upi_bool_ is_file_exist(char *filename)
#endif  ///< end of uG31xx_OS_WINDOWS
{
#ifndef uG31xx_BOOT_LOADER

#ifndef CONFIG_ASUS_ENGINEER_MODE

#ifdef  UG31XX_USE_SHELL_AP_FOR_FILE_OP

        struct subprocess_info *sub_info;
        char *argv[] = {shell_ap_name, "BACKUP_FILE", "EXIST", filename, NULL};
        char *env[] = {NULL};
        int rtn;

        sub_info = NULL;
        sub_info = call_usermodehelper_setup(argv[0], argv, env, GFP_ATOMIC);
        if(sub_info == NULL) {
                return (_UPI_FALSE_);
        }
        UG31_LOGN("[%s]: call_usermodehelper_setup() done (%d)\n", __func__, (int)sub_info);

        rtn = call_usermodehelper_exec(sub_info, UMH_WAIT_PROC);
        UG31_LOGN("[%s]: call_usermodehelper_exec() = %d\n", __func__, rtn);
        return ((rtn == 0) ? _UPI_TRUE_ : _UPI_FALSE_);

#else   ///< else of UG31XX_USE_SHELL_AP_FOR_FILE_OP

#ifdef  UG31XX_USE_DAEMON_AP_FOR_FILE_OP

        if(get_file_op_status() & UG31XX_KERNEL_FILE_FINISH) {
                set_file_op_status_bit(UG31XX_KERNEL_FILE_EXIST);
                return (_UPI_FALSE_);
        }

        if(get_file_op_status() & UG31XX_USER_FILE_EXIST) {
                clear_file_op_status_bit(UG31XX_KERNEL_FILE_EXIST);
                set_file_op_status_bit(UG31XX_KERNEL_FILE_READ);
                set_file_op_status_bit(UG31XX_KERNEL_FILE_FINISH);
                return (_UPI_TRUE_);
        }

        set_file_op_status_bit(UG31XX_KERNEL_FILE_EXIST);
        set_file_op_status_bit(UG31XX_KERNEL_FILE_FINISH);
        return (_UPI_FALSE_);

#else   ///< else of UG31XX_USE_DAEMON_AP_FOR_FILE_OP

        struct file *fp;
        _upi_u8_ retry;

        retry = 3;
        while(retry) {
                fp = filp_open(filename, O_RDONLY, 0644);
                if(!is_err(fp)) {
                        break;
                }

                retry = retry - 1;
        }
        if(retry == 0) {
                return (_UPI_FALSE_);
        }

        filp_close(fp, _UPI_NULL_);

#endif  ///< end of UG31XX_USE_DAEMON_AP_FOR_FILE_OP

#endif  ///< end of UG31XX_USE_SHELL_AP_FOR_FILE_OP

#else
        _upi_u8_ backup_tag = 0;
        if (ug31xx_read_backup_tag("ug31xx", &backup_tag))
                return (_UPI_FALSE_);
        if (backup_tag != BACKUP_BATTERY_KEY)
                return (_UPI_FALSE_);
        return (_UPI_TRUE_);
#endif

#endif  ///< end of uG31xx_BOOT_LOADER

        return (_UPI_TRUE_);
}

/**
 * @brief write_file
 *
 *  Write data to file
 *
 * @para  fp  address of struct fp
 * @para  data  address of data buffer to be written
 * @para  size  size of data buffer
 */
#if defined(uG31xx_BOOT_LOADER) || defined(ANDROID_SHELL_ALGORITHM)

#define write_file

#else   ///< else of defined(uG31xx_BOOT_LOADER) || defined(ANDROID_SHELL_ALGORITHM)

void write_file(struct file *fp, _upi_u8_ *data, _upi_u8_ size)
{
        mm_segment_t oldFS;
        loff_t pos;
        _upi_s32_ rtn;
        _upi_u8_ idx;

        oldFS = get_fs();
        set_fs(get_ds());

        pos = 0;
        idx = 0;
        rtn = 1;
        UG31_LOGN("[%s] Write file ->", __func__);
        while(idx < size) {
                rtn = (_upi_s32_)vfs_write(fp, (char *)(&data[idx]), 1, &pos);
                ug31_printk(LOG_LEVEL_NOTICE, " %02x", data[idx]);

                idx = idx + 1;
                if(rtn != 1) {
                        break;
                }
        }
        ug31_printk(LOG_LEVEL_NOTICE, "\n");

        if(rtn != 1) {
                UG31_LOGE("[%s]: Write file fail\n", __func__);
        } else {
                UG31_LOGN("[%s]: Write %d (%d) bytes to file\n", __func__, idx, size);
        }

        set_fs(oldFS);
}

#endif  ///< end of defined(uG31xx_BOOT_LOADER) || defined(ANDROID_SHELL_ALGORITHM)

/**
 * @brief create_backup_file
 *
 *  Create backup file on system
 *
 * @para  filename  address of filename string
 * @return  _UPI_TRUE_ if success
 */
#ifdef  uG31xx_OS_WINDOWS
_upi_bool_ create_backup_file(const wchar_t *filename, _upi_u8_ *data, _upi_u8_ size)
#else   ///< else of uG31xx_OS_WINDOWS
_upi_bool_ create_backup_file(char *filename, _upi_u8_ *data, _upi_u8_ size)
#endif  ///< end of uG31xx_OS_WINDOWS
{
#ifndef uG31xx_BOOT_LOADER

#ifndef CONFIG_ASUS_ENGINEER_MODE

#ifdef  UG31XX_USE_SHELL_AP_FOR_FILE_OP

        struct subprocess_info *sub_info;
        char *argv[] = {shell_ap_name, "BACKUP_FILE", "CREATE", filename, NULL};
        char *env[] = {NULL};
        int rtn;

        sub_info = NULL;
        sub_info = call_usermodehelper_setup(argv[0], argv, env, GFP_ATOMIC);
        if(sub_info == NULL) {
                return (_UPI_FALSE_);
        }
        UG31_LOGN("[%s]: call_usermodehelper_setup() done (%d - %d - %d)\n", __func__, (int)sub_info, (int)data, (int)size);

        rtn = call_usermodehelper_exec(sub_info, UMH_WAIT_PROC);
        UG31_LOGN("[%s]: call_usermodehelper_exec() = %d\n", __func__, rtn);
        return ((rtn == 0) ? _UPI_TRUE_ : _UPI_FALSE_);

#else   ///< else of UG31XX_USE_SHELL_AP_FOR_FILE_OP

#ifdef  UG31XX_USE_DAEMON_AP_FOR_FILE_OP

        if(get_file_op_status() & UG31XX_KERNEL_FILE_FINISH) {
                clear_file_op_status_bit(UG31XX_KERNEL_FILE_EXIST);
                clear_file_op_status_bit(UG31XX_KERNEL_FILE_READ);
                set_file_op_status_bit(UG31XX_KERNEL_FILE_WRITE);
                return (_UPI_FALSE_);
        }

        if(get_file_op_status() & UG31XX_USER_FILE_WRITE) {
                clear_file_op_status_bit(UG31XX_KERNEL_FILE_WRITE);
                set_file_op_status_bit(UG31XX_KERNEL_FILE_READ);
                set_file_op_status_bit(UG31XX_KERNEL_FILE_FINISH);
                return (_UPI_TRUE_);
        }

        clear_file_op_status_bit(UG31XX_KERNEL_FILE_EXIST);
        clear_file_op_status_bit(UG31XX_KERNEL_FILE_READ);
        set_file_op_status_bit(UG31XX_KERNEL_FILE_WRITE);
        set_file_op_status_bit(UG31XX_KERNEL_FILE_FINISH);
        return (_UPI_FALSE_);

#else   ///< else of UG31XX_USE_DAEMON_AP_FOR_FILE_OP

        struct file *fp;
        _upi_u8_ retry;

        retry = 3;
        while(retry) {
                fp = filp_open(filename, O_CREAT | O_RDWR, 0644);
                if(!is_err(fp)) {
                        break;
                }

                retry = retry - 1;
        }
        if(retry == 0) {
                return (_UPI_FALSE_);
        }

        /// [AT-PM] : Write data to file ; 02/21/2013
        write_file(fp, data, size);

        filp_close(fp, _UPI_NULL_);

#endif  ///< end of UG31XX_USE_DAEMON_AP_FOR_FILE_OP

#endif  ///< end of UG31XX_USE_SHELL_AP_FOR_FILE_OP

#else
        int backup_tag = BACKUP_BATTERY_KEY;
        if (ug31xx_save_config_data("ug31xx", data, size)) {
                UG31_LOGE("[%s]: fail to write Intel UMIP data\n", __func__);
                return (_UPI_FALSE_);
        } else {
                if (ug31xx_write_backup_tag("ug31xx", &backup_tag)) {
                        UG31_LOGE("[%s]: fail to write Intel UMIP backup tag\n", __func__);
                        return (_UPI_FALSE_);
                }
        }
        return (_UPI_TRUE_);
#endif

#endif  ///< end of uG31xx_BOOT_LOADER

        return (_UPI_TRUE_);
}

#ifdef  uG31xx_NO_MEM_UNIT

#define MEMORY_BUFFER_COUNT     (0x10)
#define MEMORY_BUFFER_SIZE      (0x400)

static _upi_bool_ memory_idx[] = {  _UPI_FALSE_,  _UPI_FALSE_,  _UPI_FALSE_,  _UPI_FALSE_,
                                    _UPI_FALSE_,  _UPI_FALSE_,  _UPI_FALSE_,  _UPI_FALSE_,
                                    _UPI_FALSE_,  _UPI_FALSE_,  _UPI_FALSE_,  _UPI_FALSE_,
                                    _UPI_FALSE_,  _UPI_FALSE_,  _UPI_FALSE_,  _UPI_FALSE_,
                                 };
static _upi_u8_ memory_buffer[MEMORY_BUFFER_COUNT][MEMORY_BUFFER_SIZE];

#endif  ///< end of uG31xx_NO_MEM_UNIT

/**
 * @brief upi_free
 *
 *  Free memory for uG31xx driver
 *
 * @para  obj address of memory
 * @return  NULL
 */
void upi_free(void *obj)
{
#if defined(uG31xx_OS_ANDROID)
#if defined(uG31xx_BOOT_LOADER) || defined(ANDROID_SHELL_ALGORITHM)
#ifdef  uG31xx_NO_MEM_UNIT
        _upi_u8_ idx;

        idx = 0;
        while(idx < MEMORY_BUFFER_COUNT) {
                if(obj == &memory_buffer[idx][0]) {
                        memory_idx[idx] = _UPI_FALSE_;
                        break;
                }
                idx = idx + 1;
        }
#else   ///< else of uG31xx_NO_MEM_UNIT
        free(obj);
#endif  ///< end of uG31xx_NO_MEM_UNIT
#else   ///< else of defined(uG31xx_BOOT_LOADER) || defined(ANDROID_SHELL_ALGORITHM)
        kfree(obj);
#endif  ///< end of defined(uG31xx_BOOT_LOADER) || defined(ANDROID_SHELL_ALGORITHM)
#else   ///< else of defined(uG31xx_OS_ANDROID)
        free(obj);
#endif  ///< end of defined(uG31xx_OS_ANDROID)

        obj = _UPI_NULL_;
}

/**
 * @brief upi_malloc
 *
 *  Allocate memory for uG31xx driver
 *
 * @para  size  size of memory
 * @return  address of memory
 */
void *upi_malloc(_upi_u32_ size)
{
#if defined(uG31xx_OS_ANDROID)
#if defined(uG31xx_BOOT_LOADER) || defined(ANDROID_SHELL_ALGORITHM)
#ifdef  uG31xx_NO_MEM_UNIT
        _upi_u8_ idx;

        idx = 0;
        while(idx < MEMORY_BUFFER_COUNT) {
                if(memory_idx[idx] == _UPI_FALSE_) {
                        memory_idx[idx] = _UPI_TRUE_;
                        return ((void *)&memory_buffer[idx][0]);
                }
                idx = idx + 1;
        }
        return (_UPI_NULL_);
#else   ///< else of uG31xx_NO_MEM_UNIT
        return (malloc(size));
#endif  ///< end of uG31xx_NO_MEM_UNIT
#else   ///< else of defined(uG31xx_BOOT_LOADER) || defined(ANDROID_SHELL_ALGORITHM)
        return (kzalloc(size, GFP_KERNEL));
#endif  ///< end of defined(uG31xx_BOOT_LOADER) || defined(ANDROID_SHELL_ALGORITHM)
#else   ///< else of defined(uG31xx_OS_ANDROID)
        return (malloc(size));
#endif  ///< end of defined(uG31xx_OS_ANDROID)
}

/**
 * @brief upi_memcpy
 *
 *  Copy memory for uG31xx driver
 *
 * @para  dest  address of destination
 * @para  src address of source
 * @para  size  size of data to be copied
 * @return  NULL
 */
void upi_memcpy(void *dest, void *src, _upi_u32_ size)
{
#ifdef  uG31xx_NO_MEM_UNIT
        _upi_u64_ idx;
        _upi_u8_ *ptr_dest;
        _upi_u8_ *ptr_src;

        idx = 0;
        ptr_dest = (_upi_u8_ *)dest;
        ptr_src = (_upi_u8_ *)src;
        while(idx < size) {
                *(ptr_dest + idx) = *(ptr_src + idx);
                idx = idx + 1;
        }
#else   ///< else of uG31xx_NO_MEM_UNIT
        memcpy(dest, src, size);
#endif  ///< end of uG31xx_NO_MEM_UNIT
}

/**
 * @brief read_file
 *
 *  Read data from file
 *
 * @para  data  address of BackupDataType
 * @para  fp  address of struct fp
 */
#if defined(uG31xx_BOOT_LOADER) || defined(ANDROID_SHELL_ALGORITHM)

#define read_file

#else   ///< else of defined(uG31xx_BOOT_LOADER) || defined(ANDROID_SHELL_ALGORITHM)

void read_file(struct file *fp, _upi_u8_ *data, _upi_u8_ size)
{
        mm_segment_t oldFS;
        loff_t pos;
        _upi_s32_ rtn;
        _upi_u8_ idx;

        oldFS = get_fs();
        set_fs(get_ds());

        pos = 0;
        idx = 0;
        rtn = 1;
        UG31_LOGN("[%s]: Read file ->", __func__);
        while(idx < size) {
                rtn = (_upi_s32_)vfs_read(fp, (char *)(&data[idx]), 1, &pos);
                ug31_printk(LOG_LEVEL_NOTICE, " %02x", data[idx]);

                idx = idx + 1;
                if(rtn != 1) {
                        break;
                }
        }
        ug31_printk(LOG_LEVEL_NOTICE, "\n");

        if(rtn != 1) {
                UG31_LOGE("[%s]: Read file fail.\n", __func__);
        } else {
                UG31_LOGN("[%s]: Read %d (%d) bytes from file\n", __func__, idx, size);
        }

        set_fs(oldFS);
}

#endif  ///< end of defined(uG31xx_BOOT_LOADER) || defined(ANDROID_SHELL_ALGORITHM)

/**
 * @brief read_backup_file
 *
 *  Read data from backup file
 *
 * @para  filename  address of backup filename string
 * @para  data  address of data buffer
 * @para  size  size of data to be read
 * @return  _UPI_TRUE_ if success
 */
#ifdef  uG31xx_OS_WINDOWS
extern _upi_bool_ read_backup_file(const wchar_t *filename, _upi_u8_ *data, _upi_u32_ size)
#else   ///< else of uG31xx_OS_WINDOWS
extern _upi_bool_ read_backup_file(char *filename, _upi_u8_ *data, _upi_u32_ size)
#endif  ///< end of uG31xx_OS_WINDOWS
{
#ifndef uG31xx_BOOT_LOADER

#ifndef CONFIG_ASUS_ENGINEER_MODE

#ifdef  UG31XX_USE_SHELL_AP_FOR_FILE_OP

        struct subprocess_info *sub_info;
        char *argv[] = {shell_ap_name, "BACKUP_FILE", "READ", filename, NULL};
        char *env[] = {NULL};
        int rtn;

        sub_info = NULL;
        sub_info = call_usermodehelper_setup(argv[0], argv, env, GFP_ATOMIC);
        if(sub_info == NULL) {
                return (_UPI_FALSE_);
        }
        UG31_LOGN("[%s]: call_usermodehelper_setup() done (%d - %d - %d)\n", __func__, (int)sub_info, (int)data, (int)size);

        rtn = call_usermodehelper_exec(sub_info, UMH_WAIT_PROC);
        UG31_LOGN("[%s]: call_usermodehelper_exec() = %d\n", __func__, rtn);
        return ((rtn == 0) ? _UPI_TRUE_ : _UPI_FALSE_);

#else   ///< else of UG31XX_USE_SHELL_AP_FOR_FILE_OP

#ifdef  UG31XX_USE_DAEMON_AP_FOR_FILE_OP

        if(get_file_op_status() & UG31XX_KERNEL_FILE_FINISH) {
                clear_file_op_status_bit(UG31XX_KERNEL_FILE_EXIST);
                clear_file_op_status_bit(UG31XX_KERNEL_FILE_WRITE);
                set_file_op_status_bit(UG31XX_KERNEL_FILE_READ);
                return (_UPI_FALSE_);
        }

        if(get_file_op_status() & UG31XX_USER_FILE_READ) {
                clear_file_op_status_bit(UG31XX_KERNEL_FILE_EXIST);
                clear_file_op_status_bit(UG31XX_KERNEL_FILE_WRITE);
                set_file_op_status_bit(UG31XX_KERNEL_FILE_READ);
                set_file_op_status_bit(UG31XX_KERNEL_FILE_FINISH);
                return (_UPI_TRUE_);
        }

        clear_file_op_status_bit(UG31XX_KERNEL_FILE_EXIST);
        clear_file_op_status_bit(UG31XX_KERNEL_FILE_WRITE);
        set_file_op_status_bit(UG31XX_KERNEL_FILE_READ);
        set_file_op_status_bit(UG31XX_KERNEL_FILE_FINISH);
        return (_UPI_FALSE_);

#else   ///< else of UG31XX_USE_DAEMON_AP_FOR_FILE_OP

        struct file *fp;
        _upi_u8_ retry;

        retry = 3;
        while(retry) {
                fp = filp_open(filename, O_RDWR, 0644);
                if(!is_err(fp)) {
                        break;
                }

                retry = retry - 1;
        }
        if(retry == 0) {
                return (_UPI_FALSE_);
        }

        /// [AT-PM] : Write data to file ; 02/21/2013
        read_file(fp, data, size);

        filp_close(fp, _UPI_NULL_);

#endif  ///< end of UG31XX_USE_DAEMON_AP_FOR_FILE_OP

#endif  ///< end of UG31XX_USE_SHELL_AP_FOR_FILE_OP

#else
        if (ug31xx_restore_config_data("ug31xx", data, size)) {
                UG31_LOGE("[%s]: fail to read Intel UMIP data\n", __func__);
                return (_UPI_FALSE_);
        }
        return (_UPI_TRUE_);
#endif

#endif  ///< end of uG31xx_BOOT_LOADER

        return (_UPI_TRUE_);
}

/**
 * @brief write_backup_file
 *
 *  Write data to backup file
 *
 * @para  filename  address of backup filename string
 * @para  data  address of data buffer
 * @para  size  size of data to be written
 * @return  _UPI_TRUE_ if success
 */
#ifdef  uG31xx_OS_WINDOWS
extern _upi_bool_ write_backup_file(const wchar_t *filename, _upi_u8_ *data, _upi_u32_ size)
#else   ///< else of uG31xx_OS_WINDOWS
extern _upi_bool_ write_backup_file(char *filename, _upi_u8_ *data, _upi_u32_ size)
#endif  ///< end of uG31xx_OS_WINDOWS
{
#ifndef uG31xx_BOOT_LOADER

#ifndef CONFIG_ASUS_ENGINEER_MODE

#ifdef  UG31XX_USE_SHELL_AP_FOR_FILE_OP

        struct subprocess_info *sub_info;
        char *argv[] = {shell_ap_name, "BACKUP_FILE", "WRITE", filename, NULL};
        char *env[] = {NULL};
        int rtn;

        sub_info = NULL;
        sub_info = call_usermodehelper_setup(argv[0], argv, env, GFP_ATOMIC);
        if(sub_info == NULL) {
                return (_UPI_FALSE_);
        }
        UG31_LOGN("[%s]: call_usermodehelper_setup() done (%d - %d - %d)\n", __func__, (int)sub_info, (int)data, (int)size);

        rtn = call_usermodehelper_exec(sub_info, UMH_WAIT_PROC);
        UG31_LOGN("[%s]: call_usermodehelper_exec() = %d\n", __func__, rtn);
        return ((rtn == 0) ? _UPI_TRUE_ : _UPI_FALSE_);

#else   ///< else of UG31XX_USE_SHELL_AP_FOR_FILE_OP

#ifdef  UG31XX_USE_DAEMON_AP_FOR_FILE_OP

        if(get_file_op_status() & UG31XX_KERNEL_FILE_FINISH) {
                clear_file_op_status_bit(UG31XX_KERNEL_FILE_EXIST);
                clear_file_op_status_bit(UG31XX_KERNEL_FILE_READ);
                set_file_op_status_bit(UG31XX_KERNEL_FILE_WRITE);
                return (_UPI_FALSE_);
        }

        if(get_file_op_status() & UG31XX_USER_FILE_WRITE) {
                clear_file_op_status_bit(UG31XX_KERNEL_FILE_EXIST);
                clear_file_op_status_bit(UG31XX_KERNEL_FILE_WRITE);
                set_file_op_status_bit(UG31XX_KERNEL_FILE_READ);
                set_file_op_status_bit(UG31XX_KERNEL_FILE_FINISH);
                return (_UPI_TRUE_);
        }

        clear_file_op_status_bit(UG31XX_KERNEL_FILE_EXIST);
        clear_file_op_status_bit(UG31XX_KERNEL_FILE_READ);
        set_file_op_status_bit(UG31XX_KERNEL_FILE_WRITE);
        set_file_op_status_bit(UG31XX_KERNEL_FILE_FINISH);
        return (_UPI_FALSE_);

#else   ///< else of UG31XX_USE_DAEMON_AP_FOR_FILE_OP

        struct file *fp;
        _upi_u8_ retry;

        retry = 3;
        while(retry) {
                fp = filp_open(filename, O_CREAT | O_RDWR, 0644);
                if(!is_err(fp)) {
                        break;
                }

                retry = retry - 1;
        }
        if(retry == 0) {
                return (_UPI_FALSE_);
        }

        /// [AT-PM] : Write data to file ; 02/21/2013
        write_file(fp, data, size);

        filp_close(fp, _UPI_NULL_);

#endif  ///< end of UG31XX_USE_DAEMON_AP_FOR_FILE_OP

#endif  ///< end of UG31XX_USE_SHELL_AP_FOR_FILE_OP

#else
        int backup_tag = BACKUP_BATTERY_KEY;
        if (ug31xx_save_config_data("ug31xx", data, size)) {
                UG31_LOGE("[%s]: fail to write Intel UMIP data\n", __func__);
                return (_UPI_FALSE_);
        }
        return (_UPI_TRUE_);
#endif

#endif  ///< end of uG31xx_BOOT_LOADER

        return (_UPI_TRUE_);
}

/**
 * @brief upi_memcmp
 *
 *  Memory compare for uG31xx driver
 *
 * @para  s1  address of memory 1
 * @para  s2  address of memory 2
 * @para  size  size to be compared
 * @return  0 if the same
 */
_upi_u32_ upi_memcmp(void *s1, void *s2, _upi_u32_ size)
{
#ifdef  uG31xx_NO_MEM_UNIT
        _upi_u64_ idx;
        _upi_u8_ *ptr_s1;
        _upi_u8_ *ptr_s2;

        idx = 0;
        ptr_s1 = (_upi_u8_ *)s1;
        ptr_s2 = (_upi_u8_ *)s2;
        while(idx < size) {
                if(*(ptr_s1 + idx) != *(ptr_s2 + idx)) {
                        return (1);
                }
                idx = idx + 1;
        }
        return (0);
#else   ///< else of uG31xx_NO_MEM_UNIT
        return ((_upi_u32_)memcmp(s1, s2, size));
#endif  ///< end of uG31xx_NO_MEM_UNIT
}

#if defined(uG31xx_OS_ANDROID)

/**
 * @brief GetTickCount
 *
 *  Get system time tick
 *
 * @return  time tick
 */
_upi_u32_ GetTickCount(void)
{
#if defined(uG31xx_BOOT_LOADER) || defined(ANDROID_SHELL_ALGORITHM)

        return (0);

#else   ///< else of defined(uG31xx_BOOT_LOADER) || defined(ANDROID_SHELL_ALGORITHM)

        return jiffies_to_msecs(jiffies);      //20121121/jacky

#endif  ///< end of defined(uG31xx_BOOT_LOADER) || defined(ANDROID_SHELL_ALGORITHM)
}

/**
 * @brief GetSysTickCount
 *
 *  Get system time
 *
 * @return  system time in millisecond
 */
_upi_u32_ GetSysTickCount(void)
{
#if defined(uG31xx_BOOT_LOADER) || defined(ANDROID_SHELL_ALGORITHM)

        return (0);

#else   ///< else of defined(uG31xx_BOOT_LOADER) || defined(ANDROID_SHELL_ALGORITHM)

        struct timeval current_tick;

        do_gettimeofday(&current_tick);

        return current_tick.tv_sec * 1000 + current_tick.tv_usec/1000;

#endif  ///< end of defined(uG31xx_BOOT_LOADER) || defined(ANDROID_SHELL_ALGORITHM)
}

#endif  ///< end of defined(uG31xx_OS_ANDROID)

/**
 * @brief upi_memset
 *
 *  Set memory initial value for uG31xx driver
 *
 * @para  ptr address of memory
 * @para  value initial value
 * @para  size  size to be set
 * @return  NULL
 */
void upi_memset(void *ptr, _upi_u8_ value, _upi_u32_ size)
{
#ifdef  uG31xx_NO_MEM_UNIT
        _upi_u64_ idx;
        _upi_u8_ *ptr_ptr;

        idx = 0;
        ptr_ptr = (_upi_u8_ *)ptr;
        while(idx < size) {
                *(ptr_ptr + idx) = value;
                idx = idx + 1;
        }
#else   ///< else of uG31xx_NO_MEM_UNIT
        memset(ptr, value, size);
#endif  ///< end of uG31xx_NO_MEM_UNIT
}

_upi_u8_ Ug31DebugEnable = LOG_LEVEL_ERROR;

#ifdef  uG31xx_OS_WINDOWS

unsigned int debugViewLines = 0;
CString debugViewFileName = _T("uG3100-1");

#else   ///< else of uG31xx_OS_WINDOWS

#ifndef  uG31xx_BOOT_LOADER

/**
 * @brief ug31_printk
 *
 *  Print debug message
 *
 * @para  level message level
 * @para  fmt message
 * @return  integer
 */
int ug31_printk(int level, const char *fmt, ...)
{
#ifdef  UG31XX_LIB_DEBUG_MSG

        va_list args;
        int r;

        r = 0;
        if(level <= Ug31DebugEnable) {
                va_start(args, fmt);
                r = vprintk(fmt, args);
                va_end(args);
        }

        return (r);

#else   ///< else of UG31XX_LIB_DEBUG_MSG

        return (0);

#endif  ///< end of UG31XX_LIB_DEBUG_MSG
}

#endif  ///< end of uG31xx_BOOT_LOADER

#endif  ///< end of uG31xx_OS_WINDOWS

#ifndef uG31xx_OS_WINDOWS

/**
 * @brief upi_strlen
 *
 *  Get string length for uG31xx driver
 *
 * @para  stream  address of string
 * @return  string length
 */
_upi_u32_ upi_strlen(char *stream)
{
#ifdef  uG31xx_BOOT_LOADER

        return (0);

#else   ///< else of uG31xx_BOOT_LOADER

        return ((_upi_u32_)strlen(stream));

#endif  ///< end of uG31xx_BOOT_LOADER
}

#endif  ///< end of uG31xx_OS_WINDOWS

#ifndef uG31xx_OS_WINDOWS

#if !defined(uG31xx_BOOT_LOADER) && !defined(ANDROID_SHELL_ALGORITHM)

#define SECURITY_KEY    (0x5A)    //i2c read/write 
#define ONE_BYTE        (0x1)
#define TWO_BYTE        (0x0)

static struct i2c_client *ug31xx_client = _UPI_NULL_;

void ug31xx_i2c_client_set(struct i2c_client *client)
{
        ug31xx_client = client;
        dev_info(&ug31xx_client->dev, "%s: Ug31xx i2c client saved.\n", __func__);
}

_upi_s32_ ug31xx_read_i2c(struct i2c_client *client, _upi_u8_ reg, _upi_s32_ *rt_value, _upi_s32_ b_single)
{
        struct i2c_msg msg[2];
        _upi_u8_ data[4];
        _upi_s32_ err;

        if((!client) || (!client->adapter)) {
                return -ENODEV;
        }

        if(!rt_value) {
                return -EINVAL;
        }

        data[0] = reg;

        msg[0].addr = client->addr;
        msg[0].flags = 0 | I2C_M_NOSTART;
        msg[0].len = 1;
        if(reg >= 0x80) {
                data[1] = SECURITY_KEY;
                msg[0].len++;
        }
        msg[0].buf = (unsigned char *)data;

        msg[1].addr = client->addr;
        msg[1].flags = (I2C_M_RD);
        msg[1].len = b_single ? 1 : 2;
        msg[1].buf = (unsigned char *)data;

        err = i2c_transfer(client->adapter, msg, sizeof(msg)/sizeof(struct i2c_msg));

        if(err < 0) {
                return err;
        }

        if(b_single) {
                *rt_value = (_upi_s32_)data[0];
        } else {
                *rt_value = (_upi_s32_)get_unaligned_le16(data);
        }

        return 0;
}

_upi_s32_ ug31xx_write_i2c(struct i2c_client *client, _upi_u8_ reg, _upi_s32_ rt_value, _upi_s32_ b_single)
{
        struct i2c_msg msg[1];
        _upi_u8_ data[4];
        _upi_s32_ err;
        _upi_s32_ idx;
        _upi_s32_ tmp_buf=0;

        if((!client) || (!client->adapter)) {
                return -ENODEV;
        }

        idx = 0;
        data[idx++] = reg;
        if (reg >= 0x80) {
                data[idx++] = SECURITY_KEY;
        }
        data[idx++] = (_upi_u8_)(rt_value & 0x0FF);
        data[idx++] = (_upi_u8_)((rt_value & 0x0FF00) >> 8);

        msg[0].addr = client->addr;
        msg[0].flags = 0 | I2C_M_NOSTART;
        msg[0].len = b_single ? idx-1 : idx;
        msg[0].buf = (unsigned char *)data;

        err = i2c_transfer(client->adapter, msg, sizeof(msg)/sizeof(struct i2c_msg));

        if(err >= 0) {
                err = ug31xx_read_i2c(client, reg, &tmp_buf, b_single);
        }
        return (err < 0 ? err : 0);
}

_upi_bool_ _API_I2C_Write(_upi_u16_ writeAddress, _upi_u8_ writeLength, _upi_u8_ *PWriteData)
{
        _upi_s32_ i, ret, tmp_buf;
        _upi_s32_ byte_flag=0;

        if (!PWriteData) {
                dev_err(&ug31xx_client->dev, "%s: Write buffer pointer error.\n", __func__);
                return false;
        }

        byte_flag = ONE_BYTE;

        for(i=0; i<writeLength; i++) {
                tmp_buf = PWriteData[i];

                ret = ug31xx_write_i2c(ug31xx_client, (_upi_u8_)(writeAddress + i), tmp_buf, byte_flag);
                if(ret) {
                        dev_err(&ug31xx_client->dev, "%s: Write data(0x%02X) fail. %d\n", __func__, i, ret);
                        return (_UPI_FALSE_);
                }
        }

        return (_UPI_TRUE_);
}

_upi_bool_ _API_I2C_Read(_upi_u16_ readAddress, _upi_u8_ readLength, _upi_u8_ *pReadDataBuffer)
{
        _upi_s32_ i, ret, tmp_buf;
        _upi_s32_ byte_flag = 0;

        if(!pReadDataBuffer) {
                dev_err(&ug31xx_client->dev, "%s: Read buffer pointer error.\n", __func__);
                return false;
        }

        byte_flag = ONE_BYTE;

        for(i=0; i<readLength; i++) {
                tmp_buf = 0;

                ret = ug31xx_read_i2c(ug31xx_client, (_upi_u8_)(readAddress + i), &tmp_buf, byte_flag);
                if (ret) {
                        dev_err(&ug31xx_client->dev, "%s: read data(0x%02X) fail. %d\n", __func__, i, ret);
                        return false;
                }
                pReadDataBuffer[i] = (_upi_u8_)tmp_buf;
        }

        return (_UPI_TRUE_);
}

/**
 * @brief API_I2C_Read
 *
 *  I2C read function for uG31xx driver
 *
 * @para  bSecurityMode useless parameter
 * @para  bHighSpeedMode  useless parameter
 * @para  bTenBitMode useless parameter
 * @para  readAddress register address to be read
 * @para  readLength  length of data to be read
 * @para  pReadDataBuffer address of buffer
 * @return  _UPI_TRUE_ if success
 */
_upi_bool_ API_I2C_Read(_upi_bool_ bSecurityMode, _upi_bool_ bHighSpeedMode, _upi_bool_ bTenBitMode, _upi_u16_ readAddress, _upi_u8_ readLength, _upi_u8_ *pReadDataBuffer)
{
        return (_API_I2C_Read(readAddress, readLength, pReadDataBuffer));
}

/**
 * @brief API_I2C_Write
 *
 *  I2C write function for uG31xx driver
 *
 * @para  bSecurityMode useless parameter
 * @para  bHighSpeedMode  useless parameter
 * @para  bTenBitMode useless parameter
 * @para  writeAddress register address to be written
 * @para  writeLength  length of data to be written
 * @para  pWriteData address of buffer
 * @return  _UPI_TRUE_ if success
 */
_upi_bool_ API_I2C_Write(_upi_bool_ bSecurityMode, _upi_bool_ bHighSpeedMode, _upi_bool_ bTenBitMode, _upi_u16_ writeAddress, _upi_u8_ writeLength, _upi_u8_ *pWriteData)
{
        return (_API_I2C_Write(writeAddress, writeLength, pWriteData));
}

/**
 * @brief API_I2C_Init
 *
 *  Initialize i2c device
 *
 * @para  client  address of i2c_client
 * @return  NULL
 */
void API_I2C_Init(void *client)
{
        ug31xx_i2c_client_set((struct i2c_client *)client);
}

#endif  ///< end of !defined(uG31xx_BOOT_LOADER) && !defined(ANDROID_SHELL_ALGORITHM)

#endif  ///< end of uG31xx_OS_WINDOWS

/**
 * @brief SleepMiniSecond
 *
 *  Sleep for mini-seconds
 *
 * @para  msec  mini-seconds to be sleep
 * @return  NULL
 */
void SleepMiniSecond(_upi_u32_ msec)
{
#ifdef  uG31xx_OS_WINDOWS

        Sleep(msec);

#else   ///< else of uG31xx_OS_WINDOWS

#ifdef  uG31xx_BOOT_LOADER

        volatile int counter = 0;
        volatile int end_counter = 0;
        volatile int ms_counter = 0;

        ms_counter = 1000;
        end_counter = 650 * msec;
        while(ms_counter) {
                counter = 0;
                while(end_counter > counter) {
                        counter ++;
                }
                ms_counter = ms_counter - 1;
        }

#else   ///< else of uG31xx_BOOT_LOADER

        mdelay(msec);

#endif  ///< end of uG31xx_BOOT_LOADER

#endif  ///< end of uG31xx_OS_WINDOWS
}

