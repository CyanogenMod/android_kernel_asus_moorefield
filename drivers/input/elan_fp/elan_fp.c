//#define _MTK_PLATFORM_

#ifndef _MTK_PLATFORM_
#include <linux/module.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/wait.h>

#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/syscalls.h>
#include <linux/err.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/wakelock.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/pm.h>

#include <linux/namei.h>
#include <linux/mount.h>
#include <linux/earlysuspend.h>
#else // -------------------------------------------
#define ALIG_4_BYTE // change by customer
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/ioctl.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/wakelock.h>
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>
#include <linux/spi/spi.h>
#include <linux/io.h>

#include <linux/gpio.h>
#include <linux/kpd.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include "mach/mt_gpio.h"
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpt.h>

#include "cust_gpio_usage.h"
#include "cust_eint.h"
#endif
	
#define VERSION_LOG	"ELAN FINGER PRINT V1.4.3.2"

#ifdef _MTK_PLATFORM_
#define ELAN_FINGERPRINT_SPI 	"Elan_Finger_SPI"
#endif

#ifndef _MTK_PLATFORM_
#define GPIO_EFSA120_IRQ	105//39 // change by customer
#define GPIO_EFSA120_RST	106//35 // change by customer
#else
#define GPIO_EFSA120_IRQ	GPIO_DTV_EINT_PIN 	// change by customer
#define GPIO_EFSA120_RST	GPIO_DTV_RST_PIN 	// change by customer
#endif

#define SPI_MAX_SPEED		12000000//10000000 // chagne by customer
#define _SIGNAL_MODE_		// define = Signal Event, no def = Key mode

#define KEY_FP_INT			KEY_POWER //KEY_WAKEUP 	// change by customer & framework support

static int sig; 			// added v1.43
static pid_t pid; 			// added v1.43
static unsigned char bsig_pid = 0; // added v1.43

//#define CONFIG_PM_SLEEP
//#define _ELAN_DEBUG_
#ifdef _ELAN_DEBUG_
         #define ELAN_DEBUG(dev, format, args ...) 		printk("%s %s: [ELAN]:%5d: " format, dev_driver_string(dev), dev_name(dev), __LINE__, ##args)
#else
         #define ELAN_DEBUG(dev, format, args ...)
#endif

#define _ELAN_DEBUG_DATA_
#ifdef _ELAN_DEBUG_DATA_
         #define ELAN_DEBUG_DATA(dev, format, args ...) 	printk("%s %s: [ELAN]:%5d: " format, dev_driver_string(dev), dev_name(dev), __LINE__, ##args)
#else
         #define ELAN_DEBUG_DATA(dev, format, args ...)
#endif

#define WRITE_REG_HEAD			0x80
#define READ_REG_HEAD			0x40
#define READ_SERIER_REG_HEAD	0xC0
#define INT_NORMAL_HIGH			0x40
#define START_SCAN				0x01
#define START_READ_IMAGE		0x10
#define ADDR_SIZE_WH			0x01


struct efsa120s_data  {
	struct spi_device 	*spi;
	struct input_dev 	*input_dev;
	struct mutex 		spi_mutex;
	struct cdev 		spi_cdev;
	struct class 		*spi_class;
	struct device		*spi_device;

	/* File I/O for user-space */
	struct mutex 		sysfs_mutex;
	struct miscdevice 	efsa120_dev;	/* char device for ioctl */
	
	int 			intr_gpio;
	int 			rst_gpio;
	struct work_struct  work;	
	unsigned char bImageReady;
	spinlock_t irq_lock;
	int irq_is_disable;
	struct early_suspend early_suspend;
};

#define FINGERPRINT_IOCTL				0x80
#define ID_IOCTL_INIT					_IOW(FINGERPRINT_IOCTL, 0,  int) /* To Get Raw Image (14->8)*/
#define ID_IOCTL_READ_REGISTER			_IOW(FINGERPRINT_IOCTL, 2,  int)
#define ID_IOCTL_WRITE_REGISTER			_IOW(FINGERPRINT_IOCTL, 3,  int)
#define ID_IOCTL_RESET					_IOW(FINGERPRINT_IOCTL, 6,  int)
#define ID_IOCTL_GET_RAW_IMAGE			_IOW(FINGERPRINT_IOCTL, 10, int) /* To Get Raw Image (Original)*/
#define ID_IOCTL_STATUS					_IOW(FINGERPRINT_IOCTL, 12, int)
#define ID_IOCTL_SET_AUTO_RAW_IMAGE		_IOW(FINGERPRINT_IOCTL, 13, int)
#define ID_IOCTL_GET_AUTO_RAW_IMAGE		_IOW(FINGERPRINT_IOCTL, 14, int)
#define ID_IOCTL_READ_CMD				_IOW(FINGERPRINT_IOCTL, 15, int) // General read cmd
#define ID_IOCTL_WRITE_CMD				_IOW(FINGERPRINT_IOCTL, 16, int) // General write cmd
#define ID_IOCTL_IOIRQ_STATUS			_IOW(FINGERPRINT_IOCTL, 17, int) // Use INT to read buffer
#define ID_IOCTL_SPI_STATUS				_IOW(FINGERPRINT_IOCTL, 18, int) // UPdate SPI Speed & CS delay
#define ID_IOCTL_SIG_PID				_IOW(FINGERPRINT_IOCTL, 19, int) // WOE signal event to pid


static int major_number = 0; 
static int minor_number = 0; 

static unsigned int IMG_WIDTH = 120;//56
static unsigned int IMG_HEIGHT = 120;//192
static unsigned int IMG_WIDTH_DEFAULT = 120;//56
static unsigned int IMG_HEIGHT_DEFAULT = 120;//192
static unsigned int IMG_SIZE = 14400;

static unsigned short * get_image_buff = NULL;
static unsigned short * image_sub_background_buff  = NULL;
static unsigned short * image_background = NULL;
static unsigned char * image_output = NULL; 	// for 14->8bit data
static unsigned char * imagebuffer = NULL; 	// for SPI Transfer and orginal data


static unsigned char status[5] = {0}; // added by samuel
static int auto_raw_image_num = 0; 	// added by samuel
static int auto_raw_image_count = 0; 	// added by samuel
static unsigned char bCMD_REG = 0; 	// CMD = 0, REG= 1
static int nWaitImage = 100000; 		// wait for max 1ms
static int nWaitImage_Count = 0;
static int Image_index = 0;

static unsigned char IOIRQ_STATUS = 0;
static unsigned int spi_speed = SPI_MAX_SPEED; // 10MHz
static unsigned short spi_cs_delay = 5; 		// 5us

static unsigned char image_data_offset = 16;

static struct workqueue_struct *efsa120s_wq;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void efsa120s_early_suspend(struct early_suspend *h);
static void efsa120s_late_resume(struct early_suspend *h);
static unsigned char bState = 1; // 1=on, 0=off
#endif

static unsigned char bProbeCheck[20] = {0};

#ifdef _MTK_PLATFORM_
//kent
#define CUST_EINT_DTV_NUM				54		// mediatek\custom\lge75_v4_jb\kernel\dct\dct\custo_eint.h
#define CUST_EINT_DTV_DEBOUTCE_CN		0
#define CUST_EINT_DTV_POLARITY			0
#define CUST_EINT_DTV_SENSITIVE			1
#define CUST_EINT_DTV_DEBOUNCE_EN		0
#define GPIO_DVT_EINT_PIN_M_EINT                4

static void mt_efsa120s_irq_handler(void);
static struct efsa120s_data *mt_fp = NULL;

void elan_isdbt_gpio_interrupt_register(void)
{

	// configuration for detect
	printk("<isdbt> elan_isdbt_gpio_interrupt_register, IN [+]\n");
	mt_eint_set_sens(CUST_EINT_DTV_NUM, CUST_EINT_DTV_SENSITIVE);
	mt_eint_set_hw_debounce(CUST_EINT_DTV_NUM, CUST_EINT_DTV_DEBOUTCE_CN);
	//printk("<isdbt>elan_isdbt_gpio_interrupt_register, (%d)[%d]\n",GPIO_DVT_EINT_PIN,CUST_EINT_DTV_NUM);

	// gpio
	//mt_set_gpio_mode(NMI_IRQN_PIN, GPIO_DVT_EINT_PIN_M_EINT);
	//mt_set_gpio_dir(NMI_IRQN_PIN, GPIO_DIR_IN);
	//mt_set_gpio_pull_enable(NMI_IRQN_PIN, false);
	//mt_set_gpio_pull_enable(NMI_IRQN_PIN, GPIO_PULL_ENABLE);
	//mt_set_gpio_pull_select(NMI_IRQN_PIN, GPIO_PULL_UP );

	// irq register
	//mt65xx_eint_registration(CUST_EINT_DTV_NUM, CUST_EINT_DTV_DEBOUNCE_EN, CUST_EINT_DTV_POLARITY, isdbt_irq_handler, 0);
	mt_eint_registration(CUST_EINT_DTV_NUM, EINTF_TRIGGER_RISING, mt_efsa120s_irq_handler, 1);
	// disable irq
	mt_eint_mask(CUST_EINT_DTV_NUM);
	printk("<isdbt> elan_isdbt_gpio_interrupt_register, OUT [-]\n");
	printk("[isdbt] elan_isdbt_gpio_interrupt_register!\n");
}

void efsa120s_gpio_power_on(void)
{
    if(TRUE != hwPowerOn(MT6325_POWER_LDO_VCAM_IO, VOL_1800,ELAN_FINGERPRINT_SPI))
    {
        printk("Fail to enable VDDIO_18 digital power\n");
    }

   //if(TRUE != hwPowerOn(MT6325_POWER_LDO_VGP2, VOL_1200,SONY_HW_DRVNAME))
   //{
   // 	SPIDEV_ERR("Fail to enable VDD_12 digital power\n");
   //}

	hwPowerOn(MT6325_POWER_LDO_VTCXO1, VOL_2800, ELAN_FINGERPRINT_SPI);
	
	// ANT switch as high
	//mt_set_gpio_out(GPIO_ANT_SW_PIN, 1);
}

void efsa120s_gpio_power_off(void)
{
	// VDTV_2V8(RF1628) Vdd disable
    hwPowerDown(MT6325_POWER_LDO_VTCXO1, ELAN_FINGERPRINT_SPI); // ELAN_FINGERPRINT_SPI = ??

	//mt_set_gpio_out(NMI_POWER_PIN, 0);

	// ANT switch as low
 	//mt_set_gpio_out(GPIO_ANT_SW_PIN, 0);

	//if(TRUE != hwPowerDown(NMI_DTV_VDDIO_18, NMI_HW_DRVNAME))
	//{
	//    printk("[ISDBT] Fail to disable VDDIO_18 digital power\n");
	//}

	//if(TRUE != hwPowerDown(MT6325_POWER_LDO_VGP2, SONY_HW_DRVNAME))
	//{
	//	 SPIDEV_ERR("Fail to disable VDD_12 digital power\n");
	//}
}
#endif

static int efsa120s_spi_transfer(struct spi_device *spi, char *txbuf, char *rxbuf, int len)
{
	struct efsa120s_data *fp = spi_get_drvdata(spi);
	struct spi_transfer t;
	struct spi_message m;

	//mutex_lock(&fp->spi_mutex);
	
	memset(&t, 0, sizeof(t));
	spi_message_init(&m);
	//m.spi = spi;
	t.tx_buf = txbuf;
	t.rx_buf = rxbuf;
	t.bits_per_word = 8;
	t.len = len;
	t.speed_hz = spi_speed;
	//t.delay_usecs = spi_cs_delay;
	spi_message_add_tail(&t, &m);

	//mutex_unlock(&fp->spi_mutex);
	
	return spi_sync(fp->spi, &m);
}

static int efsa120s_send_signal_to_pid(void)
{
	struct siginfo info;

	if(bsig_pid == 0)
		return -1;
		
	if(bState == 0) // Display off:0x31
		info.si_signo = sig + 1; // global para	
	else 			// Display on:0x30
		info.si_signo = sig; // global para	
		
	info.si_errno = 0;
	info.si_code = SI_USER; // send by kill, sigsend, raise ?
	info.si_pid = get_current()->pid; // global para
	info.si_uid = current_uid();

	//printk("Send state(%x), sig = %d, pid = %d. =======================================================\r\n", bState, sig, pid);

	return kill_proc_info(info.si_signo, &info, pid);
}

static int efsa120s_kmalloc_image(void)
{
	IMG_SIZE = (IMG_WIDTH * IMG_HEIGHT);

	get_image_buff = kmalloc(sizeof(unsigned short) * IMG_SIZE, GFP_KERNEL);
	if (get_image_buff == NULL)
  		goto km_get_image_buff_fail;

	image_sub_background_buff = kmalloc(sizeof(unsigned short) * IMG_SIZE, GFP_KERNEL);
	if (image_sub_background_buff == NULL)
  		goto km_image_sub_background_buff_fail;

	image_background = kmalloc(sizeof(unsigned short) * IMG_SIZE, GFP_KERNEL);
	if (image_background == NULL)
  		goto km_image_background_fail;

	image_output = kmalloc(sizeof(unsigned char) * IMG_SIZE, GFP_KERNEL);
	if (image_output == NULL)
  		goto km_image_output_fail;

	#ifdef ALIG_4_BYTE
	imagebuffer = kmalloc(sizeof(unsigned char) * (IMG_WIDTH*2+4)*IMG_HEIGHT, GFP_KERNEL); // add 4 = 2 + 2 is for aligment
	#else
	imagebuffer = kmalloc(sizeof(unsigned char) * (IMG_WIDTH*2+2)*IMG_HEIGHT, GFP_KERNEL); // add 2 is cmd & dummy
	#endif
	if (imagebuffer == NULL)
  		goto km_imagebuffer_fail;	

	return 0;

km_imagebuffer_fail:
	kfree(image_output);
km_image_output_fail:
	kfree(image_background);
km_image_background_fail:
	kfree(image_sub_background_buff);
km_image_sub_background_buff_fail:
	kfree(get_image_buff);
km_get_image_buff_fail:
	printk("[ELAN]: efsa120s_kmalloc_image fail. Memory alloc Error.\r\n");
	return -ENOMEM;	
}

static int efsa120s_receive_image(struct spi_device *spi)
{
	int i;
#ifdef ALIG_4_BYTE
	char txbuf[IMG_WIDTH*2+4];
#else
	char txbuf[IMG_WIDTH*2+2];
#endif

	
//SPI Command (Image_Burst_Read)
	txbuf[0] = START_READ_IMAGE;

#ifdef ALIG_4_BYTE //MTK DMA Mode So Used ALIG 4 Bytes
	for(i = 0; i < IMG_HEIGHT; i++){
		ELAN_DEBUG_DATA(&spi->dev,"H2[%d] ", i);
		efsa120s_spi_transfer(spi, txbuf, &(imagebuffer[i*(IMG_WIDTH*2+4)]), IMG_WIDTH*2+2);
	}
#else
	for(i = 0; i < IMG_HEIGHT; i++){
		ELAN_DEBUG_DATA(&spi->dev,"H2[%d] ", i);
		efsa120s_spi_transfer(spi, txbuf, &(imagebuffer[i*(IMG_WIDTH*2+2)]), IMG_WIDTH*2+2);
	}
#endif
	return 0;
}

static int efsa120s_receive_image_1BYte(struct spi_device *spi)
{
	int i;
#ifdef ALIG_4_BYTE
	char txbuf[IMG_WIDTH+4];
#else
	char txbuf[IMG_WIDTH+2];
#endif

//SPI Command (Image_Burst_Read)
	txbuf[0] = START_READ_IMAGE;

#ifdef ALIG_4_BYTE //MTK DMA Mode So Used ALIG 4 Bytes
	for(i = 0; i < IMG_HEIGHT; i++){
		ELAN_DEBUG_DATA(&spi->dev,"H1[%d] ", i);
		efsa120s_spi_transfer(spi, txbuf, &(imagebuffer[i*(IMG_WIDTH+4)]), IMG_WIDTH+2);
	}
#else
	for(i = 0; i < IMG_HEIGHT; i++){
		ELAN_DEBUG_DATA(&spi->dev,"H1[%d] ", i);
		efsa120s_spi_transfer(spi, txbuf, &(imagebuffer[i*(IMG_WIDTH+2)]), IMG_WIDTH+2);
	}
#endif
	return 0;
}

static int efsa120s_fingerprint_init(struct spi_device *spi, unsigned char data[])
{
	//struct efsa120s_data *fp = spi_get_drvdata(spi);
	unsigned char tbl_init_para1[] = {0x00, 0x5A, 0x08, 0x04, 0x0B, 0x71, 0x0C, 0x49, 0x0F, 0x2B, 0x11, 0x2B, 0x13, 0x28, 0x15, 0x28, 0x18, 0x04, 0x21, 0x20, 0x22, 0x36, 0x06, 0xED, 0x05, 0x6F};
	//unsigned char tbl_init_para2[] = {0x2A, 0x03};//AFE Regulator turn on
	unsigned char tbl_init_para2[] = {0x2A, 0x0F};
	unsigned char tbl_init_para3[] = {0x2A, 0x0B};	
	unsigned char tbl_init_para4[] = {0x2A, 0x4B};	//FUNC Enable
	unsigned char tbl_init_para5[] = {0x2C, 0x09};	//INTERRUPT
	char txbuf[4], rxbuf[4];

	int i;

#ifndef _MTK_PLATFORM_
	/* Developement platform */
	gpio_direction_output(GPIO_EFSA120_RST, 0);
	mdelay(5);
	gpio_direction_output(GPIO_EFSA120_RST, 1);
	mdelay(50);
#else
	/* MTK platform */
	mt_set_gpio_pull_select(GPIO145, GPIO_PULL_UP);
	mt_set_gpio_pull_select(GPIO146, GPIO_PULL_UP);
	mt_set_gpio_out(GPIO_EFSA120_RST, 0);
	mdelay(5);
	mt_set_gpio_out(GPIO_EFSA120_RST, 1);
	mdelay(50);
#endif
	
	//SPI Command (Fuse Load)
	txbuf[0] = 0x04;
	efsa120s_spi_transfer(spi ,txbuf, rxbuf, 1);
	ELAN_DEBUG(&spi->dev,"Fuse Load\r\n");
	
	mdelay(1);
	
	for(i=0; i<sizeof(tbl_init_para1); i+=2)
	{
		txbuf[0] = WRITE_REG_HEAD + tbl_init_para1[i]; 	// Regist Address
		txbuf[1] = tbl_init_para1[i+1];		 			// Regist Values
		efsa120s_spi_transfer(spi, txbuf, rxbuf, 2);
	}
	ELAN_DEBUG(&spi->dev,"Write REG Data Value\r\n");
	
	mdelay(1);
	
	for(i=0; i<sizeof(tbl_init_para2); i+=2)
	{
		txbuf[0] = WRITE_REG_HEAD + tbl_init_para2[i];	// Regist Address
		txbuf[1] = tbl_init_para2[i+1];					// Regist Values
		efsa120s_spi_transfer(spi, txbuf, rxbuf, 2);
	}
	ELAN_DEBUG(&spi->dev,"AFE Regulator turn on\r\n");
	
	mdelay(1);
	
	for(i=0; i<sizeof(tbl_init_para3); i+=2)
	{
		txbuf[0] = WRITE_REG_HEAD + tbl_init_para3[i];	// Regist Address
		txbuf[1] = tbl_init_para3[i+1];					// Regist Values
		efsa120s_spi_transfer(spi, txbuf, rxbuf, 2);
	}
	ELAN_DEBUG(&spi->dev,"AFE Regulator turn on\r\n");
	
	mdelay(1);
	
	//FUNC Enable
	for(i=0; i<sizeof(tbl_init_para4); i+=2)
	{
		txbuf[0] = WRITE_REG_HEAD + tbl_init_para4[i];	// Regist Address
		txbuf[1] = tbl_init_para4[i+1];					// Regist Values
		efsa120s_spi_transfer(spi, txbuf, rxbuf, 2);
	}
	ELAN_DEBUG(&spi->dev,"AFE Regulator turn on\r\n");
	
	mdelay(1);
	
	//INT Mode Enable
	tbl_init_para5[1] |= (status[0] & INT_NORMAL_HIGH);
	for(i=0; i<sizeof(tbl_init_para5); i+=2)
	{
		txbuf[0] = WRITE_REG_HEAD + tbl_init_para5[i];	// Regist Address
		txbuf[1] = tbl_init_para5[i+1];					// Regist Values
		efsa120s_spi_transfer(spi, txbuf, rxbuf, 2);	
		
	}
	ELAN_DEBUG(&spi->dev,"INT Mode Enable\r\n");
	
	mdelay(1);
	
	return 0;
}

static int efsa120s_read_register(struct spi_device *spi, unsigned char *RegInfo)
{
	// [0] = length, [1] = register address, [2~length+1] = data
	char *txbuf = NULL;
	char *rxbuf = NULL;

	int i;
	int len = RegInfo[0] + 2;
	
#ifdef ALIG_4_BYTE	
	if((len % 4) != 0)
		len = len / 4 * 4 + 4;
#endif
		
	txbuf = kmalloc(len, GFP_KERNEL); // (+2 = cmd & dummy)
	rxbuf = kmalloc(len, GFP_KERNEL); // (+2 = cmd & dummy)
  	if (txbuf == NULL)
  		return -ENOMEM;
	else if (rxbuf == NULL)
  		return -ENOMEM;
  		
	if(RegInfo[0] < 2 || bCMD_REG == 0) // read with dummy 
	{
		if(bCMD_REG == 1) // 0 == CMD, 1 == REG
    		txbuf[0] = READ_REG_HEAD + RegInfo[1]; // one byte data read (+1 = cmd)
		else if(bCMD_REG == 0) // 0 == CMD, 1 == REG
			txbuf[0] = RegInfo[1]; // one byte data read (+1 = cmd)
			
		efsa120s_spi_transfer(spi, txbuf, rxbuf, RegInfo[0] + 1);
		
		if(RegInfo[0] < 2) // read reg
		{
			RegInfo[2] = rxbuf[1];
			ELAN_DEBUG(&spi->dev, "[ELAN] %s() Read = 0x%02x\r\n", __func__, rxbuf[1]);
		}
		else // read cmd over one byte
		{
			for(i = 0; i < RegInfo[0] - 1; i++)
			{
				RegInfo[i + 2] = rxbuf[i + 2];
				ELAN_DEBUG(&spi->dev, "[ELAN] %s() Read CMD = 0x%02x\r\n", __func__, rxbuf[i + 2]);
			}
		}	
	}
	else
	{
		txbuf[0] = READ_SERIER_REG_HEAD + RegInfo[1]; // mutli-byte read (+2 = cmd & dummy)
    	efsa120s_spi_transfer(spi, txbuf, rxbuf, RegInfo[0] + 2);
		
		for(i = 0; i < RegInfo[0]; i++)
			RegInfo[i + 2] = rxbuf[i + 2];

		ELAN_DEBUG(&spi->dev, "[ELAN] %s() Read = ", __func__);

		for(i = 0; i < RegInfo[0]; i++)
			ELAN_DEBUG(&spi->dev, "0x%02x ", rxbuf[i + 2]);

		ELAN_DEBUG(&spi->dev, "\r\n");
	}
  	kfree(rxbuf);
	kfree(txbuf);
	return 0;
}

static int efsa120s_write_register(struct spi_device *spi, unsigned char *RegInfo)
{
	// [0] = length, [1] = register address, [2~length+1] = data
	char *txbuf = NULL;
	int i;
	
	int len = RegInfo[0] + 1;
	
#ifdef ALIG_4_BYTE	
	if((len % 4) != 0)
		len = len / 4 * 4 + 4;
#endif

	txbuf = kmalloc(len, GFP_KERNEL);
	if (txbuf == NULL)
		return -ENOMEM;

	if(bCMD_REG == 1) // 0 == CMD, 1 == REG
		txbuf[0] = WRITE_REG_HEAD + RegInfo[1];
	else if(bCMD_REG == 0)
		txbuf[0] = RegInfo[1];

	for(i = 0; i < RegInfo[0]; i++)
		txbuf[i + 1] = RegInfo[i + 2];

	efsa120s_spi_transfer(spi, txbuf, NULL, RegInfo[0] + 1);
	ELAN_DEBUG(&spi->dev, "[ELAN] %s() ", __func__);

	for(i = 0; i < RegInfo[0]; i++)
		ELAN_DEBUG(&spi->dev, "0x%02x ", txbuf[i + 1]);

	ELAN_DEBUG(&spi->dev, "\n");
  	kfree(txbuf);
	return 0;

}

static ssize_t show_drv_version_value(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", VERSION_LOG);
}

static DEVICE_ATTR(drv_version, S_IRUGO, show_drv_version_value, NULL);

static struct attribute *efsa120s_attributes[] = {
	&dev_attr_drv_version.attr,
	NULL
};

static struct attribute_group efsa120s_attr_group = {
	.attrs = efsa120s_attributes,
};

static void efsa120s_reset(void)
{
#ifndef _MTK_PLATFORM_
	/* Developement platform */
	gpio_direction_output(GPIO_EFSA120_RST, 0);
	mdelay(5);
	gpio_direction_output(GPIO_EFSA120_RST, 1);
	mdelay(50);
#else
	/* MTK platform */
	mt_set_gpio_out(GPIO_EFSA120_RST, 0);
	mdelay(5);
	mt_set_gpio_out(GPIO_EFSA120_RST, 1);
	mdelay(50);
#endif
}

static int efsa120s_open(struct inode *inode, struct file *filp)
{
	struct efsa120s_data *fp;
	
	fp = container_of(inode->i_cdev, struct efsa120s_data, spi_cdev);
	filp->private_data = fp;
	
	ELAN_DEBUG(&fp->spi->dev, "[ELAN] %s()\n", __func__);
			
	return 0;
}

static int efsa120s_close(struct inode *inode, struct file *filp)
{
	struct efsa120s_data *fp = filp->private_data;
	ELAN_DEBUG(&fp->spi->dev, "[ELAN] %s()\n", __func__);
	return 0;
}

static ssize_t efsa120s_read(struct file *filp, char *user_buf, size_t len, loff_t *off)
{
	struct efsa120s_data *fp = filp->private_data;
	struct spi_device *spi = fp->spi;
  	int ret = 0; 
	int err = 0;
	int j = 0; 
	int i = 0; 
	int y = 0; 
	ELAN_DEBUG(&spi->dev, "%s() len=%d\r\n", __func__, (int) len);
	
	// Switch INT mode or Polling mode
	if(IOIRQ_STATUS & 0X08) // Buffer INT Enable
	{
		// Wait for INT IRQ Read complete.
		nWaitImage_Count = 0;
		while(nWaitImage_Count < nWaitImage)
		{
			if(fp->bImageReady == 1)
				break;
			nWaitImage_Count++;
			udelay(1); // 1us * 1000 = 1ms
		}
		if(nWaitImage_Count >= nWaitImage)
			return -1;

		fp->bImageReady = 0;
		Image_index = 0;
	}	
	else
	{
		if(status[1] & 0x1) // Image 1 Byte
		{ 
			err = efsa120s_receive_image_1BYte(fp->spi);
			if(err == -1)
				return -1;
			j = 1;
		}
		else{ // Image 2 Byte
			err = efsa120s_receive_image(fp->spi);
			if(err == -1)
				return -1;
			j = 2;
		}
	}
	
	for(y = 0; y < IMG_HEIGHT; y++) {
		#ifdef ALIG_4_BYTE
		i=(IMG_WIDTH * j + 4) * y + 2;
		#else
		i=(IMG_WIDTH * j + 2) * y + 2;
		#endif				
		err = copy_to_user(user_buf+(y*IMG_WIDTH * j), &imagebuffer[i], sizeof(unsigned char)*IMG_WIDTH * j);
		if(err)
			break;
	}

  	return (ret==0) ? len : ret;
}

/*******************************************************
Function:
    Disable irq function
Input:
    file: file
Output:
    None.

*********************************************************/
void efsa120s_irq_disable(void *_fp)
{
	struct efsa120s_data *fp = _fp;
#ifndef _MTK_PLATFORM_	
	unsigned long irqflags;
	ELAN_DEBUG(&fp->spi->dev, "IRQ Disable = %d.\n", fp->intr_gpio);

	spin_lock_irqsave(&fp->irq_lock, irqflags);
	if (!fp->irq_is_disable)
	{
		fp->irq_is_disable = 1; 
		disable_irq_nosync(fp->intr_gpio);
	}
	spin_unlock_irqrestore(&fp->irq_lock, irqflags);
#else
	mt_eint_mask(fp->intr_gpio);
#endif
}

/*******************************************************
Function:
    Enable irq function
Input:
    file: file
Output:
    None.
*********************************************************/
void efsa120s_irq_enable(void *_fp)
{
	struct efsa120s_data *fp = _fp;
#ifndef _MTK_PLATFORM_	
	unsigned long irqflags = 0;
	ELAN_DEBUG(&fp->spi->dev, "IRQ Enable = %d.\n", fp->intr_gpio);
  
	spin_lock_irqsave(&fp->irq_lock, irqflags);
	if (fp->irq_is_disable) 
	{
		enable_irq(fp->intr_gpio);
		fp->irq_is_disable = 0; 
	}
	spin_unlock_irqrestore(&fp->irq_lock, irqflags);
#else
	mt_eint_unmask(fp->intr_gpio);
#endif
}

static ssize_t efsa120s_write(struct file *filp, const char __user *user_buf, size_t len, loff_t *off)
{
	struct efsa120s_data *fp = filp->private_data;
	struct spi_device *spi = fp->spi;
	int ret = 0;
	unsigned char buf[130];
	unsigned char * txbuf = NULL;
	unsigned char * rxbuf = NULL;
	int i = 0;
	int j = 0;
	int ii = 0;	
	int nMax = 0, nMin = 65535, nData = 0;
	int len_buf = len;

	//ELAN_DEBUG(&spi->dev, "%s() len=%d\r\n", __func__, (int) len);
	/* for test */
	if(user_buf[0] == 0) // show function list
	{	
		printk("*******************************************************\r\n");
		printk("0: show function list. %s\r\n", VERSION_LOG);
		printk("1: efsa120s initialize ...\r\n");
		printk("2: start scan.\r\n");
		printk("3: show image min & max.\r\n");	
		printk("4: show image data.\r\n");
		printk("5/0: spi speed now\r\n");
		printk("5/1/data: spi speed update. data *100k\r\n");
		printk("6: hardware reset.\r\n");
		printk("7/0: interupt disable.\r\n");
		printk("7/1: interupt enable.\r\n");
		printk("8/data: write data. head, dummy ... you should set by yourself!!!\r\n");
		printk("9/cmd/len: read data. head, dummy ... you should count to len!!!\r\n");
		printk("A/data: IOIRQ_Status update in driver\r\n");
		printk("B: Probe log\r\n");
		printk("C/sig[4]/pid[4]:signal to pid.\r\n");
		printk("D/State: send power_key, State 0=up, 1=down.\r\n");
		printk("E/TOUCH/State:State 0=up, 1=down.\r\n");
		printk("*******************************************************\r\n");
	}
	else if(user_buf[0] == 1) // do init
	{
		IOIRQ_STATUS = 0X09;
		efsa120s_fingerprint_init(fp->spi, buf); // buf do not need
		printk("[ELAN] %s init ...\r\n", VERSION_LOG);
	}
	else if(user_buf[0] == 2) // start scan
	{
		Image_index = 0;
		fp->bImageReady = 0;
		nWaitImage_Count = 0;
		
		#ifdef ALIG_4_BYTE	
			len_buf = 4;
		#endif
		
		//SPI Command (Start_Scan)
		rxbuf = kzalloc(len_buf, GFP_KERNEL);		
		if (rxbuf == NULL)
			printk("memory error.\r\n");
		txbuf = kzalloc(len_buf, GFP_KERNEL);
		if (txbuf == NULL){
			printk("memory error.\r\n");
			kfree(rxbuf);
		}
		else{
			txbuf[0] = START_SCAN;
			efsa120s_spi_transfer(spi, txbuf, rxbuf, 1);
			printk("[ELAN] %s scan ...\r\n", VERSION_LOG);
			kfree(rxbuf);
			kfree(txbuf);
		}
	}
	else if(user_buf[0] == 3) // show max min data
	{
		//ELAN_DEBUG(&spi->dev, "log image [%dx%d]\r\n", IMG_WIDTH, IMG_HEIGHT);
		for(i = 0; i < IMG_HEIGHT; i++)
		{
			for(j = 0; j < IMG_WIDTH; j++)
			{
				ii = i * (IMG_WIDTH * 2 + 2) + j * 2 + 2;
				nData = (imagebuffer[ii] * 256 + imagebuffer[ii+1]);
				if(nMax < nData)
					nMax = nData;
				if(nMin > nData)
					nMin = nData;
			}		
		}
		printk("[ELAN] [Min,Max] = [%d,%d]\r\n", nMin, nMax);
	}
	else if(user_buf[0] == 4) // show image data
	{
		for(i = 0; i < IMG_HEIGHT; i++)
		{			
			for(j = 0; j < IMG_WIDTH; j++)
			{
				#ifdef ALIG_4_BYTE	
				ii = i * (IMG_WIDTH * 2 + 4) + j * 2 + 2;
				#else
				ii = i * (IMG_WIDTH * 2 + 2) + j * 2 + 2;
				#endif
				nData = (imagebuffer[ii] * 256 + imagebuffer[ii+1]);
				printk("%d ", nData);
			}		
		}
	}
	else if(user_buf[0] == 5)
	{		
		if(len == 4 && user_buf[1] == 1) // write
		{
			if(user_buf[1] == 1) // write
				spi_speed = (int) (100000 * user_buf[2]);
				printk("SPI speed update to %d.\r\n", spi_speed);
		}
		else if(len == 3 && user_buf[1] == 0) // read
			printk("SPI speed = %d.\r\n", spi_speed);
		else
			printk("SPI speed update length error. len = %d.\r\n", (int) len);
	}
	else if(user_buf[0] == 6) // do reset
	{
		efsa120s_reset();
		printk("efsa120s_reset\r\n");
	}
	else if(user_buf[0] == 7) // enable interrupt or disable
	{
		if(len == 3)
		{
			if(user_buf[1] == 1){
				printk("Interrupt enable.\r\n");
				efsa120s_irq_enable(fp);}
			else if(user_buf[1] == 0){
				printk("Interrupt enable.\r\n");
				efsa120s_irq_disable(fp);}
			else
				printk("Interrupt enable/disable data error.\r\n");
		}
		else
			printk("Interrupt enable/disable length error.\r\n");
	}
	else if(user_buf[0] == 8) // write spi
	{
		
		#ifdef ALIG_4_BYTE	
		if(((len_buf - 2) % 4) != 0)
			len_buf = (len - 2) / 4 * 4 + 4;
		#endif
		rxbuf = kzalloc(len_buf, GFP_KERNEL);		
		if (rxbuf == NULL)
			printk("memory error.\r\n");
		txbuf = kzalloc(len_buf, GFP_KERNEL);
		if (txbuf == NULL){
			printk("memory error.\r\n");
			kfree(rxbuf);
		}
		else{
			for(i=0; i< len-2; i++)
				txbuf[i] = user_buf[i+1];
			efsa120s_spi_transfer(spi, txbuf, rxbuf, len - 2);
			for(i=0; i< len - 2; i++)
				printk("[%d]%x ", i, txbuf[i]);
			printk("\r\n");
			for(i=0; i< len - 2; i++)
				printk("[%d]%x ", i, rxbuf[i]);
			printk("\r\n");
			kfree(rxbuf);
			kfree(txbuf);
		}		
	}
	else if(user_buf[0] == 9) // read spi
	{
		
		#ifdef ALIG_4_BYTE	
		if((user_buf[2] % 4) != 0)
			len_buf = user_buf[2] / 4 * 4 + 4;
		#endif
		
		rxbuf = kzalloc(len_buf, GFP_KERNEL);		
		if (rxbuf == NULL)
			printk("memory error.\r\n");
		txbuf = kzalloc(len_buf, GFP_KERNEL);
		if (txbuf == NULL){
			printk("memory error.\r\n");
			kfree(rxbuf);
		}
		else{
			txbuf[0] = user_buf[1];
			efsa120s_spi_transfer(spi, txbuf, rxbuf, user_buf[2]);
			for(i=0; i< user_buf[2]; i++)
				printk("[%d]%x ", i, txbuf[i]);
			printk("\r\n");
			for(i=0; i< user_buf[2]; i++)
				printk("[%d]%x ", i, rxbuf[i]);
			printk("\r\n");
			kfree(rxbuf);
			kfree(txbuf);
		}
	}
	else if(user_buf[0] == 10) // update ioirq
	{
		printk("IOIRQ = 0x%x -> 0x%x.\r\n", IOIRQ_STATUS, user_buf[1]);
		IOIRQ_STATUS = user_buf[1];	
	}
	else if(user_buf[0] == 11) // probe log
	{
		printk("[ELAN] Probe end. %x %x %x %x %x %x %x %x %x %x %x.\r\n", bProbeCheck[0], bProbeCheck[1], bProbeCheck[2], bProbeCheck[3], bProbeCheck[4], bProbeCheck[5], bProbeCheck[6], bProbeCheck[7], bProbeCheck[8], bProbeCheck[9], bProbeCheck[10]);	
	}
	else if(user_buf[0] == 12)// send signal to pid
	{
		if(len == 10) // cmd + 4 + 4 + na
		{
			if(copy_from_user(&sig, &user_buf[1], 4))
				return -1;
			if(copy_from_user(&pid, &user_buf[5], 4))
				return -1;
			bsig_pid = 1;
			efsa120s_send_signal_to_pid();
			printk("send signal sig=%d, pid=%d.\r\n", sig, (int) pid);
		}
		else
		{
			printk("send signal len error. len = %d. require 10.\r\n", (int) len);
		}
		
	}
	else if(user_buf[0] == 13) // report power key event, if you need report any key -> input_set_capability @ probe
	{
		if(!(user_buf[1] == 0 || user_buf[1] == 1))
			return -1;

		printk("KEY_#0x%x = %x.\r\n", KEY_FP_INT, user_buf[1]);
		input_report_key(fp->input_dev, KEY_FP_INT, user_buf[1]); // Added for KEY Event
		input_sync(fp->input_dev);
		mdelay(1);
	}
	else if(user_buf[0] == 14) // report touch event
	{
		printk("KEY_#0x%x = %x. It not work now.\r\n", user_buf[1], user_buf[2]);
	}
  	return (ret==0) ? len : ret;
}

static long efsa120s_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct efsa120s_data *fp = filp->private_data;
	unsigned char buf[16];
	unsigned char *pUserBuf;
	int err = 0;
	int i = 0;
	int j = 0;
	int y = 0;
	int value = 0;

	ELAN_DEBUG(&fp->spi->dev, "%s() : cmd = [%04X]\r\n", __func__, cmd);

	switch(cmd)
	{
		case ID_IOCTL_INIT:
			if(copy_from_user(buf, (int *)arg, sizeof(unsigned char)*16))
				return -1;
			efsa120s_fingerprint_init(fp->spi, buf);
			if(copy_to_user((int *)arg, buf, sizeof(unsigned char)*16))
				return -1;
			ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_INIT\r\n", __func__);
			break;
		
		case ID_IOCTL_READ_REGISTER:
			pUserBuf = (unsigned char *)arg;

			bCMD_REG = 1; // CMD = 0, REG= 1

			efsa120s_read_register(fp->spi, pUserBuf);

			if(copy_to_user((unsigned char *)arg, pUserBuf, pUserBuf[0] + 2))
				return -1;

			ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_READ_REGISTER\r\n", __func__);
			break;
			
		case ID_IOCTL_WRITE_REGISTER:	
			pUserBuf = (unsigned char *)arg;

			bCMD_REG = 1; // CMD = 0, REG= 1
			
			if(copy_from_user(buf, pUserBuf, pUserBuf[0] + 2))
				return -1;	
				
			efsa120s_write_register(fp->spi, buf);

			ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_WRITE_REGISTER\r\n", __func__);
			break;
		
		case ID_IOCTL_RESET:
			efsa120s_reset();
			ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_RESET\r\n", __func__);
			break;
		
		case ID_IOCTL_GET_RAW_IMAGE:
			ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_GET_RAW_IMAGE START\r\n", __func__);
			pUserBuf = (unsigned char *)arg;

			// Switch INT mode or Polling mode
			if(IOIRQ_STATUS & 0X08) // Buffer INT Enable
			{
				// Wait for INT IRQ Read complete.
				
				nWaitImage_Count = 0;
				while(nWaitImage_Count < nWaitImage)
				{
					if(fp->bImageReady == 1)
						break;
					nWaitImage_Count++;
					udelay(1); // 1us * 1000 = 1ms
				}
				if(nWaitImage_Count >= nWaitImage)
					return -1;

				fp->bImageReady = 0;
				Image_index = 0;		
			}	
			else
			{
				if(status[1] & 0x1){ // added by samuel
					ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_GET_RAW_IMAGE 1 BYTE\r\n", __func__);
					err = efsa120s_receive_image_1BYte(fp->spi);
					if(err == -1)
						return -1;
				}
				else{
					ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_GET_RAW_IMAGE 2 BYTE\r\n", __func__);
					err = efsa120s_receive_image(fp->spi);
					if(err == -1)
						return -1;
				}				
			}
			if(status[1] & 0x1)
				j = 1;
			else
				j = 2;
			for(y = 0; y < IMG_HEIGHT; y++) {
				#ifdef ALIG_4_BYTE
				i=(IMG_WIDTH * j + 4) * y + 2;
				#else
				i=(IMG_WIDTH * j + 2) * y + 2;
				#endif		
	
				err = copy_to_user(pUserBuf+(y*IMG_WIDTH * j), &imagebuffer[i], sizeof(unsigned char)*IMG_WIDTH * j);
				if(err)
					return -1;
			}
			ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_GET_RAW_IMAGE\r\n", __func__);
			break;
			
		case ID_IOCTL_STATUS:
			pUserBuf = (unsigned char *)arg;

			if(copy_from_user(&status[1], pUserBuf + 1, sizeof(unsigned char)*4)) // attention size, status[0] is read only
				return -1;		

			ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_STATUS = RO[0]%02x [1]%02x [2]%02x [3]%02x [4]%02x\r\n", __func__, status[0], status[1], status[2], status[3] , status[4]);
			if(status[1] & 0x04)
			{
				IMG_WIDTH = (unsigned int) (((status[2] & 0xF) << 8) | status[4]);
				IMG_HEIGHT = (unsigned int) (((status[2] & 0xF0) << 4) | status[3]);
				ELAN_DEBUG(&fp->spi->dev, "update HW: Y=%d X=%d\r\n", IMG_WIDTH, IMG_HEIGHT);
				if(IMG_WIDTH_DEFAULT < IMG_WIDTH){
					ELAN_DEBUG(&fp->spi->dev, "update HW: Y=%d -> %d\r\n", IMG_WIDTH, IMG_WIDTH_DEFAULT);	
					IMG_WIDTH = IMG_WIDTH_DEFAULT;
				}
				if(IMG_HEIGHT_DEFAULT < IMG_HEIGHT){
					ELAN_DEBUG(&fp->spi->dev, "update HW: X=%d -> %d\r\n", IMG_HEIGHT, IMG_HEIGHT_DEFAULT);	
					IMG_HEIGHT = IMG_HEIGHT_DEFAULT;
				}					
			}
			else
			{
				status[2] = (IMG_WIDTH >> 8);
				status[4] = (IMG_WIDTH & 0xff);
				status[2] = ((IMG_HEIGHT >> 4) & 0xf0);
				status[3] = (IMG_HEIGHT & 0xff);
				ELAN_DEBUG(&fp->spi->dev, "update HW: Y=%d X=%d\r\n", IMG_WIDTH, IMG_HEIGHT);	
			}

			if(copy_to_user((unsigned char *)arg, status, sizeof(unsigned char)* 5)) // attention size
				return -1;

			status[0] &= 0xFE; // added by samuel, clear interrupt status.
			break;
			
 		case ID_IOCTL_SET_AUTO_RAW_IMAGE:
			if(copy_from_user(&auto_raw_image_num, (int *)arg, sizeof(int)))
				return -1;
			ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_SET_AUTO_RAW_IMAGE = %d\r\n", __func__, auto_raw_image_num);
			break;
			
 		case ID_IOCTL_GET_AUTO_RAW_IMAGE:
			pUserBuf = (unsigned char *)arg;			
			for(auto_raw_image_count = 0; auto_raw_image_count < auto_raw_image_num; auto_raw_image_count++)
			{
				// Switch INT mode or Polling mode
				if(IOIRQ_STATUS & 0X08) // Buffer INT Enable
				{
					// Wait for INT IRQ Read complete.
				
					nWaitImage_Count = 0;
					while(nWaitImage_Count < nWaitImage)
					{
						if(fp->bImageReady == 1)
							break;
						nWaitImage_Count++;
						udelay(1); // 1us * 1000 = 1ms
					}
					if(nWaitImage_Count >= nWaitImage)
						return -1;

					fp->bImageReady = 0;
					Image_index = 0;		
				}	
				else
				{
					if(status[1] & 0x1){ // added by samuel
						ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_GET_RAW_IMAGE 1 BYTE\r\n", __func__);
						err = efsa120s_receive_image_1BYte(fp->spi);
						if(err == -1)
							return -1;
					}
					else{
						ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_GET_RAW_IMAGE 2 BYTE\r\n", __func__);
						err = efsa120s_receive_image(fp->spi);
						if(err == -1)
							return -1;
					}				
				}
				if(status[1] & 0x1)
					j = 1;
				else
					j = 2;
				for(y = 0; y < IMG_HEIGHT; y++) {
					#ifdef ALIG_4_BYTE
					i=(IMG_WIDTH * j + 4) * y + 2;
					#else
					i=(IMG_WIDTH * j + 2) * y + 2;
					#endif		
	
					err = copy_to_user(pUserBuf+(y*IMG_WIDTH*j)+auto_raw_image_count*(IMG_HEIGHT*IMG_WIDTH*j), &imagebuffer[i], sizeof(unsigned char)*IMG_WIDTH*j);
					if(err)
						return -1;
				}			
				// add interrupt event control			
			}
			ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_GET_AUTO_RAW_IMAGE\r\n", __func__);
			break;
			
		case ID_IOCTL_READ_CMD:
			pUserBuf = (unsigned char *)arg;

			bCMD_REG = 0; // CMD = 0, REG= 1

			efsa120s_read_register(fp->spi, pUserBuf);

			if(copy_to_user((unsigned char *)arg, pUserBuf, pUserBuf[0] + 2))
				return -1;

			ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_READ_CMD\r\n", __func__);
			break;
			
		case ID_IOCTL_WRITE_CMD:	
			pUserBuf = (unsigned char *)arg;

			bCMD_REG = 0; // CMD = 0, REG= 1
			
			if(copy_from_user(buf, pUserBuf, pUserBuf[0] + 2))
				return -1;

			if(pUserBuf[1] == START_SCAN){
				fp->bImageReady = 0;
				Image_index = 0;
			}
			efsa120s_write_register(fp->spi, buf);

			ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_WRITE_CMD\r\n", __func__);
			break;
			
		case ID_IOCTL_IOIRQ_STATUS:	
			pUserBuf = (unsigned char *)arg;

			if(copy_from_user(&IOIRQ_STATUS, pUserBuf, 1))
				return -1;
			if((IOIRQ_STATUS & INT_NORMAL_HIGH) == (status[0] & INT_NORMAL_HIGH)) // INT Normal status check
			{
				ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_IOIRQ_STATUS: IOIRQ = %x.\r\n", __func__, IOIRQ_STATUS);
			}
			else
			{
				ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_IOIRQ_STATUS: INT Normal status setting error. IOIRQ = %x, Driver = %x.\r\n", __func__, (IOIRQ_STATUS & INT_NORMAL_HIGH), (status[0] & INT_NORMAL_HIGH));
				IOIRQ_STATUS = 0; // clear
				return -1;		
			}

			if((IOIRQ_STATUS & 0xA0) || (IOIRQ_STATUS & 0X08))
				efsa120s_irq_enable(fp);
			break;
			
		case ID_IOCTL_SPI_STATUS:
			pUserBuf = (unsigned char *)arg;			
			
			if(pUserBuf[0] & 0x80) // Update spi parameter
			{
				spi_speed = (unsigned int) (pUserBuf[1] | (pUserBuf[2] << 8) | (pUserBuf[3] << 16) | (pUserBuf[4] << 24));
				spi_cs_delay = (unsigned int) (pUserBuf[5] | (pUserBuf[6] << 8));
				ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_SPI_STATUS: Update SPI : Speed=%d CS_Delay=%d.\r\n", __func__, spi_speed, spi_cs_delay);
			}
			else
			{
				if(copy_to_user(&pUserBuf[1], &spi_speed, 4))
					return -1;
				if(copy_to_user(&pUserBuf[5], &spi_cs_delay, 2))
					return -1;
				ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_SPI_STATUS: Read SPI : %x %x %x %x, %x %x\r\n", __func__, pUserBuf[1], pUserBuf[2], pUserBuf[3], pUserBuf[4], pUserBuf[5], pUserBuf[6]);
			}
			break;
			
		case ID_IOCTL_SIG_PID:
			pUserBuf = (unsigned char *)arg;

			if(pUserBuf[0] & 0x80) // Update SIG PID
			{
				if(copy_from_user(&sig, &pUserBuf[1], 4))
					return -1;
				if(copy_from_user(&pid, &pUserBuf[5], 4))
					return -1;
				bsig_pid = 1;
				ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_SIG_PID: Update Event: sig=%d, pid=%d.\r\n", __func__, sig, (int) pid);
			}
			else // read sig pid now
			{
				if(copy_to_user(&pUserBuf[1], &sig, 4))
					return -1;
				if(copy_to_user(&pUserBuf[5], &pid, 4))
					return -1;
				ELAN_DEBUG(&fp->spi->dev, "%s() : ID_IOCTL_SIG_PID: Read Event: sig=%d, pid=%d.\r\n", __func__, sig, (int) pid);
			}
			break;
			
		default:
			ELAN_DEBUG(&fp->spi->dev, "%s() : Unknown cmd\r\n", __func__);
			break;
	}
	//mutex_unlock(&fp->spi_mutex);
	return 0;
}

#ifdef _MTK_PLATFORM_
static long efsa120s_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return efsa120s_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

/*******************************************************
Function:
    efsa120s fingerprint work function
Input:
    work: work
Output:
    None.
*********************************************************/
static void efsa120s_fp_work_func(struct work_struct *work)
{
	struct efsa120s_data *fp;

	int i;
	int j = 0;
	int k = 0;
	
#ifdef ALIG_4_BYTE
	char txbuf[IMG_WIDTH*2+4];
#else
	char txbuf[IMG_WIDTH*2+2];
#endif

	fp = container_of(work, struct efsa120s_data, work);

	ELAN_DEBUG(&fp->spi->dev,"[ELAN] %s() IOIRQ=%x.\r\n", __func__, IOIRQ_STATUS);

	status[0] |= 1; // added by samuel

	if(IOIRQ_STATUS & 0xA0) // WOE Interrupt Enable
	{
		//#ifdef _SIGNAL_MODE_
		efsa120s_send_signal_to_pid();
		//#else
		if(bState == 0) // Display off
		{
			/*input_report_key(fp->input_dev,KEY_FP_INT, 1); // Added for KEY Event
			input_sync(fp->input_dev);
			mdelay(300);
			input_report_key(fp->input_dev,KEY_FP_INT, 0); // Added for KEY Event
			input_sync(fp->input_dev);
			mdelay(300);
			input_report_key(fp->input_dev,KEY_FP_INT, 1); // Added for KEY Event
			input_sync(fp->input_dev);
			mdelay(300);
			input_report_key(fp->input_dev,KEY_FP_INT, 0); // Added for KEY Event
			input_sync(fp->input_dev);
			mdelay(300);
			ELAN_DEBUG(&fp->spi->dev, "%s() RESERVED=0x%X\r\n", __func__, KEY_FP_INT);*/
		}
		/*
		else // Display on
		{
			input_report_key(fp->input_dev,KEY_FP_INT2, 1); // Added for KEY Event
			input_sync(fp->input_dev);
			mdelay(1);
			input_report_key(fp->input_dev,KEY_FP_INT2, 0); // Added for KEY Event
			input_sync(fp->input_dev);
			mdelay(1);
			ELAN_DEBUG(&fp->spi->dev, "%s() RESERVED=0x%X\r\n", __func__, KEY_FP_INT2);
		}*/
		//#endif
		return;
	}

	if(IOIRQ_STATUS & 0x08) // BUFFER Interrupt Enable
	{
		txbuf[0] = START_READ_IMAGE;		

#ifdef ALIG_4_BYTE //MTK DMA Mode So Used ALIG 4 Bytes
	j = 4;
#else
	j = 2;
#endif

		if(status[1] & 0x1)
			k = 1;
		else
			k = 2;
		
		//ELAN_DEBUG_DATA(&fp->spi->dev,"H2[%d] ", i);
		nWaitImage_Count = 0;					
		i = Image_index * (IMG_WIDTH * k + j); // k = 1 BYte or 2 Byte, j = ALIG
		Image_index++;
		ELAN_DEBUG(&fp->spi->dev,"[ELAN] Image_Index=%d H=%d\r\n", Image_index, IMG_HEIGHT);
		if(Image_index <= IMG_HEIGHT) // if over, memory will crash
			efsa120s_spi_transfer(fp->spi, txbuf, &(imagebuffer[i]), IMG_WIDTH * k + 2); // +2 is cmd & dummy

		if(Image_index == IMG_HEIGHT){
			fp->bImageReady = 1;
			Image_index = 0;
		}
	}

	efsa120s_irq_enable(fp);
}

#ifdef _MTK_PLATFORM_
static void mt_efsa120s_irq_handler(void)
{
	
	if(mt_fp==NULL)
		return;
	ELAN_DEBUG(&mt_fp->spi->dev, "%s()\n", __func__);

	efsa120s_irq_disable(mt_fp);
	
	queue_work(efsa120s_wq, &mt_fp->work);

	//return IRQ_HANDLED;
}
#endif

static irqreturn_t efsa120s_irq_handler(int irq, void *_fp)
{
	struct efsa120s_data *fp = _fp;

	ELAN_DEBUG(&fp->spi->dev, "%s()\n", __func__);

	efsa120s_irq_disable(fp);
	
	queue_work(efsa120s_wq, &fp->work);

	return IRQ_HANDLED;
}

static const struct file_operations efsa120s_fops = {
	.owner 			= THIS_MODULE,
	.open 			= efsa120s_open,
	.read 			= efsa120s_read,
	.write 			= efsa120s_write,
	.unlocked_ioctl = efsa120s_ioctl,
#ifdef _MTK_PLATFORM_
	.compat_ioctl 	= efsa120s_compat_ioctl,
#endif
	.release 		= efsa120s_close,
};

static int efsa120s_setup_cdev(struct efsa120s_data *finger)
{
	struct efsa120s_data *fp = spi_get_drvdata(finger->spi);
	dev_t devNum;
	int err = 0;
	
	ELAN_DEBUG(&fp->spi->dev, "%s()\n", __func__);

	err = alloc_chrdev_region(&devNum, 0, 1, "fingerprint");
	if(err < 0) { 
		printk("Alloc char dev region fail.\r\n");
		goto alloc_chrdev_region_fail;
	}

	bProbeCheck[4] = 1;
         
	major_number = MAJOR(devNum);
	minor_number = MINOR(devNum);	

	// Init Char Dev
	cdev_init(&finger->spi_cdev, &efsa120s_fops);
	finger->spi_cdev.owner = THIS_MODULE;
	finger->spi_cdev.ops = &efsa120s_fops;
	    
	// Region Chae dev under /proc/dev
	err = cdev_add(&finger->spi_cdev, devNum, 1);
	if (err < 0) {
	 	printk("add chr dev failed\n");
		goto cdev_add_fail;
	}

	bProbeCheck[4] = 2;

	// Create class under /sysfs
	finger->spi_class = class_create(THIS_MODULE, "fingerprint_class");
	if(IS_ERR(finger->spi_class))
	{
		err = -1;
		printk("class create failed\n");
    		goto class_create_fail;
	}

	bProbeCheck[4] = 3;
	// Create device under /dev
	finger->spi_device = device_create(finger->spi_class, NULL, devNum, "%s", "fingerprint");
	if(IS_ERR(finger->spi_device))
	{
		err = -1;
		printk("device create failed\n");
    		goto device_create_fail;
	}

	bProbeCheck[4] = 4;	

	return 0;

	device_destroy(finger->spi_class, devNum);
device_create_fail:
	class_destroy(finger->spi_class);
class_create_fail:	
	cdev_del(&fp->spi_cdev);
cdev_add_fail:
	unregister_chrdev_region(devNum,1);
alloc_chrdev_region_fail:
	
	return err;
}

static int efsa120s_sysfs_create(struct efsa120s_data *sysfs)
{
	struct efsa120s_data *fp = spi_get_drvdata(sysfs->spi);
	int error = 0;
	
	mutex_init(&fp->sysfs_mutex);
	
	/* Register sysfs */
	error = sysfs_create_group(&fp->spi->dev.kobj, &efsa120s_attr_group);
	if (error) {
		dev_err(&fp->spi->dev, "[ELAN] Failed to create sysfs attributes, err: %d\n", error);
		goto fail_un;
	}
	
fail_un:
	/* Remove sysfs */
	sysfs_remove_group(&fp->spi->dev.kobj, &efsa120s_attr_group);
	
	/* Release Mutex */
	if(&fp->sysfs_mutex)
		mutex_destroy(&fp->sysfs_mutex);
		
	return error;
}
	  
static char efsa120s_gpio_config(void *_fp)
{	
	struct efsa120s_data *fp = _fp;	
	int ret;
#ifndef _MTK_PLATFORM_
	/* Developement platform */
	// Configure INT GPIO (Input)
	ret = gpio_request(GPIO_EFSA120_IRQ, "efsa120-irq");
	if (ret < 0) 
	{
		printk("[ELAN] %s() IRQ%d request fail, err=0x%x.\n", __func__, GPIO_EFSA120_IRQ, ret);
		ret = -ENODEV;
	}
	else
	{
		gpio_direction_input(GPIO_EFSA120_IRQ);
		fp->intr_gpio = gpio_to_irq(GPIO_EFSA120_IRQ);
		printk("[ELAN] %s() IRQ%d=%d request success, err=0x%x.\n", __func__, GPIO_EFSA120_IRQ, fp->intr_gpio, ret);
	}
	
	// Configure RST GPIO (Output)
	ret =  gpio_request(GPIO_EFSA120_RST, "efsa120-reset");
	if (ret < 0) 
	{
		gpio_free(GPIO_EFSA120_IRQ);
		free_irq(fp->intr_gpio, fp);
		printk("[ELAN] %s() RST%d request fail, err=0x%x.\n", __func__, GPIO_EFSA120_RST, ret);
		ret = -ENODEV;
	}
	else
	{
		printk("[ELAN] %s() RST%d request success, err=0x%x.\n", __func__, GPIO_EFSA120_RST, ret);
		gpio_direction_output(GPIO_EFSA120_RST, 0);
		mdelay(20);
		gpio_direction_output(GPIO_EFSA120_RST, 1);
		mdelay(20);
		printk("[ELAN] %s() Reset ...\n", __func__);
	}
#else	
	/* MTK platform */
	mt_set_gpio_mode(GPIO_EFSA120_RST, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_EFSA120_RST, GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(GPIO_EFSA120_RST, true);
	mt_set_gpio_out(GPIO_EFSA120_RST, 1);

	//kent
	//mt_set_gpio_mode(GPIO_EFSA120_IRQ, GPIO_MODE_01);
	//mt_set_gpio_dir(GPIO_EFSA120_IRQ, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_EFSA120_IRQ, GPIO_DVT_EINT_PIN_M_EINT);
  	mt_set_gpio_dir(GPIO_EFSA120_IRQ, GPIO_DIR_IN);
  	mt_set_gpio_pull_enable(GPIO_EFSA120_IRQ, false);
	fp->intr_gpio =CUST_EINT_DTV_NUM ;//gpio_to_irq(GPIO_EFSA120_IRQ);
	//mt_set_gpio_pull_enable(GPIO_EFSA120_IRQ, true); // Interrupr need pull-up ?
	//mt_set_gpio_out(GPIO_EFSA120_IRQ, 0); // // Interrupt is input, out value ?
#endif
	return ret;
}

static int efsa120s_probe(struct spi_device *spi)
{	
	struct efsa120s_data *fp = NULL;
	struct input_dev *input_dev = NULL;
	int err = 0;
	unsigned char buf[6] = {0};
	
	printk("%s() %s\n", __func__, VERSION_LOG);
	
	/* Setup SPI */
	spi->mode = SPI_MODE_0; 		// set at spi_board_info
	spi->max_speed_hz = SPI_MAX_SPEED; 	// set at spi_board_info
	spi->chip_select = 0; 		// set at spi_board_info
	spi->bits_per_word = 8;			// do not change
	
	bProbeCheck[0] = 0;
	bProbeCheck[1] = 0;
	bProbeCheck[2] = 0;
	bProbeCheck[3] = 0;
	bProbeCheck[4] = 0;
	bProbeCheck[5] = 0;
	bProbeCheck[6] = 0;
	bProbeCheck[7] = 0;
	bProbeCheck[8] = 0;
	bProbeCheck[9] = 0;
	bProbeCheck[10] = 0;
	
	err = spi_setup(spi);
	if (err < 0)
	{
		bProbeCheck[0] = 1;
		printk("[ELAN] spi_setup fail (0x%x).\r\n", err);
		//return err;
	}	
	
	/* Allocate Device Data */
	fp = kzalloc(sizeof(struct efsa120s_data), GFP_KERNEL);
	if(!fp) {
		bProbeCheck[1] = 1;
		printk("[ELAN] alloc efsa120s data fail.\r\n");		
		//goto alloc_mem_fail;
	}
		
#ifdef _MTK_PLATFORM_
	mt_fp = fp;
#endif
	
	/* Init Mutex */
	mutex_init(&fp->spi_mutex);	
	
	/* Init Input Device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		bProbeCheck[2] = 1;
		printk("[ELAN] alloc input_dev fail.\r\n");
		//goto input_allocate_device_fail;
	}
	
	spi->irq = GPIO_EFSA120_IRQ;
	fp->spi = spi;		
	
	spi_set_drvdata(spi, fp);	
	
	input_dev->name = "efsa120s";
	input_dev->id.bustype = BUS_SPI;
	input_dev->dev.parent = &spi->dev;
	input_set_drvdata(input_dev, fp);	
	
	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY);
	input_set_capability(input_dev, EV_KEY, KEY_FP_INT); 	// change by customer, send key event to framework. KEY_xxx could be changed.
	//input_set_capability(input_dev, EV_KEY, KEY_FP_INT2); 	// change by customer, send key event to framework. KEY_xxx could be changed.
	
	fp->input_dev = input_dev;	

	/* Init Sysfs */
	/*err = efsa120s_sysfs_create(fp);
	if(err < 0) {
		bProbeCheck[3] = 1;
		printk("[ELAN] efsa120s sysfs fail.\r\n");
		//goto sysfs_create_fail;
	}*/
	
	/* Init Char Device */
	err = efsa120s_setup_cdev(fp);
	if(err < 0) {
		//bProbeCheck[4] = 1;
		printk("[ELAN] efsa120s setup device fail.\r\n");
		//goto cdev_setup_fail;
	}
	
	/* Register Input Device */
	err = input_register_device(input_dev);
	if(err) {
		bProbeCheck[5] = 1;
		printk("[ELAN] Unable to register input device, error: %d!\r\n", err);
		//goto input_dev_creat_fail;
	}
	
	/* MTK platform */
#ifdef _MTK_PLATFORM_
	efsa120s_gpio_power_on();
	mdelay(1000);
#endif

	/* Init EFSA120S GPIO */
	err = efsa120s_gpio_config(fp);
	if(err < 0) {
		bProbeCheck[6] = 1;
		printk("[ELAN] GPIO request fail (%d).\r\n", err);
		//goto gpio_config_fail;
	}

	/* Can use SPI Transfer now*/
	buf[0] = 4; 			// read length
	buf[1] = ADDR_SIZE_WH; 	// reg addr
	bCMD_REG = 1; 			// CMD = 0, REG= 1
	err = efsa120s_read_register(fp->spi, buf);
	if(err < 0) {
		bProbeCheck[7] = 1;
		printk("[ELAN] efsa120s read device length & width fail.\r\n");
		//goto read_WH_fail;
	}
	else
	{
		printk("[ELAN] efsa120s read device %02x %02x %02x %02x.\r\n", buf[2], buf[3], buf[4], buf[5]);
		IMG_WIDTH = (unsigned int)(buf[5] - buf[4] + 1);
		IMG_HEIGHT = (unsigned int)(buf[3] - buf[2] + 1);
		IMG_WIDTH_DEFAULT = IMG_WIDTH;
		IMG_HEIGHT_DEFAULT = IMG_HEIGHT;
		printk("[ELAN] efsa120s WIDTH(Y)=%d, HEIGHT(X)=%d.\r\n", IMG_WIDTH, IMG_HEIGHT);
	}

	/* Allocate image buffer */
	err = efsa120s_kmalloc_image();
	if(err) {
		bProbeCheck[8] = 1;
		printk("[ELAN] Unable to kmalloc image buffer, error (%d).\r\n", err);
		//goto kmalloc_image_fail;
	}

	efsa120s_wq = create_singlethread_workqueue("efsa120s_wq");
	if (!efsa120s_wq)
	{
		bProbeCheck[9] = 1;
		printk("[ELAN] Work Create error! \r\n");
		//goto  request_irq_fail;
	}

	INIT_WORK(&fp->work, efsa120s_fp_work_func);

	spin_lock_init(&fp->irq_lock); // Added for ISR 2.6.39 later  
  	//fp->irq_lock = SPIN_LOCK_UNLOCKED;   // Added for ISR 2.6.39 & before
  	
#ifdef _MTK_PLATFORM_
	elan_isdbt_gpio_interrupt_register();

	mt_eint_unmask(CUST_EINT_DTV_NUM);
#else
	/* Init IRQ FUNC */
	err = request_irq(fp->intr_gpio, efsa120s_irq_handler, IRQF_NO_SUSPEND | IRQF_TRIGGER_HIGH, "fingerprint-irq", fp);
	//status[0] |= 0x40; // For INT Normal High(Trigger Low), Mark it when Normal low.
	if(err) {
		bProbeCheck[10] = 1;
		printk("[ELAN] Failed to request IRQ %d.\r\n", err);
		//goto  request_irq_fail;
	}
	irq_set_irq_wake(fp->intr_gpio, 1);
#endif

	printk("[ELAN] Probe end. %x %x %x %x %x %x %x %x %x %x %x.\r\n", bProbeCheck[0], bProbeCheck[1], bProbeCheck[2], bProbeCheck[3], bProbeCheck[4], bProbeCheck[5], bProbeCheck[6], bProbeCheck[7], bProbeCheck[8], bProbeCheck[9], bProbeCheck[10]);

#ifdef CONFIG_HAS_EARLYSUSPEND
	fp->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	fp->early_suspend.suspend = efsa120s_early_suspend;
	fp->early_suspend.resume = efsa120s_late_resume;
	register_early_suspend(&fp->early_suspend);
#endif

	return 0;

request_irq_fail:

gpio_config_fail:
	// memory have been kfree in efsa120s_kmalloc_image function.

kmalloc_image_fail:
	// memory have been kfree in efsa120s_kmalloc_image function.

read_WH_fail:

input_dev_creat_fail:
	spi_set_drvdata(spi, NULL);
	input_free_device(input_dev);
	input_dev = NULL;

cdev_setup_fail:
	//cdev_del(&fp->spi_cdev);
	//unregister_chrdev_region(MKDEV(major_number, minor_number), 1); // marked by samuel, had been done before.
sysfs_create_fail:

input_allocate_device_fail:

alloc_mem_fail:
	kfree(fp);
	return -ENOMEM;
}

#ifndef _MTK_PLATFORM_
static int efsa120s_remove(struct spi_device *spi)
#else
static int efsa120s_remove(struct spi_device *spi)
#endif
{
	struct efsa120s_data *fp = spi_get_drvdata(spi);
#ifdef _MTK_PLATFORM_
#else
	if (fp->intr_gpio)
		free_irq(fp->intr_gpio, fp);
#endif

	gpio_free(GPIO_EFSA120_IRQ);
	gpio_free(GPIO_EFSA120_RST);
	
	cdev_del(&fp->spi_cdev);
	device_destroy(fp->spi_class, MKDEV(major_number, minor_number));	//delete device node under /dev
	class_destroy(fp->spi_class);
	unregister_chrdev_region(MKDEV(major_number, minor_number), 1);		//delecte class create by bus
	input_free_device(fp->input_dev);
	
	if(&fp->spi_mutex)
		mutex_destroy(&fp->spi_mutex);
		
	kfree(fp);

	spi_set_drvdata(spi, NULL);
		
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int efsa120s_suspend(struct device *dev)
{
	printk("[ELAN] efsa120s suspend!\n");
	return 0;
}

static int efsa120s_resume(struct device *dev)
{
	printk("[ELAN] efsa120s resume!\n");
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(efsa120s_pm_ops, efsa120s_suspend, efsa120s_resume);

static void efsa120s_early_suspend(struct early_suspend *h)
{
	bState = 0;
	printk("[ELAN] efsa120s early suspend!\n");	
}

static void efsa120s_late_resume(struct early_suspend *h)
{
	bState = 1;
	printk("[ELAN] efsa120s late resume!\n");	
}

static struct spi_driver efsa120s_driver = {
	.driver = {
		//.name 	= "efsa120s",
		.name 	= "fingerprint",
		.owner = THIS_MODULE,
		.pm 	= &efsa120s_pm_ops,
	},
	.probe 	= efsa120s_probe,
#ifndef _MTK_PLATFORM_
	.remove = efsa120s_remove,
#else
	.remove = __exit_p(efsa120s_remove),
#endif
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend    = efsa120s_early_suspend,
	.resume     = efsa120s_late_resume,
#endif
};

static struct spi_board_info efsa120s_spi_board_info[] = {
        [0] = {
				.modalias               = "efsa120s",
			#ifndef _MTK_PLATFORM_
				.bus_num                = 1, // change by customer
			#else
				.bus_num                = 3//0, // change by customer
			#endif
                .chip_select            = 0, // change by customer, usually = 0.
                .max_speed_hz           = SPI_MAX_SPEED,
				.mode					= SPI_MODE_0,
        },
};

static int __init efsa120s_init(void)
{
	printk("******************* efsa120s_init *******************\r\n");
	
	//spi_register_board_info(efsa120s_spi_board_info, ARRAY_SIZE(efsa120s_spi_board_info));
	
	return spi_register_driver(&efsa120s_driver);
}

static void __exit efsa120s_exist(void)
{	
	
	spi_unregister_driver(&efsa120s_driver);
	if(efsa120s_wq)
	{
		destroy_workqueue(efsa120s_wq);
	}
}

module_init(efsa120s_init);
module_exit(efsa120s_exist);

MODULE_AUTHOR("KennyKang <kenny.kang@emc.com.tw>");
MODULE_DESCRIPTION("ELAN SPI FingerPrint eFSA120S driver");
MODULE_VERSION(VERSION_LOG);
MODULE_LICENSE("GPL");
