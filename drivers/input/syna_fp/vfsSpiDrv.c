/*! @file spiDrv.c
*******************************************************************************
**  SPI Driver Interface Functions
**
**  This file contains the SPI driver interface functions.
**
**  Copyright 2013-2014 Synaptics, Inc. All Rights Reserved.
**
**  This program is free software; you can redistribute it and/or
**  modify it under the terms of the GNU General Public License
**  as published by the Free Software Foundation; either version 2
**  of the License, or (at your option) any later version.
**
**  This program is distributed in the hope that it will be useful,
**  but WITHOUT ANY WARRANTY; without even the implied warranty of
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**  GNU General Public License for more details.
**
**  You should have received a copy of the GNU General Public License
**  along with this program; if not, write to the Free Software
**  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
**  MA  02110-1301, USA."
**
*/

#include <vfsDrvIoctl.h>
#include <vfsSpiDrv.h>

#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
//#include <asm-generic/uaccess.h>
#include <linux/fdtable.h>
#include <linux/eventfd.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif /* CONFIG_COMPAT */

#define VFSSPI_TEST_SPI_COMMUNICATION

/* The spi driver private structure. */
/**
 * vfsspi_devData - The spi driver private structure
 * @devt:Device ID
 * @cdev:Character device handle
 * @vfsSpiLock:The lock for the spi device
 * @spi:The spi device
 * @deviceEntry:Device entry list
 * @bufferMutex:The lock for the transfer buffer
 * @isOpened:Indicates that driver is opened
 * @rxBuffer:buffer for retrieving data
 * @txBuffer:buffer for transmitting data
 * @drdyPin:DRDY GPIO pin number
 * @sleepPin:Sleep GPIO pin number
 * @userPID:User process ID, to which the kernel signal
	indicating DRDY event is to be sent
 * @signalID:Signal ID which kernel uses to indicating
	user mode driver that DRDY is asserted
 * @curSpiSpeed:Current baud rate of SPI master clock
 * @isDrdyIrqEnabled:Indicates that DRDY irq is enabled
*/
struct vfsspi_devData {
	dev_t devt;
	struct cdev cdev;
	spinlock_t vfsSpiLock;
	struct spi_device *spi;
	struct class *devClass;
	struct list_head deviceEntry;
	struct mutex bufferMutex;
	unsigned int isOpened;
	unsigned char *rxBuffer;
	unsigned char *txBuffer;
	unsigned int drdyPin;
	unsigned int sleepPin;
	int gpio_irq;
	int userPID;
	int signalID;
	unsigned int curSpiSpeed;
	unsigned int isDrdyIrqEnabled;
	int status;
};

#ifdef CONFIG_COMPAT
/*
 * Used by VFSSPI_IOCTL_RW_SPI_MESSAGE IOCTL compat command:
 *
 * @rxBuffer:pointer to retrieved data
 * @txBuffer:pointer to transmitted data
 * @len:transmitted/retrieved data size
 */
struct vfsspi_compat_ioctl_transfer {
	compat_uptr_t rxBuffer;
	compat_uptr_t txBuffer;
	unsigned int len;
};
#endif /* CONFIG_COMPAT */

static int vfsspi_open(struct inode *inode, struct file *filp);
static int vfsspi_release(struct inode *inode, struct file *filp);

static int vfsspi_probe(struct spi_device *spi);
static int vfsspi_remove(struct spi_device *spi);

static int vfsspi_devInit(struct spi_device *spi,
	struct vfsspi_devData **spiDev);
static void vfsspi_devUnInit(struct vfsspi_devData *vfsSpiDev);

static int vfsspi_chrDevRegister(struct vfsspi_devData *vfsSpiDev);
static void vfsspi_chrDevUnregister(struct vfsspi_devData *vfsSpiDev);

static int vfsspi_gpioInit(struct vfsspi_devData *vfsSpiDev);
static void vfsspi_gpioUnInit(struct vfsspi_devData *vfsSpiDev);

static irqreturn_t vfsspi_irq(int irq, void *context);
static void vfsspi_enableIrq(struct vfsspi_devData *vfsSpiDev);
static void vfsspi_disableIrq(struct vfsspi_devData *vfsSpiDev);

static int vfsspi_sendDrdyNotify(struct vfsspi_devData *vfsSpiDev);

static ssize_t vfsspi_read(struct file *filp, char __user *buf,
	size_t count, loff_t *fPos);
static ssize_t vfsspi_write(struct file *filp, const char __user *buf,
	size_t count, loff_t *fPos);
static int vfsspi_xfer(struct vfsspi_devData *vfsSpiDev,
	struct vfsspi_iocTransfer *tr);
static inline ssize_t vfsspi_readSync(struct vfsspi_devData *vfsSpiDev,
	unsigned char *buf, size_t len);
static inline ssize_t vfsspi_writeSync(struct vfsspi_devData *vfsSpiDev,
	unsigned char *buf, size_t len);
static long vfsspi_unlockedIoctl(struct file *filp, unsigned int cmd,
	unsigned long arg);
#ifdef CONFIG_COMPAT
static long vfsspi_compatIoctl(struct file *filp, unsigned int cmd,
						 unsigned long arg);
#endif /* CONFIG_COMPAT */
static long vfsspi_ioctl(struct file *filp, unsigned int cmd,
						 unsigned long arg, int compat);
static void vfsspi_hardReset(struct vfsspi_devData *vfsSpiDev);
static void vfsspi_suspend(struct vfsspi_devData *vfsSpiDev);
static int vfsspi_transferSpiData(struct vfsspi_devData *vfsSpiDev,
	unsigned long arg, int compat);
static int vfsspi_setClk(struct vfsspi_devData *vfsSpiDev,
	unsigned long arg);
static int vfsspi_registerDrdySignal(struct vfsspi_devData *vfsSpiDev,
	unsigned long arg);
static int vfsspi_setDrdyInt(struct vfsspi_devData *vfsSpiDev,
	unsigned long arg);
static int vfsspi_selectDrdyNtfType(struct vfsspi_devData *vfsSpiDev,
	unsigned long arg);

static inline void shortToLittleEndian(char *buf, size_t len);

/* SPI driver info */
struct spi_driver vfsspi_spi = {
	.driver = {
		   .name = SYNA_PART_NAME,
		   .owner = THIS_MODULE,
		   },
	.probe = vfsspi_probe,
	//.remove = __devexit_p(vfsspi_remove),
	.remove = vfsspi_remove,
};

/* file operations associated with device */
const struct file_operations vfsspi_fops = {
	.owner = THIS_MODULE,
	.write = vfsspi_write,
	.read = vfsspi_read,
	.unlocked_ioctl = vfsspi_unlockedIoctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = vfsspi_compatIoctl,
#endif /* CONFIG_COMPAT */
	.open = vfsspi_open,
	.release = vfsspi_release,
};

static LIST_HEAD(deviceList);
static DEFINE_MUTEX(deviceListMutex);

inline void shortToLittleEndian(char *buf, size_t len)
{
#if PLATFORM_BIG_ENDIAN
	int i = 0;
	int j = 0;
	char LSB, MSB;

	for (i = 0; i < len; i++, j++) {
		LSB = buf[i];
		i++;

		MSB = buf[i];
		buf[j] = MSB;

		j++;
		buf[j] = LSB;

	}
#endif /* PLATFORM_BIG_ENDIAN */
}

static ssize_t fp_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct vfsspi_devData *vfsSpiDev = NULL;

	vfsSpiDev = dev_get_drvdata(dev);
	return sprintf(buf, "%s fp status= %x\n", __func__, vfsSpiDev->status);
}

static DEVICE_ATTR(fp_status, 0440, fp_status_show, NULL);


static int vfsspi_devInit(struct spi_device *spi,
	struct vfsspi_devData **spiDev)
{
	int status;
	struct vfsspi_devData *vfsSpiDev = NULL;

	PR_INFO("vfsspi_devInit\n");

	vfsSpiDev = kzalloc(sizeof(*vfsSpiDev), GFP_KERNEL);
	if (vfsSpiDev == NULL) {
		PR_ERR("Failed to allocate buffer\n");
		status = -ENOMEM;
		goto cleanup;
	}

	/* Initialize driver data. */
	vfsSpiDev->curSpiSpeed = MAX_BAUD_RATE;
	spin_lock_init(&vfsSpiDev->vfsSpiLock);
	mutex_init(&vfsSpiDev->bufferMutex);
	INIT_LIST_HEAD(&vfsSpiDev->deviceEntry);

	spi->bits_per_word = BITS_PER_WORD;
	spi->max_speed_hz = MAX_BAUD_RATE;
	spi->mode = SPI_MODE_0;

	status = spi_setup(spi);
	if (0 != status) {
		PR_ERR("spi_setup() is failed! status= %d\n", status);
		goto cleanup;
	}

	status = vfsspi_chrDevRegister(vfsSpiDev);
	if (0 != status) {
		PR_ERR("vfsspi_chrDevRegister failed! status= %d\n", status);
		goto cleanup;
	}

	spi_set_drvdata(spi, vfsSpiDev);

	vfsSpiDev->spi = spi;
	*spiDev = vfsSpiDev;
	status = 0;

cleanup:
	if (0 != status && NULL != vfsSpiDev) {
		/* Release allocated resources */
		mutex_destroy(&vfsSpiDev->bufferMutex);
		kfree(vfsSpiDev);
	}

	return status;
}

static void vfsspi_devUnInit(struct vfsspi_devData *vfsSpiDev)
{
	PR_INFO("vfsspi_devUnInit\n");

	if (vfsSpiDev != NULL) {
		spin_lock_irq(&vfsSpiDev->vfsSpiLock);
		spi_set_drvdata(vfsSpiDev->spi, NULL);
		vfsSpiDev->spi = NULL;
		spin_unlock_irq(&vfsSpiDev->vfsSpiLock);

		vfsspi_chrDevUnregister(vfsSpiDev);

		mutex_destroy(&vfsSpiDev->bufferMutex);

		kfree(vfsSpiDev);
	}
}

static int vfsspi_chrDevRegister(struct vfsspi_devData *vfsSpiDev)
{
	int status;
	struct device *dev = NULL;
	int chrDevAlloc = 0;
	int chrDevInit = 0;
	int classCreate = 0;

	PR_INFO("vfsspi_chrDevRegister\n");

	mutex_lock(&deviceListMutex);

	/* register major number for character device */
	status = alloc_chrdev_region(&(vfsSpiDev->devt),
		0, 1, SYNA_PART_NAME);
	if (status < 0) {
		PR_ERR("alloc_chrdev_region failed\n");
		goto cleanup;
	}

	chrDevAlloc = 1;

	cdev_init(&(vfsSpiDev->cdev), &vfsspi_fops);
	vfsSpiDev->cdev.owner = THIS_MODULE;

	status = cdev_add(&(vfsSpiDev->cdev), vfsSpiDev->devt, 1);
	if (status < 0) {
		PR_ERR("cdev_add failed\n");
		goto cleanup;
	}

	chrDevInit = 1;

	vfsSpiDev->devClass = class_create(THIS_MODULE, SYNA_PART_NAME);
	if (IS_ERR(vfsSpiDev->devClass)) {
		PR_ERR("class_create failed\n");
		status = PTR_ERR(vfsSpiDev->devClass);
		goto cleanup;
	}

	classCreate = 1;

	dev = device_create(vfsSpiDev->devClass, &vfsSpiDev->spi->dev,
		vfsSpiDev->devt, vfsSpiDev, SYNA_DEV_NAME);
	if (IS_ERR(dev)) {
		PR_ERR("device_create failed\n");
		status = PTR_ERR(dev);
		goto cleanup;
	}

	status = device_create_file(dev, &dev_attr_fp_status);
	if (status)
	{
		PR_ERR("create file failed\n");
		goto cleanup;
	}

	list_add(&vfsSpiDev->deviceEntry, &deviceList);
	status = 0;

cleanup:
	mutex_unlock(&deviceListMutex);

	if (0 != status) {
		/* Release allocated resources */
		if (0 != classCreate)
			class_destroy(vfsSpiDev->devClass);

		if (0 != chrDevInit)
			cdev_del(&(vfsSpiDev->cdev));

		if (0 != chrDevAlloc)
			unregister_chrdev_region(vfsSpiDev->devt, 1);
	}

	return status;
}

static void vfsspi_chrDevUnregister(struct vfsspi_devData *vfsSpiDev)
{
	PR_INFO("vfsspi_chrDevUnregister\n");

	if (NULL != vfsSpiDev) {
		mutex_lock(&deviceListMutex);

		/* Destroy character device */
		list_del(&vfsSpiDev->deviceEntry);

		device_destroy(vfsSpiDev->devClass, vfsSpiDev->devt);
		class_destroy(vfsSpiDev->devClass);

		cdev_del(&(vfsSpiDev->cdev));
		unregister_chrdev_region(vfsSpiDev->devt, 1);

		mutex_unlock(&deviceListMutex);
	}
}

static int vfsspi_gpioInit(struct vfsspi_devData *vfsSpiDev)
{
	int status = 0;
	int drdyPinInitalized = 0;
	int sleepPinInitalized = 0;

	PR_INFO("vfsspi_gpioInit\n");

	if (vfsSpiDev == NULL) {
		PR_ERR("vfsspi_gpioInit: vfsSpiDev is NULL\n");
		status = -EFAULT;
		goto cleanup;
	}

	vfsSpiDev->drdyPin = VFSSPI_DRDY_PIN;
	vfsSpiDev->sleepPin = VFSSPI_SLEEP_PIN;

	status = gpio_request(vfsSpiDev->drdyPin, "vfsspi_drdy");
	if (status < 0) {
		PR_ERR("gpio_request(DRDY) is failed! status=%d\n", status);
		status = -EBUSY;
		goto cleanup;
	}
	drdyPinInitalized = 1;

	status = gpio_request(vfsSpiDev->sleepPin, "vfsspi_sleep");
	if (status < 0) {
		PR_ERR("gpio_request(SLEEP)is failed! status=%d\n", status);
		status = -EBUSY;
		goto cleanup;
	}
	sleepPinInitalized = 1;

	status = gpio_direction_output(vfsSpiDev->sleepPin, 1);
	if (status < 0) {
		PR_ERR("gpio_direction_output is failed! status=%d\n", status);
		status = -EBUSY;
		goto cleanup;
	}

	status = gpio_direction_input(vfsSpiDev->drdyPin);
	if (status < 0) {
		PR_ERR("gpio_direction_input is failed! status=%d\n", status);
		status = -EBUSY;
		goto cleanup;
	}

	vfsSpiDev->gpio_irq = gpio_to_irq(vfsSpiDev->drdyPin);
	if (vfsSpiDev->gpio_irq < 0) {
		PR_ERR("gpio_to_irq failed! gpio_irq=%d\n",
			vfsSpiDev->gpio_irq);
		status = -EBUSY;
		goto cleanup;
	}

	status = request_irq(vfsSpiDev->gpio_irq, vfsspi_irq,
		DRDY_IRQ_FLAG, "vfsspi_irq", vfsSpiDev);
	if (status < 0) {
		PR_ERR("request_irq failed! status=%d\n", status);
		status = -EBUSY;
		goto cleanup;
	}

	vfsSpiDev->isDrdyIrqEnabled = DRDY_IRQ_ENABLE;
	status = 0;

cleanup:
	if (0 != status) {
		if (0 != drdyPinInitalized)
			gpio_free(vfsSpiDev->drdyPin);

		if (0 != sleepPinInitalized)
			gpio_free(vfsSpiDev->sleepPin);
	}
	return status;
}

static void vfsspi_gpioUnInit(struct vfsspi_devData *vfsSpiDev)
{
	PR_INFO("vfsspi_gpioUnInit\n");

	if (vfsSpiDev != NULL) {
		free_irq(vfsSpiDev->gpio_irq, vfsSpiDev);
		vfsSpiDev->isDrdyIrqEnabled = DRDY_IRQ_DISABLE;
		gpio_free(vfsSpiDev->sleepPin);
		gpio_free(vfsSpiDev->drdyPin);
	}
}

static void vfsspi_enableIrq(struct vfsspi_devData *vfsSpiDev)
{
	PR_INFO("vfsspi_enableIrq\n");

	if (vfsSpiDev->isDrdyIrqEnabled == DRDY_IRQ_ENABLE)
		PR_DEBUG("DRDY irq already enabled\n");
	else {
		enable_irq(vfsSpiDev->gpio_irq);
		vfsSpiDev->isDrdyIrqEnabled = DRDY_IRQ_ENABLE;
	}
}

static void vfsspi_disableIrq(struct vfsspi_devData *vfsSpiDev)
{
	PR_INFO("vfsspi_disableIrq\n");

	if (vfsSpiDev->isDrdyIrqEnabled == DRDY_IRQ_DISABLE)
		PR_DEBUG("DRDY irq already disabled\n");
	else {
		disable_irq_nosync(vfsSpiDev->gpio_irq);
		vfsSpiDev->isDrdyIrqEnabled = DRDY_IRQ_DISABLE;
	}
}

static irqreturn_t vfsspi_irq(int irq, void *context)
{
	struct vfsspi_devData *vfsSpiDev = context;
	int drdy_value = 10;

	PR_INFO("vfsspi_irq\n");

	/* Linux kernel is designed so that when you disable
	an edge-triggered interrupt, and the edge happens while
	the interrupt is disabled, the system will re-play the
	interrupt at enable time.
	Therefore, we are checking DRDY GPIO pin state to make sure
	if the interrupt handler has been called actually by DRDY
	interrupt and it's not a previous interrupt re-play */

	drdy_value = gpio_get_value(vfsSpiDev->drdyPin);
	PR_INFO("vfsspi drdyPin value = %d\n", drdy_value);	
	
	//if (gpio_get_value(vfsSpiDev->drdyPin) == DRDY_ACTIVE_STATUS) {
	if (gpio_get_value(vfsSpiDev->drdyPin) > 0) {
		vfsspi_disableIrq(vfsSpiDev);
		vfsspi_sendDrdyNotify(vfsSpiDev);
	}

	return IRQ_HANDLED;
}

static int vfsspi_sendDrdyNotify(struct vfsspi_devData *vfsSpiDev)
{
	struct task_struct *t;
	struct file *efd_file = NULL;
	struct eventfd_ctx *efd_ctx = NULL;
	int status = 0;

	PR_INFO("vfsspi_sendDrdyNotify\n");

	if (vfsSpiDev->userPID != 0) {
		/* find the task_struct associated with userpid */
		PR_INFO("Searching task with PID=%08x\n", vfsSpiDev->userPID);

		rcu_read_lock();
		t = pid_task(find_pid_ns(vfsSpiDev->userPID, &init_pid_ns),
			PIDTYPE_PID);
		if (t == NULL) {
			rcu_read_unlock();
			PR_DEBUG("No such pid\n");
			status = -ENODEV;
			goto cleanup;
		}

		efd_file = fcheck_files(t->files, vfsSpiDev->signalID);
		rcu_read_unlock();

		if (efd_file == NULL) {
			PR_DEBUG("No such efd_file\n");
			status = -ENODEV;
			goto cleanup;
		}

		efd_ctx = eventfd_ctx_fileget(efd_file);
		if (efd_ctx == NULL) {
			PR_ERR("eventfd_ctx_fileget is failed\n");
			status = -ENODEV;
			goto cleanup;
		}

		/* notify DRDY eventfd to user process */
		eventfd_signal(efd_ctx, 1);

		/* Release eventfd context */
		eventfd_ctx_put(efd_ctx);
	}

cleanup:
	return status;
}

/* Return no.of bytes written to device. Negative number for errors */
static inline ssize_t vfsspi_writeSync(struct vfsspi_devData *vfsSpiDev,
	unsigned char *buf, size_t len)
{
	int status = 0;
	struct spi_message m;
	struct spi_transfer t;

	spi_message_init(&m);
	memset(&t, 0, sizeof(t));

	t.rx_buf = NULL;
	t.tx_buf = buf;
	t.len = len;
	t.speed_hz = vfsSpiDev->curSpiSpeed;

	spi_message_add_tail(&t, &m);

	status = spi_sync(vfsSpiDev->spi, &m);

	if (status == 0)
		status = m.actual_length;
	else
		PR_ERR("spi_sync fail, status=%d\n", status);

	return status;
}

/* Return no.of bytes read >0. negative integer incase of error. */
static inline ssize_t vfsspi_readSync(struct vfsspi_devData *vfsSpiDev,
	unsigned char *buf, size_t len)
{
	int status = 0;
	struct spi_message m;
	struct spi_transfer t;

	spi_message_init(&m);
	memset(&t, 0x0, sizeof(t));

	t.tx_buf = NULL;
	t.rx_buf = buf;
	t.len = len;
	t.speed_hz = vfsSpiDev->curSpiSpeed;

	spi_message_add_tail(&t, &m);

	status = spi_sync(vfsSpiDev->spi, &m);

	if (status == 0)
		status = m.actual_length;
	else
		PR_ERR("spi_sync fail, status=%d\n", status);

	return status;
}

static ssize_t vfsspi_write(struct file *filp, const char __user *buf,
	size_t count, loff_t *fPos)
{
	struct vfsspi_devData *vfsSpiDev = NULL;
	ssize_t status = 0;

	PR_INFO("vfsspi_write\n");

	vfsSpiDev = filp->private_data;
	if (NULL == vfsSpiDev) {
		PR_ERR("filp->private_data is NULL for writing data\n");
		status = -EFAULT;
		goto cleanup;
	}

	if (count > DEFAULT_BUFFER_SIZE || count <= 0) {
		PR_ERR("passed incorrect buffer length - %d\n", count);
		status = -EMSGSIZE;
		goto cleanup;
	}

	mutex_lock(&vfsSpiDev->bufferMutex);

	if (vfsSpiDev->rxBuffer) {
		if (copy_from_user(vfsSpiDev->rxBuffer, buf, count) == 0) {
			shortToLittleEndian((char *)vfsSpiDev->rxBuffer, count);
			status = vfsspi_writeSync(vfsSpiDev,
				vfsSpiDev->rxBuffer, count);
		} else {
			PR_ERR("copy to user failed\n");
			status = -EFAULT;
		}
	}

	mutex_unlock(&vfsSpiDev->bufferMutex);

cleanup:
	return status;
}

static ssize_t vfsspi_read(struct file *filp, char __user *buf, size_t count,
	loff_t *fPos)
{
	struct vfsspi_devData *vfsSpiDev = NULL;
	ssize_t status = 0;

	PR_INFO("vfsspi_read\n");

	vfsSpiDev = filp->private_data;

	if (NULL == vfsSpiDev) {
		PR_ERR("filp->private_data is NULL for reading data\n");
		status = -EFAULT;
		goto cleanup;
	}

	if (count > DEFAULT_BUFFER_SIZE || count <= 0) {
		PR_ERR("passed incorrect buffer length - %d\n", count);
		status = -EMSGSIZE;
		goto cleanup;
	}

	if (buf == NULL) {
		PR_ERR("passed buffer is NULL\n");
		status = -EFAULT;
		goto cleanup;
	}

	mutex_lock(&vfsSpiDev->bufferMutex);

	status = vfsspi_readSync(vfsSpiDev, vfsSpiDev->rxBuffer, count);

	if (status > 0) {
		unsigned long missing = 0;
		/* data read. Copy to user buffer. */
		shortToLittleEndian((char *)vfsSpiDev->rxBuffer, status);

		missing = copy_to_user(buf, vfsSpiDev->rxBuffer, status);

		if (missing == status) {
			PR_ERR("copy_to_user failed\n");
			/* Nothing was copied to user space buffer. */
			status = -EFAULT;
		} else {
			status = status - missing;
		}
	}

	mutex_unlock(&vfsSpiDev->bufferMutex);

cleanup:
	return status;
}

static int vfsspi_xfer(struct vfsspi_devData *vfsSpiDev,
	struct vfsspi_iocTransfer *tr)
{
	int status = 0;
	struct spi_message m;
	struct spi_transfer t;

	if (vfsSpiDev == NULL || tr == NULL) {
		PR_ERR("passed NULL parameter\n");
		status = -EFAULT;
		goto cleanup;
	}

	if (tr->len > DEFAULT_BUFFER_SIZE || tr->len <= 0) {
		PR_ERR("passed incorrect buffer length - %d\n", tr->len);
		status = -EMSGSIZE;
		goto cleanup;
	}

	if (tr->txBuffer != NULL) {
		if (copy_from_user(vfsSpiDev->txBuffer, tr->txBuffer,
			tr->len) != 0) {
			status = -EFAULT;
			goto cleanup;
		}

		shortToLittleEndian((char *)vfsSpiDev->txBuffer, tr->len);
	}

	spi_message_init(&m);
	memset(&t, 0, sizeof(t));

	t.tx_buf = vfsSpiDev->txBuffer;
	t.rx_buf = vfsSpiDev->rxBuffer;
	t.len = tr->len;
	t.speed_hz = vfsSpiDev->curSpiSpeed;

	spi_message_add_tail(&t, &m);

	status = spi_sync(vfsSpiDev->spi, &m);

	if (status == 0) {
		if (tr->rxBuffer != NULL) {
			unsigned missing = 0;

			shortToLittleEndian((char *)vfsSpiDev->rxBuffer,
				tr->len);
			missing = copy_to_user(tr->rxBuffer,
				vfsSpiDev->rxBuffer, tr->len);

			if (missing != 0)
				tr->len = tr->len - missing;
		}
	}

cleanup:
	return status;
}

long vfsspi_unlockedIoctl(struct file *filp, unsigned int cmd,
	unsigned long arg)
{
	PR_INFO("vfsspi_unlockedIoctl");
	return vfsspi_ioctl(filp, cmd, arg, 0);
}

#ifdef CONFIG_COMPAT
long vfsspi_compatIoctl(struct file *filp, unsigned int cmd,
	unsigned long arg)
{
	PR_INFO("vfsspi_compatIoctl");
	return vfsspi_ioctl(filp, cmd, arg, 1);
}
#endif /* CONFIG_COMPAT */

long vfsspi_ioctl(struct file *filp, unsigned int cmd, unsigned long arg,
	int compat)
{
	int status = 0;
	struct vfsspi_devData *vfsSpiDev = NULL;

	if (_IOC_TYPE(cmd) != VFSSPI_IOCTL_MAGIC) {
		PR_DEBUG("invalid magic. cmd=0x%X Received=0x%X "
			"Expected=0x%X\n",
			cmd, _IOC_TYPE(cmd), VFSSPI_IOCTL_MAGIC);
		status = -ENOTTY;
		goto cleanup;
	}

	vfsSpiDev = filp->private_data;
	if (NULL == vfsSpiDev) {
		PR_ERR("filp->private_data is NULL\n");
		status = -EFAULT;
		goto cleanup;
	}

	mutex_lock(&vfsSpiDev->bufferMutex);

	switch (cmd) {
	case VFSSPI_IOCTL_DEVICE_SUSPEND:
	{
		PR_INFO("VFSSPI_IOCTL_DEVICE_SUSPEND:");
		vfsspi_suspend(vfsSpiDev);
		break;
	}
	case VFSSPI_IOCTL_DEVICE_RESET:
	{
		PR_INFO("VFSSPI_IOCTL_DEVICE_RESET:");
		vfsspi_hardReset(vfsSpiDev);
		break;
	}
	case VFSSPI_IOCTL_RW_SPI_MESSAGE:
	{
		PR_INFO("VFSSPI_IOCTL_RW_SPI_MESSAGE:");
		status = vfsspi_transferSpiData(vfsSpiDev, arg, compat);
		break;
	}
	case VFSSPI_IOCTL_SET_CLK:
	{
		PR_INFO("VFSSPI_IOCTL_SET_CLK:");
		status = vfsspi_setClk(vfsSpiDev, arg);
		break;
	}
	case VFSSPI_IOCTL_REGISTER_DRDY_SIGNAL:
	{
		PR_INFO("VFSSPI_IOCTL_REGISTER_DRDY_SIGNAL:");
		status = vfsspi_registerDrdySignal(vfsSpiDev, arg);
		break;
	}
	case VFSSPI_IOCTL_SET_DRDY_INT:
	{
		PR_INFO("VFSSPI_IOCTL_SET_DRDY_INT:");
		status = vfsspi_setDrdyInt(vfsSpiDev, arg);
		break;
	}
	case VFSSPI_IOCTL_SELECT_DRDY_NTF_TYPE:
	{
		PR_INFO("VFSSPI_IOCTL_SELECT_DRDY_NTF_TYPE:");
		status = vfsspi_selectDrdyNtfType(vfsSpiDev, arg);
		break;
	}
    case VFSSPI_IOCTL_POWER_ON:
	{
        /* Add code here to turn on sensor power, if need */
		break;
	}
    case VFSSPI_IOCTL_POWER_OFF:
	{
        /* Add code here to turn off sensor power, if need */
		break;
	}
    case VFSSPI_IOCTL_SET_SPI_CONFIGURATION:
	{
        /* Perform SPI core initialization and/or SPI clock enabling
        from power consumption perspective */
		break;
	}
    case VFSSPI_IOCTL_RESET_SPI_CONFIGURATION:
	{
        /* Perform SPI clock disabling and/or SPI core un-initialization
        from power consumption perspective */
		break;
	}
    default:
    {
		PR_DEBUG("Unknown cmd=0x%X\n", cmd);
		status = -EFAULT;
		break;
    }

	}
	mutex_unlock(&vfsSpiDev->bufferMutex);

cleanup:
	return status;
}

void vfsspi_hardReset(struct vfsspi_devData *vfsSpiDev)
{
	PR_INFO("vfsspi_hardReset\n");

	if (vfsSpiDev != NULL) {
		spin_lock(&vfsSpiDev->vfsSpiLock);
		gpio_set_value(vfsSpiDev->sleepPin, 0);
		mdelay(1);
		gpio_set_value(vfsSpiDev->sleepPin, 1);
		spin_unlock(&vfsSpiDev->vfsSpiLock);
		mdelay(5);
	}
}

void vfsspi_suspend(struct vfsspi_devData *vfsSpiDev)
{
	PR_INFO("vfsspi_suspend\n");

	if (vfsSpiDev != NULL) {
		spin_lock(&vfsSpiDev->vfsSpiLock);
		gpio_set_value(vfsSpiDev->sleepPin, 0);
		spin_unlock(&vfsSpiDev->vfsSpiLock);
	}
}

static int vfsspi_transferSpiData(struct vfsspi_devData *vfsSpiDev,
	unsigned long arg, int compat)
{
	int status = -EFAULT;
	struct vfsspi_iocTransfer *trData = NULL;
#ifdef CONFIG_COMPAT
	struct vfsspi_compat_ioctl_transfer trDataCompat;
#endif /* CONFIG_COMPAT */
	PR_INFO("vfsspi_transferSpiData\n");

	trData = kmalloc(sizeof(struct vfsspi_iocTransfer), GFP_KERNEL);
	if (trData == NULL) {
		PR_ERR("trData is NULL\n");
		status = -ENOMEM;
		goto cleanup;
	}

#ifdef CONFIG_COMPAT
	if (compat != 0) {
		if (copy_from_user(&trDataCompat, (void *)arg,
			sizeof(struct vfsspi_compat_ioctl_transfer)) != 0) {
			PR_ERR("copy from user failed\n");
			status = -EFAULT;
			goto cleanup;
		}
		trData->rxBuffer = (unsigned char *)trDataCompat.rxBuffer;
		trData->txBuffer = (unsigned char *)trDataCompat.txBuffer;
		trData->len = trDataCompat.len;
	} else
#endif /* CONFIG_COMPAT */
	{
		if (copy_from_user(trData, (void *)arg,
			sizeof(struct vfsspi_iocTransfer)) != 0) {
			PR_ERR("copy from user failed\n");
			status = -EFAULT;
			goto cleanup;
		}
	}

	status = vfsspi_xfer(vfsSpiDev, trData);
	if (status != 0) {
		PR_ERR("vfsspi_xfer is failed\n");
		goto cleanup;
	}

#ifdef CONFIG_COMPAT
	if (compat != 0) {
		trDataCompat.len = trData->len;
		status = copy_to_user((void *)arg, &trDataCompat,
			sizeof(struct vfsspi_compat_ioctl_transfer));
	} else
#endif /* CONFIG_COMPAT */
	{
		status = copy_to_user((void *)arg, trData,
			sizeof(struct vfsspi_iocTransfer));
	}

	if (status != 0) {
		PR_ERR("copy to user failed\n");
		status = -EFAULT;
		goto cleanup;
	}

cleanup:
	if (NULL != trData)
		kfree(trData);

	return status;
}

static int vfsspi_setClk(struct vfsspi_devData *vfsSpiDev,
	unsigned long arg)
{
	struct spi_device *spidev = NULL;
	int status = 0;
	unsigned short clock = 0;

	if (copy_from_user(&clock, (void *)arg,
		sizeof(unsigned short)) != 0) {
		PR_ERR("Failed copy from user.\n");
		status = -EFAULT;
	} else {
		spin_lock_irq(&vfsSpiDev->vfsSpiLock);
		spidev = spi_dev_get(vfsSpiDev->spi);
		spin_unlock_irq(&vfsSpiDev->vfsSpiLock);

		if (spidev != NULL) {
			vfsSpiDev->curSpiSpeed = clock * BAUD_RATE_COEF;

			if (vfsSpiDev->curSpiSpeed > MAX_BAUD_RATE)
				vfsSpiDev->curSpiSpeed = MAX_BAUD_RATE;

			spidev->max_speed_hz = vfsSpiDev->curSpiSpeed;
			spi_dev_put(spidev);
			PR_INFO("Baud rate is set to %d Mbit.\n",
				vfsSpiDev->curSpiSpeed);
		}
	}

	return status;
}

static int vfsspi_registerDrdySignal(struct vfsspi_devData *vfsSpiDev,
	unsigned long arg)
{
	struct vfsspi_iocRegSignal usrSignal;
	int status = 0;

	if (copy_from_user(&usrSignal, (void *)arg,
		sizeof(usrSignal)) != 0) {
		PR_ERR("copy from user failed.\n");
		status = -EFAULT;
	} else {
		vfsSpiDev->userPID = usrSignal.userPID;
		vfsSpiDev->signalID = usrSignal.signalID;
	}

	return status;
}

static int vfsspi_setDrdyInt(struct vfsspi_devData *vfsSpiDev,
	unsigned long arg)
{
	int status = 0;
	unsigned short drdy_enable_flag;

	if (copy_from_user(&drdy_enable_flag, (void *)arg,
				 sizeof(drdy_enable_flag)) != 0) {
		PR_ERR("Failed copy from user.\n");
		status = -EFAULT;
	} else {
		if (drdy_enable_flag == 0)
			vfsspi_disableIrq(vfsSpiDev);
		else {
			/* Workaround the issue where the system
			  misses DRDY notification to host when
			  DRDY pin was asserted before enabling
			  device.*/
			if (gpio_get_value(vfsSpiDev->drdyPin) ==
				DRDY_ACTIVE_STATUS)
				vfsspi_sendDrdyNotify(vfsSpiDev);
            else
                vfsspi_enableIrq(vfsSpiDev);
		}
	}

	return status;
}

static int vfsspi_selectDrdyNtfType(struct vfsspi_devData *vfsSpiDev,
	unsigned long arg)
{
	int status = 0;
	vfsspi_iocSelectDrdyNtfType_t drdyNtfType;

	drdyNtfType.selectedType = VFSSPI_DRDY_NOTIFY_TYPE_EVENTFD;
	if (copy_to_user((void *)arg, &(drdyNtfType),
		sizeof(vfsspi_iocSelectDrdyNtfType_t)) != 0) {
		PR_ERR("copy to user failed\n");
		status = -EFAULT;
	}

	return status;
}

#ifdef VFSSPI_TEST_SPI_COMMUNICATION
static int vfsspi_test(struct spi_device *spi)
{
	char tx_buf[64] = {1};
	char rx_buf[64] = {0};
	struct spi_transfer t;
	struct spi_message m;
	int i = 0;
	int status = 0;
	unsigned int temp_val;

	pr_info("synafpspi-COMMTEST: Start SPI communication test");

	/* EP0 Read */
	tx_buf[0] = 1;
	tx_buf[1] = 0;
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	memset(&t, 0, sizeof(t));
	t.tx_buf = tx_buf;
	t.rx_buf = rx_buf;
	t.len = 6;

	spi_setup(spi);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);


	status = spi_sync(spi, &m);
	pr_info("synafpspi-COMMTEST: EP0 read is returned %d", status);
	if (status != 0) {
		status = -EFAULT;
		goto cleanup;
	}

	pr_info("synafpspi-COMMTEST: EP0 status is:");
	for (i = 0; i < 6; i++) {
        pr_info("synafpspi-COMMTEST: data[%d]=0x%02x ", i, rx_buf[i]);
    }

	mdelay(10);

	/* EP0 read */
	tx_buf[0] = 1;
	tx_buf[1] = 0;
	memset(&t, 0, sizeof(t));
	t.tx_buf = tx_buf;
	t.rx_buf = rx_buf;
	t.len = 6;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	status = spi_sync(spi, &m);
	pr_info("synafpspi-COMMTEST: EP0 read is returned %d", status);
	if (status != 0) {
		status = -EFAULT;
		goto cleanup;
	}

	/* Check that sensor is alive and sensor can accept the command */
	temp_val = *(unsigned int *)(rx_buf + 2);
	if ((temp_val & 0x80000000) == 1 ||
		(temp_val & 0x08000000) == 0) {
		pr_info("synafpspi-COMMTEST: incorrect EP0 data is read %d",
			temp_val);
		status = -EFAULT;
		goto cleanup;
	}

	pr_info("synafpspi-COMMTEST: EP0 status is:");
	for (i = 0; i < 6; i++) {
        pr_info("synafpspi-COMMTEST: data[%d]=0x%02x ", i, rx_buf[i]);
	}

	mdelay(5);

	/* GET_VERSION command on EP1OUT */
	tx_buf[0] = 2;
	tx_buf[1] = 1;

	memset(&t, 0, sizeof(t));
	t.tx_buf = tx_buf;
	t.rx_buf = rx_buf;
	t.len = 2;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	status = spi_sync(spi, &m);
	pr_info("synafpspi-COMMTEST: GET_VERSION command send is returned %d",
		status);
	if (status != 0) {
		status = -EFAULT;
		goto cleanup;
	}
	mdelay(5);

	/* Read GET_VERSION command reply */
	tx_buf[0] = 3;
	tx_buf[1] = 0;

	memset(&t, 0, sizeof(t));
	t.tx_buf = tx_buf;
	t.rx_buf = rx_buf;
	t.len = 40;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	status = spi_sync(spi, &m);
	pr_info("synafpspi-COMMTEST: GET_VERSION reply read is returned %d",
		status);
	if (status != 0) {
		status = -EFAULT;
		goto cleanup;
	}

	/* Check reply status */
	temp_val = *(unsigned short *)(rx_buf + 2);
	if (temp_val != 0) {
		pr_info("synafpspi-COMMTEST: incorrect reply status is read 0x%04x",
			temp_val);
		status = -EFAULT;
		goto cleanup;
	}

	pr_info("synafpspi-COMMTEST: GET_VERSION reply is:");
	for (i = 0; i < 40; i++) {
        pr_info("synafpspi-COMMTEST: data[%d]=%02x ", i, rx_buf[i]);
	}
	status = 0;

cleanup:
	pr_info("synafpspi-COMMTEST: Test is %s",
		status < 0 ? "failed" : "succeeded");
	return status;
}
#endif /* VFSSPI_TEST_SPI_COMMUNICATION */

int vfsspi_open(struct inode *inode, struct file *filp)
{
	struct vfsspi_devData *vfsSpiDev = NULL;
	int status = -ENXIO;

	PR_INFO("vfsspi_open\n");

	mutex_lock(&deviceListMutex);
	list_for_each_entry(vfsSpiDev, &deviceList, deviceEntry) {
		if (vfsSpiDev->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status == 0) {
		if (vfsSpiDev->isOpened != 0) {
			status = -EBUSY;
		} else {
			vfsSpiDev->userPID = 0;
			if (vfsSpiDev->rxBuffer == NULL) {
				vfsSpiDev->txBuffer =
					kmalloc(DEFAULT_BUFFER_SIZE,
							GFP_KERNEL);
				vfsSpiDev->rxBuffer =
					kmalloc(DEFAULT_BUFFER_SIZE,
							GFP_KERNEL);

				if (vfsSpiDev->rxBuffer == NULL ||
					vfsSpiDev->txBuffer == NULL) {
					PR_ERR("Failed to allocate buffer\n");
					status = -ENOMEM;
				} else {
					vfsSpiDev->isOpened = 1;
					filp->private_data = vfsSpiDev;
					nonseekable_open(inode, filp);
				}
			}
		}
	}

	mutex_unlock(&deviceListMutex);

	return status;
}

int vfsspi_release(struct inode *inode, struct file *filp)
{
	struct vfsspi_devData *vfsSpiDev = NULL;
	int status = 0;

	PR_INFO("vfsspi_release\n");

	mutex_lock(&deviceListMutex);
	vfsSpiDev = filp->private_data;
	if (vfsSpiDev != NULL) {
		filp->private_data = NULL;
		vfsSpiDev->isOpened = 0;
		if (vfsSpiDev->rxBuffer != NULL) {
			kfree(vfsSpiDev->rxBuffer);
			vfsSpiDev->rxBuffer = NULL;
		}

		if (vfsSpiDev->txBuffer != NULL) {
			kfree(vfsSpiDev->txBuffer);
			vfsSpiDev->txBuffer = NULL;
		}
	}
	mutex_unlock(&deviceListMutex);

	return status;
}

int vfsspi_probe(struct spi_device *spi)
{
	int status = 0;
	struct vfsspi_devData *vfsSpiDev = NULL;

	PR_INFO("vfsspi_probe\n");

	status = vfsspi_devInit(spi, &vfsSpiDev);
	if (0 != status) {
		PR_ERR("vfsspi_devInit is failed! status= %d\n", status);
		goto cleanup;
	}

	status = vfsspi_gpioInit(vfsSpiDev);
	if (0 != status) {
		PR_ERR("vfsspi_gpioInit is failed! status= %d\n", status);
		vfsspi_devUnInit(vfsSpiDev);
		goto cleanup;
	}

#ifdef VFSSPI_TEST_SPI_COMMUNICATION
	vfsspi_hardReset(vfsSpiDev);
	mdelay(20);
	vfsSpiDev->status = vfsspi_test(spi);
	vfsspi_hardReset(vfsSpiDev);
	if (0 != vfsSpiDev->status) {
		PR_ERR("vfsspi_test is failed! status= %d\n", vfsSpiDev->status);
		goto cleanup;
	}
#endif /* VFSSPI_TEST_SPI_COMMUNICATION */

	PR_INFO("vfsspi_probe succeeded\n");

cleanup:
	return status;
}

int vfsspi_remove(struct spi_device *spi)
{
	int status = 0;
	struct vfsspi_devData *vfsSpiDev = NULL;

	PR_INFO("vfsspi_remove\n");

	vfsSpiDev = spi_get_drvdata(spi);

	if (NULL != vfsSpiDev) {
		vfsspi_gpioUnInit(vfsSpiDev);

		vfsspi_devUnInit(vfsSpiDev);
	}

	return status;
}

static int __init vfsspi_init(void)
{
	int status = 0;

	PR_INFO("vfsspi_init\n");

	status = spi_register_driver(&vfsspi_spi);
	if (status < 0)
		PR_ERR("spi_register_driver is failed with status %d", status);
	else
		PR_INFO("vfsspi_init succeeded\n");


	return status;
}

static void __exit vfsspi_exit(void)
{
	PR_INFO("vfsspi_exit\n");

	spi_unregister_driver(&vfsspi_spi);
}

module_init(vfsspi_init);
module_exit(vfsspi_exit);

//MODULE_LICENSE("GPL");
