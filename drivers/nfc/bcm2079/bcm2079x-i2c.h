
#ifndef __BCM2079X_I2C_H__
#define __BCM2079X_I2C_H__

struct bcm2079x_dev {
	wait_queue_head_t read_wq;
	struct mutex read_mutex;
	struct i2c_client *client;
	struct miscdevice bcm2079x_device;
	unsigned int wake_gpio;
	unsigned int en_gpio;
	unsigned int irq_gpio;
	bool irq_enabled;
	spinlock_t irq_enabled_lock;
	unsigned int count_irq;
};

int bcm2079x_probe_test(struct i2c_client *client);
void nfc_set_client_addr(struct bcm2079x_dev *bcm2079x_dev, int addr);
void bcm2079x_disable_irq(struct bcm2079x_dev *bcm2079x_dev);
void bcm2079x_enable_irq(struct bcm2079x_dev *bcm2079x_dev);

#endif  //__BCM2079X_I2C_H__


