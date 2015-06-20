/* -------------------------------------------------------------------------
 * Copyright (C) 2010 Inside Secure
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 * ------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------
 * Copyright (C) 2014-2016, Intel Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 * ------------------------------------------------------------------------- */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <asm/delay.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
#define __devexit_p(x) x
#endif

#include "fdp_main.h"

/* define to debug all function calls */
//#define TRACE_THIS_MODULE

/***************************************/
/******  Platform dependent  ***********/

/* I2C parameters, must be in sync with your board configuration */
#define I2C_ID_NAME "fdp"
#define I2C_DEVICE_ADDR 0x5E

/***************************************/

#define I2C_WRITE_DATA_LENGTH 288   /* Maximum number of bytes to Write during an I2C Write cycle (FieldsPeak RX FIFO Size + Len + CRC) */
#define I2C_READ_DATA_LENGTH  288   /* Number of bytes to Read during an I2C Read cycle (FieldsPeak TX FIFO Size + Len + CRC) */

#define I2C_LENGTH_FRAME_SIZE   5

#define ENTER() \
    pr_debug("%s\n", __FUNCTION__)

/* The normal driver driver sequence is:
 - open
 - ... reset / read / write / poll
 - close
 */
enum custom_state {
    CUSTOM_INIT = 0,
    CUSTOM_PROBED,
    CUSTOM_OPENED,
};

#define RCV_BUFFER_NB               2

#define IOH_PHONE_ON                0
#define IOH_PHONE_OFF               1

#define RST_RESET                   0
#define RST_NO_RESET                1

/* Context variable */

struct fdp_custom_device {

    /* configuration stuff */
    enum custom_state state;

    /* mutex for concurrency */
    struct mutex    mutex;

    /* I2C Driver related stuff */
    struct i2c_client *i2c_client;  /* I2C Driver registering structure */

    unsigned int rst_gpio;
    unsigned int irq_gpio;

    unsigned int    irqout;

    /* I2C receiver */
    uint16_t        next_receive_length;

    uint8_t         next_to_read;
    uint8_t         next_to_write;

    uint8_t         rx_buffer[RCV_BUFFER_NB][I2C_READ_DATA_LENGTH];
    uint16_t        rx_data_length[RCV_BUFFER_NB];

    uint8_t         rx_scratch_buffer[I2C_READ_DATA_LENGTH];

    /* process synchronization (poll) */
    wait_queue_head_t read_queue;
};

static struct fdp_custom_device *fdp_p_device = NULL;

/**
  * Function to initialize the state of the driver for new session
  *
  * It flushes all received data
  * This function should be called each time a new session is done, e.g.
  * - after initial opening of the device
  * - when the FieldsPeak is reset
  */

static void fdp_struct_initialize(struct fdp_custom_device *p_device)
{
    int i;
    ENTER();

    if (!p_device) {
        printk(KERN_ERR "fdp_struct_initialize: Internal error p_device is missing\n");
        return;
    }

    p_device->next_receive_length = I2C_LENGTH_FRAME_SIZE;
    p_device->next_to_read = 0;
    p_device->next_to_write = 0;

    for (i=0; i<RCV_BUFFER_NB; i++)
    {
        p_device->rx_data_length[i] = 0;
    }
}

/**
  *  Function to cleanup the state of the driver
  *
  * Reverts to CUSTOM_PROBED state.
  *
  * This function should be called when the device is closed.
  */

static void fdp_struct_cleanup(struct fdp_custom_device *p_device)
{
    ENTER();
    if (!p_device) {
        printk(KERN_ERR "fdp_struct_cleanup: Internal error p_device is missing\n");
        return;
    }

    if (p_device->state == CUSTOM_OPENED) { /* it is opened */

        p_device->state = CUSTOM_PROBED;
    }
}

/**
  *  Function used to reset the chip
  *
  *  Prefered method is to use RESET, fallback to REFIOH
  *
  */

static void fdp_reset(struct fdp_custom_device *p_device)
{
    ENTER();

    if (!p_device) {
        printk(KERN_ERR "fdp_update_stack_state: Internal error p_device is missing\n");
        return;
    }

    if (gpio_is_valid(p_device->rst_gpio))
    {
        /* Reset RST/WakeUP for at least 2 micro-second */
        gpio_set_value(p_device->rst_gpio, RST_RESET);
        udelay(2);
        gpio_set_value(p_device->rst_gpio, RST_NO_RESET);

    }
}

/**
  *  Function used to initialize stack state management.
  *
  *  Prefered method is to use REFIOH, fallback to RESET
  *
  */

static int fdp_initialize_stack_state(struct fdp_custom_device *p_device)
{
    int rc = 0;

    ENTER();

    if (!p_device) {
        printk(KERN_ERR "fdp_initialize_stack_state: Internal error p_device is missing\n");
        return -1;
    }

    if (gpio_is_valid(p_device->rst_gpio)) {

        rc = gpio_direction_output(p_device->rst_gpio, RST_NO_RESET);
    }

    return rc;
}

/**
  *  Function used to update stack state.
  *
  *  Prefered method is to use REFIOH, fallback to RESET
  *
  */

static void fdp_update_stack_state(struct fdp_custom_device *p_device, uint8_t active)
{
    ENTER();

    if (!p_device) {
        printk(KERN_ERR "fdp_update_stack_state: Internal error p_device is missing\n");
        return;
    }
}

/**
  * Function called when the user opens /dev/nfcc
  *
  * @return 0 on success, a negative value on failure.
  */
int fdp_custom_open(struct inode *inode, struct file *filp)
{
    struct fdp_custom_device *p_device = fdp_p_device;
    int             retval = 0;

    ENTER();

    if (!p_device) {
        printk(KERN_ERR "fdp_custom_open: Internal error p_device is missing\n");
        return -ENODEV;
    }

    mutex_lock(&p_device->mutex);

    if (p_device->state == CUSTOM_INIT) {
        printk(KERN_ERR "I2C controller was not probed, unable to send i2c data to the FDP!\n");
        retval = -ENODEV;
        goto end;
    }

    if (p_device->state == CUSTOM_OPENED) {
        printk(KERN_ERR "The device is already opened\n");
        retval = -EBUSY;
        goto end;
    }

    filp->private_data = p_device;

    /* Initialize the context for a new session */
    fdp_struct_initialize(p_device);

    p_device->state = CUSTOM_OPENED;

    /* Inform the chip that the stack is active */
    fdp_update_stack_state(p_device, 1);

end:
    mutex_unlock(&p_device->mutex);

    return retval;
}

/**
  * Function called when the user reads data
  *
  * @return 0 on success, a negative value on failure.
  */
ssize_t fdp_custom_read(struct file * filp, char __user * buf, size_t count, loff_t * f_pos)
{
    struct fdp_custom_device *p_device = filp->private_data;
    ssize_t         retval;

    ENTER();

    if (!p_device) {
        printk(KERN_ERR "fdp_custom_read: Internal error p_device is missing\n");
        return -ENODEV;
    }

    mutex_lock(&p_device->mutex);

    if (!p_device->rx_data_length[p_device->next_to_read]) {
        /* no data available */
        retval = -EAGAIN;
        goto end;
    }

    /* we have data to read in p_device->rx_buffer[p_device->next_to_read] */

    if (count < (p_device->rx_data_length[p_device->next_to_read] - 3)) {
        /* supplied buffer is too small */
        printk(KERN_ERR "fdp_custom_read : provided buffer too short.\n");
        retval = -ENOSPC;
        goto end;
    }

    if (copy_to_user(buf, & p_device->rx_buffer[p_device->next_to_read][2], p_device->rx_data_length[p_device->next_to_read] - 3)) {
        printk(KERN_ERR "fdp_custom_read : unable to access to user buffer.\n");
        retval = -EFAULT;
        goto end;
    }

    retval = p_device->rx_data_length[p_device->next_to_read] - 3;
    p_device->rx_data_length[p_device->next_to_read] = 0;

    if (p_device->next_to_read < (RCV_BUFFER_NB - 1))
    {
        p_device->next_to_read++;
    }
    else
    {
        p_device->next_to_read = 0;
    }

end:
    mutex_unlock(&p_device->mutex);

    return retval;
}

/**
  * Function called when the user writes data
  *
  * @return 0 on success, a negative value on failure.
  */

ssize_t fdp_custom_write(struct file * filp, const char __user * buf, size_t count, loff_t * f_pos)
{
    struct fdp_custom_device *p_device = filp->private_data;
    int             retval;
    uint8_t         tmpbuf[I2C_WRITE_DATA_LENGTH];
    int             i;

    ENTER();

    if (!p_device) {
        printk(KERN_ERR "fdp_custom_write: Internal error p_device is missing\n");
        return -ENODEV;
    }

    if (count > I2C_WRITE_DATA_LENGTH - 3)
    {
        printk(KERN_ERR "fdp_custom_write: buffer too long\n");
        return -EFAULT;
    }

    tmpbuf[0] =  count >> 8;
    tmpbuf[1] =  count & 0xFF;

    if (copy_from_user(&tmpbuf[2], buf, count) != 0) {
        printk(KERN_ERR "fdp_custom_write  : copy_from_user failed\n");
        retval = -EFAULT;
        goto end;
    }

    tmpbuf[2 + count] = 0;

    for (i=0; i<2+count; i++)
    {
        tmpbuf[2 + count] ^= tmpbuf[i];
    }

    mutex_lock(&p_device->mutex);

    retval = i2c_master_send(p_device->i2c_client, tmpbuf, count + 3);

    if (retval != count + 3) {
        printk(KERN_ERR "fdp_custom_write : i2c_master_send() failed (returns %d)\n", retval);
        retval = -EFAULT;
    }

    mutex_unlock(&p_device->mutex);

end:

    return retval;
}

/**
  * Processes the OPEN_NFC_IOC_RESET ioctl
  *
  * @return 0 on success, a negative value on failure.
  */

long fdp_custom_ioctl_reset(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct fdp_custom_device *p_device = filp->private_data;
    int             rc = 0;

    ENTER();

    if (!p_device) {
        printk(KERN_ERR "fdp_custom_ioctl_reset: Internal error p_device is missing\n");
        return -ENODEV;
    }

    disable_irq(p_device->irqout);

    mutex_lock(&p_device->mutex);

    fdp_reset(p_device);

    /* Reinitialize context for a new session */
    fdp_struct_initialize(p_device);

    enable_irq(p_device->irqout);

    mutex_unlock(&p_device->mutex);

    return rc;
}

/**
  * Process the poll()
  *
  * @return the poll status
  */

unsigned int fdp_custom_poll(struct file *filp, poll_table * wait)
{
    struct fdp_custom_device *p_device = filp->private_data;
    unsigned int    mask = 0;

    ENTER();

    if (!p_device) {
        printk(KERN_ERR "fdp_custom_poll: Internal error p_device is missing\n");
        return -ENODEV;
    }

    /* We accept the function be called in all states */
    poll_wait(filp, &p_device->read_queue, wait);

    mutex_lock(&p_device->mutex);

    if (p_device->rx_data_length[p_device->next_to_read]) {
        mask |= POLLIN | POLLRDNORM;
    }

    mutex_unlock(&p_device->mutex);

    return mask;
}

/**
  * Function called when the user closes file
  *
  * @return 0 on success, a negative value on failure.
  */
int fdp_custom_release(struct inode *inode, struct file *filp)
{
    struct fdp_custom_device *p_device = filp->private_data;

    ENTER();

    if (!p_device) {
        printk(KERN_ERR "fdp_custom_release: Internal error p_device is missing\n");
        return -ENODEV;
    }

    mutex_lock(&p_device->mutex);

    /* Inform the chip the stack is no longer active */
    fdp_update_stack_state(p_device, 0);

    /* Go back to PROBED state */
    fdp_struct_cleanup(p_device);

    mutex_unlock(&p_device->mutex);
    return 0;
}

/**
  * Perform an I2C Read cycle, upon IRQOUT.
  *
  * @return void
  */
static void fdp_irqout_read(struct fdp_custom_device *p_device)
{
    int rc = 0, i;
    uint8_t lrc;
    uint8_t * p_buffer;

    /* Do NOT interrupt an outgoing WRITE cycle */
    mutex_lock(&p_device->mutex);

    while (gpio_get_value(p_device->irq_gpio)) {

        /* IRQ is high, FDP has some data for us */

        if ((p_device->state == CUSTOM_OPENED) && (! p_device->rx_data_length[p_device->next_to_write]))
        {
            p_buffer = p_device->rx_buffer[p_device->next_to_write];
        }
        else
        {
            /* The buffer pool is full - use scratch buffer */
            p_buffer = p_device->rx_scratch_buffer;
        }

        rc = i2c_master_recv(p_device->i2c_client, p_buffer, p_device->next_receive_length);

        if (rc != p_device->next_receive_length) {

            /* We did not received the amount of bytes expected.
             * This is very strange, since FDP should never NAK on reception
             * A transmission error during I2C ACK bit may explain this situation
             * ... */

            printk(KERN_ERR "fdp_irqout_read: i2c_master_recv() failed (returns %d)\n", rc);

            p_device->next_receive_length = 5;
            continue;
        }

        /* Check the received I2C paquet integrity */

        for (lrc=i=0; i<rc; i++)
            lrc ^= p_buffer[i];

        if (lrc) {

            /* LRC check fails.
             * This may due to transmission error or desynchronization between driver and FDP.
             * Drop the paquet and force resynchronization */

            printk(KERN_ERR "fdp_irqout_read: corrupted packet\n");

            p_device->next_receive_length = 5;
            continue;
        }

        /* All is fine, process the received paquet */

        if ( (p_buffer[0] == 0x00) && (p_buffer[1] == 0x00)) {

            /* We received a I2C frame that carries a length */

            /* Compute the next I2C packet size */
            p_device->next_receive_length = (p_buffer[2] << 8) + p_buffer[3] + 3;
        }
        else {

            /* We received a I2C data  */

            /* Next expected packet is a I2C length */
            p_device->next_receive_length = 5;

            if ( ((p_buffer[0]<< 8) + p_buffer[1]) != (rc - 3)) {

                /* Length specified in the data packet does not match the expected length
                 * This is very strange. */

                printk(KERN_ERR "fdp_irqout_read : unexpected data frame length");
                continue;
            }

            if (p_buffer != p_device->rx_scratch_buffer)
            {
                /* store received data length for user read */
                p_device->rx_data_length[p_device->next_to_write] = rc;

                if (p_device->next_to_write < (RCV_BUFFER_NB - 1))
                {
                    p_device->next_to_write++;
                }
                else
                {
                    p_device->next_to_write = 0;
                }

                /* wake up poll() */
                wake_up(&p_device->read_queue);
            }
        }
    };

    mutex_unlock(&p_device->mutex);
}

/**
  * IRQOUT Interrupt handler.
  * @return the IRQ processing status
  */

static irqreturn_t fdp_i2c_irq_thread_fn(int irq, void *dev_id)
{
    struct fdp_custom_device *p_device = fdp_p_device;

    if (!p_device) {
        printk(KERN_ERR "fdp_i2c_interrupt: Internal error p_device is missing, disabing the IRQ\n");
    } else {
        fdp_irqout_read(p_device);
    }

    return IRQ_HANDLED;
}

/**
  * Device/Driver binding: probe
  *
  * @return 0 if successfull, or error code
  */
static int fdp_probe(struct i2c_client *client, const struct i2c_device_id *idp)
{
    int    rc = 0;
    struct fdp_custom_device *p_device = fdp_p_device;
    struct fdp_i2c_platform_data * p_platform_data = dev_get_platdata(&client->dev);

    printk (KERN_INFO "fdp_probe: IRQ %d", client->irq);
    ENTER();

    if (!p_device) {
        printk(KERN_ERR "fdp_probe: Internal error p_device is missing\n");
        return -ENODEV;
    }

    if (!p_platform_data) {
        printk(KERN_ERR "fdp_probe: missing platform data");
        return -EINVAL;
    }

    mutex_lock(&p_device->mutex);

    if (p_device->state != CUSTOM_INIT) {
        rc = EEXIST;
        goto unlock;
    }

    p_device->i2c_client = client;
    i2c_set_clientdata(client, p_device);

    rc = gpio_request(p_platform_data->irq_gpio, NFC_HOST_INT_GPIO);
    if (rc) {
        dev_err(&client->dev, "Request NFC IRQOUT GPIO fails %d\n", rc);
        rc = -ENODEV;
        goto unlock;
    }

    rc = gpio_direction_input(p_platform_data->irq_gpio);
    if (rc) {
        dev_err(&client->dev, "Set IRQ GPIO direction fails %d\n", rc);
        rc = -ENODEV;
        goto err_int_gpio;
    }

    /* Map IRQ nb to GPIO id */
    client->irq = gpio_to_irq(p_platform_data->irq_gpio);

    rc = gpio_request(p_platform_data->rst_gpio, NFC_RESET_GPIO);
    if (rc) {
        dev_err(&client->dev,
            "Request for NFC Reset GPIO fails %d\n", rc);
        rc = -ENODEV;
        goto err_int_gpio;
    }

    rc = gpio_direction_output(p_platform_data->rst_gpio, RST_RESET);
    if (rc) {
        dev_err(&client->dev, "Set Reset GPIO direction fails %d\n", rc);
        rc = -ENODEV;
        goto err_rst_gpio;
    }

    p_device->rst_gpio = p_platform_data->rst_gpio;
    p_device->irq_gpio = p_platform_data->irq_gpio;
    printk(KERN_INFO "RST IRQ gpios %d %d\n", p_device->rst_gpio,
        p_device->irq_gpio);

    /* Phone ON/OFF management */
    rc = fdp_initialize_stack_state(p_device);

    if (rc < 0)
    {
        printk(KERN_ERR "fail to initialize stack state management\n");
        goto unlock;
    }

    p_device->irqout = client->irq;

    p_device->next_receive_length = I2C_LENGTH_FRAME_SIZE;

    rc = request_threaded_irq(client->irq, NULL, fdp_i2c_irq_thread_fn, IRQF_TRIGGER_RISING | IRQF_ONESHOT, "IRQOUT_input", p_device);

    if (rc < 0) {
        printk(KERN_ERR "fdp_probe : failed to register IRQOUT\n");
        goto unlock;
    }

    /* enable to wake the device using this IRQ */
    rc = irq_set_irq_wake(client->irq, 1);
    if (rc < 0) {
        printk(KERN_ERR "fdp_probe : failed to set irq as wake up source");
    }

    /* The IRQ can be trigged here at most once, because we are holding the mutex (cannot be re-enabled after disable) */
    p_device->state = CUSTOM_PROBED;

err_rst_gpio:
    gpio_free(p_platform_data->rst_gpio);
err_int_gpio:
    gpio_free(p_platform_data->irq_gpio);
unlock:
    mutex_unlock(&p_device->mutex);

    return rc;
}

int fdp_suspend(struct i2c_client * client, pm_message_t mesg)
{
    struct fdp_custom_device *p_device = fdp_p_device;

    printk(KERN_INFO "fdp_suspend\n");

    if (! p_device)
    {
        printk(KERN_ERR "fdp_suspend: Internal error p_device is missing\n");
        return 0;
    }

    if (p_device->state == CUSTOM_OPENED)
    {
        //disable_irq(p_device->irqout);
    }

    return 0;
}

int fdp_resume(struct i2c_client * client)
{
    struct fdp_custom_device *p_device = fdp_p_device;

    printk(KERN_INFO "fdp_resume\n");

    if (! p_device)
    {
        printk(KERN_ERR "fdp_resume: Internal error p_device is missing\n");
        return 0;
    }

    if (p_device->state == CUSTOM_OPENED)
    {
        //enable_irq(p_device->irqout);
    }

    return 0;
}

/**
  * Device/Driver binding: remove
  *
  * @return 0 if successfull, or error code
  */
static int fdp_remove(struct i2c_client *client)
{
    struct fdp_custom_device *p_device;

    ENTER();

    p_device = fdp_p_device;

    /* disable to wake the device using this IRQ */
    irq_set_irq_wake(p_device->irqout, 0);

    /* free the IRQ */
    free_irq(p_device->irqout, p_device);

    if (gpio_is_valid(p_device->irq_gpio))
        gpio_free(p_device->irq_gpio);

    if (gpio_is_valid(p_device->rst_gpio))
        gpio_free(p_device->rst_gpio);

    if (p_device && (p_device->state >= CUSTOM_PROBED)) {
        i2c_set_clientdata(p_device->i2c_client, NULL);
    }

    p_device->state = CUSTOM_INIT;

    return 0;
}

/*
   Client structure holds device-specific information like the
   driver model device node, and its I2C address
*/
static const struct i2c_device_id fdp_id[] = {
    { I2C_ID_NAME, I2C_DEVICE_ADDR  },
    { }
};

MODULE_DEVICE_TABLE(i2c, fdp_id);

static struct i2c_driver fdp_i2c_driver = {
    .driver = {
           .owner = THIS_MODULE,
           .name = I2C_ID_NAME,
    },
    .probe = fdp_probe,
    .remove = __devexit_p(fdp_remove),
    .suspend = fdp_suspend,
    .resume = fdp_resume,
    .id_table = fdp_id,

};

/**
  * Specific initialization, when driver module is inserted.
  *
  * @return 0 if successfull, or error code
  */
int
fdp_custom_init(void)
{
    int             retval = 0;

    struct fdp_custom_device *p_device;

    ENTER();

    p_device = kmalloc(sizeof(struct fdp_custom_device), GFP_KERNEL);
    if (p_device == NULL) {
        return -ENOMEM;
    }
    memset(p_device, 0, sizeof(struct fdp_custom_device));

    init_waitqueue_head(&p_device->read_queue);
    mutex_init(&p_device->mutex);

    /* Save device context (needed in .probe()) */
    fdp_p_device = p_device;

    retval = i2c_add_driver(&fdp_i2c_driver);
    if (retval < 0) {
        printk(KERN_ERR "fdp_custom_init : failed to add I2C driver\n");
        goto end;
    }

    printk(KERN_INFO "fdp driver loaded\n");

end:
    return retval;
}

/**
  * Specific cleanup, when driver module is removed.
  *
  * @return void
  */
void fdp_custom_exit(void)
{
    struct fdp_custom_device *p_device;

    ENTER();

    p_device = fdp_p_device;
    if (p_device == NULL)
        return;

    i2c_del_driver(&fdp_i2c_driver);
    mutex_destroy(&p_device->mutex);

    /* free the custom device context */
    kfree(p_device);

    fdp_p_device = NULL;

}

/* EOF */
