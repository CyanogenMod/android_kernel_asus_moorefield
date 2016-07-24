#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/hwmon-sysfs.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <asm/intel-mid.h>
#include <asm/intel_mid_powerbtn.h>

#define DRIVER_NAME "hall_sensor"
//#define DISABLE_POWER_BUTTON

extern int Read_Boot_Mode(void);

static struct workqueue_struct *hall_sensor_wq;
static struct workqueue_struct *hall_sensor_do_wq = NULL;
static struct kobject *hall_sensor_kobj;
static struct platform_device *pdev;

static struct input_device_id mID[] = {
        { .driver_info = 1 },		//scan all device to match hall sensor
        { },
};

static struct hall_sensor_str {
 	int irq;
	int status;
	int gpio;
	int enable; 
	spinlock_t mHallSensorLock;
	struct wake_lock wake_lock;
	struct input_dev *lid_indev;
	struct input_handler lid_handler;
	struct input_handle lid_handle;
 	struct delayed_work hall_sensor_work;
 	struct delayed_work hall_sensor_dowork;
	
}* hall_sensor_dev;

static void hall_sensor_shutdown(struct platform_device *pdev);
int lid_connect(struct input_handler *handler, struct input_dev *dev, const struct input_device_id *id){
	printk("[%s] hall_sensor connect to handler\n", DRIVER_NAME);
	return 0;
}

void lid_event(struct input_handle *handle, unsigned int type, unsigned int code, int value){
	if(type==EV_SW && code==SW_LID ){
     		if(value != 2 && !!test_bit(code, hall_sensor_dev->lid_indev->sw) != !hall_sensor_dev->status){
			__change_bit(code,  hall_sensor_dev->lid_indev->sw);
			printk("[%s] reset dev->sw=%d \n", DRIVER_NAME,!hall_sensor_dev->status);
		}
	}
}

bool lid_match(struct input_handler *handler, struct input_dev *dev){
	if(dev->name && handler->name)
		if(!strcmp(dev->name,"lid_input") && !strcmp(handler->name,"lid_input_handler"))
		        return true;
		
	return false;
}

static ssize_t show_action_status(struct device *dev,struct device_attribute *attr, char *buf)
{
	if(!hall_sensor_dev)
		return sprintf(buf, "Hall sensor does not exist!\n");
	return sprintf(buf, "%d\n",hall_sensor_dev->status);
}
static ssize_t store_action_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int request;
	unsigned long flags;
	if(!hall_sensor_dev)
                return sprintf(buf, "Hall sensor does not exist!\n");
        sscanf(buf, "%du", &request);
        spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
        if (!request)
                hall_sensor_dev->status = 0;
	else
        	hall_sensor_dev->status = 1;
	spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);
        pr_info("[%s] SW_LID rewite value = %d\n", DRIVER_NAME,!hall_sensor_dev->status);
	return count;
}

static ssize_t show_hall_sensor_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
        if(!hall_sensor_dev)
                return sprintf(buf, "Hall sensor does not exist!\n");
	return sprintf(buf, "%d\n",hall_sensor_dev->enable);
}

static ssize_t store_hall_sensor_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int request;
	if(!hall_sensor_dev)
                return sprintf(buf, "Hall sensor does not exist!\n");
	sscanf(buf, "%du", &request);
	if(request==hall_sensor_dev->enable){
		return count;
	}
	else {
		unsigned long flags;
		spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
		if (hall_sensor_dev->enable==0){
			enable_irq(hall_sensor_dev->irq);
			hall_sensor_dev->enable=1;
		}
		else if (hall_sensor_dev->enable==1){		
			disable_irq(hall_sensor_dev->irq);
			hall_sensor_dev->enable=0;
		}
		spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);
	}
	return count;
}


static SENSOR_DEVICE_ATTR_2(action_status, S_IRUGO|S_IWUSR, show_action_status, store_action_status, 0, 0);
static SENSOR_DEVICE_ATTR_2(activity, S_IRUGO|S_IWUSR,show_hall_sensor_enable, store_hall_sensor_enable, 0, 0);

static struct attribute *hall_sensor_attrs[] = {
	&sensor_dev_attr_action_status.dev_attr.attr,
	&sensor_dev_attr_activity.dev_attr.attr,
	NULL
};

static struct attribute_group hall_sensor_group = {
	.name = "hall_sensor",
	.attrs = hall_sensor_attrs
};

static int lid_input_device_create(void)
{
	int err = 0;

	hall_sensor_dev->lid_indev = input_allocate_device();     
	if(!hall_sensor_dev->lid_indev){
		pr_info("[%s] lid_indev allocation fails\n", DRIVER_NAME);
		err = -ENOMEM;
		goto exit;
	}

	hall_sensor_dev->lid_indev->name = "lid_input";
	hall_sensor_dev->lid_indev->phys= "/dev/input/lid_indev";
	hall_sensor_dev->lid_indev->dev.parent= NULL;
	input_set_capability(hall_sensor_dev->lid_indev, EV_SW, SW_LID);

	err = input_register_device(hall_sensor_dev->lid_indev);
	if (err) {
		pr_info("[%s] input registration fails\n", DRIVER_NAME);
		err = -1;
		goto exit_input_free;
	}
	hall_sensor_dev->lid_handler.match=lid_match;
	hall_sensor_dev->lid_handler.connect=lid_connect;
	hall_sensor_dev->lid_handler.event=lid_event;
	hall_sensor_dev->lid_handler.name="lid_input_handler";
	hall_sensor_dev->lid_handler.id_table = mID;	
	err=input_register_handler(& hall_sensor_dev->lid_handler);
	if(err){
		pr_info("[%s] handler registration fails\n", DRIVER_NAME);
		err = -1;
		goto exit_unregister_input_dev;
	}
	hall_sensor_dev->lid_handle.name="lid_handle";
	hall_sensor_dev->lid_handle.open=1;         //receive any event from hall sensor
	hall_sensor_dev->lid_handle.dev=hall_sensor_dev->lid_indev;
	hall_sensor_dev->lid_handle.handler=&hall_sensor_dev->lid_handler;
	err=input_register_handle(& hall_sensor_dev->lid_handle);
	if(err){
		pr_info("[%s] handle registration fails\n", DRIVER_NAME);
		err = -1;
		goto exit_unregister_handler;
	}
	return 0;

exit_unregister_handler:
       input_unregister_handler(& hall_sensor_dev->lid_handler);
exit_unregister_input_dev:
       input_unregister_device(hall_sensor_dev->lid_indev);
exit_input_free:
       input_free_device(hall_sensor_dev->lid_indev);
       hall_sensor_dev->lid_indev = NULL;
exit:
       return err;
}



static void lid_do_work_function(struct work_struct *dat)
{
	pr_info("[%s] hall_sensor_interrupt = %d\n", DRIVER_NAME,hall_sensor_dev->irq);
	cancel_delayed_work(&hall_sensor_dev->hall_sensor_work);
	queue_delayed_work(hall_sensor_wq, &hall_sensor_dev->hall_sensor_work, 0);
}


static void lid_report_function(struct work_struct *dat)
{
     	unsigned long flags;
	int counter, status = 0, initial_status;
	initial_status = hall_sensor_dev->status;
	for (counter = 0;counter < 3;counter++){
		msleep(50);
        	spin_lock_irqsave(&hall_sensor_dev->mHallSensorLock, flags);
        	if (gpio_get_value(hall_sensor_dev->gpio) > 0){
			hall_sensor_dev->status = 1;
			status++;
        	}else{
                	hall_sensor_dev->status = 0;
		}
		pr_info("[%s] SW_LID check value = %d\n", DRIVER_NAME,!hall_sensor_dev->status);
        	spin_unlock_irqrestore(&hall_sensor_dev->mHallSensorLock, flags);
	}
	if((status > 0) && (status < 3)){
		pr_info("[%s] SW_LID do not report to framework.\n", DRIVER_NAME);
		hall_sensor_dev->status = initial_status;
        	input_report_switch(hall_sensor_dev->lid_indev, SW_LID, 2);
        	input_sync(hall_sensor_dev->lid_indev);
		wake_unlock(&hall_sensor_dev->wake_lock);
 		return;
	}

        input_report_switch(hall_sensor_dev->lid_indev, SW_LID, !hall_sensor_dev->status);
        input_sync(hall_sensor_dev->lid_indev);
#ifdef DISABLE_POWER_BUTTON
        if (hall_sensor_dev->status > 0)
		pwn_enable(true);
        else
                pwn_enable(false);
#endif
	wake_unlock(&hall_sensor_dev->wake_lock);
	wake_lock_timeout(&hall_sensor_dev->wake_lock, msecs_to_jiffies(3000));
	pr_info("[%s] SW_LID report value = %d\n", DRIVER_NAME,!hall_sensor_dev->status);
 

}

static irqreturn_t hall_sensor_interrupt_handler(int irq, void *dev_id)
{
	if (hall_sensor_do_wq==NULL)  return IRQ_NONE;
	pr_info("[%s] hall_sensor_interrupt_handler = %d\n", DRIVER_NAME,hall_sensor_dev->irq);
	queue_delayed_work(hall_sensor_do_wq, &hall_sensor_dev->hall_sensor_dowork, msecs_to_jiffies(0));
	wake_lock(&hall_sensor_dev->wake_lock);
	return IRQ_HANDLED;
}

static int set_irq_hall_sensor(void)
{
	int rc = 0 ;
	pr_info("[%s] hall_sensor gpio = %d\n", DRIVER_NAME,hall_sensor_dev->gpio);
	hall_sensor_dev->irq = gpio_to_irq(hall_sensor_dev->gpio);
	pr_info("[%s] hall_sensor irq = %d\n", DRIVER_NAME,hall_sensor_dev->irq);
	rc = request_irq(hall_sensor_dev->irq,hall_sensor_interrupt_handler,
			IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,"hall_sensor_irq",hall_sensor_dev);
	if (rc<0) {
		pr_info("[%s] Could not register for hall sensor interrupt, irq = %d, rc = %d\n", DRIVER_NAME,hall_sensor_dev->irq,rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}

	enable_irq_wake(hall_sensor_dev->irq);

	return 0;

err_gpio_request_irq_fail:
	return rc;
}



//+++++++++++++for pm_ops callback+++++++++++++++

static int lid_suspend_noirq(struct device *dev){
	return 0;
}

static int lid_suspend_prepare(struct device *dev){
	return 0;
}

static void lid_resume_complete(struct device *dev){

}

static int lid_probe(struct platform_device *pdev){
	//printk("[%s]lid_probe!\n",DRIVER_NAME);
	return 0;
}

static const struct dev_pm_ops lid_dev_pm_ops = {
	.prepare	 = lid_suspend_prepare,
	.suspend_noirq	 = lid_suspend_noirq,
	.complete	 = lid_resume_complete,
};

static const struct platform_device_id lid_id_table[] = {
        {DRIVER_NAME, 1},
};

static struct platform_driver lid_platform_driver = {
	.driver.name    = DRIVER_NAME,
	.driver.owner	= THIS_MODULE,
	.driver.pm      = &lid_dev_pm_ops,
	.shutdown	= hall_sensor_shutdown, 
	.probe          = lid_probe,
	.id_table	= lid_id_table,
};

//----------------for pm_ops callback----------------



static void hall_sensor_shutdown(struct platform_device *pdev)
{

        disable_irq(hall_sensor_dev->irq);
        gpio_free(hall_sensor_dev->gpio);
        destroy_workqueue(hall_sensor_do_wq);
        destroy_workqueue(hall_sensor_wq);
        input_free_device(hall_sensor_dev->lid_indev);
        wake_lock_destroy(&hall_sensor_dev->wake_lock);
        hall_sensor_dev->lid_indev=NULL;
        kfree(hall_sensor_dev);
        hall_sensor_dev=NULL;
        kobject_put(hall_sensor_kobj);
		
}

static int __init hall_sensor_init(void)
{	


	int ret;

	if (Read_Boot_Mode()==4) {  //do not enable hall sensor in charger mode
		return 0;
	}

	//insert pm_ops
	pdev= platform_device_alloc(DRIVER_NAME,-1);
	if (!pdev)
		return -1;
	ret = platform_device_add(pdev);
        if (ret) 
		return -1;
	ret = platform_driver_register(&lid_platform_driver);
	if (ret)
		return ret;

	//set file node
	hall_sensor_kobj = kobject_create_and_add("hall_sensor_kobject", kernel_kobj);
	if (!hall_sensor_kobj){
		pr_info("[%s] hall_sensor_kobject fails for hall sensor\n", DRIVER_NAME);
		platform_device_unregister(pdev);
		platform_driver_unregister(&lid_platform_driver);
		return -ENOMEM;
	}
	ret = sysfs_create_group(hall_sensor_kobj, &hall_sensor_group);
	if (ret){
		goto fail_for_hall_sensor;
	}

	//Memory allocation
	hall_sensor_dev = kzalloc(sizeof (struct hall_sensor_str), GFP_KERNEL);
	if (!hall_sensor_dev) {
		pr_info("[%s] Memory allocation fails for hall sensor\n", DRIVER_NAME);
		ret = -ENOMEM;
		goto fail_for_hall_sensor;
	}
	spin_lock_init(&hall_sensor_dev->mHallSensorLock);
	hall_sensor_dev->enable = 1;


	//set gpio
#if 0
	hall_sensor_dev->gpio = get_gpio_by_name("Hall_Det#");
#else
	hall_sensor_dev->gpio = 45;
#endif
	 if (!gpio_is_valid(hall_sensor_dev->gpio))
	{
		pr_info("[%s] GPIO for hall sensor does not exist.\n", DRIVER_NAME);
		ret= -1;
		goto fail_for_set_gpio_hall_sensor;
	}
	gpio_request(hall_sensor_dev->gpio,"hall_sensor_gpio");
	gpio_direction_input(hall_sensor_dev->gpio);

	//set irq
	ret = set_irq_hall_sensor();
	if (ret < 0)
		goto fail_for_irq_hall_sensor;

	//create input_dev
	hall_sensor_dev->lid_indev = NULL;
	ret = lid_input_device_create();
	if (ret < 0)
		goto fail_for_create_input_dev;

	//init workqueue & start detect signal
	hall_sensor_wq = create_singlethread_workqueue("hall_sensor_wq");
	hall_sensor_do_wq = create_singlethread_workqueue("hall_sensor_do_wq");
	INIT_DELAYED_WORK(&hall_sensor_dev->hall_sensor_work, lid_report_function);
	INIT_DELAYED_WORK(&hall_sensor_dev->hall_sensor_dowork, lid_do_work_function);


	queue_delayed_work(hall_sensor_do_wq, &hall_sensor_dev->hall_sensor_dowork, 0);
	wake_lock_init(&hall_sensor_dev->wake_lock, WAKE_LOCK_SUSPEND, "lid_suspend_blocker");
	return 0;
	
fail_for_create_input_dev:		
	free_irq(hall_sensor_dev->irq, hall_sensor_dev);	

fail_for_irq_hall_sensor:
	gpio_free(hall_sensor_dev->gpio);
fail_for_set_gpio_hall_sensor:
	kfree(hall_sensor_dev);
	hall_sensor_dev=NULL;

fail_for_hall_sensor:
	kobject_put(hall_sensor_kobj);
	return ret;

}

static void __exit hall_sensor_exit(void)
{
    disable_irq(hall_sensor_dev->irq);
	gpio_free(hall_sensor_dev->gpio);
    destroy_workqueue(hall_sensor_do_wq);
    destroy_workqueue(hall_sensor_wq);
	input_free_device(hall_sensor_dev->lid_indev);
    wake_lock_destroy(&hall_sensor_dev->wake_lock);
	hall_sensor_dev->lid_indev=NULL;
	kfree(hall_sensor_dev);
	hall_sensor_dev=NULL;
	kobject_put(hall_sensor_kobj);
	platform_driver_unregister(&lid_platform_driver);
	platform_device_unregister(pdev);
}


module_init(hall_sensor_init);
module_exit(hall_sensor_exit);


MODULE_DESCRIPTION("Intel Hall sensor Driver");
MODULE_LICENSE("GPL v2");
