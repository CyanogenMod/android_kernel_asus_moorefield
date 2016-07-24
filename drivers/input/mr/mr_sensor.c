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
#include <linux/HWVersion.h>


#define DRIVER_NAME "mr_sensor"
//#define DISABLE_POWER_BUTTON

static struct workqueue_struct *mr_sensor_wq;
static struct workqueue_struct *mr_sensor_do_wq;
static struct kobject *mr_sensor_kobj;
static struct platform_device *pdev;

static struct input_device_id mID[] = {
        { .driver_info = 1 },		//scan all device to match mr sensor
        { },
};

static struct mr_sensor_str {
 	int irq;
	int status;
	int gpio;
	int enable; 
	spinlock_t mMRSensorLock;
	struct wake_lock wake_lock;
	struct input_dev *mr_indev;
	struct input_handler mr_handler;
	struct input_handle mr_handle;
 	struct delayed_work mr_sensor_work;
 	struct delayed_work mr_sensor_dowork;
	
}* mr_sensor_dev;

static void mr_sensor_shutdown(struct platform_device *pdev);
int mr_connect(struct input_handler *handler, struct input_dev *dev, const struct input_device_id *id){
	printk("[%s] mr_sensor connect to handler\n", DRIVER_NAME);
	return 0;
}

void mr_event(struct input_handle *handle, unsigned int type, unsigned int code, int value){
	if(type==EV_SW && code==SW_CAMERA ){
     		if(!!test_bit(code, mr_sensor_dev->mr_indev->sw) != !mr_sensor_dev->status){
			__change_bit(code,  mr_sensor_dev->mr_indev->sw);
			printk("[%s] reset dev->sw=%d \n", DRIVER_NAME,!mr_sensor_dev->status);
		}
	}
}

bool mr_match(struct input_handler *handler, struct input_dev *dev){
	if(dev->name && handler->name)
		if(!strcmp(dev->name,"mr_input") && !strcmp(handler->name,"mr_input_handler"))
		        return true;
		
	return false;
}

static ssize_t show_action_status(struct device *dev,struct device_attribute *attr, char *buf)
{
	if(!mr_sensor_dev)
		return sprintf(buf, "MR sensor does not exist!\n");
	return sprintf(buf, "%d\n",mr_sensor_dev->status);
}
static ssize_t store_action_status(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int request;
	unsigned long flags;
	if(!mr_sensor_dev)
                return sprintf(buf, "MR sensor does not exist!\n");
        sscanf(buf, "%du", &request);
        spin_lock_irqsave(&mr_sensor_dev->mMRSensorLock, flags);
        if (!request)
                mr_sensor_dev->status = 0;
	else
        	mr_sensor_dev->status = 1;
	spin_unlock_irqrestore(&mr_sensor_dev->mMRSensorLock, flags);
        pr_info("[%s] SW_CAMERA rewite value = %d\n", DRIVER_NAME,!mr_sensor_dev->status);
	return count;
}

static ssize_t show_mr_sensor_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
        if(!mr_sensor_dev)
                return sprintf(buf, "MR sensor does not exist!\n");
	return sprintf(buf, "%d\n",mr_sensor_dev->enable);
}

static ssize_t store_mr_sensor_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int request;
	if(!mr_sensor_dev)
                return sprintf(buf, "MR sensor does not exist!\n");
	sscanf(buf, "%du", &request);
	if(request==mr_sensor_dev->enable){
		return count;
	}
	else {
		unsigned long flags;
		spin_lock_irqsave(&mr_sensor_dev->mMRSensorLock, flags);
		if (mr_sensor_dev->enable==0){
			enable_irq(mr_sensor_dev->irq);
			mr_sensor_dev->enable=1;
		}
		else if (mr_sensor_dev->enable==1){		
			disable_irq(mr_sensor_dev->irq);
			mr_sensor_dev->enable=0;
		}
		spin_unlock_irqrestore(&mr_sensor_dev->mMRSensorLock, flags);
	}
	return count;
}


static SENSOR_DEVICE_ATTR_2(action_status, S_IRUGO|S_IWUSR, show_action_status, store_action_status, 0, 0);
static SENSOR_DEVICE_ATTR_2(activity, S_IRUGO|S_IWUSR,show_mr_sensor_enable, store_mr_sensor_enable, 0, 0);

static struct attribute *mr_sensor_attrs[] = {
	&sensor_dev_attr_action_status.dev_attr.attr,
	&sensor_dev_attr_activity.dev_attr.attr,
	NULL
};

static struct attribute_group mr_sensor_group = {
	.name = "mr_sensor",
	.attrs = mr_sensor_attrs
};

static int mr_input_device_create(void)
{
	int err = 0;

	mr_sensor_dev->mr_indev = input_allocate_device();     
	if(!mr_sensor_dev->mr_indev){
		pr_info("[%s] mr_indev allocation fails\n", DRIVER_NAME);
		err = -ENOMEM;
		goto exit;
	}

	mr_sensor_dev->mr_indev->name = "mr_input";
	mr_sensor_dev->mr_indev->phys= "/dev/input/mr_indev";
	mr_sensor_dev->mr_indev->dev.parent= NULL;
	input_set_capability(mr_sensor_dev->mr_indev, EV_SW, SW_CAMERA);

	err = input_register_device(mr_sensor_dev->mr_indev);
	if (err) {
		pr_info("[%s] input registration fails\n", DRIVER_NAME);
		err = -1;
		goto exit_input_free;
	}
	mr_sensor_dev->mr_handler.match=mr_match;
	mr_sensor_dev->mr_handler.connect=mr_connect;
	mr_sensor_dev->mr_handler.event=mr_event;
	mr_sensor_dev->mr_handler.name="mr_input_handler";
	mr_sensor_dev->mr_handler.id_table = mID;	
	err=input_register_handler(& mr_sensor_dev->mr_handler);
	if(err){
		pr_info("[%s] handler registration fails\n", DRIVER_NAME);
		err = -1;
		goto exit_unregister_input_dev;
	}
	mr_sensor_dev->mr_handle.name="mr_handle";
	mr_sensor_dev->mr_handle.open=1;         //receive any event from mr sensor
	mr_sensor_dev->mr_handle.dev=mr_sensor_dev->mr_indev;
	mr_sensor_dev->mr_handle.handler=&mr_sensor_dev->mr_handler;
	err=input_register_handle(& mr_sensor_dev->mr_handle);
	if(err){
		pr_info("[%s] handle registration fails\n", DRIVER_NAME);
		err = -1;
		goto exit_unregister_handler;
	}
	return 0;

exit_unregister_handler:
       input_unregister_handler(& mr_sensor_dev->mr_handler);
exit_unregister_input_dev:
       input_unregister_device(mr_sensor_dev->mr_indev);
exit_input_free:
       input_free_device(mr_sensor_dev->mr_indev);
       mr_sensor_dev->mr_indev = NULL;
exit:
       return err;
}



static void mr_do_work_function(struct work_struct *dat)
{
	pr_info("[%s] mr_sensor_interrupt = %d\n", DRIVER_NAME,mr_sensor_dev->irq);
	cancel_delayed_work(&mr_sensor_dev->mr_sensor_work);
	queue_delayed_work(mr_sensor_wq, &mr_sensor_dev->mr_sensor_work, 0);
}


static void mr_report_function(struct work_struct *dat)
{
     	unsigned long flags;
	int counter, status = 0, initial_status;
	initial_status = mr_sensor_dev->status;
	for (counter = 0;counter < 3;counter++){
		msleep(50);
        	spin_lock_irqsave(&mr_sensor_dev->mMRSensorLock, flags);
        	if (gpio_get_value(mr_sensor_dev->gpio) > 0){
			mr_sensor_dev->status = 1;
			status++;
        	}else{
                	mr_sensor_dev->status = 0;
		}
		pr_info("[%s] SW_CAMERA check value = %d\n", DRIVER_NAME,!mr_sensor_dev->status);
        	spin_unlock_irqrestore(&mr_sensor_dev->mMRSensorLock, flags);
	}
	if((status > 0) && (status < 3)){
		pr_info("[%s] SW_CAMERA do not report to framework.\n", DRIVER_NAME);
		mr_sensor_dev->status = initial_status;
		wake_unlock(&mr_sensor_dev->wake_lock);
 		return;
	}

        input_report_switch(mr_sensor_dev->mr_indev, SW_CAMERA, !mr_sensor_dev->status);
        input_sync(mr_sensor_dev->mr_indev);
#ifdef DISABLE_POWER_BUTTON
        if (mr_sensor_dev->status > 0)
		pwn_enable(true);
        else
                pwn_enable(false);
#endif
	wake_unlock(&mr_sensor_dev->wake_lock);
	wake_lock_timeout(&mr_sensor_dev->wake_lock, msecs_to_jiffies(3000));
	pr_info("[%s] SW_CAMERA report value = %d\n", DRIVER_NAME,!mr_sensor_dev->status);
 

}

static irqreturn_t mr_sensor_interrupt_handler(int irq, void *dev_id)
{
	pr_info("[%s] mr_sensor_interrupt_handler = %d\n", DRIVER_NAME,mr_sensor_dev->irq);
	queue_delayed_work(mr_sensor_do_wq, &mr_sensor_dev->mr_sensor_dowork, msecs_to_jiffies(0));
	wake_lock(&mr_sensor_dev->wake_lock);
	return IRQ_HANDLED;
}

static int set_irq_mr_sensor(void)
{
	int rc = 0 ;
	pr_info("[%s] mr_sensor gpio = %d\n", DRIVER_NAME,mr_sensor_dev->gpio);
	mr_sensor_dev->irq = gpio_to_irq(mr_sensor_dev->gpio);
	pr_info("[%s] mr_sensor irq = %d\n", DRIVER_NAME,mr_sensor_dev->irq);
	rc = request_irq(mr_sensor_dev->irq,mr_sensor_interrupt_handler,
			IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING,"mr_sensor_irq",mr_sensor_dev);
	if (rc<0) {
		pr_info("[%s] Could not register for mr sensor interrupt, irq = %d, rc = %d\n", DRIVER_NAME,mr_sensor_dev->irq,rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}

	enable_irq_wake(mr_sensor_dev->irq);

	return 0;

err_gpio_request_irq_fail:
	return rc;
}



//+++++++++++++for pm_ops callback+++++++++++++++

static int mr_suspend_noirq(struct device *dev){
	return 0;
}

static int mr_suspend_prepare(struct device *dev){
	return 0;
}

static void mr_resume_complete(struct device *dev){

}

static int mr_probe(struct platform_device *pdev){
	//printk("[%s]mr_probe!\n",DRIVER_NAME);
	return 0;
}

static const struct dev_pm_ops mr_dev_pm_ops = {
	.prepare	 = mr_suspend_prepare,
	.suspend_noirq	 = mr_suspend_noirq,
	.complete	 = mr_resume_complete,
};

static const struct platform_device_id mr_id_table[] = {
        {DRIVER_NAME, 1},
};

static struct platform_driver mr_platform_driver = {
	.driver.name    = DRIVER_NAME,
	.driver.owner	= THIS_MODULE,
	.driver.pm      = &mr_dev_pm_ops,
	.shutdown	= mr_sensor_shutdown, 
	.probe          = mr_probe,
	.id_table	= mr_id_table,
};

//----------------for pm_ops callback----------------



static void mr_sensor_shutdown(struct platform_device *pdev)
{

        disable_irq(mr_sensor_dev->irq);
        gpio_free(mr_sensor_dev->gpio);
        destroy_workqueue(mr_sensor_do_wq);
        destroy_workqueue(mr_sensor_wq);
        input_free_device(mr_sensor_dev->mr_indev);
        wake_lock_destroy(&mr_sensor_dev->wake_lock);
        mr_sensor_dev->mr_indev=NULL;
        kfree(mr_sensor_dev);
        mr_sensor_dev=NULL;
        kobject_put(mr_sensor_kobj);
		
}

static int __init mr_sensor_init(void)
{	


	int ret;

    	switch (Read_PROJ_ID()) {

	    case PROJ_ID_ZE500ML:
	    case PROJ_ID_ZE550ML:
	    case PROJ_ID_ZE551ML:
	    case PROJ_ID_ZX550ML:
		pr_info("Project ID is NOT ZR550ML, MR sesnor exit...\n");
		return -1;
	    break;
	    default:
		pr_info("Project ID is NOT ZR550ML, MR sesnor exit...\n");
		return -1;
	    break;
    	}//end switch

	pdev= platform_device_alloc(DRIVER_NAME,-1);
	if (!pdev)
		return -1;
	ret = platform_device_add(pdev);
        if (ret) 
		return -1;
	ret = platform_driver_register(&mr_platform_driver);
	if (ret)
		return ret;

	//set file node
	mr_sensor_kobj = kobject_create_and_add("mr_sensor_kobject", kernel_kobj);
	if (!mr_sensor_kobj){
		pr_info("[%s] mr_sensor_kobject fails for mr sensor\n", DRIVER_NAME);
		platform_device_unregister(pdev);
		platform_driver_unregister(&mr_platform_driver);
		return -ENOMEM;
	}
	ret = sysfs_create_group(mr_sensor_kobj, &mr_sensor_group);
	if (ret){
		goto fail_for_mr_sensor;
	}

	//Memory allocation
	mr_sensor_dev = kzalloc(sizeof (struct mr_sensor_str), GFP_KERNEL);
	if (!mr_sensor_dev) {
		pr_info("[%s] Memory allocation fails for mr sensor\n", DRIVER_NAME);
		ret = -ENOMEM;
		goto fail_for_mr_sensor;
	}
	spin_lock_init(&mr_sensor_dev->mMRSensorLock);
	mr_sensor_dev->enable = 1;


	//set gpio
#if 0
	mr_sensor_dev->gpio = get_gpio_by_name("MR_Det#");
#else
	mr_sensor_dev->gpio = 48;
#endif
	 if (!gpio_is_valid(mr_sensor_dev->gpio))
	{
		pr_info("[%s] GPIO for mr sensor does not exist.\n", DRIVER_NAME);
		ret= -1;
		goto fail_for_set_gpio_mr_sensor;
	}
	gpio_request(mr_sensor_dev->gpio,"mr_sensor_gpio");
	gpio_direction_input(mr_sensor_dev->gpio);

	//set irq
	ret = set_irq_mr_sensor();
	if (ret < 0)
		goto fail_for_irq_mr_sensor;

	//create input_dev
	mr_sensor_dev->mr_indev = NULL;
	ret = mr_input_device_create();
	if (ret < 0)
		goto fail_for_create_input_dev;

	//init workqueue & start detect signal
	mr_sensor_wq = create_singlethread_workqueue("mr_sensor_wq");
	mr_sensor_do_wq = create_singlethread_workqueue("mr_sensor_do_wq");
	INIT_DELAYED_WORK(&mr_sensor_dev->mr_sensor_work, mr_report_function);
	INIT_DELAYED_WORK(&mr_sensor_dev->mr_sensor_dowork, mr_do_work_function);


	queue_delayed_work(mr_sensor_do_wq, &mr_sensor_dev->mr_sensor_dowork, 0);
	wake_lock_init(&mr_sensor_dev->wake_lock, WAKE_LOCK_SUSPEND, "mr_suspend_blocker");
	return 0;
	
fail_for_create_input_dev:		
	free_irq(mr_sensor_dev->irq, mr_sensor_dev);	

fail_for_irq_mr_sensor:
	gpio_free(mr_sensor_dev->gpio);
fail_for_set_gpio_mr_sensor:
	kfree(mr_sensor_dev);
	mr_sensor_dev=NULL;

fail_for_mr_sensor:
	kobject_put(mr_sensor_kobj);
	return ret;

}

static void __exit mr_sensor_exit(void)
{
    disable_irq(mr_sensor_dev->irq);
	gpio_free(mr_sensor_dev->gpio);
    destroy_workqueue(mr_sensor_do_wq);
    destroy_workqueue(mr_sensor_wq);
	input_free_device(mr_sensor_dev->mr_indev);
    wake_lock_destroy(&mr_sensor_dev->wake_lock);
	mr_sensor_dev->mr_indev=NULL;
	kfree(mr_sensor_dev);
	mr_sensor_dev=NULL;
	kobject_put(mr_sensor_kobj);
	platform_driver_unregister(&mr_platform_driver);
	platform_device_unregister(pdev);
}


module_init(mr_sensor_init);
module_exit(mr_sensor_exit);


MODULE_DESCRIPTION("Intel MR sensor Driver");
MODULE_LICENSE("GPL v2");
