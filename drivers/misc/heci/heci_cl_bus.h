#ifndef _LINUX_HECI_CL_BUS_H
#define _LINUX_HECI_CL_BUS_H

#include <linux/device.h>
#include <linux/uuid.h>

struct heci_cl_device;

struct heci_cl_driver {
	struct device_driver driver;
	const char *name;

	const struct heci_cl_device_id *id_table;

	int (*probe)(struct heci_cl_device *dev,
		     const struct heci_cl_device_id *id);
	int (*remove)(struct heci_cl_device *dev);
};

int __heci_cl_driver_register(struct heci_cl_driver *driver,
				struct module *owner);
#define heci_cl_driver_register(driver)             \
	__heci_cl_driver_register(driver, THIS_MODULE)

void heci_cl_driver_unregister(struct heci_cl_driver *driver);

int heci_cl_send(struct heci_cl_device *device, u8 *buf, size_t length);
int heci_cl_recv(struct heci_cl_device *device, u8 *buf, size_t length);

typedef void (*heci_cl_event_cb_t)(struct heci_cl_device *device,
			       u32 events, void *context);
int heci_cl_register_event_cb(struct heci_cl_device *device,
			  heci_cl_event_cb_t read_cb, void *context);

#define HECI_CL_EVENT_RX 0
#define HECI_CL_EVENT_TX 1

void *heci_cl_get_drvdata(const struct heci_cl_device *device);
void heci_cl_set_drvdata(struct heci_cl_device *device, void *data);

int heci_cl_enable_device(struct heci_cl_device *device);
int heci_cl_disable_device(struct heci_cl_device *device);

#endif /* _LINUX_HECI_CL_BUS_H */
