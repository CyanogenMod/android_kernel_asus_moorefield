/*
 * Intel MID Platform XHCI/SSIC Controller PCI Bus Glue.
 *
 * Copyright (c) 2014, Intel Corporation.
 * Author: Tang, Jianqiang <jianqiang.tang@intel.com>
 * Some code borrowed from usbcore hub and xhci driver.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License 2 as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/usb/xhci-ssic-pci.h>

static const char ssic_group_name[] = "ssic";

static struct pci_dev		*ssic_pci_dev;
static struct ssic_xhci_hcd	ssic_hcd;

static struct class *ssic_class;
static struct device *ssic_class_dev;

static int create_ssic_class_device_files(struct pci_dev *pdev);
static void remove_ssic_class_device_files(void);
static int xhci_ssic_private_reset(struct usb_hcd *hcd);

static int is_ssic_host(struct usb_device *udev)
{
	struct pci_dev *pdev;

	pdev = to_pci_dev(udev->bus->controller);

	if (!pdev || !udev) {
		pr_err("%s %d pdev or udev is NULL, return\n",
				__func__, __LINE__);
		return 0;
	}

	dev_dbg(&udev->dev, "device ID: %d, portnum: %d",
			pdev->device, udev->portnum);

	/* CherryTrail and ANN SSIC Controller */
	if (pdev->vendor == PCI_VENDOR_ID_INTEL) {
		if (pdev->device != PCI_DEVICE_ID_INTEL_CHT_USH &&
			pdev->device != PCI_DEVICE_ID_INTEL_CHT_USH_A1 &&
				pdev->device != PCI_DEVICE_ID_INTEL_MOOR_SSIC) {
			dev_dbg(&udev->dev, "NOT SSIC Controller uevent, ignore\n");
			return 0;
		}
	}

	/* Ignore USB devices on external hub */
	if (udev->parent && udev->parent->parent)
		return 0;

	return 1;
}
static void ssic_wake_lock(void)
{
	mutex_lock(&ssic_hcd.wakelock_mutex);
	if (ssic_hcd.wakelock_state == UNLOCKED) {
		wake_lock(&ssic_hcd.ssic_wake_lock);
		ssic_hcd.wakelock_state = LOCKED;
	}
	mutex_unlock(&ssic_hcd.wakelock_mutex);
}

static void ssic_wake_unlock(void)
{
	mutex_lock(&ssic_hcd.wakelock_mutex);
	if (ssic_hcd.wakelock_state == LOCKED) {
		wake_unlock(&ssic_hcd.ssic_wake_lock);

		ssic_hcd.wakelock_state = UNLOCKED;
	}
	mutex_unlock(&ssic_hcd.wakelock_mutex);
}

int ssic_set_port_link_state(struct usb_hub *hub,
			int port1, unsigned int link_status)
{
	return set_port_feature(hub->hdev,
			port1 | (link_status << 3),
			USB_PORT_FEAT_LINK_STATE);
}

static int ssic_get_port_status(struct usb_device *hdev, int port1,
		struct usb_port_status *data)
{
	int i, status = -ETIMEDOUT;

	for (i = 0; i < SSIC_STS_RETRIES &&
			(status == -ETIMEDOUT || status == -EPIPE); i++) {
		status = usb_control_msg(hdev, usb_rcvctrlpipe(hdev, 0),
			USB_REQ_GET_STATUS, USB_DIR_IN | USB_RT_PORT, 0, port1,
			data, sizeof(*data), SSIC_STS_TIMEOUT);
	}
	return status;
}

static int ssic_hub_port_status(struct usb_hub *hub, int port1,
		u16 *status, u16 *change)
{
	int ret;

	mutex_lock(&hub->status_mutex);
	ret = ssic_get_port_status(hub->hdev, port1, &hub->status->port);
	if (ret < 4) {
		if (ret != -ENODEV)
			dev_err(hub->intfdev,
				"%s failed (err = %d)\n", __func__, ret);
		if (ret >= 0)
			ret = -EIO;
	} else {
		*status = le16_to_cpu(hub->status->port.wPortStatus);
		*change = le16_to_cpu(hub->status->port.wPortChange);

		ret = 0;
	}
	mutex_unlock(&hub->status_mutex);
	return ret;
}

static int hub_usb3_port_disable(struct usb_hub *hub, int port1)
{
	int ret;
	int total_time;
	u16 portchange, portstatus;

	if (!hub)
		return -EINVAL;

	if (!(hub->hdev->speed == USB_SPEED_SUPER))
		return -EINVAL;

	ret = ssic_set_port_link_state(hub, port1, USB_SS_PORT_LS_SS_DISABLED);
	if (ret)
		return ret;

	/* Wait for the link to enter the disabled state. */
	for (total_time = 0;; total_time += SSIC_HUB_DEBOUNCE_STEP) {
		ret = ssic_hub_port_status(hub, port1, &portstatus, &portchange);
		if (ret < 0)
			return ret;

		if ((portstatus & USB_PORT_STAT_LINK_STATE) ==
				USB_SS_PORT_LS_SS_DISABLED)
			break;
		if (total_time >= SSIC_HUB_DEBOUNCE_TIMEOUT)
			break;
		msleep(SSIC_HUB_DEBOUNCE_STEP);
	}

	if (total_time >= SSIC_HUB_DEBOUNCE_TIMEOUT) {
		dev_warn(hub->intfdev, "Could not disable port %d after %d ms\n",
				port1, total_time);
		return -ETIMEDOUT;
	}

	pr_err("SSIC: Place Link to SS.Disable successful\n");

	return 0;
}

static int ssic_port_enable(struct xhci_hcd *xhci, int enable)
{
	int			status;
	u32			temp;
	 __le32 __iomem		**port_array;

	port_array = xhci->usb3_ports;

	if (enable) {
		if (ssic_hcd.rh_dev) {
			dev_dbg(&ssic_pci_dev->dev,
					"%s----> enable port\n", __func__);

			/* In Cherryview, REGISTER_BANK_VALID is lost after D3/D0 transition.
			 * Make sure this bit is set before PROG_DONE bit is set
			 */
			temp = xhci_readl(xhci, &ssic_hcd.profile_regs->access_control);
			if (!(temp & REGISTER_BANK_VALID)) {
				temp |= REGISTER_BANK_VALID;
				xhci_writel(xhci, temp, &ssic_hcd.profile_regs->access_control);
			}

			/* Config SSIC Configuration Register2 */
			temp = xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg2);
			xhci_dbg(xhci, "Config Register2 = 0x%08X before write\n", temp);

			/* Write PROG_DONE[30] == 0 */
			xhci_dbg(xhci, "Clear PROG_DONE\n");
			temp &= ~PROG_DONE;
			xhci_writel(xhci, temp, &ssic_hcd.policy_regs->config_reg2);

			/* Config SSIC Configuration Register2 */
			temp = xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg2);
			xhci_dbg(xhci, "Config Register2 = 0x%08X after clear PROG_DONE\n", temp);

			if (temp & SSIC_PORT_UNUSED) {
				xhci_dbg(xhci, "temp & SSIC_PORT_UNUSED, config_register2 = 0x%08X\n", temp);
				temp &= ~SSIC_PORT_UNUSED;
				xhci_writel(xhci, temp, &ssic_hcd.policy_regs->config_reg2);
			}

			/* Config SSIC Configuration Register2 */
			temp = xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg2);
			xhci_dbg(xhci, "Config Register2 = 0x%08X after clear PORT_UNUSED\n", temp);

			temp |= PROG_DONE;
			xhci_writel(xhci, temp, &ssic_hcd.policy_regs->config_reg2);

			/* Config SSIC Configuration Register2 */
			temp = xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg2);
			xhci_dbg(xhci, "Config Register2 = 0x%08X after set PROG_DONE\n", temp);

			temp = xhci_readl(xhci, port_array[ssic_hcd.ssic_port - 1]);
			xhci_dbg(xhci, "ssic_enable = 1 , before set PP, portsc = 0x%X\n", temp);

			status = set_port_feature(ssic_hcd.rh_dev,
							ssic_hcd.ssic_port,
							USB_PORT_FEAT_POWER);
			if (status < 0) {
				dev_err(&ssic_pci_dev->dev,
						"%s set port power failed, return %d\n",
						__func__, status);
				return status;
			}

			if (status == 0)
				xhci_dbg(xhci, "set port_power successful\n");

			temp = xhci_readl(xhci, port_array[ssic_hcd.ssic_port - 1]);
			xhci_dbg(xhci, "ssic_enable = 1 , after set PP, portsc = 0x%X\n", temp);

			if (ssic_hcd.modem_dev) {
				dev_dbg(&ssic_pci_dev->dev,
						"Disable auto suspend in port enable\n");
				usb_disable_autosuspend(ssic_hcd.modem_dev);
			}

			usb_disable_autosuspend(ssic_hcd.rh_dev);
			ssic_hcd.autosuspend_enable = 0;
			ssic_hcd.u1_enable = 0;
			ssic_hcd.u2_enable = 0;
		}
	} else {
		if (ssic_hcd.rh_dev) {
			dev_dbg(&ssic_pci_dev->dev,
				"%s----> disable port\n", __func__);

			hub_usb3_port_disable(usb_hub_to_struct_hub(ssic_hcd.rh_dev), ssic_hcd.ssic_port);

			/* Config SSIC Configuration Register2 */
			temp = xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg2);
			xhci_dbg(xhci, "Config Register2 = 0x%08X before write\n", temp);

			/* Write PROG_DONE[30] == 0 */
			temp &= ~PROG_DONE;
			xhci_writel(xhci, temp, &ssic_hcd.policy_regs->config_reg2);
			xhci_dbg(xhci, "Config Register2 = 0x%08X after clear PROG_DONE\n", temp);

			/* Write PORT_UNUSED[31] == 1 */
			temp |= SSIC_PORT_UNUSED;
			xhci_writel(xhci, temp, &ssic_hcd.policy_regs->config_reg2);

			/* Config SSIC Configuration Register2 */
			temp = xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg2);
			xhci_dbg(xhci, "Config Register2 = 0x%08X after set PORT_UNUSED\n", temp);

			xhci_dbg(xhci, "Begin to do 2ms sleep\n");
			usleep_range(2000, 2500);
			xhci_dbg(xhci, "Finish the 2ms sleep\n");

			temp = xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg2);
			xhci_dbg(xhci, "Config Register2 = 0x%08X after 2ms sleep\n", temp);

			/* Write PROG_DONE[30] == 1 */
			temp |= PROG_DONE;
			xhci_writel(xhci, temp, &ssic_hcd.policy_regs->config_reg2);

			temp = xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg2);
			xhci_dbg(xhci, "Config Register2 = 0x%08X after write PROG_DONE\n", temp);

			xhci_dbg(xhci, "Before Clear PP, portsc = %X\n",
					xhci_readl(xhci, port_array[ssic_hcd.ssic_port - 1]));

			status = clear_port_feature(ssic_hcd.rh_dev,
							ssic_hcd.ssic_port,
						USB_PORT_FEAT_POWER);
			if (status < 0) {
				dev_err(&ssic_pci_dev->dev,
						"%s clear port power failed, return %d\n",
						__func__, status);
				return status;
			}
			if (status == 0)
				xhci_dbg(xhci, "clear port power() successful\n");
			xhci_dbg(xhci, "After clear PP, portsc = 0x%X\n",
			xhci_readl(xhci, port_array[ssic_hcd.ssic_port - 1]));

			status = set_port_feature(ssic_hcd.rh_dev,
							ssic_hcd.ssic_port,
						USB_PORT_FEAT_POWER);
			if (status < 0) {
				dev_err(&ssic_pci_dev->dev,
						"%s set port power failed, return %d\n",
						__func__, status);
				return status;
			}
			if (status == 0)
				xhci_dbg(xhci, "set port power() successful\n");
			xhci_dbg(xhci, "After set PP, portsc = 0x%X\n",
			xhci_readl(xhci, port_array[ssic_hcd.ssic_port - 1]));

			dev_dbg(&ssic_pci_dev->dev,
					"Enable root hub auto suspend in port disable\n");
			usb_enable_autosuspend(ssic_hcd.rh_dev);
			ssic_hcd.autosuspend_enable = 1;
			}
		}

	/* set the ssic_enable state */
	ssic_hcd.ssic_enable = enable;
	/* kick the change if modem attached */
	if (ssic_hcd.modem_dev) {
		xhci_dbg(xhci, "Modem is there\n");
		usb_set_change_bits(ssic_hcd.rh_dev,
						ssic_hcd.ssic_port);
		usb_kick_khubd(ssic_hcd.rh_dev);

		/*
		 * need to do delay 150-200ms to confirm
		 * disconnect flow finished, this info is
		 * from COE guys.
		 */
		msleep(200);
	}

	return 0;
}

static ssize_t ssic_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ssic_hcd.ssic_enable);
}

static ssize_t ssic_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int		retval;
	int		value;
	struct usb_hcd  *hcd = dev_get_drvdata(&ssic_pci_dev->dev);
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);

	if (sscanf(buf, "%d", &value) != 1) {
		dev_dbg(dev, "Invalid, value\n");
		return -EINVAL;
	}

	/* just return if ssic_enable == 0 and request enable == 0 */
	if ((!ssic_hcd.ssic_enable) && (value == 0)) {
		dev_dbg(dev, "SSIC state is already %d,ignore request\n",
							value);
		return -EINVAL;
	}

	usb_lock_device(ssic_hcd.rh_dev);
	mutex_lock(&ssic_hcd.ssic_mutex);
	if (!ssic_hcd.rh_dev) {
		dev_dbg(dev, "root hub is already removed\n");
		retval = -ENODEV;
		goto out;
	}

	/* need to resume if Controller already in D0i3 */
	if (ssic_hcd.modem_dev)
		pm_runtime_get_sync(&ssic_hcd.modem_dev->dev);

	if (ssic_hcd.rh_dev)
		pm_runtime_get_sync(&ssic_hcd.rh_dev->dev);

	if (value) {
		/* request IPC on while IPC already on */
		if (ssic_hcd.ssic_enable) {
			/* need to disable and re-enable again */
			dev_dbg(dev, "disable IPC before enable it\n");
			retval = ssic_port_enable(xhci, 0);
			if (retval) {
				dev_err(dev, "disable IPC failed, retval = %d\n",
						retval);
				goto pm_out;
			}

			/* re-enable IPC again */
			retval = ssic_port_enable(xhci, 1);
			if (retval) {
				dev_err(dev, "enable IPC failed, retval = %d\n",
						retval);
				goto pm_out;
			}
		} else {
		/* request IPC on while current IPC is off */
			dev_dbg(dev, "enable SSIC\n");

			retval = ssic_port_enable(xhci, 1);
			if (retval) {
				dev_err(dev, "enable IPC failed, retval = %d\n",
						retval);
				goto pm_out;
			}
		}
		/* hold wakelock */
		ssic_wake_lock();
	} else {
		if (ssic_hcd.ssic_enable) {
			/* disable IPC if current state is on
			 * and request IPC off
			 */
			dev_dbg(dev, "disable SSIC IPC\n");

			retval = ssic_port_enable(xhci, 0);
			if (retval) {
				dev_err(dev, "disable IPC failed, retval = %d\n",
						retval);
				goto pm_out;
			}
			/* hold wakelock */
			ssic_wake_unlock();
		}
	}
pm_out:
	/* need to resume if Controller already in D0i3 */
	if (ssic_hcd.modem_dev)
		pm_runtime_put(&ssic_hcd.modem_dev->dev);
	if (ssic_hcd.rh_dev)
		pm_runtime_put(&ssic_hcd.rh_dev->dev);
out:
	mutex_unlock(&ssic_hcd.ssic_mutex);
	usb_unlock_device(ssic_hcd.rh_dev);
	return size;
}

static DEVICE_ATTR(ssic_enable, S_IRUGO | S_IWUSR | S_IROTH | S_IWOTH,
		ssic_enable_show, ssic_enable_store);

static ssize_t ssic_show_registers(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	char			*next;
	unsigned		size;
	unsigned		t = 0;
	int			status;
	struct usb_hcd		*hcd;
	struct xhci_hcd		*xhci;
	__le32 __iomem		**port_array;

	next = buf;
	size = PAGE_SIZE;

	/* check if pm_runtime_get_sync successful or not */
	status = pm_runtime_get_sync(&ssic_pci_dev->dev);
	if (status < 0) {
		dev_err(&ssic_pci_dev->dev,
			"%s: pm_runtime_get_sync FAILED err = %d\n",
			__func__, status);
		pm_runtime_put_sync(&ssic_pci_dev->dev);
		return -EINVAL;
	}

	hcd = dev_get_drvdata(&ssic_pci_dev->dev);
	if (!hcd) {
		dev_err(&ssic_pci_dev->dev, "hcd is NULL\n");
		pm_runtime_put_sync(&ssic_pci_dev->dev);
		return -ENODEV;
	}

	if (hcd->regs) {
		xhci = hcd_to_xhci(hcd);
		if (!xhci) {
			dev_err(&ssic_pci_dev->dev, "xhci is NULL\n");
			pm_runtime_put_sync(&ssic_pci_dev->dev);
			return -ENODEV;
		}

		port_array = xhci->usb3_ports;

		t = scnprintf(next, size,
			"\n"
			"Controller Virtual Base Address = 0x%p\n"
			"USBCMD = 0x%x\n"
			"USBSTS = 0x%x\n"
			"PORTSC1 = 0x%x, address = 0x%p\n"
			"PORTPMSC1 = 0x%x\n"
			"PORTLI1 = 0x%x\n"
			"Capability ID = 0x%x\n"
			"Global Control = 0x%x\n"
			"Config register1 = 0x%x\n"
			"Config register2 = 0x%x\n"
			"Config register3 = 0x%x\n"
			"Config register4 = 0x%x\n"
			"Capability Register = 0x%x\n"
			"Access Control = 0x%x\n"
			"Access Control Status = 0x%x\n"
			"PORT1: Config Register2 = 0x%x\n",
			hcd->regs,
			xhci_readl(xhci, &xhci->op_regs->command),
			xhci_readl(xhci, &xhci->op_regs->status),
			xhci_readl(xhci, port_array[ssic_hcd.ssic_port - 1]),
			port_array[ssic_hcd.ssic_port - 1],
			xhci_readl(xhci, port_array[ssic_hcd.ssic_port - 1] + 1),
			xhci_readl(xhci, &xhci->op_regs->port_link_base),
			xhci_readl(xhci, &ssic_hcd.policy_regs->cap_id),
			xhci_readl(xhci, &ssic_hcd.policy_regs->global_control),
			xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg1),
			xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg2),
			xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg3),
			xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg4),
			xhci_readl(xhci, &ssic_hcd.profile_regs->cap_reg),
			xhci_readl(xhci, &ssic_hcd.profile_regs->access_control),
			xhci_readl(xhci, &ssic_hcd.profile_regs->access_status),
			xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg2 + 12));
	}

	pm_runtime_put_sync(&ssic_pci_dev->dev);

	size -= t;
	next += t;

	return PAGE_SIZE - size;
}
static DEVICE_ATTR(ssic_registers, S_IRUGO, ssic_show_registers, NULL);

/* port inactivityDuration sysfs interface */
static ssize_t ssic_port_inactivity_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ssic_hcd.port_inactivity_duration);
}

static ssize_t ssic_port_inactivity_duration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned	duration;

	if (sscanf(buf, "%d", &duration) != 1) {
		dev_dbg(dev, "Invalid, value\n");
		return -EINVAL;
	}

	if (duration < 0) {
		dev_dbg(dev, "Invalid, value\n");
		return -EINVAL;
	}

	mutex_lock(&ssic_hcd.ssic_mutex);
	if (ssic_hcd.modem_dev != NULL) {
		pm_runtime_set_autosuspend_delay
		(&ssic_hcd.modem_dev->dev, duration);
		ssic_hcd.port_inactivity_duration = duration;
		dev_dbg(dev, "port Duration after change: %d\n",
				ssic_hcd.port_inactivity_duration);
	} else {
		dev_dbg(dev, "No SSIC Modem, just ignore this request\n");
		mutex_unlock(&ssic_hcd.ssic_mutex);
		return -ENODEV;
	}

	mutex_unlock(&ssic_hcd.ssic_mutex);
	return size;
}

static DEVICE_ATTR(port_inactivity_duration,
		S_IRUGO | S_IWUSR | S_IROTH | S_IWOTH,
		ssic_port_inactivity_duration_show,
		ssic_port_inactivity_duration_store);

static ssize_t ssic_u1_inactivity_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ssic_hcd.u1_inactivity_duration);
}

static ssize_t ssic_u1_inactivity_duration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned	duration;

	if (sscanf(buf, "%d", &duration) != 1) {
		dev_dbg(dev, "Invalid, value\n");
		return -EINVAL;
	}

	if (duration <= 0 || duration > SSIC_MAX_U1_TIMEOUT) {
		dev_dbg(dev, "Invalid, value\n");
		return -EINVAL;
	}

	mutex_lock(&ssic_hcd.ssic_mutex);
	if (ssic_hcd.modem_dev != NULL) {
		ssic_hcd.u1_inactivity_duration = duration;
		dev_dbg(dev, "u1 Duration after change: %d\n",
				ssic_hcd.u1_inactivity_duration);
	} else {
		dev_dbg(dev, "No SSIC Modem, just ignore this request\n");
		mutex_unlock(&ssic_hcd.ssic_mutex);
		return -ENODEV;
	}

	mutex_unlock(&ssic_hcd.ssic_mutex);
	return size;
}

static DEVICE_ATTR(u1_inactivity_duration,
		S_IRUGO | S_IWUSR | S_IROTH | S_IWOTH,
		ssic_u1_inactivity_duration_show,
		ssic_u1_inactivity_duration_store);

static ssize_t ssic_u2_inactivity_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ssic_hcd.u2_inactivity_duration);
}

static ssize_t ssic_u2_inactivity_duration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned	duration;

	if (sscanf(buf, "%d", &duration) != 1) {
		dev_dbg(dev, "Invalid, value\n");
		return -EINVAL;
	}

	if (duration <= 0 || duration > SSIC_MAX_U2_TIMEOUT) {
		dev_dbg(dev, "Invalid, value\n");
		return -EINVAL;
	}

	mutex_lock(&ssic_hcd.ssic_mutex);
	if (ssic_hcd.modem_dev != NULL) {
		ssic_hcd.u2_inactivity_duration = duration;
		dev_dbg(dev, "u2 Duration after change: %d\n",
				ssic_hcd.u1_inactivity_duration);
	} else {
		dev_dbg(dev, "No SSIC Modem, just ignore this request\n");
		mutex_unlock(&ssic_hcd.ssic_mutex);
		return -ENODEV;
	}

	mutex_unlock(&ssic_hcd.ssic_mutex);
	return size;
}

static DEVICE_ATTR(u2_inactivity_duration,
		S_IRUGO | S_IWUSR | S_IROTH | S_IWOTH,
		ssic_u2_inactivity_duration_show,
		ssic_u2_inactivity_duration_store);

static int ssic_set_device_initiated_lpm(struct usb_device *udev,
		enum usb3_link_state state, bool enable)
{
	int ret;
	int feature;

	switch (state) {
	case USB3_LPM_U1:
		feature = USB_DEVICE_U1_ENABLE;
		break;
	case USB3_LPM_U2:
		feature = USB_DEVICE_U2_ENABLE;
		break;
	default:
		dev_warn(&udev->dev, "%s: Can't %s non-U1 or U2 state.\n",
				__func__, enable ? "enable" : "disable");
		return -EINVAL;
	}

	if (udev->state != USB_STATE_CONFIGURED) {
		dev_dbg(&udev->dev, "%s: Can't %s %s state "
				"for unconfigured device.\n",
				__func__, enable ? "enable" : "disable",
				state == USB_DEVICE_U1_ENABLE ? "U1" : "U2");
		return 0;
	}

	if (enable) {
		/*
		 * Now send the control transfer to enable device-initiated LPM
		 * for either U1 or U2.
		 */
		ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
				USB_REQ_SET_FEATURE,
				USB_RECIP_DEVICE,
				feature,
				0, NULL, 0,
				USB_CTRL_SET_TIMEOUT);
	} else {
		ret = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
				USB_REQ_CLEAR_FEATURE,
				USB_RECIP_DEVICE,
				feature,
				0, NULL, 0,
				USB_CTRL_SET_TIMEOUT);
	}
	if (ret < 0) {
		dev_warn(&udev->dev, "%s of device-initiated %s failed.\n",
				enable ? "Enable" : "Disable",
				state == USB_DEVICE_U1_ENABLE ? "U1" : "U2");
		return -EBUSY;
	}
	return 0;
}

/*
 * A U1 timeout of 0x0 means the parent hub will reject any transitions to U1.
 * 0xff means the parent hub will accept transitions to U1, but will not
 * initiate a transition.
 *
 * A U1 timeout of 0x1 to 0x7F also causes the hub to initiate a transition to
 * U1 after that many microseconds.  Timeouts of 0x80 to 0xFE are reserved
 * values.
 *
 * A U2 timeout of 0x0 means the parent hub will reject any transitions to U2.
 * 0xff means the parent hub will accept transitions to U2, but will not
 * initiate a transition.
 *
 * A U2 timeout of 0x1 to 0xFE also causes the hub to initiate a transition to
 * U2 after N*256 microseconds.  Therefore a U2 timeout value of 0x1 means a U2
 * idle timer of 256 microseconds, 0x2 means 512 microseconds, 0xFE means
 * 65.024ms.
 */
static int ssic_set_host_initiated_lpm(struct device *dev,
		enum usb3_link_state state, bool enable)
{
	struct usb_hcd		*hcd;
	struct xhci_hcd		*xhci;
	__le32 __iomem		**port_array;
	u32 temp, timeout, port;
	int ret;

	hcd = dev_get_drvdata(&ssic_pci_dev->dev);

	if (!hcd)
		return -ENODEV;

	xhci = hcd_to_xhci(hcd);
	port_array = xhci->usb3_ports;
	port = ssic_hcd.ssic_port - 1;

	if (state != USB3_LPM_U1 && state != USB3_LPM_U2)
		return -EINVAL;

	switch (state) {
	case USB3_LPM_U1:
		if (enable)
			timeout = ssic_hcd.u1_inactivity_duration;
		else
			timeout = 0;
		temp = xhci_readl(xhci, port_array[port] + PORTPMSC);
		temp &= ~PORT_U1_TIMEOUT_MASK;
		temp |= PORT_U1_TIMEOUT(timeout);
		xhci_writel(xhci, temp, port_array[port] + PORTPMSC);
		break;

	case USB3_LPM_U2:
		if (enable)
			timeout = ssic_hcd.u1_inactivity_duration * 4;
		else
			timeout = 0;
		temp = xhci_readl(xhci, port_array[port] + PORTPMSC);
		temp &= ~PORT_U2_TIMEOUT_MASK;
		temp |= PORT_U2_TIMEOUT(timeout);
		xhci_writel(xhci, temp, port_array[port] + PORTPMSC);
		break;
	}

	return 0;
}

/* Interfaces for u1_enable */
static ssize_t ssic_u1_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ssic_hcd.u1_enable);
}

static ssize_t ssic_u1_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int		request;

	if (sscanf(buf, "%d", &request) != 1) {
		dev_dbg(dev, "Invalid, value\n");
		return -EINVAL;
	}

	if (!ssic_hcd.modem_dev || !ssic_hcd.rh_dev) {
		dev_err(dev, "Modem/roothub is not connected, ignore any request\n");
		return -ENODEV;
	}

	pm_runtime_get_sync(&ssic_hcd.modem_dev->dev);
	mutex_lock(&ssic_hcd.ssic_mutex);

	if (request) {
		dev_dbg(dev, "Enable U1 for SSIC\n");
		ssic_set_host_initiated_lpm(dev, USB3_LPM_U1, true);
		ssic_set_device_initiated_lpm(ssic_hcd.modem_dev, USB3_LPM_U1, true);
	} else {
		dev_dbg(dev, "Disable U1 for SSIC\n");
		ssic_set_host_initiated_lpm(dev, USB3_LPM_U1, false);
		ssic_set_device_initiated_lpm(ssic_hcd.modem_dev, USB3_LPM_U1, false);
	}

	ssic_hcd.u1_enable = request;
	mutex_unlock(&ssic_hcd.ssic_mutex);
	pm_runtime_put(&ssic_hcd.modem_dev->dev);

	return size;
}
static DEVICE_ATTR(u1_enable, S_IRUGO | S_IWUSR | S_IROTH | S_IWOTH,
		ssic_u1_enable_show,
		ssic_u1_enable_store);

static ssize_t ssic_u2_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int		request;

	if (sscanf(buf, "%d", &request) != 1) {
		dev_dbg(dev, "Invalid, value\n");
		return -EINVAL;
	}

	if (!ssic_hcd.modem_dev || !ssic_hcd.rh_dev) {
		dev_err(dev, "Modem/roothub is not connected, ignore any request\n");
		return -ENODEV;
	}

	pm_runtime_get_sync(&ssic_hcd.modem_dev->dev);
	mutex_lock(&ssic_hcd.ssic_mutex);

	if (request) {
		dev_dbg(dev, "SSIC U2 enabled\n");
		ssic_set_host_initiated_lpm(dev, USB3_LPM_U2, true);
		ssic_set_device_initiated_lpm(ssic_hcd.modem_dev, USB3_LPM_U2, true);
	} else {
		dev_dbg(dev, "SSIC U2 disabled\n");
		ssic_set_host_initiated_lpm(dev, USB3_LPM_U2, false);
		ssic_set_device_initiated_lpm(ssic_hcd.modem_dev, USB3_LPM_U2, false);
	}

	ssic_hcd.u2_enable = request;
	mutex_unlock(&ssic_hcd.ssic_mutex);
	pm_runtime_put(&ssic_hcd.modem_dev->dev);
	return size;
}

/* Interfaces for u2_enable */
static ssize_t ssic_u2_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ssic_hcd.u2_enable);
}

static DEVICE_ATTR(u2_enable, S_IRUGO | S_IWUSR | S_IROTH | S_IWOTH,
		ssic_u2_enable_show,
		ssic_u2_enable_store);

/* Interfaces for L2 suspend */
static ssize_t ssic_autosuspend_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ssic_hcd.autosuspend_enable);
}

static ssize_t ssic_autosuspend_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int		request;

	if (sscanf(buf, "%d", &request) != 1) {
		dev_dbg(dev, "Invalid, value\n");
		return -EINVAL;
	}

	mutex_lock(&ssic_hcd.ssic_mutex);

	if (ssic_hcd.modem_dev != NULL) {
		if (request == 0) {
			dev_dbg(dev, "SSIC: modem autosuspend disable\n");
			usb_disable_autosuspend(ssic_hcd.modem_dev);
		} else {
			dev_dbg(dev, "SSIC: modem dev autosuspend enable\n");
			usb_enable_autosuspend(ssic_hcd.modem_dev);
		}
	}

	if (ssic_hcd.rh_dev != NULL) {
		if (request == 0) {
			dev_dbg(dev, "SSIC: bus autosuspend disable\n");
			usb_disable_autosuspend(ssic_hcd.rh_dev);
		} else {
			dev_dbg(dev, "SSIC: bus autosuspend enable\n");
			usb_enable_autosuspend(ssic_hcd.rh_dev);
		}
	}

	ssic_hcd.autosuspend_enable = request;

	mutex_unlock(&ssic_hcd.ssic_mutex);
	return size;
}

static DEVICE_ATTR(autosuspend_enable, S_IRUGO | S_IWUSR | S_IROTH | S_IWOTH,
		ssic_autosuspend_enable_show,
		ssic_autosuspend_enable_store);

static ssize_t ssic_bus_inactivity_duration_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ssic_hcd.bus_inactivity_duration);
}

static ssize_t ssic_bus_inactivity_duration_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned	duration;

	if (sscanf(buf, "%d", &duration) != 1) {
		dev_dbg(dev, "Invalid, value\n");
		return -EINVAL;
	}

	mutex_lock(&ssic_hcd.ssic_mutex);
	if (ssic_hcd.rh_dev != NULL) {
		pm_runtime_set_autosuspend_delay
			(&ssic_hcd.rh_dev->dev,
				duration);
		ssic_hcd.bus_inactivity_duration = duration;
		dev_dbg(dev, "bus Duration: %d\n",
				ssic_hcd.bus_inactivity_duration);
	} else {
		dev_dbg(dev, "No SSIC root hub, just ignore this request\n");
		mutex_unlock(&ssic_hcd.ssic_mutex);
		return -ENODEV;
	}

	mutex_unlock(&ssic_hcd.ssic_mutex);
	return size;
}

static DEVICE_ATTR(bus_inactivity_duration,
		S_IRUGO | S_IWUSR | S_IROTH | S_IWOTH,
		ssic_bus_inactivity_duration_show,
		ssic_bus_inactivity_duration_store);

/* group all the SSIC related attributes */
static struct attribute *ssic_attrs[] = {
	&dev_attr_ssic_registers.attr,
	&dev_attr_autosuspend_enable.attr,
	&dev_attr_port_inactivity_duration.attr,
	&dev_attr_bus_inactivity_duration.attr,
	&dev_attr_u1_inactivity_duration.attr,
	&dev_attr_u2_inactivity_duration.attr,
	&dev_attr_ssic_enable.attr,
	&dev_attr_u1_enable.attr,
	&dev_attr_u2_enable.attr,
	NULL,
};

/* SSIC PM interface:
 * Bit0 - U3 enable/disable
 * Bit1 - U2 enable/disable
 * Bit2 - U1 enale/disable
 */
static ssize_t ssic_pm_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ssic_hcd.autosuspend_enable | ssic_hcd.u2_enable << 1
			| ssic_hcd.u1_enable << 2);
}

static ssize_t ssic_pm_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int rc1, rc2, rc3;
	u8 pm_enable;

	if (size > HSIC_ENABLE_SIZE)
		return -EINVAL;

	if (sscanf(buf, "%d", &pm_enable) != 1) {
		dev_dbg(dev, "Invalid, value\n");
		return -EINVAL;
	}

	if (pm_enable & 1)
		 rc1 = ssic_autosuspend_enable_store(dev, attr, "1", size);
	else
		 rc1 = ssic_autosuspend_enable_store(dev, attr, "0", size);


	if (pm_enable & 2)
		rc2 = ssic_u2_enable_store(dev, attr, "1", size);
	else
		rc2 = ssic_u2_enable_store(dev, attr, "0", size);

	if (pm_enable & 4)
		rc3 = ssic_u1_enable_store(dev, attr, "1", size);
	else
		rc3 = ssic_u1_enable_store(dev, attr, "0", size);

	if (rc1 == size && rc2 == size && rc3 == size)
		return size;
	else
		return -EINVAL;
}

static DEVICE_ATTR(pm_enable, S_IRUGO | S_IWUSR | S_IROTH,
		ssic_pm_enable_show,
		 ssic_pm_enable_store);


static struct attribute_group ssic_attr_group = {
	.name   = ssic_group_name,
	.attrs  = ssic_attrs,
};

static void ssicdev_add(struct usb_device *udev)
{
	if (!is_ssic_host(udev)) {
		pr_debug("%s Invalid device add event, ignore\n", __func__);
		return;
	}

	if (udev->speed == USB_SPEED_HIGH) {
		dev_dbg(&udev->dev, "USB2 root hub or device, just ignore\n");
		return;
	}

	dev_dbg(&udev->dev, "notify SSIC add device\n");

	/* Root hub */
	if (!udev->parent) {
		if (udev->speed == USB_SPEED_SUPER) {
			dev_dbg(&udev->dev, "%s rh device set\n", __func__);
			ssic_hcd.rh_dev = udev;
			dev_dbg(&udev->dev,
				"%s enable roothub autosuspend\n", __func__);
			pm_runtime_set_autosuspend_delay(&udev->dev,
				ssic_hcd.bus_inactivity_duration);
			usb_enable_autosuspend(udev);
			/* enable autosuspend before modem coming */
			ssic_hcd.autosuspend_enable = 1;
		}
	} else {
		if (udev->portnum != ssic_hcd.ssic_port) {
			dev_dbg(&udev->dev, "%s ignore XHCI ports except SSIC port\n",
					__func__);
			dev_dbg(&udev->dev, "%s ush ports %d\n", __func__,
					udev->portnum);
			return;
		}

		/* ssic modem coming now */
		if (udev->speed == USB_SPEED_SUPER) {
			/* Modem devices */
			/* hold wakelock */
			ssic_wake_lock();
			ssic_hcd.modem_dev = udev;
			pm_runtime_set_autosuspend_delay
				(&udev->dev, ssic_hcd.port_inactivity_duration);
			/* disable modem persist_enable feature */
			udev->persist_enabled = 0;

			/* ssic should always use In-band remote wakeup */
			if (ssic_hcd.remote_wakeup_enable) {
				dev_dbg(&udev->dev, "%s Modem dev remote wakeup enabled\n",
						__func__);
				device_set_wakeup_capable(&ssic_hcd.modem_dev->dev, 1);
				device_set_wakeup_capable(&ssic_hcd.rh_dev->dev, 1);
			} else {
				dev_dbg(&udev->dev, "%s Modem dev remote wakeup disabled\n",
						__func__);
				device_set_wakeup_capable
					(&ssic_hcd.modem_dev->dev, 0);
				device_set_wakeup_capable
					(&ssic_hcd.rh_dev->dev, 0);
			}

			ssic_hcd.autosuspend_enable = SSIC_AUTOSUSPEND;

			if (ssic_hcd.autosuspend_enable) {
				dev_dbg(&udev->dev, "%s enable Modem autosuspend\n",
						__func__);
				usb_enable_autosuspend(ssic_hcd.modem_dev);
				usb_enable_autosuspend(ssic_hcd.rh_dev);
			}

			if (ssic_hcd.autosuspend_enable == 0) {
				dev_dbg(&udev->dev, "%s disable Modem dev autosuspend\n",
					__func__);
				usb_disable_autosuspend(ssic_hcd.modem_dev);
				usb_disable_autosuspend(ssic_hcd.rh_dev);
			}
		}
	}
}

static void ssicdev_remove(struct usb_device *udev)
{
	if (!is_ssic_host(udev)) {
		pr_debug("%s Invalid device add event, ignore\n", __func__);
		return;
	}

	if (udev->speed == USB_SPEED_HIGH) {
		dev_dbg(&udev->dev, "USB2 root hub or device, just ignore\n");
		return;
	}

	dev_dbg(&udev->dev, "Notify SSIC remove device\n");

	/* Root hub */
	if (!udev->parent) {
		if (udev->speed == USB_SPEED_SUPER) {
			dev_dbg(&udev->dev,
				"%s root hub dev deleted\n", __func__);
			mutex_lock(&ssic_hcd.ssic_mutex);
			ssic_hcd.rh_dev = NULL;
			mutex_unlock(&ssic_hcd.ssic_mutex);
		} else {
			dev_dbg(&udev->dev, " USB2 root hub remove, ignore\n");
			return;
		}
	} else {
		if (udev->portnum != ssic_hcd.ssic_port) {
			dev_dbg(&udev->dev, "%s ignore XHCI ports except SSIC\n",
					__func__);
			dev_dbg(&udev->dev, "%s SSIC ports %d\n", __func__,
					udev->portnum);
			return;
		}

		if (udev->speed == USB_SPEED_SUPER) {
			/* Modem devices */
			dev_dbg(&udev->dev, "%s modem dev deleted\n", __func__);
			mutex_lock(&ssic_hcd.ssic_mutex);
			ssic_hcd.modem_dev = NULL;

			hub_usb3_port_disable(usb_hub_to_struct_hub(ssic_hcd.rh_dev), ssic_hcd.ssic_port);

			/* enable autosuspend when modem remove
			 * sync internal autosuspend_enable value
			 */
			if (ssic_hcd.rh_dev)
				usb_enable_autosuspend(ssic_hcd.rh_dev);
			ssic_hcd.autosuspend_enable = 1;

			mutex_unlock(&ssic_hcd.ssic_mutex);

		}
	}
}

/* the root hub will call this callback when device added/removed */
static int ssic_notify(struct notifier_block *self,
		unsigned long action, void *dev)
{
	switch (action) {
	case USB_DEVICE_ADD:
		ssicdev_add(dev);
		break;
	case USB_DEVICE_REMOVE:
		ssicdev_remove(dev);
		break;
	default:
		pr_debug("%s Invalid event = %lu, ignore\n",
				__func__, action);
		return NOTIFY_DONE;
	}
	return NOTIFY_OK;
}

static void ssic_port_suspend(struct usb_device *udev)
{
	if (!is_ssic_host(udev)) {
		pr_debug("port suspend event not belongs to SSIC\n");
		return;
	}

	if (udev->portnum != ssic_hcd.ssic_port) {
		dev_dbg(&udev->dev, "%s ignore ush ports %d\n",
			__func__, udev->portnum);
		return;
	}

	/* Modem dev */
	if ((udev->parent)) {
		dev_dbg(&udev->dev, "%s s3 wlock unlocked\n", __func__);
		ssic_wake_unlock();
	}
}

static void ssic_port_resume(struct usb_device *udev)
{
	if (!is_ssic_host(udev)) {
		pr_debug("port_resume event not belongs to SSIC\n");
		return;
	}

	if (udev->portnum != ssic_hcd.ssic_port) {
		dev_dbg(&udev->dev, "%s ignore ush ports %d\n",
			__func__, udev->portnum);
		return;
	}

	/* Modem dev */
	if ((udev->parent) && (ssic_hcd.power_state != POWER_SUSPENDING)) {
		dev_dbg(&udev->dev, "%s s3 wlock locked\n", __func__);
		ssic_wake_lock();
	}
}

static int ssic_pm_notify(struct notifier_block *self,
		unsigned long action, void *dev)
{
	switch (action) {
	case USB_PORT_SUSPEND:
		ssic_port_suspend(dev);
		break;
	case USB_PORT_RESUME:
		ssic_port_resume(dev);
		break;
	default:
		pr_debug("%s Invalid event = %lu, ignore\n",
				__func__, action);
		return NOTIFY_DONE;
	}
	return NOTIFY_OK;
}

static int ssic_power_notify(struct notifier_block *self,
		unsigned long action, void *dummy)
{
	switch (action) {
	case PM_SUSPEND_PREPARE:
		ssic_hcd.power_state = POWER_SUSPENDING;
		break;
	default:
		pr_debug("%s Invalid event = %lu, ignore\n",
			__func__, action);
		return NOTIFY_DONE;
	}
	return NOTIFY_OK;
}

static int xhci_ssic_disable_feature(struct xhci_hcd *xhci)
{
	u32			temp;

	/* disable SSIC port1 for ANN and CHT as port1 never used */
	temp = xhci_readl(xhci, (&ssic_hcd.policy_regs->config_reg2 + 11));
	xhci_dbg(xhci, "CONFIG register2 for Port1 before write = 0x%X\n", temp);
	temp |= SSIC_PORT_UNUSED;
	xhci_writel(xhci, temp, (&ssic_hcd.policy_regs->config_reg2 + 11));

	xhci_dbg(xhci, "CONFIG register2 for Port1 after write = 0x%X\n",
			xhci_readl(xhci,
				(&ssic_hcd.policy_regs->config_reg2 + 11)));

	/* disable STALL in U0 in config register 3 for port 0 */
	temp = xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg3);
	xhci_dbg(xhci, "CONFIG register3 before write = 0x%X\n", temp);
	temp |= DISABLE_U0_STALL;
	temp &= ~LUP_LDN_TIMER_MAX;
	temp &= ~(1<<20);
	xhci_writel(xhci, temp, &ssic_hcd.policy_regs->config_reg3);
	xhci_dbg(xhci, "CONFIG register3 after write = 0x%X\n",
			xhci_readl(xhci,
				&ssic_hcd.policy_regs->config_reg3));

	/* disable scrambling in config register 2 for port 0 */
	temp = xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg2);
	xhci_dbg(xhci, "CONFIG register2 before write = 0x%X\n", temp);
	temp |= DISABLE_SCRAMBLING;
	xhci_writel(xhci, temp, &ssic_hcd.policy_regs->config_reg2);
	xhci_dbg(xhci, "CONFIG register2 after write = 0x%X\n",
			xhci_readl(xhci,
				&ssic_hcd.policy_regs->config_reg2));

	return 0;
}

struct mphy_attr_setting ssic_register_bank[] = {
	{LOCAL_PHY, ATTRID_TX_HSRATE_SERIES, ATTR_VAL_HSMODE_RATE_SERIES_A}, /* 0x22, 0x01 */
	{LOCAL_PHY, ATTRID_RX_HSRATE_SERIES, ATTR_VAL_HSMODE_RATE_SERIES_A}, /* 0xA2, 0x01 */
	{LOCAL_PHY, ATTRID_TX_HS_SYNC_LENGTH, ATTR_VAL_SYNC_LENGTH(5) | ATTR_VAL_SYNC_RANGE(1)}, /* 0x28, 0x45*/
	{LOCAL_PHY, ATTRID_TX_PWM_BURST_CLOSURE_EXTENDSION, ATTR_VAL_BURST_CLOSURE_SEQ_DURATION(0x14)}, /* 0x2D, 0x14 */
	/* All modem side registers definition are for reference only.
	 * Because there have no formal definition document released by modem team so far.
	 * Below definition are refer to MIPI spec. But maybe intel modify them for other usage.
	 * for example: 0xD7 ATTR in MIPI spec, only bit 0 is valid. But in here, we set it as 0x39.
	 */
	{REMOTE_PHY, ATTRID_MODEM_CB_REG_A38, 0x18}, /* 0xA38, 0x18 */
	{REMOTE_PHY, ATTRID_MODEM_CB_REG_A39, 0x30}, /* 0xA39, 0x30 */
	{REMOTE_PHY, ATTRID_MODEM_CB_UPDATE_40A, 0x01}, /* 0x40A, 0x01 */
	{REMOTE_PHY, ATTRID_MODEM_CUSTOM_REG_CD, 0x88}, /* 0xCD, 0x88 */
	{REMOTE_PHY, ATTRID_MC_RX_LA_CAPABILITY, 0x39}, /* 0xD7, 0x39 */
	{REMOTE_PHY, ATTRID_MC_HS_START_TIME_VAR_CAPABILITY, 0x19}, /* 0xD4, 0x19 */
	{REMOTE_PHY, ATTRID_MODEM_CUSTOM_REG_C5, 0x07}, /* 0xC5, 0x07 */
	{REMOTE_PHY, ATTRID_RX_TERMINATION_FORCE_ENABLE, ATTR_VAL_ENABLE_RDIF_RX}, /* 0xA9, 0x01 */
	{REMOTE_PHY, ATTRID_TX_HS_SYNC_LENGTH, ATTR_VAL_SYNC_LENGTH(6) | ATTR_VAL_SYNC_RANGE(1)}, /* 0x28, 0x46 */
	{REMOTE_PHY, ATTRID_TX_HS_PREPARE_LENGTH, ATTR_VAL_M_TX_LENGTH_MULTIPLIER(6)}, /* 0x29, 0x06 */
	{REMOTE_PHY, ATTRID_DISABLE_SCRAMBLING, ATTR_VAL_DISABLE_HSMODE_SCRAMBLING}, /* 0x403, 0x01 */
	{REMOTE_PHY, ATTRID_DISABLE_STALL_IN_U0, ATTR_VAL_DISABLE_USP_STALL_IN_U0}, /* 0x404, 0x01 */
	{REMOTE_PHY, ATTRID_MODEM_CUSTOM_REG_F3, 0x21}, /* 0xF3, 0x21 */
};

static int ssic_config_register_bank(struct xhci_hcd *xhci, struct mphy_attr_setting *settings, int size)
{
	u32 temp = 0, i;
	struct mphy_attr_setting index;

	if (!xhci || !settings)
		return -EINVAL;

	temp = xhci_readl(xhci, &ssic_hcd.profile_regs->access_control);
	xhci_dbg(xhci, "SSIC: access_control = 0x%X, address = %p\n",
				temp, (&ssic_hcd.profile_regs->access_control));
	/* set Register Bank is valid to indicate to Controller that register bank
	 * will be configured with commands that should be automatically issued.
	 */
	temp |= REGISTER_BANK_VALID;
	/* set HS_CONFIG to 1 to indicate Controller should automatically issue
	 * CONFIGURE_FOR_HS RRAP command when finished issuing the RRAP commands
	 * in register bank.
	 */
	temp |= HS_CONFIG;

	/* Upon detecting Command Phase Done in the control register (and executing
	 * Config for HS if HS Config flag is set), the host controller starts
	 * executing commands from the register bank if Register Bank Valid is
	 * set to ‘1’.*/
	temp |= COMMAND_PHASE_DONE;

	xhci_writel(xhci, temp, &ssic_hcd.profile_regs->access_control);
	xhci_dbg(xhci, "access_control after write = 0x%X\n",
			xhci_readl(xhci, &ssic_hcd.profile_regs->access_control));

	for (i = 0; i < size; i++) {
		index = settings[i];
		temp = 0;

		/* Attribute ID[27:16] */
		temp |= ATTRIBUTE_ID(index.attr_id);

		/* Target PHY[14] */
		if (index.target == LOCAL_PHY)
			temp |= TARGET_PHY;

		/* Attribute Value[7:0] */
		temp |= index.val;

		/* Write Valid[15] to 1*/
		temp |= ATTRIBUTE_VALID;


		xhci_dbg(xhci, "%s: set %s attr 0x%x with value 0x%x\n", __func__,
				temp & (1 << 14) ? "LOCAL PHY" : "REMOTE PHY", (temp >> 16) & 0xFFF,
				temp & 0xFF);
		xhci_writel(xhci, temp, &ssic_hcd.profile_regs->attribute_base + i);
	}

	return 0;
}

static int ann_config_register(struct usb_hcd *hcd)
{
	struct xhci_hcd *xhci;
	u32		temp;

	if (!hcd) {
		pr_err("%s hcd is NULL, return -EINVAL\n", __func__);
		return -EINVAL;
	}

	xhci = hcd_to_xhci(hcd);
	if (!xhci) {
		pr_err("%s xhci is NULL\n", __func__);
		return -ENODEV;
	}

	if (hcd->regs) {
		/* XHCC1 Write Value : 0x3401FD */
		temp = xhci_readl(xhci, hcd->regs + 0x8640);
		xhci_dbg(xhci, "XHCC1(0x8640) read value is = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x8640));
		temp |= (0x1 << 18);
		temp |= (0x3 << 20);
		xhci_writel(xhci, temp, hcd->regs + 0x8640);
		xhci_dbg(xhci, "XHCC1(0x8640) write value is = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x8640));

		/* XHCC2 Write Value : 0x3CFC68F */
		temp = xhci_readl(xhci, hcd->regs + 0x8644);
		xhci_dbg(xhci, "XHCC2(0x8644) = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x8644));
		temp |= (0x1 << 25);
		temp |= (0x7 << 22);
		temp |= (0x3F << 14);
		temp &= ~(0x1 << 11);
		temp |= (0x1 << 10);
		temp |= (0x2 << 8);
		temp |= (0x2 << 6);
		temp |= (0x1 << 3);
		temp |= (0x7 << 0);
		xhci_writel(xhci, temp, hcd->regs + 0x8644);
		xhci_dbg(xhci, "XHCC2 after Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x8644));

		/* XHCLKGTEN  Write Value : 0xFBF6D3F */
		temp = xhci_readl(xhci, hcd->regs + 0x8650);
		temp |= (0x1 << 27);
		temp |= (0x1 << 26);
		temp |= (0x1 << 25);
		temp |= (0x1 << 24);
		temp |= (0xb << 20);
		temp |= (0xF << 16);
		temp &= ~(0x1 << 15);
		temp |= (0x1 << 14);
		temp |= (0x1 << 13);
		temp |= (0x3 << 10);
		temp |= (0x1 << 8);
		temp |= (0x1 << 5);
		temp |= (0x1 << 4);
		temp |= (0x1 << 3);
		temp |= (0x1 << 2);
		temp |= (0x1 << 1);
		temp |= (0x1 << 0);
		xhci_writel(xhci, temp, hcd->regs + 0x8650);
		xhci_dbg(xhci, "XHCLKGTEN(0x8650) after Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x8650));

		/* PCE 9C  0x40000 */
		temp = xhci_readl(xhci, hcd->regs + 0x869C);
		temp &= ~(0x1 << 21);
		temp &= ~(0x1 << 19);
		temp |= (0x1 << 18);
		temp &= ~(0x1 << 17);
		temp &= ~(0x1 << 16);
		xhci_writel(xhci, temp, hcd->regs + 0x869C);
		xhci_dbg(xhci, "PCE(0x869C) after Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x869C));

		/* HSCFG2 0x3800 */
		temp = xhci_readl(xhci, hcd->regs + 0x86A4);
		xhci_dbg(xhci, "HSCFG2 before Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x86A4));
		temp |= (0x3 << 11);
		xhci_writel(xhci, temp, hcd->regs + 0x86A4);
		xhci_dbg(xhci, "HSCFG2(0x86A4) after Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x86A4));

		/* SSCFG1 0x200CF */
		temp = xhci_readl(xhci, hcd->regs + 0x86A8);
		xhci_dbg(xhci, "SSCFG1 before Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x86A8));
		temp |= (0x1 << 17);
		temp &= ~(0x1 << 14);
		xhci_writel(xhci, temp, hcd->regs + 0x86A8);
		xhci_dbg(xhci, "SSCFG1(0x86A8) after Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x86A8));

		/* U2DEL and U1DEL 0x20000A */
		temp = xhci_readl(xhci, &xhci->cap_regs->hcs_params3);
		xhci_dbg(xhci, "HCSPARAM3 before Write = 0x%X\n", temp);
		temp &= ~(0x1 << 0);
		temp &= ~(0x1 << 18);
		temp |= (0x200 << 16);
		temp |= (0xA);
		xhci_writel(xhci, temp, &xhci->cap_regs->hcs_params3);
		xhci_dbg(xhci, "HCSPARAM3 after Write = 0x%X\n",
				xhci_readl(xhci, &xhci->cap_regs->hcs_params3));

		/* XECP_CMDM_CTRL_REG1 0x3511AEFC */
		temp = xhci_readl(xhci, hcd->regs + 0x818C);
		xhci_dbg(xhci, "XECP_CMDM_CTRL_REG1 before Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x818C));
		temp |= (0x1 << 20);
		temp |= (0x1 << 16);
		temp &= ~(0x1 << 8);
		xhci_writel(xhci, temp, hcd->regs + 0x818C);
		xhci_dbg(xhci, "XECP_CMDM_CTRL_REG1(0x818C) after Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x818C));

		/* XECP_CMDM_CTRL_REG3 0x220505A */
		temp = xhci_readl(xhci, hcd->regs + 0x8194);
		xhci_dbg(xhci, "XECP_CMDM_CTRL_REG3 before Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x8194));
		temp |= (0x1 << 25);
		xhci_writel(xhci, temp, hcd->regs + 0x8194);
		xhci_dbg(xhci, "XECP_CMDM_CTRL_REG3(0x8194) after Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x8194));

		/* PMCTRL 0x1C1FF94 */
		temp = xhci_readl(xhci, hcd->regs + 0x80A4);
		xhci_dbg(xhci, "PMCTRL before Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x80A4));
		temp &= ~(0x1 << 31);
		temp &= ~(0x1 << 29);
		temp |= (0x1 << 24);
		temp |= (0x1 << 23);
		temp |= (0x1 << 22);
		temp |= (0x1 << 2);
		xhci_writel(xhci, temp, hcd->regs + 0x80A4);
		xhci_dbg(xhci, "PMCTRL(0x80A4) after Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x80A4));

		/* AUX_CTRL_REG1  0x80CCBCE0 */
		temp = xhci_readl(xhci, hcd->regs + 0x80E0);
		xhci_dbg(xhci, "AUX_CTRL_REG1 before Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x80E0));
		temp |= (0x1 << 22);
		temp &= ~(0x1 << 16);
		temp &= ~(0x1 << 9);
		temp |= (0x1 << 6);
		xhci_writel(xhci, temp, hcd->regs + 0x80E0);
		xhci_dbg(xhci, "AUX_CTRL_REG1(0x80E0) after Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x80E0));

		/* HOST_CTRL_SCH_REG  0xA0C100 */
		temp = xhci_readl(xhci, hcd->regs + 0x8094);
		xhci_dbg(xhci, "HOST_CTRL_SCH_REG before Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x8094));
		temp |= (0x1 << 23);
		temp |= (0x1 << 21);
		temp |= (0x1 << 14);
		xhci_writel(xhci, temp, hcd->regs + 0x8094);
		xhci_dbg(xhci, "HOST_CTRL_SCH_REG(0x8094) after Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x8094));

		/* HOST_CTRL_TRM_REG2 0xF0FC8B88 */
		temp = xhci_readl(xhci, hcd->regs + 0x8110);
		xhci_dbg(xhci, "HOST_CTRL_TRM_REG2 before Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x8110));
		temp |= (0x1 << 20);
		temp |= (0x1 << 11);
		temp &= ~(0x1 << 2);
		xhci_writel(xhci, temp, hcd->regs + 0x8110);
		xhci_dbg(xhci, "HOST_CTRL_TRM_REG2(0x8110) after Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x8110));

		/* AUX_CTRL_REG2 0x81192206 */
		temp = xhci_readl(xhci, hcd->regs + 0x8154);
		xhci_dbg(xhci, "AUX_CTRL_REG2 before Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x8154));
		temp |= (0x1 << 31);
		temp &= ~(0x1 << 21);
		temp |= (0x1 << 13);
		xhci_writel(xhci, temp, hcd->regs + 0x8154);
		xhci_dbg(xhci, "AUX_CTRL_REG2 after Write(0x8154) = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x8154));

		/* AUX_CLOCK_CONTROL 0xC403C */
		temp = xhci_readl(xhci, hcd->regs + 0x816C);
		xhci_dbg(xhci, "AUX_CLOCK_CTRL before Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x816C));
		temp |= (0x1 << 19);
		temp |= (0x1 << 18);
		temp |= (0x1 << 14);
		temp &= ~(0x3 << 12);
		temp &= ~(0xF << 8);
		temp |= (0x1 << 5);
		temp |= (0x1 << 4);
		temp |= (0x1 << 3);
		temp |= (0x1 << 2);
		xhci_writel(xhci, temp, hcd->regs + 0x816C);
		xhci_dbg(xhci, "AUX_CLOCK_CTRL after(0x816C) Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x816C));

		/* IF_PWR_CTRL_REG0  0xFF03C132 */
		temp = xhci_readl(xhci, hcd->regs + 0x8140);
		xhci_dbg(xhci, "IF_PWR_CTRL_REG0 before Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x8140));
		temp |= (0xFF << 24);
		temp &= ~(0x1 << 12);
		temp &= ~(0x1 << 15);
		temp &= ~(0x1 << 16);
		temp |= (0x3C << 12);
		temp |= (0x132 << 0);
		xhci_writel(xhci, temp, hcd->regs + 0x8140);
		xhci_dbg(xhci, "IF_PWR_CTRL_REG0(0x8140) after Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x8140));

		/* IF_PWR_CTRL_REG1  0x33F */
		temp = xhci_readl(xhci, hcd->regs + 0x8144);
		xhci_dbg(xhci, "IF_PWR_CTRL_REG1 before Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x8144));
		temp |= (0x1 << 8);
		temp &= ~(0x1 << 7);
		temp &= ~(0x1 << 6);
		xhci_writel(xhci, temp, hcd->regs + 0x8144);
		xhci_dbg(xhci, "IF_PWR_CTRL_REG1(0x8144) after Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x8144));

		/* Latency Tolerance 0x40047D */
		temp = xhci_readl(xhci, hcd->regs + 0x8174);
		xhci_dbg(xhci, "Latency Tolerance before Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x8174));
		temp &= ~(0x1 << 24);
		xhci_writel(xhci, temp, hcd->regs + 0x8174);
		xhci_dbg(xhci, "Latency Tolerance(0x8174) after Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x8174));

		/* MISC REG 0x101037F */
		temp = xhci_readl(xhci, hcd->regs + 0x80B0);
		xhci_err(xhci, "MISC REG before Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x80B0));
		temp |= (0x1 << 24);
		xhci_writel(xhci, temp, hcd->regs + 0x80B0);
		xhci_err(xhci, "MISC REG(0x80B0) after Write = 0x%X\n",
				xhci_readl(xhci, hcd->regs + 0x80B0));
	}
	return 0;
}

/* xhci_ssic_init: Controller initialization flow for config local
 * and remote MPHY via MMIO registers.
 */
static int xhci_ssic_init(struct usb_hcd *hcd)
{
	struct xhci_hcd		*xhci;
	u32			temp;
	struct pci_dev		*pdev;
	int			retval;

	if (!hcd) {
		pr_err("%s hcd is NULL, return -EINVAL\n", __func__);
		return -EINVAL;
	}

	xhci = hcd_to_xhci(hcd);
	if (!xhci) {
		pr_err("%s xhci is NULL\n", __func__);
		return -ENODEV;
	}

	pdev = to_pci_dev(hcd->self.controller);
	if (!pdev) {
		pr_err("%s pdev is NULL, return\n", __func__);
		return -ENODEV;
	}

	/* ANN only MMIO register set config */
	if (pdev->vendor == PCI_VENDOR_ID_INTEL && pdev->device == PCI_DEVICE_ID_INTEL_MOOR_SSIC) {
		xhci_dbg(xhci, "ANN SSIC Controller, need to config MMIO register set\n");
		retval = ann_config_register(hcd);
		if (retval < 0) {
			xhci_dbg(xhci, "ANN config register return %d\n", retval);
			return retval;
		}
	}

	/* Config SSIC Configuration Register2 */
	temp = xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg2);
	xhci_dbg(xhci, " Config Register2 after boot up = 0x%08X\n", temp);
	/* Set the U3 workaround in driver */
	temp |= 0xF << 21;
	xhci_writel(xhci, temp, &ssic_hcd.policy_regs->config_reg2);

	temp = xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg2);
	xhci_dbg(xhci, "config register2 after write U3 workaround = 0x%X\n", temp);

	/* Configure Local/Remote MPHY. */
	ssic_config_register_bank(xhci, ssic_register_bank, ARRAY_SIZE(ssic_register_bank));
	xhci_ssic_disable_feature(xhci);

	/* Config SSIC Configuration Register2 */
	temp = xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg2);
	xhci_dbg(xhci, " Config Register2 = 0x%08X\n", temp);

	/*WA: Set T_ACT_H8_EXIT = 0x7 to fix U3 exit failure */
	temp = xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg1);
	temp |= 0x7 << 8;
	xhci_writel(xhci, temp, &ssic_hcd.policy_regs->config_reg1);
	temp = xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg1);
	xhci_dbg(xhci, "Config Regsiter1 = 0x%08X\n", temp);

	return 0;
}

static int xhci_ipc_get_ports(struct usb_hcd *hcd,
				__le32 __iomem ***port_array)
{
	int max_ports;
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);

	if (hcd->speed == HCD_USB3) {
		max_ports = xhci->num_usb3_ports;
		*port_array = xhci->usb3_ports;
	} else {
		max_ports = xhci->num_usb2_ports;
		*port_array = xhci->usb2_ports;
	}

	return max_ports;
}

static int xhci_ipc_hub_control(struct usb_hcd *hcd,
				u16 type_request, u16 type_value,
				u16 type_index, char *buf, u16 type_length)
{
	struct xhci_hcd *xhci = hcd_to_xhci(hcd);
	int		max_ports;
	unsigned long	flags;
	u32		temp, value;
	u16		index;
	int		retval = 0;
	__le32 __iomem	**port_array;

	if (hsic_pdata->has_ssic) {
		max_ports = xhci_ipc_get_ports(hcd, &port_array);

		/**
		 * WorkAround: sighting HSD 5015749 for ANN/CHT A0
		 * Controller stuck in USP disconnect state due to
		 * no pwr ack from SSIC MPHY.
		 * Details: After USP disconnect and Controller sends
		 * PORTSC event(CSC) event with CCS == 0
		 * Driver need to do following steps:
		 * 1) Set DL_PWR_GATE_DIS bit
		 * 2) Wait for 5us
		 * 3) Clear DL_PWR_GATE_DIS bit
		 */

		xhci_dbg(xhci, "max_ports = %d\n", max_ports);

		spin_lock_irqsave(&xhci->lock, flags);

		index = type_index;

		if (type_request == SSIC_GET_PORT_STATUS) {
			if (!index || index > max_ports) {
				xhci_err(xhci, "wIndex == 0 or wIndex > max_ports\n");
				retval = -EPIPE;
				goto error;
			}

			index--;

			/* 1) Must be careful about the index match for SSIC
			 * 2) Do NOT do any modification of type_index, otherwise
			 * driver will stuck.
			 */
			if (type_index == ssic_hcd.ssic_port) {
				temp = xhci_readl(xhci, port_array[index]);
				if (temp == 0xffffffff) {
					retval = -ENODEV;
					goto error;
				}

				xhci_dbg(xhci, "get port status, actual port %d status  = 0x%x\n",
							index, temp);

				/* CSC == 1  && CCS == 0 */
				if ((temp & PORT_CSC) && !(temp & PORT_CONNECT)) {
					/* set the DL_PWR_GATE_DIS_BIT */
					value = xhci_readl(xhci,
							&ssic_hcd.policy_regs->config_reg3);
					xhci_dbg(xhci, "config reg3 = 0x%08X\n",
							value);
					xhci_writel(xhci, value | DL_PWR_GATE_DIS,
							&ssic_hcd.policy_regs->config_reg3);
					xhci_dbg(xhci,
							"config reg3 after write = 0x%08X\n",
							value);
					/* FIXME OR use usleep is also enough? */
					udelay(5);

					value &= ~DL_PWR_GATE_DIS;
					xhci_writel(xhci, value,
							&ssic_hcd.policy_regs->config_reg3);
				}
			}
		}

		spin_unlock_irqrestore(&xhci->lock, flags);
	}

		retval = xhci_hub_control(hcd, type_request,
					type_value, type_index,
					buf, type_length);

	return retval;
error:
	spin_unlock_irqrestore(&xhci->lock, flags);
	return retval;
}

static int create_ssic_class_device_files(struct pci_dev *pdev)
{
	int retval;
	ssic_class = class_create(NULL, "ssic");

	if (IS_ERR(ssic_class))
		return -EFAULT;

	ssic_class_dev = device_create(ssic_class, &pdev->dev,
			MKDEV(0, 0), NULL, "ssic0");

	if (IS_ERR(ssic_class_dev)) {
		retval = -EFAULT;
		goto ssic_class_fail;
	}

	retval = device_create_file(ssic_class_dev, &dev_attr_ssic_registers);
	if (retval < 0) {
		dev_dbg(&pdev->dev, "error create ssic_register\n");
		goto ssic_class_dev_fail;
	}

	retval = device_create_file(ssic_class_dev, &dev_attr_ssic_enable);
	if (retval < 0) {
		dev_dbg(&pdev->dev, "error create ssic_enable\n");
		goto ssic_class_dev_fail;
	}

	retval = device_create_file(ssic_class_dev,
				&dev_attr_autosuspend_enable);
	if (retval < 0) {
		dev_dbg(&pdev->dev, "Error create autosuspend_enable\n");
		goto ssic_class_dev_fail;
	}

	retval = device_create_file(ssic_class_dev,
				&dev_attr_port_inactivity_duration);
	if (retval < 0) {
		dev_dbg(&pdev->dev, "Error create port_inactiveDuration\n");
		goto ssic_class_dev_fail;
	}

	retval = device_create_file(ssic_class_dev,
				&dev_attr_bus_inactivity_duration);
	if (retval < 0) {
		dev_dbg(&pdev->dev, "Error create bus_inactiveDuration\n");
		goto ssic_class_dev_fail;
	}

	retval = device_create_file(ssic_class_dev,
				&dev_attr_u1_inactivity_duration);
	if (retval < 0) {
		dev_dbg(&pdev->dev, "Error create u1_inactiveDuration\n");
		goto ssic_class_dev_fail;
	}

	retval = device_create_file(ssic_class_dev,
				&dev_attr_u2_inactivity_duration);
	if (retval < 0) {
		dev_dbg(&pdev->dev, "Error create u2_inactiveDuration\n");
		goto ssic_class_dev_fail;
	}

	retval = device_create_file(ssic_class_dev,
				&dev_attr_u1_enable);
	if (retval < 0) {
		dev_dbg(&pdev->dev, "Error create u1_enable\n");
		goto ssic_class_dev_fail;
	}

	retval = device_create_file(ssic_class_dev,
				&dev_attr_u2_enable);
	if (retval < 0) {
		dev_dbg(&pdev->dev, "Error create u2_enable\n");
		goto ssic_class_dev_fail;
	}

	retval = device_create_file(ssic_class_dev,
				&dev_attr_pm_enable);
	if (retval < 0) {
		dev_dbg(&pdev->dev, "Error create pm_enable\n");
		goto ssic_class_dev_fail;
	}


	if (retval == 0)
		return retval;

ssic_class_dev_fail:
	device_destroy(ssic_class, ssic_class_dev->devt);
ssic_class_fail:
	class_destroy(ssic_class);
	return retval;
}

static void remove_ssic_class_device_files(void)
{
	device_destroy(ssic_class, ssic_class_dev->devt);
	class_destroy(ssic_class);
}

static int xhci_ssic_private_reset(struct usb_hcd *hcd)
{
	struct xhci_hcd		*xhci;
	u32			temp;
	int			i;
	struct pci_dev		*pdev;

	if (!hcd) {
		pr_err("%s hcd is NULL, return -EINVAL\n",
				__func__);
		return -EINVAL;
	}

	xhci = hcd_to_xhci(hcd);

	if (!xhci) {
		pr_err("%s xhci is NULL\n", __func__);
		return -EINVAL;
	}

	pdev = to_pci_dev(hcd->self.controller);
	if (!pdev) {
		pr_err("%s pdev is NULL, return\n", __func__);
		return -ENODEV;
	}

	if (pdev->vendor == PCI_VENDOR_ID_INTEL && pdev->device == PCI_DEVICE_ID_INTEL_MOOR_SSIC) {
		if (!hsic_pdata) {
			pr_err("%s hsic_pdata is NULL, return\n", __func__);
			return -ENODEV;
		}

		if (hsic_pdata->has_ssic) {
			if (ssic_hcd.first_reset == 0) {
				ssic_hcd.policy_regs = hcd->regs + SSIC_POLICY_BASE;
				ssic_hcd.profile_regs = hcd->regs + SSIC_LOCAL_REMOTE_PROFILE_REGISTER;
				ssic_hcd.first_reset = 1;

				/* disable SSIC port1 for ANN */
				temp = xhci_readl(xhci, (&ssic_hcd.policy_regs->config_reg2 + 12));
				xhci_dbg(xhci, "CONFIG register2 for Port1 before write = 0x%X\n", temp);
				temp |= SSIC_PORT_UNUSED;
				xhci_writel(xhci, temp,
						(&ssic_hcd.policy_regs->config_reg2 + 12));
				xhci_dbg(xhci, "CONFIG register2 for Port1 after write = 0x%X\n",
						xhci_readl(xhci,
							(&ssic_hcd.policy_regs->config_reg2 + 12)));

				/* Config SSIC Configuration Register2 */
				temp = xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg2);
				xhci_dbg(xhci, "Config register2 = %X\n", temp);
				/* disable SSIC port0 for ANN for the first time bring-up */
				temp = xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg2);
				temp |= SSIC_PORT_UNUSED;
				xhci_writel(xhci, temp, &ssic_hcd.policy_regs->config_reg2);
				xhci_dbg(xhci, "CONFIG register2 for Port0 after write = 0x%X\n",
						xhci_readl(xhci, (&ssic_hcd.policy_regs->config_reg2)));
			}

		/* do flow before do HCRST every time */
		for (i = 0; i < 2; i++) {
			/* read the config register 2 */
			temp =  xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg2 + 12 * i);
			if (temp & SSIC_PORT_UNUSED) {
				temp &= ~PROG_DONE;
				xhci_dbg(xhci, "Clear PROG_DONE for port %d\n", i);
				xhci_writel(xhci, temp, (&ssic_hcd.policy_regs->config_reg2 + 12 * i));
				xhci_dbg(xhci, "CONFIG register2  after clear PROG_DONE = 0x%X\n",
						xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg2 + 12 * i));

				temp &= ~SSIC_PORT_UNUSED;
				xhci_writel(xhci, temp, (&ssic_hcd.policy_regs->config_reg2 + 12 * i));
				xhci_dbg(xhci, "CONFIG register2  after clear PORT_UNUSED = 0x%X\n",
						xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg2 + 12 * i));

				temp |= PROG_DONE;
				xhci_dbg(xhci, "Set PROG_DONE for port %d\n", i);
				xhci_writel(xhci, temp, (&ssic_hcd.policy_regs->config_reg2 + 12 * i));
				xhci_dbg(xhci, "CONFIG register2  after Set PROG_DONE = 0x%X\n",
				xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg2 + 12 * i));

				xhci_dbg(xhci, "Begin to do msleep 100 ms\n");
				msleep(100);
				xhci_dbg(xhci, "End of msleep 100 ms\n");

				temp &= ~PROG_DONE;
				xhci_dbg(xhci, "Clear PROG_DONE for port %d\n", i);
				xhci_writel(xhci, temp, (&ssic_hcd.policy_regs->config_reg2 + 12 * i));
				xhci_dbg(xhci, "CONFIG register2  after clear PROG_DONE = 0x%X\n",
				xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg2 + 12 * i));

				temp |= SSIC_PORT_UNUSED;
				xhci_writel(xhci, temp, (&ssic_hcd.policy_regs->config_reg2 + 12 * i));
				xhci_dbg(xhci, "CONFIG register2  after set PORT_UNUSED = 0x%X\n",
				xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg2 + 12 * i));

				temp |= PROG_DONE;
				xhci_dbg(xhci, "Set PROG_DONE for port %d\n", i);
				xhci_writel(xhci, temp, (&ssic_hcd.policy_regs->config_reg2 + 12 * i));
				xhci_dbg(xhci, "CONFIG register2  after Set PROG_DONE = 0x%X\n",
				xhci_readl(xhci, &ssic_hcd.policy_regs->config_reg2 + 12 * i));

				}
			}
		}
	}

	return 0;
}
