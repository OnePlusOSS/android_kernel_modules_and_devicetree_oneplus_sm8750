// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2024 . Oplus All rights reserved.
*/

#define pr_fmt(fmt) "[OPLUS_GAMEPAD]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/notifier.h>
#include <linux/usb.h>
#include <linux/hid.h>
#include <linux/atomic.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>

#include <oplus_chg.h>
#include <oplus_mms_wired.h>

#define PD_GET_SINK_CAP_INTERVAL_MS	 900
static atomic_t other_device_cnt = ATOMIC_INIT(0);
static atomic_t is_mcu_connected = ATOMIC_INIT(0);
static atomic_t is_hub_connected = ATOMIC_INIT(0);

static void pd_get_sink_cap_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(pd_get_sink_cap_dwork, pd_get_sink_cap_work);

static const struct usb_device_id hub_phantom_csc_list[] = {
	{ USB_DEVICE(0x1a40, 0x0101) },
	{ USB_DEVICE(0x1a86, 0x8091) },
	{ } /* terminating entry */
};

/* Device that must not be disrupted by hub resets */
static const struct usb_device_id hub_protected_dev_list[] = {
	{ USB_DEVICE(0x22d9, 0x386b) },
	{ } /* terminating entry */
};

static int device_lo_or_hi_match(struct usb_device *dev,
			  const struct usb_device_id *id)
{
	if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_LO) &&
		(id->bcdDevice_lo > le16_to_cpu(dev->descriptor.bcdDevice)))
		return 0;

	if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_HI) &&
		(id->bcdDevice_hi < le16_to_cpu(dev->descriptor.bcdDevice)))
		return 0;

	return 1;
}

static int device_class_or_subclass_match(struct usb_device *dev,
			  const struct usb_device_id *id)
{
	if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_CLASS) &&
		(id->bDeviceClass != dev->descriptor.bDeviceClass))
		return 0;

	if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_SUBCLASS) &&
		(id->bDeviceSubClass != dev->descriptor.bDeviceSubClass))
		return 0;

	return 1;
}

static int oplus_usb_match_device(struct usb_device *dev, const struct usb_device_id *id)
{
	if ((id->match_flags & USB_DEVICE_ID_MATCH_VENDOR) &&
		id->idVendor != le16_to_cpu(dev->descriptor.idVendor))
		return 0;

	if ((id->match_flags & USB_DEVICE_ID_MATCH_PRODUCT) &&
		id->idProduct != le16_to_cpu(dev->descriptor.idProduct))
		return 0;

	/* No need to test id->bcdDevice_lo != 0, since 0 is never */
	/*   greater than any unsigned number. */
	if (!device_lo_or_hi_match(dev, id))
		return 0;

	if (!device_class_or_subclass_match(dev, id))
		return 0;

	if ((id->match_flags & USB_DEVICE_ID_MATCH_DEV_PROTOCOL) &&
		(id->bDeviceProtocol != dev->descriptor.bDeviceProtocol))
		return 0;

	return 1;
}

static bool is_mcu_device(struct usb_device *udev)
{
	const struct usb_device_id *id;

	for (id = hub_protected_dev_list; id->match_flags; id++) {
		if (oplus_usb_match_device(udev, id))
			return true;
	}
	return false;
}

static bool is_hub_device(struct usb_device *udev)
{
	const struct usb_device_id *id;

	for (id = hub_phantom_csc_list; id->match_flags; id++) {
		if (oplus_usb_match_device(udev, id))
			return true;
	}
	return false;
}

static bool is_device_on_hub(struct usb_device *udev)
{
	struct usb_device *hub_udev;

	if (udev == NULL || udev->parent == NULL)
		return false;

	hub_udev = udev->parent;
	chg_info("GAMEPAD: hub device(%04x, %04x)\n",
		le16_to_cpu(hub_udev->descriptor.idVendor), le16_to_cpu(hub_udev->descriptor.idProduct));
	if (is_hub_device(hub_udev))
		return true;
	return false;
}

static void pd_get_sink_cap_work(struct work_struct *work)
{
	if (!oplus_wired_is_gamepad_active()) {
		chg_info("gamepad not active, skip send get sink cap\n");
		return;
	}
	(void)oplus_wired_send_get_sink_cap();

	schedule_delayed_work(&pd_get_sink_cap_dwork,
			      msecs_to_jiffies(PD_GET_SINK_CAP_INTERVAL_MS));
}

static void pd_get_sink_cap_start(void)
{
	if (delayed_work_pending(&pd_get_sink_cap_dwork))
		return;

	mod_delayed_work(system_wq, &pd_get_sink_cap_dwork, 0);
}

static void hub_phantom_csc_on_remove(struct usb_device *udev)
{
	chg_info("OPLUS PD: remove device(%04x, %04x)\n",
		le16_to_cpu(udev->descriptor.idVendor), le16_to_cpu(udev->descriptor.idProduct));
	if (is_hub_device(udev))
		atomic_set(&is_hub_connected, 0);
	else if (is_mcu_device(udev))
		atomic_set(&is_mcu_connected, 0);
	else if (is_device_on_hub(udev))
		atomic_dec_if_positive(&other_device_cnt);

	chg_info("OPLUS PD: hub: %d, mcu: %d others: %d\n", atomic_read(&is_hub_connected),
		atomic_read(&is_mcu_connected), atomic_read(&other_device_cnt));
	if (atomic_read(&is_hub_connected) == 0 || atomic_read(&other_device_cnt) <= 0) {
		chg_info("OPLUS PD: cancel get sink cap\n");
		atomic_set(&other_device_cnt, 0);
		cancel_delayed_work_sync(&pd_get_sink_cap_dwork);
	}
}

static void hub_phantom_csc_on_add(struct usb_device *udev)
{
	chg_info("OPLUS PD: add device(%04x, %04x)\n",
		le16_to_cpu(udev->descriptor.idVendor), le16_to_cpu(udev->descriptor.idProduct));
	if (is_hub_device(udev))
		atomic_set(&is_hub_connected, 1);
	else if (is_mcu_device(udev))
		atomic_set(&is_mcu_connected, 1);
	else if (is_device_on_hub(udev))
		atomic_inc(&other_device_cnt);

	chg_info("OPLUS PD: hub: %d, mcu: %d others: %d\n", atomic_read(&is_hub_connected),
		atomic_read(&is_mcu_connected), atomic_read(&other_device_cnt));
	if (atomic_read(&other_device_cnt) > 0 && atomic_read(&is_hub_connected)
		&& atomic_read(&is_mcu_connected)) {
		chg_info("OPLUS PD: all devices connected, start get sink cap\n");
		pd_get_sink_cap_start();
	}
}

static int hub_phantom_csc_notify(struct notifier_block *nb,
	unsigned long action, void *data)
{
	struct usb_device *udev = data;

	if (udev == NULL)
		return NOTIFY_DONE;

	if (action != USB_DEVICE_REMOVE && action != USB_DEVICE_ADD)
		return NOTIFY_DONE;

	if (action == USB_DEVICE_REMOVE)
		hub_phantom_csc_on_remove(udev);

	if (action == USB_DEVICE_ADD)
		hub_phantom_csc_on_add(udev);

	return NOTIFY_DONE;
}

static struct notifier_block hub_phantom_csc_notifier = {
	.notifier_call = hub_phantom_csc_notify,
};

void oplus_gamepad_init(void)
{
	usb_register_notify(&hub_phantom_csc_notifier);
}

void oplus_gamepad_exit(void)
{
	usb_unregister_notify(&hub_phantom_csc_notifier);
	cancel_delayed_work_sync(&pd_get_sink_cap_dwork);
	atomic_set(&other_device_cnt, 0);
	atomic_set(&is_mcu_connected, 0);
	atomic_set(&is_hub_connected, 0);
}
