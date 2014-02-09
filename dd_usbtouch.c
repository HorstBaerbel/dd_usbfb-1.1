/******************************************************************************
 * dd_artistausbtouch.c
 * Driver for ArtistaUSB touchscreen portion.
 *
 * Copyright (C) 2012 by Wolfgang Wegner <wegner@distec.de>
 *
 * This driver is based on and derived from
 * usbtouchscreen.c:
 *
 * Copyright (C) 2004-2007 by Daniel Ritz <daniel.ritz@gmx.ch>
 * Copyright (C) by Todd E. Johnson (mtouchusb.c)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *****************************************************************************/

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/usb/input.h>
#include <linux/hid.h>

#include "dd_usb.h"

#define DRIVER_AUTHOR		"Wolfgang Wegner <wegner@distec.de>"
#define DRIVER_DESC		"ArtistaUSB Touchscreen Driver"

/*
 * Experimental feature: emulate real mouse pointer by
 * sending a button event only when the pressure (actually,
 * resistance) value is below half the threshold value.
 */
#define EXPERIMENTAL_BUTTON_THRESHOLD	0

static bool swap_xy;
module_param(swap_xy, bool, 0644);
MODULE_PARM_DESC(swap_xy, "If set, X and Y axes are swapped.");

/* sorry, does not yet work... */
static bool emulate_rightbutton;
module_param(emulate_rightbutton, bool, 0644);
MODULE_PARM_DESC(emulate_rightbutton, "If set, right button is emulated by pressing longer than 3 seconds.");

/* device specifc data/functions */
struct artistausbtouch_usb;

/* a artistausbtouch device */
struct artistausbtouch_usb {
	unsigned char *data;
	dma_addr_t data_dma;
	unsigned char *buffer;
	int buf_len;
	struct urb *irq;
	struct usb_interface *interface;
	struct input_dev *input;
	char name[128];
	char phys[64];
	void *priv;

	int min_x, max_x;
	int min_y, max_y;
	int min_press, max_press;
	int threshold;
};

static const struct usb_device_id artistausbtouch_devices[] = {
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_XGA_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_WXGA_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_SVGA_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_SXGA_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_WVGA_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_FULLHD_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_VGA_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_WSVGA_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_ECO_VGA_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_ECO_SVGA_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_ECO_XGA_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_ECO_WSVGA_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_ECO_WVGA_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_ECO_1440_TOUCH)},
	{}
};


/*****************************************************************************
 * Generic Part
 */
static void artistausbtouch_process_pkt(struct artistausbtouch_usb *artistausbtouch,
                                 unsigned char *pkt, int len)
{
	int x, y;
	int touch, press;
	TouchData *data = (TouchData *)pkt;

	x = le16_to_cpu(data->x);
	y = le16_to_cpu(data->y);
	press = le16_to_cpu(data->pressure);

	/* WW1: is there a better algorithm to decide which event to generate? */
#if EXPERIMENTAL_BUTTON_THRESHOLD
	if (press < (artistausbtouch->threshold / 2))
#else
	if (press < artistausbtouch->threshold)
#endif
		touch = 1;
	else
		touch = 0;

	/* calculate correct pressure value */
	if (press < artistausbtouch->threshold)
		press = artistausbtouch->threshold - press;
	else
		press = 0;

	input_report_key(artistausbtouch->input, BTN_TOUCH, touch);

	if (swap_xy) {
		input_report_abs(artistausbtouch->input, ABS_X, y);
		input_report_abs(artistausbtouch->input, ABS_Y, x);
	} else {
		input_report_abs(artistausbtouch->input, ABS_X, x);
		input_report_abs(artistausbtouch->input, ABS_Y, y);
	}

	input_report_abs(artistausbtouch->input, ABS_PRESSURE, press);
	input_sync(artistausbtouch->input);
}



static void artistausbtouch_irq(struct urb *urb)
{
	struct artistausbtouch_usb *artistausbtouch = urb->context;
	int retval;

	switch (urb->status) {
	case 0:
		/* success */
		break;
	case -ETIME:
		/* this urb is timing out */
		pr_info("%s - urb timed out - was the device unplugged?",
		    __func__);
		return;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
	case -EPIPE:
		/* this urb is terminated, clean up */
		pr_info("%s - urb shutting down with status: %d",
		    __func__, urb->status);
		return;
	default:
		pr_info("%s - nonzero urb status received: %d",
		    __func__, urb->status);
		goto exit;
	}

	artistausbtouch_process_pkt(artistausbtouch, artistausbtouch->data, urb->actual_length);

exit:
	usb_mark_last_busy(interface_to_usbdev(artistausbtouch->interface));
	retval = usb_submit_urb(urb, GFP_ATOMIC);
	if (retval)
		pr_err("%s - usb_submit_urb failed with result: %d",
		    __func__, retval);
}

static int artistausbtouch_open(struct input_dev *input)
{
	struct artistausbtouch_usb *artistausbtouch = input_get_drvdata(input);
	int r;

	artistausbtouch->irq->dev = interface_to_usbdev(artistausbtouch->interface);

	r = usb_autopm_get_interface(artistausbtouch->interface) ? -EIO : 0;
	if (r < 0)
		goto out;

	if (usb_submit_urb(artistausbtouch->irq, GFP_KERNEL)) {
		r = -EIO;
		goto out_put;
	}

	artistausbtouch->interface->needs_remote_wakeup = 1;
out_put:
	usb_autopm_put_interface(artistausbtouch->interface);
out:
	return r;
}

static void artistausbtouch_close(struct input_dev *input)
{
	struct artistausbtouch_usb *artistausbtouch = input_get_drvdata(input);
	int r;

	usb_kill_urb(artistausbtouch->irq);
	r = usb_autopm_get_interface(artistausbtouch->interface);
	artistausbtouch->interface->needs_remote_wakeup = 0;
	if (!r)
		usb_autopm_put_interface(artistausbtouch->interface);
}

static int artistausbtouch_suspend
(struct usb_interface *intf, pm_message_t message)
{
	struct artistausbtouch_usb *artistausbtouch = usb_get_intfdata(intf);

	usb_kill_urb(artistausbtouch->irq);

	return 0;
}

static int artistausbtouch_resume(struct usb_interface *intf)
{
	struct artistausbtouch_usb *artistausbtouch = usb_get_intfdata(intf);
	struct input_dev *input = artistausbtouch->input;
	int result = 0;

	mutex_lock(&input->mutex);
	if (input->users)
		result = usb_submit_urb(artistausbtouch->irq, GFP_NOIO);
	mutex_unlock(&input->mutex);

	return result;
}

static int artistausbtouch_reset_resume(struct usb_interface *intf)
{
	struct artistausbtouch_usb *artistausbtouch = usb_get_intfdata(intf);
	struct input_dev *input = artistausbtouch->input;
	int err = 0;

	/* restart IO if needed */
	mutex_lock(&input->mutex);
	if (input->users)
		err = usb_submit_urb(artistausbtouch->irq, GFP_NOIO);
	mutex_unlock(&input->mutex);

	return err;
}

static void artistausbtouch_free_buffers(struct usb_device *udev,
				  struct artistausbtouch_usb *artistausbtouch)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 34)
	usb_buffer_free(udev, sizeof(TouchData),
			  artistausbtouch->data, artistausbtouch->data_dma);
#else
	usb_free_coherent(udev, sizeof(TouchData),
			  artistausbtouch->data, artistausbtouch->data_dma);
#endif
	kfree(artistausbtouch->buffer);
}

static struct usb_endpoint_descriptor *
artistausbtouch_get_input_endpoint(struct usb_host_interface *interface)
{
	int i;

	for (i = 0; i < interface->desc.bNumEndpoints; i++)
		if (usb_endpoint_dir_in(&interface->endpoint[i].desc))
			return &interface->endpoint[i].desc;

	return NULL;
}

/**
 * calibration data access via sysfs.
 */
static ssize_t get_calib_data(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct usb_device *udev = to_usb_device(dev);
	CalibrateData calib;
	int result;
	/* tmpdata is needed on 64 bit systems for proper conversion */
	int32_t tmpdata[4];

	/* read calibration data */
	result = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
		VX_GET_CALIBRATION_DATA, USB_DIR_IN | USB_TYPE_VENDOR,
		0x01, 0x00, (u8 *)&calib, 0x19, USB_CTRL_GET_TIMEOUT);
	if (result != 0x19) {
		pr_err("%s - reading calibration data failed", __func__);
		return 0;
	}
	/*
	 * temporary conversion to avoid problems with negative values
	 * on 64 bit systems
	 */
	tmpdata[0] = be32_to_cpu(calib.x_scale);
	tmpdata[1] = be32_to_cpu(calib.y_scale);
	tmpdata[2] = be32_to_cpu(calib.xy_scale);
	tmpdata[3] = be32_to_cpu(calib.yx_scale);
	return snprintf(buf, PAGE_SIZE, "%d %d %d %d %hd %hd 0x%hhx %hhu %hu\n",
		tmpdata[0], tmpdata[1], tmpdata[2], tmpdata[3],
		be16_to_cpu(calib.x_offset), be16_to_cpu(calib.y_offset),
		calib.flags, calib.filter_depth,
		be16_to_cpu(calib.pressure_threshold));
}

static ssize_t set_calib_data(struct device * dev,
	struct device_attribute *attr,
	const char * buf, size_t size)
{
	struct usb_device *udev = to_usb_device(dev);
	struct artistausbtouch_usb *artistausbtouch;
	CalibrateData calib;
	int i, offset, result;
	long ltemp[4];
	int temp[2];
	const char *buf_end = buf + size - 2;

	artistausbtouch = usb_get_intfdata(usb_ifnum_to_if(usb_get_dev(udev), 1));
	for (i = 0;
		(i < 4) && (buf < buf_end);
		i++) {
		if (1 != sscanf(buf, "%ld%n", &ltemp[i], &offset))
			return -EINVAL;
		buf += offset;
	}
	calib.x_scale = cpu_to_be32(ltemp[0]);
	calib.y_scale = cpu_to_be32(ltemp[1]);
	calib.xy_scale = cpu_to_be32(ltemp[2]);
	calib.yx_scale = cpu_to_be32(ltemp[3]);
	for (i = 0;
		(i < 2) && (buf < buf_end);
		i++) {
		if (1 != sscanf(buf, "%d%n", &temp[i], &offset))
			return -EINVAL;
		buf += offset;
	}
	calib.x_offset = cpu_to_be16(temp[0]);
	calib.y_offset = cpu_to_be16(temp[1]);
	if (1 != sscanf(buf, "%i%n", &temp[0], &offset))
		return -EINVAL;
	buf += offset;
	calib.flags = (u8)temp[0];
	for (i = 0;
		(i < 2) && (buf < buf_end);
		i++) {
		if (1 != sscanf(buf, "%u%n", &temp[i], &offset))
			return -EINVAL;
		buf += offset;
	}
	calib.filter_depth = (u8)temp[0];
	calib.pressure_threshold = cpu_to_be16(temp[1]);
	/* write calibration data */
	result = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
		VX_SET_CALIBRATION_DATA, USB_DIR_OUT | USB_TYPE_VENDOR,
		0x03, 0x00, (u8 *)&calib, 0x18, USB_CTRL_SET_TIMEOUT);
	if (result != 0x18) {
		pr_err("%s - writing calibration data failed, result %d", __func__, result);
		return -EFAULT;
	}
	artistausbtouch->threshold = be16_to_cpu(calib.pressure_threshold);
	artistausbtouch->min_press = 0;
	artistausbtouch->max_press = artistausbtouch->threshold;
	return size;
}

static DEVICE_ATTR(calib, S_IRUGO | S_IWUSR,
	get_calib_data, set_calib_data);

static int artistausbtouch_probe(struct usb_interface *intf,
			  const struct usb_device_id *id)
{
	struct artistausbtouch_usb *artistausbtouch;
	struct input_dev *input_dev;
	struct usb_endpoint_descriptor *endpoint;
	struct usb_device *udev = interface_to_usbdev(intf);
	int err = -ENOMEM;
	u8 buffer[25];
	Resolution *res = (Resolution *)buffer;
	CalibrateData *calib = (CalibrateData *)buffer;
	int result;

	if (intf != usb_ifnum_to_if(usb_get_dev(udev), 1))
		return -ENODEV; /* ignore other interfaces */

	endpoint = artistausbtouch_get_input_endpoint(intf->cur_altsetting);
	if (!endpoint)
		return -ENXIO;

	artistausbtouch = kzalloc(sizeof(struct artistausbtouch_usb), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!artistausbtouch || !input_dev)
		goto out_free;

	/* read resolution for max coordinates */
	result = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
		VX_RESOLUTION, USB_DIR_IN | USB_TYPE_VENDOR,
		0x00, 0x00, buffer, 0x0E, USB_CTRL_GET_TIMEOUT);
	if (result != 0x0E) {
		pr_err("%s - reading resolution failed", __func__);
		goto out_free;
	}
	/* convert x and y resolution from big endian to host format */
	artistausbtouch->min_x = 0;
	artistausbtouch->max_x = be16_to_cpu(res->x);
	artistausbtouch->min_y = 0;
	artistausbtouch->max_y = be16_to_cpu(res->y);

	/* read calibration data for pressure range */
	result = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
		VX_GET_CALIBRATION_DATA, USB_DIR_IN | USB_TYPE_VENDOR,
		0x01, 0x00, buffer, 0x19, USB_CTRL_GET_TIMEOUT);
	if (result != 0x19) {
		pr_err("%s - reading calibration data failed", __func__);
		goto out_free;
	}

	artistausbtouch->threshold = be16_to_cpu(calib->pressure_threshold);
	artistausbtouch->min_press = 0;
	artistausbtouch->max_press = artistausbtouch->threshold;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 34)
	artistausbtouch->data = usb_buffer_alloc(udev, sizeof(TouchData),
					    GFP_KERNEL, &artistausbtouch->data_dma);
#else
	artistausbtouch->data = usb_alloc_coherent(udev, sizeof(TouchData),
					    GFP_KERNEL, &artistausbtouch->data_dma);
#endif
	if (!artistausbtouch->data)
		goto out_free;

	artistausbtouch->irq = usb_alloc_urb(0, GFP_KERNEL);
	if (!artistausbtouch->irq) {
		pr_info("%s - usb_alloc_urb failed: artistausbtouch->irq", __func__);
		goto out_free_buffers;
	}

	artistausbtouch->interface = intf;
	artistausbtouch->input = input_dev;

	if (udev->manufacturer)
		strlcpy(artistausbtouch->name, udev->manufacturer, sizeof(artistausbtouch->name));

	if (udev->product) {
		if (udev->manufacturer)
			strlcat(artistausbtouch->name, " ", sizeof(artistausbtouch->name));
		strlcat(artistausbtouch->name, udev->product, sizeof(artistausbtouch->name));
	}

	if (!strlen(artistausbtouch->name))
		snprintf(artistausbtouch->name, sizeof(artistausbtouch->name),
			"ArtistaUSB Touchscreen %04x:%04x, driver version %s",
			 le16_to_cpu(udev->descriptor.idVendor),
			 le16_to_cpu(udev->descriptor.idProduct),
			 DRIVER_VERSION);

	usb_make_path(udev, artistausbtouch->phys, sizeof(artistausbtouch->phys));
	strlcat(artistausbtouch->phys, "/input0", sizeof(artistausbtouch->phys));

	input_dev->name = artistausbtouch->name;
	input_dev->phys = artistausbtouch->phys;
	usb_to_input_id(udev, &input_dev->id);
	input_dev->dev.parent = &intf->dev;

	input_set_drvdata(input_dev, artistausbtouch);

	input_dev->open = artistausbtouch_open;
	input_dev->close = artistausbtouch_close;

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(input_dev, ABS_X, artistausbtouch->min_x, artistausbtouch->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, artistausbtouch->min_y, artistausbtouch->max_y, 0, 0);
	if (artistausbtouch->max_press)
		input_set_abs_params(input_dev, ABS_PRESSURE, artistausbtouch->min_press,
		                     artistausbtouch->max_press, 0, 0);

	usb_fill_int_urb(artistausbtouch->irq, udev,
			 usb_rcvintpipe(udev, endpoint->bEndpointAddress),
			 artistausbtouch->data, sizeof(TouchData),
			 artistausbtouch_irq, artistausbtouch, endpoint->bInterval);

	artistausbtouch->irq->dev = udev;
	artistausbtouch->irq->transfer_dma = artistausbtouch->data_dma;
	artistausbtouch->irq->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	err = input_register_device(artistausbtouch->input);
	if (err) {
		pr_info("%s - input_register_device failed, err: %d", __func__, err);
		goto out_free_urb;
	}

	err = device_create_file(&udev->dev, &dev_attr_calib);
	if (err)
		goto out_unregister_device;

	usb_set_intfdata(intf, artistausbtouch);

	return 0;

/* skip to here in case more initialization is needed */
out_unregister_device:
	input_unregister_device(input_dev);
	input_dev = NULL;
out_free_urb:
	usb_free_urb(artistausbtouch->irq);
out_free_buffers:
	artistausbtouch_free_buffers(udev, artistausbtouch);
out_free:
	input_free_device(input_dev);
	kfree(artistausbtouch);
	return err;
}

static void artistausbtouch_disconnect(struct usb_interface *intf)
{
	struct artistausbtouch_usb *artistausbtouch = usb_get_intfdata(intf);
	struct usb_device *udev = interface_to_usbdev(intf);

	pr_info("%s - called", __func__);

	if (!artistausbtouch)
		return;

	pr_info("%s - artistausbtouch is initialized, cleaning up", __func__);
	usb_set_intfdata(intf, NULL);
	device_remove_file(&udev->dev, &dev_attr_calib);
	/* this will stop IO via close */
	input_unregister_device(artistausbtouch->input);
	usb_free_urb(artistausbtouch->irq);
	artistausbtouch_free_buffers(interface_to_usbdev(intf), artistausbtouch);
	kfree(artistausbtouch);
}

MODULE_DEVICE_TABLE(usb, artistausbtouch_devices);

static struct usb_driver artistausbtouch_driver = {
	.name		= "artistausbtouchscreen",
	.probe		= artistausbtouch_probe,
	.disconnect	= artistausbtouch_disconnect,
	.suspend	= artistausbtouch_suspend,
	.resume		= artistausbtouch_resume,
	.reset_resume	= artistausbtouch_reset_resume,
	.id_table	= artistausbtouch_devices,
	.supports_autosuspend = 1,
};

/*
 * module_usb_driver was introduced in linux-3.3.0 - the #ifdef was
 * stolen from some v4l discussion.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 3, 0)
static int __init dd_usbtouch_init(void)
{
	int result;

	result = usb_register(&artistausbtouch_driver);
	if (result)
		pr_err("usb_register failed. Error number %d", result);

	return result;
}

static void __exit dd_usbtouch_exit(void)
{
	usb_deregister(&artistausbtouch_driver);
}

module_init(dd_usbtouch_init);
module_exit(dd_usbtouch_exit);
#else
module_usb_driver(artistausbtouch_driver);
#endif

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
