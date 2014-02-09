/*
 * ArtistaUSB framebuffer driver
 * Copyright (c) 2012 Distec GmbH (wegner@distec.de)
 * based on usb-skeleton.c, smscufxusb.c, (udlfb.c)
 *
 * usb-skeleton.c parts:
 *
 * Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com)
 *
 * smscufxusb.c and (if any) udlfb.c parts:
 *
 * Copyright (C) 2011 Steve Glendinning <steve.glendinning@smsc.com>
 * Copyright (C) 2009 Roberto De Ioris <roberto@unbit.it>
 * Copyright (C) 2009 Jaya Kumar <jayakumar.lkml@gmail.com>
 * Copyright (C) 2009 Bernie Thompson <bernie@plugable.com>
 *
 * Adjusted as a proper kernel module for current kernels >= 3.7 
 * by Bim.Overbohm@googlemail.com in February 2014.
 * The orginal Distec driver crashed the kernel when displaying 
 * pictures on the console, thus I tried to make it work using the 
 * udlfb 0.4 kernel module as a reference.
 * I made it work and did some code cleanup, not much more...
 * TODO: Touch is probably not working. Try using dd_usbtouch.
 * TODO: Region writes not implemented. Take a look at udlfb for info.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/version.h>
#include <generated/autoconf.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/usb.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include "dd_usb.h"

#define check_warn_return(status, fmt, args...) \
	({ if (status < 0) { pr_warn(fmt, ##args); return status; } })

#define check_warn_goto_error(status, fmt, args...) \
	({ if (status < 0) { pr_warn(fmt, ##args); goto error; } })

/* some defines ----------------------------------------------------------- */

#define VX_GET_BKL                  0xB0
#define VX_SET_BKL                  0xB1
#define VX_GET_DISPLAY_POWER        0xB4
#define VX_SET_DISPLAY_POWER        0xB5
#define VX_GET_TRANSFER_COUNTER     0xB6
#define VX_SET_TRANSFER_COUNTER     0xB7
#define VX_GET_CALIBRATION_DATA     0xBC
#define VX_SET_CALIBRATION_DATA     0xBD
#define VX_RESOLUTION               0xC4

/*
 * TODO: Propose standard fb.h ioctl for reporting damage,
 * using _IOWR() and one of the existing area structs from fb.h
 * Consider these ioctls deprecated, but they're still used by the
 * DisplayLink X server as yet - need both to be modified in tandem
 * when new ioctl(s) are ready.
 */
// #define artistausb_IOCTL_RETURN_EDID	(0xAD)
#define artistausb_IOCTL_REPORT_DAMAGE	(0xAA)

/* -BULK_SIZE as per usb-skeleton. Can we get full page and avoid overhead? */
#define BULK_SIZE		(512)
#define MAX_TRANSFER		(PAGE_SIZE*16 - BULK_SIZE)
#define WRITES_IN_FLIGHT	(4)

#define GET_URB_TIMEOUT		(HZ)
#define FREE_URB_TIMEOUT	(HZ*2)

#define BPP			2

#define ARTISTAUSB_DEFIO_WRITE_DELAY	5 /* fb_deferred_io.delay in jiffies */
#define ARTISTAUSB_DEFIO_WRITE_DISABLE	(HZ*60) /* "disable" with long delay */

/*
 * more or less a marker to leave some code snippets in for later if we
 * can do region write.
 * If set to 0, always the complete frame buffer is transferred.
 * Regional write would require a modified protocol.
 */
#define REGION_WRITE		0

/* daste structures ------------------------------------------------------- */

typedef struct _TouchData{
	uint16_t x;
	uint16_t y;
	uint16_t pressure;
} TouchData;  

typedef struct _CalibrateData {
	int32_t x_scale;
	int32_t y_scale;
	int32_t xy_scale;
	int32_t yx_scale;
	int16_t x_offset;
	int16_t y_offset;
	uint8_t flags; /* reserved for future use (e.g. swap_xy, ...) */
	uint8_t filter_depth; /* min. 2; max = MAX_FILTER_DEPTH = 40 */
	uint16_t pressure_threshold;
} CalibrateData;
  
typedef struct
{
	uint8_t depth;
	uint8_t offset;
} ColorMapping;

typedef struct
{
	uint16_t x;
	uint16_t y;
	ColorMapping b;
	ColorMapping g;
	ColorMapping r;
	ColorMapping a;
	uint16_t mode; /* rotate, ... */
} Resolution;

struct dloarea {
	int x, y;
	int w, h;
};

struct urb_node {
	struct list_head entry;
	struct artistausb_data *dev;
	struct delayed_work release_urb_work;
	struct urb *urb;
};

struct urb_list {
	struct list_head list;
	spinlock_t lock;
	struct semaphore limit_sem;
	int available;
	int count;
	size_t size;
};

struct artistausb_data {
	struct usb_device *udev;
	struct device *gdev; /* &udev->dev */
	struct fb_info *info;
	struct urb_list urbs;
	struct kref kref;
	int fb_count;
	bool virtualized; /* true when physical usb device not present */
	struct delayed_work free_framebuffer_work;
	atomic_t usb_active; /* 0 = update virtual buffer, but no usb traffic */
	atomic_t lost_pixels; /* 1 = a render op failed. Need screen refresh */
	u8 *edid; /* null until we read edid from hw or get from sysfs */
	size_t edid_size;
	u32 pseudo_palette[256];
	int blank_mode;
};

static struct fb_fix_screeninfo artistausb_fix = {
	.id =           "artistausbfb",
	.type =         FB_TYPE_PACKED_PIXELS,
	.visual =       FB_VISUAL_DIRECTCOLOR,
	.xpanstep =     0,
	.ypanstep =     0,
	.ywrapstep =    0,
	.accel =        FB_ACCEL_NONE,
};

static const u32 artistausb_info_flags = FBINFO_DEFAULT | FBINFO_READS_FAST |
	FBINFO_VIRTFB |	FBINFO_HWACCEL_IMAGEBLIT | FBINFO_HWACCEL_FILLRECT |
	FBINFO_HWACCEL_COPYAREA | FBINFO_MISC_ALWAYS_SETPAR;

static struct usb_device_id id_table[] = {
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_XGA_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_XGA)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_WXGA_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_WXGA)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_SVGA_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_SVGA)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_SXGA_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_SXGA)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_WVGA_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_WVGA)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_FULLHD_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_FULLHD)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_VGA_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_VGA)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_WSVGA_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_WSVGA)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_II)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_ECO_VGA_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_ECO_VGA)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_ECO_SVGA_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_ECO_SVGA)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_ECO_XGA_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_ECO_XGA)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_ECO_WSVGA_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_ECO_WSVGA)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_ECO_WVGA_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_ECO_WVGA)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_ECO_1440_TOUCH)},
	{USB_DEVICE(USB_VENDOR_ID_DISTEC, USB_PRODUCT_ID_ARTISTAUSB_ECO_1440)},
	{}
};
MODULE_DEVICE_TABLE(usb, id_table);

/* module options --------------------------------------------------------- */

static bool console = 0;   /* Optionally allow fbcon to consume first framebuffer */
static bool fb_defio = 1;  /* Optionally enable fb_defio mmap support */

/* set up framebuffer operations ------------------------------------------ */

static void artistausb_urb_completion(struct urb *urb);
static struct urb *artistausb_get_urb(struct artistausb_data *dev);
static int artistausb_submit_urb(struct artistausb_data *dev, struct urb * urb, size_t len);
static int artistausb_alloc_urb_list(struct artistausb_data *dev, int count, size_t size);
static void artistausb_free_urb_list(struct artistausb_data *dev);

/* Forward declarations needed for fb_ops setup */
static int artistausb_ops_blank(int blank_mode, struct fb_info *info);
static int artistausb_ops_mmap(struct fb_info *info, struct vm_area_struct *vma);
static ssize_t artistausb_ops_write(struct fb_info *info, const char __user *buf, size_t count, loff_t *ppos);
static void artistausb_ops_copyarea(struct fb_info *info, const struct fb_copyarea *area);
static void artistausb_ops_fillrect(struct fb_info *info, const struct fb_fillrect *rect);
static void artistausb_ops_imageblit(struct fb_info *info, const struct fb_image *image);
static int artistausb_ops_open(struct fb_info *info, int user);
static int artistausb_ops_release(struct fb_info *info, int user);
static int artistausb_ops_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg);
static int artistausb_ops_setcolreg(unsigned regno, unsigned red, unsigned green, unsigned blue, unsigned transp, struct fb_info *info);
static int artistausb_ops_check_var(struct fb_var_screeninfo *var, struct fb_info *info);
static int artistausb_ops_set_par(struct fb_info *info);

static struct fb_ops artistausb_ops = {
	.owner = THIS_MODULE,
	.fb_read = fb_sys_read,
	.fb_write = artistausb_ops_write,
	.fb_setcolreg = artistausb_ops_setcolreg,
	.fb_fillrect = artistausb_ops_fillrect,
	.fb_copyarea = artistausb_ops_copyarea,
	.fb_imageblit = artistausb_ops_imageblit,
	.fb_mmap = artistausb_ops_mmap,
	.fb_ioctl = artistausb_ops_ioctl,
	.fb_open = artistausb_ops_open,
	.fb_release = artistausb_ops_release,
	.fb_blank = artistausb_ops_blank,
	.fb_check_var = artistausb_ops_check_var,
	.fb_set_par = artistausb_ops_set_par,
};

/* framebuffer device operations ------------------------------------------ */

static int artistausb_set_transfercount(struct artistausb_data *dev)
{
	int retval;
	unsigned char buffer[2];

	buffer[0] = 0x01;
	buffer[1] = VX_SET_TRANSFER_COUNTER;
	retval = usb_control_msg(dev->udev, usb_rcvctrlpipe(dev->udev, 0),
		VX_SET_TRANSFER_COUNTER, USB_DIR_IN | USB_TYPE_VENDOR,
		0x0004, 0x00, buffer, 0x02, USB_CTRL_GET_TIMEOUT);
	if (retval != 0x02)
		return -EFAULT;

	return 0;
}

static int artistausb_init(struct artistausb_data *dev)
{
	int result;

	result = artistausb_set_transfercount(dev);
	/* anything to do here? */

	return result;
}

static int artistausb_set_vid_mode(struct artistausb_data *dev, struct fb_var_screeninfo *var)
{
	/* no video mode setting currently... */
	return 0;
}

static int artistausb_ops_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	unsigned long start = vma->vm_start;
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long page, pos;

	if (vma->vm_pgoff > (~0UL >> PAGE_SHIFT))
		return -EINVAL;
	if (size > info->fix.smem_len)
		return -EINVAL;
	if (offset > info->fix.smem_len - size)
		return -EINVAL;

	pos = (unsigned long)info->fix.smem_start + offset;

	pr_debug("mmap() framebuffer addr:%lu size:%lu\n",
		  pos, size);

	while (size > 0) {
		page = vmalloc_to_pfn((void *)pos);
		if (remap_pfn_range(vma, start, page, PAGE_SIZE, PAGE_SHARED))
			return -EAGAIN;

		start += PAGE_SIZE;
		pos += PAGE_SIZE;
		if (size > PAGE_SIZE)
			size -= PAGE_SIZE;
		else
			size = 0;
	}

	return 0;
}

/*
 * This is the core function that does actual data transfer!
 */
int artistausb_handle_damage(struct artistausb_data *dev, int x, int y,
	int width, int height)
{
	size_t packed_line_len;
	int len, status, urb_lines, line, start_line = 0;

    //pr_notice("Painting (%d,%d) %dx%d\n", x, y, width, height);

	packed_line_len = ALIGN((width * BPP), 4);
	if ((width <= 0) || (height <= 0) ||
	    (x + width > dev->info->var.xres) ||
	    (y + height > dev->info->var.yres)) {
        pr_warn("Bad width/height!\n");
		return -EINVAL;
    }

	if (!atomic_read(&dev->usb_active)) {
        pr_warn("Device inactive!\n");
		return 0;
    }

	status = artistausb_ops_blank(FB_BLANK_UNBLANK, dev->info);
	if (status) {
		pr_warn("%s: unblank failed", __FUNCTION__);
		return 0;
	}

	if (atomic_read(&dev->lost_pixels)) {
		status = artistausb_set_transfercount(dev);
		if (status) {
			pr_warn("%s: reset transfercount failed", __FUNCTION__);
			return 0;
		}
		/* is this safe, or do we need locking? */
		atomic_set(&dev->lost_pixels, 0);
	}

	while (start_line < height) {
		struct urb *urb = artistausb_get_urb(dev);
		if (!urb) {
			pr_warn("%s: unable to get urb", __FUNCTION__);
			return 0;
		}

		/* assume we have enough space to transfer at least one line */
		BUG_ON(urb->transfer_buffer_length < (width * BPP));

		/* calculate the maximum number of lines we could fit in */
		urb_lines = urb->transfer_buffer_length / packed_line_len;

		/* but we might not need this many */
		urb_lines = min(urb_lines, (height - start_line));

		/*
		 * This was present in smsc driver, but I don't see its use?
		 * memset(urb->transfer_buffer, 0, urb->transfer_buffer_length);
		 */
		for (line = 0; line < urb_lines; line++) {
			const int line_offset = dev->info->fix.line_length * (y + start_line + line);
			const int byte_offset = line_offset + (x * BPP);
			memcpy(urb->transfer_buffer + (packed_line_len * line),
				(char *)dev->info->fix.smem_start + byte_offset, width * BPP);
		}
		len = packed_line_len * urb_lines;

		status = artistausb_submit_urb(dev, urb, len);
		check_warn_return(status, "Error submitting URB");

		start_line += urb_lines;
	}

	return 0;
}

/* Path triggered by usermode clients who write to filesystem
 * e.g. cat filename > /dev/fb1
 * Not used by X Windows or text-mode console. But useful for testing.
 * Slow because of extra copy and we must assume all pixels dirty. */
static ssize_t artistausb_ops_write(struct fb_info *info, const char __user *buf,
			  size_t count, loff_t *ppos)
{
	ssize_t result;
	struct artistausb_data *dev = info->par;
	u32 offset = (u32) *ppos;

	result = fb_sys_write(info, buf, count, ppos);

	if (result > 0) {
		int start = max((int)(offset / info->fix.line_length), 0);// - 1, 0);
		int lines = min((u32)((result / info->fix.line_length) + 1),
				(u32)info->var.yres);

		artistausb_handle_damage(dev, 0, start, info->var.xres, lines);
	}
    else {
        pr_warn("fb_sys_write failed: %d", (int)result);
    }

	return result;
}

static void artistausb_ops_copyarea(struct fb_info *info,
				const struct fb_copyarea *area)
{

	struct artistausb_data *dev = info->par;

	sys_copyarea(info, area);

	artistausb_handle_damage(dev, area->dx, area->dy,
			area->width, area->height);
}

static void artistausb_ops_imageblit(struct fb_info *info,
				const struct fb_image *image)
{
	struct artistausb_data *dev = info->par;

	sys_imageblit(info, image);

	artistausb_handle_damage(dev, image->dx, image->dy,
			image->width, image->height);
}

static void artistausb_ops_fillrect(struct fb_info *info,
			  const struct fb_fillrect *rect)
{
	struct artistausb_data *dev = info->par;

	sys_fillrect(info, rect);

	artistausb_handle_damage(dev, rect->dx, rect->dy, rect->width,
			      rect->height);
}

/* NOTE: fb_defio.c is holding info->fbdefio.mutex
 *   Touching ANY framebuffer memory that triggers a page fault
 *   in fb_defio will cause a deadlock, when it also tries to
 *   grab the same mutex. */
static void artistausb_dpy_deferred_io(struct fb_info *info,
				struct list_head *pagelist)
{
#if REGION_WRITE
	struct page *cur;
	struct fb_deferred_io *fbdefio = info->fbdefio;
#endif
	struct artistausb_data *dev = info->par;

	if (!fb_defio)
		return;

	if (!atomic_read(&dev->usb_active))
		return;

#if REGION_WRITE
	/* walk the written page list and render each to device */
	list_for_each_entry(cur, &fbdefio->pagelist, lru) {
		/* create a rectangle of full screen width that encloses the
		 * entire dirty framebuffer page */
		const int x = 0;
		const int width = dev->info->var.xres;
		const int y = (cur->index << PAGE_SHIFT) / (width * 2);
		int height = (PAGE_SIZE / (width * 2)) + 1;
		height = min(height, (int)(dev->info->var.yres - y));

		BUG_ON(y >= dev->info->var.yres);
		BUG_ON((y + height) > dev->info->var.yres);

		artistausb_handle_damage(dev, x, y, width, height);
	}
#else
	artistausb_handle_damage(dev, 0, 0, info->var.xres, info->var.yres);
#endif
}

static int artistausb_ops_ioctl(struct fb_info *info, unsigned int cmd,
			 unsigned long arg)
{
	struct artistausb_data *dev = info->par;

	if (!atomic_read(&dev->usb_active))
		return 0;

#if 0
	/* TODO: Update X server to get this from sysfs instead */
	/* TODO (ww1): prepare fake EDID data for client to use */
	if (cmd == artistausb_IOCTL_RETURN_EDID) {
		u8 __user *edid = (u8 __user *)arg;
		if (copy_to_user(edid, dev->edid, dev->edid_size))
			return -EFAULT;
		return 0;
	}
#endif

	/* TODO: Help propose a standard fb.h ioctl to report mmap damage */
	if (cmd == artistausb_IOCTL_REPORT_DAMAGE) {
		struct dloarea area;

		if (copy_from_user(&area, (void __user *)arg,
				  sizeof(struct dloarea)))
			return -EFAULT;

		/* If we have a damage-aware client, turn fb_defio "off"
		 * To avoid perf imact of unecessary page fault handling.
		 * Done by resetting the delay for this fb_info to a very
		 * long period. Pages will become writable and stay that way.
		 * Reset to normal value when all clients have closed this fb.
		 */
		if (info->fbdefio)
			info->fbdefio->delay = ARTISTAUSB_DEFIO_WRITE_DISABLE;

		if (area.x < 0)
			area.x = 0;

		if (area.x > info->var.xres)
			area.x = info->var.xres;

		if (area.y < 0)
			area.y = 0;

		if (area.y > info->var.yres)
			area.y = info->var.yres;

		artistausb_handle_damage(dev, area.x, area.y, area.w, area.h);
	}

	return 0;
}

/* taken from vesafb */
static int artistausb_ops_setcolreg(unsigned regno, unsigned red, unsigned green,
	       unsigned blue, unsigned transp, struct fb_info *info)
{
	int err = 0;

	if (regno >= info->cmap.len)
		return 1;

	if (regno < 16) {
		if (info->var.red.offset == 10) {
			/* 1:5:5:5 */
			((u32 *) (info->pseudo_palette))[regno] =
			    ((red & 0xf800) >> 1) |
			    ((green & 0xf800) >> 6) | ((blue & 0xf800) >> 11);
		} else {
			/* 0:5:6:5 */
			((u32 *) (info->pseudo_palette))[regno] =
			    ((red & 0xf800)) |
			    ((green & 0xfc00) >> 5) | ((blue & 0xf800) >> 11);
		}
	}

	return err;
}

/* It's common for several clients to have framebuffer open simultaneously.
 * e.g. both fbcon and X. Makes things interesting.
 * Assumes caller is holding info->lock (for open and release at least) */
static int artistausb_ops_open(struct fb_info *info, int user)
{
	struct artistausb_data *dev = info->par;

	/* fbcon aggressively connects to first framebuffer it finds,
	 * preventing other clients (X) from working properly. Usually
	 * not what the user wants. Fail by default with option to enable. */
	if ((user == 0) && (!console))
		return -EBUSY;

	/* If the USB device is gone, we don't accept new opens */
	if (dev->virtualized)
		return -ENODEV;

	dev->fb_count++;

	kref_get(&dev->kref);

	if (fb_defio && (info->fbdefio == NULL)) {
		/* enable defio at last moment if not disabled by client */

		struct fb_deferred_io *fbdefio;

		fbdefio = kzalloc(sizeof(struct fb_deferred_io), GFP_KERNEL);

		if (fbdefio) {
			fbdefio->delay = ARTISTAUSB_DEFIO_WRITE_DELAY;
			fbdefio->deferred_io = artistausb_dpy_deferred_io;
		}

		info->fbdefio = fbdefio;
		fb_deferred_io_init(info);
	}

	pr_debug("open /dev/fb%d user=%d fb_info=%p count=%d",
		info->node, user, info, dev->fb_count);

	return 0;
}

/*
 * Called when all client interfaces to start transactions have been disabled,
 * and all references to our device instance (artistausb_data) are released.
 * Every transaction must have a reference, so we know are fully spun down
 */
static void artistausb_free(struct kref *kref)
{
	struct artistausb_data *dev = container_of(kref, struct artistausb_data, kref);

	pr_debug("freeing artistausb_data %p", dev);

	kfree(dev);
}

static void artistausb_release_urb_work(struct work_struct *work)
{
	struct urb_node *unode = container_of(work, struct urb_node,
					      release_urb_work.work);

	up(&unode->dev->urbs.limit_sem);
}

static void artistausb_free_framebuffer(struct artistausb_data *dev)
{
	struct fb_info *info = dev->info;

	if (info) {
		int node = info->node;

		unregister_framebuffer(info);

		if (info->cmap.len != 0)
			fb_dealloc_cmap(&info->cmap);
		if (info->screen_base)
			vfree(info->screen_base);

		fb_destroy_modelist(&info->modelist);

		dev->info = NULL;

		/* Assume info structure is freed after this point */
		framebuffer_release(info);

		pr_warn("fb_info for /dev/fb%d has been freed\n", node);
	}

	/* ref taken in probe() as part of registering framebfufer */
	kref_put(&dev->kref, artistausb_free);
}

static void artistausb_free_framebuffer_work(struct work_struct *work)
{
	struct artistausb_data *dev = container_of(work, struct artistausb_data,
					     free_framebuffer_work.work);
	artistausb_free_framebuffer(dev);
}

/*
 * Assumes caller is holding info->lock mutex (for open and release at least)
 */
static int artistausb_ops_release(struct fb_info *info, int user)
{
	struct artistausb_data *dev = info->par;

	dev->fb_count--;

	/* We can't free fb_info here - fbmem will touch it when we return */
	if (dev->virtualized && (dev->fb_count == 0))
		schedule_delayed_work(&dev->free_framebuffer_work, HZ);

	if ((dev->fb_count == 0) && (info->fbdefio)) {
		fb_deferred_io_cleanup(info);
		kfree(info->fbdefio);
		info->fbdefio = NULL;
		info->fbops->fb_mmap = artistausb_ops_mmap;
	}

	pr_debug("released /dev/fb%d user=%d count=%d",
		  info->node, user, dev->fb_count);

	kref_put(&dev->kref, artistausb_free);

	return 0;
}

/*
 * Check whether a video mode is supported;
 * currently, we only support native resolution mode.
 */
static int artistausb_is_valid_mode(struct fb_videomode *mode,
		struct fb_info *info)
{
	if (info->var.xres != mode->xres)
		return 0;
	if (info->var.yres != mode->yres)
		return 0;

	return 1;
}

static void artistausb_var_color_format(struct fb_var_screeninfo *var)
{
	const struct fb_bitfield red = { 11, 5, 0 };
	const struct fb_bitfield green = { 5, 6, 0 };
	const struct fb_bitfield blue = { 0, 5, 0 };

	var->bits_per_pixel = 16;
	var->red = red;
	var->green = green;
	var->blue = blue;
}

static int artistausb_ops_check_var(struct fb_var_screeninfo *var,
				struct fb_info *info)
{
	struct fb_videomode mode;

	/* TODO: support dynamically changing framebuffer size */
	if ((var->xres * var->yres * 2) > info->fix.smem_len)
		return -EINVAL;

	/* set device-specific elements of var unrelated to mode */
	artistausb_var_color_format(var);

	fb_var_to_videomode(&mode, var);

	if (!artistausb_is_valid_mode(&mode, info))
		return -EINVAL;

	return 0;
}

static int artistausb_ops_set_par(struct fb_info *info)
{
	struct artistausb_data *dev = info->par;
	int result;
	u16 *pix_framebuffer;
	int i;

	pr_debug("set_par mode %dx%d", info->var.xres, info->var.yres);
	result = artistausb_set_vid_mode(dev, &info->var);

	if ((result == 0) && (dev->fb_count == 0)) {
		/* paint greenscreen */
		pix_framebuffer = (u16 *) info->screen_base;
		for (i = 0; i < info->fix.smem_len / 2; i++)
			pix_framebuffer[i] = 0xb0ef;//0x37e6;

		artistausb_handle_damage(dev, 0, 0, info->var.xres, info->var.yres);
	}

	/* re-enable defio if previously disabled by damage tracking */
/*	if (info->fbdefio)
		info->fbdefio->delay = ARTISTAUSB_DEFIO_WRITE_DELAY;*/

	return result;
}

static int artistausb_ops_blank(int blank_mode, struct fb_info *info)
{
	struct artistausb_data *dev = info->par;
	int retval;
	int power;
	unsigned char buffer[2];

	if ((dev->blank_mode == blank_mode) &&
	    (blank_mode != FB_BLANK_POWERDOWN)) {
		return 0;
	}

	pr_info("/dev/fb%d FB_BLANK mode %d --> %d\n",
		info->node, dev->blank_mode, blank_mode);

	power = (blank_mode == FB_BLANK_UNBLANK)?0x100:0;

	buffer[0] = 0x01;
	buffer[1] = VX_SET_DISPLAY_POWER;
	retval = usb_control_msg(dev->udev, usb_rcvctrlpipe(dev->udev, 0),
		VX_SET_DISPLAY_POWER, USB_DIR_IN | USB_TYPE_VENDOR,
		0x0001 | power, 0x00, buffer, 0x02, USB_CTRL_GET_TIMEOUT);
	if (retval != 0x02)
		return -EFAULT;

	buffer[0] = VX_SET_BKL;
	retval = usb_control_msg(dev->udev, usb_rcvctrlpipe(dev->udev, 0),
		VX_SET_BKL, USB_DIR_IN | USB_TYPE_VENDOR,
		0x0001 | power, 0x0, buffer, 0x01, USB_CTRL_GET_TIMEOUT);
	if (retval != 0x01)
		return -EFAULT;

	dev->blank_mode = blank_mode;

	return 0;
}

/* Assumes &info->lock held by caller
 * Assumes no active clients have framebuffer open */
static int artistausb_realloc_framebuffer(struct artistausb_data *dev, struct fb_info *info)
{
	int retval = -ENOMEM;
	int old_len = info->fix.smem_len;
	int new_len;
	unsigned char *old_fb = info->screen_base;
	unsigned char *new_fb;

	pr_debug("Reallocating framebuffer. Addresses will change!");

	new_len = info->fix.line_length * info->var.yres;

	if (PAGE_ALIGN(new_len) > old_len) {
		/*
		 * Alloc system memory for virtual framebuffer
		 */
		new_fb = vmalloc(new_len);
		if (!new_fb) {
			pr_err("Virtual framebuffer alloc failed");
			goto error;
		}

		if (info->screen_base) {
			memcpy(new_fb, old_fb, old_len);
			vfree(info->screen_base);
		}

		info->screen_base = new_fb;
		info->fix.smem_len = PAGE_ALIGN(new_len);
		info->fix.smem_start = (unsigned long) new_fb;
		info->flags = artistausb_info_flags;
	}

	retval = 0;

error:
	return retval;
}

/* USB functions ---------------------------------------------------------- */

/*
 * Read ArtistaUSB resolution and set up framebuffer parameter structure
 * with sensible values.
 * Returns 0 if successful
 */
static int artistausb_setup_mode(struct artistausb_data *dev, struct fb_info *info,
	char *default_edid, size_t default_edid_size)
{
	int result = 0;
	u8 buffer[16];
	Resolution *res = (Resolution *)buffer;
	struct fb_videomode fb_vmode = {0};

	if (info->dev) /* only use mutex if info has been registered */
		mutex_lock(&info->lock);

	/* read resolution */
	result = usb_control_msg(dev->udev, usb_rcvctrlpipe(dev->udev, 0),
		VX_RESOLUTION, USB_DIR_IN | USB_TYPE_VENDOR,
		0x00, 0x00, buffer, 0x0E, USB_CTRL_GET_TIMEOUT);
	if (result != 0x0E)
		goto error;
	/* convert x and y resolution from big endian to host format */
	res->x = be16_to_cpu(res->x);
	res->y = be16_to_cpu(res->y);
	pr_debug("ArtistaUSB: Resolution %dx%d\n",
		 res->x, res->y);

	fb_vmode.xres = res->x;
	fb_vmode.yres = res->y;
	fb_vmode.refresh = 60;

	info->var.xres = res->x;
	info->var.yres = res->y;
	info->var.xres_virtual = res->x;
	info->var.yres_virtual = res->y;
	info->var.xoffset = 0;
	info->var.yoffset = 0;
	/* fake values from ArtistaNET-III driver */
	info->var.pixclock = 20000;
	info->var.left_margin = 64;
	info->var.right_margin = 64;
	info->var.upper_margin = 32;
	info->var.lower_margin = 32;
	info->var.hsync_len = 64;
	info->var.vsync_len = 2;
	info->var.sync = 0;
	info->var.vmode = FB_VMODE_NONINTERLACED & FB_VMODE_MASK;

	artistausb_var_color_format(&info->var);

	/* with mode size info, we can now alloc our framebuffer */
	memcpy(&info->fix, &artistausb_fix, sizeof(artistausb_fix));
	info->fix.line_length = info->var.xres *
		(info->var.bits_per_pixel / 8);

	result = artistausb_realloc_framebuffer(dev, info);

	/* WW1 FIXME: should we set up info->modelist here, too? */

error:
	if (info->dev)
		mutex_unlock(&info->lock);

	return result;
}

static int artistausb_usb_probe(struct usb_interface *interface,
			const struct usb_device_id *id)
{
	struct usb_device *usbdev;
	struct artistausb_data *dev;
	struct fb_info *info = 0;
	int retval = -ENOMEM;

	/* usb initialization */
	usbdev = interface_to_usbdev(interface);
	BUG_ON(!usbdev);

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (dev == NULL) {
		dev_err(&usbdev->dev, "artistausb_usb_probe: failed alloc of dev struct\n");
		goto error;
	}

	dev->udev = usb_get_dev(interface_to_usbdev(interface));
	if (interface != usb_ifnum_to_if(dev->udev, 0)) {
		retval = -ENODEV;
		goto error;
	}

	/* we need to wait for both usb and fbdev to spin down on disconnect */
	kref_init(&dev->kref); /* matching kref_put in usb .disconnect fn */
	kref_get(&dev->kref); /* matching kref_put in free_framebuffer_work */

	dev->udev = usbdev;
	dev->gdev = &usbdev->dev; /* our generic struct device * */
	usb_set_intfdata(interface, dev);

	dev_dbg(dev->gdev, "%s %s - serial #%s\n",
		usbdev->manufacturer, usbdev->product, usbdev->serial);
	dev_dbg(dev->gdev, "vid_%04x&pid_%04x&rev_%04x driver's artistausb_data struct at %p\n",
		usbdev->descriptor.idVendor, usbdev->descriptor.idProduct,
		usbdev->descriptor.bcdDevice, dev);
	dev_dbg(dev->gdev, "console enable=%d\n", console);
	dev_dbg(dev->gdev, "fb_defio enable=%d\n", fb_defio);

	if (!artistausb_alloc_urb_list(dev, WRITES_IN_FLIGHT, MAX_TRANSFER)) {
		retval = -ENOMEM;
		dev_err(dev->gdev, "artistausb_alloc_urb_list failed\n");
		goto error;
	}

	/* We don't register a new USB class. Our client interface is fbdev */

	/* allocates framebuffer driver structure, not framebuffer memory */
	info = framebuffer_alloc(0, &usbdev->dev);
	if (!info) {
		retval = -ENOMEM;
		dev_err(dev->gdev, "framebuffer_alloc failed\n");
		goto error;
	}

	dev->info = info;
	dev->blank_mode = -1; /* force execution of artistausb_ops_blank() */
	info->par = dev;
	info->pseudo_palette = dev->pseudo_palette;
	info->fbops = &artistausb_ops;

	retval = fb_alloc_cmap(&info->cmap, 256, 0);
	if (retval < 0) {
		dev_err(dev->gdev, "fb_alloc_cmap failed %x\n", retval);
		goto error;
	}

	INIT_DELAYED_WORK(&dev->free_framebuffer_work,
			  artistausb_free_framebuffer_work);

	INIT_LIST_HEAD(&info->modelist);

	dev_dbg(dev->gdev, "resetting device");
	retval = artistausb_init(dev);
	check_warn_goto_error(retval, "error %d resetting device", retval);

	dev_dbg(dev->gdev, "selecting display mode");
	retval = artistausb_setup_mode(dev, info, NULL, 0);
	check_warn_goto_error(retval, "unable to read mode of display");

	/* ready to begin using device */
	atomic_set(&dev->usb_active, 1);

	dev_dbg(dev->gdev, "checking var");
	retval = artistausb_ops_check_var(&info->var, info);
	check_warn_goto_error(retval, "error %d artistausb_ops_check_var", retval);

	dev_dbg(dev->gdev, "setting par");
	retval = artistausb_ops_set_par(info);
	check_warn_goto_error(retval, "error %d artistausb_ops_set_par", retval);

	dev_dbg(dev->gdev, "registering framebuffer");
	retval = register_framebuffer(info);
	check_warn_goto_error(retval, "error %d register_framebuffer", retval);

	dev_info(dev->gdev, "ArtistaUSB device /dev/fb%d attached. %dx%d resolution."
		" Using %dK framebuffer memory. Driver version %s\n", info->node,
		info->var.xres, info->var.yres, info->fix.smem_len >> 10, DRIVER_VERSION);

	return 0;

error:
	if (dev) {
		if (info) {
			if (info->cmap.len != 0)
				fb_dealloc_cmap(&info->cmap);
			if (info->screen_base)
				vfree(info->screen_base);

			fb_destroy_modelist(&info->modelist);

			framebuffer_release(info);
		}

		kref_put(&dev->kref, artistausb_free); /* ref for framebuffer */
		kref_put(&dev->kref, artistausb_free); /* last ref from kref_init */

		/* dev has been deallocated. Do not dereference */
	}

	return retval;
}

static void artistausb_usb_disconnect(struct usb_interface *interface)
{
	struct artistausb_data *dev;
	struct fb_info *info;

	dev = usb_get_intfdata(interface);
	info = dev->info;

	pr_debug("USB disconnect starting\n");

	/* we virtualize until all fb clients release. Then we free */
	dev->virtualized = true;

	/* When non-active we'll update virtual framebuffer, but no new urbs */
	atomic_set(&dev->usb_active, 0);

	/* this function will wait for all in-flight urbs to complete */
	artistausb_free_urb_list(dev);

	if (info) {
		/* remove dd_usbfb's sysfs interfaces */
		unlink_framebuffer(info);
	}

	usb_set_intfdata(interface, NULL);
	dev->udev = NULL;
	dev->gdev = NULL;

	/* if clients still have us open, will be freed on last close */
	if (dev->fb_count == 0)
		schedule_delayed_work(&dev->free_framebuffer_work, 0);

	/* release reference taken by kref_init in probe() */
	kref_put(&dev->kref, artistausb_free);

	/* consider artistausb_data freed */
}

static void artistausb_urb_completion(struct urb *urb)
{
	struct urb_node *unode = urb->context;
	struct artistausb_data *dev = unode->dev;
	unsigned long flags;

	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		    urb->status == -ECONNRESET ||
		    urb->status == -ESHUTDOWN)) {
			pr_err("%s - nonzero write bulk status received: %d\n",
				__func__, urb->status);
			atomic_set(&dev->lost_pixels, 1);
		}
	}

	urb->transfer_buffer_length = dev->urbs.size; /* reset to actual */

	spin_lock_irqsave(&dev->urbs.lock, flags);
	list_add_tail(&unode->entry, &dev->urbs.list);
	dev->urbs.available++;
	spin_unlock_irqrestore(&dev->urbs.lock, flags);

	/*
	 * When using fb_defio, we deadlock if up() is called
	 * while another is waiting. So queue to another process.
	 */
	if (fb_defio)
		schedule_delayed_work(&unode->release_urb_work, 0);
	else
		up(&dev->urbs.limit_sem);
}

static void artistausb_free_urb_list(struct artistausb_data *dev)
{
	int count = dev->urbs.count;
	struct list_head *node;
	struct urb_node *unode;
	struct urb *urb;
	int ret;
	unsigned long flags;

	pr_debug("Waiting for completes and freeing all render urbs\n");

	/* keep waiting and freeing, until we've got 'em all */
	while (count--) {
		/* Getting interrupted means a leak, but ok at shutdown*/
		ret = down_interruptible(&dev->urbs.limit_sem);
		if (ret)
			break;

		spin_lock_irqsave(&dev->urbs.lock, flags);

		node = dev->urbs.list.next; /* have reserved one with sem */
		list_del_init(node);

		spin_unlock_irqrestore(&dev->urbs.lock, flags);

		unode = list_entry(node, struct urb_node, entry);
		urb = unode->urb;

		/* Free each separately allocated piece */
		usb_free_coherent(urb->dev, dev->urbs.size,
				  urb->transfer_buffer, urb->transfer_dma);
		usb_free_urb(urb);
		kfree(node);
	}

	dev->urbs.count = 0;
}

static int artistausb_alloc_urb_list(struct artistausb_data *dev, int count, size_t size)
{
	int i = 0;
	struct urb *urb;
	struct urb_node *unode;
	char *buf;

	spin_lock_init(&dev->urbs.lock);

	dev->urbs.size = size;
	INIT_LIST_HEAD(&dev->urbs.list);

	while (i < count) {
		unode = kzalloc(sizeof(struct urb_node), GFP_KERNEL);
		if (!unode)
			break;
		unode->dev = dev;

		INIT_DELAYED_WORK(&unode->release_urb_work,
			  artistausb_release_urb_work);

		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb) {
			kfree(unode);
			break;
		}
		unode->urb = urb;

		buf = usb_alloc_coherent(dev->udev, MAX_TRANSFER /*size*/, GFP_KERNEL,
					 &urb->transfer_dma);
		if (!buf) {
			kfree(unode);
			usb_free_urb(urb);
			break;
		}

		/* urb->transfer_buffer_length set to actual before submit */
		usb_fill_bulk_urb(urb, dev->udev, usb_sndbulkpipe(dev->udev, 2),
			buf, size, artistausb_urb_completion, unode);
		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

		list_add_tail(&unode->entry, &dev->urbs.list);

		i++;
	}

	sema_init(&dev->urbs.limit_sem, i);
	dev->urbs.count = i;
	dev->urbs.available = i;

	pr_debug("allocated %d %d byte urbs\n", i, (int) size);

	return i;
}

static struct urb *artistausb_get_urb(struct artistausb_data *dev)
{
	int ret = 0;
	struct list_head *entry;
	struct urb_node *unode;
	struct urb *urb = NULL;
	unsigned long flags;

	/* Wait for an in-flight buffer to complete and get re-queued */
	ret = down_timeout(&dev->urbs.limit_sem, GET_URB_TIMEOUT);
	if (ret) {
		atomic_set(&dev->lost_pixels, 1);
		pr_warn("wait for urb interrupted: %x available: %d\n",
		       ret, dev->urbs.available);
		goto error;
	}

	spin_lock_irqsave(&dev->urbs.lock, flags);

	BUG_ON(list_empty(&dev->urbs.list)); /* reserved one with limit_sem */
	entry = dev->urbs.list.next;
	list_del_init(entry);
	dev->urbs.available--;

	spin_unlock_irqrestore(&dev->urbs.lock, flags);

	unode = list_entry(entry, struct urb_node, entry);
	urb = unode->urb;

error:
	return urb;
}

static int artistausb_submit_urb(struct artistausb_data *dev, struct urb *urb, size_t len)
{
	int ret;

	BUG_ON(len > dev->urbs.size);

	urb->transfer_buffer_length = len; /* set to actual payload len */
	ret = usb_submit_urb(urb, GFP_KERNEL);
	if (ret) {
		artistausb_urb_completion(urb); /* because no one else will */
		atomic_set(&dev->lost_pixels, 1);
		pr_err("usb_submit_urb error %x\n", ret);
	}
	return ret;
}

/* module setup ----------------------------------------------------------- */

static struct usb_driver artistausb_driver = {
	.name = "artistausbfb",
	.probe = artistausb_usb_probe,
	.disconnect = artistausb_usb_disconnect,
	.id_table = id_table,
};

module_usb_driver(artistausb_driver);

module_param(console, bool, S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP);
MODULE_PARM_DESC(console, "Allow fbcon to be used on this display");

module_param(fb_defio, bool, S_IWUSR | S_IRUSR | S_IWGRP | S_IRGRP);
MODULE_PARM_DESC(fb_defio, "Enable fb_defio mmap support");

MODULE_AUTHOR("Wolfgang Wegner <wegner@distec.de>, Bim Overbohm <Bim.Overbohm@googlemail.com>");
MODULE_DESCRIPTION("ArtistaUSB kernel framebuffer driver");
MODULE_LICENSE("GPL");
