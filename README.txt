These are drivers to use Data Display Group ArtistaUSB devices
under Linux.

To compile the drivers for the local machine and currently
running kernel, simply type "make". To compile for a different
kernel version, set KERNEL_VERSION and KERNEL_DIR accordingly.

For cross-compilation, refer to your cross-compilation guru
for now... (normally, setting KERNEL_VERSION and KERNEL_DIR as
above and additionally ARCH and CROSS_COMPILE should be
sufficient. This is an example command line:
make ARCH=arm CROSS_COMPILE=/bigdisk/home/ww1/Artista/ArtistaNET-III/Software/toolchain/arm-2010q1/bin/arm-none-linux-gnueabi- KERNEL_DIR=/bigdisk/home/ww1/Artista/ArtistaNET-III/Software/kernel/trunk KERNEL_VERSION=2.6.36 INSTALL_DIR=/bigdisk/home/ww1/Artista/ArtistaNET-III/Software/filesystem/rootfs )


FEATURES

Framebuffer driver:
The framebuffer registers a device /dev/fb%d which can be used
by arbitrary programs. mmap()ed operation is supported using
the framebuffer deferred i/o framework.

Touch driver:
The touch driver uses the stored calibration data of the device.
The pressure threshold can also used to distinguish between pure
coordinate and button events by using half the threshold value
as the button threshold if the experimental feature flag
EXPERIMENTAL_BUTTON_THRESHOLD is set to 1 during compilation.
The driver provides two device nodes: /dev/input/event%d and
/dev/input/mouse%d.

Calibration tool:
This is a very simple tool that can be used to calibrate the
touch screen of the ArtistaUSB device. It needs at least 3
parameters: framebuffer device, touch event device, and the
sysfs control file for the calibration data. To find the
sysfs control file, currently use "find /sys -name calib".
The calibration data is read and written by reading or writing
a stream of numbers to this file. The calibration tool clears
the framebuffer screen and draws 3 crosshairs in sequence to the
screen for the user to press on.
The calibration tool does not need any special libraries - this
makes it easier to compile or port, but less comfortable. Further
improvements should adhere to this and not introduce any libraries
either.

A sample config for xorg is included in the package; this sample
configuration has so far only been tested on x86, so for usage
on an embedded system, the keyboard input most likely will have
to be changed.


KNOWN ISSUES

- the touch driver can not operate correctly without calibration
  data stored in the ArtistaUSB device (calibtool is now supplied
  for the calibration)
- CONFIG_FB_DEFERRED_IO=y is needed in the kernel configuration.
  This can not be explicitly set, enabling CONFIG_HID_PICOLCD=m
  is a workaround. The resulting module does not have to be
  installed, but the kernel has to be re-built with the configuration
  resulting in some bits being added to fb_info data structure.
- in 2.6. series kernels, the framebuffer driver has some unload issues
  leaving the old device present when using /dev/fb0. This is under
  investigation. Workaround: load another framebuffer driver (vfb)
  before to have a permanent /dev/fb0 device.
- artistausb_ops_blank always reports device /dev/fb0 regardless
  of the actual device being claimed.
- finding device names and sysfs files (esp. raw usb touch device)
  is not straight forward


TODO

- improve calibration access
- fix bugs ;-)
- more testing. Current test systems include:
  2.6.36 (ARM)
  2.6.32-5 (x86, Debian)
  2.6.39.4 (x86, vanilla)
  3.2.0-0 (amd64, Debian)
