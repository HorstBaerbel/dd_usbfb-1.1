ArtistaUSB framebuffer driver for Data Display Group / Apollo Display Technologies board and USB monitors
========

This is a driver to use Data Display Group / Apollo Display Technologies [ArtistaUSB](www.datadisplay-group.de/tft-controller/tft-controller-usb-lan/artistausb/) devices under Linux. ArtistaUSB framebuffer driver for Distec USB monitors. It is a "fork" of the [original GPLv2 driver v1.0.0.35007](http://www.datadisplay-group.de/service/downloads/artista-downloads/).  
I adjusted it as a proper kernel module for current kernels >= 3.7. The orginal driver crashed the kernel when displaying pictures on the console, thus I tried to make it work using the udlfb 0.4 kernel module as a reference. I finally got it to run and did some code cleanup, not much more... 

License
========

[GPLv2](http://opensource.org/licenses/GPL-2.0) as original driver, see [LICENSE.md](LICENSE.md)

Based on original version 1.0.0.35077:  
Copyright (c) 2012 Distec GmbH (wegner@distec.de)

which is in turn based on usb-skeleton.c parts:  
Copyright (C) 2001-2004 Greg Kroah-Hartman (greg@kroah.com)

and based on smscufxusb.c and (if any) udlfb.c parts:  
Copyright (C) 2011 Steve Glendinning <steve.glendinning@smsc.com>  
Copyright (C) 2009 Roberto De Ioris <roberto@unbit.it>  
Copyright (C) 2009 Jaya Kumar <jayakumar.lkml@gmail.com>  
Copyright (C) 2009 Bernie Thompson <bernie@plugable.com>  

Building
========

This driver will only work for kernels >= 3.0 (most probably only for kernels >= 3.5). Make sure to also read the original [README.txt](README.txt). Please note that most of these steps need super-user rights aka su or sudo...

 * Copy to /usr/src/dd_usbfb-1.1
 * Enter that directory
 * ```make```
 * ```make install```

Now a kernel module ```dd_usbfb``` should be available. Plug in your USB screen - It should show a pink screen if the driver loaded succesfully. Now you can use the new framebuffer device e.g. ```/dev/fb2``` to display data from the console, eg. using fbi: ```fbi -d /dev/fb2 -a -noverbose <FILE>```.

Known issues (from original README.txt)
========

* PLEASE NOTE: The touch module is currently NOT being compiled! Adjust the Makefile if you need it.
* The touch driver can not operate correctly without calibration data stored in the ArtistaUSB device (calibtool is now supplied for the calibration)
* ```CONFIG\_FB\_DEFERRED\_IO=y``` is needed in the kernel configuration. This can not be explicitly set, enabling ```CONFIG\_HID\_PICOLCD=m``` is a workaround. The resulting module does not have to be installed, but the kernel has to be re-built with the configuration resulting in some bits being added to fb_info data structure.
* artistausb_ops_blank always reports device /dev/fb0 regardless of the actual device being claimed.
* Finding device names and sysfs files (esp. raw usb touch device) is not straight forward

Todo (from original README.txt)
========

* improve calibration access
* fix bugs ;-)
* more testing. Current test systems include:
  * v1.0:
    * 2.6.36 (ARM)
    * 2.6.32-5 (x86, Debian)
    * 2.6.39.4 (x86, vanilla)
    * 3.2.0-0 (amd64, Debian)
  * v1.1
    * 3.11.0-15 (amd64, Ubuntu 13.10)

I found a bug or have suggestion
========

The best way to report a bug or suggest something is to post an issue on GitHub. Try to make it simple. You can also contact me via email if you want to.
