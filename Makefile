ifndef KERNEL_VERSION
	KERNEL_VERSION=$(shell uname -r)
endif

ifndef KERNEL_DIR
	KERNEL_DIR=/usr/src/linux-headers-$(KERNEL_VERSION)
endif

TARGET = dd_usbfb.ko

obj-m += dd_usbfb.o

all: release

release:
	make -C $(KERNEL_DIR) M=`pwd` $(MAKE_ENV) \
		EXTRA_CFLAGS="$(EXTRA_CFLAGS)" modules

.PHONY: clean install

clean:
	-rm -rf *.o *mod* *.ko .cmem* .tmp* .*.cmd Module.symvers

install:
	-rmmod dd_usbfb
	dkms add -m dd_usbfb -v 1.1
	dkms build -m dd_usbfb -v 1.1
	dkms install -m dd_usbfb -v 1.1

uninstall:
	-rmmod dd_usbfb
	dkms remove -m dd_usbfb/1.1 --all

