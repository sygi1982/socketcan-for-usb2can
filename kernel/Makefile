MODULE = ux2can

obj-m := $(MODULE).o

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
default:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules

# clean locally
clean:
	rm -f *.ko *.o Module.symvers modules.order
