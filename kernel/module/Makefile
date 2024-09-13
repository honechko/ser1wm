obj-m := ser1wm.o

KDIR ?= /lib/modules/`uname -r`/build

ser1wm.ko:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

load: ser1wm.ko
	modprobe wire && insmod ./ser1wm.ko

