obj-m := mmsensdrv.o

SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

out-of-tree:
	KERNEL_SRC=/home/strahinja/work/qemu/blog/testing/linux/build_vexpress
	$(MAKE) ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabihf- -C $(KERNEL_SRC) M=$(SRC)

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c *.mod
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
