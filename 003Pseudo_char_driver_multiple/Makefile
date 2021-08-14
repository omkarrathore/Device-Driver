obj-m:=pcd_n.o
HOST_KERN_DIR = /lib/modules/$(shell uname -r)/build/
all:
	make -C /lib/modules/4.14.108-ti-r134/build M=$(shell pwd) modules
clean:
	make -C $(HOST_KERN_DIR) M=$(PWD) clean
