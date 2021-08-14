source1 = pcd_device_setup.o 
source2 = pcd_platform_driver.o 
obj-m:=${source1} ${source2}
HOST_KERN_DIR = /lib/modules/$(shell uname -r)/build/
all:
	make -C /lib/modules/4.14.108-ti-r134/build M=$(shell pwd) modules
clean:
	make -C $(HOST_KERN_DIR) M=$(shell pwd) clean
