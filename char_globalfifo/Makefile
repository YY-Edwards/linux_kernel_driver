KVERS = $(shell uname -r)

#Kernel modules
obj-m +=chardriver_globalfifo.o
chardriver_globalfifo-objs := globalfifo.o
#Specify flags for the module compilation.
EXTRA_CFLAGS=-g -o0

bulid: kernel_modules

kernel_modules:

	make -C /lib/modules/$(KVERS)/build M=$(CURDIR) modules
	
clean:
 
	make -C /lib/modules/$(KVERS)/build M=$(CURDIR) clean