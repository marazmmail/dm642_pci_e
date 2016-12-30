CONFIG_MODULE_SIG=n
dm642_pci-objs := pci.o mem.o proc.o task.o mmap.o dm642.o
obj-m	:= dm642_pci.o

#EXTRA_CFLAGS += -ffreestanding -O2 -Wextra -Wall -fno-exceptions -fno-rtti
#CCFLAGS += -DDEBUG -g

PWD       := $(shell pwd)

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

