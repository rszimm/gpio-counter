# Ne name of this Kernel Module
MODULE=gpio-counter
ccflags-y := -std=gnu99 -Wall -Wextra -Wno-declaration-after-statement -Wno-unused-parameter
 
KERNEL_SRC=/lib/modules/$(shell uname -r)/build/
 
obj-m += ${MODULE}.o
 
module_upload=${MODULE}.ko
 
all: ${MODULE}.ko
 
${MODULE}.ko: ${MODULE}.c
	make -C ${KERNEL_SRC} M=$(PWD) modules
 
clean:
	make -C ${KERNEL_SRC} M=$(PWD) clean
 
info:
	modinfo  ${module_upload}
