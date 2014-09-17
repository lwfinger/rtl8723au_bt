	8723A_SRC_DIR = 8723A
	8723B_SRC_DIR = 8723B
	8821A_SRC_DIR = 8821A
	8761A_SRC_DIR = 8761A

	SRC_DIR	:= .
	MDL_DIR	:= /lib/modules/$(shell uname -r)
	DRV_DIR	:= $(MDL_DIR)/kernel/drivers/bluetooth
	

ifneq ($(KERNELRELEASE),)
	obj-m := rtk_btusb.o

else
	PWD := $(shell pwd)
	KVER := $(shell uname -r)
	KDIR := /lib/modules/$(KVER)/build

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules -s

clean:
	rm -rf *.o *.mod.c *.mod.o *.ko *.symvers *.order *.a

endif

##########################################
install:	
	make -C $(8723A_SRC_DIR) -s
	make -C $(8723B_SRC_DIR) -s
	make -C $(8821A_SRC_DIR) -s
	make -C $(8761A_SRC_DIR) -s

	- rmmod btusb
	- mv -n $(DRV_DIR)/btusb.ko $(DRV_DIR)/btusb_bak
	- rmmod rtk_btusb
	make -C $(SRC_DIR) -s
	cp -f $(SRC_DIR)/rtk_btusb.ko $(DRV_DIR)/rtk_btusb.ko
	depmod -a $(MDL_DIR)
	@echo "install rtk_btusb success!"

##########################################

uninstall:	
	make -C $(8723A_SRC_DIR) clean -s
	make -C $(8723B_SRC_DIR) clean -s
	make -C $(8821A_SRC_DIR) clean -s
	make -C $(8761A_SRC_DIR) clean -s

	- mv -n $(DRV_DIR)/btusb_bak $(DRV_DIR)/btusb.ko 
	- rmmod rtk_btusb
	rm -f $(DRV_DIR)/rtk_btusb.ko
	depmod -a $(MDL_DIR)
	echo "uninstall rtk_btusb success!"
##########################################

