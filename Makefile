FW_DIR	:= /lib/firmware/rtk_bt
MDL_DIR	:= /lib/modules/$(shell uname -r)
DRV_DIR	:= $(MDL_DIR)/kernel/drivers/bluetooth

ifneq ($(KERNELRELEASE),)

	obj-m := rtk_btusb.o

else
	PWD := $(shell pwd)
	KVER := $(shell uname -r)
	KDIR := /lib/modules/$(KVER)/build

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	rm -rf *.o *.mod.c *.mod.o *.ko *.symvers *.order *.a
endif

install:
	mkdir -p $(FW_DIR)
	cp -f rlt8723a_chip_b_cut_bt40_fw_asic_rom_patch-svn8511-0x0020342E-20121105-LINUX_USB.bin $(FW_DIR)/rtk8723a.bin
	cp -f rtk_btusb.ko $(DRV_DIR)/rtk_btusb.ko
	depmod -a $(MDL_DIR)
	echo "install rtk_btusb success!"

uninstall:
	rm -f $(DRV_DIR)/rtk_btusb.ko
	depmod -a $(MDL_DIR)
	rm -f $(FW_DIR)/rtk8723a.bin
	echo "uninstall rtk_btusb success!"
