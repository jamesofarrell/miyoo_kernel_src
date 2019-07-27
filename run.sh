#!/bin/sh
#make ARCH=arm CROSS_COMPILE=arm-linux-gnueabi- miyoo_defconfig

rm -rf out
mkdir out/
mkdir out/boot/
mkdir out/rootfs/
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabi- -j4 zImage dtbs modules
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabi- INSTALL_MOD_PATH=out/rootfs modules_install -j2

#sudo ./sunxi-fel -p spiflash-write 1048576 arch/arm/boot/dts/suniv-f1c500s-miyoo.dtb && sudo ./sunxi-fel -p spiflash-write 1114112 arch/arm/boot/zImage

#cp arch/arm/boot/zImage /media/steward/boot/
#cp arch/arm/boot/dts/suniv-f1c500s-miyoo.dtb /media/steward/boot/
#sudo umount /media/steward/*
#sync

