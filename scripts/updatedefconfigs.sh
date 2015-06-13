#!/bin/bash
#
# This script is meant to update all defconfigs when any Kconfig changes.
# Just go to kernel root directory and execute:
# $ scripts/updatedefconfigs.sh
#
# For each ${ARCH}_XXX_defconfig inside arch/x86/configs it will ask to
# run either make oldconfig, make menuconfig or make xconfig.
# In the end all defconfigs will be updated and ready for a patch.


for arch in x86_64 i386
do
	ALLDEFCONFIGS="`ls arch/x86/configs/${arch}_*_defconfig`"
	for conf in $ALLDEFCONFIGS; do
		echo "Updating $conf (O=oldconfig, m=menuconfig, x=xconfig): [O/m/x default=O]?"
		read -s -n1 ANSWER < /dev/tty
		cp $conf .config
		case "$ANSWER" in
			[mM] )
				make ARCH=${arch} menuconfig
			;;
			[xX] )
				make ARCH=${arch} xconfig
			;;
			 *)
				make ARCH=${arch} oldconfig
			;;
		esac
		cp .config $conf
	done;
done;

make mrproper

