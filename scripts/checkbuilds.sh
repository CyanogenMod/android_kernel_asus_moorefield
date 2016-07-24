#!/bin/bash

ALLDEFCONFIGS="`ls arch/x86/configs/i386_*_defconfig`"
NJOBS=`cat /proc/cpuinfo | grep processor | wc -l`
OUTPUT_DIR=".tmp_kernel_build"

usage()
{
	echo "Help not implemented yet..."
	echo
}

failed()
{
	make mrproper
	rm -fr $OUTPUT_DIR
	echo
	echo "Build failed: $1"
	exit 1
}

ALL=1

while getopts "i" OPTION; do
	case $OPTION in
	i)
		ALL=0
		;;
	?)
		usage
		exit 1
		;;
	esac
done

rm -fr $OUTPUT_DIR
make mrproper
mkdir $OUTPUT_DIR
for conf in $ALLDEFCONFIGS; do
	echo -n "Check $conf: "
	if [ "$ALL" != "1" ]; then
		echo "[Y/a/n] "
		read -s -n1 ANSWER < /dev/tty
	else
		echo
	fi
	cp $conf $OUTPUT_DIR/.config

	if [ "$ANSWER" = "a" ]; then
		ALL=1
	elif [ "$ANSWER" = "n" ]; then
		continue
	fi
	echo
	echo "Building..."
	echo
	make ARCH=i386 O=$OUTPUT_DIR oldconfig
	make ARCH=i386 O=$OUTPUT_DIR clean
	make ARCH=i386 O=$OUTPUT_DIR -j$NJOBS bzImage modules || failed $conf
done;

make mrproper
rm -fr $OUTPUT_DIR

echo
echo "All builds successfully done!"

