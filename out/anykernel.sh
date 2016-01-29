# AnyKernel 2.0 Ramdisk Mod Script 
# osm0sis @ xda-developers

## AnyKernel setup
# EDIFY properties
kernel.string=Zenfone 2 AnyKernel Script
do.devicecheck=0
do.initd=1
do.modules=0
do.cleanup=0
device.name1=Z00A
device.name2=z008
device.name3=
device.name4=
device.name5=

# shell variables
block=/dev/block/by-name/boot;

## end setup


## AnyKernel methods (DO NOT CHANGE)
# set up extracted files and directories
ramdisk=/tmp/anykernel/ramdisk;
bin=/tmp/anykernel/tools;
split_img=/tmp/anykernel/split_img;
patch=/tmp/anykernel/patch;

chmod -R 777 $bin;
mkdir -p $ramdisk $split_img;
#cd /tmp/anykernel;

OUTFD=`ps | grep -v "grep" | grep -oE "update(.*)" | cut -d" " -f3`;
ui_print() { echo "ui_print $1" >&$OUTFD; echo "ui_print" >&$OUTFD; }

# dump boot and extract ramdisk
dump_boot() {
  dd if=$block of=/tmp/anykernel/boot.img;
  /tmp/anykernel/tools/unmkbootimg /tmp/anykernel/boot.img;
   #/tmp/anykernel/tools/unmkbootimg /tmp/anykernel/boot.img;
  if [ $? != 0 ]; then
    ui_print "Dumping/unpacking image failed. Aborting...";
    echo "Unpack Failed" > /tmp/anykernel/exitcode; exit;
  fi;
}

# repack ramdisk then build and write image
write_boot() {
  /tmp/anykernel/tools/mkbootimg --kernel /tmp/anykernel/bzImage --ramdisk /tmp/anykernel/initramfs.cpio.gz --cmdline "init=/init pci=noearly loglevel=0 vmalloc=256M androidboot.hardware=mofd_v1 watchdog.watchdog_thresh=60 androidboot.spid=xxxx:xxxx:xxxx:xxxx:xxxx:xxxx androidboot.serialno=01234567890123456789 snd_pcm.maximum_substreams=8 ip=50.0.0.2:50.0.0.1::255.255.255.0::usb0:on debug_locks=0 bootboost=1 androidboot.selinux=permissive" --base 0x10000000 --pagesize 2048 --ramdisk_offset 0x01000000 --tags_offset 0x00000100 --second /tmp/anykernel/second.gz -o /tmp/anykernel/boot-new.img;

  if [ $? != 0 -o `wc -c < /tmp/anykernel/boot-new.img` -gt `wc -c < /tmp/anykernel/boot.img` ]; then
    ui_print "Repacking image failed. Aborting...";
    echo "Repack Failed" > /tmp/anykernel/exitcode; exit;
  fi;
  dd if=/tmp/anykernel/boot-new.img of=$block;
}

## end methods

## AnyKernel install
dump_boot;

write_boot;

## end install

