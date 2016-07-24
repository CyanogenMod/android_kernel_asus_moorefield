#!/bin/bash

target=$1

if [ "$target" = "32bit" ];
then
    echo "32bit"
elif [ "$target" = "64bit" ];
then
        echo "64bit"
    else
          inval="false"
     while [ "$inval" = "false" ]; do
        read -p "Plese enter '32bit' or '64bit' " target
        if [ "$target" = "32bit" ] || [ "$target" = "64bit" ];
        then
          inval="true"
        fi
      done

fi

if [ -z $TARGET_PRODUCT ]; then
if [ -z $WW_NUMBER ]; then
    inval="false"
    while [ $inval = "false" ]; do
        read -p "Enter WW_NUMBER " WW_NUMBER
        if [ -z $WW_NUMBER ] ; then
            inval="false"
        else
            export WW_NUMBER
            inval="true"
        fi
    done

fi

cd ../../../ww$WW_NUMBER
source build/envsetup.sh
lunch
cd -
fi

HARDWARE=${TARGET_PRODUCT%_64}
export $HARDWARE

if [ "$target" = "32bit" ];
then
    target=i386
    objcopy -B i386 -I binary -O elf32-i386 vidt_sign.bin vidt_sign.o 
fi

if [ "$target" = "64bit" ];
then
    target=x86_64
# enable it after signing is enabled
    objcopy -B i386 -I binary -O elf64-x86-64 vidt_sign.bin vidt_sign.o 
fi

#export ANDROID_LINUX_PATH=~/Merr_WW16/kernel/
#create signature
make ARCH=$target HARDWARE=$HARDWARE
echo "Signing vidt driver with kernel keys"
    `perl ${ANDROID_LINUX_PATH}/kernel/scripts/sign-file sha256 ${ANDROID_LINUX_PATH}/../out/target/product/$HARDWARE/linux/kernel/signing_key.priv ${ANDROID_LINUX_PATH}/../out/target/product/$HARDWARE/linux/kernel/signing_key.x509 vidt_driver.ko`

