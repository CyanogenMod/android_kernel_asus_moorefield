LDFLAGS := -T $(ANDROID_BUILD_TOP)/linux/modules/sldriver/elf_i386.lds
EXTRA_CFLAGS :=-fno-pic
TARGET_MODULE += vidt_driver

ifeq ($(ARCH) , x86_64)
$(TARGET_MODULE)-objs += vmm_hsym.o kern_ta.o vidt.o isr64.o
endif

obj-$(CONFIG_VIDT) += $(TARGET_MODULE).o

all:
	@echo Arvind...
	@cp vidt_sign.bin $(ANDROID_PRODUCT_OUT)/root/lib/modules/

default:
	@cp vidt_sign.bin $(ANDROID_PRODUCT_OUT)/root/lib/modules/
	$(MAKE) -C $(KERNEL_SRC_DIR) CROSS_COMPILE=$(CROSS_COMPILE) M=$(PWD) modules PWD=$(PWD)
clean:
	@rm -f *.o *.ko
