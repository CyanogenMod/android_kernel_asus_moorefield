css2600-mod-objs			+= css2600.o css2600-bus.o css2600-dma.o
obj-$(CONFIG_VIDEO_CSS2600)		+= css2600-mod.o
obj-$(CONFIG_VIDEO_CSS2600)		+= css2600-mmu.o
css2600-isys-mod-objs			+= css2600-isys.o css2600-isys-csi2.o \
					   css2600-isys-csi2-2401.o \
					   css2600-isys-isa.o \
					   css2600-isys-video.o \
					   css2600-isys-queue.o \
					   css2600-isys-subdev.o \
					   css2600-isys-tpg.o \
					   css2600-isys-tpg-2401.o
obj-$(CONFIG_VIDEO_CSS2600)		+= css2600-isys-mod.o
css2600-psys-mod-objs			+= css2600-psys.o
ifeq ($(CONFIG_COMPAT),y)
css2600-psys-mod-objs			+= css2600-psys-compat32.o
endif
obj-$(CONFIG_VIDEO_CSS2600)		+= css2600-psys-mod.o
obj-$(CONFIG_VIDEO_CSS2600)		+= css2600-buttress.o
obj-$(CONFIG_VIDEO_CSS2600_2401)	+= css2600-wrapper-2401.o
obj-$(CONFIG_VIDEO_CSS2600_2600)	+= css2600-wrapper-2600.o
ifdef CONFIG_VIDEO_CSS2600_2401
include $(srctree)/$(src)/Makefile.lib2401
else
include $(srctree)/$(src)/Makefile.lib2600
include $(srctree)/$(src)/Makefile.lib2600psys
endif
ccflags-y += -I$(src)/../../../../include/uapi/
