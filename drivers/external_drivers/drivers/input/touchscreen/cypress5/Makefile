obj-$(CONFIG_CYPRESS_CYTTSP5_BUS) += cyttsp5_bus.o
obj-$(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5)	+= cyttsp5_core.o
ifdef CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5
obj-y += cyttsp5_platform.o
endif
obj-$(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_I2C)	+= cyttsp5_i2c.o
obj-$(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_SPI)	+= cyttsp5_spi.o
obj-$(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_MT_A)	+= cyttsp5_mt_a.o
cyttsp5_mt_a-y := cyttsp5_mta.o cyttsp5_mt_common.o
obj-$(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_BUTTON)	+= cyttsp5_btn.o
obj-$(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_PROXIMITY)	+= cyttsp5_proximity.o
obj-$(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICE_ACCESS)	+= cyttsp5_device_access.o
obj-$(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_LOADER)	+= cyttsp5_loader.o
obj-$(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEBUG_MODULE)	+= cyttsp5_debug.o
obj-$(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEVICETREE_SUPPORT)	+= cyttsp5_devtree.o
ifeq ($(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_DEBUG),y)
CFLAGS_cyttsp5_bus.o += -DDEBUG
CFLAGS_cyttsp5_core.o += -DDEBUG
CFLAGS_cyttsp5_i2c.o += -DDEBUG
CFLAGS_cyttsp5_spi.o += -DDEBUG
CFLAGS_cyttsp5_mta.o += -DDEBUG
CFLAGS_cyttsp5_mtb.o += -DDEBUG
CFLAGS_cyttsp5_mt_a.o += -DDEBUG
CFLAGS_cyttsp5_mt_b.o += -DDEBUG
CFLAGS_cyttsp5_mt_common.o += -DDEBUG
CFLAGS_cyttsp5_btn.o += -DDEBUG
CFLAGS_cyttsp5_proximity.o += -DDEBUG
CFLAGS_cyttsp5_device_access.o += -DDEBUG
CFLAGS_cyttsp5_loader.o += -DDEBUG
CFLAGS_cyttsp5_debug.o += -DDEBUG
CFLAGS_cyttsp5_devtree.o += -DDEBUG
CFLAGS_cyttsp5_platform.o += -DDEBUG
endif
ifeq ($(CONFIG_TOUCHSCREEN_CYPRESS_CYTTSP5_VDEBUG),y)
CFLAGS_cyttsp5_bus.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp5_core.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp5_i2c.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp5_spi.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp5_mta.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp5_mtb.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp5_mt_a.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp5_mt_b.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp5_mt_common.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp5_btn.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp5_proximity.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp5_device_access.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp5_loader.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp5_debug.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp5_devtree.o += -DVERBOSE_DEBUG
CFLAGS_cyttsp5_platform.o += -DVERBOSE_DEBUG
endif
