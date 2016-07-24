ccflags-y += -Idrivers/external_drivers/camera/drivers/media/i2c/LaserSensor/Olivia_msm/laura/inc
ccflags-y += -Idrivers/external_drivers/camera/drivers/media/i2c/LaserSensor/Olivia_msm/show_lib/inc

olivia_lasersensor-objs = laura/src/func/laura_shipping_func.o \
                          laura/src/func/laura_factory_func.o \
                          laura/src/dev/olivia_dev.o \
                          laura/src/msm_laser_focus_hepta.o \
                          laura/src/i2c/laura_i2c.o \
                          laura/src/debug/laura_debug.o \
                          laura/src/crc/crc8.o \
                          laura/src/interface/laura_interface.o \
                          show_lib/src/log/show_log.o \
                          show_lib/src/cci_i2c/laser_focus_i2c.o \
                          show_lib/src/msm_laser_focus/msm_laser_focus.o \
                          show_lib/src/gpio/show_gpio.o \
                          show_lib/src/debug/show_debug.o \
                          show_lib/src/mutex/show_mutex.o \
                          show_lib/src/parser/show_parser.o \
                          show_lib/src/timer/show_timer.o \
                          show_lib/src/sysfs/show_procfs.o \
                          show_lib/src/sysfs/show_sysfs.o \
                          show_lib/src/verify/show_verify.o \

obj-$(CONFIG_OLIVIA_LASERSENSOR) += olivia_lasersensor.o
