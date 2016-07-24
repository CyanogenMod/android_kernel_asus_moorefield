
# Makefile for regulator drivers.
#

CFLAGS_pmic_avp.o			:= -Werror

obj-$(CONFIG_REGULATOR_PMIC_AVP)	+= pmic_avp.o
obj-$(CONFIG_REGULATOR_DOLLAR_COVE) += pmic_dollar_cove.o

ccflags-$(CONFIG_REGULATOR_DEBUG)	+= -DDEBUG
