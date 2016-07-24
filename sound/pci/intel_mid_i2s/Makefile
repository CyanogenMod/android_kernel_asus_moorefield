# SSP I2S audio drivers for intel Mid platforms
# Copyright (c) 2010, Intel Corporation.

# This program is free software; you can redistribute it and/or modify it
# under the terms and conditions of the GNU General Public License,
# version 2, as published by the Free Software Foundation.

# This program is distributed in the hope it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.

# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.

obj-$(CONFIG_SND_INTEL_MID_I2S)    +=  intel_mid_i2s.o

obj-$(CONFIG_SND_INTEL_ALSA_SSP) += snd-alsa-ssp.o

obj-$(CONFIG_SND_INTEL_ALSA_IFX) += snd-alsa-ifx.o

snd-alsa-ssp-objs := intel_alsa_ssp_snd_card.o intel_alsa_ssp_hw_interface.o

snd-alsa-ifx-objs := intel_alsa_ifx_snd_card.o intel_alsa_ifx_hw_interface.o

