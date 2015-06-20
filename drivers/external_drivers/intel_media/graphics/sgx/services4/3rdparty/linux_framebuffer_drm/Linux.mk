#
# Copyright (C) Imagination Technologies Ltd. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify it
# under the terms and conditions of the GNU General Public License,
# version 2, as published by the Free Software Foundation.
#
# This program is distributed in the hope it will be useful but, except
# as otherwise stated in writing, without any warranty; without even the
# implied warranty of merchantability or fitness for a particular purpose.
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
#
# The full GNU General Public License is included in this distribution in
# the file called "COPYING".
#
# Contact Information:
# Imagination Technologies Ltd. <gpl-support@imgtec.com>
# Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK
#
#

modules := medfield_drm moorestown_drm poulsbo_drm

medfield_drm_type := kernel_module
medfield_drm_target := medfield_drm.ko
medfield_drm_makefile := $(THIS_DIR)/Medfield.mk

moorestown_drm_type := kernel_module
moorestown_drm_target := moorestown_drm.ko
moorestown_drm_makefile := $(THIS_DIR)/Moorestown.mk

poulsbo_drm_type := kernel_module
poulsbo_drm_target := poulsbo_drm.ko
poulsbo_drm_makefile := $(THIS_DIR)/Poulsbo.mk
