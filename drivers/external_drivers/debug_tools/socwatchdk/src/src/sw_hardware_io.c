/*

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2014 - 2015 Intel Corporation.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  Contact Information:
  SoC Watch Developer Team <socwatchdevelopers@intel.com>
  Intel Corporation,
  1906 Fox Drive,
  Champaign, IL 61820

  BSD LICENSE

  Copyright(c) 2014 - 2015 Intel Corporation.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/pci.h> // "pci_get_bus_and_slot"
#include <asm/msr.h>
#ifdef CONFIG_RPMSG_IPC
    #include <asm/intel_mid_rpmsg.h>
#endif // CONFIG_RPMSG_IPC

#include "sw_types.h"
#include "sw_structs.h"
#include "sw_defines.h"
#include "sw_hardware_io.h"
#include "sw_telem.h"

/*
 * Compile time constants.
 */
/*
 * Should we be doing 'direct' PCI reads and writes?
 * '1' ==> YES, call "pci_{read,write}_config_dword()" directly
 * '0' ==> NO, Use the "intel_mid_msgbus_{read32,write32}_raw()" API (defined in 'intel_mid_pcihelpers.c')
 */
#define DO_DIRECT_PCI_READ_WRITE 0
#if !DO_ANDROID || !defined(CONFIG_X86_WANT_INTEL_MID)
    /*
     * 'intel_mid_pcihelpers.h' is probably not present -- force
     * direct PCI calls in this case.
     */
    #undef DO_DIRECT_PCI_READ_WRITE
    #define DO_DIRECT_PCI_READ_WRITE  1
#endif
#if !DO_DIRECT_PCI_READ_WRITE
    #include <asm/intel_mid_pcihelpers.h>
#endif

#define SW_PCI_MSG_CTRL_REG 0x000000D0
#define SW_PCI_MSG_DATA_REG 0x000000D4

/*
 * Local data structures.
 */
/*
 * TODO: separate into H/W and S/W IO?
 */
typedef enum sw_io_type {
    SW_IO_MSR        = 0,
    SW_IO_IPC        = 1,
    SW_IO_MMIO       = 2,
    SW_IO_PCI        = 3,
    SW_IO_CONFIGDB   = 4,
    SW_IO_TRACE_ARGS = 5,
    SW_IO_WAKEUP     = 6,
    SW_IO_SOCPERF    = 7,
    SW_IO_PROC_NAME  = 8,
    SW_IO_IRQ_NAME   = 9,
    SW_IO_WAKELOCK   =10,
    SW_IO_TELEM      =11,
    SW_IO_MAX        =12
} sw_io_type_t;

/*
 * Function declarations.
 */
/*
 * Exported by the SOCPERF driver.
 */
extern void SOCPERF_Read_Data(void *data_buffer);

/*
 * Init functions.
 */
int sw_ipc_mmio_descriptor_init_func_i(struct sw_driver_io_descriptor *descriptor);

/*
 * Read functions.
 */
void sw_read_msr_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_read_ipc_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_read_mmio_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_read_pci_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_read_configdb_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_read_socperf_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);

/*
 * Write functions.
 */
void sw_write_msr_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_write_ipc_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_write_mmio_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_write_pci_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_write_configdb_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_write_trace_args_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_write_wakeup_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);
void sw_write_socperf_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes);

/*
 * Print functions.
 */
int sw_print_msr_io_descriptor(const struct sw_driver_io_descriptor *descriptor);

// Reset functions -- equal but opposite of init.

/*
 * Helper functions.
 */
u32 sw_platform_configdb_read32(u32 address);
u32 sw_platform_pci_read32(u32 bus, u32 device, u32 function, u32 ctrl_offset, u32 ctrl_value, u32 data_offset);

/*
 * Table of collector operations.
 */
static const struct sw_hw_ops s_hw_ops[] = {
    [SW_IO_MSR] = {
        .name = "MSR",
        .init = NULL,
        .read = &sw_read_msr_info_i,
        .write = &sw_write_msr_info_i,
        .print = &sw_print_msr_io_descriptor,
        .reset = NULL,
        .available = NULL
    },
    [SW_IO_IPC] = {
        .name = "IPC",
        .init = &sw_ipc_mmio_descriptor_init_func_i,
        .read = &sw_read_ipc_info_i,
        /* Other fields are don't care (will be set to NULL) */
    },
    [SW_IO_MMIO] = {
        .name = "MMIO",
        .init = &sw_ipc_mmio_descriptor_init_func_i,
        .read = &sw_read_mmio_info_i,
        .write = &sw_write_mmio_info_i,
        /* Other fields are don't care (will be set to NULL) */
    },
    [SW_IO_PCI] = {
        .name = "PCI",
        .read = &sw_read_pci_info_i,
        /* Other fields are don't care (will be set to NULL) */
    },
    [SW_IO_CONFIGDB] = {
        .name = "CONFIGDB",
        .read = &sw_read_configdb_info_i,
        /* Other fields are don't care (will be set to NULL) */
    },
    [SW_IO_WAKEUP] = {
        .name = "WAKEUP",
        /* Other fields are don't care (will be set to NULL) */
    },
    [SW_IO_SOCPERF] = {
        .name = "SOCPERF",
        .read = &sw_read_socperf_info_i,
        /* Other fields are don't care (will be set to NULL) */
    },
    [SW_IO_PROC_NAME] = {
        .name = "PROC-NAME",
        /* Other fields are don't care (will be set to NULL) */
    },
    [SW_IO_IRQ_NAME] = {
        .name = "IRQ-NAME",
        /* Other fields are don't care (will be set to NULL) */
    },
    [SW_IO_WAKELOCK] = {
        .name = "WAKELOCK",
        /* Other fields are don't care (will be set to NULL) */
    },
    [SW_IO_TELEM] = {
        .name = "TELEM",
        .init = &sw_telem_init_func,
        .read = &sw_read_telem_info,
        .reset = &sw_reset_telem,
        .available = &sw_telem_available,
        /* Other fields are don't care (will be set to NULL) */
    },
    [SW_IO_MAX] = {
        .name = NULL,
        /* Other fields are don't care (will be set to NULL) */
    }
};

#define NUM_HW_OPS SW_ARRAY_SIZE(s_hw_ops)
#define FOR_EACH_HW_OP(idx, op) for (idx=0; idx<NUM_HW_OPS && (op=&s_hw_ops[idx]); ++idx)

/*
 * Function definitions.
 */
int sw_ipc_mmio_descriptor_init_func_i(struct sw_driver_io_descriptor *descriptor)
{
    // Perform any required 'io_remap' calls here
    struct sw_driver_ipc_mmio_io_descriptor *__ipc_mmio = NULL;
    if (!descriptor) { // Should NEVER happen
        return -PW_ERROR;
    }
    if (descriptor->collection_type == SW_IO_IPC) {
        __ipc_mmio = &descriptor->ipc_descriptor;
    } else {
        __ipc_mmio = &descriptor->mmio_descriptor;
    }
    pw_pr_debug("cmd = %u, sub-cmd = %u, data_addr = 0x%llx\n", __ipc_mmio->command, __ipc_mmio->sub_command, __ipc_mmio->data_address);
    /*
    if (__ipc_mmio->command || __ipc_mmio->sub_command) {
        __ipc_mmio->ipc_command = ((pw_u32_t)__ipc_mmio->sub_command << 12) | (pw_u32_t)__ipc_mmio->command;
    }
    */
    if (__ipc_mmio->data_address) {
        __ipc_mmio->data_remapped_address = (pw_u64_t)(unsigned long)ioremap_nocache((unsigned long)__ipc_mmio->data_address, descriptor->counter_size_in_bytes);
        if ((void *)(unsigned long)__ipc_mmio->data_remapped_address == NULL) {
            return -PW_ERROR;
        }
    }
    return PW_SUCCESS;
}

void sw_read_msr_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptors, u16 counter_size_in_bytes)
{
    u64 address = descriptors->msr_descriptor.address;
    u32 l=0, h=0;

    if (likely(cpu == RAW_CPU())) {
        rdmsr((unsigned long)address, l, h);
    } else {
        rdmsr_on_cpu(cpu, (unsigned long)address, &l, &h);
    }
    switch (counter_size_in_bytes) {
    case 4:
        *((u32 *)dst_vals) = l;
        break;
    case 8:
        *((u64 *)dst_vals) = ((u64)h << 32) | l;
        break;
    default:
        break;
    }
    return;
}

#ifdef CONFIG_RPMSG_IPC
#    define SW_DO_IPC(cmd, sub_cmd) rpmsg_send_generic_simple_command(cmd, sub_cmd)
#else
#    define SW_DO_IPC(cmd, sub_cmd) (-ENODEV)
#endif // CONFIG_RPMSG_IPC

void sw_read_ipc_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptors, u16 counter_size_in_bytes)
{
    u16 cmd = descriptors->ipc_descriptor.command, sub_cmd = descriptors->ipc_descriptor.sub_command;
    unsigned long remapped_address = (unsigned long)descriptors->ipc_descriptor.data_remapped_address;

    if (cmd || sub_cmd) {
        pw_pr_debug("EXECUTING IPC Cmd = %u, %u\n", cmd, sub_cmd);
        if (SW_DO_IPC(cmd, sub_cmd)) {
            pw_pr_error("ERROR running IPC command(s)\n");
            return;
        }
    }

    if (remapped_address) {
        // memcpy(&value, (void *)remapped_address, counter_size_in_bytes);
        pw_pr_debug("COPYING MMIO size %u\n", counter_size_in_bytes);
        memcpy(dst_vals, (void *)remapped_address, counter_size_in_bytes);
    }
    pw_pr_debug("Value = %llu\n", *((u64 *)dst_vals));
}
void sw_read_mmio_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptors, u16 counter_size_in_bytes)
{
    unsigned long remapped_address = (unsigned long)descriptors->mmio_descriptor.data_remapped_address;
    if (remapped_address) {
        memcpy_fromio(dst_vals, (void *)remapped_address, counter_size_in_bytes);
    }
    pw_pr_debug("Value = %llu\n", *((u64 *)dst_vals));
}
void sw_read_pci_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptors, u16 counter_size_in_bytes)
{
    u32 bus = descriptors->pci_descriptor.bus, device = descriptors->pci_descriptor.device;
    u32 function = descriptors->pci_descriptor.function, offset = descriptors->pci_descriptor.offset;
    u32 data = sw_platform_pci_read32(bus, device, function, 0 /* CTRL-OFFSET */, 0 /* CTRL-DATA, don't care */, offset /* DATA-OFFSET */);
    /*
     * 'counter_size_in_bytes' is ignored, for now.
     */
    *((u32 *)dst_vals) = data;
    return;
}
void sw_read_configdb_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptors, u16 counter_size_in_bytes)
{
    {
        pw_u32_t address = descriptors->configdb_descriptor.address;
        u32 data = sw_platform_configdb_read32(address);
        pw_pr_debug( "ADDRESS = 0x%x, CPU = %d, dst_vals = %p, counter size = %u, data = %u\n", address, cpu, dst_vals, counter_size_in_bytes, data);
        /*
         * 'counter_size_in_bytes' is ignored, for now.
         */
        *((u32 *)dst_vals) = data;
    }
    return;
}
void sw_read_socperf_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptors, u16 counter_size_in_bytes)
{
#if DO_SOCPERF
    u64 *socperf_buffer = (u64 *)dst_vals;

    memset(socperf_buffer, 0, counter_size_in_bytes);
    SOCPERF_Read_Data(socperf_buffer);
#endif // DO_SOCPERF
    return;
}


/**
 * sw_platform_configdb_read32 - for reading PCI space through config registers
 *                               of the platform.
 * @address: An address in the PCI space
 *
 * Returns: the value read from address.
 */
u32 sw_platform_configdb_read32(u32 address)
{
    u32 read_value = 0;
#if DO_DIRECT_PCI_READ_WRITE
    return sw_platform_pci_read32(0/*bus*/, 0/*device*/, 0/*function*/, SW_PCI_MSG_CTRL_REG/*ctrl-offset*/, address/*ctrl-value*/, SW_PCI_MSG_DATA_REG/*data-offset*/);
#else // !DO_DIRECT_PCI_READ_WRITE
    read_value = intel_mid_msgbus_read32_raw(address);
    pw_pr_debug("address = %u, value = %u\n", address, read_value);
#endif // if DO_DIRECT_PCI_READ_WRITE
    return read_value;
}

u32 sw_platform_pci_read32(u32 bus, u32 device, u32 function, u32 write_offset, u32 write_value, u32 read_offset)
{
    u32 read_value = 0;
    struct pci_dev *pci_root = pci_get_bus_and_slot(bus, PCI_DEVFN(device, function)); // 0, PCI_DEVFN(0, 0));
    if (!pci_root) {
        return 0; /* Application will verify the data */
    }
    if (write_offset) {
        pci_write_config_dword(pci_root, write_offset, write_value); // SW_PCI_MSG_CTRL_REG, address);
    }
    pci_read_config_dword(pci_root, read_offset, &read_value); // SW_PCI_MSG_DATA_REG, &read_value);
    return read_value;
}

void sw_write_msr_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes)
{
    u64 write_value = descriptor->write_value;
    u64 address = descriptor->msr_descriptor.address;
    pw_pr_debug("ADDRESS = 0x%llx, CPU = %d, counter size = %u, value = %llu\n", address, cpu, counter_size_in_bytes, write_value);
    if (likely(cpu == RAW_CPU())) {
        wrmsrl((unsigned long)address, write_value);
    } else {
        u32 l = write_value & 0xffffffff, h = (write_value >> 32) & 0xffffffff;
        wrmsr_on_cpu(cpu, (u32)address, l, h);
    }
    return;
};

void sw_write_mmio_info_i(char *dst_vals, int cpu, const struct sw_driver_io_descriptor *descriptor, u16 counter_size_in_bytes)
{
    unsigned long remapped_address = (unsigned long)descriptor->mmio_descriptor.data_remapped_address;
    u64 write_value = descriptor->write_value;

    if (remapped_address) {
        memcpy_toio((void *)remapped_address, &write_value, counter_size_in_bytes);
    }
    pw_pr_debug("Value = %llu\n", *((u64 *)dst_vals));
};

int sw_print_msr_io_descriptor(const struct sw_driver_io_descriptor *descriptor)
{
    if (!descriptor) {
        return -PW_ERROR;
    }
    pw_pr_debug("MSR address = 0x%llx\n", descriptor->msr_descriptor.address);
    return PW_SUCCESS;
}

int sw_get_hw_op_id(const struct sw_hw_ops *ops)
{
    int id = -1;
    if (ops) {
        id = (int)(ops - s_hw_ops);
    }
    return sw_is_valid_hw_op_id(id) ? id : -1;
}

const struct sw_hw_ops *sw_get_hw_ops_for(int id)
{
    if (!sw_is_valid_hw_op_id(id)) {
        return NULL;
    }
    return &s_hw_ops[id];
}

bool sw_is_valid_hw_op_id(int id)
{
    if (id < SW_IO_MSR || id >= SW_IO_MAX || s_hw_ops[id].name == NULL) {
        return false;
    }
    return true;
}

const char *sw_get_hw_op_abstract_name(const struct sw_hw_ops *op)
{
    if (op) {
        return op->name;
    }
    return NULL;
}

int sw_for_each_hw_op(int (*func)(const struct sw_hw_ops *op, void *priv), void *priv, bool return_on_error)
{
    int retVal = PW_SUCCESS;
    size_t idx=0;
    const struct sw_hw_ops *op = NULL;
    if (func) {
        FOR_EACH_HW_OP(idx, op) {
            if ((*func)(op, priv)) {
                retVal = -PW_ERROR;
                if (return_on_error) {
                    break;
                }
            }
        }
    }
    return retVal;
}
