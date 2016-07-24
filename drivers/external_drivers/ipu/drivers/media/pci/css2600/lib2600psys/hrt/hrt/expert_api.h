#ifndef _HRT_EXPERT_API_H_
#define _HRT_EXPERT_API_H_

#include "api.h"

/* Expert hrt api.
   This api provides a stable interface to functions that do not exist or that
   are internal in the standard hrt api, intended for verification purposes:
   - get values from status and control memories
   - access to individual bits in status and control memory
   - get cell memory sizes
   - slave_port loads and stores
   - pass requests from the master host / cell application to a device implementation
 */

#define _hrt_sc_get_bits(cell, reg, lsb, num) \
  _hrt_get_bits(_hrt_sc_get_reg(cell, reg), lsb, num)

/* These are equivalent to their counterparts in stat_ctrl.h */

#define hrtx_ctl_get_break(cell) hrt_ctl_get_break(cell)
#define hrt_ctl_get_break(cell) \
  _hrt_sc_get_bit(cell, _hrt_cell_break_flag_register(cell), _hrt_cell_break_flag_bit(cell))
#define hrtx_ctl_get_irq_broken(cell) hrt_ctl_get_irq_broken(cell)
#define hrt_ctl_get_irq_broken(cell) \
  _hrt_sc_get_bit(cell, _hrt_cell_broken_irq_mask_flag_register(cell), _hrt_cell_broken_irq_mask_flag_bit(cell))
#define hrtx_ctl_get_irq_ready(cell) hrt_ctl_get_irq_ready(cell)
#define hrt_ctl_get_irq_ready(cell) \
  _hrt_sc_get_bit(cell, _hrt_cell_ready_irq_mask_flag_register(cell), _hrt_cell_ready_irq_mask_flag_bit(cell))
#define hrtx_ctl_get_irq_sleeping(cell) hrt_ctl_get_irq_sleeping(cell)
#define hrt_ctl_get_irq_sleeping(cell) \
  _hrt_sc_get_bit(cell, _hrt_cell_sleeping_irq_mask_flag_register(cell), _hrt_cell_sleeping_irq_mask_flag_bit(cell))
#define hrtx_ctl_get_arbiter_period(cell, arb) \
  _hrt_sc_get_bits(cell, \
                   _hrt_cell_arbiter_period_register(cell, arb), \
                   _hrt_cell_arbiter_period_lsb(cell, arb), \
                   _hrt_cell_arbiter_period_num_bits(cell, arb))
#define hrtx_ctl_get_arbiter_contender_bandwidth(cell, cont) \
  _hrt_sc_get_bits(cell, \
                   _hrt_cell_arbiter_contender_bandwidth_register(cell, cont), \
                   _hrt_cell_arbiter_contender_bandwidth_lsb(cell, cont), \
                   _hrt_cell_arbiter_contender_bandwidth_num_bits(cell, cont))
#define hrtx_ctl_get_start_address(cell) hrt_ctl_get_start_address(cell)
#define hrt_ctl_get_start_address(cell) \
  _hrt_sc_get_reg(cell, _hrt_cell_start_address_register(cell))
#define hrtx_ctl_get_break_address(cell) hrt_ctl_get_break_address(cell)
#define hrt_ctl_get_break_address(cell) \
  _hrt_sc_get_reg(cell, _hrt_cell_break_address_register(cell))
#define hrtx_ctl_get_single_step(cell) hrt_ctl_get_single_step(cell)
#define hrt_ctl_get_single_step(cell) \
  _hrt_sc_get_bit(cell, _hrt_cell_debug_step_flag_register(cell), _hrt_cell_debug_step_flag_bit(cell))
#define hrtx_ctl_get_ext_base_address(cell, mt_int, segment) \
  _hrt_sc_get_reg(cell, _hrt_cell_base_address_lowest_register(cell, mt_int) + 3*segment)
#define hrtx_ctl_get_ext_base_info(cell, mt_int, segment) \
  _hrt_sc_get_reg(cell, _hrt_cell_base_address_lowest_register(cell, mt_int) + 3*segment+1)
#define hrtx_ctl_get_ext_base_info_override(cell, mt_int, segment) \
  _hrt_sc_get_reg(cell, _hrt_cell_base_address_lowest_register(cell, mt_int) + 3*segment+2)
#define hrtx_ctl_get_burst_length(cell, mt_int) \
  _hrt_sc_get_bits(cell, \
                   _hrt_cell_burst_size_register(cell, mt_int), \
                   _hrt_cell_burst_size_lsb(cell, mt_int), \
                   _hrt_cell_burst_size_num_bits(cell, mt_int))
#define hrtx_ctl_get_burst_interval(cell, mt_int) \
  _hrt_sc_get_bits(cell, \
                   _hrt_cell_burst_timeout_register(cell, mt_int), \
                   _hrt_cell_burst_timeout_lsb(cell, mt_int), \
                   _hrt_cell_burst_timeout_num_bits(cell, mt_int))

/* Access to invidual bits and registers in status and control memory: */
#define hrtx_stat_ctrl_slave_port(cell)                 _hrt_sc_slave(cell)
#define hrtx_stat_ctrl_set_register(cell,reg,val)       _hrt_sc_set_reg(cell,reg,val)
#define hrtx_stat_ctrl_get_register(cell,reg)           _hrt_sc_get_reg(cell,reg)
#define hrtx_stat_ctrl_set_status_bit(cell,reg,bit,val) _hrt_sc_set_bit(cell,reg,bit,val)
#define hrtx_stat_ctrl_get_status_bit(cell,reg,bit)     _hrt_sc_get_bit(cell, reg, bit)

/* Memory size as seen from the cell (from the LSU that is) */
#define hrtx_cell_mem_size(cell,mem) _hrt_cell_mem_size(cell,mem)

/* Store 32bit words to slave ports */
#define hrtx_slave_port_load_32(slave_port, addr)        _hrt_slave_port_load_32(slave_port, addr)
#define hrtx_slave_port_store_32(slave_port, addr, data) _hrt_slave_port_store_32(slave_port, addr, data)

/* The functions below are used to access hrt information using integer identifiers,
 * this is needed to use hrt from code where the system constants are not know yet 
 * (fpga debug library for instance) 
 */

extern int hrtx_cellid_prog_mem_width(hive_cell_id cell);
extern hive_address hrtx_cellid_prog_mem_base(hive_cell_id cell);
extern hive_address hrtx_cellid_view_table_base(hive_cell_id cell);
extern int hrtx_cellid_num_views(hive_cell_id cell);
extern int hrtx_cellid_int_size(hive_cell_id cell);
extern hive_mem_id hrtx_cellid_default_mem(hive_cell_id cell);
extern hive_port_id hrtx_cellid_prog_mem_slave_port(hive_cell_id cell);
extern hive_port_id hrtx_cellid_slave_port(hive_cell_id cell, hive_port_id cell_port);

extern void hrtx_slave_portid_store(hive_port_id port, hive_address addr, const void *data, hive_uint bytes);
extern void hrtx_slave_portid_load(hive_port_id port, hive_address addr, void *data, hive_uint bytes);
static inline void hrtx_slave_portid_store_8(hive_port_id port, hive_address addr, hive_uint8 data)
{
  hrtx_slave_portid_store(port, addr, &data, sizeof(data));
}

static inline void hrtx_slave_portid_store_16(hive_port_id port, hive_address addr, hive_uint16 data)
{
  hrtx_slave_portid_store(port, addr, &data, sizeof(data));
}

static inline void hrtx_slave_portid_store_32(hive_port_id port, hive_address addr, hive_uint32 data)
{
  hrtx_slave_portid_store(port, addr, &data, sizeof(data));
}

static inline hive_uint8 hrtx_slave_portid_load_8(hive_port_id port, hive_address addr)
{
  hive_uint8 data;
  hrtx_slave_portid_load(port, addr, &data, sizeof(data));
  return data;
}

extern hive_uint hrtx_ctlid_program_counter(hive_cell_id cell);
static inline hive_uint16 hrtx_slave_portid_load_16(hive_port_id port, hive_address addr)
{
  hive_uint16 data;
  hrtx_slave_portid_load(port, addr, &data, sizeof(data));
  return data;
}

extern hive_uint hrtx_ctlid_program_counter(hive_cell_id cell);
static inline hive_uint32 hrtx_slave_portid_load_32(hive_port_id port, hive_address addr)
{
  hive_uint32 data;
  hrtx_slave_portid_load(port, addr, &data, sizeof(data));
  return data;
}

extern hive_uint hrtx_ctlid_program_counter(hive_cell_id cell);
extern void hrtx_ctlid_set_break_address(hive_cell_id cell, hive_uint addr);
extern hive_bool hrtx_ctlid_is_broken(hive_cell_id cell);
extern hive_bool hrtx_ctlid_get_break(hive_cell_id cell);
extern hive_bool hrtx_ctlid_get_irq_broken(hive_cell_id cell);
extern hive_bool hrtx_ctlid_get_irq_ready(hive_cell_id cell);
extern hive_bool hrtx_ctlid_get_irq_sleeping(hive_cell_id cell);
extern void hrtx_ctlid_set_arbiter_period(hive_cell_id cell, hive_uint arb, hive_uint32 period);
extern void hrtx_ctlid_set_arbiter_contender_bandwidth(hive_cell_id cell, hive_uint contender, hive_uint32 bandwidth);
extern void hrtx_ctlid_set_arbiter(hive_cell_id cell, hive_uint arbiter, hive_uint32 period, hive_uint32 *bandwidth, hive_uint length);
extern hive_uint32 hrtx_ctlid_get_arbiter_period(hive_cell_id cell, hive_uint arb);
extern hive_uint32 hrtx_ctlid_get_arbiter_period_num_bits(hive_cell_id cell, hive_uint arb);
extern hive_uint32 hrtx_ctlid_get_arbiter_contender_bandwidth(hive_cell_id cell, hive_uint contender);
extern hive_uint32 hrtx_ctlid_get_arbiter_contender_bandwidth_num_bits(hive_cell_id cell, hive_uint contender);
extern hive_uint32 hrtx_ctlid_get_start_address(hive_cell_id cell);
extern hive_uint32 hrtx_ctlid_get_break_address(hive_cell_id cell);

extern hive_address hrtx_ctlid_get_ext_base_address(hive_cell_id cell, hive_uint mt_int, hive_uint segment);
extern void hrtx_ctlid_set_ext_base_address(hive_cell_id cell, hive_uint mt_int, hive_uint segment, hive_address addr);
extern void hrtx_ctlid_set_burst_interval(hive_cell_id cell, hive_uint mt_int, hive_uint32 interval);
extern hive_uint32 hrtx_ctlid_get_burst_interval(hive_cell_id cell, hive_uint mt_int);
extern hive_uint32 hrtx_ctlid_get_burst_length(hive_cell_id cell, hive_uint mt_int);
extern void hrtx_ctlid_set_burst_length(hive_cell_id cell, hive_uint mt_int, hive_uint32 length);
extern void hrtx_ctlid_set_single_step(hive_cell_id cell, hive_bool enable);
extern hive_bool hrtx_ctlid_get_single_step(hive_cell_id cell);
extern void hrtx_ctlid_start(hive_cell_id cell);
extern hive_uint hrtx_cellid_mem_size(hive_cell_id cell, hive_mem_id mem);
extern hive_uint hrtx_cellid_mem_physical_size(hive_cell_id cell, hive_mem_id mem);

extern hive_bool hrtx_fifoid_snd_poll(hive_fifo_id fifo);
extern hive_bool hrtx_fifoid_rcv_poll(hive_fifo_id fifo);
extern void hrtx_fifoid_snd(hive_fifo_id fifo, hive_uint32 data);
extern hive_int32 hrtx_fifoid_rcv(hive_fifo_id fifo);

#ifdef C_RUN
#define hrtx_sysmemid_scalar_store(type, var, data, mem, addr) \
	(var) = (data)
#define hrtx_sysmemid_scalar_load(type, var, mem, addr) \
  (var)
#define hrtx_sysmemid_indexed_store(type, array, index, data, mem, addr) \
	((array)[index]) = (data)
#define hrtx_sysmemid_indexed_load(type, array, index, mem, addr) \
	((array)[index])
#else
#define hrtx_sysmemid_scalar_store(type, var, data, mem, addr) \
        HRTCAT(hrtx_sysmemid_store_,type)(mem, addr, data) 
#define hrtx_sysmemid_scalar_load(type, var, mem, addr) \
        HRTCAT(hrtx_sysmemid_load_,type)(mem, addr)
#define hrtx_sysmemid_indexed_store(type, array, index, data, mem, addr) \
        HRTCAT(hrtx_sysmemid_store_,type)(mem, (addr) + ((index)*sizeof(type)), data)
#define hrtx_sysmemid_indexed_load(type, array, index, mem, addr) \
        HRTCAT(hrtx_sysmemid_load_,type)(mem, (addr) + ((index)*sizeof(type)))
#endif

#define hrtx_sysmemid_load_8(sysmem, addr) \
	hrtx_slave_portid_load_8(hrtx_sysmemid_slave_port(sysmem), (addr))
#define hrtx_sysmemid_load_16(sysmem, addr) \
	hrtx_slave_portid_load_16(hrtx_sysmemid_slave_port(sysmem), (addr))
#define hrtx_sysmemid_load_32(sysmem, addr) \
	hrtx_slave_portid_load_32(hrtx_sysmemid_slave_port(sysmem), (addr))

#define hrtx_sysmemid_store_8(sysmem, addr, value) \
        hrtx_slave_portid_store_8(hrtx_sysmemid_slave_port(sysmem), (addr), value)
#define hrtx_sysmemid_store_16(sysmem, addr, value) \
        hrtx_slave_portid_store_16(hrtx_sysmemid_slave_port(sysmem), (addr), value)
#define hrtx_sysmemid_store_32(sysmem, addr, value) \
        hrtx_slave_portid_store_32(hrtx_sysmemid_slave_port(sysmem), (addr), value)

#define hrtx_sysmemid_load_char(sysmem, addr)  hrtx_sysmemid_load_8(sysmem, addr)
#define hrtx_sysmemid_load_short(sysmem, addr) hrtx_sysmemid_load_16(sysmem, addr)
#define hrtx_sysmemid_load_int(sysmem, addr)   hrtx_sysmemid_load_32(sysmem, addr)
#define hrtx_sysmemid_load_long(sysmem, addr)  hrtx_sysmemid_load_32(sysmem, addr)

#define hrtx_sysmemid_store_char(sysmem, addr, value)  hrtx_sysmemid_store_8(sysmem, addr, value)
#define hrtx_sysmemid_store_short(sysmem, addr, value) hrtx_sysmemid_store_16(sysmem, addr, value)
#define hrtx_sysmemid_store_int(sysmem, addr, value)   hrtx_sysmemid_store_32(sysmem, addr, value)
#define hrtx_sysmemid_store_long(sysmem, addr, value)  hrtx_sysmemid_store_32(sysmem, addr, value)

extern hive_port_id hrtx_sysmemid_slave_port(hive_mem_id sysmem);
extern void hrtx_sysmemid_store(hive_mem_id sysmem, hive_address addr, const void *data, hive_uint bytes);
extern void hrtx_sysmemid_load(hive_mem_id sysmem, hive_address addr, void *data, hive_uint bytes);

extern void hrtx_memid_store(hive_cell_id cell, hive_mem_id mem, hive_address addr, const void *data, hive_uint bytes);
extern void hrtx_memid_load(hive_cell_id cell, hive_mem_id mem, hive_address addr, void *data, hive_uint bytes);
extern void hrtx_memid_set(hive_cell_id cell, hive_mem_id mem, hive_address addr, int c, hive_uint bytes);

#define hrtx_memid_store_char(cell, mem, addr, val)  hrtx_memid_store_8(cell, mem, addr, val)
#define hrtx_memid_store_short(cell, mem, addr, val) hrtx_memid_store_16(cell, mem, addr, val)
#define hrtx_memid_store_int(cell, mem, addr, val)   hrtx_memid_store_32(cell, mem, addr, val)
#define hrtx_memid_store_long(cell, mem, addr, val)  hrtx_memid_store_32(cell, mem, addr, val)

#define hrtx_memid_load_char(cell, mem, addr)  hrtx_memid_load_8(cell, mem, addr)
#define hrtx_memid_load_short(cell, mem, addr) hrtx_memid_load_16(cell, mem, addr)
#define hrtx_memid_load_int(cell, mem, addr)   hrtx_memid_load_32(cell, mem, addr)
#define hrtx_memid_load_long(cell, mem, addr)  hrtx_memid_load_32(cell, mem, addr)

#define hrtx_memid_zero(cell, mem, addr, bytes) hrtx_memid_set(cell, mem, addr, 0, bytes)
static inline void hrtx_memid_store_8(hive_cell_id cell, hive_mem_id mem, hive_address addr, hive_int8 data)
{
  hrtx_memid_store(cell, mem, addr, &data, sizeof(data));
}
static inline void hrtx_memid_store_16(hive_cell_id cell, hive_mem_id mem, hive_address addr, hive_int16 data)
{
  hrtx_memid_store(cell, mem, addr, &data, sizeof(data));
}
static inline void hrtx_memid_store_32(hive_cell_id cell, hive_mem_id mem, hive_address addr, hive_int32 data)
{
  hrtx_memid_store(cell, mem, addr, &data, sizeof(data));
}
static inline hive_int8 hrtx_memid_load_8(hive_cell_id cell, hive_mem_id mem, hive_address addr)
{
  hive_int8 data;
  hrtx_memid_load(cell, mem, addr, &data, sizeof(data));
 return data;
}
static inline hive_int16 hrtx_memid_load_16(hive_cell_id cell, hive_mem_id mem, hive_address addr)
{
  hive_int16 data;
  hrtx_memid_load(cell, mem, addr, &data, sizeof(data));
 return data;
}
static inline hive_int32 hrtx_memid_load_32(hive_cell_id cell, hive_mem_id mem, hive_address addr)
{
  hive_int32 data;
  hrtx_memid_load(cell, mem, addr, &data, sizeof(data));
 return data;
}

#ifdef HRT_CSIM
/* CSim Device request API. This enables controlling the device models from
   the host application.
*/
#if defined(HRT_CELL) && !defined(C_RUN)
/* simulator request interface only available to cell programs in c_run*/
#define hrtx_cell_simulator_request(cell,request)                      _hrt_error_use_of_hrtx_cell_simulator_request_on_a_cell_in_a_non_c_run_build
#define hrtx_host_simulator_request(host_id,request)                   _hrt_error_use_of_hrtx_host_simulator_request_on_a_cell_in_a_non_c_run_build
#define hrtx_sysmem_simulator_request(mem_id,request)                  _hrt_error_use_of_hrtx_sysmem_simulator_request_on_a_cell_in_a_non_c_run_build
#define hrtx_fifo_adapter_simulator_request(fifo_adapter_id,request)   _hrt_error_use_of_hrtx_fifo_adapter_simulator_request_on_a_cell_in_a_non_c_run_build
#define hrtx_fifo_simulator_request(fifo_id,request)                   _hrt_error_use_of_hrtx_fifo_simulator_request_on_a_cell_in_a_non_c_run_build
#define hrtx_sysbus_simulator_request(bus_id,request)                  _hrt_error_use_of_hrtx_sysbus_simulator_request_on_a_cell_in_a_non_c_run_build
#define hrtx_bridge_simulator_request(bridge_id,request)               _hrt_error_use_of_hrtx_bridge_simulator_request_on_a_cell_in_a_non_c_run_build
#define hrtx_custom_device_simulator_request(custom_device_id,request) _hrt_error_use_of_hrtx_custom_device_simulator_request_on_a_cell_in_a_non_c_run_build
#define hrtx_system_simulator_request(request)                         _hrt_error_use_of_hrtx_system_simulator_request_on_a_cell_in_a_non_c_run_build
#else
void *_hrtx_cell_simulator_request(hive_cell_id cell_id, void *request);
void *_hrtx_host_simulator_request(hive_host_id host_id, void *request);
void *_hrtx_sysmem_simulator_request(hive_mem_id mem_id, void *request);
void *_hrtx_fifo_adapter_simulator_request(hive_fifo_adapter_id fifo_adapter_id, void *request);
void *_hrtx_fifo_simulator_request(hive_fifo_id fifo_id, void *request);
void *_hrtx_sysbus_simulator_request(hive_bus_id bus_id, void *request);
void *_hrtx_bridge_simulator_request(hive_bridge_id bridge_id, void *request);
void *_hrtx_custom_device_simulator_request(hive_custom_device_id custom_device_id, void *request);
void *_hrtx_system_simulator_request(void *request); 

#define hrtx_cell_simulator_request(cell,request) \
  (_hrt_current_device_set_source_info(__FILE__,__LINE__),_hrtx_cell_simulator_request(cell,request))

#define hrtx_host_simulator_request(host_id,request) \
  (_hrt_current_device_set_source_info(__FILE__,__LINE__),_hrtx_host_simulator_request(host_id,request))

#define hrtx_sysmem_simulator_request(mem_id,request) \
  (_hrt_current_device_set_source_info(__FILE__,__LINE__),_hrtx_sysmem_simulator_request(mem_id,request))

#define hrtx_fifo_adapter_simulator_request(fifo_adapter_id,request) \
  (_hrt_current_device_set_source_info(__FILE__,__LINE__),_hrtx_fifo_adapter_simulator_request(fifo_adapter_id,request))

#define hrtx_fifo_simulator_request(fifo_id,request) \
  (_hrt_current_device_set_source_info(__FILE__,__LINE__),_hrtx_fifo_simulator_request(fifo_id,request))

#define hrtx_sysbus_simulator_request(bus_id,request) \
  (_hrt_current_device_set_source_info(__FILE__,__LINE__),_hrtx_sysbus_simulator_request(bus_id,request))
     
#define hrtx_bridge_simulator_request(bridge_id,request) \
  (_hrt_current_device_set_source_info(__FILE__,__LINE__),_hrtx_bridge_simulator_request(bridge_id,request))

#define hrtx_custom_device_simulator_request(custom_device_id,request) \
  (_hrt_current_device_set_source_info(__FILE__,__LINE__),_hrtx_custom_device_simulator_request(custom_device_id,request))

#define hrtx_system_simulator_request(request) \
  (_hrt_current_device_set_source_info(__FILE__,__LINE__),_hrtx_system_simulator_request(request))
#endif
#endif // HRT_CSIM

#endif /* _HRT_EXPERT_API_H_ */
