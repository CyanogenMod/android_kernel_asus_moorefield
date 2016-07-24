#ifndef _HOST_VIED_DEVICE_ACCESS_IMPL_H
#define _HOST_VIED_DEVICE_ACCESS_IMPL_H

#include <vied/vied_system.h>
#include <vied/vied_subsystem_access.h>
#include <vied/vied_device_property.h>

/* Address calculation functions */
static inline
vied_address_t _vied_get_register_address(const vied_device_route_t *route,
                                          vied_device_reg_id_t reg)
{
	vied_address_t offset  = route->internal_route->address_map[reg];
        vied_address_t res = route->internal_route->slave_address + offset;
	return res;
}

static inline
vied_address_t _vied_get_memory_address(const vied_device_route_t *route,
                                        vied_device_memory_id_t mem,
                                        vied_address_t mem_offset)
{
	vied_address_t mem_from_slave = route->internal_route->address_map[mem];
        vied_address_t res = route->internal_route->slave_address +
                             mem_from_slave + mem_offset;
	return res;
}

/* Device access handle */
_VIED_DEVICE_ACCESS_INLINE
const vied_device_t *vied_device_open(vied_subsystem_t tgt_ss, vied_device_id_t tgt_dev)
{
	vied_device_t *res = vied_system_malloc(sizeof(vied_device_t));
	/* device -> slave -> route */
	const vied_internal_route_t **route_map_map = vied_system_get_address_table(tgt_ss);
        size_t i, num_slave_ports;
        
	res->subsystem_id = tgt_ss;
        num_slave_ports = vied_device_get_num_slave_ports (tgt_ss, tgt_dev);
        res->route_map = vied_system_malloc (sizeof(vied_device_route_t) * num_slave_ports);
        for (i=0; i < num_slave_ports; i++) {
          res->route_map[i].subsystem_id = tgt_ss;
          res->route_map[i].internal_route = &route_map_map[tgt_dev][i];
        }
	return res;
}

_VIED_DEVICE_ACCESS_INLINE
void vied_device_close(const vied_device_t *tgt_dev)
{
	vied_system_free((void*)tgt_dev->route_map);
	vied_system_free((void*)tgt_dev);
}

/* Device addressing */
_VIED_DEVICE_ACCESS_INLINE
const vied_device_route_t *
vied_device_get_route(const vied_device_t *dev, vied_device_port_id_t  port)
{
	return &dev->route_map[port];
}

/* Device register access */
_VIED_DEVICE_ACCESS_INLINE
void vied_device_reg_store_32(const vied_device_route_t *route,
                              vied_device_reg_id_t reg, uint32_t data)
{
	vied_address_t address = _vied_get_register_address(route, reg);
	vied_subsystem_store_32(route->subsystem_id, address, data);
}

_VIED_DEVICE_ACCESS_INLINE
uint32_t vied_device_reg_load_32(const vied_device_route_t *route,
                                 vied_device_reg_id_t reg)
{
	uint32_t res;
	vied_address_t address = _vied_get_register_address(route, reg);
	res = vied_subsystem_load_32(route->subsystem_id, address);
	return res;
}

/* Device memory access */
_VIED_DEVICE_ACCESS_INLINE
void vied_device_mem_store_8(const vied_device_route_t *route,
                             vied_device_memory_id_t mem, vied_address_t mem_addr,
                             uint8_t data)
{
        vied_address_t address = _vied_get_memory_address(route, mem, mem_addr);
	vied_subsystem_store_8(route->subsystem_id, address, data);
}

_VIED_DEVICE_ACCESS_INLINE void
vied_device_mem_store_16(const vied_device_route_t *route,
                         vied_device_memory_id_t mem, vied_address_t mem_addr,
                         uint16_t data)
{
        vied_address_t address = _vied_get_memory_address(route, mem, mem_addr);
	vied_subsystem_store_16(route->subsystem_id, address, data);
}

_VIED_DEVICE_ACCESS_INLINE void
vied_device_mem_store_32(const vied_device_route_t *route,
                         vied_device_memory_id_t mem, vied_address_t mem_addr,
                         uint32_t data)
{
        vied_address_t address = _vied_get_memory_address(route, mem, mem_addr);
	vied_subsystem_store_32(route->subsystem_id, address, data);
}

_VIED_DEVICE_ACCESS_INLINE void
vied_device_mem_store(const vied_device_route_t *route,
                      vied_device_memory_id_t mem, vied_address_t mem_addr,
                      const void *data, unsigned int size)
{
        vied_address_t address = _vied_get_memory_address(route, mem, mem_addr);
	vied_subsystem_store(route->subsystem_id, address, data, size);

}

_VIED_DEVICE_ACCESS_INLINE uint8_t
vied_device_mem_load_8(const vied_device_route_t *route,
                       vied_device_memory_id_t mem, vied_address_t mem_addr)
{
	uint8_t res;
	vied_address_t address = _vied_get_memory_address(route, mem, mem_addr);
	res = vied_subsystem_load_8(route->subsystem_id, address);
	return res;
}

_VIED_DEVICE_ACCESS_INLINE uint16_t
vied_device_mem_load_16(const vied_device_route_t *route,
                        vied_device_memory_id_t mem, vied_address_t mem_addr)
{
	uint16_t res;
	vied_address_t address = _vied_get_memory_address(route, mem, mem_addr);
	res = vied_subsystem_load_16(route->subsystem_id, address);
	return res;
}

_VIED_DEVICE_ACCESS_INLINE uint32_t
vied_device_mem_load_32(const vied_device_route_t *route,
                        vied_device_memory_id_t mem, vied_address_t mem_addr)
{
	uint32_t res;
	vied_address_t address = _vied_get_memory_address(route, mem, mem_addr);
	res = vied_subsystem_load_32(route->subsystem_id, address);
	return res;
}

_VIED_DEVICE_ACCESS_INLINE void
vied_device_mem_load(const vied_device_route_t *route,
                     vied_device_memory_id_t mem, vied_address_t mem_addr,
                     void *data, unsigned int size)
{
	vied_address_t address = _vied_get_memory_address(route, mem, mem_addr);
	vied_subsystem_load(route->subsystem_id, address, data, size);
}

#endif /* _HOST_VIED_DEVICE_ACCESS_IMPL_H */
