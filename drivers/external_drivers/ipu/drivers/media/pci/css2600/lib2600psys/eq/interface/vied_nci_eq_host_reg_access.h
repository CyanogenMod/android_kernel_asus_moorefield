#ifndef _VIED_NCI_EQ_HOST_REG_ACCESS_H_
#define _VIED_NCI_EQ_HOST_REG_ACCESS_H_

/* Main purpose of this library is to avoid missing prototypes compilation warnings */

void event_queue_ip_reg_store(
    const vied_nci_eq_device_t	dev,
    const unsigned int	reg,
    const unsigned int	value);

unsigned int event_queue_ip_reg_load(
    const vied_nci_eq_device_t	dev,
    const unsigned int	reg);

void event_queue_op_reg_store(
    const vied_nci_eq_device_t	dev,
    const unsigned int	reg,
    const unsigned int	value);

unsigned int event_queue_op_reg_load(const unsigned int reg);

#endif /* _VIED_NCI_EQ_HOST_REG_ACCESS_H_ */
