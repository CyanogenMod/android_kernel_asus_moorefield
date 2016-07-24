/*
 * HECI bus layer messages handling
 *
 * Copyright (c) 2003-2014, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef _HECI_HBM_H_
#define _HECI_HBM_H_

struct heci_device;
struct heci_msg_hdr;
struct heci_cl;

/**
 * enum heci_hbm_state - host bus message protocol state
 *
 * @HECI_HBM_IDLE : protocol not started
 * @HECI_HBM_START : start request message was sent
 * @HECI_HBM_ENUM_CLIENTS : enumeration request was sent
 * @HECI_HBM_CLIENT_PROPERTIES : acquiring clients properties
 */
enum heci_hbm_state {
	HECI_HBM_IDLE = 0,
	HECI_HBM_START,
	HECI_HBM_STARTED,
	HECI_HBM_ENUM_CLIENTS,
	HECI_HBM_CLIENT_PROPERTIES,
	HECI_HBM_STOPPED,
};

void heci_hbm_dispatch(struct heci_device *dev, struct heci_msg_hdr *hdr);

static inline void heci_hbm_hdr(struct heci_msg_hdr *hdr, size_t length)
{
	hdr->host_addr = 0;
	hdr->me_addr = 0;
	hdr->length = length;
	hdr->msg_complete = 1;
	hdr->reserved = 0;
}

int heci_hbm_start_req(struct heci_device *dev);
int heci_hbm_start_wait(struct heci_device *dev);
int heci_hbm_cl_flow_control_req(struct heci_device *dev, struct heci_cl *cl);
int heci_hbm_cl_disconnect_req(struct heci_device *dev, struct heci_cl *cl);
int heci_hbm_cl_connect_req(struct heci_device *dev, struct heci_cl *cl);
void heci_hbm_enum_clients_req(struct heci_device *dev);

#endif /* _HECI_HBM_H_ */
