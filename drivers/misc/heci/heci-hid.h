#ifndef HECI_HID__H
#define	HECI_HID__H

/*
 * Constraint 1: currently we support only 1 ISH in a system
 */

static const uuid_le ish_heci_guid = UUID_LE(0x33AECD58, 0xB679, 0x4E54, 0x9B, 0xD9, 0xA0, 0x4D, 0x34, 0xF0, 0xC2, 0x26);

struct hostif_msg_hdr {
	uint8_t	command;	/* Bit 7: is_response */
#define	CMD_MASK	0x7F
#define	IS_RESPONSE	0x80
	uint8_t	device_id;
	uint8_t	status;
	uint8_t	reserved;
} __packed;

struct hostif_msg {
	struct hostif_msg_hdr	hdr;
} __packed;

struct hostif_msg_to_sensor {
	struct hostif_msg_hdr	hdr;
	uint8_t	report_id;
} __packed;

struct device_info {
	uint32_t	dev_id;
	uint8_t		dev_class;
	uint16_t	pid;
	uint16_t	vid;
} __packed;

#if 0
/* Needed? */
struct heci_client {
	uint32_t	max_msg_len;
	uint8_t		proto_ver;
} __packed;
#endif

struct heci_version {
	uint8_t	major;
	uint8_t	minor;
	uint8_t	hotfix;
	uint16_t	build;
} __packed;

/* HOSTIF commands */
#define	HOSTIF_HID_COMMAND_BASE		0
#define	HOSTIF_GET_HID_DESCRIPTOR	0
#define	HOSTIF_GET_REPORT_DESCRIPTOR	1
#define HOSTIF_GET_FEATURE_REPORT	2
#define	HOSTIF_SET_FEATURE_REPORT	3
#define	HOSTIF_GET_INPUT_REPORT		4
#define	HOSTIF_PUBLISH_INPUT_REPORT	5
#define	HOSTIF_GET_OUTPUT_REPORT	6
#define	HOSTIF_SET_OUTPUT_REPORT	7
#define	HOSTIF_HID_COMMAND_MAX		31
#define	HOSTIF_DM_COMMAND_BASE		32
#define	HOSTIF_DM_ENUM_DEVICES		33
#define	HOSTIF_DM_ADD_DEVICE		34

/* Meaning, too large data source = "over 9000?" :-) */
#define	MAX_DATA_BUF	9000

#define	MAX_HID_DEVICES	32

#include "utils.h"

#endif	/* HECI_HID__H */
