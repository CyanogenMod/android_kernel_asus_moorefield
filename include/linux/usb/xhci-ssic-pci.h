/**
 * Intel MID Platform XHCI SSIC Controller Definition.
 *
 * Copyright (c) 2014, Intel Corporation.
 * Author: Tang, Jianqiang <jianqiang.tang@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License 2 as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __XHCI_SSIC_PCI_H
#define __XHCI_SSIC_PCI_H

/*SSIC Policy and Implementation Specific Registers */
#define SSIC_POLICY_BASE		0x8800

/* Capability ID Register */
#define SSIC_POLICY_CAPABILITY_ID	0x00
#define SUPPORTED_PROTOCOL_ID		(1<<0) /* Core, Supported Protocol ID */
#define NEXT_CAPABILITY_POINTER		(1<<8) /* Next Capability Pointer */

/* SSIC Global Configuration Control */
#define SSIC_GLOBAL_CONFIGURATION_CONTROL 0x04
#define PWM_CLK_GATE_DIS		(1<<0)
#define HS_CLK_GATE_DIS			(1<<1)
#define	CMN_LANE_PWRGATE_DIS		(1<<2)

/* SSIC Configuration Register 1 */
#define SSIC_CONFIGURATION_REGISTER1	0x08
#define SSICLANE			(1<<0)
#define HSGEAR				(1<<2)
#define SSICRATE			(1<<4)
#define MIN_HIBERN8_TIME		(1<<5)
#define T_ACT_H8_EXIT			(1<<8)
#define RRAP_BYPASS			(1<<12)
#define TX_MIN_ACTIVATE_DEFAULT		(1<<13)
#define DSP_DISC_BURST_CLOSE_RRAP	(1<<16)
#define PWM_EXIT_TIME			(1<<17)
#define TX_MIN_STALL			(1<<22)
#define NUM_RRAP_ATTEMPT		(1<<30)

/* SSIC Configuration Register 2 */
#define SSIC_CONFIGURATION_REGISTER2	0x0C
#define ACTIVATE_LRST_TIME		(1<<0)
#define LRST_TIME			(1<<8)
#define PHY_RESET_TIME			(1<<16)
#define RETRAIN_TIME			(1<<21)
#define DISABLE_SCRAMBLING		(1<<25)
#define NUM_OF_MK0			(1<<26)
#define PROG_DONE			(1<<30)
#define SSIC_PORT_UNUSED		(1<<31)

/* SSIC Configuration Register 3 */
#define SSIC_CONFIGURATION_REGISTER3	0x10
#define SSIC_PG_U2_DIS			(1<<0)
#define SSIC_PG_U3_DIS			(1<<1)
#define DISABLE_U0_STALL		(1<<2)
#define LUP_LDN_TIMER_MAX		(1<<19)
#define HIBERN8_ENTER_TX		(1<<21)
#define DL_PWR_GATE_DIS			(1<<26)
#define MPHY_TEST_MODE_EN		(1<<27)
#define U0_STALL_TO			(1<<28)

/* SSIC Configuration Register 4 */
#define SSIC_CONFIGURATION_REGISTER4	0x14
#define CRD_PEND_TIMER_MAX		(1<<0)
#define HP_PEND_TIMER_MAX		(1<<8)
#define ENTRY_TIMER_MAX			(1<<16)
#define LC_MAX_TIMER			(1<<24)

/* SSIC Loopback Config Register */
#define SSIC_LOOPBACK_REGISTER		0x18
#define RX_LOOPBACK_CNTR_RESET		(1<<0)
#define LOOPBACK_EN			(1<<4)

/* Loopback Burst Count Register */
#define LOOPBACK_BURST_COUNT_REGISTER	0x1C
#define RX_BURST_COUNT_LANE0		(1<<0)
#define RX_BURST_COUNT_LANE1		(1<<1)
#define RX_BURST_COUNT_LANE2		(1<<2)
#define RX_BURST_COUNT_LANE3		(1<<3)

/* SSIC Loopback Error Count Register */
#define LOOPBACK_ERROR_COUNT_REGISTER	0x20
#define RX_ERR_COUNT_LANE0		(1<<0)
#define RX_ERR_COUNT_LANE1		(1<<8)
#define RX_ERR_COUNT_LANE2		(1<<16)
#define RX_ERR_COUNT_LANE3		(1<<24)

/* SSIC Local and Remote Profile Registers */
#define SSIC_LOCAL_REMOTE_PROFILE_REGISTER 0x8900

/* SSIC Capability Register */
#define SSIC_CAPABILITY_REGISTER	0x00
#define CAPABILITY_ID			(1<<0)
#define NEXT_CAPABILITY_POINTER		(1<<8)

/* SSIC Port N Register Access Control */
#define PORT_REGISTER_ACCESS_CONTROL	0x04
#define ATTRIBUTE_WRITE_DATA		(1<<0)
#define ACCESS_CONTROL_ATTRIBUTE_ID	(1<<8)
#define READ_WRITE			(1<<20)
#define COMMAND_VALID			(1<<21)
#define COMMAND_PHASE_DONE		(1<<22)
#define HS_CONFIG			(1<<23)
#define ACCESS_CONTROL_TARGET_PHY	(1<<24)
#define REGISTER_BANK_VALID		(1<<25)

/* SSIC Port N Register Access Status */
#define REGISTER_ACCESS_STATUS		0x08
#define READ_DATA			(1<<0)
#define COMMAND_COMPLETION_STATUS	(1<<8)

/* Profile Attributes */
#define PROFILE_ATTRIBUTES_START(port)	(0x0C + (port) * 0x110)
#define PROFILE_ATTRIBUTES_END(port)	(0x108+(port) * 0x110)

#define ATTRIBUTE_VALUE		(1<<0)
#define TARGET_PHY		(1<<14)
#define ATTRIBUTE_VALID		(1<<15)
#define ATTRIBUTE_ID(v)		((v & 0xFFF) << 16)

/**
 * struct ssic_policy_registers - Policy and Implementation Specific Registers
 * @cap_id:		Capability ID Register
 * @global_control:	Global Configuration Control
 * @config_reg1:	Configuration Register 1
 * @config_reg2:	Configuration Register 2
 * @config_reg3:	Configuration Register 3
 * @config_reg4:	Configuration Register 4
 * @loopback_reg:	Loopback Config Register
 * @loopback_burst_conut:	Loopback Burst Count Register
 * @loopback_err_count:	Loopback Error Count Register
 */
struct ssic_policy_regs {
	__le32  cap_id;
	__le32  global_control;
	__le32  config_reg1;
	__le32  config_reg2;
	__le32  config_reg3;
	__le32  config_reg4;
	__le32  loopback_reg;
	__le32  loopback_burst_count;
	__le32  loopback_err_count;
};

/**
 * struct ssic_profile_registers - Policy and Implementation Specific Registers
 * @cap_reg:		Capability Register
 * @access_control:	Port N Register Access Control
 * @access_status:	Port N Register Access Status
 * @attribute_base:	attribute base address
 */
struct ssic_profile_regs {
	__le32  cap_reg;
	__le32  access_control;
	__le32  access_status;
	__le32	attribute_base;
};

typedef enum {
	LOCAL_PHY,
	REMOTE_PHY
} target_phy;

struct mphy_attr_setting {
	target_phy target;
	u16 attr_id;
	u8 val;
};

/* Just define the Max SSIC port to 1 */
#define NUM_SSIC_PORTS		1
/* Attribute Max items now for each port */
#define ATTRIBUTE_MAX		64

/* Profile Attribute used */
#define ATTRID_TX_HSRATE_SERIES 0x22
#define ATTRID_RX_HSRATE_SERIES	0xA2
#define ATTRID_TX_HS_SYNC_LENGTH 0x28
#define ATTRID_TX_HS_PREPARE_LENGTH 0x29
#define ATTRID_TX_PWM_BURST_CLOSURE_EXTENDSION 0x2D
#define ATTRID_MC_RX_LA_CAPABILITY 0xD7
#define ATTRID_MC_HS_START_TIME_VAR_CAPABILITY 0xD4
#define ATTRID_RX_TERMINATION_FORCE_ENABLE 0xA9

/* MODEM CUSTOM ATTR IDs */
#define ATTRID_MODEM_CB_REG_A38	0xA38
#define ATTRID_MODEM_CB_REG_A38	0xA38
#define ATTRID_MODEM_CB_REG_A39	0xA39
#define ATTRID_MODEM_CB_UPDATE_40A	0x40A
#define ATTRID_MODEM_CUSTOM_REG_CD	0xCD
#define ATTRID_MODEM_CUSTOM_REG_C5	0xC5
#define ATTRID_MODEM_CUSTOM_REG_F3	0xF3

#define ATTRID_DISABLE_SCRAMBLING	0x403
#define ATTRID_DISABLE_STALL_IN_U0	0x404

/* DISABLE_STALL_IN_U0
 * A DSP to disable STALL entry in U0 in an USP.
 * A DSP or a USP in the MPHY.TEST state shall ignore
 * writes to this register.
 *
 * Write 1 configures the USP to disable STALL entry while
 * in U0.
 * Write 0 shall have no effect.
 */
#define ATTR_VAL_DISABLE_USP_STALL_IN_U0	1

/* DISABLE_SCRAMBLING
 * DSP to indicate to an USP that data transmission is
 * HS-MODE shall have scrambling disabled.
 * Disable HS-MODE scrambling: 1
 * Write 0 shall have no effect.
 */
#define ATTR_VAL_DISABLE_HSMODE_SCRAMBLING	1

/* TX_HSRATE_SERIES/RX_HSRATE_SERIES
 * HS mode RATE serious value:
 * A = 1
 * B = 2
 * */
#define ATTR_VAL_HSMODE_RATE_SERIES_A		0x1
#define ATTR_VAL_HSMODE_RATE_SERIES_B		0x2

/* TX_HS_SYNC_LENGTH
 * B[7:6] : SYNC_RANGE
 * B[5:0] : SYNC_LENGTH
 */
#define ATTR_VAL_SYNC_RANGE(v)		((v & 0x3) << 6)
#define ATTR_VAL_SYNC_LENGTH(v)		(v & 0x3F)

/* TX_PWM_BURST_Closure_Extension
 * B[7:0] : BURST CLOSURE sequence duration in SI
 */
#define ATTR_VAL_BURST_CLOSURE_SEQ_DURATION(v) (v & 0xFF)

/* RX_Termination_Force_Enable
 * Force connection of differential termination resistance,
 * RDIF_RX to enabled state, for RX S-Parameter test purposes.
 * NO = 0
 * YES = 1
 */
#define ATTR_VAL_ENABLE_RDIF_RX	1
#define ATTR_VAL_DISABLE_RDIF_RX	0

/* MC_RX_LA_CAPABILITY
 * Specifies whether or not OMC supports Large Amplitude
 * FALSE = 0
 * TRUE = 1
 */
#define ATTR_VAL_OMC_SUPPORTS_LARGE_AMPLITUDE	1

/* TX_HS_PREPARE_LENGTH
 * B[3:0] : HS PREPARE length multiplier for M-TX
 */
#define ATTR_VAL_M_TX_LENGTH_MULTIPLIER(v)	(v & 0xF)

#define SSIC_PORT_INACTIVITYDURATION		500
#define SSIC_AUTOSUSPEND			0
#define SSIC_BUS_INACTIVITYDURATION		500
#define SSIC_REMOTEWAKEUP			1

#define SSIC_GET_PORT_STATUS			(0xa300 | USB_REQ_GET_STATUS)
#define SSIC_HUB_DEBOUNCE_STEP			25
#define SSIC_HUB_DEBOUNCE_TIMEOUT		2000
#define SSIC_STS_TIMEOUT			1000
#define SSIC_STS_RETRIES			5

enum power_state {
	POWER_RESUMED,
	POWER_RESUMING,
	POWER_SUSPENDED,
	POWER_SUSPENDING
};

/**
 * struct usb ssic_hcd - SSIC host controller driver
 * @ssic_mutex:		mutex for protect interfaces
 * @ssic_mutex_init:	bitfield flag
 * @ssic_lock:		ssic driver internal spinlock
 */
struct ssic_xhci_hcd {
	struct mutex		ssic_mutex;
	struct ssic_policy_regs __iomem *policy_regs;
	struct ssic_profile_regs  __iomem *profile_regs;

	struct mutex		wakelock_mutex;
	spinlock_t		ssic_lock;

	/* roothub and modem device */
	struct usb_device	*rh_dev;
	struct usb_device	*modem_dev;

	/* autosuspend and U3 Timeout field */
	unsigned		autosuspend_enable;
	unsigned		port_inactivity_duration;
	unsigned		bus_inactivity_duration;

	/* remote wakeup field */
	unsigned		remote_wakeup_enable;

	unsigned		ssic_enable;
	int			ssic_port;

	unsigned		u1_enable;
	unsigned		u2_enable;
	unsigned		u1_inactivity_duration;
	unsigned		u2_inactivity_duration;

	/* Modem device add/remove notifier */
	struct notifier_block	ssicdev_nb;
	struct notifier_block	ssic_pm_nb;
	struct notifier_block	ssic_power_nb;

	struct wake_lock	ssic_wake_lock;
	unsigned long		wakelock_state;
	unsigned long		power_state;
	unsigned		first_reset;
};
#endif
