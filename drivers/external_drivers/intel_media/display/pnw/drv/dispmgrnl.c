/*
 * Copyright © 2012 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 * Hitesh K. Patel <hitesh.k.patel@intel.com>
 *
 */

#include <linux/version.h>
#include <linux/fs.h>
#include <net/genetlink.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include "dispmgrnl.h"
#include "psb_dpst_func.h"
#include "psb_powermgmt.h"

#define NETLINK_DISPMGR		20

static unsigned int g_pid;

struct sock *nl_sk;

static void execute_recv_command(struct dispmgr_command_hdr *cmd_hdr)
{
	switch (cmd_hdr->module) {
	case DISPMGR_MOD_NETLINK:
		{
			switch (cmd_hdr->cmd) {
			case DISPMGR_TEST:
				{
					struct dispmgr_command_hdr send_cmd_hdr;
					unsigned long data = 0xdeadbeef;
					if (cmd_hdr->data_size) {
						unsigned long value =
						    *((unsigned long *)
						      cmd_hdr->data);
						printk
						("kdispmgr: received DISPMGR_"
						"TEST cmd data = 0x%lx.\n",
						value);
					} else {
						printk
						("kdispmgr: received DISPMGR_"
						"TEST cmd NO data.\n");
					}

					send_cmd_hdr.data_size = sizeof(data);
					send_cmd_hdr.data = &data;
					send_cmd_hdr.module =
					    DISPMGR_MOD_NETLINK;
					send_cmd_hdr.cmd = DISPMGR_TEST;
					dispmgr_nl_send_msg(&send_cmd_hdr);
				}
				break;
			case DISPMGR_TEST_TEXT:
				{
					struct dispmgr_command_hdr send_cmd_hdr;
					char *data = "can you hear me?";
					if (cmd_hdr->data_size) {
						printk
						("kdispmgr: received DISPMGR_"
						"TEST_TEXT cmd text = 0x%s.\n",
						(char *)cmd_hdr->data);
					} else {
						printk
						("kdispmgr: received DISPMGR_"
						"TEST_TEXT cmd NO text.\n");
					}
					send_cmd_hdr.module =
					    DISPMGR_MOD_NETLINK;
					send_cmd_hdr.cmd = DISPMGR_TEST_TEXT;
					send_cmd_hdr.data_size =
					    strlen(data) + 1;
					send_cmd_hdr.data = (void *)data;
					dispmgr_nl_send_msg(&send_cmd_hdr);
				}
				break;
			default:
				{
					printk
					("kdispmgr: received unknown "
					"command = %d.\n",
					cmd_hdr->cmd);
				};
			};	/* switch */
		}
		break;
	case DISPMGR_MOD_DPST:
		{
			dpst_execute_recv_command(cmd_hdr);
		}
		break;
	default:
		{
			printk
			("kdispmgr: received unknown "
			"module = %d.\n", cmd_hdr->module);
		};
	}			/* switch */
}

/* Send Message to user mode */
void dispmgr_nl_send_msg(struct dispmgr_command_hdr *cmd_hdr)
{
	struct nlmsghdr *nlh;
	struct sk_buff *skb_out;
	unsigned int msg_size = 0;
	unsigned int data_size = 0;
	unsigned int hdr_size = 0;
	int ret = 0;

	/* if no user mode process active */
	if (!g_pid)
		return;

	hdr_size = sizeof(struct dispmgr_command_hdr);
	data_size = hdr_size + cmd_hdr->data_size;
	msg_size = data_size + sizeof(struct nlmsghdr);

	skb_out = nlmsg_new(msg_size, 0);
	if (!skb_out) {
		printk
		("kdispmgr: Failed to allocated skb\n");
		return;
	}

	nlh = nlmsg_put(skb_out, 0, 0, NLMSG_DONE, msg_size, 0);
	NETLINK_CB(skb_out).dst_group = 0;	/* not in mcast group */

	memcpy(nlmsg_data(nlh), cmd_hdr, hdr_size);
	if (cmd_hdr->data_size) {
		memcpy(nlmsg_data(nlh) + hdr_size, cmd_hdr->data,
		       cmd_hdr->data_size);
	}
	ret = netlink_unicast(nl_sk, skb_out, g_pid, MSG_DONTWAIT);
}

/* Receive Message from Kernel */
static void nl_recv_msg(struct sk_buff *skb)
{
	struct nlmsghdr *nlh = NULL;
	struct dispmgr_command_hdr cmd_hdr = {0, };
	unsigned int hdr_size = sizeof(struct dispmgr_command_hdr);

	if (skb == NULL) {
		DRM_ERROR("kdispmgr: received null command.\n");
		return;
	}

	nlh = (struct nlmsghdr *)skb->data;
	g_pid = nlh->nlmsg_pid;
	pr_debug("kdispmgr: received message from user mode\n");

	memcpy((void *)(&cmd_hdr), NLMSG_DATA(nlh), hdr_size);
	if (cmd_hdr.data_size)
		cmd_hdr.data = NLMSG_DATA(nlh) + hdr_size;

	execute_recv_command(&cmd_hdr);
}

#if DEBUG
static void dispmgr_nl_exit(void)
{
	printk(KERN_INFO "kdispmgr: exiting hello module\n");
	netlink_kernel_release(nl_sk);
	g_pid = 0;
}
#endif

static int dispmgr_nl_init(void)
{
	int ret = 0;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0))
	nl_sk = netlink_kernel_create(&init_net,
				      NETLINK_DISPMGR,
				      0, nl_recv_msg, NULL, THIS_MODULE);
#else
        struct netlink_kernel_cfg cfg = {
                .groups = 0,
		.input = nl_recv_msg,
        };
        nl_sk = netlink_kernel_create(&init_net, NETLINK_DISPMGR, &cfg);
#endif
	if (!nl_sk) {
		printk(KERN_ALERT "kdispmgr: error creating netlink socket.\n");
		ret = -10;
	} else {
		printk(KERN_ALERT
		       "kdispmgr: netlink socket created successfully.\n");
		ret = 0;
	}

	return ret;
}

void dispmgr_start(struct drm_device *dev)
{
	pr_info("kdispmgr: display manager start.\n");
	dispmgr_nl_init();
	return;
}

/* this function is only called by dpms on or late resume function */
void dpstmgr_reg_restore_locked(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *ctx = NULL;
	struct drm_psb_private *dev_priv = NULL;
	struct mdfld_dsi_hw_registers *regs = NULL;

	if (!dsi_config || !dsi_config->dev)
		return;
	ctx = &dsi_config->dsi_hw_context;
	regs = &dsi_config->regs;
	dev_priv = dsi_config->dev->dev_private;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				OSPM_UHB_ONLY_IF_ON))
			return;

	PSB_WVDC32(ctx->histogram_intr_ctrl, regs->histogram_intr_ctrl_reg);
	PSB_WVDC32(ctx->histogram_logic_ctrl, regs->histogram_logic_ctrl_reg);
	PSB_WVDC32(ctx->aimg_enhance_bin, regs->aimg_enhance_bin_reg);
	PSB_WVDC32(ctx->lvds_port_ctrl, regs->lvds_port_ctrl_reg);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}
