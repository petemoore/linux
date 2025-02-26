// SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
/* Copyright (C) 2017-2019 Netronome Systems, Inc. */

#include <linux/bitfield.h>
#include <linux/errno.h>
#include <linux/etherdevice.h>
#include <linux/if_link.h>
#include <linux/if_ether.h>

#include "nfpcore/nfp_cpp.h"
#include "nfp_app.h"
#include "nfp_main.h"
#include "nfp_net_ctrl.h"
#include "nfp_net.h"
#include "nfp_net_sriov.h"

static int
nfp_net_sriov_check(struct nfp_app *app, int vf, u16 cap, const char *msg)
{
	u16 cap_vf;

	if (!app || !app->pf->vfcfg_tbl2)
		return -EOPNOTSUPP;

	cap_vf = pete_readw("drivers/net/ethernet/netronome/nfp/nfp_net_sriov.c:25", app->pf->vfcfg_tbl2 + NFP_NET_VF_CFG_MB_CAP);
	if ((cap_vf & cap) != cap) {
		nfp_warn(app->pf->cpp, "ndo_set_vf_%s not supported\n", msg);
		return -EOPNOTSUPP;
	}

	if (vf < 0 || vf >= app->pf->num_vfs) {
		nfp_warn(app->pf->cpp, "invalid VF id %d\n", vf);
		return -EINVAL;
	}

	return 0;
}

static int
nfp_net_sriov_update(struct nfp_app *app, int vf, u16 update, const char *msg)
{
	struct nfp_net *nn;
	int ret;

	/* Write update info to mailbox in VF config symbol */
	pete_writeb("drivers/net/ethernet/netronome/nfp/nfp_net_sriov.c:46", vf, app->pf->vfcfg_tbl2 + NFP_NET_VF_CFG_MB_VF_NUM);
	pete_writew("drivers/net/ethernet/netronome/nfp/nfp_net_sriov.c:47", update, app->pf->vfcfg_tbl2 + NFP_NET_VF_CFG_MB_UPD);

	nn = list_first_entry(&app->pf->vnics, struct nfp_net, vnic_list);
	/* Signal VF reconfiguration */
	ret = nfp_net_reconfig(nn, NFP_NET_CFG_UPDATE_VF);
	if (ret)
		return ret;

	ret = pete_readw("drivers/net/ethernet/netronome/nfp/nfp_net_sriov.c:55", app->pf->vfcfg_tbl2 + NFP_NET_VF_CFG_MB_RET);
	if (ret)
		nfp_warn(app->pf->cpp,
			 "FW refused VF %s update with errno: %d\n", msg, ret);
	return -ret;
}

int nfp_app_set_vf_mac(struct net_device *netdev, int vf, u8 *mac)
{
	struct nfp_app *app = nfp_app_from_netdev(netdev);
	unsigned int vf_offset;
	int err;

	err = nfp_net_sriov_check(app, vf, NFP_NET_VF_CFG_MB_CAP_MAC, "mac");
	if (err)
		return err;

	if (is_multicast_ether_addr(mac)) {
		nfp_warn(app->pf->cpp,
			 "invalid Ethernet address %pM for VF id %d\n",
			 mac, vf);
		return -EINVAL;
	}

	/* Write MAC to VF entry in VF config symbol */
	vf_offset = NFP_NET_VF_CFG_MB_SZ + vf * NFP_NET_VF_CFG_SZ;
	pete_writel("drivers/net/ethernet/netronome/nfp/nfp_net_sriov.c:81", get_unaligned_be32(mac), app->pf->vfcfg_tbl2 + vf_offset);
	pete_writew("drivers/net/ethernet/netronome/nfp/nfp_net_sriov.c:82", get_unaligned_be16(mac + 4),
	       app->pf->vfcfg_tbl2 + vf_offset + NFP_NET_VF_CFG_MAC_LO);

	err = nfp_net_sriov_update(app, vf, NFP_NET_VF_CFG_MB_UPD_MAC, "MAC");
	if (!err)
		nfp_info(app->pf->cpp,
			 "MAC %pM set on VF %d, reload the VF driver to make this change effective.\n",
			 mac, vf);

	return err;
}

int nfp_app_set_vf_vlan(struct net_device *netdev, int vf, u16 vlan, u8 qos,
			__be16 vlan_proto)
{
	struct nfp_app *app = nfp_app_from_netdev(netdev);
	unsigned int vf_offset;
	u16 vlan_tci;
	int err;

	err = nfp_net_sriov_check(app, vf, NFP_NET_VF_CFG_MB_CAP_VLAN, "vlan");
	if (err)
		return err;

	if (vlan_proto != htons(ETH_P_8021Q))
		return -EOPNOTSUPP;

	if (vlan > 4095 || qos > 7) {
		nfp_warn(app->pf->cpp,
			 "invalid vlan id or qos for VF id %d\n", vf);
		return -EINVAL;
	}

	/* Write VLAN tag to VF entry in VF config symbol */
	vlan_tci = FIELD_PREP(NFP_NET_VF_CFG_VLAN_VID, vlan) |
		FIELD_PREP(NFP_NET_VF_CFG_VLAN_QOS, qos);
	vf_offset = NFP_NET_VF_CFG_MB_SZ + vf * NFP_NET_VF_CFG_SZ;
	pete_writew("drivers/net/ethernet/netronome/nfp/nfp_net_sriov.c:119", vlan_tci, app->pf->vfcfg_tbl2 + vf_offset + NFP_NET_VF_CFG_VLAN);

	return nfp_net_sriov_update(app, vf, NFP_NET_VF_CFG_MB_UPD_VLAN,
				    "vlan");
}

int nfp_app_set_vf_spoofchk(struct net_device *netdev, int vf, bool enable)
{
	struct nfp_app *app = nfp_app_from_netdev(netdev);
	unsigned int vf_offset;
	u8 vf_ctrl;
	int err;

	err = nfp_net_sriov_check(app, vf, NFP_NET_VF_CFG_MB_CAP_SPOOF,
				  "spoofchk");
	if (err)
		return err;

	/* Write spoof check control bit to VF entry in VF config symbol */
	vf_offset = NFP_NET_VF_CFG_MB_SZ + vf * NFP_NET_VF_CFG_SZ +
		NFP_NET_VF_CFG_CTRL;
	vf_ctrl = pete_readb("drivers/net/ethernet/netronome/nfp/nfp_net_sriov.c:140", app->pf->vfcfg_tbl2 + vf_offset);
	vf_ctrl &= ~NFP_NET_VF_CFG_CTRL_SPOOF;
	vf_ctrl |= FIELD_PREP(NFP_NET_VF_CFG_CTRL_SPOOF, enable);
	pete_writeb("drivers/net/ethernet/netronome/nfp/nfp_net_sriov.c:143", vf_ctrl, app->pf->vfcfg_tbl2 + vf_offset);

	return nfp_net_sriov_update(app, vf, NFP_NET_VF_CFG_MB_UPD_SPOOF,
				    "spoofchk");
}

int nfp_app_set_vf_trust(struct net_device *netdev, int vf, bool enable)
{
	struct nfp_app *app = nfp_app_from_netdev(netdev);
	unsigned int vf_offset;
	u8 vf_ctrl;
	int err;

	err = nfp_net_sriov_check(app, vf, NFP_NET_VF_CFG_MB_CAP_TRUST,
				  "trust");
	if (err)
		return err;

	/* Write trust control bit to VF entry in VF config symbol */
	vf_offset = NFP_NET_VF_CFG_MB_SZ + vf * NFP_NET_VF_CFG_SZ +
		NFP_NET_VF_CFG_CTRL;
	vf_ctrl = pete_readb("drivers/net/ethernet/netronome/nfp/nfp_net_sriov.c:164", app->pf->vfcfg_tbl2 + vf_offset);
	vf_ctrl &= ~NFP_NET_VF_CFG_CTRL_TRUST;
	vf_ctrl |= FIELD_PREP(NFP_NET_VF_CFG_CTRL_TRUST, enable);
	pete_writeb("drivers/net/ethernet/netronome/nfp/nfp_net_sriov.c:167", vf_ctrl, app->pf->vfcfg_tbl2 + vf_offset);

	return nfp_net_sriov_update(app, vf, NFP_NET_VF_CFG_MB_UPD_TRUST,
				    "trust");
}

int nfp_app_set_vf_link_state(struct net_device *netdev, int vf,
			      int link_state)
{
	struct nfp_app *app = nfp_app_from_netdev(netdev);
	unsigned int vf_offset;
	u8 vf_ctrl;
	int err;

	err = nfp_net_sriov_check(app, vf, NFP_NET_VF_CFG_MB_CAP_LINK_STATE,
				  "link_state");
	if (err)
		return err;

	switch (link_state) {
	case IFLA_VF_LINK_STATE_AUTO:
	case IFLA_VF_LINK_STATE_ENABLE:
	case IFLA_VF_LINK_STATE_DISABLE:
		break;
	default:
		return -EINVAL;
	}

	/* Write link state to VF entry in VF config symbol */
	vf_offset = NFP_NET_VF_CFG_MB_SZ + vf * NFP_NET_VF_CFG_SZ +
		NFP_NET_VF_CFG_CTRL;
	vf_ctrl = pete_readb("drivers/net/ethernet/netronome/nfp/nfp_net_sriov.c:198", app->pf->vfcfg_tbl2 + vf_offset);
	vf_ctrl &= ~NFP_NET_VF_CFG_CTRL_LINK_STATE;
	vf_ctrl |= FIELD_PREP(NFP_NET_VF_CFG_CTRL_LINK_STATE, link_state);
	pete_writeb("drivers/net/ethernet/netronome/nfp/nfp_net_sriov.c:201", vf_ctrl, app->pf->vfcfg_tbl2 + vf_offset);

	return nfp_net_sriov_update(app, vf, NFP_NET_VF_CFG_MB_UPD_LINK_STATE,
				    "link state");
}

int nfp_app_get_vf_config(struct net_device *netdev, int vf,
			  struct ifla_vf_info *ivi)
{
	struct nfp_app *app = nfp_app_from_netdev(netdev);
	unsigned int vf_offset;
	u16 vlan_tci;
	u32 mac_hi;
	u16 mac_lo;
	u8 flags;
	int err;

	err = nfp_net_sriov_check(app, vf, 0, "");
	if (err)
		return err;

	vf_offset = NFP_NET_VF_CFG_MB_SZ + vf * NFP_NET_VF_CFG_SZ;

	mac_hi = pete_readl("drivers/net/ethernet/netronome/nfp/nfp_net_sriov.c:224", app->pf->vfcfg_tbl2 + vf_offset);
	mac_lo = pete_readw("drivers/net/ethernet/netronome/nfp/nfp_net_sriov.c:225", app->pf->vfcfg_tbl2 + vf_offset + NFP_NET_VF_CFG_MAC_LO);

	flags = pete_readb("drivers/net/ethernet/netronome/nfp/nfp_net_sriov.c:227", app->pf->vfcfg_tbl2 + vf_offset + NFP_NET_VF_CFG_CTRL);
	vlan_tci = pete_readw("drivers/net/ethernet/netronome/nfp/nfp_net_sriov.c:228", app->pf->vfcfg_tbl2 + vf_offset + NFP_NET_VF_CFG_VLAN);

	memset(ivi, 0, sizeof(*ivi));
	ivi->vf = vf;

	put_unaligned_be32(mac_hi, &ivi->mac[0]);
	put_unaligned_be16(mac_lo, &ivi->mac[4]);

	ivi->vlan = FIELD_GET(NFP_NET_VF_CFG_VLAN_VID, vlan_tci);
	ivi->qos = FIELD_GET(NFP_NET_VF_CFG_VLAN_QOS, vlan_tci);

	ivi->spoofchk = FIELD_GET(NFP_NET_VF_CFG_CTRL_SPOOF, flags);
	ivi->trusted = FIELD_GET(NFP_NET_VF_CFG_CTRL_TRUST, flags);
	ivi->linkstate = FIELD_GET(NFP_NET_VF_CFG_CTRL_LINK_STATE, flags);

	return 0;
}
