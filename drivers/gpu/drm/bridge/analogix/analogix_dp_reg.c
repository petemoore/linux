// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Analogix DP (Display port) core register interface driver.
 *
 * Copyright (C) 2012 Samsung Electronics Co., Ltd.
 * Author: Jingoo Han <jg1.han@samsung.com>
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/iopoll.h>

#include <drm/bridge/analogix_dp.h>

#include "analogix_dp_core.h"
#include "analogix_dp_reg.h"

#define COMMON_INT_MASK_1	0
#define COMMON_INT_MASK_2	0
#define COMMON_INT_MASK_3	0
#define COMMON_INT_MASK_4	(HOTPLUG_CHG | HPD_LOST | PLUG)
#define INT_STA_MASK		INT_HPD

void analogix_dp_enable_video_mute(struct analogix_dp_device *dp, bool enable)
{
	u32 reg;

	if (enable) {
		reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:31", dp->reg_base + ANALOGIX_DP_VIDEO_CTL_1);
		reg |= HDCP_VIDEO_MUTE;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:33", reg, dp->reg_base + ANALOGIX_DP_VIDEO_CTL_1);
	} else {
		reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:35", dp->reg_base + ANALOGIX_DP_VIDEO_CTL_1);
		reg &= ~HDCP_VIDEO_MUTE;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:37", reg, dp->reg_base + ANALOGIX_DP_VIDEO_CTL_1);
	}
}

void analogix_dp_stop_video(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:45", dp->reg_base + ANALOGIX_DP_VIDEO_CTL_1);
	reg &= ~VIDEO_EN;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:47", reg, dp->reg_base + ANALOGIX_DP_VIDEO_CTL_1);
}

void analogix_dp_lane_swap(struct analogix_dp_device *dp, bool enable)
{
	u32 reg;

	if (enable)
		reg = LANE3_MAP_LOGIC_LANE_0 | LANE2_MAP_LOGIC_LANE_1 |
		      LANE1_MAP_LOGIC_LANE_2 | LANE0_MAP_LOGIC_LANE_3;
	else
		reg = LANE3_MAP_LOGIC_LANE_3 | LANE2_MAP_LOGIC_LANE_2 |
		      LANE1_MAP_LOGIC_LANE_1 | LANE0_MAP_LOGIC_LANE_0;

	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:61", reg, dp->reg_base + ANALOGIX_DP_LANE_MAP);
}

void analogix_dp_init_analog_param(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = TX_TERMINAL_CTRL_50_OHM;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:69", reg, dp->reg_base + ANALOGIX_DP_ANALOG_CTL_1);

	reg = SEL_24M | TX_DVDD_BIT_1_0625V;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:72", reg, dp->reg_base + ANALOGIX_DP_ANALOG_CTL_2);

	if (dp->plat_data && is_rockchip(dp->plat_data->dev_type)) {
		reg = REF_CLK_24M;
		if (dp->plat_data->dev_type == RK3288_DP)
			reg ^= REF_CLK_MASK;

		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:79", reg, dp->reg_base + ANALOGIX_DP_PLL_REG_1);
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:80", 0x95, dp->reg_base + ANALOGIX_DP_PLL_REG_2);
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:81", 0x40, dp->reg_base + ANALOGIX_DP_PLL_REG_3);
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:82", 0x58, dp->reg_base + ANALOGIX_DP_PLL_REG_4);
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:83", 0x22, dp->reg_base + ANALOGIX_DP_PLL_REG_5);
	}

	reg = DRIVE_DVDD_BIT_1_0625V | VCO_BIT_600_MICRO;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:87", reg, dp->reg_base + ANALOGIX_DP_ANALOG_CTL_3);

	reg = PD_RING_OSC | AUX_TERMINAL_CTRL_50_OHM |
		TX_CUR1_2X | TX_CUR_16_MA;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:91", reg, dp->reg_base + ANALOGIX_DP_PLL_FILTER_CTL_1);

	reg = CH3_AMP_400_MV | CH2_AMP_400_MV |
		CH1_AMP_400_MV | CH0_AMP_400_MV;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:95", reg, dp->reg_base + ANALOGIX_DP_TX_AMP_TUNING_CTL);
}

void analogix_dp_init_interrupt(struct analogix_dp_device *dp)
{
	/* Set interrupt pin assertion polarity as high */
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:101", INT_POL1 | INT_POL0, dp->reg_base + ANALOGIX_DP_INT_CTL);

	/* Clear pending regisers */
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:104", 0xff, dp->reg_base + ANALOGIX_DP_COMMON_INT_STA_1);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:105", 0x4f, dp->reg_base + ANALOGIX_DP_COMMON_INT_STA_2);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:106", 0xe0, dp->reg_base + ANALOGIX_DP_COMMON_INT_STA_3);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:107", 0xe7, dp->reg_base + ANALOGIX_DP_COMMON_INT_STA_4);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:108", 0x63, dp->reg_base + ANALOGIX_DP_INT_STA);

	/* 0:mask,1: unmask */
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:111", 0x00, dp->reg_base + ANALOGIX_DP_COMMON_INT_MASK_1);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:112", 0x00, dp->reg_base + ANALOGIX_DP_COMMON_INT_MASK_2);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:113", 0x00, dp->reg_base + ANALOGIX_DP_COMMON_INT_MASK_3);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:114", 0x00, dp->reg_base + ANALOGIX_DP_COMMON_INT_MASK_4);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:115", 0x00, dp->reg_base + ANALOGIX_DP_INT_STA_MASK);
}

void analogix_dp_reset(struct analogix_dp_device *dp)
{
	u32 reg;

	analogix_dp_stop_video(dp);
	analogix_dp_enable_video_mute(dp, 0);

	if (dp->plat_data && is_rockchip(dp->plat_data->dev_type))
		reg = RK_VID_CAP_FUNC_EN_N | RK_VID_FIFO_FUNC_EN_N |
			SW_FUNC_EN_N;
	else
		reg = MASTER_VID_FUNC_EN_N | SLAVE_VID_FUNC_EN_N |
			AUD_FIFO_FUNC_EN_N | AUD_FUNC_EN_N |
			HDCP_FUNC_EN_N | SW_FUNC_EN_N;

	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:133", reg, dp->reg_base + ANALOGIX_DP_FUNC_EN_1);

	reg = SSC_FUNC_EN_N | AUX_FUNC_EN_N |
		SERDES_FIFO_FUNC_EN_N |
		LS_CLK_DOMAIN_FUNC_EN_N;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:138", reg, dp->reg_base + ANALOGIX_DP_FUNC_EN_2);

	usleep_range(20, 30);

	analogix_dp_lane_swap(dp, 0);

	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:144", 0x0, dp->reg_base + ANALOGIX_DP_SYS_CTL_1);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:145", 0x40, dp->reg_base + ANALOGIX_DP_SYS_CTL_2);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:146", 0x0, dp->reg_base + ANALOGIX_DP_SYS_CTL_3);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:147", 0x0, dp->reg_base + ANALOGIX_DP_SYS_CTL_4);

	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:149", 0x0, dp->reg_base + ANALOGIX_DP_PKT_SEND_CTL);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:150", 0x0, dp->reg_base + ANALOGIX_DP_HDCP_CTL);

	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:152", 0x5e, dp->reg_base + ANALOGIX_DP_HPD_DEGLITCH_L);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:153", 0x1a, dp->reg_base + ANALOGIX_DP_HPD_DEGLITCH_H);

	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:155", 0x10, dp->reg_base + ANALOGIX_DP_LINK_DEBUG_CTL);

	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:157", 0x0, dp->reg_base + ANALOGIX_DP_PHY_TEST);

	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:159", 0x0, dp->reg_base + ANALOGIX_DP_VIDEO_FIFO_THRD);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:160", 0x20, dp->reg_base + ANALOGIX_DP_AUDIO_MARGIN);

	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:162", 0x4, dp->reg_base + ANALOGIX_DP_M_VID_GEN_FILTER_TH);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:163", 0x2, dp->reg_base + ANALOGIX_DP_M_AUD_GEN_FILTER_TH);

	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:165", 0x00000101, dp->reg_base + ANALOGIX_DP_SOC_GENERAL_CTL);
}

void analogix_dp_swreset(struct analogix_dp_device *dp)
{
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:170", RESET_DP_TX, dp->reg_base + ANALOGIX_DP_TX_SW_RESET);
}

void analogix_dp_config_interrupt(struct analogix_dp_device *dp)
{
	u32 reg;

	/* 0: mask, 1: unmask */
	reg = COMMON_INT_MASK_1;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:179", reg, dp->reg_base + ANALOGIX_DP_COMMON_INT_MASK_1);

	reg = COMMON_INT_MASK_2;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:182", reg, dp->reg_base + ANALOGIX_DP_COMMON_INT_MASK_2);

	reg = COMMON_INT_MASK_3;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:185", reg, dp->reg_base + ANALOGIX_DP_COMMON_INT_MASK_3);

	reg = COMMON_INT_MASK_4;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:188", reg, dp->reg_base + ANALOGIX_DP_COMMON_INT_MASK_4);

	reg = INT_STA_MASK;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:191", reg, dp->reg_base + ANALOGIX_DP_INT_STA_MASK);
}

void analogix_dp_mute_hpd_interrupt(struct analogix_dp_device *dp)
{
	u32 reg;

	/* 0: mask, 1: unmask */
	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:199", dp->reg_base + ANALOGIX_DP_COMMON_INT_MASK_4);
	reg &= ~COMMON_INT_MASK_4;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:201", reg, dp->reg_base + ANALOGIX_DP_COMMON_INT_MASK_4);

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:203", dp->reg_base + ANALOGIX_DP_INT_STA_MASK);
	reg &= ~INT_STA_MASK;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:205", reg, dp->reg_base + ANALOGIX_DP_INT_STA_MASK);
}

void analogix_dp_unmute_hpd_interrupt(struct analogix_dp_device *dp)
{
	u32 reg;

	/* 0: mask, 1: unmask */
	reg = COMMON_INT_MASK_4;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:214", reg, dp->reg_base + ANALOGIX_DP_COMMON_INT_MASK_4);

	reg = INT_STA_MASK;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:217", reg, dp->reg_base + ANALOGIX_DP_INT_STA_MASK);
}

enum pll_status analogix_dp_get_pll_lock_status(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:224", dp->reg_base + ANALOGIX_DP_DEBUG_CTL);
	if (reg & PLL_LOCK)
		return PLL_LOCKED;
	else
		return PLL_UNLOCKED;
}

void analogix_dp_set_pll_power_down(struct analogix_dp_device *dp, bool enable)
{
	u32 reg;
	u32 mask = DP_PLL_PD;
	u32 pd_addr = ANALOGIX_DP_PLL_CTL;

	if (dp->plat_data && is_rockchip(dp->plat_data->dev_type)) {
		pd_addr = ANALOGIX_DP_PD;
		mask = RK_PLL_PD;
	}

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:242", dp->reg_base + pd_addr);
	if (enable)
		reg |= mask;
	else
		reg &= ~mask;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:247", reg, dp->reg_base + pd_addr);
}

void analogix_dp_set_analog_power_down(struct analogix_dp_device *dp,
				       enum analog_power_block block,
				       bool enable)
{
	u32 reg;
	u32 phy_pd_addr = ANALOGIX_DP_PHY_PD;
	u32 mask;

	if (dp->plat_data && is_rockchip(dp->plat_data->dev_type))
		phy_pd_addr = ANALOGIX_DP_PD;

	switch (block) {
	case AUX_BLOCK:
		if (dp->plat_data && is_rockchip(dp->plat_data->dev_type))
			mask = RK_AUX_PD;
		else
			mask = AUX_PD;

		reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:268", dp->reg_base + phy_pd_addr);
		if (enable)
			reg |= mask;
		else
			reg &= ~mask;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:273", reg, dp->reg_base + phy_pd_addr);
		break;
	case CH0_BLOCK:
		mask = CH0_PD;
		reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:277", dp->reg_base + phy_pd_addr);

		if (enable)
			reg |= mask;
		else
			reg &= ~mask;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:283", reg, dp->reg_base + phy_pd_addr);
		break;
	case CH1_BLOCK:
		mask = CH1_PD;
		reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:287", dp->reg_base + phy_pd_addr);

		if (enable)
			reg |= mask;
		else
			reg &= ~mask;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:293", reg, dp->reg_base + phy_pd_addr);
		break;
	case CH2_BLOCK:
		mask = CH2_PD;
		reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:297", dp->reg_base + phy_pd_addr);

		if (enable)
			reg |= mask;
		else
			reg &= ~mask;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:303", reg, dp->reg_base + phy_pd_addr);
		break;
	case CH3_BLOCK:
		mask = CH3_PD;
		reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:307", dp->reg_base + phy_pd_addr);

		if (enable)
			reg |= mask;
		else
			reg &= ~mask;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:313", reg, dp->reg_base + phy_pd_addr);
		break;
	case ANALOG_TOTAL:
		/*
		 * There is no bit named DP_PHY_PD, so We used DP_INC_BG
		 * to power off everything instead of DP_PHY_PD in
		 * Rockchip
		 */
		if (dp->plat_data && is_rockchip(dp->plat_data->dev_type))
			mask = DP_INC_BG;
		else
			mask = DP_PHY_PD;

		reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:326", dp->reg_base + phy_pd_addr);
		if (enable)
			reg |= mask;
		else
			reg &= ~mask;

		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:332", reg, dp->reg_base + phy_pd_addr);
		if (dp->plat_data && is_rockchip(dp->plat_data->dev_type))
			usleep_range(10, 15);
		break;
	case POWER_ALL:
		if (enable) {
			reg = DP_ALL_PD;
			pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:339", reg, dp->reg_base + phy_pd_addr);
		} else {
			reg = DP_ALL_PD;
			pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:342", reg, dp->reg_base + phy_pd_addr);
			usleep_range(10, 15);
			reg &= ~DP_INC_BG;
			pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:345", reg, dp->reg_base + phy_pd_addr);
			usleep_range(10, 15);

			pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:348", 0x00, dp->reg_base + phy_pd_addr);
		}
		break;
	default:
		break;
	}
}

int analogix_dp_init_analog_func(struct analogix_dp_device *dp)
{
	u32 reg;
	int timeout_loop = 0;

	analogix_dp_set_analog_power_down(dp, POWER_ALL, 0);

	reg = PLL_LOCK_CHG;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:364", reg, dp->reg_base + ANALOGIX_DP_COMMON_INT_STA_1);

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:366", dp->reg_base + ANALOGIX_DP_DEBUG_CTL);
	reg &= ~(F_PLL_LOCK | PLL_LOCK_CTRL);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:368", reg, dp->reg_base + ANALOGIX_DP_DEBUG_CTL);

	/* Power up PLL */
	if (analogix_dp_get_pll_lock_status(dp) == PLL_UNLOCKED) {
		analogix_dp_set_pll_power_down(dp, 0);

		while (analogix_dp_get_pll_lock_status(dp) == PLL_UNLOCKED) {
			timeout_loop++;
			if (DP_TIMEOUT_LOOP_COUNT < timeout_loop) {
				dev_err(dp->dev, "failed to get pll lock status\n");
				return -ETIMEDOUT;
			}
			usleep_range(10, 20);
		}
	}

	/* Enable Serdes FIFO function and Link symbol clock domain module */
	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:385", dp->reg_base + ANALOGIX_DP_FUNC_EN_2);
	reg &= ~(SERDES_FIFO_FUNC_EN_N | LS_CLK_DOMAIN_FUNC_EN_N
		| AUX_FUNC_EN_N);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:388", reg, dp->reg_base + ANALOGIX_DP_FUNC_EN_2);
	return 0;
}

void analogix_dp_clear_hotplug_interrupts(struct analogix_dp_device *dp)
{
	u32 reg;

	if (dp->hpd_gpiod)
		return;

	reg = HOTPLUG_CHG | HPD_LOST | PLUG;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:400", reg, dp->reg_base + ANALOGIX_DP_COMMON_INT_STA_4);

	reg = INT_HPD;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:403", reg, dp->reg_base + ANALOGIX_DP_INT_STA);
}

void analogix_dp_init_hpd(struct analogix_dp_device *dp)
{
	u32 reg;

	if (dp->hpd_gpiod)
		return;

	analogix_dp_clear_hotplug_interrupts(dp);

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:415", dp->reg_base + ANALOGIX_DP_SYS_CTL_3);
	reg &= ~(F_HPD | HPD_CTRL);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:417", reg, dp->reg_base + ANALOGIX_DP_SYS_CTL_3);
}

void analogix_dp_force_hpd(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:424", dp->reg_base + ANALOGIX_DP_SYS_CTL_3);
	reg = (F_HPD | HPD_CTRL);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:426", reg, dp->reg_base + ANALOGIX_DP_SYS_CTL_3);
}

enum dp_irq_type analogix_dp_get_irq_type(struct analogix_dp_device *dp)
{
	u32 reg;

	if (dp->hpd_gpiod) {
		reg = gpiod_get_value(dp->hpd_gpiod);
		if (reg)
			return DP_IRQ_TYPE_HP_CABLE_IN;
		else
			return DP_IRQ_TYPE_HP_CABLE_OUT;
	} else {
		/* Parse hotplug interrupt status register */
		reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:441", dp->reg_base + ANALOGIX_DP_COMMON_INT_STA_4);

		if (reg & PLUG)
			return DP_IRQ_TYPE_HP_CABLE_IN;

		if (reg & HPD_LOST)
			return DP_IRQ_TYPE_HP_CABLE_OUT;

		if (reg & HOTPLUG_CHG)
			return DP_IRQ_TYPE_HP_CHANGE;

		return DP_IRQ_TYPE_UNKNOWN;
	}
}

void analogix_dp_reset_aux(struct analogix_dp_device *dp)
{
	u32 reg;

	/* Disable AUX channel module */
	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:461", dp->reg_base + ANALOGIX_DP_FUNC_EN_2);
	reg |= AUX_FUNC_EN_N;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:463", reg, dp->reg_base + ANALOGIX_DP_FUNC_EN_2);
}

void analogix_dp_init_aux(struct analogix_dp_device *dp)
{
	u32 reg;

	/* Clear inerrupts related to AUX channel */
	reg = RPLY_RECEIV | AUX_ERR;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:472", reg, dp->reg_base + ANALOGIX_DP_INT_STA);

	analogix_dp_set_analog_power_down(dp, AUX_BLOCK, true);
	usleep_range(10, 11);
	analogix_dp_set_analog_power_down(dp, AUX_BLOCK, false);

	analogix_dp_reset_aux(dp);

	/* AUX_BIT_PERIOD_EXPECTED_DELAY doesn't apply to Rockchip IP */
	if (dp->plat_data && is_rockchip(dp->plat_data->dev_type))
		reg = 0;
	else
		reg = AUX_BIT_PERIOD_EXPECTED_DELAY(3);

	/* Disable AUX transaction H/W retry */
	reg |= AUX_HW_RETRY_COUNT_SEL(0) |
	       AUX_HW_RETRY_INTERVAL_600_MICROSECONDS;

	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:490", reg, dp->reg_base + ANALOGIX_DP_AUX_HW_RETRY_CTL);

	/* Receive AUX Channel DEFER commands equal to DEFFER_COUNT*64 */
	reg = DEFER_CTRL_EN | DEFER_COUNT(1);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:494", reg, dp->reg_base + ANALOGIX_DP_AUX_CH_DEFER_CTL);

	/* Enable AUX channel module */
	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:497", dp->reg_base + ANALOGIX_DP_FUNC_EN_2);
	reg &= ~AUX_FUNC_EN_N;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:499", reg, dp->reg_base + ANALOGIX_DP_FUNC_EN_2);
}

int analogix_dp_get_plug_in_status(struct analogix_dp_device *dp)
{
	u32 reg;

	if (dp->hpd_gpiod) {
		if (gpiod_get_value(dp->hpd_gpiod))
			return 0;
	} else {
		reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:510", dp->reg_base + ANALOGIX_DP_SYS_CTL_3);
		if (reg & HPD_STATUS)
			return 0;
	}

	return -EINVAL;
}

void analogix_dp_enable_sw_function(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:522", dp->reg_base + ANALOGIX_DP_FUNC_EN_1);
	reg &= ~SW_FUNC_EN_N;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:524", reg, dp->reg_base + ANALOGIX_DP_FUNC_EN_1);
}

void analogix_dp_set_link_bandwidth(struct analogix_dp_device *dp, u32 bwtype)
{
	u32 reg;

	reg = bwtype;
	if ((bwtype == DP_LINK_BW_2_7) || (bwtype == DP_LINK_BW_1_62))
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:533", reg, dp->reg_base + ANALOGIX_DP_LINK_BW_SET);
}

void analogix_dp_get_link_bandwidth(struct analogix_dp_device *dp, u32 *bwtype)
{
	u32 reg;

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:540", dp->reg_base + ANALOGIX_DP_LINK_BW_SET);
	*bwtype = reg;
}

void analogix_dp_set_lane_count(struct analogix_dp_device *dp, u32 count)
{
	u32 reg;

	reg = count;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:549", reg, dp->reg_base + ANALOGIX_DP_LANE_COUNT_SET);
}

void analogix_dp_get_lane_count(struct analogix_dp_device *dp, u32 *count)
{
	u32 reg;

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:556", dp->reg_base + ANALOGIX_DP_LANE_COUNT_SET);
	*count = reg;
}

void analogix_dp_enable_enhanced_mode(struct analogix_dp_device *dp,
				      bool enable)
{
	u32 reg;

	if (enable) {
		reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:566", dp->reg_base + ANALOGIX_DP_SYS_CTL_4);
		reg |= ENHANCED;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:568", reg, dp->reg_base + ANALOGIX_DP_SYS_CTL_4);
	} else {
		reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:570", dp->reg_base + ANALOGIX_DP_SYS_CTL_4);
		reg &= ~ENHANCED;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:572", reg, dp->reg_base + ANALOGIX_DP_SYS_CTL_4);
	}
}

void analogix_dp_set_training_pattern(struct analogix_dp_device *dp,
				      enum pattern_set pattern)
{
	u32 reg;

	switch (pattern) {
	case PRBS7:
		reg = SCRAMBLING_ENABLE | LINK_QUAL_PATTERN_SET_PRBS7;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:584", reg, dp->reg_base + ANALOGIX_DP_TRAINING_PTN_SET);
		break;
	case D10_2:
		reg = SCRAMBLING_ENABLE | LINK_QUAL_PATTERN_SET_D10_2;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:588", reg, dp->reg_base + ANALOGIX_DP_TRAINING_PTN_SET);
		break;
	case TRAINING_PTN1:
		reg = SCRAMBLING_DISABLE | SW_TRAINING_PATTERN_SET_PTN1;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:592", reg, dp->reg_base + ANALOGIX_DP_TRAINING_PTN_SET);
		break;
	case TRAINING_PTN2:
		reg = SCRAMBLING_DISABLE | SW_TRAINING_PATTERN_SET_PTN2;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:596", reg, dp->reg_base + ANALOGIX_DP_TRAINING_PTN_SET);
		break;
	case DP_NONE:
		reg = SCRAMBLING_ENABLE |
			LINK_QUAL_PATTERN_SET_DISABLE |
			SW_TRAINING_PATTERN_SET_NORMAL;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:602", reg, dp->reg_base + ANALOGIX_DP_TRAINING_PTN_SET);
		break;
	default:
		break;
	}
}

void analogix_dp_set_lane0_pre_emphasis(struct analogix_dp_device *dp,
					u32 level)
{
	u32 reg;

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:614", dp->reg_base + ANALOGIX_DP_LN0_LINK_TRAINING_CTL);
	reg &= ~PRE_EMPHASIS_SET_MASK;
	reg |= level << PRE_EMPHASIS_SET_SHIFT;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:617", reg, dp->reg_base + ANALOGIX_DP_LN0_LINK_TRAINING_CTL);
}

void analogix_dp_set_lane1_pre_emphasis(struct analogix_dp_device *dp,
					u32 level)
{
	u32 reg;

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:625", dp->reg_base + ANALOGIX_DP_LN1_LINK_TRAINING_CTL);
	reg &= ~PRE_EMPHASIS_SET_MASK;
	reg |= level << PRE_EMPHASIS_SET_SHIFT;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:628", reg, dp->reg_base + ANALOGIX_DP_LN1_LINK_TRAINING_CTL);
}

void analogix_dp_set_lane2_pre_emphasis(struct analogix_dp_device *dp,
					u32 level)
{
	u32 reg;

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:636", dp->reg_base + ANALOGIX_DP_LN2_LINK_TRAINING_CTL);
	reg &= ~PRE_EMPHASIS_SET_MASK;
	reg |= level << PRE_EMPHASIS_SET_SHIFT;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:639", reg, dp->reg_base + ANALOGIX_DP_LN2_LINK_TRAINING_CTL);
}

void analogix_dp_set_lane3_pre_emphasis(struct analogix_dp_device *dp,
					u32 level)
{
	u32 reg;

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:647", dp->reg_base + ANALOGIX_DP_LN3_LINK_TRAINING_CTL);
	reg &= ~PRE_EMPHASIS_SET_MASK;
	reg |= level << PRE_EMPHASIS_SET_SHIFT;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:650", reg, dp->reg_base + ANALOGIX_DP_LN3_LINK_TRAINING_CTL);
}

void analogix_dp_set_lane0_link_training(struct analogix_dp_device *dp,
					 u32 training_lane)
{
	u32 reg;

	reg = training_lane;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:659", reg, dp->reg_base + ANALOGIX_DP_LN0_LINK_TRAINING_CTL);
}

void analogix_dp_set_lane1_link_training(struct analogix_dp_device *dp,
					 u32 training_lane)
{
	u32 reg;

	reg = training_lane;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:668", reg, dp->reg_base + ANALOGIX_DP_LN1_LINK_TRAINING_CTL);
}

void analogix_dp_set_lane2_link_training(struct analogix_dp_device *dp,
					 u32 training_lane)
{
	u32 reg;

	reg = training_lane;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:677", reg, dp->reg_base + ANALOGIX_DP_LN2_LINK_TRAINING_CTL);
}

void analogix_dp_set_lane3_link_training(struct analogix_dp_device *dp,
					 u32 training_lane)
{
	u32 reg;

	reg = training_lane;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:686", reg, dp->reg_base + ANALOGIX_DP_LN3_LINK_TRAINING_CTL);
}

u32 analogix_dp_get_lane0_link_training(struct analogix_dp_device *dp)
{
	return pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:691", dp->reg_base + ANALOGIX_DP_LN0_LINK_TRAINING_CTL);
}

u32 analogix_dp_get_lane1_link_training(struct analogix_dp_device *dp)
{
	return pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:696", dp->reg_base + ANALOGIX_DP_LN1_LINK_TRAINING_CTL);
}

u32 analogix_dp_get_lane2_link_training(struct analogix_dp_device *dp)
{
	return pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:701", dp->reg_base + ANALOGIX_DP_LN2_LINK_TRAINING_CTL);
}

u32 analogix_dp_get_lane3_link_training(struct analogix_dp_device *dp)
{
	return pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:706", dp->reg_base + ANALOGIX_DP_LN3_LINK_TRAINING_CTL);
}

void analogix_dp_reset_macro(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:713", dp->reg_base + ANALOGIX_DP_PHY_TEST);
	reg |= MACRO_RST;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:715", reg, dp->reg_base + ANALOGIX_DP_PHY_TEST);

	/* 10 us is the minimum reset time. */
	usleep_range(10, 20);

	reg &= ~MACRO_RST;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:721", reg, dp->reg_base + ANALOGIX_DP_PHY_TEST);
}

void analogix_dp_init_video(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = VSYNC_DET | VID_FORMAT_CHG | VID_CLK_CHG;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:729", reg, dp->reg_base + ANALOGIX_DP_COMMON_INT_STA_1);

	reg = 0x0;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:732", reg, dp->reg_base + ANALOGIX_DP_SYS_CTL_1);

	reg = CHA_CRI(4) | CHA_CTRL;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:735", reg, dp->reg_base + ANALOGIX_DP_SYS_CTL_2);

	reg = 0x0;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:738", reg, dp->reg_base + ANALOGIX_DP_SYS_CTL_3);

	reg = VID_HRES_TH(2) | VID_VRES_TH(0);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:741", reg, dp->reg_base + ANALOGIX_DP_VIDEO_CTL_8);
}

void analogix_dp_set_video_color_format(struct analogix_dp_device *dp)
{
	u32 reg;

	/* Configure the input color depth, color space, dynamic range */
	reg = (dp->video_info.dynamic_range << IN_D_RANGE_SHIFT) |
		(dp->video_info.color_depth << IN_BPC_SHIFT) |
		(dp->video_info.color_space << IN_COLOR_F_SHIFT);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:752", reg, dp->reg_base + ANALOGIX_DP_VIDEO_CTL_2);

	/* Set Input Color YCbCr Coefficients to ITU601 or ITU709 */
	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:755", dp->reg_base + ANALOGIX_DP_VIDEO_CTL_3);
	reg &= ~IN_YC_COEFFI_MASK;
	if (dp->video_info.ycbcr_coeff)
		reg |= IN_YC_COEFFI_ITU709;
	else
		reg |= IN_YC_COEFFI_ITU601;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:761", reg, dp->reg_base + ANALOGIX_DP_VIDEO_CTL_3);
}

int analogix_dp_is_slave_video_stream_clock_on(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:768", dp->reg_base + ANALOGIX_DP_SYS_CTL_1);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:769", reg, dp->reg_base + ANALOGIX_DP_SYS_CTL_1);

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:771", dp->reg_base + ANALOGIX_DP_SYS_CTL_1);

	if (!(reg & DET_STA)) {
		dev_dbg(dp->dev, "Input stream clock not detected.\n");
		return -EINVAL;
	}

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:778", dp->reg_base + ANALOGIX_DP_SYS_CTL_2);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:779", reg, dp->reg_base + ANALOGIX_DP_SYS_CTL_2);

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:781", dp->reg_base + ANALOGIX_DP_SYS_CTL_2);
	dev_dbg(dp->dev, "wait SYS_CTL_2.\n");

	if (reg & CHA_STA) {
		dev_dbg(dp->dev, "Input stream clk is changing\n");
		return -EINVAL;
	}

	return 0;
}

void analogix_dp_set_video_cr_mn(struct analogix_dp_device *dp,
				 enum clock_recovery_m_value_type type,
				 u32 m_value, u32 n_value)
{
	u32 reg;

	if (type == REGISTER_M) {
		reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:799", dp->reg_base + ANALOGIX_DP_SYS_CTL_4);
		reg |= FIX_M_VID;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:801", reg, dp->reg_base + ANALOGIX_DP_SYS_CTL_4);
		reg = m_value & 0xff;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:803", reg, dp->reg_base + ANALOGIX_DP_M_VID_0);
		reg = (m_value >> 8) & 0xff;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:805", reg, dp->reg_base + ANALOGIX_DP_M_VID_1);
		reg = (m_value >> 16) & 0xff;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:807", reg, dp->reg_base + ANALOGIX_DP_M_VID_2);

		reg = n_value & 0xff;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:810", reg, dp->reg_base + ANALOGIX_DP_N_VID_0);
		reg = (n_value >> 8) & 0xff;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:812", reg, dp->reg_base + ANALOGIX_DP_N_VID_1);
		reg = (n_value >> 16) & 0xff;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:814", reg, dp->reg_base + ANALOGIX_DP_N_VID_2);
	} else  {
		reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:816", dp->reg_base + ANALOGIX_DP_SYS_CTL_4);
		reg &= ~FIX_M_VID;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:818", reg, dp->reg_base + ANALOGIX_DP_SYS_CTL_4);

		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:820", 0x00, dp->reg_base + ANALOGIX_DP_N_VID_0);
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:821", 0x80, dp->reg_base + ANALOGIX_DP_N_VID_1);
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:822", 0x00, dp->reg_base + ANALOGIX_DP_N_VID_2);
	}
}

void analogix_dp_set_video_timing_mode(struct analogix_dp_device *dp, u32 type)
{
	u32 reg;

	if (type == VIDEO_TIMING_FROM_CAPTURE) {
		reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:831", dp->reg_base + ANALOGIX_DP_VIDEO_CTL_10);
		reg &= ~FORMAT_SEL;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:833", reg, dp->reg_base + ANALOGIX_DP_VIDEO_CTL_10);
	} else {
		reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:835", dp->reg_base + ANALOGIX_DP_VIDEO_CTL_10);
		reg |= FORMAT_SEL;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:837", reg, dp->reg_base + ANALOGIX_DP_VIDEO_CTL_10);
	}
}

void analogix_dp_enable_video_master(struct analogix_dp_device *dp, bool enable)
{
	u32 reg;

	if (enable) {
		reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:846", dp->reg_base + ANALOGIX_DP_SOC_GENERAL_CTL);
		reg &= ~VIDEO_MODE_MASK;
		reg |= VIDEO_MASTER_MODE_EN | VIDEO_MODE_MASTER_MODE;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:849", reg, dp->reg_base + ANALOGIX_DP_SOC_GENERAL_CTL);
	} else {
		reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:851", dp->reg_base + ANALOGIX_DP_SOC_GENERAL_CTL);
		reg &= ~VIDEO_MODE_MASK;
		reg |= VIDEO_MODE_SLAVE_MODE;
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:854", reg, dp->reg_base + ANALOGIX_DP_SOC_GENERAL_CTL);
	}
}

void analogix_dp_start_video(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:862", dp->reg_base + ANALOGIX_DP_VIDEO_CTL_1);
	reg |= VIDEO_EN;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:864", reg, dp->reg_base + ANALOGIX_DP_VIDEO_CTL_1);
}

int analogix_dp_is_video_stream_on(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:871", dp->reg_base + ANALOGIX_DP_SYS_CTL_3);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:872", reg, dp->reg_base + ANALOGIX_DP_SYS_CTL_3);

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:874", dp->reg_base + ANALOGIX_DP_SYS_CTL_3);
	if (!(reg & STRM_VALID)) {
		dev_dbg(dp->dev, "Input video stream is not detected.\n");
		return -EINVAL;
	}

	return 0;
}

void analogix_dp_config_video_slave_mode(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:887", dp->reg_base + ANALOGIX_DP_FUNC_EN_1);
	if (dp->plat_data && is_rockchip(dp->plat_data->dev_type)) {
		reg &= ~(RK_VID_CAP_FUNC_EN_N | RK_VID_FIFO_FUNC_EN_N);
	} else {
		reg &= ~(MASTER_VID_FUNC_EN_N | SLAVE_VID_FUNC_EN_N);
		reg |= MASTER_VID_FUNC_EN_N;
	}
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:894", reg, dp->reg_base + ANALOGIX_DP_FUNC_EN_1);

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:896", dp->reg_base + ANALOGIX_DP_VIDEO_CTL_10);
	reg &= ~INTERACE_SCAN_CFG;
	reg |= (dp->video_info.interlaced << 2);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:899", reg, dp->reg_base + ANALOGIX_DP_VIDEO_CTL_10);

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:901", dp->reg_base + ANALOGIX_DP_VIDEO_CTL_10);
	reg &= ~VSYNC_POLARITY_CFG;
	reg |= (dp->video_info.v_sync_polarity << 1);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:904", reg, dp->reg_base + ANALOGIX_DP_VIDEO_CTL_10);

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:906", dp->reg_base + ANALOGIX_DP_VIDEO_CTL_10);
	reg &= ~HSYNC_POLARITY_CFG;
	reg |= (dp->video_info.h_sync_polarity << 0);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:909", reg, dp->reg_base + ANALOGIX_DP_VIDEO_CTL_10);

	reg = AUDIO_MODE_SPDIF_MODE | VIDEO_MODE_SLAVE_MODE;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:912", reg, dp->reg_base + ANALOGIX_DP_SOC_GENERAL_CTL);
}

void analogix_dp_enable_scrambling(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:919", dp->reg_base + ANALOGIX_DP_TRAINING_PTN_SET);
	reg &= ~SCRAMBLING_DISABLE;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:921", reg, dp->reg_base + ANALOGIX_DP_TRAINING_PTN_SET);
}

void analogix_dp_disable_scrambling(struct analogix_dp_device *dp)
{
	u32 reg;

	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:928", dp->reg_base + ANALOGIX_DP_TRAINING_PTN_SET);
	reg |= SCRAMBLING_DISABLE;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:930", reg, dp->reg_base + ANALOGIX_DP_TRAINING_PTN_SET);
}

void analogix_dp_enable_psr_crc(struct analogix_dp_device *dp)
{
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:935", PSR_VID_CRC_ENABLE, dp->reg_base + ANALOGIX_DP_CRC_CON);
}

static ssize_t analogix_dp_get_psr_status(struct analogix_dp_device *dp)
{
	ssize_t val;
	u8 status;

	val = drm_dp_dpcd_readb(&dp->aux, DP_PSR_STATUS, &status);
	if (val < 0) {
		dev_err(dp->dev, "PSR_STATUS read failed ret=%zd", val);
		return val;
	}
	return status;
}

int analogix_dp_send_psr_spd(struct analogix_dp_device *dp,
			     struct dp_sdp *vsc, bool blocking)
{
	unsigned int val;
	int ret;
	ssize_t psr_status;

	/* don't send info frame */
	val = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:959", dp->reg_base + ANALOGIX_DP_PKT_SEND_CTL);
	val &= ~IF_EN;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:961", val, dp->reg_base + ANALOGIX_DP_PKT_SEND_CTL);

	/* configure single frame update mode */
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:964", PSR_FRAME_UP_TYPE_BURST | PSR_CRC_SEL_HARDWARE,
	       dp->reg_base + ANALOGIX_DP_PSR_FRAME_UPDATE_CTRL);

	/* configure VSC HB0~HB3 */
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:968", vsc->sdp_header.HB0, dp->reg_base + ANALOGIX_DP_SPD_HB0);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:969", vsc->sdp_header.HB1, dp->reg_base + ANALOGIX_DP_SPD_HB1);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:970", vsc->sdp_header.HB2, dp->reg_base + ANALOGIX_DP_SPD_HB2);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:971", vsc->sdp_header.HB3, dp->reg_base + ANALOGIX_DP_SPD_HB3);

	/* configure reused VSC PB0~PB3, magic number from vendor */
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:974", 0x00, dp->reg_base + ANALOGIX_DP_SPD_PB0);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:975", 0x16, dp->reg_base + ANALOGIX_DP_SPD_PB1);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:976", 0xCE, dp->reg_base + ANALOGIX_DP_SPD_PB2);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:977", 0x5D, dp->reg_base + ANALOGIX_DP_SPD_PB3);

	/* configure DB0 / DB1 values */
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:980", vsc->db[0], dp->reg_base + ANALOGIX_DP_VSC_SHADOW_DB0);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:981", vsc->db[1], dp->reg_base + ANALOGIX_DP_VSC_SHADOW_DB1);

	/* set reuse spd inforframe */
	val = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:984", dp->reg_base + ANALOGIX_DP_VIDEO_CTL_3);
	val |= REUSE_SPD_EN;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:986", val, dp->reg_base + ANALOGIX_DP_VIDEO_CTL_3);

	/* mark info frame update */
	val = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:989", dp->reg_base + ANALOGIX_DP_PKT_SEND_CTL);
	val = (val | IF_UP) & ~IF_EN;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:991", val, dp->reg_base + ANALOGIX_DP_PKT_SEND_CTL);

	/* send info frame */
	val = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:994", dp->reg_base + ANALOGIX_DP_PKT_SEND_CTL);
	val |= IF_EN;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:996", val, dp->reg_base + ANALOGIX_DP_PKT_SEND_CTL);

	if (!blocking)
		return 0;

	/*
	 * db[1]!=0: entering PSR, wait for fully active remote frame buffer.
	 * db[1]==0: exiting PSR, wait for either
	 *  (a) ACTIVE_RESYNC - the sink "must display the
	 *      incoming active frames from the Source device with no visible
	 *      glitches and/or artifacts", even though timings may still be
	 *      re-synchronizing; or
	 *  (b) INACTIVE - the transition is fully complete.
	 */
	ret = readx_poll_timeout(analogix_dp_get_psr_status, dp, psr_status,
		psr_status >= 0 &&
		((vsc->db[1] && psr_status == DP_PSR_SINK_ACTIVE_RFB) ||
		(!vsc->db[1] && (psr_status == DP_PSR_SINK_ACTIVE_RESYNC ||
				 psr_status == DP_PSR_SINK_INACTIVE))),
		1500, DP_TIMEOUT_PSR_LOOP_MS * 1000);
	if (ret) {
		dev_warn(dp->dev, "Failed to apply PSR %d\n", ret);
		return ret;
	}
	return 0;
}

ssize_t analogix_dp_transfer(struct analogix_dp_device *dp,
			     struct drm_dp_aux_msg *msg)
{
	u32 reg;
	u32 status_reg;
	u8 *buffer = msg->buffer;
	unsigned int i;
	int num_transferred = 0;
	int ret;

	/* Buffer size of AUX CH is 16 bytes */
	if (WARN_ON(msg->size > 16))
		return -E2BIG;

	/* Clear AUX CH data buffer */
	reg = BUF_CLR;
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:1039", reg, dp->reg_base + ANALOGIX_DP_BUFFER_DATA_CTL);

	switch (msg->request & ~DP_AUX_I2C_MOT) {
	case DP_AUX_I2C_WRITE:
		reg = AUX_TX_COMM_WRITE | AUX_TX_COMM_I2C_TRANSACTION;
		if (msg->request & DP_AUX_I2C_MOT)
			reg |= AUX_TX_COMM_MOT;
		break;

	case DP_AUX_I2C_READ:
		reg = AUX_TX_COMM_READ | AUX_TX_COMM_I2C_TRANSACTION;
		if (msg->request & DP_AUX_I2C_MOT)
			reg |= AUX_TX_COMM_MOT;
		break;

	case DP_AUX_NATIVE_WRITE:
		reg = AUX_TX_COMM_WRITE | AUX_TX_COMM_DP_TRANSACTION;
		break;

	case DP_AUX_NATIVE_READ:
		reg = AUX_TX_COMM_READ | AUX_TX_COMM_DP_TRANSACTION;
		break;

	default:
		return -EINVAL;
	}

	reg |= AUX_LENGTH(msg->size);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:1067", reg, dp->reg_base + ANALOGIX_DP_AUX_CH_CTL_1);

	/* Select DPCD device address */
	reg = AUX_ADDR_7_0(msg->address);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:1071", reg, dp->reg_base + ANALOGIX_DP_AUX_ADDR_7_0);
	reg = AUX_ADDR_15_8(msg->address);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:1073", reg, dp->reg_base + ANALOGIX_DP_AUX_ADDR_15_8);
	reg = AUX_ADDR_19_16(msg->address);
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:1075", reg, dp->reg_base + ANALOGIX_DP_AUX_ADDR_19_16);

	if (!(msg->request & DP_AUX_I2C_READ)) {
		for (i = 0; i < msg->size; i++) {
			reg = buffer[i];
			pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:1080", reg, dp->reg_base + ANALOGIX_DP_BUF_DATA_0 +
			       4 * i);
			num_transferred++;
		}
	}

	/* Enable AUX CH operation */
	reg = AUX_EN;

	/* Zero-sized messages specify address-only transactions. */
	if (msg->size < 1)
		reg |= ADDR_ONLY;

	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:1093", reg, dp->reg_base + ANALOGIX_DP_AUX_CH_CTL_2);

	ret = readx_poll_timeout(readl, dp->reg_base + ANALOGIX_DP_AUX_CH_CTL_2,
				 reg, !(reg & AUX_EN), 25, 500 * 1000);
	if (ret) {
		dev_err(dp->dev, "AUX CH enable timeout!\n");
		goto aux_error;
	}

	/* TODO: Wait for an interrupt instead of looping? */
	/* Is AUX CH command reply received? */
	ret = readx_poll_timeout(readl, dp->reg_base + ANALOGIX_DP_INT_STA,
				 reg, reg & RPLY_RECEIV, 10, 20 * 1000);
	if (ret) {
		dev_err(dp->dev, "AUX CH cmd reply timeout!\n");
		goto aux_error;
	}

	/* Clear interrupt source for AUX CH command reply */
	pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:1112", RPLY_RECEIV, dp->reg_base + ANALOGIX_DP_INT_STA);

	/* Clear interrupt source for AUX CH access error */
	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:1115", dp->reg_base + ANALOGIX_DP_INT_STA);
	status_reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:1116", dp->reg_base + ANALOGIX_DP_AUX_CH_STA);
	if ((reg & AUX_ERR) || (status_reg & AUX_STATUS_MASK)) {
		pete_writel("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:1118", AUX_ERR, dp->reg_base + ANALOGIX_DP_INT_STA);

		dev_warn(dp->dev, "AUX CH error happened: %#x (%d)\n",
			 status_reg & AUX_STATUS_MASK, !!(reg & AUX_ERR));
		goto aux_error;
	}

	if (msg->request & DP_AUX_I2C_READ) {
		for (i = 0; i < msg->size; i++) {
			reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:1127", dp->reg_base + ANALOGIX_DP_BUF_DATA_0 +
				    4 * i);
			buffer[i] = (unsigned char)reg;
			num_transferred++;
		}
	}

	/* Check if Rx sends defer */
	reg = pete_readl("drivers/gpu/drm/bridge/analogix/analogix_dp_reg.c:1135", dp->reg_base + ANALOGIX_DP_AUX_RX_COMM);
	if (reg == AUX_RX_COMM_AUX_DEFER)
		msg->reply = DP_AUX_NATIVE_REPLY_DEFER;
	else if (reg == AUX_RX_COMM_I2C_DEFER)
		msg->reply = DP_AUX_I2C_REPLY_DEFER;
	else if ((msg->request & ~DP_AUX_I2C_MOT) == DP_AUX_I2C_WRITE ||
		 (msg->request & ~DP_AUX_I2C_MOT) == DP_AUX_I2C_READ)
		msg->reply = DP_AUX_I2C_REPLY_ACK;
	else if ((msg->request & ~DP_AUX_I2C_MOT) == DP_AUX_NATIVE_WRITE ||
		 (msg->request & ~DP_AUX_I2C_MOT) == DP_AUX_NATIVE_READ)
		msg->reply = DP_AUX_NATIVE_REPLY_ACK;

	return num_transferred > 0 ? num_transferred : -EBUSY;

aux_error:
	/* if aux err happen, reset aux */
	analogix_dp_init_aux(dp);

	return -EREMOTEIO;
}
