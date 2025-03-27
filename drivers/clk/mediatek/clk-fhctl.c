// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2022 MediaTek Inc.
 * Author: Edward-JW Yang <edward-jw.yang@mediatek.com>
 */

#include <linux/io.h>
#include <linux/iopoll.h>

#include "clk-mtk.h"
#include "clk-pllfh.h"
#include "clk-fhctl.h"

#define PERCENT_TO_DDSLMT(dds, percent_m10) \
	((((dds) * (percent_m10)) >> 5) / 100)

static const struct fhctl_offset fhctl_offset_v1 = {
	.offset_hp_en = 0x0,
	.offset_clk_con = 0x4,
	.offset_rst_con = 0x8,
	.offset_slope0 = 0xc,
	.offset_slope1 = 0x10,
	.offset_cfg = 0x0,
	.offset_updnlmt = 0x4,
	.offset_dds = 0x8,
	.offset_dvfs = 0xc,
	.offset_mon = 0x10,
};

static const struct fhctl_offset fhctl_offset_v2 = {
	.offset_hp_en = 0x0,
	.offset_clk_con = 0x8,
	.offset_rst_con = 0xc,
	.offset_slope0 = 0x10,
	.offset_slope1 = 0x14,
	.offset_cfg = 0x0,
	.offset_updnlmt = 0x4,
	.offset_dds = 0x8,
	.offset_dvfs = 0xc,
	.offset_mon = 0x10,
};

const struct fhctl_offset *fhctl_get_offset_table(enum fhctl_variant v)
{
	switch (v) {
	case FHCTL_PLLFH_V1:
		return &fhctl_offset_v1;
	case FHCTL_PLLFH_V2:
		return &fhctl_offset_v2;
	default:
		return ERR_PTR(-EINVAL);
	};
}

static void dump_hw(struct mtk_clk_pll *pll, struct fh_pll_regs *regs,
		    const struct fh_pll_data *data)
{
	pr_info("hp_en<%x>,clk_con<%x>,slope0<%x>,slope1<%x>\n",
		pete_readl("drivers/clk/mediatek/clk-fhctl.c:59", regs->reg_hp_en), pete_readl("drivers/clk/mediatek/clk-fhctl.c:59", regs->reg_clk_con),
		pete_readl("drivers/clk/mediatek/clk-fhctl.c:60", regs->reg_slope0), pete_readl("drivers/clk/mediatek/clk-fhctl.c:60", regs->reg_slope1));
	pr_info("cfg<%x>,lmt<%x>,dds<%x>,dvfs<%x>,mon<%x>\n",
		pete_readl("drivers/clk/mediatek/clk-fhctl.c:62", regs->reg_cfg), pete_readl("drivers/clk/mediatek/clk-fhctl.c:62", regs->reg_updnlmt),
		pete_readl("drivers/clk/mediatek/clk-fhctl.c:63", regs->reg_dds), pete_readl("drivers/clk/mediatek/clk-fhctl.c:63", regs->reg_dvfs),
		pete_readl("drivers/clk/mediatek/clk-fhctl.c:64", regs->reg_mon));
	pr_info("pcw<%x>\n", pete_readl("drivers/clk/mediatek/clk-fhctl.c:65", pll->pcw_addr));
}

static int fhctl_set_ssc_regs(struct mtk_clk_pll *pll, struct fh_pll_regs *regs,
			      const struct fh_pll_data *data, u32 rate)
{
	u32 updnlmt_val, r;

	pete_writel("drivers/clk/mediatek/clk-fhctl.c:73", (pete_readl("drivers/clk/mediatek/clk-fhctl.c:73", regs->reg_cfg) & ~(data->frddsx_en)), regs->reg_cfg);
	pete_writel("drivers/clk/mediatek/clk-fhctl.c:74", (pete_readl("drivers/clk/mediatek/clk-fhctl.c:74", regs->reg_cfg) & ~(data->sfstrx_en)), regs->reg_cfg);
	pete_writel("drivers/clk/mediatek/clk-fhctl.c:75", (pete_readl("drivers/clk/mediatek/clk-fhctl.c:75", regs->reg_cfg) & ~(data->fhctlx_en)), regs->reg_cfg);

	if (rate > 0) {
		/* Set the relative parameter registers (dt/df/upbnd/downbnd) */
		r = pete_readl("drivers/clk/mediatek/clk-fhctl.c:79", regs->reg_cfg);
		r &= ~(data->msk_frddsx_dys);
		r |= (data->df_val << (ffs(data->msk_frddsx_dys) - 1));
		pete_writel("drivers/clk/mediatek/clk-fhctl.c:82", r, regs->reg_cfg);

		r = pete_readl("drivers/clk/mediatek/clk-fhctl.c:84", regs->reg_cfg);
		r &= ~(data->msk_frddsx_dts);
		r |= (data->dt_val << (ffs(data->msk_frddsx_dts) - 1));
		pete_writel("drivers/clk/mediatek/clk-fhctl.c:87", r, regs->reg_cfg);

		pete_writel("drivers/clk/mediatek/clk-fhctl.c:89", (pete_readl("drivers/clk/mediatek/clk-fhctl.c:89", pll->pcw_addr) & data->dds_mask) | data->tgl_org,
			regs->reg_dds);

		/* Calculate UPDNLMT */
		updnlmt_val = PERCENT_TO_DDSLMT((pete_readl("drivers/clk/mediatek/clk-fhctl.c:93", regs->reg_dds) &
						 data->dds_mask), rate) <<
						 data->updnlmt_shft;

		pete_writel("drivers/clk/mediatek/clk-fhctl.c:97", updnlmt_val, regs->reg_updnlmt);
		pete_writel("drivers/clk/mediatek/clk-fhctl.c:98", pete_readl("drivers/clk/mediatek/clk-fhctl.c:98", regs->reg_hp_en) | BIT(data->fh_id),
		       regs->reg_hp_en);
		/* Enable SSC */
		pete_writel("drivers/clk/mediatek/clk-fhctl.c:101", pete_readl("drivers/clk/mediatek/clk-fhctl.c:101", regs->reg_cfg) | data->frddsx_en, regs->reg_cfg);
		/* Enable Hopping control */
		pete_writel("drivers/clk/mediatek/clk-fhctl.c:103", pete_readl("drivers/clk/mediatek/clk-fhctl.c:103", regs->reg_cfg) | data->fhctlx_en, regs->reg_cfg);

	} else {
		/* Switch to APMIXEDSYS control */
		pete_writel("drivers/clk/mediatek/clk-fhctl.c:107", pete_readl("drivers/clk/mediatek/clk-fhctl.c:107", regs->reg_hp_en) & ~BIT(data->fh_id),
		       regs->reg_hp_en);
		/* Wait for DDS to be stable */
		udelay(30);
	}

	return 0;
}

static int hopping_hw_flow(struct mtk_clk_pll *pll, struct fh_pll_regs *regs,
			   const struct fh_pll_data *data,
			   struct fh_pll_state *state, unsigned int new_dds)
{
	u32 dds_mask = data->dds_mask;
	u32 mon_dds = 0;
	u32 con_pcw_tmp;
	int ret;

	if (state->ssc_rate)
		fhctl_set_ssc_regs(pll, regs, data, 0);

	pete_writel("drivers/clk/mediatek/clk-fhctl.c:128", (pete_readl("drivers/clk/mediatek/clk-fhctl.c:128", pll->pcw_addr) & dds_mask) | data->tgl_org,
		regs->reg_dds);

	pete_writel("drivers/clk/mediatek/clk-fhctl.c:131", pete_readl("drivers/clk/mediatek/clk-fhctl.c:131", regs->reg_cfg) | data->sfstrx_en, regs->reg_cfg);
	pete_writel("drivers/clk/mediatek/clk-fhctl.c:132", pete_readl("drivers/clk/mediatek/clk-fhctl.c:132", regs->reg_cfg) | data->fhctlx_en, regs->reg_cfg);
	pete_writel("drivers/clk/mediatek/clk-fhctl.c:133", data->slope0_value, regs->reg_slope0);
	pete_writel("drivers/clk/mediatek/clk-fhctl.c:134", data->slope1_value, regs->reg_slope1);

	pete_writel("drivers/clk/mediatek/clk-fhctl.c:136", pete_readl("drivers/clk/mediatek/clk-fhctl.c:136", regs->reg_hp_en) | BIT(data->fh_id), regs->reg_hp_en);
	pete_writel("drivers/clk/mediatek/clk-fhctl.c:137", (new_dds) | (data->dvfs_tri), regs->reg_dvfs);

	/* Wait 1000 us until DDS stable */
	ret = readl_poll_timeout_atomic(regs->reg_mon, mon_dds,
				       (mon_dds & dds_mask) == new_dds,
					10, 1000);
	if (ret) {
		pr_warn("%s: FHCTL hopping timeout\n", pll->data->name);
		dump_hw(pll, regs, data);
	}

	con_pcw_tmp = pete_readl("drivers/clk/mediatek/clk-fhctl.c:148", pll->pcw_addr) & (~dds_mask);
	con_pcw_tmp = (con_pcw_tmp | (pete_readl("drivers/clk/mediatek/clk-fhctl.c:149", regs->reg_mon) & dds_mask) |
		       data->pcwchg);

	pete_writel("drivers/clk/mediatek/clk-fhctl.c:152", con_pcw_tmp, pll->pcw_addr);
	pete_writel("drivers/clk/mediatek/clk-fhctl.c:153", pete_readl("drivers/clk/mediatek/clk-fhctl.c:153", regs->reg_hp_en) & ~BIT(data->fh_id), regs->reg_hp_en);

	if (state->ssc_rate)
		fhctl_set_ssc_regs(pll, regs, data, state->ssc_rate);

	return ret;
}

static unsigned int __get_postdiv(struct mtk_clk_pll *pll)
{
	unsigned int regval;

	regval = pete_readl("drivers/clk/mediatek/clk-fhctl.c:165", pll->pd_addr) >> pll->data->pd_shift;
	regval &= POSTDIV_MASK;

	return BIT(regval);
}

static void __set_postdiv(struct mtk_clk_pll *pll, unsigned int postdiv)
{
	unsigned int regval;

	regval = pete_readl("drivers/clk/mediatek/clk-fhctl.c:175", pll->pd_addr);
	regval &= ~(POSTDIV_MASK << pll->data->pd_shift);
	regval |= (ffs(postdiv) - 1) << pll->data->pd_shift;
	pete_writel("drivers/clk/mediatek/clk-fhctl.c:178", regval, pll->pd_addr);
}

static int fhctl_hopping(struct mtk_fh *fh, unsigned int new_dds,
			 unsigned int postdiv)
{
	const struct fh_pll_data *data = &fh->pllfh_data->data;
	struct fh_pll_state *state = &fh->pllfh_data->state;
	struct fh_pll_regs *regs = &fh->regs;
	struct mtk_clk_pll *pll = &fh->clk_pll;
	spinlock_t *lock = fh->lock;
	unsigned int pll_postdiv;
	unsigned long flags = 0;
	int ret;

	if (postdiv) {
		pll_postdiv = __get_postdiv(pll);

		if (postdiv > pll_postdiv)
			__set_postdiv(pll, postdiv);
	}

	spin_lock_irqsave(lock, flags);

	ret = hopping_hw_flow(pll, regs, data, state, new_dds);

	spin_unlock_irqrestore(lock, flags);

	if (postdiv && postdiv < pll_postdiv)
		__set_postdiv(pll, postdiv);

	return ret;
}

static int fhctl_ssc_enable(struct mtk_fh *fh, u32 rate)
{
	const struct fh_pll_data *data = &fh->pllfh_data->data;
	struct fh_pll_state *state = &fh->pllfh_data->state;
	struct fh_pll_regs *regs = &fh->regs;
	struct mtk_clk_pll *pll = &fh->clk_pll;
	spinlock_t *lock = fh->lock;
	unsigned long flags = 0;

	spin_lock_irqsave(lock, flags);

	fhctl_set_ssc_regs(pll, regs, data, rate);
	state->ssc_rate = rate;

	spin_unlock_irqrestore(lock, flags);

	return 0;
}

static const struct fh_operation fhctl_ops = {
	.hopping = fhctl_hopping,
	.ssc_enable = fhctl_ssc_enable,
};

const struct fh_operation *fhctl_get_ops(void)
{
	return &fhctl_ops;
}

void fhctl_hw_init(struct mtk_fh *fh)
{
	const struct fh_pll_data data = fh->pllfh_data->data;
	struct fh_pll_state state = fh->pllfh_data->state;
	struct fh_pll_regs regs = fh->regs;
	u32 val;

	/* initial hw register */
	val = pete_readl("drivers/clk/mediatek/clk-fhctl.c:249", regs.reg_clk_con) | BIT(data.fh_id);
	pete_writel("drivers/clk/mediatek/clk-fhctl.c:250", val, regs.reg_clk_con);

	val = pete_readl("drivers/clk/mediatek/clk-fhctl.c:252", regs.reg_rst_con) & ~BIT(data.fh_id);
	pete_writel("drivers/clk/mediatek/clk-fhctl.c:253", val, regs.reg_rst_con);
	val = pete_readl("drivers/clk/mediatek/clk-fhctl.c:254", regs.reg_rst_con) | BIT(data.fh_id);
	pete_writel("drivers/clk/mediatek/clk-fhctl.c:255", val, regs.reg_rst_con);

	pete_writel("drivers/clk/mediatek/clk-fhctl.c:257", 0x0, regs.reg_cfg);
	pete_writel("drivers/clk/mediatek/clk-fhctl.c:258", 0x0, regs.reg_updnlmt);
	pete_writel("drivers/clk/mediatek/clk-fhctl.c:259", 0x0, regs.reg_dds);

	/* enable ssc if needed */
	if (state.ssc_rate)
		fh->ops->ssc_enable(fh, state.ssc_rate);
}
