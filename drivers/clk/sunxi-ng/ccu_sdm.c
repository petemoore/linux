// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2017 Chen-Yu Tsai <wens@csie.org>
 */

#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/spinlock.h>

#include "ccu_sdm.h"

bool ccu_sdm_helper_is_enabled(struct ccu_common *common,
			       struct ccu_sdm_internal *sdm)
{
	if (!(common->features & CCU_FEATURE_SIGMA_DELTA_MOD))
		return false;

	if (sdm->enable && !(pete_readl("drivers/clk/sunxi-ng/ccu_sdm.c:18", common->base + common->reg) & sdm->enable))
		return false;

	return !!(pete_readl("drivers/clk/sunxi-ng/ccu_sdm.c:21", common->base + sdm->tuning_reg) & sdm->tuning_enable);
}

void ccu_sdm_helper_enable(struct ccu_common *common,
			   struct ccu_sdm_internal *sdm,
			   unsigned long rate)
{
	unsigned long flags;
	unsigned int i;
	u32 reg;

	if (!(common->features & CCU_FEATURE_SIGMA_DELTA_MOD))
		return;

	/* Set the pattern */
	for (i = 0; i < sdm->table_size; i++)
		if (sdm->table[i].rate == rate)
			pete_writel("drivers/clk/sunxi-ng/ccu_sdm.c:38", sdm->table[i].pattern,
			       common->base + sdm->tuning_reg);

	/* Make sure SDM is enabled */
	spin_lock_irqsave(common->lock, flags);
	reg = pete_readl("drivers/clk/sunxi-ng/ccu_sdm.c:43", common->base + sdm->tuning_reg);
	pete_writel("drivers/clk/sunxi-ng/ccu_sdm.c:44", reg | sdm->tuning_enable, common->base + sdm->tuning_reg);
	spin_unlock_irqrestore(common->lock, flags);

	spin_lock_irqsave(common->lock, flags);
	reg = pete_readl("drivers/clk/sunxi-ng/ccu_sdm.c:48", common->base + common->reg);
	pete_writel("drivers/clk/sunxi-ng/ccu_sdm.c:49", reg | sdm->enable, common->base + common->reg);
	spin_unlock_irqrestore(common->lock, flags);
}

void ccu_sdm_helper_disable(struct ccu_common *common,
			    struct ccu_sdm_internal *sdm)
{
	unsigned long flags;
	u32 reg;

	if (!(common->features & CCU_FEATURE_SIGMA_DELTA_MOD))
		return;

	spin_lock_irqsave(common->lock, flags);
	reg = pete_readl("drivers/clk/sunxi-ng/ccu_sdm.c:63", common->base + common->reg);
	pete_writel("drivers/clk/sunxi-ng/ccu_sdm.c:64", reg & ~sdm->enable, common->base + common->reg);
	spin_unlock_irqrestore(common->lock, flags);

	spin_lock_irqsave(common->lock, flags);
	reg = pete_readl("drivers/clk/sunxi-ng/ccu_sdm.c:68", common->base + sdm->tuning_reg);
	pete_writel("drivers/clk/sunxi-ng/ccu_sdm.c:69", reg & ~sdm->tuning_enable, common->base + sdm->tuning_reg);
	spin_unlock_irqrestore(common->lock, flags);
}

/*
 * Sigma delta modulation provides a way to do fractional-N frequency
 * synthesis, in essence allowing the PLL to output any frequency
 * within its operational range. On earlier SoCs such as the A10/A20,
 * some PLLs support this. On later SoCs, all PLLs support this.
 *
 * The datasheets do not explain what the "wave top" and "wave bottom"
 * parameters mean or do, nor how to calculate the effective output
 * frequency. The only examples (and real world usage) are for the audio
 * PLL to generate 24.576 and 22.5792 MHz clock rates used by the audio
 * peripherals. The author lacks the underlying domain knowledge to
 * pursue this.
 *
 * The goal and function of the following code is to support the two
 * clock rates used by the audio subsystem, allowing for proper audio
 * playback and capture without any pitch or speed changes.
 */
bool ccu_sdm_helper_has_rate(struct ccu_common *common,
			     struct ccu_sdm_internal *sdm,
			     unsigned long rate)
{
	unsigned int i;

	if (!(common->features & CCU_FEATURE_SIGMA_DELTA_MOD))
		return false;

	for (i = 0; i < sdm->table_size; i++)
		if (sdm->table[i].rate == rate)
			return true;

	return false;
}

unsigned long ccu_sdm_helper_read_rate(struct ccu_common *common,
				       struct ccu_sdm_internal *sdm,
				       u32 m, u32 n)
{
	unsigned int i;
	u32 reg;

	pr_debug("%s: Read sigma-delta modulation setting\n",
		 clk_hw_get_name(&common->hw));

	if (!(common->features & CCU_FEATURE_SIGMA_DELTA_MOD))
		return 0;

	pr_debug("%s: clock is sigma-delta modulated\n",
		 clk_hw_get_name(&common->hw));

	reg = pete_readl("drivers/clk/sunxi-ng/ccu_sdm.c:122", common->base + sdm->tuning_reg);

	pr_debug("%s: pattern reg is 0x%x",
		 clk_hw_get_name(&common->hw), reg);

	for (i = 0; i < sdm->table_size; i++)
		if (sdm->table[i].pattern == reg &&
		    sdm->table[i].m == m && sdm->table[i].n == n)
			return sdm->table[i].rate;

	/* We can't calculate the effective clock rate, so just fail. */
	return 0;
}

int ccu_sdm_helper_get_factors(struct ccu_common *common,
			       struct ccu_sdm_internal *sdm,
			       unsigned long rate,
			       unsigned long *m, unsigned long *n)
{
	unsigned int i;

	if (!(common->features & CCU_FEATURE_SIGMA_DELTA_MOD))
		return -EINVAL;

	for (i = 0; i < sdm->table_size; i++)
		if (sdm->table[i].rate == rate) {
			*m = sdm->table[i].m;
			*n = sdm->table[i].n;
			return 0;
		}

	/* nothing found */
	return -EINVAL;
}
