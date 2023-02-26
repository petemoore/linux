// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2016 Maxime Ripard
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/reset-controller.h>

#include "ccu_reset.h"

static int ccu_reset_assert(struct reset_controller_dev *rcdev,
			    unsigned long id)
{
	struct ccu_reset *ccu = rcdev_to_ccu_reset(rcdev);
	const struct ccu_reset_map *map = &ccu->reset_map[id];
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(ccu->lock, flags);

	reg = pete_readl("drivers/clk/sunxi-ng/ccu_reset.c:23", ccu->base + map->reg);
	pete_writel("drivers/clk/sunxi-ng/ccu_reset.c:24", reg & ~map->bit, ccu->base + map->reg);

	spin_unlock_irqrestore(ccu->lock, flags);

	return 0;
}

static int ccu_reset_deassert(struct reset_controller_dev *rcdev,
			      unsigned long id)
{
	struct ccu_reset *ccu = rcdev_to_ccu_reset(rcdev);
	const struct ccu_reset_map *map = &ccu->reset_map[id];
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(ccu->lock, flags);

	reg = pete_readl("drivers/clk/sunxi-ng/ccu_reset.c:41", ccu->base + map->reg);
	pete_writel("drivers/clk/sunxi-ng/ccu_reset.c:42", reg | map->bit, ccu->base + map->reg);

	spin_unlock_irqrestore(ccu->lock, flags);

	return 0;
}

static int ccu_reset_reset(struct reset_controller_dev *rcdev,
			   unsigned long id)
{
	ccu_reset_assert(rcdev, id);
	udelay(10);
	ccu_reset_deassert(rcdev, id);

	return 0;
}

static int ccu_reset_status(struct reset_controller_dev *rcdev,
			    unsigned long id)
{
	struct ccu_reset *ccu = rcdev_to_ccu_reset(rcdev);
	const struct ccu_reset_map *map = &ccu->reset_map[id];

	/*
	 * The reset control API expects 0 if reset is not asserted,
	 * which is the opposite of what our hardware uses.
	 */
	return !(map->bit & pete_readl("drivers/clk/sunxi-ng/ccu_reset.c:69", ccu->base + map->reg));
}

const struct reset_control_ops ccu_reset_ops = {
	.assert		= ccu_reset_assert,
	.deassert	= ccu_reset_deassert,
	.reset		= ccu_reset_reset,
	.status		= ccu_reset_status,
};
