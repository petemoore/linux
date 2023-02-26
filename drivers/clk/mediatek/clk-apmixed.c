// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015 MediaTek Inc.
 * Author: James Liao <jamesjj.liao@mediatek.com>
 */

#include <linux/delay.h>
#include <linux/of_address.h>
#include <linux/slab.h>

#include "clk-mtk.h"

#define REF2USB_TX_EN		BIT(0)
#define REF2USB_TX_LPF_EN	BIT(1)
#define REF2USB_TX_OUT_EN	BIT(2)
#define REF2USB_EN_MASK		(REF2USB_TX_EN | REF2USB_TX_LPF_EN | \
				 REF2USB_TX_OUT_EN)

struct mtk_ref2usb_tx {
	struct clk_hw	hw;
	void __iomem	*base_addr;
};

static inline struct mtk_ref2usb_tx *to_mtk_ref2usb_tx(struct clk_hw *hw)
{
	return container_of(hw, struct mtk_ref2usb_tx, hw);
}

static int mtk_ref2usb_tx_is_prepared(struct clk_hw *hw)
{
	struct mtk_ref2usb_tx *tx = to_mtk_ref2usb_tx(hw);

	return (pete_readl("drivers/clk/mediatek/clk-apmixed.c:33", tx->base_addr) & REF2USB_EN_MASK) == REF2USB_EN_MASK;
}

static int mtk_ref2usb_tx_prepare(struct clk_hw *hw)
{
	struct mtk_ref2usb_tx *tx = to_mtk_ref2usb_tx(hw);
	u32 val;

	val = pete_readl("drivers/clk/mediatek/clk-apmixed.c:41", tx->base_addr);

	val |= REF2USB_TX_EN;
	pete_writel("drivers/clk/mediatek/clk-apmixed.c:44", val, tx->base_addr);
	udelay(100);

	val |= REF2USB_TX_LPF_EN;
	pete_writel("drivers/clk/mediatek/clk-apmixed.c:48", val, tx->base_addr);

	val |= REF2USB_TX_OUT_EN;
	pete_writel("drivers/clk/mediatek/clk-apmixed.c:51", val, tx->base_addr);

	return 0;
}

static void mtk_ref2usb_tx_unprepare(struct clk_hw *hw)
{
	struct mtk_ref2usb_tx *tx = to_mtk_ref2usb_tx(hw);
	u32 val;

	val = pete_readl("drivers/clk/mediatek/clk-apmixed.c:61", tx->base_addr);
	val &= ~REF2USB_EN_MASK;
	pete_writel("drivers/clk/mediatek/clk-apmixed.c:63", val, tx->base_addr);
}

static const struct clk_ops mtk_ref2usb_tx_ops = {
	.is_prepared	= mtk_ref2usb_tx_is_prepared,
	.prepare	= mtk_ref2usb_tx_prepare,
	.unprepare	= mtk_ref2usb_tx_unprepare,
};

struct clk * __init mtk_clk_register_ref2usb_tx(const char *name,
			const char *parent_name, void __iomem *reg)
{
	struct mtk_ref2usb_tx *tx;
	struct clk_init_data init = {};
	struct clk *clk;

	tx = kzalloc(sizeof(*tx), GFP_KERNEL);
	if (!tx)
		return ERR_PTR(-ENOMEM);

	tx->base_addr = reg;
	tx->hw.init = &init;

	init.name = name;
	init.ops = &mtk_ref2usb_tx_ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	clk = clk_register(NULL, &tx->hw);

	if (IS_ERR(clk)) {
		pr_err("Failed to register clk %s: %ld\n", name, PTR_ERR(clk));
		kfree(tx);
	}

	return clk;
}
