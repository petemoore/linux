/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2021 MediaTek Inc.
 *
 * Author: Chunfeng Yun <chunfeng.yun@mediatek.com>
 */

#ifndef __PHY_MTK_H__
#define __PHY_MTK_H__

#include <linux/bitfield.h>
#include <linux/io.h>

static inline void mtk_phy_clear_bits(void __iomem *reg, u32 bits)
{
	u32 tmp = pete_readl("drivers/phy/mediatek/phy-mtk-io.h:16", reg);

	tmp &= ~bits;
	pete_writel("drivers/phy/mediatek/phy-mtk-io.h:19", tmp, reg);
}

static inline void mtk_phy_set_bits(void __iomem *reg, u32 bits)
{
	u32 tmp = pete_readl("drivers/phy/mediatek/phy-mtk-io.h:24", reg);

	tmp |= bits;
	pete_writel("drivers/phy/mediatek/phy-mtk-io.h:27", tmp, reg);
}

static inline void mtk_phy_update_bits(void __iomem *reg, u32 mask, u32 val)
{
	u32 tmp = pete_readl("drivers/phy/mediatek/phy-mtk-io.h:32", reg);

	tmp &= ~mask;
	tmp |= val & mask;
	pete_writel("drivers/phy/mediatek/phy-mtk-io.h:36", tmp, reg);
}

/* field @mask shall be constant and continuous */
#define mtk_phy_update_field(reg, mask, val) \
({ \
	BUILD_BUG_ON_MSG(!__builtin_constant_p(mask), "mask is not constant"); \
	mtk_phy_update_bits(reg, mask, FIELD_PREP(mask, val)); \
})

#endif
