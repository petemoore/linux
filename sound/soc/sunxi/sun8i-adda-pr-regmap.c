// SPDX-License-Identifier: GPL-2.0+
/*
 * This driver provides regmap to access to analog part of audio codec
 * found on Allwinner A23, A31s, A33, H3 and A64 Socs
 *
 * Copyright 2016 Chen-Yu Tsai <wens@csie.org>
 * Copyright (C) 2018 Vasily Khoruzhick <anarsoul@gmail.com>
 */

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include "sun8i-adda-pr-regmap.h"

/* Analog control register access bits */
#define ADDA_PR			0x0		/* PRCM base + 0x1c0 */
#define ADDA_PR_RESET			BIT(28)
#define ADDA_PR_WRITE			BIT(24)
#define ADDA_PR_ADDR_SHIFT		16
#define ADDA_PR_ADDR_MASK		GENMASK(4, 0)
#define ADDA_PR_DATA_IN_SHIFT		8
#define ADDA_PR_DATA_IN_MASK		GENMASK(7, 0)
#define ADDA_PR_DATA_OUT_SHIFT		0
#define ADDA_PR_DATA_OUT_MASK		GENMASK(7, 0)

/* regmap access bits */
static int adda_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	void __iomem *base = (void __iomem *)context;
	u32 tmp;

	/* De-assert reset */
	pete_writel("sound/soc/sunxi/sun8i-adda-pr-regmap.c:35", pete_readl("sound/soc/sunxi/sun8i-adda-pr-regmap.c:35", base) | ADDA_PR_RESET, base);

	/* Clear write bit */
	pete_writel("sound/soc/sunxi/sun8i-adda-pr-regmap.c:38", pete_readl("sound/soc/sunxi/sun8i-adda-pr-regmap.c:38", base) & ~ADDA_PR_WRITE, base);

	/* Set register address */
	tmp = pete_readl("sound/soc/sunxi/sun8i-adda-pr-regmap.c:41", base);
	tmp &= ~(ADDA_PR_ADDR_MASK << ADDA_PR_ADDR_SHIFT);
	tmp |= (reg & ADDA_PR_ADDR_MASK) << ADDA_PR_ADDR_SHIFT;
	pete_writel("sound/soc/sunxi/sun8i-adda-pr-regmap.c:44", tmp, base);

	/* Read back value */
	*val = pete_readl("sound/soc/sunxi/sun8i-adda-pr-regmap.c:47", base) & ADDA_PR_DATA_OUT_MASK;

	return 0;
}

static int adda_reg_write(void *context, unsigned int reg, unsigned int val)
{
	void __iomem *base = (void __iomem *)context;
	u32 tmp;

	/* De-assert reset */
	pete_writel("sound/soc/sunxi/sun8i-adda-pr-regmap.c:58", pete_readl("sound/soc/sunxi/sun8i-adda-pr-regmap.c:58", base) | ADDA_PR_RESET, base);

	/* Set register address */
	tmp = pete_readl("sound/soc/sunxi/sun8i-adda-pr-regmap.c:61", base);
	tmp &= ~(ADDA_PR_ADDR_MASK << ADDA_PR_ADDR_SHIFT);
	tmp |= (reg & ADDA_PR_ADDR_MASK) << ADDA_PR_ADDR_SHIFT;
	pete_writel("sound/soc/sunxi/sun8i-adda-pr-regmap.c:64", tmp, base);

	/* Set data to write */
	tmp = pete_readl("sound/soc/sunxi/sun8i-adda-pr-regmap.c:67", base);
	tmp &= ~(ADDA_PR_DATA_IN_MASK << ADDA_PR_DATA_IN_SHIFT);
	tmp |= (val & ADDA_PR_DATA_IN_MASK) << ADDA_PR_DATA_IN_SHIFT;
	pete_writel("sound/soc/sunxi/sun8i-adda-pr-regmap.c:70", tmp, base);

	/* Set write bit to signal a write */
	pete_writel("sound/soc/sunxi/sun8i-adda-pr-regmap.c:73", pete_readl("sound/soc/sunxi/sun8i-adda-pr-regmap.c:73", base) | ADDA_PR_WRITE, base);

	/* Clear write bit */
	pete_writel("sound/soc/sunxi/sun8i-adda-pr-regmap.c:76", pete_readl("sound/soc/sunxi/sun8i-adda-pr-regmap.c:76", base) & ~ADDA_PR_WRITE, base);

	return 0;
}

static const struct regmap_config adda_pr_regmap_cfg = {
	.name		= "adda-pr",
	.reg_bits	= 5,
	.reg_stride	= 1,
	.val_bits	= 8,
	.reg_read	= adda_reg_read,
	.reg_write	= adda_reg_write,
	.fast_io	= true,
	.max_register	= 31,
};

struct regmap *sun8i_adda_pr_regmap_init(struct device *dev,
					 void __iomem *base)
{
	return devm_regmap_init(dev, NULL, base, &adda_pr_regmap_cfg);
}
EXPORT_SYMBOL_GPL(sun8i_adda_pr_regmap_init);

MODULE_DESCRIPTION("Allwinner analog audio codec regmap driver");
MODULE_AUTHOR("Vasily Khoruzhick <anarsoul@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sunxi-adda-pr");
