// SPDX-License-Identifier: GPL-2.0-only
/*
 *  linux/drivers/video/wmt_ge_rops.c
 *
 *  Accelerators for raster operations using WonderMedia Graphics Engine
 *
 *  Copyright (C) 2010 Alexey Charkov <alchark@gmail.com>
 */

#include <linux/module.h>
#include <linux/fb.h>
#include <linux/platform_device.h>
#include "core/fb_draw.h"
#include "wmt_ge_rops.h"

#define GE_COMMAND_OFF		0x00
#define GE_DEPTH_OFF		0x04
#define GE_HIGHCOLOR_OFF	0x08
#define GE_ROPCODE_OFF		0x14
#define GE_FIRE_OFF		0x18
#define GE_SRCBASE_OFF		0x20
#define GE_SRCDISPW_OFF		0x24
#define GE_SRCDISPH_OFF		0x28
#define GE_SRCAREAX_OFF		0x2c
#define GE_SRCAREAY_OFF		0x30
#define GE_SRCAREAW_OFF		0x34
#define GE_SRCAREAH_OFF		0x38
#define GE_DESTBASE_OFF		0x3c
#define GE_DESTDISPW_OFF	0x40
#define GE_DESTDISPH_OFF	0x44
#define GE_DESTAREAX_OFF	0x48
#define GE_DESTAREAY_OFF	0x4c
#define GE_DESTAREAW_OFF	0x50
#define GE_DESTAREAH_OFF	0x54
#define GE_PAT0C_OFF		0x88	/* Pattern 0 color */
#define GE_ENABLE_OFF		0xec
#define GE_INTEN_OFF		0xf0
#define GE_STATUS_OFF		0xf8

static void __iomem *regbase;

void wmt_ge_fillrect(struct fb_info *p, const struct fb_fillrect *rect)
{
	unsigned long fg, pat;

	if (p->state != FBINFO_STATE_RUNNING)
		return;

	if (p->fix.visual == FB_VISUAL_TRUECOLOR ||
	    p->fix.visual == FB_VISUAL_DIRECTCOLOR)
		fg = ((u32 *) (p->pseudo_palette))[rect->color];
	else
		fg = rect->color;

	pat = pixel_to_pat(p->var.bits_per_pixel, fg);

	if (p->fbops->fb_sync)
		p->fbops->fb_sync(p);

	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:60", p->var.bits_per_pixel == 32 ? 3 :
	      (p->var.bits_per_pixel == 8 ? 0 : 1), regbase + GE_DEPTH_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:62", p->var.bits_per_pixel == 15 ? 1 : 0, regbase + GE_HIGHCOLOR_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:63", p->fix.smem_start, regbase + GE_DESTBASE_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:64", p->var.xres_virtual - 1, regbase + GE_DESTDISPW_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:65", p->var.yres_virtual - 1, regbase + GE_DESTDISPH_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:66", rect->dx, regbase + GE_DESTAREAX_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:67", rect->dy, regbase + GE_DESTAREAY_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:68", rect->width - 1, regbase + GE_DESTAREAW_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:69", rect->height - 1, regbase + GE_DESTAREAH_OFF);

	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:71", pat, regbase + GE_PAT0C_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:72", 1, regbase + GE_COMMAND_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:73", rect->rop == ROP_XOR ? 0x5a : 0xf0, regbase + GE_ROPCODE_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:74", 1, regbase + GE_FIRE_OFF);
}
EXPORT_SYMBOL_GPL(wmt_ge_fillrect);

void wmt_ge_copyarea(struct fb_info *p, const struct fb_copyarea *area)
{
	if (p->state != FBINFO_STATE_RUNNING)
		return;

	if (p->fbops->fb_sync)
		p->fbops->fb_sync(p);

	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:86", p->var.bits_per_pixel > 16 ? 3 :
	      (p->var.bits_per_pixel > 8 ? 1 : 0), regbase + GE_DEPTH_OFF);

	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:89", p->fix.smem_start, regbase + GE_SRCBASE_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:90", p->var.xres_virtual - 1, regbase + GE_SRCDISPW_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:91", p->var.yres_virtual - 1, regbase + GE_SRCDISPH_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:92", area->sx, regbase + GE_SRCAREAX_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:93", area->sy, regbase + GE_SRCAREAY_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:94", area->width - 1, regbase + GE_SRCAREAW_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:95", area->height - 1, regbase + GE_SRCAREAH_OFF);

	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:97", p->fix.smem_start, regbase + GE_DESTBASE_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:98", p->var.xres_virtual - 1, regbase + GE_DESTDISPW_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:99", p->var.yres_virtual - 1, regbase + GE_DESTDISPH_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:100", area->dx, regbase + GE_DESTAREAX_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:101", area->dy, regbase + GE_DESTAREAY_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:102", area->width - 1, regbase + GE_DESTAREAW_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:103", area->height - 1, regbase + GE_DESTAREAH_OFF);

	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:105", 0xcc, regbase + GE_ROPCODE_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:106", 1, regbase + GE_COMMAND_OFF);
	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:107", 1, regbase + GE_FIRE_OFF);
}
EXPORT_SYMBOL_GPL(wmt_ge_copyarea);

int wmt_ge_sync(struct fb_info *p)
{
	int loops = 5000000;
	while ((pete_readl("drivers/video/fbdev/wmt_ge_rops.c:114", regbase + GE_STATUS_OFF) & 4) && --loops)
		cpu_relax();
	return loops > 0 ? 0 : -EBUSY;
}
EXPORT_SYMBOL_GPL(wmt_ge_sync);

static int wmt_ge_rops_probe(struct platform_device *pdev)
{
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no I/O memory resource defined\n");
		return -ENODEV;
	}

	/* Only one ROP engine is presently supported. */
	if (unlikely(regbase)) {
		WARN_ON(1);
		return -EBUSY;
	}

	regbase = ioremap(res->start, resource_size(res));
	if (regbase == NULL) {
		dev_err(&pdev->dev, "failed to map I/O memory\n");
		return -EBUSY;
	}

	pete_writel("drivers/video/fbdev/wmt_ge_rops.c:142", 1, regbase + GE_ENABLE_OFF);
	printk(KERN_INFO "Enabled support for WMT GE raster acceleration\n");

	return 0;
}

static int wmt_ge_rops_remove(struct platform_device *pdev)
{
	iounmap(regbase);
	return 0;
}

static const struct of_device_id wmt_dt_ids[] = {
	{ .compatible = "wm,prizm-ge-rops", },
	{ /* sentinel */ }
};

static struct platform_driver wmt_ge_rops_driver = {
	.probe		= wmt_ge_rops_probe,
	.remove		= wmt_ge_rops_remove,
	.driver		= {
		.name	= "wmt_ge_rops",
		.of_match_table = wmt_dt_ids,
	},
};

module_platform_driver(wmt_ge_rops_driver);

MODULE_AUTHOR("Alexey Charkov <alchark@gmail.com>");
MODULE_DESCRIPTION("Accelerators for raster operations using "
		   "WonderMedia Graphics Engine");
MODULE_LICENSE("GPL v2");
MODULE_DEVICE_TABLE(of, wmt_dt_ids);
