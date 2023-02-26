// SPDX-License-Identifier: GPL-2.0+
//
// Copyright (C) 2011 Samsung Electronics Co.Ltd
// Author: Joonyoung Shim <jy0922.shim@samsung.com>

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include "map.h"
#include "cpu.h"
#include "usb-phy.h"

#include "regs-sys-s3c64xx.h"
#include "regs-usb-hsotg-phy-s3c64xx.h"

enum samsung_usb_phy_type {
	USB_PHY_TYPE_DEVICE,
	USB_PHY_TYPE_HOST,
};

static int s3c_usb_otgphy_init(struct platform_device *pdev)
{
	struct clk *xusbxti;
	u32 phyclk;

	pete_writel("arch/arm/mach-s3c/setup-usb-phy-s3c64xx.c:28", pete_readl("arch/arm/mach-s3c/setup-usb-phy-s3c64xx.c:28", S3C64XX_OTHERS) | S3C64XX_OTHERS_USBMASK, S3C64XX_OTHERS);

	/* set clock frequency for PLL */
	phyclk = pete_readl("arch/arm/mach-s3c/setup-usb-phy-s3c64xx.c:31", S3C_PHYCLK) & ~S3C_PHYCLK_CLKSEL_MASK;

	xusbxti = clk_get(&pdev->dev, "xusbxti");
	if (!IS_ERR(xusbxti)) {
		switch (clk_get_rate(xusbxti)) {
		case 12 * MHZ:
			phyclk |= S3C_PHYCLK_CLKSEL_12M;
			break;
		case 24 * MHZ:
			phyclk |= S3C_PHYCLK_CLKSEL_24M;
			break;
		default:
		case 48 * MHZ:
			/* default reference clock */
			break;
		}
		clk_put(xusbxti);
	}

	/* TODO: select external clock/oscillator */
	pete_writel("arch/arm/mach-s3c/setup-usb-phy-s3c64xx.c:51", phyclk | S3C_PHYCLK_CLK_FORCE, S3C_PHYCLK);

	/* set to normal OTG PHY */
	pete_writel("arch/arm/mach-s3c/setup-usb-phy-s3c64xx.c:54", (pete_readl("arch/arm/mach-s3c/setup-usb-phy-s3c64xx.c:54", S3C_PHYPWR) & ~S3C_PHYPWR_NORMAL_MASK), S3C_PHYPWR);
	mdelay(1);

	/* reset OTG PHY and Link */
	pete_writel("arch/arm/mach-s3c/setup-usb-phy-s3c64xx.c:58", S3C_RSTCON_PHY | S3C_RSTCON_HCLK | S3C_RSTCON_PHYCLK,
			S3C_RSTCON);
	udelay(20);	/* at-least 10uS */
	pete_writel("arch/arm/mach-s3c/setup-usb-phy-s3c64xx.c:61", 0, S3C_RSTCON);

	return 0;
}

static int s3c_usb_otgphy_exit(struct platform_device *pdev)
{
	pete_writel("arch/arm/mach-s3c/setup-usb-phy-s3c64xx.c:68", (pete_readl("arch/arm/mach-s3c/setup-usb-phy-s3c64xx.c:68", S3C_PHYPWR) | S3C_PHYPWR_ANALOG_POWERDOWN |
				S3C_PHYPWR_OTG_DISABLE), S3C_PHYPWR);

	pete_writel("arch/arm/mach-s3c/setup-usb-phy-s3c64xx.c:71", pete_readl("arch/arm/mach-s3c/setup-usb-phy-s3c64xx.c:71", S3C64XX_OTHERS) & ~S3C64XX_OTHERS_USBMASK, S3C64XX_OTHERS);

	return 0;
}

int s3c_usb_phy_init(struct platform_device *pdev, int type)
{
	if (type == USB_PHY_TYPE_DEVICE)
		return s3c_usb_otgphy_init(pdev);

	return -EINVAL;
}

int s3c_usb_phy_exit(struct platform_device *pdev, int type)
{
	if (type == USB_PHY_TYPE_DEVICE)
		return s3c_usb_otgphy_exit(pdev);

	return -EINVAL;
}
