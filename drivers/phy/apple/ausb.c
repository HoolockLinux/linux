// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/*
 * Apple AUSB PHY
 *
 * Copyright (C) 2025 Nick Chan
 */


#include <linux/kernel.h>
#include <linux/bitfield.h>
#include <linux/soc/apple/tunable.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/delay.h>

/* These look similar to ATC's USB2PHY_CTL and USB2PHY_SIG */
#define AUSB_CTL			0x00
#define AUSB_CTL_RESET			BIT(0)
#define AUSB_CTL_PWRDOWN		BIT(2)
#define AUSB_CTL_SIDDQ   		BIT(3)

#define AUSB_SIG			0x04
#define AUSB_SIG_VBUSDET_FORCE_EN	BIT(1)
#define AUSB_SIG_CABLE_CONNECTED	BIT(8)
#define AUSB_SIG_HOST			(7 << 12)

/* Tunable registers */
#define AUSB_CFG0			0x08
#define AUSB_CFG1			0x0c

struct ausb_usbphy {
	struct device *dev;
	struct phy *phy;
	void __iomem *base;
	struct phy_provider *phy_provider;
	u32 cfg0_device;
	u32 cfg1_device;
	u32 cfg0_host;
	u32 cfg1_host;
};

static inline void mask32(void __iomem *reg, u32 mask, u32 set)
{
	u32 value = readl(reg);
	value &= ~mask;
	value |= set;
	writel(value, reg);
}

static inline void set32(void __iomem *reg, u32 set)
{
	mask32(reg, 0, set);
}

static inline void clear32(void __iomem *reg, u32 clear)
{
	mask32(reg, clear, 0);
}

#if 0
static __maybe_unused int ausb_phy_usb2_set_mode(struct phy *phy, enum phy_mode mode,
				int submode)
{
	struct ausb_usbphy *aphy = phy_get_drvdata(phy);

	switch (mode) {
	case PHY_MODE_USB_HOST:
	case PHY_MODE_USB_HOST_LS:
	case PHY_MODE_USB_HOST_FS:
	case PHY_MODE_USB_HOST_HS:
	case PHY_MODE_USB_HOST_SS:
		writel(aphy->cfg0_host, aphy->base + AUSB_CFG0);
		writel(aphy->cfg1_host, aphy->base + AUSB_CFG1);
		set32(aphy->base + AUSB_SIG, AUSB_SIG_HOST);
		return 0;

	case PHY_MODE_USB_DEVICE:
	case PHY_MODE_USB_DEVICE_LS:
	case PHY_MODE_USB_DEVICE_FS:
	case PHY_MODE_USB_DEVICE_HS:
	case PHY_MODE_USB_DEVICE_SS:
		writel(aphy->cfg0_device, aphy->base + AUSB_CFG0);
		writel(aphy->cfg1_device, aphy->base + AUSB_CFG1);
		clear32(aphy->base + AUSB_SIG, AUSB_SIG_HOST);
		return 0;

	default:
		dev_err(aphy->dev, "Unknown mode for usb2 phy: %d\n", mode);
		return -EINVAL;
	}
}
#endif

static int ausb_phy_usb2_power_on(struct phy *phy)
{
	struct ausb_usbphy *aphy = phy_get_drvdata(phy);

	/* set a sane tunable */
	writel(aphy->cfg0_device, aphy->base + AUSB_CFG0);
	writel(aphy->cfg1_device, aphy->base + AUSB_CFG1);

	set32(aphy->base + AUSB_CTL, AUSB_CTL_RESET);
	udelay(20);
	clear32(aphy->base, AUSB_CTL_PWRDOWN | AUSB_CTL_SIDDQ);
	udelay(20);
	clear32(aphy->base + AUSB_CTL, AUSB_CTL_RESET);
	udelay(1500);
	set32(aphy->base + AUSB_SIG, AUSB_SIG_VBUSDET_FORCE_EN);

	return 0;
}

static int ausb_phy_usb2_power_off(struct phy *phy)
{
	struct ausb_usbphy *aphy = phy_get_drvdata(phy);

	set32(aphy->base + AUSB_CTL, AUSB_CTL_RESET);
	udelay(20);
	set32(aphy->base, AUSB_CTL_PWRDOWN | AUSB_CTL_SIDDQ);
	udelay(20);
	clear32(aphy->base + AUSB_CTL, AUSB_CTL_RESET);
	udelay(1500);
	clear32(aphy->base + AUSB_SIG, AUSB_SIG_VBUSDET_FORCE_EN);

	return 0;
}


static const struct phy_ops ausb_phy_usb2_ops = {
	.owner = THIS_MODULE,
	//.set_mode = ausb_phy_usb2_set_mode,
	.init = ausb_phy_usb2_power_on,
	.exit = ausb_phy_usb2_power_off,
};

static int ausb_usbphy_probe(struct platform_device *pdev) {
	struct ausb_usbphy *aphy;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct resource *res;

	aphy = devm_kzalloc(&pdev->dev, sizeof(*aphy), GFP_KERNEL);
	if (!aphy)
		return -ENOMEM;

	if (of_property_read_u32(np, "apple,cfg0-device", &aphy->cfg0_device))
		return -ENODEV;
	if (of_property_read_u32(np, "apple,cfg1-device", &aphy->cfg1_device))
		return -ENODEV;
	if (of_property_read_u32(np, "apple,cfg0-host", &aphy->cfg0_host))
		return -ENODEV;
	if (of_property_read_u32(np, "apple,cfg1-host", &aphy->cfg1_host))
		return -ENODEV;

	aphy->dev = dev;

	aphy->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(aphy->base))
		return PTR_ERR(aphy->base);

	aphy->phy = devm_phy_create(aphy->dev, NULL, &ausb_phy_usb2_ops);
	if (IS_ERR(aphy->phy))
		return PTR_ERR(aphy->phy);

	phy_set_drvdata(aphy->phy, aphy);

	aphy->phy_provider = devm_of_phy_provider_register(&pdev->dev,
		of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(aphy->phy_provider);
}

static const struct of_device_id ausb_usbphy_of_match[] = {
	{ .compatible = "apple,ausb-phy" },
	{},
};

static struct platform_driver ausb_usbphy_driver = {
	.probe = ausb_usbphy_probe,
	.driver = {
		.of_match_table = ausb_usbphy_of_match,
		.name = "phy-apple-ausb",
	}

};

module_platform_driver(ausb_usbphy_driver);
