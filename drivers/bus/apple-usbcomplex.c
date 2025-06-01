// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * Apple USB Complex
 *
 * Copyright (C) 2025 Nick Chan
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

/* S5L8960X remap registers */
#define USBX_REMAP_EN_S5L8960X		BIT(8)
#define USBX_REMAP_VAL_MASK_S5L8960X 	GENMASK(3, 0)
#define USBX_USBDEV_REMAP_S5L8960X	0x1c
#define USBX_EHCI0_REMAP_S5L8960X	0x3c
#define USBX_OHCI0_REMAP_S5L8960X	0x5c
#define USBX_EHCI1_REMAP_S5L8960X	0x7c
/* T7000/S8000/S8003 only, ignored by other hardware */
#define USBX_EHCI2_REMAP_T7000		0x9c

/* T8011/T8015 remap register values */
#define USBX_CTL_T8011			0x00
#define USBX_CTL_EN_T8011		BIT(0)
#define USBX_REMAP_EN1_T8011		BIT(24)
#define USBX_REMAP_EN2_T8011		BIT(25)
#define USBX_REMAP_VAL_MASK1_T8011	GENMASK(3, 0)
#define USBX_REMAP_VAL_MASK2_T8011	GENMASK(7, 4)
#define USBX_REMAP_VAL_SHIFT1_T8011	0
#define USBX_REMAP_VAL_SHIFT2_T8011	4

/* T8011 remap registers */
#define USBX_USB3DEV_REMAP_CTL_T8011	0x18
#define USBX_USB2DEV_REMAP_CTL_T8011	0x24
#define USBX_EHCI_REMAP_CTL_T8011	0x14 /* 0x74 */
#define USBX_XHCI_REMAP_CTL_T8011	0x24 /* 0x84 */

/* T8015 remap registers */
#define USBX_EHCI0_REMAP_CTL_T8015	0x18
#define USBX_OHCI0_REMAP_CTL_T8015	0x28
#define USBX_EHCI1_REMAP_CTL_T8015	0x38
#define USBX_USBDEV_REMAP_CTL_T8015	0x48

#define USBX_REMAP_ADDR_MASK		GENMASK(35, 32)

struct apple_usbcomplex_hw {
	u32 remap_regs[5];
	u32 hi_remap_regs[2];
	u32 (*remap_reg_value)(u32 hi_bits);
	void (*enable)(void __iomem *base);
};

struct apple_usbcomplex {
	void __iomem *base;
	void __iomem *hi_remap_regs;
	struct clk_bulk_data *clks;
	const struct apple_usbcomplex_hw *hw;
	u32 hi_bits;
	int num_clks;
};

static u32 apple_usbcomplex_remap_reg_value_s5l8960x(u32 hi_bits)
{
	return (hi_bits & USBX_REMAP_VAL_MASK_S5L8960X) | USBX_REMAP_EN_S5L8960X;
}

static u32 apple_usbcomplex_remap_reg_value_t8011(u32 hi_bits)
{
	return ((hi_bits << USBX_REMAP_VAL_SHIFT1_T8011) & USBX_REMAP_VAL_MASK1_T8011)
		| ((hi_bits << USBX_REMAP_VAL_SHIFT2_T8011) & USBX_REMAP_VAL_MASK2_T8011)
		| USBX_REMAP_EN1_T8011 | USBX_REMAP_EN2_T8011;
}

static void apple_usbcomplex_enable_t8011(void __iomem *base) {
	writel(USBX_CTL_EN_T8011, base + USBX_CTL_T8011);
};

static void apple_usbcomplex_init(struct apple_usbcomplex *complex) {
	if (complex->hw->enable)
		complex->hw->enable(complex->base);

	u32 remap_val = complex->hw->remap_reg_value(complex->hi_bits);

	for (int i = 0; i < sizeof(complex->hw->remap_regs)/sizeof(u32); i++) {
		if (complex->hw->remap_regs[i])
			writel(remap_val, complex->base + complex->hw->remap_regs[i]);
	}

	for (int i = 0; i < sizeof(complex->hw->hi_remap_regs)/sizeof(u32); i++) {
		if (complex->hw->hi_remap_regs[i])
			writel(remap_val, complex->hi_remap_regs + complex->hw->hi_remap_regs[i]);
	}
};

static int apple_usbcomplex_probe(struct platform_device *pdev)
{
	const struct device *dev = &pdev->dev;
	const struct of_dev_auxdata *lookup = dev_get_platdata(dev);
	struct device_node *np = dev->of_node;
	struct resource *res;
	struct of_range_parser parser;
	struct of_range range;
	struct apple_usbcomplex *complex;
	int num_reg, ret;

	ret = of_dma_range_parser_init(&parser, np);

	if (ret < 0)
		return ret;

	for_each_of_range(&parser, &range) {
		if (range.size != BIT(32) || range.cpu_addr & U32_MAX || range.bus_addr != 0)
			return -EINVAL;

		break;
	}

	num_reg = of_address_count(np);

	complex = devm_kzalloc(&pdev->dev, sizeof(*complex), GFP_KERNEL);

	if (!complex)
		return -ENOMEM;

        complex->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
        if (IS_ERR(complex->base))
                return PTR_ERR(complex->base);
	if (num_reg > 1) {
		complex->hi_remap_regs = devm_platform_get_and_ioremap_resource(pdev, 1, &res);
		if (IS_ERR(complex->hi_remap_regs))
			return PTR_ERR(complex->hi_remap_regs);
	}

	complex->hw = of_device_get_match_data(&pdev->dev);
	if (!complex->hw)
		return -ENODEV;

	complex->num_clks = devm_clk_bulk_get_all(&pdev->dev, &complex->clks);
	if (complex->num_clks < 0)
		return dev_err_probe(&pdev->dev, complex->num_clks, "failed to get clocks\n");

	complex->hi_bits = FIELD_GET(USBX_REMAP_ADDR_MASK, range.cpu_addr);

	dev_set_drvdata(&pdev->dev, complex);

	pm_runtime_enable(&pdev->dev);

	apple_usbcomplex_init(complex);

	if (np)
		of_platform_populate(np, NULL, lookup, &pdev->dev);

	return 0;
}

static void apple_usbcomplex_remove(struct platform_device *pdev)
{
	const void *data = of_device_get_match_data(&pdev->dev);

	if (device_has_driver_override(&pdev->dev) || data)
		return;

	pm_runtime_disable(&pdev->dev);
}

static const struct apple_usbcomplex_hw s5l8960x_usbcomplex = {
	.remap_regs = {
	  USBX_USBDEV_REMAP_S5L8960X,
	  USBX_EHCI0_REMAP_S5L8960X,
	  USBX_OHCI0_REMAP_S5L8960X,
	  USBX_EHCI1_REMAP_S5L8960X,
	  USBX_EHCI2_REMAP_T7000
	},
	.remap_reg_value = &apple_usbcomplex_remap_reg_value_s5l8960x,
};

static const struct apple_usbcomplex_hw t8011_usbcomplex = {
	.remap_regs = {
	  USBX_USB3DEV_REMAP_CTL_T8011,
	  USBX_USB2DEV_REMAP_CTL_T8011
	},
	.hi_remap_regs = {
	  USBX_EHCI_REMAP_CTL_T8011,
	  USBX_XHCI_REMAP_CTL_T8011
	},
	.remap_reg_value = &apple_usbcomplex_remap_reg_value_t8011,
	.enable = apple_usbcomplex_enable_t8011,
};

static const struct apple_usbcomplex_hw t8015_usbcomplex = {
	.remap_regs = {
	  USBX_EHCI0_REMAP_CTL_T8015,
	  USBX_OHCI0_REMAP_CTL_T8015,
	  USBX_EHCI1_REMAP_CTL_T8015,
	  USBX_USBDEV_REMAP_CTL_T8015
	},
	.remap_reg_value = &apple_usbcomplex_remap_reg_value_t8011,
	.enable = apple_usbcomplex_enable_t8011,
};

static const struct of_device_id apple_usbcomplex_of_match[] = {
	{ .compatible = "apple,s5l8960x-usb-complex", .data = &s5l8960x_usbcomplex },
	{ .compatible = "apple,t8011-usb-complex", .data = &t8011_usbcomplex },
	{ .compatible = "apple,t8015-usb-complex", .data = &t8015_usbcomplex },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, apple_usbcomplex_of_match);

static struct platform_driver apple_usbcomplex_driver = {
	.probe = apple_usbcomplex_probe,
	.remove = apple_usbcomplex_remove,
	.driver = {
		.name = "apple-usb-complex",
		.of_match_table = apple_usbcomplex_of_match,
	},
};

module_platform_driver(apple_usbcomplex_driver);

MODULE_DESCRIPTION("Apple USB Complex Remapper");
MODULE_AUTHOR("Nick Chan <towinchenmi@gmail.com>");
