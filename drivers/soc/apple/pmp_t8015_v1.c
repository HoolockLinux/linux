// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple T8015 PMP (Power Management Processor) Driver
 *
 * Copyright 2026 Nick Chan
 */

#include <linux/bitfield.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/dma-mapping.h>
#include <linux/ioport.h>
#include <linux/io.h>

#define AKF_REMAP_AP_LO		0x8
#define AKF_REMAP_AP_HI		0x10
#define AKF_REMAP_IOP_LO	0x18
#define AKF_REMAP_IOP_HI	0x20
#define AKF_REMAP_SZ_LO		0x28
#define AKF_REMAP_SZ_HI		0x30
#define AKF_REMAP_ENABLE	0x38
#define AKF_REG_80C		0x80c

struct apple_pmp {
	void __iomem *akf_base;
	void __iomem *ctl_base;
	void __iomem *sram_base;
	struct resource *sram;
	struct device *dev;
	const char *fw_name;
	struct apple_rtkit *rtk;
};

static const struct apple_rtkit_ops apple_pmp_rtkit_ops = {};

static int apple_pmp_upload_firmware(struct apple_pmp *pmp)
{
	const struct firmware *fw __free(firmware) = NULL;
	int ret;

	ret = request_firmware(&fw, pmp->fw_name, pmp->dev);
	if (ret) {
		dev_err(pmp->dev, "unable to load firmware\n");
		return ret;
	}

	if (fw->size > resource_size(pmp->sram)) {
		dev_err(pmp->dev, "firmware larger than SRAM: %zu > %llu",
			fw->size, resource_size(pmp->sram));
		return -ENOSPC;
	}

	memcpy_toio(pmp->sram_base, fw->data, fw->size);

	/* Map PMP SRAM to its reset vector (0x0) */
	writel(0, pmp->akf_base + AKF_REMAP_IOP_LO);
	writel(0, pmp->akf_base + AKF_REMAP_IOP_HI);
	writel(resource_size(pmp->sram) & 0xffffffff, pmp->akf_base + AKF_REMAP_SZ_LO);
	writel((resource_size(pmp->sram) >> 32) & 0xffffffff,
		pmp->akf_base + AKF_REMAP_SZ_HI);
	writel((uintptr_t)pmp->sram->start & 0xffffffff,
		pmp->akf_base + AKF_REMAP_AP_LO);
	writel(((uintptr_t)pmp->sram->start >> 32) & 0xffffffff,
		pmp->akf_base + AKF_REMAP_AP_HI);
	writel(1, pmp->akf_base + AKF_REMAP_ENABLE);

	/* Unknown */
	writel(0, pmp->akf_base + AKF_REG_80C);
	writel(2, pmp->akf_base + AKF_REG_80C);

	/* Start PMP */
	writel(1, pmp->ctl_base);

	return 0;
}

static const struct dev_pm_domain_attach_data pd_data = {
	.pd_flags = PD_FLAG_DEV_LINK_ON,
};

static int apple_pmp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dev_pm_domain_list *domains;
	struct apple_pmp *pmp;
	struct resource *res;
	int ret;

	ret = dev_pm_domain_attach_list(dev, &pd_data, &domains);
	if (ret < 0)
		return ret;

	pmp = devm_kzalloc(&pdev->dev, sizeof(*pmp), GFP_KERNEL);
	if (!pmp)
		return -ENOMEM;

	pmp->dev = dev;

	pmp->akf_base = devm_platform_ioremap_resource_byname(pdev, "akf");
	if (IS_ERR(pmp->akf_base))
		return PTR_ERR(pmp->akf_base);

	pmp->ctl_base = devm_platform_ioremap_resource_byname(pdev, "control");
	if (IS_ERR(pmp->ctl_base))
		return PTR_ERR(pmp->ctl_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sram");
	if (IS_ERR(res))
		return PTR_ERR(res);

	pmp->sram = res;
	pmp->sram_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pmp->sram_base))
		return PTR_ERR(pmp->sram_base);

	ret = device_property_read_string(dev, "firmware-name", &pmp->fw_name);
	if (ret)
		return dev_err_probe(dev, ret, "unable to get firmware name\n");

	ret = apple_pmp_upload_firmware(pmp);
	if (ret)
		return ret;

	pmp->rtk = devm_apple_rtkit_init(dev, pmp, NULL, 0, &apple_pmp_rtkit_ops);
	if (IS_ERR(pmp->rtk))
		return dev_err_probe(dev, PTR_ERR(pmp->rtk), "Failed to initialize RTKit");

	if (dma_set_mask_and_coherent(pmp->dev, DMA_BIT_MASK(64))) {
		ret = -ENXIO;
		return ret;
	}

	ret = apple_rtkit_wake(pmp->rtk);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to wake PMP");

	return 0;
}

static const struct of_device_id apple_pmp_of_match[] = {
	{ .compatible = "apple,t8015-pmp-v1" },
	{}
};
MODULE_DEVICE_TABLE(of, apple_pmp_of_match);


static struct platform_driver apple_pmp_driver = {
	.driver = {
		.name = "pmp-t8015-v1",
		.of_match_table = apple_pmp_of_match,
	},
	.probe = apple_pmp_probe,
};
module_platform_driver(apple_pmp_driver);

MODULE_AUTHOR("Nick Chan <towinchenmi@gmail.com>");
MODULE_LICENSE("Dual MIT/GPL");
MODULE_DESCRIPTION("Apple T8015 PMP driver");
