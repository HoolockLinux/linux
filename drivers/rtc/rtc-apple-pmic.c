// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple PMIC RTC driver
 *
 * Based on rtc-pm8xxx.c
 *
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2023, Linaro Limited
 * Copyright (c) 2025, Nick Chan
 */
#include <linux/of.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/init.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_wakeirq.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>

#include <asm/byteorder.h>

#define RTC_COUNTER_SIZE 6

/**
 * struct apple_pmic_rtc_regs -	RTC Hardware registers
 * @control:			RTC Control
 * @counter:			Time Counter
 */

struct apple_pmic_rtc_regs {
	u32 control;
	u32 counter;
};

/**
 * struct apple_pmic_rtc -  RTC driver internal structure
 * @rtc:		RTC device
 * @regmap:		regmap used to access registers
 * @nvmem_cell:		nvmem cell for offset
 * @regs:		hardware registers
 * @base:		RTC base address
 * @offset:		offset from epoch in seconds
 * @offset_dirty:	offset needs to be stored on shutdown
 */
struct apple_pmic_rtc {
	struct rtc_device *rtc;
	struct regmap *regmap;
	struct device *dev;
	struct nvmem_cell *nvmem_cell;
	const struct apple_pmic_rtc_regs *regs;
	u32 base;
	int offset;
	bool offset_dirty;
};

static int apple_pmic_rtc_read_nvmem_offset(struct apple_pmic_rtc *ap_rtc)
{
	size_t len;
	void *buf;
	int rc;

	buf = nvmem_cell_read(ap_rtc->nvmem_cell, &len);
	if (IS_ERR(buf)) {
		rc = PTR_ERR(buf);
		dev_dbg(ap_rtc->dev, "failed to read nvmem offset: %d\n", rc);
		return rc;
	}

	if (len != sizeof(int)) {
		dev_dbg(ap_rtc->dev, "unexpected nvmem cell size %zu\n", len);
		kfree(buf);
		return -EINVAL;
	}

	ap_rtc->offset = get_unaligned_le32(buf);

	kfree(buf);

	return 0;
}

static int apple_pmic_rtc_write_nvmem_offset(struct apple_pmic_rtc *ap_rtc, int offset)
{
	u8 buf[sizeof(u32)];
	int rc;

	put_unaligned_le32(offset, buf);

	rc = nvmem_cell_write(ap_rtc->nvmem_cell, buf, sizeof(buf));
	if (rc < 0) {
		dev_dbg(ap_rtc->dev, "failed to write nvmem offset: %d\n", rc);
		return rc;
	}

	return 0;
}

static int apple_pmic_rtc_read_raw(struct apple_pmic_rtc *ap_rtc, u32 *secs)
{
	u64 value;
	int rc;

	rc = regmap_bulk_read(ap_rtc->regmap,
		ap_rtc->base + ap_rtc->regs->counter, &value, RTC_COUNTER_SIZE);
	if (rc)
		return rc;

	*secs = (u32)(value >> 16);
	return 0;
}

static int apple_pmic_rtc_update_offset(struct apple_pmic_rtc *ap_rtc, int secs)
{
	u32 raw_secs;
	int offset;
	int rc;

	rc = apple_pmic_rtc_read_raw(ap_rtc, &raw_secs);
	if (rc)
		return rc;

	offset = secs - raw_secs;

	if (offset == ap_rtc->offset)
		return 0;

	/*
	 * Reduce wear by deferring updates due to clock drift until shutdown.
	 */
	if (abs_diff(offset, ap_rtc->offset) < 30) {
		ap_rtc->offset_dirty = true;
		goto out;
	}

	rc = apple_pmic_rtc_write_nvmem_offset(ap_rtc, offset);

	if (rc)
		return rc;

	ap_rtc->offset_dirty = false;
out:
	ap_rtc->offset = offset;

	return 0;
}

static int apple_pmic_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct apple_pmic_rtc *ap_rtc = dev_get_drvdata(dev);
	int secs;
	int rc;

	secs = rtc_tm_to_time64(tm);

	rc = apple_pmic_rtc_update_offset(ap_rtc, secs);
	if (rc)
		return rc;

	dev_dbg(dev, "set time: %ptRd %ptRt (%u + %u)\n", tm, tm,
			secs - ap_rtc->offset, ap_rtc->offset);
	return 0;
}

static int apple_pmic_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct apple_pmic_rtc *ap_rtc = dev_get_drvdata(dev);
	int secs;
	int rc;

	rc = apple_pmic_rtc_read_raw(ap_rtc, &secs);
	if (rc)
		return rc;

	secs += ap_rtc->offset;
	rtc_time64_to_tm(secs, tm);

	dev_dbg(dev, "read time: %ptRd %ptRt (%u + %u)\n", tm, tm,
			secs - ap_rtc->offset, ap_rtc->offset);
	return 0;
}

static const struct rtc_class_ops apple_pmic_rtc_ops = {
	.read_time	= apple_pmic_rtc_read_time,
	.set_time	= apple_pmic_rtc_set_time,
};

static const struct apple_pmic_rtc_regs amber_regs = {
	.control = 0x4,
	.counter = 0x6
};

static const struct apple_pmic_rtc_regs antigua_regs = {
	.control = 0x0,
	.counter = 0x2
};

static const struct of_device_id apple_pmic_id_table[] = {
	{ .compatible = "apple,amber-pmic-rtc", .data = &amber_regs },
	{ .compatible = "apple,antigua-pmic-rtc", .data = &antigua_regs },
	{ },
};
MODULE_DEVICE_TABLE(of, apple_pmic_id_table);

static int apple_pmic_rtc_probe_offset(struct apple_pmic_rtc *ap_rtc)
{
	int rc;

	ap_rtc->nvmem_cell = devm_nvmem_cell_get(ap_rtc->dev, "rtc_offset");
	if (IS_ERR(ap_rtc->nvmem_cell)) {
		rc = PTR_ERR(ap_rtc->nvmem_cell);
		if (rc != -ENOENT)
			return rc;
		ap_rtc->nvmem_cell = NULL;
	} else {
		return apple_pmic_rtc_read_nvmem_offset(ap_rtc);
	}

	return 0;
}

static int apple_pmic_rtc_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	const struct of_device_id *match;
	struct apple_pmic_rtc *ap_rtc;
	int rc;

	match = of_match_node(apple_pmic_id_table, node);
	if (!match)
		return -ENXIO;

	ap_rtc = devm_kzalloc(&pdev->dev, sizeof(*ap_rtc), GFP_KERNEL);
	if (ap_rtc == NULL)
		return -ENOMEM;

	rc = of_property_read_u32_index(node, "reg", 0, &ap_rtc->base);
	if (rc)
		return -ENXIO;

	ap_rtc->dev = &pdev->dev;
	ap_rtc->regs = match->data;

	ap_rtc->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!ap_rtc->regmap)
		return -ENXIO;

	rc = apple_pmic_rtc_probe_offset(ap_rtc);
	if (rc)
		return rc;

	platform_set_drvdata(pdev, ap_rtc);

	ap_rtc->rtc = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(ap_rtc->rtc))
		return PTR_ERR(ap_rtc->rtc);

	ap_rtc->rtc->ops = &apple_pmic_rtc_ops;
	ap_rtc->rtc->range_min = INT_MIN;
	ap_rtc->rtc->range_max = INT_MAX;

	return devm_rtc_register_device(ap_rtc->rtc);
}

static void apple_pmic_shutdown(struct platform_device *pdev)
{
	struct apple_pmic_rtc *ap_rtc = platform_get_drvdata(pdev);

	if (ap_rtc->offset_dirty)
		apple_pmic_rtc_write_nvmem_offset(ap_rtc, ap_rtc->offset);
}

static struct platform_driver apple_pmic_rtc_driver = {
	.probe		= apple_pmic_rtc_probe,
	.shutdown	= apple_pmic_shutdown,
	.driver	= {
		.name		= "rtc-apple-pmic",
		.of_match_table	= apple_pmic_id_table,
	},
};

module_platform_driver(apple_pmic_rtc_driver);

MODULE_ALIAS("platform:rtc-apple-pmic");
MODULE_DESCRIPTION("Apple PMIC RTC driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Nick Chan <towinchenmi@gmail.com>");
