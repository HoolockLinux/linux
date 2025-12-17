// SPDX-License-Identifier: GPL-2.0-only
/*
 * Dialog DA2XXX RTC driver
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
 * struct da2xxx_rtc_regs -	RTC Hardware registers
 * @control:			RTC Control
 * @counter:			Time Counter
 */

struct da2xxx_rtc_regs {
	u32 control;
	u32 counter;
};

/**
 * struct da2xxx_rtc -  RTC driver internal structure
 * @rtc:		RTC device
 * @regmap:		regmap used to access registers
 * @nvmem_cell:		nvmem cell for offset
 * @regs:		hardware registers
 * @base:		RTC base address
 * @offset:		offset from epoch in seconds
 * @offset_dirty:	offset needs to be stored on shutdown
 */
struct da2xxx_rtc {
	struct rtc_device *rtc;
	struct regmap *regmap;
	struct device *dev;
	struct nvmem_cell *nvmem_cell;
	const struct da2xxx_rtc_regs *regs;
	u32 base;
	int offset;
	bool offset_dirty;
};

static int da2xxx_rtc_read_nvmem_offset(struct da2xxx_rtc *da_rtc)
{
	size_t len;
	void *buf;
	int rc;

	buf = nvmem_cell_read(da_rtc->nvmem_cell, &len);
	if (IS_ERR(buf)) {
		rc = PTR_ERR(buf);
		dev_dbg(da_rtc->dev, "failed to read nvmem offset: %d\n", rc);
		return rc;
	}

	if (len != sizeof(int)) {
		dev_dbg(da_rtc->dev, "unexpected nvmem cell size %zu\n", len);
		kfree(buf);
		return -EINVAL;
	}

	da_rtc->offset = get_unaligned_le32(buf);

	kfree(buf);

	return 0;
}

static int da2xxx_rtc_write_nvmem_offset(struct da2xxx_rtc *da_rtc, int offset)
{
	u8 buf[sizeof(u32)];
	int rc;

	put_unaligned_le32(offset, buf);

	rc = nvmem_cell_write(da_rtc->nvmem_cell, buf, sizeof(buf));
	if (rc < 0) {
		dev_dbg(da_rtc->dev, "failed to write nvmem offset: %d\n", rc);
		return rc;
	}

	return 0;
}

static int da2xxx_rtc_read_raw(struct da2xxx_rtc *da_rtc, u32 *secs)
{
	u64 value;
	int rc;

	rc = regmap_bulk_read(da_rtc->regmap,
		da_rtc->base + da_rtc->regs->counter, &value, RTC_COUNTER_SIZE);
	if (rc)
		return rc;

	*secs = (u32)(value >> 16);
	return 0;
}

static int da2xxx_rtc_update_offset(struct da2xxx_rtc *da_rtc, int secs)
{
	u32 raw_secs;
	int offset;
	int rc;

	rc = da2xxx_rtc_read_raw(da_rtc, &raw_secs);
	if (rc)
		return rc;

	offset = secs - raw_secs;

	if (offset == da_rtc->offset)
		return 0;

	/*
	 * Reduce wear by deferring updates due to clock drift until shutdown.
	 */
	if (abs_diff(offset, da_rtc->offset) < 30) {
		da_rtc->offset_dirty = true;
		goto out;
	}

	rc = da2xxx_rtc_write_nvmem_offset(da_rtc, offset);

	if (rc)
		return rc;

	da_rtc->offset_dirty = false;
out:
	da_rtc->offset = offset;

	return 0;
}

static int da2xxx_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct da2xxx_rtc *da_rtc = dev_get_drvdata(dev);
	int secs;
	int rc;

	secs = rtc_tm_to_time64(tm);

	rc = da2xxx_rtc_update_offset(da_rtc, secs);
	if (rc)
		return rc;

	dev_dbg(dev, "set time: %ptRd %ptRt (%u + %u)\n", tm, tm,
			secs - da_rtc->offset, da_rtc->offset);
	return 0;
}

static int da2xxx_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct da2xxx_rtc *da_rtc = dev_get_drvdata(dev);
	int secs;
	int rc;

	rc = da2xxx_rtc_read_raw(da_rtc, &secs);
	if (rc)
		return rc;

	secs += da_rtc->offset;
	rtc_time64_to_tm(secs, tm);

	dev_dbg(dev, "read time: %ptRd %ptRt (%u + %u)\n", tm, tm,
			secs - da_rtc->offset, da_rtc->offset);
	return 0;
}

static const struct rtc_class_ops da2xxx_rtc_ops = {
	.read_time	= da2xxx_rtc_read_time,
	.set_time	= da2xxx_rtc_set_time,
};

static const struct da2xxx_rtc_regs da2045_rtc_regs = {
	.control = 0x4,
	.counter = 0x6
};

static const struct da2xxx_rtc_regs da2255_rtc_regs = {
	.control = 0x0,
	.counter = 0x2
};

static const struct of_device_id da2xxx_id_table[] = {
	{ .compatible = "dlg,da2045-rtc", .data = &da2045_rtc_regs },
	{ .compatible = "dlg,da2255-rtc", .data = &da2255_rtc_regs },
	{ },
};
MODULE_DEVICE_TABLE(of, da2xxx_id_table);

static int da2xxx_rtc_probe_offset(struct da2xxx_rtc *da_rtc)
{
	int rc;

	da_rtc->nvmem_cell = devm_nvmem_cell_get(da_rtc->dev, "rtc_offset");
	if (IS_ERR(da_rtc->nvmem_cell)) {
		rc = PTR_ERR(da_rtc->nvmem_cell);
		if (rc != -ENOENT)
			return rc;
		da_rtc->nvmem_cell = NULL;
	} else {
		return da2xxx_rtc_read_nvmem_offset(da_rtc);
	}

	return 0;
}

static int da2xxx_rtc_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	const struct of_device_id *match;
	struct da2xxx_rtc *da_rtc;
	int rc;

	match = of_match_node(da2xxx_id_table, node);
	if (!match)
		return -ENXIO;

	da_rtc = devm_kzalloc(&pdev->dev, sizeof(*da_rtc), GFP_KERNEL);
	if (da_rtc == NULL)
		return -ENOMEM;

	rc = of_property_read_u32_index(node, "reg", 0, &da_rtc->base);
	if (rc)
		return -ENXIO;

	da_rtc->dev = &pdev->dev;
	da_rtc->regs = match->data;

	da_rtc->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!da_rtc->regmap)
		return -ENXIO;

	rc = da2xxx_rtc_probe_offset(da_rtc);
	if (rc)
		return rc;

	platform_set_drvdata(pdev, da_rtc);

	da_rtc->rtc = devm_rtc_allocate_device(&pdev->dev);
	if (IS_ERR(da_rtc->rtc))
		return PTR_ERR(da_rtc->rtc);

	da_rtc->rtc->ops = &da2xxx_rtc_ops;
	da_rtc->rtc->range_min = INT_MIN;
	da_rtc->rtc->range_max = INT_MAX;

	return devm_rtc_register_device(da_rtc->rtc);
}

static void da2xxx_shutdown(struct platform_device *pdev)
{
	struct da2xxx_rtc *da_rtc = platform_get_drvdata(pdev);

	if (da_rtc->offset_dirty)
		da2xxx_rtc_write_nvmem_offset(da_rtc, da_rtc->offset);
}

static struct platform_driver da2xxx_rtc_driver = {
	.probe		= da2xxx_rtc_probe,
	.shutdown	= da2xxx_shutdown,
	.driver	= {
		.name		= "rtc-da2xxx",
		.of_match_table	= da2xxx_id_table,
	},
};

module_platform_driver(da2xxx_rtc_driver);

MODULE_ALIAS("platform:rtc-da2xxx");
MODULE_DESCRIPTION("Dialog DA2XXX RTC driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Nick Chan <towinchenmi@gmail.com>");
