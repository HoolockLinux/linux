// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * Apple PMIC Backlight Driver
 *
 * Copyright (c) 2025 Nick Chan <towinchenmi@gmail.com>
 */

#include <linux/backlight.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>

#define APPLE_PMIC_BL_MAX_BRIGHTNESS 2047

enum apple_pmic_type {
	PMIC_TYPE_ANYA,
	PMIC_TYPE_ARIA
};

struct apple_pmic_bl {
	struct regmap *regmap;
	u32 base;
	enum apple_pmic_type type;
};

static int apple_pmic_bl_update_status(struct backlight_device *bl)
{
	struct apple_pmic_bl *data = bl_get_data(bl);

	int brightness = backlight_get_brightness(bl);

	u8 cmd[2];

	switch (data->type) {
		case PMIC_TYPE_ANYA:
			cmd[0] = (brightness >> 3) & 0xff;
			cmd[1] = brightness & 0x7;
			break;
		case PMIC_TYPE_ARIA:
			cmd[0] = brightness & 0xff;
			cmd[1] = (brightness >> 8) & 0x7;
			break;
	}

	return regmap_bulk_write(data->regmap, data->base, cmd, 2);
}

static int apple_pmic_bl_get_brightness(struct backlight_device *bl)
{
	struct apple_pmic_bl *data = bl_get_data(bl);

	u8 cmd[2];

	int ret = regmap_bulk_read(data->regmap, data->base, &cmd, 2);

	if (ret)
		return ret;

	switch (data->type) {
		case PMIC_TYPE_ANYA:
			return (cmd[0] << 3) | (cmd[1] & 7);
		case PMIC_TYPE_ARIA:
			return ((cmd[1] & 7) << 8) | (cmd[0] & 0xff);
	}
}

static const struct backlight_ops apple_pmic_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.get_brightness = apple_pmic_bl_get_brightness,
	.update_status	= apple_pmic_bl_update_status
};

static int apple_pmic_bl_probe(struct platform_device *dev)
{
	struct backlight_device *bl;
	struct backlight_properties props;
	struct apple_pmic_bl *data;

	data = devm_kzalloc(&dev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->regmap = dev_get_regmap(dev->dev.parent, NULL);
	if (!data->regmap)
		return -ENODEV;

	if (of_property_read_u32(dev->dev.of_node, "reg", &data->base))
		return -ENODEV;

	data->type = (enum apple_pmic_type)of_device_get_match_data(&dev->dev);

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = APPLE_PMIC_BL_MAX_BRIGHTNESS;
	props.scale = BACKLIGHT_SCALE_LINEAR;

	bl = devm_backlight_device_register(&dev->dev, dev->name, &dev->dev,
					data, &apple_pmic_bl_ops, &props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);

	platform_set_drvdata(dev, data);

	bl->props.brightness = apple_pmic_bl_get_brightness(bl);

	return 0;
}

static const struct of_device_id apple_pmic_bl_of_match[] = {
	{ .compatible = "apple,anya-pmic-bl", .data = (void*)PMIC_TYPE_ANYA },
	{ .compatible = "apple,aria-pmic-bl", .data = (void*)PMIC_TYPE_ARIA },
	{},
};

MODULE_DEVICE_TABLE(of, apple_pmic_bl_of_match);

static struct platform_driver apple_pmic_bl_driver = {
	.driver		= {
		.name	= "apple-pmic-bl",
		.of_match_table = apple_pmic_bl_of_match
	},
	.probe		= apple_pmic_bl_probe,
};

module_platform_driver(apple_pmic_bl_driver);

MODULE_DESCRIPTION("Apple PMIC Backlight Driver");
MODULE_AUTHOR("Nick Chan <towinchenmi@gmail.com>");
MODULE_LICENSE("Dual MIT/GPL");
