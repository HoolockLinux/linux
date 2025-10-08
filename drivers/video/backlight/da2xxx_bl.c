// SPDX-License-Identifier: GPL-2.0 OR MIT
/*
 * Dialog DA2089/DA2207/DA2257/DA2400 PMIC Backlight Driver
 *
 * Copyright (c) 2025 Nick Chan <towinchenmi@gmail.com>
 */

#include <linux/backlight.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/platform_device.h>

#define DIALOG_DA2XXX_BL_MAX_BRIGHTNESS 2047

enum dialog_hw_type {
	DA2089,
	DA2257
};

struct dialog_da2xxx_bl_hw {
	enum dialog_hw_type type;
};

struct dialog_da2xxx_bl {
	enum dialog_hw_type type;
	struct regmap *regmap;
	u32 base;
};

static int dialog_da2xxx_bl_update_status(struct backlight_device *bl)
{
	struct dialog_da2xxx_bl *data = bl_get_data(bl);

	int brightness = backlight_get_brightness(bl);

	u8 cmd[2];

	switch (data->type) {
		case DA2089:
			cmd[0] = (brightness >> 3) & 0xff;
			cmd[1] = brightness & 0x7;
			break;
		case DA2257:
			cmd[0] = brightness & 0xff;
			cmd[1] = (brightness >> 8) & 0x7;
			break;
	}

	return regmap_bulk_write(data->regmap, data->base, cmd, 2);
}

static int dialog_da2xxx_bl_get_brightness(struct backlight_device *bl)
{
	struct dialog_da2xxx_bl *data = bl_get_data(bl);

	u8 cmd[2];

	int ret = regmap_bulk_read(data->regmap, data->base, &cmd, 2);

	if (ret)
		return ret;

	switch (data->type) {
		case DA2089:
			return (cmd[0] << 3) | (cmd[1] & 7);
		case DA2257:
			return ((cmd[1] & 7) << 8) | (cmd[0] & 0xff);
	}

	unreachable();
}

static const struct backlight_ops dialog_da2xxx_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.get_brightness = dialog_da2xxx_bl_get_brightness,
	.update_status	= dialog_da2xxx_bl_update_status
};

static int dialog_da2xxx_bl_probe(struct platform_device *dev)
{
	struct backlight_device *bl;
	struct backlight_properties props;
	struct dialog_da2xxx_bl *data;

	data = devm_kzalloc(&dev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->regmap = dev_get_regmap(dev->dev.parent, NULL);
	if (!data->regmap)
		return -ENODEV;

	if (of_property_read_u32(dev->dev.of_node, "reg", &data->base))
		return -ENODEV;

	const struct dialog_da2xxx_bl_hw *hw = of_device_get_match_data(&dev->dev);

	data->type = hw->type;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = DIALOG_DA2XXX_BL_MAX_BRIGHTNESS;
	props.scale = BACKLIGHT_SCALE_LINEAR;

	bl = devm_backlight_device_register(&dev->dev, dev->name, &dev->dev,
					data, &dialog_da2xxx_bl_ops, &props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);

	platform_set_drvdata(dev, data);

	bl->props.brightness = dialog_da2xxx_bl_get_brightness(bl);

	return 0;
}

static struct dialog_da2xxx_bl_hw da2089_bl_hw = {
	.type = DA2089
};

static struct dialog_da2xxx_bl_hw da2257_bl_hw = {
	.type = DA2257
};

static const struct of_device_id dialog_da2xxx_bl_of_match[] = {
	{ .compatible = "dlg,da2089-bl", .data = &da2089_bl_hw },
	{ .compatible = "dlg,da2257-bl", .data = &da2257_bl_hw },
	{},
};

MODULE_DEVICE_TABLE(of, dialog_da2xxx_bl_of_match);

static struct platform_driver dialog_da2xxx_bl_driver = {
	.driver		= {
		.name	= "apple-dialog-bl",
		.of_match_table = dialog_da2xxx_bl_of_match
	},
	.probe		= dialog_da2xxx_bl_probe,
};

module_platform_driver(dialog_da2xxx_bl_driver);

MODULE_DESCRIPTION("Dialog DA2089/DA2207/DA2257/DA2400 PMIC Backlight Driver");
MODULE_AUTHOR("Nick Chan <towinchenmi@gmail.com>");
MODULE_LICENSE("Dual MIT/GPL");
