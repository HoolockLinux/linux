/* SPDX-License-Identifier: GPL-2.0-only OR MIT */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/soc/apple/tunable.h>

int devm_apple_parse_tunable(struct device *dev, struct device_node *np,
			     struct apple_tunable *tunable, const char *name)
{
	struct property *prop;
	const __le32 *p = NULL;
	int i;

	prop = of_find_property(np, name, NULL);
	if (!prop) {
		dev_err(dev, "tunable %s not found\n", name);
		return -ENOENT;
	}

	if (prop->length % (3 * sizeof(u32)))
		return -EINVAL;

	tunable->sz = prop->length / (3 * sizeof(u32));
	tunable->values = devm_kcalloc(dev, tunable->sz,
				       sizeof(*tunable->values), GFP_KERNEL);
	if (!tunable->values) {
		tunable->sz = 0;
		return -ENOMEM;
	}

	for (i = 0; i < tunable->sz; ++i) {
		p = of_prop_next_u32(prop, p, &tunable->values[i].offset);
		p = of_prop_next_u32(prop, p, &tunable->values[i].mask);
		p = of_prop_next_u32(prop, p, &tunable->values[i].value);
	}

	return 0;
}
EXPORT_SYMBOL(devm_apple_parse_tunable);

void devm_apple_free_tunable(struct device *dev, struct apple_tunable *tunable)
{
	devm_kfree(dev, tunable->values);
	tunable->sz = 0;
}
EXPORT_SYMBOL(devm_apple_free_tunable);

void apple_apply_tunable(void __iomem *regs, struct apple_tunable *tunable)
{
	size_t i;

	for (i = 0; i < tunable->sz; ++i) {
		u32 val, old_val;

		val = old_val = readl_relaxed(regs + tunable->values[i].offset);
		val &= ~tunable->values[i].mask;
		val |= tunable->values[i].value;
		if (val != old_val)
			writel_relaxed(val, regs + tunable->values[i].offset);
	}
}
EXPORT_SYMBOL(apple_apply_tunable);

MODULE_LICENSE("Dual MIT/GPL");
MODULE_AUTHOR("Sven Peter <sven@svenpeter.dev>");
MODULE_DESCRIPTION("Apple Silicon hardware tunable support");
