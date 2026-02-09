/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Apple Silicon hardware tunable support
 *
 * Each tunable is a list with each entry containing a offset into the MMIO
 * region, a mask of bits to be cleared and a set of bits to be set. These
 * tunables are passed along by the previous boot stages and vary from device
 * to device such that they cannot be hardcoded in the individual drivers.
 *
 * Copyright (C) The Asahi Linux Contributors
 */

#ifndef _LINUX_SOC_APPLE_TUNABLE_H_
#define _LINUX_SOC_APPLE_TUNABLE_H_

#include <linux/device.h>
#include <linux/types.h>

/*
 * Struct to store a Apple Silicon hardware tunable.
 *
 * sz: number of [offset, mask, value] tuples stored in values.
 * values: array containing the hardware tunables.
 */
struct apple_tunable {
	size_t sz;
	struct {
		u32 offset;
		u32 mask;
		u32 value;
	} * values;
};

/*
 * Parse an array of hardware tunables from the device tree.
 *
 * Return 0 on sucess, -ENOMEM if the allocation failed and -ENOENT if the
 * tunable could not be found or was in an invalid format.
 *
 * dev: Device node used for devm_kzalloc internally.
 * np: Device node which contains the tunable array.
 * tunable: Pointer to where the parsed tunables will be stored.
 * name: Name of the device tree property which contains the tunables.
 */
int devm_apple_parse_tunable(struct device *dev, struct device_node *np,
			     struct apple_tunable *tunable, const char *name);

/*
 * Apply a previously loaded hardware tunable.
 *
 * regs: MMIO to which the tunable will be applied.
 * tunable: Pointer to the tunable.
 */
void apple_apply_tunable(void __iomem *regs, struct apple_tunable *tunable);

/*
 * Manually frees a previous allocated tunable.
 *
 * dev: Device node used for devm_apple_parse_tunable
 * tunable: Tunable allocaated by devm_apple_parse_tunable
 */
void devm_apple_free_tunable(struct device *dev, struct apple_tunable *tunable);

#endif
