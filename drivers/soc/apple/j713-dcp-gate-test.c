// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * TEST ONLY, not for upstream: apply the J713 display power-down overlay to
 * the boot loader's device tree before platform devices are created, when
 * the kernel command line has j713_dcp_gate_test_overlay=1.
 */
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/of.h>

extern u8 __dtbo_j713_dcp_gate_test_begin[];
extern u8 __dtbo_j713_dcp_gate_test_end[];

static bool overlay;
core_param(j713_dcp_gate_test_overlay, overlay, bool, 0444);

static int __init j713_dcp_gate_test_apply(void)
{
	int id = -1, ret;

	if (!overlay || !of_machine_is_compatible("apple,j713"))
		return 0;
	ret = of_overlay_fdt_apply(__dtbo_j713_dcp_gate_test_begin,
				   __dtbo_j713_dcp_gate_test_end -
				   __dtbo_j713_dcp_gate_test_begin, &id, NULL);
	pr_info("j713-dcp-gate-test: overlay %s (%d, id %d)\n",
		ret ? "failed" : "applied", ret, id);
	return 0;
}
arch_initcall(j713_dcp_gate_test_apply);
