/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <shell/shell.h>
#include <init.h>
#include <string.h>
#include <device.h>

extern struct device __device_start[];
extern struct device __device_PRE_KERNEL_1_start[];
extern struct device __device_PRE_KERNEL_2_start[];
extern struct device __device_POST_KERNEL_start[];
extern struct device __device_APPLICATION_start[];
extern struct device __device_end[];

#ifdef CONFIG_SMP
extern struct device __device_SMP_start[];
#endif

static const struct device *levels[] = {
	__device_PRE_KERNEL_1_start,
	__device_PRE_KERNEL_2_start,
	__device_POST_KERNEL_start,
	__device_APPLICATION_start,
#ifdef CONFIG_SMP
	__device_SMP_start,
#endif
	/* End marker */
	__device_end,
};

static bool device_get_config_level(const struct shell *shell, int level)
{
	const struct device *dev;
	bool devices = false;

	for (dev = levels[level]; dev < levels[level+1]; dev++) {
		if (z_device_ready(dev)) {
			devices = true;

			shell_fprintf(shell, SHELL_NORMAL, "- %s\n", dev->name);
		}
	}
	return devices;
}

static int cmd_device_levels(const struct shell *shell,
			      size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	bool ret;

	shell_fprintf(shell, SHELL_NORMAL, "POST_KERNEL:\n");
	ret = device_get_config_level(shell, _SYS_INIT_LEVEL_POST_KERNEL);
	if (ret == false) {
		shell_fprintf(shell, SHELL_NORMAL, "- None\n");
	}

	shell_fprintf(shell, SHELL_NORMAL, "APPLICATION:\n");
	ret = device_get_config_level(shell, _SYS_INIT_LEVEL_APPLICATION);
	if (ret == false) {
		shell_fprintf(shell, SHELL_NORMAL, "- None\n");
	}

	shell_fprintf(shell, SHELL_NORMAL, "PRE KERNEL 1:\n");
	ret = device_get_config_level(shell, _SYS_INIT_LEVEL_PRE_KERNEL_1);
	if (ret == false) {
		shell_fprintf(shell, SHELL_NORMAL, "- None\n");
	}

	shell_fprintf(shell, SHELL_NORMAL, "PRE KERNEL 2:\n");
	ret = device_get_config_level(shell, _SYS_INIT_LEVEL_PRE_KERNEL_2);
	if (ret == false) {
		shell_fprintf(shell, SHELL_NORMAL, "- None\n");
	}

	return 0;
}

static int cmd_device_list(const struct shell *shell,
			      size_t argc, char **argv)
{
	const struct device *dev;
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_fprintf(shell, SHELL_NORMAL, "devices:\n");

	for (dev = __device_start; dev != __device_end; dev++) {
		if (!z_device_ready(dev)) {
			continue;
		}

		shell_fprintf(shell, SHELL_NORMAL, "- %s", dev->name);

#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
		uint32_t state = DEVICE_PM_ACTIVE_STATE;
		int err;

		err = device_get_power_state(dev, &state);
		if (!err) {
			shell_fprintf(shell, SHELL_NORMAL, " (%s)",
				      device_pm_state_str(state));
		}
#endif /* CONFIG_DEVICE_POWER_MANAGEMENT */
		shell_fprintf(shell, SHELL_NORMAL, "\n");
	}

	return 0;
}


SHELL_STATIC_SUBCMD_SET_CREATE(sub_device,
	SHELL_CMD(levels, NULL, "List configured devices by levels", cmd_device_levels),
	SHELL_CMD(list, NULL, "List configured devices", cmd_device_list),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(device, &sub_device, "Device commands", NULL);
