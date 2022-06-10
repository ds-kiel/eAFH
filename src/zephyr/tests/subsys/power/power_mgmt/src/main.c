/*
 * Copyright (c) 2020 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sys/printk.h>
#include <zephyr.h>
#include <zephyr/types.h>
#include <ztest.h>
#include <ksched.h>
#include <kernel.h>
#include <power/power.h>
#include "dummy_driver.h"

#define SLEEP_MSEC 100
#define SLEEP_TIMEOUT K_MSEC(SLEEP_MSEC)

/* for checking power suspend and resume order between system and devices */
static bool enter_low_power;
static bool notify_app_entry;
static bool notify_app_exit;
static bool set_sys_power;
static bool leave_idle;
static bool idle_entered;

static const struct device *dev;
static struct dummy_driver_api *api;
/*
 * Weak power hook functions. Used on systems that have not implemented
 * power management.
 */
__weak void sys_set_power_state(enum power_states state)
{
	/* at this point, sys_pm_notify_power_state_entry() implemented in
	 * this file has been called and set_sys_power should have been set
	 */
	zassert_true(set_sys_power == true,
		     "Notification to enter suspend was not sent to the App");

	/* this function is called after devices enter low power state */
	uint32_t device_power_state;
	/* at this point, devices have been deactivated */
	device_get_power_state(dev, &device_power_state);
	zassert_false(device_power_state == DEVICE_PM_ACTIVE_STATE, NULL);

	/* this function is called when system entering low power state, so
	 * parameter state should not be SYS_POWER_STATE_ACTIVE
	 */
	zassert_false(state == SYS_POWER_STATE_ACTIVE,
		      "Entering low power state with a wrong parameter");
}

__weak void _sys_pm_power_state_exit_post_ops(enum power_states state)
{
	/* _sys_suspend is entered with irq locked
	 * unlock irq before leave _sys_suspend
	 */
	irq_unlock(0);
}

__weak bool sys_pm_policy_low_power_devices(enum power_states pm_state)
{
	return sys_pm_is_sleep_state(pm_state);
}

/* Our PM policy handler */
enum power_states sys_pm_policy_next_state(int ticks)
{
	enum power_states state;

	/* make sure this is idle thread */
	zassert_true(z_is_idle_thread_object(_current), NULL);
	zassert_true(ticks == _kernel.idle, NULL);
	idle_entered = true;

	if (enter_low_power) {
		enter_low_power = false;
		notify_app_entry = true;
		state = SYS_POWER_STATE_SLEEP_1;
	} else {
		/* only test sys_pm_policy_next_state()
		 * no PM operation done
		 */
		state = SYS_POWER_STATE_ACTIVE;
	}
	return state;
}

/* implement in application, called by idle thread */
void sys_pm_notify_power_state_entry(enum power_states state)
{
	uint32_t device_power_state;

	/* enter suspend */
	zassert_true(notify_app_entry == true,
		     "Notification to enter suspend was not sent to the App");
	zassert_true(z_is_idle_thread_object(_current), NULL);
	zassert_equal(state, SYS_POWER_STATE_SLEEP_1, NULL);

	/* at this point, devices are active */
	device_get_power_state(dev, &device_power_state);
	zassert_equal(device_power_state, DEVICE_PM_ACTIVE_STATE, NULL);
	set_sys_power = true;
	notify_app_exit = true;
}

/* implement in application, called by idle thread */
void sys_pm_notify_power_state_exit(enum power_states state)
{
	uint32_t device_power_state;

	/* leave suspend */
	zassert_true(notify_app_exit == true,
		     "Notification to leave suspend was not sent to the App");
	zassert_true(z_is_idle_thread_object(_current), NULL);
	zassert_equal(state, SYS_POWER_STATE_SLEEP_1, NULL);

	/* at this point, devices are active again*/
	device_get_power_state(dev, &device_power_state);
	zassert_equal(device_power_state, DEVICE_PM_ACTIVE_STATE, NULL);
	leave_idle = true;

}

/*
 * @brief test power idle
 *
 * @details
 *  - The global idle routine executes when no other work is available
 *  - The idle routine provide a timeout parameter to the suspend routine
 *    indicating the amount of time guaranteed to expire before the next
 *    timeout, sys_pm_policy_next_state() handle this parameter.
 *  - In this case, sys_pm_policy_next_sate() return SYS_POWER_STATE_ACTIVE,
 *    so there is no low power operation happen.
 *
 * @see sys_pm_policy_next_state()
 *
 * @ingroup power_tests
 */
void test_power_idle(void)
{
	TC_PRINT("give way to idle thread\n");
	k_sleep(SLEEP_TIMEOUT);
	zassert_true(idle_entered, "Never entered idle thread");
}

/*
 * @brief test power state transition
 *
 * @details
 *  - The system support control of power state ordering between
 *    subsystems and devices
 *  - The application can control system power state transitions in idle thread
 *    through sys_pm_notify_power_state_entry and sys_pm_notify_power_state_exit
 *
 * @see sys_pm_notify_power_state_entry(), sys_pm_notify_power_state_exit()
 *
 * @ingroup power_tests
 */
void test_power_state_trans(void)
{
	enter_low_power = true;
	/* give way to idle thread */
	k_sleep(SLEEP_TIMEOUT);
	zassert_true(leave_idle, NULL);
}

/*
 * @brief notification between system and device
 *
 * @details
 *  - device driver notify its power state change by device_pm_get and
 *    device_pm_put
 *  - system inform device system power state change through device interface
 *    device_pm_control
 *
 * @see device_pm_get(), device_pm_put(), device_set_power_state(),
 *      device_get_power_state()
 *
 * @ingroup power_tests
 */
void test_power_state_notification(void)
{
	uint32_t device_power_state;

	device_get_power_state(dev, &device_power_state);
	zassert_equal(device_power_state, DEVICE_PM_ACTIVE_STATE, NULL);

	api->close(dev);
	device_get_power_state(dev, &device_power_state);
	zassert_equal(device_power_state, DEVICE_PM_SUSPEND_STATE, NULL);
	/* reopen device as it will be closed in teardown */
	api->open(dev);
}

void test_setup(void)
{
	int ret;

	dev = device_get_binding(DUMMY_DRIVER_NAME);
	api = (struct dummy_driver_api *)dev->api;
	ret = api->open(dev);
	zassert_true(ret == 0, "Fail to open device");
}

void test_teardown(void)
{
	api->close(dev);
}

void test_main(void)
{
	ztest_test_suite(power_management_test,
			 ztest_1cpu_unit_test(test_power_idle),
			 ztest_unit_test_setup_teardown(test_power_state_trans,
							test_setup,
							test_teardown),
			 ztest_unit_test_setup_teardown(
						test_power_state_notification,
						test_setup,
						test_teardown));
	ztest_run_test_suite(power_management_test);
}
