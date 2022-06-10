/*
 * Copyright (c) 2020 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <arch/x86/arch.h>
#include <kernel.h>
#include <sys_clock.h>
#include <timing/timing.h>

static uint64_t tsc_freq;

void timing_x86_init(void)
{
	uint32_t cyc_start = k_cycle_get_32();
	uint64_t tsc_start = z_tsc_read();

	k_busy_wait(10 * USEC_PER_MSEC);

	uint32_t cyc_end = k_cycle_get_32();
	uint64_t tsc_end = z_tsc_read();

	uint64_t cyc_freq = sys_clock_hw_cycles_per_sec();

	/*
	 * cycles are in 32-bit, and delta must be
	 * calculated in 32-bit percision. Or it would
	 * wrapping around in 64-bit.
	 */
	uint64_t dcyc = (uint32_t)cyc_end - (uint32_t)cyc_start;

	uint64_t dtsc = tsc_end - tsc_start;

	tsc_freq = (cyc_freq * dtsc) / dcyc;
}

uint64_t timing_x86_freq_get(void)
{
	return tsc_freq;
}

void timing_init(void)
{
	timing_x86_init();
}
void timing_start(void)
{
}

void timing_stop(void)
{
}

timing_t timing_counter_get(void)
{
	return k_cycle_get_32();
}

uint64_t timing_cycles_get(volatile timing_t *const start,
				  volatile timing_t *const end)
{
	return (*end - *start);
}


uint64_t timing_freq_get(void)
{
	return timing_x86_freq_get();
}

uint64_t timing_cycles_to_ns(uint64_t cycles)
{
	return k_cyc_to_ns_floor64(cycles);
}

uint64_t timing_cycles_to_ns_avg(uint64_t cycles, uint32_t count)
{
	return timing_cycles_to_ns(cycles) / count;
}

uint32_t timing_freq_get_mhz(void)
{
	return (uint32_t)(timing_freq_get() / 1000000);
}
