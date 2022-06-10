/*
 * Copyright (c) 2019 Intel Corporation
 * Copyright (c) 2019 Stephanos Ioannidis <root@stephanos.io>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ARCH_X86_ARCH_INLINES_H_
#define ZEPHYR_INCLUDE_ARCH_X86_ARCH_INLINES_H_

#ifndef _ASMLANGUAGE

#if defined(CONFIG_X86_64)

#include <arch/x86/intel64/thread.h>
#include <kernel_structs.h>

static inline struct _cpu *arch_curr_cpu(void)
{
	struct _cpu *cpu;

	__asm__ volatile("movq %%gs:(%c1), %0"
			 : "=r" (cpu)
			 : "i" (offsetof(x86_tss64_t, cpu)));

	return cpu;
}

#endif /* CONFIG_X86_64 */

#endif /* !_ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_ARCH_X86_ARCH_INLINES_H_ */
