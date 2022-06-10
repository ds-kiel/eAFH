/*
 * Copyright (c) 2011-2016 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <ztest.h>
#include <debug/object_tracing.h>

extern void test_obj_tracing(void);

#define STSIZE (1024 + CONFIG_TEST_EXTRA_STACKSIZE)
#define N_PHILOSOPHERS  5

#define TOTAL_TEST_NUMBER 2
#define ZTEST_THREADS_CREATED 1
#define TOTAL_THREADS (N_PHILOSOPHERS + ZTEST_THREADS_CREATED)
#define TOTAL_OBJECTS (N_PHILOSOPHERS)

#define OBJ_LIST_NAME k_sem
#define OBJ_LIST_TYPE struct k_sem

#define FORK(x) (&forks[x])
#define TAKE(x) k_sem_take(x, K_FOREVER)
#define GIVE(x) k_sem_give(x)

#define RANDDELAY(x) k_msleep(10 * (x) + 1)

static int initial_count;

/* Must account for:
 *	N Philosopher threads
 *	1 Object monitor thread
 *	1 System idle thread
 *	1 System workqueue thread
 */

void *force_sys_work_q_in = (void *)&k_sys_work_q;

K_THREAD_STACK_ARRAY_DEFINE(phil_stack, N_PHILOSOPHERS, STSIZE);
static struct k_thread phil_data[N_PHILOSOPHERS];
K_THREAD_STACK_DEFINE(mon_stack, STSIZE);
static struct k_thread mon_data;
struct k_sem forks[N_PHILOSOPHERS];

K_SEM_DEFINE(f3, -5, 1);

/**
 * @brief Object Tracing Tests
 * @defgroup kernel_objtracing_tests Object Tracing Tests
 * @ingroup all_tests
 * @{
 * @}
 */
static inline void thread_list_cb(const struct k_thread *thread, void *data)
{
	int *ctr = data;

	if (thread->base.prio == -1) {
		TC_PRINT("PREMPT: %p OPTIONS: 0x%02x, STATE: 0x%02x\n",
			 thread,
			 thread->base.user_options,
			 thread->base.thread_state);
	} else {
		TC_PRINT("COOP: %p OPTIONS: 0x%02x, STATE: 0x%02x\n",
			 thread,
			 thread->base.user_options,
			 thread->base.thread_state);
	}
	(*ctr)++;
}

static inline int thread_monitor(void)
{
	int obj_counter = 0;

	k_thread_foreach(thread_list_cb, &obj_counter);
	TC_PRINT("THREAD QUANTITY: %d\n", obj_counter);

	return obj_counter;
}

static void object_monitor(void)
{
	int obj_counter = 0;
	int thread_counter = 0, sem = 0;

	void *obj_list   = NULL;

	k_sem_take(&f3, K_NO_WAIT);
	/* ztest use one semaphore so use one count less than expected to pass
	 * test
	 */
	obj_list   = SYS_TRACING_HEAD(OBJ_LIST_TYPE, OBJ_LIST_NAME);
	while (obj_list != NULL) {
		TC_PRINT("SEMAPHORE REF: %p\n", obj_list);
		obj_list = SYS_TRACING_NEXT(OBJ_LIST_TYPE, OBJ_LIST_NAME,
					    obj_list);

		for (sem = 0; sem < N_PHILOSOPHERS; sem++) {
			if (obj_list == &forks[sem] || obj_list == &f3) {
				obj_counter++;
				break;
			}
		}
	}
	TC_PRINT("SEMAPHORE QUANTITY: %d\n", obj_counter);

	thread_counter += thread_monitor();

	zassert_true(((thread_counter == (TOTAL_THREADS + initial_count)) &&
		      (obj_counter == TOTAL_OBJECTS)), "test failed");
}

static void phil_entry(void)
{
	int counter;
	struct k_sem *f1;       /* fork #1 */
	struct k_sem *f2;       /* fork #2 */
	static int myId;        /* next philosopher ID */
	unsigned int pri = irq_lock();   /* interrupt lock level */
	int id = myId++;        /* current philosopher ID */

	irq_unlock(pri);

	/* always take the lowest fork first */
	if ((id + 1) != N_PHILOSOPHERS) {
		f1 = FORK(id);
		f2 = FORK(id + 1);
	} else {
		f1 = FORK(0);
		f2 = FORK(id);
	}

	for (counter = 0; counter < 5; counter++) {
		TAKE(f1);
		TAKE(f2);

		RANDDELAY(id);

		GIVE(f2);
		GIVE(f1);

		RANDDELAY(id);
	}
	GIVE(&f3);
}

/**
 * @brief Trace the number of objects created
 *
 * @ingroup kernel_objtracing_tests
 *
 * @details The test uses dining philsophers problem as
 * an application that implements multiple threads that
 * are synchronized with semaphores.
 */
void test_philosophers_tracing(void)
{
	int i;

	for (i = 0; i < N_PHILOSOPHERS; i++) {
		k_sem_init(&forks[i], 1, 1);
	}

	/* create philosopher threads */
	for (i = 0; i < N_PHILOSOPHERS; i++) {
		k_thread_create(&phil_data[i], &phil_stack[i][0], STSIZE,
				(k_thread_entry_t)phil_entry, NULL, NULL, NULL,
				K_PRIO_COOP(6), 0, K_NO_WAIT);
	}

	/* create object counter monitor thread */
	k_thread_create(&mon_data, mon_stack, STSIZE,
			(k_thread_entry_t)object_monitor, NULL, NULL, NULL,
			K_PRIO_COOP(7), 0, K_NO_WAIT);
}

void test_main(void)
{

	initial_count = thread_monitor();

	ztest_test_suite(obj_tracing,
			 ztest_unit_test(test_philosophers_tracing),
			 ztest_unit_test(test_obj_tracing));
	ztest_run_test_suite(obj_tracing);
}
