/*
 * Copyright (c) 2020 Kiel University
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <bluetooth/hci.h>
#include <sys/byteorder.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <logging/log.h>

#include "util/util.h"
#include "util/memq.h"
#include "util/mayfly.h"

#include "hal/ticker.h"
#include "hal/ccm.h"
#include "ticker/ticker.h"

#include "pdu.h"
#include "ll.h"
#include "ll_feat.h"

#include "lll.h"
#include "lll_vendor.h"
#include "lll_clock.h"
#include "lll_adv.h"
#include "lll_scan.h"
#include "lll_conn.h"
#include "lll_master.h"
#include "lll_filter.h"
#include "lll_tim_internal.h"

#include "ull_adv_types.h"
#include "ull_scan_types.h"
#include "ull_conn_types.h"
#include "ull_filter.h"

#include "ull_internal.h"
#include "ull_scan_internal.h"
#include "ull_conn_internal.h"
#include "ull_master_internal.h"

#include "ull_afh.h"
#if defined(CONFIG_BT_AFH_NONE)
	//nothing to include
#elif defined(CONFIG_BT_AFH_EAFH)
	#include "afh_impl/eAFH.h"
#elif defined(CONFIG_BT_AFH_PDR_EXCLUSION)
	#include "afh_impl/PDR_exclusion.h"
#endif


LOG_MODULE_REGISTER(bt_afh, CONFIG_BT_ULL_AFH_LOG_LEVEL);

K_THREAD_STACK_DEFINE(afh_workqueue_stack, AFH_WORKQUEUE_STACK_SIZE);
static struct k_work_q ull_afh_workqueue_q;

// AFH process for each connection
static struct ull_afh_process afh_conn_process[CONFIG_BT_MAX_CONN];
static uint16_t call_cnt;

bool chm_equals(uint8_t *chm_1, uint8_t *chm_2);


// Init Global AFH process
uint8_t ull_afh_init(void)
{
	int i = 0;
	for (i=0; i<CONFIG_BT_MAX_CONN; ++i) {
		afh_conn_process[i].handle = 0xFFFF;
	}
	k_work_q_start(&ull_afh_workqueue_q, afh_workqueue_stack,
			   K_THREAD_STACK_SIZEOF(afh_workqueue_stack), AFH_WORKQUEUE_PRIORITY);
	return 0;
}

// Init AFH process per connection
uint8_t ull_afh_conn_init(uint16_t handle)
{
	int i = 0;
	while (afh_conn_process[i].handle != 0xFFFF) {
		++i;
	}
	
	// Max num of connections reached
	if (i >= CONFIG_BT_MAX_CONN)
		return 1;
	// init new afh process
	afh_conn_process[i].handle = handle;
	// Init events queue that will contain low-level events
	afh_conn_process[i].measurements_queue.next_write = 0;
	afh_conn_process[i].measurements_queue.next_read = 0;
	afh_conn_process[i].measurements_queue.has_new_data = false;
	afh_conn_process[i].measurements_queue.isr_has_modified = false;
	// init process
	k_work_init(&afh_conn_process[i].remap_fn, ull_afh_conn_chm_remap);
	// call technique-implementation of init function
	afh_algo_init(handle);
	return 0;
}

int8_t ull_afh_conn_chm_update(uint8_t const *const chm, uint16_t handle)
{
	uint8_t ret;

	// todo keep?
	// ull_chan_map_set(chm);

	struct ll_conn *conn;

	conn = ll_connected_get(handle);
	if (!conn || conn->lll.role) {
		return 1;
	}

	ret = ull_conn_llcp_req(conn);
	if (ret) {
		return ret;
	}

	memcpy(conn->llcp.chan_map.chm, chm,
		   sizeof(conn->llcp.chan_map.chm));
	/* conn->llcp.chan_map.instant     = 0; */
	conn->llcp.chan_map.initiate = 1U;

	conn->llcp_type = LLCP_CHAN_MAP;
	conn->llcp_req++;
	return 0;
}

// Apply AFH Selection algorithm on one connection - run as workqueue thread
void ull_afh_conn_chm_remap(struct k_work* item)
{
	// retrieve afh_process struct around this work
	struct ull_afh_process* afh = CONTAINER_OF(item, struct ull_afh_process, remap_fn);
	// call technique-specific implementation
	conn_chm_remap(afh->handle, &afh->measurements_queue);
}

// Apply AFH selection algorithm to the global channel map - run as workqueue thread
void ull_afh_global_chm_remap(struct k_work *item)
{
	// retrieve afh_process struct around this work
	// struct ull_afh_process* afh = CONTAINER_OF(item, struct ull_afh_process, remap_fn);
	// choose implementation
	return;
}


bool ull_afh_chm_equals(uint8_t *chm_1, uint8_t *chm_2)
{
	for (int i = 0; i < 5; i++) {
		if (chm_1[i] != chm_2[i]) {
			return false;
		}
	}
	return true;
}

int8_t ull_afh_measurement_queue_put(struct ull_afh_measurement_queue* q, struct ull_afh_measurement* data)
{
	// ensure concurrency is detected
	q->isr_has_modified = true;
	q->has_new_data = true;
	memcpy(&q->queue[q->next_write], data, sizeof(struct ull_afh_measurement));
	//printk("Putting event [%u]:{%i,%i} to queue\n",q->next_write, data->type, data->data);
	q->next_write = (q->next_write+1)%BT_AFH_MEASUREMENT_QUEUE_SIZE;
	return 0;
}

int8_t ull_afh_measurement_queue_get(struct ull_afh_measurement_queue* q, struct ull_afh_measurement* ev) {
	if (!q->has_new_data) {
		return -ENODATA;
	}
	// avoid concurrency
	do {
		q->isr_has_modified = false;
		memcpy(ev, &q->queue[q->next_read], sizeof(struct ull_afh_measurement));
	} while (q->isr_has_modified);
	// We managed to read one value, move next_read by one
	// Todo what happens if concurrency here?
	q->next_read = (q->next_read+1)%BT_AFH_MEASUREMENT_QUEUE_SIZE;
		if (q->next_read==q->next_write) {
			q->has_new_data = false;
	}
	return 0;
}

int8_t ull_afh_measurement_queue_get_all(struct ull_afh_measurement_queue* q, struct ull_afh_measurement buffer[]) {
	if (!q->has_new_data) {
		return -ENODATA;
	}
	// avoid concurrency
	do {
		q->isr_has_modified = false;
		int num_items;
		if (q->next_write <= q->next_read) {
			memcpy(&buffer[0], &q->queue[q->next_read], (BT_AFH_MEASUREMENT_QUEUE_SIZE-q->next_read)*sizeof(struct ull_afh_measurement));
			memcpy(&buffer[BT_AFH_MEASUREMENT_QUEUE_SIZE-q->next_read], &q->queue[0], (q->next_write)*sizeof(struct ull_afh_measurement));
		} else {
			num_items = q->next_write - q->next_read;
			memcpy(&buffer[0], &q->queue[q->next_read], num_items*sizeof(struct ull_afh_measurement));
		}
	} while (q->isr_has_modified);
	q->next_read = (q->next_read+1)%BT_AFH_MEASUREMENT_QUEUE_SIZE;
		if (q->next_read==q->next_write) {
			q->has_new_data = false;
	}
	return 0;
}

// Called by lower layer after each event, as ISR
int8_t lll_afh_conn_isr(uint16_t handle, struct ull_afh_measurement* data)
{
	// todo this is O(n) but we are in ISR
	uint8_t id = CONFIG_BT_MAX_CONN;
	for (uint16_t i=0;i<CONFIG_BT_MAX_CONN;++i) {
		if (afh_conn_process[i].handle == handle) {
			id = i;
			break;
		}
	}
	// Ensure that we run AFH for this connection
	if (id>=CONFIG_BT_MAX_CONN) {
		return 1;
	}
	ull_afh_measurement_queue_put(&afh_conn_process[id].measurements_queue, data);
	// Call AFH algorithm
	if (call_cnt%8==0) {
		call_cnt = 0;
		k_work_submit_to_queue(&ull_afh_workqueue_q, &afh_conn_process[id].remap_fn);
	}
	return 0;
}
