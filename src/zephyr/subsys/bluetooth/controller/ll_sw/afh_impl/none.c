/*
 * Copyright (c) 2020 Kiel University
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <math.h>
#include <bluetooth/hci.h>
#include <sys/byteorder.h>
#include <sys/ring_buffer.h>
#include <stdlib.h>
#include <random/rand32.h>
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

LOG_MODULE_DECLARE(bt_afh, CONFIG_BT_ULL_AFH_LOG_LEVEL);

#define PRINT_BITMAP(x) printk("%u%u%u%u%u%u%u%u", (x&0x01), (x&0x02)>>1, (x&0x04)>>2, (x&0x08)>>3, (x&0x10)>>4, (x&0x20)>>5, (x&0x40)>>6, (x&0x80)>>7);

void printk_timestamp(uint32_t timestamp) {
	// taken from logging module
	uint32_t total_seconds;
	uint32_t remainder;
	uint32_t seconds;
	uint32_t hours;
	uint32_t mins;
	uint32_t ms;
	uint32_t us;
	uint32_t freq = sys_clock_hw_cycles_per_sec();
	uint32_t timestamp_div = 1U;
	timestamp /= timestamp_div;
	total_seconds = timestamp / freq;
	seconds = total_seconds;
	hours = seconds / 3600U;
	seconds -= hours * 3600U;
	mins = seconds / 60U;
	seconds -= mins * 60U;
	remainder = timestamp % freq;
	ms = (remainder * 1000U) / freq;
	us = (1000 * (remainder * 1000U - (ms * freq))) / freq;
	printk("[%02d:%02d:%02d.%03d,%03d] ", hours, mins, seconds, ms, us);
}

// AFHIoT Algorithms
#if CONFIG_BT_AFH_NONE

		struct afh_debug_event {
			uint16_t event_counter;
			uint8_t channel;
			uint8_t pdr;
			uint8_t rssi;
			uint8_t noisefloor;
			uint8_t chm[5];
			uint32_t timestamp;
		};

		static struct afh_debug_event dbg_ev;

		void afh_algo_init(uint16_t handle) {}

		/* Main function called by ull_afh */
		void conn_chm_remap(uint16_t handle, struct ull_afh_measurement_queue* events) {
			/* Read from FIFO queue for last events */
			struct ull_afh_measurement ev;
			struct ll_conn *conn = ll_conn_get(handle);
			uint8_t* chm = (uint8_t *)&conn->lll.data_chan_map[0];
			uint8_t desired_chm[5] = {0xff, 0xff, 0xff, 0xff, 0x1f};
			int err = ull_afh_measurement_queue_get(events, &ev);
			while (!err) {
				// We are starting a new event, print everything we know about the previous one
				if (ev.event_counter != dbg_ev.event_counter) {
					// printk_timestamp(dbg_ev.timestamp);
					// printk("e: %u c: %u m: 0x%x%x%x%x%x d: %u p: %u r: %u n: %u\n", dbg_ev.event_counter,
					// 															  dbg_ev.channel,
					// 															  chm[4],chm[3],chm[2],chm[1],chm[0],
					// 															  dbg_ev.pdr,
					// 															  dbg_ev.pdr*1000,
					// 															  dbg_ev.rssi,
					// 															  0);
					LOG_DBG("e: %u c: %u m: 0x%02x%02x%02x%02x%02x p: %u \n", dbg_ev.event_counter,
																				  dbg_ev.channel,
																				  chm[4],chm[3],chm[2],chm[1],chm[0],
																				  dbg_ev.pdr);
					// set everything back to 0 by default
					memset(&dbg_ev, 0, sizeof(struct afh_debug_event));
				}
				// write up new infos
				dbg_ev.event_counter = ev.event_counter;
				dbg_ev.channel = ev.channel;
				dbg_ev.timestamp = ev.timestamp;
				memcpy(&dbg_ev.chm, chm, 5*sizeof(uint8_t));
				if (ev.type == BT_AFH_MEASUREMENT_TX) {
					dbg_ev.pdr = ev.data;
				} else if (ev.type == BT_AFH_MEASUREMENT_RSSI) {
					dbg_ev.rssi = ev.data;
				} 
				
				err = ull_afh_measurement_queue_get(events, &ev);
			}
			if (!ull_afh_chm_equals((uint8_t *)&chm, desired_chm)) {
				//ull_afh_conn_chm_update(desired_chm, handle);
			}
		}


#endif /* CONFIG_BT_AFH_NONE */