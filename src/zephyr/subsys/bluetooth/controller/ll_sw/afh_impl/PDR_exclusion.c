#include <zephyr.h>
#include <bluetooth/hci.h>
#include <sys/byteorder.h>
#include <sys/ring_buffer.h>
#include <stdlib.h>
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
#include "afh_impl/PDR_exclusion.h"

LOG_MODULE_DECLARE(bt_afh, CONFIG_BT_ULL_AFH_LOG_LEVEL);

#define LOG_DBG_CHANMAP(x, pos) LOG_DBG("[%u] %u%u%u%u%u%u%u%u", pos, (x&0x01), (x&0x02)>>1, (x&0x04)>>2, (x&0x08)>>3, (x&0x10)>>4, (x&0x20)>>5, (x&0x40)>>6, (x&0x80)>>7);

#if CONFIG_BT_AFH_PDR_EXCLUSION

	struct graz_event {
		uint16_t event_counter;
		uint8_t channel;
		uint8_t pdr;
	};
	static struct graz_event ev;

	/* PDR sliding-window */
	static uint8_t sw_channel_pdr[37][WINDOW_SIZE] = {{1}};
	/* indexes of next values in the sliding window */
	static uint8_t sw_channel_idx[37] = {0};
	/* sum of successes within the sliding-window */
	static uint8_t sw_channel_pdr_sum[37] = {0};

	void afh_algo_init(uint16_t handle) {
		/* initialize sliding window */
		uint8_t i,j;
		for (i=0;i<37;++i) {
			for (j=0;j<WINDOW_SIZE;++j) {
				sw_channel_pdr[i][j] = 1;
			}
			sw_channel_pdr_sum[i] = WINDOW_SIZE;
		}
	}

	void afh_graz_update_pdr(uint8_t channel, uint8_t status) {
		sw_channel_pdr[channel][sw_channel_idx[channel]] = status;
		sw_channel_idx[channel] = (sw_channel_idx[channel]+1)%WINDOW_SIZE;
		// recompute sum
		sw_channel_pdr_sum[channel] = 0;
		uint8_t i;
		for (i=0;i<WINDOW_SIZE;++i) {
			sw_channel_pdr_sum[channel] += sw_channel_pdr[channel][i];
		}
		// printk("[%u] %i/%u\n", channel, sw_channel_pdr_sum[channel], WINDOW_SIZE);
	}

	void afh_graz_compute_chan_map(uint8_t chm[]) {
		uint8_t i,j,cnt = 0;
		float chan_score;
		for (i=0;i<37;++i) {
			chan_score = ((float)sw_channel_pdr_sum[i])/((float)WINDOW_SIZE);
			if (chan_score >= MIN_SCORE) {
				chm[i>>3] |= 1 << (0x07 & i);
				++cnt;
			}
		}
		/* ensure that at least MIN_CHANNELS are selected */
		if (cnt >= MIN_NUM_CHANNELS) {
			return;
		}
		/* Reset all channels */
		memset(chm, 0xff, 5*sizeof(uint8_t));
		chm[4] = 0x1f;
		/* reset all stats! */
		for (i=0;i<37;++i) {
			for (j=0;j<WINDOW_SIZE;++j) {
				sw_channel_pdr[i][j] = 1;
			}
			sw_channel_pdr_sum[i] = WINDOW_SIZE;
		}
		/* Todo: 
		In the paper, the connection interval is decrased by:
			new_interval = curr_interval / sup(37/MIN_NUM_CHANNELS)
		This new interval, used for probing channels, is used for:
			probing_len = S_chan * 37 * new_interval
		This faster probing behavior is not implemented here because we use the fastest interval
		*/
	}

/* main function - parse logged AFH measurements, compute new channel map and request change to stack */
	void conn_chm_remap(uint16_t handle, struct ull_afh_measurement_queue* measures) {
		/* Read from FIFO queue for last events */
		struct ull_afh_measurement measure;
		uint8_t parsed_one_event = 0;
		/* Important! An AFH measure represents one measure (i.e., rssi of one RX, or success/failure of one RX/TX */
		/* We can have more than one AFH measure per connection event! */
		struct ll_conn *conn = ll_conn_get(handle);
		uint8_t* curr_chm = (uint8_t *)&conn->lll.data_chan_map[0];
		uint8_t new_chm[5] = {0};
		int err = ull_afh_measurement_queue_get(measures, &measure);
		while (!err) {
			/* All measures for previous connection event have been parsed */
			if (measure.event_counter != ev.event_counter) {
				afh_graz_update_pdr(ev.channel, ev.pdr);
				parsed_one_event = 1;
					LOG_DBG("e: %u c: %u m: 0x%02x%02x%02x%02x%02x p: %u \n", ev.event_counter,
																				  ev.channel,
																				  curr_chm[4],curr_chm[3],curr_chm[2],curr_chm[1],curr_chm[0],
																				  ev.pdr);
				/* Prepare for next connection event */
				memset(&ev, 0, sizeof(struct graz_event));
			}
			/* save event data into ev */
			ev.event_counter = measure.event_counter;
			ev.channel = measure.channel;
			if (measure.type == BT_AFH_MEASUREMENT_TX) {
				ev.pdr = measure.data;
			}
			err = ull_afh_measurement_queue_get(measures, &measure);
		} /* end while new afh measurement */
		if (!parsed_one_event) {
			return;
		}
		/* select channels to include */
		afh_graz_compute_chan_map(new_chm);
		/* print desired chanmap */
		// LOG_DBG("%x%x%x%x%x", curr_chm[0],curr_chm[1],curr_chm[2],curr_chm[3],curr_chm[4]);

		if (!ull_afh_chm_equals((uint8_t *)&curr_chm, (uint8_t *)&new_chm)) {
			ull_afh_conn_chm_update((uint8_t *)&new_chm, handle);
		}
	}

#endif /* CONFIG_BT_AFH_PDR_EXCLUSION */