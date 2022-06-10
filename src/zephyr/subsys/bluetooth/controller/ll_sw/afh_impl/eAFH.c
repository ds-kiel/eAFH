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
#include "afh_impl/eAFH.h"

#if LOG_STORAGE
void log_write_line(uint16_t event_counter, uint8_t channel, uint8_t pdr, uint8_t chm[]); // defined in main.c
#endif


LOG_MODULE_DECLARE(bt_afh, CONFIG_BT_ULL_AFH_LOG_LEVEL);

// static uint32_t t1,t2;

#define PRINT_BITMAP(x) printk("%u%u%u%u%u%u%u%u", (x&0x01), (x&0x02)>>1, (x&0x04)>>2, (x&0x08)>>3, (x&0x10)>>4, (x&0x20)>>5, (x&0x40)>>6, (x&0x80)>>7);


static uint8_t bit_count(uint8_t u) { return (u - (u >> 1) - (u >> 2) - (u >> 3) - (u >> 4) - (u >> 5) - (u >> 6) - (u >> 7)); }

// AFHIoT Algorithms
#if CONFIG_BT_AFH_EAFH

		/* store PDR of connection event we are currently parsing */
		struct sysname_event {
			uint16_t event_counter;
			uint8_t channel;
			uint8_t pdr;
		};
		static struct sysname_event ev;
		
		static uint16_t last_update_cnt;
		
		/* long-term PDR estimation using a sliding window */
		static uint8_t sw_channel_pdr[37][WINDOW_SIZE] = {{1}};
		static uint8_t longterm_sw_channel_pdr[37][LONGTERM_WINDOW_SIZE] = {{1}};
		/* indexes of next values in the sliding window */
		static uint8_t sw_channel_idx[37] = {0};
		static uint8_t longterm_sw_channel_idx[37] = {0};
		/* sum of successes within the sliding-window */
		static uint8_t sw_channel_pdr_sum[37] = {0};
		static uint8_t longterm_sw_channel_pdr_sum[37] = {0};
		/* incremental counter since last channel use */
		static uint16_t channel_last_use_counter[37] = {1};
		/* score given to measure future exploration/inclusion of channels */
		static float channel_exploration_score[37] = {0.0f};
		/* score given to measure unstability of nearby channels */
		static float channel_leaky_losses[37] = {0.0f};
		/* overall score given to channels */
		static float explo_score[37] = {0.0f};
		static float chan_score[37] = {0.0f};

		void afh_algo_init(uint16_t handle) {
			uint8_t i,j;
			for (i=0;i<37;++i) {
				/* initialize sliding window */
				for (j=0;j<WINDOW_SIZE;++j) {
					sw_channel_pdr[i][j] = 1;
				}
				for (j=0;j<LONGTERM_WINDOW_SIZE;++j) {
					longterm_sw_channel_pdr[i][j] = 1;
				}
			}
			memset(channel_last_use_counter, 0, 37*sizeof(uint8_t));
			printk("Starting AFH Kiel\n");
		}

		/* update channel PDR after connection event */ 
		void afh_sysname_update_pdr(uint8_t used_channel, uint8_t status) {
			sw_channel_pdr[used_channel][sw_channel_idx[used_channel]] = status;
			sw_channel_idx[used_channel] = (sw_channel_idx[used_channel]+1)%WINDOW_SIZE;
			longterm_sw_channel_pdr[used_channel][longterm_sw_channel_idx[used_channel]] = status;
			longterm_sw_channel_idx[used_channel] = (longterm_sw_channel_idx[used_channel]+1)%LONGTERM_WINDOW_SIZE;
			uint8_t ws_idx, ch_idx;
			// recompute long-term pdr within sliding window
			memset(&sw_channel_pdr_sum, 0, 37*sizeof(uint8_t));
			memset(&longterm_sw_channel_pdr_sum, 0, 37*sizeof(uint8_t));
			for (ch_idx=0;ch_idx<37;++ch_idx) {
				for (ws_idx=0;ws_idx<LONGTERM_WINDOW_SIZE;++ws_idx) {
					if (ws_idx<WINDOW_SIZE) {
						sw_channel_pdr_sum[ch_idx] += sw_channel_pdr[ch_idx][ws_idx];
					}
					longterm_sw_channel_pdr_sum[ch_idx] += longterm_sw_channel_pdr[ch_idx][ws_idx];
				}
			}
		}

		/* update exploration score after connection event */ 
		void afh_sysname_update_exploration(uint8_t used_channel, uint8_t status) {
			uint8_t i;
			channel_last_use_counter[used_channel] = 0;
			for (i=0;i<37;++i) {
				channel_last_use_counter[i] +=1;
			}
		}

		/* compute exploration score once all events have been treated */ 
		void afh_sysname_compute_exploration_score() {
			uint8_t i;
			float timeout, cnt = 0.0;
			// compute exploration score
			for (i=0;i<37;++i) {
				// timeout might be higher than float precision if we account the number of events
				timeout = (float) (1<<(LONGTERM_WINDOW_SIZE+1-longterm_sw_channel_pdr_sum[i]));
				//timeout = (float) EXCLUSION_PERIOD;
				cnt = (float) channel_last_use_counter[i];
				// divide by exponential back-off
				cnt = cnt/timeout;
				// add connection-event scaling
				channel_exploration_score[i] = cnt/EXCLUSION_PERIOD;
			}
		}

		/* update nearby-channel unstability after treaing ALL logged connection events */ 
		void afh_sysname_update_leaky_losses() {
			// long-term PDR sum has been recomputed earlier in afh_sysname_update_exploration
			uint8_t i;
			for (i=1;i<36;++i) {
				channel_leaky_losses[i] = ((float)(longterm_sw_channel_pdr_sum[i-1]+longterm_sw_channel_pdr_sum[i+1])) / ((float) 2*LONGTERM_WINDOW_SIZE);
				channel_leaky_losses[i] = - (1.0 - channel_leaky_losses[i]);
			}
			channel_leaky_losses[0] = ((float)longterm_sw_channel_pdr_sum[1]) / ((float)LONGTERM_WINDOW_SIZE);
			channel_leaky_losses[0] = - (1.0 - channel_leaky_losses[0]);
			channel_leaky_losses[36] = ((float)longterm_sw_channel_pdr_sum[35]) / ((float)LONGTERM_WINDOW_SIZE);
			channel_leaky_losses[36] = - (1.0 - channel_leaky_losses[36]);
		}


		void afh_sysname_exclude_channels(uint8_t chm[]) {
			/* We exclude channels that had at least two losses over the last 10 events */
			uint8_t i;
			/* reset desired chan map */
			uint8_t to_keep[5] = {0};
			/* include all channels with required score */
			float pdr;
			for (i=0;i<37;++i) {
				pdr = ((float)sw_channel_pdr_sum[i])/((float)WINDOW_SIZE);
				if (pdr >= EXCLUSION_THRESHOLD) {
					to_keep[i>>3] |= 1 << (0x07 & i);
				} 
			}
			// exclude channels from channel map
			for (i=0;i<5;++i) {
				chm[i] &= to_keep[i];
			}
			return;
		}

		void afh_sysname_explore_channels(uint8_t chm[]) {
			uint8_t i;
			/* Compute new inclusion score */
			for (i=0;i<37;++i) {
				explo_score[i] = channel_exploration_score[i] + 2*channel_leaky_losses[i];
			}
			/* reset desired chan map */
			uint8_t to_explore[5] = {0};
			/* include all channels with required score */
			for (i=0;i<37;++i) {
				if (explo_score[i] >= INCLUSION_THRESHOLD) {
				// 	if ( (chm[i>>3]&(1<<(0x07&i)))==0 && last_update_cnt > 6) {
				// 		LOG_DBG("leaky: %i, %i, %i\n", i,
				// 										EXCLUSION_PERIOD * (1<<(LONGTERM_WINDOW_SIZE+1-longterm_sw_channel_pdr_sum[i])),
				// 										channel_last_use_counter[i]);
				// }
					to_explore[i>>3] |= 1 << (0x07 & i);
				}
			}
			for (i=0;i<5;++i) {
				chm[i] |= to_explore[i];
			}
			return;
		}

		void afh_sysname_ensure_min_channels(uint8_t chm[]) {
			/* ensure that at least MIN_CHANNELS are selected */
			uint8_t cnt,i,j=0;
			uint8_t to_keep[5] = {0};
			for (i=0;i<5;++i) {
				cnt += bit_count(chm[i]);
			}
			if (cnt >= MIN_NUM_CHANNELS) {
				return;
			}
			/* find channels with highest score */
			for (i=0;i<37;++i) {
				// score = pdr (+ explo?)
				chan_score[i] = ((float)(longterm_sw_channel_pdr_sum[i]))/((float)LONGTERM_WINDOW_SIZE);
				// if channel is already selected
				if (chm[i>>3]&(1<<(0x07&i))) {
					chan_score[i] = 0;
				}
			}
			float best_score = -1.0f;
			uint8_t best_chan = 0;
			for (j=0;j<MIN_NUM_CHANNELS-cnt;j++) {
				best_score = -1.0f;
				best_chan = 0;
				for (i=0;i<37;i++) {
					if (chan_score[i] > best_score) {
						best_score = chan_score[i];
						best_chan = i;
					}
				}
			to_keep[best_chan>>3] |= 1 << (0x07 & best_chan);
			chan_score[best_chan] = 0.0f;
			}
			for (i=0;i<5;++i) {
				chm[i] |= to_keep[i];
			}
		}

		/* main function - parse logged AFH measurements, compute new channel map and request change to stack */
		void conn_chm_remap(uint16_t handle, struct ull_afh_measurement_queue* measures) {
			// t1 = k_cycle_get_32();
			/* Read from FIFO queue for last events */
			struct ull_afh_measurement measure;
			uint8_t parsed_one_event = 0;
			/* Important! An AFH measure represents one measure (i.e., rssi of one RX, or success/failure of one RX/TX */
			/* We can have more than one AFH measure per connection event! */
			struct ll_conn *conn = ll_conn_get(handle);
			uint8_t* curr_chm = (uint8_t *)&conn->lll.data_chan_map[0];
			uint8_t new_chm[5] = {0};
			memcpy(&new_chm, curr_chm, 5*sizeof(uint8_t));
			int err = ull_afh_measurement_queue_get(measures, &measure);
			while (!err) {
				/* All measures for previous connection event have been parsed */
				if (measure.event_counter != ev.event_counter) {
					afh_sysname_update_pdr(ev.channel, ev.pdr);
					afh_sysname_update_exploration(ev.channel, ev.pdr);
					parsed_one_event = 1;
					#if LOG_STORAGE
						log_write_line(ev.event_counter, ev.channel, ev.pdr, curr_chm);
					#else
					LOG_DBG("e: %u c: %u m: 0x%02x%02x%02x%02x%02x p: %u \n", ev.event_counter,
																				  ev.channel,
																				  curr_chm[4],curr_chm[3],curr_chm[2],curr_chm[1],curr_chm[0],
																				  ev.pdr);
					#endif /* LOG_STORAGE */
					/* Prepare for next connection event */
					memset(&ev, 0, sizeof(struct sysname_event));
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
			/* compute new explorations score */
			afh_sysname_compute_exploration_score();
			/* apply leaky losses update */
			afh_sysname_update_leaky_losses();
			/* exclude bad channels */
			afh_sysname_exclude_channels(new_chm);
			/* explore channels */
			afh_sysname_explore_channels(new_chm);
			/* ensure enough channels are available */
			afh_sysname_ensure_min_channels(new_chm);

			if (last_update_cnt > 6) {
				if (!ull_afh_chm_equals(curr_chm, (uint8_t *)&new_chm)) {
					ull_afh_conn_chm_update((uint8_t *)&new_chm, handle);
					last_update_cnt = 0;
					/* reset stats of blacklisted channels */
					uint8_t i,j;
					for (i=0;i<37;++i) {
						if ((new_chm[i>>3]&(1<<(0x07 & i))) == 0) {
							for (j=0;j<WINDOW_SIZE;++j) {
								sw_channel_pdr[i][j] = 1;
							}
						}
					}
					
				}
			}
			last_update_cnt++;
			// t2 = k_cycle_get_32();
			// LOG_DBG("exec: %u\n",t2-t1);
		}


#endif /* CONFIG_BT_AFH_EAFH */