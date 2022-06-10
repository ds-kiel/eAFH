/*
 * Copyright (c) 2018-2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

void ull_master_setup(memq_link_t *link, struct node_rx_hdr *rx,
		      struct node_rx_ftr *ftr, struct lll_conn *lll);
void ull_master_ticker_cb(uint32_t ticks_at_expire, uint32_t remainder, uint16_t lazy,
			  void *param);
