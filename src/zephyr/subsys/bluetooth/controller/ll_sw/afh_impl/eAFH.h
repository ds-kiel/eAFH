/*
 * Copyright (c) 2020 Kiel University
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if defined(CONFIG_BT_AFH_EAFH)
	#define EXCLUSION_PERIOD 200 // default:100
	#define WINDOW_SIZE 20 // default:10
	#define LONGTERM_WINDOW_SIZE 25 // default:20
	#define MIN_NUM_CHANNELS 10 // default:6
	#define INCLUSION_THRESHOLD 1.0f // default:1.0
	#define EXCLUSION_THRESHOLD 0.95f // default:0.90 (WINDOW_SIZE-1)/WINDOW_SIZE
#endif
