/*
 * Copyright (c) 2020 Kiel University
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#if CONFIG_BOARD_NRF52840DK_NRF52840
	#define LOG_STORAGE 0
#elif CONFIG_BOARD_ADAFRUIT_FEATHER_NRF52840
	#define LOG_STORAGE 1
#endif /* CONFIG_BOARD */

#define CONFIG_BT_ULL_AFH_LOG_LEVEL LOG_LEVEL_DBG
#define AFH_WORKQUEUE_STACK_SIZE 256
#define AFH_WORKQUEUE_PRIORITY 10

#define BT_AFH_MEASUREMENT_QUEUE_SIZE 32

enum ull_afh_measurement_type {
	BT_AFH_MEASUREMENT_RSSI,
	BT_AFH_MEASUREMENT_RX,
	BT_AFH_MEASUREMENT_TX,
	};

struct ull_afh_measurement { 
	enum ull_afh_measurement_type type;
	uint16_t event_counter;
	uint8_t channel;
	int16_t data;
	uint32_t timestamp;
};

struct ull_afh_measurement_queue {
	struct ull_afh_measurement queue[BT_AFH_MEASUREMENT_QUEUE_SIZE];
	uint8_t next_write;
	uint8_t next_read;
	bool has_new_data;
	bool isr_has_modified;
};

struct ull_afh_process {
	uint16_t handle;
	struct k_work remap_fn;
	struct ull_afh_measurement_queue measurements_queue;
};

uint8_t ull_afh_init(void);
uint8_t ull_afh_conn_init(uint16_t handle);
void ull_afh_global_chm_remap(struct k_work *item);
bool ull_afh_chm_equals(uint8_t *chm_1, uint8_t *chm_2);
int8_t ull_afh_conn_chm_update(uint8_t const *const chm, uint16_t handle);
int8_t ull_afh_measurement_queue_put(struct ull_afh_measurement_queue* q, struct ull_afh_measurement* data);
int8_t ull_afh_measurement_queue_get(struct ull_afh_measurement_queue* q, struct ull_afh_measurement* data);
int8_t lll_afh_conn_isr(uint16_t handle, struct ull_afh_measurement* data);
void ull_afh_conn_chm_remap(struct k_work* item);
/* New AFH techniques must implement these two functions*/
void afh_algo_init(uint16_t handle);
void conn_chm_remap(uint16_t handle, struct ull_afh_measurement_queue* measurements);