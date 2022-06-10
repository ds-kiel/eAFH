/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <drivers/hwinfo.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/bas.h>
#include <bluetooth/services/hrs.h>


	// Log storage writes statistics onto the SD card
	#define LOG_STORAGE 0
	// 40 x 1,25 ms = 50 ms
	#define BLE_PERIOD 0x28
	// connection timeout = SUPERVISION_TIMEOUT x 10 ms
	#define SUPERVISION_TIMEOUT 1000

#if CONFIG_BOARD_NRF52840DK_NRF52840
	// 16 x 1,25 ms = 20 ms
	#define BLE_PERIOD 0x10
#elif CONFIG_BOARD_ADAFRUIT_FEATHER_NRF52840
	// Log storage writes statistics onto the SD card
	#define LOG_STORAGE 1
	// 64 x 1,25 ms = 80 ms
	#define BLE_PERIOD 0x40
#endif /* CONFIG_BOARD */

#if LOG_STORAGE
	#include <device.h>
	#include <disk/disk_access.h>
	#include <fs/fs.h>
	#include <ff.h>
	#include <stdio.h>
	#define LOG_LEN 24
	struct afh_log { 
		uint16_t event_counter;
		uint8_t channel;
		int16_t pdr;
		uint8_t chm[5];
};
	static struct afh_log afh_logs[LOG_LEN];
	static uint8_t log_pos = 0;
	static uint8_t log_new[LOG_LEN] = {0};
#endif /* LOG_STORAGE */

// Create an array containing all devices used, as welkl as their role
struct device_role {
	uint8_t addr_id[8];
	int8_t role;
};
#define CENTRAL 1
#define PERIPHERAL 2
#define NUMBER_DEVICES 2
// This application prints the hardware address at bootup. Run it once to discover your own device address.
static struct device_role devices[NUMBER_DEVICES] = {{{0xd0,0xd4,0xd3,0x73,0x7a,0xe4,0xeb,0x11}, PERIPHERAL}, // The hardware address of the device used as peripheral
												 {{0xf2,0x72,0xd4,0xb5,0xe1,0xe6,0xda,0x9b}, CENTRAL}, // The hardware address of the device used as central
												};


// Variables common to peripheral and central devices
struct bt_conn *default_conn;
static uint8_t hw_addr_id[8] = {0}; // to store the hardware ID and decide on role

// Central
static struct bt_uuid_16 uuid = BT_UUID_INIT_16(0);
static struct bt_gatt_discover_params discover_params;
static struct bt_gatt_subscribe_params subscribe_params;

// Peripheral
static const struct bt_data advertisement[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_HRS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_DIS_VAL))
};


// declare
static void ctrl_start_scan(void);


/* Central functions */
static uint8_t ctrl_notify_func(struct bt_conn *conn,
			   struct bt_gatt_subscribe_params *params,
			   const void *data, uint16_t length)
{
	if (!data) {
		printk("[UNSUBSCRIBED]\n");
		params->value_handle = 0U;
		return BT_GATT_ITER_STOP;
	}
	//printk("[NOTIFICATION] data %p length %u\n", data, length);
	return BT_GATT_ITER_CONTINUE;
}

static uint8_t ctrl_discover_func(struct bt_conn *conn,
			     const struct bt_gatt_attr *attr,
			     struct bt_gatt_discover_params *params)
{
	int err;
	if (!attr) {
		printk("Discover complete\n");
		(void)memset(params, 0, sizeof(*params));
		return BT_GATT_ITER_STOP;
	}
	printk("[ATTRIBUTE] handle %u\n", attr->handle);
	if (!bt_uuid_cmp(discover_params.uuid, BT_UUID_HRS)) {
		memcpy(&uuid, BT_UUID_HRS_MEASUREMENT, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.start_handle = attr->handle + 1;
		discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else if (!bt_uuid_cmp(discover_params.uuid,
				BT_UUID_HRS_MEASUREMENT)) {
		memcpy(&uuid, BT_UUID_GATT_CCC, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.start_handle = attr->handle + 2;
		discover_params.type = BT_GATT_DISCOVER_DESCRIPTOR;
		subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);

		err = bt_gatt_discover(conn, &discover_params);
		if (err) {
			printk("Discover failed (err %d)\n", err);
		}
	} else {
		subscribe_params.notify = ctrl_notify_func;
		subscribe_params.value = BT_GATT_CCC_NOTIFY;
		subscribe_params.ccc_handle = attr->handle;

		err = bt_gatt_subscribe(conn, &subscribe_params);
		if (err && err != -EALREADY) {
			printk("Subscribe failed (err %d)\n", err);
		} else {
			printk("[SUBSCRIBED]\n");
		}
		return BT_GATT_ITER_STOP;
	}
	return BT_GATT_ITER_STOP;
}

static bool ctrl_eir_found(struct bt_data *data, void *user_data)
{
	bt_addr_le_t *addr = user_data;
	int i;
	// printk("[AD]: %u data_len %u\n", data->type, data->data_len);
	switch (data->type) {
	case BT_DATA_UUID16_SOME:
	case BT_DATA_UUID16_ALL:
		if (data->data_len % sizeof(uint16_t) != 0U) {
			printk("AD malformed\n");
			return true;
		}
		for (i = 0; i < data->data_len; i += sizeof(uint16_t)) {
			struct bt_le_conn_param *param;
			struct bt_uuid *uuid;
			uint16_t u16;
			int err;
			memcpy(&u16, &data->data[i], sizeof(u16));
			uuid = BT_UUID_DECLARE_16(sys_le16_to_cpu(u16));
			if (bt_uuid_cmp(uuid, BT_UUID_HRS)) {
				continue;
			}
			err = bt_le_scan_stop();
			if (err) {
				printk("Stop LE scan failed (err %d)\n", err);
				continue;
			}
			//Default case: BT_LE_CONN_PARAM_DEFAULT;BT_LE_CONN_PARAM(BT_GAP_INIT_CONN_INT_MIN, BT_GAP_INIT_CONN_INT_MAX, 0, 2000);
			param =  BT_LE_CONN_PARAM(BLE_PERIOD, BLE_PERIOD, 0, SUPERVISION_TIMEOUT);
			err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN,
						param, &default_conn);
			if (err) {
				printk("Create conn failed (err %d)\n", err);
				ctrl_start_scan();
			}
			return false;
		}
	}
	return true;
}

static void ctrl_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	char dev[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, dev, sizeof(dev));
	// printk("[DEVICE]: %s, AD evt type %u, AD data len %u, RSSI %i\n",
	//        dev, type, ad->len, rssi);

	/* We're only interested in connectable events */
	if (type == BT_GAP_ADV_TYPE_ADV_IND ||
	    type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		bt_data_parse(ad, ctrl_eir_found, (void *)addr);
	}
}

static void ctrl_start_scan(void)
{
	int err;

	/* Use active scanning and disable duplicate filtering to handle any
	 * devices that might update their advertising data at runtime. */
	struct bt_le_scan_param scan_param = {
		.type       = BT_LE_SCAN_TYPE_ACTIVE,
		.options    = BT_LE_SCAN_OPT_NONE,
		.interval   = BT_GAP_SCAN_FAST_INTERVAL,
		.window     = BT_GAP_SCAN_FAST_WINDOW,
	};

	err = bt_le_scan_start(&scan_param, ctrl_device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
}

static void ctrl_connected(struct bt_conn *conn, uint8_t conn_err)
{
	char addr[BT_ADDR_LE_STR_LEN];
	int err;
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	if (conn_err) {
		printk("Failed to connect to %s (%u)\n", addr, conn_err);
		bt_conn_unref(default_conn);
		default_conn = NULL;
		ctrl_start_scan();
		return;
	}
	printk("Connected: %s\n", addr);
	if (conn == default_conn) {
		memcpy(&uuid, BT_UUID_HRS, sizeof(uuid));
		discover_params.uuid = &uuid.uuid;
		discover_params.func = ctrl_discover_func;
		discover_params.start_handle = 0x0001;
		discover_params.end_handle = 0xffff;
		discover_params.type = BT_GATT_DISCOVER_PRIMARY;
		err = bt_gatt_discover(default_conn, &discover_params);
		if (err) {
			printk("Discover failed(err %d)\n", err);
			return;
		}
	}
}

static void ctrl_disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Disconnected: %s (reason 0x%02x)\n", addr, reason);
	if (default_conn != conn) {
		return;
	}
	bt_conn_unref(default_conn);
	default_conn = NULL;
	ctrl_start_scan();
}



/* peripheral functions */
static void peripheral_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		default_conn = bt_conn_ref(conn);
		printk("Connected\n");
	}
}

static void peripheral_disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);

	if (default_conn) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
	}
}

static void peripheral_bt_ready(void)
{
	int err;
	printk("Bluetooth initialized\n");
	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, advertisement, ARRAY_SIZE(advertisement), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}
	printk("Advertising successfully started\n");
}

static void peripheral_auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Pairing cancelled: %s\n", addr);
}

static void peripheral_bas_notify(void)
{
	uint8_t battery_level = bt_bas_get_battery_level();
	battery_level--;
	if (!battery_level) {
		battery_level = 100U;
	}
	bt_bas_set_battery_level(battery_level);
}

static void peripheral_hrs_notify(void)
{
	static uint8_t heartrate = 90U;
	/* Heartrate measurements simulation */
	heartrate++;
	if (heartrate == 160U) {
		heartrate = 90U;
	}
	bt_hrs_notify(heartrate);
}

/* Callbacks */
static struct bt_conn_cb prph_conn_callbacks = {
	.connected = peripheral_connected,
	.disconnected = peripheral_disconnected,
};
static struct bt_conn_cb ctrl_conn_callbacks = {
	.connected = ctrl_connected,
	.disconnected = ctrl_disconnected,
};
static struct bt_conn_auth_cb prph_auth_cb_display = {
	.cancel = peripheral_auth_cancel,
};


int8_t get_device(uint8_t address[])
{
	uint8_t i = 0;
	for (i=0; i<NUMBER_DEVICES;++i) {
		if (memcmp(address, devices[i].addr_id, 8*sizeof(uint8_t))==0) {
			return i;
		}
	}
	return -1;
}


#if LOG_STORAGE
	static FATFS fat_fs; // file system
	/* mounting info */
	static struct fs_mount_t mp = {
		.type = FS_FATFS,
		.fs_data = &fat_fs,
	};
	static const char *disk_mount_pt = "/SD:";
	struct fs_file_t file;

	void init_storage()
	{
			/* raw disk i/o */
	do {
		static const char *disk_pdrv = "SD";
		uint32_t block_count;
		uint32_t block_size;

		if (disk_access_init(disk_pdrv) != 0) {
			break;
		}

		if (disk_access_ioctl(disk_pdrv,
				DISK_IOCTL_GET_SECTOR_COUNT, &block_count)) {
			break;
		}

		if (disk_access_ioctl(disk_pdrv,
				DISK_IOCTL_GET_SECTOR_SIZE, &block_size)) {
			break;
		}

	} while (0);
	mp.mnt_point = disk_mount_pt;
	int res = fs_mount(&mp);
	if (res == FR_OK) {
		printk("Disk mounted.\n");
	} else {
		printk("Error mounting disk.\n");
	}
}

	int open_file(char *path)
	{
		//fs_file_t_init(file);
		int rc;
		rc = fs_open(&file, path, FS_O_CREATE | FS_O_RDWR);
		printk("Opening file... Status: %i\n",rc);
		return rc;
	}

	void log_write_line(uint16_t event_counter, uint8_t channel, uint8_t pdr, uint8_t chm[]) {
		afh_logs[log_pos].event_counter = event_counter;
		afh_logs[log_pos].channel = channel;
		afh_logs[log_pos].pdr = pdr;
		for (uint8_t i=0;i<5;i++) {
			afh_logs[log_pos].chm[i] = chm[i];
		}
		log_new[log_pos] = 1;
		log_pos +=1;
		if (log_pos>=LOG_LEN){
			log_pos=0;
		}
	}

	void log_write_to_file() {
		// go to end of file
		fs_seek(&file, 0, FS_SEEK_END);
		// write log
		char log[64];
		for (int i=0;i<LOG_LEN;i++) {
			if (log_new[i]) {
				sprintf(log,
						"e: %u c: %u m: 0x%02x%02x%02x%02x%02x p: %u \n",
						afh_logs[log_pos].event_counter,
						afh_logs[log_pos].channel,
						afh_logs[log_pos].chm[4],afh_logs[log_pos].chm[3],afh_logs[log_pos].chm[2],afh_logs[log_pos].chm[1],afh_logs[log_pos].chm[0],
						afh_logs[log_pos].pdr);
				fs_write(&file, log, strlen(log) * sizeof(char));
				fs_sync(&file);
				printk(log);
				log_new[i]=0;
			}
		}
		// // close
        // //fs_close(&file);
		return;
	}


#endif /* LOG_STORAGE */



void main(void)
{
	int err;
	k_sleep(K_SECONDS(5));
	// read device ID
	hwinfo_get_device_id(hw_addr_id,8);
	printk("This device address: ");
	printk("0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x\n", hw_addr_id[0],
																	hw_addr_id[1],
																	hw_addr_id[2],
																	hw_addr_id[3],
																	hw_addr_id[4],
																	hw_addr_id[5],
																	hw_addr_id[6],
																	hw_addr_id[7]
																	);
	uint8_t device_id = get_device(hw_addr_id);
	if (device_id<0) {
		printk("Device unknown\n");
		return;
	}
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}
	if (devices[device_id].role==CENTRAL) 
	{
		printk("Starting as CENTRAL\n");
		#if LOG_STORAGE
			init_storage();
			open_file("/SD:/log.txt");
		#endif /* LOG_STORAGE */
		bt_conn_cb_register(&ctrl_conn_callbacks);
		ctrl_start_scan();
		while(1) {
			k_sleep(K_MSEC(1));
			#if LOG_STORAGE
				log_write_to_file();
			#endif /* LOG_STORAGE */
		}
	}
	else if (devices[device_id].role==PERIPHERAL) 
	{
		printk("Starting as PERIPHERAL\n");
		peripheral_bt_ready();
		bt_conn_cb_register(&prph_conn_callbacks);
		bt_conn_auth_cb_register(&prph_auth_cb_display);
		while (1) {
			k_sleep(K_SECONDS(1));
			/* Heartrate measurements simulation */
			peripheral_hrs_notify();
			/* Battery level simulation */
			peripheral_bas_notify();
		}
	}
	else
	{
		printk("This device address has no defined role, going into passive mode...\n");
			while (1) {
			k_sleep(K_SECONDS(1));
		}
	}
}
