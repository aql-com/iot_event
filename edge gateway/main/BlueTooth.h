//////////////////////////////////////////////
//
// BlueTooth.h
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "MAC_Rpt.h"		// for BLUETOOTH_CMD_LIST_LENGTH, BLUETOOTH_CMD_LIST_ENTRY_LENGTH] defs...


#include "esp_bt.h"

#ifdef USE_BLUETOOTH_CLASSIC_DETECT
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#endif


extern unsigned char enable_bluetooth_whitelist_flag;		// Bluetooth whitelist enable flag
extern unsigned char enable_bluetooth_blacklist_flag;		// Bluetooth blacklist enable flag

extern unsigned int num_bt_wl_entries;
extern unsigned int num_bt_bl_entries;

extern char bluetooth_cmd_whitelist[BLUETOOTH_CMD_LIST_LENGTH][BLUETOOTH_CMD_LIST_ENTRY_LENGTH];
extern char bluetooth_cmd_blacklist[BLUETOOTH_CMD_LIST_LENGTH][BLUETOOTH_CMD_LIST_ENTRY_LENGTH];

extern char bluetooth_devname_whitelist[BLUETOOTH_DEVNAME_LIST_LENGTH][BLUETOOTH_DEVNAME_LIST_ENTRY_LENGTH];

extern const char bluetooth_wl_init[BLUETOOTH_CMD_LIST_INIT_LENGTH][BLUETOOTH_CMD_LIST_ENTRY_LENGTH];
extern const char bluetooth_bl_init[BLUETOOTH_CMD_LIST_INIT_LENGTH][BLUETOOTH_CMD_LIST_ENTRY_LENGTH];
extern const char bluetooth_dw_init[BLUETOOTH_DEVNAME_LIST_INIT_LENGTH][BLUETOOTH_DEVNAME_LIST_ENTRY_LENGTH];

extern const char bluetooth_wl_init_len;
extern const char bluetooth_bl_init_len;
extern const char bluetooth_dw_init_len;


typedef struct {
    char scan_local_name[32];
    uint8_t name_len;
} ble_scan_local_name_t;

typedef struct {
    uint8_t *q_data;
    uint16_t q_data_len;
} host_rcv_data_t;


void ble_set_adv_data(void);

/*
 * @brief: BT controller callback function, used to notify the upper layer that
 *         controller is ready to receive command
 */
//void controller_rcv_pkt_ready(void);

/*
 * @brief: BT controller callback function, to transfer data packet to upper
 *         controller is ready to receive command
 */
//int host_rcv_pkt(uint8_t *data, uint16_t len);

//esp_vhci_host_callback_t vhci_host_cb;

//void hci_cmd_send_reset(void);

//void hci_cmd_send_ble_adv_start(void);

//void hci_cmd_send_ble_set_adv_param(void);

//void hci_cmd_send_ble_set_adv_data(void);

/*
 * @brief: send HCI commands to perform BLE advertising;
 */
void bleAdvtTask(void);	// *pvParameters);

void bleScanSetup(void);
void hci_evt_process(void *pvParameters);
static esp_err_t get_local_name (uint8_t *data_msg, uint8_t data_len, ble_scan_local_name_t *scanned_packet);


#ifdef USE_BLUETOOTH_CLASSIC_DETECT

#define GAP_TAG          "GAP"

typedef enum {
    APP_GAP_STATE_IDLE = 0,
    APP_GAP_STATE_DEVICE_DISCOVERING,
    APP_GAP_STATE_DEVICE_DISCOVER_COMPLETE,
    APP_GAP_STATE_SERVICE_DISCOVERING,
    APP_GAP_STATE_SERVICE_DISCOVER_COMPLETE,
} app_gap_state_t;

typedef struct {
    bool dev_found;
    uint8_t bdname_len;
    uint8_t eir_len;
    uint8_t rssi;
    uint32_t cod;
    uint8_t eir[ESP_BT_GAP_EIR_DATA_LEN];
    uint8_t bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
    esp_bd_addr_t bda;
    app_gap_state_t state;
} app_gap_cb_t;



void bt_app_gap_start_up(void);
static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
static char *bda2str(esp_bd_addr_t bda, char *str, size_t size);
static char *uuid2str(esp_bt_uuid_t *uuid, char *str, size_t size);
static bool get_name_from_eir(uint8_t *eir, uint8_t *bdname, uint8_t *bdname_len);
static void update_device_info(esp_bt_gap_cb_param_t *param);
static void bt_app_gap_init(void);
void bt_start_discovery(unsigned int max_scan_time);


#endif



#define BT_BUF_SIZE				 80	// BlueTooth msg buffer size (was 16)


#endif