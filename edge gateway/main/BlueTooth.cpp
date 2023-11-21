//////////////////////////////////////////////
//
// BlueTooth.cpp
//
// aql Ltd
//
// Auth: DLT
//
// based on espressif Systems app_bt.c
// https://github.com/pcbreflux/espressif/blob/master/esp32/app/ble_app_eddystone/main/app_bt.c
//
//////////////////////////////////////////////


#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <memory.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_task_wdt.h"

#include "esp_bt.h"
#include "bt_hci_common.h"
#include "esp_log.h"
#include "nvs_flash.h"


#include "MAC_Rpt.h"		// for BLUETOOTH_CMD_LIST_LENGTH, BLUETOOTH_CMD_LIST_ENTRY_LENGTH] defs...
#include "MAC_Rpt_rtns.h"

#include "BlueTooth.h"

#ifdef USE_BLUETOOTH_CLASSIC_DETECT
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"

#endif



host_rcv_data_t bt_buffer[BT_BUF_SIZE];	//    uint8_t *q_data, + uint16_t q_data_len;

extern unsigned char bt_msg_ready_count;

extern unsigned char continue_ble_commands;

static const char *TAG = "BLE_ADV";

static uint8_t hci_cmd_buf[258];	//128];
extern uint16_t scanned_count;
QueueHandle_t adv_queue;
QueueHandle_t out_queue;

extern unsigned char mqtt_login_state;


#ifdef BLUETOOTH_SCAN
extern unsigned int bluetooth_MAC_entries;
//extern uint64_t* bluetooth_MAC_list;		// Bluetooth MAC addr is 6 bytes
extern struct BT_MAC_INFO* bluetooth_MAC_list[BLUETOOTH_LIST_LEN];
#endif

// I2C values
extern unsigned char bt_sensor_data[];
extern unsigned char bt_sensor_data_len;

extern unsigned char bt_advertising_ready_flag;

//extern unsigned int i2c_temp,i2c_humid,i2c_light,i2c_audio,i2c_batt,i2c_co2,i2c_voc;

extern struct BT_DEVNAME_INFO bluetooth_devname_list[BLUETOOTH_DEVNAME_LIST_LEN];

extern unsigned int bluetooth_devname_entries;
extern unsigned char enable_bluetooth_devname_whitelist_flag;	// Bluetooth device list enable flag

extern unsigned char queues_enabled_flag;

// NOTE array sizes must both use the [BLUETOOTH_CMD_LIST_LENGTH][BLUETOOTH_CMD_LIST_ENTRY_LENGTH] 
// so that they match the parameters of the listmatch() routine in MAC_rpt_rtns.cpp...

char bluetooth_cmd_whitelist[BLUETOOTH_CMD_LIST_LENGTH][BLUETOOTH_CMD_LIST_ENTRY_LENGTH] = 
{
//234567890
	0x1A,0xAA,0xAA,0xAA,0xAA,0xAA
//	0x1B,0xBB,0xBB,0xBB,0xBB,0xBB,
//	0x8D,0x94,0x94,0x36,0x46,0xCA,		// DLT BT puck
//	0x1A,0xF0,0x25,0xA4,0x25,0xE0		// AM  BT puck

};

char bluetooth_cmd_blacklist[BLUETOOTH_CMD_LIST_LENGTH][BLUETOOTH_CMD_LIST_ENTRY_LENGTH] = 
{
//234567890
//	0x2E,0xEE,0xEE,0xEE,0xEE,0xEE, 
//	0x2F,0xFF,0xFF,0xFF,0xFF,0xFF
};

char bluetooth_devname_whitelist[BLUETOOTH_DEVNAME_LIST_LENGTH][BLUETOOTH_DEVNAME_LIST_ENTRY_LENGTH] = 
{
//234567890
//	"Name_1",
//	"FSC-BP108",
//	"P RHT 90094D"
//	0x1B,0xBB,0xBB,0xBB,0xBB,0xBB,
//	0x8D,0x94,0x94,0x36,0x46,0xCA,		// DLT BT puck
//	0x1A,0xF0,0x25,0xA4,0x25,0xE0		// AM  BT puck

};

const char bluetooth_wl_init[BLUETOOTH_CMD_LIST_INIT_LENGTH][BLUETOOTH_CMD_LIST_ENTRY_LENGTH] = 
{
//234567890
//	0x1A,0xAA,0xAA,0xAA,0xAA,0xAA,
//	0x1B,0xBB,0xBB,0xBB,0xBB,0xBB,
	0x8D,0x94,0x94,0x36,0x46,0xCA,		// DLT BT puck
	0x1A,0xF0,0x25,0xA4,0x25,0xE0,		// AM  BT puck
	0x1A,0xAA,0xAA,0xAA,0xAA,0xAA
	
};

const char bluetooth_bl_init[BLUETOOTH_CMD_LIST_INIT_LENGTH][BLUETOOTH_CMD_LIST_ENTRY_LENGTH] = 
{
//234567890
	0x2E,0xEE,0xEE,0xEE,0xEE,0xEE
//	0x2F,0xFF,0xFF,0xFF,0xFF,0xFF
};

const char bluetooth_dw_init[BLUETOOTH_DEVNAME_LIST_INIT_LENGTH][BLUETOOTH_DEVNAME_LIST_ENTRY_LENGTH] = 
{
//   12345678901234567890
	"Name_1",
	"FSC-BP108",
	"P RHT 90094D"
};

const char bluetooth_wl_init_len = 3;
const char bluetooth_bl_init_len = 1;
const char bluetooth_dw_init_len = 3;


extern unsigned int bluetooth_sensor_adv_time;
extern char bluetooth_sensor_adv_name[];


///////////////////////////////////////////////////////////////////////////////////
// from app_bt.c...
#define HCI_H4_CMD_PREAMBLE_SIZE           (4)

/*  HCI Command opcode group field(OGF) */
#define HCI_GRP_HOST_CONT_BASEBAND_CMDS    (0x03 << 10)            /* 0x0C00 */
#define HCI_GRP_BLE_CMDS                   (0x08 << 10)

#define HCI_SET_EVT_MASK                   (0x0001 | HCI_GRP_HOST_CONT_BASEBAND_CMDS)
#define HCI_RESET                          (0x0003 | HCI_GRP_HOST_CONT_BASEBAND_CMDS)
/* Advertising commands */
#define HCI_BLE_WRITE_ADV_PARAMS           (0x0006 | HCI_GRP_BLE_CMDS)
#define HCI_BLE_WRITE_ADV_DATA             (0x0008 | HCI_GRP_BLE_CMDS)
#define HCI_BLE_WRITE_ADV_ENABLE           (0x000A | HCI_GRP_BLE_CMDS)
/* Scan commands */
#define HCI_BLE_WRITE_SCAN_PARAM           (0x000B | HCI_GRP_BLE_CMDS)
#define HCI_BLE_WRITE_SCAN_ENABLE          (0x000C | HCI_GRP_BLE_CMDS)

// in bt_hci_common.h
//#define HCIC_PARAM_SIZE_WRITE_ADV_ENABLE        (1)
//#define HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS    (15)
//#define HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA      (31)

#define BD_ADDR_LEN     (6)                     /* Device address length */
typedef uint8_t bd_addr_t[BD_ADDR_LEN];         /* Device address */

// the following macros put out LS byte first!
#define UINT16_TO_STREAM(p, u16) {*(p)++ = (uint8_t)(u16); *(p)++ = (uint8_t)((u16) >> 8);}
#define UINT8_TO_STREAM(p, u8)   {*(p)++ = (uint8_t)(u8);}
#define BDADDR_TO_STREAM(p, a)   {int ijk; for (ijk = 0; ijk < BD_ADDR_LEN;  ijk++) *(p)++ = (uint8_t) a[BD_ADDR_LEN - 1 - ijk];}
#define ARRAY_TO_STREAM(p, a, len) {int ijk; for (ijk = 0; ijk < len;        ijk++) *(p)++ = (uint8_t) a[ijk];}

// aql - the following macros put out MS byte first!
#define AQL_DATA_UINT16_TO_STREAM(p, u16) { *(p)++ = (uint8_t)((u16) >> 8);  *(p)++ = (uint8_t)(u16);}

/*
// in bt_hci_common.h
enum {
    H4_TYPE_COMMAND = 1,
    H4_TYPE_ACL     = 2,
    H4_TYPE_SCO     = 3,
    H4_TYPE_EVENT   = 4
};
*/
//static uint8_t hci_cmd_buf[128];


// in bt_hci_common.h
#if 0
/*
 * @brief: BT controller callback function, used to notify the upper layer that
 *         controller is ready to receive command
 */
static void controller_rcv_pkt_ready(void)
{
 printf("controller rcv pkt ready\n");
}

/*
 * @brief: BT controller callback function, to transfer data packet to upper
 *         controller is ready to receive command
 */

/*
static int host_rcv_pkt(uint8_t *data, uint16_t len)
{
    printf("host rcv pkt: ");
    for (uint16_t i = 0; i < len; i++) {
        printf("%02x", data[i]);
    }
    printf("\n");
    return 0;
}
*/


static esp_vhci_host_callback_t vhci_host_cb = {
    controller_rcv_pkt_ready,							// tx_ready
    host_rcv_pkt										// rx ready
};
#endif

uint16_t make_cmd_reset(uint8_t *buf)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_RESET);
    UINT8_TO_STREAM (buf, 0);
    return HCI_H4_CMD_PREAMBLE_SIZE;
}


uint16_t make_cmd_set_evt_mask (uint8_t *buf, uint8_t *evt_mask)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
	UINT16_TO_STREAM (buf, HCI_SET_EVT_MASK);
    UINT8_TO_STREAM (buf, HCIC_PARAM_SIZE_SET_EVENT_MASK);
//    ARRAY_TO_STREAM (buf, evt_mask, HCIC_PARAM_SIZE_SET_EVENT_MASK+1);
    ARRAY_TO_STREAM (buf, evt_mask, HCIC_PARAM_SIZE_SET_EVENT_MASK);
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_SET_EVENT_MASK;
}

uint16_t make_cmd_ble_set_adv_enable (uint8_t *buf, uint8_t adv_enable)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_ADV_ENABLE);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_WRITE_ADV_ENABLE);
    UINT8_TO_STREAM (buf, adv_enable);
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_WRITE_ADV_ENABLE;
}

uint16_t make_cmd_ble_set_adv_param (uint8_t *buf, uint16_t adv_int_min, uint16_t adv_int_max,
        uint8_t adv_type, uint8_t addr_type_own,
        uint8_t addr_type_dir, bd_addr_t direct_bda,
        uint8_t channel_map, uint8_t adv_filter_policy)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_ADV_PARAMS);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS );

    UINT16_TO_STREAM (buf, adv_int_min);
    UINT16_TO_STREAM (buf, adv_int_max);
    UINT8_TO_STREAM (buf, adv_type);
    UINT8_TO_STREAM (buf, addr_type_own);
    UINT8_TO_STREAM (buf, addr_type_dir);
    BDADDR_TO_STREAM (buf, direct_bda);
    UINT8_TO_STREAM (buf, channel_map);
    UINT8_TO_STREAM (buf, adv_filter_policy);
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_BLE_WRITE_ADV_PARAMS;
}


#if 1
uint16_t make_cmd_ble_set_adv_data(uint8_t *buf, uint8_t data_len, uint8_t *p_data)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_ADV_DATA);
    UINT8_TO_STREAM  (buf, HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA + 1);

    memset(buf, 0, HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA);

    if (p_data != NULL && data_len > 0) {
        if (data_len > HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA) {
            data_len = HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA;
        }

        UINT8_TO_STREAM (buf, data_len);

        ARRAY_TO_STREAM (buf, p_data, data_len);
    }
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA + 1;
}
#endif


 uint16_t make_cmd_ble_set_scan_params (uint8_t *buf, uint8_t scan_type,
                                       uint16_t scan_interval, uint16_t scan_window, uint8_t own_addr_type,
                                       uint8_t filter_policy)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_SCAN_PARAM);
    UINT8_TO_STREAM (buf, HCIC_PARAM_SIZE_BLE_WRITE_SCAN_PARAM);
    UINT8_TO_STREAM (buf, scan_type);
	UINT16_TO_STREAM (buf, scan_interval);
	UINT16_TO_STREAM (buf, scan_window);
    UINT8_TO_STREAM (buf, own_addr_type);
    UINT8_TO_STREAM (buf, filter_policy);	
    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_BLE_WRITE_SCAN_PARAM;	// 4+7 = 11
}


/**
* ### CHECK bt_hci_common.h!
 * @brief   This function is used to set the data used in advertising packets that have a data field.
 *
 * @param   buf                 Input buffer to write which will be sent to controller.
 * @param   scan_enable         Enable or disable scanning.
 * @param   filter_duplicates   Filter duplicates enable or disable.
 *
 * @return  Size of buf after writing into it.
 */
uint16_t make_cmd_ble_set_scan_enable (uint8_t *buf, uint8_t scan_enable,
                                       uint8_t filter_duplicates)
{
    UINT8_TO_STREAM (buf, H4_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_BLE_WRITE_SCAN_ENABLE);
    UINT8_TO_STREAM (buf, HCIC_PARAM_SIZE_BLE_WRITE_SCAN_ENABLE);
	UINT8_TO_STREAM (buf, scan_enable);
	UINT8_TO_STREAM (buf, filter_duplicates);

    return HCI_H4_CMD_PREAMBLE_SIZE + HCIC_PARAM_SIZE_BLE_WRITE_SCAN_ENABLE;	//4+2 = 6
}


///////////////////////////////////////////////////////////////////////////////////

/*
 * @brief: BT controller callback function, used to notify the upper layer that
 *         controller is ready to receive command
 */
static void controller_rcv_pkt_ready(void)
{
//	bt_advertising_ready_flag = 1;

//	show_time(DBG,1);
//  printf("  BT controller tx pkt ready %d\n",bt_advertising_ready_flag);
}

/*
 * @brief: BT controller callback function, to transfer data packet to upper
 *         controller is ready to receive command
 */
/*
static int host_rcv_pkt(uint8_t *data, uint16_t len)
{
    printf("host rcv pkt: ");
    for (uint16_t i = 0; i < len; i++) {
        printf("%02x", data[i]);
    }
    printf("\n");
    return 0;
}
*/

static int host_rcv_pkt(uint8_t *data, uint16_t len)
{
    host_rcv_data_t send_data;
    uint8_t *data_pkt;
// Line 1268!	
	if(debug_do(DBG_BLUETOOTH))
		{
//		show_time(DBG,1);
//		printf("  ");
#ifdef BT_SNIFFER
		printf("BT rx: ");
#if 1
		for (uint16_t i = 0; i < 7; i++)	//len; i++) 
			{
			printf("%02x", data[i]);
			}

        printf("  ");

		for (uint16_t i = 7; i < 13; i++)	//len; i++) 
			{
			printf("%02x", data[i]);
			}

        printf("  ");
		
		for (uint16_t i = 13; i < (len-1); i++) 
			{
			if ((data[i] > 0x1F) && (data[i] < 0x7F))
				printf("%c", data[i]);
			else
				printf(".");
			}

		printf(" %ddBm", (signed char)data[len-1]);

#endif
		printf("\r\n");

#endif
		}

// example 1: host rcv pkt: 0200200b00070006000104002d100f0f
// 02 00 20 0b 00 07 00 06 00 01 04 00 2d 10 0f 0f
// example 2: host rcv pkt: 04050400000013
// 04 05 04 00 00 00 13
		
    /* Check second byte for HCI event. If event opcode is 0x0e, the event is
     * HCI Command Complete event. Since we have received "0x0e" event, we can
     * check for byte 4 for command opcode and byte 6 for it's return status. */
    if (data[1] == 0x0e) 
		{
        if (data[6] == 0) 
			{
            ESP_LOGI(TAG, "Event opcode 0x%02x success.", data[4]);
			} 
		else 
			{
            ESP_LOGE(TAG, "Event opcode 0x%02x fail with reason: 0x%02x.", data[4], data[6]);
            return ESP_FAIL;
			}
		}

#ifdef USE_M4G_MQTT
	if(debug_do(DBG_BLUETOOTH))
		{
		printf("host_rcv_pkt:M4G\n");
		}
		
	if (len)	// ie, if not zero length...
	{
    data_pkt = (uint8_t *)malloc(sizeof(uint8_t) * len);
    if (data_pkt == NULL) 
		{
        ESP_LOGE(TAG, "Malloc data_pkt failed! [%d]",len);
        return ESP_FAIL;
		}

//	if (mqtt_login_state >= SENSOR_DATA_IDLE)	//UPDATE_SRVR_INFO_WAIT)	// if connected and ready to send
	if (queues_enabled_flag)
		{		
		memcpy(data_pkt, data, len);
		send_data.q_data = data_pkt;
		send_data.q_data_len = len;
		if (xQueueSend(adv_queue, (void *)&send_data, ( TickType_t ) 0) != pdTRUE) 
			{
			ESP_LOGD(TAG, "Failed to enqueue advertising report. Queue full.");
			/* If data sent successfully, then free the pointer in `xQueueReceive'
			 * after processing it. Or else if enqueue in not successful, free it
			 * here. */
			free(data_pkt);
			}
		}
	else											// not connected and ready to send
		free(data_pkt);
	}
#endif
	if(debug_do(DBG_BLUETOOTH))
		{
		printf("host_rcv_pkt:END\n");
		}

    return ESP_OK;
}
// vhci_host_cb structure has:
// vhci_callback.notify_host_send_available = notifyHostSendAvailable;  
// vhci_callback.notify_host_recv = notifyHostRecv;
static esp_vhci_host_callback_t vhci_host_cb = {
    controller_rcv_pkt_ready,							// tx_ready
    host_rcv_pkt										// rx ready
};

static void hci_cmd_send_reset(void)
{
    uint16_t sz = make_cmd_reset (hci_cmd_buf);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_set_evt_mask(void)
{
    /* Set bit 61 in event mask to enable LE Meta events. */
    uint8_t evt_mask[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20};
    uint16_t sz = make_cmd_set_evt_mask(hci_cmd_buf, evt_mask);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

static void hci_cmd_send_ble_set_adv_param(void)
{
// adv time:0x0020 to 0x4000 Default: N = 0x0800 (1.28 second) 
// Time = N * 0.625 msec Time Range: 20 ms to 10.24 sec
    uint16_t adv_intv_min = bluetooth_sensor_adv_time;	//BLUETOOTH_ADVERTISING_MIN_TIME; // 1 sec: 256 = 160ms	1.6 * ms value
    uint16_t adv_intv_max = bluetooth_sensor_adv_time;	//BLUETOOTH_ADVERTISING_MAX_TIME; // 1 sec: 256 = 160ms
    uint8_t adv_type = 0; // connectable undirected advertising (ADV_IND)
    uint8_t own_addr_type = 0; // Public Device Address
    uint8_t peer_addr_type = 0; // Public Device Address
    uint8_t peer_addr[6] = {0x80, 0x81, 0x82, 0x83, 0x84, 0x85};
    uint8_t adv_chn_map = 0x07; // 37, 38, 39
    uint8_t adv_filter_policy = 0; // Process All Conn and Scan

    uint16_t sz = make_cmd_ble_set_adv_param(hci_cmd_buf,
                  adv_intv_min,
                  adv_intv_max,
                  adv_type,
                  own_addr_type,
                  peer_addr_type,
                  peer_addr,
                  adv_chn_map,
                  adv_filter_policy);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}

//unsigned int i2c_temp,i2c_humid,i2c_light,i2c_audio,i2c_batt,i2c_co2,i2c_voc;


// 02 01 06 0E 09 45 53 50 2D 42 4C 45 2D 48 45 4C 4C 4F
// LEN 2: 01     06

// LEN 14: 09    45 53 50 2D 42 4C 45 2D 48 45 4C 4C 4F	// dev name field
//         Name   E  S  P  -  B  L  E  -  H  E  L  L  O

//static void hci_cmd_send_ble_set_adv_data(char* adv_name, uint8_t* adv_data,uint8_t adv_data_len)
#if 0
static void hci_cmd_send_ble_set_adv_data(void)
{
    char *adv_name = BLUETOOTH_ADVERTISING_NAME;	//"ESP-BLE-HELLO"; Now in MAC_Rpt.h...
    uint8_t name_len = (uint8_t)strlen(adv_name);
    uint8_t adv_data[31] = {0x02, 0x01, 0x06, 0x0, 0x09};
    uint8_t adv_data_len;

    adv_data[3] = name_len + 1;
    for (int i = 0; i < name_len; i++) {
        adv_data[5 + i] = (uint8_t)adv_name[i];
    }
    adv_data_len = 5 + name_len;

    uint16_t sz = make_cmd_ble_set_adv_data(hci_cmd_buf, adv_data_len, (uint8_t *)adv_data);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}
#endif

#if 1
//unsigned int i2c_temp,i2c_humid,i2c_light,i2c_audio,i2c_batt,i2c_co2,i2c_voc;
extern unsigned int bluetooth_sensor_adv_time;
extern char bluetooth_sensor_adv_name[];

static void hci_cmd_send_ble_set_adv_data(void)
{
// watch out for limit of 31 chars!
    char *adv_name = bluetooth_sensor_adv_name;	//BLUETOOTH_ADVERTISING_NAME;	//"ESP-BLE-HELLO"; Now in MAC_Rpt.h...
    uint8_t name_len = (uint8_t)strlen(adv_name);
    uint8_t adv_data[HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA] = {0x02, 0x01, 0x06};
    uint8_t adv_data_len;

	uint8_t p = 0;	
	
	uint8_t* buf = &adv_data[3]; 
	
	p = 3;
	

//	printf("I2C Val: %04X %04X %04X %04X %04X %04X %04X\n",i2c_temp,i2c_humid,i2c_light,i2c_audio,i2c_batt,i2c_co2,i2c_voc);
	
/*
// aql standard - put out MS byte FIRST - so use the AQL_DATA_ macros...
	AQL_DATA_UINT16_TO_STREAM(buf,i2c_temp);		// 14 bytes of data...
	AQL_DATA_UINT16_TO_STREAM(buf,i2c_humid);
	AQL_DATA_UINT16_TO_STREAM(buf,i2c_light);
	AQL_DATA_UINT16_TO_STREAM(buf,i2c_audio);
	AQL_DATA_UINT16_TO_STREAM(buf,i2c_batt);
	AQL_DATA_UINT16_TO_STREAM(buf,i2c_co2);
	AQL_DATA_UINT16_TO_STREAM(buf,i2c_voc);
//	UINT16_TO_STREAM(buf,0x00);		// dummy to make up complete 32 bit chunks in this field type
*/	
	if (bt_sensor_data_len)
		{
		UINT8_TO_STREAM(buf,bt_sensor_data_len+1);			// length of data field
		UINT8_TO_STREAM(buf,0x05);			// data field type

		p = p + 2;

// aql standard - put out MS byte FIRST - so use the AQL_DATA_ macros...
		for (unsigned char i = 0;i< bt_sensor_data_len;i++)
			{
			UINT8_TO_STREAM(buf,bt_sensor_data[i]);
			}
			
		p = p + bt_sensor_data_len;
		bt_sensor_data_len = 0;
		}
	
    adv_data[p++] = name_len + 1;
	adv_data[p++] =0x09;	// Local name field
    for (int i = 0; i < name_len; i++) 
		{
        adv_data[p + i] = (uint8_t)adv_name[i];
		}
    adv_data_len = p + name_len;

	if(debug_do(DBG_BLUETOOTH))
		{
		if (p > HCIC_PARAM_SIZE_BLE_WRITE_ADV_DATA)
			printf("### BT ADV - size error!\n");
		}
	
    uint16_t sz = make_cmd_ble_set_adv_data(hci_cmd_buf, adv_data_len, (uint8_t *)adv_data);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}
#endif

static void hci_cmd_send_ble_adv_start(void)
{
    uint16_t sz = make_cmd_ble_set_adv_enable (hci_cmd_buf, 1);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
//    ESP_LOGI(TAG, "BLE Advertising started..");
}

static void hci_cmd_send_ble_scan_params(void)
{
    /* Set scan type to 0x01 for active scanning and 0x00 for passive scanning. */
    uint8_t scan_type = 0x01;

    /* Scan window and Scan interval are set in terms of number of slots. Each slot is of 625 microseconds. */
    uint16_t scan_interval = 0x50; /* 50 ms */
    uint16_t scan_window = 0x30; /* 30 ms */

    uint8_t own_addr_type = 0x00; /* Public Device Address (default). */
    uint8_t filter_policy = 0x00; /* Accept all packets except directed advertising packets (default). */
    uint16_t sz = make_cmd_ble_set_scan_params(hci_cmd_buf, scan_type, scan_interval, scan_window, own_addr_type, filter_policy);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
}




static void hci_cmd_send_ble_scan_start(void)
{
    uint8_t scan_enable = 0x01; /* Scanning enabled. */
    uint8_t filter_duplicates = 0x00; /* Duplicate filtering disabled. */
    uint16_t sz = make_cmd_ble_set_scan_enable(hci_cmd_buf, scan_enable, filter_duplicates);
    esp_vhci_host_send_packet(hci_cmd_buf, sz);
//    ESP_LOGI(TAG, "BLE Scanning started..");
}

void bleScanSetup(void)
{
    int cmd_cnt = 0;
	unsigned char n;
    bool send_avail = false;
 
    printf("BLE scan setup start\n");

/* A queue for storing received HCI packets. */
	adv_queue = xQueueCreate(BT_BUF_SIZE, sizeof(host_rcv_data_t)); // was 15
	if (adv_queue == NULL) 
		{
		ESP_LOGE(TAG, "adv_queue creation failed\n");
		}
/* Msg output queue for msgs to server. */
	out_queue = xQueueCreate(BT_BUF_SIZE, sizeof(host_rcv_data_t));	// was 15
	if (out_queue == NULL) 
		{
		ESP_LOGE(TAG, "out_queue creation failed\n");
		}


	esp_vhci_host_register_callback(&vhci_host_cb);
	xTaskCreatePinnedToCore(&hci_evt_process,"hci_evt_process", 2048, NULL, 6, NULL, 0);

}

void ble_set_adv_data(void)
{
	hci_cmd_send_ble_set_adv_data();
}

/*
 * @brief: send HCI commands to perform BLE advertising;
 */
void bleAdvtTask(void)	// *pvParameters)
{
    int cmd_cnt = 0;
	unsigned char n;
    bool send_avail = false;
//    esp_vhci_host_register_callback(&vhci_host_cb);

//    printf("BLE advt task start\n");
	continue_ble_commands = 1;
	
//    while (1)
	while (continue_ble_commands)
		{


//for (n=0;n<100;n++)
			{
// reset WatchDog Timer for this thread...
			esp_task_wdt_reset();		
			vTaskDelay(10 / portTICK_PERIOD_MS);
			}

		if (continue_ble_commands) 
			{
			
			send_avail = esp_vhci_host_check_send_available();

//			printf("BLE Advertise, flag_send_avail: %d, cmd_sent: %d\n", send_avail, cmd_cnt);

			if (send_avail) 
				{
				switch (cmd_cnt) 
					{
					case 0: hci_cmd_send_reset(); ++cmd_cnt; break;
					case 1: hci_cmd_send_set_evt_mask(); ++cmd_cnt; break;
				//case 1: ++cmd_cnt; break;
#ifdef USE_BLUETOOTH_ADVERTISING
// Advertising commands
					case 2: hci_cmd_send_ble_set_adv_param(); ++cmd_cnt; break;
					case 3: hci_cmd_send_ble_set_adv_data(); ++cmd_cnt; break;
					case 4: hci_cmd_send_ble_adv_start(); ++cmd_cnt; break;
// Scan commands
					case 5: hci_cmd_send_ble_scan_params(); ++cmd_cnt; break;
					case 6: hci_cmd_send_ble_scan_start(); ++cmd_cnt; break;
#else
// Scan commands
					case 2: hci_cmd_send_ble_scan_params(); ++cmd_cnt; break;
					case 3: hci_cmd_send_ble_scan_start(); ++cmd_cnt; break;
#endif
					default: continue_ble_commands = 0; break;

					}
				
				if(debug_do(DBG_BLUETOOTH))
					{
					printf("BLE Advertise, flag_send_avail: %d, cmd_sent: %d\n", send_avail, cmd_cnt);
					}
				}
			}
		else
			cmd_cnt = 0;

		}

}


void bleScantask(void)
{
    int cmd_cnt = 0;
	unsigned char n;
    bool send_avail = false;
 /*
// A queue for storing received HCI packets. 
	adv_queue = xQueueCreate(15, sizeof(host_rcv_data_t));
	if (adv_queue == NULL) {
		ESP_LOGE(TAG, "Queue creation failed\n");
	}


 esp_vhci_host_register_callback(&vhci_host_cb);
    printf("BLE scan setup start\n");
*/
while (continue_ble_commands) 
			{
			for (n=0;n<100;n++)
				{
// reset WatchDog Timer for this thread...
				esp_task_wdt_reset();		
				vTaskDelay(10 / portTICK_PERIOD_MS);
				}
			
			send_avail = esp_vhci_host_check_send_available();


			if (send_avail) 
				{
				printf("BLE Scan Setup, cmd_sent: %d\n", cmd_cnt);
				
				switch (cmd_cnt) 
					{
					case 0: hci_cmd_send_reset(); ++cmd_cnt; break;					// 0x03
//				case 1: hci_cmd_send_set_evt_mask(); ++cmd_cnt; break;
					case 1: ++cmd_cnt; break;										// 0x01
// Advertising commands
					case 2: hci_cmd_send_ble_set_adv_param(); ++cmd_cnt; break;		// 0x06
					case 3: hci_cmd_send_ble_set_adv_data(); ++cmd_cnt; break;		// 0x08
					case 4: hci_cmd_send_ble_adv_start(); ++cmd_cnt; break;			// 0x0A
// Scan commands
					case 5: hci_cmd_send_ble_scan_params(); ++cmd_cnt; break;		// 0x0B
					case 6: hci_cmd_send_ble_scan_start(); ++cmd_cnt; break;		// 0x0C
					default: continue_ble_commands = 0; break;

					}
				}
			}

}


void hci_evt_process(void *pvParameters)
{
// This is pinned to the core functions and executes in the background as as separate process.

// example 1: host rcv pkt: 0200200b00070006000104002d100f0f
// 02 00 20 0b 00 07 00 06 00 01 04 00 2d 10 0f 0f
// example 2: host rcv pkt: 04050400000013
// 04 05 04 00 00 00 13


// 04 3e 28 02 01 03 00 a1 5b 3c ef c0 84 1c 1b ff 75 00
//	04 = HCI_TYPE_EVENT
//	3E = LE_META_EVENT
//	28 = data length
//	02 = HCI_LE_ADV_REPORT
//	01 = num_responses
//	03 = report 1 data len
//	00 = report 1 data byte1
//	A1 = report 1 data byte2
//	5B = report 1 data byte3
//	3C EF C0 84 1C 1B = BD address for report 1
//	FF = data len for report 1

// It appears that a <single> returned data block may contain a <number> of responses 
// (as indicated by num_responses held at offset 6). The data sections are then arranged in lists by type:

// num_responses * event_type	(n * 1 byte)
// num_responses * adv_type		(n * 1 byte - advertising frame type)
// num_responses * addr			(n * 6 bytes)
// num_responses * data_len		(n * 1 byte)

// The data lengths \ payload sizes may all be different; hence the msg_data for all msgs is placed 
// in a single data block:

// data_msg						(no of bytes = sum of all data_lens in previous list

// num_responses * rssi			(n * 1 byte)
// num_responses * scanned_name	(n * (32 bytes name + 1 byte length))


unsigned char i,prt_raw_flag,mac_list_len, mac_list_ptr;
char mac_addr[7];
unsigned char mac_list[10][6];
unsigned char mac_list_str[8];
unsigned long long int mac_list_addr;

	ESP_LOGE(TAG, "HCI EVT PROCESS!");

    host_rcv_data_t *rcv_data = (host_rcv_data_t *)malloc(sizeof(host_rcv_data_t)); // Q ptr + 1 byte Q len...
    if (rcv_data == NULL) {
        ESP_LOGE(TAG, "Malloc rcv_data failed!");
        return;
    }
    esp_err_t msg_with_local_name, ret;

    host_rcv_data_t send_data;
	
// DLT puck MAC addr
	mac_list[0][0] = 0xCA;
	mac_list[0][1] = 0x46;
	mac_list[0][2] = 0x36;
	mac_list[0][3] = 0x94;
	mac_list[0][4] = 0x94;
	mac_list[0][5] = 0x8D;

// AM puck MAC addr
	mac_list[1][0] = 0xE0;
	mac_list[1][1] = 0x25;
	mac_list[1][2] = 0xA4;
	mac_list[1][3] = 0x25;
	mac_list[1][4] = 0xF0;
	mac_list[1][5] = 0x1A;

	mac_list_len = 2;
	
    while (1) 
		{
        uint8_t sub_event, num_responses, total_data_len, data_msg_ptr, hci_event_opcode;
        uint8_t *queue_data = NULL, *event_type = NULL, *adv_type = NULL, *addr = NULL, *data_len = NULL, *data_msg = NULL;
//        short int *rssi = NULL;
        uint8_t *rssi = NULL;
        uint16_t data_ptr;
		
		uint8_t *msg_out = NULL;
		
		char c;
		unsigned char j,k,n;
		unsigned char reason;
		uint8_t dm_offset;


#ifdef BLUETOOTH_SCAN
//		unsigned char found_flag;
#endif
//		unsigned char send_bluetooth_data_flag = 1;
        uint8_t *found_flag = NULL, *send_bluetooth_data_flag = NULL;
		nvs_handle bt_mac_handle;
		char nvs_str[16];
		char tmp_str[6];
		
		reason = 0;
		
        ble_scan_local_name_t *scanned_name = NULL;
        total_data_len = 0;
        data_msg_ptr = 0;
        if (xQueueReceive(adv_queue, rcv_data, portMAX_DELAY) != pdPASS) 
			{
            ESP_LOGE(TAG, "Queue receive error");
			} 
		else
			{
			prt_raw_flag = 0;
            /* `data_ptr' keeps track of current position in the received data. */
            data_ptr = 0;
            queue_data = rcv_data->q_data;

#if 0
// check to see if it is on the MAC address list...
			for (i=0;i<6;i++)
				{
				mac_addr[i] = queue_data[12-i];
				}
			mac_addr[i] = 0x00;
/*
			mac_addr[0] = queue_data[12];
			mac_addr[1] = queue_data[11];
			mac_addr[2] = queue_data[10];
			mac_addr[3] = queue_data[9];
			mac_addr[4] = queue_data[8];
			mac_addr[5] = queue_data[7];
			mac_addr[7] = 0x00;
*/
			if(debug_do(DBG_BLUETOOTH))
				printf("\nBT MAC: %02X:%02X:%02X:%02X:%02X:%02X\t\t",mac_addr[0],mac_addr[1],mac_addr[2],mac_addr[3],mac_addr[4],mac_addr[5]);			

#ifdef BLUETOOTH_SCAN
				found_flag = 0;

// add this MAC addr to the list of MAC addresses
			if (bluetooth_MAC_entries < BLUETOOTH_LIST_LEN)
				{
				unsigned int i;
//				unsigned char found_flag = 0;

// check first that the MAC address isnt already on the list?					
				for (i=0;i<bluetooth_MAC_entries;i++)
					{
					if (!memcmp(mac_addr,bluetooth_MAC_list[i]->bluetooth_MAC_addr,6))
						{
						if(debug_do(DBG_BLUETOOTH))
							printf("BT MAC already on MAC scan list!\n");
						
						found_flag = 1;
						}
					}

// While the data contains only 1 MAC addr, it may contain multiple responses each with own RSSI. Need to get just the RSSI of the first response
// So if the found_flag is clear, get the MAC_addr and rssi later as each response is decoded... (see line )

/*
				if (!found_flag)						// if no match found on the list
					{
					memcpy(&bluetooth_MAC_list[bluetooth_MAC_entries].bluetooth_MAC_addr,mac_addr,6);	// add new MAC addr to the list
					bluetooth_MAC_list[bluetooth_MAC_entries].bluetooth_rssi =  
					bluetooth_MAC_entries++;
					}
				else
					printf("BT MAC already on MAC scan list!\n");
				
*/				
				}

#endif

/*
#ifdef USE_BLUETOOTH_MAC_FILTERING
			nvs_open("BT_MAC_LIST",NVS_READWRITE, &bt_mac_handle);
//			nvs_set_i8(bt_mac_handle,"BT_MAC_MAX",1);
//			nvs_commit(bt_mac_handle);

			nvs_get_i8(bt_mac_handle,"BT_MAC_MAX",(int8_t*)&mac_list_len);		// get number of items in list
//			printf("Chk Max: %d\n",mac_list_len);
			
			mac_list_ptr = 0;
			for (i=0;i<mac_list_len;i++)
				{
				sprintf(nvs_str,"BT_MAC_%03d",mac_list_ptr);					// make NVS key string for this record
				nvs_get_u64(bt_mac_handle,nvs_str,&mac_list_addr);				// get the data from NVS
//				sprintf(mac_list_addr,"%BT_MAC_%03d",mac_list_ptr);
//				if (!strncmp((char*)mac_addr,(char*)mac_list[i],6))

//				memcpy(tmp_str,&mac_list_addr,6);
//				printf("\nCHK MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",tmp_str[0],tmp_str[1],tmp_str[2],tmp_str[3],tmp_str[4],tmp_str[5]);			
				
				if (!memcmp((char*)mac_addr,&mac_list_addr,6))
					{
					if(debug_do(DBG_BLUETOOTH_FILT))
						{
						printf ("MAC match - entry %d\n",i);
// if MAC filtering and DBG_BLUEOOTH_FILT is on, show raw data for all packets
						prt_raw_flag = 1;
						}
					i = 0xF0;
					}
				mac_list_ptr++;	
				}

			nvs_close(bt_mac_handle);
				
			if (i == mac_list_len) 
				{
					
				if (debug_do(DBG_BLUETOOTH_FILT))
					printf ("No MAC match\n");
					
				if ((!debug_do(DBG_BLUETOOTH)) && (!debug_do(DBG_BLUETOOTH_FILT)))
					goto reset;
				}

#else
			prt_raw_flag = 1;
#endif
*/

// new MAC filtering:  \ black \ white list
			{
//			unsigned char send_bluetooth_data_flag = 1;

			if (enable_bluetooth_blacklist_flag)
				{
				if(debug_do(DBG_BLUETOOTH))
					printf("BT BL:\n");
				
//				if (listmatch(BLUETOOTH_CMD_LIST_LENGTH, BLUETOOTH_CMD_LIST_ENTRY_LENGTH, (char *)&bluetooth_cmd_blacklist, mac_addr, 1, 1))	//(char*)bt_out_data->q_data,1))
				if (listmatch(num_bt_bl_entries, BLUETOOTH_CMD_LIST_ENTRY_LENGTH, (char *)&bluetooth_cmd_blacklist, mac_addr, 1, 1))	//(char*)bt_out_data->q_data,1))
					{
					send_bluetooth_data_flag = 0;									
					}
				}

			if ((enable_bluetooth_whitelist_flag) && (send_bluetooth_data_flag))		// if whitelist enabled and the data hasn't been blacklisted already...
				{
				if(debug_do(DBG_BLUETOOTH))
					printf("BT WL:\n");

				send_bluetooth_data_flag = 0;
//				if (listmatch(BLUETOOTH_CMD_LIST_LENGTH, BLUETOOTH_CMD_LIST_ENTRY_LENGTH, (char *)&bluetooth_cmd_whitelist, mac_addr, 1, 1))	//(char*)bt_out_data->q_data,1))
				if (listmatch(num_bt_wl_entries, BLUETOOTH_CMD_LIST_ENTRY_LENGTH, (char *)&bluetooth_cmd_whitelist, mac_addr, 1, 1))	//(char*)bt_out_data->q_data,1))
					{
					send_bluetooth_data_flag = 1;
					}
				}							

/*
// MOVED TO LINE 1074
// if MAC address either blacklisted or not on the whitelist, dont process the msg
			if (!send_bluetooth_data_flag)
				{
				printf("BT MAC not valid\n");
				goto reset;
				}
*/
			}
#endif// end of "#if 0/1..."
			
// if MAC filtering and DBG_BLUEOOTH is on, show raw data for all packets
			if(debug_do(DBG_BLUETOOTH))
				prt_raw_flag = 1;

			
/*
			printf("\nBLE rcv event!\n");
			{
			unsigned char i;
			
			for (i=0;i<8;i++)
				{
				printf("%02X ",queue_data[i]);	
				}
			printf("\n");
			}
*/		

            /* Parsing `data' and copying in various fields. */
            hci_event_opcode = queue_data[++data_ptr];
            if (hci_event_opcode == LE_META_EVENTS) 	// 0x3E
				{
                /* Set `data_ptr' to 4th entry, which will point to sub event. */
                data_ptr += 2;
                sub_event = queue_data[data_ptr++];
                /* Check if sub event is LE advertising report event. */
                if (sub_event == HCI_LE_ADV_REPORT) 	// 0x02
					{
                    scanned_count += 1;

                    /* Get number of advertising reports. */
                    num_responses = queue_data[data_ptr++];
					if 	((debug_do(DBG_BLUETOOTH)) && (num_responses > 1))
						printf("*** multiple msgs found ***\n");
					
                    event_type = (uint8_t *)malloc(sizeof(uint8_t) * num_responses);
                    if (event_type == NULL) 
						{
                        ESP_LOGE(TAG, "Malloc event_type failed!");
						reason = 1;
                        goto reset;
						}
						
                    for (uint8_t i = 0; i < num_responses; i += 1) 
						{
                        event_type[i] = queue_data[data_ptr++];
						}

                    /* Get advertising type for every report. */
                    adv_type = (uint8_t *)malloc(sizeof(uint8_t) * num_responses);
                    if (adv_type == NULL) 	
						{
                        ESP_LOGE(TAG, "Malloc adv_type failed!");
						reason = 2;
                        goto reset;
						}
						
                    for (uint8_t i = 0; i < num_responses; i += 1)
						{
                        adv_type[i] = queue_data[data_ptr++];
						}

                    /* Get BT address in every advertising report and store in
                     * single array of length `6 * num_responses' as each address
                     * will take 6 spaces. */
                    addr = (uint8_t *)malloc(sizeof(uint8_t) * 6 * num_responses);
                    if (addr == NULL) 
						{
                        ESP_LOGE(TAG, "Malloc addr failed!");
						reason = 3;
                        goto reset;
						}

#ifdef BLUETOOTH_SCAN
                    found_flag = (uint8_t *)malloc(sizeof(uint8_t) * num_responses);
                    if (found_flag == NULL) 
						{
                        ESP_LOGE(TAG, "Malloc found_flag failed!");
						reason = 4;
                        goto reset;
						}
#endif

                    for (int i = 0; i < num_responses; i += 1) 
						{
						unsigned char j;
// MAC addrs are reversed in the message, so extract in correct order:						
                        for (j = 0; j < 6; j += 1) 
							{
//                          addr[(6 * i) + j] = queue_data[data_ptr];		// addr is the array of MAC addresses for all responses
//							mac_addr[j] = queue_data[data_ptr++];			// mac_addr is the single MAC address for this response
                            addr[(6 * i) + 5 - j] = queue_data[data_ptr];		// addr is the array of MAC addresses for all responses
							mac_addr[5 - j] = queue_data[data_ptr++];			// mac_addr is the single MAC address for this response
							}
						mac_addr[6] = 0x00;

						
						if(debug_do(DBG_BLUETOOTH))
							printf("\nBT MAC: %02X:%02X:%02X:%02X:%02X:%02X\t\t",mac_addr[0],mac_addr[1],mac_addr[2],mac_addr[3],mac_addr[4],mac_addr[5]);			

#ifdef BLUETOOTH_SCAN
						found_flag[i] = 0;

// add this MAC addr to the list of MAC addresses
						if (bluetooth_MAC_entries < BLUETOOTH_LIST_LEN)
							{
							unsigned int j;
//				unsigned char found_flag = 0;

// check first that the MAC address isnt already on the list?					
							for (j=0;j<bluetooth_MAC_entries;j++)
								{
								if (!memcmp(mac_addr,bluetooth_MAC_list[j]->bluetooth_MAC_addr,6))
									{
									if(debug_do(DBG_BLUETOOTH))
										printf("BT MAC already on MAC scan list!\n");
									
									found_flag[i] = 1;
									}
								}
							
							}
#endif
// new MAC filtering:  \ black \ white list

						send_bluetooth_data_flag = (uint8_t *)malloc(sizeof(uint8_t) * num_responses);
						if (send_bluetooth_data_flag == NULL) 
							{
							ESP_LOGE(TAG, "Malloc send_bluetooth_data_flag failed!");
							reason = 5;
							goto reset;
							}

///////////////////////////////////////
// BLUETOOTH MAC BLACK \ WHITELISTING
///////////////////////////////////////
						send_bluetooth_data_flag[i] = 1;

						if (enable_bluetooth_blacklist_flag)
							{
							if(debug_do(DBG_BLUETOOTH))
								printf("BT BL:\n");
							
//							if (listmatch(BLUETOOTH_CMD_LIST_LENGTH, BLUETOOTH_CMD_LIST_ENTRY_LENGTH, (char *)&bluetooth_cmd_blacklist, mac_addr, 1, 1))	//(char*)bt_out_data->q_data,1))
							if (listmatch(num_bt_bl_entries, BLUETOOTH_CMD_LIST_ENTRY_LENGTH, (char *)&bluetooth_cmd_blacklist, mac_addr, 1, 1))	//(char*)bt_out_data->q_data,1))
								{
								send_bluetooth_data_flag[i] = 0;									
								}
							}

						if ((enable_bluetooth_whitelist_flag) && (send_bluetooth_data_flag))		// if whitelist enabled and the data hasn't been blacklisted already...
							{
							if(debug_do(DBG_BLUETOOTH))
								printf("BT WL:\n");

//							send_bluetooth_data_flag[i] = 0;
//							if (!listmatch(BLUETOOTH_CMD_LIST_LENGTH, BLUETOOTH_CMD_LIST_ENTRY_LENGTH, (char *)&bluetooth_cmd_whitelist, mac_addr, 1, 1))	//(char*)bt_out_data->q_data,1))
							if (!listmatch(num_bt_wl_entries, BLUETOOTH_CMD_LIST_ENTRY_LENGTH, (char *)&bluetooth_cmd_whitelist, mac_addr, 1, 1))	//(char*)bt_out_data->q_data,1))
								{
//								send_bluetooth_data_flag[i] = 1;
								send_bluetooth_data_flag[i] = 0;
								}
							}							



						}

                    /* Get length of data for each advertising report. */
                    data_len = (uint8_t *)malloc(sizeof(uint8_t) * num_responses);
                    if (data_len == NULL) 
						{
                        ESP_LOGE(TAG, "Malloc data_len failed!");
						reason = 6;
                        goto reset;
						}
						
                    for (uint8_t i = 0; i < num_responses; i += 1) 
						{
                        data_len[i] = queue_data[data_ptr];
                        total_data_len += queue_data[data_ptr++];
						}

                    if (total_data_len != 0) 
						{
                        /* Get all data packets. */
                        data_msg = (uint8_t *)malloc(sizeof(uint8_t) * total_data_len);
                        if (data_msg == NULL) 
							{
                            ESP_LOGE(TAG, "Malloc data_msg failed!");
							reason = 7;
                            goto reset;
							}
							
                        for (uint8_t i = 0; i < num_responses; i += 1) 
							{
                            for (uint8_t j = 0; j < data_len[i]; j += 1) 
								{
                                data_msg[data_msg_ptr++] = queue_data[data_ptr++];
								}
							}
						}

//                    rssi = (short int *)malloc(sizeof(short int) * num_responses);
                    rssi = (uint8_t *)malloc(sizeof(int8_t) * num_responses);
                    if (data_len == NULL) 
						{
                        ESP_LOGE(TAG, "Malloc rssi failed!");
						reason = 8;
                        goto reset;
						}

// THIS SECTION will print the RSSI for all msgs received; 
// the later section only prints for msgs with a valid local name...						
                    for (uint8_t i = 0; i < num_responses; i += 1) 
						{
//                        rssi[i] = -(0xFF - queue_data[data_ptr++]);
                        rssi[i] = queue_data[data_ptr++];
						if(debug_do(DBG_BLUETOOTH))
							printf("RSSI HEX[%d]: %02X [%d]\n",i,queue_data[data_ptr-1],num_responses);
						}

                    /* Extracting advertiser's name. */
                    data_msg_ptr = 0;
                    scanned_name = (ble_scan_local_name_t *)malloc(num_responses * sizeof(ble_scan_local_name_t));
                    if (scanned_name == NULL) 
						{
                        ESP_LOGE(TAG, "Malloc scanned_name failed!");
						reason = 9;
                        goto reset;
						}
						
					dm_offset = 0;
                    for (uint8_t i = 0; i < num_responses; i += 1) 
						{
						char local_name[32];		// packet size is 31 max, minus 6 header...
						
						strcpy(local_name,"NOT VALID");		// test to see if a valid local name is correctly inserted on this cycle...
						
                        msg_with_local_name = get_local_name(&data_msg[data_msg_ptr], data_len[i], scanned_name);

                        /* Print the data if adv report has a valid name. */
                        if (msg_with_local_name == ESP_OK) 
							{
//							char local_name[25];		// packet size is 31 max, minus 6 header...
							if(debug_do(DBG_BLUETOOTH))
								{
								printf("\n");
								show_time(DBG,1);
								printf("  ******** Bluetooth Msg: Response %d/%d ********\n", i + 1, num_responses);
								printf("Event type: %02x\nAddress type: %02x\nAddress: ", event_type[i], adv_type[i]);
//XX                            for (int j = 5; j >= 0; j -= 1) 
								for (int j = 0; j < 6; j++) 
									{
									printf("%02x", addr[(6 * i) + j]);
									if (j < 5) 
										{
										printf(":");
										}
									}

								printf("\nData length: %d", data_len[i]);
								printf("\nAdvertisement Name: ");
								for (int k = 0; k < scanned_name->name_len; k += 1 ) 
									{
									printf("%c", scanned_name->scan_local_name[k]);
									}
									
								printf("\nRSSI: %ddBm\n", (signed char)rssi[i]);
								}
                            data_msg_ptr += data_len[i];
							prt_raw_flag = 1;


// copy scanned_local_name into local_name and check if local name contains a pipe character...
							{
							unsigned int p = 0;
							
							for (int k=0; k<scanned_name->name_len;k++)
								{
								if (scanned_name->scan_local_name[k] == '|')	// "pipe" character...
									{
									local_name[p++] = 0xC2;	// replace pipe with "broken bar pipe"
									local_name[p++] = 0xA6;
									}
								else
									local_name[p++] = scanned_name->scan_local_name[k];
								}

//							memcpy(local_name,scanned_name->scan_local_name,scanned_name->name_len);
//							local_name[scanned_name->name_len] = 0x00;
							local_name[p] = 0x00;
							}

///////////////////////////////////////
// BLUETOOTH LOCAL NAME WHITELISTING
///////////////////////////////////////							
// check for devname match and add msg to output queue
							if (enable_bluetooth_devname_whitelist_flag)
								{
//								memcpy(local_name,scanned_name->scan_local_name,scanned_name->name_len);
//								local_name[scanned_name->name_len] = 0x00;
								
//								if (listmatch(BLUETOOTH_DEVNAME_LIST_LENGTH, BLUETOOTH_DEVNAME_LIST_ENTRY_LENGTH, (char *)&bluetooth_devname_whitelist, local_name, 0, 1))	//(char*)bt_out_data->q_data,1))
								if (listmatch(bluetooth_devname_entries, BLUETOOTH_DEVNAME_LIST_ENTRY_LENGTH, (char *)&bluetooth_devname_whitelist, local_name, 0, 1))	//(char*)bt_out_data->q_data,1))
									{
									unsigned char mac_found_flag = 0;
									
	//								printf("Devname: match found\n");
									if(debug_do(DBG_BLUETOOTH))
										printf("Devname: match found: %s\n",local_name);
									
									for (int x=0;x<bluetooth_devname_entries;x++)
										{
										if (!memcmp(bluetooth_devname_list[x].bluetooth_MAC_addr,&addr[6*i],6))
											mac_found_flag = 1;
										}
										
									if ((msg_with_local_name == ESP_OK) && (!mac_found_flag))		// if msg has local name and is not on mac list...
										{
										if (bluetooth_devname_entries < BLUETOOTH_DEVNAME_LIST_LENGTH)
											{
											strcpy(bluetooth_devname_list[bluetooth_devname_entries].bluetooth_devname,local_name);
											memcpy(bluetooth_devname_list[bluetooth_devname_entries].bluetooth_MAC_addr,&addr[6*i],6);
											bluetooth_devname_list[bluetooth_devname_entries].bluetooth_rssi = rssi[i];

											bluetooth_devname_entries++;
											}
										else
											{
											if(debug_do(DBG_BLUETOOTH))											
												printf("Device table full!\n");
											}
										}
									else
										{
										if(debug_do(DBG_BLUETOOTH))
											printf("Device already in list!\n");
										}
									}
								else
									{
									if(debug_do(DBG_BLUETOOTH))
										printf("Devname: match not found\n");
									}
								}
							}
/*
						}

						
						
// check for MAC match and add msg to output queue
					dm_offset = 0;
                    for (uint8_t i = 0; i < num_responses; i += 1) 
						{
//						uint8_t n;
*/
						
// check to see if it is on the MAC address list...
#if 0
						for (j=0;j<6;j++)
							{
							mac_addr[j] = queue_data[12-j];
							}

						mac_addr[j] = 0x00;
#endif
						if(debug_do(DBG_BLUETOOTH))
							printf("BT MAC: %02X:%02X:%02X:%02X:%02X:%02X\t\t",addr[(6 * i) + 0],addr[(6 * i) + 1],addr[(6 * i) + 2],addr[(6 * i) + 3],addr[(6 * i) + 4],addr[(6 * i) + 5]);			

#ifdef BLUETOOTH_SCAN
// moved from line 740
						if ((msg_with_local_name == ESP_OK) && (!found_flag[i]))						// if good msg with local name and no match found on the list...
							{
							memcpy(bluetooth_MAC_list[bluetooth_MAC_entries]->bluetooth_MAC_addr,&addr[(6 * i)],6);	// add new MAC addr to the list
							bluetooth_MAC_list[bluetooth_MAC_entries]->bluetooth_rssi = rssi[i];
							strcpy(bluetooth_MAC_list[bluetooth_MAC_entries]->bluetooth_devname,local_name);
//#define DATA_TEST
#ifdef DATA_TEST
							printf("[%d]LOCAL NAME: %s\n",i,bluetooth_MAC_list[bluetooth_MAC_entries]->bluetooth_devname);
							printf("MAC ADDR  : %02X:%02X:%02X:%02X:%02X:%02X\n",addr[6*i],addr[6*i+1],addr[6*i+2],addr[6*i+3],addr[6*i+4],addr[6*i+5]);
							printf("RSSI      : %02X\n",bluetooth_MAC_list[bluetooth_MAC_entries]->bluetooth_rssi);
#endif

							bluetooth_MAC_entries++;
							
//							found_flag = 1;		// just add first address and rssi...
							}
//				else
//					printf("BT MAC already on MAC scan list!\n");
#endif

// moved from line 860...
// if MAC address either blacklisted or not on the whitelist, dont process the msg
#if 0
						if (!send_bluetooth_data_flag)
							{
							if(debug_do(DBG_BLUETOOTH))
								printf("BT MAC not valid\n");
							goto reset;
							}
#endif


// no longer used - use black \ wgite listing instead. See line 813 - we dont get as far as this if the MAc addr desotn match...
/*
#ifdef USE_BLUETOOTH_MAC_FILTERING
						for (j=0;j<mac_list_len;j++)
							{
							if (!strncmp((char*)mac_addr,(char*)mac_list[j],6))
								{
								if(debug_do(DBG_BLUETOOTH_FILT))
									{
									printf ("out MAC match - entry %d\n",i);
									}
								j = 0xF0;
								}
							}
							
// tried all the MAC addresses, or found a valid one...				
						if (j == mac_list_len) 
							{
								
							if (debug_do(DBG_BLUETOOTH_FILT))
								printf ("No out MAC match\n");
								
//							if ((!debug_do(DBG_BLUETOOTH)) && (!debug_do(DBG_BLUETOOTH_FILT)))
//								goto reset;
							}
						else		// valid MAC found - put msg on output queue
#endif
*/
						if ((!send_bluetooth_data_flag[i]) || (!queues_enabled_flag))
							{
							if(debug_do(DBG_BLUETOOTH))
								printf("BT MAC not valid\n");
							}
						else
							{
// outgoing msg:
// MAC addr 		6 bytes
// rssi				1 byte
// data_len			1 byte
// data 			value of above
#ifndef INHIBIT_BT_SENSOR_DATA
							if (data_len[i])		// dont output zero length data packets!
								{
								msg_out = (uint8_t *)malloc(8 + data_len[i]);
								if (msg_out == NULL) 
									{
									ESP_LOGE(TAG, "Malloc Msg Out failed!");
//									printf("Malloc msg_out fail\n");
									reason = 10;
									goto reset;
									}
// MAC addr, rssi, data len:								
								memcpy(msg_out,mac_addr,6);
								msg_out[6] = rssi[i];
								msg_out[7] = data_len[i];
// data
								for (uint8_t k = 0;k<data_len[i];k++)
									{
									msg_out[k+8] = data_msg[dm_offset + k];
									}

// show finished msg_out packet
								if(debug_do(DBG_BLUETOOTH))
									{	
									printf("Msg out to server[%d]:\n",i);
									for (uint8_t k = 0;k<(8+data_len[i]);k++)
										{
										printf("%02X ",msg_out[k]);	
										}
									printf("\n");
									
									printf("MAC addr: %02X:%02X:%02X:%02X:%02X:%02X\t\t",mac_addr[0],mac_addr[1],mac_addr[2],mac_addr[3],mac_addr[4],mac_addr[5]);			
									printf("rssi: %ddBm   data_len: %d\n\n",(signed char)rssi[i],data_len[i]);
									}
								
// put msg onto outgoing msg queue...
								send_data.q_data = msg_out;
								send_data.q_data_len = 8 + data_len[i];
								if (xQueueSend(out_queue, (void *)&send_data, ( TickType_t ) 0) != pdTRUE) 
									{
									ESP_LOGD(TAG, "Failed to enqueue BT msg output. Queue full.");
									if(debug_do(DBG_BLUETOOTH))
										printf("BT msg queue full.\r\n");
									
// msg not put on queue and never used - so free memory taken by msg_out packet
									free(msg_out);
									}
								else
									{
									bt_msg_ready_count++;				// flag to fg to send msg data

									if(debug_do(DBG_BLUETOOTH))
										{
										printf("No of queued BT out_msgs: %d\t\t",bt_msg_ready_count);
										printf("Queue free size: %d\n", uxQueueSpacesAvailable(out_queue));
										}
									}

// free memory taken by msg_out packet
//								free(msg_out);
								}
#endif
								
							}

						dm_offset = dm_offset + data_len[i];
						}							
                    /* Freeing all spaces allocated. */
reset:
					if (reason)
						printf("Err: Reason: %d\n",reason);

                    /* Freeing all spaces allocated. */
                    free(scanned_name);
                    free(rssi);
                    free(data_msg);
                    free(data_len);
                    free(addr);
					free(found_flag);
					free(send_bluetooth_data_flag);
                    free(adv_type);
                    free(event_type);
                }
            }
			
#if 1	//(CONFIG_LOG_DEFAULT_LEVEL_DEBUG || CONFIG_LOG_DEFAULT_LEVEL_VERBOSE)
			if ((debug_do(DBG_BLUETOOTH)) && (prt_raw_flag))
				{
				printf("Raw Data: [ %d ]\n",rcv_data->q_data_len);

				for (uint8_t j = 0; j < rcv_data->q_data_len; j += 1) 
					{
					printf(" %02x", queue_data[j]);

					if (j%16 == 15)
						{
						n = j - (j%16);
						printf("          ");
						for (k=n;k<=j;k++)
							{
							if ((queue_data[k] < 0x20) || (queue_data[k] > 0x7F))
								c = '.';
							else
								c = queue_data[k];
								
							printf("%c", c);
							}
						printf("\n");
						}
					}
					
				j = rcv_data->q_data_len %16;
				n = rcv_data->q_data_len - j;
				if (j)
					{
					printf("          ");
					for (k=0;k<3*(16-j);k++)
						printf(" ");
						
					for (k=n;k<(n+j);k++)
						{
						if ((queue_data[k] < 0x20) || (queue_data[k] > 0x7F))
							c = '.';
						else
							c = queue_data[k];
							
						printf("%c", c);
						}
					printf("\n");
					}
					printf("\n******** Bluetooth msg end ********\n\n");
				}
/*
			for (uint8_t j = 0; j < rcv_data->q_data_len; j += 1) 
				{
                printf(" %02x", queue_data[j]);
				}
			
			printf("\n\n");
            
			for (uint8_t j = 0; j < rcv_data->q_data_len; j += 1) 
				{
				if ((queue_data[j] < 0x20) || (queue_data[j] < 0x7F))
					c = '.';
				else
					c = queue_data[j];
					
                printf("%c", c);
				}
*/

			if(debug_do(DBG_BLUETOOTH))
				printf("\nQueue free size: %d\n", uxQueueSpacesAvailable(adv_queue));
#endif
            free(queue_data);
        }
        memset(rcv_data, 0, sizeof(host_rcv_data_t));
    }
}

static esp_err_t get_local_name (uint8_t *data_msg, uint8_t data_len, ble_scan_local_name_t *scanned_packet)
{
    uint8_t curr_ptr = 0, curr_len, curr_type;
 
//    ESP_LOGE(TAG, "GET LOCAL NAME!");

	while (curr_ptr < data_len) 
		{
        curr_len = data_msg[curr_ptr++];
        curr_type = data_msg[curr_ptr++];
		
        if (curr_len == 0) 
			{
            return ESP_FAIL;
			}

        /* Search for current data type and see if it contains name as data (0x08 or 0x09). */
        if (curr_type == 0x08 || curr_type == 0x09) 
			{
            for (uint8_t i = 0; i < curr_len - 1; i += 1) 
				{
                scanned_packet->scan_local_name[i] = data_msg[curr_ptr + i];
				}
            scanned_packet->name_len = curr_len - 1;
            return ESP_OK;
			} 
		else 
			{
            /* Search for next data. Current length includes 1 octate for AD Type (2nd octate). */
            curr_ptr += curr_len - 1;
			}
		}
    return ESP_FAIL;
}
// SEE https://github.com/espressif/esp-idf/blob/master/examples/bluetooth/hci/ble_adv_scan_combined/main/app_bt.c


///////////////////////////////////////
// BLUETOOTH CLASSIC SCAN CODE
///////////////////////////////////////							

#ifdef USE_BLUETOOTH_CLASSIC_DETECT
// new code for BT Classic detect - see link:
// https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/bluedroid/classic_bt/bt_discovery


static app_gap_cb_t m_dev_info;


void bt_app_gap_start_up(void)
{
    app_gap_cb_t *p_dev = &m_dev_info;

    /* register GAP callback function */
    esp_bt_gap_register_callback(bt_app_gap_cb);

    char *dev_name = "ESP_GAP_INQUIRY";			// BT name of <this> device...
    esp_bt_dev_set_device_name(dev_name);

    /* set discoverable and connectable mode, wait to be connected */
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);


    /* inititialize device information and status */
//    bt_app_gap_init();
//	static void bt_app_gap_init(void)
//{
//    app_gap_cb_t *p_dev = &m_dev_info;
//    p_dev = &m_dev_info;
    memset(p_dev, 0, sizeof(app_gap_cb_t));

    p_dev->state = APP_GAP_STATE_IDLE;
//}

    /* start to discover nearby Bluetooth devices */
//    app_gap_cb_t *p_dev = &m_dev_info;
    p_dev->state = APP_GAP_STATE_DEVICE_DISCOVERING;
//    p_dev = &m_dev_info;
//    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, 10, 0);	// 10 = 10 * 1.28 sec max inquiry time 
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, BLUETOOTH_CLASSIC_SCAN_LENGTH, 0);	// 10 = 10 * 1.28 sec max inquiry time 
	
	
}

void bt_start_discovery(unsigned int max_scan_time)
{
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, max_scan_time, 0);	// 10 = 10 * 1.28 sec max inquiry time 	
}

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    app_gap_cb_t *p_dev = &m_dev_info;
    char bda_str[18];
    char uuid_str[37];

	printf("***   GAP CB   ***\n");
	
    switch (event) {
    case ESP_BT_GAP_DISC_RES_EVT: {
        update_device_info(param);
        break;
    }
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT: {
        if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
            ESP_LOGI(GAP_TAG, "Device discovery stopped.");
 // debug...
			if (p_dev->state == APP_GAP_STATE_DEVICE_DISCOVER_COMPLETE)
				printf("DISC COMPLETE\n");
            if (p_dev->state == APP_GAP_STATE_DEVICE_DISCOVERING)
				printf("DISCOVERING\n");
            if (p_dev->dev_found)
				printf("DISC FOUND\n");



            if ( (p_dev->state == APP_GAP_STATE_DEVICE_DISCOVER_COMPLETE ||
                    p_dev->state == APP_GAP_STATE_DEVICE_DISCOVERING)
                    && p_dev->dev_found) {
                p_dev->state = APP_GAP_STATE_SERVICE_DISCOVERING;
				
// esp_bt_gap_get_remote_services() cant run due to lack of memory (see output of heap_caps_get_largest_free_block()).
//                ESP_LOGI(GAP_TAG, "Discover services ...");

//				printf("Free: %d\n",heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

//				esp_bt_gap_get_remote_services(p_dev->bda);
            }
        } else if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STARTED) {
            ESP_LOGI(GAP_TAG, "Discovery started.");
        }
        break;
    }
    case ESP_BT_GAP_RMT_SRVCS_EVT: {
        if (memcmp(param->rmt_srvcs.bda, p_dev->bda, ESP_BD_ADDR_LEN) == 0 &&
                p_dev->state == APP_GAP_STATE_SERVICE_DISCOVERING) {
            p_dev->state = APP_GAP_STATE_SERVICE_DISCOVER_COMPLETE;
            if (param->rmt_srvcs.stat == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(GAP_TAG, "Services for device %s found",  bda2str(p_dev->bda, bda_str, 18));
                for (int i = 0; i < param->rmt_srvcs.num_uuids; i++) {
                    esp_bt_uuid_t *u = param->rmt_srvcs.uuid_list + i;
                    ESP_LOGI(GAP_TAG, "--%s", uuid2str(u, uuid_str, 37));
                }
            } else {
                ESP_LOGI(GAP_TAG, "Services for device %s not found",  bda2str(p_dev->bda, bda_str, 18));
            }
        }
        break;
    }
    case ESP_BT_GAP_RMT_SRVC_REC_EVT:
    default: {
        ESP_LOGI(GAP_TAG, "event: %d", event);
        break;
    }
    }
    return;
}

static char *bda2str(esp_bd_addr_t bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

static char *uuid2str(esp_bt_uuid_t *uuid, char *str, size_t size)
{
    if (uuid == NULL || str == NULL) {
        return NULL;
    }

    if (uuid->len == 2 && size >= 5) {
        sprintf(str, "%04x", uuid->uuid.uuid16);
    } else if (uuid->len == 4 && size >= 9) {
        sprintf(str, "%08x", uuid->uuid.uuid32);
    } else if (uuid->len == 16 && size >= 37) {
        uint8_t *p = uuid->uuid.uuid128;
        sprintf(str, "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
                p[15], p[14], p[13], p[12], p[11], p[10], p[9], p[8],
                p[7], p[6], p[5], p[4], p[3], p[2], p[1], p[0]);
    } else {
        return NULL;
    }

    return str;
}

static bool get_name_from_eir(uint8_t *eir, uint8_t *bdname, uint8_t *bdname_len)
{
    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;

    if (!eir) {
        return false;
    }

    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }

    if (rmt_bdname) 
		{
        if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) 
			{
            rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
			}

        if (bdname) 
			{
            memcpy(bdname, rmt_bdname, rmt_bdname_len);
            bdname[rmt_bdname_len] = '\0';
			}
        if (bdname_len) 
			{
            *bdname_len = rmt_bdname_len;
			}
        return true;
		}

    return false;
}

static void update_device_info(esp_bt_gap_cb_param_t *param)
{
    char bda_str[18];
    uint32_t cod = 0;
    int32_t rssi = -129; /* invalid value */
    uint8_t *bdname = NULL;
    uint8_t bdname_len = 0;
    uint8_t *eir = NULL;
    uint8_t eir_len = 0;
    esp_bt_gap_dev_prop_t *p;
	unsigned char found_flag;
	unsigned char name_found_flag = 0;
	
    uint8_t *mac_ptr = param->disc_res.bda;
	
	
	printf("***   GAP INFO   ***\n");

    ESP_LOGI(GAP_TAG, "Device found: %s [%d]", bda2str(param->disc_res.bda, bda_str, 18), param->disc_res.num_prop);
    for (int i = 0; i < param->disc_res.num_prop; i++) {
        p = param->disc_res.prop + i;
        switch (p->type) {
        case ESP_BT_GAP_DEV_PROP_COD:
            cod = *(uint32_t *)(p->val);
            ESP_LOGI(GAP_TAG, "--Class of Device: 0x%x", cod);
            break;
        case ESP_BT_GAP_DEV_PROP_RSSI:
            rssi = *(int8_t *)(p->val);
            ESP_LOGI(GAP_TAG, "--RSSI: %d", rssi);
            break;
        case ESP_BT_GAP_DEV_PROP_BDNAME:
            bdname_len = (p->len > ESP_BT_GAP_MAX_BDNAME_LEN) ? ESP_BT_GAP_MAX_BDNAME_LEN :
                          (uint8_t)p->len;
            bdname = (uint8_t *)(p->val);
			name_found_flag = 1;
            ESP_LOGI(GAP_TAG, "--Name: %s", bdname);
            break;
        case ESP_BT_GAP_DEV_PROP_EIR: {
            eir_len = p->len;
            eir = (uint8_t *)(p->val);
            ESP_LOGI(GAP_TAG, "--EIR:[%d]",eir_len);
#if 0
			for (i=0;i<eir_len;i++)
				{
				printf("%02X ",eir[i]);
				}
			printf("\n");
#endif
            break;
        }
        default:
            break;
        }
    }

    /* search for device with Major device type "PHONE" or "Audio/Video" in COD */
    app_gap_cb_t *p_dev = &m_dev_info;
//    if (p_dev->dev_found) {
//		printf("Ret!\n");
//        return;
//    }

    if (!esp_bt_gap_is_valid_cod(cod) ||
	    (!(esp_bt_gap_get_cod_major_dev(cod) == ESP_BT_COD_MAJOR_DEV_PHONE) &&
             !(esp_bt_gap_get_cod_major_dev(cod) == ESP_BT_COD_MAJOR_DEV_AV))) {
        printf("Inv COD!\n");
		return;
    }

    memcpy(p_dev->bda, param->disc_res.bda, ESP_BD_ADDR_LEN);
    p_dev->dev_found = true;

    p_dev->cod = cod;
    p_dev->rssi = rssi;

// name can come from Local Name descriptor or EIR data portion... 
	if (bdname_len > 0) 
		{
		printf("using bdname...\n");
        memcpy(p_dev->bdname, bdname, bdname_len);
        p_dev->bdname[bdname_len] = '\0';
        p_dev->bdname_len = bdname_len;
		}
		
    if (eir_len > 0) {
        memcpy(p_dev->eir, eir, eir_len);
        p_dev->eir_len = eir_len;
    }

// if name not in descriptor, use EIR info if available...
    if (p_dev->eir && bdname_len == 0) 
		{
        printf("getting EIR name...\n");
		get_name_from_eir(p_dev->eir, p_dev->bdname, &p_dev->bdname_len);
		name_found_flag = 1;
		}

	if (name_found_flag == 0)
		{
		strcpy((char *)p_dev->bdname,"NO NAME");
		}
		
    ESP_LOGI(GAP_TAG, "Found a target device:\n   MAC address %s\n   name %s", bda_str, p_dev->bdname);
// want continuous discovery...
//    p_dev->state = APP_GAP_STATE_DEVICE_DISCOVER_COMPLETE;
//    ESP_LOGI(GAP_TAG, "Cancel device discovery ...");
//   esp_bt_gap_cancel_discovery();


	found_flag = 0;

// add this MAC addr to the list of MAC addresses
	if (bluetooth_MAC_entries < BLUETOOTH_LIST_LEN)
		{
		unsigned int j;
		unsigned char *ptr;

//	printf("Checking Classic MAC:\n");
// check first that the MAC address isnt already on the list?					
		for (j=0;j<bluetooth_MAC_entries;j++)
			{
			if (!memcmp(param->disc_res.bda,bluetooth_MAC_list[j]->bluetooth_MAC_addr,6))
				{
//				printf("BT CLASSIC MAC already on MAC scan list!");
				if(debug_do(DBG_BLUETOOTH))
					{
					printf("BT CLASSIC MAC already on MAC scan list!");
					ptr = bluetooth_MAC_list[j]->bluetooth_MAC_addr;
//					printf("BT CLASSIC MAC already on MAC scan list! [%d][%02X:%02X,%02X:%02X,%02X:%02X][%d]\n",j,ptr[0],ptr[1],ptr[2],ptr[3],ptr[4],ptr[5],bluetooth_MAC_entries);
					printf(" [%d][%02X:%02X,%02X:%02X,%02X:%02X][%d]\n",j,ptr[0],ptr[1],ptr[2],ptr[3],ptr[4],ptr[5],bluetooth_MAC_entries);
					}
				else
					printf("\n");
				
				found_flag = 1;
				}
			}


//		if ((msg_with_local_name == ESP_OK) && (!found_flag[i]))						// if good msg with local name and no match found on the list...
		if (!found_flag)						// if good msg with local name and no match found on the list...
			{
			ptr = bluetooth_MAC_list[bluetooth_MAC_entries]->bluetooth_MAC_addr;
			
			memcpy(bluetooth_MAC_list[bluetooth_MAC_entries]->bluetooth_MAC_addr,param->disc_res.bda,6);	// add new MAC addr to the list
			bluetooth_MAC_list[bluetooth_MAC_entries]->bluetooth_rssi = p_dev->rssi;
			strcpy(bluetooth_MAC_list[bluetooth_MAC_entries]->bluetooth_devname,(char *)p_dev->bdname);
#define CLASSIC_DATA_TEST
#ifdef CLASSIC_DATA_TEST
							printf("CLASSIC LOCAL NAME: %s\n",bluetooth_MAC_list[bluetooth_MAC_entries]->bluetooth_devname);
							printf("CLASSIC MAC ADDR  : %02X:%02X:%02X:%02X:%02X:%02X\n",ptr[0],ptr[1],ptr[2],ptr[3],ptr[4],ptr[5]);
							printf("CLASSIC RSSI      : %02X\n",bluetooth_MAC_list[bluetooth_MAC_entries]->bluetooth_rssi);
#endif

			bluetooth_MAC_entries++;
			}
		}

}

static void bt_app_gap_init(void)
{
    app_gap_cb_t *p_dev = &m_dev_info;
    memset(p_dev, 0, sizeof(app_gap_cb_t));

    p_dev->state = APP_GAP_STATE_IDLE;
}

#endif
