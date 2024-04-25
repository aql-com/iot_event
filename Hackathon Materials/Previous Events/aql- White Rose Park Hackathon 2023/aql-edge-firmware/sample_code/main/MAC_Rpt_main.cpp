//////////////////////////////////////////////
//
// MAC_Rpt_main.cpp
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////


#include <stdio.h>
#ifdef __cplusplus
	#include <string>
#endif
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_event_loop.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_adc_cal.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "driver/i2c.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "soc/uart_struct.h"

#include "nvs_flash.h"
#include "sdkconfig.h"
#include "esp_partition.h"

#include "esp32/rom/ets_sys.h"	// ets usec delays

#include "esp_err.h"

#include "bt_hci_common.h"		// BlueTooth
#include "esp_bt.h"	// BlueTooth

#include <esp_http_server.h>



#include "MAC_Rpt.h"
#include "MAC_Rpt_rtns.h"
#include "MAC_Rpt_wifi.h"
#include "MAC_Rpt_four_g.h"
#include "BlueTooth.h"

#include "MAC_Rpt_uart_intr.h"

#ifdef USE_M4G_HTTP
#include "MAC_Rpt_four_g_http_defs.h"
#include "MAC_Rpt_four_g_http.h"
#endif

#ifdef USE_M4G_MQTT
//#include "MAC_Rpt_four_g_mqtt_defs.h"
#include "MAC_Rpt_four_g_mqtt.h"

#include "MAC_Rpt_four_g_simcom.h"

#endif

//#define USE_M4G_UDP

//#define USE_WIFI_UDP

#ifdef USE_M4G_UDP
#include "MAC_Rpt_four_g_udp.h"
#endif

#ifdef USE_WIFI_UDP
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "MAC_Rpt_wifi_udp.h"

#endif

//#define USE_WIFI_MQTT	// now in MAC_Rpt.h!
#ifdef USE_WIFI_MQTT
#include "mqtt_client.h"
#include "MAC_Rpt_wifi_mqtt.h"
#warning "Using wifi MQTT!"
#endif

//#include "MAC_Rpt_temp.h"

#ifdef USE_WEBSERVER
#include "MAC_Rpt_http_server.h"
#endif


#if defined(USE_LORA) || defined(USE_LORA_LINK)
#include "MAC_Rpt_SX1276_defs.h"
#include "MAC_Rpt_SX1276.h"
#include "MAC_Rpt_LoRa.h"
#endif

#define GET_PARTITION_TABLE
#ifdef GET_PARTITION_TABLE
#include "esp_partition.h"
#endif

#ifdef USE_ZULU_RADIO
#include "MAC_Rpt_ZULU_Radio.h"
#include "MAC_Rpt_ZULU_Radio.cpp"

//#include "driver/ledc.h"
#endif


#ifdef USE_LORA_LINK
unsigned char lora_tx_data[80];
unsigned char lora_tx_data_len;
unsigned char lora_tx_timer;	//, lora_tx_timer_flag;
unsigned char lora_tx_flag;
unsigned char lora_rx_data[80];
unsigned char lora_rx_data_len;
unsigned char lora_rx_flag;
#endif



unsigned char x100msec, secs, mins, hrs;
unsigned char days, month, year;
unsigned char x100msec_flag, x1sec_flag, x10sec_flag;
unsigned char day_of_week;
unsigned char tilt_sw, fill_sw, purge_sw;

unsigned char u100msec, u1sec, u1min, u1hr;
unsigned int u1day;

unsigned int num_dispense_outputs;
uint8_t mac_addr[6];
char mac_str[20];
char mac_bin_str[20];
char module_mac_str[20];
char tmp_str[80];

char mac_ID_str[16];

unsigned char cmd_string[DBG_RCV_BUF_SIZE];

//unsigned char dispense_timer_end_flag;
unsigned char delay_timer_active_flag,prev_delay_timer_active_flag;

//unsigned char tank_countdown;

unsigned char status_array[NUM_DEVS];
//unsigned char tank_status;

unsigned long int dbgbits;
unsigned char dbgflag;

//unsigned long pump_motor_ave_current;

unsigned char wifi_scan_valid;

unsigned int adc_val_raw_sv;	

unsigned int supply_voltage;
uint32_t voltage;
unsigned char temperature;


uint8_t four_g_tx_data[FOUR_G_TX_BUFSIZE];
uint8_t four_g_rx_data[FOUR_G_RX_BUFSIZE];
unsigned int four_g_tx_length = 0;
unsigned int four_g_rx_length = 0;

unsigned int num_MAC_addrs;
unsigned char wifi_scan_flag, wifi_check_flag;

unsigned int uart_stay_awake_time;
unsigned int uart_stay_awake_count;

unsigned char four_g_rst_flag, four_g_pwr_flag, four_g_psu_en_flag, four_g_connected_flag;
unsigned char lora_radio_rst_flag, lora_radio_connected_flag;
signed char lora_rssi, lora_pkt_rssi, lora_snr;

unsigned char simcom_disable_flag;
unsigned char mqtt_disable_flag;
unsigned char udp_disable_flag;
unsigned char wifi_disable_flag;
unsigned char wifi_mqtt_disable_flag;

unsigned int dbg_count;

//#ifdef V1_PCB
#if PCB_VER == VER_1_0_C
unsigned char devpin[NUM_DEVS] =
// yields device GPIO pin based on device name from devs enum in MAC_Rpt.h
// names here and pins in enum devs must be in same order!
{
SDA,		// LORA RTS
SCL,		// LORA CTS
RTS2,		// 4G RTS
CTS2,		// 4G CTS
DTR2,		// 4G DTR
DCD2,		// 4G DCD
LED_0,
LED_1,
LED_2,
LED_3,
LORA_RADIO_RESET,
FOUR_G_RESET,
FOUR_G_POWER
};
#endif

#if PCB_VER == VER_1_0_D
unsigned char devpin[NUM_DEVS] =
// yields device GPIO pin based on device name from devs enum in MAC_Rpt.h
// names here and pins in enum devs must be in same order!
{
//SDA,		// LORA RTS		// NOT USED!
//SCL,		// LORA CTS		// NOT USED!
RTS2,		// 4G RTS
CTS2,		// 4G CTS
DTR2,		// 4G DTR
DCD2		// 4G DCD
/*
// following are on shift register outputs!
FOUR_G_RESET,
FOUR_G_PSU_EN,
FOUR_G_POWER,
LORA_RADIO_RESET,
LED_0,
LED_1,
LED_2,
LED_3
*/
};

/*
LED_0,
LED_1,
LED_2,
LED_3,
LORA_RADIO_RESET,
FOUR_G_RESET,
FOUR_G_POWER
};
FOUR_G_RESET,
FOUR_G_PSU_EN,
FOUR_G_POWER,
LORA_RADIO_RESET,
LED_0,
LED_1,
LED_2,
LED_3,
*/
#endif


#if (PCB_VER == VER_BTS_1_0_C) || (PCB_VER == VER_BTS_1_0_D)
unsigned char devpin[NUM_DEVS] =
// yields device GPIO pin based on device name from devs enum in MAC_Rpt.h
// names here and pins in enum devs must be in same order!
{
//SDA,		// LORA RTS		// NOT USED!
//SCL,		// LORA CTS		// NOT USED!
RTS2,		// 4G RTS
CTS2,		// 4G CTS
DTR2,		// 4G DTR
DCD2		// 4G DCD
/*
// following are on shift register outputs!
FOUR_G_RESET,
FOUR_G_PSU_EN,
FOUR_G_POWER,
LORA_RADIO_RESET,
LED_0,
LED_1,
LED_2,
LED_3
*/
};
#endif

char progress_indicator[4] = 
{
'-',
'/',
'-',
'\\'
};

wifi_scan_config_t wifi_scan_cfg;

//struct wifi_ap_record_t;
//wifi_ap_record_t wifi_list_A[WIFI_LIST_LEN];
//wifi_ap_record_t wifi_list_B[WIFI_LIST_LEN];

// changed to calloc memory, to allow access to extra DRAM space which can only be malloc'ed or calloc'ed
// see start io app_main...
#ifdef WIFI_SCAN
wifi_ap_record_t *wifi_list_A;
wifi_ap_record_t *wifi_list_B;

wifi_ap_record_t* wifi_list_current;
wifi_ap_record_t* wifi_list_previous;
wifi_ap_record_t* wifi_list_tmp;

unsigned int wifi_current_num, wifi_previous_num;
unsigned int wifi_dev_count, wifi_total_dev_count;
unsigned char wf_found, wf_rssi_too_low, wf_private;

signed char hp1_wifi_rssi, hp2_wifi_rssi;
uint8_t hp1_mac_addr[6], hp2_mac_addr[6];
unsigned int hp1_mac_index, hp2_mac_index;	
#endif

unsigned char wifi_scan_enable_flag;
unsigned char wifi_is_on;

unsigned char ntptime_sync_flag;
unsigned int TIM_msg_time;


#ifdef BLUETOOTH_SCAN
unsigned int bluetooth_MAC_entries;
struct BT_MAC_INFO* bluetooth_MAC_list[BLUETOOTH_LIST_LEN];
//{
//uint64_t* bluetooth_MAC_addr;		// Bluetooth MAC addr is 6 bytes
//signed char bluetoth_rssi;
//}

#endif

//struct BT_DEVNAME_INFO* bluetooth_devname_list[BLUETOOTH_DEVNAME_LIST_LEN];

struct BT_DEVNAME_INFO bluetooth_devname_list[BLUETOOTH_DEVNAME_LIST_LEN];


unsigned int bluetooth_devname_entries;

unsigned char bt_i2c_update_timer;

unsigned char bt_sensor_data[16];
unsigned char bt_sensor_data_len;

#ifdef LORAWAN_SCAN
unsigned int lorawan_MAC_entries;
uint32_t* lorawan_MAC_list;		// LoRaWAN devaddr is 4 bytes
#endif

unsigned char tm_len;
unsigned char tm_data[15];

unsigned char sw_status, prev_sw_status;
unsigned int up_time;


volatile SemaphoreHandle_t xSemaphore = NULL;
volatile unsigned char idata, inopulse;

int64_t pulse_start_time, pulse_end_time, pulse_time;

char gps_lat_str[15];
char gps_lon_str[15];
unsigned int gps_spd;
unsigned long lora_radio_freq;

//#ifdef V2_PCB
#if PCB_VER == VER_1_0_D
unsigned char sd_data_in, sd_data_out;
#endif


#if (PCB_VER == VER_BTS_1_0_C) || (PCB_VER == VER_BTS_1_0_D)
unsigned char sd_data_in, sd_data_out;
#endif

unsigned char led_states, prev_led_states;
unsigned char led_modes[4];

char vtaskList[600];

unsigned char four_g_state, prev_four_g_state;
unsigned int four_g_timeout_timer;
unsigned int four_g_timeout_val;
unsigned char four_g_response_flag, four_g_eol_response_flag;
unsigned char simcom_response_flag, simcom_eol_response_flag;
unsigned char mqtt_response_flag, mqtt_eol_response_flag;
unsigned char udp_response_flag, udp_eol_response_flag;
unsigned char udp_test_flag;

/*
unsigned int four_g_state_mc_timer;
unsigned char four_g_sm_timer_flag;
*/
unsigned char simcom_busy_flag, prev_simcom_busy_flag;
unsigned int simcom_state_mc_timer;		// simcom power up \ power down timer
unsigned char simcom_sm_timer_flag;
unsigned int simcom_timeout_timer;
unsigned int simcom_timeout_val;

unsigned int mqtt_timeout_timer;
unsigned int mqtt_timeout_val;

unsigned int udp_timeout_timer;
unsigned int udp_timeout_val;

/*
unsigned int mqtt_state_mc_timer;
unsigned char mqtt_sm_timer_flag;
unsigned int udp_state_mc_timer;
unsigned char udp_sm_timer_flag;
*/

unsigned char simcom_comms_flag;
unsigned char simcom_powerdown_attempts;

unsigned char mqtt_preferred_transport_mode;

unsigned char mqtt_transport_mode;
unsigned char wifi_sta_connected;
unsigned char wifi_mqtt_connected;
unsigned char wifi_mqtt_started;
unsigned char wifi_mqtt_attempts;

unsigned char wifi_sta_auth,wifi_ap_auth;

unsigned char udp_preferred_transport_mode;
unsigned char udp_transport_mode;

char wifi_sta_SSID[WIFI_CREDENTIALS_MAX][32];
char wifi_sta_PWD[WIFI_CREDENTIALS_MAX][32];
unsigned char wifi_sta_credentials_index;

char modtype_str[32];
char fwver_str[32];
char imei_str[20];
char simnum_str[22];
char ipaddr_str[16];

unsigned char simcom_state, prev_simcom_state;

unsigned char mqtt_state, prev_mqtt_state;
unsigned char udp_state, prev_udp_state;
unsigned char wifi_mqtt_state, prev_wifi_mqtt_state;

#ifdef USE_WIFI_MQTT
esp_mqtt_client_handle_t mqtt_client;
#endif

#ifdef USE_M4G_MQTT
unsigned int mqtt_sensor_msg_timer;
unsigned char mqtt_login_state, prev_mqtt_login_state, mqtt_sensor_msg_flag;
extern char mqtt_will_topic_str[];	//80];
//extern unsigned int mqtt_payload_length;

extern char mqtt_topic_str[];	//80];
extern char mqtt_payload_str[];	//80];
extern unsigned int mqtt_payload_length;

QueueHandle_t mqtt_cmd_queue;
QueueHandle_t mqtt_data_queue;
unsigned char CMQTT_rx_flag;
unsigned char CMQTT_rx_msg;
unsigned int rcv_timeout_timer;
char server_cmd_test_str[30];
unsigned char server_cmd_test_flag;
#endif


#if defined(USE_M4G_UDP) || defined(USE_WIFI_UDP)
extern char udp_payload_str[];	//80];
extern unsigned int udp_payload_length;

QueueHandle_t udp_cmd_queue;
QueueHandle_t udp_data_queue;
#endif

#ifdef USE_WIFI_UDP
char wifi_udp_url[80];	// = "mqtt.core.aql.com";

unsigned int wifi_udp_port;	// = 8883;
ip_addr_t wifi_udp_ip_addr;

ip4_addr_t ip;
ip4_addr_t gw;
ip4_addr_t msk;

bool wifi_udp_connected;	// = false;
bool wifi_udp_DNSfound, prev_wifi_udp_DNSfound;	// = false;
bool wifi_udp_DNSstart;
unsigned char wifi_udp_DNS_timer;

int wifi_udp_sock;
struct sockaddr_in dest_addr;
int wifi_udp_addr_family;
int wifi_udp_ip_protocol;

char wifi_udp_rx_buffer[128];
#endif

#if defined(USE_M4G_MQTT) || defined(USE_M4G_UDP)
char server_addr_and_port [4][ADDR_AND_PORT_LEN] = 
{
SERVER_ADDRESS_AND_PORT_0,
SERVER_ADDRESS_AND_PORT_1,
SERVER_ADDRESS_AND_PORT_2,
SERVER_ADDRESS_AND_PORT_3
};

unsigned char server_ssl_mode [4] = 
{
SERVER_SSL_MODE_0,
SERVER_SSL_MODE_1,
SERVER_SSL_MODE_2,
SERVER_SSL_MODE_3
};

unsigned char server0_addr_ptr, server1_addr_ptr;
// unsigned char server0_ssl_mode, server1_ssl_mode;

#endif

char response_str[100];
char response_flag;
char server_response_flag;

#ifdef USE_BLUETOOTH
unsigned char continue_ble_commands;
uint16_t scanned_count = 0;
unsigned int bt_scan_time;
//static QueueHandle_t adv_queue;

extern unsigned char bt_buffer[BT_BUF_SIZE];

extern QueueHandle_t out_queue;
#endif

unsigned char bt_msg_ready_count;

signed char cell_rssi;

unsigned int i2c_temp,i2c_humid,i2c_light,i2c_audio,i2c_batt,i2c_co2,i2c_voc,i2c_pressure,i2c_proximity;

QueueHandle_t four_g_uart_queue;
unsigned char four_g_msg_rx_count;

unsigned char tank_sensors_present_flag;
unsigned char tank_sensor_0, tank_sensor_1, tank_sensor_percent_0, tank_sensor_percent_1;

unsigned char zulu_radio_run_flag;
#ifdef USE_ZULU_RADIO
QueueHandle_t zulu_radio_uart_queue;
uint8_t zulu_radio_tx_data[ZULU_RADIO_TX_BUFSIZE];
uint8_t zulu_radio_rx_data[ZULU_RADIO_RX_BUFSIZE];
unsigned int zulu_radio_status_timer;
unsigned int zulu_radio_hb_timer;
unsigned int zulu_radio_timeout_timer;
unsigned char zulu_radio_tx_seq_num,zulu_radio_rx_seq_num, zulu_radio_slave_status;

unsigned char zulu_radio_get_timer_flag,zulu_radio_set_timer_flag,zulu_radio_msgs_flag;

unsigned int zulu_radio_run_timer;
unsigned char zulu_radio_response_received, zulu_radio_rcv_count;
unsigned char print_end_msg_flag;

#endif


//#endif
unsigned char i2c_sensor_count;
unsigned char ambi_done_flag,ambi_seq_num;
unsigned char bme280_done_flag,num,bme280_seq_num;
unsigned char bme688_done_flag,bme688_seq_num;
unsigned char ltr559_done_flag,ltr559_seq_num;

unsigned char lsm303d_done_flag,lsm303d_seq_num;
unsigned char lsm303d_section;


unsigned char tx_bluetooth_whitelist_flag;
unsigned char tx_bluetooth_blacklist_flag;
unsigned char tx_bluetooth_devname_whitelist_flag;
unsigned char enable_bluetooth_whitelist_flag;				// Bluetooth whitelist enable flag
unsigned char enable_bluetooth_blacklist_flag;				// Bluetooth blacklist enable flag
unsigned char enable_bluetooth_devname_whitelist_flag;		// Bluetooth device name whitelist enable flag
unsigned char bluetooth_whitelist_mode;
unsigned char bluetooth_blacklist_mode;
unsigned char bluetooth_beacon_whitelist_mode;
unsigned int num_bt_wl_entries;
unsigned int num_bt_bl_entries;
unsigned int num_bt_devname_wl_entries;

unsigned char tx_lorawan_whitelist_flag;
unsigned char tx_lorawan_blacklist_flag;
unsigned char enable_lorawan_whitelist_flag;				// LoRaWAN whitelist enable flag
unsigned char enable_lorawan_blacklist_flag;				// LoRaWAN blacklist enable flag
unsigned char lorawan_whitelist_mode;
unsigned char lorawan_blacklist_mode;
unsigned int num_lora_wl_entries;
unsigned int num_lora_bl_entries;

char list_empty_str[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,};

unsigned char tx_serial_settings_flag, serial_autobaud_flag;
unsigned long serial_bitrate;
char eolstr[10];
unsigned char eol_remove_flag;
int32_t auto_serial_bitrate;

unsigned char binary_data_flag, auto_binary_data_flag, binary_mode_flag;
unsigned char bt_advertising_ready_flag;

int64_t pcmqtt_time;
int64_t pcudp_time;

unsigned char ssl_enable_flag;

char pattern_detect_char, prev_pattern_detect_char;;

#if defined(USE_LORA) || defined(USE_LORA_LINK)
spi_device_handle_t spi;
unsigned char lora_reset_timer;
unsigned char lora_reset_complete_flag;
unsigned char lora_tx_packet_open;

QueueHandle_t lora_data_queue;
QueueHandle_t lora_gpio_queue;

host_rcv_data_t *lora_rx_data_wr;		// global so can be accessed from interrupt rtn...

unsigned char LORA_int_flag;
unsigned char prev_dio_val;

unsigned char SX1276_dio1_map,SX1276_dio2_map;

unsigned int lora_packet_count, lora_prev_packet_count;
#endif

uint64_t bit_time_start;
uint32_t ext_serial_bit_time, prev_ext_serial_bit_time;
unsigned char bit_time_counter, ext_serial_bit_time_change_flag;


#ifdef USE_ULTRASONIC_SENSOR
unsigned int usm_trigger_count;
unsigned int usm_timeout_count;
unsigned char usm_trigger_flag;
unsigned char usm_complete_flag;
unsigned char usm_state, prev_usm_state;
unsigned char usm_int_count;
uint64_t usm_echo_time;
unsigned int usm_distance, usm_ave_distance, usm_accum, usm_max,usm_ave_max;
unsigned char usm_ave_count, usm_completed_ok_flag, usm_cabin_empty_flag;
uint64_t usm_start_time;
uint64_t usm_end_time;

#endif


#ifdef LORA_USED
static void IRAM_ATTR lora_dio0_isr_handler(void* arg);
static void IRAM_ATTR lora_dio1_isr_handler(void* arg);
static void IRAM_ATTR lora_dio2_isr_handler(void* arg);
static void IRAM_ATTR lora_dio3_isr_handler(void* arg);
static void IRAM_ATTR lora_dio4_isr_handler(void* arg);
//static void IRAM_ATTR lora_dio5_isr_handler(void* arg);

void lora_rcv(void);
static void lora_rcv_int(void);
#endif

#ifdef USE_ULTRASONIC_SENSOR
static void IRAM_ATTR usm_isr_handler(void* arg);
#endif

static void IRAM_ATTR ext_serial_bitrate_isr_handler(void* arg);

unsigned char lora_msg_ready_count;

unsigned int gateway_sensor_timer;
unsigned int gps_sensor_timer;
unsigned int i2c_sensor_timer;
unsigned int tank_sensor_timer;
unsigned int bluetooth_sensor_timer;
unsigned int bluetooth_MAC_addr_timer;
unsigned int bluetooth_devname_timer;
unsigned int bluetooth_classic_scan_timer;
unsigned int lora_devaddr_timer;
unsigned int wifi_scan_timer;

unsigned int gateway_sensor_time;
unsigned int gps_sensor_time;
unsigned int i2c_sensor_time;
unsigned int tank_sensor_time;
unsigned int bluetooth_sensor_time;
unsigned int bluetooth_MAC_addr_time;
unsigned int bluetooth_devname_time;
unsigned int bluetooth_classic_scan_time;
unsigned int lora_devaddr_time;
unsigned int wifi_scan_time;
unsigned int bluetooth_sensor_adv_time;
char bluetooth_sensor_adv_name[15];


unsigned char rl1_flag, rl2_flag;

unsigned char event_msg_flag;


#ifdef ZULU_RADIO_SLAVE
#endif


unsigned char evt_num, evt_enable_flag,evt_mode_flag,evt_day,evt_t1_hrs,evt_t1_mins,evt_t2_hrs,evt_t2_mins,evt_pwm;
unsigned int evt_time[NUM_EVENTS_MAX];
unsigned int evt_timer[NUM_EVENTS_MAX];
unsigned int evt_repeat_timer[NUM_EVENTS_MAX];

unsigned char evt_curr_num;
unsigned char evt_curr_mode_flag;
unsigned char evt_start_stop_flag;
unsigned char evt_curr_PWM;
unsigned char evt_curr_error_byte;
QueueHandle_t evt_notification_queue;
unsigned char evt_notifications;

struct EVENT_INFO event_list[NUM_EVENTS_MAX];

unsigned char get_server_time_flag;
unsigned int get_server_time_timer;
unsigned char test_server_time_flag;
unsigned int srvr_test_run_flag,srvr_test_count,srvr_test_OK_count;

unsigned char srvr_return_cmd,srvr_return_cmd_arg;
unsigned char srvr_return_data_type;
char srvr_return_str[30];
unsigned int srvr_return_value;
unsigned char srvr_return_flag;

//#ifdef NEW_MQTT_SERIAL
// need to be globals, to preserve across fg loop!		
//	char* dtmp;
//	char* dleft;
 //   size_t dleft_size;
  //  size_t dleft_pos;
//    size_t dprev_size;
//	unsigned char end_flag;			// ??
//	unsigned char endflag;		// ??
//	unsigned char mqtt_step;		
//#endif

httpd_handle_t http_server = NULL;


static const char* TAG = "MAC_Rpt";



unsigned char gpiox;


#ifdef ADCS_USED
unsigned char adc_0, adc_1;
#endif

unsigned char queues_enabled_flag;
unsigned char net_test_flag;

//////////////////////////////////////////////
// function prototypes
//////////////////////////////////////////////

//static void periodic_IR_timer_callback(void* arg);
static void periodic_100mseconds_timer_callback(void* arg);

static void oneshot_timer_callback(void* arg);

//static void IRAM_ATTR gpio_isr_handler(void* arg);

void spi_pre_transfer_callback(spi_transaction_t *t);


#if defined(USE_LORA) || defined(USE_LORA_LINK)
void print_lora_msg(char *name, unsigned char *data,unsigned char len);
#endif

void mqtt_send(unsigned char mqtt_transport_mode, unsigned char next_login_state);

// define main as extern "C"...
extern "C"
{
void app_main(void);
}

//////////////////////////////////////////////
// main function
//////////////////////////////////////////////
void app_main()
{
//unsigned int wifi_scan_timer;

unsigned char c,i,j;
signed char s;
unsigned int t;

char uart_data[DBG_RCV_BUF_SIZE];
unsigned char uart_len;
unsigned char msg_len;

unsigned char cmd_flag;
char str[DBG_RCV_BUF_SIZE];

unsigned char ch;

int adc_val;	

unsigned long x;

unsigned char four_g_count;

unsigned int TIM_msg_time_count;

unsigned int wfi, wfj;
unsigned char lora_radio_msg_type;
//unsigned char lora_radio_error_byte;
unsigned char lora_radio_send_msg_flag;

unsigned char MAC_seq_number;

uint8_t geo_mac_addr[4][6];
signed char geo_rssi[2];

int64_t start_time, end_time, loop_time;

unsigned char sleep_flag, sleep_resume_flag;
unsigned char sleep_cycle_count;

unsigned char led_count;
//unsigned char binary_data_flag, auto_binary_data_flag;	// WILL BE PICKED UP BY THE SERIAL PAYLOAD ROUTINES...


#ifdef WIFI_SCAN
// dont calloc this memory if no wifi scan required. Uses 1024 * sizeof(wifi_ap_record_t)...!
// changed to calloc memory, to allow access to extra DRAM space which can only be malloc'ed or calloc'ed
wifi_list_A = (wifi_ap_record_t*)calloc( sizeof(wifi_ap_record_t),WIFI_LIST_LEN);
wifi_list_B = (wifi_ap_record_t*)calloc( sizeof(wifi_ap_record_t),WIFI_LIST_LEN);
#endif

#ifdef BLUETOOTH_SCAN
bluetooth_MAC_entries = 0;
//bluetooth_MAC_list = (uint64_t*)calloc(sizeof(uint64_t),BLUETOOTH_LIST_LEN);		// Bluetooth MAC addr is 6 bytes
for (unsigned int x=0;x<BLUETOOTH_LIST_LEN;x++)
	{
	bluetooth_MAC_list[x] = (BT_MAC_INFO*)calloc(sizeof(BT_MAC_INFO),1);		// Bluetooth MAC addr is 6 bytes
	}

if (debug_do(DBG_ALLOC_MEM))
	printf("BT mac list alloc %d\n",sizeof(BT_MAC_INFO) * BLUETOOTH_LIST_LEN);
#endif



#ifdef LORAWAN_SCAN
lorawan_MAC_entries = 0;
lorawan_MAC_list = (uint32_t*)calloc(sizeof(int32_t),LORAWAN_LIST_LEN);		// LoRaWAN devaddr is 4 bytes

if (debug_do(DBG_ALLOC_MEM))
	printf("LoRaWAN scan MAC list alloc %d\n",LORAWAN_LIST_LEN);
#endif

/*
#ifdef V1_PCB
printf("Ver 1 PCB\r\n");
#endif
#ifdef V2_PCB
printf("Ver 2 PCB\r\n");
#endif
*/

//dbgbits = 0;
//dbgflag = 0;

#if 0	// activate debug at power-on
dbgbits = DBG_4G_TXRX;	//dbgbits | (1<<DBG_MQTT);
//dbgbits = dbgbits + DBG_MQTT;	//dbgbits | (1<<DBG_MQTT);
dbgbits = dbgbits + DBG_UDP;	//dbgbits | (1<<DBG_MQTT);

dbgflag = 1;
printf("dbgbits: %08lX    dbgflag: %02X\n",dbgbits,dbgflag);
#endif


//printf("%s PCB\r\n",PCB_VER_STR);

//unsigned char pump_test_count, pump_test_flag;

// create one-shot timer
	esp_timer_handle_t oneshot_timer;

	const esp_timer_create_args_t oneshot_timer_args = {
            .callback = &oneshot_timer_callback,
            /* argument specified here will be passed to timer callback function */
            .arg = NULL,	//(void*) periodic_timer,
            .name = "one-shot"
		};

    ESP_ERROR_CHECK(esp_timer_create(&oneshot_timer_args, &oneshot_timer));
// timer created and ready for one-shot operation	
//	ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, 100000));	

// create periodic timer
// hh:mm:ss seconds timer
    esp_timer_handle_t periodic_100mseconds_timer;

	const esp_timer_create_args_t periodic_100mseconds_timer_args = 
		{
        .callback = &periodic_100mseconds_timer_callback,
        /* name is optional, but may help identify the timer when debugging */
		.arg = (void *)oneshot_timer,
        .name = "100mseconds periodic"
		};

//    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    esp_timer_create(&periodic_100mseconds_timer_args, &periodic_100mseconds_timer);
    /* The timer has been created but is not running yet */
	ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_100mseconds_timer, 100000));	


wifi_scan_cfg.ssid = 0;
wifi_scan_cfg.bssid = 0;
wifi_scan_cfg.channel = 0;	// 0 = all channel scan 
wifi_scan_cfg.show_hidden = 1;
wifi_scan_cfg.scan_type = WIFI_SCAN_TYPE_ACTIVE;
#ifdef USE_BLUETOOTH
wifi_scan_cfg.scan_time.active.min = 0;		// 0 = DFLT for wifi + BLE
wifi_scan_cfg.scan_time.active.max = 0;		// 0 = DFLT for wifi + BLE
#else
wifi_scan_cfg.scan_time.active.min = 120;		// 0 = DFLT for wifi + BLE
wifi_scan_cfg.scan_time.active.max = 150;		// 0 = DFLT for wifi + BLE
#endif
four_g_pwr_flag = 0;
four_g_rst_flag = 0;
four_g_psu_en_flag = 0;
four_g_connected_flag = 0;

lora_radio_rst_flag = 0;
lora_radio_connected_flag = 0;

lora_rssi = 0xFF;
lora_pkt_rssi = 0xFF;
lora_snr = 0xFF;


//four_g_hold_in_reset();
//four_g_power_key_off();
//lora_radio_hold_in_reset();

four_g_release_from_reset();		// 4G reset is onyl for emergencies when the power on \ off does not work...

//////////////////////////////////////////////
// SET GPIOs
//////////////////////////////////////////////
//make GPIO 32 and 33 pins available as GPIO
//REG_CLR_BIT(RTC_IO_XTAL_32K_PAD_REG, RTC_IO_X32P_MUX_SEL); /* gpio32 route to digital io_mux */
//REG_CLR_BIT(RTC_IO_XTAL_32K_PAD_REG, RTC_IO_X32N_MUX_SEL); /* gpio33 route to digital io_mux */

//    gpio_set_pull_mode(BLINK_GPIO, GPIO_PULLUP_ONLY); // CMD, needed in 4- and 1-line modes

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

#if 1
////////////////////////////////////
//
// use common GPIO initialisation
//
////////////////////////////////////
// common gpio initialisation
	sd_data_out = 0;

	shift_out(sd_data_out);

	led_states = 0;
	prev_led_states = 0;
	
	led_modes[0] = 0;
	led_modes[1] = 0;
	led_modes[2] = 0;
	led_modes[3] = 0;
	
// silicon bug on GPIO32 \ tank sensor 3:
//	rtc_gpio_pullup_en(TANK_SENSOR_3);



//#endif		// end of "use PCB version GPIO initialisation..."


	sd_data_out = 0;
	shift_out(sd_data_out);

	gpio_init(GPIO_SET);

	
	event_msg_flag = EVENT_NO_MSG;



#ifdef LORA_USED
#define ESP_INTR_FLAG_DEFAULT 0
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);

// multiple edge triggered ints on port groups (0-31) or (32-39) may not work (ESP32 Errata)
#if 1
	gpio_isr_handler_add(LORA_DIO0,lora_dio0_isr_handler, (void*) LORA_DIO0);
	gpio_set_intr_type(LORA_DIO0,GPIO_INTR_ANYEDGE);	//POSEDGE);
	gpio_intr_enable(LORA_DIO0);
#endif
#if 1
	gpio_isr_handler_add(LORA_DIO1,lora_dio1_isr_handler, (void*) LORA_DIO1);
	gpio_set_intr_type(LORA_DIO1,GPIO_INTR_ANYEDGE);
	gpio_intr_enable(LORA_DIO1);
#endif

#if 1
	gpio_isr_handler_add(LORA_DIO2,lora_dio2_isr_handler, (void*) LORA_DIO2);
	gpio_set_intr_type(LORA_DIO2,GPIO_INTR_ANYEDGE);
	gpio_intr_enable(LORA_DIO2);
#endif

#if 1
	gpio_isr_handler_add(LORA_DIO3,lora_dio3_isr_handler, (void*) LORA_DIO3);
	gpio_set_intr_type(LORA_DIO3,GPIO_INTR_ANYEDGE);
	gpio_intr_enable(LORA_DIO3);
#endif

#if 0
	gpio_isr_handler_add(LORA_DIO4,lora_dio4_isr_handler, (void*) LORA_DIO4);
	gpio_set_intr_type(LORA_DIO4,GPIO_INTR_ANYEDGE);
	gpio_intr_enable(LORA_DIO4);
#endif

//	gpio_isr_handler_add(LORA_DIO5,lora_dio5_isr_handler, (void*) LORA_DIO5);
//	gpio_set_intr_type(LORA_DIO5,GPIO_INTR_POSEDGE);
//	gpio_intr_enable(LORA_DIO5);
#endif	// end of "#ifdef USE_LORA"

#ifdef USE_ULTRASONIC_SENSOR
#define ESP_INTR_FLAG_DEFAULT 0
	printf("Setting up USM interrupt...\n");
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	gpio_isr_handler_add(USM_ECHO,usm_isr_handler, (void*) USM_ECHO);
	gpio_set_intr_type(USM_ECHO,GPIO_INTR_ANYEDGE);
//	gpio_intr_enable(USM_ECHO);

#endif

// end of common gpio initialisation



#endif		// end of "NEW init

//#endif		// end of "use PCB version GPIO initialisation..."





////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////



#ifdef ZULU_RADIO_MASTER	
	gpio_set_level(ZULU_RADIO_MASTER_RTS,0);		// 
#endif


#ifdef ZULU_RADIO_SLAVE
	gpio_set_level(ZULU_RADIO_SLAVE_RTS,0);		// 
#endif

// set up special functions (ADC, etc)

//	lora_radio_hold_in_reset();
	
//////////////////////////////////////////////
// SET UP UARTs
//////////////////////////////////////////////

// DEBUG MODE DETECT:
// CAN WE set RXD2 pull-down: look at RXD2 at boot; if LOW set a flag for "no output" of status msg?
// Then reset pull-down...
// Any advantage to doing this for RXD0 too?

// Configure UART parameters
///////////////////
// DBG UART
///////////////////
	uart_config_t dbg_uart_config = 
		{
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.rx_flow_ctrl_thresh = 110,
//		.source_clk = UART_SCLK_REF_TICK		// more modern version of use_ref_tick...
		.use_ref_tick = 1						// used for sleep \ wake from UART input...
		};
//ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));	
	uart_param_config(DBG, &dbg_uart_config);	

#ifdef V1_PCB_XXX
	uart_set_pin(DBG, TXD0, RXD0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE );

// Install UART driver using an event queue here
	const int debug_uart_buffer_size = (1024 * 2);
	QueueHandle_t debug_uart_queue;
//ESP_ERROR_CHECK(uart_driver_install(UART2, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
	uart_driver_install(DBG, debug_uart_buffer_size, debug_uart_buffer_size, 10, &debug_uart_queue, 0);
//	uart_driver_install(DBG, debug_uart_buffer_size, 0, 0, NULL, 0);
#endif



#ifdef USE_M4G_MQTT
///////////////////
// 4G UART
///////////////////
	printf("Installing UART 2 for MQTT...\n");

	uart_config_t four_g_uart_config = 
		{
		.baud_rate = 115200,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,	//UART_HW_FLOWCTRL_DISABLE,
		.rx_flow_ctrl_thresh = 110
		};
//ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));	
	uart_param_config(FOUR_G, &four_g_uart_config);	

// Set UART pins(TX: IO16 (UART2 default), RX: IO17 (UART2 default), RTS: IO18, CTS: IO19)
//ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, 18, 19));

#ifdef RTS2
	uart_set_pin(FOUR_G, TXD2, RXD2, RTS2, CTS2 );
#else
	uart_set_pin(FOUR_G, TXD2, RXD2, UART_PIN_NO_CHANGE, CTS2 );
	set_output(DEV_FOUR_G_RTS,DEVICE_ON,1);
#endif


// Install UART driver using an event queue here
	const int four_g_uart_buffer_size = (1024 * 2);
//ESP_ERROR_CHECK(uart_driver_install(UART2, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
	uart_driver_install(FOUR_G, four_g_uart_buffer_size, four_g_uart_buffer_size, 10, &four_g_uart_queue, 0);

	uart_hw_flowcontrol_t fctrl;

	uart_get_hw_flow_ctrl(FOUR_G,&fctrl);
	printf("OLD SIMCOM UART FCTL: %02X\n",fctrl);

	fctrl = UART_HW_FLOWCTRL_CTS_RTS;
	uart_set_hw_flow_ctrl(FOUR_G,fctrl,UART_FIFO_LEN - 100);	// UART_HW_FIFO_LEN is new version - U_F_L is deprecated...

	uart_get_hw_flow_ctrl(FOUR_G,&fctrl);
	printf("NEW SIMCOM UART FCTL: %02X\n",fctrl);
	
#if 1
// usual UART interrupt routine used:
// Set uart pattern detect function.
	prev_pattern_detect_char = 0xFF;
	pattern_detect_char = set_uart_pattern_detect(DFLT_EOLSTR);
	
//    uart_enable_pattern_det_intr(FOUR_G, 0x0A, 1, 10000, 10, 10);		// detect 1 x LF char
#else
// custom UART interrupt routine used:
// release the pre registered UART handler/subroutine
	ESP_ERROR_CHECK(uart_isr_free(EX_UART_NUM));

	// register new UART subroutine
	ESP_ERROR_CHECK(uart_isr_register(EX_UART_NUM,uart_intr_handle, NULL, ESP_INTR_FLAG_IRAM, &handle_console));

	// enable RX interrupt
	ESP_ERROR_CHECK(uart_enable_rx_intr(EX_UART_NUM));
#endif

//#ifndef NEW_MQTT_SERIAL
// Create a task to handler UART event from ISR
    xTaskCreate(four_g_uart_task, "four_g_uart_task", 2048, (void*)FOUR_G, 12, NULL);
//#endif	

	printf("Creating MQTT queues...\n");

	mqtt_cmd_queue = xQueueCreate(2048,sizeof(char));	// char-by-char queue
	printf("MQTT CMD q ");
	if (mqtt_cmd_queue == NULL)
		printf("fail\n");
	else
		printf("OK\n");

	mqtt_data_queue = xQueueCreate(2048,sizeof(char));	// char-by-char queue
	printf("MQTT DATA q ");
	if (mqtt_data_queue == NULL)
		printf("fail\n");
	else
		printf("OK\n");
#endif

#ifdef USE_M4G_UDP
	udp_cmd_queue = xQueueCreate(2048,sizeof(char));
	printf("UDP CMD q ");
	if (mqtt_cmd_queue == NULL)
		printf("fail\n");
	else
		printf("OK\n");

	udp_data_queue = xQueueCreate(2048,sizeof(char));
	printf("UDP DATA q ");
	if (udp_data_queue == NULL)
		printf("fail\n");
	else
		printf("OK\n");
#endif


#ifdef ZULU_RADIO_SLAVE
///////////////////////////
// ZULU RADIO SLAVE UART
///////////////////////////
	printf("Installing UART 2 for ZULU RADIO SLAVE...\n");

	uart_config_t zulu_radio_uart_config = 
		{
		.baud_rate = 38400,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//		.flow_ctrl = UART_HW_FLOWCTRL_CTS,
		.rx_flow_ctrl_thresh = 110
		};
//ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));	
	uart_param_config(ZULU_RADIO, &zulu_radio_uart_config);	

// Set UART pins(TX: IO16 (UART2 default), RX: IO17 (UART2 default), RTS: IO18, CTS: IO19)
//ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, 18, 19));

	uart_set_pin(ZULU_RADIO, TXD2, RXD2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);	//RTS2, CTS2 );

// Install UART driver using an event queue here
	const int zulu_radio_uart_buffer_size = (1024 * 2);
//ESP_ERROR_CHECK(uart_driver_install(UART2, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
	uart_driver_install(ZULU_RADIO, zulu_radio_uart_buffer_size, zulu_radio_uart_buffer_size, 10, &zulu_radio_uart_queue, 0);

// Create a task to handler UART event from ISR
//    xTaskCreate(zulu_radio_uart_task, "zulu_radio_uart_task", 2048, (void*)ZULU_RADIO, 12, NULL);
	gpio_set_level(ZULU_RADIO_SLAVE_RTS,0);	// RTS is active LOW
#endif





#ifdef LORA_USED	//_LORA
	lora_data_queue = xQueueCreate(100,sizeof(host_rcv_data_t));	//sizeof(uint32_t));	//sizeof(host_rcv_data_t)
	printf("LORA data q ");
	if (lora_data_queue == NULL)
		printf("fail\n");
	else
		printf("OK\n");

	lora_gpio_queue = xQueueCreate(100,sizeof(uint32_t));	//sizeof(uint32_t));  host_rcv_data_t
	printf("LORA gpio q ");
	if (lora_gpio_queue == NULL)
		printf("fail\n");
	else
		printf("OK\n");
#endif

// Write data to UART.
//	const char* four_g_test_str = "DEBUG test string.\r\n";
//	uart_write_bytes(FOUR_G, (const char*)four_g_test_str, strlen(four_g_test_str));

/*
// AUX RX DISCONNECTED ON VER 1 PCB!
// Read data from UART.
	uint8_t four_g_rx_data[128];
	int four_g_rx_length = 0;
//	ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length));
	uart_get_buffered_data_len(AUX, (size_t*)&four_g_rx_length);
	four_g_rx_length = uart_read_bytes(AUX, four_g_rx_data, four_g_rx_length, 100);
*/

#ifdef SERIAL2_USED	
	printf("UART 1 used...\n");
//#ifdef USE_EXT_SERIAL


#ifdef ZULU_RADIO_MASTER
///////////////////////////
// ZULU MASTER RADIO UART
///////////////////////////
	printf("Installing UART 1 for ZULU RADIO MASTER...\n");

	uart_config_t zulu_uart_config = 
		{
		.baud_rate = 38400,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,	//CTS_RTS,
		.rx_flow_ctrl_thresh = 110
		};
		

//ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));	
	uart_param_config(ZULU_RADIO, &zulu_uart_config);	

// Set UART pins(TX: IO16 (UART2 default), RX: IO17 (UART2 default), RTS: IO18, CTS: IO19)
//ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, 18, 19));
//	uart_set_pin(RADIO, TXD1, RXD1, RTS1, CTS1);
//#ifndef USE_LORA	// conflict with LORA_DIO2
	uart_set_pin(ZULU_RADIO, TXD1, RXD1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
//#endif
// Install UART driver using an event queue here
	const int zulu_radio_uart_buffer_size = (1024 * 2);
//ESP_ERROR_CHECK(uart_driver_install(UART2, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
	uart_driver_install(ZULU_RADIO, zulu_radio_uart_buffer_size, zulu_radio_uart_buffer_size, 10, &zulu_radio_uart_queue, 0);

// Set uart pattern detect function.
// Create a task to handler UART event from ISR
//    xTaskCreate(zulu_radio_uart_task, "zulu_radio_uart_task", 2048, (void*)ZULU_RADIO, 12, NULL);

//	printf("Got to here...\n");
//	vTaskDelay(100 / portTICK_PERIOD_MS);

//#ifdef SERIAL2_USED	//#if 1

#ifndef LORA_USED	//_LORA
#define ESP_INTR_FLAG_DEFAULT 0
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
#endif

	gpio_isr_handler_add(RXD1,ext_serial_bitrate_isr_handler, (void*) RXD1);
	gpio_set_intr_type(RXD1,GPIO_INTR_ANYEDGE);
	gpio_intr_enable(RXD1);
//#endif
	ext_serial_bit_time = 0xFFFFFFFF;
	prev_ext_serial_bit_time = 0xFFFFFFFF;
	ext_serial_bit_time_change_flag = 0;
	bit_time_counter = 0;

#endif	// end of "#ifdef ZULU_RADIO_MASTER..."




	
#endif	
	binary_data_flag = 0;			// assume serial data will be 7 bit ASCII unless told otherwise...
	auto_binary_data_flag = 0;		// assume serial data will be 7 bit ASCII unless told otherwise...
	binary_mode_flag = 0;	


#if defined(USE_LORA) || defined(USE_LORA_LINK)
///////////////////
// LORA Setup
///////////////////
printf("Initialising LoRa SPI bus...\n");

spi_bus_config_t spi_cfg = 
{ 
.mosi_io_num = LORA_MOSI,					// MISO pin
.miso_io_num = LORA_MISO,					// MOSI pin
.sclk_io_num = LORA_SCK,					// SCK pin
.quadwp_io_num = -1,						// quad mode WP pin
.quadhd_io_num = -1,						// quad mode MSB
.max_transfer_sz = LORA_REG_BUFFER_SIZE,	// max transfer size
.flags = 0,									// flags
.intr_flags = 0								// interrupt flags
};
// set up spi bus
spi_bus_initialize(SPI2_HOST,&spi_cfg,0);	// 0 = SPI_DMA_DISABLED   //SPI_DMA_CH_AUTO);	//DISABLED);

 spi_device_interface_config_t lora_cfg =
 {
 .command_bits = 0,
 .address_bits = 8,
 .mode=0,                                	//SPI mode 0
 .clock_speed_hz=1*1000*1000,               //Clock out at 1 MHz (10MHz is abs max)
 .spics_io_num=LORA_NSS,               		//CS pin
 .queue_size=7,                          	//We want to be able to queue 7 transactions at a time
 .pre_cb=spi_pre_transfer_callback,  		//Specify pre-transfer callback to handle D/C line
 };
 // attach the LORA spi device
spi_bus_add_device(SPI2_HOST,&lora_cfg,&spi);	// spi is registered as the device handle

// switch on debug for development...
dbgflag = 1;
dbgbits = dbgbits | DBG_LORA;

#ifdef SIMCOM_PROGRAMMING_MODE
dbgflag = 1;
dbgbits = dbgbits | DBG_4G_TXRX;
#endif

//SX1276_reset();

//SX1276_init(spi,SX1276_init_vals);
//SX1276_set_868(spi);

#endif

	
#ifdef USE_I2C
///////////////////
// I2C
///////////////////
	printf("Installing I2C driver...\r\n");
	ESP_ERROR_CHECK(i2c_master_init());
	printf("I2C driver done\r\n");
	
	bt_i2c_update_timer = 0;
#endif

#ifdef USE_TANK_SENSORS
	gpio_set_level(TSENSCTRL,1);			// set tank sensor power OFF

	printf("Tank sensors: ");
	
//	if ((gpio_get_level(TANKSENSOR0) == 0) && (gpio_get_level(TANKSENSOR1) == 0))	// must be pulled down by the tank sensor PCB
	if (gpio_get_level(TANKSENSOR0) == 0)	// must be pulled down by the tank sensor PCB
		tank_sensors_present_flag = 1;
	else
		{
		tank_sensors_present_flag = 0;
		printf("NOT ");
		}
		
	printf("present\r\n");

//	tank_sensors_present_flag = 1;	// fudge for now...
		
#endif

	
//////////////////////////////////////////////
// START CODE
//////////////////////////////////////////////

	
/////////////////////
// SET UP VARIABLES
/////////////////////

	secs = 0;
	mins = 0;
	hrs = 0;
	days = 1;
	month = 1;
	year = 0;
	day_of_week = 0;
	
	wifi_scan_time = DFLT_WIFI_SCAN_TIME;

	MAC_seq_number = 0;
	
	for (i=0;i<NUM_DEVS;i++)
		{
		status_array[i] = 0;
		}


	cmd_flag = 0;
	cmd_string[0] = 0x00;

	four_g_count = 0;

#ifdef WIFI_SCAN
	wifi_dev_count = 0;
	wifi_total_dev_count = 0;
	wifi_current_num = 0;
	wifi_previous_num = 0;
	wifi_list_current = wifi_list_A;
	wifi_list_previous = wifi_list_B;
	wifi_scan_enable_flag = 1;
	wifi_scan_valid = 0;
#endif

	wifi_is_on = 1;

#ifdef USE_BLUETOOTH
	wifi_scan_enable_flag = 1;
	wifi_is_on = 0;
#endif
	bt_msg_ready_count = 0;
//	bt_sensor_data[16];
	bt_sensor_data_len = 0;
	bt_advertising_ready_flag = 0;
					
	ntptime_sync_flag = 0;
	TIM_msg_time = 100;		// forces 10 sec time display
	TIM_msg_time_count = 0;
		
	num_MAC_addrs = 0;
	wifi_scan_flag = 0;
	wifi_check_flag = 0;
	
    x100msec_flag = 0;
	x1sec_flag = 0;
	x10sec_flag = 0;

// up-time counter
	u100msec = 0;
	u1sec = 0;
	u1min = 0;
	u1hr = 0;
	u1day = 0;

	
	wifi_scan_timer = 0;


// initialise storage for geo MAC addresses
	for (i=0;i<4;i++)
		{
		for (j=0;j<6;j++)
			{
			geo_mac_addr[i][j] = 0;
			}
		}
	
	geo_rssi[0] = -127;
	geo_rssi[1] = -127;
	
	cell_rssi = -127;
	
	init_tm_data(1);

	
	tm_len = 0;
	
	up_time = 0;
	
	temperature = 0;

	sleep_flag = 1;				// enable sleep mode
	sleep_resume_flag = 0;
	sleep_cycle_count = 0;
	uart_stay_awake_time = DFLT_UART_STAY_AWAKE_TIME;
	uart_stay_awake_count = 0;

	led_count = 0;

	sw_status = 0;
	prev_sw_status = 0;

#ifdef USE_M4G
//	four_g_state = M4G_POWER_OFF;
#endif
//	prev_four_g_state = 0xFF;

#ifdef USE_M4G
	simcom_busy_flag = SIMCOM_BUSY_IDLE;
	prev_simcom_busy_flag = 0xFF;
	
	simcom_state = SIMCOM_POWER_OFF;
	simcom_comms_flag = 0;			// havent detected good SIMCOM comms yet...
	simcom_powerdown_attempts = 0;
#endif
	prev_simcom_state = 0xFF;

#ifdef USE_M4G_MQTT
	mqtt_state = MQTT_NO_COMM_IDLE;
	prev_mqtt_state = 0xFF;
#endif

#ifdef USE_M4G_UDP
	udp_state = UDP_NO_COMM_IDLE;
	prev_udp_state = 0xFF;
#endif

#ifdef USE_WIFI_MQTT
	wifi_mqtt_state = MQTT_NO_COMM_IDLE;
	prev_wifi_mqtt_state = 0xFF;
	wifi_mqtt_disable_flag = 0;
#else
	wifi_mqtt_disable_flag = 1;
#endif

#ifdef USE_M4G_MQTT
	mqtt_sensor_msg_timer = 0;
	mqtt_login_state = NOT_SRVR_INITIALISED;
	prev_mqtt_login_state = 0xFF;
	mqtt_sensor_msg_flag = 0;
	four_g_msg_rx_count = 0;
	CMQTT_rx_flag = SIMCOM_DATA;	
	CMQTT_rx_msg = 0;
	server_cmd_test_flag = 0;
	server_cmd_test_str[0] = 0x00;

	server0_addr_ptr = 0;
	server1_addr_ptr = 0;

#endif

#ifdef USE_M4G_MQTT_SSL
	ssl_enable_flag = 1;
#else
	ssl_enable_flag = 0;
#endif

	four_g_timeout_timer = 0xFFFF;			// set to max so timeout  doesnt go off by accident
	four_g_timeout_val = 0xFFFF;			// set to max so timeout  doesnt go off by accident
	four_g_response_flag = 0;
	four_g_eol_response_flag = 0;

	simcom_response_flag = 0;
	simcom_eol_response_flag = 0;
	mqtt_response_flag = 0;
	mqtt_eol_response_flag = 0;
	udp_response_flag = 0;
	udp_eol_response_flag = 0;
	udp_test_flag = 0;
	
//	four_g_sm_timer_flag = 0;

	response_flag = 0;
	server_response_flag = SRVR_NO_RESPONSE;


	simcom_timeout_timer = 0xFFFF;			// set to max so timeout  doesnt go off by accident
	simcom_timeout_val = 0xFFFF;			// set to max so timeout  doesnt go off by accident

	mqtt_timeout_timer = 0xFFFF;			// set to max so timeout  doesnt go off by accident
	mqtt_timeout_val = 0xFFFF;			// set to max so timeout  doesnt go off by accident

	udp_timeout_timer = 0xFFFF;			// set to max so timeout  doesnt go off by accident
	udp_timeout_val = 0xFFFF;			// set to max so timeout  doesnt go off by accident

#ifdef USE_WIFI_MQTT
	mqtt_transport_mode = MQTT_MODE_NONE;	//MQTT_MODE_WIFI;	//MQTT_MODE_4G;
	mqtt_preferred_transport_mode = PREFERRED_MQTT_MODE;
	wifi_sta_connected = 0;
	wifi_mqtt_connected = 0;
	wifi_mqtt_started = 0;
	wifi_mqtt_attempts = 0;
#endif

	for (i=0;i<WIFI_CREDENTIALS_MAX;i++)
		{
		wifi_sta_SSID[i][0] = 0x00;
		wifi_sta_PWD[i][0] = 0x00;
		}
	wifi_sta_credentials_index = 0;

#ifdef USE_WIFI_UDP
#define PREFERRED_UDP_MODE		UDP_MODE_WIFI

	udp_transport_mode = UDP_MODE_NONE;	//MQTT_MODE_WIFI;	//MQTT_MODE_4G;
	udp_preferred_transport_mode = PREFERRED_UDP_MODE;
#endif

#ifdef USE_BLUETOOTH
	continue_ble_commands = 1;
#endif
	
	strcpy(gps_lat_str,"test_lat");
	strcpy(gps_lon_str,"test_long");
	gps_spd = 0;
	lora_radio_freq = 868000;
	
	tank_sensor_0 = 0;
	tank_sensor_1 = 0;
	tank_sensor_percent_0 = 0;
	tank_sensor_percent_1 = 0;


	ambi_seq_num = 0;
	bme280_seq_num = 0;
	bme688_seq_num = 0;
	ltr559_seq_num = 0;

	lsm303d_seq_num = 0;
	lsm303d_section = 0;
	
	ambi_done_flag = I2C_SENSOR_READY;
	bme280_done_flag = I2C_SENSOR_READY;
	bme688_done_flag = I2C_SENSOR_READY;
	ltr559_done_flag = I2C_SENSOR_READY;
	lsm303d_done_flag = I2C_SENSOR_READY;
	
	tx_bluetooth_whitelist_flag = 0;
	tx_bluetooth_blacklist_flag = 0;
	tx_bluetooth_devname_whitelist_flag = 0;
	enable_bluetooth_whitelist_flag = 0;		// Bluetooth whitelist enable flag
	enable_bluetooth_blacklist_flag = 0;		// Bluetooth blacklist enable flag
	bluetooth_whitelist_mode = WL_IDLE;
	bluetooth_blacklist_mode = WL_IDLE;
	bluetooth_beacon_whitelist_mode	= WL_IDLE;
	num_bt_wl_entries = 0;
	num_bt_bl_entries = 0;
	num_bt_devname_wl_entries = 0;

	bluetooth_devname_entries = 0;

	tx_lorawan_whitelist_flag = 0;
	tx_lorawan_blacklist_flag = 0;
	enable_lorawan_whitelist_flag = 0;		// LoRaWAN whitelist enable flag
	enable_lorawan_blacklist_flag = 0;		// LoRaWAN blacklist enable flag
	lorawan_whitelist_mode = WL_IDLE;
	lorawan_blacklist_mode = WL_IDLE;
	num_lora_wl_entries = 0;
	num_lora_bl_entries = 0;
	

	tx_serial_settings_flag = 0;
	serial_bitrate = DFLT_EXT_SERIAL_BPS;
	auto_serial_bitrate = 0;
	eolstr[0] = DFLT_EOLSTR;
	eolstr[1] = 0x00;
	eol_remove_flag = 0;
 	
#ifdef LORA_USED	//_LORA
	lora_reset_timer = 0xFF;
	lora_reset_complete_flag = 0;
	lora_tx_packet_open = 0;

	prev_dio_val = 0;
	
	lora_packet_count =0;
	lora_prev_packet_count = 0;
#endif
	lora_msg_ready_count = 0;

#ifdef USE_LORA_LINK
	lora_rx_data_len = 0;
	lora_tx_flag = 0;
	lora_rx_flag = 0;
#endif

#if !defined(USE_M4G_MQTT) && !defined(USE_M4G_UDP)
	simcom_disable_flag = 1;
#else
	simcom_disable_flag = 0;
#endif

#ifdef USE_M4G_MQTT
	mqtt_disable_flag = 0;
#else
	mqtt_disable_flag = 1;
#endif

#ifdef USE_M4G_UDP
	udp_disable_flag = 0;
#else
	udp_disable_flag = 1;
#endif

#ifdef USE_WIFI_UDP
	strcpy(wifi_udp_url,"lora.core.aql.com");
	wifi_udp_port = 1700;
	wifi_udp_connected = false;
	wifi_udp_DNSfound = false;
	prev_wifi_udp_DNSfound = false;
	wifi_udp_DNSstart = false;
	wifi_udp_DNS_timer = 0;
	wifi_udp_addr_family = 0;
	wifi_udp_ip_protocol = 0;

	
#endif

#ifdef USE_WIFI_MQTT
	wifi_disable_flag = 0;
#else
	wifi_disable_flag = 1;
#endif

	gateway_sensor_timer = 0;
	gps_sensor_timer = 0;
	i2c_sensor_timer = 0;
	tank_sensor_timer = 0;
	bluetooth_sensor_timer = 0;
	bluetooth_MAC_addr_timer = 0;
	bluetooth_devname_timer = 0;
	bluetooth_classic_scan_timer = 0;
	lora_devaddr_timer = 0;

	gateway_sensor_time = GATEWAY_SENSOR_TIME;
	gps_sensor_time = GPS_SENSOR_TIME;
	i2c_sensor_time = I2C_SENSOR_TIME;
	tank_sensor_time = TANK_SENSOR_TIME;
	bluetooth_sensor_time = BLUETOOTH_SENSOR_TIME;
	bluetooth_MAC_addr_time = BLUETOOTH_MAC_ADDR_TIME;
	bluetooth_devname_time = BLUETOOTH_DEVNAME_TIME;
#ifdef USE_BLUETOOTH_CLASSIC_DETECT
	bluetooth_classic_scan_time = BLUETOOTH_CLASSIC_SCAN_TIME;
#endif
	lora_devaddr_time = LORA_DEVADDR_TIME;

#ifdef USE_ULTRASONIC_SENSOR
	usm_trigger_count = 0;
	usm_timeout_count = 0;
	usm_state = USM_IDLE;
	prev_usm_state = 0xFF;
	usm_int_count = 0;
	usm_trigger_flag = 0;
	usm_complete_flag = 0;
	usm_echo_time = 0;
	usm_distance = 0;
	usm_ave_distance = 0;
	usm_ave_count = 0;
	usm_accum = 0;
	usm_ave_max = 500;
	usm_completed_ok_flag = 0;
	usm_cabin_empty_flag = 0;
	UV_clean_trigger_flag = 0;
#endif

	zulu_radio_run_flag = 0;
#ifdef USE_ZULU_RADIO
	zulu_radio_hb_timer = 0;	
	zulu_radio_status_timer = 0;	
	zulu_radio_timeout_timer = 0;	
	zulu_radio_tx_seq_num = 0;
	zulu_radio_rx_seq_num = 0;
	zulu_radio_slave_status = 0;
	zulu_radio_get_timer_flag = 0;
	zulu_radio_set_timer_flag = 0;
	zulu_radio_run_timer = 0;
	zulu_radio_msgs_flag = 1;
	zulu_radio_response_received = 1;
#endif



#ifdef ZULU_RADIO_SLAVE

#endif


	
//#ifdef NEW_MQTT_SERIAL
// need to be globals, to preserve across fg loop!		
//	dtmp = NULL;
//	dleft = NULL;
 //   dleft_size = 0;
//    dleft_pos = 0;
//	dprev_size = 0;
//	end_flag = 0;			// ??
//	endflag = 0;		// ??
//	mqtt_step = 0;		
//#endif

	dbg_count = 0;

	evt_num = 0;
	evt_enable_flag = 0;
	evt_mode_flag = EVENT_MODE_IDLE;
	evt_day = 1;
	evt_t1_hrs = 0;
	evt_t1_mins = 0;
	evt_t2_hrs = 0;
	evt_t2_mins = 0;
	evt_pwm = 0;;

	evt_curr_num = 0;
	evt_curr_mode_flag = EVENT_MODE_SCHEDULED;
	evt_start_stop_flag = EVENT_STOP;
	evt_curr_PWM = 0;
	evt_curr_error_byte = 0;
	evt_notifications = 0;
	
// initialise event variables (should be overwritten with NVS values later...)
	for (i=0;i<NUM_EVENTS_MAX;i++)
		{
		event_list[i].evt_enable_flag = 0;
		event_list[i].evt_mode_flag   = EVENT_MODE_IDLE;
		event_list[i].evt_day         = 0;
		event_list[i].evt_t1_hrs      = 0;
		event_list[i].evt_t1_mins     = 0;
		event_list[i].evt_t2_hrs      = 0;
		event_list[i].evt_t2_mins     = 0;
		event_list[i].evt_pwm         = 0;
		event_list[i].evt_running     = 0;

		evt_timer[i] = EVENT_TIMER_IDLE;
		evt_repeat_timer[i] = EVENT_TIMER_IDLE;
		}


	get_server_time_flag = 1;									// set so that unit requests server time at boot...
	get_server_time_timer = GET_SERVER_TIME_PERIOD * 60;		// set so that unit requests server time at boot...
	test_server_time_flag = 0;									// 
	srvr_test_count = 0;
	srvr_test_OK_count = 0;
	srvr_test_run_flag = 0;
	srvr_return_flag = 0;
	
	queues_enabled_flag = 0;
	net_test_flag = 0;

	printf("Vars set\n");
	
///////////////////
// SET UP
///////////////////
// stop radio reset
		set_output(DEV_LORA_RADIO_RESET,DEVICE_OFF,INVERTED);
// stop 4G reset
		set_output(DEV_FOUR_G_RESET,DEVICE_OFF,0);
// stop  4G power
		set_output(DEV_FOUR_G_POWER,DEVICE_OFF,0);				

#if PCB_VER == VER_1_0_D
// stop  4G power supply
		set_output(DEV_FOUR_G_PSU_EN,DEVICE_OFF,0);				
#endif

// stop IR LED
//		set_output(DEV_IR_LED,DEVICE_OFF,INVERTED);
// stop LED_0
		set_output(DEV_LED_0,DEVICE_OFF,1);
// stop LED_1
		set_output(DEV_LED_1,DEVICE_OFF,1);
// stop LED_2
		set_output(DEV_LED_2,DEVICE_OFF,1);
// stop LED_3
		set_output(DEV_LED_3,DEVICE_OFF,1);

// V1_0_D shift reg test 
while(0)
	{
	dbgflag = 1;
	dbgbits = 1<<3;	// DBG_SD_IN_OUT

	if (x100msec_flag)
		{
		x100msec_flag = 0;
		led_count++;
		
		printf("LED count: %02X\n",led_count);
		
		if (led_count & (1<<3))
			set_output(DEV_LED_0,DEVICE_ON,1);
		else
			set_output(DEV_LED_0,DEVICE_OFF,1);

		if (led_count & (1<<4))
			set_output(DEV_LED_1,DEVICE_ON,1);
		else
			set_output(DEV_LED_1,DEVICE_OFF,1);

		if (led_count & (1<<5))
			set_output(DEV_LED_2,DEVICE_ON,1);
		else
			set_output(DEV_LED_2,DEVICE_OFF,1);

		if (led_count & (1<<6))
			set_output(DEV_LED_3,DEVICE_ON,1);
		else
			set_output(DEV_LED_3,DEVICE_OFF,1);
		}

// reset WatchDog Timer for this thread...
	esp_task_wdt_reset();		
// yield to OS for a while (required)
	vTaskDelay(10 / portTICK_PERIOD_MS);
	}

	printf("\r\n\r\nInitialising...\r\n\r\n");
		

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) 
		{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
		}
    ESP_ERROR_CHECK(ret);

//	uart_set_tx_empty_threshold(DBG,127);
	
		
#if 1
/////////////////////////////////////////
// Initialise NVS-Related Vars...
/////////////////////////////////////////
{
unsigned char var = 0xFF;
unsigned int ivar;
unsigned long long int lvar = 0;
unsigned char mac_list[6];
unsigned char i,j;
unsigned int n;
char str[40];
size_t len;
esp_err_t err,err2;
	
nvs_handle nvs_handle;


/////////////////////////////////////////
// check the GateWay Vars...
/////////////////////////////////////////

// GATEWAY VARIABLE SPACE (max 15 chars!)
err = nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
if (err != ESP_OK)
	printf("nvs_open: %d\n",err);

err = nvs_get_u32(nvs_handle,"GW_SER_RATE",(uint32_t*)&ivar);
if (err != ESP_OK)
	{
	nvs_gw_var_init();

	}
else
	{
	printf("*** NVS GW VAR values already set...\n");
	}

	set_gw_vars_from_nvs();

/////////////////////////////////////////
// check the Bluetooth Vars...
/////////////////////////////////////////
#ifdef  USE_BLUETOOTH
nvs_open("BT_MAC_LIST",NVS_READWRITE, &nvs_handle);
//nvs_erase_all(nvs_handle);
//nvs_commit(nvs_handle);

err = nvs_get_i8(nvs_handle,"BTW_MAC_MAX",(int8_t*)&var);
err2 = nvs_get_i8(nvs_handle,"BTB_MAC_MAX",(int8_t*)&var);
nvs_close(nvs_handle);

if (err != ESP_OK)	// BT whitelist
	{
	printf("*** setting BTW MAC values...\n");

	nvs_bl_var_init("BT_MAC_LIST", "BTW_MAC_MAX", "BTW_MAC", (char*)bluetooth_wl_init, bluetooth_wl_init_len, BLUETOOTH_CMD_LIST_ENTRY_LENGTH,BINARY_VAL);
	}
else
	{
	printf("*** BTW MAC values already set...\n");	
	}

set_bl_vars_from_nvs("BT_MAC_LIST", "BTW_MAC_MAX", "BTW_MAC", (char*)bluetooth_cmd_whitelist, &num_bt_wl_entries, BLUETOOTH_CMD_LIST_ENTRY_LENGTH,BINARY_VAL);


if (err2 != ESP_OK)	// BT blacklist
	{
	printf("*** setting BTB MAC values...\n");

	nvs_bl_var_init("BT_MAC_LIST", "BTB_MAC_MAX", "BTB_MAC", (char*)bluetooth_bl_init, bluetooth_bl_init_len, BLUETOOTH_CMD_LIST_ENTRY_LENGTH,BINARY_VAL);
	}
else
	{
	printf("*** BTB MAC values already set...\n");	
	}
set_bl_vars_from_nvs("BT_MAC_LIST", "BTB_MAC_MAX", "BTB_MAC", (char*)bluetooth_cmd_blacklist, &num_bt_bl_entries, BLUETOOTH_CMD_LIST_ENTRY_LENGTH,BINARY_VAL);


nvs_open("BD_MAC_LIST",NVS_READWRITE, &nvs_handle);
//nvs_erase_all(nvs_handle);
//nvs_commit(nvs_handle);

err = nvs_get_i8(nvs_handle,"BDW_MAC_MAX",(int8_t*)&var);
nvs_close(nvs_handle);

if (err != ESP_OK)	// BT devname whitelist
	{
	printf("*** setting BDW MAC values...\n");

	nvs_bl_var_init("BD_MAC_LIST", "BDW_MAC_MAX", "BDW_MAC", (char*)bluetooth_dw_init, bluetooth_dw_init_len, BLUETOOTH_DEVNAME_LIST_ENTRY_LENGTH,ASCII_VAL);
	}
else
	{
	printf("*** BDW MAC values already set...\n");	
	}
set_bl_vars_from_nvs("BD_MAC_LIST", "BDW_MAC_MAX", "BDW_MAC", (char*)bluetooth_devname_whitelist, &num_bt_devname_wl_entries, BLUETOOTH_DEVNAME_LIST_ENTRY_LENGTH,ASCII_VAL);

#endif	// end of "#ifdef USE_BLUETOOTH..."


#ifdef LORA_USED	//_LORA
nvs_open("LORA_MAC_LIST",NVS_READWRITE, &nvs_handle);
//nvs_erase_all(nvs_handle);
//nvs_commit(nvs_handle);

err = nvs_get_i8(nvs_handle,"LOW_MAC_MAX",(int8_t*)&var);
err2 = nvs_get_i8(nvs_handle,"LOB_MAC_MAX",(int8_t*)&var);
nvs_close(nvs_handle);

if (err != ESP_OK)
	{
	printf("*** setting LORA_W MAC values...\n");

	nvs_bl_var_init("LORA_MAC_LIST", "LOW_MAC_MAX", "LOW_MAC", (char*)&lorawan_wl_init, lorawan_wl_init_len, LORAWAN_CMD_LIST_ENTRY_LENGTH,BINARY_VAL);
	}
else
	{
	printf("*** LOW MAC values already set...\n");	
	}

set_bl_vars_from_nvs("LORA_MAC_LIST", "LOW_MAC_MAX", "LOW_MAC", (char*)&lorawan_cmd_whitelist, &num_lora_wl_entries, LORAWAN_CMD_LIST_ENTRY_LENGTH,BINARY_VAL);

if (err2 != ESP_OK)
	{
	printf("*** setting LORA_B MAC values...\n");

	nvs_bl_var_init("LORA_MAC_LIST", "LOB_MAC_MAX", "LOB_MAC", (char*)&lorawan_bl_init, lorawan_bl_init_len, LORAWAN_CMD_LIST_ENTRY_LENGTH,BINARY_VAL);
	}
else
	{
	printf("*** LORA_B MAC values already set...\n");	
	}
set_bl_vars_from_nvs("LORA_MAC_LIST", "LOB_MAC_MAX", "LOB_MAC", (char*)&lorawan_cmd_blacklist, &num_lora_bl_entries, LORAWAN_CMD_LIST_ENTRY_LENGTH,BINARY_VAL);


#endif	// end of "#ifdef USE_LORA..."

// Bluetooth Sensor
nvs_open("BT_SENSOR",NVS_READWRITE, &nvs_handle);
err = nvs_get_u16(nvs_handle,"BTS_ADV_TIME",(uint16_t*)&ivar);
nvs_close(nvs_handle);

if (err != ESP_OK)
	{
	printf("*** setting BT Sensor values...\n");
	nvs_open("BT_SENSOR",NVS_READWRITE, &nvs_handle);

	nvs_set_str(nvs_handle,"BTS_NAME",BLUETOOTH_ADVERTISING_NAME);
	nvs_set_u16(nvs_handle,"BTS_ADV_TIME",(uint16_t)BLUETOOTH_ADVERTISING_TIME);

	nvs_commit(nvs_handle);		
	nvs_close(nvs_handle);

	}
else
	{
	printf("*** BT Sensor values already set...\n");	
	}

	{
	size_t n = 15;
	
	nvs_open("BT_SENSOR",NVS_READWRITE, &nvs_handle);

	nvs_get_str(nvs_handle,"BTS_NAME",bluetooth_sensor_adv_name,&n);
	nvs_get_u16(nvs_handle,"BTS_ADV_TIME",(uint16_t*)&bluetooth_sensor_adv_time);

	nvs_close(nvs_handle);
	
	printf("BlueTooth Sensor Local Device Name:  %s\n",bluetooth_sensor_adv_name);
	printf("BlueTooth Sensor Advertising Time:  %d msec\n",bluetooth_sensor_adv_time*10/16);
	}


//////////////////////////////////////////////
// check the Server Address And Port Vars...
//////////////////////////////////////////////

#if defined(USE_M4G_MQTT) || defined(USE_WIFI_MQTT)
// ADDR and PORT VARIABLE SPACE (max 15 chars!)
err = nvs_open("SRVR_ADDR_LIST",NVS_READWRITE, &nvs_handle);
if (err != ESP_OK)
	printf("nvs_open err: %d\n",err);

//#ifdef USE_M4G_MQTT


nvs_close(nvs_handle);
	
if (err != ESP_OK)
	{
	nvs_srvr_addr_port_var_init();
	}
else
	{
	printf("*** NVS SRVR ADDR and PORT values already set...\n");

	}

set_srvr_addr_port_vars_from_nvs();

show_server_addrs();
#endif


#ifdef LORA_USED
// LORA_REG VARIABLE SPACE (max 15 chars!)
nvs_open("LORA_REG_LIST",NVS_READWRITE, &nvs_handle);
err = nvs_get_u8(nvs_handle,"LORA_REG_29",(uint8_t*)&var);	//LREG_MDM_CFG1
nvs_close(nvs_handle);

if (err != ESP_OK)
	{
	printf("*** setting LORA REG values...\n");
	nvs_lora_reg_var_init();
	}
// DONT NEED to set vars as this is done in SX1276_init()
/*
else
	{
	printf("*** LORA REG values already set...\n");	
	}

set_lora_reg_vars_from_nvs();
*/

#endif

// Wifi init vars




#ifdef USE_WIFI_MQTT
nvs_open("WIFI_MQT_VARS",NVS_READWRITE, &nvs_handle);
//nvs_erase_all(nvs_handle);
//nvs_commit(nvs_handle);
//printf("max len: %d\n",NVS_KEY_NAME_MAX_SIZE-1);

// specify size of receiving string...
len = 32;
err = nvs_get_str(nvs_handle,"WIFI_MQ_STA_ID",str,&len);
//      nvs_get_str(nvs_handle,"WIFI_MQ_STA_ID",str,&buflen);
	printf("%s  %d\n",str,err);
nvs_close(nvs_handle);

if (err != ESP_OK)
	{
	printf("rslt: %d\n",err);

	printf("*** setting Wifi MQTT values...\n");

	nvs_wifi_sta_var_init();
	}
else
	{
	printf("*** Wifi MQTT values already set...\n");	
	}


set_wifi_sta_vars_from_nvs();

	printf("%s %s\n",wifi_sta_SSID[wifi_sta_credentials_index], wifi_sta_PWD[wifi_sta_credentials_index]);

#endif

}

printf("\n");

{
// get stats for nvs use...
nvs_stats_t nvs_stats;
nvs_get_stats(NULL, &nvs_stats);
printf("NVS Area: UsedEntries = (%d), FreeEntries = (%d), AllEntries = (%d)\n",
       nvs_stats.used_entries, nvs_stats.free_entries, nvs_stats.total_entries);

//nvs_stats_t nvs_stats;
//nvs_get_stats(NULL, &nvs_stats);
//printf("Count: UsedEntries = (%lu), FreeEntries = (%lu), AvailableEntries = (%lu), AllEntries = (%lu)\n",
//       nvs_stats.used_entries, nvs_stats.free_entries, nvs_stats.available_entries, nvs_stats.total_entries);
}

#endif

/////////////////////
// initialise wifi
/////////////////////

	esp_efuse_mac_get_default(mac_addr);
	mac_to_str(mac_addr, 0,mac_bin_str);

	mac_to_str(mac_addr, ':',mac_str);
	printf("MQTT MAC  address is %s\r\n",mac_bin_str);
	printf("EFUSE MAC address is %s\r\n",mac_str);
		
	esp_base_mac_addr_set(mac_addr);
		
	esp_read_mac(mac_addr,ESP_MAC_WIFI_SOFTAP);
	mac_to_str(mac_addr, ':',mac_str);
	printf("Wifi MAC  address is %s\r\n\r\n",mac_str);


	
// set power, reset signals to inactive state...
	set_output(DEV_FOUR_G_POWER,0,0);


/////////////////////
// initialise 4G
/////////////////////

// start 4G using state machine...
#ifdef USE_M4G
//	four_g_state = M4G_POWER_EN;
#endif

#ifdef USE_M4G
	simcom_state = SIMCOM_POWER_EN;
#endif

/////////////////////////////
// start wifi event handler 
/////////////////////////////
// use Wifi MAC addr as unique SSID identifier...
	esp_read_mac(mac_addr,ESP_MAC_WIFI_SOFTAP);
	mac_to_str(mac_addr, 0,mac_ID_str);

// look in MAC_Rpt_wifi.cpp:  password = "password"; IP = 102,168.4.2
//#ifndef USE_BLUETOOTH
	printf("Initialising Wifi...\r\n");
	wifi_init_sta();
	printf("Wifi initialised!\r\n");
//#endif
	wifi_scan_timer = 0;
	wifi_scan_flag = 1;			// start wifi scanning loop


/////////////////////////////
// start BlueTooth
/////////////////////////////
#ifdef USE_BLUETOOTH
	printf("Initialising BlueTooth...\r\n");

// reset WatchDog Timer for this thread...
	esp_task_wdt_reset();		

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	bt_cfg.mode = ESP_BT_MODE_BTDM;	// force dual Classic \ BLE mode...

#ifndef USE_BLUETOOTH_CLASSIC_DETECT	
// only do this if Classic mode not required...
	ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret) 
		{
        ESP_LOGI(TAG, "Bluetooth controller release classic bt memory failed: %s", esp_err_to_name(ret));
		}
	else
#endif
		{
		 if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) 
			{
			ESP_LOGI(TAG, "Bluetooth controller initialize failed: %s", esp_err_to_name(ret));
			}
		else
			{
//			if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BLE)) != ESP_OK) 
			if ((ret = esp_bt_controller_enable((esp_bt_mode_t)bt_cfg.mode)) != ESP_OK) // MUST BE same as bt_controller_init() mode!
				{
				ESP_LOGI(TAG, "Bluetooth controller enable failed: %s", esp_err_to_name(ret));
				}
			else
				{
//				xTaskCreatePinnedToCore(&bleAdvtTask, "bleAdvtTask", 2048, NULL, 5, NULL, 0);


				bleScanSetup();
				
//				xTaskCreatePinnedToCore(&hci_evt_process,"hci_evt_process", 2048, NULL, 6, NULL, 0);

				printf("BlueTooth init OK\r\n");
				}
				
#ifdef USE_BLUETOOTH_CLASSIC_DETECT
			if ((ret = esp_bluedroid_init()) != ESP_OK) 
				{
				ESP_LOGE(GAP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
				}
			else
				{
				if ((ret = esp_bluedroid_enable()) != ESP_OK) 
					{
					ESP_LOGE(GAP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));

					}
				else					
					bt_app_gap_start_up();

				printf("BlueTooth Classic init OK\r\n");
				}
#endif
				
			}
		}

    host_rcv_data_t *bt_out_data = (host_rcv_data_t *)calloc(sizeof(host_rcv_data_t),1); // Q ptr + 1 byte Q len...	//##

    if (bt_out_data == NULL) 
		{
        ESP_LOGE(TAG, "Malloc bt_out_data failed!");
        return;
		}

	if (debug_do(DBG_ALLOC_MEM))
		printf("BT out data alloc %d\n",sizeof(host_rcv_data_t));


#ifdef USE_BLUETOOTH_ADVERTISING
// set advertising power level
//	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV,ESP_PWR_LVL_P9);

	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9); 
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
	esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN ,ESP_PWR_LVL_P9);

	esp_bredr_tx_power_set(ESP_PWR_LVL_P9,ESP_PWR_LVL_P9  );

	vTaskDelay(1000 / portTICK_PERIOD_MS);
		{
		esp_power_level_t min,max;
		esp_bredr_tx_power_get(&min,&max);	
		printf("BT Power - min: %d  max: %d\n",min, max);

		int pwrAdv  = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_ADV);	
		int pwrScan = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_SCAN);
		int pwrDef  = esp_ble_tx_power_get(ESP_BLE_PWR_TYPE_DEFAULT);
		printf("BT Power: %d  %d  %d\n",pwrAdv,pwrScan,pwrDef);
		}
#endif	
////////////////////////////////////////
// set up advertising and receive scan		
////////////////////////////////////////
// MUST do this for BT receive AND \ OR BT Advertise...
#ifndef USE_BLUETOOTH_CLASSIC_DETECT
	bleAdvtTask();		// sets up advertising and receive scan...
#endif

//while (1)
	{
	unsigned char n;
	
	for (n=0;n<50;n++)
		{
// reset WatchDog Timer for this thread...
		esp_task_wdt_reset();		
		vTaskDelay(10 / portTICK_PERIOD_MS);
		}

//	bleAdvtTask();

	}
#endif


// reset WatchDog Timer for this thread...
	esp_task_wdt_reset();		

///////////////////////
// start webserver
///////////////////////
#ifdef USE_WEBSERVER

#ifdef CONFIG_EXAMPLE_CONNECT_WIFI
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &http_server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &http_server));
#endif // CONFIG_EXAMPLE_CONNECT_WIFI

	http_server = start_webserver();
	
//	printf("Username: %s\n",CONFIG_EXAMPLE_BASIC_AUTH_USERNAME);
//	printf("Password: %s\n",CONFIG_EXAMPLE_BASIC_AUTH_PASSWORD);
#endif

// check if wifi connecetd...
    EventGroupHandle_t s_wifi_event_group = xEventGroupCreate();

#ifdef USE_WIFI_MQTT
// wifi connection diagnostics...
//#define WIFI_CONNECTED_BIT BIT0
//#define WIFI_FAIL_BIT      BIT1

printf("Checking Wifi connection...\n");
	
EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            20 / portTICK_PERIOD_MS);	//portMAX_DELAY);

printf("Result: ");
    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) 
		{
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_STA_PREFIX, WIFI_STA_PASSWORD);
		} 
	else if (bits & WIFI_FAIL_BIT) 
		{
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_STA_PREFIX, WIFI_STA_PASSWORD);
		} 
	else 
		{
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
		}
#endif		

#ifdef USE_LORA_LINK
///////////////////////////
// set up for LoRa link
///////////////////////////
// set up SPI bus for LoRa module
// set up and leave in receive state...
//SX1276_init(spi_device_handle_t spi, unsigned char *init_array);
//SX1276_set_868(spi_device_handle_t spi);
//SX1276_set_freq(spi_device_handle_t spi, unsigned long freq, unsigned char no_lora_check);

#if 0
if(debug_do(DBG_LORA))
	{
	printf("LoRa Link Init\n");
	}
	
SX1276_calibrate_rx(spi);
	
// now device is reset, perform register initialisation
SX1276_init(spi,SX1276_init_vals);		// init for Rx; need to invert IQ for GW Tx...
//SX1276_set_868(spi);

SX1276_set_lora(spi);

SX1276_set_freq(spi,868400000, 0);

SX1276_set_rx(spi);
#endif

{
esp_err_t spi_err = 0;
unsigned char x;

//spi_err = SX1276_read_reg(spi, REG_CAL_RX, &x,1);
//x = x | CAL_RX_START;
x = 0x73;	// IMPLICIT MODE
spi_err = SX1276_write_reg(spi, LREG_MDM_CFG1, &x,1);
x = LORA_PACKET_LENGTH;	//now 16	//14;	// PACKET LENGTH
spi_err = SX1276_write_reg(spi, LREG_PAYLOAD_LEN, &x,1);
}

lora_tx_timer = 0;
//lora_tx_timer_flag = 0;
#endif


///////////////////////
// set up wifi MQTT
///////////////////////
#ifdef USE_WIFI_MQTT
//    ESP_ERROR_CHECK(esp_event_loop_create_default());
//    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &http_server));

//    mqtt_app_start();
//	wifi_mqtt_start();		// now done as part of the "if (wifi_mqtt_diable_flag)..." code...

#endif


///////////////////////
// show startup msgs
///////////////////////

	printf("%c",0x0C);
	show_inf(0);

	show_options();


	start_time = esp_timer_get_time();

#ifdef GET_PARTITION_TABLE
{// test code to show partition table
esp_partition_iterator_t pt;
esp_partition_t *ptentry;
unsigned char i = 0;

//pt = esp_partition_find((esp_partition_type_t)0xFF,ESP_PARTITION_SUBTYPE_ANY,NULL);	// find first entry no matter what type it is...
pt = esp_partition_find(ESP_PARTITION_TYPE_APP,ESP_PARTITION_SUBTYPE_ANY,NULL);	// find first entry no matter what type it is...

printf(" # Type  SubType  Addr      Size      Enc  Label\n");
while(pt != NULL)
{
const esp_partition_t *ptentry = esp_partition_get(pt);	// find first entry no matter what type it is...
printf("%2d  %02X     %02X     %08X  %08X   %d   %s\n",i,ptentry->type,ptentry->subtype,ptentry->address,ptentry->size,ptentry->encrypted,ptentry->label);

pt = esp_partition_next(pt);	// find first entry no matter what type it is...
i++;
}
esp_partition_iterator_release(pt);

pt = esp_partition_find(ESP_PARTITION_TYPE_DATA,ESP_PARTITION_SUBTYPE_ANY,NULL);	// find first entry no matter what type it is...

while(pt != NULL)
{
const esp_partition_t *ptentry = esp_partition_get(pt);	// find first entry no matter what type it is...
printf("%2d  %02X     %02X     %08X  %08X   %d   %s\n",i,ptentry->type,ptentry->subtype,ptentry->address,ptentry->size,ptentry->encrypted,ptentry->label);

pt = esp_partition_next(pt);	// find first entry no matter what type it is...
i++;
}
esp_partition_iterator_release(pt);

printf("\n");
}

{
	const void *mmap_ptr;
//	esp_partition_mmap_handle_t *mmap_handle;
	spi_flash_mmap_handle_t *mmap_handle;
	esp_partition_iterator_t pt;
//	esp_partition_t *ptentry;
	const esp_partition_t *cert_space_pt;
	char pbuf[20];
	unsigned int cert_name_pos, server_cert_pos, client_cert_pos, client_key_pos;
	unsigned int cert_name_len, server_cert_len, client_cert_len, client_key_len;
	
	pt = esp_partition_find(ESP_PARTITION_TYPE_DATA,ESP_PARTITION_SUBTYPE_ANY,"cert_space");	// find first entry no matter what type it is...

	if (pt != NULL)
		{	
		cert_space_pt = esp_partition_get(pt);
		printf("found %s\n",cert_space_pt->label);

		esp_partition_read(cert_space_pt,(size_t)0,pbuf,16);
		for (i=0;i<16;i++)
			{
			printf("%02X ",pbuf[i]);
			}
		printf("\n");

		server_cert_pos = pbuf[0] + 256*pbuf[1];
		server_cert_len = pbuf[2] + 256*pbuf[3];
		client_cert_pos = pbuf[4] + 256*pbuf[5];
		client_cert_len = pbuf[6] + 256*pbuf[7];
		client_key_pos  = pbuf[8] + 256*pbuf[9];
		client_key_len  = pbuf[10] + 256*pbuf[11];
		cert_name_pos   = pbuf[12] + 256*pbuf[13];
		cert_name_len   = pbuf[14] + 256*pbuf[15];

		esp_partition_read(cert_space_pt,(size_t)server_cert_pos,pbuf,16);
		pbuf[15] = 0;
		printf("%s\n",pbuf);
		esp_partition_read(cert_space_pt,(size_t)client_cert_pos,pbuf,16);
		pbuf[15] = 0;
		printf("%s\n",pbuf);
		esp_partition_read(cert_space_pt,(size_t)client_key_pos,pbuf,16);
		pbuf[15] = 0;
		printf("%s\n",pbuf);
		esp_partition_read(cert_space_pt,(size_t)cert_name_pos,pbuf,16);
		pbuf[15] = 0;
		printf("%s\n",pbuf);
		
		}
	else
		{
		printf("Cert_space not found\n");
		}

	
	esp_partition_iterator_release(pt);
	
// esp_partition_mmap(pt,0,1024,0x10000,&mmap_ptr,&mmap_handle);
// then use databyte = *mmap_pr++;
//	esp_partition_munmap(mmap_handle);

// or dont mmap, but 
//	esp_err_t esp_partition_read(const esp_partition_t *partition, size_t src_offset, void *dst, size_t size)
//	esp_err_t esp_partition_read(pt, i, &databyte, 1);

}
#endif

// FLASH test...
#if 0
{
// test code to print contents of locations in FLASH memory
//
// See ESP32 memory map.doc in trunk folder
//
// external SPI Flash is at 0x3F40_0000 - 0x3F7F_FFFF
//
// cert files are stored in external SPI!
//
// last used rodata location: 0x3f4190c4 (see MAC_Rpt.map file)
// SPI RAM top (4MB) = 3F7F_FFFF
// 16K cert space - start at; 3F7C_0000

extern const unsigned int ca_cert_size;	// = 2162;
extern const unsigned char ca_cert_data[];	//2163] = 

extern const unsigned int client_cert_size;	// = 1670;
extern const unsigned char client_cert_data[];	//1671] = 

extern const unsigned int client_key_size;		// 1679	// defined in client_key.h
extern const unsigned char client_key_data[];	//1680];// defined in client_key.h

printf("Location of CA_cert Size:      %08lX:   %04x [%d]\n",(long)&ca_cert_size,ca_cert_size,ca_cert_size);
printf("Location of CA cert Array:     %08lX:\n",(long)&ca_cert_data[0]);

printf("Location of Client_cert Size:  %08lX:   %04x [%d]\n",(long)&client_cert_size,client_cert_size,client_cert_size);
printf("Location of Client cert Array: %08lX:\n",(long)&client_cert_data[0]);

printf("Location of Client_key Size:   %08lX:   %04x [%d]\n",(long)&client_key_size,client_key_size,client_key_size);
printf("Location of Client key Array:  %08lX:\n",(long)&client_key_data[0]);


//printf("Contents of Location %04x [%d]:\n\n",client_key_size,client_key_size);
print_hex((unsigned char *)&client_key_data[0], 256, 1, 1);


#if 0
while (1)
	{// keep watchdog and system happy while in loop
		
// reset WatchDog Timer for this thread...
	esp_task_wdt_reset();		
// yield to OS for a while (required)
	vTaskDelay(100 / portTICK_PERIOD_MS);
	}
#endif
}
#endif

#ifdef USE_I2C
check_i2c_devices_present();

// I2C sensor initialisation
{
unsigned char tmp[16];

#ifdef USE_I2C_AMBIMATE4	
if (check_i2c_device(0x2A,0x00)== ESP_OK)
	{
	printf("I2C present @ 0x2A: AmbiMate4\n");

// start measurement:
	tmp[0] = 0x7F;
	i2c_master_write_slave(I2C_MASTER_NUM,0x2A,0xC0,tmp,1);
	}
#endif// end of "#define USE_I2C_AMBIMATE4

#ifdef USE_I2C_PIMORONI
// BME688: gas, pressure, temperature & humidity
// I2C addr 76 \ 77: reg D0 = 0x61 (id)
if (check_i2c_device(0x76,0x00)== ESP_OK)
	{
//	if (i2c_read(
// get values:
	}


// BME280: temp, pressure, humidity
// I2C addr 76 \ 77: reg D0 = 0x60 (id)
if (check_i2c_device(0x76,0x00)== ESP_OK)
	{
// get values:
	}
	
// PIM376/LSM303D: BDOF Motion Sensor
// I2C addr 1D \ 1E
if (check_i2c_device(0x1D,0x00)== ESP_OK)
	{
// initiaisation
	tmp[0] = 0x80;	// enable temperature readings
	i2c_master_write_slave(I2C_MASTER_NUM,0x1D,0x24,tmp,1);

	tmp[0] = 0x00;	// enable continuous readings
	i2c_master_write_slave(I2C_MASTER_NUM,0x1D,0x26,tmp,1);
	}

// PIM413/LTR559: Light & proximity sensor
// I2C addr 23
if (check_i2c_device(0x23,0x00)== ESP_OK)
	{
// initiaisation
	tmp[0] = 0x01;
	i2c_master_write_slave(I2C_MASTER_NUM,0x23,0x80,tmp,1);

	tmp[0] = 0x03;
	i2c_master_write_slave(I2C_MASTER_NUM,0x23,0x81,tmp,1);

	}
#endif	// end of "#ifdef USE_I2C_PIMORONI...

}
#endif	// end of "#ifdef USE_I2C...

#ifdef USE_VTASKLIST			
vTaskList(vtaskList);
ESP_LOGW("MAC Rpt Tasks A:","%s",vtaskList);			
#endif
		
		

//	printf("Sizeof INT: %d\n",sizeof(int));
	
/////////////////////////
// start LORA
/////////////////////////
#ifdef LORA_USED	//_LORA
if(debug_do(DBG_LORA))
	{
	printf("LoRa Used Init\n");
	}

//    host_rcv_data_t *lora_rx_data_wr = (host_rcv_data_t *)malloc(sizeof(host_rcv_data_t)); // Q ptr + 1 byte Q len...
    lora_rx_data_wr = (host_rcv_data_t *)malloc(sizeof(host_rcv_data_t)); // Q ptr + 1 byte Q len...
    if (lora_rx_data_wr == NULL) 
		{
        ESP_LOGE(TAG, "Malloc lora_rx_data_wr failed!");
        return;
		}

	if (debug_do(DBG_ALLOC_MEM))
		printf("LoRa Rx wr data alloc %d\n",sizeof(host_rcv_data_t));

    host_rcv_data_t *lora_rx_data_rd = (host_rcv_data_t *)malloc(sizeof(host_rcv_data_t)); // Q ptr + 1 byte Q len...
    if (lora_rx_data_rd == NULL) 
		{
        ESP_LOGE(TAG, "Malloc lora_rx_data_rd failed!");
        return;
		}

	if (debug_do(DBG_ALLOC_MEM))
		printf("Lora Rx rd data alloc %d\n",sizeof(host_rcv_data_t));

SX1276_reset();
// above only starts the reset; wait for the completion flag...
while (!lora_reset_complete_flag)
	{
	if(debug_do(DBG_LORA))
		printf("LORA reset wait... %d  %d\n",x100msec, lora_reset_timer);

	if (lora_reset_timer <= (LORA_RESET_TIME+1))
		lora_reset_timer++;
		
	if (lora_reset_timer == LORA_RESET_TIME)
// on shift reg pin on V1_0_D, so must use set_output function...
		{
		set_output(DEV_LORA_RADIO_RESET,1,0);	// 
		printf("...LORA reset high...\n");
		}
// SX2176 is ready to use after 10ms, but allow 100...
	if (lora_reset_timer == (LORA_RESET_TIME+1))
		{	
		lora_reset_complete_flag = 1;
		printf("...LORA reset end\n");
		}



// reset WatchDog Timer for this thread...
	esp_task_wdt_reset();		
// yield to OS for a while (required)
	vTaskDelay(100 / portTICK_PERIOD_MS);
	}
	
SX1276_calibrate_rx(spi);
	
// now device is reset, perform register initialisation
SX1276_init(spi,SX1276_init_vals);		// init for Rx; need to invert IQ for GW Tx...
//SX1276_set_868(spi);

/*
SX1276_set_idle(spi);

SX1276_get_mode(spi,&i);
printf("LoRa Mode = %d\n",i);

SX1276_set_lora(spi);

SX1276_get_mode(spi,&i);
printf("LoRa Mode = %d\n",i);
*/

SX1276_set_freq(spi,868400000, 0);
SX1276_set_rx(spi);

{
// for single freq setup: on LHT sensor:
// AT+NJM = 0			// set ABP mode
// AT+ADR = 0			// Adaptve Data Rate (0=OFF: 1 = ON)
// AT+DR = 5			// set Data Rate(0-7 = DR_X)
// AT+CHS = 868400000	// freq

unsigned char x;
#if 0
SX1276_read_reg(spi,LREG_OPMODE,&x,1);
x = x | 0x07;	// channel activity detect mode
SX1276_write_reg(spi,LREG_OPMODE,&x,1);
#endif

#if 0
SX1276_read_reg(spi,LREG_OPMODE,&x,1);
//x = (x & 0xF8) | 0x07;	// channel activity detect mode
x = (x & 0xF8) | 0x05;	// Rx_Continuous mode
SX1276_write_reg(spi,LREG_OPMODE,&x,1);
#endif

#if 0
// try setting for wider bandwidth: STOPS RECEPTION!
SX1276_read_reg(spi,LREG_MDM_CFG1,&x,1);
x = (x & 0xF0) | 0x90;		// 500khz bandwidth
SX1276_write_reg(spi,LREG_MDM_CFG1,&x,1);
#endif
 
//SX1276_set_idle(spi);
//SX1276_set_rx(spi);

}

{
esp_err_t spi_err = 0;
unsigned char x;

//spi_err = SX1276_read_reg(spi, REG_CAL_RX, &x,1);
//x = x | CAL_RX_START;
x = 0x73;	// IMPLICIT MODE
spi_err = SX1276_write_reg(spi, LREG_MDM_CFG1, &x,1);
x = LORA_PACKET_LENGTH;	//now 16	//14;	// PACKET LENGTH
spi_err = SX1276_write_reg(spi, LREG_PAYLOAD_LEN, &x,1);
}

#endif

#if 0
j = 0;
while (1)
{
unsigned char val[4] = {0, 0, 0, 0};
esp_err_t spi_err;

	val[0] = 0x80;
	spi_err = SX1276_write_reg(spi, 1,val,1);

	spi_err = SX1276_read_reg(spi,1,val,1);
	printf("A%02X:    %02X:  %02X  %02X  %02X\n",i,val[0],val[1],val[2],val[3]);

	val[0] = 0x81;
	spi_err = SX1276_write_reg(spi, 1,val,1);

	spi_err = SX1276_read_reg(spi,1,val,1);
	printf("B%02X:    %02X:  %02X  %02X  %02X\n",i,val[0],val[1],val[2],val[3]);

// reset WatchDog Timer for this thread...
	esp_task_wdt_reset();		
// yield to OS for a while (required)
	vTaskDelay(10 / portTICK_PERIOD_MS);

}


{
unsigned char reg,i;
unsigned char val[4];
esp_err_t spi_err;	
//SX1276_read_reg(spi_device_handle_t spi, unsigned char reg, unsigned char *regval, unsigned char len)

// switch mode to LoRa
reg = 0x00;		// sleep mode
spi_err = SX1276_write_reg(spi, LREG_OPMODE,&reg,1);
reg = 0x80;		// LoRa mode
spi_err = SX1276_write_reg(spi, LREG_OPMODE,&reg,1);

#if 0
for (i=0;i<0x40;i++)
	{
	spi_err = SX1276_read_reg(spi,i,&reg,1);
//	spi_device_get_trans_result(spi,
	printf("%02X:  %02X  %04X\n",i,reg,spi_err);
	}
	
	reg = j;
	spi_err = SX1276_write_reg(spi,0x07,&reg,1);

	spi_err = SX1276_read_reg(spi,0x07,&reg,1);

	printf("Reg 0x07:   %02X  [%02X]   %04X\n",reg,j,spi_err);

	j++;
#endif

for (i=0;i<0x40;i++)
	{
	spi_err = SX1276_read_reg(spi,i,val,4);
//	spi_device_get_trans_result(spi,
	printf("%02X:    %02X:  %02X  %02X  %02X\n",i,val[0],val[1],val[2],val[3]);
	}
	
}

// reset WatchDog Timer for this thread...
	esp_task_wdt_reset();		
// yield to OS for a while (required)
	vTaskDelay(10 / portTICK_PERIOD_MS);


#endif

// set LED states \ uses
#ifdef HEARTBEAT_LED
set_led(HEARTBEAT_LED, FLASH);
#else
set_led(0, FLASH);
#endif

#ifdef NETWORK_LED
set_led(NETWORK_LED, DEVICE_OFF);
#else
set_led(1, DEVICE_OFF);
#endif

set_led(2, DEVICE_OFF);
set_led(3, DEVICE_OFF);


///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
////
////  MAIN LOOP
////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
	while (1)
		{		
//////////////////////////////////////////////////////////

//////////////////////////////
// Set MQTT transport mode:
//////////////////////////////

// if 4G is preference, or WIFI not available:
		if((mqtt_transport_mode != MQTT_MODE_4G) && (mqtt_state > MQTT_CONNECT_CHECK) && 			// if not 4G and 4G is available
		   ((mqtt_preferred_transport_mode == MQTT_MODE_4G) || 											// if 4G preferred
		   (wifi_mqtt_connected == 0))																// or wifi not available
		  )
			{
//			if (mqtt_state == MQTT_CONNECT_IDLE)
				{
				printf("MQTT MODE changed to 4G  [%d %d]\n",mqtt_transport_mode,MQTT_MODE_4G);
				mqtt_transport_mode = MQTT_MODE_4G;
				}
			}			

// if WIFI is preference or 4G not available:
		else if((mqtt_transport_mode != MQTT_MODE_WIFI) && (wifi_mqtt_connected) &&					// if mot WIFI and WIFI available
		    (((mqtt_preferred_transport_mode == MQTT_MODE_WIFI) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// if wifi preferred (and 4G MQTT is between transactions)
			(mqtt_state <= MQTT_CONNECT_CHECK) || 													// or 4G not connected 
			(simcom_state == SIMCOM_NONE_FOUND))													// or no SIMCOM model detected
		  )
			{
//			if (mqtt_state == MQTT_CONNECT_IDLE)
				{
				printf("MQTT MODE changed to WIFI [%d %d]\n",mqtt_transport_mode,MQTT_MODE_WIFI);
				mqtt_transport_mode = MQTT_MODE_WIFI;
				}
			
			}
		else if ((mqtt_state <= MQTT_CONNECT_CHECK) && (wifi_mqtt_connected == 9))					// if neither 4G or WIFI available
			mqtt_transport_mode = MQTT_MODE_NONE;
						

//////////////////////////////
// Set UDP transport mode:
//////////////////////////////
#ifdef USE_UDP
// if 4G is preference, or WIFI not available:
		if((udp_transport_mode != UDP_MODE_4G) && (mqtt_state > MQTT_CONNECT_CHECK) && 			// if not 4G and 4G is available
		   ((udp_preferred_transport_mode == UDP_MODE_4G) || 											// if 4G preferred
		   (wifi_udp_connected == 0))																// or wifi not available
		  )
			{
//			if (mqtt_state == MQTT_CONNECT_IDLE)
				{
				printf("UDP MODE changed to 4G  [%d %d]\n",udp_transport_mode,UDP_MODE_4G);
				udp_transport_mode = UDP_MODE_4G;
				}
			}			

// if WIFI is preference or 4G not available:
		else if((udp_transport_mode != UDP_MODE_WIFI) && (wifi_udp_connected) &&					// if mot WIFI and WIFI available
		    (((udp_preferred_transport_mode == UDP_MODE_WIFI) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// if wifi preferred (and 4G MQTT is between transactions)
			(mqtt_state <= MQTT_CONNECT_CHECK) || 													// or 4G not connected 
			(simcom_state == SIMCOM_NONE_FOUND))													// or no SIMCOM model detected
		  )
			{
//			if (mqtt_state == MQTT_CONNECT_IDLE)
				{
				printf("UDP MODE changed to WIFI [%d %d]\n",udp_transport_mode,UDP_MODE_WIFI);
				udp_transport_mode = UDP_MODE_WIFI;
				}
			
			}
		else if ((mqtt_state <= MQTT_CONNECT_CHECK) && (wifi_mqtt_connected == 9))					// if neither 4G or WIFI available
			udp_transport_mode = UDP_MODE_NONE;

#endif 	// end of "#ifdef USE_UDP..."						


// enable enqueuing of MQTT data only if MQTT is active
		queues_enabled_flag = 0;
#ifdef USE_MQTT
		if ((transport_mode == MQTT_MODE_4G) && (mqtt_state > MQTT_CONNECT_CHECK) && (mqtt_state < MQTT_COMMS_INIT_ERROR_END))
			queues_enabled_flag = 1;
		else
			
#endif
#ifdef USE_UDP
		if ((udp_state >UDP_COMM_IDLE_2) && (udp_state >UDP_SOCKET_CLOSE))
			queues_enabled_flag = 1;
#endif
		
#ifdef USE_WIFI_MQTT	// NEEDS TO BE FIXED!
		if ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected))
			queues_enabled_flag = 1;
#endif

#ifdef USE_WIFI_UDP
// resolve UDP URL using DNS
		if ((!prev_wifi_udp_DNSfound) && (wifi_udp_DNSfound))
			{
			printf("WIFI UDP - GOT DNS\n");
//			wifi_udp_DNSstart = 0;
			
// socket info
//https://github.com/espressif/esp-idf/blob/b4268c874a4cf8fcf7c0c4153cffb76ad2ddda4e/examples/protocols/sockets/udp_client/main/udp_client.c

// s_addr = TYPE IN_ADDR: https://www.gta.ufrj.br/ensino/eel878/sockets/sockaddr_inman.html
// wifi_udp_ip_addr = IP_ADDR_T (4 bytes ?)
//			dest_addr.sin_addr.s_addr = inet_addr(wifi_udp_ip_addr);
//			dest_addr.sin_addr.s_addr = inet_toa(wifi_udp_ip_addr));	// s_addr is ulong: addr = ip_addr_t
			inet_aton(inet_ntoa(wifi_udp_ip_addr), &dest_addr.sin_addr.s_addr);
			dest_addr.sin_family = AF_INET;
			dest_addr.sin_port = htons(wifi_udp_port);
			wifi_udp_addr_family = AF_INET;
			wifi_udp_ip_protocol = IPPROTO_IP;

// if socket previously opened, close it before re-opening...
            if (wifi_udp_sock >=0)
				{
				printf("Refreshing UDP socket...\n");
				shutdown(wifi_udp_sock, 0);
				close(wifi_udp_sock);
				}

	        wifi_udp_sock = socket(wifi_udp_addr_family, SOCK_DGRAM, wifi_udp_ip_protocol);
			if (wifi_udp_sock < 0) 
				{				
				ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
				break;
				}
// Set timeout
			struct timeval wifi_udp_timeout;
			wifi_udp_timeout.tv_sec = 10;
			wifi_udp_timeout.tv_usec = 0;
			setsockopt (wifi_udp_sock, SOL_SOCKET, SO_RCVTIMEO, &wifi_udp_timeout, sizeof wifi_udp_timeout);

			ESP_LOGI(TAG, "wifi udp socket created, destination %s:%d", inet_ntoa(wifi_udp_ip_addr), wifi_udp_port);

			wifi_udp_connected = 1;
			}
			
		prev_wifi_udp_DNSfound = wifi_udp_DNSfound;
		
		
#endif


#ifdef CTS2_TEST
if (CTS2_int_flag != 0xFF)
	{
	printf("     ### CTS2 Int %d!\n",CTS2_int_flag);
	CTS2_int_flag = 0xFF;
	}
#endif

//	printf("start while(1)\n");



// run LoRa code here for faster response:
#ifdef LORA_USED	//_LORA
#if 1
{
uint32_t io_num;
char dio_str[16];
unsigned char x,y;


//LORA_int_flag = 0;

//printf("LORA Q\n");
//while ((xQueueReceive(lora_gpio_queue, &io_num, 0))==pdTRUE)
#if 1
	while ((xQueueReceive(lora_gpio_queue, &io_num, 0))==pdTRUE)
		{
		LORA_int_flag |= 2;

		if(debug_do(DBG_LORA_INT_FLAGS))
			printf("### GPIO[%d]: %d\n",io_num,gpio_get_level((gpio_num_t)io_num));
		}
#endif
	{
	strcpy(dio_str,"DIOx ");
    strcat(dio_str,"0 0 0 0 0");
/*
	if (io_num ==36)
		dio_str[3] = '0';
	else if (io_num == 39)
		dio_str[3] = '1';
	else if (io_num == 27)
		dio_str[3] = '2';
	else if (io_num == 4)
		dio_str[3] = '3';
	else if (io_num == 32)
		dio_str[3] = '4';
	else 
		strcpy(dio_str,"????");
*/
	y = 0;
	x = gpio_get_level(LORA_DIO0);
	if (x)
		{
		dio_str[13] = '1';
		y = y | 1;
		}
	x = gpio_get_level(LORA_DIO1);
	if (x)
		{
		dio_str[11] = '1';
		y = y | 2;
		}
	x = gpio_get_level(LORA_DIO2);
	if (x)
		{
		dio_str[9] = '1';
		y = y | 4;
		}
	x = gpio_get_level(LORA_DIO3);
	if (x)
		{
		dio_str[7] = '1';
		y = y | 8;
		}
	x = gpio_get_level(LORA_DIO4);
	if (x)
		{
		dio_str[5] = '1';
		y = y | 16;
		}
	
	if (prev_dio_val != y)
		{
		if(debug_do(DBG_LORA_INT_FLAGS))
			{
//			printf("GPIO[%d] / %s intr, val: %d\n", io_num, dio_str, gpio_get_level((gpio_num_t)io_num));
			printf("###############\n");
			printf("### DIO[0x%02X]  %s \n", y,dio_str);
			printf("###############\n");
			}

		prev_dio_val = y;
		LORA_int_flag |= 1;
		}
		
	if (LORA_int_flag)
		{
//		printf("entering rcv\n");
		lora_rcv();
//		printf("left rcv\n");
		x = 0xFF;	// need to write a '1' to clear an int bit...
		SX1276_write_reg(spi,LREG_IRQ_FLAGS,&x,1);

		LORA_int_flag = 0;

		}

	}	

//	printf("end Lora code\n");
//	ets_delay_us(1000);
}
#endif
#endif


// end of LoRa code
//////////////////////////////////////////////////////////
#ifdef TEST_I2C	
{
unsigned char gp_timer = 0;

while(1)
	{
	
	if (x100msec_flag)
		{
		x100msec_flag = 0;
#if 0
		i = gpio_get_level(SDA);	//(gpio_num_t)25);
		j = gpio_get_level(SCL);	//(gpio_num_t)26);
		printf("GPIO: %d %d\n",i,j);
#else
		gpio_pad_select_gpio(SDA);
		gpio_set_direction(SDA, GPIO_MODE_OUTPUT);
		gpio_pad_select_gpio(SCL);
		gpio_set_direction(SCL, GPIO_MODE_OUTPUT);
		
		if (gp_timer == 20)
			{
			printf ("GPIO 1  0\n");
			gpio_set_level(SDA,1);
			gpio_set_level(SCL,0);
			}

		if (gp_timer == 40)
			{
			printf ("GPIO 0  1\n");
			gpio_set_level(SDA,0);
			gpio_set_level(SCL,1);
			gp_timer = 0;
			}
			
		gp_timer++;
		
#endif
		}

// reset WatchDog Timer for this thread...
	esp_task_wdt_reset();		
// yield to OS for a while (required)
	vTaskDelay(100 / portTICK_PERIOD_MS);
	
	}
}
#endif	// end of "#ifdef TEST_I2C...
	
		if (x100msec_flag)
		{
		
		x100msec_flag = 0;

//	printf("x100ms\n");
//	ets_delay_us(1000);


// increment 100msec timers
/*
		if (four_g_state_mc_timer < 0xFFFF)
			four_g_state_mc_timer++;

		if (four_g_state_mc_timer%10 == 0)
			four_g_sm_timer_flag = 1;
*/
		if ((four_g_timeout_timer < 0xFFFF) && (four_g_timeout_timer > 0))	// 0xFFFF inhibits timeout
			four_g_timeout_timer--;

		if (simcom_state_mc_timer < 0xFFFF)
			simcom_state_mc_timer++;

		if (simcom_state_mc_timer%10 == 0)
			simcom_sm_timer_flag = 1;

		if ((simcom_timeout_timer < 0xFFFF) && (simcom_timeout_timer > 0))	// 0xFFFF inhibits timeout
			simcom_timeout_timer--;

		if ((mqtt_timeout_timer < 0xFFFF) && (mqtt_timeout_timer > 0))	// 0xFFFF inhibits timeout
			mqtt_timeout_timer--;

		if ((udp_timeout_timer < 0xFFFF) && (udp_timeout_timer > 0))	// 0xFFFF inhibits timeout
			udp_timeout_timer--;

#ifdef USE_WIFI_UDP
		if (wifi_udp_DNS_timer < 0xFF)
			wifi_udp_DNS_timer++;

#endif

/*
		if (mqtt_state_mc_timer < 0xFFFF)
			mqtt_state_mc_timer++;

		if (mqtt_state_mc_timer%10 == 0)
			mqtt_sm_timer_flag = 1;

		if (udp_state_mc_timer < 0xFFFF)
			udp_state_mc_timer++;

		if (udp_state_mc_timer%10 == 0)
			udp_sm_timer_flag = 1;
*/
#ifdef XXX_USE_LORA
		printf("LORA count...\n");
		if (lora_reset_timer <= (LORA_RESET_TIME+1))
			lora_reset_timer++;
		
		if (lora_reset_timer == LORA_RESET_TIME)
// on shift reg pin on V1_0_D, so must use set_output function...
			{
			set_output(DEV_LORA_RADIO_RESET,1,0);	// 
			printf("...LORA reset high...\n");
			}
// SX2176 is ready to use after 10ms, but allow 100...
		if (lora_reset_timer == (LORA_RESET_TIME+1))
			{	
			lora_reset_complete_flag = 1;
			printf("...LORA reset end\n");
			}
	
#endif		

#ifdef USE_LORA_LINK
		if (lora_tx_timer < 0xFF)
			lora_tx_timer++;
#endif		


#ifdef USE_ULTRASONIC_SENSOR
		if (usm_trigger_count >= USM_TRIGGER_TIME)
			usm_trigger_flag = 1;
		else
			usm_trigger_count++;

		if (usm_timeout_count < 0xFFFF)
			usm_timeout_count++;
	
#endif

#ifdef USE_ZULU_RADIO
		if (zulu_radio_status_timer < 0xFFFF)
			zulu_radio_status_timer++;

		if (zulu_radio_hb_timer < 0xFFFF)
			zulu_radio_hb_timer++;

		if (zulu_radio_timeout_timer < 0xFFFF)
			zulu_radio_timeout_timer++;

		if (zulu_radio_run_timer < 0xFFFF)
			zulu_radio_run_timer++;
#endif


// end of "increment 100msec timers"

		if (x1sec_flag)
		{
		x1sec_flag = 0;

#if 0
// test "get server time" function...
		if (mqtt_login_state > UPDATE_SRVR_INFO_WAIT)	// if connected and ready to send
			{
			if (secs%10 == 0)
				{
				if (get_server_time_flag == 0)
					{
					printf("Setting srvr test flag...\n");
					srvr_test_run_flag = 1;
					get_server_time_flag = 1;
					test_server_time_flag = 0;
					}
				}
			if ((secs%10 == 9) && (srvr_test_run_flag))	// ensure the 0 sec part runs first...
				{
				srvr_test_run_flag = 0;					// ensure the 0 sec part runs first...
				
				printf("Srvr time test: ");
				srvr_test_count++;	
				if (test_server_time_flag)
					{
					srvr_test_OK_count++;
					test_server_time_flag = 0;
					printf("OK!");
					}
				else
					printf("ERR!");

				printf(" [%d %d]\n",srvr_test_count,srvr_test_OK_count);
				}
			}
#endif

			
// increment 1sec timers
#ifdef USE_M4G_MQTT
		if (mqtt_sensor_msg_timer < MQTT_SENSOR_TIME)
			{
			mqtt_sensor_msg_timer++;
			}
		else
			{
			mqtt_sensor_msg_flag = 1;
			mqtt_sensor_msg_timer = 0;
			}
#endif
// increment sensor timers
#ifdef USE_BLUETOOTH
		if (bluetooth_sensor_timer < 0xFFFF)
			bluetooth_sensor_timer++;

#ifdef BLUETOOTH_SCAN
		if (bluetooth_MAC_addr_timer < 0xFFFF)
			bluetooth_MAC_addr_timer++;
#endif		
		if (bluetooth_devname_timer < 0xFFFF)
			bluetooth_devname_timer++;

#ifdef USE_BLUETOOTH_CLASSIC_DETECT
		if (bluetooth_classic_scan_timer < 0xFFFF)
			bluetooth_classic_scan_timer++;
#endif		

#endif		// end of "#ifdef USE_BLUETOOTH..."


#ifdef LORAWAN_SCAN
		if (lora_devaddr_timer < 0xFFFF)
			lora_devaddr_timer++;
#endif		
#ifdef WIFI_SCAN
		if (wifi_scan_timer < wifi_scan_time)
			wifi_scan_timer++;
#endif		
		
		if (gateway_sensor_timer < 0xFFFF)
			gateway_sensor_timer++;
		if (gps_sensor_timer < 0xFFFF)
			gps_sensor_timer++;
#ifdef USE_I2C
		if (i2c_sensor_timer < 0xFFFF)
			i2c_sensor_timer++;
#endif
#ifdef USE_TANK_SENSORS
		if (tank_sensor_timer < 0xFFFF)
			tank_sensor_timer++;
#endif





#ifdef USE_BLUETOOTH_CLASSIC_DETECT
		if (bluetooth_classic_scan_timer >= bluetooth_classic_scan_time)
			{
			printf("*** BT Classic Scan - discovery started... duration %d sec\n",BLUETOOTH_CLASSIC_SCAN_LENGTH*128/100);
			bt_start_discovery(BLUETOOTH_CLASSIC_SCAN_LENGTH);
			bluetooth_classic_scan_timer = 0;
			}
#endif


//	printf("## CHR %02X: %02X %02X\n",CMQTT_rx_flag,pattern_detect_char,eolstr[0]);
	
		if (secs == 0)		// ie once per minute
			{
			unsigned char running_events = 0;
			

			}		// end of "if (secs == 0)..."

		if (secs == 30)		// check UVC detect status and temperature via serial msg
			{

			}

		}
// end of "increment 1sec timers"


#ifdef USE_TANK_SENSORS
		gpio_set_level(TSENSCTRL,0);			// set tank sensor power ON
#endif

//#define SHOW_LOOPTIME
#ifdef SHOW_LOOPTIME
	end_time = esp_timer_get_time();
	loop_time = end_time - start_time;
	printf("Loop Time: %lluusec\r\n",loop_time);
	start_time = end_time;
#endif	
//	show_time(DBG,1);	

#ifdef USE_CPU_SLEEP
	if (uart_stay_awake_count)
		{
		uart_stay_awake_count--;
		if (uart_stay_awake_count==0)
		printf("Serial Session End\r\n");
		}
#endif

#ifdef USE_CPU_SLEEP
	if (sleep_flag)
		{
#ifdef SHOW_SLEEP_MSGS
		if(debug_do(DBG_SLEEP))
			{
			printf("...Wake\r\n");
			}
#endif		
		sleep_flag = 0;
		}
#endif
	
	sleep_resume_flag = 0;

#ifdef USE_SWITCHES
// check user sw lines on J10
// NOTE active LOW!
	j = gpio_get_level(SW0);
	sw_status = (~j & 0x01);

	j = gpio_get_level(SW1);
	sw_status = sw_status | ((~j & 0x01) << 1);

	j = gpio_get_level(SW2);
	sw_status = sw_status | ((~j & 0x01) << 2);

	if (sw_status != prev_sw_status)
		printf("SWS: %02X\r\n",sw_status);

	prev_sw_status = sw_status;
#endif

#ifdef USE_BLUETOOTH
//			bleAdvtTask();
//			scanned_count++;
#endif

#if 0
// now done by setting TIM_msg_time to 100... 
		if (x10sec_flag)
			{
			show_time(DBG,0);
			x10sec_flag = 0;
			printf("\r\n");
			}
#endif	

	if (TIM_msg_time)
		{
		if (TIM_msg_time_count < TIM_msg_time)
			TIM_msg_time_count++;
		else
			{
			printf("Time: ");
			show_time(DBG,0);
			if (cell_rssi != -127)
				printf("[%ddBm]",cell_rssi);
			else
				printf("[---]");

#if	1	// def USE_4G
			printf("    SIMCOM timer: %d",simcom_state_mc_timer);
#endif
#ifdef USE_BLUETOOTH
			printf("    BLE scan count: %d",scanned_count);
#endif
			printf("\r\n");			
			TIM_msg_time_count = 0;
			}			
		}
	else
		{
		TIM_msg_time_count = 0;	
		}

#ifdef LEDS_SEQ
//#ifndef USE_ULTRASONIC_SENSOR
#if !defined(USE_ULTRASONIC_SENSOR) && !defined(USE_ZULU_RADIO))
	led_count++;
		
	if (led_count & (1<<3))
		set_output(DEV_LED_0,DEVICE_ON,1);
	else
		set_output(DEV_LED_0,DEVICE_OFF,1);

	if (led_count & (1<<4))
		set_output(DEV_LED_1,DEVICE_ON,1);
	else
		set_output(DEV_LED_1,DEVICE_OFF,1);

	if (led_count & (1<<5))
		set_output(DEV_LED_2,DEVICE_ON,1);
	else
		set_output(DEV_LED_2,DEVICE_OFF,1);

	if (led_count & (1<<6))
		set_output(DEV_LED_3,DEVICE_ON,1);
	else
		set_output(DEV_LED_3,DEVICE_OFF,1);
	
#if 0	// ###!###
		if ((led_count & 0x0F) == (1<<3))
			printf("LEDs %02X\r\n",led_count>>3);
#endif

#endif

#if defined(USE_ZULU_RADIO)
	led_count++;

	if (led_count & (1<<3))
		set_output(DEV_LED_0,DEVICE_ON,1);
	else
		set_output(DEV_LED_0,DEVICE_OFF,1);

#endif

#else	// use individual LEDs for indicators
	
//led_modes[0] = DEVICE_OFF;
//led_modes[1] = DEVICE_ON;
//led_modes[2] = FLASH;
//led_modes[3] = FAST_FLASH;

/*
set_led(0, DEVICE_OFF);
set_led(1, DEVICE_ON);
set_led(2, FLASH);
set_led(3, FAST_FLASH);
*/
//led_0_state = 1;

//led_states = 0;	
	if (led_count == 0)		// 400ms & 800ms
		{
		if ((led_modes[0] == FLASH) || (led_modes[0] == FAST_FLASH))
			{
			if (led_states & 0x01) 
				led_states = led_states & ~(0x01);
			else
				led_states = led_states | 0x01;
				

//			led_0_state = 1 - led_0_state;
//			set_output(DEV_LED_0,led_0_state,1);
			}

		if ((led_modes[1] == FLASH) || (led_modes[1] == FAST_FLASH))
			{
			if (led_states & 0x02) 
				led_states = led_states & ~(0x02);
			else
				led_states = led_states | 0x02;

//			led_1_state = 1 - led_1_state;
//			set_output(DEV_LED_1,led_1_state,1);
			}

		if ((led_modes[2] == FLASH) || (led_modes[2] == FAST_FLASH))
			{
			if (led_states & 0x04) 
				led_states = led_states & ~(0x04);
			else
				led_states = led_states | 0x04;

//			led_2_state = 1 - led_2_state;
//			set_output(DEV_LED_2,led_2_state,1);
			}

		if ((led_modes[3] == FLASH) || (led_modes[3] == FAST_FLASH))
			{
			if (led_states & 0x08) 
				led_states = led_states & ~(0x08);
			else
				led_states = led_states | 0x08;

//			led_3_state = 1 - led_3_state;
//			set_output(DEV_LED_3,led_3_state,1);
			}
			
		}
		
	if (led_count == 4)		// 400ms
		{
		if (led_modes[0] == FAST_FLASH)
			{
			if (led_states & 0x01) 
				led_states = led_states & ~(0x01);
			else
				led_states = led_states | 0x01;

//			led_0_state = 1 - led_0_state;
//			set_output(DEV_LED_0,led_0_state,1);
			}

		if (led_modes[1] == FAST_FLASH)
			{
			if (led_states & 0x02) 
				led_states = led_states & ~(0x02);
			else
				led_states = led_states | 0x02;

//			led_1_state = 1 - led_1_state;
//			set_output(DEV_LED_1,led_1_state,1);
			}

		if (led_modes[2] == FAST_FLASH)
			{
			if (led_states & 0x04) 
				led_states = led_states & ~(0x04);
			else
				led_states = led_states | 0x04;

//			led_2_state = 1 - led_2_state;
//			set_output(DEV_LED_2,led_2_state,1);
			}

		if (led_modes[3] == FAST_FLASH)
			{
			if (led_states & 0x08) 
				led_states = led_states & ~(0x08);
			else
				led_states = led_states | 0x08;

//			led_3_state = 1 - led_3_state;
//			set_output(DEV_LED_3,led_3_state,1);
			}

		}

	if (prev_led_states != led_states)	// only shift out data if changed - time consuming!
		{
//		printf("count %d:  led_states: %02X  %02X  sd: %04X  ",led_count, led_states, (~led_states & 0x0F), sd_data_out);
//		sd_data_out = (sd_data_out & ~(0x0F << (DEV_LED_0 - SR_THRESHOLD))) | (((~led_states) & 0x0F) << (DEV_LED_0 - SR_THRESHOLD));
#if LED_POLARITY == NOT_INVERTED
//		printf("count %d:  led_states: %02X %d=N %02X  sd: %04X  ",led_count, led_states, LED_POLARITY, (led_states & 0x0F), sd_data_out);
		sd_data_out = (sd_data_out & ~(0x0F << (DEV_LED_0 - SR_THRESHOLD))) | ((led_states & 0x0F) << (DEV_LED_0 - SR_THRESHOLD));
#else
//		printf("count %d:  led_states: %02X %d=I %02X  sd: %04X  ",led_count, led_states, LED_POLARITY, (~led_states & 0x0F), sd_data_out);
		sd_data_out = (sd_data_out & ~(0x0F << (DEV_LED_0 - SR_THRESHOLD))) | ((~led_states & 0x0F) << (DEV_LED_0 - SR_THRESHOLD));
#endif
//		printf("%04X\n",sd_data_out);
		shift_out(sd_data_out);
		prev_led_states = led_states;
		}
		
	led_count++;
	if (led_count >=8)			// 800ms
		{
		led_count = 0;
		}

	
#endif	// end of "#ifdef LEDS_SEQ..."


#ifdef USE_ULTRASONIC_SENSOR

	mqtt_disable_flag = 1;

	if ((debug_do(DBG_USM)) && (usm_state != prev_usm_state))
		{
		printf("USM_STATE: %dm        int: %d\n",usm_state,usm_int_count);
		prev_usm_state = usm_state;
		}	
		
	switch (usm_state)
		{
		case USM_IDLE:	// not triggered
			if (usm_trigger_flag)
				{
				usm_trigger_count = 0;
				usm_timeout_count = 0;
				usm_echo_time = 0;
				usm_distance = 0;
// trigger the Ultrasonic sensor with a 10us pulse
				gpio_set_level(USM_TRIG,1);
				ets_delay_us(1000);
				gpio_set_level(USM_TRIG,0);

				printf("***********************\n");
				printf("* Start Of USM Reading\n");
				printf("***********************\n");
//				printf("USM trig!\n");

// result will be read by interrupt routine, or timeout
				usm_int_count = 0;
				gpio_intr_enable(USM_ECHO);

				usm_trigger_flag = 0;
				usm_state = USM_TRIGGERED_WAITING;
				}
			break;
		case USM_TRIGGERED_WAITING:	// triggered, waiting
			if (usm_timeout_count >= USM_TIMEOUT)				
				{
				usm_state = USM_EXCEEDED_TIMEOUT;
				}
			break;
		case USM_COMPLETED:	// completed ok
			gpio_intr_disable(USM_ECHO);
			usm_distance = usm_echo_time * 1 / USM_COEFF;
			printf("USM OK: %d cm   ave: %dcm   max: %dcm    ave max: %dcm    [%d]\n",usm_distance,usm_ave_distance,usm_max,usm_ave_max,usm_ave_count);
			printf("***********************\n");
			printf("Start: %lld  stop: %lld  time: %lldus\n",usm_start_time, usm_end_time,usm_end_time - usm_start_time);
			usm_completed_ok_flag = 1;
			usm_state = USM_IDLE;			
			break;
		case USM_EXCEEDED_TIMEOUT:	// timed out waiting for return echo 
			gpio_intr_disable(USM_ECHO);
			printf("USM TO!\n");
			usm_state = USM_IDLE;			
			break;
		default:	// all other values not valid
			usm_state = USM_IDLE;
			break;
		}
		
	if ((debug_do(DBG_USM)) && (usm_state != prev_usm_state))
		{
		printf("USM_STATE: %d        int: %d\n",usm_state,usm_int_count);
		prev_usm_state = usm_state;
		}	
		
	if (usm_completed_ok_flag)
		{
		usm_ave_count++;
		usm_accum = usm_accum + usm_distance;
//		printf("Accum: %d\n",usm_accum);
		
		if (usm_ave_count >= USM_AVE)
			{
			
			usm_ave_distance = usm_accum / USM_AVE;
			usm_accum = 0;
			usm_ave_count = 0;

// get the longest distance	
			usm_ave_max = ((usm_ave_max * (USM_AVE-1)) + usm_ave_distance) / USM_AVE;
			
			if (usm_ave_distance > usm_ave_max)
				usm_max = usm_ave_distance;
			
//			printf("Current max: %dcm\n",usm_max);

			if (usm_ave_distance >= (usm_ave_max *95/100))
				{
				usm_cabin_empty_flag = 1;
				set_output(DEV_LED_1,DEVICE_ON,1);
				}
			else
				{
				usm_cabin_empty_flag = 0;
				set_output(DEV_LED_1,DEVICE_OFF,1);
				}
			
			}
//		
//		usm_completed_ok_flag = 0;
		}

// check conditions - always done on every pass of the foreground loop...
			
			
		if (usm_completed_ok_flag)		// only show cleaning status after every USM measurement
			{
			printf("USM empty: %d    UV clean: %d\n",usm_cabin_empty_flag,UV_clean_trigger_flag);
			usm_completed_ok_flag = 0;
			}

#endif


#ifdef LORA_USED	//_LORA
#if 0
{
uint32_t io_num;
char dio_str[16];
unsigned char x,y;

LORA_int_flag = 0;

//printf("LORA Q\n");
//while ((xQueueReceive(lora_gpio_queue, &io_num, 0))==pdTRUE)
	while ((xQueueReceive(lora_gpio_queue, &io_num, 0))==pdTRUE)
		{
//		printf("### GPIO[%d]: %d\n",io_num,gpio_get_level((gpio_num_t)io_num));
		LORA_int_flag = 1;
		}
	{
	strcpy(dio_str,"DIOx");
    strcat(dio_str,"0 0 0 0 0");
/*
	if (io_num ==36)
		dio_str[3] = '0';
	else if (io_num == 39)
		dio_str[3] = '1';
	else if (io_num == 27)
		dio_str[3] = '2';
	else if (io_num == 4)
		dio_str[3] = '3';
	else if (io_num == 32)
		dio_str[3] = '4';
	else 
		strcpy(dio_str,"????");
*/
	y = 0;
	x = gpio_get_level(LORA_DIO0);
	if (x)
		{
		dio_str[12] = '1';
		y = y | 1;
		}
	x = gpio_get_level(LORA_DIO1);
	if (x)
		{
		dio_str[10] = '1';
		y = y | 2;
		}
	x = gpio_get_level(LORA_DIO2);
	if (x)
		{
		dio_str[8] = '1';
		y = y | 4;
		}
	x = gpio_get_level(LORA_DIO3);
	if (x)
		{
		dio_str[6] = '1';
		y = y | 8;
		}
	x = gpio_get_level(LORA_DIO4);
	if (x)
		{
		dio_str[4] = '1';
		y = y | 16;
		}
	
	if (prev_dio_val != y)
		{
		if(debug_do(DBG_LORA))
			{
//			printf("GPIO[%d] / %s intr, val: %d\n", io_num, dio_str, gpio_get_level((gpio_num_t)io_num));
			printf("###############\n");
			printf("### DIO[%d]  %s \n", y,dio_str);
			printf("###############\n");
			}

		prev_dio_val = y;
		LORA_int_flag = 1;
		}

//	LORA_int_flag = 1;

// reset WatchDog Timer for this thread...
	esp_task_wdt_reset();		
// yield to OS for a while (required)	TOO SLOW???!?
//	vTaskDelay(10 / portTICK_PERIOD_MS);

//	printf("LORA Q lOOP\n");
	}

// reset int reg flags
if (LORA_int_flag)
{
#if 0
unsigned char x;
unsigned char val[4];

printf("%c",0x07);	// BELL char

SX1276_read_reg(spi,LREG_IRQ_FLAGS,&x,1);
printf("LORA int reg: %02X\n",x);

if (x & 0x15)	// hdr \ CAD flags
{
unsigned char x;
/*
SX1276_read_reg(spi,LREG_OPMODE,&x,1);
x = (x & 0xF8) | 0x01;	// STDBY mode
SX1276_write_reg(spi,LREG_OPMODE,&x,1);

SX1276_read_reg(spi,LREG_OPMODE,&x,1);
//x = (x & 0xF8) | 0x07;	// channel activity detect mode
x = (x & 0xF8) | 0x05;	// Rx_Continuous mode
SX1276_write_reg(spi,LREG_OPMODE,&x,1);
*/
}

if (x & 0x50)	// Rx Done \ Valid Hdr flags
{
unsigned char i,n;
unsigned char rxdata[50];

if (x == 0x50)
	{
	unsigned char n;
	uint8_t *lora_rx_msg = NULL;
	
	SX1276_read_reg(spi,LREG_RX_NUM_BYTES,&n,1);

	lora_rx_msg = (uint8_t *)malloc(n);
	
	if (lora_rx_msg != NULL)
	{
// set up FIFO address for Rx data
	SX1276_read_reg(spi,LREG_FIFO_RX_CURR_ADDR,&x,1);
	SX1276_write_reg(spi,LREG_FIFO_ADDR_PTR,&x,1);

	printf("Rx data[%d]:\n",n);
	SX1276_read_reg(spi,LREG_FIFO,val,4);
	for (i=0;i<4;i++)
		{
		lora_rx_msg[i] = val[i];
		printf(" %02X",val[i]);
		}

	SX1276_read_reg(spi,LREG_FIFO,val,4);
	for (i=4;i<8;i++)
		{
		lora_rx_msg[i] = val[i-4];
		printf(" %02X",val[i-4]);
		}

	for (i=8;i<n;i++)
		{
		SX1276_read_reg(spi,LREG_FIFO,&x,1);
		lora_rx_msg[i] = x;
		printf(" %02X",x);
		}
	printf("\n\n");
	
// put msg onto outgoing msg queue...
//	lora_rx_data_wr->q_data = lora_rx_msg;
//	lora_rx_data_wr->q_data_len = n;

#ifdef LORAWAN_SCAN
// add this MAC addr to the list of MAC addresses
			if (lorawan_MAC_entries < LORAWAN_LIST_LEN)
				{
				unsigned char found_flag = 0;
				unsigned char dev_addr[5];
// check to see if it is on the MAC address list...
			for (i=0;i<4;i++)
				{
//				dev_addr[i] = lora_rx_data_wr->q_data[1+i];
				dev_addr[i] = lora_rx_msg[1+i];
				}
			dev_addr[i] = 0x00;
// check first that the MAC address isnt already on the list?					
				for (i=0;i<lorawan_MAC_entries;i++)
					{
					if (!memcmp(dev_addr,&lorawan_MAC_list[i],4))
						found_flag = 1;
					}
					
				if (!found_flag)						// if no match found on the list
					{
// do we need to check first that the MAC address isnt already on the list?					
					memcpy(&lorawan_MAC_list[lorawan_MAC_entries],&lora_rx_msg[1],4);	// LoRaWAN DecAddr is 4 bytes long...
					lorawan_MAC_entries++;
					}
				}

#endif
	
// put msg onto outgoing msg queue...
	lora_rx_data_wr->q_data = lora_rx_msg;
	lora_rx_data_wr->q_data_len = n;

/////////////////////////////////////////////////////////////
// whitelist \ blacklist now done before the data is equeued...
						{


						unsigned char send_lora_data_flag = 1;
						char dev_addr[5];
						
						printf("dev_addr: ");
//						memcpy(dev_addr,&lora_rx_data_rd->q_data[4],4);
						for (x=0;x<4;x++)
							{
							dev_addr[x] = lora_rx_msg[3 + 1 - x];
							printf("%02X:",dev_addr[x]);
							}
						printf("\n");
						
						dev_addr[4] = 0;

// send data if whitelisting not enabled, or there is a data match...				
							
//						if (bt_out_data->q_data[bt_out_data->q_data_len-1] == 0x0A)
									
						if (enable_lorawan_blacklist_flag)
							{
							printf("LoRa BL:");
//							if (listmatch(LORAWAN_CMD_LIST_LENGTH, LORAWAN_CMD_LIST_ENTRY_LENGTH, (char(*)[LORAWAN_CMD_LIST_ENTRY_LENGTH])lorawan_cmd_blacklist, (char*)lora_rx_data_rd->q_data,1))
//							if (listmatch(LORAWAN_CMD_LIST_LENGTH, LORAWAN_CMD_LIST_ENTRY_LENGTH, (char *)lorawan_cmd_blacklist, dev_addr,1,1))	//(char*)lora_rx_data_rd->q_data, 1, 1))
							if (listmatch(num_lora_bl_entries, LORAWAN_CMD_LIST_ENTRY_LENGTH, (char *)lorawan_cmd_blacklist, dev_addr,1,1))	//(char*)lora_rx_data_rd->q_data, 1, 1))
								{
								send_lora_data_flag = 0;									
								}
							}

						if ((enable_lorawan_whitelist_flag) && (send_lora_data_flag))		// if whitelist enabled and the data hasn't been blacklisted already...
							{
							send_lora_data_flag = 0;
							printf("LoRa WL:");
//							if (listmatch(LORAWAN_CMD_LIST_LENGTH, LORAWAN_CMD_LIST_ENTRY_LENGTH, (char(*)[LORAWAN_CMD_LIST_ENTRY_LENGTH])lorawan_cmd_whitelist, (char*)lora_rx_data_rd->q_data,1))
//							if (listmatch(LORAWAN_CMD_LIST_LENGTH, LORAWAN_CMD_LIST_ENTRY_LENGTH, (char *)lorawan_cmd_whitelist, dev_addr,1,1))	//(char*)lora_rx_data_rd->q_data,1 1, ))
							if (listmatch(num_lora_wl_entries, LORAWAN_CMD_LIST_ENTRY_LENGTH, (char *)lorawan_cmd_whitelist, dev_addr,1,1))	//(char*)lora_rx_data_rd->q_data,1 1, ))
								{
								send_lora_data_flag = 1;
								}
							}							




						if (send_lora_data_flag)

/////////////////////////////////////////////////////////////
							{
#if 0
	printf("\nCheck:\n");
	for (i=0;i<12;i++)
		{	
		printf("%02X %02X|",lora_rx_msg[i],lora_rx_data_wr->q_data[i]); 
		}
	printf("\n\n");
#endif
	
	if (queues_enabled_flag)
		{
		if (xQueueSend(lora_data_queue, lora_rx_data_wr, ( TickType_t ) 0) != pdTRUE) 
			{
			ESP_LOGD(TAG, "Failed to enqueue LORA msg output. Queue full.");
			printf("LORA msg queue full.\r\n");
// free memory taken by lora_rx_msg packet, as not used due to error
			free(lora_rx_msg);
			}
		else
			{
			lora_msg_ready_count++;				// flag to fg to send msg data

	//		if(debug_do(DBG_BLUETOOTH))
				{
				printf("No of queued LORA rx_msgs: %d\t\t",lora_msg_ready_count);
				printf("Queue free size: %d\n", uxQueueSpacesAvailable(lora_data_queue));
				}
			}
//	printf("end of LORA enqueue\n");
		}
	else
		free(lora_rx_msg);

							}
						}
							
	}
	else
		printf("LORA malloc error!\n");


	}
else
	printf("Rx ERR: FLAGS=%02X\n",x);

}

x = 0xFF;	// need to write a '1' to clear an int bit...
SX1276_write_reg(spi,LREG_IRQ_FLAGS,&x,1);

LORA_int_flag = 0;

//printf("LORA Exit\n");
#endif

//printf("%c",0x07);	// BELL char

//lora_rcv();
}
	
//printf("LORA Q Exit\n");

} 
#endif	// end of "#if 1 / 0"...
#endif	// end of "#ifdef USE_LORA"...

////////////////////////////////////////////
// set up ADC
////////////////////////////////////////////

#ifdef USE_ADCS
 //Characterize ADC at particular atten
 #define DEFAULT_VREF 	1100
		esp_adc_cal_characteristics_t *adc_chars_1 = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
		if (adc_chars_1 == NULL)
			printf("Couldn't calloc adc memory!\n");
		else
			{
			esp_adc_cal_value_t val_type_1 = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars_1);

//take motor current ADC measurement (V1: GPIO34 \ ADC1 chan 6   V2: GPIO39 \ ADC1 chan 3)
			adc1_config_width(ADC_WIDTH_BIT_12);
			adc1_config_channel_atten(ADC_VMON, ADC_ATTEN_DB_11);	// V1: ADC1_CHANNEL_6
			
// take supply voltage measurement (V1: GPIO35, \ ADC1 chan 7   V2: GPIO38, \ ADC1 chan 2)
			adc_val = adc1_get_raw(ADC_VMON);					// V1: ADC1_CHANNEL_7
			adc_val_raw_sv = adc_val;
//		printf("ADC val: %d\r\n",adc_val);
//		x = adc_val * ADC_REF * VMON_RATIO/(4095 * 10);
			x = adc_val * ADC_REF * VMON_RATIO_UPR/(4095 * VMON_RATIO_LWR);
// create value of voltage*10 so that 1dp of accuracy is available
			supply_voltage = (x + 5)/10;	// 50 causes carry over to 3 dp.

			voltage = esp_adc_cal_raw_to_voltage(adc_val, adc_chars_1);		
//		voltage = voltage * VMON_RATIO * 3/(100 * 100);
			voltage = voltage * VMON_RATIO_UPR * 3/(100 * 10 * VMON_RATIO_LWR);
			}

// deallocate the memory calloc'd above, or it just builds up and blows out...			
			free(adc_chars_1);
			
#endif	// end of "#ifdef USE_ADCS..."


#ifdef USE_TANK_SENSORS
	if (tank_sensors_present_flag == 1)
		{
 //Characterize ADC_2 at particular atten
//		esp_adc_cal_characteristics_t *adc_chars_2 = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
//		esp_adc_cal_value_t val_type_2 = esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars_2);

//		adc2_config_width(ADC_WIDTH_BIT_12);

// PROBLEMS WITH ADC2!
// ADC2 cant be used if there is any wifi or bluetooth activity!
// tank sensor 0 on SW0
////##		adc1_config_channel_atten(ADC_TANKSENSOR0, ADC_ATTEN_DB_11);	// V1: ADC1_CHANNEL_3
		adc1_config_channel_atten(ADC_TANKSENSOR0, ADC_ATTEN_DB_6);	// V1: ADC1_CHANNEL_3
//		esp_err_t adc_err = adc2_get_raw(ADC_TANKSENSOR0, ADC_WIDTH_12Bit, &adc_val);					// V1: ADC1_CHANNEL_3
		adc_val = adc1_get_raw(ADC_TANKSENSOR0);
		tank_sensor_0 = (unsigned char)(adc_val>>4);
/*
// tank sensor 1 on SW1
		adc2_config_channel_atten(ADC_TANKSENSOR1, ADC_ATTEN_DB_11);	// V1: ADC1_CHANNEL_3
		adc_err = adc2_get_raw(ADC_TANKSENSOR1, ADC_WIDTH_12Bit, &adc_val);					// V1: ADC1_CHANNEL_3
		tank_sensor_1 = (unsigned char)(adc_val>>4);
*/
#ifdef TANKSENSOR_1_ON_GPIO39
// tank sensor 1 on GPIO39 - (also shared with LORA_RADIO_DIO1...)
		adc1_config_channel_atten(ADC_TANKSENSOR1, ADC_ATTEN_DB_6);	// V1: ADC1_CHANNEL_3
//		esp_err_t adc_err = adc2_get_raw(ADC_TANKSENSOR0, ADC_WIDTH_12Bit, &adc_val);					// V1: ADC1_CHANNEL_3
		adc_val = adc1_get_raw(ADC_TANKSENSOR1);
		tank_sensor_1 = (unsigned char)(adc_val>>4);
#endif


// deallocate the memory calloc'd above, or it just builds up and blows out...			
//		free(adc_chars_2);
		}
	else
		{
		tank_sensor_0 = 128;
		tank_sensor_1 = 128;
		}
#if 1
	tank_sensor_percent_0 = (tank_sensor_0*100/TANK_SENSOR_MAX) + 1;
// fix the end values...
	if (tank_sensor_percent_0 > 100)
		tank_sensor_percent_0 = 100;
	else if (tank_sensor_0 == 0)
		tank_sensor_percent_0 = 0;
	
	tank_sensor_percent_1 = (tank_sensor_1*100/TANK_SENSOR_MAX) + 1;
// fix the end values...
	if (tank_sensor_percent_1 > 100)
		tank_sensor_percent_1 = 100;
	else if (tank_sensor_1 == 0)
		tank_sensor_percent_1 = 0;

	if (debug_do(DBG_TANK_SENSORS))
		{
		printf("Tank sens: %d (%d%%) ",tank_sensor_0,tank_sensor_percent_0);
		printf(" %d (%d%%)",tank_sensor_1,tank_sensor_percent_1);
		printf("\n");
		}

#else
	if (debug_do(DBG_TANK_SENSORS))
		{
		printf("Tank sens: %d ",tank_sensor_0);
		printf(" %d ",tank_sensor_1);
		printf("\n");
		}
#endif

	
	gpio_set_level(TSENSCTRL,1);			// set tank sensor power OFF


#endif

#ifdef USE_ADCS
		adc1_config_channel_atten(ADC_INPUT_0, ADC_ATTEN_DB_6);	// V1: ADC1_CHANNEL_5
		adc_val = adc1_get_raw(ADC_INPUT_0);
		adc_0 = (unsigned char)(adc_val>>4);

		adc1_config_channel_atten(ADC_INPUT_1, ADC_ATTEN_DB_6);	// V1: ADC1_CHANNEL_3
		adc_val = adc1_get_raw(ADC_INPUT_1);
		adc_1 = (unsigned char)(adc_val>>4);


	if (debug_do(DBG_ADCS))	
		{
		printf("ADC0: 0x%02X  [%3d]    ADC1: 0x%02X  [%3d]\n",adc_0,adc_0,adc_1,adc_1); 
		}		
#endif

#define I2C_SENSOR_DEBUG
#ifdef USE_I2C
// get I2C sensor values
//if (bt_advertising_ready_flag)
#ifdef USE_BLUETOOTH_ADVERTISING
// commit data to Bluetooth output
//if (bt_i2c_update_timer >= (2*bluetooth_sensor_adv_time*10*10/16*1000))	// advertising time in sec
if (bt_i2c_update_timer >= (bluetooth_sensor_adv_time/80))	// advertising time in sec
#endif
	{
unsigned char tmp[16];
unsigned char tmp2[16];

bt_i2c_update_timer = 0;
//{

//printf("bt_adv_flag: %d\n",bt_advertising_ready_flag);

	printf("\n");
	show_time(DBG,1);
	printf("   Sensor reading:  ");
	

if(0)		// necessary so that following sections can start as "else if"
	{}		// but code still works if the section is missing \ not compiled in
	
#ifdef USE_I2C_AMBIMATE4	
else if ((ambi_done_flag == I2C_SENSOR_READY) && (check_i2c_device(0x2A,0x00)== ESP_OK))
	{
	ambi_done_flag = I2C_SENSOR_DONE;
#ifdef I2C_SENSOR_DEBUG
	printf("Sensor=AmbiMate4\n");
#endif

// start measurement:
	tmp[0] = 0x7F;
	i2c_master_write_slave(I2C_MASTER_NUM,0x2A,0xC0,tmp,1);

// get values:
	i2c_master_read_slave(I2C_MASTER_NUM,0x2A,0x00,tmp,15);

#if 0
	for (i=0;i<15;i++)
		{
		printf("Reg: %d   Data: %02X\r\n",i,tmp[i]);
		}
#endif

#if 0
	printf("I2C Reg: ");
	for (i=0;i<15;i++)
		{
		printf("  %02X",tmp[i]);
		}
		printf("\r\n");
#endif

// use bt_sensor_data and bt_sensor_data_len to transfer data to bluetooth advertising in Bluetooth.cpp...
//	bt_sensor_data_len = 14;
//	memcpy(bt_sensor_data,&tmp[1],bt_sensor_data_len);

//unsigned int i2c_temp,i2c_humid,i2c_light,i2c_audio,i2c_batt,i2c_co2,i2c_voc;
	i2c_temp  = tmp[ 1]*256 + tmp[ 2];
	i2c_humid = tmp[ 3]*256 + tmp[ 4];
	i2c_light = tmp[ 5]*256 + tmp[ 6];
//	i2c_audio = tmp[ 7]*256 + tmp[ 8];
	i2c_audio =               tmp[ 8];
//	i2c_batt  = tmp[ 9]*256 + tmp[10];
	i2c_batt  = tmp[ 9]*256 + tmp[10] - 256;
	i2c_co2   = tmp[11]*256 + tmp[12];
	i2c_voc   = tmp[13]*256 + tmp[14];
	
// assemble BT advertising payload (new style with byte 0 = payload type)
	bt_sensor_data[0] = 0x00;	// payload type - AmbiMate4
	bt_sensor_data[1] = ambi_seq_num++;	// sequence number
	memcpy(&bt_sensor_data[2],&tmp[1],6);
	bt_sensor_data[7] = tmp[8];	// audio - 1 byte
	bt_sensor_data[8] = i2c_batt;
	memcpy(&bt_sensor_data[9],&tmp[11],4);
	bt_sensor_data_len = 14;
	}
#endif// end of "#define USE_I2C_AMBIMATE4

#ifdef USE_I2C_PIMORONI
// BME280: temp, pressure, humidity
// I2C addr 76 \ 77 reg D0 = 0x60 (id)
else if ((bme280_done_flag == I2C_SENSOR_READY) && (check_i2c_device(0x76,0x00)== ESP_OK))
	{
	bme280_done_flag = I2C_SENSOR_DONE;
#ifdef I2C_SENSOR_DEBUG
	printf("Sensor=BME280\n");
#endif
	
// get values:
	i2c_master_read_slave(I2C_MASTER_NUM,0x76,0xF7,tmp,8);

// assemble BT advertising payload (new style with byte 0 = payload type)
	bt_sensor_data[0] = 0x01;	// payload type - Pimoroni \ Bosch BME688
	bt_sensor_data[1] = bme280_seq_num++;	// sequence number
//	memcpy(&bt_sensor_data[1],&tmp[1],6);
	bt_sensor_data[2] = i2c_temp>>8;		// temperature
	bt_sensor_data[3] = i2c_temp&0xFF;		// temperature
	bt_sensor_data[4] = i2c_humid>>8;		// humidity
	bt_sensor_data[5] = i2c_humid&0xFF;		// humidity
	bt_sensor_data[6] = i2c_pressure>>8;	// pressure;
	bt_sensor_data[7] = i2c_pressure&0xFF;	// pressure;

	bt_sensor_data_len = 8;
	}
	
// BME688: gas, pressure, temperature & humidity
// I2C addr 76 \ 77: reg D0 = 0x61 (id)
else if ((bme688_done_flag == I2C_SENSOR_READY) && (check_i2c_device(0x76,0x00)== ESP_OK))	// NEEDS WORK!
	{
	bme688_done_flag = I2C_SENSOR_DONE;
#ifdef I2C_SENSOR_DEBUG
	printf("Sensor=BME688\n");
#endif
	
// get values:
	i2c_master_read_slave(I2C_MASTER_NUM,0x76,0x00,tmp,15);

// assemble BT advertising payload (new style with byte 0 = payload type)
	bt_sensor_data[0] = 0x02;	// payload type - Pimoroni \ Bosch BME688
	bt_sensor_data[1] = bme688_seq_num++;	// sequence number
	memcpy(&bt_sensor_data[2],&tmp[1],6);
	bt_sensor_data[7] = tmp[8];	// audio - 1 byte
	bt_sensor_data[8] = i2c_batt;
	memcpy(&bt_sensor_data[9],&tmp[11],4);
	bt_sensor_data_len = 13;

	}


// PIM376/LSM303D: BDOF Motion Sensor
// I2C addr 1D \ 1E
else if ((lsm303d_done_flag == I2C_SENSOR_READY) && (check_i2c_device(0x1D,0x00)== ESP_OK))
	{
	unsigned int x_mag,y_mag,z_mag,x_accel,y_accel,z_accel = 0;	
	unsigned char a,b,c,x,y;
		
	if (lsm303d_section == 0x01)
		{
		lsm303d_done_flag = I2C_SENSOR_DONE;
		}

#ifdef I2C_SENSOR_DEBUG
	printf("Sensor=LSM303D: section %d\n",lsm303d_section);
#endif

// initialisation
	tmp[0] = 0x80;	// enable temperature readings
	i2c_master_write_slave(I2C_MASTER_NUM,0x1D,0x24,tmp,1);
	vTaskDelay(10 / portTICK_PERIOD_MS);
	tmp[0] = 0x00;	// enable continuous readings
	i2c_master_write_slave(I2C_MASTER_NUM,0x1D,0x26,tmp,1);
	vTaskDelay(10 / portTICK_PERIOD_MS);

// get values:
	i2c_master_read_slave(I2C_MASTER_NUM,0x1D,0x05,tmp,3);
	vTaskDelay(10 / portTICK_PERIOD_MS);
	i2c_master_read_slave(I2C_MASTER_NUM,0x1D,0x0F,tmp2,1);
	vTaskDelay(10 / portTICK_PERIOD_MS);

	i2c_master_read_slave(I2C_MASTER_NUM,0x1D,0x05,&a,1);
	vTaskDelay(10 / portTICK_PERIOD_MS);
	i2c_master_read_slave(I2C_MASTER_NUM,0x1D,0x06,&b,1);
	vTaskDelay(10 / portTICK_PERIOD_MS);
	i2c_master_read_slave(I2C_MASTER_NUM,0x1D,0x07,&c,1);
	vTaskDelay(10 / portTICK_PERIOD_MS);
	i2c_master_read_slave(I2C_MASTER_NUM,0x1D,0x24,&x,1);
	vTaskDelay(10 / portTICK_PERIOD_MS);
	i2c_master_read_slave(I2C_MASTER_NUM,0x1D,0x26,&y,1);

	printf("LSM303D Val: %02X %02X:  %02X %02X %02X %02X:    %02X %02X %02X\n",x,y,tmp[0],tmp[1],tmp[2],tmp2[0],a,b,c);

// use bt_sensor_data and bt_sensor_data_len to transfer data to bluetooth advertising in Bluetooth.cpp...
//	bt_sensor_data_len = 3;
//	memcpy(bt_sensor_data,tmp,bt_sensor_data_len);

// assemble BT advertising payload (new style with byte 0 = payload type)
	if (lsm303d_section == 0x00)
		{
		bt_sensor_data[0] = 0x03;	// payload type - Pimoroni \ LSM303D
		bt_sensor_data[1] = lsm303d_seq_num;	// sequence number
//	memcpy(&bt_sensor_data[1],&tmp[1],6);
		bt_sensor_data[2] = x_mag>>8;		// x_mag
		bt_sensor_data[3] = x_mag&0xFF;		// x_mag
		bt_sensor_data[2] = y_mag>>8;		// y_mag
		bt_sensor_data[3] = y_mag&0xFF;		// y_mag
		bt_sensor_data[2] = z_mag>>8;		// z_mag
		bt_sensor_data[3] = z_mag&0xFF;		// z_mag

		bt_sensor_data[2] = x_accel>>8;		// x_accel
		bt_sensor_data[3] = x_accel&0xFF;	// x_accel
		bt_sensor_data[2] = y_accel>>8;		// y_accel
		bt_sensor_data[3] = y_accel&0xFF;	// y_accel
		bt_sensor_data[2] = z_accel>>8;		// z_accel
		bt_sensor_data[3] = z_accel&0xFF;	// z_accel

		bt_sensor_data_len = 3;
		lsm303d_section = 1;
		}
	else
		{
		bt_sensor_data[0] = 0x04;	// payload type - Pimoroni \ LSM303D
		bt_sensor_data[1] = lsm303d_seq_num++;	// sequence number
//	memcpy(&bt_sensor_data[1],&tmp[1],6);
		bt_sensor_data[2] = i2c_temp>>8;		// temperature
		bt_sensor_data[3] = i2c_temp&0xFF;		// temperature

		bt_sensor_data_len = 4;
		lsm303d_section = 0;
		}
		
	}

// PIM413/LTR559: Light & proximity sensor
// I2C addr 23
else if ((ltr559_done_flag == I2C_SENSOR_READY) && (check_i2c_device(0x23,0x00)== ESP_OK))
	{
	ltr559_done_flag = I2C_SENSOR_DONE;
#ifdef I2C_SENSOR_DEBUG
	printf("Sensor=LTR559\n");
#endif
	
// iniialise sensor
	tmp[0] = 0x01;
	i2c_master_write_slave(I2C_MASTER_NUM,0x23,0x80,tmp,1);

	tmp[0] = 0x03;
	i2c_master_write_slave(I2C_MASTER_NUM,0x23,0x81,tmp,1);

// get values:
	i2c_master_read_slave(I2C_MASTER_NUM,0x23,0x88,tmp,7);
	printf("LTR559 Val: %02X %02X %02X %02X %02X %02X %02X \n",tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],tmp[5],tmp[6]);

// use bt_sensor_data and bt_sensor_data_len to transfer data to bluetooth advertising in Bluetooth.cpp...
//	bt_sensor_data_len = 6;
//	memcpy(bt_sensor_data,tmp,bt_sensor_data_len);

// assemble BT advertising payload (new style with byte 0 = payload type)
	bt_sensor_data[0] = 0x05;	// payload type - Pimoroni \ PIM413 \ LTR559
	bt_sensor_data[1] = ltr559_seq_num++;	// sequence number
	bt_sensor_data[2] = i2c_light>>8;		// light level
	bt_sensor_data[3] = i2c_light&0xFF;		// light level
	bt_sensor_data[4] = i2c_proximity>>8;	// proximity
	bt_sensor_data[5] = i2c_proximity&0xFF;	// proximity

	bt_sensor_data_len = 6;

	}

#endif// end of "#define USE_I2C_PIMORONI
else		// all other sensors done - reset flags to start loop again
	{
	ambi_done_flag = I2C_SENSOR_READY;
	bme280_done_flag = I2C_SENSOR_READY;
	bme688_done_flag = I2C_SENSOR_READY;
	lsm303d_done_flag = I2C_SENSOR_READY;
	ltr559_done_flag = I2C_SENSOR_READY;
	}

#ifdef USE_BLUETOOTH_ADVERTISING
if (bt_sensor_data_len)
	{
	printf("BT - new adv data\n");
//		ble_set_adv_data();
	bleAdvtTask();		// restart BT advertising with new parameters
	}
#endif
	
}
#ifdef USE_BLUETOOTH_ADVERTISING
else
	bt_i2c_update_timer++;

#endif// end of "#ifdef USE_BLUETOOTH_ADVERTISING...


#ifdef XXX_USE_BLUETOOTH_ADVERTISING
// commit data to Bluetooth output
 if (bt_i2c_update_timer >= 10)
	{
	bt_i2c_update_timer = 0;

	if (bt_sensor_data_len)
		{
//		ble_set_adv_data();
		bleAdvtTask();		// restart BT advertising with new parameters
		}
	}
else
	bt_i2c_update_timer++;

//bleAdvtTask();
#endif// end of "#ifdef USE_BLUETOOTH_ADVERTISING...


//bt_advertising_ready_flag = 0;	


#endif	// end of "#ifdef USE_I2C...


// check for debug input

	str[0] = 0x00;
	fgets((char*)str,DBG_RCV_BUF_SIZE,stdin);
	if (strlen((char*)str))
		{
		strcat((char*)cmd_string,(char*)str);
		uart_len = strlen((char*)cmd_string);
//	if (uart_len && (cmd_string[uart_len-2]  == 0x0D) && (cmd_string[uart_len-1]  == 0x0A))
		if (uart_len && (cmd_string[uart_len-1]  == 0x0A))	
			{
/*
			for (i=0;i<uart_len;i++)
				{
				printf("%02X ",cmd_string[i]);
				}
			printf("\r\n");
*/			
			cmd_flag = 1;
			
			}
		}	

// serial autobaud
	if(ext_serial_bit_time_change_flag)
		{
		unsigned char change_flag = 0;
//		int32_t diff,rate,auto_serial_bitrate;
		int32_t diff,rate;
		
		diff = prev_ext_serial_bit_time - ext_serial_bit_time;
		if (diff < 0)
			diff = - diff;
		if (diff > prev_ext_serial_bit_time/10)
			change_flag = 1;
		rate = 1000000/ext_serial_bit_time;
		if (debug_do(DBG_AUTOBAUD))
			printf("Ext serial rate change: %d:    %db/s [%dus] [%08X]\n",change_flag,rate,ext_serial_bit_time,ext_serial_bit_time);


		if (rate > 100)			// rate = 0 can occur if there is a spike - so dont consider it in the checks below...
			{
			if (rate < 2000)
				auto_serial_bitrate = 1200;
			else if ((rate >= 2000) && (rate < 3600))
				auto_serial_bitrate = 2400;
			else if ((rate >= 3600) && (rate < 7200))
				auto_serial_bitrate = 4800;
			else if ((rate >= 7200) && (rate < 14400))
				auto_serial_bitrate = 9600;
			else if ((rate >= 14400) && (rate < 28800))
				auto_serial_bitrate = 19200;
			else if ((rate > 28800) && (rate < 48000))
				auto_serial_bitrate = 38400;
			else if ((rate > 48000) && (rate < 86400))
				auto_serial_bitrate = 57600;
			else 
				auto_serial_bitrate = 115200;
			}
			
						
		if (debug_do(DBG_AUTOBAUD))
			printf("Calc Rate: %db/s\n",auto_serial_bitrate);

		if (serial_autobaud_flag)							// using auto_serial_bitrate
			{
			if (serial_bitrate != auto_serial_bitrate)
				{
				nvs_handle nvs_handle;
				
				serial_bitrate = auto_serial_bitrate;
				uart_set_baudrate(NMEA,serial_bitrate);

				if (debug_do(DBG_AUTOBAUD))	
					{
					printf("New uart rate [AUTO] = %ldb/s\n",serial_bitrate);
					printf("Saving to NVS...\n");
					}
					
				nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
				nvs_set_u32(nvs_handle,"GW_SER_RATE",serial_bitrate);
				nvs_commit(nvs_handle);		
				nvs_close(nvs_handle);
 
				if (debug_do(DBG_AUTOBAUD))
					printf("New uart rate [AUTO] = %ldb/s\n",serial_bitrate);
				}
			}
		
		ext_serial_bit_time_change_flag = 0;
		prev_ext_serial_bit_time = ext_serial_bit_time;
		}


////////////////////////////////////////////////////////////////////////////
//
// wifi MAC address detection
//
////////////////////////////////////////////////////////////////////////////
#ifdef WIFI_SCAN
// wifi_scan_time is the scan period; if set to 0, no scan is required, so no data is available		
// increment wifi_scan_timer; if at limit, set wifi_scan_flag and get MAC data
		if (wifi_scan_time)		// perform scan if scan enabled
			{
			if (wifi_scan_timer >= wifi_scan_time)
				{
// make sure Wifi is switched ON and not modem sleeping...
				wifi_scan_timer = 0;
				wifi_scan_flag = 1;
				}
			}
		else
			{
// switch off Wifi and perform modem sleep...
			wifi_scan_timer = 0;
			wifi_scan_flag = 0;
			}


		if ((wifi_scan_enable_flag) && (wifi_scan_flag) && (!MAC_seq_number)) 
			{
			dbgprintf(DBG,"*******************************************************\r\n");
			dbgprintf(DBG,"*  Wifi Scan Begin ");
			show_time(DBG,1);
			dbgprintf(DBG,"  (every %d sec)\r\n",wifi_scan_time);
			dbgprintf(DBG,"*******************************************************\r\n");
			}
		else
			sleep_resume_flag = 1;

		if ((wifi_scan_flag) && (!wifi_is_on))
			{
// enable wifi...
			esp_wifi_start();
			wifi_is_on = 1;
			dbgprintf(DBG,"Started Wifi\r\n");
			}
			
		
		if ((wifi_scan_enable_flag) && (wifi_scan_flag)) 
			{	
// make sure Wifi is switched ON and not sleeping...
			show_time(DBG,0);
			dbgprintf(DBG,"Wifi scan started...\r\n");
			esp_wifi_scan_start(&wifi_scan_cfg,0);		// 0 = non-blocking: 1 = blocking
			wifi_scan_flag = 0;
			}

		if (wifi_check_flag)			// set in wifi.cpp when scan complete
			{
			dbgprintf(DBG,"Wifi scan finished\r\n");
#if 0
// switch off Wifi and perform modem sleep...
			if (wifi_is_on)
				{
				esp_wifi_stop();
				wifi_is_on = 0;
				dbgprintf(DBG,"Stopped Wifi\r\n");
				}
#endif
			dbgprintf(DBG,"Wifi scan... checking results\r\n\r\n");
// better to use a ptr to the struct then switch pointers to pass current -> previous?

// switch list pointers: current_>previous, empty->current
			wifi_list_tmp      = wifi_list_previous;
			wifi_list_previous = wifi_list_current;
			wifi_list_current  = wifi_list_tmp;
// switch list length indicators
			wifi_previous_num  = wifi_current_num;
//			wifi_current_num   = 0;
			
// get wifi MAC list and RSSIs into wifi_list_current
			wifi_current_num = WIFI_LIST_LEN;			// set max list size
			esp_wifi_scan_get_ap_records((uint16_t*)&wifi_current_num, wifi_list_current);

#if 1
// switch off Wifi and perform modem sleep...
#ifndef USE_BLUETOOTH
			if (wifi_is_on)
				{
				esp_wifi_stop();
				wifi_is_on = 0;
				dbgprintf(DBG,"Stopped Wifi\r\n");
				}
#endif
#endif


			wifi_dev_count = 0;

			if ((debug_do(DBG_MAC_TRAFFIC_LL)) || (!wifi_scan_valid))
				{
				dbgprintf(DBG,"Prev: %d  Curr: %d\r\n",wifi_previous_num,wifi_current_num);

				dbgprintf(DBG,"Current:\r\n");
				for (wfi = 0;wfi<wifi_current_num;wfi++)						
					{
//				dbgprintf(DBG,"%06X ",wifi_list_current[wfi].bssid);
					for (i=0;i<6;i++)
						{
						dbgprintf(DBG,"%02X",wifi_list_current[wfi].bssid[i]);
						if (i<5)
							dbgprintf(DBG,":");
						else
							dbgprintf(DBG,"  ");
						}
					if ((wfi) && (wfi%4 == 3))
						dbgprintf(DBG,"\r\n");
					}
				dbgprintf(DBG,"\r\n\r\n");

				dbgprintf(DBG,"Previous:\r\n");
				for (wfj = 0;wfj<wifi_previous_num;wfj++)						
					{
//				dbgprintf(DBG,"%06X ",wifi_list_previous[wfj].bssid);
					for (i=0;i<6;i++)
						{
						dbgprintf(DBG,"%02X",wifi_list_previous[wfj].bssid[i]);
						if (i<5)
							dbgprintf(DBG,":");
						else
							dbgprintf(DBG,"  ");
						}

					if ((wfj) && (wfj%4 == 3))
						dbgprintf(DBG,"\r\n");
					}
				dbgprintf(DBG,"\r\n\r\n");
				}			
			
// check lists for new devices...
//			wifi_dev_count = 0;
			hp1_wifi_rssi = -127;
			//hp1_mac_addr;
			//hp2_mac_addr;
			hp1_mac_index = 0xFFFF;
			hp2_mac_index = 0xFFFF;	
			
// for each addr in the current list...
			for (wfi = 0;wfi<wifi_current_num;wfi++)						
				{
				wf_found = 0;
				wf_rssi_too_low = 0;
				wf_private = 0;
// ...compare to each addr in previous list
				for (wfj = 0;wfj<wifi_previous_num;wfj++)
					{
//					if (wifi_list_current[wfi].bssid == wifi_list_previous[wfj].bssid)	
					if (!strcmp((char*)wifi_list_current[wfi].bssid,(char *)wifi_list_previous[wfj].bssid))	
						wf_found = 1;
					
					}
			
// stop low level signals from fixed sites from appearing \ disappearing on consecutive scans...
					if (wifi_list_current[wfi].rssi < MAC_COUNT_RSSI_THRESHOLD)
						wf_rssi_too_low = 1;
						
// check bottom 2 bits of first byte - should be 0 for a public device
					if (wifi_list_current[wfi].bssid[0] & 0x03)
						wf_private = 1;
					
// if not found, record new device
				if ((!wf_found) && (wifi_dev_count<65335))
					{
					if (!wf_rssi_too_low)
						wifi_dev_count++;
					
//					dbgprintf(DBG,"New: [%3d] %32.32s  %012X  %u\r\n",wfi,wifi_list_current[wfi].ssid,wifi_list_current[wfi].bssid,wifi_list_current[wfi].rssi);
					if ((debug_do(DBG_MAC_TRAFFIC)) || (!wifi_scan_valid))
						{
						dbgprintf(DBG,"New:[%3d] %32.32s  ",wfi,wifi_list_current[wfi].ssid);
						
						for (i=0;i<6;i++)
							{
							dbgprintf(DBG,"%02X",wifi_list_current[wfi].bssid[i]);
							if (i<5)
								dbgprintf(DBG,":");
							}
							
						dbgprintf(DBG,"  % 3ddB",wifi_list_current[wfi].rssi);
						
						if (wf_rssi_too_low)						
							dbgprintf(DBG," [LOW]");

//						dbgprintf(DBG,"%02X ",wifi_list_current[wfi].bssid[0]);
						if (wf_private)						
							dbgprintf(DBG," [PR]");
						
						dbgprintf(DBG,"\r\n");
						}

					if ((debug_do(DBG_MAC_TRAFFIC)) || (!wifi_scan_valid))
						{
						if (wifi_dev_count)	
							dbgprintf(DBG,"\r\n");
						}
					}

// check if this is the highest power SSID
//				dbgprintf(DBG,"RS: %d %d\r\n", hp1_wifi_rssi,wifi_list_current[wfi].rssi);
				
				if ((wifi_list_current[wfi].rssi > hp1_wifi_rssi) && (!wf_private))
					{
// check if this is a public MAC address...
					hp1_wifi_rssi = wifi_list_current[wfi].rssi;
					hp1_mac_index = wfi;
//					dbgprintf(DBG,"  Y");
					}

//				dbgprintf(DBG,"\r\n");

				}

			wifi_total_dev_count = wifi_total_dev_count + wifi_dev_count;

			dbgprintf(DBG,"Num Wifi MACs found this search = %d found %d new [%d total] \r\n\r\n",wifi_current_num,wifi_dev_count,wifi_total_dev_count);
//			wifi_dev_count = 0;
			wifi_check_flag = 0;
//			wifi_scan_flag = 1;

// check if this is the second highest power SSID
			hp2_wifi_rssi = -127;
			for (wfi = 0;wfi<wifi_current_num;wfi++)						
				{
// exclude previously found highest hp1_mac_index to find second highest...
				if ((wifi_list_current[wfi].rssi > hp2_wifi_rssi) && (wfi != hp1_mac_index)) 
					{
// check if this is a public MAC address...
					if ((wifi_list_current[wfi].bssid[0] & 0x03) == 0)
						{
						hp2_wifi_rssi = wifi_list_current[wfi].rssi;
						hp2_mac_index = wfi;
						}
					}
				}

//			dbgprintf(DBG,"HP1: %d   HP2:   %d\r\n",hp1_mac_index,hp2_mac_index);

			dbgprintf(DBG,"Hi Pwr MACs: ");
			if (hp1_mac_index != 0xFFFF)
				{
				dbgprintf(DBG,"(%4d) ",hp1_mac_index);
				for (i=0;i<6;i++)
					{
					dbgprintf(DBG,"%02X",wifi_list_current[hp1_mac_index].bssid[i]);
					if (i<5)
						dbgprintf(DBG,":");
					}

				dbgprintf(DBG,"  [%3d]   ",hp1_wifi_rssi);
				}
				
			if (hp2_mac_index != 0xFFFF)
				{
				dbgprintf(DBG,"(%4d) ",hp2_mac_index);
				for (i=0;i<6;i++)
					{
					dbgprintf(DBG,"%02X",wifi_list_current[hp2_mac_index].bssid[i]);
					if (i<5)
						dbgprintf(DBG,":");
					}

				dbgprintf(DBG,"  [%3d]",hp2_wifi_rssi);
				}
				
			dbgprintf(DBG,"\r\n\r\n");
			
			
			if (hp1_mac_index != 0xFFFF)
				{
				memcpy(geo_mac_addr[0],wifi_list_current[hp1_mac_index].bssid,6);
				geo_rssi[0] = wifi_list_current[hp1_mac_index].rssi;
				}
				
			if (hp2_mac_index != 0xFFFF)
				{
				memcpy(geo_mac_addr[1],wifi_list_current[hp2_mac_index].bssid,6);
				geo_rssi[1] = wifi_list_current[hp2_mac_index].rssi;
				}

			wifi_scan_valid = 1;
#ifdef USE_CPU_SLEEP
			sleep_resume_flag = 1;			// resume sleep
#endif			
			dbgprintf(DBG,"****************************************\r\n");
			dbgprintf(DBG,"*    Wifi Scan End  ");
			show_time(DBG,1);
			dbgprintf(DBG,"\r\n");
			dbgprintf(DBG,"****************************************\r\n");

#if 0	// ###!###
wifi_scan_enable_flag = 0;
#endif
			}	// end of "if wifi_check_flag)...

#endif// end of "#ifdef WIFI_SCAN			

#if 0			
// test DBG port output
		four_g_count++;
		if (four_g_count>=6)
			{
//			uart_write_bytes(AUX, "TEST DBG!\r\n", 12);
			dbgprintf(FOUR_G,"%c4G PORT:\r\n",0x0C);
			show_status(FOUR_G);
			four_g_count = 0;
			}
#endif





#ifdef USE_M4G_HTTP
// do pre-state machine functions...

// run the state machine...
		four_g_http_state_machine(&four_g_state, &prev_four_g_state);
#endif


#ifdef USE_M4G_MQTT


///////////////////////////////////////////////////////
//
// MQTT subscribe message decode
//
///////////////////////////////////////////////////////

		if ((CMQTT_rx_msg) || (server_cmd_test_flag))
			{
			unsigned int len,rx_bytes;
			unsigned char x;
			unsigned char c;
			unsigned char mqtt_rx_data[600];
			unsigned int mqtt_rx_length;
			char mqtt_rx_topic[80];
			char mqtt_rx_msg[600];
			char mqtt_rx_param[50];
			char tmp_topic_str[80];
			unsigned char get_flag;
			nvs_handle nvs_handle;


//#define MQTT_RX_DEBUG			
			printf("MQTTMSG: %d\n",CMQTT_rx_msg);

			if (server_cmd_test_flag)
				len = strlen(server_cmd_test_str);
			else
				len = uxQueueMessagesWaiting(mqtt_data_queue);


//			printf("MQTT len %d\n",len);



// get first line (+CMQTTSTART) and discard
// get next  line (+CMQTTTOPIC) and get params; get topic on next line
// get next  line (+CMQTTPAYLOAD) and get params; get topic data on next line
// get next  line (+CMQTTEND) and discard
			if (len)
				{
				if (server_cmd_test_flag)
					{
					strcpy(mqtt_rx_topic,"TEST");
					strcpy(mqtt_rx_msg,server_cmd_test_str);
					}
				else
					{
					printf("MQTT msg\n");
					rx_bytes = 0;
					get_flag = 0;
					
					while (rx_bytes < len)
						{
						mqtt_rx_length = 0;
						x = pdTRUE;
						c = 0x00;
						
						while ((c!=0x0A) && (x == pdTRUE))	//((c != 0x0A) && (x == pdTRUE))
							{
							x = xQueueReceive(mqtt_data_queue,&c, 10 / portTICK_PERIOD_MS);
							if (x == pdTRUE)
								{
								mqtt_rx_data[mqtt_rx_length] = c;
								mqtt_rx_length++;
								rx_bytes++;
								}
							}				
						mqtt_rx_data[mqtt_rx_length-2] = 0x00;	// remove CR-LF and terminate string
					
//#define MQTT_RX_DEBUG			
#ifdef MQTT_RX_DEBUG			
						printf("LINE: %s [%d]\n",mqtt_rx_data,get_flag);
#endif					
						if (get_flag == 1)
							{
							strcpy((char*)mqtt_rx_topic,(char*)mqtt_rx_data);
							get_flag = 0;
							}
						else if (get_flag == 2)
							{
							strcpy((char*)mqtt_rx_msg,(char*)mqtt_rx_data);
							get_flag = 0;
							}
						else
							{
							if (!strncmp((char*)mqtt_rx_data,"+CMQTTRXTOPIC",strlen("+CMQTTRXTOPIC")))
								{
#ifdef MQTT_RX_DEBUG			
								printf("Found TOPIC!\n");
#endif
								get_flag = 1;
								}
							else if (!strncmp((char*)mqtt_rx_data,"+CMQTTRXPAYLOAD",strlen("+CMQTTRXPAYLOAD")))
								{
#ifdef MQTT_RX_DEBUG			
								printf("Found PAYLOAD!\n");
#endif
								get_flag = 2;
								}
							}
						}
					}
					
				printf("\tMQTTRX:\n");
// check topic
				printf("\tTOPIC: %s\n",mqtt_rx_topic);
// act on topic cmessage			
				printf("\tMSG  : %s\n",mqtt_rx_msg);

				sprintf(tmp_topic_str,"/gateway_dtm/%s",mac_bin_str);
//				printf("Check for topic %s == %s\n",mqtt_rx_topic,tmp_topic_str);
//				debug_hex_msg((uint8_t*)mqtt_rx_topic,strlen(mqtt_rx_topic),"MQ :");
//				debug_hex_msg((uint8_t*)tmp_topic_str,strlen(tmp_topic_str),"TMP:");
				
				if ((!strncmp(mqtt_rx_topic,tmp_topic_str,strlen(tmp_topic_str))) || (server_cmd_test_flag))	// mqtt_rx_topic has /r on the end!
					{
					char listfield_str[100];
					unsigned int p = 0;
					unsigned char cmd, cmd_arg;
					unsigned char search_flag = 1;
					unsigned char check_flag;
					
//					printf("found rx topic...\n");
					printf("Received msg from server...\n");
					
//					sprintf(mqtt_rx_msg,"disconnect");		// TEST!
					debug_hex_msg((uint8_t*)mqtt_rx_msg,strlen(mqtt_rx_msg),"MQR:");
///////////////////////////
// new code for server message decode - using switch - case rather than string decode...
					get_listfield(&p,mqtt_rx_msg,listfield_str);
					
					if ((mqtt_rx_msg[0] > 0x2F) && (mqtt_rx_msg[0] < 0x3A))		// ie is numeric...
						cmd = atoi (listfield_str);
					else
						cmd = 0xFF;			// command string not numeric - error!
					
					response_flag = SRVR_ERR_RESPONSE;		// preset response flag: (2 = "ERROR")
					
					switch(cmd)
						{
/*
						case SCMD_NWLIST:		//  1 = NMEA whitelist
						break;

					case SCMD_NBLIST:		//  2 = NMEA blacklist
						break;
*/
					case SCMD_BWLIST:		//  3 = Bluetooth whitelist
#ifdef USE_BLUETOOTH
						bluetooth_whitelist_mode = WL_IDLE;
						printf("WLIST: ");
// format:
//  bwlist | ~on
//  bwlist | ~off
//  bwlist | ~add | $aaaaa | $bbbbbb etc
//  bwlist | ~rem | $aaaaa | $bbbbbb etc
										//	wlist | ~on  | $aaaaa | $bbbbbb etc
										//  wlist | ~off | $aaaaa | $bbbbbb etc
//  bwlist | ~rmall
//  bwlist | ~tx
						while (search_flag)
							{
							printf("NEW CMD: %s\n",mqtt_rx_msg);
//							search_flag = get_pipe(&p,mqtt_rx_msg);
							printf("ptr: %d  ",p);

							if (search_flag)
								{
								printf("CMD: %d  ",cmd);
// get argument string
								check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);
//								printf("Check: %d ",check_flag);
								if (check_flag)
									{
									if ((listfield_str[0] > 0x2F) && (listfield_str[0] < 0x3A))		// ie is numeric...
										cmd_arg = atoi(listfield_str);
									else
										cmd_arg = 0xFF;

									printf("ARG: %s  [%d]\n",listfield_str,cmd_arg);
									
										switch(cmd_arg)
											{
											case SARG_ON:		//  0 = turn listing ON
												printf("ON!");
												if (enable_bluetooth_whitelist_flag != 1)
													{
													enable_bluetooth_whitelist_flag = 1;
/*
													nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
													nvs_set_u8(nvs_handle,"GW_BWL_FLAG",(uint8_t)enable_bluetooth_whitelist_flag);
													nvs_commit(nvs_handle);		
													nvs_close(nvs_handle);
*/
													nvs_wr_u8("GW_VAR_LIST","GW_BWL_FLAG",NO_INX,(uint8_t)enable_bluetooth_whitelist_flag);
													}

												response_flag = SRVR_OK_RESPONSE;
												break;

											case SARG_OFF:			//  1 = turn listing OFF
												printf("OFF!");
												if (enable_bluetooth_whitelist_flag != 0)
													{
													enable_bluetooth_whitelist_flag = 0;
/*
													nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
													nvs_set_u8(nvs_handle,"GW_BWL_FLAG",(uint8_t)enable_bluetooth_whitelist_flag);
													nvs_commit(nvs_handle);		
													nvs_close(nvs_handle);
*/
													nvs_wr_u8("GW_VAR_LIST","GW_BWL_FLAG",NO_INX,(uint8_t)enable_bluetooth_whitelist_flag);
													}

												response_flag = SRVR_OK_RESPONSE;
												break;

											case SARG_ADD:			//  3 = ADD to list
												bluetooth_whitelist_mode = WL_ADD;
												break;

											case SARG_REMOVE:		//  4 = REMOVE to list
												bluetooth_whitelist_mode = WL_REM;
												break;

											case SARG_REMALL:		//  5 = REMOVE ALL from list
												printf("RMALL!");

												nvs_open("BT_MAC_LIST",NVS_READWRITE, &nvs_handle);

												for (unsigned char i=0;i<BLUETOOTH_CMD_LIST_LENGTH;i++)
													{
		//											bluetooth_cmd_whitelist[i][0] = 0x00;
													memcpy(&bluetooth_cmd_whitelist[i], list_empty_str, 6);							// delete the entry
													nvs_set_addr(nvs_handle,"BTW_MAC",i,(unsigned char*)list_empty_str,6);	// cant delete so set to 0
													}

												num_bt_wl_entries = 0;
												nvs_set_i8(nvs_handle,"BTW_MAC_MAX",num_bt_wl_entries);
												nvs_close(nvs_handle);

												response_flag = SRVR_OK_RESPONSE;
												break;

											case SARG_TX:			//  6 = send list to server
												printf("TX!");
												tx_bluetooth_whitelist_flag = 1;

												response_flag = SRVR_OK_RESPONSE;
												break;

											default:				// ERROR
												response_flag = SRVR_ERR_RESPONSE;		// "ERR"
												break;
											}

										if (bluetooth_whitelist_mode != BL_IDLE)
										{
										unsigned char mac_addr[6];
// get remaining MAC addr string 										
										check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);
										
										for (i=0;i<6;i++)
											{
											mac_addr[i] = asc2hex((unsigned char *)&listfield_str[2*i]);
											}												
//										printf("\nDATA: ");
										if (bluetooth_whitelist_mode == WL_ADD)
											{
											unsigned char found_flag;	
											printf("ADDMODE: ");
// if MAC addr not already in the list...
											found_flag = 0;	
											for (unsigned char f= 0;f< BLUETOOTH_CMD_LIST_LENGTH;f++)
												{
												if (!memcmp((char*)bluetooth_cmd_whitelist[i],mac_addr,6))			// if string found...
													{
													printf("Already on list!\n");
													found_flag = 1;
													}
												}
												
											if (found_flag == 0)
												{
												for (unsigned char i=0;i<BLUETOOTH_CMD_LIST_LENGTH;i++)
													{
//												printf("i: %d ",i);
//												if ((bluetooth_cmd_whitelist[i][0] == 0x00) && (strlen(listfield_str) <= (BLUETOOTH_CMD_LIST_ENTRY_LENGTH - 1)))	
// if space and str is short enough to fit...
													if (!memcmp(&bluetooth_cmd_whitelist[i], list_empty_str, 6))
														{
//													printf("Found empty entry...\n");
													if (strlen(listfield_str) == (2*BLUETOOTH_CMD_LIST_ENTRY_LENGTH))	// if space and str is short enough to fit...
														{
//													strcpy((char*)bluetooth_cmd_whitelist[i],listfield_str);			// copy the string into the table
															memcpy((char*)bluetooth_cmd_whitelist[i],mac_addr,6);			// copy the string into the table
															num_bt_wl_entries++;
														
// now add to NVS
															nvs_open("BT_MAC_LIST",NVS_READWRITE, &nvs_handle);
															nvs_set_addr(nvs_handle,"BTW_MAC",i,mac_addr,6);
															nvs_set_i8(nvs_handle,"BTW_MAC_MAX",num_bt_wl_entries);
															nvs_close(nvs_handle);

															printf("[%s] added @ %d  [%d]",listfield_str,i,num_bt_wl_entries);

															break;											// ... and end the loop
															}
														}
													else
														{
														for (unsigned char j=0;j<6;j++)
															{
															printf("%02X:",bluetooth_cmd_whitelist[i][j]);
															}
														printf("\n");
														}
													}

												response_flag = SRVR_OK_RESPONSE;
												}
										// else mac addr already exists in tale - response_flag is left at 0 - Error!
										
											}
										if (bluetooth_whitelist_mode == WL_REM)
											{
											printf("REMMODE: ");
											for (unsigned char i=0;i<BLUETOOTH_CMD_LIST_LENGTH;i++)
												{
//												printf("i: %d ",i);
												if (!memcmp((char*)bluetooth_cmd_whitelist[i],mac_addr,6))			// if string found...
													{
//													bluetooth_cmd_whitelist[i][0] = 0x00;							// delete the entry
													memcpy(&bluetooth_cmd_whitelist[i], list_empty_str, 6);							// delete the entry
													num_bt_wl_entries--;

// now update to NVS
													nvs_open("BT_MAC_LIST",NVS_READWRITE, &nvs_handle);
													nvs_set_addr(nvs_handle,"BTW_MAC",i,(unsigned char*)list_empty_str,6);	// cant delete so set to 0
													nvs_set_i8(nvs_handle,"BTW_MAC_MAX",num_bt_wl_entries);
													nvs_close(nvs_handle);

													
													printf("%s removed @ %d  [%d]\n",listfield_str,i,num_bt_wl_entries);
													break;											// ... and end the loop
													}
												}

											response_flag = SRVR_OK_RESPONSE;
											}
											
										bluetooth_whitelist_mode = WL_IDLE;
										}
										
									if (check_flag > 1)			// if this field was the last...
										search_flag = 0;

									printf("\n");
									}
								}
							}
						printf("\n");
#endif
						break;

					case SCMD_BBLIST:		//  4 = Bluetooth blacklist
#ifdef USE_BLUETOOTH
						bluetooth_blacklist_mode = WL_IDLE;
						printf("BLIST: ");
// format:
//  bblist | ~on
//  bblist | ~off
//  bblist | ~add | $aaaaa | $bbbbbb etc
//  bblist | ~rem | $aaaaa | $bbbbbb etc
										//	wlist | ~on  | $aaaaa | $bbbbbb etc
										//  wlist | ~off | $aaaaa | $bbbbbb etc
//  bblist | ~rmall
//  bblist | ~tx
						while (search_flag)
							{
							printf("NEW CMD: %s\n",mqtt_rx_msg);
//							search_flag = get_pipe(&p,mqtt_rx_msg);
							printf("ptr: %d  ",p);

							if (search_flag)
								{
								printf("CMD: %d  ",cmd);
// get argument string
								check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);
//								printf("Check: %d ",check_flag);
								if (check_flag)
									{
									if ((listfield_str[0] > 0x2F) && (listfield_str[0] < 0x3A))		// ie is numeric...
										cmd_arg = atoi(listfield_str);
									else
										cmd_arg = 0xFF;

									printf("ARG: %s  [%d]\n",listfield_str,cmd_arg);
									
										switch(cmd_arg)
											{
											case SARG_ON:		//  0 = turn listing ON
												printf("ON!");
												if (enable_bluetooth_blacklist_flag != 1)
													{
													enable_bluetooth_blacklist_flag = 1;
/*
													nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
													nvs_set_u8(nvs_handle,"GW_BBL_FLAG",(uint8_t)enable_bluetooth_blacklist_flag);
													nvs_commit(nvs_handle);		
													nvs_close(nvs_handle);
*/
													nvs_wr_u8("GW_VAR_LIST","GW_BBL_FLAG",NO_INX,(uint8_t)enable_bluetooth_blacklist_flag);
													}

												response_flag = SRVR_OK_RESPONSE;
												break;

											case SARG_OFF:			//  1 = turn listing OFF
												printf("OFF!");
												if (enable_bluetooth_blacklist_flag != 0)
													{
													enable_bluetooth_blacklist_flag = 0;
/*
													nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
													nvs_set_u8(nvs_handle,"GW_BBL_FLAG",(uint8_t)enable_bluetooth_blacklist_flag);
													nvs_commit(nvs_handle);		
													nvs_close(nvs_handle);
*/
													nvs_wr_u8("GW_VAR_LIST","GW_BBL_FLAG",NO_INX,(uint8_t)enable_bluetooth_blacklist_flag);
													}

												response_flag = SRVR_OK_RESPONSE;
												break;

											case SARG_ADD:			//  3 = ADD to list
												bluetooth_blacklist_mode = BL_ADD;
												break;

											case SARG_REMOVE:		//  4 = REMOVE to list
												bluetooth_blacklist_mode = BL_REM;
												break;

											case SARG_REMALL:		//  5 = REMOVE ALL from list
												printf("RMALL!");

												nvs_open("BT_MAC_LIST",NVS_READWRITE, &nvs_handle);

												for (unsigned char i=0;i<BLUETOOTH_CMD_LIST_LENGTH;i++)
													{
		//											bluetooth_cmd_whitelist[i][0] = 0x00;
													memcpy(&bluetooth_cmd_blacklist[i], list_empty_str, 6);							// delete the entry
													nvs_set_addr(nvs_handle,"BTB_MAC",i,(unsigned char*)list_empty_str,6);	// cant delete so set to 0
													}

												num_bt_bl_entries = 0;
												nvs_set_i8(nvs_handle,"BTB_MAC_MAX",num_bt_bl_entries);
												nvs_close(nvs_handle);

												response_flag = SRVR_OK_RESPONSE;
												break;

											case SARG_TX:			//  6 = send list to server
												printf("TX!");
												tx_bluetooth_blacklist_flag = 1;

												response_flag = SRVR_OK_RESPONSE;
												break;

											default:				// ERROR
												response_flag = SRVR_ERR_RESPONSE;		// "ERR"
												break;
											}

										


										if (bluetooth_blacklist_mode != BL_IDLE)
										{
										unsigned char mac_addr[6];
// get remaining MAC addr string 										
										check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);
										
										for (i=0;i<6;i++)
											{
											mac_addr[i] = asc2hex((unsigned char *)&listfield_str[2*i]);
											}												

										if (bluetooth_blacklist_mode == BL_ADD)
											{
											unsigned char found_flag;	
											printf("ADDMODE: ");
// if MAC addr not already in the list...
											found_flag = 0;	
											for (unsigned char f= 0;f< BLUETOOTH_CMD_LIST_LENGTH;f++)
												{
												if (!memcmp((char*)bluetooth_cmd_blacklist[i],mac_addr,6))			// if string found...
													{
													printf("Already on list!\n");
													found_flag = 1;
													}
												}
												
											if (found_flag == 0)
												{
												for (unsigned char i=0;i<BLUETOOTH_CMD_LIST_LENGTH;i++)
													{
//												printf("i: %d ",i);
//												if ((bluetooth_cmd_whitelist[i][0] == 0x00) && (strlen(listfield_str) <= (BLUETOOTH_CMD_LIST_ENTRY_LENGTH - 1)))	// if space and str is short enough to fit...
													if (!memcmp(&bluetooth_cmd_blacklist[i], list_empty_str, 6))
														{
														if (strlen(listfield_str) == (2*BLUETOOTH_CMD_LIST_ENTRY_LENGTH))	// if space and str is short enough to fit...
															{
															printf("%s added @ %d",listfield_str,i);
//													strcpy((char*)bluetooth_cmd_whitelist[i],listfield_str);			// copy the string into the table
															memcpy((char*)bluetooth_cmd_blacklist[i],mac_addr,6);			// copy the string into the table
															num_bt_bl_entries++;

// now add to NVS
															nvs_open("BT_MAC_LIST",NVS_READWRITE, &nvs_handle);
															nvs_set_addr(nvs_handle,"BTB_MAC",i,mac_addr,6);
															nvs_set_i8(nvs_handle,"BTB_MAC_MAX",num_bt_bl_entries);
															nvs_close(nvs_handle);

															printf("[%s] added @ %d  [%d]",listfield_str,i,num_bt_bl_entries);

															break;											// ... and end the loop
															}
														else
															{
															for (unsigned char j=0;j<6;j++)
																{
																printf("%02X:",bluetooth_cmd_blacklist[i][j]);
																}
															printf("\n");
															}
														}
													}

												response_flag = SRVR_OK_RESPONSE;
												}
											}
											
										if (bluetooth_blacklist_mode == BL_REM)
											{
											printf("REMMODE: ");
											for (unsigned char i=0;i<BLUETOOTH_CMD_LIST_LENGTH;i++)
												{
//												printf("i: %d ",i);
												if (!memcmp((char*)bluetooth_cmd_blacklist[i],mac_addr,6))			// if string found...
													{
//													bluetooth_cmd_whitelist[i][0] = 0x00;							// delete the entry
													memcpy(&bluetooth_cmd_blacklist[i], list_empty_str, 6);							// delete the entry
													num_bt_bl_entries--;

// now add to NVS
													nvs_open("BT_MAC_LIST",NVS_READWRITE, &nvs_handle);
													nvs_set_addr(nvs_handle,"BTB_MAC",i,mac_addr,6);
													nvs_set_i8(nvs_handle,"BTB_MAC_MAX",num_bt_bl_entries);
													nvs_close(nvs_handle);

													printf("%s removed @ %d  [%d]\n",listfield_str,i,num_bt_bl_entries);
													break;											// ... and end the loop
													}
												}

											response_flag = SRVR_OK_RESPONSE;
											}
											
										bluetooth_blacklist_mode = BL_IDLE;
										}
										
									if (check_flag > 1)			// if this field was the last...
										search_flag = 0;

									printf("\n");
									}
								}
							}
						printf("\n");
#endif
						break;

					case SCMD_LWLIST:		//  5 = LoRaWAN whitelist
#ifdef USE_LORA
						lorawan_whitelist_mode = WL_IDLE;
						printf("LoRa WLIST: ");
// format:
//  lwlist | ~on
//  lwlist | ~off
//  lwlist | ~add | $aaaaa | $bbbbbb etc
//  lwlist | ~rem | $aaaaa | $bbbbbb etc
										//	wlist | ~on  | $aaaaa | $bbbbbb etc
										//  wlist | ~off | $aaaaa | $bbbbbb etc
//  lwlist | ~rmall
//  lwlist | ~tx
						while (search_flag)
							{
							printf("NEW CMD: %s\n",mqtt_rx_msg);
							search_flag = get_pipe(&p,mqtt_rx_msg);
							printf("ptr: %d  ",p);

							if (search_flag)
								{
								printf("CMD: %d  ",cmd);

								check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);

//								printf("Check: %d ",check_flag);
								if (check_flag)
									{
									cmd_arg = atoi(listfield_str);
									printf("ARG: %s  [%d]\n",listfield_str,cmd_arg);

									switch(cmd_arg)
										{
										case SARG_ON:		//  0 = turn listing ON
											printf("ON!");
											if (enable_lorawan_whitelist_flag != 1)
												{
												enable_lorawan_whitelist_flag = 1;
/*
												nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
												nvs_set_u8(nvs_handle,"GW_LWL_FLAG",(uint8_t)enable_lorawan_whitelist_flag);
												nvs_commit(nvs_handle);		
												nvs_close(nvs_handle);
*/
												nvs_wr_u8("GW_VAR_LIST","GW_LWL_FLAG",NO_INX,(uint8_t)enable_lorawan_whitelist_flag);
												}

											response_flag = SRVR_OK_RESPONSE;
											break;
											
										case SARG_OFF:			//  1 = turn listing OFF
											printf("OFF!");
											if (enable_lorawan_whitelist_flag != 0)
												{
												enable_lorawan_whitelist_flag = 0;
/*
												nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
												nvs_set_u8(nvs_handle,"GW_LWL_FLAG",(uint8_t)enable_lorawan_whitelist_flag);
												nvs_commit(nvs_handle);		
												nvs_close(nvs_handle);
*/
												nvs_wr_u8("GW_VAR_LIST","GW_LWL_FLAG",NO_INX,(uint8_t)enable_lorawan_whitelist_flag);
												}

											response_flag = SRVR_OK_RESPONSE;
											break;
											
										case SARG_ADD:			//  3 = ADD to list
											lorawan_whitelist_mode = WL_ADD;
											break;
											
										case SARG_REMOVE:		//  4 = REMOVE to list
											lorawan_whitelist_mode = WL_REM;
											break;
											
										case SARG_REMALL:		//  5 = REMOVE ALL from list
											printf("RMALL!");

											nvs_open("LORA_MAC_LIST",NVS_READWRITE, &nvs_handle);

											for (unsigned char i=0;i<LORAWAN_CMD_LIST_LENGTH;i++)
												{
												memcpy(&lorawan_cmd_whitelist[i], list_empty_str, 4);							// delete the entry
												nvs_set_addr(nvs_handle,"LOW_MAC",i,(unsigned char*)list_empty_str,4);	// cant delete so set to 0
												}

											num_lora_wl_entries = 0;
											nvs_set_i8(nvs_handle,"LOW_MAC_MAX",num_bt_wl_entries);
											nvs_close(nvs_handle);

											response_flag = SRVR_OK_RESPONSE;
											break;
											
										case SARG_TX:			//  6 = send list to server
											printf("TX!");
											tx_lorawan_whitelist_flag = 1;

											response_flag = SRVR_OK_RESPONSE;
											break;

										default:				// ERROR
											response_flag = SRVR_ERR_RESPONSE;		// "ERR"
											break;
											
										}
										
										if (lorawan_whitelist_mode != BL_IDLE)
										{
										unsigned char dev_addr[4];
// get remaining MAC addr string 										
										check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);
										
										for (i=0;i<4;i++)
											{
											dev_addr[i] = asc2hex((unsigned char *)&listfield_str[2*i]);
											}												
//										printf("DATA: ");
										if (lorawan_whitelist_mode == WL_ADD)
											{
											unsigned char found_flag;	

											printf("ADDMODE: ");
// if MAC addr not already in the list...
											found_flag = 0;	
											for (unsigned char f= 0;f< LORAWAN_CMD_LIST_LENGTH;f++)
												{
												if (!memcmp((char*)lorawan_cmd_whitelist[i],dev_addr,4))			// if string found...
													{
													printf("Already on list!\n");
													found_flag = 1;
													}
												}
												
											if (found_flag == 0)
												{
												for (unsigned char i=0;i<LORAWAN_CMD_LIST_LENGTH;i++)
													{
//												printf("i: %d ",i);
//												if ((lorawan_cmd_whitelist[i][0] == 0x00) && (strlen(listfield_str) <= (LORAWAN_CMD_LIST_ENTRY_LENGTH - 1)))	// if space and str is short enough to fit...
													if (!memcmp(&lorawan_cmd_whitelist[i], list_empty_str, 4)) 
														{
														if (strlen(listfield_str) == (2*LORAWAN_CMD_LIST_ENTRY_LENGTH))	// if space and str is short enough to fit...
															{
															memcpy((char*)lorawan_cmd_whitelist[i],dev_addr,4);			// copy the string into the table
															num_lora_wl_entries++;

// now add to NVS
															nvs_open("LORA_MAC_LIST",NVS_READWRITE, &nvs_handle);
															nvs_set_addr(nvs_handle,"LOW_MAC",i,dev_addr,4);
															nvs_set_i8(nvs_handle,"LOW_MAC_MAX",num_lora_wl_entries);
															nvs_close(nvs_handle);

															break;											// ... and end the loop
															}
														}
													}

												response_flag = SRVR_OK_RESPONSE;
												}
											// else - dev addr already on list - leave repsonse_flag at 0 - Error!
											}
											
										if (lorawan_whitelist_mode == WL_REM)
											{
											printf("REMMODE: ");
											for (unsigned char i=0;i<LORAWAN_CMD_LIST_LENGTH;i++)
												{
												printf("i: %d ",i);
												if (!memcmp((char*)lorawan_cmd_whitelist[i],dev_addr,4))			// if string found...
													{
													memcpy(&lorawan_cmd_whitelist[i], list_empty_str, 4);							// delete the entry
													num_lora_wl_entries--;

// now add to NVS
													nvs_open("LORA_MAC_LIST",NVS_READWRITE, &nvs_handle);
													nvs_set_addr(nvs_handle,"LOW_MAC",i,mac_addr,4);
													nvs_set_i8(nvs_handle,"LOW_MAC_MAX",num_lora_wl_entries);
													nvs_close(nvs_handle);

													printf("%s removed @ %d  [%d]\n",listfield_str,i,num_bt_wl_entries);
													break;											// ... and end the loop
													}
												}

											response_flag = SRVR_OK_RESPONSE;
											}
										
										lorawan_whitelist_mode = WL_IDLE;
										}
										
									if (check_flag > 1)			// if this field was the last...
										search_flag = 0;

									printf("\n");
									}
								}
							}
						printf("\n");
#endif
						break;

					case SCMD_LBLIST:		//  6 = LoRaWAN blacklist
#ifdef USE_LORA
						printf("LoRa BLIST: ");
// format:
//  lblist | ~on
//  lblist | ~off
//  lblist | ~add | $aaaaa | $bbbbbb etc
//  lblist | ~rem | $aaaaa | $bbbbbb etc
										//	wlist | ~on  | $aaaaa | $bbbbbb etc
										//  wlist | ~off | $aaaaa | $bbbbbb etc
//  lblist | ~rmall
//  lblist | ~tx
						while (search_flag)
							{
							printf("NEW CMD: %s\n",mqtt_rx_msg);
							search_flag = get_pipe(&p,mqtt_rx_msg);
							printf("ptr: %d  ",p);

							if (search_flag)
								{
								printf("CMD: %d  ",cmd);

								check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);

//								printf("Check: %d ",check_flag);
								if (check_flag)
									{
									cmd_arg = atoi(listfield_str);
									printf("ARG: %s  [%d]\n",listfield_str,cmd_arg);

									switch(cmd_arg)
										{
										case SARG_ON:		//  0 = turn listing ON
											printf("ON!");
											if (enable_lorawan_blacklist_flag != 1)
												{
												enable_lorawan_blacklist_flag = 1;
/*
												nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
												nvs_set_u8(nvs_handle,"GW_LBL_FLAG",(uint8_t)enable_lorawan_blacklist_flag);
												nvs_commit(nvs_handle);		
												nvs_close(nvs_handle);
*/
												nvs_wr_u8("GW_VAR_LIST","GW_LBL_FLAG",NO_INX,(uint8_t)enable_lorawan_blacklist_flag);
												}

											response_flag = SRVR_OK_RESPONSE;
											break;
											
										case SARG_OFF:			//  1 = turn listing OFF
											printf("OFF!");
											if (enable_lorawan_blacklist_flag != 0)
												{
												enable_lorawan_blacklist_flag = 0;
/*
												nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
												nvs_set_u8(nvs_handle,"GW_LBL_FLAG",(uint8_t)enable_lorawan_blacklist_flag);
												nvs_commit(nvs_handle);		
												nvs_close(nvs_handle);
*/
												nvs_wr_u8("GW_VAR_LIST","GW_LBL_FLAG",NO_INX,(uint8_t)enable_lorawan_blacklist_flag);
												}

											response_flag = SRVR_OK_RESPONSE;
											break;
											
										case SARG_ADD:			//  3 = ADD to list
											lorawan_blacklist_mode = BL_ADD;
											break;
											
										case SARG_REMOVE:		//  4 = REMOVE to list
											lorawan_blacklist_mode = BL_REM;
											break;
											
										case SARG_REMALL:		//  5 = REMOVE ALL from list
											printf("RMALL!");

											nvs_open("LORA_MAC_LIST",NVS_READWRITE, &nvs_handle);

											for (unsigned char i=0;i<LORAWAN_CMD_LIST_LENGTH;i++)
												{
												memcpy(&lorawan_cmd_blacklist[i], list_empty_str, 4);							// delete the entry
												nvs_set_addr(nvs_handle,"LOB_MAC",i,(unsigned char*)list_empty_str,4);	// cant delete so set to 0
												}
											num_lora_bl_entries = 0;
											nvs_set_i8(nvs_handle,"LOB_MAC_MAX",num_lora_bl_entries);
											nvs_close(nvs_handle);

											response_flag = SRVR_OK_RESPONSE;
											break;
											
										case SARG_TX:			//  6 = send list to server
											printf("TX!");
											tx_lorawan_blacklist_flag = 1;

											response_flag = SRVR_OK_RESPONSE;
											break;

										default:				// ERROR
											response_flag = SRVR_ERR_RESPONSE;		// "ERR"
											break;
											
										}
										
										if (lorawan_whitelist_mode != BL_IDLE)
										{
										unsigned char dev_addr[6];
// get remaining MAC addr string 										
										check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);
										
										for (i=0;i<4;i++)
											{
											dev_addr[i] = asc2hex((unsigned char *)&listfield_str[2*i]);
											}												
//										printf("DATA: ");
										if (lorawan_blacklist_mode == BL_ADD)
											{
											unsigned char found_flag;	

											printf("ADDMODE: ");
// if MAC addr not already in the list...
											found_flag = 0;	
											for (unsigned char f= 0;f< LORAWAN_CMD_LIST_LENGTH;f++)
												{
												if (!memcmp((char*)lorawan_cmd_blacklist[i],dev_addr,4))			// if string found...
													{
													printf("Already on list!\n");
													found_flag = 1;
													}
												}
												
											if (found_flag == 0)
												{
												for (unsigned char i=0;i<LORAWAN_CMD_LIST_LENGTH;i++)
													{
//												printf("i: %d ",i);
//												if ((lorawan_cmd_whitelist[i][0] == 0x00) && (strlen(listfield_str) <= (LORAWAN_CMD_LIST_ENTRY_LENGTH - 1)))	// if space and str is short enough to fit...
													if (!memcmp(&lorawan_cmd_blacklist[i], list_empty_str, 4))
														{
														if (strlen(listfield_str) == (2*LORAWAN_CMD_LIST_ENTRY_LENGTH))	// if space and str is short enough to fit...
															{
//													printf("%s added @ %d",listfield_str,i);
//													strcpy((char*)lorawan_cmd_whitelist[i],listfield_str);			// copy the string into the table
															memcpy((char*)lorawan_cmd_blacklist[i],listfield_str,4);			// copy the string into the table
															num_lora_bl_entries++;

// now add to NVS
															nvs_open("LORA_MAC_LIST",NVS_READWRITE, &nvs_handle);
															nvs_set_addr(nvs_handle,"LOB_MAC",i,mac_addr,4);
															nvs_set_i8(nvs_handle,"LOB_MAC_MAX",num_lora_bl_entries);
															nvs_close(nvs_handle);

															printf("[%s] added @ %d  [%d]",listfield_str,i,num_lora_bl_entries);
															break;											// ... and end the loop
															}
														}
													}
													response_flag = SRVR_OK_RESPONSE;
												}
											}
											
										if (lorawan_blacklist_mode == BL_REM)
											{
											printf("REMMODE: ");
											for (unsigned char i=0;i<LORAWAN_CMD_LIST_LENGTH;i++)
												{
												printf("i: %d ",i);
												if (!strcmp((char*)lorawan_cmd_blacklist[i],listfield_str))			// if string found...
													{
													printf("%s Removed!",lorawan_cmd_blacklist[i]);
//													lorawan_cmd_whitelist[i][0] = 0x00;							// delete the entry
													memcpy(&lorawan_cmd_blacklist[i], list_empty_str, 4);							// delete the entry
													num_lora_bl_entries--;

// now add to NVS
													nvs_open("LORA_MAC_LIST",NVS_READWRITE, &nvs_handle);
													nvs_set_addr(nvs_handle,"LOB_MAC",i,dev_addr,4);
													nvs_set_i8(nvs_handle,"LOB_MAC_MAX",num_lora_bl_entries);
													nvs_close(nvs_handle);

													printf("[%s] added @ %d  [%d]",listfield_str,i,num_lora_bl_entries);
													break;											// ... and end the loop
													}
												}

											response_flag = SRVR_OK_RESPONSE;
											}
										
										lorawan_blacklist_mode = BL_IDLE;
										}
										
									if (check_flag > 1)			// if this field was the last...
										search_flag = 0;

									printf("\n");
									}
								}
							}
						printf("\n");
#endif
						break;

					case SCMD_BAUD:			//  7 = set serial bitrate
// format:
// baud | nnnn
// baud | ?
// baud | tx
						unsigned long tmp_bitrate;
						unsigned char tmp_autobaud_flag;
						
						search_flag = get_pipe(&p,mqtt_rx_msg);

						check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);
						printf("BAUD: %d ",check_flag);
						if (check_flag)
							{
							tmp_bitrate = atoi(listfield_str);
							if (tmp_bitrate < SARG_VAL)
								cmd_arg = tmp_bitrate;
							else 
								cmd_arg = SARG_VAL;
							
							switch(cmd_arg)
								{
								case SARG_QUERY:				// query current state
									response_flag = SRVR_OK_RESPONSE;
									break;
									
								case SARG_TX:			//  6 = send list to server	
									tx_serial_settings_flag = 1;

									response_flag = SRVR_OK_RESPONSE;
									break;
								
								case SARG_VAL:							// value to be used...
									tmp_bitrate = atoi(listfield_str);
									if (tmp_bitrate == 1)				// 1 = use auto_serial_bitrate (dont use 0 as this can occur if str is not numeric!)
										{
										tmp_autobaud_flag = 1;
										serial_bitrate = auto_serial_bitrate;
										}
									else
										{
										tmp_autobaud_flag = 0;	
										serial_bitrate = tmp_bitrate;
										}

// save to NVS \ FLASH ROM for next boot...
								
//								if (debug_do(DBG_AUTOBAUD))	
										{
										printf("Saving to NVS\n");
										}
									nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
									if (tmp_autobaud_flag != serial_autobaud_flag)
										{
										serial_autobaud_flag = tmp_autobaud_flag;
										nvs_set_u8(nvs_handle,"GW_AUTOBAUD",serial_autobaud_flag);
										}
									if (tmp_bitrate == 1)				// auto mode
										{
										if (serial_bitrate != auto_serial_bitrate)
											{
											serial_bitrate = auto_serial_bitrate;
											nvs_set_u32(nvs_handle,"GW_SER_RATE",serial_bitrate);
											}
										}
									else								// non-auto - use tmp_bitrate as sent from server...
										{
										if (serial_bitrate != tmp_bitrate)
											{
											serial_bitrate = tmp_bitrate;
											nvs_set_u32(nvs_handle,"GW_SER_RATE",serial_bitrate);
											}
										}
									
//								nvs_set_u8(nvs_handle,"GW_BIN_FLAG",0);
//								nvs_set_str(nvs_handle,"GW_EOL_STR",tmpstr);
//								nvs_set_i8(nvs_handle,"GW_SER_RATE",2);
									nvs_commit(nvs_handle);		
									nvs_close(nvs_handle);

									nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
									nvs_get_u32(nvs_handle,"GW_SER_RATE",(uint32_t*)&tmp_bitrate);
//								nvs_get_u8(nvs_handle,"GW_BIN_FLAG",(uint8_t*)&binary_data_flag);
//								nvs_get_str(nvs_handle,"GW_EOL_STR",eolstr,&n);					// n is number of bytes read out...
									printf("Stored value: %ld\n",tmp_bitrate);
									nvs_close(nvs_handle);
									
									uart_set_baudrate(NMEA,serial_bitrate);
									printf("New uart rate = %ldb/s ",serial_bitrate);
									if (serial_autobaud_flag)							// using auto_serial_bitrate
										{
										printf("[Auto]");
										}
									printf("\n");									

									response_flag = SRVR_OK_RESPONSE;


									break;

								default:
									break;

								}
								
							}
						break;

					case SCMD_BINARY:		//  8 = set serial binary data mode (0,1,2)
// format:
//  bin | n
						search_flag = get_pipe(&p,mqtt_rx_msg);

						check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);
						printf("BIN: %s\n",listfield_str);

						if (check_flag)
							{				
							if (listfield_str[0] == '0')
								binary_data_flag = 0;
							else if (listfield_str[0] == '1')
								binary_data_flag = 1;
							else
								binary_data_flag = 2;		// values > 1 mean "auto mode"
//							else
//								printf("ERROR!\n");

// save to NVS \ FLASH ROM for next boot...
							printf("Saving to NVS\n");
							nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
//							nvs_set_u32(nvs_handle,"GW_SER_RATE",DFLT_EXT_SERIAL_BPS);
							nvs_set_u8(nvs_handle,"GW_BIN_FLAG",0);
//							nvs_set_str(nvs_handle,"GW_EOL_STR",tmpstr);
//							nvs_set_i8(nvs_handle,"GW_SER_RATE",2);
							nvs_commit(nvs_handle);		
							nvs_close(nvs_handle);
					
							nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
//							nvs_get_u32(nvs_handle,"GW_SER_RATE",(uint32_t*)&tmp_bitrate);
							nvs_get_u8(nvs_handle,"GW_BIN_FLAG",(uint8_t*)&i);
//							nvs_get_str(nvs_handle,"GW_EOL_STR",tmpstr,&n);					// n is number of bytes read out...
							printf("Stored value: %d\n",i);
							
							nvs_close(nvs_handle);

							if ((binary_data_flag == 1) || ((binary_data_flag == 2) && (auto_binary_data_flag == 1)))
								binary_mode_flag = 1;
							else
								binary_mode_flag = 0;

							response_flag = SRVR_OK_RESPONSE;
							}

						break;

					case SCMD_EOLSTR:		//  9 = End Of Line character string
// format:
//  eolstr | cccc
						search_flag = get_pipe(&p,mqtt_rx_msg);

						check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);
						printf("EOL: %s\n",listfield_str);
						if (check_flag)
							{		
// convert from 2 byte ASCII to binary byte values

							for (i=0;i<strlen(listfield_str);i=i+2)
								{
								eolstr[i/2] = asc2hex((unsigned char *)&listfield_str[i]);
								printf("%02X ",eolstr[i/2]);
								printf("%02X ",listfield_str[i]*16 + listfield_str[i+1]);	// NOT RIGHT!
								}
							eolstr[i/2] = 0x00;
							printf("\n");	
								
// save to NVS \ FLASH ROM for next boot...
							printf("Saving to NVS\n");
							nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
//								nvs_set_u32(nvs_handle,"GW_SER_RATE",serial_bitrate);
//							nvs_set_u8(nvs_handle,"GW_BIN_FLAG",0);
							nvs_set_str(nvs_handle,"GW_EOL_STR",eolstr);
//							nvs_set_i8(nvs_handle,"GW_SER_RATE",2);
							nvs_commit(nvs_handle);		
							nvs_close(nvs_handle);

							nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
//							nvs_get_u32(nvs_handle,"GW_SER_RATE",(uint32_t*)&tmp_bitrate);
//							nvs_get_u8(nvs_handle,"GW_BIN_FLAG",(uint8_t*)&binary_data_flag);
							nvs_get_str(nvs_handle,"GW_EOL_STR",tmp_str,&p);					// n is number of bytes read out...
							printf("Stored value:");
							for (i=0;i<p;i++)
								{
								printf("%02X ",tmp_str[i]);								
								}
							printf("\n");
							
							nvs_close(nvs_handle);

							response_flag = SRVR_OK_RESPONSE;
							}
						break;

					case SCMD_EOLCTRL:		// 10 = set serial End Of Line string removal on \ off
// format:
//  eolctl | ~on
//  eolctl | ~off
						search_flag = get_pipe(&p,mqtt_rx_msg);

						check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);
						printf("EOLCTL: %s\n",listfield_str);

						if (check_flag)
							{				
							cmd_arg = atoi(listfield_str);
							switch(cmd_arg)
								{
								case SARG_ON:
									printf("ON!");
									if (eol_remove_flag != 1)
										{
										eol_remove_flag = 1;
/*
										nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
										nvs_set_u8(nvs_handle,"GW_EOLREM_FLAG",(uint8_t)eol_remove_flag);
										nvs_commit(nvs_handle);		
										nvs_close(nvs_handle);
*/
										nvs_wr_u8("GW_VAR_LIST","GW_EOLREM_FLAG",NO_INX,(uint8_t)eol_remove_flag);
										}

									response_flag = SRVR_OK_RESPONSE;
									break;
								
								case SARG_OFF:
									printf("OFF!");
									if (eol_remove_flag != 0)
										{
										eol_remove_flag = 0;
/*
										nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
										nvs_set_u8(nvs_handle,"GW_EOLREM_FLAG",(uint8_t)eol_remove_flag);
										nvs_commit(nvs_handle);		
										nvs_close(nvs_handle);
*/
										nvs_wr_u8("GW_VAR_LIST","GW_EOLREM_FLAG",NO_INX,(uint8_t)eol_remove_flag);
										}

									response_flag = SRVR_OK_RESPONSE;
									break;
								
								default:
									response_flag = SRVR_ERR_RESPONSE;		// "ERR"
									break;
								
								}
								

							}
						break;

					case SCMD_TIMER:			// 11 = set message timer
// format:
//  tmr | gw  | 65535 		in sec
//  tmr | gps | 65535 		in sec
//  tmr | i2c | 65535 		in sec
//  tmr | tnk | 65535 		in sec
//  tmr | bt  | 65535 		in sec
//  tmr | lo  | 65535		in sec
						unsigned int n;
						unsigned char i;

						search_flag = get_pipe(&p,mqtt_rx_msg);

						check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);
						printf("TIMER: %s\n",listfield_str);

						if (check_flag)
							{				
							cmd_arg = atoi(listfield_str);
							switch(cmd_arg)
								{
								case SARG_GW:			//  9 = gateway sensor timer ID
									check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);
									n = atoi(listfield_str);
									gateway_sensor_time = n;
									gateway_sensor_timer = n;
									nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
									nvs_set_u16(nvs_handle,"GW_SENSOR_TMR",gateway_sensor_time);
									nvs_commit(nvs_handle);		
									nvs_close(nvs_handle);
									printf("gateway_sensor_timer updated\n");

									response_flag = SRVR_OK_RESPONSE;
									break;									
									
								case SARG_GPS:			// 10 = GPS sensor timer ID
									check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);
									n = atoi(listfield_str);
									gps_sensor_time = n;
									gps_sensor_timer = n;
									nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
									nvs_set_u16(nvs_handle,"GW_GPS_TMR",gps_sensor_time);
									nvs_commit(nvs_handle);		
									nvs_close(nvs_handle);
									printf("gps_sensor_timer updated\n");

									response_flag = SRVR_OK_RESPONSE;
									break;									
									
								case SARG_I2C:			// 11 = I2C sensor timer ID
									check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);
									n = atoi(listfield_str);
									i2c_sensor_time = n;
									i2c_sensor_timer = n;
									nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
									nvs_set_u16(nvs_handle,"GW_I2C_TMR",i2c_sensor_time);
									nvs_commit(nvs_handle);		
									nvs_close(nvs_handle);
									printf("i2c_sensor_timer updated\n");

									response_flag = SRVR_OK_RESPONSE;
									break;									
									
								case SARG_TANK:			// 12 = Tank sensor timer ID
									check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);
									n = atoi(listfield_str);
									tank_sensor_time = n;
									tank_sensor_timer = n;
									nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
									nvs_set_u16(nvs_handle,"GW_TANK_TMR",tank_sensor_time);
									nvs_commit(nvs_handle);		
									nvs_close(nvs_handle);
									printf("tank_sensor_timer updated\n");

									response_flag = SRVR_OK_RESPONSE;
									break;									
									
								case SARG_BLUETOOTH:		// 13 = Bluetooth MAC address timer ID
#ifdef USE_BLUETOOTH
									check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);
									n = atoi(listfield_str);
									bluetooth_MAC_addr_time = n;
									bluetooth_MAC_addr_timer = n;
									nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
									nvs_set_u16(nvs_handle,"GW_BTADDR_TMR",bluetooth_MAC_addr_time);
									nvs_commit(nvs_handle);		
									nvs_close(nvs_handle);
									printf("bluetooth_MAC_addr_timer updated\n");

									response_flag = SRVR_OK_RESPONSE;
#endif									
									break;									
									
								case SARG_LORAWAN:		// 14 = LoRaWAN Dev Address timer ID
#ifdef USE_LORA
									check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);
									n = atoi(listfield_str);
									lora_devaddr_time = n;
									lora_devaddr_timer = n;
									nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
									nvs_set_u16(nvs_handle,"GW_LOADDR_TMR",lora_devaddr_time);
									nvs_commit(nvs_handle);		
									nvs_close(nvs_handle);
									printf("lora_devaddr_timer updated\n");

									response_flag = SRVR_OK_RESPONSE;

#endif								
									break;									

								case SARG_BT_DEVNAME:		// 13 = Bluetooth device name timer ID
#ifdef USE_BLUETOOTH
									check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);
									n = atoi(listfield_str);
									bluetooth_devname_time = n;
									bluetooth_devname_timer = n;
									nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
									nvs_set_u16(nvs_handle,"GW_BTDEVNM_TMR",bluetooth_devname_time);
									nvs_commit(nvs_handle);		
									nvs_close(nvs_handle);
									printf("bluetooth_devname_timer updated\n");

									response_flag = SRVR_OK_RESPONSE;
#endif									
									break;									
									
								default:
									response_flag = SRVR_ERR_RESPONSE;		// "ERR"
									break;

								}
								
							}
						break;

					case SCMD_DISCONN:		// 12 = forced disconnect
						printf("disconnecting...\n");
						mqtt_login_state = SRVR_DISCONNECT;	// need to latch state so it isnt overridden...
// send response to server
						response_flag = SRVR_OK_RESPONSE;
						break;

					case SCMD_PAIR:			// 13 = 
						printf("pairing...\n");
						response_flag = SRVR_OK_RESPONSE;
						break;

					case SCMD_REBOOT:		// 14 = 
						printf("rebooting...\n");
						esp_restart();
						response_flag = SRVR_OK_RESPONSE;
						break;

					case SCMD_SCAN:			// 15 = 
						printf("starting Bluetooth scan...\n");
#ifdef USE_BLUETOOTH
						if (strlen(mqtt_rx_msg) > 4)
							{						
							bt_scan_time = atoi(&mqtt_rx_msg[4]);
							printf("Scan time: %d\n",bt_scan_time);
							}
						else
							bt_scan_time = 300;
#endif
						response_flag = SRVR_OK_RESPONSE;

						break;

					case SCMD_PING:			// 16 = 
						response_flag = SRVR_OK_RESPONSE;	
						break;

					case SCMD_BDWLIST:		//  3 = Bluetooth Beacon \ Device name whitelist
#ifdef USE_BLUETOOTH
						bluetooth_beacon_whitelist_mode = WL_IDLE;
						printf("WLIST: ");
// format:
//  bwlist | ~on
//  bwlist | ~off
//  bwlist | ~add | $aaaaa | $bbbbbb etc
//  bwlist | ~rem | $aaaaa | $bbbbbb etc
										//	wlist | ~on  | $aaaaa | $bbbbbb etc
										//  wlist | ~off | $aaaaa | $bbbbbb etc
//  bwlist | ~rmall
//  bwlist | ~tx
						while (search_flag)
							{
							printf("NEW CMD: %s\n",mqtt_rx_msg);
//							search_flag = get_pipe(&p,mqtt_rx_msg);
							printf("ptr: %d  ",p);

							if (search_flag)
								{
								printf("CMD: %d  ",cmd);
// get argument string
								check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);
//								printf("Check: %d ",check_flag);
								if (check_flag)
									{
									if ((listfield_str[0] > 0x2F) && (listfield_str[0] < 0x3A))		// ie is numeric...
										cmd_arg = atoi(listfield_str);
									else
										cmd_arg = 0xFF;

									printf("ARG: %s  [%d]\n",listfield_str,cmd_arg);
									
										switch(cmd_arg)
											{
											case SARG_ON:		//  0 = turn listing ON
												printf("ON!");
												if (enable_bluetooth_devname_whitelist_flag != 1)
													{
													enable_bluetooth_devname_whitelist_flag = 1;
/*
													nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
													nvs_set_u8(nvs_handle,"GW_BDW_FLAG",(uint8_t)enable_bluetooth_devname_whitelist_flag);
													nvs_commit(nvs_handle);		
													nvs_close(nvs_handle);
*/
													nvs_wr_u8("GW_VAR_LIST","GW_BDW_FLAG",NO_INX,(uint8_t)enable_bluetooth_devname_whitelist_flag);
													}

												response_flag = SRVR_OK_RESPONSE;
												break;

											case SARG_OFF:			//  1 = turn listing OFF
												printf("OFF!");
												if (enable_bluetooth_devname_whitelist_flag != 0)
													{
													enable_bluetooth_devname_whitelist_flag = 0;
/*
													nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
													nvs_set_u8(nvs_handle,"GW_BDW_FLAG",(uint8_t)enable_bluetooth_devname_whitelist_flag);
													nvs_commit(nvs_handle);		
													nvs_close(nvs_handle);
*/
													nvs_wr_u8("GW_VAR_LIST","GW_BDW_FLAG",NO_INX,(uint8_t)enable_bluetooth_devname_whitelist_flag);
													}

												response_flag = SRVR_OK_RESPONSE;
												break;

											case SARG_ADD:			//  3 = ADD to list
												bluetooth_beacon_whitelist_mode = WL_ADD;
												break;

											case SARG_REMOVE:		//  4 = REMOVE to list
												bluetooth_beacon_whitelist_mode = WL_REM;
												break;

											case SARG_REMALL:		//  5 = REMOVE ALL from list
												printf("RMALL!");

												nvs_open("BD_MAC_LIST",NVS_READWRITE, &nvs_handle);

												for (unsigned char i=0;i<BLUETOOTH_DEVNAME_LIST_LENGTH;i++)
													{
		//											bluetooth_cmd_whitelist[i][0] = 0x00;
													bluetooth_devname_whitelist[i][0] = 0x00;						// delete the entry
													nvs_set_addr(nvs_handle,"BDW_MAC",i,(unsigned char*)list_empty_str,6);	// cant delete so set to 0
													}

												num_bt_devname_wl_entries = 0;
												nvs_set_i8(nvs_handle,"BDW_MAC_MAX",num_bt_devname_wl_entries);
												nvs_close(nvs_handle);

												response_flag = SRVR_OK_RESPONSE;
												break;

											case SARG_TX:			//  6 = send list to server
												printf("TX!");
												tx_bluetooth_devname_whitelist_flag = 1;

												response_flag = SRVR_OK_RESPONSE;
												break;

											default:				// ERROR
												response_flag = SRVR_ERR_RESPONSE;		// "ERR"
												break;
											}

										if (bluetooth_beacon_whitelist_mode != BL_IDLE)
										{
										unsigned char mac_addr[6];
// get remaining MAC addr string 										
										check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);
										printf("Name: %s\n",listfield_str);
										
										for (i=0;i<6;i++)
											{
											mac_addr[i] = asc2hex((unsigned char *)&listfield_str[2*i]);
											}												
//										printf("\nDATA: ");
										if (bluetooth_beacon_whitelist_mode == WL_ADD)
											{
											printf("ADDMODE: ");

											for (unsigned char i=0;i<BLUETOOTH_DEVNAME_LIST_LENGTH;i++)
												{
//												printf("i: %d ",i);
//												if ((bluetooth_cmd_whitelist[i][0] == 0x00) && (strlen(listfield_str) <= (BLUETOOTH_CMD_LIST_ENTRY_LENGTH - 1)))	// if space and str is short enough to fit...
												if (strlen(bluetooth_devname_whitelist[i]) == 0)
													{
//													printf("Found empty entry...\n");
													if (strlen(listfield_str) < BLUETOOTH_DEVNAME_LIST_ENTRY_LENGTH)	// if space and str is short enough to fit...
														{
//													strcpy((char*)bluetooth_cmd_whitelist[i],listfield_str);			// copy the string into the table
														strcpy((char*)bluetooth_devname_whitelist[i],listfield_str);			// copy the string into the table
														num_bt_devname_wl_entries++;
														
// now add to NVS
														nvs_open("BD_MAC_LIST",NVS_READWRITE, &nvs_handle);
														nvs_set_addr(nvs_handle,"BDW_MAC",i,mac_addr,6);
														nvs_set_i8(nvs_handle,"BDW_MAC_MAX",num_bt_devname_wl_entries);
														nvs_close(nvs_handle);

														printf("[%s] added @ %d  [%d]",listfield_str,i,num_bt_devname_wl_entries);

														break;											// ... and end the loop
														}
													}
												else
													{
													printf("%s\n",bluetooth_devname_whitelist[i]);
													}
												}

											response_flag = SRVR_OK_RESPONSE;
											}
											
										if (bluetooth_beacon_whitelist_mode == WL_REM)
											{
											printf("REMMODE: ");
											for (unsigned char i=0;i<BLUETOOTH_DEVNAME_LIST_LENGTH;i++)
												{
//												printf("i: %d ",i);
												if (!strcmp((char*)bluetooth_devname_whitelist[i],listfield_str))			// if string found...
													{
//													bluetooth_cmd_whitelist[i][0] = 0x00;							// delete the entry
													(bluetooth_devname_whitelist[i][0] = 0x00);						// delete the entry
													num_bt_devname_wl_entries--;

// now update to NVS
													nvs_open("BB_MAC_LIST",NVS_READWRITE, &nvs_handle);
													nvs_set_addr(nvs_handle,"BDW_MAC",i,(unsigned char*)list_empty_str,6);	// cant delete so set to 0
													nvs_set_i8(nvs_handle,"BDW_MAC_MAX",num_bt_devname_wl_entries);
													nvs_close(nvs_handle);

													
													printf("%s removed @ %d  [%d]\n",listfield_str,i,num_bt_devname_wl_entries);
													break;											// ... and end the loop
													}
												}

											response_flag = SRVR_OK_RESPONSE;
											}
											
										bluetooth_beacon_whitelist_mode = WL_IDLE;
										}
										
									if (check_flag > 1)			// if this field was the last...
										search_flag = 0;

									printf("\n");
									}
								}
							}
						printf("\n");
#endif
						break;

					case SCMD_LIFT:			// 18 = lift system commands
						break;


					case SCMD_DATETIME:		//  19 = DATE and TIME from server
// FORMAT:  19|2206170930    (ie, 17/06/22 at 09:30)
						{
						char tmpstr[5];
// preset all values to fail if not received...	
// Then if it passes, all values must be in bounds...
						unsigned char srvr_year = 100;
						unsigned char srvr_month = 12;
						unsigned char srvr_day = 32;
						unsigned char srvr_hrs = 24;
						unsigned char srvr_mins = 60;
						unsigned char srvr_day_of_week = 7;
						
						printf("Date / Time!\n");
						check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);
						strncpy(tmpstr,listfield_str,2);		// year
						srvr_year = atoi(tmpstr);
						strncpy(tmpstr,&listfield_str[2],2);	// month
						srvr_month = atoi(tmpstr);
						strncpy(tmpstr,&listfield_str[4],2);	// day
						srvr_day = atoi(tmpstr);
						strncpy(tmpstr,&listfield_str[6],2);	// hrs
						srvr_hrs = atoi(tmpstr);
						strncpy(tmpstr,&listfield_str[8],2);	// mins
						srvr_mins = atoi(tmpstr);
						strncpy(tmpstr,&listfield_str[10],3);	// mins

//						srvr_day_of_week = 0xFF;
						for (unsigned char z=0;z<7;z++)
							{
 #define strnicmp  strncasecmp
							if (!strnicmp(tmpstr,days_of_week[z],3))
								{
								srvr_day_of_week = z;
								}
							}
						
						printf("Srvr date/time: %s, %02d/%02d/20%02d %02d:%02d\n",days_of_week[srvr_day_of_week],srvr_day,srvr_month,srvr_year,srvr_hrs,srvr_mins);
// set system time and date:
						if ((srvr_year < 100) && (srvr_month < 12) && (srvr_day < 32) && (srvr_hrs < 24) && (srvr_mins < 60) && (srvr_day_of_week < 7)) 
							{
							year = srvr_year;						
							month = srvr_month;						
							days = srvr_day;
							hrs = srvr_hrs;
							mins = srvr_mins;
							secs = 0;
							day_of_week = srvr_day_of_week;
							
// show time has been sync'ed to server...						
							ntptime_sync_flag = 1;
// send OK response to server
							response_flag = SRVR_OK_RESPONSE;
							test_server_time_flag = 1;
							}
						else
								printf("### bad srvr time!\n");
							
						}
						break;

#if defined(USE_LORA) || defined(USE_LORA_LINK)
					case SCMD_LORA:			//  20 = LoRa controls
						{
						
//						search_flag = get_pipe(&p,mqtt_rx_msg);

						check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);
						printf("LoRa: %s\n",listfield_str);

						if (check_flag)
							{	
							unsigned int n;
							unsigned char i,j;
							unsigned char val,outval;
							char tmpstr[15];
							esp_err_t err;
							
							j = 1;		// preset 1 event to change...
							
//							search_flag = get_pipe(&p,mqtt_rx_msg);
							cmd_arg = atoi(listfield_str);
//							printf("CMD: [%s] [%s] %d\n",mqtt_rx_msg,listfield_str,cmd_arg);

							check_flag = get_listfield(&p,mqtt_rx_msg,listfield_str);
							n = atoi(listfield_str);
#define LORA_SRVR_DBG	
// if any LoRa values changed, save to NVS...
							err = nvs_open("LORA_REG_LIST",NVS_READWRITE, &nvs_handle);
						
							switch(cmd_arg)
								{
								case SARG_LORA_BANDWIDTH:		// 20|32 = set LoRa bandwidth
// bandwidth bits (b7-b4), values 0-9					
									if (n < 10)
										{
										err = SX1276_read_reg(spi, LREG_MDM_CFG1,&val,1);
										outval = (val & 0x0F) | (n<<4);
										err = SX1276_write_reg(spi, LREG_MDM_CFG1,&outval,1);
										sprintf(tmpstr,"LORA_REG_%02d",LREG_MDM_CFG1);
										nvs_set_u8(nvs_handle,tmpstr,outval);

#ifdef LORA_SRVR_DBG
										printf("LREG_MDM_CFG1: %02X %02X %d\n",val,outval,err);	
#endif
// send OK response to server
										response_flag = SRVR_OK_RESPONSE;
										}
									
									break;
								case SARG_LORA_CODING_RATE:	// 33 = set LoRa bandwidth
// coding rate bits (b3-b1), values 1-4
									if ((n>0) && (n < 5))
										{
										err = SX1276_read_reg(spi, LREG_MDM_CFG1,&val,1);
										outval = (val & 0xF1) | (n<<1);
										err = SX1276_write_reg(spi, LREG_MDM_CFG1,&outval,1);
										sprintf(tmpstr,"LORA_REG_%02d",LREG_MDM_CFG1);
										nvs_set_u8(nvs_handle,tmpstr,outval);
#ifdef LORA_SRVR_DBG
										printf("LREG_MDM_CFG1: %02X %02X %d\n",val,outval,err);	
#endif
// send OK response to server
										response_flag = SRVR_OK_RESPONSE;
										}
									break;
								case SARG_LORA_SPREAD_FACTOR:	// 34 = set LoRa spread factor
// spreading factor (b7-b4), values 6-12
									if ((n>5) && (n < 13))
										{
										err = SX1276_read_reg(spi, LREG_MDM_CFG2,&val,1);
										outval = (val & 0x0F) | (n<<4);
										err = SX1276_write_reg(spi, LREG_MDM_CFG2,&outval,1);
										sprintf(tmpstr,"LORA_REG_%02d",LREG_MDM_CFG2);
										nvs_set_u8(nvs_handle,tmpstr,outval);
#ifdef LORA_SRVR_DBG
										printf("LREG_MDM_CFG2: %02X %02X %d\n",val,outval,err);	
#endif
// send OK response to server
										response_flag = SRVR_OK_RESPONSE;
										}
									break;
								case SARG_LORA_MAX_POWER:		// 35 = set LoRa tx max powor
// 	max power bits (b6-b4)					
									if (n < 8)
										{
										err = SX1276_read_reg(spi, LREG_PA_CFG,&val,1);
										outval = (val & 0x8F) | (n<<4);
										outval = outval | 0x80;		// PA BOOST bit must be set to make the LoRa output work!
										err = SX1276_write_reg(spi, LREG_PA_CFG,&outval,1);
										sprintf(tmpstr,"LORA_REG_%02d",LREG_PA_CFG);
										nvs_set_u8(nvs_handle,tmpstr,outval);
#ifdef LORA_SRVR_DBG
										printf("LREG_PA_CFG: %02X %02X %d\n",val,outval,err);	
#endif
// send OK response to server
										response_flag = SRVR_OK_RESPONSE;
										}
									break;
								case SARG_LORA_TX_POWER:		// 36 = set LoRa Tx power
// 	output power bits (b3-b0)					
									if (n < 16)
										{
										err = SX1276_read_reg(spi, LREG_PA_CFG,&val,1);
										outval = (val & 0xF0) | n;
										outval = outval | 0x80;		// PA BOOST bit must be set to make the LoRa output work!
										err = SX1276_write_reg(spi, LREG_PA_CFG,&outval,1);
										sprintf(tmpstr,"LORA_REG_%02d",LREG_PA_CFG);
										nvs_set_u8(nvs_handle,tmpstr,outval);
#ifdef LORA_SRVR_DBG
										printf("LREG_PA_CFG: %02X %02X %d\n",val,outval,err);	
#endif
										}
// send OK response to server
										response_flag = SRVR_OK_RESPONSE;
									break;
									
								case SARG_LORA_GET_BW_CODING:	// 36 = get LoRa bandwidth and coding rate
									err = SX1276_read_reg(spi, LREG_MDM_CFG1,&val,1);
									
									srvr_return_cmd = cmd;
									srvr_return_cmd_arg = cmd_arg;
									srvr_return_data_type = SRVR_RETURN_U8;
									srvr_return_value = val;
									srvr_return_flag = 1;
// send OK response to server
										response_flag = SRVR_OK_RESPONSE;
									break;
								case SARG_LORA_GET_SPREAD:	// 38 = get spread factor
									err = SX1276_read_reg(spi, LREG_MDM_CFG2,&val,1);

									srvr_return_cmd = cmd;
									srvr_return_cmd_arg = cmd_arg;
									srvr_return_data_type = SRVR_RETURN_U8;
									srvr_return_value = val;
									srvr_return_flag = 1;
// send OK response to server
										response_flag = SRVR_OK_RESPONSE;
									break;
								case SARG_LORA_GET_TX_POWER:	// 37 = get LoRa Tx power and max power
									err = SX1276_read_reg(spi, LREG_PA_CFG,&val,1);

									srvr_return_cmd = cmd;
									srvr_return_cmd_arg = cmd_arg;
									srvr_return_data_type = SRVR_RETURN_U8;
									srvr_return_value = val;
									srvr_return_flag = 1;
// send OK response to server
										response_flag = SRVR_OK_RESPONSE;
									break;

								default:
									printf("Unknown LoRa server arg %d\n",cmd_arg);
									response_flag = SRVR_ERR_RESPONSE;		// "ERR"
									break;
								}
// if any LoRa values changed, save to NVS...
							nvs_commit(nvs_handle);
							nvs_close(nvs_handle);

							}
						}							
						break;
#endif	// end of "#if defined(USE_LORA) || defined(USE_LORA_LINK)..."

					case SCMD_ERROR:		//  0 = ERROR
					default:
						printf("Unknown server command %d\n",cmd);
						response_flag = SRVR_ERR_RESPONSE;		// "ERR"
						break;
					}

				}
				
			}
				
// send server response
			if (response_flag)
				{
				strcpy(response_str,mqtt_rx_msg);
				strcat(response_str,"|");

				if (response_flag == SRVR_OK_RESPONSE)
					{
					strcat(response_str,"OK");
					}
				if (response_flag == SRVR_ERR_RESPONSE)
					{
					strcat(response_str,"ERR");
					}
				response_flag = SRVR_NO_RESPONSE;
				server_response_flag = SRVR_OK_RESPONSE;
				}
				
//			CMQTT_rx_msg = 0;// changed 25/07/23

//			CMQTT_rx_msg--;	// allows for multiple queued CMQTT msgs

			if (server_cmd_test_flag)
				{
				printf("\nSCT: %s",mqtt_rx_msg);
				if (response_flag == 1)
					printf(" : OK");
				else if (response_flag == 2)
					printf(" : ERR");

				printf("\tDONE\n");

				server_cmd_test_flag = 0;
				}
// only decrement CMQTT_rx_msg count if it wasnt a test msg...
			else
				{
				CMQTT_rx_msg--;	// allows for multiple queued CMQTT msgs
				}

			
			}	// end of "if (CMQTT_rx_msg)...
	

// restart 4G using state machine...
#ifdef USE_M4G
//	if (!mqtt_disable_flag)
		{
//		if (four_g_state == M4G_POWER_OFF)
		if (simcom_disable_flag)
			{
//			mqtt_login_state = NOT_SRVR_INITIALISED;
			simcom_state = SIMCOM_POWER_OFF;
			}
		else
			{
			if (simcom_state == SIMCOM_POWER_OFF)
				{
//				mqtt_login_state = NOT_SRVR_INITIALISED;
//			four_g_state = M4G_POWER_EN;
				simcom_state = SIMCOM_POWER_EN;
				}
			}
		}
#if 0
	else
		{		
//		if ((four_g_state < M4G_POWER_DOWN_KEY) && (four_g_state != M4G_POWER_OFF))
		if ((simcom_state < SIMCOM_POWER_DOWN_KEY) && (simcom_state != SIMCOM_POWER_OFF))
			{
			printf("SIMCOM Power Down...\n");
			mqtt_login_state = NOT_SRVR_INITIALISED;
//			four_g_state = M4G_POWER_DOWN_KEY;
// mqtt_state = MQTT_UNSUBSCRIBE_TOPIC;	// MQTT_DISCONNECT;
			simcom_state = SIMCOM_POWER_DOWN_KEY;
			}
		}
#endif

#endif

#ifdef USE_M4G_MQTT
// if SIMCOM ready, start MQTT...
	if (mqtt_disable_flag)
		{
		mqtt_state = MQTT_NO_COMM_IDLE;	
		simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
		}
	else if ((simcom_state == SIMCOM_READY_IDLE) && (mqtt_state == MQTT_NO_COMM_IDLE))
		{
		printf("Starting MQTT...\n");
		mqtt_state = MQTT_COMM_INIT;		// done in mqtt_login state mc...

		}
#endif

#ifdef USE_M4G_UDP
	if (udp_disable_flag)
		{
		udp_state = UDP_NO_COMM_IDLE;	
		simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_UDP);
		}
		
// if SIMCOM ready, start UDP...
	else if ((simcom_state == SIMCOM_READY_IDLE) && (udp_state == UDP_NO_COMM_IDLE))
		{
		printf("Starting UDP...\n");
		udp_state = UDP_COMM_INIT_2;	
		}
		
// test the UDP 
	else if (udp_state == UDP_COMM_IDLE_2)
		{
		printf("Sending UDP msg...\n");
		strcpy(udp_payload_str,"Hello UDP world!\n");
		udp_state = UDP_SEND;
		}
#endif

#ifdef USE_WIFI_MQTT
	if ((wifi_disable_flag) || (wifi_mqtt_disable_flag))
		{
		if (wifi_mqtt_started)
			{
			wifi_mqtt_stop();
			}
		}
		
	if ((wifi_disable_flag == 0) && (wifi_mqtt_disable_flag == 0))
		{
		if ((wifi_mqtt_started == 0) && (wifi_sta_connected))
			{
			wifi_mqtt_start();
			}
		}
#endif

///////////////////////
// set up wifi UDP
///////////////////////
#ifdef USE_WIFI_UDP
if (wifi_udp_DNS_timer >= WIFI_UDP_DNS_TIMEOUT)	// time to re-check DNS lookup!
	{
	printf("DNS refresh started...\n");	
	wifi_udp_DNS_timer = 0;		// set timer running again
	wifi_udp_DNSstart = 0;		// re-start DNS lookup
	wifi_udp_DNSfound = 0;
	prev_wifi_udp_DNSfound = 0;
	}
	
if ((wifi_sta_connected) && (!wifi_udp_DNSfound) && (!wifi_udp_DNSstart))
	{
	printf("attempting to resolve wifi udp DNS...\n");
	dns_gethostbyname(wifi_udp_url, &wifi_udp_ip_addr, wifi_udp_dns_found_cb, NULL );	
	wifi_udp_DNSstart = 1;		// only do once...
	}

#endif


////////////////////////////////////////////////////////////////////////////////////////////
//
//  Gateway to Server messaging
//
////////////////////////////////////////////////////////////////////////////////////////////

// system rests in M4G_POWER_OFF state
// manual change to M4G_POWER_EN starts 4G power up 
// system rests in M4G_NO_COMM_IDLE state
// manual change to M4G_COMM_INIT starts 4G connect
// system rests in M4G_CONNECT_IDLE state

	if (prev_mqtt_login_state != mqtt_login_state)
		{
		if (debug_do(DBG_MQTT))
			printf("MQTT login state: [%03d]  %s\n",mqtt_login_state,mqtt_login_state_str[mqtt_login_state]);
		
		prev_mqtt_login_state = mqtt_login_state;
		}
		
	switch(mqtt_login_state)
		{
			case GET_NEW_CERTIFICATES:
				mqtt_login_state = GET_NEW_CERTIFICATES_WAIT;
				break;
				
			case GET_NEW_CERTIFICATES_WAIT:
				mqtt_login_state = NOT_SRVR_INITIALISED;
				break;
				
			case NOT_SRVR_INITIALISED:
//			{
#ifdef SIMCOM_PROGRAMMING_MODE
			if (0)			// just power up SIMCOM module and then wait for programming to occur...
#else
//			if ((simcom_state == SIMCOM_READY_IDLE))	// && (mqtt_state == MQTT_NO_COMM_IDLE))

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (simcom_state == SIMCOM_READY_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
#endif
				{
//					printf("MAC str is %s\r\n",mac_bin_str);
					
//*****************************************
//*                                       *
//   Setting payloads for MQTT CONNECT    *
//*                                       *
//*****************************************
// set topic and msg payload for login
				
				print_banner_msg("Setting payloads for MQTT CONNECT");
#ifdef AQL_SERVER
				strcpy(mqtt_topic_str,"/gateway_dom/");	//node/1");
				strcat(mqtt_topic_str,mac_bin_str);		// UID from server
//				strcat(mqtt_topic_str,"/1");			// NODE ID=1 for on-board sensors

//			printf("topic str is %s\r\n",mqtt_topic_str);

//			strcat(mqtt_topic_str,"001122334455");	//node/1");
#endif
#ifdef SSS_SERVER
				strcpy(mqtt_topic_str,"/dom/100234/frame/node/1");
#endif
#ifdef MOSQUITTO_SERVER
				strcpy(mqtt_topic_str,"/dom/100234/frame/node/1");
#endif

				printf("Sending to topic: %s\n",mqtt_topic_str);
				
// set data payload for login
#ifdef AQL_SERVER
// initialisation string - MAC addr, Model str, Hw version, SW version			
// should only be necessary once, to allow the server to cre4ate a record for this unit 
// and send back a unique ID to be stored in NVS
//			strcpy(module_mac_str,"001122334455");
//			sprintf(mqtt_payload_str,"1|%04X|2|%02X",supply_voltage,mins);	// sensor-num | value, :repeat...
				sprintf(mqtt_payload_str,"%s|aql-bwc01|%d.%d.%c|%d.%d.%d",mac_bin_str,HW_MAJ,HW_MIN,HW_SUB,VER_MAJ,VER_MIN,VER_SUB);
					
				mqtt_payload_length = strlen(mqtt_payload_str);
				
				printf("Login Payload = %s \r\n",mqtt_payload_str);
#endif
#ifdef SSS_SERVER
				mqtt_payload_str[0] = 0x03;
				mqtt_payload_str[1] = 0xCD;
				mqtt_payload_str[2] = 0xCC;
				mqtt_payload_str[3] = 0xC8;
				mqtt_payload_str[4] = 0x41;
					
				mqtt_payload_length = 5;	//strlen(mqtt_payload_str);
#endif
#ifdef MOSQUITTO_SERVER
				mqtt_payload_str[0] = 0x03;
				mqtt_payload_str[1] = 0xCD;
				mqtt_payload_str[2] = 0xCC;
				mqtt_payload_str[3] = 0xC8;
				mqtt_payload_str[4] = 0x41;
					
				mqtt_payload_length = 5;	//strlen(mqtt_payload_str);
#endif

//				mqtt_state = MQTT_COMM_INIT;

//				mqtt_login_state = SRVR_INIT_WAIT;


#ifdef USE_WIFI_MQTT
				if(mqtt_transport_mode == MQTT_MODE_WIFI)
// send via wifi
					{
					printf("Sending via Wifi MQTT...\n");
					esp_mqtt_client_publish(mqtt_client, mqtt_topic_str, mqtt_payload_str, 0, 0, 0);	// blocking version
					printf("Wifi MQTT send done.\n");

					}
#endif

#ifdef USE_M4G_MQTT
				if (mqtt_transport_mode == MQTT_MODE_4G)
// send via 4G	
					{
					mqtt_state = MQTT_COMM_INIT;
					}
#endif
					
				mqtt_login_state = SRVR_INIT_WAIT;

				}
//			}
			break;
		case SRVR_INIT_WAIT:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				mqtt_login_state = NOT_SRVR_CONNECTED;
			break;
			
		case NOT_SRVR_CONNECTED:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
// set topic and msg payload for log in
//*********************************************
//*                                           *
//*  Setting payloads for MQTT SERVER LOGIN   *
//*                                           *
//*********************************************

				print_banner_msg("Setting payloads for MQTT SERVER LOGIN");

#ifdef AQL_SERVER
				strcpy(mqtt_topic_str,"/gateway_dom/");	//node/1");
				strcat(mqtt_topic_str,mac_bin_str);	//node/1");		// UID
#endif
#ifdef SSS_SERVER
				strcpy(mqtt_topic_str,"/dom/100234/frame/node/1");
#endif
#ifdef MOSQUITTO_SERVER
				strcpy(mqtt_topic_str,"/dom/100234/frame/node/1");
#endif

//				printf("Sending to topic: %s\n",mqtt_topic_str);


#ifdef AQL_SERVER
// logon string - "connect"
				sprintf(mqtt_payload_str,"connect");	
				mqtt_payload_length = strlen(mqtt_payload_str);
#endif
#ifdef SSS_SERVER
// logon string - "connect"
				sprintf(mqtt_payload_str,"connect");
				mqtt_payload_length = strlen(mqtt_payload_str);
#endif
#ifdef MOSQUITTO_SERVER
// logon string - "connect"
				sprintf(mqtt_payload_str,"connect");
				mqtt_payload_length = strlen(mqtt_payload_str);
#endif
				
				printf("Payload: %s\n",mqtt_payload_str);
				
//				mqtt_state = MQTT_SET_TOPIC;				
//				mqtt_login_state = SRVR_CONNECT_WAIT;	//SRVR_CONNECTED;	// NEED TO CHECK WAS SUCCESSFUL!

				mqtt_send(mqtt_transport_mode,SRVR_CONNECT_WAIT);
				}

			break;
		case SRVR_CONNECT_WAIT:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				mqtt_login_state = UPDATE_SRVR_INFO;	//SRVR_CONNECTED;	// NEED TO CHECK WAS SUCCESSFUL!
			break;

		case UPDATE_SRVR_INFO:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{

//*************************************
//*                                   *
//   Setting payloads for SRVR INFO   *
//*                                   *
//*************************************
// set topic and msg payload for login

				print_banner_msg("Setting payloads for SERVER INFO");

#ifdef AQL_SERVER
				strcpy(mqtt_topic_str,"/gateway_dom/meta/");	//node/1");
				strcat(mqtt_topic_str,mac_bin_str);		// UID from server
#endif		
#ifdef SSS_SERVER
				strcpy(mqtt_topic_str,"/dom/100234/frame/node/1");
#endif
#ifdef MOSQUITTO_SERVER
				strcpy(mqtt_topic_str,"/dom/100234/frame/node/1");
#endif

//				printf("Sending to topic: %s\n",mqtt_topic_str);

#ifdef AQL_SERVER
// logon string - "connect"
				strcpy(mqtt_payload_str,"imei|");
				strcat(mqtt_payload_str,imei_str);
				strcat(mqtt_payload_str,"|sim|");
				strcat(mqtt_payload_str,simnum_str);
					
				mqtt_payload_length = strlen(mqtt_payload_str);
#endif
#ifdef SSS_SERVER
				mqtt_payload_str[0] = 0x03;
				mqtt_payload_str[1] = 0xCD;
				mqtt_payload_str[2] = 0xCC;
				mqtt_payload_str[3] = 0xC8;
				mqtt_payload_str[4] = 0x41;
					
				mqtt_payload_length = 5;	//strlen(mqtt_payload_str);
#endif
#ifdef MOSQUITTO_SERVER
				mqtt_payload_str[0] = 0x03;
				mqtt_payload_str[1] = 0xCD;
				mqtt_payload_str[2] = 0xCC;
				mqtt_payload_str[3] = 0xC8;
				mqtt_payload_str[4] = 0x41;
					
				mqtt_payload_length = 5;	//strlen(mqtt_payload_str);
#endif

//				mqtt_state = MQTT_SET_TOPIC;
//				mqtt_login_state = UPDATE_SRVR_INFO_WAIT;	// NEED TO CHECK WAS SUCCESSFUL!


				mqtt_send(mqtt_transport_mode,UPDATE_SRVR_INFO_WAIT);

				}
			break;
			
		case UPDATE_SRVR_INFO_WAIT:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
//				mqtt_login_state = GATEWAY_SENSOR_DATA;	//SRVR_CONNECTED;	// NEED TO CHECK WAS SUCCESSFUL!
				mqtt_login_state = SENSOR_DATA_IDLE;
			break;
// DATA types:
// ===========
// NMEA data
// BlueTooth data
// BlueTooth scan data
// LORA sensor data
// LORA scan data
// Gateway sensor data
// GPS data
// I2C sensor data
// Tank sensor data


///////////////////////////////////////////
//
//  Gateway to Server message scheduling
//
///////////////////////////////////////////

		case SENSOR_DATA_IDLE:
// wait here and then assign next sensor data to be transmitted...
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
				if (server_response_flag)													// makes it possible to use #ifdefs below...
					mqtt_login_state = SEND_CMD_RESPONSE;

				else if (srvr_return_flag)
					mqtt_login_state = SEND_RETURN_VALUE;
					
#ifdef USE_BLUETOOTH
				else if (tx_bluetooth_whitelist_flag)							// bluetooth data available
					mqtt_login_state = GET_BT_WHITELIST_TABLE;

				else if (tx_bluetooth_blacklist_flag)							// bluetooth data available
					mqtt_login_state = GET_BT_BLACKLIST_TABLE;

#ifndef INHIBIT_BT_SENSOR_DATA
				else if ((bt_msg_ready_count > (BT_BUF_SIZE*3/4))  || (bluetooth_sensor_timer >=bluetooth_sensor_time))							// bluetooth data available
					mqtt_login_state = BT_SENSOR_DATA;
#endif

#ifdef BLUETOOTH_SCAN
				else if (bluetooth_MAC_addr_timer >= bluetooth_MAC_addr_time)	//BLUETOOTH_MAC_ADDR_TIME)		// Bluetooth data available
					mqtt_login_state = BT_MAC_ADDR_DATA;
#endif
				else if (bluetooth_devname_timer >= bluetooth_devname_time)	//BLUETOOTH_MAC_ADDR_TIME)		// Bluetooth data available
					mqtt_login_state = BT_DEVNAME_DATA;

#endif

#ifdef USE_LORA
				else if (tx_lorawan_whitelist_flag)							// LORA data available
					mqtt_login_state = GET_LORA_WHITELIST_TABLE;

				else if (tx_lorawan_blacklist_flag)							// LORA data available
					mqtt_login_state = GET_LORA_BLACKLIST_TABLE;

				else if (lora_msg_ready_count)							// LORA data available
					mqtt_login_state = LORA_SENSOR_DATA;

#ifdef LORAWAN_SCAN
				else if (lora_devaddr_timer >= lora_devaddr_time)	//LORA_DEVADDR_TIME)		// LORA data available
					mqtt_login_state = LORA_DEVADDR_DATA;
#endif
#endif

#ifdef WIFI_SCAN
				else if (wifi_scan_valid)							//LORA_DEVADDR_TIME)		// LORA data available
					mqtt_login_state = WIFI_SCAN_DATA;
#endif
				
				else if ((mqtt_transport_mode == MQTT_MODE_4G) && (gps_sensor_timer >= gps_sensor_time))	//GPS_SENSOR_TIME)
					mqtt_login_state = GET_GPS_INFO;
				
				else if (gateway_sensor_timer >= gateway_sensor_time)	//GATEWAY_SENSOR_TIME)
					mqtt_login_state = GATEWAY_SENSOR_DATA;

				else if (get_server_time_flag)
					mqtt_login_state = SEND_GET_TIME_REQUEST;

#ifdef USE_I2C
				else if (i2c_sensor_timer >= i2c_sensor_time)	//I2C_SENSOR_TIME)
					mqtt_login_state = I2C_SENSOR_DATA;
#endif
#ifdef USE_TANK_SENSORS
				else if (tank_sensor_timer >= tank_sensor_time)	//TANK_SENSOR_TIME)
					mqtt_login_state = TANK_SENSOR_DATA;
#endif

				else if (net_test_flag)
					mqtt_login_state = NET_4G_REPORT;

				else if (mqtt_transport_mode == MQTT_MODE_4G)
					mqtt_login_state = IDLE_CHECK_SIGNAL_QUALITY;		// can only check 4G signal quality in 4G mode!
				}
			break;

		case SEND_GET_TIME_REQUEST:	
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
//*********************************************
//*                                           *
//   Setting payloads for Get Time Request    *
//*                                           *
//*********************************************
// set topic and msg payload for response to server command

				print_banner_msg("Setting payloads for GET TIME REQUEST");

#ifdef AQL_SERVER
				strcpy(mqtt_topic_str,"/gateway_dom/frame/");	//node/1");
				strcat(mqtt_topic_str,mac_bin_str);	//node/1");
#endif	
//				printf("Sending to topic: %s\n",mqtt_topic_str);

				printf("Requesting date and time from server...\n");
						
				sprintf(mqtt_payload_str,"get server time");
				mqtt_payload_length = strlen(mqtt_payload_str);

				get_server_time_flag = 0;

//				if(mqtt_transport_mode == MQTT_MODE_WIFI)
//				esp_mqtt_client_publish(client, "/topic/test3", "Hello World", 0, 0, 0);	
//				esp_mqtt_client_enqueue(client, "/topic/test3", "Hello World", 0, 0, 0, 0);	// non-blocking version of above
//				esp_mqtt_client_enqueue(mqtt_client, mqtt_topic_str, mqtt_payload_str, 0, 0, 0, 0);	// non-blocking version of above
//				else
//##				mqtt_state = MQTT_SET_TOPIC;
					
//##				mqtt_login_state = GATEWAY_SENSOR_DATA_WAIT;		// go even if no data ready from gateway

				mqtt_send(mqtt_transport_mode,MQTT_SEND_WAIT);

				}
			break;
			
		case SEND_CMD_RESPONSE:	
			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
				server_response_flag = SRVR_NO_RESPONSE;

//*********************************************
//*                                           *
//   Setting payloads for Response To Server  *
//*                                           *
//*********************************************
// set topic and msg payload for response to server command

				print_banner_msg("Setting payloads for RESPONSE TO SERVER");

#ifdef AQL_SERVER
				strcpy(mqtt_topic_str,"/gateway_dom/frame/");	//node/1");
				strcat(mqtt_topic_str,mac_bin_str);	//node/1");
#endif	
//				printf("Sending to topic: %s\n",mqtt_topic_str);
			
				strcpy(mqtt_payload_str,response_str);
				mqtt_payload_length = strlen(mqtt_payload_str);				

//##				mqtt_state = MQTT_SET_TOPIC;
					
//##				mqtt_login_state = GATEWAY_SENSOR_DATA_WAIT;		// go even if no data ready from gateway				

				mqtt_send(mqtt_transport_mode,MQTT_SEND_WAIT);
				}
			break;

		case SEND_RETURN_VALUE:
			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
				char tmpstr[30];
				
				srvr_return_flag = 0;

//*************************************************
//*                                               *
//   Setting payloads for Return Value To Server  *
//*                                               *
//*************************************************

				print_banner_msg("Setting payloads for RETURN VALUE TO SERVER");

#ifdef AQL_SERVER
				strcpy(mqtt_topic_str,"/gateway_dom/frame/");	//node/1");
				strcat(mqtt_topic_str,mac_bin_str);	//node/1");
				
				sprintf(mqtt_payload_str,"55|%d|%d|%d|",srvr_return_cmd,srvr_return_cmd_arg,srvr_return_data_type);	// 

				switch(srvr_return_data_type)
					{
					case SRVR_RETURN_STR:
						sprintf(tmpstr,"%s",srvr_return_str);	// sensor-num | value, :repeat...
						break;
					case SRVR_RETURN_U8:
					case SRVR_RETURN_S8:
						sprintf(tmpstr,"%02X",srvr_return_value);	// sensor-num | value, :repeat...
						break;
					case SRVR_RETURN_U16:
					case SRVR_RETURN_S16:
						sprintf(tmpstr,"%04X",srvr_return_value);	// sensor-num | value, :repeat...
						break;

					}

				strcat(mqtt_payload_str,tmpstr);
				mqtt_payload_length = strlen(mqtt_payload_str);
				
#endif	
			
				mqtt_send(mqtt_transport_mode,MQTT_SEND_WAIT);
				}
			break;
		
		case GATEWAY_SENSOR_DATA:	//SRVR_CONNECTED:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
//				if (mqtt_sensor_msg_flag)			// if sensor report timer has expired...
					{
//					mqtt_sensor_msg_flag = 0;
					gateway_sensor_timer = 0;
					
//*********************************************
//*                                           *
//   Setting payloads for Gateway DATA        *
//*                                           *
//*********************************************
// set topic and msg payload for sensor msg

					print_banner_msg("Setting payloads for GATEWAY SENSOR DATA");

#ifdef AQL_SERVER
					strcpy(mqtt_topic_str,"/gateway_dom/frame/");	//node/1");
					strcat(mqtt_topic_str,mac_bin_str);	//node/1");
#endif
#ifdef SSS_SERVER
					strcpy(mqtt_topic_str,"/dom/100234/frame/node/1");
#endif
#ifdef MOSQUITTO_SERVER
					strcpy(mqtt_topic_str,"/dom/100234/frame/node/1");
#endif

//				printf("Sending to topic: %s\n",mqtt_topic_str);

#ifdef AQL_SERVER
// logon string - MAC addr, Model str, Hw version, SW version			
					sprintf(mqtt_payload_str,"1|%04X|2|%04X",adc_val_raw_sv,up_time);	// sensor-num | value, :repeat...
// gps lat,lon, string of 11 chars+ 1 char N\S or E\W
// gps spd in knots (u_int?)
/*
					if ((strlen(gps_lat_str) == 0) || (strlen(gps_lon_str) == 0))
						sprintf(tmp_str,"|6|%03ld.%03ld|7|%d",lora_radio_freq/1000,lora_radio_freq%1000,cell_rssi);
					else
*/
					sprintf(tmp_str,"|3|%s|4|%s|5|%02X|6|%03ld.%03ld|7|%d",gps_lat_str,gps_lon_str,gps_spd,lora_radio_freq/1000,lora_radio_freq%1000,cell_rssi);
					strcat(mqtt_payload_str,tmp_str);

#if 1
					sprintf(tmp_str,"|8|%d|9|%ld|10|%d|11|%d|12|%d|13|",auto_serial_bitrate,serial_bitrate,binary_mode_flag,binary_data_flag,eol_remove_flag);
					strcat(mqtt_payload_str,tmp_str);
					for (unsigned char n=0;n<1;n++)	//strlen(eolstr);n++)
						{
						sprintf(tmp_str,"%02X",eolstr[n]);
						strcat(mqtt_payload_str,tmp_str);
						}

					sprintf(tmp_str,"|14|%d|15|%d",0,0);
					strcat(mqtt_payload_str,tmp_str);
					sprintf(tmp_str,"|16|%d|17|%d",enable_bluetooth_whitelist_flag,enable_bluetooth_blacklist_flag);
					strcat(mqtt_payload_str,tmp_str);
					sprintf(tmp_str,"|18|%d|19|%d",enable_lorawan_whitelist_flag,enable_lorawan_blacklist_flag);
					strcat(mqtt_payload_str,tmp_str);

					sprintf(tmp_str,"|20|%d|21|%d|22|%d|23|%d|24|%d|25|%d",gateway_sensor_time,gps_sensor_time,i2c_sensor_time,tank_sensor_time,bluetooth_MAC_addr_time,lora_devaddr_time);
					strcat(mqtt_payload_str,tmp_str);

					sprintf(tmp_str,"|26|%d|27|%d",enable_bluetooth_devname_whitelist_flag,bluetooth_devname_time);
					strcat(mqtt_payload_str,tmp_str);

#ifdef USE_ADCS
					sprintf(tmp_str,"|57|%d|58|%d",adc_0,adc_1);
					strcat(mqtt_payload_str,tmp_str);
#endif
/*
					if (get_server_time_flag)
							printf("Requesting date and time from server...\n");
						
					sprintf(tmp_str,"|28|%d",get_server_time_flag);
					strcat(mqtt_payload_str,tmp_str);
					get_server_time_flag = 0;
*/					
#endif				
					mqtt_payload_length = strlen(mqtt_payload_str);
#endif
#ifdef SSS_SERVER
					sprintf(mqtt_payload_str,"1|%04X|2|%04X",adc_val_raw_sv,up_time);	// sensor-num | value, :repeat...
					mqtt_payload_length = strlen(mqtt_payload_str);
#endif
#ifdef MOSQUITTO_SERVER
					sprintf(mqtt_payload_str,"1|%04X|2|%04X",adc_val_raw_sv,up_time);	// sensor-num | value, :repeat...
					mqtt_payload_length = strlen(mqtt_payload_str);
#endif

//##					mqtt_state = MQTT_SET_TOPIC;
					}
					
//##				mqtt_login_state = GATEWAY_SENSOR_DATA_WAIT;		// go even if no data ready from gateway

				mqtt_send(mqtt_transport_mode,MQTT_SEND_WAIT);

				}
			break;

		case GATEWAY_SENSOR_DATA_WAIT:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
				mqtt_login_state = SENSOR_DATA_IDLE;
				}
			break;
/*
		case NMEA_SENSOR_DATA:
			break;
		case NMEA_SENSOR_DATA_WAIT:
			break;
			
		case GET_NMEA_WHITELIST_TABLE:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
				tx_nmea_whitelist_flag = 0;
//				mqtt_login_state = GET_BW_LIST_TABLE_WAIT;

				mqtt_send(mqtt_transport_mode,MQTT_SEND_WAIT);
				}
			break;
			
		case GET_NMEA_BLACKLIST_TABLE:
//			if (mqtt_state == MQTT_CONNECT_IDLE)	// M4G_NO_COMM_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
				tx_nmea_blacklist_flag = 0;
//##				mqtt_login_state = GET_BW_LIST_TABLE_WAIT;

				mqtt_send(mqtt_transport_mode,MQTT_SEND_WAIT);
				}
			break;

		case GET_SERIAL_SETTINGS:
//			if (mqtt_state == MQTT_CONNECT_IDLE)	// M4G_NO_COMM_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
				tx_serial_settings_flag = 0;
//##				mqtt_login_state = GET_BW_LIST_TABLE_WAIT;

				mqtt_send(mqtt_transport_mode,MQTT_SEND_WAIT);
				}
			break;
*/
		case GET_BW_LIST_TABLE_WAIT:
			if (mqtt_state == MQTT_CONNECT_IDLE)	// M4G_NO_COMM_IDLE)
				{
				mqtt_login_state = SENSOR_DATA_IDLE;
				}
			break;

		case BT_SENSOR_DATA:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
#ifdef USE_BLUETOOTH
				bluetooth_sensor_timer = 0;

				if (bt_msg_ready_count)
					{
					unsigned char bluetooth_payload_flag;
					unsigned int payload_left = MQTT_PAYLOAD_SIZE;
					unsigned int msgs_added = 0;

//*****************************************
//*                                       *
//* Setting payloads for BLUETOOTH DATA   *
//*                                       *
//*****************************************

					print_banner_msg("Setting payloads for BLUETOOTH DATA");

#ifdef AQL_SERVER
// extract BT MAC from payload data (first 6 bytes \ 12 chars):
//					memcpy(tmp_str,mqtt_payload_str,12);
//					tmp_str[12] = 0x00;
		
					strcpy(mqtt_topic_str,"/gateway_dom/frame/");	//
					strcat(mqtt_topic_str,mac_bin_str);				// gateway MAC addr
					strcat(mqtt_topic_str,"/bt");						//
//						strcat(mqtt_topic_str,tmp_str);					// BT device MAC addr
//						strcat(mqtt_topic_str,"/386");					//

// add Bluetooth MAC address to topic string... see later...

#endif

//					printf("Sending to topic: %s\n",mqtt_topic_str);
					
					mqtt_payload_str[0] = 0x00;
					mqtt_payload_length = 0;

					printf("Payload available: %d\n",payload_left);
					bluetooth_payload_flag = 1;

					while (bluetooth_payload_flag)
						{
						xQueuePeek(out_queue, bt_out_data, portMAX_DELAY);
						if (((2*bt_out_data->q_data_len) + 1) < payload_left)	// if there is space in the payload buffer...
																				// '+1' is for pipe separator
							{
// get payload from queue			
							if (xQueueReceive(out_queue, bt_out_data, portMAX_DELAY) != pdPASS) 
								{
								ESP_LOGE(TAG, "out_queue receive error");
								} 
							else
								{
//							unsigned char mplen = bt_out_data->q_data_len;
	// black, whitelisting now done in Bluetooth.cpp...
								unsigned char send_bluetooth_data_flag = 1;
#if 0
								unsigned char x;
								char MAC_addr[7];
								char MAC_addr_str[15];
								char tmpstr[4];
// Enqueued msg:
// MAC addr 		6 bytes
// rssi				1 byte
// data_len			1 byte
// data 			value of above
							
								printf("BT MAC_addr: ");
								MAC_addr_str[0] = 0x00;
								for (x=0;x<6;x++)
									{
									MAC_addr[x] = bt_out_data->q_data[5 - x];
									printf("%02X:",MAC_addr[x]);
									sprintf(tmpstr,"%02X",MAC_addr[x]);
									strcat(MAC_addr_str,tmpstr);
									}
								printf("\n");
								
								MAC_addr[6] = 0;
#endif

// add Bluetooth MAC address to topic string...
#if 0
								strcat(mqtt_topic_str,"/");						
								strcat(mqtt_topic_str,MAC_addr_str);				// gateway MAC addr
								printf("Sending to topic: %s\n",mqtt_topic_str);
#endif
															
// assemble payload
								if (payload_left != MQTT_PAYLOAD_SIZE)	// dont insert "|" char first time around...
									{
									strcat(mqtt_payload_str,"|");
									mqtt_payload_length++;
									}
									
								for (unsigned char p=0;p<bt_out_data->q_data_len;p++)
									{
									sprintf(tmp_str,"%02X",bt_out_data->q_data[p]);
									strcat(mqtt_payload_str,tmp_str);
//								if (p<12)		// print the first 12 chars for debug...
//									printf("%02X_%s|",bt_out_data->q_data[p],tmp_str);
									}
//								printf("\r\n");
							
								mqtt_payload_length = mqtt_payload_length + (bt_out_data->q_data_len * 2);	// 2 ASCII hex chars per byte...
								
								payload_left = MQTT_PAYLOAD_SIZE - mqtt_payload_length;
								
								msgs_added++;

//							printf("BT MAC:         %s\r\n",tmp_str);
								

// deallocate memory now msg has been received...						
								free(bt_out_data->q_data);
							
								bt_msg_ready_count--;
							
								if (bt_msg_ready_count ==  0)		// if last msg...
									bluetooth_payload_flag = 0;		// end loop and transmit
								
								}
							
						
//						mqtt_state = MQTT_SET_TOPIC;
//						mqtt_login_state = BT_SENSOR_DATA_WAIT;	
							}
						}
			
					
					printf("\nBT Msgs added: %d\n", msgs_added);
					printf("BT payload:     %s\r\n",mqtt_payload_str);
					printf("BT payload len: %d [%d]\r\n",mqtt_payload_length, payload_left);

					printf("BT msgs still on queue: %d\n",bt_msg_ready_count);
					}	// end of "if (bt_msg_rdy_count)...
					
//##				if (mqtt_payload_length)
//##					mqtt_state = MQTT_SET_TOPIC;

#endif	// end of "#ifdef USE_BLUETOOTH...

//##				mqtt_login_state = BT_SENSOR_DATA_WAIT;

				mqtt_send(mqtt_transport_mode,MQTT_SEND_WAIT);

				}
			break;

		case BT_MAC_ADDR_DATA:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
				bluetooth_MAC_addr_timer = 0;

//				printf("### *** ### BT_MAC_ADDR DATA\n");
				
#ifdef USE_BLUETOOTH
#ifdef BLUETOOTH_SCAN
				if (bluetooth_MAC_entries)
					{
//**************************************************
//*                                                *
//* Setting payloads for BLUETOOTH MAC ADDR DATA   *
//*                                                *
//**************************************************

					print_banner_msg("Setting payloads for BLUETOOTH MAC ADDRESS DATA");

#ifdef AQL_SERVER
// extract BT MAC from payload data (first 6 bytes \ 12 chars):
					memcpy(tmp_str,mqtt_payload_str,12);
					tmp_str[12] = 0x00;
			
					strcpy(mqtt_topic_str,"/gateway_dom/frame/");	//
					strcat(mqtt_topic_str,mac_bin_str);				// gateway MAC addr
					strcat(mqtt_topic_str,"/btscan");						//
//						strcat(mqtt_topic_str,tmp_str);					// BT device MAC addr
//						strcat(mqtt_topic_str,"/386");					//
#endif

//					printf("Sending to topic: %s\n",mqtt_topic_str);

// assemble payload
					mqtt_payload_str[0] = 0x00;
					mqtt_payload_length = 0;
					
// as max bluetooth data = 512 * 12 bytes, this can never exceed mqtt buffer size of 8192 - 
// else we would have to check that the buffer wasnt exceeded with every MAC address we add...
					for (unsigned int p=0;p<bluetooth_MAC_entries;p++)
						{
//tmp_str[0] = 0;
//						sprintf(tmp_str,"%012llX;%02X",*bluetooth_MAC_list[p]->bluetooth_MAC_addr,bluetooth_MAC_list[p]->bluetooth_rssi);
						sprintf(tmp_str,"%02X%02X%02X%02X%02X%02X:%02X:%s",
							bluetooth_MAC_list[p]->bluetooth_MAC_addr[0],
							bluetooth_MAC_list[p]->bluetooth_MAC_addr[1],
							bluetooth_MAC_list[p]->bluetooth_MAC_addr[2],
							bluetooth_MAC_list[p]->bluetooth_MAC_addr[3],
							bluetooth_MAC_list[p]->bluetooth_MAC_addr[4],
							bluetooth_MAC_list[p]->bluetooth_MAC_addr[5],
							(unsigned char)bluetooth_MAC_list[p]->bluetooth_rssi,bluetooth_MAC_list[p]->bluetooth_devname);
							
						strcat(mqtt_payload_str,tmp_str);
						mqtt_payload_length = mqtt_payload_length + strlen(tmp_str);
						
						if (p<(bluetooth_MAC_entries - 1))		// ie, not at end of list
							{
							strcat(mqtt_payload_str,"|");
							mqtt_payload_length++;
							}
//						if (p<12)		// print the first 12 chars for debug...
//							printf("%012llX_%s|",bluetooth_MAC_list[p],tmp_str);
						}
					printf("\r\n");

// 2 ASCII hex chars per byte... 12 chars per MAC addr, 1 '|'char, and last entry doesnt have '|'								
//					mqtt_payload_length = (bluetooth_MAC_entries * 13) - 1;	

// 2 ASCII hex chars per byte... 12 chars per MAC addr, 1 char ';', 2 chars per rssi, 1 '|'char, and last entry doesnt have '|'								
//					mqtt_payload_length = (bluetooth_MAC_entries * 16) - 1;	

					printf("payload data = %s\n",mqtt_payload_str);
					printf("payload len  = %d\n",mqtt_payload_length);
					
					
					bluetooth_MAC_entries = 0;

//##					mqtt_state = MQTT_SET_TOPIC;
					}
#endif	// end of "#ifdef BLUETOOTH_SCAN
#endif	// end of "#ifdef USE_BLUETOOTH...

//##				mqtt_login_state = BT_SENSOR_DATA_WAIT;

				mqtt_send(mqtt_transport_mode,MQTT_SEND_WAIT);
				}
			break;			

		case BT_DEVNAME_DATA:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
				bluetooth_devname_timer = 0;

//				printf("### *** ### BT_DEV NAME DATA\n");
				
#ifdef USE_BLUETOOTH
#ifdef BLUETOOTH_SCAN
// NOTE: whitelist match causes bluetooth_devname_entries to increment;
// if whitelisting not enabled, devname messages will not be sent to server!
				if (bluetooth_devname_entries)
					{
//**************************************************
//*                                                *
//* Setting payloads for BLUETOOTH DEV NAME DATA   *
//*                                                *
//**************************************************

					print_banner_msg("Setting payloads for BLUETOOTH DEV NAME DATA");

#ifdef AQL_SERVER
// extract BT MAC from payload data (first 6 bytes \ 12 chars):
					memcpy(tmp_str,mqtt_payload_str,12);
					tmp_str[12] = 0x00;
			
					strcpy(mqtt_topic_str,"/gateway_dom/frame/");	//
					strcat(mqtt_topic_str,mac_bin_str);				// gateway MAC addr
					strcat(mqtt_topic_str,"/btbeaconscan");						//
//						strcat(mqtt_topic_str,tmp_str);					// BT device MAC addr
//						strcat(mqtt_topic_str,"/386");					//
#endif

//					printf("Sending to topic: %s\n",mqtt_topic_str);

// assemble payload
					mqtt_payload_str[0] = 0x00;
					mqtt_payload_length = 0;

					printf("BTB: number of entries: %d\n",bluetooth_devname_entries);
					
// as max bluetooth data = 512 * 12 bytes, this can never exceed mqtt buffer size of 8192 - 
// else we would have to check that the buffer wasnt exceeded with every MAC address we add...
					for (unsigned int p=0;p<bluetooth_devname_entries;p++)
						{
// insert dev name
						sprintf(tmp_str,"%s:",bluetooth_devname_list[p].bluetooth_devname);
						strcat(mqtt_payload_str,tmp_str);
	
// insert MAC address and RSSI
						sprintf(tmp_str,"%02X%02X%02X%02X%02X%02X:%02X",
							bluetooth_devname_list[p].bluetooth_MAC_addr[0],
							bluetooth_devname_list[p].bluetooth_MAC_addr[1],
							bluetooth_devname_list[p].bluetooth_MAC_addr[2],
							bluetooth_devname_list[p].bluetooth_MAC_addr[3],
							bluetooth_devname_list[p].bluetooth_MAC_addr[4],
							bluetooth_devname_list[p].bluetooth_MAC_addr[5],
							(unsigned char)bluetooth_devname_list[p].bluetooth_rssi);

						printf("%03d) BTB DNAME: %s\t",p,bluetooth_devname_list[p].bluetooth_devname);
						printf("BTB Data: %s\n",tmp_str);
						
						strcat(mqtt_payload_str,tmp_str);
						if (p<(bluetooth_devname_entries - 1))		// ie, not at end of list
							{
							strcat(mqtt_payload_str,"|");
							mqtt_payload_length++;
							}
//						if (p<12)		// print the first 12 chars for debug...
//							printf("%012llX_%s|",bluetooth_MAC_list[p],tmp_str);

// string length + 6 byte MAC plus 1 byte RSSI + 2 ":" separators...						
						mqtt_payload_length = mqtt_payload_length + strlen(bluetooth_devname_list[p].bluetooth_devname) + 14 + 2;
						
						}
					printf("\r\n");

// 2 ASCII hex chars per byte... 12 chars per MAC addr, 1 '|'char, and last entry doesnt have '|'								
//					mqtt_payload_length = (bluetooth_MAC_entries * 13) - 1;	

// 2 ASCII hex chars per byte... 12 chars per MAC addr, 1 char ';', 2 chars per rssi, 1 '|'char, and last entry doesnt have '|'								
//					mqtt_payload_length = (bluetooth_MAC_entries * 16) - 1;	

//					printf("payload data = %s\n",mqtt_payload_str);
//					printf("payload len  = %d\n",mqtt_payload_length);
					
					
					bluetooth_devname_entries = 0;

//##					mqtt_state = MQTT_SET_TOPIC;
					}
#endif	// end of "#ifdef BLUETOOTH_SCAN
#endif	// end of "#ifdef USE_BLUETOOTH...

//##				mqtt_login_state = BT_SENSOR_DATA_WAIT;

				mqtt_send(mqtt_transport_mode,MQTT_SEND_WAIT);
				}
			break;			

		case BT_SENSOR_DATA_WAIT:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
				mqtt_login_state = SENSOR_DATA_IDLE;
				}
			break;

		case GET_BT_WHITELIST_TABLE:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
#ifdef USE_BLUETOOTH
//***************************************************
//*                                                 *
//* Setting payloads for BLUETOOTH WHITELIST TABLE  *
//*                                                 *
//***************************************************
				print_banner_msg("Setting payloads for BLUETOOTH WHITELIST TABLE");

#ifdef AQL_SERVER		
				strcpy(mqtt_topic_str,"/gateway_dom/frame/");	//
				strcat(mqtt_topic_str,mac_bin_str);				// gateway MAC addr
#endif

//				printf("Sending to topic: %s\n",mqtt_topic_str);

				sprintf(mqtt_payload_str,"bwlist|%d|",enable_bluetooth_whitelist_flag);			

				for (unsigned char i=0;i<num_bt_wl_entries;i++)	
					{
					for (unsigned char j=0;j<6;j++)
						{
						sprintf(tmp_str,"%02X",bluetooth_cmd_whitelist[i][j]);			
						strcat(mqtt_payload_str,tmp_str);							
						}
//					sprintf(tmp_str,"|%s",blutooth_cmd_whitelist[i]);			
//					strcat(mqtt_payload_str,tmp_str);							
					}

				mqtt_payload_length = strlen(mqtt_payload_str);
//##				mqtt_state = MQTT_SET_TOPIC;
#endif
				tx_bluetooth_whitelist_flag = 0;
//##				mqtt_login_state = GET_BW_LIST_TABLE_WAIT;

				mqtt_send(mqtt_transport_mode,MQTT_SEND_WAIT);
				}
			break;


		case GET_BT_BLACKLIST_TABLE:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
#ifdef USE_BLUETOOTH
//***************************************************
//*                                                 *
//* Setting payloads for BLUETOOTH BLACKLIST TABLE  *
//*                                                 *
//***************************************************
				print_banner_msg("Setting payloads for BLUETOOTH BLACKLIST TABLE");

#ifdef AQL_SERVER		
				strcpy(mqtt_topic_str,"/gateway_dom/frame/");	//
				strcat(mqtt_topic_str,mac_bin_str);				// gateway MAC addr
#endif

//				printf("Sending to topic: %s\n",mqtt_topic_str);

				sprintf(mqtt_payload_str,"bblist|%d|",enable_bluetooth_blacklist_flag);			

				for (unsigned char i=0;i<num_bt_bl_entries;i++)	
					{
					for (unsigned char j=0;j<6;j++)
						{
						sprintf(tmp_str,"%02X",bluetooth_cmd_blacklist[i][j]);			
						strcat(mqtt_payload_str,tmp_str);							
						}
//					sprintf(tmp_str,"|%s",blutooth_cmd_whitelist[i]);			
//					strcat(mqtt_payload_str,tmp_str);							
					}

				mqtt_payload_length = strlen(mqtt_payload_str);
//##				mqtt_state = MQTT_SET_TOPIC;
#endif

				tx_bluetooth_blacklist_flag = 0;
//##				mqtt_login_state = GET_BW_LIST_TABLE_WAIT;

				mqtt_send(mqtt_transport_mode,MQTT_SEND_WAIT);
				}
			break;


		case LORA_SENSOR_DATA:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{

#ifdef USE_LORA
				if (lora_msg_ready_count)
					{
					unsigned char lora_payload_flag;
					unsigned int payload_left = MQTT_PAYLOAD_SIZE;
					unsigned int msgs_added = 0;

//*******************************************
//*                                         *
//* Setting payloads for LORA SENSOR DATA   *
//*                                         *
//*******************************************

					print_banner_msg("Setting payloads for LORA SENSOR DATA");

#ifdef AQL_SERVER
					strcpy(mqtt_topic_str,"/gateway_dom/frame/");	//node/1");
					strcat(mqtt_topic_str,mac_bin_str);	//node/1");
					strcat(mqtt_topic_str,"/lora");
// LoRa dev addr added later....					
#endif

//					printf("Sending to topic: %s\n",mqtt_topic_str);

					mqtt_payload_str[0] = 0x00;
					mqtt_payload_length = 0;

					printf("Payload available: %d\n",payload_left);
					lora_payload_flag = 1;

					while (lora_payload_flag)
						{
						xQueuePeek(lora_data_queue, lora_rx_data_rd, portMAX_DELAY);
						if (((2*lora_rx_data_rd->q_data_len) + 1) < payload_left)	// if there is space in the payload buffer...
																				// '+1' is for pipe separator
							{

// get payload from queue			
							if (xQueueReceive(lora_data_queue, lora_rx_data_rd, portMAX_DELAY) != pdPASS) 
								{
								ESP_LOGE(TAG, "LORA rx_queue receive error");
								} 
							else
								{
//							unsigned char mplen = lora_rx_data_rd->q_data_len;

#if 0
								unsigned char send_lora_data_flag = 1;
								unsigned char x;
								char dev_addr[5];
								char dev_addr_str[15];
								char tmpstr[4];
								
								printf("LoRa dev_addr: ");
//						memcpy(dev_addr,&lora_rx_data_rd->q_data[4],4);
								dev_addr_str[0] = 0x00;
								for (x=0;x<4;x++)
									{
									dev_addr[x] = lora_rx_data_rd->q_data[3 + 1 - x];
									printf("%02X:",dev_addr[x]);
									sprintf(tmpstr,"%02X",dev_addr[x]);
									strcat(dev_addr_str,tmpstr);
									}
								printf("\n");
								
								dev_addr[4] = 0;
#endif

// add dev_addr to topic...
#if 0
								strcat(mqtt_topic_str,"/");
								strcat(mqtt_topic_str,dev_addr_str);
								strcat(mqtt_topic_str,"/lora");
								printf("Sending to topic: %s\n",mqtt_topic_str);
#endif

								
// assemble payload
								if (payload_left != MQTT_PAYLOAD_SIZE)
									{
									strcat(mqtt_payload_str,"|");
									mqtt_payload_length++;
									}
								
//						printf("LORA Len: %d %d \n",lora_rx_data_rd->q_data_len,mplen);

// LoRa data may be 8 bit binary; convert to 2 character ASCII format for MQTT transfer..						
								for (unsigned char p=0;p<lora_rx_data_rd->q_data_len;p++)
									{
									sprintf(tmp_str,"%02X",lora_rx_data_rd->q_data[p]);
									strcat(mqtt_payload_str,tmp_str);
//							if (p<12)		// print the first 12 chars for debug...
//								printf("%02X_%s|",lora_rx_data_rd->q_data[p],tmp_str);
									}
//						printf("\r\n");
						
								mqtt_payload_length = mqtt_payload_length + (lora_rx_data_rd->q_data_len * 2);	// 2 ASCII hex chars per byte...
								
								payload_left = MQTT_PAYLOAD_SIZE - mqtt_payload_length;

								msgs_added++;
//							printf("LORA payload len = %d\n",mqtt_payload_length);

// deallocate memory now msg has been received...						
//						printf("Freeing LORA mem\n");
								free(lora_rx_data_rd->q_data);
//						printf("LORA mem freed OK\n");
							
								lora_msg_ready_count--;
							
								if (lora_msg_ready_count == 0)		// if last msg...
									lora_payload_flag = 0;			// end loop and transmit

//						printf("LORA MAC:     %s\r\n",tmp_str);
//							printf("LORA payload: %s\r\n",mqtt_payload_str);
								}
							}
						}

					printf("\nLORA Msgs added: %d\n", msgs_added);
//					printf("LORA payload:     %s\r\n",mqtt_payload_str);
//					printf("LORA payload len: %d [%d]\r\n",mqtt_payload_length, payload_left);
			
					printf("LORA msgs still on queue: %d\n",lora_msg_ready_count);
					}	// end of "if (lora_msg_ready_count)...
					
//##					if (mqtt_payload_length)
//##						mqtt_state = MQTT_SET_TOPIC;

#endif	// USE_LORA
//##				mqtt_login_state = LORA_SENSOR_DATA_WAIT;

				mqtt_send(mqtt_transport_mode,MQTT_SEND_WAIT);
				}
			break;			

		case LORA_DEVADDR_DATA:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
				lora_devaddr_timer = 0;

#ifdef USE_LORA
#ifdef LORAWAN_SCAN
				if (lorawan_MAC_entries)
					{

//********************************************
//*                                          *
//* Setting payloads for LORA DEVADDR DATA   *
//*                                          *
//********************************************

					print_banner_msg("Setting payloads for LORA DEVADDR DATA");

#ifdef AQL_SERVER
					strcpy(mqtt_topic_str,"/gateway_dom/frame/");	//node/1");
					strcat(mqtt_topic_str,mac_bin_str);	//node/1");
					strcat(mqtt_topic_str,"/lorascan");
#endif

//					printf("Sending to topic: %s\n",mqtt_topic_str);


// assemble payload
					mqtt_payload_str[0] = 0x00;
// as max lora data = 512 * 9 bytes, this can never exceed mqtt buffer size of 8192 - 
// else we would have to check that the buffer wasnt exceeded with every MAC address we add...
					for (unsigned int p=0;p<lorawan_MAC_entries;p++)
						{
						sprintf(tmp_str,"%08X",lorawan_MAC_list[p]);
						strcat(mqtt_payload_str,tmp_str);
						if (p<(lorawan_MAC_entries - 1))		// ie, not at end of list
							strcat(mqtt_payload_str,"|");
//						if (p<12)		// print the first 12 chars for debug...
//							printf("%08X_%s|",1,tmp_str);
						}
					printf("\r\n");

// 2 ASCII hex chars per byte... 8 chars per MAC addr, 1 '|'char, and last entry doesnt have '|'								
					mqtt_payload_length = (lorawan_MAC_entries * 9) - 1;	

//					printf("payload data = %s\n",mqtt_payload_str);
//					printf("payload len  = %d\n",mqtt_payload_length);
						
					lorawan_MAC_entries = 0;

//##					mqtt_state = MQTT_SET_TOPIC;

					}
#endif	// LORAWAN_SCAN
#endif	// USE_LORA
//##				mqtt_login_state = LORA_SENSOR_DATA_WAIT;

				mqtt_send(mqtt_transport_mode,MQTT_SEND_WAIT);
				}
			break;			
			
		case LORA_SENSOR_DATA_WAIT:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
				mqtt_login_state = SENSOR_DATA_IDLE;
				}
			break;

		case GET_LORA_WHITELIST_TABLE:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
#ifdef USE_LORA
//***************************************************
//*                                                 *
//* Setting payloads for LORAWAN WHITELIST TABLE  *
//*                                                 *
//***************************************************
				print_banner_msg("Setting payloads for LORAWAN WHITELIST TABLE");

#ifdef AQL_SERVER		
				strcpy(mqtt_topic_str,"/gateway_dom/frame/");	//
				strcat(mqtt_topic_str,mac_bin_str);				// gateway MAC addr
#endif

//				printf("Sending to topic: %s\n",mqtt_topic_str);

				sprintf(mqtt_payload_str,"bwlist|%d|",enable_lorawan_whitelist_flag);			

				for (unsigned char i=0;i<num_lora_wl_entries;i++)	
					{
					for (unsigned char j=0;j<4;j++)
						{
						sprintf(tmp_str,"%02X",lorawan_cmd_whitelist[i][j]);			
						strcat(mqtt_payload_str,tmp_str);							
						}
//					sprintf(tmp_str,"|%s",blutooth_cmd_whitelist[i]);			
//					strcat(mqtt_payload_str,tmp_str);							
					}

				mqtt_payload_length = strlen(mqtt_payload_str);
//##				mqtt_state = MQTT_SET_TOPIC;
#endif

				tx_lorawan_whitelist_flag = 0;
//##				mqtt_login_state = GET_BW_LIST_TABLE_WAIT;

				mqtt_send(mqtt_transport_mode,MQTT_SEND_WAIT);
				}
			break;

		case GET_LORA_BLACKLIST_TABLE:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
#ifdef USE_LORA
//***************************************************
//*                                                 *
//* Setting payloads for LORAWAN BLACKLIST TABLE    *
//*                                                 *
//***************************************************
				print_banner_msg("Setting payloads for LORAWAN BLACKLIST TABLE");

#ifdef AQL_SERVER		
				strcpy(mqtt_topic_str,"/gateway_dom/frame/");	//
				strcat(mqtt_topic_str,mac_bin_str);				// gateway MAC addr
#endif

//				printf("Sending to topic: %s\n",mqtt_topic_str);

				sprintf(mqtt_payload_str,"bwlist|%d|",enable_lorawan_blacklist_flag);			

				for (unsigned char i=0;i<num_lora_bl_entries;i++)	
					{
					for (unsigned char j=0;j<4;j++)
						{
						sprintf(tmp_str,"%02X",lorawan_cmd_blacklist[i][j]);			
						strcat(mqtt_payload_str,tmp_str);							
						}
//					sprintf(tmp_str,"|%s",blutooth_cmd_whitelist[i]);			
//					strcat(mqtt_payload_str,tmp_str);							
					}

				mqtt_payload_length = strlen(mqtt_payload_str);
//##				mqtt_state = MQTT_SET_TOPIC;
#endif

				tx_lorawan_blacklist_flag = 0;
//##				mqtt_login_state = GET_BW_LIST_TABLE_WAIT;

				mqtt_send(mqtt_transport_mode,MQTT_SEND_WAIT);
				}
			break;

		case WIFI_SCAN_DATA:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
				wifi_scan_timer = 0;

#ifdef WIFI_SCAN
#if 1
///wifi_list_current[wfi].bssid // 6 char array
//% 3ddB",wifi_list_current[wfi].rssi // 1 byte rssi
//wifi_current_num	// number of entries
				if (wifi_current_num)					
					{

//********************************************
//*                                          *
//* Setting payloads for WIFI SCAN DATA      *
//*                                          *
//********************************************

					print_banner_msg("Setting payloads for WIFI SCAN DATA");

#ifdef AQL_SERVER
					strcpy(mqtt_topic_str,"/gateway_dom/frame/");	//node/1");
					strcat(mqtt_topic_str,mac_bin_str);	//node/1");
					strcat(mqtt_topic_str,"/wifiscan");
#endif

//					printf("Sending to topic: %s\n",mqtt_topic_str);


// assemble payload
					mqtt_payload_str[0] = 0x00;
					mqtt_payload_length = 0;
					
// as max lora data = 512 * 9 bytes, this can never exceed mqtt buffer size of 8192 - 
// else we would have to check that the buffer wasnt exceeded with every MAC address we add...
					for (unsigned int p=0;p<wifi_current_num;p++)
						{
						sprintf(tmp_str,"%02X%02X%02X%02X%02X%02X:%02X:%s",
						wifi_list_current[p].bssid[0],
						wifi_list_current[p].bssid[1],
						wifi_list_current[p].bssid[2],
						wifi_list_current[p].bssid[3],
						wifi_list_current[p].bssid[4],
						wifi_list_current[p].bssid[5],
						(unsigned char)wifi_list_current[p].rssi,wifi_list_current[p].ssid);
						
						strcat(mqtt_payload_str,tmp_str);

						mqtt_payload_length = mqtt_payload_length + strlen(tmp_str);

						if (p<(wifi_current_num - 1))		// ie, not at end of list
							{
							strcat(mqtt_payload_str,"|");
							mqtt_payload_length++;
							}
							
							
//						if (p<12)		// print the first 12 chars for debug...
//							printf("%08X_%s|",1,tmp_str);
						}
					printf("\r\n");

// 2 ASCII hex chars per byte... 12 chars per MAC addr, 1 ':'char, 2 char rssi, one '|' char and last entry doesnt have '|'								
//					mqtt_payload_length = (wifi_current_num * 16) - 1;	

//					printf("payload data = %s\n",mqtt_payload_str);
//					printf("payload len  = %d\n",mqtt_payload_length);
						
					wifi_current_num = 0;	// necessary?
					}

//##				mqtt_state = MQTT_SET_TOPIC;

#endif
#endif	// WIFI_SCAN

//##				mqtt_login_state = WIFI_SCAN_DATA_WAIT;

				mqtt_send(mqtt_transport_mode,MQTT_SEND_WAIT);
				}
			wifi_scan_valid = 0;
			break;

		case WIFI_SCAN_DATA_WAIT:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
				mqtt_login_state = SENSOR_DATA_IDLE;
				}
			break;

		case I2C_SENSOR_DATA:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
				i2c_sensor_timer = 0;
#ifdef USE_I2C
//*************************************8****
//*                                        *
//* Setting payloads for I2C SENSOR DATA   *
//*                                        *
//****************************************8*
				print_banner_msg("Setting payloads for I2C SENSOR DATA");

#ifdef AQL_SERVER
				strcpy(mqtt_topic_str,"/gateway_dom/frame/");	//node/1");
				strcat(mqtt_topic_str,mac_bin_str);	//node/1");
				strcat(mqtt_topic_str,"/sb/i2c");
#endif
#ifdef SSS_SERVER
				strcpy(mqtt_topic_str,"/dom/100234/frame/node/1");
#endif
#ifdef MOSQUITTO_SERVER
				strcpy(mqtt_topic_str,"/dom/100234/frame/node/1");
#endif

//				printf("Sending to topic: %s\n",mqtt_topic_str);


#ifdef AQL_SERVER
// tank sensor
// I2C temp, humidity, light, audio, batt, CO2, VOC
				sprintf(mqtt_payload_str,"8|%04X|9|%04X|10|%04X|11|%04X|12|%04X|13|%04X|14|%04X",i2c_temp,i2c_humid,i2c_light,i2c_audio,i2c_batt,i2c_co2,i2c_voc);
				mqtt_payload_length = strlen(mqtt_payload_str);
#endif
#ifdef SSS_SERVER
// I2C temp, humidity, light, audio, batt, CO2, VOC
				sprintf(mqtt_payload_str,"8|%04X|9|%04X|10|%04X|11|%04X|12|%04X|13|%04X|14|%04X",i2c_temp,i2c_humid,i2c_light,i2c_audio,i2c_batt,i2c_co2,i2c_voc);
				mqtt_payload_length = strlen(mqtt_payload_str);
#endif
#ifdef MOSQUITTO_SERVER
// I2C temp, humidity, light, audio, batt, CO2, VOC
				sprintf(mqtt_payload_str,"8|%04X|9|%04X|10|%04X|11|%04X|12|%04X|13|%04X|14|%04X",i2c_temp,i2c_humid,i2c_light,i2c_audio,i2c_batt,i2c_co2,i2c_voc);
				mqtt_payload_length = strlen(mqtt_payload_str);
#endif

//##				mqtt_state = MQTT_SET_TOPIC;


#endif	// USE_I2C
//##				mqtt_login_state = I2C_SENSOR_DATA_WAIT;

				mqtt_send(mqtt_transport_mode,MQTT_SEND_WAIT);
				}
			break;

		case I2C_SENSOR_DATA_WAIT:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
				mqtt_login_state = SENSOR_DATA_IDLE;
				}
			break;

		case TANK_SENSOR_DATA:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
				tank_sensor_timer = 0;
#ifdef AQL_SERVER
				strcpy(mqtt_topic_str,"/gateway_dom/frame/");	//node/1");
				strcat(mqtt_topic_str,mac_bin_str);	//node/1");
				strcat(mqtt_topic_str,"/sb/ts");	//node/1");
#endif
#ifdef SSS_SERVER
				strcpy(mqtt_topic_str,"/dom/100234/frame/node/1");
#endif
#ifdef MOSQUITTO_SERVER
				strcpy(mqtt_topic_str,"/dom/100234/frame/node/1");
#endif

//				printf("Sending to topic: %s\n",mqtt_topic_str);

#ifdef AQL_SERVER
// tank sensor
				sprintf(mqtt_payload_str,"15|%02X|16|%02X",tank_sensor_0,tank_sensor_1);			
				mqtt_payload_length = strlen(mqtt_payload_str);
#endif
#ifdef SSS_SERVER
				sprintf(mqtt_payload_str,"15|%02X|16|%02X",tank_sensor_0,tank_sensor_1);			
				mqtt_payload_length = strlen(mqtt_payload_str);
#endif
#ifdef MOSQUITTO_SERVER
				sprintf(mqtt_payload_str,"15|%02X|16|%02X",tank_sensor_0,tank_sensor_1);			
				mqtt_payload_length = strlen(mqtt_payload_str);
#endif
//##				mqtt_state = MQTT_SET_TOPIC;
//##				mqtt_login_state = TANK_SENSOR_DATA_WAIT;

				mqtt_send(mqtt_transport_mode,MQTT_SEND_WAIT);
				}
			break;

		case TANK_SENSOR_DATA_WAIT:
//			if (mqtt_state == MQTT_CONNECT_IDLE)

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
				mqtt_login_state = SENSOR_DATA_IDLE;
				}
			break;




// the following cses ar efor SIMCOM test \ diag and we cant use the mqtt_send(mqtt_transport_mode,MQTT_SEND_WAIT) function to start mqtt...
		case NET_4G_REPORT:
			if (mqtt_state == MQTT_CONNECT_IDLE)
				{
				net_test_flag = 0;	
//				printf("Getting signal strength info...\n");
				mqtt_state = MQTT_IDLE_CHECK_4G_NET;
				mqtt_login_state = NET_4G_REPORT_WAIT;
				}		
			break;

		case NET_4G_REPORT_WAIT:
			if (mqtt_state == MQTT_CONNECT_IDLE)
				{
				mqtt_login_state = SENSOR_DATA_IDLE;
				}
			break;
		
		case IDLE_CHECK_SIGNAL_QUALITY:
			if (mqtt_state == MQTT_CONNECT_IDLE)
				{
//				printf("Getting signal strength info...\n");
				mqtt_state = MQTT_IDLE_CHECK_SIG_QUALITY;
				mqtt_login_state = IDLE_CHECK_SIGNAL_QUAL_WAIT;
				}		
			break;
		
		case IDLE_CHECK_SIGNAL_QUAL_WAIT:
			if (mqtt_state == MQTT_CONNECT_IDLE)
				{
				mqtt_login_state = SENSOR_DATA_IDLE;
				}
			break;

		case GET_GPS_INFO:
//			if (mqtt_state == MQTT_CONNECT_IDLE)
			if ((simcom_state == SIMCOM_READY_IDLE) && (mqtt_state == MQTT_CONNECT_IDLE))
				{
				gps_sensor_timer = 0;
//				four_g_state = M4G_GPS_CHECK;
				simcom_state = SIMCOM_GPS_CHECK;
//			four_g_state = M4G_GPS_INFO;
				mqtt_login_state = GET_GPS_INFO_WAIT;
				}
			break;
			
		case GET_GPS_INFO_WAIT:
			if (simcom_state == SIMCOM_READY_IDLE)
				{
//				four_g_state = M4G_CONNECT_IDLE;
				mqtt_login_state = SENSOR_DATA_IDLE;
				}
			break;

		case MQTT_SEND_WAIT:
// this is where all frame send cases return to...
#ifdef XXX_M4G_MQTT
			if ((mqtt_transport_mode == MQTT_MODE_4G) && (simcom_state == SIMCOM_CONNECT_IDLE))
				{
				mqtt_login_state = SENSOR_DATA_IDLE;
				}
#endif

#ifdef XXX_USE_WIFI_MQTT
			if (mqtt_transport_mode == MQTT_MODE_WIFI)// && (simcom_state == SIMCOM_READY_IDLE))
				{
				mqtt_login_state = SENSOR_DATA_IDLE;
				}
#endif

			if (((mqtt_transport_mode == MQTT_MODE_4G) && (mqtt_state == MQTT_CONNECT_IDLE)) ||	// && (mqtt_state == MQTT_NO_COMM_IDLE))
			    ((mqtt_transport_mode == MQTT_MODE_WIFI) && (wifi_mqtt_connected)) )
				{
				mqtt_login_state = SENSOR_DATA_IDLE;
				}

			break;
		
		case SRVR_DISCONNECT:
// get here due to subscribe msg "disconnect"...		
			if (mqtt_state == MQTT_CONNECT_IDLE)
				{
				printf("Starting disconnect...\n");
				mqtt_state = MQTT_SERVER_DISCONNECT;	// M4G_MQTT_UNSUBSCRIBE_TOPIC;  dont bother with unsubscribe!
				mqtt_login_state = SRVR_DISCONNECT_WAIT;
				}
			break;

		case SRVR_DISCONNECT_WAIT:
			if (mqtt_state == MQTT_COMM_IDLE)
				{
				printf("Disconnecting...\n");
				mqtt_state = MQTT_NO_COMM_IDLE;
				mqtt_login_state = NOT_SRVR_INITIALISED;
				}
			break;
		}	// end of "switch (mqtt_login_state)...

#ifdef USE_WIFI_UDP
#ifdef WIFI_UDP_TEST		
// UDP call
// example: https://github.com/espressif/esp-idf/blob/master/examples/protocols/sockets/udp_client/main/udp_client.c
		if ((udp_test_flag)  && (wifi_udp_connected) && (wifi_udp_DNSfound))
			{
			unsigned int i;
			unsigned char c;
#if 0
			unsigned char lora_msg[24] ={
				0x40, 0xF1, 0x1A, 0x01, 0x26, 0x00, 0xB5, 0x00, 0x02, 0x88,
				0x8D, 0x95, 0x67, 0x38, 0x25, 0xC3, 0x0C, 0x11, 0xE1, 0x6B,
				0x06, 0x9B, 0x91, 0xE8};


			udp_payload_length = 24;
#endif
#if 1
// https://github.com/Lora-net/packet_forwarder/blob/master/PROTOCOL.TXT
/*
``` json
{"rxpk":[
	{
		"time":"2013-03-31T16:21:17.528002Z",
		"tmst":3512348611,
		"chan":2,
		"rfch":0,
		"freq":866.349812,
		"stat":1,
		"modu":"LORA",
		"datr":"SF7BW125",
		"codr":"4/6",
		"rssi":-35,
		"lsnr":5.1,
		"size":32,
		"data":"-DS4CGaDCdG+48eJNM3Vai-zDpsR71Pn9CPA9uCON84"
	},{
		"time":"2013-03-31T16:21:17.530974Z",
		"tmst":3512348514,
		"chan":9,
		"rfch":1,
		"freq":869.1,
		"stat":1,
		"modu":"FSK",
		"datr":50000,
		"rssi":-75,
		"size":16,
		"data":"VEVTVF9QQUNLRVRfMTIzNA=="
//              01234567890123456789012345678901234567890123456789

	},{
		"time":"2013-03-31T16:21:17.532038Z",
		"tmst":3316387610,
		"chan":0,
		"rfch":0,
		"freq":863.00981,
		"stat":1,
		"modu":"LORA",
		"datr":"SF10BW125",
		"codr":"4/7",
		"rssi":-38,
		"lsnr":5.5,
		"size":32,
		"data":"ysgRl452xNLep9S1NTIg2lomKDxUgn3DJ7DE+b00Ass"
	}
]}
```
//              01234567890123456789012345678901234567890123456789
``` json
{"rxpk":[
	{
		"time":"2013-03-31T16:21:17.528002Z",
		"tmst":3512348611,
		"chan":2,
		"rfch":0,
		"freq":866.349812,
		"stat":1,
		"modu":"LORA",
		"datr":"SF7BW125",
		"codr":"4/6",
		"rssi":-35,
		"lsnr":5.1,
		"size":32,
		"data":"-DS4CGaDCdG+48eJNM3Vai-zDpsR71Pn9CPA9uCON84"
	}
]}
```
*/

			unsigned char lora_hdr[] ={0x02, 0x67,0x78, 0x00};	// protocol ver 2: 2 byts of random token: 0x00 = PUSH_DATA ID

/*
			unsigned char lora_msg[] ={"```json{\"rxpk\":[{\"time\":\"2023-11-02T11:15:17.528002Z\",\"tmst\":3512348611,\"chan\":2,\"rfch\":0,\"freq\":866.349812,\"stat\":1,\
			\"modu\":\"LORA\",\"datr\":\"SF7BW125\",\"codr\":\"4/6\",\"rssi\":-35,\"lsnr\":5.1,\"size\":32,\"data\":\"-DS4CGaDCdG+48eJNM3Vai-zDpsR71Pn9CPA9uCON84\"}]}```"}; 
*/			
#if 0
// watch out - TABs can be included in the string if the second line is tabbed out for formatting purposes...
			unsigned char lora_msg[] ={"{\"rxpk\":[{\"time\":\"2023-11-02T11:15:17.528002Z\",\"tmst\":3512348611,\"chan\":2,\"rfch\":0,\"freq\":866.349812,\"stat\":1,\
\"modu\":\"LORA\",\"datr\":\"SF7BW125\",\"codr\":\"4/6\",\"rssi\":-35,\"lsnr\":5.1,\"size\":32,\"data\":\"-DS4CGaDCdG+48eJNM3Vai-zDpsR71Pn9CPA9uCON84\"}]}"}; 
#endif

#if 0
// watch out - TABs can be included in the string if the second line is tabbed out for formatting purposes...
			unsigned char lora_msg[] ={"{\"rxpk\":[{\"time\":\"2023-11-02T11:15:17.528002Z\",\"tmst\":3512348611,\"chan\":2,\"rfch\":0,\"freq\":866.349812,\"stat\":1,\
\"modu\":\"LORA\",\"datr\":\"SF7BW125\",\"codr\":\"4/6\",\"rssi\":-35,\"lsnr\":5.1,\"size\":32,\"data\":\"ysgRl452xNLep9S1NTIg2lomKDxUgn3DJ7DE+b00Ass\"}]}"}; 
#endif

//              QDSQAieAxzACEbaXbugkLxN50YqNSHawd7CDvBHV8Jer4ZAYINNtnFllElLdH1EQ
//              0123456789012345678901234567890123456789012345678901234567890123456789
#if 1
// watch out - TABs can be included in the string if the second line is tabbed out for formatting purposes...
			unsigned char lora_msg[] ={"{\"rxpk\":[{\"time\":\"2023-11-02T11:15:17.528002Z\",\"tmst\":3512348611,\"chan\":2,\"rfch\":0,\"freq\":866.349812,\"stat\":1,\
\"modu\":\"LORA\",\"datr\":\"SF7BW125\",\"codr\":\"4/6\",\"rssi\":-35,\"lsnr\":5.1,\"size\":70,\"data\":\"QDSQAieAxzACEbaXbugkLxN50YqNSHawd7CDvBHV8Jer4ZAYINNtnFllElLdH1EQ\"}]}"}; 
#endif
//example UTC time string: 2023-11-02T11:15:17.528002Z
// UTC strftime string:    %Y - %m-%dT%X.
// gettimeofday();
			udp_payload_length = strlen((char*)lora_msg);

#endif
			
			printf("UDP frame test!\n");
//			strcat(udp_payload_str,"UDP Test!");
//			udp_payload_length = strlen(udp_payload_str);
			printf("MAC: %s\n",mac_bin_str);
			
// add packet header...
			memcpy(udp_payload_str,lora_hdr,4);
// add gateway ID...
// if MAC address is 5B:A0:CB:80:04:2B then the EUI is 5B A0 CB FF FE 80 04 2B (ie, insert FF FE in middle of addr)
			
			esp_efuse_mac_get_default(mac_addr);		// get hex version of gateway address
			memcpy(&udp_payload_str[4],mac_addr,3);		// first 3 bytes of MAC addr...

			udp_payload_str[7] = 0xFF;
			udp_payload_str[8] = 0xFE;

			memcpy(&udp_payload_str[9],&mac_addr[3],3);		// last 3 bytes of MAC addr...

// add data			
			memcpy(&udp_payload_str[12],lora_msg,udp_payload_length);

			udp_payload_length = udp_payload_length +12;
			udp_payload_str[udp_payload_length] = 0x00;
/*
// add packet header...
			for (i=0;i<4;i++)
				{
				udp_payload_str[i] = lora_hdr[i];
				printf("%02X ",udp_payload_str[i]);
				}
// add MAC address of gateway...
			for (i=4;i<10;i++)
				{
				udp_payload_str[i] = mac_bin_str[i-4];
				printf("%02X ",udp_payload_str[i]);
				}
// add data			
			for (i=10;i<udp_payload_length+10;i++)
				{
				udp_payload_str[i] = lora_msg[i-10];
				printf("%02X ",udp_payload_str[i]);
				}
			printf("\n");	

			udp_payload_str[i] = 0x00;
			udp_payload_length = udp_payload_length +10;
*/

			printf("Wifi UDP lora msg data: [%d]\n",udp_payload_length);	//udp_payload_length);	
// show packet data in hex...
/*
			for (i=0;i<udp_payload_length;i++)
				{
				printf("%02X ",udp_payload_str[i]);
				}
			printf("\n");	
*/
// show packet data in ASCII...
			for (i=0;i<udp_payload_length;i++)
				{
				c = udp_payload_str[i];
				if ((c>0x1F) && (c<0x80))		// if printable char
					printf("%c ",c);
				else
					printf(". ");					
				}
			printf("\n");	
			
// send UDP message
			printf("Wifi UDP payload: [%d]\n",udp_payload_length);	
			for (i=0;i<udp_payload_length;i++)
				{
				printf("%02X ",udp_payload_str[i]);
				}
			printf("\n");	
			
            int err = sendto(wifi_udp_sock, udp_payload_str, udp_payload_length, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) 
				{
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
				}
			else				
				ESP_LOGI(TAG, "udp Message sent to %s:%d\n",inet_ntoa(wifi_udp_ip_addr),wifi_udp_port);

			
// receive UDP message
            struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(wifi_udp_sock, wifi_udp_rx_buffer, sizeof(wifi_udp_rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) 
				{
                ESP_LOGE(TAG, "udp recvfrom failed: errno %d", errno);
				}
// Data received
            else 
				{
                wifi_udp_rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, inet_ntoa(wifi_udp_ip_addr));
                ESP_LOGI(TAG, "%s", wifi_udp_rx_buffer);
// show Rx packet data in HEX...
				for (i=0;i<len;i++)
					{
					printf("%02X ",wifi_udp_rx_buffer[i]);
					}
				printf("\n");	
		
                if (strncmp(wifi_udp_rx_buffer, "OK: ", 4) == 0) 
					{
                    ESP_LOGI(TAG, "Received expected message, reconnecting");
					}
				}

			udp_test_flag = 0;
			}
#endif// end of "#ifdef WIFI_UDP_TEST.."			
#endif// end of "#ifdef USE_WIFI_UDP..."

//		four_g_mqtt_state_machine(&four_g_state, &prev_four_g_state);
		
#ifdef USE_M4G
// run the SIMCOM state machine...
	if (simcom_state != SIMCOM_NONE_FOUND)
		{
		four_g_simcom_state_machine(&simcom_state,&prev_simcom_state);
		simcom_busy_change(&simcom_busy_flag, &prev_simcom_busy_flag, "SIMCOM_SS");
		}

	if (simcom_state == SIMCOM_READY_IDLE)
		{
#ifdef USE_M4G_MQTT
//		if (!(simcom_busy_flag & SIMCOM_BUSY_UDP))		// (!(simcom_busy_flag & (~SIMCOM_BUSY_MQTT)))  ie, not any other flag than _MQTT...
		if (!(simcom_busy_flag & (~SIMCOM_BUSY_MQTT)))  // ie, not any other flag than _MQTT...
			{
			four_g_mqtt_state_machine(&mqtt_state, &prev_mqtt_state, &simcom_state);
			simcom_busy_change(&simcom_busy_flag, &prev_simcom_busy_flag, "MQTT_SS");
			}
#endif		
#ifdef USE_M4G_UDP
//		if (!(simcom_busy_flag & SIMCOM_BUSY_MQTT))
		if (!(simcom_busy_flag & (~SIMCOM_BUSY_UDP)))  // ie, not any other flag than _UDP...
			{
			four_g_udp_state_machine(&udp_state, &prev_udp_state, &simcom_state);
			simcom_busy_change(&simcom_busy_flag, &prev_simcom_busy_flag, "UDP_SS");
			}
#endif		

	
		}
	else
		{
//		mqtt_state = SIMCOM_POWER_OFF;
//		udp_state = SIMCOM_POWER_OFF;
		}
		
#endif

#endif			
//		four_g_state_machine(&four_g_state, &prev_four_g_state);
//		four_g_http_state_machine(&four_g_state, &prev_four_g_state);


//*****************************************
//*                                       *
//   ZULU Radio Comms                     *
//*                                       *
//*****************************************

#ifdef USE_ZULU_RADIO

#ifdef ZULU_RADIO_MASTER
// Head end \ radio master
//if (gpio_get_level(ZULU_RADIO_MASTER_CTS) == 0) // CTS is active LOW
if(zulu_radio_response_received)
	{
	unsigned int zulu_radio_tx_data_len, zulu_radio_rx_data_len;
	unsigned int ret;
	unsigned char send_msg_flag,send_resp_flag,errcheck;
	unsigned char tx_msg_type, rx_msg_err;
	unsigned char zulu_radio_SRVR_NO_RESPONSE_expected;
// reset WatchDog Timer...
	esp_task_wdt_reset();		

	zulu_radio_SRVR_NO_RESPONSE_expected = 0;
	
// send to slave
	send_msg_flag = 1;

	if (0)
		{
		}
	else if (zulu_radio_run_flag)			// run immediate
		{
//		zulu_radio_hb_timer = 0;

		tx_msg_type = ZULU_RUN_MSG;	// message type

// if run time = 0, use timers
		zulu_radio_tx_data[DATA_START_BYTE+0] = zulu_radio_run_flag - 1;	//run_timer>>8;	
		zulu_radio_tx_data[DATA_START_BYTE+1] = 1;	//run_time>>8;	
		zulu_radio_tx_data[DATA_START_BYTE+2] = 2;	//run_time&0xFF;
		zulu_radio_tx_data_len = 3;

		zulu_radio_run_flag = 0;				// only reset this after it has been used in the data packet!
		}
	else if (zulu_radio_get_timer_flag)			// get timer
		{
		zulu_radio_get_timer_flag = 0;
//		zulu_radio_hb_timer = 0;
		
		tx_msg_type = ZULU_GET_TIMER_MSG;	// message type

		zulu_radio_tx_data[DATA_START_BYTE+0] = 0;	// timer number
		zulu_radio_tx_data_len = 1;
		}
	else if (zulu_radio_set_timer_flag)			// set timer
		{
		zulu_radio_set_timer_flag = 0;
//		zulu_radio_hb_timer = 0;
		
		tx_msg_type = ZULU_SET_TIMER_MSG;	// message type

		zulu_radio_tx_data[DATA_START_BYTE+0] = 0;	// timer number
		zulu_radio_tx_data[DATA_START_BYTE+1] = 1;	// run length>>8 byte
		zulu_radio_tx_data[DATA_START_BYTE+2] = 2;	// run length&0xFF byte
		zulu_radio_tx_data[DATA_START_BYTE+3] = 1;	// timer hrs byte
		zulu_radio_tx_data[DATA_START_BYTE+4] = 2;	// timer mins byte
		zulu_radio_tx_data[DATA_START_BYTE+5] = 3;	// timer secs byte
		zulu_radio_tx_data[DATA_START_BYTE+6] = 3;	// repeat flag byte
		zulu_radio_tx_data_len = 7;
		}
	else if (zulu_radio_status_timer >= ZULU_RADIO_STATUS_TIME)		// status request : lowest priority
		{
		zulu_radio_status_timer = 0;

		tx_msg_type = ZULU_STATUS_MSG;		// message type

		zulu_radio_tx_data_len = 0;
		}
	else if (zulu_radio_hb_timer >= ZULU_RADIO_HEARTBEAT_TIME)		// heartbeat : lowest priority
		{
		zulu_radio_hb_timer = 0;

		tx_msg_type = ZULU_HBEAT_MSG;		// message type

		zulu_radio_tx_data[DATA_START_BYTE+0] = hrs;	
		zulu_radio_tx_data[DATA_START_BYTE+1] = mins;	
		zulu_radio_tx_data[DATA_START_BYTE+2] = secs;	
		zulu_radio_tx_data_len = 3;

		zulu_radio_SRVR_NO_RESPONSE_expected = 1;		// preset this, as no response expected from a heartbeat msg
		}
	else						// no msg to send
		send_msg_flag = 0;

	if (send_msg_flag)
		{
		zulu_radio_rcv_count = 0;
//		zulu_radio_response_received = 0;
		print_end_msg_flag = 0;

		printf("\n");
		if (zulu_radio_msgs_flag)
			print_chars('=',60,1);
		
		gpio_set_level(ZULU_RADIO_MASTER_RTS,0);	// RTS is active LOW

		send_zulu_radio_msg(zulu_radio_tx_data, ZULU_RADIO_MASTER_ID, zulu_radio_tx_seq_num, tx_msg_type, zulu_radio_tx_data_len);

		if (zulu_radio_SRVR_NO_RESPONSE_expected)		// ie, has been preset, as no response expected from a heartbeat msg
			{
			zulu_radio_response_received = 1;
			zulu_radio_rcv_count = 0xFF;		// turn off rcv warning msg
			print_end_msg_flag = 1;
			zulu_radio_tx_seq_num++;
			}
		else		
			{
			zulu_radio_response_received = 0;
			zulu_radio_rcv_count = 0;
			print_end_msg_flag = 0;
			}

		uart_wait_tx_done(ZULU_RADIO,100);
//		gpio_set_level(ZULU_RADIO_MASTER_RTS,1);	// RTS is active LOW

		}
	}
else
	{	
	if (zulu_radio_msgs_flag)
		printf("Radio CTS not ready!\n");
	}


// receive response from slave...
	{
	unsigned int zulu_radio_rx_data_len;
	unsigned int ret;
	unsigned char send_resp_flag,errcheck;
	unsigned char rx_msg_err;
//	unsigned char print_end_msg_flag = 0;

	if ((zulu_radio_msgs_flag) && (!print_end_msg_flag))
		printf("Rx...");
	
uart_get_buffered_data_len(ZULU_RADIO,&zulu_radio_rx_data_len);
if (zulu_radio_rx_data_len)
	{
	printf("ZM rx\n");

// get response from slave
	zulu_radio_rx_data_len = 0;
	zulu_radio_rx_data[0] = 0x00;

	zulu_radio_rx_data_len = get_uart_rx_msg(ZULU_RADIO, zulu_radio_rx_data, ZULU_RADIO_RX_BUFSIZE, 50);

// for test...
#if 0
	memcpy(zulu_radio_rx_data,zulu_radio_tx_data, 12);
	zulu_radio_rx_data_len = zulu_radio_rx_data[MSG_LEN_BYTE] + 4;
#endif
	
//		vTaskDelay(100 / portTICK_PERIOD_MS);	// yield to OS for 100 milliseconds

//	if ((zulu_radio_rx_data_len) && (zulu_radio_rx_data[0] ==  STX) && (zulu_radio_rx_data[zulu_radio_rx_data_len - 1] == ETX))
	if (zulu_radio_rx_data_len)
		{
		zulu_radio_timeout_timer = 0;
		zulu_radio_response_received = 1;
		
		if (zulu_radio_msgs_flag)
			{
			show_time(DBG,1);
			printf("  Radio Master Rx RESPONSE: [%s]  [%d]\n",zulu_radio_msg_names[zulu_radio_rx_data[MSG_TYPE_BYTE]],zulu_radio_rx_data_len);
			printf("STX DEV SEQ TYP LEN DATA");
			if (zulu_radio_rx_data_len > 9)
				{
				for (unsigned char i=0;i<(zulu_radio_rx_data_len-9);i++)
					{
					printf("----");
					}
				}
				
			printf("->  CHK ETX\n");

			for (unsigned char i=0;i<zulu_radio_rx_data_len;i++)
				{
				printf("%02X  ",zulu_radio_rx_data[i]);
				}
			printf("\n\n");
			}	

// check for errors

		rx_msg_err = 0;
		
		if (zulu_radio_rx_data_len > MSG_LEN_BYTE)
		{
// check STX and ETX flags
		if ((zulu_radio_rx_data[0] != STX) || (zulu_radio_rx_data[zulu_radio_rx_data_len - 1] != ETX))
			rx_msg_err = rx_msg_err | ZULU_RADIO_FLAG_ERROR;

// check data checksum		
		errcheck = 0;
		for (unsigned char i=1;i<(zulu_radio_rx_data_len-2);i++)
			{
			errcheck = errcheck + zulu_radio_rx_data[i];
			}
			
		if (zulu_radio_msgs_flag)
			printf("err chk - 0x%02X : 0x%02X - ",zulu_radio_rx_data[zulu_radio_rx_data_len - 2],errcheck);
		if (errcheck == zulu_radio_rx_data[zulu_radio_rx_data_len - 2])
			{
			if (zulu_radio_msgs_flag)				
				printf("OK");
			}
		else
			{
			rx_msg_err = rx_msg_err | ZULU_RADIO_CHKSUM_ERROR;
			if (zulu_radio_msgs_flag)
				printf("ERROR");
			}

// check sequence number		
		if (zulu_radio_msgs_flag)
			printf("\t\tSeq num %d : %d - ",zulu_radio_rx_data[SEQ_NUM_BYTE],zulu_radio_tx_seq_num);
		if (zulu_radio_rx_data[SEQ_NUM_BYTE] == zulu_radio_tx_seq_num)
			{
			if (zulu_radio_msgs_flag)
				printf("OK");
			}
		else
			{
			rx_msg_err = rx_msg_err | ZULU_RADIO_SEQ_NUM_ERROR;
			zulu_radio_rx_seq_num = zulu_radio_rx_data[SEQ_NUM_BYTE];
			if (zulu_radio_msgs_flag)
				printf("ERROR");
			}

		if (zulu_radio_msgs_flag)
			printf("\n\n");
		}

// can only inc seq num after the seq num test has been done!
		zulu_radio_tx_seq_num++;

		
//		if ((zulu_radio_rx_data[SEQ_NUM_BYTE] == zulu_radio_rx_seq_num) && (errcheck == zulu_radio_rx_data[zulu_radio_rx_data_len - 2]))
//		if (errcheck == zulu_radio_rx_data[zulu_radio_rx_data_len - 2])
		if ((rx_msg_err & (!ZULU_RADIO_SEQ_NUM_ERROR)) == 0)	// igonore seq num errors...
			{
//			printf("processing msg...\n");
			send_resp_flag = 1;
			switch(zulu_radio_rx_data[MSG_TYPE_BYTE])
				{
				case ZULU_HBEAT_MSG:
//					printf("HB msg\n");
	//				x = zulu_radio_rx_data[DATA_START];		// TOD hrs
	//				x = zulu_radio_rx_data[DATA_START+1];	// TOD mins
	//				x = zulu_radio_rx_data[DATA_START+2];	// TOD secs
	// check local clock and adjust if necessary...
					send_resp_flag = 0;
					break;
				case ZULU_STATUS_MSG:
					break;
				case ZULU_RUN_MSG:
					break;
				case ZULU_GET_TIMER_MSG:
					break;
				case ZULU_SET_TIMER_MSG:
					break;
				default:
					send_resp_flag = 0;
					break;
				}
			}
		else
			printf("Slave response error - code %d\n",rx_msg_err);

		zulu_radio_rx_seq_num++;
		}
	else
		{
		if (zulu_radio_timeout_timer >= ZULU_RX_TIMEOUT)
			{
			if (zulu_radio_msgs_flag)
				printf("No response from Slave\n\n");
			}
		}
	
/*
	if (zulu_radio_msgs_flag)
		{
		print_chars('=',60,1);
		printf("\n");
		}
*/
	print_end_msg_flag = 1;

	}
else
	{
	}
	
//if (!zulu_radio_response_received)
if (zulu_radio_rcv_count < 0xFF)
	{
	zulu_radio_rcv_count++;
	if (zulu_radio_rcv_count ==80)
		{
		zulu_radio_response_received = 1;
		zulu_radio_tx_seq_num++;
		if (zulu_radio_msgs_flag)
			{
			show_time(DBG,1);
			printf("  Slave response timeout\n");
			print_end_msg_flag = 1;
			}
		}
	}

if ((print_end_msg_flag) && (zulu_radio_msgs_flag))
	{
	print_chars('=',60,1);
	printf("\n");
	print_end_msg_flag = 0;
	}
	
	
	}



	
#endif	//end of "#ifdef ZULU_RADIO_MASTER...



#ifdef ZULU_RADIO_SLAVE
// device end \ radio slave
{
unsigned int zulu_radio_tx_data_len, zulu_radio_rx_data_len;
unsigned char send_msg_flag,send_resp_flag,errcheck;
unsigned char tx_msg_type, rx_msg_err;

// reset WatchDog Timer...
	esp_task_wdt_reset();		

// receive from master
printf("ZS rx\n");
zulu_radio_rx_data_len = get_uart_rx_msg(ZULU_RADIO, zulu_radio_rx_data, ZULU_RADIO_RX_BUFSIZE, 50);

//if ((zulu_radio_rx_data_len) && (zulu_radio_rx_data[0] ==  STX) && (zulu_radio_rx_data[zulu_radio_rx_data_len - 1] == ETX))
if (zulu_radio_rx_data_len)
	{
	zulu_radio_timeout_timer = 0;
		
	if (zulu_radio_msgs_flag)
		{	
		print_chars('=',60,1);
		show_time(DBG,1);
		printf("  Radio Slave Rx CMD: [%s]  [%d]\n",zulu_radio_msg_names[zulu_radio_rx_data[MSG_TYPE_BYTE]],zulu_radio_rx_data_len);
		printf("STX DEV SEQ TYP LEN DATA");
		if (zulu_radio_rx_data_len > 9)
			{
			for (unsigned char i=0;i<(zulu_radio_rx_data_len-9);i++)
				{
				printf("----");
				}
			}
		printf("->  CHK ETX\n");

		for (unsigned char i=0;i<zulu_radio_rx_data_len;i++)
			{
			printf("%02X  ",zulu_radio_rx_data[i]);
			}
		printf("\n\n");
		}
		
// check for errors
		rx_msg_err = 0;
		if (zulu_radio_rx_data_len > MSG_LEN_BYTE)
		{
// check STX and ETX flags
		if ((zulu_radio_rx_data[0] !=  STX) || (zulu_radio_rx_data[zulu_radio_rx_data_len - 1] != ETX))
			rx_msg_err = rx_msg_err | ZULU_RADIO_FLAG_ERROR;

// check data checksum
			errcheck = 0;
	for (unsigned char i=1;i<(zulu_radio_rx_data_len-2);i++)
		{
		errcheck = errcheck + zulu_radio_rx_data[i];
		}

		if (zulu_radio_msgs_flag)
			printf("err chk - 0x%02X : 0x%02X - ",zulu_radio_rx_data[zulu_radio_rx_data_len - 2],errcheck);
		if (errcheck == zulu_radio_rx_data[zulu_radio_rx_data_len - 2])
			{	
			if (zulu_radio_msgs_flag)
				printf("OK");
			}
		else
			{
			rx_msg_err = rx_msg_err | ZULU_RADIO_CHKSUM_ERROR;
			if (zulu_radio_msgs_flag)
				printf("ERROR");
			}

// check sequence number		
		if (zulu_radio_msgs_flag)
			printf("\t\tSeq num %d : %d - ",zulu_radio_rx_data[SEQ_NUM_BYTE],zulu_radio_rx_seq_num+1);
		if (zulu_radio_rx_data[SEQ_NUM_BYTE] == (zulu_radio_rx_seq_num+1))
			{	
			zulu_radio_rx_seq_num++;
			if (zulu_radio_msgs_flag)
				printf("OK");
			}
		else
			{
			rx_msg_err = rx_msg_err | ZULU_RADIO_SEQ_NUM_ERROR;
			zulu_radio_rx_seq_num = zulu_radio_rx_data[SEQ_NUM_BYTE];
			if (zulu_radio_msgs_flag)
				printf("ERROR");
			}
		
		if (zulu_radio_msgs_flag)
			printf("\n\n");
		}
		
//	if (errcheck == zulu_radio_rx_data[zulu_radio_rx_data_len - 2])
	if ((rx_msg_err & (!ZULU_RADIO_SEQ_NUM_ERROR)) == 0)	// igonore seq num errors...
		{
		send_resp_flag = 1;
		tx_msg_type = 0;			// some initialised value - keeps the compiler happy...
		zulu_radio_tx_data_len = 0;			// some initialised value - keeps the compiler happy...
		switch(zulu_radio_rx_data[MSG_TYPE_BYTE])
			{
			case ZULU_HBEAT_MSG:
//				x = zulu_radio_rx_data[DATA_START];		// TOD hrs
//				x = zulu_radio_rx_data[DATA_START+1];	// TOD mins
//				x = zulu_radio_rx_data[DATA_START+2];	// TOD secs
// check local clock and adjust if necessary...
				if ((hrs != zulu_radio_rx_data[DATA_START_BYTE]) || (mins != zulu_radio_rx_data[DATA_START_BYTE+1]) || (secs != zulu_radio_rx_data[DATA_START_BYTE+2]))
					{
					hrs = zulu_radio_rx_data[DATA_START_BYTE];
					mins = zulu_radio_rx_data[DATA_START_BYTE+1];
					secs = zulu_radio_rx_data[DATA_START_BYTE+2];
					printf("Clock Resync!\n");
					}
				
				send_resp_flag = 0;
				break;
			case ZULU_STATUS_MSG:
// create slave status byte			
				zulu_radio_slave_status = 0;

				tx_msg_type = ZULU_STATUS_MSG;	// message type

				zulu_radio_tx_data[DATA_START_BYTE+0] = zulu_radio_slave_status;	// timer number
				zulu_radio_tx_data[DATA_START_BYTE+1] = 1;							// timer hrs byte
				zulu_radio_tx_data[DATA_START_BYTE+2] = 2;							// timer mins byte
				zulu_radio_tx_data[DATA_START_BYTE+3] = 3;							// timer secs byte

				zulu_radio_tx_data[DATA_START_BYTE+4] = 1;							// local hrs byte
				zulu_radio_tx_data[DATA_START_BYTE+5] = 2;							// local mins byte
				zulu_radio_tx_data[DATA_START_BYTE+6] = 3;							// local secs byte
				zulu_radio_tx_data_len = 7;

				break;
				
			case ZULU_RUN_MSG:
				if (zulu_radio_msgs_flag)
					printf("Run = %d\n",zulu_radio_tx_data[DATA_START_BYTE+0]);
				
				if (zulu_radio_tx_data[DATA_START_BYTE+0])
					{
					set_output(DEV_LED_3,DEVICE_ON,1);
					zulu_radio_run_timer = 0;
					}
				else
					{
					set_output(DEV_LED_3,DEVICE_OFF,1);
					}

// create slave status byte			
				zulu_radio_slave_status = 0;

				tx_msg_type = ZULU_RUN_MSG;	// message type

				zulu_radio_tx_data[DATA_START_BYTE+0] = zulu_radio_slave_status;	// timer number
				zulu_radio_tx_data[DATA_START_BYTE+1] = 0;		// sw status byte
				zulu_radio_tx_data[DATA_START_BYTE+2] = 1;							// timer hrs byte
				zulu_radio_tx_data[DATA_START_BYTE+3] = 2;							// timer mins byte

				zulu_radio_tx_data_len = 4;
				break;
				
			case ZULU_GET_TIMER_MSG:
				break;
				
			case ZULU_SET_TIMER_MSG:
				break;
				
			default:
				send_resp_flag = 0;
				break;
			}
		}
	else
		{
		printf("Master cmd error - code %d\n",rx_msg_err);

// send STATUS = NAK response to master
// create slave status byte			
		zulu_radio_slave_status = 0;

		tx_msg_type = ZULU_STATUS_MSG;	// message type

		zulu_radio_tx_data[DATA_START_BYTE+0] = zulu_radio_slave_status;	// timer number
		zulu_radio_tx_data[DATA_START_BYTE+1] = 1;							// timer hrs byte
		zulu_radio_tx_data[DATA_START_BYTE+2] = 2;							// timer mins byte
		zulu_radio_tx_data[DATA_START_BYTE+3] = 3;							// timer secs byte

		zulu_radio_tx_data[DATA_START_BYTE+4] = 1;							// local hrs byte
		zulu_radio_tx_data[DATA_START_BYTE+5] = 2;							// local mins byte
		zulu_radio_tx_data[DATA_START_BYTE+6] = 3;							// local secs byte
		zulu_radio_tx_data_len = 7;

		send_resp_flag = 1;
		}
		
// send response to master		
	if (send_resp_flag)
		{
		gpio_set_level(ZULU_RADIO_SLAVE_RTS,0);	// RTS is active LOW

// create slave status byte			
		zulu_radio_slave_status = 0;
		send_zulu_radio_msg(zulu_radio_tx_data, 0x01, zulu_radio_rx_seq_num, tx_msg_type, zulu_radio_tx_data_len);
//		ret = uart_write_bytes(ZULU_RADIO, (char *)zulu_radio_tx_data, zulu_radio_data_len);

		uart_wait_tx_done(ZULU_RADIO,100);
//		gpio_set_level(ZULU_RADIO_SLAVE_RTS,1);	// RTS is active LOW
		}

	if (zulu_radio_msgs_flag)
		{
		print_chars('=',60,1);
		printf("\n");
		}
	}
else
	{
	if (zulu_radio_timeout_timer >= ZULU_RX_TIMEOUT)
		{
		if (zulu_radio_msgs_flag)
			printf("No response from Master\n\n");
		}
	}

if (zulu_radio_run_timer > ZULU_RUN_TIME)
	{
	set_output(DEV_LED_3,DEVICE_OFF,1);
	}

}
#endif	// end of "#ifdef ZULU_RADIO_SLAVE...


#endif	// end of "#ifdef USE_ZULU_RADIO...


/////////////////////////////////
//
// debug interpreter			
//
/////////////////////////////////
		if (cmd_flag)
			{
//			printf("Enter CLI\r\n");
//			printf(">%s\r\n",cmd_string);
			cli(cmd_string);
			cmd_string[0] = 0x00;
			cmd_flag = 0;
			}


#ifdef USE_CPU_SLEEP
	if (sleep_resume_flag)
		{
		sleep_flag = 1;
		sleep_resume_flag = 0;
#ifdef SHOW_SLEEP_MSGS
		if(debug_do(DBG_SLEEP))
			{
			printf("Sleep...\r\n");
			}
#endif
		}
#endif

	if(wifi_is_on)
		{
		vTaskDelay(25 / portTICK_PERIOD_MS);	// yield to OS for 25 milliseconds
//	printf("Slp1,,,\r\n");
		}
	else
		{
		vTaskDelay(1 / portTICK_PERIOD_MS);	// yield to OS for 1 millisecond

#ifdef XX_USE_CPU_SLEEP
// put ESP32 into light sleep mode (memory retained OK)
		esp_sleep_enable_timer_wakeup(24000);	// time in usec
		esp_light_sleep_start();
#else
		vTaskDelay(24 / portTICK_PERIOD_MS);	// yield to OS for 25 milliseconds
#endif
		}

	}	// end of "if (x100ms_flag)...
	
// reset WatchDog Timer for this thread...
	esp_task_wdt_reset();		

//	printf("Slp2,,,\r\n");
#ifdef USE_CPU_SLEEP
/*
	if (uart_stay_awake_count)
		{
		uart_stay_awake_count--;
		if (uart_stay_awake_count==0)
		printf("END!");
		}
	else
*/
	if (uart_stay_awake_count==0)
		sleep_cycle_count++;
	
	if ((sleep_flag) && (sleep_cycle_count >= 20))	// 20 msec
		{
		sleep_cycle_count = 0;
// put ESP32 into light sleep mode (memory retained OK)
//#ifdef V1_PCB
		esp_sleep_enable_timer_wakeup(100000);	// 500msec; time in usec
//ESP_ERROR_CHECK(esp_sleep_enable_uart_wakeup(0));
//ESP_ERROR_CHECK(uart_set_wakeup_threshold(0, 3));
//ESP_ERROR_CHECK(esp_sleep_enable_uart_wakeup(UART_NUM_1));
//ESP_ERROR_CHECK(uart_set_wakeup_threshold(UART_NUM_1, 3));
		esp_sleep_enable_uart_wakeup(UART_NUM_0);
		uart_set_wakeup_threshold(UART_NUM_0, 3);
		esp_light_sleep_start();

		esp_sleep_wakeup_cause_t wakeup_cause;
		
		wakeup_cause = esp_sleep_get_wakeup_cause();
		if (wakeup_cause == ESP_SLEEP_WAKEUP_UART)
			{
			uart_stay_awake_count = uart_stay_awake_time;
			printf("Serial Session Start: %dsec\r\n",uart_stay_awake_count/10);
			}
//#endif

		
		}
	else
		{
		vTaskDelay(1 / portTICK_PERIOD_MS);	// 1 millisecond	
		}
		
#else
    vTaskDelay(1 / portTICK_PERIOD_MS);	// 1 millisecond
#endif


#ifdef USE_VTASKLIST			
vTaskList(vtaskList);
ESP_LOGW("MAC Rpt Tasks B:","%s",vtaskList);			
#endif


	}	// end of while(1) loop


}	// end of main


static void periodic_100mseconds_timer_callback(void* arg)
{
int64_t time_since_boot = esp_timer_get_time();
unsigned char inc_month = 0;

if (debug_do(DBG_TIMER_CALLS))
   ESP_LOGI(TAG, "Periodic timer called, time since boot: %lld us", time_since_boot);

	x100msec++;

	if (x100msec>=10)
		{
		x100msec = 0;
		secs++;
		x1sec_flag = 1;

#ifdef USE_WIFI_UDP
#ifdef WIFI_UDP_TEST
// every 5 sec test UDP...
		if (secs%5 == 0)
			udp_test_flag = 1;
#endif
#endif

		if (secs % 10 == 0)
			x10sec_flag = 1;
		
		if (secs>=60)
			{
			secs = 0;
			mins++;
			up_time++;

			if (get_server_time_timer < (GET_SERVER_TIME_PERIOD * 60))
				get_server_time_timer++;
			else
				{
				get_server_time_timer = 0;
				get_server_time_flag = 1;
				}

			
			if (mins >= 60)
				{
				mins = 0;
				hrs++;

				if (hrs >= 24)
					{
					hrs = 0;
					days++;
					
					day_of_week++;
					if (day_of_week >= 7)
						day_of_week = 0;		// 0 = Sun, 6 = Sat
					
					switch(month)
						{
						case 1:		// Jan 31
						case 3:		// Mar 31
						case 5:		// May 31
						case 7:		// Jul 31
						case 8:		// Aug 31
						case 10:	// Oct 31
						case 12:	// Dec 31
							if (days < 31)
								inc_month = 1;
							break;
						case 4:		// Apr 30
						case 6:		// Jun 30
						case 9:		// Sep 30
						case 11:	// Nov 30
							if (days < 30)
								inc_month = 1;
							break;

						case 2:		// Feb 28 or 29
							if (days < 28)
								inc_month = 1;
							break;
						
						}
					
					if (inc_month)
						{
						days = 1;
						month++;
						if (month > 12)
							{
							month = 1;
							year++;
							
							if (year > 99)
								year = 0;
							}
						}
						
					}
				}
			}
		}
		
    x100msec_flag = 1;

// up-time counter
	u100msec++;
	if (u100msec>=10)
		{
		u100msec = 0;
		u1sec++;
		if (u1sec>=60)
			{
			u1sec = 0;
			u1min++;
			if (u1min>=60)
				{
				u1min=0;
				u1hr++;
				if (u1hr>=24)
					{
					u1hr=0;
					u1day++;
					}
				}
			}
		}

}


static void oneshot_timer_callback(void* arg)
{
//    int64_t time_since_boot = esp_timer_get_time();
//    ESP_LOGI(TAG, "One-shot timer expired: %lld us", time_since_boot);

    ESP_LOGI("ONS", "MRI en\n");

} 


//SemaphoreHandle_t xSemaphore = NULL;
//xSemaphore = xSemaphoreCreateBinary();

////////////////////////////////
// IR sensor pulse measurement
////////////////////////////////
/*
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
//uint32_t gpio_num = (uint32_t) arg;

pulse_end_time = esp_timer_get_time();

pulse_time = pulse_end_time - pulse_start_time;
pulse_start_time = pulse_end_time;

if ((pulse_time > 750) && (pulse_time < 1100))  // times in usec...
	{	
	idata++;
	inopulse = 0;
	}
else
	{
	idata = 0;
	inopulse = 1;
	}
//xSemaphoreGiveFromISR(xSemaphore,NULL);
//printf("INT\r\n");
//idata++;
}
*/


#ifdef LORA_USED	//_LORA
// LORA int pin service routines
static void IRAM_ATTR lora_dio0_isr_handler(void* arg)
{
uint32_t gpio_num = (uint32_t) arg;

xQueueSendFromISR(lora_gpio_queue, &gpio_num, NULL);

LORA_int_flag |= 4;

//	ESP_LOGI("LOR","DIO0\n");

//printf("DIO0 int!\n");	

//lora_rcv_int();
//printf("%c",0x07);	// BELL char
}

static void IRAM_ATTR lora_dio1_isr_handler(void* arg)
{
uint32_t gpio_num = (uint32_t) arg;

xQueueSendFromISR(lora_gpio_queue, &gpio_num, NULL);
//printf("DIO1 int!\n");	
}

static void IRAM_ATTR lora_dio2_isr_handler(void* arg)
{
uint32_t gpio_num = (uint32_t) arg;

xQueueSendFromISR(lora_gpio_queue, &gpio_num, NULL);
//printf("DIO2 int!\n");	
}

static void IRAM_ATTR lora_dio3_isr_handler(void* arg)
{
uint32_t gpio_num = (uint32_t) arg;

xQueueSendFromISR(lora_gpio_queue, &gpio_num, NULL);
//printf("DIO3 int!\n");	
}

static void IRAM_ATTR lora_dio4_isr_handler(void* arg)
{
uint32_t gpio_num = (uint32_t) arg;

xQueueSendFromISR(lora_gpio_queue, &gpio_num, NULL);
//printf("DIO4 int!\n");	
}

/*
static void IRAM_ATTR lora_dio5_isr_handler(void* arg)
{
printf("DIO5 int!\n");	
}
*/
#endif

static void IRAM_ATTR ext_serial_bitrate_isr_handler(void* arg)
{
//uint32_t gpio_num = (uint32_t) arg;
uint32_t bit_time;
uint64_t bit_time_end;

bit_time_end = esp_timer_get_time();		// time in usec

if (bit_time_end > bit_time_start)
	{
	bit_time = bit_time_end - bit_time_start + 2;		// 2 us offset occurring?
	bit_time_start = bit_time_end;

	if((bit_time_counter > 20) && (bit_time < ext_serial_bit_time))
		{
		ext_serial_bit_time = bit_time;
		ext_serial_bit_time_change_flag = 1;
		}
	}
	
// reset the detector every few bytes (in case the bit_time gets set to an unusually low number)...	
bit_time_counter++;
if(bit_time_counter > 250)
	{
	ext_serial_bit_time = 0xFFFFFFFF;
	bit_time_counter = 0;
	}
//printf("DIO4 int!\n");	
}


//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
void spi_pre_transfer_callback(spi_transaction_t *t) 
{
//	printf("spi pre\n");
//    int dc=(int)t->user;
//    gpio_set_level(PIN_NUM_DC, dc);
}

#ifdef USE_ULTRASONIC_SENSOR
//uint64_t usm_start_time = 0;
//uint64_t usm_end_time;

static void IRAM_ATTR usm_isr_handler(void* arg)
{
	// DONT TRY TO USE PRINTF IN HERE!
// isr is triggered on both edges of the GPIO line...

if (gpio_get_level(USM_ECHO) == 1)	// start of echo pulse
	{
	usm_start_time = esp_timer_get_time();		// time in usec
//	printf("U=1\n");
	}
else								// end of echo pulse
	{
	usm_end_time = esp_timer_get_time();		// time in usec
	usm_echo_time = usm_end_time - usm_start_time;		// time in usec
	usm_state = USM_COMPLETED;
//	printf("U=0\n");
	}
usm_int_count++;
}
#endif



#ifdef LORA_USED	//_LORA
void lora_rcv(void)
{
unsigned char x,y;
unsigned char val[4];

//printf("entered rcv\n");

// get int reg
SX1276_read_reg(spi,LREG_IRQ_FLAGS,&x,1);


// put radio into sleep mode while reading...
SX1276_read_reg(spi,LREG_OPMODE,&y,1);
//x = (x & 0xF8) | 0x07;	// channel activity detect mode
x = (x & 0xF8) | 0x00;	// sleep mode
SX1276_write_reg(spi,LREG_OPMODE,&y,1);



//x = 0xFF;	// need to write a '1' to clear an int bit...
SX1276_write_reg(spi,LREG_IRQ_FLAGS,&x,1);

// after important stuff, sound the bell...
if (debug_do(DBG_LORA_RCV))
	{
	printf("%c",0x07);	// BELL char
	printf("LORA int reg: %02X  flag: %02X\n",x,LORA_int_flag);
	}

LORA_int_flag = 0;

//printf("rcv process 1\n");

if (x & 0x15)	// hdr \ CAD flags
{
unsigned char x;
/*
SX1276_read_reg(spi,LREG_OPMODE,&x,1);
x = (x & 0xF8) | 0x01;	// STDBY mode
SX1276_write_reg(spi,LREG_OPMODE,&x,1);

SX1276_read_reg(spi,LREG_OPMODE,&x,1);
//x = (x & 0xF8) | 0x07;	// channel activity detect mode
x = (x & 0xF8) | 0x05;	// Rx_Continuous mode
SX1276_write_reg(spi,LREG_OPMODE,&x,1);
*/
}

//printf("rcv process 2\n");

if (x & 0x50)	// Rx Done \ Valid Hdr flags
{
unsigned char i,n;
unsigned char rxdata[50];

if (1)	//x == 0x40)	//50)
	{
	unsigned char n;
	uint8_t *lora_rx_msg = NULL;


// get SNR
	SX1276_read_reg(spi,LREG_PKT_SNR_VAL,&n,1);
	lora_snr = ((n ^ 0xFF) + 1)/4;
	if (debug_do(DBG_LORA_RCV))
		printf("SNR: %ddB",lora_snr);
// get last packet RSSI
	SX1276_read_reg(spi,LREG_PKT_RSSI_VAL,&n,1);
	lora_pkt_rssi = n - 157;
	if (debug_do(DBG_LORA_RCV))
		printf("  Pkt RSSI: %ddB",lora_pkt_rssi);
// get current RSSI
	SX1276_read_reg(spi,LREG_RSSI_VAL,&n,1);
	lora_rssi = n - 157;
	if (debug_do(DBG_LORA_RCV))
		printf("  Curr RSSI: %ddB\n",lora_rssi);

// get Rx data	
	SX1276_read_reg(spi,LREG_RX_NUM_BYTES,&n,1);

//	if (n == 0)
//	printf("n = %d\n",n);
//	ets_delay_us(1000);

// problem here if rally code receives < 8 bytes...
	if (n < 10)
		{	
		lora_rx_msg = (uint8_t *)malloc(10);
		if (debug_do(DBG_ALLOC_MEM))
			printf("LoRa rx_msg data alloc %d\n",10);
		}
	else
		{
		lora_rx_msg = (uint8_t *)malloc(n);
		if (debug_do(DBG_ALLOC_MEM))
			printf("LoRa rx_msg data alloc %d\n",n);
		}	
	if (lora_rx_msg != NULL)
	{
// set up FIFO address for Rx data
	SX1276_read_reg(spi,LREG_FIFO_RX_CURR_ADDR,&x,1);
	SX1276_write_reg(spi,LREG_FIFO_ADDR_PTR,&x,1);

	if (debug_do(DBG_LORA_RCV))
		printf("Rx data[%d]:\n",n);
	SX1276_read_reg(spi,LREG_FIFO,val,4);
	for (i=0;i<4;i++)
		{
		lora_rx_msg[i] = val[i];
		if (debug_do(DBG_LORA_RCV))
			printf(" %02X",val[i]);
		}

	SX1276_read_reg(spi,LREG_FIFO,val,4);
	for (i=4;i<8;i++)
		{
		lora_rx_msg[i] = val[i-4];
		if (debug_do(DBG_LORA_RCV))
			printf(" %02X",val[i-4]);
		}

	for (i=8;i<n;i++)
		{
		SX1276_read_reg(spi,LREG_FIFO,&x,1);
		lora_rx_msg[i] = x;
		if (debug_do(DBG_LORA_RCV))
			printf(" %02X",x);
		}
	if (debug_do(DBG_LORA_RCV))
		printf("\n\n");

#ifdef USE_LORA_LINK
	if (lora_rx_msg[0] == LORA_HDR)
		{
		for (i=0;i<n;i++)
			lora_rx_data[i] = lora_rx_msg[i];
		
		lora_rx_data[i] = 0x00;
		
		lora_rx_data_len = n;
		lora_rx_flag = 1;
		}
#endif
	
// put msg onto outgoing msg queue...
//	lora_rx_data_wr->q_data = lora_rx_msg;
//	lora_rx_data_wr->q_data_len = n;

#ifdef LORAWAN_SCAN
// add this MAC addr to the list of MAC addresses
			if (lorawan_MAC_entries < LORAWAN_LIST_LEN)
				{
				unsigned char found_flag = 0;
				unsigned char dev_addr[5];
// check to see if it is on the MAC address list...
			for (i=0;i<4;i++)
				{
//				dev_addr[i] = lora_rx_data_wr->q_data[1+i];
				dev_addr[i] = lora_rx_msg[1+i];
				}
			dev_addr[i] = 0x00;
// check first that the MAC address isnt already on the list?					
				for (i=0;i<lorawan_MAC_entries;i++)
					{
					if (!memcmp(dev_addr,&lorawan_MAC_list[i],4))
						found_flag = 1;
					}
					
				if (!found_flag)						// if no match found on the list
					{
// do we need to check first that the MAC address isnt already on the list?					
					memcpy(&lorawan_MAC_list[lorawan_MAC_entries],&lora_rx_msg[1],4);	// LoRaWAN DecAddr is 4 bytes long...
					lorawan_MAC_entries++;
					}
				}

#endif

#ifdef USE_LORA		// UES_LORA?
// put msg onto outgoing msg queue...
	lora_rx_data_wr->q_data = lora_rx_msg;
	lora_rx_data_wr->q_data_len = n;

/////////////////////////////////////////////////////////////
// whitelist \ blacklist now done before the data is enqueued...
				{
				unsigned char send_lora_data_flag = 1;
				char dev_addr[5];
				
				send_lora_data_flag = queues_enabled_flag;
				
				printf("dev_addr: ");
//				memcpy(dev_addr,&lora_rx_data_rd->q_data[4],4);
				for (x=0;x<4;x++)
					{
					dev_addr[x] = lora_rx_msg[3 + 1 - x];
					printf("%02X:",dev_addr[x]);
					}
				dev_addr[4] = 0;

				lora_packet_count = lora_rx_msg[5] * 256 + lora_rx_msg[6];
				printf("     packet count / prev: %d / %d",lora_packet_count,lora_prev_packet_count);
				printf("\n");
						
				if (lora_packet_count != lora_prev_packet_count + 1)
					{
					printf("Resyncing packet count...\n");
					lora_packet_count = lora_rx_msg[7] * 256 + lora_rx_msg[8];
					}

				lora_prev_packet_count = lora_packet_count;

						
// send data if whitelisting not enabled, or there is a data match...				
							
//						if (bt_out_data->q_data[bt_out_data->q_data_len-1] == 0x0A)
									
				if (enable_lorawan_blacklist_flag)
					{
					printf("LoRa BL:");
//					if (listmatch(LORAWAN_CMD_LIST_LENGTH, LORAWAN_CMD_LIST_ENTRY_LENGTH, (char(*)[LORAWAN_CMD_LIST_ENTRY_LENGTH])lorawan_cmd_blacklist, (char*)lora_rx_data_rd->q_data,1))
//					if (listmatch(LORAWAN_CMD_LIST_LENGTH, LORAWAN_CMD_LIST_ENTRY_LENGTH, (char *)lorawan_cmd_blacklist, dev_addr,1,1))	//(char*)lora_rx_data_rd->q_data, 1, 1))
					if (listmatch(num_lora_bl_entries, LORAWAN_CMD_LIST_ENTRY_LENGTH, (char *)lorawan_cmd_blacklist, dev_addr,1,1))	//(char*)lora_rx_data_rd->q_data, 1, 1))
						{
						send_lora_data_flag = 0;									
						}
					}

				if ((enable_lorawan_whitelist_flag) && (send_lora_data_flag))		// if whitelist enabled and the data hasn't been blacklisted already...
					{
					send_lora_data_flag = 0;
					printf("LoRa WL:");
//					if (listmatch(LORAWAN_CMD_LIST_LENGTH, LORAWAN_CMD_LIST_ENTRY_LENGTH, (char(*)[LORAWAN_CMD_LIST_ENTRY_LENGTH])lorawan_cmd_whitelist, (char*)lora_rx_data_rd->q_data,1))
//					if (listmatch(LORAWAN_CMD_LIST_LENGTH, LORAWAN_CMD_LIST_ENTRY_LENGTH, (char *)lorawan_cmd_whitelist, dev_addr,1,1))	//(char*)lora_rx_data_rd->q_data,1 1, ))
					if (listmatch(num_lora_wl_entries, LORAWAN_CMD_LIST_ENTRY_LENGTH, (char *)lorawan_cmd_whitelist, dev_addr,1,1))	//(char*)lora_rx_data_rd->q_data,1 1, ))
						{
						send_lora_data_flag = 1;
						}
					}							





/////////////////////////////////////////////////////////////
				if (send_lora_data_flag)
					{
#if 0
					printf("\nCheck:\n");
					for (i=0;i<12;i++)
						{	
						printf("%02X %02X|",lora_rx_msg[i],lora_rx_data_wr->q_data[i]); 
						}
					printf("\n\n");
#endif
//xQueueSendFromISR		
					if (xQueueSend(lora_data_queue, lora_rx_data_wr, ( TickType_t ) 0) != pdTRUE) 
//##	if (xQueueSendFromISR(lora_data_queue, lora_rx_data_wr, NULL) != pdTRUE) 
						{
						ESP_LOGD(TAG, "Failed to enqueue LORA msg output. Queue full.");
						printf("LORA msg queue full.\r\n");
// free memory taken by lora_rx_msg packet, as not used due to error
						free(lora_rx_msg);
						}
					else
						{
						lora_msg_ready_count++;				// flag to fg to send msg data

//						if(debug_do(DBG_BLUETOOTH))
							{
							printf("No of queued LORA rx_msgs: %d\t\t",lora_msg_ready_count);
							printf("Queue free size: %d\n", uxQueueSpacesAvailable(lora_data_queue));
							printf("%c",0x07);	// BELL char
							}
						}
//	printf("end of LORA enqueue\n");
					}
				else
					{
					free(lora_rx_msg);	// alloc for message was not used...
					printf("LoRa mem freed!\n");
					}
				}
#endif							
	}
	else
		printf("LORA malloc error!\n");


	}
else
	printf("Rx ERR: FLAGS=%02X\n",x);

}

// set back to Rx mode
SX1276_read_reg(spi,LREG_OPMODE,&x,1);
//x = (x & 0xF8) | 0x07;	// channel activity detect mode
x = (x & 0xF8) | 0x05;	// Rx_Continuous mode
SX1276_write_reg(spi,LREG_OPMODE,&x,1);

//LORA_int_flag = 0;

//printf("LORA Exit\n");

//printf("leaving rcv\n");

}

static void lora_rcv_int(void)
{
unsigned char x;
unsigned char val[4];

/////////////
// CANT REALLY USE THIS IN AN INT AS SPI ACCESS TOO LONG AND INT WDT GOES OFF!
/////////////

// put radio into sleep mode while reading...
SX1276_read_reg(spi,LREG_OPMODE,&x,1);
//x = (x & 0xF8) | 0x07;	// channel activity detect mode
x = (x & 0xF8) | 0x00;	// sleep mode
SX1276_write_reg(spi,LREG_OPMODE,&x,1);

// get int reg
SX1276_read_reg(spi,LREG_IRQ_FLAGS,&x,1);
//##printf("LORA int reg: %02X\n",x);

//x = 0xFF;	// need to write a '1' to clear an int bit...
SX1276_write_reg(spi,LREG_IRQ_FLAGS,&x,1);

// after important stuff, sound the bell...
//##printf("%c",0x07);	// BELL char

if (x & 0x15)	// hdr \ CAD flags
{
unsigned char x;
/*
SX1276_read_reg(spi,LREG_OPMODE,&x,1);
x = (x & 0xF8) | 0x01;	// STDBY mode
SX1276_write_reg(spi,LREG_OPMODE,&x,1);

SX1276_read_reg(spi,LREG_OPMODE,&x,1);
//x = (x & 0xF8) | 0x07;	// channel activity detect mode
x = (x & 0xF8) | 0x05;	// Rx_Continuous mode
SX1276_write_reg(spi,LREG_OPMODE,&x,1);
*/
}

if (x & 0x50)	// Rx Done \ Valid Hdr flags
{
unsigned char i,n;
unsigned char rxdata[50];

if (x == 0x50)
	{
	unsigned char n;
	uint8_t *lora_rx_msg = NULL;

	
	SX1276_read_reg(spi,LREG_RX_NUM_BYTES,&n,1);

	lora_rx_msg = (uint8_t *)malloc(n);
	
	if (lora_rx_msg != NULL)
		{
		if (debug_do(DBG_ALLOC_MEM))
			printf("LoRa rcv int data alloc %d\n",n);

// set up FIFO address for Rx data
		SX1276_read_reg(spi,LREG_FIFO_RX_CURR_ADDR,&x,1);
		SX1276_write_reg(spi,LREG_FIFO_ADDR_PTR,&x,1);

//##	printf("Rx data[%d]:\n",n);
		SX1276_read_reg(spi,LREG_FIFO,val,4);
		for (i=0;i<4;i++)
			{
			lora_rx_msg[i] = val[i];
//##		printf(" %02X",val[i]);
			}

		SX1276_read_reg(spi,LREG_FIFO,val,4);
		for (i=4;i<8;i++)
			{
			lora_rx_msg[i] = val[i-4];
//##		printf(" %02X",val[i-4]);
			}

		for (i=8;i<n;i++)
			{
			SX1276_read_reg(spi,LREG_FIFO,&x,1);
			lora_rx_msg[i] = x;
//##		printf(" %02X",x);
			}
//##	printf("\n\n");
	
// put msg onto outgoing msg queue...
//	lora_rx_data_wr->q_data = lora_rx_msg;
//	lora_rx_data_wr->q_data_len = n;

#ifdef LORAWAN_SCAN
// add this MAC addr to the list of MAC addresses
		if (lorawan_MAC_entries < LORAWAN_LIST_LEN)
			{
			unsigned char found_flag = 0;
			unsigned char dev_addr[5];
// check to see if it is on the MAC address list...
			for (i=0;i<4;i++)
				{
//				dev_addr[i] = lora_rx_data_wr->q_data[1+i];
				dev_addr[i] = lora_rx_msg[1+i];
				}
			dev_addr[i] = 0x00;
// check first that the MAC address isnt already on the list?					
			for (i=0;i<lorawan_MAC_entries;i++)
				{
				if (!memcmp(dev_addr,&lorawan_MAC_list[i],4))
					found_flag = 1;
				}
							
			if (!found_flag)						// if no match found on the list
				{
// do we need to check first that the MAC address isnt already on the list?					
				memcpy(&lorawan_MAC_list[lorawan_MAC_entries],&lora_rx_msg[1],4);	// LoRaWAN DecAddr is 4 bytes long...
				lorawan_MAC_entries++;
				}
			}

#endif
	
// put msg onto outgoing msg queue...
		lora_rx_data_wr->q_data = lora_rx_msg;
		lora_rx_data_wr->q_data_len = n;

/////////////////////////////////////////////////////////////
// whitelist \ blacklist now done before the data is enqueued...
		{
		unsigned char send_lora_data_flag = 1;
		char dev_addr[5];
						
//##						printf("dev_addr: ");
//						memcpy(dev_addr,&lora_rx_data_rd->q_data[4],4);
		for (x=0;x<4;x++)
			{
			dev_addr[x] = lora_rx_msg[3 + 1 - x];
//##							printf("%02X:",dev_addr[x]);
			}
//##						printf("\n");
						
		dev_addr[4] = 0;

// send data if whitelisting not enabled, or there is a data match...				
							
//						if (bt_out_data->q_data[bt_out_data->q_data_len-1] == 0x0A)
									
		if (enable_lorawan_blacklist_flag)
			{
//##							printf("LoRa BL:");
//							if (listmatch(LORAWAN_CMD_LIST_LENGTH, LORAWAN_CMD_LIST_ENTRY_LENGTH, (char(*)[LORAWAN_CMD_LIST_ENTRY_LENGTH])lorawan_cmd_blacklist, (char*)lora_rx_data_rd->q_data,1))
			if (listmatch(LORAWAN_CMD_LIST_LENGTH, LORAWAN_CMD_LIST_ENTRY_LENGTH, (char *)lorawan_cmd_blacklist, dev_addr,1,1))	//(char*)lora_rx_data_rd->q_data, 1, 1))
				{
				send_lora_data_flag = 0;									
				}
			}

		if ((enable_lorawan_whitelist_flag) && (send_lora_data_flag))		// if whitelist enabled and the data hasn't been blacklisted already...
			{
			send_lora_data_flag = 0;
//##							printf("LoRa WL:");
//							if (listmatch(LORAWAN_CMD_LIST_LENGTH, LORAWAN_CMD_LIST_ENTRY_LENGTH, (char(*)[LORAWAN_CMD_LIST_ENTRY_LENGTH])lorawan_cmd_whitelist, (char*)lora_rx_data_rd->q_data,1))
//			if (listmatch(LORAWAN_CMD_LIST_LENGTH, LORAWAN_CMD_LIST_ENTRY_LENGTH, (char *)lorawan_cmd_whitelist, dev_addr,1,1))	//(char*)lora_rx_data_rd->q_data,1 1, ))
			if (listmatch(num_lora_bl_entries, LORAWAN_CMD_LIST_ENTRY_LENGTH, (char *)lorawan_cmd_whitelist, dev_addr,1,1))	//(char*)lora_rx_data_rd->q_data,1 1, ))
				{
				send_lora_data_flag = 1;
				}
			}							




		if (send_lora_data_flag)
////////////////////////////////////////////////////////////
			{
#if 0
			printf("\nCheck:\n");
			for (i=0;i<12;i++)
				{	
				printf("%02X %02X|",lora_rx_msg[i],lora_rx_data_wr->q_data[i]); 
				}
			printf("\n\n");
#endif
//xQueueSendFromISR		
//	if (xQueueSend(lora_data_queue, lora_rx_data_wr, ( TickType_t ) 0) != pdTRUE) 
// needs to be checked against queus_enable_flag!
//ERROR!
			if (xQueueSendFromISR(lora_data_queue, lora_rx_data_wr, NULL) != pdTRUE) 
				{
				ESP_LOGD(TAG, "Failed to enqueue LORA msg output. Queue full.");
//##		printf("LORA msg queue full.\r\n");
// free memory taken by lora_rx_msg packet, as not used due to error
				free(lora_rx_msg);
				}
			else
				{
				lora_msg_ready_count++;				// flag to fg to send msg data

//		if(debug_do(DBG_BLUETOOTH))
//##			{
//##			printf("No of queued LORA rx_msgs: %d\t\t",lora_msg_ready_count);
//##			printf("Queue free size: %d\n", uxQueueSpacesAvailable(lora_data_queue));
//##			printf("%c",0x07);	// BELL char
//##			}
				}
//	printf("end of LORA enqueue\n");
			}
		}
		}
//##	else
//##		printf("LORA malloc error!\n");
	}
//##else
//##	printf("Rx ERR: FLAGS=%02X\n",x);

}

// set back to Rx mode
SX1276_read_reg(spi,LREG_OPMODE,&x,1);
//x = (x & 0xF8) | 0x07;	// channel activity detect mode
x = (x & 0xF8) | 0x05;	// Rx_Continuous mode
SX1276_write_reg(spi,LREG_OPMODE,&x,1);

//LORA_int_flag = 0;

//printf("LORA Exit\n");

}

#endif

#if defined(USE_LORA) || defined(USE_LORA_LINK)
void print_lora_msg(char *name, unsigned char *data,unsigned char len)
{
unsigned char i;
signed int tx_power;
unsigned char tx_power_dp;

printf("%s LoRa msg [%d]: ",name,len);

if (!strcmp(name,"Rx"))
	printf("  [RSSI: %ddBm]",lora_pkt_rssi);
else if (!strcmp(name,"Tx"))
	{
	tx_power = SX1276_get_tx_power(spi);
	tx_power_dp = abs(tx_power%10);
	printf("  [Tx Power: %d.%ddBm]",tx_power/10, tx_power_dp);
	}	
printf("\n");
		
printf("HEADER 	ORIG_ID	 RELAY_ID  EXPIRE  MSG_TYPE  MSG\n");
printf(" [%02X}    [%02X}      [%02X}     [%02X}     [%02X}  ",data[0],data[1],data[2],data[3],data[4]);
for (i=5;i<len;i++)
	{
	printf("  %02X",data[i]);
	}

printf("\n");
}
#endif



void mqtt_send(unsigned char mqtt_transport_mode, unsigned char next_login_state)
{
#ifdef USE_WIFI_MQTT
if(mqtt_transport_mode == MQTT_MODE_WIFI)
// send via wifi
	{
	if (mqtt_payload_length)
// only trigger MQTT stack if there is data to send...
		{
		printf("Sending via Wifi MQTT...\n");
//				esp_mqtt_client_publish(client, "/topic/test3", "Hello World", 0, 0, 0);	
//				esp_mqtt_client_enqueue(client, "/topic/test3", "Hello World", 0, 0, 0, 0);	// non-blocking version of above
//	esp_mqtt_client_enqueue(mqtt_client, mqtt_topic_str, mqtt_payload_str, 0, 0, 0, 0);	// non-blocking version of above

		esp_mqtt_client_publish(mqtt_client, mqtt_topic_str, mqtt_payload_str, 0, 0, 0);	// blocking version
	
//		printf("Wifi MQTT send done.\n");
		}

	mqtt_login_state = next_login_state;	// usually MQTT_SEND_WAIT;		// go even if no data ready from gateway
	}
#endif

#ifdef USE_M4G_MQTT
if (mqtt_transport_mode == MQTT_MODE_4G)
// send via 4G	
	{
// only trigger MQTT stack if there is data to send...
	if (mqtt_payload_length)
		{
		mqtt_state = MQTT_SET_TOPIC;
		printf("Sending via 4G MQTT...\n");
		}

					
	mqtt_login_state = next_login_state;	// usually MQTT_SEND_WAIT;		// go even if no data ready from gateway
	}
#endif
	
// show topic and payload to user
	if (mqtt_payload_length)
		{
		printf("Sending to topic: %s\n",mqtt_topic_str);
		if (mqtt_payload_length < 255)
			printf("Payload: [%d]\n%s\r\n",mqtt_payload_length,mqtt_payload_str);
		else
			{
			char pstr[256];
						
			strncpy(pstr,mqtt_payload_str,255);
			pstr[255] = 0x00;
			printf("Payload: [%d]\n%s\r\n",mqtt_payload_length,pstr);						
			}

#ifdef USE_WIFI_MQTT
		if(mqtt_transport_mode == MQTT_MODE_WIFI)
			printf("Wifi MQTT send done.\n");
#endif
		}

		
}

void udp_send(unsigned char udp_transport_mode, unsigned char next_login_state)
{
int err;

#ifdef USE_WIFI_UDP
if(udp_transport_mode == UDP_MODE_WIFI)
// send via wifi
	{
	if (udp_payload_length)
// only trigger MQTT stack if there is data to send...
		{
		printf("Sending via Wifi UDP to: %s:%d\n",inet_ntoa(wifi_udp_ip_addr),wifi_udp_port); //
        err = sendto(wifi_udp_sock, udp_payload_str, udp_payload_length, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) 
			{
               ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
			}
		else				
			ESP_LOGI(TAG, "udp Message sent to %s:%d\n",inet_ntoa(wifi_udp_ip_addr),wifi_udp_port);
		
		
		}
	}
#endif

#ifdef USE_M4G_UDP
if (udp_transport_mode == UDP_MODE_4G)
// send via 4G	
	{
// only trigger MQTT stack if there is data to send...
	if (udp_payload_length)
		{
//		mqtt_state = MQTT_SET_TOPIC;
		printf("Sending via 4G UDP to: %s:%d\n",inet_ntoa(wifi_udp_ip_addr),wifi_udp_port); //

		}

					
//	udp_login_state = next_login_state;	// usually MQTT_SEND_WAIT;		// go even if no data ready from gateway
	}
#endif

#if defined(USE_M4G_UDP) || defined(USE_WIFI_UDP)
// show topic and payload to user
	if (udp_payload_length)
		{
		if (udp_payload_length < 255)
			printf("Payload: [%d]\n%s\r\n",udp_payload_length,udp_payload_str);
		else
			{
			char pstr[256];
						
			strncpy(pstr,udp_payload_str,255);
			pstr[255] = 0x00;
			printf("Payload: [%d]\n%s\r\n",udp_payload_length,pstr);						
			}

#ifdef USE_WIFI_UDP
		if(mqtt_transport_mode == MQTT_MODE_WIFI)
			printf("Wifi UDP send done.\n");
#endif
		}
#endif


}
