////////////////////////////////////////////
//
// MAC_Rpt_four_g_mqtt.cpp
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

// new code for separate SIMCOM, MQTT and UDP modes

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_spi_flash.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "sdkconfig.h"
#include "esp_wifi_types.h"

#include "esp_partition.h"

#include "MAC_Rpt.h"
#include "MAC_Rpt_Rtns.h"
#include "MAC_Rpt_four_g.h"
//#include "MAC_Rpt_four_g_mqtt_defs.h"
#include "MAC_Rpt_four_g_simcom.h"
#include "MAC_Rpt_four_g_mqtt.h"

#include "ca_cert.h"
#include "client_cert.h"
#include "client_key.h"

extern uint8_t four_g_tx_data[FOUR_G_TX_BUFSIZE];
extern uint8_t four_g_rx_data[FOUR_G_RX_BUFSIZE];
extern unsigned int four_g_tx_length;
extern unsigned int four_g_rx_length;
extern unsigned char mqtt_response_flag, mqtt_eol_response_flag;

extern unsigned char four_g_rst_flag, four_g_pwr_flag, four_g_connected_flag;

extern unsigned char mqtt_state, prev_mqtt_state;
//extern unsigned char mqtt_state_str[][];
//extern unsigned int mqtt_state_mc_timer;
//extern unsigned char mqtt_sm_timer_flag;
extern unsigned int four_g_timeout_timer;
extern unsigned int four_g_timeout_val;

extern unsigned int mqtt_timeout_timer;
extern unsigned int mqtt_timeout_val;

extern char mac_bin_str[];
extern char fwver_str[];
extern char imei_str[];
extern char simnum_str[];
extern char ipaddr_str[];

extern const char mqtt_state_str[MQTT_NUM_STATES][32];

extern unsigned int supply_voltage;
extern uint32_t voltage;
extern unsigned char temperature;

extern char gps_lat_str[];
extern char gps_lon_str[];
extern unsigned int gps_spd,lora_radio_freq;
extern unsigned char cell_csq;

unsigned char mqtt_attempts, prev_mqtt_attempts;
unsigned char mqtt_error_attempts;

extern char module_mac_str[];

char mqtt_will_topic_str[80];
char mqtt_will_msg_str[40];
char mqtt_topic_str[80];
char mqtt_prev_topic_str[80];
unsigned int mqtt_topic_length;
char mqtt_payload_str[MQTT_PAYLOAD_SIZE];		//256...
unsigned int mqtt_payload_length;
char mqtt_subscribe_topic_str[80];
unsigned int mqtt_subcribe_topic_length;

extern unsigned char mqtt_login_state;
//char gps_str[80];

extern signed char cell_rssi;

extern QueueHandle_t mqtt_cmd_queue;

extern unsigned char x100msec, secs, mins, hrs;

extern unsigned char ssl_enable_flag;

extern unsigned char server_ssl_mode[4];

extern unsigned char server0_addr_ptr;


extern char pattern_detect_char;

extern unsigned char eolstr[10];

unsigned char mqtt_last_error_state;

int64_t init_time = 0;
int64_t start_time = 0;
int64_t end_time = 0;
int64_t final_time = 0;

extern int64_t pcmqtt_time;

extern unsigned char server0_addr_ptr, server1_addr_ptr;

extern unsigned int dbg_count;

extern unsigned char simcom_busy_flag;

extern unsigned char mqtt_transport_mode;
extern 
unsigned char queues_enabled_flag;


//#define USE_CERT_PARTITION
#ifdef USE_CERT_PARTITION
esp_partition_iterator_t pt;
const esp_partition_t *cert_space_pt;
char pbuf[20];
unsigned int cert_name_pos, server_cert_pos, client_cert_pos, client_key_pos;
unsigned int cert_name_len, server_cert_len, client_cert_len, client_key_len;
#endif

//extern esp_timer_handle_t oneshot_timer;

/*
const char mqtt_state_str[MQTT_NUM_STATES][32] = 
{
//23456789012345678901
"POWER_OFF",
"POWER_EN",
"POWER_EN_WAIT",
"RESET",
"RESET_WAIT",
"POWER_ON_KEY",
"POWER_ON_KEY_WAIT",
"POWER_ON_KEY_RESP_WAIT",

"NO_COMM_IDLE",

"GET_SIM_IMEI",
"GET_SIM_NUM",
"CLOSEALL",
"CHECK_SIG_QUALITY",
"CHECK_4G_NETWORK",
"CHECK_GPRS_NETWORK",
"SET_PDP",

"SET_SSL_CONFIG",
"SET_SSL_AUTHMODE",
"SET_SSL_SERVER_CERTIFICATE",
"SET_SSL_CLIENT_CERTIFICATE",
"SET_SSL_CLIENT_KEY",

"MQTT_COMM_START",

"MQTT_SET_WILLTOPIC",
"MQTT_SET_WILLMSG",
"MQTT_SET_UTF8_MODE",
"MQTT_CONNECT",
"MQTT_SUBSCRIBE_TOPIC",
"MQTT_SET_TOPIC",
"MQTT_SET_PAYLOAD",
"MQTT_PUBLISH_MSG",
"MQTT_SET_SUBSCRIBE_TOPIC",
"MQTT_UNSUBSCRIBE_TOPIC",
"MQTT_SERVER_DISCONNECT",
"MQTT_STOP",

"MQTT_COMM_IDLE",

"POWER_DOWN_KEY",
"POWER_DOWN_KEY_WAIT",
"POWER_DOWN_RESP_WAIT",
"POWER_DISABLE",
"END"
};
*/

size_t heap_free, prev_heap_free;

// function prototypes
void mqtt_reset_flags(unsigned int timeout, unsigned char clear_mqtt_attempts);

//static void oneshot_timer_callback(void* arg);

// code
void check_heap(char* str, unsigned char show_flag)
{
	heap_free = heap_caps_get_free_size(MALLOC_CAP_8BIT);
	if ((prev_heap_free != heap_free) || (show_flag))
		{
		printf("@%s: Heap free: %d\n",str,heap_free);
		}
	prev_heap_free = heap_free;
}

void four_g_mqtt_state_machine(unsigned char *mqtt_state,unsigned char *prev_mqtt_state, unsigned char *simcom_state)
{
int i;
unsigned char j,k;
unsigned int n;
char c;
char * p;
char tmpstr[20];


char eolstr[5];
//char mqtt_topic_str[80];
//char mqtt_payload_str[80];
//unsigned int mqtt_payload_length;

int64_t ltmp = 0;

unsigned char dbg_4g_flag;

// reset WatchDog Timer...
	esp_task_wdt_reset();		

if(debug_do(DBG_4G_TXRX))
	dbg_4g_flag = 1;
else
	dbg_4g_flag = 0;


// set MQTT network LED...
// check here so that state can still be overridden by the following code (eg, to flash for data Tx \ Rx)
// if SIMCOM off...
//	if (*simcom_state < SIMCOM_READY_IDLE)
#ifdef NETWORK_LED
//		set_led(NETWORK_LED, DEVICE_OFF);
#else
//		set_led(1, DEVICE_OFF);
#endif
// SIMCOM OK
	if ((*mqtt_state > MQTT_NO_COMM_IDLE) && (*mqtt_state <= MQTT_CHECK_4G_NETWORK))
#ifdef NETWORK_LED
		set_led(NETWORK_LED, FLASH);
#else
		set_led(1, FLASH);
#endif
// network connected
	else if ((*mqtt_state > MQTT_CHECK_4G_NETWORK) && (*mqtt_state <= MQTT_CONNECT))
#ifdef NETWORK_LED
		set_led(NETWORK_LED, FAST_FLASH);
#else
		set_led(1, FAST_FLASH);
#endif
	else
// network \ MQTT connected
#ifdef NETWORK_LED
		set_led(NETWORK_LED, DEVICE_ON);
#else
		set_led(1, DEVICE_ON);
#endif


#ifdef DBG_TEST
if (*mqtt_state != *prev_mqtt_state)
#else
if ((debug_do(DBG_MQTT)) && (*mqtt_state != *prev_mqtt_state))
#endif
	{
	printf("\r\nMQTT State: [%03d] MQTT_%s\r\n",*mqtt_state, mqtt_state_str[*mqtt_state]);
	}

*prev_mqtt_state = *mqtt_state;
eolstr[0] = 0x0A;
eolstr[1] = 0x00;

heap_free = heap_caps_get_free_size(MALLOC_CAP_8BIT);
if (debug_do(DBG_MQTT))
	{
	if (prev_heap_free != heap_free)
		{
		printf("MQTT: Heap free: %d\n",heap_free);
		}
	}
prev_heap_free = heap_free;
	
// system rests in MQTT_POWER_OFF state
// manual change to MQTT_POWER_EN starts 4G power up 
// system rests in MQTT_NO_COMM_IDLE state
// manual change to MQTT_COMM_INIT starts 4G connect
// system rests in MQTT_CONNECT_IDLE state
switch(*mqtt_state)
	{
	case MQTT_NO_COMM_IDLE:
//		printf("Reached NO_COMM_IDLE\r\n");
		mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
		mqtt_error_attempts = 0;

		break;


	case MQTT_COMM_INIT:
		if(debug_do(DBG_4G_TXRX))
			printf("Comms Init...\r\n");

// Set default uart pattern detect function.
		pattern_detect_char = set_uart_pattern_detect(eolstr[0]);
	
		*mqtt_state = MQTT_CLOSEALL;
//		*mqtt_state = MQTT_SET_SSL_CONFIG;	// FOR TEST!
		mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
//		mqtt_attempts = 0;
		break;
		

	case MQTT_CLOSEALL:
//		AT+CIPSHUT		// respond with SHUT OK
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				printf("RESP_NONE\n");
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
				four_g_tx_length = four_g_pack_data("AT+NETCLOSE\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag != RESP_WAIT)	
				{															
				printf("RESP NOT WAIT\n");
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+NETCLOSE", "", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*mqtt_state = MQTT_CHECK_SIG_QUALITY;	//MQTT_SET_PDP;
					mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
					}
				else
					{
					printf("RESP ELSE\n");
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
				}
				
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				printf("RESP_WAIT\n");
				}
			else
				{
				printf("RESP ELSE\n");
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;

///////////////////////////

	case MQTT_CHECK_SIG_QUALITY:
//		AT+CGDCONT=1, "IP", "AQL"	// respond with OK
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
				four_g_tx_length = four_g_pack_data("AT+CSQ\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CSQ", "OK", "", eolstr, 1, 6, 0x0D, tmpstr, 15);
				if (i)
					{
					i = 0;
					j = 0;
					while((j==0) && (tmpstr[i] != 0x00))
						{
						if (tmpstr[i] == ',')
							{
							tmpstr[i] = 0x00;
							j = 1;
							k = atoi(tmpstr);
							}
						i++;
						}
						
//					*mqtt_state = MQTT_COMMS_INIT_ERROR_END;

					if (j)
						{
						if (((k==99) || (k==199)))
							{
							cell_rssi = -127;
							if (debug_do(DBG_MQTT))
								printf("No network signal detected!\r\n");
//							*mqtt_state = MQTT_ERROR_END;
							mqtt_attempts++;
							mqtt_response_flag = RESP_NONE;
							}
						else
							{
							if (k<32)
								{
								cell_rssi = -113 + (2*k);	// -113 to -53dBm
								}
							else if(k>190)
								{
								cell_rssi = -25;
								}
							else
								{
								cell_rssi = 216 - k;
								}
								
							if (debug_do(DBG_MQTT))
								printf("Network signal strength = %ddBm\r\n",cell_rssi);
							
							*mqtt_state = MQTT_CHECK_OPS_1;	//MQTT_CHECK_4G_NETWORK;	//MQTT_NET_OPEN;	//MQTT_SET_APN;
							mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
							simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
							}
						}
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}			
			}
		
		break;

///
	case MQTT_CHECK_OPS_1:
//		AT_COPS=1,2,"23429"			// respond with OK
//		four_g_tx_length = four_g_pack_data("AT_COPS=1,2,\"23429\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_VLONG_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
				four_g_tx_length = four_g_pack_data("AT+COPS=?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("", "OK", "", eolstr, 0,0,0,NULL,0);	//1, 8, 0x0D, tmpstr, 15);
				if (i)
					{
					*mqtt_state =  MQTT_CHECK_4G_NETWORK;	
					mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}				
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;

///

	case MQTT_CHECK_4G_NETWORK:
//		AT+CSTT="m2m.aql.net"	// respond with OK	AT+CSOCKSETPN=1
//		four_g_tx_length = four_g_pack_data("AT+CSTT=\"m2m.aql.net\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
				four_g_tx_length = four_g_pack_data("AT+CREG?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CREG", "OK", "", eolstr, 1, 9, 0x0D, tmpstr, 15);
				if (i)
					{
					n = atoi(tmpstr);
//					printf("CREG: <%s> %d\n",tmpstr,n);
						printf("NETWORK STATUS = %d\t - ",n);

					switch(n)
						{
						case 0:
							printf("Not registered on network");
							break;
						case 1:
							printf("Registered on home network");
							break;
						case 2:
							printf("Not registered on network,searching for new network");
							break;
						case 3:
							printf("Registration denied");
							break;
						case 4:
							printf("Unknown ststus");
							break;
						case 5:
							printf("Registered \ in roaming mode");
							break;
						default:
							break;
						}
					
					if (cell_rssi != -127)
						printf("  [%ddBm]",cell_rssi);
					printf("\n");
					
					if (n != 1)
						{
						mqtt_response_flag = RESP_NONE;
//						if (n >= 3)
							mqtt_attempts++;
						}
					else
						{
						*mqtt_state = MQTT_CHECK_GPRS_NETWORK;
						mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
//				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
						simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
						}
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			
			}
		break;

	case MQTT_CHECK_GPRS_NETWORK:
//		AT_COPS=1,2,"23429"			// respond with OK
//		four_g_tx_length = four_g_pack_data("AT_COPS=1,2,\"23429\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
				four_g_tx_length = four_g_pack_data("AT+CGREG?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CGREG", "OK", "", eolstr, 1, 8, 0x0D, tmpstr, 15);
				if (i)
					{
//					*mqtt_state =  MQTT_SET_PDP;	//MQTT_CHECKSTOP;	//MQTT_SET_PDP;	//MQTT_CHECK_CONN;
					*mqtt_state =  MQTT_CHECK_OPS;	//MQTT_CHECKSTOP;	//MQTT_SET_PDP;	//MQTT_CHECK_CONN;
					mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}				
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;

	case MQTT_CHECK_OPS:
//		AT_COPS=1,2,"23429"			// respond with OK
//		four_g_tx_length = four_g_pack_data("AT_COPS=1,2,\"23429\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
				four_g_tx_length = four_g_pack_data("AT+COPS?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+COPS", "OK", "", eolstr, 0,0,0,NULL,0);	//1, 8, 0x0D, tmpstr, 15);
				if (i)
					{
#define USE_RELASE_AND_STOP_STATES
#ifdef USE_RELASE_AND_STOP_STATES
					*mqtt_state =  MQTT_CHECK_RELEASE;	//MQTT_SET_PDP;	//MQTT_CHECKSTOP;	//MQTT_SET_PDP;	//MQTT_CHECK_CONN;
#else
					*mqtt_state =  MQTT_SET_PDP;	
#endif
					mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}				
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;

#ifdef USE_RELASE_AND_STOP_STATES
	case MQTT_CHECK_RELEASE:
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
				four_g_tx_length = four_g_pack_data("AT+CMQTTREL=0\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag == RESP_OK)	
				{																				
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

//				i = get_AT_value("AT+CMQTTREL", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				i = get_AT_value("AT+CMQTTREL", "", "", eolstr, 0, 0, 0, NULL, 0);	// client rel error can indicate that wasnt acquired in the first place!
				if (i)
					{
					*mqtt_state = MQTT_CHECK_STOP;
					mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
				}
#if 1
			else if (mqtt_response_flag == RESP_ERROR)
				{
// client rel error can indicate that wasn't acquired in the first place!
				*mqtt_state = MQTT_CHECK_STOP;
				mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
				simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
				}
#endif
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;
		
	case MQTT_CHECK_STOP:
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
				four_g_tx_length = four_g_pack_data("AT+CMQTTSTOP\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if ((mqtt_response_flag == RESP_OK) || (mqtt_response_flag == RESP_ERROR))	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTSTOP", "+CMQTTSTOP", "", eolstr, 1, 13, 0x0D, tmpstr, 15);
//<<<<<<< .mine
				printf("CHKSTOP: %d [%s]\n",atoi(tmpstr),tmpstr);
//||||||| .r93
//				printf("STOP: %d 9%s]\n",atoi(tmpstr),tmpstr);
//=======
//				printf("STOP: %d %s]\n",atoi(tmpstr),tmpstr);
//>>>>>>> .r101
				if (i)
					{
					*mqtt_state = MQTT_SET_PDP;	//MQTT_CHECK_CONN;
					mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}				
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;
#endif
		
	case MQTT_SET_PDP:
//		AT+CGDCONT=1, "IP", "AQL"	// respond with OK
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
#if 1
				four_g_tx_length = four_g_pack_data("AT+CGSOCKCONT=1, \"IP\", \"AQL\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
#else
				four_g_tx_length = four_g_pack_data("AT+CGDCONT=1,\"IP\",\"AQL\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
#endif

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CGSOCKCONT", "OK", "", eolstr, 0, 0, 0, NULL, 0);
//				i = get_AT_value("AT+CGDCONT", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					printf("ssl_en: %d    ssl_mode: %d\n",ssl_enable_flag, server_ssl_mode[server0_addr_ptr]);						
#ifdef USE_M4G_MQTT_SSL
					if ((ssl_enable_flag) && (server_ssl_mode[server0_addr_ptr]))
						*mqtt_state = MQTT_SET_SSL_CONFIG;
					else
#endif
						*mqtt_state = MQTT_COMM_START;	//MQTT_ACT_PDP;	//MQTT_COMM_START;	//MQTT_NET_OPEN;	//MQTT_SET_APN;
					
					mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}				
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;

	case MQTT_ACT_PDP:
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_LONG_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
				four_g_tx_length = four_g_pack_data("AT+CGACT=1,1\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CGACT", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					printf("ssl_en: %d    ssl_mode: %d\n",ssl_enable_flag, server_ssl_mode[server0_addr_ptr]);
#ifdef USE_M4G_MQTT_SSL
//					if (ssl_enable_flag)
					if ((ssl_enable_flag) && (server_ssl_mode[server0_addr_ptr]))
						*mqtt_state = MQTT_SET_SSL_CONFIG;
					else
#endif
						*mqtt_state = MQTT_COMM_START;
						
					mqtt_error_attempts = 0;							// have passed through this phase of state machine successfully

//###endif
					mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}				
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				printf("RX ERR\n");
				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}			
			}
		break;

/// SSL states
	case MQTT_SET_SSL_CONFIG:
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
//		AT_COPS=0			// respond with +COPS: 0,2,"310410",7
				four_g_tx_length = four_g_pack_data("AT+CSSLCFG=\"sslversion\",0,4\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
//				four_g_tx_length = four_g_pack_data("AT+CSSLCFG=\"sslversion\",0,3\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CSSLCFG", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*mqtt_state = MQTT_SET_SSL_AUTHMODE;
					mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;
		
	case MQTT_SET_SSL_AUTHMODE:
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;

//		AT_CSQ				// respond with +CSQ: 14,99
// 0 = no auth
// 1 = server only
// 2 = full auth
				four_g_tx_length = four_g_pack_data("AT+CSSLCFG=\"authmode\",0,2\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length); 
				}
			else if (mqtt_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CSSLCFG", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
#ifdef USE_CERT_PARTITION
// setup variables to allow certs to be read from cert partition...
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
						}
					else
						{
						printf("Cert_space not found\n");
						}
						
#endif

					*mqtt_state = MQTT_LOAD_SSL_SERVER_CERTIFICATE;
					mqtt_reset_flags(COMM_CERT_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;

//#define DEBUG_CERTS
	case MQTT_LOAD_SSL_SERVER_CERTIFICATE:
//		if ((mqtt_state_mc_timer >= COMM_DELAY) && (mqtt_attempts < COMM_PHASE_RETRIES))
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
//			printf("Load SSL Server Cert: resp = [%d] RESP_%s\n",mqtt_response_flag,four_g_resps[mqtt_response_flag]);
			if (mqtt_response_flag == RESP_NONE)
				{
//#define DBG_HEAP_LOSS
#ifdef DBG_HEAP_LOSS
				check_heap("SSC start",1);
#endif
				mqtt_reset_flags(COMM_CERT_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
				sprintf((char*)four_g_tx_data,"AT+CCERTDOWN=\"server_cert.pem\",%d\r",ca_cert_size);
				four_g_tx_length = strlen((char *)four_g_tx_data);

#ifdef DBG_HEAP_LOSS
				check_heap("SSC pre patt",1);
#endif

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

#ifdef DBG_HEAP_LOSS
				check_heap("SSC post patt",1);
#endif
				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);

#ifdef DBG_HEAP_LOSS
				check_heap("SSC post send",1);
#endif
				}
			else if (mqtt_response_flag == RESP_R_ARROW)	//RESP_EOL)	
				{						
#ifdef DBG_HEAP_LOSS
				check_heap("SSC resp start",1);
#endif			
// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect(eolstr[0]);

#ifdef DBG_HEAP_LOSS
				check_heap("SSC pre rcv",1);
#endif
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

#ifdef DBG_HEAP_LOSS
				check_heap("SSC post rcv",1);
#endif
				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CCERTDOWN", ">", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
#ifdef USE_CERT_PARTITION
					for (n=0;n<server_cert_len;n++)
						{
						esp_partition_read(cert_space_pt,(size_t)server_cert_pos,pbuf,1);

//						sprintf((char *)four_g_tx_data,"%c",ca_cert_data[n]);
						uart_write_bytes(FOUR_G, (char *)&pbuf[0],1);	//four_g_tx_data, 1);
						}
						
					four_g_rx_length = server_cert_len+6;	// allow for 0x0D,0x0D, O, K, 0x0D,0x0A
				
#else
					for (n=0;n<ca_cert_size;n++)
						{
						sprintf((char *)four_g_tx_data,"%c",ca_cert_data[n]);
						uart_write_bytes(FOUR_G, (char *)four_g_tx_data, 1);
						}
						
					four_g_rx_length = ca_cert_size+6;	// allow for 0x0D,0x0D, O, K, 0x0D,0x0A
#endif
#ifdef DBG_HEAP_LOSS
					check_heap("SSC post AT",1);
#endif
// now recv the echoed data block and "OK" at end
#ifdef DEBUG_CERTS
					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,100);

					if(debug_do(DBG_4G_TXRX))
						printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);
#else
					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,(four_g_rx_length/10)+50);	//250);		//150); // maybe (data len \ 10) millisec @115k?

#ifdef DBG_HEAP_LOSS
					check_heap("SSC post rcv2",1);	
#endif

#endif
					i = get_AT_value("", "OK", "", eolstr, 0, 0, 0, NULL, 0);
					if (i)
						{
						*mqtt_state = MQTT_LOAD_SSL_CLIENT_CERTIFICATE;
						mqtt_reset_flags(COMM_CERT_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
						simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
						}
					else
						{
						mqtt_attempts++;
						mqtt_response_flag = RESP_NONE;
						}		
#ifdef DBG_HEAP_LOSS
				check_heap("SSC post AT2",1);
#endif
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
					
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;
		
	case MQTT_LOAD_SSL_CLIENT_CERTIFICATE:
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_CERT_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
				sprintf((char*)four_g_tx_data,"AT+CCERTDOWN=\"client_cert.pem\",%d\r",client_cert_size);
				four_g_tx_length = strlen((char *)four_g_tx_data);

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag == RESP_R_ARROW)	//RESP_EOL)	
				{															
// Set uart pattern detect function. 0);
				pattern_detect_char = set_uart_pattern_detect(eolstr[0]);

//				vTaskDelay(10/portTICK_PERIOD_MS);	// delay 500ms
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CCERTDOWN", ">", "", eolstr, 0, 0, 0, NULL, 0);				
				if (i)
					{
#ifdef USE_CERT_PARTITION
					for (n=0;n<client_cert_len;n++)
						{
						esp_partition_read(cert_space_pt,(size_t)client_cert_pos,pbuf,1);

//						sprintf((char *)four_g_tx_data,"%c",ca_cert_data[n]);
						uart_write_bytes(FOUR_G, (char *)&pbuf[0],1);	//four_g_tx_data, 1);
						}
						
					four_g_rx_length = client_cert_len+6;	// allow for 0x0D,0x0D, O, K, 0x0D,0x0A
				
#else
					for (n=0;n<client_cert_size;n++)
						{
						sprintf((char *)four_g_tx_data,"%c",client_cert_data[n]);
						uart_write_bytes(FOUR_G, (char *)four_g_tx_data, 1);
						}

					four_g_rx_length = client_cert_size+6;	// allow for 0x0D,0x0D, O, K, 0x0D,0x0A
#endif

#ifdef DEBUG_CERTS
					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,100);

					if(debug_do(DBG_4G_TXRX))
						printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);
#else
					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,0,(four_g_rx_length/10)+50);	//150);
#endif
					i = get_AT_value("", "OK", "", eolstr, 0, 0, 0, NULL, 0);
					if (i)
						{
						*mqtt_state = MQTT_LOAD_SSL_CLIENT_KEY;
						mqtt_reset_flags(COMM_CERT_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
						simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
						}
					else
						{
						mqtt_attempts++;
						mqtt_response_flag = RESP_NONE;
						}		
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
				
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;
		
	case MQTT_LOAD_SSL_CLIENT_KEY:
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_CERT_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
				sprintf((char*)four_g_tx_data,"AT+CCERTDOWN=\"client_key.pem\",%d\r",client_key_size);
				four_g_tx_length = strlen((char *)four_g_tx_data);
	
// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag == RESP_R_ARROW)	//RESP_EOL)	
				{															
// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect(eolstr[0]);

//				vTaskDelay(10/portTICK_PERIOD_MS);	// delay 500ms
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CCERTDOWN", ">", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
#ifdef USE_CERT_PARTITION
					for (n=0;n<client_key_len;n++)
						{
						esp_partition_read(cert_space_pt,(size_t)client_key_pos,pbuf,1);

//						sprintf((char *)four_g_tx_data,"%c",ca_cert_data[n]);
						uart_write_bytes(FOUR_G, (char *)&pbuf[0],1);	//four_g_tx_data, 1);
						}
						
					four_g_rx_length = client_key_len+6;	// allow for 0x0D,0x0D, O, K, 0x0D,0x0A
				
#else
					for (n=0;n<client_key_size;n++)
						{
						sprintf((char *)four_g_tx_data,"%c",client_key_data[n]);
						uart_write_bytes(FOUR_G, (char *)four_g_tx_data, 1);
						}

				four_g_rx_length = client_key_size+6;	// allow for 0x0D,0x0D, O, K, 0x0D,0x0A
#endif

#ifdef DEBUG_CERTS
					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,100);
					if(debug_do(DBG_4G_TXRX))
						printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);
#else
					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,0,(four_g_rx_length/10)+50);	//150);
#endif
					i = get_AT_value("", "OK", "", eolstr, 0, 0, 0, NULL, 0);
					if (i)
						{
						*mqtt_state = MQTT_SHOW_SSL_CERTIFICATES;
						mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
						simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
						}
					else
						{
						mqtt_attempts++;
						mqtt_response_flag = RESP_NONE;
						}		
					}					
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;
		
	case MQTT_SHOW_SSL_CERTIFICATES:
//		if ((mqtt_state_mc_timer >= COMM_DELAY) && (mqtt_attempts < COMM_PHASE_RETRIES))
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
#ifdef USE_CERT_PARTITION
				esp_partition_iterator_release(pt);
#endif
//		AT_CPSI?			// respond with 
//+CPSI: LTE CAT-M1,Online,310-410,0x4804,74777865,343,EUTRAN-BAND2,875,4,4,-13,-112,-82,14
				four_g_tx_length = four_g_pack_data("AT+CCERTLIST\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
//			uart_flush(FOUR_G);
				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);		
				}
//			else if (mqtt_response_flag == RESP_WAIT)	
			else if (mqtt_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				*mqtt_state = MQTT_SET_SSL_SERVER_CERTIFICATE;
				mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
				simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;
		
	case MQTT_SET_SSL_SERVER_CERTIFICATE:
//		if ((mqtt_state_mc_timer >= COMM_DELAY) && (mqtt_attempts < COMM_PHASE_RETRIES))
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;

//		AT_CPSI?			// respond with 
//+CPSI: LTE CAT-M1,Online,310-410,0x4804,74777865,343,EUTRAN-BAND2,875,4,4,-13,-112,-82,14
				four_g_tx_length = four_g_pack_data("AT+CSSLCFG=\"cacert\",0,\"server_cert.pem\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);
	
				i = get_AT_value("AT+CSSLCFG", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*mqtt_state = MQTT_SET_SSL_CLIENT_CERTIFICATE;
					mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;
		

	case MQTT_SET_SSL_CLIENT_CERTIFICATE:
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;

//		AT+CGATT?		// respond with +CGATT: 1 <cr-lf><cr-lf> OK
				four_g_tx_length = four_g_pack_data("AT+CSSLCFG=\"clientcert\",0,\"client_cert.pem\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CSSLCFG", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*mqtt_state = MQTT_SET_SSL_CLIENT_KEY;
					mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;
	case MQTT_SET_SSL_CLIENT_KEY:
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;

//		AT+CNACT=1,"m2m.aql.net"	//respond with OK <cr-lf><cr-lf> +APP PDP: ACTIVE
				four_g_tx_length = four_g_pack_data("AT+CSSLCFG=\"clientkey\",0,\"client_key.pem\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}

			else if (mqtt_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CSSLCFG", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
						
/*
// test - set SSL context...
					four_g_tx_length = four_g_pack_data("AT+CMQTTSSLCFG=0,0\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
					four_g_send_data(four_g_tx_data,four_g_tx_length);

					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

*/
					*mqtt_state = MQTT_COMM_START;
					mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
//					mqtt_error_attempts = 0;							// have passed through this phase of state machine successfully
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;
/// SSL end		

	case MQTT_COMM_START:
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
//				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				mqtt_reset_flags(50,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
				four_g_tx_length = four_g_pack_data("AT+CMQTTSTART\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
// response to CMQTTSTART is just "OK"... but +CMQTTSTART:0 should follow...
			else if ((mqtt_response_flag == RESP_OK) || (mqtt_response_flag == RESP_PCMQTT))	//RESP_OK)	
				{		
				printf("RSP OK\n");				
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTSTART", "+CMQTT", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*mqtt_state = MQTT_ACQUIRE_CLIENT;
					mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
					mqtt_error_attempts = 0;							// have passed through this phase of state machine successfully
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				printf("RSP ERR\n");				
				if(debug_do(DBG_4G_TXRX))
					{
					four_g_rx_length = 0;
					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);
					}

				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;
		
	case MQTT_ACQUIRE_CLIENT:
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;

//		AT+CIICR		// respond with OK

// !!! seems to be a 22 byte limit on the client ID?? !!!

#ifdef USE_M4G_MQTT_SSL
//				if (ssl_enable_flag)
				if ((ssl_enable_flag) && (server_ssl_mode[server0_addr_ptr]))
					sprintf((char *)four_g_tx_data,"AT+CMQTTACCQ=0,\"aql_gw_%s\",1\r",mac_bin_str);		// ,1 is for SSL
				else
#endif
					sprintf((char *)four_g_tx_data,"AT+CMQTTACCQ=0,\"aql_gw_%s\"\r",mac_bin_str);		// respond with number <cr-lf> OK
//			sprintf((char *)four_g_tx_data,"AT+CMQTTACCQ=0,\"aql gw 001122334455\"\r");		// respond with number <cr-lf> OK
//			four_g_tx_length = four_g_pack_data("AT+CMQTTACCQ=0,\"client test0\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
				four_g_tx_length = strlen((char *)four_g_tx_data);

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTACCQ", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*mqtt_state = MQTT_SET_WILLTOPIC;
					mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;

	case MQTT_SET_WILLTOPIC:
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;

				strcpy(mqtt_will_topic_str, "/gateway_dom/");
				strcat(mqtt_will_topic_str,mac_bin_str);

				sprintf((char *)four_g_tx_data,"AT+CMQTTWILLTOPIC=0,%d\r",strlen(mqtt_will_topic_str));		// respond with number <cr-lf> OK

//			four_g_tx_length = four_g_pack_data("AT+CMQTTWILLTOPIC=0,%d\r",(char *)four_g_tx_data,strlen(mqtt_will_topic_str));		// respond with number <cr-lf> OK
				four_g_tx_length = strlen((char *)four_g_tx_data);		// respond with number <cr-lf> OK

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag == RESP_R_ARROW)	
				{									
// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect(eolstr[0]);
				
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);
				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTWILLTOPIC", ">", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					xQueueReset(mqtt_cmd_queue);

//			four_g_tx_length = four_g_pack_data("POST /api/resource/aqlgateway/dom/1234 HTTP/1.1\r\n",(char *)four_g_tx_data);
//			strcpy((char *)four_g_tx_data,"POST /api/resource/aqlgateway/dom/");
//			strcat((char *)four_g_tx_data, "HTTP/1.1\r\n");
//			four_g_tx_length = strlen((char *)four_g_tx_data);


//				four_g_tx_length = four_g_pack_data("WILLTOPIC\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
					four_g_tx_length = strlen(mqtt_will_topic_str);
					four_g_send_data((uint8_t *)mqtt_will_topic_str,four_g_tx_length);

					vTaskDelay(100/portTICK_PERIOD_MS);	// delay 500ms

					*mqtt_state = MQTT_SET_WILLMSG;	//MQTT_SET_APN;
					mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;
		
	case MQTT_SET_WILLMSG:
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
				strcpy(mqtt_will_msg_str, "disconnect");

				sprintf((char *)four_g_tx_data,"AT+CMQTTWILLMSG=0,%d,1\r",strlen(mqtt_will_msg_str));		// respond with number <cr-lf> OK

//			four_g_tx_length = four_g_pack_data("AT+CMQTTWILLMSG=0,%d,1\r",(char *)four_g_tx_data,strlen(mqtt_will_msg_str));		// respond with number <cr-lf> OK
				four_g_tx_length = strlen((char *)four_g_tx_data);		// respond with number <cr-lf> OK

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag == RESP_R_ARROW)	
				{									
// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect(eolstr[0]);

				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTWILLMSG", ">", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
//				four_g_tx_length = four_g_pack_data("WILLMSG\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
					four_g_tx_length = strlen(mqtt_will_msg_str);
					four_g_send_data((uint8_t *)mqtt_will_msg_str,four_g_tx_length);

					vTaskDelay(100/portTICK_PERIOD_MS);	// delay 500ms

					*mqtt_state = MQTT_SET_UTF8_MODE;	//MQTT_SET_APN;
					mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
					}
				else
					{					
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;
		
	case MQTT_SET_UTF8_MODE:
//		AT+CIPOPEN="TCP","iot-visualiser.aql.com","80"
// respond with OK <cr-lf><cr-lf> CONNECT OK <cr-lf><cr-lf>
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
				four_g_tx_length = four_g_pack_data("AT+CMQTTCFG=\"checkUTF8\",0,0\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
//				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,30);
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,1,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTCFG", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*mqtt_state = MQTT_CONNECT;
					mqtt_reset_flags(COMM_CONN_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
//					mqtt_error_attempts = 0;							// have passed through this phase of state machine successfully
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;
		
	case MQTT_CONNECT:
//		AT+CIPSEND	// start input; terminate with 0x1A;	responds with SEND OK
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				char tmp_connect_str[100];
				
				start_time = esp_timer_get_time();
				pcmqtt_time = 0;
				
				mqtt_reset_flags(COMM_CONN_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;

//		four_g_tx_length = four_g_pack_data("AT+CMQTTCONNECT=0,\"tcp://test.mosquitto.org:1883\",60,1\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
#ifdef AQL_SERVER
	#ifdef USE_M4G_MQTT_SSL
#if 1
// assemble connect string from selected address
				strcpy(tmp_connect_str,"AT+CMQTTCONNECT=0,\"");

				switch(server0_addr_ptr)
					{
		#ifdef SERVER_ADDRESS_AND_PORT_0
					case 0:
						strcat(tmp_connect_str,SERVER_ADDRESS_AND_PORT_0);
						break;
		#endif				
		#ifdef SERVER_ADDRESS_AND_PORT_1
					case 1:
						strcat(tmp_connect_str,SERVER_ADDRESS_AND_PORT_1);
						break;
		#endif				
		#ifdef SERVER_ADDRESS_AND_PORT_2
					case 2:
						strcat(tmp_connect_str,SERVER_ADDRESS_AND_PORT_2);
						break;
		#endif				
		#ifdef SERVER_ADDRESS_AND_PORT_3
					case 3:
						strcat(tmp_connect_str,SERVER_ADDRESS_AND_PORT_3);
						break;
		#endif	
					default:
						strcat(tmp_connect_str,DFLT_SERVER_ADDRESS_AND_PORT);
					}

				strcat(tmp_connect_str,"\",60,1\r");
				four_g_tx_length = four_g_pack_data(tmp_connect_str,(char *)four_g_tx_data);		// no username, passwd when using SSL...
#else
//				if (ssl_enable_flag)
				if ((ssl_enable_flag) && (server_ssl_mode[server0_addr_ptr]))
					four_g_tx_length = four_g_pack_data("AT+CMQTTCONNECT=0,\"tcp://iot-visualiser.aql.com:8883\",60,1\r",(char *)four_g_tx_data);		// no username, passwd when using SSL...
				else
//			four_g_tx_length = four_g_pack_data("AT+CMQTTCONNECT=0,\"tcp://iot-visualiser.aql.com:8883\",60,1,\"board_test\",\"MyPassWd\"\r",(char *)four_g_tx_data);		// no username, passwd when using SSL...
//			four_g_tx_length = four_g_pack_data("AT+CMQTTCONNECT=0,\"tcp://iot-visualiser.aql.com:8883\",60,1,\"server_admin\",\"K6kZDc5QjaYW2TF\"\r",(char *)four_g_tx_data);		// try username, passwd when using SSL...
#endif
	#else
//			four_g_tx_length = four_g_pack_data("AT+CMQTTCONNECT=0,\"tcp://iot.smartsensorserver.com:1883\",60,1,\"device\",\"K6kZDc5QjaYW2TF\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
//			four_g_tx_length = four_g_pack_data("AT+CMQTTCONNECT=0,\"tcp://test.mosquitto.org:1883\",60,1\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
				four_g_tx_length = four_g_pack_data("AT+CMQTTCONNECT=0,\"tcp://iot-visualiser.aql.com:1883\",60,1,\"server_admin\",\"K6kZDc5QjaYW2TF\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
//			four_g_tx_length = four_g_pack_data("AT+CMQTTCONNECT=0,\"tcp://iot-visualiser.aql.com:1883\",60,1\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
	#endif

#endif
#ifdef SSS_SERVER
				four_g_tx_length = four_g_pack_data("AT+CMQTTCONNECT=0,\"tcp://iot.smartsensorserver.com:1883\",60,1,\"device\",\"K6kZDc5QjaYW2TF\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
#endif
#ifdef MOSQUITTO_SERVER
				four_g_tx_length = four_g_pack_data("AT+CMQTTCONNECT=0,\"tcp://iot.smartsensorserver.com:1883\",60,1,\"device\",\"K6kZDc5QjaYW2TF\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
#endif

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
// response to CONNECT is just "OK"...	with +CMQTTCONNECT following later...			
			else if (mqtt_response_flag == RESP_OK)	//RESP_PCMQTT)	//RESP_OK)	
				{															
// ***		
/// NEED TO wait longer for the last part of the response to come in! Redesign rcv routine!
// ***
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

//				i = get_AT_value("AT+CSQ", "OK", "", eolstr, 1, 6, 0X0D, tmpstr, 15);
				i = get_AT_value("AT+CMQTTCONNECT", "+CMQTTCONNECT", "", eolstr, 4, 23, 0x0D, tmpstr, 2);	//1, 23, 0x0D, tmpstr, 15);
				n = atoi(tmpstr);
//				n = 0;
				if ((i) && (n==0))	// n== 0: no connect error
					{
					*mqtt_state = MQTT_CONNECT_CHECK;	//MQTT_SET_SUBSCRIBE_TOPIC;
					mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
					mqtt_error_attempts = 0;							// have passed through this phase of state machine successfully
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);

					if (pcmqtt_time)
						{
						ltmp = pcmqtt_time - start_time;
						if(debug_do(DBG_4G_TXRX))
							printf("Conn time: %lld.%06lldsec\n",ltmp/1000000,ltmp%1000000);
						}
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
/*
			else if (mqtt_response_flag == RESP_OK)
				{
				}
*/
			else
				{
				printf("Error! resp flag: %d [%s]\n",mqtt_response_flag,four_g_resps[mqtt_response_flag]);
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}

/*
			if (pcmqtt_time)
				{
				ltmp = pcmqtt_time - start_time;
				if(debug_do(DBG_4G_TXRX))
					printf("Conn time: %lld.%06lldsec\n",ltmp/1000000,ltmp%1000000);
				}
*/

			}
		break;
		
	case MQTT_CONNECT_CHECK:
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
				four_g_tx_length = four_g_pack_data("AT+CMQTTCONNECT?\r",(char *)four_g_tx_data);
				mqtt_response_flag = RESP_WAIT;
				xQueueReset(mqtt_cmd_queue);
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag == RESP_OK)	
				{																				
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("", "OK","", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
//*** FOR TEST!
					simcom_net_check("Post connect",1);

					printf("MQTT Connected\n");				

					*mqtt_state = MQTT_SUBSCRIBE_MSG;	//MQTT_SET_SUBSCRIBE_TOPIC;
					mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
					
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;

// can set several topics to subscribe to with multiple calls to AT+CMQTTSUBTOPIC;
// then apply all topic subscribes using AT+CMQTTSUB=0; OR
// subscribe to 1 topic and apply immediately with AT+CMQTTSUB=0,len, and input topic at the ">" prompt

//////////////////////
// NOT USED!
//////////////////////
	case MQTT_SET_SUBSCRIBE_TOPIC:
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
				sprintf(mqtt_subscribe_topic_str,"/gateway_dtm/%s\r",mac_bin_str);
				mqtt_subcribe_topic_length = strlen(mqtt_subscribe_topic_str);
				sprintf((char *)four_g_tx_data,"AT+CMQTTSUBTOPIC=0,%d,1\r",mqtt_subcribe_topic_length);
				four_g_tx_length = strlen((char *)four_g_tx_data);
			
//			four_g_tx_length = four_g_pack_data("AT+CMQTTSUBTOPIC=0,%d,1\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag == RESP_R_ARROW)	//RESP_OK)	
				{																				
// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect(eolstr[0]);

				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTSUBTOPIC", ">", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
//##DIVIDE!!!
//				four_g_tx_length = four_g_pack_data("/gateway_dtm/001122334455\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
					four_g_send_data((uint8_t *)mqtt_subscribe_topic_str,mqtt_subcribe_topic_length);

					four_g_rx_length = 0;
					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

					if(debug_do(DBG_4G_TXRX))
						printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

					i = get_AT_value("", "OK","", eolstr, 0, 0, 0, NULL, 0);
					if (i)
						{
						*mqtt_state = MQTT_SUBSCRIBE_MSG;	//TOPIC;
						mqtt_reset_flags(COMM_LONG_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
						simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
						}
					else
						{
						mqtt_attempts++;
						mqtt_response_flag = RESP_NONE;
						}		

//			printf("Receive successful!\r\n");
//		*mqtt_state = MQTT_SUBSCRIBE_MSG;

					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
			
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;

	case MQTT_SUBSCRIBE_TOPIC:
			// respond with CLOSED
// needs to be issued after topic data is input by 
// MQTT_SET_SUBSCRIBE_TOPIC:	AT+CMQTTSUBTOPIC

		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_LONG_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
				sprintf(mqtt_subscribe_topic_str,"/gateway_dtm/%s\r",mac_bin_str);
				mqtt_subcribe_topic_length = strlen(mqtt_subscribe_topic_str);
				sprintf((char *)four_g_tx_data,"AT+CMQTTSUBTOPIC=0,%d,1\r",mqtt_subcribe_topic_length);
				four_g_tx_length = strlen((char *)four_g_tx_data);

//			four_g_tx_length = four_g_pack_data("AT+CMQTTSUBTOPIC=0,9,1\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(mqtt_cmd_queue);
				four_g_tx_length = four_g_pack_data("/gateway_dom/SUBMSG\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag == RESP_R_ARROW)	
				{									
// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect(eolstr[0]);

				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("+CMQTTSUBTOPIC", ">", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
//##DIVIDE!!!

					*mqtt_state = MQTT_SUBSCRIBE_MSG;
//			printf("Receive successful!\r\n");
					mqtt_reset_flags(COMM_LONG_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
//			else if (mqtt_response_flag == OK)
//				{
//				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}

		break;

#define DBG_SEND_TIME		// measure time from start of SET_TOPIC to end of PUBLISH
#define DBG_STEP_TIME		// measure time for SET_TOPIC, SET_PAYLOAD, PUBLISH. MUST HAVE DBG_SEND_TIME defined!

	case MQTT_SUBSCRIBE_MSG:
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_LONG_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;

// The string needs a \r to terminate the 
//                                                1234567890123
//				sprintf(mqtt_subscribe_topic_str,"/gateway_dtm/%s\r",mac_bin_str);
				sprintf(mqtt_subscribe_topic_str,"/gateway_dtm/%s",mac_bin_str);
//			sprintf(mqtt_subscribe_topic_str,"dlt\r");
//			sprintf(mqtt_subscribe_topic_str,"/gateway_dtm/#\r");
//			sprintf(mqtt_subscribe_topic_str,"testing\r");	// TEST!
//			sprintf(mqtt_subscribe_topic_str,"#\r");	// TEST!
//			sprintf(mqtt_subscribe_topic_str,"/gateway_dtm/#\r");	// TEST! # = All topics; WONT WORK! Disconnects...
//			sprintf(mqtt_subscribe_topic_str,"/gateway_dom/frame/%s\r",mac_bin_str);	// TEST! This is the DATA topic!
				mqtt_subcribe_topic_length = strlen(mqtt_subscribe_topic_str);	// - 1;

				sprintf((char *)four_g_tx_data,"AT+CMQTTSUB=0,%d,1\r",mqtt_subcribe_topic_length);
				four_g_tx_length = strlen((char *)four_g_tx_data);
			
////			four_g_tx_length = four_g_pack_data("AT+CMQTTSUB=0\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);				// send "AT+CMQTTSUB=0,..."
				dbg_count = 3;
#ifdef DBG_SEND_TIME
				printf("SND SUB: ");
				show_time(DBG,1);
				printf("\n");
#endif
				}
			else if (mqtt_response_flag == RESP_R_ARROW)
				{									
// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect(eolstr[0]);

				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,100);		// receive ">" prompt

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTSUB", ">", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
//##DIVIDE!!!
//					printf("sending sub topic str...\n");
					four_g_send_data((uint8_t *)mqtt_subscribe_topic_str,mqtt_subcribe_topic_length);	// send topic string

					four_g_rx_length = 0;
					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,2000);					// receive topic string plus "OK"...

					if(debug_do(DBG_4G_TXRX))
						printf("Rx: <\n %s >  [%d] [%d]\r\n",four_g_rx_data, four_g_rx_length,dbg_count);

					i = get_AT_value("", "OK","+CMQTTSUB:", eolstr, 0, 0, 0, NULL, 0);							// check returned topic string
					if (i)
						{
						*mqtt_state = MQTT_SET_TOPIC;	//TOPIC;
						mqtt_reset_flags(COMM_VLONG_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
						simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
						}
					else
						{
						mqtt_attempts++;
						mqtt_response_flag = RESP_NONE;
						}		

/*
//			printf("Receive successful!\r\n");
				*mqtt_state = MQTT_SET_TOPIC;
				mqtt_state_mc_timer = 0;
				mqtt_attempts = 0;
*/

					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;
		

#ifdef DBG_STEP_TIME
	#ifndef DBG_SEND_TIME
	#error "must define DBG_SEND_TIME too!"
	#endif
#endif	
	case MQTT_SET_TOPIC:
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_VLONG_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
//					printf("MAC str is %s\r\n",mac_bin_str);

#ifdef DBG_SEND_TIME
				if (debug_do(DBG_MQTT))
					printf("SEND MSG START: SET_TOPIC...\n");
				
//				ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, 5000000));
				init_time = esp_timer_get_time();
				start_time = init_time;
#endif

/*
// show topic and payload to user	NOW DONE IN mqtt_send() rtn!
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
*/
#ifdef DBG_TEST
printf("Got here 1\n");
#endif
//### NEED TO clean up the use of the topic string and length...

//		four_g_tx_length = four_g_pack_data("AT+CMQTTTOPIC=0,9\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				sprintf((char *)four_g_tx_data,"AT+CMQTTTOPIC=0,%d\r",strlen(mqtt_topic_str));

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,strlen((char *)four_g_tx_data));
				}
			else if (mqtt_response_flag == RESP_R_ARROW)	
				{									
// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect(eolstr[0]);

				if(debug_do(DBG_4G_TXRX))
					printf("Time: %d\n",mqtt_timeout_timer);
				
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);		// just detected an arrow = the last char sent, so its all arrived; delay = 0

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("", ">", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
//##DIVIDE!!!
					{
					strcat(mqtt_topic_str,"\r");
//			four_g_tx_length = four_g_pack_data("/dom/100234/frame/node/1\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
					four_g_tx_length = four_g_pack_data(mqtt_topic_str,(char *)four_g_tx_data);		// respond with number <cr-lf> OK

					four_g_send_data(four_g_tx_data,four_g_tx_length);

					four_g_rx_length = 0;
					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,20);

					if(debug_do(DBG_4G_TXRX))
						printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

					i = get_AT_value("", "OK", "", eolstr, 0, 0, 0, NULL, 0);
					if (i)
						{
						*mqtt_state = MQTT_SET_PAYLOAD;
						mqtt_reset_flags(COMM_LONG_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
						simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);

#ifdef DBG_STEP_TIME
						end_time = esp_timer_get_time();
						ltmp = end_time - start_time;
						if (debug_do(DBG_MQTT))
							printf("SET TOPIC END:  TIME: %lld.%06lldsec\n",ltmp/1000000,ltmp%1000000);
						
						start_time = end_time;					// get ready for next reading
#endif
						}
					else
						{
						mqtt_attempts++;
						mqtt_response_flag = RESP_NONE;
						}		

//			printf("Receive successful!\r\n");
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,100);
				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);
					
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;
	case MQTT_SET_PAYLOAD:
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_LONG_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;

#ifdef DBG_SEND_TIME
				if (debug_do(DBG_MQTT))
					printf("SEND MSG START: SET_PAYLOAD...\n");
				
//				ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, 5000000));
//				init_time = esp_timer_get_time();
//				start_time = init_time;

				start_time = esp_timer_get_time();
#endif


// *** PAYLOAD can be up to 10240 bytes long!				
				sprintf((char *)four_g_tx_data,"AT+CMQTTPAYLOAD=0,%d\r",mqtt_payload_length);
				four_g_tx_length = strlen((char *)four_g_tx_data);

#ifdef DBG_TEST
printf("Got here 2\n");
#endif
// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
#ifdef DBG_TEST
printf("Got here 3\n");
#endif
				four_g_send_data(four_g_tx_data,four_g_tx_length);
#ifdef DBG_TEST
printf("Got here 4\n");
#endif
				}
			else if (mqtt_response_flag == RESP_R_ARROW)	
				{									
// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect(eolstr[0]);

				four_g_rx_length = 0;
#ifdef DBG_TEST
printf("Got here 5\n");
#endif
// receive the arrow character...
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);		// just detected an arrow = the last char sent, so its all arrived; delay = 0
#ifdef DBG_TEST
printf("Got here 6\n");
#endif
				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("", ">", "", eolstr, 0, 0, 0, NULL, 0);
// if arrow character received ok...
				if (i)
					{
//##DIVIDE!!!

//			four_g_tx_length = four_g_pack_data("TOPIC\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
////						four_g_tx_length = mqtt_payload_length;
////						memcpy((char *)four_g_tx_data, mqtt_payload_str,four_g_tx_length+1);	// +1 for 0x00 terminator

//				&(char *)four_g_tx_data[four_g_tx_length] = 0x00;
				
//				uart_flush(FOUR_G);
					xQueueReset(mqtt_cmd_queue);
//						four_g_send_data(four_g_tx_data,four_g_tx_length);
#ifdef DBG_TEST
printf("Got here 7\n");
#endif
// send payload data
					four_g_send_data((uint8_t*)mqtt_payload_str,mqtt_payload_length);		// use the mqtt arrays directly, to save duplicating RAM in the four_g_data array...

#ifdef DBG_TEST
printf("Got here 8\n");
#endif
//					vTaskDelay(10 / portTICK_PERIOD_MS);	// 10 milliseconds
//					vTaskDelay((mqtt_payload_length/5) / portTICK_PERIOD_MS);	// allow 2 x serial tx due to simcom out + ESP in times
//					vTaskDelay(10 / portTICK_PERIOD_MS);	// 10 milliseconds

					esp_task_wdt_reset();		// for long rx delay
					
// receive copy of payload with "CR-LF OK" appended
					four_g_rx_length = 0;
//					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,2500);	//mqtt_payload_length/8);		// was 40; for 4000 bytes, this is 100msec...
					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,(10+mqtt_payload_length/2));		// allow 2 x serial tx due to SIMCOM out + ESP in times

#ifdef DBG_TEST
printf("Got here 9\n");
#endif
					if(debug_do(DBG_4G_TXRX))
						printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

					i = get_AT_value("", "OK", "", eolstr, 0, 0, 0, NULL, 0);
					if (i)
						{
//			printf("Receive successful!\r\n");
						*mqtt_state = MQTT_PUBLISH_MSG;
						mqtt_reset_flags(COMM_LONG_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
						simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);

#ifdef DBG_STEP_TIME
						end_time = esp_timer_get_time();
						ltmp = end_time - start_time;
						if (debug_do(DBG_MQTT))
							printf("SET PAYLOAD END:  TIME: %lld.%06lldsec\n",ltmp/1000000,ltmp%1000000);
						
						start_time = end_time;					// get ready for next reading
#endif
						}
					else
						{
						mqtt_attempts++;
						mqtt_response_flag = RESP_NONE;
						}		
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}				
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;

	case MQTT_PUBLISH_MSG:
	// NOTE: once msg is published, the topic string disappears - have to do SET_TOPIC for every PUBLISH...
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_PUBLISH_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;

				start_time = esp_timer_get_time();

//		mqtt_payload_length = 5;	//strlen(mqtt_payload_str);
//		sprintf((char *)four_g_tx_data,"AT+CMQTTPUB=0,1,60\r");
//		four_g_tx_length = strlen((char *)four_g_tx_data);

				four_g_tx_length = four_g_pack_data("AT+CMQTTPUB=0,1,60\r",(char *)four_g_tx_data);
			
				xQueueReset(mqtt_cmd_queue);
// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect(eolstr[0]);

				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);

// flash network LED to show data transmitted...
#ifdef NETWORK_LED
				set_led(NETWORK_LED, DEVICE_OFF);
#else
				set_led(1, DEVICE_OFF);
#endif

				}
			else if (mqtt_response_flag == RESP_PCMQTT)	//OK)	
				{									
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,20);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTPUB", "+CMQTTPUB:", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*mqtt_state = MQTT_CONNECT_IDLE;
					mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
//					mqtt_error_attempts = 0;							// have passed through this phase of state machine successfully
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);

#ifdef DBG_SEND_TIME
					end_time = esp_timer_get_time();
#endif
#ifdef DBG_STEP_TIME
					ltmp = end_time - start_time;
					if (debug_do(DBG_MQTT))
						printf("PUBLISH END:  TIME: %lld.%06lldsec\n",ltmp/1000000,ltmp%1000000);
					
//					start_time = end_time;					// get ready for next reading
#endif
#ifdef DBG_SEND_TIME
					ltmp = end_time - init_time;
					if (debug_do(DBG_MQTT))
						printf("SEND MSG END: TIME: %lld.%06lldsec\n",ltmp/1000000,ltmp%1000000);
#endif
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else if (mqtt_response_flag == RESP_OK)
				{
				}
			else
				{
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;

	case MQTT_CONNECT_IDLE:
		mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
		mqtt_error_attempts = 0;
		break;
	
	case MQTT_UNSUBSCRIBE_TOPIC:
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
				sprintf(mqtt_subscribe_topic_str,"/gateway_dtm/%s\r",mac_bin_str);
				mqtt_subcribe_topic_length = strlen(mqtt_subscribe_topic_str);
				sprintf((char *)four_g_tx_data,"AT+CMQTTUNSUB=0,%d,0\r",mqtt_subcribe_topic_length);
				four_g_tx_length = strlen((char *)four_g_tx_data);

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag == RESP_R_ARROW)	
				{									
// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect(eolstr[0]);

				four_g_rx_length = 0;
					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);
					
					if(debug_do(DBG_4G_TXRX))
						printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

					i = get_AT_value("AT+CMQTTUNSUB", ">", "", eolstr, 0, 0, 0, NULL, 0);
					if (i)
						{
//				four_g_tx_length = four_g_pack_data("UNSUBMSG\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
//				four_g_send_data(four_g_tx_data,four_g_tx_length);
						four_g_send_data((uint8_t *)mqtt_subscribe_topic_str,mqtt_subcribe_topic_length);

//			printf("Receive successful!\r\n");
						*mqtt_state = MQTT_CONNECT_IDLE;
//				*mqtt_state = MQTT_SERVER_DISCONNECT;
						mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
//					mqtt_state_mc_timer = 0;
//					mqtt_attempts = 0;
						simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
						}
					else
						{
						mqtt_attempts++;
						mqtt_response_flag = RESP_NONE;
						}		
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;

	case MQTT_SERVER_DISCONNECT:
//		if ((mqtt_state_mc_timer >= COMM_DELAY) && (mqtt_attempts < COMM_PHASE_RETRIES))
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
				four_g_tx_length = four_g_pack_data("AT+CMQTTDISC=0,120\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
//			else if (mqtt_response_flag == RESP_WAIT)	
			else if (mqtt_response_flag == RESP_OK)	
				{																				
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTDISC", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*mqtt_state = MQTT_CLIENT_RELEASE;
					mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
				}
			else if (mqtt_response_flag == RESP_ERROR)
				{
// may not be connected - so ERROR if we try to disconn when not connected - proceed anyway
				*mqtt_state = MQTT_CLIENT_RELEASE;
				mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
				simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
				}
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;

	case MQTT_CLIENT_RELEASE:
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
				four_g_tx_length = four_g_pack_data("AT+CMQTTREL=0\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag == RESP_OK)	
				{																				
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTREL", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*mqtt_state = MQTT_STOP;
					mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;

	case MQTT_STOP:
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
				four_g_tx_length = four_g_pack_data("AT+CMQTTSTOP\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag == RESP_OK)	
				{																				
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTSTOP", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*mqtt_state = MQTT_NO_COMM_IDLE;
					mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}
			}
		break;

	case MQTT_COMM_IDLE:
		mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
		simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
		break;
/*
	case MQTT_POWER_DOWN_KEY:
		mqtt_reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
		*mqtt_state = MQTT_POWER_DOWN_KEY_WAIT;
		break;		

	case MQTT_POWER_DOWN_KEY_WAIT:
		if (mqtt_state_mc_timer >= FOUR_G_POWER_DOWN_KEY_WAIT)
			{
			mqtt_reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
			*mqtt_state = MQTT_POWER_DOWN_RESP_WAIT;
			}				
		break;
		
	case MQTT_POWER_DOWN_RESP_WAIT:
		if (mqtt_state_mc_timer >= FOUR_G_POWER_DOWN_RESP_WAIT)
			{
			mqtt_reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
			*mqtt_state = MQTT_POWER_DISABLE;
			}
		break;
		
	case MQTT_POWER_DISABLE:
		four_g_power_disable();
		mqtt_reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
		*mqtt_state = MQTT_POWER_OFF;
		mqtt_error_attempts = 0;							// have passed through this phase of state machine successfully
		break;	
*/		
////////////////////////////////////////
	case MQTT_IDLE_CHECK_4G_NET:
		simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;

		simcom_net_check("CMD",1);

		*mqtt_state = MQTT_CONNECT_IDLE;	//MQTT_NET_OPEN;	//MQTT_SET_APN;
		mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
		simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
		break;
		
	case MQTT_IDLE_CHECK_SIG_QUALITY:
//		AT+CGDCONT=1, "IP", "AQL"	// respond with OK
		if ((mqtt_timeout_timer) && (mqtt_attempts < COMM_PHASE_RETRIES))
			{
			if (mqtt_response_flag == RESP_NONE)
				{
				mqtt_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT mqtt_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_MQTT;
				four_g_tx_length = four_g_pack_data("AT+CSQ\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				mqtt_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (mqtt_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CSQ", "OK", "", eolstr, 1, 6, 0x0D, tmpstr, 15);
				if (i)
					{
					i = 0;
					j = 0;
					while((j==0) && (tmpstr[i] != 0x00))
						{
						if (tmpstr[i] == ',')
							{
							tmpstr[i] = 0x00;
							j = 1;
							k = atoi(tmpstr);
							}
						i++;
						}
						
					*mqtt_state = MQTT_COMMS_INIT_ERROR_END;
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);

					if (j)
						{
						if (((k==99) || (k==199)))
							{
							if (debug_do(DBG_MQTT))
								printf("No network signal detected!\r\n");
//							*mqtt_state = MQTT_ERROR_END;
							}
						else
							{
							if (k<32)
								{
								cell_rssi = -113 + (2*k);	// -113 to -53dBm
								}
							else if(k>190)
								{
								cell_rssi = -25;
								}
							else
								{
								cell_rssi = 216 - k;
								}
								
							if (debug_do(DBG_MQTT))
								printf("Network signal strength = %ddBm\r\n",cell_rssi);
							
							*mqtt_state = MQTT_CONNECT_IDLE;	//MQTT_NET_OPEN;	//MQTT_SET_APN;
							mqtt_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
							simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
							}
						}
					}
				else
					{
					mqtt_attempts++;
					mqtt_response_flag = RESP_NONE;
					}		
				}
//			else if (mqtt_response_flag == RESP_ERROR)
			else if (mqtt_response_flag == RESP_WAIT)
				{
				}
			else
				{
				mqtt_attempts++;
				mqtt_response_flag = RESP_NONE;
				}			
			}
		
		break;

//////////////////////////////////////////
		
/*
	case MQTT_POWER_UP_ERROR_END:
		mqtt_reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
		*mqtt_state = MQTT_POWER_EN;					// retry power-up
		break;

	case MQTT_CHECKS_ERROR_END:
		mqtt_reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
		*mqtt_state = MQTT_GET_MODULETYPE;				// retry checks
		break;
*/
	case MQTT_COMMS_INIT_ERROR_END:
		mqtt_reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
		*mqtt_state = MQTT_COMM_INIT;					// retry comm init
		printf("Retry - MQTT_COMM_INIT\n");
		break;

	case MQTT_CONNECT_ERROR_END:
		mqtt_reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
		*mqtt_state = MQTT_SERVER_DISCONNECT;	//MQTT_ACQUIRE_CLIENT;				// retry connect
		printf("Retry - MQTT_SERVER_DISCONNECT\n");	//MQTT_ACQUIRE_CLIENT\n");
		break;

	case MQTT_SEND_RCV_ERROR_END:
		mqtt_reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
		*mqtt_state = MQTT_SET_TOPIC;	// SUBSCRIBE_TOPIC;	// retry send \ rcv
		printf("Retry - MQTT_SET_TOPIC\n");
		break;


	case MQTT_DISC_PWRDOWN_ERROR_END:
		mqtt_reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
		*mqtt_state = MQTT_NO_COMM_IDLE;				// goto no comms \ idle state
		*simcom_state = SIMCOM_POWER_DOWN_KEY;			// goto power off state
		printf("Retry - SIMCOM_POWER_DOWN_KEY\n");
		break;

	case MQTT_END:
		mqtt_reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, mqtt_attempts, response flag
		break;
	default:
		break;
	}


//	if (mqtt_attempts)
//		printf("[ %s ]  mqtt_attempts: %d\n",mqtt_state_str[mqtt_last_error_state],mqtt_attempts);
	if ((mqtt_attempts) && (mqtt_attempts != prev_mqtt_attempts))
		printf("[ MQTT_%s ]  mqtt_attempts: %d\n",mqtt_state_str[*mqtt_state],mqtt_attempts);
	
	if (mqtt_timeout_timer == 0)
		{
		printf("MQTT Timeout [%d.%d sec] [rsp: %d %s] [eol: %d %s]\n",
				mqtt_timeout_val/10,mqtt_timeout_val%10,mqtt_response_flag,four_g_resps[mqtt_response_flag],mqtt_eol_response_flag,four_g_resps[mqtt_eol_response_flag]);

		four_g_rx_length = 0;
		four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,100);
		printf("TO Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

// reset uart pattern detect function.
		pattern_detect_char = set_uart_pattern_detect(eolstr[0]);
		
		mqtt_response_flag = RESP_NONE;		// try command again...
		mqtt_timeout_timer = 	mqtt_timeout_val;	// restore previous timeout value and try again...

		xQueueReset(mqtt_cmd_queue);

		mqtt_attempts++;
		}
		
	if (mqtt_attempts >= COMM_PHASE_RETRIES)
		{
		mqtt_last_error_state = *mqtt_state;
		printf("MQTT comm fail at %d [ MQTT_%s ] - too many retries [%d ]  [errors: %d]\r\n",mqtt_last_error_state,mqtt_state_str[mqtt_last_error_state],mqtt_attempts,mqtt_error_attempts);

		four_g_rx_length = 0;
		four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,100);
		printf("RT Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

		mqtt_reset_flags(mqtt_timeout_val,1);		// with last timeout val to be used
//		mqtt_attempts = 0;
		mqtt_error_attempts++;

		if (mqtt_error_attempts < ERROR_RETRIES)
			{
/*
			if (*mqtt_state <= MQTT_POWER_ON_RESP_WAIT)
				{
				*mqtt_state = MQTT_POWER_UP_ERROR_END;
				}
			else if ((*mqtt_state > MQTT_POWER_ON_RESP_WAIT) && (*mqtt_state <= MQTT_NO_COMM_IDLE))
				{
				*mqtt_state = MQTT_CHECKS_ERROR_END;
				}
			else 
*/
			if ((*mqtt_state > MQTT_NO_COMM_IDLE) && (*mqtt_state <= MQTT_COMM_START))
				{
				*mqtt_state = MQTT_COMMS_INIT_ERROR_END;	// goes to MQTT_COMM_INIT, directly after MQTT_NO_COMM_IDLE
				}
			else if ((*mqtt_state > MQTT_COMM_START) && (*mqtt_state <= MQTT_CONNECT))
				{
				*mqtt_state = MQTT_CONNECT_ERROR_END;		// goes to MQTT_ACQUIRE_CLIENT, directly after MQTT_COMM_START
				}
			else if ((*mqtt_state > MQTT_CONNECT) && (*mqtt_state <= MQTT_CONNECT_IDLE))
				{
				*mqtt_state = MQTT_SEND_RCV_ERROR_END;		// goes to MQTT_SET_TOPIC, directly after 
				}

			else if (*mqtt_state > MQTT_CONNECT_IDLE)
				{
				*mqtt_state = MQTT_DISC_PWRDOWN_ERROR_END;
				}

			}

// if too many error retries, downpower the SIMCOM module and start from the beginning...
		if (mqtt_error_attempts > MQTT_MAX_ERROR_RETRIES)
			{
#ifdef SIMCOM_PROGRAMMING_MODE
//			printf("SIMCOM comms fail - idling with power ON\r\n");
//			*mqtt_state = MQTT_NO_COMM_IDLE;				// stop coms mqtt_attempts but leave SIMCOM powered up...
// keep trying last step with power on (for debug)			
			mqtt_attempts = 0;
			prev_mqtt_attempts = 0;
			mqtt_error_attempts = 0;

#else
			*mqtt_state = MQTT_DISC_PWRDOWN_ERROR_END;		// goto power off state

//			*mqtt_state = MQTT_NO_COMM_IDLE;
//			*simcom_state = SIMCOM_POWER_DOWN_KEY;			// goto power off state

#endif
			}

		simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_MQTT);
		}
	prev_mqtt_attempts = mqtt_attempts;


		

}


void mqtt_reset_flags(unsigned int timeout, unsigned char clear_mqtt_attempts)
{
//mqtt_state_mc_timer = 0;
mqtt_timeout_timer = timeout;
mqtt_timeout_val = timeout;
mqtt_response_flag = RESP_NONE;
mqtt_eol_response_flag = RESP_NONE;
if (clear_mqtt_attempts)
	{
	mqtt_attempts = 0;	
	prev_mqtt_attempts = 0;
	}

// Set uart pattern detect function.
pattern_detect_char = set_uart_pattern_detect(eolstr[0]);

}

unsigned char mqtt_check_topic_str(void)
{
	// returns 1 if new topic set; 0 if same as old topic
unsigned char ret;

if (!strcmp(mqtt_topic_str,mqtt_prev_topic_str))	// if strings the same
	ret = 1;
else
	{
	strcpy(mqtt_prev_topic_str,mqtt_topic_str);	// set prev topic 
	ret = 0;
	}
	
return ret;
	
}

void print_mqtt_state(unsigned char chan)
{
//dbgprintf(chan,"MQTT State:  [%03d] < MQTT_%s >    MQTT timer: %d\r\n",mqtt_state,mqtt_state_str[mqtt_state],mqtt_state_mc_timer);
dbgprintf(chan,"MQTT 4G State   :  [%03d] < MQTT_%s >\r\n",mqtt_state,mqtt_state_str[mqtt_state]);
}

void print_mqtt_login_state(unsigned char chan)
{
//dbgprintf(chan,"MQTT State:  [%03d] < MQTT_%s >    MQTT timer: %d\r\n",mqtt_state,mqtt_state_str[mqtt_state],mqtt_state_mc_timer);
//dbgprintf(chan,"MQTT login State:  [%03d] < %s >\r\n",mqtt_login_state,mqtt_state_str[mqtt_state]);
//dbgprintf(chan,"MQTT login State:  [%03d]\r\n",mqtt_login_state);
dbgprintf(chan,"MQTT login State:  [%03d] < %s >    ",mqtt_login_state,mqtt_login_state_str[mqtt_login_state]);
dbgprintf(chan,"MQTT Queues Enabled: %d\n",queues_enabled_flag);
}
