////////////////////////////////////////////
//
// MAC_Rpt_four_g_mqtt.c
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

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

#include "MAC_Rpt.h"
#include "MAC_Rpt_Rtns.h"
#include "MAC_Rpt_four_g.h"
#include "MAC_Rpt_four_g_mqtt_defs.h"

#include "ca_cert.h"
#include "client_cert.h"
#include "client_key.h"

extern uint8_t four_g_tx_data[FOUR_G_TX_BUFSIZE];
extern uint8_t four_g_rx_data[FOUR_G_RX_BUFSIZE];
extern unsigned int four_g_tx_length;
extern unsigned int four_g_rx_length;
extern unsigned char four_g_response_flag, four_g_eol_response_flag;
extern unsigned char four_g_sm_timer_flag;

extern unsigned char four_g_rst_flag, four_g_pwr_flag, four_g_connected_flag;

extern unsigned char four_g_state, prev_four_g_state;
//extern unsigned char four_g_state_str[][];
extern unsigned int four_g_state_mc_timer;
extern unsigned int four_g_timeout_timer;
extern unsigned int four_g_timeout_val;

extern char mac_bin_str[];
extern char fwver_str[];
extern char imei_str[];
extern char simnum_str[];
extern char ipaddr_str[];

extern const char four_g_state_str[M4G_NUM_STATES][32];

extern unsigned int supply_voltage;
extern uint32_t voltage;
extern unsigned char temperature;

extern char gps_lat_str[];
extern char gps_lon_str[];
extern unsigned int gps_spd,lora_radio_freq;
extern unsigned char cell_csq;

unsigned char attempts, prev_attempts;
unsigned char error_attempts;

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

char gps_str[80];

extern signed char cell_rssi;

extern QueueHandle_t mqtt_cmd_queue;

extern unsigned char x100msec, secs, mins, hrs;

extern unsigned char ssl_enable_flag;

extern unsigned char server_ssl_mode[4];

extern unsigned char server0_addr_ptr;


extern char pattern_detect_char;

extern unsigned char eolstr[10];

unsigned char last_error_state;

int64_t init_time = 0;
int64_t start_time = 0;
int64_t end_time = 0;
int64_t final_time = 0;

extern int64_t pcmqtt_time;

extern unsigned char server0_addr_ptr, server1_addr_ptr;

extern unsigned int dbg_count;

//extern esp_timer_handle_t oneshot_timer;

/*
const char four_g_state_str[M4G_NUM_STATES][32] = 
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

// function prototypes
void reset_flags(unsigned int timeout, unsigned char clear_attempts);

//static void oneshot_timer_callback(void* arg);

// code
void four_g_mqtt_state_machine(unsigned char *four_g_state,unsigned char *prev_four_g_state)
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


#ifdef DBG_TEST
if (*four_g_state != *prev_four_g_state)
#else
if ((debug_do(DBG_MQTT)) && (*four_g_state != *prev_four_g_state))
#endif
	{
	printf("\r\n4G State: [%03d] M4G_%s\r\n",*four_g_state, four_g_state_str[*four_g_state]);
	}

*prev_four_g_state = *four_g_state;
eolstr[0] = 0x0A;
eolstr[1] = 0x00;

// system rests in M4G_POWER_OFF state
// manual change to M4G_POWER_EN starts 4G power up 
// system rests in M4G_NO_COMM_IDLE state
// manual change to M4G_COMM_INIT starts 4G connect
// system rests in M4G_CONNECT_IDLE state
switch(*four_g_state)
	{
	case M4G_POWER_OFF:
		reset_flags(COMM_TIMEOUT_DEFEAT,0);		// resets state_mc_timer, timeout_timer, attempts, response flag
		error_attempts = 0;
		last_error_state = M4G_NO_ERROR;
		break;
	case M4G_POWER_EN:
		four_g_power_en();
		reset_flags(COMM_TIMEOUT_DEFEAT,0);		// resets state_mc_timer, timeout_timer, attempts, response flag
		*four_g_state = M4G_POWER_EN_WAIT;
		break;		
	case M4G_POWER_EN_WAIT:
		if (four_g_state_mc_timer >= FOUR_G_POWER_EN_WAIT)
			{
			reset_flags(COMM_TIMEOUT_DEFEAT,0);		// resets state_mc_timer, timeout_timer, attempts, response flag
			*four_g_state = M4G_RESET;
			}
		break;
	case M4G_RESET:
		four_g_hold_in_reset();
		reset_flags(COMM_TIMEOUT_DEFEAT,0);		// resets state_mc_timer, timeout_timer, attempts, response flag
		*four_g_state = M4G_RESET_WAIT;
		break;
	case M4G_RESET_WAIT:
		if (four_g_state_mc_timer >= FOUR_G_RESET_WAIT)
			{
			four_g_release_from_reset();
			reset_flags(COMM_TIMEOUT_DEFEAT,0);		// resets state_mc_timer, timeout_timer, attempts, response flag
			*four_g_state = M4G_POWER_ON_KEY;
			}
		break;
	case M4G_POWER_ON_KEY:
		four_g_power_key_on();
		reset_flags(COMM_TIMEOUT_DEFEAT,0);		// resets state_mc_timer, timeout_timer, attempts, response flag
		*four_g_state = M4G_POWER_ON_KEY_WAIT;
		break;
	case M4G_POWER_ON_KEY_WAIT:
		if (four_g_state_mc_timer >= FOUR_G_POWER_ON_WAIT)
			{
			four_g_power_key_off();
			reset_flags(COMM_TIMEOUT_DEFEAT,0);		// resets state_mc_timer, timeout_timer, attempts, response flag
			*four_g_state = M4G_POWER_ON_RESP_WAIT;
			}
		break;
	case M4G_POWER_ON_RESP_WAIT:
		if (four_g_state_mc_timer < FOUR_G_POWER_ON_RESP_WAIT)
			{
			if (four_g_state_mc_timer%10 == 0)
			if (four_g_sm_timer_flag)
				{
				if (debug_do(DBG_4G_TXRX))
					{
					printf("%d",four_g_state_mc_timer/10);
					if (gpio_get_level(STATUS))					// dont exit early, as SIMCOM needs the full 16 sec to establish network handshake...
						printf(" + ");
					else
						printf(" - ");
					}
				four_g_sm_timer_flag = 0;
				}
/*
			if (gpio_get_level(STATUS))					// dont exit early, as SIMCOM needs the full 16 sec to establish network handshake...
				{
				printf(" + ");
//				printf("STATUS line is set...\r\n");
//				*four_g_state = M4G_GET_MODULETYPE;
//				reset_flags(COMM_TIMEOUT);		// resets state_mc_timer, timeout_timer, attempts, response flag
//				four_g_state_mc_timer = 0;
//				error_attempts = 0;							// have passed through this phase of state machine successfully
				}
			else
				printf(" - ");
*/					
			}
		else
			{
			if(debug_do(DBG_4G_TXRX))
				{
				if (gpio_get_level(STATUS))
					printf("STATUS line is set - OK\r\n");
				else
					printf("STATUS line not set!\r\n");
				}
				
//#ifndef SIMCOM_PROGRAMMING_MODE
			*four_g_state = M4G_GET_MODULETYPE;	// M4G_GPS_START;
			reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
			error_attempts = 0;							// have passed through this phase of state machine successfully
//#endif
			}
				
		break;

	case M4G_GET_MODULETYPE:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CGMM\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_OK)	
				{
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);
					
				*four_g_state = M4G_GET_FWVERSION;
				reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;

	case M4G_GET_FWVERSION:
// Known working Firmware versions:
// LE11B12SIM7600M22_HB
// LE11B13SIM7600M22

// non-working versions:
// LE11B08SIM7600M22		// CSSLCFG fails...

// LE20B04SIM7600M22	// ???

		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CGMR\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_OK)	
				{				
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);
				
// save fw ver number
				strcpy(fwver_str,(char *)&four_g_rx_data[17]);
				i = 0;
// remove trailing "OK" and CR-LFs...
				while (fwver_str[i] != 0x0D) 
					{i++;}
				fwver_str[i] = 0x00;
				
//				*four_g_state = M4G_GPS_CHECK;
				*four_g_state = M4G_GET_SIM_IMEI;
				reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}

			}
		break;

	case M4G_GET_SIM_MAC:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CWMACADDR?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("", "OK", "", eolstr, 1, 0, 0x0D, module_mac_str, 20);

				if(debug_do(DBG_4G_TXRX))
					printf("MAC addr: %s\r\n",module_mac_str);
					
				if (i)
					{
					*four_g_state = M4G_GET_SIM_IMEI;
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;
		
	case M4G_GET_SIM_IMEI:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+GSN\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+GSN", "OK", "", eolstr, 1, 0, 0x0D, imei_str, 20);
				if (i)
					{
					*four_g_state = M4G_GET_SIM_NUM;
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
//					four_g_state_mc_timer = 0;
//					attempts = 0;
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;

	case M4G_GET_SIM_NUM:
//		AT+CCID		// respond with number <cr-lf> OK; if number ends in F, strip...
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CICCID\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CICCID", "OK", "", eolstr, 1, 8, 0x0D, simnum_str, 22);
				if (i)
					{
//					*four_g_state = M4G_NO_COMM_IDLE;
//##					*four_g_state = M4G_GPS_CHECK;
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag

#ifndef SIMCOM_PROGRAMMING_MODE
					if(debug_do(DBG_MQTT))
#endif
						{
						printf("\nFW  version: %s\n",fwver_str);
						printf("IMEI number: %s\n",imei_str);
						printf("SIM  number: %s\n\n",simnum_str);
						}
#ifdef SIMCOM_PROGRAMMING_MODE
					*four_g_state = M4G_NO_COMM_IDLE;
#else					
					*four_g_state = M4G_GPS_CHECK;	
#endif
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}				
			}
		break;


	case M4G_GPS_CHECK:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CGPS?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_OK)	
				{								
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);
					
				tmpstr[0] = 0x00;
				i = get_AT_value("", "OK", "", eolstr, 1, 7, 0x0D, tmpstr, 4);
				
				if(debug_do(DBG_4G_TXRX))
					printf("GPS Status: %s\r\n",tmpstr);
					
				if (i)
					{
					if (tmpstr[0] == '1')				// if GPS already on...
						*four_g_state = M4G_GPS_INFO;
					else
						*four_g_state = M4G_GPS_START;

					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}				

			}
		break;
		
	case M4G_GPS_START:
// AT+CGPS=1,1
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CGPS=1,1\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_OK)	
				{												
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("", "OK", "", eolstr, 1, 0, 0x0D, NULL, 0);
	//			printf("MAC addr: %s\r\n",module_mac_str);
				if (i)
					{
					*four_g_state = M4G_GPS_INFO;
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}				

			}

		break;

	case M4G_GPS_INFO:
// AT+CGPS=1,1
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CGPSINFO\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);				
				}
			else if (four_g_response_flag == RESP_OK)	
				{												
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("", "OK", "", eolstr, 1, 0, 0x0D, gps_str, 80);

				if(debug_do(DBG_4G_TXRX))
					printf("GPS: %s\r\n",gps_str);
					
				if (i)
					{
					char tmp_str[80];
					char time_str[10];
					unsigned int n;
					
					j = 11;	
					j = j + 1 + get_str(&gps_str[j], ",", gps_lat_str,dbg_4g_flag);	// latitude
					j = j + 1 + get_str(&gps_str[j], ",", tmp_str,dbg_4g_flag);		// N/S
					strcat(gps_lat_str,tmp_str);
					j = j + 1 + get_str(&gps_str[j], ",", gps_lon_str,dbg_4g_flag);	// longtitude
					j = j + 1 + get_str(&gps_str[j], ",", tmp_str,dbg_4g_flag);		// E/W
					strcat(gps_lon_str,tmp_str);
					j = j + 1 + get_str(&gps_str[j], ",", tmp_str,dbg_4g_flag);		// date
					j = j + 1 + get_str(&gps_str[j], ",", time_str,dbg_4g_flag);	// time
					j = j + 1 + get_str(&gps_str[j], ",", tmp_str,dbg_4g_flag);		// altitude
					j = j + 1 + get_str(&gps_str[j], ",", tmp_str,dbg_4g_flag);		// speed
					gps_spd = atoi(tmp_str);
					j = j + 1 + get_str(&gps_str[j], ",", tmp_str,dbg_4g_flag);		// course

	//unsigned char x100msec, secs, mins, hrs;
					if (strlen(time_str) == 6)
						{
	// hrs
						tmp_str[0] = time_str[0];
						tmp_str[1] = time_str[1];
						tmp_str[2] = 0x00;
						n = atoi(tmp_str);

	// mins
						tmp_str[0] = time_str[2];
						tmp_str[1] = time_str[3];
						tmp_str[2] = 0x00;
						n = atoi(tmp_str);

	// secs
						tmp_str[0] = time_str[4];
						tmp_str[1] = time_str[5];
						tmp_str[2] = 0x00;
						n = atoi(tmp_str);
						
						}
						
//					*four_g_state = M4G_GET_SIM_IMEI;	// M4G_NO_COMM_IDLE;						
					*four_g_state = M4G_NO_COMM_IDLE;						
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					error_attempts = 0;							// have passed through this phase of state machine successfully

					if(debug_do(DBG_4G_TXRX))
						printf("Reached NO_COMM_IDLE\r\n");
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}

		break;

	case M4G_NO_COMM_IDLE:
//		printf("Reached NO_COMM_IDLE\r\n");
		reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag

		break;

	case M4G_COMM_INIT:
		if(debug_do(DBG_4G_TXRX))
			printf("Comms Init...\r\n");

// Set default uart pattern detect function.
		pattern_detect_char = set_uart_pattern_detect(eolstr[0]);
	
		*four_g_state = M4G_CLOSEALL;
		reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
//		attempts = 0;
		break;
		

	case M4G_CLOSEALL:
//		AT+CIPSHUT		// respond with SHUT OK
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+NETCLOSE\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag != RESP_WAIT)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+NETCLOSE", "", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*four_g_state = M4G_CHECK_SIG_QUALITY;	//M4G_SET_PDP;
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
				}
				
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;

///////////////////////////

	case M4G_CHECK_SIG_QUALITY:
//		AT+CGDCONT=1, "IP", "AQL"	// respond with OK
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
				four_g_tx_length = four_g_pack_data("AT+CSQ\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_OK)	
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
						
//					*four_g_state = M4G_COMMS_INIT_ERROR_END;

					if (j)
						{
						if (((k==99) || (k==199)))
							{
							if (debug_do(DBG_MQTT))
								printf("No network signal detected!\r\n");
//							*four_g_state = M4G_ERROR_END;
							attempts++;
							four_g_response_flag = RESP_NONE;
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
							
							*four_g_state = M4G_CHECK_4G_NETWORK;	//M4G_NET_OPEN;	//M4G_SET_APN;
							reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
							}
						}
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}			
			}
		
		break;


	case M4G_CHECK_4G_NETWORK:
//		AT+CSTT="m2m.aql.net"	// respond with OK	AT+CSOCKSETPN=1
//		four_g_tx_length = four_g_pack_data("AT+CSTT=\"m2m.aql.net\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
				four_g_tx_length = four_g_pack_data("AT+CREG?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_OK)	
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
							printf("Not registered on network\n");
							break;
						case 1:
							printf("Registered on home network\n");
							break;
						case 2:
							printf("Not registered on network,searching for new network\n");
							break;
						case 3:
							printf("Registration denied\n");
							break;
						case 4:
							printf("Unknown ststus\n");
							break;
						case 5:
							printf("Registered \ in roaming mode\n");
							break;
						default:
							break;
						}
						
					if (n!= 1)
						{
						attempts++;
						four_g_response_flag = RESP_NONE;
						}
					else
						{
						*four_g_state = M4G_CHECK_GPRS_NETWORK;
						reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
						}
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			
			}
		break;

	case M4G_CHECK_GPRS_NETWORK:
//		AT_COPS=1,2,"23429"			// respond with OK
//		four_g_tx_length = four_g_pack_data("AT_COPS=1,2,\"23429\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
				four_g_tx_length = four_g_pack_data("AT+CGREG?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CGREG", "OK", "", eolstr, 1, 8, 0x0D, tmpstr, 15);
				if (i)
					{
//					*four_g_state =  M4G_SET_PDP;	//M4G_MQTT_CHECKSTOP;	//M4G_SET_PDP;	//M4G_CHECK_CONN;
					*four_g_state =  M4G_CHECK_OPS;	//M4G_MQTT_CHECKSTOP;	//M4G_SET_PDP;	//M4G_CHECK_CONN;
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}				
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;

	case M4G_CHECK_OPS:
//		AT_COPS=1,2,"23429"			// respond with OK
//		four_g_tx_length = four_g_pack_data("AT_COPS=1,2,\"23429\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
				four_g_tx_length = four_g_pack_data("AT+COPS?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_OK)	
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
					*four_g_state =  M4G_MQTT_CHECK_RELEASE;	//M4G_SET_PDP;	//M4G_MQTT_CHECKSTOP;	//M4G_SET_PDP;	//M4G_CHECK_CONN;
#else
					*four_g_state =  M4G_SET_PDP;	
#endif
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}				
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;

#ifdef USE_RELASE_AND_STOP_STATES
	case M4G_MQTT_CHECK_RELEASE:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
				four_g_tx_length = four_g_pack_data("AT+CMQTTREL=0\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_OK)	
				{																				
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTREL", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*four_g_state = M4G_MQTT_CHECK_STOP;
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;
		
	case M4G_MQTT_CHECK_STOP:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
				four_g_tx_length = four_g_pack_data("AT+CMQTTSTOP\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if ((four_g_response_flag == RESP_OK) || (four_g_response_flag == RESP_ERROR))	
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
					*four_g_state = M4G_SET_PDP;	//M4G_CHECK_CONN;
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}				
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;
#endif
		
	case M4G_SET_PDP:
//		AT+CGDCONT=1, "IP", "AQL"	// respond with OK
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
#if 1
				four_g_tx_length = four_g_pack_data("AT+CGSOCKCONT=1, \"IP\", \"AQL\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
#else
				four_g_tx_length = four_g_pack_data("AT+CGDCONT=1,\"IP\",\"AQL\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
#endif

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CGSOCKCONT", "OK", "", eolstr, 0, 0, 0, NULL, 0);
//				i = get_AT_value("AT+CGDCONT", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
#ifdef USE_M4G_MQTT_SSL
					if ((ssl_enable_flag) && (server_ssl_mode[server0_addr_ptr]))
						*four_g_state = M4G_SET_SSL_CONFIG;
					else
#endif
						*four_g_state = M4G_COMM_START;	//M4G_ACT_PDP;	//M4G_COMM_START;	//M4G_NET_OPEN;	//M4G_SET_APN;
					
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}				
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;

	case M4G_ACT_PDP:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_LONG_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
				four_g_tx_length = four_g_pack_data("AT+CGACT=1,1\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CGACT", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
#ifdef USE_M4G_MQTT_SSL
//					if (ssl_enable_flag)
					if ((ssl_enable_flag) && (server_ssl_mode[server0_addr_ptr]))
						*four_g_state = M4G_SET_SSL_CONFIG;
					else
#endif
						*four_g_state = M4G_COMM_START;
						
					error_attempts = 0;							// have passed through this phase of state machine successfully

//###endif
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}				
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				printf("RX ERR\n");
				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				attempts++;
				four_g_response_flag = RESP_NONE;
				}			
			}
		break;

/// SSL states
	case M4G_SET_SSL_CONFIG:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
//		AT_COPS=0			// respond with +COPS: 0,2,"310410",7
				four_g_tx_length = four_g_pack_data("AT+CSSLCFG=\"sslversion\",0,4\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CSSLCFG", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*four_g_state = M4G_SET_SSL_AUTHMODE;
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;
		
	case M4G_SET_SSL_AUTHMODE:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
//		AT_CSQ				// respond with +CSQ: 14,99
// 0 = no auth
// 1 = server only
// 2 = full auth
				four_g_tx_length = four_g_pack_data("AT+CSSLCFG=\"authmode\",0,2\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length); 
				}
			else if (four_g_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CSSLCFG", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*four_g_state = M4G_LOAD_SSL_SERVER_CERTIFICATE;
					reset_flags(COMM_CERT_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;

//#define DEBUG_CERTS
	case M4G_LOAD_SSL_SERVER_CERTIFICATE:
//		if ((four_g_state_mc_timer >= COMM_DELAY) && (attempts < COMM_PHASE_RETRIES))
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_CERT_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
				sprintf((char*)four_g_tx_data,"AT+CCERTDOWN=\"server_cert.pem\",%d\r",ca_cert_size);
				four_g_tx_length = strlen((char *)four_g_tx_data);

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_R_ARROW)	//RESP_EOL)	
				{									
// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect(eolstr[0]);

				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CCERTDOWN", ">", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					for (n=0;n<ca_cert_size;n++)
						{
						sprintf((char *)four_g_tx_data,"%c",ca_cert_data[n]);
						uart_write_bytes(FOUR_G, (char *)four_g_tx_data, 1);
						}
						
					four_g_rx_length = ca_cert_size+6;	// allow for 0x0D,0x0D, O, K, 0x0D,0x0A
#ifdef DEBUG_CERTS
					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,100);

					if(debug_do(DBG_4G_TXRX))
						printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);
#else
					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,0,150);
#endif
					i = get_AT_value("", "OK", "", eolstr, 0, 0, 0, NULL, 0);
					if (i)
						{
						*four_g_state = M4G_LOAD_SSL_CLIENT_CERTIFICATE;
						reset_flags(COMM_CERT_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
						}
					else
						{
						attempts++;
						four_g_response_flag = RESP_NONE;
						}		
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
					
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;
		
	case M4G_LOAD_SSL_CLIENT_CERTIFICATE:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_CERT_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
				sprintf((char*)four_g_tx_data,"AT+CCERTDOWN=\"client_cert.pem\",%d\r",client_cert_size);
				four_g_tx_length = strlen((char *)four_g_tx_data);

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_R_ARROW)	//RESP_EOL)	
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
					for (n=0;n<client_cert_size;n++)
						{
						sprintf((char *)four_g_tx_data,"%c",client_cert_data[n]);
						uart_write_bytes(FOUR_G, (char *)four_g_tx_data, 1);
						}

					four_g_rx_length = client_cert_size+6;	// allow for 0x0D,0x0D, O, K, 0x0D,0x0A

#ifdef DEBUG_CERTS
					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,100);

					if(debug_do(DBG_4G_TXRX))
						printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);
#else
					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,0,150);
#endif
					i = get_AT_value("", "OK", "", eolstr, 0, 0, 0, NULL, 0);
					if (i)
						{
						*four_g_state = M4G_LOAD_SSL_CLIENT_KEY;
						reset_flags(COMM_CERT_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
						}
					else
						{
						attempts++;
						four_g_response_flag = RESP_NONE;
						}		
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
				
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;
		
	case M4G_LOAD_SSL_CLIENT_KEY:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_CERT_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
				sprintf((char*)four_g_tx_data,"AT+CCERTDOWN=\"client_key.pem\",%d\r",client_key_size);
				four_g_tx_length = strlen((char *)four_g_tx_data);
	
// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_R_ARROW)	//RESP_EOL)	
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
					for (n=0;n<client_key_size;n++)
						{
						sprintf((char *)four_g_tx_data,"%c",client_key_data[n]);
						uart_write_bytes(FOUR_G, (char *)four_g_tx_data, 1);
						}

				four_g_rx_length = client_key_size+6;	// allow for 0x0D,0x0D, O, K, 0x0D,0x0A

#ifdef DEBUG_CERTS
					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,100);
					if(debug_do(DBG_4G_TXRX))
						printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);
#else
					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,0,150);
#endif
					i = get_AT_value("", "OK", "", eolstr, 0, 0, 0, NULL, 0);
					if (i)
						{
						*four_g_state = M4G_SHOW_SSL_CERTIFICATES;
						reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
						}
					else
						{
						attempts++;
						four_g_response_flag = RESP_NONE;
						}		
					}					
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;
		
	case M4G_SHOW_SSL_CERTIFICATES:
//		if ((four_g_state_mc_timer >= COMM_DELAY) && (attempts < COMM_PHASE_RETRIES))
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
//		AT_CPSI?			// respond with 
//+CPSI: LTE CAT-M1,Online,310-410,0x4804,74777865,343,EUTRAN-BAND2,875,4,4,-13,-112,-82,14
				four_g_tx_length = four_g_pack_data("AT+CCERTLIST\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
//			uart_flush(FOUR_G);
				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);		
				}
//			else if (four_g_response_flag == RESP_WAIT)	
			else if (four_g_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				*four_g_state = M4G_SET_SSL_SERVER_CERTIFICATE;
				reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;
		
	case M4G_SET_SSL_SERVER_CERTIFICATE:
//		if ((four_g_state_mc_timer >= COMM_DELAY) && (attempts < COMM_PHASE_RETRIES))
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
//		AT_CPSI?			// respond with 
//+CPSI: LTE CAT-M1,Online,310-410,0x4804,74777865,343,EUTRAN-BAND2,875,4,4,-13,-112,-82,14
				four_g_tx_length = four_g_pack_data("AT+CSSLCFG=\"cacert\",0,\"server_cert.pem\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);
	
				i = get_AT_value("AT+CSSLCFG", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*four_g_state = M4G_SET_SSL_CLIENT_CERTIFICATE;
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;
		

	case M4G_SET_SSL_CLIENT_CERTIFICATE:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
//		AT+CGATT?		// respond with +CGATT: 1 <cr-lf><cr-lf> OK
				four_g_tx_length = four_g_pack_data("AT+CSSLCFG=\"clientcert\",0,\"client_cert.pem\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CSSLCFG", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*four_g_state = M4G_SET_SSL_CLIENT_KEY;
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;
	case M4G_SET_SSL_CLIENT_KEY:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
//		AT+CNACT=1,"m2m.aql.net"	//respond with OK <cr-lf><cr-lf> +APP PDP: ACTIVE
				four_g_tx_length = four_g_pack_data("AT+CSSLCFG=\"clientkey\",0,\"client_key.pem\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}

			else if (four_g_response_flag == RESP_OK)	
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
					*four_g_state = M4G_COMM_START;
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
//					error_attempts = 0;							// have passed through this phase of state machine successfully
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;
/// SSL end		

	case M4G_COMM_START:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
//				reset_flags(COMM__TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
				reset_flags(50,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
				four_g_tx_length = four_g_pack_data("AT+CMQTTSTART\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_PCMQTT)	//RESP_OK)	
				{		
				printf("RSP OK\n");				
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTSTART", "+CMQTT", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*four_g_state = M4G_MQTT_ACQUIRE_CLIENT;
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					error_attempts = 0;							// have passed through this phase of state machine successfully
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				printf("RSP ERR\n");				
				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;
		
	case M4G_MQTT_ACQUIRE_CLIENT:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
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
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTACCQ", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*four_g_state = M4G_MQTT_SET_WILLTOPIC;
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;

	case M4G_MQTT_SET_WILLTOPIC:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
				strcpy(mqtt_will_topic_str, "/gateway_dom/");
				strcat(mqtt_will_topic_str,mac_bin_str);

				sprintf((char *)four_g_tx_data,"AT+CMQTTWILLTOPIC=0,%d\r",strlen(mqtt_will_topic_str));		// respond with number <cr-lf> OK

//			four_g_tx_length = four_g_pack_data("AT+CMQTTWILLTOPIC=0,%d\r",(char *)four_g_tx_data,strlen(mqtt_will_topic_str));		// respond with number <cr-lf> OK
				four_g_tx_length = strlen((char *)four_g_tx_data);		// respond with number <cr-lf> OK

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_R_ARROW)	
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

					*four_g_state = M4G_MQTT_SET_WILLMSG;	//M4G_SET_APN;
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;
		
	case M4G_MQTT_SET_WILLMSG:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
				strcpy(mqtt_will_msg_str, "disconnect");

				sprintf((char *)four_g_tx_data,"AT+CMQTTWILLMSG=0,%d,1\r",strlen(mqtt_will_msg_str));		// respond with number <cr-lf> OK

//			four_g_tx_length = four_g_pack_data("AT+CMQTTWILLMSG=0,%d,1\r",(char *)four_g_tx_data,strlen(mqtt_will_msg_str));		// respond with number <cr-lf> OK
				four_g_tx_length = strlen((char *)four_g_tx_data);		// respond with number <cr-lf> OK

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_R_ARROW)	
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

					*four_g_state = M4G_MQTT_SET_UTF8_MODE;	//M4G_SET_APN;
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					}
				else
					{					
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;
		
	case M4G_MQTT_SET_UTF8_MODE:
//		AT+CIPOPEN="TCP","iot-visualiser.aql.com","80"
// respond with OK <cr-lf><cr-lf> CONNECT OK <cr-lf><cr-lf>
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
				four_g_tx_length = four_g_pack_data("AT+CMQTTCFG=\"checkUTF8\",0,0\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
//				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,30);
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,1,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTCFG", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*four_g_state = M4G_MQTT_CONNECT;
					reset_flags(COMM_CONN_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
//					error_attempts = 0;							// have passed through this phase of state machine successfully
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;
		
	case M4G_MQTT_CONNECT:
//		AT+CIPSEND	// start input; terminate with 0x1A;	responds with SEND OK
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				char tmp_connect_str[100];
				
				start_time = esp_timer_get_time();
				pcmqtt_time = 0;
				
				reset_flags(COMM_CONN_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
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
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_PCMQTT)	//RESP_OK)	
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
					*four_g_state = M4G_MQTT_CONNECT_CHECK;	//M4G_MQTT_SET_SUBSCRIBE_TOPIC;
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					error_attempts = 0;							// have passed through this phase of state machine successfully

					if (pcmqtt_time)
						{
						ltmp = pcmqtt_time - start_time;
						if(debug_do(DBG_4G_TXRX))
							printf("Conn time: %lld.%06lldsec\n",ltmp/1000000,ltmp%1000000);
						}
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else if (four_g_response_flag == RESP_OK)
				{
				}
			else
				{
				printf("Error! resp flag: %d\n",four_g_response_flag);
				attempts++;
				four_g_response_flag = RESP_NONE;
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
		
	case M4G_MQTT_CONNECT_CHECK:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
				four_g_tx_length = four_g_pack_data("AT+CMQTTCONNECT?\r",(char *)four_g_tx_data);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_OK)	
				{																				
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("", "OK","", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*four_g_state = M4G_MQTT_SUBSCRIBE_MSG;	//M4G_MQTT_SET_SUBSCRIBE_TOPIC;
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
					
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;

// can set several topics to subscribe to with multiple calls to AT+CMQTTSUBTOPIC;
// then apply all topic subscribes using AT+CMQTTSUB=0; OR
// subscribe to 1 topic and apply immediately with AT+CMQTTSUB=0,len, and input topic at the ">" prompt

//////////////////////
// NOT USED!
//////////////////////
	case M4G_MQTT_SET_SUBSCRIBE_TOPIC:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
				sprintf(mqtt_subscribe_topic_str,"/gateway_dtm/%s\r",mac_bin_str);
				mqtt_subcribe_topic_length = strlen(mqtt_subscribe_topic_str);
				sprintf((char *)four_g_tx_data,"AT+CMQTTSUBTOPIC=0,%d,1\r",mqtt_subcribe_topic_length);
				four_g_tx_length = strlen((char *)four_g_tx_data);
			
//			four_g_tx_length = four_g_pack_data("AT+CMQTTSUBTOPIC=0,%d,1\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_R_ARROW)	//RESP_OK)	
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
						*four_g_state = M4G_MQTT_SUBSCRIBE_MSG;	//TOPIC;
						reset_flags(COMM_LONG_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
						}
					else
						{
						attempts++;
						four_g_response_flag = RESP_NONE;
						}		

//			printf("Receive successful!\r\n");
//		*four_g_state = M4G_MQTT_SUBSCRIBE_MSG;

					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
			
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;

	case M4G_MQTT_SUBSCRIBE_TOPIC:
			// respond with CLOSED
// needs to be issued after topic data is input by 
// MQTT_SET_SUBSCRIBE_TOPIC:	AT+CMQTTSUBTOPIC

		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_LONG_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
				sprintf(mqtt_subscribe_topic_str,"/gateway_dtm/%s\r",mac_bin_str);
				mqtt_subcribe_topic_length = strlen(mqtt_subscribe_topic_str);
				sprintf((char *)four_g_tx_data,"AT+CMQTTSUBTOPIC=0,%d,1\r",mqtt_subcribe_topic_length);
				four_g_tx_length = strlen((char *)four_g_tx_data);

//			four_g_tx_length = four_g_pack_data("AT+CMQTTSUBTOPIC=0,9,1\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(mqtt_cmd_queue);
				four_g_tx_length = four_g_pack_data("/gateway_dom/SUBMSG\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_R_ARROW)	
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

					*four_g_state = M4G_MQTT_SUBSCRIBE_MSG;
//			printf("Receive successful!\r\n");
					reset_flags(COMM_LONG_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
//			else if (four_g_response_flag == OK)
//				{
//				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}

		break;

#define DBG_SEND_TIME		// measure time from start of SET_TOPIC to end of PUBLISH
#define DBG_STEP_TIME		// measure time for SET_TOPIC, SET_PAYLOAD, PUBLISH. MUST HAVE DBG_SEND_TIME defined!

	case M4G_MQTT_SUBSCRIBE_MSG:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_LONG_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
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
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);				// send "AT+CMQTTSUB=0,..."
				dbg_count = 3;
#ifdef DBG_SEND_TIME
				printf("SND SUB: ");
				show_time(DBG,1);
				printf("\n");
#endif
				}
			else if (four_g_response_flag == RESP_R_ARROW)
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
						*four_g_state = M4G_MQTT_SET_TOPIC;	//TOPIC;
						reset_flags(COMM_VLONG_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
						}
					else
						{
						attempts++;
						four_g_response_flag = RESP_NONE;
						}		

/*
//			printf("Receive successful!\r\n");
				*four_g_state = M4G_MQTT_SET_TOPIC;
				four_g_state_mc_timer = 0;
				attempts = 0;
*/

					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;
		

#ifdef DBG_STEP_TIME
	#ifndef DBG_SEND_TIME
	#error "must define DBG_SEND_TIME too!"
	#endif
#endif	
	case M4G_MQTT_SET_TOPIC:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_VLONG_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
//					printf("MAC str is %s\r\n",mac_bin_str);

#ifdef DBG_SEND_TIME
				if (debug_do(DBG_MQTT))
					printf("SEND MSG START: SET_TOPIC...\n");
				
//				ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, 5000000));
				init_time = esp_timer_get_time();
				start_time = init_time;
#endif

// show topic and payload to user
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
#ifdef DBG_TEST
printf("Got here 1\n");
#endif
//### NEED TO clean up the use of the topic string and length...

//		four_g_tx_length = four_g_pack_data("AT+CMQTTTOPIC=0,9\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				sprintf((char *)four_g_tx_data,"AT+CMQTTTOPIC=0,%d\r",strlen(mqtt_topic_str));

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,strlen((char *)four_g_tx_data));
				}
			else if (four_g_response_flag == RESP_R_ARROW)	
				{									
// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect(eolstr[0]);

				if(debug_do(DBG_4G_TXRX))
					printf("Time: %d\n",four_g_timeout_timer);
				
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
						*four_g_state = M4G_MQTT_SET_PAYLOAD;
						reset_flags(COMM_LONG_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag

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
						attempts++;
						four_g_response_flag = RESP_NONE;
						}		

//			printf("Receive successful!\r\n");
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,100);
				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);
					
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;
	case M4G_MQTT_SET_PAYLOAD:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_LONG_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!

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
				four_g_response_flag = RESP_WAIT;
#ifdef DBG_TEST
printf("Got here 3\n");
#endif
				four_g_send_data(four_g_tx_data,four_g_tx_length);
#ifdef DBG_TEST
printf("Got here 4\n");
#endif
				}
			else if (four_g_response_flag == RESP_R_ARROW)	
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
						*four_g_state = M4G_MQTT_PUBLISH_MSG;
						reset_flags(COMM_LONG_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag

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
						attempts++;
						four_g_response_flag = RESP_NONE;
						}		
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}				
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;

	case M4G_MQTT_PUBLISH_MSG:
	// NOTE: once msg is published, the topic string disappears - have to do SET_TOPIC for every PUBLISH...
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_PUBLISH_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!

				start_time = esp_timer_get_time();

//		mqtt_payload_length = 5;	//strlen(mqtt_payload_str);
//		sprintf((char *)four_g_tx_data,"AT+CMQTTPUB=0,1,60\r");
//		four_g_tx_length = strlen((char *)four_g_tx_data);

				four_g_tx_length = four_g_pack_data("AT+CMQTTPUB=0,1,60\r",(char *)four_g_tx_data);
			
				xQueueReset(mqtt_cmd_queue);
// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect(eolstr[0]);

				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_PCMQTT)	//OK)	
				{									
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,20);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTPUB", "+CMQTTPUB:", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*four_g_state = M4G_CONNECT_IDLE;
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
//					error_attempts = 0;							// have passed through this phase of state machine successfully

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
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else if (four_g_response_flag == RESP_OK)
				{
				}
			else
				{
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;

	case M4G_CONNECT_IDLE:
		reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
		error_attempts = 0;
		break;
	
	case M4G_MQTT_UNSUBSCRIBE_TOPIC:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
				sprintf(mqtt_subscribe_topic_str,"/gateway_dtm/%s\r",mac_bin_str);
				mqtt_subcribe_topic_length = strlen(mqtt_subscribe_topic_str);
				sprintf((char *)four_g_tx_data,"AT+CMQTTUNSUB=0,%d,0\r",mqtt_subcribe_topic_length);
				four_g_tx_length = strlen((char *)four_g_tx_data);

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_R_ARROW)	
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
						*four_g_state = M4G_CONNECT_IDLE;
//				*four_g_state = M4G_MQTT_SERVER_DISCONNECT;
						reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
//					four_g_state_mc_timer = 0;
//					attempts = 0;
						}
					else
						{
						attempts++;
						four_g_response_flag = RESP_NONE;
						}		
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;

	case M4G_MQTT_SERVER_DISCONNECT:
//		if ((four_g_state_mc_timer >= COMM_DELAY) && (attempts < COMM_PHASE_RETRIES))
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
				four_g_tx_length = four_g_pack_data("AT+CMQTTDISC=0,120\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
//			else if (four_g_response_flag == RESP_WAIT)	
			else if (four_g_response_flag == RESP_OK)	
				{																				
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTDISC", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*four_g_state = M4G_MQTT_CLIENT_RELEASE;
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;

	case M4G_MQTT_CLIENT_RELEASE:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
				four_g_tx_length = four_g_pack_data("AT+CMQTTREL=0\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_OK)	
				{																				
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTREL", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*four_g_state = M4G_MQTT_STOP;
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;

	case M4G_MQTT_STOP:
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
				four_g_tx_length = four_g_pack_data("AT+CMQTTSTOP\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_OK)	
				{																				
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTSTOP", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*four_g_state = M4G_COMM_IDLE;
					reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}
			}
		break;

	case M4G_COMM_IDLE:
		reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
		break;

	case M4G_POWER_DOWN_KEY:
		reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
		*four_g_state = M4G_POWER_DOWN_KEY_WAIT;
		break;		

	case M4G_POWER_DOWN_KEY_WAIT:
		if (four_g_state_mc_timer >= FOUR_G_POWER_DOWN_KEY_WAIT)
			{
			reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
			*four_g_state = M4G_POWER_DOWN_RESP_WAIT;
			}				
		break;
		
	case M4G_POWER_DOWN_RESP_WAIT:
		if (four_g_state_mc_timer >= FOUR_G_POWER_DOWN_RESP_WAIT)
			{
			reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
			*four_g_state = M4G_POWER_DISABLE;
			}
		break;
		
	case M4G_POWER_DISABLE:
		four_g_power_disable();
		reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
		*four_g_state = M4G_POWER_OFF;
		error_attempts = 0;							// have passed through this phase of state machine successfully
		break;	
////////////////////////////////////////
	case M4G_MQTT_IDLE_CHECK_SIG_QUALITY:
//		AT+CGDCONT=1, "IP", "AQL"	// respond with OK
		if ((four_g_timeout_timer) && (attempts < COMM_PHASE_RETRIES))
			{
			if (four_g_response_flag == RESP_NONE)
				{
				reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT attempts!
				four_g_tx_length = four_g_pack_data("AT+CSQ\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (four_g_response_flag == RESP_OK)	
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
						
					*four_g_state = M4G_COMMS_INIT_ERROR_END;

					if (j)
						{
						if (((k==99) || (k==199)))
							{
							if (debug_do(DBG_MQTT))
								printf("No network signal detected!\r\n");
//							*four_g_state = M4G_ERROR_END;
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
							
							*four_g_state = M4G_CONNECT_IDLE;	//M4G_NET_OPEN;	//M4G_SET_APN;
							reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
							}
						}
					}
				else
					{
					attempts++;
					four_g_response_flag = RESP_NONE;
					}		
				}
//			else if (four_g_response_flag == RESP_ERROR)
			else if (four_g_response_flag == RESP_WAIT)
				{
				}
			else
				{
				attempts++;
				four_g_response_flag = RESP_NONE;
				}			
			}
		
		break;

//////////////////////////////////////////
		
	case M4G_POWER_UP_ERROR_END:
		reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
		*four_g_state = M4G_POWER_EN;					// retry power-up
		break;

	case M4G_CHECKS_ERROR_END:
		reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
		*four_g_state = M4G_GET_MODULETYPE;				// retry checks
		break;

	case M4G_COMMS_INIT_ERROR_END:
		reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
		*four_g_state = M4G_COMM_INIT;					// retry comm init
		break;

	case M4G_CONNECT_ERROR_END:
		reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
		*four_g_state = M4G_MQTT_ACQUIRE_CLIENT;		// retry connect
		break;

	case M4G_SEND_RCV_ERROR_END:
		reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
		*four_g_state = M4G_MQTT_SET_TOPIC;	// SUBSCRIBE_TOPIC;	// retry send \ rcv
		break;

	case M4G_DISC_PWRDOWN_ERROR_END:
		reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
		*four_g_state = M4G_POWER_DOWN_KEY;				// goto power off state
		break;
	case M4G_END:
		reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
		break;
	default:
		break;
	}


//	if (attempts)
//		printf("[ %s ]  Attempts: %d\n",four_g_state_str[last_error_state],attempts);
	if ((attempts) && (attempts != prev_attempts))
		printf("[ %s ]  Attempts: %d\n",four_g_state_str[*four_g_state],attempts);
	
	if (four_g_timeout_timer == 0)
		{
		printf("MQTT Timeout [%d.%d sec] [rsp: %d %s] [eol: %d %s]\n",
				four_g_timeout_val/10,four_g_timeout_val%10,four_g_response_flag,four_g_resps[four_g_response_flag],four_g_eol_response_flag,four_g_resps[four_g_eol_response_flag]);

		four_g_rx_length = 0;
		four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,100);
		printf("TO Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

// reset uart pattern detect function.
		pattern_detect_char = set_uart_pattern_detect(eolstr[0]);
		
		four_g_response_flag = RESP_NONE;		// try command again...
		four_g_timeout_timer = 	four_g_timeout_val;	// restore previous timeout value and try again...

		xQueueReset(mqtt_cmd_queue);

		attempts++;
		}
		
	if (attempts >= COMM_PHASE_RETRIES)
		{
		last_error_state = *four_g_state;
		printf("MQTT comm fail at %d [ %s ] - too many retries [%d ]  [errors: %d]\r\n",last_error_state,four_g_state_str[last_error_state],attempts,error_attempts);

		four_g_rx_length = 0;
		four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,100);
		printf("RT Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

		reset_flags(four_g_timeout_val,1);		// with last timeout val to be used
//		attempts = 0;
		error_attempts++;

		if (error_attempts < ERROR_RETRIES)
			{
			if (*four_g_state <= M4G_POWER_ON_RESP_WAIT)
				{
				*four_g_state = M4G_POWER_UP_ERROR_END;
				}
			else if ((*four_g_state > M4G_POWER_ON_RESP_WAIT) && (*four_g_state <= M4G_NO_COMM_IDLE))
				{
				*four_g_state = M4G_CHECKS_ERROR_END;
				}
			else if ((*four_g_state > M4G_NO_COMM_IDLE) && (*four_g_state <= M4G_COMM_START))
				{
				*four_g_state = M4G_COMMS_INIT_ERROR_END;
				}
			else if ((*four_g_state > M4G_COMM_START) && (*four_g_state <= M4G_MQTT_CONNECT))
				{
				*four_g_state = M4G_CONNECT_ERROR_END;
				}
			else if ((*four_g_state > M4G_MQTT_CONNECT) && (*four_g_state <= M4G_CONNECT_IDLE))
				{
				*four_g_state = M4G_SEND_RCV_ERROR_END;
				}
			else if (*four_g_state > M4G_CONNECT_IDLE)
				{
				*four_g_state = M4G_DISC_PWRDOWN_ERROR_END;
				}
			}

// if too many error retries, downpower the SIMVCOM module and start from the beginning...
		if (error_attempts > MAX_ERROR_RETRIES)
			{
#ifdef SIMCOM_PROGRAMMING_MODE
//			printf("SIMCOM comms fail - idling with power ON\r\n");
//			*four_g_state = M4G_NO_COMM_IDLE;				// stop coms attempts but leave SIMCOM powered up...
// keep tryinglast step with power on (for debug)			
			attempts = 0;
			prev_attempts = 0;
			error_attempts = 0;

#else
			*four_g_state = M4G_POWER_DOWN_KEY;
#endif
			}

		}
	prev_attempts = attempts;

}


void reset_flags(unsigned int timeout, unsigned char clear_attempts)
{
four_g_state_mc_timer = 0;
four_g_timeout_timer = timeout;
four_g_timeout_val = timeout;
four_g_response_flag = RESP_NONE;
four_g_eol_response_flag = RESP_NONE;
if (clear_attempts)
	{
	attempts = 0;	
	prev_attempts = 0;
	}

// Set uart pattern detect function.
pattern_detect_char = set_uart_pattern_detect(eolstr[0]);

}

unsigned char check_topic_str(void)
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
