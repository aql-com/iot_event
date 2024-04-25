////////////////////////////////////////////
//
// MAC_Rpt_four_g_udp.cpp
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

#include "MAC_Rpt.h"
#include "MAC_Rpt_Rtns.h"
#include "MAC_Rpt_four_g.h"
#include "MAC_Rpt_four_g_simcom.h"
//#include "MAC_Rpt_four_g_udp_defs.h"
#include "MAC_Rpt_four_g_udp.h"

//#include "ca_cert.h"
//#include "client_cert.h"
//#include "client_key.h"

extern uint8_t four_g_tx_data[FOUR_G_TX_BUFSIZE];
extern uint8_t four_g_rx_data[FOUR_G_RX_BUFSIZE];
extern unsigned int four_g_tx_length;
extern unsigned int four_g_rx_length;
extern unsigned char udp_response_flag, udp_eol_response_flag;

extern unsigned char udp_sm_timer_flag;

extern unsigned char four_g_rst_flag, four_g_pwr_flag, four_g_connected_flag;

extern unsigned char udp_state, prev_udp_state;
/*
extern unsigned char udp_state_str[][];
extern unsigned int udp_state_mc_timer;
extern unsigned char four_g_sm_timer_flag;
*/
extern unsigned int four_g_timeout_timer;
extern unsigned int four_g_timeout_val;

extern unsigned int udp_timeout_timer;
extern unsigned int udp_timeout_val;

extern unsigned char simcom_busy_flag;

extern char mac_bin_str[];
extern char fwver_str[];
extern char imei_str[];
extern char simnum_str[];
extern char ipaddr_str[];

extern const char udp_state_str[UDP_NUM_STATES][32];

extern unsigned int supply_voltage;
extern uint32_t voltage;
extern unsigned char temperature;

extern char gps_lat_str[];
extern char gps_lon_str[];
extern unsigned int gps_spd,lora_radio_freq;
extern unsigned char cell_csq;

unsigned char udp_attempts, prev_udp_attempts;
unsigned char udp_error_attempts;

extern char module_mac_str[];

char udp_will_topic_str[80];
char udp_will_msg_str[40];
char udp_topic_str[80];
char udp_prev_topic_str[80];
unsigned int udp_topic_length;
char udp_payload_str[UDP_PAYLOAD_SIZE];		//256...
unsigned int udp_payload_length;
char udp_subscribe_topic_str[80];
unsigned int udp_subcribe_topic_length;

//char gps_str[80];

extern signed char cell_rssi;

extern QueueHandle_t udp_cmd_queue;

extern unsigned char x100msec, secs, mins, hrs;

extern unsigned char ssl_enable_flag;

extern unsigned char server_ssl_mode[4];

extern unsigned char server0_addr_ptr;


extern char pattern_detect_char;

extern unsigned char eolstr[10];

unsigned char udp_last_error_state;

int64_t udp_init_time = 0;
int64_t udp_start_time = 0;
int64_t udp_end_time = 0;
int64_t udp_final_time = 0;

extern int64_t pcudp_time;

extern unsigned char server0_addr_ptr, server1_addr_ptr;

extern unsigned int dbg_count;

//extern esp_timer_handle_t oneshot_timer;

/*
const char udp_state_str[UDP_NUM_STATES][32] = 
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
void udp_reset_flags(unsigned int timeout, unsigned char clear_udp_attempts);

//static void oneshot_timer_callback(void* arg);

// code
void four_g_udp_state_machine(unsigned char *udp_state,unsigned char *prev_udp_state, unsigned char *simcom_state)
{
int i;
unsigned char j,k;
unsigned int n;
char c;
char * p;
char tmpstr[20];


char eolstr[5];
//char udp_topic_str[80];
//char udp_payload_str[80];
//unsigned int udp_payload_length;

int64_t ltmp = 0;

unsigned char dbg_4g_flag;

// reset WatchDog Timer...
	esp_task_wdt_reset();		

if(debug_do(DBG_4G_TXRX))
	dbg_4g_flag = 1;
else
	dbg_4g_flag = 0;


#ifdef DBG_TEST
if (*udp_state != *prev_udp_state)
#else
if ((debug_do(DBG_UDP)) && (*udp_state != *prev_udp_state))
#endif
	{
	printf("\r\nUDP State: [%03d] UDP_%s\r\n",*udp_state, udp_state_str[*udp_state]);
	}

*prev_udp_state = *udp_state;
eolstr[0] = 0x0A;
eolstr[1] = 0x00;

// system rests in UDP_POWER_OFF state
// manual change to UDP_POWER_EN starts 4G power up 
// system rests in UDP_NO_COMM_IDLE state
// manual change to UDP_COMM_INIT starts 4G connect
// system rests in UDP_CONNECT_IDLE state
switch(*udp_state)
	{
	case UDP_NO_COMM_IDLE:
//		printf("Reached NO_COMM_IDLE\r\n");
		udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
		simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_UDP);

		break;
		
	case UDP_COMM_INIT_2:
		*udp_state = UDP_SET_PDP_2;						
		udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
		break;

// NEW UDP states
	case UDP_SET_PDP_2:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				printf("$");
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_UDP;
				four_g_tx_length = four_g_pack_data("AT+CGDCONT=2,\"IP\",\"UDP_NET\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag != RESP_WAIT)	
				{															
				printf("£");
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CGDCONT", "", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*udp_state = UDP_SET_MODE;	//UDP_SET_PDP;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_UDP);
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
				
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				printf("&");
				}
			else
				{
				printf("%%");
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;
	case UDP_SET_MODE:				// NONTRANSPARENT
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_UDP;
				four_g_tx_length = four_g_pack_data("AT+CIPMODE=0\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag != RESP_WAIT)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CIPMODE", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*udp_state = UDP_NET_OPEN;	//UDP_SET_PDP;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_UDP);
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
				
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;
	case UDP_NET_OPEN:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_UDP;
				four_g_tx_length = four_g_pack_data("AT+NETOPEN\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag != RESP_WAIT)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+NETOPEN", "", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*udp_state = UDP_SOCKET_OPEN;	//UDP_SET_PDP;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_UDP);
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
				
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;
	case UDP_SOCKET_OPEN:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_UDP;
				four_g_tx_length = four_g_pack_data("AT+CIPOPEN=1,\"UDP\",,,5000\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag != RESP_WAIT)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CIPOPEN", "", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*udp_state = UDP_SET_RESPONSE;	//UDP_SET_PDP;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_UDP);
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
				
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;
	case UDP_SET_RESPONSE:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_UDP;
				four_g_tx_length = four_g_pack_data("AT+CIPRXGET=0\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag != RESP_WAIT)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CIPRXGET", "", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*udp_state = UDP_COMM_IDLE_2;	//UDP_SET_PDP;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_UDP);
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
				
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;
		
	case UDP_COMM_IDLE_2:
		udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
		break;

	case UDP_SEND:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_LONG_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_UDP;
// The string needs a \r to terminate the 
//                                                1234567890123
//				sprintf(udp_subscribe_topic_str,"/gateway_dtm/%s",mac_bin_str);
				udp_payload_length = strlen(udp_payload_str);	// - 1;

//				sprintf((char *)four_g_tx_data,"AT+CMQTTSUB=0,%d,1\r",udp_subcribe_topic_length);
// NEW ADDRESS: external internet 109.239.102.45 on port 1700
//				sprintf((char *)four_g_tx_data,"AT+CIPSEND=1,%d,\"192.168.0.3\",7000\r",udp_payload_length);
				sprintf((char *)four_g_tx_data,"AT+CIPSEND=1,%d,\"109.239.102.45\",1700\r",udp_payload_length);
				four_g_tx_length = strlen((char *)four_g_tx_data);
			
////			four_g_tx_length = four_g_pack_data("AT+CMQTTSUB=0\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);				// send "AT+CMQTTSUB=0,..."
				dbg_count = 3;
#ifdef DBG_Sudp_end_time
				printf("SND SUB: ");
				show_time(DBG,1);
				printf("\n");
#endif
				}
			else if (udp_response_flag == RESP_R_ARROW)
				{									
// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect(eolstr[0]);

				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,100);		// receive ">" prompt

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CIPSEND", ">", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
//##DIVIDE!!!
//					printf("sending sub topic str...\n");
					four_g_send_data((uint8_t *)udp_payload_str,udp_payload_length);	// send topic string

					four_g_rx_length = 0;
					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,2000);					// receive topic string plus "OK"...

					if(debug_do(DBG_4G_TXRX))
						printf("Rx: <\n %s >  [%d] [%d]\r\n",four_g_rx_data, four_g_rx_length,dbg_count);

					i = get_AT_value("", "OK","+CIPSEND:", eolstr, 0, 0, 0, NULL, 0);							// check returned topic string
					if (i)
						{
						*udp_state = UDP_COMM_IDLE_2;	//TOPIC;
						udp_reset_flags(COMM_LONG_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
						simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_UDP);
						}
					else
						{
						udp_attempts++;
						udp_response_flag = RESP_NONE;
						}		

/*
//			printf("Receive successful!\r\n");
				*udp_state = UDP_MQTT_SET_TOPIC;
				udp_state_mc_timer = 0;
				udp_attempts = 0;
*/

					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;

	case UDP_SOCKET_CLOSE:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_UDP;
				four_g_tx_length = four_g_pack_data("AT+CIPCLOSE=2\r",(char *)four_g_tx_data);		// close socket 2 - respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag != RESP_WAIT)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CIPCLOSE", "", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
//					*udp_state = UDP_NET_CLOSE;	//UDP_SET_PDP;
					*udp_state = UDP_NO_COMM_IDLE;	//UDP_SET_PDP;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_UDP);
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
				
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;
		
	case UDP_NET_CLOSE:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
				simcom_busy_flag = simcom_busy_flag | SIMCOM_BUSY_UDP;
				four_g_tx_length = four_g_pack_data("AT+NETCLOSE\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag != RESP_WAIT)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+NETCLOSE", "", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*udp_state = UDP_NO_COMM_IDLE;	//UDP_SET_PDP;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_UDP);
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
				
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;

/*
// OLD MQTT code
	case UDP_COMM_INIT:
		if(debug_do(DBG_4G_TXRX))
			printf("Comms Init...\r\n");

// Set default uart pattern detect function.
		pattern_detect_char = set_uart_pattern_detect(eolstr[0]);
	
		*udp_state = UDP_CLOSEALL;
		udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
//		udp_attempts = 0;
		break;
		

	case UDP_CLOSEALL:
//		AT+CIPSHUT		// respond with SHUT OK
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+NETCLOSE\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag != RESP_WAIT)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+NETCLOSE", "", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*udp_state = UDP_CHECK_SIG_QUALITY;	//UDP_SET_PDP;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
				
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;

///////////////////////////

	case UDP_CHECK_SIG_QUALITY:
//		AT+CGDCONT=1, "IP", "AQL"	// respond with OK
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
				four_g_tx_length = four_g_pack_data("AT+CSQ\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_OK)	
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
						
//					*udp_state = UDP_COMMS_INIT_ERROR_END;

					if (j)
						{
						if (((k==99) || (k==199)))
							{
							if (debug_do(DBG_UDP))
								printf("No network signal detected!\r\n");
//							*udp_state = UDP_ERROR_END;
							udp_attempts++;
							udp_response_flag = RESP_NONE;
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
								
							if (debug_do(DBG_UDP))
								printf("Network signal strength = %ddBm\r\n",cell_rssi);
							
							*udp_state = UDP_CHECK_4G_NETWORK;	//UDP_NET_OPEN;	//UDP_SET_APN;
							udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
							}
						}
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}			
			}
		
		break;


	case UDP_CHECK_4G_NETWORK:
//		AT+CSTT="m2m.aql.net"	// respond with OK	AT+CSOCKSETPN=1
//		four_g_tx_length = four_g_pack_data("AT+CSTT=\"m2m.aql.net\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
				four_g_tx_length = four_g_pack_data("AT+CREG?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_OK)	
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
						udp_attempts++;
						udp_response_flag = RESP_NONE;
						}
					else
						{
						*udp_state = UDP_CHECK_GPRS_NETWORK;
						udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
						}
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			
			}
		break;

	case UDP_CHECK_GPRS_NETWORK:
//		AT_COPS=1,2,"23429"			// respond with OK
//		four_g_tx_length = four_g_pack_data("AT_COPS=1,2,\"23429\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
				four_g_tx_length = four_g_pack_data("AT+CGREG?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CGREG", "OK", "", eolstr, 1, 8, 0x0D, tmpstr, 15);
				if (i)
					{
//					*udp_state =  UDP_SET_PDP;	//UDP_MQTT_CHECKSTOP;	//UDP_SET_PDP;	//UDP_CHECK_CONN;
					*udp_state =  UDP_CHECK_OPS;	//UDP_MQTT_CHECKSTOP;	//UDP_SET_PDP;	//UDP_CHECK_CONN;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}				
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;

	case UDP_CHECK_OPS:
//		AT_COPS=1,2,"23429"			// respond with OK
//		four_g_tx_length = four_g_pack_data("AT_COPS=1,2,\"23429\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
				four_g_tx_length = four_g_pack_data("AT+COPS?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_OK)	
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
					*udp_state =  UDP_MQTT_CHECK_RELEASE;	//UDP_SET_PDP;	//UDP_MQTT_CHECKSTOP;	//UDP_SET_PDP;	//UDP_CHECK_CONN;
#else
					*udp_state =  UDP_SET_PDP;	
#endif
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}				
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;

#ifdef USE_RELASE_AND_STOP_STATES
	case UDP_MQTT_CHECK_RELEASE:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
				four_g_tx_length = four_g_pack_data("AT+CMQTTREL=0\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_OK)	
				{																				
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTREL", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*udp_state = UDP_MQTT_CHECK_STOP;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;
		
	case UDP_MQTT_CHECK_STOP:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
				four_g_tx_length = four_g_pack_data("AT+CMQTTSTOP\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if ((udp_response_flag == RESP_OK) || (udp_response_flag == RESP_ERROR))	
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
					*udp_state = UDP_SET_PDP;	//UDP_CHECK_CONN;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}				
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;
#endif
		
	case UDP_SET_PDP:
//		AT+CGDCONT=1, "IP", "AQL"	// respond with OK
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
#if 1
				four_g_tx_length = four_g_pack_data("AT+CGSOCKCONT=1, \"IP\", \"AQL\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
#else
				four_g_tx_length = four_g_pack_data("AT+CGDCONT=1,\"IP\",\"AQL\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
#endif

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CGSOCKCONT", "OK", "", eolstr, 0, 0, 0, NULL, 0);
//				i = get_AT_value("AT+CGDCONT", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
#ifdef USE_UDP_MQTT_SSL
					if ((ssl_enable_flag) && (server_ssl_mode[server0_addr_ptr]))
						*udp_state = UDP_SET_SSL_CONFIG;
					else
#endif
						*udp_state = UDP_COMM_START;	//UDP_ACT_PDP;	//UDP_COMM_START;	//UDP_NET_OPEN;	//UDP_SET_APN;
					
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}				
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;

	case UDP_ACT_PDP:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_LONG_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
				four_g_tx_length = four_g_pack_data("AT+CGACT=1,1\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CGACT", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
#ifdef XX_USE_UDP_MQTT_SSL
//					if (ssl_enable_flag)
					if ((ssl_enable_flag) && (server_ssl_mode[server0_addr_ptr]))
						*udp_state = UDP_SET_SSL_CONFIG;
					else
#endif
						*udp_state = UDP_COMM_START;
						
					udp_error_attempts = 0;							// have passed through this phase of state machine successfully

//###endif
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}				
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				printf("RX ERR\n");
				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}			
			}
		break;

#ifdef XX_USE_UDP_MQTT_SSL
/// SSL states
	case UDP_SET_SSL_CONFIG:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
//		AT_COPS=0			// respond with +COPS: 0,2,"310410",7
				four_g_tx_length = four_g_pack_data("AT+CSSLCFG=\"sslversion\",0,4\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CSSLCFG", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*udp_state = UDP_SET_SSL_AUTHMODE;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;
		
	case UDP_SET_SSL_AUTHMODE:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
//		AT_CSQ				// respond with +CSQ: 14,99
// 0 = no auth
// 1 = server only
// 2 = full auth
				four_g_tx_length = four_g_pack_data("AT+CSSLCFG=\"authmode\",0,2\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length); 
				}
			else if (udp_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CSSLCFG", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*udp_state = UDP_LOAD_SSL_SERVER_CERTIFICATE;
					udp_reset_flags(COMM_CERT_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;

//#define DEBUG_CERTS
	case UDP_LOAD_SSL_SERVER_CERTIFICATE:
//		if ((udp_state_mc_timer >= COMM_DELAY) && (udp_attempts < COMM_PHASE_RETRIES))
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_CERT_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
				sprintf((char*)four_g_tx_data,"AT+CCERTDOWN=\"server_cert.pem\",%d\r",ca_cert_size);
				four_g_tx_length = strlen((char *)four_g_tx_data);

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_R_ARROW)	//RESP_EOL)	
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
						*udp_state = UDP_LOAD_SSL_CLIENT_CERTIFICATE;
						udp_reset_flags(COMM_CERT_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
						}
					else
						{
						udp_attempts++;
						udp_response_flag = RESP_NONE;
						}		
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
					
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;
		
	case UDP_LOAD_SSL_CLIENT_CERTIFICATE:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_CERT_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
				sprintf((char*)four_g_tx_data,"AT+CCERTDOWN=\"client_cert.pem\",%d\r",client_cert_size);
				four_g_tx_length = strlen((char *)four_g_tx_data);

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_R_ARROW)	//RESP_EOL)	
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
						*udp_state = UDP_LOAD_SSL_CLIENT_KEY;
						udp_reset_flags(COMM_CERT_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
						}
					else
						{
						udp_attempts++;
						udp_response_flag = RESP_NONE;
						}		
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;
		
	case UDP_LOAD_SSL_CLIENT_KEY:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_CERT_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
				sprintf((char*)four_g_tx_data,"AT+CCERTDOWN=\"client_key.pem\",%d\r",client_key_size);
				four_g_tx_length = strlen((char *)four_g_tx_data);
	
// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_R_ARROW)	//RESP_EOL)	
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
						*udp_state = UDP_SHOW_SSL_CERTIFICATES;
						udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
						}
					else
						{
						udp_attempts++;
						udp_response_flag = RESP_NONE;
						}		
					}					
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;
		
	case UDP_SHOW_SSL_CERTIFICATES:
//		if ((udp_state_mc_timer >= COMM_DELAY) && (udp_attempts < COMM_PHASE_RETRIES))
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
//		AT_CPSI?			// respond with 
//+CPSI: LTE CAT-M1,Online,310-410,0x4804,74777865,343,EUTRAN-BAND2,875,4,4,-13,-112,-82,14
				four_g_tx_length = four_g_pack_data("AT+CCERTLIST\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
//			uart_flush(FOUR_G);
				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);		
				}
//			else if (udp_response_flag == RESP_WAIT)	
			else if (udp_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				*udp_state = UDP_SET_SSL_SERVER_CERTIFICATE;
				udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;
		
	case UDP_SET_SSL_SERVER_CERTIFICATE:
//		if ((udp_state_mc_timer >= COMM_DELAY) && (udp_attempts < COMM_PHASE_RETRIES))
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
//		AT_CPSI?			// respond with 
//+CPSI: LTE CAT-M1,Online,310-410,0x4804,74777865,343,EUTRAN-BAND2,875,4,4,-13,-112,-82,14
				four_g_tx_length = four_g_pack_data("AT+CSSLCFG=\"cacert\",0,\"server_cert.pem\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);
	
				i = get_AT_value("AT+CSSLCFG", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*udp_state = UDP_SET_SSL_CLIENT_CERTIFICATE;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;
		

	case UDP_SET_SSL_CLIENT_CERTIFICATE:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
//		AT+CGATT?		// respond with +CGATT: 1 <cr-lf><cr-lf> OK
				four_g_tx_length = four_g_pack_data("AT+CSSLCFG=\"clientcert\",0,\"client_cert.pem\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CSSLCFG", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*udp_state = UDP_SET_SSL_CLIENT_KEY;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;
	case UDP_SET_SSL_CLIENT_KEY:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
//		AT+CNACT=1,"m2m.aql.net"	//respond with OK <cr-lf><cr-lf> +APP PDP: ACTIVE
				four_g_tx_length = four_g_pack_data("AT+CSSLCFG=\"clientkey\",0,\"client_key.pem\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}

			else if (udp_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CSSLCFG", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
						
#if 0
// test - set SSL context...
					four_g_tx_length = four_g_pack_data("AT+CMQTTSSLCFG=0,0\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
					four_g_send_data(four_g_tx_data,four_g_tx_length);

					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

#endif
					*udp_state = UDP_COMM_START;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
//					udp_error_attempts = 0;							// have passed through this phase of state machine successfully
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;
/// SSL end		
#endif

	case UDP_COMM_START:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
//				udp_reset_flags(COMM__TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
				udp_reset_flags(50,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
				four_g_tx_length = four_g_pack_data("AT+CMQTTSTART\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_PCMQTT)	//RESP_OK)	
				{		
				printf("RSP OK\n");				
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTSTART", "+CMQTT", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*udp_state = UDP_MQTT_ACQUIRE_CLIENT;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					udp_error_attempts = 0;							// have passed through this phase of state machine successfully
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				printf("RSP ERR\n");				
				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;
		
	case UDP_MQTT_ACQUIRE_CLIENT:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
//		AT+CIICR		// respond with OK

// !!! seems to be a 22 byte limit on the client ID?? !!!

#ifdef USE_UDP_MQTT_SSL
//				if (ssl_enable_flag)
				if ((ssl_enable_flag) && (server_ssl_mode[server0_addr_ptr]))
					sprintf((char *)four_g_tx_data,"AT+CMQTTACCQ=0,\"aql_gw_%s\",1\r",mac_bin_str);		// ,1 is for SSL
				else
#endif
					sprintf((char *)four_g_tx_data,"AT+CMQTTACCQ=0,\"aql_gw_%s\"\r",mac_bin_str);		// respond with number <cr-lf> OK
//			sprintf((char *)four_g_tx_data,"AT+CMQTTACCQ=0,\"aql gw 001122334455\"\r");		// respond with number <cr-lf> OK
//			four_g_tx_length = four_g_pack_data("AT+CMQTTACCQ=0,\"client test0\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
				four_g_tx_length = strlen((char *)four_g_tx_data);

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTACCQ", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*udp_state = UDP_MQTT_SET_WILLTOPIC;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;

	case UDP_MQTT_SET_WILLTOPIC:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
				strcpy(udp_will_topic_str, "/gateway_dom/");
				strcat(udp_will_topic_str,mac_bin_str);

				sprintf((char *)four_g_tx_data,"AT+CMQTTWILLTOPIC=0,%d\r",strlen(udp_will_topic_str));		// respond with number <cr-lf> OK

//			four_g_tx_length = four_g_pack_data("AT+CMQTTWILLTOPIC=0,%d\r",(char *)four_g_tx_data,strlen(udp_will_topic_str));		// respond with number <cr-lf> OK
				four_g_tx_length = strlen((char *)four_g_tx_data);		// respond with number <cr-lf> OK

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_R_ARROW)	
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
					xQueueReset(udp_cmd_queue);

//			four_g_tx_length = four_g_pack_data("POST /api/resource/aqlgateway/dom/1234 HTTP/1.1\r\n",(char *)four_g_tx_data);
//			strcpy((char *)four_g_tx_data,"POST /api/resource/aqlgateway/dom/");
//			strcat((char *)four_g_tx_data, "HTTP/1.1\r\n");
//			four_g_tx_length = strlen((char *)four_g_tx_data);


//				four_g_tx_length = four_g_pack_data("WILLTOPIC\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
					four_g_tx_length = strlen(udp_will_topic_str);
					four_g_send_data((uint8_t *)udp_will_topic_str,four_g_tx_length);

					vTaskDelay(100/portTICK_PERIOD_MS);	// delay 500ms

					*udp_state = UDP_MQTT_SET_WILLMSG;	//UDP_SET_APN;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;
		
	case UDP_MQTT_SET_WILLMSG:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
				strcpy(udp_will_msg_str, "disconnect");

				sprintf((char *)four_g_tx_data,"AT+CMQTTWILLMSG=0,%d,1\r",strlen(udp_will_msg_str));		// respond with number <cr-lf> OK

//			four_g_tx_length = four_g_pack_data("AT+CMQTTWILLMSG=0,%d,1\r",(char *)four_g_tx_data,strlen(udp_will_msg_str));		// respond with number <cr-lf> OK
				four_g_tx_length = strlen((char *)four_g_tx_data);		// respond with number <cr-lf> OK

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_R_ARROW)	
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
					four_g_tx_length = strlen(udp_will_msg_str);
					four_g_send_data((uint8_t *)udp_will_msg_str,four_g_tx_length);

					vTaskDelay(100/portTICK_PERIOD_MS);	// delay 500ms

					*udp_state = UDP_MQTT_SET_UTF8_MODE;	//UDP_SET_APN;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					}
				else
					{					
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;
		
	case UDP_MQTT_SET_UTF8_MODE:
//		AT+CIPOPEN="TCP","iot-visualiser.aql.com","80"
// respond with OK <cr-lf><cr-lf> CONNECT OK <cr-lf><cr-lf>
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
				four_g_tx_length = four_g_pack_data("AT+CMQTTCFG=\"checkUTF8\",0,0\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
//				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,30);
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,1,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTCFG", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*udp_state = UDP_MQTT_CONNECT;
					udp_reset_flags(COMM_CONN_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
//					udp_error_attempts = 0;							// have passed through this phase of state machine successfully
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;
		
	case UDP_MQTT_CONNECT:
//		AT+CIPSEND	// start input; terminate with 0x1A;	responds with SEND OK
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				char tmp_connect_str[100];
				
				udp_start_time = esp_timer_get_time();
				pcudp_time = 0;
				
				udp_reset_flags(COMM_CONN_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
//		four_g_tx_length = four_g_pack_data("AT+CMQTTCONNECT=0,\"tcp://test.mosquitto.org:1883\",60,1\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
#ifdef AQL_SERVER
	#ifdef USE_UDP_MQTT_SSL
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

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_PCMQTT)	//RESP_OK)	
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
					*udp_state = UDP_MQTT_CONNECT_CHECK;	//UDP_MQTT_SET_SUBSCRIBE_TOPIC;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					udp_error_attempts = 0;							// have passed through this phase of state machine successfully

					if (pcudp_time)
						{
						ltmp = pcudp_time - udp_start_time;
						if(debug_do(DBG_4G_TXRX))
							printf("Conn time: %lld.%06lldsec\n",ltmp/1000000,ltmp%1000000);
						}
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else if (udp_response_flag == RESP_OK)
				{
				}
			else
				{
				printf("Error! resp flag: %d\n",udp_response_flag);
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}

#if 0
			if (pcudp_time)
				{
				ltmp = pcudp_time - udp_start_time;
				if(debug_do(DBG_4G_TXRX))
					printf("Conn time: %lld.%06lldsec\n",ltmp/1000000,ltmp%1000000);
				}
#endif

			}
		break;
		
	case UDP_MQTT_CONNECT_CHECK:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
				four_g_tx_length = four_g_pack_data("AT+CMQTTCONNECT?\r",(char *)four_g_tx_data);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_OK)	
				{																				
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("", "OK","", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*udp_state = UDP_MQTT_SUBSCRIBE_MSG;	//UDP_MQTT_SET_SUBSCRIBE_TOPIC;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
					
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;

// can set several topics to subscribe to with multiple calls to AT+CMQTTSUBTOPIC;
// then apply all topic subscribes using AT+CMQTTSUB=0; OR
// subscribe to 1 topic and apply immediately with AT+CMQTTSUB=0,len, and input topic at the ">" prompt

//////////////////////
// NOT USED!
//////////////////////

	case UDP_MQTT_SET_SUBSCRIBE_TOPIC:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
				sprintf(udp_subscribe_topic_str,"/gateway_dtm/%s\r",mac_bin_str);
				udp_subcribe_topic_length = strlen(udp_subscribe_topic_str);
				sprintf((char *)four_g_tx_data,"AT+CMQTTSUBTOPIC=0,%d,1\r",udp_subcribe_topic_length);
				four_g_tx_length = strlen((char *)four_g_tx_data);
			
//			four_g_tx_length = four_g_pack_data("AT+CMQTTSUBTOPIC=0,%d,1\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_R_ARROW)	//RESP_OK)	
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
					four_g_send_data((uint8_t *)udp_subscribe_topic_str,udp_subcribe_topic_length);

					four_g_rx_length = 0;
					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

					if(debug_do(DBG_4G_TXRX))
						printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

					i = get_AT_value("", "OK","", eolstr, 0, 0, 0, NULL, 0);
					if (i)
						{
						*udp_state = UDP_MQTT_SUBSCRIBE_MSG;	//TOPIC;
						udp_reset_flags(COMM_LONG_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
						}
					else
						{
						udp_attempts++;
						udp_response_flag = RESP_NONE;
						}		

//			printf("Receive successful!\r\n");
//		*udp_state = UDP_MQTT_SUBSCRIBE_MSG;

					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
			
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;

	case UDP_MQTT_SUBSCRIBE_TOPIC:
			// respond with CLOSED
// needs to be issued after topic data is input by 
// MQTT_SET_SUBSCRIBE_TOPIC:	AT+CMQTTSUBTOPIC

		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_LONG_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
				sprintf(udp_subscribe_topic_str,"/gateway_dtm/%s\r",mac_bin_str);
				udp_subcribe_topic_length = strlen(udp_subscribe_topic_str);
				sprintf((char *)four_g_tx_data,"AT+CMQTTSUBTOPIC=0,%d,1\r",udp_subcribe_topic_length);
				four_g_tx_length = strlen((char *)four_g_tx_data);

//			four_g_tx_length = four_g_pack_data("AT+CMQTTSUBTOPIC=0,9,1\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(udp_cmd_queue);
				four_g_tx_length = four_g_pack_data("/gateway_dom/SUBMSG\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_R_ARROW)	
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

					*udp_state = UDP_MQTT_SUBSCRIBE_MSG;
//			printf("Receive successful!\r\n");
					udp_reset_flags(COMM_LONG_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
//			else if (udp_response_flag == OK)
//				{
//				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}

		break;

#define DBG_Sudp_end_time		// measure time from start of SET_TOPIC to end of PUBLISH
#define DBG_STEP_TIME		// measure time for SET_TOPIC, SET_PAYLOAD, PUBLISH. MUST HAVE DBG_Sudp_end_time defined!

	case UDP_MQTT_SUBSCRIBE_MSG:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_LONG_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
// The string needs a \r to terminate the 
//                                                1234567890123
//				sprintf(udp_subscribe_topic_str,"/gateway_dtm/%s\r",mac_bin_str);
				sprintf(udp_subscribe_topic_str,"/gateway_dtm/%s",mac_bin_str);
//			sprintf(udp_subscribe_topic_str,"dlt\r");
//			sprintf(udp_subscribe_topic_str,"/gateway_dtm/#\r");
//			sprintf(udp_subscribe_topic_str,"testing\r");	// TEST!
//			sprintf(udp_subscribe_topic_str,"#\r");	// TEST!
//			sprintf(udp_subscribe_topic_str,"/gateway_dtm/#\r");	// TEST! # = All topics; WONT WORK! Disconnects...
//			sprintf(udp_subscribe_topic_str,"/gateway_dom/frame/%s\r",mac_bin_str);	// TEST! This is the DATA topic!
				udp_subcribe_topic_length = strlen(udp_subscribe_topic_str);	// - 1;

				sprintf((char *)four_g_tx_data,"AT+CMQTTSUB=0,%d,1\r",udp_subcribe_topic_length);
				four_g_tx_length = strlen((char *)four_g_tx_data);
			
////			four_g_tx_length = four_g_pack_data("AT+CMQTTSUB=0\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);				// send "AT+CMQTTSUB=0,..."
				dbg_count = 3;
#ifdef DBG_Sudp_end_time
				printf("SND SUB: ");
				show_time(DBG,1);
				printf("\n");
#endif
				}
			else if (udp_response_flag == RESP_R_ARROW)
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
					four_g_send_data((uint8_t *)udp_subscribe_topic_str,udp_subcribe_topic_length);	// send topic string

					four_g_rx_length = 0;
					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,2000);					// receive topic string plus "OK"...

					if(debug_do(DBG_4G_TXRX))
						printf("Rx: <\n %s >  [%d] [%d]\r\n",four_g_rx_data, four_g_rx_length,dbg_count);

					i = get_AT_value("", "OK","+CMQTTSUB:", eolstr, 0, 0, 0, NULL, 0);							// check returned topic string
					if (i)
						{
						*udp_state = UDP_MQTT_SET_TOPIC;	//TOPIC;
						udp_reset_flags(COMM_VLONG_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
						}
					else
						{
						udp_attempts++;
						udp_response_flag = RESP_NONE;
						}		

#if 0
//			printf("Receive successful!\r\n");
				*udp_state = UDP_MQTT_SET_TOPIC;
				udp_state_mc_timer = 0;
				udp_attempts = 0;
#endif

					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;
		

#ifdef DBG_STEP_TIME
	#ifndef DBG_Sudp_end_time
	#error "must define DBG_Sudp_end_time too!"
	#endif
#endif	
	case UDP_MQTT_SET_TOPIC:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_VLONG_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
//					printf("MAC str is %s\r\n",mac_bin_str);

#ifdef DBG_Sudp_end_time
				if (debug_do(DBG_MQTT))
					printf("SEND MSG START: SET_TOPIC...\n");
				
//				ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, 5000000));
				udp_init_time = esp_timer_get_time();
				udp_start_time = udp_init_time;
#endif

// show topic and payload to user
				printf("Sending to topic: %s\n",udp_topic_str);
				if (udp_payload_length < 255)
					printf("Payload: [%d]\n%s\r\n",udp_payload_length,udp_payload_str);
				else
					{
					char pstr[256];
					
					strncpy(pstr,udp_payload_str,255);
					pstr[255] = 0x00;
					printf("Payload: [%d]\n%s\r\n",udp_payload_length,pstr);						
					}
#ifdef DBG_TEST
printf("Got here 1\n");
#endif
//### NEED TO clean up the use of the topic string and length...

//		four_g_tx_length = four_g_pack_data("AT+CMQTTTOPIC=0,9\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				sprintf((char *)four_g_tx_data,"AT+CMQTTTOPIC=0,%d\r",strlen(udp_topic_str));

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,strlen((char *)four_g_tx_data));
				}
			else if (udp_response_flag == RESP_R_ARROW)	
				{									
// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect(eolstr[0]);

				if(debug_do(DBG_4G_TXRX))
					printf("Time: %d\n",udp_timeout_timer);
				
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);		// just detected an arrow = the last char sent, so its all arrived; delay = 0

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("", ">", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
//##DIVIDE!!!
					{
					strcat(udp_topic_str,"\r");
//			four_g_tx_length = four_g_pack_data("/dom/100234/frame/node/1\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
					four_g_tx_length = four_g_pack_data(udp_topic_str,(char *)four_g_tx_data);		// respond with number <cr-lf> OK

					four_g_send_data(four_g_tx_data,four_g_tx_length);

					four_g_rx_length = 0;
					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,20);

					if(debug_do(DBG_4G_TXRX))
						printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

					i = get_AT_value("", "OK", "", eolstr, 0, 0, 0, NULL, 0);
					if (i)
						{
						*udp_state = UDP_MQTT_SET_PAYLOAD;
						udp_reset_flags(COMM_LONG_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag

#ifdef DBG_STEP_TIME
						udp_end_time = esp_timer_get_time();
						ltmp = udp_end_time - udp_start_time;
						if (debug_do(DBG_MQTT))
							printf("SET TOPIC END:  TIME: %lld.%06lldsec\n",ltmp/1000000,ltmp%1000000);
						
						udp_start_time = udp_end_time;					// get ready for next reading
#endif
						}
					else
						{
						udp_attempts++;
						udp_response_flag = RESP_NONE;
						}		

//			printf("Receive successful!\r\n");
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,100);
				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);
					
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;
	case UDP_MQTT_SET_PAYLOAD:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_LONG_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!

#ifdef DBG_Sudp_end_time
				if (debug_do(DBG_MQTT))
					printf("SEND MSG START: SET_PAYLOAD...\n");
				
//				ESP_ERROR_CHECK(esp_timer_start_once(oneshot_timer, 5000000));
//				udp_init_time = esp_timer_get_time();
//				udp_start_time = udp_init_time;

				udp_start_time = esp_timer_get_time();
#endif


// *** PAYLOAD can be up to 10240 bytes long!				
				sprintf((char *)four_g_tx_data,"AT+CMQTTPAYLOAD=0,%d\r",udp_payload_length);
				four_g_tx_length = strlen((char *)four_g_tx_data);

#ifdef DBG_TEST
printf("Got here 2\n");
#endif
// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
#ifdef DBG_TEST
printf("Got here 3\n");
#endif
				four_g_send_data(four_g_tx_data,four_g_tx_length);
#ifdef DBG_TEST
printf("Got here 4\n");
#endif
				}
			else if (udp_response_flag == RESP_R_ARROW)	
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
////						four_g_tx_length = udp_payload_length;
////						memcpy((char *)four_g_tx_data, udp_payload_str,four_g_tx_length+1);	// +1 for 0x00 terminator

//				&(char *)four_g_tx_data[four_g_tx_length] = 0x00;
				
//				uart_flush(FOUR_G);
					xQueueReset(udp_cmd_queue);
//						four_g_send_data(four_g_tx_data,four_g_tx_length);
#ifdef DBG_TEST
printf("Got here 7\n");
#endif
// send payload data
					four_g_send_data((uint8_t*)udp_payload_str,udp_payload_length);		// use the mqtt arrays directly, to save duplicating RAM in the four_g_data array...

#ifdef DBG_TEST
printf("Got here 8\n");
#endif
//					vTaskDelay(10 / portTICK_PERIOD_MS);	// 10 milliseconds
//					vTaskDelay((udp_payload_length/5) / portTICK_PERIOD_MS);	// allow 2 x serial tx due to simcom out + ESP in times
//					vTaskDelay(10 / portTICK_PERIOD_MS);	// 10 milliseconds

					esp_task_wdt_reset();		// for long rx delay
					
// receive copy of payload with "CR-LF OK" appended
					four_g_rx_length = 0;
//					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,2500);	//udp_payload_length/8);		// was 40; for 4000 bytes, this is 100msec...
					four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,(10+udp_payload_length/2));		// allow 2 x serial tx due to SIMCOM out + ESP in times

#ifdef DBG_TEST
printf("Got here 9\n");
#endif
					if(debug_do(DBG_4G_TXRX))
						printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

					i = get_AT_value("", "OK", "", eolstr, 0, 0, 0, NULL, 0);
					if (i)
						{
//			printf("Receive successful!\r\n");
						*udp_state = UDP_MQTT_PUBLISH_MSG;
						udp_reset_flags(COMM_LONG_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag

#ifdef DBG_STEP_TIME
						udp_end_time = esp_timer_get_time();
						ltmp = udp_end_time - udp_start_time;
						if (debug_do(DBG_MQTT))
							printf("SET PAYLOAD END:  TIME: %lld.%06lldsec\n",ltmp/1000000,ltmp%1000000);
						
						udp_start_time = udp_end_time;					// get ready for next reading
#endif
						}
					else
						{
						udp_attempts++;
						udp_response_flag = RESP_NONE;
						}		
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}				
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;

	case UDP_MQTT_PUBLISH_MSG:
	// NOTE: once msg is published, the topic string disappears - have to do SET_TOPIC for every PUBLISH...
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_PUBLISH_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!

				udp_start_time = esp_timer_get_time();

//		udp_payload_length = 5;	//strlen(udp_payload_str);
//		sprintf((char *)four_g_tx_data,"AT+CMQTTPUB=0,1,60\r");
//		four_g_tx_length = strlen((char *)four_g_tx_data);

				four_g_tx_length = four_g_pack_data("AT+CMQTTPUB=0,1,60\r",(char *)four_g_tx_data);
			
				xQueueReset(udp_cmd_queue);
// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect(eolstr[0]);

				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_PCMQTT)	//OK)	
				{									
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,20);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTPUB", "+CMQTTPUB:", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*udp_state = UDP_CONNECT_IDLE;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
//					udp_error_attempts = 0;							// have passed through this phase of state machine successfully

#ifdef DBG_Sudp_end_time
					udp_end_time = esp_timer_get_time();
#endif
#ifdef DBG_STEP_TIME
					ltmp = udp_end_time - udp_start_time;
					if (debug_do(DBG_MQTT))
						printf("PUBLISH END:  TIME: %lld.%06lldsec\n",ltmp/1000000,ltmp%1000000);
					
//					udp_start_time = udp_end_time;					// get ready for next reading
#endif
#ifdef DBG_Sudp_end_time
					ltmp = udp_end_time - udp_init_time;
					if (debug_do(DBG_MQTT))
						printf("SEND MSG END: TIME: %lld.%06lldsec\n",ltmp/1000000,ltmp%1000000);
#endif
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else if (udp_response_flag == RESP_OK)
				{
				}
			else
				{
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;

	case UDP_CONNECT_IDLE:
		udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
		udp_error_attempts = 0;
		break;
	
	case UDP_MQTT_UNSUBSCRIBE_TOPIC:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
				sprintf(udp_subscribe_topic_str,"/gateway_dtm/%s\r",mac_bin_str);
				udp_subcribe_topic_length = strlen(udp_subscribe_topic_str);
				sprintf((char *)four_g_tx_data,"AT+CMQTTUNSUB=0,%d,0\r",udp_subcribe_topic_length);
				four_g_tx_length = strlen((char *)four_g_tx_data);

// Set uart pattern detect function.
				pattern_detect_char = set_uart_pattern_detect('>');

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_R_ARROW)	
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
						four_g_send_data((uint8_t *)udp_subscribe_topic_str,udp_subcribe_topic_length);

//			printf("Receive successful!\r\n");
						*udp_state = UDP_CONNECT_IDLE;
//				*udp_state = UDP_MQTT_SERVER_DISCONNECT;
						udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
//					udp_state_mc_timer = 0;
//					udp_attempts = 0;
						}
					else
						{
						udp_attempts++;
						udp_response_flag = RESP_NONE;
						}		
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;

	case UDP_MQTT_SERVER_DISCONNECT:
//		if ((udp_state_mc_timer >= COMM_DELAY) && (udp_attempts < COMM_PHASE_RETRIES))
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
				four_g_tx_length = four_g_pack_data("AT+CMQTTDISC=0,120\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
//			else if (udp_response_flag == RESP_WAIT)	
			else if (udp_response_flag == RESP_OK)	
				{																				
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTDISC", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*udp_state = UDP_MQTT_CLIENT_RELEASE;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;

	case UDP_MQTT_CLIENT_RELEASE:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
				four_g_tx_length = four_g_pack_data("AT+CMQTTREL=0\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_OK)	
				{																				
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTREL", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*udp_state = UDP_MQTT_STOP;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;

	case UDP_MQTT_STOP:
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
				four_g_tx_length = four_g_pack_data("AT+CMQTTSTOP\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_OK)	
				{																				
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CMQTTSTOP", "OK", "", eolstr, 0, 0, 0, NULL, 0);
				if (i)
					{
					*udp_state = UDP_COMM_IDLE;
					udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}
			}
		break;
*/
	case UDP_COMM_IDLE:
		udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
		break;

/*
	case UDP_POWER_DOWN_KEY:
		udp_reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
		*udp_state = UDP_POWER_DOWN_KEY_WAIT;
		break;		

	case UDP_POWER_DOWN_KEY_WAIT:
		if (udp_state_mc_timer >= FOUR_G_POWER_DOWN_KEY_WAIT)
			{
			udp_reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
			*udp_state = UDP_POWER_DOWN_RESP_WAIT;
			}				
		break;
		
	case UDP_POWER_DOWN_RESP_WAIT:
		if (udp_state_mc_timer >= FOUR_G_POWER_DOWN_RESP_WAIT)
			{
			udp_reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
			*udp_state = UDP_POWER_DISABLE;
			}
		break;
		
	case UDP_POWER_DISABLE:
		four_g_power_disable();
		udp_reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
		*udp_state = UDP_POWER_OFF;
		udp_error_attempts = 0;							// have passed through this phase of state machine successfully
		break;	
		
*/
////////////////////////////////////////
	case UDP_MQTT_IDLE_CHECK_SIG_QUALITY:
//		AT+CGDCONT=1, "IP", "AQL"	// respond with OK
		if ((udp_timeout_timer) && (udp_attempts < COMM_PHASE_RETRIES))
			{
			if (udp_response_flag == RESP_NONE)
				{
				udp_reset_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT udp_attempts!
				four_g_tx_length = four_g_pack_data("AT+CSQ\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(udp_cmd_queue);
				udp_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (udp_response_flag == RESP_OK)	
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
						
					*udp_state = UDP_COMMS_INIT_ERROR_END;

					if (j)
						{
						if (((k==99) || (k==199)))
							{
							if (debug_do(DBG_MQTT))
								printf("No network signal detected!\r\n");
//							*udp_state = UDP_ERROR_END;
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
							
							*udp_state = UDP_CONNECT_IDLE;	//UDP_NET_OPEN;	//UDP_SET_APN;
							udp_reset_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
							}
						}
					}
				else
					{
					udp_attempts++;
					udp_response_flag = RESP_NONE;
					}		
				}
//			else if (udp_response_flag == RESP_ERROR)
			else if (udp_response_flag == RESP_WAIT)
				{
				}
			else
				{
				udp_attempts++;
				udp_response_flag = RESP_NONE;
				}			
			}
		
		break;

//////////////////////////////////////////
		
/*
	case UDP_POWER_UP_ERROR_END:
		udp_reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
		*udp_state = UDP_POWER_EN;					// retry power-up
		break;

	case UDP_CHECKS_ERROR_END:
		udp_reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
		*udp_state = UDP_GET_MODULETYPE;				// retry checks
		break;
*/
	case UDP_COMMS_INIT_ERROR_END:
		udp_reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
		*udp_state = UDP_COMM_INIT_2;					// retry comm init
		break;
/*
	case UDP_CONNECT_ERROR_END:
		udp_reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
		*udp_state = UDP_MQTT_ACQUIRE_CLIENT;		// retry connect
		break;
*/
	case UDP_SEND_RCV_ERROR_END:
		udp_reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
		*udp_state = UDP_COMM_IDLE_2;	// SUBSCRIBE_TOPIC;	// retry send \ rcv
		break;

/*
	case UDP_DISC_PWRDOWN_ERROR_END:
		udp_reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
		*udp_state = UDP_POWER_DOWN_KEY;				// goto power off state
		break;
*/
	case UDP_END:
		udp_reset_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, udp_attempts, response flag
		break;
	default:
		break;
	}


//	if (udp_attempts)
//		printf("[ %s ]  udp_attempts: %d\n",udp_state_str[udp_last_error_state],udp_attempts);
	if ((udp_attempts) && (udp_attempts != prev_udp_attempts))
		printf("[ UDP_%s ]  udp_attempts: %d\n",udp_state_str[*udp_state],udp_attempts);
	
	if (udp_timeout_timer == 0)
		{
		printf("UDP Timeout [%d.%d sec] [rsp: %d %s] [eol: %d %s]\n",
				udp_timeout_val/10,udp_timeout_val%10,udp_response_flag,four_g_resps[udp_response_flag],udp_eol_response_flag,four_g_resps[udp_eol_response_flag]);

		four_g_rx_length = 0;
		four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,100);
		printf("TO Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

// reset uart pattern detect function.
		pattern_detect_char = set_uart_pattern_detect(eolstr[0]);
		
		udp_response_flag = RESP_NONE;		// try command again...
		udp_timeout_timer = 	udp_timeout_val;	// restore previous timeout value and try again...

		xQueueReset(udp_cmd_queue);

		udp_attempts++;
		}
		
	if (udp_attempts >= COMM_PHASE_RETRIES)
		{
		udp_last_error_state = *udp_state;
		printf("UDP comm fail at %d [ %s ] - too many retries [%d ]  [errors: %d]\r\n",udp_last_error_state,udp_state_str[udp_last_error_state],udp_attempts,udp_error_attempts);

		four_g_rx_length = 0;
		four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,100);
		printf("RT Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

		udp_reset_flags(udp_timeout_val,1);		// with last timeout val to be used
//		udp_attempts = 0;
		udp_error_attempts++;

		if (udp_error_attempts < ERROR_RETRIES)
			{
/*
			if (*udp_state <= UDP_POWER_ON_RESP_WAIT)
				{
				*udp_state = UDP_POWER_UP_ERROR_END;
				}
			else if ((*udp_state > UDP_POWER_ON_RESP_WAIT) && (*udp_state <= UDP_NO_COMM_IDLE))
				{
				*udp_state = UDP_CHECKS_ERROR_END;
				}
			else 
*/
			if ((*udp_state > UDP_NO_COMM_IDLE) && (*udp_state <= UDP_SET_RESPONSE))
				{
				*udp_state = UDP_COMM_INIT_2;
				}
			else if ((*udp_state > UDP_SET_RESPONSE) && (*udp_state < UDP_SOCKET_CLOSE))
				{
				*udp_state = UDP_SEND_RCV_ERROR_END;
				}
/*
			else if (*udp_state > UDP_CONNECT_IDLE)
				{
				*udp_state = UDP_DISC_PWRDOWN_ERROR_END;
				}

			*udp_state = UDP_COMM_INIT_2;
*/
			}


// if too many error retries, downpower the SIMVCOM module and start from the beginning...
		if (udp_error_attempts > UDP_MAX_ERROR_RETRIES)
			{
#ifdef SIMCOM_PROGRAMMING_MODE
//			printf("SIMCOM comms fail - idling with power ON\r\n");
//			*udp_state = UDP_NO_COMM_IDLE;				// stop coms udp_attempts but leave SIMCOM powered up...
// keep tryinglast step with power on (for debug)			
			udp_attempts = 0;
			prev_udp_attempts = 0;
			udp_error_attempts = 0;

#else
			*udp_state = UDP_NO_COMM_IDLE;
#endif
			}

		simcom_busy_flag = simcom_busy_flag & (~SIMCOM_BUSY_UDP);
		}
	prev_udp_attempts = udp_attempts;


}


void udp_reset_flags(unsigned int timeout, unsigned char clear_udp_attempts)
{
//udp_state_mc_timer = 0;
udp_timeout_timer = timeout;
udp_timeout_val = timeout;
udp_response_flag = RESP_NONE;
udp_eol_response_flag = RESP_NONE;
if (clear_udp_attempts)
	{
	udp_attempts = 0;	
	prev_udp_attempts = 0;
	}

// Set uart pattern detect function.
//## MAY NEED ATTENTION!
//pattern_detect_char = set_uart_pattern_detect(eolstr[0]);

}

unsigned char udp_check_topic_str(void)
{
	// returns 1 if new topic set; 0 if same as old topic
unsigned char ret;

if (!strcmp(udp_topic_str,udp_prev_topic_str))	// if strings the same
	ret = 1;
else
	{
	strcpy(udp_prev_topic_str,udp_topic_str);	// set prev topic 
	ret = 0;
	}
	
return ret;
	
}

void print_udp_state(unsigned char chan)
{
//dbgprintf(chan,"UDP State:  [%03d] < UDP_%s >    UDP timer: %d\r\n",udp_state,udp_state_str[udp_state],udp_state_mc_timer);
dbgprintf(chan,"UDP SM State    :  [%03d] < UDP_%s >\r\n",udp_state,udp_state_str[udp_state]);
}
