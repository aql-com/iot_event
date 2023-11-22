////////////////////////////////////////////
//
// MAC_Rpt_four_g_simcom.c
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
#include "MAC_Rpt_four_g_simcom.h"

//#include "ca_cert.h"
//#include "client_cert.h"
//#include "client_key.h"

extern uint8_t four_g_tx_data[FOUR_G_TX_BUFSIZE];
extern uint8_t four_g_rx_data[FOUR_G_RX_BUFSIZE];
extern unsigned int four_g_tx_length;
extern unsigned int four_g_rx_length;
extern unsigned char simcom_response_flag, simcom_eol_response_flag;
extern unsigned char simcom_sm_timer_flag;

extern unsigned char four_g_rst_flag, four_g_pwr_flag, four_g_connected_flag;

extern unsigned char simcom_state, prev_simcom_state;
//extern unsigned char simcom_state_str[][];
extern unsigned int simcom_state_mc_timer;
extern unsigned int four_g_timeout_timer;
extern unsigned int four_g_timeout_val;

extern unsigned int simcom_timeout_timer;
extern unsigned int simcom_timeout_val;

extern char mac_bin_str[];
extern char modtype_str[];
extern char fwver_str[];
extern char imei_str[];
extern char simnum_str[];
extern char ipaddr_str[];

extern const char simcom_state_str[SIMCOM_NUM_STATES][32];

extern unsigned int supply_voltage;
extern uint32_t voltage;
extern unsigned char temperature;

extern char gps_lat_str[];
extern char gps_lon_str[];
extern unsigned int gps_spd,lora_radio_freq;
extern unsigned char cell_csq;

unsigned char simcom_attempts, prev_simcom_attempts;
unsigned char simcom_error_attempts;

extern unsigned char simcom_powerdown_attempts;

extern char module_mac_str[];

/*
char mqtt_will_topic_str[80];
char mqtt_will_msg_str[40];
char mqtt_topic_str[80];
char mqtt_prev_topic_str[80];
unsigned int mqtt_topic_length;
char mqtt_payload_str[MQTT_PAYLOAD_SIZE];		//256...
unsigned int mqtt_payload_length;
char mqtt_subscribe_topic_str[80];
unsigned int mqtt_subcribe_topic_length;
*/

char simcom_gps_str[80];

extern signed char cell_rssi;

extern QueueHandle_t mqtt_cmd_queue;

extern unsigned char x100msec, secs, mins, hrs;

extern unsigned char ssl_enable_flag;

extern unsigned char server_ssl_mode[4];

extern unsigned char server0_addr_ptr;


extern char pattern_detect_char;

extern unsigned char eolstr[10];

unsigned char simcom_last_error_state;

//int64_t init_time = 0;
//int64_t start_time = 0;
//int64_t end_time = 0;
//int64_t final_time = 0;

//extern int64_t pcmqtt_time;

extern unsigned char server0_addr_ptr, server1_addr_ptr;

extern unsigned int dbg_count;

extern unsigned char simcom_comms_flag;

//extern esp_timer_handle_t oneshot_timer;


// function prototypes
void reset_simcom_flags(unsigned int timeout, unsigned char clear_simcom_attempts);

//static void oneshot_timer_callback(void* arg);

// code
void four_g_simcom_state_machine(unsigned char *simcom_state,unsigned char *prev_simcom_state)
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

if (*simcom_state < SIMCOM_READY_IDLE)
#ifdef NETWORK_LED
	set_led(NETWORK_LED, DEVICE_OFF);
#else
	set_led(1, DEVICE_OFF);
#endif

#ifdef DBG_TEST
if (*simcom_state != *prev_simcom_state)
#else
if ((debug_do(DBG_MQTT)) && (*simcom_state != *prev_simcom_state))
#endif
	{
	printf("\r\nSIMCOM State: [%03d] SIMCOM_%s\r\n",*simcom_state, simcom_state_str[*simcom_state]);
	}

*prev_simcom_state = *simcom_state;
eolstr[0] = 0x0A;
eolstr[1] = 0x00;

// system rests in SIMCOM_POWER_OFF state
// manual change to SIMCOM_POWER_EN starts 4G power up 
// system rests in SIMCOM_NO_COMM_IDLE state
// manual change to SIMCOM_COMM_INIT starts 4G connect
// system rests in SIMCOM_CONNECT_IDLE state
switch(*simcom_state)
	{
	case SIMCOM_POWER_OFF:
		reset_simcom_flags(COMM_TIMEOUT_DEFEAT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
		simcom_error_attempts = 0;
		simcom_last_error_state = SIMCOM_NO_ERROR;
		break;
	case SIMCOM_POWER_EN:
		four_g_power_en();
		reset_simcom_flags(COMM_TIMEOUT_DEFEAT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
		*simcom_state = SIMCOM_POWER_EN_WAIT;
		break;		
	case SIMCOM_POWER_EN_WAIT:
		if (simcom_state_mc_timer >= SIMCOM_POWER_EN_WAIT_TIME)
			{
			reset_simcom_flags(COMM_TIMEOUT_DEFEAT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
			*simcom_state = SIMCOM_RESET;
			}
		break;
	case SIMCOM_RESET:
		four_g_hold_in_reset();
		reset_simcom_flags(COMM_TIMEOUT_DEFEAT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
		*simcom_state = SIMCOM_RESET_WAIT;
		break;
	case SIMCOM_RESET_WAIT:
		if (simcom_state_mc_timer >= SIMCOM_RESET_WAIT_TIME)
			{
			four_g_release_from_reset();
			reset_simcom_flags(COMM_TIMEOUT_DEFEAT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
			*simcom_state = SIMCOM_POWER_ON_KEY;
			}
		break;
	case SIMCOM_POWER_ON_KEY:
		four_g_power_key_on();
		reset_simcom_flags(COMM_TIMEOUT_DEFEAT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
		*simcom_state = SIMCOM_POWER_ON_KEY_WAIT;
		break;
	case SIMCOM_POWER_ON_KEY_WAIT:
		if (simcom_state_mc_timer >= SIMCOM_POWER_ON_WAIT_TIME)
			{
			four_g_power_key_off();
			reset_simcom_flags(COMM_TIMEOUT_DEFEAT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
			*simcom_state = SIMCOM_POWER_ON_RESP_WAIT;
			}
		break;
	case SIMCOM_POWER_ON_RESP_WAIT:
		if (simcom_state_mc_timer < SIMCOM_POWER_ON_RESP_WAIT_TIME)
			{
			if (simcom_state_mc_timer%10 == 0)
				{
					if (simcom_sm_timer_flag)
					{
					if (debug_do(DBG_4G_TXRX))
						{
						printf("%d",simcom_state_mc_timer/10);
						if (gpio_get_level(STATUS))					// dont exit early, as SIMCOM needs the full 16 sec to establish network handshake...
							printf(" + ");
						else
							printf(" - ");
						}
					simcom_sm_timer_flag = 0;
					}
/*
			if (gpio_get_level(STATUS))					// dont exit early, as SIMCOM needs the full 16 sec to establish network handshake...
				{
				printf(" + ");
//				printf("STATUS line is set...\r\n");
//				*simcom_state = SIMCOM_GET_MODULETYPE;
//				reset_simcom_flags(COMM_TIMEOUT);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
//				simcom_state_mc_timer = 0;
//				simcom_error_attempts = 0;							// have passed through this phase of state machine successfully
				}
			else
				printf(" - ");
*/					
				}
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

#ifndef DEFEAT_SIMCOM_STATUS_LINE
			if (gpio_get_level(STATUS)==0)	// ie STATUS line not set...
				{
				simcom_attempts++;
// wait for timeout...					
				}	
			else
#endif
				{
				
//#ifndef SIMCOM_PROGRAMMING_MODE
				*simcom_state = SIMCOM_SET_FLOWCTRL;	// SIMCOM_GPS_START;
				reset_simcom_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				simcom_error_attempts = 0;							// have passed through this phase of state machine successfully
//#endif
				}
			}
				
		break;
	
	case SIMCOM_SET_FLOWCTRL:
		if ((simcom_timeout_timer) && (simcom_attempts < COMM_PHASE_RETRIES))
			{
			if (simcom_response_flag == RESP_NONE)
				{
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+IFC=2,2\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				simcom_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (simcom_response_flag == RESP_OK)	
				{
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

					
				*simcom_state = SIMCOM_GET_MODULETYPE;
				reset_simcom_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
// good response from simcom - SIMCOM must be present...
				simcom_comms_flag = 1;
				}
//			else if (simcom_response_flag == RESP_ERROR)
			else if (simcom_response_flag == RESP_WAIT)
				{
//					printf("Wait...\n");
				}
			else
				{
				printf("Resp Error: %d\n",simcom_response_flag);
				simcom_attempts++;
				simcom_response_flag = RESP_NONE;
				}
			}
		
		break;

	case SIMCOM_GET_MODULETYPE:
		if ((simcom_timeout_timer) && (simcom_attempts < COMM_PHASE_RETRIES))
			{
			if (simcom_response_flag == RESP_NONE)
				{
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CGMM\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				simcom_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (simcom_response_flag == RESP_OK)	
				{
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

// save module type
				strcpy(modtype_str,(char *)&four_g_rx_data[17]);
				i = 0;
// remove trailing "OK" and CR-LFs...
				while (modtype_str[i] != 0x0D) 
					{i++;}
				modtype_str[i] = 0x00;
					
				*simcom_state = SIMCOM_GET_FWVERSION;
				reset_simcom_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				}
//			else if (simcom_response_flag == RESP_ERROR)
			else if (simcom_response_flag == RESP_WAIT)
				{
//					printf("Wait...\n");
				}
			else
				{
				printf("Resp Error: %d\n",simcom_response_flag);
				simcom_attempts++;
				simcom_response_flag = RESP_NONE;
				}
			}
		break;

	case SIMCOM_GET_FWVERSION:
// Known working Firmware versions:
// LE11B12SIM7600M22_HB
// LE11B13SIM7600M22

// non-working versions:
// LE11B08SIM7600M22		// CSSLCFG fails...

// LE20B04SIM7600M22	// ???
// LE11B04SIM7600M21-A
		if ((simcom_timeout_timer) && (simcom_attempts < COMM_PHASE_RETRIES))
			{
			if (simcom_response_flag == RESP_NONE)
				{
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CGMR\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				simcom_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (simcom_response_flag == RESP_OK)	
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
				
//				*simcom_state = SIMCOM_GPS_CHECK;
				*simcom_state = SIMCOM_GET_SIM_IMEI;
				reset_simcom_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				}
//			else if (simcom_response_flag == RESP_ERROR)
			else if (simcom_response_flag == RESP_WAIT)
				{
				}
			else
				{
				simcom_attempts++;
				simcom_response_flag = RESP_NONE;
				}

			}
		break;

	case SIMCOM_GET_SIM_MAC:
		if ((simcom_timeout_timer) && (simcom_attempts < COMM_PHASE_RETRIES))
			{
			if (simcom_response_flag == RESP_NONE)
				{
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CWMACADDR?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				simcom_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (simcom_response_flag == RESP_OK)	
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
					*simcom_state = SIMCOM_GET_SIM_IMEI;
					reset_simcom_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
					}
				else
					{
					simcom_attempts++;
					simcom_response_flag = RESP_NONE;
					}
				}
//			else if (simcom_response_flag == RESP_ERROR)
			else if (simcom_response_flag == RESP_WAIT)
				{
				}
			else
				{
				simcom_attempts++;
				simcom_response_flag = RESP_NONE;
				}
			}
		break;
		
	case SIMCOM_GET_SIM_IMEI:
		if ((simcom_timeout_timer) && (simcom_attempts < COMM_PHASE_RETRIES))
			{
			if (simcom_response_flag == RESP_NONE)
				{
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+GSN\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				simcom_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (simcom_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+GSN", "OK", "", eolstr, 1, 0, 0x0D, imei_str, 20);
				if (i)
					{
					*simcom_state = SIMCOM_GET_SIM_NUM;
					reset_simcom_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
//					simcom_state_mc_timer = 0;
//					simcom_attempts = 0;
					}
				else
					{
					simcom_attempts++;
					simcom_response_flag = RESP_NONE;
					}
				}
//			else if (simcom_response_flag == RESP_ERROR)
			else if (simcom_response_flag == RESP_WAIT)
				{
				}
			else
				{
				simcom_attempts++;
				simcom_response_flag = RESP_NONE;
				}
			}
		break;

	case SIMCOM_GET_SIM_NUM:
//		AT+CCID		// respond with number <cr-lf> OK; if number ends in F, strip...
		if ((simcom_timeout_timer) && (simcom_attempts < COMM_PHASE_RETRIES))
			{
			if (simcom_response_flag == RESP_NONE)
				{
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CICCID\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				simcom_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (simcom_response_flag == RESP_OK)	
				{															
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("AT+CICCID", "OK", "", eolstr, 1, 8, 0x0D, simnum_str, 22);
				if (i)
					{
//					*simcom_state = SIMCOM_NO_COMM_IDLE;
//##					*simcom_state = SIMCOM_GPS_CHECK;
					reset_simcom_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag

#ifndef SIMCOM_PROGRAMMING_MODE
					if(debug_do(DBG_MQTT))
#endif
						{
						printf("\nFW  version: %s\n",fwver_str);
						printf("IMEI number: %s\n",imei_str);
						printf("SIM  number: %s\n\n",simnum_str);
						}
#ifdef SIMCOM_PROGRAMMING_MODE
					*simcom_state = SIMCOM_NO_COMM_IDLE;
#else					
//					*simcom_state = SIMCOM_GPS_CHECK;	
					*simcom_state = SIMCOM_GET_NWK_SETTINGS;	
#endif
					}
				else
					{
					simcom_attempts++;
					simcom_response_flag = RESP_NONE;
					}		
				}
//			else if (simcom_response_flag == RESP_ERROR)
			else if (simcom_response_flag == RESP_WAIT)
				{
//					printf("waiting...\n");
				}
			else
				{
				printf("Resp Error: %d\n",simcom_response_flag);
				simcom_attempts++;
				simcom_response_flag = RESP_NONE;
				}				
			}
		break;

	case SIMCOM_GET_NWK_SETTINGS:
#if 1
// AT+CSPN		get service provider name from SIM
// AT+CREG		network registration (already using this?) 			MAKE SURE DISABLING UNSOLICITED CODE!
// AT+COPS		operator selection
// AT+CPOL		preferred operator list								**
// AT+COPN		read operator names
// AT+CNMP		set preferred mode									**
// AT+CNBP		set preferred band									**
// AT+CNAOP		operator order of preference (CDMA\GSM\LTE,etc)
// AT+CPSI		show UE sys info									**
// AT+CNSMOD	Show net system mode
// AT+CTZU		auto time and zone update for RTC
// AT+CTZR		time and zone change reporting
// AT+CCLK		real Time Clock control								**		

#if 0
// AT+CPOL
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CPOL?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_send_data(four_g_tx_data,four_g_tx_length);

				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

//				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

/*
				i = get_AT_value("AT+CICCID", "OK", "", eolstr, 1, 8, 0x0D, simnum_str, 22);
				if (i)
					{
					reset_simcom_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
					}
*/

// AT+CNMP
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CNMP?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_send_data(four_g_tx_data,four_g_tx_length);

				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

//				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

// AT+CNBP
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CNBP?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_send_data(four_g_tx_data,four_g_tx_length);

				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

//				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

// AT+CPSI
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CPSI?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_send_data(four_g_tx_data,four_g_tx_length);

				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

//				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

// AT+CCLK
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CCLK?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_send_data(four_g_tx_data,four_g_tx_length);

				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

//				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);
#else
	simcom_net_check("SIMCOM start",dbg_4g_flag);

#endif


#endif
		reset_simcom_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
		*simcom_state = SIMCOM_GPS_CHECK;	
	
		break;
	
	case SIMCOM_GPS_CHECK:
		if ((simcom_timeout_timer) && (simcom_attempts < COMM_PHASE_RETRIES))
			{
			if (simcom_response_flag == RESP_NONE)
				{
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CGPS?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				simcom_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (simcom_response_flag == RESP_OK)	
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
						*simcom_state = SIMCOM_GPS_INFO;
					else
						*simcom_state = SIMCOM_GPS_START;

					reset_simcom_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
					}
				else
					{
					simcom_attempts++;
					simcom_response_flag = RESP_NONE;
					}
				}
//			else if (simcom_response_flag == RESP_ERROR)
			else if (simcom_response_flag == RESP_WAIT)
				{
				}
			else
				{
				simcom_attempts++;
				simcom_response_flag = RESP_NONE;
				}				

			}
		break;
		
	case SIMCOM_GPS_START:
// AT+CGPS=1,1
		if ((simcom_timeout_timer) && (simcom_attempts < COMM_PHASE_RETRIES))
			{
			if (simcom_response_flag == RESP_NONE)
				{
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CGPS=1,1\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				simcom_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (simcom_response_flag == RESP_OK)	
				{												
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("", "OK", "", eolstr, 1, 0, 0x0D, NULL, 0);
	//			printf("MAC addr: %s\r\n",module_mac_str);
				if (i)
					{
					*simcom_state = SIMCOM_GPS_INFO;
					reset_simcom_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
					}
				else
					{
					simcom_attempts++;
					simcom_response_flag = RESP_NONE;
					}
				}
//			else if (simcom_response_flag == RESP_ERROR)
			else if (simcom_response_flag == RESP_WAIT)
				{
				}
			else
				{
				simcom_attempts++;
				simcom_response_flag = RESP_NONE;
				}				

			}

		break;

	case SIMCOM_GPS_INFO:
// AT+CGPS=1,1
		if ((simcom_timeout_timer) && (simcom_attempts < COMM_PHASE_RETRIES))
			{
			if (simcom_response_flag == RESP_NONE)
				{
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CGPSINFO\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				simcom_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);				
				}
			else if (simcom_response_flag == RESP_OK)	
				{												
				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				i = get_AT_value("", "OK", "", eolstr, 1, 0, 0x0D, simcom_gps_str, 80);

				if(debug_do(DBG_4G_TXRX))
					printf("GPS: %s\r\n",simcom_gps_str);
					
				if (i)
					{
					char tmp_str[80];
					char time_str[10];
					unsigned int n;
					
					j = 11;	
					j = j + 1 + get_str(&simcom_gps_str[j], ",", gps_lat_str,dbg_4g_flag);	// latitude
					j = j + 1 + get_str(&simcom_gps_str[j], ",", tmp_str,dbg_4g_flag);		// N/S
					strcat(gps_lat_str,tmp_str);
					j = j + 1 + get_str(&simcom_gps_str[j], ",", gps_lon_str,dbg_4g_flag);	// longtitude
					j = j + 1 + get_str(&simcom_gps_str[j], ",", tmp_str,dbg_4g_flag);		// E/W
					strcat(gps_lon_str,tmp_str);
					j = j + 1 + get_str(&simcom_gps_str[j], ",", tmp_str,dbg_4g_flag);		// date
					j = j + 1 + get_str(&simcom_gps_str[j], ",", time_str,dbg_4g_flag);	// time
					j = j + 1 + get_str(&simcom_gps_str[j], ",", tmp_str,dbg_4g_flag);		// altitude
					j = j + 1 + get_str(&simcom_gps_str[j], ",", tmp_str,dbg_4g_flag);		// speed
					gps_spd = atoi(tmp_str);
					j = j + 1 + get_str(&simcom_gps_str[j], ",", tmp_str,dbg_4g_flag);		// course

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
						
//					*simcom_state = SIMCOM_GET_SIM_IMEI;	// SIMCOM_NO_COMM_IDLE;						
					*simcom_state = SIMCOM_READY_IDLE;						
					reset_simcom_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
					simcom_error_attempts = 0;							// have passed through this phase of state machine successfully

//					if(debug_do(DBG_4G_TXRX))
						printf("Reached SIMCOM_READY_IDLE\r\n");
					}
				else
					{
					simcom_attempts++;
					simcom_response_flag = RESP_NONE;
					}
				}
//			else if (simcom_response_flag == RESP_ERROR)
			else if (simcom_response_flag == RESP_WAIT)
				{
				}
			else
				{
				simcom_attempts++;
				simcom_response_flag = RESP_NONE;
				}
			}

		break;

	case SIMCOM_READY_IDLE:
//		printf("Reached SIMCOM_READY_IDLE\r\n");
		reset_simcom_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag

		break;
		
	case SIMCOM_POWER_DOWN_KEY:
		reset_simcom_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
		*simcom_state = SIMCOM_POWER_DOWN_KEY_WAIT;
		break;		

	case SIMCOM_POWER_DOWN_KEY_WAIT:
		if (simcom_state_mc_timer >= SIMCOM_POWER_DOWN_KEY_WAIT_TIME)
			{
			reset_simcom_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
			*simcom_state = SIMCOM_POWER_DOWN_RESP_WAIT;
			}				
		break;
		
	case SIMCOM_POWER_DOWN_RESP_WAIT:
		if (simcom_state_mc_timer >= SIMCOM_POWER_DOWN_RESP_WAIT_TIME)
			{
			reset_simcom_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
			*simcom_state = SIMCOM_POWER_DISABLE;
			}
		break;
		
	case SIMCOM_POWER_DISABLE:
		four_g_power_disable();
		reset_simcom_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
		*simcom_state = SIMCOM_POWER_OFF;
		simcom_error_attempts = 0;							// have passed through this phase of state machine successfully
		break;	
////////////////////////////////////////
	case SIMCOM_MQTT_IDLE_CHECK_SIG_QUALITY:
//		AT+CGDCONT=1, "IP", "AQL"	// respond with OK
		if ((simcom_timeout_timer) && (simcom_attempts < COMM_PHASE_RETRIES))
			{
			if (simcom_response_flag == RESP_NONE)
				{
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, response flag, NOT simcom_attempts!
				four_g_tx_length = four_g_pack_data("AT+CSQ\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				simcom_response_flag = RESP_WAIT;
				four_g_send_data(four_g_tx_data,four_g_tx_length);
				}
			else if (simcom_response_flag == RESP_OK)	
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
						
					*simcom_state = SIMCOM_COMMS_INIT_ERROR_END;

					if (j)
						{
						if (((k==99) || (k==199)))
							{
							if (debug_do(DBG_MQTT))
								printf("No network signal detected!\r\n");
//							*simcom_state = SIMCOM_ERROR_END;
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
							
							*simcom_state = SIMCOM_READY_IDLE;	//SIMCOM_NET_OPEN;	//SIMCOM_SET_APN;
							reset_simcom_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
							}
						}
					}
				else
					{
					simcom_attempts++;
					simcom_response_flag = RESP_NONE;
					}		
				}
//			else if (simcom_response_flag == RESP_ERROR)
			else if (simcom_response_flag == RESP_WAIT)
				{
				}
			else
				{
				simcom_attempts++;
				simcom_response_flag = RESP_NONE;
				}			
			}
		
		break;

//////////////////////////////////////////
		
	case SIMCOM_POWER_UP_ERROR_END:
		reset_simcom_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
		*simcom_state = SIMCOM_POWER_EN;					// retry power-up
		break;

	case SIMCOM_CHECKS_ERROR_END:
		reset_simcom_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, attempts, response flag
		*simcom_state = SIMCOM_GET_MODULETYPE;				// retry checks
		break;

	case SIMCOM_DISC_PWRDOWN_ERROR_END:
		reset_simcom_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
		*simcom_state = SIMCOM_POWER_DOWN_KEY;				// goto power off state
		break;

	case SIMCOM_END:
		reset_simcom_flags(COMM_TIMEOUT_DEFEAT,1);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
		break;
	default:
		break;
	}


//	if (simcom_attempts)
//		printf("[ %s ]  simcom_attempts: %d\n",simcom_state_str[simcom_last_error_state],simcom_attempts);
	if ((simcom_attempts) && (simcom_attempts != prev_simcom_attempts))
		printf("[ %s ]  simcom_attempts: %d\n",simcom_state_str[*simcom_state],simcom_attempts);
	
	if (simcom_timeout_timer == 0)
		{
		printf("SIMCOM Timeout [%d.%d sec] [rsp: %d %s] [eol: %d %s]\n",
				simcom_timeout_val/10,simcom_timeout_val%10,simcom_response_flag,four_g_resps[simcom_response_flag],simcom_eol_response_flag,four_g_resps[simcom_eol_response_flag]);

		four_g_rx_length = 0;
		four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,100);
		printf("TO Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

// reset uart pattern detect function.
		pattern_detect_char = set_uart_pattern_detect(eolstr[0]);
		
		simcom_response_flag = RESP_NONE;		// try command again...
		simcom_timeout_timer = 	simcom_timeout_val;	// restore previous timeout value and try again...

		xQueueReset(mqtt_cmd_queue);

		simcom_attempts++;
		}
		
	if (simcom_attempts >= COMM_PHASE_RETRIES)
		{
		simcom_last_error_state = *simcom_state;
		printf("SIMCOM comm fail at %d [ %s ] - too many retries [%d ]  [errors: %d]\r\n",simcom_last_error_state,simcom_state_str[simcom_last_error_state],simcom_attempts,simcom_error_attempts);

		four_g_rx_length = 0;
		four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,100);
		printf("RT Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

		reset_simcom_flags(simcom_timeout_val,1);		// with last timeout val to be used
//		simcom_attempts = 0;
		simcom_error_attempts++;

		if (simcom_error_attempts < ERROR_RETRIES)		// retries on a particular state
			{
			if (*simcom_state <= SIMCOM_POWER_ON_RESP_WAIT)
				{
				*simcom_state = SIMCOM_POWER_UP_ERROR_END;
				}
			else if ((*simcom_state > SIMCOM_POWER_ON_RESP_WAIT) && (*simcom_state <= SIMCOM_READY_IDLE))
				{
				*simcom_state = SIMCOM_CHECKS_ERROR_END;
				}
			else if (*simcom_state > SIMCOM_READY_IDLE)
				{
				*simcom_state = SIMCOM_DISC_PWRDOWN_ERROR_END;
				}
			}

// if too many error retries, downpower the SIMVCOM module and start from the beginning...
		if (simcom_error_attempts > SIMCOM_MAX_ERROR_RETRIES)	// retries after dropping back to previous block starting point
			{
#ifdef SIMCOM_PROGRAMMING_MODE
//			printf("SIMCOM comms fail - idling with power ON\r\n");
//			*simcom_state = SIMCOM_NO_COMM_IDLE;				// stop coms simcom_attempts but leave SIMCOM powered up...
// keep tryinglast step with power on (for debug)			
			simcom_attempts = 0;
			prev_simcom_attempts = 0;
			simcom_error_attempts = 0;

#else
//			if (*simcom_state == SIMCOM_GET_MODULETYPE)
//				*simcom_state = SIMCOM_NONE_FOUND;
//			else
			*simcom_state = SIMCOM_POWER_DOWN_KEY;
			simcom_powerdown_attempts++;
#endif
			}
			
		if (simcom_powerdown_attempts > SIMCOM_MAX_POWERDOWN_RETRIES)	// retries after powerdown
			{
			if (simcom_comms_flag == 0)
				{
				printf("** NO SIMCOM MODULE DETECTED!\n");
				*simcom_state = SIMCOM_NONE_FOUND;
				}

			}
		}
	prev_simcom_attempts = simcom_attempts;

}


void reset_simcom_flags(unsigned int timeout, unsigned char clear_simcom_attempts)
{
simcom_state_mc_timer = 0;
simcom_timeout_timer = timeout;
simcom_timeout_val = timeout;
simcom_response_flag = RESP_NONE;
simcom_eol_response_flag = RESP_NONE;
if (clear_simcom_attempts)
	{
	simcom_attempts = 0;	
	prev_simcom_attempts = 0;
	}

// Set uart pattern detect function.
//pattern_detect_char = set_uart_pattern_detect(eolstr[0]);

}

void print_simcom_state(unsigned char chan)
{
dbgprintf(chan,"SIMCOM State    :  [%03d] < SIMCOM_%s >    SIMCOM timer: %d\r\n",simcom_state,simcom_state_str[simcom_state],simcom_state_mc_timer);
}


void simcom_busy_change(unsigned char *simcom_busy_flag, unsigned char *prev_simcom_busy_flag, char *str)
{
if ((debug_do(DBG_SIMCOM_BUSY_FLAGS)) && (*simcom_busy_flag != *prev_simcom_busy_flag))
//if (((debug_do(DBG_MQTT)) | (debug_do(DBG_UDP))) && (*simcom_busy_flag != *prev_simcom_busy_flag))
//if ((debug_do(DBG_MQTT)) | (debug_do(DBG_UDP)))
//{
//printf("BUSY CHECK: %d %d  ",*simcom_busy_flag, *prev_simcom_busy_flag);
//	if (*simcom_busy_flag != *prev_simcom_busy_flag)
	{
//		printf("DIFF! ");
	printf("\r\n%s:  BUSY State: [%03d] SIMCOM_BUSY_%s\r\n",str, *simcom_busy_flag, simcom_busy_state_str[*simcom_busy_flag]);
	}
//	else
//printf("\n");

if (*simcom_busy_flag == (SIMCOM_BUSY_MQTT + SIMCOM_BUSY_UDP))
	printf("SIMCOM BUSY Error!\n");

//}

*prev_simcom_busy_flag = *simcom_busy_flag;


}

void simcom_net_check(char *str, unsigned char dbg_4g_flag)
{
unsigned char i,j,k,p,val;
char c;
char tmpstr[80];
unsigned char binstr[4];	
char* ptr;

				printf("Net check @ %s:\n",str);
// AT+CPOL
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CPOL?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_send_data(four_g_tx_data,four_g_tx_length);

				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

//				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

/*
				i = get_AT_value("AT+CICCID", "OK", "", eolstr, 1, 8, 0x0D, simnum_str, 22);
				if (i)
					{
					reset_simcom_flags(COMM_TIMEOUT,1);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
					}
*/

// AT+COPN
#if 0
// check mcc-mnc.com website; there are potentially 119 x 25 x 27 bytes each = > 80kB of data coming from the SIMCOM for this!
// Have verified that UK-network is on there (234-20) so cometing the test out for noew...
				printf("COPN start:\n");
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+COPN\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_send_data(four_g_tx_data,four_g_tx_length);

				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,1000);

//				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				printf("COPN end\n");
#endif

// AT+CNMP
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CNMP?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_send_data(four_g_tx_data,four_g_tx_length);

				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

//				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

// AT+CNBP
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CNBP?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_send_data(four_g_tx_data,four_g_tx_length);

				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

//				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				ptr = strstr((char*)four_g_rx_data,"+CNBP:");

				if (ptr != NULL)
					{
					j = (ptr - (char*)four_g_rx_data) + 7;	//"//+CNBP: "

					j = j + 1 + get_str((char*)&four_g_rx_data[j], ",", tmpstr,dbg_4g_flag);	// Band Select Mode
					printf("Band Select Mode Flags: %s\n",tmpstr);
					p = strlen(tmpstr) - 1;
					i = 0;
					while (i<64)	// up to 255 bits = 64 * 4 ( i_steps * k_steps)
						{
						c = tmpstr[p];	
//						printf("%c [%02X]",c,c);
						
						if (c == 'x')
							i = 0xFF;	// terminate while loop
						else
							{
							if (c>0x40)	// alpha char
								val = c - (0x41 - 10);
							else
								val = c - 0x30;

//							printf(" [%02X]",val);
							
							for(k=0;k<4;k++)
								{
//								printf("  b%d: [%02X}",k,(val | (1 << k)));
								
								if ((val & (1 << k)) != 0)
									{
									binstr[k] = 1;
									printf("bit %3d set\n",(i*4)+k);
									}
								else
									binstr[k] = 0;							
							
								}
						
							i++;
							p--;
							if (p==0)
								i = 0xFF;	// terminate while loop
							}
							
//						printf("\n");
						}
					printf("\n");

					j = j + 1 + get_str((char*)&four_g_rx_data[j], ",", tmpstr,dbg_4g_flag);	// LTE Select Mode
					printf("LTE Mode Flags: %s\n",tmpstr);
					p = strlen(tmpstr) - 1;
					i = 0;
					while (i<64)	// up to 255 bits = 64 * 4 ( i_steps * k_steps)
						{
						c = tmpstr[p];	
						
						if (c == 'x')
							i = 0xFF;	// terminate while loop
						else
							{
							if (c>0x40)	// alpha char
								val = c - (0x41 - 10);
							else
								val = c - 0x30;
							
							for(k=0;k<4;k++)
								{
								if ((val & (1 << k)) != 0)
									{
									binstr[k] = 1;
									printf("bit %3d set\n",(i*4)+k);
									}
								else
									binstr[3-k] = 0;							
							
								}
							i++;
							p--;
							if (p==0)
								i = 0xFF;	// terminate while loop
							}
						}
					printf("\n");


					j = j + 1 + get_str((char*)&four_g_rx_data[j], "\r", tmpstr,dbg_4g_flag);	// TDS Select Mode
					printf("TDS Mode Flags: %s\n",tmpstr);
					p = strlen(tmpstr) - 1;
					i = 0;
					while (i<64)	// up to 255 bits = 64 * 4 ( i_steps * k_steps)
						{
						c = tmpstr[p];	
						
						if (c == 'x')
							i = 0xFF;	// terminate while loop
						else
							{
							if (c>0x40)	// alpha char
								val = c - (0x41 - 10);
							else
								val = c - 0x30;
							
							for(k=0;k<4;k++)
								{
								if ((val & (1 << k)) != 0)
									{
									binstr[k] = 1;
									printf("bit %3d set\n",(i*4)+k);
									}
								else
									binstr[3-k] = 0;							
							
								}
							i++;
							p--;
							if (p==0)
								i = 0xFF;	// terminate while loop
							}
						}
					printf("\n");

					}

// AT+CPSI
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CPSI?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_send_data(four_g_tx_data,four_g_tx_length);

				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

//				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

				ptr = strstr((char*)four_g_rx_data,"+CPSI:");

				if (ptr != NULL)
					{
					j = ptr - (char*)four_g_rx_data + 7;	//"//+CPSI: "
					
					j = j + 1 + get_str((char*)&four_g_rx_data[j], ",", tmpstr,dbg_4g_flag);	// System Mode
					printf("\tSysMode:        %s\n",tmpstr);
					k = get_str((char*)&four_g_rx_data[j], ",", tmpstr,dbg_4g_flag);	// Operation Mode
					j = j + 1 + k;
// debug...					
//					printf("DBG: %s  [%d]\n",tmpstr,k);
					
					if (four_g_rx_length < 50)	// ie, nwk disconnected and only the first 2 params are present...
						{
						c = 0;
						i = 0;
						p = (strlen(tmpstr)) - 1;
						while ((c != 0x0D) && ( i < p))
							{
							c = tmpstr[i];
							if (c == 0x0D)
								tmpstr[i] = 0x00;
							
							i++;
							}
						printf("\tOpMode:         %s\n",tmpstr);
						}
					else
						{
//						j = j + 1 + k;
						printf("\tOpMode:         %s\n",tmpstr);
						j = j + 1 + get_str((char*)&four_g_rx_data[j], "-", tmpstr,dbg_4g_flag);	// MCC (Country Code)
						printf("\tCountryCode:    %s\n",tmpstr);
						j = j + 1 + get_str((char*)&four_g_rx_data[j], ",", tmpstr,dbg_4g_flag);	// MNC (Provider Code)
						printf("\tProviderCode:   %s\n",tmpstr);
						j = j + 1 + get_str((char*)&four_g_rx_data[j], ",", tmpstr,dbg_4g_flag);	// TAC (Trace Area Code)
						printf("\tTraceAreaCode : %s\n",tmpstr);
						j = j + 1 + get_str((char*)&four_g_rx_data[j], ",", tmpstr,dbg_4g_flag);	// SCellID (Service Cell ID) ???
						printf("\tServiceCellID:  %s\n",tmpstr);
						j = j + 1 + get_str((char*)&four_g_rx_data[j], ",", tmpstr,dbg_4g_flag);	// PCellID (Physical Cell ID)
						printf("\tPhysicalCellID: %s\n",tmpstr);
						j = j + 1 + get_str((char*)&four_g_rx_data[j], ",", tmpstr,dbg_4g_flag);	// FreqBand (Frequency Band)
						printf("\tFreqBand:       %s\n",tmpstr);
						j = j + 1 + get_str((char*)&four_g_rx_data[j], ",", tmpstr,dbg_4g_flag);	// EARFCN (RF Cnannel number)
						printf("\tRFChanNumber:   %s\n",tmpstr);
						j = j + 1 + get_str((char*)&four_g_rx_data[j], ",", tmpstr,dbg_4g_flag);	// DLBW (Downlink Bandwidth)
						printf("\tDownLink BW:    %s\n",tmpstr);
						j = j + 1 + get_str((char*)&four_g_rx_data[j], ",", tmpstr,dbg_4g_flag);	// ULBW (Uplink Bandwidth)
						printf("\tUpLink BW:      %s\n",tmpstr);
						j = j + 1 + get_str((char*)&four_g_rx_data[j], ",", tmpstr,dbg_4g_flag);	// RSRQ (Rx Quality)
						printf("\tRxQuality:      %s\n",tmpstr);
						j = j + 1 + get_str((char*)&four_g_rx_data[j], ",", tmpstr,dbg_4g_flag);	// RSRP (Tx Power)
						printf("\tTxPower:        %s\n",tmpstr);
						j = j + 1 + get_str((char*)&four_g_rx_data[j], ",", tmpstr,dbg_4g_flag);	// RSSI(Rx signal strength)
						printf("\tRxSigStrength:  %s\n",tmpstr);
						j = j + 1 + get_str((char*)&four_g_rx_data[j], ",", tmpstr,dbg_4g_flag);	// RSSN (Rx Signal-Noise ratio)
						printf("\tRxSigNoiseRatio %s\n",tmpstr);
						}

					printf("\n");
					}

// AT+CTZU - set (RTC update enable)
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CTZU=1\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_send_data(four_g_tx_data,four_g_tx_length);

				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

//				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

// AT+CTZU - read
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CTZU?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_send_data(four_g_tx_data,four_g_tx_length);

				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

//				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);


// AT+CNTP
// set NTP server IP
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CNTP=132.163.97.5,0\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_send_data(four_g_tx_data,four_g_tx_length);

				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

//				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

// cause NTP sync
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CNTP\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_send_data(four_g_tx_data,four_g_tx_length);

				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

//				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

// AT+CCLK
				reset_simcom_flags(COMM_TIMEOUT,0);		// resets state_mc_timer, timeout_timer, simcom_attempts, response flag
				four_g_tx_length = four_g_pack_data("AT+CCLK?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK

				xQueueReset(mqtt_cmd_queue);
				four_g_send_data(four_g_tx_data,four_g_tx_length);

				four_g_rx_length = 0;
				four_g_rcv_data(four_g_rx_data, &four_g_rx_length,dbg_4g_flag,0);

//				if(debug_do(DBG_4G_TXRX))
					printf("Rx: <\n %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

	
}