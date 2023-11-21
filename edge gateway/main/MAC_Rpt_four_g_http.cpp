//////////////////////////////////////////////
//
// MAC_Rpt_four_g.c
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
#include "MAC_Rpt_four_g_http_defs.h"

extern uint8_t four_g_tx_data[FOUR_G_TX_BUFSIZE];
extern uint8_t four_g_rx_data[FOUR_G_RX_BUFSIZE];
extern unsigned int four_g_tx_length;
extern unsigned int four_g_rx_length;

extern unsigned char four_g_rst_flag, four_g_pwr_flag, four_g_connected_flag;

extern unsigned char four_g_state, prev_four_g_state;
//extern unsigned char four_g_state_str[][];
extern unsigned int four_g_state_mc_timer;

extern char imei_str[];
extern char simnum_str[];
extern char ipaddr_str[];

extern const char four_g_state_str[M4G_NUM_STATES][32];

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
"SET_PDP",
"SET_APN",
"FORCE_CONN",
"CHECK_CONN",
"M4G_CONN_AUTO",
"CHECK_SIG_QUALITY",
"GET_NET_INFO",
"CHECK_ATTACH",
"NET_ACT",
"START_WIRELESS",
"NET_OPEN",
"GET_IP_ADDR",
"TCP_OPEN",
"TCP_SEND",
"TCP_RCV",
"TCP_IDLE",

"POWER_DN_KEY",
"POWER_DN_KEY_WAIT",
"POWER_DN_RESP_WAIT",
"POWER_DISABLE",
"END"
};
*/

void four_g_http_state_machine(unsigned char *four_g_state,unsigned char *prev_four_g_state)
{
int i;
char c;
char * p;
char tmpstr[10];

if (*four_g_state != *prev_four_g_state)
	{
	printf("4G State: [%03d] M4G_%s\r\n",*four_g_state, four_g_state_str[*four_g_state]);
	}

*prev_four_g_state = *four_g_state;

switch(*four_g_state)
	{
	case M4G_POWER_OFF:
		break;
	case M4G_POWER_EN:
		four_g_power_en();
		four_g_state_mc_timer = 0;
		*four_g_state = M4G_POWER_EN_WAIT;
		break;		
	case M4G_POWER_EN_WAIT:
		if (four_g_state_mc_timer >= FOUR_G_POWER_EN_WAIT)
			{
			*four_g_state = M4G_RESET;
			}
		break;
	case M4G_RESET:
		four_g_hold_in_reset();
		four_g_state_mc_timer = 0;
		*four_g_state = M4G_RESET_WAIT;
		break;
	case M4G_RESET_WAIT:
		if (four_g_state_mc_timer >= FOUR_G_RESET_WAIT)
			{
			four_g_release_from_reset();
			*four_g_state = M4G_POWER_ON_KEY;
			}
		break;
	case M4G_POWER_ON_KEY:
		four_g_power_key_on();
		four_g_state_mc_timer = 0;
		*four_g_state = M4G_POWER_ON_KEY_WAIT;
		break;
	case M4G_POWER_ON_KEY_WAIT:
		if (four_g_state_mc_timer >= FOUR_G_POWER_ON_WAIT)
			{
			four_g_power_key_off();
			four_g_state_mc_timer = 0;
			*four_g_state = M4G_POWER_ON_RESP_WAIT;
			}
		break;
	case M4G_POWER_ON_RESP_WAIT:
		if (four_g_state_mc_timer < FOUR_G_POWER_ON_RESP_WAIT)
			{
			if (four_g_state_mc_timer%10 == 0)
				printf("%d ",four_g_state_mc_timer/10);

			if (gpio_get_level(STATUS))
				{
				printf("STATUS line is set...\r\n");
				*four_g_state = M4G_NO_COMM_IDLE;
				}
				
			}
		else
			{
			if (gpio_get_level(STATUS))
				printf("STATUS line is set - OK\r\n");
			else
				printf("STATUS line not set!\r\n");
			
			*four_g_state = M4G_NO_COMM_IDLE;
			}
				
		break;
		
	case M4G_NO_COMM_IDLE:
		printf("Reached NO_COMM_IDLE\r\n");
		four_g_state_mc_timer = 0;
		break;
		
	case M4G_GET_SIM_IMEI:
		if (four_g_state_mc_timer > 10)
			{
		four_g_tx_length = four_g_pack_data("AT+GSN\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		uart_flush(FOUR_G);
		four_g_send_data(four_g_tx_data,four_g_tx_length);
		four_g_rcv_data(four_g_rx_data, &four_g_rx_length,1,100);
		printf("Rx: < %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);
/*
		i = strncmp((char *)four_g_rx_data,"AT+GSN",6);
		p = strstr((char *)four_g_rx_data,"OK");
		if ((i==0) && (p != NULL))
			{
			printf("Found <OK>\r\n");
			
			tmpstr[0] = 0x0A;
			tmpstr[1] = 0x00;
			p = strstr((char *)four_g_rx_data,tmpstr);
			if (p!=NULL)
				{
				c = 0;
				i=0;
				p++;
				while (*p!=0x0D)
					{
					imei_str[i++] = *(p++);
					}
				imei_str[i] = 0x00;
				printf("found IMEI: %s\r\n",imei_str);
				*four_g_state = M4G_GET_SIM_NUM;
				}
			}		
*/
		tmpstr[0] = 0x0A;
		tmpstr[1] = 0x00;
		i = get_AT_value("AT+GSN", "OK", "", tmpstr, 1, 0, 0x0D, imei_str, 20);
		if (i)
			{
			*four_g_state = M4G_GET_SIM_NUM;
			}

		four_g_state_mc_timer = 0;
		
			}

//		*four_g_state = M4G_GET_SIM_NUM;
		break;
		
	case M4G_GET_SIM_NUM:
//		AT+CCID		// respond with number <cr-lf> OK; if number ends in F, strip...
		if (four_g_state_mc_timer > 10)
			{
		four_g_tx_length = four_g_pack_data("AT+CICCID\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		uart_flush(FOUR_G);
		four_g_send_data(four_g_tx_data,four_g_tx_length);
		four_g_rcv_data(four_g_rx_data, &four_g_rx_length,1,100);
		printf("Rx: < %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);
/*
i = strncmp((char *)four_g_rx_data,"AT+CICCID",9);
		p = strstr((char *)four_g_rx_data,"OK");
		if ((i==0) && (p != NULL))
			{
			printf("Found <OK>\r\n");
			
			tmpstr[0] = 0x0A;
			tmpstr[1] = 0x00;
			p = strstr((char *)four_g_rx_data,tmpstr);
			if (p!=NULL)
				{
				c = 0;
				i=0;
				p = p + 1 + 8;
				while (*p!=0x0D)
					{
					simnum_str[i++] = *(p++);
					}
				simnum_str[i] = 0x00;
				printf("found SIMNUM: %s\r\n",simnum_str);
				*four_g_state = M4G_CLOSEALL;
				}
			}		
*/
		tmpstr[0] = 0x0A;
		tmpstr[1] = 0x00;

		i = get_AT_value("AT+CICCID", "OK", "", tmpstr, 1, 8, 0x0D, simnum_str, 22);
		if (i)
			{
			*four_g_state = M4G_CLOSEALL;
			}

		four_g_state_mc_timer = 0;
		
			}
		
//		while (1) {};
		break;
		
	case M4G_CLOSEALL:
//		AT+CIPSHUT		// respond with SHUT OK
		if (four_g_state_mc_timer > 10)
			{
		four_g_tx_length = four_g_pack_data("AT+NETCLOSE\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		uart_flush(FOUR_G);
		four_g_send_data(four_g_tx_data,four_g_tx_length);
		four_g_rcv_data(four_g_rx_data, &four_g_rx_length,1,100);
		printf("Rx: < %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

		tmpstr[0] = 0x0A;
		tmpstr[1] = 0x00;

		i = get_AT_value("AT+NETCLOSE", "", "", tmpstr, 0, 0, 0, NULL, 0);
		if (i)
			{
			*four_g_state = M4G_SET_APN;	//M4G_SET_PDP;
			four_g_state_mc_timer = 0;
			}
		
			}
		break;
	case M4G_SET_PDP:
//		AT+CGDCONT=1, "IP", "AQL"	// respond with OK
		if (four_g_state_mc_timer > 10)
			{
		four_g_tx_length = four_g_pack_data("AT+CGDCONT=1, \"IP\", \"AQL\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		uart_flush(FOUR_G);
		four_g_send_data(four_g_tx_data,four_g_tx_length);
		four_g_rcv_data(four_g_rx_data, &four_g_rx_length,1,100);
		printf("Rx: < %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

/*
four_g_send_data(four_g_tx_data,four_g_tx_length);
		p = findstr(four_g_rx_data, "OK");
		if (p != NULL)
*/
		tmpstr[0] = 0x0A;
		tmpstr[1] = 0x00;

		i = get_AT_value("AT+CGDCONT", "OK", "", tmpstr, 0, 0, 0, NULL, 0);
		if (i)
			{
			*four_g_state = M4G_SET_APN;	//M4G_NET_OPEN;	//M4G_SET_APN;
			four_g_state_mc_timer = 0;
			}
//		else
//			*four_g_state = M4G_SET_PDP;
		
			}
		
		break;
	case M4G_SET_APN:
//		AT+CSTT="m2m.aql.net"	// respond with OK	AT+CSOCKSETPN=1
//		four_g_tx_length = four_g_pack_data("AT+CSTT=\"m2m.aql.net\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		if (four_g_state_mc_timer > 10)
			{
		four_g_tx_length = four_g_pack_data("AT+CSOCKSETPN=1\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		uart_flush(FOUR_G);
		four_g_send_data(four_g_tx_data,four_g_tx_length);
		four_g_rcv_data(four_g_rx_data, &four_g_rx_length,1,100);
		printf("Rx: < %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

		tmpstr[0] = 0x0A;
		tmpstr[1] = 0x00;

		i = get_AT_value("AT+CSOCKSETPN", "OK", "", tmpstr, 0, 0, 0, NULL, 0);
		if (i)
			{
			*four_g_state = M4G_FORCE_CONN;
			four_g_state_mc_timer = 0;
			}
		
			}
		break;
	case M4G_FORCE_CONN:
//		AT_COPS=1,2,"23429"			// respond with OK
//		four_g_tx_length = four_g_pack_data("AT_COPS=1,2,\"23429\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		if (four_g_state_mc_timer > 10)
			{
		four_g_tx_length = four_g_pack_data("AT+CIPMODE=0\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		uart_flush(FOUR_G);
		four_g_send_data(four_g_tx_data,four_g_tx_length);
		four_g_rcv_data(four_g_rx_data, &four_g_rx_length,1,100);
		printf("Rx: < %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

			{
			*four_g_state = M4G_NET_OPEN;	//M4G_CHECK_CONN;
			four_g_state_mc_timer = 0;
			}
		
			}
		break;
	case M4G_CHECK_CONN:
//		AT_COPS?			// respond with +COPS: 1,2,"23429",n (n is the response type)
		four_g_tx_length = four_g_pack_data("AT+COPS?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		four_g_send_data(four_g_tx_data,four_g_tx_length);
			*four_g_state = M4G_CONN_AUTO;
		break;
	case M4G_CONN_AUTO:
//		AT_COPS=0			// respond with +COPS: 0,2,"310410",7
		four_g_tx_length = four_g_pack_data("AT_COPS=0\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		four_g_send_data(four_g_tx_data,four_g_tx_length);
		*four_g_state = M4G_CHECK_SIG_QUALITY;
		break;
	case M4G_CHECK_SIG_QUALITY:
//		AT_CSQ				// respond with +CSQ: 14,99
		four_g_tx_length = four_g_pack_data("AT+CSQ\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		four_g_send_data(four_g_tx_data,four_g_tx_length);
		*four_g_state = M4G_GET_NET_INFO;
		break;
	case M4G_GET_NET_INFO:
//		AT_CPSI?			// respond with 
//+CPSI: LTE CAT-M1,Online,310-410,0x4804,74777865,343,EUTRAN-BAND2,875,4,4,-13,-112,-82,14
		four_g_tx_length = four_g_pack_data("AT_COPS=1,2,\"23429\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		four_g_send_data(four_g_tx_data,four_g_tx_length);
		*four_g_state = M4G_CHECK_ATTACH;
		break;
	case M4G_CHECK_ATTACH:
//		AT+CGATT?		// respond with +CGATT: 1 <cr-lf><cr-lf> OK
		four_g_tx_length = four_g_pack_data("AT+CGATT?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		four_g_send_data(four_g_tx_data,four_g_tx_length);
		*four_g_state = M4G_NET_ACT;
		break;
	case M4G_NET_ACT:
//		AT+CNACT=1,"m2m.aql.net"	//respond with OK <cr-lf><cr-lf> +APP PDP: ACTIVE
		four_g_tx_length = four_g_pack_data("AT+CNACT=1,\"m2m.aql.net\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		four_g_send_data(four_g_tx_data,four_g_tx_length);
			*four_g_state = M4G_START_WIRELESS;
		break;
	case M4G_START_WIRELESS:
//		AT+CIICR		// respond with OK
		four_g_tx_length = four_g_pack_data("AT+CIICR\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		four_g_send_data(four_g_tx_data,four_g_tx_length);
		*four_g_state = M4G_GET_IP_ADDR;
		break;
	case M4G_NET_OPEN:
		if (four_g_state_mc_timer > 10)
			{
		four_g_tx_length = four_g_pack_data("AT+NETOPEN\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		uart_flush(FOUR_G);
		four_g_send_data(four_g_tx_data,four_g_tx_length);
		(10 / portTICK_PERIOD_MS);	// 10 milliseconds
		four_g_rcv_data(four_g_rx_data, &four_g_rx_length,1,100);
		printf("Rx: < %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

		tmpstr[0] = 0x0A;
		tmpstr[1] = 0x00;

		i = get_AT_value("AT+NETOPEN", "+NETOPEN:", "", tmpstr, 0, 0, 0, NULL, 0);
		if (i)
			{
			*four_g_state = M4G_GET_IP_ADDR;	//M4G_SET_APN;
			four_g_state_mc_timer = 0;
			}
			}
		break;
	case M4G_GET_IP_ADDR:
		if (four_g_state_mc_timer >= 50)
		{
		four_g_tx_length = four_g_pack_data("AT+IPADDR\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		uart_flush(FOUR_G);
		four_g_send_data(four_g_tx_data,four_g_tx_length);
		four_g_rcv_data(four_g_rx_data, &four_g_rx_length,1,100);
		printf("Rx: < %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

		tmpstr[0] = 0x0A;
		tmpstr[1] = 0x00;

		i = get_AT_value("AT+IPADDR", "OK", "", tmpstr, 1, 9, 0x0D, ipaddr_str, 16);
		if (i)
			{
			*four_g_state = M4G_TCP_OPEN;	//M4G_SET_APN;
			four_g_state_mc_timer = 0;
			}
		}
		break;
	case M4G_TCP_OPEN:
//		AT+CIPOPEN="TCP","iot-visualiser.aql.com","80"
// respond with OK <cr-lf><cr-lf> CONNECT OK <cr-lf><cr-lf>
		if (four_g_state_mc_timer > 10)
			{
		four_g_tx_length = four_g_pack_data("AT+CIPOPEN=0,\"TCP\",\"iot-visualiser.aql.com\",80\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		uart_flush(FOUR_G);
		four_g_send_data(four_g_tx_data,four_g_tx_length);
		four_g_rcv_data(four_g_rx_data, &four_g_rx_length,1,100);
		printf("Rx: < %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

		tmpstr[0] = 0x0A;
		tmpstr[1] = 0x00;

		i = get_AT_value("AT+CIPOPEN", "OK", "", tmpstr, 0, 0, 0, NULL, 0);
		if (i)
			{
			*four_g_state = M4G_TCP_SEND;
			four_g_state_mc_timer = 0;
			}
		}
		break;
	case M4G_TCP_SEND:
//		AT+CIPSEND	// start input; terminate with 0x1A;	responds with SEND OK
		if (four_g_state_mc_timer > 10)
			{
		four_g_tx_length = four_g_pack_data("AT+CIPSEND=0,\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		uart_flush(FOUR_G);
		four_g_send_data(four_g_tx_data,four_g_tx_length);
		four_g_rcv_data(four_g_rx_data, &four_g_rx_length,1,100);
		printf("Rx: < %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

		tmpstr[0] = 0x0A;
		tmpstr[1] = 0x00;

		i = get_AT_value("AT+CIPSEND", ">", "", tmpstr, 0, 0, 0, NULL, 0);
		if (i)
			{
			uart_flush(FOUR_G);
//			four_g_tx_length = four_g_pack_data("POST /api/resource/aqlgateway/dom/1234 HTTP/1.1\r\n",(char *)four_g_tx_data);
			strcpy((char *)four_g_tx_data,"POST /api/resource/aqlgateway/dom/");
			strcat((char *)four_g_tx_data, imei_str);	// add unique ID
			strcat((char *)four_g_tx_data, "HTTP/1.1\r\n");
			four_g_tx_length = strlen((char *)four_g_tx_data);
			
			four_g_send_data(four_g_tx_data,four_g_tx_length);
			four_g_tx_length = four_g_pack_data("Host: iot-visualiser.aql.com\r\n",(char *)four_g_tx_data);
			four_g_send_data(four_g_tx_data,four_g_tx_length);
			four_g_tx_length = four_g_pack_data("Accept: application/json\r\n",(char *)four_g_tx_data);
			four_g_send_data(four_g_tx_data,four_g_tx_length);
			four_g_tx_length = four_g_pack_data("Content-Type: application/json\r\n",(char *)four_g_tx_data);
			four_g_send_data(four_g_tx_data,four_g_tx_length);
			four_g_tx_length = four_g_pack_data("Content-Length: 277\r\n\r\n",(char *)four_g_tx_data);
			four_g_send_data(four_g_tx_data,four_g_tx_length);
			
//			four_g_tx_length = four_g_pack_data("{\"cell_nwinfo\":{\"imei\":\"100234\",\"sim\":\"abd200\",\"mccmnc\":\"310410\",\"nwtype\": \"LTE\"},\"data\":[{\"node_id\":\"1\",\"int-temp\": \"1.23\",\"vin\": \"12.1\",\"int-vbat\": \"4.2\",\"gps-lat\": \"1.0\",\"gps-lon\": \"1.5\",\"gps-spd\": \"2\",\"radio-freq\": \"915.5\",\"cell-csq\": \"20\",\"cycles\": \"2\",\"up-time\": \"100\"}]}",(char *)four_g_tx_data);
			strcpy((char *)four_g_tx_data,"{\"cell_nwinfo\":{\"imei\":\"");
			strcat((char *)four_g_tx_data,imei_str);
			strcat((char *)four_g_tx_data,"\",\"sim\":\"abd200\",\"mccmnc\":\"310410\",\"nwtype\": \"LTE\"},\"data\":[{\"node_id\":\"1\",\"int-temp\": \"1.23\",\"vin\": \"12.1\",\"int-vbat\": \"4.2\",\"gps-lat\": \"1.0\",\"gps-lon\": \"1.5\",\"gps-spd\": \"2\",\"radio-freq\": \"915.5\",\"cell-csq\": \"20\",\"cycles\": \"2\",\"up-time\": \"100\"}]}");
			four_g_tx_length = strlen((char *)four_g_tx_data);
			
			four_g_send_data(four_g_tx_data,four_g_tx_length);
			four_g_tx_data[0] = 0x1A;
			four_g_tx_data[1] = 0x00;
			four_g_send_data(four_g_tx_data,1);

			*four_g_state = M4G_TCP_RCV;
			four_g_state_mc_timer = 0;
			}
			}
		break;
	case M4G_TCP_RCV:
			// respond with CLOSED
		if (four_g_state_mc_timer > 10)
			{
		four_g_rcv_data(four_g_rx_data, &four_g_rx_length,1,100);
		printf("Rx: < %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

		tmpstr[0] = 0x0A;
		tmpstr[1] = 0x00;

		i = get_AT_value("POST", "\"result\":\"success\"", "", tmpstr, 0, 0, 0, NULL, 0);
		if (i)
			{
			printf("Receive successful!\r\n");
			}
			
		*four_g_state = M4G_TCP_IDLE;
			}
		break;
	case M4G_TCP_IDLE:
		break;

	case M4G_POWER_DOWN_KEY:
		four_g_state_mc_timer = 0;
		*four_g_state = M4G_POWER_DOWN_KEY_WAIT;
		break;		
	case M4G_POWER_DOWN_KEY_WAIT:
		if (four_g_state_mc_timer >= FOUR_G_POWER_DOWN_KEY_WAIT)
			{
			*four_g_state = M4G_POWER_DOWN_RESP_WAIT;
			}				
		break;				
	case M4G_POWER_DOWN_RESP_WAIT:
		if (four_g_state_mc_timer >= FOUR_G_POWER_DOWN_RESP_WAIT)
			{
			*four_g_state = M4G_POWER_DISABLE;
			}
		break;
	case M4G_POWER_DISABLE:
		four_g_power_disable();
//		four_g_state_mc_timer = 0;
		*four_g_state = M4G_POWER_OFF;
		break;		
	case M4G_END:
		break;
	default:
		break;
	}
}


