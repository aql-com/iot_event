//////////////////////////////////////////////
//
// MAC_Rpt_four_g_7000.c
//
// aql Ltd
//
// Auth: DLT
//
// for SIMCOM 7000 - under development - not tested yet!
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
#include "MAC_Rpt_four_g_state_mc_defs.h"

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

unsigned int four_g_pack_data(char *four_g_data, char *outstr)
{
strcpy(outstr,four_g_data);
return strlen((char *)outstr);
}

int four_g_send_data(uint8_t *four_g_data, unsigned int four_g_data_len)
{
int ret = 0;

	ret = uart_write_bytes(FOUR_G, (char *)four_g_data, four_g_data_len);

	printf("4G: sending <%s>\r\n",(char *)four_g_data);
	printf("4G: result: %d\r\n",ret);


return ret;
}

int four_g_rcv_data(uint8_t *four_g_data, unsigned int *four_g_data_len)
{
*four_g_data_len = get_uart_rx_msg(FOUR_G, four_g_data, FOUR_G_RX_BUFSIZE, 100);

debug_hex_msg(four_g_data,*four_g_data_len,"Rx:");

return 0;
}

unsigned char four_g_power_en(void)
{
// place holder 
return 0;
}

unsigned char four_g_power_disable(void)
{
// place holder 
return 0;
}

// state of four_g reset is held by the four_g_rst_flag variable...
unsigned char four_g_hold_in_reset(void)
{
// 4G reset: hold 7600 "RST" pin LOW; so hold ESP32 "FOUR_G_RESET" pin high...
unsigned char error;

error = gpio_set_level(FOUR_G_RESET,1);	// ESP signal is inverted by FET
four_g_rst_flag = 1;

return four_g_rst_flag;

}

unsigned char four_g_release_from_reset(void)
{
unsigned char error;

error = gpio_set_level(FOUR_G_RESET,0);	// ESP signal is inverted by FET
four_g_rst_flag = 0;

return four_g_rst_flag;
}

unsigned char four_g_reset_state(void)
{

return four_g_rst_flag;
}

// state of four_g power is held by the four_g_pwr_flag variable...
unsigned char four_g_power_key_on(void)
{
// PWR_KEY is active low; FET inverts ESP32 signal; so "FOUR_G_POWER" pin must go high...
// needs to be low for 500ms...!
unsigned char error;

error = gpio_set_level(FOUR_G_POWER,1);	// ESP signal is inverted by FET
four_g_pwr_flag = 1;

return four_g_pwr_flag;
}

unsigned char four_g_power_key_off(void)
{
// PWR_KEY is active low; FET inverts ESP32 signal; so "FOUR_G_POWER" pin must go low...
unsigned char error;

error = gpio_set_level(FOUR_G_POWER,0);	// ESP signal is inverted by FET
four_g_pwr_flag = 1;

return four_g_pwr_flag;
}

unsigned char four_g_power_state(void)
{

return four_g_pwr_flag;
}

unsigned char four_g_check(unsigned char retries)
{
unsigned char c,i;
unsigned int j = 0;
int x;

strcpy((char *)four_g_tx_data,"AT\r");

for (i=0;i<retries;i++)
	{
	four_g_send_data(four_g_tx_data, strlen((char *)four_g_tx_data));

	four_g_rx_length = get_uart_rx_msg(FOUR_G, four_g_rx_data, FOUR_G_RX_BUFSIZE, 100);

	printf("4G: %d\r\n",four_g_rx_length);
	for (j=0;j<four_g_rx_length;j++)
		{
		if ((four_g_rx_data[j] > 0x1F) && (four_g_rx_data[j] < 0x80))
			c = four_g_rx_data[j];
		else
			c = '.';
		printf("%c  ",c);
		
		}
	printf("\r\n");

	for (j=0;j<four_g_rx_length;j++)
		{
		printf("%02X ",four_g_rx_data[j]);
		
		}
	printf("\r\n");
	
	x=strcmp((char *)four_g_rx_data,"AT\r\r\nOK\r\n");
	if ((four_g_rx_length) && (x==0))
		{
		i = retries;
		}
	else
		{
		printf("CMP: %d\r\n",x);
		vTaskDelay(500 / portTICK_PERIOD_MS);	// 1 millisecond
		}
	}
	
if (i > retries)
	{
	printf("4G connected\r\n");
	four_g_connected_flag = 1;
	}
else
	{
	printf("4G not connected\r\n");
	four_g_connected_flag = 0;
	}
	
return four_g_connected_flag;
}

void four_g_state_machine(unsigned char *four_g_state,unsigned char *prev_four_g_state)
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
			*four_g_state = M4G_POWER_ON_RESP_WAIT;
			}
		break;
	case M4G_POWER_ON_RESP_WAIT:
		if (four_g_state_mc_timer < FOUR_G_POWER_ON_RESP_WAIT)
			{
			if (gpio_get_level(STATUS))
				{
				printf("STATUS line is set...\r\n");
				*four_g_state = M4G_NO_COMM_IDLE;
				}
				
			}
		else
			{
			if (gpio_get_level(STATUS))
				printf("STATUS line is set...\r\n");
			else
				printf("STATUS line not set...\r\n");
			
			*four_g_state = M4G_NO_COMM_IDLE;
			}
				
		break;
		
	case M4G_NO_COMM_IDLE:
		break;
		
	case M4G_GET_SIM_IMEI:
		four_g_tx_length = four_g_pack_data("AT+GSN\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		uart_flush(FOUR_G);
		four_g_send_data(four_g_tx_data,four_g_tx_length);
		four_g_rcv_data(four_g_rx_data, &four_g_rx_length);
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
		i = get_AT_value("AT+GSN", "OK", tmpstr, 1, 0, 0x0D, imei_str, 20);
		if (i)
			*four_g_state = M4G_GET_SIM_NUM;
		

//		*four_g_state = M4G_GET_SIM_NUM;
		break;
		
	case M4G_GET_SIM_NUM:
//		AT+CCID		// respond with number <cr-lf> OK; if number ends in F, strip...
		four_g_tx_length = four_g_pack_data("AT+CICCID\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		uart_flush(FOUR_G);
		four_g_send_data(four_g_tx_data,four_g_tx_length);
		four_g_rcv_data(four_g_rx_data, &four_g_rx_length);
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

		i = get_AT_value("AT+CICCID", "OK", tmpstr, 1, 8, 0x0D, simnum_str, 22);
		if (i)
			*four_g_state = M4G_CLOSEALL;
		
//		while (1) {};
		break;
		
	case M4G_CLOSEALL:
//		AT+CIPSHUT		// respond with SHUT OK
		four_g_tx_length = four_g_pack_data("AT+NETCLOSE\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		uart_flush(FOUR_G);
		four_g_send_data(four_g_tx_data,four_g_tx_length);
		four_g_rcv_data(four_g_rx_data, &four_g_rx_length);
		printf("Rx: < %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

		tmpstr[0] = 0x0A;
		tmpstr[1] = 0x00;

		i = get_AT_value("AT+NETCLOSE", "", tmpstr, 0, 0, 0, NULL, 0);
		if (i)
			*four_g_state = M4G_SET_PDP;
		break;
	case M4G_SET_PDP:
//		AT+CGDCONT=1, "IP", "AQL"	// respond with OK
		four_g_tx_length = four_g_pack_data("AT+CGDCONT=1, \"IP\", \"AQL\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		uart_flush(FOUR_G);
		four_g_send_data(four_g_tx_data,four_g_tx_length);
		four_g_rcv_data(four_g_rx_data, &four_g_rx_length);
		printf("Rx: < %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

/*
four_g_send_data(four_g_tx_data,four_g_tx_length);
		p = findstr(four_g_rx_data, "OK");
		if (p != NULL)
*/
		tmpstr[0] = 0x0A;
		tmpstr[1] = 0x00;

		i = get_AT_value("AT+CGDCONT", "OK", tmpstr, 0, 0, 0, NULL, 0);
		if (i)
			*four_g_state = M4G_SET_APN;
//		else
//			*four_g_state = M4G_SET_PDP;
		
		break;
	case M4G_SET_APN:
//		AT+CSTT="m2m.aql.net"	// respond with OK
		four_g_tx_length = four_g_pack_data("AT+CSTT=\"m2m.aql.net\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		uart_flush(FOUR_G);
		four_g_send_data(four_g_tx_data,four_g_tx_length);
		four_g_rcv_data(four_g_rx_data, &four_g_rx_length);
		printf("Rx: < %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);

		tmpstr[0] = 0x0A;
		tmpstr[1] = 0x00;

		i = get_AT_value("AT+CSTT", "OK", tmpstr, 0, 0, 0, NULL, 0);
		if (i)
		*four_g_state = M4G_FORCE_CONN;
		break;
	case M4G_FORCE_CONN:
//		AT_COPS=1,2,"23429"			// respond with OK
		four_g_tx_length = four_g_pack_data("AT_COPS=1,2,\"23429\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		four_g_send_data(four_g_tx_data,four_g_tx_length);
		*four_g_state = M4G_CHECK_CONN;
		break;
	case M4G_CHECK_CONN:
//		AT_COPS?			// respond with +COPS: 1,2,"23429",n (n is the response type)
		four_g_tx_length = four_g_pack_data("AT_COPS?\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
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
	case M4G_GET_IP_ADDR:
//		AT+CISFR	// respond with 10.191.64.52
		four_g_tx_length = four_g_pack_data("AT+CIFSR\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		four_g_send_data(four_g_tx_data,four_g_tx_length);
		*four_g_state = M4G_TCP_OPEN;
		break;
	case M4G_TCP_OPEN:
//		AT+CIPSTART="TCP","iot-visualiser.aql.com","80"
// respond with OK <cr-lf><cr-lf> CONNECT OK <cr-lf><cr-lf>
		four_g_tx_length = four_g_pack_data("AT+CIPSTART=\"TCP\",\"iot-visualiser.aql.com\",\"80\"\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		four_g_send_data(four_g_tx_data,four_g_tx_length);
			*four_g_state = M4G_TCP_SEND;
		break;
	case M4G_TCP_SEND:
//		AT+CIPSEND	// start input; terminate with 0x1A;	responds with SEND OK
		four_g_tx_length = four_g_pack_data("AT+CIPSEND\r",(char *)four_g_tx_data);		// respond with number <cr-lf> OK
		four_g_send_data(four_g_tx_data,four_g_tx_length);
		*four_g_state = M4G_TCP_RCV;
		break;
	case M4G_TCP_RCV:
			// respond with CLOSED
			
			*four_g_state = M4G_TCP_IDLE;
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


char* findstr(unsigned char *str1, char *str2)
{
unsigned char c, gotline, found;
unsigned int i,j,str1len;
char tmpstr[80];
char *p;

found = 0;
i = 0;
str1len = strlen((char *)str1);
while (found == 0)
	{
	gotline = 0;
	c = 0x00;
	j = 0;
	while (gotline == 0)
		{
		c = str1[i++];
		tmpstr[j++] = c;
		if (c == 0x0A)
			gotline = 1;
		if (i >= str1len)
			gotline = 2;
		
		}
	if (gotline == 1)		// found a line of text
		{
		p = strstr(tmpstr, (char *)str2);
		if (p != NULL)
			{
			found = 1;
			printf("found string <%s>\r\n",str2);
			}
		}
	if (gotline == 2)		// end of input rx string
		{
		found = 1;
		p = NULL;
		printf("string <%s> not found\r\n",str2);
		}
	}
	
return p;
}

/*
char *getstr(unsigned char *str1,unsigned char *str2)
{
char *p;

		

return p;
}
*/

void print_4g_state(unsigned char chan)
{
dbgprintf(chan,"4G State:  [%03d] < M4G_%s >    4G timer: %d\r\n",four_g_state,four_g_state_str[four_g_state],four_g_state_mc_timer);
}


unsigned char get_AT_value(char *cmdstr, char *okstr, char *eolstr, unsigned char line, unsigned char offset, char endchar, char *outstr, unsigned int outlen)
{
unsigned char ret = 0;
unsigned int i = 0;
char j;
char *p;
	
		i = strncmp((char *)four_g_rx_data,cmdstr,strlen(cmdstr));
		
		if (strlen(okstr)==0) 
			j = 1;
		else if ((p = strstr((char *)four_g_rx_data,okstr)) != NULL)
			{
			j = 2;
			printf("Found OK string: <%s>\r\n",okstr);
			}
		else
			j = 0;
		
//printf("i=%d j=%d\r\n",i,j);

		if ((i==0) && (j))		// if cmd str found in rx and OK string found or not required
			{	
//			printf("0>?");

			p = (char *)four_g_rx_data;
			if (line)
				{
				for (i=0;i<line;i++)
					{
//					printf("i=%d\r\n",i);
					p = strstr(p,eolstr);
					}
				}
//			printf("1>?");

			if (p!=NULL)
				{
//				c = 0;
//			printf("2>?");

				if (outlen)		// if want to find value string
					{
					i = 0;
					p = p + 1 + offset;
					while ((*p!=endchar) && (i < outlen))
						{
						outstr[i++] = *(p++);
						}
					outstr[i] = 0x00;
					printf("Found [%s]: %s\r\n",cmdstr,outstr);
					ret = 1;
					}
				else
					{
					printf("Cmd [%s] complete\r\n",cmdstr);
					ret = 1;
					}
					
//				*four_g_state = M4G_CLOSEALL;
				}
			}
		else	
			printf("%s not found\r\n",cmdstr);

return ret;
}
//uart_enable_pattern_det_baud_intr(UART_NUM_1, 0x0a, 1, 9, 0, 0); // pattern is LF

