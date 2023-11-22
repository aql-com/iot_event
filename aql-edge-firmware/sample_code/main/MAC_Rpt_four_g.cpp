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
//#include "MAC_Rpt_four_g_state_mc_defs.h"
#include "MAC_Rpt_four_g_simcom.h"

#ifdef USE_M4G_HTTP
#include "MAC_Rpt_four_g_simcom.h"

#include "MAC_Rpt_four_g_http_defs.h"
#include "MAC_Rpt_four_g_http.h"
#endif


#ifdef USE_M4G_MQTT
//#include "MAC_Rpt_four_g_mqtt_defs.h"
#include "MAC_Rpt_four_g_mqtt.h"
#endif

// default values
#ifndef USE_M4G_HTTP
#ifndef USE_M4G_MQTT
//#define M4G_NUM_STATES	1
#include "MAC_Rpt_four_g_http_defs.h"
#endif
#endif

extern unsigned long int dbgbits;
extern unsigned char dbgflag;

extern uint8_t four_g_tx_data[FOUR_G_TX_BUFSIZE];
extern uint8_t four_g_rx_data[FOUR_G_RX_BUFSIZE];
extern unsigned int four_g_tx_length;
extern unsigned int four_g_rx_length;

extern unsigned char four_g_rst_flag, four_g_pwr_flag, four_g_psu_en_flag, four_g_connected_flag;

extern unsigned char four_g_state, prev_four_g_state;
//extern unsigned char four_g_state_str[][];
extern unsigned int four_g_state_mc_timer;

extern unsigned char simcom_state, prev_simcom_state;
extern unsigned int simcom_state_mc_timer;

extern char imei_str[];
extern char simnum_str[];
extern char ipaddr_str[];
extern const char simcom_state_str[SIMCOM_NUM_STATES][32];

#ifdef USE_M4G_MQTT
extern QueueHandle_t mqtt_cmd_queue;
extern unsigned char CMQTT_rx_flag;
#endif
extern unsigned int rcv_timeout_timer;

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

unsigned int four_g_pack_data(char *four_g_data, char *outstr)
{
strcpy(outstr,four_g_data);
return strlen((char *)outstr);
}

int four_g_send_data(uint8_t *four_g_data, unsigned int four_g_data_len)
{
int ret = 0;
char prt_str[256];
int sent_len;

unsigned int len;

//	esp_task_wdt_reset();		
//	sent_len = ret = uart_write_bytes(FOUR_G, (char *)four_g_data, four_g_data_len);
//	esp_task_wdt_reset();		
	
	len = four_g_data_len;
	sent_len = 0;

	while(len > 128)
		{
#ifdef CTS2_TEST
		printf("     ### CTS2: %d!\n",gpio_get_level(CTS2));
#endif
	
		ret = uart_write_bytes(FOUR_G, (char *)four_g_data, 128);
		sent_len = sent_len + ret;
		len = len - ret;

#ifdef CTS2_TEST
		printf("     ### CTS2: %d!\n",gpio_get_level(CTS2));
#endif

		if (debug_do(DBG_MQTT))
			printf("sent %d bytes...\n",ret);
		
		esp_task_wdt_reset();		
		vTaskDelay(100 / portTICK_PERIOD_MS);	// 100 milliseconds
		}
	
	if (len)
		{
#ifdef CTS2_TEST
		printf("     ### CTS2: %d!\n",gpio_get_level(CTS2));
#endif

		ret = uart_write_bytes(FOUR_G, (char *)four_g_data, len);
		sent_len = sent_len + ret;

#ifdef CTS2_TEST
		printf("     ### CTS2: %d!\n",gpio_get_level(CTS2));
#endif

		if (debug_do(DBG_MQTT))
			printf("sent last %d bytes...\n",ret);
		}
		

	if (sent_len < 0)
		{
		ret = -1;
		sent_len = 0;
		}
	else	
		ret = sent_len;
	
	if (1)	//four_g_data_len < 256)
		{
		if (debug_do(DBG_MQTT))
			printf("4G: sending <%s\n>  [%d] [%d]\r\n",(char *)four_g_data,four_g_data_len,sent_len);
		}
	else
		{
		strncpy(prt_str,(char *)four_g_data,255);
		prt_str[255] = 0x00;
		if (debug_do(DBG_MQTT))
			printf("4G: sending (part) <%s\n>  [%d] [%d]\r\n",prt_str,four_g_data_len,sent_len);
		}
	
	if (debug_do(DBG_MQTT))
		printf("4G: result: %d\r\n",ret);


return ret;
}

int four_g_rcv_data(uint8_t *four_g_data, unsigned int *four_g_data_len, unsigned char debugflag, unsigned int timeout)
{
// if timeout = 0, wait for default 110ms  before receiving any data from system comms buffer
// if timeout > 0, wait for timeout period before receiving any data from system comms buffer 	
unsigned char x = pdTRUE;
unsigned char c = 0x00;
unsigned int len,tmp,tmp2,timeout_time,bytes_to_rx;
int64_t xtime;
unsigned char i = 0;

#if 0
	if (debug_do(DBG_MQTT))
	printf("Rcv Start\n");
#endif

//*four_g_data_len = 0;
//tmp = *four_g_data_len;
bytes_to_rx = *four_g_data_len;

if (timeout > 110)
	timeout_time = timeout;
else
	timeout_time = 110;	// 110 ms min timeout...

#ifndef USE_M4G_MQTT

*four_g_data_len = get_uart_rx_msg(FOUR_G, four_g_data, FOUR_G_RX_BUFSIZE, 100);

/*
rcv_timeout_timer = 0;
while (rcv_timeout_timer < 100)	// && ())
	{
	
	}
*/
#else
if (1)	//*four_g_data_len == 0)		// receive all data available
{
#ifdef DBG_TEST
printf("\nGot here 10a %d\n",timeout);	
#endif
#ifdef STACK_TEST
printf("\nFour_g rx timeout: %d\n",timeout);	
printf("HP free %d\n",heap_caps_get_free_size(MALLOC_CAP_8BIT));
printf("HP OK %d\n",heap_caps_check_integrity_all(1));
#endif

/*
if (timeout)
	{
#if 0	
	unsigned char i;
	unsigned char loop = 1;
	unsigned char remainder;
	if (timeout > 100)
		{
		loop = timeout / 100;
		remainder = timeout % 100;
		for (i=0;i<loop;i++)
			{
printf("*");
			esp_task_wdt_reset();		
			vTaskDelay(100 / portTICK_PERIOD_MS);	// 100 milliseconds
			}
#ifdef DBG_TEST
printf("\nGot here 10b\n");
#endif
		if (remainder >= portTICK_PERIOD_MS)
			vTaskDelay(remainder / portTICK_PERIOD_MS);	// <100 milliseconds
#ifdef DBG_TEST
printf("\nGot here 10c\n");
#endif
		}
	else
#endif	// end of "if 0..."
		{
#ifdef DBG_TEST
printf("\nGot here 10d %d\n",timeout);
#endif
#ifdef STACK_TEST
printf("HP free %d\n",heap_caps_get_free_size(MALLOC_CAP_8BIT));
printf("HP OK %d\n",heap_caps_check_integrity_all(1));
#endif
//##		vTaskDelay(timeout / portTICK_PERIOD_MS);	// 100 milliseconds
//		vTaskDelay(100 / portTICK_PERIOD_MS);	// 100 milliseconds
#ifdef DBG_TEST
printf("post delay\n");
#endif
		}
	}
else
	vTaskDelay(100 / portTICK_PERIOD_MS);	// 100 milliseconds
*/

// get ready for reception:
	*four_g_data_len = 0;

#if 1
// wait for a few ms, then get whatever daa is available, up to the full timeout
// prevents 2k UART buffer overloading if msg > 2k and four_g_data buf > 2k...
#define RCV_WAIT	50
	while (timeout_time)
		{
// first wait for RCV_WAIT time chunks before receiving into the four_g_data array...			
		if (timeout_time / RCV_WAIT)
			{
			vTaskDelay(RCV_WAIT / portTICK_PERIOD_MS);	// 100 milliseconds
			timeout_time = timeout_time - RCV_WAIT;	
			i++;
			}
		else
			{
// if time left < RCV_WAITm, do the remaining time (or 10ms if it is < 10ms)...
			if (timeout_time >= portTICK_PERIOD_MS)
				vTaskDelay(timeout_time / portTICK_PERIOD_MS);	// 100 milliseconds
			else
				vTaskDelay(10 / portTICK_PERIOD_MS);	// 10 milliseconds
				
			timeout_time = 0;
			}
			
		if ((debugflag) && (debug_do(DBG_MQTT)))
			printf("T_time: %d %d   %d %d %d\n",timeout,timeout_time,RCV_WAIT,i,(i+1)*RCV_WAIT);
#endif
		
#ifdef DBG_TEST
printf("Got here 11\n");
#endif

#ifdef SET_PAYLOAD_TIME_TEST
xtime = esp_timer_get_time();
printf("Rx @ [%lld.%06lldsec]\n",xtime/1000000,xtime%1000000);
#endif
	
len = uxQueueMessagesWaiting(mqtt_cmd_queue);

if ((debugflag) && (debug_do(DBG_MQTT)))
	printf("4GRX: ");

#ifdef DBG_TEST
printf("Got here 12\n");
vTaskDelay(10 / portTICK_PERIOD_MS);	// 100 milliseconds
#endif

if (len)
	{	
	x = pdTRUE;
//	while (x == pdTRUE)	//((c != 0x0A) && (x == pdTRUE))
// receive bytes if:
// 		bytes_to_rx has been set to 0 (receive as much as possible in the timeout period)  OR
// 		bytes_to_rx is non-zero and four_g_data_len (actual rx bytes) is less than bytes_to_rx (ie, rx specified number of bytes)
// AND xQueueReceive flag is still pdTRUE
	while (((bytes_to_rx == 0) || (bytes_to_rx > *four_g_data_len)) && ((FOUR_G_RX_BUFSIZE-1) > *four_g_data_len) && (x == pdTRUE))	//((c != 0x0A) && (x == pdTRUE))
		{
		x = xQueueReceive(mqtt_cmd_queue,&c, 10 / portTICK_PERIOD_MS);
		if (x == pdTRUE)
			{
			four_g_data[*four_g_data_len] = c;
			(*four_g_data_len)++;
			}
		}
#ifdef DBG_TEST
printf("Got here 13\n");
#endif
// force while loop expiry if bytes_to_rx was specified as non-zero and we have the desired number of bytes...
	if ((bytes_to_rx != 0) && (*four_g_data_len >= bytes_to_rx))
		timeout_time = 0;
	}

	if ((debugflag) && (debug_do(DBG_MQTT)))
		printf("%d\n",*four_g_data_len);

		}
}

#if 0
/*
else										// only receive "four_g_data_len" bytes
{
	unsigned int rx_timeout;
#ifdef DBG_TEST
printf("Got here 16\n");
#endif
//if (0)	//CMQTT_rx_flag == 0)
	{
	if ((debugflag) && (debug_do(DBG_MQTT)))
		printf("UARTRX: ");
		
// if no complete text lines, grab contents of the UART buffer directly, in case there is a ">", etc
//	*four_g_data_len = get_uart_rx_msg(FOUR_G, four_g_data, FOUR_G_RX_BUFSIZE, 10);
//	*four_g_data_len = *four_g_data_len + uart_read_bytes(FOUR_G, &four_g_data[*four_g_data_len], FOUR_G_RX_BUFSIZE, 100 / portTICK_PERIOD_MS);

//	tmp2 = *four_g_data_len;
	if (bytes_to_rx == 0)
		rx_timeout = timeout;
	else
		rx_timeout = bytes_to_rx/10;
	
	if (rx_timeout < 100)
		rx_timeout = 100;
	
//	*four_g_data_len = uart_read_bytes(FOUR_G, &four_g_data[*four_g_data_len], *four_g_data_len, 100 / portTICK_PERIOD_MS);
	*four_g_data_len = uart_read_bytes(FOUR_G, four_g_data, bytes_to_rx, rx_timeout / portTICK_PERIOD_MS);

		
	if ((debugflag) && (debug_do(DBG_MQTT)))
		printf("%d\n",*four_g_data_len);

	if (*four_g_data_len < tmp)
		printf("Not all data received!\n");
	}
//	printf("RCV: %d\n",*four_g_data_len);

#if 0
    uart_get_buffered_data_len(FOUR_G, &len);	
	if (len)
		printf("%d bytes waiting!\n",len);
#endif

		
}
*/
#endif


if (*four_g_data_len < FOUR_G_RX_BUFSIZE)
	four_g_data[*four_g_data_len] = 0x00;
else
	printf("4G rcv len error!\n");

if (*four_g_data_len >= (FOUR_G_RX_BUFSIZE-1))
	printf("4G rcv buf full error!\n");

if (debugflag)
	{
#ifdef DBG_TEST
printf("Got here 14\n");
#endif		
	if (1)	//*four_g_data_len < 256)
		{
		if (debug_do(DBG_MQTT))
			debug_hex_msg(four_g_data,*four_g_data_len,"Rcv:");
		}
	else
		{
		if (debug_do(DBG_MQTT))
			{
			debug_hex_msg(four_g_data,256,"Rcv part:");
			printf("... ETC...\n");
			}
		}
	}
else
	{
	if (debug_do(DBG_MQTT))
		printf("Rx: [%d %d]\r\n",bytes_to_rx,*four_g_data_len);
	}
#ifdef DBG_TEST
printf("Got here 15\n");
#endif

#endif
	
#if 0
	if (debug_do(DBG_MQTT))
		printf("Rcv End [%d]\n",*four_g_data_len);
#endif
	
return 0;
}

unsigned char four_g_power_en(void)
{
#if 1	//PCB_VER == VER_1_0_D
// 4G psu_enable: hold  ESP32 "FOUR_G_PSU_EN" pin high...
unsigned char err_flag;
// on shift reg pin on V1_0_D, so must use set_output function...
err_flag = set_output(DEV_FOUR_G_PSU_EN,1,0);	// ESP signal is not inverted by transistor
#endif
four_g_psu_en_flag = 1;

return four_g_psu_en_flag;
}

unsigned char four_g_power_disable(void)
{
#if 1	//PCB_VER == VER_1_0_D
// 4G psu_disable: hold  ESP32 "FOUR_G_PSU_EN" pin low...
unsigned char err_flag;
// on shift reg pin on V1_0_D, so must use set_output function...
err_flag = set_output(DEV_FOUR_G_PSU_EN,0,0);	// ESP signal is not inverted by transistor
#endif
four_g_psu_en_flag = 1;

return four_g_psu_en_flag;
}

// state of four_g reset is held by the four_g_rst_flag variable...
unsigned char four_g_hold_in_reset(void)
{
// 4G reset: hold 7600 "RST" pin LOW; so hold ESP32 "FOUR_G_RESET" pin high...
unsigned char err_flag;
// on shift reg pin on V1_0_D, so must use set_output function...
err_flag = set_output(DEV_FOUR_G_RESET,1,0);	// ESP signal is inverted by FET
four_g_rst_flag = 1;

return four_g_rst_flag;

}

unsigned char four_g_release_from_reset(void)
{
unsigned char err_flag;
// on shift reg pin on V1_0_D, so must use set_output function...
err_flag = set_output(DEV_FOUR_G_RESET,0,0);	// ESP signal is inverted by FET
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
unsigned char err_flag;
// on shift reg pin on V1_0_D, so must use set_output function...
err_flag = set_output(DEV_FOUR_G_POWER,1,0);	// ESP signal is inverted by FET
four_g_pwr_flag = 1;

return four_g_pwr_flag;
}

unsigned char four_g_power_key_off(void)
{
// PWR_KEY is active low; FET inverts ESP32 signal; so "FOUR_G_POWER" pin must go low...
unsigned char err_flag;
// on shift reg pin on V1_0_D, so must use set_output function...
err_flag = set_output(DEV_FOUR_G_POWER,0,0);	// ESP signal is inverted by FET
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
			if (debug_do(DBG_4G_TXRX))
				printf("found string <%s>\r\n",str2);
			}
		}
	if (gotline == 2)		// end of input rx string
		{
		found = 1;
		p = NULL;
		if (debug_do(DBG_4G_TXRX))
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
dbgprintf(chan,"4G State:  [%03d] < M4G_%s >    4G timer: %d\r\n",simcom_state,simcom_state_str[simcom_state],simcom_state_mc_timer);
}


unsigned char get_AT_value(char *cmdstr, char *okstr, char *resultstr, char *eolstr, unsigned char line, unsigned char offset, char endchar, char *outstr, unsigned int outlen)
{
// searches for <cmdstr> at start of the first line of the receive data;
// seaches for <chkstr> anywhere in the rcv data
// if line !=0 counts text lines suffixed with <eolstr>, goes to <offset> and retrieves <outlen> chars into result string <outstr>.

// typical use:
// AT+CMD
// OK
//
// +AT+CMD: 1,2

// 1) find cmdstr = AT_CMD on first line
// 2) find okstr = OK on any line
// 3) find results 1,2 at line:offset
// 4) return <outlen> chars of result data

unsigned char ret = 0;
unsigned int i = 0;
unsigned int n;
char j,k;		// only used as condition flags...
char *p;
		if (outstr!= NULL)		// if outstr provided...
			outstr[0] = 0x00;	// preset to NULL str value...
	
		if (strlen(cmdstr) != 0)	// i = 0 if found
			i = strncmp((char *)four_g_rx_data,cmdstr,strlen(cmdstr));
		
		if (strlen(okstr)==0) 
			j = 1;
		else if ((p = strstr((char *)four_g_rx_data,okstr)) != NULL)
			{
			j = 2;
			if (debug_do(DBG_4G_TXRX))
				printf("Found OK string: <%s>\r\n",okstr);
			}
		else
			j = 0;		// j = 0 if not found

		if (strlen(resultstr)==0) 
			k = 1;
		else if ((p = strstr((char *)four_g_rx_data,resultstr)) != NULL)
			{
			k = 2;
			if (debug_do(DBG_4G_TXRX))
				printf("Found result string: <%s>\r\n",resultstr);
			}
		else
			k = 0;		// k = 0 if not found
		
//printf("i=%d j=%d\r\n",i,j);

		if ((i==0) && (j) && (k))		// if cmd str found in rx and OK string found or not required
			{	
//			printf("0>?");

			p = (char *)four_g_rx_data;
			if (line)
				{
				for (n=0;n<line;n++)
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
					n = 0;
					p = p + 1 + offset;
					while ((*p!=endchar) && (n < outlen))
						{
						outstr[n++] = *(p++);
						}
					outstr[n] = 0x00;
					if (debug_do(DBG_4G_TXRX))
						printf("Found [%s]: %s\r\n",cmdstr,outstr);
					
					ret = 1;
					}
				else
					{
					if (debug_do(DBG_4G_TXRX))
						printf("Cmd [%s] complete\r\n",cmdstr);
					
					ret = 1;
					}
					
//				*four_g_state = M4G_CLOSEALL;
				}
			}

		if (debug_do(DBG_4G_TXRX))
			{
			if (i)
				printf("%s not found\r\n",cmdstr);
			if (j==0)
				printf("%s not found\r\n",okstr);
			if (k==0)
				printf("%s not found\r\n",resultstr);
			}
return ret;
}
//uart_enable_pattern_det_baud_intr(UART_NUM_1, 0x0a, 1, 9, 0, 0); // pattern is LF

unsigned char get_str(char *instr, char *str, char *outstr, unsigned char dbg_flag)
{
// find str in instr; copy instr (up to str found point) into outstr; return length of outstr
unsigned char i = 0;
char c;
char *ptr;

ptr = strstr(instr,str);

if (ptr != NULL)
	{
	i = ptr - instr;

	strncpy(outstr,instr,i);
	outstr[i] = 0x00;
//	i++;

	}
else
	{
//	printf("cant find str!\r\n");
	strcpy(outstr,instr);
	i = strlen(outstr);
	}

if(dbg_flag)
	printf("GET_STR:[%d] < %s >\r\n",i,outstr);
	
return i;
}
