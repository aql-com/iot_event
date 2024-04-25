//////////////////////////////////////////////
//
// MAC_Rpt_radio.c
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
#include "sdkconfig.h"
#include "esp_wifi_types.h"

#include "MAC_Rpt.h"
#include "MAC_Rpt_rtns.h"
#include "MAC_Rpt_radio.h"

extern uint8_t lora_radio_tx_data[13];
extern uint8_t lora_radio_rx_data[LORA_RADIO_RX_BUFSIZE];
extern unsigned int lora_radio_rx_length;

extern unsigned char lora_radio_rst_flag;
extern unsigned char lora_radio_connected_flag;

int lora_radio_send_data(uint8_t *lora_radio_data, unsigned int lora_radio_data_len)
{
int ret = 0;

	ret = uart_write_bytes(RADIO, (char *)lora_radio_data, lora_radio_data_len);

	printf("RADIO: sending <%s>\r\n",(char *)lora_radio_data);
	printf("RADIO: result: %d\r\n",lora_radio_data_len);

return ret;
}

// state of radio reset is held by the lora_radio_rst_flag variable...
unsigned char lora_radio_hold_in_reset(void)
{
// radio reset is inverted
unsigned char error;

//error = gpio_set_level(LORA_RADIO_RESET,1);
set_output(DEV_LORA_RADIO_RESET,DEVICE_ON,INVERTED);
//    gpio_set_direction(LORA_RADIO_RESET, GPIO_MODE_OUTPUT);
lora_radio_rst_flag = 1;

return lora_radio_rst_flag;
}

unsigned char lora_radio_release_from_reset(void)
{
// radio reset is inverted
unsigned char error;

//error = gpio_set_level(LORA_RADIO_RESET,0);
set_output(DEV_LORA_RADIO_RESET,DEVICE_OFF,INVERTED);
//    gpio_set_direction(LORA_RADIO_RESET, GPIO_MODE_INPUT);

lora_radio_rst_flag = 0;

return lora_radio_rst_flag;
}

unsigned char lora_radio_reset_state(void)
{

return lora_radio_rst_flag;
}

unsigned char lora_radio_check(unsigned char retries)
{
unsigned char c,i;
unsigned int j = 0;
int x;

strcpy((char *)lora_radio_tx_data,"AT+CGMI?\r");

for (i=0;i<retries;i++)
	{
	lora_radio_send_data(lora_radio_tx_data, strlen((char *)lora_radio_tx_data));

	lora_radio_rx_length = get_uart_rx_msg(RADIO, lora_radio_rx_data, LORA_RADIO_RX_BUFSIZE, 100);

	printf("4G: %d\r\n",lora_radio_rx_length);
	for (j=0;j<lora_radio_rx_length;j++)
		{
		if ((lora_radio_rx_data[j] > 0x1F) && (lora_radio_rx_data[j] < 0x80))
			c = lora_radio_rx_data[j];
		else
			c = '.';
		printf("%c  ",c);
		
		}
	printf("\r\n");

	for (j=0;j<lora_radio_rx_length;j++)
		{
		printf("%02X ",lora_radio_rx_data[j]);
		
		}
	printf("\r\n");
	
	x = strcmp((char *)lora_radio_rx_data,"\r\n+CGMI?=Ebyte\r\nOK\r\n");
	if ((lora_radio_rx_length) && (x==0))
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
	printf("Radio connected\r\n");
	lora_radio_connected_flag = 1;
	}
else
	{
	printf("Radio not connected\r\n");
	lora_radio_connected_flag = 0;
	}
	
return lora_radio_connected_flag;
}

