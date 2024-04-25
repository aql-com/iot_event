//////////////////////////////////////////////
//
// MAC_Rpt_ZULU_radio.cpp
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
#include "MAC_Rpt_ZULU_radio.h"



extern unsigned char zulu_radio_msgs_flag;


unsigned char send_zulu_radio_msg(unsigned char *tx_data, unsigned char dev_num, unsigned char seq_num, unsigned char msg_type, unsigned char tx_data_len)
{
unsigned char ret, errcheck;

printf("ZT\n");

// add prefix
	tx_data[MSG_START_BYTE] = STX;	
	tx_data[DEV_NUM_BYTE]   = dev_num;										// device number
	tx_data[SEQ_NUM_BYTE]   = seq_num;						// msg sequence number
	tx_data[MSG_TYPE_BYTE]  = msg_type;							// message type
	tx_data[MSG_LEN_BYTE]   = DATA_START_BYTE + tx_data_len;	// message type

// add suffix
	tx_data[DATA_START_BYTE + tx_data_len] = CR;	
	tx_data[DATA_START_BYTE + tx_data_len + 1] = LF;	
	
	errcheck = 0;
	for (unsigned char i=1;i<(DATA_START_BYTE + tx_data_len + 2);i++)
		{
		errcheck = errcheck + tx_data[i];
		}

	tx_data[DATA_START_BYTE + tx_data_len + 2] = errcheck;	
	tx_data[DATA_START_BYTE + tx_data_len + 3] = ETX;	

	if (zulu_radio_msgs_flag)
		{
		show_time(DBG,1);
		printf("  Radio ");
		if (dev_num == ZULU_RADIO_MASTER_ID)
			printf("Master Tx CMD:");
		else
			printf("Slave Tx RESPONSE:");
		
		printf(" [%s]  [%d]\n",zulu_radio_msg_names[msg_type],DATA_START_BYTE + tx_data_len + 4);		
			
		printf("STX DEV SEQ TYP LEN DATA");
		for (unsigned char i=0;i<(tx_data_len);i++)
			{
			printf("----");
			}
		printf("->  CHK ETX\n");
		
		for (unsigned char i=0;i<(DATA_START_BYTE + tx_data_len + 4);i++)
			{
			printf("%02X  ",tx_data[i]);
			}
		printf("\n\n");
		}
		
	ret = uart_write_bytes(ZULU_RADIO, (char *)tx_data, DATA_START_BYTE + tx_data_len + 4);


return ret;
}