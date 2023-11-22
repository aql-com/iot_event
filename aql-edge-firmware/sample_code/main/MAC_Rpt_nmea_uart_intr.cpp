//////////////////////////////////////////////
//
// MAC_Rpt_nmea_uart_intr.cpp
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

#include <stdio.h>
#include <string.h>
#include <memory.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"


#include "driver/uart.h"

#include "bluetooth.h"	// host_rcv_data_t definition
#include "MAC_Rpt.h"

//#include "MAC_Rpt_four_g_mqtt_defs.h"
#include "MAC_Rpt_four_g_mqtt.h"

extern QueueHandle_t nmea_data_queue;

extern QueueHandle_t nmea_uart_queue;
extern unsigned char nmea_msg_rx_count;

extern unsigned char mqtt_login_state;

extern char eolstr[10];
extern unsigned char binary_mode_flag;
extern unsigned char eol_remove_flag;

extern unsigned char queues_enabled_flag;

//extern unsigned char nmea_rx_flag;
//extern unsigned char nmea_rx_msg;

void nmea_uart_task(void *pvParameters)
{
    int uart_num = (int) pvParameters;
    uart_event_t event;
    size_t buffered_size, rlen;
	size_t blen[10];
	unsigned char nmea_linenum;
	char tmpstr[50];
	char srchstr1[15];
	char srchstr2[15];
//	unsigned char endflag;
	char prt_str[256];
	unsigned int num_chars = 0;

		
//    uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE);
    for(;;) 
		{
//Waiting for UART event; test event queue (NOT data queue!)
        if(xQueueReceive(nmea_uart_queue, (void * )&event, (portTickType)portMAX_DELAY)) 		// if uart event...
			{
//            ESP_LOGI(TAG, "uart[%d] event:", uart_num);
            switch(event.type) 
				{
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
//                    ESP_LOGI(TAG, "hw fifo overflow\n");
                    //If fifo overflow happened, you should consider adding flow control for your application.
                    //We can read data out out the buffer, or directly flush the rx buffer.
                    uart_flush((uart_port_t)uart_num);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
//                    ESP_LOGI(TAG, "ring buffer full\n");
                    // increase the buffer size!
                    //Can read data out of the buffer, or flush the buffer.
                    uart_flush((uart_port_t)uart_num);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
//                    ESP_LOGI(TAG, "uart rx break\n");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
//                    ESP_LOGI(TAG, "uart parity error\n");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
//                    ESP_LOGI(TAG, "uart frame error\n");
                    break;
                //UART_PATTERN_DET
                case UART_DATA:
					num_chars++;
					if (num_chars < (MQTT_PAYLOAD_SIZE/(1 + binary_mode_flag)))
						break;
//					ELSE buffer full! Drop through and send data					
//                    uart_get_buffered_data_len(uart_num, &buffered_size);
//                    ESP_LOGI(TAG, "data, len: %d; buffered len: %d", event.size, buffered_size);
                case UART_PATTERN_DET:
//                    ESP_LOGI(TAG, "uart pattern detected\n");
//					printf("$");
//					nmea_msg_rx_count++;
#if 1
//#define NMEA_Q_DEBUG
//#define NMEA_Q_DEBUG2
#define NMEA_Q_DEBUG3
					uart_pattern_pop_pos((uart_port_t)uart_num);			// removes position info from position queue and stops queue overflow...

// get data len and create temporary buffer
					uart_get_buffered_data_len((uart_port_t)uart_num, &buffered_size);
					if (buffered_size > 0)
					{
					char c;
					char *pos;					
					unsigned int strpos,i;
					unsigned char flag,j;
					
					char* dtmp = (char*) malloc(buffered_size+1);	

					if (dtmp != NULL)
						{
#ifdef NMEA_Q_DEBUG
						printf("***\n");
#endif
//						get_uart_rx_msg(FOUR_G, dtmp, buffered_size, 10);
						rlen = uart_read_bytes((uart_port_t)uart_num, (uint8_t*)dtmp, buffered_size, 10 / portTICK_PERIOD_MS);
//						endflag = 0;
						if (rlen != buffered_size)
							printf("$$");
							
// get preview of buffer contents...
//						xQueuePeek(four_g_uart_queue, (void * )dtmp, (portTickType)portMAX_DELAY);
						dtmp[buffered_size] = 0x00;

//						printf("%d[%d] %s\n",uart_num,buffered_size,dtmp);

// look for "+CMQTTRX" string
//						if (nmea_rx_flag != 2)
						if (0)
							{
							strcpy(srchstr1,"$G");
							if (!strncmp(dtmp,srchstr1,strlen(srchstr1)))		// does line start with "$G" ?
								{
								pos = strstr(dtmp,srchstr1);	// "SIMCOM"
								if (pos != NULL)
									{
#ifdef NMEA_Q_DEBUG
									printf("*** Found %s !\n",srchstr1);
#endif
									strpos = pos - dtmp + strlen(srchstr1);	// "SIMCOM"	// get start of next part of string

//									nmea_rx_flag = 1;
//									endflag = 1;
//									nmea_msg_rx_count++;
									}							
								}
							else
								{
//								nmea_rx_flag = 0;	// cmd data
								}
							}
/*
						else		// flag is = 2
							{
//							nmea_rx_flag  = 1;		// get following line of text
							}
*/
#ifdef NMEA_Q_DEBUG2
						printf("%d[%d,%d] %s\n",uart_num,nmea_msg_rx_count,buffered_size,dtmp);
#endif
//						nmea_msg_rx_count++;
							

/*
if (nmea_rx_flag == 0)		// SIMCOM command responses
							{
							while ((c != 0x0A) && (j != pdFALSE))
								{
								c = dtmp[i];
								j = xQueueSend(mqtt_cmd_queue, &c, 10 / portTICK_PERIOD_MS);
								i++;
								}								
							}
						else				// SIMCOM unsolicited receive data
*/
							{
							host_rcv_data_t send_data;
//						i = 0;
//						j = pdTRUE;
//						c = 0xFF;
//							printf("$$$Msg:");

/*
							while ((c != 0x0A) && (j != pdFALSE))
								{
//								send_data.q_data = msg_out;
//								send_data.q_data_len = 8 + data_len[i];
//								if (xQueueSend(out_queue, (void *)&send_data, ( TickType_t ) 0) != pdTRUE) 
									
								c = dtmp[i];
								j = xQueueSend(nmea_data_queue, &c, 10 / portTICK_PERIOD_MS);
								printf("%c",c);
								i++;
								}	
*/

#ifdef NMEA_Q_DEBUG3
							strncpy(prt_str,dtmp,15);
							prt_str[15] = 0x00;
//							printf("$$$Msg: %s [%d] {%d}\n",prt_str,buffered_size,nmea_msg_rx_count);	// dtmp
#endif

#ifdef USE_M4G_MQTT
//							if (mqtt_login_state > UPDATE_SRVR_INFO_WAIT)	// if connected and ready to send
							if (queues_enabled_flag)
								{
								send_data.q_data = (uint8_t*)dtmp;
								if (eol_remove_flag)
									send_data.q_data_len = buffered_size - 2;		// remove CR-LF at end if eol_remove_flag set to 1...
								else
									send_data.q_data_len = buffered_size;			// do not remove CR-LF at end if eol_remove_flag set to 0...
								
								if (send_data.q_data[send_data.q_data_len-1] == eolstr[0])	//0x0A)
									printf("@@");
									
// SHOULD THIS BE XQUEUESENDFROMISR? SEE METER READ...
								if (xQueueSend(nmea_data_queue, (void *)&send_data, ( TickType_t ) 0) != pdTRUE) 	// if malloc'ed area not properly enqueued...
									{
									free(dtmp);																		// ...remove it
									}
								else
									{
									nmea_msg_rx_count++;															// else increment msg count for fg code
									}									
								

#ifdef NMEA_Q_DEBUG3
/*
								strncpy(prt_str,dtmp,15);
								prt_str[15] = 0x00;
								printf("$$$Msg: %s [%d] {%d}\n",prt_str,buffered_size,nmea_msg_rx_count);	// dtmp
*/
#endif
								}
							else											// not connected and ready to send
#endif// end of "#ifdef USE_M4G_MQTT...
								{
								xQueueReset(nmea_data_queue);				// so dump any enqueued data
								nmea_msg_rx_count = 0;
								free(dtmp);
								}

							num_chars = 0;									// reset waiting character count

#ifdef NMEA_Q_DEBUG3
// now we have the msg_rx_count, send the printf msg...
							printf("$$$Msg: %s [%d] {%d}\n",prt_str,buffered_size,nmea_msg_rx_count);	// dtmp
#endif

/*								
							if (endflag)
								{
								nmea_rx_msg = 1;
								nmea_msg_rx_count = 0;
								}
*/
/*
if (CMQTT_rx_flag == 2)
								{
								CMQTT_rx_flag = 1;	
								}
*/
							}

//						free(dtmp);
//						dtmp = NULL;
						}						
					}	
					
#endif						
/*
if (four_g_linenum)
						{
						blen[buffered_size] = blen[buffered_size-1]
						}
					else
						{
						blen[0] = buffered_size;
						}
*/					

                    break;
                case UART_DATA_BREAK:
                    break;
                case UART_EVENT_MAX:
                    break;


                //Others
                default:
//                    ESP_LOGI(TAG, "uart event type: %d\n", event.type);
                    break;
				}
			}
		}
//    free(dtmp);
//    dtmp = NULL;
    vTaskDelete(NULL);
}
