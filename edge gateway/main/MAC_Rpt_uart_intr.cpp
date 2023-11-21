//////////////////////////////////////////////
//
// MAC_Rpt_uart_intr.cpp
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

#include <stdio.h>
#include <string.h>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "driver/uart.h"

#include "MAC_Rpt.h"
#include "MAC_Rpt_Rtns.h"
#include "MAC_Rpt_four_g.h"

extern QueueHandle_t mqtt_cmd_queue;
extern QueueHandle_t mqtt_data_queue;

extern QueueHandle_t four_g_uart_queue;
extern unsigned char four_g_msg_rx_count;

extern unsigned char CMQTT_rx_flag;
extern unsigned char CMQTT_rx_msg;
extern unsigned char four_g_response_flag, four_g_eol_response_flag;
extern unsigned char simcom_response_flag, simcom_eol_response_flag;
extern unsigned char mqtt_response_flag, mqtt_eol_response_flag;
extern unsigned char udp_response_flag, udp_eol_response_flag;

extern char pattern_detect_char;
extern unsigned int four_g_timeout_timer;

extern int64_t pcmqtt_time;

extern unsigned char mqtt_transport_mode;

unsigned int xi;
unsigned char xstep;

size_t dtmp_size;

// these were extern defined & initialised in main{}...
char* dtmp = NULL;
char* dleft = NULL;
size_t dleft_size = 0;
size_t dleft_pos = 0;
size_t dprev_size = 0;
unsigned char mqtt_step = 0;		

void four_g_uart_task(void *pvParameters)
{
#ifdef USE_M4G_MQTT

    int uart_num = (int) pvParameters;
    uart_event_t event;
    size_t buffered_size;
//	size_t blen[10];
	unsigned char four_g_linenum;		// #def'd out
//	char tmpstr[50];		// #def'd out
	char srchstr1[15];		// only used to find +CMQTTRX msg
	char srchstr2[100];		// inx limited to 100
	unsigned char endflag;
//	char tststr[10];

	size_t strpos;
	unsigned char cmqtt_rx_found;
	size_t inx;
	unsigned char found_flag;
    size_t linestart = 0;
    size_t linelen = 0;
	char strp[80];			// diag only, inx limited to 80 chars

	char c;
	char *pos;					
	unsigned int i,k;
	unsigned char flag,j;

	char msgbyte;
	char lastchar;
	char pathchar;

	int rx_bytes;
	
	
	unsigned int xbuffered_size;
	char* xtmp = NULL;
	int64_t xtime;
	
// Algorithm:
//
// 1) make data block from residual data from last pass + new data
// 2) scan for line end; if found, somehow attach line end used for this msg... send msg and loop until no more line endings
// 3) store residual data after last line end
	
static const char* TAG = "UInt";

//#define SIMCOM_DATA		0
//#define CMQTTRX_CMD		1
//#define CMQTTRX_DATA		2

// initialise flag to expect SIMCOM data...
	CMQTT_rx_flag = SIMCOM_DATA;	
	lastchar = 0x00;
	pathchar = 'D';
	xstep = 0;
	
//    uint8_t* dtmp = (uint8_t*) malloc(BUF_SIZE);
    for(;;) 
		{
//Waiting for UART event; test event queue (NOT data queue!)
        if(xQueueReceive(four_g_uart_queue, (void * )&event, (portTickType)portMAX_DELAY)) 		// if uart event...
			{
//            ESP_LOGI(TAG, "%d %d", uart_num,event.type);
#if 0
    UART_DATA,              /* 0 !< UART data event*/
    UART_BREAK,             /* 1 !< UART break event*/
    UART_BUFFER_FULL,       /* 2 !< UART RX buffer full event*/
    UART_FIFO_OVF,          /* 3 !< UART FIFO overflow event*/
    UART_FRAME_ERR,         /* 4 !< UART RX frame error event*/
    UART_PARITY_ERR,        /* 5 !< UART RX parity event*/
    UART_DATA_BREAK,        /* 6 !< UART TX data and break event*/
    UART_PATTERN_DET,       /* 7 !< UART pattern detected */
    UART_EVENT_MAX,         /* 8 !< UART event max index*/
#endif
            switch(event.type) 
				{
                case UART_DATA:
// if we arrive here its because the UARtr has filled its hardware buffer and the UART_PATT_DETECT has not occurred; 
// this data is just a part of the current message, so carry on sending it to the stream indicated by the CMQTT_rx_flag

// Also need to gather any left-over data from previous loop and put that at the front...

#if 1	// extra code for blocks > hardware buffer size which dont contain a special character
// get data len and create temporary buffer
					uart_get_buffered_data_len((uart_port_t)uart_num, &buffered_size);

					if (buffered_size > 0)
					{
//					char c;
//					char *pos;					
//					unsigned int strpos,i;
//					unsigned char flag,j;
					
//					char* dtmp = (char*) malloc(buffered_size+1);	
					dtmp = (char*) malloc(buffered_size + dleft_size + 1);	
					dtmp_size = buffered_size + dleft_size + 1;
					
					if (dtmp != NULL)
						{
						unsigned char cmqtt_found_flag = 0;
//						printf("***\n");

						cmqtt_rx_found = 0;

#ifdef NEW_SERIAL_DBG1
					printf("New Serial - alloc OK\n");
#endif

					dprev_size = dleft_size;

// if any data left from previous loop copy into buffer first, and free old memory block
					if (dleft_size)
						{
#ifdef NEW_SERIAL_DBG1
						printf("New Serial - free dleft %d\n",dleft_size);
#endif

#ifdef NEW_SERIAL_DBG2
						debug_hex_msg((uint8_t*)dleft+dleft_pos,dleft_size,"DLEFT:");
#endif
						if (dleft != NULL)
							{
							memcpy(dtmp,dleft+dleft_pos,dleft_size);
							free(dleft);
							dleft = NULL;
							dleft_size = 0;
							dleft_pos = 0;
							}
#ifdef NEW_SERIAL_DBG1
						else
							printf("Dleft was NULL!\n");
#endif
						}
// AT THIS POINT dleft_size = 0, so dont need in any code...
									
// get new data into allocated memory
#ifdef NEW_SERIAL_DBG1
				printf("New Serial - read bytes %d\n",buffered_size);
#endif


//						get_uart_rx_msg(FOUR_G, dtmp, buffered_size, 10);
						rx_bytes = uart_read_bytes((uart_port_t)uart_num, (uint8_t*)(dtmp+dprev_size), buffered_size, 10 / portTICK_PERIOD_MS);
//&&						rx_bytes = uart_read_bytes((uart_port_t)uart_num, (uint8_t*)dtmp, buffered_size, 10 / portTICK_PERIOD_MS);
						endflag = 0;
//$$						if (rx_bytes != buffered_size)
//$$							printf("### RXB BUF %d %d\n",rx_bytes,buffered_size);

// left-over data and new data now in dtmp: just send to current selected stream:
									i = 0;
									j = pdTRUE;
									c = 0xFF;
									k = buffered_size+dprev_size;

									if (CMQTT_rx_flag == SIMCOM_DATA)		// SIMCOM command responses
										{
#ifdef NEW_SERIAL_DBG3
										printf("SIMCOM: [%d %d] ",linestart,linelen);
#endif
										pathchar = 'D';
	//									while ((i<buffered_size) && (c != 0x0A) && (j != pdFALSE))
										while ((i<k) && (j != pdFALSE))
											{
											c = dtmp[i];
											j = xQueueSend(mqtt_cmd_queue, &c, 10 / portTICK_PERIOD_MS);
											i++;
											lastchar = c;
#ifdef NEW_SERIAL_DBG3
											if ((c>0x1F) && (c<0x80))
												printf("%c",c);
											else
												printf(".");
#endif
											}								
#ifdef SET_PAYLOAD_TIME_TEST
										xtime = esp_timer_get_time();
										printf("[%lld.%06lldsec]",xtime/1000000,xtime%1000000);
#endif
#ifdef NEW_SERIAL_DBG3
										printf("\n");
#endif
										}
									else				// SIMCOM unsolicited receive data
										{
#ifdef NEW_SERIAL_DBG3
										printf("UNSOLICITED: [%d %d] ",linestart,linelen);
#endif
										pathchar = 'C';
//										printf("## CM_RX\n");
										
	//									while ((i<buffered_size) && (c != 0x0A) && (j != pdFALSE))
										while ((i<k) && (j != pdFALSE))
											{
											c = dtmp[i];
											j = xQueueSend(mqtt_data_queue, &c, 10 / portTICK_PERIOD_MS);
											i++;
											lastchar = c;
#ifdef NEW_SERIAL_DBG3
											if ((c>0x1F) && (c<0x80))
												printf("%c",c);
											else
												printf(".");
#endif
											}	
#ifdef SET_PAYLOAD_TIME_TEST
										xtime = esp_timer_get_time();
										printf("[%lld.%06lldsec]",xtime/1000000,xtime%1000000);
#endif
#ifdef NEW_SERIAL_DBG3
										printf("\n");
#endif
										}

// sent all the data - now clean up...
// free the memory block
#ifdef NEW_SERIAL_DBG1
								printf("New Serial - free dtmp\n");
#endif
							
							
								free(dtmp);
								dtmp = NULL;


						}
					}
#endif		// end of extra code...

//                    uart_get_buffered_data_len((uart_port_t)uart_num, &buffered_size);
//                    ESP_LOGI(TAG, "data, len: %d; buffered len: %d", event.size, buffered_size);
/*
					{
					char* dtmp = (char*) malloc(event.size);	//(buffered_size+1);	
					uart_read_bytes((uart_port_t)uart_num, (uint8_t*)dtmp, event.size, portMAX_DELAY);
                    uart_write_bytes((uart_port_t)uart_num, (const char*) dtmp, event.size);
					free(dtmp);
					}
*/					
//						printf("$");

//					xQueuePeek(four_g_uart_queue, (void * )tststr, (portTickType)portMAX_DELAY);
//					if (tststr[0] == '>')
//					xQueuePeek(four_g_uart_queue, &msgbyte, (portTickType)portMAX_DELAY);
//					if (msgbyte == '>')

//						printf("***Got >\n");
//					else
//						printf("*");

#if 0
                    uart_get_buffered_data_len((uart_port_t)uart_num, &xbuffered_size);
					xtmp = (char*) malloc(xbuffered_size);	//(buffered_size+1);	
					xQueuePeek(four_g_uart_queue, (void * )xtmp, (portTickType)portMAX_DELAY);
					xi = 0;

					printf("## CM? %d ##\n",xbuffered_size);
					while (xi<xbuffered_size)
						{
//						if ((xtmp[xi] == '+') && (xstep == 0))
						if (xtmp[xi] == '+')
							xstep = 1;
						else if ((xtmp[xi] == 'C') && (xstep == 1))
							xstep = 2;
						else if ((xtmp[xi] == 'M') && (xstep == 2))
							xstep = 3;
						else if ((xtmp[xi] == 'Q') && (xstep == 3))
							xstep = 4;
						else if ((xtmp[xi] == 'T') && (xstep == 4))
							xstep = 5;
						else if ((xtmp[xi] == 'T') && (xstep == 5))
							xstep = 6;
						else if ((xtmp[xi] == 'R') && (xstep == 6))
							xstep = 7;
						else if ((xtmp[xi] == 'X') && (xstep == 7))
							{
							printf("## CMF ##\n");
							xstep = 0;
							}
						else
							xstep = 0;

						xi++;
						}
					

					free(xtmp);
#endif
					
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
//                    ESP_LOGI(TAG, "uart rx break\n");
                    break;
                 //Event of UART ring buffer full
                case UART_BUFFER_FULL:
//                    ESP_LOGI(TAG, "ring buffer full\n");
                    // increase the buffer size!
                    //Can read data out of the buffer, or flush the buffer.
                    uart_flush((uart_port_t)uart_num);
                    break;
               //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
//                    ESP_LOGI(TAG, "hw fifo overflow\n");
                    //If fifo overflow happened, you should consider adding flow control for your application.
                    //We can read data out out the buffer, or directly flush the rx buffer.
                    uart_flush((uart_port_t)uart_num);
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
//                    ESP_LOGI(TAG, "uart frame error\n");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
//                    ESP_LOGI(TAG, "uart parity error\n");
                    break;
                //UART_PATTERN_DET
                case UART_DATA_BREAK:
                    break;
                case UART_PATTERN_DET:
//                    ESP_LOGI(TAG, "uart pattern detected\n");
//					printf("$");

/*
					uart_pattern_pop_pos((uart_port_t)uart_num);

					if (pattern_detect_char == 0x0A)
					{
					four_g_msg_rx_count++;
*/
#if 1
// ### NOW IN MAC_Rpt.h...!
//#define QINT_DEBUG
//#define QINT_DEBUG2
//#define QINT_DEBUG3
//#define NEW_SERIAL_DBG1		// program labels
//#define NEW_SERIAL_DBG2		// show data
//#define NEW_SERIAL_DBG3		// show outgoing to fg

// get data len and create temporary buffer
					uart_get_buffered_data_len((uart_port_t)uart_num, &buffered_size);

					if (buffered_size > 0)
					{
//					char c;
//					char *pos;					
//					unsigned int strpos,i;
//					unsigned char flag,j;
					
//					char* dtmp = (char*) malloc(buffered_size+1);	
					dtmp = (char*) malloc(buffered_size + dleft_size + 1);	
					dtmp_size = buffered_size + dleft_size + 1;
					
					if (dtmp != NULL)
						{
						unsigned char cmqtt_found_flag = 0;
//						printf("***\n");

						cmqtt_rx_found = 0;

#ifdef NEW_SERIAL_DBG1
					printf("New Serial - alloc OK\n");
#endif

					dprev_size = dleft_size;

// if any data left from previous loop copy into buffer first, and free old memory block
					if (dleft_size)
						{
#ifdef NEW_SERIAL_DBG1
						printf("New Serial - free dleft %d\n",dleft_size);
#endif

#ifdef NEW_SERIAL_DBG2
						debug_hex_msg((uint8_t*)dleft+dleft_pos,dleft_size,"DLEFT:");
#endif
						if (dleft != NULL)
							{
							memcpy(dtmp,dleft+dleft_pos,dleft_size);
							free(dleft);
							dleft = NULL;
							dleft_size = 0;
							dleft_pos = 0;
							}
#ifdef NEW_SERIAL_DBG1
						else
							printf("Dleft was NULL!\n");
#endif
						}
// AT THIS POINT dleft_size = 0, so dont need in any code...
									
// get new data into allocated memory
#ifdef NEW_SERIAL_DBG1
				printf("New Serial - read bytes %d\n",buffered_size);
#endif

//						get_uart_rx_msg(FOUR_G, dtmp, buffered_size, 10);
						rx_bytes = uart_read_bytes((uart_port_t)uart_num, (uint8_t*)(dtmp+dprev_size), buffered_size, 10 / portTICK_PERIOD_MS);
//&&						rx_bytes = uart_read_bytes((uart_port_t)uart_num, (uint8_t*)dtmp, buffered_size, 10 / portTICK_PERIOD_MS);
						endflag = 0;
//$$						if (rx_bytes != buffered_size)
//$$							printf("### RXB BUF %d %d\n",rx_bytes,buffered_size);
						
						if (rx_bytes >  0)	// 0 = no data; -ve = error
							{
						
// get preview of buffer contents...
//						xQueuePeek(four_g_uart_queue, (void * )dtmp, (portTickType)portMAX_DELAY);
//							dtmp[buffered_size+dleft_size] = 0x00;
							dtmp[dprev_size + rx_bytes] = 0x00;
//&&							dtmp[buffered_size] = 0x00;
//						printf("%d[%d] %s\n",uart_num,buffered_size,dtmp);


							inx = 0;
							found_flag = 0;
							linestart = 0;
		
// loop until cant find CR-LF or '>'		
//							while (inx < (buffered_size + dleft_size))
//							while (inx < (dtmp_size - 1))
							while (inx < (dprev_size + rx_bytes))	/// - 1))
//&&							while (inx < buffered_size)
								{
								c = *(dtmp+inx);
								
								if (c == 0x0A)
									{
									found_flag = FOUND_EOL;
									four_g_response_flag = RESP_NONE;
									
									linelen = inx + 1 - linestart;
									
//									dleft_size = buffered_size - (inx + 1);
									dleft_size = dprev_size + rx_bytes - (inx + 1);
									dleft_pos = inx+1;						
//$$
/*
									if (linelen > 80)
										printf("### LL!\n");	// FOUND THE ERROR!


									memcpy(strp,&dtmp[linestart],linelen);
									strp[linelen] = 0x00;
*/
// strp now only used for diagnostic printf below...
									{
									unsigned char x;
									
									if (linelen < 80)
										x = linelen;
									else
										x = 79;
									memcpy(strp,&dtmp[linestart],x);
									strp[x] = 0x00;
									}

									cmqtt_found_flag = 0;
									cmqtt_rx_found = 0;


//$$									if (!strncmp(strp,"OK",2))		// does line start with "+CMQTTRX" ?
									if (!strncmp(&dtmp[linestart],"OK",2))		// does line start with "+CMQTTRX" ?
										{
										four_g_response_flag = RESP_OK;			// defined in MAC_Rpt_four_g.h....
										if (debug_do(DBG_MQTT))
											printf("&&& Found OK!\n");
										}
//$$									else if (!strncmp(strp,"ERROR",5))		// does line start with "+CMQTTRX" ?
									else if (!strncmp(&dtmp[linestart],"ERROR",5))		// does line start with "+CMQTTRX" ?
										{
										four_g_response_flag = RESP_ERROR;
										if (debug_do(DBG_MQTT))
											printf("&&& Found ERROR!\n");
										}
//$$									else if (!strncmp(strp,"+CMQTT",6))				// does line start with "+CMQTT" ? and not CMQTTRX?
									else if (!strncmp(&dtmp[linestart],"+CMQTT",6))				// does line start with "+CMQTT" ? and not CMQTTRX?
										{
//$$										if (strncmp(strp,"+CMQTTRX",8))				// not "+CMQTTRX"
										if (strncmp(&dtmp[linestart],"+CMQTTRX",8))				// not "+CMQTTRX"
											{
											four_g_response_flag = RESP_PCMQTT;
											if (debug_do(DBG_MQTT))
												printf("&&& Found +CMQTT!\n");
											
											//printf("%s\n",dtmp);
											cmqtt_found_flag = 1;
											cmqtt_rx_found = 0;
											pcmqtt_time = esp_timer_get_time();
											}
										else
											{
											cmqtt_rx_found = 1;
											cmqtt_found_flag = 0;
											if (debug_do(DBG_MQTT))
												printf("&&& Found +CMQTTRX!\n");
											}

										}
									else 				// all other lines
										{
										four_g_eol_response_flag = RESP_EOL;
										simcom_eol_response_flag = RESP_EOL;
										mqtt_eol_response_flag = RESP_EOL;
										udp_eol_response_flag = RESP_EOL;

			//							printf("&&& Found EOL!\n");
										}

									simcom_response_flag = four_g_response_flag;
									mqtt_response_flag = four_g_response_flag;
									udp_response_flag = four_g_response_flag;
									
#ifdef QINT_DEBUG2
									printf("FL:[%d,%d,%d] %s\n",cmqtt_found_flag,cmqtt_rx_found,CMQTT_rx_flag,strp);
#endif
							
// if unsolicited data stream found (signified by +CMQTTRX header)...
									if (CMQTT_rx_flag != CMQTTRX_DATA)		// 0 = data, 	1 = check following line of text - should be CMQTTRX cmd,	2 = following line of text is CMQTTRX data - dont process
										{
										strcpy(srchstr1,"+CMQTTRX");
										if (cmqtt_found_flag == 1)
											{
											if (debug_do(DBG_MQTT))
												{
												strncpy(srchstr2,dtmp,99);
												srchstr2[99] = 0x00;
												printf("Str: < %s >\n", srchstr2);
												}
											
											}
//$$										if (!strncmp(strp,srchstr1,strlen(srchstr1)))		// does line start with "+CMQTTRX" ?
										if (!strncmp(&dtmp[linestart],srchstr1,strlen(srchstr1)))		// does line start with "+CMQTTRX" ?
											{
//$$											pos = strstr(strp,srchstr1);	// "SIMCOM"
											pos = strstr(&dtmp[linestart],srchstr1);	// "SIMCOM"
											if (pos != NULL)
												{
#ifdef QINT_DEBUG
												printf("*** Found %s !\n",srchstr1);
#endif
												strpos = strlen(srchstr1);	// get start of next part of string after "+CMQTTRX"
#ifdef QINT_DEBUG
#if 0
// get next part of string
												c = dtmp[strpos];
												i = 0;
												while ((c != ':') && (c!= 0x00))
													{
													tmpstr[i] = c;
													i++;
													c = dtmp[strpos+i];
													}
												tmpstr[i] = 0x00;
									
//							printf("TMP:%d %s\n",strpos,tmpstr);
//							printf("CMP:%d %s\n",i,&dtmp[strpos]);
#endif
												printf("##: %s\n",&dtmp[strpos]);

#endif
//$$												if (!strncmp(&strp[strpos],"START",5))	//_SIM7600E"))	//START"))
												if (!strncmp(&dtmp[linestart+strpos],"START",5))	//_SIM7600E"))	//START"))
													{
													if (mqtt_step == 0)
														{
#ifdef QINT_DEBUG3
														printf("*** Got START!\n");
#endif
														CMQTT_rx_flag = CMQTTRX_CMD;
														mqtt_step = 1;
														}
													else 
														mqtt_step = 1;
													}
//$$												else if (!strncmp(&strp[strpos],"TOPIC",5))
												else if (!strncmp(&dtmp[linestart+strpos],"TOPIC",5))
													{
													if (mqtt_step == 1)
														{
#ifdef QINT_DEBUG3
														printf("*** Got TOPIC!\n");
#endif
														CMQTT_rx_flag = CMQTTRX_DATA;
														mqtt_step = 2;
														}
													else 
														mqtt_step = 0;
													}
//$$												else if (!strncmp(&strp[strpos],"PAYLOAD",7))
												else if (!strncmp(&dtmp[linestart+strpos],"PAYLOAD",7))
													{
													if (mqtt_step == 2)
														{
#ifdef QINT_DEBUG3
														printf("*** Got PAYLOAD!\n");
#endif
														CMQTT_rx_flag = CMQTTRX_DATA;
														mqtt_step = 3;
														}
													else 
														mqtt_step = 0;
													}
//$$												else if (!strncmp(&strp[strpos],"END",3))
												else if (!strncmp(&dtmp[linestart+strpos],"END",3))
													{
													if (mqtt_step == 3)
														{
#ifdef QINT_DEBUG3
														printf("*** Got END!\n");
#endif
														CMQTT_rx_flag = CMQTTRX_CMD;	// need to send this "END"  msg through as CMQTT data...
														endflag = 1;
														mqtt_step = 0;
														}
													else 
														mqtt_step = 0;
													}
												else 
													{
													CMQTT_rx_flag = SIMCOM_DATA;	// cmd data
													mqtt_step = 0;
													}
												}	// endof "if (pos != NULL)..."
											}
										else
											{
											CMQTT_rx_flag = SIMCOM_DATA;	// cmd data
											mqtt_step = 0;
											}
										}
									else		// flag is = 2
										{
											CMQTT_rx_flag = CMQTTRX_CMD;		// get following line of text
										}
#ifdef QINT_DEBUG2
									printf("%d[%d,%d,%d] %s\n",uart_num,four_g_msg_rx_count,buffered_size,CMQTT_rx_flag,strp);
#endif
							
							
									}
								else if (c == '>')
									{
#ifdef NEW_SERIAL_DBG1
									printf("New Serial - found >\n");
#endif

// found RIGHT_ARROW character...
									found_flag = FOUND_RIGHT_ARROW;

									four_g_response_flag = RESP_R_ARROW;			// defined in MAC_Rpt_four_g.h....

									simcom_response_flag = four_g_response_flag;
									mqtt_response_flag = four_g_response_flag;
									udp_response_flag = four_g_response_flag;

									linelen = inx + 1 - linestart;
									
//									dleft_size = buffered_size - (inx + 1);
									dleft_size = dprev_size + rx_bytes - (inx + 1);
									dleft_pos = inx+1;						

									if (debug_do(DBG_MQTT))
										{
										printf("PD Time: %d\n",four_g_timeout_timer);
										printf("&&& Found >! [%d]\n",four_g_response_flag);
										}
									}
								else
									{
// no termination character found...
//							if (CMQTT_rx_flag == CMQTTRX_DATA)		// we are receiving a CMQTT data line (no +CMQTTRX header...)
//								found_flag = FOUND_EOL;
									}
							
							
// now send data to fg
								if (found_flag)		// == FOUND_EOL)
//					if ((found_flag) || (CMQTT_rx_flag == CMQTTRX_DATA))// == FOUND_EOL)
									{
									unsigned char i,j;
									char c;
#ifdef NEW_SERIAL_DBG1
									printf("New Serial - send to fg [%d]  - ",found_flag);
#endif

							
									i = 0;
									j = pdTRUE;
									c = 0xFF;

									if (CMQTT_rx_flag == SIMCOM_DATA)		// SIMCOM command responses
										{
#ifdef NEW_SERIAL_DBG3
										printf("SIMCOM: [%d %d] ",linestart,linelen);
#endif
										pathchar = 'D';
	//									while ((i<buffered_size) && (c != 0x0A) && (j != pdFALSE))
										while ((i<linelen) && (c != 0x0A) && (j != pdFALSE))
											{
											c = dtmp[linestart+i];
											j = xQueueSend(mqtt_cmd_queue, &c, 10 / portTICK_PERIOD_MS);
											i++;
											lastchar = c;
#ifdef NEW_SERIAL_DBG3
											if ((c>0x1F) && (c<0x80))
												printf("%c",c);
											else
												printf(".");
#endif
											}								
#ifdef SET_PAYLOAD_TIME_TEST
										xtime = esp_timer_get_time();
										printf("[%lld.%06lldsec]",xtime/1000000,xtime%1000000);
#endif
#ifdef NEW_SERIAL_DBG3
										printf("\n");
#endif
										}
									else				// SIMCOM unsolicited receive data
										{
#ifdef NEW_SERIAL_DBG3
										printf("UNSOLICITED: [%d %d] ",linestart,linelen);
#endif
										pathchar = 'C';
//										printf("## CM_RX\n");
										
	//									while ((i<buffered_size) && (c != 0x0A) && (j != pdFALSE))
									if (mqtt_transport_mode==MQTT_MODE_4G)	// only enqueue the data if SIMOCOM 4G is current transport mode...
										{										
										while ((i<linelen) && (c != 0x0A) && (j != pdFALSE))
											{
											c = dtmp[linestart+i];
											j = xQueueSend(mqtt_data_queue, &c, 10 / portTICK_PERIOD_MS);
											i++;
											lastchar = c;
#ifdef NEW_SERIAL_DBG3
											if ((c>0x1F) && (c<0x80))
												printf("%c",c);
											else
												printf(".");
#endif
											}	
#ifdef SET_PAYLOAD_TIME_TEST
										xtime = esp_timer_get_time();
										printf("[%lld.%06lldsec]",xtime/1000000,xtime%1000000);
#endif

#ifdef NEW_SERIAL_DBG3
										printf("\n");
#endif
										}
										
										if (endflag)
											{
//								printf("endflag");
										
//											CMQTT_rx_msg = 1;	// changed 25/07/23
											if (mqtt_transport_mode==MQTT_MODE_4G)	// only enqueue the data if SIMOCOM 4G is current transport mode...
												CMQTT_rx_msg++;
											
											four_g_msg_rx_count = 0;
// got MQTT_CMD and MQTT_DATA - now expect next input is SIMCOM_DATA...
											CMQTT_rx_flag = SIMCOM_DATA;	// cmd data
											endflag = 0;		// ??
											}
										}

									found_flag = 0;
										
									linestart = inx+1;		// change at end, ready for next loop
									}


								inx++;
								}	// end of "while (inx < (buffered_size + dleft_size))..." loop
						


// if no end of line found, must be residual data...					
//					if (!found_flag)
							if (dleft_size)
								{
#ifdef NEW_SERIAL_DBG1
								printf("New Serial - data left %d\n",dleft_size);
#endif
							
// dleft_size, dleft_pos set above...
								dleft = dtmp;
								}
							else
								{
// free the memory block
#ifdef NEW_SERIAL_DBG1
								printf("New Serial - free dtmp\n");
#endif
							
							
								free(dtmp);
								dtmp = NULL;
								}						
							}	
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
/*
					}
				else if (pattern_detect_char == '>')
					{
// get previous data and following ">" into the SIMCOM response output queue:					
					
					four_g_response_flag = RESP_R_ARROW;			// defined in MAC_Rpt_four_g.h....
					if (debug_do(DBG_MQTT))
						{
						printf("PD Time: %d\n",four_g_timeout_timer);
						printf("&&& Found >! [%d]\n",four_g_response_flag);
						}

					uart_get_buffered_data_len((uart_port_t)uart_num, &buffered_size);
					dtmp = (char*) malloc(buffered_size + 1);	

					if (dtmp != NULL)
						{
						uart_read_bytes((uart_port_t)uart_num, (uint8_t*)dtmp, buffered_size, 10 / portTICK_PERIOD_MS);
					
						i = 0;
						j = pdTRUE;
						c = 0xFF;
						while ((c != '>') && (j != pdFALSE))
							{
							c = dtmp[i];
							j = xQueueSend(mqtt_cmd_queue, &c, 10 / portTICK_PERIOD_MS);
							i++;
							}								
					
						free(dtmp);
						dtmp = NULL;
						}

					}						
*/
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
	
#endif
}
