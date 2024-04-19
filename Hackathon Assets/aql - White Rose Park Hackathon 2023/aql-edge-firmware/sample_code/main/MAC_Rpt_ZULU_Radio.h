//////////////////////////////////////////////
//
// MAC_Rpt_ZULU_radio.h
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

#ifndef MAC_RPT_ZULU_RADIO_H
#define MAC_RPT_ZULU_RADIO_H

unsigned char send_zulu_radio_msg(unsigned char *tx_data, unsigned char dev_num, unsigned char seq_num, unsigned char msg_type, unsigned char tx_data_len);


enum ZULU_radio_msgs { ZULU_HBEAT_MSG,ZULU_STATUS_MSG,ZULU_RUN_MSG,ZULU_GET_TIMER_MSG, ZULU_SET_TIMER_MSG };

const char zulu_radio_msg_names[5][15] = 
{
//2345678901235467890	
"HEARTBEAT",
"STATUS",
"RUN",
"GET_TIMER",
"SET_TIMER"
};

#define MSG_START_BYTE					 0
#define DEV_NUM_BYTE					 1
#define SEQ_NUM_BYTE					 2
#define MSG_TYPE_BYTE					 3
#define MSG_LEN_BYTE					 4
#define DATA_START_BYTE					 5

#define ZULU_RADIO_MASTER_ID			 0	// master device number is always 0


#define ZULU_RADIO_HEARTBEAT_TIME	    10	// in 100msec increments
#define ZULU_RADIO_STATUS_TIME		    50	// in 100msec increments
#define ZULU_RX_TIMEOUT				   100	// in 100msec increments

#define STX								0x02
#define ETX								0x03
#define CR								0x0D
#define LF								0x0A

#define ZULU_RADIO_FLAG_ERROR			1
#define ZULU_RADIO_CHKSUM_ERROR			2
#define ZULU_RADIO_SEQ_NUM_ERROR		4





#endif