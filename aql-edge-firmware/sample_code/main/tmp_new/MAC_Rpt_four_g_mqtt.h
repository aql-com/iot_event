//////////////////////////////////////////////
//
// MAC_Rpt_four_g_mqtt.h
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

// new code for separate SIMCOM, MQTT and UDP modes

#ifndef FOUR_G_MQTT_H
#define FOUR_G_MQTT_H

#include "MAC_Rpt.h"

void four_g_mqtt_state_machine(unsigned char *four_g_state,unsigned char *prev_four_g_state);
unsigned char check_topic_str(void);

enum M4G_STATES
{
M4G_NO_COMM_IDLE,			// (system WAIT) <= goes to M4G_CHECKS_ERROR_END -> M4G_GET_MODULETYPE:	error_attempts = 0;
// comms init
M4G_COMM_INIT,
M4G_CLOSEALL,
M4G_CHECK_SIG_QUALITY,
M4G_CHECK_4G_NETWORK,
M4G_CHECK_GPRS_NETWORK,
M4G_CHECK_OPS,
M4G_MQTT_CHECK_RELEASE,
M4G_MQTT_CHECK_STOP,
M4G_SET_PDP,
M4G_ACT_PDP,				// THIS STATE NOT USED! error_attempts = 0;

M4G_SET_SSL_CONFIG,
M4G_SET_SSL_AUTHMODE,
M4G_LOAD_SSL_SERVER_CERTIFICATE,
M4G_LOAD_SSL_CLIENT_CERTIFICATE,
M4G_LOAD_SSL_CLIENT_KEY,
M4G_SHOW_SSL_CERTIFICATES,
M4G_SET_SSL_SERVER_CERTIFICATE,
M4G_SET_SSL_CLIENT_CERTIFICATE,
M4G_SET_SSL_CLIENT_KEY,		// WAS error_attempts = 0;

M4G_COMM_START,				// (not a system WAIT) <= goes to M4G_COMMS_INIT_ERROR_END -> M4G_COMM_INIT:	error_attempts = 0;
// connect
M4G_MQTT_ACQUIRE_CLIENT,
M4G_MQTT_SET_WILLTOPIC,
M4G_MQTT_SET_WILLMSG,
M4G_MQTT_SET_UTF8_MODE,		// WAS error_attempts = 0;
M4G_MQTT_CONNECT,			// (not a system WAIT) <= goes to M4G_CONNECT_ERROR_END -> M4G_MQTT_ACQUIRE_CLIENT:	error_attempts = 0;
M4G_MQTT_CONNECT_CHECK,
// send \ receive
M4G_MQTT_SET_SUBSCRIBE_TOPIC,
M4G_MQTT_SUBSCRIBE_TOPIC,
M4G_MQTT_SUBSCRIBE_MSG,
M4G_MQTT_SET_TOPIC,
M4G_MQTT_SET_PAYLOAD,
M4G_MQTT_PUBLISH_MSG,		// WAS error_attempts = 0;

M4G_CONNECT_IDLE,			// (system WAIT) <= goes to M4G_SEND_RCV_ERROR_END -> M4G_MQTT_SET_TOPIC:	error_attempts = 0;
// disconnect
M4G_MQTT_UNSUBSCRIBE_TOPIC,	// > M4G_CONNECT_IDLE goes to M4G_DISC_PWRDOWN_ERROR_END -> M4G_POWER_DOWN_KEY
M4G_MQTT_SERVER_DISCONNECT,
M4G_MQTT_CLIENT_RELEASE,
M4G_MQTT_STOP,

M4G_COMM_IDLE,				//  (system WAIT)

M4G_MQTT_IDLE_CHECK_SIG_QUALITY,

M4G_POWER_UP_ERROR_END,
M4G_CHECKS_ERROR_END,
M4G_COMMS_INIT_ERROR_END,
M4G_CONNECT_ERROR_END,
M4G_SEND_RCV_ERROR_END,
M4G_DISC_PWRDOWN_ERROR_END,
M4G_END,
M4G_NO_ERROR,
M4G_NUM_STATES
};

const char four_g_state_str[M4G_NUM_STATES][32] = 
{
//        1         2         3
//234567890123456789012345678901

"NO_COMM_IDLE",

"COMM_INIT",

"CLOSEALL",
"CHECK_SIG_QUALITY",
"CHECK_4G_NETWORK",
"CHECK_GPRS_NETWORK",
"CHECK_OPS",
"MQTT_CHECK_RELEASE",
"MQTT_CHECK_STOP",
"SET_PDP",
"ACT_PDP",

"SET_SSL_CONFIG",
"SET_SSL_AUTHMODE",
"LOAD_SSL_SERVER_CERTIFICATE",
"LOAD_SSL_CLIENT_CERTIFICATE",
"LOAD_SSL_CLIENT_KEY",
"SHOW_SSL_CERTIFICATES",

"SET_SSL_SERVER_CERTIFICATE",
"SET_SSL_CLIENT_CERTIFICATE",
"SET_SSL_CLIENT_KEY",

"MQTT_COMM_START",

"MQTT_ACQUIRE_CLIENT",
"MQTT_SET_WILLTOPIC",
"MQTT_SET_WILLMSG",
"MQTT_SET_UTF8_MODE",
"MQTT_CONNECT",
"MQTT_CONNECT_CHECK",
"MQTT_SET_SUBSCRIBE_TOPIC",
"MQTT_SUBSCRIBE_TOPIC",
"MQTT_SUBSCRIBE_MSG",
"MQTT_SET_TOPIC",
"MQTT_SET_PAYLOAD",
"MQTT_PUBLISH_MSG",

"MQTT_CONNECT_IDLE",

"MQTT_UNSUBSCRIBE_TOPIC",
"MQTT_SERVER_DISCONNECT",
"MQTT_CLIENT_RELEASE",
"MQTT_STOP",

"MQTT_COMM_IDLE",

"MQTT_IDLE_SIG_QUALITY",

"POWER_UP_ERROR",
"CHECKS_ERROR",
"COMMS_INIT_ERROR",
"CONNECT_ERROR",
"SEND_RCV_ERROR",
"DISC_PWRDOWN_ERROR",
"END",
"NO_ERROR"
};

/*
const char nmea_cmd_list[NMEA_CMD_LIST_LENGTH][NMEA_CMD_LIST_ENTRY_LENGTH] = 
{
//234567890
"$PGPPADV",
"$GPGBS",
"$GPGGA",
"$GPGSA"	
};
*/
#define ADDR_AND_PORT_LEN	100





#define FOUR_G_POWER_EN_WAIT		   	   2	// 200msec
#define FOUR_G_RESET_WAIT				   5 	// 500msec
#define FOUR_G_POWER_ON_WAIT			   5 	// 500msec
#define FOUR_G_POWER_ON_RESP_WAIT		 160 	// 16 sec
#define FOUR_G_POWER_DOWN_KEY_WAIT		  25	// 2.5 sec
#define FOUR_G_POWER_DOWN_RESP_WAIT		  10	// 1 sec 	// 260	// 26 sec
#define COMM_DELAY						  20	//10	// 1 sec
#define COMM_TIMEOUT					  20	//10	// 0.5 sec
#define COMM_LONG_TIMEOUT				  40	//20	// 1 sec
#define COMM_VLONG_TIMEOUT				  60	//30	// 1 sec
#define COMM_CERT_TIMEOUT				 100	//40	// 3 sec
#define COMM_PUBLISH_TIMEOUT			 200	//30	// 1 sec
#define COMM_CONN_TIMEOUT				 200	// 3 sec
#define COMM_TIMEOUT_DEFEAT			  0xFFFF	// long as possible...
#define COMM_PHASE_RETRIES				   2	// 1 attempt + 2 retries
#define ERROR_RETRIES				   	   2	// 1 attempt + 2 retries
#define MAX_ERROR_RETRIES		   	      10	// 1 attempt + 9 retries

#define MQTT_SENSOR_TIME				  30	// 180sec = 3 min

#define MQTT_PAYLOAD_SIZE				 8192	// max 10240

#endif