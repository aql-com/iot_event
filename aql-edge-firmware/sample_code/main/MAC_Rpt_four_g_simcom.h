//////////////////////////////////////////////
//
// MAC_Rpt_SIMCOM_simcom.h
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

#ifndef MAC_RPT_SIMCOM_SIMCOM_H
#define MAC_RPT_SIMCOM_SIMCOM_H


void four_g_simcom_state_machine(unsigned char *SIMCOM_state,unsigned char *prev_SIMCOM_state);
unsigned char check_topic_str(void);
void print_simcom_state(unsigned char chan);
void simcom_busy_change(unsigned char *simcom_busy_flag, unsigned char *prev_simcom_busy_flag, char *str);

void simcom_net_check(char *str, unsigned char dbg_4g_flag);

enum SIMCOM_STATES
{
// power up
SIMCOM_POWER_OFF,				// error_attempts = 0;
SIMCOM_POWER_EN,
SIMCOM_POWER_EN_WAIT,
SIMCOM_RESET,
SIMCOM_RESET_WAIT,
SIMCOM_POWER_ON_KEY,
SIMCOM_POWER_ON_KEY_WAIT,
SIMCOM_POWER_ON_RESP_WAIT,		// (not a system WAIT) <= goes to SIMCOM_POWER_UP_ERROR_END -> SIMCOM_POWER_EN:	error_attempts = 0;
// module checks
SIMCOM_SET_FLOWCTRL,
SIMCOM_GET_MODULETYPE,
SIMCOM_GET_FWVERSION,
SIMCOM_GET_SIM_MAC,
SIMCOM_GET_SIM_IMEI,
SIMCOM_GET_SIM_NUM,
SIMCOM_GET_NWK_SETTINGS,
SIMCOM_GPS_CHECK,
SIMCOM_GPS_START,
SIMCOM_GPS_INFO,				// WAS error_attempts = 0;

SIMCOM_READY_IDLE,				// (system WAIT) <= goes to SIMCOM_CHECKS_ERROR_END -> SIMCOM_GET_MODULETYPE:	error_attempts = 0;

// power down
SIMCOM_POWER_DOWN_KEY,	
SIMCOM_POWER_DOWN_KEY_WAIT,
SIMCOM_POWER_DOWN_RESP_WAIT,
SIMCOM_POWER_DISABLE,			// error_attempts = 0;

SIMCOM_MQTT_IDLE_CHECK_SIG_QUALITY,

SIMCOM_POWER_UP_ERROR_END,
SIMCOM_CHECKS_ERROR_END,
SIMCOM_COMMS_INIT_ERROR_END,
SIMCOM_CONNECT_ERROR_END,
SIMCOM_SEND_RCV_ERROR_END,
SIMCOM_DISC_PWRDOWN_ERROR_END,
SIMCOM_END,
SIMCOM_NO_ERROR,
SIMCOM_NONE_FOUND,

SIMCOM_NUM_STATES
};


const char simcom_state_str[SIMCOM_NUM_STATES][32] = 
{
//        1         2         3
//234567890123456789012345678901
"POWER_OFF",
"POWER_EN",
"POWER_EN_WAIT",
"RESET",
"RESET_WAIT",
"POWER_ON_KEY",
"POWER_ON_KEY_WAIT",
"POWER_ON_KEY_RESP_WAIT",
"SET_FLOWCTRL",
"GET_MODULETYPE",
"GET_FIRMWARE_VERSION",
"GET_SIM_MAC",
"GET_SIM_IMEI",
"GET_SIM_NUM",
"GET NWK SETTINGS",
"GPS_CHECK",
"GPS_START",
"GPS_INFO",

"READY_IDLE",

"POWER_DOWN_KEY",
"POWER_DOWN_KEY_WAIT",
"POWER_DOWN_RESP_WAIT",
"POWER_DISABLE",

"MQTT_IDLE_SIG_QUALITY",

"POWER_UP_ERROR",
"CHECKS_ERROR",
"COMMS_INIT_ERROR",
"CONNECT_ERROR",
"SEND_RCV_ERROR",
"DISC_PWRDOWN_ERROR",
"END",
"NO_ERROR",
"NONE_FOUND"
};

const char simcom_busy_state_str[4][10] = 
{
"IDLE",
"MQTT",
"UDP",
"MQTT+UDP"
};

#define SIMCOM_BUSY_IDLE		0
#define SIMCOM_BUSY_MQTT		1
#define SIMCOM_BUSY_UDP			2
#define SIMCOM_BUSY_MQTT_UDP	3

#define SIMCOM_POWER_EN_WAIT_TIME	   	   2	// 200msec
#define SIMCOM_RESET_WAIT_TIME			   5 	// 500msec
#define SIMCOM_POWER_ON_WAIT_TIME		   5 	// 500msec
#define SIMCOM_POWER_ON_RESP_WAIT_TIME	 160 	// 16 sec
#define SIMCOM_POWER_DOWN_KEY_WAIT_TIME	  25	// 2.5 sec
#define SIMCOM_POWER_DOWN_RESP_WAIT_TIME  10	// 1 sec 	// 260	// 26 sec
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
#define SIMCOM_MAX_ERROR_RETRIES		   3	// 1 attempt + 9 retries
#define SIMCOM_MAX_POWERDOWN_RETRIES	   3	// 3 attempts at powerdown\restart before declaring "no SIMCOM module"

#endif
