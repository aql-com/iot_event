//////////////////////////////////////////////
//
// MAC_Rpt_four_g_http_defs.h
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

enum M4G_STATES
{
M4G_POWER_OFF,
M4G_POWER_EN,
M4G_POWER_EN_WAIT,
M4G_RESET,
M4G_RESET_WAIT,
M4G_POWER_ON_KEY,
M4G_POWER_ON_KEY_WAIT,
M4G_POWER_ON_RESP_WAIT,
M4G_NO_COMM_IDLE,
M4G_GET_SIM_IMEI,
M4G_GET_SIM_NUM,
M4G_CLOSEALL,
M4G_SET_PDP,
M4G_SET_APN,
M4G_FORCE_CONN,
M4G_CHECK_CONN,
M4G_CONN_AUTO,
M4G_CHECK_SIG_QUALITY,
M4G_GET_NET_INFO,
M4G_CHECK_ATTACH,
M4G_NET_ACT,
M4G_START_WIRELESS,
M4G_NET_OPEN,
M4G_GET_IP_ADDR,
M4G_TCP_OPEN,
M4G_TCP_SEND,
M4G_TCP_RCV,
M4G_TCP_IDLE,
M4G_POWER_DOWN_KEY,
M4G_POWER_DOWN_KEY_WAIT,
M4G_POWER_DOWN_RESP_WAIT,
M4G_POWER_DISABLE,
M4G_END,
M4G_NUM_STATES
};

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


#define FOUR_G_POWER_EN_WAIT		   	   2	// 200msec
#define FOUR_G_RESET_WAIT				   5 	// 500msec
#define FOUR_G_POWER_ON_WAIT			   5 	// 500msec
#define FOUR_G_POWER_ON_RESP_WAIT		 160 	// 16 sec
#define FOUR_G_POWER_DOWN_KEY_WAIT		  25	// 2.5 sec
#define FOUR_G_POWER_DOWN_RESP_WAIT		 260	// 26 sec
//#define 
//#define 