//////////////////////////////////////////////
//
// MAC_Rpt_four_g_state_mc_defs_7000.h
//
// aql Ltd
//
// Auth: DLT
//
// for SIMCOM 7000 - under development - not tested yet!
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


#define FOUR_G_POWER_EN_WAIT		   	   2	// 200msec
#define FOUR_G_RESET_WAIT				   5 	// 500msec
#define FOUR_G_POWER_ON_WAIT			   5 	// 500msec
#define FOUR_G_POWER_ON_RESP_WAIT		 160 	// 16 sec
#define FOUR_G_POWER_DOWN_KEY_WAIT		  25	// 2.5 sec
#define FOUR_G_POWER_DOWN_RESP_WAIT		 260	// 26 sec
//#define 
//#define 