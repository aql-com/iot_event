//////////////////////////////////////////////
//
// MAC_Rpt_LoRa.cpp
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

// LoRa point-to-point send-receive
// LORA_MASTER initiates coms by sending a message; 
// then swicthes to receive mode for a timeout period
// LORA_MASTER returns to Tx mode either after a receive or a timeout
//
// LORA_SLAVE sits in receive mode; when a msg is received, can switch to Tx mode and send a response.
// LORA_SLAVE then switches back to Rx mode
// packet structure:
// SOPkt   addr	  data.....    EOPkt
// SOPkt = 0xE5
// addr = 0x01-> to slave    0x02->master, 
// EOPkt = 0x5E

//#include "driver/spi_common.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "memory.h"

#include "MAC_Rpt.h"
#include "MAC_Rpt_Rtns.h"
#include "MAC_Rpt_SX1276.h"
#include "MAC_Rpt_LoRa.h"


// NOTE array sizes must both use the [LORAWAN_CMD_LIST_LENGTH][LORAWAN_CMD_LIST_ENTRY_LENGTH] 
// so that they match the parameters of the listmatch() routine in MAC_rpt_rtns.cpp...

char lorawan_cmd_whitelist[LORAWAN_CMD_LIST_LENGTH][LORAWAN_CMD_LIST_ENTRY_LENGTH] = 
{
//234567890
	0x1A,0xAA,0xAA,0xAA,0xAA,0xAA
//	0x1B,0xBB,0xBB,0xBB,0xBB,0xBB,
//	0x8D,0x94,0x94,0x36,0x46,0xCA,		// DLT BT puck
//	0x1A,0xF0,0x25,0xA4,0x25,0xE0		// AM  BT puck

};

char lorawan_cmd_blacklist[LORAWAN_CMD_LIST_LENGTH][LORAWAN_CMD_LIST_ENTRY_LENGTH] = 
{
//234567890
//	0x2E,0xEE,0xEE,0xEE,0xEE,0xEE, 
//	0x2F,0xFF,0xFF,0xFF,0xFF,0xFF
};

const char lorawan_wl_init[LORAWAN_CMD_LIST_INIT_LENGTH][LORAWAN_CMD_LIST_ENTRY_LENGTH] = 
{
//234567890
//	0x1A,0xAA,0xAA,0xAA,0xAA,0xAA,
//	0x1B,0xBB,0xBB,0xBB,0xBB,0xBB,
	0x8D,0x94,0x94,0x36,		// DLT BT puck
	0x1A,0xF0,0x25,0xA4,		// AM  BT puck
	0x1A,0xAA,0xAA,0xAA,
	
};

const char lorawan_bl_init[LORAWAN_CMD_LIST_INIT_LENGTH][LORAWAN_CMD_LIST_ENTRY_LENGTH] = 
{
//234567890
	0x2E,0xEE,0xEE,0xEE
//	0x2F,0xFF,0xFF,0xFF,0xFF,0xFF
};

const char lorawan_wl_init_len = 3;
const char lorawan_bl_init_len = 1;




unsigned char LoRa_packet_start(void)
{
// resets SX1276 buffer pointers; loads packet header in to SX1276 payload register
unsigned char ret = 0;
	
return ret;
}

unsigned char LoRa_packet_write(unsigned char c)
{
// writes a byte to SX1276 payload register
unsigned char ret = 0;
	
return ret;
}

unsigned char LoRa_packet_printf(const char *format, ...)
{
// uses LoRaPacket_write() to send a printf-style string to the SX1276 payload register
unsigned char ret = 0;
	
return ret;
}

unsigned char LoRa_packet_end(void)
{
// loads packet tail into SXC1276 payload register; sets Tx mode; sends packet; sets rx mode
unsigned char ret = 0;
	
return ret;
}

