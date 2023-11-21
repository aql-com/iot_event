//////////////////////////////////////////////
//
// MAC_Rpt_LoRa.h
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

#ifndef LORA_H
#define LORA_H


//#include "driver/spi_common.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "memory.h"

#include "MAC_Rpt.h"		// for LORAWAN_CMD_LIST_LENGTH, LORAWAN_CMD_LIST_ENTRY_LENGTH] defs...
#include "MAC_Rpt_Rtns.h"
#include "MAC_Rpt_SX1276.h"

extern unsigned char enable_lorawan_whitelist_flag;		// LORAWAN whitelist enable flag
extern unsigned char enable_lorawan_blacklist_flag;		// LORAWAN blacklist enable flag

extern unsigned int num_lora_wl_entries;
extern  unsigned int num_lora_bl_entries;

extern char lorawan_cmd_whitelist[LORAWAN_CMD_LIST_LENGTH][LORAWAN_CMD_LIST_ENTRY_LENGTH];
extern char lorawan_cmd_blacklist[LORAWAN_CMD_LIST_LENGTH][LORAWAN_CMD_LIST_ENTRY_LENGTH];

extern const char lorawan_wl_init[LORAWAN_CMD_LIST_INIT_LENGTH][LORAWAN_CMD_LIST_ENTRY_LENGTH];
extern const char lorawan_bl_init[LORAWAN_CMD_LIST_INIT_LENGTH][LORAWAN_CMD_LIST_ENTRY_LENGTH];

extern const char lorawan_wl_init_len;
extern const char lorawan_bl_init_len;



unsigned char LoRa_packet_start(void);
unsigned char LoRa_packet_write(unsigned char c);
unsigned char LoRa_packet_printf(const char *format, ...);
unsigned char LoRa_packet_end(void);

#endif
