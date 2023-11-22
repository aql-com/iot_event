//////////////////////////////////////////////
//
// MAC_Rpt_radio.h
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

#ifndef MAC_RPT_LORA_RADIO_H
#define MAC_RPT_LORA_RADIO_H


int lora_radio_send_data(uint8_t *lora_radio_data, unsigned int lora_radio_data_len);

unsigned char lora_radio_hold_in_reset(void);
unsigned char lora_radio_release_from_reset(void);
unsigned char lora_radio_reset_state(void);

unsigned char lora_radio_check(unsigned char retries);

#endif
