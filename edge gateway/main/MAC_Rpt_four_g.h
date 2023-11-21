//////////////////////////////////////////////
//
// MAC_Rpt_four_g.h
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

#ifndef MAC_RPT_FOUR_G_H
#define MAC_RPT_FOUR_G_H

#define RESP_NONE		0
#define RESP_WAIT		1
#define RESP_OK			2
#define RESP_EOL		3
#define RESP_R_ARROW	4
#define RESP_PCMQTT		5
#define RESP_ERROR		6

const char four_g_resps[7][10] = 
{
"NONE",
"WAIT",
"OK",
"EOL",
"R_ARROW",
"PCMQTT",
"ERROR"
};

unsigned int four_g_pack_data(char *four_g_data, char *outstr);
int four_g_send_data(uint8_t *four_g_data, unsigned int four_g_data_len);
int four_g_rcv_data(uint8_t *four_g_data, unsigned int *four_g_data_len, unsigned char debugflag, unsigned int timeout);

unsigned char four_g_power_en(void);
unsigned char four_g_power_disable(void);

unsigned char four_g_hold_in_reset(void);
unsigned char four_g_release_from_reset(void);
unsigned char four_g_reset_state(void);

unsigned char four_g_power_key_on(void);
unsigned char four_g_power_key_off(void);
unsigned char four_g_power_state(void);

unsigned char four_g_check(unsigned char retries);

void four_g_state_machine(unsigned char *four_g_state,unsigned char *prev_four_g_state);
char* findstr(unsigned char *str1, char *str2);
void print_4g_state(unsigned char chan);

unsigned char get_AT_value(char *cmdstr, char *okstr, char *resultstr, char *eolstr, unsigned char line, unsigned char offset, char endchar, char *outstr, unsigned int outlen);

unsigned char get_str(char *instr, char *str, char *outstr, unsigned char dbg_flag);

#endif
