//////////////////////////////////////////////
//
// MAC_Rpt_rtns.h
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

#ifndef MAC_RPT_RTNS_H
#define MAC_RPT_RTNS_H

#include "driver/i2c.h"
#include "nvs_flash.h"			// for nvs_handle def

#include "MAC_Rpt.h"

unsigned char set_output(unsigned char device, unsigned char state, unsigned char inverted);
//unsigned char get_input(unsigned char device, unsigned char inverted);
//unsigned char get_output(unsigned char device);

void set_led(unsigned char led, unsigned char mode);

unsigned char cli(unsigned char* clistr);
unsigned char skip_whitespace(unsigned char* string, unsigned char* p);
unsigned char find_whitespace(unsigned char* string, unsigned char* p);
unsigned int get_param(unsigned char* string, unsigned char* p, char* prmstr, unsigned char uc_flag);

void status(unsigned char chan, const char* devicestr, unsigned char device);

void init_tm_data(unsigned char init_flag);

void show_options(void);

//int dbgprintf(FILE* dbgfile, const char * restrict format, ...);
#ifdef __cplusplus
int dbgprintf(unsigned char dbgfile, const char * format, ...);
#else
int dbgprintf(unsigned char dbgfile, const char *restrict format, ...);
#endif
unsigned char debug_do(unsigned int mask);
void toupperstr(char* str);

void mac_to_str(uint8_t* mac_addr, char separator, char* mac_str);
void str_to_mac(uint8_t* mac_str, char separator, char* mac_addr);
void show_inf(unsigned char simcom_info_flag);
void show_status(unsigned char chan);
signed int get_uart_rx_msg(unsigned char uart, unsigned char* data, unsigned int datalen, unsigned char timeout);
void show_time(unsigned char chan, unsigned char msec_flag);
void show_events(void);
void get_event_line(unsigned char evt_num, char* str, EVENT_INFO event_list[NUM_EVENTS_MAX]);
void get_event_line_html(unsigned char evt_num, char* str, EVENT_INFO event_list[NUM_EVENTS_MAX]);

unsigned char get_lora_radio_temperature(void);

unsigned char debug_hex_msg(uint8_t *msg_data, unsigned int msg_length, char *ID_str);

esp_err_t i2c_master_init(void);
esp_err_t i2c_slave_init(void);
esp_err_t check_i2c_device(unsigned char dev, unsigned char reg);
esp_err_t check_i2c_bus(unsigned char reg);
esp_err_t check_i2c_devices_present(void);

esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, unsigned char dev, unsigned char reg, uint8_t *data_wr, size_t size);
esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, unsigned char dev, unsigned char reg, uint8_t *data_rd, size_t size);


void print_banner_msg(char *str);
void print_chars(char c, unsigned char num, unsigned char nl_flag);

char set_uart_pattern_detect(char pattern_chr);
char set_meter_uart_pattern_detect(char pattern_chr);

unsigned char listmatch2(unsigned char listlen, unsigned char entrylen, void *list, char* matchitem, unsigned char dbg_level);

//unsigned char listmatch(unsigned char listlen, unsigned char entrylen, char list[NMEA_CMD_LIST_LENGTH][NMEA_CMD_LIST_ENTRY_LENGTH], char* matchitem, unsigned char dbg_level);
//unsigned char listmatch(unsigned char listlen, unsigned char entrylen, char list[listlen][entrylen], char* matchitem, unsigned char dbg_level);
unsigned char listmatch(unsigned int listlen, unsigned char entrylen, char *list, char *matchitem, unsigned char binary, unsigned char dbg_level);
unsigned char get_pipe(unsigned int *p, char *msgstr);
unsigned char get_listfield(unsigned int *p, char *msgstr, char *listfield);

unsigned char print_hex(unsigned char *dataptr, unsigned int len, unsigned char addrnotindex, unsigned char txt);

unsigned char nvs_gw_var_init(void);
unsigned char set_gw_vars_from_nvs(void);
unsigned char nvs_bl_var_init(char* area, char* num_str, char* item_str, char *init_array, unsigned char init_len, unsigned int entrylen, unsigned char ascii_flag);
unsigned char set_bl_vars_from_nvs(char* area, char* num_str, char* item_str, char* table, unsigned int* num_entries, unsigned int entrylen, unsigned char ascii_flag);
unsigned char nvs_set_addr(nvs_handle handle, char* str, unsigned char inx,unsigned char* addr, unsigned char addr_len);
unsigned char nvs_srvr_addr_port_var_init(void);
unsigned char set_srvr_addr_port_vars_from_nvs(void);
void show_server_addrs(void);
unsigned char nvs_lora_reg_var_init(void);
unsigned char set_lora_reg_vars_from_nvs(void);
/*
unsigned char nvs_wifi_var_init(void);
unsigned char set_wifi_vars_from_nvs(void);
*/
unsigned char nvs_wifi_sta_var_init(void);
unsigned char set_wifi_sta_vars_from_nvs(void);

void print_spaces(unsigned char num);

unsigned char nvs_evt_var_init(void);
unsigned char set_evt_vars_from_nvs(void);

unsigned char nvs_wr_u8(char* area, char *id_str, unsigned char inx, unsigned char val);

unsigned char timer_active(unsigned char day, unsigned char hrs, unsigned char mins,unsigned char day_1, unsigned char hrs_1, unsigned char mins_1,unsigned char day_2, unsigned char hrs_2, unsigned char mins_2);
unsigned char get_url_port(char* urlport_str, char*url_str, unsigned int* port);


#if PCB_VER == VER_1_0_D
//unsigned char shift_out(unsigned char sd_data_out);
#endif
#if PCB_VER == VER_LIFT_1_0_A
//unsigned char shift_out(unsigned int sd_data_out);
#endif
#if SHIFT_REG_LENGTH == 16
unsigned char shift_out(unsigned int sd_data_out);
#else
unsigned char shift_out(unsigned char sd_data_out);
#endif

unsigned char asc2hex(unsigned char* str);

void start_event(unsigned char event_num, unsigned char pwm);
void stop_event(unsigned char event_num);
void set_event(unsigned char evt_num, unsigned char evt_enable_flag,unsigned char evt_mode_flag,unsigned char evt_day,unsigned char evt_t1_hrs,unsigned char evt_t1_mins,unsigned char evt_t2_hrs,unsigned char evt_t2_mins,unsigned char evt_pwm, unsigned char evt_running);

void gpio_set(gpio_num_t gpio_num,char * namestr, gpio_mode_t mode, unsigned char val, unsigned char dbg);
void gpio_init(unsigned char dbg);

void* malloc_chk(size_t size, unsigned int* total);
void* calloc_chk(size_t num, size_t size, unsigned int* total);


#endif			// end of "#ifndef MAC_RPT_RTNS_H..."

