//////////////////////////////////////////////
//
// MAC_Rpt_rtns.c
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#include "nvs_flash.h"
#include "nvs.h"
//#include "sdkconfig.h"


#include "MAC_Rpt.h"
#include "MAC_Rpt_rtns.h"
#include "MAC_Rpt_four_g.h"
//#include "MAC_Rpt_four_g_state_mc_defs.h"
//#include "MAC_Rpt_four_g_http_defs.h"
//#include "MAC_Rpt_four_g_http.h"

//#include "MAC_Rpt_four_g_mqtt_defs.h"
#include "MAC_Rpt_four_g_simcom.h"
#include "MAC_Rpt_four_g_mqtt.h"
#include "MAC_Rpt_four_g_udp.h"
#include "MAC_Rpt_wifi_mqtt.h"


#ifdef USE_BLUETOOTH
#include "esp_bt_device.h"
#include "Bluetooth.h"

#endif

#ifdef LORA_USED
#include "driver/spi_common.h"
#include "driver/spi_master.h"
//#include "MAC_Rpt_SX1276_defs.h"
#include "MAC_Rpt_SX1276.h"
#include "MAC_Rpt_LoRa.h"
#endif

#ifdef USE_WIFI_UDP
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "MAC_Rpt_wifi_udp.h"
#endif



#ifdef LORA_USED
extern spi_device_handle_t spi;
extern unsigned char SX1276_init_vals[];
#endif

#ifdef USE_LORA_LINK
//unsigned char lora_tx_data[80];
//unsigned char lora_tx_data_len;
extern unsigned char lora_tx_timer;	//, lora_tx_timer_flag;
#endif

extern unsigned char x100msec, secs, mins, hrs;
extern unsigned char days, month, year;
extern unsigned char day_of_week;

extern unsigned char u100msec, u1sec, u1min, u1hr;
extern unsigned int u1day;


extern unsigned long int dbgbits;
extern unsigned char dbgflag;
extern unsigned char status_array[];
extern unsigned char tank_status;

//#ifdef V1_PCB
//#if PCB_VER == VER_1_0_C
extern unsigned char devpin[];
//#endif

#if PCB_VER == VER_1_0_D
extern unsigned char sd_data_in, sd_data_out;
#endif


#if (PCB_VER == VER_BTS_1_0_C) || (PCB_VER == VER_BTS_1_0_D)
extern unsigned char sd_data_in, sd_data_out;
#endif

extern unsigned char led_states, prev_led_states;
extern unsigned char led_modes[4];

extern unsigned int num_dispense_outputs;
extern unsigned char tank_countdown;

extern wifi_config_t wifi_config_sta, wifi_config_ap;
extern unsigned char wifi_sta_auth,wifi_ap_auth;


extern unsigned int instantaneous_pump_motor_current;
extern unsigned int pump_motor_current[PUMP_MOTOR_SAMPLES];
extern unsigned long pump_motor_ave_current;

extern signed char lora_rssi, lora_pkt_rssi, lora_snr;

extern unsigned int lora_radio_msg_time;
extern unsigned int wifi_scan_time, wifi_scan_timer;

extern unsigned int adc_val_raw_sv;	
extern unsigned int supply_voltage;
extern uint32_t voltage;


extern unsigned char temperature;


#ifdef WIFI_SCAN
extern wifi_ap_record_t wifi_list_A[];	
extern wifi_ap_record_t wifi_list_B[];	

extern wifi_ap_record_t* wifi_list_current;
extern wifi_ap_record_t* wifi_list_previous;
extern wifi_ap_record_t* wifi_list_tmp;

extern unsigned int wifi_current_num, wifi_previous_num;
extern unsigned int wifi_dev_count;
#endif

extern unsigned char wifi_scan_enable_flag;

extern unsigned char ntptime_sync_flag;

extern unsigned int TIM_msg_time;
extern unsigned int uart_stay_awake_time;
extern unsigned int uart_stay_awake_count;

extern unsigned char tm_len;
extern unsigned char tm_data[];

extern uint8_t four_g_tx_data[FOUR_G_TX_BUFSIZE];
extern uint8_t four_g_rx_data[FOUR_G_RX_BUFSIZE];
extern unsigned int four_g_tx_length;
extern unsigned int four_g_rx_length;

extern unsigned char simcom_state, prev_simcom_state;
extern unsigned char mqtt_state, prev_mqtt_state;
extern unsigned char udp_state, prev_udp_state;
extern unsigned int four_g_state_mc_timer;

extern unsigned char simcom_response_flag, simcom_eol_response_flag;
extern unsigned char mqtt_response_flag, mqtt_eol_response_flag;
extern unsigned char udp_response_flag, udp_eol_response_flag;

extern unsigned char simcom_attempts, simcom_error_attempts,simcom_powerdown_attempts;
extern unsigned char mqtt_attempts, mqtt_error_attempts;
extern unsigned char udp_attempts, udp_error_attempts;
extern unsigned int four_g_timeout_timer,four_g_timeout_val;;

extern unsigned char ssl_enable_flag;


extern char modtype_str[];
extern char fwver_str[];
extern char imei_str[];
extern char simnum_str[];

//extern char four_g_state_str[M4G_NUM_STATES][32];

#ifdef USE_M4G_MQTT
extern QueueHandle_t mqtt_cmd_queue;
extern QueueHandle_t mqtt_data_queue;
extern unsigned char server_cmd_test_flag;
extern char server_cmd_test_str[];

extern unsigned char mqtt_login_state;

extern char server_addr_and_port [4][ADDR_AND_PORT_LEN];
extern unsigned char server_ssl_mode [4];

extern unsigned char server0_addr_ptr, server1_addr_ptr;
//extern unsigned char server0_ssl_mode, server1_ssl_mode;

extern unsigned char CMQTT_rx_flag;

#endif

extern unsigned char simcom_disable_flag;
extern unsigned char mqtt_disable_flag;
extern unsigned char wifi_disable_flag;
extern unsigned char udp_disable_flag;

extern unsigned char mqtt_transport_mode,mqtt_preferred_transport_mode;

extern unsigned char tank_sensors_present_flag;
extern unsigned char tank_sensor_0, tank_sensor_1, tank_sensor_percent_0, tank_sensor_percent_1;

extern unsigned char lora_msg_ready_count;

extern signed char cell_rssi;

extern unsigned long serial_bitrate;
extern int32_t auto_serial_bitrate;
extern unsigned char binary_data_flag, auto_binary_data_flag, binary_mode_flag;
extern char eolstr[10];
extern unsigned char eol_remove_flag;

extern unsigned char tx_serial_settings_flag, serial_autobaud_flag;

extern unsigned char enable_bluetooth_whitelist_flag;			// Bluetooth whitelist enable flag
extern unsigned char enable_bluetooth_blacklist_flag;			// Bluetooth blacklist enable flag
extern unsigned char enable_bluetooth_devname_whitelist_flag;	// Bluetooth device list enable flag
extern unsigned char bluetooth_whitelist_mode;
extern unsigned char bluetooth_blacklist_mode;
extern unsigned int num_bt_wl_entries;
extern unsigned int num_bt_bl_entries;

extern unsigned char tx_lorawan_whitelist_flag;
extern unsigned char tx_lorawan_blacklist_flag;
extern unsigned char enable_lorawan_whitelist_flag;				// LoRaWAN whitelist enable flag
extern unsigned char enable_lorawan_blacklist_flag;				// LoRaWAN blacklist enable flag
extern unsigned int num_lora_wl_entries;
extern unsigned int num_lora_bl_entries;

extern unsigned int num_bt_devname_wl_entries;

extern unsigned int gateway_sensor_time;
extern unsigned int gps_sensor_time;
extern unsigned int i2c_sensor_time;
extern unsigned int tank_sensor_time;
extern unsigned int bluetooth_sensor_time;
extern unsigned int bluetooth_MAC_addr_time;
extern unsigned int bluetooth_devname_time;
extern unsigned int bluetooth_classic_scan_time;
extern unsigned int lora_devaddr_time;

extern unsigned int gateway_sensor_timer;
extern unsigned int gps_sensor_timer;
extern unsigned int i2c_sensor_timer;
extern unsigned int tank_sensor_timer;
extern unsigned int bluetooth_sensor_timer;
extern unsigned int bluetooth_MAC_addr_timer;
extern unsigned int bluetooth_devname_timer;
extern unsigned int bluetooth_classic_scan_timer;
extern unsigned int lora_devaddr_timer;

extern unsigned int bluetooth_sensor_adv_time;
extern char bluetooth_sensor_adv_name[];

extern unsigned int i2c_temp,i2c_humid,i2c_light,i2c_audio,i2c_batt,i2c_co2,i2c_voc;

extern unsigned char queues_enabled_flag;

#ifdef USE_WIFI_UDP
extern bool wifi_udp_connected;	// = false;
extern bool wifi_udp_DNSfound;	// = false;
#endif

#ifdef USE_ZULU_RADIO
extern uint8_t zulu_radio_rx_data[ZULU_RADIO_RX_BUFSIZE];
extern unsigned char zulu_radio_run_flag,zulu_radio_msgs_flag;
#endif


extern char wifi_sta_SSID[WIFI_CREDENTIALS_MAX][32];
extern char wifi_sta_PWD[WIFI_CREDENTIALS_MAX][32];
extern unsigned char wifi_sta_credentials_index;


extern struct EVENT_INFO event_list[NUM_EVENTS_MAX];

extern unsigned int evt_timer[NUM_EVENTS_MAX];
extern unsigned int evt_repeat_timer[NUM_EVENTS_MAX];

extern unsigned char get_server_time_flag;
extern unsigned int get_server_time_timer;




extern unsigned char net_test_flag;

extern char pattern_detect_char, prev_pattern_detect_char;

#if PCB_VER == VER_1_0_C
char devname[NUM_DEVS][15] = 
{
//12345678901234567890	
 "LORA_RADIO_RTS",
 "LORA_RADIO_CTS",
 "FOUR_G_RTS",
 "FOUR_G_CTS",
 "FOUR_G_DTR",
 "FOUR_G_DCD",
 "LED_0",
 "LED_1",
 "LED_2",
 "LED_3",
 "LORA_RESET",
 "FOUR_G_RESET",
 "FOUR_G_POWER"
};
#endif

#if PCB_VER == VER_1_0_D
char devname[NUM_DEVS][15] = 
{
//12345678901234567890	
 "FOUR_G_RTS",
 "FOUR_G_CTS",
 "FOUR_G_DTR",
 "FOUR_G_DCD",
 
 "FOUR_G_RESET",
 "FOUR_G_PS_EN",
 "FOUR_G_POWER",
 "LORA_RESET",
 "LED_0",
 "LED_1",
 "LED_2",
 "LED_3"
};
#endif


#if (PCB_VER == VER_BTS_1_0_C) || (PCB_VER == VER_BTS_1_0_D)
char devname[NUM_DEVS][15] = 
{
//12345678901234567890	
 "FOUR_G_RTS",
 "FOUR_G_CTS",
 "FOUR_G_DTR",
 "FOUR_G_DCD",
 
 "FOUR_G_RESET",
 "FOUR_G_PS_EN",
 "FOUR_G_POWER",
 "LORA_RESET",
 "LED_0",
 "LED_1",
 "LED_2",
 "LED_3"
};
#endif

char mqtt_transport_names[3][5] = 
{
"NONE",
"4G",
"WIFI"
};

#if defined(USE_LORA) || defined(USE_LORA_LINK)
// 100* LoRa bandwith
unsigned int LoRa_bandwidth[10] = 
{
	  780,
	 1040,
 	 1560,
	 2080,
	 3125,
	 4170,
	 6250,
	12500,
	25000,
	50000
	
};
#endif

char wifi_auth[12][14] = 
{
//2345678901234567890	
"NONE",
"WEP",
"WPA_PSK",
"WPA2_PSK",
"WPA_WPA2_PSK",
"ENTERPRISE",
"WPA2_ENTPRISE",
"WPA3_PSK",
"WPA2_WPA3_PSK",
"WAPI",
"OWE",
"WPA3_ENT_192"	
};

// CLI definitions
enum cli_cmds 
{
CMD_HEL,		// HELP - list all commands
CMD_DBG,		// control debug system
CMD_STA,		// board status
CMD_INF,		// show build info
CMD_RTM,
CMD_RTX,
CMD_RAD,
CMD_TIM,		// show system time
CMD_WFL,
CMD_WFS,
CMD_WTM,
CMD_TMS,
CMD_SES,
CMD_OPT,		// show build options
CMD_M4G,		// send \ receive MQTT msg (not finished yet!)
CMD_SSL,		// MQTT SSL status \ enable
CMD_BWL,		// list contents of black and white lists and enable flags
CMD_SWE,		// serial \ NMEA whitelist status \ enable
CMD_SBE,		// serial \ NMEA blacklist status \ enable
CMD_BWE,		// BlueTooth whitelist status \ enable
CMD_BBE,		// BlueTooth blacklist status \ enable
CMD_BDE,		// BlueTooth device list status \ enable
CMD_LWE,		// LoRaWAN whitelist status \ enable
CMD_LBE,		// LoRaWAN blacklist status \ enable
CMD_LOR,		// show LoRaWAN status
CMD_MQT,		// status \ enable for MQTT
CMD_SRV,		// show server black\white list-related values
CMD_TMR,		// show server timer-related values
CMD_NVG,		// initialise server gateway-related settings
CMD_NVS,		// initialise NMEA\serial white \ black lists
CMD_NVB,		// initialise bluetooth white \ black lists
CMD_NVL,		// initialise LoRaWAN white \ black lists
CMD_NVX,		// initialise Bluetooth Sensor details
CMD_SCT,		// test server code by injecting a string into the receive side
CMD_BSN,		// show \ edit Bluetooth Sensor Local Device Name
CMD_BST,		// show \ edit Bluetooth Sensor Advertising time period
CMD_I2C,		// show \ edit I2C  data
CMD_I2B,		// check I2C bus for devices
CMD_TZR,		// test zulu radio link
CMD_ZRM,		// show zulu radio msgs
CMD_ZRC,		// zulu radio config
CMD_NVA,		// initialise server address settings
CMD_SSA,		// choose server address
CMD_RST,		// restart ESP module
CMD_PWM,		// test lift PWM on GPIO 18
CMD_RLA,		// test lift relay on GPIO 33
CMD_RL1,		// test lift relay on GPIO 33
CMD_RL2,		// test lift relay on GPIO 33
CMD_MRM,		// meter read messages on \ off
CMD_WFN,		// wifi sniff info
CMD_WST,		// show \ set wifi subtype
CMD_SEV,		// show event table
CMD_SET,		// server event test
CMD_GST,		// get server time
CMD_LFS,		// lift status 
CMD_LCT,		// lift UVC \ PWM control
CMD_GPI,		// show GPIO settings 
CMD_DMP,		// dump memory locations
CMD_MRD,		// meter read
CMD_MRS,		// meter read - set start point
CMD_UDP,		// status \ enable for UDP
CMD_DLT,		// set up debug
CMD_LRM,		// show \ set LoRa link Master mode
CMD_RAS,		// rally safety
CMD_RAM,		// Rally safety master \ slave
CMD_RID,		// Rally safety ID
CMD_RAL,		// Rally alert on \ off
CMD_RCN,		// Rally safety cancel
CMD_RTS,		// Rally test on \ off
CMD_RTP,		// Rally test ping
CMD_NET,		// 4G network test cmd
CMD_STM,		// set local time
CMD_SED,		// set event duration
CMD_SWS,		// show \ edit Wifi SSID
CMD_STR,		// show \ set preferred MQTT transport mode
CMD_UVC,		// show lift UVC parameters
CMD_UPT,		// show system up-time
CMD_LPW,		// show \ set LoRa output power
CMD_LRS,		// show LoRa average RSSI
CMD_LBW,		// show \ set LoRa bandwidth
CMD_LSF,		// show \ set LoRa spread factor
CMD_WFC,		// show current wifi config
CMD_WFA,		// show \ set wifi authentication
CMD_TST,		// debug test cmd
CMD_END
};

#define NUM_CMDS	CMD_END
#define CMDLEN		3

const char cmdlist[NUM_CMDS][4] = 
{
"HEL",
"DBG",
"STA",
"INF",
"RTM",
"RTX",
"RAD",
"TIM",
"WFL",
"WFS",
"WTM",
"TMS",
"SES",
"OPT",
"M4G",
"SSL",
"BWL",
"SWE",
"SBE",
"BWE",
"BBE",
"BDE",
"LWE",
"LBE",
"LOR",
"MQT",
"SRV",
"TMR",
"NVG",
"NVS",
"NVB",
"NVL",
"NVX",
"SCT",
"BSN",
"BST",
"I2C",
"I2B",
"TZR",
"ZRM",
"ZRC",
"NVA",
"SSA",
"RST",
"PWM",
"RLA",
"RL1",
"RL2",
"MRM",
"WFN",
"WST",
"SEV",
"SET",
"GST",
"LFS",
"LCT",
"GPI",
"DMP",
"MRD",
"MRS",
"UDP",
"DLT",
"LRM",
"RAS",
"RAM",
"RID",
"RAL",
"RCN",
"RTS",
"RTP",
"NET",
"STM",
"SED",
"SWS",
"STR",
"UVC",
"UPT",
"LPW",
"LRS",
"LBW",
"LSF",
"WFC",
"WFA",
"TST"
};






//#ifdef V1_PCB
#if PCB_VER == VER_1_0_C

unsigned char set_output(unsigned char device, unsigned char state, unsigned char inverted)
{
unsigned char error,i;

i = state;
if (inverted)
	i = 1 - i;

error = gpio_set_level((gpio_num_t)devpin[device],i);		// invert output if necessary...

status_array[device] = state;	// write logical state to status array, not actual value

if ((error) || (0))
	printf("SET %d %d %s %d\r\n",device,devpin[device],devname[device],state);

return error;
}
/*
unsigned char get_input(unsigned char device, unsigned char inverted)
{
unsigned char state;
	
state = gpio_get_level((gpio_num_t)devpin[device]);	

if (inverted)
	state = 1 - state;

status_array[device] = state;

return state;
	
}
*/
#endif

/*
unsigned char get_output(unsigned char device)
{
unsigned char state;

if (status_array[device])
	state = DEVICE_ON;
else
	state = DEVICE_OFF;
	
return state;
}
*/

//#ifdef V2_PCB
#if (PCB_VER == VER_1_0_D) || (PCB_VER == VER_BTS_1_0_C) || (PCB_VER == VER_BTS_V_1_0_D)
unsigned char set_output(unsigned char device, unsigned char state, unsigned char inverted)
{
unsigned char i,error;

error = 0;

i = state;
if (inverted)
	i = 1 - i;

if (device < SR_THRESHOLD)	// its a GPIO pin...
	{
	error = gpio_set_level((gpio_num_t)devpin[device],i);		// invert output if necessary...
	}
else					// its a shift register pin...
	{
//error = gpio_set_level(devpin[device],i);		// invert output if necessary...
	if (i)
		sd_data_out = sd_data_out | (1<<(device - SR_THRESHOLD));
	else
		sd_data_out = sd_data_out & (~(1<<(device - SR_THRESHOLD)));

//	printf("SD %02X\n",sd_data_out);
	
	shift_out(sd_data_out);
	}
	
status_array[device] = state;	// write logical state to status array, not actual value
//	printf("SET %d %d %d %s %d\r\n",i,device,devpin[device],devname[device],state);


if ((error) || (debug_do(DBG_SD_IN_OUT)))
	printf("SET %d %d %s %d\r\n",device,devpin[device],devname[device],state);

return error;
}
/*
unsigned char get_input(unsigned char device, unsigned char inverted)
{
unsigned char state;
	
//state = gpio_get_level(devpin[device]);	
if (sd_data_in & (1<<sd_in_device[device]))
	state = 1;
else
	state = 0;

if (inverted)
	state = 1 - state;

status_array[device] = state;

return state;
	
}
*/
#endif




////////////////////////////////
// IO shift register routines
////////////////////////////////
#if SHIFT_REG_LENGTH == 16
unsigned char shift_out(unsigned int sd_data_out)
#else
unsigned char shift_out(unsigned char sd_data_out)
#endif
{
// shift register control: shift output bits out
// get outputs to "safe" positions

unsigned char i,sdbit;
#if SHIFT_REG_LENGTH == 16
unsigned int mask;
#else
unsigned char mask;
#endif

#if SHIFT_REG_LENGTH == 16
//	printf("SR_16\n");
#else
//	printf("SR_8\n");
#endif

//	sd_data_out = 0;
	
/*
// outputs are in locations 8-15...
	for (i=8;i<16;i++)
		{
		if(status_array[i])
			sd_data_out = sd_data_out | mask;
		
		mask = mask << 1;
		}
*/
	if(debug_do(DBG_SD_IN_OUT))
		{
		printf("SDOUT: %02X\r\n",sd_data_out);
		}
	
	gpio_set_level(SR_STB,0);

//	mask = 0x80;
	mask = (1<<(SHIFT_REG_LENGTH-1));
	
	for (i=0;i<SHIFT_REG_LENGTH;i++)
		{
		gpio_set_level(SR_CLK,0);
		ets_delay_us(1);
	
		if (sd_data_out & mask)
			sdbit = 1;
		else
			sdbit = 0;
		
		gpio_set_level(SR_DATA,sdbit);
// this code length acts as a setup delay for the data bit...
		mask = mask >> 1;
		
// shift the data in after a short setup delay 
		gpio_set_level(SR_CLK,1);
		ets_delay_us(1);

		}

	gpio_set_level(SR_CLK,0);
	gpio_set_level(SR_STB,0);
	ets_delay_us(1);

	gpio_set_level(SR_STB,1);
	ets_delay_us(1);
	gpio_set_level(SR_STB,0);

return 0;
}

void set_led(unsigned char led, unsigned char mode)
{
led_modes[led] = mode;

if (mode < FLASH)		// ie, DEVICE_OFF or DEVICE ON...
	{
	led_states = (led_states & ~(1<<led)) | (mode<<led);	
// immediate update of LED output...
	if (prev_led_states != led_states)
		{
#if LED_POLARITY == NOT_INVERTED
		sd_data_out = (sd_data_out & ~(0x0F << (DEV_LED_0 - SR_THRESHOLD))) | ((led_states & 0x0F) << (DEV_LED_0 - SR_THRESHOLD));
#else
		sd_data_out = (sd_data_out & ~(0x0F << (DEV_LED_0 - SR_THRESHOLD))) | ((~led_states & 0x0F) << (DEV_LED_0 - SR_THRESHOLD));
#endif
		shift_out(sd_data_out);
		prev_led_states = led_states;
		}
	}
}

//////////////////////////////////////////////////
//
// CLI routine
//
//////////////////////////////////////////////////
unsigned char cli(unsigned char* clistr)
{
char c;
unsigned char cmd, i, j, k, p, q, r, error, dbgbit;
char str[80];
char cmdstr[10];
char paramstr[4][40];
char paramstr1[80];
char paramstr2[80];
char paramstr2all[80];
char paramstr3all[80];
unsigned char param;
unsigned char num_params;

uint8_t mac_addr;

unsigned int mask;
unsigned int ival,x,y;

unsigned char tm_ptr;

nvs_handle nvs_handle;

static const char* TAG = "CLI";

//printf("CLI\r\n");

error = 0;

cmd = 0xFF;
p = 0;

skip_whitespace(clistr, &p);

strncpy(cmdstr,(char *)&clistr[p],CMDLEN);
cmdstr[CMDLEN] = 0x00;

p = p + CMDLEN;

toupperstr(cmdstr);

if (debug_do(DBG_CLI))
	printf("CMDSTR=%s  ",cmdstr);

for (i=0;i<NUM_CMDS;i++)
	{
	if(!strncmp(cmdlist[i],cmdstr,CMDLEN))
		{
		cmd = i;
		i = 0xF0;
		}
	
	}

if (cmd != 0xFF)
	{
	if (debug_do(DBG_CLI))
		dbgprintf(DBG,"CMD=%s  ",cmdstr);
	
	skip_whitespace(clistr, &p);
	q = p;

	paramstr2all[0] = 0x00;
	paramstr3all[0] = 0x00;

// get 1st parameter without conversion to upper case
	get_param(clistr, &q, paramstr1,0);			
	j = skip_whitespace(clistr, &q);				// skip the space

	if (j == 0)
		{
// get whole str from 2nd parm to end; remove CR-LF but no case conversion (SSIDs and passwords with spaces!)
		r = q;							// dont alter the value of q! (Needed later)
		c = clistr[r];
		k = 0;
		while((c != 0x00) && (c != 0x0D) && (c != 0x0A))
			{
			paramstr2all[k++] = c;
			r++;
			c = clistr[r];
			}
		paramstr2all[k] = 0x00;
	
// get 2nd parameter without conversion to upper case
		get_param(clistr, &q, paramstr2,0);			
		j = skip_whitespace(clistr, &q);				// skip the space

		if (j == 0)
			{
// get whole str from 3nd parm to end; remove CR-LF but no case conversion (SSIDs and passwords with spaces!)
			r = q;							// dont alter the value of q! (Needed later)
			c = clistr[r];
			k = 0;
			while((c != 0x00) && (c != 0x0D) && (c != 0x0A))
				{
				paramstr3all[k++] = c;
				r++;
				c = clistr[r];
				}
			paramstr3all[k] = 0x00;
			}	
		}
		
// now get all params individually
	num_params = 0;
	i = 0;
	while((i == 0) && (num_params<4))
		{
		i = get_param(clistr, &p, paramstr[num_params],1);
		skip_whitespace(clistr, &p);			// skip the space
		
		if (debug_do(DBG_CLI))
			dbgprintf(DBG,"PRM[%d]=%s\r\n",num_params,paramstr[num_params]);

		if (i < 0x80)		// zero parm length flag bit = 0x80
			num_params++;	

		}
		

	switch(cmd)
		{
		case CMD_HEL:
			printf("Command Help:\r\n");
			
			for (i=0;i<NUM_CMDS;i++)
				{
				printf("%s\r\n",cmdlist[i]);
				}
			printf("\r\n");
			break;
			
		case CMD_DBG:
			if (!strcmp(paramstr[0],"ON"))
				dbgflag = 1;
			else if (!strcmp(paramstr[0],"OFF"))
				dbgflag = 0;
			else if (paramstr[0][0] == 'S')
				{
				i = atoi(&paramstr[0][1]);
				dbgbits = dbgbits | (1<<i);
				}
			else if (paramstr[0][0] == 'R')
				{
				i = atoi(&paramstr[0][1]);
				dbgbits = dbgbits & ~(1<<i);
				}
			else if (strlen(paramstr[0]))
				dbgbits = atoi(paramstr[0]);
		
// display Debug state:
			if (dbgflag)
				strcpy(str,"ON ");
			else
				strcpy(str,"OFF");
			dbgprintf(DBG,"DBG: %s  [%08lX]  ",str,dbgbits);
			mask = 0x80000000;
			for (i=0;i<32;i++)
				{
				if (dbgbits & mask)
					dbgbit = 1;
				else
					dbgbit = 0;
				dbgprintf(DBG,"%d",dbgbit);
				mask = mask >> 1;
				if (i%4 == 3)
					dbgprintf(DBG," ");
				if (i%8 == 7)
					dbgprintf(DBG," ");
				if (i%16 == 15)
					dbgprintf(DBG," ");
					
				
				}
			dbgprintf(DBG,"\r\n");	
			break;

		case CMD_STA:
			show_status(DBG);
			break;

		case CMD_INF:
			show_inf(1);
			
			break;

		
		case CMD_RTM:
/*
			if ((paramstr[0] == 'R') || (paramstr[0] == 'r'))
				lora_radio_msg_time = DFLT_LORA_RADIO_MSG_TIME;
			else if (strlen(paramstr))
				{
				ival = atoi(paramstr);
//				if ((ival >0) && (ival <65000))
				if (ival <65000)
					lora_radio_msg_time = ival;
				}

			dbgprintf(DBG,"Radio Msg Time = %dsec [%dmin %dsec]",lora_radio_msg_time/10,lora_radio_msg_time/600,lora_radio_msg_time%600/10);
			if (!lora_radio_msg_time)
				dbgprintf(DBG,"  [Radio Msgs OFF]\r\n");
			
			dbgprintf(DBG,"\r\n");
*/
			break;
		
		case CMD_RTX:
/*
if (strlen(paramstr))
				{
				ival = atoi(paramstr);
				if ((ival >0) && (ival <256))
					lora_radio_test_count = ival;
				lora_radio_test_flag = 0;
				}
			else
				{
				lora_radio_test_count = 0;
				lora_radio_test_flag = 1;
				}
*/			
			break;
			
			
		case CMD_TIM:
//			dbgprintf(DBG,"System Time: %dd %02d:%02d:%02d\t", days, hrs, mins, secs);
			i = 0;
			if (strlen(paramstr[0]))
				{
				if ((paramstr[0][0] == 'L') || (paramstr[0][0] == 'l'))
					{
					i = 2;
					}
				else
					{
					ival = atoi(paramstr[0]);
//				if ((ival >0) && (ival <65000))
					if (ival <65000)
						TIM_msg_time = ival;
					}
				}

			show_time(DBG,i);
			if (cell_rssi != -127)
				printf("[%ddBm]",cell_rssi);
			else
				printf("[---]");

			if (ntptime_sync_flag)
				dbgprintf(DBG,"\t[SYNC]");
			else
				dbgprintf(DBG,"   [NOT SYNCED]");
			
			if (TIM_msg_time)
				dbgprintf(DBG,"   [%5d]",TIM_msg_time);

			dbgprintf(DBG,"\r\n");
				
			break;
	
		case CMD_WFL:
#ifdef WIFI_SCAN
			printf("Wifi Lists:\r\n");
			
			printf("\r\nCurrent [%2d]:\r\n",wifi_current_num);
			for (x=0;x<wifi_current_num;x++)
				{
				for (i=0;i<6;i++)
					{
					dbgprintf(DBG,"%02X",wifi_list_current[x].bssid[i]);
					if (i<5)
						dbgprintf(DBG,":");
					else
						dbgprintf(DBG,"  ");
					}

//				printf("%02X:%02X:%02X:%02X:%02X:%02X:  ",1,2,3,4,5,6);
				if ((x+1)%4 == 0)
					printf("\n");

				}
			printf("\r\n");

			printf("\r\nPrevious [%2d]:\r\n",wifi_previous_num);
			for (x=0;x<wifi_previous_num;x++)
				{
				for (i=0;i<6;i++)
					{
					dbgprintf(DBG,"%02X",wifi_list_previous[x].bssid[i]);
					if (i<5)
						dbgprintf(DBG,":");
					else
						dbgprintf(DBG,"  ");
					}
					

//				printf("%02X:%02X:%02X:%02X:%02X:%02X:  ",1,2,3,4,5,6);
				if ((x+1)%4 == 0)
					printf("\n");

				}
			printf("\r\n\r\n");
#endif
			break;
		
		case CMD_WFS:		
			if (!strcmp(paramstr[0],"ON"))
				wifi_scan_enable_flag = 1;
			else if (!strcmp(paramstr[0],"OFF"))
				wifi_scan_enable_flag = 0;

// display wifi_scan_enable state:
			if (wifi_scan_enable_flag)
				strcpy(str,"ON ");
			else
				strcpy(str,"OFF");
			dbgprintf(DBG,"Wifi Scan: %s\r\n",str);

			break;

		case CMD_WTM:
			if ((paramstr[0][0] == 'R') || (paramstr[0][0] == 'r'))
				wifi_scan_time = DFLT_WIFI_SCAN_TIME;
			else if (strlen(paramstr[0]))
				{
				ival = atoi(paramstr[0]);
//				if ((ival >0) && (ival <65000))
				if (ival <65000)
					wifi_scan_time = ival;
				}

			dbgprintf(DBG,"Wifi Scan Time = %dsec [%dmin %dsec]",wifi_scan_time,wifi_scan_time/60,wifi_scan_time%60);

			if (!wifi_scan_time)
				{
//				wifi_scan_enable_flag = 0;
				dbgprintf(DBG,"  [Wifi Scan OFF]\r\n");
				}
/*
			else
				{
				wifi_scan_enable_flag = 1;
				}
*/			
			dbgprintf(DBG,"\r\n");
			break;
		

		case CMD_TMS:
			tm_ptr = 0;
			p = 3;
			k = skip_whitespace(clistr, &p);
			j = 0;

			if((clistr[p] & 0x5F) == 'R')
				init_tm_data(0);
			else if((clistr[p] & 0x5F) == 'V')
				init_tm_data(1);

			else if(clistr[p] == '#')
				{
//				printf("HASH\r\n");
				p++;
				get_param(clistr, &p, str,1);
				tm_ptr = atoi(str);
				skip_whitespace(clistr, &p);
				j = 1;	
				}
			else if (!k)
				j = 1;	
		
			while(j)
				{
//				printf("TMS TEST\r\n");
				get_param(clistr, &p, str,1);
				tm_data[tm_ptr] = strtol(str,NULL,16);
				tm_ptr++;
				k = skip_whitespace(clistr, &p);
				if ((tm_ptr > 13) || (k))
					j = 0;
				}			
			
// show test message
			dbgprintf(DBG,"BYTE:  ");
			for (i=0;i<13;i++)
				{
				dbgprintf(DBG,"%5d",i);
				}
			dbgprintf(DBG,"\r\n");
				
			if(tm_data[1] == STATUS_MSG)
				dbgprintf(DBG,"STATUS:  LEN  TYPE TANK DSPh DSPl L100 TEMP BATT MTRI ERR  DEVh DEVl SPARE\r\n");
			else if(tm_data[1] == GEOLOCATION_MSG)
				dbgprintf(DBG,"GEOLOC:  LEN  TYPE SEQ  ADR0 ADR1 ADR2 ADR3 ADR4 ADR5 RSSI\r\n");
			else
				dbgprintf(DBG,"UNKNWN:  ???  ???  ???  ???  ???  ???  ???  ???  ???  ???  ???\r\n");
				
			dbgprintf(DBG,"VAL:    ");
			
			for (i=0;i<13;i++)
				{
				dbgprintf(DBG,"  %02X ",tm_data[i]);				
				}

			dbgprintf(DBG,"\r\n");

			break;

		case CMD_SES:
			if (strlen(paramstr[0]))
				{
				ival = atoi(paramstr[0]);
//				if ((ival >0) && (ival <65000))
				if ((ival <65000) && (ival > 50))
					{
					uart_stay_awake_time = ival;
					uart_stay_awake_count = ival;
					}

				}


			dbgprintf(DBG,"UART stay awake time = %dsec\r\n",uart_stay_awake_time/10);
			
			break;

		case CMD_OPT:
			show_options();
			break;

		case CMD_M4G:
			four_g_tx_length = four_g_pack_data(paramstr[0],(char *)four_g_tx_data);		// respond with number <cr-lf> OK
			four_g_send_data(four_g_tx_data,four_g_tx_length);
			four_g_rcv_data(four_g_rx_data, &four_g_rx_length,1,100);
			printf("Rx: < %s >  [%d]\r\n",four_g_rx_data, four_g_rx_length);
			
			break;

		case CMD_SSL:
#ifdef USE_M4G_MQTT_SSL
			if (!strcmp(paramstr[0],"ON"))
				ssl_enable_flag = 1;
			else if (!strcmp(paramstr[0],"OFF"))
#endif			
				ssl_enable_flag = 0;

// display wifi_scan_enable state:
#ifdef USE_M4G_MQTT_SSL
			if (ssl_enable_flag)
				strcpy(str,"ON ");
			else
#endif			
				strcpy(str,"OFF");
			
			dbgprintf(DBG,"SSL: %s\r\n",str);

			break;

		case CMD_BWL:
//			printf("Black / White lists:\n");
			if (strlen(paramstr[0]))
				{
				i = atoi(paramstr[0]);
				}
			else
				{
				i = 15;
				}
#ifdef USE_BLUETOOTH
			if (i & 2)
				{
				unsigned char x,y,z;
				char *p;
				
				printf("Bluetooth Whitelist:\n");
				printf("whitelist enable: %d     entries: %d\n",enable_bluetooth_whitelist_flag,num_bt_wl_entries);
				
				x=0;		// counts valid entries
				y=0;		// index into table
				
//				for (unsigned char i=0;i<num_bt_wl_entries;i++)
				while((x < num_bt_wl_entries) && (y < BLUETOOTH_CMD_LIST_LENGTH))
					{
					p = (char *)&bluetooth_cmd_whitelist[y];
					printf("%03d) [ ",y);
					z = 0;
					for (unsigned char j = 0; j<6;j++)
						{
						printf("%02X ",*(p+j));
						if (*(p+j) != 0)
							z = 1;
						}
					printf("]\n");
					if (z)				// if address was valid \ non-zero, increment the valid entry count
						x++;
					y++;				// always increment the table index
					}

				printf("\nBlacklist:\n");
				printf("blacklist enable: %d     entries: %d\n",enable_bluetooth_blacklist_flag,num_bt_bl_entries);
				for (unsigned char i=0;i<num_bt_bl_entries;i++)
					{
					char *p = (char *)&bluetooth_cmd_blacklist[i];
					printf("%03d) [ ",i);
					for (unsigned char j = 0; j<6;j++)
						{
						printf("%02X ",*(p+j));
						}
					printf("]\n");
					}
				}
#endif				
#ifdef USE_LORA
			if (i & 4)
				{
				printf("LoRaWAN Whitelist:\n");
				printf("whitelist enable: %d     entries: %d\n",enable_lorawan_whitelist_flag,num_lora_wl_entries);
				for (unsigned char i=0;i<num_lora_wl_entries;i++)
					{
					char *p = (char *)&lorawan_cmd_whitelist[i];
					printf("%03d) [ ",i);
					for (unsigned char j = 0; j<4;j++)
						{
						printf("%02X ",*(p+j));
						}
					printf("]\n");
					}

				printf("\nBlacklist:\n");
				printf("blacklist enable: %d     entries: %d\n",enable_lorawan_blacklist_flag,num_lora_bl_entries);
				for (unsigned char i=0;i<num_lora_bl_entries;i++)
					{
					char *p = (char *)&lorawan_cmd_blacklist[i];
					printf("%03d) [ ",i);
					for (unsigned char j = 0; j<4;j++)
						{
						printf("%02X ",*(p+j));
						}
					printf("]\n");
					}
				}
#endif				

#ifdef USE_BLUETOOTH
			if (i & 8)
				{
				printf("Bluetooth Beacon Device Whitelist:\n");

				for (unsigned char i=0;i<num_bt_devname_wl_entries;i++)
					{
					printf("%03d)  [%s]\n", i,bluetooth_devname_whitelist[i]);
					}				
				}
#endif	

			break;
//CMD_BWE,		// BlueTooth whitelist status \ enable
//CMD_BBE,		// BlueTooth blacklist status \ enable
//CMD_LWE,		// LoRaWAN whitelist status \ enable
//CMD_LBE,		// LoRaWAN blacklist status \ enable

		case CMD_SWE:
			break;

		case CMD_SBE:
			break;

		case CMD_BWE:
#ifdef USE_BLUETOOTH
			if (strlen(paramstr[0]))
				{
				ival = atoi(paramstr[0]);
				if (ival <2)				// 0 or 1
					{
					enable_bluetooth_whitelist_flag = ival;
					nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
					nvs_set_u8(nvs_handle,"GW_BWL_FLAG",(uint8_t)enable_bluetooth_whitelist_flag);
					nvs_commit(nvs_handle);		
					nvs_close(nvs_handle);
					}
				}

			dbgprintf(DBG,"BlueTooth Whitelist Enabled: %d\n",enable_bluetooth_whitelist_flag);
#endif
			break;

		case CMD_BBE:
#ifdef USE_BLUETOOTH
			if (strlen(paramstr[0]))
				{
				ival = atoi(paramstr[0]);
				if (ival <2)				// 0 or 1
					{
					enable_bluetooth_blacklist_flag = ival;
					nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
					nvs_set_u8(nvs_handle,"GW_BBL_FLAG",(uint8_t)enable_bluetooth_blacklist_flag);
					nvs_commit(nvs_handle);		
					nvs_close(nvs_handle);
					}
				}

			dbgprintf(DBG,"BlueTooth Blacklist Enabled: %d\n",enable_bluetooth_blacklist_flag);
#endif
			break;

		case CMD_BDE:
#ifdef USE_BLUETOOTH
			if (strlen(paramstr[0]))
				{
				ival = atoi(paramstr[0]);
				if (ival <2)				// 0 or 1
					{
					enable_bluetooth_devname_whitelist_flag = ival;
					nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
					nvs_set_u8(nvs_handle,"GW_BDL_FLAG",(uint8_t)enable_bluetooth_devname_whitelist_flag);
					nvs_commit(nvs_handle);		
					nvs_close(nvs_handle);
					}
				}

			dbgprintf(DBG,"BlueTooth Device list Enabled: %d\n",enable_bluetooth_devname_whitelist_flag);
#endif
			break;

		case CMD_LWE:
#ifdef USE_LORA
			if (strlen(paramstr[0]))
				{
				ival = atoi(paramstr[0]);
				if (ival <2)				// 0 or 1
					{
					enable_lorawan_whitelist_flag = ival;
					nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
					nvs_set_u8(nvs_handle,"GW_LWL_FLAG",(uint8_t)enable_lorawan_whitelist_flag);
					nvs_commit(nvs_handle);		
					nvs_close(nvs_handle);
					}
				}

			dbgprintf(DBG,"LoRaWAN Whitelist Enabled: %d\n",enable_lorawan_whitelist_flag);
#endif
			break;

		case CMD_LBE:
#ifdef USE_LORA
			if (strlen(paramstr[0]))
				{
				ival = atoi(paramstr[0]);
				if (ival <2)				// 0 or 1
					{
					enable_lorawan_blacklist_flag = ival;
					nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
					nvs_set_u8(nvs_handle,"GW_LBL_FLAG",(uint8_t)enable_lorawan_blacklist_flag);
					nvs_commit(nvs_handle);		
					nvs_close(nvs_handle);
					}
				}

			dbgprintf(DBG,"LoRaWAN Blacklist Enabled: %d\n",enable_lorawan_blacklist_flag);
#endif
			break;

		case CMD_LOR:
//#ifdef USE_LORA_USED
#ifdef LORA_USED
			printf("LORA status pins:\n");
#if 0
			printf("DIO0: %d\n",gpio_get_level((gpio_num_t)LORA_DIO0));
			printf("DIO1: %d\n",gpio_get_level((gpio_num_t)LORA_DIO1));
			printf("DIO2: %d\n",gpio_get_level((gpio_num_t)LORA_DIO2));
			printf("DIO3: %d\n",gpio_get_level((gpio_num_t)LORA_DIO3));
			printf("DIO4: %d\n",gpio_get_level((gpio_num_t)LORA_DIO4));
#else
			printf("DIO4:0:   ");
			printf(" %d",gpio_get_level((gpio_num_t)LORA_DIO4));
			printf(" %d",gpio_get_level((gpio_num_t)LORA_DIO3));
			printf(" %d",gpio_get_level((gpio_num_t)LORA_DIO2));
			printf(" %d",gpio_get_level((gpio_num_t)LORA_DIO1));
			printf(" %d\n",gpio_get_level((gpio_num_t)LORA_DIO0));
#endif
			{
			unsigned char reg, val;
			SX1276_read_reg(spi,LREG_OPMODE,&reg,1);
			printf("LORA OP mode reg: %02X\n",reg);

			SX1276_read_reg(spi,LREG_IRQ_FLAGS,&reg,1);
			printf("LORA int reg: %02X\n",reg);

			SX1276_read_reg(spi,LREG_MDM_STAT,&reg,1);
			printf("LORA mdm stat: %02X\n",reg);

			SX1276_read_reg(spi,LREG_RX_NUM_BYTES,&reg,1);
			printf("LORA pkt size: %02X\n",reg);

			SX1276_read_reg(spi,LREG_RX_HDR_COUNT_MSB,&val,1);
			SX1276_read_reg(spi,LREG_RX_HDR_COUNT_LSB,&reg,1);
			printf("LORA HDR Count: %02X%02X\n",val,reg);

			SX1276_read_reg(spi,LREG_RX_PKT_COUNT_MSB,&val,1);
			SX1276_read_reg(spi,LREG_RX_PKT_COUNT_LSB,&reg,1);
			printf("LORA PKT Count: %02X%02X\n",val,reg);

			printf("\nLORA msg ready Count: %d\n",lora_msg_ready_count);
			}
#endif			
			printf("\n\n");
			
			break;

		case CMD_MQT:
			if (!strcmp(paramstr[0],"ON"))
				{
				mqtt_disable_flag = 0;
				wifi_disable_flag = 0;
				simcom_disable_flag = 0;
				}
			else if (!strcmp(paramstr[0],"OFF"))
				{
				mqtt_disable_flag = 1;
				wifi_disable_flag = 1;
				if (udp_disable_flag)
					simcom_disable_flag = 1;
				}

			printf("MQTT enable: ");
			if (mqtt_disable_flag)
				printf("OFF\n");
			else
				printf("ON\n");
							
			break;

		case CMD_SRV:
			{
			unsigned char val;
			unsigned int ival;
			unsigned long long int lval = 0;
			unsigned int n;
			char tmpstr[10];
			esp_err_t err;

//			nvs_handle nvs_handle;
			
			printf("Server side settings:\n");
			printf("Var:\t\tVal:\t\tNVS val:\n");
			
			err = nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);
			
			if(err == ESP_OK)
				{
				nvs_get_u8(nvs_handle,"GW_AUTOBAUD",(uint8_t*)&val);
				printf("GW_AUTOBAUD\t%02X\t\t%02X\n",serial_autobaud_flag,val);
				nvs_get_u32(nvs_handle,"GW_SER_RATE",(uint32_t*)&ival);
				printf("GW_SER_RATE\t%ldb/s\t\t%lldb/s\n",serial_bitrate,lval);
				nvs_get_u8(nvs_handle,"GW_BIN_FLAG",(uint8_t*)&val);
				printf("GW_BIN_FLAG\t%02X\t\t%02X\n",binary_data_flag,val);

//			nvs_get_str(nvs_handle,"GW_EOL_STR",eolstr,&n);					// n is number of bytes read out...
//			printf("GW_SER_RATE\t%02X\t\t%02X\n",binary_data_flag,val);


#ifdef USE_BLUETOOTH			
				nvs_get_u8(nvs_handle,"GW_BWL_FLAG",(uint8_t*)&val);								// Bluetooth whitelist enable flag
				printf("GW_BWL_FLAG\t%02X\t\t%02X\n",enable_bluetooth_whitelist_flag,val);
							
				nvs_get_u8(nvs_handle,"GW_BBL_FLAG",(uint8_t*)&val);								// Bluetooth blacklist enable flag
				printf("GW_BBL_FLAG\t%02X\t\t%02X\n",enable_bluetooth_blacklist_flag,val);
#endif
#ifdef USE_LORA
				nvs_get_u8(nvs_handle,"GW_LWL_FLAG",(uint8_t*)&val);								// LoRaWAN whitelist enable flag
				printf("GW_LWL_FLAG\t%02X\t\t%02X\n",enable_lorawan_whitelist_flag,val);
				
				nvs_get_u8(nvs_handle,"GW_LBL_FLAG",(uint8_t*)&val);								// LoRaWAN blacklist enable flag
				printf("GW_LBL_FLAG\t%02X\t\t%02X\n",enable_lorawan_blacklist_flag,val);
#endif
//			nvs_set_u8(nvs_handle,"GW_EOLREM_FLAG",(uint8_t)eol_remove_flag);

				nvs_close(nvs_handle);
				}
			else
				printf("Error oopening NVS Flash storage\n");
/*			
nvs_get_u8(nvs_handle,"GW_AUTOBAUD",(uint8_t*)&serial_autobaud_flag);
nvs_get_u32(nvs_handle,"GW_SER_RATE",(uint32_t*)&serial_bitrate);
nvs_get_u8(nvs_handle,"GW_BIN_FLAG",(uint8_t*)&binary_data_flag);
nvs_get_str(nvs_handle,"GW_EOL_STR",eolstr,&n);					// n is number of bytes read out...
printf("Serial_AutoBaud_Flag = %d\t",serial_autobaud_flag);
printf("Serial_BitRate       = %ld\n",serial_bitrate);
printf("Serial_Binary_Flag   = %d\t",binary_data_flag);
printf("Serial EOL String    = ");
i = 0;
while (eolstr[i] != 0x00)
	{
	printf("%02X ",eolstr[i]);		
	i++;
	}
printf("\n\n");

nvs_get_u8(nvs_handle,"GW_BWL_FLAG",(uint8_t*)&enable_bluetooth_whitelist_flag);		// Bluetooth whitelist enable flag
nvs_get_u8(nvs_handle,"GW_BBL_FLAG",(uint8_t*)&enable_bluetooth_blacklist_flag);		// Bluetooth blacklist enable flag
nvs_get_u8(nvs_handle,"GW_LWL_FLAG",(uint8_t*)&enable_lorawan_whitelist_flag);		// LoRaWAN whitelist enable flag
nvs_get_u8(nvs_handle,"GW_LBL_FLAG",(uint8_t*)&enable_lorawan_blacklist_flag);		// LoRaWAN blacklist enable flag
nvs_set_u8(nvs_handle,"GW_EOLREM_FLAG",(uint8_t)eol_remove_flag);
*/
			}
			break;

		case CMD_TMR:
			printf("System timer settings:\n");
			printf("Timer:\t\t\t\t Val:\t\t\t Current:\n");

			printf("Get Server Time        :\t%4d sec\t\t[current: %4d]\n",GET_SERVER_TIME_PERIOD * 60,get_server_time_timer);
			printf("Gateway Sensor         :\t%4d sec\t\t[current: %4d]\n",gateway_sensor_time,gateway_sensor_timer);
			printf("GPS Sensor             :\t%4d sec\t\t[current: %4d]\n",gps_sensor_time,gps_sensor_timer);
#ifdef WIFI_SCAN
			printf("Wifi Scan              :\t%4d sec\t\t[current: %4d]\n",wifi_scan_time,wifi_scan_timer);
#endif
#ifdef USE_I2C
			printf("I2C Sensor             :\t%4d sec\t\t[current: %4d]\n",i2c_sensor_time,i2c_sensor_timer);
#endif
#ifdef USE_TANK_SENSORS
			printf("TankSensor             :\t%4d sec\t\t[current: %4d]\n",tank_sensor_time,tank_sensor_timer);
#endif
#ifdef USE_BLUETOOTH
			printf("Bluetooth Sensor       :\t%4d sec\t\t[current: %4d]\n",bluetooth_sensor_time,bluetooth_sensor_timer);
	#ifdef BLUETOOTH_SCAN
			printf("Bluetooth Scan         :\t%4d sec\t\t[current: %4d]\n",bluetooth_MAC_addr_time,bluetooth_MAC_addr_timer);
	#endif
			printf("Bluetooth Beacon       :\t%4d sec\t\t[current: %4d]\n",bluetooth_devname_time,bluetooth_devname_timer);
	#ifdef USE_BLUETOOTH_CLASSIC_DETECT
			printf("Bluetooth Classic Scan :\t%4d sec\t\t[current: %4d]\n",bluetooth_classic_scan_time,bluetooth_classic_scan_timer);
	#endif
#endif
#ifdef USE_LORA
			printf("LoRaWAN Scan           :\t%4d sec\t\t[current: %4d]\n",lora_devaddr_time,lora_devaddr_timer);
#endif

			break;

		case CMD_NVG:
			printf("Non-Volatile Memory - Factory Init Gateway Values\n");
			nvs_gw_var_init();
			set_gw_vars_from_nvs();
			break;
	
		case CMD_NVS:		// initialise  white \ black lists
			break;
	
		case CMD_NVB:		// initialise bluetooth white \ black lists
#ifdef USE_BLUETOOTH
//			nvs_bl_var_init("BT_MAC_LIST", "BTW_MAC_MAX", "BTW_MAC");
			nvs_bl_var_init("BT_MAC_LIST", "BTW_MAC_MAX", "BTW_MAC", (char*)&bluetooth_wl_init, bluetooth_wl_init_len, BLUETOOTH_CMD_LIST_ENTRY_LENGTH,BINARY_VAL);
			set_bl_vars_from_nvs("BT_MAC_LIST", "BTW_MAC_MAX", "BTW_MAC", (char*)&bluetooth_cmd_whitelist, &num_bt_wl_entries, BLUETOOTH_CMD_LIST_ENTRY_LENGTH,BINARY_VAL);

			nvs_bl_var_init("BT_MAC_LIST", "BTB_MAC_MAX", "BTB_MAC", (char*)&bluetooth_bl_init, bluetooth_bl_init_len, BLUETOOTH_CMD_LIST_ENTRY_LENGTH,BINARY_VAL);
			set_bl_vars_from_nvs("BT_MAC_LIST", "BTB_MAC_MAX", "BTB_MAC", (char*)&bluetooth_cmd_blacklist, &num_bt_bl_entries, BLUETOOTH_CMD_LIST_ENTRY_LENGTH,BINARY_VAL);

			nvs_bl_var_init("BD_MAC_LIST", "BDW_MAC_MAX", "BDW_MAC", (char*)&bluetooth_dw_init, bluetooth_dw_init_len, BLUETOOTH_DEVNAME_LIST_ENTRY_LENGTH,ASCII_VAL);
			set_bl_vars_from_nvs("BD_MAC_LIST", "BDW_MAC_MAX", "BDW_MAC", (char*)&bluetooth_devname_whitelist, &num_bt_devname_wl_entries, BLUETOOTH_DEVNAME_LIST_ENTRY_LENGTH,ASCII_VAL);
#endif	
			break;
	
		case CMD_NVL:		// initialise LoRaWAN white \ black lists
#ifdef USE_LORA

#endif
			break;
	

		case CMD_NVX:		// initialise Bluetooth Sensor details
#ifdef USE_BLUETOOTH
			printf("*** setting BT Sensor values...\n");
			nvs_open("BT_SENSOR",NVS_READWRITE, &nvs_handle);

			nvs_set_str(nvs_handle,"BTS_NAME",BLUETOOTH_ADVERTISING_NAME);
			nvs_set_u16(nvs_handle,"BTS_ADV_TIME",(uint16_t)BLUETOOTH_ADVERTISING_TIME);

			nvs_commit(nvs_handle);		
			nvs_close(nvs_handle);
			
			strcpy(bluetooth_sensor_adv_name,BLUETOOTH_ADVERTISING_NAME);
			bluetooth_sensor_adv_time = BLUETOOTH_ADVERTISING_TIME;

			bleAdvtTask();		// restart BT advertising with new parameters
#endif			
			break;

		case CMD_SCT:
#ifdef USE_M4G_MQTT		
			if (strlen(paramstr[0]))
				{
				printf("Server Cmd Test:\n");
				strcpy(server_cmd_test_str,paramstr1);	// paramstr1 is long string and  has not been converted to upper case
				server_cmd_test_flag = 1;
				}	
#endif				
			break;

		case CMD_BSN:
#ifdef USE_BLUETOOTH
//			printf("PRM: [%s]\n",paramstr);
			i = strlen(paramstr[0]);
			if ((i > 0) && (i <= 10))
				{
				strcpy(bluetooth_sensor_adv_name,paramstr[0]);
				nvs_open("BT_SENSOR",NVS_READWRITE, &nvs_handle);
				nvs_set_str(nvs_handle,"BTS_NAME",bluetooth_sensor_adv_name);
				nvs_commit(nvs_handle);		
				nvs_close(nvs_handle);

				bleAdvtTask();		// restart BT advertising with new parameters
				}

			dbgprintf(DBG,"BlueTooth Sensor Local Device Name:  %s\n",bluetooth_sensor_adv_name);
#endif
			break;
		
		case CMD_BST:
#ifdef USE_BLUETOOTH
			if (strlen(paramstr[0]))
				{
				ival = atoi(paramstr[0]) * 16/10;	// value is 1.6x time in msec
				if (ival > 160)					// 160 = 100msec
					{
					bluetooth_sensor_adv_time = ival;
					nvs_open("BT_SENSOR",NVS_READWRITE, &nvs_handle);
					nvs_set_u16(nvs_handle,"BTS_ADV_TIME",(uint16_t)bluetooth_sensor_adv_time);
					nvs_commit(nvs_handle);		
					nvs_close(nvs_handle);
					
					bleAdvtTask();		// restart BT advertising with new parameters
					}
				}

			dbgprintf(DBG,"BlueTooth Sensor Advertising Time:  %d.%d sec  [%d msec]\n",bluetooth_sensor_adv_time/1600,(bluetooth_sensor_adv_time*10/16)%1000,bluetooth_sensor_adv_time*10/16);
#endif
			break;
		
		case CMD_I2C:
#ifdef USE_I2C		
			check_i2c_devices_present();
#endif			
			break;
			
		case CMD_I2B:
#ifdef USE_I2C		
			printf("Check for devices on the I2C bus at address: \n");
			check_i2c_bus(0);
#endif			
			break;

		case CMD_TZR:
#ifdef ZULU_RADIO_MASTER
			printf("Setting run flag for test...\n");
			zulu_radio_run_flag = 1;
#endif
			break;

		case CMD_ZRM:
#ifdef USE_ZULU_RADIO
			if (!strcmp(paramstr[0],"ON"))
				zulu_radio_msgs_flag = 1;
			else if (!strcmp(paramstr[0],"OFF"))
				zulu_radio_msgs_flag = 0;

			printf("Zulu Radio msgs: ");
			if (zulu_radio_msgs_flag)
				printf("ON\n");
			else
				printf("OFF\n");
#endif							
			break;
			
		case CMD_ZRC:
#ifdef USE_ZULU_RADIO
			{
			char tmpstr[10];

#ifdef ZULU_RADIO_MASTER			
		gpio_set_level(ZULU_RADIO_MASTER_RTS,1);	// RTS is active LOW
#endif
#ifdef ZULU_RADIO_SLAVE			
		gpio_set_level(ZULU_RADIO_SLAVE_RTS,1);		// RTS is active LOW
#endif

//			memcmp(tmpstr,"+++");
			uart_write_bytes(ZULU_RADIO, "+++", 3);
			vTaskDelay(25 / portTICK_PERIOD_MS);	// yield to OS for 25 milliseconds
			uart_write_bytes(ZULU_RADIO, "?", 1);
			vTaskDelay(25 / portTICK_PERIOD_MS);	// yield to OS for 25 milliseconds
			get_uart_rx_msg(ZULU_RADIO, zulu_radio_rx_data, ZULU_RADIO_RX_BUFSIZE, 100);
			printf("Radio Config Data:\n%s\n\n",zulu_radio_rx_data);
			uart_write_bytes(ZULU_RADIO, "Q", 1);
			vTaskDelay(25 / portTICK_PERIOD_MS);	// yield to OS for 25 milliseconds

#ifdef ZULU_RADIO_MASTER			
		gpio_set_level(ZULU_RADIO_MASTER_RTS,0);	// RTS is active LOW
#endif
#ifdef ZULU_RADIO_SLAVE			
		gpio_set_level(ZULU_RADIO_SLAVE_RTS,0);		// RTS is active LOW
#endif
			}
#endif							
			break;

		case CMD_NVA:
			printf("Non-Volatile Memory - Factory Init Server Address Values\n");
			nvs_srvr_addr_port_var_init();
			set_srvr_addr_port_vars_from_nvs();
			break;
		
		case CMD_SSA:
#ifdef USE_M4G_MQTT
			j = server0_addr_ptr;
			k = server1_addr_ptr;
//			printf("%d: %s: %s:\n",num_params,paramstr[0],paramstr[1]);
			if (num_params == 2)
				{
				if (atoi(paramstr[0]) == 0)
					{
					if (0)
					{}
#ifdef SERVER_ADDRESS_AND_PORT_0
					else if (atoi(paramstr[1]) == 0)
						j = 0;

#endif				
#ifdef SERVER_ADDRESS_AND_PORT_1
					else if (atoi(paramstr[1]) == 1)
						j = 1;
#endif				
#ifdef SERVER_ADDRESS_AND_PORT_2
					else if (atoi(paramstr[1]) == 2)
						j = 2;

#endif				
#ifdef SERVER_ADDRESS_AND_PORT_3
					else if (atoi(paramstr[1]) == 3)
						j = 3;
					
#endif				
					}
				else if (atoi(paramstr[0]) == 1)
					{
					if (0)
					{}
#ifdef SERVER_ADDRESS_AND_PORT_0
					else if (atoi(paramstr[1]) == 0)
						k = 0;

#endif				
#ifdef SERVER_ADDRESS_AND_PORT_1
					else if (atoi(paramstr[1]) == 1)
						k = 1;

#endif				
#ifdef SERVER_ADDRESS_AND_PORT_2
					else if (atoi(paramstr[1]) == 2)
						k = 2;

#endif				
#ifdef SERVER_ADDRESS_AND_PORT_3
					else if (atoi(paramstr[1]) == 3)
						k = 3;
					
#endif				
					}				
					
				if ((server0_addr_ptr != j) || (server1_addr_ptr != k))
					{
					printf("Changing server pointers,,,\n");
					nvs_open("SRVR_ADDR_LIST",NVS_READWRITE, &nvs_handle);
					if (server0_addr_ptr != j)
						{
						nvs_set_u8(nvs_handle,"SA_ADDR_PTR_0",j);
						server0_addr_ptr = j;
						}
					if (server1_addr_ptr != k)
						{
						nvs_set_u8(nvs_handle,"SA_ADDR_PTR_1",k);
						server1_addr_ptr = k;
						}
					nvs_commit(nvs_handle);

					nvs_close(nvs_handle);
					}
				}
				
			show_server_addrs();
#endif
			break;

		case CMD_RST:
			esp_restart();
			break;
			
		case CMD_PWM:		// test lift PWM on GPIO 18
			break;
			
		case CMD_RLA:		// test lift relay on GPIO 33
		case CMD_RL1:		// test lift relay on GPIO 33
			break;

		case CMD_RL2:		// test lift relay on GPIO 33
			break;
			
		case CMD_MRM:
			break;

		case CMD_WFN:
			break;

		case CMD_SEV:
			show_events();

			break;

		case CMD_SET:
//	dbgprintf(chan,"%s, %02d/%02d/%02d ", days_of_week[day_of_week],days,month,year);
	//	dbgprintf(chan,"%02d:%02d:%02d", hrs, mins, secs);

// set test event for next minute, 1 minute duration:
			event_list[0].evt_mode_flag = EVENT_MODE_SCHEDULED;
			event_list[0].evt_day = 7;	// TEST "EVERY DAY" mode	//day_of_week;
			event_list[0].evt_t1_hrs = hrs+((mins+1)/60);
			event_list[0].evt_t1_mins = (mins+1);
			event_list[0].evt_t2_hrs = 0;
			event_list[0].evt_t2_mins = 1;
			event_list[0].evt_pwm = 25;
			event_list[0].evt_enable_flag = 1;
//			event_list[0].evt_running;

			printf("Test event set:\n\n");
			show_events();
			break;
			
		case CMD_GST:
			if (paramstr[0][0] == '1')
				get_server_time_flag = 1;
			else if (paramstr[0][0] == '0')
				get_server_time_flag = 0;
				
			printf("Get Server Time flag: %d  [ %02d:%02d ]\n",get_server_time_flag,get_server_time_timer/60,get_server_time_timer%60);
			break;

		case CMD_LFS:
			break;

		case CMD_LCT:
			
			break;

		case CMD_GPI:
			printf("GPIO settings:\n");
			if (paramstr[0][0] == '1')
			gpio_init(GPIO_REPORT+GPIO_CSV);
			else
				gpio_init(GPIO_REPORT);
			break;

		case CMD_DMP:
			{
			uint8_t *cptr;
			unsigned int *iptr;
			unsigned int len;
			char str[100];
// server cert
			printf("Srv:\n");
			iptr = (unsigned int *)0x3F14C000;
			cptr = (uint8_t *)0x3F14C002;
			printf("spr:\n");
			sprintf(str,"Server Cert start @ 0x%08lX, length %04X:",(unsigned long)cptr,*iptr);
//			sprintf(str,"Server Cert start @ 0x%08lX, length %04X:",(unsigned long)cptr,0);
			printf("%s\n",str);
			printf("dbg:\n");
			debug_hex_msg(cptr, 32, str);
			cptr = (uint8_t *)(0x3F14C874 - 32);
			sprintf(str,"Server Cert end @ 0x%06lX::",(unsigned long)cptr);
			debug_hex_msg(cptr, 32, str);

// client cert
			printf("Cli:\n");
			iptr = (unsigned int *)0x3F14D000;
			cptr = (uint8_t *)0x3F14D002;
			sprintf(str,"Client Cert start @ 0x%06lX, length %04X:",(unsigned long)cptr,*iptr);
			debug_hex_msg(cptr, 32, str);
			cptr = (uint8_t *)(0x3F14D874 - 32);
			sprintf(str,"Client Cert end @ 0x%06lX::",(unsigned long)cptr);
			debug_hex_msg(cptr, 32, str);
// client key
			printf("Key:\n");
			iptr = (unsigned int *)0x3F14E000;
			cptr = (uint8_t *)0x3F14E002;
			sprintf(str,"Client Key start @ 0x%06lX, length %04X:",(unsigned long)cptr,*iptr);
			debug_hex_msg(cptr, 32, str);
			cptr = (uint8_t *)(0x3F14CE74 - 32);
			sprintf(str,"Client Key end @ 0x%06lX::",(unsigned long)cptr);
			debug_hex_msg(cptr, 32, str);
// cert_name
			printf("CNm:\n");
			iptr = (unsigned int *)0x3F14F000;
			cptr = (uint8_t *)0x3F14F002;
			sprintf(str,"Cert Name start @ 0x%06lX, length %04X:",(unsigned long)cptr,*iptr);
			debug_hex_msg(cptr, 32, str);
			cptr = (uint8_t *)(0x3F14E30 - 32);
			sprintf(str,"Cert Name end @ 0x%06lX:",(unsigned long)cptr);
			debug_hex_msg(cptr, 32, str);
			}			
			break;

		case CMD_MRD:
			break;

		case CMD_MRS:
			break;

		case CMD_UVC:
			break;

		case CMD_UDP:
			if (!strcmp(paramstr[0],"ON"))
				{
				udp_disable_flag = 0;
				simcom_disable_flag = 0;
				}
			else if (!strcmp(paramstr[0],"OFF"))
				{
				udp_disable_flag = 1;
				if (mqtt_disable_flag)
					simcom_disable_flag = 1;
				}
				

			printf("UDP enable: ");
			if (udp_disable_flag)
				printf("OFF\n");
			else
				printf("ON\n");
							
			break;

		case CMD_LRM:

			break;

		case CMD_DLT:
			dbgflag = 1;
			dbgbits = dbgbits + DBG_4G_TXRX;
			dbgbits = dbgbits + DBG_MQTT;
			dbgbits = dbgbits + DBG_UDP;
		
			break;

		case CMD_RAS:
			break;
			
		case CMD_RAM:
			break;
			
		case CMD_RID:
			break;

		case CMD_RAL:
			break;

		case CMD_RCN:
			break;
			
		case CMD_RTS:
			break;

		case CMD_RTP:
			break;
			
		case CMD_NET:
			if (mqtt_transport_mode == MQTT_MODE_4G)
				net_test_flag = 1;
			break;
	
		case CMD_STM:
			i = 99;		// set impossible values
			j = 99;
			k = 00;
			
			if (num_params == 2)
				{
				j = atoi(paramstr[0]);	// hrs
				k = atoi(paramstr[1]);	// mins
				}
			else if (num_params == 3)
				{
				i = atoi(paramstr[0]);	// day
				j = atoi(paramstr[1]);	// hrs
				k = atoi(paramstr[2]);	// mins
				}

			if ((j < 24) && (k<60))
				{
				if (i<7)
					day_of_week = i;	
					
				hrs = j;
				mins = k;
					
				ntptime_sync_flag = 1;
				}
				
			show_time(DBG,2);
			printf("\n");
			
			break;

		case CMD_SED:
			if (num_params)								// if there are parameters
				{
				i = atoi(paramstr[0]);					// timer number

				if (num_params == 2)
					{
					evt_timer[i] = atoi(paramstr[1]);	// timer value
					}
				
				printf("Event timer %d: %5d\n",i,evt_timer[i]);
				}
				
			break;


		case CMD_WST:		// wifi sniffer - set wifi subtype to search for
			break;
	
		case CMD_SWS:
			if (num_params > 1)
				{
				esp_err_t err = 1;

//				printf("STR[%d]: %s\n",strlen(paramstr2all),paramstr2all);

// set SSID
				if((paramstr[0][0] == 'S') || (paramstr[0][0] == 's'))
					{
					x = atoi(paramstr[1]);
					if (x < WIFI_CREDENTIALS_MAX)
						{	
						if (strlen(paramstr3all) < 31)
							{
							printf("SSID: %s\n",paramstr3all);
							strcpy(wifi_sta_SSID[x],paramstr3all);	// paramstr2 is not forced to upper case...

							nvs_open("WIFI_MQT_VARS", NVS_READWRITE, &nvs_handle);
// wifi sta SSID string
							err = nvs_set_str(nvs_handle,"WIFI_MQ_STA_ID",wifi_sta_SSID[x]);
//							printf("rslt: %d\n",err);
							nvs_commit(nvs_handle);
							nvs_close(nvs_handle);
							}
						}

					}
		
// set Password
				if((paramstr[0][0] == 'P') || (paramstr[0][0] == 'p'))
					{
					x = atoi(paramstr[1]);
					if (x < WIFI_CREDENTIALS_MAX)
						{	
						if (strlen(paramstr2all) < 31)
							{	
							printf("PWD:  %s\n",paramstr3all);
							strcpy(wifi_sta_PWD[0],paramstr3all);		// paramstr2 is not forced to upper case...						

							nvs_open("WIFI_MQT_VARS", NVS_READWRITE, &nvs_handle);
// wifi sta PWD string
							err = nvs_set_str(nvs_handle,"WIFI_MQ_STA_PW",wifi_sta_PWD[x]);
//							printf("rslt: %d\n",err);
							nvs_commit(nvs_handle);
							nvs_close(nvs_handle);
							}
						}
					}
					
				if (err != 0)
					printf("Err: %d\n",err);
				}
					
			printf("STA SSID / STA PWD: \n");
//			printf("STA PWD:  %s\n",wifi_sta_PWD[0]);
			for (i=0;i<WIFI_CREDENTIALS_MAX;i++)
				{
				printf("%s   %s\n",wifi_sta_SSID[i],wifi_sta_PWD[i]);
				}
			break;

		case CMD_STR:
			if (num_params == 1)
				{
				if(paramstr[0][0] == '?')
					{
					printf("Mode names:\n");
					for (i=0;i<3;i++)
						{
						printf("%s\t%d\n",mqtt_transport_names[i],i);
						}
					printf("\n");
					}
				else			
					{						
					i = atoi(paramstr[0]);
					if (i<3)
						{
						esp_err_t err;
						
						mqtt_preferred_transport_mode = i;

						err = nvs_open("SRVR_ADDR_LIST",NVS_READWRITE, &nvs_handle);
						nvs_set_u8(nvs_handle,"SA_PREF_TRANSP",(uint8_t)mqtt_preferred_transport_mode);
						nvs_commit(nvs_handle);
						nvs_close(nvs_handle);	
						}
						
					}
				}
				
			printf("Preferred transport method: %s   current: %s\n",mqtt_transport_names[mqtt_preferred_transport_mode],mqtt_transport_names[mqtt_transport_mode]);
			
			break;

		case CMD_UPT:
			printf("Up-Time: %d days, %d hrs, %dmins, %d secs\n",u1day, u1hr,u1min,u1sec);

			break;

		case CMD_LPW:
#if defined(USE_LORA) || defined(USE_LORA_LINK)
// esp_err_t SX1276_read_reg(spi_device_handle_t spi, unsigned char reg, unsigned char *regval, unsigned char len)
			{
			unsigned char val,pa_boost,pa_max,pa_index,i,wr_flag;
			signed int pa_power;
			unsigned char pa_power_dp;
			
			esp_err_t err;
			
			err = SX1276_read_reg(spi, LREG_PA_CFG,&val,1);

			wr_flag = 0;
			
			if (num_params == 1)
				{
				if(paramstr[0][0] == '?')
					{
					printf("LPW B n  controls PA_boost bit   (n=0 or 1)\n");
					printf("LPW M i  sets the max PA power   (i=0 -  7)\n");
					printf("LPW P i  sets the PA power index (i=0 - 15)\n");

					}
				}
			
			if (num_params == 2)
				{
				if((paramstr[0][0] == 'B') || (paramstr[0][0] == 'b'))
					{
// set \ clear pa_boost bit	(b7)
					if(paramstr[1][0] == '1')
						{
						val = val | 0x80;
						wr_flag = 1;
						}
					else if(paramstr[1][0] == '0')
						{
						val = val & 0x7F;
						wr_flag = 1;
						}
					
					}
				if((paramstr[0][0] == 'M') || (paramstr[0][0] == 'm'))
					{
// 	max power bits (b6-b4)					
					i = atoi(paramstr[1]);
					
					if (i < 8)
						{
						val = (val & 0x8F) | (i<<4);
						wr_flag = 1;
						}
					}
				if((paramstr[0][0] == 'P') || (paramstr[0][0] == 'p'))
					{
// 	output power bits (b3-b0)					
					i = atoi(paramstr[1]);
					
					if (i < 16)
						{
						val = (val & 0xF0) | i;
						wr_flag = 1;
						}
					}
				}
			
			pa_boost = val>>7;
// following calcs use 10 * pa_max, pa_power values for 1 dp accuracy			
			if (pa_boost)
				pa_max = 170;
			else
				pa_max = 108 + 6*((val>>4) & 0x07);
			
			pa_index = val & 0x0F;
			pa_power = pa_max - 150 + (10*pa_index);
			
			pa_power_dp = abs(pa_power%10);

#define LORA_TX_POWER_LIMIT		14 // 14dB limit in EU
#define LORA_REGION_STR 		"EU"

#ifdef LORA_TX_POWER_LIMIT
			if (pa_power > (LORA_TX_POWER_LIMIT*10)) 
				printf("Tx power of %d.%ddBm is over %s legal limit!\n",pa_power/10,pa_power_dp,LORA_REGION_STR);
			else
#endif
				{
				if (wr_flag)
					{
					char tmpstr[16];

					err = SX1276_write_reg(spi, LREG_PA_CFG,&val,1);

					err = nvs_open("LORA_REG_LIST",NVS_READWRITE, &nvs_handle);
					sprintf(tmpstr,"LORA_REG_%02d",LREG_PA_CFG);
					nvs_set_u8(nvs_handle,tmpstr,val);
					nvs_commit(nvs_handle);
					nvs_close(nvs_handle);					
					}
				}

			printf("PA Reg: %02X  PA Boost:%d  PA_max:%d.%ddBm  PA index: %d  PA output power:%d.%ddBm\n",val, pa_boost,pa_max/10,pa_max%10,pa_index,pa_power/10,pa_power_dp);
			}
#endif
			break;

		case CMD_LRS:
#if defined(USE_LORA) || defined(USE_LORA_LINK)
			{
//			signed char rssi;		
//			rssi = SX1276_get_rssi(spi);		
//			printf("LoRa ave RSSI: %ddBm\n",rssi);

//			signed char lora_rssi, lora_pkt_rssi, lora_snr;
			if (lora_rssi == 0xFF)
				printf("LoRa RSSI not read yet\n");
			else
				printf("Lora SNR: %ddB  RSSI: %ddB  ave RSSI: %ddB\n",lora_snr,lora_rssi,lora_pkt_rssi);
			
			}
#endif			
			break;
		case CMD_LBW:
#if defined(USE_LORA) || defined(USE_LORA_LINK)
// 125kHz for LoRa (valid range 0 - 9) and coding rate (valid range 1 - 4)
			unsigned char val,bw_factor,cr_index,wr_flag;
			esp_err_t err;
			
			err = SX1276_read_reg(spi, LREG_MDM_CFG1,&val,1);

			wr_flag = 0;
			
			if (num_params == 1)
				{
				if(paramstr[0][0] == '?')
					{
					printf("LBW B n  selects bandwidth (0-9)\n");
					printf("LBW C i  selects coding rate (1-4)\n");

					}
				}
			
			if (num_params == 2)
				{
				if((paramstr[0][0] == 'B') || (paramstr[0][0] == 'b'))
					{
// bandwidth bits (b7-b4), values 0-9
					i = atoi(paramstr[1]);
					
					if (i < 10)
						{
						val = (val & 0x0F) | (i<<4);
						wr_flag = 1;
						}
					}
				if((paramstr[0][0] == 'C') || (paramstr[0][0] == 'c'))
					{
// coding rate bits (b3-b1), values 1-4
					i = atoi(paramstr[1]);
					
					if ((i>0) && (i < 5))
						{
						val = (val & 0xF1) | (i<<1);
						wr_flag = 1;
						}
					}
					
				if (wr_flag)
					{
					char tmpstr[16];

					err = SX1276_write_reg(spi, LREG_MDM_CFG1,&val,1);

					err = nvs_open("LORA_REG_LIST",NVS_READWRITE, &nvs_handle);
					sprintf(tmpstr,"LORA_REG_%02d",LREG_MDM_CFG1);
					nvs_set_u8(nvs_handle,tmpstr,val);
					nvs_commit(nvs_handle);
					nvs_close(nvs_handle);					
					}
				}


			bw_factor = (val>>4)&0x0F;
			cr_index =  (val>>1)&0x07;
			
			printf("LoRa bandwidth  : %d%02dkHz  [%d]\n",LoRa_bandwidth[bw_factor]/100,LoRa_bandwidth[bw_factor]%100,bw_factor);
			printf("LoRa coding rate: 4/%d  [%d]\n",4+cr_index,cr_index);
#endif			
			break;
				
		case CMD_LSF:
#if defined(USE_LORA) || defined(USE_LORA_LINK)
// up to 12 for LoRa ( range 6 - 12)
			{
			unsigned char val,spread_factor,wr_flag;
			esp_err_t err;
			
			err = SX1276_read_reg(spi, LREG_MDM_CFG2,&val,1);

			wr_flag = 0;

			if (num_params == 1)
				{
				if(paramstr[0][0] == '?')
					{
					printf("LSF S n  selects spreading factor (6-12)\n");

					}
				}
			
			if (num_params == 2)
				{
				if((paramstr[0][0] == 'S') || (paramstr[0][0] == 's'))
					{
// spreading factor (b7-b4), values 6-12
					i = atoi(paramstr[1]);
					
					if ((i>5) && (i < 13))
						{
						val = (val & 0x0F) | (i<<4);
						wr_flag = 1;
						}
					}
					
				if (wr_flag)
					{
					char tmpstr[16];

					err = SX1276_write_reg(spi, LREG_MDM_CFG2,&val,1);

					err = nvs_open("LORA_REG_LIST",NVS_READWRITE, &nvs_handle);
					sprintf(tmpstr,"LORA_REG_%02d",LREG_MDM_CFG2);
					nvs_set_u8(nvs_handle,tmpstr,val);
					nvs_commit(nvs_handle);
					nvs_close(nvs_handle);					
					}

				}



			spread_factor = (val>>4)&0x0F;
			
			printf("LoRa spread factor: %d\n",spread_factor);
			}
#endif			
			break;
	
		case CMD_WFC:
			{
			wifi_config_t cfg;
			
			esp_wifi_get_config(WIFI_IF_STA,&cfg);
			printf("WIFI CONFIG: STATION parameters:\n");
			printf("SSID: %s\tPASSWORD: %s\n",cfg.sta.ssid, cfg.sta.password);
			printf("AUTH; %s [%d]\tRSSI: %d\n",wifi_auth[cfg.sta.threshold.authmode],cfg.sta.threshold.authmode,cfg.sta.threshold.rssi);
			
			esp_wifi_get_config(WIFI_IF_AP,&cfg);
			printf("WIFI CONFIG: ACCESS POINT parameters:\n");
			printf("SSID: %s\tPASSWORD: %s\n",cfg.ap.ssid, cfg.ap.password);
			printf("AUTH; %s [%d]\n",wifi_auth[cfg.ap.authmode],cfg.ap.authmode);
			
			}
			break;

		case CMD_WFA:
			{
			unsigned char a,s;
			wifi_config_t cfg;
			esp_err_t err;
			
			if (paramstr[0][0] == '?')
				{
				printf("Wifi authentication methods are:\n");
				for (i=0;i<12;i++)
					{
					printf("%d %s\n",i,wifi_auth[i]);
					}
				printf("\n");
				}
			else
				{
				if (num_params == 2)
					{
					esp_wifi_get_config(WIFI_IF_STA,&cfg);
					s = cfg.sta.threshold.authmode;

					esp_wifi_get_config(WIFI_IF_AP,&cfg);
					a = cfg.ap.authmode;
				
					if((paramstr[0][0] == 'S') || (paramstr[0][0] == 's'))
						{
						i = atoi(paramstr[1]);
						if (i<12)
							{
							esp_wifi_get_config(WIFI_IF_STA,&cfg);
							cfg.sta.threshold.authmode = (wifi_auth_mode_t)i;
							err = esp_wifi_set_config(WIFI_IF_STA,&cfg);
							if (err != ESP_OK)
								printf("Err %d setting STA config!\n",err);
							else
								{
// save to NVS
								err = nvs_open("WIFI_MQT_VARS",NVS_READWRITE, &nvs_handle);
								nvs_set_u8(nvs_handle,"WIFI_STA_AUTH",i);
								nvs_commit(nvs_handle);
								nvs_close(nvs_handle);					
								
								}

// restart wifi to accept new parameters
							}

						}
					else if((paramstr[0][0] == 'A') || (paramstr[0][0] == 'a'))
						{
						i = atoi(paramstr[1]);
						if (i<12)
							{
							esp_wifi_get_config(WIFI_IF_AP,&cfg);
							cfg.ap.authmode = (wifi_auth_mode_t)i;
							err = esp_wifi_set_config(WIFI_IF_AP,&cfg);
							if (err != ESP_OK)
								printf("Err %d setting AP config!\n",err);
							else
								{
// save to NVS
								err = nvs_open("WIFI_MQT_VARS",NVS_READWRITE, &nvs_handle);
								nvs_set_u8(nvs_handle,"WIFI_AP_AUTH",i);
								nvs_commit(nvs_handle);
								nvs_close(nvs_handle);					
								
								}

// restart wifi to accept new parameters
							}
						}
					}
					
					esp_wifi_get_config(WIFI_IF_STA,&cfg);
					s = cfg.sta.threshold.authmode;

					esp_wifi_get_config(WIFI_IF_AP,&cfg);
					a = cfg.ap.authmode;
				
				printf("Wifi Authentication:\n");
				printf("    STA: %s [%d]\n",wifi_auth[s],s);
				printf("    AP:  %s [%d]\n",wifi_auth[a],a);
				}
			}
			break;
			
		case CMD_TST:
#ifdef USE_M4G_MQTT
			printf("CMQTT_rx_flag = %d\n",CMQTT_rx_flag);
#endif			
			printf("HP free %d\n",heap_caps_get_free_size(MALLOC_CAP_8BIT));
			printf("HP OK %d\n",heap_caps_check_integrity_all(1));
			
#ifdef USE_WIFI_MQTT
			{
// check if wifi connected...
			EventGroupHandle_t s_wifi_event_group = xEventGroupCreate();

			printf("Checking Wifi connection...\n");

			EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
				WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
				pdFALSE,
				pdFALSE,
				10 / portTICK_PERIOD_MS);	//portMAX_DELAY);

			printf("Result: ");
    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
			if (bits & WIFI_CONNECTED_BIT) 
				{
				ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
						 WIFI_STA_PREFIX, WIFI_STA_PASSWORD);
				} 
			else if (bits & WIFI_FAIL_BIT) 
				{
				ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
						 WIFI_STA_PREFIX, WIFI_STA_PASSWORD);
				} 
			else 
				{
				ESP_LOGE(TAG, "UNEXPECTED EVENT");
				j = 1<<16;
				for(i=0;i<16;i++)
					{

					if (bits & j)
						k = 1;
					else
						k = 0;	
					printf("%d ",k);
					j = j>>1;
					
					}
				printf("\n");
				}
			}
#endif

#ifdef USE_WIFI_UDP
	printf("wifi udp DNS    : %d\n",wifi_udp_DNSfound);
	printf("wifi udp connect: %d\n",wifi_udp_connected);
#endif

			{
			char url_str[80];
			unsigned int port;

			get_url_port("Test/test:1000", url_str, &port);
			printf("%s %d\n",url_str,port);

			get_url_port("Test/2", url_str, &port);
			printf("%s %d\n",url_str,port);
			}
			break;

		default:
			break;
		}
	}
		
return error;
}

void status(unsigned char chan, const char* devicestr, unsigned char device)
{
char str[10], spstr[12];
unsigned char i,j;

if (status_array[device])
	strcpy(str,"ON ");
else
	strcpy(str,"OFF");
	
dbgprintf(chan,"%s",devicestr);
j = 9 - strlen(devicestr);
spstr[0] = 0x00;

for (i=0;i<j;i++)
	{
	spstr[i] = ' ';
	}
spstr[i] = 0x00;
	
dbgprintf(chan,"%s: %s ",spstr,str);
}

unsigned char skip_whitespace(unsigned char* string, unsigned char* p)
{
unsigned char result = 0;
char c;

c = string[*p];	

//printf("SW:<%s> %d ",string,*p);

while (((c == ' ') || (c == '\t')) && (c != 0x00) && (c != 0x0D) && (c != 0x0A))
	{
	(*p)++;
	c = string[*p];	
	}

//printf(" %d\r\n",*p);
if ((c == 0x00) || (c == 0x0D) || (c == 0x0A))
	result = 1;
	
return result;
}

unsigned char find_whitespace(unsigned char* string, unsigned char* p)
{
unsigned char result = 0;
char c;

//printf("FW:<%s> %d ",string,*p);

c = string[*p];	

while((c != ' ') && (c != 0x00) && (c != 0x0D) && (c != 0x0A))
	{
	(*p)++;
	c = string[*p];	
	}

//printf(" %d\r\n",*p);

return result;
}

unsigned int get_param(unsigned char* string, unsigned char* p, char* prmstr, unsigned char uc_flag)
{
unsigned int result = 0;
unsigned char i = 0;
char c;

//printf("GP:<%s> %d ",string,*p);

c = string[*p];	

while((c != ' ') && (c != 0x00) && (c != 0x0D) && (c != 0x0A))
	{
	prmstr[i++] = c;
	(*p)++;
	c = string[*p];		
	}

//printf(" %d\r\n",*p);
	
prmstr[i] = 0x00;

if (c != ' ')	// ie string has ended with 0x00, 0x0D or 0x0A
	result = 1;

if (i==0)	
	result = result | 0x80;

if (uc_flag)
	toupperstr(prmstr);
	
if (debug_do(DBG_CLI))
	dbgprintf(DBG,"PRM=%s  ",prmstr);
	
return result;
}


//int dbgprintf(DBG,FILE* dbgfile, const char * restrict format, ...)
#ifdef __cplusplus
int dbgprintf(unsigned char dbgfile, const char * format, ...)
#else
int dbgprintf(unsigned char dbgfile, const char *restrict format, ...)
#endif
{
char str[90];

#if 1
va_list ap;

va_start(ap,format);

if (dbgfile!=DBG)
	{


//vfprintf(dbgfile, format,ap);

	vsprintf(str, format,ap);
	uart_write_bytes((uart_port_t)dbgfile, str, strlen(str));

//vprintf(format,ap);


//if(testmode & 0x80)							// debug window is open
//	vprintf(format,ap);

//if ((dbgflag) && (dbgfile != NULL))			// dbgflag is a global....
//	vfprintf(dbgfile, format,ap);


	}
else
	{
	vprintf(format,ap);
	}

va_end(ap);
	
#endif
return 0;
}

unsigned char debug_do(unsigned int mask)
{
if ((dbgflag)  && (dbgbits & mask))
	return 1;
else
	return 0;
}

void toupperstr(char* str)
{
unsigned char i = 0;

while (str[i] != 0x00)
	{
	str[i] = toupper(str[i]);
	i++;
	}
}

void mac_to_str(uint8_t* mac_addr, char separator, char* mac_str)
{
unsigned char i;
char str[10];

mac_str[0] = 0x00;

for (i=0;i<6;i++)
	{
	sprintf(str,"%02X",mac_addr[i]);
	strcat(mac_str,str);
	if ((separator != 0) && (i!=5))
		{
		sprintf(str,"%c",separator);		
		strcat(mac_str,str);
		}
	}
//sprintf(str,"%02X",mac_addr[5]);
//strcat(mac_str,str);

}

void str_to_mac(uint8_t* mac_str, char separator, char* mac_addr)
{
unsigned char i,j;

if (separator != 0)
	j = 3;
else
	j = 2;
	
for (i=0;i<6;i++)
	{
	mac_addr[i] = asc2hex(&mac_str[j*i]);
	}
}

// compile .bat file "touches" this source file so __DATE__, __TIME__, always recompiled
void show_inf(unsigned char simcom_info_flag)
{
uint8_t mac_addr[6];
char mac_str[20];
//unsigned char pcb_ver;
unsigned char pcb_ver_str[15];
uint8_t* bt_mac_str;
/*
//#ifdef V1_PCB
pcb_ver = 1;
#endif
#ifdef V2_PCB
pcb_ver = 2;
#endif
*/

printf("*********************************\r\n");
printf("* MAC_Rpt - Ver %d.%d.%d\r\n",VER_MAJ,VER_MIN,VER_SUB);
//printf("* For PCB Mk%d",pcb_ver);
printf("* For PCB %s",PCB_VER_STR);
printf("\r\n");
#ifdef SIMCOM_PROGRAMMING_MODE
printf("* DIAGNOSTIC CODE!\r\n");
#endif
printf("* Built: %s   %s\r\n",__DATE__, __TIME__);
//printf("* Built: %s   %s\r\n",PROG_DATE, PROG_TIME);
printf("*********************************\r\n\r\n");

dbgprintf(DBG,"IDF version: %s", esp_get_idf_version());

// ESP chip information
esp_chip_info_t chip_info;
esp_chip_info(&chip_info);
printf("ESP32 with %d CPU cores, WiFi%s%s, ",
		chip_info.cores,
		(chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
		(chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

printf("silicon rev %d, ", chip_info.revision);

printf("%dMB %s flash\r\n", spi_flash_get_chip_size() / (1024 * 1024),
	(chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");


esp_efuse_mac_get_default(mac_addr);
mac_to_str(mac_addr, ':',mac_str);
printf("EFUSE     MAC address is %s\r\n",mac_str);
	
esp_read_mac(mac_addr,ESP_MAC_WIFI_SOFTAP);
mac_to_str(mac_addr, ':',mac_str);
printf("Wifi      MAC address is %s\r\n\r\n",mac_str);
//printf("Unique ID  is %s\r\n\r\n",lora_radio_ID_str);

#ifdef USE_BLUETOOTH
//bt_mac_str = (uint8_t*)esp_bt_dev_get_address();
mac_addr[5]++;
mac_to_str(mac_addr, ':',mac_str);
printf("Bluetooth MAC address is %s\r\n\r\n",mac_str);
#endif

printf("Wifi STA SSID     = %s\r\n",wifi_config_sta.sta.ssid);
printf("Wifi STA password = %s\r\n",wifi_config_sta.sta.password);

printf("Wifi AP  SSID     = %s\r\n",wifi_config_ap.sta.ssid);
printf("Wifi AP  password = %s\r\n",wifi_config_ap.sta.password);

dbgprintf(DBG,"\r\n");

if(simcom_info_flag)
	{
	dbgprintf(DBG,"SIMCOM MODULE  : %s\r\n",modtype_str);
	dbgprintf(DBG,"SIMCOM FW   ver: %s\r\n",fwver_str);
	dbgprintf(DBG,"SIMCOM IMEI num: %s\r\n",imei_str);
	dbgprintf(DBG,"SIMCOM SIM  num: %s\r\n",simnum_str);
	}
}

signed int get_uart_rx_msg(unsigned char uart, unsigned char* data, unsigned int datalen, unsigned char timeout)
{	
unsigned char i,j;
int rx_length;
char c;

printf("GURM\n");
	
	data[0] = 0x00;
#ifdef USE_M4G_MQTT
	if (uart == FOUR_G)
		{
		rx_length = 0;
		j = pdTRUE;
		c = 0x00;
		while ((rx_length < datalen) && (c != 0x0A) && (j != pdFALSE))
			{
			j = xQueueReceive(mqtt_cmd_queue, &c, timeout / portTICK_PERIOD_MS);
			if (j == pdTRUE)
				{
				data[rx_length] = c;
				rx_length++;
				}
			}
		}
	else
#endif
		rx_length = uart_read_bytes((uart_port_t)uart, data, datalen, timeout / portTICK_PERIOD_MS);
		
//	fgets((char*)lora_radio_rx_data,DBG_RCV_BUF_SIZE,RADIO);
//	if (strlen((char*)lora_radio_rx_data))
/*
	if (debug_do(DBG_LORA_RADIO_TXRX))
		{
		printf("RADIO: [%d] ",rx_length);

		for (i=0;i<rx_length;i++)
			{
			printf("%02X ",data[i]);
			}
		printf("    ");
			
		for (i=0;i<rx_length;i++)
			{
			c = data[i];
			if ((c < 0x20) || (c > 0x7F))				
				c = '.';
			printf("%c ",c);
			}
		printf("\r\n");
		}
*/
// terminate the string...
	data[rx_length] = 0x00;

return rx_length;
}

void show_status(unsigned char chan)
{
unsigned char i;

//			dbgprintf(chan,"Status:\t\t[%dd %02d:%02d:%02d]\r\n", days, hrs, mins, secs);
//			dbgprintf(DBG,"[%d  %02d:%02d:%02d]Sending radio msg...\r\n", days, hrs, mins, secs);

			dbgprintf(chan,"******************************\n");
			dbgprintf(chan,"* Status:\t");
			show_time(chan,0);
			dbgprintf(chan,"  *\r\n");
			dbgprintf(chan,"******************************\n");

//			dbgprintf(chan,"4G State:  [%03d] <%s>\r\n",four_g_state,four_g_state_str[four_g_state]);
// transport status:		
			dbgprintf(chan,"MQTT transport: preferred: %s  current: %s\n",mqtt_transport_names[mqtt_preferred_transport_mode],mqtt_transport_names[mqtt_transport_mode]);
/*
			if (mqtt_transport_mode == MQTT_MODE_4G)
				dbgprintf(chan,"4G\n");
			if (mqtt_transport_mode == MQTT_MODE_WIFI)
				dbgprintf(chan,"WIFI\n");
			if (mqtt_transport_mode == MQTT_MODE_NONE)
				dbgprintf(chan,"NONE\n");
*/				
			dbgprintf(chan,"\n");
			
		
#ifdef USE_M4G
// SIMCOM module status:		
			if (simcom_state == SIMCOM_POWER_OFF)
				dbgprintf(chan,"SIMCOM powered OFF\n");
			else if (simcom_state < SIMCOM_GET_MODULETYPE)
				dbgprintf(chan,"SIMCOM powering UP\n");
			else if (simcom_state < SIMCOM_READY_IDLE)
				dbgprintf(chan,"performing SIMCOM checks\n");
			else if (simcom_state == SIMCOM_READY_IDLE)
				{
				dbgprintf(chan,"SIMCOM ready\n");

				if (mqtt_state < MQTT_SET_SSL_CONFIG)
					dbgprintf(chan,"MQTT initialising connection parameters\n");
				else if (mqtt_state < MQTT_COMM_START)
					dbgprintf(chan,"MQTT setting up SSL encryption\n");
				else if (mqtt_state < MQTT_CONNECT)
					dbgprintf(chan,"MQTT starting connection\n");
				else if (mqtt_state < MQTT_CONNECT_IDLE)
					dbgprintf(chan,"MQTT connected - sending payload\n");
				else if (mqtt_state == MQTT_CONNECT_IDLE)
					dbgprintf(chan,"SIMCOM connected - idle\n");
				else if (mqtt_state >  MQTT_COMM_IDLE)
					dbgprintf(chan,"MQTT disconnecting\n");
				}
				
			else if (simcom_state < SIMCOM_MQTT_IDLE_CHECK_SIG_QUALITY)
				dbgprintf(chan,"SIMCOM powering DOWN\n");
			else if (simcom_state == SIMCOM_MQTT_IDLE_CHECK_SIG_QUALITY)
				dbgprintf(chan,"SIMCOM checking network signal strength\n");
			else if (simcom_state == SIMCOM_NONE_FOUND)
				dbgprintf(chan,"SIMCOM - no SIMCOM module found\n");
			else
				dbgprintf(chan,"SIMCOM in error recovery\n");

// 4G status:		
			dbgprintf(chan,"4G signal strength: ");
			if (cell_rssi == -127)
				dbgprintf(chan,"Not connected to network yet...\n");
			else
				dbgprintf(chan,"%ddBm\n",cell_rssi);


//			dbgprintf(chan,"MQTT server login state: %d\n",mqtt_login_state);
			print_simcom_state(chan);
			print_mqtt_state(chan);

#ifdef USE_M4G_UDP
			print_udp_state(chan);
#endif
// SIMCOM Rx channel status:		
			dbgprintf(chan,"\n");
			dbgprintf(chan,"simcom resp flag: %d     eol flag: %d\n",simcom_response_flag,simcom_eol_response_flag);
			dbgprintf(chan,"simcom retry attempts: %d   simcom err attempts: %d  simcom pwrdwn  attempts: %d\n",simcom_attempts,simcom_error_attempts,simcom_powerdown_attempts);
			dbgprintf(chan,"T/O timer: %d   last T/O set: %d\n",four_g_timeout_timer, four_g_timeout_val);
			dbgprintf(chan,"mqtt   resp flag: %d     eol flag: %d\n",mqtt_response_flag,mqtt_eol_response_flag);
#ifdef USE_M4G_UDP
			dbgprintf(chan,"udp    resp flag: %d     eol flag: %d\n",udp_response_flag,udp_eol_response_flag);
#endif
//			dbgprintf(chan,"MQTT   retry attempts: %d   MQTT   err attempts: %d   T/O timer: %d   last T/O set: %d\n\n",mqtt_attempts,mqtt_error_attempts,four_g_timeout_timer, four_g_timeout_val);
//			dbgprintf(chan,"udp    retry attempts: %d   udp    err attempts: %d   T/O timer: %d   last T/O set: %d\n\n",udp_attempts,udp_error_attempts,four_g_timeout_timer, four_g_timeout_val);
#endif

#ifdef USE_WIFI_MQTT
			dbgprintf(chan,"\n");
			print_wifi_mqtt_state(chan);
#endif			

#ifdef USE_WIFI_UDP
			dbgprintf(chan,"\n");
			print_wifi_udp_state(chan);
#endif			

// Server login status:		
			dbgprintf(chan,"\n");
			print_mqtt_login_state(chan);

// queue enable flag
//			dbgprintf(chan,"\n");
//			dbgprintf(chan,"MQTT Queues Enabled: %d\n",queues_enabled_flag);
			
// GPIO \ device states
			dbgprintf(chan,"\n");
			for (i=0;i<NUM_DEVS;i++)
				{
				dbgprintf(chan,"%d ",status_array[i]);
				}
			dbgprintf(chan,"\r\n");

// board status				
//			dbgprintf(chan,"Supply Voltage: %d.%dV [%d]",supply_voltage/10,supply_voltage%10,voltage);
			dbgprintf(chan,"Supply Voltage: %d.%dV [%d]  [%d]",supply_voltage/10,supply_voltage%10,voltage,adc_val_raw_sv);
			dbgprintf(chan,"              Temperature: %dC\r\n",(signed char)(temperature-128));
			dbgprintf(chan,"Tank Sensors: ");
			if (!tank_sensors_present_flag)
				dbgprintf(chan,"NOT ");

			dbgprintf(chan,"present:   %d [%d%%]  %d [%d%%]\n",tank_sensor_0,tank_sensor_percent_0,tank_sensor_1,tank_sensor_percent_1);

				
			dbgprintf(chan,"autobaud: %d    bitrate: %db/s    auto: %ldb/s    binary: %d    auto: %d\n",serial_autobaud_flag,serial_bitrate,auto_serial_bitrate,binary_data_flag, auto_binary_data_flag);

/*			
			status(chan,"TiltSw", DEV_TILT_SW);
			status(chan,"FillSw", DEV_FILL_SW);
			status(chan,"PurgeSw", DEV_PURGE_SW);
			status(chan,"IR_LED", DEV_IR_LED);
			status(chan,"IR_DET", DEV_IR_DETECT);
			dbgprintf(chan,"\r\n");
			
			status(chan,"TankSens1", DEV_TANK_SENSOR_1);
			status(chan,"TankSens2", DEV_TANK_SENSOR_2);
			status(chan,"TankSens3", DEV_TANK_SENSOR_3);
			status(chan,"TankSens4", DEV_TANK_SENSOR_4);
			dbgprintf(chan,"TankStat : %d %s\r\n", tank_status,tank_statstr[tank_status]);
			status(chan,"DispFlag", DEV_DISP_MODE);
			status(chan,"PumpMotor", DEV_PUMP_MOTOR);
			status(chan,"FillMotor", DEV_FILL_MOTOR);
			status(chan,"VentSol", DEV_VENT_SOLENOID);
			dbgprintf(chan,"\r\n");

			status(chan,"BlueLED", DEV_BLUE_LED);
			status(chan,"RedLED", DEV_RED_LED);
			dbgprintf(chan,"\r\n");

			dbgprintf(chan,"Dispenses: %5d ",num_dispense_outputs);
			if (tank_countdown_flag)
				dbgprintf(chan,"[%3d]\r\n",tank_countdown);
			else
				dbgprintf(chan,"[---]\r\n");

			dbgprintf(chan,"PumpTime : %d.%d ",pump_time/10,pump_time%10);
			dbgprintf(chan,"DeadTime : %d.%d ",dead_time/10,dead_time%10);
			dbgprintf(chan,"\r\n");
			
			dbgprintf(chan,"Motor Current: %5dmA        ",instantaneous_pump_motor_current);
			dbgprintf(chan,"Ave Motor Current: %5dmA  ",pump_motor_ave_current);
			dbgprintf(chan,"\r\n\r\n");

			if (debug_do(DBG_MOTOR_CURRENT))
				{
				for (i=0;i<PUMP_MOTOR_SAMPLES;i++)
					{
					dbgprintf(chan,"%d  ",pump_motor_current[i]);
					}
				dbgprintf(chan,"\r\n\r\n");
				}
*/
			dbgprintf(chan,"\r\n");
}

void show_time(unsigned char chan, unsigned char msec_flag)
{
dbgprintf(chan,"[");
	
if (msec_flag & 2)
	dbgprintf(chan,"%s, %02d/%02d/%02d ", days_of_week[day_of_week],days,month,year);
	
dbgprintf(chan,"%02d:%02d:%02d", hrs, mins, secs);

if (msec_flag & 1)
	dbgprintf(chan,".%01d00", x100msec);
	
if (!ntptime_sync_flag)
	dbgprintf(chan,"*]");
else
	dbgprintf(chan,"]");
}

void init_tm_data(unsigned char init_flag)
{
unsigned char i;

// initialise storage for user test message
	for (i=0;i<16;i++)
		{
		tm_data[i] = 0;
		}

if (init_flag)
	{
	tm_data[ 0] = 11;		// status msg len
	tm_data[ 1] = 0;		// status msg
	tm_data[ 2] = 1;		// tank status
	tm_data[ 3] = 2;		// dispense count hi
	tm_data[ 4] = 3;		// dispense count lo
	tm_data[ 5] = 0;		// last 100
	tm_data[ 6] = 35;		// temperature
	tm_data[ 7] = 125;		// battery
	tm_data[ 8] = 77;		// motor current
	tm_data[ 9] = 0;		// error
	tm_data[10] = 2;		// MAC devs hi
	tm_data[11] = 4;		// MAC devs lo
	tm_data[12] = 0;		// spare
	}

}	

void show_options(void)
{
unsigned char i = 0;
printf("#define Options:\r\n");

#ifdef USING_PROJECT_SETTINGS
printf("USE PROJECT SETTINGS: %s\n",PROJECT_STRING);
i++;
#endif

#ifdef USE_CPU_SLEEP
printf("USE CPU SLEEP\n");
i++;
#endif

#ifdef SHOW_SLEEP_MSGS
printf("SHOW SLEEP MSGS\n");
i++;
#endif

#ifdef USE_HTTP
printf("USE HTTP\n");
i++;
#endif

#ifdef USE_M4G_MQTT
printf("USE M4G MQTT\n");
i++;
#endif

#ifdef USE_WIFI_MQTT
printf("USE WIFI MQTT\n");
i++;
#endif

#ifdef WIFI_SCAN
printf("WIFI SCAN\n");
i++;
#endif

#ifdef WIFI_HIDE_SSID
printf("WIFI SSID HIDDEN\n");
i++;
#endif

#ifdef BT_SNIFFER
printf("BT SNIFFER\n");
i++;
#endif

#ifdef USE_BLUETOOTH
printf("USE BLUETOOTH\n");
i++;
#endif

#ifdef BLUETOOTH_SCAN
printf("BLUETOOTH SCAN\n");
i++;
#endif

#ifdef USE_BLUETOOTH_CLASSIC_DETECT
printf("BLUETOOTH CLASSIC DETECT\n");
i++;
#endif

#ifdef USE_BLUETOOTH_ADVERTISING
printf("USE BLUETOOTH ADVERTISING\n");
i++;
#endif

#ifdef INHIBIT_BT_SENSOR_DATA
printf("INHIBIT BT SENSOR DATA\n");
i++;
#endif

#ifdef USE_LORA
printf("USE LORA\n");
i++;
#endif

#ifdef LORAWAN_SCAN
printf("LORAWAN SCAN\n");
i++;
#endif

#ifdef USE_TANK_SENSORS
printf("USE TANK SENSORS\n");
i++;
#endif

#ifdef TANKSENSOR_1_ON_GPIO39
printf("TANK SENSOR2 ON GPIO39\n");
i++;
#endif

#ifdef USE_I2C
printf("USE I2C\n");
i++;
#endif

#ifdef USE_I2C_AMBIMATE4
printf("USE I2C AMBIMATE4\n");
i++;
#endif

#ifdef USE_I2C_PIMORONI
printf("USE I2C PIMORONI\n");
i++;
#endif

#ifdef USE_ZULU_RADIO
printf("USE ZULU RADIO\n");
i++;
#endif

#ifdef ZULU_RADIO_MASTER
printf("ZULU RADIO MASTER\n");
i++;
#endif

#ifdef ZULU_RADIO_SLAVE
printf("ZULU RADIO SLAVE\n");
i++;
#endif

#ifdef USE_WEBSERVER
printf("WEB SERVER\n");
i++;
#endif

#ifdef USE_LORA_LINK
printf("LORA LINK\n");
i++;
#endif

#ifdef USE_ADCS
printf("ADCs\n");
i++;
#endif


if (i== 0)
	printf("No Options defined\n");
		
printf("\n");
}

void show_events(void)
{
unsigned char i;
	
	printf("Timer Events:\n");
	printf("Evt\tEnable\tMode\tDay\tT1 Time\tDuration PWM\tRun\tRpt\tDur\n");
	for (i=0;i<NUM_EVENTS_MAX;i++)
		{
		printf("%2d\t",i);
		if(event_list[i].evt_enable_flag)
			printf("  Y\t");
		else
			printf("  N\t");

		if(event_list[i].evt_mode_flag == EVENT_MODE_SCHEDULED)
			{
			printf("Sch\t");
			if (event_list[i].evt_day < 7)
				printf("%s\t",days_of_week[event_list[i].evt_day]);
			else
				printf("DAILY\t");
			}

		else if(event_list[i].evt_mode_flag == EVENT_MODE_REPEAT)
			{
			printf("Rep\t");
			printf("---\t");
			}

		else 
			{
			printf(" - \t");
			printf("---\t");
			}
					
//				printf("  %c\t",event_list[i].evt_mode_flag);
		printf("%02d.",event_list[i].evt_t1_hrs);
		printf("%02d\t",event_list[i].evt_t1_mins);
		
		if (event_list[i].evt_t2_hrs != 24)
			{
			printf("%02d.",event_list[i].evt_t2_hrs);	
			printf("%02d\t",event_list[i].evt_t2_mins);
			}
		else
			printf("CONT\t");
				
		printf("%3d%%\t",event_list[i].evt_pwm);
//				printf("% d\n",event_list[i].evt_running);
		if(event_list[i].evt_running)
			printf(" Y");
		else
			printf(" N");
		printf("\t%04X\t",evt_repeat_timer[i]);
		printf("%04X\t",evt_timer[i]);
		printf("\n");
		}
				
	printf("\nTime of Day:	%d [%s]  %02d:%02d\n",day_of_week,days_of_week[day_of_week],hrs,mins);
}

void get_event_line(unsigned char evt_num, char* str, EVENT_INFO event_list[NUM_EVENTS_MAX])
{
	char tmp[20];
	
		sprintf(str,"%2d\t",evt_num);
		if(event_list[evt_num].evt_enable_flag)
			strcat(str,"  Y\t");
		else
			strcat(str,"  N\t");

		if(event_list[evt_num].evt_mode_flag == EVENT_MODE_SCHEDULED)
			{
			strcat(str,"Sch\t");
			if (event_list[evt_num].evt_day < 7)
				sprintf(tmp,"%s\t",days_of_week[event_list[evt_num].evt_day]);
			else
				sprintf(tmp,"DAILY\t");
			
			strcat(str,tmp);
			}

		else if(event_list[evt_num].evt_mode_flag == EVENT_MODE_REPEAT)
			{
			strcat(str,"Rep\t");
			strcat(str,"---\t");
			}

		else 
			{
			strcat(str," - \t");
			strcat(str,"---\t");
			}
					
//				printf("  %c\t",event_list[i].evt_mode_flag);
		sprintf(tmp,"%02d.%02d\t",event_list[evt_num].evt_t1_hrs,event_list[evt_num].evt_t1_mins);
		strcat(str,tmp);
		
		if (event_list[evt_num].evt_t2_hrs != 24)
			{
			sprintf(tmp,"%02d.%02d\t",event_list[evt_num].evt_t2_hrs,event_list[evt_num].evt_t2_mins);	
			strcat(str,tmp);
			}
		else
			strcat(str,"CONT\t");
				
		sprintf(tmp,"%3d%%\t",event_list[evt_num].evt_pwm);
		strcat(str,tmp);
//				printf("% d\n",event_list[i].evt_running);
		if(event_list[evt_num].evt_running)
			strcat(str," Y");
		else
			strcat(str," N");

		sprintf(tmp,"\t%04X\t%04X\n",evt_repeat_timer[evt_num],evt_timer[evt_num]);
		strcat(str,tmp);
}

void get_event_line_html(unsigned char evt_num, char* str, EVENT_INFO event_list[NUM_EVENTS_MAX])
{
	char tmp[40];
		sprintf(str,"<ev%d>%2d</ev%d>",evt_num,evt_num,evt_num);

		if(event_list[evt_num].evt_enable_flag)
			sprintf(tmp,"<en%d>Y</en%d>",evt_num,evt_num);
		else
			sprintf(tmp,"<en%d>N</en%d>",evt_num,evt_num);
		strcat(str,tmp);
		
		if(event_list[evt_num].evt_mode_flag == EVENT_MODE_SCHEDULED)
			{
			sprintf(tmp,"<md%d>Sch</md%d>",evt_num,evt_num);
			strcat(str,tmp);
			
			if (event_list[evt_num].evt_day < 7)
				sprintf(tmp,"<da%d>%s</da%d>",evt_num,days_of_week[event_list[evt_num].evt_day],evt_num);
			else
				sprintf(tmp,"<da%d>DAILY</da%d>",evt_num,evt_num);
			
			strcat(str,tmp);
			}

		else if(event_list[evt_num].evt_mode_flag == EVENT_MODE_REPEAT)
			{
			sprintf(tmp,"<md%d>Rep</md%d>",evt_num,evt_num);
			strcat(str,tmp);
			sprintf(tmp,"<da%d>---</da%d>",evt_num,evt_num);
			strcat(str,tmp);
			}

		else 
			{
			sprintf(tmp,"<md%d> - </md%d>",evt_num,evt_num);
			strcat(str,tmp);
			sprintf(tmp,"<da%d>---</da%d>",evt_num,evt_num);
			strcat(str,tmp);
			}
					
//				printf("  %c\t",event_list[i].evt_mode_flag);
		sprintf(tmp,"<st%d>%02d.%02d</st%d>",evt_num,event_list[evt_num].evt_t1_hrs,event_list[evt_num].evt_t1_mins,evt_num);
		strcat(str,tmp);
//		sprintf(tmp,"<td>%02d</td>",event_list[evt_num].evt_t1_mins);
//		strcat(str,tmp);
		
		if (event_list[evt_num].evt_t2_hrs != 24)
			{
			sprintf(tmp,"<du%d>%02d.%02d</du%d>",evt_num,event_list[evt_num].evt_t2_hrs,event_list[evt_num].evt_t2_mins,evt_num);	
			strcat(str,tmp);
//			sprintf(tmp,"<td>%02d</td>",event_list[evt_num].evt_t2_mins);	
//			strcat(str,tmp);
			}
		else
			{
			sprintf(tmp,"<du%d>CONT</du%d>",evt_num,evt_num);
			strcat(str,tmp);
			}
				
		sprintf(tmp,"<pw%d>%3d%%</pw%d>",evt_num,event_list[evt_num].evt_pwm,evt_num);
		strcat(str,tmp);
		
//				printf("% d\n",event_list[i].evt_running);
		if(event_list[evt_num].evt_running)
			sprintf(tmp,"<ru%d>Y</ru%d>",evt_num,evt_num);
		else
			sprintf(tmp,"<ru%d>N</ru%d>",evt_num,evt_num);
		strcat(str,tmp);

		sprintf(tmp,"<rp%d>%04X</rp%d>",evt_num,evt_repeat_timer[evt_num],evt_num);
		strcat(str,tmp);

		sprintf(tmp,"<dr%d>%04X</dr%d>",evt_num,evt_timer[evt_num],evt_num);
		strcat(str,tmp);
//		sprintf(tmp,"<td>%04X<td>",evt_timer[evt_num]);
//		strcat(str,tmp);
	
#if 0
		sprintf(str,"<td>%2d</td>",evt_num);
		if(event_list[evt_num].evt_enable_flag)
			strcat(str,"<td>Y</td>");
		else
			strcat(str,"<td>N</td>");

		if(event_list[evt_num].evt_mode_flag == EVENT_MODE_SCHEDULED)
			{
			strcat(str,"<td>Sch</td>");
			if (event_list[evt_num].evt_day < 7)
				sprintf(tmp,"<td>%s</td>",days_of_week[event_list[evt_num].evt_day]);
			else
				sprintf(tmp,"<td>DAILY</td>");
			
			strcat(str,tmp);
			}

		else if(event_list[evt_num].evt_mode_flag == EVENT_MODE_REPEAT)
			{
			strcat(str,"<td>Rep</td>");
			strcat(str,"<td>---</td>");
			}

		else 
			{
			strcat(str,"<td> - </td>");
			strcat(str,"<td>---</td>");
			}
					
//				printf("  %c\t",event_list[i].evt_mode_flag);
		sprintf(tmp,"<td>%02d.</td>",event_list[evt_num].evt_t1_hrs);
		strcat(str,tmp);
		sprintf(tmp,"<td>%02d</td>",event_list[evt_num].evt_t1_mins);
		strcat(str,tmp);
		
		if (event_list[evt_num].evt_t2_hrs != 24)
			{
			sprintf(tmp,"<td>%02d.</td>",event_list[evt_num].evt_t2_hrs);	
			strcat(str,tmp);
			sprintf(tmp,"<td>%02d</td>",event_list[evt_num].evt_t2_mins);	
			strcat(str,tmp);
			}
		else
			strcat(str,"<td>CONT</td>");
				
		sprintf(tmp,"<td>%3d%%</td>",event_list[evt_num].evt_pwm);
		strcat(str,tmp);
//				printf("% d\n",event_list[i].evt_running);
		if(event_list[evt_num].evt_running)
			strcat(str,"<td>Y</td>");
		else
			strcat(str,"<td>N</td>");

		sprintf(tmp,"<td>%04X<td>",evt_repeat_timer[evt_num]);
		strcat(str,tmp);
		sprintf(tmp,"<td>%04X<td>",evt_timer[evt_num]);
		strcat(str,tmp);
#endif
}
	
unsigned char debug_hex_msg(uint8_t *msg_data, unsigned int msg_length, char *ID_str)
{
unsigned char c,i;
unsigned int j = 0;


	printf("%s [%d]\r\n",ID_str,msg_length);
	for (j=0;j<msg_length;j++)
		{
		if ((msg_data[j] > 0x1F) && (msg_data[j] < 0x80))
			c = msg_data[j];
		else
			c = '.';
		printf("%c  ",c);
		
		}
	printf("\r\n");

	for (j=0;j<msg_length;j++)
		{
		printf("%02X ",msg_data[j]);
		
		}
	printf("\r\n");

return 0;	
}

////////////////////////////////
// I2C init routines
////////////////////////////////
//static esp_err_t i2c_master_init(void)
esp_err_t i2c_master_init(void)
{
    i2c_port_t i2c_master_port = I2C_MASTER_NUM;
	i2c_config_t conf = {};
//	memset(conf,0x00,sizeof(i2c_config_t));
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;	//ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;	//ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
//    conf.master.addr_10bit_en = 0;
    i2c_param_config(i2c_master_port, &conf);
	i2c_set_timeout(I2C_MASTER_NUM,0xFFFFF);	// ADDED DLT 27/07/2020
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

//static esp_err_t i2c_slave_init(void)
esp_err_t i2c_slave_init(void)
{
    i2c_port_t i2c_slave_port = I2C_SLAVE_NUM;
    i2c_config_t conf_slave = {};
//	memset(conf_slave,0x00,sizeof(i2c_config_t));
    conf_slave.sda_io_num = I2C_SLAVE_SDA_IO;
    conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.scl_io_num = I2C_SLAVE_SCL_IO;
    conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.mode = I2C_MODE_SLAVE;
    conf_slave.slave.addr_10bit_en = 0;
    conf_slave.slave.slave_addr = I2C_SLAVE_ADDR;
    i2c_param_config(i2c_slave_port, &conf_slave);
    return i2c_driver_install(i2c_slave_port, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);
}


// * @brief test code to read esp-i2c-slave
// * _______________________________________________________________________________________
// * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
// * --------|--------------------------|----------------------|--------------------|------|
// *

esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, unsigned char dev, unsigned char reg, uint8_t *data_rd, size_t size)
{
	signed int err;

// cmds are queued up until an i2c_master_cmd_begin is issued...		
    if (size == 0) 
        return ESP_OK;
		
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    err = i2c_master_write_byte(cmd, (dev << 1) | WRITE_BIT, ACK_CHECK_EN);
//	if(debug_do(DBG_I2C))
//		printf("R1: %04X   ",err);
    err = i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
//	if(debug_do(DBG_I2C))
//		printf("R2: %04X   ",err);

//    i2c_master_stop(cmd);

    i2c_master_start(cmd);
    err = i2c_master_write_byte(cmd, (dev << 1) | READ_BIT, ACK_CHECK_EN);

    if (size > 1) 
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);

    err = i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
//	if(debug_do(DBG_I2C))
//		printf("R3: %04X   ",err);
    err = i2c_master_stop(cmd);
//	if(debug_do(DBG_I2C))
//		printf("R4: %04X   ",err);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// * @brief Test code to write esp-i2c-slave
// *        Master device write data to slave
// * ___________________________________________________________________
// * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
// * --------|---------------------------|----------------------|------|
// *

esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, unsigned char dev, unsigned char reg, uint8_t *data_wr, size_t size)
{
	signed int err;

// cmds are queued up until an i2c_master_cmd_begin is issued...	
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    err = i2c_master_write_byte(cmd, (dev << 1) | WRITE_BIT, ACK_CHECK_EN);
//	if(debug_do(DBG_I2C))
//		printf("W1: %04X   ",err);
    err = i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
//	if(debug_do(DBG_I2C))
//		printf("W2: %04X   ",err);
    err = i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
//	if(debug_do(DBG_I2C))
//		printf("W3: %04X   ",err);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


#define NUM_TS_SAMPLES		2

// temp sensor I2C details (AD7415)		// I2C addresses moved to "per product" sections (due to overlapping addresses on TS0, LS1)
#define TS_TEMP_REG			0
#define TS_CFG_REG			1
#define TS_ALERT_OFF		0x60
#define TS_ONE_SHOT			0x04

esp_err_t check_i2c_device(unsigned char dev, unsigned char reg)
{
esp_err_t err;
unsigned char i2cdata[10];

	i2cdata[0] = 0;
	err = i2c_master_read_slave(I2C_MASTER_NUM, dev, reg, i2cdata, 1);
	if(debug_do(DBG_I2C))
		{
		printf("DEV:%02X  REG:%02X  DATA:%02X   ERR:%04X\r\n",dev,reg,i2cdata[0],err);
		}
return err;
}

esp_err_t check_i2c_bus(unsigned char reg)
{
esp_err_t err;
unsigned char i,j;

j = 0;

for (i=0;i<0x40;i++)
	{
	err = check_i2c_device(i,0);
	printf("%02X: ",i);
	if (err == ESP_OK)
		{
		printf("Y  ");
		j = 1;
		}
	else
		printf("   ");
	
	if (i%8 == 7)
		printf("\n");
	}

if (j)
	err = ESP_OK;
else
	err = ESP_FAIL;	

return err;
}

unsigned char get_temperature(unsigned char ts_addr, signed char *ts_val)
{
// take a number of samples of the temperature
signed int err;
unsigned char i,errflag;
signed int c = 0;

unsigned char i2cdata[10];

//unsigned char i2c_sample[NUM_TS_SAMPLES];
//unsigned char i2c_error[NUM_TS_SAMPLES];

errflag = 0;
err = 0;

for (i=0;i<NUM_TS_SAMPLES;i++)
	{
//	i2cdata[0] = TS_CFG_REG;
	i2cdata[0] = TS_ALERT_OFF | TS_ONE_SHOT;	// switch off alert function and make one-shot conversion

//**	err = i2c_wr(ts_addr,i2cdata,1,1);			// note lsbs are in second byte
	if(debug_do(DBG_I2C))
		{
		printf("I2C write setup info\r\n");
		}
	err = i2c_master_write_slave(I2C_MASTER_NUM, ts_addr, TS_CFG_REG, i2cdata, 1);

	if(debug_do(DBG_I2C))
		{
		printf("I2C write = %04X\r\n",err);
		}
		
	ets_delay_us(40);								// wait for conversion to happen  (~25us)

// check read of the CFG reg...
	i2cdata[0] = 0;
	err = i2c_master_read_slave(I2C_MASTER_NUM, ts_addr, TS_CFG_REG, i2cdata, 1);
	if(debug_do(DBG_I2C))
		{
		printf("CFG = %02X  %04X\r\n",i2cdata[0],err);
		}

	i2cdata[0] = 0;
	
//**	err = i2c_rd(ts_addr,i2cdata,1,2);			// note lsbs are in second byte
	if(debug_do(DBG_I2C))
		{
		printf("I2C read temp info\r\n");
		}
//static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, unsigned char dev, unsigned char reg, uint8_t *data_rd, size_t size)
// 2 byte read to get temp upper and lower bytes...
	err = i2c_master_read_slave(I2C_MASTER_NUM, ts_addr, TS_TEMP_REG, i2cdata, 2);
	if(debug_do(DBG_I2C))
		{
		printf("I2C read = %04X\r\n",err);
		}

//	printf("Process temp info\r\n");
	if(debug_do(DBG_I2C))
		{
		printf("I2C val = %02X %02X %02X\r\n",i2cdata[0],i2cdata[1],i2cdata[2]);
		}
	if (err)
		{
//		i2c_sample[i] = 0;
//		i2c_error[i] = err;						// get err number from I2C rtn
		errflag = 1;
		}
	else
		{
//		i2c_sample[i] = i2cdata[0];				// get return data - just the msbs in byte 0
		c = c + i2cdata[0];						// get return data - just the msbs in byte 0
//		i2c_error[i] = 0;
		}
	

//	c = c + i2c_sample[i];

	}

//	printf("Set val of temp info\r\n");

// now check the samples
	if (errflag)
		{
//		printf("Opt1\r\n");
		*ts_val = 0;
		}
	else
		{
//		printf("Opt2\r\n");
		c = c/NUM_TS_SAMPLES;
		*ts_val = (signed char)c;
//	err = 0;
		}
	if(debug_do(DBG_I2C))
		{
		printf("Temp end %d\r\n",*ts_val);
		}
		
return errflag;
}

void print_banner_msg(char *str)
{
unsigned char len;
len = strlen(str);
//        1234567890123456789012345678901234567890
//printf("*************************************\n");
//printf("*                                   *\n");
//printf("Setting payloads for DATA\n");
//printf("*                                   *\n");
//printf("*************************************\n");

print_chars('*',6+len,1);
printf("*");
print_chars(' ',4+len,0);
printf("*\n");
printf("*  %s  *\n",str);
printf("*");
print_chars(' ',4+len,0);
printf("*\n");
print_chars('*',6+len,1);

}

void print_chars(char c, unsigned char num, unsigned char nl_flag)
{
unsigned char i;

for (i=0;i<num;i++)
	{
	printf("%c",c);
	}
	
if (nl_flag)
	printf("\n");
}

char set_uart_pattern_detect(char pattern_chr)
{
// disable old pattern detect interrupt
	uart_disable_pattern_det_intr(FOUR_G);
// Set uart pattern detect function.
    uart_enable_pattern_det_intr(FOUR_G, pattern_chr, 1, 10000, 10, 10);		// detect 1 x char
	uart_pattern_queue_reset(FOUR_G,FOUR_G_RX_BUFSIZE/10);							// rx buffer / min msg length
	
	if ((debug_do(DBG_MQTT)) && (prev_pattern_detect_char != pattern_chr))
		{
		printf("PCHR %02X\n",pattern_chr);
		}

	prev_pattern_detect_char = pattern_chr;
		
	return pattern_chr;
}


//unsigned char listmatch(unsigned char listlen, unsigned char entrylen, char list[listlen][entrylen], char* matchitem)
//unsigned char listmatch(unsigned char listlen, unsigned char entrylen, char list[listlen][entrylen], char* matchitem, unsigned char dbg_level)
unsigned char listmatch(unsigned int listlen, unsigned char entrylen, char *list, char *matchitem, unsigned char binary, unsigned char dbg_level)
{

// if comparison not binary, items terminated by 0x00:     len = lesser of current entry (elen) and matchitem (llen)
// if comparison is  binary, items not terminated by 0x00: len = entrylen

// returns 0 is np match, 1 if match found

unsigned char ret = 0;	
unsigned int i;
unsigned char j,n,x,elen,llen;

char tmpstr1[entrylen+1];
char tmpstr2[entrylen+1];

//printf("%d %d %d %d\n",listlen,entrylen,binary,dbg_level);
n = 0;
x = 0;

/*
printf("%02X ",matchitem[0]);
printf("%02X ",matchitem[1]);
printf("%02X ",matchitem[2]);
printf("%02X ",matchitem[3]);
printf("%02X ",matchitem[4]);
printf("%02X ",matchitem[5]);
*/
for (i=0;i<listlen;i++)
	{
	if (binary)			// if search on binary items
		{
		n = entrylen;
		}
	else
		{
//	elen = strlen(list[i]);
		elen = strlen((char *)(list+(i*entrylen)));
		if (elen == 0)
			elen = entrylen;
		
		llen = strlen(matchitem);
		if (elen < llen)
			n = elen;
		else
			n = llen;
		
		if (n > entrylen)
			n = entrylen;
		}
		
//	printf("elen: %d llen: %d entrylen: %d\n",elen, llen, entrylen);
	
//	strncpy(tmpstr1,matchitem,elen);
//	tmpstr1[elen] = 0x00;
	
// check for match
//	if (!strcmp(list[i],tmpstr1))
//	if ((n) && (!strncmp(list[i],matchitem,n)))	// if table entry not zero length and strings match...
//	if ((n) && (!strncmp((char *)(list+(i*entrylen)),matchitem,n)))	// if table entry not zero length and strings match...
	if ((n) && (!memcmp((char *)(list+(i*entrylen)),matchitem,n)))	// if table entry not zero length and strings match...
		{
		ret = 1;
		}
	

/*		
// now get matchitem to entrylen
	strncpy(tmpstr1,matchitem,entrylen);
	tmpstr1[entrylen] = 0x00;

	strcpy(tmpstr2,list[i]);
// now pad out with spaces to entrylen
	for (j=strlen(list[i]);j<entrylen;j++)
		{
		tmpstr2[j] = ' ';
		}
	tmpstr2[entrylen] = 0x00;
	
//	strncpy(tmpstr,matchitem,elen);
*/

//#define DEBUG_LISTMATCH
//#define DEBUG_LISTMATCH_MATCH_ONLY
	
#ifdef DEBUG_LISTMATCH
#ifdef DEBUG_LISTMATCH_MATCH_ONLY
	if (ret)
#endif	
		{
		if (binary)
			{
			printf("LM: %03d : item: ",i);
			for (j=0;j<n;j++)
				{
				printf("%02X ",matchitem[j]);
				}
		printf("\n   table entry: ");
			for (j=0;j<n;j++)
				{
				printf("%02X ",*(list+(i*entrylen)+j));
				}

			printf("\nlen: %d : match %d\n",n,ret);
			}
		else
			{
//		printf("LM: %d : item: %s :  table entry: %s : len: %d : match %d\n",i,tmpstr1,tmpstr2,elen,ret);
//			printf("LM: %d : item: %s :  table entry: %s : len: %d : match %d\n",i,matchitem,list[i],n,ret);

			printf("LM: %d : item: ",i);
			j = 0;
			while(matchitem[j] != 0x00)
				{
				printf("%02X ",matchitem[j]);
				j++;
				}
			printf(":  table entry: ");
			j = 0;
			while(*(list+(i*entrylen)+j)!= 0x00)
				{
				printf("%02X ",*(list+(i*entrylen)+j));
				j++;
				}

			printf("\nlen: %d : match %d\n",n,ret);


			}
		}
#endif

// now i is shown in printf, can use it to terminate the loop
	if (ret)
		{
		x = i;
		i = listlen+1;	// terminate loop
		}
		
	}	// end of search loop

#ifdef DEBUG_LISTMATCH
if(ret == 0)
	printf("No match found\n");
#endif

//#define DEBUG_LISTMATCH2
#ifdef DEBUG_LISTMATCH2
/*
printf("LM:");
	if (ret)
		printf("OK\n");
	else
		printf("NM\n");
*/
	if (dbg_level)
		{
#if 0
	if (ret)
			printf("Y ");
		else
			printf("N ");
#else
		strncpy(tmpstr1,matchitem,entrylen);
		tmpstr1[entrylen] = 0x00;
//	printf("LM: item: %s : match %d\n",matchitem,ret);
//	printf("LM: item: %s : match %d\n",list[x],ret);

//		printf("LM: item: %s : match %d",tmpstr1,ret);
		printf("LM: item: ");
		for(j=0;j<n;j++)
			{
			printf("%02X ",tmpstr1[j]);
			}
		printf(" : len: %d",n);
		printf(": match %d",ret);


		if (ret)
			printf(" : inx: %d",x);
#endif
		printf("\n");
		}

#endif

	
return ret;
}

unsigned char get_pipe(unsigned int *p, char *msgstr)
{
// searches msgstr for '|' char. Returns 1 if found, 0 if not. 
// p is advanced to next char position in the string after the '|' character
unsigned char ret = 0;
unsigned int tmp;
tmp = *p;

while((msgstr[*p] != '|') && (msgstr[*p] != 0x00))
	{(*p)++;}
	
if ((msgstr[*p] == 0x00) && (*p == tmp))	// end of string with no more data
	ret = 0;
else
	{
	ret = 1;								// either a pipe was found or it is the last data segment
	
	if (msgstr[*p] == '|')
		(*p)++;								// advance index to next position in the array...
	else
		*p = tmp;							//... else reset the index to the start of data segment
	}
	
return ret;
}

unsigned char get_listfield(unsigned int *p, char *msgstr, char *listfield)
{
// gets field string from current position in mststr given by p;
// stops at either '|' char or end of string
unsigned char ret = 0;
unsigned char i = 0;

while((msgstr[*p] != '|') && (msgstr[*p] != 0x00))
	{
	listfield[i] = msgstr[*p];
	i++;
	(*p)++;
	}

if (i > 0)					// if a string found
	ret = 1;

if (msgstr[*p] == 0x00)		// if end of string reached
	ret = ret + 2;
	
listfield[i] = 0x00;	// terminate the string

(*p)++;			// advance index to next position in the array...

return ret;
}

unsigned char print_hex(unsigned char *dataptr, unsigned int len, unsigned char addrnotindex, unsigned char txt)
{
unsigned char ret = 0;
unsigned char c,i,j;
unsigned int total = 0;
unsigned char *locptr;

locptr = dataptr;
total = 0;
i = 0;
j = 16;	

while(len > total)
	{
	if (len - total < 16)					// if not a full line of 16 bytes...
		j = len - total;

	if (txt)								// if txt flag set...
		{
		for (i=0;i<j;i++)
			{
			if(i== 0)
				{
				printf("         ");
				}
			c = *(locptr+i);
			if ((c<0x20) || (c > 0x7F))
				c = '.';	//0x20;
			printf("  %c",c);

			if(i==(j-1))
				printf("\n");
			}
		}
		
	for (i=0;i<j;i++)						// the hex output
		{
		if(i== 0)
			{
			if (addrnotindex)
				printf("%08lX:",(unsigned long)locptr);
			else
				printf("%04X:",total);
			}
		printf(" %02X",*locptr);
		locptr++;
		total++;

		if(i==(j-1))
			printf("\n");
		}
		
	}	
if(i!=(j-1))
	printf("\n");
	
return ret;
}


unsigned char asc2hex(unsigned char* str)
{
unsigned char result,c,val;	

	c = str[0] - 0x30;               // upper nibble

    if (c<10)						// number
        val = c * 16;
    else
        {
        if (c>32)
			c = c-0x20;
			
        val = (c-7) * 16;
        }
        
    c = str[1] - 0x30;               // lower nibble
    if (c<10)
        val = val + c;
    else
        {
        if (c>32)
			c = c-0x20;

		val = val + (c-7);
		}
 
return val;
}


////////////////////////////////////////////
//
//  NVS variables
//
////////////////////////////////////////////
// NVS variable area list:
// GW_VAR_LIST
// BT_MAC_LIST
// BD_MAC_LIST
// LORA_MAC_LIST
// BT_SENSOR
// EVT_VARS
// WIFI_MQT_VARS
// SRVR_ADDR_LIST
// RALLY_SF_LIST

unsigned char nvs_gw_var_init(void)
{
unsigned char var = 0xFF;
unsigned int ivar;
unsigned long long int lvar = 0;
unsigned char mac_list[6];
unsigned char i,j;
unsigned int n;
esp_err_t err;
char tmpstr[10] = {0x0A,0x00};
	
nvs_handle nvs_handle;

// GATEWAY VARIABLE SPACE (max 15 chars!)
err = nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);

	
	printf("*** setting NVS GW VAR values...\n");
	printf("nvs_try: %d\n",err);
	
	nvs_set_u8(nvs_handle,"GW_AUTOBAUD",0);
	nvs_set_u32(nvs_handle,"GW_SER_RATE",DFLT_EXT_SERIAL_BPS);
	nvs_set_u8(nvs_handle,"GW_BIN_FLAG",0);
	nvs_set_str(nvs_handle,"GW_EOL_STR",tmpstr);

	nvs_set_u8(nvs_handle,"GW_BWL_FLAG",1);		// Bluetooth whitelist enable flag
	nvs_set_u8(nvs_handle,"GW_BBL_FLAG",0);		// Bluetooth blacklist enable flag
	nvs_set_u8(nvs_handle,"GW_LWL_FLAG",1);		// LoRaWAN whitelist enable flag
	nvs_set_u8(nvs_handle,"GW_LBL_FLAG",0);		// LoRaWAN blacklist enable flag
	nvs_set_u8(nvs_handle,"GW_EOLREM_FLAG",0);

	nvs_set_u16(nvs_handle,"GW_SENSOR_TMR",GATEWAY_SENSOR_TIME);
	nvs_set_u16(nvs_handle,"GW_GPS_TMR",GPS_SENSOR_TIME);
	nvs_set_u16(nvs_handle,"GW_I2C_TMR",I2C_SENSOR_TIME);
	nvs_set_u16(nvs_handle,"GW_TANK_TMR",TANK_SENSOR_TIME);
	nvs_set_u16(nvs_handle,"GW_BTADDR_TMR",BLUETOOTH_MAC_ADDR_TIME);
	nvs_set_u16(nvs_handle,"GW_LOADDR_TMR",LORA_DEVADDR_TIME);


//	nvs_set_i8(nvs_handle,"GW_SER_RATE",2);
	nvs_commit(nvs_handle);
nvs_close(nvs_handle);

return 0;
}

unsigned char set_gw_vars_from_nvs(void)
{
unsigned char var = 0xFF;
unsigned int ivar;
unsigned long long int lvar = 0;
unsigned char mac_list[6];
unsigned char i,j;
unsigned int n;
esp_err_t err;
	
nvs_handle nvs_handle;

// GATEWAY VARIABLE SPACE (max 15 chars!)
err = nvs_open("GW_VAR_LIST",NVS_READWRITE, &nvs_handle);

	
	printf("*** setting GW vars from NVS...\n");
	printf("nvs_try: %d\n",err);


nvs_get_u8(nvs_handle,"GW_AUTOBAUD",(uint8_t*)&serial_autobaud_flag);
nvs_get_u32(nvs_handle,"GW_SER_RATE",(uint32_t*)&serial_bitrate);
nvs_get_u8(nvs_handle,"GW_BIN_FLAG",(uint8_t*)&binary_data_flag);
nvs_get_str(nvs_handle,"GW_EOL_STR",eolstr,&n);					// n is number of bytes read out...
printf("Serial_AutoBaud_Flag = %d\t",serial_autobaud_flag);
printf("Serial_BitRate       = %ld\n",serial_bitrate);
printf("Serial_Binary_Flag   = %d\t",binary_data_flag);
printf("Serial EOL String    = ");
i = 0;
while (eolstr[i] != 0x00)
	{
	printf("%02X ",eolstr[i]);		
	i++;
	}
printf("\n\n");

nvs_get_u8(nvs_handle,"GW_BWL_FLAG",(uint8_t*)&enable_bluetooth_whitelist_flag);		// Bluetooth whitelist enable flag
nvs_get_u8(nvs_handle,"GW_BBL_FLAG",(uint8_t*)&enable_bluetooth_blacklist_flag);		// Bluetooth blacklist enable flag
nvs_get_u8(nvs_handle,"GW_LWL_FLAG",(uint8_t*)&enable_lorawan_whitelist_flag);		// LoRaWAN whitelist enable flag
nvs_get_u8(nvs_handle,"GW_LBL_FLAG",(uint8_t*)&enable_lorawan_blacklist_flag);		// LoRaWAN blacklist enable flag
nvs_get_u8(nvs_handle,"GW_EOLREM_FLAG",(uint8_t*)&eol_remove_flag);

	nvs_get_u16(nvs_handle,"GW_SENSOR_TMR",(uint16_t*)&gateway_sensor_time);
	nvs_get_u16(nvs_handle,"GW_GPS_TMR",(uint16_t*)&gps_sensor_time);
	nvs_get_u16(nvs_handle,"GW_I2C_TMR",(uint16_t*)&i2c_sensor_time);
	nvs_get_u16(nvs_handle,"GW_TANK_TMR",(uint16_t*)&tank_sensor_time);
	nvs_get_u16(nvs_handle,"GW_BTADDR_TMR",(uint16_t*)&bluetooth_MAC_addr_time);
	nvs_get_u16(nvs_handle,"GW_LOADDR_TMR",(uint16_t*)&lora_devaddr_time);

nvs_close(nvs_handle);

if ((binary_data_flag == 1) || ((binary_data_flag == 2) && (auto_binary_data_flag == 1)))
	binary_mode_flag = 1;
else
	binary_mode_flag = 0;

return 0;
}

unsigned char nvs_bl_var_init(char* area, char* num_str, char* item_str, char *init_array, unsigned char init_len, unsigned int entrylen, unsigned char ascii_flag)
{
char tmp_str[17];
unsigned char i, num;
unsigned long long int lvar = 0;
//unsigned char mac_list[6];
char str[80];
unsigned char buflen;

esp_err_t err;
	
nvs_handle nvs_handle;

//printf("NVS not init:         %d\n",ESP_ERR_NVS_NOT_INITIALIZED-ESP_ERR_NVS_BASE);
//printf("NVS invalid name:     %d\n",ESP_ERR_NVS_INVALID_NAME-ESP_ERR_NVS_BASE);
//printf("NVS invalid handle:   %d\n",ESP_ERR_NVS_INVALID_HANDLE-ESP_ERR_NVS_BASE);
// VARIABLE SPACE (max 15 chars!)

printf("*** setting %d NVS ",init_len);
	
if (ascii_flag)
	printf("ASCII");
else
	printf("BINARY");
		
printf(" values in %s / %s...\n",area,item_str);

for (i=0;i< init_len;i++)
	{
	err = nvs_open(area,NVS_READWRITE, &nvs_handle);
		if (err != ESP_OK)
			printf("Error %d opening NVS space %s\n",(unsigned int)err-ESP_ERR_NVS_BASE,area);
	

	sprintf(tmp_str,"%s_%03d",item_str,i);
	if (ascii_flag)
		{
//		printf("HANDLE: %08X NAME: %s    ARR: %s [%d]\n",nvs_handle,tmp_str,&init_array[i*entrylen],strlen(&init_array[i*entrylen]));
		
		err = nvs_set_str(nvs_handle,tmp_str,&init_array[i*entrylen]);

		if (err != ESP_OK)
			printf("Set Error! %d\n",err-ESP_ERR_NVS_BASE);

		err = nvs_commit(nvs_handle);
		if (err != ESP_OK)
			printf("Commit Error! %d\n",err-ESP_ERR_NVS_BASE);

//		printf("HANDLE_01: %08X\n",nvs_handle);		

		nvs_close(nvs_handle);			// ESP32 notes say once committed, handle should not be used...
		
		err = nvs_open(area,NVS_READWRITE, &nvs_handle);	// reopen handle...
		if (err != ESP_OK)
			printf("Error %d opening NVS space %s\n",(unsigned int)err-ESP_ERR_NVS_BASE,area);
		
		buflen = 0;	//strlen(&init_array[i*entrylen]);
		err = nvs_get_str(nvs_handle,tmp_str,str,(size_t*)&buflen);

		if (err != ESP_OK)
			printf("Get Error! %d\n",err-ESP_ERR_NVS_BASE);

//		printf("HANDLE_02: %08X\n",nvs_handle);		

		printf("%03d) %s = %s [%d]\n",i,tmp_str,str,buflen);


//		printf("HANDLE_03: %08X\n",nvs_handle);		
		}
	else
		{
		nvs_set_addr(nvs_handle, item_str, i, (unsigned char*)&init_array[i*entrylen],entrylen);

		nvs_get_u64(nvs_handle,tmp_str,&lvar);

		printf("%03d)  %12llX\n",i,lvar); 

		}

	err = nvs_commit(nvs_handle);
	nvs_close(nvs_handle);			// ESP32 notes say once committed, handle should not be used...

	}
		
	
err = nvs_open(area,NVS_READWRITE, &nvs_handle);	// reopen handle...
if (err != ESP_OK)
	printf("Error %d opening NVS space %s\n",(unsigned int)err-ESP_ERR_NVS_BASE,area);

err = nvs_set_i8(nvs_handle,num_str, i);
if (err != ESP_OK)
	printf("Set Num Error! %d\n",err-ESP_ERR_NVS_BASE);

nvs_commit(nvs_handle);

nvs_close(nvs_handle);

return 0;
}

unsigned char set_bl_vars_from_nvs(char* area, char* num_str, char* item_str, char* table, unsigned int* num_entries, unsigned int entrylen, unsigned char ascii_flag)
{
char tmp_str[17];
unsigned char i, num;
unsigned long long int lvar = 0;
unsigned char mac_list[6];
char str[80];
unsigned char buflen;
esp_err_t err;
	
nvs_handle nvs_handle;

// VARIABLE SPACE (max 15 chars!)


	err = nvs_open(area,NVS_READWRITE, &nvs_handle);
	if (err)
		printf("Error %d opening NVS space %s\n",err-ESP_ERR_NVS_BASE,area);

	nvs_get_i8(nvs_handle,num_str,(int8_t*)&num);

	nvs_close(nvs_handle);

	printf("*** getting %d NVS values from %s / %s...\n",num,area,item_str);

for (i=0;i<num;i++)
	{
	err = nvs_open(area,NVS_READWRITE, &nvs_handle);
	if (err)
		printf("Error %d opening NVS space %s\n",err-ESP_ERR_NVS_BASE,area);

	sprintf(tmp_str,"%s_%03d",item_str,i);
	if (ascii_flag)
		{
		buflen = 0;
		err = nvs_get_str(nvs_handle,tmp_str,str,(size_t*)&buflen);
		if (err != ESP_OK)
			printf("Get Error! %d\n",err-ESP_ERR_NVS_BASE);

		memcpy(&table[i*entrylen],str,strlen(str));
		printf("%03d) %s = %s [%d]\n",i,tmp_str,str,buflen); 
//		printf("HANDLE_01: %08X\n",nvs_handle);		
		}
	else
		{
		nvs_get_u64(nvs_handle,tmp_str,&lvar);
		memcpy(&table[i*entrylen],&lvar,6);
		printf("%03d)  %12llX\n",i,lvar); 
		}
	
	nvs_close(nvs_handle);
	}

//nvs_close(nvs_handle);
	
*num_entries = num;
	
return 0;
}
//////////////////////////////////////////
unsigned char nvs_evt_var_init(void)
{
char tmp_str[17];
unsigned char i, num;
unsigned long long int lvar = 0;
//unsigned char mac_list[6];
char str[80];
unsigned char buflen;

esp_err_t err;
	
nvs_handle nvs_handle;

// VARIABLE SPACE (max 15 chars!)
err = nvs_open("EVT_VARS", NVS_READWRITE, &nvs_handle);

	
	printf("*** setting NVS values in EVT_VARS ...\n");

for (i=0;i< NUM_EVENTS_MAX;i++)
	{
// event enable flag
	sprintf(tmp_str,"EVT_EN_%03d",i);
	nvs_set_u8(nvs_handle,tmp_str,0);

// event mode flag
	sprintf(tmp_str,"EVT_MODE_%03d",i);
	nvs_set_u8(nvs_handle,tmp_str,0);

// event day flag
	sprintf(tmp_str,"EVT_DAY_%03d",i);
	nvs_set_u8(nvs_handle,tmp_str,0);

// event T1 hrs flag
	sprintf(tmp_str,"EVT_T1HRS_%03d",i);
	nvs_set_u8(nvs_handle,tmp_str,0);

// event T1 mins flag
	sprintf(tmp_str,"EVT_T1MNS_%03d",i);
	nvs_set_u8(nvs_handle,tmp_str,0);

// event T2 hrs flag
	sprintf(tmp_str,"EVT_T2HRS_%03d",i);
	nvs_set_u8(nvs_handle,tmp_str,0);

// event T2 mins flag
	sprintf(tmp_str,"EVT_T2MNS_%03d",i);
	nvs_set_u8(nvs_handle,tmp_str,0);

// event PWM value
	sprintf(tmp_str,"EVT_PWM_%03d",i);
	nvs_set_u8(nvs_handle,tmp_str,0);

	}
		
	
nvs_set_i8(nvs_handle,"EVT_NUM_EVTS", i);
nvs_commit(nvs_handle);

nvs_close(nvs_handle);

return 0;
}

unsigned char set_evt_vars_from_nvs(void)
{
char tmp_str[17];
unsigned char i, num;
unsigned long long int lvar = 0;
unsigned char mac_list[6];
char str[80];
unsigned char buflen;
esp_err_t err;
	
nvs_handle nvs_handle;

// VARIABLE SPACE (max 15 chars!)

	printf("*** getting NVS values from EVT_VARS...\n");

	err = nvs_open("EVT_VARS", NVS_READWRITE, &nvs_handle);

	nvs_get_i8(nvs_handle,"EVT_NUM_EVTS",(int8_t*)&num);

	if (num != NUM_EVENTS_MAX)
		return 1;
	
	
for (i=0;i< NUM_EVENTS_MAX;i++)
	{
/*
unsigned char evt_enable_flag;
unsigned char evt_mode_flag;
unsigned char evt_day;
unsigned char evt_t1_hrs;
unsigned char evt_t1_mins;
unsigned char evt_t2_hrs;
unsigned char evt_t2_mins;
unsigned char evt_pwm;
unsigned char evt_running;
*/
// event enable flag
	sprintf(tmp_str,"EVT_EN_%03d",i);
	nvs_get_u8(nvs_handle,tmp_str,&event_list[i].evt_enable_flag);

// event mode flag
	sprintf(tmp_str,"EVT_MODE_%03d",i);
	nvs_get_u8(nvs_handle,tmp_str,&event_list[i].evt_mode_flag);

// event day flag
	sprintf(tmp_str,"EVT_DAY_%03d",i);
	nvs_get_u8(nvs_handle,tmp_str,&event_list[i].evt_day);

// event T1 hrs flag
	sprintf(tmp_str,"EVT_T1HRS_%03d",i);
	nvs_get_u8(nvs_handle,tmp_str,&event_list[i].evt_t1_hrs);

// event T1 mins flag
	sprintf(tmp_str,"EVT_T1MNS_%03d",i);
	nvs_get_u8(nvs_handle,tmp_str,&event_list[i].evt_t1_mins);

// event T2 hrs flag
	sprintf(tmp_str,"EVT_T2HRS_%03d",i);
	nvs_get_u8(nvs_handle,tmp_str,&event_list[i].evt_t2_hrs);

// event T2 mins flag
	sprintf(tmp_str,"EVT_T2MNS_%03d",i);
	nvs_get_u8(nvs_handle,tmp_str,&event_list[i].evt_t2_mins);

// event PWM value
	sprintf(tmp_str,"EVT_PWM_%03d",i);
	nvs_get_u8(nvs_handle,tmp_str,&event_list[i].evt_pwm);

	}

nvs_close(nvs_handle);
	
show_events();

	
return 0;
}


///////////////////////////////////

unsigned char nvs_wifi_sta_var_init(void)
{
//char tmp_str[17];
//unsigned char i, num;
//unsigned long long int lvar = 0;
//unsigned char mac_list[6];
//char str[80];
//unsigned char buflen;

esp_err_t err;
	
nvs_handle nvs_handle;

#ifdef USE_WIFI_MQTT

// VARIABLE SPACE (max 15 chars!)
err = nvs_open("WIFI_MQT_VARS", NVS_READWRITE, &nvs_handle);

	
	printf("*** setting NVS values in WIFI_MQT_VARS ...\n");

// wifi sta SSID string
	err = nvs_set_str(nvs_handle,"WIFI_MQ_STA_ID",WIFI_STA_PREFIX);
	if (err)
		printf("Init Err1: %d\n",err);
// wifi sta PWD string
	err = nvs_set_str(nvs_handle,"WIFI_MQ_STA_PW",WIFI_STA_PASSWORD);
	if (err)
		printf("Init Err2: %d\n",err);

	err = nvs_set_u8(nvs_handle,"WIFI_STA_AUTH",WIFI_AUTH_WPA2_PSK);	// WPA2_PSK
	if (err)
		printf("Init Err3: %d\n",err);

	err = nvs_set_u8(nvs_handle,"WIFI_AP_AUTH",WIFI_AUTH_WPA2_PSK);	// WPA2_PSK
	if (err)
		printf("Init Err4: %d\n",err);


nvs_commit(nvs_handle);

nvs_close(nvs_handle);

#endif

return 0;
}

unsigned char set_wifi_sta_vars_from_nvs(void)
{
char tmp_str[17];
unsigned char i, num;
unsigned long long int lvar = 0;
unsigned char mac_list[6];
char str[80];
size_t buflen;
esp_err_t err;
	
nvs_handle nvs_handle;

// VARIABLE SPACE (max 15 chars!)

	printf("*** getting NVS values from WIFI_MQT_VARS...\n");

	err = nvs_open("WIFI_MQT_VARS", NVS_READWRITE, &nvs_handle);

// specify size of receiving string...
	buflen = 32;
	err = nvs_get_str(nvs_handle,"WIFI_MQ_STA_ID",str,&buflen);
	
	if ((err == 0) && (buflen < 32))
		strcpy(wifi_sta_SSID[0],str);
	else
		printf("Err1: %d  %d\n",err,buflen);

// specify size of receiving string...
	buflen = 32;
	err = nvs_get_str(nvs_handle,"WIFI_MQ_STA_PW",str,&buflen);
	
	if ((err == 0) && (buflen < 32))
		strcpy(wifi_sta_PWD[0],str);
	else
		printf("Err2: %d  %d\n",err,buflen);
	
	err = nvs_get_u8(nvs_handle,"WIFI_STA_AUTH",(uint8_t*)&wifi_sta_auth);	// WPA2_PSK
	if (err)
		printf("Err3: %d\n",err);
			
	err = nvs_get_u8(nvs_handle,"WIFI_AP_AUTH", (uint8_t*)&wifi_ap_auth);	// WPA2_PSK
	if (err)
		printf("Err4: %d\n",err);
	

nvs_close(nvs_handle);

	
return 0;
}



unsigned char nvs_srvr_addr_port_var_init(void)
{
unsigned char var = 0xFF;
unsigned int ivar;
unsigned long long int lvar = 0;
unsigned char mac_list[6];
unsigned char i,j;
unsigned int n;
esp_err_t err;
char tmpstr[10] = {0x0A,0x00};
	
nvs_handle nvs_handle;

// GATEWAY VARIABLE SPACE (max 15 chars!)

	printf("*** setting NVS SRVR ADDR and PORT values...\n");

#if 1	
	err = nvs_open("SRVR_ADDR_LIST",NVS_READWRITE, &nvs_handle);
	printf("nvs_try: %d\n",err);

	nvs_set_u8(nvs_handle,"SA_ADDR_PTR_0",0);
	nvs_set_u8(nvs_handle,"SA_ADDR_PTR_1",0);

	nvs_set_u8(nvs_handle,"SA_PREF_TRANSP",MQTT_MODE_4G);

//	nvs_set_str(nvs_handle,"SA_ADDR_00",SERVER_ADDRESS_AND_PORT_0);	//dflt_server_addr_and_port[1]);
//	nvs_set_str(nvs_handle,"SA_ADDR_01",SERVER_ADDRESS_AND_PORT_1);	//dflt_server_addr_and_port[1]);
//	nvs_set_str(nvs_handle,"SA_ADDR_02",SERVER_ADDRESS_AND_PORT_2);	//dflt_server_addr_and_port[2]);
//	nvs_set_str(nvs_handle,"SA_ADDR_03",SERVER_ADDRESS_AND_PORT_3);	//dflt_server_addr_and_port[3]);

	nvs_commit(nvs_handle);
nvs_close(nvs_handle);
#endif
return 0;
}

unsigned char set_srvr_addr_port_vars_from_nvs(void)
{
unsigned char var = 0xFF;
unsigned int ivar;
unsigned long long int lvar = 0;
unsigned char mac_list[6];
unsigned char i,j;
unsigned int n;
esp_err_t err;
	
nvs_handle nvs_handle;

// GATEWAY VARIABLE SPACE (max 15 chars!)
	
	printf("*** getting SRVR ADDR and PORT from NVS...\n");

#ifdef USE_M4G_MQTT
err = nvs_open("SRVR_ADDR_LIST",NVS_READWRITE, &nvs_handle);
printf("nvs_try: %d\n",err);

//nvs_get_str(nvs_handle,"GW_EOL_STR",eolstr,&n);					// n is number of bytes read out...

	nvs_get_u8(nvs_handle,"SA_ADDR_PTR_0",(uint8_t*)&server0_addr_ptr);
	nvs_get_u8(nvs_handle,"SA_ADDR_PTR_1",(uint8_t*)&server1_addr_ptr);

//	server0_ssl_mode = server_ssl_mode[server0_addr_ptr];
//	server1_ssl_mode = server_ssl_mode[server1_addr_ptr];

//	nvs_get_str(nvs_handle,"SA_ADDR_00",server_addr_and_port[0],&n);
//	nvs_get_str(nvs_handle,"SA_ADDR_01",server_addr_and_port[1],&n);
//	nvs_get_str(nvs_handle,"SA_ADDR_02",server_addr_and_port[2],&n);
//	nvs_get_str(nvs_handle,"SA_ADDR_03",server_addr_and_port[3],&n);

//	printf("Server0 addr ptr  %d\n",server0_addr_ptr);	

	nvs_get_u8(nvs_handle,"SA_PREF_TRANSP",(uint8_t*)&mqtt_preferred_transport_mode);

nvs_close(nvs_handle);


#endif
return 0;
}

void show_server_addrs(void)
{
printf("Server Addresses:\n");	
#ifdef USE_M4G_MQTT
printf("SSL enable: %d\n",ssl_enable_flag);
#ifdef SERVER_ADDRESS_AND_PORT_0
printf("[0]:  %s  ",SERVER_ADDRESS_AND_PORT_0);
print_spaces(50 - strlen(SERVER_ADDRESS_AND_PORT_0));
if (SERVER_SSL_MODE_0 == 0)
	printf("[noSSL] ");
else
	printf("[SSL  ] ");

if (server0_addr_ptr == 0)
	printf("[0] ");
if (server1_addr_ptr == 0)
	printf("[1] ");
printf("\n");
#else
printf("[0]:\n"
#endif

#ifdef SERVER_ADDRESS_AND_PORT_1
printf("[1]:  %s  ",SERVER_ADDRESS_AND_PORT_1);
print_spaces(50 - strlen(SERVER_ADDRESS_AND_PORT_1));
if (SERVER_SSL_MODE_1 == 0)
	printf("[noSSL] ");
else
	printf("[SSL  ] ");

if (server0_addr_ptr == 1)
	printf("[0] ");
if (server1_addr_ptr == 1)
	printf("[1] ");
printf("\n");
#else
printf("[1]:\n"
#endif

#ifdef SERVER_ADDRESS_AND_PORT_2
printf("[2]:  %s  ",SERVER_ADDRESS_AND_PORT_2);
print_spaces(50 - strlen(SERVER_ADDRESS_AND_PORT_2));
if (SERVER_SSL_MODE_2 == 0)
	printf("[noSSL] ");
else
	printf("[SSL  ] ");

if (server0_addr_ptr == 2)
	printf("[0] ");
if (server1_addr_ptr == 2)
	printf("[1] ");
printf("\n");
#else
printf("[2]:\n"
#endif

#ifdef SERVER_ADDRESS_AND_PORT_3
printf("[3]:  %s  ",SERVER_ADDRESS_AND_PORT_3);
print_spaces(50 - strlen(SERVER_ADDRESS_AND_PORT_3));
if (SERVER_SSL_MODE_3 == 0)
	printf("[noSSL] ");
else
	printf("[SSL  ] ");

if (server0_addr_ptr == 3)
	printf("[0] ");
if (server1_addr_ptr == 3)
	printf("[1] ");
printf("\n");
#else
printf("[3]:\n"
#endif

#ifdef DFLT_SERVER_ADDRESS_AND_PORT
printf("DFLT: %s  ",DFLT_SERVER_ADDRESS_AND_PORT);
print_spaces(50 - strlen(DFLT_SERVER_ADDRESS_AND_PORT));

if (DFLT_SERVER_SSL_MODE == 0)
	printf("[noSSL]\n");
else
	printf("[SSL]\n");
#endif

// current selection
printf("Server 0: address ptr: %d    SSL mode: ",server0_addr_ptr);
//if (server0_ssl_mode_ptr == 0)
if (server_ssl_mode[server0_addr_ptr] == 0)
	printf("[noSSL]\n");
else
	printf("[SSL]\n");

printf("Server 1: address ptr: %d    SSL mode: ",server1_addr_ptr);
//if (server1_ssl_mode_ptr == 0)
if (server_ssl_mode[server1_addr_ptr] == 0)
	printf("[noSSL]\n");
else
	printf("[SSL]\n");

#else
printf("SIMCOM not used - USE_M4G_MQTT not defined!\n");
#endif

}

unsigned char get_url_port(char* urlport_str, char*url_str, unsigned int* port)
{
unsigned char result = 0;
unsigned char len,i,j,k,portflag;
char tmp_str[10];

tmp_str[0] = 0x00;
j = 0;
k = 0;
portflag = 0;
len = strlen(urlport_str);

for (i=0;i<len;i++)
	{
	if (urlport_str[i] == ':')
		{
		j = i;			// url length
		portflag = 1;
		}

	if (portflag == 0)
		{
		url_str[i] = urlport_str[i];
		j = i+1;
		}
	else
		{
		if (urlport_str[i] != ':')
			tmp_str[k++] = urlport_str[i];		
		}
	}
url_str[j] = 0x00;
tmp_str[k] = 0x00;

*port = atoi(tmp_str);
printf("j: %d  %s  %s  %s  %d\n",j,urlport_str,url_str,tmp_str,*port);

return result;
}



////////////////////////////////////
unsigned char nvs_lora_reg_var_init(void)
{
#ifdef LORA_USED
unsigned char var = 0xFF;
unsigned int ivar;
unsigned long long int lvar = 0;
unsigned char mac_list[6];
unsigned char i,j;
unsigned int n;
esp_err_t err;
char tmpstr[15] = {0x0A,0x00};
	
nvs_handle nvs_handle;

// GATEWAY VARIABLE SPACE (max 15 chars!)

	printf("*** setting NVS LORA REG values...\n");

#if 1	
	err = nvs_open("LORA_REG_LIST",NVS_READWRITE, &nvs_handle);
	printf("nvs_try: %d\n",err);

	sprintf(tmpstr,"LORA_REG_%02d",LREG_MDM_CFG1);
	printf("LoRaReg: %s\n",tmpstr);
	nvs_set_u8(nvs_handle,tmpstr,SX1276_init_vals[LREG_MDM_CFG1]);

	sprintf(tmpstr,"LORA_REG_%02d",LREG_MDM_CFG2);
	nvs_set_u8(nvs_handle,tmpstr,SX1276_init_vals[LREG_MDM_CFG2]);

	sprintf(tmpstr,"LORA_REG_%02d",LREG_PA_CFG);
	nvs_set_u8(nvs_handle,tmpstr,SX1276_init_vals[LREG_PA_CFG]);


	nvs_commit(nvs_handle);
nvs_close(nvs_handle);
#endif

#endif
return 0;
}

unsigned char set_lora_reg_vars_from_nvs(void)
{
#ifdef LORA_USED
	// NOT NEEDED as vars set from NVS in SXC1276_init()
unsigned char var = 0xFF;
unsigned int ivar;
unsigned long long int lvar = 0;
unsigned char mac_list[6];
unsigned char i,j,k;
unsigned int n;
esp_err_t err;
char tmpstr[15] = {0x0A,0x00};
	
nvs_handle nvs_handle;

// GATEWAY VARIABLE SPACE (max 15 chars!)
	
	printf("*** getting LORA REG values from NVS...\n");

//#ifdef USE_M4G_MQTT
err = nvs_open("LORA_REG_LIST",NVS_READWRITE, &nvs_handle);
printf("nvs_try: %d\n",err);


	sprintf(tmpstr,"LORA_REG_%02d",LREG_MDM_CFG1);
	nvs_get_u8(nvs_handle,tmpstr,(uint8_t*)&i);
	sprintf(tmpstr,"LORA_REG_%02d",LREG_MDM_CFG2);
	nvs_get_u8(nvs_handle,tmpstr,(uint8_t*)&j);
	sprintf(tmpstr,"LORA_REG_%02d",LREG_PA_CFG);
	nvs_get_u8(nvs_handle,tmpstr,(uint8_t*)&k);


nvs_close(nvs_handle);


//#endif

#endif
return 0;
}

/*
unsigned char nvs_wifi_var_init(void)
{
esp_err_t err;	
nvs_handle nvs_handle;

err = nvs_open("WIFI_PRM_LIST",NVS_READWRITE, &nvs_handle);

nvs_set_u8(nvs_handle,"WIFI_STA_AUTH",WIFI_AUTH_WPA2_PSK);	// WPA2_PSK
nvs_set_u8(nvs_handle,"WIFI_AP_AUTH",WIFI_AUTH_WPA2_PSK);	// WPA2_PSK

nvs_commit(nvs_handle);
nvs_close(nvs_handle);					

return 0;
}

unsigned char set_wifi_vars_from_nvs(void)
{
	// NOT NEEDED as vars set from NVS in SXC1276_init()
esp_err_t err;	
nvs_handle nvs_handle;
unsigned char i;

err = nvs_open("WIFI_PRM_LIST",NVS_READWRITE, &nvs_handle);

nvs_get_u8(nvs_handle,"WIFI_STA_AUTH",(uint8_t*)&wifi_sta_auth);	// WPA2_PSK
nvs_get_u8(nvs_handle,"WIFI_AP_AUTH", (uint8_t*)&wifi_ap_auth);	// WPA2_PSK

nvs_close(nvs_handle);					
return 0;
}
*/

/*

								err = nvs_open("WIFI_PRM_LIST",NVS_READWRITE, &nvs_handle);
								nvs_set_u8(nvs_handle,"WIFI_STA_AUTH",val);
								nvs_commit(nvs_handle);
								nvs_close(nvs_handle);					
								err = nvs_open("WIFI_PRM_LIST",NVS_READWRITE, &nvs_handle);
								nvs_set_u8(nvs_handle,"WIFI_AP_AUTH",val);
								nvs_commit(nvs_handle);
								nvs_close(nvs_handle);					
*/

////////////////////////////////////

unsigned char nvs_set_addr(nvs_handle handle, char* str, unsigned char inx,unsigned char* addr, unsigned char addr_len)
{
char tmp_str[17];
unsigned char i,ret;
unsigned long long int lvar = 0;

ret = 0;
sprintf(tmp_str,"%s_%03d",str,inx);
														
for (i=0;i<addr_len;i++)
	{
	lvar = (lvar*256) + addr[addr_len-1-i];
	}
		
nvs_set_u64(handle,tmp_str,lvar);
nvs_commit(handle);
	
return ret;
}

unsigned char nvs_wr_u8(char* area, char *id_str, unsigned char inx, unsigned char val)
{
// If inx == 0xFF writes u8 in form "ID_STR" 
// If inx <  0xFF writes u8 in form "ID_STR_inx" 
//	
unsigned char ret;
char tmp_str[17];

nvs_handle nvs_handle;

ret = 0;

if (inx == NO_INX)	// 0xFF
	strcpy(tmp_str,id_str);

else
	sprintf(tmp_str,"%s_%03d",id_str,inx);

nvs_open(area, NVS_READWRITE, &nvs_handle);
nvs_set_u8(nvs_handle, tmp_str, val);
nvs_close(nvs_handle);

printf("%s: Written %s = %d\n",id_str,tmp_str,val);

return ret;
}

unsigned int evt_time(unsigned char day, unsigned char hrs, unsigned char mins)
{
unsigned int result;

result = (((day * 24) + hrs) * 60) + mins;

return result;
}

unsigned char timer_active(unsigned char day, unsigned char hrs, unsigned char mins, unsigned char day_1, unsigned char hrs_1, unsigned char mins_1,unsigned char day_2, unsigned char hrs_2, unsigned char mins_2)
{
unsigned char ret = 0;
unsigned int curr_time, time_1, time_2;

if (day < 7)
	curr_time = (((day * 24) + hrs) * 60) + mins;
else
	curr_time = (((day_of_week * 24) + hrs) * 60) + mins;	// ie "today" - so active DAILY...

time_1 = (((day_1 * 24) + hrs_1) * 60) + mins_1;
time_2 = (((day_2 * 24) + hrs_2) * 60) + mins_2;

if (day_1 < day)		// wrap-around into next week...
	time_1 = time_1 + (7 * 24 * 60);

if (day_2 < day_1)		// wrap-around into next week...
	time_2 = time_2 + (7 * 24 * 60);
	
if ((curr_time >= time_1) && (curr_time < time_2))
		ret = 1;

printf("%d < % d < %d  ?? = %d\n",time_1,curr_time,time_2,ret);
return ret;
}

esp_err_t check_i2c_devices_present(void)
{
esp_err_t err = ESP_OK;
unsigned char i = 0;
unsigned char data;

#ifdef USE_I2C_AMBIMATE4
			if (check_i2c_device(0x2A,0x00) == ESP_OK)
				{
				printf("I2C present @ 0x2A: AmbiMate4\n");
//				printf("Val: %04X %04X %04X %04X %04X %04X %04X\n",i2c_temp,i2c_humid,i2c_light,i2c_audio,i2c_batt,i2c_co2,i2c_voc);
				i = 1;
				}
#endif

#ifdef USE_I2C_PIMORONI
			if (check_i2c_device(0x76,0x00) == ESP_OK)
				{
				i2c_master_read_slave(I2C_MASTER_NUM,0x76,0xD0,&data,1);
				printf("@0x76, Reg D0 = %02X\n",data);
				if (data == 0x61)
					{
// BME688: gas, pressure, temperature & humidity
// I2C addr 76 \ 77: reg D0 = 0x61 (id)
					printf("I2C present @ 0x76: Pimoroni BME688\n");
//				printf("Val: %04X %04X %04X %04X %04X %04X %04X\n",i2c_temp,i2c_humid,i2c_light,i2c_audio,i2c_batt,i2c_co2,i2c_voc);
					i = 1;
					}

				if (data == 0x60)
					{
// BME280: temp, pressure, humidity
// I2C addr 76 \ 77: reg D0 = 0x69 (id)
					printf("I2C present @ 0x76: Pimoroni BME280\n");
//				printf("Val: %04X %04X %04X %04X %04X %04X %04X\n",i2c_temp,i2c_humid,i2c_light,i2c_audio,i2c_batt,i2c_co2,i2c_voc);
					i = 1;
					}
					
				}


// PIM376/LSM303D: BDOF Motion Sensor
			if (check_i2c_device(0x1D,0x00) == ESP_OK)
				{
				printf("I2C present @ 0x1D: Pimoroni LSM303D\n");
//				printf("Val: %04X %04X %04X %04X %04X %04X %04X\n",i2c_temp,i2c_humid,i2c_light,i2c_audio,i2c_batt,i2c_co2,i2c_voc);
				i = 1;
				}

// PIM413/LTR559: Light & proximity sensor
			if (check_i2c_device(0x23,0x00) == ESP_OK)
				{
				printf("I2C present @ 0x23: Pimoroni LTR559\n");
//				printf("Val: %04X %04X %04X %04X %04X %04X %04X\n",i2c_temp,i2c_humid,i2c_light,i2c_audio,i2c_batt,i2c_co2,i2c_voc);
				i = 1;
				}

#endif

if (i == 0)
	{
	err = ESP_FAIL;
	printf("No I2C devices found\n");
	}

printf("\n");	

return err;
}


void gpio_set(gpio_num_t gpio_num,char * namestr, gpio_mode_t mode, unsigned char val, unsigned char dbg)
	{
	if (dbg & GPIO_REPORT)	//0x02)
		{
		if (dbg & GPIO_CSV)	//04)
			printf("GPIO, %s, number, %d, set to mode, ",namestr,gpio_num); 
		else
			printf("GPIO %-20s number %2d set to mode ",namestr,gpio_num); 
		}
	if (dbg & GPIO_SET)	//0x01)
		{
		gpio_pad_select_gpio(gpio_num);
//		gpio_reset_pin(gpio_num);
		gpio_set_direction(gpio_num,mode);
//		gpio_pulldown_dis(gpio_num);
//		gpio_pullup_en(gpio_num);
		}
	if (mode == GPIO_MODE_OUTPUT)
		{
		if (dbg & GPIO_SET)	//0x01)
			gpio_set_level(gpio_num,val&0x01);
		if (dbg & GPIO_REPORT)	//0x02)
			{
			if (dbg & GPIO_CSV)	//04)
//				printf ("OUTPUT, initial=, %d, current=, %d",val&0x01, gpio_get(gpio_num));	// OUTPUT always returns 0!
				printf ("OUTPUT, initial value, %d",val&0x01);
			else
//				printf ("OUTPUT, initial= %d current= %d",val&0x01, gpio_get(gpio_num));	// OUTPUT always returns 0!
				printf ("OUTPUT initial value %d",val&0x01);
			}				
		}
	else
		{
		if (dbg & GPIO_REPORT)	//0x02)
			{
			if (dbg & GPIO_CSV)	//04)
				printf (" INPUT, current value, %d", gpio_get_level(gpio_num));
			else
				printf (" INPUT current value %d", gpio_get_level(gpio_num));
			}
		}
	if (dbg & GPIO_REPORT)	//0x02)
		printf("\n");
	}

void gpio_init(unsigned char dbg)
{
// replaces all previous IO definitions
// uses
// #define GPIO_SET				1	// set up all GPIOs
// #define GPIO_REPORT			2	// report setup of all GPIOs
// #define GPIO_SET_AND_REPORT	3	// set up and report state
// #define GPIO_CSV				4	// sene comma separated variable string of setup to console output

#if defined (SERIAL2_USED)
	gpio_set(TXD1,"TXD1",GPIO_MODE_OUTPUT,1,dbg);
	gpio_set(RXD1,"RXD1",GPIO_MODE_INPUT,0,dbg);
#endif

#ifdef RTS1
	gpio_set(RTS1,"RTS1",GPIO_MODE_INPUT,0,dbg);
#endif
#ifdef CTS1
	gpio_set(CTS1,"CTS1",GPIO_MODE_INPUT,0,dbg);		// CHECK!
#endif

	gpio_set(TXD2,"TXD2",GPIO_MODE_OUTPUT,1,dbg);		// TO SIMCOM RXD IN
	gpio_set(RXD2,"RXD2",GPIO_MODE_INPUT,0,dbg);		// TO SIMCOM TXD OUT

#ifdef RTS2
	gpio_set(RTS2,"RTS2",GPIO_MODE_OUTPUT,0,dbg);		// TO SIMCOM CTS IN ACTIVE LOW
#endif

	gpio_set(CTS2,"CTS2",GPIO_MODE_INPUT,0,dbg);		// TO SIMCOM RTS OUT ACTIVE LOW

   	gpio_set(DTR2,"DTR2",GPIO_MODE_OUTPUT,0,dbg);		// TO SIMCOM DTR IN  ACTIVE LOW

#ifdef DCD2
	gpio_set(DCD2,"DCD2",GPIO_MODE_INPUT,0,dbg);		// TO SIMCOM DCD OUT ACTIVE LOW
#endif

#ifdef RI2
	gpio_set(RI2,"RI2",GPIO_MODE_INPUT,0,dbg);			// TO SIMCOM RI OUT ACTIVE LOW
#endif

	gpio_set(STATUS,"STATUS",GPIO_MODE_INPUT,0,dbg);

#if defined(USE_I2C) || defined(TEST_I2C)
	gpio_set(SDA,"SDA",GPIO_MODE_INPUT,0,dbg);
	gpio_set(SCL,"SCL",GPIO_MODE_INPUT,0,dbg);
#endif



// pins may be on shift reg in this PCB version!
#ifdef LORA_RADIO_RESET
	gpio_set(LORA_RADIO_RESET,"LORA_RADIO_RESET",GPIO_MODE_OUTPUT,1,dbg);
#endif
#ifdef FOUR_G_PSU_EN
	gpio_set(FOUR_G_PSU_EN,"FOUR_G_PSU_EN",GPIO_MODE_OUTPUT,0,dbg);
#endif
#ifdef FOUR_G_RESET
	gpio_set(FOUR_G_RESET,"FOUR_G_RESET",GPIO_MODE_OUTPUT,0,dbg);
#endif
#ifdef FOUR_G_POWER
	gpio_set(FOUR_G_POWER,"FOUR_G_POWER",GPIO_MODE_OUTPUT,0,dbg);
#endif


	gpio_set(LORA_MISO,"LORA_MISO",GPIO_MODE_INPUT,0,dbg);

	gpio_set(LORA_MOSI,"LORA_MOSI",GPIO_MODE_OUTPUT,0,dbg);

	gpio_set(LORA_SCK,"LORA_SCK",GPIO_MODE_OUTPUT,0,dbg);

	gpio_set(LORA_NSS,"LORA_NSS",GPIO_MODE_OUTPUT,1,dbg);

	gpio_set_level(LORA_MOSI,0);
	gpio_set_level(LORA_SCK,0);
	gpio_set_level(LORA_NSS,1);

#ifdef LORA_USED
	gpio_set(LORA_DIO0,"LORA_DIO0",GPIO_MODE_INPUT,0,dbg);

#ifdef LORA_USED
	gpio_set(LORA_DIO1,"LORA_DIO1",GPIO_MODE_INPUT,0,dbg);
	gpio_set(LORA_DIO2,"LORA_DIO2",GPIO_MODE_INPUT,0,dbg);
#endif

#ifdef LORA_DIO3
	gpio_set(LORA_DIO3,"LORA_DIO3",GPIO_MODE_INPUT,0,dbg);
#endif
#ifdef LORA_DIO4
	gpio_set(LORA_DIO4,"LORA_DIO4",GPIO_MODE_INPUT,0,dbg);
#endif

#endif



// LEDs may be on shift reg in this PCB version!
#ifdef LED_0
	gpio_set(LED_0,"LED_0",GPIO_MODE_OUTPUT,0,dbg);
#endif
#ifdef LED_1
	gpio_set(LED_1,"LED_1",GPIO_MODE_OUTPUT,0,dbg);
#endif
#ifdef LED_2
	gpio_set(LED_2,"LED_2",GPIO_MODE_OUTPUT,0,dbg);
#endif
#ifdef LED_3
	gpio_set(LED_3,"LED_3",GPIO_MODE_OUTPUT,0,dbg);
#endif
	

#ifdef SR_DATA
	gpio_set(SR_DATA,"SR_DATA",GPIO_MODE_OUTPUT,0,dbg);
#endif
#ifdef SR_CLK
	gpio_set(SR_CLK,"SR_CLK",GPIO_MODE_OUTPUT,0,dbg);
#endif
#ifdef SR_STB
	gpio_set(SR_STB,"SR_STB",GPIO_MODE_OUTPUT,0,dbg);
#endif


#ifdef USE_TANK_SENSORS
	gpio_set(TANKSENSOR0,"TANKSENSOR0",GPIO_MODE_INPUT,0,dbg);
//#ifdef TANKSENSOR_1_ON_GPIO39
	gpio_set(TANKSENSOR1,"TANKSENSOR1",GPIO_MODE_INPUT,0,dbg);
//#endif
	gpio_set(TSENSCTRL,"TSENSCTRL",GPIO_MODE_OUTPUT,0,dbg);
#endif
#ifdef USE_ULTRASONIC_SENSOR
	gpio_set(USM_ECHO,"USM_ECHO",GPIO_MODE_INPUT,0,dbg);
	gpio_set(USM_TRIG,"USM_TRIG",GPIO_MODE_OUTPUT,0,dbg);
	gpio_set(USM_SEL,"USM_SEL",GPIO_MODE_OUTPUT,0,dbg);


	gpio_set(USM_UV_TRIG,"USM_UV_TRIG",GPIO_MODE_OUTPUT,0,dbg);

#endif

#ifdef USE_ADCS
	gpio_set(ADC_0,"ADC0",GPIO_MODE_INPUT,0,dbg);
	gpio_set(ADC_1,"ADC1",GPIO_MODE_INPUT,0,dbg);

#endif


#ifdef ZULU_RADIO_MASTER
	gpio_set(ZULU_RADIO_MASTER_RTS,"ZULU_RADIO_MASTER_RTS",GPIO_MODE_OUTPUT,0,dbg);
	gpio_set(ZULU_RADIO_MASTER_CTS,"ZULU_RADIO_MASTER_CTS",GPIO_MODE_INPUT,0,dbg);


#endif

#ifdef ZULU_RADIO_SLAVE
	gpio_set(ZULU_RADIO_SLAVE_RTS,"ZULU_RADIO_SLAVE_RTS",GPIO_MODE_OUTPUT,0,dbg);
	gpio_set(ZULU_RADIO_SLAVE_CTS,"ZULU_RADIO_SLAVE_CTS",GPIO_MODE_INPUT,0,dbg);
#endif


#ifdef SW0
	gpio_set(SW0,"SW0",GPIO_MODE_INPUT,0,dbg);
#endif
#ifdef SW1
	gpio_set(SW1,"SW1",GPIO_MODE_INPUT,0,dbg);
#endif
#ifdef SW2
	gpio_set(SW2,"SW2",GPIO_MODE_INPUT,0,dbg);
#endif
	
	
}

void print_spaces(unsigned char num)
{
unsigned char i;
for (i=0;i<num;i++)
	{
	putchar(' ');
	}
}

void* malloc_chk(size_t size, unsigned int* total)
{
*total = *total + size;

return malloc(size);
}

void* calloc_chk(size_t num, size_t size, unsigned int* total)
{
*total = *total + size;

return calloc(num,size);
}

