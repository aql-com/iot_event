//////////////////////////////////////////////
//
// MAC_Rpt.h
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

#ifndef MAC_RPT_H
#define MAC_RPT_H

#include "driver/gpio.h"
#include "driver/uart.h"

// software version number:
#define VER_MAJ				 1
#define VER_MIN				 5
#define VER_SUB				 43	// 

//#define USE_WIFI_UDP
//#define WIFI_UDP_DNS_TIMEOUT		100	// time to refresh DNS lookup
//#define WIFI_UDP_TEST		

// ### SIMCOM Tx \ Rx debug defines
//#define DBG_TEST
//#define STACK_TEST
//#define SET_PAYLOAD_TIME_TEST
// ### SIMCOM UART_intr debug defines
//#define QINT_DEBUG
//#define QINT_DEBUG2
//#define QINT_DEBUG3
//#define NEW_SERIAL_DBG1		// program labels
//#define NEW_SERIAL_DBG2		// show data
//#define NEW_SERIAL_DBG3		// show outgoing to fg

// used in project setup files to define board type...
#define VER_1_0_A				1
#define VER_1_0_B				2
#define VER_1_0_C				3
#define VER_1_0_D				4
#define VER_LIFT_1_0_A			5
#define VER_BTS_1_0_C			6
#define VER_LIFT_1_0_B			7
#define VER_BTS_1_0_D			8

#define VER_10A_STR				"VER_1_0_A"
#define VER_10B_STR				"VER_1_0_B"
#define VER_10C_STR				"VER_1_0_C"
#define VER_10D_STR				"VER_1_0_D"
#define VER_LIFT_10A_STR		"LIFT VER 1_0_A"
#define VER_BTS_1_0_C_STR		"BT SENSOR 1_0_C"
#define VER_LIFT_10B_STR		"LIFT VER 1_0_B"
#define VER_BTS_1_0_D_STR		"BT SENSOR 1_0_D"

// used to define device polarities
#define NOT_INVERTED		0
#define INVERTED			1

///////////////////////////////////////////////////////////
//
// INCLUSION OF CONFIG.H
//
///////////////////////////////////////////////////////////

// current project .h file is declared in config.h, (or NONE if using default settings)
// The selected PROJECT def file in config.h should define USING PROJECT_SETTINGS
// if this is not defined, then use default settings from this h file
#include "config.h"		// contains correct .h file for this compile variant...

//#define USE_WIFI_MQTT		// FOR TEST!!!

//#define USE_M4G_UDP		// FOR TEST!!!

// following now applies to all project related configs
#if 0
#define NON_SSL_SERVER_ADDRESS_AND_PORT				"tcp://iot-visualiser.aql.com:1883"		// must be of format <tcp:\\addr.ext:port>
#define SSL_SERVER_ADDRESS_AND_PORT					"tcp://iot-visualiser.aql.com:8883"		// must be of format <tcp:\\addr.ext:port>

//#define USE_SERVER_ADDRESS_AND_PORT_LIST
#define DFLT_NON_SSL_SERVER_ADDRESS_AND_PORT_0		"tcp://iot-visualiser.aql.com:1883"		// must be of format <tcp:\\addr.ext:port>
#define DFLT_SSL_SERVER_ADDRESS_AND_PORT_0			"tcp://iot-visualiser.aql.com:8883"		// must be of format <tcp:\\addr.ext:port>

#define DFLT_NON_SSL_SERVER_ADDRESS_AND_PORT_1		"tcp://iot-visualiser.aql.com:1883"		// must be of format <tcp:\\addr.ext:port>
#define DFLT_SSL_SERVER_ADDRESS_AND_PORT_1			"tcp://iot-visualiser.aql.com:8883"		// must be of format <tcp:\\addr.ext:port>

#define DFLT_NON_SSL_SERVER_ADDRESS_AND_PORT_2		"tcp://iot-visualiser.aql.com:1883"		// must be of format <tcp:\\addr.ext:port>
#define DFLT_SSL_SERVER_ADDRESS_AND_PORT_2			"tcp://iot-visualiser.aql.com:8883"		// must be of format <tcp:\\addr.ext:port>

#define DFLT_NON_SSL_SERVER_ADDRESS_AND_PORT_3		"tcp://iot-visualiser.aql.com:1883"		// must be of format <tcp:\\addr.ext:port>
#define DFLT_SSL_SERVER_ADDRESS_AND_PORT_3			"tcp://iot-visualiser.aql.com:8883"		// must be of format <tcp:\\addr.ext:port>
#endif

#if 0	// addresses now held in each jopb file!

// GW ID for LoRa:
// 00800000a000220e
// following addresses must be of format <tcp:\\addr.ext:port>
#define SERVER_ADDRESS_AND_PORT_0		"tcp://iot-visualiser.aql.com:8883"		// main server:
#define SERVER_ADDRESS_AND_PORT_1		"tcp://109.239.102.45:1700"				// Lora server:
#define SERVER_ADDRESS_AND_PORT_2		"tcp://iot-visualiser.aql.com:2883"		// 
#define SERVER_ADDRESS_AND_PORT_3		"tcp://iot-visualiser.aql.com:3883"		// 

#define DFLT_SERVER_ADDRESS_AND_PORT	"tcp://iot-visualiser.aql.com:9883"		// 
#endif

//#define USE_BLUETOOTH_CLASSIC_DETECT
//#define BLUETOOTH_CLASSIC_SCAN_TIME		60	// number of sec between scans
//#define BLUETOOTH_CLASSIC_SCAN_LENGTH	10	// number of sec * 1.28 for maximum scan time



const char days_of_week[8][4] = 
{
"Sun",
"Mon",
"Tue",
"Wed",
"Thu",
"Fri",
"Sat",
"ALL"
};
	

///////////////////////////////////////////////////////////////
//
// default configuration if no project.h files used...
//
///////////////////////////////////////////////////////////////
#ifndef USING_PROJECT_SETTINGS
// if this isnt defined, then use default settings from this h file
#warning "USING DEFAULT BUILD SETTINGS FROM MAC_Rpt.h"

//#define USE_DEV_PCB

//#define SIMCOM_PROGRAMMING_MODE	// just power up SIMCOM module and then wait for programming to occur...
//#define TEST_I2C
#define USE_I2C					// I2C connected sensors on GPIs...	
#define USE_I2C_AMBIMATE4		// use the AmbiMate4 sensor board on I2C bus
#define USE_I2C_PIMORONI		// use the Pimoroni sensor selection on I2C bus

#define USE_M4G					// 4G comms
//#define USE_M4G_HTTP			// 4G comms over HTTP
#define USE_M4G_MQTT			// 4G comms over MQTT
#define USE_M4G_MQTT_SSL

#define AQL_SERVER
//#define SSS_SERVER
//#define MOSQUITTO_SERVER

#define USE_BLUETOOTH
#define BLUETOOTH_SCAN
#define USE_BLUETOOTH_ADVERTISING	// for Bluetooth sensor mode

//#define USE_BLUETOOTH_MAC_FILTERING	// now redundant due to white \ black listing facilities?

#define USE_LORA
//#define LORAWAN_SCAN

//#define WIFI_SCAN
//#define USE_WEBSERVER
//#define CONFIG_EXAMPLE_BASIC_AUTH
//#define CONFIG_EXAMPLE_CONNECT_WIFI

//#define USE_SWITCHES

//#define USE_TANK_SENSORS
//#define TANKSENSOR_1_ON_GPIO39

//#define USE_ULTRASONIC_SENSOR

//#define USE_ZULU_RADIO			// use RF Solutions Zulu radio modems

#if 1
// for serial
//#define USE_EXT_SERIAL
#define USE_NMEA								// fudge for now!
#define SERIAL_PUBLISH_TOPIC	"serial"
#define DFLT_EXT_SERIAL_BPS		115200
#define DFLT_EOLSTR				0x0A
#endif

#if 0
// for NMEA (special case of EXT_SERIAL)
//#define USE_NMEA
#define SERIAL_PUBLISH_TOPIC	"NMEA"
#define DFLT_EXT_SERIAL_BPS		115200	//	4800
#define DFLT_EOLSTR				0x0A
#define USE_NMEA_4800_BPS
#endif



////////////////////////////////////////////////
// temporary defines for development
/////////////////////////////////////////////////
//#define USE_CPU_SLEEP
#define SHOW_SLEEP_MSGS
//#define USE_VTASKLIST

//
////////////////////////
// set target PCB 
////////////////////////
#define V1_PCB
//#define V2_PCB

#define PCB_VER			VER_1_0_D

#define HW_MAJ				 1
#define HW_MIN				 0
#define HW_SUB				'D'


// GPIO pin defines
////////////////////////
// V1 PCB 
////////////////////////
//#ifdef V1_PCB
#if PCB_VER == VER_1_0_C
#define TXD0				(gpio_num_t)1				// OUT
#define RXD0				(gpio_num_t)3				// IN
//#define RTS1				(gpio_num_t)25				// 
//#define CTS1				(gpio_num_t)26				// 
#define SDA				 	(gpio_num_t)25				// 
#define SCL				 	(gpio_num_t)26				// 
#ifdef USE_NMEA
#define TXD1				(gpio_num_t)33	//17		// OUT
#define RXD1				(gpio_num_t)36	//16		// IN
#else
#define TXD1				(gpio_num_t)12	//17		// OUT
#define RXD1				(gpio_num_t)14	//16		// IN
#endif
#define TXD2				(gpio_num_t)23				// OUT
#define RXD2				(gpio_num_t)22				// IN
#define RTS2				(gpio_num_t)21				// 
#define CTS2				(gpio_num_t)19				// 
#define DTR2				(gpio_num_t)2				// 
#define DCD2				(gpio_num_t)15				// 
#define LORA_RADIO_RESET			(gpio_num_t)27				// OUT
#define FOUR_G_RESET		(gpio_num_t)32				// OUT
#define FOUR_G_POWER		(gpio_num_t)18				// OUT
#define LED_0				(gpio_num_t)5				// OUT
#define LED_1				(gpio_num_t)17				// OUT
#define LED_2				(gpio_num_t)16				// OUT
#define LED_3				(gpio_num_t)4				// OUT
#define VMON				(gpio_num_t)35				// OUT

#define ADC_VMON			ADC1_CHANNEL_7
#define ADC_TANKSENSOR		ADC1_CHANNEL_3

#define STATUS 				(gpio_num_t)34				// IN

#define SW0 				(gpio_num_t)39				// IN
#define SW1 				(gpio_num_t)36				// IN
#define SW2 				(gpio_num_t)33				// IN

#ifdef USE_TANK_SENSORS
#define TANKSENSOR0			(gpio_num_t)39				// IN
#define TANKSENSOR1			(gpio_num_t)36				// IN
#define TSENSCTRL			(gpio_num_t)33				// IN
#endif

#endif
 
#if PCB_VER == VER_1_0_D
#if defined(USE_ULTRASONIC_SENSOR) || defined(TANKSENSOR_1_ON_GPIO39) || defined (USE_NMEA)
#define PORTB_USED
#endif
#if ! defined(USE_ULTRASONIC_SESNSOR) && !defined(TANKSENSOR_1_ON_GPIO39) && defined (USE_TANK_SENSORS)
#define TANK_SENSOR_USED
#endif
#if ! defined(USE_ULTRASONIC_SENSOR) && !defined(TANKSENSOR_1_ON_GPIO39) && defined (USE_NMEA)
#define SERIAL2_USED
#endif
#if ! defined(USE_ULTRASONIC_SENSOR) && !defined(TANKSENSOR_1_ON_GPIO39) && defined (USE_LORA)
#define LORA_USED
#endif

#define TXD0				 (gpio_num_t)1				// OUT
#define RXD0				 (gpio_num_t)3				// IN
//#define RTS1				 (gpio_num_t)25				// 
//#define CTS1				 (gpio_num_t)26				// 

#ifdef USE_I2C
#define SDA				 	(gpio_num_t)25				// 
#define SCL				 	(gpio_num_t)26				// 
#endif

#ifdef TEST_I2C
#define SDA				 	(gpio_num_t)25				// 
#define SCL				 	(gpio_num_t)26				// 
#endif

//#ifdef USE_NMEA
//#define TXD1				(gpio_num_t)18	//17		// OUT	BETTER IF WE USE 27! with below (frees up 18)...
//#define RXD1				(gpio_num_t)27	//16		// IN	BETTER IF WE USE 39! its IN only...

#ifdef SERIAL2_USED
// J22 - 5 pin PH-K next to USB connector
#define TXD1				(gpio_num_t)27	//17		// OUT	GPIO27 = J22 pin 3	BETTER IF WE USE 27! with below (frees up 18)...
#define RXD1				(gpio_num_t)39	//16		// IN	GPI39  = J22 pin 2	BETTER IF WE USE 39! its IN only...
#endif
#define SPARE1				(gpio_num_t)18	//17		// OUT	GPIO18 = J22 pin 4	(free)

#define TXD2				(gpio_num_t)23				// OUT
#define RXD2				(gpio_num_t)22				// IN
#define RTS2				 (gpio_num_t)21				// 
#define CTS2				 (gpio_num_t)19				// 
#define DTR2				 (gpio_num_t)2				// 
#define DCD2				 (gpio_num_t)16				// 
//#define LORA_RADIO_RESET			(gpio_num_t)27				// OUT
//#define FOUR_G_RESET		(gpio_num_t)32				// OUT
//#define FOUR_G_POWER		(gpio_num_t)18				// OUT
//#define LED_0				(gpio_num_t)5				// OUT
//#define LED_1				(gpio_num_t)17				// OUT
//#define LED_2				(gpio_num_t)16				// OUT
//#define LED_3				(gpio_num_t)4				// OUT
#define VMON				(gpio_num_t)35				// OUT

#define ADC_VMON			ADC1_CHANNEL_7
#define ADC_VMON2			ADC1_CHANNEL_4
#define ADC_TANKSENSOR0		ADC1_CHANNEL_5
#ifdef TANKSENSOR_1_ON_GPIO39
#define ADC_TANKSENSOR1		ADC1_CHANNEL_3
#else
#define ADC_TANKSENSOR1		ADC2_CHANNEL_9
#endif

#define STATUS 				(gpio_num_t)34				// IN

#ifdef TANK_SENSOR_USED
// J10 - 5 pin PH-K next to DC socket
#define PORTB_USED
#define TANKSENSOR0			(gpio_num_t)33 	// 25				// IN
#ifdef TANKSENSOR_1_ON_GPIO39
#define TANKSENSOR1			(gpio_num_t)39				// IN
#else
#define TANKSENSOR1			(gpio_num_t)26				// IN
#endif
#define TSENSCTRL			(gpio_num_t)25	// 33				// IN
#endif

#ifdef USE_ULTRASONIC_SENSOR
#define PORTB_USED
#define USM_ECHO			(gpio_num_t)39				// IN
#define USM_SEL				(gpio_num_t)27				// OUT
#define USM_TRIG			(gpio_num_t)18	//17		// OUT	

#define USM_LIFT_SW			(gpio_num_t)25				// IN
#define USM_UV_TRIG			(gpio_num_t)26				// OUT

#endif

#ifndef PORTB_USED
#define SW0 				(gpio_num_t)25				// IN
#define SW1 				(gpio_num_t)26				// IN
#define SW2 				(gpio_num_t)33				// IN
#endif

#define LORA_MISO			(gpio_num_t)13				// IN
#define LORA_MOSI			(gpio_num_t)12				// OUT
#define LORA_SCK			(gpio_num_t)14				// OUT
#define LORA_NSS			(gpio_num_t)15				// OUT

#define LORA_DIO0			(gpio_num_t)36				// IN

#ifdef LORA_USED
#define LORA_DIO1			(gpio_num_t)39				// IN
#define LORA_DIO2			(gpio_num_t)27				// IN
#endif
#define LORA_DIO3			(gpio_num_t)4				// IN
#define LORA_DIO4			(gpio_num_t)32				// IN

#define VMON_2				(gpio_num_t)32				// IN

#define SR_DATA				(gpio_num_t)4				// IN
#define SR_CLK				(gpio_num_t)5				// IN
#define SR_STB				(gpio_num_t)0				// IN


#endif


#endif 	//end of "#ifndef USING PROJECT_SETTINGS
///////////////////////////////////////////////////////////////
//
// end of default configuration if no project.h files used
//
///////////////////////////////////////////////////////////////


#if PCB_VER	== VER_1_0_A
	#define PCB_VER_STR		VER_10A_STR
#elif  PCB_VER == VER_1_0_B
	#define PCB_VER_STR		VER_10B_STR
#elif PCB_VER	== VER_1_0_C
	#define PCB_VER_STR		VER_10C_STR
#elif  PCB_VER == VER_1_0_D
	#define PCB_VER_STR		VER_10D_STR
	#include "PinDefs/V1_0_D_Pins.h"
#elif  PCB_VER == VER_LIFT_1_0_A
	#define PCB_VER_STR		VER_LIFT_10A_STR
	#include "PinDefs/V_LIFT_1_0_A_Pins.h"
#elif  PCB_VER == VER_BTS_1_0_C
	#define PCB_VER_STR		VER_BTS_1_0_C_STR
	#include "PinDefs/V_BTS_1_0_D_Pins.h"
#elif  PCB_VER == VER_LIFT_1_0_B
	#define PCB_VER_STR		VER_LIFT_10B_STR
	#include "PinDefs/V_LIFT_1_0_B_Pins.h"
#endif


#define HEARTBEAT_LED		0
#define NETWORK_LED			1


////////////////////////
// V2 PCB 
////////////////////////
#ifdef V2_PCB

#endif

// I2C defs
#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */

#define I2C_SLAVE_SCL_IO 			GPIO_NUM_26	//33	//CONFIG_I2C_SLAVE_SCL      32k_XN         /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO 			GPIO_NUM_25	//32	//CONFIG_I2C_SLAVE_SDA      32k_XP         /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM 				I2C_NUM_0	//I2C_NUMBER(CONFIG_I2C_SLAVE_PORT_NUM) /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN 		(2 * DATA_LENGTH)              /*!< I2C slave tx buffer size */
#define I2C_SLAVE_RX_BUF_LEN 		(2 * DATA_LENGTH)              /*!< I2C slave rx buffer size */

#define I2C_SLAVE_ADDR				0x40

#define I2C_MASTER_SCL_IO 			GPIO_NUM_26	//33	//CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 			GPIO_NUM_25//32	//CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM 				I2C_NUM_0	//I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 			100000	//CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 	0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 	0	                           /*!< I2C master doesn't need buffer */


#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 		(i2c_ack_type_t)0x0     /*!< I2C ack value */
#define NACK_VAL 		(i2c_ack_type_t)0x1    
// end of I2C defs

#define I2C_SENSOR_READY	0
#define I2C_SENSOR_DONE		1


#define DEVICE_OFF			0
#define DEVICE_ON			1

#define FLASH				2
#define FAST_FLASH			3


#define CONFIG_OFF			0
#define CONFIG_ON			1
#define RST_OFF				0
#define RST_ON				1


#define LORA_REG_BUFFER_SIZE	32	// max spi transfer size
#define LORA_RESET_TIME			1	// LORA reset pulse length in 100ms steps

/*
// ### NOW in each project file...!
#define WIFI_STA_PREFIX		"MACRPT-S-"
//#define WIFI_AP_PREFIX		"MACRPT-A-"
#define WIFI_AP_PREFIX		"LIFT-"
#define WIFI_AP_PASSWORD	"password"
*/
// CLI definitions
#define DFLT_FILL_TIME_MAX	600	// 60 sec
#define FILL_SW_COUNT_STOP	30	// hold fill sw for 4 sec for emergency shutoff
#define FILL_SW_COUNT_MAX	60	// hold fill sw for 4 sec for emergency shutoff

#define DFLT_LORA_RADIO_MSG_TIME		9000	// 15 min = 9000 sec
#define DFLT_WIFI_SCAN_TIME		60	// sec
#define DFLT_MASK_TIME			30	// 3 sec

#define DFLT_UART_STAY_AWAKE_TIME	100	// 10 sec

#define ALL_WIFI_SUBTYPES			16
#define WIFI_CONNECTED_BIT 			BIT0
#define WIFI_FAIL_BIT      			BIT1

#define WIFI_CREDENTIALS_MAX		3

// MQTT transport modes
#define MQTT_MODE_NONE				0
#define MQTT_MODE_4G				1
#define MQTT_MODE_WIFI				2
//#define PREFERRED_MQTT_MODE			MQTT_MODE_WIFI	// now in job .h files!

#define UDP_MODE_NONE				0
#define UDP_MODE_4G					1
#define UDP_MODE_WIFI				2

#define MAC_COUNT_RSSI_THRESHOLD	-95	// RSSI thresold in dBs

#define DBG					UART_NUM_0
#define RADIO				UART_NUM_1
#define NMEA				UART_NUM_1
#define FOUR_G				UART_NUM_2
//#define ZULU_RADIO			UART_NUM_2

#define LORA_RADIO_TX_BUFSIZE	256
#define LORA_RADIO_RX_BUFSIZE	1024
#define FOUR_G_TX_BUFSIZE	256
#define FOUR_G_RX_BUFSIZE	10000	// to accept 8192 of rx from a full mqtt payload 

#define ZULU_RADIO_TX_BUFSIZE	80	// ZULU radio can only tx 52 bytes per msg!
#define ZULU_RADIO_RX_BUFSIZE	80	// 

#define DBG_RCV_BUF_SIZE	80

#define STATUS_MSG			0
#define GEOLOCATION_MSG		1
#define TEST_MSG			9

// server state machine states
// CAN alter the order of these...
#define GET_NEW_CERTIFICATES		0
#define GET_NEW_CERTIFICATES_WAIT	1
#define NOT_SRVR_INITIALISED		2
#define SRVR_INIT_WAIT				3
#define NOT_SRVR_CONNECTED			4
#define SRVR_CONNECT_WAIT			5
#define UPDATE_SRVR_INFO			6
#define UPDATE_SRVR_INFO_WAIT		7
#define SEND_CMD_RESPONSE			8
#define SENSOR_DATA_IDLE			9
#define GATEWAY_SENSOR_DATA			10
#define GATEWAY_SENSOR_DATA_WAIT	11
#define NMEA_SENSOR_DATA			12
#define NMEA_SENSOR_DATA_WAIT		13
#define GET_NMEA_WHITELIST_TABLE	14
#define GET_NMEA_BLACKLIST_TABLE	15
#define GET_SERIAL_SETTINGS			16
#define GET_BW_LIST_TABLE_WAIT		17
#define BT_SENSOR_DATA				18
#define BT_MAC_ADDR_DATA			19
#define BT_DEVNAME_DATA				20
#define BT_SENSOR_DATA_WAIT			21
#define GET_BT_WHITELIST_TABLE		22
#define GET_BT_BLACKLIST_TABLE		23
#define LORA_SENSOR_DATA			24
#define LORA_DEVADDR_DATA			25
#define LORA_SENSOR_DATA_WAIT		26
#define GET_LORA_WHITELIST_TABLE	27
#define GET_LORA_BLACKLIST_TABLE	28
#define WIFI_SCAN_DATA				29
#define WIFI_SCAN_DATA_WAIT			30
#define I2C_SENSOR_DATA				31
#define I2C_SENSOR_DATA_WAIT		32
#define TANK_SENSOR_DATA			33
#define TANK_SENSOR_DATA_WAIT		34
#define LIFT_STATUS					35
#define LIFT_EVENT_STATUS			36
#define LIFT_EVENT_MSG				37
#define LIFT_STATUS_WAIT			38
#define POWER_METER_READINGS		39
#define POWER_METER_WAIT			40
#define XXX_DATA					41
#define XXX_WAIT					42
#define RALLY_SAFETY_DATA			43
#define RALLY_SAFETY_WAIT			44
#define NET_4G_REPORT				45
#define NET_4G_REPORT_WAIT			46
#define IDLE_CHECK_SIGNAL_QUALITY	47
#define IDLE_CHECK_SIGNAL_QUAL_WAIT	48
#define GET_GPS_INFO				49
#define GET_GPS_INFO_WAIT			50
#define SEND_GET_TIME_REQUEST		51
#define MQTT_SEND_WAIT				52
#define SRVR_DISCONNECT				53
#define SRVR_DISCONNECT_WAIT		54
#define SEND_RETURN_VALUE			55
#define NUM_MQTT_LOGIN_STATES		56

const char mqtt_login_state_str[NUM_MQTT_LOGIN_STATES][25] = 
{
//234567890123456789012345
"GET_NEW_CERTS",
"GET_NEW_CERTS_WAIT",
"NOT_SRVR_INITIALISED",
"SRVR_INIT_WAIT",
"NOT_SRVR_CONNECTED",
"SRVR_CONNECT_WAIT",
"UPDATE_SRVR_INFO",
"UPDATE_SRVR_INFO_WAIT",
"SEND_CMD_RESPONSE",
"SENSOR_DATA_IDLE",
//234567890123456789012345
"GW_SENSOR_DATA",
"GW_SENSOR_DATA_WAIT",
"NMEA_SENSOR_DATA",
"NMEA_SENSOR_DATA_WAIT",
"GET_NMEA_WHITELIST_TABLE",
"GET_NMEA_BLACKLIST_TABLE",
"GET_SERIAL_SETTINGS",
"GET_BW_LIST_TABLE_WAIT",
"BT_SENSOR_DATA",
"BT_MAC_ADDR_DATA",
//234567890123456789012345
"BT_DEVNAME_DATA",
"BT_SENSOR_DATA_WAIT",
"GET_BT_WHITELIST_TABLE",
"GET_BT_BLACKLIST_TABLE",
"LORA_SENSOR_DATA",
"LORA_DEVADDR_DATA",
"LORA_SENSOR_DATA_WAIT",
"GET_LORA_WHITELIST_TABLE",
"GET_LORA_BLACKLIST_TABLE",
"WIFI_SCAN_DATA",
//234567890123456789012345
"WIFI_SCAN_DATA_WAIT",
"I2C_SENSOR_DATA",
"I2C_SENSOR_DATA_WAIT",
"TANK_SENSOR_DATA",
"TANK_SENSOR_DATA_WAIT",
"LIFT_STATUS",
"LIFT_EVENT_STATUS",
"LIFT_EVENT_MSG",
"LIFT_STATUS_WAIT",
"POWER_METER_READINGS",
//234567890123456789012345
"POWER_METER_WAIT",
"XXX_DATA",
"XXX_WAIT",
"RALLY_SAFETY_DATA",
"RALLY_SAFETY_WAIT",
"NET_4G_REPORT",
"NET_4G_REPORT_WAIT",
"IDLE_CHECK_SIG_QUALITY",
"IDLE_CHECK_SIG_QUAL_WAIT",
"GET_GPS_INFO",
//234567890123456789012345
"GET_GPS_INFO_WAIT",
"SEND_GET_TIME_REQUEST",
"SEND_GET_TIME_REQ_WAIT",
"SRVR_DISCONNECT",
"SRVR_DISCONNECT_WAIT",
"SEND_RETURN_VALUE"
};

// sizes of black and white lists and their data size
#define NMEA_CMD_LIST_LENGTH			   50	// number in list
#define NMEA_CMD_LIST_ENTRY_LENGTH	  	   15	// entry length

#define BLUETOOTH_CMD_LIST_INIT_LENGTH		5
#define BLUETOOTH_CMD_LIST_LENGTH		  100
#define BLUETOOTH_CMD_LIST_ENTRY_LENGTH	    6	//15

#define BLUETOOTH_DEVNAME_LIST_INIT_LENGTH	 5
#define BLUETOOTH_DEVNAME_LIST_LENGTH		10	//10 entries in list
#define BLUETOOTH_DEVNAME_LIST_ENTRY_LENGTH	20	// entry can be up to 19 chars + 0x00

#define LORAWAN_CMD_LIST_INIT_LENGTH	    5
#define LORAWAN_CMD_LIST_LENGTH			  100
#define LORAWAN_CMD_LIST_ENTRY_LENGTH	    4	//15

#define WL_IDLE			0
#define WL_ADD			1
#define WL_REM			2

#define BL_IDLE			0
#define BL_ADD			1
#define BL_REM			2

#define BINARY_VAL		0
#define ASCII_VAL		1

// Bluetooth adverstising packet is 31 chars max. With data payload, only leaves 10 chars for Local Device Name...
// adv time:0x0020 to 0x4000 Default: N = 0x0800 (1.28 second) 
// Time = N * 0.625 msec Time Range: 20 ms to 10.24 sec
#define BLUETOOTH_ADVERTISING_NAME		"AQLTEST_01"	// 10 chars //"ESP_BLE_HELLO"
#define BLUETOOTH_ADVERTISING_MIN_TIME	3200	//256 // 1 sec: 256 = 160ms	1.6 * ms value
#define BLUETOOTH_ADVERTISING_MAX_TIME	3200	//256 // 1 sec: 256 = 160ms	1.6 * ms value
#define BLUETOOTH_ADVERTISING_TIME		3200	//256 // 1 sec: 256 = 160ms	1.6 * ms value

// DONT THINK the order of these is dictated by the hardware - 
// so can re-order into outputs (0-7) then inputs (8-15)
// Helps with set_output() and get_input() routines..
//#ifdef V1_PCB
#if PCB_VER == VER_1_0_C
enum devs
{
DEV_LORA_RADIO_RTS,		// never used?
DEV_LORA_RADIO_CTS,		// never used?
DEV_FOUR_G_RTS,		// never used?
DEV_FOUR_G_CTS,		// never used?
DEV_FOUR_G_DTR,		// never used?
DEV_FOUR_G_DCD,		// never used?
DEV_LED_0,			// USED!
DEV_LED_1,			// USED!
DEV_LED_2,			// USED!
DEV_LED_3,			// USED!
DEV_LORA_RADIO_RESET,	// USED!
DEV_FOUR_G_RESET,	// USED!
DEV_FOUR_G_POWER,
DEV_END
};
#endif

#if PCB_VER == VER_1_0_D
enum devs
{
DEV_FOUR_G_RTS,		// never used?
DEV_FOUR_G_CTS,		// never used?
DEV_FOUR_G_DTR,		// never used?
DEV_FOUR_G_DCD,		// never used?

// following are on shift register outputs!
DEV_FOUR_G_RESET,	// USED!
DEV_FOUR_G_PSU_EN,	// USED!
DEV_FOUR_G_POWER,	// USED!
DEV_LORA_RADIO_RESET,	// USED!
DEV_LED_0,			// USED!
DEV_LED_1,			// USED!
DEV_LED_2,			// USED!
DEV_LED_3,			// USED!
DEV_END
};

#define SR_THRESHOLD	DEV_FOUR_G_RESET

#endif

#if PCB_VER == VER_LIFT_1_0_A
enum devs
{
DEV_FOUR_G_CTS,		// never used?
DEV_FOUR_G_DTR,		// never used?

// following are on shift register outputs!
DEV_FOUR_G_RESET,	// USED!
DEV_FOUR_G_PSU_EN,	// USED!
DEV_FOUR_G_POWER,	// USED!
DEV_SPR_0,			// USED!
DEV_LED_0,			// USED!
DEV_LED_1,			// USED!
DEV_LED_2,			// USED!
DEV_LED_3,			// USED!

// second SR
DEV_RELAY_0,
DEV_RELAY_1,
DEV_FOUR_G_RTS,		// never used?
DEV_RTS_1,
DEV_LORA_RADIO_RESET,
DEV_SPR_1,
DEV_SPR_2,
DEV_SPR_3,
DEV_END
};


#define SR_THRESHOLD	DEV_FOUR_G_RESET

#endif

#if PCB_VER == VER_LIFT_1_0_B
enum devs
{
DEV_FOUR_G_CTS,		// never used?
DEV_FOUR_G_DTR,		// never used?
DEV_RELAY_1,

// following are on shift register outputs!
DEV_FOUR_G_RESET,	// USED!
DEV_FOUR_G_PSU_EN,	// USED!
DEV_FOUR_G_POWER,	// USED!
DEV_SPR_0,			// USED!
DEV_LED_0,			// USED!
DEV_LED_1,			// USED!
DEV_LED_2,			// USED!
DEV_LED_3,			// USED!

// second SR
DEV_RELAY_0,
DEV_SPR_1,			// relay moved to LIFT_RLA1 pin on ESP...
DEV_SPR_2,			// RTS moved to RTS pin on ESP...
DEV_RTS1,			// never used?
DEV_LORA_RADIO_RESET,
DEV_SPR_3,
DEV_SPR_4,
DEV_SPR_5,
DEV_END
};

#define SR_THRESHOLD	DEV_FOUR_G_RESET

#endif

#if (PCB_VER == VER_BTS_1_0_C) || (PCB_VER == VER_BTS_1_0_D)
enum devs
{
DEV_FOUR_G_RTS,		// never used?
DEV_FOUR_G_CTS,		// never used?
DEV_FOUR_G_DTR,		// never used?
DEV_FOUR_G_DCD,		// never used?

// following are on shift register outputs!
DEV_FOUR_G_RESET,	// USED!
DEV_FOUR_G_PSU_EN,	// USED!
DEV_FOUR_G_POWER,	// USED!
DEV_LORA_RADIO_RESET,	// USED!
DEV_LED_0,			// USED!
DEV_LED_1,			// USED!
DEV_LED_2,			// USED!
DEV_LED_3,			// USED!
DEV_END
};

#define SR_THRESHOLD	DEV_FOUR_G_RESET

#endif


#define NUM_DEVS		DEV_END

// server side commands
// DONT alter the order of these - only add to end (as server is set up to look for these values)
enum srvr_cmds
{
SCMD_ERROR = 0,			//  0 = ERROR
SCMD_NWLIST,			//  1 = NMEA whitelist
SCMD_NBLIST,			//  2 = NMEA blacklist
SCMD_BWLIST,			//  3 = Bluetooth whitelist
SCMD_BBLIST,			//  4 = Bluetooth blacklist
SCMD_LWLIST,			//  5 = LoRaWAN whitelist
SCMD_LBLIST,			//  6 = LoRaWAN blacklist
SCMD_BAUD,				//  7 = set serial bitrate
SCMD_BINARY,			//  8 = set serial binary data mode (0,1,2)
SCMD_EOLSTR,			//  9 = End Of Line character string
SCMD_EOLCTRL,			// 10 = set serial End Of Line string removal on \ off
SCMD_TIMER,				// 11 = set message timer
SCMD_DISCONN,			// 12 = forced disconnect
SCMD_PAIR,				// 13 = 
SCMD_REBOOT,			// 14 = 
SCMD_SCAN,				// 15 = 
SCMD_PING,				// 16 = 
SCMD_BDWLIST,			// 17 = Bluetooth beacon \ device name whitelist list
SCMD_LIFT,				// 18 = lift system commands
SCMD_DATETIME,			// 19 = set Gateway Date and Time
SCMD_LORA,				// 20 = LoRa parameters
SCMD_END
};

// arguments for server side commands
// DONT alter the order of these - only add to end (as server is set up to look for these values)
enum srvr_cmdargs
{
SARG_OFF = 0,				//  0 = turn listing OFF
SARG_ON,					//  1 = turn listing ON
SARG_AUTO,					//  2 = set to auto mode
SARG_ADD,					//  3 = ADD to list
SARG_REMOVE,				//  4 = REMOVE to list
SARG_REMALL,				//  5 = REMOVE ALL from list
SARG_TX,					//  6 = send list to server
SARG_QUERY,					//  7 = query current state
SARG_VAL,					//  8 = value to be used
SARG_GW,					//  9 = gateway sensor timer ID
SARG_GPS,					// 10 = GPS sensor timer ID
SARG_I2C,					// 11 = I2C sensor timer ID
SARG_TANK,					// 12 = Tank sensor timer ID
SARG_BLUETOOTH,				// 13 = Bluetooth MAC address timer ID
SARG_LORAWAN,				// 14 = LoRaWAN Dev Address timer ID
SARG_BT_DEVNAME,			// 15 = Bluetooth Beacon \ device name timer ID
SARG_SET_LIFT_TIMER,		// 16 = set lift timer
SARG_SET_LIFT_PWM,			// 17 = set lift PWM duty (0-100%)
SARG_SELECT_EVENT,			// 18 = select timer event to be queried \ updated
SARG_SET_EVENT_ENABLE,		// 19 = set timer event enable
SARG_SET_EVENT_MODE,		// 20 = set timer event Scheduled or Repeat mode
SARG_SET_EVENT_DAY,			// 21 = set timer event day value
SARG_SET_EVENT_TIME_1,		// 22 = set timer event start time value
SARG_SET_EVENT_TIME_2,		// 23 = set timer event end time value
SARG_SET_EVENT_PWM,			// 24 = set timer event PWM value
SARG_SET_EVENT,				// 25 = set all fields of timer event at once
SARG_CLEAR_EVENT,			// 26 = clear timer event data fields and set inactive
SARG_QUERY_EVENT,			// 27 = return timer event info to server
SARG_LIFT_UVC,				// 28 = lift Ultra Violet Cleaner
SARG_LIFT_FAN,				// 29 = lift fan
SARG_SET_MULTI_EVENT,		// 30 = set multiple timer event at once
SARG_CLEAR_MULTI_EVENT,		// 31 = clear multiple timer events and set inactive
SARG_LORA_BANDWIDTH,		// 32 = set LoRa bandwidth
SARG_LORA_CODING_RATE,		// 33 = set LoRa bandwidth
SARG_LORA_SPREAD_FACTOR,	// 34 = set LoRa spread factor
SARG_LORA_MAX_POWER,		// 35 = set LoRa tx max powor
SARG_LORA_TX_POWER,			// 36 = set LoRa Tx power
SARG_LORA_GET_BW_CODING,	// 37 = get LoRa bandwidth and coiding rate
SARG_LORA_GET_SPREAD,		// 38 = set LoRa spread factor
SARG_LORA_GET_TX_POWER,		// 39 = get LoRa Tx power and max power
SARG_WIFI_AUTH,				// 40 = show \ set wifi authorisation mode

SARG_END
};

// INFO ONLY!
// server payload JSON items:
//  1 = adc_value
//  2 = up_time
//  3 = gps_lat
//  4 = gps_long
//  5 = gps_speed
//  6 = lora_freq nnn.nnn
//  7 = 4G rssi
//  8 = 4G rssi
//  9 = auto_serial_bitrate
// 10 = serial_bitrate
// 11 = binary_mode_flag
// 12 = binary_data_flag
// 13 = eol_remove_flag
// 14 = enable_nmea_whitelist_flag
// 15 = enable_nmea_blacklist_flag
// 16 = enable_bluetooth_whitelist_flag
// 17 = enable_bluetooth_blacklist_flag
// 18 = enable_lorawan_whitelist_flag
// 19 = enable_lorawan_blacklist_flag
// 20 = gateway_sensor_time
// 21 = gps_sensor_time
// 22 = i2c_sensor_time
// 23 = tank_sensor_time
// 24 = bluetooth_MAC_addr_time
// 25 = lora_devaddr_time
// 26 = enable_bluetooth_devname_whitelist_flag
// 27 = bluetooth_devname_time
// 28 = get_server_time_flag
// 29 = lift_UVC_status			(was lift_sw_status)
// 30 = lift_UVC_temperature	(was lift_sw_on_time)
// 31 = lift_current_PWM		(was lift_sw_repeat_time)
// 32 = lift_UVC_error_byte		(was lift_sw_repeat_timer)
// 33 = max number of events
// 34 = event number
// 35 = evt_enable_flag
// 36 = evt_mode_flag
// 37 = evt_day
// 38 = evt_t1_hrs
// 39 = evt_t1_mins
// 40 = evt_t2_hrs
// 41 = evt_t2_mins
// 42 = evt_pwm
// 43 = evt_running_flag

// 44 = evt start \ stop flag
// 45 = evt start \ stop date
// 46 = evt start \ stop hrs
// 47 = evt start \ stop mins
// 48 = evt start \ stop PWM
// 49 = evt start \ stop error byte

// 50 = meter flash count (number of 1Wh LED flashes)	// RENUMBER as 50 (DONE):  SHIFT UP 50-54 to 51-55 (DONE): eventually DELETE 51 - 55 
// 51 = meter read instantaneous power (3600/ LED pulse period in seconds) result in kW
// 52 = meter manufacturer code (3 chars)
// 53 = meter ID string
// 54 = meter read baud index
// 55 = meter read number of items in payload
// 56 = meter read payload item
// 57 = ADC0
// 58 = ADC1
// 59 = Rally alert status
// 60 = rally alert table start
// 61 = Wifi Sniffer MAC address
// 62 = Wifi Sniffer RSSI
// 63 = Wifi Sniffer SSID length
// 64 = Wifi Sniffer SSID string



#define SRVR_NO_RESPONSE			0
#define SRVR_OK_RESPONSE			1
#define SRVR_ERR_RESPONSE			2

////////////////////////
// DEBUG SYSTEM
////////////////////////
#define DBG_USM					(1 << 0)	//
#define DBG_AUTOBAUD			(1 << 1)	//
#define DBG_TANK_SENSORS		(1 << 2)	//
#define DBG_SD_IN_OUT			(1 << 3)
#define DBG_SLEEP				(1 << 4)
#define DBG_I2C					(1 << 5)
//#define DBG_SD_IN_OUT			(1 << 6)
//#define DBG_IR_PULSE			(1 << 7)
#define DBG_BLUETOOTH_FILT		(1 << 6)
#define DBG_BLUETOOTH			(1 << 7)

//#define DBG_LORA_RADIO_TXRX			(1 << 8)
#define DBG_4G_TXRX				(1 << 8)
#define DBG_MAC_TRAFFIC			(1 << 9)
#define DBG_MAC_TRAFFIC_LL		(1 << 10)
//#define DBG_MOTOR_CURRENT		(1 << 11)
#define DBG_MQTT				(1 << 11)
//#define DBG_LORA_RADIO_MSGS			(1 << 12)
#define DBG_LORA				(1 << 12)
#define DBG_CLI					(1 << 13)
#define DBG_TIMER_MSGS			(1 << 14)
#define DBG_TIMER_CALLS			(1 << 15)	//
// new DBG defines
#define DBG_WEBSERVER			(1 << 16)	//
#define DBG_UDP					(1 << 17)	//
#define DBG_SIMCOM_BUSY_FLAGS	(1 << 18)	//
#define DBG_ADCS				(1 << 19)	//
#define DBG_LORA_INT_FLAGS		(1 << 20)	//
#define DBG_LORA_RCV			(1 << 21)	//
#define DBG_LIFT_EVT			(1 << 22)	//
#define DBG_ALLOC_MEM			(1 << 23)	//

#define DBG_UVC_PCB				(1 << 24)	//
#define DBG_LIFT_EVENTS			(1 << 25)	//
#define DBG_26					(1 << 26)	//
#define DBG_27					(1 << 27)	//
#define DBG_28					(1 << 28)	//
#define DBG_29					(1 << 29)	//
#define DBG_30					(1 << 30)	//
#define DBG_31					(1 << 31)	//

//#define CTS2_TEST		// uses INT to check and report SIMCOM CTS2 line state...

#ifdef CONFIG_IDF_TARGET_ESP32
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (19)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_HS_CH1_GPIO       (18)
#define LEDC_HS_CH1_CHANNEL    LEDC_CHANNEL_1
#endif
#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#ifdef CONFIG_IDF_TARGET_ESP32S2
#define LEDC_LS_CH0_GPIO       (19)
#define LEDC_LS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_LS_CH1_GPIO       (18)
#define LEDC_LS_CH1_CHANNEL    LEDC_CHANNEL_1
#endif
#define LEDC_LS_CH2_GPIO       (4)
#define LEDC_LS_CH2_CHANNEL    LEDC_CHANNEL_2
#define LEDC_LS_CH3_GPIO       (5)
#define LEDC_LS_CH3_CHANNEL    LEDC_CHANNEL_3

#define LEDC_TEST_CH_NUM       (4)
#define LEDC_TEST_DUTY         (512)	//(4000)
#define LEDC_TEST_FADE_TIME    (3000)

#define LEDC_RANGE				8192

#define PUMP_MOTOR_SAMPLES		10

#define ADC_REF					330	// 3V3
#define VMON_RATIO_UPR			233	//	R1+R2 = 1.33K + 1.0K = 2.33K (x100 for accuracy)
#define VMON_RATIO_LWR			100	//	R2 = 1.0K (x100 for accuracy)

// the following list length are limited by system memory available. If some options are not compiled in, 
// there may be more memory available to make the remaining lists longer...
#ifndef USE_BLUETOOTH_CLASSIC_DETECT
#define WIFI_LIST_LEN				512	//1024
#define BLUETOOTH_LIST_LEN			512	//1024
#define BLUETOOTH_DEVNAME_LIST_LEN	512
#define LORAWAN_LIST_LEN			512	//1024
#else
#define WIFI_LIST_LEN				200	//1024
#define BLUETOOTH_LIST_LEN			200	//1024
#define BLUETOOTH_DEVNAME_LIST_LEN	200
#define LORAWAN_LIST_LEN			512	//1024
#endif


#define TANK_SENSOR_MAX			220	// value corresponding to ADC reading at top of tank sensor


#define GATEWAY_SENSOR_TIME		 30 // secs
#define GPS_SENSOR_TIME			 30 // secs
#define I2C_SENSOR_TIME		 	 40 // secs
#define TANK_SENSOR_TIME		 60 // secs
#define BLUETOOTH_SENSOR_TIME	 60 // secs
#define BLUETOOTH_MAC_ADDR_TIME	 60 // secs
#define BLUETOOTH_DEVNAME_TIME	 60 // secs
#define LORA_DEVADDR_TIME   	 60 // secs
#define WIFI_MAC_ADDR_TIME   	 60 // secs

#define USM_TRIGGER_TIME		  5	// * 100msec
#define USM_TIMEOUT				 60 // msec
#define USM_COEFF				 58	// us/cm
#define USM_AVE					  8

#define USM_IDLE					0
#define USM_TRIGGERED_WAITING		1
#define USM_COMPLETED				2
#define USM_EXCEEDED_TIMEOUT		3

// lift timer events
#define NUM_EVENTS_MAX				8
#define EVENT_NOTIFICATIONS_MAX 	NUM_EVENTS_MAX*2
#define EVENT_MODE_IDLE				0
#define EVENT_MODE_SCHEDULED		1
#define EVENT_MODE_REPEAT			2

#define EVENT_START					0
#define EVENT_STOP					1

#define EVENT_MANUAL				NUM_EVENTS_MAX + 1
#define EVENT_REMOTE				NUM_EVENTS_MAX + 2
#define EVENT_ERROR					NUM_EVENTS_MAX + 3
#define EVENT_TIMER_IDLE		0xFFFF

#define NO_INX		0xFF

#define UVC_RCV_TIMEOUT			   100	// 100ms steps rcv timeout from UVC monitor PCB
#define LIFT_UVC_THRESHOLD		   128	// threshold voltage for lift UVC detector (half rail)
#define LIFT_TEMP_THRESHOLD			60	// threshold voltage for lift temp detector
#define LIFT_MIN_PWM		 		 5	// 5%

#define LIFT_ERR_NO_UVC				 1	// ????
#define LIFT_ERR_UVC_ON				 2
#define LIFT_ERR_UVC_OVER_TEMP		 4
#define LIFT_ERR_LOW_PWM			 8
#define LIFT_ERR_NO_CHECK_DATA		16

#define LIFT_ERR_MULTIPLE_EVENTS	 1


#define GET_SERVER_TIME_PERIOD		48		// hrs

// defs for new SIMCOM serial handling...
//#define NEW_MQTT_SERIAL
#define FOUND_EOL			1
#define FOUND_RIGHT_ARROW	2

#define SIMCOM_DATA			0
#define CMQTTRX_CMD			1
#define CMQTTRX_DATA		2

// lift event status msg types
#define EVENT_NO_MSG		0
#define EVENT_START_MSG		1
#define EVENT_STOP_MSG		2

#define GPIO_SET				1
#define GPIO_REPORT				2
#define GPIO_SET_AND_REPORT		3
#define GPIO_CSV				4

// LoRa link defines
#define LORA_TEST_TX_TIME		20	// 2 sec

// server return values
#define SRVR_RETURN_STR			0
#define SRVR_RETURN_U8			1
#define SRVR_RETURN_S8			2
#define SRVR_RETURN_U16			3
#define SRVR_RETURN_S16			4
 
// time info
#define MINS_IN_A_DAY		(24 * 60)
#define MINS_IN_A_WEEK		(7 * 24 * 60)
struct WIFI_RECORD
{
unsigned char MAC_addr[8];
unsigned char RSSI;
unsigned char PAD;
};

struct BT_MAC_INFO
{
//uint64_t* bluetooth_MAC_addr;		// Bluetooth MAC addr is 6 bytes
char bluetooth_devname[20];		// Bluetooth MAC addr is 6 bytes
unsigned char bluetooth_MAC_addr[6];		// Bluetooth MAC addr is 6 bytes
unsigned char bluetooth_rssi;
};

struct BT_DEVNAME_INFO
{
char bluetooth_devname[20];		// Bluetooth MAC addr is 6 bytes
unsigned char bluetooth_MAC_addr[6];		// Bluetooth MAC addr is 6 bytes
unsigned char bluetooth_rssi;
};

struct EVENT_INFO
{
unsigned char evt_enable_flag;
unsigned char evt_mode_flag;
unsigned char evt_day;
unsigned char evt_t1_hrs;
unsigned char evt_t1_mins;
unsigned char evt_t2_hrs;
unsigned char evt_t2_mins;
unsigned char evt_pwm;
unsigned char evt_running;
};

#endif