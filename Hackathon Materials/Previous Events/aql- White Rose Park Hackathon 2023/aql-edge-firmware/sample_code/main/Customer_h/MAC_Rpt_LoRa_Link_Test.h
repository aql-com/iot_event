//////////////////////////////////////////////
//
// MAC_Rpt - LoRa_Link_Test.h
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

#ifndef LORA_LINK_TEST_H
#define LORA_LINK_TEST_H

#warning "USING DEFAULT BUILD SETTINGS FROM ESP_NOW_Test.h"

#define USING_PROJECT_SETTINGS

#define PROJECT_STRING			"LoRa_Link_Test"

#define USE_DEV_PCB

//#define SIMCOM_PROGRAMMING_MODE	// just power up SIMCOM module and then wait for programming to occur...
//#define TEST_I2C
//#define USE_I2C					// I2C connected sensors on GPIs...	
//#define USE_I2C_AMBIMATE4		// use the AmbiMate4 sensor board on I2C bus
//#define USE_I2C_PIMORONI		// use the Pimoroni sensor selection on I2C bus

/*
// following addresses must be of format <tcp:\\addr.ext:port>
#define SERVER_ADDRESS_AND_PORT_0		"tcp://iot-visualiser.aql.com:8883"		// main server:
#define SERVER_ADDRESS_AND_PORT_1		"tcp://109.239.102.45:1700"				// Lora server:
#define SERVER_ADDRESS_AND_PORT_2		"tcp://iot-visualiser.aql.com:2883"		// 
#define SERVER_ADDRESS_AND_PORT_3		"tcp://iot-visualiser.aql.com:3883"		// 

#define DFLT_SERVER_ADDRESS_AND_PORT	"tcp://iot-visualiser.aql.com:9883"		// 
*/

// default server addresses and SSL settings
#include "MAC_Rpt_dflt_server_addrs.h"

//#define USE_M4G					// 4G comms
//#define USE_M4G_HTTP			// 4G comms over HTTP
//#define USE_M4G_MQTT			// 4G comms over MQTT
//#define USE_M4G_MQTT_SSL

#define USE_WIFI_MQTT		// FOR wifi MQTT

#define AQL_SERVER
//#define SSS_SERVER
//#define MOSQUITTO_SERVER

//#define USE_BLUETOOTH
//#define BLUETOOTH_SCAN
//#define USE_BLUETOOTH_ADVERTISING	// for Bluetooth sensor mode

//#define USE_BLUETOOTH_MAC_FILTERING	// now redundant due to white \ black listing facilties?

//#define USE_LORA
//#define LORAWAN_SCAN

//#define WIFI_SCAN
//#define USE_WEBSERVER
//#define CONFIG_EXAMPLE_BASIC_AUTH
//#define CONFIG_EXAMPLE_CONNECT_WIFI

// WIFI SSID and password
#define WIFI_STA_PREFIX		"default_SSID"
#define WIFI_STA_PASSWORD	"defaut_PWD"

#define WIFI_AP_PREFIX		"MACRPT-A-"
#define WIFI_AP_PASSWORD	"password"

#define PREFERRED_MQTT_MODE	MQTT_MODE_WIFI	// MQTT_MODE_NONE=0, MQTT_MODE_4G=1, MQTT_MODE_WIFI=2

//#define USE_SWITCHES

//#define USE_TANK_SENSORS
//#define TANKSENSOR_1_ON_GPIO39

//#define USE_ULTRASONIC_SENSOR

//#define USE_ESP_NOW
#define USE_LORA_LINK

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

#define VER_10A_STR		"VER_1_0_A"
#define VER_10B_STR		"VER_1_0_B"
#define VER_10C_STR		"VER_1_0_C"
#define VER_10D_STR		"VER_1_0_D"


///////////////////////////////////////////////////////
////////////////////////
// GPIO pin defines
////////////////////////
// V1 PCB 
////////////////////////
//#ifdef V1_PCB
#if PCB_VER == VER_1_0_C

#endif
 
#if PCB_VER == VER_1_0_D
#include "../PinDefs/V1_0_D_Pins.h"

/*
#if defined(USE_ULTRASONIC_SENSOR) || defined(TANKSENSOR_1_ON_GPIO39) || defined (USE_NMEA)
#define PORTB_USED
#endif
#if ! defined(USE_ULTRASONIC_SESNSOR) && !defined(TANKSENSOR_1_ON_GPIO39) && defined (USE_TANK_SENSORS)
#define TANK_SENSOR_USED
#endif
#if ! defined(USE_ULTRASONIC_SENSOR) && !defined(TANKSENSOR_1_ON_GPIO39) && defined (USE_NMEA)
#define SERIAL2_USED
#endif
#if ! defined(USE_ULTRASONIC_SENSOR) && !defined(TANKSENSOR_1_ON_GPIO39) && (defined (USE_LORA) || defined(USE_LORA_LINK))
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

*/

#endif









#endif
