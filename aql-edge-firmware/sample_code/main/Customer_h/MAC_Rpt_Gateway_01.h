//////////////////////////////////////////////
//
// MAC_Rpt - MAC_Rpt_Gateway_01.h
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

#ifndef MAC_RPT_GATEWAY_01_H
#define MAC_RPT_GATEWAY_01_H

#warning "USING BUILD SETTINGS FROM MAC_Rpt_Gateway_01.h"

#define USING_PROJECT_SETTINGS

#define PROJECT_STRING			"MAC_Rpt_Gateway_01"

#define USE_DEV_PCB

//#define SIMCOM_PROGRAMMING_MODE	// just power up SIMCOM module and then wait for programming to occur...
//#define TEST_I2C
//#define USE_I2C					// I2C connected sensors on GPIs...	
//#define USE_I2C_AMBIMATE4		// use the AmbiMate4 sensor board on I2C bus
//#define USE_I2C_PIMORONI		// use the Pimoroni sensor selection on I2C bus

/*
// following addresses must be of format <tcp:\\addr.ext:port>
//#define SERVER_ADDRESS_AND_PORT_0		"tcp://mqtt.prod.iot.ext.lb.aql.com:8883"	
//#define SERVER_ADDRESS_AND_PORT_0		"tcp://iot-visualiser.aql.com:8883"		// main server:
//#define SERVER_ADDRESS_AND_PORT_1		"tcp://109.239.102.45:1700"				// Lora server:

#define SERVER_ADDRESS_AND_PORT_0		"tcp://iot-visualiser.aql.com:8883"		// new mqtt server
#define SERVER_ADDRESS_AND_PORT_1		"tcp://lora.core.aql.com:1700"		// new lora server
#define SERVER_ADDRESS_AND_PORT_2		"tcp://iot-visualiser.aql.com:8883"		// closed down Apr 2023
#define SERVER_ADDRESS_AND_PORT_3		"tcp://iot-visualiser-api.aql.com:3883"		// 

#define DFLT_SERVER_ADDRESS_AND_PORT	"tcp://iot-visualiser.aql.com:8883"		// 

#define SERVER_SSL_MODE_0				1
#define SERVER_SSL_MODE_1				0
#define SERVER_SSL_MODE_2				1
#define SERVER_SSL_MODE_3				0
#define DFLT_SERVER_SSL_MODE				1
*/

// default server addresses and SSL settings
#include "MAC_Rpt_dflt_server_addrs.h"

#define USE_M4G					// 4G comms
//#define USE_M4G_HTTP			// 4G comms over HTTP
#define USE_M4G_MQTT			// 4G comms over MQTT
#define USE_M4G_MQTT_SSL

#define USE_WIFI_MQTT		// FOR wifi MQTT

#define AQL_SERVER
//#define SSS_SERVER
//#define MOSQUITTO_SERVER

#define USE_BLUETOOTH
#define BLUETOOTH_SCAN
//#define USE_BLUETOOTH_ADVERTISING	// for Bluetooth sensor mode

//#define USE_BLUETOOTH_MAC_FILTERING	// now redundant due to white \ black listing facilties?

//#define USE_BLUETOOTH_CLASSIC_DETECT
#define BLUETOOTH_CLASSIC_SCAN_TIME		60	// number of sec between scans
#define BLUETOOTH_CLASSIC_SCAN_LENGTH	10	// number of sec * 1.28 for maximum scan time

//$$#define USE_LORA
//#define LORAWAN_SCAN

//#define WIFI_SCAN
#define USE_WEBSERVER
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

#define USE_ADCS

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
#define PCB_VER			VER_1_0_D

#define HW_MAJ				 1
#define HW_MIN				 0
#define HW_SUB				'D'

///////////////////////////////////////////////////////////////////////
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
#endif


#endif
