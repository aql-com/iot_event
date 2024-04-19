//////////////////////////////////////////////
//
// MAC_Rpt V1_0_C_Pins.h
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

#ifndef V1_0_C_PINS_H
#define V1_0_C_PINS_H

#warning "USING V1_0_C Pin definitions"

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

#define SERIAL2_USED
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


#ifdef USE_TANK_SENSORS
#define TANKSENSOR0			(gpio_num_t)39				// IN
#define TANKSENSOR1			(gpio_num_t)36				// IN
#define TSENSCTRL			(gpio_num_t)33				// IN
#else
#define SW0 				(gpio_num_t)39				// IN
#define SW1 				(gpio_num_t)36				// IN
#define SW2 				(gpio_num_t)33				// IN
#endif

#define SHIFT_REG_LENGTH		 8

#endif
 

#endif
