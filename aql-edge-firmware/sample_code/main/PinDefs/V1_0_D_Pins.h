//////////////////////////////////////////////
//
// MAC_Rpt V1_0_D_Pins.h
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

#ifndef V1_0_D_PINS_H
#define V1_0_D_PINS_H

#warning "USING V1_0_D Pin definitions"

// GPIO pin defines
////////////////////////
// V1 PCB 
////////////////////////
//#ifdef V1_PCB
 
#if PCB_VER == VER_1_0_D
#if defined(USE_ULTRASONIC_SENSOR) || defined(TANKSENSOR_1_ON_GPIO39) || defined (USE_NMEA)
#define PORTB_USED
#endif
#if !defined(USE_ULTRASONIC_SENSOR) && !defined(TANKSENSOR_1_ON_GPIO39) && defined (USE_TANK_SENSORS)
#define TANK_SENSOR_USED
#endif
#if !defined(USE_ULTRASONIC_SENSOR) && !defined(TANKSENSOR_1_ON_GPIO39) && (defined (USE_NMEA) || defined(METER_READING)  || defined(METER_READING_2))
#define SERIAL2_USED
#endif
#if !defined(USE_ULTRASONIC_SENSOR) && !defined(TANKSENSOR_1_ON_GPIO39) && defined (USE_LORA)
#define LORA_USED
#endif

#if !defined(USE_ULTRASONIC_SENSOR) && !defined(TANKSENSOR_1_ON_GPIO39) && defined (USE_ADCS)
#define ADCS_USED
#endif

#define TXD0				 (gpio_num_t)1				// OUT
#define RXD0				 (gpio_num_t)3				// IN
#define SDA				 	(gpio_num_t)25				// 
#define SCL				 	(gpio_num_t)26				// 


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
#define RI2				 (gpio_num_t)17				// IN

#define VMON				(gpio_num_t)35				// OUT

#define ADC_VMON			ADC1_CHANNEL_7
#define ADC_VMON2			ADC1_CHANNEL_4
#define ADC_TANKSENSOR0		ADC1_CHANNEL_5
#ifdef TANKSENSOR_1_ON_GPIO39
#define ADC_TANKSENSOR1		ADC1_CHANNEL_3
#else
#define ADC_TANKSENSOR1		ADC2_CHANNEL_9
#endif

#ifdef ADCS_USED
#define ADC_INPUT_0		ADC1_CHANNEL_5
#define ADC_INPUT_1		ADC1_CHANNEL_3
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

#ifdef ADCS_USED
#define ADC_0				(gpio_num_t)33				// IN
#define ADC_1				(gpio_num_t)39				// IN
#endif

#ifdef USE_RALLY_SAFETY
#define RALLY_SENSE			(gpio_num_t)25 	// 25				// IN
#define RALLY_BUTTON			(gpio_num_t)26				// IN
#define RALLY_CANCEL			(gpio_num_t)33	// 33				// IN

#endif

#ifdef ZULU_RADIO_MASTER
#define PORTB_USED
#define ZULU_RADIO_MASTER_RTS	(gpio_num_t)25		// J10 pin 2
#define ZULU_RADIO_MASTER_CTS	(gpio_num_t)26		// J10 pin 3
#endif

#ifdef LIFT_DEMO
#define PORTB_USED	
#define LIFT_DEMO_SW			(gpio_num_t)33		// J10 pin 4		
#define LIFT_DEMO_PWM			(gpio_num_t)18		// J22 pin 4		
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

#define SHIFT_REG_LENGTH		 8

#define LED_POLARITY			INVERTED
#endif

#endif
