//////////////////////////////////////////////
//
// MAC_Rpt - MAC_Rpt_dflt_server_addrs.h
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

#ifndef MAC_RPT_DFLT_SERVER_ADDRS_H
#define MAC_RPT_DFLT_SERVER_ADDRS_H

// following addresses must be of format <tcp:\\addr.ext:port>

#define SERVER_ADDRESS_AND_PORT_0	"tcp://iot-visualiser.aql.com:8883"		// new mqtt server
#define SERVER_SSL_MODE_0		1

#define SERVER_ADDRESS_AND_PORT_1	"tcp://lora.core.aql.com:1700"		// new lora server
#define SERVER_SSL_MODE_1		0

#define SERVER_ADDRESS_AND_PORT_2	"tcp://iot-visualiser.aql.com:8883"	// old server
#define SERVER_SSL_MODE_2		1

#define SERVER_ADDRESS_AND_PORT_3	"tcp://iot-visualiser-api.aql.com:3883"	// spare address slot
#define SERVER_SSL_MODE_3		0

#define DFLT_SERVER_ADDRESS_AND_PORT	"tcp://iot-visualiser.aql.com:8883"		// 
#define DFLT_SERVER_SSL_MODE		1


#endif
