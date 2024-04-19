//////////////////////////////////////////////
//
// MAC_Rpt_wifi_udp.cpp
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "tcpip_adapter.h"
//#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
//#include "mqtt_client.h"

#include "MAC_Rpt.h"
#include "MAC_Rpt_Rtns.h"

//#include "ca_cert.h"
//#include "client_cert.h"
//#include "client_key.h"


extern char wifi_udp_url[];	// = "mqtt.core.aql.com";

extern unsigned int wifi_udp_port;	// = 8883;
extern ip_addr_t wifi_udp_ip_addr;

extern ip4_addr_t ip;
extern ip4_addr_t gw;
extern ip4_addr_t msk;

extern bool wifi_udp_connected;	// = false;
extern bool wifi_udp_DNSfound;	// = false;

// resolve dns:
// https://gist.github.com/MakerAsia/37d2659310484bdbba9d38558e2c3cdb
void wifi_udp_dns_found_cb(const char *name, const ip_addr_t *ipaddr, void *callback_arg)
{
if (ipaddr != NULL) 
	{
	wifi_udp_ip_addr = *ipaddr;
    wifi_udp_DNSfound = true;
	
	printf("wifi_udp dns ok: %s = %s\n",wifi_udp_url,inet_ntoa(wifi_udp_ip_addr));
	}
}


unsigned char wifi_udp_connect(char* udp_url, ip_addr_t* udp_ip_addr)
{
// this starts a background task which mnay not end for a while...
// monitor the result by checking bDNSFound variable	
unsigned char err = 0;

dns_gethostbyname(udp_url, udp_ip_addr, wifi_udp_dns_found_cb, NULL );	
	
return err;
}





void print_wifi_udp_state(unsigned char chan)
{
dbgprintf(chan,"UDP WIFI State :  %d\r\n",wifi_udp_connected);
}
