//////////////////////////////////////////////
//
// MAC_Rpt_wifi.cpp
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
//#include "esp_netif.h"

#include <esp_http_server.h>

#include "lwip/err.h"
#include "lwip/sys.h"

#include "MAC_Rpt.h"
#include "MAC_Rpt_rtns.h"
#include "MAC_Rpt_http_server.h"
#include "MAC_Rpt_wifi.h"

#ifdef USE_WIFI_MQTT
#include "mqtt_client.h"
#endif
	
#define EXAMPLE_ESP_WIFI_SSID      "ESPSSID"
#define EXAMPLE_ESP_WIFI_PASS      "WIFIPASSWD"
#define EXAMPLE_ESP_MAXIMUM_RETRY  5


/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0	// 1<<0
#define WIFI_FAIL_BIT      BIT1	// 1<<1

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

static const char *TAG = "wifi station";

//wifi_config_t wifi_config_ap;
//wifi_config_t wifi_config_sta;

extern unsigned long int dbgbits;
extern unsigned char dbgflag;
extern unsigned char devpin[];
extern unsigned char status_array[];
extern unsigned char tank_status;

extern unsigned char wifi_scan_flag, wifi_check_flag;

extern unsigned char lora_radio_ID[];
extern char mac_ID_str[];
extern httpd_handle_t http_server;

extern unsigned char wifi_disable_flag;
extern unsigned char wifi_sta_connected;
extern unsigned char wifi_sta_auth,wifi_ap_auth;

#ifdef USE_WIFI_MQTT
extern char wifi_sta_SSID[WIFI_CREDENTIALS_MAX][32];
extern char wifi_sta_PWD[WIFI_CREDENTIALS_MAX][32];
extern unsigned char wifi_sta_credentials_index;

#endif


/////////////////////////////
// function prototypes
/////////////////////////////

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event);


//wifi_config_t wifi_config_sta = WIFI_INIT_CONFIG_DEFAULT();
//wifi_config_t wifi_config_ap = WIFI_INIT_CONFIG_DEFAULT();


wifi_config_t wifi_config_ap = {
.ap = {
//.ssid = "X",		//AP_EXAMPLE_WIFI_SSID,
	.ssid_len = 0,
//.password = "passwd",	//AP_EXAMPLE_WIFI_PASS,
	.channel = 1,
	.authmode = (wifi_auth_mode_t)wifi_ap_auth,	// WIFI_AUTH_WPA2_PSK,
	.max_connection = 16,
	.beacon_interval = 400
	}
//.sta = {
//.ssid = "X",	//EXAMPLE_ESP_WIFI_SSID,		//STA_EXAMPLE_WIFI_SSID,
//.password = "passwd"	//STA_EXAMPLE_WIFI_PASS,
//}
};



wifi_config_t wifi_config_sta = {
.sta = {
		.threshold = 
			{
			.authmode = (wifi_auth_mode_t)wifi_sta_auth,	// WIFI_AUTH_WPA2_PSK	// duplicated below in STA setup...
			}
	}
	
};


	
void wifi_init_sta(void)
{
uint8_t mac_addr[6];
char mac_str[20];
char ID_str[10];
unsigned char i,j,p;

    s_wifi_event_group = xEventGroupCreate();

//	ESP_ERROR_CHECK(esp_netif_init());
	tcpip_adapter_init();
	
// start wifi event handler
//    esp_netif_create_default_wifi_sta(); 
//    ESP_ERROR_CHECK(esp_event_loop_create_default());
	ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	
	
	esp_read_mac(mac_addr,ESP_MAC_WIFI_SOFTAP);
	mac_to_str(mac_addr, '-', mac_str);
	
// create SSID with radio module ID
	p = 0;
	j = strlen(mac_ID_str);

	for (i=0;i<j;i++)
		{
		ID_str[p++] = mac_ID_str[i];
		if (i % 2)
			ID_str[p++] = '-';
		}
	ID_str[p-1] = 0x00;	// remove last '-' and terminate string


//    strcpy((char*)wifi_config_ap.ap.ssid, 	  EXAMPLE_ESP_WIFI_SSID);
//    strcpy((char*)wifi_config_ap.ap.password, EXAMPLE_ESP_WIFI_PASS);

#define USE_H_FILE_SSID_PREFIXES

/////////////////////////////////////////////////////////
//
// STA config
//
/////////////////////////////////////////////////////////
#ifdef USE_H_FILE_SSID_PREFIXES

	#ifdef USE_WIFI_MQTT
		strcpy((char*)wifi_config_sta.sta.ssid,(char*)&wifi_sta_SSID[0]);	//"MACRPT-S-");		// SSID is 32 chars max	
	#else
		strcpy((char*)wifi_config_sta.sta.ssid,WIFI_STA_PREFIX);	//"MACRPT-S-");		// SSID is 32 chars max
	#endif
#else
	strcpy((char*)wifi_config_sta.sta.ssid,"MACRPT-S-");		// SSID is 32 chars max
#endif
//	strcat((char*)wifi_config_sta.sta.ssid,ID_str);				// do not append MAC address to STA SSID... this is the addres we are trying to connect to!

//	strcpy((char*)wifi_config.sta.password,"password");		// password is 64 chars max OPTIONAL?
#ifdef WIFI_STA_PASSWORD
	#ifdef USE_WIFI_MQTT
	strcpy((char*)wifi_config_sta.sta.password,(char*)&wifi_sta_PWD[wifi_sta_credentials_index]);		// password is 64 chars max OPTIONAL?
	#else
	strcpy((char*)wifi_config_sta.sta.password,(char*)&WIFI_STA_PASSWORD[wifi_sta_credentials_index]);		// password is 64 chars max OPTIONAL?
	#endif
#endif

    wifi_config_sta.sta.threshold.authmode = (wifi_auth_mode_t)wifi_sta_auth;	// WIFI_AUTH_WPA2_PSK;	//ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD;

//    wifi_config_sta.sta.sae_pwe_h2e = ESP_WIFI_SAE_MODE;
//    wifi_config_sta.sta.sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER;

/////////////////////////////////////////////////////////
//
// AP config
//
/////////////////////////////////////////////////////////
#ifdef USE_H_FILE_SSID_PREFIXES
	strcpy((char*)wifi_config_ap.ap.ssid,WIFI_AP_PREFIX);		//"MACRPT-A-");		// SSID is 32 chars max
#else
	strcpy((char*)wifi_config_ap.ap.ssid,"MACRPT-A-");		// SSID is 32 chars max
#endif
	strcat((char*)wifi_config_ap.ap.ssid,ID_str);
//	strcpy((char*)wifi_config.ap.password,"password");		// password is 64 chars max MUST BE >=8 chars!
	strcpy((char*)wifi_config_ap.ap.password,WIFI_AP_PASSWORD);		// password is 64 chars max MUST BE >=8 chars!

#ifdef WIFI_HIDE_SSID
	wifi_config_ap.ap.ssid_hidden = 1;		// hide SSID
#endif

	printf("STA SSID = %s\r\n",wifi_config_sta.sta.ssid);
	printf("AP  SSID = %s\r\n",wifi_config_ap.ap.ssid);

    wifi_config_ap.ap.authmode = (wifi_auth_mode_t)wifi_ap_auth;	// WIFI_AUTH_WPA2_PSK;	//ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD;

/*
	strcpy((char*)wifi_config_ap.sta.ssid,"MACRPT-");		// SSID is 32 chars max
	strcat((char*)wifi_config_ap.sta.ssid,ID_str);

	strcpy((char*)wifi_config_sta.sta.ssid,"MACRPT-");		// SSID is 32 chars max
	strcat((char*)wifi_config_sta.sta.ssid,ID_str);
*/
		
// STA mode for MAC counting
//    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
//    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
// AP mode for SSID broadcast...
//    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP) );
//    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config) );
// combined AP \ STA mode
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA) );
// call twice, once for AP mode, once for STA mode; 
// use seperate config arrays between calls for AP and STA!
/*
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config_ap) );

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config_sta) );
*/

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP,  &wifi_config_ap) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config_sta) );


    ESP_ERROR_CHECK(esp_wifi_start() );

	printf("Wifi  Configured\r\n");
//	printf("SSID = %s\r\n",wifi_config_sta.sta.ssid);
	printf("STA SSID = %s\r\n",wifi_config_sta.sta.ssid);
	printf("AP  SSID = %s\r\n",wifi_config_ap.ap.ssid);
#ifdef WIFI_HIDE_SSID
	printf("SSID is HIDDEN\r\n");
#endif
    ESP_LOGI(TAG, "wifi_init_sta finished.");
	
}

unsigned char wifi_connect_retry_count = 0;
extern unsigned char wifi_sta_credentials_index;

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            if (debug_do(DBG_MAC_TRAFFIC_LL))
				ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
			ESP_LOGI(TAG, "WIFI CONNECT:");
			ESP_LOGI(TAG, "SSID: %s",wifi_config_sta.sta.ssid);
			ESP_LOGI(TAG, "PWD : %s",wifi_config_sta.sta.password);
            if (wifi_disable_flag == 0)
				ESP_ERROR_CHECK(esp_wifi_connect());
			//esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            if (debug_do(DBG_MAC_TRAFFIC_LL))
				ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
            //ESP_LOGI(TAG, "Got IP: %s\n",
            //         ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
//			ip_event_got_ip_t* event = (ip_event_got_ip_t*) event->event_info;
			ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->event_info.got_ip.ip_info.ip));
			xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
			wifi_sta_connected = 1;
			wifi_connect_retry_count = 0;

/*
need:
ip4_addr_t wifi_sta_ip;
ip4_addr_t wifi_sta_gw;
ip4_addr_t wifi_sta_msk;

			wifi_sta_ip = event->event_info.got_ip.ip_info.ip;
			wifi_sta_gw = event->event_info.got_ip.ip_info.gw;
			wifi_sta_msk = event->event_info.got_ip.ip_info.netmask;
			
*/            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            if (debug_do(DBG_MAC_TRAFFIC_LL))
				ESP_LOGI(TAG, "SYSTEM_EVENT_STA_DISCONNECTED");
			ESP_LOGI(TAG, "WIFI RECONNECT:");
			ESP_LOGI(TAG, "SSID: %s",wifi_config_sta.sta.ssid);
			ESP_LOGI(TAG, "PWD : %s",wifi_config_sta.sta.password);
            if (wifi_disable_flag == 0)
				ESP_ERROR_CHECK(esp_wifi_connect());
			//esp_wifi_connect();

			wifi_connect_retry_count++;
// count connect attempts to see if another SSID \ password should be tried fom the list...			
			if (wifi_connect_retry_count > 3)
				{
#if 0
				printf("Retry >3!\n");
//unsigned char wifi_sta_credentials_index;

				wifi_sta_credentials_index++;
				if (wifi_sta_credentials_index > 3)
					{
					wifi_sta_credentials_index  = 0;
					}
// try next credentials on the list...
#endif				
				}
			wifi_sta_connected = 0;
            break;
		case SYSTEM_EVENT_SCAN_DONE:
			wifi_check_flag = 1;

            if (debug_do(DBG_MAC_TRAFFIC_LL))
				ESP_LOGI(TAG, "SYSTEM_EVENT_SCAN_DONE");
			break;

#ifdef USE_WEBSERVER
// extra cases for Ap use in webserver
		case SYSTEM_EVENT_AP_START:                 /*!< ESP32 soft-AP start */
            if (debug_do(DBG_MAC_TRAFFIC_LL))
				ESP_LOGI(TAG, "SYSTEM_EVENT_AP_START");
			
			esp_wifi_connect();
            break;
		case SYSTEM_EVENT_AP_STOP:                  /*!< ESP32 soft-AP stop */
            if (debug_do(DBG_MAC_TRAFFIC_LL))
				ESP_LOGI(TAG, "SYSTEM_EVENT_AP_STOP");
            break;
		case SYSTEM_EVENT_AP_STACONNECTED:          /*!< a station connected to ESP32 soft-AP */
            if (debug_do(DBG_MAC_TRAFFIC_LL))
				ESP_LOGI(TAG, "SYSTEM_EVENT_AP_STACONNECTED");
            break;
		case SYSTEM_EVENT_AP_STADISCONNECTED:       /*!< a station disconnected from ESP32 soft-AP */
            if (debug_do(DBG_MAC_TRAFFIC_LL))
				ESP_LOGI(TAG, "SYSTEM_EVENT_AP_STADISCONNECTED");
			
			esp_wifi_connect();
			
			if (http_server) 
				{
				ESP_LOGI(TAG, "Stopping webserver");
				stop_webserver(http_server);
				http_server = NULL;
				}

            break;
		case SYSTEM_EVENT_AP_STAIPASSIGNED:         /*!< ESP32 soft-AP assign an IP to a connected st */
            if (debug_do(DBG_MAC_TRAFFIC_LL))
				ESP_LOGI(TAG, "SYSTEM_EVENT_AP_STAIPASSIGNED");
			
		    if (http_server == NULL) 
				{
				ESP_LOGI(TAG, "Starting webserver");
				http_server = start_webserver();
				}

				
            break;
#endif

		default:
            if (debug_do(DBG_MAC_TRAFFIC_LL))
				ESP_LOGI(TAG, "UNKNOWN SYSTEM_EVENT %d",event->event_id);
            break;
    }
    return ESP_OK;
}



