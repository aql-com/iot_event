//////////////////////////////////////////////
//
// MAC_Rpt - MAC_Rpt_wifi_mqtt.cpp
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

// taken from    D:\ESP32esp-idf/examples/protocols/mqtt/ssl_mutual_auth/main/app_main.c

// other examples in 
// D:\ESP32\esp-idf\examples\common_components\protocol_examples_common\include
// D:\ESP32\esp-idf\components\mqtt\esp-mqtt\examples\mqtt_ssl_mutual_auth\main

// https://esp32tutorials.com/esp32-mqtt-client-publish-subscribe-esp-idf/

// and                
/* MQTT Mutual Authentication Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#if 1
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
#include "mqtt_client.h"

#include "MAC_Rpt.h"
#include "MAC_Rpt_Rtns.h"

#include "ca_cert.h"
#include "client_cert.h"
#include "client_key.h"

#ifdef USE_WIFI_MQTT
static const char *TAG = "MQTTS_WIFI";

/*
extern const unsigned int ca_cert_size;	// = 2162;
extern const unsigned char ca_cert_data[];	//2163] = 

extern const unsigned int client_cert_size;	// = 1670;
extern const unsigned char client_cert_data[];	//1671] = 

extern const unsigned int client_key_size;		// 1679	// defined in client_key.h
extern const unsigned char client_key_data[];	//1680];// defined in client_key.h
*/

/*
extern const uint8_t client_cert_pem_start[] asm("_binary_client_crt_start");
extern const uint8_t client_cert_pem_end[] asm("_binary_client_crt_end");
extern const uint8_t client_key_pem_start[] asm("_binary_client_key_start");
extern const uint8_t client_key_pem_end[] asm("_binary_client_key_end");
*/

extern esp_mqtt_client_handle_t mqtt_client;
extern char mac_bin_str[];
extern unsigned char wifi_mqtt_connected;
extern unsigned char wifi_mqtt_started;
extern unsigned char wifi_mqtt_attempts;

extern unsigned char mqtt_transport_mode;

extern QueueHandle_t mqtt_data_queue;
extern unsigned char CMQTT_rx_msg;

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
	char topic_str[80];
	char tmpstr[600];
	unsigned char i,j,k;
	
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_BEFORE_CONNECT:
            ESP_LOGI(TAG, "MQTT_EVENT_BEFORE_CONNECT");
            break;
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
// set up subscribe for commands from server
			strcpy(topic_str,"/gateway_dtm/");
			strcat(topic_str,mac_bin_str);
			msg_id = esp_mqtt_client_subscribe(client, topic_str, 0);
			wifi_mqtt_connected = 1;
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

//            msg_id = esp_mqtt_client_subscribe(client, "/topic/qos1", 1);
//            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

//            msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
//            ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:
			wifi_mqtt_connected = 0;
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
 //           msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
 //           ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC[%d]=%.*s\r\n", event->topic_len, event->topic_len, event->topic);
            printf("DATA[%d]=%.*s\r\n", event->data_len, event->topic_len, event->data);

			if (mqtt_transport_mode==MQTT_MODE_WIFI)	// only enqueue the data if WIFI is current transport mode...
				{
				strcpy(tmpstr,"+CMQTTRXTOPIC:\r\n");
				strncat(tmpstr,event->topic,event->topic_len);
				strcat(tmpstr,"\r\n");
				k = strlen(tmpstr);

//			ESP_LOGI(TAG, "%s  ",tmpstr);

				i = 0;
				j = pdTRUE;
				while ((i<k) && (j != pdFALSE))
					{
					j = xQueueSend(mqtt_data_queue, &tmpstr[i], 10 / portTICK_PERIOD_MS);
					i++;
					}

				strcpy(tmpstr,"+CMQTTRXPAYLOAD:\r\n");
				strncat(tmpstr,event->data,event->data_len);
				strcat(tmpstr,"\r\n");
				k = strlen(tmpstr);
				
	//			ESP_LOGI(TAG, "%s  ",tmpstr);

				i = 0;
				j = pdTRUE;
				while ((i<k) && (j != pdFALSE))
					{
					j = xQueueSend(mqtt_data_queue, &tmpstr[i], 10 / portTICK_PERIOD_MS);
					i++;
					}
				
				CMQTT_rx_msg++;
				}
				
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_app_start(void)
{
	/*
extern const unsigned int ca_cert_size;	// = 2162;
extern const unsigned char ca_cert_data[];	//2163] = 

extern const unsigned int client_cert_size;	// = 1670;
extern const unsigned char client_cert_data[];	//1671] = 

extern const unsigned int client_key_size;		// 1679	// defined in client_key.h
extern const unsigned char client_key_data[];	//1680];// defined in client_key.h

*/
esp_err_t err;

char topic_str[80];

	strcpy(topic_str,"/gateway_dom/frame/");
	strcat(topic_str,mac_bin_str);
	
    const esp_mqtt_client_config_t mqtt_cfg = {
//##        .event_handle = mqtt_event_handler,
//		.event_loop_handle = XXX,
//		.host = IP4_ADDR,
//##        .uri = "mqtts://test.mosquitto.org:8884",
//		.port = IP4_PORT,
//		.client_id STRING
//		.username STRING,
//		,passaword STRING,
//		.lwt_topic STRING,
//		.lwt_msg STRING,
// 		.lwt_qos = 
//		.lwt_retain = 
//		.lwt_msg_len = 
// 		.disable_clean_session = 
//		.keepalive = 120,
//		.disable_auto_reconnect = 
//		.user_context PTR
//		.task_prio = 
//		.task_stack = 6144
//		.buffer_size = 1024, // (only rcv buff if out_buffer defined)
//		.cert_pem	PTR
//		.cert_len = 
//##        .client_cert_pem = (const char *)client_cert_pem_start,	//		.client_cert_pem	PTR
//		.client_cert_len = 
//##        .client_key_pem = (const char *)client_key_pem_start,	//		.client_key_pem	PTR
//		.client_key_len = 
//		.transport = 
//		.refresh_connection_after_ms = 
//		.psk_hint_key PTR
//		.use_global_ca_store =
//		.reconnect_timeout_ms = 10000,	// msec
//		.alpn_protos PTR PTR
//		.clientkey_password PTR
//		.clientkey_password_len = 
//		.protocol_ver
//		.out_buffer_size = 
//		.skip_cert_common_name_check =- 
//		.use_secure_element = 
//		.ds_data PTR
//		.network_timeout_ms =10000	// msec
//		.disable_keepalive = 

        .event_handle = mqtt_event_handler,
        .uri = "mqtts://iot-visualiser.aql.com:8883",
		.lwt_topic = topic_str,
		.lwt_msg = "Wifi Disconnect",	
		.cert_pem = (const char *)&ca_cert_data[0],
        .client_cert_pem = (const char *)&client_cert_data[0],	//		.client_cert_pem	PTR
//		.client_cert_len = 
        .client_key_pem = (const char *)&client_key_data[0],	//		.client_key_pem	PTR
		.skip_cert_common_name_check = 1, 

    };

    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
//    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);			// mqtt_client defined in main.cpp

//	esp_mqtt_client_register_event(client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, client);
	if (mqtt_client != NULL)
		{
		err = esp_mqtt_client_start(mqtt_client);
		if (err)
			{
			wifi_mqtt_attempts++;
			wifi_mqtt_started = 0;
			}
		else
			{
			wifi_mqtt_attempts = 0;
			wifi_mqtt_started = 1;
			}

		}
	else
		{
		wifi_mqtt_attempts++;
		wifi_mqtt_started = 0;
		}
		
}

void wifi_mqtt_stop(void)
{
//    esp_mqtt_client_disconnect(mqtt_client);
    esp_mqtt_client_stop(mqtt_client);
	wifi_mqtt_started = 0;
}

void esp_mqtt_set_logging(void)
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);
}

void wifi_mqtt_start(void)
{
// if not already running..???
    mqtt_app_start();

}


#if 0
void app_main()
{
	esp_mqtt_set_logging();
/*	
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);
*/
    ESP_ERROR_CHECK(nvs_flash_init());
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

//     * This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
//     * Read "Establishing Wi-Fi or Ethernet Connection" section in
//     * examples/protocols/README.md for more information about this function.
//
 //   ESP_ERROR_CHECK(example_connect());

    mqtt_app_start();
}
#endif 	// end of "#if 0..."


void print_wifi_mqtt_state(unsigned char chan)
{
//dbgprintf(chan,"MQTT State:  [%03d] < MQTT_%s >    MQTT timer: %d\r\n",mqtt_state,mqtt_state_str[mqtt_state],mqtt_state_mc_timer);
dbgprintf(chan,"MQTT WIFI State :  %d\r\n",wifi_mqtt_connected);
}

#endif// end of "#ifdef USE_WIFI_MQTT..."


#endif
