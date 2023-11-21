//////////////////////////////////////////////
//
// MAC_Rpt - MAC_Rpt_wifi_mqtt.h
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

#ifndef MAC_RPT_WIFI_MQTT_H
#define MAC_RPT_WIFI_MQTT_H

#include "mqtt_client.h"

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event);

static void mqtt_app_start(void);

#warning "Using wifi MQTT defs!"

void esp_mqtt_set_logging(void);

void wifi_mqtt_start(void);
void wifi_mqtt_stop(void);

void print_wifi_mqtt_state(unsigned char chan);


#endif