/////////////////////////////////////////////
//
// MAC_Rpt_http_server.h
//
// aql Ltd
//
// Auth: DLT
//
// based on Espressif Systems code example from 
//
// https://github.com/espressif/esp-idf/blob/af28416116cd88791bdf103486be88915b111240/examples/protocols/http_server/simple/main/main.c
//
//////////////////////////////////////////////


void disconnect_handler(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data);

void connect_handler(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data);

httpd_handle_t start_webserver(void);
void stop_webserver(httpd_handle_t server);