//////////////////////////////////////////////
//
// MAC_Rpt_http_server.cpp
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

/* Simple HTTP Server Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "nvs_flash.h"
//#include "esp_netif.h"
#include "esp_eth.h"
//#include "protocol_examples_common.h"
//#include "esp_tls_crypto.h"
#include <esp_http_server.h>

#include "MAC_Rpt.h"
#include "MAC_Rpt_rtns.h"

#include "MAC_Rpt_http_index_page.h"

/* A simple example that demonstrates how to create GET and POST
 * handlers for the web server.
 */

static const char *TAG = "http server";

unsigned char testvar = 0;

unsigned char swval = 0;

extern unsigned char x100msec, secs, mins, hrs;
extern unsigned char days, month, year;
extern unsigned char day_of_week;

extern wifi_config_t wifi_config_sta, wifi_config_ap;
extern struct EVENT_INFO event_list[NUM_EVENTS_MAX];
extern unsigned char evt_start_stop_flag;
extern unsigned char evt_curr_PWM;

#ifdef CONFIG_EXAMPLE_BASIC_AUTH

typedef struct {
    char    *username;
    char    *password;
} basic_auth_info_t;

#define HTTPD_401      "401 UNAUTHORIZED"           /*!< HTTP Response 401 */



static char *http_auth_basic(const char *username, const char *password)
{
    int out;
    char *user_info = NULL;
    char *digest = NULL;
    size_t n = 0;
    asprintf(&user_info, "%s:%s", username, password);
    if (user_info==NULL) {
        ESP_LOGE(TAG, "Not enough memory for user information");
        return NULL;
    }
    esp_crypto_base64_encode(NULL, 0, &n, (const unsigned char *)user_info, strlen(user_info));

    /* 6: The length of the "Basic " string
     * n: Number of bytes for a base64 encode format
     * 1: Number of bytes for a reserved which be used to fill zero
    */
    digest = calloc(1, 6 + n + 1);
    if (digest!=NULL) {
		if (debug_do(DBG_ALLOC_MEM))
			printf("HTTP Srvr auth alloc %d\n", 6 + n + 1);

        strcpy(digest, "Basic ");
        esp_crypto_base64_encode((unsigned char *)digest + 6, n, (size_t *)&out, (const unsigned char *)user_info, strlen(user_info));
    }
    free(user_info);
    return digest;
}

/* An HTTP GET handler */
static esp_err_t basic_auth_get_handler(httpd_req_t *req)
{
    char *buf = NULL;
    size_t buf_len = 0;
    basic_auth_info_t *basic_auth_info = req->user_ctx;

	printf("## AUTH: %s %sd \n",basic_auth_info->username, basic_auth_info->password);
	
    buf_len = httpd_req_get_hdr_value_len(req, "Authorization") + 1;
    if (buf_len > 1) 
		{
        buf = calloc(1, buf_len);
        if (buf==NULL) 
			{
            ESP_LOGE(TAG, "Not enough memory for basic authorization");
            return ESP_ERR_NO_MEM;
			}

        if (httpd_req_get_hdr_value_str(req, "Authorization", buf, buf_len) == ESP_OK) 
			{
            ESP_LOGI(TAG, "Found header => Authorization: %s", buf);
			} 
		else 
			{
            ESP_LOGE(TAG, "No auth value received");
			}

        char *auth_credentials = http_auth_basic(basic_auth_info->username, basic_auth_info->password);
        if (!auth_credentials) 
			{
            ESP_LOGE(TAG, "No enough memory for basic authorization credentials");
            free(buf);
            return ESP_ERR_NO_MEM;
			}

        if (strncmp(auth_credentials, buf, buf_len)) 
			{
            ESP_LOGE(TAG, "Not authenticated");
            httpd_resp_set_status(req, HTTPD_401);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_hdr(req, "Connection", "keep-alive");
            httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"Hello\"");
            httpd_resp_send(req, NULL, 0);
			} 
		else 
			{
            ESP_LOGI(TAG, "Authenticated!");
            char *basic_auth_resp = NULL;
            httpd_resp_set_status(req, HTTPD_200);
            httpd_resp_set_type(req, "application/json");
            httpd_resp_set_hdr(req, "Connection", "keep-alive");
            asprintf(&basic_auth_resp, "{\"authenticated\": true,\"user\": \"%s\"}", basic_auth_info->username);
            if (!basic_auth_resp) 
				{
                ESP_LOGE(TAG, "No enough memory for basic authorization response");
                free(auth_credentials);
                free(buf);
                return ESP_ERR_NO_MEM;
				}
            httpd_resp_send(req, basic_auth_resp, strlen(basic_auth_resp));
            free(basic_auth_resp);
			}
        free(auth_credentials);
        free(buf);
		} 
	else 
		{
        ESP_LOGE(TAG, "No auth header received");
        httpd_resp_set_status(req, HTTPD_401);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_set_hdr(req, "Connection", "keep-alive");
        httpd_resp_set_hdr(req, "WWW-Authenticate", "Basic realm=\"Hello\"");
        httpd_resp_send(req, NULL, 0);
		}

    return ESP_OK;
}




static httpd_uri_t basic_auth = {
    .uri       = "/basic_auth",
    .method    = HTTP_GET,
    .handler   = basic_auth_get_handler,
};

static void httpd_register_basic_auth(httpd_handle_t server)
{
    basic_auth_info_t *basic_auth_info = calloc(1, sizeof(basic_auth_info_t));
    if (basic_auth_info) 
		{
        basic_auth_info->username = CONFIG_EXAMPLE_BASIC_AUTH_USERNAME;
        basic_auth_info->password = CONFIG_EXAMPLE_BASIC_AUTH_PASSWORD;

        basic_auth.user_ctx = basic_auth_info;
        httpd_register_uri_handler(server, &basic_auth);

		printf("## REG: %s %sd \n",basic_auth_info->username, basic_auth_info->password);

		free(basic_auth_info);	// added 26/06/23 
		}
}
#endif



////////////////////////////////////////////////////////
// declare the handler routines...
////////////////////////////////////////////////////////

/* An HTTP GET handler */
static esp_err_t hello_get_handler(httpd_req_t *req)
{
    char*  buf;
    size_t buf_len;

	printf("HELLO GET handler...\n");
    /* Get header value string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
    if (buf_len > 1) 
		{
        buf = (char*)malloc(buf_len);
        /* Copy null terminated value string into buffer */
        if (httpd_req_get_hdr_value_str(req, "Host", buf, buf_len) == ESP_OK)
			{
            ESP_LOGI(TAG, "Found header => Host: %s", buf);
			}
        free(buf);
		}

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-2") + 1;
    if (buf_len > 1) 
		{
        buf = (char*)malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-2", buf, buf_len) == ESP_OK) 
			{
            ESP_LOGI(TAG, "Found header => Test-Header-2: %s", buf);
			}
        free(buf);
		}

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-1") + 1;
    if (buf_len > 1) 
		{
        buf = (char*)malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-1", buf, buf_len) == ESP_OK) 
			{
            ESP_LOGI(TAG, "Found header => Test-Header-1: %s", buf);
			}
        free(buf);
		}

    /* Read URL query string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) 
		{
        buf = (char*)malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) 
			{
            ESP_LOGI(TAG, "Found URL query => %s", buf);
            char param[32];
            /* Get value of expected key from query string */
            if (httpd_query_key_value(buf, "query1", param, sizeof(param)) == ESP_OK) 
				{
                ESP_LOGI(TAG, "Found URL query parameter => query1=%s", param);
				}
            if (httpd_query_key_value(buf, "query3", param, sizeof(param)) == ESP_OK) 
				{
                ESP_LOGI(TAG, "Found URL query parameter => query3=%s", param);
				}
            if (httpd_query_key_value(buf, "query2", param, sizeof(param)) == ESP_OK) 
				{
                ESP_LOGI(TAG, "Found URL query parameter => query2=%s", param);
				}
			}
        free(buf);
    }

    /* Set some custom headers */
    httpd_resp_set_hdr(req, "Custom-Header-1", "Custom-Value-1");
    httpd_resp_set_hdr(req, "Custom-Header-2", "Custom-Value-2");

    /* Send response with custom headers and body set as the
     * string passed in user context*/
    const char* resp_str = (const char*) req->user_ctx;
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

    /* After sending the HTTP response the old HTTP request
     * headers are lost. Check if HTTP request headers can be read now. */
    if (httpd_req_get_hdr_value_len(req, "Host") == 0) {
        ESP_LOGI(TAG, "Request headers lost = OK");
    }
    return ESP_OK;
}

/* An HTTP GET handler */
static esp_err_t indexpage_get_handler(httpd_req_t *req)
{
    char*  buf;
    size_t buf_len;
	unsigned char val;
	
	printf("INDEX GET handler...\n");
    /* Get header value string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
    if (buf_len > 1) 
		{
        buf = (char*)malloc(buf_len);
        /* Copy null terminated value string into buffer */
        if (httpd_req_get_hdr_value_str(req, "Host", buf, buf_len) == ESP_OK) 
			{
            ESP_LOGI(TAG, "Found header => Host: %s", buf);
			}
        free(buf);
		}

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-2") + 1;
    if (buf_len > 1) 	
		{
        buf = (char*)malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-2", buf, buf_len) == ESP_OK) 
			{
            ESP_LOGI(TAG, "Found header => Test-Header-2: %s", buf);
			}
        free(buf);
		}

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-1") + 1;
    if (buf_len > 1) 
		{
        buf = (char*)malloc(buf_len);
        if (httpd_req_get_hdr_value_str(req, "Test-Header-1", buf, buf_len) == ESP_OK) 
			{
            ESP_LOGI(TAG, "Found header => Test-Header-1: %s", buf);
			}
        free(buf);
		}

    /* Read URL query string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) 
		{
        buf = (char*)malloc(buf_len);
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) 
			{
            ESP_LOGI(TAG, "Found URL query => %s", buf);
            char param[32];
            /* Get value of expected key from query string */
            if (httpd_query_key_value(buf, "grp1", param, sizeof(param)) == ESP_OK) 
				{
                ESP_LOGI(TAG, "Found URL query parameter => grp1=%s", param);
				val = atoi(param);
				printf("Got value %d\n",val);

/*
				switch (val)
					{
					case 0:							// OFF
						printf("Stop\n");
						stop_event(EVENT_MANUAL);
//						send_notificaton_flag = 1;
						break;
					case 1:							// ON
						printf("Start\n");
						start_event(EVENT_MANUAL,50);
//						send_notificaton_flag = 1;
						break;
					case 2:							// SCHEDULER
						printf("Scheduler\n");
//						stop_event(EVENT_MANUAL);
//						send_notificaton_flag = 1;
						break;
					default:
						break;
					}
*/
				swval = val;
				switch (swval)
					{
#ifdef LIFT_SYSTEM
					case 0:							// OFF
						printf("Stop\n");
						stop_event(EVENT_MANUAL);
						break;
					case 1:							// ON
						printf("Start 25%%\n");
						start_event(EVENT_MANUAL,25);
						break;
					case 2:							// ON
						printf("Start 50%%\n");
						start_event(EVENT_MANUAL,50);
						break;
					case 3:							// ON
						printf("Start 75%%\n");
						start_event(EVENT_MANUAL,75);
						break;
					case 4:							// ON
						printf("Start 100%%\n");
						start_event(EVENT_MANUAL,100);
						break;
#endif
					default:
						break;
					}
				
				}
			}
        free(buf);
		}

    /* Set some custom headers */
    httpd_resp_set_hdr(req, "Custom-Header-1", "Custom-Value-1");
    httpd_resp_set_hdr(req, "Custom-Header-2", "Custom-Value-2");


    /* Send response with custom headers and body set as the
     * string passed in user context*/
    const char* resp_str = (const char*) req->user_ctx;
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

    /* After sending the HTTP response the old HTTP request
     * headers are lost. Check if HTTP request headers can be read now. */
    if (httpd_req_get_hdr_value_len(req, "Host") == 0) {
        ESP_LOGI(TAG, "Request headers lost");
    }
    return ESP_OK;
}


/* An HTTP GET handler */
static esp_err_t POT_get_handler(httpd_req_t *req)
{
	char resp_str[2000];
	char tmpstr[200];
    char mac_str[20];
    uint8_t mac_addr[6];

	unsigned char i;

//    char*  buf;
 //   size_t buf_len;

	if (debug_do(DBG_WEBSERVER))
		printf("POT GET handler...   ");
	
    /* Set some custom headers */
    httpd_resp_set_hdr(req, "POTval", "99");
    httpd_resp_set_type(req, "text/xml");

 

    /* Send response with custom headers and body set as the
     * string passed in user context*/
//    const char* resp_str = "108";	//(const char*) req->user_ctx;
	
	sprintf(resp_str,"<xml><response>");

//	sprintf(resp_str,"%d,",testvar++);
	sprintf(tmpstr,"<tst>%d</tst>",testvar++);
	strcat(resp_str,tmpstr);

    esp_read_mac(mac_addr,ESP_MAC_WIFI_SOFTAP);
	mac_to_str(mac_addr, '-', mac_str);

	sprintf(tmpstr,"<uid>%s</uid>",(char*)mac_str);
	strcat(resp_str,tmpstr);

	sprintf(tmpstr,"<time>%s  %02d:%02d:%02d</time>",days_of_week[day_of_week],hrs,mins,secs);
	strcat(resp_str,tmpstr);

	for (i=0;i<NUM_EVENTS_MAX;i++)
		{
//		sprintf(tmpstr,"<evt%d>",i);
//		strcat(resp_str,tmpstr);
		get_event_line_html(i, tmpstr, event_list);
		strcat(resp_str,tmpstr);
//		sprintf(tmpstr,"</evt%d>",i);
//		strcat(resp_str,tmpstr);
		}

/*
	if (evt_start_stop_flag == EVENT_STOP)
		{
		sprintf(tmpstr,"<stat>OFF</stat><bon>0</bon><boff>1</boff><bsch>0</bsch>");
		}
	else
		{
		sprintf(tmpstr,"<stat>ON</stat><bon>1</bon><boff>0</boff><bsch>0</bsch>");
		}
*/
		if (swval)
			sprintf(tmpstr,"<stat>ON</stat><bon>%d</bon><boff>0</boff><bsch>0</bsch>",swval);
		else
			sprintf(tmpstr,"<stat>OFF</stat><bon>0</bon><boff>0</boff><bsch>0</bsch>");
			
//		sprintf(tmpstr,"<stat>OFF</stat><bon>%d</bon><boff>0</boff><bsch>0</bsch>",swval);
		
	strcat(resp_str,tmpstr);

	sprintf(tmpstr,"<pwm>%d%%</pwm>",evt_curr_PWM);
	strcat(resp_str,tmpstr);
	
	sprintf(tmpstr,"</response></xml>");
	strcat(resp_str,tmpstr);
/*
	strcat(resp_str,(char*)wifi_config.sta.ssid);
	sprintf(tmpstr,",%d  %02d:%02d:%02d,",day_of_week,hrs,mins,secs);
	strcat(resp_str,tmpstr);
	
	for (i=0;i<NUM_EVENTS_MAX;i++)
		{
		get_event_line(i, tmpstr, event_list);
		strcat(resp_str,tmpstr);
		strcat(resp_str,"\r\n,");
		}
*/
	if (debug_do(DBG_WEBSERVER))
		printf("Resp len: %d\n",strlen(resp_str));
	
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);

    /* After sending the HTTP response the old HTTP request
     * headers are lost. Check if HTTP request headers can be read now. */
    if (httpd_req_get_hdr_value_len(req, "Host") != 0) {
        ESP_LOGI(TAG, "Request headers NOT released!");
    }
    return ESP_OK;
}

/* An HTTP POST handler */
static esp_err_t grp1_post_handler(httpd_req_t *req)
{
    char buf[100];
    int ret, remaining = req->content_len;

	if (debug_do(DBG_WEBSERVER))
		printf("GRP1 POST handler...\n");
 
 while (remaining > 0) {
        /* Read the data for the request */
        if ((ret = httpd_req_recv(req, buf,
                        MIN(remaining, sizeof(buf)))) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                /* Retry receiving if timeout occurred */
                continue;
            }
            return ESP_FAIL;
        }

        /* Send back the same data */
        httpd_resp_send_chunk(req, buf, ret);
        remaining -= ret;

        /* Log data received */
        ESP_LOGI(TAG, "=========== RECEIVED DATA ==========");
        ESP_LOGI(TAG, "%.*s", ret, buf);
        ESP_LOGI(TAG, "====================================");
    }

    // End response
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}
/* An HTTP POST handler */
static esp_err_t echo_post_handler(httpd_req_t *req)
{
    char buf[100];
    int ret, remaining = req->content_len;

	if (debug_do(DBG_WEBSERVER))
		printf("ECHO GET handler...\n");
 
 while (remaining > 0) {
        /* Read the data for the request */
        if ((ret = httpd_req_recv(req, buf,
                        MIN(remaining, sizeof(buf)))) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                /* Retry receiving if timeout occurred */
                continue;
            }
            return ESP_FAIL;
        }

        /* Send back the same data */
        httpd_resp_send_chunk(req, buf, ret);
        remaining -= ret;

        /* Log data received */
        ESP_LOGI(TAG, "=========== RECEIVED DATA ==========");
        ESP_LOGI(TAG, "%.*s", ret, buf);
        ESP_LOGI(TAG, "====================================");
    }

    // End response
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}


/* This handler allows the custom error handling functionality to be
 * tested from client side. For that, when a PUT request 0 is sent to
 * URI /ctrl, the /hello and /echo URIs are unregistered and following
 * custom error handler http_404_error_handler() is registered.
 * Afterwards, when /hello or /echo is requested, this custom error
 * handler is invoked which, after sending an error message to client,
 * either closes the underlying socket (when requested URI is /echo)
 * or keeps it open (when requested URI is /hello). This allows the
 * client to infer if the custom error handler is functioning as expected
 * by observing the socket state.
 */
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
	if (debug_do(DBG_WEBSERVER))
		printf("404 Error handler...\n");
 
 if (strcmp("/hello", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/hello URI is not available");
        /* Return ESP_OK to keep underlying socket open */
        return ESP_OK;
    } else if (strcmp("/echo", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/echo URI is not available");
        /* Return ESP_FAIL to close underlying socket */
        return ESP_FAIL;
    }
    /* For any other URI send 404 and close socket */
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Some 404 error message");
    return ESP_FAIL;
}




////////////////////////////////////////////////////////
// associate the handler routines...
////////////////////////////////////////////////////////

httpd_uri_t hello = {
    .uri       = "/hello",
    .method    = HTTP_GET,
    .handler   = hello_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  = (void*)"Hello World!"
};

static const httpd_uri_t echo = {
    .uri       = "/echo",
    .method    = HTTP_POST,
    .handler   = echo_post_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t index_only = {		// "index" is already used as a system var!
    .uri       = "/index",
    .method    = HTTP_GET,
    .handler   = indexpage_get_handler,
//    .user_ctx  = (void*)"Index only"
    .user_ctx  = (void*)indexpageCode
};

static const httpd_uri_t index_htm = {
    .uri       = "/index.htm",
    .method    = HTTP_GET,
    .handler   = indexpage_get_handler,
    .user_ctx  = (void*)"Index.htm"
};

static const httpd_uri_t index_html = {
    .uri       = "/index.html",
    .method    = HTTP_GET,
    .handler   = indexpage_get_handler,
    .user_ctx  = (void*)"Index.html"
};

#if 1
static const httpd_uri_t readPOT = {
    .uri       = "/readPOT",
    .method    = HTTP_GET,
    .handler   = POT_get_handler,
    .user_ctx  = NULL //(void*)"99"
};

static const httpd_uri_t grp1 = {
    .uri       = "/grp1",
    .method    = HTTP_POST,
    .handler   = grp1_post_handler,
    .user_ctx  = NULL
};
#endif



////////////////////////////////////////////////////////////////////
// this routine registers \ deregisters the above routines,
// so must be declared after them...
////////////////////////////////////////////////////////////////////

/* An HTTP PUT handler. This demonstrates realtime
 * registration and deregistration of URI handlers
 */
static esp_err_t ctrl_put_handler(httpd_req_t *req)
{
    char buf;
    int ret;

	if (debug_do(DBG_WEBSERVER))
		printf("CTRL PUT handler...\n");

    if ((ret = httpd_req_recv(req, &buf, 1)) <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }

    if (buf == '0') {
        /* URI handlers can be unregistered using the uri string */
        ESP_LOGI(TAG, "Unregistering /hello and /echo URIs");
        httpd_unregister_uri(req->handle, "/hello");
        httpd_unregister_uri(req->handle, "/echo");
        httpd_unregister_uri(req->handle, "/grp1");
        httpd_unregister_uri(req->handle, "/index");
        httpd_unregister_uri(req->handle, "/index.htm");
        httpd_unregister_uri(req->handle, "/index.html");
        httpd_unregister_uri(req->handle, "/readPOT");
        /* Register the custom error handler */
        httpd_register_err_handler(req->handle, HTTPD_404_NOT_FOUND, http_404_error_handler);
    }
    else {
        ESP_LOGI(TAG, "Registering /hello and /echo URIs");
        httpd_register_uri_handler(req->handle, &hello);
        httpd_register_uri_handler(req->handle, &index_only);
        httpd_register_uri_handler(req->handle, &index_htm);
        httpd_register_uri_handler(req->handle, &index_html);
        httpd_register_uri_handler(req->handle, &readPOT);
        httpd_register_uri_handler(req->handle, &echo);
        httpd_register_uri_handler(req->handle, &grp1);
        /* Unregister custom error handler */
        httpd_register_err_handler(req->handle, HTTPD_404_NOT_FOUND, NULL);
    }

    /* Respond with empty body */
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t ctrl = {
    .uri       = "/ctrl",
    .method    = HTTP_PUT,
    .handler   = ctrl_put_handler,
    .user_ctx  = NULL
};







//static httpd_handle_t start_webserver(void)
httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

	config.stack_size = 6000;

	printf("httpd stack:  %d\n",config.stack_size);
	printf("Webpage size: %d\n",strlen(indexpageCode));
	
    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: %d", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &hello);
        httpd_register_uri_handler(server, &index_only);
        httpd_register_uri_handler(server, &index_htm);
        httpd_register_uri_handler(server, &index_html);
        httpd_register_uri_handler(server, &readPOT);
        httpd_register_uri_handler(server, &echo);
        httpd_register_uri_handler(server, &grp1);
        httpd_register_uri_handler(server, &ctrl);
        #ifdef CONFIG_EXAMPLE_BASIC_AUTH
        httpd_register_basic_auth(server);
        #endif
		ESP_LOGI(TAG, "Server started");
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

//static void stop_webserver(httpd_handle_t server)
void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_stop(server);
}

void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
	printf("Disconnect handler...\n");
	
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Webserver disconnected");
        stop_webserver(*server);
        *server = NULL;
    }

}

void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Connecting webserver");
        *server = start_webserver();
    }
}

#if 0
//left as example only - do this in the main() function...
void app_main(void)
{
    static httpd_handle_t server = NULL;

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    /* Register event handlers to stop the server when Wi-Fi or Ethernet is disconnected,
     * and re-start it upon connection.
     */
#ifdef CONFIG_EXAMPLE_CONNECT_WIFI
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
#endif // CONFIG_EXAMPLE_CONNECT_WIFI
#ifdef CONFIG_EXAMPLE_CONNECT_ETHERNET
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ETHERNET_EVENT_DISCONNECTED, &disconnect_handler, &server));
#endif // CONFIG_EXAMPLE_CONNECT_ETHERNET

    /* Start the server for the first time */
    server = start_webserver();
}
#endif
