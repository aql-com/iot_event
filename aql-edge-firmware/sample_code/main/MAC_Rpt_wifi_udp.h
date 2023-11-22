//////////////////////////////////////////////
//
// MAC_Rpt_wifi_udp.h
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

void wifi_udp_dns_found_cb(const char *name, const ip_addr_t *ipaddr, void *callback_arg);
unsigned char wifi_udp_connect(char* udp_url, ip_addr_t* udp_ip_addr);

void print_wifi_udp_state(unsigned char chan);
