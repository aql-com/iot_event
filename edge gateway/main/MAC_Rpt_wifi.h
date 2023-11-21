//////////////////////////////////////////////
//
// MAC_Rpt_wifi.h
//

// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

#ifndef MAC_RPT_WIFI_H
#define MAC_RPT_WIFI_H

void wifi_init_sta(void);

void wifi_init_sniff(void);


typedef struct {
// see       https://en.wikipedia.org/wiki/IEEE_802.11#Layer_2_%E2%80%93_Datagrams
  unsigned frame_ctrl:16;		// data bits	
  unsigned duration_id:16;		// data bits
  uint8_t addr1[6]; 			/* receiver address bytes */
  uint8_t addr2[6]; 			/* sender address bytes */
  uint8_t addr3[6]; 			/* filtering address bytes */
  unsigned sequence_ctrl:16;	// data bits
  uint8_t addr4[6]; 			/* optional bytes */
} wifi_ieee80211_mac_hdr_t;

typedef struct {
// see       https://en.wikipedia.org/wiki/IEEE_802.11#Layer_2_%E2%80%93_Datagrams
  unsigned frame_ctrl:16;		// data bits	
  unsigned duration_id:16;		// data bits
  uint8_t addr1[6]; 			/* receiver address bytes */
  uint8_t addr2[6]; 			/* sender address bytes */
  uint8_t addr3[6]; 			/* filtering address bytes */
  unsigned sequence_ctrl:16;	// data bits
} wifi_ieee80211_short_mac_hdr_t;

typedef struct {
  wifi_ieee80211_mac_hdr_t hdr;
  uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;

typedef struct {
  wifi_ieee80211_short_mac_hdr_t hdr;
  uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_short_packet_t;

typedef struct
{
		unsigned protocol:2;
		unsigned type:2;
		unsigned subtype:4;
		unsigned to_ds:1;
		unsigned from_ds:1;
		unsigned more_frag:1;
		unsigned retry:1;
		unsigned pwr_mgmt:1;
		unsigned more_data:1;
		unsigned wep:1;
		unsigned strict:1;
} wifi_header_frame_control_t;

typedef struct
{
	unsigned interval:16;
	unsigned capability:16;
	unsigned tag_number:8;
	unsigned tag_length:8;
	char ssid[0];
	uint8_t rates[1];
} wifi_mgmt_beacon_t;

typedef struct
{
	unsigned tag_number:8;
	unsigned tag_length:8;
	char ssid[0];
	uint8_t rates[1];
} wifi_mgmt_probe_req_t;



#endif