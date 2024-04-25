//////////////////////////////////////////////
//
// MAC_Rpt_four_g.h
//
// aql Ltd
//
// Auth: DLT
//
//////////////////////////////////////////////

#ifndef MAC_RPT_FOUR_G_MQTT_H
#define MAC_RPT_FOUR_G_MQTT_H

#include "MAC_Rpt_four_g_mqtt_defs.h"

void four_g_mqtt_state_machine(unsigned char *four_g_state,unsigned char *prev_four_g_state);
unsigned char check_topic_str(void);


#endif
