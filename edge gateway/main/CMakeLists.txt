
#target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "main/client.crt" TEXT)
#target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "main/client.key" TEXT)



idf_component_register(SRCS "MAC_Rpt_main.cpp" "MAC_Rpt_rtns.cpp" "MAC_Rpt_wifi.cpp" "MAC_Rpt_radio.cpp" "MAC_Rpt_four_g.cpp" "MAC_Rpt_four_g_http.cpp" "MAC_Rpt_four_g_simcom.cpp" "MAC_Rpt_four_g_mqtt.cpp" "MAC_Rpt_four_g_udp.cpp" "BlueTooth.cpp" "MAC_Rpt_http_server.cpp" "MAC_Rpt_uart_intr.cpp"  "MAC_Rpt_SX1276.cpp" "Mac_Rpt_LoRa.cpp"   "MAC_Rpt_wifi_mqtt.cpp" "MAC_Rpt_wifi_udp.cpp"  
INCLUDE_DIRS "" )
#$ENV{IDF_PATH}/examples/common_components/protocol_examples_common)
#REQUIRES protocol_examples_common)

