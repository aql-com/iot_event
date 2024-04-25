# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# Wifi MQTT uses an extra component for common functions such as Wi-Fi and Ethernet connection.
set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(MAC_Rpt)

message("src dir is ${CMAKE_SOURCE_DIR}")

include_directories(${CMAKE_SOURCE_DIR}/main)

#target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "SSL/aql4gboard.crt" TEXT)
#target_add_binary_data(${CMAKE_PROJECT_NAME}.elf "SSL/aql4gboard.key" TEXT)

