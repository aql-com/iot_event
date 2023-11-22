@echo off
echo ####################################
echo # STARTING COMPILER ENVIRONMENT...
echo # MAC RPT...
echo ####################################

D:
cd \ESP32\esp-idf
call D:\ESP32\.espressif\idf_cmd_init.bat "C:\Users\DLT\AppData\Local\Programs\Python\Python37\" "C:\Program Files\Git\cmd"

echo Or use the build and flash .bat files...
F:
cd "\Projects\FDT\MAC_Report\AQL_Code_GIT\sample_code"

echo Current project is:

proj
