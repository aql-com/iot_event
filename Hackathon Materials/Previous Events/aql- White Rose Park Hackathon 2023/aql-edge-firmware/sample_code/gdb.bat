REM gdb for windows doesnt like ports >com9...
REM in menuconfig -> COmponent COnfig -> ESP32 Specific -> Panic Handler, set "Invoke GDB stub"

xtensa-esp32-elf-gdb -b 115200 -ex "target remote COM2" -ex interrupt build/MAC_Rpt.elf 
