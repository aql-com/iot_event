@echo off
if [%1]==[] goto INFO
xtensa-esp32-elf-addr2line -pfiaC -e build/MAC_Rpt.elf %1
goto END
:INFO
echo addr2line address decoder:
echo type
echo    a2l 0x[==PC value==]
:END

