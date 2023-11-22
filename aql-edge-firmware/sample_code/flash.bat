@echo off
if [%2]==[] goto FLASH

proj -s %2
if %ERRORLEVEL% GTR 0 goto BADEND

:FLASH
REM force recompile of __DATE__, __TIME__
touch main\MAC_Rpt_rtns.cpp

REM compile and load
if "%1" == "" (SET cp="com14") else (SET cp=%1)
echo Using COM port %cp%

idf.py -p %cp% -b 460800 flash
goto GOODEND

:BADEND
echo Can't set project to %2 !

:GOODEND
