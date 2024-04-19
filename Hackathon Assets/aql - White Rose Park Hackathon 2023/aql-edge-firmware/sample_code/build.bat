@echo off

if [%1]==[] goto BUILD

proj -s %1
if %ERRORLEVEL% GTR 0 goto BADEND

:BUILD
idf.py build
goto GOODEND

:BADEND
echo Can't set project to %1 !

:GOODEND


