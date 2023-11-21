rem old.bat

@echo off

echo copying old files into project...

copy main\MAC_Rpt_four_g_mqtt*.* main\New

del main\MAC_Rpt_four_g_mqtt*.*

copy main\Old\MAC_Rpt_four_g_mqtt*.* main
