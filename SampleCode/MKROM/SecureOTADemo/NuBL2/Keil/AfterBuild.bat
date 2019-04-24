::@echo off

::NuBL2 Firmware Info generation
FwSign.exe .\NuBL2_FW.bin\FLASH .\NuBL2_FW.bin\FWINFO 0x5E800

::NuBL2 function pointer generation
XOMAddr .\lst\NuBL2_FW.map ..\lib\NuBL2lib.c

