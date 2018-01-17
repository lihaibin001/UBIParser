@echo off
echo.
echo ******************* Creates Combo hex out from bootloader and app hex ****************** 
echo.

SET BUILD_OUTPUT_DIR=..\CORTEX_STM32F103_IAR\Debug\Exe
SET BL_BUILD_OUTPUT_DIR=..\bootloader\IAR_STM32_BOOT\Debug\Exe

SET BOOTLOADER_NAME=DR002_bootloader
SET APPLICATION_NAME=DR002_STM32_MB
SET OUTPUT_NAME=DR002_manufacturing

SET UTILITY_TOOL=ptp_utility_v9.3.D.exe


echo ***copy %APPLICATION_NAME%.hex into output directory***
copy %BUILD_OUTPUT_DIR%\%APPLICATION_NAME%.hex .
copy %BL_BUILD_OUTPUT_DIR%\%BOOTLOADER_NAME%.hex .

echo ***convert bootloader %BOOTLOADER_NAME%.hex into ptp***
%UTILITY_TOOL% convert -replace -inputType=ihex -output=%BOOTLOADER_NAME%.ptp %BOOTLOADER_NAME%.hex

echo ***convert application %APPLICATION_NAME%.hex into ptp***
%UTILITY_TOOL% convert -replace -inputType=ihex -output=%APPLICATION_NAME%.ptp %APPLICATION_NAME%.hex

echo ***Convert %APPLICATION_NAME%.ptp into %APPLICATION_NAME%_OTA_APP.bin***
%UTILITY_TOOL% convert -replace -toType=bin8 -output=%APPLICATION_NAME%_OTA_APP.bin %APPLICATION_NAME%.ptp 


echo ***Merge bootloader %BOOTLOADER_NAME%.ptp and application %APPLICATION_NAME%.ptp into one***
%UTILITY_TOOL%   merge   -replace -output=%OUTPUT_NAME%.ptp %BOOTLOADER_NAME%.ptp %APPLICATION_NAME%.ptp


echo ***fill empty areas with ff's***
%UTILITY_TOOL% cut inputType=ptp %OUTPUT_NAME%.ptp 08000000:0803FFFF,08000000 -fillvalue=ff -output=%OUTPUT_NAME%.ptp -replace

echo ***Convert Merged %OUTPUT_NAME%.ptp into %OUTPUT_NAME%.hex***
%UTILITY_TOOL% convert -replace -toType=ihex -output=%OUTPUT_NAME%.hex %OUTPUT_NAME%.ptp 

echo ***Convert Merged %OUTPUT_NAME%.ptp into %OUTPUT_NAME%.bin***
%UTILITY_TOOL% convert -replace -toType=bin8 -output=%OUTPUT_NAME%.bin %OUTPUT_NAME%.ptp


echo ***calculate %OUTPUT_NAME%.ptp checksum***
%UTILITY_TOOL% sum -replace -fillValue=FF -LITTLEENDIAN -output=manufacturing_chksum.txt  %APPLICATION_NAME%.ptp  08000000:0803FFFF

echo ***copy to out folder***

MD update
copy %APPLICATION_NAME%_OTA_APP.bin update
copy %OUTPUT_NAME%.hex update
copy %OUTPUT_NAME%.bin update

del *.ptp
del %APPLICATION_NAME%.hex
del %OUTPUT_NAME%.*
del %BOOTLOADER_NAME%.*
del %BOOT_UTILITY_ROM_NAME%.*
del %APPLICATION_NAME%_OTA_APP.bin


echo.
echo Success!
echo.

pause