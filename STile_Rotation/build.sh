#!/bin/sh

BIN_FILE=Projects/SensorTile/Applications/BLE_SampleApp/SW4STM32/STM32L4xx-SensorTile/Release/MotEnv1_ST.bin
if test -f "$BIN_FILE"; then
  st-flash write $BIN_FILE 0x8004000
  st-flash write Utilities/BootLoader/STM32L476RG/BootLoaderL4.bin 0x8000000
else
  echo "Please build the project first!!"
fi

