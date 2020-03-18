#!/bin/bash
ELF=$1
arm-none-eabi-objcopy -O binary $ELF $ELF.bin
st-flash --reset write $ELF.bin 0x08000000