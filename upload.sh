#!/bin/bash

make -C Libmaple/libmaple library
make -C BuildAQ32

echo 1EAF > /dev/ttyACM0

sudo dfu-util -d 0483:df11 -a 0 -s 0x8010000 -D BuildAQ32/objSTM32/AeroQuad32/AeroQuadMain.bin


