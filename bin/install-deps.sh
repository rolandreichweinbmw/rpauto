#!/bin/bash

set -e

wget -O openocd.tar.gz https://github.com/raspberrypi/pico-sdk-tools/releases/download/v2.2.0-3/openocd-0.12.0+dev-x86_64-lin.tar.gz
rm -rf openocd
mkdir openocd
cd openocd
tar xf ../openocd.tar.gz
cd ..
rm openocd.tar.gz

sudo apt install cmake python3 build-essential gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib can-utils

echo "Done."
