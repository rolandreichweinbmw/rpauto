#!/bin/bash

set -e

FILE=./build-can/src/rpauto.elf

if [[ "$FILE" = "" ]] ; then
	LIST=`find . -name '*.elf'`
	if [[ "`echo $LIST | wc -w`" = "1" ]] ; then
		FILE=$LIST
	fi
fi

if [[ "$FILE" = "" ]] ; then
	echo "usage: $0 [ELF-File]"
	exit 1
fi

echo "Using $FILE for flashing."

openocd/openocd -s openocd/scripts -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000" -c "program $FILE verify reset exit"
