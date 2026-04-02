#!/bin/bash

set -e

#FILE=$1
FILE=./build-pipico/src/cabletester.elf

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

openocd -f interface/cmsis-dap.cfg -f target/rp2040.cfg -c "adapter speed 5000" -c "program $FILE verify reset exit"
