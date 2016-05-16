#!/bin/sh

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PORT=$(basename `ls /dev/cu.usbmodem*`)

stty -f /dev/$PORT 1200; stty stop /dev/$PORT; $DIR/bossac -i --port=$PORT -U false -e -w -v -b $DIR/MdkController.bin -R