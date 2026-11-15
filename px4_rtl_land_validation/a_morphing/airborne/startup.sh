#!/bin/sh
. ${R}etc/init.d-posix/rcS
mavlink start -x -u 14651 -o 14650 -t 127.0.0.1 -m onboard -r 1000000
