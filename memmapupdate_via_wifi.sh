#!/bin/bash
#
#    NLAB-MecanumCommlib for Linux, a simple library to control VStone MecanumRover 2.1
#    by David Vincze, vincze.david@webcode.hu
#    at Human-System Laboratory, Chuo University, Tokyo, Japan, 2021
#    version 0.40
#

if [ ${ROVERIP}x == x ]; then
    ROVERIP=192.168.0.2
fi

while true
do
    >memmap_0x10_tmp.dat
    >memmap_0x1F_tmp.dat
    wget -q -O memmap_0x10_tmp.dat "http://${ROVERIP}/read?i2caddr=10&addr=00&length=00"
    wget -q -O memmap_0x1F_tmp.dat "http://${ROVERIP}/read?i2caddr=1F&addr=00&length=00"
    timestamp=`date +%Y.%m.%d__%H-%M-%S.%N`
    mv memmap_0x10_tmp.dat memmap_0x10_${timestamp}.dat
    mv memmap_0x1F_tmp.dat memmap_0x1F_${timestamp}.dat
    # perform some simple integrity checks, and only use the files if both seem to be intact (length, line count, possibly 0x0D 0x0A at the end)
    if [ `/bin/ls -l memmap_0x10_${timestamp}.dat |cut -f5 -d" "` -eq 514 ] && [ `/bin/ls -l memmap_0x1F_${timestamp}.dat |cut -f5 -d" "` -eq 514 ] &&
       [ `cat memmap_0x10_${timestamp}.dat |wc -L` -eq 512 ] && [ `cat memmap_0x1F_${timestamp}.dat |wc -L` -eq 512 ] &&
       [ `cat memmap_0x10_${timestamp}.dat |wc -l` -eq   1 ] && [ `cat memmap_0x1F_${timestamp}.dat |wc -l` -eq   1 ]; then
        ln -sf memmap_0x10_${timestamp}.dat memmap_0x10.dat
        ln -sf memmap_0x1F_${timestamp}.dat memmap_0x1F.dat
    fi
    sleep 1
done
