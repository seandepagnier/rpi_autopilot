#!/bin/sh

# syncronize date and time
echo -e "sudo date -us `date -u +%D` \n sudo date -us `date -u +%T`" | ssh raspberrypi

#scp -pr build CMakeLists.txt README examples imu schematic servo raspberrypi:autopilot
