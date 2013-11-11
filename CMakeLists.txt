cmake_minimum_required(VERSION 2.8)

PROJECT(rpi_ap)

ADD_DEFINITIONS(" -g -std=gnu99")

add_subdirectory(examples)
add_subdirectory(imu)
add_subdirectory(servo)
add_subdirectory(misc)
add_subdirectory(glscope)
