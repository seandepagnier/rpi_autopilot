PROJECT(servo)
SET(PACKAGE_NAME rpi_servo)

INCLUDE_DIRECTORIES(../misc)

SET(SRC
  servo.c
)

ADD_LIBRARY(${PACKAGE_NAME} SHARED ${SRC})
