PROJECT(autopilot)
SET(PACKAGE_NAME autopilot)

SET(SRCS autopilot.c)

INCLUDE_DIRECTORIES(../../imu ../../servo)

ADD_EXECUTABLE(${PACKAGE_NAME} ${SRCS})

TARGET_LINK_LIBRARIES(${PACKAGE_NAME}
  rpi_imu rpi_servo
  )
