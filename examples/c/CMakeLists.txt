PROJECT(autopilot)
SET(PACKAGE_NAME autopilot)

INCLUDE_DIRECTORIES(../../imu ../../servo ../../misc)

SET(LIBS rpi_imu rpi_servo rpi_misc)

SET(AUTOPILOT_SRCS autopilot.c)
ADD_EXECUTABLE(autopilot ${AUTOPILOT_SRCS})
TARGET_LINK_LIBRARIES(autopilot ${LIBS})

SET(IMUTEST_SRCS imutest.c)
ADD_EXECUTABLE(imutest ${IMUTEST_SRCS})
TARGET_LINK_LIBRARIES(imutest ${LIBS})

