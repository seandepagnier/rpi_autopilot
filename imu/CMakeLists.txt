PROJECT(imu)
SET(PACKAGE_NAME rpi_imu)

INCLUDE_DIRECTORIES(../misc)

SET(SRC
  calibration.c
  imu.c
  matrix.c
  quaternion.c
  rotate.c
  vector.c
  MadgwickAHRS/MadgwickAHRS.c
  MahonyAHRS/MahonyAHRS.c
)

ADD_LIBRARY(${PACKAGE_NAME} SHARED ${SRC})

TARGET_LINK_LIBRARIES(${PACKAGE_NAME}
  m pthread
)
