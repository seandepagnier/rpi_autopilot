PROJECT(imu)
SET(PACKAGE_NAME rpi_imu)

SET(SRC
  calibration.c
  config.c
  imu.c
  matrix.c
  quaternion.c
  rotate.c
  vector.c
  config.c
  MadgwickAHRS/MadgwickAHRS.c
  MahonyAHRS/MahonyAHRS.c
)

ADD_LIBRARY(${PACKAGE_NAME} SHARED ${SRC})

TARGET_LINK_LIBRARIES(${PACKAGE_NAME}
  m pthread
)
