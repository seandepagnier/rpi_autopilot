PROJECT(misc)
SET(PACKAGE_NAME rpi_misc)

SET(SRC
  config.c
  print_value.c
)

ADD_LIBRARY(${PACKAGE_NAME} SHARED ${SRC})
