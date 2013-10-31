PROJECT(glscope)
SET(PACKAGE_NAME glscope)

SET(SRCS glscope.c)
ADD_EXECUTABLE(glscope ${SRCS})
TARGET_LINK_LIBRARIES(glscope glut fftw3 GL GLU m)
