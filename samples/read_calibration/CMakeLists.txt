cmake_minimum_required(VERSION 3.5)

project(read_calibration)

find_package( xvsdk REQUIRED )

set(SRCS read_calibration.cpp)

ADD_EXECUTABLE( ${PROJECT_NAME} ${SRCS} )
TARGET_LINK_LIBRARIES( ${PROJECT_NAME} ${xvsdk_LIBRARIES} -pthread )

