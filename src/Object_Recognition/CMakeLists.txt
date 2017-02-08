

cmake_minimum_required(VERSION 2.8.9)
set(PROJECTNAME Object_Recognition)
project(${PROJECTNAME})

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_DEBUG} -Wall -g")

find_package(YARP)
find_package(ICUB)
find_package(OpenCV)
find_package(Caffe)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})

find_package(ICUBcontrib)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

include_directories(${Caffe_INCLUDE_DIRS})

set(folder_source main.cpp)

add_definitions(-D_USE_MATH_DEFINES -DCPU_ONLY)
include_directories(${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${folder_source})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} ${Caffe_LIBRARIES} ctrlLib)


add_executable(test test.cpp)
target_link_libraries(test ${YARP_LIBRARIES} ${OpenCV_LIBRARIES} ${Caffe_LIBRARIES} ctrlLib)

install(TARGETS ${PROJECTNAME} DESTINATION bin)

