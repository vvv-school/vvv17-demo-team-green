

cmake_minimum_required(VERSION 2.8.9)
set(PROJECTNAME Gaze_Control)
project(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})

find_package(ICUBcontrib)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

set(folder_source ${CMAKE_CURRENT_SOURCE_DIR}/gaze_controller.cpp)
source_group("Source Files" FILES ${folder_source})

add_definitions(-D_USE_MATH_DEFINES)
include_directories(${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
add_executable(${PROJECTNAME} ${folder_source})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ctrlLib)
install(TARGETS ${PROJECTNAME} DESTINATION bin)

