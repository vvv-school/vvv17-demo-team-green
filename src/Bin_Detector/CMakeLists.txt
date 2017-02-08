cmake_minimum_required(VERSION 2.8.9)
project(Bin_Detector)

find_package(YARP)
find_package(ICUB)
find_package(OpenCV)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})

find_package(ICUBcontrib)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix()

set(folder_source /home/icub/green-cub/vvv17-demo-team-green/src/Bin_Detector/src/main.cpp /home/icub/green-cub/vvv17-demo-team-green/src/Bin_Detector/src/BinDetector.cpp )
source_group("Source Files" FILES ${folder_source})

include_directories(include ${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} ${folder_source})
target_link_libraries(${PROJECT_NAME} ${YARP_LIBRARIES} ctrlLib ${OpenCV_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)
