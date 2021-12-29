cmake_minimum_required(VERSION 2.8.3)
project(ld06_lidar)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ld06_lidar
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
${catkin_INCLUDE_DIRS}
)

## Get current directory files
aux_source_directory(. file_name)

if(file_name MATCHES "slbf.cpp")
  add_definitions(-D"USE_SLBF")
  message("-- #define USE_SLBF")
endif()

if(file_name MATCHES "slbi.cpp")
  add_definitions(-D"USE_SLBI")
  message("-- #define USE_SLBI")
endif()

SET(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

file(GLOB  MAIN_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)

add_executable(ld06_lidar  ${MAIN_SRC})
target_link_libraries(ld06_lidar pthread udev  ${catkin_LIBRARIES})


## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} ld06_lidar
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/ld06.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
