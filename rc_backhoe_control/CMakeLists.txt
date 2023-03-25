cmake_minimum_required(VERSION 3.0.2)
project(rc_backhoe_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    hardware_interface
    transmission_interface
    controller_manager
    control_toolbox
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES rc_backhoe_control
  CATKIN_DEPENDS roscpp hardware_interface transmission_interface controller_manager
#  DEPENDS system_lib
)


## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rc_backhoe_control src/rc_backhoe_control.cpp src/rc_backhoe_hw.cpp)
add_dependencies(rc_backhoe_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rc_backhoe_control
  ${catkin_LIBRARIES}
)

install(TARGETS rc_backhoe_control
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

###########