cmake_minimum_required(VERSION 3.0.2)
project(rc_backhoe_description)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    gazebo_dev
    gazebo_plugins
    roscpp
    std_msgs
    sensor_msgs
    control_toolbox
    controller_manager
    hardware_interface
    transmission_interface
    pluginlib
    joint_limits_interface
    urdf
    angles
)
find_package(gazebo REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES rc_backhoe_description
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
${catkin_INCLUDE_DIRS}
)

add_library(rc_backhoe_hw_sim src/rc_backhoe_hw_sim.cpp)
target_link_libraries(rc_backhoe_hw_sim ${catkin_LIBRARIES})

add_library(rc_backhoe_retrofit_model src/rc_backhoe_retrofit_model.cpp)
target_link_libraries(rc_backhoe_retrofit_model ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/gazebo.test)
endif()
