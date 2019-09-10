## Husky High Level Controller

cmake_minimum_required(VERSION 2.8.3)
project(husky_highlevel_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs visualization_msgs robot_localization
)

###########
## Build ##
###########

## Include dirs
include_directories(include ${catkin_INCLUDE_DIRS})

## Executables
add_executable(${PROJECT_NAME} src/husky_highlevel_controller_node.cpp src/HuskyHighLevelController.cpp)
add_executable(husky_proximity_detection src/husky_proximity_detection_node.cpp src/HuskyProximitySensor.cpp)

## Linking
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(husky_proximity_detection ${catkin_LIBRARIES})
