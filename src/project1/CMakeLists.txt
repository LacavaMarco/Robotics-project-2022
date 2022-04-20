cmake_minimum_required(VERSION 2.8.3)
project(project1)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs nav_msgs tf tf2 tf2_ros)

## Declare a catkin package
#  catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES subscriber_class
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
#)

catkin_package( CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
)

## Build talker and listener

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(velocity src/velocity.cpp)
add_dependencies(velocity ${catkin_EXPORTED_TARGETS} project1_generate_messages)
target_link_libraries(velocity ${catkin_LIBRARIES})

add_executable(odometry src/odometry.cpp)
add_dependencies(odometry ${catkin_EXPORTED_TARGETS} project1_generate_messages)
target_link_libraries(odometry ${catkin_LIBRARIES})

add_executable(tf2broadcaster src/tf2broadcaster.cpp)
add_dependencies(tf2broadcaster ${catkin_EXPORTED_TARGETS} project1_generate_messages)
target_link_libraries(tf2broadcaster ${catkin_LIBRARIES})
