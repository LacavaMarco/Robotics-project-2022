cmake_minimum_required(VERSION 2.8.3)
project(project1)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    tf2
    tf2_ros
    dynamic_reconfigure
    message_generation
)

## Declare a catkin package
#  catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES subscriber_class
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
#)
add_message_files(
    FILES
    WheelsVector4.msg
)

add_service_files(
    FILES
    Reset.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs ## both Header and float64 of WheelsVector4 message are defined in std_msgs
    geometry_msgs
)

generate_dynamic_reconfigure_options(
    cfg/velParameters.cfg
    cfg/odomParameters.cfg
)

catkin_package(CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    message_runtime
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
# add_dependencies(tf2broadcaster ${catkin_EXPORTED_TARGETS} project1_generate_messages)
target_link_libraries(tf2broadcaster ${catkin_LIBRARIES})

add_executable(tf2bag src/tf2bag.cpp)
# add_dependencies(tf2bag ${catkin_EXPORTED_TARGETS} project1_generate_messages)
target_link_libraries(tf2bag ${catkin_LIBRARIES})

add_executable(control src/control.cpp)
add_dependencies(control ${catkin_EXPORTED_TARGETS} project1_generate_messages)
target_link_libraries(control ${catkin_LIBRARIES})

add_executable(reset_client src/client.cpp)
add_dependencies(reset_client ${catkin_EXPORTED_TARGETS} project1_generate_messages)
target_link_libraries(reset_client ${catkin_LIBRARIES})
