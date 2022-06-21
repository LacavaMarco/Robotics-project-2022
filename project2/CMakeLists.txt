cmake_minimum_required(VERSION 3.0.2)
project(project2)

find_package(catkin REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
    tf2
    tf2_ros
    message_generation
)

add_service_files(
    FILES
    SaveTrajectory.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    message_runtime
)

include_directories(
    include ${catkin_INCLUDE_DIRS}
)

add_executable(odom_tf
    src/odom_tf.cpp
)
add_dependencies(odom_tf ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(odom_tf ${catkin_LIBRARIES})
