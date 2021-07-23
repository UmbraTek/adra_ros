
cmake_minimum_required(VERSION 2.8.3)
project(adra_ros)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
add_message_files(FILES AdraMsg.msg)
add_service_files(FILES Api.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)


include_directories(include ${catkin_INCLUDE_DIRS})

include_directories("utapi")
add_subdirectory(utapi utapi)



add_executable(adra_listener src/adra_listener.cpp)
target_link_libraries(adra_listener ${catkin_LIBRARIES})
add_dependencies(adra_listener adra_ros_gencpp)


add_executable(adra_server src/adra_server.cpp)
target_link_libraries(adra_server ${catkin_LIBRARIES} utapi pthread rt m)
add_dependencies(adra_server adra_ros_gencpp)

add_executable(adra_client src/adra_client.cpp)
target_link_libraries(adra_client ${catkin_LIBRARIES})
add_dependencies(adra_client adra_ros_gencpp)

