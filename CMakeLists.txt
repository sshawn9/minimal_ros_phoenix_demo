cmake_minimum_required(VERSION 3.0)
project(minimal_ros_phoenix_demo)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        std_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(ros_phoenix_pub_demo
        ros_phoenix_pub_demo.cpp
)

target_link_libraries(ros_phoenix_pub_demo
        ${catkin_LIBRARIES}
)

add_executable(ros_phoenix_sub_demo
        ros_phoenix_sub_demo.cpp
)

target_link_libraries(ros_phoenix_sub_demo
        ${catkin_LIBRARIES}
)
