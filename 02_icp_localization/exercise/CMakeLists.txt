cmake_minimum_required(VERSION 3.16)

project(icp_localization)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    nav_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    tf2_ros)

catkin_package(
    INCLUDE_DIRS
    "include"
    CATKIN_DEPENDS
    roscpp
    nav_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    tf2_ros
)

add_library(map SHARED src/map.cpp)
target_include_directories(map PUBLIC ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include)
target_link_libraries(map PUBLIC ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} Eigen3::Eigen)

add_library(icp SHARED src/icp/eigen_icp_2d.cpp)
target_include_directories(icp PUBLIC include/icp)
target_link_libraries(icp PUBLIC Eigen3::Eigen)

# Create rules for the localizer2d SHARED library
# source files are:
# - src/localizer2d.cpp
# - src/ros_bridge.cpp
# It depends on ROS packages (catkin), Eigen and icp

add_library(localizer2d SHARED src/localizer2d.cpp src/ros_bridge.cpp)
target_include_directories(localizer2d PUBLIC ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} include)
target_link_libraries(localizer2d PUBLIC ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} Eigen3::Eigen icp)

# Create rules for the localizer_node (located in bin/localizer_node.cpp)
# It depends on ROS packages (catkin), Eigen, map and localizer2d

add_executable(localizer_node bin/localizer_node.cpp)
target_include_directories(localizer_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(localizer_node PRIVATE ${catkin_LIBRARIES} Eigen3::Eigen map localizer2d)

