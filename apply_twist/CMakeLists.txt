cmake_minimum_required(VERSION 2.8.3)

project(apply_twist)

find_package(catkin REQUIRED COMPONENTS rtt_ros std_msgs geometry_msgs eigen_conversions)

find_package(OROCOS-RTT REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

orocos_component(apply_twist src/ApplyTwist.cpp)
target_link_libraries(apply_twist ${catkin_LIBRARIES})

orocos_generate_package()
