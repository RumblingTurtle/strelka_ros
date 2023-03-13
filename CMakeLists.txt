cmake_minimum_required(VERSION 3.0.2)
project(strelka_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  elevation_mapping
  tf
)

SET(CMAKE_MODULE_PATH ${${PROJECT_NAME}_SOURCE_DIR}/cmake)

find_package(lcm REQUIRED)
if (LCM_FOUND)
    include_directories(${LCM_INCLUDE_DIRS})
endif (LCM_FOUND)

find_package(catkin REQUIRED COMPONENTS
    controller_manager
    genmsg
    joint_state_controller
    robot_state_publisher
    roscpp
    gazebo_ros
    std_msgs
    tf
    geometry_msgs
    unitree_legged_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES strelka_ros
  CATKIN_DEPENDS elevation_mapping 
  unitree_legged_msgs 
)

include_directories(
  ./include
  ${catkin_INCLUDE_DIRS}
)

add_executable(a1_transform_publisher src/A1/a1_transform_publisher.cpp)

target_link_libraries( a1_transform_publisher
${catkin_LIBRARIES}
lcm
)

add_executable(a1_command_publisher src/A1/a1_command_publisher.cpp)

target_link_libraries( a1_command_publisher
${catkin_LIBRARIES}
lcm
)

add_executable(a1_local_planner src/A1/a1_local_planner.cpp)

target_link_libraries(a1_local_planner
${catkin_LIBRARIES}
strelka_robots
lcm
)

add_executable(a1_state_estimator src/A1/a1_state_estimator.cpp)

target_link_libraries(a1_state_estimator
${catkin_LIBRARIES}
strelka_robots
lcm
)

add_executable(a1_wbic src/A1/a1_wbic.cpp)

target_link_libraries(a1_wbic
${catkin_LIBRARIES}
strelka_robots
lcm
)