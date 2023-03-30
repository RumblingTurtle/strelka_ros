cmake_minimum_required(VERSION 3.0.2)
project(strelka_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

SET(CMAKE_MODULE_PATH ${${PROJECT_NAME}_SOURCE_DIR}/cmake)

find_package(lcm REQUIRED)
if (LCM_FOUND)
    include_directories(${LCM_INCLUDE_DIRS})
endif (LCM_FOUND)

find_package(strelka REQUIRED)

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
    grid_map_core
    grid_map_ros
    grid_map_msgs
    message_generation
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES strelka_ros
  CATKIN_DEPENDS elevation_mapping 
  unitree_legged_msgs 
  grid_map_core
  grid_map_ros
  grid_map_msgs
)

include_directories(
  ./include
  ${catkin_INCLUDE_DIRS}
  ${strelka_INCLUDE_DIRS}
)

add_executable(a1_transform_publisher src/TransformPublisher.cpp)

target_link_libraries( a1_transform_publisher
${catkin_LIBRARIES}
${strelka_LIBRARIES}
lcm
)

add_executable(a1_bridge src/A1/A1Bridge.cpp)

target_link_libraries( a1_bridge
${catkin_LIBRARIES}
${strelka_LIBRARIES}
lcm
)

add_executable(a1_local_planner src/A1/LocalPlanner.cpp src/ElevationAwareFootPlanner.cpp)

target_link_libraries(a1_local_planner
${catkin_LIBRARIES}
${strelka_LIBRARIES}
lcm
)

add_executable(a1_state_estimator src/A1/StateEstimator.cpp)

target_link_libraries(a1_state_estimator
${catkin_LIBRARIES}
${strelka_LIBRARIES}
lcm
)

add_executable(a1_wbic src/A1/WBIC.cpp)

target_link_libraries(a1_wbic
${catkin_LIBRARIES}
${strelka_LIBRARIES}
lcm
)

add_executable(a1_high_command_publisher src/A1/HighLevelCommandPublisher.cpp)

target_link_libraries(a1_high_command_publisher 
${catkin_LIBRARIES}
${strelka_LIBRARIES}
lcm
)

add_executable(a1_cheater_state_estimator src/A1/A1CheaterEstimator.cpp)

target_link_libraries(a1_cheater_state_estimator 
${catkin_LIBRARIES}
${strelka_LIBRARIES}
lcm
)

