cmake_minimum_required(VERSION 3.5.1)
project(strelka_elevation)

# Better with serial algorithms.
set(CMAKE_CXX_STANDARD 11)
# We want performance (fast-math) but also need a representation for NaN values to represent missing values. 
# Therefore, we disable the finite-math-only flag that was set by fast-math.  
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ffast-math -fno-finite-math-only")
add_compile_options(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

## Create a list of catkin package dependencies used in header files of this package.
set(CATKIN_PACKAGE_HEADER_DEPENDENCIES
  grid_map_core
  grid_map_ros
  grid_map_msgs
  filters
)

## Create a list of catkin package dependencies, now for both header and source files.
set(CATKIN_PACKAGE_DEPENDENCIES
  ${CATKIN_PACKAGE_HEADER_DEPENDENCIES}
)

add_definitions(-DTHREAD_MAX_MUM=5) 
## Find catkin dependencies for building this package.
find_package(catkin REQUIRED
  COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
  elevation_mapping
  grid_map_rviz_plugin
  rviz
)

find_package(TBB 2020.1 EXACT REQUIRED)

find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
    ${OpenCV_INCLUDE_DIRS} # TODO Remove this include directory, currently it exported by the medianFillFilter header.
  LIBRARIES
    ${PROJECT_NAME}
    ${PROJECT_NAME}_plugins
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_HEADER_DEPENDENCIES}
    elevation_mapping
    grid_map_rviz_plugin
    rviz
  DEPENDS
    OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/StepFilter.cpp
  src/SlidingWindowExpressionFilter.cpp
  src/ConstInpaint.cpp
  src/SDF2D.cpp
  src/MathExpressionFilter.cpp
)

target_include_directories(${PROJECT_NAME}
  PRIVATE
    ${TBB_INCLUDE_DIRS}
)

target_include_directories(${PROJECT_NAME}
  SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${TBB_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

# Instantiate plugin exports for every filter in this package.
add_library(${PROJECT_NAME}_plugins
  src/plugins.cpp
)

target_include_directories(${PROJECT_NAME}_plugins
  SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_plugins
  ${PROJECT_NAME}
  ${TBB_LIBRARIES} # is this necessary?
  ${OpenCV_LIBRARIES} # is this necessary?
)

add_dependencies(${PROJECT_NAME}_plugins
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_plugins
  ARCHIVE DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
  FILES
    filter_plugins.xml
  DESTINATION
    ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
