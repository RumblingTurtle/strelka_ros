<p align="center">
  <img src="resources/demo.gif" alt="animated" />
</p>

# ROS support for strelka library
- Gazebo launch files for UnitreeA1 robot
- Livox mid-70 robot mount
- Foothold selection algorithms implemented using [grid_map](https://github.com/ANYbotics/grid_map)
- move_base global planner

## Requirements
- Ubuntu(=18.04,20.04)
- ROS(=Melodic,Noetic)
- [Gazebo](http://gazebosim.org) (= 9.x+)
- [livox_laser_simulation](https://github.com/RumblingTurtle/livox_laser_simulation) (custom fork)
- [unitree_ros](https://github.com/unitreerobotics/unitree_ros)
- [strelka](https://github.com/RumblingTurtle/strelka) library 
- [grid_map](https://github.com/ANYbotics/grid_map)
- [move_base](http://wiki.ros.org/move_base)
```
sudo apt install ros-${ROS_DISTRO}-navigation
```

## Installation
Make sure to build your catkin workspace with optimizations. They greatly improve both elevation_mapping and foothold adaptation speeds.
```
catkin build -DCMAKE_BUILD_TYPE=Release
```
## Usage
### Start simulation
```
roslaunch strelka_ros a1_simulation.launch.launch lidar:=[true|false] lidar_pitch:=0.8 wname:=[empty|stairs|bumpy_terrain ...]
```

### [Edit control parameters using yaml config](config/a1_full_pipeline.yaml)

### Run the pipeline
```
roslaunch strelka_ros a1_full_pipeline.launch
```