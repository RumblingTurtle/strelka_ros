# ROS support for strelka library
A package to provide plug-in for [Livox Series LiDAR](https://www.livoxtech.com).

## Requirements
- ROS(=Melodic,Noetic)
- Gazebo (= 9.x+, http://gazebosim.org/)
- Ubuntu(=18.04,20.04)
- livox_laser_simulation (custom fork https://github.com/RumblingTurtle/livox_laser_simulation)
- unitree_ros https://github.com/unitreerobotics/unitree_ros
- strelka library (https://github.com/RumblingTurtle/strelka)
- grid_map
## Usage

### Start simulation
```
# With livox
roslaunch strelka_ros a1_livox.launch 

# Without livox
roslaunch strelka_ros a1_normal.launch

# Livox can have custom pitch value
roslaunch strelka_ros a1_livox.launch livox_pitch:=0.9

# Custom worlds can also be used
roslaunch strelka_ros a1_normal/a1_livox.launch wname:=[empty,stairs,bumpy_terrain ...]
```
### Run low level control and state estimation separately

```
# Start WBIC and state estimation node together
roslaunch strelka_ros a1_wbic_observer.launch

# Or separately
roslaunch strelka_ros a1_state_estimator.launch
roslaunch strelka_ros a1_wbic.launch
```
### Start local planner node
```
# Blind local planner node 
roslaunch strelka_ros a1_local_planner.launch

# Elevation aware planner
roslaunch strelka_ros a1_elevation_aware_local_planner.launch
roslaunch strelka_elevation a1.launch map_type:=steps
```
### Run full controller pipeline
```
roslaunch strelka_ros a1_full_pipeline.launch blind:=false/true
```
## Livox mid-70 mount
![](resources/livox.gif)
