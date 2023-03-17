<p align="center">
  <img src="resources/elevation.gif" alt="animated" />
</p>

# Overview
Custom set of elevation_mapping filters and configs for [strelka_ros](https://github.com/RumblingTurtle/strelka_ros/tree/master/strelka_ros).

- SDF2D - 2D signed distance field filter
```
  - name: sdf
    type: elevationMappingFilters/SDF2D
    params:
      input_layer: normal_vectors_z
      output_layer: sdf
      threshold: 0.6 # Threshold after which the cell is considered to be an obstacle
      safe_zone: 0.1 # Clamps the SDF value to 0 if the cell value is less thansafe_zone 
      radius: 0.10 # Search radius
```
- ConstInpaint - inpaint filter with custom NAN value subsitution
```
  - name: inpaint
    type: elevationMappingFilters/ConstInpaint
    params:
      input_layer: elevation
      output_layer: elevation_inpainted
      fill_value: -5 # Subsitutes NAN values
      radius: 0.02
```
- StepFilter - clamps the cell elevation to the closest threshold value in the array
```
  - name: step
    type: elevationMappingFilters/StepFilter
    params:
      input_layer: inpaint
      output_layer: elevation_inpainted
      step_heights: [0.0,0.12,0.24,0.36,0.48,0.6]
``` 

- SlidingWindowExpressionFilter and MathExpressionFilter are fixes to the currently unresolved issues in gridmap repo

# Requirements

- [strelka_ros](https://github.com/RumblingTurtle/strelka_ros)
- [elevation_mapping](https://github.com/ANYbotics/elevation_mapping)

# Installation note
Make sure to build catkin workspace in release mode. This will greatly improve elevation mapping processing times
```
catkin build -DCMAKE_BUILD_TYPE=Release
```

# Usage 
```
roslaunch strelka_elevation a1.launch map_type:=[stairs,steps,default]
```
