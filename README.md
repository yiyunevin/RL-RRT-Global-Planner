# Globla Planner Based on Bidirectional-RRT*

+ A global planner based on Bidirectional-RRT*
+ Use intermidiate points as the global information
+ Use ROS Occupancy Map

## System

+ Ubuntu 20.04
+ ROS Noetic (Read Map)

## Usage

### Environment setting

1. Build a ROS package named `rrt_planner`
2. Replace with the files of this repository
3. `$ catkin_make`

### Run the test program

```bash
$ roslaunch rrt_planner global_planner.launch test:=true
$ python3 ~/rrt_rl_review_ws/src/rrt_planner/script/rrt_planner_test.py --planner
```

### Files

+ `node.h`: structure of each node
+ `grid_map.h` & `grid_map.cpp`: map processing, including collision-checking
+ `rrt_planner.h` & `rrt_planner.cpp`: main part of the algorithm
+ `rrt_planner_client.py`: API to use the function of the algorithm through ROS Service (path-planning & map info.) 
+ `rrt_planner_test.py`: the main file (test file)
+ `base_global_planner_params.yaml`: parameters setting

## Result

red: start ; green: destination; cyan: inter. points.

<p align=center>
  <img width="50%" src=https://github.com/yiyunevin/Globla-Planner/blob/main/maps/stage_1/global_planning_result.png?raw=true>
</p>
