# SAS Space Explore Planner
## Overview
This package is an implementation of the SAS space exploration method explained by `speed up search by defining effective zone and scalable motion primitives for Nonholonomic Vehicles`. This package includes two main parts, the SAS space exploration algorithm library and the ros node examples([sas_explore_node](https://github.com/bit-ivrc/hmpl/blob/feature_13_sas-space-explore/sas_space_explore/src/sas_explore_node.cpp)).


## Ros Interface of the Global Path Node

### Subscribers:
* `/initialpose` ([geometry_msgs::PoseWithCovarianceStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))
* `/move_base_simple/goal`([geometry_msgs::PoseStamped]())  


### Publisher:
* `/result_path`([nav_msgs::Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))
* `/grid_map`([nav_msgs::OccupancyGrid]())

### Preloaded Data
* `obstacle.png`  
This example node reads the image data as an environment grid map, then plans a path and publishes th grid map.
