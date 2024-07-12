# MRELocalPlanner

This node implements Yamauchi's method from 1999. It relies in a configuration space, frontier discovery services, and a sub goal navigation module. Some of its functionality should be coded as services.

Source: [MRELocalPlanner.cpp](../../src/multirobotexploration/source/navigation/MRELocalPlanner.cpp)

## Parameters

* ```/robots```

* ```id```

* ```min_dist```

* ```min_displacement```

* ```waypoints_to_use```

* ```via_points_increment```

## Subscribed Topics

* ```<namespace>/obstacle_array``` ([costmap_converter::ObstacleArrayMsg])

* ```<namespace>/gmapping_pose/world_pose``` ([multirobotsimulations::CustomPose](../../src/multirobotsimulations/msg/CustomPose.msg))

* ```<namespace>/sub_goal_nav/current_path``` ([nav_msgs::Path](https://docs.ros.org/en/api/nav_msgs/html/msg/Path.html))

* ```<namespace>/mock_communication_model/robots_in_comm``` ([std_msgs::Int8MultiArray](https://docs.ros.org/en/api/std_msgs/html/msg/Int8MultiArray.html))

* ```<namespace>/robot_<id>/avgd_average_displacement``` ([std_msgs::Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))

* ```<namespace>/relative_pose_estimator/distances``` ([std_msgs::Float64MultiArray](https://docs.ros.org/en/api/std_msgs/html/msg/Float64MultiArray.html))
  
## Published Topics

* ```<namespace>/node_rosaria/cmd_vel``` ([geometry_msgs::Twist](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html))

* ```<namespace>/mre_local_planner/global_via_points``` ([visualization_msgs::Marker](https://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html))

<!-- ## Published Transforms

* ```odom``` -->
