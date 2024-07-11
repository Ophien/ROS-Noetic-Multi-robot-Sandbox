# PotentialFieldLocalPlanner

Source: [PotentialFieldLocalPlanner.cpp](../../src/multirobotexploration/source/navigation/PotentialFieldLocalPlanner.cpp)

## Parameters

* ```queue_size```

* ```local_view_size```

* ```k_rep```

* ```k_att```

* ```r```

* ```rate```

## Subscribed Topics

* ```<namespace>/map``` ([nav_msgs::OccupancyGrid](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))


* ```<namespace>/c_space_local``` ([nav_msgs::OccupancyGrid](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

* ```<namespace>/gmapping_pose/world_pose``` ([multirobotsimulations::CustomPose](../../src/multirobotsimulations/msg/CustomPose.msg))

* ```<namespace>/pfield_local_planner/goal_occ``` ([geometry_msgs::Pose](https://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html))
  
## Published Topics

* ```<namespace>/pfield_local_planner/potential_force``` ([geometry_msgs::Pose](https://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html))

<!-- ## Published Transforms

* ```odom``` -->
