# LocalDynamicMap

Source: [LocalDynamicMap.cpp](../../src/multirobotexploration/source/map/LocalDynamicMap.cpp)

## Parameters

* ```id```

* ```rate```

* ```queue_size```

* ```max_lidar_range```

* ```free_inflation_radius```

* ```ocu_inflation_radius```

* ```lidar_sources```

* ```local_view_size```

## Subscribed Topics

* ```<namespace>/map``` ([nav_msgs::OccupancyGrid](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

* ```<namespace>/gmapping_pose/world_pose``` ([multirobotsimulations::CustomPose](../../src/multirobotsimulations/msg/CustomPose.msg))

* ```<namespace>/laser_to_world/lidar_occ_<id>``` ([geometry_msgs::PoseArray](ttps://docs.ros.org/en/api/geometry_msgs/html/msg/PoseArray.html))

## Published Topics

* ```<namespace>/obstacle_cells``` ([costmap_converter::ObstacleArrayMsg](ttps://docs.ros.org/en/api/costmap_converter/html/msg/ObstacleArrayMsg.html))

* ```<namespace>/c_space_local``` ([nav_msgs::OccupancyGrid](ttps://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

* ```<namespace>/local_occupied_poses``` ([geometry_msgs::PoseArray](ttps://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

* ```<namespace>/local_free_poses``` ([geometry_msgs::PoseArray](ttps://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

<!-- ## Published Transforms

* ```odom``` -->
