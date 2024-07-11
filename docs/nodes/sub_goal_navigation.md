# SubGoalNav

This node implements a simple sub goal following controller.

Source: [SubGoalNav.cpp](../../src/multirobotexploration/source/navigation/SubGoalNav.cpp)

## Parameters

* ```id```

Id of this robot.

* ```rate```

Main loop rate in hertz.

* ```queue_size```

Queue size of publishers and subscribers.

* ```reach_threshold```

Distance in meters used to verify is the robot reached the desired location.

* ```delta_threshold```

Time threshold in seconds that the robot must wait before engaging in recover behavior. If the average robot's velocity is bellow a threshold (e.g., the robot is not moving), then the controller will start counting. If the time that it is stuck is greater than ```delta_threshold```, then the robot will engage in recovery behavior.

## Subscribed Topics

* ```<namespace>/desai_controller/clear_complete``` ([std_msgs::String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))

* ```<namespace>/c_space``` ([nav_msgs::OccupancyGrid](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

* ```<namespace>/gmapping_pose/world_pose``` ([multirobotsimulations::CustomPose](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

* ```<namespace>/avgd_average_displacement``` ([std_msgs::Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))

* ```<namespace>/sub_goal_nav/goal``` ([geometry_msgs::Pose](https://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html))

* ```<namespace>/sub_goal_nav/clear``` ([std_msgs::String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))

* ```<namespace>/sub_goal_nav/stop``` ([std_msgs::String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))

## Published Topics

* ```<namespace>/pfield_local_planner/goal_occ``` ([geometry_msgs::Pose](https://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html))

* ```<namespace>/desai_controller/enable``` ([std_msgs::Bool](https://docs.ros.org/en/api/std_msgs/html/msg/Bool.html))

* ```<namespace>/desai_controller/set_clear_state``` ([std_msgs::String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))

* ```<namespace>/desai_controller/set_goal_state``` ([std_msgs::String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))

* ```<namespace>/desai_controller/set_idle_state``` ([std_msgs::String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))

* ```<namespace>/sub_goal_nav/path``` ([visualization_msgs::Marker](https://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html))

* ```<namespace>/sub_goal_nav/finish``` ([std_msgs::String](https://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html))

* ```<namespace>/sub_goal_nav/clear_finish``` ([std_msgs::String](https://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html))

* ```<namespace>/sub_goal_nav/current_path``` ([nav_msgs::Path](https://docs.ros.org/en/api/nav_msgs/html/msg/Path.html))

<!-- ## Published Transforms

* ```odom``` -->
