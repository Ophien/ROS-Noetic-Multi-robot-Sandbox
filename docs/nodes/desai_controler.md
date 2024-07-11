# DesaiController

Source: [DesaiController.cpp](../../src/multirobotexploration/source/navigation/DesaiController.cpp)

## Parameters

* ```/robots```

* ```id```

* ```queue_size```

* ```clearing_time```

* ```rate```

* ```max_linear_vel```

* ```max_angular_vel```

* ```D```

## Subscribed Topics

* ```<namespace>/gmapping_pose/world_pose``` ([multirobotsimulations::CustomPose](../../src/multirobotsimulations/msg/CustomPose.msg))

* ```<namespace>/pfield_local_planner/potential_force``` ([geometry_msgs::Pose](https://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html))

* ```<namespace>/desai_controller/set_clear_state``` ([std_msgs::String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))

* ```<namespace>/desai_controller/set_goal_state``` ([std_msgs::String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))

* ```<namespace>/desai_controller/set_idle_state``` ([std_msgs::String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))

## Published Topics

* ```<namespace>/cmd_vel``` ([geometry_msgs::Twist](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html))

* ```<namespace>/desai_controller/clear_complete``` ([std_msgs::String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))

* ```<namespace>/desai_controller/reached_goal``` ([std_msgs::String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))

<!-- ## Published Transforms

* ```odom``` -->
