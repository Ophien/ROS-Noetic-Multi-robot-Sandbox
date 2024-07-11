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

* ```/c_space``` ([nav_msgs::OccupancyGrid](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

The configuration space with inflated obstacles for navigation. It must handle other robots, dynamic and static obstacles.

## Published Topics

* ```/sub_goal_nav/goal``` ([geometry_msgs::Pose](https://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html))

The goal location to explore to the sub goal navigation module.

<!-- ## Published Transforms

* ```odom``` -->
