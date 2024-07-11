# MockNetworkHandler

Source: [MockNetworkHandler.cpp](../../src/multirobotexploration/source/communication/MockNetworkHandler.cpp)

## Parameters

* ```/robots```

* ```<namespace>/id```

* ```<namespace>/rate```
  
* ```<namespace>/queue_size```

## Subscribed Topics

* ```<namespace>/mock_communication_model/robots_in_comm``` ([std_msgs::Int8MultiArray](https://docs.ros.org/en/api/std_msgs/html/msg/Int8MultiArray.html))

* ```/frontier_discovery/frontiers_clusters``` ([multirobotsimulations::Frontiers](../../src/multirobotsimulations/msg/Frontiers.msg))

* ```/mock_ether``` ([multirobotsimulations::MockPackage](../../src/multirobotsimulations/msg/MockPackage.msg))

* ```<namespace>/fusion``` ([nav_msgs::OccupancyGrid](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

* ```<namespace>/avgd_average_displacement``` ([std_msgs::Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))

* ```<namespace>/gmapping_pose/world_pose``` ([multirobotsimulations::CustomPose](../../src/multirobotsimulations/msg/CustomPose.msg))

## Published Topics

* ```/mock_ether``` ([multirobotsimulations::MockPackage](../../src/multirobotsimulations/msg/MockPackage.msg))

* ```<namespace>/received_comm_packages``` ([multirobotsimulations::MockPackage](../../src/multirobotsimulations/msg/MockPackage.msg))

<!-- ## Published Transforms

* ```odom``` -->
