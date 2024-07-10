# Multi-robot Exploration

It contains the exploration stack from this [research](https://arxiv.org/abs/2309.13494) fully opperational that allow you to start your development out-of-the-box. 

## Nodes

### Policies

- [Yamaychi1999 node](nodes/yamauchi1999.md)
- [RandomizedSocialWelfare node](nodes/randomized_social_welfare.md)
- [Alysson2024 node](nodes/alysson2024.md)

### Navigation

- [DesaiController node](nodes/desai_controler.md)
- [PotentialFieldLocalPlanner node](nodes/potential_field_local_planner.md)
- [MRELocalPlanner node](nodes/mre_local_planner.md)
- [SubGoalNav node](nodes/sub_goal_navigation.md)

### Map

- [CSpace node](nodes/cspace.md)
- [FBMapFusionRelative node](nodes/fb_map_fusion_relative.md)
- [LocalDynamicMap node](nodes/local_dynamic_map.md)

### Localization

- [AverageDisplacement node](nodes/average_displacement.md)
- [GmappingPose node](nodes/gmapping_pose.md)
- [RelativePoseEstimator node](nodes/relative_pose_estimator.md)

### Lidar

- [LaserNoise node](nodes/laser_noiser.md)
- [LaserToWorld node](nodes/laser_to_world.md)

### Frontier

- [FrontierDiscovery node](nodes/frontier_discovery.md)

### Communication

- [MockCommunicationModel node](nodes/mock_communication_model.md)
- [MockNetworkHandler node](nodes/mock_network_handler.md)

## Objects and Algorithms

- [RendezvosPlan](nodes/rendezvous_plan.md)
- [RRTRoot](nodes/rrt_root.md)
- [SearchAlgorithms](nodes/search_algorithms.md)
- [Common](nodes/common.md)