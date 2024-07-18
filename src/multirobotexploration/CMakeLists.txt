cmake_minimum_required(VERSION 3.0.2)
project(multirobotexploration)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  roslib
  message_generation
  message_runtime
  dynamic_reconfigure
  teb_local_planner
  multirobotsimulations
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS roscpp
    rospy
    std_msgs
    sensor_msgs
    nav_msgs
    tf2_ros
    roslib
    message_generation
    message_runtime
    dynamic_reconfigure
    teb_local_planner
    multirobotsimulations
)


###########
## Build ##
###########

include_directories(
  include/lib
  include/nodes
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  source/algorithms/SearchAlgorithms.cpp
  source/algorithms/RRTRoot.cpp
  source/algorithms/RendezvousPlan.cpp
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(DENABLE_PRECOMPILED_HEADERS OFF)

###################
## communication ##
###################

add_executable(mockcommunicationmodel source/communication/MockCommunicationModel.cpp)
target_link_libraries(mockcommunicationmodel multirobotexploration ${catkin_LIBRARIES})

add_executable(mocknetworkhandler source/communication/MockNetworkHandler.cpp)
target_link_libraries(mocknetworkhandler multirobotexploration ${catkin_LIBRARIES})

##################
## localization ##
##################
add_executable(gmappingpose source/localization/GmappingPose.cpp)
target_link_libraries(gmappingpose ${catkin_LIBRARIES})

add_executable(averagedisplacement source/localization/AverageDisplacement.cpp)
target_link_libraries(averagedisplacement multirobotexploration ${catkin_LIBRARIES})

add_executable(relativeposeestimator source/localization/RelativePoseEstimator.cpp)
target_link_libraries(relativeposeestimator multirobotexploration ${catkin_LIBRARIES})


################
## navigation ##
################
add_executable(potentialfieldlocalplanner source/navigation/PotentialFieldLocalPlanner.cpp)
target_link_libraries(potentialfieldlocalplanner multirobotexploration ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(desaicontroller source/navigation/DesaiController.cpp)
target_link_libraries(desaicontroller multirobotexploration ${catkin_LIBRARIES})

add_executable(subgoalnav source/navigation/SubGoalNav.cpp)
target_link_libraries(subgoalnav multirobotexploration ${catkin_LIBRARIES})

add_executable(mrelocalplanner source/navigation/MRELocalPlanner.cpp)
target_link_libraries(mrelocalplanner multirobotexploration ${catkin_LIBRARIES})

###############
## frontiers ##
###############
add_executable(frontierdiscoverynode source/frontier/FrontierDiscoveryNode.cpp)
target_link_libraries(frontierdiscoverynode multirobotexploration ${catkin_LIBRARIES})

#############
## mapping ##
#############
add_executable(cspace source/map/CSpace.cpp)
target_link_libraries(cspace multirobotexploration ${catkin_LIBRARIES})

add_executable(localdynamicmap source/map/LocalDynamicMap.cpp)
target_link_libraries(localdynamicmap multirobotexploration ${catkin_LIBRARIES})

add_executable(fbmapfusionrelative source/map/FBMapFusionRelative.cpp)
target_link_libraries(fbmapfusionrelative multirobotexploration ${OpenCV_LIBS} ${catkin_LIBRARIES})

#############
## sensors ##
#############
add_executable(lasertoworld source/laser/LaserToWorld.cpp)
target_link_libraries(lasertoworld multirobotexploration ${catkin_LIBRARIES})

add_executable(lasernoiser source/laser/LaserNoiser.cpp)
target_link_libraries(lasernoiser multirobotexploration ${catkin_LIBRARIES})

##############
## policies ##
##############
add_executable(alysson2024 source/policies/Alysson2024.cpp)
target_link_libraries(alysson2024 multirobotexploration ${catkin_LIBRARIES})

add_executable(randomizedsocialwelfare source/policies/RandomizedSocialWelfare.cpp)
target_link_libraries(randomizedsocialwelfare multirobotexploration ${catkin_LIBRARIES})

add_executable(yamauchi1999 source/policies/Yamauchi1999.cpp)
target_link_libraries(yamauchi1999 multirobotexploration ${catkin_LIBRARIES})

#############
## Install ##
#############
catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)