/*
 * Copyright (c) 2020, Alysson Ribeiro da Silva
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. All advertising materials mentioning features or use of this software must
 *    display the following acknowledgement:
 *    This product includes software developed by Alysson Ribeiro da Silva.
 * 
 * 4. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * 5. The source and the binary form, and any modifications made to them
 *    may not be used for the purpose of training or improving machine learning
 *    algorithms, including but not limited to artificial intelligence, natural
 *    language processing, or data mining. This condition applies to any derivatives,
 *    modifications, or updates based on the Software code. Any usage of the source
 *    or the binary form in an AI-training dataset is considered a breach of
 *    this License.
 * 
 * THIS SOFTWARE IS PROVIDED BY COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * ros master dependency
 */
#include "ros/ros.h"
#include "ros/package.h"

/*
 * messages used to read ccupancy grid data
 * processed configuration spaces
 * and estimate pose from the particle filter
 */
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int8MultiArray.h"
#include "multirobotsimulations/CustomPose.h"
#include "multirobotsimulations/rendezvous.h"
#include "SearchAlgorithms.h"
#include "nav_msgs/OccupancyGrid.h"
#include "Common.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Float64MultiArray.h"

/*
 * tf for potential fields and navigation stuff
 */
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/tf.h"
#include "tf/LinearMath/Scalar.h"
#include "multirobotsimulations/Frontiers.h"

// numeric limits for max and min 
#include <limits>
#include <vector>
#include <queue>
#include <list>
#include <map>
#include <random>
#include <time.h>
#include <iostream>
#include <sstream>
#include <fstream>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Path.h"

typedef enum {
    state_select_frontier = 2,
    state_exploring = 3,
    state_idle = 11,
    state_exploration_finished = 12,
    state_back_to_base = 25,
    state_back_to_base_finished = 26,
    state_compute_centroids = 27,
    state_waiting_centroids = 28,
    state_set_back_to_base = 30,
    state_planning = 31,
} ExplorerState;

// store the last obtained pose for processing the 
// current position estimate
int TICKS_COUNTER;
int CURRENT_STATE;

bool HAS_POSE;
bool RECEIVED_FRONTIERS;
bool RECEIVED_CS;

Vec2i POS;
tf::Vector3 WORLD_POS;
multirobotsimulations::Frontiers CENTROIDS;
nav_msgs::OccupancyGrid OCC;

std::random_device rd;
std::mt19937 gen(rd());

void Initialize(void) {
    // control variables
    HAS_POSE = false;
    RECEIVED_FRONTIERS = false;
    RECEIVED_CS = false;

    // node behavior variables
    CURRENT_STATE = state_idle;

    // 10 exploration zones and 4 semantic actions
    TICKS_COUNTER = 0;
}

void EstimatePoseCallback(const multirobotsimulations::CustomPose& rMsg) {
    WORLD_POS.setX(rMsg.pose.position.x);
    WORLD_POS.setY(rMsg.pose.position.y);
    HAS_POSE = true;
}

void ClustersCallback(const multirobotsimulations::Frontiers& rMsg) {
    CENTROIDS.centroids.poses.assign(rMsg.centroids.poses.begin(), rMsg.centroids.poses.end());
    CENTROIDS.centroids.header = rMsg.centroids.header;
    CENTROIDS.costs.data.assign(rMsg.costs.data.begin(), rMsg.costs.data.end());
    CENTROIDS.utilities.data.assign(rMsg.utilities.data.begin(), rMsg.utilities.data.end());
    CURRENT_STATE = state_select_frontier;
    RECEIVED_FRONTIERS = true;
}

void SetIdleCallback(const std_msgs::String& rMsg) {
    CURRENT_STATE = state_idle;
}

void SubGoalFinishCallback(const std_msgs::String& rMsg) {
    if(CURRENT_STATE == state_exploring) {
        CURRENT_STATE = state_exploration_finished;
    }
    if(CURRENT_STATE == state_back_to_base) {
        CURRENT_STATE = state_back_to_base_finished;
    }
}

void CSpaceCallback(const nav_msgs::OccupancyGrid& rMsg) {
    OCC.info = rMsg.info;
    OCC.header = rMsg.header;
    RECEIVED_CS = true;
}

void SetExploringCallback(const std_msgs::String& rMsg) {
    CURRENT_STATE = state_compute_centroids;
}

void SetMotherbaseCallback(const std_msgs::String& rMsg) {
    CURRENT_STATE = state_set_back_to_base;  
}

int SelectFrontier(multirobotsimulations::Frontiers& rCentroids, 
                    tf::Vector3& rOutFrontierWorld) {
    rOutFrontierWorld.setX(rCentroids.centroids.poses[rCentroids.highest_utility_index].position.x);
    rOutFrontierWorld.setY(rCentroids.centroids.poses[rCentroids.highest_utility_index].position.y);
    return rCentroids.highest_utility_index;
}

int RoulettFrontier(multirobotsimulations::Frontiers& rCentroids, 
                    tf::Vector3& rOutFrontierWorld) {
    std::uniform_int_distribution<int> distr(0, rCentroids.centroids.poses.size()-1); // define the range
    int selected = distr(gen); 
    rOutFrontierWorld.setX(rCentroids.centroids.poses[selected].position.x);
    rOutFrontierWorld.setY(rCentroids.centroids.poses[selected].position.y);
    return selected;  
}

void SetGoal(tf::Vector3& rGoal, ros::Publisher& rPublisher) {
    geometry_msgs::Pose pose_msg;
    pose_msg.position.x = rGoal.getX();
    pose_msg.position.y = rGoal.getY();
    rPublisher.publish(pose_msg);
}

void CreateMarker(visualization_msgs::Marker& rInput, const char* pNs, const int& rId, const int& rSeq) {
    rInput.id = rId;
    rInput.header.frame_id = std::string("robot_") + std::to_string(rId) + std::string("/map");
    rInput.header.stamp = ros::Time().now();
    rInput.ns = pNs;
    rInput.points.clear();
    rInput.type = visualization_msgs::Marker::CYLINDER;
    rInput.action = visualization_msgs::Marker::MODIFY;
    rInput.pose.orientation.x = 0.0;
    rInput.pose.orientation.y = 0.0;
    rInput.pose.orientation.z = 0.0;
    rInput.pose.orientation.w = 1.0;
    rInput.scale.x = 0.5;
    rInput.scale.y = 0.5;
    rInput.scale.z = 0.5;
    rInput.color.a = 1.0;
    rInput.color.r = 0.3;
    rInput.color.g = 1.0;
    rInput.color.b = 0.0;
    rInput.lifetime = ros::Duration(1);
}

bool CheckNear(const std_msgs::Int8MultiArray& rMsg, const int& rRobotId) {
    for(size_t i = 0; i < rMsg.data.size(); ++i) {
        if(i == rRobotId) continue;
        if(rMsg.data[i] == 1) return true;
    }
    return false;
}

int main(int argc, char* argv[]) {
    /*
     * ros initialization of the node
     */
    srand (time(NULL));
    ros::init(argc, argv, "randomizedsw");
    ros::NodeHandle node_handle;
    ros::NodeHandle private_handle("~");
    std::string ns = ros::this_node::getNamespace();
    Initialize();

    /////////////////////////////////////////////////////////////////
    //                      Yamauchi 1999 Base                     //
    /////////////////////////////////////////////////////////////////

    /*
     * loop frequency to publish stuff
     */
    int queue_size = 1;
    int rate = 10;
    int robots = 1;
    int robot_id = -1;
    double delta_time = 0.0;
    ros::Time last_time(0,0);

    /*
     * Rendezvous related stuff
     */
    bool set_motherbase = false;
    tf::Vector3 goal_frontier;
    tf::Vector3 goal_motherbase;
    std::vector<ros::Subscriber> subs;

    node_handle.getParam("/robots", robots);
    private_handle.getParam("id", robot_id);
    private_handle.getParam("rate", rate);
    private_handle.getParam("queue_size", queue_size);

    ros::Rate loop_frequency(rate);

    /*
     * Outer nodes topics
     */
    subs.push_back(node_handle.subscribe(ns + "/c_space", queue_size, &CSpaceCallback));
    subs.push_back(node_handle.subscribe(ns + "/frontier_discovery/frontiers_clusters", queue_size, &ClustersCallback));
    subs.push_back(node_handle.subscribe(ns + "/gmapping_pose/world_pose", queue_size, &EstimatePoseCallback));
    subs.push_back(node_handle.subscribe(ns + "/sub_goal_nav/finish", queue_size, &SubGoalFinishCallback));

    ros::Publisher sub_goal_pub = node_handle.advertise<geometry_msgs::Pose>(ns + "/sub_goal_nav/goal", queue_size);
    ros::Publisher fro_comp = node_handle.advertise<std_msgs::String>(ns + "/frontier_discovery/compute", queue_size);

    /*
     * This node's topics
     */    
    subs.push_back(node_handle.subscribe(ns + "/explorer/set_idle", queue_size, &SetIdleCallback));
    subs.push_back(node_handle.subscribe(ns + "/explorer/set_exploring", queue_size, &SetExploringCallback));
    subs.push_back(node_handle.subscribe("/global_explorer/back_to_base", queue_size, &SetMotherbaseCallback));
    subs.push_back(node_handle.subscribe("/global_explorer/set_exploring", queue_size, &SetExploringCallback));

    // ------------------ End Yamauchi 1999 base ----------------- //
    

    /////////////////////////////////////////////////////////////////
    //        Basic Utility Theory Randomized SocialWelfare        //
    /////////////////////////////////////////////////////////////////

    // get the relative distances to check if this robot should use
    // the randomized utility or not
    std_msgs::Int8MultiArray robots_in_comm;
    bool received_comm = false;
    
    std_msgs::Int8MultiArray* robots_in_comm_ptr = &robots_in_comm;
    bool* received_distances_ptr = &received_comm;
    ros::Subscriber dist_sub = node_handle.subscribe<std_msgs::Int8MultiArray>(
        ns + "/mock_communication_model/robots_in_comm", 
        queue_size,
        [received_distances_ptr, robots_in_comm_ptr](std_msgs::Int8MultiArray::ConstPtr rMsg) {
            robots_in_comm_ptr->data.assign(rMsg->data.begin(), rMsg->data.end());
            (*received_distances_ptr) = true;
        });

    // ---- End Basic Utility Theory Randomized SocialWelfare ---- //


    while(ros::ok()) {
        if(HAS_POSE && RECEIVED_CS && received_comm) {
            WorldToMap(OCC, WORLD_POS, POS);

            if(set_motherbase == false) {
                set_motherbase = true;
                goal_motherbase.setX(WORLD_POS.getX());
                goal_motherbase.setY(WORLD_POS.getY());
            }

            switch(CURRENT_STATE) {
               case state_idle:
                    // just wait for command
               break;

               case state_compute_centroids:
                    // ask for centroids to avoid
                    // unnecessary computations
                    fro_comp.publish(std_msgs::String());
                    CURRENT_STATE = state_waiting_centroids;
               break;

               case state_waiting_centroids:
                    // just wait for the centroids to arrive
               break;

               case state_select_frontier:
                    if(CENTROIDS.centroids.poses.size() == 0) {
                        SetGoal(goal_motherbase, sub_goal_pub);
                        CURRENT_STATE = state_set_back_to_base;
                        ROS_INFO("[%s Explorer] Not more clusters to explore [%.2f %.2f]", 
                                 ns.c_str(), 
                                 goal_motherbase.getX(), 
                                 goal_motherbase.getY());
                        break;
                    }
                    
                    if(CENTROIDS.centroids.poses.size() > 0) {
                        if(CheckNear(robots_in_comm, robot_id)) {
                            RoulettFrontier(CENTROIDS, goal_frontier);
                            ROS_INFO("[%s Explorer] roulett frontier for randomized utility.", ns.c_str());
                        } else {
                            SelectFrontier(CENTROIDS, goal_frontier);
                            ROS_INFO("[%s Explorer] maximizing utility.", ns.c_str());
                        }
                        
                        SetGoal(goal_frontier, sub_goal_pub);
                        CURRENT_STATE = state_exploring;
                        ROS_INFO("[%s Explorer] selected frontier [%.2f %.2f]", 
                                    ns.c_str(),
                                    goal_frontier.getX(),
                                    goal_frontier.getY());
                    } else {
                        CURRENT_STATE = state_set_back_to_base;
                    }
               break;

               case state_set_back_to_base:
                    SetGoal(goal_motherbase, sub_goal_pub);
                    CURRENT_STATE = state_back_to_base;
                    ROS_INFO("[%s Explorer] going back to base at [%.2f %.2f]", 
                                ns.c_str(), 
                                goal_motherbase.getX(), 
                                goal_motherbase.getY());                
               break;

               case state_back_to_base:

               break;

               case state_back_to_base_finished:
                    CURRENT_STATE = state_idle;
                    ROS_INFO("[%s Explorer] reached motherbase.", ns.c_str());
               break;

               case state_exploration_finished:
                    CURRENT_STATE = state_compute_centroids;
                    ROS_INFO("[%s Explorer] reached frontier.", ns.c_str());
               break;
            }
            
        }

        delta_time = ros::Time::now().sec - last_time.sec;
        ROS_INFO("Current state %d delta time: %f", CURRENT_STATE, delta_time);

        // run spin to get the data
        last_time = ros::Time::now();
        ros::spinOnce();
        loop_frequency.sleep();
    }

    return 0;
}