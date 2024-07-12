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

#include <stdio.h>
#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "multirobotsimulations/CustomPose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "Common.h"
#include "visualization_msgs/Marker.h"
#include "SearchAlgorithms.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <queue>
#include "std_msgs/Float32.h"

typedef enum {
    state_idle = 0,
    state_executing_path = 1,
    state_set_subgoal = 2,
    state_set_clear = 3,
    state_clearing = 4,
    state_finish_clearing = 5
} SubGoalState;

// store the last obtained pose for processing the 
// current position estimate
bool HAS_POSE;
bool CELL_DECOMPOSITION_RECEIVED;
bool RECEIVED_FRONTIERS;
bool HAS_OBJECTIVE_FRONTIER;
bool HAS_AVERAGE_DISPLACEMENT;
nav_msgs::OccupancyGrid OCC;

// everything tf::vector3 is in world coordinates
// others are in map coordinates, such as Vec2i
std::list<Vec2i> SUB_GOAL_PATH;
tf::Vector3 LAST_POS;
tf::Vector3 WORLD_POS;
tf::Vector3 CURRENT_GOAL;
Vec2i POS;
double YAW;
double SUB_GOAL_THRESHOLD;
double AVERAGE_DISPLACEMENT;
SubGoalState CURRENT_STATE;

void Initialize() {
    HAS_POSE = false;
    CELL_DECOMPOSITION_RECEIVED = false;
    RECEIVED_FRONTIERS = false;
    HAS_OBJECTIVE_FRONTIER = false;
    HAS_AVERAGE_DISPLACEMENT = false;
    YAW = 0.0;
    SUB_GOAL_THRESHOLD = 1.0;
    CURRENT_STATE = state_idle;
    AVERAGE_DISPLACEMENT = 0.0;
}

void AverageDisplacementCallback(const std_msgs::Float32& rMsg) {
    AVERAGE_DISPLACEMENT = rMsg.data;
    HAS_AVERAGE_DISPLACEMENT = true;
}

void EstimatePoseCallback(const multirobotsimulations::CustomPose& rMsg) {
    WORLD_POS.setX(rMsg.pose.position.x);
    WORLD_POS.setY(rMsg.pose.position.y);
    YAW = tf::getYaw(rMsg.pose.orientation);
    HAS_POSE = true;
}

void CSpaceCallback(const nav_msgs::OccupancyGrid& rMsg) {
    OCC.data.assign(rMsg.data.begin(), rMsg.data.end());
    OCC.info = rMsg.info;
    OCC.header = rMsg.header;
    CELL_DECOMPOSITION_RECEIVED = true;
}

void SetIdleCallback(const std_msgs::String& rMsg) {
    CURRENT_STATE = state_idle;
}

void GoalCallback(const geometry_msgs::Pose& rMsg) {
    CURRENT_GOAL.setX(rMsg.position.x);
    CURRENT_GOAL.setY(rMsg.position.y);
    CURRENT_STATE = state_set_subgoal;
}

void ClearCompleteCallback(const std_msgs::String& rMsg) {
    CURRENT_STATE = state_finish_clearing;
}

void ClearCallback(const std_msgs::String& rMsg) {
    CURRENT_STATE = state_set_clear;
}

void CreateMarker(visualization_msgs::Marker& rInput, const char* pNs, const int& rId, const int& rSeq) {
    rInput.id = rId;
    rInput.header.frame_id = "robot_" + std::to_string(rId) + std::string("/map");
    rInput.header.stamp = ros::Time().now();
    rInput.ns = pNs;
    rInput.points.clear();
    rInput.type = visualization_msgs::Marker::LINE_STRIP;
    rInput.action = visualization_msgs::Marker::MODIFY;
    rInput.pose.position = OCC.info.origin.position;
    rInput.pose.orientation.x = 0.0;
    rInput.pose.orientation.y = 0.0;
    rInput.pose.orientation.z = 0.0;
    rInput.pose.orientation.w = 1.0;
    rInput.scale.x = 0.25;
    rInput.scale.y = 0.25;
    rInput.scale.z = 0.5;
    rInput.color.a = 1.0;
    rInput.color.r = 1.0;
    rInput.color.g = 0.0;
    rInput.color.b = 0.0;
    rInput.lifetime = ros::Duration(1);
}

void StopCallBack(std_msgs::String) {
    CURRENT_STATE = state_idle;
    SUB_GOAL_PATH.clear();
}

void DepthFirstSearchFreePath(
    nav_msgs::OccupancyGrid& rCSpace, 
    Vec2i& rAgentPos,
    Vec2i& rSource, 
    Vec2i& rClosest,
    std::list<Vec2i>& rOutPath) {

    // used to mark found frontiers and clusters
    // used to mark found frontiers and clusters
    Vec2i source = rSource;
    Matrix<bool> visited(rCSpace.info.width, rCSpace.info.height);
    visited.clear(0);

    // filter source to maximum cell decomposition bounds
    if(source.x >= rCSpace.info.width) source.x = rCSpace.info.width - 1;
    if(source.y >= rCSpace.info.height) source.y = rCSpace.info.height - 1;
    if(source.x < 0) source.x = 0;
    if(source.y < 0) source.y = 0;

    std::queue<Vec2i> q;
    q.push(source);
    Vec2i current;
    visited[source.y][source.x] = true;
    while(q.size() > 0) {
        current = q.front();
        q.pop();
        sa::ComputePath(rCSpace, rAgentPos, current, rOutPath);
        if(rOutPath.size() != 0) {
            rClosest = current;
            break;
        }

        for(int col = 0; col < 3; ++col) {
            for(int row = 0; row < 3; ++row) {
                Vec2i temp = Vec2i::Create(current.x - 1 + col,current.y - 1 + row);
                if(sa::IsInBounds(rCSpace, temp) && col != row && visited[temp.y][temp.x] == false) {
                    q.push(temp);
                    visited[temp.y][temp.x] = true;
                }
            }
        }
    }
}

void GetParameters(ros::NodeHandle& node_handle, const std::string& ns, int& id, int& queue_size, double& rate, double& sub_goal_threshold, double& delta_threshold) {
    if (!node_handle.getParam("id", id)) {
        throw std::runtime_error("Parameter id not found");
    }
    if (!node_handle.getParam("rate", rate)) {
        throw std::runtime_error("Parameter rate_subgoal not found");
    }
    if (!node_handle.getParam("queue_size", queue_size)) {
        throw std::runtime_error("Parameter queue_size not found");
    }
    if (!node_handle.getParam("reach_threshold", sub_goal_threshold)) {
        throw std::runtime_error("Parameter reach_threshold not found");
    }
    if(!node_handle.getParam("delta_threshold", delta_threshold)) {
        throw std::runtime_error("Parameter delta_threshold not found");
    }
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "node_sub_goal_nav");
    ros::NodeHandle node_handle;
    ros::NodeHandle private_handle("~");

    // node's parameters
    int queue_size;
    int id;
    double rate;
    int seq;
    bool has_viable_subgoal;
    double distance;
    std::string ns;
    std_msgs::String msg_str;
    std_msgs::Bool msg_bool;
    geometry_msgs::Pose pose_msg;
    visualization_msgs::Marker p;
    nav_msgs::Path to_publish;
    Vec2i temp_goal;
    ros::Time last_time;

    // invalid goal controls
    int delta_time_sec;
    double delta_threshold;
    double time_delta_bellow_threshold;

    // init
    queue_size = 1;
    id = -1;
    rate = 10;
    seq = 0;
    last_time = ros::Time::now();
    time_delta_bellow_threshold = 0.0;
    Initialize();

    try {
        ns = node_handle.getNamespace();
        GetParameters(private_handle, ns, id, queue_size, rate, SUB_GOAL_THRESHOLD, delta_threshold);
    } catch (const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
    }

    ros::Rate loop_frequency(rate);

    /*
     * Outer nodes topics
     */
    ros::Subscriber sub_clear_complete = node_handle.subscribe(ns + "/desai_controller/clear_complete", queue_size, &ClearCompleteCallback);
    ros::Subscriber cell_sub =  node_handle.subscribe(ns + "/c_space", queue_size, &CSpaceCallback);
    ros::Subscriber estimate_pose_sub = node_handle.subscribe(ns + "/gmapping_pose/world_pose", queue_size, &EstimatePoseCallback);
    ros::Subscriber average_displace_sub = node_handle.subscribe(ns + "/avgd_average_displacement", queue_size, &AverageDisplacementCallback);
    ros::Publisher  field_pub = node_handle.advertise<geometry_msgs::Pose>(ns + "/pfield_local_planner/goal_occ", queue_size);
    ros::Publisher enable = node_handle.advertise<std_msgs::Bool>(ns + "/desai_controller/enable", queue_size);
    ros::Publisher clear_position = node_handle.advertise<std_msgs::String>(ns + "/desai_controller/set_clear_state", queue_size);
    ros::Publisher goal_position = node_handle.advertise<std_msgs::String>(ns + "/desai_controller/set_goal_state", queue_size);
    ros::Publisher idle = node_handle.advertise<std_msgs::String>(ns + "/desai_controller/set_idle_state", queue_size);
    
    /*
     * This node network interactions
     */
    ros::Subscriber goal_sub = node_handle.subscribe(ns + "/sub_goal_nav/goal", queue_size, &GoalCallback);
    ros::Subscriber clear_sub = node_handle.subscribe(ns + "/sub_goal_nav/clear", queue_size, &ClearCallback);
    ros::Subscriber stop_callback = node_handle.subscribe(ns + "/sub_goal_nav/stop", queue_size, &StopCallBack);
    ros::Publisher path_pub =  node_handle.advertise<visualization_msgs::Marker>(ns + "/sub_goal_nav/path", queue_size);
    ros::Publisher sub_goal_finish_pub = node_handle.advertise<std_msgs::String>(ns + "/sub_goal_nav/finish", queue_size);
    ros::Publisher clear_pub = node_handle.advertise<std_msgs::String>(ns + "/sub_goal_nav/clear_finish", queue_size);
    ros::Publisher flw_pub = node_handle.advertise<nav_msgs::Path>(ns + "/sub_goal_nav/current_path", queue_size);

    while(ros::ok()) {
        if(CELL_DECOMPOSITION_RECEIVED == true && HAS_POSE == true && HAS_AVERAGE_DISPLACEMENT == true) {
            WorldToMap(OCC, WORLD_POS, POS);
            to_publish.poses.clear();
            switch(CURRENT_STATE) {
                case state_idle:
                    // if in idle mode always clear path
                    SUB_GOAL_PATH.clear();
                    msg_bool.data = false;

                    // tell potential field to stay still
                    enable.publish(msg_bool);
                    idle.publish(msg_str);
                break;
                case state_set_subgoal:
                    // tell the potential field to start following goal
                    msg_bool.data = true;
                    goal_position.publish(msg_str);
                    enable.publish(msg_bool);

                    // change state to exploring
                    CURRENT_STATE = state_executing_path;
                break;
                case state_executing_path:
                    // goal conversion to grid coordinates
                    // to avoid wrong conversions due to 
                    // the OCC dynamic nature
                    WorldToMap(OCC, CURRENT_GOAL, temp_goal);
 
                    /*
                    * Compute a path from the cell position to the selected free space
                    * that is near to the frontier estimate pose
                    */
                    DepthFirstSearchFreePath(OCC, 
                                             POS, 
                                             temp_goal, 
                                             temp_goal,
                                             SUB_GOAL_PATH);
                    MapToWorld(OCC, temp_goal, CURRENT_GOAL);
                    // sa::ComputePath(OCC, POS, temp_goal, SUB_GOAL_PATH);
                                             
                    distance = WORLD_POS.distance(CURRENT_GOAL);
                    if(distance <= SUB_GOAL_THRESHOLD) {
                        CURRENT_STATE = state_idle;
                        sub_goal_finish_pub.publish(msg_str);
                        SUB_GOAL_PATH.clear();
                        ROS_INFO("[%s SubGoalNav] path ended.", ns.c_str());                        
                    } else {
                        if(SUB_GOAL_PATH.size() > 0 ) {
                            CreateMarker(p, ns.c_str(), id, seq);

                            for(auto& lit : SUB_GOAL_PATH) {
                                geometry_msgs::Point pose;
                                pose.x = lit.x * OCC.info.resolution;
                                pose.y = lit.y * OCC.info.resolution;
                                pose.z = 0.0;
                                p.points.push_back(pose);

                                geometry_msgs::PoseStamped pose_msg;
                                pose_msg.header = OCC.header;
                                pose_msg.pose.orientation = geometry_msgs::Quaternion();

                                tf::Vector3 world;
                                MapToWorld(OCC, lit, world);
                                
                                pose_msg.pose.position.x = world.getX();
                                pose_msg.pose.position.y = world.getY();
                                to_publish.poses.push_back(pose_msg);
                            }

                            path_pub.publish(p);

                            // check if it is stuck
                            if(AVERAGE_DISPLACEMENT < 0.01) {
                                delta_time_sec = ros::Time::now().sec - last_time.sec;
                                time_delta_bellow_threshold += delta_time_sec;
                                // ROS_INFO("[%s SubGoalnav] time stuck: %f.", ns.c_str(), time_delta_bellow_threshold);
                            } else {
                                time_delta_bellow_threshold = 0.0;
                            }
                        } 

                        if(time_delta_bellow_threshold > delta_threshold || SUB_GOAL_PATH.size() == 0) {
                            CURRENT_STATE = state_idle;
                            sub_goal_finish_pub.publish(msg_str);
                            SUB_GOAL_PATH.clear();
                            time_delta_bellow_threshold = 0.0;
                            ROS_INFO("[%s SubGoalNav] should reset planner.", ns.c_str());   
                        }
                    }
                    
                break;
                // TODO:impement clear state in MRE Local Planner
                case state_set_clear:
                    SUB_GOAL_PATH.clear();
                    has_viable_subgoal = false;
                    msg_bool.data = true;
                    enable.publish(msg_bool);
                    clear_position.publish(msg_str);
                    CURRENT_STATE = state_clearing;
                    ROS_INFO("[%s SubGoalNav] entering clearing state.", ns.c_str());
                break;
                case state_clearing:
                    // just wait
                break;
                case state_finish_clearing:
                    clear_pub.publish(msg_str);
                    CURRENT_STATE = state_idle;
                    ROS_INFO("[%s SubGoalNav] clearing finished.", ns.c_str());
                break;
            }

            flw_pub.publish(to_publish);
        }

        seq++;
        last_time = ros::Time::now();
        ros::spinOnce();        
        loop_frequency.sleep();
    }

    return 0;
}