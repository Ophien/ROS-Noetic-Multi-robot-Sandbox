/*
 * Copyright (c) 2023, Alysson Ribeiro da Silva
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
#include "multirobotsimulations/rendezvous.h"
#include "RendezvousPlan.h"

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
    
    state_set_rendezvous_location = 55,
    state_navigating_to_rendezvous = 56,
    state_at_rendezvous = 57,
    state_updating_plan = 58,
    state_waiting_consensus = 59,
    state_waiting_centroids_for_plan = 60,
    state_select_new_rendezvous = 61
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

std::string StateToStr(const int& state) {
    switch(state) {
        case state_select_frontier:
            return "state_select_frontier";
        break;
        case state_exploring:
            return "state_exploring";
        break;
        case state_idle:
            return "state_idle";
        break;
        case state_exploration_finished:
            return "state_exploration_finished";
        break;
        case state_back_to_base:
            return "state_back_to_base";
        break;
        case state_back_to_base_finished:
            return "state_back_to_base_finished";
        break;
        case state_compute_centroids:
            return "state_compute_centroids";
        break;
        case state_waiting_centroids:
            return "state_waiting_centroids";
        break;
        case state_set_back_to_base:
            return "state_set_back_to_base";
        break;
        case state_planning:
            return "state_planning";
        break;
        case state_set_rendezvous_location:
            return "state_set_rendezvous_location";
        break;
        case state_navigating_to_rendezvous:
            return "state_navigating_to_rendezvous";
        break;
        case state_at_rendezvous:
            return "state_at_rendezvous";
        break;
        case state_updating_plan:
            return "state_updating_plan";
        break;
        case state_waiting_consensus:
            return "state_waiting_consensus";
        break;
        case state_waiting_centroids_for_plan:
            return "state_waiting_centroids_for_plan";
        break;
        case state_select_new_rendezvous:
            return "state_select_new_rendezvous";
        break;
    }
    return "";
}

void ChangeState(const int& newState) {
    CURRENT_STATE = newState;
    ROS_INFO("[Explorer] Current state %s", StateToStr(CURRENT_STATE).c_str());
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

    if(CURRENT_STATE == state_waiting_centroids) 
        CURRENT_STATE = state_select_frontier;
    else if(CURRENT_STATE == state_waiting_centroids_for_plan)
        CURRENT_STATE = state_select_new_rendezvous;
    
    RECEIVED_FRONTIERS = true;
}

void SetIdleCallback(const std_msgs::String& rMsg) {
    CURRENT_STATE = state_idle;
}

void SubGoalFinishCallback(const std_msgs::String& rMsg) {
    if(CURRENT_STATE == state_exploring) {
        ChangeState(state_exploration_finished);
    }
    if(CURRENT_STATE == state_back_to_base) {
        ChangeState(state_back_to_base_finished);
    }
    if(CURRENT_STATE == state_navigating_to_rendezvous) {
        ChangeState(state_at_rendezvous);
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
                    tf::Vector3& rWorldPos, 
                    tf::Vector3& rOutFrontierWorld) {
    if(rCentroids.centroids.poses.size() == 0) rCentroids.centroids.poses.end();
    double max = std::numeric_limits<double>::min();
    tf::Vector3 temp_c;
    int selected = 0;
    double i_gain_heuristic = 0.0;
    ROS_INFO("[Explorer] %ld available frontiers.", rCentroids.centroids.poses.size());
    for(size_t i = 0; i < rCentroids.centroids.poses.size(); ++i) {
        temp_c.setX(rCentroids.centroids.poses[i].position.x);
        temp_c.setY(rCentroids.centroids.poses[i].position.y);
        i_gain_heuristic = rCentroids.utilities.data[i] - rCentroids.costs.data[i];

        // compute cosine of the frontier here too!

        ROS_INFO("\t[%.2f %.2f] information gain: %.2f", temp_c.getX(), temp_c.getY(), i_gain_heuristic);
        if(i_gain_heuristic > max) {
            max = i_gain_heuristic;
            selected = i;
        }
    }
    rOutFrontierWorld.setX(rCentroids.centroids.poses[selected].position.x);
    rOutFrontierWorld.setY(rCentroids.centroids.poses[selected].position.y);
    return selected;
}

int RoulettFrontier(multirobotsimulations::Frontiers& rCentroids, 
                    tf::Vector3& rWorldPos, 
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

bool CheckNear(const std_msgs::Int8MultiArray& rMsg, const int& rRobotId) {
    for(size_t i = 0; i < rMsg.data.size(); ++i) {
        if(i == rRobotId) continue;
        if(rMsg.data[i] == 1) return true;
    }
    return false;
}

bool CanCommunicate(const std_msgs::Int8MultiArray& rMsg, const int& otherId) {
    if(otherId < 0 || otherId >= rMsg.data.size()) throw std::out_of_range("Robot id is out of range in CanCommunicate.");
    if(rMsg.data[otherId] == 1) return true;
    return false;
}

bool FinishedMission() {
    return (CURRENT_STATE == state_back_to_base ||
            CURRENT_STATE == state_back_to_base_finished ||
            CURRENT_STATE == state_set_back_to_base);
}

int main(int argc, char* argv[]) {
    /*
     * ros initialization of the node
     */
    srand (time(NULL));
    ros::init(argc, argv, "alysson2024");
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
    int queue_size = -1;
    int rate = -1;
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
    //        Alysson 2024 Rendezvous realization control          //
    /////////////////////////////////////////////////////////////////

    tf::Vector3 rendezvous_footprint;
    tf::Vector3 first_rendezvous;
    tf::Vector3 goal_rendezvous;
    std::map<std::string, double> pose;
    std::shared_ptr<RendezvousPlan> planPtr = std::make_shared<RendezvousPlan>(node_handle, robot_id);

    // messages to broadcast intentions regarding which rendezvous to realize
    // and how to update the plan
    multirobotsimulations::rendezvous rendezvous_msg;
    multirobotsimulations::CustomPose rendezvous_new_pose_msg;

    // safety cluster mechanism
    double waiting_threshold = 120.0;
    double time_waiting = 0.0;

    // first rendezvous location for all robots
    node_handle.getParam("/first_rendezvous", pose);
    first_rendezvous.setX(pose["x"]);
    first_rendezvous.setY(pose["y"]);
    ROS_INFO("[Explorer] First rendezvous location: %f %f", pose["x"], pose["y"]); 

    // load rendezvous footprint
    std::string key = "/footprint_robot_" + std::to_string(robot_id);
    node_handle.getParam(key, pose);
    rendezvous_footprint.setX(pose["x"]);
    rendezvous_footprint.setY(pose["y"]);
    ROS_INFO("[Explorer] Rendezvous footprint: %f %f %f", pose["x"], pose["y"], pose["z"]);

    // waiting at rendezvous message id should always be this robot's id
    rendezvous_msg.robot_id = robot_id;
    rendezvous_new_pose_msg.robot_id = robot_id;
    
    /*
     * Initialize the rendezvous plan handler with the 
     * initial rendezvous location
     */
    planPtr->InitializeLocation(first_rendezvous);
    planPtr->PrintLocations();
    planPtr->Print();

    // get the relative distances to check if this robot should use
    // the randomized utility or not
    std_msgs::Int8MultiArray robots_in_comm;
    bool received_comm = false;
    std_msgs::Int8MultiArray* robots_in_comm_ptr = &robots_in_comm;
    bool* received_distances_ptr = &received_comm;

    subs.push_back(node_handle.subscribe<std_msgs::Int8MultiArray>(
        ns + "/mock_communication_model/robots_in_comm", 
        queue_size,
        [received_distances_ptr, robots_in_comm_ptr](std_msgs::Int8MultiArray::ConstPtr rMsg) {
            // mock communication model and receive plans from robots that can
            // communicate
            robots_in_comm_ptr->data.assign(rMsg->data.begin(), rMsg->data.end());
            (*received_distances_ptr) = true;
        }));

    /*
     * This publisher is used to tell other robots which plan I'm trying to fulfill
     */
    ros::Publisher plan_realization_pu = node_handle.advertise<multirobotsimulations::rendezvous>(ns+"/realizing_plan", queue_size);

    /*
     * This set of subscribers keeps listening plan realization broadcasts
     */
    for(int robot = 0; robot < robots; ++robot) {
        if(robot == robot_id) continue;
        subs.push_back(node_handle.subscribe<multirobotsimulations::rendezvous>(
            "/robot_" + std::to_string(robot) + "/realizing_plan",
            queue_size,
            [planPtr, robots_in_comm_ptr, robot_id](multirobotsimulations::rendezvous::ConstPtr rMsg) {
                // realize plan if it is the same as mine
                if(rMsg->plan == planPtr->GetCurrentAgreementUniqueID() &&
                   CanCommunicate((*robots_in_comm_ptr), rMsg->robot_id))
                        planPtr->RealizePlan(rMsg->robot_id);
            }
        ));
    }

    /*
     * This publisher is used to tell how to update the current sub-team rendezvous location
     */
    ros::Publisher plan_location_updater = node_handle.advertise<multirobotsimulations::CustomPose>(ns+"/plan_updater", queue_size);

    /*
     * This set of subscribers are used to received plan updates regarding the sub-teams' rendezvous locations
     * from robots realizing the same plan and that can communicate
     */
    tf::Vector3 new_rendezvous_location;
    bool received_new_rendezvous_location = false;
    tf::Vector3* nrl_ptr = &new_rendezvous_location;
    bool* rec_ptr = &received_new_rendezvous_location;

    for(int robot = 0; robot < robots; ++robot) {
        if(robot == robot_id) continue;
        subs.push_back(node_handle.subscribe<multirobotsimulations::CustomPose>(
            "/robot_" + std::to_string(robot) + "/plan_updater",
            queue_size,
            [planPtr, robots_in_comm_ptr,rec_ptr, nrl_ptr](multirobotsimulations::CustomPose::ConstPtr rMsg) {
                // this should work because robots cannot rendezvous at two 
                // simultaneous places
                int waiting_id_from_consensus = planPtr->GetCurrentAgreementConsensusID();

                // realize plan if it is the same as mine
                // always check if can communicate to simulate networking
                if(waiting_id_from_consensus == rMsg->robot_id &&
                   CanCommunicate((*robots_in_comm_ptr), rMsg->robot_id)) {
                        nrl_ptr->setX(rMsg->pose.position.x);
                        nrl_ptr->setY(rMsg->pose.position.y);
                        nrl_ptr->setZ(rMsg->pose.position.z);
                        (*rec_ptr) = true;
                   }
            }
        ));
    }

    // ------------ End Rendezvous Realization Control ----------- //
    

    while(ros::ok()) {
        if(HAS_POSE && RECEIVED_CS && received_comm && last_time.sec > 0) {
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
                    ChangeState(state_waiting_centroids);
               break;

               case state_waiting_centroids:
                    // just wait for the centroids to arrive
               break;

               case state_select_frontier:
                    if(CENTROIDS.centroids.poses.size() == 0) {
                        SetGoal(goal_motherbase, sub_goal_pub);
                        ChangeState(state_set_back_to_base);
                        ROS_INFO("[Explorer] Not more clusters to explore [%.2f %.2f]", 
                                 goal_motherbase.getX(), 
                                 goal_motherbase.getY());
                        break;
                    }
                    
                    if(CENTROIDS.centroids.poses.size() > 0) {
                        if(CheckNear(robots_in_comm, robot_id)) {
                            RoulettFrontier(CENTROIDS, WORLD_POS, goal_frontier);
                            ROS_INFO("[Explorer] roulett frontier for randomized utility.");
                        } else {
                            SelectFrontier(CENTROIDS, WORLD_POS, goal_frontier);
                            ROS_INFO("[Explorer] maximizing utility.");
                        }
                        
                        SetGoal(goal_frontier, sub_goal_pub);
                        ChangeState(state_exploring);
                        ROS_INFO("[Explorer] selected frontier [%.2f %.2f]", 
                                    goal_frontier.getX(),
                                    goal_frontier.getY());
                    } else {
                        ChangeState(state_set_back_to_base);
                    }
               break;

               case state_set_back_to_base:
                    SetGoal(goal_motherbase, sub_goal_pub);
                    ChangeState(state_back_to_base);
                    ROS_INFO("[Explorer] going back to base at [%.2f %.2f]",  
                                goal_motherbase.getX(), 
                                goal_motherbase.getY());                
               break;

               case state_back_to_base:

               break;

               case state_back_to_base_finished:
                    ChangeState(state_idle);
                    ROS_INFO("[Explorer] reached motherbase.");
               break;

               case state_exploration_finished:
                    ChangeState(state_compute_centroids);
                    ROS_INFO("[Explorer] reached frontier.");
               break;

               case state_set_rendezvous_location:
                    goal_rendezvous = planPtr->GetCurrentAgreementLocation();

                    // consider the footprint for this robot
                    // this was one of the most efficient ways to avoid
                    // collision
                    goal_rendezvous += rendezvous_footprint;

                    SetGoal(goal_rendezvous, sub_goal_pub);
                    ChangeState(state_navigating_to_rendezvous);
                    ROS_INFO("[Explorer] fulfilling plan %d of unique id %d",
                                planPtr->GetCurrentAgreement(), 
                                planPtr->GetCurrentAgreementUniqueID());
               break;

               case state_navigating_to_rendezvous:
                    // just wait until reaching the rendezvous footprint 
                    // position
               break;

               case state_at_rendezvous:
                    // broadcast the plan I'm trying to fulfill
                    rendezvous_msg.plan = planPtr->GetCurrentAgreementUniqueID();
                    plan_realization_pu.publish(rendezvous_msg);

                    // this is a hack, check a better way to do it
                    planPtr->RealizePlan(robot_id);

                    // This robot is going to wait until it receives all callback
                    // calls from others that can communicate and are trying to 
                    // fulfill the same plan
                    if(planPtr->WasPlanRealized() == true) {
                        if(planPtr->CheckConsensusCurrentPlan() == true) {
                            ChangeState(state_updating_plan);
                            ROS_INFO("[Explorer] I am consensus robot.");
                            time_waiting = 0.0;
                        } else {
                            ChangeState(state_waiting_consensus);
                            ROS_INFO("[Explorer] Waiting consensus.");
                            time_waiting = 0.0;
                        }
                    } else {
                        ROS_INFO("[Explorer] Waiting at rendezvous for %fs", time_waiting);

                        if(time_waiting > waiting_threshold) {
                            ChangeState(state_compute_centroids);
                            time_waiting = 0.0;
                            planPtr->ResetPlanRealization();
                            ROS_INFO("[Explorer] No one went to rendezvous, reseting this plan.");
                        }
                    }

                    time_waiting += delta_time;
               break;

               case state_updating_plan:
                    // ask for centroids to avoid
                    // unnecessary computations
                    fro_comp.publish(std_msgs::String());
                    ChangeState(state_waiting_centroids_for_plan);
                    ROS_INFO("[Explorer] Consensus asking for frontiers to update rendezvous location.");
               break;

               case state_waiting_centroids_for_plan:
                    // just wait
               break;

               case state_select_new_rendezvous:
                    // randomize plan location update
                    // to maximize utility spreading the rendezvous 
                    // as rational agents
                    if(CENTROIDS.centroids.poses.size() > 0)
                        RoulettFrontier(CENTROIDS, WORLD_POS, goal_frontier);

                    // aways send updated plan to unstuck 
                    // in situations where there are no more frontiers
                    // to explore
                    planPtr->UpdateCurrentAgreementLocation(goal_frontier);
                    planPtr->ResetPlanRealization();
                    planPtr->SetNextAgreement();

                    ROS_INFO("[Explorer] New location selected [%f %f], sending to others.",
                        goal_frontier.getX(), goal_frontier.getY());

                    // send the new location to the other robots and start
                    // exploring again
                    rendezvous_new_pose_msg.pose.position.x = goal_frontier.getX();
                    rendezvous_new_pose_msg.pose.position.y = goal_frontier.getY();
                    rendezvous_new_pose_msg.pose.position.z = WORLD_POS.getZ();
                    plan_location_updater.publish(rendezvous_new_pose_msg);

                    ChangeState(state_select_frontier);
               break;

               case state_waiting_consensus:                
                    // waiting for new rendezvous location from the
                    // consensus robot
                    if(received_new_rendezvous_location) {
                        planPtr->UpdateCurrentAgreementLocation(new_rendezvous_location);
                        planPtr->ResetPlanRealization();
                        planPtr->PrintCurrent();
                        ROS_INFO("[Explorer] received new rendezvous location [%f %f].",
                            new_rendezvous_location.getX(), 
                            new_rendezvous_location.getY());
                        planPtr->SetNextAgreement();
                        ROS_INFO("[Explorer] the new agreement to be fulfilled was updated to.");
                        planPtr->PrintCurrent();
                        ChangeState(state_compute_centroids);
                        received_new_rendezvous_location = false;
                    }
               break;
            }

            // update rendezvous plan only if started
            // the mission
            if(CURRENT_STATE != state_idle)
                planPtr->Update(delta_time);

            if(CURRENT_STATE < state_set_rendezvous_location) {
                planPtr->PrintCurrent();   
            }
        }

        // follow the rendezgous policy only if
        // has something to explore, otherwise,
        // let robots go back to base
        if(planPtr->ShouldFulfillAgreement() && 
            CURRENT_STATE < state_set_rendezvous_location &&
            CURRENT_STATE != state_idle &&
            FinishedMission() == false) {
            ChangeState(state_set_rendezvous_location);
        }

        delta_time = ros::Time::now().sec - last_time.sec;

        // run spin to get the data
        last_time = ros::Time::now();
        ros::spinOnce();
        loop_frequency.sleep();
    }

    return 0;
}