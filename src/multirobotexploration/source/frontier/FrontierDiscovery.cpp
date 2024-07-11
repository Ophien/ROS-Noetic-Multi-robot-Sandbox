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

#include <vector>
#include <iostream>
#include <thread>
#include <mutex>
#include "ros/ros.h"
#include "ros/package.h"
#include "tf/tf.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include "SearchAlgorithms.h"
#include "multirobotsimulations/CustomPose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseArray.h"
#include "Common.h"
#include "multirobotsimulations/Frontiers.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"

using namespace std;

typedef enum{
    IDLE = 0,
    PROCESSING = 1,
    FINISHED = 2
}FrontierState;

int SEQ;
int ID;
bool RECEIVED_CS;
bool HAS_POSE;
double YAW;
FrontierState STATE;
Vec2i POS;
geometry_msgs::Pose WORLD_POS;
nav_msgs::OccupancyGrid OCC;
std::mutex MAP_MUT;

void CSpaceCallback(const nav_msgs::OccupancyGrid& rMsg) {
    OCC.data.assign(rMsg.data.begin(), rMsg.data.end());
    OCC.info = rMsg.info;
    OCC.header = rMsg.header;
    RECEIVED_CS = true;
}

void EstimatePoseCallback(const multirobotsimulations::CustomPose& rMsg) {
    WORLD_POS.position = rMsg.pose.position;
    WORLD_POS.orientation = rMsg.pose.orientation;
    YAW = tf::getYaw(rMsg.pose.orientation);
    HAS_POSE = true;
}

void ComputeCallback(const std_msgs::String& rMsg) {
    STATE = FrontierState::PROCESSING;
}

void CreateMarker(visualization_msgs::Marker& rInput, const char* pNs, const int& rId, const int& rSeq) {
    rInput.id = rId;
    rInput.header.frame_id = "robot_" + std::to_string(rId) + std::string("/map");
    rInput.header.stamp = ros::Time().now();
    rInput.ns = pNs;
    rInput.points.clear();
    rInput.type = visualization_msgs::Marker::CUBE_LIST;
    rInput.action = visualization_msgs::Marker::MODIFY;
    rInput.pose.orientation.x = 0.0;
    rInput.pose.orientation.y = 0.0;
    rInput.pose.orientation.z = 0.0;
    rInput.pose.orientation.w = 1.0;
    rInput.scale.x = 0.25;
    rInput.scale.y = 0.25;
    rInput.scale.z = 0.5;
    rInput.color.a = 1.0;
    rInput.color.r = 0.0;
    rInput.color.g = 0.3;
    rInput.color.b = 1.0;
    rInput.lifetime = ros::Duration(60);
}

void SetPoseArr(geometry_msgs::PoseArray& rArr, const int& rSeq) {
    rArr.poses.clear();
    rArr.header.frame_id = std::string("robot_") + std::to_string(ID) + std::string("/map");
    rArr.header.seq = rSeq;
    rArr.header.stamp = ros::Time::now();
}

int main(int argc, char* argv[]) {
    std::string node_name = "frontier_discovery";
    ros::init(argc, argv, node_name);
    ros::NodeHandle node_handle;
    ros::NodeHandle private_handle("~");
    std::string ns = ros::this_node::getNamespace();

    int queue_size = 1;
    int rate = 1;
    int id = -1;
    double max_lidar_range = 10.0;

    private_handle.getParam("id", id);
    private_handle.getParam("rate", rate);
    private_handle.getParam("queue_size", queue_size);
    private_handle.getParam("max_lidar_range", max_lidar_range);

    // set global ID
    ID = id;

    ros::Rate loop_rate(rate);
    std::string path = ros::package::getPath("multirobotexploration");

    RECEIVED_CS = false;
    HAS_POSE = false;
    SEQ = 0;

    ros::Subscriber csp = node_handle.subscribe(ns + "/c_space", queue_size, &CSpaceCallback);
    ros::Subscriber pos = node_handle.subscribe(ns + "/gmapping_pose/world_pose", queue_size, &EstimatePoseCallback);  
    ros::Subscriber fcc = node_handle.subscribe(ns + "/frontier_discovery/compute", queue_size, &ComputeCallback);
    ros::Publisher  cap = node_handle.advertise<visualization_msgs::Marker>(ns + "/frontier_discovery/frontiers_clusters_markers", queue_size);
    ros::Publisher  fro = node_handle.advertise<nav_msgs::OccupancyGrid>(ns + "/frontier_discovery/frontiers", queue_size);
    ros::Publisher  poa = node_handle.advertise<multirobotsimulations::Frontiers>(ns + "/frontier_discovery/frontiers_clusters", queue_size);

    std::vector<Vec2i> frontiers;
    std::vector<Vec2i> centroids;
    std::vector<Vec2i> filtered_centroids;
    std::vector<std::vector<Vec2i>> fclusters;
    std::vector<std::vector<Vec2i>> filtered_clusters;
    std::list<Vec2i> ppath;
    tf::Vector3 temp_world;
    
    visualization_msgs::Marker cluster_marker_msg;
    geometry_msgs::PoseArray pose_arr_msg;
    nav_msgs::OccupancyGrid frontiers_map;
    
    multirobotsimulations::Frontiers frontiers_msg;

    STATE = FrontierState::IDLE;

    double vis_rad = max_lidar_range / 2.0;
    double cost = -1;
    double utility_heuristic = 2.0 * M_PI * vis_rad * vis_rad;
    double gain = -1;
    int cluster_detection_min = 15;
    int highest = -1;
    int lowest = std::numeric_limits<int>::max();
    int highest_index = -1;
    int lowest_index = -1;

    while(ros::ok()) {
        if(RECEIVED_CS == true && HAS_POSE == true) {
            switch(STATE) {
                case IDLE:
                    // do nothing
                break;
                case PROCESSING:
                    WorldToMap(OCC, WORLD_POS, POS);

                    // compute frontiers from c_space
                    sa::ComputeFrontiers(OCC, frontiers_map, frontiers);
                    sa::ComputeClusters(frontiers_map, frontiers, fclusters);
                    // sa::ComputeCentroids(POS, fclusters, centroids);
                    filtered_clusters.clear();
                    for(auto& cluster : fclusters) {
                        if(cluster.size() > cluster_detection_min) filtered_clusters.push_back(cluster);
                    }
                    sa::ComputeAverageCentroids(POS, filtered_clusters, centroids);

                    // filter reachable
                    filtered_centroids.clear();
                    frontiers_msg.centroids.poses.clear();
                    frontiers_msg.costs.data.clear();
                    frontiers_msg.utilities.data.clear();
                    frontiers_msg.gains.data.clear();

                    for(size_t i = 0; i < centroids.size(); ++i) {
                        sa::ComputePathWavefront(OCC, POS, centroids[i], ppath);

                        // compute convex hull of the frontiers
                        if(ppath.size() > 0) {
                            cost = (double)ppath.size();
                            gain = utility_heuristic / cost;

                            filtered_centroids.push_back(centroids[i]);
                            frontiers_msg.costs.data.push_back(cost);
                            frontiers_msg.utilities.data.push_back(utility_heuristic);
                            frontiers_msg.gains.data.push_back(gain);

                            if(gain > highest) {
                                highest = gain;
                                highest_index = i;
                            }
                            if(gain < lowest) {
                                lowest = gain;
                                lowest_index = i;
                            }
                        }
                    }

                    // publish the found frontiers centroids into the network
                    if(filtered_centroids.size() > 0) {
                        frontiers_msg.highest_index = highest_index;
                        frontiers_msg.lowest_index = lowest_index;

                        SetPoseArr(pose_arr_msg, SEQ);
                        CreateMarker(cluster_marker_msg, ns.c_str(), id, SEQ);

                        ROS_INFO("[FrontierDiscovery] %ld available frontiers.", filtered_centroids.size());
                        for(size_t i = 0; i < filtered_centroids.size(); ++i) {                            
                            MapToWorld(OCC, filtered_centroids[i], temp_world);
                            geometry_msgs::Point p;
                            geometry_msgs::Pose po;
                            p.z = 0.25;
                            p.x = temp_world.getX();
                            p.y = temp_world.getY();
                            po.position.x = temp_world.getX();
                            po.position.y = temp_world.getY();
                            cluster_marker_msg.points.push_back(p);
                            pose_arr_msg.poses.push_back(po);

                            ROS_INFO("\t[%.2f %.2f] - cost: %.2f utility: %.2f gain: %.2f", 
                                temp_world.getX(),
                                temp_world.getY(),
                                frontiers_msg.costs.data[i], 
                                frontiers_msg.utilities.data[i], 
                                frontiers_msg.gains.data[i]);
                        }
                        frontiers_msg.centroids = pose_arr_msg;
                        cap.publish(cluster_marker_msg);
                        poa.publish(frontiers_msg);
                        fro.publish(frontiers_map);
                        SEQ += 1;
                    } else {
                        poa.publish(frontiers_msg);
                    }
                    STATE = FrontierState::IDLE;
                break;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}