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

#include <string.h>
#include <iostream>
#include <signal.h>
#include <vector>
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int8MultiArray.h"
#include "Common.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "multirobotsimulations/CustomPose.h"
#include "multirobotsimulations/MockPackage.h"

using namespace std;

void fusemaps(nav_msgs::OccupancyGrid &rSelfOcc, 
                const nav_msgs::OccupancyGrid &rOtherOcc, 
                geometry_msgs::PoseArray* relative, 
                int i, 
                const bool& rReplace) {
    // transformation does not invert indexes
    int x, y;
    int8_t val;
    int8_t self_val;
    tf::Vector3 world_relative;
    tf::Vector3 world_origin;
    tf::Vector3 map_origin;
    tf::Vector3 map_relative;
    tf::Vector3 relative_transform;

    PoseToVector3(relative->poses[i], world_relative);
    WorldToMap(rSelfOcc, world_origin, map_origin);
    WorldToMap(rSelfOcc, world_relative, map_relative);
    relative_transform = map_relative - map_origin;

    for(size_t index = 0; index < rSelfOcc.data.size(); ++index) {
        x = index % rSelfOcc.info.width;
        y = index / rSelfOcc.info.width;
        
        int res_x = x + relative_transform.getX();
        int res_y = y + relative_transform.getY();

        int other_index = res_y * rSelfOcc.info.width + res_x;

        self_val = rSelfOcc.data[index];
        val = rOtherOcc.data[index];
        if(res_x > 0 && res_x < rSelfOcc.info.width && res_y > 0 && res_y < rSelfOcc.info.height) {
           if(rReplace == true) {
                if(val != -1) {
                    rSelfOcc.data[other_index] = val;
                }
            } else {
                if(self_val == -1) {
                    if(val != -1)  {
                        rSelfOcc.data[other_index] = val;
                    }
                }
            }
        }
    }
}


bool CheckData(bool* pReceivedMaps, bool pReceivedPoses, const int& rIndex) {
    if(pReceivedMaps[rIndex] != true ||
        pReceivedPoses != true) return false;
    return true;
}

int main(int argc, char* argv[]) {
    // init node and get node handle
    ros::init(argc, argv, "fbmapfusion");
    ros::NodeHandle nh;
    ros::NodeHandle private_handle("~");

    // come configurations
    int queue_size = 1;
    int rate = 1;
    int robots = 1;
    int id = -1;
    std::string ns = nh.getNamespace();

    // get configurations from parameters
    nh.getParam("/robots", robots);
    private_handle.getParam("id", id);
    private_handle.getParam("rate", rate);
    private_handle.getParam("queue_size", queue_size);
    
    std::string package_path = ros::package::getPath("multirobotexploration");
    ros::Rate hz(rate);
    
    // original maps and fusion
    nav_msgs::OccupancyGrid robots_occ[robots];
    nav_msgs::OccupancyGrid fusion;

    // starting poses and real world coordinates
    geometry_msgs::PoseArray robots_relative_poses;
    std_msgs::Float64MultiArray robots_relative_distances;

    // used for markers, program flow and etc
    int current_iteration = 0;
    bool received_obstacles[robots];
    bool received_maps[robots];
    bool received_poses = false;
    bool received_distances = false;

    // subs
    std::vector<ros::Subscriber> subs;
    memset(received_obstacles, 0, sizeof(bool)*robots);
    memset(received_maps, 0, sizeof(bool)*robots);

    // fusion advertiser
    ros::Publisher fus_pub = nh.advertise<nav_msgs::OccupancyGrid>(ns + "/fusion", queue_size);

    // 1 - get my start position
    // 2 - get others start position
    // 3 - compute relative poses array
    // 4 - store array the same way it was being created before
    bool *rec_maps = received_maps;
    nav_msgs::OccupancyGrid *rocc_temp = robots_occ;
    ROS_INFO("ROS %s", ns.c_str());
    subs.push_back(nh.subscribe<nav_msgs::OccupancyGrid>(ns + "/map", 
                queue_size, 
                [rocc_temp, rec_maps, id](nav_msgs::OccupancyGrid::ConstPtr rMsg){
                    // check if distance is less than a threshold in meters
                    // compute distance to robot to simulate communication radius
                    rocc_temp[id].data.assign(rMsg->data.begin(), rMsg->data.end());
                    rocc_temp[id].info = rMsg->info;
                    rocc_temp[id].header = rMsg->header;
                    rec_maps[id] = true;
                }));

    geometry_msgs::PoseArray* robots_relative_poses_temp = &robots_relative_poses;
    bool* received_poses_temp = &received_poses;
    subs.push_back(nh.subscribe<geometry_msgs::PoseArray>(
                ns + "/relative_pose_estimator/relative_start", 
                queue_size,
                [robots_relative_poses_temp, received_poses_temp](geometry_msgs::PoseArray::ConstPtr rMsg) {
                    robots_relative_poses_temp->header = rMsg->header;
                    robots_relative_poses_temp->poses.assign(rMsg->poses.begin(), rMsg->poses.end());
                    (*received_poses_temp) = true;
                }));

    std_msgs::Int8MultiArray robots_in_comm;
    robots_in_comm.data.assign(robots,0);
    std_msgs::Int8MultiArray* robots_in_comm_ptr = &robots_in_comm;
    subs.push_back(nh.subscribe<std_msgs::Int8MultiArray>(
                ns + "/mock_communication_model/robots_in_comm", 
                queue_size,
                [robots_in_comm_ptr](std_msgs::Int8MultiArray::ConstPtr rMsg) {
                    robots_in_comm_ptr->data.assign(rMsg->data.begin(), rMsg->data.end());
                }));

    /*
     * The following subscriber should be replaced with a network package handler
    */
    rocc_temp = robots_occ;
    rec_maps = received_maps;
    for(int i = 0; i < robots; ++i) {
        if(i == id) continue;
        subs.push_back(nh.subscribe<nav_msgs::OccupancyGrid>("/robot_" + std::to_string(i) + "/fusion", 
                    queue_size, 
                    [rocc_temp, rec_maps, robots_in_comm_ptr, i](nav_msgs::OccupancyGrid::ConstPtr rMsg){
                        if(robots_in_comm_ptr->data[i] == 0) return;
                        rocc_temp[i].data.assign(rMsg->data.begin(), rMsg->data.end());
                        rocc_temp[i].info = rMsg->info;
                        rocc_temp[i].header = rMsg->header;
                        rec_maps[i] = true;
                    }));
    }

    bool first = true;
    while(ros::ok()) {   
        if(CheckData(received_maps, received_poses, id)) {
            // all fusions are done here, all the relative poses 
            // should also be computed here
            fusion.info = robots_occ[id].info;
            fusion.header = robots_occ[id].header;

            if(first == true) {
                fusion.data.assign(robots_occ[id].data.size(), -1);
            } else {
                first = false;
            }

            // do the map transformation here
            // other adjustments can be made
            // but for the purpose of several experiments,
            // initial translation should be enough
            for(int i = 0; i < robots; ++i) {
                if(CheckData(received_maps, received_poses, i) && i != id) {
                    // just fuse the maps
                    fusemaps(fusion, robots_occ[i], &robots_relative_poses, i, false);
                    // ROS_INFO("%f %f", robots_relative_poses.poses[i].position.x, robots_relative_poses.poses[i].position.y);
                }
            }

            // copy my map on top of everyone
            fusemaps(fusion, robots_occ[id], &robots_relative_poses, id, true);

            // publish fusion, this robot's map will always be on top
            fus_pub.publish(fusion);
        }

        current_iteration += 1;
        ros::spinOnce();
        hz.sleep();
    }

    return 0;
}