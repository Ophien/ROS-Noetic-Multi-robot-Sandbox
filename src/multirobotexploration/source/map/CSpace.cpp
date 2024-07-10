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

#include <string>
#include <ros/ros.h>
#include <signal.h>
#include "multirobotsimulations/CustomOcc.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "costmap_converter/ObstacleArrayMsg.h"
#include "multirobotsimulations/CustomPose.h"
#include "tf/tf.h"
#include "string.h"
#include "Common.h"

void Inflate(nav_msgs::OccupancyGrid& occ,
            nav_msgs::OccupancyGrid& free,
            nav_msgs::OccupancyGrid& occupied, 
            const double& freeInflationRadius,
            const double& occupiedInflationRadius, 
            const int8_t& occupancyThreshold = 90,
            const int8_t& freeThreshold = 50,
            const int8_t& occupiedValue = 100,
            const int8_t& freeVal = 1) {
    free.data.assign(occ.data.begin(), occ.data.end());
    free.header = occ.header;
    free.info = occ.info;

    occupied.data.assign(occ.data.begin(), occ.data.end());
    occupied.header = occ.header;
    occupied.info = occ.info;

    int index;
    int8_t val;
    int8_t raw_val;
    int width = occ.info.width;
    int height = occ.info.height;
    int irp_occu = (int)(occupiedInflationRadius / occ.info.resolution);
    int irp_free = (int)(freeInflationRadius / occ.info.resolution);
    for(int y = 0; y < occ.info.height; ++y) {
        for(int x = 0; x < occ.info.width; ++x) {
            index = y * width + x;
            val = occ.data[index];
            if(val > occupancyThreshold)
                ApplyMask(x, y, irp_occu, occupied.data, occupiedValue, width, height);
            else if(val >= 0 && val < freeThreshold)
                ApplyMask(x, y, irp_free, free.data, freeVal, width, height);
        }
    }
}

void ApplyDynamicData(nav_msgs::OccupancyGrid& occ,
                                         nav_msgs::OccupancyGrid& dynamicOcc,
                                         std::vector<geometry_msgs::PoseArray>& lidarSources,
                                         const double& maxLidarRange = 10.0,
                                         const int8_t& occupiedValue = 100) {
    dynamicOcc.data.assign(occ.data.begin(), occ.data.end());
    dynamicOcc.header = occ.header;
    dynamicOcc.info = occ.info;
    double range;
    int width = occ.info.width;
    Vec2i pos;

    // enhance with peripheral lidars
    for(size_t source = 0; source < lidarSources.size(); ++source) {
        for(size_t i = 0; i < lidarSources[source].poses.size(); ++i) {
            pos.x = (int)(lidarSources[source].poses[i].position.x);
            pos.y = (int)(lidarSources[source].poses[i].position.y);
            range = lidarSources[source].poses[i].position.z;
            if(range <= maxLidarRange & range >= 0.01) { 
                dynamicOcc.data[pos.y*width+pos.x] = occupiedValue;
            }
        }
    }
}

void GenerateCSpace(nav_msgs::OccupancyGrid& free,
                    nav_msgs::OccupancyGrid& occupied,
                    nav_msgs::OccupancyGrid& cspace,
                    tf::Vector3& occ_pose,
                    const int8_t& unknownVal = -1) {
    // copy data for local grids
    cspace.data.assign(occupied.data.begin(), occupied.data.end());
    cspace.header = occupied.header;
    cspace.info = occupied.info;

    int occ_val;
    int fre_val;
    int x = occ_pose.getX();
    int y = occ_pose.getY();
    
    for(size_t i = 0; i < cspace.data.size(); ++i) {
        occ_val = occupied.data[i];
        fre_val = free.data[i];

        // should cpy the filtered free values and then the occupied ones
        // this creates a map without noise
        if(fre_val != unknownVal) cspace.data[i] = fre_val;
        if(occ_val != unknownVal) cspace.data[i] = occ_val;
    }
}

void InflatePoseForPlanner(nav_msgs::OccupancyGrid& cspace,
                           const double& freeInflationRadius,
                           const int& x, 
                           const int& y,
                           const int8_t& occupancyThreshold = 90,
                           const int8_t& freeVal = 1) {

    int irp_free = (int)(freeInflationRadius / cspace.info.resolution);
    ApplyMask(x,y,irp_free,cspace.data,freeVal,cspace.info.width,cspace.info.height,occupancyThreshold,false);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "cspace");
    ros::NodeHandle node_handle;
    ros::NodeHandle private_handle("~");
    std::string ns = ros::this_node::getNamespace();

    int queue_size = 1;
    int rate = 10;
    int id = -1;
    int lidar_sources_count = 1;

    double laser_range = 10.0;
    double free_inflate_radius = 0.3;
    double occu_inflate_radius = 0.5;

    private_handle.getParam("id", id);
    private_handle.getParam("rate", rate);
    private_handle.getParam("queue_size", queue_size);
    private_handle.getParam("max_lidar_range", laser_range);
    private_handle.getParam("free_inflation_radius", free_inflate_radius);
    private_handle.getParam("ocu_inflation_radius", occu_inflate_radius);
    private_handle.getParam("lidar_sources", lidar_sources_count);

    ros::Rate loop_hz(rate);

    nav_msgs::OccupancyGrid occ;
    nav_msgs::OccupancyGrid free;
    nav_msgs::OccupancyGrid occupied;
    nav_msgs::OccupancyGrid cspace;
    nav_msgs::OccupancyGrid with_dynamic_data;
    multirobotsimulations::CustomPose world_pose;
    tf::Vector3 occ_pose;
    bool has_occ = false;
    bool has_pose = false;

    nav_msgs::OccupancyGrid* occ_ptr = &occ;
    bool* has_occ_ptr = &has_occ;
    ros::Subscriber occ_sub = node_handle.subscribe<nav_msgs::OccupancyGrid>(ns + "/map", queue_size, 
        [occ_ptr,
        has_occ_ptr](nav_msgs::OccupancyGrid::ConstPtr rMsg){
            if((*has_occ_ptr) == false) (*has_occ_ptr) = true;
            occ_ptr->data.assign(rMsg->data.begin(), rMsg->data.end());
            occ_ptr->info = rMsg->info;
            occ_ptr->header = rMsg->header;
        }
    );

    multirobotsimulations::CustomPose* world_pose_ptr = &world_pose;
    tf::Vector3* occ_pose_ptr = &occ_pose;
    bool* has_pose_ptr = &has_pose;
    ros::Subscriber world_pose_sub = node_handle.subscribe<multirobotsimulations::CustomPose>(ns + "/gmapping_pose/world_pose", queue_size, 
        [world_pose_ptr,occ_pose_ptr,occ_ptr,has_occ_ptr,has_pose_ptr](multirobotsimulations::CustomPose::ConstPtr rMsg){
            if((*has_occ_ptr) == false) return;
            if((*has_pose_ptr) == false)(*has_pose_ptr) = true;
            world_pose_ptr->robot_id = rMsg->robot_id;
            world_pose_ptr->pose.position = rMsg->pose.position;
            world_pose_ptr->pose.orientation = rMsg->pose.orientation;

            // compute pose in occ
            WorldToMap(*occ_ptr, world_pose_ptr->pose, (*occ_pose_ptr));
        }
    );

    ros::Publisher cspace_pub = node_handle.advertise<nav_msgs::OccupancyGrid>(ns + "/c_space", queue_size);

    // subscribe to all lidar sources
    std::vector<ros::Subscriber> lidar_subs;
    std::vector<geometry_msgs::PoseArray> lidar_sources;
    std::vector<geometry_msgs::PoseArray>* lidar_sources_ptr = &lidar_sources;
    for(int i = 0; i < lidar_sources_count; ++i) lidar_sources.push_back(geometry_msgs::PoseArray());
    for(int i = 0; i < lidar_sources_count; ++i) {
        lidar_subs.push_back(node_handle.subscribe<geometry_msgs::PoseArray>(ns + "/laser_to_world/lidar_occ_" + std::to_string(i), queue_size,
            [lidar_sources_ptr,i](geometry_msgs::PoseArray::ConstPtr msg){
                lidar_sources_ptr->at(i).header = msg->header;
                lidar_sources_ptr->at(i).poses.assign(msg->poses.begin(), msg->poses.end());
            }
        ));
    }

    while(ros::ok()) {
        if(has_occ && has_pose) {
            ApplyDynamicData(occ, with_dynamic_data, lidar_sources);

            // create c_space at the secified rate
            Inflate(with_dynamic_data, free, occupied, free_inflate_radius, occu_inflate_radius);
            GenerateCSpace(free, occupied, cspace, occ_pose);
            InflatePoseForPlanner(cspace, free_inflate_radius, occ_pose.getX(), occ_pose.getY());

            // publish the occ
            cspace_pub.publish(cspace);
        }

        loop_hz.sleep();
        ros::spinOnce();
    }
}