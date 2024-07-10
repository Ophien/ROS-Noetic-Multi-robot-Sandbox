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

void CreateLocal(nav_msgs::OccupancyGrid& dynamicOcc, 
                nav_msgs::OccupancyGrid& localMap,
                costmap_converter::ObstacleArrayMsg& obsarraymsg,
                geometry_msgs::PoseArray& occupiedPoses,
                geometry_msgs::PoseArray& freePoses,
                tf::Vector3& worldPose,
                tf::Vector3& occPose,
                const double& freeInflationRadius,
                const double& occupiedInflationRadius, 
                const int& windws_size_meters,
                const int8_t& occupancyThreshold = 90,
                const int8_t& freeThreshold = 50,
                const int8_t& occupiedValue = 100,
                const int8_t& freeVal = 1,
                const int8_t& unknownVal = -1) {
    // this should be received as parameter
    obsarraymsg.obstacles.clear();

    // convert local view size here
    int size_in_pixels = (int)(windws_size_meters / dynamicOcc.info.resolution) + 1;
    int length = size_in_pixels + 1;
    int half_length = (int)(length/2.0);
    Vec2i start = Vec2i::Create(occPose.getX() - half_length, occPose.getY() - half_length);
    Vec2i end   = Vec2i::Create(occPose.getX() + half_length, occPose.getY() + half_length);
    int ix, iy, index;
    int irp_occu = (int)(occupiedInflationRadius / dynamicOcc.info.resolution);
    int irp_free = (int)(freeInflationRadius / dynamicOcc.info.resolution);

    // allocate or initialize data
    localMap.data.assign(length*length, -1);
    localMap.header = dynamicOcc.header;
    localMap.info = dynamicOcc.info;
    localMap.info.width = length;
    localMap.info.height = length;
    localMap.info.origin.position.x = worldPose.getX() - (localMap.info.width * dynamicOcc.info.resolution) / 2.0;
    localMap.info.origin.position.y = worldPose.getY() - (localMap.info.height * dynamicOcc.info.resolution) / 2.0;

    nav_msgs::OccupancyGrid free;
    nav_msgs::OccupancyGrid occu;
    free.data.assign(length*length, -1);
    occu.data.assign(length*length, -1);
    obsarraymsg.header = dynamicOcc.header;

    // copy local
    int id = 0;
    for(int y = 0; y < length; ++y) {
        iy = y + start.y;
        for(int x = 0; x < length; ++x) {
            ix = x + start.x;
            index = iy * dynamicOcc.info.width + ix;

            // free
            if(dynamicOcc.data[index] >= 0 && dynamicOcc.data[index] <= 50)
                ApplyMask(x,y,irp_free, free.data, freeVal, length, length);
            if(dynamicOcc.data[index] > occupancyThreshold) {
                ApplyMask(x,y,irp_occu, occu.data, occupiedValue, length, length);
                id += 1;
            }
        }
    }

    // inflate and check obstacles polygons' positions
    // get valid obstacles and free pos
    occupiedPoses.poses.clear();
    freePoses.poses.clear();
    ApplyMask(half_length,half_length, 3, free.data, freeVal, length, length);
    localMap.data.assign(free.data.begin(), free.data.end());

    for(int y = 0; y < length; ++y) {
        iy = y + start.y;
        for(int x = 0; x < length; ++x) {
            ix = x + start.x;
            int index = y*length+x;
            if(occu.data[index] > occupancyThreshold) {
                localMap.data[index] = occupiedValue;
                
                // set and obstacle in the obstacle msg
                costmap_converter::ObstacleMsg obs_msg;
                obs_msg.header = dynamicOcc.header;
                obs_msg.id = id;
                Vec2i cur = Vec2i::Create(ix,iy);
                tf::Vector3 world;
                MapToWorld(dynamicOcc, cur, world);
                geometry_msgs::Point32 pmsg;
                pmsg.x = world.getX();
                pmsg.y = world.getY();
                pmsg.z = 0.0;
                obs_msg.polygon.points.push_back(pmsg);
                obs_msg.radius = 0.0;
                obs_msg.velocities = geometry_msgs::TwistWithCovariance();
                obsarraymsg.obstacles.push_back(obs_msg);

                // set pose array message wih the obstacle's position
                geometry_msgs::Pose p; p.position.x = x; p.position.y = y;
                occupiedPoses.poses.push_back(p);
            } else if (localMap.data[index] >= 0 && localMap.data[index] < occupiedValue) {
                geometry_msgs::Pose p; p.position.x = x; p.position.y = y;
                freePoses.poses.push_back(p);
            }
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

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "localdynamicmap");
    ros::NodeHandle node_handle;
    std::string ns = ros::this_node::getNamespace();

    int queue_size = 1;
    int rate = 10;
    int id = -1;

    double laser_range = 10.0;
    double free_inflate_radius = 0.3;
    double occu_inflate_radius = 0.5;
    int lidar_sources_count = 1;
    int local_view_size = 0;

    node_handle.getParam(ns+"/id", id);
    node_handle.getParam(ns+"/rate_ldm", rate);
    node_handle.getParam(ns+"/ldm_queue_size", queue_size);
    node_handle.getParam(ns+"/ldm_max_lidar_range", laser_range);
    node_handle.getParam(ns+"/ldm_free_inflation_radius", free_inflate_radius);
    node_handle.getParam(ns+"/ldm_ocu_inflation_radius", occu_inflate_radius);
    node_handle.getParam(ns+"/ldm_lidar_sources", lidar_sources_count);
    node_handle.getParam(ns+"/ldm_local_view_size", local_view_size);

    // control variables
    ros::Rate loop_hz(rate);
    nav_msgs::OccupancyGrid occ;
    nav_msgs::OccupancyGrid with_dynamic_data;
    tf::Vector3 occ_pose;
    tf::Vector3 world_pose;
    multirobotsimulations::CustomPose world_pose_msg;

    bool has_occ = false;
    bool has_pose = false;

    // messages to publish
    nav_msgs::OccupancyGrid local_cspace;
    costmap_converter::ObstacleArrayMsg obsarraymsg;
    geometry_msgs::PoseArray occupiedPoses;
    geometry_msgs::PoseArray freePoses;
   
    // subscriptions and advertisers
    nav_msgs::OccupancyGrid* occ_ptr = &occ;
    bool* has_occ_ptr = &has_occ;
    ros::Subscriber occ_sub = node_handle.subscribe<nav_msgs::OccupancyGrid>(ns + "/map", queue_size, 
        [occ_ptr,
        has_occ_ptr](nav_msgs::OccupancyGrid::ConstPtr msg){
            if((*has_occ_ptr) == false) (*has_occ_ptr) = true;
            occ_ptr->data.assign(msg->data.begin(), msg->data.end());
            occ_ptr->info = msg->info;
            occ_ptr->header = msg->header;
        }
    );

    multirobotsimulations::CustomPose* world_pose_msg_ptr = &world_pose_msg;
    tf::Vector3* world_pose_ptr = &world_pose;
    tf::Vector3* occ_pose_ptr = &occ_pose;
    bool* has_pose_ptr = &has_pose;
    ros::Subscriber world_pose_sub = node_handle.subscribe<multirobotsimulations::CustomPose>(ns + "/gmapping_pose/world_pose", queue_size, 
        [world_pose_msg_ptr,world_pose_ptr,occ_pose_ptr,occ_ptr,has_occ_ptr,has_pose_ptr](multirobotsimulations::CustomPose::ConstPtr msg){
            if((*has_occ_ptr) == false) return;
            if((*has_pose_ptr) == false)(*has_pose_ptr) = true;
            world_pose_msg_ptr->robot_id = msg->robot_id;
            world_pose_msg_ptr->pose.position = msg->pose.position;
            world_pose_msg_ptr->pose.orientation = msg->pose.orientation;

            // compute pose in occ
            WorldToMap(*occ_ptr, world_pose_msg_ptr->pose, (*occ_pose_ptr));
            world_pose_ptr->setX(msg->pose.position.x);
            world_pose_ptr->setY(msg->pose.position.y);
        }
    );

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

    ros::Publisher lob = node_handle.advertise<costmap_converter::ObstacleArrayMsg>(ns + "/obstacle_cells", queue_size);
    ros::Publisher csl = node_handle.advertise<nav_msgs::OccupancyGrid>(ns + "/c_space_local", queue_size);
    ros::Publisher lop = node_handle.advertise<geometry_msgs::PoseArray>(ns + "/local_occupied_poses", queue_size);
    ros::Publisher lfp = node_handle.advertise<geometry_msgs::PoseArray>(ns + "/local_free_poses", queue_size);

    while(ros::ok()) {
        if(has_occ && has_pose) {
            ApplyDynamicData(occ, with_dynamic_data, lidar_sources);
            CreateLocal(with_dynamic_data, 
                        local_cspace, 
                        obsarraymsg, 
                        occupiedPoses, 
                        freePoses, 
                        world_pose,
                        occ_pose,
                        free_inflate_radius, 
                        occu_inflate_radius, 
                        local_view_size);
            lob.publish(obsarraymsg);
            csl.publish(local_cspace);
            lop.publish(occupiedPoses);
            lfp.publish(freePoses);
        }

        loop_hz.sleep();
        ros::spinOnce();
    }
}