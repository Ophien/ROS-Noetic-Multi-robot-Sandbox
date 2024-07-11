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

#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "tf2_ros/transform_listener.h"
#include "tf/tf.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "multirobotsimulations/CustomPose.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include <stdio.h>
#include <string.h>
#include "nav_msgs/Path.h"

using namespace std;

bool HAS_OCC = false;
nav_msgs::MapMetaData OCC_METADATA;

void OccupancyGridCallback(const nav_msgs::OccupancyGrid& rMsg) {
    OCC_METADATA = rMsg.info;
    HAS_OCC = true;
}

int main(int argc, char* argv[]) {
    /*
     * Initialize node with its name
     */
    ros::init(argc, argv, "gmapping_pose");

    /*
     * Get node handle to handle publisher and subscribers
     */
    ros::NodeHandle node_handle;
    ros::NodeHandle private_handle("~");
    std::string ns = ros::this_node::getNamespace();

    /*
     * Define loop frequency and publishing frequency in hertz (publishes per second)
     */
    int queue_size = 1;
    int loop_frequency = 10;
    int robot_id = -1;

    private_handle.getParam("id", robot_id);
    private_handle.getParam("rate", loop_frequency);
    private_handle.getParam("queue_size", queue_size);

    /*
     * TransformListener is used to listen to all transforms into the 
     * ROS transform stream
     */
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    /*
     * Set topic to advertise gmapping pose
     */
    std::string ns_clean = ns;
    ns_clean.erase(remove(ns_clean.begin(), ns_clean.end(), '/'), ns_clean.end());
    ROS_INFO("[%s gmapping_pose] lookup transform name: %s from: %s", ns.c_str(), ns_clean.c_str(), ns.c_str());

    std::string tf_map = "";
    std::string tf_base_link = "";

    if(ns_clean.size() == 0) {
        tf_map = "map";
        tf_base_link = "base_link";
    } else {
        tf_map = ns_clean + "/map";
        tf_base_link = ns_clean +  "/base_link"; 
    }

    ROS_INFO("[%s gmapping_pose] looking for tf: %s", ns.c_str(), tf_map.c_str());

    /*
     * Outer nodes topics
     */
    ros::Subscriber occupancy_grd_sub = node_handle.subscribe(ns + "/map", queue_size, &OccupancyGridCallback);

    /*
     * This node's topics
     */    
    ros::Publisher pose_pub = node_handle.advertise<multirobotsimulations::CustomPose>(ns + "/gmapping_pose/world_pose", queue_size);
    ros::Publisher pose_stamped_pub = node_handle.advertise<multirobotsimulations::CustomPose>(ns + "/gmapping_pose/pose_stamped", queue_size);
    ros::Publisher path_pub = node_handle.advertise<nav_msgs::Path>(ns + "/path", queue_size);

    // world and occ coordinates
    double x, y, z, ox, oy, oz;
    geometry_msgs::Pose pose;
    multirobotsimulations::CustomPose cpose;

    nav_msgs::Path path;

    ros::Rate hz(loop_frequency);
    while(ros::ok()) {
        /*
         *  get current estimated pose from gmapping
         */
        geometry_msgs::TransformStamped map_base_link;
        try{
            /*
             * Transform lookup do the transformation between the map reference frame 
             * and the base_link reference frame
             */
            map_base_link = tfBuffer.lookupTransform(tf_map, tf_base_link, ros::Time(0));

            /*
             * Process captured transform
             */
            x = map_base_link.transform.translation.x;
            y = map_base_link.transform.translation.y;
            z = map_base_link.transform.translation.z;

            geometry_msgs::PoseStamped p;
            p.header = map_base_link.header;
            p.pose.position.x = x;
            p.pose.position.y = y;
            p.pose.position.z = z;
            pose.orientation = map_base_link.transform.rotation;
            pose_stamped_pub.publish(p);

            path.header = map_base_link.header;
            path.poses.push_back(p);
            path_pub.publish(path);

            /*
             * Prepare pose to be published with captured transformation data
             */
            pose.position.x = x;
            pose.position.y = y;
            pose.position.z = z;
            pose.orientation = map_base_link.transform.rotation;
            cpose.pose = pose;
            cpose.robot_id = robot_id;
            pose_pub.publish(cpose);
        } catch(tf2::TransformException &ex) {
            ROS_INFO("[%s gmapping_pose] %s", ns.c_str(), ex.what());
        }

        ros::spinOnce();
        hz.sleep();
    }

    return 0;
}