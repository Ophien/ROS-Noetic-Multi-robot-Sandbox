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
#include <vector>
#include "ros/ros.h"
#include "tf/tf.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Int8MultiArray.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "multirobotsimulations/CustomPose.h"
#include "Common.h"
#include "multirobotsimulations/MockPackage.h"

void PrepareMarkers(visualization_msgs::Marker& rInput, const char* pNs, const int& rId, const int& rSeq) {
    rInput.id = rId;
    rInput.header.frame_id = std::string("robot_") + std::to_string(rId) + std::string("/map");
    rInput.header.stamp = ros::Time().now();
    rInput.ns = pNs;
    rInput.points.clear();
    rInput.type = visualization_msgs::Marker::CUBE_LIST;
    rInput.action = visualization_msgs::Marker::MODIFY;
    rInput.pose.orientation.x = 0.0;
    rInput.pose.orientation.y = 0.0;
    rInput.pose.orientation.z = 0.0;
    rInput.pose.orientation.w = 1.0;
    rInput.scale.x = 0.5;
    rInput.scale.y = 0.5;
    rInput.scale.z = 0.5;
    rInput.color.a = 1.0;
    rInput.color.r = 1.0;
    rInput.color.g = 0.3;
    rInput.color.b = 0.1;
    rInput.lifetime = ros::Duration(60);
}

void SetNear(visualization_msgs::Marker& rInput) {
    rInput.color.a = 1.0;
    rInput.color.r = 0.0;
    rInput.color.g = 1.0;
    rInput.color.b = 0.1;
}

void SetFar(visualization_msgs::Marker& rInput) {
    rInput.color.a = 1.0;
    rInput.color.r = 1.0;
    rInput.color.g = 0.0;
    rInput.color.b = 0.1;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "relative_pose_estimator");
    ros::NodeHandle handle;
    ros::NodeHandle private_handle("~");

    int robots = 1;
    int id = -1;
    int rate = 5;
    int queue_size = 1;
    std::string ns = handle.getNamespace();

    handle.getParam("/robots", robots);
    private_handle.getParam("id", id);
    private_handle.getParam("queue_size", queue_size);
    private_handle.getParam("rate", rate);

    if(robots <= 0) robots = 1;
    
    // also send the starting poses of all robots
    geometry_msgs::PoseArray robots_relative_starting_positions;
    robots_relative_starting_positions.poses.assign(robots, geometry_msgs::Pose());

    // read relative start poses parameters
    std::vector<std::map<std::string, double>> poses;
    std::string key = "";
    for (int i = 0; i < robots; ++i) {
        key = "/start_pose_robot_" + std::to_string(i);
        std::map<std::string, double> pose;
        handle.getParam(key, pose);
        poses.push_back(pose);
        ROS_INFO("[%s relative_pose_estimator]: %s: %f %f %f", ns.c_str(), key.c_str(), pose["x"], pose["y"], pose["z"]);
    }

    // compute relative poses from file
    std::vector<tf::Vector3> relative(robots, tf::Vector3(0.0,0.0,0.0));
    tf::Vector3 my_pose(poses[id]["x"], poses[id]["y"], poses[id]["z"]);
    for(unsigned int i = 0; i < poses.size(); ++i) {
        if(i!=id) {
            tf::Vector3 other_pose(poses[i]["x"], poses[i]["y"], poses[i]["z"]);
            tf::Vector3 dir = other_pose - my_pose;
            relative[i] = dir;
            Vector3ToPose(dir, robots_relative_starting_positions.poses[i]);
            ROS_INFO("[%s relative_pose_estimator] relative to %d: %f %f %f", ns.c_str(), i, dir.getX(), dir.getY(), dir.getZ());
        } else {
            ROS_INFO("[%s relative_pose_estimator] relative to self: %f %f %f", ns.c_str(), relative[i].getX(), relative[i].getY(), relative[i].getZ());
        }
    }

    std::vector<ros::Subscriber> subs;
    ros::Rate sleep(rate);

    bool received_poses[robots];
    memset(received_poses, 0, sizeof(bool)*robots);

    /*
     * The following subscriber should be replaced with a network package handler
    */
    geometry_msgs::Pose robots_world_poses[robots];
    bool* temp_poses = received_poses;
    geometry_msgs::Pose* robots_world_poses_temp = robots_world_poses;
    for(int i = 0; i < robots; ++i) {
        subs.push_back(handle.subscribe<multirobotsimulations::CustomPose>("/robot_" + std::to_string(i) +"/gmapping_pose/world_pose", queue_size, 
            [temp_poses, robots_world_poses_temp, i](multirobotsimulations::CustomPose::ConstPtr rMsg) {
                robots_world_poses_temp[i].position = rMsg->pose.position;
                robots_world_poses_temp[i].orientation = rMsg->pose.orientation;
                temp_poses[i] = true;
            }));
    }

    std::vector<int> robots_in_comm(robots, 0);
    std::vector<int>* robots_in_comm_ptr = &robots_in_comm;
    ros::Subscriber mock_comm_sub = handle.subscribe<std_msgs::Int8MultiArray>(ns + "/mock_communication_model/robots_in_comm", queue_size,
        [robots_in_comm_ptr](std_msgs::Int8MultiArray::ConstPtr rMsg){
            robots_in_comm_ptr->assign(rMsg->data.begin(), rMsg->data.end());
        }
    );

    ros::Publisher star_sender = handle.advertise<geometry_msgs::PoseArray>(ns + "/relative_pose_estimator/relative_start", queue_size);
    ros::Publisher pose_sender = handle.advertise<geometry_msgs::PoseArray>(ns + "/relative_pose_estimator/relative_poses", queue_size);
    ros::Publisher dist_sender = handle.advertise<std_msgs::Float64MultiArray>(ns + "/relative_pose_estimator/distances", queue_size);
    ros::Publisher mark_sender = handle.advertise<visualization_msgs::Marker>(ns + "/relative_pose_estimator/pose_markers", queue_size);
    ros::Publisher marf_sender = handle.advertise<visualization_msgs::Marker>(ns + "/relative_pose_estimator/pose_far_markers", queue_size);

    // the message to be sent should be global and updated
    // when having the opportunity to do so
    geometry_msgs::PoseArray robots_relative_poses;
    robots_relative_poses.poses.assign(robots, geometry_msgs::Pose());

    // also store the distances to facilitate the work of other nodes
    std_msgs::Float64MultiArray robots_relative_distances;
    robots_relative_distances.data.assign(robots, 0.0);

    // use cluster marker for visuualization purposes
    visualization_msgs::Marker cluster_marker_msg;
    visualization_msgs::Marker cluster_marker_far_msg;

    // iteration sequence
    long long unsigned int seq = 0;

    while(ros::ok()) {
        // create a new cluster marker msg to be sent
        PrepareMarkers(cluster_marker_msg, ns.c_str(), id, seq);
        SetNear(cluster_marker_msg);

        PrepareMarkers(cluster_marker_far_msg, ns.c_str(), id, seq);
        SetFar(cluster_marker_far_msg);

        // translate all the poses here
        tf::Vector3 relative_pose;
        tf::Vector3 mine;
        PoseToVector3(robots_world_poses[id], mine);
        double distance = 0.0;
        for(int i = 0; i < robots; ++i) {
            if(i == id) continue;
            if(received_poses[i] == true) {
                PoseToVector3(robots_world_poses[i], relative_pose); 
                relative_pose += relative[i];

                geometry_msgs::Point p;

                if(robots_in_comm[i] == true) {
                    // store into pose array to be sent
                    Vector3ToPose(relative_pose, robots_relative_poses.poses[i]);
                    p.z = 0.25;
                    p.x = robots_relative_poses.poses[i].position.x;
                    p.y = robots_relative_poses.poses[i].position.y;

                    // update marker array for relative poses
                    // and do it only for the other robots
                    cluster_marker_msg.points.push_back(p);
                    
                    // check distance to me to see if this the relative positions should be updated
                    // ROS_INFO("%f %f %f %f %f %f", relative_pose.getX(), relative_pose.getY(), relative_pose.getZ(), 
                    //    mine.getX(), mine.getY(), mine.getZ());
                    distance = relative_pose.distance(mine);
                } else {
                    p.z = 0.25;
                    p.x = robots_relative_poses.poses[i].position.x;
                    p.y = robots_relative_poses.poses[i].position.y;

                    cluster_marker_far_msg.points.push_back(p);
                    distance = 10000000.0;
                }

                // also set the distances array
                robots_relative_distances.data[i] = distance;
            }
        }

        // send data to the system
        star_sender.publish(robots_relative_starting_positions);
        pose_sender.publish(robots_relative_poses);
        dist_sender.publish(robots_relative_distances);
        mark_sender.publish(cluster_marker_msg);
        marf_sender.publish(cluster_marker_far_msg);

        ros::spinOnce();
        seq += 1;
        sleep.sleep();
    }
    
    return 0;
}