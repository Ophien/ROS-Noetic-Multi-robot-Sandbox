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

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "mockcommunicationmodel");
    ros::NodeHandle handle;
    ros::NodeHandle private_handle("~");

    int robots = 1;
    int id = -1;
    int rate = 5;
    int queue_size = 1;
    double comm_dist = 5.0;
    std::string ns = handle.getNamespace();

    handle.getParam("/robots", robots);
    private_handle.getParam("id", id);
    private_handle.getParam("queue_size", queue_size);
    private_handle.getParam("comm_dist", comm_dist);
    private_handle.getParam("rate", rate);

    if(robots <= 0) robots = 1;

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
            ROS_INFO("[%s relative_pose_estimator] relative to %d: %f %f %f", ns.c_str(), i, dir.getX(), dir.getY(), dir.getZ());
        } else {
            ROS_INFO("[%s relative_pose_estimator] relative to self: %f %f %f", ns.c_str(), relative[i].getX(), relative[i].getY(), relative[i].getZ());
        }
    }

    std::vector<ros::Subscriber> subs;
    ros::Rate sleep(rate);

    std::vector<bool> received_poses(robots, false);
    std::vector<bool>* temp_poses = &received_poses;

    geometry_msgs::Pose robots_world_poses[robots];
    geometry_msgs::Pose* robots_world_poses_temp = robots_world_poses;

    for(int i = 0; i < robots; ++i) {    
        subs.push_back(handle.subscribe<multirobotsimulations::CustomPose>("/robot_" + std::to_string(i) + "/gmapping_pose/world_pose", queue_size, 
            [temp_poses, i, robots_world_poses_temp](multirobotsimulations::CustomPose::ConstPtr rMsg) {
                robots_world_poses_temp[i].position = rMsg->pose.position;
                robots_world_poses_temp[i].orientation = rMsg->pose.orientation;
                temp_poses->at(i) = true;
            }));
    }

    ros::Publisher comm_sender = handle.advertise<std_msgs::Int8MultiArray>(ns + "/mock_communication_model/robots_in_comm", queue_size);

    // nearby robots indicator it is used as a simplification of the communication model
    std_msgs::Int8MultiArray robots_in_comm;
    robots_in_comm.data.assign(robots, 0);

    // iteration sequence
    long long unsigned int seq = 0;

    while(ros::ok()) {
        // translate all the poses here
        tf::Vector3 relative_pose;
        tf::Vector3 mine;
        PoseToVector3(robots_world_poses[id], mine);
        double distance = 0.0;
        for(int i = 0; i < robots; ++i) {
            if(received_poses[i] == true) {
                PoseToVector3(robots_world_poses[i], relative_pose);
                relative_pose += relative[i];

                // check distance to me to see if this the relative positions should be updated
                distance = relative_pose.distance(mine);

                if(distance < comm_dist) {
                    // set nearby robots
                    robots_in_comm.data[i] = 1;
                } else {
                    robots_in_comm.data[i] = 0;
                }
            }
        }

        // send data to the system
        comm_sender.publish(robots_in_comm);

        ros::spinOnce();
        seq += 1;
        sleep.sleep();
    }
    
    return 0;
}