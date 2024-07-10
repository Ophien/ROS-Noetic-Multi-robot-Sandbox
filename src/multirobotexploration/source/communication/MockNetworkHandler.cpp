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
#include "std_msgs/Int8MultiArray.h"
#include "multirobotsimulations/MockPackage.h"
#include <list>
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "multirobotsimulations/CustomPose.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "mocknetworkhandler");
    ros::NodeHandle node_handle;
    std::string ns = node_handle.getNamespace();
    
    int id = -1;
    int robot_count = 4;
    int queue_size = 1;
    double communication_range = 1.0;
    double rate = 1.0;
    ros::Rate hz(rate);
    
    node_handle.getParam("robots", robot_count);
    node_handle.getParam(ns+"/id",id);
    node_handle.getParam(ns+"/rate_mocknethandler", rate);
    node_handle.getParam(ns+"/mocknethandler_queue_size", queue_size);


    // ------------------------------------------------------------

    std::vector<int> robots_in_comm(robot_count, 0);
    std::vector<int>* robots_in_comm_ptr = &robots_in_comm;
    ros::Subscriber mock_comm_sub = node_handle.subscribe<std_msgs::Int8MultiArray>(ns + "/mock_communication_model/robots_in_comm", queue_size,
        [robots_in_comm_ptr](std_msgs::Int8MultiArray::ConstPtr rMsg){
            robots_in_comm_ptr->assign(rMsg->data.begin(), rMsg->data.end());
        }
    );

    // ------------------------------------------------------------
    // this topic is used to mock a TCP connection between robots
    // it should be replaced by a socket connection
    // ------------------------------------------------------------

    std::list<multirobotsimulations::MockPackage> received_packages_arr;
    std::list<multirobotsimulations::MockPackage>* packages_arr_ptr = &received_packages_arr;
    ros::Publisher mock_sock_connection = node_handle.advertise<multirobotsimulations::MockPackage>("/mock_ether", queue_size);

    // package receiver
    ros::Subscriber mock_received_packages = node_handle.subscribe<multirobotsimulations::MockPackage>("/mock_ether", queue_size,
        [packages_arr_ptr, id](multirobotsimulations::MockPackage::ConstPtr rMsg) {
            // add only messages where I'm the destination
            if(rMsg->destination != id) return;
            multirobotsimulations::MockPackage new_pkg;

            // copy package content here
            new_pkg.occ.data.assign(rMsg->occ.data.begin(), rMsg->occ.data.end());
            new_pkg.occ.header = rMsg->occ.header;
            new_pkg.occ.info = rMsg->occ.info;
            new_pkg.pose.position = rMsg->pose.position;
            new_pkg.pose.orientation = rMsg->pose.orientation;
            new_pkg.average_velocity = rMsg->average_velocity;
            new_pkg.sender = rMsg->sender;
            new_pkg.destination = rMsg->destination;

            packages_arr_ptr->push_back(new_pkg);
        }
    );

    // ------------------------------------------------------------
    

    // create a local network to propagate the received TCP mock package intra robot topics
    ros::Publisher intra_robot_network_sink = node_handle.advertise<multirobotsimulations::MockPackage>(ns + "/received_comm_packages", queue_size);

    nav_msgs::OccupancyGrid occ;
    nav_msgs::OccupancyGrid* occ_ptr = &occ;
    bool received_occ = false;
    bool* received_occ_ptr = &received_occ;
    ros::Subscriber intra_robot_fusion = node_handle.subscribe<nav_msgs::OccupancyGrid>(ns+"/fusion", queue_size, 
        [occ_ptr, received_occ_ptr](nav_msgs::OccupancyGrid::ConstPtr rMsg) {
            occ_ptr->info = rMsg->info;
            occ_ptr->header = rMsg->header;
            occ_ptr->data.assign(rMsg->data.begin(), rMsg->data.end());
            (*received_occ_ptr) = true;
        }
    );

    std_msgs::Float32 average_vel;
    std_msgs::Float32* average_vel_ptr = &average_vel;
    bool received_average_vel = false;
    bool* received_average_vel_ptr = &received_average_vel;
    ros::Subscriber intra_robot_avg_vel = node_handle.subscribe<std_msgs::Float32>(ns+"/avgd_average_displacement", queue_size, 
        [average_vel_ptr, received_average_vel_ptr](std_msgs::Float32::ConstPtr rMsg) {
            average_vel_ptr->data = rMsg->data;
            (*received_average_vel_ptr) = true;
        }
    );

    geometry_msgs::Pose pose;
    geometry_msgs::Pose* pose_ptr = &pose;
    bool received_pose = false;
    bool* received_pose_ptr = &received_pose;
    ros::Subscriber intra_robot_world_pose = node_handle.subscribe<multirobotsimulations::CustomPose>(ns+"/gmapping_pose/world_pose", queue_size, 
        [pose_ptr, received_pose_ptr](multirobotsimulations::CustomPose::ConstPtr rMsg) {
            pose_ptr->position = rMsg->pose.position;
            pose_ptr->orientation = rMsg->pose.orientation;
            (*received_pose_ptr) = true;
        }
    );

    while(ros::ok()) {
        for(int robot = 0; robot < robot_count; ++robot) {
            // if can communicate with robot 1,
            // send my data bundle to it, simulating the 
            // centralizing communication and avoiding deployments
            // with multimaster fkie
            //
            // This is a mock example with topics and
            // in a real deployment, this should be replaced
            // by a TPC or UDP socket connection
            if(robots_in_comm[robot] == 1 && robot != id) {
                multirobotsimulations::MockPackage to_send;
                
                // create package
                if(received_occ && received_pose && received_average_vel) {
                    to_send.average_velocity = average_vel.data;
                    to_send.pose.position = pose.position;
                    to_send.pose.orientation = pose.orientation;
                    to_send.sender = id;
                    to_send.destination = robot;
                    to_send.occ.data.assign(occ.data.begin(), occ.data.end());
                    to_send.occ.info = occ.info;
                    to_send.occ.header = occ.header;
                }

                mock_sock_connection.publish(to_send);
            }
        }

        // process one package at a time and send it to the intra robot components
        while(received_packages_arr.size() > 0) {
            intra_robot_network_sink.publish(received_packages_arr.front());
            received_packages_arr.pop_front();
        }

        ros::spinOnce();
        hz.sleep();
    }
}