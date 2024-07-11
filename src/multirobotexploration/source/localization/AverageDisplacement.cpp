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
#include <string.h>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose.h"
#include "multirobotsimulations/CustomPose.h"
#include "tf/tf.h"
#include <deque>

double ComputeAverageDisplacement(std::deque<double>& deltas) {
    double average_delta = 0.0;
    for(std::deque<double>::iterator i = deltas.begin(); i != deltas.end(); ++i) average_delta += (*i);
    if(deltas.size() > 0) average_delta /= deltas.size();
    return average_delta;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "averagedisplacement");
    ros::NodeHandle node_handle;
    ros::NodeHandle private_handle("~");

    std::string ns = node_handle.getNamespace();
    int id = -1;
    double rate = 1.0;
    int queue_size = 1;
    int pos_to_consider = 2;

    private_handle.getParam("id", id);
    private_handle.getParam("rate", rate);
    private_handle.getParam("queue_size", queue_size);
    private_handle.getParam("count", pos_to_consider);

    bool received_position = false;
    tf::Vector3 last_pos;
    tf::Vector3 world_pos;
    tf::Vector3* robotlastPosePtr = &last_pos;
    tf::Vector3* robotPosePtr = &world_pos;
    std_msgs::Float32 displacement;
    bool* received_position_ptr = &received_position;
    ros::Subscriber ep = node_handle.subscribe<multirobotsimulations::CustomPose>(ns + "/gmapping_pose/world_pose", queue_size, 
        [robotPosePtr,received_position_ptr,robotlastPosePtr](multirobotsimulations::CustomPoseConstPtr rMsg) {
            robotPosePtr->setX(rMsg->pose.position.x);
            robotPosePtr->setY(rMsg->pose.position.y);
            if(!(*received_position_ptr)) { 
                (*robotlastPosePtr) = (*robotPosePtr);
                (*received_position_ptr) = true;
            }
        });

    ros::Rate hz(rate);
    std::deque<double> delta_distances;
    ros::Publisher displace_pub = node_handle.advertise<std_msgs::Float32>(ns + "/avgd_average_displacement", queue_size);

    while(ros::ok()) {
        if(received_position) {
            double average_displace = ComputeAverageDisplacement(delta_distances);
            displacement.data = average_displace;
            displace_pub.publish(displacement);
        }

        delta_distances.push_back(world_pos.distance(last_pos));
        if(delta_distances.size() > pos_to_consider) delta_distances.pop_front();
        last_pos = world_pos;

        ros::spinOnce();
        hz.sleep();
    }
}