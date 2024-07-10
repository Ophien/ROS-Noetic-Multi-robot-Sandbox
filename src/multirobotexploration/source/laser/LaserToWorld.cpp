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
#include "tf/tf.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "multirobotsimulations/CustomPose.h"
#include "Common.h"
#include <vector>

using namespace std;

tf::Vector3 robot_world;
tf::Vector3 map_origin;
tf::Vector3 laser_world_origin;
double robot_yaw;
vector<geometry_msgs::Pose> world_readings;
vector<geometry_msgs::Pose> occ_readings;
nav_msgs::OccupancyGrid OCC;
bool HAS_LASER = false;
bool HAS_POSE = false;
bool HAS_OCC = false;
double LASER_ERROR = 0.3;
double RESOLUTION = 0.1;
tf::Vector3 translate;
tf::Quaternion orientation;

void EstimatePoseWorldCallback(const multirobotsimulations::CustomPose& rMsg) {
    robot_world.setX(rMsg.pose.position.x);
    robot_world.setY(rMsg.pose.position.y);
    robot_world.setZ(0.0);
    robot_yaw = tf::getYaw(rMsg.pose.orientation);
    HAS_POSE = true;
}

void OccupancyGridCallback(const nav_msgs::OccupancyGrid& rMsg) {
    OCC.data.assign(rMsg.data.begin(), rMsg.data.end());
    OCC.header = rMsg.header;
    OCC.info = rMsg.info;
    map_origin.setX(rMsg.info.origin.position.x);
    map_origin.setY(rMsg.info.origin.position.y);
    map_origin.setZ(0.0);
    RESOLUTION = rMsg.info.resolution;
    HAS_OCC = true;
}

void LaserCapture(const sensor_msgs::LaserScan& rMsg) {
    if(HAS_POSE == false || HAS_OCC == false) return

    // clear previous readings
    world_readings.clear();
    occ_readings.clear();
    double increment   = rMsg.angle_increment;
    double theta       = rMsg.angle_min;
    tf::Vector3 rot_axis(0,0,1);
    tf::Vector3 pose_tf(robot_world);

    // compute laser robot frame exact positions
    for(size_t beam = 0; beam < rMsg.ranges.size(); ++beam) {
        double range = rMsg.ranges[beam];

        // crop lasers with threshold
        if(range > rMsg.range_max - LASER_ERROR) {
            theta += increment;
            continue;
        }

        // do rotation on YZ plane normal X
        tf::Vector3 laser_vec(1,0,0);

        // rotate arround Z
        laser_vec = laser_vec.rotate(rot_axis, theta);

        // extend vector with range in meters
        tf::Vector3 laser_world = laser_vec * range + tf::Vector3(0.3,0.0,0.0);
        laser_world = laser_world.rotate(tf::Vector3(0,0,1), robot_yaw);
        laser_world = laser_world.rotate(tf::Vector3(0,0,1), tf::getYaw(orientation));
        laser_world += pose_tf;
        geometry_msgs::Pose laser_pose_world;
        laser_pose_world.position.x = laser_world.getX();
        laser_pose_world.position.y = laser_world.getY();
        laser_pose_world.position.z = range;

        // convert world laser coordinates into occ laser coordinates
        tf::Vector3 world_occ;
        WorldToMap(OCC, laser_world, world_occ);
        geometry_msgs::Pose laser_pose_occ;
        laser_pose_occ.position.x = world_occ.getX();
        laser_pose_occ.position.y = world_occ.getY();
        laser_pose_occ.position.z = range;

        // add laser point to readings array
        world_readings.push_back(laser_pose_world);
        occ_readings.push_back(laser_pose_occ);

        // increment ray angle
        theta += increment;
    }

    HAS_LASER = true;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "laser_to_world");
    std::string ns = ros::this_node::getNamespace();
    ros::NodeHandle node_handle;
    ros::NodeHandle private_handle("~");
    int queue_size = 1;
    int rate = 10;

    private_handle.getParam("rate", rate);
    private_handle.getParam("queue_size", queue_size);
    ros::Rate loop_rate(rate);

    double x,y,z,r,p,yaw;
    private_handle.getParam("x", x);
    private_handle.getParam("y", y);
    private_handle.getParam("z", z);
    private_handle.getParam("r", r);
    private_handle.getParam("p", p);
    private_handle.getParam("y", yaw);

    translate = tf::Vector3(x,y,0.0);
    orientation = tf::createQuaternionFromRPY(r,p,yaw);

    /*
     * Outer nodes topics
     */
    ros::Subscriber laser_sub = node_handle.subscribe(ns + "/scan", queue_size, &LaserCapture);
    ros::Subscriber pose_sub = node_handle.subscribe(ns + "/gmapping_pose/world_pose", queue_size, &EstimatePoseWorldCallback);
    ros::Subscriber occ_sub = node_handle.subscribe(ns + "/map", queue_size, &OccupancyGridCallback);

    /*
     * This node's topics
     */ 
    ros::Publisher  laser_pub = node_handle.advertise<geometry_msgs::PoseArray>(ns + "/laser_to_world/laser_world", queue_size);
    ros::Publisher  laser_occ_pub = node_handle.advertise<geometry_msgs::PoseArray>(ns + "/laser_to_world/laser_occ", queue_size);

    while(ros::ok()) {
        if(HAS_LASER) {
            geometry_msgs::PoseArray world_msg;
            world_msg.poses.assign(world_readings.begin(), world_readings.end());
            laser_pub.publish(world_msg);
            geometry_msgs::PoseArray occmsg;
            occmsg.poses.assign(occ_readings.begin(), occ_readings.end());
            laser_occ_pub.publish(occmsg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}