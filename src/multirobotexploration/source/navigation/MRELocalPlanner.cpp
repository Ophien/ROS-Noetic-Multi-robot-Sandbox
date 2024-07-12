/*
 * Copyright (c) 2023, Alysson Ribeiro da Silva
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
#include "teb_local_planner/teb_local_planner_ros.h"
#include "multirobotsimulations/CustomPose.h"
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Float32.h"
#include "multirobotsimulations/MockPackage.h"
#include "std_msgs/Float64MultiArray.h"

using namespace teb_local_planner; // it is ok here to import everything for testing purposes

void CreateMarker(visualization_msgs::Marker& rInput, const char* pNs, const int& rId, const int& rSeq) {
    rInput.id = rId;
    rInput.header.frame_id = "robot_" + std::to_string(rId) + std::string("/map");
    rInput.header.stamp = ros::Time().now();
    rInput.ns = pNs;
    rInput.points.clear();
    rInput.type = visualization_msgs::Marker::LINE_STRIP;
    rInput.action = visualization_msgs::Marker::MODIFY;
    rInput.pose.orientation.x = 0.0;
    rInput.pose.orientation.y = 0.0;
    rInput.pose.orientation.z = 0.0;
    rInput.pose.orientation.w = 1.0;
    rInput.scale.x = 0.05;
    rInput.scale.y = 0.05;
    rInput.scale.z = 0.05;
    rInput.color.a = 1.0;
    rInput.color.r = 0.3;
    rInput.color.g = 0.3;
    rInput.color.b = 1.3;
    rInput.lifetime = ros::Duration(60);
}

bool CheckNearPriority(std::vector<double>& avg_displacement, 
                       std::vector<double>& distances,
                       std::vector<int>& robots_in_comm,
                       const int& rRobotId,
                       const double& rCommDist,
                       const double& min_displacement) {
    for(size_t i = 0; i < robots_in_comm.size(); ++i) {
        if(i == rRobotId) continue;
        if(robots_in_comm[i] == 1 && 
           avg_displacement[i] > min_displacement &&
           i < rRobotId &&
           distances[i] < rCommDist) return true;
    }
    return false;
}

void AssembleSparsePath(nav_msgs::Path& currentPath, nav_msgs::Path& filteredPath, const int& viaIncrement, visualization_msgs::Marker& globalPath) {
    filteredPath.poses.clear();
    globalPath.points.clear();
    int size = currentPath.poses.size();
    int increment = viaIncrement;
    size_t i = 0;
    filteredPath.poses.push_back(currentPath.poses[i]);
    geometry_msgs::Point p;
    p.x = currentPath.poses[i].pose.position.x;
    p.y = currentPath.poses[i].pose.position.y;
    globalPath.points.push_back(p);
    if(size > increment) i = increment;
    while(i < size) {
        filteredPath.poses.push_back(currentPath.poses[i]);
        geometry_msgs::Point p;
        p.x = currentPath.poses[i].pose.position.x;
        p.y = currentPath.poses[i].pose.position.y;
        globalPath.points.push_back(p);
        if(i + increment >= size && i != size - 1) {
            i = size-2;
            increment = 1;
        }
        i += increment;
    }
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "node_mre_local_planner");
    TebConfig config;
    ros::NodeHandle private_handle("~");
    ros::NodeHandle node_handle;
    ros::Rate rate(100);
    std::string ns = node_handle.getNamespace();
    int queue_size = 100;
    int seq = 0;
    int id = 0;
    int robots = 0;
    int max_waypoints = 5;
    int via_increment = 16;
    int increment = 0;
    double comm_dist = 1.0;
    double min_displacement = 0.1;

    node_handle.getParam("/robots", robots);
    private_handle.getParam("id", id);
    private_handle.getParam("min_dist", comm_dist);
    private_handle.getParam("min_displacement", min_displacement);
    private_handle.getParam("waypoints_to_use", max_waypoints);
    private_handle.getParam("via_points_increment", via_increment);

    // load ros parameters from node handle
    config.loadRosParamFromNodeHandle(private_handle);
    std::vector<ObstaclePtr> obst_vector;
    ViaPointContainer via_points;
    RobotFootprintModelPtr robot_footprint(new PointRobotFootprint());
    TebVisualizationPtr visual = TebVisualizationPtr(new TebVisualization(private_handle, config));
    PlannerInterfacePtr planner = PlannerInterfacePtr(new HomotopyClassPlanner(config, &obst_vector, robot_footprint, visual, &via_points));
    std::vector<ros::Subscriber> subs;

    std::vector<ObstaclePtr>* obst_vector_ptr = &obst_vector;
    subs.push_back(node_handle.subscribe<costmap_converter::ObstacleArrayMsg>(
        ns + "/obstacle_array",
        queue_size,
        [obst_vector_ptr](costmap_converter::ObstacleArrayConstPtr rMsg){
            obst_vector_ptr->clear();
            for(auto& obs : rMsg->obstacles) {
                PolygonObstacle obstacle;
                for(auto& point : obs.polygon.points)
                    obstacle.pushBackVertex(point.x, point.y);
                obstacle.finalizePolygon();
                obst_vector_ptr->push_back(boost::make_shared<PolygonObstacle>(obstacle));
            }
        }
    ));

    multirobotsimulations::CustomPose robotPose;
    multirobotsimulations::CustomPose* robotPosePtr = &robotPose;
    subs.push_back(node_handle.subscribe<multirobotsimulations::CustomPose>(ns + "/gmapping_pose/world_pose", queue_size, 
        [robotPosePtr](multirobotsimulations::CustomPoseConstPtr rMgs) {
            robotPosePtr->robot_id = rMgs->robot_id;
            robotPosePtr->pose = rMgs->pose;
        }));
    
    nav_msgs::Path current_path;
    nav_msgs::Path filtered_path;
    nav_msgs::Path* currentPathPtr = &current_path;
    subs.push_back(node_handle.subscribe<nav_msgs::Path>(ns + "/sub_goal_nav/current_path", queue_size,
        [currentPathPtr](nav_msgs::PathConstPtr rMsg){
            currentPathPtr->header = rMsg->header;
            currentPathPtr->poses.assign(rMsg->poses.begin(), rMsg->poses.end());
        }));

    ros::Publisher cmd_vel = node_handle.advertise<geometry_msgs::Twist>(ns + "/node_rosaria/cmd_vel", queue_size);
    geometry_msgs::Twist twist_vel;
    geometry_msgs::PoseStamped prev_pose;
    geometry_msgs::PoseStamped last_pose;

    visualization_msgs::Marker global_path;
    ros::Publisher via_points_path = node_handle.advertise<visualization_msgs::Marker>(ns + "/mre_local_planner/global_via_points", queue_size);

    std::vector<int> robots_in_comm(robots, 0);
    bool received_comm = false;
    
    std::vector<int>* robots_in_comm_ptr = &robots_in_comm;
    bool* received_distances_ptr = &received_comm;
    subs.push_back(node_handle.subscribe<std_msgs::Int8MultiArray>(
        ns + "/mock_communication_model/robots_in_comm", 
        queue_size,
        [received_distances_ptr, robots_in_comm_ptr](std_msgs::Int8MultiArray::ConstPtr rMsg) {
            robots_in_comm_ptr->assign(rMsg->data.begin(), rMsg->data.end());
            (*received_distances_ptr) = true;
        }));

    /*
     * The following subscriber should be replaced with a network package handler
    */
    std::vector<double> average_displacements(robots, 0.0);
    std::vector<double>* avg_displace_ptr = &average_displacements;
    for(int i = 0; i < robots; ++i) {
        if(i == id) continue;
        subs.push_back(node_handle.subscribe<std_msgs::Float32>("/robot_"+std::to_string(i)+"/avgd_average_displacement", 
            queue_size,
            [avg_displace_ptr,i](std_msgs::Float32::ConstPtr rMsg) {
                avg_displace_ptr->at(i) = rMsg->data;
            }
        ));
    }

    std::vector<double> distances(robots, 0.0);
    std::vector<double>* dist_ptr = &distances;
    ros::Subscriber dist_sub = node_handle.subscribe<std_msgs::Float64MultiArray>(ns + "/relative_pose_estimator/distances", queue_size,
        [dist_ptr](std_msgs::Float64MultiArray::ConstPtr rMsg) {
            dist_ptr->assign(rMsg->data.begin(), rMsg->data.end());
        }
    );
   
    while(ros::ok()) {
        twist_vel.linear.x = 0.0;
        twist_vel.angular.z = 0.0;
        
        // not optimal mechanism to avoid traffic
        if(received_comm && 
           current_path.poses.size() > 1 && 
           CheckNearPriority(average_displacements, 
                             distances, 
                             robots_in_comm, 
                             id, 
                             comm_dist,
                             min_displacement) == false) {
            /*
             * Global path markers
             */
            CreateMarker(global_path, ns.c_str(), id, seq);
            AssembleSparsePath(current_path, filtered_path, via_increment, global_path);

            /*
             * Compute sparse via poses
             */
            via_points.clear();
            int i = 0;
            while(via_points.size() < max_waypoints && via_points.size() < filtered_path.poses.size()) {
                via_points.push_back(Eigen::Vector2d(filtered_path.poses[i].pose.position.x, filtered_path.poses[i].pose.position.y));
                i++;
            }

            /*
             * Compute final pose
            */
            // get the two last poses to compute the final yaw configuration
            prev_pose = filtered_path.poses[via_points.size()-2];
            last_pose = filtered_path.poses[via_points.size()-1];

            // get the yaw from the first to the last point
            double cur_angle = tf::getYaw(robotPose.pose.orientation);
            double end_pose_yaw = atan2(last_pose.pose.position.y - prev_pose.pose.position.y, last_pose.pose.position.x - prev_pose.pose.position.x);

            /*
             * Publish visuals and path
             */      
            visual->publishObstacles(obst_vector);
            visual->publishViaPoints(via_points);
            via_points_path.publish(global_path);
            planner->plan(PoseSE2(robotPose.pose.position.x,robotPose.pose.position.y,cur_angle), 
                          PoseSE2(last_pose.pose.position.x, last_pose.pose.position.y,end_pose_yaw));
            planner->visualize();

            /*
             * Publish velocity commands
             */       
            double vx, vy, omega;
            planner->getVelocityCommand(vx, vy, omega, 3);
            twist_vel.linear.x = vx;
            twist_vel.angular.z = omega;

            seq += 1;
        }
        
        cmd_vel.publish(twist_vel);
        ros::spinOnce();
        rate.sleep();
    }
}