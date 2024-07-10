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
#include "geometry_msgs/Twist.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "multirobotsimulations/CustomPose.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64MultiArray.h"

using namespace std;

typedef enum {
    state_idle = 0,
    state_follow_goal = 1,
    state_clear = 2,
} RobotState;

double YAW;
double ROTATION_COUNT;
double ROTATION_SECONDS;
bool HAS_GOAL;
bool HAS_YAW;
bool SEND_VEL;
tf::Vector3 GOAL_WORLD_POS;
ros::Publisher CMD_VEL_PUB;
ros::Time EXPLORATION_BEGIN;
RobotState CURRENT_STATE;

void Initialize(void) {
    CURRENT_STATE = state_idle;
    HAS_GOAL = false;
    SEND_VEL = false;
    HAS_YAW = false;
}

void ClearingTimerInit() {
    ROTATION_COUNT = ROTATION_SECONDS;
    EXPLORATION_BEGIN = ros::Time().now();
}

void ClearingTimerDecay() {
    // decay rendezvous counter
    if(ROTATION_COUNT > 0.0) {
        ROTATION_COUNT -= (ros::Time().now() - EXPLORATION_BEGIN).toSec();
        EXPLORATION_BEGIN = ros::Time().now();
    }

    // clamp for positive values
    if(ROTATION_COUNT < 0.0) {
        ROTATION_COUNT = 0.0;
    }
}

void Stop(ros::Publisher& rCmdVelPub) {
    geometry_msgs::Twist vel;
    vel.linear.x = 0.0;
    vel.angular.z = 0.0;
    rCmdVelPub.publish(vel);
}

void Rotate(ros::Publisher& rCmdVelPub, const double& rAngularVel) {
    geometry_msgs::Twist vel;
    vel.linear.x = 0.0;
    vel.angular.z = 1.0;
    rCmdVelPub.publish(vel);
}

void MoveForward(ros::Publisher& rCmdVelPub) {
    geometry_msgs::Twist cmd_vel_msg;
    cmd_vel_msg.angular.z = 0;
    cmd_vel_msg.linear.x = 0.2;
    rCmdVelPub.publish(cmd_vel_msg);
}

void StartCallback(const std_msgs::Bool& rMsg) {
    SEND_VEL = rMsg.data;
}

void PoseCallback(const multirobotsimulations::CustomPose& rMsg) {
    YAW = tf::getYaw(rMsg.pose.orientation);
    HAS_YAW = true;
}

void GoalPositionCallback(const geometry_msgs::Pose& rMsg) {
    GOAL_WORLD_POS.setX(rMsg.position.x);
    GOAL_WORLD_POS.setY(rMsg.position.y);
    GOAL_WORLD_POS.setZ(0.0);
    HAS_GOAL = true;
}

void SetGoalStateCallback(const std_msgs::String& rMsg) {
    CURRENT_STATE = state_follow_goal;
}

void SetIdleStateCallback(const std_msgs::String& rMsg) {
    Stop(CMD_VEL_PUB);
    CURRENT_STATE = state_idle;
}

void SetClearStateCallback(const std_msgs::String& rMsg) {
    Stop(CMD_VEL_PUB);
    ClearingTimerInit();
    CURRENT_STATE = state_clear;
}

void DesaiController(double& theta, double& dx, double& dy, double& d, double& v, double& w) {
    double cosine_q = cos(theta);
    double sine_q   = sin(theta);

    // check if robot orientation is the same as the sub-goal
    // to se the linear velocity. This will force the robot to turn to the
    // sub-goal before going ahead and prevent accidents.
    tf::Vector3 orientation(1,0,0);
    tf::Vector3 sub_goal_dir = tf::Vector3(dx, dy, 0).normalize();
    orientation.rotate(tf::Vector3(0,0,1), theta);

    // angular velocity should allow the robot to turn at will
    v = (dx * cosine_q + dy * sine_q) * 0.2;
    w = dx * ((-sine_q)/d) + dy * (cosine_q / d);
}

void GetParameters(ros::NodeHandle& nodeHandle, 
                   const std::string& ns, 
                   int& robots, 
                   int& robotId, 
                   int& queueSize, 
                   int& rate, 
                   double& rorationSeconds, 
                   double& maxV, 
                   double& maxW, 
                   double& D) {
    if(!nodeHandle.getParam("/robots", robots)) {
        throw std::runtime_error("Parameter " + ns + "/robots not found");
    }

    if(!nodeHandle.getParam(ns+"/id", robotId)) {
        throw std::runtime_error("Parameter " + ns + "/id not found");
    }

    if(!nodeHandle.getParam(ns+"/desai_queue_size", queueSize)) {
        throw std::runtime_error("Parameter " + ns + "/desai_queue_size not found");
    }

    if(!nodeHandle.getParam(ns+"/clearing_time", rorationSeconds)) {
        throw std::runtime_error("Parameter " + ns + "/clearin_time not found");
    }

    if(!nodeHandle.getParam(ns+"/rate_desai", rate)) {
        throw std::runtime_error("Parameter " + ns + "/rate_desai not found");
    }

    if(!nodeHandle.getParam(ns+"/desai_max_linear_vel", maxV)) {
        throw std::runtime_error("Parameter " + ns + "/desai_max_linear_vel not found");
    }

    if(!nodeHandle.getParam(ns+"/desai_max_angular_vel", maxW)) {
        throw std::runtime_error("Parameter " + ns + "/desai_max_angular_vel not found");
    }

    if(!nodeHandle.getParam(ns+"/desai_D", D)) {
        throw std::runtime_error("Parameter " + ns + "/desai_D not found");
    }
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "desai_controller");
    Initialize();
    ros::NodeHandle node_handle;

    int queue_size = 1;
    int rate = 10;
    int robot_id = -1;
    int robots = -1;
    double v = 0.0;
    double w = 0.0;
    double rho = 0.0;
    double d = 0.0;
    double max_v = 0.4;
    double max_w = 0.5;
    double tolerance = 0.2;
    std::string ns;

    try {
        ns = ros::this_node::getNamespace();
        GetParameters(node_handle, ns, robots, robot_id, queue_size, rate, ROTATION_SECONDS, max_v, max_w, d);
        ROS_INFO("Parameters retrieved successfully.");
        ROS_INFO("robots: %d", robots);
        ROS_INFO("id: %d", robot_id);
        ROS_INFO("desai_queue_size: %d", queue_size);
        ROS_INFO("rate_desai: %d", rate);
        ROS_INFO("clearing_time: %f", ROTATION_SECONDS);
        ROS_INFO("desai_max_linear_vel: %f", max_v);
        ROS_INFO("desai_max_angular_vel: %f", max_w);
        ROS_INFO("desai_D: %f", d);
    } catch(const std::runtime_error& e) {
        ROS_ERROR("%s", e.what());
    }

    ros::Rate loop_rate(rate);

    /*
     * Outer nodes topics
     */
    CMD_VEL_PUB = node_handle.advertise<geometry_msgs::Twist>(ns + "/cmd_vel", queue_size);
    ros::Subscriber yaw_sub = node_handle.subscribe(ns + "/gmapping_pose/world_pose", queue_size, &PoseCallback);

    /*
     * This node's topics
     */    
    ros::Subscriber enable_sub = node_handle.subscribe(ns + "/desai_controller/enable", queue_size, &StartCallback);
    ros::Subscriber goal_dir_sub = node_handle.subscribe(ns + "/pfield_local_planner/potential_force", queue_size, &GoalPositionCallback);
    ros::Subscriber clear_position = node_handle.subscribe(ns + "/desai_controller/set_clear_state", queue_size, &SetClearStateCallback);
    ros::Subscriber goal_position = node_handle.subscribe(ns + "/desai_controller/set_goal_state", queue_size, &SetGoalStateCallback);
    ros::Subscriber idle_position = node_handle.subscribe(ns + "/desai_controller/set_idle_state", queue_size, &SetIdleStateCallback);
    ros::Publisher  clear_complete = node_handle.advertise<std_msgs::String>(ns + "/desai_controller/clear_complete", queue_size);
    ros::Publisher  goal_complete = node_handle.advertise<std_msgs::String>(ns + "/desai_controller/reached_goal", queue_size);

    geometry_msgs::Twist twist_msg;
    std_msgs::String std_msg;
    double dx, dy;

    while(ros::ok()) {
        if(SEND_VEL == true && HAS_YAW == true) {
            switch(CURRENT_STATE) {
                case state_idle:
                break;
                case state_follow_goal:
                    // TODO::convert force from potential field to world coordinates
                    dx = GOAL_WORLD_POS.getX();// - WORLD_POS.getX();
                    dy = GOAL_WORLD_POS.getY();// - WORLD_POS.getY();
                    rho = sqrt(dx * dx + dy * dy);
                    
                    if (rho < tolerance) {
                        v = 0.0;
                        w = 0.0;

                        // tell system that it reached goal
                        goal_complete.publish(std_msg);
                        ROS_INFO("[%s Dessai Controller] reached goal.", ns.c_str());
                    }

                    // compute linear and angular velocities
                    DesaiController(YAW, dx, dy, d, v, w);

                    // filter velocities
                    if(isnan(w)) w = 0.0;
                    if(isnan(v)) v = 0.0;

                    // send velocities to motors
                    twist_msg.linear.x = v;
                    twist_msg.angular.z = w;

                    CMD_VEL_PUB.publish(twist_msg);
                break;
                case state_clear:
                    ClearingTimerDecay();

                    if(ROTATION_COUNT <= 0.0) {
                        Stop(CMD_VEL_PUB);
                        CURRENT_STATE = state_idle;
                        clear_complete.publish(std_msg);
                        ROS_INFO("[%s Dessai Controller] finished clearing.", ns.c_str());
                    } else {
                        Rotate(CMD_VEL_PUB, max_w);
                    }
                break;
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
