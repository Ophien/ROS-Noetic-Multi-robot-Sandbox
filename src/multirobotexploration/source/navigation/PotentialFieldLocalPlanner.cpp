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
#include "geometry_msgs/PoseArray.h"
#include "multirobotsimulations/CustomImage.h"
#include "multirobotsimulations/CustomPose.h"
#include <vector>
#include "Common.h"

/*
 * used the opencv for debug purposes 
 * on the received configuration space image
 */
//#include <opencv4/opencv2/core.hpp>
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"

//#include <opencv4/opencv2/core.hpp>
//#include <opencv4/opencv2/highgui.hpp>

// potential scales (classic)
double K_ATT;
double K_REP;
double R;
double OBJ_RADIUS;
double VIEW_SIZE_METERS;
int VIEW_SIZE;
bool NORM;
bool RECEIVED_C_SPACE;
bool HAS_POSE;
bool HAS_ULTRASONIC;
bool DEBUG;
bool MAP_INFO;
//cv::Mat C_SPACE;
//cv::Mat POTENTIAL_VIEWS;
tf::Vector3 WORLD_POS;
tf::Vector3 CURRENT_FORCE;
std::vector<tf::Vector3> OBSTACLES;
Vec2i POS;
Vec2i GOAL_OCC;
geometry_msgs::PoseArray OPOSES;
geometry_msgs::PoseArray FPOSES;
nav_msgs::OccupancyGrid OCC;
nav_msgs::OccupancyGrid MAP;
std::mutex POSES_MUTEX;
bool FP;
bool OP;

void Initialize(void) {
    K_ATT = 0.0;
    K_REP = 0.0;
    R = 5.0;
    OBJ_RADIUS = 0.0;
    VIEW_SIZE = 50+1;
    FP = false;
    OP = false;
    NORM = true;
    MAP_INFO = false;
    RECEIVED_C_SPACE = false;
    HAS_POSE = false;
    HAS_ULTRASONIC = false;
    DEBUG = true;
    CURRENT_FORCE = tf::Vector3(0,0,0);
    GOAL_OCC = Vec2i::Create(1000,1000);
}

void EstimatePoseCallback(const multirobotsimulations::CustomPose& rMsg) {
    WORLD_POS.setX(rMsg.pose.position.x);
    WORLD_POS.setY(rMsg.pose.position.y);
    HAS_POSE = true;
}

void MapInfoCallback(const nav_msgs::OccupancyGrid& rMsg) {
    MAP.header = rMsg.header;
    MAP.info = rMsg.info;
    MAP_INFO = true;
}

void CSpaceCallback(const nav_msgs::OccupancyGrid& rMsg) {
    // do not need to copy the data...
    // since it is used only to the the
    // current scale and origin of the cspace
    OCC.info = rMsg.info;
    OCC.header = rMsg.header;
    OCC.data.assign(rMsg.data.begin(), rMsg.data.end());
    OBSTACLES.clear();
    for(size_t i = 0; i < OCC.data.size(); ++i) {
        int x = (int)(i % OCC.info.width);
        int y = (int)(i / OCC.info.width);
        if(OCC.data[i] > 90) OBSTACLES.push_back(tf::Vector3(x,y,0.0));
    }
    RECEIVED_C_SPACE = true;
}

void GoalSetCallback(const geometry_msgs::Pose& rMsg) {
    GOAL_OCC.x = rMsg.position.x;
    GOAL_OCC.y = rMsg.position.y;
}

void BuildPotentialFieldViewNew(const tf::Vector3& rCellObj, 
                             const tf::Vector3& rCellPos, 
                             const std::vector<tf::Vector3>& rObstacles,
                             tf::Vector3& rOutForce,
                             //cv::Mat& rOutVisualizationMap,
                             const double& kAtt,
                             const double& kRep,
                             const double& R,
                             const double& objRadius) {   
    //rOutVisualizationMap = cv::Scalar(0,0,0);
    const int half = (int)(OCC.info.width / 2.0);
    tf::Vector3 att(0,0,0);
    tf::Vector3 potential(0,0,0);

    // compute attraction
    att = rCellObj - rCellPos;
    if(!att.isZero()) att.normalize();

    // compute potential for all cells
    double len, len_n, len_inv, d, force_threshold;
    bool is_near;
    int dx, dy;
    for(int y = 0; y < OCC.info.height; ++y) {
        for(int x = 0; x < OCC.info.width; ++x) {
            tf::Vector3 rep(0,0,0);
            tf::Vector3 cell(x,y,0.0);
            is_near = false;
            double c = 0.0;
            for(size_t obs = 0; obs < rObstacles.size(); ++obs) {
                tf::Vector3 v =  cell - rObstacles[obs];
                d = v.length();
                if(d > 0.0 && d <= R) //rep += v;
                {
                    tf::Vector3 force = v.normalize();
                    force = force * ((1.0/d) - (1.0/R)) * (1.0 / (d * d));
                    rep += kRep*force;
                } 
                if(d <= objRadius)  is_near = true;
            }

            // compute repulsions
            potential = kAtt*att + rep;
            if(x == half && y == half) {
                rOutForce = potential;
            }

            //if(is_near == false) {
            //    len_n = 1.0 / (1.0 + (double)potential.length());
            //    len_inv = 1.0 - len_n;
            //    rOutVisualizationMap.at<cv::Vec3b>(y,x) = cv::Vec3b((int)(255.0 * len_n), 0, (int)(255.0 * len_inv));
            //}
        }
    }
}

void OccupiedPosesCallback(const geometry_msgs::PoseArray& rMsg) {
    OPOSES.poses.assign(rMsg.poses.begin(), rMsg.poses.end());
    OP = true;
}

void FreePosesCallback(const geometry_msgs::PoseArray& rMsg) {
    FPOSES.poses.assign(rMsg.poses.begin(), rMsg.poses.end());
    FP = true;
}

int main(int argc, char* argv[]) {
    /*
     * Node initialization routines.
     */
    ros::init(argc, argv, "pfield_local_planner");
    ros::NodeHandle node_handle;
    std::string ns = ros::this_node::getNamespace();

    Initialize();
    int queue_size = 1;
    int rate = 30;

    node_handle.getParam(ns+"/pfield_queue_size", queue_size);
    node_handle.getParam(ns+"/pfield_local_view_size", VIEW_SIZE_METERS);
    node_handle.getParam(ns+"/pfield_k_rep", K_REP);
    node_handle.getParam(ns+"/pfield_k_att", K_ATT);
    node_handle.getParam(ns+"/pfield_r", R);
    node_handle.getParam(ns+"/rate_pfield", rate);

    ros::Rate loop_rate(rate);

    /*
     * Messages to publish force and potential field debug heatmap.
     */
    geometry_msgs::Pose force;
    ros::Subscriber ob = node_handle.subscribe(ns + "/map", queue_size, &MapInfoCallback);
    ros::Subscriber oc = node_handle.subscribe(ns + "/c_space_local", queue_size, &CSpaceCallback);
    ros::Subscriber ep = node_handle.subscribe(ns + "/gmapping_pose/world_pose", queue_size, &EstimatePoseCallback);
    ros::Subscriber goal_point_sub = node_handle.subscribe(ns + "/pfield_local_planner/goal_occ", queue_size, &GoalSetCallback);
    ros::Publisher  force_pub = node_handle.advertise<geometry_msgs::Pose>(ns + "/pfield_local_planner/potential_force", queue_size);
    tf::Vector3 temp_pos;
    tf::Vector3 temp_obj;
    bool init_view = false;

    while(ros::ok()) {
        if(HAS_POSE && RECEIVED_C_SPACE && MAP_INFO) {
            WorldToMap(MAP, WORLD_POS, POS);
            temp_pos.setX(POS.x);
            temp_pos.setY(POS.y);
            temp_obj.setX(GOAL_OCC.x);
            temp_obj.setY(GOAL_OCC.y);
            BuildPotentialFieldViewNew(temp_obj, 
                            temp_pos,
                            OBSTACLES, 
                            CURRENT_FORCE,
                            // POTENTIAL_VIEWS,
                            K_ATT,
                            K_REP,
                            R,
                            OBJ_RADIUS);

            // convert force in the OCC space to world space          
            force.position.x = CURRENT_FORCE.getX();
            force.position.y = CURRENT_FORCE.getY();
            force_pub.publish(force);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}