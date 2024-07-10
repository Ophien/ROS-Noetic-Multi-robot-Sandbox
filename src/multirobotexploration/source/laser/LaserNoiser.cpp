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
#include <vector>
#include <signal.h>
#include <random>

using namespace std;

void sig_handler(int sig) {
    switch (sig) {
        case SIGKILL:
            fprintf(stderr, "program killed...\n");
            abort();
        case SIGSEGV:
            fprintf(stderr, "give out a backtrace or something...\n");
            abort();
        default:
            fprintf(stderr, "wasn't expecting that!\n");
            abort();
    }
}

class LaserNoise{
    public:
        LaserNoise(const int& rMaxRange = 10, const int& rMaxNoiseDist = 5, const double& rNoiseProb = 0.999){
            apLaser = nullptr;
            aMaxRange = rMaxRange;
            aMaxNoiseDist = rMaxNoiseDist;
            aNoiseProb = rNoiseProb;
        }
        ~LaserNoise(){
            delete apLaser;
            apLaser = nullptr;
        }
        void LaserCapture(const sensor_msgs::LaserScan& rMsg) {
            if(apLaser == nullptr) apLaser = new sensor_msgs::LaserScan();
            apLaser->angle_increment = rMsg.angle_increment;
            apLaser->angle_max = rMsg.angle_max;
            apLaser->angle_min = rMsg.angle_min;
            apLaser->header = rMsg.header;
            apLaser->intensities.assign(rMsg.intensities.begin(), rMsg.intensities.end());
            apLaser->range_max = rMsg.range_max;
            apLaser->range_min = rMsg.range_min;
            apLaser->ranges.assign(rMsg.ranges.begin(), rMsg.ranges.end());
            apLaser->scan_time = rMsg.scan_time;
            apLaser->time_increment = rMsg.time_increment;
        }
        bool HasData() {
            return apLaser != nullptr;
        }
        void AddError() {
            // clear output intensities and make a copy
            // with all infinities
            double r = 0.0;

            // set inf to some measurements
            for(size_t i = 0; i < apLaser->ranges.size(); ++i) {
                r = ((double) rand() / (RAND_MAX));
                if(isnan(apLaser->ranges[i]) || isinf(apLaser->ranges[i])) apLaser->ranges[i] = INFINITY;
                if(r > aNoiseProb) apLaser->ranges[i] = aMaxRange + ((double) rand() / (RAND_MAX)) * aMaxNoiseDist;
            }
        }
        sensor_msgs::LaserScan* apLaser;
        int aMaxRange;
        int aMaxNoiseDist;
        double aNoiseProb;
};

int main(int argc, char* argv[]) {
    signal(SIGSEGV, sig_handler);
    srand(time(NULL));
    ros::init(argc, argv, "laser_noiser");
    std::string ns = ros::this_node::getNamespace();
    ros::NodeHandle node_handle;
    int queue_size = 1;
    int rate = 10;

    node_handle.getParam(ns+"/lasernoiser_queue_size", queue_size);
    node_handle.getParam(ns+"/rate_lasernoiser", rate);
    ros::Rate loop_rate(rate);
    LaserNoise* pNoiser = new LaserNoise();
    ros::Subscriber laser_sub = node_handle.subscribe(ns + "/scan", queue_size, &LaserNoise::LaserCapture, pNoiser);
    ros::Publisher  laser_pub = node_handle.advertise<sensor_msgs::LaserScan>(ns + "/laser_noiser/scan", queue_size);
    std::vector<float> ranges;
    uint32_t seq = 0;

    while(ros::ok()) {
        if(pNoiser->HasData()) {
            pNoiser->AddError();
            laser_pub.publish(*(pNoiser->apLaser));
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    delete pNoiser;
    pNoiser = nullptr;
}