#include "LocalPlannerNode.h"

LocalPlannerNode::LocalPlannerNode() {

}

LocalPlannerNode::~LocalPlannerNode() {

}

void LocalPlannerNode::Update() {
    
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "localplannernode");
    std::unique_ptr<LocalPlannerNode> localPlannerNode = std::make_unique<LocalPlannerNode>();
    ros::spin();
}