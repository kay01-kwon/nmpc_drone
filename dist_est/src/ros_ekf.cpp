#include "node/ekf_node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf_node");

    ros::NodeHandle nh;

    EkfNode ekf_node(nh);
    ekf_node.run();
    return 0;
}