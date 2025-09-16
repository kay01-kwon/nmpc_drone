#include "node/hgdo_node_v2.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hgdo_node");

    ros::NodeHandle nh;

    HgdoNode2 hgdo_node(nh);

    hgdo_node.run();

    return 0;
}