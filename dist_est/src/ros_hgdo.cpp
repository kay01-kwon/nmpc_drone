#include "node/hgdo_node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hgdo_node");

    ros::NodeHandle nh;

    HgdoNode hgdo_node(nh);

    hgdo_node.run();

    return 0;
}