#include "node/sync_node_example.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_sync_example");
    ros::NodeHandle nh("~");

    SyncNodeExample sync_node_example(nh);
    sync_node_example.run();

    return 0;
}