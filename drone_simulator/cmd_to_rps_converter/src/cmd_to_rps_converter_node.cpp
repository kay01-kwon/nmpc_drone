#include "cmd_to_rps_converter/cmd_to_rps_converter.h"

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "cmd_to_rps_converter_node");
    ros::NodeHandle nh;

    CmdToRpsConverter converter(nh);
    converter.run();

    return 0;
}