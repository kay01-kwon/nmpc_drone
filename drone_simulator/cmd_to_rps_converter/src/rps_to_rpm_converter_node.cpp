#include "rps_to_rpm_converter/rps_to_rpm_converter.h"

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "rps_to_rpm_converter_node");
    ros::NodeHandle nh;

    RpsToRpmConverter converter(nh);

    ros::spin();

    return 0;
}