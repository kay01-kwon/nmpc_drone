#include "estimator_test/load_param.h"

#ifndef VARIABLE_H_
#include "estimator_test/variable_def.h"
#endif

int main(int argc, char**argv)
{
    
    ros::init(argc, argv, "Test node");
    
    // Declare ROS NodeHandle to get yaml directory.
    ros::NodeHandle nh;
    
    load_param(nh);




    return EXIT_SUCCESS;
}