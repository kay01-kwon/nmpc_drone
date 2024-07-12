#include "estimator_test/load_param.h"

int main(int argc, char**argv)
{
    
    ros::init(argc, argv, "Test node");
    
    // Declare ROS NodeHandle to get yaml directory.
    ros::NodeHandle nh;
    
    load_param(nh);

    cout << simulation_time.capacity() << endl;

    assert(simulation_time.capacity() == N);


    return EXIT_SUCCESS;
}