#include "estimator_test/estimator_test_tools.h"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "Test node");
    
    // Declare ROS NodeHandle to get yaml directory.
    ros::NodeHandle nh;

    param_setup(nh);

    rpm << 0, 0, 0, 0;

    sigma_ext << 0, 0, 0.01;
    theta_ext << 0, 0, 0;

    assert(sigma_ext.size() == 3);
    assert(theta_ext.size() == 3);

    for(size_t i = 1; i < N; i++)
    {
        cout<<"Simulation time: "<<simulation_time[i]<<endl;
        play_simulation_model(rpm, sigma_ext, theta_ext, simulation_time[i]);

        // Get state from the simulator model.
        simulation_model_ptr->get_state(p_state[i], v_state[i], 
        q_state[i], w_state[i]);

        cout<<"Position: ";
        for(size_t j = 0; j < 3; j++)
            cout<<p_state[i](j)<<endl;

    }

    return EXIT_SUCCESS;
}
