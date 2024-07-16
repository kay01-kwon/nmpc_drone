#include "estimator_test/estimator_test_tools.h"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "Test node");
    
    // Declare ROS NodeHandle to get yaml directory.
    ros::NodeHandle nh;

    param_setup(nh);

    rpm << 0, 0, 0, 0;

    sigma_ext << 0, 0, 0;
    theta_ext << 0, 0, 0;

    assert(sigma_ext.size() == 3);
    assert(theta_ext.size() == 3);

    mat31_t u_comp, mu_comp;
    u_comp.setZero();
    mu_comp.setZero();


    for(size_t i = 0; i < N; i++)
    {
        play_simulation_model(rpm, sigma_ext, theta_ext, simulation_time[i]);

        // Get state from the simulator model.
        simulation_model_ptr->get_state(p_state[i], v_state[i], 
        q_state[i], w_state[i]);

        reference_model_ptr->set_input_state_disturbance_time(u_comp, mu_comp,
        p_state[i], v_state[i], q_state[i], w_state[i],
        sigma_est_noisy, theta_est_noisy, simulation_time[i]);
        reference_model_ptr->integrate();

        reference_model_ptr->get_state_from_ref_model(p_ref[i], v_ref[i], q_ref[i], w_ref[i]);


        demux_simulation_state(p_state[i], v_state[i], q_state[i], w_state[i]);
        demux_reference_state(p_ref[i], v_ref[i], q_ref[i], w_ref[i]);

    }


    plt::plot(simulation_time, z_state);
    plt::grid(true);
    plt::show();

    delete simulation_model_ptr;
    delete reference_model_ptr;
    delete disturbance_est_ptr;

    return EXIT_SUCCESS;
}
