#include "estimator_test/estimator_test_tools.h"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "Test node");
    
    // Declare ROS NodeHandle to get yaml directory.
    ros::NodeHandle nh;

    param_setup(nh);

    rpm << 0, 0, 0, 0;

    sigma_ext << 1, 2, 0;
    theta_ext << 0, 0, 0;

    mat31_t u_comp, mu_comp;
    u_comp.setZero();
    mu_comp.setZero();

    mat31_t p_state_prev, v_state_prev;
    p_state_prev.setZero();
    v_state_prev.setZero();

    double time;

    for(int i = 0; i < N; i++)
    {
        play_simulation_model(rpm, sigma_ext, theta_ext, simulation_time[i]);

        // Get state from the simulator model.
        simulation_model_ptr->get_state(p_state, v_state, 
        q_state, w_state);

        // Set control input, measured state, disturbance and simulation time
        reference_model_ptr->set_input_state_disturbance_time(u_comp, mu_comp,
        p_state_prev, v_state_prev, q_state, w_state,
        sigma_est_noisy, theta_est_noisy, simulation_time[i]);

        // Integrate the reference model (Prediction step)
        reference_model_ptr->prediction();

        // Set disturbance estimator
        disturbance_est_ptr->set_state_time(p_state_prev, p_ref, 
        v_state_prev, v_ref, q_state, q_ref, w_state, w_ref, simulation_time[i]);

        // Get state from the reference model (Prediction)
        reference_model_ptr->get_state_from_ref_model(p_ref, v_ref, q_ref, w_ref);

        p_state_prev = p_state;
        v_state_prev = v_state;

        disturbance_est_ptr->solve();

        disturbance_est_ptr->get_est_raw(sigma_est_noisy, theta_est_noisy);

        disturbance_est_ptr->get_est_filtered(sigma_est_lpf, theta_est_lpf);


        demux_simulation_state(p_state, v_state, q_state, w_state);
        demux_reference_state(p_ref, v_ref, q_ref, w_ref);
        demux_disturbance_ext(sigma_ext, theta_ext);
        demux_disturbance_est_noisy(sigma_est_noisy, theta_est_noisy);
        demux_disturbance_est_filtered(sigma_est_lpf, theta_est_lpf);

        if(i < 10)
        {
            cout << "Simulation time: " << simulation_time[i] << endl;
            cout << "Reference (x): " << p_ref(0) << endl;
            cout << "Reference (vx): " << v_ref(0) << endl;
            cout << "State (x): "<< p_state(0) << endl;
            cout << "State (vx): " << v_state(0) << endl;
            cout << "Disturbance ext (x): "<< sigma_ext(0) << endl;
            cout << "Disturbance_est_noisy (x): "<<sigma_est_noisy(0) << endl;
            cout << "Disturbance_est_lpf (x): "<<sigma_est_lpf(0) << endl;
            cout << endl;
        }


    }

    plt::plot(simulation_time, sigma_est_x);
    plt::plot(simulation_time, sigma_est_lpf_x);
    plt::plot(simulation_time, sigma_ext_x);
    plt::grid(true);
    plt::show();


    // plt::plot(simulation_time, vx_state);
    // plt::grid(true);
    // plt::show();
    

    delete simulation_model_ptr;
    delete reference_model_ptr;
    delete disturbance_est_ptr;

    return EXIT_SUCCESS;
}
