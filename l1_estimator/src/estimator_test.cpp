#include "estimator_test/estimator_test_tools.h"
#include <assert.h>
#include "estimator_test/plot_tools_for_estimator_test.h"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "Test node");
    
    // Declare ROS NodeHandle to get yaml directory.
    ros::NodeHandle nh;

    param_setup(nh);

    rpm << 0, 0, 0, 0;

    sigma_ext << 1, 2, 0;
    theta_ext << 1, 0, 0;

    mat31_t u_comp, mu_comp;
    u_comp.setZero();
    mu_comp.setZero();


    assert(simulation_time.capacity() == N);

    for(int i = 0; i < N; i++)
    {
        play_simulation_model(rpm, sigma_ext, theta_ext, simulation_time[i]);

        // Get state from the simulator model.
        simulation_model_ptr->get_state(p_state, v_state, 
        q_state, w_state);

        // Set control input, measured state, disturbance and simulation time
        reference_model_ptr->set_input_state_disturbance_time(u_comp, mu_comp,
        p_state_prev, v_state_prev, q_state_prev, w_state_prev,
        sigma_est_noisy, theta_est_noisy, simulation_time[i]);

        // Integrate the reference model (Prediction step)
        reference_model_ptr->prediction();

        // Set disturbance estimator
        disturbance_est_ptr->set_state_time(p_state_prev, p_ref, 
        v_state_prev, v_ref, q_state_prev, q_ref, w_state_prev, w_ref, simulation_time[i]);

        // Get state from the reference model (Prediction)
        reference_model_ptr->get_state_from_ref_model(p_ref, v_ref, q_ref, w_ref);

        disturbance_est_ptr->solve();

        disturbance_est_ptr->get_est_raw(sigma_est_noisy, theta_est_noisy);

        disturbance_est_ptr->get_est_filtered(sigma_est_lpf, theta_est_lpf);

        p_state_prev = p_state;
        v_state_prev = v_state;
        q_state_prev = q_state;
        w_state_prev = w_state;

        demux_simulation_state(p_state, v_state, q_state, w_state);
        demux_reference_state(p_ref, v_ref, q_ref, w_ref);
        demux_disturbance_ext(sigma_ext, theta_ext);
        demux_disturbance_est_noisy(sigma_est_noisy, theta_est_noisy);
        demux_disturbance_est_filtered(sigma_est_lpf, theta_est_lpf);

        if(i <= 10)
        {
            cout << "Simulation time: " << simulation_time[i] << endl;

            cout << "State (p): " << p_state(0) 
            << " " << p_state(1) << " " << p_state(2) << endl;
            
            cout << "State (v): " << v_state(0) 
            << " " << v_state(1) << " " << v_state(2) << endl;

            cout << "State (q): " << q_state.w() 
            << " " << q_state.x() << " " << q_state.y() <<
            " " << q_state.z() << endl;

            cout << "State (w): " << w_state(0) 
            << " " << w_state(1) << " " << w_state(2) << endl;



            cout << "Reference (p): " << p_ref(0) 
            << " " << p_ref(1) << " " << p_ref(2) << endl;
            
            cout << "Reference (v): " << v_ref(0) 
            << " " << v_ref(1) << " " << v_ref(2) << endl;

            cout << "Reference (q): " << q_ref.w() 
            << " " << q_ref.x() << " " << q_ref.y() <<
            " " << q_ref.z() << endl;

            cout << "Reference (w): " << w_ref(0) 
            << " " << w_ref(1) << " " << w_ref(2) << endl;

            cout<<endl;

            cout << "Translational disturbance info" << endl;
            cout << "Disturbance ext: "<< sigma_ext(0) 
            <<" " << sigma_ext(1) << " " << sigma_ext(2) << endl;

            cout << "Disturbance est noisy: "<< sigma_est_noisy(0) 
            <<" " << sigma_est_noisy(1) << " " << sigma_est_noisy(2) << endl;

            cout << "Disturbance est filtered: "<< sigma_est_lpf(0) 
            <<" " << sigma_est_lpf(1) << " " << sigma_est_lpf(2) << endl;

            cout<<endl;

            cout << "Orientational disturbance info" << endl;
            cout << "Disturbance ext: "<< theta_ext(0) 
            <<" " << theta_ext(1) << " " << theta_ext(2) << endl;

            cout << "Disturbance est noisy: "<< theta_est_noisy(0) 
            <<" " << theta_est_noisy(1) << " " << theta_est_noisy(2) << endl;

            cout << "Disturbance est filtered: "<< theta_est_lpf(0) 
            <<" " << theta_est_lpf(1) << " " << theta_est_lpf(2) << endl;

            cout << endl;
        }


    }


    cout << "At final time step" << endl;

    cout << "Translational disturbance info" << endl;
    cout << "Disturbance ext: "<< sigma_ext(0) 
    <<" " << sigma_ext(1) << " " << sigma_ext(2) << endl;

    cout << "Disturbance est noisy: "<< sigma_est_noisy(0) 
    <<" " << sigma_est_noisy(1) << " " << sigma_est_noisy(2) << endl;

    cout << "Disturbance est filtered: "<< sigma_est_lpf(0) 
    <<" " << sigma_est_lpf(1) << " " << sigma_est_lpf(2) << endl;

    cout<<endl;

    cout << "Orientational disturbance info" << endl;
    cout << "Disturbance ext: "<< theta_ext(0) 
    <<" " << theta_ext(1) << " " << theta_ext(2) << endl;

    cout << "Disturbance est noisy: "<< theta_est_noisy(0) 
    <<" " << theta_est_noisy(1) << " " << theta_est_noisy(2) << endl;

    cout << "Disturbance est filtered: "<< theta_est_lpf(0) 
    <<" " << theta_est_lpf(1) << " " << theta_est_lpf(2) << endl;

    plt::plot(simulation_time, theta_est_x);
    plt::plot(simulation_time, theta_est_lpf_x);
    plt::plot(simulation_time, theta_ext_x);
    plt::grid(true);
    plt::show();

    

    delete simulation_model_ptr;
    delete reference_model_ptr;
    delete disturbance_est_ptr;

    return EXIT_SUCCESS;
}
