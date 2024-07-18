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

    // sigma_ext << 1, 2, 0;
    // theta_ext << 1, 0, 0;

    mat31_t u_comp, mu_comp;
    u_comp.setZero();
    mu_comp.setZero();


    assert(simulation_time.capacity() == N);

    for(int i = 0; i < N; i++)
    {
        sigma_ext << 1*cos(0.3*simulation_time[i]),
        2*cos(1*simulation_time[i]),
        0;

        theta_ext << 3*cos(0.4*simulation_time[i]),
        3*sin(simulation_time[i]),
        4*cos(simulation_time[i]);
        

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
            print_state(simulation_time[i], p_state, v_state, q_state, w_state,
            p_ref, v_ref, q_ref, w_ref, theta_ext, sigma_ext, theta_est_lpf,
            sigma_est_lpf, theta_est_noisy, sigma_est_noisy);
        }

    }

    cout << "At final time step" << endl;
    print_state(simulation_time[N-1], p_state, v_state, q_state, w_state,
    p_ref, v_ref, q_ref, w_ref, theta_ext, sigma_ext, theta_est_lpf,
    sigma_est_lpf, theta_est_noisy, sigma_est_noisy);


    keywords_setup(line_width, font_size);

    // ticks_setup(Tf, 20, 0, 5, 5);    
    // y_label = "x";
    // data1_label = "x ref";
    // data2_label = "x state";

    // plot_data(simulation_time, y_label, 
    // data1_label, data2_label ,x_ref, x_state);
    // plt::show();

    // ticks_setup(Tf, 20, 0, 5, 5);
    
    // y_label = "y";
    // data1_label = "y ref";
    // data2_label = "y state";

    // plot_data(simulation_time, y_label, 
    // data1_label, data2_label ,y_ref, y_state);
    // plt::show();


    // ticks_setup(Tf, 20, 0, 5, 5);    
    // y_label = "z";
    // data1_label = "z ref";
    // data2_label = "z state";

    // plot_data(simulation_time, y_label, 
    // data1_label, data2_label ,z_ref, z_state);
    // plt::show();

    // ticks_setup(Tf, 2, -2, 5, 5);
    
    // y_label = "$d_{x}$";
    // data1_label = "$d_{est}$";
    // data2_label = "$d_{gnd}$";

    // plot_data(simulation_time, y_label, 
    // data1_label, data2_label ,sigma_est_x, sigma_ext_x);
    // plt::show();


    ticks_setup(Tf, 2, -2, 5, 5);
    
    y_label = "$\theta_{x}$";
    data1_label = "$\theta_{est}$";
    data2_label = "$\theta_{gnd}$";

    plot_data(simulation_time, y_label, 
    data1_label, data2_label ,theta_est_x, theta_ext_x);
    plt::show();

    ticks_setup(Tf, 2, -2, 5, 5);
    
    y_label = "$\theta_{y}$";
    data1_label = "$\theta_{est}$";
    data2_label = "$\theta_{gnd}$";

    plot_data(simulation_time, y_label, 
    data1_label, data2_label ,theta_est_y, theta_ext_y);
    plt::show();

    ticks_setup(Tf, 2, -2, 5, 5);
    
    y_label = "$\theta_{z}$";
    data1_label = "$\theta_{est}$";
    data2_label = "$\theta_{gnd}$";

    plot_data(simulation_time, y_label, 
    data1_label, data2_label ,theta_est_z, theta_ext_z);
    plt::show();

    delete simulation_model_ptr;
    delete reference_model_ptr;
    delete disturbance_est_ptr;

    return EXIT_SUCCESS;
}
