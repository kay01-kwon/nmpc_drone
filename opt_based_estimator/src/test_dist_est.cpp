#include "test_dist_est.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Test_Dist_EST");

    NodeHandle nh;

    cout << "Node handler created" << endl;

    set_parameter(nh);

    for(size_t i = 0; i < N; i++)
    {
        M << 0, 0, 0;
        theta_exg << 5*cos(curr_time), 1*cos(curr_time), 2*cos(curr_time);
        
        // Set control input and disturbance
        // and then simulate it.
        rot_sim_ptr->setInput(M, theta_exg);
        rot_sim_ptr->do_simulation();
        
        // Get time, quaternion and angular velocity state variables, respectively
        curr_time = rot_sim_ptr->get_time();
        q_obs = rot_sim_ptr->get_quaternion();
        w_obs = rot_sim_ptr->get_angular_velocity();

        rot_dist_est_ptr->set_input(M, theta_est);
        rot_dist_est_ptr->set_meas_state(q_obs, w_obs);
        rot_dist_est_ptr->set_time(curr_time);

        rot_dist_est_ptr->solve();

        theta_est = rot_dist_est_ptr->get_est_dist();

        prev_time = curr_time;

        time_vec.push_back(curr_time);
        push_back_vector(q_obs, q_obs_vec);
        push_back_vector(w_obs, w_obs_vec);
        push_back_vector(theta_exg, theta_true_vec);
        push_back_vector(theta_est, theta_est_vec);

    }

    plt::figure_size(3500,2000);
    plt::subplot(3,1,1);
    title_name = "$θ_{x} - t$";
    y_label_name = "$θ_{x} (Nm)$";
    data1_name = "$θ_{x,est} - t$";
    data2_name = "$θ_{x,true} - t$";



    plt::show();

}