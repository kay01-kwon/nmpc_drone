#include "test_dist_est.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Test_Dist_EST");

    NodeHandle nh;

    cout << "Node handler created" << endl;

    set_parameter(nh);

    // PlotTool plot_tool_obj = PlotTool(VECTOR_T_DIM, N, 
    // x_tick_size, y_tick_size);

    // plot_tool_obj.set_keywords(line_width, label_font_size, tick_font_size);

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

        // plot_tool_obj.add_data(curr_time, theta_exg, theta_est);

    }


    plt::figure(1);
    plt::figure_size(1000,1000);

    cout<<"Plotting..."<<endl;
    
    // plt::subplot(3,1,1);
    
    title_name = "$θ_{x} - t$";
    y_label_name = "$θ_{x} (Nm)$";
    data1_name = "$θ_{x,est}$";
    data2_name = "$θ_{x,true}$";
    // plot_tool_obj.plot_data(title_name, y_label_name, 
    // data1_name, data2_name, idx[0]);
    // plt::title(title_name, {{"fontsize","15"}});
    
    plt::plot(time_vec, theta_est_vec[0], { {"label",data1_name},
    {"color", "red"}, {"linewidth", "4"}});
    cout << "No problem 2 here" << endl;
    plt::plot(time_vec, theta_true_vec[0], 
    { {"label",data2_name},
    {"color","limegreen"}, {"linewidth", "4"},
    {"linestyle","--"}});
    cout << "No problem 3 here" << endl;
    plt::xlabel("time (s)",{{"fontsize","15"}});
    plt::ylabel(y_label_name,{{"fontsize","15"}});
    plt::legend({{"fontsize","15"}});
    plt::grid(true);

    plt::show();

    cout << "Plotting 2..." << endl;
    plt::figure(2);
    plt::figure_size(1000,1000);


    // plt::subplot(3,1,2);
    title_name = "$θ_{y} - t$";
    y_label_name = "$θ_{y} (Nm)$";
    data1_name = "$θ_{y,est}$";
    data2_name = "$θ_{y,true}$";
    // plot_tool_obj.plot_data(title_name, y_label_name, 
    // data1_name, data2_name, idx[0]);
    // plt::title(title_name, {{"fontsize","20"}});
    
    plt::plot(time_vec, theta_est_vec[1], { {"label",data1_name},
    {"color", "red"}, {"linewidth", "4"}});

    plt::plot(time_vec, theta_true_vec[1], 
    { {"label",data2_name},
    {"color","limegreen"}, {"linewidth", "4"},
    {"linestyle","--"}});
    
    plt::xlabel("time (s)",{{"fontsize","15"}});
    plt::ylabel(y_label_name,{{"fontsize","15"}});
    plt::legend({{"fontsize","15"}});
    plt::grid(true);

    plt::show();

    // plt::subplot(3,1,3);

    plt::figure(3);
    plt::figure_size(1000,1000);
    title_name = "$θ_{z} - t$";
    y_label_name = "$θ_{z} (Nm)$";
    data1_name = "$θ_{z,est}$";
    data2_name = "$θ_{z,true}$";
    // plot_tool_obj.plot_data(title_name, y_label_name, 
    // data1_name, data2_name, idx[0]);
    // plt::title(title_name, {{"fontsize","20"}});
    
    plt::plot(time_vec, theta_est_vec[2], { {"label",data1_name},
    {"color", "red"}, {"linewidth", "4"}});

    plt::plot(time_vec, theta_true_vec[2], 
    { {"label",data2_name},
    {"color","limegreen"}, {"linewidth", "4"},
    {"linestyle","--"}});
    
    plt::xlabel("time (s)",{{"fontsize","15"}});
    plt::ylabel(y_label_name,{{"fontsize","15"}});
    plt::legend({{"fontsize","15"}});
    plt::grid(true);

    // plt::xticks(time_vec, {}, {{"fontsize","15"}});
    // plt::yticks(theta_est_vec[0], {}, {{"fontsize","15"}});

    plt::show();

    plt::save("/home/kay/Pictures/test.png", 600);

}