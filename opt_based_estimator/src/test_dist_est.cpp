#include <ros/ros.h>
// #include "opt_based_estimator/rotational_disturbance_estimator.hpp"
#include "simulation_model/rotational_simulation_class.hpp"
#include "utils/plot_tools.hpp"
#include <vector>

int main(int argc, char** argv)
{
    mat33_t J;

    J << 1, 0, 0,
    0, 1, 0,
    0, 0, 1;


    // PlotTool plot_tool_obj;
    RotationalSimulation rot_sim_obj(J);

    double Tf = 10;
    double rate = 100;

    vector_t M, theta_exg;

    M << 1, 0, 0;
    theta_exg << 0, 0, 0;

    size_t N = (int) Tf*100;

    vector<double> time_vec;

    vector< vector<double> > q_obs_vec;
    vector< vector<double> > w_obs_vec;

    quaternion_t q_obs;
    vector_t w_obs;

    q_obs_vec.reserve(4);
    w_obs_vec.reserve(3);

    for(size_t i = 0; i < q_obs_vec.capacity(); i++)
        q_obs_vec[i].reserve(N);
    
    for(size_t i = 0; i < w_obs_vec.capacity(); i++)
        w_obs_vec[i].reserve(N);

    for(size_t i = 0; i < N; i++)
    {
        rot_sim_obj.setInput(M, theta_exg);
        rot_sim_obj.do_simulation();

        q_obs = rot_sim_obj.get_quaternion();
        w_obs = rot_sim_obj.get_angular_velocity();
        
        time_vec.push_back(rot_sim_obj.get_time());
        
        q_obs_vec[0].push_back(q_obs.w());
        q_obs_vec[1].push_back(q_obs.x());
        q_obs_vec[2].push_back(q_obs.y());
        q_obs_vec[3].push_back(q_obs.z());

        w_obs_vec[0].push_back(w_obs(0));
        w_obs_vec[1].push_back(w_obs(1));
        w_obs_vec[2].push_back(w_obs(2));
        
    }

    plt::figure_size(1280,780);
    
    plt::subplot(3,1,1);
    plt::plot(time_vec, w_obs_vec[0]);

    plt::subplot(3,1,2);
    plt::plot(time_vec, w_obs_vec[1]);

    plt::subplot(3,1,3);
    plt::plot(time_vec, w_obs_vec[2]);

    // plt::subplot(4,1,4);
    // plt::plot(time_vec, q_obs_vec[3]);

    plt::show();

}