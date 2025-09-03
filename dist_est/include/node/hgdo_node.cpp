#include "hgdo_node.h"

HgdoNode::HgdoNode()
{
    // Default constructor
}

HgdoNode::HgdoNode(ros::NodeHandle &nh)
:nh_(nh)
{
    MavParam mav_param;
    HgdoParam hgdo_param;

    // Get publish rate from parameter server
    double publish_rate;
    std::string node_name = ros::this_node::getName();
    double duration = 1.0/publish_rate;
    ROS_INFO("HGDO publish rate: %f Hz", publish_rate);

    // Set hgdo parameters
    std::string hgdo_param_name = "/hgdo_param";
    setParam(hgdo_param_name, hgdo_param);

    // Set mav parameters
    std::string nominal_param_name = "/nominal_param";
    setParam(nominal_param_name, mav_param);

    // Initialize converter
    converter_ = new FDynamics(mav_param);

    // Initialize HGDO
    hgdo_dist_est_ = new HGDO(mav_param, hgdo_param);

    f_tau_ext_.setZero();

}

HgdoNode::~HgdoNode()
{
    if (hgdo_est_thread_.joinable()) {
        hgdo_est_thread_.join();
    }
    if(hgdo_dist_est_ != nullptr) {
        delete hgdo_dist_est_;
    }
    if(converter_ != nullptr) {
        delete converter_;
    }
}

void HgdoNode::run()
{
    ros::spin();
}

void HgdoNode::rpmCallback(const ros_libcanard::hexa_actual_rpm &rpm_msg)
{

}

void HgdoNode::stateCallback(const Odometry &state_msg)
{

}

void HgdoNode::estimate()
{

}

void HgdoNode::publishCallback(const ros::TimerEvent&)
{

}

void HgdoNode::publishWrench()
{

}

void HgdoNode::setParam(const std::string param_name, MavParam &mav_param)
{
    std::string node_name = ros::this_node::getName();
    std::string param_total_name = node_name + param_name;

    double m;
    double moi_xx, moi_yy, moi_zz;

    nh_.param(param_total_name + "/m", m, 2.9);
    nh_.param(param_total_name + "/moi/xx", moi_xx, 0.052);
    nh_.param(param_total_name + "/moi/yy", moi_yy, 0.052);
    nh_.param(param_total_name + "/moi/zz", moi_zz, 0.080);
    nh_.param(param_total_name + "/l", mav_param.l, 0.265);
    nh_.param(param_total_name + "/C_T", mav_param.C_T, 1.465e-07);
    nh_.param(param_total_name + "/k_m", mav_param.k_m, 0.01569);

    mav_param.m = m;  // Mass
    mav_param.J << moi_xx, 0.0, 0.0,
                   0.0, moi_yy, 0.0,
                   0.0, 0.0, moi_zz;  // Inertia matrix

    ROS_INFO("MavParam set: m = %f, moi = [%f, %f, %f]", 
             mav_param.m, moi_xx, moi_yy, moi_zz);

    ROS_INFO("MavParam set: l = %f, C_T = %e, k_m = %f", 
             mav_param.l, mav_param.C_T, mav_param.k_m);

}

void HgdoNode::setParam(const std::string param_name, HgdoParam &hgdo_param)
{
    std::string node_name = ros::this_node::getName();
    std::string param_total_name = node_name + param_name;

    double eps_f, eps_tau;

    nh_.param(param_total_name + "/eps_f", eps_f, 0.01);
    nh_.param(param_total_name + "/eps_tau", eps_tau, 0.01);

    hgdo_param.eps_f = eps_f;
    hgdo_param.eps_tau = eps_tau;

    ROS_INFO("HgdoParam set: eps_f = %f, eps_tau = %f", 
             hgdo_param.eps_f, hgdo_param.eps_tau);
}