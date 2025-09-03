#ifndef EKF_NODE_CPP
#define EKF_NODE_CPP

#include <ros/ros.h>
#include "dist_est/ekf_dist_est.h"
#include "utils/forward_dynamics.h"
#include "utils/circular_buffer.h"

#include "ros_libcanard/hexa_actual_rpm.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Wrench.h"

#include <thread>
#include <mutex>
#include <chrono>
#include <condition_variable>

using nav_msgs::Odometry;
using geometry_msgs::Wrench;

using std::thread;
using std::mutex;
using std::condition_variable;

struct StateData{
    double time_stamp;
    Vec3d p;
    Vec3d v;
    Quatd q;
    Vec3d w;
};

struct RpmData{
    double time_stamp;
    Vec6i16 rpm;
};

class EkfNode
{

    public:

    EkfNode();

    EkfNode(ros::NodeHandle &nh);

    ~EkfNode();

    void run();

    private:

    ros::NodeHandle nh_;
    ros::Subscriber rpm_sub_;
    ros::Subscriber pose_sub_;

    ros::Timer publish_timer_;
    ros::Publisher state_pub_;
    ros::Publisher wrench_pub_;

    nav_msgs::Odometry state_msg_;

    geometry_msgs::Wrench wrench_msg_;

    double t_curr_{0.0};
    double t_prev_{0.0};

    EkfDistEst* ekf_dist_est_;
    FDynamics* converter_;

    EkfData ekf_data_;

    CircularBuffer<StateData> state_buffer_;
    CircularBuffer<RpmData> rpm_buffer_;

    thread ekf_est_thread_;

    mutex mBuf_;

    condition_variable cvBuf_;

    bool state_ready_{false};

    void rpmCallback(const ros_libcanard::hexa_actual_rpm &msg);

    void stateCallback(const Odometry &msg);

    void estimate();

    void publishCallback(const ros::TimerEvent&);

    void publishState();

    void publishWrench();

    Vec4d interpolate_vec4(const double &t0,
                       const Vec4d &v0,
                       const double &t1,
                       const Vec4d &v1,
                       const double &tm);

    void setParam(const std::string param_name, EKFParams &ekf_params);

    void setParam(const std::string param_name, MavParam &mav_param);


};


#endif