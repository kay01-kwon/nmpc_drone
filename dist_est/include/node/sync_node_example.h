#ifndef SYNC_NODE_EXAMPLE_H_
#define SYNC_NODE_EXAMPLE_H_

#include <ros/ros.h>
#include <thread>
#include <mutex>
#include <condition_variable>

#include <deque>
#include <vector>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <nav_msgs/Odometry.h>
#include <ros_libcanard/hexa_actual_rpm.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Float64.h>

using nav_msgs::Odometry;
using ros_libcanard::hexa_actual_rpm;
using geometry_msgs::Wrench;
using std_msgs::Float64;

using std::deque;
using std::vector;

using std::thread;
using std::mutex;
using std::condition_variable;


struct StateData
{
    double time_stamp;
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Quaterniond q;
    Eigen::Vector3d w;
};

struct RpmData
{
    double time_stamp;
    Eigen::Matrix<double, 6, 1> rpm;
};

struct WrenchData
{
    double time_stamp;
    Eigen::Vector3d force;
    Eigen::Vector3d torque;
};

class SyncNodeExample {


    public:
        
        explicit SyncNodeExample(ros::NodeHandle &nh);

        ~SyncNodeExample();

        void run();

    private:

        ros::NodeHandle nh_;
        
        ros::Subscriber state_sub_;
        ros::Subscriber rpm_sub_;
        ros::Subscriber wrench_sub_;
        
        ros::Timer publish_timer_;
        ros::Publisher time_sync_pub_;

        thread sync_thread_;
        thread process_thread_;

        mutex mBuf_;
        condition_variable cv_;

        deque<StateData> state_buffer_;
        deque<RpmData> rpm_buffer_;
        deque<WrenchData> wrench_buffer_;

        bool is_first_run_{true};

        bool fresh_data_[2];

        double t_curr_{0.0};
        double t_prev_{-0.01};
        double period_{0.01};

        double time_latest_[2]; // 1: state, 2: rpm
        double time_out_[2];
        double last_rx_wall_[2];

        void stateCallback(const Odometry::ConstPtr &state_msg);
        void rpmCallback(const hexa_actual_rpm::ConstPtr &rpm_msg);

        void aggregate_thread();

        void process_thread();

        bool freshByTTL(int i, double now_wall);

        double watermark_event_time();

        void publishCallback(const ros::TimerEvent&);

};


#endif