#ifndef DRONE_GAZEBO_ROS_IMU_PLUGIN_H
#define DRONE_GAZEBO_ROS_IMU_PLUGIN_H


#include <random>
#include <iostream>

#include <Eigen/Core>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

namespace gazebo{

    // Default values for IMU parameters

    // Pixhawk IMU default values

    static constexpr double DefaultAccNoiseDensity = 0.0070708;

    static constexpr double DefaultAccRandomWalk = 3.6921e-4;

    static constexpr double DefaultGyroNoiseDensity = 0.0004326;

    static constexpr double DefaultGyroRandomWalk = 1.2810e-5;

    static constexpr double DefaultGravityMagnitude = 9.81;

    struct ImuParameters{

        // Accelerometer noise density [m/s/s/Hz]
        double acc_noise_density;

        // Accelerometer bias random walk [m/s/s/sqrt(Hz)]
        double acc_random_walk;

        // Gyroscope noise density  [rad/s/Hz]
        double gyro_noise_density;

        // Gyroscope bias random walk [rad/s/s/sqrt(Hz)]
        double gyro_random_walk;

        double gravitiy_magnitude;

        // Default constructor initializing parameters to default values
        ImuParameters()
        : acc_noise_density(DefaultAccNoiseDensity),
          acc_random_walk(DefaultAccRandomWalk),
          gyro_noise_density(DefaultGyroNoiseDensity),
          gyro_random_walk(DefaultGyroRandomWalk),
          gravitiy_magnitude(DefaultGravityMagnitude)
        {}
    };

    class GazeboRosImuPlugin : public ModelPlugin
    {
        public:

        GazeboRosImuPlugin();
        ~GazeboRosImuPlugin();

        protected:
            // Load the plugin
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

            // Add noise to the IMU measurements
            void AddNoise(
                Eigen::Vector3d* linear_acceleration,
                Eigen::Vector3d* angular_velocity,
                double dt
            );

            // This callback is called by Gazebo 
            // when the plugin is updated
            void OnUpdate();

        private:

            std::string namespace_;
            std::string imu_topic_;

            std::string frame_id_;
            std::string link_name_;

            ros::NodeHandle *ros_node_handle_ptr_;
            ros::Publisher ros_imu_publisher_;

            // Pointer to world
            physics::WorldPtr world_;
            
            // Pointer to the model
            physics::ModelPtr model_;

            // Pointer to the link
            physics::LinkPtr link_;

            // Connection Pointer to the update event
            event::ConnectionPtr updateConnection_;

            common::Time last_time_;

            sensor_msgs::Imu ros_imu_msg_;

            ignition::math::Vector3d gravity_W_;

            Eigen::Vector3d acc_bias_;
            Eigen::Vector3d gyro_bias_;

            double acc_full_scale_{32.0};
            double gyro_full_scale_{4000.0};

            ImuParameters imu_params_;

            std::random_device rd_{};
            std::mt19937 gen_{rd_()};

            std::normal_distribution<double> normal_dist_{0.0, 1.0};

    };
}



#endif  //DRONE_GAZEBO_ROS_IMU_PLUGIN_H