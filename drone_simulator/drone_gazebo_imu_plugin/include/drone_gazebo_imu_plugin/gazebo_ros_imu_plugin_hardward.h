#ifndef DRONE_GAZEBO_ROS_IMU_PLUGIN_HARDWARE_H
#define DRONE_GAZEBO_ROS_IMU_PLUGIN_HARDWARE_H


#include <random>
#include <iostream>

#include <Eigen/Core>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

namespace gazebo{


    class GazeboRosImuPluginHW : public ModelPlugin
    {
        public:

        GazeboRosImuPluginHW();
        ~GazeboRosImuPluginHW();

        protected:
            // Load the plugin
            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);


            // This callback is called by Gazebo 
            // when the plugin is updated
            void OnUpdate();

        private:

            void callback(const sensor_msgs::Imu::ConstPtr& imu_msg);

            std::string namespace_;
            std::string imu_topic_;

            std::string frame_id_;
            std::string link_name_;

            ros::NodeHandle *ros_node_handle_ptr_;
            ros::Publisher ros_imu_publisher_;
            ros::Subscriber ros_imu_subscriber_;

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

            Eigen::Vector3d acc_mean_;
            Eigen::Vector3d gyro_mean_;

            double acc_full_scale_{32.0};
            double gyro_full_scale_{4000.0};

    };
}



#endif  //DRONE_GAZEBO_ROS_IMU_PLUGIN_H