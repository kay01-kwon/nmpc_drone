#include "drone_gazebo_imu_plugin/gazebo_ros_imu_plugin.h"

#include <boost/bind.hpp>
#include <chrono>


namespace gazebo{

    GZ_REGISTER_MODEL_PLUGIN(GazeboRosImuPlugin);

    GazeboRosImuPlugin::GazeboRosImuPlugin()
    : ModelPlugin(),
    ros_node_handle_ptr_(nullptr),
    acc_bias_(0, 0, 0),
    gyro_bias_(0, 0, 0)
    {}

    GazeboRosImuPlugin::~GazeboRosImuPlugin()
    {
        ros_node_handle_ptr_->shutdown();
        delete ros_node_handle_ptr_;
    }

    void GazeboRosImuPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        _model->GetName();
        model_ = _model;
        world_ = model_->GetWorld();
        gravity_W_ = world_->Gravity();
        namespace_.clear();

        if(_sdf->HasElement("robotNamespace"))
        {
            namespace_ = _sdf->Get<std::string>("robotNamespace");
            std::cout << "[gazebo_imu_plugin]"
            << "Robot namespace: " << namespace_ << "\n";
        }
        else
        {
            std::cout << "[gazebo_imu_plugin]"
            << "Please specify a robot namespace. \n";
        }

        if(_sdf->HasElement("linkName"))
        {
            link_name_ = _sdf->Get<std::string>("linkName");
            std::cout << "[gazebo_imu_plugin]"
            << " Link name: " << link_name_ << "\n";
        }
        else
        {
            std::cout << "[gazebo_imu_plugin]"
            << " Please specify a link name. \n";
            return;
        }

        link_ = model_->GetLink(link_name_);
        
        if(link_ == nullptr)
        {
            std::cout << "[gazebo_imu_plugin]"
            << " Link with name '" << link_name_ << "' not found. \n";
            return;
        }

        std::cout << "[gazebo_imu_plugin] get sdf Param \n";

        // 1. Initialize imu topic name
        if(_sdf->HasElement("imuTopic"))
        {
            imu_topic_ = _sdf->Get<std::string>("imuTopic");
            std::cout << "[gazebo_imu_plugin]"
            << " IMU topic: " << imu_topic_ << "\n";
        }
        else
        {
            std::cout << "[gazebo_imu_plugin]"
            << " Please specify an IMU topic name. \n";
            return;
        }

        // 2. Initialize IMU parameters
        // 2.1 Initialize the accelerometer noise density
        if(_sdf->HasElement("accNoiseDensity"))
        {
            imu_params_.acc_noise_density = 
            _sdf->Get<double>("accNoiseDensity");
            std::cout << "[gazebo_imu_plugin]"
            << " Accelerometer noise density: "
            << imu_params_.acc_noise_density << "\n";
        }
        else
        {
            std::cout << "[gazebo_imu_plugin]"
            << " Using default accelerometer noise density: "
            << DefaultAccNoiseDensity << "\n";
        }


        // 2.2 Initialize the accelerometer random walk
        if(_sdf->HasElement("accRandomWalk"))
        {
            imu_params_.acc_random_walk = 
            _sdf->Get<double>("accRandomWalk");
            std::cout << "[gazebo_imu_plugin]"
            << " Accelerometer random walk: "
            << imu_params_.acc_random_walk << "\n";
        }
        else
        {
            std::cout << "[gazebo_imu_plugin]"
            << " Using default accelerometer random walk: "
            << DefaultAccRandomWalk << "\n";
        }

        // 2.3 Initialize the gyro noise density
        if(_sdf->HasElement("gyroNoiseDensity"))
        {
            imu_params_.gyro_noise_density = 
            _sdf->Get<double>("gyroNoiseDensity");
            std::cout << "[gazebo_imu_plugin]"
            << " Gyroscope noise density: "
            << imu_params_.gyro_noise_density << "\n";
        }
        else
        {
            std::cout << "[gazebo_imu_plugin]"
            << " Using default gyroscope noise density: "
            << DefaultGyroNoiseDensity << "\n";
        }

        // 2.4 Initialize the gyro random walk
        if(_sdf->HasElement("gyroRandomWalk"))
        {
            imu_params_.gyro_random_walk = 
            _sdf->Get<double>("gyroRandomWalk");
            std::cout << "[gazebo_imu_plugin]"
            << " Gyroscope random walk: "
            << imu_params_.gyro_random_walk << "\n";
        }
        else
        {
            std::cout << "[gazebo_imu_plugin]"
            << " Using default gyroscope random walk: "
            << DefaultGyroRandomWalk << "\n";
        }

        if(_sdf->HasElement("accFullScale"))
        {
            acc_full_scale_ = 
            _sdf->Get<double>("accFullScale");
            std::cout << "[gazebo_imu_plugin]"
            << " Accelerometer full scale: "
            << acc_full_scale_ << "\n";
        }
        else
        {
            acc_full_scale_ = 32.0; // Default value
            std::cout << "[gazebo_imu_plugin]"
            << " Using default accelerometer full scale: "
            << acc_full_scale_ << "\n";
        }

        if(_sdf->HasElement("gyroFullScale"))
        {
            gyro_full_scale_ = 
            _sdf->Get<double>("gyroFullScale");
            std::cout << "[gazebo_imu_plugin]"
            << " Gyroscope full scale: "
            << gyro_full_scale_ << "\n";
        }
        else
        {
            gyro_full_scale_ = 4000.0; // Default value
            std::cout << "[gazebo_imu_plugin]"
            << " Using default gyroscope full scale: "
            << gyro_full_scale_ << "\n";
        }
        
        frame_id_ = link_name_;

        std::cout << "[gazebo_imu_plugin] ROS node initialized \n";

        // Initalize the transport node
        ros_node_handle_ptr_ = new ros::NodeHandle(this->namespace_);
        
        std::cout << "[gazebo_imu_plugin] Allocate node on the heap\n";

        ros_imu_publisher_ = ros_node_handle_ptr_->
        advertise<sensor_msgs::Imu>
        ("/" + namespace_ + "/" + imu_topic_, 1);

        std::cout<< "[gazebo_imu_plugin] IMU publisher initialized \n";

        ros_imu_msg_.header.frame_id = frame_id_;

        // Connect the update event
        std::cout << "[gazebo_imu_plugin] Connecting to update event \n";

        this->updateConnection_ = 
        event::Events::ConnectWorldUpdateBegin(
            boost::bind(&GazeboRosImuPlugin::OnUpdate, this)
        );

        last_time_ = world_->SimTime();

        ROS_INFO_STREAM(
            "[gazebo_imu_plugin] IMU plugin loaded for model: "
            << model_->GetName() << " on link: " << link_name_
        );

    }

    void GazeboRosImuPlugin::OnUpdate()
    {
        common::Time current_time = world_->SimTime();
        double dt = (current_time - last_time_).Double();

        // Get the transformation matrix from world to the link
        ignition::math::Pose3d tf_w_I = link_->WorldPose();
        ignition::math::Quaterniond quat_w_I = tf_w_I.Rot();

        // Get the linear acceleration of the imu link
        ignition::math::Vector3d acc_I = 
        link_->RelativeLinearAccel() - 
        quat_w_I.RotateVectorReverse(gravity_W_);

        // Get the angular velocity of the imu link
        ignition::math::Vector3d ang_vec_I = 
        link_->RelativeAngularVel();

        Eigen::Vector3d linear_acc(
            acc_I.X(),
            acc_I.Y(),
            acc_I.Z()
        );

        Eigen::Vector3d angular_vel(
            ang_vec_I.X(),
            ang_vec_I.Y(),
            ang_vec_I.Z()
        );

        // Add noise and bias to the IMU measurement
        AddNoise(&linear_acc, &angular_vel, dt);

        ros_imu_msg_.header.stamp.sec = current_time.sec;
        ros_imu_msg_.header.stamp.nsec = current_time.nsec;

        ros_imu_msg_.orientation.w = quat_w_I.W();
        ros_imu_msg_.orientation.x = quat_w_I.X();
        ros_imu_msg_.orientation.y = quat_w_I.Y();
        ros_imu_msg_.orientation.z = quat_w_I.Z();

        for(size_t i = 0; i < 3; i++)
        {
            // Scale the accelerometer and gyro values
            if (linear_acc[i] > acc_full_scale_*9.81)
            {
                linear_acc[i] = acc_full_scale_*9.81;
            }
            else if (linear_acc[i] < -acc_full_scale_*9.81)
            {
                linear_acc[i] = -acc_full_scale_*9.81;
            }

            if(angular_vel[i] > gyro_full_scale_*M_PI/180.0)
            {
                angular_vel[i] = gyro_full_scale_;
            }
            else if (angular_vel[i] < -gyro_full_scale_*M_PI/180.0)
            {
                angular_vel[i] = -gyro_full_scale_;
            }

        }

        ros_imu_msg_.linear_acceleration.x = linear_acc[0];
        ros_imu_msg_.linear_acceleration.y = linear_acc[1];
        ros_imu_msg_.linear_acceleration.z = linear_acc[2];

        ros_imu_msg_.angular_velocity.x = angular_vel[0];
        ros_imu_msg_.angular_velocity.y = angular_vel[1];
        ros_imu_msg_.angular_velocity.z = angular_vel[2];

        // Publish the ROS IMU message
        ros_imu_publisher_.publish(ros_imu_msg_);
        ros::spinOnce();

        last_time_ = current_time;

    }

    void GazeboRosImuPlugin::AddNoise(
    Eigen::Vector3d* linear_acc,
    Eigen::Vector3d* angular_vel,
    double dt)
    {
        GZ_ASSERT(linear_acc != nullptr,
        "linear_acc cannot be null");
        GZ_ASSERT(angular_vel != nullptr,
        "angular_vel cannot be null");
        GZ_ASSERT(dt > 0.0,
        "dt must be greater than 0.0");

        // According to the imu noise model proposed by Kalibr
        // implemented the noise and bias model for the IMU

        for(size_t i = 0; i < 3; i++)
        {
            // Add random walk to the bias
            acc_bias_[i] += 
            imu_params_.acc_random_walk 
            * normal_dist_(gen_)*sqrt(dt);

            gyro_bias_[i] +=
            imu_params_.gyro_random_walk
            * normal_dist_(gen_)*sqrt(dt);
            
            // Add noise and bias to the accelerometer and gyro
            (*linear_acc)[i] = 
            (*linear_acc)[i] + acc_bias_[i]
            + imu_params_.acc_noise_density
            * normal_dist_(gen_)/sqrt(dt);
            
            (*angular_vel)[i] = 
            (*angular_vel)[i] + gyro_bias_[i]
            + imu_params_.gyro_noise_density
            * normal_dist_(gen_)/sqrt(dt);
        }
    }

}