# nmpc_drone

It is of paramount to take frame into account when employing controller.

Frame info:

/hummingbird/ground_truth/odometry

position - world frame,

linear velocity - body frame (Child frame)

quaternion - body to world frame

angular velocity - body frame

By transforming the linear velocity from body frame to world frame, the position controller stability issue is fixed when it reaches the desired yaw.

# nmpc node for Hummingbird (Quadrotor)

https://github.com/kay01-kwon/nmpc_quad/assets/46738866/12070ae6-7412-4a48-b1e3-84e50055672b

quad/assets/46738866/22456794-04a5-4aff-ab14-ad91c27cef27

# nmpc node for firefly (Hexacopter)

https://github.com/user-attachments/assets/5d4ef436-6e6e-4a0f-bbd5-910981c7cc54



# Run the nmpc node for hummingbird (quadrotor)

```
chmod +x hummingbird_nmpc_node.py
```

```
rosrun nmpc_drone hummingbird_nmpc_node
```

# Run the nmpc node for firefly (hexarotor)

```
chmod +x firefly_nmpc_node.py
```

```
rosrun nmpc_drone firefly_nmpc_node
```

# Customized Hexacopter

Navigate to the following file.

rotors_simulator/rotors_gazebo_plugins/gazebo_imu_plugin.cpp

Erase line 278.

```
last_time_ = current_time;
```

Go to the line 333.

```
// Publish the IMU message
imu_pub_->Publish(imu_message_);
```

Modify this line like the below.

```
if(dt >= 0.005)
{
    imu_pub_->Publish(imu_message_);
    last_time_ = current_time;
}
```

```
roslaunch drone_gazebo mav_custom_hexacopter.launch
```

In the drone_gazebo/worlds/drone_world.world file, the max_step_size is changed from 0.01 to 0.001, which means that the rate of simulator is set to 1000 Hz.

```
<max_step_size>0.001</max_step_size>
```