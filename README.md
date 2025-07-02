# nmpc_drone

It is of paramount to take frame into account when employing controller.

Frame info:

/hummingbird/ground_truth/odometry

position - world frame,

linear velocity - body frame (Child frame)

quaternion - body to world frame

angular velocity - body frame

By transforming the linear velocity from body frame to world frame, the position controller stability issue is fixed when it reaches the desired yaw.

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

How to launch simulator.

```
roslaunch drone_gazebo mav_custom_hexacopter.launch
```

IMU white and random walk noise data (Pixhawk 6x mini)

| Parameter | continuous | discrete - 100 Hz|
|:---:|:---:|:---:|
|Accel noise density| 0.00658978579 | 0.0658978579 |
|Accel random walk| 0.0006166810650256017 | - |
|Gyro noise density|0.00037331694 | 0.0037331694 |
|Gyro random walk|1.808187069983041e-05 | - |


1. Noise density (white noise):
${\sigma_{discrete}} = \sigma_{conti}\frac{1}{\sqrt{\Delta t}}$

2. Random walk:
${\sigma_{d}} = \sigma_{conti} \sqrt{\Delta t}$

Note that you should put the continuous noise density in the imu model.

```
    accelerometer_noise_density="0.00658978579"
    accelerometer_random_walk="0.0006166810650256017"
    accelerometer_full_scale="32.0"
    gyroscope_noise_density="0.00037331694"
    gyroscope_random_walk="1.808187069983041e-05"
    gyroscope_full_scale="4000.0"
```

## How to modify the location of center of mass

The location of xacro

:custom_hexacopter/multirotor_base.xacro

Line 44

```
    <link name="${robot_namespace}/base_link_inertia">
      <inertial>
        <mass value="${mass}" />  <!-- [kg] -->
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <xacro:insert_block name="inertia" />
      </inertial>
```

rx = 0.02
ry = 0.03

```
    <link name="${robot_namespace}/base_link_inertia">
      <inertial>
        <mass value="${mass}" />  <!-- [kg] -->
        <origin xyz="0.02 0.03 0" rpy="0 0 0"/>
        <xacro:insert_block name="inertia" />
      </inertial>
```