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


```
roslaunch drone_gazebo mav_custom_hexacopter.launch
```

IMU white and random walk noise data (Pixhawk 6x mini)

| Parameter | 200 Hz value | 100 Hz value (x ${\sqrt{2}})|
|:---:|:---:|:---:|
|Accel noise density| 0.004997776 | 0.0070708|
|Accel random walk| 2.610869e-04 | 3.6921e-04|
|Gyro noise density|0.0003059355 | 0.0004326|
|Gyro random walk|9.059705e-06 | 1.2810e-05|