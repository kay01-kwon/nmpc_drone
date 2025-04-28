# nmpc_drone

It is of paramount to take frame into account when employing controller.

Frame info:

/hummingbird/ground_truth/odometry

position - world frame,

linear velocity - body frame (Child frame)

quaternion - body to world frame

angular velocity - body frame

By transforming the linear velocity from body frame to world frame, the position controller stability issue is fixed when it reaches the desired yaw.


https://github.com/kay01-kwon/nmpc_quad/assets/46738866/12070ae6-7412-4a48-b1e3-84e50055672b

quad/assets/46738866/22456794-04a5-4aff-ab14-ad91c27cef27

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