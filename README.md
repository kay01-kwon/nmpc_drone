# nmpc_quad

It is of paramount to take frame into account when employing controller.

Frame info:

/hummingbird/ground_truth/odometry: World frame

/hummingbird/imu: hummingbird/imu_link frame


To do list:

- [ ] Construct message filter to subscribe to /hummingbird/ground_truth/odometry, /hummingbird/imu, and /nmpc_quad/ref topics.

- [ ] Test yaw control stability when reaching the desired yaw.

https://github.com/kay01-kwon/nmpc_quad/assets/46738866/22456794-04a5-4aff-ab14-ad91c27cef27

# Continuous low pass filter

## Noise information

Magnitude: 5

Standard deviation: 0.5

frequency: 1 Hz


## Low pass filter setup

$\tau$ = 20.0

Continuous low pass filter result is shown below. 

<img src='l1_estimator/figures/ros_low_pass_filter_test_result.png'>