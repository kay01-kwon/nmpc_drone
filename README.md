# nmpc_quad

It is of paramount to take frame into account when employing controller.

Frame info:

/hummingbird/ground_truth/odometry

position - world frame,

linear velocity - body frame (Child frame)

quaternion - body to world frame

angular velocity - body frame

By transforming the linear velocity from body frame to world frame, the position controller stability issue is fixed when it reaches the desired yaw.


To do list:

- [ ] Construct message filter to subscribe to /hummingbird/ground_truth/odometry, /hummingbird/imu, and /nmpc_quad/ref topics.

- [x] Fix position control stability when reaching the desired yaw.

https://github.com/kay01-kwon/nmpc_quad/assets/46738866/12070ae6-7412-4a48-b1e3-84e50055672b

quad/assets/46738866/22456794-04a5-4aff-ab14-ad91c27cef27

# Continuous low pass filter

## Noise information

Magnitude: 5

Standard deviation: 0.5

frequency: 1 Hz


## Low pass filter setup

$\tau$ = 20.0

Continuous low pass filter result is shown below. 

<img src='l1_estimator/figures/ros_low_pass_filter_test_result.png'>


### Translational disturbance estimation result through L1 estimator

<img src='l1_estimator/figures/disturbance_estimation_result(trans).png'>

### Rotational disturbance estimation result through L1 estimator
<img src='l1_estimator/figures/disturbance_estimation_result(orien).png'>

But, it is problematic since it only utilizes angular velocity data which is susceptible to bias.

$\omega_{meas} = \omega + \eta_{\omega} + b_{\omega}$

where $\eta_{\omega}$ ~ $\mathcal{N}(0,Q)$ and 
$\dot{b_{\omega}}$ ~ $\mathcal{N}(0,R)$.

Thus, quaternion should be used to estimate the disturbance perfectly.

## Simulation model

To do list

- [x] Construct simulation model to test l1 estimator.


## Source code to test

To do list

- [x] Employ matplotlib to plot data.

- [x] Fix the nan problem on quaternion and $\theta_{est}$.

- [x] Make the plot tools in the estimator_test_tool folder.