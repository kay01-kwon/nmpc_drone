#!/usr/bin/env python3.8
"""
Improved Manual Control Node for Hexacopter Drone

This node handles manual control of a hexacopter using RC input, motion capture
data, and force/torque measurements. It implements a real-time control loop
with linear interpolation for wrench data synchronization.

Author: Improved version
"""

import os
import sys
import threading
from typing import Tuple, Optional
from dataclasses import dataclass
from enum import Enum

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from mavros_msgs.msg import RCIn
from ros_libcanard.msg import hexa_cmd_raw
from geometry_msgs.msg import WrenchStamped

# Setup path for local imports
dir_path = os.path.dirname(os.path.realpath(__file__))
pkg_dir = dir_path[:dir_path.rfind('/')]
sys.path.append(pkg_dir)

from utils import CustomQueue
from manual_control.rc_converter import RcConverter, FlightMode
from manual_control.rc_controller import RcController
from utils.inverse_dynamics import InverseDynamics
from utils import quaternion_math


@dataclass
class ControllerConfig:
    """Configuration parameters for the controller."""
    max_bit: int = 8191
    max_rpm: int = 9800
    period: float = 0.01
    loopback_time: float = 0.05
    max_catchup_steps: int = 3
    buffer_size: int = 20
    timeout_rc: float = 0.015
    timeout_mocap: float = 0.015
    timeout_wrench: float = 0.015
    max_extrapolation_time: float = 0.005  # Maximum allowed extrapolation time


class DataType(Enum):
    """Enum for different data types."""
    RC = 0
    MOCAP = 1
    WRENCH = 2


class MessageParser:
    """Static methods for parsing ROS messages."""

    @staticmethod
    def parse_rc_input(msg: RCIn) -> Tuple[float, np.ndarray]:
        """Parse RC input message."""
        return (msg.header.stamp.to_sec(),
                np.array(msg.channels, dtype=np.int32))

    @staticmethod
    def parse_odometry(msg: Odometry) -> np.ndarray:
        """Parse odometry message into state vector."""
        pose = msg.pose.pose
        twist = msg.twist.twist

        return np.array([
            msg.header.stamp.to_sec(),
            pose.position.x, pose.position.y, pose.position.z,
            twist.linear.x, twist.linear.y, twist.linear.z,
            pose.orientation.w, pose.orientation.x,
            pose.orientation.y, pose.orientation.z,
            twist.angular.x, twist.angular.y, twist.angular.z
        ])

    @staticmethod
    def parse_wrench(msg: WrenchStamped) -> np.ndarray:
        """Parse wrench message."""
        return np.array([
            msg.header.stamp.to_sec(),
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z
        ])


class WrenchInterpolator:
    """Handles linear interpolation of wrench data."""

    def __init__(self, config: ControllerConfig):
        self.config = config
        self.buffer = CustomQueue(maxsize=config.buffer_size)

    def add_wrench_data(self, data: np.ndarray) -> None:
        """Add wrench data to buffer."""
        if self.buffer.full():
            self.buffer.pop()
        self.buffer.push(data)

    def interpolate_wrench(self, target_time: float) -> Optional[np.ndarray]:
        """
        Interpolate wrench data at target time.
        Returns None if extrapolation is required beyond allowed limit.
        """
        if self.buffer.empty():
            return None

        # Clean old data
        cutoff = target_time - self.config.loopback_time
        while not self.buffer.empty() and self.buffer.front()[0] < cutoff:
            self.buffer.pop()

        if self.buffer.empty():
            return None

        # Find the two closest points for interpolation
        data_list = []
        temp_buffer = CustomQueue(maxsize=self.config.buffer_size)

        # Extract data from buffer (preserving buffer state)
        while not self.buffer.empty():
            item = self.buffer.front()
            self.buffer.pop()
            data_list.append(item)
            temp_buffer.push(item)

        # Restore buffer
        while not temp_buffer.empty():
            self.buffer.push(temp_buffer.front())

        if len(data_list) == 0:
            return None

        # Sort by timestamp
        data_list.sort(key=lambda x: x[0])

        # Handle single data point case
        if len(data_list) == 1:
            time_diff = abs(data_list[0][0] - target_time)
            if time_diff > self.config.max_extrapolation_time:
                rospy.logwarn(
                    f"Wrench extrapolation required: {time_diff * 1000:.2f}ms > {self.config.max_extrapolation_time * 1000:.2f}ms")
                return None
            return data_list[0][1:]  # Return wrench data without timestamp

        # Find interpolation points
        before_idx = None
        after_idx = None

        for i, data in enumerate(data_list):
            if data[0] <= target_time:
                before_idx = i
            if data[0] >= target_time and after_idx is None:
                after_idx = i

        # Interpolation cases
        if before_idx is not None and after_idx is not None:
            # Normal interpolation case
            if before_idx == after_idx:
                # Exact match
                return data_list[before_idx][1:]

            # Linear interpolation
            t0, data0 = data_list[before_idx][0], data_list[before_idx][1:]
            t1, data1 = data_list[after_idx][0], data_list[after_idx][1:]

            alpha = (target_time - t0) / (t1 - t0)
            interpolated = data0 + alpha * (data1 - data0)

            return interpolated

        elif before_idx is not None:
            # Need forward extrapolation
            time_diff = target_time - data_list[before_idx][0]
            if time_diff > self.config.max_extrapolation_time:
                rospy.logwarn(
                    f"Forward wrench extrapolation required: {time_diff * 1000:.2f}ms > {self.config.max_extrapolation_time * 1000:.2f}ms")
                return None
            return data_list[before_idx][1:]

        elif after_idx is not None:
            # Need backward extrapolation
            time_diff = data_list[after_idx][0] - target_time
            if time_diff > self.config.max_extrapolation_time:
                rospy.logwarn(
                    f"Backward wrench extrapolation required: {time_diff * 1000:.2f}ms > {self.config.max_extrapolation_time * 1000:.2f}ms")
                return None
            return data_list[after_idx][1:]

        return None


class DataBuffer:
    """Manages data buffers with timing synchronization."""

    def __init__(self, config: ControllerConfig):
        self.config = config
        self.buffers = [
            CustomQueue(maxsize=config.buffer_size),  # RC
            CustomQueue(maxsize=config.buffer_size),  # MOCAP
        ]
        self.wrench_interpolator = WrenchInterpolator(config)

        # Timing arrays: [RC, MOCAP, WRENCH]
        self.time_latest = np.array([-1e3, -1e3, -1e3])
        self.latency = np.array([0.0, 0.0, 0.01])
        self.latest_rx_wall = np.zeros(3)
        self.timeouts = np.array([
            config.timeout_rc,
            config.timeout_mocap,
            config.timeout_wrench
        ])

    def add_data(self, data_type: DataType, data) -> None:
        """Add data to appropriate buffer."""
        idx = data_type.value

        if data_type == DataType.WRENCH:
            self.wrench_interpolator.add_wrench_data(data)
        else:
            buffer = self.buffers[idx]
            if buffer.full():
                buffer.pop()
            buffer.push(data)

        self.time_latest[idx] = max(
            self.time_latest[idx] - self.latency[idx],
            data[0]
        )
        self.latest_rx_wall[idx] = rospy.get_time()

    def prepare_buffer_near(self, data_type: DataType, t_ref: float) -> bool:
        """Prepare buffer by removing old data."""
        if data_type == DataType.WRENCH:
            return True  # Wrench interpolator handles its own data management

        buffer = self.buffers[data_type.value]
        if buffer.empty():
            return False

        cutoff = t_ref - self.config.loopback_time
        while not buffer.empty() and buffer.front()[0] < cutoff:
            buffer.pop()

        return not buffer.empty()

    def get_most_recent_data(self, data_type: DataType):
        """Get the most recent data from buffer."""
        if data_type == DataType.WRENCH:
            return None  # Wrench data is handled by interpolator

        buffer = self.buffers[data_type.value]
        if buffer.empty():
            return None
        return buffer.back()

    def interpolate_wrench_at_time(self, target_time: float) -> Optional[np.ndarray]:
        """Get interpolated wrench data at target time."""
        return self.wrench_interpolator.interpolate_wrench(target_time)

    def is_data_fresh(self, data_type: DataType) -> bool:
        """Check if data is fresh based on timeout."""
        idx = data_type.value
        now = rospy.get_time()
        return (self.latest_rx_wall[idx] > 0.0 and
                now - self.latest_rx_wall[idx] <= self.timeouts[idx])

    def get_watermark_time(self, t_prev: float) -> float:
        """Get the watermark time for synchronization."""
        now = rospy.get_time()
        fresh_indices = [i for i in range(3) if self.is_data_fresh(DataType(i))]

        if not fresh_indices:
            return t_prev

        return min(self.time_latest[i] for i in fresh_indices)


class ManualControlNode:
    """Main manual control node class."""

    def __init__(self):
        """Initialize the manual control node."""
        self.config = ControllerConfig()
        self.data_buffer = DataBuffer(self.config)

        # Initialize state
        self.state = self._initialize_state()
        self.tau = np.zeros(3)

        # Setup components
        self._setup_control_components()
        self._setup_ros_interface()

        # Timing variables
        self.is_first_run = True
        self.t_curr = 0.0
        self.t_prev = 0.0

        # Threading
        self.cv = threading.Condition()
        self.process_thread = threading.Thread(
            target=self._process_loop, daemon=True)

        # Start processing thread
        self.process_thread.start()
        rospy.loginfo("Manual Control Node initialized successfully")

    def _initialize_state(self) -> np.ndarray:
        """Initialize the drone state vector."""
        position = np.zeros(3)
        velocity = np.zeros(3)
        quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z
        angular_velocity = np.zeros(3)

        return np.concatenate([position, velocity, quaternion, angular_velocity])

    def _setup_control_components(self) -> None:
        """Setup control-related components."""
        manual_param = self._load_rc_parameters()
        gain_param, dyn_param = self._load_control_parameters()
        drone_param = self._load_drone_parameters()

        self.rc_converter = RcConverter(manualParam=manual_param)
        self.rc_controller = RcController(
            GainParam=gain_param, DynParam=dyn_param)
        self.inverse_dynamics = InverseDynamics(Param=drone_param)

    def _setup_ros_interface(self) -> None:
        """Setup ROS subscribers and publishers."""
        # Subscribers
        self.rc_sub = rospy.Subscriber(
            '/mavros/rc/in', RCIn, self._rc_callback,
            queue_size=10, tcp_nodelay=True)

        self.state_sub = rospy.Subscriber(
            '/mavros/odometry/in', Odometry, self._mocap_callback,
            queue_size=10, tcp_nodelay=True)

        self.wrench_sub = rospy.Subscriber(
            '/wrench', WrenchStamped, self._wrench_callback,
            queue_size=10, tcp_nodelay=True)

        # Publisher
        self.cmd_pub = rospy.Publisher(
            '/uav/cmd_raw', hexa_cmd_raw, queue_size=10)

        # Command message
        self.cmd_msg = hexa_cmd_raw()

        # Publishing timer
        self.pub_timer = rospy.Timer(
            rospy.Duration(self.config.period), self._publish_callback)

    def _rc_callback(self, msg: RCIn) -> None:
        """Handle RC input messages."""
        with self.cv:
            data = MessageParser.parse_rc_input(msg)
            self.data_buffer.add_data(DataType.RC, data)
            self.cv.notify_all()

    def _mocap_callback(self, msg: Odometry) -> None:
        """Handle motion capture odometry messages."""
        with self.cv:
            data = MessageParser.parse_odometry(msg)
            self.data_buffer.add_data(DataType.MOCAP, data)
            self.cv.notify_all()

    def _wrench_callback(self, msg: WrenchStamped) -> None:
        """Handle wrench messages."""
        with self.cv:
            data = MessageParser.parse_wrench(msg)
            self.data_buffer.add_data(DataType.WRENCH, data)
            self.cv.notify_all()

    def _publish_callback(self, event) -> None:
        """Publish command messages at fixed rate."""
        self.cmd_pub.publish(self.cmd_msg)

    def _process_loop(self) -> None:
        """Main processing loop running in separate thread."""
        rospy.loginfo("Starting control processing thread")

        while not rospy.is_shutdown():
            self._initialize_timing()

            with self.cv:
                # Wait for data or timeout
                self.cv.wait_for(
                    timeout=2 * self.config.period,
                    predicate=lambda: (
                                              self.data_buffer.get_watermark_time(self.t_prev) -
                                              self.t_prev >= self.config.period
                                      ) or rospy.is_shutdown()
                )

                self._process_control_steps()
                self.cv.notify_all()

    def _initialize_timing(self) -> None:
        """Initialize timing on first run."""
        if (self.is_first_run and
                not self.data_buffer.buffers[DataType.MOCAP.value].empty()):
            self.t_curr = self.data_buffer.buffers[DataType.MOCAP.value].back()[0]
            self.t_prev = self.t_curr - self.config.period
            self.is_first_run = False

    def _process_control_steps(self) -> None:
        """Process control steps with catch-up mechanism."""
        watermark = self.data_buffer.get_watermark_time(self.t_prev)
        steps = 0

        while (self.t_prev + self.config.period <= watermark and
               steps < self.config.max_catchup_steps):
            # Advance time
            self.t_curr = self.t_prev + self.config.period

            # Check data availability
            rc_ok = self.data_buffer.prepare_buffer_near(DataType.RC, self.t_curr)
            mocap_ok = self.data_buffer.prepare_buffer_near(DataType.MOCAP, self.t_curr)
            wrench_ok = self.data_buffer.is_data_fresh(DataType.WRENCH)

            # Update control
            self._update_control(rc_ok, mocap_ok, wrench_ok)

            self.t_prev = self.t_curr
            steps += 1

    def _update_control(self, rc_ok: bool, mocap_ok: bool, wrench_ok: bool) -> None:
        """Update control based on available data."""
        # Handle RC input
        if not rc_ok:
            self._set_all_motors(0)
            return

        rc_data = self.data_buffer.get_most_recent_data(DataType.RC)
        if rc_data is None:
            self._set_all_motors(0)
            return

        self.rc_converter.set_rc(rc_data[1])
        a_des, z_des, dpsi_dt_des, mode = self.rc_converter.get_rc_state()

        # Handle flight modes
        if mode == FlightMode.KILL:
            self._set_all_motors(0)
            return
        elif mode == FlightMode.ARMED:
            self._set_all_motors(2000)
            return
        elif mode != FlightMode.MANUAL_STAB:
            return

        # Require both mocap and wrench for manual stabilization
        if not (mocap_ok and wrench_ok):
            rospy.logdebug('Mocap or wrench data not available')
            return

        # Get most recent mocap state
        mocap_data = self.data_buffer.get_most_recent_data(DataType.MOCAP)
        if mocap_data is None:
            rospy.logwarn('No mocap data available')
            return

        mocap_time = mocap_data[0]
        state_curr = mocap_data[1:]

        # Interpolate wrench at mocap time
        wrench_curr = self.data_buffer.interpolate_wrench_at_time(mocap_time)
        if wrench_curr is None:
            rospy.logwarn('Wrench interpolation failed or extrapolation required')
            return

        # Use only torque part of wrench (indices 3:6 in wrench data)
        wrench_torque = wrench_curr[3:6]

        # Log timing information
        rospy.logdebug(f'Control update at mocap time: {mocap_time:.3f}s')

        # Compute control
        ref = np.concatenate([a_des, np.array([z_des, dpsi_dt_des])])
        self.rc_controller.set_ref_state(ref, state_curr, wrench_torque)
        u = self.rc_controller.get_control_input()

        # Convert to motor commands
        rpm_des = self.inverse_dynamics.compute_des_rpm(u[0], u[1:])
        self._set_motor_rpm(rpm_des)

    def _set_all_motors(self, value: int) -> None:
        """Set all motors to the same value."""
        for i in range(6):
            self.cmd_msg.raw[i] = int(value)

    def _set_motor_rpm(self, rpm_des: np.ndarray) -> None:
        """Set motor RPM based on desired values."""
        for i in range(6):
            normalized_rpm = rpm_des[i] * self.config.max_bit / self.config.max_rpm
            self.cmd_msg.raw[i] = int(np.clip(normalized_rpm, 0, self.config.max_bit))

    def _load_rc_parameters(self) -> dict:
        """Load RC-related parameters from ROS parameter server."""
        node_name = rospy.get_name()

        return {
            'a_max': rospy.get_param(f'{node_name}/constraint/a_max'),
            'z_max': rospy.get_param(f'{node_name}/constraint/z_max'),
            'dpsi_dt_max': rospy.get_param(f'{node_name}/constraint/dpsi_dt_max')
        }

    def _load_control_parameters(self) -> Tuple[dict, dict]:
        """Load control gain and dynamics parameters."""
        node_name = rospy.get_name()

        gain_param = {
            'Kp_trans': rospy.get_param(f'{node_name}/pid/trans/Kp'),
            'Kd_trans': rospy.get_param(f'{node_name}/pid/trans/Kd'),
            'Kp_ori': rospy.get_param(f'{node_name}/pid/ori/Kp'),
            'Kd_ori': rospy.get_param(f'{node_name}/pid/ori/Kd')
        }

        dyn_param = {
            'm': rospy.get_param(f'{node_name}/nominal_dynamics/m'),
            'J': rospy.get_param(f'{node_name}/nominal_dynamics/J')
        }

        return gain_param, dyn_param

    def _load_drone_parameters(self) -> dict:
        """Load drone-specific parameters."""
        node_name = rospy.get_name()

        return {
            'arm_length': rospy.get_param(f'{node_name}/drone/arm_length'),
            'rotor_const': rospy.get_param(f'{node_name}/drone/rotor_const'),
            'moment_const': rospy.get_param(f'{node_name}/drone/moment_const'),
            'T_min': rospy.get_param(f'{node_name}/drone/T_min'),
            'T_max': rospy.get_param(f'{node_name}/drone/T_max')
        }

    def shutdown(self) -> None:
        """Clean shutdown of the node."""
        rospy.loginfo("Shutting down Manual Control Node")
        if hasattr(self, 'process_thread') and self.process_thread.is_alive():
            self.process_thread.join(timeout=1.0)


def main():
    """Main function."""
    rospy.init_node('manual_control_node')
    node = ManualControlNode()

    # Setup shutdown hook
    rospy.on_shutdown(node.shutdown)

    # Keep the node running
    rospy.spin()


if __name__ == '__main__':
    main()