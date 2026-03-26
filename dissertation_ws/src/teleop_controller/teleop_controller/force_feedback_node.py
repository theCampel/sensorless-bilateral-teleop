#!/usr/bin/env python3
"""
Force Feedback Node

Reads effort (motor current) from follower joint states and sends proportional
opposing PWM to the leader arm when effort exceeds a threshold.

Known limitation: cannot distinguish gravity from obstacles (both cause high
effort), so works best when the arm is near vertical.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from interbotix_xs_msgs.srv import RegisterValues, TorqueEnable

ARM_JOINTS = ['waist', 'shoulder', 'elbow', 'wrist_angle']


class ForceFeedbackNode(Node):
    def __init__(self):
        super().__init__('force_feedback_node')

        self.declare_parameter('leader_name', 'leader')
        self.declare_parameter('follower_name', 'follower')

        self.leader_name = self.get_parameter('leader_name').value
        self.follower_name = self.get_parameter('follower_name').value

        self.declare_parameter('enabled', True)
        self.declare_parameter('effort_threshold', 150.0)
        self.declare_parameter('max_pwm', 350.0)
        self.declare_parameter('pwm_scale', 0.5)
        self.declare_parameter('smoothing_factor', 0.15)
        self.declare_parameter('pwm_deadband', 20)
        self.declare_parameter('update_rate', 30.0)

        self.enabled = self.get_parameter('enabled').value
        self.effort_threshold = self.get_parameter('effort_threshold').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.pwm_scale = self.get_parameter('pwm_scale').value
        self.smoothing_factor = self.get_parameter('smoothing_factor').value
        self.pwm_deadband = self.get_parameter('pwm_deadband').value
        self.update_rate = self.get_parameter('update_rate').value
        self.startup_service_timeout = 15.0  # seconds

        self.follower_effort = {j: 0.0 for j in ARM_JOINTS}
        self.current_pwm = {j: 0.0 for j in ARM_JOINTS}
        self.last_pwm_sent = {j: 0 for j in ARM_JOINTS}

        self.motion_active = False
        self.create_subscription(Bool, '/motion_active', self._motion_active_cb, 10)

        self.last_follower_time = self.get_clock().now()
        self.watchdog_timeout = 1.0

        self.pwm_client = self.create_client(
            RegisterValues,
            f'/{self.leader_name}/set_motor_registers'
        )

        self.torque_client = self.create_client(
            TorqueEnable,
            f'/{self.leader_name}/torque_enable'
        )

        self._wait_for_service(
            self.pwm_client,
            f'/{self.leader_name}/set_motor_registers',
            self.startup_service_timeout,
        )
        self._wait_for_service(
            self.torque_client,
            f'/{self.leader_name}/torque_enable',
            self.startup_service_timeout,
        )

        self.set_arm_torque(self.enabled)

        self.follower_sub = self.create_subscription(
            JointState,
            f'/{self.follower_name}/joint_states',
            self.follower_callback,
            10
        )

        self.timer = self.create_timer(1.0 / self.update_rate, self.control_loop)

        self.get_logger().info('=' * 50)
        self.get_logger().info(f'FORCE FEEDBACK: {self.leader_name} <- {self.follower_name}')
        self.get_logger().info(f'Enabled: {self.enabled}')
        self.get_logger().info(f'Effort threshold: {self.effort_threshold} mA')
        self.get_logger().info(f'PWM scale: {self.pwm_scale}, Max: {self.max_pwm}')
        self.get_logger().info(f'Smoothing: {self.smoothing_factor}')
        self.get_logger().info('=' * 50)

    def _wait_for_service(self, client, service_name: str, timeout_sec: float):
        self.get_logger().info(f'Waiting for {service_name} service...')
        waited = 0.0
        while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
            waited += 1.0
            self.get_logger().info(
                f'Still waiting for {service_name} ({waited:.0f}s/{timeout_sec:.0f}s)...'
            )
            if waited >= timeout_sec:
                raise RuntimeError(
                    f'Timed out waiting for {service_name}; '
                    'leader arm/service is not available.'
                )

        if not rclpy.ok():
            raise RuntimeError(f'Shutdown requested while waiting for {service_name}')

    def follower_callback(self, msg: JointState):
        self.last_follower_time = self.get_clock().now()
        for i, name in enumerate(msg.name):
            if name in ARM_JOINTS and i < len(msg.effort):
                self.follower_effort[name] = msg.effort[i]

    def _motion_active_cb(self, msg: Bool):
        self.motion_active = msg.data

    def control_loop(self):
        if self.motion_active:
            for joint in ARM_JOINTS:
                self.send_pwm(joint, 0)
            return

        if not self.enabled:
            for joint in ARM_JOINTS:
                self.send_pwm(joint, 0)
            return

        time_since_follower = (
            self.get_clock().now() - self.last_follower_time
        ).nanoseconds / 1e9

        if time_since_follower > self.watchdog_timeout:
            for joint in ARM_JOINTS:
                self.send_pwm(joint, 0)
            return

        for joint in ARM_JOINTS:
            effort = self.follower_effort.get(joint, 0.0)

            target_pwm = 0.0
            if abs(effort) >= self.effort_threshold:
                effort_above_threshold = abs(effort) - self.effort_threshold
                pwm_magnitude = min(effort_above_threshold * self.pwm_scale, self.max_pwm)

                if effort > 0:
                    target_pwm = -pwm_magnitude
                else:
                    target_pwm = pwm_magnitude

            self.current_pwm[joint] = (
                self.smoothing_factor * target_pwm +
                (1 - self.smoothing_factor) * self.current_pwm[joint]
            )

            pwm_to_send = int(self.current_pwm[joint])

            if abs(pwm_to_send - self.last_pwm_sent[joint]) >= self.pwm_deadband:
                if abs(pwm_to_send) > 10:
                    self.get_logger().info(f'{joint}: effort={effort:+.0f}mA → PWM={pwm_to_send:+d}')
                self.send_pwm(joint, pwm_to_send)

    def send_pwm(self, joint_name: str, pwm_value: int):
        if pwm_value == self.last_pwm_sent[joint_name]:
            return

        self.last_pwm_sent[joint_name] = pwm_value

        request = RegisterValues.Request()
        request.cmd_type = 'single'
        request.name = joint_name
        request.reg = 'Goal_PWM'
        request.value = pwm_value
        self.pwm_client.call_async(request)

    def set_arm_torque(self, enable: bool):
        """Enable/disable torque on leader arm joints (not gripper)."""
        request = TorqueEnable.Request()
        request.cmd_type = 'group'
        request.name = 'arm'
        request.enable = enable
        self.torque_client.call_async(request)
        if enable:
            self.get_logger().info('Arm torque ENABLED (force feedback active, arm resists)')
        else:
            self.get_logger().info('Arm torque DISABLED (easy to move)')

    def shutdown(self):
        self.get_logger().info('Shutting down - disabling torque and zeroing PWM')
        self.set_arm_torque(False)
        for joint in ARM_JOINTS:
            request = RegisterValues.Request()
            request.cmd_type = 'single'
            request.name = joint
            request.reg = 'Goal_PWM'
            request.value = 0
            self.pwm_client.call_async(request)


def main(args=None):
    rclpy.init(args=args)
    node = ForceFeedbackNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
