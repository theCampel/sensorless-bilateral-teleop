#!/usr/bin/env python3
"""
Motion Node - Interactive RViz markers that trigger predefined motion sequences.

Switches leader from PWM to position mode during a motion, steps through
waypoints non-blockingly, then restores PWM mode when done.
"""

import signal

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand
from interbotix_xs_msgs.srv import OperatingModes, RegisterValues, TorqueEnable
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers import InteractiveMarkerServer


ARM_JOINTS = ['waist', 'shoulder', 'elbow', 'wrist_angle']

# Waypoint format: ([waist, shoulder, elbow, wrist_angle], gripper_pos, hold_time_sec)
MOTIONS = {
    'wave': {
        'profile_velocity': 600,
        'waypoints': [
            ([0.0, -0.35, -0.7, -0.3], 0.5, 0.2),
            ([0.3, -0.35, -0.7, 0.2], 0.5, 0.15),
            ([-0.3, -0.35, -0.7, -0.2], 0.5, 0.15),
            ([0.3, -0.35, -0.7, 0.2], 0.5, 0.15),
            ([-0.3, -0.35, -0.7, -0.2], 0.5, 0.15),
            ([0.0, -0.5, -0.8, -0.1], -0.3, 0.3),
            ([0.0, 0.1, -0.3, -0.5], -0.3, 0.4),
            ([0.0, -1.88, 1.5, 0.8], 0.0, 0.3),
        ],
    },
}


class MotionNode(Node):
    def __init__(self):
        super().__init__('motion_node')

        self.declare_parameter('leader_name', 'leader')
        self.declare_parameter('follower_name', 'follower')
        self.declare_parameter('marker_y', 0.0)

        self.leader_name = self.get_parameter('leader_name').value
        self.follower_name = self.get_parameter('follower_name').value
        marker_y = self.get_parameter('marker_y').value

        self.phase = 'idle'  # 'idle', 'setup', 'waypoint', 'teardown'
        self.current_motion = None
        self.waypoint_index = 0
        self._pending_future = None

        self.motion_active_pub = self.create_publisher(Bool, '/motion_active', 10)
        self.leader_cmd_pub = self.create_publisher(
            JointSingleCommand, f'/{self.leader_name}/commands/joint_single', 10)
        self.follower_cmd_pub = self.create_publisher(
            JointGroupCommand, f'/{self.follower_name}/commands/joint_group', 10)

        self.leader_op_modes = self.create_client(
            OperatingModes, f'/{self.leader_name}/set_operating_modes')
        self.leader_registers = self.create_client(
            RegisterValues, f'/{self.leader_name}/set_motor_registers')
        self.leader_torque = self.create_client(
            TorqueEnable, f'/{self.leader_name}/torque_enable')
        self.follower_registers = self.create_client(
            RegisterValues, f'/{self.follower_name}/set_motor_registers')

        self._wait_for_service(self.leader_op_modes,
                               f'/{self.leader_name}/set_operating_modes')
        self._wait_for_service(self.follower_registers,
                               f'/{self.follower_name}/set_motor_registers')

        self.create_timer(0.5, self._heartbeat)

        self.marker_server = InteractiveMarkerServer(self, 'motion_markers')
        self._make_button_marker(
            name='wave',
            label='Wave',
            position=(0.15, marker_y, 0.45),
            color=(0.2, 0.6, 1.0, 0.9),
        )
        self.marker_server.applyChanges()
        self.get_logger().info(
            f'Motion node ready ({self.leader_name}/{self.follower_name}) '
            f'- click markers in RViz (use Interact tool)')

    def _wait_for_service(self, client, name, timeout=15.0):
        self.get_logger().info(f'Waiting for {name}...')
        waited = 0.0
        while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
            waited += 1.0
            if waited >= timeout:
                raise RuntimeError(f'Timed out waiting for {name}')
        self.get_logger().info(f'{name} available')

    def _once(self, delay, callback):
        """One-shot timer (ROS2 Humble lacks native one_shot support)."""
        timer = self.create_timer(delay, lambda: (timer.cancel(), callback()))

    def _wait_for_future(self, future, callback):
        self._pending_future = future
        self._poll_cb = callback
        self._poll_timer = self.create_timer(0.05, self._poll_future)

    def _poll_future(self):
        if self._pending_future.done():
            self._poll_timer.cancel()
            try:
                self._pending_future.result()
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
            self._poll_cb()

    def _heartbeat(self):
        msg = Bool()
        msg.data = (self.phase != 'idle')
        self.motion_active_pub.publish(msg)

    def _publish_active(self, active):
        msg = Bool()
        msg.data = active
        self.motion_active_pub.publish(msg)

    def _make_button_marker(self, name, label, position, color):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'world'
        int_marker.name = name
        int_marker.description = label
        int_marker.pose.position.x = position[0]
        int_marker.pose.position.y = position[1]
        int_marker.pose.position.z = position[2]
        int_marker.scale = 0.08

        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = 0.06
        sphere.scale.y = 0.06
        sphere.scale.z = 0.06
        sphere.color.r = color[0]
        sphere.color.g = color[1]
        sphere.color.b = color[2]
        sphere.color.a = color[3]

        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.BUTTON
        control.always_visible = True
        control.markers.append(sphere)
        int_marker.controls.append(control)

        self.marker_server.insert(int_marker, feedback_callback=self._marker_cb)

    def _marker_cb(self, feedback):
        motion_name = feedback.marker_name
        if self.phase != 'idle':
            self.get_logger().warn(f'Motion already running, ignoring {motion_name}')
            return
        if motion_name not in MOTIONS:
            return

        self.get_logger().info(f'Starting motion: {motion_name}')
        self.current_motion = MOTIONS[motion_name]
        self.waypoint_index = 0
        self._start_setup()

    def _start_setup(self):
        self.phase = 'setup'
        self._publish_active(True)

        pv = self.current_motion['profile_velocity']
        for joint in ARM_JOINTS:
            req = RegisterValues.Request()
            req.cmd_type = 'single'
            req.name = joint
            req.reg = 'Profile_Velocity'
            req.value = pv
            self.follower_registers.call_async(req)

        req = OperatingModes.Request()
        req.cmd_type = 'group'
        req.name = 'arm'
        req.mode = 'position'
        req.profile_type = 'time'
        req.profile_velocity = pv
        req.profile_acceleration = 100
        future = self.leader_op_modes.call_async(req)
        self._wait_for_future(future, self._start_waypoints)

    def _start_waypoints(self):
        self.phase = 'waypoint'
        self.get_logger().info('Leader mode switch confirmed - starting waypoints')
        self._send_next_waypoint()

    def _send_next_waypoint(self):
        if self.waypoint_index >= len(self.current_motion['waypoints']):
            self._start_teardown()
            return

        arm_pos, gripper_pos, hold_time = self.current_motion['waypoints'][self.waypoint_index]
        pv = self.current_motion['profile_velocity']

        self.get_logger().info(
            f'  Waypoint {self.waypoint_index + 1}/{len(self.current_motion["waypoints"])}')

        for i, joint in enumerate(ARM_JOINTS):
            cmd = JointSingleCommand()
            cmd.name = joint
            cmd.cmd = float(arm_pos[i])
            self.leader_cmd_pub.publish(cmd)

        arm_cmd = JointGroupCommand()
        arm_cmd.name = 'arm'
        arm_cmd.cmd = [float(v) for v in arm_pos]
        self.follower_cmd_pub.publish(arm_cmd)

        gripper_cmd = JointGroupCommand()
        gripper_cmd.name = 'gripper'
        gripper_cmd.cmd = [float(gripper_pos)]
        self.follower_cmd_pub.publish(gripper_cmd)

        wait_sec = (pv / 1000.0) + hold_time
        self.waypoint_index += 1
        self._once(wait_sec, self._send_next_waypoint)

    def _start_teardown(self):
        self.phase = 'teardown'
        self.get_logger().info('Motion complete - restoring modes...')

        for joint in ARM_JOINTS:
            req = RegisterValues.Request()
            req.cmd_type = 'single'
            req.name = joint
            req.reg = 'Profile_Velocity'
            req.value = 50
            self.follower_registers.call_async(req)

        req = OperatingModes.Request()
        req.cmd_type = 'group'
        req.name = 'arm'
        req.mode = 'pwm'
        req.profile_type = 'velocity'
        req.profile_velocity = 0
        req.profile_acceleration = 0
        future = self.leader_op_modes.call_async(req)
        self._wait_for_future(future, self._finish_teardown)

    def _finish_teardown(self):
        # Zero Goal_PWM so leader doesn't jump when torque re-enables
        for joint in ARM_JOINTS:
            req = RegisterValues.Request()
            req.cmd_type = 'single'
            req.name = joint
            req.reg = 'Goal_PWM'
            req.value = 0
            self.leader_registers.call_async(req)

        torque_req = TorqueEnable.Request()
        torque_req.cmd_type = 'group'
        torque_req.name = 'arm'
        torque_req.enable = False
        self.leader_torque.call_async(torque_req)

        self._once(0.2, self._finish)

    def _finish(self):
        self.phase = 'idle'
        self.current_motion = None
        self._publish_active(False)
        self.get_logger().info('Motion done - teleop resumed')

    def shutdown(self):
        if self.phase == 'idle':
            return

        self.get_logger().warn('Shutdown mid-motion - restoring leader to PWM mode')
        try:
            req = OperatingModes.Request()
            req.cmd_type = 'group'
            req.name = 'arm'
            req.mode = 'pwm'
            req.profile_type = 'velocity'
            req.profile_velocity = 0
            req.profile_acceleration = 0
            self.leader_op_modes.call_async(req)

            torque_req = TorqueEnable.Request()
            torque_req.cmd_type = 'group'
            torque_req.name = 'arm'
            torque_req.enable = False
            self.leader_torque.call_async(torque_req)

            self._publish_active(False)
        except Exception as e:
            self.get_logger().error(f'Shutdown cleanup failed: {e}')


_shutdown_requested = False


def _signal_handler(sig, frame):
    global _shutdown_requested
    _shutdown_requested = True


def main(args=None):
    global _shutdown_requested
    rclpy.init(args=args)
    node = MotionNode()

    signal.signal(signal.SIGINT, _signal_handler)

    while rclpy.ok() and not _shutdown_requested:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
