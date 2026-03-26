import signal
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from interbotix_xs_msgs.msg import JointGroupCommand
from interbotix_xs_msgs.srv import RegisterValues
import tf2_ros

ARM_JOINT_NAMES = ['waist', 'shoulder', 'elbow', 'wrist_angle']
GRIPPER_JOINT_NAME = 'gripper'

WALL_MARGIN = 0.02
WALL_HYSTERESIS = 0.0
BLEND_RATE = 0.05


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')

        self.declare_parameter('leader_name', 'leader')
        self.declare_parameter('follower_name', 'follower')

        self.leader_name = self.get_parameter('leader_name').value
        self.follower_name = self.get_parameter('follower_name').value

        self.declare_parameter('wall_enabled', True)
        self.declare_parameter('wall_y_position', -0.145)
        self.declare_parameter('debug_wall', False)
        # -1: freeze when Y < wall, +1: freeze when Y > wall
        self.declare_parameter('wall_direction', -1)

        self.wall_enabled = self.get_parameter('wall_enabled').value
        self.wall_y = self.get_parameter('wall_y_position').value
        self.debug_wall = self.get_parameter('debug_wall').value
        self.wall_direction = self.get_parameter('wall_direction').value

        self.declare_parameter('sleep_position', [0.0, -1.88, 1.5, 0.8, 0.0])
        self.declare_parameter('shutdown_move_time', 1500)
        self.declare_parameter('shutdown_timeout', 3.0)

        self.sleep_position = self.get_parameter('sleep_position').value
        self.shutdown_move_time = self.get_parameter('shutdown_move_time').value
        self.shutdown_timeout = self.get_parameter('shutdown_timeout').value

        self.register_client = self.create_client(
            RegisterValues,
            f'/{self.follower_name}/set_motor_registers'
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.last_safe_joints = {
            'waist': 0.0,
            'shoulder': 0.0,
            'elbow': 0.0,
            'wrist': 0.0,
        }
        self.wall_active = False
        self.blending = False
        self.blend_joints = None

        self.motion_active = False
        self.create_subscription(Bool, '/motion_active', self._motion_active_cb, 10)

        self.startup_phase = True
        self.startup_time = None

        self.subscription = self.create_subscription(
            JointState,
            f'/{self.leader_name}/joint_states',
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(
            JointGroupCommand,
            f'/{self.follower_name}/commands/joint_group',
            10)

        if self.wall_enabled:
            self.get_logger().info(
                f'Teleop Node started: {self.leader_name} -> {self.follower_name} | '
                f'Wall at Y={self.wall_y}m (dir={self.wall_direction}) Debug={self.debug_wall}')
        else:
            self.get_logger().info(
                f'Teleop Node started: {self.leader_name} -> {self.follower_name} (wall disabled).')

    def get_follower_gripper_y(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                f'{self.follower_name}/base_link',
                f'{self.follower_name}/ee_arm_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.01))
            return trans.transform.translation.y
        except Exception:
            return None

    def get_leader_gripper_y(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                f'{self.leader_name}/base_link',
                f'{self.leader_name}/ee_arm_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.01))
            return trans.transform.translation.y
        except Exception:
            return None

    def _set_profile_velocity(self, value):
        for joint in ARM_JOINT_NAMES:
            request = RegisterValues.Request()
            request.cmd_type = 'single'
            request.name = joint
            request.reg = 'Profile_Velocity'
            request.value = value
            self.register_client.call_async(request)

    def _motion_active_cb(self, msg: Bool):
        self.motion_active = msg.data

    def listener_callback(self, msg: JointState):
        if self.motion_active:
            return

        if self.startup_phase:
            if self.startup_time is None:
                self.get_logger().info('Startup: setting slow velocity for initial movement...')
                self._set_profile_velocity(self.shutdown_move_time)
                self.startup_time = time.time()
                return

            elapsed = time.time() - self.startup_time
            if elapsed < 0.1:
                return

            if elapsed > (self.shutdown_move_time / 1000.0 + 0.5):
                self._set_profile_velocity(50)
                self.startup_phase = False
                self.get_logger().info('Startup complete - fast tracking enabled')

        try:
            pos_dict = dict(zip(msg.name, msg.position))
        except Exception as e:
            self.get_logger().warn(f'Error creating position dictionary: {e}')
            return

        try:
            waist = pos_dict['waist']
            shoulder = pos_dict['shoulder']
            elbow = pos_dict['elbow']
            wrist = pos_dict['wrist_angle']

            if self.wall_enabled:
                # Margin pulls effective wall inward so it triggers before physical contact
                effective_wall = self.wall_y - (self.wall_direction * WALL_MARGIN)

                follower_y = self.get_follower_gripper_y()
                leader_y = self.get_leader_gripper_y()

                # Follower arms are ~2x longer, so predicted follower Y ~= 2 * leader Y
                predicted_follower_y = (2.0 * leader_y) if leader_y is not None else None

                follower_str = f"{follower_y:.3f}" if follower_y is not None else "N/A"
                leader_str = f"{leader_y:.3f}" if leader_y is not None else "N/A"
                predicted_str = f"{predicted_follower_y:.3f}" if predicted_follower_y is not None else "N/A"

                if self.debug_wall:
                    self.get_logger().info(
                        f'Wall: waist={waist:.3f} sh={shoulder:.3f} | '
                        f'follower_y={follower_str} leader_y={leader_str} pred={predicted_str} '
                        f'wall={effective_wall:.3f} active={self.wall_active}',
                        throttle_duration_sec=0.2)

                if self.wall_active:
                    deactivate_threshold = effective_wall - (self.wall_direction * WALL_HYSTERESIS)

                    predicted_safe = (predicted_follower_y is not None and
                                      self.wall_direction * predicted_follower_y <= self.wall_direction * deactivate_threshold)
                    if predicted_safe:
                        self.get_logger().info(
                            f'Wall DEACTIVATED: predicted={predicted_follower_y:.3f} >= {deactivate_threshold:.3f} - starting blend')
                        self.wall_active = False
                        self.blending = True
                        self.blend_joints = dict(self.last_safe_joints)
                    else:
                        waist = self.last_safe_joints['waist']
                        shoulder = self.last_safe_joints['shoulder']
                        elbow = self.last_safe_joints['elbow']
                        wrist = self.last_safe_joints['wrist']

                elif self.blending:
                    self.blend_joints['waist'] += BLEND_RATE * (waist - self.blend_joints['waist'])
                    self.blend_joints['shoulder'] += BLEND_RATE * (shoulder - self.blend_joints['shoulder'])
                    self.blend_joints['elbow'] += BLEND_RATE * (elbow - self.blend_joints['elbow'])
                    self.blend_joints['wrist'] += BLEND_RATE * (wrist - self.blend_joints['wrist'])

                    max_diff = max(
                        abs(waist - self.blend_joints['waist']),
                        abs(shoulder - self.blend_joints['shoulder']),
                        abs(elbow - self.blend_joints['elbow']),
                        abs(wrist - self.blend_joints['wrist'])
                    )
                    if max_diff < 0.01:
                        self.get_logger().info('Blend complete - resuming normal tracking')
                        self.blending = False
                        self.blend_joints = None
                    else:
                        waist = self.blend_joints['waist']
                        shoulder = self.blend_joints['shoulder']
                        elbow = self.blend_joints['elbow']
                        wrist = self.blend_joints['wrist']

                    self.last_safe_joints = {
                        'waist': waist, 'shoulder': shoulder,
                        'elbow': elbow, 'wrist': wrist
                    }

                else:
                    crossed_wall = (follower_y is not None and
                                    self.wall_direction * follower_y > self.wall_direction * effective_wall)
                    if crossed_wall:
                        self.get_logger().info(
                            f'Wall ACTIVATED: follower_y={follower_y:.3f} past wall={effective_wall:.3f} - FREEZING ALL JOINTS')
                        self.wall_active = True
                        self.blending = False
                        waist = self.last_safe_joints['waist']
                        shoulder = self.last_safe_joints['shoulder']
                        elbow = self.last_safe_joints['elbow']
                        wrist = self.last_safe_joints['wrist']
                    else:
                        self.last_safe_joints = {
                            'waist': waist, 'shoulder': shoulder,
                            'elbow': elbow, 'wrist': wrist
                        }

            arm_cmd_msg = JointGroupCommand()
            arm_cmd_msg.name = 'arm'
            arm_cmd_msg.cmd = [float(waist), float(shoulder), float(elbow), float(wrist)]
            self.publisher.publish(arm_cmd_msg)

        except KeyError:
            self.get_logger().warn('Leader arm joint states not fully populated yet.',
                                    throttle_duration_sec=1.0)
        except Exception as e:
            self.get_logger().error(f'Error processing arm command: {e}')


        try:
            gripper_cmd_msg = JointGroupCommand()
            gripper_cmd_msg.name = 'gripper'
            gripper_cmd_msg.cmd = [float(pos_dict[GRIPPER_JOINT_NAME])]
            self.publisher.publish(gripper_cmd_msg)

        except KeyError:
            self.get_logger().warn(f"'{GRIPPER_JOINT_NAME}' not in joint_states.",
                                    throttle_duration_sec=1.0)
        except Exception as e:
            self.get_logger().error(f'Error processing gripper command: {e}')

    def shutdown(self):
        """Move follower to sleep position before shutdown.

        Works because follower xs_sdk has SIGINT blocked (via launch prefix),
        so it stays alive and processes our commands while we shut down.
        """
        self.get_logger().info('Shutdown: moving follower to sleep position...')

        try:
            self._set_profile_velocity(self.shutdown_move_time)
            time.sleep(0.1)

            for _ in range(3):
                arm_cmd = JointGroupCommand()
                arm_cmd.name = 'arm'
                arm_cmd.cmd = [float(v) for v in self.sleep_position[:4]]
                self.publisher.publish(arm_cmd)

                gripper_cmd = JointGroupCommand()
                gripper_cmd.name = 'gripper'
                gripper_cmd.cmd = [float(self.sleep_position[4])]
                self.publisher.publish(gripper_cmd)

                time.sleep(0.02)

            wait_time = min(self.shutdown_move_time / 1000.0 + 0.5, self.shutdown_timeout)
            self.get_logger().info(f'Shutdown: waiting {wait_time:.1f}s for movement...')
            time.sleep(wait_time)

            self.get_logger().info('Shutdown complete')
        except Exception as e:
            self.get_logger().warn(f'Shutdown sequence failed: {e}')


_shutdown_requested = False


def _signal_handler(sig, frame):
    global _shutdown_requested
    _shutdown_requested = True


def main(args=None):
    global _shutdown_requested
    rclpy.init(args=args)
    node = TeleopNode()

    # Override SIGINT so rclpy doesn't tear down before we publish the sleep command
    signal.signal(signal.SIGINT, _signal_handler)

    while rclpy.ok() and not _shutdown_requested:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
