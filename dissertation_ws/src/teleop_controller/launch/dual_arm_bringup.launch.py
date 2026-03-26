import os
import subprocess
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def create_arm_pair(pair_name, leader_name, follower_name,
                    leader_motor_config, follower_motor_config,
                    leader_mode_config, follower_mode_config,
                    teleop_config, leader_urdf_description, follower_urdf_description,
                    leader_tf_x, leader_tf_y,
                    follower_tf_x, follower_tf_y):
    leader_xs_sdk = Node(
        package='interbotix_xs_sdk',
        executable='xs_sdk',
        name='xs_sdk',
        namespace=leader_name,
        parameters=[{
            'motor_configs': leader_motor_config,
            'mode_configs': leader_mode_config,
            'load_configs': True,
            'robot_description': leader_urdf_description,
            'xs_driver_logging_level': 'INFO',
        }],
        output='screen',
    )

    leader_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=leader_name,
        parameters=[{
            'robot_description': leader_urdf_description,
        }],
        output='screen',
    )

    # SIGINT blocked so follower stays alive during graceful shutdown sequence
    follower_xs_sdk = Node(
        package='interbotix_xs_sdk',
        executable='xs_sdk',
        name='xs_sdk',
        namespace=follower_name,
        prefix='python3 -c "import os,signal,sys;signal.pthread_sigmask(signal.SIG_BLOCK,[signal.SIGINT]);os.execvp(sys.argv[1],sys.argv[1:])"',
        parameters=[{
            'motor_configs': follower_motor_config,
            'mode_configs': follower_mode_config,
            'load_configs': True,
            'robot_description': follower_urdf_description,
            'xs_driver_logging_level': 'INFO',
        }],
        output='screen',
    )

    follower_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=follower_name,
        parameters=[{
            'robot_description': follower_urdf_description,
        }],
        output='screen',
    )

    teleop_node = Node(
        package='teleop_controller',
        executable='teleop_node',
        name=f'teleop_node_{pair_name}',
        parameters=[teleop_config],
        output='screen',
    )

    ff_node = Node(
        package='teleop_controller',
        executable='force_feedback_node',
        name=f'force_feedback_node_{pair_name}',
        parameters=[teleop_config],
        output='screen',
    )

    motion_node = Node(
        package='teleop_controller',
        executable='motion_node',
        name=f'motion_node_{pair_name}',
        parameters=[teleop_config],
        output='screen',
    )

    leader_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{leader_name}_static_tf',
        arguments=[
            str(leader_tf_x), str(leader_tf_y), '0',
            '0', '0', '0',
            'world',
            f'{leader_name}/base_link',
        ],
    )

    follower_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{follower_name}_static_tf',
        arguments=[
            str(follower_tf_x), str(follower_tf_y), '0',
            '0', '0', '0',
            'world',
            f'{follower_name}/base_link',
        ],
    )

    return [
        leader_xs_sdk,
        leader_rsp,
        follower_xs_sdk,
        follower_rsp,
        teleop_node,
        ff_node,
        motion_node,
        leader_tf,
        follower_tf,
    ]


def _read_port_from_motor_config(config_path):
    with open(config_path, 'r', encoding='utf-8') as f:
        for line in f:
            stripped = line.strip()
            if stripped.startswith('port:'):
                return stripped.split(':', 1)[1].strip().strip('\'"')
    raise RuntimeError(f'No "port:" entry found in motor config: {config_path}')


def _find_duplicates(name_to_value):
    value_to_names = {}
    for name, value in name_to_value.items():
        value_to_names.setdefault(value, []).append(name)
    return {value: names for value, names in value_to_names.items() if len(names) > 1}


def _load_expected_serials_from_udev_rules():
    rules_path = '/etc/udev/rules.d/99-dynamixel.rules'
    expected = {}
    if not os.path.exists(rules_path):
        return expected

    try:
        with open(rules_path, 'r', encoding='utf-8') as f:
            for line in f:
                stripped = line.strip()
                if (not stripped or stripped.startswith('#') or
                        'ATTRS{serial}==' not in stripped or 'SYMLINK+="' not in stripped):
                    continue
                try:
                    serial = stripped.split('ATTRS{serial}=="', 1)[1].split('"', 1)[0]
                    symlink = stripped.split('SYMLINK+="', 1)[1].split('"', 1)[0]
                    expected[f'/dev/{symlink}'] = serial
                except IndexError:
                    continue
    except OSError:
        return {}

    return expected


def _has_generic_interbotix_ttydxl_rule():
    rules_path = '/etc/udev/rules.d/99-interbotix-udev.rules'
    if not os.path.exists(rules_path):
        return False
    try:
        with open(rules_path, 'r', encoding='utf-8') as f:
            text = f.read()
    except OSError:
        return False
    return 'SYMLINK+="ttyDXL%n"' in text


def _read_port_serial(port_path):
    try:
        output = subprocess.check_output(
            ['udevadm', 'info', '-q', 'property', '-n', port_path],
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except (subprocess.CalledProcessError, FileNotFoundError, PermissionError, OSError):
        return None

    for line in output.splitlines():
        if line.startswith('ID_SERIAL_SHORT='):
            return line.split('=', 1)[1].strip()
    return None


def _validate_unique_serial_ports(arm_to_motor_config):
    configured_ports = {}
    resolved_ports = {}
    missing_ports = {}

    for arm, config_path in arm_to_motor_config.items():
        port = _read_port_from_motor_config(config_path)
        configured_ports[arm] = port
        if os.path.exists(port):
            resolved_ports[arm] = os.path.realpath(port)
        else:
            missing_ports[arm] = port

    duplicate_configured = _find_duplicates(configured_ports)
    duplicate_resolved = _find_duplicates(resolved_ports)
    expected_serials = _load_expected_serials_from_udev_rules()

    errors = []
    if missing_ports:
        missing_list = ', '.join(f'{arm}={port}' for arm, port in sorted(missing_ports.items()))
        errors.append(f'missing serial devices: {missing_list}')

    if duplicate_configured:
        for port, arms in sorted(duplicate_configured.items()):
            errors.append(f'duplicate configured port "{port}" used by {", ".join(sorted(arms))}')

    if duplicate_resolved:
        for device, arms in sorted(duplicate_resolved.items()):
            errors.append(
                f'duplicate physical adapter "{device}" resolved from {", ".join(sorted(arms))}'
            )

    for arm, port in sorted(configured_ports.items()):
        expected_serial = expected_serials.get(port)
        if expected_serial is None or arm in missing_ports:
            continue
        actual_serial = _read_port_serial(port)
        if actual_serial is None:
            errors.append(f'unable to read serial for {arm} on {port}')
        elif actual_serial != expected_serial:
            errors.append(
                f'{arm} on {port} has serial {actual_serial}, expected {expected_serial} from udev'
            )

    if errors:
        details = '\n  - '.join(errors)
        extra_hint = ''
        if _has_generic_interbotix_ttydxl_rule():
            extra_hint = (
                '\nDetected /etc/udev/rules.d/99-interbotix-udev.rules with '
                'SYMLINK+="ttyDXL%n", which can conflict with per-serial mappings.'
            )
        raise RuntimeError(
            'Serial port validation failed before launch:\n'
            f'  - {details}\n'
            'Fix /dev/ttyDXL* mapping (run identify_arms.py --refresh and replug if needed), '
            f'then relaunch.{extra_hint}'
        )


def generate_launch_description():

    pkg_dir = get_package_share_directory('teleop_controller')
    leader_urdf_file = os.path.join(
        get_package_share_directory('interbotix_xsarm_descriptions'),
        'urdf',
        'px100.urdf.xacro',
    )

    leader_mode_config = os.path.join(pkg_dir, 'config', 'leader_modes.yaml')
    follower_mode_config = os.path.join(pkg_dir, 'config', 'follower_modes.yaml')
    teleop_config = os.path.join(pkg_dir, 'config', 'teleop_config.yaml')
    follower_urdf_file = os.path.join(pkg_dir, 'urdf', 'px100_large.urdf.xacro')

    leader_left_motor = os.path.join(pkg_dir, 'config', 'leader_left_px100.yaml')
    leader_right_motor = os.path.join(pkg_dir, 'config', 'leader_right_px100.yaml')
    follower_left_motor = os.path.join(pkg_dir, 'config', 'follower_left_px100.yaml')
    follower_right_motor = os.path.join(pkg_dir, 'config', 'follower_right_px100.yaml')

    _validate_unique_serial_ports({
        'leader_left': leader_left_motor,
        'follower_left': follower_left_motor,
        'leader_right': leader_right_motor,
        'follower_right': follower_right_motor,
    })

    leader_left_urdf = xacro.process_file(
        leader_urdf_file,
        mappings={'robot_name': 'leader_left', 'use_world_frame': 'false'}
    ).toxml()

    leader_right_urdf = xacro.process_file(
        leader_urdf_file,
        mappings={'robot_name': 'leader_right', 'use_world_frame': 'false'}
    ).toxml()

    follower_left_urdf = xacro.process_file(
        follower_urdf_file,
        mappings={'robot_name': 'follower_left', 'use_world_frame': 'false'}
    ).toxml()

    follower_right_urdf = xacro.process_file(
        follower_urdf_file,
        mappings={'robot_name': 'follower_right', 'use_world_frame': 'false'}
    ).toxml()

    # Layout (top-down, X=forward, Y=left):
    #   follower_left (+Y)  |wall|  follower_right (-Y)
    #   leader_left (+Y)             leader_right (-Y)
    left_nodes = create_arm_pair(
        pair_name='left',
        leader_name='leader_left',
        follower_name='follower_left',
        leader_motor_config=leader_left_motor,
        follower_motor_config=follower_left_motor,
        leader_mode_config=leader_mode_config,
        follower_mode_config=follower_mode_config,
        teleop_config=teleop_config,
        leader_urdf_description=leader_left_urdf,
        follower_urdf_description=follower_left_urdf,
        leader_tf_x=-0.5, leader_tf_y=0.375,
        follower_tf_x=0.0, follower_tf_y=0.375,
    )

    right_nodes = create_arm_pair(
        pair_name='right',
        leader_name='leader_right',
        follower_name='follower_right',
        leader_motor_config=leader_right_motor,
        follower_motor_config=follower_right_motor,
        leader_mode_config=leader_mode_config,
        follower_mode_config=follower_mode_config,
        teleop_config=teleop_config,
        leader_urdf_description=leader_right_urdf,
        follower_urdf_description=follower_right_urdf,
        leader_tf_x=-0.5, leader_tf_y=-0.375,
        follower_tf_x=0.0, follower_tf_y=-0.375,
    )

    rviz_config_file = os.path.join(pkg_dir, 'config', 'teleop_rviz.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )

    effort_graph_left = Node(
        package='teleop_controller',
        executable='effort_graph_node',
        name='effort_graph_left',
        parameters=[{'follower_name': 'follower_left'}],
        output='screen',
    )

    effort_graph_right = Node(
        package='teleop_controller',
        executable='effort_graph_node',
        name='effort_graph_right',
        parameters=[{'follower_name': 'follower_right'}],
        output='screen',
    )

    return LaunchDescription(
        left_nodes + right_nodes +
        [rviz_node, effort_graph_left, effort_graph_right]
    )
