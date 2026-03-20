from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for Offboard Control Node.

    Examples:
        ros2 launch offboard_control offboard_control.launch.py
        ros2 launch offboard_control offboard_control.launch.py map_frame_id:=odom
        ros2 launch offboard_control offboard_control.launch.py default_speed:=1.0
    """

    # Launch arguments (overridable from CLI)

    map_frame_id_arg = DeclareLaunchArgument(
        'map_frame_id',
        default_value='map',
        description='Frame ID for local coordinate system (e.g. map, odom).'
    )

    transform_timeout_arg = DeclareLaunchArgument(
        'transform_timeout',
        default_value='0.5',
        description='TF lookup timeout (seconds).'
    )

    telemetry_transform_timeout_arg = DeclareLaunchArgument(
        'telemetry_transform_timeout',
        default_value='0.5',
        description='TF timeout for telemetry (seconds).'
    )

    offboard_timeout_arg = DeclareLaunchArgument(
        'offboard_timeout',
        default_value='3.0',
        description='Timeout to enter OFFBOARD mode (seconds).'
    )

    land_timeout_arg = DeclareLaunchArgument(
        'land_timeout',
        default_value='3.0',
        description='Land command timeout (seconds).'
    )

    arming_timeout_arg = DeclareLaunchArgument(
        'arming_timeout',
        default_value='4.0',
        description='Arm command timeout (seconds).'
    )

    default_speed_arg = DeclareLaunchArgument(
        'default_speed',
        default_value='0.5',
        description='Default navigate speed (m/s).'
    )

    auto_release_arg = DeclareLaunchArgument(
        'auto_release',
        default_value='true',
        description='Stop setpoint timer after navigation completes.'
    )

    land_only_in_offboard_arg = DeclareLaunchArgument(
        'land_only_in_offboard',
        default_value='true',
        description='Allow land only in OFFBOARD mode.'
    )

    nav_from_sp_arg = DeclareLaunchArgument(
        'nav_from_sp',
        default_value='true',
        description='Start navigate from current setpoint when available.'
    )

    check_kill_switch_arg = DeclareLaunchArgument(
        'check_kill_switch',
        default_value='true',
        description='Check RC kill switch state.'
    )

    state_timeout_arg = DeclareLaunchArgument(
        'state_timeout',
        default_value='3.0',
        description='VehicleStatus message timeout (seconds).'
    )

    local_position_timeout_arg = DeclareLaunchArgument(
        'local_position_timeout',
        default_value='2.0',
        description='Local position message timeout (seconds).'
    )

    velocity_timeout_arg = DeclareLaunchArgument(
        'velocity_timeout',
        default_value='2.0',
        description='Velocity message timeout (seconds).'
    )

    global_position_timeout_arg = DeclareLaunchArgument(
        'global_position_timeout',
        default_value='10.0',
        description='Global position message timeout (seconds).'
    )

    battery_timeout_arg = DeclareLaunchArgument(
        'battery_timeout',
        default_value='2.0',
        description='Battery status message timeout (seconds).'
    )

    manual_control_timeout_arg = DeclareLaunchArgument(
        'manual_control_timeout',
        default_value='0.0',
        description='ManualControlSetpoint timeout (seconds); 0.0 disables.'
    )

    setpoint_rate_arg = DeclareLaunchArgument(
        'setpoint_rate',
        default_value='50.0',
        description='Setpoint publish rate (Hz).'
    )

    map_frame_id = LaunchConfiguration('map_frame_id')
    transform_timeout = LaunchConfiguration('transform_timeout')
    telemetry_transform_timeout = LaunchConfiguration('telemetry_transform_timeout')
    offboard_timeout = LaunchConfiguration('offboard_timeout')
    land_timeout = LaunchConfiguration('land_timeout')
    arming_timeout = LaunchConfiguration('arming_timeout')
    default_speed = LaunchConfiguration('default_speed')
    auto_release = LaunchConfiguration('auto_release')
    land_only_in_offboard = LaunchConfiguration('land_only_in_offboard')
    nav_from_sp = LaunchConfiguration('nav_from_sp')
    check_kill_switch = LaunchConfiguration('check_kill_switch')
    state_timeout = LaunchConfiguration('state_timeout')
    local_position_timeout = LaunchConfiguration('local_position_timeout')
    velocity_timeout = LaunchConfiguration('velocity_timeout')
    global_position_timeout = LaunchConfiguration('global_position_timeout')
    battery_timeout = LaunchConfiguration('battery_timeout')
    manual_control_timeout = LaunchConfiguration('manual_control_timeout')
    setpoint_rate = LaunchConfiguration('setpoint_rate')

    offboard_control = Node(
        package='offboard_control',
        executable='offboard_control',
        name='offboard_control',
        parameters=[
            {'map_frame_id': map_frame_id},
            {'transform_timeout': transform_timeout},
            {'telemetry_transform_timeout': telemetry_transform_timeout},
            {'offboard_timeout': offboard_timeout},
            {'land_timeout': land_timeout},
            {'arming_timeout': arming_timeout},
            {'default_speed': default_speed},
            {'auto_release': auto_release},
            {'land_only_in_offboard': land_only_in_offboard},
            {'nav_from_sp': nav_from_sp},
            {'check_kill_switch': check_kill_switch},
            {'state_timeout': state_timeout},
            {'local_position_timeout': local_position_timeout},
            {'velocity_timeout': velocity_timeout},
            {'global_position_timeout': global_position_timeout},
            {'battery_timeout': battery_timeout},
            {'manual_control_timeout': manual_control_timeout},
            {'setpoint_rate': setpoint_rate},
        ],
        output='screen'
    )

    return LaunchDescription([
        map_frame_id_arg,
        transform_timeout_arg,
        telemetry_transform_timeout_arg,
        offboard_timeout_arg,
        land_timeout_arg,
        arming_timeout_arg,
        default_speed_arg,
        auto_release_arg,
        land_only_in_offboard_arg,
        nav_from_sp_arg,
        check_kill_switch_arg,
        state_timeout_arg,
        local_position_timeout_arg,
        velocity_timeout_arg,
        global_position_timeout_arg,
        battery_timeout_arg,
        manual_control_timeout_arg,
        setpoint_rate_arg,
        offboard_control,
    ])
