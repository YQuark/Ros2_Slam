#!/usr/bin/env python3

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _package_exists(pkg_name: str) -> bool:
    try:
        get_package_share_directory(pkg_name)
        return True
    except Exception:
        return False


def _read_lidar_port_hint(params_file: str) -> str:
    if not params_file or not os.path.isfile(params_file):
        return ''
    try:
        with open(params_file, 'r', encoding='utf-8') as handle:
            for line in handle:
                match = re.match(r'^\s*port\s*:\s*["\']?([^"\']+)["\']?\s*$', line)
                if match:
                    return match.group(1).strip()
    except OSError:
        return ''
    return ''


def _validate_and_compose(context):
    mode = LaunchConfiguration('mode').perform(context)
    mapping_source = LaunchConfiguration('mapping_source').perform(context)
    use_rviz = LaunchConfiguration('use_rviz')
    use_lidar = LaunchConfiguration('use_lidar')
    use_camera = LaunchConfiguration('use_camera')
    use_base_legacy = LaunchConfiguration('use_base').perform(context).lower() == 'true'
    base_mode = LaunchConfiguration('base_mode').perform(context).strip().lower()
    base_fusion_mode = LaunchConfiguration('base_fusion_mode').perform(context).strip().lower()
    use_visual_odom = LaunchConfiguration('use_visual_odom')
    fallback_static_odom = LaunchConfiguration('fallback_static_odom').perform(context).lower() == 'true'
    map_file = LaunchConfiguration('map_file').perform(context)
    rviz_enabled = use_rviz.perform(context).lower() == 'true'
    lidar_params_file = LaunchConfiguration('lidar_params_file').perform(context)
    auto_mapping_drive = LaunchConfiguration('auto_mapping_drive').perform(context).lower() == 'true'
    auto_mapping_nav2_params_file = LaunchConfiguration('auto_mapping_nav2_params_file').perform(context)
    base_status_log_interval_sec = LaunchConfiguration('base_status_log_interval_sec').perform(context)
    base_cmd_log_interval_sec = LaunchConfiguration('base_cmd_log_interval_sec').perform(context)
    base_cmd_timeout = LaunchConfiguration('base_cmd_timeout').perform(context)
    base_drive_keepalive_sec = LaunchConfiguration('base_drive_keepalive_sec').perform(context)
    base_port_value = LaunchConfiguration('base_port').perform(context).strip()
    nav2_start = LaunchConfiguration('nav2_start').perform(context).lower() == 'true'

    if mode not in ('mapping', 'navigation'):
        raise RuntimeError("Invalid 'mode'. Use 'mapping' or 'navigation'.")

    this_share = get_package_share_directory('robot_bringup')

    if base_mode not in ('real', 'fake', 'none'):
        raise RuntimeError("Invalid 'base_mode'. Use 'real', 'fake' or 'none'.")
    if base_fusion_mode not in ('none', 'ekf'):
        raise RuntimeError("Invalid 'base_fusion_mode'. Use 'none' or 'ekf'.")

    # Backward compatibility: legacy use_base:=false disables real base by default.
    if base_mode == 'real' and not use_base_legacy:
        base_mode = 'none'

    real_base_enabled = base_mode == 'real'
    fake_base_enabled = base_mode == 'fake'
    no_base = base_mode == 'none'
    use_base_ekf = real_base_enabled and base_fusion_mode == 'ekf'
    lidar_enabled = use_lidar.perform(context).lower() == 'true'
    camera_enabled = use_camera.perform(context).lower() == 'true'
    if mode == 'mapping' and mapping_source == 'camera':
        lidar_enabled = False
    lidar_port_hint = _read_lidar_port_hint(lidar_params_file) if lidar_enabled else ''
    if auto_mapping_drive:
        if not real_base_enabled and not fake_base_enabled:
            raise RuntimeError("auto_mapping_drive requires base_mode:=real or base_mode:=fake.")
        if not os.path.isfile(auto_mapping_nav2_params_file):
            raise RuntimeError(f"auto_mapping_drive nav2 params file does not exist: {auto_mapping_nav2_params_file}")
        if real_base_enabled:
            if base_port_value.lower() != 'auto' and not os.path.exists(base_port_value):
                raise RuntimeError(f"auto_mapping_drive base_port does not exist: {base_port_value}")
            if (
                base_port_value.lower() != 'auto'
                and lidar_port_hint
                and os.path.realpath(base_port_value) == os.path.realpath(lidar_port_hint)
            ):
                raise RuntimeError(
                    f"auto_mapping_drive cannot share one serial port between base and lidar: {base_port_value}"
                )
        try:
            if float(base_status_log_interval_sec) <= 0.0:
                base_status_log_interval_sec = '10.0'
        except ValueError:
            pass
        try:
            if float(base_cmd_log_interval_sec) <= 0.0:
                base_cmd_log_interval_sec = '5.0'
        except ValueError:
            pass

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_share, 'launch', 'sensors.launch.py')),
        launch_arguments={
            'use_lidar': 'true' if lidar_enabled else 'false',
            'use_camera': 'true' if camera_enabled else 'false',
            'lidar_params_file': LaunchConfiguration('lidar_params_file'),
            'lidar_raw_scan_topic': LaunchConfiguration('lidar_raw_scan_topic'),
            'lidar_scan_topic': LaunchConfiguration('lidar_scan_topic'),
            'lidar_scan_output_size': LaunchConfiguration('lidar_scan_output_size'),
            'lidar_scan_angle_min': LaunchConfiguration('lidar_scan_angle_min'),
            'lidar_scan_angle_max': LaunchConfiguration('lidar_scan_angle_max'),
            'lidar_scan_normalizer_log_interval_sec': LaunchConfiguration('lidar_scan_normalizer_log_interval_sec'),
            'lidar_tf_x': LaunchConfiguration('lidar_tf_x'),
            'lidar_tf_y': LaunchConfiguration('lidar_tf_y'),
            'lidar_tf_z': LaunchConfiguration('lidar_tf_z'),
            'lidar_tf_roll': LaunchConfiguration('lidar_tf_roll'),
            'lidar_tf_pitch': LaunchConfiguration('lidar_tf_pitch'),
            'lidar_tf_yaw': LaunchConfiguration('lidar_tf_yaw'),
            'camera_enable_color': LaunchConfiguration('camera_enable_color'),
            'camera_enable_ir': LaunchConfiguration('camera_enable_ir'),
            'camera_use_uvc': LaunchConfiguration('camera_use_uvc'),
            'camera_color_info_url': LaunchConfiguration('camera_color_info_url'),
            'camera_ir_info_url': LaunchConfiguration('camera_ir_info_url'),
        }.items(),
    )

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_share, 'launch', 'base.launch.py')),
        launch_arguments={
            'port': LaunchConfiguration('base_port'),
            'baudrate': LaunchConfiguration('base_baudrate'),
            'max_linear': LaunchConfiguration('base_max_linear'),
            'max_angular': LaunchConfiguration('base_max_angular'),
            'cmd_timeout': base_cmd_timeout,
            'drive_keepalive_sec': base_drive_keepalive_sec,
            'publish_tf': 'false' if use_base_ekf else LaunchConfiguration('base_publish_tf').perform(context),
            'imu_enabled': LaunchConfiguration('base_imu_enabled'),
            'publish_imu': 'true' if use_base_ekf else 'false',
            'imu_topic': LaunchConfiguration('base_imu_topic'),
            'imu_frame_id': LaunchConfiguration('base_imu_frame_id'),
            'use_status_yaw': LaunchConfiguration('base_use_status_yaw'),
            'status_yaw_mode': LaunchConfiguration('base_status_yaw_mode'),
            'status_yaw_jump_reject_deg': LaunchConfiguration('base_status_yaw_jump_reject_deg'),
            'odom_feedback_source': LaunchConfiguration('base_odom_feedback_source'),
            'wheel_radius': LaunchConfiguration('base_wheel_radius'),
            'wheel_track_width': LaunchConfiguration('base_wheel_track_width'),
            'encoder_cpr': LaunchConfiguration('base_encoder_cpr'),
            'odom_linear_scale': LaunchConfiguration('base_odom_linear_scale'),
            'odom_angular_scale': LaunchConfiguration('base_odom_angular_scale'),
            'odom_angular_sign': LaunchConfiguration('base_odom_angular_sign'),
            'status_log_interval_sec': base_status_log_interval_sec,
            'cmd_log_interval_sec': base_cmd_log_interval_sec,
            'excluded_ports': lidar_port_hint,
        }.items(),
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_share, 'launch', 'base_ekf.launch.py')),
        launch_arguments={
            'ekf_params_file': LaunchConfiguration('base_ekf_params_file'),
        }.items(),
    )

    fake_base_node = Node(
        package='robot_bringup',
        executable='fake_base_odom.py',
        name='fake_base_odom',
        output='screen',
    )

    viz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_share, 'launch', 'viz.launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items(),
    )

    actions = [sensors_launch]
    if real_base_enabled:
        actions.append(base_launch)
        if use_base_ekf:
            if not _package_exists('robot_localization'):
                raise RuntimeError(
                    "Missing package: robot_localization. Install it first, e.g. 'sudo apt install ros-foxy-robot-localization'."
                )
            actions.append(ekf_launch)
    if fake_base_enabled:
        actions.append(fake_base_node)

    visual_odom_enabled = use_visual_odom.perform(context).lower() == 'true'

    if mode == 'mapping' and mapping_source not in ('camera', 'lidar'):
        raise RuntimeError("Invalid 'mapping_source'. Use 'camera' or 'lidar'.")

    if mode == 'mapping' and mapping_source == 'camera':
        if not camera_enabled:
            raise RuntimeError("Camera mapping requires 'use_camera:=true'.")
        if not _package_exists('depthimage_to_laserscan'):
            raise RuntimeError("Missing package: depthimage_to_laserscan.")
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(this_share, 'launch', 'camera_scan.launch.py')),
                launch_arguments={
                    'depth_topic': LaunchConfiguration('camera_scan_depth_topic'),
                    'camera_info_topic': LaunchConfiguration('camera_scan_info_topic'),
                    'scan_topic': LaunchConfiguration('camera_scan_topic'),
                }.items(),
            )
        )
        if no_base and visual_odom_enabled:
            if not _package_exists('rtabmap_odom'):
                raise RuntimeError("Missing package: rtabmap_odom. Install 'ros-foxy-rtabmap-ros'.")
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(this_share, 'launch', 'visual_odom.launch.py')),
                    launch_arguments={
                        'rgb_topic': LaunchConfiguration('visual_odom_rgb_topic'),
                        'depth_topic': LaunchConfiguration('visual_odom_depth_topic'),
                        'camera_info_topic': LaunchConfiguration('visual_odom_camera_info_topic'),
                        'base_frame': LaunchConfiguration('visual_odom_base_frame'),
                        'odom_frame': LaunchConfiguration('visual_odom_odom_frame'),
                    }.items(),
                )
            )
    elif mode == 'mapping' and no_base and camera_enabled and visual_odom_enabled:
        if not _package_exists('rtabmap_odom'):
            raise RuntimeError("Missing package: rtabmap_odom. Install 'ros-foxy-rtabmap-ros'.")
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(this_share, 'launch', 'visual_odom.launch.py')),
                launch_arguments={
                    'rgb_topic': LaunchConfiguration('visual_odom_rgb_topic'),
                    'depth_topic': LaunchConfiguration('visual_odom_depth_topic'),
                    'camera_info_topic': LaunchConfiguration('visual_odom_camera_info_topic'),
                    'base_frame': LaunchConfiguration('visual_odom_base_frame'),
                    'odom_frame': LaunchConfiguration('visual_odom_odom_frame'),
                }.items(),
            )
        )

    # In no-base mapping, always keep a valid odom->base_link chain.
    if mode == 'mapping' and no_base and fallback_static_odom and (
        (mapping_source == 'camera' and not visual_odom_enabled)
        or (mapping_source == 'lidar' and not (camera_enabled and visual_odom_enabled))
    ):
        actions.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='odom_fallback_static_tf',
                arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
                output='screen',
            )
        )

    if mode == 'mapping':
        actions.append(LogInfo(msg=f'[robot_bringup] mode=mapping, mapping_source={mapping_source}, base_mode={base_mode}'))
        if not _package_exists('slam_toolbox'):
            raise RuntimeError('Missing package: slam_toolbox. Install and rebuild workspace.')

        scan_topic = LaunchConfiguration('camera_scan_topic') if mapping_source == 'camera' else LaunchConfiguration('lidar_scan_topic')
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(this_share, 'launch', 'slam.launch.py')),
                launch_arguments={
                    'slam_params_file': LaunchConfiguration('slam_params_file'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'scan_topic': scan_topic,
                }.items(),
            )
        )

        if auto_mapping_drive:
            if not _package_exists('nav2_bringup'):
                raise RuntimeError(
                    "auto_mapping_drive requires nav2_bringup. Install it first, e.g. "
                    "'sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup'."
                )
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(this_share, 'launch', 'nav2.launch.py')),
                    launch_arguments={
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'params_file': LaunchConfiguration('auto_mapping_nav2_params_file'),
                        'default_bt_xml_filename': LaunchConfiguration('auto_mapping_bt_xml_file'),
                        'autostart': LaunchConfiguration('nav2_autostart'),
                    }.items(),
                )
            )
            actions.append(
                Node(
                    package='robot_bringup',
                    executable='frontier_explorer.py',
                    name='frontier_explorer',
                    output='screen',
                    parameters=[{
                        'map_topic': LaunchConfiguration('auto_mapping_map_topic'),
                        'map_frame': LaunchConfiguration('auto_mapping_map_frame'),
                        'base_frame': LaunchConfiguration('auto_mapping_base_frame'),
                        'navigate_action': LaunchConfiguration('auto_mapping_navigate_action'),
                        'tick_hz': LaunchConfiguration('auto_mapping_tick_hz'),
                        'startup_delay_sec': LaunchConfiguration('auto_mapping_startup_delay_sec'),
                        'max_duration_sec': LaunchConfiguration('auto_mapping_max_duration_sec'),
                        'status_log_interval_sec': LaunchConfiguration('auto_mapping_status_log_interval_sec'),
                        'map_timeout_sec': LaunchConfiguration('auto_mapping_map_timeout_sec'),
                        'tf_timeout_sec': LaunchConfiguration('auto_mapping_tf_timeout_sec'),
                        'goal_timeout_sec': LaunchConfiguration('auto_mapping_goal_timeout_sec'),
                        'frontier_min_distance': LaunchConfiguration('auto_mapping_frontier_min_distance'),
                        'frontier_max_distance': LaunchConfiguration('auto_mapping_frontier_max_distance'),
                        'frontier_sample_stride': LaunchConfiguration('auto_mapping_frontier_sample_stride'),
                        'min_frontier_cluster_cells': LaunchConfiguration('auto_mapping_min_frontier_cluster_cells'),
                        'goal_approach_offset': LaunchConfiguration('auto_mapping_goal_approach_offset'),
                        'goal_approach_max_offset': LaunchConfiguration('auto_mapping_goal_approach_max_offset'),
                        'goal_approach_step': LaunchConfiguration('auto_mapping_goal_approach_step'),
                        'goal_clearance_radius': LaunchConfiguration('auto_mapping_goal_clearance_radius'),
                        'goal_unknown_clearance_radius': LaunchConfiguration('auto_mapping_goal_unknown_clearance_radius'),
                        'blacklist_radius': LaunchConfiguration('auto_mapping_blacklist_radius'),
                        'blacklist_ttl_sec': LaunchConfiguration('auto_mapping_blacklist_ttl_sec'),
                    }],
                )
            )

        # RViz map plugin may fail on some GPUs; publish map as point cloud for robust visualization.
        if rviz_enabled and mapping_source == 'lidar':
            actions.append(
                Node(
                    package='robot_bringup',
                    executable='map_to_pointcloud_viz.py',
                    name='map_to_pointcloud_viz',
                    output='screen',
                    parameters=[{
                        'map_topic': '/map',
                        'points_topic': '/map_points',
                        'occupied_threshold': 50,
                        'free_threshold': 20,
                        'publish_free_cells': True,
                        'free_cell_stride': 4,
                        'z_height': 0.02,
                    }],
                )
            )
    else:
        actions.append(LogInfo(msg=f'[robot_bringup] mode=navigation, base_mode={base_mode}'))
        if no_base:
            raise RuntimeError("Navigation mode requires base_mode:=real or base_mode:=fake.")
        if not _package_exists('nav2_bringup'):
            raise RuntimeError(
                "Missing package: nav2_bringup. Install nav2 first, e.g. 'sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup'."
            )
        if map_file == '' or not os.path.isfile(map_file):
            raise RuntimeError("Navigation mode requires a valid 'map_file' (*.yaml).")

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(this_share, 'launch', 'localization.launch.py')),
                launch_arguments={
                    'map_file': LaunchConfiguration('map_file'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'params_file': LaunchConfiguration('nav2_params_file'),
                    'autostart': LaunchConfiguration('nav2_autostart'),
                }.items(),
            )
        )
        if nav2_start:
            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(this_share, 'launch', 'nav2.launch.py')),
                    launch_arguments={
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'params_file': LaunchConfiguration('nav2_params_file'),
                        'default_bt_xml_filename': LaunchConfiguration('nav2_bt_xml_file'),
                        'autostart': LaunchConfiguration('nav2_autostart'),
                    }.items(),
                )
            )

    actions.append(viz_launch)
    return actions


def generate_launch_description():
    rb_share = get_package_share_directory('robot_bringup')

    default_nav2_params = os.path.join(rb_share, 'config', 'nav2_params_robot.yaml')
    if not os.path.isfile(default_nav2_params):
        default_nav2_params = os.path.join(rb_share, 'config', 'nav2_params_fallback.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='mapping'),
        DeclareLaunchArgument('mapping_source', default_value='lidar'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_camera', default_value='false'),
        DeclareLaunchArgument('base_mode', default_value='real'),
        DeclareLaunchArgument('use_base', default_value='true'),
        DeclareLaunchArgument('use_visual_odom', default_value='false'),
        DeclareLaunchArgument('fallback_static_odom', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('camera_enable_color', default_value='true'),
        DeclareLaunchArgument('camera_enable_ir', default_value='false'),
        DeclareLaunchArgument('camera_use_uvc', default_value='true'),
        DeclareLaunchArgument('camera_color_info_url', default_value=''),
        DeclareLaunchArgument('camera_ir_info_url', default_value=''),

        DeclareLaunchArgument('lidar_params_file', default_value=os.path.join(rb_share, 'config', 'ydlidar_X2_mapping.yaml')),
        DeclareLaunchArgument('lidar_raw_scan_topic', default_value='/scan_raw'),
        DeclareLaunchArgument('lidar_scan_topic', default_value='/scan'),
        DeclareLaunchArgument('lidar_scan_output_size', default_value='425'),
        DeclareLaunchArgument('lidar_scan_angle_min', default_value=str(-3.141592653589793)),
        DeclareLaunchArgument('lidar_scan_angle_max', default_value=str(3.141592653589793)),
        DeclareLaunchArgument('lidar_scan_normalizer_log_interval_sec', default_value='30.0'),
        DeclareLaunchArgument('lidar_tf_x', default_value='0.07'),
        DeclareLaunchArgument('lidar_tf_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_z', default_value='0.13'),
        DeclareLaunchArgument('lidar_tf_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_yaw', default_value='0.0'),
        DeclareLaunchArgument('slam_params_file', default_value=os.path.join(rb_share, 'config', 'slam_toolbox_mapping.yaml')),
        DeclareLaunchArgument('map_file', default_value=''),

        DeclareLaunchArgument('auto_mapping_drive', default_value='false'),
        DeclareLaunchArgument('auto_mapping_nav2_params_file', default_value=os.path.join(rb_share, 'config', 'nav2_mapping_params.yaml')),
        DeclareLaunchArgument('auto_mapping_bt_xml_file', default_value=os.path.join(rb_share, 'behavior_trees', 'mapping_navigate_to_pose.xml')),
        DeclareLaunchArgument('auto_mapping_map_topic', default_value='/map'),
        DeclareLaunchArgument('auto_mapping_map_frame', default_value='map'),
        DeclareLaunchArgument('auto_mapping_base_frame', default_value='base_link'),
        DeclareLaunchArgument('auto_mapping_navigate_action', default_value='/navigate_to_pose'),
        DeclareLaunchArgument('auto_mapping_tick_hz', default_value='2.0'),
        DeclareLaunchArgument('auto_mapping_startup_delay_sec', default_value='5.0'),
        DeclareLaunchArgument('auto_mapping_max_duration_sec', default_value='180.0'),
        DeclareLaunchArgument('auto_mapping_map_timeout_sec', default_value='3.0'),
        DeclareLaunchArgument('auto_mapping_tf_timeout_sec', default_value='0.10'),
        DeclareLaunchArgument('auto_mapping_goal_timeout_sec', default_value='25.0'),
        DeclareLaunchArgument('auto_mapping_frontier_min_distance', default_value='0.70'),
        DeclareLaunchArgument('auto_mapping_frontier_max_distance', default_value='2.5'),
        DeclareLaunchArgument('auto_mapping_frontier_sample_stride', default_value='2'),
        DeclareLaunchArgument('auto_mapping_min_frontier_cluster_cells', default_value='10'),
        DeclareLaunchArgument('auto_mapping_goal_approach_offset', default_value='0.55'),
        DeclareLaunchArgument('auto_mapping_goal_approach_max_offset', default_value='0.90'),
        DeclareLaunchArgument('auto_mapping_goal_approach_step', default_value='0.05'),
        DeclareLaunchArgument('auto_mapping_goal_clearance_radius', default_value='0.38'),
        DeclareLaunchArgument('auto_mapping_goal_unknown_clearance_radius', default_value='0.35'),
        DeclareLaunchArgument('auto_mapping_blacklist_radius', default_value='0.35'),
        DeclareLaunchArgument('auto_mapping_blacklist_ttl_sec', default_value='30.0'),
        DeclareLaunchArgument('auto_mapping_status_log_interval_sec', default_value='8.0'),
        DeclareLaunchArgument('auto_mapping_cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('auto_mapping_use_frontier_exploration', default_value='true'),
        DeclareLaunchArgument('auto_mapping_frontier_forward_only', default_value='true'),
        DeclareLaunchArgument('auto_mapping_publish_hz', default_value='10.0'),
        DeclareLaunchArgument('auto_mapping_scan_timeout_sec', default_value='1.0'),
        DeclareLaunchArgument('auto_mapping_linear_speed', default_value='0.10'),
        DeclareLaunchArgument('auto_mapping_turn_speed', default_value='1.65'),
        DeclareLaunchArgument('auto_mapping_emergency_stop_distance', default_value='0.25'),
        DeclareLaunchArgument('auto_mapping_front_stop_distance', default_value='0.40'),
        DeclareLaunchArgument('auto_mapping_front_resume_distance', default_value='0.48'),
        DeclareLaunchArgument('auto_mapping_front_slow_distance', default_value='0.80'),
        DeclareLaunchArgument('auto_mapping_side_emergency_stop_distance', default_value='0.12'),
        DeclareLaunchArgument('auto_mapping_side_stop_distance', default_value='0.10'),
        DeclareLaunchArgument('auto_mapping_side_resume_distance', default_value='0.18'),
        DeclareLaunchArgument('auto_mapping_front_angle_deg', default_value='35.0'),
        DeclareLaunchArgument('auto_mapping_linear_accel_limit', default_value='0.08'),
        DeclareLaunchArgument('auto_mapping_angular_accel_limit', default_value='0.50'),
        DeclareLaunchArgument('auto_mapping_cmd_deadzone_norm', default_value='0.04'),
        DeclareLaunchArgument('auto_mapping_cmd_deadzone_margin_norm', default_value='0.005'),
        DeclareLaunchArgument('auto_mapping_cmd_min_effective_norm', default_value='0.085'),

        DeclareLaunchArgument('base_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('base_baudrate', default_value='115200'),
        DeclareLaunchArgument('base_max_linear', default_value='1.20'),
        DeclareLaunchArgument('base_max_angular', default_value='19.27'),
        DeclareLaunchArgument('base_cmd_timeout', default_value='0.30'),
        DeclareLaunchArgument('base_drive_keepalive_sec', default_value='0.20'),
        DeclareLaunchArgument('base_publish_tf', default_value='true'),
        DeclareLaunchArgument('base_imu_enabled', default_value='true'),
        DeclareLaunchArgument('base_fusion_mode', default_value='none'),
        DeclareLaunchArgument('base_imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('base_imu_frame_id', default_value='base_link'),
        DeclareLaunchArgument('base_use_status_yaw', default_value='true'),
        DeclareLaunchArgument('base_status_yaw_mode', default_value='relative'),
        DeclareLaunchArgument('base_status_yaw_jump_reject_deg', default_value='25.0'),
        DeclareLaunchArgument('base_odom_feedback_source', default_value='status_twist'),
        DeclareLaunchArgument('base_wheel_radius', default_value='0.0325'),
        DeclareLaunchArgument('base_wheel_track_width', default_value='0.1250'),
        DeclareLaunchArgument('base_encoder_cpr', default_value='2340.0'),
        DeclareLaunchArgument('base_odom_linear_scale', default_value='1.0'),
        DeclareLaunchArgument('base_odom_angular_scale', default_value='1.0'),
        DeclareLaunchArgument('base_odom_angular_sign', default_value='1.0'),
        DeclareLaunchArgument('base_status_log_interval_sec', default_value='0.0'),
        DeclareLaunchArgument('base_cmd_log_interval_sec', default_value='0.0'),
        DeclareLaunchArgument('base_ekf_params_file', default_value=os.path.join(rb_share, 'config', 'ekf_base.yaml')),

        DeclareLaunchArgument('visual_odom_rgb_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('visual_odom_depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('visual_odom_camera_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('visual_odom_base_frame', default_value='base_link'),
        DeclareLaunchArgument('visual_odom_odom_frame', default_value='odom'),
        DeclareLaunchArgument('camera_scan_depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('camera_scan_info_topic', default_value='/camera/depth/camera_info'),
        DeclareLaunchArgument('camera_scan_topic', default_value='/camera/scan'),

        DeclareLaunchArgument('nav2_params_file', default_value=default_nav2_params),
        DeclareLaunchArgument('nav2_bt_xml_file', default_value=os.path.join(rb_share, 'behavior_trees', 'mapping_navigate_to_pose.xml')),
        DeclareLaunchArgument('nav2_autostart', default_value='true'),
        DeclareLaunchArgument('nav2_start', default_value='true'),

        DeclareLaunchArgument('rviz_config', default_value=os.path.join(rb_share, 'rviz', 'system.rviz')),

        OpaqueFunction(function=_validate_and_compose),
    ])
