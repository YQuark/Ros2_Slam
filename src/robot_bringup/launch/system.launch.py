#!/usr/bin/env python3

import os

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

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(this_share, 'launch', 'sensors.launch.py')),
        launch_arguments={
            'use_lidar': 'true' if lidar_enabled else 'false',
            'use_camera': 'true' if camera_enabled else 'false',
            'lidar_params_file': LaunchConfiguration('lidar_params_file'),
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
            'publish_tf': 'false' if use_base_ekf else LaunchConfiguration('base_publish_tf').perform(context),
            'imu_enabled': LaunchConfiguration('base_imu_enabled'),
            'publish_imu': 'true' if use_base_ekf else 'false',
            'imu_topic': LaunchConfiguration('base_imu_topic'),
            'imu_frame_id': LaunchConfiguration('base_imu_frame_id'),
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

        scan_topic = LaunchConfiguration('camera_scan_topic') if mapping_source == 'camera' else '/scan'
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
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(this_share, 'launch', 'nav2.launch.py')),
                launch_arguments={
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'params_file': LaunchConfiguration('nav2_params_file'),
                    'autostart': LaunchConfiguration('nav2_autostart'),
                }.items(),
            )
        )

    actions.append(viz_launch)
    return actions


def generate_launch_description():
    rb_share = get_package_share_directory('robot_bringup')
    ydlidar_share = get_package_share_directory('ydlidar_ros2_driver')

    default_nav2_params = '/opt/ros/foxy/share/nav2_bringup/params/nav2_params.yaml'
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

        DeclareLaunchArgument('lidar_params_file', default_value=os.path.join(ydlidar_share, 'params', 'X2.yaml')),
        DeclareLaunchArgument('lidar_tf_x', default_value='0.07'),
        DeclareLaunchArgument('lidar_tf_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_z', default_value='0.13'),
        DeclareLaunchArgument('lidar_tf_roll', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_pitch', default_value='0.0'),
        DeclareLaunchArgument('lidar_tf_yaw', default_value='0.0'),
        DeclareLaunchArgument('slam_params_file', default_value=os.path.join(rb_share, 'config', 'slam_toolbox_mapping.yaml')),
        DeclareLaunchArgument('map_file', default_value=''),

        DeclareLaunchArgument('base_port', default_value='auto'),
        DeclareLaunchArgument('base_baudrate', default_value='115200'),
        DeclareLaunchArgument('base_max_linear', default_value='0.50'),
        DeclareLaunchArgument('base_max_angular', default_value='1.50'),
        DeclareLaunchArgument('base_publish_tf', default_value='true'),
        DeclareLaunchArgument('base_imu_enabled', default_value='true'),
        DeclareLaunchArgument('base_fusion_mode', default_value='none'),
        DeclareLaunchArgument('base_imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('base_imu_frame_id', default_value='base_link'),
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
        DeclareLaunchArgument('nav2_autostart', default_value='true'),

        DeclareLaunchArgument('rviz_config', default_value=os.path.join(rb_share, 'rviz', 'system.rviz')),

        OpaqueFunction(function=_validate_and_compose),
    ])
