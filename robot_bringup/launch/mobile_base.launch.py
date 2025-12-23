#!/usr/bin/env python3
"""
============================================================================
Robot Bringup Launch File (Real Hardware)
============================================================================

실제 하드웨어용 통합 런치 파일
- MD Motor 4WD Hardware Interface (ros2_control)
- robot_localization (EKF) - odom TF 발행
- diff_drive_controller - odom TF 비활성화 (중복 방지)

[토픽 흐름]
/diff_drive_controller/cmd_vel_unstamped (Twist)
    ↓
MD4WDHardware (4륜 모터 제어)
    ↓
/diff_drive_controller/odom (Odometry) ← TF 발행 안 함
    ↓
robot_localization (EKF)
    ↓
/odometry/filtered + odom→base_link TF 발행

[사용법]
ros2 launch robot_bringup robot_bringup.launch.py

[파라미터 오버라이드]
ros2 launch robot_bringup robot_bringup.launch.py use_ekf:=false  # EKF 없이 실행

============================================================================
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml_params(yaml_path):
    """YAML 파일에서 파라미터 로드"""
    with open(yaml_path, 'r') as f:
        params = yaml.safe_load(f)
    return params.get('md_4wd_hardware', {}).get('ros__parameters', {})


def generate_launch_description():
    # ========================================================================
    # 패키지 경로
    # ========================================================================
    md_motor_hardware_pkg = get_package_share_directory('md_motor_hardware')
    robot_bringup_pkg = get_package_share_directory('robot_bringup')
    
    # ========================================================================
    # 설정 파일 경로
    # ========================================================================
    hardware_yaml = os.path.join(md_motor_hardware_pkg, 'config', 'md_4wd_hardware.yaml')
    # EKF 사용 시 odom TF를 비활성화한 컨트롤러 설정 사용
    controller_yaml = os.path.join(robot_bringup_pkg, 'config', 'md_4wd_controllers_no_odom_tf.yaml')
    # EKF 미사용 시 기본 컨트롤러 설정 사용
    controller_yaml_with_tf = os.path.join(md_motor_hardware_pkg, 'config', 'md_4wd_controllers.yaml')
    ekf_yaml = os.path.join(robot_bringup_pkg, 'config', 'ekf.yaml')
    urdf_file = os.path.join(md_motor_hardware_pkg, 'urdf', 'md_4wd_robot.urdf.xacro')
    
    # YAML에서 기본값 로드
    yaml_params = load_yaml_params(hardware_yaml)
    
    # ========================================================================
    # Launch Arguments
    # ========================================================================
    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf',
        default_value='true',
        description='Whether to use robot_localization EKF (disables diff_drive odom TF)'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value=str(yaml_params.get('port', '/dev/ttyUSB0')),
        description='Serial port for MD motor driver'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value=str(yaml_params.get('baudrate', 57600)),
        description='Serial baudrate'
    )
    
    front_driver_id_arg = DeclareLaunchArgument(
        'front_driver_id',
        default_value=str(yaml_params.get('front_driver_id', 1)),
        description='Front motor driver ID'
    )
    
    rear_driver_id_arg = DeclareLaunchArgument(
        'rear_driver_id',
        default_value=str(yaml_params.get('rear_driver_id', 2)),
        description='Rear motor driver ID'
    )
    
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value=str(yaml_params.get('wheel_radius', 0.05)),
        description='Wheel radius in meters'
    )
    
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation',
        default_value=str(yaml_params.get('wheel_separation', 0.3)),
        description='Distance between left and right wheels'
    )
    
    wheelbase_arg = DeclareLaunchArgument(
        'wheelbase',
        default_value=str(yaml_params.get('wheelbase', 0.3)),
        description='Distance between front and rear axles'
    )
    
    gear_ratio_arg = DeclareLaunchArgument(
        'gear_ratio',
        default_value=str(yaml_params.get('gear_ratio', 15)),
        description='Motor gear ratio'
    )
    
    poles_arg = DeclareLaunchArgument(
        'poles',
        default_value=str(yaml_params.get('poles', 10)),
        description='Number of motor poles'
    )
    
    # ========================================================================
    # Robot Description (URDF)
    # ========================================================================
    robot_description_content = Command([
        'xacro ', urdf_file,
        ' port:=', LaunchConfiguration('port'),
        ' baudrate:=', LaunchConfiguration('baudrate'),
        ' front_driver_id:=', LaunchConfiguration('front_driver_id'),
        ' rear_driver_id:=', LaunchConfiguration('rear_driver_id'),
        ' id_mdui:=', str(yaml_params.get('id_mdui', 184)),
        ' id_mdt:=', str(yaml_params.get('id_mdt', 183)),
        ' wheel_radius:=', LaunchConfiguration('wheel_radius'),
        ' wheel_separation:=', LaunchConfiguration('wheel_separation'),
        ' wheelbase:=', LaunchConfiguration('wheelbase'),
        ' gear_ratio:=', LaunchConfiguration('gear_ratio'),
        ' poles:=', LaunchConfiguration('poles'),
    ])
    
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    # ========================================================================
    # Nodes - Robot State Publisher
    # ========================================================================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )
    
    # ========================================================================
    # Nodes - Controller Manager (EKF 사용 시 - odom TF 비활성화)
    # ========================================================================
    controller_manager_with_ekf = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            controller_yaml,
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_ekf')),
    )
    
    # ========================================================================
    # Nodes - Controller Manager (EKF 미사용 시 - odom TF 활성화)
    # ========================================================================
    controller_manager_without_ekf = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            controller_yaml_with_tf,
        ],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('use_ekf')),
    )
    
    # ========================================================================
    # Controller Spawners
    # ========================================================================
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )
    
    # ========================================================================
    # Nodes - Robot Localization (EKF)
    # ========================================================================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml],
        condition=IfCondition(LaunchConfiguration('use_ekf')),
    )
    
    # ========================================================================
    # Launch Description
    # ========================================================================
    return LaunchDescription([
        # Arguments
        use_ekf_arg,
        port_arg,
        baudrate_arg,
        front_driver_id_arg,
        rear_driver_id_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        wheelbase_arg,
        gear_ratio_arg,
        poles_arg,
        
        # Nodes
        robot_state_publisher_node,
        controller_manager_with_ekf,
        controller_manager_without_ekf,
        joint_state_broadcaster_spawner,
        delayed_diff_drive_spawner,
        ekf_node,
    ])
