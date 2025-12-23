#!/usr/bin/env python3
"""
============================================================================
MD Motor 4WD Hardware Interface Test Launch File
============================================================================

4륜 개별 제어 테스트용 런치 파일
- ros2_control 하드웨어 인터페이스 로드
- diff_drive_controller 활성화
- Joint State Broadcaster 활성화

[파라미터 설정]
config/md_4wd_hardware.yaml 파일을 수정하여 설정 변경
- 시리얼 포트, 드라이버 ID
- 휠 반지름, 휠 간격, 기어비 등

[사용법]
ros2 launch md_motor_hardware md_4wd_test.launch.py

[런타임 파라미터 오버라이드]
ros2 launch md_motor_hardware md_4wd_test.launch.py wheel_radius:=0.06

============================================================================
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml_params(yaml_path):
    """YAML 파일에서 파라미터 로드"""
    with open(yaml_path, 'r') as f:
        params = yaml.safe_load(f)
    # md_4wd_hardware.ros__parameters 안의 값을 반환
    return params.get('md_4wd_hardware', {}).get('ros__parameters', {})


def generate_launch_description():
    # ========================================================================
    # 패키지 경로
    # ========================================================================
    pkg_share = get_package_share_directory('md_motor_hardware')
    
    # ========================================================================
    # YAML에서 기본값 로드
    # ========================================================================
    hardware_yaml = os.path.join(pkg_share, 'config', 'md_4wd_hardware.yaml')
    yaml_params = load_yaml_params(hardware_yaml)
    
    # ========================================================================
    # Launch Arguments (YAML 기본값 사용, 런타임 오버라이드 가능)
    # ========================================================================
    
    # 시리얼 통신
    port_arg = DeclareLaunchArgument(
        'port',
        default_value=str(yaml_params.get('port', '/dev/ttyMotor')),
        description='Serial port for MD motor driver'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value=str(yaml_params.get('baudrate', 57600)),
        description='Serial baudrate'
    )
    
    # 모터 드라이버 ID
    front_driver_id_arg = DeclareLaunchArgument(
        'front_driver_id',
        default_value=str(yaml_params.get('front_driver_id', 1)),
        description='Front motor driver ID (for FL, FR wheels)'
    )
    
    rear_driver_id_arg = DeclareLaunchArgument(
        'rear_driver_id',
        default_value=str(yaml_params.get('rear_driver_id', 2)),
        description='Rear motor driver ID (for RL, RR wheels)'
    )
    
    id_mdui_arg = DeclareLaunchArgument(
        'id_mdui',
        default_value=str(yaml_params.get('id_mdui', 184)),
        description='MDUI protocol ID'
    )
    
    id_mdt_arg = DeclareLaunchArgument(
        'id_mdt',
        default_value=str(yaml_params.get('id_mdt', 183)),
        description='MDT protocol ID'
    )
    
    # 로봇 기구학
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value=str(yaml_params.get('wheel_radius', 0.05)),
        description='Wheel radius in meters'
    )
    
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation',
        default_value=str(yaml_params.get('wheel_separation', 0.3)),
        description='Distance between left and right wheels in meters'
    )
    
    wheelbase_arg = DeclareLaunchArgument(
        'wheelbase',
        default_value=str(yaml_params.get('wheelbase', 0.3)),
        description='Distance between front and rear axles in meters'
    )
    
    # 모터/엔코더
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
    # URDF 파일 처리 (YAML 파라미터 → xacro args)
    # ========================================================================
    urdf_file = os.path.join(pkg_share, 'urdf', 'md_4wd_robot.urdf.xacro')
    
    robot_description_content = Command([
        'xacro ', urdf_file,
        # 시리얼 통신
        ' port:=', LaunchConfiguration('port'),
        ' baudrate:=', LaunchConfiguration('baudrate'),
        # 모터 드라이버 ID
        ' front_driver_id:=', LaunchConfiguration('front_driver_id'),
        ' rear_driver_id:=', LaunchConfiguration('rear_driver_id'),
        ' id_mdui:=', LaunchConfiguration('id_mdui'),
        ' id_mdt:=', LaunchConfiguration('id_mdt'),
        # 로봇 기구학
        ' wheel_radius:=', LaunchConfiguration('wheel_radius'),
        ' wheel_separation:=', LaunchConfiguration('wheel_separation'),
        ' wheelbase:=', LaunchConfiguration('wheelbase'),
        # 모터/엔코더
        ' gear_ratio:=', LaunchConfiguration('gear_ratio'),
        ' poles:=', LaunchConfiguration('poles'),
    ])
    
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    # ========================================================================
    # 컨트롤러 설정 파일
    # ========================================================================
    controller_config = os.path.join(pkg_share, 'config', 'md_4wd_controllers.yaml')
    
    # ========================================================================
    # Nodes
    # ========================================================================
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )
    
    # Controller Manager
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            controller_config,
        ],
        output='screen',
    )
    
    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Diff Drive Controller Spawner
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # ========================================================================
    # Event Handlers (순차 실행)
    # ========================================================================
    
    # Joint State Broadcaster가 먼저 활성화된 후 Diff Drive Controller 활성화
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_spawner],
        )
    )
    
    # ========================================================================
    # Launch Description
    # ========================================================================
    return LaunchDescription([
        # Arguments
        port_arg,
        baudrate_arg,
        front_driver_id_arg,
        rear_driver_id_arg,
        id_mdui_arg,
        id_mdt_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        wheelbase_arg,
        gear_ratio_arg,
        poles_arg,
        
        # Nodes
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_broadcaster_spawner,
        delayed_diff_drive_spawner,
    ])
