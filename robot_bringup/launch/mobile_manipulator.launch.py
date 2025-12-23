#!/usr/bin/env python3
"""
Mobile Manipulator Full System Launch
=====================================

실제 하드웨어 + MoveIt2 통합 런치 파일
- MD Motor 4WD (모바일 베이스)
- MyActuator RMD (4-DOF 암)
- MoveIt2 (암 모션 플래닝)
- robot_localization (EKF)

사용법:
    # 전체 시스템 실행 (RViz 포함)
    ros2 launch robot_bringup mobile_manipulator.launch.py

    # RViz 없이 실행
    ros2 launch robot_bringup mobile_manipulator.launch.py rviz:=false

    # 모바일 베이스만 (암 제외)
    ros2 launch robot_bringup mobile_manipulator.launch.py use_arm:=false

    # 암만 (모바일 베이스 제외)
    ros2 launch robot_bringup mobile_manipulator.launch.py use_mobile:=false
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
    GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    """YAML 파일 로드"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as f:
            return yaml.safe_load(f)
    except:
        return None


def generate_launch_description():
    # ========================================================================
    # Package paths
    # ========================================================================
    robot_bringup_pkg = get_package_share_directory('robot_bringup')
    robot_description_pkg = get_package_share_directory('robot_description')
    md_motor_pkg = get_package_share_directory('md_motor_hardware')
    myactuator_pkg = get_package_share_directory('myactuator_hardware')
    moveit_pkg = get_package_share_directory('mobile_manipulator_moveit_config')

    # ========================================================================
    # Launch Arguments
    # ========================================================================
    declared_arguments = [
        # System options
        DeclareLaunchArgument('rviz', default_value='true',
            description='Launch RViz'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
            description='Use simulation time'),
        DeclareLaunchArgument('log_level', default_value='info',
            description='Log level'),

        # Component selection
        DeclareLaunchArgument('use_mobile', default_value='true',
            description='Enable mobile base (MD Motor 4WD)'),
        DeclareLaunchArgument('use_arm', default_value='true',
            description='Enable manipulator arm (MyActuator RMD)'),
        DeclareLaunchArgument('use_moveit', default_value='true',
            description='Enable MoveIt2 for arm control'),
        DeclareLaunchArgument('use_ekf', default_value='true',
            description='Enable EKF for localization'),

        # Mobile base parameters
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
            description='MD Motor serial port'),
        DeclareLaunchArgument('baudrate', default_value='57600',
            description='MD Motor baudrate'),

        # Arm parameters
        DeclareLaunchArgument('can_interface', default_value='can0',
            description='CAN interface for arm motors'),
        DeclareLaunchArgument('motor1_id', default_value='1',
            description='Arm motor 1 ID'),
        DeclareLaunchArgument('motor2_id', default_value='2',
            description='Arm motor 2 ID'),
        DeclareLaunchArgument('motor3_id', default_value='3',
            description='Arm motor 3 ID'),
        DeclareLaunchArgument('motor4_id', default_value='4',
            description='Arm motor 4 ID'),
    ]

    # Get configurations
    rviz_arg = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    use_mobile = LaunchConfiguration('use_mobile')
    use_arm = LaunchConfiguration('use_arm')
    use_moveit = LaunchConfiguration('use_moveit')
    use_ekf = LaunchConfiguration('use_ekf')

    # ========================================================================
    # Robot Description (URDF)
    # ========================================================================
    urdf_path = os.path.join(robot_description_pkg, 'urdf', 'mobile_manipulator.xacro')
    robot_description_content = Command(['xacro ', urdf_path])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # ========================================================================
    # MoveIt Configuration
    # ========================================================================
    srdf_path = os.path.join(moveit_pkg, 'config', 'mobile_manipulator.srdf')
    with open(srdf_path, 'r') as f:
        robot_description_semantic = {'robot_description_semantic': f.read()}

    kinematics_yaml = load_yaml('mobile_manipulator_moveit_config', 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml('mobile_manipulator_moveit_config', 'config/joint_limits.yaml')
    moveit_controllers_yaml = load_yaml('mobile_manipulator_moveit_config', 'config/moveit_controllers.yaml')

    robot_description_kinematics = {'robot_description_kinematics': kinematics_yaml}
    robot_description_planning = {'robot_description_planning': joint_limits_yaml}

    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_controllers_yaml.get('moveit_simple_controller_manager', {}),
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }

    ompl_planning_pipeline = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/ResolveConstraintFrames '
                               'default_planner_request_adapters/FixWorkspaceBounds '
                               'default_planner_request_adapters/FixStartStateBounds '
                               'default_planner_request_adapters/FixStartStateCollision '
                               'default_planner_request_adapters/FixStartStatePathConstraints',
            'response_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization',
            'start_state_max_bounds_error': 0.1,
        }
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    planning_scene_monitor = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # ========================================================================
    # Controller config paths
    # ========================================================================
    mobile_controller_yaml = os.path.join(robot_bringup_pkg, 'config', 'md_4wd_controllers_no_odom_tf.yaml')
    arm_controller_yaml = os.path.join(moveit_pkg, 'config', 'ros2_controllers.yaml')
    ekf_yaml = os.path.join(robot_bringup_pkg, 'config', 'ekf.yaml')

    # ========================================================================
    # Nodes
    # ========================================================================

    # Robot State Publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    # ros2_control for arm
    arm_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, arm_controller_yaml, {'use_sim_time': use_sim_time}],
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(use_arm),
    )

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    # Arm Controller
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(use_arm),
    )

    # Gripper Controller
    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(use_arm),
    )

    # Delayed controller spawners
    delay_joint_state_broadcaster = TimerAction(
        period=2.0,
        actions=[joint_state_broadcaster_spawner],
    )

    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    delay_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    # MoveIt Move Group
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline,
            trajectory_execution,
            planning_scene_monitor,
            moveit_controllers,
            {'use_sim_time': use_sim_time},
        ],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(PythonExpression(["'", use_arm, "' == 'true' and '", use_moveit, "' == 'true'"])),
    )

    delay_move_group = TimerAction(period=6.0, actions=[move_group_node])

    # EKF Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml],
        condition=IfCondition(PythonExpression(["'", use_mobile, "' == 'true' and '", use_ekf, "' == 'true'"])),
    )

    # RViz
    rviz_config = os.path.join(moveit_pkg, 'config', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            {'use_sim_time': use_sim_time},
        ],
        condition=IfCondition(rviz_arg),
    )

    delay_rviz = TimerAction(period=8.0, actions=[rviz_node])

    # ========================================================================
    # Launch Description
    # ========================================================================
    return LaunchDescription(
        declared_arguments + [
            robot_state_pub,
            arm_control_node,
            delay_joint_state_broadcaster,
            delay_arm_controller,
            delay_gripper_controller,
            delay_move_group,
            ekf_node,
            delay_rviz,
        ]
    )
