#!/usr/bin/env python3
"""
============================================================================
Motor Ready Position Calibration Launch File
============================================================================

모터의 현재 위치를 "ready" 위치로 파일에 저장하는 런치파일
(일자로 쭉 펴진 상태)

[사용법]
1. 전원 OFF 상태에서 로봇팔을 일자로 쭉 펴진 상태로 수동으로 이동
2. 전원 ON
3. 이 런치파일 실행:
   ros2 launch robot_bringup calibrate_ready.launch.py

[저장 위치]
~/.ros/frbot_ready_positions.yaml

============================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # ========================================================================
    # Launch Arguments
    # ========================================================================
    declared_arguments = [
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
            description="CAN interface name"
        ),
        DeclareLaunchArgument(
            "can_device",
            default_value="/dev/ttyACM0",
            description="Serial device for SLCAN"
        ),
        DeclareLaunchArgument(
            "setup_can",
            default_value="true",
            description="Run SLCAN setup script before calibration"
        ),
    ]

    can_interface = LaunchConfiguration("can_interface")
    setup_can = LaunchConfiguration("setup_can")

    # ========================================================================
    # Workspace path
    # ========================================================================
    import subprocess
    result = subprocess.run(['ros2', 'pkg', 'prefix', 'robot_bringup'], 
                          capture_output=True, text=True)
    pkg_prefix = result.stdout.strip() if result.returncode == 0 else ""
    
    if pkg_prefix:
        workspace_path = os.path.dirname(os.path.dirname(pkg_prefix))
    else:
        workspace_path = os.path.expanduser("~/frbot_ws")
    
    setup_slcan_script = os.path.join(workspace_path, "scripts", "setup_slcan.sh")
    calibrate_script = os.path.join(workspace_path, "scripts", "calibrate_ready.sh")

    # ========================================================================
    # Log messages
    # ========================================================================
    log_start = LogInfo(msg="""
================================================================================
  MOTOR READY POSITION CALIBRATION
================================================================================

이 스크립트는 현재 모터 위치를 "ready" 위치로 파일에 저장합니다.
(일자로 쭉 펴진 상태)

[주의사항]
1. 전원 OFF 상태에서 로봇팔을 일자로 쭉 펴진 상태로 수동으로 이동하세요
2. 전원을 켜고 이 스크립트를 실행하세요
3. 저장된 위치는 ~/.ros/frbot_ready_positions.yaml 에 저장됩니다

================================================================================
""")

    # ========================================================================
    # SLCAN Setup
    # ========================================================================
    setup_slcan_process = ExecuteProcess(
        cmd=['sudo', 'bash', setup_slcan_script],
        name='setup_slcan',
        output='screen',
        condition=IfCondition(setup_can),
    )

    # ========================================================================
    # Calibration Process
    # ========================================================================
    calibrate_process_after_slcan = ExecuteProcess(
        cmd=['bash', calibrate_script, 'can0'],
        name='calibrate_ready',
        output='screen',
    )

    calibrate_process_direct = ExecuteProcess(
        cmd=['bash', calibrate_script, 'can0'],
        name='calibrate_ready',
        output='screen',
    )

    # After SLCAN setup
    delay_calibrate = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=setup_slcan_process,
            on_exit=[calibrate_process_after_slcan],
        )
    )

    # Direct calibration when setup_can is false
    direct_calibrate = TimerAction(
        period=1.0,
        actions=[calibrate_process_direct],
        condition=UnlessCondition(setup_can),
    )

    # ========================================================================
    # Launch Description
    # ========================================================================
    return LaunchDescription(
        declared_arguments + [
            log_start,
            setup_slcan_process,
            delay_calibrate,
            direct_calibrate,
        ]
    )
