#!/usr/bin/env python3
"""
============================================================================
Motor Home Position Calibration Launch File
============================================================================

모터의 현재 위치를 홈(0) 위치로 설정하는 캘리브레이션 런치파일

[사용법]
1. 전원 OFF 상태에서 로봇팔을 원하는 초기 위치로 수동으로 이동
2. 전원 ON
3. 이 런치파일 실행:
   ros2 launch robot_bringup calibrate_home.launch.py

4. 캘리브레이션 완료 후 mm_moveit_hardware.launch.py 실행

[주의]
- 이 런치파일은 현재 모터 위치를 0으로 설정합니다
- 캘리브레이션 후에는 모터를 재시작해야 적용됩니다

============================================================================
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    TimerAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


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
            "can_baudrate",
            default_value="1000000",
            description="CAN baudrate"
        ),
        DeclareLaunchArgument(
            "setup_can",
            default_value="true",
            description="Run SLCAN setup script before calibration"
        ),
    ]

    can_interface = LaunchConfiguration("can_interface")
    can_device = LaunchConfiguration("can_device")
    can_baudrate = LaunchConfiguration("can_baudrate")
    setup_can = LaunchConfiguration("setup_can")

    # ========================================================================
    # Workspace path
    # ========================================================================
    # 소스 디렉터리 기준으로 스크립트 경로 찾기
    import subprocess
    result = subprocess.run(['ros2', 'pkg', 'prefix', 'robot_bringup'], capture_output=True, text=True)
    pkg_prefix = result.stdout.strip() if result.returncode == 0 else ""
    
    # 워크스페이스 루트 찾기 (install 폴더의 상위)
    if pkg_prefix:
        workspace_path = os.path.dirname(os.path.dirname(pkg_prefix))
    else:
        workspace_path = os.path.expanduser("~/frbot_ws")
    
    setup_slcan_script = os.path.join(workspace_path, "scripts", "setup_slcan.sh")
    calibrate_script = os.path.join(workspace_path, "scripts", "calibrate_home.sh")

    # ========================================================================
    # Log messages
    # ========================================================================
    log_start = LogInfo(msg="""
================================================================================
  MOTOR HOME POSITION CALIBRATION
================================================================================

이 스크립트는 현재 모터 위치를 홈(0) 위치로 설정합니다.

[주의사항]
1. 전원 OFF 상태에서 로봇팔을 원하는 초기 위치로 수동으로 이동하세요
2. 전원을 켜고 이 스크립트를 실행하세요
3. 캘리브레이션이 완료되면 모터의 현재 위치가 0으로 설정됩니다

================================================================================
""")

    # ========================================================================
    # SLCAN Setup
    # ========================================================================
    setup_slcan_process = ExecuteProcess(
        cmd=['sudo', 'bash', setup_slcan_script, can_device, can_baudrate],
        name='setup_slcan',
        output='screen',
        condition=IfCondition(setup_can),
    )

    # ========================================================================
    # Calibration Process
    # ========================================================================
    calibrate_process_after_slcan = ExecuteProcess(
        cmd=['bash', calibrate_script, 'can0'],
        name='calibrate_home',
        output='screen',
    )

    calibrate_process_direct = ExecuteProcess(
        cmd=['bash', calibrate_script, 'can0'],
        name='calibrate_home',
        output='screen',
    )

    # Delay calibration after SLCAN setup (when setup_can=true)
    delay_calibrate = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=setup_slcan_process,
            on_exit=[calibrate_process_after_slcan],
        )
    )

    # For when setup_can is false, run calibration directly after a short delay
    from launch.conditions import UnlessCondition
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
