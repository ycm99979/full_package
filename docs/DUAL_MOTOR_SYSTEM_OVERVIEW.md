# Dual Motor System Overview

## ⚠️ 중요: 초기 위치 동기화 문제

**현재 상황:**
- URDF에서 `initial_value`가 joint1=0.0, joint2=0.0으로 설정됨
- `dual_motor_test.launch.py` 실행 시 실제 모터가 0, 0 위치로 이동함
- MoveIt의 `initial_positions.yaml`도 0, 0으로 설정되어 있음
- **문제**: MoveIt은 가상의 0, 0에서 시작하지만 실제 하드웨어는 이미 0, 0으로 이동한 상태

**해결 방법:**
1. 현재 실제 joint 상태 확인:
   ```bash
   ros2 topic echo /joint_states --once
   ```

2. 확인된 실제 위치를 MoveIt 설정에 반영:
   - `src/dual_motor_moveit_config/config/initial_positions.yaml` 수정
   - 실제 하드웨어 위치와 MoveIt 시작 위치를 일치시킴

3. 또는 하드웨어 시작 시 현재 위치를 유지하도록 URDF 수정

---

## 시스템 구조

### 1. 하드웨어 패키지 (`myactuator_hardware`)
MyActuator RMD 모터 2개를 제어하는 ros2_control 하드웨어 인터페이스

#### 주요 파일들:
- **`config/dual_motor_test.urdf.xacro`**: 로봇 URDF (FR Arm 기반 2-DOF 구조)
- **`config/dual_motor_controllers.yaml`**: ros2_control 컨트롤러 설정
- **`launch/dual_motor_test.launch.py`**: 하드웨어 인터페이스 실행
- **`scripts/dual_motor_teleop.py`**: 키보드 텔레오프 (기존)
- **`scripts/dual_motor_trajectory_teleop.py`**: 트래젝토리 기반 텔레오프 (새로 추가)

#### 로봇 구조:
```
world → base_footprint → base_link → actuator1 → [joint1] → actuator2 → [joint2] → upperarm_link → end_effector_link
```

- **joint1**: Z축 회전 (continuous, base rotation)
- **joint2**: Y축 회전 (revolute, ±180°, shoulder pitch)

### 2. MoveIt 패키지 (`dual_motor_moveit_config`)
MoveIt Setup Assistant로 생성된 모션 플래닝 패키지

#### 주요 파일들:
- **`config/dual_motor_robot.srdf`**: 시맨틱 로봇 설명
- **`config/moveit_controllers.yaml`**: MoveIt 컨트롤러 매핑
- **`config/kinematics.yaml`**: 역기구학 솔버 설정
- **`launch/demo.launch.py`**: MoveIt 데모 실행

#### Planning Group:
- **`arm`**: joint1, joint2 포함
- **End Effector**: end_effector_link

## 컨트롤러 구조

### ros2_control 컨트롤러:
1. **`joint_state_broadcaster`**: 조인트 상태 퍼블리시
2. **`joint_trajectory_controller`**: 트래젝토리 실행 (MoveIt 호환)

### 토픽 구조:
```
/joint_states                                    # 조인트 상태
/joint_trajectory_controller/joint_trajectory    # 트래젝토리 명령
/joint_trajectory_controller/follow_joint_trajectory  # 액션 서버
```

## 실행 방법

### 1. CAN 연결 설정
```bash
sudo ./setup_slcan.sh
```

### 2. 하드웨어 인터페이스 실행
```bash
# 터미널 1
source install/setup.bash
ros2 launch myactuator_hardware dual_motor_test.launch.py
```

### 3. MoveIt 실행
```bash
# 터미널 2  
source install/setup.bash
ros2 launch dual_motor_moveit_config demo.launch.py
```

### 4. 텔레오프 실행 (선택사항)
```bash
# 터미널 3 - 기존 방식 (현재 작동 안함)
ros2 run myactuator_hardware dual_motor_teleop.py

# 또는 트래젝토리 방식 (새로 추가)
ros2 run myactuator_hardware dual_motor_trajectory_teleop.py
```

## 제어 방법

### MoveIt을 통한 제어:
1. RViz에서 Planning Scene 확인
2. Interactive Marker로 목표 위치 설정
3. "Plan & Execute" 클릭
4. 실제 로봇이 계획된 경로로 이동

### 직접 트래젝토리 명령:
```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
header:
  stamp: {sec: 0, nanosec: 0}
joint_names: ['joint1', 'joint2']
points:
- positions: [0.5, -0.5]
  time_from_start: {sec: 2, nanosec: 0}
"
```

## 하드웨어 설정

### CAN 설정:
- **인터페이스**: can0 (기본값)
- **보드레이트**: 1Mbps
- **모터 ID**: motor1=1, motor2=2

### 모터 사양:
- **토크 상수**: 0.32 Nm/A
- **최대 속도**: 4.19 rad/s
- **최대 토크**: 1.5 Nm

## 문제 해결

### 컨트롤러 상태 확인:
```bash
ros2 control list_controllers
```

### CAN 연결 확인:
```bash
ip link show can0
candump can0
```

### 토픽 확인:
```bash
ros2 topic list | grep -E "(joint|trajectory)"
ros2 topic echo /joint_states
```

### 로그 확인:
```bash
ros2 launch myactuator_hardware dual_motor_test.launch.py --ros-args --log-level DEBUG
```

## 파일 구조

```
src/
├── myactuator_hardware/
│   ├── config/
│   │   ├── dual_motor_test.urdf.xacro      # 로봇 URDF
│   │   └── dual_motor_controllers.yaml     # 컨트롤러 설정
│   ├── launch/
│   │   └── dual_motor_test.launch.py       # 하드웨어 실행
│   └── scripts/
│       ├── dual_motor_teleop.py            # 기존 텔레오프
│       └── dual_motor_trajectory_teleop.py # 트래젝토리 텔레오프
│
├── dual_motor_moveit_config/
│   ├── config/
│   │   ├── dual_motor_robot.srdf           # 시맨틱 설명
│   │   ├── moveit_controllers.yaml         # MoveIt 컨트롤러
│   │   └── kinematics.yaml                 # 역기구학 설정
│   └── launch/
│       └── demo.launch.py                  # MoveIt 데모
│
└── fr_arm_description/                     # 3D 메시 파일들
    └── meshes/
        ├── actuator1.stl
        ├── actuator2.stl
        └── upperarm_link.stl
```

## 주요 특징

1. **실제 하드웨어 연동**: CAN 통신으로 MyActuator RMD 모터 제어
2. **MoveIt 호환**: 경로 계획 및 충돌 회피
3. **실시간 제어**: ros2_control 기반 200Hz 업데이트
4. **3D 시각화**: FR Arm 메시 기반 정확한 시각화
5. **다중 제어 방식**: MoveIt, 트래젝토리, 텔레오프 지원