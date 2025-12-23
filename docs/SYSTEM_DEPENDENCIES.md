# 시스템 의존성 요약

## 자동 설치 명령어

```bash
# Jetson Orin에서 전체 시스템 설치
cd colcon_ws/src
./install_dependencies.sh
```

## 패키지별 의존성

### myactuator_hardware
**핵심 기능**: MyActuator RMD 모터 하드웨어 인터페이스

**ROS 2 패키지**:
- `ros2-control` (하드웨어 인터페이스)
- `ros2-controllers` (컨트롤러들)
- `controller-manager` (컨트롤러 관리)
- `joint-trajectory-controller` (궤적 제어)
- `robot-state-publisher` (로봇 상태)
- `xacro` (URDF 처리)

**시스템 패키지**:
- `can-utils` (CAN 통신)
- `iproute2` (네트워크 인터페이스)

**커스텀 의존성**:
- `myactuator_rmd` (모터 드라이버)
- `fr_arm_description` (3D 메시)

### dual_motor_moveit_config
**핵심 기능**: MoveIt2 모션 플래닝

**ROS 2 패키지**:
- `moveit` (전체 MoveIt2 스택)
- `moveit-ros-move-group` (모션 플래닝)
- `moveit-ros-visualization` (RViz 플러그인)
- `moveit-kinematics` (역기구학)
- `moveit-planners` (경로 계획 알고리즘)
- `rviz2` (시각화)

## 시스템 요구사항

### 하드웨어:
- **Jetson Orin** (Nano/NX/AGX)
- **USB-CAN 어댑터** (SLCAN 호환)
- **MyActuator RMD 모터** 2개

### 소프트웨어:
- **Ubuntu 20.04/22.04**
- **ROS 2 Humble**
- **Python 3.8+**

### 메모리:
- **최소**: 4GB RAM
- **권장**: 8GB RAM (MoveIt 사용 시)

## 설치 후 확인

### 1. ROS 2 환경:
```bash
echo $ROS_DISTRO  # humble 출력되어야 함
```

### 2. 패키지 설치 확인:
```bash
ros2 pkg list | grep -E "(moveit|control|myactuator)"
```

### 3. CAN 인터페이스:
```bash
ip link show  # can0 인터페이스 확인
```

### 4. 권한 확인:
```bash
groups | grep dialout  # dialout 그룹 포함되어야 함
```

## 빌드 명령어

### 전체 빌드:
```bash
cd ~/colcon_ws
colcon build
source install/setup.bash
```

### 특정 패키지만:
```bash
colcon build --packages-select myactuator_hardware dual_motor_moveit_config
```

### 의존성 설치:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## 실행 순서

### 1. CAN 연결:
```bash
sudo ./setup_slcan.sh
```

### 2. 하드웨어 인터페이스:
```bash
ros2 launch myactuator_hardware dual_motor_test.launch.py
```

### 3. MoveIt (별도 터미널):
```bash
ros2 launch dual_motor_moveit_config demo.launch.py
```

## 문제 해결

### 빌드 실패:
1. 의존성 재설치: `rosdep install --from-paths src --ignore-src -r -y`
2. 클린 빌드: `rm -rf build install log && colcon build`

### CAN 연결 실패:
1. 디바이스 확인: `ls /dev/ttyACM* /dev/ttyUSB*`
2. 권한 확인: `sudo chmod 666 /dev/ttyACM0`
3. 모듈 로드: `sudo modprobe slcan`

### 메모리 부족 (Jetson Nano):
1. 스왑 생성: `sudo fallocate -l 4G /swapfile`
2. 불필요한 프로세스 종료
3. 빌드 시 `-j1` 옵션 사용

이 문서를 참조하여 Jetson Orin에서 전체 시스템을 설치하고 실행할 수 있습니다.