# Jetson Orin 설치 가이드

## 1. 시스템 요구사항

- **Jetson Orin** (Nano/NX/AGX)
- **Ubuntu 20.04/22.04** (JetPack 5.x)
- **ROS 2 Humble**

## 2. 자동 설치 스크립트

### 전체 시스템 설치:
```bash
# 이 저장소를 클론한 후
cd colcon_ws/src
./install_dependencies.sh
```

### 수동 설치 (단계별):

#### Step 1: ROS 2 Humble 설치
```bash
# ROS 2 Humble 설치 (이미 설치되어 있다면 스킵)
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-humble-desktop
```

#### Step 2: 개발 도구 설치
```bash
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep python3-vcstool
sudo rosdep init
rosdep update
```

#### Step 3: ROS 2 Control 설치
```bash
sudo apt install ros-humble-ros2-control
sudo apt install ros-humble-ros2-controllers
sudo apt install ros-humble-controller-manager
sudo apt install ros-humble-joint-state-broadcaster
sudo apt install ros-humble-joint-trajectory-controller
sudo apt install ros-humble-forward-command-controller
```

#### Step 4: MoveIt2 설치
```bash
sudo apt install ros-humble-moveit
sudo apt install ros-humble-moveit-ros-move-group
sudo apt install ros-humble-moveit-ros-visualization
sudo apt install ros-humble-moveit-kinematics
sudo apt install ros-humble-moveit-planners
sudo apt install ros-humble-moveit-simple-controller-manager
sudo apt install ros-humble-moveit-configs-utils
sudo apt install ros-humble-moveit-setup-assistant
```

#### Step 5: 로봇 관련 패키지
```bash
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-xacro
sudo apt install ros-humble-urdf
```

#### Step 6: 시각화 및 도구
```bash
sudo apt install ros-humble-rviz2
sudo apt install ros-humble-rviz-common
sudo apt install ros-humble-rviz-default-plugins
sudo apt install ros-humble-tf2-ros
sudo apt install ros-humble-tf2-geometry-msgs
```

#### Step 7: CAN 유틸리티
```bash
sudo apt install can-utils
sudo apt install iproute2
```

#### Step 8: Python 의존성
```bash
sudo apt install python3-pip
pip3 install numpy
```

## 3. 워크스페이스 설정

### 의존성 자동 설치:
```bash
cd ~/colcon_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 빌드:
```bash
colcon build
source install/setup.bash
```

## 4. 하드웨어 설정

### CAN 인터페이스 설정:
```bash
# CAN 모듈 로드
sudo modprobe can
sudo modprobe can_raw
sudo modprobe slcan

# 권한 설정
sudo usermod -a -G dialout $USER
# 재로그인 필요
```

### USB-CAN 어댑터 설정:
```bash
# 디바이스 확인
ls /dev/ttyACM* /dev/ttyUSB*

# SLCAN 설정 스크립트 실행 권한
chmod +x setup_slcan.sh stop_slcan.sh
```

## 5. 테스트

### 기본 테스트:
```bash
# 터미널 1: 하드웨어 인터페이스
ros2 launch myactuator_hardware dual_motor_test.launch.py

# 터미널 2: MoveIt
ros2 launch dual_motor_moveit_config demo.launch.py
```

### 연결 확인:
```bash
# CAN 인터페이스 확인
ip link show can0

# 컨트롤러 상태 확인
ros2 control list_controllers

# 토픽 확인
ros2 topic list
```

## 6. 문제 해결

### 일반적인 문제들:

#### CAN 연결 실패:
```bash
# 디바이스 권한 확인
ls -l /dev/ttyACM0
sudo chmod 666 /dev/ttyACM0

# 또는 udev 규칙 추가
sudo nano /etc/udev/rules.d/99-can.rules
# 내용: SUBSYSTEM=="tty", ATTRS{idVendor}=="your_vendor_id", MODE="0666"
sudo udevadm control --reload-rules
```

#### 메모리 부족 (Jetson Nano):
```bash
# 스왑 파일 생성
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

#### 빌드 실패:
```bash
# 의존성 재설치
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 클린 빌드
rm -rf build install log
colcon build
```

## 7. 성능 최적화 (Jetson)

### CPU 성능 모드:
```bash
# 최대 성능 모드
sudo nvpmodel -m 0
sudo jetson_clocks
```

### 메모리 최적화:
```bash
# 불필요한 서비스 비활성화
sudo systemctl disable cups
sudo systemctl disable bluetooth
```

## 8. 자동 시작 설정

### systemd 서비스 생성:
```bash
sudo nano /etc/systemd/system/dual-motor-robot.service
```

```ini
[Unit]
Description=Dual Motor Robot System
After=network.target

[Service]
Type=simple
User=your_username
Environment=ROS_DOMAIN_ID=0
WorkingDirectory=/home/your_username/colcon_ws
ExecStart=/bin/bash -c "source install/setup.bash && ros2 launch myactuator_hardware dual_motor_test.launch.py"
Restart=always

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable dual-motor-robot.service
sudo systemctl start dual-motor-robot.service
```

이 가이드를 따라하면 Jetson Orin에서 전체 시스템을 자동으로 설치하고 실행할 수 있습니다!