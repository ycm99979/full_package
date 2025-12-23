# Behavior Tree 기반 자율 로봇 시스템 가이드

## 시스템 개요

**목표**: 자동차가 움직여서 물체를 집고, 다른 위치로 이동하여 놓는 자율 시스템

**구성 요소**:
- 이동 로봇 (자동차)
- 로봇 팔 (dual motor system)
- 행동트리 (BehaviorTree.CPP)
- MoveIt (매니퓰레이션)
- Navigation2 (이동)

## 1. 전체 아키텍처

```
Behavior Tree Executive
├── Navigation Actions (자동차 이동)
├── Manipulation Actions (로봇 팔 제어)
├── Perception Actions (물체 인식)
└── Gripper Actions (집기/놓기)
```

## 2. 필요한 ROS2 패키지

### 행동트리 관련:
```bash
sudo apt install ros-humble-behaviortree-cpp-v3
sudo apt install ros-humble-groot
```

### Navigation2 (자동차 이동):
```bash
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

### 추가 패키지:
```bash
sudo apt install ros-humble-tf2-tools
sudo apt install ros-humble-robot-localization
```

## 3. 행동트리 구조 설계

### 메인 트리 (Main Tree):
```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="PickAndPlace">
      <Action ID="NavigateToObject" target="pickup_location"/>
      <Action ID="ApproachObject"/>
      <Action ID="PickObject"/>
      <Action ID="NavigateToDropoff" target="dropoff_location"/>
      <Action ID="PlaceObject"/>
      <Action ID="NavigateToHome" target="home_location"/>
    </Sequence>
  </BehaviorTree>
</root>
```

### 세부 액션들:

#### Navigation Actions:
- `NavigateToObject`: 물체 위치로 이동
- `NavigateToDropoff`: 목표 위치로 이동
- `NavigateToHome`: 홈 위치로 복귀

#### Manipulation Actions:
- `ApproachObject`: 로봇 팔을 물체에 접근
- `PickObject`: 물체 집기
- `PlaceObject`: 물체 놓기

## 4. 패키지 구조

```
src/
├── autonomous_robot_bt/
│   ├── config/
│   │   ├── behavior_trees/
│   │   │   └── pick_and_place.xml
│   │   ├── navigation/
│   │   │   └── nav2_params.yaml
│   │   └── manipulation/
│   │       └── moveit_config.yaml
│   ├── launch/
│   │   ├── autonomous_system.launch.py
│   │   └── bt_executor.launch.py
│   ├── src/
│   │   ├── bt_nodes/
│   │   │   ├── navigation_actions.cpp
│   │   │   ├── manipulation_actions.cpp
│   │   │   └── perception_actions.cpp
│   │   └── bt_executor.cpp
│   └── include/
└── mobile_manipulator_description/
    ├── urdf/
    │   └── mobile_robot_with_arm.urdf.xacro
    └── meshes/
```

## 5. 구현 단계

### Phase 1: 기본 인프라 구축
1. **모바일 베이스 URDF 생성**
   - 자동차 모델 + 로봇 팔 결합
   - tf2 트리 설정

2. **Navigation2 설정**
   - 맵 생성 (SLAM)
   - 경로 계획 설정
   - 장애물 회피 설정

### Phase 2: 행동트리 노드 개발
1. **Navigation Action Nodes**:
   ```cpp
   class NavigateToAction : public BT::AsyncActionNode
   {
   public:
     NavigateToAction(const std::string& name, const BT::NodeConfiguration& config);
     BT::NodeStatus tick() override;
     void halt() override;
   private:
     rclcpp::Node::SharedPtr node_;
     rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_client_;
   };
   ```

2. **Manipulation Action Nodes**:
   ```cpp
   class PickObjectAction : public BT::AsyncActionNode
   {
   public:
     PickObjectAction(const std::string& name, const BT::NodeConfiguration& config);
     BT::NodeStatus tick() override;
   private:
     moveit::planning_interface::MoveGroupInterface move_group_;
   };
   ```

### Phase 3: 통합 및 테스트
1. **시뮬레이션 환경 구축** (Gazebo)
2. **실제 하드웨어 테스트**
3. **성능 최적화**

## 6. 핵심 코드 예시

### BT Executor:
```cpp
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>

class BTExecutor : public rclcpp::Node
{
public:
  BTExecutor() : Node("bt_executor")
  {
    // BT Factory 설정
    factory_.registerNodeType<NavigateToAction>("NavigateToObject");
    factory_.registerNodeType<PickObjectAction>("PickObject");
    
    // XML 파일 로드
    auto tree = factory_.createTreeFromFile("pick_and_place.xml");
    
    // 실행 루프
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this, tree]() { tree.tickRoot(); }
    );
  }

private:
  BT::BehaviorTreeFactory factory_;
  rclcpp::TimerBase::SharedPtr timer_;
};
```

### Launch 파일:
```python
def generate_launch_description():
    return LaunchDescription([
        # Navigation2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('nav2_bringup'),
                '/launch/navigation_launch.py'
            ])
        ),
        
        # MoveIt
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('dual_motor_moveit_config'),
                '/launch/demo.launch.py'
            ])
        ),
        
        # Behavior Tree Executor
        Node(
            package='autonomous_robot_bt',
            executable='bt_executor',
            parameters=[{'bt_xml': 'pick_and_place.xml'}]
        )
    ])
```

## 7. 개발 순서 권장사항

### 1단계: 시뮬레이션 환경
- Gazebo에서 모바일 매니퓰레이터 모델 생성
- Navigation2 + MoveIt 통합 테스트

### 2단계: 기본 행동트리
- 단순한 이동 → 집기 → 놓기 시퀀스
- 각 액션을 개별적으로 테스트

### 3단계: 고급 기능
- 물체 인식 (컴퓨터 비전)
- 동적 경로 계획
- 오류 복구 메커니즘

### 4단계: 실제 하드웨어
- 시뮬레이션에서 검증된 코드를 실제 로봇에 적용
- 센서 데이터 통합
- 안전 기능 추가

## 8. 유용한 도구들

### 디버깅:
- **Groot**: 행동트리 시각화 및 모니터링
- **RViz2**: 로봇 상태 및 경로 시각화
- **rqt_graph**: 노드 연결 상태 확인

### 테스트:
- **Gazebo**: 물리 시뮬레이션
- **Stage**: 2D 네비게이션 테스트
- **MoveIt Planning Scene**: 매니퓰레이션 테스트

## 9. 참고 자료

- [BehaviorTree.CPP Documentation](https://www.behaviortree.dev/)
- [Navigation2 Tutorials](https://navigation.ros.org/)
- [MoveIt2 Tutorials](https://moveit.picknik.ai/humble/)
- [ROS2 Mobile Manipulation](https://github.com/ros-planning/moveit2_tutorials)

이 가이드를 따라 단계별로 구현하시면 원하시는 자율 로봇 시스템을 만들 수 있을 것입니다!