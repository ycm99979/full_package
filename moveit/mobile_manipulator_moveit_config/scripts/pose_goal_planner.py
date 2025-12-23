#!/usr/bin/env python3
"""
Pose Goal Planner - MoveIt을 사용한 목표 좌표 기반 경로 계획 및 실행

목표 Position을 받으면 MoveIt이 자동으로 경로를 계획하고 실행합니다.
4DOF 팔이므로 Position만 지정하고 Orientation은 자유롭게 둡니다.

Usage:
    # Terminal 1: Launch MoveIt demo
    ros2 launch mobile_manipulator_moveit_config demo.launch.py
    
    # Terminal 2: Run this script
    python3 src/mobile_manipulator_ws/src/mobile_manipulator_moveit_config/scripts/pose_goal_planner.py
    
    # Terminal 3: Send target position (x, y, z)
    ros2 topic pub /target_position geometry_msgs/msg/Point "{x: 0.2, y: 0.0, z: 0.3}" --once
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point, Pose, PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import JointState

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState, PositionIKRequest, MoveItErrorCodes

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from rclpy.action import ActionClient

import time
import math


class PoseGoalPlanner(Node):
    def __init__(self):
        super().__init__('pose_goal_planner')
        
        self.callback_group = ReentrantCallbackGroup()
        self._busy = False
        
        # Parameters
        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('end_effector_link', 'gripper_base')
        self.declare_parameter('base_frame', 'arm_base_link')
        self.declare_parameter('velocity_scaling', 0.3)
        
        self.planning_group = self.get_parameter('planning_group').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.base_frame = self.get_parameter('base_frame').value
        self.velocity_scaling = self.get_parameter('velocity_scaling').value
        
        # Current joint states
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        # IK service client
        self.ik_client = self.create_client(
            GetPositionIK, '/compute_ik',
            callback_group=self.callback_group
        )
        
        # Trajectory action client
        self.arm_trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        
        # Wait for services
        self.get_logger().info('Waiting for IK service...')
        if not self.ik_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('IK service not available!')
            return
        self.get_logger().info('IK service ready!')
        
        self.get_logger().info('Waiting for trajectory action server...')
        if not self.arm_trajectory_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Trajectory action server not available!')
            return
        self.get_logger().info('Trajectory action server ready!')
        
        # Subscribe to target position topic (x, y, z only)
        self.target_position_sub = self.create_subscription(
            Point, '/target_position', self.target_position_callback, 10,
            callback_group=self.callback_group
        )
        
        # Subscribe to target pose topic (full pose)
        self.target_pose_sub = self.create_subscription(
            Pose, '/target_pose', self.target_pose_callback, 10,
            callback_group=self.callback_group
        )
        
        # Status publisher
        self.status_pub = self.create_publisher(String, '/planner_status', 10)
        
        # Arm joint names
        self.arm_joints = [
            'link2_to_link1',
            'link3_to_link2', 
            'link4_to_link3',
            'gripper_to_link4'
        ]
        
        self.get_logger().info('='*50)
        self.get_logger().info('Pose Goal Planner Ready!')
        self.get_logger().info('='*50)
        self.get_logger().info('Subscribe to:')
        self.get_logger().info('  /target_position (geometry_msgs/Point) - position only')
        self.get_logger().info('  /target_pose (geometry_msgs/Pose) - full pose')
        self.get_logger().info('')
        self.get_logger().info('Example commands:')
        self.get_logger().info('  ros2 topic pub /target_position geometry_msgs/msg/Point \\')
        self.get_logger().info('    "{x: 0.2, y: 0.0, z: 0.3}" --once')
        self.get_logger().info('='*50)
    
    def joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg
    
    def publish_status(self, message: str):
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        self.get_logger().info(f'[Status] {message}')
    
    def target_position_callback(self, msg: Point):
        """Callback for target position (x, y, z only)"""
        # Try multiple orientations to find IK solution
        orientations = [
            (0.0, 0.0, 0.0, 1.0),      # Identity
            (0.0, 0.707, 0.0, 0.707),  # Point down (pitch -90)
            (0.5, 0.5, 0.5, 0.5),      # Mixed
            (0.707, 0.0, 0.0, 0.707),  # Roll 90
            (0.0, 0.0, 0.707, 0.707),  # Yaw 90
        ]
        
        for i, (ox, oy, oz, ow) in enumerate(orientations):
            pose = Pose()
            pose.position = msg
            pose.orientation.x = ox
            pose.orientation.y = oy
            pose.orientation.z = oz
            pose.orientation.w = ow
            
            self.get_logger().info(f'Trying orientation {i+1}/{len(orientations)}: [{ox}, {oy}, {oz}, {ow}]')
            if self.plan_and_execute(pose):
                return
        
        self.publish_status('ERROR: No valid IK solution found for any orientation')
    
    def target_pose_callback(self, msg: Pose):
        """Callback for full target pose"""
        self.plan_and_execute(msg)
    
    def plan_and_execute(self, target_pose: Pose) -> bool:
        """Compute IK and execute motion to target pose"""
        if self._busy:
            self.get_logger().warn('Planner is busy, ignoring new target')
            return False
        
        self._busy = True
        
        try:
            pos = target_pose.position
            self.publish_status(f'Target received: pos=[{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}]')
            
            # Wait for current joint state
            if self.current_joint_state is None:
                self.publish_status('Waiting for joint states...')
                timeout = 5.0
                start = time.time()
                while self.current_joint_state is None and (time.time() - start) < timeout:
                    time.sleep(0.1)
                
                if self.current_joint_state is None:
                    self.publish_status('ERROR: No joint states received')
                    return False
            
            # Compute IK
            self.publish_status('Computing IK...')
            joint_positions = self.compute_ik(target_pose)
            
            if joint_positions is None:
                self.publish_status('ERROR: No IK solution found')
                return False
            
            self.publish_status(f'IK solution: {[f"{p:.3f}" for p in joint_positions]}')
            
            # Execute trajectory
            self.publish_status('Executing trajectory...')
            success = self.execute_trajectory(joint_positions)
            
            if success:
                self.publish_status('SUCCESS: Motion completed!')
                return True
            else:
                self.publish_status('ERROR: Trajectory execution failed')
                return False
                
        except Exception as e:
            self.publish_status(f'ERROR: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False
        finally:
            self._busy = False
    
    def compute_ik(self, target_pose: Pose) -> list:
        """Compute inverse kinematics for target pose"""
        request = GetPositionIK.Request()
        
        ik_request = PositionIKRequest()
        ik_request.group_name = self.planning_group
        ik_request.robot_state.joint_state = self.current_joint_state
        ik_request.avoid_collisions = True
        ik_request.timeout.sec = 1
        ik_request.timeout.nanosec = 0
        
        # Set target pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.base_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = target_pose
        
        ik_request.pose_stamped = pose_stamped
        
        request.ik_request = ik_request
        
        # Call IK service
        future = self.ik_client.call_async(request)
        
        # Wait for result
        timeout = 5.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.05)
        
        if not future.done():
            self.get_logger().error('IK service timeout')
            return None
        
        result = future.result()
        
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f'IK failed with error code: {result.error_code.val}')
            return None
        
        # Extract joint positions for arm joints
        joint_positions = []
        for joint_name in self.arm_joints:
            if joint_name in result.solution.joint_state.name:
                idx = result.solution.joint_state.name.index(joint_name)
                joint_positions.append(result.solution.joint_state.position[idx])
            else:
                self.get_logger().error(f'Joint {joint_name} not found in IK solution')
                return None
        
        return joint_positions
    
    def execute_trajectory(self, joint_positions: list, duration_sec: float = 3.0) -> bool:
        """Execute trajectory to joint positions"""
        try:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = self.arm_joints
            
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.velocities = [0.0] * len(joint_positions)
            point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
            goal.trajectory.points.append(point)
            
            future = self.arm_trajectory_client.send_goal_async(goal)
            
            # Wait for goal acceptance
            timeout = 5.0
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                time.sleep(0.05)
            
            if not future.done():
                self.get_logger().error('Goal send timeout')
                return False
            
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().error('Goal rejected')
                return False
            
            # Wait for result
            result_future = goal_handle.get_result_async()
            timeout = duration_sec + 5.0
            start = time.time()
            while not result_future.done() and (time.time() - start) < timeout:
                time.sleep(0.05)
            
            if result_future.done():
                return True
            else:
                self.get_logger().error('Execution timeout')
                return False
            
        except Exception as e:
            self.get_logger().error(f'Trajectory execution error: {str(e)}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    node = PoseGoalPlanner()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
