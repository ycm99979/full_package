#!/usr/bin/env python3
"""
Simple Joint Control - 직접 joint 각도로 제어

MoveIt의 time parameterization 문제를 우회하여
직접 joint trajectory controller를 사용합니다.

Usage:
    python3 simple_joint_control.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState

import time
import math


class SimpleJointControl(Node):
    def __init__(self):
        super().__init__('simple_joint_control')
        
        # Trajectory action client
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory'
        )
        
        # Current joint states
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        # Wait for action server
        self.get_logger().info('Waiting for arm controller...')
        if not self.arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Arm controller not available!')
            return
        self.get_logger().info('Arm controller ready!')
        
        # Arm joint names
        self.arm_joints = [
            'link2_to_link1',
            'link3_to_link2', 
            'link4_to_link3',
            'gripper_to_link4'
        ]
        
        self.get_logger().info('='*50)
        self.get_logger().info('Simple Joint Control Ready!')
        self.get_logger().info('='*50)
        
        # Wait for joint states
        self.wait_for_joint_states()
        
        # Demo movements
        self.run_demo()
    
    def joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg
    
    def wait_for_joint_states(self):
        self.get_logger().info('Waiting for joint states...')
        timeout = 10.0
        start = time.time()
        while self.current_joint_state is None and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.current_joint_state is None:
            self.get_logger().error('No joint states received!')
            return False
        
        self.get_logger().info('Joint states received!')
        return True
    
    def move_to_joints(self, joint_positions: list, duration_sec: float = 3.0) -> bool:
        """Move to specific joint positions"""
        try:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = self.arm_joints
            
            # Add trajectory point with explicit time
            point = JointTrajectoryPoint()
            point.positions = joint_positions
            point.velocities = [0.0] * len(joint_positions)
            point.accelerations = [0.0] * len(joint_positions)
            point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
            
            goal.trajectory.points.append(point)
            
            self.get_logger().info(f'Moving to joints: {[f"{p:.3f}" for p in joint_positions]}')
            
            future = self.arm_client.send_goal_async(goal)
            
            # Wait for goal acceptance
            timeout = 5.0
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                rclpy.spin_once(self, timeout_sec=0.05)
            
            if not future.done():
                self.get_logger().error('Goal send timeout')
                return False
            
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().error('Goal rejected')
                return False
            
            self.get_logger().info('Goal accepted, executing...')
            
            # Wait for result
            result_future = goal_handle.get_result_async()
            timeout = duration_sec + 5.0
            start = time.time()
            while not result_future.done() and (time.time() - start) < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if result_future.done():
                self.get_logger().info('Movement completed!')
                return True
            else:
                self.get_logger().error('Execution timeout')
                return False
            
        except Exception as e:
            self.get_logger().error(f'Movement error: {str(e)}')
            return False
    
    def run_demo(self):
        """Run demo movements"""
        self.get_logger().info('Starting demo movements...')
        
        # Home position
        home_joints = [0.0, 0.0, 0.0, 0.0]
        self.move_to_joints(home_joints, 3.0)
        time.sleep(1.0)
        
        # Position 1: Base rotation
        pos1_joints = [0.5, 0.0, 0.0, 0.0]
        self.move_to_joints(pos1_joints, 2.0)
        time.sleep(1.0)
        
        # Position 2: Shoulder movement
        pos2_joints = [0.5, -0.3, 0.0, 0.0]
        self.move_to_joints(pos2_joints, 2.0)
        time.sleep(1.0)
        
        # Position 3: Elbow movement
        pos3_joints = [0.5, -0.3, 0.5, 0.0]
        self.move_to_joints(pos3_joints, 2.0)
        time.sleep(1.0)
        
        # Position 4: Wrist movement
        pos4_joints = [0.5, -0.3, 0.5, 1.0]
        self.move_to_joints(pos4_joints, 2.0)
        time.sleep(1.0)
        
        # Back to home
        self.move_to_joints(home_joints, 3.0)
        
        self.get_logger().info('Demo completed!')


def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleJointControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()