#!/usr/bin/env python3
"""
MoveIt Executor with Time Parameterization Fix
Fixes the "time between points is not strictly increasing" issue in MoveIt2 Humble
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
    RobotState, WorkspaceParameters
)
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

import time
import math


class MoveItExecutor(Node):
    def __init__(self):
        super().__init__('moveit_executor')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # Current joint states
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        
        # Planning service client
        self.plan_client = self.create_client(
            GetMotionPlan, '/plan_kinematic_path',
            callback_group=self.callback_group
        )
        
        # Trajectory execution client
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        
        # Arm joint names
        self.arm_joints = [
            'link2_to_link1',
            'link3_to_link2',
            'link4_to_link3',
            'gripper_to_link4'
        ]
        
        # Joint limits for time calculation
        self.max_velocity = 2.0  # rad/s
        self.max_acceleration = 5.0  # rad/s^2
        
        self.get_logger().info('Waiting for services and action servers...')
        self.plan_client.wait_for_service(timeout_sec=10.0)
        self.trajectory_client.wait_for_server(timeout_sec=10.0)
        self.get_logger().info('MoveIt Executor ready!')
    
    def joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg
    
    def get_current_positions(self) -> dict:
        """Get current joint positions as dict"""
        if self.current_joint_state is None:
            return {}
        positions = {}
        for i, name in enumerate(self.current_joint_state.name):
            if name in self.arm_joints:
                positions[name] = self.current_joint_state.position[i]
        return positions
    
    def add_time_parameterization(self, trajectory):
        """Add proper time_from_start to trajectory points"""
        if len(trajectory.points) < 2:
            return trajectory
        
        # Get current positions
        current_pos = self.get_current_positions()
        
        # Calculate time for each point based on max joint displacement
        cumulative_time = 0.0
        prev_positions = [current_pos.get(j, 0.0) for j in trajectory.joint_names]
        
        for i, point in enumerate(trajectory.points):
            if i == 0:
                # First point: small time offset
                cumulative_time = 0.1
            else:
                # Calculate max displacement from previous point
                max_displacement = 0.0
                for j, pos in enumerate(point.positions):
                    displacement = abs(pos - prev_positions[j])
                    max_displacement = max(max_displacement, displacement)
                
                # Time based on trapezoidal velocity profile
                # t = sqrt(2 * d / a) for acceleration phase
                # Simplified: use max velocity with safety factor
                segment_time = max(0.5, max_displacement / self.max_velocity * 1.5)
                cumulative_time += segment_time
            
            # Set time_from_start
            sec = int(cumulative_time)
            nanosec = int((cumulative_time - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            
            prev_positions = list(point.positions)
            
            self.get_logger().info(f'Point {i}: time={cumulative_time:.3f}s')
        
        return trajectory
    
    def plan_to_joint_positions(self, target_positions: list) -> bool:
        """Plan and execute to joint positions"""
        if self.current_joint_state is None:
            self.get_logger().error('No joint state received')
            return False
        
        self.get_logger().info(f'Planning to: {[f"{p:.2f}" for p in target_positions]}')
        
        # Create motion plan request
        request = GetMotionPlan.Request()
        mp_request = MotionPlanRequest()
        
        mp_request.group_name = "arm"
        mp_request.num_planning_attempts = 10
        mp_request.allowed_planning_time = 5.0
        mp_request.max_velocity_scaling_factor = 0.5
        mp_request.max_acceleration_scaling_factor = 0.5
        
        # Set start state
        mp_request.start_state.joint_state = self.current_joint_state
        
        # Set goal constraints
        constraints = Constraints()
        for i, joint_name in enumerate(self.arm_joints):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = target_positions[i]
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        mp_request.goal_constraints.append(constraints)
        request.motion_plan_request = mp_request
        
        # Call planning service
        future = self.plan_client.call_async(request)
        
        # Wait for result
        timeout = 10.0
        start = time.time()
        while not future.done() and (time.time() - start) < timeout:
            time.sleep(0.05)
        
        if not future.done():
            self.get_logger().error('Planning timeout')
            return False
        
        response = future.result()
        
        if response.motion_plan_response.error_code.val != 1:  # SUCCESS
            self.get_logger().error(f'Planning failed: {response.motion_plan_response.error_code.val}')
            return False
        
        self.get_logger().info('Planning succeeded, fixing time parameterization...')
        
        # Fix time parameterization
        trajectory = response.motion_plan_response.trajectory.joint_trajectory
        trajectory = self.add_time_parameterization(trajectory)
        
        # Execute trajectory
        return self.execute_trajectory(trajectory)
    
    def execute_trajectory(self, trajectory) -> bool:
        """Execute trajectory with fixed timing"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        
        self.get_logger().info('Executing trajectory...')
        
        future = self.trajectory_client.send_goal_async(goal)
        
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
        
        self.get_logger().info('Goal accepted, waiting for completion...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        timeout = 30.0
        start = time.time()
        while not result_future.done() and (time.time() - start) < timeout:
            time.sleep(0.1)
        
        if result_future.done():
            self.get_logger().info('Trajectory execution completed!')
            return True
        else:
            self.get_logger().error('Execution timeout')
            return False
    
    def move_to_named(self, name: str) -> bool:
        """Move to named position"""
        positions = {
            'home': [0.0, 0.0, 0.0, 0.0],
            'ready': [0.0, -0.785, 0.785, 0.0],
            'front': [0.0, -1.0, 1.0, 0.5],
            'left': [1.0, -1.0, 1.0, 0.5],
            'right': [-1.0, -1.0, 1.0, 0.5],
        }
        
        if name not in positions:
            self.get_logger().error(f'Unknown position: {name}')
            return False
        
        return self.plan_to_joint_positions(positions[name])


def main(args=None):
    rclpy.init(args=args)
    
    node = MoveItExecutor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    # Wait for joint states with spinning
    node.get_logger().info('Waiting for joint states...')
    for i in range(50):  # 5 seconds max
        executor.spin_once(timeout_sec=0.1)
        if node.current_joint_state is not None:
            node.get_logger().info('Joint states received!')
            break
    
    if node.current_joint_state is None:
        node.get_logger().error('Failed to receive joint states after 5 seconds')
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # Demo: move to different positions
    node.get_logger().info('=== Starting Demo ===')
    
    # Spin while executing
    import threading
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    node.move_to_named('ready')
    time.sleep(1.0)
    
    node.move_to_named('front')
    time.sleep(1.0)
    
    node.move_to_named('home')
    
    node.get_logger().info('=== Demo Complete ===')
    node.get_logger().info('Press Ctrl+C to exit')
    
    try:
        spin_thread.join()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
