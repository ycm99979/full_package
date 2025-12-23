#!/usr/bin/env python3
"""
Simple Trajectory - 현재 위치에서 목표 관절값으로 trajectory 생성 후 전송
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as DurationMsg

import numpy as np
import sys


class SimpleTrajectory(Node):
    def __init__(self):
        super().__init__('simple_trajectory')

        # Joint names for the arm
        self.joint_names = [
            'link2_to_link1',
            'link3_to_link2',
            'link4_to_link3',
            'gripper_to_link4'
        ]

        # Current joint state
        self.current_positions = [0.0] * 4
        self.joint_state_received = False

        # Joint state subscriber
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Action client
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        self.get_logger().info('Simple Trajectory node initialized')

    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
        self.joint_state_received = True

    def generate_trajectory(self, target_positions, duration_sec=2.0, num_points=10):
        """
        Generate trajectory from current position to target with proper timestamps
        Linear interpolation between current and target positions
        """
        trajectory = JointTrajectory()
        trajectory.header.frame_id = 'base_link'
        trajectory.joint_names = self.joint_names

        # Time step
        dt = duration_sec / num_points

        for i in range(num_points + 1):
            t = i * dt
            alpha = i / num_points  # 0 to 1

            point = JointTrajectoryPoint()

            # Linear interpolation: current + alpha * (target - current)
            point.positions = [
                self.current_positions[j] + alpha * (target_positions[j] - self.current_positions[j])
                for j in range(4)
            ]

            # Simple velocity estimation
            if i == 0 or i == num_points:
                point.velocities = [0.0] * 4
            else:
                point.velocities = [
                    (target_positions[j] - self.current_positions[j]) / duration_sec
                    for j in range(4)
                ]

            # Time from start
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            point.time_from_start = DurationMsg(sec=sec, nanosec=nanosec)

            trajectory.points.append(point)

        return trajectory

    def send_trajectory(self, target_positions, duration_sec=2.0):
        """Send trajectory to controller"""
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Trajectory action server not available')
            return False

        # Generate trajectory
        trajectory = self.generate_trajectory(target_positions, duration_sec)

        self.get_logger().info(f'Generated trajectory with {len(trajectory.points)} points')
        self.get_logger().info(f'Timestamps: {[f"{p.time_from_start.sec}.{p.time_from_start.nanosec:09d}" for p in trajectory.points]}')

        # Send goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info(f'Sending trajectory: {self.current_positions} -> {target_positions}')

        send_goal_future = self.trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server')
            return False

        self.get_logger().info('Goal accepted, executing...')

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_sec + 10.0)

        result = result_future.result()
        if result.result.error_code == 0:  # SUCCESSFUL
            self.get_logger().info('Trajectory executed successfully!')
            return True
        else:
            self.get_logger().error(f'Trajectory failed with error code: {result.result.error_code}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = SimpleTrajectory()

    # Wait for joint state
    node.get_logger().info('Waiting for joint state...')
    timeout = 10.0
    start_time = node.get_clock().now()
    while not node.joint_state_received:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (node.get_clock().now() - start_time).nanoseconds > timeout * 1e9:
            node.get_logger().error('Timeout waiting for joint state')
            node.destroy_node()
            rclpy.shutdown()
            return

    node.get_logger().info(f'Current joint positions: {node.current_positions}')

    # Parse target positions from command line or use defaults
    if len(sys.argv) > 1:
        try:
            target_positions = [float(x) for x in sys.argv[1:5]]
            if len(target_positions) < 4:
                target_positions.extend([0.0] * (4 - len(target_positions)))
        except ValueError:
            target_positions = [0.3, 0.2, 0.1, 0.0]
    else:
        target_positions = [0.3, 0.2, 0.1, 0.0]

    node.get_logger().info(f'Target positions: {target_positions}')

    # Execute trajectory
    success = node.send_trajectory(target_positions, duration_sec=3.0)

    if success:
        node.get_logger().info('Movement complete!')
    else:
        node.get_logger().error('Movement failed!')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
