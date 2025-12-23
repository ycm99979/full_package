#!/usr/bin/env python3
"""
MoveIt with Timestamps - MoveIt으로 plan 후 timestamp 추가해서 execute
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as DurationMsg

import numpy as np
import sys


class MoveItWithTimestamps(Node):
    def __init__(self):
        super().__init__('moveit_with_timestamps')

        # Joint names
        self.joint_names = [
            'link2_to_link1',
            'link3_to_link2',
            'link4_to_link3',
            'gripper_to_link4'
        ]

        # Current joint state
        self.current_positions = [0.0] * 4
        self.joint_state_received = False

        # Subscriber
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # MoveGroup action client (for planning only)
        self.move_group_client = ActionClient(
            self, MoveGroup, '/move_action'
        )

        # Arm controller action client (for execution)
        self.trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory'
        )

        self.get_logger().info('MoveIt with Timestamps node initialized')

    def joint_state_callback(self, msg):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
        self.joint_state_received = True

    def plan_to_joint_goal(self, target_positions):
        """Use MoveIt to plan to joint goal (plan_only=True)"""
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action server not available')
            return None

        # Build goal
        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0

        # Joint constraints
        constraints = Constraints()
        for i, name in enumerate(self.joint_names):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = target_positions[i]
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints.append(constraints)

        # PLAN ONLY - don't execute
        goal.planning_options.plan_only = True

        self.get_logger().info(f'Planning to joint goal: {target_positions}')

        # Send goal
        future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Planning goal rejected')
            return None

        # Get result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)

        result = result_future.result().result
        if result.error_code.val != 1:  # not SUCCESS
            self.get_logger().error(f'Planning failed with error code: {result.error_code.val}')
            return None

        self.get_logger().info('Planning succeeded!')
        return result.planned_trajectory

    def add_timestamps_to_trajectory(self, planned_trajectory, total_duration=3.0):
        """Add proper timestamps to trajectory points"""
        joint_traj = planned_trajectory.joint_trajectory

        if len(joint_traj.points) == 0:
            self.get_logger().error('Empty trajectory!')
            return None

        # Create new trajectory with timestamps
        new_traj = JointTrajectory()
        new_traj.header = joint_traj.header
        new_traj.joint_names = joint_traj.joint_names

        num_points = len(joint_traj.points)
        dt = total_duration / max(num_points - 1, 1)

        for i, point in enumerate(joint_traj.points):
            new_point = JointTrajectoryPoint()
            new_point.positions = list(point.positions)

            # Add velocities if not present
            if len(point.velocities) == 0:
                if i == 0 or i == num_points - 1:
                    new_point.velocities = [0.0] * len(point.positions)
                else:
                    # Estimate velocity from positions
                    prev_pos = joint_traj.points[i-1].positions
                    next_pos = joint_traj.points[i+1].positions if i+1 < num_points else point.positions
                    new_point.velocities = [
                        (next_pos[j] - prev_pos[j]) / (2 * dt)
                        for j in range(len(point.positions))
                    ]
            else:
                new_point.velocities = list(point.velocities)

            # Set timestamp
            t = i * dt
            new_point.time_from_start = DurationMsg(
                sec=int(t),
                nanosec=int((t - int(t)) * 1e9)
            )

            new_traj.points.append(new_point)

        self.get_logger().info(f'Added timestamps to {num_points} points')
        for p in new_traj.points:
            t = p.time_from_start.sec + p.time_from_start.nanosec / 1e9
            self.get_logger().info(f'  t={t:.3f}s: {[f"{x:.3f}" for x in p.positions]}')

        return new_traj

    def execute_trajectory(self, trajectory, timeout=10.0):
        """Execute trajectory on arm_controller"""
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Trajectory action server not available')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info('Executing trajectory...')

        future = self.trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Trajectory goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)

        result = result_future.result()
        if result.result.error_code == 0:
            self.get_logger().info('Execution successful!')
            return True
        else:
            self.get_logger().error(f'Execution failed: {result.result.error_code}')
            return False

    def move_to(self, target_positions, duration=3.0):
        """Plan with MoveIt, add timestamps, execute"""
        # 1. Plan
        planned = self.plan_to_joint_goal(target_positions)
        if planned is None:
            return False

        # 2. Add timestamps
        traj = self.add_timestamps_to_trajectory(planned, duration)
        if traj is None:
            return False

        # 3. Execute
        return self.execute_trajectory(traj, timeout=duration + 5.0)


def main(args=None):
    rclpy.init(args=args)
    node = MoveItWithTimestamps()

    # Wait for joint state
    node.get_logger().info('Waiting for joint state...')
    timeout = 10.0
    start = node.get_clock().now()
    while not node.joint_state_received:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (node.get_clock().now() - start).nanoseconds > timeout * 1e9:
            node.get_logger().error('Timeout waiting for joint state')
            node.destroy_node()
            rclpy.shutdown()
            return

    node.get_logger().info(f'Current positions: {node.current_positions}')

    # Parse target
    if len(sys.argv) > 1:
        try:
            target = [float(x) for x in sys.argv[1:5]]
            target.extend([0.0] * (4 - len(target)))
        except ValueError:
            target = [0.3, 0.2, 0.1, 0.0]
    else:
        target = [0.3, 0.2, 0.1, 0.0]

    node.get_logger().info(f'Target positions: {target}')

    # Move!
    success = node.move_to(target, duration=3.0)

    if success:
        node.get_logger().info('Done!')
    else:
        node.get_logger().error('Failed!')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
