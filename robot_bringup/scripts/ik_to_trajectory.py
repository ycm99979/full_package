#!/usr/bin/env python3
"""
IK to Trajectory - MoveIt IK 서비스로 관절 위치 계산 후 직접 trajectory controller로 전송
MoveIt의 time parameterization 문제를 우회
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration

from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState, PositionIKRequest
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration as DurationMsg

import numpy as np


class IKToTrajectory(Node):
    def __init__(self):
        super().__init__('ik_to_trajectory')

        self.callback_group = ReentrantCallbackGroup()

        # Joint names for the arm
        self.joint_names = [
            'link2_to_link1',
            'link3_to_link2',
            'link4_to_link3',
            'gripper_to_link4'
        ]

        # Current joint state
        self.current_joint_positions = [0.0] * 4
        self.joint_state_received = False

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Service clients
        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik',
            callback_group=self.callback_group
        )

        # Action client
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )

        self.get_logger().info('IK to Trajectory node initialized')

    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_joint_positions[i] = msg.position[idx]
        self.joint_state_received = True

    def compute_ik(self, target_pose: Pose):
        """Compute IK for target pose"""
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('IK service not available')
            return None

        # Build request
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'arm'
        request.ik_request.robot_state.joint_state.name = self.joint_names
        request.ik_request.robot_state.joint_state.position = self.current_joint_positions
        request.ik_request.avoid_collisions = True
        request.ik_request.timeout = Duration(seconds=1.0).to_msg()

        # Set target pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = 'base_link'
        pose_stamped.pose = target_pose
        request.ik_request.pose_stamped = pose_stamped

        # Call service
        future = self.ik_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            result = future.result()
            if result.error_code.val == 1:  # SUCCESS
                # Extract joint positions for arm
                joint_positions = []
                for name in self.joint_names:
                    if name in result.solution.joint_state.name:
                        idx = result.solution.joint_state.name.index(name)
                        joint_positions.append(result.solution.joint_state.position[idx])
                    else:
                        joint_positions.append(0.0)
                return joint_positions
            else:
                self.get_logger().error(f'IK failed with error code: {result.error_code.val}')
                return None
        return None

    def send_trajectory(self, target_positions, duration_sec=2.0):
        """Send trajectory directly to controller"""
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Trajectory action server not available')
            return False

        # Build trajectory
        trajectory = JointTrajectory()
        trajectory.header.frame_id = 'base_link'
        trajectory.joint_names = self.joint_names

        # Add current position as start point (optional)
        # start_point = JointTrajectoryPoint()
        # start_point.positions = self.current_joint_positions.copy()
        # start_point.velocities = [0.0] * 4
        # start_point.time_from_start = DurationMsg(sec=0, nanosec=100000000)  # 0.1 sec
        # trajectory.points.append(start_point)

        # Add target point
        target_point = JointTrajectoryPoint()
        target_point.positions = target_positions
        target_point.velocities = [0.0] * 4
        target_point.time_from_start = DurationMsg(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        trajectory.points.append(target_point)

        # Send goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info(f'Sending trajectory to positions: {target_positions}')

        future = self.trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_sec + 5.0)

        result = result_future.result()
        if result.result.error_code == 0:  # SUCCESSFUL
            self.get_logger().info('Trajectory executed successfully!')
            return True
        else:
            self.get_logger().error(f'Trajectory failed with error code: {result.result.error_code}')
            return False

    def move_to_joint_positions(self, positions, duration_sec=2.0):
        """Move to specific joint positions"""
        return self.send_trajectory(positions, duration_sec)


def main(args=None):
    rclpy.init(args=args)

    node = IKToTrajectory()

    # Wait for joint state
    node.get_logger().info('Waiting for joint state...')
    while not node.joint_state_received and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

    node.get_logger().info(f'Current joint positions: {node.current_joint_positions}')

    # Example: Move to target positions directly (bypassing IK)
    target_positions = [0.3, 0.2, 0.1, 0.0]  # radians

    node.get_logger().info(f'Moving to target positions: {target_positions}')
    success = node.move_to_joint_positions(target_positions, duration_sec=3.0)

    if success:
        node.get_logger().info('Move completed!')
    else:
        node.get_logger().error('Move failed!')

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
