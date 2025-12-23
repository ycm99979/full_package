#!/usr/bin/env python3
"""
Pick and Place Controller - MoveIt2 with Topic-based Target Position

토픽으로 목표 좌표를 받아서 pick and place를 수행하는 노드

사용법:
    ros2 launch mobile_manipulator_moveit_config demo.launch.py
    ros2 run robot_bringup topic_pick_place.py

    # 목표 위치 발행 (x, y, z in meters)
    ros2 topic pub /target_position geometry_msgs/Point "{x: 0.25, y: 0.0, z: 0.15}" --once
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Point, Pose
from std_msgs.msg import String
from sensor_msgs.msg import JointState

from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, PositionConstraint, 
    BoundingVolume, MoveItErrorCodes, RobotTrajectory
)
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.action import ExecuteTrajectory
from control_msgs.action import GripperCommand, FollowJointTrajectory
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from rclpy.action import ActionClient

import math
import time
import copy


class TopicPickAndPlace(Node):
    def __init__(self):
        super().__init__('topic_pick_place')

        # Declare parameters with default values
        self.declare_parameter('approach_offset', 0.05)
        self.declare_parameter('retreat_offset', 0.1)
        self.declare_parameter('place_offset_x', 0.15)
        self.declare_parameter('gripper_open_position', 0.04)
        self.declare_parameter('gripper_close_position', 0.0)
        self.declare_parameter('gripper_max_effort', 50.0)
        self.declare_parameter('z_min', 0.05)
        self.declare_parameter('z_max', 0.4)
        self.declare_parameter('xy_max', 0.4)

        # Get parameter values
        self.approach_offset = self.get_parameter('approach_offset').value
        self.retreat_offset = self.get_parameter('retreat_offset').value
        self.place_offset_x = self.get_parameter('place_offset_x').value
        self.gripper_open_position = self.get_parameter('gripper_open_position').value
        self.gripper_close_position = self.get_parameter('gripper_close_position').value
        self.gripper_max_effort = self.get_parameter('gripper_max_effort').value
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value
        self.xy_max = self.get_parameter('xy_max').value

        self.callback_group = ReentrantCallbackGroup()
        self._busy = False

        # Current joint states
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )

        # Service client for motion planning
        self.plan_client = self.create_client(
            GetMotionPlan, '/plan_kinematic_path',
            callback_group=self.callback_group
        )

        # Action clients
        self.execute_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory',
            callback_group=self.callback_group
        )

        self.gripper_client = ActionClient(
            self, GripperCommand, '/gripper_controller/gripper_cmd',
            callback_group=self.callback_group
        )

        self.arm_trajectory_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )

        # Wait for services and action servers
        self.get_logger().info('Waiting for services and action servers...')
        self.plan_client.wait_for_service(timeout_sec=10.0)
        self.execute_client.wait_for_server(timeout_sec=10.0)
        self.gripper_client.wait_for_server(timeout_sec=10.0)
        self.arm_trajectory_client.wait_for_server(timeout_sec=10.0)
        self.get_logger().info('Services and action servers ready!')

        # Subscribe to target position topic
        self.target_sub = self.create_subscription(
            Point, '/target_position', self.target_callback, 10,
            callback_group=self.callback_group
        )

        # Status publisher
        self.status_pub = self.create_publisher(String, '/pick_place_status', 10)
        
        # Arm joint names
        self.arm_joints = [
            'link2_to_link1',
            'link3_to_link2', 
            'link4_to_link3',
            'gripper_to_link4'
        ]
        
        # Predefined positions
        self.named_positions = {
            'init': [0.0, 0.0, 0.0, 0.0],
            'home': [0.0, -1.57, 1.57, 1.57]
        }
        
        self.get_logger().info('Topic Pick and Place node ready!')
        self.get_logger().info('Example: ros2 topic pub /target_position geometry_msgs/Point "{x: 0.25, y: 0.0, z: 0.15}" --once')
    
    def joint_state_callback(self, msg: JointState):
        self.current_joint_state = msg
        
    def publish_status(self, message: str):
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
        self.get_logger().info(f'Status: {message}')
    
    def add_time_to_trajectory(self, trajectory: RobotTrajectory, velocity_scale: float = 0.3) -> RobotTrajectory:
        """Add time parameterization to trajectory"""
        result = copy.deepcopy(trajectory)
        
        if len(result.joint_trajectory.points) == 0:
            return result
        
        # Simple time parameterization based on joint distance
        max_joint_velocity = 1.0 * velocity_scale  # rad/s
        
        prev_positions = result.joint_trajectory.points[0].positions
        total_time = 0.0
        
        for i, point in enumerate(result.joint_trajectory.points):
            if i == 0:
                point.time_from_start = Duration(sec=0, nanosec=0)
            else:
                # Calculate max joint displacement
                max_displacement = 0.0
                for j, pos in enumerate(point.positions):
                    displacement = abs(pos - prev_positions[j])
                    max_displacement = max(max_displacement, displacement)
                
                # Calculate time needed
                time_needed = max_displacement / max_joint_velocity
                time_needed = max(time_needed, 0.1)  # Minimum 0.1 seconds
                total_time += time_needed
                
                sec = int(total_time)
                nanosec = int((total_time - sec) * 1e9)
                point.time_from_start = Duration(sec=sec, nanosec=nanosec)
                
                prev_positions = point.positions
        
        return result
    
    def move_to_pose_sync(self, x: float, y: float, z: float, description: str = "") -> bool:
        """Move arm to target pose using plan service + execute action"""
        try:
            self.get_logger().info(f'Moving to {description}: ({x:.3f}, {y:.3f}, {z:.3f})')
            
            # Create motion plan request
            request = GetMotionPlan.Request()
            request.motion_plan_request.group_name = "arm"
            request.motion_plan_request.num_planning_attempts = 10
            request.motion_plan_request.allowed_planning_time = 5.0
            request.motion_plan_request.max_velocity_scaling_factor = 0.3
            request.motion_plan_request.max_acceleration_scaling_factor = 0.3
            
            # Position constraint
            pos_constraint = PositionConstraint()
            pos_constraint.header.frame_id = "base_link"
            pos_constraint.link_name = "gripper_base"
            
            bounding_volume = BoundingVolume()
            sphere = SolidPrimitive()
            sphere.type = SolidPrimitive.SPHERE
            sphere.dimensions = [0.01]
            bounding_volume.primitives.append(sphere)
            
            sphere_pose = Pose()
            sphere_pose.position.x = x
            sphere_pose.position.y = y
            sphere_pose.position.z = z
            sphere_pose.orientation.w = 1.0
            bounding_volume.primitive_poses.append(sphere_pose)
            
            pos_constraint.constraint_region = bounding_volume
            pos_constraint.weight = 1.0
            
            constraints = Constraints()
            constraints.position_constraints.append(pos_constraint)
            request.motion_plan_request.goal_constraints.append(constraints)
            
            # Call planning service
            future = self.plan_client.call_async(request)
            
            timeout = 10.0
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                time.sleep(0.1)
            
            if not future.done():
                self.get_logger().error(f'Planning timeout for {description}')
                return False
            
            response = future.result()
            if response.motion_plan_response.error_code.val != MoveItErrorCodes.SUCCESS:
                self.get_logger().error(f'Planning failed for {description}, error: {response.motion_plan_response.error_code.val}')
                return False
            
            self.get_logger().info(f'Planning succeeded for {description}, executing...')
            
            # Add time parameterization
            trajectory = self.add_time_to_trajectory(response.motion_plan_response.trajectory)
            
            # Execute trajectory
            execute_goal = ExecuteTrajectory.Goal()
            execute_goal.trajectory = trajectory
            
            future = self.execute_client.send_goal_async(execute_goal)
            
            timeout = 5.0
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                time.sleep(0.1)
            
            if not future.done():
                self.get_logger().error(f'Execute goal send timeout for {description}')
                return False
            
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                self.get_logger().error(f'Execute goal rejected for {description}')
                return False
            
            # Wait for result
            result_future = goal_handle.get_result_async()
            timeout = 30.0
            start = time.time()
            while not result_future.done() and (time.time() - start) < timeout:
                time.sleep(0.1)
            
            if not result_future.done():
                self.get_logger().error(f'Execute result timeout for {description}')
                return False
            
            result = result_future.result()
            if result and result.result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.get_logger().info(f'Successfully moved to {description}')
                return True
            else:
                error_code = result.result.error_code.val if result else "unknown"
                self.get_logger().error(f'Execution failed for {description}, error: {error_code}')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Error moving to {description}: {str(e)}')
            return False
    
    def move_to_joint_positions_sync(self, positions: list, duration_sec: float = 3.0) -> bool:
        """Move arm to joint positions directly"""
        try:
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = self.arm_joints
            
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
            goal.trajectory.points.append(point)
            
            future = self.arm_trajectory_client.send_goal_async(goal)
            
            timeout = 5.0
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                time.sleep(0.1)
            
            if not future.done():
                return False
            
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                return False
            
            result_future = goal_handle.get_result_async()
            timeout = duration_sec + 5.0
            start = time.time()
            while not result_future.done() and (time.time() - start) < timeout:
                time.sleep(0.1)
            
            return result_future.done()
            
        except Exception as e:
            self.get_logger().error(f'Joint move error: {str(e)}')
            return False
    
    def move_to_named_sync(self, target_name: str) -> bool:
        """Move to named target"""
        if target_name not in self.named_positions:
            self.get_logger().error(f'Unknown target: {target_name}')
            return False
        
        self.get_logger().info(f'Moving to {target_name}')
        return self.move_to_joint_positions_sync(self.named_positions[target_name])
    
    def control_gripper_sync(self, open: bool) -> bool:
        """Control gripper"""
        try:
            action = "open" if open else "close"
            self.get_logger().info(f'Gripper: {action}')
            
            goal = GripperCommand.Goal()
            goal.command.position = self.gripper_open_position if open else self.gripper_close_position
            goal.command.max_effort = self.gripper_max_effort
            
            future = self.gripper_client.send_goal_async(goal)
            
            timeout = 5.0
            start = time.time()
            while not future.done() and (time.time() - start) < timeout:
                time.sleep(0.1)
            
            if not future.done():
                return False
            
            goal_handle = future.result()
            if not goal_handle or not goal_handle.accepted:
                return False
            
            result_future = goal_handle.get_result_async()
            timeout = 10.0
            start = time.time()
            while not result_future.done() and (time.time() - start) < timeout:
                time.sleep(0.1)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Gripper error: {str(e)}')
            return False
    
    def pick_and_place_sequence(self, pick_x: float, pick_y: float, pick_z: float):
        """Execute pick and place sequence"""
        self.publish_status(f"Starting pick and place at ({pick_x:.3f}, {pick_y:.3f}, {pick_z:.3f})")
        
        # 1. Open gripper
        self.publish_status("Opening gripper")
        self.control_gripper_sync(open=True)
        time.sleep(0.5)
        
        # 2. Approach position
        if not self.move_to_pose_sync(pick_x, pick_y, pick_z + self.approach_offset, "approach"):
            self.publish_status("Failed: approach position")
            return
        
        # 3. Pick position
        if not self.move_to_pose_sync(pick_x, pick_y, pick_z, "pick"):
            self.publish_status("Failed: pick position")
            return
        
        # 4. Close gripper
        self.publish_status("Grasping")
        self.control_gripper_sync(open=False)
        time.sleep(0.5)
        
        # 5. Retreat
        if not self.move_to_pose_sync(pick_x, pick_y, pick_z + self.retreat_offset, "retreat"):
            self.publish_status("Failed: retreat")
            return
        
        # 6. Go to home position
        self.publish_status("Moving to home position")
        if not self.move_to_named_sync("home"):
            self.publish_status("Failed: home position")
            return
        time.sleep(0.5)
        
        # 7. Calculate place position (180 degrees opposite of pick position)
        place_x = -pick_x
        place_y = -pick_y
        
        # 8. Place approach (180 degrees opposite)
        if not self.move_to_pose_sync(place_x, place_y, pick_z + self.approach_offset, "place approach"):
            self.publish_status("Failed: place approach")
            return
        
        # 9. Place position
        if not self.move_to_pose_sync(place_x, place_y, pick_z, "place"):
            self.publish_status("Failed: place position")
            return
        
        # 10. Release
        self.publish_status("Releasing")
        self.control_gripper_sync(open=True)
        time.sleep(0.5)
        
        # 11. Retreat from place
        if not self.move_to_pose_sync(place_x, place_y, pick_z + self.retreat_offset, "place retreat"):
            self.publish_status("Failed: place retreat")
            return
        
        # 12. Return to home
        self.publish_status("Returning to home")
        self.move_to_named_sync("home")
        
        self.publish_status("Pick and place completed!")
    
    def target_callback(self, msg: Point):
        """Callback for target position"""
        if self._busy:
            self.get_logger().warn('Busy, ignoring new target')
            return
            
        self.get_logger().info(f'Received target: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})')
        
        # Validate
        if msg.z < self.z_min or msg.z > self.z_max:
            self.publish_status(f"Invalid z: {msg.z} (must be {self.z_min}-{self.z_max})")
            return

        if abs(msg.x) > self.xy_max or abs(msg.y) > self.xy_max:
            self.publish_status(f"Target out of reach (max: {self.xy_max})")
            return
        
        self._busy = True
        try:
            self.pick_and_place_sequence(msg.x, msg.y, msg.z)
        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')
            self.publish_status(f"Error: {str(e)}")
        finally:
            self._busy = False


def main(args=None):
    rclpy.init(args=args)
    
    node = TopicPickAndPlace()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        node.get_logger().info('Starting Topic Pick and Place node...')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
